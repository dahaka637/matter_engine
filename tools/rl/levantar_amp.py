#!/usr/bin/env python3
"""O treino do levantar livre (31/10; vault 04_locomocao/plano_compostos_31_10.md, "O levantar livre"): uma rede so
para levantar de qualquer queda (de brucos, de costas, de lado), pelo AMP (Peng et al. 2021, "AMP: Adversarial Motion
Priors for Stylized Physics-Based Character Control"): PPO com a nota da tarefa (ficar de pe, do lote) mais a do
estilo, de um discriminador que separa as transicoes do corpo das dos clipes (os de levantar de brucos e de costas -
com o rolamento -, os espelhados e o parado). Nada de seguir o clipe quadro a quadro.

Do artigo (secao 6 e o material suplementar): discriminador 1024-512 ReLU por minimos quadrados (alvo +1 nos clipes,
-1 no corpo), penalidade do gradiente nos clipes, nota do estilo max(0, 1 - 0,25 (d - 1)^2), nota total 0,5 tarefa +
0,5 estilo, uma reserva de transicoes recentes do corpo para o discriminador; ator e critico 1024-512.
Declarado: gamma 0,99 (o levantar leva segundos), lambda 0,95; a acao normalizada na faixa de cada junta (o meio
mais a metade vezes a saida, como no ASE), o desvio dela aprendido (comeca em 0,1, preso entre 0,03 e 0,5); Adam (ator
5e-5, critico 2e-4, discriminador 1e-4); penalidade do gradiente 5 e regularizacao da ultima camada 0,01 (os do ASE).
Num treino novo, a rede comeca imitando a de brucos do DeepMimic (levantar_frente, 100% de brucos; warm_start) e as
primeiras 20 atualizacoes so ensinam o critico e o discriminador.

A nota por passo entra vezes 1 - gamma (o retorno entre 0 e 1).
A medida honesta (o log, linha "aval", e o laboratorio): a rede media em 64 quedas fixas, sem os comecos nos clipes -
de pe no fim, se ficou de pe alguma vez e quando, por lado.

  .venv/bin/python levantar_amp.py --treino levantar_amp --threads 3
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from rastreador_treino import DATA, REPO, RunningNorm, native  # noqa: E402

LOG_STD_MIN, LOG_STD_MAX = math.log(0.03), math.log(0.5)
SIDES = ("brucos", "costas", "lado")


def mlp(inputs: int, outputs: int) -> nn.Sequential:
    return nn.Sequential(nn.Linear(inputs, 1024), nn.ReLU(), nn.Linear(1024, 512), nn.ReLU(), nn.Linear(512, outputs))


class Policy(nn.Module):
    """A acao da rede e normalizada na faixa de cada junta (como no AMP/ASE): o alvo e o meio da faixa mais a metade
    dela vezes a saida - a saida zero e o meio do movimento (nao a pose em T) e o ruido e proporcional a cada junta."""

    def __init__(self, obs: int, actions: int, ranges: np.ndarray):
        super().__init__()
        self.norm = RunningNorm(obs)
        self.actor = mlp(obs, actions)
        self.critic = mlp(obs, 1)
        self.log_std = nn.Parameter(torch.full((actions,), math.log(0.1)))
        low, high = torch.as_tensor(ranges[:, 0]), torch.as_tensor(ranges[:, 1])
        self.register_buffer("action_mid", 0.5 * (low + high))
        self.register_buffer("action_half", torch.clamp(0.5 * (high - low), min=0.05))

    def env_action(self, a: torch.Tensor) -> torch.Tensor:
        return self.action_mid + self.action_half * a

    def distribution(self, obs_n: torch.Tensor) -> torch.distributions.Normal:
        mean = self.actor(obs_n)
        std = self.log_std.clamp(LOG_STD_MIN, LOG_STD_MAX).exp().expand_as(mean)
        return torch.distributions.Normal(mean, std)

    def value(self, obs_n: torch.Tensor) -> torch.Tensor:
        return self.critic(obs_n).squeeze(-1)


class Discriminator(nn.Module):
    """D(s, s'): as duas metades normalizadas pela mesma media (as caracteristicas de um instante)."""

    def __init__(self, size: int):
        super().__init__()
        self.size = size
        self.norm = RunningNorm(size)
        self.net = mlp(2 * size, 1)

    def inputs(self, pairs: torch.Tensor) -> torch.Tensor:
        return torch.cat([self.norm(pairs[:, :self.size]), self.norm(pairs[:, self.size:])], dim=1)

    def forward(self, pairs: torch.Tensor) -> torch.Tensor:
        return self.net(self.inputs(pairs)).squeeze(-1)


def export(model: Policy, path: Path, meta: dict) -> None:
    import struct
    from datetime import datetime, timezone
    floats, offset = [], 0

    def push(array: torch.Tensor) -> int:
        nonlocal offset
        data = array.detach().to("cpu", torch.float32).contiguous().numpy().reshape(-1)
        floats.append(data)
        start = offset
        offset += data.size
        return start

    mean_at = push(model.norm.mean.float())
    inv_at = push(1.0 / torch.sqrt(model.norm.var.float() + 1e-8))
    # (A ultima camada ja sai em radianos: o meio e a metade da faixa de cada junta embutidos nela.)
    linears = [m for m in model.actor if isinstance(m, nn.Linear)]
    layers = []
    for k, m in enumerate(linears):
        weight, bias = m.weight, m.bias
        if k == len(linears) - 1:
            weight = model.action_half[:, None] * weight
            bias = model.action_mid + model.action_half * bias
        layers.append({"inputs": m.in_features, "outputs": m.out_features, "weights": push(weight), "bias": push(bias)})
    header = dict(meta)
    header.update({"schema": "matter-rl-policy-1", "observationSize": layers[0]["inputs"],
                   "actionSize": layers[-1]["outputs"], "activation": "relu",
                   "normalizer": {"mean": mean_at, "invStd": inv_at, "clip": 5.0}, "actionLimit": 3.14159,
                   "layers": layers, "observationLayout": "levantar-livre-v1", "actionLayout": "levantar-v1",
                   "createdUtc": datetime.now(timezone.utc).isoformat(timespec="seconds")})
    text = json.dumps(header, ensure_ascii=False).encode("utf-8")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "wb") as handle:
        handle.write(b"MRLPOL01")
        handle.write(struct.pack("<I", len(text)))
        handle.write(text)
        handle.write(b"\0" * ((-(12 + len(text))) % 4))
        for data in floats:
            handle.write(data.astype("<f4").tobytes())
    tmp.replace(path)


def summarize(rows: np.ndarray, prefix: str) -> dict:
    """As medidas de um conjunto de episodios (as colunas de finished_episodes)."""
    out: dict = {}
    if not len(rows):
        return out
    stood = rows[:, 5] >= 0
    out[f"{prefix}de_pe"] = float(rows[:, 9].mean())
    out[f"{prefix}levantou"] = float(stood.mean())
    out[f"{prefix}tempo"] = float(rows[stood, 5].mean()) if stood.any() else None
    for k, side in enumerate(SIDES):
        sel = rows[:, 2] == k
        out[f"{prefix}{side}"] = float(rows[sel, 9].mean()) if sel.any() else None
        out[f"{prefix}n_{side}"] = int(sel.sum())
    return out


def evaluate(model: Policy, batch, device: torch.device) -> dict:
    """A rede media nas 64 quedas fixas (a semente nao muda): um episodio de cada mundo."""
    batch.reset(12345)
    n = batch.num_envs
    done: dict[int, list] = {}
    for _ in range(int(batch.episode_seconds / batch.policy_dt) + 2):
        with torch.no_grad():
            obs = torch.as_tensor(batch.observations(), device=device)
            action = model.env_action(model.actor(model.norm(obs))).cpu().numpy().astype(np.float32)
        batch.step(action)
        for row in batch.finished_episodes().tolist():
            done.setdefault(int(row[0]), row)
        if len(done) >= n:
            break
    return summarize(np.array(list(done.values())), "")


def warm_start(model: Policy, threads: int, steps: int, device: torch.device) -> dict:
    """O comeco pela rede de brucos do DeepMimic (levantar_frente, que levanta 100% de brucos): a rede nova (sem a fase
    do clipe) aprende a acao dela nos estados dela, com o ruido do DART (Laskey et al. 2017: o professor age com um pouco
    de ruido, a aluna ve estados um pouco fora do caminho dele). O treino parte sabendo levantar de brucos."""
    from levantar_treino import GetUpPolicy as Teacher
    state = torch.load(DATA / "levantar_frente" / "ultimo.pt", map_location=device, weights_only=False)
    source = native.GetUpBatch(str(REPO / "assets"), clip="frente", environments=256, threads=threads, seed=7)
    teacher = Teacher(source.obs_size, source.action_size).to(device)
    teacher.load_state_dict(state["model"])
    teacher.eval()
    rows_x, rows_y = [], []
    with torch.no_grad():
        for _ in range(steps):
            obs = torch.as_tensor(source.observations(), device=device)
            act = teacher.actor(teacher.norm(obs))
            rows_x.append(obs[:, :-1].clone())
            rows_y.append((act - model.action_mid) / model.action_half)
            source.step((act + 0.05 * torch.randn_like(act)).cpu().numpy().astype(np.float32))
            source.finished_episodes()
    x, y = torch.cat(rows_x), torch.cat(rows_y)
    for chunk in x.split(16384):
        model.norm.update(chunk)
    optimizer = torch.optim.Adam(model.actor.parameters(), lr=3e-4)
    loss = torch.zeros(())
    for _ in range(30):
        for idx in torch.randperm(len(x), device=device).split(4096):
            loss = (model.actor(model.norm(x[idx])) - y[idx]).pow(2).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
    # A conferencia: a aluna no lote do professor, comecando deitada de brucos (o inicio do clipe).
    check = native.GetUpBatch(str(REPO / "assets"), clip="frente", environments=64, threads=threads, seed=9,
                              evaluation=True)
    ended: dict[int, list] = {}
    for _ in range(int(check.clip_seconds / check.policy_dt) + 2):
        with torch.no_grad():
            obs = torch.as_tensor(check.observations(), device=device)[:, :-1]
            action = model.env_action(model.actor(model.norm(obs))).cpu().numpy().astype(np.float32)
        check.step(action)
        for row in check.finished_episodes().tolist():
            ended.setdefault(int(row[0]), row)
    standing = float(np.mean([row[6] for row in ended.values()])) if ended else 0.0
    return {"aprendeu": True, "amostras_professor": int(len(x)), "perda": float(loss.item()), "de_pe_brucos": standing}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", required=True)
    parser.add_argument("--mundos", type=int, default=512)
    parser.add_argument("--passos", type=int, default=32)
    parser.add_argument("--threads", type=int, default=3)
    parser.add_argument("--semente", type=int, default=1)
    parser.add_argument("--atualizacoes", type=int, default=0, help="0: ate parar")
    parser.add_argument("--salvar-cada", type=int, default=50)
    parser.add_argument("--epocas", type=int, default=4)
    parser.add_argument("--minilotes", type=int, default=4)
    parser.add_argument("--taxa-ator", type=float, default=5e-5)
    parser.add_argument("--taxa-critico", type=float, default=2e-4)
    parser.add_argument("--taxa-disc", type=float, default=1e-4)
    parser.add_argument("--peso-tarefa", type=float, default=0.5)
    parser.add_argument("--peso-estilo", type=float, default=0.5)
    parser.add_argument("--comeco-clipe", type=float, default=0.25, help="a fatia dos comecos nos clipes de levantar")
    parser.add_argument("--professor-passos", type=int, default=800,
                        help="num treino novo: os passos da rede de brucos (256 mundos cada) para a aluna aprender")
    parser.add_argument("--aquecer-critico", type=int, default=20,
                        help="as primeiras atualizacoes so do critico e do discriminador (a acao aprendida fica)")
    args = parser.parse_args()

    from matter_rl.system import require_watchdog
    require_watchdog()
    torch.set_num_threads(1)

    stop = {"pedido": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(pedido=True))

    run = DATA / args.treino
    run.mkdir(parents=True, exist_ok=True)
    for name, data in (("run.json", {"kind": "levantar", "task": "levantar_livre"}),
                       ("tarefa.json", {"schema": "matter-getup-amp-1", "id": "levantar_livre"})):
        if not (run / name).exists():
            (run / name).write_text(json.dumps(data, ensure_ascii=False))
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(args.semente)
    batch = native.GetUpAmpBatch(str(REPO / "assets"), environments=args.mundos, threads=args.threads,
                                 seed=args.semente, clip_start_fraction=args.comeco_clipe)
    evaluation = native.GetUpAmpBatch(str(REPO / "assets"), environments=64, threads=1, seed=12345, evaluation=True)
    n, obs_size, act_size, amp = batch.num_envs, batch.obs_size, batch.action_size, batch.amp_size
    reference = torch.as_tensor(batch.reference_pairs(), device=device)
    model = Policy(obs_size, act_size, batch.action_ranges()).to(device)
    disc = Discriminator(amp).to(device)
    actor_opt = torch.optim.Adam(list(model.actor.parameters()) + [model.log_std], lr=args.taxa_ator)
    critic_opt = torch.optim.Adam(model.critic.parameters(), lr=args.taxa_critico)
    disc_opt = torch.optim.Adam(disc.net.parameters(), lr=args.taxa_disc, weight_decay=1e-4)
    counters = {"atualizacao": 0, "amostras": 0, "episodios": 0}
    settings = {"mundos": n, "passos": args.passos, "peso_tarefa": args.peso_tarefa, "peso_estilo": args.peso_estilo,
                "comeco_clipe": args.comeco_clipe, "observacao": batch.observation_names(), "acao": batch.action_names(),
                "pares_dos_clipes": int(reference.shape[0])}
    checkpoint = run / "ultimo.pt"
    if checkpoint.exists():
        state = torch.load(checkpoint, map_location=device, weights_only=False)
        model.load_state_dict(state["model"])
        disc.load_state_dict(state["disc"])
        actor_opt.load_state_dict(state["actor_opt"])
        critic_opt.load_state_dict(state["critic_opt"])
        disc_opt.load_state_dict(state["disc_opt"])
        counters = state["counters"]
    log = open(run / "log.jsonl", "a")
    gamma, lam, clip, T = 0.99, 0.95, 0.2, args.passos
    minibatch = (T * n) // max(1, args.minilotes)
    replay = torch.zeros(0, 2 * amp, device=device)
    obs = torch.as_tensor(batch.observations(), device=device)
    window: list = []
    with torch.no_grad():
        disc.norm.update(reference[:, :amp])

    def export_policy() -> None:
        export(model, run / "exports" / "policy_latest.mrlpolicy",
               {"run": args.treino, "iteration": counters["atualizacao"], "steps": counters["amostras"],
                "observationNames": settings["observacao"]})

    def save() -> None:
        tmp = run / "ultimo.tmp"
        torch.save({"model": model.state_dict(), "disc": disc.state_dict(), "actor_opt": actor_opt.state_dict(),
                    "critic_opt": critic_opt.state_dict(), "disc_opt": disc_opt.state_dict(), "counters": counters,
                    "settings": settings}, tmp)
        tmp.replace(checkpoint)
        export_policy()

    if not checkpoint.exists() and args.professor_passos > 0:
        row = {"atualizacao": 0, **warm_start(model, args.threads, args.professor_passos, device)}
        log.write(json.dumps(row) + "\n")
        log.flush()
        print("aprendeu com a rede de brucos: " + " ".join(f"{k}={v}" for k, v in row.items()), flush=True)
        export_policy()

    while (args.atualizacoes <= 0 or counters["atualizacao"] < args.atualizacoes) and not stop["pedido"]:
        begin = time.time()
        buf = {k: torch.zeros(T, n, *s, device=device) for k, s in
               (("obs", (obs_size,)), ("act", (act_size,)), ("logp", ()), ("val", ()), ("task", ()), ("boot", ()),
                ("done", ()), ("pairs", (2 * amp,)))}
        with torch.no_grad():
            for t in range(T):
                model.norm.update(obs)
                obs_n = model.norm(obs)
                dist = model.distribution(obs_n)
                action = dist.sample()
                buf["obs"][t], buf["act"][t] = obs, action
                buf["logp"][t] = dist.log_prob(action).sum(-1)
                buf["val"][t] = model.value(obs_n)
                task, term, trunc = batch.step(model.env_action(action).cpu().numpy().astype(np.float32))
                buf["task"][t] = torch.as_tensor(task, device=device)
                buf["pairs"][t] = torch.as_tensor(batch.amp_pairs(), device=device)
                trunc = torch.as_tensor(trunc, device=device).float()
                if trunc.any():
                    final = torch.as_tensor(batch.final_observations(), device=device)
                    buf["boot"][t] = trunc * model.value(model.norm(final))
                buf["done"][t] = torch.maximum(torch.as_tensor(term, device=device).float(), trunc)
                obs = torch.as_tensor(batch.observations(), device=device)
            last_value = model.value(model.norm(obs))
            pairs = buf["pairs"].reshape(T * n, 2 * amp)
            disc.norm.update(torch.cat([pairs[:, :amp], reference[:, :amp]]))
            d = disc(pairs).reshape(T, n)
            style = torch.clamp(1.0 - 0.25 * (d - 1.0).pow(2), min=0.0)
            # (A nota por passo vezes 1 - gamma: o retorno fica entre 0 e 1 e o critico nao precisa correr atras da
            # escala de ~100 de um gamma 0,99.)
            rew = (1.0 - gamma) * (args.peso_tarefa * buf["task"] + args.peso_estilo * style) + gamma * buf["boot"]
        adv = torch.zeros(T, n, device=device)
        gae = torch.zeros(n, device=device)
        for t in reversed(range(T)):
            next_value = last_value if t == T - 1 else buf["val"][t + 1]
            nonterminal = 1.0 - buf["done"][t]
            delta = rew[t] + gamma * next_value * nonterminal - buf["val"][t]
            gae = delta + gamma * lam * nonterminal * gae
            adv[t] = gae
        ret = adv + buf["val"]
        flat = lambda x: x.reshape(T * n, *x.shape[2:])  # noqa: E731
        f_obs, f_act, f_logp, f_adv, f_ret = flat(buf["obs"]), flat(buf["act"]), flat(buf["logp"]), flat(adv), flat(ret)
        f_adv = (f_adv - f_adv.mean()) / (f_adv.std() + 1e-8)
        replay = torch.cat([replay, pairs])[-100_000:]
        stats = {"pol": 0.0, "vf": 0.0, "clip": 0.0, "disc": 0.0, "d_clipe": 0.0, "d_corpo": 0.0, "gp": 0.0}
        steps = 0
        for _ in range(args.epocas):
            for idx in torch.randperm(T * n, device=device).split(minibatch):
                obs_n = model.norm(f_obs[idx])
                vf_loss = (model.value(obs_n) - f_ret[idx]).pow(2).mean()
                critic_opt.zero_grad()
                vf_loss.backward()
                nn.utils.clip_grad_norm_(model.critic.parameters(), 1.0)
                critic_opt.step()
                logp = model.distribution(obs_n).log_prob(f_act[idx]).sum(-1)
                ratio = torch.exp(logp - f_logp[idx])
                pol_loss = -torch.min(ratio * f_adv[idx], ratio.clamp(1 - clip, 1 + clip) * f_adv[idx]).mean()
                if counters["atualizacao"] >= args.aquecer_critico:
                    actor_opt.zero_grad()
                    pol_loss.backward()
                    nn.utils.clip_grad_norm_(list(model.actor.parameters()) + [model.log_std], 1.0)
                    actor_opt.step()
                # O discriminador: metade do lote de agora e metade da reserva, contra os clipes (com reposicao).
                half = len(idx) // 2
                body = torch.cat([pairs[idx[:half]], replay[torch.randint(len(replay), (half,), device=device)]])
                clips = reference[torch.randint(len(reference), (len(body),), device=device)]
                x_clip = disc.inputs(clips).detach().requires_grad_(True)
                d_clip = disc.net(x_clip).squeeze(-1)
                d_body = disc.net(disc.inputs(body)).squeeze(-1)
                grad = torch.autograd.grad(d_clip.sum(), x_clip, create_graph=True)[0]
                gp = grad.pow(2).sum(-1).mean()
                last = disc.net[-1].weight
                disc_loss = 0.5 * ((d_clip - 1.0).pow(2).mean() + (d_body + 1.0).pow(2).mean()) + 5.0 * gp \
                    + 0.01 * last.pow(2).sum()
                disc_opt.zero_grad()
                disc_loss.backward()
                disc_opt.step()
                stats["pol"] += pol_loss.item()
                stats["vf"] += vf_loss.item()
                stats["clip"] += ((ratio - 1).abs() > clip).float().mean().item()
                stats["disc"] += disc_loss.item()
                stats["d_clipe"] += d_clip.mean().item()
                stats["d_corpo"] += d_body.mean().item()
                stats["gp"] += gp.item()
                steps += 1
        episodes = batch.finished_episodes()
        if len(episodes):
            window.extend(episodes.tolist())
            window = window[-3000:]
        counters["atualizacao"] += 1
        counters["amostras"] += T * n
        counters["episodios"] += len(episodes)
        if counters["atualizacao"] % 20 == 0:
            recent = np.array(window) if window else np.zeros((0, 10))
            row = {"atualizacao": counters["atualizacao"], "amostras": counters["amostras"],
                   "tarefa": float(buf["task"].mean()), "estilo": float(style.mean()),
                   "desvio": float(model.log_std.detach().clamp(LOG_STD_MIN, LOG_STD_MAX).exp().mean()),
                   "am_s": T * n / (time.time() - begin), **{k: v / max(1, steps) for k, v in stats.items()}}
            row.update(summarize(recent[recent[:, 1] == 0] if len(recent) else recent, "queda_"))
            row.update(summarize(recent[recent[:, 1] == 1] if len(recent) else recent, "clipe_"))
            log.write(json.dumps(row) + "\n")
            log.flush()
            print(" ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in row.items()), flush=True)
        if counters["atualizacao"] % 10 == 0:
            export_policy()
        if counters["atualizacao"] % args.salvar_cada == 0:
            save()
            row = {"atualizacao": counters["atualizacao"], "amostras": counters["amostras"], "aval": True,
                   **evaluate(model, evaluation, device)}
            log.write(json.dumps(row) + "\n")
            log.flush()
            print("aval " + " ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}" for k, v in row.items()),
                  flush=True)
    save()
    print(f"salvo na atualizacao {counters['atualizacao']}" + (" (sinal de parar)" if stop["pedido"] else ""), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
