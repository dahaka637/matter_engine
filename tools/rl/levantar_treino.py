#!/usr/bin/env python3
"""O treino do levantar sem ajuda (DeepMimic; vault
04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md, secao 8).

Fiel ao artigo (secao 5.2 e o material suplementar D): ator e critico 1024-512
ReLU com saida linear; a gaussiana com desvio FIXO (hiperparametro); lote de 4096
amostras (512 mundos x 8 passos), minilotes de 256; gamma 0,95; lambda 0,95 (TD e
GAE); corte 0,2; SGD com momento 0,9, passo 5e-5 no ator e 1e-2 no critico;
normalizacao da entrada.
Declarado (o artigo nao da): o desvio fixo de 0,1 rad; uma passada pelo lote por
atualizacao (16 minilotes); o fim do clipe com o valor do estado final (bootstrap).

  .venv/bin/python levantar_treino.py --treino lev_frente --clipe frente
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from rastreador_treino import DATA, REPO, RunningNorm, native  # noqa: E402

SIGMA = 0.1


def mlp(inputs: int, outputs: int) -> nn.Sequential:
    return nn.Sequential(nn.Linear(inputs, 1024), nn.ReLU(), nn.Linear(1024, 512), nn.ReLU(), nn.Linear(512, outputs))


class GetUpPolicy(nn.Module):
    def __init__(self, obs: int, actions: int):
        super().__init__()
        self.norm = RunningNorm(obs)
        self.actor = mlp(obs, actions)
        self.critic = mlp(obs, 1)

    def distribution(self, obs_n: torch.Tensor) -> torch.distributions.Normal:
        mean = self.actor(obs_n)
        return torch.distributions.Normal(mean, torch.full_like(mean, SIGMA))

    def value(self, obs_n: torch.Tensor) -> torch.Tensor:
        return self.critic(obs_n).squeeze(-1)


def export(model: GetUpPolicy, path: Path, meta: dict) -> None:
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
    layers = [{"inputs": m.in_features, "outputs": m.out_features, "weights": push(m.weight), "bias": push(m.bias)}
              for m in model.actor if isinstance(m, nn.Linear)]
    header = dict(meta)
    header.update({"schema": "matter-rl-policy-1", "observationSize": layers[0]["inputs"],
                   "actionSize": layers[-1]["outputs"], "activation": "relu",
                   "normalizer": {"mean": mean_at, "invStd": inv_at, "clip": 5.0}, "actionLimit": 3.14159,
                   "layers": layers, "observationLayout": "levantar-v1", "actionLayout": "levantar-v1",
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", required=True)
    parser.add_argument("--clipe", default="frente", choices=["frente", "costas"])
    parser.add_argument("--mundos", type=int, default=512)
    parser.add_argument("--passos", type=int, default=8)
    parser.add_argument("--threads", type=int, default=12)
    parser.add_argument("--semente", type=int, default=1)
    parser.add_argument("--atualizacoes", type=int, default=0, help="0: ate parar")
    parser.add_argument("--salvar-cada", type=int, default=100)
    args = parser.parse_args()

    run = DATA / args.treino
    run.mkdir(parents=True, exist_ok=True)
    for name, data in (("run.json", {"kind": "levantar", "task": f"levantar_{args.clipe}"}),
                       ("tarefa.json", {"schema": "matter-getup-1", "id": f"levantar_{args.clipe}"})):
        if not (run / name).exists():
            (run / name).write_text(json.dumps(data, ensure_ascii=False))
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(args.semente)
    batch = native.GetUpBatch(str(REPO / "assets"), clip=args.clipe, environments=args.mundos, threads=args.threads,
                              seed=args.semente)
    n, obs_size, act_size = batch.num_envs, batch.obs_size, batch.action_size
    model = GetUpPolicy(obs_size, act_size).to(device)
    actor_opt = torch.optim.SGD(model.actor.parameters(), lr=5e-5, momentum=0.9)
    critic_opt = torch.optim.SGD(model.critic.parameters(), lr=1e-2, momentum=0.9)
    counters = {"atualizacao": 0, "amostras": 0, "episodios": 0}
    settings = {"clipe": args.clipe, "mundos": n, "passos": args.passos, "sigma": SIGMA,
                "observacao": batch.observation_names(), "acao": batch.action_names()}
    checkpoint = run / "ultimo.pt"
    if checkpoint.exists():
        state = torch.load(checkpoint, map_location=device, weights_only=False)
        model.load_state_dict(state["model"])
        actor_opt.load_state_dict(state["actor_opt"])
        critic_opt.load_state_dict(state["critic_opt"])
        counters = state["counters"]
    log = open(run / "log.jsonl", "a")
    gamma, lam, clip, minibatch, T = 0.95, 0.95, 0.2, 256, args.passos
    obs = torch.as_tensor(batch.observations(), device=device)
    window = []
    while args.atualizacoes <= 0 or counters["atualizacao"] < args.atualizacoes:
        begin = time.time()
        buf = {k: torch.zeros(T, n, *s, device=device) for k, s in
               (("obs", (obs_size,)), ("act", (act_size,)), ("logp", ()), ("val", ()), ("rew", ()), ("done", ()))}
        with torch.no_grad():
            for t in range(T):
                model.norm.update(obs)
                obs_n = model.norm(obs)
                dist = model.distribution(obs_n)
                action = dist.sample()
                buf["obs"][t], buf["act"][t] = obs, action
                buf["logp"][t] = dist.log_prob(action).sum(-1)
                buf["val"][t] = model.value(obs_n)
                rew, term, trunc = batch.step(action.cpu().numpy().astype(np.float32))
                rew = torch.as_tensor(rew, device=device)
                trunc = torch.as_tensor(trunc, device=device).float()
                if trunc.any():
                    final = torch.as_tensor(batch.final_observations(), device=device)
                    rew = rew + gamma * trunc * model.value(model.norm(final))
                buf["rew"][t] = rew
                buf["done"][t] = torch.maximum(torch.as_tensor(term, device=device).float(), trunc)
                obs = torch.as_tensor(batch.observations(), device=device)
            last_value = model.value(model.norm(obs))
        adv = torch.zeros(T, n, device=device)
        gae = torch.zeros(n, device=device)
        for t in reversed(range(T)):
            next_value = last_value if t == T - 1 else buf["val"][t + 1]
            nonterminal = 1.0 - buf["done"][t]
            delta = buf["rew"][t] + gamma * next_value * nonterminal - buf["val"][t]
            gae = delta + gamma * lam * nonterminal * gae
            adv[t] = gae
        ret = adv + buf["val"]
        flat = lambda x: x.reshape(T * n, *x.shape[2:])  # noqa: E731
        f_obs, f_act, f_logp, f_adv, f_ret = flat(buf["obs"]), flat(buf["act"]), flat(buf["logp"]), flat(adv), flat(ret)
        f_adv = (f_adv - f_adv.mean()) / (f_adv.std() + 1e-8)
        perm = torch.randperm(T * n, device=device)
        stats = {"pol": 0.0, "vf": 0.0, "clip": 0.0}
        steps = 0
        for start in range(0, T * n, minibatch):
            idx = perm[start:start + minibatch]
            obs_n = model.norm(f_obs[idx])
            vf_loss = (model.value(obs_n) - f_ret[idx]).pow(2).mean()
            critic_opt.zero_grad()
            vf_loss.backward()
            critic_opt.step()
            logp = model.distribution(obs_n).log_prob(f_act[idx]).sum(-1)
            ratio = torch.exp(logp - f_logp[idx])
            pol_loss = -torch.min(ratio * f_adv[idx], ratio.clamp(1 - clip, 1 + clip) * f_adv[idx]).mean()
            actor_opt.zero_grad()
            pol_loss.backward()
            actor_opt.step()
            stats["pol"] += pol_loss.item()
            stats["vf"] += vf_loss.item()
            stats["clip"] += ((ratio - 1).abs() > clip).float().mean().item()
            steps += 1
        episodes = batch.finished_episodes()
        if len(episodes):
            window.extend(episodes.tolist())
            window = window[-2000:]
        counters["atualizacao"] += 1
        counters["amostras"] += T * n
        counters["episodios"] += len(episodes)
        if counters["atualizacao"] % 20 == 0:
            recent = np.array(window) if window else np.zeros((0, 7))
            row = {"atualizacao": counters["atualizacao"], "amostras": counters["amostras"],
                   "nota_passo": float(buf["rew"].mean()), "am_s": T * n / (time.time() - begin),
                   **{k: v / max(1, steps) for k, v in stats.items()}}
            if len(recent):
                row.update({"de_pe": float(recent[:, 6].mean()), "cabeca": float(recent[:, 4].mean()),
                            "de_pe_do_inicio": float(recent[recent[:, 1] < 0.2, 6].mean())
                            if (recent[:, 1] < 0.2).any() else None})
            log.write(json.dumps(row) + "\n")
            log.flush()
            print(" ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in row.items()), flush=True)
        if counters["atualizacao"] % args.salvar_cada == 0:
            tmp = run / "ultimo.tmp"
            torch.save({"model": model.state_dict(), "actor_opt": actor_opt.state_dict(),
                        "critic_opt": critic_opt.state_dict(), "counters": counters, "settings": settings}, tmp)
            tmp.replace(checkpoint)
            export(model, run / "exports" / "policy_latest.mrlpolicy",
                   {"run": args.treino, "iteration": counters["atualizacao"], "steps": counters["amostras"],
                    "observationNames": settings["observacao"]})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
