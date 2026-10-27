#!/usr/bin/env python3
"""O treino do rastreador fisico (DReCon; vault
04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md, secao 6).

Fiel ao artigo: rede 2x128 tanh (ator e critico), ultima camada do ator em
U(-0,01, 0,01), ~40 mil amostras por rodada (512 mundos x 80 passos), gamma 0,99.
O PPO e o do OpenAI Baselines (ppo1, o que o DReCon usou) nos valores dele que o
artigo nao da: lambda 0,95, corte 0,2, 10 epocas, Adam 3e-4, desvio inicial 1
(log 0) aprendido, normalizacao da entrada (media e desvio corridos, corte +-5).
Declarado: minilotes de 4096 (o ppo1 usa 64; o mesmo gradiente esperado, bem
mais rapido); o fim pelo tempo com o valor do estado final (bootstrap); e a
normalizacao da entrada atualizada a cada passo da coleta, antes de a rede ver a
observacao (o ppo1 a atualiza com o lote inteiro antes da otimizacao).

  .venv/bin/python rastreador_treino.py --treino rast_g1 [--cenario frente] [--rodadas 0]
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import os
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, os.environ.get("MATTER_TRACKING_NATIVE", str(REPO / "build-rl" / "python")))
import matter_tracking_native as native  # noqa: E402

DATA = Path.home() / ".local" / "share" / "matter-engine" / "rl" / "runs"


class RunningNorm(nn.Module):
    """Media e desvio corridos (como o RunningMeanStd do Baselines), corte em +-5."""

    def __init__(self, size: int):
        super().__init__()
        self.register_buffer("mean", torch.zeros(size, dtype=torch.float64))
        self.register_buffer("var", torch.ones(size, dtype=torch.float64))
        self.register_buffer("count", torch.tensor(1e-4, dtype=torch.float64))

    def update(self, x: torch.Tensor) -> None:
        x = x.double()
        batch_mean, batch_var, n = x.mean(0), x.var(0, unbiased=False), x.shape[0]
        delta = batch_mean - self.mean
        total = self.count + n
        self.mean += delta * n / total
        self.var = (self.var * self.count + batch_var * n + delta.pow(2) * self.count * n / total) / total
        self.count = total

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return ((x - self.mean.float()) / torch.sqrt(self.var.float() + 1e-8)).clamp(-5.0, 5.0)


def mlp(inputs: int, outputs: int) -> nn.Sequential:
    return nn.Sequential(nn.Linear(inputs, 128), nn.Tanh(), nn.Linear(128, 128), nn.Tanh(), nn.Linear(128, outputs))


class Policy(nn.Module):
    def __init__(self, obs: int, actions: int):
        super().__init__()
        self.norm = RunningNorm(obs)
        self.actor = mlp(obs, actions)
        self.critic = mlp(obs, 1)
        last = self.actor[-1]
        nn.init.uniform_(last.weight, -0.01, 0.01)  # DReCon 6.6
        nn.init.uniform_(last.bias, -0.01, 0.01)
        self.log_std = nn.Parameter(torch.zeros(actions))  # o Baselines: desvio inicial 1

    def distribution(self, obs_n: torch.Tensor) -> torch.distributions.Normal:
        return torch.distributions.Normal(self.actor(obs_n), self.log_std.exp().expand(obs_n.shape[0], -1))

    def value(self, obs_n: torch.Tensor) -> torch.Tensor:
        return self.critic(obs_n).squeeze(-1)


def export_policy(model: Policy, path: Path, meta: dict) -> None:
    """O .mrlpolicy que o motor le (PolicyRuntime3D: o ator, a normalizacao, tanh; a acao media)."""
    floats: list[np.ndarray] = []
    offset = 0

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
                   "actionSize": layers[-1]["outputs"], "activation": "tanh",
                   "normalizer": {"mean": mean_at, "invStd": inv_at, "clip": 5.0}, "actionLimit": 3.14159,
                   "layers": layers, "observationLayout": "rastreador-v1", "actionLayout": "rastreador-v1",
                   "createdUtc": datetime.now(timezone.utc).isoformat(timespec="seconds")})
    text = json.dumps(header, ensure_ascii=False).encode("utf-8")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with open(temporary, "wb") as handle:
        handle.write(b"MRLPOL01")
        handle.write(struct.pack("<I", len(text)))
        handle.write(text)
        handle.write(b"\0" * ((-(12 + len(text))) % 4))
        for data in floats:
            handle.write(data.astype("<f4").tobytes())
    temporary.replace(path)


def write_run_files(run: Path, cenario: str) -> None:
    """O treino como a tela de Aprendizado o reconhece: run.json (kind rastreador) e tarefa.json."""
    for name, data in (("run.json", {"kind": "rastreador", "task": f"rastreador_{cenario}"}),
                       ("tarefa.json", {"schema": "matter-tracking-1", "id": f"rastreador_{cenario}",
                                        "displayName": f"Rastreador fisico (DReCon), {cenario}"})):
        if not (run / name).exists():
            (run / name).write_text(json.dumps(data, ensure_ascii=False, indent=1))
    # O comando deste treinador: o "Continuar treino" da tela o reabre igual.
    config = json.loads((run / "run.json").read_text())
    config.update({"comando": [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]], "pasta": str(HERE)})
    (run / "run.json").write_text(json.dumps(config, ensure_ascii=False, indent=1))


def write_status(run: Path, state: str, counters: dict, row: dict, args) -> None:
    """O estado do treinador para a tela (o status.json do supervisor; o pid diz que ele esta vivo)."""
    status = {"run": args.treino, "task": f"rastreador_{args.cenario}",
              "taskName": f"Rastreador, {args.cenario}" + (f" (de {args.de})" if args.de else ""),
              "state": state, "pid": os.getpid(), "hora": time.time(), "iteration": counters["rodada"],
              "steps": counters["decisoes"], "episodes": counters["episodios"],
              "trainSeconds": counters.get("segundos", 0.0), "sps": row.get("dec_s", 0.0), "threads": args.threads,
              "environments": args.mundos, "device": "cuda" if torch.cuda.is_available() else "cpu",
              "last": {"reward": row.get("nota_passo", 0.0), "episode_seconds": row.get("ep_segundos", 0.0),
                       "fall_fraction": row.get("quedas", -1.0)}}
    tmp = run / "status.tmp"
    tmp.write_text(json.dumps(status, ensure_ascii=False))
    tmp.replace(run / "status.json")


def save(path: Path, model: Policy, optimizer, counters: dict, settings: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    torch.save({"model": model.state_dict(), "optimizer": optimizer.state_dict(), "counters": counters,
                "settings": settings}, tmp)
    tmp.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", required=True)
    parser.add_argument("--cenario", default="frente", choices=["frente", "completo"])
    parser.add_argument("--teto", type=float, default=3.0, help="o teto da velocidade no cenario frente")
    parser.add_argument("--mundos", type=int, default=512)
    parser.add_argument("--passos", type=int, default=80)
    parser.add_argument("--threads", type=int, default=12)
    parser.add_argument("--semente", type=int, default=1)
    parser.add_argument("--rodadas", type=int, default=0, help="0: ate parar")
    parser.add_argument("--salvar-cada", type=int, default=10)
    parser.add_argument("--sem-corrida", action="store_true", help="o jogo inteiro so com o trote (o G2)")
    parser.add_argument("--projeteis", action="store_true", help="os cubos do DReCon 7.1 (o G3)")
    parser.add_argument("--de", default="", help="comeca do ultimo checkpoint de outro treino (o degrau anterior)")
    parser.add_argument("--parado", type=float, default=0.0, help="parte dos episodios parado no idle")
    parser.add_argument("--giro", type=float, default=0.0, help="parte dos episodios parado com giro suave")
    parser.add_argument("--giro-rapido", type=float, default=0.0, help="parte dos episodios com giro rapido parado")
    parser.add_argument("--estrela", type=float, default=0.0, help="parte dos episodios na estrela de 8 pontas, peito livre")
    parser.add_argument("--estrela-fixo", type=float, default=0.0, help="parte na estrela com o peito fixo (lado, costas, diagonais)")
    parser.add_argument("--estrela-girando", type=float, default=0.0, help="parte na estrela com o peito girando (a camera)")
    parser.add_argument("--estrela-velocidade", type=float, default=3.0, help="o trote da estrela (m/s; hoje 3, meta do jogo 4,5)")
    parser.add_argument("--estrela-raio", type=float, default=10.0, help="o raio da estrela (m)")
    parser.add_argument("--so-queda-real", action="store_true", help="o episodio so acaba na queda de verdade (cabeca abaixo de 1,1 m), nao pela distancia ao lider")
    parser.add_argument("--suavidade", type=float, default=0.0,
                        help="custo de suavidade: a nota vezes exp(-k media((a - a_anterior)^2)); 0 = o DReCon")
    parser.add_argument("--peso-pernas", type=float, default=1.0, help="peso das pernas e pes na pose, sempre (1 = DReCon)")
    parser.add_argument("--peso-velocidade", type=float, default=1.0, help="peso da velocidade do centro de massa (1 = DReCon)")
    parser.add_argument("--peso-bracos", type=float, default=1.0, help="peso dos bracos na pose (1 = DReCon)")
    parser.add_argument("--peso-tronco", type=float, default=1.0, help="peso do tronco (pelve, abdomen, peito) na pose (1 = DReCon)")
    parser.add_argument("--premio-de-pe", type=float, default=0.0, help="premio fixo por decisao de pe (0 = DReCon)")
    parser.add_argument("--escala-velocidade", type=float, default=1.0, help="escala dos pedidos de velocidade no treino")
    parser.add_argument("--pernas-adaptaveis", action="store_true",
                        help="em perigo (a cabeca se afastando do lider ou um tropeco) as pernas pesam 0,3 na nota")
    args = parser.parse_args()

    run = DATA / args.treino
    run.mkdir(parents=True, exist_ok=True)
    write_run_files(run, args.cenario)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(args.semente)
    batch = native.TrackingBatch(str(REPO / "assets"), environments=args.mundos, threads=args.threads,
                                 seed=args.semente, scenario=args.cenario, forward_maximum_speed=args.teto,
                                 allow_sprint=not args.sem_corrida, projectiles=args.projeteis,
                                 idle_share=args.parado, turn_share=args.giro, fast_turn_share=args.giro_rapido,
                                 action_rate_weight=args.suavidade, adaptive_legs=args.pernas_adaptaveis,
                                 leg_weight=args.peso_pernas, center_of_mass_velocity_weight=args.peso_velocidade,
                                 arm_weight=args.peso_bracos, trunk_weight=args.peso_tronco,
                                 alive_bonus=args.premio_de_pe, speed_scale=args.escala_velocidade,
                                 star_share=args.estrela, star_fixed_share=args.estrela_fixo,
                                 star_turning_share=args.estrela_girando, star_speed=args.estrela_velocidade,
                                 star_radius=args.estrela_raio, real_fall_only=args.so_queda_real)
    n, obs_size, act_size = batch.num_envs, batch.obs_size, batch.action_size
    model = Policy(obs_size, act_size).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=3e-4)
    counters = {"rodada": 0, "decisoes": 0, "episodios": 0}
    settings = {"cenario": args.cenario, "teto": args.teto, "corrida": not args.sem_corrida, "projeteis": args.projeteis,
                "mundos": n,
                "passos": args.passos,
                "observacao": batch.observation_names(), "acao": batch.action_names()}
    checkpoint = run / "ultimo.pt"
    if not checkpoint.exists() and args.de:
        # O degrau seguinte comeca do cerebro do anterior (a rede, a normalizacao e o otimizador).
        source = DATA / args.de / "ultimo.pt"
        checkpoint.write_bytes(source.read_bytes())
        print(f"comecando de {args.de}", flush=True)
    if checkpoint.exists():
        state = torch.load(checkpoint, map_location=device, weights_only=False)
        model.load_state_dict(state["model"])
        optimizer.load_state_dict(state["optimizer"])
        counters = state["counters"]
        print(f"retomando {args.treino} da rodada {counters['rodada']}", flush=True)
    log = open(run / "log.jsonl", "a")
    # Parar (SIGTERM, o "Parar" da tela) fecha a rodada, salva e sai; SIGUSR1 salva na proxima rodada.
    requests = {"parar": False, "salvar": False}
    signal.signal(signal.SIGTERM, lambda *_: requests.__setitem__("parar", True))
    signal.signal(signal.SIGUSR1, lambda *_: requests.__setitem__("salvar", True))
    counters.setdefault("segundos", 0.0)
    write_status(run, "treinando", counters, {}, args)

    gamma, lam, clip, epochs, minibatch = 0.99, 0.95, 0.2, 10, 4096
    T = args.passos
    obs = torch.as_tensor(batch.observations(), device=device)
    while args.rodadas <= 0 or counters["rodada"] < args.rodadas:
        begin = time.time()
        buf_obs = torch.zeros(T, n, obs_size, device=device)
        buf_act = torch.zeros(T, n, act_size, device=device)
        buf_logp = torch.zeros(T, n, device=device)
        buf_val = torch.zeros(T, n, device=device)
        buf_rew = torch.zeros(T, n, device=device)
        buf_done = torch.zeros(T, n, device=device)
        sim_seconds = 0.0
        with torch.no_grad():
            for t in range(T):
                model.norm.update(obs)
                obs_n = model.norm(obs)
                dist = model.distribution(obs_n)
                action = dist.sample()
                buf_obs[t], buf_act[t] = obs, action
                buf_logp[t] = dist.log_prob(action).sum(-1)
                buf_val[t] = model.value(obs_n)
                s0 = time.time()
                rew, term, trunc = batch.step(action.cpu().numpy().astype(np.float32))
                sim_seconds += time.time() - s0
                rew = torch.as_tensor(rew, device=device)
                term = torch.as_tensor(term, device=device).float()
                trunc = torch.as_tensor(trunc, device=device).float()
                if trunc.any():
                    # O fim pelo tempo: soma o valor do estado final (bootstrap).
                    final = torch.as_tensor(batch.final_observations(), device=device)
                    rew = rew + gamma * trunc * model.value(model.norm(final))
                buf_rew[t] = rew
                buf_done[t] = torch.maximum(term, trunc)
                obs = torch.as_tensor(batch.observations(), device=device)
            last_value = model.value(model.norm(obs))
        # GAE(lambda).
        adv = torch.zeros(T, n, device=device)
        gae = torch.zeros(n, device=device)
        for t in reversed(range(T)):
            next_value = last_value if t == T - 1 else buf_val[t + 1]
            nonterminal = 1.0 - buf_done[t]
            delta = buf_rew[t] + gamma * next_value * nonterminal - buf_val[t]
            gae = delta + gamma * lam * nonterminal * gae
            adv[t] = gae
        ret = adv + buf_val
        flat = lambda x: x.reshape(T * n, *x.shape[2:])  # noqa: E731
        f_obs, f_act, f_logp, f_adv, f_ret = flat(buf_obs), flat(buf_act), flat(buf_logp), flat(adv), flat(ret)
        f_adv = (f_adv - f_adv.mean()) / (f_adv.std() + 1e-8)
        stats = {"kl": 0.0, "clip": 0.0, "pol": 0.0, "vf": 0.0}
        updates = 0
        for _ in range(epochs):
            perm = torch.randperm(T * n, device=device)
            for start in range(0, T * n, minibatch):
                idx = perm[start:start + minibatch]
                obs_n = model.norm(f_obs[idx])
                dist = model.distribution(obs_n)
                logp = dist.log_prob(f_act[idx]).sum(-1)
                ratio = torch.exp(logp - f_logp[idx])
                surr = torch.min(ratio * f_adv[idx], ratio.clamp(1 - clip, 1 + clip) * f_adv[idx])
                pol_loss = -surr.mean()
                vf_loss = (model.value(obs_n) - f_ret[idx]).pow(2).mean()
                loss = pol_loss + vf_loss
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                with torch.no_grad():
                    stats["kl"] += (f_logp[idx] - logp).mean().item()
                    stats["clip"] += ((ratio - 1).abs() > clip).float().mean().item()
                    stats["pol"] += pol_loss.item()
                    stats["vf"] += vf_loss.item()
                updates += 1
        for k in stats:
            stats[k] /= max(1, updates)
        episodes = batch.finished_episodes()
        counters["rodada"] += 1
        counters["decisoes"] += T * n
        counters["episodios"] += len(episodes)
        elapsed = time.time() - begin
        row = {"rodada": counters["rodada"], "decisoes": counters["decisoes"], "hora": time.time(),
               "nota_passo": float(buf_rew.mean()), "desvio": float(model.log_std.exp().mean()),
               "episodios": int(len(episodes)), "dec_s": T * n / elapsed, "sim_s": sim_seconds, "total_s": elapsed, **stats}
        if len(episodes):
            row.update({"ep_segundos": float(episodes[:, 1].mean()), "quedas": float(episodes[:, 2].mean()),
                        "erro_vel_cm": float(episodes[:, 4].mean()), "vel_corpo": float(episodes[:, 5].mean()),
                        "vel_pedida": float(episodes[:, 6].mean())})
            # Por direcao (8 rumos de 45 graus, 0 frente, 2 esquerda): os graficos da tela seguem o rumo mostrado.
            # Os exercicios parados (coluna 11: 1 parado, 2 giro, 3 giro rapido, 4 a 6 as estrelas) tem a chave propria.
            bins = np.round(np.mod(episodes[:, 10], 2 * math.pi) / (math.pi / 4)).astype(int) % 8
            keys = np.array([str(b) for b in bins], dtype=object)
            for code, name in ((1, "parado"), (2, "giro"), (3, "giro_rapido"), (4, "estrela"),
                              (5, "estrela_fixo"), (6, "estrela_girando")):
                keys[episodes[:, 11] == code] = name
            row["rumos"] = {k: {"n": int((keys == k).sum()), "sem_cair": float(1.0 - episodes[keys == k, 2].mean()),
                                "nota": float(episodes[keys == k, 3].mean()),
                                "ep_segundos": float(episodes[keys == k, 1].mean()),
                                "erro_vel_cm": float(episodes[keys == k, 4].mean()),
                                "vel_corpo": float(episodes[keys == k, 5].mean()),
                                "vel_pedida": float(episodes[keys == k, 6].mean())}
                            for k in sorted(set(keys))}
        log.write(json.dumps(row) + "\n")
        log.flush()
        print(" ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in row.items()
                       if k not in ("hora", "rumos")),
              flush=True)
        counters["segundos"] += elapsed
        write_status(run, "treinando", counters, row, args)
        if counters["rodada"] % args.salvar_cada == 0 or requests["parar"] or requests["salvar"]:
            requests["salvar"] = False
            save(checkpoint, model, optimizer, counters, settings)
            save(run / f"r{counters['rodada']:06d}.pt", model, optimizer, counters, settings)
            export_policy(model, run / "exports" / "policy_latest.mrlpolicy",
                          {"run": args.treino, "iteration": counters["rodada"], "steps": counters["decisoes"],
                           "observationNames": settings["observacao"]})
            (run / "latest.json").write_text(json.dumps({"iteration": counters["rodada"]}))
        if requests["parar"]:
            print(f"parado na rodada {counters['rodada']} (pedido de parar)", flush=True)
            break
    save(checkpoint, model, optimizer, counters, settings)
    write_status(run, "parado", counters, {}, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
