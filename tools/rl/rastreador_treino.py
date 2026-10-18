#!/usr/bin/env python3
"""O treino do rastreador fisico (DReCon; vault
04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md, secao 6).

Fiel ao artigo: rede 2x128 tanh (ator e critico), ultima camada do ator em
U(-0,01, 0,01), ~40 mil amostras por rodada (512 mundos x 80 passos), gamma 0,99.
O PPO e o do OpenAI Baselines (ppo1, o que o DReCon usou) nos valores dele que o
artigo nao da: lambda 0,95, corte 0,2, 10 epocas, Adam 3e-4, desvio inicial 1
(log 0) aprendido, normalizacao da entrada (media e desvio corridos, corte +-5).
Declarado: minilotes de 4096 (o ppo1 usa 64; o mesmo gradiente esperado, bem
mais rapido) e o fim pelo tempo com o valor do estado final (bootstrap).

  .venv/bin/python rastreador_treino.py --treino rast_g1 [--cenario frente] [--rodadas 0]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "build-rl" / "python"))
import matter_tracking_native as native  # noqa: E402

DATA = Path.home() / ".local" / "share" / "matter-engine" / "rl" / "rastreador"


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
    args = parser.parse_args()

    run = DATA / args.treino
    run.mkdir(parents=True, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(args.semente)
    batch = native.TrackingBatch(str(REPO / "assets"), environments=args.mundos, threads=args.threads,
                                 seed=args.semente, scenario=args.cenario, forward_maximum_speed=args.teto)
    n, obs_size, act_size = batch.num_envs, batch.obs_size, batch.action_size
    model = Policy(obs_size, act_size).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=3e-4)
    counters = {"rodada": 0, "decisoes": 0, "episodios": 0}
    settings = {"cenario": args.cenario, "teto": args.teto, "mundos": n, "passos": args.passos,
                "observacao": batch.observation_names(), "acao": batch.action_names()}
    checkpoint = run / "ultimo.pt"
    if checkpoint.exists():
        state = torch.load(checkpoint, map_location=device, weights_only=False)
        model.load_state_dict(state["model"])
        optimizer.load_state_dict(state["optimizer"])
        counters = state["counters"]
        print(f"retomando {args.treino} da rodada {counters['rodada']}", flush=True)
    log = open(run / "log.jsonl", "a")

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
        log.write(json.dumps(row) + "\n")
        log.flush()
        print(" ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in row.items() if k != "hora"),
              flush=True)
        if counters["rodada"] % args.salvar_cada == 0:
            save(checkpoint, model, optimizer, counters, settings)
            save(run / f"r{counters['rodada']:06d}.pt", model, optimizer, counters, settings)
    save(checkpoint, model, optimizer, counters, settings)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
