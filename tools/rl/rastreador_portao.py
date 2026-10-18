#!/usr/bin/env python3
"""O portao do rastreador (vault .../pesquisas/12_especificacao_rastreador.md, secao 9):
o ultimo checkpoint de um treino, com a acao media (sigma = 0), de frente na
velocidade dada, 64 mundos, o primeiro episodio de cada um (20 s no maximo).

G1 passa com >= 90% dos episodios sem queda e erro da velocidade do centro de
massa <= 0,3 m/s (limiares nossos, declarados antes).

  .venv/bin/python rastreador_portao.py --treino rast_g1 [--velocidade 3] [--mundos 64]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from rastreador_treino import DATA, REPO, Policy, native  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", required=True)
    parser.add_argument("--checkpoint", default="ultimo.pt")
    parser.add_argument("--velocidade", type=float, default=3.0)
    parser.add_argument("--mundos", type=int, default=64)
    parser.add_argument("--threads", type=int, default=6)
    parser.add_argument("--zero", action="store_true", help="sem rede (a malha aberta, a linha de base)")
    args = parser.parse_args()
    run = DATA / args.treino
    batch = native.TrackingBatch(str(REPO / "assets"), environments=args.mundos, threads=args.threads, seed=20261018,
                                 scenario="frente", evaluation=True, evaluation_speed=args.velocidade)
    model = Policy(batch.obs_size, batch.action_size)
    rodada = 0
    if not args.zero:
        state = torch.load(run / args.checkpoint, map_location="cpu", weights_only=False)
        model.load_state_dict(state["model"])
        rodada = state["counters"]["rodada"]
    model.eval()
    first: dict[int, np.ndarray] = {}
    with torch.no_grad():
        for _ in range(int(21.0 / 0.0333) + 2):
            obs = torch.as_tensor(batch.observations())
            action = np.zeros((batch.num_envs, batch.action_size), np.float32) if args.zero \
                else model.actor(model.norm(obs)).numpy().astype(np.float32)
            batch.step(action)
            for row in batch.finished_episodes():
                first.setdefault(int(row[0]), row)
            if len(first) >= batch.num_envs:
                break
    rows = np.array(list(first.values()))
    falls = float(rows[:, 2].mean())
    error = float(rows[:, 4].mean())
    passed = falls <= 0.10 and error <= 0.30
    result = {"treino": args.treino, "rodada": rodada, "velocidade": args.velocidade, "episodios": len(rows),
              "quedas": falls, "segundos_medio": float(rows[:, 1].mean()), "erro_vel_cm": error,
              "vel_corpo": float(rows[:, 5].mean()), "vel_pedida": float(rows[:, 6].mean()),
              "dist_cabeca": float(rows[:, 7].mean()), "passa_g1": passed, "hora": time.time()}
    if rows.shape[1] >= 10:
        # Dos fins antes dos 20 s: queda de verdade (a cabeca abaixo de 1,1 m) ou o
        # corpo de pe que se afastou do lider.
        result["quedas_de_verdade"] = float(rows[:, 8].mean())
        result["afastou_de_pe"] = float((rows[:, 2] - rows[:, 8]).mean())
    print(json.dumps(result, ensure_ascii=False))
    if not args.zero:
        with open(run / "portao.jsonl", "a") as f:
            f.write(json.dumps(result) + "\n")
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
