#!/usr/bin/env python3
"""O portao do rastreador (vault .../pesquisas/12_especificacao_rastreador.md, secao 9):
o ultimo checkpoint de um treino, com a acao media (sigma = 0), de frente na
velocidade dada, 64 mundos, o primeiro episodio de cada um (20 s no maximo).

G1 passa com >= 90% dos episodios sem queda (a queda de verdade: a cabeca abaixo
de 1,1 m; o afastamento de pe do lider aparece a parte) e erro da velocidade do
centro de massa <= 0,3 m/s (limiares nossos, declarados antes).

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
    parser.add_argument("--so-queda-real", action="store_true", help="falhar so na queda de verdade (como o treino com --so-queda-real)")
    parser.add_argument("--zero", action="store_true", help="sem rede (a malha aberta, a linha de base)")
    parser.add_argument("--modo", default="frente", choices=["frente", "rumos", "trocas", "parado", "giro", "giro_rapido",
                                                           "estrela", "estrela_fixo", "estrela_girando"],
                        help="frente (G1); rumos: os 8 rumos em relacao ao olhar; trocas: o protocolo do DReCon (7.1); "
                             "parado, giro, giro_rapido: os exercicios parados (o G2 exige os tres); "
                             "estrela, estrela_fixo, estrela_girando: a estrela de 8 pontas na velocidade --velocidade")
    parser.add_argument("--duracao", type=float, default=0.0, help="segundos por episodio (padrao: 20; trocas: 60)")
    parser.add_argument("--cubo-kg", type=float, default=0.0, help="com os projeteis do DReCon nessa massa (a Fig. 11)")
    args = parser.parse_args()
    run = DATA / args.treino
    duration = args.duracao or (60.0 if args.modo == "trocas" else 20.0)
    batch = native.TrackingBatch(str(REPO / "assets"), environments=args.mundos, threads=args.threads, seed=20261018,
                                 scenario="frente", evaluation=True, evaluation_speed=args.velocidade,
                                 episode_seconds=duration, evaluation_mode=args.modo,
                                 real_fall_only=args.so_queda_real, projectiles=args.cubo_kg > 0.0, projectile_minimum_kg=max(args.cubo_kg, 0.01),
                                 projectile_maximum_kg=max(args.cubo_kg, 0.01))
    model = Policy(batch.obs_size, batch.action_size)
    rodada = 0
    if not args.zero:
        state = torch.load(run / args.checkpoint, map_location="cpu", weights_only=False)
        model.load_state_dict(state["model"])
        rodada = state["counters"]["rodada"]
    model.eval()
    first: dict[int, np.ndarray] = {}
    with torch.no_grad():
        for _ in range(int((duration + 1.0) / 0.0333) + 2):
            obs = torch.as_tensor(batch.observations())
            action = np.zeros((batch.num_envs, batch.action_size), np.float32) if args.zero \
                else model.actor(model.norm(obs)).numpy().astype(np.float32)
            batch.step(action)
            for row in batch.finished_episodes():
                first.setdefault(int(row[0]), row)
            if len(first) >= batch.num_envs:
                break
    rows = np.array(list(first.values()))
    # O criterio declarado e a queda (a auditoria do Codex, 18/10): a queda de
    # verdade (a cabeca abaixo de 1,1 m); o afastamento de pe fica a parte.
    falls = float(rows[:, 8].mean()) if rows.shape[1] >= 10 else float(rows[:, 2].mean())
    error = float(rows[:, 4].mean())
    passed = falls <= 0.10 and error <= 0.30
    if args.modo in ("parado", "giro", "giro_rapido"):
        # Parado (e girando parado): todos chegam ao fim, sem cair nem se afastar do lider.
        passed = float(rows[:, 2].mean()) == 0.0
    result = {"treino": args.treino, "rodada": rodada, "modo": args.modo, "duracao": duration, "cubo_kg": args.cubo_kg,
              "velocidade": args.velocidade, "episodios": len(rows),
              "quedas": falls, "fins_antes_do_tempo": float(rows[:, 2].mean()), "segundos_medio": float(rows[:, 1].mean()), "erro_vel_cm": error,
              "vel_corpo": float(rows[:, 5].mean()), "vel_pedida": float(rows[:, 6].mean()),
              "dist_cabeca": float(rows[:, 7].mean()), "passa_g1": passed, "passa": passed, "hora": time.time()}
    if rows.shape[1] >= 10:
        # Dos fins antes dos 20 s: queda de verdade (a cabeca abaixo de 1,1 m) ou o
        # corpo de pe que se afastou do lider.
        result["quedas_de_verdade"] = float(rows[:, 8].mean())
        result["afastou_de_pe"] = float((rows[:, 2] - rows[:, 8]).mean())
    if args.modo == "rumos":
        # Por rumo em relacao ao olhar (o mundo i no rumo i mod 8).
        names = ["frente", "diag_esq", "esquerda", "tras_esq", "costas", "tras_dir", "direita", "diag_dir"]
        result["por_rumo"] = {names[k]: {"quedas": float(rows[rows[:, 0] % 8 == k, 2].mean()),
                                        "quedas_de_verdade": float(rows[rows[:, 0] % 8 == k, 8].mean()),
                                        "erro_vel_cm": float(rows[rows[:, 0] % 8 == k, 4].mean())}
                              for k in range(8) if (rows[:, 0] % 8 == k).any()}
    print(json.dumps(result, ensure_ascii=False))
    if not args.zero:
        with open(run / "portao.jsonl", "a") as f:
            f.write(json.dumps(result) + "\n")
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
