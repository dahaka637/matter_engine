#!/usr/bin/env python3
"""Cada exercicio de uma tarefa com a politica exportada de um treino: 64 mundos,
o primeiro episodio de cada um, com quedas, erro de velocidade, pontos por
minuto, deriva na espera, afastamento da curva, base, pes cruzados, erro do
peito, pes parados, pivo, postura, escorregar e a inclinacao do peito e da pelve
(graus para a frente; negativa e para tras) com a fatia do peito para tras, e a
linha do tronco (da pelve ate a cabeca) com a fatia dela para tras.

  .venv/bin/python exercicios.py RUN TAREFA [SEGUNDOS=30] [SOLVER=20 (jogo) | -1 (treino)] [SEMENTE]
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build-rl" / "python"))
import matter_rl_native as native  # noqa: E402

DATA = Path(os.environ.get("MATTER_RL_DATA", Path.home() / ".local" / "share" / "matter-engine" / "rl"))


def mean_valid(rows: np.ndarray, column: int) -> float:
    values = rows[:, column]
    values = values[values >= 0.0]
    return float(values.mean()) if len(values) else float("nan")


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    run, task = sys.argv[1], sys.argv[2]
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
    solver = int(sys.argv[4]) if len(sys.argv) > 4 else 20
    seed = int(sys.argv[5]) if len(sys.argv) > 5 else 20261015
    policy = native.Policy(str(DATA / "runs" / run / "exports" / "policy_latest.mrlpolicy"))
    path = str(REPO / "assets" / "rl" / "tasks" / f"{task}.json")
    probe = native.BodyBatch(path, environments=1, per_scene=1, threads=1)
    print(f"{run} em {task}, solver {'jogo' if solver == 20 else 'treino'}, {seconds:.0f} s, semente {seed}")
    for drill in list(probe.drill_ids):
        batch = native.BodyBatch(path, environments=64, per_scene=4, threads=8, seed=seed, evaluation=True,
                                 evaluation_seconds=seconds, solver_position=solver, solver_velocity=solver, drill=drill)
        col = {name: i for i, name in enumerate(batch.episode_columns())}
        first: dict[int, np.ndarray] = {}
        for _ in range(int((seconds + 0.5) / batch.policy_dt)):
            obs = np.ascontiguousarray(batch.observations(), dtype=np.float32)
            action = np.ascontiguousarray(policy.evaluate(obs), dtype=np.float32)
            batch.step(action, action)
            for row in batch.finished_episodes():
                first.setdefault(int(row[col["mundo"]]), row)
            if len(first) >= 64:
                break
        rows = np.array(list(first.values()))
        falls = np.isin(rows[:, col["razao"]].astype(int), [1, 2]).mean()
        print(f"{drill:14s} quedas {100 * falls:4.0f}% | erro_vel med {np.median(rows[:, col['erro_velocidade']]):.2f}"
              f" | alvos/min {rows[:, col['alvos_por_minuto']].mean():4.1f}"
              f" | deriva espera {mean_valid(rows, col['deriva_espera']):.2f} m"
              f" | curva {mean_valid(rows, col['afastamento_curva']):.2f} m"
              f" | base {mean_valid(rows, col['largura_base']):.3f} m cruzados {100 * rows[:, col['pes_cruzados']].mean():.0f}%"
              f" | peito {np.degrees(np.median(rows[:, col['erro_olhar']])):.0f} graus"
              f" | pes parado {rows[:, col['parado_pes']].mean():.3f} | pivo {rows[:, col['pivo_pe']].mean():.2f} rad/s"
              f" | postura {rows[:, col['postura']].mean():.3f} | escorregar {rows[:, col['escorregar']].mean():.3f}"
              f" | segundos {rows[:, col['segundos']].mean():.1f}"
              + (f" | peito {rows[:, col['peito_frente']].mean():+.1f} pelve {rows[:, col['pelve_frente']].mean():+.1f} graus"
                 f" tras {100 * rows[:, col['peito_tras']].mean():.0f}%" if "peito_frente" in col else "")
              + (f" | tronco {rows[:, col['tronco_frente']].mean():+.1f} graus tras {100 * rows[:, col['tronco_tras']].mean():.0f}%"
                 if "tronco_frente" in col else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
