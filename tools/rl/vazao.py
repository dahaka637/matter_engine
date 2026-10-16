#!/usr/bin/env python3
"""A vazao do lote do Corpo (02_estrutura/vazao_dos_treinos.md): decisoes por
segundo so do step() (sem a rede nem o PPO), com a divisao por fase.

A politica (uma exportada) decide fora do relogio, para os corpos andarem como
no treino. O solver e o do treino (10/10 com a mistura de 20/20).

  .venv/bin/python vazao.py [--tarefa fase2g] [--run f2g] [--mundos 512] [--por-cena 4]
                            [--threads 10] [--passos 300] [--aquecimento 30]
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1].parent
import os
sys.path.insert(0, str(REPO / os.environ.get("MATTER_RL_BUILD", "build-rl") / "python"))
import matter_rl_native as native  # noqa: E402

DATA = Path.home() / ".local" / "share" / "matter-engine" / "rl"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tarefa", default="fase2g")
    parser.add_argument("--run", default="f2g")
    parser.add_argument("--mundos", type=int, default=512)
    parser.add_argument("--por-cena", type=int, default=4)
    parser.add_argument("--threads", type=int, default=10)
    parser.add_argument("--passos", type=int, default=300)
    parser.add_argument("--aquecimento", type=int, default=30)
    parser.add_argument("--solver", type=int, default=0, help="iteracoes fixas do solver em todos os mundos (0 = a mistura do treino)")
    args = parser.parse_args()
    task = REPO / "assets" / "rl" / "tasks" / f"{args.tarefa}.json"
    policy = native.Policy(str(DATA / "runs" / args.run / "exports" / "policy_latest.mrlpolicy"))
    extra = {"solver_position": args.solver, "solver_velocity": args.solver} if args.solver else {}
    batch = native.BodyBatch(str(task), environments=args.mundos, per_scene=args.por_cena, threads=args.threads, seed=7, **extra)
    stepped = 0.0
    for i in range(args.aquecimento + args.passos):
        if i == args.aquecimento:
            batch.take_phase_seconds()
            stepped = 0.0
        obs = np.ascontiguousarray(batch.observations(), dtype=np.float32)
        action = np.ascontiguousarray(policy.evaluate(obs), dtype=np.float32)
        noisy = np.ascontiguousarray(action + np.random.normal(0.0, 0.06, action.shape).astype(np.float32))
        begin = time.perf_counter()
        batch.step(noisy, action)
        stepped += time.perf_counter() - begin
    phases = batch.take_phase_seconds()
    decisions = args.mundos * args.passos
    split = {k: phases.pop(k) for k in ("fisica_antes", "fisica_jolt", "fisica_depois")}
    total = sum(phases.values())
    print(f"{args.mundos} mundos, {args.por_cena} por cena, {args.threads} threads: "
          f"{decisions / stepped:,.0f} decisoes/s so do step() ({1e6 * stepped / args.passos / 1000:.1f} ms por passo)")
    print("  fases (tempo de thread): " + ", ".join(f"{k} {100 * v / total:.0f}%" for k, v in phases.items()))
    print("  a fisica por dentro: " + ", ".join(f"{k.split('_')[1]} {100 * v / phases['fisica']:.0f}%" for k, v in split.items()))
    print(f"  {1e6 * total / decisions:.0f} us de thread por decisao; paralelismo efetivo {total / stepped:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
