#!/usr/bin/env python3
"""A medida do tremor do PARADO (so le o checkpoint): o quanto o corpo treme parado,
por mundo, depois de 3 s (junta, peito, pelve, acao e escorregar dos pes), na
rede e no classico.

  .venv/bin/python tremor_parado.py --treino loco_v3
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from matter_rl import paths, storage  # noqa: E402
from matter_rl.locomotion_trainer import native  # noqa: E402
from matter_rl.model import ActorCritic  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", required=True)
    parser.add_argument("--exercicio", default="parado")
    parser.add_argument("--mundos", type=int, default=32)
    parser.add_argument("--segundos", type=float, default=15.0)
    args = parser.parse_args()
    run = paths.run_dir(args.treino)
    state = storage.load_checkpoint(storage.latest_checkpoint(run), "cpu")
    net = state["network"]
    model = ActorCritic(len(state["obs_names"]), len(state["critic_names"]), len(state["action_names"]),
                        np.full(len(state["action_names"]), 0.3, dtype=np.float32), actor_hidden=net["actor"],
                        critic_hidden=net["critic"])
    model.load_state_dict(state["model"])
    model.eval()
    scale = torch.as_tensor(np.asarray(state["action_scale"], dtype=np.float32))
    nat = native()
    print(f"{args.treino} rodada {state.get('counters', {}).get('iteration')} · {args.exercicio} · {args.mundos} mundos x {args.segundos:.0f} s")
    for solver, authority in (("rede", (1, 1, 1, 1)), ("classico", (0, 0, 0, 0))):
        batch = nat.LocomotionBatch(str(run / "tarefa.json"), str(paths.ASSETS), environments=args.mundos, threads=3,
                                    seed=5150, evaluation=True, evaluation_seconds=args.segundos, drill=args.exercicio)
        names = list(batch.observation_names())
        joint_vel = [i for i, n in enumerate(names) if n.startswith("junta.") and n.endswith(".vel")]
        chest_gyro = [names.index(f"corpo.Chest.giro.{a}") for a in "xyz"]
        pelvis_vel = [names.index(f"corpo.Pelvis.vel.{a}") for a in "xyz"]
        pelvis_gyro = [names.index(f"corpo.Pelvis.giro.{a}") for a in "xyz"]
        batch.set_authority(*[float(a) for a in authority])
        batch.reset(5150)
        dt = batch.policy_dt
        steps = int(args.segundos / dt)
        series = {k: [] for k in ("junta", "peito", "peito_acel", "pelve_vel", "pelve_giro", "acao_delta")}
        last_obs = None
        last_action = None
        alive = np.ones(args.mundos, dtype=bool)
        columns = {n: i for i, n in enumerate(batch.episode_columns())}
        with torch.no_grad():
            for step in range(steps):
                obs = np.array(batch.observations())
                if solver == "classico":
                    action = np.zeros((args.mundos, batch.action_size), dtype=np.float32)
                else:
                    action = np.ascontiguousarray((model.actor(model.obs_norm(torch.as_tensor(obs))) * scale).numpy(), dtype=np.float32)
                if step * dt > 3.0:
                    series["junta"].append(np.sqrt((obs[:, joint_vel] ** 2).mean(axis=1)) * 10.0)   # rad/s (a entrada vem x0,1)
                    series["peito"].append(np.linalg.norm(obs[:, chest_gyro], axis=1))
                    series["pelve_vel"].append(np.linalg.norm(obs[:, pelvis_vel[:2]], axis=1))
                    series["pelve_giro"].append(np.linalg.norm(obs[:, pelvis_gyro], axis=1))
                    if last_obs is not None:
                        series["peito_acel"].append(np.linalg.norm(obs[:, chest_gyro] - last_obs[:, chest_gyro], axis=1) / dt)
                    if last_action is not None:
                        series["acao_delta"].append(np.abs(action - last_action).mean(axis=1))
                last_obs, last_action = obs, action
                batch.step(action)
                for row in batch.finished_episodes():
                    alive[int(row[columns["mundo"]])] = False
        print(f"\n  {solver} ({int(alive.sum())}/{args.mundos} mundos sem terminar)")
        labels = {"junta": "velocidade das juntas (rad/s, RMS)", "peito": "giro do peito (rad/s)",
                  "peito_acel": "aceleracao do giro do peito (rad/s2)", "pelve_vel": "velocidade da pelve (m/s)",
                  "pelve_giro": "giro da pelve (rad/s)", "acao_delta": "variacao da acao entre decisoes"}
        for key, label in labels.items():
            if not series[key]:
                continue
            values = np.array(series[key])[:, alive] if alive.any() else np.array(series[key])
            per_world = values.mean(axis=0)
            print(f"    {label:42s} mediana {np.median(per_world):.4f}   p90 {np.percentile(per_world, 90):.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
