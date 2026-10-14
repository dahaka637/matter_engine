#!/usr/bin/env python3
"""A previa do Corpo v2: uma politica exportada (uma copia; nada se escreve no
treino) num exercicio da tarefa, com a fisica do jogo (solver 20/20), mundo a
mundo, mais os dumps para as folhas de quadros (MatterModelInspector quadros).

A acao e a media da politica (como no jogo). A entrada e a do contrato v2: a
politica precisa ter os mesmos nomes de entrada da tarefa.

  .venv/bin/python previa_corpo.py --tarefa corpo2 --exercicio deslocamento \\
      --politica ~/.local/share/matter-engine/rl/runs/corpo2/exports/policy_latest.mrlpolicy \\
      --segundos 20 --saida /tmp/previa --mundos 0,3
  ../../build-profile/MatterModelInspector quadros --dump /tmp/previa/m0.dump --quadros 16 --colunas 8
"""

from __future__ import annotations

import argparse
import json
import shutil
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "build-rl" / "python"))
import matter_rl_native as native  # noqa: E402


def policy_names(path: Path) -> list[str]:
    data = path.read_bytes()
    size = struct.unpack_from("<I", data, 8)[0]
    return list(json.loads(data[12:12 + size]).get("observationNames", []))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tarefa", default="corpo2", help="o id da tarefa (assets/rl/tasks) ou um caminho")
    parser.add_argument("--exercicio", default="", help="o exercicio de todos os mundos (vazio = as cotas)")
    parser.add_argument("--politica", required=True)
    parser.add_argument("--segundos", type=float, default=20.0)
    parser.add_argument("--mundos-total", type=int, default=16)
    parser.add_argument("--mundos", default="0", help="os mundos com dump, separados por virgula")
    parser.add_argument("--solver", choices=["jogo", "treino"], default="jogo")
    parser.add_argument("--saida", required=True)
    args = parser.parse_args()

    out = Path(args.saida)
    out.mkdir(parents=True, exist_ok=True)
    policy_file = out / "politica.mrlpolicy"
    shutil.copy(args.politica, policy_file)
    task = Path(args.tarefa) if Path(args.tarefa).exists() else REPO / "assets" / "rl" / "tasks" / f"{args.tarefa}.json"
    policy = native.Policy(str(policy_file))
    worlds = args.mundos_total
    iterations = 20 if args.solver == "jogo" else -1
    batch = native.BodyBatch(str(task), environments=worlds, per_scene=4, threads=4, seed=20261014, evaluation=True,
                             evaluation_seconds=args.segundos + 2.0, solver_position=iterations,
                             solver_velocity=iterations, drill=args.exercicio)
    names = list(batch.observation_names())
    wanted = policy_names(policy_file)
    if wanted and wanted != names:
        print("a politica nao e deste contrato (as entradas diferem)", file=sys.stderr)
        return 1
    columns = batch.episode_columns()
    col = {name: i for i, name in enumerate(columns)}
    watch = {int(w) for w in args.mundos.split(",") if w.strip()}
    dumps = {w: [f"S mundo{w}"] for w in watch}
    alive = np.ones(worlds, bool)
    ending = [""] * worlds
    summary: list[list[float] | None] = [None] * worlds
    velocity_error = [[] for _ in range(worlds)]
    for step in range(int(args.segundos / batch.policy_dt)):
        obs = np.ascontiguousarray(batch.observations(), dtype=np.float32)
        action = np.ascontiguousarray(policy.evaluate(obs), dtype=np.float32)
        batch.step(action, action)
        for row in batch.finished_episodes():
            e = int(row[col["mundo"]])
            if alive[e]:
                alive[e] = False
                ending[e] = native.BODY_END_REASONS[int(row[col["razao"]])] + f" {row[col['segundos']]:.1f}s"
                summary[e] = row
        for e in range(worlds):
            if not alive[e]:
                continue
            lines = batch.frame_lines(e)
            for line in lines.splitlines():
                if line.startswith("U "):
                    wanted_speed, speed = (float(v) for v in line.split()[1:3])
                    velocity_error[e].append(abs(speed - wanted_speed))
            if e in dumps and step % 2 == 0:
                dumps[e] += [f"F {(step + 1) * batch.policy_dt:.4f} 0", lines.rstrip()]
    for w, lines in dumps.items():
        (out / f"m{w}.dump").write_text("\n".join(lines) + "\n")
    drills = list(batch.drill_ids)
    print(f"politica rodada {policy.iteration} · tarefa {task.stem} · exercicio {args.exercicio or '(as cotas)'} · "
          f"solver {args.solver} · {args.segundos:.0f} s")
    for e in range(worlds):
        drill = drills[int(batch.world_drills()[e])]
        speed = f" | velocidade {np.mean(velocity_error[e]):.2f} m/s fora (modulo)" if velocity_error[e] else ""
        print(f"mundo {e:2d} ({drill}): {'de pe' if alive[e] else ending[e]:22s}{speed}")
    print("de pe no fim:", int(alive.sum()), "de", worlds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
