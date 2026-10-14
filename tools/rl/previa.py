#!/usr/bin/env python3
"""A previa de um exercicio: uma politica (uma copia; nada se escreve no treino)
numa tarefa e num exercicio, com a fisica do jogo, mundo a mundo, mais os dumps
para as folhas de quadros (MatterModelInspector quadros).

A observacao da politica e montada pelos nomes das entradas dela (as que a
tarefa nao tem entram com zero, como na migracao do treinador); uma politica
sem nomes le o comeco da observacao.

  .venv/bin/python previa.py --tarefa corpo --exercicio estrela --peito 1 \\
      --politica ~/.local/share/matter-engine/rl/runs/principal/exports/policy_latest.mrlpolicy \\
      --segundos 30 --saida /tmp/previa --mundos 0,4
"""

from __future__ import annotations

import argparse
import json
import math
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
    parser.add_argument("--tarefa", required=True, help="o id da tarefa (assets/rl/tasks) ou um caminho")
    parser.add_argument("--exercicio", default="", help="o exercicio (drill) de todos os mundos")
    parser.add_argument("--peito", type=int, default=-1, help="o comando de olhar: 1 fixo, 0 livre, -1 o da tarefa")
    parser.add_argument("--politica", required=True)
    parser.add_argument("--segundos", type=float, default=30.0)
    parser.add_argument("--mundos-total", type=int, default=16)
    parser.add_argument("--mundos", default="0", help="os mundos com dump, separados por virgula")
    parser.add_argument("--saida", required=True)
    args = parser.parse_args()

    out = Path(args.saida)
    out.mkdir(parents=True, exist_ok=True)
    policy_file = out / "politica.mrlpolicy"
    shutil.copy(args.politica, policy_file)
    task = Path(args.tarefa) if Path(args.tarefa).exists() else REPO / "assets" / "rl" / "tasks" / f"{args.tarefa}.json"
    policy = native.Policy(str(policy_file))
    worlds = args.mundos_total
    batch = native.Batch(str(task), environments=worlds, per_scene=4, threads=4, seed=20261014, evaluation=True,
                         evaluation_seconds=args.segundos + 2.0, solver_position=20, solver_velocity=20,
                         posture="alerta", facing=args.peito, drill=args.exercicio)
    names = list(batch.observation_names())
    wanted = policy_names(policy_file)
    if len(wanted) == policy.obs_size:
        pick = [names.index(n) if n in names else -1 for n in wanted]
        missing = [n for n in wanted if n not in names]
    else:
        pick, missing = list(range(policy.obs_size)), []
    clips = [n for n in names if n.startswith("reference.clip.")]
    clip_index = [names.index(c) for c in clips]
    watch = {int(w) for w in args.mundos.split(",") if w.strip()}
    dumps = {w: [f"S mundo{w}"] for w in watch}
    alive = np.ones(worlds, bool)
    ending = [""] * worlds
    arrivals = [[] for _ in range(worlds)]
    last_target = [None] * worlds
    used = [dict() for _ in range(worlds)]
    speed_error = [[] for _ in range(worlds)]
    chest_error = [[] for _ in range(worlds)]
    facing = (names.index("command.facing.sin"), names.index("command.facing.cos"),
              names.index("command.facing.active")) if "command.facing.active" in names else None
    for step in range(int(args.segundos / batch.policy_dt)):
        obs = np.asarray(batch.observations())
        x = np.zeros((worlds, len(pick)), dtype=np.float32)
        for j, i in enumerate(pick):
            if i >= 0:
                x[:, j] = obs[:, i]
        for e in range(worlds):
            if not alive[e]:
                continue
            clip = clips[int(np.argmax(obs[e, clip_index]))].split(".")[-1]
            used[e][clip] = used[e].get(clip, 0) + 1
            if facing and obs[e, facing[2]] > 0.5:
                chest_error[e].append(abs(math.degrees(math.atan2(obs[e, facing[0]], obs[e, facing[1]]))))
        batch.step(np.ascontiguousarray(policy.evaluate(x), dtype=np.float32))
        for row in batch.finished_episodes():
            e = int(row[0])
            if alive[e]:
                alive[e] = False
                ending[e] = native.END_REASONS[int(row[1])] + f" {row[5]:.1f}s"
        for e in range(worlds):
            if not alive[e]:
                continue
            lines = batch.frame_lines(e)
            target = velocity = pelvis = command = None
            for line in lines.splitlines():
                tag = line[:2]
                if tag == "A ":
                    if last_target[e] is not None and line != last_target[e]:
                        arrivals[e].append(round((step + 1) * batch.policy_dt, 1))
                    last_target[e] = line
                    target = [float(v) for v in line.split()[1:3]]
                elif tag == "V ":
                    velocity = [float(v) for v in line.split()[1:3]]
                elif tag == "U ":
                    command = float(line.split()[2])
                elif line.startswith(("B Pelvis ", "C Pelvis ")):
                    pelvis = [float(v) for v in line.split()[2:4]]
            # A velocidade em direcao ao alvo contra a pedida de agora.
            if target and velocity and pelvis and command is not None:
                dx, dy = target[0] - pelvis[0], target[1] - pelvis[1]
                distance = math.hypot(dx, dy)
                if distance > 1e-3:
                    speed_error[e].append((velocity[0] * dx + velocity[1] * dy) / distance - command)
            if e in dumps and step % 2 == 0:
                dumps[e] += [f"F {(step + 1) * batch.policy_dt:.4f} 0", lines.rstrip()]
    for w, lines in dumps.items():
        (out / f"m{w}.dump").write_text("\n".join(lines) + "\n")
    print(f"politica rodada {policy.iteration} · tarefa {task.stem} · exercicio {args.exercicio or '(o primeiro)'} · "
          f"peito {'fixo' if args.peito == 1 else 'livre' if args.peito == 0 else 'da tarefa'} · {args.segundos:.0f} s")
    if missing:
        print(f"entradas que esta tarefa nao tem (entram com zero): {len(missing)}")
    for e in range(worlds):
        top = sorted(used[e].items(), key=lambda kv: -kv[1])[:3]
        chest = f" | peito {np.mean(chest_error[e]):.0f} graus" if chest_error[e] else ""
        speed = f" | velocidade {np.sqrt(np.mean(np.square(speed_error[e]))):.2f} m/s fora" if speed_error[e] else ""
        print(f"mundo {e:2d}: {'de pe' if alive[e] else ending[e]:22s} | alvos {len(arrivals[e]):2d} "
              f"{arrivals[e][:4]}{speed}{chest} | {top}")
    print("de pe no fim:", int(alive.sum()), "de", worlds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
