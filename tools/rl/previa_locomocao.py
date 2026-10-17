#!/usr/bin/env python3
"""A previa da locomocao (o RL dentro dos modulos do jogo, contrato v2): o
ultimo checkpoint de um treino (uma leitura; nada se escreve nele) - ou o
classico, sem rede - num exercicio da tarefa, mundo a mundo, mais os dumps para
as folhas de quadros (MatterModelInspector quadros).

A acao e a media da politica (como no jogo), na escala do teto da fase.

  .venv/bin/python previa_locomocao.py --treino loco1 --exercicio estrela --segundos 12 \\
      --saida /tmp/previa --mundos 0,1 --folha
  .venv/bin/python previa_locomocao.py --classico --tarefa locomocao_f1f2 --exercicio curva --saida /tmp/c --folha

Com --folha, cada mundo com dump vira uma folha de quadros (m<N>.png).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
from matter_rl import paths, storage  # noqa: E402
from matter_rl.locomotion_trainer import LocomotionRunConfig, native  # noqa: E402
from matter_rl.model import ActorCritic  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", default="", help="o treino (o ultimo checkpoint dele)")
    parser.add_argument("--classico", action="store_true", help="sem rede: o classico (autoridade zero)")
    parser.add_argument("--tarefa", default="", help="a tarefa (padrao: a copia do treino)")
    parser.add_argument("--exercicio", default="", help="o exercicio de todos os mundos (vazio = as fatias)")
    parser.add_argument("--segundos", type=float, default=12.0)
    parser.add_argument("--mundos-total", type=int, default=8)
    parser.add_argument("--mundos", default="0", help="os mundos com dump, separados por virgula")
    parser.add_argument("--semente", type=int, default=20261016)
    parser.add_argument("--saida", required=True)
    parser.add_argument("--folha", action="store_true")
    parser.add_argument("--quadros", type=int, default=16)
    parser.add_argument("--vista", default="34")
    args = parser.parse_args()
    if not args.treino and not args.classico:
        parser.error("--treino ou --classico")

    module = native()
    out = Path(args.saida)
    out.mkdir(parents=True, exist_ok=True)
    run_dir = paths.run_dir(args.treino) if args.treino else None
    if args.tarefa:
        task = Path(args.tarefa) if Path(args.tarefa).exists() else paths.task_path(args.tarefa)
    elif run_dir is not None:
        task = run_dir / "tarefa.json"
    else:
        parser.error("--tarefa (sem treino)")
    batch = module.LocomotionBatch(str(task), str(paths.ASSETS), environments=args.mundos_total, threads=4,
                                   seed=args.semente, evaluation=True, evaluation_seconds=args.segundos + 2.0,
                                   drill=args.exercicio)
    model = None
    scale = torch.as_tensor(np.asarray(batch.action_cap_fractions(), dtype=np.float32))
    iteration = "-"
    if args.classico:
        batch.set_authority(0.0, 0.0, 0.0, 0.0)
    else:
        reference = storage.latest_checkpoint(run_dir)
        if reference is None:
            print(f"o treino {args.treino} nao tem checkpoint", file=sys.stderr)
            return 1
        state = storage.load_checkpoint(reference, "cpu")
        if state["obs_names"] != list(batch.observation_names()) or state["action_names"] != list(batch.action_names()):
            print("o checkpoint e de outro contrato (as entradas ou as acoes diferem)", file=sys.stderr)
            return 1
        config = LocomotionRunConfig.load(run_dir / "run.json")
        network = state.get("network", config.network)
        model = ActorCritic(len(state["obs_names"]), len(state["critic_names"]), len(state["action_names"]),
                            actor_hidden=network["actor"], critic_hidden=network["critic"])
        model.load_state_dict(state["model"])
        model.eval()
        if state.get("action_scale"):
            scale = torch.as_tensor(np.asarray(state["action_scale"], dtype=np.float32))
        authority = float(min(1.0, max(0.0, state["counters"].get("authority", 1.0))))
        batch.set_authority(authority, authority, authority, authority)
        iteration = state["counters"]["iteration"]
    batch.reset(args.semente)
    columns = batch.episode_columns()
    col = {name: i for i, name in enumerate(columns)}
    reasons = batch.end_reasons()
    worlds = batch.num_envs
    watch = {int(w) for w in args.mundos.split(",") if w.strip()}
    dumps = {w: [f"S mundo{w}"] for w in watch}
    alive = np.ones(worlds, bool)
    ending = [""] * worlds
    speeds = [[] for _ in range(worlds)]
    for step in range(int(args.segundos / batch.policy_dt)):
        obs = torch.as_tensor(np.asarray(batch.observations(), dtype=np.float32))
        if model is None:
            action = np.zeros((worlds, batch.action_size), dtype=np.float32)
        else:
            with torch.no_grad():
                action = (model.actor(model.obs_norm(obs)) * scale).numpy().astype(np.float32)
        batch.step(np.ascontiguousarray(action))
        for row in batch.finished_episodes():
            e = int(row[col["mundo"]])
            if alive[e]:
                alive[e] = False
                ending[e] = reasons[int(row[col["razao"]])] + f" {row[col['segundos']]:.1f}s"
        for e in range(worlds):
            if not alive[e]:
                continue
            lines = batch.frame_lines(e)
            for line in lines.splitlines():
                if line.startswith("U "):
                    wanted, speed = (float(v) for v in line.split()[1:3])
                    speeds[e].append((wanted, speed))
            if e in dumps and step % 2 == 0:
                dumps[e] += [f"F {(step + 1) * batch.policy_dt:.4f} 0", lines.rstrip()]
    for w, lines in dumps.items():
        (out / f"m{w}.dump").write_text("\n".join(lines) + "\n")
    if args.folha:
        inspector = paths.inspector()
        if inspector is None:
            print("MatterModelInspector ausente: as folhas ficam por fazer", file=sys.stderr)
        else:
            for w in sorted(dumps):
                subprocess.run([str(inspector), "quadros", "--dump", str(out / f"m{w}.dump"), "--quadros",
                                str(args.quadros), "--colunas", "8", "--largura", "180", "--altura", "260", "--vista",
                                args.vista, "--saida", str(out / f"m{w}.png")], check=False, capture_output=True)
                print(f"folha: {out / f'm{w}.png'}")
    drills = list(batch.drill_ids)
    who = "classico" if model is None else f"{args.treino} rodada {iteration}"
    print(f"{who} · tarefa {Path(task).stem} · exercicio {args.exercicio or '(as fatias)'} · {args.segundos:.0f} s")
    for e in range(worlds):
        drill = drills[int(batch.world_drills()[e])]
        pairs = np.array(speeds[e]) if speeds[e] else np.zeros((0, 2))
        speed = (f" | pedida {pairs[:, 0].mean():.2f}, feita {pairs[:, 1].mean():.2f} m/s" if len(pairs) else "")
        print(f"mundo {e:2d} ({drill}): {'de pe' if alive[e] else ending[e]:16s}{speed}")
    print("de pe no fim:", int(alive.sum()), "de", worlds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
