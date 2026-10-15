#!/usr/bin/env python3
"""A avaliacao de um degrau da escada de recuperacao (plano, secao 6): a politica
de um treino (o ultimo checkpoint ou o dado), a acao media (como no jogo), em
mundos fixos, a partir do parado, no solver do jogo (20/20) e no do treino.

  .venv/bin/python avaliar.py RUN [--pedido 1.0] [--mundos 64] [--segundos 20]
                              [--exercicio andar_frente] [--solver jogo|treino|ambos]

--pedido fixa a velocidade pedida (a janela do curriculo vira [v, v]); sem ele vale a
janela da tarefa do treino. Mostra, por exercicio e solver: as quedas (com o limite
de cima de 95%), o erro de velocidade, os passos uteis por segundo, a partida, a
sequencia e a caminhada sustentada (>= 6 passos seguidos e 8 s sem cair).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

from matter_rl import native, paths, storage
from matter_rl.body_trainer import BodyRunConfig, drill_summary
from matter_rl.model import ActorCritic


def load_policy(run: str, iteration: int | None):
    directory = paths.run_dir(run)
    references = storage.list_checkpoints(directory)
    if iteration is not None:
        references = [r for r in references if r.iteration == iteration]
    if not references:
        raise SystemExit(f"{run}: sem checkpoint")
    state = storage.load_checkpoint(references[-1], "cpu")
    network = state.get("network", BodyRunConfig().network)
    model = ActorCritic(len(state["obs_names"]), len(state["critic_names"]), len(state["action_names"]),
                        actor_hidden=network["actor"], critic_hidden=network["critic"])
    model.load_state_dict(state["model"])
    model.eval()
    return model, state, references[-1].iteration


def evaluate(run: str, request: float | None, worlds: int, seconds: float, drill: str, solver: str,
             iteration: int | None = None, seed: int = 20261014) -> dict:
    model, state, found = load_policy(run, iteration)
    task = json.loads((paths.run_dir(run) / "tarefa.json").read_text())
    if request is not None:
        task.setdefault("curriculum", {}).update(speedMin=request, speedMax=request)
    scratch = paths.run_dir(run) / "avaliacao_tmp.json"
    scratch.write_text(json.dumps(task))
    module = native.load()
    iterations = 20 if solver == "jogo" else -1
    batch = module.BodyBatch(str(scratch), environments=worlds, per_scene=4, threads=8, seed=seed, evaluation=True,
                             evaluation_seconds=seconds, solver_position=iterations, solver_velocity=iterations,
                             drill=drill)
    columns = {name: i for i, name in enumerate(batch.episode_columns())}
    first: dict[int, np.ndarray] = {}
    with torch.no_grad():
        for _ in range(int(seconds / batch.policy_dt) + 2):
            obs = torch.tensor(np.asarray(batch.observations()))
            action = np.ascontiguousarray(model.actor(model.obs_norm(obs)).numpy(), dtype=np.float32)
            batch.step(action, action)
            for row in batch.finished_episodes():
                first.setdefault(int(row[columns["mundo"]]), np.array(row))
            if len(first) >= batch.num_envs:
                break
    rows = np.array(list(first.values())) if first else np.zeros((0, len(columns)))
    summary = drill_summary(rows, batch.episode_columns(), list(batch.drill_ids)).get(drill or batch.drill_ids[0], {})
    summary.update(checkpoint=found, steps=state.get("counters", {}).get("steps"), request=request, solver=solver,
                   worlds=batch.num_envs)
    scratch.unlink(missing_ok=True)
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run")
    parser.add_argument("--pedido", type=float)
    parser.add_argument("--mundos", type=int, default=64)
    parser.add_argument("--segundos", type=float, default=20.0)
    parser.add_argument("--exercicio", default="")
    parser.add_argument("--solver", choices=["jogo", "treino", "ambos"], default="jogo")
    parser.add_argument("--rodada", type=int)
    args = parser.parse_args(argv)
    results = {}
    for solver in (["jogo", "treino"] if args.solver == "ambos" else [args.solver]):
        results[solver] = evaluate(args.run, args.pedido, args.mundos, args.segundos, args.exercicio, solver,
                                   args.rodada)
    for solver, r in results.items():
        if not r:
            print(f"[{solver}] nenhum episodio terminou")
            continue
        print(f"[{solver}] rodada {r['checkpoint']} ({(r['steps'] or 0) / 1e6:.1f} M decisoes), "
              f"pedido {r['request'] if r['request'] is not None else 'da tarefa'}, {r['worlds']} mundos")
        print(f"  quedas {r['falls']:.0%} (limite de cima {r['falls_upper95']:.0%}); segundos {r['seconds']:.1f}")
        print(f"  erro de velocidade: mediana {r['velocity_error_median']:.2f}, p90 {r['velocity_error_p90']:.2f} m/s "
              f"(pedido medio {r['requested']:.2f}); nota de progresso media {r['velocity_score']:.2f}")
        start = f"{r['first_step_median']:.2f} s" if r["first_step_median"] is not None else "sem passo"
        print(f"  passos uteis {r['useful_steps']:.2f}/s; partida {start} ({r['first_step_fraction']:.0%} deram o primeiro "
              f"passo); sequencia mediana {r['step_run_median']:.0f}; sustentada {r['sustained']:.0%}")
        print(f"  escorregar {r['slip']:.3f} m/s; porta de pe {r['stand_gate']:.2f}; cadencia {r['cadence']:.2f}/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
