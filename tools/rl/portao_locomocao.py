#!/usr/bin/env python3
"""O portao de uma fase da locomocao (vault 04_locomocao/cartao_fase3.md): o
ultimo checkpoint de um treino (so lido; o resultado vai em portao.json na pasta
dele) em cada
exercicio da tarefa dele, com a acao media, contra o classico (autoridade zero)
e contra a mesma rede sem a pose e a rigidez (a ablacao). Passa quando:

- cada exercicio fica no minimo dele (as quedas e o erro mediano de velocidade
  da tarefa);
- nenhum exercicio piora contra o classico (as quedas ate 5 pontos acima; o
  erro de velocidade ate 0,03 m/s acima - o ruido dos parados);
- a rede sem pose e rigidez ainda anda mais perto do pedido que o classico onde
  ha movimento (a melhora nao vem so delas).

A cara (as folhas e o laboratorio) e do usuario: o portao so mede.

  .venv/bin/python portao_locomocao.py --treino loco_f1f2 [--mundos 32] [--segundos 20]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from matter_rl import paths, storage  # noqa: E402
from matter_rl.locomotion_trainer import drill_summary, native  # noqa: E402
from matter_rl.model import ActorCritic  # noqa: E402


_BATCHES: dict = {}


def evaluate(module, task: Path, drill: str, solver: str, model, scale, worlds: int, seconds: float, threads: int) -> dict:
    # (Um lote por exercicio, reaproveitado nos tres solvers: o reset com a
    # mesma semente recomeca igual.)
    key = (str(task), drill, worlds, seconds, threads)
    batch = _BATCHES.get(key)
    if batch is None:
        batch = module.LocomotionBatch(str(task), str(paths.ASSETS), environments=worlds, threads=threads,
                                       seed=20261018, evaluation=True, evaluation_seconds=seconds, drill=drill)
        _BATCHES.clear()
        _BATCHES[key] = batch
    authority = {"rede": (1, 1, 1, 1), "sem_pose_rigidez": (1, 1, 0, 0), "classico": (0, 0, 0, 0)}[solver]
    batch.set_authority(*authority)
    batch.reset(20261018)
    columns = batch.episode_columns()
    col = {name: i for i, name in enumerate(columns)}
    first: dict[int, np.ndarray] = {}
    for _ in range(int(seconds / batch.policy_dt) + 2):
        if solver == "classico":
            action = np.zeros((worlds, batch.action_size), dtype=np.float32)
        else:
            obs = torch.as_tensor(np.asarray(batch.observations(), dtype=np.float32))
            with torch.no_grad():
                action = (model.actor(model.obs_norm(obs)) * scale).numpy().astype(np.float32)
        batch.step(np.ascontiguousarray(action))
        for row in batch.finished_episodes():
            first.setdefault(int(row[col["mundo"]]), np.array(row))
        if len(first) >= worlds:
            break
    rows = np.array(list(first.values()))
    summary = drill_summary(rows, columns, list(batch.drill_ids), batch.end_reasons()).get(drill, {})
    summary["minimums"] = batch.drill_minimums().get(drill, {})
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", required=True)
    parser.add_argument("--tarefa", default="", help="outra tarefa (padrao: a copia do treino)")
    parser.add_argument("--mundos", type=int, default=128)
    parser.add_argument("--segundos", type=float, default=20.0)
    parser.add_argument("--threads", type=int, default=6)
    args = parser.parse_args()

    from matter_rl import system
    system.refuse_tuned_environment("portao")
    module = native()
    run = paths.run_dir(args.treino)
    task = (Path(args.tarefa) if Path(args.tarefa).exists() else paths.task_path(args.tarefa)) if args.tarefa \
        else run / "tarefa.json"
    reference = storage.latest_checkpoint(run)
    if reference is None:
        print(f"o treino {args.treino} nao tem checkpoint", file=sys.stderr)
        return 1
    state = storage.load_checkpoint(reference, "cpu")
    network = state["network"]
    model = ActorCritic(len(state["obs_names"]), len(state["critic_names"]), len(state["action_names"]),
                        actor_hidden=network["actor"], critic_hidden=network["critic"])
    model.load_state_dict(state["model"])
    model.eval()
    scale = torch.as_tensor(np.asarray(state["action_scale"], dtype=np.float32))
    drills = json.loads(Path(task).read_text())["drills"]
    print(f"portao: {args.treino} rodada {reference.iteration} · tarefa {json.loads(Path(task).read_text())['id']} · "
          f"{args.mundos} episodios de {args.segundos:.0f} s por exercicio e solver")
    results: dict = {}
    cache_path = run / "portao_classico.json"
    # (O binario entra na chave: um modulo recompilado refaz o classico.)
    build_id = str(int(Path(module.__file__).stat().st_mtime))
    try:
        classic_cache = json.loads(cache_path.read_text())
    except (OSError, ValueError):
        classic_cache = {}
    verdict = True
    reasons: list[str] = []
    for drill in drills:
        d = drill["id"]
        # (O classico nao depende da rede: em cache pela tarefa, o contrato, os
        # mundos e os segundos, num arquivo da pasta do treino.)
        classic_key = "|".join((hashlib.sha256(Path(task).read_bytes()).hexdigest()[:16], str(state.get("contract_hash", "")),
                                build_id, d, str(args.mundos), str(args.segundos)))
        results[d] = {solver: evaluate(module, task, d, solver, model, scale, args.mundos, args.segundos, args.threads)
                      for solver in ("rede", "sem_pose_rigidez")}
        if classic_key in classic_cache:
            results[d]["classico"] = classic_cache[classic_key]
        else:
            results[d]["classico"] = evaluate(module, task, d, "classico", model, scale, args.mundos, args.segundos,
                                              args.threads)
            classic_cache[classic_key] = results[d]["classico"]
            storage.write_json_atomic(cache_path, classic_cache)
        net, ablation, classic = results[d]["rede"], results[d]["sem_pose_rigidez"], results[d]["classico"]
        minimum = net.get("minimums", {})
        checks = []
        falls_cap = minimum.get("fall_fraction", -1.0)
        if falls_cap >= 0.0 and net["falls"] > falls_cap:
            checks.append(f"quedas {net['falls']:.0%} acima do minimo {falls_cap:.0%}")
        # Com poucos episodios o ponto e ruidoso (4 de 64 = 6%): alem do minimo, o
        # limite superior Wilson-95% das quedas fica SEMPRE ate 10% (a regra do
        # Codex, 17/10; 18/10: tambem nos exercicios de minimo maior).
        if falls_cap >= 0.0 and net["falls_upper95"] > 0.10:
            checks.append(f"quedas: o limite superior de 95% e {net['falls_upper95']:.0%} (ate 10%)")
        error_cap = minimum.get("velocity_error", -1.0)
        if error_cap >= 0.0 and net["velocity_error_median"] > error_cap:
            checks.append(f"erro de velocidade {net['velocity_error_median']:.2f} acima de {error_cap:.2f}")
        if net["falls"] > classic["falls"] + 0.05:
            checks.append(f"cai mais que o classico ({net['falls']:.0%} contra {classic['falls']:.0%})")
        if net["velocity_error_median"] > classic["velocity_error_median"] + 0.03:
            checks.append("o erro de velocidade piorou contra o classico")
        moving = net["requested"] > 0.3
        if moving and ablation["velocity_error_median"] >= classic["velocity_error_median"]:
            checks.append("sem pose e rigidez, nao melhora o classico (a melhora vem so delas)")
        passed = not checks
        verdict = verdict and passed
        reasons += [f"{d}: {c}" for c in checks]
        print(f"  {d:15s} {'passa' if passed else 'NAO '}  rede: quedas {net['falls']:.0%} (ate {net['falls_upper95']:.0%}) "
              f"vel {net['speed']:.2f}/{net['requested']:.2f} erro {net['velocity_error_median']:.2f} "
              f"estilo {net['style_error'] or -1:.3f} ajuda {net['assist']:.2f} | sem pose/rigidez erro "
              f"{ablation['velocity_error_median']:.2f} | classico: quedas {classic['falls']:.0%} vel "
              f"{classic['speed']:.2f} erro {classic['velocity_error_median']:.2f}", flush=True)
    print("PORTAO: " + ("passa (falta a cara: as folhas e o laboratorio)" if verdict else "nao passa"))
    for reason in reasons:
        print("  - " + reason)
    previous = {}
    try:
        previous = json.loads((run / "portao.json").read_text())
    except (OSError, ValueError):
        pass
    # Dois portoes seguidos, de execucoes independentes (a rodada avancou), para avancar.
    task_id = json.loads(Path(task).read_text())["id"]
    consecutive = verdict and bool(previous.get("passed")) and previous.get("iteration", -1) < reference.iteration \
        and previous.get("task_id") == task_id
    print("DOIS SEGUIDOS: " + ("sim - pode avancar" if consecutive else "ainda nao"))
    storage.write_json_atomic(run / "portao.json", {"consecutive": consecutive, "time": time.time(), "iteration": reference.iteration,
                                                    "task": str(task), "task_id": task_id, "passed": verdict, "reasons": reasons,
                                                    "results": results})
    return 0 if verdict else 2


if __name__ == "__main__":
    raise SystemExit(main())
