"""Avaliacoes fora do treino: a bateria num checkpoint qualquer (o A/B entre
dois) e a gravacao inteira de mundos escolhidos, para as folhas. Roda na CPU,
com prioridade baixa, sem falar com o treinador."""
from __future__ import annotations

import math
import os
from pathlib import Path

import torch

from . import native, paths, storage
from .evaluation import Battery, Evaluator
from .model import ActorCritic, migrate_inputs


def resolve(run: str, spec: str) -> storage.CheckpointRef:
    """Um checkpoint do treino: o numero da rodada, "ultimo" ou "melhor"."""
    directory = paths.run_dir(run)
    found = storage.list_checkpoints(directory)
    if not found:
        raise SystemExit(f"o treino {run} nao tem checkpoints")
    latest = storage.latest_checkpoint(directory)
    if spec in ("ultimo", "latest"):
        return latest
    if spec in ("melhor", "best"):
        spec = str(latest.manifest.get("best", {}).get("iteration", -1))
    try:
        wanted = int(spec)
    except ValueError:
        raise SystemExit(f"checkpoint desconhecido: {spec} (o numero da rodada, ultimo ou melhor)") from None
    for reference in found:
        if reference.iteration == wanted:
            return reference
    kept = ", ".join(str(reference.iteration) for reference in found[-10:])
    raise SystemExit(f"o treino {run} nao tem checkpoint da rodada {wanted} (os guardados: {kept})")


def evaluate(run: str, spec: str, threads: int = 2, record: Path | None = None, watch: tuple[int, ...] = (0,),
             solvers: list[str] | None = None, postures: list[str] | None = None,
             names: list[str] | None = None) -> tuple[storage.CheckpointRef, dict]:
    """As baterias do treino (a tarefa com os pesos de agora) num checkpoint:
    {bateria: {solver: metricas}}."""
    os.nice(10)  # o desktop e o treinador passam na frente
    reference = resolve(run, spec)
    state = storage.load_checkpoint(reference)
    model = ActorCritic(len(state["obs_names"]), len(state["critic_names"]), len(state["action_names"]))
    model.load_state_dict(state["model"])
    model.eval()
    battery = Battery()
    if solvers:
        battery.solvers = {name: value for name, value in battery.solvers.items() if name in solvers}
    torch.set_num_threads(1)
    evaluator = Evaluator(native.load(), paths.run_dir(run) / "tarefa.json", threads, battery, postures, names)
    names = evaluator.observation_names()
    if names != state["obs_names"]:
        # Um checkpoint de antes de uma entrada nova: a mesma migracao do treino
        # (a rede faz o mesmo; a entrada nova entra com peso zero).
        critic = next(iter(evaluator.batches.values())).critic_names()
        model = migrate_inputs(model, state["obs_names"], names, state["critic_names"], critic)
        model.eval()
    return reference, evaluator.run(model, torch.device("cpu"), record, watch)


ROWS = [
    ("de pe por 60 s", lambda m: f"{100 * m['stand_fraction']:.0f}%"),
    ("tempo medio (s)", lambda m: f"{m['mean_seconds']:.1f}"),
    ("oscilacao (cm)", lambda m: f"{100 * m['sway']:.2f}"),
    ("deriva dos pes: media (cm)", lambda m: f"{100 * m['foot_drift']:.1f}"),
    ("deriva dos pes: mediana (cm)", lambda m: f"{100 * m.get('foot_drift_median', m['foot_drift']):.1f}"),
    ("deriva dos pes: pior mundo (cm)", lambda m: f"{100 * m.get('foot_drift_worst', m['foot_drift']):.1f}"),
    ("giro do tronco: media (graus/s)", lambda m: f"{math.degrees(m.get('turn', 0.0)):.1f}"),
    ("giro do tronco: pior mundo (graus/s)", lambda m: f"{math.degrees(m.get('turn_worst', 0.0)):.1f}"),
    ("pe girando no chao: media (graus)", lambda m: f"{math.degrees(m.get('foot_turn', 0.0)):.1f}"),
    ("pe girando no chao: pior mundo (graus)", lambda m: f"{math.degrees(m.get('foot_turn_worst', 0.0)):.1f}"),
    ("velocidade das juntas (rad/s)", lambda m: f"{m.get('joint_speed', 0.0):.3f}"),
    ("erro da postura pedida (graus, RMS das juntas)", lambda m: f"{math.degrees(m.get('pose_error', 0.0)):.1f}"),
    ("andar: erro da velocidade (m/s, RMS)", lambda m: f"{m.get('velocity_error', 0.0):.2f}"),
    ("andar: desvio para o lado (m)", lambda m: f"{m.get('lateral_drift', 0.0):.2f}"),
    ("andar: distancia (m)", lambda m: f"{m.get('distance', 0.0):.1f}"),
    ("ir ate o ponto: erro no fim (m)", lambda m: f"{m['goal_error']:.2f}" if "goal_error" in m else "—"),
    ("tremor (mudanca da acao)", lambda m: f"{m['action_rate']:.4f}"),
    ("esforco", lambda m: f"{m['effort']:.3f}"),
    ("postura", lambda m: f"{m['upright']:.3f}"),
    ("nota", lambda m: f"{m['score']:.3f}"),
]


def compare(run: str, a: str, b: str, threads: int = 2) -> str:
    """A/B: a mesma bateria nos dois checkpoints, lado a lado (Markdown)."""
    directory = paths.run_dir(run) / "comparacoes"
    results = []
    for spec in (a, b):
        reference = resolve(run, spec)
        record = directory / f"ck-{reference.iteration:09d}"
        results.append(evaluate(run, spec, threads, record))
    (ra, ma), (rb, mb) = results
    lines = [f"# {run}: rodada {ra.iteration} x rodada {rb.iteration}", ""]
    for battery in ma:
        for solver in ma[battery]:
            lines += [f"## {battery}, solver {solver}", "", f"| | {ra.iteration} | {rb.iteration} |", "|---|---:|---:|"]
            lines += [f"| {label} | {show(ma[battery][solver])} | {show(mb[battery][solver])} |" for label, show in ROWS]
            lines.append("")
    lines.append("Gravacoes (para `matter-rl folha --episodio ...`):")
    lines += [f"- {path}" for path in sorted(directory.glob("ck-*.dump"))
              if path.stem.startswith((f"ck-{ra.iteration:09d}", f"ck-{rb.iteration:09d}"))]
    return "\n".join(lines)
