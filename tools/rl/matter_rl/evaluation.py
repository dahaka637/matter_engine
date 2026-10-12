"""A bateria de avaliacao: sementes fixas, politica sem ruido, mundo sem
variacao, episodios de 60 s, no solver do treino e no do jogo (20/20). Mede o
que a recompensa nao paga diretamente e grava as quedas para as folhas."""
from __future__ import annotations

import collections
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import torch

from .model import ACTION_LIMIT, ActorCritic

REASONS = ["nenhum", "queda_contato", "queda_altura", "numerico", "tempo", "limite"]
# As colunas do resumo de um episodio (finished_episodes do modulo nativo).
EPISODE_COLUMNS = 14
FALLS = {1, 2, 3}


@dataclass
class Battery:
    name: str = "equilibrio_v1"
    environments: int = 32
    seconds: float = 60.0
    seed: int = 20261012
    solvers: dict = field(default_factory=lambda: {"treino": (-1, -1), "jogo": (20, 20)})


def score(metrics: dict) -> float:
    """Quanto ficou de pe, descontando oscilacao, tremedeira, deriva e o tronco
    girando (o giro passa despercebido na oscilacao do centro de massa)."""
    return (metrics["stand_fraction"]
            - 0.15 * min(1.0, metrics["sway"] / 0.05)
            - 0.15 * min(1.0, metrics["action_rate"] / 0.05)
            - 0.10 * min(1.0, metrics["foot_drift"] / 0.20)
            - 0.15 * min(1.0, metrics.get("turn", 0.0) / 0.30))


class Evaluator:
    def __init__(self, native, task_path: Path, threads: int, battery: Battery | None = None):
        self.battery = battery or Battery()
        self.native = native
        self.batches = {}
        for solver, (position, velocity) in self.battery.solvers.items():
            self.batches[solver] = native.Batch(
                str(task_path), environments=self.battery.environments, per_scene=4, threads=threads,
                seed=self.battery.seed, evaluation=True, evaluation_seconds=self.battery.seconds,
                solver_position=position, solver_velocity=velocity)

    def observation_names(self) -> list[str]:
        return next(iter(self.batches.values())).observation_names()

    @torch.no_grad()
    def run(self, model: ActorCritic, device: torch.device, record: Path | None = None,
            watch: tuple[int, ...] = (0,)) -> dict:
        """watch: os mundos gravados inteiros (alem das primeiras quedas)."""
        results = {}
        for solver, batch in self.batches.items():
            results[solver] = self._run_one(model, device, batch, record, solver, watch)
        return results

    def _run_one(self, model: ActorCritic, device: torch.device, batch, record: Path | None, solver: str,
                 watch: tuple[int, ...]) -> dict:
        envs = batch.num_envs
        dt = batch.policy_dt
        batch.reset(self.battery.seed)
        first: dict[int, np.ndarray] = {}
        rings = [collections.deque(maxlen=48) for _ in range(envs)]
        watch = tuple(w for w in watch if 0 <= w < envs)
        full: dict[int, list[tuple[float, str]]] = {w: [] for w in watch}
        falls: list[tuple[str, list]] = []
        limit = int(math.ceil(self.battery.seconds / dt)) + 2
        for step in range(limit):
            obs = torch.tensor(batch.observations(), device=device)
            action = model.actor(model.obs_norm(obs)).clamp(-ACTION_LIMIT, ACTION_LIMIT)
            batch.step(np.ascontiguousarray(action.cpu().numpy(), dtype=np.float32))
            t = (step + 1) * dt
            # (Quem acabou ja recomecou dentro do step: o anel guarda ate o
            # quadro anterior ao fim.)
            for row in batch.finished_episodes():
                e = int(row[0])
                if e in first:
                    continue
                first[e] = row
                reason = int(row[1])
                if record is not None and reason in FALLS and len(falls) < 4:
                    falls.append((f"queda_mundo{e}_{REASONS[reason]}_{row[5]:.1f}s", list(rings[e])))
            if record is not None and step % 2 == 0:
                for e in range(envs):
                    if e not in first:
                        rings[e].append((t, batch.frame_lines(e)))
                for w in watch:
                    if w not in first:
                        full[w].append((t, rings[w][-1][1]))
            if len(first) >= envs:
                break
        rows = (np.array([first[e] for e in sorted(first)], dtype=np.float32) if first
                else np.zeros((0, EPISODE_COLUMNS)))
        metrics = self._metrics(rows, envs)
        if record is not None:
            record.parent.mkdir(parents=True, exist_ok=True)
            with open(record.with_name(record.stem + f"-{solver}.dump"), "w") as out:
                for name, frames in [(f"parado_mundo{w}_{solver}", full[w]) for w in watch] + falls:
                    out.write(f"S {name}\n")
                    for t, lines in frames:
                        out.write(f"F {t:.4f} 0\n")
                        out.write(lines)
        return metrics

    @staticmethod
    def _metrics(rows: np.ndarray, envs: int) -> dict:
        if len(rows) == 0:
            return {"stand_fraction": 0.0, "mean_seconds": 0.0, "sway": 1.0, "foot_drift": 1.0, "foot_drift_median": 1.0,
                    "foot_drift_worst": 1.0, "action_rate": 1.0, "effort": 1.0, "upright": 0.0, "turn": 1.0,
                    "turn_worst": 1.0, "joint_speed": 1.0, "ends": {}, "score": -1.0, "episodes": 0}
        reasons = rows[:, 1].astype(int)
        stood = reasons == 4
        subset = rows[stood] if stood.any() else rows
        ends = collections.Counter(REASONS[r] for r in reasons)
        metrics = {
            "episodes": int(len(rows)),
            "stand_fraction": float(stood.sum()) / float(envs),
            "mean_seconds": float(rows[:, 5].mean()),
            "sway": float(subset[:, 7].mean()),
            "foot_drift": float(subset[:, 8].mean()),
            "foot_drift_median": float(np.median(subset[:, 8])),
            "foot_drift_worst": float(subset[:, 8].max()),
            "action_rate": float(subset[:, 9].mean()),
            "effort": float(subset[:, 10].mean()),
            "upright": float(subset[:, 12].mean()),
            # O giro do tronco (rad/s, RMS de cada episodio): a media e o pior mundo.
            "turn": float(subset[:, 13].mean()),
            "turn_worst": float(subset[:, 13].max()),
            "joint_speed": float(subset[:, 11].mean()),
            "ends": dict(ends),
        }
        metrics["score"] = score(metrics)
        return metrics
