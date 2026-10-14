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

REASONS = ["nenhum", "queda_contato", "queda_altura", "numerico", "tempo", "limite", "alvo_perdido"]
# As colunas do resumo de um episodio (finished_episodes do modulo nativo).
EPISODE_COLUMNS = 41
FALLS = {1, 2, 3}


BASE_BATTERY = "equilibrio_v1"


def battery_name(posture: str) -> str:
    """A bateria de cada postura (a da base mantem o nome de sempre)."""
    return BASE_BATTERY if posture == "natural" else f"postura_{posture}_v1"


def battery_posture(name: str) -> str:
    """A postura de uma bateria de ficar de pe pelo nome (vazio nas outras)."""
    if name == BASE_BATTERY:
        return "natural"
    if name.startswith("postura_") and name.endswith("_v1"):
        return name.removeprefix("postura_").removesuffix("_v1")
    return ""


def battery_kind(name: str) -> str:
    """O tipo de uma bateria pelo nome (a convencao das tarefas): andar_* e
    girar_* seguem um roteiro de velocidades, ir_ate_* vao a um ponto,
    corrida_*, caminhada_*, direcoes_*, percursos_*, tiro_*, estrela_*, frear_* e curvas_* vao ate
    os alvos da tarefa (o alvo no mundo, com imitacao); giro_* gira no lugar com
    o peito no rumo que gira."""
    if name.startswith(("andar_", "girar_")):
        return "andar"
    if name.startswith("ir_ate_"):
        return "alvo"
    if name.startswith("giro_"):
        return "giro"
    if name.startswith(("corrida_", "caminhada_", "direcoes_", "percursos_", "tiro_", "estrela_", "frear_", "curvas_", "pulos_")):
        return "corrida"
    return "de_pe"


def task_postures(task_path: Path) -> list[str]:
    """As posturas que a tarefa pede, na ordem dela (sem a lista: a base)."""
    import json
    data = json.loads(Path(task_path).read_text())
    return [entry["id"] for entry in data.get("postures", [])] or ["natural"]


def task_batteries(task_path: Path) -> list[dict]:
    """As baterias da tarefa ("evaluation" no arquivo; as desligadas ficam de
    fora). Sem a lista: uma de ficar de pe por postura. Cada uma: name,
    posture, kind (de pe, andar por roteiro, ir ate um ponto), seconds,
    schedule e goal."""
    import json
    data = json.loads(Path(task_path).read_text())
    entries = data.get("evaluation") or [{"name": battery_name(p), "posture": p} for p in task_postures(task_path)]
    batteries = []
    for entry in entries:
        if not entry.get("enabled", True):
            continue
        kind = ("andar" if entry.get("schedule") else "alvo" if entry.get("goal")
                else battery_kind(entry["name"]) if battery_kind(entry["name"]) in ("corrida", "giro") else "de_pe")
        batteries.append(dict(entry, kind=kind, posture=entry.get("posture", "natural")))
    return batteries


@dataclass
class Battery:
    name: str = "equilibrio_v1"
    environments: int = 32
    seconds: float = 60.0
    seed: int = 20261012
    solvers: dict = field(default_factory=lambda: {"treino": (-1, -1), "jogo": (20, 20)})


# A versao da conta da nota: um "melhor" de outra versao nao se compara (2:
# entrou o erro da postura pedida).
SCORE_VERSION = 5


def walking_score(metrics: dict) -> float:
    """Andar: nao cair, acompanhar a velocidade pedida, ir reto, sem balancar
    o tronco para os lados nem tremer."""
    return (metrics["stand_fraction"]
            - 0.40 * min(1.0, metrics.get("velocity_error", 1.0) / 1.0)
            - 0.10 * min(1.0, metrics.get("lateral_drift", 0.0) / 3.0)
            - 0.15 * min(1.0, metrics.get("turn", 0.0) / 0.6)
            - 0.15 * min(1.0, metrics["action_rate"] / 0.05))


def goal_score(metrics: dict) -> float:
    """Ir ate um ponto: chegar perto dele, sem cair e sem tremer."""
    return (metrics["stand_fraction"]
            - 0.50 * min(1.0, metrics.get("goal_error", 10.0) / 2.0)
            - 0.15 * min(1.0, metrics["action_rate"] / 0.05))


def drill_weights(drills: list[dict], scores: dict[str, float], rule: dict) -> list[tuple[str, float, float]]:
    """As fatias dos exercicios pelo exame (a metodologia das 3 areas): o
    exercicio atras do melhor ganha mais mundos (gain por ponto de nota de
    atraso, ate maximumFactor vezes o peso de base); o que alcanca os outros
    volta ao peso de base. scores: a nota de cada exercicio (a pior das baterias
    dele). Devolve (id, peso de base, peso novo) de cada exercicio com nota."""
    if not scores:
        return []
    top = max(scores.values())
    gain = float(rule.get("gain", 10.0))
    cap = max(1.0, float(rule.get("maximumFactor", 3.0)))
    out = []
    for drill in drills:
        if drill["id"] not in scores:
            continue
        base = float(drill.get("baseWeight", drill.get("weight", 1.0)))
        deficit = max(0.0, top - scores[drill["id"]])
        out.append((drill["id"], base, round(base * min(cap, 1.0 + gain * deficit), 4)))
    return out


def race_score(metrics: dict) -> float:
    """Ir ate o alvo: nao cair, chegar, chegar rapido (a velocidade da
    referencia vale tudo: 6 m/s correndo, 1,1 caminhando) e se mover parecido
    com ela (o erro das juntas abaixo de 0,5 rad). Com o comando de velocidade,
    a parte da velocidade e acertar a pedida (o erro abaixo de 1,5 m/s), nao
    correr mais. Com o comando de olhar, o peito fora dele desconta ate 0,25
    (60 graus ou mais). Nas curvas, sair da linha desconta ate 0,20 (1 m ou
    mais)."""
    if metrics.get("speed_error", -1.0) >= 0:
        speed = max(0.0, 1.0 - metrics["speed_error"] / 1.5)
    else:
        speed = min(1.0, metrics.get("target_speed", 0.0) / metrics.get("reference_speed", 6.0))
    return (0.30 * metrics["stand_fraction"]
            + 0.30 * metrics.get("reached_fraction", 0.0)
            + 0.25 * speed
            + 0.15 * max(0.0, 1.0 - metrics.get("imitation_error", 1.0) / 0.5)
            - (0.25 * min(1.0, metrics["facing_error"] / 60.0) if metrics.get("facing_error", -1.0) >= 0 else 0.0)
            - (0.20 * min(1.0, metrics["line_error"] / 1.0) if metrics.get("line_error", -1.0) >= 0 else 0.0)
            - human_penalty(metrics))


def human_penalty(metrics: dict) -> float:
    """O que um humano nao faz: passinhos (um pe que mal sai do chao; 1 por
    segundo ou mais tira 0,15) e pisar rapido demais (acima de 3,5 passos por
    segundo de cada pe; 5 ou mais tira 0,10)."""
    return (0.15 * min(1.0, metrics.get("chatter", 0.0) / 1.0)
            + 0.10 * min(1.0, max(0.0, metrics.get("cadence", 0.0) - 3.5) / 1.5))


def turn_score(metrics: dict) -> float:
    """Girar parado: nao cair, o peito acompanhando o rumo que gira (45 graus
    fora ou mais tiram metade) e a sola sem escorregar (0,3 m/s ou mais tiram
    0,15)."""
    facing = metrics.get("facing_error", -1.0)
    slip = metrics.get("foot_slip", -1.0)
    return (metrics["stand_fraction"]
            - (0.5 * min(1.0, facing / 45.0) if facing >= 0 else 0.5)
            - (0.15 * min(1.0, slip / 0.3) if slip >= 0 else 0.0)
            - human_penalty(metrics))


def score(metrics: dict) -> float:
    """Quanto ficou de pe, descontando oscilacao, tremedeira, deriva, o tronco
    girando (o giro passa despercebido na oscilacao do centro de massa) e o
    quanto se afasta da postura pedida (de pe em outra postura nao conta)."""
    return (metrics["stand_fraction"]
            - 0.15 * min(1.0, metrics["sway"] / 0.05)
            - 0.15 * min(1.0, metrics["action_rate"] / 0.05)
            - 0.10 * min(1.0, metrics["foot_drift"] / 0.20)
            - 0.15 * min(1.0, metrics.get("turn", 0.0) / 0.30)
            - 0.15 * min(1.0, metrics.get("pose_error", 0.0) / 0.15)
            - human_penalty(metrics))


class Evaluator:
    """As baterias da tarefa (cada habilidade aprovada tem a sua: de pe numa
    postura, andar por um roteiro de velocidades, ir ate um ponto), cada uma
    no solver do treino e no do jogo."""

    def __init__(self, native, task_path: Path, threads: int, battery: Battery | None = None,
                 postures: list[str] | None = None, names: list[str] | None = None):
        self.battery = battery or Battery()
        self.native = native
        self.specs = [spec for spec in task_batteries(task_path)
                      if (postures is None or spec["posture"] in postures) and (names is None or spec["name"] in names)]
        if not self.specs:
            raise ValueError("nenhuma bateria com esses filtros")
        self.batches = {}
        for spec in self.specs:
            for solver, (position, velocity) in self.battery.solvers.items():
                self.batches[(spec["name"], solver)] = native.Batch(
                    str(task_path), environments=self.battery.environments, per_scene=4, threads=threads,
                    seed=self.battery.seed, evaluation=True,
                    evaluation_seconds=float(spec.get("seconds", self.battery.seconds)), solver_position=position,
                    solver_velocity=velocity, posture=spec["posture"],
                    schedule=[tuple(float(v) for v in step) for step in spec.get("schedule", [])],
                    facing=int(spec.get("facing", -1)), drill=str(spec.get("drill", "")))
        self.by_name = {spec["name"]: spec for spec in self.specs}

    def observation_names(self) -> list[str]:
        return next(iter(self.batches.values())).observation_names()

    def posture_of(self, battery: str) -> str:
        return self.by_name[battery]["posture"]

    def kind_of(self, battery: str) -> str:
        return self.by_name[battery]["kind"]

    def scored(self, battery: str) -> bool:
        """A bateria entra na nota (com "scored": false, so e medida: uma
        habilidade que o treino de agora nao pratica)."""
        return bool(self.by_name[battery].get("scored", True))

    @torch.no_grad()
    def run(self, model: ActorCritic, device: torch.device, record: Path | None = None,
            watch: tuple[int, ...] = (0,)) -> dict:
        """{bateria: {solver: metricas}}. watch: os mundos gravados inteiros
        (alem das primeiras quedas)."""
        results: dict[str, dict] = {}
        for (name, solver), batch in self.batches.items():
            results.setdefault(name, {})[solver] = self._run_one(
                model, device, batch, record, solver, watch, self.by_name[name])
        return results

    @staticmethod
    def labels(spec: dict, solver: str) -> tuple[str, str]:
        """O sufixo da gravacao e o nome das cenas (a de pe na base mantem os de
        sempre: aval-N-jogo.dump, parado_mundo0_jogo)."""
        if spec["kind"] == "de_pe":
            natural = spec["posture"] == "natural"
            return (solver if natural else f"{spec['posture']}-{solver}"), ("parado" if natural else spec["posture"])
        return f"{spec['name']}-{solver}", spec["name"]

    def _goal_commands(self, batch, goal: dict) -> None:
        """Ir ate um ponto: o alvo fica a distance metros na frente do rumo
        inicial; a velocidade pedida e a do alvo, freando perto dele."""
        positions = batch.root_positions()
        heading = positions[:, 5]
        along = ((positions[:, 0] - positions[:, 3]) * np.cos(heading)
                 + (positions[:, 1] - positions[:, 4]) * np.sin(heading))
        remaining = float(goal["distance"]) - along
        speed = np.clip(1.5 * remaining, 0.0, float(goal.get("speed", 2.0)))
        for e in range(batch.num_envs):
            batch.set_command(e, float(speed[e]) if remaining[e] > 0.3 else 0.0, 0.0, 0.0)

    def _run_one(self, model: ActorCritic, device: torch.device, batch, record: Path | None, solver: str,
                 watch: tuple[int, ...], spec: dict) -> dict:
        envs = batch.num_envs
        dt = batch.policy_dt
        for e in range(envs):
            batch.clear_command(e)
        batch.reset(self.battery.seed)
        first: dict[int, np.ndarray] = {}
        rings = [collections.deque(maxlen=48) for _ in range(envs)]
        watch = tuple(w for w in watch if 0 <= w < envs)
        full: dict[int, list[tuple[float, str]]] = {w: [] for w in watch}
        falls: list[tuple[str, list]] = []
        label, scene = self.labels(spec, solver)
        seconds = float(spec.get("seconds", self.battery.seconds))
        limit = int(math.ceil(seconds / dt)) + 2
        for step in range(limit):
            if spec["kind"] == "alvo":
                self._goal_commands(batch, spec["goal"])
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
                    prefix = "queda" if label == solver else f"queda_{scene}"
                    falls.append((f"{prefix}_mundo{e}_{REASONS[reason]}_{row[5]:.1f}s", list(rings[e])))
            if record is not None and step % 2 == 0:
                for e in range(envs):
                    if e not in first:
                        rings[e].append((t, batch.frame_lines(e)))
                for w in watch:
                    if w not in first:
                        full[w].append((t, rings[w][-1][1]))
            if len(first) >= envs:
                break
        for e in range(envs):
            batch.clear_command(e)
        rows = (np.array([first[e] for e in sorted(first)], dtype=np.float32) if first
                else np.zeros((0, EPISODE_COLUMNS)))
        metrics = self._metrics(rows, envs, spec)
        if record is not None:
            record.parent.mkdir(parents=True, exist_ok=True)
            with open(record.with_name(record.stem + f"-{label}.dump"), "w") as out:
                for name, frames in [(f"{scene}_mundo{w}_{solver}", full[w]) for w in watch] + falls:
                    out.write(f"S {name}\n")
                    for t, lines in frames:
                        out.write(f"F {t:.4f} 0\n")
                        out.write(lines)
        return metrics

    @staticmethod
    def _metrics(rows: np.ndarray, envs: int, spec: dict | None = None) -> dict:
        kind = (spec or {}).get("kind", "de_pe")
        if len(rows) == 0:
            return {"stand_fraction": 0.0, "mean_seconds": 0.0, "sway": 1.0, "foot_drift": 1.0, "foot_drift_median": 1.0,
                    "foot_drift_worst": 1.0, "action_rate": 1.0, "effort": 1.0, "upright": 0.0, "turn": 1.0,
                    "turn_worst": 1.0, "joint_speed": 1.0, "foot_turn": 1.0, "foot_turn_worst": 1.0,
                    "pose_error": 1.0, "velocity_error": 10.0, "lateral_drift": 10.0, "goal_error": 10.0,
                    "targets": 0.0, "reached_fraction": 0.0, "first_target_seconds": -1.0, "target_speed": 0.0,
                    "imitation_error": 1.0, "ends": {}, "kind": kind, "score": -1.0, "episodes": 0}
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
            # O maior giro de um pe no chao (rad): a media e o pior mundo.
            "foot_turn": float(subset[:, 14].mean()),
            "foot_turn_worst": float(subset[:, 14].max()),
            # O quanto as juntas ficam longe da postura pedida (RMS, rad).
            "pose_error": float(subset[:, 15].mean()),
            # Andar (todos os episodios, quedas inclusive): o erro da velocidade e
            # do giro (RMS), a distancia do comeco e o desvio para o lado (m).
            "velocity_error": float(rows[:, 16].mean()),
            "yaw_error": float(rows[:, 17].mean()),
            "distance": float(rows[:, 19].mean()),
            "lateral_drift": float(rows[:, 20].mean()),
            "ends": dict(ends),
            # Os passos (todos os episodios): pisadas por segundo de cada pe e
            # passinhos (pe menos de 0,1 s no ar) por segundo.
            "cadence": float(rows[:, 31].mean()) if rows.shape[1] > 32 else 0.0,
            "chatter": float(rows[:, 32].mean()) if rows.shape[1] > 32 else 0.0,
            # O tremor: a reversao media das juntas por decisao (ir e voltar).
            "reversal": float(rows[:, 33].mean()) if rows.shape[1] > 33 else 0.0,
        }
        if kind == "alvo":
            # O quanto parou longe do ponto (pelo centro de massa no fim).
            target = float(spec["goal"]["distance"])
            along = np.sqrt(np.maximum(rows[:, 19] ** 2 - rows[:, 20] ** 2, 0.0))
            metrics["goal_error"] = float(np.sqrt((target - along) ** 2 + rows[:, 20] ** 2).mean())
        if kind in ("corrida", "giro"):
            # Indo ate o alvo (ou girando no lugar), "sem cair" e nao cair (passar do alvo encerra o
            # episodio sem queda).
            metrics["stand_fraction"] = float((~np.isin(reasons, list(FALLS))).sum()) / float(envs)
            # Os alvos (todos os episodios, quedas inclusive): quantos, a fatia que
            # chegou ao primeiro, quando chegou e a velocidade ate eles; e o erro
            # das juntas contra a referencia (RMS por eixo, rad).
            reached = rows[:, 21] >= 1
            metrics["targets"] = float(rows[:, 21].mean())
            metrics["reached_fraction"] = float(reached.sum()) / float(envs)
            metrics["first_target_seconds"] = float(rows[reached, 22].mean()) if reached.any() else -1.0
            metrics["target_speed"] = float(rows[reached, 23].mean()) if reached.any() else 0.0
            errors = rows[:, 24][rows[:, 24] >= 0]
            metrics["imitation_error"] = float(errors.mean()) if len(errors) else 1.0
            # Com o comando de olhar: o quanto o peito (ou a pelve) ficou fora
            # dele (RMS por episodio, em graus; -1 sem o comando).
            facing = rows[:, 25][rows[:, 25] >= 0]
            metrics["facing_error"] = float(np.degrees(facing.mean())) if len(facing) else -1.0
            # Com o comando de velocidade: o quanto ele ficou longe da pedida de
            # agora (RMS por episodio, m/s; -1 sem o comando).
            speeds = rows[:, 27][rows[:, 27] >= 0]
            metrics["speed_error"] = float(speeds.mean()) if len(speeds) else -1.0
            # A sola escorregando no chao (m/s, a media dos episodios; so medida).
            slips = rows[:, 28][rows[:, 28] >= 0]
            metrics["foot_slip"] = float(slips.mean()) if len(slips) else -1.0
            # Pare aqui: o quanto passou do disco antes de parar (m; so medida).
            overshoots = rows[:, 29][rows[:, 29] >= 0]
            metrics["stop_overshoot"] = float(overshoots.mean()) if len(overshoots) else -1.0
            # As curvas: o quanto saiu da linha dos alvos (RMS, m).
            lines = rows[:, 30][rows[:, 30] >= 0]
            metrics["line_error"] = float(lines.mean()) if len(lines) else -1.0
            # Os cortes: o tempo medio de cada um (s; so medido).
            cuts = rows[:, 34][rows[:, 34] >= 0]
            metrics["cut_seconds"] = float(cuts.mean()) if len(cuts) else -1.0
            # Os pulos (so medidos): as zonas vencidas correndo e paradas, a
            # altura alcancada menos a pedida, as quedas na aterrissagem e a
            # volta a velocidade depois do pulo correndo.
            run = rows[:, 35][rows[:, 35] >= 0]
            stand = rows[:, 36][rows[:, 36] >= 0]
            jumped = (rows[:, 35] >= 0) | (rows[:, 36] >= 0)
            landing = rows[:, 38][rows[:, 38] >= 0]
            recovery = rows[:, 39][rows[:, 39] >= 0]
            metrics["jump_run"] = float(run.mean()) if len(run) else -1.0
            metrics["jump_stand"] = float(stand.mean()) if len(stand) else -1.0
            metrics["jump_rise_error"] = float(rows[jumped, 37].mean()) if jumped.any() else 0.0
            metrics["landing_falls"] = float(landing.mean()) if len(landing) else -1.0
            metrics["jump_recovery"] = float(recovery.mean()) if len(recovery) else -1.0
            # As forcas: o tempo ate voltar, em pe e na pedida (s; so medido).
            pushed = rows[:, 40][rows[:, 40] >= 0]
            metrics["push_recovery"] = float(pushed.mean()) if len(pushed) else -1.0
            metrics["reference_speed"] = float((spec or {}).get("referenceSpeed", 6.0))
        metrics["kind"] = kind
        metrics["score"] = (walking_score(metrics) if kind == "andar" else goal_score(metrics) if kind == "alvo"
                            else race_score(metrics) if kind == "corrida" else turn_score(metrics) if kind == "giro"
                            else score(metrics))
        return metrics
