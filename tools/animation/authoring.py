"""Montagem, validacao e gravacao de um clipe autorado.

Uma animacao e uma funcao do tempo que devolve uma Pose (ver clips/*.py).
Este modulo amostra essa funcao, valida TODOS os quadros e so grava o clipe se
tudo passar. A validacao cobre o que o visualizador de animacao nao mostra
(ele desenha o clipe sem fisica):

  - limites articulares, antes de qualquer clamp;
  - folga de autocolisao (a pose tem de caber no NOSSO corpo);
  - pes: no chao durante o apoio, e parados no MUNDO - pe que desliza no
    apoio vira patinacao no jogo. Clipe de locomocao e in-place: o mundo anda
    para tras na velocidade nominal, e e contra isso que o pe e medido;
  - sola nunca abaixo do chao, em nenhum quadro (o chao responde com
    milhares de newtons);
  - fechamento do loop e maior velocidade angular de junta.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from matter_clip import Clip, save_clip
from matter_rig import REPO_ROOT, Rig, qangle_degrees
from pose import Pose
from validate import (limit_violations, lowest_point, self_collision_gaps)

CLIP_DIRECTORY = REPO_ROOT / "assets" / "animations" / "clips"
FOOT_LINKS = ("LeftFoot", "RightFoot")

# Tolerancias. Cada uma diz o que protege.
GROUND_TOLERANCE_METERS = 0.004        # sola no chao durante o apoio
PLANT_SLIDE_TOLERANCE_METERS = 0.003   # pe de apoio parado no mundo
LOOP_CLOSURE_DEGREES = 0.5             # emenda do loop invisivel
# Nenhuma junta mais rapida que isto. O joelho de um velocista passa de
# 20 rad/s; acima de 30, e salto de pose, nao movimento.
MAXIMUM_JOINT_RATE_RADIANS = 30.0
CLEARANCE_TOLERANCE_METERS = 0.0005    # ruido numerico da folga exigida


@dataclass
class ClipDefinition:
    id: str
    display_name: str
    duration: float
    loops: bool
    pose_at: Callable[[float], Pose]
    # Contato por pe em funcao do tempo (1 = apoio). None = sempre apoiado.
    contact_at: Callable[[float], tuple[float, float]] | None = None
    sample_rate: float = 30.0
    nominal_speed: float = 0.0
    # Direcao de deslocamento em relacao a pelve (0 = frente, 90 = esquerda).
    travel_direction_degrees: float = 0.0
    # Chao em espaco de modelo (z da sola no bind).
    floor_z: float = 0.0
    # Arquivo que descreve o clipe (padrao: clips/<id>.py).
    source: str | None = None


@dataclass
class BuildResult:
    clip: Clip
    report: dict
    frames: list[tuple[float, np.ndarray, np.ndarray]]  # (t, posicoes, orientacoes)


def build_clip(rig: Rig, definition: ClipDefinition) -> BuildResult:
    count = int(round(definition.duration * definition.sample_rate))
    times = np.linspace(0.0, definition.duration, count + 1)
    poses = [definition.pose_at(float(t)) for t in times]
    coordinates = np.stack([p.coordinates for p in poses])
    root_offsets = np.stack([p.root_offset for p in poses])
    root_rotations = np.stack([p.root_rotation for p in poses])
    contacts = np.array([definition.contact_at(float(t)) if definition.contact_at
                         else (1.0, 1.0) for t in times])

    problems: list[str] = []
    residuals: dict[str, float] = {}
    for pose in poses:
        for key, value in pose.residuals.items():
            residuals[key] = max(residuals.get(key, 0.0), value)
    for key, value in residuals.items():
        problems.append(f"solver nao alcancou a geometria pedida em {key} "
                        f"({value:.2f})")

    limit_hits = 0
    worst_clearance: dict[tuple[str, str], tuple[float, float]] = {}
    frames = []
    ground_error = 0.0
    penetration = 0.0
    plant_positions: dict[str, list[np.ndarray]] = {f: [] for f in FOOT_LINKS}
    worst_slide = 0.0
    direction = math.radians(definition.travel_direction_degrees)
    travel_velocity = definition.nominal_speed * np.array(
        [math.cos(direction), math.sin(direction), 0.0])
    for f, t in enumerate(times):
        violations = limit_violations(rig, coordinates[f])
        limit_hits += len(violations)
        for link, axis, excess in violations:
            problems.append(f"t={t:.2f}s {link} eixo {axis} fora do limite "
                            f"em {math.degrees(excess):.2f} graus")
        positions, orientations = rig.forward_kinematics(
            root_offsets[f], root_rotations[f], coordinates[f])
        frames.append((float(t), positions, orientations))
        for a, b, gap, required in self_collision_gaps(rig, positions,
                                                       orientations):
            current = worst_clearance.get((a, b))
            if current is None or gap - required < current[0] - current[1]:
                worst_clearance[(a, b)] = (gap, required)
        for side, foot in enumerate(FOOT_LINKS):
            sole = lowest_point(rig, positions, orientations, [foot])
            penetration = max(penetration, definition.floor_z - sole)
            if contacts[f][side] < 0.5:
                plant_positions[foot] = []
                continue
            ground_error = max(ground_error, abs(sole - definition.floor_z))
            # Posicao no mundo: o clipe e in-place, o corpo anda.
            anchor = positions[rig.index(foot)] + travel_velocity * t
            plant_positions[foot].append(anchor.copy())
            first = plant_positions[foot][0]
            worst_slide = max(worst_slide,
                              float(np.linalg.norm((anchor - first)[:2])))

    for (a, b), (gap, required) in worst_clearance.items():
        if gap < required - CLEARANCE_TOLERANCE_METERS:
            problems.append(f"autocolisao {a} x {b}: folga {gap * 1000:.1f} mm, "
                            f"exigido {required * 1000:.1f} mm")
    if ground_error > GROUND_TOLERANCE_METERS:
        problems.append(f"sola fora do chao no apoio: {ground_error * 1000:.1f} mm")
    if worst_slide > PLANT_SLIDE_TOLERANCE_METERS:
        problems.append(f"pe de apoio deslizou {worst_slide * 1000:.1f} mm")
    if penetration > GROUND_TOLERANCE_METERS:
        problems.append(f"sola abaixo do chao: {penetration * 1000:.1f} mm")

    step = 0.0
    rate = 0.0
    for f in range(1, len(times)):
        delta = float(np.max(np.abs(coordinates[f] - coordinates[f - 1])))
        step = max(step, math.degrees(delta))
        rate = max(rate, delta / max(times[f] - times[f - 1], 1e-6))
    if rate > MAXIMUM_JOINT_RATE_RADIANS:
        problems.append(f"junta a {rate:.1f} rad/s")

    closure = 0.0
    root_closure = 0.0
    if definition.loops:
        closure = math.degrees(float(np.max(np.abs(coordinates[-1]
                                                   - coordinates[0]))))
        root_closure = qangle_degrees(root_rotations[-1], root_rotations[0])
        translation_closure = float(np.linalg.norm(root_offsets[-1]
                                                   - root_offsets[0]))
        if closure > LOOP_CLOSURE_DEGREES:
            problems.append(f"loop nao fecha nas juntas: {closure:.2f} graus")
        if root_closure > LOOP_CLOSURE_DEGREES or translation_closure > 0.001:
            problems.append("loop nao fecha na raiz")

    tightest = sorted(((gap - required, a, b, gap)
                       for (a, b), (gap, required) in worst_clearance.items()))[:4]
    report = {
        "passed": not problems,
        "problems": problems,
        "frames": len(times),
        "limitHitCount": limit_hits,
        "maximumJointStepDegrees": round(step, 3),
        "maximumJointRateRadiansPerSecond": round(rate, 2),
        "loopClosureDegrees": round(closure, 4),
        "rootLoopClosureDegrees": round(root_closure, 4),
        "groundErrorMillimeters": round(ground_error * 1000, 2),
        "plantedFootSlideMillimeters": round(worst_slide * 1000, 2),
        "tightestSelfClearance": [
            {"pair": f"{a} x {b}", "gapMillimeters": round(gap * 1000, 1)}
            for _, a, b, gap in tightest],
    }
    clip = Clip(
        id=definition.id, display_name=definition.display_name,
        target_rig=rig.id, duration=definition.duration,
        sample_rate=definition.sample_rate, loops=definition.loops,
        times=times, root_offsets=root_offsets,
        root_rotations=root_rotations, coordinates=coordinates,
        left_foot_contact=contacts[:, 0], right_foot_contact=contacts[:, 1],
        nominal_speed=definition.nominal_speed,
        travel_direction_degrees=definition.travel_direction_degrees,
        source=definition.source
        or f"tools/animation/clips/{definition.id}.py")
    return BuildResult(clip, report, frames)


def write_clip(rig: Rig, result: BuildResult,
               directory: Path = CLIP_DIRECTORY) -> Path:
    if not result.report["passed"]:
        raise RuntimeError("clipe reprovado; nada foi gravado:\n  "
                           + "\n  ".join(result.report["problems"]))
    path = directory / f"{result.clip.id}.matteranim.json"
    save_clip(result.clip, rig, path, result.report)
    return path
