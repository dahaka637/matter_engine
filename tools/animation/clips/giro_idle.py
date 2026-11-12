"""O giro parado do modo padrao, do IDLE ALERTA, como clipe (04/11, pedido do usuario: "o giro parado deveria ser feito do
IDLE alerta, e ja temos os movimentos de giro do footplanner, que ele DEVERIA usar como referencia, e nao esse atual
inventado" - o turn_left/turn_right da Mixamo).

A fonte e o proprio modo padrao: o lider do rastreador (TrackingLeader3D, o DefaultLocomotion3D com o StandTurnFeet3D)
parado no idle alerta e girando a vista a 90 graus/s, gravado a 120 Hz pelo teste do laboratorio:

  MATTER_RL_TEST_GIRO=build-profile/tmp/giro/captura_90.jsonl MATTER_RL_TEST_GIRO_TAXA=90 \\
      build-profile/MatterReinforcementLearningScreenTests          (e -90 para a direita)
  python3 tools/animation/clips/giro_idle.py build-profile/tmp/giro/captura_90.jsonl build-profile/tmp/giro/captura_-90.jsonl

Daqui: um ciclo do regime (os dois passos, o periodo pelos pousos do pe esquerdo), a 30 Hz, com o rumo do comeco tirado (o
clipe gira de 0 ao giro do ciclo, turnCycleDegrees, e se encadeia), a raiz no lugar (a media do ciclo no plano tirada), as
pontas emendadas (a respiracao do idle nao fecha num ciclo de 1 s) e o apoio de cada pe pela sola: perto do chao e quase
parado.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from matter_clip import Clip, save_clip  # noqa: E402
from matter_rig import REPO_ROOT, load_default_character, qconj, qfrom_axis_angle, qmul, qnormalize, qrotate  # noqa: E402

CLIPS = REPO_ROOT / "assets" / "animations" / "clips"
RATE = 30.0
# O pe apoiado: a sola a ate isto acima do chao dela (m) e andando a menos disto (m/s) - o pe que gira sobre a bola desliza.
CONTACT_HEIGHT = 0.015
CONTACT_SPEED = 0.35
# A emenda: os ultimos quadros vao ao primeiro (girado) nesta fracao do ciclo.
SEAM = 0.15


def yaw_of(q: np.ndarray) -> float:
    forward = qrotate(q, np.array([1.0, 0.0, 0.0]))
    return math.atan2(forward[1], forward[0])


def cycle(path: Path, rig):
    lines = [json.loads(line) for line in path.read_text().splitlines()]
    head, rows = lines[0], lines[1:]
    ids = head["links"]
    order = [ids.index(link.id) for link in rig.links]
    t = np.array([r["t"] for r in rows])
    feet = np.array([r["pes"] for r in rows])
    ground = feet[:, :, 2].min()
    air = (feet[:, :, 2] - ground > CONTACT_HEIGHT) | (np.hypot(feet[:, :, 3], feet[:, :, 4]) > CONTACT_SPEED)
    # O periodo: o do regime (depois de 4 s), o atraso em que a pose das pernas mais se repete (0,6 a 1,6 s); o comeco, o
    # instante do primeiro periodo com mais apoio dos dois pes (a emenda num momento quieto).
    coords_all = np.array([r["coordenadas"] for r in rows])
    legs = [i for i, name in enumerate(ids) if any(k in name for k in ("Thigh", "Shin", "Foot", "Toe"))]
    dt = float(np.median(np.diff(t)))
    base = np.searchsorted(t, 4.0)
    window = int(round(1.5 / dt))
    best, period = 1e9, 1.0
    for lag in range(int(0.6 / dt), int(1.6 / dt)):
        if base + window + lag >= len(t):
            break
        err = np.abs(coords_all[base:base + window, legs] - coords_all[base + lag:base + window + lag, legs]).mean()
        if err < best:
            best, period = err, lag * dt
    span = np.arange(base, base + int(round(period / dt)))
    planted = (~air[span]).sum(axis=1)
    start = float(t[span[int(np.argmax(planted + 1e-3 * np.arange(len(span)) * 0))]])
    pelvis = rig.links[0]
    model = pelvis.model_orientation
    deltas = np.array([r["delta"] for r in rows])
    worlds = np.array([qnormalize(qmul(model, d)) for d in deltas])
    yaws = np.unwrap([yaw_of(q) for q in worlds])
    roots = np.array([r["deslocamento"] for r in rows]) + pelvis.model_position
    coords = np.array([r["coordenadas"] for r in rows])[:, order]

    def at(time: float, values: np.ndarray) -> np.ndarray:
        k = np.clip(np.searchsorted(t, time) - 1, 0, len(t) - 2)
        a = (time - t[k]) / (t[k + 1] - t[k])
        return values[k] * (1 - a) + values[k + 1] * a

    frames = int(round(period * RATE))
    times = start + np.arange(frames + 1) * period / frames
    yaw0 = float(at(start, yaws))
    turn = float(at(start + period, yaws)) - yaw0
    undo = qfrom_axis_angle([0.0, 0.0, 1.0], -yaw0)
    world = np.array([at(x, roots) for x in times])
    # (O centro do giro e a capsula, na origem: a pelve do idle fica ~5 cm fora dela e roda em volta.)
    center = np.zeros(2)
    offsets, rotations = [], []
    for x, r in zip(times, world):
        k = np.clip(np.searchsorted(t, x) - 1, 0, len(t) - 2)
        a = (x - t[k]) / (t[k + 1] - t[k])
        q0, q1 = worlds[k], worlds[k + 1]
        if np.dot(q0, q1) < 0:
            q1 = -q1
        q = qnormalize(q0 * (1 - a) + q1 * a)
        local = qrotate(undo, np.array([r[0] - center[0], r[1] - center[1], r[2]]))
        offsets.append(local - pelvis.model_position)
        rotations.append(qnormalize(qmul(qconj(model), qmul(undo, q))))
    offsets, rotations = np.array(offsets), np.array(rotations)
    coordinates = np.array([at(x, coords) for x in times])
    contact = np.array([[not at(x, air[:, s].astype(float)) >= 0.5 for s in (0, 1)] for x in times], dtype=float)
    # A emenda: o fim vai ao comeco (girado pelo giro do ciclo, so a rotacao da raiz - o deslocamento e as juntas iguais).
    spin = qfrom_axis_angle([0.0, 0.0, 1.0], turn)
    first_rotation = qnormalize(qmul(qconj(model), qmul(spin, qmul(model, rotations[0]))))
    first_offset = qrotate(spin, offsets[0] + pelvis.model_position) - pelvis.model_position
    seam_before = (np.degrees(np.abs(coordinates[-1] - coordinates[0])).max(),
                   float(np.linalg.norm(offsets[-1] - first_offset)))
    n = len(times)
    for i in range(n):
        u = (i / (n - 1) - (1 - SEAM)) / SEAM
        if u <= 0:
            continue
        w = u * u * (3 - 2 * u)
        coordinates[i] = coordinates[i] * (1 - w) + coordinates[0] * w
        offsets[i] = offsets[i] * (1 - w) + first_offset * w
        r0, r1 = rotations[i], first_rotation
        if np.dot(r0, r1) < 0:
            r1 = -r1
        rotations[i] = qnormalize(r0 * (1 - w) + r1 * w)
    limit_hits = 0
    for f in range(n):
        for i in range(1, len(rig.links)):
            clamped = rig.clamp(i, coordinates[f, i])
            if np.abs(clamped - coordinates[f, i]).max() > 1e-4:
                limit_hits += 1
                coordinates[f, i] = clamped
    steps = np.degrees(np.abs(np.diff(coordinates, axis=0))).max()
    report = {"passed": True, "maximumJointStepDegrees": float(steps), "limitHitCount": int(limit_hits),
              "source": "captura do modo padrao (TrackingLeader3D + StandTurnFeet3D)", "periodSeconds": period,
              "turnCycleDegrees": math.degrees(turn), "seamJointDegreesBefore": float(seam_before[0]),
              "seamRootMetersBefore": seam_before[1]}
    return period, turn, offsets, rotations, coordinates, contact, report, head["taxa"]


def main() -> int:
    rig, _ = load_default_character()
    for arg in sys.argv[1:]:
        period, turn, offsets, rotations, coordinates, contact, report, rate = cycle(Path(arg), rig)
        left = rate > 0
        clip_id = "giro_idle_esquerda" if left else "giro_idle_direita"
        clip = Clip(
            id=clip_id, display_name=f"Giro parado do idle alerta para a {'esquerda' if left else 'direita'} (modo padrao)",
            target_rig="FootballPlayerV1", duration=period, sample_rate=RATE, loops=True,
            times=np.arange(len(offsets)) * period / (len(offsets) - 1), root_offsets=offsets, root_rotations=rotations,
            coordinates=coordinates, left_foot_contact=contact[:, 0], right_foot_contact=contact[:, 1],
            source="tools/animation/clips/giro_idle.py (o modo padrao: TrackingLeader3D + StandTurnFeet3D, 90 graus/s)")
        path = CLIPS / f"{clip_id}.matteranim.json"
        save_clip(clip, rig, path, report)
        data = json.loads(path.read_text())
        ordered = {}
        for key, value in data.items():
            ordered[key] = value
            if key == "loops":
                ordered["turnCycleDegrees"] = round(math.degrees(turn), 4)
        path.write_text(json.dumps(ordered, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"{path.name}: {period:.3f} s, gira {math.degrees(turn):+.1f} graus por ciclo, passo maximo"
              f" {report['maximumJointStepDegrees']:.1f} graus/quadro, limites {report['limitHitCount']},"
              f" emenda antes {report['seamJointDegreesBefore']:.1f} graus / {100 * report['seamRootMetersBefore']:.1f} cm,"
              f" apoio E {contact[:, 0].mean():.0%} D {contact[:, 1].mean():.0%}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
