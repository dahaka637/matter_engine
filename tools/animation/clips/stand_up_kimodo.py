"""Levantar do chao gerado no Kimodo (NVIDIA, 04/10): de bruços e de costas.

A geracao (texto + poses-chave de corpo inteiro: deitado no quadro 0, de pe
no ultimo) sai do Kimodo em BVH (esqueleto SOMA) e e retargeteada pelo
import_humanoid_animation.py (--source-rig soma --reference-only). Aqui so a
limpeza para o nosso corpo:

  - corta o comeco parado (o Kimodo segura a pose deitada ~1 s);
  - raiz so com inclinacao e altura (o rumo de cada quadro sai, como no
    runtime) e o deslocamento no plano no rumo do primeiro quadro;
  - o tornozelo gira para o pe nao atravessar o chao (o nosso estica menos);
  - cada quadro apoiado no chao (deitado pelo tronco e pernas, de pe pelos
    pes);
  - o fim mistura para a pose de pe limpa e grava os canais de apoio.

Uso:
  python3 tools/animation/clips/stand_up_kimodo.py front
  python3 tools/animation/clips/stand_up_kimodo.py back
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from matter_clip import load_clip  # noqa: E402
from matter_rig import Rig, qfrom_axis_angle, qmul, qnormalize, qrotate  # noqa: E402
from validate import lowest_point  # noqa: E402

import stand_up_getup as g  # noqa: E402

SOURCES = g.SOURCE.parent
CLIPS = g.CLIPS
RATE = 30.0
END_BLEND = 0.35
SIGMA = 0.05        # s, suavizacao leve (o Kimodo ja sai limpo)
FOOT_SIGMA = 0.06
GROUND_SIGMA = 0.05
ARMS = g.ARMS

VARIANTS = {
    "front": dict(source="kimodo_getup_front.retarget.json", id="stand_up_front_kimodo",
                  name="Levantar de bruços (Kimodo)", start=0.85),
    "back": dict(source="kimodo_getup_back.retarget.json", id="stand_up_back_kimodo",
                 name="Levantar de costas (Kimodo)", start=0.85),
}


def heading_of(rotation: np.ndarray) -> float:
    """Rumo da raiz estavel deitado de bruços E de costas (o mesmo do
    runtime): frente + cima * (-frente.z). Em pe e a frente; de bruços a
    frente aponta para o chao e o cima para a cabeca; de costas a frente
    aponta para o ceu e o -cima para os pes - nos dois, o rumo e para onde
    ele vai ficar de frente ao levantar (passando de 90 graus nao vira)."""
    forward = qrotate(rotation, np.array([1.0, 0.0, 0.0]))
    up = qrotate(rotation, np.array([0.0, 0.0, 1.0]))
    planar = forward[:2] - up[:2] * forward[2]
    return math.atan2(planar[1], planar[0])


def tilt_only(rotation: np.ndarray) -> np.ndarray:
    return qnormalize(qmul(qfrom_axis_angle(np.array([0.0, 0.0, 1.0]), -heading_of(rotation)), rotation))


def main() -> None:
    variant = VARIANTS[sys.argv[1] if len(sys.argv) > 1 else "front"]
    rig = Rig(g.PROFILE)
    ref = load_clip(SOURCES / variant["source"], rig)
    old = load_clip(CLIPS / "stand_up_front.matteranim.json", rig)
    standing = old.coordinates[-1].copy()
    standing_rotation = tilt_only(old.root_rotations[-1])
    standing_height = old.root_offsets[-1][2]

    start = variant["start"]
    count = int(math.floor((ref.duration - start) * RATE)) + 1
    times = np.array([start + f / RATE for f in range(count)])
    heights = np.zeros(count)
    planar = np.zeros((count, 2))
    pitches = np.zeros(count)
    rolls = np.zeros(count)
    coordinates = np.zeros((count, len(rig.links), 3))
    for f, t in enumerate(times):
        offset, rotation, c = ref.sample(float(t), loop=False)
        heights[f] = offset[2]
        planar[f] = offset[:2]
        pitches[f], rolls[f] = g.pitch_roll(tilt_only(rotation))
        coordinates[f] = c
    pitches = np.unwrap(pitches)
    rolls = np.unwrap(rolls)
    _, first_rotation, _ = ref.sample(float(times[0]), loop=False)
    yaw0 = heading_of(first_rotation)
    c0, s0 = math.cos(-yaw0), math.sin(-yaw0)
    planar = planar - planar[0]
    planar = np.stack([planar[:, 0] * c0 - planar[:, 1] * s0, planar[:, 0] * s0 + planar[:, 1] * c0], axis=1)

    sigma = SIGMA * RATE
    planar = g.gaussian(planar, sigma)
    heights = g.gaussian(heights[:, None], sigma)[:, 0]
    pitches = g.gaussian(pitches[:, None], sigma)[:, 0]
    rolls = g.gaussian(rolls[:, None], sigma)[:, 0]
    for i in range(1, len(rig.links)):
        coordinates[:, i, :] = g.gaussian(coordinates[:, i, :], sigma)
    rotations = [g.from_pitch_roll(p, r) for p, r in zip(pitches, rolls)]

    floor = float(rig.links[0].model_position[2]) - rig.standing_root_height
    body = [l.id for l in rig.links if not l.id.endswith("Foot") and not any(a in l.id for a in ARMS)]
    no_arms = [l.id for l in rig.links if not any(a in l.id for a in ARMS)]

    def offset_of(f):
        return np.array([planar[f][0], planar[f][1], heights[f]])

    # Pes: o tornozelo gira (nos limites) para o pe atravessar o minimo.
    for f in range(count):
        for side in ("Left", "Right"):
            foot = rig.index(f"{side}Foot")
            axis = rig.links[foot].axes[0]
            base = coordinates[f][foot].copy()
            best = None
            for k in range(13):
                trial = base.copy()
                trial[0] = axis.minimum + (axis.maximum - axis.minimum) * k / 12.0
                coordinates[f][foot] = trial
                p, o = rig.forward_kinematics(offset_of(f), rotations[f], coordinates[f])
                score = min(0.0, lowest_point(rig, p, o, [f"{side}Foot"]) - min(floor, lowest_point(rig, p, o, body)))
                value = score - abs(trial[0] - base[0]) * 0.01
                if best is None or value > best[0]:
                    best = (value, trial)
            coordinates[f][foot] = best[1]
    for side in ("Left", "Right"):
        foot = rig.index(f"{side}Foot")
        coordinates[:, foot, :] = g.gaussian(coordinates[:, foot, :], FOOT_SIGMA * RATE)

    # Cada quadro no chao: o ponto mais baixo (sem os bracos) no chao.
    lift = np.zeros(count)
    for f in range(count):
        p, o = rig.forward_kinematics(offset_of(f), rotations[f], coordinates[f])
        lift[f] = floor - lowest_point(rig, p, o, no_arms)
    heights = heights + g.gaussian(lift[:, None], GROUND_SIGMA * RATE)[:, 0]

    # Maos: os bracos do nosso corpo tem outra proporcao - apoiando, a mao
    # atravessava o chao (ate ~9 cm). O punho sobe (IK no braco) ate a mao
    # encostar.
    for f in range(count):
        for side in ("Left", "Right"):
            for _ in range(3):
                p, o = rig.forward_kinematics(offset_of(f), rotations[f], coordinates[f])
                gap = lowest_point(rig, p, o, [f"{side}Hand"]) - floor
                if gap > -0.005:
                    break
                target = g.wrist_world(rig, p, o, side) + np.array([0.0, 0.0, -gap])
                coordinates[f] = g.solve_arm(rig, offset_of(f), rotations[f], coordinates[f], side, target)
    for side in ("Left", "Right"):
        for name in ARMS:
            i = rig.index(f"{side}{name}")
            coordinates[:, i, :] = g.gaussian(coordinates[:, i, :], 0.03 * RATE)

    # Fim: a pose de pe limpa.
    blend = int(round(END_BLEND * RATE))
    for k in range(blend + 1):
        f = count - 1 - blend + k
        w = g.smooth(k / blend)
        coordinates[f] = coordinates[f] * (1 - w) + standing * w
        heights[f] = heights[f] * (1 - w) + standing_height * w
        r1 = standing_rotation if np.dot(rotations[f], standing_rotation) >= 0 else -standing_rotation
        rotations[f] = qnormalize(rotations[f] * (1 - w) + r1 * w)
    for f in range(count):
        for i in range(1, len(rig.links)):
            coordinates[f][i] = rig.clamp(i, coordinates[f][i])

    # Maos: apoio pela geometria (a mao encostando no chao).
    windows = {}
    hand_low = {}
    for side in ("Left", "Right"):
        down = []
        lows = []
        for f in range(count):
            p, o = rig.forward_kinematics(offset_of(f), rotations[f], coordinates[f])
            low = lowest_point(rig, p, o, [f"{side}Hand"]) - floor
            lows.append(low)
            if low < 0.06:
                down.append(f)
        windows[side] = (down[0], down[-1] + 1) if down else (0, 0)
        hand_low[side] = min(lows)
    offsets = [offset_of(f) for f in range(count)]
    contacts = g.support_channels(rig, offsets, rotations, coordinates, floor, windows, RATE)

    steps = np.abs(np.diff(coordinates[:, 1:, :], axis=0))
    max_step = math.degrees(float(steps.max()))
    template = json.loads((CLIPS / "stand_up_front.matteranim.json").read_text(encoding="utf-8"))
    tracks = []
    for i, link in enumerate(rig.links):
        if i == 0:
            keys = [{"timeSeconds": round(f / RATE, 7),
                     "rootTranslationOffsetMeters": [round(float(planar[f][0]), 7), round(float(planar[f][1]), 7),
                                                     round(float(heights[f]), 7)],
                     "rootRotationDelta": [round(float(x), 8) for x in rotations[f]]}
                    for f in range(count)]
            tracks.append({"targetLinkId": link.id, "space": "root", "keyframes": keys})
            continue
        keys = [{"timeSeconds": round(f / RATE, 7),
                 "jointPositionRadians": [round(float(x), 7) for x in coordinates[f][i]]}
                for f in range(count)]
        tracks.append({"targetLinkId": link.id, "space": "joint", "keyframes": keys})
    duration = (count - 1) / RATE
    clip = {
        "schema": template["schema"],
        "id": variant["id"],
        "displayName": variant["name"],
        "targetRigId": template["targetRigId"],
        "sourceAssetPath": "Kimodo-SOMA-RP-v1.1 (texto + poses-chave), BVH retargeteado",
        "durationSeconds": round(duration, 7),
        "sourceSampleRateHz": RATE,
        "loops": False,
        "sourceRootDisplacementMeters": [0.0, 0.0, 0.0],
        "nominalSpeedMetersPerSecond": 0.0,
        "contacts": contacts,
        "rootMotionPolicy": "preserve",
        "retargetReport": {
            "origin": "authored",
            "passed": max_step < 60.0,
            "directionRmsDegrees": 0.0,
            "directionMaxDegrees": 0.0,
            "limbDirectionMaxDegrees": 0.0,
            "maximumJointStepDegrees": round(max_step, 3),
            "limitHitCount": 0,
        },
        "tracks": tracks,
    }
    out = CLIPS / f"{variant['id']}.matteranim.json"
    out.write_text(json.dumps(clip, indent=1), encoding="utf-8")
    print(f"{out.name}: {count} quadros, {duration:.2f} s; salto max {max_step:.1f} graus; "
          f"maos no chao: " + ", ".join(f"{s} {a / RATE:.2f}-{b / RATE:.2f} s (mais baixo {hand_low[s]*100:+.1f} cm)"
                                        for s, (a, b) in windows.items()))


if __name__ == "__main__":
    main()
