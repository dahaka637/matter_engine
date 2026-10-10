"""Carrinho (pedido do usuario, 04/10): Soccer Tackle e Running Slide.

Referencias (Mixamo, fornecidas pelo usuario), retargeteadas com
  import_humanoid_animation.py --source-rig mixamo --reference-only
  -> assets/animations/source/retargeted/{soccer_tackle,running_slide}.retarget.json

  - Soccer Tackle (1,77 s): da corrida, firma o pe esquerdo, mergulha de lado
    sobre o quadril esquerdo com a perna direita esticada a frente (o bote),
    o braco esquerdo apoiando atras; desliza quase deitado e levanta rolando
    por cima do joelho esquerdo.
  - Running Slide (1,53 s): da corrida, senta no quadril com a perna esquerda
    esticada a frente e a direita dobrada por baixo, a mao direita apoiando
    atras; desliza e levanta ja correndo, empurrando com a perna dobrada.

O jogo usa o de lado mais devagar e o sentado (que desliza mais) mais
rapido. Limpeza para o nosso corpo:

  - a raiz fica no rumo do primeiro quadro (a direcao do carrinho), COM o
    giro relativo a ele: no carrinho o corpo desliza de lado (~70 graus da
    direcao) - tirando o rumo de cada quadro, como nos levantares, o
    "deitado para tras de lado" virava "inclinado de lado de frente". O
    deslocamento no plano tambem fica nesse rumo: o jogo usa o perfil de
    velocidade do clipe para guiar o deslize;
  - apoio no chao pelos contatos da referencia (pe parado correndo, corpo
    baixo no deslize); entre eles (o voo da corrida) interpolado;
  - tornozelo girado para o pe nao atravessar o chao; maos sobem por IK ate
    encostar;
  - o Soccer Tackle termina na pose de pe limpa; o Running Slide termina
    correndo (a locomocao continua dali).

Uso:
  python3 tools/animation/clips/tackle.py
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
from strafe import mirror_pose  # noqa: E402

SOURCES = g.SOURCE.parent
CLIPS = g.CLIPS
RATE = 30.0
SIGMA = 0.04          # s, suavizacao leve
FOOT_SIGMA = 0.05
END_BLEND = 0.30

VARIANTS = [
    # (fonte, inicio, fim, termina de pe, id, nome, espelhado). So as duas
    # originais (pedido do usuario: as espelhadas ficavam artificiais): o
    # de lado mais devagar, o sentado (que desliza mais) mais rapido.
    ("soccer_tackle", 0.0, 1.767, True, "tackle_soccer", "Carrinho de lado (mais devagar)", False),
    ("running_slide", 0.0, 1.533, False, "tackle_slide", "Carrinho sentado (mais rapido)", False),
]


def heading_of(rotation: np.ndarray) -> float:
    forward = qrotate(rotation, np.array([1.0, 0.0, 0.0]))
    up = qrotate(rotation, np.array([0.0, 0.0, 1.0]))
    planar = forward[:2] - up[:2] * forward[2]
    return math.atan2(planar[1], planar[0])


def tilt_only(rotation: np.ndarray) -> np.ndarray:
    return qnormalize(qmul(qfrom_axis_angle(np.array([0.0, 0.0, 1.0]), -heading_of(rotation)), rotation))


def build(rig: Rig, source: str, start: float, stop: float, ends_standing: bool):
    ref = load_clip(SOURCES / f"{source}.retarget.json", rig)
    old = load_clip(CLIPS / "stand_up_front.matteranim.json", rig)
    standing = old.coordinates[-1].copy()
    standing_rotation = tilt_only(old.root_rotations[-1])
    standing_height = old.root_offsets[-1][2]

    stop = min(stop, ref.duration)
    count = int(math.floor((stop - start) * RATE)) + 1
    heights = np.zeros(count)
    planar = np.zeros((count, 2))
    rotations = np.zeros((count, 4))
    coordinates = np.zeros((count, len(rig.links), 3))
    for f in range(count):
        offset, rotation, c = ref.sample(start + f / RATE, loop=False)
        heights[f] = offset[2]
        planar[f] = offset[:2]
        rotations[f] = rotation
        coordinates[f] = c
    _, first_rotation, _ = ref.sample(start, loop=False)
    yaw0 = heading_of(first_rotation)
    unturn = qfrom_axis_angle(np.array([0.0, 0.0, 1.0]), -yaw0)
    rotations = np.array([qnormalize(qmul(unturn, r)) for r in rotations])
    c0, s0 = math.cos(-yaw0), math.sin(-yaw0)
    planar = planar - planar[0]
    planar = np.stack([planar[:, 0] * c0 - planar[:, 1] * s0, planar[:, 0] * s0 + planar[:, 1] * c0], axis=1)

    sigma = SIGMA * RATE
    planar = g.gaussian(planar, sigma)
    heights = g.gaussian(heights[:, None], sigma)[:, 0]
    for i in range(1, len(rig.links)):
        coordinates[:, i, :] = g.gaussian(coordinates[:, i, :], sigma)
    # Rotacoes: suaviza pela media dos quaternions vizinhos (mesmo hemisferio).
    for f in range(1, count):
        if np.dot(rotations[f], rotations[f - 1]) < 0:
            rotations[f] = -rotations[f]
    rotations = np.array([qnormalize(r) for r in g.gaussian(rotations, sigma)])

    floor = float(rig.links[0].model_position[2]) - rig.standing_root_height
    body = [l.id for l in rig.links if not l.id.endswith("Foot") and not any(a in l.id for a in g.ARMS)]
    no_arms = [l.id for l in rig.links if not any(a in l.id for a in g.ARMS)]

    def offset_of(f):
        return np.array([planar[f][0], planar[f][1], heights[f]])

    # Apoio no chao pelos contatos da referencia (a altura da raiz do
    # retarget deriva - o boneco "afunda" ao longo do clipe): contato e o pe
    # parado no chao (correndo) ou o corpo baixo (no deslize). Nesses quadros
    # o ponto mais baixo (sem os bracos) vai ao chao; entre eles (o voo da
    # corrida) a correcao e interpolada - o voo fica.
    lows = np.zeros(count)
    pelvis_up = np.zeros(count)
    feet_xy = np.zeros((count, 2, 2))
    feet_low = np.zeros((count, 2))
    for f in range(count):
        p, o = rig.forward_kinematics(offset_of(f), rotations[f], coordinates[f])
        lows[f] = lowest_point(rig, p, o, no_arms) - floor
        pelvis_up[f] = p[0][2] - floor - lows[f]
        for k, side in enumerate(("Left", "Right")):
            i = rig.index(f"{side}Foot")
            feet_xy[f, k] = p[i][:2]
            feet_low[f, k] = lowest_point(rig, p, o, [f"{side}Foot"]) - floor
    contact = np.zeros(count, dtype=bool)
    for f in range(count):
        a, b = max(0, f - 1), min(count - 1, f + 1)
        for k in range(2):
            speed = np.linalg.norm(feet_xy[b, k] - feet_xy[a, k]) / ((b - a) / RATE)
            if speed < 2.5 and feet_low[f, k] - lows[f] < 0.05:
                contact[f] = True
        if pelvis_up[f] < 0.40:
            contact[f] = True
    frames = np.nonzero(contact)[0]
    ground = np.interp(np.arange(count), frames, lows[frames]) if len(frames) else np.zeros(count)
    heights = heights - g.gaussian(ground[:, None], 0.03 * RATE)[:, 0]

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
    # Nada atravessa o chao (sem os bracos): sobe o que ainda atravessar.
    lift = np.zeros(count)
    for f in range(count):
        p, o = rig.forward_kinematics(offset_of(f), rotations[f], coordinates[f])
        lift[f] = max(0.0, floor - lowest_point(rig, p, o, no_arms))
    heights = heights + g.gaussian(lift[:, None], 0.04 * RATE)[:, 0]

    # Maos: sobem por IK ate encostar.
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
        for name in g.ARMS:
            i = rig.index(f"{side}{name}")
            coordinates[:, i, :] = g.gaussian(coordinates[:, i, :], 0.03 * RATE)

    if ends_standing:
        # De pe, no rumo em que ele terminou.
        end_yaw = heading_of(rotations[-1])
        standing_rotation = qnormalize(qmul(qfrom_axis_angle(np.array([0.0, 0.0, 1.0]), end_yaw), standing_rotation))
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
    return count, heights, planar, rotations, coordinates, floor


def write(rig: Rig, variant_id: str, name: str, source: str, count, heights, planar, rotations, coordinates, floor):
    offsets = [np.array([planar[f][0], planar[f][1], heights[f]]) for f in range(count)]
    windows = {}
    for side in ("Left", "Right"):
        down = []
        for f in range(count):
            p, o = rig.forward_kinematics(offsets[f], rotations[f], coordinates[f])
            if lowest_point(rig, p, o, [f"{side}Hand"]) - floor < 0.06:
                down.append(f)
        windows[side] = (down[0], down[-1] + 1) if down else (0, 0)
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
    travel = float(np.hypot(*(planar[-1] - planar[0])))
    clip = {
        "schema": template["schema"],
        "id": variant_id,
        "displayName": name,
        "targetRigId": template["targetRigId"],
        "sourceAssetPath": f"{source} (FBX do usuario, retargeteado; tools/animation/clips/tackle.py)",
        "durationSeconds": round(duration, 7),
        "sourceSampleRateHz": RATE,
        "loops": False,
        "sourceRootDisplacementMeters": [round(float(planar[-1][0]), 5), round(float(planar[-1][1]), 5), 0.0],
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
    out = CLIPS / f"{variant_id}.matteranim.json"
    out.write_text(json.dumps(clip, indent=1), encoding="utf-8")
    print(f"{out.name}: {count} quadros, {duration:.2f} s, percurso {travel:.2f} m; salto max {max_step:.1f} graus")


def main() -> None:
    rig = Rig(g.PROFILE)
    cache = {}
    for source, start, stop, ends_standing, variant_id, name, mirrored in VARIANTS:
        key = (source, start, stop, ends_standing)
        if key not in cache:
            cache[key] = build(rig, source, start, stop, ends_standing)
        count, heights, planar, rotations, coordinates, floor = cache[key]
        if mirrored:
            m_heights, m_planar, m_rotations, m_coordinates = heights.copy(), planar.copy(), rotations.copy(), coordinates.copy()
            for f in range(count):
                offset, rotation, c = mirror_pose(rig, np.array([planar[f][0], planar[f][1], heights[f]]),
                                                  rotations[f], coordinates[f])
                m_planar[f] = offset[:2]
                m_heights[f] = offset[2]
                m_rotations[f] = rotation
                m_coordinates[f] = c
            write(rig, variant_id, name, source, count, m_heights, m_planar, m_rotations, m_coordinates, floor)
        else:
            write(rig, variant_id, name, source, count, heights, planar, rotations, coordinates, floor)


if __name__ == "__main__":
    main()
