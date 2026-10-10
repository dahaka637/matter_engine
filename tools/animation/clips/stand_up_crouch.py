"""Levantar de bruços pelo agachado (referencias do usuario, 04/10): Prone To
Crouch Transition (de bruços ao ajoelhado num joelho) e Crouch To Stand (do
ajoelhado em pe). As duas sao Mixamo de fuzil e encaixam: a primeira termina
na pose em que a segunda comeca (pe esquerdo plantado a frente, joelho
direito no chao).

Na referencia as maos estao no fuzil e o quadril sobe de bruços sem apoio
nenhum - no corpo fisico isso nao acontece. Aqui:

  - deitado -> ajoelhado mais lento, comecando na posicao de flexao: as maos
    ao lado do peito, no chao, empurrando (IK) enquanto o tronco sobe e a
    perna esquerda passa para a frente; soltam quando o braco nao alcanca
    mais o chao (o tronco ja quase em pe) e vao a frente, perto dos joelhos,
    equilibrando;
  - ajoelhado -> em pe com os bracos relaxando ao subir;
  - a mesma limpeza do levantar pelo Getting Up (stand_up_getup.py): raiz so
    com inclinacao e altura, coluna suave, cada quadro apoiado no chao e o
    tornozelo sem atravessar o chao.

Uso:
  python3 tools/animation/clips/stand_up_crouch.py
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
from matter_rig import REPO_ROOT, Rig, qnormalize, qrotate  # noqa: E402
from validate import lowest_point  # noqa: E402

import stand_up_getup as g  # noqa: E402

SOURCES = REPO_ROOT / "assets" / "animations" / "source" / "retargeted"
CLIPS = REPO_ROOT / "assets" / "animations" / "clips"
RATE = 30.0
A_END = 1.15        # s da Prone To Crouch: ajoelhado (o resto e acomodar)
A_SCALE = 1.7       # mais lento: no corpo fisico o tronco sobe pelos bracos
JOIN = 0.35         # s misturando o ajoelhado de uma no da outra
B_END = 2.3         # s da Crouch To Stand: ja de pe
B_SCALE = 1.1
END_BLEND = 0.4
HAND_FORWARD = 0.10   # mao a frente do ombro, de bruços (posicao de flexao)
HAND_OUTWARD = 0.13   # e para fora
CROUCH_REACH = np.array([0.28, 0.10, -0.42])  # punho a partir do ombro, agachado
ARM_SWAP = 0.35
HANDS_UNTIL = 2.0     # s: maos no chao ate o ajoelhado (pe da frente e joelho de tras)
MAX_BEND = 1.6        # rad de flexao a mais na coluna inteira, no maximo
TRUNK_RISE = 0.5      # s do tronco subindo depois que as maos soltam
STANDING_HEIGHT = 0.72
ROLL_SHARE = 0.3
SPINE_SIDE_SHARE = 0.5
SIGMA = 0.10
FOOT_SIGMA = 0.10
ARM_SIGMA = 0.05
ARMS = g.ARMS
SPINE = g.SPINE


def main() -> None:
    rig = Rig(g.PROFILE)
    prone = load_clip(SOURCES / "prone_to_crouch.retarget.json", rig)
    stand = load_clip(SOURCES / "crouch_to_stand.retarget.json", rig)
    old = load_clip(CLIPS / "stand_up_front.matteranim.json", rig)
    standing = old.coordinates[-1].copy()
    standing_rotation = g.tilt_only(old.root_rotations[-1])
    standing_height = old.root_offsets[-1][2]

    heights, pitches, rolls, poses, planar = [], [], [], [], []
    frame_of = {}

    def add(offset, rotation, coordinates, source=None):
        heights.append(offset[2])
        # No plano: no rumo do primeiro quadro de cada referencia, emendado
        # onde a anterior parou (so define quem fica parado no chao).
        key = id(source)
        if key not in frame_of:
            base = planar[-1] if planar else np.zeros(2)
            frame_of[key] = (offset[:2].copy(), g.heading_of(rotation), base)
        origin, yaw0, base = frame_of[key]
        d = offset[:2] - origin
        c0, s0 = math.cos(-yaw0), math.sin(-yaw0)
        planar.append(base + np.array([d[0] * c0 - d[1] * s0, d[0] * s0 + d[1] * c0]))
        p, r = g.pitch_roll(g.tilt_only(rotation))
        pitches.append(p)
        rolls.append(r)
        poses.append(coordinates.copy())

    for f in range(int(round(A_END * A_SCALE * RATE)) + 1):
        add(*prone.sample(f / RATE / A_SCALE, loop=False), source=prone)
    a_count = len(poses)
    a_end = (heights[-1], pitches[-1], rolls[-1], poses[-1].copy())
    b0 = stand.sample(0.0, loop=False)
    b0_pitch, b0_roll = g.pitch_roll(g.tilt_only(b0[1]))
    join = int(round(JOIN * RATE))
    for f in range(1, join + 1):
        w = g.smooth(f / join)
        heights.append(a_end[0] * (1 - w) + b0[0][2] * w)
        pitches.append(a_end[1] * (1 - w) + b0_pitch * w)
        rolls.append(a_end[2] * (1 - w) + b0_roll * w)
        poses.append(a_end[3] * (1 - w) + b0[2] * w)
        planar.append(planar[-1].copy())
    for f in range(1, int(round(B_END * B_SCALE * RATE)) + 1):
        add(*stand.sample(f / RATE / B_SCALE, loop=False), source=stand)
    count = len(poses)
    heights = np.array(heights)
    pitches = np.unwrap(np.array(pitches))
    rolls = np.array(rolls)
    coordinates = np.array(poses)
    planar = g.gaussian(np.array(planar), SIGMA * RATE)

    sigma = SIGMA * RATE
    heights = g.gaussian(heights[:, None], sigma)[:, 0]
    pitches = np.maximum(g.gaussian(pitches[:, None], sigma)[:, 0], 0.0)
    rolls = g.gaussian(rolls[:, None], sigma)[:, 0] * ROLL_SHARE
    for name in SPINE:
        i = rig.index(name)
        track = g.gaussian(coordinates[:, i, :], sigma)
        for axis in (0, 2):
            track[:, axis] *= SPINE_SIDE_SHARE
        coordinates[:, i, :] = track
    for side in ("Left", "Right"):
        for name in ("Thigh", "Shin"):
            i = rig.index(f"{side}{name}")
            coordinates[:, i, :] = g.gaussian(coordinates[:, i, :], 0.04 * RATE)
    rotations = [g.from_pitch_roll(p, r) for p, r in zip(pitches, rolls)]

    floor = float(rig.links[0].model_position[2]) - rig.standing_root_height
    body = [l.id for l in rig.links if not l.id.endswith("Foot") and not any(a in l.id for a in ARMS)]
    no_arms = [l.id for l in rig.links if not any(a in l.id for a in ARMS)]

    # Pes: o tornozelo gira para o pe atravessar o minimo (o nosso estica
    # menos que o da referencia).
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
                p, o = rig.forward_kinematics(np.array([planar[f][0], planar[f][1], heights[f]]), rotations[f], coordinates[f])
                score = min(0.0, lowest_point(rig, p, o, [f"{side}Foot"]) - lowest_point(rig, p, o, body))
                value = score - abs(trial[0] - base[0]) * 0.01
                if best is None or value > best[0]:
                    best = (value, trial)
            coordinates[f][foot] = best[1]
    for side in ("Left", "Right"):
        foot = rig.index(f"{side}Foot")
        coordinates[:, foot, :] = g.gaussian(coordinates[:, foot, :], FOOT_SIGMA * RATE)

    # Cada quadro apoiado no chao: deitado/ajoelhado pelo tronco e pernas, de
    # pe pelos pes (sem os bracos: eles vem depois, por IK).
    lift = np.zeros(count)
    for f in range(count):
        p, o = rig.forward_kinematics(np.array([planar[f][0], planar[f][1], heights[f]]), rotations[f], coordinates[f])
        up = qrotate(rotations[f], np.array([0.0, 0.0, 1.0]))
        lift[f] = floor - lowest_point(rig, p, o, body if up[2] < 0.65 else no_arms)
    heights = heights + g.gaussian(lift[:, None], 0.08 * RATE)[:, 0]

    # Bracos. De bruços as maos no chao ao lado do peito (flexao), presas
    # enquanto o braco alcanca; depois a frente perto dos joelhos e, subindo,
    # relaxadas.
    reach = {}
    for side in ("Left", "Right"):
        upper, fore, hand = (rig.index(f"{side}{n}") for n in ARMS)
        reach[side] = 0.97 * (np.linalg.norm(rig.links[fore].anchor - rig.links[upper].anchor)
                              + np.linalg.norm(rig.links[hand].anchor - rig.links[fore].anchor))
    p0, o0 = rig.forward_kinematics(np.array([planar[0][0], planar[0][1], heights[0]]), rotations[0], coordinates[0])
    spots, release, held = {}, {}, {}
    for side, sign in (("Left", 1.0), ("Right", -1.0)):
        shoulder = g.joint_world(rig, p0, o0, f"{side}UpperArm")
        spots[side] = np.array([shoulder[0] + HAND_FORWARD, shoulder[1] + sign * HAND_OUTWARD, floor + 0.05])
        release[side] = count

    # Tronco curvado enquanto as pernas vem para a frente (pedido do usuario:
    # a mao no chao dando base enquanto pes e joelhos vem): ate o ajoelhado
    # a coluna dobra para a frente o minimo para os bracos alcancarem as maos
    # no chao; depois o tronco sobe aos poucos. Na referencia (fuzil) o
    # tronco endireitava cedo e as maos saiam com so um pe apoiando.
    hands_until = int(round(HANDS_UNTIL * RATE))
    spine_ids = [rig.index(n) for n in ("Abdomen", "Chest", "UpperChest")]
    shares = (0.30, 0.35, 0.35)

    def reaches(f: int, bend: float) -> bool:
        trial = coordinates[f].copy()
        for i, share in zip(spine_ids, shares):
            trial[i][1] += bend * share
        p, o = rig.forward_kinematics(np.array([planar[f][0], planar[f][1], heights[f]]), rotations[f], trial)
        return all(np.linalg.norm(spots[s] - g.joint_world(rig, p, o, f"{s}UpperArm")) <= reach[s]
                   for s in ("Left", "Right"))

    extra = np.zeros(count)
    for f in range(min(count, hands_until)):
        if reaches(f, 0.0):
            continue
        lo, hi = 0.0, MAX_BEND
        if reaches(f, hi):
            for _ in range(10):
                mid = 0.5 * (lo + hi)
                if reaches(f, mid):
                    hi = mid
                else:
                    lo = mid
        extra[f] = hi
    rise = int(round(TRUNK_RISE * RATE))
    peak = extra[min(count, hands_until) - 1] if hands_until > 0 else 0.0
    for f in range(hands_until, min(count, hands_until + rise)):
        extra[f] = peak * (1.0 - g.smooth((f - hands_until) / max(1, rise)))
    extra = g.gaussian(extra[:, None], 0.08 * RATE)[:, 0]
    for f in range(count):
        for i, share in zip(spine_ids, shares):
            coordinates[f][i][1] += extra[f] * share

    for f in range(count):
        root_offset = np.array([planar[f][0], planar[f][1], heights[f]])
        for side in ("Left", "Right"):
            if release[side] < count:
                continue
            p, o = rig.forward_kinematics(root_offset, rotations[f], coordinates[f])
            out_of_reach = np.linalg.norm(spots[side] - g.joint_world(rig, p, o, f"{side}UpperArm")) > reach[side]
            if f > 0 and (out_of_reach or f >= hands_until):
                release[side] = f
                continue
            start = coordinates[f].copy()
            if side in held:
                for i in (rig.index(f"{side}{n}") for n in ARMS):
                    start[i] = held[side][i]
            solved = g.solve_arm(rig, root_offset, rotations[f], start, side, spots[side])
            # A palma no chao: o alvo do punho corrigido pela mao.
            p, o = rig.forward_kinematics(root_offset, rotations[f], solved)
            gap = lowest_point(rig, p, o, [f"{side}Hand"]) - floor
            if abs(gap) > 0.01 and f == 0:
                spots[side] = spots[side] - np.array([0.0, 0.0, gap])
                solved = g.solve_arm(rig, root_offset, rotations[f], solved, side, spots[side])
            for i in (rig.index(f"{side}{n}") for n in ARMS):
                coordinates[f][i] = solved[i]
            held[side] = solved
    balance_previous = {}
    for f in range(count):
        root_offset = np.array([planar[f][0], planar[f][1], heights[f]])
        p, o = rig.forward_kinematics(root_offset, rotations[f], coordinates[f])
        pelvis_height = p[0][2] - floor
        for side, sign in (("Left", 1.0), ("Right", -1.0)):
            if f < release[side]:
                continue
            shoulder = p[rig.index(f"{side}UpperArm")]
            target = shoulder + np.array([CROUCH_REACH[0], sign * CROUCH_REACH[1], CROUCH_REACH[2]])
            last = held[side]
            start = balance_previous.get(side, last)
            balance = g.solve_arm(rig, root_offset, rotations[f], start, side, target)
            balance_previous[side] = balance
            w = g.smooth((f - release[side]) / (ARM_SWAP * RATE))
            relax = g.smooth((pelvis_height - STANDING_HEIGHT) / 0.10)
            for i in (rig.index(f"{side}{n}") for n in ARMS):
                arm = last[i] * (1 - w) + balance[i] * w
                coordinates[f][i] = arm * (1 - relax) + standing[i] * relax
    for side in ("Left", "Right"):
        for name in ARMS:
            i = rig.index(f"{side}{name}")
            coordinates[:, i, :] = g.gaussian(coordinates[:, i, :], ARM_SIGMA * RATE)

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

    offsets = [np.array([planar[f][0], planar[f][1], heights[f]]) for f in range(count)]
    contacts = g.support_channels(rig, offsets, rotations, coordinates, floor,
                                  {side: (0, release[side]) for side in ("Left", "Right")}, RATE)

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
        "id": "stand_up_front_crouch",
        "displayName": "Levantar de bruços pelo agachado",
        "targetRigId": template["targetRigId"],
        "sourceAssetPath": "Prone To Crouch Transition.fbx + Crouch To Stand.fbx (Mixamo, retargeteados; bracos autorais)",
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
    out = CLIPS / "stand_up_front_crouch.matteranim.json"
    out.write_text(json.dumps(clip, indent=1), encoding="utf-8")
    print(f"{out.name}: {count} quadros, {duration:.2f} s (ajoelhado aos {a_count / RATE:.2f} s); "
          f"salto max {max_step:.1f} graus; maos soltam: "
          + ", ".join(f"{s} {f / RATE:.2f} s" for s, f in release.items()))


if __name__ == "__main__":
    main()
