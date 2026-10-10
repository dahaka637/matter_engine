"""Levantar de bruços pelo Getting Up (referencia do usuario, 04/10).

De bruços: flexao de bracos, o quadril recua para cima dos joelhos (as maos
ainda no chao a frente), um pe vem para a frente num agachamento com os pes
embaixo do quadril e dali ele fica em pe. Os pes quase nao saem do lugar: e o
corpo que recua para cima deles - o centro de massa passa de sobre as maos
para sobre os joelhos e pes sem nunca ficar a frente dos pes (o que derrubava
o levantar antigo sem ajuda).

A referencia e de um bebado. O que sai:
  - raiz: so inclinacao e altura (o runtime ancora o levantar onde o corpo
    deitou e tira o rumo); a rolagem de lado reduzida e suavizada, e no fim
    sem a inclinacao para tras;
  - coluna, pescoco e cabeca suavizados, com giro e inclinacao lateral
    reduzidos;
  - pernas da referencia com suavizacao leve;
  - bracos: os da referencia enquanto a mao apoia no chao; quando ela sai, o
    braco vai a frente equilibrando (IK) e no fim relaxa ao lado do corpo
    (os bracos de pe do fim do levantar antigo);
  - fim: mistura curta para a pose de pe limpa.

Uso:
  python3 tools/animation/clips/stand_up_getup.py
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from matter_clip import load_clip  # noqa: E402
from matter_rig import (REPO_ROOT, Rig, qconj, qfrom_axis_angle, qmul,  # noqa: E402
                        qnormalize, qrotate)
from validate import lowest_point  # noqa: E402

SOURCE = REPO_ROOT / "assets" / "animations" / "source" / "retargeted" / "getting_up.retarget.json"
CLIPS = REPO_ROOT / "assets" / "animations" / "clips"
PROFILE = REPO_ROOT / "assets" / "characters" / "football_player" / "FootballPlayerV1.ragdoll.json"
RATE = 30.0
START = 0.6            # s da referencia: deitado, antes das maos virem
STOP = 4.9             # s: ja em pe (dali em diante so o bebado balancando)
END_BLEND = 0.45       # s finais misturando para a pose de pe limpa
ROLL_SHARE = 0.3       # rolagem de lado da raiz que fica
SPINE_SIDE_SHARE = 0.4  # giro e inclinacao lateral da coluna/cabeca que ficam
SPINE_SIGMA = 0.12     # s, suavizacao de coluna, pescoco, cabeca e raiz
LEG_SIGMA = 0.04
FOOT_SIGMA = 0.10
ARM_SIGMA = 0.05
HAND_DOWN = 0.08       # mao a menos disto do chao: apoiando
ARM_SWAP = 0.30        # s da mao saindo ate o braco de equilibrio
REACH = np.array([0.30, 0.08, -0.38])  # punho a partir do ombro, braco de equilibrio
STANDING_HEIGHT = 0.72  # pelve acima disto (do chao): subindo, bracos relaxam
FEET_SETTLE = 0.15     # s depois dos dois pes plantados ate as maos soltarem
ARMS = ("UpperArm", "Forearm", "Hand")
SPINE = ("Abdomen", "Chest", "UpperChest", "Neck", "Head")


def smooth(t: float) -> float:
    t = min(max(t, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def gaussian(values: np.ndarray, sigma_frames: float) -> np.ndarray:
    """Suaviza ao longo do tempo (eixo 0), com as pontas repetidas."""
    if sigma_frames <= 0.0:
        return values.copy()
    radius = int(math.ceil(3.0 * sigma_frames))
    kernel = np.exp(-0.5 * (np.arange(-radius, radius + 1) / sigma_frames) ** 2)
    kernel /= kernel.sum()
    padded = np.concatenate([np.repeat(values[:1], radius, axis=0), values,
                             np.repeat(values[-1:], radius, axis=0)], axis=0)
    out = np.zeros_like(values)
    for k, w in enumerate(kernel):
        out += w * padded[k:k + len(values)]
    return out


def heading_of(rotation: np.ndarray) -> float:
    forward = qrotate(rotation, np.array([1.0, 0.0, 0.0]))
    up = qrotate(rotation, np.array([0.0, 0.0, 1.0]))
    planar = forward[:2] + up[:2]
    return math.atan2(planar[1], planar[0])


def tilt_only(rotation: np.ndarray) -> np.ndarray:
    """Sem rumo (frente + cima no plano: estavel de bruços e em pe)."""
    forward = qrotate(rotation, np.array([1.0, 0.0, 0.0]))
    up = qrotate(rotation, np.array([0.0, 0.0, 1.0]))
    planar = forward[:2] + up[:2]
    yaw = math.atan2(planar[1], planar[0])
    return qnormalize(qmul(qfrom_axis_angle(np.array([0.0, 0.0, 1.0]), -yaw), rotation))


def pitch_roll(rotation: np.ndarray) -> tuple[float, float]:
    """Inclinacao para a frente (em torno de y) e de lado da raiz sem rumo."""
    up = qrotate(rotation, np.array([0.0, 0.0, 1.0]))
    forward = qrotate(rotation, np.array([1.0, 0.0, 0.0]))
    pitch = math.atan2(up[0] + (-forward[2]), up[2] + forward[0])
    roll = math.asin(max(-1.0, min(1.0, -up[1] if abs(up[2]) > 0.3 else forward[1])))
    return pitch, roll


def from_pitch_roll(pitch: float, roll: float) -> np.ndarray:
    return qnormalize(qmul(qfrom_axis_angle(np.array([0.0, 1.0, 0.0]), pitch),
                           qfrom_axis_angle(np.array([1.0, 0.0, 0.0]), roll)))


def wrist_world(rig: Rig, positions, orientations, side: str) -> np.ndarray:
    hand = rig.index(f"{side}Hand")
    fore = rig.links[hand].parent
    parent = rig.links[fore]
    anchor = qrotate(qconj(parent.model_orientation), rig.links[hand].anchor - parent.model_position)
    return positions[fore] + qrotate(orientations[fore], anchor)


def joint_world(rig: Rig, positions, orientations, child: str) -> np.ndarray:
    """Ponto da junta de `child` (no pai) no mundo do clipe."""
    i = rig.index(child)
    parent = rig.links[i].parent
    p = rig.links[parent]
    anchor = qrotate(qconj(p.model_orientation), rig.links[i].anchor - p.model_position)
    return positions[parent] + qrotate(orientations[parent], anchor)


def support_channels(rig: Rig, root_offsets, rotations, coordinates, floor: float,
                     hand_windows: dict, rate: float) -> dict:
    """Canais de apoio do clipe (1 apoiando): maos pelas janelas em que o
    montador as mantem no chao; joelhos e pes pela altura (deslizar pelo chao
    continua sendo apoio), sem piscar (no minimo 3 quadros)."""
    count = len(coordinates)
    raw = {k: np.zeros(count) for k in ("leftHand", "rightHand", "leftKnee", "rightKnee", "leftFoot", "rightFoot")}
    for f in range(count):
        p, o = rig.forward_kinematics(root_offsets[f], rotations[f], coordinates[f])
        for side, key in (("Left", "left"), ("Right", "right")):
            start, end = hand_windows.get(side, (0, 0))
            raw[f"{key}Hand"][f] = 1.0 if start <= f < end else 0.0
            raw[f"{key}Knee"][f] = 1.0 if joint_world(rig, p, o, f"{side}Shin")[2] - floor < 0.13 else 0.0
            raw[f"{key}Foot"][f] = 1.0 if joint_world(rig, p, o, f"{side}Foot")[2] - floor < 0.20 else 0.0
    channels = {}
    for key, values in raw.items():
        v = values.copy()
        # sem trechos de menos de 3 quadros
        f = 0
        while f < count:
            g_ = f
            while g_ < count and v[g_] == v[f]:
                g_ += 1
            if g_ - f < 3 and f > 0:
                v[f:g_] = v[f - 1]
            f = g_
        channels[key] = [{"timeSeconds": round(k / rate, 7), "value": float(v[k])} for k in range(count)]
    return channels


def solve_arm(rig: Rig, root_offset, root_rotation, coordinates, side: str,
              target: np.ndarray) -> np.ndarray:
    upper, fore = rig.index(f"{side}UpperArm"), rig.index(f"{side}Forearm")
    free = [(upper, a) for a in range(3) if rig.links[upper].axes[a].enabled]
    free += [(fore, a) for a in range(3) if rig.links[fore].axes[a].enabled]
    rest = coordinates.copy()
    c = coordinates.copy()

    def residual(values: np.ndarray) -> np.ndarray:
        trial = c.copy()
        for (link, axis), v in zip(free, values):
            trial[link][axis] = v
        p, o = rig.forward_kinematics(root_offset, root_rotation, trial)
        drift = np.array([trial[link][axis] - rest[link][axis] for link, axis in free]) * 0.05
        return np.concatenate([wrist_world(rig, p, o, side) - target, drift])

    values = np.array([c[link][axis] for link, axis in free])
    for _ in range(12):
        r = residual(values)
        jacobian = np.zeros((len(r), len(values)))
        for k in range(len(values)):
            probe = values.copy()
            probe[k] += 1e-4
            jacobian[:, k] = (residual(probe) - r) / 1e-4
        step = np.linalg.lstsq(jacobian, -r, rcond=None)[0]
        values += step
        for k, (link, axis) in enumerate(free):
            limit = rig.links[link].axes[axis]
            values[k] = min(max(values[k], limit.minimum), limit.maximum)
        if np.linalg.norm(step) < 1e-6:
            break
    for (link, axis), v in zip(free, values):
        c[link][axis] = v
    return c


def main() -> None:
    rig = Rig(PROFILE)
    ref = load_clip(SOURCE, rig)
    old = load_clip(CLIPS / "stand_up_front.matteranim.json", rig)
    standing = old.coordinates[-1].copy()
    standing_rotation = tilt_only(old.root_rotations[-1])
    standing_height = old.root_offsets[-1][2]

    count = int(round((STOP - START) * RATE)) + 1
    times = np.array([START + f / RATE for f in range(count)])
    heights = np.zeros(count)
    planar = np.zeros((count, 2))
    pitches = np.zeros(count)
    rolls = np.zeros(count)
    coordinates = np.zeros((count, len(rig.links), 3))
    for f, t in enumerate(times):
        offset, rotation, c = ref.sample(float(t), loop=False)
        heights[f] = offset[2]
        planar[f] = offset[:2]
        pitches[f], rolls[f] = pitch_roll(tilt_only(rotation))
        coordinates[f] = c
    pitches = np.unwrap(pitches)
    # Deslocamento da raiz no plano, no rumo do primeiro quadro (o runtime
    # ancora o levantar onde o corpo deitou; aqui ele so define quem fica
    # parado no chao - as maos enquanto o corpo recua).
    _, first_rotation, _ = ref.sample(float(times[0]), loop=False)
    yaw0 = heading_of(first_rotation)
    c0, s0 = math.cos(-yaw0), math.sin(-yaw0)
    planar = planar - planar[0]
    planar = np.stack([planar[:, 0] * c0 - planar[:, 1] * s0, planar[:, 0] * s0 + planar[:, 1] * c0], axis=1)
    planar = gaussian(planar, SPINE_SIGMA * RATE)

    # Raiz: suave, pouca rolagem e, no fim, sem inclinar para tras.
    sigma = SPINE_SIGMA * RATE
    heights = gaussian(heights[:, None], sigma)[:, 0]
    pitches = gaussian(pitches[:, None], sigma)[:, 0]
    rolls = gaussian(rolls[:, None], sigma)[:, 0] * ROLL_SHARE
    pitches = np.maximum(pitches, 0.0)

    # Coluna, pescoco e cabeca: suaves, com giro e lado reduzidos.
    for name in SPINE:
        i = rig.index(name)
        track = gaussian(coordinates[:, i, :], sigma)
        for axis in range(3):
            if not rig.links[i].axes[axis].enabled:
                continue
            # eixo 1 = flexao (frente/tras); 0 = giro, 2 = lado
            if axis != 1:
                track[:, axis] *= SPINE_SIDE_SHARE
        coordinates[:, i, :] = track
    for side in ("Left", "Right"):
        for name in ("Thigh", "Shin", "Foot"):
            i = rig.index(f"{side}{name}")
            coordinates[:, i, :] = gaussian(coordinates[:, i, :], LEG_SIGMA * RATE)

    rotations = [from_pitch_roll(p, r) for p, r in zip(pitches, rolls)]

    # (a busca abaixo, quadro a quadro, sai em degraus: suavizada no fim)
    # Pes: o nosso tornozelo nao estica tanto quanto o da referencia e a
    # caixa do pe cravava 15-20 cm no chao deitado e ajoelhado. Em cada
    # quadro o tornozelo gira (dentro dos limites) para o pe atravessar o
    # minimo: deitado e ajoelhado ele fica na ponta, como no corpo fisico.
    feet_floor = float(rig.links[0].model_position[2]) - rig.standing_root_height
    for f in range(count):
        for side in ("Left", "Right"):
            foot = rig.index(f"{side}Foot")
            axis = rig.links[foot].axes[0]
            best = None
            base = coordinates[f][foot].copy()
            for k in range(13):
                trial = base.copy()
                trial[0] = axis.minimum + (axis.maximum - axis.minimum) * k / 12.0
                coordinates[f][foot] = trial
                p, o = rig.forward_kinematics(np.array([planar[f][0], planar[f][1], heights[f]]), rotations[f], coordinates[f])
                low = lowest_point(rig, p, o, [f"{side}Foot"])
                # Sem atravessar, o do clipe; atravessando, o que menos atravessa.
                score = min(0.0, low - (lowest_point(rig, p, o, [l.id for l in rig.links
                                                               if not l.id.endswith("Foot") and not any(a in l.id for a in ARMS)])))
                penalty = abs(trial[0] - base[0]) * 0.01
                value = score - penalty
                if best is None or value > best[0]:
                    best = (value, trial)
            coordinates[f][foot] = best[1]

    for side in ("Left", "Right"):
        foot = rig.index(f"{side}Foot")
        coordinates[:, foot, :] = gaussian(coordinates[:, foot, :], FOOT_SIGMA * RATE)

    # O chao do runtime: a pelve esperada e a altura de pe mais o
    # deslocamento da raiz, entao o chao fica a altura de pe abaixo da pelve
    # de bind.
    floor = float(rig.links[0].model_position[2]) - rig.standing_root_height
    # Cada quadro apoiado no chao. Deitado/engatinhando (pelve inclinada mais
    # de ~50 graus) pelo tronco, coxas e canelas: as maos e os pes da
    # referencia atravessavam o chao (~18 e ~20 cm; o nosso tornozelo nao
    # estica tanto). De pe e agachado, pelos pes. A correcao e suavizada
    # (sem degrau na troca).
    not_feet = [link.id for link in rig.links
                if not link.id.endswith("Foot") and not any(a in link.id for a in ARMS)]
    everything = [link.id for link in rig.links if not any(a in link.id for a in ARMS)]
    lift = np.zeros(count)
    for f in range(count):
        p, o = rig.forward_kinematics(np.array([planar[f][0], planar[f][1], heights[f]]), rotations[f], coordinates[f])
        up = qrotate(rotations[f], np.array([0.0, 0.0, 1.0]))
        support = not_feet if up[2] < 0.65 else everything
        lift[f] = floor - lowest_point(rig, p, o, support)
    heights = heights + gaussian(lift[:, None], 0.08 * RATE)[:, 0]

    # Bracos. A mao solta de vez na ultima vez que esta no chao (antes disso
    # ela pode se reposicionar - sobe e volta - e o braco e o da referencia).
    release = {}
    hand_spot = {}
    for side in ("Left", "Right"):
        last_down = 0
        for f in range(count):
            p, o = rig.forward_kinematics(np.array([planar[f][0], planar[f][1], heights[f]]), rotations[f], coordinates[f])
            low = lowest_point(rig, p, o, [f"{side}Hand"])
            if low - floor < HAND_DOWN:
                last_down = f
                # O punho com a mao encostando no chao.
                hand_spot[side] = wrist_world(rig, p, o, side) - np.array([0.0, 0.0, low - floor])
        release[side] = last_down + 1
    # As maos so soltam com os dois pes plantados de vez embaixo do corpo: a
    # referencia (bebado) tirava a mao esquerda do chao enquanto o pe
    # esquerdo ainda vinha para a frente - sem apoio nenhum daquele lado, o
    # corpo fisico tombava de lado. Ate la a mao fica no ultimo ponto do
    # chao (IK), enquanto o braco alcancar.
    feet_down = []
    for f in range(count):
        p, o = rig.forward_kinematics(np.array([planar[f][0], planar[f][1], heights[f]]), rotations[f], coordinates[f])
        feet_down.append(all(lowest_point(rig, p, o, [f"{s}Foot"]) - floor < 0.04 for s in ("Left", "Right")))
    planted = count - 1
    while planted > 0 and feet_down[planted - 1]:
        planted -= 1
    hold_until = planted + int(round(FEET_SETTLE * RATE))
    reach = {}
    for side in ("Left", "Right"):
        upper, fore, hand = (rig.index(f"{side}{n}") for n in ARMS)
        reach[side] = 0.98 * (np.linalg.norm(rig.links[fore].anchor - rig.links[upper].anchor)
                              + np.linalg.norm(rig.links[hand].anchor - rig.links[fore].anchor))
    for side in ("Left", "Right"):
        if side not in hand_spot:
            continue
        held = coordinates[release[side] - 1].copy() if release[side] > 0 else coordinates[0].copy()
        for f in range(release[side], min(count, hold_until)):
            root_offset = np.array([planar[f][0], planar[f][1], heights[f]])
            p, o = rig.forward_kinematics(root_offset, rotations[f], coordinates[f])
            if np.linalg.norm(hand_spot[side] - p[rig.index(f"{side}UpperArm")]) > reach[side]:
                break
            start = coordinates[f].copy()
            for i in (rig.index(f"{side}{n}") for n in ARMS):
                start[i] = held[i]
            solved = solve_arm(rig, root_offset, rotations[f], start, side, hand_spot[side])
            for i in (rig.index(f"{side}{n}") for n in ARMS):
                coordinates[f][i] = solved[i]
            held = solved
            release[side] = f + 1
    previous_balance = {}
    for f in range(count):
        root_offset = np.array([planar[f][0], planar[f][1], heights[f]])
        p, o = rig.forward_kinematics(root_offset, rotations[f], coordinates[f])
        pelvis_height = p[0][2] - floor
        for side, sign in (("Left", 1.0), ("Right", -1.0)):
            links = [rig.index(f"{side}{n}") for n in ARMS]
            if f < release[side]:
                continue
            shoulder = p[rig.index(f"{side}UpperArm")]
            target = shoulder + np.array([REACH[0], sign * REACH[1], REACH[2]])
            start = previous_balance.get(side, coordinates[release[side] - 1] if release[side] > 0 else coordinates[f])
            balance = solve_arm(rig, root_offset, rotations[f], start, side, target)
            previous_balance[side] = balance
            w = smooth((f - release[side]) / (ARM_SWAP * RATE))
            relax = smooth((pelvis_height - STANDING_HEIGHT) / 0.08)
            # Parte da ultima pose de apoio (a da mao no chao), nao do
            # braco da referencia naquele quadro.
            last = coordinates[release[side] - 1] if release[side] > 0 else coordinates[f]
            for i in links:
                arm = last[i] * (1 - w) + balance[i] * w
                coordinates[f][i] = arm * (1 - relax) + standing[i] * relax

    for side in ("Left", "Right"):
        for name in ARMS:
            i = rig.index(f"{side}{name}")
            coordinates[:, i, :] = gaussian(coordinates[:, i, :], ARM_SIGMA * RATE)

    # Fim: a pose de pe limpa.
    blend_frames = int(round(END_BLEND * RATE))
    for k in range(blend_frames + 1):
        f = count - 1 - blend_frames + k
        w = smooth(k / blend_frames)
        coordinates[f] = coordinates[f] * (1 - w) + standing * w
        heights[f] = heights[f] * (1 - w) + standing_height * w
        r0, r1 = rotations[f], standing_rotation
        if np.dot(r0, r1) < 0:
            r1 = -r1
        rotations[f] = qnormalize(r0 * (1 - w) + r1 * w)

    for i, link in enumerate(rig.links):
        if i == 0:
            continue
        for f in range(count):
            coordinates[f][i] = rig.clamp(i, coordinates[f][i])

    windows = {}
    for side in ("Left", "Right"):
        first = 0
        for f in range(count):
            p, o = rig.forward_kinematics(np.array([planar[f][0], planar[f][1], heights[f]]), rotations[f], coordinates[f])
            if lowest_point(rig, p, o, [f"{side}Hand"]) - floor < HAND_DOWN:
                first = f
                break
        windows[side] = (first, release.get(side, 0))
    offsets = [np.array([planar[f][0], planar[f][1], heights[f]]) for f in range(count)]
    contacts = support_channels(rig, offsets, rotations, coordinates, floor, windows, RATE)

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
    steps = np.abs(np.diff(coordinates[:, 1:, :], axis=0))
    max_step = math.degrees(float(steps.max())) if len(steps) else 0.0
    limit_hits = 0
    for i, link in enumerate(rig.links):
        if i == 0:
            continue
        for axis in range(3):
            a = link.axes[axis]
            if a.enabled:
                limit_hits += int(np.sum((coordinates[:, i, axis] <= a.minimum + 1e-4)
                                         | (coordinates[:, i, axis] >= a.maximum - 1e-4)))
    clip = {
        "schema": template["schema"],
        "id": "stand_up_front_getup",
        "displayName": "Levantar de bruços (Getting Up limpo)",
        "targetRigId": template["targetRigId"],
        "sourceAssetPath": "Getting Up.fbx (Mixamo, retargeteado; limpo do bebado, bracos autorais)",
        "durationSeconds": round(duration, 7),
        "sourceSampleRateHz": RATE,
        "loops": False,
        "sourceRootDisplacementMeters": [0.0, 0.0, 0.0],
        "nominalSpeedMetersPerSecond": 0.0,
        "contacts": contacts,
        "rootMotionPolicy": "preserve",
        # Clipe montado aqui (a referencia so da o movimento): sem erro de
        # direcao de retarget; o salto maximo de junta por quadro e os
        # limites vem da curva final.
        "retargetReport": {
            "origin": "authored",
            "passed": max_step < 60.0,
            "directionRmsDegrees": 0.0,
            "directionMaxDegrees": 0.0,
            "limbDirectionMaxDegrees": 0.0,
            "maximumJointStepDegrees": round(max_step, 3),
            "limitHitCount": limit_hits,
        },
        "tracks": tracks,
    }
    out = CLIPS / "stand_up_front_getup.matteranim.json"
    out.write_text(json.dumps(clip, indent=1), encoding="utf-8")
    print(f"{out.name}: {count} quadros, {duration:.2f} s; salto max {max_step:.1f} graus, limites {limit_hits}; maos soltam: "
          + ", ".join(f"{s} {f / RATE:.2f} s" for s, f in release.items()))


if __name__ == "__main__":
    main()
