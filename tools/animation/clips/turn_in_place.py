"""Giro parado (pedido do usuario, 04/10): Left Turn e Right Turn da Mixamo.

Com a ajuda cheia o giro parado toca estes clipes direto (pernas e pes), na
velocidade que o giro pedido precisa. Cada um gira ~86 graus em 1,03 s a
partir da base escalonada do parado em alerta (a mesma geometria dos pes em
relacao a pelve) e termina nela, girada: os giros se encadeiam.

A fonte e o FBX retargeteado pelo import_humanoid_animation.py
(--source-rig mixamo --root-motion preserve --reference-only). Aqui:
  - a raiz guarda so o giro (yaw relativo ao primeiro quadro) e o
    deslocamento no plano (no rumo do primeiro quadro); a altura e a
    inclinacao ficam com o parado no motor;
  - contatos dos pes recalculados pela sola: perto do chao do clipe e quase
    parada (o retarget marcava o pe direito no ar 0,1 s antes de ele sair);
  - juntas dentro dos limites do corpo;
  - a postura do parado (pose.ergonomic_posture, 09/10): o pescoco reto e o
    olhar no horizonte, como no parado em alerta de onde o giro sai.

Uso:
  python3 tools/animation/clips/turn_in_place.py
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
from matter_rig import Rig, qfrom_axis_angle, qrotate  # noqa: E402
from pose import ergonomic_posture  # noqa: E402
from validate import lowest_point  # noqa: E402

import stand_up_getup as g  # noqa: E402

SOURCES = g.SOURCE.parent
CLIPS = g.CLIPS
RATE = 30.0
# Contato: a sola a ate isto acima do ponto mais baixo dela no clipe (m) e
# andando a menos disto (m/s), sem trechos de menos de 2 quadros.
CONTACT_HEIGHT = 0.025
CONTACT_SPEED = 0.35

VARIANTS = {
    "left": dict(source="turn_left.retarget.json", id="turn_left", name="Giro parado para a esquerda (Mixamo)"),
    "right": dict(source="turn_right.retarget.json", id="turn_right", name="Giro parado para a direita (Mixamo)"),
}


def heading_of(rotation: np.ndarray) -> float:
    forward = qrotate(rotation, np.array([1.0, 0.0, 0.0]))
    return math.atan2(forward[1], forward[0])


def build(key: str) -> None:
    variant = VARIANTS[key]
    rig = Rig(g.PROFILE)
    ref = load_clip(SOURCES / variant["source"], rig)
    count = int(math.floor(ref.duration * RATE)) + 1
    offsets, rotations, coordinates = [], [], []
    for f in range(count):
        offset, rotation, c = ref.sample(min(f / RATE, ref.duration), loop=False)
        offsets.append(np.array(offset, float))
        rotations.append(np.array(rotation, float))
        coordinates.append(np.array(c, float))
    yaw0 = heading_of(rotations[0])
    yaws = np.unwrap([heading_of(r) for r in rotations]) - yaw0
    c0, s0 = math.cos(-yaw0), math.sin(-yaw0)
    planar = []
    for o in offsets:
        d = o[:2] - offsets[0][:2]
        planar.append(np.array([d[0] * c0 - d[1] * s0, d[0] * s0 + d[1] * c0]))
    for f in range(count):
        ergonomic_posture(rig, coordinates[f])
        for i in range(1, len(rig.links)):
            coordinates[f][i] = rig.clamp(i, coordinates[f][i])

    # Contatos pela sola (no proprio clipe, com a raiz original).
    soles = {"Left": np.zeros(count), "Right": np.zeros(count)}
    centers = {"Left": np.zeros((count, 2)), "Right": np.zeros((count, 2))}
    for f in range(count):
        p, o = rig.forward_kinematics(offsets[f], rotations[f], coordinates[f])
        for side in ("Left", "Right"):
            soles[side][f] = lowest_point(rig, p, o, [f"{side}Foot"])
            centers[side][f] = p[rig.index(f"{side}Foot")][:2]
    contacts = {}
    for side, key_name in (("Left", "leftFoot"), ("Right", "rightFoot")):
        floor = soles[side].min()
        speed = np.zeros(count)
        speed[1:] = np.linalg.norm(np.diff(centers[side], axis=0), axis=1) * RATE
        speed[0] = speed[1]
        v = ((soles[side] - floor < CONTACT_HEIGHT) & (speed < CONTACT_SPEED)).astype(float)
        # Comeca e termina numa base: os dois pes no chao nas pontas (o clipe
        # acaba no instante em que o ultimo pe pousa, 3,5 cm acima).
        v[:2] = 1.0
        v[-2:] = 1.0
        f = 0
        while f < count:
            e = f
            while e < count and v[e] == v[f]:
                e += 1
            if e - f < 2 and f > 0:
                v[f:e] = v[f - 1]
            f = e
        contacts[key_name] = [{"timeSeconds": round(k / RATE, 7), "value": float(v[k])} for k in range(count)]
        swings = [k / RATE for k in range(1, count) if v[k] != v[k - 1]]
        print(f"  {side}: trocas de contato em {', '.join(f'{t:.2f}' for t in swings)} s")

    template = json.loads((CLIPS / "stand_up_front.matteranim.json").read_text(encoding="utf-8"))
    tracks = []
    for i, link in enumerate(rig.links):
        if i == 0:
            keys = [{"timeSeconds": round(f / RATE, 7),
                     "rootTranslationOffsetMeters": [round(float(planar[f][0]), 7), round(float(planar[f][1]), 7), 0.0],
                     "rootRotationDelta": [round(float(x), 8) for x in qfrom_axis_angle(np.array([0.0, 0.0, 1.0]), float(yaws[f]))]}
                    for f in range(count)]
            tracks.append({"targetLinkId": link.id, "space": "root", "keyframes": keys})
            continue
        keys = [{"timeSeconds": round(f / RATE, 7),
                 "jointPositionRadians": [round(float(x), 7) for x in coordinates[f][i]]}
                for f in range(count)]
        tracks.append({"targetLinkId": link.id, "space": "joint", "keyframes": keys})
    steps = np.abs(np.diff(np.array(coordinates)[:, 1:, :], axis=0))
    max_step = math.degrees(float(steps.max()))
    duration = (count - 1) / RATE
    clip = {
        "schema": template["schema"],
        "id": variant["id"],
        "displayName": variant["name"],
        "targetRigId": template["targetRigId"],
        "sourceAssetPath": f"assets/animations/source/mixamo/{'LeftTurn' if key == 'left' else 'RightTurn'}.fbx (Mixamo), retargeteado",
        "durationSeconds": round(duration, 7),
        "sourceSampleRateHz": RATE,
        "loops": False,
        "sourceRootDisplacementMeters": [round(float(planar[-1][0]), 7), round(float(planar[-1][1]), 7), 0.0],
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
    print(f"{out.name}: {count} quadros, {duration:.2f} s; giro {math.degrees(yaws[-1]):+.1f} graus; "
          f"raiz anda ({planar[-1][0]:+.2f}, {planar[-1][1]:+.2f}) m; salto max {max_step:.1f} graus")


def main() -> None:
    for key in (sys.argv[1:] or ["left", "right"]):
        build(key)


if __name__ == "__main__":
    main()
