"""Pulos: poses-chave ao longo do voo, interpoladas.

Um pulo aqui e o VOO: da decolagem (fase 0) ao apice (0,5) e ao instante de
tocar o chao (1). O runtime toca o clipe pela fase do voo, calculada da
velocidade vertical - funciona para qualquer altura de pulo e para cair de um
degrau. Nao ha preparacao agachada: no jogo o pulo sai no instante do botao.

Cada chave descreve o corpo como alguem descreveria: onde cada pe esta em
relacao ao proprio quadril (a frente, para fora, quanto abaixo), inclinacao
do pe, pelve, coluna, cabeca e bracos. Os parametros sao interpolados por
Catmull-Rom (velocidade continua entre as chaves) e as pernas saem por IK.

Espaco de modelo: frente +X, esquerda +Y, cima +Z.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, fields

import numpy as np

from authoring import ClipDefinition
from matter_rig import Rig
from pose import ArmSpec, LegSpec, Pose, PoseBuilder

SIDES = (("Left", 1.0), ("Right", -1.0))


@dataclass
class Foot:
    forward: float = 0.0     # m a frente do quadril
    outward: float = 0.0     # m para fora do quadril (+ afasta do centro)
    drop: float = 0.8        # m abaixo do quadril (perna esticada: ~0,83)
    pitch: float = 0.0       # graus, + ponta para baixo
    toe_out: float = 5.0
    # 0 = pe orientado no mundo (pitch acima); 1 = pe acompanhando a canela,
    # para a perna recolhida atras, com a canela apontando para cima.
    follow: float = 0.0


@dataclass
class Arm:
    lowered: float = 70.0
    forward: float = 0.0
    elbow: float = 30.0


@dataclass
class JumpKey:
    phase: float
    left_foot: Foot = field(default_factory=Foot)
    right_foot: Foot = field(default_factory=Foot)
    left_arm: Arm = field(default_factory=Arm)
    right_arm: Arm = field(default_factory=Arm)
    pelvis_pitch: float = 0.0
    pelvis_roll: float = 0.0          # + tomba para a direita
    spine_flexion: float = 0.0
    spine_lateral: float = 0.0        # + inclina para a esquerda
    head_nod: float = 0.0


def _flatten(key: JumpKey) -> np.ndarray:
    values = []
    for f in fields(key):
        value = getattr(key, f.name)
        if f.name == "phase":
            continue
        if isinstance(value, (Foot, Arm)):
            values.extend(getattr(value, g.name) for g in fields(value))
        else:
            values.append(value)
    return np.array(values, dtype=np.float64)


def _unflatten(values: np.ndarray, phase: float) -> JumpKey:
    it = iter(values.tolist())
    key = JumpKey(phase)
    for f in fields(key):
        if f.name == "phase":
            continue
        value = getattr(key, f.name)
        if isinstance(value, (Foot, Arm)):
            for g in fields(value):
                setattr(value, g.name, next(it))
        else:
            setattr(key, f.name, next(it))
    return key


def _catmull_rom(p0, p1, p2, p3, u):
    return 0.5 * ((2 * p1) + (-p0 + p2) * u
                  + (2 * p0 - 5 * p1 + 4 * p2 - p3) * u * u
                  + (-p0 + 3 * p1 - 3 * p2 + p3) * u * u * u)


class Jump:
    def __init__(self, rig: Rig, keys: list[JumpKey]):
        self.rig = rig
        self.keys = sorted(keys, key=lambda k: k.phase)
        self.values = [_flatten(k) for k in self.keys]
        builder = PoseBuilder(rig)
        self.hips = {side: builder.bind_anchor(f"{side}Thigh")
                     for side, _ in SIDES}

    def key_at(self, phase: float) -> JumpKey:
        phases = [k.phase for k in self.keys]
        phase = min(max(phase, phases[0]), phases[-1])
        i = max(0, min(len(phases) - 2,
                       int(np.searchsorted(phases, phase, side="right")) - 1))
        u = (phase - phases[i]) / max(phases[i + 1] - phases[i], 1e-6)
        v = self.values
        p0 = v[max(i - 1, 0)]
        p3 = v[min(i + 2, len(v) - 1)]
        return _unflatten(_catmull_rom(p0, v[i], v[i + 1], p3, u), phase)

    def pose_at(self, phase: float) -> Pose:
        key = self.key_at(phase)
        b = PoseBuilder(self.rig)
        b.root(pitch_degrees=key.pelvis_pitch, roll_degrees=key.pelvis_roll)
        b.spine(flexion=key.spine_flexion, lateral=key.spine_lateral)
        b.head(nod=key.head_nod - 0.7 * (key.pelvis_pitch + key.spine_flexion))
        for side, s in SIDES:
            foot: Foot = getattr(key, f"{side.lower()}_foot")
            hip = self.hips[side]
            ankle = hip + np.array([foot.forward, s * foot.outward, -foot.drop])
            toe = math.radians(foot.toe_out)
            b.leg(side, LegSpec(
                ankle=ankle,
                knee_direction=np.array([math.cos(toe), s * math.sin(toe), 0.0]),
                toe_out_degrees=foot.toe_out,
                foot_pitch_degrees=foot.pitch,
                ankle_follow=min(max(foot.follow, 0.0), 1.0),
                ankle_flexion_degrees=15.0))
            arm: Arm = getattr(key, f"{side.lower()}_arm")
            b.arm(side, ArmSpec(lowered_degrees=arm.lowered,
                                forward_degrees=arm.forward,
                                elbow_flexion_degrees=arm.elbow,
                                # Zero: ver GaitSpec.pronation em gait.py.
                                pronation_degrees=0.0))
        return b.solve()


def jump_definition(rig: Rig, clip_id: str, display_name: str,
                    keys: list[JumpKey],
                    direction_degrees: float = 0.0) -> ClipDefinition:
    jump = Jump(rig, keys)
    # Duracao nominal de 1 s: o runtime amostra pela fase do voo.
    return ClipDefinition(
        id=clip_id, display_name=display_name, duration=1.0, loops=False,
        pose_at=jump.pose_at, contact_at=lambda t: (0.0, 0.0),
        sample_rate=30.0, travel_direction_degrees=direction_degrees,
        floor_z=-10.0)
