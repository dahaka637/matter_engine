"""Parado natural - o idle do personagem.

Corpo em pe e relaxado, respirando e transferindo o peso de uma perna para a
outra, com os pes plantados. Nada vem de captura nem de biblioteca: a pose e o
movimento sao descritos aqui.

Decisoes de pose (e por que):
  - bracos a ~15 graus da vertical: pendem soltos sem encostar no tronco.
    Mais fechados, a capsula do braco toca o peito e a autocolisao empurra o
    braco para fora no jogo - o "braco levantado" do idle antigo;
  - palmas para as coxas, polegar a frente (braco girado ~10 graus para
    dentro): e como a mao
    cai quando ninguem a segura;
  - cotovelos levemente dobrados, joelhos soltos (pelve 1,2 cm abaixo do bind):
    articulacao travada le como rigidez;
  - pes um pouco alem da largura do quadril, pontas abertas 7 graus.

Movimento, todo com periodo do loop (fecha exatamente):
  - respiracao: peito sobe, ombros acompanham, cabeca levanta um pouco;
  - transferencia de peso: a pelve desliza ~1 cm e o quadril do lado sem apoio
    cai ~1 grau; a coluna compensa para a cabeca ficar centrada; os pes ficam
    parados (IK);
  - a cabeca deriva ~2 graus, fora de fase com o resto;
  - bracos com um pendulo sutil, atrasado em relacao ao balanco.
"""

from __future__ import annotations

import math

import numpy as np

from authoring import ClipDefinition
from matter_rig import Rig
from pose import ArmSpec, LegSpec, Pose, PoseBuilder
from validate import lowest_point

ID = "natural_idle"
DISPLAY_NAME = "Parado natural"
DURATION = 4.0

# Pose base.
PELVIS_DROP = 0.012          # m abaixo do bind: joelhos soltos
PELVIS_PITCH = 1.0           # graus, para a frente
SPINE_FLEXION = 3.0
HEAD_NOD = 4.0
STANCE_HALF_WIDTH = 0.105    # m, tornozelo ao plano medio
TOE_OUT = 7.0
ARM_LOWERED = 75.0           # 90 = colado ao corpo
ARM_FORWARD = 6.0
ELBOW_FLEXION = 16.0
# Palma para a coxa, polegar a frente: giro do braco para dentro. O antebraco
# e uma dobradica (a pronacao do ALS girava o antebraco em volta do braco).
ARM_INWARD_ROLL = 10.0
WRIST_FLEXION = 6.0

# Amplitudes do movimento.
BREATH_CHEST = 1.2           # graus de extensao no pico da inspiracao
BREATH_SHOULDER = 1.0        # graus que o braco abre com o ombro subindo
BREATH_HEAD = 0.8
SWAY_METERS = 0.010
SWAY_ROLL = 1.2              # graus; o quadril sem apoio cai
SWAY_SPINE_COMPENSATION = 0.8
HEAD_DRIFT = 2.0
ARM_PENDULUM = 1.2           # graus para frente/tras
ARM_PENDULUM_LAG = 0.6       # rad de atraso em relacao ao balanco


def definition(rig: Rig) -> ClipDefinition:
    builder = PoseBuilder(rig)
    floor = lowest_point(rig, builder.bind_positions,
                         builder.bind_orientations, ["LeftFoot", "RightFoot"])
    ankle_height = builder.floor_ankle_height()
    toe = math.radians(TOE_OUT)

    def pose_at(time: float) -> Pose:
        phase = 2.0 * math.pi * time / DURATION
        breath = 0.5 - 0.5 * math.cos(phase)            # 0 -> 1 -> 0
        sway = math.sin(phase)
        pendulum = math.sin(phase - ARM_PENDULUM_LAG)

        b = PoseBuilder(rig)
        b.root(offset=(0.0, SWAY_METERS * sway, -PELVIS_DROP),
               pitch_degrees=PELVIS_PITCH,
               # Peso na perna esquerda (sway > 0) -> quadril direito cai.
               roll_degrees=SWAY_ROLL * sway)
        b.spine(flexion=SPINE_FLEXION - BREATH_CHEST * breath,
                lateral=SWAY_SPINE_COMPENSATION * SWAY_ROLL * sway)
        b.head(nod=HEAD_NOD - BREATH_HEAD * breath,
               turn=HEAD_DRIFT * math.sin(phase + 1.1),
               tilt=-0.5 * SWAY_ROLL * sway)
        for side, s in (("Left", 1.0), ("Right", -1.0)):
            b.leg(side, LegSpec(
                ankle=np.array([0.005, s * STANCE_HALF_WIDTH,
                                floor + ankle_height]),
                knee_direction=np.array([math.cos(toe), s * math.sin(toe),
                                         0.0]),
                toe_out_degrees=TOE_OUT))
            b.arm(side, ArmSpec(
                lowered_degrees=ARM_LOWERED - BREATH_SHOULDER * breath,
                forward_degrees=ARM_FORWARD + ARM_PENDULUM * pendulum,
                elbow_flexion_degrees=ELBOW_FLEXION + 1.5 * breath,
                inward_roll_degrees=ARM_INWARD_ROLL,
                wrist_flexion_degrees=WRIST_FLEXION))
        return b.solve()

    return ClipDefinition(id=ID, display_name=DISPLAY_NAME, duration=DURATION,
                          loops=True, pose_at=pose_at, floor_z=floor)
