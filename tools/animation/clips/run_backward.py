"""Corrida de costas - recuar olhando para a frente.

E o que o personagem faz quando o movimento aponta para tras da camera
(inclusive nas diagonais para tras): em vez de dar meia-volta, recua
correndo. Inspirada (medida, nao copiada) em Running Backward.fbx e
Run Backward.fbx, que concordam entre si:

  - ~3 m/s a 189 passos/min, com voo curto;
  - a ponta do pe toca primeiro, atras do quadril, com o joelho bem dobrado
    (~60 graus); o calcanhar desce e o pe sai chato, a frente do corpo;
  - base mais larga que a da corrida para a frente (~12 cm do centro);
  - tronco levemente a frente, bracos dobrados balancando curto.

E a passada para a frente invertida no tempo (roll_at_touchdown).
"""

from __future__ import annotations

from gait import (RUNNING_ARM_INWARD_ROLL, RUNNING_ARM_LOWERED,
                  RUNNING_WRIST_FLEXION, GaitSpec, gait_definition)
from matter_rig import Rig

BACKWARD = GaitSpec(
    id="run_backward",
    display_name="Corrida de costas",
    speed=2.9,
    direction_degrees=180.0,
    cadence=3.15,             # 189 passos/min, como as referencias
    duty=0.23,                # voo de ~43%
    stance_width=0.12,
    stance_center=-0.04,      # toque ~11 cm atras do quadril, saida ~10 a frente
    swing_height=0.12,
    swing_peak=0.5,
    roll_at_touchdown=True,
    roll_fraction=0.3,
    toe_off_pitch=30.0,       # a ponta toca primeiro
    landing_dorsiflex=0.0,
    toe_out=6.0,
    pelvis_drop=0.077,        # preserva altura no apoio ao reduzir o bob
    pelvis_bob=-0.018,
    pelvis_sway=0.015,
    pelvis_yaw=6.0,
    pelvis_list=3.0,
    lean_forward=7.0,
    lean_travel=0.0,
    spine_flexion=3.0,
    shoulder_turn=10.0,
    head_nod=2.0,
    arm_lowered=RUNNING_ARM_LOWERED,
    arm_inward_roll=RUNNING_ARM_INWARD_ROLL,
    wrist_flexion=RUNNING_WRIST_FLEXION,
    arm_forward=-20.0,
    arm_swing=35.0,
    elbow=75.0,
    elbow_swing=25.0,
)


def definition(rig: Rig):
    return gait_definition(rig, BACKWARD)
