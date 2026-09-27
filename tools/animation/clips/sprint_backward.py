"""Sprint de costas - recuar o mais rapido que da, olhando para a frente.

E o recuo (run_backward) com o corpo pedindo velocidade: o que o jogador faz
segurando sprint com o movimento para tras da camera - um zagueiro
acompanhando o atacante sem dar as costas para a bola. Nada de meia-volta.

Correr de costas rapido e questao de ritmo, nao de passada: o quadril nao
estende muito (limite do rig: 30 graus em relacao a pelve) e o pe que toca
atras nao pode ir longe. Entao, em relacao ao recuo:

  - ~4,4 m/s (recuo: 2,9), cadencia de 216 passos/min (189) e passo de
    1,2 m (0,9), com mais voo;
  - joelho mais alto no balanco e a ponta tocando mais atras;
  - tronco mais a frente, que e o contrapeso de quem acelera para tras;
  - bracos mais amplos e com mais forca, os ombros girando junto.
"""

from __future__ import annotations

from gait import (RUNNING_ARM_INWARD_ROLL, RUNNING_ARM_LOWERED,
                  RUNNING_WRIST_FLEXION, GaitSpec, gait_definition)
from matter_rig import Rig

SPRINT_BACKWARD = GaitSpec(
    id="sprint_backward",
    display_name="Sprint de costas",
    speed=4.4,
    direction_degrees=180.0,
    cadence=3.6,              # 216 passos/min
    duty=0.21,                # voo de 45% (gait_metrics)
    stance_width=0.11,
    stance_center=-0.03,
    swing_height=0.16,
    swing_peak=0.5,
    roll_at_touchdown=True,
    roll_fraction=0.3,
    toe_off_pitch=30.0,
    landing_dorsiflex=0.0,
    toe_out=5.0,
    pelvis_drop=0.108,         # preserva altura no apoio ao reduzir o bob
    pelvis_bob=-0.022,
    pelvis_sway=0.012,
    pelvis_yaw=8.0,
    pelvis_list=3.0,
    lean_forward=7.0,
    lean_travel=0.0,
    spine_flexion=4.0,
    shoulder_turn=14.0,
    head_nod=2.0,
    arm_lowered=RUNNING_ARM_LOWERED,
    arm_inward_roll=RUNNING_ARM_INWARD_ROLL,
    wrist_flexion=RUNNING_WRIST_FLEXION,
    arm_forward=-15.0,
    arm_swing=46.0,
    elbow=80.0,
    elbow_swing=20.0,
)


def definition(rig: Rig):
    return gait_definition(rig, SPRINT_BACKWARD)
