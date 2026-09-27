"""Sprint - corrida a toda velocidade.

Corre no antepe: o calcanhar nao toca, o apoio e curto (um quarto do ciclo)
e ha fase de voo entre um pe e outro. Na recuperacao o calcanhar sobe em
direcao ao gluteo e o joelho vem alto e a frente, antes de a perna estender
para o proximo toque. Tronco inclinado para a frente, cotovelos em torno de 95 graus
balancando forte e contra as pernas.

Inspiracao leve em Running(2).fbx (medida com gait_metrics.py, nao
copiada): pisar mais perto de baixo do corpo (toque a ~29 cm do quadril, eram
38), calcanhar subindo mais atras e ombros girando com os bracos (16 graus,
eram 4). A referencia e uma corrida de 4 m/s; do sprint continuam a
velocidade, a cadencia, o antepe e o joelho alto - a 7,5 m/s a coxa precisa
subir. O cotovelo da referencia (73-117) nao coube: no ritmo do sprint o
braco atrasa um pouco na fisica, e se o cotovelo nao abre abaixo de ~70 no
balanco para tras a mao engancha no quadril e na outra mao (medido: antebraco
travado a 150 graus do alvo). Aberto a 62 atras, o corpo acompanha.

So existe para a frente: ninguem corre de costas a 7,5 m/s. Fora do cone de
sprint (movimento muito para tras da camera) o runtime volta para a
corrida de costas.
"""

from __future__ import annotations

from gait import (RUNNING_ARM_INWARD_ROLL, RUNNING_ARM_LOWERED,
                  RUNNING_WRIST_FLEXION, GaitSpec, gait_definition)
from matter_rig import Rig

SPRINT = GaitSpec(
    id="sprint",
    display_name="Sprint",
    speed=7.5,
    cadence=4.3,              # passos/s; passada de 1,74 m
    duty=0.23,
    stance_width=0.095,       # os pes quase numa linha: as coxas ja se tocam
    stance_center=-0.11,      # alcance: a perna chega na frente e atras
    swing_height=0.42,        # recuperacao: calcanhar em direcao ao gluteo
    swing_peak=0.38,
    swing_ankle_follow=0.85,  # na recuperacao o pe acompanha a canela
    swing_recovery=0.2,
    stance_pitch=6.0,         # antepe: o calcanhar nao toca
    roll_fraction=0.35,
    toe_off_pitch=30.0,
    landing_dorsiflex=0.0,
    toe_out=3.0,
    pelvis_drop=0.087,        # preserva altura no apoio ao reduzir o bob
    pelvis_bob=-0.018,        # 3,6 cm de oscilacao; preserva cadencia e apoio
    pelvis_sway=0.006,
    pelvis_yaw=8.0,
    pelvis_list=3.5,
    # A pelve inclinada e o que deixa o quadril estender na saida do pe
    # (limite do rig: 30 graus de extensao em relacao a pelve).
    lean_forward=5.0,
    lean_travel=4.0,
    spine_flexion=5.0,
    shoulder_turn=16.0,
    head_nod=2.0,
    arm_lowered=RUNNING_ARM_LOWERED,
    arm_inward_roll=RUNNING_ARM_INWARD_ROLL,
    wrist_flexion=RUNNING_WRIST_FLEXION,
    arm_forward=-10.0,
    arm_swing=50.0,
    elbow=82.0,               # 70 atras, 94 a frente
    elbow_swing=12.0,
)


def definition(rig: Rig):
    return gait_definition(rig, SPRINT)
