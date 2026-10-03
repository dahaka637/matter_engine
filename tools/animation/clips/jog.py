"""Corrida leve - a passada padrao do personagem.

Um trote de jogador de futebol, inspirado (medido, nao copiado) em Slow Run.fbx
(assets/animations/source/mixamo/study; numeros em gait_metrics.py). Da
referencia vieram as caracteristicas, nao os valores crus:

  - ritmo: ~3 m/s a 164 passos/min, com voo entre um pe e outro;
  - o pe toca perto de baixo do corpo (a referencia pisa 17 cm a frente do
    quadril; a caminhada antiga pisava 36);
  - ombros girando com os bracos, cotovelo em torno de 90 graus, bracos mais
    para tras que para a frente, junto ao corpo (postura padrao de corrida,
    gait.RUNNING_ARM_*);
  - tronco inclinado para a frente.

Ficou de fora o que e exagero da referencia ou nao cabe no nosso corpo: a
pelve subindo e descendo 23 cm, o quadril caindo 8 graus e os pes na linha do
meio (aqui as coxas ja se tocam no repouso).

Anda para a frente em relacao a pelve; o runtime gira a pelve para o
movimento e o tronco devolve o giro para a camera.
"""

from __future__ import annotations

from gait import (RUNNING_ARM_INWARD_ROLL, RUNNING_ARM_LOWERED,
                  RUNNING_WRIST_FLEXION, GaitSpec, gait_definition)
from matter_rig import Rig

JOG = GaitSpec(
    id="jog",
    display_name="Corrida leve",
    speed=3.0,
    cadence=2.73,             # 164 passos/min, como a referencia
    duty=0.26,                # voo de ~33% (a referencia tem 41)
    stance_width=0.085,
    stance_center=-0.12,      # toque ~20 cm a frente do quadril (ref. 17)
    swing_height=0.18,
    swing_peak=0.4,
    swing_ankle_follow=0.6,
    swing_recovery=0.25,      # o calcanhar sobe atras antes de a perna vir
    roll_fraction=0.35,
    toe_off_pitch=25.0,
    landing_dorsiflex=4.0,
    toe_out=4.0,
    # A versao anterior baixava a pelve mais de 10 cm e mantinha o joelho de
    # apoio entre 51 e 61 graus: parecia uma corrida agachada. O jogador de
    # futebol trota alto, com flexao suficiente para absorver sem sentar.
    pelvis_drop=0.055,
    pelvis_bob=-0.016,
    pelvis_sway=0.012,
    pelvis_yaw=4.0,
    pelvis_list=4.0,
    lean_forward=4.0,
    lean_travel=2.0,
    spine_flexion=3.0,
    shoulder_turn=16.0,
    head_nod=2.0,
    arm_lowered=RUNNING_ARM_LOWERED,
    arm_inward_roll=RUNNING_ARM_INWARD_ROLL,
    wrist_flexion=RUNNING_WRIST_FLEXION,
    arm_forward=-15.0,        # o braco vive mais atras do tronco
    arm_swing=28.0,           # do ombro; o cotovelo quase nao muda
    elbow=90.0,
    elbow_swing=10.0,
)


def definition(rig: Rig):
    return gait_definition(rig, JOG)
