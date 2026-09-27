"""Pulos: parado, correndo para a frente, para tras e para os lados.

Inspirados (medidos, nao copiados) em Jumping(2).fbx (parado), Jump.fbx
(correndo para a frente) e Jump(2).fbx (para tras; a primeira versao veio
de Jump(1).fbx). Das referencias vieram as
caracteristicas; o exagero ficou de fora, como o usuario pediu:

  - parado: pernas quase esticadas saindo do chao, recolhe no apice e
    estica de novo para receber o chao; bracos sobem a frente, mas NAO acima
    da cabeca (a referencia leva os bracos a 150 graus);
  - para a frente: salto de passada, em tesoura - a perna de impulso sai
    esticada para tras, a outra sobe com o joelho a frente e e ela que chega
    primeiro no chao; bracos contra as pernas, sem o giro largo da
    referencia, e com o cotovelo abaixo de 70 graus (perto de 90 o antebraco
    do ALS gira sozinho em volta do braco - ver gait.GaitSpec.pronation);
  - para tras: salto em tesoura de costas - impulso da perna da frente, a
    de tras recebe o chao, a da frente sobe no voo; tronco para tras no
    impulso e a frente na queda; bracos a frente equilibrando;
  - para os lados (sem referencia, pedido do usuario): impulso da perna de
    fora, a perna do lado do salto abre e recebe o chao, tronco inclinado
    para o lado do salto, bracos abertos equilibrando.

Fase 0 = sai do chao, 0,5 = apice, 1 = toca o chao. Direita = espelho da
esquerda.
"""

from __future__ import annotations

from dataclasses import replace

from jump import Arm, Foot, JumpKey, jump_definition
from matter_rig import Rig

STANDING = [
    JumpKey(0.0, left_foot=Foot(0.0, 0.0, 0.81, 30.0),
            right_foot=Foot(0.0, 0.0, 0.81, 30.0),
            left_arm=Arm(66, 30, 35), right_arm=Arm(66, 30, 35),
            pelvis_pitch=4, spine_flexion=4),
    JumpKey(0.3, left_foot=Foot(-0.04, 0.0, 0.68, 18.0),
            right_foot=Foot(-0.04, 0.0, 0.68, 18.0),
            left_arm=Arm(62, 50, 45), right_arm=Arm(62, 50, 45),
            pelvis_pitch=6, spine_flexion=6),
    JumpKey(0.5, left_foot=Foot(-0.02, 0.01, 0.58, 12.0),
            right_foot=Foot(-0.02, 0.01, 0.58, 12.0),
            left_arm=Arm(60, 55, 50), right_arm=Arm(60, 55, 50),
            pelvis_pitch=8, spine_flexion=8, head_nod=4),
    JumpKey(0.78, left_foot=Foot(0.04, 0.02, 0.72, 10.0),
            right_foot=Foot(0.04, 0.02, 0.72, 10.0),
            left_arm=Arm(64, 35, 40), right_arm=Arm(64, 35, 40),
            pelvis_pitch=5, spine_flexion=6),
    JumpKey(1.0, left_foot=Foot(0.05, 0.02, 0.80, 8.0),
            right_foot=Foot(0.05, 0.02, 0.80, 8.0),
            left_arm=Arm(66, 25, 35), right_arm=Arm(66, 25, 35),
            pelvis_pitch=4, spine_flexion=6),
]

# Salto de passada: impulso da direita, a esquerda sobe e recebe o chao.
# Bracos com a abertura da corrida (arm_lowered 70-74): saindo do trote com
# os bracos juntos ao corpo, abrir para 66 no voo deixava o antebraco
# direito 32 graus atras do alvo no corpo fisico (limite 30); com 70, 29.
FORWARD = [
    JumpKey(0.0, left_foot=Foot(0.22, 0.0, 0.52, 5.0),
            right_foot=Foot(-0.34, 0.0, 0.74, 30.0),
            left_arm=Arm(70, -35, 60), right_arm=Arm(70, 45, 68),
            pelvis_pitch=8, spine_flexion=4),
    JumpKey(0.35, left_foot=Foot(0.34, 0.0, 0.50, 5.0),
            right_foot=Foot(-0.34, 0.0, 0.58, 25.0, follow=0.8),
            left_arm=Arm(70, -30, 65), right_arm=Arm(70, 40, 68),
            pelvis_pitch=8, spine_flexion=4),
    JumpKey(0.6, left_foot=Foot(0.36, 0.0, 0.58, 5.0),
            right_foot=Foot(-0.22, 0.0, 0.50, 20.0, follow=0.8),
            left_arm=Arm(70, -20, 65), right_arm=Arm(70, 30, 68),
            pelvis_pitch=6, spine_flexion=4),
    JumpKey(0.85, left_foot=Foot(0.26, 0.0, 0.76, 2.0),
            right_foot=Foot(-0.06, 0.0, 0.54, 15.0, follow=0.5),
            left_arm=Arm(70, -10, 60), right_arm=Arm(70, 20, 68),
            pelvis_pitch=5, spine_flexion=4),
    JumpKey(1.0, left_foot=Foot(0.18, 0.0, 0.80, 0.0),
            right_foot=Foot(0.02, 0.0, 0.58, 12.0),
            left_arm=Arm(70, -5, 55), right_arm=Arm(70, 15, 65),
            pelvis_pitch=5, spine_flexion=4),
]

# Para tras (27/09, inspirado em Jump(2).fbx, sem os bracos dela): salto de
# costas em tesoura. O impulso e da direita, que fica a frente do corpo e sai
# esticada (quem salta para tras empurra com o pe adiante); a esquerda sobe
# dobrada atras, estica no voo e e ela que recebe o chao; a direita sobe
# dobrada a frente e desce depois do toque. Tronco um pouco para tras no
# impulso, em pe no apice e a frente na queda, como na referencia (-10, 0 e
# +12 graus). A perna da frente sobe menos que la (a referencia levanta o pe
# ate a altura do quadril). Bracos nossos: a frente e pouco abertos,
# equilibrando, o esquerdo mais a frente (contra a perna da frente) - a
# referencia os agita acima dos ombros.
BACKWARD = [
    JumpKey(0.0, left_foot=Foot(-0.28, 0.02, 0.42, 10.0, follow=0.8),
            right_foot=Foot(0.28, 0.0, 0.76, 28.0),
            left_arm=Arm(66, 22, 40), right_arm=Arm(66, 8, 35),
            pelvis_pitch=-4, spine_flexion=2),
    JumpKey(0.3, left_foot=Foot(-0.38, 0.03, 0.36, 10.0, follow=0.8),
            right_foot=Foot(0.38, 0.0, 0.70, 18.0),
            left_arm=Arm(62, 28, 45), right_arm=Arm(62, 4, 35),
            pelvis_pitch=0, spine_flexion=3),
    JumpKey(0.5, left_foot=Foot(-0.38, 0.03, 0.55, 5.0, follow=0.4),
            right_foot=Foot(0.44, 0.0, 0.45, 10.0),
            left_arm=Arm(60, 28, 45), right_arm=Arm(60, 0, 35),
            pelvis_pitch=4, spine_flexion=4, head_nod=2),
    JumpKey(0.8, left_foot=Foot(-0.20, 0.03, 0.76, 0.0),
            right_foot=Foot(0.40, 0.0, 0.48, 10.0),
            left_arm=Arm(62, 24, 40), right_arm=Arm(62, 4, 35),
            pelvis_pitch=8, spine_flexion=5),
    JumpKey(1.0, left_foot=Foot(-0.10, 0.03, 0.80, 0.0),
            right_foot=Foot(0.32, 0.0, 0.58, 10.0),
            left_arm=Arm(64, 20, 40), right_arm=Arm(64, 4, 35),
            pelvis_pitch=10, spine_flexion=6),
]

# Para a esquerda: impulso da direita (de fora), a esquerda abre e recebe o
# chao. pelvis_roll negativo e spine_lateral positivo inclinam para a esquerda.
LEFT = [
    JumpKey(0.0, left_foot=Foot(0.0, 0.14, 0.58, 5.0),
            right_foot=Foot(0.0, 0.16, 0.78, 30.0),
            left_arm=Arm(55, 10, 40), right_arm=Arm(55, 10, 40),
            pelvis_pitch=4, pelvis_roll=-6, spine_lateral=6, spine_flexion=4),
    JumpKey(0.5, left_foot=Foot(0.0, 0.20, 0.60, 5.0),
            right_foot=Foot(0.0, 0.04, 0.62, 15.0),
            left_arm=Arm(52, 12, 45), right_arm=Arm(52, 12, 45),
            pelvis_pitch=5, pelvis_roll=-4, spine_lateral=4, spine_flexion=5),
    JumpKey(1.0, left_foot=Foot(0.0, 0.22, 0.80, 5.0),
            right_foot=Foot(0.0, 0.02, 0.64, 12.0),
            left_arm=Arm(56, 10, 40), right_arm=Arm(56, 10, 40),
            pelvis_pitch=4, pelvis_roll=-4, spine_lateral=3, spine_flexion=5),
]


def _mirror(keys: list[JumpKey]) -> list[JumpKey]:
    """Esquerda <-> direita: troca pes e bracos e inverte as inclinacoes."""
    return [replace(k, left_foot=replace(k.right_foot),
                    right_foot=replace(k.left_foot),
                    left_arm=replace(k.right_arm),
                    right_arm=replace(k.left_arm),
                    pelvis_roll=-k.pelvis_roll,
                    spine_lateral=-k.spine_lateral) for k in keys]


def definitions(rig: Rig):
    return [
        jump_definition(rig, "jump_standing", "Pulo parado", STANDING),
        jump_definition(rig, "jump_forward", "Pulo correndo", FORWARD),
        jump_definition(rig, "jump_backward", "Pulo para tras", BACKWARD,
                        direction_degrees=180.0),
        jump_definition(rig, "jump_left", "Pulo para a esquerda", LEFT,
                        direction_degrees=90.0),
        jump_definition(rig, "jump_right", "Pulo para a direita",
                        _mirror(LEFT), direction_degrees=-90.0),
    ]
