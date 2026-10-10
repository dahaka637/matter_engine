"""Sprint de costas do modo padrao - longo e ereto, como a referencia.

Pedido do usuario (05/10): a corrida de costas como a do Sprint Backward.fbx,
com velocidade perto da corrida para a frente. Copiar a referencia nao cabe
(ela recua 1,84 m por passo a 7,35 m/s; o quadril do nosso rig nao estende
tanto e o pe de apoio dela bate no chao, +22/-18/+25 graus em tres quadros):
o recuo e gerado (gait.py) com o desenho dela, medido em gait_metrics - o
corpo ereto (tronco a ~3 graus; o nosso sprint de costas ia a 9), a pelve
alta (quadril 0,81 m acima do tornozelo; o nosso, agachado, 0,72), as pernas
mais estendidas no toque (joelho a ~40 graus; o nosso, 79) e a passada
longa, com muito voo. Os bracos sao os de corrida (a referencia segura um
fuzil). O sprint_backward de antes continua no modo fisico.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sprint_backward  # noqa: E402

from gait import gait_definition  # noqa: E402
from matter_rig import Rig  # noqa: E402

SPRINT_BACKWARD_LONG = replace(
    sprint_backward.SPRINT_BACKWARD,
    id="sprint_backward_long",
    display_name="Sprint de costas (longo)",
    speed=7.0,
    cadence=4.6,              # 276 passos/min: passo de 1,52 m
    duty=0.17,                # voo de ~57%
    # A pelve 8,5 cm abaixo de em pe (o sprint de costas: 10,8). Mais alta
    # (3 cm) o joelho tocava a ~59 graus, mas com o apoio curto que ela
    # exigia (12%) o pe freava de uma vez: pernas a 960 m/s2, joelho a
    # 29 rad/s (o sprint para a frente: 400). Assim: 620 e 16 rad/s.
    pelvis_drop=0.085,
    stance_center=0.0,
    arm_swing=40.0,
    lean_forward=2.0,
    spine_flexion=1.0,
)


def definition(rig: Rig):
    return gait_definition(rig, SPRINT_BACKWARD_LONG)
