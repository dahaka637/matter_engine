"""Corrida de costas no jeito do trote de costas (05/11 03h15, pedido do usuario).

"No corrida de costas (diagonais tambem de costas) seria melhor usarmos algo mais
semelhante com o trote costas (claro permitindo melhor desenvoltura das pernas), pois o
clipe que usamos atualmente para a corrida de costas deixa a base muito baixa, o que fica
meio estranho."

O sprint_backward abaixa a pelve 10,8 cm (o trote de costas, 4,5). Este e o trote de costas
(clips/jog_backward.py) com a velocidade do sprint_backward (4,4 m/s, para a referencia da
corrida mudar so na forma): a pelve alta dele, a passada mais longa e mais rapida, mais voo,
o joelho subindo mais no balanco e os bracos mais amplos.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import jog_backward  # noqa: E402

from gait import gait_definition  # noqa: E402
from matter_rig import Rig  # noqa: E402

RUN_BACKWARD_TROTE = replace(
    jog_backward.JOG_BACKWARD,
    id="run_backward_trote",
    display_name="Corrida de costas (do trote)",
    speed=4.4,
    cadence=3.4,          # 204 passos/min (trote de costas 183, sprint de costas 216)
    duty=0.23,
    swing_height=0.20,
    pelvis_drop=0.05,
    lean_forward=7.0,
    arm_swing=40.0,
    elbow_swing=24.0,
)


def definition(rig: Rig):
    return gait_definition(rig, RUN_BACKWARD_TROTE)
