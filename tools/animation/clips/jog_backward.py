"""Trote de costas como o trote para a frente (28/10, pedido do usuario).

"Refazer o movimento de andar para tras (e diagonais para tras) do TROTE (o
para corrida permanece o mesmo), eu quero o mesmo tipo de movimento do trotar
para frente e diagonais para frente no para tras (o que vai mudar e a
inclinacao da coluna obviamente, e a ordem dos passos obviamente...), pois nao
gosto da forma como ele anda para tras 'sentado'."

A receita do trote para a frente (clips/jog.py): a mesma pelve alta, a mesma
passada, os mesmos bracos e o mesmo tronco vivo. O "sentado" da corrida de
costas (clips/run_backward.py) vinha da pelve 7,7 cm abaixo do repouso (no
trote, 3,2), do joelho bem dobrado e do tronco 7 graus a frente. Muda so o que
recuar pede:
  - o rumo, 180 graus (a ordem dos passos vem dele);
  - a rolagem no toque: a ponta toca primeiro e o calcanhar desce - a passada
    para a frente invertida no tempo, com o apoio do outro lado do quadril (sem
    a recuperacao do calcanhar, que e do comeco do balanco para a frente, nem a
    ponta para cima no fim dele);
  - o tronco quase ereto (no trote ele vai 6 graus a frente, no rumo);
  - a base um pouco mais larga.
A corrida de costas fica para o sprint e para a sombra do RL.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import jog  # noqa: E402

from gait import gait_definition  # noqa: E402
from matter_rig import Rig  # noqa: E402

JOG_BACKWARD = replace(
    jog.JOG,
    id="jog_backward",
    display_name="Trote de costas",
    direction_degrees=180.0,
    roll_at_touchdown=True,
    swing_recovery=0.0,
    swing_ankle_follow=0.0,
    landing_dorsiflex=0.0,
    lean_forward=1.0,
    lean_travel=0.0,
    stance_width=0.095,
    # Invertida no tempo, o apoio troca de lado (o centro vai ao longo do rumo):
    # a ponta toca bem atras (onde a do trote sai) e o pe sai chato a frente
    # (onde o do trote toca).
    stance_center=-jog.JOG.stance_center,
)


def definition(rig: Rig):
    return gait_definition(rig, JOG_BACKWARD)
