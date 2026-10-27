"""Sprint maximo - o sprint do Shift duplo (24/10, pedido do usuario).

A nossa corrida (sprint.py) com so duas mudancas:
  - o tronco mais inclinado: pela pelve (15 graus a mais), a coluna nao curva mais que na corrida;
  - a passada mais longa: a mesma cadencia (4,3 passos/s) com mais velocidade - passo de 2,00 m (era 1,74).
Para a passada caber na perna do rig, o pe fica menos no chao (apoio 0,19 do ciclo, era 0,23; o resto vira voo) e a
pelve desce 2 cm (assim passa limpo na validacao: junta maxima 23 rad/s). Mais que 2,0 m o rig nao alcanca (medido: a 2,09 m o tornozelo passa 28-42 mm do alcance e o
quadril passa do limite de extensao).
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sprint  # noqa: E402

from gait import gait_definition  # noqa: E402
from matter_rig import Rig  # noqa: E402

SPRINT_MAXIMO = replace(
    sprint.SPRINT,
    id="sprint_maximo",
    display_name="Sprint maximo",
    speed=8.6,
    lean_forward=20.0,
    duty=0.19,
    pelvis_drop=0.055,
)


def definition(rig: Rig):
    return gait_definition(rig, SPRINT_MAXIMO)
