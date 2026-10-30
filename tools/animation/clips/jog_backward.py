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

28/10 noite (pedido do usuario: "mudar o movimento de trote para tras, para algo
estilo isso [Running Backward(2).fbx] menos exagerado"): o recuo atletico da
referencia, medido com gait_metrics.py (a referencia | este | o de antes):
cadencia 189 | 183 | 164 passos/min, passo 0,99 | 0,98 | 1,10 m, tronco 10,5 |
7,6 | 2,6 graus a frente, base 14,9 | 12,4 | 8,4 cm, pelve sobe-desce 7,4 | 5 | 3
cm, bracos atras do tronco com o cotovelo vivo. Menos exagerado que ela no que
ele ja recusou: o joelho no toque 45 graus (ela 63; sem "sentar") e a cabeca
quase reta (ela olha 21 graus para baixo).

29/10 noite (o corpo fisico): o sobe-desce da pelve de 5 cm a 183 passos/min fazia o
centro de massa cair a 1,6-2,5 g no voo (a auditoria da referencia do rastreador) - um
corpo fisico nao cai mais rapido que a gravidade, pousa atrasado e tropeca. 3,1 cm (o
do trote para a frente, 3 cm a 161 passos/min, cai a 1 g) leva o voo a ~1 g; o resto
do recuo atletico fica igual.
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
    lean_forward=6.0,
    lean_travel=0.0,
    stance_width=0.135,
    # Invertida no tempo, o apoio troca de lado (o centro vai ao longo do rumo):
    # a ponta toca atras e o pe sai a frente. (Mais perto do quadril que 0,06 a
    # perna nao alcanca o pe na passada curta.)
    stance_center=0.06,
    cadence=3.05,
    pelvis_drop=0.045,
    pelvis_bob=-0.031,
    neck_upright=0.0,
    # (A cabeca calma: com a cadencia e o sobe-desce maiores ela ia a 64 graus/s
    # de p95 no jogo; travada um pouco mais, 48.)
    head_bob=0.5,
    head_steady=0.95,
    arm_forward=-28.0,
    arm_lowered=69.0,
    elbow=80.0,
    elbow_swing=22.0,
)


def definition(rig: Rig):
    return gait_definition(rig, JOG_BACKWARD)
