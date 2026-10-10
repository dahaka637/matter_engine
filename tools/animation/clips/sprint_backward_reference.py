"""Sprint de costas do modo padrao, fiel a referencia Sprint Backward.fbx.

Pedido do usuario (05/10): a corrida de costas (e as diagonais de costas, que
usam este clipe com a pelve girada) como a da referencia, com velocidade
perto da corrida para a frente. A referencia corre de costas a ~7,3 m/s
segurando um fuzil: os bracos dela ficam de fora - balancam pelas pernas
(cada um com a coxa do outro lado), na postura de corrida. O resto e o mesmo
caminho do trote (clips/trot.py): retarget cru, pe de apoio parado e plano,
chao, coxas, pes e bracos sem se tocar, quadros intermediarios por IK.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import trot  # noqa: E402

from authoring import ClipDefinition  # noqa: E402
from matter_rig import Rig  # noqa: E402

# Bracos: fracao do angulo da coxa oposta e o cotovelo (graus) - mais
# dobrado que no trote, como numa corrida.
ARMS_FROM_LEGS = 1.0
ARM_SWING_SHARE = 0.55
ARM_ELBOW = 88.0
# O pe de apoio da referencia bate no chao (+22, -18 e +25 graus em tres
# quadros): com o limite de 20 graus so um quadro plantava, e o joelho
# passava de 30 rad/s saindo dele. Os tres plantam.
STANCE_PITCH = 40.0
BLEND_FRAMES = 3
PLANT_AT_LANDING = False
REACH_MARGIN = 0.01


def definitions(rig: Rig) -> list[ClipDefinition]:
    return [trot._clip(rig, trot._gait(rig, trot.ReferenceGait, "sprint_backward_reference", 180.0,
                                       plant_at_landing=PLANT_AT_LANDING, arms_from_legs=ARMS_FROM_LEGS,
                                       arm_swing_share=ARM_SWING_SHARE, arm_elbow=ARM_ELBOW,
                                       blend_frames=BLEND_FRAMES, reach_margin=REACH_MARGIN,
                                       stance_pitch=STANCE_PITCH),
                       "sprint_backward_reference", "Sprint de costas (referencia)", 180.0)]
