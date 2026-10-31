"""O curriculo de velocidade nos exercicios que trocam de direcao (31/10): a queda do episodio conta em todas as
celulas da fronteira por onde ele passou (a troca cai depois da troca, em outra celula); no reto, so na celula
da queda, como antes."""
from __future__ import annotations

import numpy as np

from matter_rl.curriculum import CELLS, SpeedCurriculum

COLUMNS = (["exercicio", "celula_queda"] + [f"cel{k:02d}.fronteira" for k in range(CELLS)]
           + [f"cel{k:02d}.rastreio_fronteira" for k in range(CELLS)])


def _task():
    return {"speedCurriculum": {"enabled": True, "minCruise": 1},
            "drills": [{"id": "frente", "pilot": {"kind": "stick"}},
                       {"id": "troca", "pilot": {"kind": "stick", "changeDegrees": [120, 180]}}]}


def _row(drill, fall_cell):
    # a fronteira nas celulas 0 (frente) e 8 (costas), a queda na 8
    cruise = [0.0] * CELLS
    cruise[0] = cruise[8] = 10.0
    return [drill, fall_cell] + cruise + [c * 0.9 for c in cruise]


def test_change_drill_falls_count_in_every_visited_cell():
    curriculum = SpeedCurriculum(_task(), ["frente", "troca"])
    curriculum.observe(np.array([_row(0, 8), _row(1, 8)], dtype=np.float32), COLUMNS)
    assert curriculum.records["frente"][0][-1][1] is False and curriculum.records["frente"][8][-1][1] is True
    assert curriculum.records["troca"][0][-1][1] is True and curriculum.records["troca"][8][-1][1] is True


def test_change_drill_without_fall_counts_none():
    curriculum = SpeedCurriculum(_task(), ["frente", "troca"])
    curriculum.observe(np.array([_row(1, -1)], dtype=np.float32), COLUMNS)
    assert curriculum.records["troca"][0][-1][1] is False and curriculum.records["troca"][8][-1][1] is False
