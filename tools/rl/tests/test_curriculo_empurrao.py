"""O curriculo do empurrao por tipo (24/10): sobe depois de duas janelas boas, volta numa ruim,
nunca passa do topo nem desce da partida, e o continuo anda a duracao quando a forca chega ao topo."""
from __future__ import annotations

import numpy as np

from matter_rl.curriculum import PushCurriculum

COLUMNS = ["mundo", "exercicio", "razao", "empurrao_tipo"]
FALL = 3


def _task():
    rule = {"enabled": True, "window": 4, "raiseBelow": 0.15, "lowerAbove": 0.35,
            "short": {"start": [900, 1200], "step": 100, "maximum": 1400},
            "hold": {"start": [150, 220], "step": 25, "maximum": 245, "seconds": [1.0, 3.0],
                     "secondsStep": 0.5, "secondsMaximum": 4.0}}
    return {"drills": [{"id": "parado"},
                       {"id": "forte", "pushCurriculum": rule, "pushMinimumNewtons": 900, "pushMaximumNewtons": 1200,
                        "holdMinimumNewtons": 150, "holdMaximumNewtons": 220, "holdMinimumSeconds": 1.0,
                        "holdMaximumSeconds": 3.0}]}


def _window(curriculum, kind, falls, drill=1):
    rows = np.array([[0, drill, FALL if i < falls else 0, kind] for i in range(4)], dtype=np.float32)
    curriculum.observe(rows, COLUMNS, [FALL])


def test_push_curriculum_moves_each_kind_inside_its_limits():
    task = _task()
    forte = task["drills"][1]
    curriculum = PushCurriculum(task, ["parado", "forte"])
    assert curriculum.enabled and list(curriculum.rules) == ["forte"]
    # Uma janela boa nao basta; a segunda sobe o curto um passo (o continuo nao viu nada).
    _window(curriculum, 1, 0)
    assert curriculum.advance(task) == []
    _window(curriculum, 1, 0)
    assert curriculum.advance(task) and (forte["pushMinimumNewtons"], forte["pushMaximumNewtons"]) == (1000, 1300)
    # O topo corta o passo (1300 -> 1400, nao 1500) e depois nao anda mais.
    for _ in range(6):
        _window(curriculum, 1, 0)
        curriculum.advance(task)
    assert (forte["pushMinimumNewtons"], forte["pushMaximumNewtons"]) == (1100, 1400)
    # Uma janela ruim volta um passo; muitas param na faixa de partida.
    for _ in range(5):
        _window(curriculum, 1, 3)
        curriculum.advance(task)
    assert (forte["pushMinimumNewtons"], forte["pushMaximumNewtons"]) == (900, 1200)
    # Quedas no meio da faixa: fica.
    _window(curriculum, 1, 1)
    _window(curriculum, 1, 1)
    assert curriculum.advance(task) == [] and forte["pushMaximumNewtons"] == 1200
    # O continuo: a forca ate o topo (220 -> 245), depois a duracao (3 -> 3,5 -> 4); descendo, a duracao antes.
    for _ in range(6):
        _window(curriculum, 2, 0)
        _window(curriculum, 2, 0)
        curriculum.advance(task)
    assert (forte["holdMaximumNewtons"], forte["holdMaximumSeconds"]) == (245, 4.0)
    _window(curriculum, 2, 4)
    curriculum.advance(task)
    assert (forte["holdMaximumNewtons"], forte["holdMaximumSeconds"]) == (245, 3.5)
    # Os episodios sem empurrao (tipo 0) e os de outro exercicio nao contam.
    _window(curriculum, 0, 4)
    _window(curriculum, 1, 4, drill=0)
    assert curriculum.advance(task) == []
    snapshot = curriculum.snapshot(task)
    assert snapshot["forte"]["curto"] == [900, 1200] and snapshot["forte"]["continuo"][2] == 3.5
