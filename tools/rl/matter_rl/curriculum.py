"""O curriculo de velocidade por celulas (vault 04_locomocao/referencia/pesquisas/
04_curriculo_e_ajuda.md e 08_plano_consenso.md, S1).

A celula e o rumo do pedido em relacao a camera (8 setores de 45 graus) e o modo do
peito (camera parada ou girando): 16 por exercicio. Cada uma tem um teto de
velocidade que corta o que o piloto pede (so no treino; a avaliacao e o portao pedem
a faixa inteira). O teto abre um degrau quando a celula ja rastreia o pedido de
agora - o nucleo de velocidade medio no cruzeiro >= tracking (0,8: erro ~0,24 m/s,
a regua do legged_gym) - sem cair mais que maxFalls, com pelo menos minEpisodes
episodios desde a ultima abertura (a janela: os ultimos window). Nunca fecha.

O estado que importa (os tetos) mora no tarefa.json do treino; as contagens se
refazem depois de uma retomada.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

CELLS = 16
BIN_NAMES = ("frente", "diag_esq", "esquerda", "tras_esq", "costas", "tras_dir", "direita", "diag_dir")


def cell_name(cell: int) -> str:
    return f"{BIN_NAMES[cell // 2]}/{'girando' if cell % 2 else 'fixo'}"


@dataclass
class CurriculumRule:
    enabled: bool = False
    step: float = 0.25
    min_episodes: int = 32
    window: int = 64
    tracking: float = 0.8
    max_falls: float = 0.15
    # O minimo de decisoes de cruzeiro (30 Hz) para um episodio contar numa celula.
    min_cruise: int = 15
    # O teto de partida (m/s): um para todos, ou por exercicio.
    start: dict = field(default_factory=dict)
    start_default: float = 1.0
    # O teto de partida por setor de rumo (8 valores, frente primeiro, a esquerda
    # positiva; a medida M0.1 de 18/10): vale sobre o start quando dado.
    bin_start: list = field(default_factory=list)

    @staticmethod
    def from_task(task: dict) -> "CurriculumRule":
        block = task.get("speedCurriculum") or {}
        start = block.get("start", 1.0)
        return CurriculumRule(
            enabled=bool(block.get("enabled", False)), step=float(block.get("step", 0.25)),
            min_episodes=int(block.get("minEpisodes", 32)), window=int(block.get("window", 64)),
            tracking=float(block.get("tracking", 0.8)), max_falls=float(block.get("maxFalls", 0.15)),
            min_cruise=int(block.get("minCruise", 15)),
            start=dict(start) if isinstance(start, dict) else {},
            start_default=float(start) if not isinstance(start, dict) else float(block.get("startDefault", 1.0)),
            bin_start=[float(v) for v in block.get("binStart", [])])


def band_maximum(drill: dict) -> float:
    """A maior velocidade que o roteiro do exercicio pede (o teto nao passa dela)."""
    bands = (drill.get("pilot") or {}).get("bands") or []
    speeds = [float(b.get("speed", [0.0, 0.0])[1]) for b in bands if isinstance(b.get("speed"), list)]
    return max(speeds) if speeds else 0.0


class SpeedCurriculum:
    def __init__(self, task: dict, drill_ids: list[str]):
        self.rule = CurriculumRule.from_task(task)
        self.drill_ids = list(drill_ids)
        self.records = {d: [deque(maxlen=self.rule.window) for _ in range(CELLS)] for d in self.drill_ids}
        self.since = {d: [0] * CELLS for d in self.drill_ids}

    def initial_caps(self, task: dict) -> dict:
        """Os tetos de partida dos exercicios que se movem (os que ja estao na tarefa ficam)."""
        block = task.setdefault("speedCurriculum", {})
        caps = block.setdefault("caps", {})
        for drill in task.get("drills", []):
            top = band_maximum(drill)
            if top <= 0.0 or drill["id"] in caps:
                continue
            start = float(self.rule.start.get(drill["id"], self.rule.start_default))
            if len(self.rule.bin_start) == CELLS // 2:
                caps[drill["id"]] = [round(min(top, self.rule.bin_start[k // 2]), 4) for k in range(CELLS)]
            else:
                caps[drill["id"]] = [round(min(top, start), 4)] * CELLS
        return caps

    def observe(self, rows: np.ndarray, columns: list[str]) -> None:
        """Os episodios que acabaram: o rastreio e a queda de cada celula por onde passaram."""
        if not len(rows):
            return
        col = {name: i for i, name in enumerate(columns)}
        for row in rows:
            drill = self.drill_ids[int(row[col["exercicio"]])]
            fall_cell = int(row[col["celula_queda"]])
            for k in range(CELLS):
                cruise = row[col[f"cel{k:02d}.cruzeiro"]]
                if cruise < self.rule.min_cruise:
                    continue
                tracking = float(row[col[f"cel{k:02d}.rastreio"]] / cruise)
                self.records[drill][k].append((tracking, fall_cell == k))
                self.since[drill][k] += 1

    def advance(self, task: dict) -> list[str]:
        """Abre os tetos que passaram na regra; devolve o que abriu (texto)."""
        if not self.rule.enabled:
            return []
        caps = task.get("speedCurriculum", {}).get("caps", {})
        tops = {d["id"]: band_maximum(d) for d in task.get("drills", [])}
        opened = []
        for drill, cells in caps.items():
            if drill not in self.records:
                continue
            for k in range(CELLS):
                window = self.records[drill][k]
                if self.since[drill][k] < self.rule.min_episodes or cells[k] >= tops.get(drill, 0.0) - 1e-6:
                    continue
                tracking = float(np.mean([t for t, _ in window]))
                falls = float(np.mean([f for _, f in window]))
                if tracking >= self.rule.tracking and falls <= self.rule.max_falls:
                    old = cells[k]
                    cells[k] = round(min(tops[drill], old + self.rule.step), 4)
                    window.clear()
                    self.since[drill][k] = 0
                    opened.append(f"{drill} {cell_name(k)}: {old:.2f} -> {cells[k]:.2f} m/s "
                                  f"(rastreio {tracking:.2f}, quedas {falls:.0%})")
        return opened

    def snapshot(self, task: dict) -> dict:
        """O estado para a telemetria: o teto de cada celula e a janela atual."""
        caps = task.get("speedCurriculum", {}).get("caps", {})
        out = {}
        for drill, cells in caps.items():
            if drill not in self.records:
                continue
            out[drill] = {cell_name(k): {"teto": cells[k], "episodios": self.since[drill][k],
                                         "rastreio": (float(np.mean([t for t, _ in self.records[drill][k]]))
                                                      if self.records[drill][k] else None)}
                          for k in range(CELLS)}
        return out
