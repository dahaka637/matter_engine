"""O curriculo de velocidade por celulas (vault 04_locomocao/referencia/pesquisas/
04_curriculo_e_ajuda.md e 08_plano_consenso.md, S1).

A celula e o rumo do pedido em relacao a camera (8 setores de 45 graus) e o modo do
peito (camera parada ou girando): 16 por exercicio. Cada uma tem um teto de
velocidade que corta o que o piloto pede (so no treino; a avaliacao e o portao pedem
a faixa inteira). O teto abre um degrau quando a celula ja rastreia o pedido de
agora na fronteira (as decisoes de cruzeiro com o pedido ate frontier abaixo do teto) - o
nucleo de velocidade medio >= tracking (0,8: erro ~0,24 m/s, a regua do legged_gym) - sem
cair mais que maxFalls, com pelo menos minEpisodes
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


def carry_caps(old: dict, new: dict) -> dict:
    """A troca de tarefa (o degrau seguinte, o mesmo cerebro): os tetos ja abertos seguem
    nos exercicios em comum, para o degrau novo nao devolver o antigo ao teto de partida."""
    caps = (old.get("speedCurriculum") or {}).get("caps", {})
    ids = {d["id"] for d in new.get("drills", [])}
    if caps and (new.get("speedCurriculum") or {}).get("enabled"):
        new["speedCurriculum"].setdefault("caps", {}).update({d: c for d, c in caps.items() if d in ids})
    return new


class SpeedCurriculum:
    def __init__(self, task: dict, drill_ids: list[str]):
        self.rule = CurriculumRule.from_task(task)
        self.drill_ids = list(drill_ids)
        self.records = {d: [deque(maxlen=self.rule.window) for _ in range(CELLS)] for d in self.drill_ids}
        self.since = {d: [0] * CELLS for d in self.drill_ids}
        self.base_aid = float((task.get("assist") or {}).get("standing", 0.0))

    def initial_caps(self, task: dict) -> dict:
        """Os tetos de partida dos exercicios que se movem (os que ja estao na tarefa ficam).
        O exercicio novo de um degrau nasce na fronteira que o corpo ja tem em cada celula
        (o maior teto dos que ja estao), nunca abaixo do teto de partida."""
        block = task.setdefault("speedCurriculum", {})
        caps = block.setdefault("caps", {})
        frontier = [max((cells[k] for cells in caps.values()), default=0.0) for k in range(CELLS)]
        for drill in task.get("drills", []):
            top = band_maximum(drill)
            if top <= 0.0 or drill["id"] in caps:
                continue
            start = float(self.rule.start.get(drill["id"], self.rule.start_default))
            if len(self.rule.bin_start) == CELLS // 2:
                first = [self.rule.bin_start[k // 2] for k in range(CELLS)]
            else:
                first = [start] * CELLS
            caps[drill["id"]] = [round(min(top, max(first[k], frontier[k])), 4) for k in range(CELLS)]
        return caps

    def observe(self, rows: np.ndarray, columns: list[str]) -> None:
        """Os episodios que acabaram: o rastreio e a queda de cada celula por onde passaram."""
        if not len(rows):
            return
        col = {name: i for i, name in enumerate(columns)}
        aid = col.get("ajuda_nivel")
        for row in rows:
            # So os episodios na ajuda base da tarefa (a ajuda sorteada, S4, fica de fora): o
            # teto abre pelo que o corpo faz sem ajuda, nao ajudado (o A/B S4 abriu 15 tetos contra 8).
            if aid is not None and row[aid] > self.base_aid + 1e-3:
                continue
            drill = self.drill_ids[int(row[col["exercicio"]])]
            fall_cell = int(row[col["celula_queda"]])
            for k in range(CELLS):
                # So a fronteira conta (o pedido no teto da celula): os pedidos baixos
                # rastreados nao podem abrir o teto alto (revisao de 18/10).
                cruise = row[col[f"cel{k:02d}.fronteira"]]
                if cruise < self.rule.min_cruise:
                    continue
                tracking = float(row[col[f"cel{k:02d}.rastreio_fronteira"]] / cruise)
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


# O tipo do empurrao na coluna empurrao_tipo do episodio.
PUSH_KINDS = {1: "curto", 2: "continuo"}


class PushCurriculum:
    """O curriculo do empurrao por tipo (24/10, pedido do usuario: o empurrao forte sem plato;
    desenho com o Codex). Nos exercicios com pushCurriculum, cada tipo (o curto e o continuo)
    anda a faixa um passo quando as quedas dele ficam abaixo de raiseBelow em duas janelas
    seguidas e volta um passo quando passam de lowerAbove (nunca abaixo da faixa de partida).
    O curto anda a forca ate maximum; o continuo anda a forca e, no topo, a duracao maxima.
    As faixas moram nos campos do exercicio na copia da tarefa (o lote le em update_task);
    as janelas se refazem numa retomada."""

    def __init__(self, task: dict, drill_ids: list[str]):
        self.drill_ids = list(drill_ids)
        self.rules = {d["id"]: d["pushCurriculum"] for d in task.get("drills", [])
                      if (d.get("pushCurriculum") or {}).get("enabled") and d["id"] in self.drill_ids}
        self.records = {(d, k): [] for d in self.rules for k in PUSH_KINDS}
        self.good = {key: 0 for key in self.records}

    @property
    def enabled(self) -> bool:
        return bool(self.rules)

    def observe(self, rows: np.ndarray, columns: list[str], fall_codes: list[int]) -> None:
        if not self.rules or not len(rows):
            return
        col = {name: i for i, name in enumerate(columns)}
        for row in rows:
            key = (self.drill_ids[int(row[col["exercicio"]])], int(row[col["empurrao_tipo"]]))
            if key in self.records:
                self.records[key].append(int(row[col["razao"]]) in fall_codes)

    def advance(self, task: dict) -> list[str]:
        """Anda as faixas que fecharam janela; devolve o que mudou (texto)."""
        drills = {d["id"]: d for d in task.get("drills", [])}
        moved = []
        for (drill, kind), record in self.records.items():
            rule = self.rules[drill]
            if len(record) < int(rule.get("window", 256)) or drill not in drills:
                continue
            rate = sum(record) / len(record)
            record.clear()
            step = 0
            if rate > float(rule.get("lowerAbove", 0.35)):
                self.good[(drill, kind)], step = 0, -1
            elif rate < float(rule.get("raiseBelow", 0.15)):
                self.good[(drill, kind)] += 1
                if self.good[(drill, kind)] >= 2:
                    self.good[(drill, kind)], step = 0, 1
            else:
                self.good[(drill, kind)] = 0
            text = self.step(drills[drill], kind, rule, step) if step else None
            if text:
                moved.append(f"{drill} {PUSH_KINDS[kind]}: {text} (quedas {rate:.0%})")
        return moved

    @staticmethod
    def step(drill: dict, kind: int, rule: dict, sign: int) -> str | None:
        part = rule["short" if kind == 1 else "hold"]
        low_key, high_key = ("pushMinimumNewtons", "pushMaximumNewtons") if kind == 1 \
            else ("holdMinimumNewtons", "holdMaximumNewtons")
        low, high = float(drill[low_key]), float(drill[high_key])
        top, start_low = float(part["maximum"]), float(part["start"][0])
        if kind == 2:
            # (Sobe: a forca ate o topo, depois a duracao; desce: a duracao primeiro.)
            seconds, first = float(drill["holdMaximumSeconds"]), float(part["seconds"][1])
            if (sign > 0 and high >= top - 1e-6) or (sign < 0 and seconds > first + 1e-6):
                limit = float(part["secondsMaximum"]) if sign > 0 else first
                new = min(limit, seconds + float(part["secondsStep"])) if sign > 0 \
                    else max(limit, seconds - float(part["secondsStep"]))
                if abs(new - seconds) < 1e-6:
                    return None
                drill["holdMaximumSeconds"] = round(new, 3)
                return f"duracao ate {seconds:.1f} -> {new:.1f} s"
        shift = min(float(part["step"]), top - high) if sign > 0 else max(-float(part["step"]), start_low - low)
        if abs(shift) < 1e-6:
            return None
        drill[low_key], drill[high_key] = round(low + shift, 2), round(high + shift, 2)
        return f"{low:.0f}-{high:.0f} -> {low + shift:.0f}-{high + shift:.0f} N"

    def snapshot(self, task: dict) -> dict:
        """O estado para a telemetria: as faixas de cada tipo e a janela atual."""
        out = {}
        for d in task.get("drills", []):
            if d["id"] not in self.rules:
                continue
            out[d["id"]] = {
                "curto": [d.get("pushMinimumNewtons"), d.get("pushMaximumNewtons")],
                "continuo": [d.get("holdMinimumNewtons"), d.get("holdMaximumNewtons"), d.get("holdMaximumSeconds")],
                "janela": {PUSH_KINDS[k]: [len(self.records[(d["id"], k)]),
                                          (sum(self.records[(d["id"], k)]) / len(self.records[(d["id"], k)])
                                           if self.records[(d["id"], k)] else None)] for k in PUSH_KINDS}}
        return out
