"""O treino no PC: modos de desempenho, prioridade, temperatura e memoria."""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

RESTART_CODE = 75  # o supervisor reabre o treino (troca de modo)


@dataclass(frozen=True)
class Mode:
    name: str
    label: str
    threads: int
    idle: bool  # prioridade ociosa: so usa o que o resto do PC deixa
    nice: int


MODES = {
    "maximo": Mode("maximo", "Máximo", 10, False, 10),
    "equilibrado": Mode("equilibrado", "Equilibrado", 8, True, 19),
    "leve": Mode("leve", "Leve", 4, True, 19),
}
ORDER = ["maximo", "equilibrado", "leve"]


def mode(name: str) -> Mode:
    if name not in MODES:
        raise ValueError(f"modo desconhecido: {name} (maximo, equilibrado, leve)")
    return MODES[name]


def threads_for(mode_: Mode) -> int:
    return max(1, min(mode_.threads, (os.cpu_count() or 2) - 1))


def apply_priority(mode_: Mode) -> None:
    """Aplica a prioridade a todas as threads do processo (as novas herdam)."""
    for task in Path("/proc/self/task").iterdir():
        tid = int(task.name)
        try:
            if mode_.idle:
                os.sched_setscheduler(tid, os.SCHED_IDLE, os.sched_param(0))
            else:
                os.sched_setscheduler(tid, os.SCHED_OTHER, os.sched_param(0))
            os.setpriority(os.PRIO_PROCESS, tid, mode_.nice)
        except (PermissionError, ProcessLookupError, OSError):
            pass


def cpu_temperature() -> float | None:
    """Tctl do Ryzen (k10temp) em graus, ou a maior temperatura de CPU que achar."""
    best = None
    for hwmon in Path("/sys/class/hwmon").glob("hwmon*"):
        try:
            name = (hwmon / "name").read_text().strip()
        except OSError:
            continue
        if name not in ("k10temp", "coretemp", "zenpower"):
            continue
        for sensor in hwmon.glob("temp*_input"):
            try:
                value = int(sensor.read_text()) / 1000.0
            except (OSError, ValueError):
                continue
            best = value if best is None else max(best, value)
    return best


def memory_mb() -> float:
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return 0.0
