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
    # 12 threads (16/10): a fisica e o gargalo e as 12 threads do Ryzen 5 3600 rendem ~+10% no treino real.
    "maximo": Mode("maximo", "Máximo", 12, False, 10),
    "equilibrado": Mode("equilibrado", "Equilibrado", 8, True, 19),
    "leve": Mode("leve", "Leve", 4, True, 19),
}
ORDER = ["maximo", "equilibrado", "leve"]


def mode(name: str) -> Mode:
    if name not in MODES:
        raise ValueError(f"modo desconhecido: {name} (maximo, equilibrado, leve)")
    return MODES[name]


def threads_for(mode_: Mode) -> int:
    # MATTER_RL_THREADS fixa as threads da fisica (a medida da vazao; ate todas).
    forced = os.environ.get("MATTER_RL_THREADS")
    if forced:
        return max(1, min(int(forced), os.cpu_count() or 1))
    return max(1, min(mode_.threads, os.cpu_count() or 1))


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


def tuned_environment() -> dict[str, str]:
    """As constantes de calibragem lidas do ambiente pelo motor (T_*: os
    TEMPTUNE do controle, ~580 leituras). Uma esquecida num terminal muda o
    treino, a sonda ou o jogo sem rastro (vault 02_estrutura/estudo_desempenho.md,
    atleta-11)."""
    import re
    return {k: v for k, v in os.environ.items() if re.match(r"^T_[A-Za-z]", k)}


def refuse_tuned_environment(what: str) -> dict[str, str]:
    """Recusa comecar com T_* definidas, salvo MATTER_RL_PERMITIR_T=1 (e entao
    devolve o que estava definido, para registrar)."""
    tuned = tuned_environment()
    if tuned and os.environ.get("MATTER_RL_PERMITIR_T") != "1":
        names = ", ".join(sorted(tuned))
        raise RuntimeError(f"{what}: constantes de calibragem no ambiente ({names}); tire-as ou use "
                           "MATTER_RL_PERMITIR_T=1 para rodar com elas de proposito")
    return tuned
