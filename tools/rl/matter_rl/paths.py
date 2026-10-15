"""Onde ficam o repositorio, os treinos, o socket e o modulo nativo."""
from __future__ import annotations

import os
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
ASSETS = REPO / "assets"
TASKS = ASSETS / "rl" / "tasks"
DATA = Path(os.environ.get("MATTER_RL_DATA", Path.home() / ".local" / "share" / "matter-engine" / "rl"))
RUNS = DATA / "runs"
DATABASE = DATA / "rl.db"
RUNTIME = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) / "matter-rl"
SOCKET = Path(os.environ.get("MATTER_RL_SOCKET", RUNTIME / "control.sock"))
NATIVE_DIR = Path(os.environ.get("MATTER_RL_NATIVE", REPO / "build-rl" / "python"))


def task_path(task: str) -> Path:
    """Aceita o id da tarefa (aquisicao_p3a) ou um caminho."""
    candidate = Path(task)
    if candidate.suffix == ".json" and candidate.exists():
        return candidate.resolve()
    return TASKS / f"{task}.json"


def run_dir(run: str) -> Path:
    return RUNS / run


def inspector() -> Path | None:
    """O MatterModelInspector mais recente entre as builds (para as folhas)."""
    found = [p for p in REPO.glob("build-*/MatterModelInspector") if p.is_file()]
    return max(found, key=lambda p: p.stat().st_mtime) if found else None
