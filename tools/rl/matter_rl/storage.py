"""Persistencia: o banco SQLite (historico) e os checkpoints (o estado inteiro).

O banco guarda o que aconteceu (rodadas, avaliacoes, eventos) e cataloga os
checkpoints; o estado para continuar o treino fica em arquivos ao lado, gravados
de forma atomica (diretorio temporario renomeado de uma vez).
"""
from __future__ import annotations

import fcntl
import json
import os
import shutil
import sqlite3
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from . import paths

SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    name TEXT PRIMARY KEY,
    task TEXT NOT NULL,
    parent TEXT,
    parent_checkpoint TEXT,
    created_utc TEXT NOT NULL,
    body_hash TEXT,
    config TEXT
);
CREATE TABLE IF NOT EXISTS sessions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run TEXT NOT NULL,
    start_utc TEXT NOT NULL,
    finish_utc TEXT,
    start_iteration INTEGER,
    mode TEXT,
    device TEXT,
    pid INTEGER,
    end_reason TEXT
);
CREATE TABLE IF NOT EXISTS iterations (
    run TEXT NOT NULL,
    iteration INTEGER NOT NULL,
    time REAL NOT NULL,
    steps INTEGER NOT NULL,
    sps REAL,
    reward REAL,
    episodes INTEGER,
    episode_return REAL,
    episode_seconds REAL,
    fall_fraction REAL,
    components TEXT,
    ends TEXT,
    policy_loss REAL,
    value_loss REAL,
    entropy REAL,
    kl REAL,
    clip_fraction REAL,
    learning_rate REAL,
    action_std REAL,
    grad_norm REAL,
    collect_seconds REAL,
    update_seconds REAL,
    velocity_tracking REAL,
    forward_limit REAL,
    PRIMARY KEY (run, iteration)
);
CREATE TABLE IF NOT EXISTS evaluations (
    run TEXT NOT NULL,
    iteration INTEGER NOT NULL,
    time REAL NOT NULL,
    steps INTEGER NOT NULL,
    battery TEXT NOT NULL,
    solver TEXT NOT NULL,
    stand_fraction REAL,
    mean_seconds REAL,
    sway REAL,
    foot_drift REAL,
    action_rate REAL,
    effort REAL,
    upright REAL,
    score REAL,
    ends TEXT,
    turn REAL,
    joint_speed REAL,
    foot_turn REAL,
    pose_error REAL,
    velocity_error REAL,
    lateral_drift REAL,
    goal_error REAL,
    targets REAL,
    reached_fraction REAL,
    first_target_seconds REAL,
    target_speed REAL,
    imitation_error REAL,
    facing_error REAL,
    speed_error REAL,
    foot_slip REAL,
    stop_overshoot REAL,
    line_error REAL,
    cadence REAL,
    chatter REAL,
    reversal REAL,
    cut_seconds REAL,
    jump_run REAL,
    jump_stand REAL,
    jump_rise_error REAL,
    landing_falls REAL,
    jump_recovery REAL,
    push_recovery REAL,
    PRIMARY KEY (run, iteration, battery, solver)
);
CREATE TABLE IF NOT EXISTS checkpoints (
    run TEXT NOT NULL,
    iteration INTEGER NOT NULL,
    time REAL NOT NULL,
    steps INTEGER NOT NULL,
    path TEXT NOT NULL,
    kind TEXT NOT NULL,
    score REAL,
    PRIMARY KEY (run, iteration)
);
CREATE TABLE IF NOT EXISTS events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run TEXT,
    time REAL NOT NULL,
    kind TEXT NOT NULL,
    author TEXT NOT NULL,
    message TEXT,
    data TEXT
);
CREATE INDEX IF NOT EXISTS iterations_time ON iterations(run, time);
CREATE INDEX IF NOT EXISTS events_time ON events(run, time);
"""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class Database:
    """Uma conexao por processo. O treinador escreve; a tela e a CLI leem."""

    def __init__(self, path: Path = paths.DATABASE, readonly: bool = False):
        path.parent.mkdir(parents=True, exist_ok=True)
        if readonly:
            self.connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True, timeout=10)
        else:
            self.connection = sqlite3.connect(str(path), timeout=30)
            self.connection.execute("PRAGMA journal_mode=WAL")
            self.connection.execute("PRAGMA synchronous=NORMAL")
            self.connection.executescript(SCHEMA)
            # Colunas que entraram depois (bancos de antes delas).
            known = {row[1] for row in self.connection.execute("PRAGMA table_info(evaluations)")}
            for column in ("turn", "joint_speed", "foot_turn", "pose_error", "velocity_error", "lateral_drift",
                           "goal_error", "targets", "reached_fraction", "first_target_seconds", "target_speed",
                           "imitation_error", "facing_error", "speed_error", "foot_slip", "stop_overshoot", "line_error", "cadence", "chatter", "reversal", "cut_seconds", "jump_run", "jump_stand",
                           "jump_rise_error", "landing_falls", "jump_recovery", "push_recovery"):
                if column not in known:
                    self.connection.execute(f"ALTER TABLE evaluations ADD COLUMN {column} REAL")
            known = {row[1] for row in self.connection.execute("PRAGMA table_info(iterations)")}
            for column in ("velocity_tracking", "forward_limit"):
                if column not in known:
                    self.connection.execute(f"ALTER TABLE iterations ADD COLUMN {column} REAL")
            self.connection.commit()
        self.connection.row_factory = sqlite3.Row

    def close(self) -> None:
        self.connection.close()

    def execute(self, sql: str, values: tuple | dict = ()) -> sqlite3.Cursor:
        return self.connection.execute(sql, values)

    def commit(self) -> None:
        self.connection.commit()

    # --- escrita -------------------------------------------------------------
    def ensure_run(self, name: str, task: str, body_hash: str, config: dict, parent: str | None = None,
                   parent_checkpoint: str | None = None) -> None:
        self.execute("""INSERT OR IGNORE INTO runs(name, task, parent, parent_checkpoint, created_utc, body_hash, config)
                        VALUES (?, ?, ?, ?, ?, ?, ?)""",
                     (name, task, parent, parent_checkpoint, utc_now(), body_hash, json.dumps(config)))
        self.commit()

    def begin_session(self, run: str, iteration: int, mode: str, device: str) -> int:
        # A trava do treino garante um processo por vez: uma sessao ainda aberta
        # e de um processo que morreu sem fechar (queda de energia, kill).
        self.execute("UPDATE sessions SET finish_utc=?, end_reason='interrompida' WHERE run=? AND finish_utc IS NULL",
                     (utc_now(), run))
        cursor = self.execute("""INSERT INTO sessions(run, start_utc, start_iteration, mode, device, pid)
                                 VALUES (?, ?, ?, ?, ?, ?)""", (run, utc_now(), iteration, mode, device, os.getpid()))
        self.commit()
        return int(cursor.lastrowid)

    def end_session(self, session: int, reason: str) -> None:
        self.execute("UPDATE sessions SET finish_utc=?, end_reason=? WHERE id=?", (utc_now(), reason, session))
        self.commit()

    def log_iteration(self, row: dict) -> None:
        columns = ", ".join(row)
        marks = ", ".join(f":{key}" for key in row)
        self.execute(f"INSERT OR REPLACE INTO iterations({columns}) VALUES ({marks})", row)

    def log_evaluation(self, row: dict) -> None:
        columns = ", ".join(row)
        marks = ", ".join(f":{key}" for key in row)
        self.execute(f"INSERT OR REPLACE INTO evaluations({columns}) VALUES ({marks})", row)
        self.commit()

    def log_checkpoint(self, run: str, iteration: int, steps: int, path: Path, kind: str, score: float | None) -> None:
        self.execute("""INSERT OR REPLACE INTO checkpoints(run, iteration, time, steps, path, kind, score)
                        VALUES (?, ?, ?, ?, ?, ?, ?)""", (run, iteration, time.time(), steps, str(path), kind, score))
        self.commit()

    def forget_checkpoint(self, run: str, iteration: int) -> None:
        self.execute("DELETE FROM checkpoints WHERE run=? AND iteration=?", (run, iteration))
        self.commit()

    def event(self, run: str | None, kind: str, message: str, author: str = "treinador", data: Any = None) -> None:
        self.execute("INSERT INTO events(run, time, kind, author, message, data) VALUES (?, ?, ?, ?, ?, ?)",
                     (run, time.time(), kind, author, message, json.dumps(data) if data is not None else None))
        self.commit()

    def rewind(self, run: str, iteration: int) -> int:
        """Apaga as rodadas e avaliacoes depois do checkpoint retomado (o processo
        caiu antes do proximo): o historico segue numa linha so."""
        lost = self.execute("SELECT COUNT(*) FROM iterations WHERE run=? AND iteration>?", (run, iteration)).fetchone()[0]
        self.execute("DELETE FROM iterations WHERE run=? AND iteration>?", (run, iteration))
        self.execute("DELETE FROM evaluations WHERE run=? AND iteration>?", (run, iteration))
        self.commit()
        return int(lost)

    def last_evaluation(self, run: str) -> dict:
        """A avaliacao mais recente do treino, no formato do status (a tela e a
        CLI mostram a ultima nota logo depois de reabrir)."""
        rows = self.execute("""SELECT * FROM evaluations WHERE run=? AND iteration=
                               (SELECT MAX(iteration) FROM evaluations WHERE run=?)""", (run, run)).fetchall()
        if not rows:
            return {}
        from .evaluation import BASE_BATTERY, battery_kind, battery_posture
        keys = ("stand_fraction", "mean_seconds", "sway", "foot_drift", "action_rate", "effort", "upright", "turn",
                "joint_speed", "foot_turn", "pose_error", "velocity_error", "lateral_drift", "goal_error", "score")
        batteries: dict[str, dict] = {}
        for row in rows:
            entry = batteries.setdefault(row["battery"], {"posture": battery_posture(row["battery"]),
                                                          "kind": battery_kind(row["battery"]), "solvers": {}})
            entry["solvers"][row["solver"]] = ({key: row[key] if key in row.keys() else None for key in keys}
                                               | {"ends": json.loads(row["ends"] or "{}")})
        first = batteries.get(BASE_BATTERY) or next(iter(batteries.values()))
        return {"iteration": rows[0]["iteration"],
                "score": min(m["score"] for b in batteries.values() for m in b["solvers"].values()),
                "solvers": first["solvers"], "batteries": batteries, "seconds": 0.0,
                "time": max(row["time"] for row in rows)}

    def sampled_iterations(self, run: str, points: int = 2000, since: float = 0.0) -> list[sqlite3.Row]:
        """Ate `points` rodadas espalhadas no tempo (a mais recente de cada janela).
        Por tempo, e nao pelo numero da rodada: as antigas, ja compactadas em uma
        a cada 10 min, continuam nas curvas."""
        first, last = self.execute("SELECT MIN(time), MAX(time) FROM iterations WHERE run=? AND time>=?",
                                   (run, since)).fetchone()
        if first is None:
            return []
        window = max((last - first) / max(1, points), 1e-6)
        # Com um MAX() so, o SQLite tira as outras colunas da mesma linha.
        return self.execute("""SELECT MAX(iteration) AS iteration, * FROM iterations WHERE run=? AND time>=?
                               GROUP BY CAST((time - ?) / ? AS INTEGER) ORDER BY 1""",
                            (run, since, first, window)).fetchall()

    def compact(self, run: str, keep_days: float = 3.0) -> int:
        """Rodadas com mais de 3 dias viram uma a cada 10 min (a sanitizacao do
        historico: ~100 MB por dia de treino viram ~0,1 MB)."""
        limit = time.time() - keep_days * 86400.0
        rows = self.execute("""SELECT iteration, time FROM iterations WHERE run=? AND time<? ORDER BY time""",
                            (run, limit)).fetchall()
        drop, last_kept = [], -1e18
        for row in rows:
            if row["time"] - last_kept >= 600.0:
                last_kept = row["time"]
            else:
                drop.append((run, row["iteration"]))
        self.connection.executemany("DELETE FROM iterations WHERE run=? AND iteration=?", drop)
        self.commit()
        return len(drop)


# --- trava de um escritor por treino -------------------------------------------
class RunLock:
    """flock no arquivo `lock` do treino: some sozinho se o processo morrer."""

    def __init__(self, run_directory: Path):
        run_directory.mkdir(parents=True, exist_ok=True)
        self.path = run_directory / "lock"
        self.handle = open(self.path, "a+")
        try:
            fcntl.flock(self.handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self.handle.seek(0)
            owner = self.handle.read().strip()
            self.handle.close()
            raise RuntimeError(f"o treino ja esta aberto por outro processo ({owner or '?'})") from error
        self.handle.seek(0)
        self.handle.truncate()
        self.handle.write(f"{os.getpid()}\n")
        self.handle.flush()

    def release(self) -> None:
        try:
            fcntl.flock(self.handle, fcntl.LOCK_UN)
        finally:
            self.handle.close()


def run_locked(run_directory: Path) -> bool:
    path = run_directory / "lock"
    if not path.exists():
        return False
    with open(path, "a+") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        fcntl.flock(handle, fcntl.LOCK_UN)
    return False


# --- checkpoints ----------------------------------------------------------------
@dataclass
class CheckpointRef:
    iteration: int
    path: Path
    manifest: dict


def _fsync_directory(path: Path) -> None:
    fd = os.open(path, os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _write_json_atomic(path: Path, data: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with open(temporary, "w") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def save_checkpoint(run_directory: Path, iteration: int, state: dict, manifest: dict) -> CheckpointRef:
    """Grava ck-<iteracao>/ (model.pt + manifest.json) e aponta latest.json para ele."""
    import torch  # so aqui: o supervisor e a CLI nao carregam o PyTorch
    directory = run_directory / "checkpoints"
    directory.mkdir(parents=True, exist_ok=True)
    final = directory / f"ck-{iteration:09d}"
    temporary = directory / f"ck-{iteration:09d}.tmp"
    if temporary.exists():
        shutil.rmtree(temporary)
    temporary.mkdir()
    torch.save(state, temporary / "model.pt")
    manifest = dict(manifest, iteration=iteration, saved_utc=utc_now())
    with open(temporary / "manifest.json", "w") as handle:
        json.dump(manifest, handle, indent=2, ensure_ascii=False)
        handle.flush()
        os.fsync(handle.fileno())
    with open(temporary / "model.pt", "rb") as handle:
        os.fsync(handle.fileno())
    if final.exists():
        shutil.rmtree(final)
    os.replace(temporary, final)
    _fsync_directory(directory)
    _write_json_atomic(run_directory / "latest.json", {"iteration": iteration, "path": final.name})
    return CheckpointRef(iteration, final, manifest)


def list_checkpoints(run_directory: Path) -> list[CheckpointRef]:
    directory = run_directory / "checkpoints"
    result = []
    if not directory.exists():
        return result
    for item in sorted(directory.glob("ck-*")):
        if item.suffix == ".tmp" or not (item / "manifest.json").exists() or not (item / "model.pt").exists():
            continue
        try:
            manifest = json.loads((item / "manifest.json").read_text())
        except (OSError, json.JSONDecodeError):
            continue
        result.append(CheckpointRef(int(manifest.get("iteration", int(item.name[3:]))), item, manifest))
    return result


def latest_checkpoint(run_directory: Path) -> CheckpointRef | None:
    pointer = run_directory / "latest.json"
    if pointer.exists():
        try:
            data = json.loads(pointer.read_text())
            item = run_directory / "checkpoints" / data["path"]
            if (item / "manifest.json").exists() and (item / "model.pt").exists():
                return CheckpointRef(int(data["iteration"]), item, json.loads((item / "manifest.json").read_text()))
        except (OSError, KeyError, ValueError, json.JSONDecodeError):
            pass
    found = list_checkpoints(run_directory)
    return found[-1] if found else None


def load_checkpoint(reference: CheckpointRef, device: str = "cpu") -> dict:
    import torch
    return torch.load(reference.path / "model.pt", map_location=device, weights_only=True)


def clean_temporaries(run_directory: Path) -> int:
    """Checkpoints que ficaram pela metade (o PC desligou gravando)."""
    count = 0
    for item in (run_directory / "checkpoints").glob("*.tmp"):
        shutil.rmtree(item, ignore_errors=True)
        count += 1
    return count


def prune_checkpoints(run_directory: Path, protected: set[int], now: float | None = None) -> list[int]:
    """A sanitizacao dos checkpoints. Ficam: os 3 ultimos, os protegidos (o
    melhor de cada etapa, os fixados), 1 por hora nas ultimas 24 h e 1 por dia
    antes disso. O resto e apagado."""
    now = time.time() if now is None else now
    found = list_checkpoints(run_directory)
    keep: set[int] = {ref.iteration for ref in found[-3:]} | set(protected)
    buckets: dict[tuple[str, int], int] = {}
    for ref in found:
        saved = ref.manifest.get("time", ref.path.stat().st_mtime)
        age = now - saved
        bucket = ("h", int(saved // 3600)) if age < 86400 else ("d", int(saved // 86400))
        buckets.setdefault(bucket, ref.iteration)
    keep |= set(buckets.values())
    removed = []
    for ref in found:
        if ref.iteration not in keep:
            shutil.rmtree(ref.path, ignore_errors=True)
            removed.append(ref.iteration)
    return removed


def directory_size_mb(path: Path) -> float:
    total = 0
    for item in path.rglob("*"):
        if item.is_file():
            total += item.stat().st_size
    return total / (1024.0 * 1024.0)


def write_json_atomic(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_atomic(path, data)
