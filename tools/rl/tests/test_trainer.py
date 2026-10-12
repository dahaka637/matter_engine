"""O treinador inteiro num treino pequeno (16 mundos, CPU, uma pasta temporaria):
parar e continuar do checkpoint, e a rede que vira NaN volta ao ultimo
checkpoint bom sem deixar rodadas perdidas no historico."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

SCRIPT = r'''
import os, sys
from pathlib import Path
data = Path(sys.argv[1])
os.environ["MATTER_RL_DATA"] = str(data)
os.environ["MATTER_RL_SOCKET"] = str(data / "controle.sock")
sys.path.insert(0, sys.argv[2])
import torch
from matter_rl import storage, trainer as T

def make(iterations):
    t = T.Trainer("t", mode="maximo", environments=16, device="cpu", seed=3, iterations=iterations)
    t.config.evaluation_minutes = 1e9  # sem a bateria (lenta) no teste
    return t

first = make(3)
assert first.loop() == 0
assert storage.latest_checkpoint(first.directory).iteration == 3

second = make(6)
assert second.counters["iteration"] == 3, "continua do checkpoint"
original, calls = second.iteration, {"n": 0}
def poisoned():
    result = original()
    calls["n"] += 1
    if calls["n"] == 2:  # a rodada 5 estraga a rede
        with torch.no_grad():
            next(second.model.parameters()).fill_(float("nan"))
    return result
second.iteration = poisoned
assert second.loop() == 0
assert all(torch.isfinite(p).all() for p in second.model.parameters())

db = storage.Database(data / "rl.db", readonly=True)
kinds = [row[0] for row in db.execute("SELECT kind FROM events WHERE run='t' ORDER BY id")]
assert "recuperacao" in kinds, kinds
recovered = db.execute("SELECT time FROM events WHERE kind='recuperacao'").fetchone()[0]
rows = db.execute("SELECT iteration, time FROM iterations WHERE run='t' ORDER BY iteration").fetchall()
assert [r[0] for r in rows] == [1, 2, 3, 4, 5, 6], [r[0] for r in rows]
assert rows[3][1] > recovered, "a rodada 4 de antes da recuperacao saiu do historico"
assert [r[1] for r in rows] == sorted(r[1] for r in rows)
sessions = db.execute("SELECT end_reason FROM sessions WHERE run='t' ORDER BY id").fetchall()
assert [s[0] for s in sessions] == ["parado", "parado"], [s[0] for s in sessions]
print("ok")
'''


def test_trainer_resume_and_nan_recovery(tmp_path: Path):
    script = tmp_path / "treino.py"
    script.write_text(SCRIPT)
    package = Path(__file__).resolve().parents[1]
    result = subprocess.run([sys.executable, str(script), str(tmp_path / "dados"), str(package)],
                            capture_output=True, text=True, timeout=600)
    assert result.returncode == 0 and "ok" in result.stdout.splitlines(), result.stdout + result.stderr
