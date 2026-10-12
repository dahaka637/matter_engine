"""O treinador inteiro num treino pequeno (16 mundos, CPU, uma pasta temporaria):
parar e continuar do checkpoint, a rede que vira NaN voltando ao ultimo
checkpoint bom sem deixar rodadas perdidas no historico, e a troca de
habilidade (H1 -> postura de alerta) no mesmo cerebro."""
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
import json
import torch
from matter_rl import storage, trainer as T
from matter_rl.evaluation import Battery
T.Battery = lambda: Battery(environments=4, seconds=1.0)  # a bateria pequena do teste

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

# A habilidade seguinte no mesmo cerebro: a tarefa do treino troca, a rede e a
# mesma (o comando da postura ja estava na entrada), cada habilidade tem o
# seu melhor e as duas baterias rodam.
second.evaluate()
h1_best = dict(second.best)
third = T.Trainer("t", task="h1b_em_pe_posturado", mode="maximo", environments=16, device="cpu", seed=3, iterations=8)
third.config.evaluation_minutes = 60.0  # so a avaliacao imediata da habilidade nova
assert third.switched_from == "h1_ficar_em_pe"
assert json.loads(third.live_task.read_text())["id"] == "h1b_em_pe_posturado"
assert third.obs_names == second.obs_names and third.obs_names[-1] == "command.posture.alerta"
assert third.counters["iteration"] == 6 and third.best["iteration"] == -1
assert third.bests["h1_ficar_em_pe"]["iteration"] == h1_best["iteration"]
assert third.loop() == 0  # avalia logo na primeira rodada (habilidade nova)
batteries = {row[0] for row in db.execute("SELECT DISTINCT battery FROM evaluations WHERE run='t' AND iteration > 6")}
assert batteries == {"equilibrio_v1", "postura_alerta_v1"}, batteries
assert set(third.last_evaluation["batteries"]) == batteries
assert third.bests["h1b_em_pe_posturado"]["iteration"] > 6 and "h1_ficar_em_pe" in third.bests
assert third.best["version"] == T.SCORE_VERSION
i = third.obs_names.index("command.posture.alerta")
assert float(third.model.obs_norm.mean[i]) == 0.0 and float(third.model.obs_norm.var[i]) == 1.0
events = [row[0] for row in db.execute("SELECT kind FROM events WHERE run='t' ORDER BY id")]
assert "tarefa" in events and "migracao" not in events, events

print("ok")
'''


def test_trainer_resume_and_nan_recovery(tmp_path: Path):
    script = tmp_path / "treino.py"
    script.write_text(SCRIPT)
    package = Path(__file__).resolve().parents[1]
    result = subprocess.run([sys.executable, str(script), str(tmp_path / "dados"), str(package)],
                            capture_output=True, text=True, timeout=600)
    assert result.returncode == 0 and "ok" in result.stdout.splitlines(), result.stdout + result.stderr
