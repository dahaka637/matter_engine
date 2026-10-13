"""O aprendizado guiado: a memoria das decisoes do professor, o peso que some,
os mundos do professor (a janela de queda) e a fase inteira no treinador (a
imitacao, o critico sozinho primeiro e o termo de imitacao no PPO)."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import torch

from matter_rl.imitation import Demonstrations, ImitationSettings, decay_weight


def test_demonstrations_ring():
    memory = Demonstrations(5, 3, 2, torch.device("cpu"))
    memory.add(torch.ones(3, 3), torch.ones(3, 2))
    assert memory.size == 3 and memory.cursor == 3
    memory.add(torch.full((4, 3), 2.0), torch.full((4, 2), 2.0))
    assert memory.size == 5 and memory.cursor == 2
    # As mais antigas sairam: so uma das primeiras ficou.
    assert int((memory.obs[:, 0] == 1.0).sum()) == 1
    obs, actions = memory.sample(8)
    assert obs.shape == (8, 3) and actions.shape == (8, 2)


def test_settings_and_decay():
    settings = ImitationSettings.from_dict({"minutes": 3, "actionStd": 0.4, "decayIterations": 100,
                                            "decayWeight": 0.5, "desconhecida": 1})
    assert settings.minutes == 3 and settings.action_std == 0.4 and settings.decay_iterations == 100
    assert decay_weight(settings, 0) == 0.5 and decay_weight(settings, 50) == 0.25 and decay_weight(settings, 200) == 0.0


SCRIPT = r'''
import os, sys, math
from pathlib import Path
data = Path(sys.argv[1])
os.environ["MATTER_RL_DATA"] = str(data)
os.environ["MATTER_RL_SOCKET"] = str(data / "controle.sock")
sys.path.insert(0, sys.argv[2])
# O andar por comando (a H3 v6, com o professor): a H3 do repositorio virou a corrida ate o alvo.
WALKING = Path(sys.argv[2]).resolve().parents[1] / "tests" / "RL" / "data" / "andar_comando.json"
import numpy as np
import torch
from matter_rl import native, paths, trainer as T
from matter_rl.imitation import HOLD, TeacherWorlds

# Os mundos do professor: so saem as decisoes que passaram da janela de queda.
worlds = TeacherWorlds(native.load(), WALKING, 4, 2, 5, torch.device("cpu"))
released = 0
for _ in range(HOLD + 10):
    obs, actions = worlds.step()
    assert obs.shape[0] == actions.shape[0] and torch.isfinite(actions).all()
    released += obs.shape[0]
assert 0 < released <= 10 * 4, released
del worlds

t = T.Trainer("t", task=str(WALKING), mode="maximo", environments=16, device="cpu", seed=3, iterations=3)
t.config.evaluation_minutes = 1e9
try:
    t.command("professor", {"minutos": 0.1})
    raise SystemExit("aceitou 0,1 min")
except ValueError:
    pass
# (Na tarefa de agora o professor esta desligado: o pedido e recusado.)
try:
    t.command("professor", {})
    raise SystemExit("aceitou o professor desligado")
except ValueError:
    pass
import json
live = json.loads(t.live_task.read_text())
live["teacher"]["enabled"] = True
t.live_task.write_text(json.dumps(live))
assert t.command("professor", {})["ok"] and t.imitation_request is not None
t.imitation_request = None
t.imitate({"minutes": 0.05, "environments": 8, "batch_size": 256, "capacity": 20000, "critic_iterations": 1,
           "decay_iterations": 2})
assert t.demonstrations is not None and t.demonstrations.size > 256
assert abs(float(t.model.log_std.exp().mean()) - 0.45) < 1e-4
kinds = [k for (k,) in t.db.execute("SELECT kind FROM events WHERE run='t' AND kind='professor'")]
assert len(kinds) == 2, kinds
# A primeira rodada depois e so do critico: o ator nao muda.
before = [p.detach().clone() for p in t.model.actor.parameters()]
t.iteration()
assert all(torch.equal(a, b) for a, b in zip(before, t.model.actor.parameters())), "o ator mudou na rodada do critico"
assert t.status()["kickstart"]["weight"] > 0.0
t.iteration()
assert any(not torch.equal(a, b) for a, b in zip(before, t.model.actor.parameters())), "o ator nao voltou a aprender"
t.iteration()
t.iteration()
assert t.kickstart is None, "a imitacao sai no fim"
t.server.close()
t.lock.release()
print("ok")
'''


def test_teacher_phase(tmp_path: Path):
    script = tmp_path / "professor.py"
    script.write_text(SCRIPT)
    package = Path(__file__).resolve().parents[1]
    result = subprocess.run([sys.executable, str(script), str(tmp_path / "dados"), str(package)],
                            capture_output=True, text=True, timeout=900)
    assert result.returncode == 0 and "ok" in result.stdout.splitlines(), result.stdout + result.stderr
