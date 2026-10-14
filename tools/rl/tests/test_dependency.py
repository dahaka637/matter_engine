"""O teste de dependencia do Corpo v2 (plano_claude_rl.md secao 12.6): num
empacotamento sem os arquivos de animacao (o corpo, o perfil e a tarefa numa
pasta temporaria; nada se apaga do projeto), o lote roda, o treino sem o juiz
treina e exporta, e a politica exportada roda no runtime em C++. A entrada
exportada nao tem canal de clipe, de alvo ou de exercicio."""
from __future__ import annotations

import json
import shutil
import struct

import numpy as np
import pytest

from matter_rl import native, paths, storage

module = native.load()


def package(directory, reference_fraction: float = 0.0):
    """O Corpo v2 empacotado sem animacoes: o acervo aponta para onde nao ha nada."""
    (directory / "bodies").mkdir(parents=True)
    (directory / "tasks").mkdir()
    (directory / "characters").mkdir()
    shutil.copy(paths.REPO / "assets" / "characters" / "football_player" / "FootballPlayerV1.ragdoll.json",
                directory / "characters" / "perfil.ragdoll.json")
    body = json.loads((paths.ASSETS / "rl" / "bodies" / "football_player_v2.json").read_text())
    body["profile"] = "../characters/perfil.ragdoll.json"
    (directory / "bodies" / "corpo.json").write_text(json.dumps(body))
    task = json.loads(paths.task_path("corpo2").read_text())
    task["body"] = "../bodies/corpo.json"
    task["style"]["dataset"] = "../style/nao_existe.json"
    task["initialState"]["referenceFraction"] = reference_fraction
    path = directory / "tasks" / "corpo2.json"
    path.write_text(json.dumps(task))
    assert not list(directory.rglob("*.matteranim.json"))
    return path


def test_package_really_has_no_animations(tmp_path):
    # Com a fatia do acervo ligada, o lote precisa dos clipes: no pacote, falha.
    with pytest.raises(Exception):
        module.BodyBatch(str(package(tmp_path, reference_fraction=0.4)), environments=2, per_scene=2, threads=1)


def test_training_without_animation_files(tmp_path, monkeypatch):
    from matter_rl.body_trainer import BodyTrainer
    task = package(tmp_path / "pacote")
    batch = module.BodyBatch(str(task), environments=4, per_scene=2, threads=1, seed=2)
    zero = np.zeros((4, batch.action_size), dtype=np.float32)
    for _ in range(30):
        batch.step(zero, zero)
        assert np.isfinite(batch.observations()).all()
    # O treino sem o juiz (o fim do caminho: beta zero e nenhum clipe).
    monkeypatch.setattr(paths, "DATA", tmp_path / "dados")
    monkeypatch.setattr(paths, "RUNS", tmp_path / "dados" / "runs")
    monkeypatch.setattr(paths, "DATABASE", tmp_path / "dados" / "rl.db")
    trainer = BodyTrainer("sem_clipes", task=str(task), environments=8, seed=1, iterations=2, style=False,
                          socket_path=tmp_path / "s.sock")
    trainer.settings.steps_per_env = 8
    trainer.config.evaluation_minutes = 0.0
    assert trainer.judge is None
    assert trainer.loop() == 0
    exported = paths.run_dir("sem_clipes") / "exports" / "policy_latest.mrlpolicy"
    # A entrada exportada: nada de clipe, alvo, exercicio ou fase.
    data = exported.read_bytes()
    size = struct.unpack_from("<I", data, 8)[0]
    names = json.loads(data[12:12 + size])["observationNames"]
    assert names == batch.observation_names()
    assert not [n for n in names if n.startswith(("reference.", "target.", "drill.", "jump.zone", "pilot."))]
    # A politica roda no runtime em C++ sobre o lote do pacote.
    policy = module.Policy(str(exported))
    for _ in range(10):
        action = np.ascontiguousarray(policy.evaluate(np.ascontiguousarray(batch.observations(), dtype=np.float32)),
                                      dtype=np.float32)
        batch.step(action, action)
        assert np.isfinite(batch.rewards()).all()
    rows = storage.Database(paths.DATABASE).execute("SELECT COUNT(*) FROM iterations WHERE run='sem_clipes'").fetchone()
    assert rows[0] == 2
