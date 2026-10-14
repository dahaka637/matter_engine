"""O treinador do Corpo v2 num treino minusculo, numa pasta temporaria (nada
toca os treinos de verdade nem o banco deles)."""
from __future__ import annotations

import json

import numpy as np
import pytest
import torch

from matter_rl import paths, storage


@pytest.fixture()
def scratch(tmp_path, monkeypatch):
    monkeypatch.setattr(paths, "DATA", tmp_path)
    monkeypatch.setattr(paths, "RUNS", tmp_path / "runs")
    monkeypatch.setattr(paths, "DATABASE", tmp_path / "rl.db")
    return tmp_path


def tiny(trainer_class, scratch, run, **kwargs):
    trainer = trainer_class(run, task="corpo2", environments=16, seed=3, iterations=3,
                            socket_path=scratch / f"{run}.sock", **kwargs)
    # Rodadas curtas e o juiz pequeno (a mecanica, nao o aprendizado).
    trainer.settings.steps_per_env = 8
    if trainer.judge is not None:
        trainer.judge.settings.batch = 64
        trainer.judge.settings.updates = 2
    return trainer


@pytest.mark.parametrize("overlap", [False, True])
def test_body_trainer_runs_and_saves(scratch, overlap):
    from matter_rl.body_trainer import BodyTrainer, is_body_run
    trainer = tiny(BodyTrainer, scratch, f"teste_{int(overlap)}", overlap=overlap)
    assert trainer.model.actor_hidden == (1024, 512, 256) and trainer.judge is not None
    assert trainer.loop() == 0
    assert is_body_run(f"teste_{int(overlap)}")
    directory = paths.run_dir(f"teste_{int(overlap)}")
    assert (directory / "exports" / "policy_latest.mrlpolicy").exists()
    assert storage.latest_checkpoint(directory) is not None
    db = storage.Database(paths.DATABASE)
    rows = db.execute("SELECT iteration, details FROM iterations WHERE run=? ORDER BY iteration",
                      (f"teste_{int(overlap)}",)).fetchall()
    assert [row["iteration"] for row in rows] == [1, 2, 3]
    details = json.loads(rows[-1]["details"])
    assert details["beta"] == 1.0 and details["style"] is not None
    assert {"judge_accuracy", "judge_real", "judge_fake", "style_reserved"} <= set(details["judge"])


def test_body_trainer_beta_and_variant(scratch):
    from matter_rl.body_trainer import BodyTrainer
    trainer = tiny(BodyTrainer, scratch, "beta")
    trainer.max_iterations = 1
    trainer.loop()
    # Reaberto: o beta muda pelo comando (checkpoint antes) e fica na copia da tarefa.
    trainer = tiny(BodyTrainer, scratch, "beta")
    trainer.command("peso", {"componente": "estilo", "valor": 0.25})
    assert trainer.beta == 0.25
    assert json.loads((paths.run_dir("beta") / "tarefa.json").read_text())["style"]["beta"] == 0.25
    trainer.command("peso", {"componente": "slip", "valor": 0.5})
    assert json.loads((paths.run_dir("beta") / "tarefa.json").read_text())["reward"]["costs"]["slip"] == 0.5
    with pytest.raises(ValueError):
        trainer.command("peso", {"componente": "inventado", "valor": 1.0})
    trainer.max_iterations = trainer.counters["iteration"] + 1
    trainer.loop()
    # A variante B1: sem o juiz, e o beta nao existe.
    plain = tiny(BodyTrainer, scratch, "sem_juiz", style=False)
    assert plain.judge is None
    with pytest.raises(ValueError):
        plain.command("peso", {"componente": "estilo", "valor": 0.5})
    plain.loop()


def test_contract_one_run_is_refused(scratch):
    from matter_rl.body_trainer import BodyTrainer
    directory = paths.run_dir("antigo")
    directory.mkdir(parents=True)
    (directory / "run.json").write_text(json.dumps({"task": "corpo", "environments": 4}))
    with pytest.raises(RuntimeError):
        BodyTrainer("antigo", task="corpo2", environments=4, socket_path=scratch / "x.sock")


def test_judge_learns_to_separate():
    """O juiz separa o acervo de um corpo que nao se parece com ele."""
    from matter_rl import native
    from matter_rl.amp import Judge, JudgeSettings
    module = native.load()
    dataset = module.motion_dataset(str(paths.ASSETS / "rl" / "bodies" / "football_player_v2.json"),
                                    str(paths.ASSETS / "rl" / "style" / "acervo_corpo.json"))
    torch.manual_seed(0)
    judge = Judge(dataset, JudgeSettings(hidden=[128, 64], batch=256, updates=40, learning_rate=1e-3),
                  torch.device("cpu"))
    # O "corpo": os pares do acervo com as juntas tremendo (ruido grande).
    rng = np.random.default_rng(0)
    fake = np.asarray(dataset["pairs"], dtype=np.float32)[rng.integers(0, len(dataset["pairs"]), 4096)].copy()
    fake += rng.normal(0.0, 0.6, fake.shape).astype(np.float32)
    judge.remember(fake)
    health = {}
    for _ in range(5):
        health = judge.update()
    assert health["judge_accuracy"] > 0.8
    assert health["judge_real"] > health["judge_fake"]
    assert health["style_reserved"] > health["style_agent"]
    assert 0.0 <= health["style_agent"] <= 1.0


def test_mirror_maps_and_loss():
    """O mapa do espelho pela ponte e a perda do espelho no ator."""
    from matter_rl import native
    from matter_rl.model import ActorCritic
    from matter_rl.symmetry import ChannelMirror, MirrorLoss
    module = native.load()
    batch = module.BodyBatch(str(paths.task_path("corpo2")), environments=4, per_scene=2, threads=1, seed=1)
    maps = batch.mirror_maps()
    assert maps["asymmetry"] < 0.01
    device = torch.device("cpu")
    observation = ChannelMirror(maps["observation"], device)
    obs = torch.tensor(np.asarray(batch.observations()))
    assert torch.allclose(observation(observation(obs)), obs, atol=1e-4)
    names = batch.observation_names()
    left = names.index("LeftThigh.swing1.velocity")
    mirrored = observation(obs)
    assert names[int(observation.source[left])] == "RightThigh.swing1.velocity"
    torch.manual_seed(0)
    model = ActorCritic(batch.obs_size, batch.critic_size, batch.action_size, actor_hidden=[64], critic_hidden=[64])
    model.obs_norm.update(obs)
    loss = MirrorLoss(maps, device)
    optimizer = torch.optim.Adam(model.actor.parameters(), lr=1e-2)
    obs_n = model.obs_norm(obs)
    before = float(loss(model, obs_n).detach())
    for _ in range(50):
        optimizer.zero_grad()
        value = loss(model, obs_n)
        value.backward()
        optimizer.step()
    assert float(loss(model, obs_n).detach()) < 0.5 * before
    assert mirrored.shape == obs.shape


def test_body_trainer_with_mirror_loss(scratch):
    from matter_rl.body_trainer import BodyTrainer
    trainer = tiny(BodyTrainer, scratch, "espelho")
    trainer.config.mirror_loss = 0.5
    from matter_rl.symmetry import MirrorLoss
    trainer.mirror = MirrorLoss(trainer.batch.mirror_maps(), trainer.device)
    trainer.max_iterations = 2
    assert trainer.loop() == 0
    db = storage.Database(paths.DATABASE)
    row = db.execute("SELECT details FROM iterations WHERE run='espelho' ORDER BY iteration DESC").fetchone()
    assert json.loads(row["details"])["mirror"] > 0.0


def test_body_trainer_evaluation(scratch):
    from matter_rl.body_trainer import BodyTrainer
    trainer = tiny(BodyTrainer, scratch, "avaliacao")
    trainer.config.evaluation_worlds = 4
    trainer.config.evaluation_seconds = 2.0
    trainer.max_iterations = 1
    trainer.loop()
    trainer = tiny(BodyTrainer, scratch, "avaliacao")
    trainer.config.evaluation_worlds = 4
    trainer.config.evaluation_seconds = 2.0
    result = trainer.command("avaliar", {})["evaluation"]
    assert set(result["drills"]) == {"parado", "deslocamento", "estrela"}
    for summary in result["drills"].values():
        assert summary["worlds"] == 4 and summary["episodes"] == 4
    db = storage.Database(paths.DATABASE)
    rows = db.execute("SELECT drill, data FROM body_evaluations WHERE run='avaliacao'").fetchall()
    assert len(rows) == 3
    trainer.max_iterations = trainer.counters["iteration"] + 1
    trainer.loop()


def test_affinity_judge(tmp_path):
    """O juiz de avaliacao congelado: nota alta nos reservados (que nunca viu) e
    baixa nos estragados; salva e carrega igual."""
    from matter_rl import native
    from matter_rl.affinity import AffinityJudge, AffinitySettings, corrupted, policy_pairs
    from matter_rl.model import ActorCritic, export_policy
    module = native.load()
    dataset = module.motion_dataset(str(paths.ASSETS / "rl" / "bodies" / "football_player_v2.json"),
                                    str(paths.ASSETS / "rl" / "style" / "acervo_corpo.json"))
    task = str(paths.task_path("corpo2"))
    batch = module.BodyBatch(task, environments=2, per_scene=2, threads=1, seed=1)
    torch.manual_seed(1)
    model = ActorCritic(batch.obs_size, batch.critic_size, batch.action_size, actor_hidden=[32], critic_hidden=[32])
    policy = tmp_path / "aleatoria.mrlpolicy"
    export_policy(model, policy, {"observationNames": batch.observation_names()})
    agent = policy_pairs(module, task, str(policy), worlds=4, seconds=1.0)
    rng = np.random.default_rng(0)
    negatives = np.concatenate([corrupted(np.asarray(dataset["pairs"]), rng), agent])
    judge = AffinityJudge(dataset, AffinitySettings(hidden=[64, 32], steps=400, batch=256, learning_rate=1e-3))
    result = judge.train(negatives)
    assert result["reserved"] > 0.5 and result["reserved"] > result["negatives"] + 0.2
    # (No primeiro segundo a politica aleatoria ainda esta no alerta do comeco,
    # que e um quadro de clipe: a nota dela pode ser alta.)
    assert 0.0 <= judge.score(agent) <= 1.0
    path = tmp_path / "juiz.pt"
    judge.save(path)
    again = AffinityJudge.load(path, dataset)
    assert abs(again.score(agent) - judge.score(agent)) < 1e-5
