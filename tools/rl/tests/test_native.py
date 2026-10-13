"""Testes com o modulo nativo: o lote visto do Python e a paridade entre a
politica exportada (C++) e o PyTorch."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import torch

from matter_rl import native, paths
from matter_rl.model import ActorCritic, export_policy

module = native.load()
TASK = str(paths.task_path("h1_ficar_em_pe"))


def test_batch_shapes_and_views():
    batch = module.Batch(TASK, environments=8, threads=2, seed=3)
    assert batch.action_size == 39
    obs = batch.observations()
    assert obs.shape == (8, batch.obs_size) and obs.dtype == np.float32
    assert not obs.flags.writeable
    assert len(batch.observation_names()) == batch.obs_size
    assert len(batch.critic_names()) == batch.critic_size
    batch.step(np.zeros((8, 39), dtype=np.float32))
    assert np.isfinite(batch.rewards()).all()
    assert batch.reward_components().shape == (8, len(batch.reward_names()))
    assert np.allclose(batch.reward_components().sum(1), batch.rewards(), atol=1e-4)
    lines = batch.frame_lines(0)
    assert lines.count("\n") >= 18 and lines.startswith(("C ", "B "))


def test_postures_through_the_bridge():
    posturado = str(paths.task_path("h1b_em_pe_posturado"))
    batch = module.Batch(TASK, environments=2, threads=1)
    assert batch.posture_ids() == ["natural", "alerta"] and batch.task_postures() == ["natural"]
    names = batch.observation_names()
    alert = names.index("command.posture.alerta")
    assert names[alert + 1:] == ["command.velocity.forward", "command.velocity.lateral", "command.velocity.yaw",
                                 "command.gait.sin", "command.gait.cos"]
    assert (batch.observations()[:, alert] == 0).all()
    forced = module.Batch(posturado, environments=2, threads=1, evaluation=True, posture="alerta")
    assert forced.task_postures() == ["natural", "alerta"]
    assert (forced.observations()[:, alert] == 1).all()
    forced.set_posture("natural")
    forced.reset(3)
    assert (forced.observations()[:, alert] == 0).all()
    with pytest.raises(Exception):
        module.Batch(TASK, environments=1, threads=1, posture="sentado")


def test_battery_per_posture():
    """A tarefa da postura de alerta tem duas baterias: a da base e a dela."""
    from matter_rl.evaluation import Battery, Evaluator
    evaluator = Evaluator(module, paths.task_path("h1b_em_pe_posturado"), 2, Battery(environments=4, seconds=1.0))
    probe = module.Batch(TASK, environments=1, threads=1)
    model = ActorCritic(probe.obs_size, probe.critic_size, probe.action_size)
    results = evaluator.run(model, torch.device("cpu"))
    assert set(results) == {"equilibrio_v1", "postura_alerta_v1"}
    assert set(results["postura_alerta_v1"]) == {"treino", "jogo"}
    assert all(m["episodes"] == 4 for solvers in results.values() for m in solvers.values())


def test_walking_batteries():
    """A tarefa de andar: o roteiro de velocidades e o ponto a 10 m; parada, a
    politica zero nao anda (erro de velocidade alto, longe do ponto)."""
    from matter_rl.evaluation import Battery, Evaluator
    # O andar por comando (a H3 v6, guardada para os testes).
    walking = Path(__file__).resolve().parents[3] / "tests" / "RL" / "data" / "andar_comando.json"
    evaluator = Evaluator(module, walking, 2, Battery(environments=4),
                          names=["andar_frente_v1", "ir_ate_10m_v1"])
    probe = module.Batch(str(walking), environments=1, threads=1)
    model = ActorCritic(probe.obs_size, probe.critic_size, probe.action_size)
    with torch.no_grad():
        last = [layer for layer in model.actor if isinstance(layer, torch.nn.Linear)][-1]
        last.weight.zero_()
        last.bias.zero_()
    results = evaluator.run(model, torch.device("cpu"))
    assert set(results) == {"andar_frente_v1", "ir_ate_10m_v1"}
    walk, goal = results["andar_frente_v1"]["jogo"], results["ir_ate_10m_v1"]["jogo"]
    assert walk["kind"] == "andar" and goal["kind"] == "alvo"
    assert walk["velocity_error"] > 0.5 and goal["goal_error"] > 5.0
    assert walk["score"] < 0.8 and goal["score"] < 0.8


def test_batch_rejects_bad_actions():
    batch = module.Batch(TASK, environments=2, threads=1)
    with pytest.raises(ValueError):
        batch.step(np.zeros((2, 10), dtype=np.float32))


def test_exported_policy_matches_torch(tmp_path: Path):
    torch.manual_seed(11)
    model = ActorCritic(160, 185, 39)
    for p in model.actor.parameters():
        p.data.normal_(0.0, 0.1)
    model.obs_norm.update(torch.randn(500, 160) * 2 + 0.5)
    path = tmp_path / "p.mrlpolicy"
    export_policy(model, path, {"bodyHash": "teste", "iteration": 7})
    runtime = module.Policy(str(path))
    assert runtime.obs_size == 160 and runtime.action_size == 39 and runtime.iteration == 7
    obs = (np.random.default_rng(2).normal(size=(16, 160)) * 3).astype(np.float32)
    with torch.no_grad():
        expected = model.actor(model.obs_norm(torch.tensor(obs))).clamp(-10, 10).numpy()
    got = runtime.evaluate(obs)
    assert np.abs(got - expected).max() < 1e-5
