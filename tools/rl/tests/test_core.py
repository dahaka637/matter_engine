"""Testes do treinador que nao precisam do modulo nativo: PPO, rede,
persistencia e o canal de controle."""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import numpy as np
import pytest
import torch

from matter_rl import control, storage
from matter_rl.model import ActorCritic, RunningNorm, migrate_inputs, rescale_actions
from matter_rl.ppo import PPO, PPOSettings, Rollout


def test_running_norm_matches_numpy():
    norm = RunningNorm(3)
    data = np.random.default_rng(1).normal([1.0, -2.0, 5.0], [0.5, 2.0, 0.1], size=(4000, 3)).astype(np.float32)
    for chunk in np.split(data, 8):
        norm.update(torch.tensor(chunk))
    assert np.allclose(norm.mean.numpy(), data.mean(0), atol=1e-3)
    assert np.allclose(norm.var.numpy(), data.var(0), rtol=1e-2)


def test_gae_matches_hand_computation():
    rollout = Rollout(3, 1, 1, 1, 1, torch.device("cpu"))
    rollout.rewards[:, 0] = torch.tensor([1.0, 1.0, 1.0])
    rollout.values[:, 0] = torch.tensor([0.5, 0.5, 0.5])
    rollout.dones[:, 0] = torch.tensor([0.0, 1.0, 0.0])
    rollout.compute_returns(torch.tensor([2.0]), gamma=0.9, lam=0.8)
    # t=2: delta = 1 + 0.9*2 - 0.5 = 2.3 ; t=1 (fim): delta = 1 - 0.5 = 0.5 ; t=0: 1 + 0.45 - 0.5 = 0.95 + 0.72*0.5
    assert rollout.advantages[2, 0].item() == pytest.approx(2.3)
    assert rollout.advantages[1, 0].item() == pytest.approx(0.5)
    assert rollout.advantages[0, 0].item() == pytest.approx(0.95 + 0.72 * 0.5)


def test_ppo_learns_a_simple_task():
    """Copiar o alvo (recompensa -(a - x)^2): o PPO precisa aprender em segundos."""
    torch.manual_seed(0)
    model = ActorCritic(1, 1, 1, np.array([0.5], dtype=np.float32))
    settings = PPOSettings(steps_per_env=8, epochs=4, minibatches=4, learning_rate=3e-3, entropy_coef=0.0)
    ppo = PPO(model, settings)
    envs = 64
    def error():
        x = torch.linspace(-1, 1, 101)[:, None]
        with torch.no_grad():
            return float((model.actor(model.obs_norm(x)) - x).abs().mean())
    before = error()
    for _ in range(120):
        rollout = Rollout(settings.steps_per_env, envs, 1, 1, 1, torch.device("cpu"))
        for _ in range(settings.steps_per_env):
            x = torch.rand(envs, 1) * 2 - 1
            model.obs_norm.update(x)
            model.critic_norm.update(x)
            obs_n, critic_n = model.obs_norm(x), model.critic_norm(x)
            with torch.no_grad():
                dist = model.distribution(obs_n)
                action = dist.sample()
                reward = -(action - x).pow(2).squeeze(-1)
                rollout.add(obs_n, critic_n, action, dist.log_prob(action).sum(-1), model.value(critic_n), reward,
                            torch.ones(envs), dist.mean, dist.stddev)
        rollout.compute_returns(torch.zeros(envs), settings.gamma, settings.lam)
        ppo.update(rollout)
    after = error()
    assert after < 0.1 < before


def test_migration_preserves_function():
    torch.manual_seed(3)
    old = ActorCritic(4, 5, 2)
    old.obs_norm.update(torch.randn(100, 4) * 3 + 1)
    old.critic_norm.update(torch.randn(100, 5))
    old_obs = ["a", "b", "c", "d"]
    new_obs = ["a", "novo", "b", "c", "d", "outro"]
    old_critic = ["a", "b", "c", "d", "e"]
    new_critic = old_critic + ["f"]
    fresh = migrate_inputs(old, old_obs, new_obs, old_critic, new_critic)
    x = torch.randn(10, 4)
    y = torch.zeros(10, 6)
    for j, name in enumerate(new_obs):
        if name in old_obs:
            y[:, j] = x[:, old_obs.index(name)]
        else:
            y[:, j] = torch.randn(10)  # o canal novo nao muda nada no comeco
    with torch.no_grad():
        assert torch.allclose(old.actor(old.obs_norm(x)), fresh.actor(fresh.obs_norm(y)), atol=1e-5)
        c = torch.randn(10, 5)
        c2 = torch.cat([c, torch.randn(10, 1)], 1)
        assert torch.allclose(old.value(old.critic_norm(c)), fresh.value(fresh.critic_norm(c2)), atol=1e-5)


def test_rescale_actions_keeps_joint_targets():
    torch.manual_seed(4)
    model = ActorCritic(3, 3, 2)
    for p in model.actor.parameters():
        p.data.normal_()
    x = torch.randn(5, 3)
    with torch.no_grad():
        before = model.actor(model.obs_norm(x)) * torch.tensor([0.5, 2.0])
    rescale_actions(model, torch.tensor([0.5, 2.0]))
    with torch.no_grad():
        assert torch.allclose(model.actor(model.obs_norm(x)), before, atol=1e-5)


def test_checkpoints_atomic_and_pruned(tmp_path: Path):
    run = tmp_path / "treino"
    now = time.time()
    for iteration in range(1, 31):
        manifest = {"time": now - (30 - iteration) * 7200.0}
        storage.save_checkpoint(run, iteration, {"x": torch.tensor([float(iteration)])}, manifest)
    latest = storage.latest_checkpoint(run)
    assert latest is not None and latest.iteration == 30
    assert storage.load_checkpoint(latest)["x"].item() == 30.0
    (run / "checkpoints" / "ck-000000099.tmp").mkdir()
    assert storage.clean_temporaries(run) == 1
    removed = storage.prune_checkpoints(run, protected={2}, now=now)
    kept = {ref.iteration for ref in storage.list_checkpoints(run)}
    assert {28, 29, 30, 2} <= kept and removed
    assert len(kept) < 30


def test_run_lock_is_exclusive(tmp_path: Path):
    lock = storage.RunLock(tmp_path)
    with pytest.raises(RuntimeError):
        storage.RunLock(tmp_path)
    assert storage.run_locked(tmp_path)
    lock.release()
    assert not storage.run_locked(tmp_path)


def test_database_roundtrip(tmp_path: Path):
    db = storage.Database(tmp_path / "rl.db")
    db.ensure_run("t", "h1", "abc", {"x": 1})
    db.log_iteration({"run": "t", "iteration": 1, "time": time.time(), "steps": 10, "reward": 0.5})
    db.commit()
    db.event("t", "teste", "ola", "claude")
    reader = storage.Database(tmp_path / "rl.db", readonly=True)
    assert reader.execute("SELECT reward FROM iterations").fetchone()[0] == 0.5
    assert reader.execute("SELECT author FROM events").fetchone()[0] == "claude"


def test_control_roundtrip(tmp_path: Path):
    socket_path = tmp_path / "c.sock"
    server = control.ControlServer(lambda: {"state": "treinando"}, socket_path)
    def worker():
        deadline = time.time() + 5
        while time.time() < deadline:
            for command, arguments, answer in server.pending():
                control.reply(answer, {"ok": True, "echo": [command, arguments]})
            time.sleep(0.01)
    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    assert control.request("status", path=socket_path)["status"]["state"] == "treinando"
    answer = control.request("pausar", {"autor": "teste"}, path=socket_path)
    assert answer["echo"] == ["pausar", {"autor": "teste"}]
    server.close()
    assert control.request("status", path=socket_path) is None
