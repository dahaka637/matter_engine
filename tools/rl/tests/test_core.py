"""Testes do treinador que nao precisam do modulo nativo: PPO, rede,
persistencia e o canal de controle."""
from __future__ import annotations

import math
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


def test_command_channels_pass_raw():
    """Os canais de comando nao acumulam media e variancia (valem 0 ou 1)."""
    norm = RunningNorm(3)
    norm.fix_channels(torch.tensor([False, True, False]))
    for _ in range(5):
        norm.update(torch.rand(100, 3) * torch.tensor([10.0, 0.0, 1.0]))
    assert float(norm.mean[1]) == 0.0 and float(norm.var[1]) == 1.0
    assert float(norm.mean[0]) > 1.0
    out = norm(torch.tensor([[5.0, 1.0, 0.5]]))
    assert abs(float(out[0, 1]) - 1.0) < 1e-6


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


def test_migration_with_player_channels_keeps_the_policy():
    """O corpo variavel (26/10): a rede migrada ganha jogador.altura e jogador.massa no
    fim da entrada (e antes do drill.* no critico), crus e com peso zero, e da a mesma
    saida - no corpo de hoje (os canais em zero) e, no comeco, em qualquer outro."""
    from matter_rl.locomotion_trainer import RAW_PREFIXES
    assert "jogador." in RAW_PREFIXES
    torch.manual_seed(7)
    old_obs = ["corpo.pelve.altura", "comando.vel.x", "antes_1.pes.esq.pouso.x"]
    new_obs = old_obs + ["jogador.altura", "jogador.massa"]
    old_critic = old_obs + ["drill.parado", "pilot.alvo.x"]
    new_critic = new_obs + ["drill.parado", "pilot.alvo.x"]
    old = ActorCritic(len(old_obs), len(old_critic), 2)
    for norm, names in ((old.obs_norm, old_obs), (old.critic_norm, old_critic)):
        norm.fix_channels(torch.tensor([n.startswith(RAW_PREFIXES) for n in names]))
        norm.update(torch.randn(200, len(names)) * 2 + 0.5)
    # (A ordem do treinador: restore migra, depois fix_raw_channels.)
    fresh = migrate_inputs(old, old_obs, new_obs, old_critic, new_critic)
    for norm, names in ((fresh.obs_norm, new_obs), (fresh.critic_norm, new_critic)):
        norm.fix_channels(torch.tensor([n.startswith(RAW_PREFIXES) for n in names]))
    assert torch.all(fresh.obs_norm.mean[-2:] == 0.0) and torch.all(fresh.obs_norm.var[-2:] == 1.0)  # crus
    x, c = torch.randn(10, len(old_obs)), torch.randn(10, len(old_critic))
    with torch.no_grad():
        for player in (torch.zeros(10, 2), torch.tensor([[2.0, 1.5]]).repeat(10, 1)):
            y = torch.cat([x, player], 1)
            c2 = torch.cat([c[:, :len(old_obs)], player, c[:, len(old_obs):]], 1)
            assert torch.allclose(old.actor(old.obs_norm(x)), fresh.actor(fresh.obs_norm(y)), atol=1e-6)
            assert torch.allclose(old.value(old.critic_norm(c)), fresh.value(fresh.critic_norm(c2)), atol=1e-6)


def test_body_strata_only_with_varied_bodies():
    """As quedas por corpo vao para o banco so com corpos variados (sem o bloco, o
    resumo de antes)."""
    from matter_rl.locomotion_trainer import body_strata
    columns = ["razao", "segundos", "retorno", "erro_velocidade", "corpo_altura", "corpo_massa"]
    reasons = ["nenhum", "queda", "numerico", "tempo", "limite", "falha"]
    base = np.array([[1, 2.0, 0.5, 0.3, 1.8, 80.0], [3, 16.0, 4.0, 0.2, 1.8, 80.0]], dtype=np.float32)
    assert body_strata(base, columns, reasons) == {}
    assert body_strata(base[:, :4], columns[:4], reasons) == {}
    varied = np.vstack([base, np.array([[1, 1.0, 0.1, 0.5, 2.0, 95.0]], dtype=np.float32)])
    strata = body_strata(varied, columns, reasons)
    assert set(strata) == {"1.80x80", "2.00x95"}
    assert strata["1.80x80"]["falls"] == 0.5 and strata["2.00x95"]["falls"] == 1.0 and strata["1.80x80"]["episodes"] == 2


def test_run_config_native_only_when_set(tmp_path):
    """O modulo nativo do treino no run.json: so quando dado (o run.json de antes fica igual)."""
    from matter_rl.locomotion_trainer import LocomotionRunConfig
    path = tmp_path / "run.json"
    LocomotionRunConfig().save(path)
    assert "native" not in json.loads(path.read_text())
    LocomotionRunConfig(native="/x/build-rl-corpos/python").save(path)
    assert LocomotionRunConfig.load(path).native == "/x/build-rl-corpos/python"


def test_migration_adds_action_channels_without_changing_the_old_ones():
    torch.manual_seed(5)
    old = ActorCritic(4, 5, 2)
    old.obs_norm.update(torch.randn(100, 4) * 2)
    old_obs = ["a", "b", "c", "d"]
    new_obs = ["a", "b", "c", "d", "turno.ativo", "antes_1.turno.ganho"]
    old_critic = old_obs + ["e"]
    new_critic = new_obs + ["e"]
    old_actions = ["x", "y"]
    new_actions = ["x", "y", "turno.ganho"]
    fresh = migrate_inputs(old, old_obs, new_obs, old_critic, new_critic, old_actions, new_actions,
                           torch.full((3,), math.log(0.3)))
    x = torch.randn(10, 4)
    y = torch.cat([x, torch.randn(10, 2)], 1)
    with torch.no_grad():
        a_old = old.actor(old.obs_norm(x))
        a_new = fresh.actor(fresh.obs_norm(y))
    assert a_new.shape == (10, 3)
    assert torch.allclose(a_old, a_new[:, :2], atol=1e-5)   # os canais de antes saem iguais
    assert torch.all(a_new[:, 2] == 0.0)                     # o novo pede zero (o professor inteiro)
    assert torch.allclose(fresh.log_std[:2], old.log_std, atol=1e-6)
    assert abs(float(fresh.log_std[2]) - math.log(0.3)) < 1e-6


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


def test_task_switch_keeps_the_stochastic_policy():
    """A troca de tarefa (o critico ganha entradas, os tetos mudam): na mesma
    acao fisica, a log-probabilidade, o desvio fisico e o valor do critico ficam
    iguais (o que o PPO ve no primeiro instante da tarefa nova)."""
    torch.manual_seed(7)
    obs_names = ["a", "b", "c"]
    critic_old = ["a", "b", "c", "d"]
    critic_new = critic_old + ["novo1", "novo2"]
    model = ActorCritic(3, 4, 3)
    for p in model.actor.parameters():
        p.data.normal_()
    model.log_std.data.normal_(-1.0, 0.3)
    model.obs_norm.update(torch.randn(64, 3) * 2)
    model.critic_norm.update(torch.randn(64, 4))
    old_scale = torch.tensor([0.10, 0.06, 0.2])
    new_scale = torch.tensor([0.15, 0.09, 0.2])
    x = torch.randn(16, 3)
    c = torch.randn(16, 4)
    c_new = torch.cat([c, torch.randn(16, 2)], 1)
    with torch.no_grad():
        before = model.distribution(model.obs_norm(x))
        physical = before.sample() * old_scale
        logp_before = before.log_prob(physical / old_scale).sum(-1) - torch.log(old_scale).sum()
        std_before = before.stddev * old_scale
        value_before = model.value(model.critic_norm(c))
    fresh = migrate_inputs(model, obs_names, obs_names, critic_old, critic_new)
    rescale_actions(fresh, old_scale / new_scale)
    with torch.no_grad():
        after = fresh.distribution(fresh.obs_norm(x))
        logp_after = after.log_prob(physical / new_scale).sum(-1) - torch.log(new_scale).sum()
        assert torch.allclose(after.stddev * new_scale, std_before, atol=1e-6)
        assert torch.allclose(logp_after, logp_before, atol=5e-3)  # (float32: o desvio e pequeno)
        assert torch.allclose(fresh.value(fresh.critic_norm(c_new)), value_before, atol=1e-5)
        assert float(fresh.critic[0].weight[:, 4:].abs().max()) == 0.0


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


def test_resume_after_crash_rewinds_history(tmp_path: Path):
    """Um processo morto sem fechar (kill, queda de energia): a sessao aberta
    fecha como interrompida e o que veio depois do checkpoint sai do banco."""
    db = storage.Database(tmp_path / "rl.db")
    db.ensure_run("t", "h1", "abc", {})
    crashed = db.begin_session("t", 0, "maximo", "cuda")
    for iteration in range(1, 21):
        db.log_iteration({"run": "t", "iteration": iteration, "time": time.time(), "steps": iteration})
    db.log_body_evaluation("t", 15, 15, "andar_frente", "jogo", {"falls": 0.0})
    db.commit()
    resumed = db.begin_session("t", 10, "maximo", "cuda")
    assert db.rewind("t", 10) == 10
    rows = db.execute("SELECT id, end_reason, finish_utc FROM sessions ORDER BY id").fetchall()
    assert rows[0]["id"] == crashed and rows[0]["end_reason"] == "interrompida" and rows[0]["finish_utc"]
    assert rows[1]["id"] == resumed and rows[1]["finish_utc"] is None
    assert db.execute("SELECT MAX(iteration) FROM iterations").fetchone()[0] == 10
    assert db.execute("SELECT COUNT(*) FROM body_evaluations").fetchone()[0] == 0


def test_history_sampled_by_time_keeps_compacted_rows(tmp_path: Path):
    """Semanas de treino: as rodadas antigas compactadas (uma a cada 10 min) e
    as recentes densas aparecem espalhadas no tempo, nao so as recentes."""
    db = storage.Database(tmp_path / "rl.db")
    db.ensure_run("t", "h1", "abc", {})
    now = time.time()
    old_start = now - 10 * 86400.0
    iteration = 0
    for day in range(7):  # 7 dias compactados: uma rodada a cada 10 min
        for block in range(144):
            iteration += 1 + (block % 7)
            db.log_iteration({"run": "t", "iteration": iteration, "time": old_start + day * 86400 + block * 600,
                              "steps": iteration})
    for second in range(0, 86400, 2):  # 1 dia denso
        iteration += 1
        db.log_iteration({"run": "t", "iteration": iteration, "time": now - 86400 + second, "steps": iteration})
    db.commit()
    rows = db.sampled_iterations("t", 200)
    times = [row["time"] for row in rows]
    assert 150 <= len(rows) <= 201
    assert sum(1 for t in times if t < now - 2 * 86400) > 100  # o historico antigo continua la
    assert [row["iteration"] for row in rows] == sorted(row["iteration"] for row in rows)
    assert db.compact("t") == 0  # o que tem mais de 3 dias ja esta em uma a cada 10 min


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


def test_floor_noise_scales_with_body_height():
    """O ruido perto do chao em metros x s: a pelve a 0,60 x s m de um corpo de 2,00 m e o
    limiar alto dele (sem ruido extra), como 0,60 m no de 1,80."""
    model = ActorCritic(2, 1, 1, np.array([0.5], dtype=np.float32))
    model.floor_noise = (0, 0.60, 0.35, 2.5, 1)
    base = model.log_std.clamp(-5.0, 1.5).exp()
    s = 2.0 / 1.8
    obs = torch.tensor([[0.60 * s, 2.0], [0.60, 0.0], [0.35 * s, 2.0]])
    std = model.distribution(model.obs_norm(obs)).stddev[:, 0]
    assert torch.allclose(std[:2], base.expand(2), atol=1e-5)
    assert float(std[2]) == pytest.approx(float(base) * 2.5, rel=1e-4)
