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


def tiny(trainer_class, scratch, run, task="corpo2", **kwargs):
    trainer = trainer_class(run, task=task, environments=16, seed=3, iterations=3,
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
    assert set(result["solvers"]) == {"jogo", "treino"}
    for solver in result["solvers"].values():
        assert set(solver) == {"parado", "deslocamento", "estrela"}
        for summary in solver.values():
            assert summary["worlds"] == 4 and summary["episodes"] == 4
            assert summary["falls"] <= summary["falls_upper95"] <= 1.0
    db = storage.Database(paths.DATABASE)
    rows = db.execute("SELECT drill, data FROM body_evaluations WHERE run='avaliacao'").fetchall()
    assert len(rows) == 6
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


def test_wilson_upper():
    from matter_rl.body_trainer import wilson_upper
    assert 0.10 < wilson_upper(0, 32) < 0.12
    assert wilson_upper(16, 32) > 0.5 and wilson_upper(32, 32) == 1.0


# --- o plano de recuperacao (R1, itens 5, 6 e 10) ---------------------------------------


def test_reward_composition_matches_the_batch_without_style(scratch):
    """Sem o juiz, a conta do treinador e a do C++ (a regra do total nunca negativo
    incluida); com o juiz, so o estilo pela porta G entra."""
    from matter_rl.body_trainer import BodyTrainer
    trainer = tiny(BodyTrainer, scratch, "composicao", task="aquisicao_p1", style=False)
    batch = trainer.batch
    assert trainer.positive_only
    rng = np.random.default_rng(0)
    for _ in range(6):
        action = rng.normal(size=(batch.num_envs, batch.action_size)).astype(np.float32) * 0.5
        batch.step(action, action)
        parts = torch.tensor(np.asarray(batch.reward_components()))
        mine = trainer.compose_rewards(parts, None).numpy()
        assert np.allclose(mine, np.asarray(batch.rewards()), atol=1e-6)
    # O estilo so paga pela porta G: onde G = 0 (parado com pedido de andar) nada entra.
    style = torch.ones(batch.num_envs)
    base = trainer.compose_rewards(parts, None)
    paid = trainer.compose_rewards(parts, style)
    gate = parts[:, trainer.ri["style_gate"]]
    assert torch.all(paid - base >= -1e-9) and torch.all((paid - base)[gate == 0] == 0)


def test_fixed_sigma_and_seeds(scratch):
    from matter_rl.body_trainer import BodyTrainer, axis_group
    a = tiny(BodyTrainer, scratch, "sigma_a", task="aquisicao_p0", style=False)
    b = tiny(BodyTrainer, scratch, "sigma_b", task="aquisicao_p0", style=False)
    # A mesma semente, a mesma rede inicial (o PyTorch tambem e semeado).
    assert all(torch.equal(x, y) for x, y in zip(a.model.actor.parameters(), b.model.actor.parameters()))
    assert not a.model.log_std.requires_grad
    radians = np.asarray(a.batch.action_radians_per_unit())
    sigma = a.model.log_std.detach().exp().cpu().numpy() * radians
    for i, name in enumerate(a.action_names):
        expected = {"legs": 0.06, "toes": 0.06, "arms": 0.05, "trunk": 0.03}[axis_group(name)]
        assert abs(sigma[i] - expected) < 1e-3, name
    before = a.model.log_std.detach().clone()
    a.max_iterations = 2
    a.loop()
    assert torch.equal(a.model.log_std.detach(), before)
    assert a.settings.entropy_coef == 0.0
    manifest = json.loads((paths.run_dir("sigma_a") / "checkpoints" / sorted(
        p.name for p in (paths.run_dir("sigma_a") / "checkpoints").iterdir())[-1] / "manifest.json").read_text())
    assert manifest["seed"] == 3 and manifest["exploration"]["mode"] == "fixed" and manifest["positive_only"]


def test_two_optimizers_and_kl_stop():
    """O critico aprende mesmo quando o KL interrompe o ator."""
    from matter_rl.model import ActorCritic
    from matter_rl.ppo import PPO, PPOSettings, Rollout
    torch.manual_seed(0)
    model = ActorCritic(6, 7, 3, actor_hidden=[16], critic_hidden=[16])
    settings = PPOSettings(steps_per_env=8, epochs=2, minibatches=2, separate_critic=True, kl_stop_factor=1e-9)
    ppo = PPO(model, settings)
    assert ppo.critic_optimizer is not None
    actor_before = [p.detach().clone() for p in model.actor.parameters()]
    critic_before = [p.detach().clone() for p in model.critic.parameters()]
    rollout = Rollout(8, 16, 6, 7, 3, torch.device("cpu"))
    for _ in range(8):
        obs, critic = torch.randn(16, 6), torch.randn(16, 7)
        dist = model.distribution(obs)
        action = dist.sample()
        rollout.add(obs, critic, action, dist.log_prob(action).sum(-1).detach(), model.value(critic).detach(), torch.randn(16),
                    torch.zeros(16), dist.mean.detach(), dist.stddev.detach())
    rollout.compute_returns(torch.zeros(16), settings.gamma, settings.lam)
    stats = ppo.update(rollout)
    # (O primeiro minibatch ve KL ~0 e atualiza o ator; a partir do segundo, ja parou.)
    assert stats["kl_halted"] == 1.0 and stats["actor_steps"] <= 1
    assert any(not torch.equal(x, y) for x, y in zip(critic_before, model.critic.parameters()))
    # Sem a parada, o ator anda; e o estado dos dois otimizadores vai e volta.
    ppo.settings.kl_stop_factor = 0.0
    stats = ppo.update(rollout)
    assert stats["actor_steps"] == 4 and stats["critic_grad_norm"] > 0.0
    assert any(not torch.equal(x, y) for x, y in zip(actor_before, model.actor.parameters()))
    state = ppo.state_dict()
    assert "critic_optimizer" in state
    PPO(ActorCritic(6, 7, 3, actor_hidden=[16], critic_hidden=[16]), settings).load_state_dict(state)


def test_frozen_normalization_and_raw_mirror(scratch):
    """A estatistica da normalizacao so muda na fronteira da rodada, e o espelho
    usa a observacao crua (o corte em +-5 nao se desfaz)."""
    from matter_rl.body_trainer import BodyTrainer
    from matter_rl.symmetry import MirrorLoss
    trainer = tiny(BodyTrainer, scratch, "congelada", task="aquisicao_p0", style=False)
    trainer.settings.steps_per_env = 4
    before = trainer.model.obs_norm.mean.clone()
    rollout, info = trainer.collect()
    # Na coleta a estatistica ficou congelada e a candidata andou.
    assert torch.equal(trainer.collector.obs_norm.mean, before) and torch.equal(trainer.model.obs_norm.mean, before)
    assert not torch.equal(trainer.obs_candidate.mean, before)
    # As observacoes guardadas sao as que a politica viu, e as cruas batem com elas.
    assert torch.allclose(trainer.model.obs_norm(rollout.raw), rollout.obs, atol=1e-6)
    stats = trainer.update(rollout)
    assert torch.equal(trainer.model.obs_norm.mean, before)
    trainer.finish(stats, info)
    assert torch.equal(trainer.model.obs_norm.mean, trainer.obs_candidate.mean)
    assert torch.equal(trainer.collector.obs_norm.mean, trainer.model.obs_norm.mean)
    # O espelho cru: uma entrada que o corte em +-5 teria mudado.
    mirror = MirrorLoss(trainer.batch.mirror_maps(), trainer.device)
    raw = rollout.raw.reshape(-1, rollout.raw.shape[-1])[:64].clone()
    raw[:, 0] += 50.0
    exact = float(mirror(trainer.model, raw, raw=True).detach())
    recovered = float(mirror(trainer.model, trainer.model.obs_norm(raw)).detach())
    assert np.isfinite(exact) and np.isfinite(recovered)
    mirrored = mirror.observation(raw)
    manual = (trainer.model.actor(trainer.model.obs_norm(mirrored))
              - mirror.action(trainer.model.actor(trainer.model.obs_norm(raw)))).pow(2).sum(-1).mean()
    assert abs(exact - float(manual.detach())) < 1e-5


def test_transfer_between_steps_and_judge_warmup(scratch):
    """Um treino novo que nasce de outro: o ator e a normalizacao passam, o
    critico e o juiz recomecam, e o ator espera o aquecimento do juiz."""
    from matter_rl.body_trainer import BodyTrainer
    first = tiny(BodyTrainer, scratch, "degrau_a", task="aquisicao_p0", style=False)
    first.max_iterations = 1
    first.loop()
    second = tiny(BodyTrainer, scratch, "degrau_b", task="aquisicao_p3a", transfer_from="degrau_a", warmup=2)
    assert second.counters["iteration"] == 0 and second.counters["warmup_left"] == 2
    assert all(torch.equal(x, y) for x, y in zip(first.model.actor.parameters(), second.model.actor.parameters()))
    assert torch.equal(first.model.obs_norm.mean, second.model.obs_norm.mean)
    assert second.config.transferred_from == "degrau_a" and second.judge is not None
    actor_start = [p.detach().clone() for p in second.model.actor.parameters()]
    second.max_iterations = 2
    second.loop()
    # As duas rodadas de aquecimento nao mexeram no ator; a seguinte mexe.
    assert second.counters["warmup_left"] == 0
    assert all(torch.equal(x, y) for x, y in zip(actor_start, second.model.actor.parameters()))
    third = tiny(BodyTrainer, scratch, "degrau_b", task="aquisicao_p3a")
    assert third.counters["iteration"] == 2 and third.counters["warmup_left"] == 0
    third.max_iterations = 4
    third.loop()
    assert any(not torch.equal(x, y) for x, y in zip(actor_start, third.model.actor.parameters()))
    db = storage.Database(paths.DATABASE)
    events = [row["kind"] for row in db.execute("SELECT kind FROM events WHERE run='degrau_b'").fetchall()]
    assert "transferencia" in events and "aquecimento" in events


def test_judge_refuses_another_acervo_and_weights_the_normalization():
    from matter_rl.amp import Judge, JudgeSettings
    rng = np.random.default_rng(0)
    width = 6
    pairs = rng.normal(size=(40, width)).astype(np.float32)
    pairs[:20] += 5.0
    weights = np.concatenate([np.full(20, 0.9 / 20), np.full(20, 0.1 / 20)])

    def dataset(name):
        return {"pairs": pairs, "pair_weight": weights, "hash": name,
                "clips": [{"reserved": False, "first_pair": 0, "pairs": 40}]}

    device = torch.device("cpu")
    judge = Judge(dataset("a"), JudgeSettings(hidden=[8]), device)
    # A media com os pesos: a metade pesada (+5) domina.
    half = width // 2
    expected = (pairs[:, :half].T @ weights + pairs[:, half:].T @ weights) / (2 * weights.sum())
    assert np.allclose(judge.mean[:half].numpy(), expected, atol=1e-4)
    other = Judge(dataset("b"), JudgeSettings(hidden=[8]), device)
    with pytest.raises(RuntimeError):
        other.load_state_dict(judge.state_dict())
    twin = Judge(dataset("a"), JudgeSettings(hidden=[8]), device)
    twin.load_state_dict(judge.state_dict())


def test_start_arguments_keep_the_transfer(scratch):
    """O iniciar e o servir montam o comando do treino por uma construcao so: a
    transferencia, o aquecimento e a semente nao se perdem no caminho."""
    import argparse
    from matter_rl.__main__ import start_arguments, trainer_command
    args = argparse.Namespace(run="p3", tarefa="aquisicao_p3a", modo=None, mundos=None, semente=5,
                              de_checkpoint="p0:9", transferir="ator", aquecimento=30, desvio_aprendido=None,
                              sem_estilo=False, sem_sobreposicao=False, sobreposicao=False, dispositivo="auto")
    command = start_arguments(args, ["x"])
    assert command[command.index("--de-checkpoint") + 1] == "p0:9"
    assert command[command.index("--transferir") + 1] == "ator" and command[command.index("--aquecimento") + 1] == "30"
    assert command[command.index("--semente") + 1] == "5" and "--sem-estilo" not in command
    again = trainer_command(args)
    assert "--de-checkpoint" in again and "--semente" in again


def test_old_runs_keep_their_learned_sigma_and_the_epochs_persist(scratch):
    from matter_rl.body_trainer import BodyRunConfig, BodyTrainer
    old = scratch / "antigo.json"
    old.write_text(json.dumps({"task": "corpo2", "environments": 4, "overlap": True}))
    config = BodyRunConfig.load(old)
    assert config.exploration["mode"] == "learned"
    trainer = tiny(BodyTrainer, scratch, "epocas", task="aquisicao_p0", style=False)
    trainer.max_iterations = 1
    trainer.counters["epochs"] = 3
    trainer.ppo.settings.epochs = 3
    trainer.loop()
    reopened = tiny(BodyTrainer, scratch, "epocas", task="aquisicao_p0", style=False)
    assert reopened.ppo.settings.epochs == 3 and reopened.counters["epochs"] == 3


def test_recovery_syncs_the_scoring_judge(scratch):
    from matter_rl.body_trainer import BodyTrainer
    trainer = tiny(BodyTrainer, scratch, "recuperar", task="aquisicao_p3a")
    trainer.max_iterations = 1
    trainer.loop()
    trainer = tiny(BodyTrainer, scratch, "recuperar", task="aquisicao_p3a")
    for parameter in trainer.judge.net.parameters():
        parameter.data.add_(1.0)
    trainer.sync()
    trainer.recover()
    assert all(torch.equal(a, b) for a, b in zip(trainer.scorer.parameters(), trainer.judge.net.parameters()))
