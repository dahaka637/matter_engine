"""As pecas do plano de 18/10 (vault 04_locomocao/referencia/pesquisas/08_plano_consenso.md):
a normalizacao refeita sem mudar a funcao, o recentramento do ator, a perda de limites e a
KL sem os canais saturados, o curriculo de velocidade por celulas e os limites do desvio."""
from __future__ import annotations

import numpy as np
import torch

from matter_rl.curriculum import CELLS, SpeedCurriculum, band_maximum
from matter_rl.locomotion_trainer import action_group, channel_values
from matter_rl.model import ActorCritic, recenter_actor, refit_normalizer
from matter_rl.ppo import PPO, PPOSettings, Rollout


def _model(obs=12, critic=14, actions=5, seed=0) -> ActorCritic:
    torch.manual_seed(seed)
    model = ActorCritic(obs, critic, actions, actor_hidden=(32, 32), critic_hidden=(32, 32))
    with torch.no_grad():
        # A media saturada como a de 18/10 (|mu| de 2 a 3,6 em parte dos canais).
        last = [layer for layer in model.actor if isinstance(layer, torch.nn.Linear)][-1]
        last.weight.mul_(60.0)
        last.bias.copy_(torch.tensor([2.5, -3.0, -0.5, -0.8, 1.5])[:actions])
    return model


def test_refit_normalizer_keeps_the_function_where_nothing_is_cut():
    model = _model()
    old = torch.randn(4000, 12) * 0.5 + 0.3
    model.obs_norm.update(old)
    shifted = torch.randn(4000, 12) * torch.linspace(0.2, 2.0, 12) + torch.linspace(-1.0, 1.0, 12)
    raw = torch.zeros(12, dtype=torch.bool)
    raw[3] = True  # uma bandeira que passa crua
    with torch.no_grad():
        before = model.actor(model.obs_norm(shifted))
        rows_old = (((shifted - model.obs_norm.mean) * model.obs_norm.inverse_std()).abs() < 5.0).all(-1)
        refit_normalizer(model.obs_norm, model.actor[0], shifted, raw, count=1e6)
        after = model.actor(model.obs_norm(shifted))
        rows_new = (((shifted - model.obs_norm.mean) * model.obs_norm.inverse_std()).abs() < 5.0).all(-1)
    rows = rows_old & rows_new
    assert rows.float().mean() > 0.3
    assert float((after[rows] - before[rows]).abs().max()) < 1e-4
    assert float(model.obs_norm.mean[3]) == 0.0 and abs(float(model.obs_norm.var[3]) - 1.0) < 1e-6
    assert float(model.obs_norm.count) == 1e6


def test_refit_normalizer_keeps_binary_flags_exact_even_where_the_old_norm_cut_them():
    model = _model()
    base = torch.randn(3000, 12)
    base[:, 5] = 0.0  # uma bandeira quase sempre 0 (desvio minusculo): o 1 era cortado em +5
    base[:10, 5] = 1.0
    base[:, 7] = -0.2  # um canal constante (o olhar), nunca variou
    model.obs_norm.update(base)
    raw = torch.zeros(12, dtype=torch.bool)
    raw[5] = raw[7] = True
    with torch.no_grad():
        before = model.actor(model.obs_norm(base))
        refit_normalizer(model.obs_norm, model.actor[0], base, raw)
        after = model.actor(model.obs_norm(base))
        others = ~raw
        rows = ((((base - model.obs_norm.mean) * model.obs_norm.inverse_std()).abs() < 5.0) | raw).all(-1)
    assert rows[:10].all()  # as linhas com a bandeira em 1 entram na prova
    assert float((after[rows] - before[rows]).abs().max()) < 1e-4
    assert float(model.actor[0].weight[:, 7].abs().max()) == 0.0  # constante: coluna zero
    assert bool(others.any())


def test_recenter_actor_brings_the_mean_inside_and_keeps_the_applied_action():
    model = _model()
    obs = torch.randn(3000, 12)
    lower = torch.tensor([-1.0, -1.0, -1.333, 0.0, -1.0])
    upper = torch.tensor([1.0, 1.0, 1.0, 1.0, 0.5])
    with torch.no_grad():
        mean = model.actor(obs)
    assert float(((mean < lower) | (mean > upper)).float().mean()) > 0.2
    report = recenter_actor(model, obs, lower, upper, iterations=1500, learning_rate=3e-3,
                            generator=torch.Generator().manual_seed(1))
    assert report["saturated_after"] < 0.5 * report["saturated_before"]
    assert report["applied_rms_change"] < 0.05


def _rollout(model: ActorCritic, mean_offset: float) -> Rollout:
    torch.manual_seed(3)
    steps, envs, obs, critic, actions = 8, 64, 12, 14, 5
    rollout = Rollout(steps, envs, obs, critic, actions, torch.device("cpu"))
    with torch.no_grad():
        for _ in range(steps):
            o = torch.randn(envs, obs)
            c = torch.randn(envs, critic)
            dist = model.distribution(o)
            action = dist.sample()
            rollout.add(o, c, action, dist.log_prob(action).sum(-1), model.value(c), torch.randn(envs),
                        torch.zeros(envs), dist.mean + mean_offset, dist.stddev)
        rollout.compute_returns(torch.zeros(envs), 0.99, 0.95)
    return rollout


def test_bound_loss_pulls_the_saturated_mean_back():
    model = _model()
    settings = PPOSettings(steps_per_env=8, minibatches=2, epochs=4, learning_rate=1e-3, bound_loss_coef=1.0)
    ppo = PPO(model, settings)
    obs = torch.randn(2000, 12)
    with torch.no_grad():
        before = float((model.actor(obs).abs() - 1.0).clamp(min=0.0).pow(2).sum(-1).mean())
    for _ in range(10):
        stats = ppo.update(_rollout(model, 0.0))
    with torch.no_grad():
        after = float((model.actor(obs).abs() - 1.0).clamp(min=0.0).pow(2).sum(-1).mean())
    assert stats["bound_loss"] > 0.0
    assert after < 0.5 * before


def test_kl_without_saturated_channels_changes_only_the_rate_signal():
    model = _model()
    plain = PPO(model, PPOSettings(steps_per_env=8, minibatches=2, epochs=1))
    stats = plain.update(_rollout(model, 0.0))
    assert abs(stats["kl_adapt"] - stats["kl"]) < 1e-9  # desligado: a mesma KL de sempre
    assert len(stats["kl_per_channel"]) == 5 and abs(sum(stats["kl_per_channel"]) - stats["kl"]) < 1e-5
    model2 = _model()
    excluded = PPO(model2, PPOSettings(steps_per_env=8, minibatches=2, epochs=1, kl_exclude_saturated=True))
    stats2 = excluded.update(_rollout(model2, 0.0))
    assert stats2["kl_adapt"] <= stats2["kl"] + 1e-9


def test_speed_curriculum_opens_a_cell_that_tracks_and_never_passes_the_band():
    task = {"speedCurriculum": {"enabled": True, "start": 1.0, "minEpisodes": 4, "window": 8, "minCruise": 5},
            "drills": [{"id": "parado", "pilot": {"kind": "hold"}},
                       {"id": "trote", "pilot": {"kind": "star", "bands": [{"speed": [1.8, 3.0]}, {"speed": [3.0, 4.5]}]}}]}
    curriculum = SpeedCurriculum(task, ["parado", "trote"])
    caps = curriculum.initial_caps(task)
    assert "parado" not in caps and caps["trote"] == [1.0] * CELLS
    assert band_maximum(task["drills"][1]) == 4.5
    columns = ["exercicio", "celula_queda"] + [f"cel{k:02d}.{f}" for f in ("fronteira", "rastreio_fronteira")
                                                for k in range(CELLS)]
    col = {name: i for i, name in enumerate(columns)}

    def episode(cell: int, tracking: float, fell: bool) -> np.ndarray:
        row = np.zeros(len(columns), dtype=np.float32)
        row[col["exercicio"]] = 1
        row[col["celula_queda"]] = cell if fell else -1
        row[col[f"cel{cell:02d}.fronteira"]] = 10
        row[col[f"cel{cell:02d}.rastreio_fronteira"]] = 10 * tracking
        return row

    curriculum.observe(np.stack([episode(0, 0.9, False) for _ in range(4)]), columns)
    curriculum.observe(np.stack([episode(4, 0.5, False) for _ in range(4)]), columns)
    curriculum.observe(np.stack([episode(2, 0.95, True) for _ in range(4)]), columns)
    opened = curriculum.advance(task)
    assert len(opened) == 1 and caps["trote"][0] == 1.25
    assert caps["trote"][4] == 1.0 and caps["trote"][2] == 1.0  # rastreia mal / cai: nao abre
    for _ in range(20):
        curriculum.observe(np.stack([episode(0, 0.9, False) for _ in range(4)]), columns)
        curriculum.advance(task)
    assert caps["trote"][0] == 4.5  # nunca passa da faixa
    by_bin = {"speedCurriculum": {"enabled": True, "binStart": [1.5, 1.5, 0.75, 0.75, 0.75, 0.75, 0.75, 1.5]},
              "drills": [{"id": "andar", "pilot": {"kind": "star", "bands": [{"speed": [0.8, 1.4]}]}}]}
    caps2 = SpeedCurriculum(by_bin, ["andar"]).initial_caps(by_bin)
    assert caps2["andar"][0] == 1.4 and caps2["andar"][1] == 1.4 and caps2["andar"][4] == 0.75 and caps2["andar"][15] == 1.4


def test_std_tables_by_prefix_and_groups():
    names = ["equilibrio.inclina.frente", "equilibrio.cm.x", "equilibrio.altura", "pose.x", "passada.cadencia"]
    floor = channel_values(names, {"equilibrio.inclina": 0.10, "equilibrio.cm": 0.10, "*": 0.05}, 1e-3)
    assert floor.tolist() == [np.float32(0.10), np.float32(0.10), np.float32(0.05), np.float32(0.05), np.float32(0.05)]
    assert [action_group(n) for n in names] == ["equilibrio", "equilibrio", "equilibrio", "pose", "pes"]
