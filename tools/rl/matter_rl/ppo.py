"""PPO enxuto (os valores de partida do rsl_rl): GAE, recorte, valor recortado,
taxa de aprendizado adaptada pelo KL e o desvio aprendido por canal."""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import torch
from torch import nn

from .model import ActorCritic


@dataclass
class PPOSettings:
    steps_per_env: int = 24
    epochs: int = 5
    minibatches: int = 4
    gamma: float = 0.99
    lam: float = 0.95
    clip: float = 0.2
    value_coef: float = 1.0
    entropy_coef: float = 0.003
    learning_rate: float = 3e-4
    # O piso do rsl_rl: com o desvio da acao caindo, o mesmo passo da mais KL,
    # e um piso alto (era 5e-5) deixava a taxa colada nele com o KL acima do alvo.
    min_learning_rate: float = 1e-5
    max_learning_rate: float = 1e-3
    # O KL e somado nas 39 juntas: o alvo do rsl_rl (0,01 para 12 juntas)
    # vezes ~2 (medido: com 0,01 a taxa ia ao minimo em 8 rodadas).
    desired_kl: float = 0.02
    learning_rate_step: float = 1.3
    max_grad_norm: float = 1.0
    # O teto do desvio da acao (unidades de acao; 0 = sem teto). Na imitacao, o
    # bonus de entropia fazia o desvio subir sem parar (1,2 -> 1,4 em 40 min,
    # ~0,35 rad de ruido por junta) e o treino caia bem mais que a bateria.
    max_std: float = 0.0
    # O Corpo v2 (plano de recuperacao, 5.3): o critico com otimizador, taxa e
    # corte de gradiente proprios (parar o ator pelo KL nao pode secar o
    # critico), e a parada das atualizacoes do ator quando o KL contra quem
    # coletou passa de kl_stop_factor x o alvo (0 = desligada).
    separate_critic: bool = False
    critic_learning_rate: float = 3e-4
    critic_max_grad_norm: float = 1.0
    kl_stop_factor: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(data: dict) -> "PPOSettings":
        known = {k: v for k, v in data.items() if k in PPOSettings.__dataclass_fields__}
        return PPOSettings(**known)


class Rollout:
    """As amostras de uma rodada: steps x envs, ja normalizadas como a politica viu."""

    def __init__(self, steps: int, envs: int, obs: int, critic: int, actions: int, device: torch.device):
        self.steps, self.envs = steps, envs
        shape = (steps, envs)
        self.obs = torch.zeros(*shape, obs, device=device)
        # As observacoes cruas (antes da normalizacao): o espelho as espelha e
        # normaliza os dois lados com a mesma estatistica congelada.
        self.raw = torch.zeros(*shape, obs, device=device)
        self.critic = torch.zeros(*shape, critic, device=device)
        self.actions = torch.zeros(*shape, actions, device=device)
        self.log_probs = torch.zeros(shape, device=device)
        self.values = torch.zeros(shape, device=device)
        self.rewards = torch.zeros(shape, device=device)
        self.dones = torch.zeros(shape, device=device)
        self.advantages = torch.zeros(shape, device=device)
        self.returns = torch.zeros(shape, device=device)
        self.mean = torch.zeros(*shape, actions, device=device)
        self.std = torch.zeros(*shape, actions, device=device)
        # A politica de onde o KL que ajusta a taxa e medido (None = a que
        # coletou). Com a coleta sobreposta, os dados sao de uma rodada antes:
        # medir contra quem coletou somaria a mudanca da atualizacao anterior e
        # cortaria a taxa pela metade. A razao do PPO continua contra quem coletou.
        self.anchor_mean: torch.Tensor | None = None
        self.anchor_std: torch.Tensor | None = None
        self.position = 0

    def add(self, obs, critic, actions, log_probs, values, rewards, dones, mean, std, raw=None) -> None:
        t = self.position
        self.obs[t], self.critic[t], self.actions[t] = obs, critic, actions
        if raw is not None:
            self.raw[t] = raw
        self.log_probs[t], self.values[t], self.rewards[t], self.dones[t] = log_probs, values, rewards, dones
        self.mean[t], self.std[t] = mean, std
        self.position += 1

    def compute_returns(self, last_values: torch.Tensor, gamma: float, lam: float) -> None:
        advantage = torch.zeros_like(last_values)
        for t in reversed(range(self.steps)):
            next_values = last_values if t == self.steps - 1 else self.values[t + 1]
            not_done = 1.0 - self.dones[t]
            delta = self.rewards[t] + gamma * next_values * not_done - self.values[t]
            advantage = delta + gamma * lam * not_done * advantage
            self.advantages[t] = advantage
        self.returns = self.advantages + self.values

    def minibatches(self, count: int):
        total = self.steps * self.envs
        size = total // count
        order = torch.randperm(total, device=self.obs.device)
        flat = lambda x: x.reshape(total, *x.shape[2:])  # noqa: E731
        mean = self.mean if self.anchor_mean is None else self.anchor_mean
        std = self.std if self.anchor_std is None else self.anchor_std
        data = [flat(x) for x in (self.obs, self.critic, self.actions, self.log_probs, self.values,
                                  self.advantages, self.returns, mean, std)]
        for start in range(0, size * count, size):
            index = order[start:start + size]
            yield [x[index] for x in data]


class PPO:
    def __init__(self, model: ActorCritic, settings: PPOSettings):
        self.model = model
        self.settings = settings
        self.learning_rate = settings.learning_rate
        self.critic_optimizer = None
        if settings.separate_critic:
            # Dois otimizadores: o ator (e o desvio) com a taxa adaptada pelo KL,
            # o critico com a dele.
            self.optimizer = torch.optim.Adam([*model.actor.parameters(), model.log_std], lr=self.learning_rate)
            self.critic_optimizer = torch.optim.Adam(model.critic.parameters(), lr=settings.critic_learning_rate)
        else:
            self.optimizer = torch.optim.Adam(model.parameters(), lr=self.learning_rate)

    def state_dict(self) -> dict:
        state = {"optimizer": self.optimizer.state_dict(), "learning_rate": self.learning_rate}
        if self.critic_optimizer is not None:
            state["critic_optimizer"] = self.critic_optimizer.state_dict()
        return state

    def load_state_dict(self, state: dict) -> None:
        self.optimizer.load_state_dict(state["optimizer"])
        if self.critic_optimizer is not None and "critic_optimizer" in state:
            self.critic_optimizer.load_state_dict(state["critic_optimizer"])
        self.learning_rate = float(state.get("learning_rate", self.learning_rate))
        for group in self.optimizer.param_groups:
            group["lr"] = self.learning_rate

    def update(self, rollout: Rollout, actor_frozen: bool = False, auxiliary=None) -> dict:
        """Uma atualizacao do PPO. actor_frozen: so o critico aprende (o ator e o
        desvio ficam como estao: depois da imitacao, ate os valores fazerem
        sentido). auxiliary: uma funcao que devolve uma perda a somar (a
        imitacao do professor sumindo aos poucos), ou None."""
        s = self.settings
        model = self.model
        totals = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0, "kl": 0.0, "clip_fraction": 0.0,
                  "grad_norm": 0.0, "auxiliary": 0.0, "critic_grad_norm": 0.0}
        count = 0
        kl_max = 0.0
        actor_steps = 0
        halted = False
        for _ in range(s.epochs):
            for obs, critic, actions, old_log_probs, old_values, advantages, returns, old_mean, old_std in \
                    rollout.minibatches(s.minibatches):
                dist = model.distribution(obs)
                log_probs = dist.log_prob(actions).sum(-1)
                entropy = dist.entropy().sum(-1).mean()
                values = model.value(critic)
                with torch.no_grad():
                    # KL entre a politica que coletou e a de agora (gaussianas).
                    kl = (torch.log(dist.stddev / old_std) + (old_std.pow(2) + (old_mean - dist.mean).pow(2))
                          / (2.0 * dist.stddev.pow(2)) - 0.5).sum(-1).mean()
                    kl_max = max(kl_max, float(kl))
                    if s.kl_stop_factor > 0.0 and kl > s.kl_stop_factor * s.desired_kl:
                        halted = True
                    if actor_frozen:
                        pass
                    elif kl > 2.0 * s.desired_kl:
                        self.learning_rate = max(s.min_learning_rate, self.learning_rate / s.learning_rate_step)
                    elif 0.0 < kl < 0.5 * s.desired_kl:
                        self.learning_rate = min(s.max_learning_rate, self.learning_rate * s.learning_rate_step)
                    for group in self.optimizer.param_groups:
                        group["lr"] = self.learning_rate
                advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
                ratio = torch.exp(log_probs - old_log_probs)
                surrogate = -advantages * ratio
                clipped = -advantages * ratio.clamp(1.0 - s.clip, 1.0 + s.clip)
                policy_loss = torch.max(surrogate, clipped).mean()
                value_clipped = old_values + (values - old_values).clamp(-s.clip, s.clip)
                value_loss = torch.max((values - returns).pow(2), (value_clipped - returns).pow(2)).mean()
                if self.critic_optimizer is not None:
                    # O ator (se nao esta parado nem interrompido pelo KL) e o
                    # critico, cada um no seu otimizador e no seu corte.
                    grad_norm = torch.zeros(())
                    if not actor_frozen and not halted:
                        actor_loss = policy_loss - s.entropy_coef * entropy
                        extra = auxiliary() if auxiliary is not None else None
                        if extra is not None:
                            actor_loss = actor_loss + extra
                            totals["auxiliary"] += float(extra.detach())
                        self.optimizer.zero_grad(set_to_none=True)
                        actor_loss.backward()
                        grad_norm = nn.utils.clip_grad_norm_([*model.actor.parameters(), model.log_std],
                                                             s.max_grad_norm)
                        self.optimizer.step()
                        actor_steps += 1
                    self.critic_optimizer.zero_grad(set_to_none=True)
                    (s.value_coef * value_loss).backward()
                    critic_norm = nn.utils.clip_grad_norm_(model.critic.parameters(), s.critic_max_grad_norm)
                    self.critic_optimizer.step()
                    totals["critic_grad_norm"] += float(critic_norm)
                else:
                    if actor_frozen:
                        loss = s.value_coef * value_loss
                    else:
                        loss = policy_loss + s.value_coef * value_loss - s.entropy_coef * entropy
                    extra = auxiliary() if auxiliary is not None and not actor_frozen else None
                    if extra is not None:
                        loss = loss + extra
                        totals["auxiliary"] += float(extra.detach())
                    self.optimizer.zero_grad(set_to_none=True)
                    loss.backward()
                    if actor_frozen:
                        # (Sem gradiente o Adam pula o parametro: nem o embalo antigo o move.)
                        for parameter in [*model.actor.parameters(), model.log_std]:
                            parameter.grad = None
                    grad_norm = nn.utils.clip_grad_norm_(model.parameters(), s.max_grad_norm)
                    self.optimizer.step()
                totals["policy_loss"] += policy_loss.item()
                totals["value_loss"] += value_loss.item()
                totals["entropy"] += entropy.item()
                totals["kl"] += float(kl)
                totals["clip_fraction"] += float(((ratio - 1.0).abs() > s.clip).float().mean())
                totals["grad_norm"] += float(grad_norm)
                count += 1
        if s.max_std > 0.0:
            with torch.no_grad():
                model.log_std.clamp_(max=math.log(s.max_std))
        result = {key: value / max(1, count) for key, value in totals.items()}
        result["kl_max"] = kl_max
        result["actor_steps"] = actor_steps
        result["kl_halted"] = float(halted)
        result["learning_rate"] = self.learning_rate
        result["action_std"] = float(model.log_std.detach().exp().mean())
        return result
