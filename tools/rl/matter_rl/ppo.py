"""PPO enxuto (os valores de partida do rsl_rl): GAE, recorte, valor recortado,
taxa de aprendizado adaptada pelo KL e o desvio aprendido por canal."""
from __future__ import annotations

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
    min_learning_rate: float = 5e-5
    max_learning_rate: float = 1e-3
    # O KL e somado nas 39 juntas: o alvo do rsl_rl (0,01 para 12 juntas)
    # vezes ~2 (medido: com 0,01 a taxa ia ao minimo em 8 rodadas).
    desired_kl: float = 0.02
    learning_rate_step: float = 1.3
    max_grad_norm: float = 1.0

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
        self.position = 0

    def add(self, obs, critic, actions, log_probs, values, rewards, dones, mean, std) -> None:
        t = self.position
        self.obs[t], self.critic[t], self.actions[t] = obs, critic, actions
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
        data = [flat(x) for x in (self.obs, self.critic, self.actions, self.log_probs, self.values,
                                  self.advantages, self.returns, self.mean, self.std)]
        for start in range(0, size * count, size):
            index = order[start:start + size]
            yield [x[index] for x in data]


class PPO:
    def __init__(self, model: ActorCritic, settings: PPOSettings):
        self.model = model
        self.settings = settings
        self.learning_rate = settings.learning_rate
        self.optimizer = torch.optim.Adam(model.parameters(), lr=self.learning_rate)

    def state_dict(self) -> dict:
        return {"optimizer": self.optimizer.state_dict(), "learning_rate": self.learning_rate}

    def load_state_dict(self, state: dict) -> None:
        self.optimizer.load_state_dict(state["optimizer"])
        self.learning_rate = float(state.get("learning_rate", self.learning_rate))
        for group in self.optimizer.param_groups:
            group["lr"] = self.learning_rate

    def update(self, rollout: Rollout) -> dict:
        s = self.settings
        model = self.model
        totals = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0, "kl": 0.0, "clip_fraction": 0.0,
                  "grad_norm": 0.0}
        count = 0
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
                    if kl > 2.0 * s.desired_kl:
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
                loss = policy_loss + s.value_coef * value_loss - s.entropy_coef * entropy
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                grad_norm = nn.utils.clip_grad_norm_(model.parameters(), s.max_grad_norm)
                self.optimizer.step()
                totals["policy_loss"] += policy_loss.item()
                totals["value_loss"] += value_loss.item()
                totals["entropy"] += entropy.item()
                totals["kl"] += float(kl)
                totals["clip_fraction"] += float(((ratio - 1.0).abs() > s.clip).float().mean())
                totals["grad_norm"] += float(grad_norm)
                count += 1
        result = {key: value / max(1, count) for key, value in totals.items()}
        result["learning_rate"] = self.learning_rate
        result["action_std"] = float(model.log_std.detach().exp().mean())
        return result
