"""O aprendizado guiado: a politica imita o professor (a locomocao fisica do
jogo conduzindo o corpo nos mundos de treino, num roteiro de se estabilizar,
girar, andar para todos os lados e aguentar empurroes) e o treino normal segue
com o que ela extraiu: a imitacao vira um termo do PPO que some aos poucos."""
from __future__ import annotations

import math
import re
from dataclasses import asdict, dataclass

import numpy as np
import torch

from . import paths

# O que o professor fazia no ultimo meio segundo antes de cair nao se imita.
HOLD = 15


@dataclass
class ImitationSettings:
    minutes: float = 15.0
    environments: int = 128
    # A memoria das decisoes do professor (na GPU) e o lote de cada passo.
    capacity: int = 600_000
    batch_size: int = 4096
    # Passos da imitacao a cada passo do professor (por mundo) e a taxa.
    updates_per_step: float = 0.25
    learning_rate: float = 3e-4
    # O parado da propria politica (H1/H1b), para nao esquecer: a fracao de
    # cada lote que vem dele.
    rehearsal_fraction: float = 0.2
    # Depois: o desvio da acao para o PPO (o ruido nao desmancha a passada), as
    # rodadas so do critico (o ator parado ate os valores fazerem sentido) e o
    # termo de imitacao no PPO, que some em decay_iterations.
    action_std: float = 0.45
    critic_iterations: int = 20
    decay_iterations: int = 300
    decay_weight: float = 0.5

    def to_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(data: dict | None) -> "ImitationSettings":
        """Aceita as chaves como na tarefa (camelCase) ou como aqui."""
        fields = ImitationSettings.__dataclass_fields__
        known = {}
        for key, value in (data or {}).items():
            name = re.sub(r"([A-Z])", r"_\1", key).lower()
            if name in fields:
                known[name] = value
        return ImitationSettings(**known)


class Demonstrations:
    """Decisoes (observacao crua e acao) numa memoria circular na GPU."""

    def __init__(self, capacity: int, obs_size: int, action_size: int, device: torch.device):
        self.obs = torch.zeros(capacity, obs_size, device=device)
        self.actions = torch.zeros(capacity, action_size, device=device)
        self.capacity, self.size, self.cursor = capacity, 0, 0

    def add(self, obs: torch.Tensor, actions: torch.Tensor) -> None:
        n = int(obs.shape[0])
        if n == 0:
            return
        if n > self.capacity:
            obs, actions, n = obs[-self.capacity:], actions[-self.capacity:], self.capacity
        first = min(n, self.capacity - self.cursor)
        self.obs[self.cursor:self.cursor + first] = obs[:first]
        self.actions[self.cursor:self.cursor + first] = actions[:first]
        if first < n:
            self.obs[:n - first] = obs[first:]
            self.actions[:n - first] = actions[first:]
        self.cursor = (self.cursor + n) % self.capacity
        self.size = min(self.capacity, self.size + n)

    def sample(self, count: int) -> tuple[torch.Tensor, torch.Tensor]:
        index = torch.randint(0, self.size, (count,), device=self.obs.device)
        return self.obs[index], self.actions[index]


class TeacherWorlds:
    """Os mundos conduzidos pelo professor. Cada step() devolve as decisoes que
    ja sairam da janela de queda (HOLD decisoes atras): uma queda descarta as
    do mesmo mundo que ainda estavam nela."""

    def __init__(self, native, task_path, environments: int, threads: int, seed: int, device: torch.device):
        self.batch = native.Batch(str(task_path), environments=environments, per_scene=1, threads=threads,
                                  seed=seed, teacher_assets=str(paths.ASSETS))
        n, o, a = self.batch.num_envs, self.batch.obs_size, self.batch.action_size
        self.device = device
        self.obs = torch.zeros(HOLD, n, o, device=device)
        self.actions = torch.zeros(HOLD, n, a, device=device)
        self.valid = torch.zeros(HOLD, n, dtype=torch.bool, device=device)
        self.zero = np.zeros((n, a), dtype=np.float32)
        names = self.batch.observation_names()
        self.forward = names.index("command.velocity.forward")
        self.lateral = names.index("command.velocity.lateral")
        self.cursor = 0
        self.decisions = 0
        self.episodes = 0
        self.falls = 0
        self.speed_sum = 0.0

    def step(self) -> tuple[torch.Tensor, torch.Tensor]:
        obs = torch.tensor(np.asarray(self.batch.observations()), device=self.device)
        self.batch.step(self.zero)
        actions = torch.tensor(np.asarray(self.batch.teacher_actions()), device=self.device)
        fell = torch.tensor(np.asarray(self.batch.terminated()) != 0, device=self.device)
        slot = self.cursor % HOLD
        released_obs = self.obs[slot][self.valid[slot]].clone()
        released_actions = self.actions[slot][self.valid[slot]].clone()
        self.obs[slot] = obs
        self.actions[slot] = actions
        self.valid[slot] = True
        if bool(fell.any()):
            self.valid[:, fell] = False
        self.cursor += 1
        self.decisions += int(obs.shape[0])
        finished = self.batch.finished_episodes()
        if len(finished):
            rows = np.asarray(finished)
            self.episodes += len(rows)
            self.falls += int(np.isin(rows[:, 1].astype(int), [1, 2, 3]).sum())
        # (A velocidade que ele tenta: o comando da observacao, x3.)
        self.speed_sum += float(torch.hypot(obs[:, self.forward], obs[:, self.lateral]).sum()) * 3.0
        return released_obs, released_actions

    @property
    def mean_speed(self) -> float:
        return self.speed_sum / max(1, self.decisions)

    @property
    def fall_fraction(self) -> float:
        return self.falls / max(1, self.episodes)


def imitation_loss(model, obs: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
    """O erro quadratico medio da media da politica contra a acao do professor
    (com a normalizacao das observacoes de agora, sem atualiza-la)."""
    mean = model.actor(model.obs_norm(obs))
    return (mean - targets).pow(2).mean()


def decay_weight(settings: ImitationSettings, iterations_since: int) -> float:
    """O peso do termo de imitacao no PPO: some linearmente."""
    if settings.decay_iterations <= 0:
        return 0.0
    return settings.decay_weight * max(0.0, 1.0 - iterations_since / settings.decay_iterations)


def std_log(value: float) -> float:
    return math.log(max(1e-3, value))
