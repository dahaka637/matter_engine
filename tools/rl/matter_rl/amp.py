"""O juiz de estilo (AMP; plano_claude_rl.md secao 6): uma rede que separa os
pares de instantes do acervo (os clipes tocados no corpo do RL) dos pares do
corpo, e a nota de estilo que sai dela. So no treino: o jogo nunca ve o juiz.

A conta (Peng et al., 2021): minimos quadrados com o acervo valendo +1 e o
corpo -1, a penalidade de gradiente nos pares do acervo, uma regularizacao leve
na ultima camada e a memoria dos pares recentes do corpo. A nota e
max(0, 1 - 0,25 (D - 1)^2), entre 0 e 1, e o treinador soma dt x beta x nota.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np
import torch
from torch import nn


@dataclass
class JudgeSettings:
    hidden: list[int] = field(default_factory=lambda: [1024, 512])
    learning_rate: float = 1e-4
    # Pares do acervo e do corpo em cada lote, e os lotes por rodada (os do PPO:
    # 5 epocas x 4 lotes; o Experimento A mediu o custo com 2.048).
    batch: int = 2048
    updates: int = 20
    gradient_penalty: float = 5.0
    logit_regularization: float = 0.05
    weight_decay: float = 1e-4
    # A memoria dos pares do corpo (os mais recentes).
    replay: int = 1_000_000
    # A normalizacao pelo acervo: o desvio minimo de cada medida e o corte.
    min_std: float = 0.05
    clip: float = 10.0

    def to_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(data: dict | None) -> "JudgeSettings":
        known = {k: v for k, v in (data or {}).items() if k in JudgeSettings.__dataclass_fields__}
        return JudgeSettings(**known)


def style_score(d: torch.Tensor) -> torch.Tensor:
    """A nota de estilo pela saida do juiz (0 a 1; 1 = o juiz acha que e do acervo)."""
    return torch.clamp(1.0 - 0.25 * (d - 1.0).pow(2), min=0.0)


class StyleJudge(nn.Module):
    """MLP com ReLU e uma saida."""

    def __init__(self, size: int, hidden: list[int]):
        super().__init__()
        layers: list[nn.Module] = []
        previous = size
        for width in hidden:
            linear = nn.Linear(previous, width)
            nn.init.kaiming_uniform_(linear.weight, nonlinearity="relu")
            nn.init.zeros_(linear.bias)
            layers += [linear, nn.ReLU()]
            previous = width
        self.last = nn.Linear(previous, 1)
        nn.init.uniform_(self.last.weight, -1e-3, 1e-3)
        nn.init.zeros_(self.last.bias)
        layers.append(self.last)
        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x).squeeze(-1)


class AgentReplay:
    """A memoria dos pares do corpo (na CPU, em anel)."""

    def __init__(self, capacity: int, width: int):
        self.data = np.zeros((max(1, capacity), width), dtype=np.float32)
        self.size = 0
        self.head = 0

    def add(self, rows: np.ndarray) -> None:
        rows = np.asarray(rows, dtype=np.float32)
        capacity = len(self.data)
        if len(rows) >= capacity:
            rows = rows[-capacity:]
        first = min(len(rows), capacity - self.head)
        self.data[self.head:self.head + first] = rows[:first]
        rest = len(rows) - first
        if rest:
            self.data[:rest] = rows[first:]
        self.head = (self.head + len(rows)) % capacity
        self.size = min(capacity, self.size + len(rows))

    def sample(self, count: int, rng: np.random.Generator) -> np.ndarray:
        return self.data[rng.integers(0, self.size, count)]


class Judge:
    """O juiz no treino: a rede, o acervo, a memoria do corpo, a atualizacao,
    a nota e a saude (sadia com acuracia por volta de 60-80%)."""

    def __init__(self, dataset: dict, settings: JudgeSettings, device: torch.device, seed: int = 0):
        self.settings, self.device = settings, device
        pairs = np.asarray(dataset["pairs"], dtype=np.float32)
        weights = np.asarray(dataset["pair_weight"], dtype=np.float64)
        reserved = np.zeros(len(pairs), dtype=bool)
        for clip in dataset["clips"]:
            if clip["reserved"]:
                reserved[clip["first_pair"]:clip["first_pair"] + clip["pairs"]] = True
        training = weights > 0.0
        self.width = pairs.shape[1]
        self.dataset_hash = dataset["hash"]
        # A normalizacao pelo acervo de treino: as duas metades do par sao a
        # mesma medida em instantes diferentes, e dividem a estatistica.
        half = self.width // 2
        stacked = np.concatenate([pairs[training, :half], pairs[training, half:]])
        mean = stacked.mean(0)
        std = np.maximum(stacked.std(0), settings.min_std)
        self.mean = torch.tensor(np.tile(mean, 2), device=device, dtype=torch.float32)
        self.inv_std = torch.tensor(1.0 / np.tile(std, 2), device=device, dtype=torch.float32)
        self.pairs = torch.tensor(pairs[training], device=device)
        self.weights = torch.tensor(weights[training] / weights[training].sum(), device=device, dtype=torch.float32)
        self.reserved = torch.tensor(pairs[reserved], device=device)
        self.net = StyleJudge(self.width, settings.hidden).to(device)
        self.optimizer = torch.optim.Adam(self.net.parameters(), lr=settings.learning_rate,
                                          weight_decay=settings.weight_decay)
        self.replay = AgentReplay(settings.replay, self.width)
        self.rng = np.random.default_rng(seed)

    def normalize(self, pairs: torch.Tensor) -> torch.Tensor:
        return ((pairs - self.mean) * self.inv_std).clamp(-self.settings.clip, self.settings.clip)

    @torch.no_grad()
    def score(self, pairs: torch.Tensor) -> torch.Tensor:
        """A nota de estilo de cada par do corpo (0 a 1)."""
        return style_score(self.net(self.normalize(pairs)))

    def remember(self, pairs: np.ndarray) -> None:
        self.replay.add(pairs)

    def update(self) -> dict:
        """Uma rodada do juiz. Devolve a saude (vazia ate a memoria ter um lote)."""
        s = self.settings
        if self.replay.size < s.batch:
            return {}
        totals = {"judge_loss": 0.0, "judge_real": 0.0, "judge_fake": 0.0, "judge_accuracy": 0.0,
                  "judge_gradient": 0.0, "style_agent": 0.0}
        for _ in range(s.updates):
            index = torch.multinomial(self.weights, s.batch, replacement=True)
            real = self.normalize(self.pairs[index]).requires_grad_(True)
            fake = self.normalize(torch.tensor(self.replay.sample(s.batch, self.rng), device=self.device))
            d_real, d_fake = self.net(real), self.net(fake)
            loss_real = (d_real - 1.0).pow(2).mean()
            loss_fake = (d_fake + 1.0).pow(2).mean()
            gradient = torch.autograd.grad(d_real.sum(), real, create_graph=True)[0]
            penalty = gradient.pow(2).sum(-1).mean()
            logits = self.net.last.weight.pow(2).sum()
            loss = 0.5 * (loss_real + loss_fake) + s.gradient_penalty * penalty + s.logit_regularization * logits
            self.optimizer.zero_grad(set_to_none=True)
            loss.backward()
            self.optimizer.step()
            with torch.no_grad():
                totals["judge_loss"] += float(loss)
                totals["judge_real"] += float(d_real.mean())
                totals["judge_fake"] += float(d_fake.mean())
                totals["judge_accuracy"] += 0.5 * float((d_real > 0).float().mean() + (d_fake < 0).float().mean())
                totals["judge_gradient"] += float(penalty)
                totals["style_agent"] += float(style_score(d_fake).mean())
        result = {key: value / s.updates for key, value in totals.items()}
        with torch.no_grad():
            if len(self.reserved):
                result["style_reserved"] = float(self.score(self.reserved).mean())
        return result

    def state_dict(self) -> dict:
        return {"net": self.net.state_dict(), "optimizer": self.optimizer.state_dict(),
                "mean": self.mean.cpu(), "inv_std": self.inv_std.cpu(), "dataset_hash": self.dataset_hash}

    def load_state_dict(self, state: dict) -> None:
        self.net.load_state_dict(state["net"])
        self.optimizer.load_state_dict(state["optimizer"])
        # A normalizacao do acervo em que ele aprendeu (o acervo pode ter mudado).
        self.mean = state["mean"].to(self.device)
        self.inv_std = state["inv_std"].to(self.device)
