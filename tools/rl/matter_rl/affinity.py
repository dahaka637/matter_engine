"""A afinidade com o acervo (plano_claude_rl.md secao 6.6): um juiz de
avaliacao congelado. Ele e treinado uma vez contra o acervo (+1), amostras de
varias politicas e movimentos estragados de proposito (-1), e nunca mais muda.

Mede compatibilidade com o acervo, nao naturalidade: uma recuperacao boa que
nao esta no acervo pode tirar nota baixa, e um defeito repetido nos clipes pode
tirar nota alta. E uma medida auxiliar, nunca decide sozinha (quem decide sao as
folhas). A prova de que ele mede o certo: nota alta nos clipes reservados, que
ele nunca viu.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import torch

from .amp import StyleJudge, style_score


@dataclass
class AffinitySettings:
    hidden: list[int] = field(default_factory=lambda: [1024, 512])
    learning_rate: float = 1e-4
    batch: int = 2048
    steps: int = 3000
    gradient_penalty: float = 5.0
    min_std: float = 0.05
    clip: float = 10.0


def corrupted(pairs: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Movimentos estragados de proposito a partir dos pares do acervo: ruido
    grande, metades de pares diferentes e a pose exagerada (os desvios da media
    ampliados). Congelar ou inverter o tempo nao entram: em 33 ms o corpo quase
    nao muda de pose, e o par estragado ficaria quase o de verdade."""
    count = len(pairs)
    half = pairs.shape[1] // 2
    std = pairs.std(0) + 1e-3
    mean = pairs.mean(0)
    third = max(1, count // 3)
    pick = lambda n: pairs[rng.integers(0, count, n)].copy()  # noqa: E731
    noisy = pick(third)
    noisy += rng.normal(0.0, 1.0, noisy.shape).astype(np.float32) * std
    mixed = pick(third)
    mixed[:, half:] = pick(third)[:, half:]
    exaggerated = mean + (pick(third) - mean) * 1.6
    return np.concatenate([noisy, mixed, exaggerated]).astype(np.float32)


def policy_pairs(module, task: str, policy: str, worlds: int = 16, seconds: float = 10.0, seed: int = 1) -> np.ndarray:
    """Os pares do juiz de uma politica (a acao media) em cada exercicio da
    tarefa, so dos mundos em pe (o par antes de um recomeco conta)."""
    runtime = module.Policy(policy)
    out = []
    probe = module.BodyBatch(task, environments=1, per_scene=1, threads=1, seed=seed, evaluation=True)
    for drill in probe.drill_ids:
        batch = module.BodyBatch(task, environments=worlds, per_scene=4, threads=4, seed=seed, evaluation=True,
                                 evaluation_seconds=seconds + 1.0, drill=drill)
        for _ in range(int(seconds / batch.policy_dt)):
            obs = np.ascontiguousarray(batch.observations(), dtype=np.float32)
            action = np.ascontiguousarray(runtime.evaluate(obs), dtype=np.float32)
            batch.step(action, action)
            out.append(np.array(batch.style_pairs(), dtype=np.float32))
    pairs = np.concatenate(out)
    return pairs[np.abs(pairs).sum(1) > 0.0]


class AffinityJudge:
    def __init__(self, dataset: dict, settings: AffinitySettings | None = None, device: str = "cpu"):
        self.settings = settings or AffinitySettings()
        self.device = torch.device(device)
        pairs = np.asarray(dataset["pairs"], dtype=np.float32)
        weights = np.asarray(dataset["pair_weight"], dtype=np.float64)
        reserved = np.zeros(len(pairs), dtype=bool)
        for clip in dataset["clips"]:
            if clip["reserved"]:
                reserved[clip["first_pair"]:clip["first_pair"] + clip["pairs"]] = True
        training = weights > 0.0
        self.dataset_hash = dataset["hash"]
        self.names = list(dataset["names"])
        half = pairs.shape[1] // 2
        stacked = np.concatenate([pairs[training, :half], pairs[training, half:]])
        self.mean = torch.tensor(np.tile(stacked.mean(0), 2), device=self.device)
        self.inv_std = torch.tensor(1.0 / np.tile(np.maximum(stacked.std(0), self.settings.min_std), 2), device=self.device)
        self.training_pairs = pairs[training]
        self.weights = weights[training] / weights[training].sum()
        self.reserved_pairs = pairs[reserved]
        self.net = StyleJudge(pairs.shape[1], self.settings.hidden).to(self.device)

    def normalize(self, pairs: torch.Tensor) -> torch.Tensor:
        return ((pairs - self.mean) * self.inv_std).clamp(-self.settings.clip, self.settings.clip)

    def train(self, negatives: np.ndarray, seed: int = 0) -> dict:
        """Uma vez: o acervo de treino contra os negativos (politicas e estragados)."""
        s = self.settings
        rng = np.random.default_rng(seed)
        torch.manual_seed(seed)
        optimizer = torch.optim.Adam(self.net.parameters(), lr=s.learning_rate)
        negatives = torch.tensor(negatives, device=self.device)
        for _ in range(s.steps):
            index = rng.choice(len(self.training_pairs), s.batch, p=self.weights)
            real = self.normalize(torch.tensor(self.training_pairs[index], device=self.device)).requires_grad_(True)
            fake = self.normalize(negatives[torch.randint(0, len(negatives), (s.batch,), device=self.device)])
            d_real, d_fake = self.net(real), self.net(fake)
            gradient = torch.autograd.grad(d_real.sum(), real, create_graph=True)[0]
            loss = 0.5 * ((d_real - 1.0).pow(2).mean() + (d_fake + 1.0).pow(2).mean()) \
                + s.gradient_penalty * gradient.pow(2).sum(-1).mean()
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()
        self.net.eval()
        return {"training": self.score(self.training_pairs), "reserved": self.score(self.reserved_pairs),
                "negatives": self.score(negatives.cpu().numpy())}

    @torch.no_grad()
    def score(self, pairs: np.ndarray) -> float:
        """A afinidade media (0 a 1) de um conjunto de pares."""
        if len(pairs) == 0:
            return float("nan")
        values = []
        for start in range(0, len(pairs), 8192):
            chunk = torch.tensor(np.asarray(pairs[start:start + 8192], dtype=np.float32), device=self.device)
            values.append(style_score(self.net(self.normalize(chunk))).cpu())
        return float(torch.cat(values).mean())

    def save(self, path: Path, info: dict | None = None) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        torch.save({"net": self.net.state_dict(), "mean": self.mean.cpu(), "inv_std": self.inv_std.cpu(),
                    "settings": asdict(self.settings), "dataset_hash": self.dataset_hash, "names": self.names,
                    "info": info or {}}, path)

    @staticmethod
    def load(path: Path, dataset: dict, device: str = "cpu") -> "AffinityJudge":
        state = torch.load(path, map_location=device, weights_only=False)
        if state["names"] != list(dataset["names"]):
            raise RuntimeError("o juiz de afinidade e de outras medidas: treine outro")
        judge = AffinityJudge(dataset, AffinitySettings(**state["settings"]), device)
        judge.net.load_state_dict(state["net"])
        judge.mean = state["mean"].to(judge.device)
        judge.inv_std = state["inv_std"].to(judge.device)
        judge.net.eval()
        return judge


def describe(result: dict) -> str:
    return json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in result.items()}, ensure_ascii=False)
