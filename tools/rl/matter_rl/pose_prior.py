"""A nota fixa de pose (sem adversario): o quanto a pose do corpo se parece com a
pose media de um acervo, nas features de estilo so de pose.

O juiz (AMP) satura num acervo quase parado: ele separa pelo tremor da
exploracao (as velocidades) e por qualquer diferenca de pose, que a normalizacao
pelo proprio clipe (desvio no piso) amplia (diario 28). Aqui o alvo e fixo e a
nota e tolerante, com gradiente em toda parte:

    S = max sobre (clipe, espelho) de exp(-media dos grupos da media de ((x - ref) / sigma)^2)

Os grupos: os angulos dos eixos (".position", rad), os afastamentos da cabeca,
das maos e dos pes em relacao a pelve (".offset_heading.", escala da altura) e a
altura da pelve. Um sigma de desvio em todo canal da exp(-1) ~ 0,37.
"""
from __future__ import annotations

import numpy as np
import torch

DEFAULT_SIGMAS = {"position": 0.25, "offset": 0.10, "height": 0.05}


class PoseScorer:
    def __init__(self, dataset: dict, settings: dict, device: torch.device):
        names = list(dataset["names"])
        half = len(names)
        pairs = np.asarray(dataset["pairs"], dtype=np.float32)
        frames = pairs[:, :half]
        mirrored = np.asarray(dataset.get("pair_mirrored", np.zeros(len(pairs))), dtype=bool)
        sigmas = {**DEFAULT_SIGMAS, **settings.get("sigmas", {})}
        groups = {
            "position": [i for i, n in enumerate(names) if n.endswith(".position")],
            "offset": [i for i, n in enumerate(names) if ".offset_heading." in n],
            "height": [i for i, n in enumerate(names) if n == "pelvis.height"],
        }
        self.half = half
        self.groups = [(torch.tensor(index, device=device), float(sigmas[key])) for key, index in groups.items() if index]
        references = [frames[~mirrored].mean(0)]
        if mirrored.any():
            references.append(frames[mirrored].mean(0))
        self.references = [torch.tensor(r, device=device, dtype=torch.float32) for r in references]

    def __call__(self, pairs: torch.Tensor) -> torch.Tensor:
        """A nota de cada par do corpo (0 a 1), pelo primeiro quadro do par."""
        x = pairs[:, : self.half]
        best = None
        for reference in self.references:
            error = sum((((x[:, index] - reference[index]) / sigma) ** 2).mean(-1) for index, sigma in self.groups)
            score = torch.exp(-error / len(self.groups))
            best = score if best is None else torch.maximum(best, score)
        return best
