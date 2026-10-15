"""O espelho no treino (plano_claude_rl.md secao 8; a forma escolhida no
Experimento A: uma perda so no ator, sem mexer na conta do PPO). A acao no
estado espelhado tem de ser a acao espelhada. O mapa vem do corpo (Mirror3D:
os nomes dos canais e os sinais medidos nas poses), pela ponte."""
from __future__ import annotations

import numpy as np
import torch


class ChannelMirror:
    """Um layout espelhado: cada canal vem de source vezes scale mais offset."""

    def __init__(self, data: dict, device: torch.device):
        self.source = torch.as_tensor(np.asarray(data["source"], dtype=np.int64), device=device)
        self.scale = torch.as_tensor(np.asarray(data["scale"], dtype=np.float32), device=device)
        self.offset = torch.as_tensor(np.asarray(data["offset"], dtype=np.float32), device=device)

    def __call__(self, x: torch.Tensor) -> torch.Tensor:
        return x[..., self.source] * self.scale + self.offset


class MirrorLoss:
    """|| media(espelho(s)) - espelho(media(s)) ||^2, somado nos eixos. A entrada
    chega normalizada (como o PPO guarda): ela volta ao valor cru, e espelhada e
    normalizada de novo."""

    def __init__(self, maps: dict, device: torch.device):
        self.observation = ChannelMirror(maps["observation"], device)
        self.action = ChannelMirror(maps["action"], device)

    def __call__(self, model, obs, raw: bool = False, norm: tuple | None = None) -> torch.Tensor:
        """raw=True: a entrada e a observacao CRUA (como o lote entrega): ela e
        espelhada crua e os dois lados passam pela mesma estatistica congelada.
        raw=False: a entrada ja esta normalizada; volta ao valor cru pela
        estatistica de agora (o corte em +-5 nao se desfaz: use raw=True no
        treino)."""
        stats = model.obs_norm
        if norm is not None:
            # A estatistica com que a rodada foi coletada (media, inverso do desvio).
            mean, inverse = norm
            normalize = lambda x: ((x - mean) * inverse).clamp(-stats.clip, stats.clip)  # noqa: E731
        else:
            normalize = stats
        if raw:
            observed = normalize(obs)
        else:
            observed = obs
            obs = obs * torch.sqrt(stats.var + 1e-8) + stats.mean
        mirrored_n = normalize(self.observation(obs))
        mean = model.actor(observed)
        return (model.actor(mirrored_n) - self.action(mean)).pow(2).sum(-1).mean()
