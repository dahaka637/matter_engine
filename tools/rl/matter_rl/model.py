"""A rede: ator e critico (MLP com ELU), a normalizacao das observacoes, a
exportacao para o C++ (.mrlpolicy) e o crescimento sem perder o que ja sabe."""
from __future__ import annotations

import json
import math
import struct
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch
from torch import nn

HIDDEN = (512, 256, 128)
# O mesmo corte do ambiente (RLActionLimit no C++).
ACTION_LIMIT = 10.0


class RunningNorm(nn.Module):
    """Media e variancia acumuladas (Welford em lote), salvas no checkpoint."""

    def __init__(self, size: int, clip: float = 5.0):
        super().__init__()
        self.clip = clip
        self.register_buffer("mean", torch.zeros(size))
        self.register_buffer("var", torch.ones(size))
        self.register_buffer("count", torch.tensor(1e-4))
        self.fixed: torch.Tensor | None = None

    @torch.no_grad()
    def fix_channels(self, mask: torch.Tensor, gain: torch.Tensor | None = None) -> None:
        """Canais que passam crus (media 0, sem acumular): os comandos. Eles valem 0
        ou 1 e mudam de distribuicao de uma tarefa para outra (na H1 o comando da
        postura e sempre 0, e a variancia iria a zero). gain: um ganho fixo por canal
        (o desvio vira 1/ganho): a velocidade pedida entra como v/6 e, sem ganho, a
        rede quase nao a enxergava (a acao media mudava 0,002 rad do pedido 0,5 ao 3,0)."""
        self.fixed = mask.to(torch.bool).cpu()
        self.gain = None if gain is None else gain.to(torch.float32).cpu()
        self._hold()

    @torch.no_grad()
    def _hold(self) -> None:
        if self.fixed is not None and bool(self.fixed.any()):
            mask = self.fixed.to(self.mean.device)
            self.mean[mask] = 0.0
            self.var[mask] = 1.0
            gain = getattr(self, "gain", None)
            if gain is not None:
                self.var[mask] = (1.0 / gain.to(self.var.device)[mask].pow(2)).to(self.var.dtype)

    @torch.no_grad()
    def update(self, batch: torch.Tensor) -> None:
        batch = batch.reshape(-1, batch.shape[-1]).to(self.mean.dtype)
        n = batch.shape[0]
        if n == 0:
            return
        mean = batch.mean(0)
        var = batch.var(0, unbiased=False)
        total = self.count + n
        delta = mean - self.mean
        self.mean += delta * (n / total)
        self.var = (self.var * self.count + var * n + delta.pow(2) * self.count * n / total) / total
        self.count = total
        self._hold()

    def inverse_std(self) -> torch.Tensor:
        return torch.rsqrt(self.var + 1e-8)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return ((x - self.mean) * self.inverse_std()).clamp(-self.clip, self.clip)


def mlp(inputs: int, outputs: int, hidden=HIDDEN, last_gain: float = 1.0) -> nn.Sequential:
    layers: list[nn.Module] = []
    previous = inputs
    for width in hidden:
        linear = nn.Linear(previous, width)
        nn.init.orthogonal_(linear.weight, gain=math.sqrt(2.0))
        nn.init.zeros_(linear.bias)
        layers += [linear, nn.ELU()]
        previous = width
    last = nn.Linear(previous, outputs)
    nn.init.orthogonal_(last.weight, gain=last_gain)
    nn.init.zeros_(last.bias)
    layers.append(last)
    return nn.Sequential(*layers)


class ActorCritic(nn.Module):
    """Politica gaussiana: a media sai do ator, o desvio e um parametro por
    canal (comeca de modo que o ruido seja ~0,15 rad em cada junta). As camadas
    escondidas do ator e do critico sao configuraveis (o Corpo v2 usa
    1024-512-256 nas duas); o padrao e o do contrato 1."""

    def __init__(self, obs_size: int, critic_size: int, action_size: int, initial_std: np.ndarray | None = None,
                 actor_hidden=HIDDEN, critic_hidden=HIDDEN):
        super().__init__()
        self.obs_size, self.critic_size, self.action_size = obs_size, critic_size, action_size
        self.actor_hidden, self.critic_hidden = tuple(actor_hidden), tuple(critic_hidden)
        self.actor = mlp(obs_size, action_size, self.actor_hidden, last_gain=0.01)
        self.critic = mlp(critic_size, 1, self.critic_hidden, last_gain=1.0)
        std = torch.full((action_size,), 0.3) if initial_std is None else torch.as_tensor(initial_std, dtype=torch.float32)
        self.log_std = nn.Parameter(std.clamp(0.02, 2.0).log())
        self.obs_norm = RunningNorm(obs_size)
        self.critic_norm = RunningNorm(critic_size)

    def distribution(self, obs_n: torch.Tensor) -> torch.distributions.Normal:
        mean = self.actor(obs_n)
        std = self.log_std.clamp(-5.0, 1.5).exp().expand_as(mean)
        return torch.distributions.Normal(mean, std)

    def value(self, critic_n: torch.Tensor) -> torch.Tensor:
        return self.critic(critic_n).squeeze(-1)

    @torch.no_grad()
    def act(self, obs_n: torch.Tensor, critic_n: torch.Tensor, deterministic: bool = False):
        dist = self.distribution(obs_n)
        action = dist.mean if deterministic else dist.sample()
        return action, dist.log_prob(action).sum(-1), self.value(critic_n)


# --- exportacao para o C++ ----------------------------------------------------------
def export_policy(model: ActorCritic, path: Path, meta: dict) -> None:
    """Escreve o .mrlpolicy que o PolicyRuntime3D le (o ator + a normalizacao)."""
    floats: list[np.ndarray] = []
    offset = 0

    def push(array: torch.Tensor) -> int:
        nonlocal offset
        data = array.detach().to("cpu", torch.float32).contiguous().numpy().reshape(-1)
        floats.append(data)
        start = offset
        offset += data.size
        return start

    mean_at = push(model.obs_norm.mean)
    inv_at = push(model.obs_norm.inverse_std())
    layers = []
    for module in model.actor:
        if isinstance(module, nn.Linear):
            layers.append({"inputs": module.in_features, "outputs": module.out_features,
                           "weights": push(module.weight), "bias": push(module.bias)})
    header = dict(meta)
    header.update({
        "schema": "matter-rl-policy-1",
        "observationSize": model.obs_size,
        "actionSize": model.action_size,
        "activation": "elu",
        "normalizer": {"mean": mean_at, "invStd": inv_at, "clip": model.obs_norm.clip},
        "actionLimit": ACTION_LIMIT,
        "layers": layers,
        "createdUtc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    })
    text = json.dumps(header, ensure_ascii=False).encode("utf-8")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with open(temporary, "wb") as handle:
        handle.write(b"MRLPOL01")
        handle.write(struct.pack("<I", len(text)))
        handle.write(text)
        handle.write(b"\0" * ((-(12 + len(text))) % 4))
        for data in floats:
            handle.write(data.astype("<f4").tobytes())
    temporary.replace(path)


# --- crescer sem esquecer -------------------------------------------------------------
@torch.no_grad()
def migrate_inputs(model: ActorCritic, old_obs: list[str], new_obs: list[str], old_critic: list[str],
                   new_critic: list[str], old_actions: list[str] | None = None, new_actions: list[str] | None = None,
                   new_log_std: torch.Tensor | None = None) -> ActorCritic:
    """Uma rede para a entrada (e a saida) nova a partir da antiga, fazendo
    exatamente o mesmo no primeiro instante: os canais que ja existiam levam os
    pesos deles, os novos entram com peso zero (e normalizacao neutra). Uma
    acao nova (um canal de saida novo) nasce com pesos e vies zero e o desvio
    dado (new_log_std, um por canal da saida nova): a acao media dela e zero e
    a politica antiga segue igual. So acrescentar: o que some ou muda de nome
    nao migra (o treinador recusa antes)."""
    old_actions = list(old_actions) if old_actions is not None else None
    new_actions = list(new_actions) if new_actions is not None else None
    actions_changed = old_actions is not None and new_actions is not None and old_actions != new_actions
    action_size = len(new_actions) if actions_changed else model.action_size
    fresh = ActorCritic(len(new_obs), len(new_critic), action_size, actor_hidden=model.actor_hidden,
                        critic_hidden=model.critic_hidden).to(next(model.parameters()).device)
    last_index = [i for i, m in enumerate(model.actor) if isinstance(m, nn.Linear)][-1]
    skip = ("actor.0.", "critic.0.", "obs_norm.", "critic_norm.")
    if actions_changed:
        skip += (f"actor.{last_index}.", "log_std")
    fresh.load_state_dict({k: v for k, v in model.state_dict().items() if not k.startswith(skip)}, strict=False)
    if actions_changed:
        fresh_last = fresh.actor[last_index]
        last = model.actor[last_index]
        fresh_last.weight.zero_()
        fresh_last.bias.zero_()
        if new_log_std is not None:
            fresh.log_std.copy_(new_log_std.to(fresh.log_std.device))
        position = {name: i for i, name in enumerate(old_actions)}
        for j, name in enumerate(new_actions):
            i = position.get(name)
            if i is None:
                continue
            fresh_last.weight[j] = last.weight[i]
            fresh_last.bias[j] = last.bias[i]
            fresh.log_std[j] = model.log_std[i]
    for network, norm, fresh_network, fresh_norm, old, new in (
            (model.actor, model.obs_norm, fresh.actor, fresh.obs_norm, old_obs, new_obs),
            (model.critic, model.critic_norm, fresh.critic, fresh.critic_norm, old_critic, new_critic)):
        first, fresh_first = network[0], fresh_network[0]
        fresh_first.weight.zero_()
        fresh_first.bias.copy_(first.bias)
        fresh_norm.mean.zero_()
        fresh_norm.var.fill_(1.0)
        fresh_norm.count.copy_(norm.count)
        position = {name: i for i, name in enumerate(old)}
        for j, name in enumerate(new):
            i = position.get(name)
            if i is None:
                continue
            fresh_first.weight[:, j] = first.weight[:, i]
            fresh_norm.mean[j] = norm.mean[i]
            fresh_norm.var[j] = norm.var[i]
    return fresh


@torch.no_grad()
def rescale_actions(model: ActorCritic, factor: torch.Tensor) -> None:
    """Se a escala de um canal de acao mudar (radianos por unidade), a rede
    passa a pedir a mesma junta: a saida e o desvio sao multiplicados."""
    last = [m for m in model.actor if isinstance(m, nn.Linear)][-1]
    last.weight.mul_(factor[:, None])
    last.bias.mul_(factor)
    model.log_std.add_(factor.log())
