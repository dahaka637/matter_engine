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
        # A memoria da estatistica (amostras; 0 = sem limite, o de sempre): com
        # limite, as rodadas novas pesam o bastante para a media acompanhar a tarefa.
        self.max_count = 0.0
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
        if self.max_count > 0.0 and float(self.count) > self.max_count:
            self.count = torch.as_tensor(self.max_count, dtype=self.count.dtype, device=self.count.device)
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
        self.floor_noise: tuple | None = None

    def distribution(self, obs_n: torch.Tensor) -> torch.distributions.Normal:
        mean = self.actor(obs_n)
        std = self.log_std.clamp(-5.0, 1.5).exp().expand_as(mean)
        if self.floor_noise is not None:
            # Mais exploracao perto do chao (o levantar): o desvio sobe com a pelve baixa.
            index, high, low, factor, body = self.floor_noise
            height = obs_n[..., index] * torch.sqrt(self.obs_norm.var[index] + 1e-8) + self.obs_norm.mean[index]
            if body is not None:
                # (O corpo variavel: os limiares em metros x s; jogador.altura = (H - 1,80)/0,10, cru.)
                height = height / (1.0 + obs_n[..., body] * (0.10 / 1.80))
            near = ((high - height) / max(high - low, 1e-3)).clamp(0.0, 1.0)
            std = std * (1.0 + (factor - 1.0) * near).unsqueeze(-1)
        return torch.distributions.Normal(mean, std)

    def value(self, critic_n: torch.Tensor) -> torch.Tensor:
        return self.critic(critic_n).squeeze(-1)

    @torch.no_grad()
    def act(self, obs_n: torch.Tensor, critic_n: torch.Tensor, deterministic: bool = False):
        dist = self.distribution(obs_n)
        action = dist.mean if deterministic else dist.sample()
        return action, dist.log_prob(action).sum(-1), self.value(critic_n)


# --- exportacao para o C++ ----------------------------------------------------------
def export_policy(model: ActorCritic, path: Path, meta: dict, *, action_limit: float = ACTION_LIMIT) -> None:
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
        "actionLimit": action_limit,
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


@torch.no_grad()
def refit_normalizer(norm: RunningNorm, first: nn.Linear, samples: torch.Tensor, raw: torch.Tensor | None = None,
                     count: float | None = None, refit_continuous: bool = True) -> None:
    """A normalizacao refeita sem mudar a funcao (vault .../pesquisas/03_otimizacao_ppo.md, Q3).

    Os canais normalizados ganham media e desvio novos das amostras, e a primeira camada absorve
    a troca, W' = W.s'/s e b' = b + W.(m'-m)/s: igual onde nenhuma das duas normalizacoes corta.
    Os canais em raw passam a crus (media 0, desvio 1) e a coluna deles reproduz a contribuicao
    antiga - com o corte de antes - em dois pontos: nas bandeiras (dois valores) os dois valores
    observados, exata nos dois (a absorcao linear transformaria o 1 cortado em +5 num +1/desvio);
    nos continuos os extremos dentro da faixa que a antiga nao cortava (exata ali). Canal
    constante nas amostras (a rede nunca o viu variar): coluna zero, como um canal novo (o vies
    guarda o valor). refit_continuous=False: so as bandeiras mudam (a funcao igual em todo
    estado; os continuos acompanham a tarefa pelo limite da memoria da normalizacao)."""
    old_mean, old_inv = norm.mean.clone().double(), norm.inverse_std().clone().double()
    samples = samples.reshape(-1, samples.shape[-1]).to(norm.mean.dtype)
    raw_mask = (raw.to(torch.bool).to(norm.mean.device) if raw is not None
                else torch.zeros(samples.shape[-1], dtype=torch.bool, device=norm.mean.device))
    new_mean = samples.mean(0)
    new_var = samples.var(0, unbiased=False)
    if not refit_continuous:
        # (So as bandeiras mudam: os continuos ficam com a estatistica de agora - a
        # funcao igual onde ela cortava - e acompanham a tarefa pelo limite da memoria.)
        new_mean = norm.mean.clone()
        new_var = norm.var.clone()
    new_mean[raw_mask] = 0.0
    new_var[raw_mask] = 1.0
    norm.mean.copy_(new_mean)
    norm.var.copy_(new_var)
    if count is not None:
        norm.count.fill_(count)
    new_inv = norm.inverse_std().double()
    weight = first.weight.double()
    bias = first.bias.double()
    out_weight = weight * (old_inv / new_inv).unsqueeze(0)
    bias = bias + weight @ (old_inv * (new_mean.double() - old_mean))
    clip = float(norm.clip)
    for j in torch.nonzero(raw_mask).flatten().tolist():
        lo, hi = float(samples[:, j].min()), float(samples[:, j].max())
        if torch.unique(samples[:, j]).numel() > 2:
            # Continuo: os extremos dentro da faixa que a antiga nao cortava (exata ali).
            reach = clip / float(old_inv[j])
            lo, hi = max(lo, float(old_mean[j]) - reach), min(hi, float(old_mean[j]) + reach)
        old_at = lambda v: weight[:, j] * max(-clip, min(clip, (v - float(old_mean[j])) * float(old_inv[j])))  # noqa: E731
        # (A conta acima ja somou a absorcao linear deste canal ao vies: desfaz e refaz pelos extremos.)
        bias = bias - weight[:, j] * float(old_inv[j]) * (float(new_mean[j]) - float(old_mean[j]))
        if hi - lo < 1e-6:
            out_weight[:, j] = 0.0
            bias = bias + old_at(lo)
        else:
            slope = (old_at(hi) - old_at(lo)) / (hi - lo)
            out_weight[:, j] = slope
            bias = bias + old_at(lo) - slope * lo
    first.weight.copy_(out_weight.to(first.weight.dtype))
    first.bias.copy_(bias.to(first.bias.dtype))


def recenter_actor(model: ActorCritic, observations: torch.Tensor, lower: torch.Tensor, upper: torch.Tensor,
                   iterations: int = 3000, learning_rate: float = 1e-4, batch: int = 4096,
                   generator: torch.Generator | None = None, margin: float = 0.05) -> dict:
    """O recentramento do ator (vault .../pesquisas/08_plano_consenso.md, rodada 3): a media
    que passou do limite de um canal volta para dentro dele (lower..upper, a unidade da rede:
    o contrato corta ali, entao e a acao aplicada): a acao deterministica no mundo fica a
    mesma e a media volta para onde o gradiente e a exploracao existem. observations: as
    entradas ja normalizadas. Devolve a fracao fora dos limites antes e depois e quanto a
    acao aplicada mudou (o maximo, o RMS e o p99).
    shortcut: regressao no ator inteiro (todas as camadas), o suficiente para os ~50 k estados
    do treino; se a acao aplicada mudar demais fora deles, regredir so a ultima camada."""
    def applied(mean: torch.Tensor) -> torch.Tensor:
        return torch.maximum(torch.minimum(mean, upper), lower)

    def outside(mean: torch.Tensor) -> float:
        return float(((mean < lower) | (mean > upper)).float().mean())

    # O alvo fica margin para dentro do limite (no limite exato, metade das amostras
    # sobra um fio fora pelo erro da regressao); a acao aplicada muda no maximo margin.
    inner_lower = torch.where(lower < 0.0, lower + margin, lower)
    inner_upper = torch.where(upper > 0.0, upper - margin, upper)
    with torch.no_grad():
        original = model.actor(observations)
        reference = applied(original)
        target = torch.maximum(torch.minimum(original, inner_upper), inner_lower)
        before = outside(original)
    optimizer = torch.optim.Adam(model.actor.parameters(), lr=learning_rate)
    total = observations.shape[0]
    for _ in range(iterations):
        index = torch.randint(0, total, (min(batch, total),), device=observations.device, generator=generator)
        prediction = model.actor(observations[index])
        loss = (prediction - target[index]).pow(2).mean()
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        optimizer.step()
    with torch.no_grad():
        mean = model.actor(observations)
        after = outside(mean)
        change = applied(mean) - reference
    return {"saturated_before": before, "saturated_after": after, "applied_max_change": float(change.abs().max()),
            "applied_rms_change": float(change.pow(2).mean().sqrt()),
            "applied_p99_change": float(change.abs().flatten().quantile(0.99)) if change.numel() < 16_000_000 else None}
