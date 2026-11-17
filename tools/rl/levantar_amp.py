#!/usr/bin/env python3
"""O treino do levantar livre (31/10; vault 04_locomocao/plano_compostos_31_10.md, "O levantar livre"): uma rede so
para levantar de qualquer queda (de brucos, de costas, de lado), pelo AMP (Peng et al. 2021, "AMP: Adversarial Motion
Priors for Stylized Physics-Based Character Control"): PPO com a nota da tarefa (ficar de pe, do lote) mais a do
estilo, de um discriminador que separa as transicoes do corpo das dos clipes (os de levantar de brucos e de costas -
com o rolamento -, os espelhados e o parado). Nada de seguir o clipe quadro a quadro.

Do artigo (secao 6 e o material suplementar): discriminador 1024-512 ReLU por minimos quadrados (alvo +1 nos clipes,
-1 no corpo), penalidade do gradiente nos clipes, nota do estilo max(0, 1 - 0,25 (d - 1)^2), nota total 0,5 tarefa +
0,5 estilo (aqui 0,7 tarefa + 0,3 estilo, 31/10), uma reserva de transicoes recentes do corpo para o discriminador; ator e critico 1024-512.
Declarado: gamma 0,99 (o levantar leva segundos), lambda 0,95; a acao normalizada na faixa de cada junta (o meio
mais a metade vezes a saida, como no ASE), o desvio dela aprendido (comeca em 0,1, preso entre 0,05 e 0,5; --desvio troca); Adam (ator
5e-5, critico 2e-4, discriminador 1e-4); penalidade do gradiente 5 e regularizacao da ultima camada 0,01 (os do ASE).
Num treino novo, a rede comeca imitando a de brucos do DeepMimic (levantar_frente, 100% de brucos; warm_start) e as
primeiras 20 atualizacoes so ensinam o critico e o discriminador.

A nota por passo entra vezes 1 - gamma (o retorno entre 0 e 1).
A medida honesta (o log, linha "aval", e o laboratorio): a rede media em 64 quedas fixas, sem os comecos nos clipes -
de pe no fim, se ficou de pe alguma vez e quando, por lado.

  .venv/bin/python levantar_amp.py --treino levantar_amp --threads 3
"""
from __future__ import annotations

import argparse
import concurrent.futures
import copy
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from matter_rl import storage  # noqa: E402
from rastreador_treino import DATA, REPO, RunningNorm, native  # noqa: E402
from skill_policy import SkillHead, SplitNorm, split_state  # noqa: E402
from espelho import Espelho  # noqa: E402

LOG_STD_MIN, LOG_STD_MAX = math.log(0.05), math.log(0.5)
SIDES = ("brucos", "costas", "lado")
# O rastreio da locomocao (locomotion_feedback, por mundo): a nota e os erros de cada parte (03/11).
TRACKING_TERMS = ("nota", "pelve", "pernas", "tronco", "bracos", "pes", "velocidade", "vel_rumo", "r_vel", "r_semelhanca",
                  "r_estabilidade", "giro_pelve", "giro_cabeca", "mao_sustenta_n", "mao_empurra_n", "mao_torque_nm",
                  "coluna_graus", "cabeca_graus", "bracos_graus", "altura_pelve", "pulo_pelve", "inclinacao_graus",
                  "fora_percurso_m", "arrasto_pe",
                  # (04/11 16h30) A firmeza (a estabilidade v13): os pilares, a margem de Hof (m), o giro alem do clipe e ela.
                  "firme_equilibrio", "firme_base", "firme_corpo", "firme_ritmo", "margem_hof_m", "giro_corpo_excesso",
                  "firmeza", "escorrega_contato", "tropeco",
                  # (08/11 23h55) O portao da velocidade da nota andando (0: a nota andando nao foi calculada).
                  "portao_velocidade",
                  # (09/11 01h20) O joelho para dentro da linha quadril-tornozelo alem do clipe (m).
                  "joelho_dentro",
                  # (12/11 16h30) A liberdade sob o fogo da locomocao (0-1; ver --locomocao-fogo-liberdade).
                  "fogo_liberdade")
# Os rumos da locomocao (31/10 22h35, os basicos do B21) por exercicio: o rumo pedido (0 a frente, de 45 em 45 graus
# para a esquerda) - as diagonais com os dois lados, como no B21.
HEADINGS = {"frente": (0,), "diagonal_frente": (1, 7), "esquerda": (2,), "diagonal_costas": (3, 5), "tras": (4,),
            "direita": (6,)}
# As faixas dos rumos da rl_final (a coluna 25 dos episodios).
BANDS = ("andar", "trote", "corrida", "sprint")


# (04/11 15h, os passos de recuperacao - vault 04_locomocao/plano_passos_de_recuperacao_04_11.md) Todos os lotes do levantar no
# mesmo layout: com --equilibrio, o levantar-v7 (as entradas do equilibrio e do passo antes da habilidade, os 4 canais do passo
# depois dos eixos da acao).
BATCH_EXTRA: dict = {}
STEP_CHANNELS = 4


# (06/11 17h40) A velocidade-alvo na acao (--acao-velocidade): ligada em todo lote, antes de qualquer uso do tamanho da acao.
VELOCITY_ACTIONS = [False]


def get_up_batch(*args, **kwargs):
    b = native.GetUpAmpBatch(*args, **{**BATCH_EXTRA, **kwargs})
    if VELOCITY_ACTIONS[0]:
        b.set_velocity_actions(True)
    return b


def grow_balance(state: dict, observations: int, actions: int, model) -> dict:
    """A rede de antes no layout do equilibrio: as entradas novas entram ANTES da habilidade (a ultima, que roteia as cabecas)
    com peso zero e a normalizacao neutra (calibrada depois, calibrate_new_columns); as saidas novas (os canais do passo) com
    peso zero e o desvio de exploracao de 0,1; o meio e a metade da faixa da acao, os da rede nova. A rede faz o mesmo de
    antes no primeiro instante."""
    out = dict(state)
    for key in [k for k in out if k.endswith(".0.weight") and k.startswith(("actor.", "critic."))]:
        w = out[key]
        extra = observations - w.shape[1]
        if extra > 0:
            out[key] = torch.cat([w[:, :-1], torch.zeros(w.shape[0], extra, dtype=w.dtype, device=w.device), w[:, -1:]], 1)
    for key in [k for k in out if k.startswith("norm.") and k.endswith((".mean", ".var"))]:
        v = out[key]
        extra = observations - v.shape[0]
        if extra > 0:
            fill = 0.0 if key.endswith(".mean") else 1.0
            out[key] = torch.cat([v[:-1], torch.full((extra,), fill, dtype=v.dtype, device=v.device), v[-1:]])
    for key in [k for k in out if k.startswith("actor.") and k.endswith((".4.weight", ".4.bias"))]:
        v = out[key]
        extra = actions - v.shape[0]
        if extra > 0:
            out[key] = torch.cat([v, torch.zeros((extra,) + tuple(v.shape[1:]), dtype=v.dtype, device=v.device)], 0)
    for key in ("log_std", "loco_log_std"):
        if key in out and actions > out[key].shape[0]:
            v = out[key]
            out[key] = torch.cat([v, torch.full((actions - v.shape[0],), math.log(0.1), dtype=v.dtype, device=v.device)])
    for key in ("action_mid", "action_half"):
        out[key] = getattr(model, key).detach().clone()
    return out


def calibrate_new_columns(model, batch, device: torch.device, start: int, stop: int, decisions: int = 48) -> dict:
    """As estatisticas das entradas novas (start:stop) nas duas normalizacoes, medidas numa coleta com a rede como esta (a
    recuperacao congelada daqui em diante, como o resto dela). Com os pesos delas em zero, a saida nao muda."""
    rows = {True: [], False: []}
    with torch.no_grad():
        for _ in range(decisions):
            obs = torch.as_tensor(batch.observations(), device=device)
            moving = obs[:, -1] > .5
            for flag in (True, False):
                sel = moving if flag else ~moving
                if sel.any():
                    rows[flag].append(obs[sel][:, start:stop].double())
            action = model.env_action(model.actor(model.norm(obs)), obs)
            batch.step(action.cpu().numpy().astype(np.float32))
            batch.finished_episodes()
        out = {}
        for flag, norm in ((True, model.norm.locomotion), (False, model.norm.recovery)):
            if rows[flag]:
                x = torch.cat(rows[flag])
                norm.mean[start:stop] = x.mean(0)
                # (O piso do desvio, 5 cm/0,05: uma entrada quase sempre zero - o estado do passo fora do idle - nao explode.)
                norm.var[start:stop] = x.var(0, unbiased=False).clamp_min(0.0025)
                out["locomocao" if flag else "recuperacao"] = int(len(x))
    return out


# (17/11, o plano de desentortar - Fase 1, item 6; --loco-saidas-vivas) As saidas sem efeito na marcha, medidas na r130240 com a
# suite/portao/sonda_saidas_mortas.py (+0,8 numa saida em toda decisao, o corpo bit a bit igual): na corrida e no comando, as 4 do passo e
# 21 velocidades-alvo (a torcao da coluna, o pescoco, a cabeca e os bracos); no parado so o passo.vai. Nas linhas da marcha (o clipe de
# ciclo, a coluna 0 do espelho) o ruido delas nao mexe no corpo: so balancava a razao do PPO e o freio de KL.
LOCO_DEAD_OUTPUTS = ("passo.vai", "passo.pouso_frente", "passo.pouso_esquerda", "passo.tempo", "vel.Abdomen.twist", "vel.Chest.twist",
                     "vel.UpperChest.twist", "vel.Neck.twist", "vel.Neck.swing1", "vel.Neck.swing2", "vel.Head.twist", "vel.Head.swing1",
                     "vel.Head.swing2", "vel.LeftUpperArm.twist", "vel.LeftUpperArm.swing1", "vel.LeftUpperArm.swing2",
                     "vel.LeftForearm.twist", "vel.LeftHand.twist", "vel.LeftHand.swing1", "vel.RightUpperArm.twist",
                     "vel.RightUpperArm.swing1", "vel.RightUpperArm.swing2", "vel.RightForearm.twist", "vel.RightHand.twist",
                     "vel.RightHand.swing1")


def skill_logp(model, dist, act, obs, gait=None):
    """O log da probabilidade da acao, sem os canais do passo nas linhas da recuperacao (ela nao os usa: aprenderia ruido) e, com
    --loco-saidas-vivas (model.gait_dead), sem as saidas mortas nas linhas da marcha (gait: o clipe de ciclo, do espelho)."""
    lp = dist.log_prob(act)
    dead = getattr(model, "gait_dead", None)
    if dead is not None and gait is not None:
        lp = torch.where(gait.unsqueeze(-1) & dead, torch.zeros_like(lp), lp)
    if model.split and lp.shape[-1] > len(model.joint_names) and model.joint_names:
        channel = torch.arange(lp.shape[-1], device=lp.device) >= len(model.joint_names)
        if model.recovery_velocity:
            # (17h55: com a velocidade no levantar, so os canais do passo ficam fora - as velocidades depois deles contam.)
            channel = channel & (torch.arange(lp.shape[-1], device=lp.device) < len(model.joint_names) + model.step_count)
        lp = torch.where((obs[..., -1:] <= .5) & channel, torch.zeros_like(lp), lp)
    return lp.sum(-1)


def set_single_brain(model) -> None:
    """(13/11, ele: "um brain so para tudo") A cabeca da locomocao comanda tambem o levantar: ator, critico, normalizacao e desvio
    dela em todas as linhas; a ultima entrada (a habilidade, que o ambiente segue pondo) vira a fase - 0 levantando, 1 de pe."""
    model.single = model.actor.single = model.critic.single = model.norm.single = True


def merge_norm_single(model, share: float, names, rows: int = 4096):
    """(13/11, --cerebro-unico) Uma vez: a normalizacao da locomocao (so de pe: deitado, 26,7 das 343 entradas por linha passavam do
    corte em 5 dela - a altura, a orientacao, as velocidades angulares - e o levantar ficava sem elas) vira, nas entradas que o
    levantar precisa, a mistura dela com a da recuperacao (share = a fracao das linhas do levantar); a primeira camada do ator e do
    critico da locomocao e compensada: z_antigo = (s_novo z_novo + m_novo - m_antigo) / s_antigo, entao W' = W s_novo/s_antigo e
    b' = b + W (m_novo - m_antigo)/s_antigo - fora do corte a saida e a mesma.
    As entradas misturadas: as que cortariam mais de 0,5% das linhas do levantar (normal pela media e desvio da recuperacao); fora os
    contatos com o chao (0/1: cortado em 5 a informacao fica, e misturar mudava a locomocao nos tropecos) e as que no levantar sao
    constantes (o estado do passo). Medido na coleta do treino (r109450): de pe, as linhas com acao mudada > 0,01 caem de 6,0% (tudo
    misturado) para 0,9%; deitado, 1,8 entradas cortadas por linha (eram 26,7). A habilidade passa crua (fica).
    Devolve a maior diferenca em estados de pe sinteticos (sem corte) e quantas entradas misturou."""
    loco, rec = model.norm.locomotion, model.norm.recovery
    gen = torch.Generator(device=loco.mean.device).manual_seed(13)
    with torch.no_grad():
        m_old, s_old = loco.mean.float().clone(), torch.sqrt(loco.var.float() + 1e-8)
        sl, sr = torch.sqrt(loco.var + 1e-8), torch.sqrt(rec.var + 1e-8)
        phi = lambda z: 0.5 * (1.0 + torch.erf(z / math.sqrt(2.0)))  # noqa: E731
        cut = 1.0 - (phi((loco.mean + 5.0 * sl - rec.mean) / sr) - phi((loco.mean - 5.0 * sl - rec.mean) / sr))
        contact = torch.tensor([n.startswith("chao.contato.") for n in names], device=cut.device)
        chosen = (cut > 0.005) & ~contact & (sr > 0.1 * sl)
        chosen[-1] = False
        # (Estados de pe a ate 3 desvios: sem corte nem antes nem depois - a mistura tem s_novo >= 0,84 s_antigo com share 0,3.)
        x = m_old + s_old * torch.randn(rows, m_old.numel(), generator=gen, device=m_old.device).clamp(-3.0, 3.0)
        x[:, -1] = 1.0
        z = lambda: torch.cat([loco(x)[:, :-1], x[:, -1:]], 1)  # noqa: E731 (como a SplitNorm: a habilidade crua)
        before = [model.actor.locomotion(z()), model.critic.locomotion(z())]
        mean = (1.0 - share) * loco.mean + share * rec.mean
        var = (1.0 - share) * (loco.var + loco.mean ** 2) + share * (rec.var + rec.mean ** 2) - mean ** 2
        mean = torch.where(chosen, mean, loco.mean)
        var = torch.where(chosen, var, loco.var).clamp_min(0.0)
        m_new, s_new = mean.float(), torch.sqrt(var.float() + 1e-8)
        for head in (model.actor.locomotion, model.critic.locomotion):
            weight, bias = head[0].weight, head[0].bias
            bias += weight[:, :-1] @ ((m_new - m_old) / s_old)[:-1]
            weight[:, :-1] *= (s_new / s_old)[:-1]
        loco.mean.copy_(mean)
        loco.var.copy_(var)
        after = [model.actor.locomotion(z()), model.critic.locomotion(z())]
    return max(float((a - b).abs().max()) for a, b in zip(after, before)), int(chosen.sum())


def sync_single_brain(model) -> None:
    """(13/11, --cerebro-unico) A cabeca da recuperacao como copia da locomocao (ao salvar e exportar): o laboratorio, a exportacao e
    as sondas escolhem a cabeca pela ultima entrada - com as duas iguais, todas rodam o cerebro unico sem mudar nada."""
    with torch.no_grad():
        model.actor.recovery.load_state_dict(model.actor.locomotion.state_dict())
        model.critic.recovery.load_state_dict(model.critic.locomotion.state_dict())
        model.norm.recovery.load_state_dict(model.norm.locomotion.state_dict())
        floor = getattr(model, "loco_log_floor", None)
        model.log_std.copy_(model.loco_log_std if floor is None else torch.maximum(model.loco_log_std, floor))


def anchor_distance(model, obs, target):
    """(E5) A distancia da ancora no campeao: a MSE da acao media normalizada da rede (com a normalizacao dela) contra a do campeao."""
    return (model.actor(model.norm(obs)) - target).square().mean()


def mlp(inputs: int, outputs: int) -> nn.Sequential:
    return nn.Sequential(nn.Linear(inputs, 1024), nn.ReLU(), nn.Linear(1024, 512), nn.ReLU(), nn.Linear(512, outputs))


# (23h10) O pedido de deslocamento (a entrada comando.velocidade) ate o qual o corpo conta como de pe para a autoridade.
THREAT_STANDING_SPEED = 0.05


DEMAND_FROM, DEMAND_SPAN = 1.0, 1.5
DEMAND_NAMES = ("comando.alvo_cos", "comando.alvo_sen", "comando.velocidade", "comando.rumo_cos", "comando.rumo_sen",
                "eq.vel_cm.frente", "eq.vel_cm.lado")


def demand_open(obs: torch.Tensor, idx) -> torch.Tensor:
    """O quanto a demanda abre (0-1, smoothstep), por linha (Policy.limit)."""
    cm, sm, v, cf, sf, vx, vy = (obs[..., i:i + 1] for i in idx)
    speed = 5.0 * v
    dx = speed * (cm * cf - sm * sf) - vx
    dy = speed * (sm * cf + cm * sf) - vy
    y = ((torch.sqrt(dx * dx + dy * dy) - DEMAND_FROM) / DEMAND_SPAN).clamp(0.0, 1.0)
    return y * y * (3.0 - 2.0 * y)


def configure_threat(model, batch, reach: float, demand: float = 1.0) -> None:
    """A autoridade sob ameaca (Policy.limit) num modelo: os indices das entradas e os eixos que abrem; e, andando, a da
    demanda."""
    if reach <= 1.0 and demand <= 1.0:
        return
    names = list(batch.observation_names())
    if "eq.ameaca" not in names or "comando.velocidade" not in names:
        return
    model.threat_index = names.index("eq.ameaca")
    model.speed_index = names.index("comando.velocidade")
    model.threat_reach = max(1.0, float(reach))
    if demand > 1.0 and all(n in names for n in DEMAND_NAMES):
        model.demand_indices = tuple(names.index(n) for n in DEMAND_NAMES)
        model.demand_reach = float(demand)
    model.threat_axes = torch.as_tensor([0.0 if n.startswith("passo.") else 1.0 for n in batch.action_names()],
                                        dtype=torch.float32, device=model.action_half.device)


def configure_idle_noise(model, batch, level: float) -> None:
    """(10/11, --ruido-parado) Os indices do pedido e da velocidade do corpo para Policy.idle_noise_scale."""
    names = list(batch.observation_names())
    if all(n in names for n in ("comando.velocidade", "eq.vel_cm.frente", "eq.vel_cm.lado")):
        model.speed_index = names.index("comando.velocidade")
        model.idle_vel_indices = (names.index("eq.vel_cm.frente"), names.index("eq.vel_cm.lado"))
        model.idle_noise = float(level)
        print(f"exploracao parada: o desvio x {level} com o pedido zero e o corpo parado (de volta ao inteiro ate 0,5 m/s)", flush=True)


def piso_do_desvio(model, batch, args, device="cpu") -> None:
    """O piso do desvio da locomocao (07/11, --ruido-velocidade: nos alvos de velocidade ele cai junto; --piso-posicao nos eixos)."""
    if args.habilidades_separadas and (args.ruido_velocidade != 1.0 or args.piso_posicao > 0):
        # (11/11, E5-13: com a cabeca de autoridade inteira, o piso dos eixos dela vezes o alcance de antes - o ruido efetivo igual.)
        head = lambda n: args.cabeca_autoridade and any(t in n.split(".")[0] for t in ("Neck", "Head"))
        model.loco_log_floor = torch.tensor([LOG_STD_MIN + math.log(args.ruido_velocidade) if n.startswith("vel.")
                                             else math.log(args.piso_posicao * (args.cabeca_clipe if head(n) else 1.0))
                                             if args.piso_posicao > 0 and not n.startswith("passo.")
                                             else LOG_STD_MIN for n in batch.action_names()], device=device)


def preparar_politica(model, batch, args) -> None:
    """O que a politica usa alem dos pesos (o treino e a suite montam igual): os eixos sem os passos e as velocidades, a
    velocidade da recuperacao e o ruido parado."""
    model.joint_names = tuple(nm for nm in batch.action_names() if not nm.startswith(("passo.", "vel.")))
    model.recovery_velocity = args.levantar_velocidade is not None
    model.step_count = sum(1 for nm in batch.action_names() if nm.startswith("passo."))
    if args.ruido_parado < 1.0:
        configure_idle_noise(model, batch, args.ruido_parado)


class Policy(nn.Module):
    """A acao da rede e normalizada na faixa de cada junta (como no AMP/ASE): o alvo e o meio da faixa mais a metade
    dela vezes a saida - a saida zero e o meio do movimento (nao a pose em T) e o ruido e proporcional a cada junta."""

    def __init__(self, obs: int, actions: int, ranges: np.ndarray, split=False):
        super().__init__()
        self.split = split
        self.norm = SplitNorm(obs) if split else RunningNorm(obs)
        self.actor = SkillHead(obs, actions, mlp) if split else mlp(obs, actions)
        self.critic = SkillHead(obs, 1, mlp) if split else mlp(obs, 1)
        self.log_std = nn.Parameter(torch.full((actions,), math.log(0.1)))
        if split:
            self.loco_log_std = nn.Parameter(self.log_std.detach().clone())
        low, high = torch.as_tensor(ranges[:, 0]), torch.as_tensor(ranges[:, 1])
        self.register_buffer("action_mid", 0.5 * (low + high))
        self.register_buffer("action_half", torch.clamp(0.5 * (high - low), min=0.05))

    # (03/11 14h10: com residual > 0, a cabeca da locomocao corrige o clipe - o ambiente soma o quadro do clipe a esta
    # acao, em radianos: metade da faixa vezes a saida vezes residual; a recuperacao segue com o alvo absoluto.)
    residual = 0.0
    # (04/11 04h30: o alcance da correcao por junta - a coluna, os bracos e a cabeca menores -; None: o residual em todas.)
    residual_joint = None
    # (04/11 15h: os nomes dos eixos de junta - o resto da acao sao os canais do passo.)
    joint_names: tuple = ()
    recovery_velocity = False
    step_count = 0
    # (13/11, --cerebro-unico: set_single_brain) A cabeca da locomocao em todas as linhas.
    single = False

    # (04/11 23h10, ele: "precisamos de algo com forte poder de adaptacao... a prova de fogo, os empurroes" e "ato de ficar
    # em pe na cabeca da locomocao") A autoridade sob ameaca: de pe, sem pedido de deslocamento, o limite da correcao abre
    # de 1 ate threat_reach com a ameaca da entrada eq.ameaca (smoothstep de 0,4 a 1,0: o ponto de captura chegando a borda
    # da base); andando, fica 1 (la o ponto de captura vive a frente dos pes). Os canais do passo nao abrem.
    threat_index = None
    speed_index = None
    threat_reach = 1.0
    threat_axes = None
    # (04/11 23h40, ele: "nas trocas de direcao nao ha autonomia suficiente na rede") Andando, a demanda: a velocidade
    # pedida (o rumo do deslocamento contra a vista, a vista contra o peito, comando.velocidade x 5) menos a do centro de
    # massa (eq.vel_cm, no rumo da pelve), m/s; o limite abre de 1 ate demand_reach de DEMAND_FROM a DEMAND_FROM +
    # DEMAND_SPAN (smoothstep). Na troca para tras ela chega a 2x a velocidade; num empurrao andando, tambem.
    demand_indices = None
    demand_reach = 1.0

    def reach(self):
        return self.residual if self.residual_joint is None else self.residual_joint

    def limit(self, obs: torch.Tensor):
        if self.threat_index is None or self.threat_axes is None or (self.threat_reach <= 1.0 and self.demand_reach <= 1.0):
            return 1.0
        x = ((obs[..., self.threat_index:self.threat_index + 1] - 0.4) / 0.6).clamp(0.0, 1.0)
        standing = (obs[..., self.speed_index:self.speed_index + 1].abs() <= THREAT_STANDING_SPEED).to(obs.dtype)
        out = (self.threat_reach - 1.0) * (x * x * (3.0 - 2.0 * x)) * standing
        if self.demand_indices is not None and self.demand_reach > 1.0:
            out = out + (self.demand_reach - 1.0) * demand_open(obs, self.demand_indices) * (1.0 - standing)
        return 1.0 + out * self.threat_axes

    def env_action(self, a: torch.Tensor, obs: torch.Tensor | None = None) -> torch.Tensor:
        absolute = self.action_mid + self.action_half * a
        if self.residual > 0 and obs is not None:
            # (04/11 09h45: a correcao presa ao alcance (|a| <= 1). Sem o limite a media da rede foi saindo - os bracos a
            # 2,8x o alcance - e o corpo se afastando do clipe: semelhanca 0,69 -> 0,46, os de costas freando para 1/3 do
            # pedido. Com o limite, a mesma rede a 100%: 0,50 -> 0,86 e o trote de costas 0,87 -> 2,31 m/s.)
            limit = self.limit(obs)
            clamped = a.clamp(-1.0, 1.0) if isinstance(limit, float) else torch.maximum(torch.minimum(a, limit), -limit)
            return torch.where(obs[..., -1:] > .5, self.action_half * clamped * self.reach(), absolute)
        return absolute

    # (14/11 16h25, --levantar-costas-adaptador) O ramo proprio do levantar de costas: uma rede pequena somada a acao so onde a marca
    # (levantar.variante, normalizada) passa do limiar - nas outras linhas ela vale exatamente zero. A ultima camada nasce em zero.
    has_costas_adapter = False

    def enable_costas_adapter(self, obs: int, actions: int, gate_index: int, gate_threshold: float, hidden: int = 256):
        net = nn.Sequential(nn.Linear(obs, hidden), nn.ReLU(), nn.Linear(hidden, hidden), nn.ReLU(), nn.Linear(hidden, actions))
        nn.init.zeros_(net[-1].weight)
        nn.init.zeros_(net[-1].bias)
        self.costas_adapter = net.to(self.action_mid.device)
        self.has_costas_adapter = True
        self.register_buffer("costas_gate", torch.tensor([float(gate_index), float(gate_threshold)], device=self.action_mid.device))

    def costas_gate_of(self, obs_n: torch.Tensor) -> torch.Tensor:
        return (obs_n[..., int(self.costas_gate[0])] >= self.costas_gate[1]).to(obs_n.dtype)[..., None]

    # (14/11 17h, --levantar-costas-enxerto; ele: "vamos levar dias para conseguir uma coisa tao banal nesse ritmo") O professor do de
    # costas inteiro dentro do cerebro, congelado: no de costas levantando sem empurrao (a marca, a fase 0, as entradas do fogo zeradas)
    # a acao media e a dele. A entrada dele refeita da normalizacao do principal (desfeita; os contatos 0/1 pelo zero normalizado; a
    # marca zerada, como ele viu) - medido: 0,09 entradas cortadas por linha no levantar, o resto exato.
    has_costas_expert = False

    def enable_costas_expert(self, teacher: "Policy", mark_index: int, mark_value: float, contact: torch.Tensor, push: torch.Tensor):
        dev = self.action_mid.device
        self.costas_expert = copy.deepcopy(teacher.actor.locomotion).to(dev)
        for parameter in self.costas_expert.parameters():
            parameter.requires_grad_(False)
        tn = teacher.norm.locomotion
        self.register_buffer("expert_mean", tn.mean.float().clone().to(dev))
        self.register_buffer("expert_std", torch.sqrt(tn.var.float() + 1e-8).clone().to(dev))
        self.register_buffer("expert_contact", contact.bool().to(dev))
        self.register_buffer("expert_push", push.bool().to(dev))
        self.register_buffer("expert_mark", torch.tensor([float(mark_index), float(mark_value)], device=dev))
        self.has_costas_expert = True

    def costas_expert_mean(self, obs_n: torch.Tensor, mean: torch.Tensor) -> torch.Tensor:
        nm = self.norm.locomotion if self.split else self.norm
        mm, sm = nm.mean.float(), torch.sqrt(nm.var.float() + 1e-8)
        raw = obs_n * sm + mm
        raw = torch.where(self.expert_contact, (obs_n > -mm / sm + 1e-3).to(raw.dtype), raw)
        raw = torch.cat([raw[..., :-1], obs_n[..., -1:]], -1)
        at, value = int(self.expert_mark[0]), float(self.expert_mark[1])
        clean = raw[..., -1] < 0.5
        if bool(self.expert_push.any()):
            clean = clean & (raw[..., self.expert_push].abs().sum(-1) < 1.0e-3)
        experts = [((raw[..., at] >= 0.5 * value) & clean, self.costas_expert, self.expert_mean, self.expert_std)]
        if self.has_brucos_expert:
            experts.append(((raw[..., at] <= 0.5 * float(self.brucos_mark[0])) & clean, self.brucos_expert, self.brucos_mean, self.brucos_std))
        seen = None
        for gate, net, e_mean, e_std in experts:
            if not bool(gate.any()):
                continue
            if seen is None:
                seen = raw.clone()
                seen[..., at] = 0.0
            z = torch.cat([((seen[..., :-1] - e_mean[:-1]) / e_std[:-1]).clamp(-5.0, 5.0), seen[..., -1:]], -1)
            mean = torch.where(gate[..., None], net(z), mean)
        return mean

    # (14/11 17h40, --levantar-brucos-enxerto/--levantar-brucos-marca) O de brucos pelo mesmo enxerto, com a marca negativa (-0,4) na
    # mesma entrada levantar.variante. A base, o ramo e o critico a veem zerada (o de brucos chega a eles como antes); so a porta a ve.
    has_brucos_expert = False

    def enable_brucos_expert(self, teacher: "Policy", mark_value: float):
        assert self.has_costas_expert and mark_value < 0.0
        dev = self.action_mid.device
        self.brucos_expert = copy.deepcopy(teacher.actor.locomotion).to(dev)
        for parameter in self.brucos_expert.parameters():
            parameter.requires_grad_(False)
        tn = teacher.norm.locomotion
        self.register_buffer("brucos_mean", tn.mean.float().clone().to(dev))
        self.register_buffer("brucos_std", torch.sqrt(tn.var.float() + 1e-8).clone().to(dev))
        self.register_buffer("brucos_mark", torch.tensor([float(mark_value)], device=dev))
        self.has_brucos_expert = True

    def without_brucos_mark(self, obs_n: torch.Tensor) -> torch.Tensor:
        if not self.has_brucos_expert:
            return obs_n
        at = int(self.expert_mark[0])
        nm = self.norm.locomotion if self.split else self.norm
        m, s = nm.mean[at].float(), torch.sqrt(nm.var[at].float() + 1e-8)
        out = obs_n.clone()
        out[..., at] = torch.where(obs_n[..., at] * s + m <= 0.5 * self.brucos_mark[0], -m / s, obs_n[..., at])
        return out

    def distribution(self, obs_n: torch.Tensor) -> torch.distributions.Normal:
        base_in = self.without_brucos_mark(obs_n)
        mean = self.actor(base_in)
        if self.has_costas_adapter:
            mean = mean + self.costas_gate_of(base_in) * self.costas_adapter(base_in)
        if self.has_costas_expert:
            mean = self.costas_expert_mean(obs_n, mean)
        std = self.log_std.clamp(LOG_STD_MIN, LOG_STD_MAX).exp().expand_as(mean)
        if self.split:
            # (07/11, --ruido-velocidade: o piso de cada eixo - nos alvos de velocidade, mais baixo.)
            floor = getattr(self, "loco_log_floor", None)
            loco = self.loco_log_std.clamp(LOG_STD_MIN, LOG_STD_MAX) if floor is None \
                else torch.maximum(self.loco_log_std, floor).clamp(max=LOG_STD_MAX)
            if self.single:
                # (13/11, --cerebro-unico) O desvio da locomocao em todas as linhas; a exploracao parada so de pe - deitado, com o
                # pedido zero e o corpo quieto, ela cortaria a exploracao no comeco do levantar.
                scale = self.idle_noise_scale(obs_n)
                if not isinstance(scale, float):
                    scale = torch.where((obs_n[..., -1] > .5)[..., None], scale, torch.ones_like(scale))
                std = (loco.exp() * scale).expand_as(mean)
            else:
                std = torch.where((obs_n[..., -1] > .5)[..., None], loco.exp() * self.idle_noise_scale(obs_n), std)
        return torch.distributions.Normal(mean, std)

    # (10/11 12h55, ele: "no treino de IDLE o corpo e MUITO instavel... precisa ficar paradinho corretinho"; --ruido-parado) Medido: com o
    # desvio da exploracao do treino o idle balancava o dobro (15 cm/s, 64 graus/s contra 7 e 25 da media). Parado, a exploracao na medida
    # da tarefa: o desvio vezes idle_noise com o pedido zero, voltando a 1 com a velocidade do centro de massa (0 a 0,5 m/s) - uma
    # funcao continua do estado, nao um modo (a exploracao dependente do estado, como o gSDE).
    idle_noise = 1.0
    idle_vel_indices = None

    def idle_noise_scale(self, obs_n: torch.Tensor):
        if self.idle_noise >= 1.0 or self.speed_index is None or self.idle_vel_indices is None:
            return 1.0
        norm = self.norm.locomotion if self.split else self.norm
        raw = lambda i: obs_n[..., i] * torch.sqrt(norm.var[i].float() + 1e-8) + norm.mean[i].float()
        commanded = (raw(self.speed_index).abs() <= THREAT_STANDING_SPEED).to(obs_n.dtype)
        f, s = self.idle_vel_indices
        body = torch.sqrt(raw(f) ** 2 + raw(s) ** 2)
        still = (1.0 - body / 0.5).clamp(0.0, 1.0)
        return (1.0 - (1.0 - self.idle_noise) * commanded * still)[..., None]

    def actor_parameters(self):
        return list(self.actor.parameters()) + [self.log_std] + ([self.loco_log_std] if self.split else [])

    def value(self, obs_n: torch.Tensor) -> torch.Tensor:
        return self.critic(self.without_brucos_mark(obs_n)).squeeze(-1)


class Discriminator(nn.Module):
    """D(s, s'): as duas metades normalizadas pela mesma media (as caracteristicas de um instante)."""

    def __init__(self, size: int):
        super().__init__()
        self.size = size
        self.norm = RunningNorm(size)
        self.net = mlp(2 * size, 1)

    def inputs(self, pairs: torch.Tensor) -> torch.Tensor:
        return torch.cat([self.norm(pairs[:, :self.size]), self.norm(pairs[:, self.size:])], dim=1)

    def forward(self, pairs: torch.Tensor) -> torch.Tensor:
        return self.net(self.inputs(pairs)).squeeze(-1)


class ContextDiscriminator(nn.Module):
    """(07/11, o juiz com o Motion Matching) D(c, s, s'): o par do AMP e o contexto (a consulta do Motion Matching do
    instante - a trajetoria pedida e os pes), cada um com a sua media."""

    def __init__(self, size: int, context: int):
        super().__init__()
        self.size, self.context = size, context
        self.norm = RunningNorm(size)
        self.context_norm = RunningNorm(context)
        self.net = mlp(2 * size + context, 1)

    def inputs(self, x: torch.Tensor) -> torch.Tensor:
        s = self.size
        return torch.cat([self.norm(x[:, :s]), self.norm(x[:, s:2 * s]), self.context_norm(x[:, 2 * s:])], dim=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(self.inputs(x)).squeeze(-1)


class MotionMatchingJudge:
    """(07/11, ele: "Juiz de estilo com Motion Matching... o uso mais correto possivel") Os bancos densos do Game Animation
    Sample (tools/rl/juiz_mm_bancos.json; o motor monta, src/Engine/Animation/MotionMatching) na placa de video: para cada
    linha, a consulta dela em cada grupo, so os grupos que a escolha (o chooser deles) permite, o quadro de menor custo
    (distancia ponderada ao quadrado + vieses) - o par dele e o lado "real" do juiz para aquele contexto."""

    def __init__(self, batch, device: torch.device):
        self.names = None
        self.groups = []
        for g in range(batch.motion_group_count):
            features, pairs, addends, sources, times = batch.motion_group(g)
            offset = batch.motion_query_offset(g)
            x = torch.as_tensor(features, device=device)
            self.groups.append({"x": x, "x2": (x * x).sum(1), "pairs": torch.as_tensor(pairs, device=device),
                                "add": torch.as_tensor(addends, device=device), "slice": slice(offset, offset + x.shape[1])})
        self.width = batch.motion_query_width
        main = self.groups[batch.motion_main_group]["slice"]
        self.main = main
        self.context = main.stop - main.start
        self.pair_size = self.groups[0]["pairs"].shape[1]

    def match(self, queries: torch.Tensor, chunk: int = 2048):
        """queries: linhas x (largura + 2). Devolve o par casado, o custo, o grupo e se a linha vale."""
        rows = queries.shape[0]
        device = queries.device
        best_cost = torch.full((rows,), float("inf"), device=device)
        best_pair = torch.zeros(rows, self.pair_size, device=device)
        best_group = torch.full((rows,), -1, dtype=torch.long, device=device)
        bits = queries[:, self.width].round().long()
        valid = queries[:, self.width + 1] > 0.5
        for g, group in enumerate(self.groups):
            allowed = ((bits >> g) & 1) > 0
            if not allowed.any():
                continue
            for start in range(0, rows, chunk):
                part = slice(start, min(rows, start + chunk))
                ok = allowed[part]
                if not ok.any():
                    continue
                q = queries[part, group["slice"]]
                cost = (q * q).sum(1, keepdim=True) + group["x2"][None, :] - 2.0 * q @ group["x"].T + group["add"][None, :]
                c, i = cost.min(1)
                c = torch.where(ok, c, torch.full_like(c, float("inf")))
                better = c < best_cost[part]
                if better.any():
                    idx = torch.nonzero(better).squeeze(1) + start
                    best_cost[idx] = c[better]
                    best_pair[idx] = group["pairs"][i[better]]
                    best_group[idx] = g
        return best_pair, best_cost, best_group, valid & (best_group >= 0)


def export(model: Policy, path: Path, meta: dict) -> None:
    import struct
    from datetime import datetime, timezone
    floats, offset = [], 0

    def push(array: torch.Tensor) -> int:
        nonlocal offset
        data = array.detach().to("cpu", torch.float32).contiguous().numpy().reshape(-1)
        floats.append(data)
        start = offset
        offset += data.size
        return start

    norm = model.norm.recovery if model.split else model.norm

    def normalizer(stats):
        mean, inv = stats.mean.float().clone(), 1.0 / torch.sqrt(stats.var.float() + 1e-8)
        if model.split:
            mean[-1], inv[-1] = 0.0, 1.0  # (a habilidade passa crua, como na SplitNorm)
        return push(mean), push(inv)

    mean_at, inv_at = normalizer(norm)
    # (A ultima camada ja sai em radianos: o meio e a metade da faixa de cada junta embutidos nela.)
    def actor_layers(actor, residual=0.0):
        linears = [m for m in actor if isinstance(m, nn.Linear)]
        layers = []
        for k, m in enumerate(linears):
            weight, bias = m.weight, m.bias
            if k == len(linears) - 1 and residual > 0:
                # (A correcao do clipe em radianos; quem roda soma o quadro do clipe.)
                reach = model.reach() if model.residual_joint is not None else residual
                weight = (model.action_half * reach)[:, None] * weight
                bias = model.action_half * reach * bias
            elif k == len(linears) - 1:
                weight = model.action_half[:, None] * weight
                bias = model.action_mid + model.action_half * bias
            layers.append({"inputs": m.in_features, "outputs": m.out_features, "weights": push(weight), "bias": push(bias)})
        return layers
    layers = actor_layers(model.actor.recovery if model.split else model.actor)
    locomotion_layers = actor_layers(model.actor.locomotion, model.residual) if model.split else None
    header = dict(meta)
    header.update({"schema": "matter-rl-policy-1", "observationSize": layers[0]["inputs"],
                   "actionSize": layers[-1]["outputs"], "activation": "relu",
                   "normalizer": {"mean": mean_at, "invStd": inv_at, "clip": 5.0}, "actionLimit": max(3.14159, float(model.action_half.abs().max())),
                   "layers": layers, "observationLayout": "levantar-v7" if norm.mean.numel() == 337 else "levantar-v6" if norm.mean.numel() == 308 else "levantar-v5", "actionLayout": "levantar-v1",
                   "createdUtc": datetime.now(timezone.utc).isoformat(timespec="seconds")})
    if locomotion_layers is not None:
        # (A normalizacao da cabeca da locomocao, a dela: 03/11.)
        loco_mean, loco_inv = normalizer(model.norm.locomotion)
        header.update(locomotionLayers=locomotion_layers, skillModeIndex=int(norm.mean.numel() - 1),
                      locomotionResidual=float(model.residual),
                      locomotionNormalizer={"mean": loco_mean, "invStd": loco_inv, "clip": 5.0})
        # (06/11, a locomocao livre: o alvo absoluto nao tem o limite da correcao - com o residuo 0 ele zerava a acao no jogo.)
        if model.residual > 0:
            header.update(locomotionActionLimits=push((model.action_half * (model.reach() if model.residual_joint is not None
                                                                                 else model.residual)).abs()))
        if model.threat_index is not None and model.residual > 0:
            # (23h10: a autoridade sob ameaca no jogo como no treino - os limites abertos e as entradas da ameaca e do pedido.)
            base = model.action_half * (model.reach() if model.residual_joint is not None else model.residual)
            header.update(locomotionThreatIndex=int(model.threat_index), locomotionSpeedIndex=int(model.speed_index),
                          locomotionStandingSpeed=THREAT_STANDING_SPEED,
                          locomotionThreatLimits=push((base * (1.0 + (model.threat_reach - 1.0) * model.threat_axes)).abs()))
            if model.demand_indices is not None:
                header.update(locomotionDemandIndices=[int(i) for i in model.demand_indices],
                              locomotionDemandFrom=DEMAND_FROM, locomotionDemandSpan=DEMAND_SPAN,
                              locomotionDemandLimits=push((base * (1.0 + (model.demand_reach - 1.0) * model.threat_axes)).abs()))
    text = json.dumps(header, ensure_ascii=False).encode("utf-8")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "wb") as handle:
        handle.write(b"MRLPOL01")
        handle.write(struct.pack("<I", len(text)))
        handle.write(text)
        handle.write(b"\0" * ((-(12 + len(text))) % 4))
        for data in floats:
            handle.write(data.astype("<f4").tobytes())
    tmp.replace(path)


SCENERY_GETUP = ("levantar_fixo_pilares", "levantar_fixo_corredor", "levantar_fixo_rampa",
                 "levantar_fixo_corpo", "levantar_fixo_pilha", "levantar_fixo_corredor_leve")
SCENERY_AVOID = ("evitar_fixo_parede", "evitar_fixo_corredor", "evitar_fixo_rampa",
                 "evitar_fixo_corpos1", "evitar_fixo_pilares")


# (03/11 23h30) Os clipes do juiz da locomocao: o trote nos 4 rumos, os giros e o parado em alerta.
# (06/11, a locomocao livre: o juiz e quem cuida da forma - tambem a corrida nos 4 rumos dos clipes rastreados, a largada
# do parado e os giros do idle do rastreio; so com o trote ele puxaria a corrida para o trote.)
LOCO_STYLE_CLIPS = ("jog", "jog_strafe_left", "jog_backward", "jog_strafe_right", "run_forward_medium", "sprint_strafe_left",
                    "run_backward_trote", "sprint_strafe_right", "idle_to_sprint", "giro_idle_esquerda", "giro_idle_direita",
                    "alert_idle", "alert_idle_espelho")


# (07/11, ele: "o trote vai ser a corrida desses clipes do Unreal... adaptada... a meta 4.5 m/s... o corrida vira o sprint
# deles... o levantar e excelente... e importante pegar essas transicoes... vamos ter todos os principais angulos... esses
# clipes do Unreal vao ser o nosso juiz de estilo a partir de agora"): o Game Animation Sample convertido para o nosso
# corpo (assets/animations/clips/referencias_unreal; build-profile/tmp/gasp_export). O juiz da locomocao: o trote (a
# corrida deles inteira a 4,5 m/s - linhas, curvas, partidas, freios, viradas, pivos, partidas virando, trocas de pe), a
# corrida (o sprint deles inteiro, no ritmo deles) e as transicoes corrida<->sprint; o parado e o giro continuam os do
# alerta (as regras dele de 31/10 e 04/11). O juiz da recuperacao: as levantadas deles (ate em pe e saindo correndo) e o
# parado em alerta (o de pe depois de levantar).
UNREAL_FOLDER = "referencias_unreal"
# (07/11 00h50, ele: "o Parado e giro pode ser do unreal engine tambem, eles sao muito bons": o parado e os giros parados do
# juiz tambem da Unreal - a base parada e os giros de 45-180 graus e os continuos; nada mais do alerta nos juizes.)
UNREAL_KEEP_LOCO = ()
UNREAL_KEEP_RECOVERY = ()


# (07/11, ele: "vamos treinar o levantar de costas e de brucos e... ter novos exercicios de levantar, o de levantar correndo"):
# os comecos deitados, o quadro 0 destas levantadas (de costas primeiro, de brucos depois: a ordem das avaliacoes 9 e 10).
LYING_STAND = [f"{UNREAL_FOLDER}/ragdoll_getup_stand_b", f"{UNREAL_FOLDER}/ragdoll_getup_stand_f"]
LYING_RUN = [f"{UNREAL_FOLDER}/ragdoll_getup_run_{side}" for side in ("b", "f", "l", "r")]
# (11/11 14h, ele: "Zerar absolutamente o brain do levantar-se... Serao 4 exercicios: levantar de costas, de brucos e o levantar
# correndo de costas, de brucos... Todos os levantares serao pelos clipes do unreal engine"; 13h45: o correndo "apenas ate a parte
# que ele fica de pe") O levantar pelo clipe (--levantar-unreal; o comeco 9 do C++, TrackingClipGetUp3D): os quatro clipes da Unreal
# com o chao corrigido para o nosso corpo e cortados de pe (build-profile/tmp/levantar_unreal_auditoria/corrigir_rapido.py; vault
# 04_locomocao/plano_levantar_unreal_11_11.md), na ordem dos exercicios 0 a 3 (as avaliacoes 11 a 14).
# (11/11 18h, ele: "mais um exercicio... de levantar de costas em que ele rola para ficar de brucos, ensinando esse ato de rolar" e
# "se nao me engano NAO e pelo do unreal esse rolar.. foi feito no kimodo": o quinto, o stand_up_back_kimodo - o "Levantar de costas"
# do visualizador dele -, reretargeteado do BVH com o pulso anatomico e o chao corrigido.)
# (12/11 02h50, ele: "o levantar de costas rolando nao vai funcionar bem.. ele ja esta todo torto e ferrado.. tira ele dos treinos")
# O rolando saiu (era o ultimo: os indices dos quatro ficam); o clipe, a variante e o rolamento livre seguem no codigo, parados.
LEVANTAR_UNREAL = [f"levantar/{name}" for name in ("levantar_costas", "levantar_brucos", "levantar_correndo_costas",
                                                     "levantar_correndo_brucos")]
LEVANTAR_UNREAL_KEYS = ("lev_levantar_costas", "lev_levantar_brucos", "lev_levantar_correndo_costas", "lev_levantar_correndo_brucos")
LEVANTAR_UNREAL_PREFIXES = ("lu_costas_", "lu_brucos_", "lu_correndo_costas_", "lu_correndo_brucos_")
# (18h05, ele: o rolando "parece estar imitando o levantar de costas normal") A entrada levantar.variante de cada exercicio
# (--levantar-unreal-variante): 1 no rolando, 0 nos outros - os dois de costas comecam iguais e sem ela a rede nao sabia qual fazer.
LEVANTAR_UNREAL_VARIANTS = (0.0, 0.0, 0.0, 0.0)
# (18h22) O vies dos comecos no meio do clipe (o expoente): no rolando, metade antes de ~0,8 s - onde ele rola.
LEVANTAR_UNREAL_START_BIAS = (1.0, 1.0, 1.0, 1.0)
# (12/11 00h35) O ponto de entrega de cada exercicio (s do clipe; 0: no fim dele). O correndo no clipe longo (1,90 e 1,80 s,
# levantar_v2/) com a entrega no corte dele (1,70 e 1,57 s, a pose da imagem dele): a estatistica conta ali e o clipe segue - desde
# que o episodio acabava na troca, a pose do fim tinha se perdido (de 1,4 a 1,75 s, de 9-16 cm a 24-47 cm).
# (12/11 04h05) Os de pe tambem entregam ao ficar de pe - a cabeca do clipe a 95% da final (costas 2,00 s, quadro 60; brucos 2,20 s,
# quadro 66), nao no fim do clipe: de 2,0 a 2,5 s (costas) e de 1,75 a 2,5 s (brucos) o clipe so assenta, o relogio da agilidade
# passava ali a 1,00x (medido r85750) e o tempo para levantar nunca chegaria aos 1,6 s dele. So a contagem: a troca segue no fim.
LEVANTAR_UNREAL_DELIVER = (60 / 30, 66 / 30, 51 / 30, 47 / 30)
# (12/11 03h20) Quais dos quatro sao o levantar correndo (o peso da agilidade deles e o segundo de --levantar-unreal-agilidade).
LEVANTAR_UNREAL_RUN = (False, False, True, True)
# (14/11 v1, ele: "fazer o ROLAR para virar de brucos no levantar de costas... adaptar o proprio levantar de costas correndo") O clipe
# emendado (tools/animation/clips/emendar_rolar_brucos.py; 05h40: o correndo de costas ate 0,80 s + o de costas de pe de 1,13 s) e a entrega
# dele (a do brucos deslocada).
COSTAS_ROLA = ("levantar/levantar_costas_rola", 1.67)


def usar_costas_rola(ligado) -> None:
    """(14/11 v1, --levantar-costas-rola [CLIPE ENTREGA]) O levantar de costas de pe por outro clipe (07h40: o rolar do Kimodo + o
    levantar de brucos); sem CLIPE, o de COSTAS_ROLA. O correndo de costas fica como esta."""
    global LEVANTAR_UNREAL_DELIVER
    if ligado is None or ligado is False:
        return
    clip, deliver = (ligado[0], float(ligado[1])) if isinstance(ligado, (list, tuple)) and len(ligado) >= 2 else COSTAS_ROLA
    LEVANTAR_UNREAL[0] = clip
    LEVANTAR_UNREAL_DELIVER = (deliver,) + tuple(LEVANTAR_UNREAL_DELIVER[1:])


def usar_brucos_clipe(ligado) -> None:
    """(14/11 17h35, --levantar-brucos-clipe CLIPE ENTREGA) O levantar de brucos de pe por outro clipe (o mesmo, mais rapido)."""
    global LEVANTAR_UNREAL_DELIVER
    if not ligado:
        return
    LEVANTAR_UNREAL[1] = ligado[0]
    LEVANTAR_UNREAL_DELIVER = (LEVANTAR_UNREAL_DELIVER[0], float(ligado[1])) + tuple(LEVANTAR_UNREAL_DELIVER[2:])


def clip_get_up_agility(args, counters: dict | None, full: bool = False) -> list:
    """(12/11 03h20, ele: "recompensar a velocidade e agilidade com que ele consegue levantar"; no correndo, "estimular extremamente
    tambem o IMPULSO de arrancada") O peso da agilidade de cada exercicio (TrackingClipGetUp3D::AgilityMaxRate): de pe e correndo,
    subindo de ~0 ao alvo em N atualizacoes desde a primeira com ela (o comeco nos contadores); full: o alvo (as avaliacoes, so o
    relogio importa). O piso de 0,001 deixa o relogio que acompanha o corpo ligado desde o comeco da rampa; sem os contadores (a
    montagem do lote, antes deles), o piso."""
    stand, run, ramp = args.levantar_unreal_agilidade
    if stand <= 0 and run <= 0:
        return [0.0] * len(LEVANTAR_UNREAL_RUN)
    frac = 1.0 if full else 0.0
    if not full and counters is not None:
        start = counters.setdefault("agilidade_inicio", counters["atualizacao"])
        frac = min(1.0, (counters["atualizacao"] - start) / max(1.0, float(ramp)))
    return [max(1e-3, (run if r else stand) * frac) if (run if r else stand) > 0 else 0.0 for r in LEVANTAR_UNREAL_RUN]


# (12/11 04h40, ele: a fase de fogo do levantar - vault 04_locomocao/plano_fogo_do_levantar_12_11.md) Os quatro niveis de cada exercicio:
# o continuo (fracao do peso) e as pancadas do comeco, do meio e do quase fim (m/s ao centro de massa); o teto fisico do continuo.
FIRE_GETUP_TYPES = ("continuo", "pancada_comeco", "pancada_meio", "pancada_fim")
FIRE_OBSERVATION_SIZE = 6  # (TrackingGetUpAmpTraining3D::FireObservationSize: a forca externa e a aceleracao, logo antes da habilidade)
# (05h40, ele: "calibrar a forca adequadamente para garantir que derrube e impeca o levantar 100% das vezes": nos correndo a de 100% e
# 0,55 do peso - o teto de 0,35 saiu.)
# (12/11 09h00) O continuo abaixo do atrito do corpo deitado no concreto (dinamico 0,58, estatico 0,72 - MaterialLibrary makeConcrete):
# com 0,83 do peso (o de costas, subido pela escada pelo levantou) o corpo deitado deslizava acelerando ~2,5 m/s2 e em ~6 s caia pela
# borda do chao a 60 m - 54 dos 334 de costas deitados com fogo (r89300) acabaram a -27..-160 m.
# (13h30, ele: "as pancadas... estao muito fortes (jogando o boneco de um lado para o outro)") As pancadas ate 2,5 m/s ao centro de massa
# (um tranco forte no futebol; antes ate 9 - a faixa da fase 2 levou o de costas a 7,9).
FIRE_GETUP_CAP = (0.55, 2.5, 2.5, 2.5)


def get_up_fire_state(args, counters: dict) -> dict | None:
    """O estado do fogo do levantar nos contadores (uma vez, do JSON de --levantar-fogo-niveis: os niveis de partida da F0, a forca em
    que a rede falhava ~100% - a ideia dele): os niveis de agora, os da prova (os de partida, fixos) e desde quando cada um."""
    if max(args.levantar_fogo) <= 0:
        return None
    state = counters.get("fogo_levantar")
    if not args.levantar_fogo_niveis:
        if state is None:
            raise SystemExit("--levantar-fogo pede --levantar-fogo-niveis (o JSON da F0)")
        return state
    start = json.loads(Path(args.levantar_fogo_niveis).read_text())
    # (05h40) Uma versao nova do JSON (a calibracao refeita) recomeca os niveis e a prova dele; a mesma versao segue o treino.
    version = int(start.get("_versao", 1))
    if state is None or int(state.get("versao", 1)) != version:
        state = {"niveis": {k: [float(x) for x in start[k]] for k in LEVANTAR_UNREAL_KEYS},
                 "prova": {k: [float(x) for x in start[k]] for k in LEVANTAR_UNREAL_KEYS},
                 "desde": {k: [int(counters["atualizacao"])] * 4 for k in LEVANTAR_UNREAL_KEYS}, "versao": version}
        counters["fogo_levantar"] = state
        print(f"fogo do levantar: niveis da versao {version} do JSON", flush=True)
    for key in LEVANTAR_UNREAL_KEYS:
        for t in range(4):
            if state["niveis"][key][t] > FIRE_GETUP_CAP[t]:
                print(f"fogo do levantar: {key} {FIRE_GETUP_TYPES[t]} {state['niveis'][key][t]} -> {FIRE_GETUP_CAP[t]} (o teto)", flush=True)
                state["niveis"][key][t] = FIRE_GETUP_CAP[t]
    return state


def fire_levels_flat(levels: dict) -> list:
    return [float(x) for k in LEVANTAR_UNREAL_KEYS for x in levels[k]]


FIRE_GETUP_FLOOR = (0.05, 0.3, 0.3, 0.3)
FIRE_WINDOWS = [0]  # (12h55) as janelas do log desde que este processo comecou - a escada pula as duas primeiras


def fire_extras(args) -> tuple:
    """(12/11 12h30, a fase 2) Os argumentos de set_get_up_fire_extras: a serie, a rasteira, o prensado (fracao, min, max), o vento
    (fracao, segundos) e a pancada de depois de ficar de pe."""
    return (int(args.levantar_fogo_serie), float(args.levantar_fogo_rasteira), *[float(x) for x in args.levantar_fogo_prensado],
            *[float(x) for x in args.levantar_fogo_vento], float(args.levantar_fogo_depois_de_pe), float(args.levantar_fogo_escorregadio),
            *[float(x) for x in args.levantar_fogo_agarrao], *[float(x) for x in args.levantar_fogo_serie_intervalo],
            *[float(x) for x in args.levantar_fogo_pancada_duracao], *[float(x) for x in args.levantar_fogo_pancada_newtons])
FIRE_GETUP_MASTERED, FIRE_GETUP_STUCK = 0.90, 0.10


def get_up_fire_step(state: dict, rows: np.ndarray, update: int, band: tuple | None = None, frozen: bool = False, tracked: int = 0) -> list:
    """(12/11 07h15, ele: "nossa meta e que ele consiga se levantar de forma dinamica e inteligente mediante adversidades 100% das vezes.
    Depois disso, aumentamos um pouco mais as adversidades, e garantimos a adaptacao".) A escada do fogo do levantar, partindo dos niveis
    da recalibracao de 10-30%: por exercicio, nas tentativas deitadas (a coluna 56) da faixa de cima (a fracao sorteada >= 0,8), o
    continuo medido nos episodios so com ele e cada momento da pancada no combinado. Dominado (o levantou menos 1,5 erro-padrao >= 90%,
    ~95% lido, com o ruido do treino) por >= 100 atualizacoes no nivel: sobe 10% (ate o teto). A guarda da adaptacao: travado (mais
    1,5 erro-padrao < 10%) por >= 200 atualizacoes: desce 10% (ate o piso). >= 40 episodios na janela do log (20 atualizacoes)."""
    changes = []
    if not len(rows) or rows.shape[1] < 57:
        return changes
    for e, key in enumerate(LEVANTAR_UNREAL_KEYS):
        sel = rows[(rows[:, 1] == 9) & (rows[:, 26] == e) & (rows[:, 56] <= 0.05)
                   & (((rows[:, 59].astype(int) & 128) == 0) if rows.shape[1] >= 60 else True)]
        if not len(sel):
            continue
        # (12/11 13h25, a guarda do limpo: na fase 2 o limpo do correndo de costas caiu de 100% a 5% em ~40 min - a rede desaprendeu o
        # levantar soterrada pelas adversidades.) O limpo deitado (o cenario 0) abaixo de 90% em >= 10 episodios: as pancadas do
        # exercicio descem 15% (>= 40 atualizacoes no nivel) e nada sobe; a forca so sobe com o limpo >= 95%.
        clean = sel[sel[:, 49] == 0]
        clean_rate = float((clean[:, 18] >= 0).mean()) if len(clean) >= 10 else 1.0
        # (14/11 15h30) No exercicio que segue o clipe no limpo (--levantar-rastrear), a coluna 18 vem -1 (o firme do adaptativo nao e
        # medido ali): a guarda lia 0% e baixou as pancadas do de costas de 2,5 a 0,3. Ali ela nao age (a escada segue congelada).
        if tracked and (tracked >> e) & 1:
            clean_rate = 1.0
        if clean_rate < 0.90:
            for t in range(1, 4):
                if update - state["desde"][key][t] >= 40:
                    old = state["niveis"][key][t]
                    new = round(max(FIRE_GETUP_FLOOR[t], old / 1.15), 4)
                    if new != old:
                        state["niveis"][key][t] = new
                        state["desde"][key][t] = int(update)
                        changes.append(f"{key} {FIRE_GETUP_TYPES[t]} {old} -> {new} (guarda do limpo: {clean_rate:.2f} em {len(clean)})")
            continue
        clean_ok = clean_rate >= 0.95 and not frozen
        # (07h55: o dominio e levantar E continuar de pe - "firme", sem queda depois da entrega. Medido: so com o continuo o de costas
        # levantava 99% e ficava firme 38% (inclinado contra 0,75 do peso, a forca sumindo); a escada subia pelo levantou.)
        # (08h02: a guarda pelo levantou - travado e nao conseguir nem ficar de pe; com a janela de firmeza o firme comeca baixo de
        # proposito e a guarda pelo firme baixava a forca enquanto ele aprendia a se firmar.)
        kind, timing, top = sel[:, 49], sel[:, 50], sel[:, 52] >= 0.8
        stood = sel[:, 18] >= 0
        handed = stood & ~np.isin(sel[:, 32], (1, 2, 3, 4, 8))
        for t in range(4):
            m = top & ((kind == 2) if t == 0 else (((kind == 1) | (kind == 3)) & (timing == t)))
            if m.sum() < 40:
                continue
            rate, old, held = float(handed[m].mean()), state["niveis"][key][t], update - state["desde"][key][t]
            up = float(stood[m].mean())
            # (Com ~55 episodios por momento da pancada, 20% de verdade le 10% ou 34%: so mexe com 1,5 erro-padrao de margem.)
            margin = 1.5 * max(0.02, float(np.sqrt(rate * (1.0 - rate) / m.sum())))
            up_margin = 1.5 * max(0.02, float(np.sqrt(up * (1.0 - up) / m.sum())))
            if band is not None:
                # (12/11 12h30, a fase 2 - ele: "as forcas e adversidades... para uma taxa de 30-50% de sucesso apenas") A calibracao: o
                # levantou (a definicao dele) segurado na faixa, +-15% a cada >= 40 atualizacoes com 1,5 erro-padrao de margem.
                if held < 40:
                    continue
                if up - up_margin > band[1] and clean_ok:
                    new = round(min(FIRE_GETUP_CAP[t], old * 1.15), 4)
                elif up + up_margin < band[0]:
                    new = round(max(FIRE_GETUP_FLOOR[t], old / 1.15), 4)
                else:
                    continue
            elif rate - margin >= FIRE_GETUP_MASTERED and held >= 100 and clean_ok:
                new = round(min(FIRE_GETUP_CAP[t], old * 1.10), 4)
            elif up + up_margin < FIRE_GETUP_STUCK and held >= 200:
                new = round(max(FIRE_GETUP_FLOOR[t], old / 1.10), 4)
            else:
                continue
            if new != old:
                state["niveis"][key][t] = new
                state["desde"][key][t] = int(update)
                changes.append(f"{key} {FIRE_GETUP_TYPES[t]} {old} -> {new} ({handed[m].mean():.2f} em {int(m.sum())})")
    return changes


assert fire_levels_flat({k: [1, 2, 3, 4] for k in LEVANTAR_UNREAL_KEYS})[:5] == [1.0, 2.0, 3.0, 4.0, 1.0]


def clip_get_up_summary(rows: np.ndarray, prefix: str = "") -> dict:
    """(11/11 14h) O levantar pelo clipe (comeco 9) nas colunas dos episodios: 18 a troca no fim do clipe (s; < 0 sem ela), 19 caiu
    na locomocao depois dela, 33 a nota media do clipe, 12 a velocidade do centro de massa na troca, 41 o voo de pe (s, os dois pes
    no ar com a cabeca acima de 1,2 m), 32 o fim (8: longe do clipe)."""
    if not len(rows) or rows.shape[1] < 42:
        return {}
    handed = rows[:, 18] >= 0
    kept = handed & (rows[:, 19] < .5)
    # (20h45: a coluna 33 e -1 sem nenhuma amostra rastreada - o rolando que acaba no rolamento livre sem virar -; a media com eles
    # dava -0,45. A semelhanca e a dos episodios com o clipe seguido; quem nao chegou la aparece em levantou/entregou.)
    tracked = rows[:, 33] >= 0
    out = {"episodios": int(len(rows)), "levantou": float(handed.mean()), "entregou": float(kept.mean()),
           "semelhanca": float(rows[tracked, 33].mean()) if tracked.any() else 0.0, "rastreados": float(tracked.mean()),
           "longe": float((rows[:, 32] == 8).mean()), "voo_s": float(rows[:, 41].mean())}
    if handed.any():
        out.update(tempo_troca=float(rows[handed, 18].mean()), vel_troca=float(rows[handed, 12].mean()))
    # (12/11 03h20, ele: "um novo dado e grafico de velocidade para levantar, dizendo quantos segundos demora") O tempo para levantar:
    # o ponto da entrega (s do clipe) dividido pelo ritmo - os segundos do clipe percorridos (47) pelos reais (18), somados nos
    # episodios que entregaram; nas avaliacoes (do quadro 0) e o tempo real ate a entrega. 48 o ponto da entrega.
    if rows.shape[1] >= 49:
        timed = handed & (rows[:, 47] > 0.05) & (rows[:, 18] > 0)
        if timed.any():
            pace = float(rows[timed, 47].sum() / rows[timed, 18].sum())
            out.update(levantar_s=float(rows[timed, 48].mean() / max(1e-6, pace)), ritmo=pace)
    # (12/11 04h30, ele: "os levantares correndo estao terminando muito cedo, antes dele engatar a postura ereta de corrida": desde as
    # 00h35 nada depois da entrega contava como queda, e os correndo caiam 100% uns 0,6-1,2 s depois dela - medido nas capturas de 01h05,
    # 01h42, 04h00 e 04h25 -, escondido.) "firme": entregou e o episodio nao acabou em queda depois (a corrida ereta engatada com a
    # locomocao; de pe, firme ate o fim). O grafico "De pe com a locomocao depois da troca" (de_pe) volta a dizer isso.
    firm = handed & ~np.isin(rows[:, 32], (1, 2, 3, 4, 8))  # (8: longe do clipe depois da entrega - o correndo que mergulha)
    out["firme"] = float(firm.mean())
    # (12/11 04h40, a fase de fogo do levantar - colunas 49-56) O levantou por cenario, a media deles com o mesmo peso ("sob fogo", o
    # grafico dele: "a media de todos os cenarios de levantar daquele exercicio que conseguiram"), o recuperou depois de ir ao chao e, no
    # levantar adaptativo, o tempo para levantar = quando ficou firme, nos limpos deitados do quadro 0.
    if rows.shape[1] >= 57:
        kind = rows[:, 49]
        # (12/11 07h05: metade do treino comeca no meio do clipe, ja meio de pe - --levantar-unreal-meio 0,5 -, e inflava o "sob fogo" do
        # treino a ~55% com a prova e a calibracao v4 deitadas em ~20%. As medidas do fogo sao das tentativas deitadas (a coluna 56, o
        # comeco no clipe); o meio fica em sob_fogo_meio. Nas avaliacoes todas sao deitadas.)
        lying_start = rows[:, 56] <= 0.05
        # (14h30) O comeco em pose variada (bit 128) fica fora dos deitados - medido a parte em pose_variada.
        varied = (rows[:, 59].astype(int) & 128) != 0 if rows.shape[1] >= 60 else np.zeros(len(rows), bool)
        if varied.any():
            out["pose_variada"] = float(handed[varied].mean())
            out["pose_variada_firme"] = float(firm[varied].mean())
            t = rows[varied & handed, 55]
            if (t >= 0).any():
                out["pose_variada_s"] = float(t[t >= 0].mean())
        lying_start = lying_start & ~varied

        def fire_mean(sel):
            per = {name: float(handed[sel & (kind == c)].mean()) for c, name in enumerate(("limpo", "pancada", "continuo", "combinado"))
                   if (sel & (kind == c)).any()}
            fire = [v for k, v in per.items() if k != "limpo"]
            return per, (float(np.mean(fire)) if fire else None)
        per, under = fire_mean(lying_start)
        out.update({f"fogo_{k}": v for k, v in per.items()})
        # (06h50, ele: "que apenas 10-30% das tentativas sob fogo tenham sucesso": o "sob fogo" so dos cenarios com fogo - os limpos, que dao
        # ~100%, ficam em fogo_limpo.)
        if under is not None:
            out["sob_fogo"] = under
        under_mid = fire_mean(~lying_start)[1]
        if under_mid is not None:
            out["sob_fogo_meio"] = under_mid
        # (12/11 07h50: a coluna 54 com o codigo do C++ - 1 foi ao chao e nao levantou depois, 2 foi ao chao e levantou depois, 3 levantou
        # e foi ao chao depois. O "recuperou" contava os 2 e os 3: medido nas linhas do treino, a recuperacao de verdade era ~0% (1 em 139)
        # e o resto, quem levantou e caiu depois.) O recuperou: dos que foram ao chao antes de levantar, os que levantaram depois.
        code = rows[:, 54]
        # (12/11 08h30, ele: "tentar se levantar novamente, APOS fracassar nas tentativas anteriores (nunca desistir)") Com a coluna 57 (as
        # tentativas, settings.adaptiveRetry): o fracasso e a queda antes de levantar ou uma tentativa nova; o recuperou, quem levantou
        # depois dele; tentou_de_novo, os episodios com fogo deitados que tiveram outra tentativa.
        retried = rows[:, 57] > 1.5 if rows.shape[1] >= 58 else np.zeros(len(rows), bool)
        first = lying_start & (np.isin(code, (1.0, 2.0)) | retried)
        if first.any():
            out["recuperou"] = float(((code[first] == 2.0) | (retried[first] & handed[first])).mean())
            out["chao_n"] = int(first.sum())
        if rows.shape[1] >= 58 and (lying_start & (kind != 0)).any():
            out["tentou_de_novo"] = float(retried[lying_start & (kind != 0)].mean())
        # (12/11 12h30, a fase 2) O levantou por adversidade nova (a coluna 59, bits); a pancada de depois de ficar de pe pelo firme.
        if rows.shape[1] >= 60:
            bits = rows[:, 59].astype(int)
            for bit, name in ((1, "serie"), (2, "rasteira"), (4, "prensado"), (8, "vento"), (32, "escorregadio"), (64, "agarrao")):
                sel = lying_start & ((bits & bit) != 0)
                if sel.any():
                    out[f"fogo_{name}"] = float(handed[sel].mean())
            sel = lying_start & ((bits & 16) != 0)
            if sel.any():
                out["fogo_depois_de_pe"] = float(firm[sel].mean())
        # (09h45) O tempo da ultima tentativa ate levantar (s), nos que levantaram numa tentativa nova.
        if rows.shape[1] >= 59 and (lying_start & (rows[:, 58] >= 0)).any():
            out["denovo_s"] = float(rows[lying_start & (rows[:, 58] >= 0), 58].mean())
        # O firme dado que levantou (so a durabilidade depois da entrega - a janela de firmeza -, sem o efeito da nova tentativa).
        fire_up = lying_start & (kind != 0) & handed
        if fire_up.any():
            out["firme_se_levantou"] = float(firm[fire_up].mean())
        # O firme sob fogo: levantou e nao caiu depois (a media dos cenarios com fogo, como o sob fogo) - o de costas so com o continuo
        # levantava 87% e caia depois 64%.
        # (18h55, ele: o sucesso com as linhas do limpo e do fogo "ate os de levantar") O firme limpo: levantou e nao caiu depois, sem fogo.
        if (lying_start & (kind == 0)).any():
            out["firme_limpo"] = float(firm[lying_start & (kind == 0)].mean())
        firm_fire = [float(firm[lying_start & (kind == c)].mean()) for c in (1, 2, 3) if (lying_start & (kind == c)).any()]
        if firm_fire:
            out["firme_sob_fogo"] = float(np.mean(firm_fire))
        above = lying_start & (kind != 0) & (rows[:, 51] >= 8)  # (12/11 05h15: a forca de cima - o rumo + 8)
        if above.any():
            out["fogo_de_cima"] = float(handed[above].mean())
        lying = (kind == 0) & lying_start & (rows[:, 55] >= 0)
        if lying.any():
            out["levantar_s"] = float(rows[lying, 55].mean())
    out.update(de_pe=out["firme"], pose=out["semelhanca"], **({"tempo": out["tempo_troca"]} if "tempo_troca" in out else {}))
    return {f"{prefix}{k}": v for k, v in out.items()}


# (12/11 19h10, ele: o "levantar e ir" - "inicia caido, de costas ou brucos, e precisa levantar, haver a troca de brain para o da
# locomocao e ele ir ate um ponto proximo... 5 - 10 metros") O prefixo da avaliacao dele.
LEVANTAR_E_IR_PREFIX = "lu_ir_"


def get_up_go_summary(rows: np.ndarray, prefix: str = "") -> dict:
    """(12/11 19h10) O levantar e ir (comeco 11 no registro) nas colunas: 18 a entrega/troca (s), 72 o tipo (1 de pe, 2 correndo), 73
    quando chegou ao ponto (s; -1 nao), 74 a distancia dele, 12 a velocidade na troca, 32 o fim, 49 o fogo do levantar (0 limpo). O
    sucesso (de_pe, e nos limpos firme_limpo): chegou ao ponto sem cair - a chegada encerra o episodio; 75 o que faltava no fim (m)."""
    if not len(rows) or rows.shape[1] < 76:
        return {}
    handed = rows[:, 18] >= 0
    arrived = rows[:, 73] >= 0
    fell = handed & ~arrived & np.isin(rows[:, 32], (1, 2, 3, 4, 8))
    clean, run = rows[:, 49] == 0, rows[:, 72] > 1.5
    out = {"episodios": int(len(rows)), "levantou": float(handed.mean()), "chegou": float(arrived.mean()), "de_pe": float(arrived.mean()),
           "distancia": float(rows[:, 74].mean())}
    miss = ~arrived
    if miss.any():
        out["faltou_m"] = float(rows[miss, 75].mean())
    if handed.any():
        out.update(caiu_depois=float(fell[handed].mean()), levantar_s=float(rows[handed, 18].mean()), vel_troca=float(rows[handed, 12].mean()))
    if arrived.any():
        out.update(tempo=float(rows[arrived, 73].mean()), percurso_s=float((rows[arrived, 73] - rows[arrived, 18]).mean()))
    if clean.any():
        out["firme_limpo"] = float(arrived[clean].mean())
    if (~clean).any():
        out["firme_sob_fogo"] = float(arrived[~clean].mean())
    for name, sel in (("normal", ~run), ("correndo", run)):
        if sel.any():
            out[f"chegou_{name}"] = float(arrived[sel].mean())
    return {f"{prefix}{k}": v for k, v in out.items()}


JUIZ_UNREAL_VERSAO = "unreal_corrida6"
# (07/11 08h, ele me deixou a direcao: "o importante e sairmos da situacao atual de estagnacao e regressao"; e 05h: "os clipes
# de trote e corrida, para os lados, frente e costas... voltar aos que nos tinhamos antes para o juiz; os do Unreal ficam
# apenas para arrancada, freada de parada, transicao"): com --juiz-ciclos, os ciclos do juiz sao os nossos regerados nas
# metas (tools/animation/clips/ciclos_meta.py) e da Unreal so as transicoes; o juiz da locomocao recomeca do zero uma vez (a
# marca "juiz_loco_versao"), o do levantar continua.
JUIZ_CICLOS = tuple(f"ciclo_{f}_{d}" for f in ("trote", "corrida") for d in ("frente", "esquerda", "direita", "costas"))
JUIZ_LOCO_VERSAO = "ciclos_meta_1"


def unreal_style_clips(transitions_only: bool = False) -> tuple[list, list]:
    """(locomocao, recuperacao): os ids dos clipes da Unreal do juiz, com a subpasta."""
    folder = REPO / "assets" / "animations" / "clips" / UNREAL_FOLDER
    ids = sorted(p.name[: -len(".matteranim.json")] for p in folder.glob("*.matteranim.json"))
    # (07/11, ele: "a corrida a 6 m/s, sendo 5,5 para lateral e costas": a corrida do juiz sao os corrida_* - o sprint deles
    # a 6,0 e a corrida de lado e de costas a 5,5; o sprint a 7,7 e as transicoes corrida<->sprint sairam. O giro parado: os
    # stand_turn de cada angulo - os idle_turn vieram sem giro nenhum da conversao.)
    # (O Run_Start_RL_Rfoot da Unreal veio com a raiz quebrada - salta 9,6 m no comeco e fica parada: fora do juiz.)
    loco = [i for i in ids if i.startswith(("trote_run_", "corrida_", "stand_")) and not i.endswith("start_rl_rfoot")]
    if transitions_only:
        # (Sem os ciclos em linha - de frente, de lado, de costas e em diagonal -, as curvas e os circulos: esses sao os nossos.)
        loco = [i for i in loco if not any(t in i for t in ("_loop_", "_arc_", "circle_strafe"))]
    recovery = [i for i in ids if i.startswith(("ragdoll_getup_", "stand_idle_loop"))]
    if len(loco) < 180 or len(recovery) < 11:
        raise SystemExit(f"--juiz-unreal: faltam clipes em {folder} ({len(loco)} da locomocao, {len(recovery)} do levantar)")
    return [f"{UNREAL_FOLDER}/{i}" for i in loco], [f"{UNREAL_FOLDER}/{i}" for i in recovery]


def finite_json(value):
    """O valor sem NaN nem infinito (None no lugar): o leitor de JSON do laboratorio descarta a linha inteira com eles."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    return value


MANEUVERS = ("lev_troca_suave", "lev_troca_media", "lev_troca_forte", "lev_troca_livre", "lev_curva_suave", "lev_curva_s",
             "lev_curva_fechada")


CALIBRATION_STRATEGIES = ("professor", "professor_ruido", "professor_solta", "deitado_parado", "acao_meio", "rede")


def calibrate_clip_get_up(model, evaluations: dict, device: torch.device, out: Path, training_batch=None) -> dict:
    """(11/11 14h, ele: "se as recompensas NOVAS DIFERENTES estarao devidamente calibradas") Os jeitos de agir no levantar pelo
    clipe, com a recuperacao no comando (depois da troca, a locomocao da rede): o professor nos motores (os alvos do clipe uma
    decisao a frente, velocidade alvo zero), o professor com ruido de 0,15 rad, o professor solto no meio do clipe (a acao do meio
    dali: cai), deitado parado (os alvos do quadro 0 o tempo todo), a acao do meio da faixa e a rede de agora. A ordem certa:
    professor > com ruido > solto > deitado parado. A nota de cada episodio ate a troca: a media do clipe (coluna 33) vezes as
    decisoes da recuperacao."""
    results = {}
    joints = len(model.joint_names) if model.joint_names else 39
    for name in (() if os.environ.get("CALIBRAR_SO_TROCA") else CALIBRATION_STRATEGIES):
        for exercise, prefix in enumerate(LEVANTAR_UNREAL_PREFIXES):
            b = evaluations[prefix]
            b.reset(12345)
            n, dt = b.num_envs, b.policy_dt
            ends = b.clip_get_up_ends()
            half_time = 0.5 * ends[3 * exercise]
            done, first, steps = {}, None, 0
            gen = torch.Generator(device="cpu").manual_seed(7)
            for _ in range(int(b.episode_seconds / dt) + 2):
                obs = torch.as_tensor(b.observations(), device=device)
                with torch.no_grad():
                    net = model.env_action(model.actor(model.norm(obs)), obs)
                clip = torch.as_tensor(b.locomotion_targets(), device=device, dtype=net.dtype).reshape(n, -1)
                if first is None:
                    first = clip.clone()
                middle = model.action_mid.to(net.dtype).expand_as(net).clone()
                middle[:, joints:] = 0.0
                if name == "professor":
                    act = clip
                elif name == "professor_ruido":
                    act = clip.clone()
                    act[:, :joints] += 0.15 * torch.randn(n, joints, generator=gen).to(device)
                elif name == "professor_solta":
                    act = clip if steps * dt < half_time else middle
                elif name == "deitado_parado":
                    act = first
                elif name == "acao_meio":
                    act = middle
                else:
                    act = net
                action = torch.where(obs[:, -1:] < .5, act, net)
                b.step(action.cpu().numpy().astype(np.float32))
                steps += 1
                for row in b.finished_episodes().tolist():
                    done.setdefault(int(row[0]), row)
                if len(done) >= n:
                    break
            rows = np.array(list(done.values()))
            summary = clip_get_up_summary(rows)
            recovery_seconds = np.where(rows[:, 18] >= 0, rows[:, 18], rows[:, 4])
            summary["nota_ate_troca"] = float((rows[:, 33] * recovery_seconds / dt).mean())
            summary["segundos"] = float(rows[:, 4].mean())
            results[f"{name}/{LEVANTAR_UNREAL_KEYS[exercise]}"] = summary
            print(f"calibracao {name:16s} {LEVANTAR_UNREAL_KEYS[exercise]:30s} levantou {summary['levantou']:.2f} entregou "
                  f"{summary['entregou']:.2f} nota/passo {summary['semelhanca']:.3f} nota ate a troca {summary['nota_ate_troca']:6.1f} "
                  f"longe {summary['longe']:.2f} voo {summary['voo_s']:.2f}s vel_troca {summary.get('vel_troca', float('nan')):.2f}",
                  flush=True)
    if training_batch is not None:
        # (A troca: no lote do treino - metade comeca no meio do clipe -, o professor nos motores ate o fim do clipe e a locomocao da
        # rede depois; os episodios do levantar pelo clipe que terminam em ~20 s de jogo.)
        b = training_batch
        b.reset(4321)
        n, dt, rows = b.num_envs, b.policy_dt, []
        for _ in range(int(20.0 / dt)):
            obs = torch.as_tensor(b.observations(), device=device)
            with torch.no_grad():
                net = model.env_action(model.actor(model.norm(obs)), obs)
            clip = torch.as_tensor(b.locomotion_targets(), device=device, dtype=net.dtype).reshape(n, -1)
            b.step(torch.where(obs[:, -1:] < .5, clip, net).cpu().numpy().astype(np.float32))
            rows += [r for r in b.finished_episodes().tolist() if int(r[1]) == 9]
        rows = np.array(rows)
        for exercise, key in enumerate(LEVANTAR_UNREAL_KEYS):
            summary = clip_get_up_summary(rows[rows[:, 26] == exercise]) if len(rows) else {}
            results[f"troca_professor/{key}"] = summary
            mine = rows[rows[:, 26] == exercise] if len(rows) else rows
            if len(mine):
                causes = {int(c): int((mine[:, 32] == c).sum()) for c in np.unique(mine[:, 32])}
                print(f"   fins {causes} (0 tempo, 1 caiu, 8 longe); segundos {mine[:, 4].mean():.2f} (max {mine[:, 4].max():.2f}); cabeca no "
                      f"fim {mine[:, 7].mean():.2f} m; nota {mine[:, 33].mean():.2f}", flush=True)
            print(f"troca professor  {key:30s} episodios {summary.get('episodios', 0)} levantou {summary.get('levantou', 0):.2f} "
                  f"entregou {summary.get('entregou', 0):.2f} vel_troca {summary.get('vel_troca', float('nan')):.2f} "
                  f"voo {summary.get('voo_s', 0):.2f}s", flush=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(results, indent=1, ensure_ascii=False))
    return results


def summarize_scenery(rows: np.ndarray, student_only: bool = True) -> dict:
    if not len(rows) or rows.shape[1] < 27:
        return {}
    out = {}
    for kind, drills in ((0, SCENERY_GETUP), (4, SCENERY_AVOID)):
        for preset, drill in enumerate(drills):
            selected = rows[(rows[:, 1] == kind) & (rows[:, 26] == preset)]
            if len(selected):
                out[drill] = dict(summarize(selected, ""), episodios=len(selected))
    # (03/11 14h10: o "sem cair" contava como sucesso os episodios que acabavam por frear ou virar sem tombar - a regua e
    # a fracao que chega ao fim do tempo e a duracao media.)
    # (04/11: o trote e a corrida - a coluna 25, a faixa - separados.)
    # (04/11 05h30, ele: "graficos da evolucao de semelhanca com o clipe apos levantar-se": o idle depois de ficar de pe nos
    # comecos caidos - as quedas e os comecos no clipe do levantar -, a semelhanca estrita e a quietude.)
    if rows.shape[1] > 34:
        risen = rows[np.isin(rows[:, 1], (0, 1)) & (rows[:, 33] >= 0)]
        if len(risen):
            out["lev_levantar"] = {"semelhanca": float(risen[:, 33].mean()), "estabilidade": float(risen[:, 34].mean()),
                                   "episodios": int(len(risen))}
        # (04/11 23h, ele: "refaca completamente o grafico do levantar e do evitar queda" - a estabilidade do evitar.)
        pushed = rows[(rows[:, 1] == 4) & (rows[:, 33] >= 0)]
        if len(pushed):
            out["lev_evitar"] = {"semelhanca": float(pushed[:, 33].mean()), "estabilidade": float(pushed[:, 34].mean()),
                                 "episodios": int(len(pushed))}
    # (07/11: o levantar deitado - o comeco no clipe, a coluna 2 o lado do comeco: 1 de costas, 0 de brucos - e o levantar
    # correndo, o levantar em movimento.)
    for key, kind, side in (("lev_levantar_costas", 1, 1), ("lev_levantar_brucos", 1, 0), ("lev_levantar_correndo", 3, None)):
        selected = rows[(rows[:, 1] == kind) & ((rows[:, 2] == side) if side is not None else True)]
        if len(selected) >= 8:
            out[key] = dict(summarize(selected, ""), episodios=int(len(selected)))
    # (04/11: as manobras - a coluna 26 nos rumos - com chave propria; os rumos retos sem elas.)
    plain = rows[:, 26] <= 0
    for key, kind, heading, run, maneuver in ([(f"lev_trote_{h}", 6, h, False, 0) for h in range(8)]
                                              + [(f"lev_corrida_{h}", 6, h, True, 0) for h in range(8)]
                                              + [(name, 6, None, None if code < 4 else False, code + 1)
                                                 for code, name in enumerate(MANEUVERS)]
                                              + [(name, 6, None, True, code + 5) for code, name in enumerate(RUN_CURVES)]
                                              + [("lev_giro", 7, None, None, None), ("lev_parado", 2, None, None, None)]
                                              + [("lev_sprint", 6, 0, 3, 0)]
                                              + [(name, 6, None, code >= 3, 8 + code % 3) for code, name in enumerate(STAR_KEYS)]
                                              + [(COMMAND_KEY, 6, None, None, 12)]):
        # (05/11: a faixa pelo numero - 1 o trote, 2 a corrida, 3 o sprint; antes "nao corrida" contava o sprint no trote.)
        band = None if run is None else run if not isinstance(run, bool) else 2 if run else 1
        selected = rows[(rows[:, 1] == kind) & ((rows[:, 2] == heading) if heading is not None else True)
                        & ((rows[:, 25] == band) if band is not None else True)
                        & ((plain if maneuver == 0 else rows[:, 26] == maneuver) if maneuver is not None else True)]
        # (Com a professora, so os episodios da aluna: os dela no volante nao sao da aluna - a revisao de 03/11: os graficos
        # do laboratorio mostravam a B21 como se fosse ela.)
        if student_only and rows.shape[1] >= 32:
            selected = selected[selected[:, 27] < 0.5]
        # (06/11 08h20: com menos de 8 episodios - a primeira linha depois de religar, so os que caem logo - o exercicio fica
        # fora da linha: no grafico era um mergulho a 0%.)
        if len(selected) >= 8:
            seconds = selected[:, 4]
            # shortcut: os 8 s do --locomocao-segundos e os 6 s do episodio de agora; ler dos argumentos se mudarem
            limit = STAR_SECONDS[0] - 0.1 if key.startswith("lev_estrela") else COMMAND_SECONDS[0] - 0.1 if key == COMMAND_KEY \
                else 7.9 if kind in (6, 7) else 5.9
            out[key] = dict(summarize(selected, ""), episodios=len(selected), segundos=float(seconds.mean()),
                            ate_o_fim=float((seconds >= limit).mean()))
            # (12/11 18h45, ele: o grafico "Ate o fim neste exercicio" deve "refletir corretamente o sucesso efetivo.. se ele conseguiu
            # completar sem cair etc.. e tudo mais") O sucesso: o episodio inteiro (o parado no dele, nao nos 6 s antigos), sem cair, sem
            # fim antes da hora (virado, fora do rumo, lento, pose perdida), no comando sem nenhuma queda (ali a queda nao encerra) e na
            # estrela perto do caminho (ali sair dele nao encerra mais).
            full = STAR_SECONDS[0] if key.startswith("lev_estrela") else COMMAND_SECONDS[0] if key == COMMAND_KEY \
                else 8.0 if kind in (6, 7) else IDLE_SECONDS[0]
            ok = (seconds >= full - 0.1) & (selected[:, 13] < 0.5) & (selected[:, 32] == 0)
            if key == COMMAND_KEY and selected.shape[1] > 68:
                ok &= selected[:, 68] == 0
            if key.startswith("lev_estrela") and STAR_PATH_WIDTH[0] > 0 and selected.shape[1] > 66:
                ok &= selected[:, 66] <= STAR_PATH_WIDTH[0]
            out[key]["sucesso"] = float(ok.mean())
            # (18h55, ele: "ter 2 linhas, a do limpo e a do fogo") O sucesso nos limpos e nos com o fogo da locomocao (a coluna 60).
            if selected.shape[1] > 60:
                fire = selected[:, 60] > 0
                for name, sel in (("sucesso_limpo", ~fire), ("sucesso_fogo", fire)):
                    if sel.sum() >= 3:
                        out[key][name] = float(ok[sel].mean())
            if kind == 6:
                # (04/11 12h35, ele: o grafico da velocidade em % do pedido - a medida da regua da forca, cada episodio ate 1.)
                out[key]["vel_pedido"] = float(np.clip(selected[:, 12] / np.maximum(selected[:, 10], 0.1), 0.0, 1.0).mean())
            if selected.shape[1] > 46 and (selected[:, 46] >= 0).any():
                # (08/11 15h20: a parada - o erro medio contra o ponto de referencia, m, e a fracao com parada.)
                stopped = selected[selected[:, 46] >= 0]
                out[key].update(parada_erro=float(stopped[:, 46].mean()), com_parada=float(len(stopped) / len(selected)))
            if (key.startswith("lev_estrela") or key == COMMAND_KEY) and selected.shape[1] > 44:
                # (08/11: a estrela - as pernas feitas por episodio e a fase.)
                out[key].update(pernas=float(selected[:, 43].mean()), fase=float(selected[:, 44].mean()))
            if rows.shape[1] > 34:
                # (04/11: a semelhanca com o clipe e a estabilidade por exercicio - os graficos do laboratorio.)
                tracked = selected[selected[:, 33] >= 0]
                if len(tracked):
                    out[key].update(semelhanca=float(tracked[:, 33].mean()), estabilidade=float(tracked[:, 34].mean()))
            if rows.shape[1] >= 32:
                # (A professora: a fracao dela no volante, as retomadas e o desvio do deslocamento contra o rumo pedido.)
                out[key].update(professora=float(selected[:, 27].mean()), retomadas=float(selected[:, 28].mean()),
                                desvio_rumo=float(selected[:, 31].mean()),
                                desvio_rumo_abs=float(np.abs(selected[:, 31]).mean()))
        elif len(selected):
            # (13/11 08h25, ele: "porque ha treinos desativados?" - com 75% dos episodios no levantar, os exercicios raros da locomocao
            # ficam com 1-7 episodios por janela e saiam da linha: o laboratorio os pintava de cinza. So a contagem: o ativo ve, os
            # graficos nao ganham o ponto ruidoso.)
            out[key] = {"episodios": int(len(selected))}
    # (11/11 14h: o levantar pelo clipe, por exercicio - a coluna 26.)
    for exercise, key in enumerate(LEVANTAR_UNREAL_KEYS):
        selected = rows[(rows[:, 1] == 9) & (rows[:, 26] == exercise)]
        if len(selected):
            out[key] = clip_get_up_summary(selected)
    # (12/11 19h20: o levantar e ir - o comeco 11 no registro.)
    if rows.shape[1] >= 76 and (rows[:, 1] == 11).any():
        out["lev_levantar_e_ir"] = get_up_go_summary(rows[rows[:, 1] == 11])
    return out


# A forca auxiliar por exercicio (04/11 03h45): a ordem do drillAssist do ambiente.
# (04/11 11h50, ele: "adicionar os exercicios das curvas agora em corridas": as curvas 5-7 na faixa da corrida, 25-27.)
RUN_CURVES = ("lev_curva_suave_corrida", "lev_curva_s_corrida", "lev_curva_fechada_corrida")
# (28, 05/11 01h05, ele: "vamos fazer um treino de SPRINT com a velocidade meta de 8 m/s".)
# (29, 05/11 04h40, ele: "vamos fazer seguir o clipe para o levantar"): o levantar pelo clipe, das quedas (comecos 0 e 1).
# (30-35, 08/11, ele: "6 exercicios novos, da estrela de oito pontas: trote peito livre, fixo e girando; corrida peito livre,
# fixo e girando" - as manobras 8 fixo, 9 livre, 10 girando, na faixa da coluna 25.)
STAR_KEYS = ("lev_estrela_trote_fixo", "lev_estrela_trote_livre", "lev_estrela_trote_girando",
             "lev_estrela_corrida_fixo", "lev_estrela_corrida_livre", "lev_estrela_corrida_girando")
# A duracao do episodio da estrela (s; --estrela-segundos).
STAR_SECONDS = [14.0]
# (36, 08/11 14h40: o comando - o jogador emulado, a manobra 12 - e a duracao do episodio dele, s.)
COMMAND_KEY = "lev_comando"
COMMAND_SECONDS = [20.0]
# (12/11 18h45) A duracao do episodio do parado (s; --parado-segundos, 0 = a de sempre) e a largura do caminho da estrela (m).
IDLE_SECONDS = [6.0]
STAR_PATH_WIDTH = [0.0]
DRILL_KEYS = tuple([f"lev_trote_{h}" for h in range(8)] + [f"lev_corrida_{h}" for h in range(8)] + list(MANEUVERS)
                   + ["lev_giro", "lev_parado"] + list(RUN_CURVES) + ["lev_sprint", "lev_levantar"] + list(STAR_KEYS)
                   + [COMMAND_KEY])


def star_key(row) -> str | None:
    """O exercicio da estrela de um episodio (a manobra 8-10 na coluna 26, a faixa na 25), ou None."""
    if len(row) > 26 and row[1] == 6 and 8 <= row[26] <= 10:
        return STAR_KEYS[(3 if row[25] == 2 else 0) + int(row[26]) - 8]
    return None
# (O levantar pelo clipe ligado neste treino: os episodios das quedas viram o exercicio lev_levantar.)
GETUP_CLIP = [False]


def drill_key(row) -> str | None:
    """O exercicio de um episodio (as colunas: 1 o comeco, 2 o rumo, 25 a faixa, 26 a manobra)."""
    if row[1] in (0, 1):
        return "lev_levantar" if GETUP_CLIP[0] else None
    if row[1] == 7:
        return "lev_giro"
    if row[1] == 2:
        # (04/11 15h: o equilibrio parado - o parado com empurroes, coluna 37.)
        return "lev_equilibrio" if len(row) > 37 and row[37] > 0.5 else "lev_parado"
    if row[1] != 6:
        return None
    if row[26] == 12:
        return COMMAND_KEY
    if row[26] >= 8:
        return star_key(row)
    if row[26] > 0:
        if int(row[26]) >= 5 and row[25] == 2:
            return RUN_CURVES[int(row[26]) - 5]
        return MANEUVERS[int(row[26]) - 1]
    if row[25] == 3:
        return "lev_sprint"
    return f"lev_{'corrida' if row[25] == 2 else 'trote'}_{int(row[2]) % 8}"


# (08/11, a fase de fogo) As faixas pelo nivel: a pancada seca (N, 0,1-0,2 s) e o empurrao continuo (N, 3-5 s). 0 fixos e
# pequenos (350 e 120 N, para padronizar), 1 as faixas dele (600-1400 e 200-500 N), 2 o teto (900-1800 e 280-700 N).
FIRE_HOLD_SECONDS = (3.0, 5.0)
FIRE_HOLD_FRACTION = 0.5


def fire_bands(level: float) -> tuple[tuple[float, float], tuple[float, float]]:
    # (12h30: abaixo de 0, ate -0,5 - o giro nao segura nem 120 N continuos: pancada 225 e continuo 80 N, fixos.)
    # (08/11 18h05, ele: "algo um pouco mais agressivo, do tipo que faca ele cair em 50%": o teto vai ao nivel 3 - pancada ate
    # 2400 N e empurrao continuo ate 900 N; ate o nivel ~1,4 nada muda. O mesmo no TrackingViewer3D, fireBands.)
    level = float(np.clip(level, -0.5, 3.0))
    if level < 0:
        return (350.0 + 250.0 * level,) * 2, (120.0 + 80.0 * level,) * 2
    hit = (350.0 + 250.0 * level, min(2400.0, 350.0 + 1050.0 * level))
    hold = (120.0 + 80.0 * level, min(900.0, 120.0 + 380.0 * level))
    return hit, hold


# (08/11 12h30) O fogo por grupo: a frente, os lados, as costas (os retos pelo rumo), o giro, o parado e o resto (trocas, curvas,
# estrela, o de pe depois de levantar) - cada um no nivel dele (a sonda: o giro cai 100% com 120 N continuos e 0% com pancadas
# de 350; a frente quase nao sente o nivel 0).
FIRE_GROUPS = ("frente", "lado", "costas", "giro", "parado", "outros")


def fire_group(row) -> int:
    if row[1] == 7:
        return 3
    if row[1] == 2:
        return 4
    if row[1] == 6 and row[26] <= 0:
        return slow_group(row[2])
    return 5


def fire_group_bands(levels) -> list[float]:
    out: list[float] = []
    for level in levels:
        hit, hold = fire_bands(level)
        out += [hit[0], hit[1], hold[0], hold[1]]
    return out


def fire_step(level: float, ratio: float) -> float:
    """O degrau do nivel de fogo pela razao do sucesso com empurrao sobre o sem (o mesmo treino, sorteio 50/50): +0,1 com
    >= 0,95 (o empurrao custa ate 5%), -0,1 com < 0,80, entre 0 e 2. (08/11 09h40: a primeira regua - "nao caiu ate 2 s depois
    do empurrao" >= 92% - nunca subiria: o ruido do treino e a estrela ja derrubam ~25% sem empurrao nenhum.)"""
    return float(np.clip(level + (0.1 if ratio >= 0.95 else -0.1 if ratio < 0.80 else 0.0), -0.5, 3.0))


def fire_step_target(level: float, with_push: float, target: float) -> float:
    """(08/11 18h05, ele: "faca ele cair em 50% dos exercicios... e ele tenha que aprender a se auto-equilibrar") O degrau pelo
    sucesso dos episodios COM empurrao contra o alvo: +0,1 acima de alvo + 0,05, -0,1 abaixo de alvo - 0,05, entre -0,5 e 3."""
    return float(np.clip(level + (0.1 if with_push >= target + 0.05 else -0.1 if with_push < target - 0.05 else 0.0), -0.5, 3.0))


assert fire_step_target(0.0, 0.8, 0.5) == 0.1 and fire_step_target(1.0, 0.3, 0.5) == 0.9 and fire_step_target(1.0, 0.5, 0.5) == 1.0
assert fire_bands(3.0) == ((1100.0, 2400.0), (360.0, 900.0)) and fire_bands(1.0) == ((600.0, 1400.0), (200.0, 500.0))


# (12h25) O piso da velocidade por grupo de rumo: a frente (0, 1, 7), os lados (2, 6), as costas (3, 4, 5).
SLOW_GROUPS = ("frente", "lado", "costas")


def slow_group(heading) -> int:
    h = int(heading) % 8
    return 1 if h in (2, 6) else 2 if h in (3, 4, 5) else 0


def slow_step(floor: float, passed: float, minimum: float = 0.4) -> float:
    """O degrau do piso: +0,05 com >= 90% sem o fim por lento, -0,05 com < 70%, entre o minimo (0,4) e 0,85."""
    return float(np.clip(floor + (0.05 if passed >= 0.90 else -0.05 if passed < 0.70 else 0.0), minimum, 0.85))


assert slow_group(6) == 1 and slow_group(4) == 2 and slow_group(7) == 0 and abs(slow_step(0.6, 0.95) - 0.65) < 1e-9
assert abs(slow_step(0.5, 0.5, 0.5) - 0.5) < 1e-9 and abs(slow_step(0.5, 0.5) - 0.45) < 1e-9


def fire_basic(row) -> bool:
    """Os exercicios que medem o fogo: os retos (sem manobra), o giro, o parado e o levantar - os que quase nao falham sem
    empurrao; as trocas, as curvas e a estrela falham por si."""
    return len(row) > 45 and (row[1] in (0, 1, 2, 7) or (row[1] == 6 and row[26] <= 0))


def fire_ratio(rows: np.ndarray) -> tuple[float, float, float, int, int]:
    """(razao, sucesso com empurrao, sem, n com, n sem): sucesso = acabou pelo tempo (coluna 32 = 0)."""
    if not len(rows):
        return float("nan"), float("nan"), float("nan"), 0, 0
    pushed = rows[:, 38] > 0
    # (12h25: so a queda de verdade - o chao, o equilibrio, o virado, a pose perdida -; o "lento" e o tempo nao contam.)
    # (08/11 19h10: e a queda que nao encerra - no equilibrio parado o corpo cai, paga a multa, marca "caiu de pe" (coluna 13)
    # e o levantar segue: o parado contava 100% de pe com qualquer forca e subiu ao teto, 2400 N e 900 N.)
    ok = ~np.isin(rows[:, 32], (1, 2, 3, 4)) & (rows[:, 13] <= 0)
    with_push = float(ok[pushed].mean()) if pushed.any() else float("nan")
    without = float(ok[~pushed].mean()) if (~pushed).any() else float("nan")
    return with_push / max(without, 1e-6), with_push, without, int(pushed.sum()), int((~pushed).sum())


# (12/11 16h30, ele: "descongelar o brain do RL da locomocao... com a implementacao da fase de FOGO, igual nos fizemos aqui... nesse
# primeiro momento sera algo bem mais SUTIL... a meta e que no final consigamos trazer exatamente o mesmo tipo e niveis de forca que o
# levantar ja sofre") O fogo da locomocao v1 (vault 04_locomocao/plano_fogo_da_locomocao_12_11.md): um nivel L (0 sutil - 1 o fogo
# do levantar) por grupo de FIRE_GROUPS. As colunas 60-63 dos episodios de locomocao (o comando fora - ele nunca tem fogo): se teve
# fogo, o nivel, as adversidades e a fracao do tempo seguindo a tarefa (com fogo desde a primeira adversidade; sem, desde 1,5 s).
# O sucesso dele: nao caiu. O firme (a qualidade): nao caiu e seguiu a tarefa em >= min(LOCO_FIRE_KEEP, LOCO_FIRE_RELATIVE x o seguiu
# do limpo do grupo) - a regua relativa ao limpo mede o efeito do fogo, nao a dificuldade do exercicio (16h45: nos laterais e nas
# manobras o limpo segue 50-60% e a regua absoluta reprovava o grupo inteiro).
# A escada (a do levantar): +0,1 com o firme - 1,5 EP >= LOCO_FIRE_MASTERED ha >= LOCO_FIRE_DWELL atualizacoes no nivel e o limpo do
# grupo perto da base dele; -0,1 pela guarda do limpo (o nao caiu do limpo + 1,5 EP abaixo da base do grupo - LOCO_FIRE_CLEAN_DROP; a
# base: a primeira medida com >= LOCO_FIRE_BASE_N limpos) ou travado (nao caiu + 1,5 EP < LOCO_FIRE_STUCK ha >= 2 x LOCO_FIRE_DWELL).
# (12/11 19h55, a revisao do fogo: em 2 h nenhum grupo subiu - o firme de 0,85 nao vinha nem no L 0,3 (giro/parado 0,55, os outros
# 0,7-0,95 de nao caiu, o firme abaixo). Dominado agora em 0,75.)
LOCO_FIRE_MASTERED, LOCO_FIRE_KEEP, LOCO_FIRE_RELATIVE, LOCO_FIRE_STUCK = 0.75, 0.7, 0.85, 0.6
# (12/11 23h50, a vigia: em 3,5 h no L 0,5 o firme da frente, lado, costas e outros ficou em 0,2-0,4 sem subir - o firme pede seguir o
# comando durante o empurrao (>= 85% do seguiu do limpo) e a nota, de proposito, solta a velocidade enquanto ele empurra (a liberdade):
# a escada cobrava o que a recompensa libera. Dominado agora: nao caiu >= LOCO_FIRE_MASTERED_OK e o seguir frouxo (>= LOCO_FIRE_RELATIVE_
# LOOSE do seguiu do limpo, no maximo LOCO_FIRE_KEEP_LOOSE) >= LOCO_FIRE_MASTERED_LOOSE, os dois com 1,5 EP.)
LOCO_FIRE_MASTERED_OK, LOCO_FIRE_MASTERED_LOOSE, LOCO_FIRE_KEEP_LOOSE, LOCO_FIRE_RELATIVE_LOOSE = 0.80, 0.5, 0.5, 0.6
# (12/11 19h55, ele: "reduzir a intensidade da fase de fogo da locomocao para 50%... conforme ela for melhorando e se adaptando, vamos
# aumentando gradualmente ate 100%") A versao 2 do estado recomeca os niveis no --locomocao-fogo-nivel uma vez; e a guarda da velocidade do
# limpo (a capacidade, a rede media sem fogo): a escada so sobe com o trote, a corrida e o trote de lado a LOCO_FIRE_SPEED_TOL da referencia
# (--locomocao-fogo-guarda-velocidade) - de 17h55 a 19h30, com o fogo em 90%, a corrida caiu 0,855 -> 0,75 e o trote de lado 0,64 -> 0,47
# e a guarda so via quedas.
# (13/11 05h30: com a fatia em 0,9 a corrida ficou 1 h entre 0,79 e 0,81 - 6% abaixo das 17h55 (0,842), no limite de 0,05; ele: "nao
# depreciar velocidade". Tolerancia 0,03 - o ruido da medida e ~0,01 com 144 episodios por marcha.)
LOCO_FIRE_VERSION, LOCO_FIRE_SPEED_TOL = 2, 0.03
# (20h30: com o torto limpo, 10 min de 90% de fogo ainda levaram o parado 0,61 -> 0,54, o torto 47% -> 16% e a corrida 0,83 -> 0,79 - o
# mesmo das 17h55-18h30 em L 0,3; com 35% de fogo (16h35-17h55) o limpo subia. A fatia agora e conquistada: comeca em
# --locomocao-fogo-fatia-inicial e o piso desce a 0,3.)
LOCO_FIRE_SIM_TOL, LOCO_FIRE_CROOKED_TOL, LOCO_FIRE_SHARE_MIN, LOCO_FIRE_SHARE_STEP = 0.05, 0.15, 0.3, 0.1
# (13/11 18h50) Quantas medidas boas seguidas para a fatia do fogo subir (--locomocao-fogo-fatia-subida).
LOCO_FIRE_SHARE_GOOD = 3
LOCO_FIRE_CLEAN_DROP, LOCO_FIRE_BASE_N, LOCO_FIRE_CLEAN_DECAY = 0.08, 60, 0.85
LOCO_FIRE_STEP, LOCO_FIRE_DWELL, LOCO_FIRE_MIN = 0.1, 100, 30
LOCO_FIRE_WINDOWS = [0]


def loco_fire_extras(b, args) -> None:
    """(16h40) As adversidades e as faixas que o fogo da locomocao usa (as do levantar: a serie, a rasteira, o prensado, o vento, a grama
    molhada, o agarrao, os intervalos, a duracao e os newtons da pancada) e o limite contra a corrida - no lote do treino o main as poe
    junto com o levantar; nos outros (a prova, as sondas), por aqui."""
    if hasattr(b, "set_get_up_fire_extras"):
        b.set_get_up_fire_extras(*fire_extras(args))
    if args.levantar_fogo_contra_corrida > 0 and hasattr(b, "set_fire_run_headwind_cap"):
        b.set_fire_run_headwind_cap(args.levantar_fogo_contra_corrida)


def loco_fire_ok(rows: np.ndarray) -> np.ndarray:
    """Nao caiu: nem a queda que encerra (as causas 1-4) nem a que nao encerra (a coluna 13)."""
    return ~np.isin(rows[:, 32], (1, 2, 3, 4)) & (rows[:, 13] <= 0)


def loco_fire_groups(rows: np.ndarray) -> np.ndarray:
    return np.array([fire_group(r) for r in rows], dtype=int) if len(rows) else np.zeros(0, dtype=int)


def loco_fire_state(args, counters: dict) -> dict | None:
    if args.locomocao_fogo <= 0:
        return None
    state = counters.get("fogo_locomocao")
    if state is None:
        state = {"niveis": [float(args.locomocao_fogo_nivel)] * len(FIRE_GROUPS),
                 "desde": [int(counters["atualizacao"])] * len(FIRE_GROUPS), "limpo_base": [None] * len(FIRE_GROUPS), "versao": 1}
        counters["fogo_locomocao"] = state
        print(f"fogo da locomocao: niveis de partida {state['niveis']}", flush=True)
    state.setdefault("limpo_base", [None] * len(FIRE_GROUPS))
    if int(state.get("versao", 1)) < LOCO_FIRE_VERSION:
        state["niveis"] = [float(args.locomocao_fogo_nivel)] * len(FIRE_GROUPS)
        state["desde"] = [int(counters["atualizacao"])] * len(FIRE_GROUPS)
        state["versao"] = LOCO_FIRE_VERSION
        print(f"fogo da locomocao: versao {LOCO_FIRE_VERSION}, os niveis recomecam em {state['niveis']}", flush=True)
    lo, hi = float(args.locomocao_fogo_faixa[0]), float(args.locomocao_fogo_faixa[1])
    # (12/11 22h35) O piso proprio de um grupo (--locomocao-fogo-piso-grupo): abaixo da faixa geral, so para ele.
    floors = {name: float(value) for name, value in (args.locomocao_fogo_piso_grupo or [])}
    state["pisos"] = floors
    for g, name in enumerate(FIRE_GROUPS):
        level = min(hi, max(min(lo, floors.get(name, lo)), float(state["niveis"][g])))
        if abs(level - float(state["niveis"][g])) > 1e-9:
            print(f"fogo da locomocao: {name} {state['niveis'][g]} -> {level} (a faixa {lo}-{hi})", flush=True)
            state["niveis"][g] = round(level, 3)
            state["desde"][g] = int(counters["atualizacao"])
    state["faixa"] = [lo, hi]
    return state


def loco_fire_split(rows: np.ndarray):
    """(os grupos, nao caiu, a mascara do fogo e a do limpo) dos episodios de locomocao elegiveis (parado, rumos sem o comando, giro)."""
    # (12/11 18h: o comando entrou no fogo - ele e do grupo "outros".)
    loco = np.isin(rows[:, 1], (2, 6, 7))
    return (loco_fire_groups(rows), loco_fire_ok(rows), loco & (rows[:, 60] > 0) & (rows[:, 62] > 0),
            loco & (rows[:, 60] <= 0) & (rows[:, 63] >= 0))


def loco_fire_threshold(rows: np.ndarray, clean: np.ndarray) -> float:
    keep = rows[clean, 63]
    return min(LOCO_FIRE_KEEP, LOCO_FIRE_RELATIVE * float(keep.mean())) if len(keep) >= LOCO_FIRE_MIN else LOCO_FIRE_KEEP


def loco_fire_summary(rows: np.ndarray) -> dict:
    """Por grupo: o limpo (n, nao caiu, seguiu) e o fogo (n, nao caiu, firme, seguiu, o nivel medio e o nao caiu por adversidade)."""
    out: dict = {}
    if not len(rows) or rows.shape[1] < 64:
        return out
    groups, ok, fire, clean = loco_fire_split(rows)
    for g, name in enumerate(FIRE_GROUPS):
        f, c = fire & (groups == g), clean & (groups == g)
        entry = {"n_limpo": int(c.sum()), "limpo": round(float(ok[c].mean()), 3) if c.any() else None,
                 "limpo_seguiu": round(float(rows[c, 63].mean()), 3) if c.any() else None, "n_fogo": int(f.sum())}
        if f.any():
            keep = rows[f, 63]
            ck = rows[c, 63]
            loose_bar = (min(LOCO_FIRE_KEEP_LOOSE, LOCO_FIRE_RELATIVE_LOOSE * float(ck.mean())) if len(ck) >= LOCO_FIRE_MIN
                         else LOCO_FIRE_KEEP_LOOSE)
            entry.update(sem_cair=round(float(ok[f].mean()), 3),
                         frouxo=round(float((ok[f] & (keep >= loose_bar)).mean()), 3),
                         firme=round(float((ok[f] & (keep >= loco_fire_threshold(rows, c))).mean()), 3),
                         seguiu=round(float(keep[keep >= 0].mean()), 3) if (keep >= 0).any() else None,
                         nivel=round(float(rows[f, 61].mean()), 3))
            bits = rows[f, 59].astype(int)
            for bit, key in ((1, "serie"), (2, "rasteira"), (4, "prensado"), (8, "vento"), (32, "molhado"), (64, "agarrao")):
                m = (bits & bit) > 0
                if m.any():
                    entry[key] = round(float(ok[f][m].mean()), 3)
        out[name] = entry
    return out


def loco_fire_clean(capacity: dict) -> dict | None:
    """O limpo da capacidade a 0% (a rede media, sem fogo): a velocidade sobre a pedida e a semelhanca com o clipe do trote, da corrida
    (rumos 0,1,3,4,5,7) e do trote de lado (2,6), a semelhanca do parado e o reajuste do parado torto."""
    c = (capacity or {}).get("0") or {}

    def mean(keys, field):
        v = [c[k][field] for k in keys if isinstance(c.get(k), dict) and c[k].get(field) is not None]
        return float(np.mean(v)) if v else None
    tro, cor, lat = [f"lev_trote_{d}" for d in "013457"], [f"lev_corrida_{d}" for d in "013457"], ["lev_trote_2", "lev_trote_6"]
    out = {"trote": mean(tro, "vel"), "corrida": mean(cor, "vel"), "lado": mean(lat, "vel"), "trote_sem": mean(tro, "sem"),
           "corrida_sem": mean(cor, "sem"), "lado_sem": mean(lat, "sem"), "parado_sem": mean(["parado_limpo"], "sem"),
           "torto": mean(["parado_torto"], "reajustou"), "torto_sem": mean(["parado_torto"], "sem")}
    return out if None not in (out["trote"], out["corrida"], out["lado"]) else None


def loco_fire_clean_ok(now: dict | None, base: dict | None, reference) -> tuple[bool, str]:
    """A guarda do limpo (12/11 20h05, ele: "nao depreciar velocidade e postura (semelhanca com o clipe nos exercicios limpos)"): a
    velocidade a ate LOCO_FIRE_SPEED_TOL da referencia (--locomocao-fogo-guarda-velocidade), a semelhanca a ate LOCO_FIRE_SIM_TOL da base
    (a primeira medida depois de 12/11 20h, o ponto das 17h55) e o reajuste do torto a ate LOCO_FIRE_CROOKED_TOL dela. Sem referencia
    (zeros), passa; com ela e sem medida ainda, segura."""
    if not any(r > 0 for r in reference):
        return True, ""
    if not now:
        return False, "sem medida da capacidade ainda"
    low = [f"{k} {now[k]:.2f}<{r - LOCO_FIRE_SPEED_TOL:.2f}" for k, r in zip(("trote", "corrida", "lado"), reference)
           if r > 0 and now.get(k) is not None and now[k] < r - LOCO_FIRE_SPEED_TOL]
    # (20h15, a calibracao: na marcha a semelhanca quase nao separa a rede boa da estragada - trote 0,174 contra 0,164 -, fica como rede
    # de seguranca relativa; no parado separa - 0,61 contra 0,43 - e o reajuste do torto 37% contra 0%.)
    # (20h45: o lado_sem de 0,05 contra 0,06 segurou a fatia com o lado mais rapido - 15% de 0,07 e ruido; 30%.)
    # (21h15: o lado_sem fora - o direito (rumo 6) fica em 0,01-0,03 desde sempre e o esquerdo oscila 0,06-0,25 entre medidas de 24.)
    # (21h55: o reajuste do torto e um limiar - pontos vizinhos da mesma rede deram 0,33, 0,26 e 0,37 com 96 mundos -: a guarda usa a media
    # das 3 ultimas medidas (o chamador poe em now["torto"]) e a semelhanca do torto, continua e estavel (0,551 -> 0,553-0,57).)
    for k, tol in (("trote_sem", 0.3), ("corrida_sem", 0.3), ("parado_sem", None), ("torto_sem", None), ("torto", None)):
        if not (base and base.get(k) is not None and now.get(k) is not None):
            continue
        floor = base[k] * (1.0 - tol) if tol is not None else base[k] - (LOCO_FIRE_CROOKED_TOL if k == "torto" else LOCO_FIRE_SIM_TOL)
        if now[k] < floor:
            low.append(f"{k} {now[k]:.2f}<{floor:.2f}")
    return not low, ", ".join(low)


def loco_fire_protect(prot: dict, ok: bool, top: float) -> str:
    """(12/11 20h05; 20h15 a cada medida ruim, o parado se desfez em 15 min) A protecao do limpo pela fatia do fogo: medida abaixo da guarda, a fatia desce LOCO_FIRE_SHARE_STEP
    (ate LOCO_FIRE_SHARE_MIN); tres seguidas boas, volta a subir ate a da receita (top). Devolve a mudanca em texto."""
    prot["fatia"] = min(float(prot.get("fatia", top)), top)
    prot["ruins"], prot["boas"] = (0, prot.get("boas", 0) + 1) if ok else (prot.get("ruins", 0) + 1, 0)
    if prot["ruins"] >= 1 and prot["fatia"] > LOCO_FIRE_SHARE_MIN + 1e-9:
        # (13/11 v1, a revisao: o degrau acelerado - --locomocao-fogo-aceleracao - so sobe; descer segue de 0,1, a de sempre.)
        prot["fatia"], prot["ruins"] = round(max(LOCO_FIRE_SHARE_MIN, prot["fatia"] - min(LOCO_FIRE_SHARE_STEP, 0.1)), 3), 0
        return f"a fatia do fogo desce para {prot['fatia']} (o limpo abaixo da guarda)"
    if prot["boas"] >= LOCO_FIRE_SHARE_GOOD and prot["fatia"] < top - 1e-9:
        prot["fatia"], prot["boas"] = round(min(top, prot["fatia"] + LOCO_FIRE_SHARE_STEP), 3), 0
        return f"a fatia do fogo volta para {prot['fatia']} (o limpo bom tres vezes)"
    return ""


def loco_fire_step(state: dict, rows: np.ndarray, update: int, frozen: bool = False, speed_ok: bool = True) -> list:
    """O degrau de cada grupo (ver LOCO_FIRE_MASTERED), dentro da faixa do estado (state["faixa"]); devolve as mudancas em texto.
    Com speed_ok falso (a guarda da velocidade do limpo), nao sobe."""
    changes = []
    lo_all, hi = state.get("faixa", [0.0, 1.0])
    if not len(rows) or rows.shape[1] < 64:
        return changes
    groups, ok, fire, clean_rows = loco_fire_split(rows)
    for g, name in enumerate(FIRE_GROUPS):
        lo = min(lo_all, float(state.get("pisos", {}).get(name, lo_all)))
        f, c = fire & (groups == g), clean_rows & (groups == g)
        level, since = float(state["niveis"][g]), update - int(state["desde"][g])
        # (12/11 18h, ele: so 10% limpos - poucos por janela: o limpo de cada grupo acumulado com decaimento LOCO_FIRE_CLEAN_DECAY por
        # janela, o nao caiu e o seguiu.)
        acc = state.setdefault("limpo_acumulado", [[0.0, 0.0, 0.0] for _ in FIRE_GROUPS])[g]
        acc[0] = LOCO_FIRE_CLEAN_DECAY * acc[0] + float(c.sum())
        acc[1] = LOCO_FIRE_CLEAN_DECAY * acc[1] + float(ok[c].sum())
        acc[2] = LOCO_FIRE_CLEAN_DECAY * acc[2] + float(rows[c, 63].sum())
        nc = int(acc[0])
        clean = acc[1] / acc[0] if acc[0] >= LOCO_FIRE_MIN else float("nan")
        clean_keep = acc[2] / acc[0] if acc[0] >= LOCO_FIRE_MIN else float("nan")
        if state["limpo_base"][g] is None and nc >= LOCO_FIRE_BASE_N:
            state["limpo_base"][g] = round(clean, 3)
            changes.append(f"{name}: a base do limpo e {clean:.3f} (n {nc})")
        base = state["limpo_base"][g]
        se_c = math.sqrt(max(clean * (1 - clean), 1e-6) / max(nc, 1)) if clean == clean else 0.0
        guard = base is not None and clean == clean and clean + 1.5 * se_c < base - LOCO_FIRE_CLEAN_DROP
        new, why = level, ""
        if guard and level > lo + 1e-9:
            new, why = max(lo, level - LOCO_FIRE_STEP), f"guarda do limpo ({clean:.2f} contra a base {base:.2f})"
        elif f.sum() >= LOCO_FIRE_MIN:
            n = int(f.sum())
            standing = float(ok[f].mean())
            threshold = min(LOCO_FIRE_KEEP, LOCO_FIRE_RELATIVE * clean_keep) if clean_keep == clean_keep else LOCO_FIRE_KEEP
            firm = float((ok[f] & (rows[f, 63] >= threshold)).mean())
            loose_bar = min(LOCO_FIRE_KEEP_LOOSE, LOCO_FIRE_RELATIVE_LOOSE * clean_keep) if clean_keep == clean_keep else LOCO_FIRE_KEEP_LOOSE
            loose = float((ok[f] & (rows[f, 63] >= loose_bar)).mean())
            se_s, se_f = math.sqrt(max(standing * (1 - standing), 1e-6) / n), math.sqrt(max(firm * (1 - firm), 1e-6) / n)
            se_l = math.sqrt(max(loose * (1 - loose), 1e-6) / n)
            clean_ok = base is None or (clean == clean and clean >= base - 0.5 * LOCO_FIRE_CLEAN_DROP)
            if standing + 1.5 * se_s < LOCO_FIRE_STUCK and since >= 2 * LOCO_FIRE_DWELL and level > lo + 1e-9:
                new, why = max(lo, level - LOCO_FIRE_STEP), f"travado (nao caiu {standing:.2f}, n {n})"
            elif (not frozen and speed_ok and not guard and since >= LOCO_FIRE_DWELL and standing - 1.5 * se_s >= LOCO_FIRE_MASTERED_OK
                  and loose - 1.5 * se_l >= LOCO_FIRE_MASTERED_LOOSE and level < hi - 1e-9 and clean_ok):
                new, why = min(hi, level + LOCO_FIRE_STEP), (f"dominado (nao caiu {standing:.2f}, seguiu frouxo {loose:.2f}, firme {firm:.2f}, "
                                                              f"n {n}, limpo {clean:.2f})")
        if abs(new - level) > 1e-9:
            state["niveis"][g] = round(new, 3)
            state["desde"][g] = int(update)
            changes.append(f"{name} {level:.2f} -> {new:.2f}: {why}")
    return changes


def _loco_fire_selfcheck() -> None:
    rows = np.zeros((200, 64), dtype=np.float32)
    rows[:, 1] = 2  # parado
    rows[:100, 60] = 1
    rows[:100, 62] = 2
    rows[:, 63] = 0.9
    state = {"niveis": [0.0] * 6, "desde": [0] * 6, "limpo_base": [None] * 6}
    held = loco_fire_step(dict(state, limpo_base=[None] * 6, niveis=[0.0] * 6), rows, 500, speed_ok=False)
    assert not any("->" in c for c in held), held
    changes = loco_fire_step(state, rows, 500)
    assert state["limpo_base"][4] == 1.0 and abs(state["niveis"][4] - 0.1) < 1e-9, (changes, state)
    cap = {"0": {**{f"lev_trote_{d}": {"vel": 0.9, "sem": 0.8} for d in "01234567"},
                 **{f"lev_corrida_{d}": {"vel": 0.7, "sem": 0.7} for d in "01234567"},
                 "parado_limpo": {"sem": 0.9}, "parado_torto": {"reajustou": 0.4}}}
    sp = loco_fire_clean(cap)
    assert abs(sp["corrida"] - 0.7) < 1e-9 and abs(sp["torto"] - 0.4) < 1e-9 and loco_fire_clean_ok(sp, sp, (0, 0, 0))[0], sp
    assert not loco_fire_clean_ok(sp, sp, (0.95, 0.8, 0.6))[0] and loco_fire_clean_ok(sp, sp, (0.92, 0.72, 0.6))[0]
    worse = dict(sp, parado_sem=0.8)
    assert not loco_fire_clean_ok(worse, sp, (0.92, 0.72, 0.6))[0] and not loco_fire_clean_ok(None, sp, (0.92, 0.72, 0.6))[0]
    prot = {"fatia": 0.9}
    assert "0.8" in loco_fire_protect(prot, False, 0.9) and prot["fatia"] == 0.8
    assert loco_fire_protect(prot, True, 0.9) == "" == loco_fire_protect(prot, True, 0.9) and "0.9" in loco_fire_protect(prot, True, 0.9)
    rows[100:, 32] = 1  # o limpo cai todo: a guarda desce
    changes = loco_fire_step(state, rows, 520)
    assert abs(state["niveis"][4] - 0.0) < 1e-9, (changes, state)


_loco_fire_selfcheck()


def star_phase_step(phase: int, end: float, legs: float) -> int:
    """A fase de um exercicio da estrela: sobe com >= 80% ate o fim e >= 3 pernas por episodio, desce com < 45% ate o fim."""
    if end >= 0.80 and legs >= 3.0 and phase < 4:
        return phase + 1
    if end < 0.45 and phase > 1:
        return phase - 1
    return phase


assert fire_bands(0.0) == ((350.0, 350.0), (120.0, 120.0)) and fire_bands(1.0) == ((600.0, 1400.0), (200.0, 500.0))
assert fire_step(1.0, 0.97) == 1.1 and abs(fire_step(0.0, 0.5) + 0.1) < 1e-9 and fire_step(-0.5, 0.1) == -0.5
assert star_phase_step(1, 0.9, 3.5) == 2
assert abs(fire_ratio(np.array([[0] * 38 + [1] + [0] * 7, [0] * 32 + [1] + [0] * 5 + [1] + [0] * 7, [0] * 46]))[0] - 0.5) < 1e-9
assert star_phase_step(4, 0.9, 5.0) == 4 and star_phase_step(2, 0.3, 1.0) == 1


def drill_performance(rows: np.ndarray, seconds: float) -> dict:
    """O desempenho de um exercicio nos episodios: ate o fim (sem cair, o tempo todo), a velocidade no rumo sobre a pedida
    (no giro, 1), a semelhanca com o clipe e a estabilidade; P = 0,40 fim + 0,25 velocidade + 0,20 semelhanca + 0,15
    estabilidade."""
    # (04/11 22h55, ele aprovou: "faca essas correcoes imediatamente" - a regua do fim sem os episodios que acabam no primeiro
    # START_GRACE s: de pe, na forca de cada exercicio, a rede chega ao fim 100% (com e sem o ruido); a queda logo no
    # comeco vem do comeco sorteado do treino, nao do andar. O parado tem o episodio dele, curto: fica como era.)
    if rows.shape[1] > 26 and rows[0, 1] == 6 and rows[0, 26] == 12:
        seconds = COMMAND_SECONDS[0]
    elif rows.shape[1] > 26 and rows[0, 1] == 6 and rows[0, 26] >= 8:
        seconds = STAR_SECONDS[0]
    if rows[0, 1] != 2 and rows.shape[0] > 0:
        late = rows[:, 4] >= START_GRACE
        if late.any():
            rows = rows[late]
    # (O parado: ate o fim e terminar de pe sem ter caido - o episodio dele e o curto, nao o da locomocao.)
    # (07h40: no parado, o fim pela pose perdida - a coluna 32 - tambem nao chega ao fim.)
    # (05/11: o levantar pelo clipe - ficou de pe, termina de pe e nao caiu depois.)
    # (05h20: num episodio com empurrao depois de ficar de pe - 1,35 por episodio no treino -, a queda e do equilibrio sob
    # empurrao, outro exercicio: ali conta levantar.)
    end = float(((rows[:, 5] >= 0) & (((rows[:, 38] > 0) if rows.shape[1] > 38 else False)
                                      | ((rows[:, 9] > 0.5) & (rows[:, 13] < 0.5)))).mean()) if rows[0, 1] in (0, 1) else \
        float(((rows[:, 9] > 0.5) & (rows[:, 13] < 0.5) & ((rows[:, 32] == 0) if rows.shape[1] > 32 else True)).mean()) \
        if rows[0, 1] == 2 else \
        float(((rows[:, 4] >= seconds - 0.05) & (rows[:, 13] < 0.5)).mean())
    speed = 1.0 if rows[0, 1] in (0, 1, 2, 7) else float(np.clip(rows[:, 12] / np.maximum(rows[:, 10], 0.1), 0.0, 1.0).mean())
    tracked = rows[rows[:, 33] >= 0] if rows.shape[1] > 34 else rows[:0]
    imitation = float(tracked[:, 33].mean()) if len(tracked) else 0.0
    # (05/11: a estabilidade so dos episodios que a mediram - no levantar pelo clipe, os que ficaram de pe.)
    firm = tracked[tracked[:, 34] >= 0] if len(tracked) else tracked
    stability = float(firm[:, 34].mean()) if len(firm) else 0.0
    # (04/11 04h30, ele: "para baixar a forca auxiliar exigir mais da semelhanca com o clipe": a semelhanca 0,20 -> 0,35.)
    out = {"fim": end, "vel": speed, "semelhanca": imitation, "estabilidade": stability,
           "P": 0.30 * end + 0.20 * speed + 0.35 * imitation + 0.15 * stability}
    if rows[0, 1] == 6 and rows.shape[1] > 31:
        out["desvio"] = float(np.abs(rows[:, 31]).mean())
    if rows.shape[1] > 42 and (rows[:, 42] >= 0).any():
        out["cruzado"] = float(rows[rows[:, 42] >= 0, 42].mean())
    if rows[0, 1] in (0, 1):
        # (O levantar: o tempo medio ate ficar de pe, nos que ficaram.)
        stood = rows[rows[:, 5] >= 0, 5]
        out["tempo"] = float(stood.mean()) if len(stood) else 99.0
    return out


# (22h55) Os episodios que acabam antes disto (s) nao contam na regua da forca (drill_performance).
START_GRACE = 1.0
# A velocidade minima no rumo, como fracao do pedido, para descer a forca (04/11 07h45, ele: "no minimo 80% do pedido").
SPEED_REQUIRED = 0.80
# (04/11 13h50, ele: "ele nao cobra estabilidade para diminuir a forca auxiliar.. deveria cobrar"): a estabilidade minima.
# (14h35: absoluta - o teto do clipe puro com a mao cheia ficou contaminado pela mao: arrastado mais rapido que a passada
# dele, o clipe escorrega os pes 56-66% do tempo de lado e o "teto" la e 0,85-0,88; 90% dele deixava passar a corrida
# direita com o pe arrastando. Medido na v12 a ~20% de forca: limpos 0,88-0,92; corrida de lado 0,81-0,84 e trote de costas
# 0,83 ficam presos.)
STABILITY_REQUIRED = 0.85


def required_stability(level: float, key: str = "", ceiling: float | None = None) -> float:
    """A estabilidade exigida para descer a forca (04/11 16h20, ele: "linear: com 100% de forca auxiliar exige-se 85%, e com
    0% no minimo 50%")."""
    # (05/11 04h50, o levantar pelo clipe: a firmeza de pe depois dele - no idle, sem mao em nenhuma forca -, subindo conforme
    # a mao do levantar sai: 0,30 com 100% (a rede faz 0,29-0,35; o clipe puro 0,22) ate os 50% do 0%.)
    if key == "lev_levantar":
        return 0.30 + 0.20 * (1.0 - float(np.clip(level, 0.0, 1.0)))
    need = 0.50 + (STABILITY_REQUIRED - 0.50) * float(np.clip(level, 0.0, 1.0))
    # (04/11 18h40: nas trocas nem o clipe puro com a mao cheia chega - a emenda dos clipes de rumo escorrega o pe da
    # referencia, p90 ~2 m/s; firmeza do clipe 0,55-0,67 contra 0,62-0,67 exigidos: nao exigir mais que 95% do clipe,
    # teto_firmeza.json.)
    # (05/11 01h35: e no sprint - o clipe puro com a mao cheia faz 0,82 no episodio, abaixo dos 0,85 da forca 100%.)
    # (05/11 02h35, o reset para 100%: a rede de agora com a forca cheia ficava abaixo dos 85% em 22 dos 29 exercicios - a mao
    # cheia arrasta o corpo mais rapido que a passada e os pes escorregam; o clipe puro faz 0,33-0,64 nos laterais, nas
    # diagonais e nas curvas. Em todos, entao: nao mais que 95% do clipe puro com a mao cheia (teto_estabilidade.json, a mesma
    # medida do portao), nem menos que os 50% do 0%.)
    if ceiling:
        need = min(need, max(0.50, 0.95 * ceiling))
    return need


def compensation(similarity: float, similarity_required: float, stability: float, stability_required: float) -> float:
    """O saldo da semelhanca com a sobra da estabilidade (pontos absolutos, 0-1; 05/11 20h35, ele): >= 0 passa. A falta de
    estabilidade nao entra (ela tem a exigencia dela) e a sobra de semelhanca nao cobre a estabilidade."""
    return (similarity - similarity_required) + max(0.0, stability - stability_required)


def dynamic_step(level: float) -> float:
    """O maior passo de descida da forca dinamica (05/11 02h30, ele: "nao vamos mais permitir ele reduzir de 5 em 5.. de 3 em
    3.. no maximo de 1 em 1 e abaixo de 30% de 0.5 em 0.5"): 1% acima de 30%, 0,5% de 30% para baixo. (Antes, 04/11: 5%
    acima de 50%, 3% ate 30%, 1% ate 5% e 0,5% abaixo.)"""
    return 0.01 if level > 0.30 + 1e-6 else 0.005


def required_similarity(level: float, key: str = "") -> float:
    """A semelhanca exigida para descer a forca, como fracao do teto do exercicio (04/11 06h25, ele: "baixar a regua para
    algo mais aceitavel: com 100% de forca algo tipo 80%, e no 0% coisa de 60% no minimo", linear)."""
    # (04/11 06h50, ele: flexibilizar - 70% com 100% de forca, linear ate 50% com 0%; antes 80% -> 60%.)
    level = float(np.clip(level, 0.0, 1.0))
    # (04/11 07h05, ele: nas trocas de direcao a regua mais baixa - o movimento ele ja aprende nos outros treinos; ali o foco
    # e nao cair e a estabilidade: 60% com 100% de forca, linear ate 35% com 0%.)
    if key.startswith("lev_troca"):
        return 0.35 + 0.25 * level
    # (04/11 07h40, ele: "o parado... independente da forca auxiliar precisa ter a exigencia de semelhanca com o clipe muito
    # mais alta, obrigatoriamente": 90% do teto com qualquer forca.)
    # (07h30: absoluta - o teto do parado, o clipe puro 0,815, fica abaixo do que a rede faz, 0,88: 90% dele era 0,73.)
    if key == "lev_parado":
        return 0.85
    # (05/11 19h50, ele: "baixe o criterio de semelhanca com o clipe dos treinos de trote linearmente onde no 0% de forca
    # auxiliar a exigencia e 40%": nos trotes, 40% do teto com 0% de forca, linear ate os 70% de 100%; sem o piso absoluto.)
    if key.startswith("lev_trote_"):
        return 0.40 + 0.30 * level
    # (05/11 04h50, ele: "melhorar os criterios do levantar... estabilidade, o quanto segue o clipe": a semelhanca absoluta,
    # com qualquer forca - a rede a 100% faz 0,51-0,54; o clipe puro com a mao, 0,38, nao serve de teto.)
    if key == "lev_levantar":
        return GETUP_SIMILARITY
    # (04/11 08h30, ele: "exigir a similaridade com clipe em 70% no minimo para forcas auxiliares abaixo de 50%
    # obrigatoriamente.. menos que isso ele se move muito estranho.. exceto os exercicios de troca de direcao": abaixo de 50%
    # de forca, 70% do teto; de 50% a 100%, a regua de antes, 60% a 70%.)
    if level <= 0.5 + 1e-6:
        return 0.70
    return 0.50 + 0.20 * level


# (05/11 04h50) O levantar pelo clipe: a semelhanca minima (absoluta) e o tempo medio maximo ate ficar de pe (s; o clipe
# no tempo dele leva ~2,4 s - acima disso, se arrastando).
GETUP_SIMILARITY, GETUP_SECONDS = 0.48, 3.0
# (05/11 06h, ele: "andando torto para a direita... no trote, nas diagonais, na corrida, ate o sprint": descer pede o desvio
# medio de rumo do deslocamento (graus, a coluna 31) ate isto - nos rumos e no sprint; as trocas e as curvas mudam de rumo.)
# (06h25, ele: "reforce isso ainda mais": 4 -> 3 graus.)
COURSE_DEGREES = 3.0
# (05/11 06h, ele: "estamos cobrando pouca diferenca com o clipe nos exercicios de trote; aumentar a regua de forma geral, no
# minimo 70% de semelhanca": a semelhanca absoluta minima - sem passar de 95% do proprio clipe; as trocas ficam na delas.)
SIMILARITY_FLOOR = 0.70


def similarity_required(level: float, key: str, ceiling: float) -> float:
    """A semelhanca exigida como fracao do teto, com o piso absoluto SIMILARITY_FLOOR (fora das trocas)."""
    need = required_similarity(level, key)
    if not key.startswith(("lev_troca", "lev_trote_")) and key not in ("lev_parado", "lev_levantar") and ceiling > 0.0:
        need = max(need, min(SIMILARITY_FLOOR / ceiling, 0.95))
    return need


# (05/11 23h, ele: "e ele AINDA esta cruzando as pegadas na corrida": o termo cobra a semelhanca, mas a sobra da estabilidade
# a compensa e a forca da corrida 0 desceu de 21 a 13% cruzando.) Na corrida para a frente e nas diagonais da frente, descer
# pede no maximo isto de apoios com o pe cruzado (a coluna 42: o pe apoiado mais de 3 cm para dentro que o do clipe). Medido
# no simulador na r31850 (o episodio inteiro, com a largada): a corrida 0 51-63%, as diagonais 2-21%, o clipe puro 7-10%.
CROSS_LIMIT = 0.20
CROSS_DRILLS = ("lev_corrida_0", "lev_corrida_1", "lev_corrida_7")


# (06/11 01h10, ele: "a troca de direcoes ainda esta muito fragil... pense bem o que fazer para ajudar ele a aprender melhor")
# A pratica das trocas (--troca-pratica): o rumo de partida das trocas 1-3 sorteado metade por igual e metade pela taxa de
# quedas recente de cada rumo (com o suavizado de Laplace), dos ultimos TROCA_WINDOW episodios de troca.
TROCA_WINDOW = 1500


def troca_heading_weights(window) -> list[float]:
    """Os 8 pesos do rumo de partida das trocas: (rumo, caiu) dos episodios recentes -> 0,5/8 + 0,5 * taxa / soma."""
    n = np.zeros(8)
    f = np.zeros(8)
    for h, fell in window:
        n[int(h) % 8] += 1
        f[int(h) % 8] += bool(fell)
    rate = (f + 1.0) / (n + 10.0)
    return [float(x) for x in 0.5 / 8 + 0.5 * rate / rate.sum()]


def course_drill(key: str) -> bool:
    # (06h12: nas curvas a coluna 31 mede contra o rumo do comeco, nao contra a curva - 41-77 graus; ficam de fora.)
    return key.startswith(("lev_trote_", "lev_corrida_")) or key == "lev_sprint"
# (Subir a forca com menos de 85% ate o fim, 04/11 11h05.)
FALL_RISE = 0.85


def required_finish(level: float) -> float:
    """O ate o fim (sem quedas) exigido para descer a forca (05/11 02h30, ele: "com 100% de forca auxiliar exige-se 100% de
    nao quedas e linearmente ate 0% de forca auxiliar exige-se 90%"; antes, 04/11 12h35, 100% com qualquer forca)."""
    return 0.90 + 0.10 * float(np.clip(level, 0.0, 1.0))


def gait_speeds(values) -> list:
    """Os 8 pedidos fixos de uma faixa (--velocidade-trote/--velocidade-corrida): 1 valor vale para todos."""
    values = [float(v) for v in values]
    if len(values) == 1:
        return values * 8
    if len(values) != 8:
        raise SystemExit("--velocidade-trote/--velocidade-corrida: 1 ou 8 valores")
    return values


def run_tracking_settings(b, v2: dict) -> None:
    """As configuracoes do rastreio de um treino (o v2 do run.json) num ambiente das ferramentas (os tetos e as sondas)."""
    b.set_lateral_hand_push(not v2.get("trote_lateral_sem_puxao", False))
    if hasattr(b, "set_phase_sync"):
        b.set_phase_sync(bool(v2.get("sincronizar_fase", False)))
    # (17/11, Fase 1 - build-base-n131) Os consertos e a paridade com o jogo, como no treino: a fase nos dois pes e a forca dos contatos
    # mudam o que a rede ve; o chao de grama, o que ela pisa. MATTER_CHAO_GRAMA troca a fatia para medir num chao so (0 ou 1).
    if hasattr(b, "set_grass_ground_fraction"):
        rumo = v2.get("rumo_medio") or [0.0]
        b.set_reference_side_fix(bool(v2.get("referencial_clipe", False)))
        b.set_phase_sync_both_feet(bool(v2.get("fase_dois_pes", False)))
        b.set_heading_bias(float(rumo[0]), float(rumo[1]) if len(rumo) > 1 else 0.10)
        b.set_contact_force_observation(bool(v2.get("forca_contatos", False)))
        b.set_grass_ground_fraction(float(os.environ.get("MATTER_CHAO_GRAMA", v2.get("chao_grama", 0.0))))
        b.set_straight_command(bool(v2.get("reta_comando", False)))
        if hasattr(b, "set_clip_velocity_rotation"):
            b.set_clip_velocity_rotation(bool(v2.get("velocidade_rotacao", False)))
        lado, portao = v2.get("lado_medio") or [0.0], v2.get("imitacao_portao") or [0.0]
        b.set_lateral_bias(float(lado[0]), float(lado[1]) if len(lado) > 1 else 0.03)
        b.set_imitation_gate(float(portao[0]), float(portao[1]) if len(portao) > 1 else 0.6)
    if hasattr(b, "set_arm_clip_reach"):
        b.set_arm_clip_reach(float(v2.get("bracos_clipe", -1.0)))
    if hasattr(b, "set_gait_cycles"):
        b.set_gait_cycles(bool(v2.get("passos_ciclos", False)))
    if hasattr(b, "set_head_clip_reach"):
        b.set_head_clip_reach(1.0 if v2.get("cabeca_autoridade") else float(v2.get("cabeca_clipe", -1.0)))
    b.set_lateral_run_leg_reach(float(v2.get("corrida_lateral_pernas", 1.0)))
    b.set_gait_speeds(gait_speeds(v2.get("velocidades_trote", [0.0])), gait_speeds(v2.get("velocidades_corrida", [0.0])))
    b.set_back_start_lean(-np.radians(float(v2.get("largada_costas", 0.0))))
    b.set_course_limit(float(v2.get("curva_fora", 0.0)))
    b.set_curve_lean(bool(v2.get("curva_inclinar", False)))
    if hasattr(b, "set_locomotion_shaping"):
        b.set_locomotion_shaping(not bool(v2.get("sem_potencial_loco", False)))
        b.set_speed_gate(float(v2.get("nota_velocidade", 0.0)))
        b.set_knee_inward(float(v2.get("joelho_dentro", 0.0)))
        b.set_idle_height(float(v2.get("idle_altura", 0.0)))
        b.set_stop_wait_body(bool(v2.get("parada_corpo", False)))
    if hasattr(b, "set_foot_width"):
        b.set_foot_width(float(v2.get("pisada_largura", 0.0)))
    if hasattr(b, "set_run_turn_brake"):
        b.set_run_turn_brake(bool(v2.get("troca_freio", False)))
        b.set_run_turn_align(bool(v2.get("troca_pe", False)))
        if hasattr(b, "set_turn_settle"):
            b.set_turn_settle(bool(v2.get("troca_assentada", False)))
            b.set_turn_launch(bool(v2.get("troca_arrancada", False)))
        if hasattr(b, "set_run_launch_ease"):
            b.set_run_launch_ease(*(v2.get("corrida_largada") or (0.0, 0.0, 0.0)))
            b.set_course_tail(float(v2.get("rumo_cauda", 0.0)))
            b.set_roll_tail(float(v2.get("inclinacao_cauda", 0.0)))
            if hasattr(b, "set_cliff_free"):
                b.set_cliff_free(bool(v2.get("sem_penhascos", False)))
                if hasattr(b, "set_launch_rise"):
                    b.set_launch_rise(float(v2.get("arrancada_pulo", 0.0)))
                if hasattr(b, "set_lateral_start"):
                    b.set_lateral_start(float(v2.get("largada_lado", 0.0)))
                if hasattr(b, "set_turn_settle_lateral"):
                    b.set_turn_settle_lateral(float(v2.get("troca_assentada_lado", 0.0)))
                if hasattr(b, "set_turn_settle_front"):
                    b.set_turn_settle_front(float(v2.get("troca_assentada_frente", 0.0)))
                if hasattr(b, "set_run_turn_time_scale"):
                    b.set_run_turn_time_scale(float(v2.get("troca_giro_tempo", 1.0)))
                if hasattr(b, "set_stop_gait_all"):
                    b.set_stop_gait_all(bool(v2.get("parada_passada", False)))
                if hasattr(b, "set_idle_pose_gate"):
                    b.set_idle_pose_gate(float(v2.get("idle_pose", 0.0)))
                if hasattr(b, "set_stop_clips_front"):
                    b.set_stop_clips_front(bool(v2.get("parada_clipes_frente", False)))
                if hasattr(b, "set_stop_clips_command"):
                    b.set_stop_clips_command(bool(v2.get("parada_clipes_comando", False)))
                if hasattr(b, "set_stop_gate_floor"):
                    b.set_stop_gate_floor(float(v2.get("parada_portao", 0.0)))
                if hasattr(b, "set_free_stop"):
                    b.set_free_stop(bool(v2.get("parada_livre", False)))
                if hasattr(b, "set_stop_posture"):
                    b.set_stop_posture(float(v2.get("parada_postura", 0.0)))
                if hasattr(b, "set_stop_idle_pose"):
                    b.set_stop_idle_pose(float(v2.get("parada_pose_sempre", 0.0)))
                if hasattr(b, "set_idle_unified"):
                    b.set_idle_unified(bool(v2.get("parado_unico", False)))
                if hasattr(b, "set_stop_to_idle"):
                    b.set_stop_to_idle(bool(v2.get("parada_direto_idle", False)))
                if hasattr(b, "set_idle_after_stop"):
                    b.set_idle_after_stop(bool(v2.get("idle_depois_freada", False)))
                if hasattr(b, "set_parked_on_locomotion"):
                    b.set_parked_on_locomotion(bool(v2.get("parado_na_locomocao", False)))
                if hasattr(b, "set_turn_variety"):
                    b.set_turn_variety(bool(v2.get("giro_variado", False)), float(v2.get("giro_coluna", 0.0)))
                if hasattr(b, "set_still_cost"):
                    b.set_still_cost(float(v2.get("quieto_custos", 0.0)))
                if hasattr(b, "set_idle_static"):
                    b.set_idle_static(bool(v2.get("parado_estatico", False)))
                if hasattr(b, "set_idle_costs"):
                    b.set_idle_costs(float(v2.get("parado_micro", 0.0)), float(v2.get("parado_fora_pose", 0.0)))
                if hasattr(b, "set_idle_reward"):
                    b.set_idle_reward(*[float(x) for x in v2.get("parado_recompensa", [0.0, 0.0, 0.0])])
                if hasattr(b, "set_upper_rigor"):
                    b.set_upper_rigor(float(v2.get("rigor_superior", 1.0)))
                if hasattr(b, "set_unreal_starts"):
                    b.set_unreal_starts(bool(v2.get("arrancada_unreal", False)))
                if hasattr(b, "set_unreal_start_scale"):
                    b.set_unreal_start_scale(float(v2.get("arrancada_unreal_escala", 0.0)))
                if hasattr(b, "set_live_launch"):
                    b.set_live_launch(bool(v2.get("largada_viva", False)))
                if hasattr(b, "set_launch_step_reward"):
                    b.set_launch_step_reward(float(v2.get("largada_passo", 0.0)))
                if hasattr(b, "set_launch_in_place_penalty"):
                    b.set_launch_in_place_penalty(float(v2.get("largada_no_lugar", 0.0)))
                    if float(v2.get("largada_inclinar", 0.0)) > 0 and hasattr(b, "set_launch_lean"):
                        b.set_launch_lean(float(v2["largada_inclinar"]))
                if hasattr(b, "set_launch_calm_arms"):
                    b.set_launch_calm_arms(bool(v2.get("largada_bracos_calmos", False)))
                if hasattr(b, "set_spine_velocity_feedforward"):
                    b.set_spine_velocity_feedforward(bool(v2.get("coluna_sem_velocidade", False)))
                if hasattr(b, "set_base_posture"):
                    b.set_base_posture(float(v2.get("postura_base", 0.0)))
                if hasattr(b, "set_motor_tone"):
                    b.set_motor_tone(float(v2.get("tonus", 1.0)))
                if hasattr(b, "set_posture_tail"):
                    b.set_posture_tail(float(v2.get("postura_cauda", 0.0)))
                if hasattr(b, "set_spine_tone"):
                    b.set_spine_tone(float(v2.get("tonus_coluna", 1.0)))
                if hasattr(b, "set_jam_cost"):
                    b.set_jam_cost(float(v2.get("custo_batente", 0.0)))
                if hasattr(b, "set_foot_slip_cost"):
                    b.set_foot_slip_cost(float(v2.get("pes_firmes", 0.0)))
                if hasattr(b, "set_motor_gravity"):
                    b.set_motor_gravity(bool(v2.get("gravidade_motores", False)))
                if hasattr(b, "set_stop_clips_base"):
                    b.set_stop_clips_base(bool(v2.get("parada_clipes_base", False)))
                if hasattr(b, "set_stop_entry"):
                    b.set_stop_entry(*[float(x) for x in v2.get("parada_entrada", [0.2, 0.3])])
                if hasattr(b, "set_stop_front_variants") and v2.get("parada_variantes", False):
                    b.set_stop_front_variants(True, int(os.environ.get("PARADA_VARIANTE", "-1")))
                if hasattr(b, "set_free_troca"):
                    b.set_free_troca(bool(v2.get("troca_livre", False)))
                if hasattr(b, "set_free_stand_gate"):
                    b.set_free_stand_gate(float(v2.get("parada_pose", 0.0)))
                if hasattr(b, "set_free_idle"):
                    b.set_free_idle(bool(v2.get("parado_livre", False)))
                if hasattr(b, "set_pose_boost"):
                    b.set_pose_boost(float(v2.get("pose_reforco", 0.0)))
                if hasattr(b, "set_free_push"):
                    b.set_free_push(bool(v2.get("empurrao_livre", False)))
                if hasattr(b, "set_free_idle_match"):
                    b.set_free_idle_match(float(v2.get("parado_semelhanca", 0.0)))
                if hasattr(b, "set_free_step_height"):
                    b.set_free_step_height(float(v2.get("passo_baixo", 0.0)))
                if hasattr(b, "set_idle_bent_end"):
                    b.set_idle_bent_end(bool(v2.get("parado_dobrado_encerra", False)))
                if hasattr(b, "set_idle_wide"):
                    b.set_idle_wide(bool(v2.get("parado_largo", False)))
                if hasattr(b, "set_mirror_cycles"):
                    b.set_mirror_cycles(bool(v2.get("simetria_ciclos", False)))
                if hasattr(b, "set_stop_clips") and v2.get("parada_clipes"):
                    b.set_stop_clips(True)
                if hasattr(b, "set_turn_settle_back"):
                    b.set_turn_settle_back(float(v2.get("troca_assentada_costas", 0.0)))
                if hasattr(b, "set_troca_fixed_chest"):
                    b.set_troca_fixed_chest(bool(v2.get("troca_peito_fixo", False)))
                if v2.get("penhascos_partes") is not None and hasattr(b, "set_cliff_parts"):
                    b.set_cliff_parts(*(part in v2["penhascos_partes"] for part in ("curvas", "pe", "antebraco", "pulo")))
        if hasattr(b, "set_foot_cross"):
            b.set_foot_cross(float(v2.get("pegada_cruzada", 0.0)))
        b.set_idle_reanchor(bool(v2.get("estab_ancora", False)))
    b.set_front_lean(*np.radians(v2.get("trote_frente_inclinar", [0.0, 0.0])))
    lean = v2.get("corrida_inclinar", [0.0, 0.0])
    b.set_run_lean(math.radians(float(lean[0])), float(lean[1]))
    lean = v2.get("corrida_costas_inclinar", [0.0, 0.0])
    b.set_run_back_lean(math.radians(float(lean[0])), float(lean[1]))
    b.set_rise_handoff(float(v2.get("entrega_subindo", 0.0)))
    b.set_get_up_tracking(bool(v2.get("levantar_clipe", False)))
    lean = v2.get("sprint_inclinar", [0.0, 0.0])
    b.set_sprint(float(v2.get("sprint_velocidade", 0.0)), math.radians(float(lean[0])), float(lean[1]))
    if v2.get("equilibrio"):
        # (04/11 15h, os passos: as sondas e os tetos como o treino - o gerador, a autoridade sob ameaca, a chegada; o equilibrio
        # parado so na avaliacao 8, no nivel do treino.)
        b.set_balance_steps(True, bool(v2.get("empurrao_idle", False)))
        b.set_idle_threat_reach(float(v2.get("idle_alcance_ameaca", 0.0)))
        b.set_get_up_arrival(float(v2.get("chegada_premio", 0.0)), float(v2.get("voo_multa", 0.0)),
                             float(v2.get("subida_equilibrio", 0.0)), float(v2.get("entrega_calma", 0.0)))
        b.set_balance_drill(0.0, float(v2.get("equilibrio_nivel", 0.5)))
        b.set_avoid_on_idle(bool(v2.get("evitar_idle", False)))
    b.set_accel_lean(math.radians(float(v2.get("troca_inclinar", 0.0))))


def batch_layout(v2: dict) -> dict:
    """O layout dos lotes de um treino (o v2 do run.json): o do equilibrio (levantar-v7) se ele tem --equilibrio."""
    out = {"balance_observation": True, "step_channels": True} if v2.get("equilibrio") else {}
    # (05/11: o levantar pelo clipe - o guiado no tempo do proprio clipe.)
    if v2.get("levantar_clipe"):
        out.update(get_up_guide=True, get_up_seconds=0.0)
    if v2.get("parado_ancora_obs"):
        out["idle_anchor_observation"] = True
    if v2.get("levantar_unreal_variante"):
        out["clip_variant_observation"] = True
    if v2.get("levantar_fogo_entrada"):
        out["fire_observation"] = True
    return out


def drill_requirements(level: float, key: str, ceiling: float | None, speed_ceiling: float | None,
                       stability_ceiling: float | None = None) -> dict:
    """O minimo de cada medida para descer a forca de um exercicio agora (os graficos do laboratorio, 04/11 12h35, ele:
    "mostrar exigencias... qual e o minimo exigido para cada caso para aceitar diminuir a forca auxiliar"): ate o fim, a
    velocidade como fracao do pedido (fora o giro e o parado) e a semelhanca absoluta (a exigida vezes o teto)."""
    out = {"fim": round(required_finish(level), 4)}
    if key not in ("lev_giro", "lev_parado", "lev_levantar"):
        out["vel"] = round(SPEED_REQUIRED * min(1.0, speed_ceiling if speed_ceiling else 1.0), 3)
    if ceiling and ceiling > 0.0:
        out["semelhanca"] = round(similarity_required(level, key, ceiling) * ceiling, 3)
    if course_drill(key):
        out["desvio"] = COURSE_DEGREES
    if key in CROSS_DRILLS:
        out["cruzado"] = CROSS_LIMIT
    if key == "lev_levantar":
        out["tempo"] = GETUP_SECONDS
    out["estabilidade"] = round(required_stability(level, key, stability_ceiling), 3)
    return out


def dynamic_assist(rows: np.ndarray, level: float, state: dict, seconds: float, minimum: int = 12, ceiling: float | None = None,
                   key: str = "", speed_ceiling: float | None = None, stability_ceiling: float | None = None):
    """A forca auxiliar dinamica de um exercicio (04/11, ele: "ve o quao bem ele esta adaptado aquele nivel... vai
    diminuindo, se piorou aumenta, sempre com o objetivo de ir diminuindo, de pouquinho, nunca bruscamente"). O estado:
    ref (a melhor marca de P, que esquece 0,002 por decisao), espera (decisoes sem descer depois de uma subida) e parado
    (decisoes seguidas sem mudar). Desce com o ate o fim >= required_finish (100% -> 90%), P >= 92% da ref e as outras
    exigencias (o passo cresce com a folga, ate dynamic_step); sobe 2-4% com o fim abaixo de 85% ou P abaixo de 75% da ref;
    parado 6 decisoes quase la, testa um quarto de passo para baixo. Devolve (nivel novo ou None, o desempenho)."""
    if len(rows) < minimum:
        return None, None
    perf = drill_performance(rows, seconds)
    p = perf["P"]
    state["ref"] = max(state.get("ref", p) - 0.002, p)
    ref = max(state["ref"], 1e-3)
    # (A semelhanca tem a marca dela: descer pede >= 97% da melhor semelhanca recente do exercicio.)
    if ceiling and ceiling > 0.0:
        # (O teto do exercicio - a semelhanca do proprio clipe no corpo fisico, teto_semelhanca.py -: a semelhanca da rede como
        # fracao dele contra a exigida no nivel de agora.)
        perf["sem_teto"] = min(1.0, perf["semelhanca"] / ceiling)
        perf["exigido"] = similarity_required(level, key, ceiling)
        # (05/11 20h35, ele: "a estabilidade compense a dissemelhanca com o clipe no que falta - estabilidade 84% (exigido
        # 68,3), superavit de 15,7; semelhanca 49% (exigido 66,4), deficit de 17,4; 17,4 - 15,7 = 1,7"): em pontos absolutos,
        # a sobra da estabilidade sobre a exigida cobre a falta da semelhanca; so nesse sentido (a estabilidade segue exigida).
        perf["compensa"] = round(compensation(perf["semelhanca"], perf["exigido"] * ceiling, perf["estabilidade"],
                                              required_stability(level, key, stability_ceiling)), 4)
        imitating = perf["compensa"] >= 0.0
        # (08h30: abaixo de 50% de forca os 70% sao obrigatorios - sem o "quase" do passo de teste.)
        strict = level <= 0.5 + 1e-6 and not key.startswith("lev_troca")
        near = perf["compensa"] >= -(0.0 if strict else 0.03 * ceiling)
    else:
        # (04/11 05h50: a marca esquecia 0,002 por decisao - ~14% por hora - e a semelhanca do trote escorregou de ~85% para
        # ~62% sem travar a descida da forca; agora 0,0005 - ~3,6% por hora.)
        state["ref_sem"] = max(state.get("ref_sem", perf["semelhanca"]) - 0.0005, perf["semelhanca"])
        imitating = near = perf["semelhanca"] >= 0.97 * state["ref_sem"]
    perf["vel_exigida"] = round(SPEED_REQUIRED * min(1.0, speed_ceiling if speed_ceiling else 1.0), 3)
    perf["estab_exigida"] = round(required_stability(level, key, stability_ceiling), 3)
    stable = perf["estabilidade"] >= perf["estab_exigida"]
    # (04/11 11h05, ele: "e inaceitavel mesmo com forca auxiliar as quedas estarem tanto, quase 20%": subir com menos de 85%
    # ate o fim (antes 60% - 20-40% de quedas passavam), descer so com 95%.)
    if perf["fim"] < FALL_RISE or p < 0.75 * ref:
        state.update(espera=3, parado=0)
        higher = min(1.0, level + (0.04 if perf["fim"] < 0.6 else 0.02))
        return (round(higher, 4) if higher > level else None), perf
    if state.get("espera", 0) > 0:
        state["espera"] -= 1
        return None, perf
    if level <= 0.0:
        return None, perf
    # (04/11 07h45, ele: "o quanto % ele acompanha o pedido... exigir no minimo 80% do pedido": descer pede a velocidade no rumo
    # >= 80% da que a referencia pede (no giro e no parado, sem pedido, 100%).)
    # (04/11 10h40: 80% do que o proprio clipe faz no exercicio - teto_velocidade.json; nas trocas o clipe puro com a mao
    # cheia so faz 44-73% do pedido e a regua absoluta as prendia em 100%.)
    fast = perf["vel"] >= perf["vel_exigida"]
    straight = (not course_drill(key) or perf.get("desvio", 0.0) <= COURSE_DEGREES) \
        and (key not in CROSS_DRILLS or perf.get("cruzado", 0.0) <= CROSS_LIMIT)
    if "tempo" in perf:
        # (O levantar: ficar de pe no tempo, no lugar da velocidade.)
        perf["tempo_exigido"] = GETUP_SECONDS
        fast = perf["tempo"] <= GETUP_SECONDS
    # (A folga de 1e-9: 57 de 60 episodios sao 0,95 e 0,90 + 0,10 * 0,5 da 0,9500000000000001.)
    finished = perf["fim"] + 1e-9 >= required_finish(level)
    if finished and p >= 0.92 * ref and imitating and fast and stable and straight:
        state["parado"] = 0
        step = dynamic_step(level) * float(np.clip((p / ref - 0.92) / 0.08, 0.25, 1.0))
        lower = level - step
        return round(lower if lower > 0.004 else 0.0, 4), perf
    state["parado"] = state.get("parado", 0) + 1
    quick = fast if "tempo" in perf else perf["vel"] >= perf["vel_exigida"] - 0.03
    if state["parado"] >= 6 and finished and near and stable and quick and straight:
        state["parado"] = 0
        lower = level - 0.25 * dynamic_step(level)
        return round(lower if lower > 0.004 else 0.0, 4), perf
    return None, perf


def summarize(rows: np.ndarray, prefix: str) -> dict:
    """As medidas de um conjunto de episodios (as colunas de finished_episodes)."""
    out: dict = {}
    if not len(rows):
        return out
    stood = rows[:, 5] >= 0
    # (04/11: o tempo medio do episodio - o fim antes da hora sem queda, o corpo virado alem do rumo, nao aparece no sem cair.)
    out[f"{prefix}seg"] = float(rows[:, 4].mean())
    if rows.shape[1] > 32:
        # (04/11, ele: "uns exercicios que no laboratorio reiniciam bem rapido as vezes, sem haver queda" - por que acabou.)
        for code, name in enumerate(("tempo", "chao", "equilibrio", "virado", "rastreio", "outro", "fora", "lento")):
            out[f"{prefix}fim_{name}"] = float((rows[:, 32] == code).mean())
    out[f"{prefix}de_pe"] = float(rows[:, 9].mean())
    out[f"{prefix}levantou"] = float(stood.mean())
    if rows.shape[1] > 41:
        # (04/11 15h, os passos: os de recuperacao e de estabilizacao por episodio, os empurroes, o pior ponto de captura fora da
        # base nos dois pes (m), se e em quanto tempo quietou depois do empurrao e a chegada do levantar na entrega, 0-1.)
        out[f"{prefix}passos_rec"] = float(rows[:, 35].mean())
        out[f"{prefix}passos_estab"] = float(rows[:, 36].mean())
        out[f"{prefix}empurroes"] = float(rows[:, 38].mean())
        out[f"{prefix}captura_fora"] = float(rows[:, 39].mean())
        quiet = rows[:, 40] >= 0
        out[f"{prefix}quietou"] = float(quiet.mean())
        out[f"{prefix}quietou_s"] = float(rows[quiet, 40].mean()) if quiet.any() else None
        arrived = rows[:, 41] >= 0
        out[f"{prefix}chegada"] = float(rows[arrived, 41].mean()) if arrived.any() else None
    out[f"{prefix}tempo"] = float(rows[stood, 5].mean()) if stood.any() else None
    for k, side in enumerate(SIDES):
        sel = rows[:, 2] == k
        out[f"{prefix}{side}"] = float(rows[sel, 9].mean()) if sel.any() else None
        out[f"{prefix}n_{side}"] = int(sel.sum())
    if rows.shape[1] > 10:
        # O levantar v2: chegou ao alvo e a velocidade rumo a ele (os com pedido), caiu depois de ja ter ficado de pe
        # e, no evitar a queda, evitou (nao caiu), se recompos e em quanto tempo.
        moving = rows[:, 10] > 0
        if moving.any():
            out[f"{prefix}chegou"] = float((rows[moving, 11] >= 0).mean())
            out[f"{prefix}vel_rumo"] = float(rows[moving, 12].mean())
        out[f"{prefix}caiu_de_pe"] = float(rows[:, 13].mean())
        out[f"{prefix}sem_cair"] = 1.0 - out[f"{prefix}caiu_de_pe"]
        # Os rumos: por exercicio (o rumo pedido no lado da linha), a velocidade no rumo, ela sobre a pedida (os que
        # comecaram pedindo), o sem cair e o erro do rumo do corpo; e o giro, o sem cair e o erro do rumo.
        loco = rows[:, 1] == 6
        for name, headings in HEADINGS.items():
            sel = loco & np.isin(rows[:, 2], headings)
            if sel.any():
                out[f"{prefix}{name}_vel"] = float(rows[sel, 12].mean())
                out[f"{prefix}{name}_razao"] = float((rows[sel, 12] / np.maximum(rows[sel, 10], 0.1)).mean())
                out[f"{prefix}{name}_sem_cair"] = float(1.0 - rows[sel, 13].mean())
                out[f"{prefix}{name}_seg"] = float(rows[sel, 4].mean())
                if rows.shape[1] > 16:
                    out[f"{prefix}{name}_rumo"] = float(rows[sel, 16].mean())
        if rows.shape[1] > 25:
            # (A rl_final, por faixa do comeco: a velocidade no rumo pedido, ela sobre a pedida, o sem cair e quantos.)
            for band, name in enumerate(BANDS):
                sel = loco & (rows[:, 25] == band)
                if sel.any():
                    out[f"{prefix}{name}_vel"] = float(rows[sel, 12].mean())
                    out[f"{prefix}{name}_razao"] = float((rows[sel, 12] / np.maximum(rows[sel, 10], 0.1)).mean())
                    out[f"{prefix}{name}_sem_cair"] = float(1.0 - rows[sel, 13].mean())
                    out[f"{prefix}{name}_n"] = int(sel.sum())
                    # (01/11, ele: "cuidar para ele nao trapacear" - o erro do rumo do corpo, de pe: virar para ir de frente
                    # quando o pedido e de costas ou de lado aparece aqui.)
                    out[f"{prefix}{name}_rumo"] = float(rows[sel, 16].mean())
        turn = rows[:, 1] == 7
        if turn.any() and rows.shape[1] > 16:
            out[f"{prefix}rumo"] = float(rows[turn, 16].mean())
        if rows.shape[1] > 15:
            # (A pose de pe no fim, perto do parado em alerta, e a coluna como a dele: os que terminaram de pe.)
            up = rows[:, 9] > 0.5
            out[f"{prefix}pose"] = float(rows[up, 15].mean()) if up.any() else None
            if rows.shape[1] > 17:
                out[f"{prefix}coluna"] = float(rows[up, 17].mean()) if up.any() else None
            if rows.shape[1] > 18:
                # (A troca de rede ao ficar de pe: quantos trocaram e em quanto tempo.)
                handed = rows[:, 18] >= 0
                out[f"{prefix}troca"] = float(handed.mean())
                out[f"{prefix}troca_s"] = float(rows[handed, 18].mean()) if handed.any() else None
                if rows.shape[1] > 21:
                    # (A parceira depois da troca: quantas cairam no tempo dela; e o tronco no chao em algum momento.)
                    partnered = rows[:, 20] >= 0
                    out[f"{prefix}parceiro_caiu"] = float(rows[partnered, 19].mean()) if partnered.any() else None
                    out[f"{prefix}sem_tocar_chao"] = float((rows[:, 21] == 0).mean())
                    if rows.shape[1] > 22:
                        out[f"{prefix}apoio"] = float(rows[:, 22].mean())
                    if rows.shape[1] > 23:
                        # (A flexao na queda: so os episodios com a queda iminente, os que tem o trecho medido.)
                        measured = rows[:, 23] != 0
                        out[f"{prefix}flexao"] = float(rows[measured, 23].mean()) if measured.any() else None
                    if rows.shape[1] > 24:
                        # (O cenario sujo: quantos levantaram com a parede ou o peso.)
                        dirty = rows[:, 24] > 0
                        out[f"{prefix}sujo_levantou"] = float((rows[dirty, 5] >= 0).mean()) if dirty.any() else None
                        stood = dirty & (rows[:, 5] >= 0)
                        out[f"{prefix}sujo_tempo"] = float(rows[stood, 5].mean()) if stood.any() else None
        avoid = rows[:, 1] == 4
        if avoid.any():
            ok = avoid & (rows[:, 14] >= 0)
            # (Com a parceira, ele pode comecar ja caindo, sem ter ficado de pe: evitou e o tronco nunca no chao.)
            out[f"{prefix}evitou"] = float((rows[avoid, 21 if rows.shape[1] > 21 else 13] == 0).mean())
            out[f"{prefix}recompos"] = float(ok.sum() / avoid.sum())
            out[f"{prefix}recompos_s"] = float(rows[ok, 14].mean()) if ok.any() else None
            if rows.shape[1] > 24:
                # (O evitar ambientado: so os episodios do evitar com o cenario.)
                scenery = avoid & (rows[:, 24] > 0)
                out[f"{prefix}evitou_ambientes"] = float((rows[scenery, 21] == 0).mean()) if scenery.any() else None
    return out


# (07/11, ele: "fica medindo ai o quanto ele esta se aproximando do juiz") O exercicio dos levantares deitados pela avaliacao.
JUDGE_KEYS = {"costas_": "lev_levantar_costas", "brucos_": "lev_levantar_brucos", "mov_": "lev_levantar_correndo"}


def evaluate(model: Policy, batch, device: torch.device, prefix: str = "", judges=None) -> dict:
    """A rede media nos comecos fixos da avaliacao (a semente nao muda): um episodio de cada mundo. Com os juizes (copias do
    da recuperacao e do da locomocao), a nota media deles em cada passo de cada episodio, como no estilo do treino - a
    locomocao pelo dela, o resto pelo outro; ~+1 parece clipe, ~-1 nao -, e por exercicio em <prefixo>juiz_exercicios."""
    batch.reset(12345)
    n = batch.num_envs
    done: dict[int, list] = {}
    judge_sum, judge_count, judged = np.zeros(n), np.zeros(n), {}
    for _ in range(int(batch.episode_seconds / batch.policy_dt) + 2):
        with torch.no_grad():
            obs = torch.as_tensor(batch.observations(), device=device)
            action = model.env_action(model.distribution(model.norm(obs)).mean, obs).cpu().numpy().astype(np.float32)
        batch.step(action)
        if judges is not None:
            with torch.no_grad():
                pairs = torch.as_tensor(batch.amp_pairs(), device=device)
                d = judges[0](pairs)
                if judges[1] is not None:
                    d = torch.where(obs[:, -1] > .5, judges[1](pairs), d)
            live = np.array([i not in done for i in range(n)])
            judge_sum[live] += d.reshape(-1).cpu().numpy()[live]
            judge_count[live] += 1
        for row in batch.finished_episodes().tolist():
            env = int(row[0])
            if env not in done and judges is not None and judge_count[env] > 0:
                judged[env] = judge_sum[env] / judge_count[env]
            done.setdefault(env, row)
        if len(done) >= n:
            break
    rows = np.array(list(done.values()))
    out = summarize(rows, prefix)
    if len(rows) and rows.shape[1] > 41 and (rows[:, 1] == 9).any():
        out.update(clip_get_up_summary(rows[rows[:, 1] == 9], prefix))
    if len(rows) and rows.shape[1] >= 76 and (rows[:, 1] == 11).any():
        out.update(get_up_go_summary(rows[rows[:, 1] == 11], prefix))
    if judges is not None and judged:
        out[f"{prefix}juiz"] = float(np.mean(list(judged.values())))
        groups: dict[str, list] = {}
        for env, row in done.items():
            if env in judged:
                key = JUDGE_KEYS.get(prefix) or drill_key(np.array(row))
                if key:
                    groups.setdefault(key, []).append(judged[env])
        out[f"{prefix}juiz_exercicios"] = {k: round(float(np.mean(v)), 4) for k, v in groups.items()}
    return out


def calibrate_locomotion_norm(model: "Policy", batch, device: torch.device, decisions: int = 48) -> dict:
    """A normalizacao da cabeca da locomocao medida numa coleta de aquecimento (a rede como veio, a acao media, so as
    linhas da locomocao) e a primeira camada do ator e do critico dela ajustadas para fazer a mesma conta de antes: com
    z_rec = (x - m_r) / s_r e z_loc = (x - m_l) / s_l, W z_rec + b = (W s_l / s_r) z_loc + b + W (m_l - m_r) / s_r. So o que
    passava do corte de +-5 muda. (03/11 11h55: sem o ajuste, a cabeca herdada da 2679 recebia as entradas em outra escala
    e perdia o equilibrio que trazia - o sem cair caiu de ~12% para ~4% antes do primeiro passo do ator.)"""
    rows = []
    with torch.no_grad():
        for _ in range(decisions):
            obs = torch.as_tensor(batch.observations(), device=device)
            moving = obs[:, -1] > .5
            if moving.any():
                rows.append(obs[moving].double())
            action = model.env_action(model.actor(model.norm(obs)), obs)
            batch.step(action.cpu().numpy().astype(np.float32))
        if not rows:
            return {"linhas": 0}
        x = torch.cat(rows)
        recovery, locomotion = model.norm.recovery, model.norm.locomotion
        mean, var = x.mean(0), x.var(0, unbiased=False).clamp_min(1e-6)
        mean[-1], var[-1] = recovery.mean[-1], recovery.var[-1]
        scale = (torch.sqrt(var + 1e-8) / torch.sqrt(recovery.var + 1e-8)).float()
        shift = ((mean - recovery.mean) / torch.sqrt(recovery.var + 1e-8)).float()
        for head in (model.actor.locomotion, model.critic.locomotion):
            first = head[0]
            first.bias += first.weight @ shift
            first.weight *= scale[None, :]
        locomotion.mean.copy_(mean)
        locomotion.var.copy_(var)
        locomotion.count.fill_(float(len(x)))
    return {"linhas": int(len(x)), "escala_max": float(scale.max()), "escala_min": float(scale.min())}


def expand_inputs(state: dict, observations: int) -> dict:
    """A rede de outro treino do levantar numa entrada maior (o v2: o comando depois do corpo): as colunas novas da
    primeira camada do ator e do critico em zero (a rede faz o mesmo de antes) e a normalizacao delas neutra."""
    out = dict(state)
    for key in (k for k in out if k.endswith(".0.weight") and k.startswith(("actor.", "critic."))):
        w = out[key]
        if observations > w.shape[1]:
            out[key] = torch.cat([w, torch.zeros(w.shape[0], observations - w.shape[1], dtype=w.dtype, device=w.device)], 1)
    for key, fill in [(k, 0.0 if k.endswith(".mean") else 1.0) for k in out
                      if k.startswith("norm.") and k.endswith((".mean", ".var"))]:
        v = out[key]
        if observations > v.shape[0]:
            out[key] = torch.cat([v, torch.full((observations - v.shape[0],), fill, dtype=v.dtype, device=v.device)])
    return out


def insert_inputs(state: dict, at: int, count: int) -> dict:
    """Entradas novas no meio (antes da habilidade, a ultima): colunas em zero na primeira camada do ator e do critico (a rede faz
    o mesmo de antes) e a normalizacao delas neutra."""
    out = dict(state)
    for key in (k for k in out if k.endswith(".0.weight") and k.startswith(("actor.", "critic."))):
        w = out[key]
        out[key] = torch.cat([w[:, :at], torch.zeros(w.shape[0], count, dtype=w.dtype, device=w.device), w[:, at:]], 1)
    for key in (k for k in out if k.startswith("norm.") and k.endswith((".mean", ".var"))):
        v = out[key]
        mid = torch.full((count,), 0.0 if key.endswith(".mean") else 1.0, dtype=v.dtype, device=v.device)
        out[key] = torch.cat([v[:at], mid, v[at:]])
    return out


def insert_adam_inputs(optimizer, at: int, count: int) -> int:
    """Os momentos do Adam das camadas que ganharam entradas: zeros nas colunas novas, o resto mantido."""
    grown = 0
    for group in optimizer.param_groups:
        for p in group["params"]:
            st = optimizer.state.get(p, {})
            for k in ("exp_avg", "exp_avg_sq"):
                m = st.get(k)
                if m is not None and m.dim() == 2 and m.shape[1] + count == p.shape[1]:
                    st[k] = torch.cat([m[:, :at], torch.zeros(m.shape[0], count, dtype=m.dtype, device=m.device), m[:, at:]], 1)
                    grown += 1
    return grown


def warm_start(model: Policy, threads: int, steps: int, device: torch.device) -> dict:
    """O comeco pela rede de brucos do DeepMimic (levantar_frente, que levanta 100% de brucos): a rede nova (sem a fase
    do clipe) aprende a acao dela nos estados dela, com o ruido do DART (Laskey et al. 2017: o professor age com um pouco
    de ruido, a aluna ve estados um pouco fora do caminho dele). O treino parte sabendo levantar de brucos."""
    from levantar_treino import GetUpPolicy as Teacher
    state = torch.load(DATA / "levantar_frente" / "ultimo.pt", map_location=device, weights_only=False)
    source = native.GetUpBatch(str(REPO / "assets"), clip="frente", environments=256, threads=threads, seed=7)
    teacher = Teacher(source.obs_size, source.action_size).to(device)
    teacher.load_state_dict(state["model"])
    teacher.eval()
    rows_x, rows_y = [], []
    with torch.no_grad():
        for _ in range(steps):
            obs = torch.as_tensor(source.observations(), device=device)
            act = teacher.actor(teacher.norm(obs))
            rows_x.append(obs[:, :-1].clone())
            rows_y.append((act - model.action_mid) / model.action_half)
            source.step((act + 0.05 * torch.randn_like(act)).cpu().numpy().astype(np.float32))
            source.finished_episodes()
    x, y = torch.cat(rows_x), torch.cat(rows_y)
    for chunk in x.split(16384):
        model.norm.update(chunk)
    optimizer = torch.optim.Adam(model.actor.parameters(), lr=3e-4)
    loss = torch.zeros(())
    for _ in range(30):
        for idx in torch.randperm(len(x), device=device).split(4096):
            loss = (model.actor(model.norm(x[idx])) - y[idx]).pow(2).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
    # A conferencia: a aluna no lote do professor, comecando deitada de brucos (o inicio do clipe).
    check = native.GetUpBatch(str(REPO / "assets"), clip="frente", environments=64, threads=threads, seed=9,
                              evaluation=True)
    ended: dict[int, list] = {}
    for _ in range(int(check.clip_seconds / check.policy_dt) + 2):
        with torch.no_grad():
            obs = torch.as_tensor(check.observations(), device=device)[:, :-1]
            action = model.env_action(model.actor(model.norm(obs))).cpu().numpy().astype(np.float32)
        check.step(action)
        for row in check.finished_episodes().tolist():
            ended.setdefault(int(row[0]), row)
    standing = float(np.mean([row[6] for row in ended.values()])) if ended else 0.0
    return {"aprendeu": True, "amostras_professor": int(len(x)), "perda": float(loss.item()), "de_pe_brucos": standing}


def build_parser() -> argparse.ArgumentParser:
    """O parser do treinador (10/11: fora do main, para a suite e a prova lerem a mesma receita)."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", required=True)
    parser.add_argument("--mundos", type=int, default=512)
    parser.add_argument("--passos", type=int, default=32)
    parser.add_argument("--threads", type=int, default=10)  # (05/11: 10 = ~93% da vazao das 12 com mais folga termica)
    parser.add_argument("--semente", type=int, default=1)
    parser.add_argument("--prova", type=str, default="",
                        help="a prova de equivalencia (prova.py): CPU deterministica e o hash de cada coleta e atualizacao neste arquivo")
    parser.add_argument("--atualizacoes", type=int, default=0, help="0: ate parar")
    parser.add_argument("--salvar-cada", type=int, default=50)
    parser.add_argument("--epocas", type=int, default=4)
    parser.add_argument("--minilotes", type=int, default=4)
    parser.add_argument("--taxa-ator", type=float, default=5e-5)
    # (03/11, a auditoria do Codex: 4 epocas x 4 minilotes sem freio, ~33% recortado; 0: sem a parada.)
    parser.add_argument("--kl-max", type=float, default=0.0, help="para os passos do ator na coleta quando o KL passa disto")
    parser.add_argument("--taxa-critico", type=float, default=2e-4)
    parser.add_argument("--taxa-disc", type=float, default=1e-4)
    # (31/10, ordem dele: o RL com liberdade para criar os proprios movimentos de levantar - a tarefa pesa mais que o
    # estilo, que so puxa para ficar parecido com os clipes.)
    parser.add_argument("--peso-tarefa", type=float, default=0.7)
    parser.add_argument("--peso-estilo", type=float, default=0.3)
    # (31/10 21h35, ele: "humanizar e suavizar um pouco os movimentos do levantar-se... bem funcionais, mas nitidamente
    # artificiais e recontorcidos": o custo do tranco - a diferenca da acao (na faixa das juntas) de uma decisao para a
    # seguinte, ao quadrado, media nos eixos - vezes este peso, na mesma escala da tarefa.)
    parser.add_argument("--suave", type=float, default=0.0, help="o peso do custo do tranco da acao")
    parser.add_argument("--desvio", type=float, default=None,
                        help="o desvio da acao (na faixa das juntas) a partir de agora, tambem ao retomar")
    # (05/11 00h50, ele: "aumentar a adaptatividade e evolucao da rede, voltou a estagnar... aumenta o ruido de exploracao".
    # O ruido branco a 30 Hz o PD e o corpo filtram; o correlacionado no tempo vira desvios que duram - pequenos empurroes
    # que a rede tem de corrigir. Hollenstein et al., "Colored Noise in PPO", AAAI 2024: o ruido correlacionado no PPO
    # explora melhor que o branco. A marginal segue N(media, desvio): o log da probabilidade do PPO e o mesmo.)
    parser.add_argument("--ruido-correlacionado", type=float, nargs=2, default=(0.0, 0.15), metavar=("FRACAO", "TAU"),
                        help="a fracao da variancia do ruido da exploracao que e correlacionada no tempo (um Ornstein-Uhlenbeck "
                             "de constante TAU s) - o resto, branco; 0: so o branco, como antes")
    # (05/11 00h50, a medida: a cabeca da locomocao usava metade do teto do KL nos 16 passos - aprendia devagar -, e a da
    # recuperacao batia o teto no 3o de 16 - a taxa alta demais, 13 dos 16 minilotes jogados fora. O ajuste de cada uma
    # pelo KL, como o rsl_rl das pernas robo.)
    parser.add_argument("--taxa-ator-rec", type=float, default=None,
                        help="(06/11) a taxa fixa so da cabeca da recuperacao (a locomocao com --taxa-ator); vazio: a mesma")
    parser.add_argument("--taxa-adaptativa", type=float, default=0.0,
                        help="o KL alvo do fim da atualizacao, por cabeca: a taxa de cada cabeca sobe ou desce ate ele "
                             "(entre 1e-6 e 3e-4); 0: a taxa fixa --taxa-ator")
    parser.add_argument("--taxa-adaptativa-loco", type=float, default=None,
                        help="(12/11 16h) o KL alvo so da cabeca da locomocao (sem: o --taxa-adaptativa); 0: a locomocao na taxa fixa "
                             "--taxa-ator (a dela antes de congelar, 11/11), mesmo retomando de um checkpoint com taxa adaptada")
    parser.add_argument("--comeco-clipe", type=float, default=0.25, help="a fatia dos comecos nos clipes de levantar")
    parser.add_argument("--comeco-de-pe", type=float, default=0.15, help="a fatia que comeca de pe (o clipe parado)")
    parser.add_argument("--professor-passos", type=int, default=800,
                        help="num treino novo: os passos da rede de brucos (256 mundos cada) para a aluna aprender")
    parser.add_argument("--imitar", type=float, default=0.0,
                        help="o peso da imitacao supervisionada dos alvos do clipe na cabeca da locomocao (0: sem)")
    parser.add_argument("--imitar-atualizacoes", type=int, default=300,
                        help="as atualizacoes em que o peso da imitacao cai linearmente a zero")
    parser.add_argument("--kl-reverter-loco", type=float, default=0.0,
                        help="(12/11 22h30) depois de cada passo, o KL da cabeca da locomocao no proprio minilote; acima disto o passo e "
                             "desfeito e a cabeca para na atualizacao (0: sem)")
    parser.add_argument("--kl-max-loco", type=float, default=0.0,
                        help="o freio de KL so da cabeca da locomocao (0: o --kl-max); 03/11 23h30: o 0,02 travava a "
                             "locomocao em toda atualizacao e ela nao aprendia o rumo")
    parser.add_argument("--estilo-loco-proprio", action="store_true",
                        help="(03/11 23h30, ele: 'porque ele nao esta tentando imitar o clipe, igual se foi feito no levantar?') "
                             "a locomocao com o juiz de estilo dela - so os clipes de locomocao como referencia - e o estilo "
                             "somado a tarefa, como no levantar (sem o estilo vezes a tarefa)")
    parser.add_argument("--loco-da-recuperacao", action="store_true",
                        help="(03/11 22h45, ele: 'ele ja sabe se equilibrar muito bem sozinho... o treino de locomocao esta "
                             "errado') a cabeca da locomocao vira uma copia da recuperacao - a que se equilibra, com o "
                             "musculo dela -, ignorando o pedido no comeco (as colunas do comando zeradas); uma vez por treino")
    parser.add_argument("--musculo", type=str, default="",
                        help="o musculo da cabeca da locomocao (json: rigidez, amortecimento, teto por eixo e gravidade)")
    parser.add_argument("--aquecer-critico", type=int, default=20,
                        help="as primeiras atualizacoes so do critico e do discriminador (a acao aprendida fica)")
    # O levantar v2 (31/10 17h, ele): levantar ja indo para um alvo e evitar a queda depois de um empurrao forte.
    parser.add_argument("--de", default="", help="num treino novo: comecar da rede de outro treino do levantar")
    parser.add_argument("--movimento", type=float, default=0.0, help="a fatia que levanta ja indo para um alvo")
    parser.add_argument("--evitar", type=float, default=0.0, help="a fatia do evitar a queda (de pe, o empurrao forte)")
    parser.add_argument("--evitar-forca", type=float, nargs=2, default=[1200.0, 1800.0], help="o empurrao curto (N)")
    parser.add_argument("--empurrao-forca", type=float, nargs=2, default=[400.0, 900.0], help="o de pe (N)")
    parser.add_argument("--velocidade", type=float, nargs=2, default=[2.5, 5.0], help="a pedida no movimento (m/s)")
    parser.add_argument("--empurrao-continuo", type=float, default=0.0,
                        help="a fatia continua dos empurroes de pe (o autoequilibrio parado)")
    parser.add_argument("--empurrao-continuo-forca", type=float, nargs=2, default=[150.0, 350.0], help="(N, por 1-3 s)")
    # (31/10 22h35, ele: so esta rede daqui em diante, com os basicos do B21 - os rumos e o giro parado suave -, o
    # autoequilibrio o principal: "se locomover resistindo a impactos e adversidades".)
    parser.add_argument("--habilidades-separadas", action="store_true", help="atores/críticos separados dentro da mesma política")
    # (04/11, ele: "adaptar os treinos de TROCAS de direcoes e curvas que possuiamos no b21": a fatia dos episodios de rumo
    # com manobra - as trocas de peito fixo, a troca livre e as curvas, no trote e na corrida - e a forca delas.)
    parser.add_argument("--manobras", type=float, default=0.0)
    parser.add_argument("--imitacao-parado", type=float, default=0.0,
                        help="o peso da imitacao do IDLE ALERTA no parado, regua estrita (04/11 04h10); 0: como antes")
    # (04/11 04h55, ele: o parado em alerta e o pos-levantar na pose do IDLE ALERTA: o idle vira a referencia ao ficar de
    # pe; o parado na cabeca da locomocao com o clipe do idle nos motores e a mao dinamica - lev_parado.)
    # (04/11 05h10, ele: "o botao do continuar treino ligue exatamente o treino da mesma forma como voce liga": o servico
    # do systemd do treino, gravado no run.json como o comando de abrir - o laboratorio usa esse comando.)
    parser.add_argument("--servico", default="", help="a unidade do systemd (--user) que abre este treino")
    # (04/11 05h50, ele: "a coluna e a cabeca mais estaveis... nao parecer tao mole": a mao tambem no tronco de cima.)
    parser.add_argument("--mao-tronco", action="store_true")
    # (04/11 06h00, ele: "garantir que a forca auxiliar atue em todos os ossos do corpo".)
    parser.add_argument("--mao-corpo", action="store_true")
    parser.add_argument("--idle-pos-levantar", action="store_true")
    parser.add_argument("--parado-locomocao", action="store_true")
    parser.add_argument("--idle-alcance", type=float, default=1.0,
                        help="o alcance da correcao no idle da cabeca da locomocao (fracao; 04/11 05h25: 0,2)")
    parser.add_argument("--idle-entrega", type=float, default=0.0,
                        help="depois de levantar, a entrega ao idle na hora com os alvos e a mao indo ao idle neste tempo (s; "
                             "04/11 09h10, ele: 'ficar de pe no idle alerta diretamente'); 0: com o corpo quieto, como antes")
    parser.add_argument("--entrega-subindo", type=float, default=0.0,
                        help="depois de levantar do chao, o idle assume ja subindo - os dois pes no chao, a cabeca acima disto (m), "
                             "a pelve acima de 0,55 m e o tronco em pe (05/11, ele: 'assim que ele esta ficando de pe ja cortasse "
                             "para o idle alerta'); 0: so de pe e sem subir, como antes")
    parser.add_argument("--naturalidade", type=float, default=0.0,
                        help="a multa da recuperacao (levantar e evitar) por aspereza dos alvos e juntas no limite, por decisao "
                             "(04/11 11h25, ele: 'recompensar a SUAVIDADE... e evitar juntas nos limites'); 0: sem")
    parser.add_argument("--corrida-lateral-pernas", type=float, default=1.0,
                        help="na corrida de lado, o alcance da correcao nas pernas vezes isto, e as pernas e os pes cobrados pela "
                             "metade (04/11 12h, ele: 'mais permissividade no movimento das pernas, movimentos mais amplos')")
    # (04/11 12h35, ele: "trote 4 m/s (todas direcoes); corrida 6 m/s para frente (e diagonais frente), 5,5 para tras e 4,5
    # lados - fazer ele aprender a pegar impulso, fazer forca contra o solo... sem tentar apenas aumentar a cadencia".)
    parser.add_argument("--velocidade-trote", type=float, nargs="+", default=[0.0],
                        help="o pedido fixo do trote (m/s; 1 valor para os 8 rumos ou 8, de 45 em 45 graus a partir da frente "
                             "para a esquerda), o clipe na cadencia dele; 0: o do clipe +-15%%")
    parser.add_argument("--velocidade-corrida", type=float, nargs="+", default=[0.0],
                        help="o pedido fixo da corrida, como --velocidade-trote")
    parser.add_argument("--curva-fora", type=float, default=0.0,
                        help="nas curvas, o rumo persegue o percurso desenhado e mais longe dele que isto (m) por 0,5 s encerra "
                             "como a queda (04/11 14h, ele: 'nas curvas ele nao segue a trajetoria e esta sendo deixado passar'); "
                             "0: o rumo no relogio, como antes")
    parser.add_argument("--curva-inclinar", action="store_true",
                        help="nas curvas, a referencia inclinada para dentro, atan(v x giro / g) (04/11 14h05, ele: 'a inclinacao "
                             "corporal correta, dinamica conforme o fechamento da curva')")
    parser.add_argument("--trote-frente-inclinar", type=float, nargs=2, default=(0.0, 0.0), metavar=("TROTE", "LARGADA"),
                        help="graus da coluna para a frente no trote de frente e nas diagonais da frente, e na largada de pe "
                             "(04/11 14h15, ele: 'uma inclinada para frente para pegar tranco; nao exigir a coluna reta'); 0: o clipe")
    parser.add_argument("--corrida-costas-inclinar", type=float, nargs=2, default=(0.0, 0.0), metavar=("GRAUS", "PERNAS"),
                        help="na corrida de costas e nas diagonais de costas, a referencia inclinada para tras, como a "
                             "--corrida-inclinar (05/11 04h10, ele: 'inclinar o corpo todo... para tras, algo sutil'); 0: o clipe")
    parser.add_argument("--corrida-inclinar", type=float, nargs=2, default=(0.0, 0.0), metavar=("GRAUS", "PERNAS"),
                        help="na corrida de frente, nas diagonais da frente e nas curvas da corrida, a referencia inclinada "
                             "para a frente (graus: a pelve e a coluna retas) com esta fracao nas pernas (0: so o dorso; 1: o "
                             "corpo todo) - 05/11, ele: 'o corpo todo mais inclinado para frente, nao a coluna apenas'; 0: o clipe")
    parser.add_argument("--largada-costas", type=float, default=0.0,
                        help="graus da coluna para tras na largada de pe dos rumos de costas, voltando aos 8 de sempre no fim "
                             "da rampa (04/11 13h40, ele: 'arranca com a coluna para frente ao inves de para tras'); 0: sem")
    parser.add_argument("--trote-lateral-sem-puxao", action="store_true",
                        help="a mao nao puxa no plano no trote de lado (04/11 12h: o pe da frente nao apoiava)")
    parser.add_argument("--idle-entrega-alcance", type=float, default=-1.0,
                        help="o alcance da correcao durante a entrega (fracao; < 0: o do idle)")
    # (08/11, ele: "6 exercicios novos, da estrela de oito pontas... garanta bem a boa estruturacao das fases de aprendizado...
    # inicialmente vamos deslocar uma % dos treinos a estrela".)
    parser.add_argument("--estrela", type=float, default=0.0,
                        help="(08/11) a fatia dos episodios de rumo do treino na estrela de 8 pontas (6 exercicios: trote e corrida, "
                             "peito fixo, livre e girando; cada um com a sua fase 1-4, que sobe com o desempenho)")
    parser.add_argument("--estrela-segundos", type=float, default=14.0, help="a duracao do episodio da estrela (s)")
    parser.add_argument("--comando-levantar", action="store_true",
                        help="(12/11 18h, ele: no COMANDO 'a conexao entre os dois brains, o da locomocao e do levantar... em perfeita "
                             "harmonia') caiu no comando, a cabeca do levantar levanta e devolve a locomocao no idle alerta; a queda nao "
                             "encerra (a multa fica)")
    parser.add_argument("--levantar-e-ir", type=float, nargs=3, default=(0.0, 5.0, 10.0), metavar=("FATIA", "MIN_M", "MAX_M"),
                        help="(12/11 19h10, ele: 'um exercicio que treina focado na TROCA de brains... inicia caido... precisa levantar, haver "
                             "a troca... e ele ir ate um ponto proximo... 5 - 10 metros') a fracao dos levantares pelo clipe que, depois da "
                             "troca, vao ate um ponto marcado a MIN-MAX m (o trote no de pe, a corrida no correndo; sem o fogo do levantar); "
                             "chegar a 1 m e o sucesso")
    parser.add_argument("--levantar-vista", type=float, default=0.0, metavar="NOTA",
                        help="(15/11 07h15, ele: 'quando ele se levanta, ele ja va girando para onde deve olhar, onde a camera estaria "
                             "apontada... mesmo que tenha que fazer um 180') nos levantares de pe com vista (o levantar e ir de pe - o ponto em "
                             "qualquer rumo, a vista nele desde o comeco - e o do comando), a vista na observacao do levantar, a nota soma NOTA x a "
                             "subida x o peito para a vista e na troca a vista fica (build-base-n113); 0: como antes")
    parser.add_argument("--levantar-brucos-tempo", type=float, default=1.0, metavar="ESCALA",
                        help="(15/11 07h12, ele: o de brucos 0,3 s mais lento que o de costas, 'ambos com as mesmas velocidades') o levantar de "
                             "brucos de pe com a faixa do tempo, o ritmo e o ritmo da imitacao vezes ESCALA (build-base-n113); 1: como antes")
    parser.add_argument("--mao-atras", type=float, default=0.0, metavar="PESO",
                        help="(15/11 08h30, ele: 'penalizar a mao ficar travada atras das costas, em todos os casos') de pe, cada mao mais de 10 cm "
                             "atras da pelve e a menos de 15 cm do meio por mais de 0,3 s seguidos custa PESO x (tempo alem de 0,3 s / 0,5 s, ate 1) "
                             "por passo (build-base-n114); 0: nada")
    parser.add_argument("--levantar-imitar-ancora", action="store_true",
                        help="(15/11 12h15) nos levantares de pe, o clipe da imitacao no ritmo preso no mundo desde o comeco (build-base-n117)")
    parser.add_argument("--pernas-alinhadas", type=float, nargs="+", default=None, metavar=("PESO", "LARGURA_GRAUS"),
                        help="(15/11 12h50) nos retos, a semelhanca vezes (1-PESO)+PESO x o alinhamento das coxas e dos pes contra o clipe (build-base-n120)")
    parser.add_argument("--largada-maos", type=float, default=0.0,
                        help="(15/11 15h56) na largada, a semelhanca cobra as maos contra o peito mais rapidas que no clipe (build-base-n128)")
    parser.add_argument("--parado-centro", type=float, default=0.0,
                        help="(15/11 15h20) no parado, o bonus vezes (1-PESO)+PESO/(1+(d/2,5cm)^2), d o centro de massa contra o meio dos pes menos o do clipe (build-base-n127)")
    parser.add_argument("--velocidade-largura", type=float, default=0.0,
                        help="(15/11 15h20) a largura do termo da velocidade como fracao da pedida (0: a de sempre, 0,35) (build-base-n127)")
    parser.add_argument("--corrida-largura", type=float, default=0.0,
                        help="(15/11 15h15) correndo reto, a semelhanca vezes (1-PESO)+PESO x exp(-((abertura-clipe-4cm)/6cm)^2) (build-base-n126)")
    parser.add_argument("--pernas-alinhadas-comando", action="store_true",
                        help="(15/11 15h15) as pernas alinhadas tambem no comando andando (build-base-n126)")
    parser.add_argument("--largada-margem-lado", action="store_true",
                        help="(15/11 15h10, a revisao) na largada, a margem do equilibrio so de lado no rumo pedido (build-base-n125)")
    parser.add_argument("--revisao-15h", action="store_true",
                        help="(15/11 15h, a revisao profunda) as misturas entre clipes corrigidas e o fogo aliviando o parado e o giro (build-base-n124)")
    parser.add_argument("--ppo-clipe-duplo", type=float, default=0.0,
                        help="(15/11 15h, a revisao profunda) o clipe duplo do PPO: com a vantagem negativa, a perda nao passa de C x A (0: desliga)")
    parser.add_argument("--parado-lugar", type=float, nargs="+", default=None, metavar=("PESO", "LARGURA_M"),
                        help="(15/11 13h00) no parado, o bonus vezes (1-PESO)+PESO/(1+(d/LARGURA)^2), d a distancia ao ponto onde o parado comecou (build-base-n121)")
    parser.add_argument("--parado-preso", action="store_true",
                        help="(15/11 12h23) no parado, o clipe do idle da imitacao preso na ancora do parado, nao na pelve de agora (build-base-n118)")
    parser.add_argument("--clipe-sob-pelve", action="store_true",
                        help="(15/11 12h20) o quadro posto sob a pelve poe a PELVE do clipe na do corpo, nao a origem dele (build-base-n117)")
    parser.add_argument("--levantar-lado-agora", action="store_true",
                        help="(15/11 09h05) o lado do clipe na nova tentativa e o do quadro de agora (set_clip_side_now): o clipe de costas ROLA "
                             "para de brucos no meio e, sem isto, seguir o clipe certo abria uma tentativa falsa no clipe de brucos (medido: a fase "
                             "da observacao pulava aos 0,7 s e tentou_de_novo do de costas era 1,00)")
    parser.add_argument("--levantar-chegada-de-pe", type=float, default=1.0, metavar="ESCALA",
                        help="(15/11 09h10, ele: 'o sucesso ao levantar deve ficar mais fraco do que o ganho por se assemelhar ao clipe') o premio "
                             "da chegada (--levantar-adaptativo-velocidade) nos levantares de pe vezes ESCALA; os correndo inteiros (build-base-n115)")
    parser.add_argument("--levantar-correndo-troca", type=float, default=-1.0, metavar="DEPOIS_S",
                        help="(12/11 18h40, ele: no levantar correndo 'assim que ele comeca ficar de pe ja trocar para a locomocao.. nunca "
                             "deixar o brain do levantar ser usado para a locomocao em si' e cortar o exercicio nos 'primeiros dois passos' "
                             "depois) >= 0: a troca no de pe (sem a velocidade nem a janela depois) e a locomocao so por estes segundos no "
                             "exercicio do correndo; < 0: como antes")
    parser.add_argument("--comando-levantar-correndo", type=float, default=0.5,
                        help="(12/11 18h, ele: 'levantar correndo... quando ele ta caido mas quer correr') no comando ligado, a fracao das "
                             "quedas andando em que o jogador segue segurando a tecla: o levantar correndo e a troca ja correndo")
    parser.add_argument("--estrela-caminho", type=float, nargs=2, default=[0.0, 1.2], metavar=("PESO", "LARGURA"),
                        help="(12/11 17h20, ele: 'tirar o encerramento automatico... por distanciamento do alvo... recompensar o nao "
                             "distanciamento') com PESO > 0 sair do caminho da perna nao encerra; a nota andando vezes 1 - PESO + PESO / "
                             "(1 + (d / LARGURA)^2), d a distancia ao caminho (m)")
    parser.add_argument("--estrela-curvas", type=float, nargs=3, default=[0.0, 0.15, 0.45], metavar=("FATIA", "MIN", "MAX"),
                        help="(12/11 17h20, ele: 'botar alguns com a obrigatoriedade de fazer curva, podendo ser mais aberta ou mais "
                             "fechada') a fatia das pernas da estrela em arco e a flecha dele (fracao da corda, para um lado sorteado)")
    # (08/11 14h40, ele: "preparativos para que essa rede consiga interpretar e responder bem aos comandos" - o jogo.)
    parser.add_argument("--comando", type=float, default=0.0,
                        help="(08/11) a fatia dos episodios de rumo no comando: um jogador emulado - WASD (8 rumos em relacao a vista), "
                             "shift (corrida), mouse (a vista: parada, panoramica, flick), soltar para parar, apertar para arrancar; a fase "
                             "1-4 sobe com >= 75%% ate o fim e >= 6 eventos, desce com < 40%%")
    parser.add_argument("--comando-segundos", type=float, default=20.0, help="a duracao do episodio do comando (s)")
    # (08/11 15h20, ele: "ao final da rodada a PARADA... a arrancada, o movimento, a parada".)
    parser.add_argument("--parada", type=float, default=0.0,
                        help="(08/11) a fatia dos episodios retos do treino que acabam com a parada: o pedido some na janela, a referencia "
                             "freia e passa ao idle; a nota pela precisao contra a freada perfeita (ate 40%%), a responsividade e o "
                             "equilibrio parado; 0: sem")
    parser.add_argument("--parada-janela", type=float, nargs=2, default=[4.5, 5.5], help="o instante da parada (s)")
    parser.add_argument("--parada-freio", type=float, default=5.0, help="a freada da referencia (m/s^2)")
    parser.add_argument("--parada-clipes", action="store_true",
                        help="(08/11 16h20) a referencia da parada pelas paradas da Unreal (8 rumos, os dois pes, trote e corrida): "
                             "o passo que freia e assenta, no lugar da passada da corrida com a velocidade caindo")
    parser.add_argument("--comando-fase", type=int, default=1, help="a fase do comeco (uma vez; depois a do treino)")
    parser.add_argument("--estrela-fases", type=int, nargs=6, default=[1, 1, 1, 1, 1, 1],
                        help="as fases do comeco (uma vez; depois as do treino): trote fixo, livre, girando, corrida fixo, livre, girando")
    # (08/11, ele: "em TODOS os treinos atuais, as pancadas e empurroes... os empurroes continuos de 3-5 segundos e as pancadas
    # secas... ir aumentando as forcas conforme ele aprende... a fase de fogo: o objetivo vai ser ter a maior taxa de sucesso de
    # nao cair possivel".)
    parser.add_argument("--fogo", type=float, default=-1.0,
                        help="(08/11) >= 0: a fase de fogo - pancadas secas e empurroes continuos de 3-5 s em todos os exercicios (em "
                             "movimento, parado e depois de levantar), no nivel do comeco (uma vez; depois o do treino): 0 = pancada 350 N "
                             "e empurrao 120 N fixos, 1 = 600-1400 e 200-500 N, ate 2 (900-1800 e 280-700); sobe 0,1 com >= 92%% sem "
                             "cair no empurrao, desce com < 75%%; < 0: sem")
    parser.add_argument("--fogo-fatia", type=float, default=0.5, help="a fatia dos episodios do treino com empurroes")
    parser.add_argument("--lento", type=float, default=0.0,
                        help="(08/11) > 0: nos retos do treino, andar abaixo desta fracao da velocidade pedida por 1 s (depois de 2,5 s, "
                             "fora do empurrao) encerra como a queda; o piso do comeco (uma vez; depois o do treino) sobe 0,05 com "
                             ">= 90%% dos retos sem o fim por lento e desce com < 70%%, entre 0,4 e 0,85; 0: sem")
    # (08/11 16h55: das 15h20 as 16h50 os pisos desceram em passeio - frente 0,70 -> 0,60, lados 0,50 -> 0,40, costas
    # 0,80 -> 0,65 - e as velocidades dos testes fixos junto: lateral do trote 70 -> 61%, da corrida 50 -> 40%, costas
    # 84 -> 74%. O minimo segura a pressao que levou as velocidades ao melhor delas.)
    parser.add_argument("--lento-faixa", action="store_true",
                        help="(09/11 01h20) o piso do lento por grupo E por faixa (trote, corrida): o trote levava o piso da frente a "
                             "0,80 e metade das corridas acabava por lento; na migracao a corrida comeca 0,10 abaixo")
    parser.add_argument("--lento-minimo", type=float, nargs=3, default=[0.4, 0.4, 0.4],
                        help="(08/11) o piso minimo de cada grupo (frente, lados, costas): o --lento nao desce abaixo dele")
    parser.add_argument("--cabeca-clipe", type=float, default=-1.0,
                        help="(08/11) >= 0: andando, o pescoco e a cabeca no clipe (como --bracos-clipe), mais a correcao da rede "
                             "vezes isto; na primeira vez, as saidas deles zeradas; vale em todos os lotes; < 0: livres")
    parser.add_argument("--lateral-pernas-rigor", type=float, default=1.0,
                        help="(08/11) nos retos de lado, a regua da imitacao das pernas e dos pes vezes isto (o chute para tras da "
                             "perna no trote e na corrida lateral); so no treino; 1: a de todos")
    parser.add_argument("--levantar-voo", type=float, default=0.0,
                        help="(08/11) no levantar, depois de tocar o chao e ainda baixo, o corpo inteiro no ar custa isto por decisao "
                             "(a flexao explosiva que tira o corpo do chao - o pulinho); so no treino")
    parser.add_argument("--fogo-alivio", action="store_true",
                        help="(08/11) durante o empurrao e ate 1,5 s depois, a regua da imitacao pela metade (recuperar vale mais que "
                             "copiar a pose); so no treino")
    parser.add_argument("--rumo-suave", type=float, default=0.0,
                        help="(08/11 18h45) o rumo da nota da velocidade numa velocidade suavizada em tantos s (0,6: um passo); "
                             "0: a de 0,25 s")
    parser.add_argument("--rumo-limite", type=float, default=0.0,
                        help="(08/11 20h05) graus: nos retos do treino, o deslocamento mais que isto fora do rumo pedido por 1 s (fora do "
                             "empurrao, da largada e da parada) encerra como a queda; 0: nao")
    parser.add_argument("--sem-potencial-loco", action="store_true",
                        help="(08/11 23h55) sem os potenciais do levantar nos passos da cabeca da locomocao (de pe cobravam 0,24 por "
                             "passo); a queda de pe dela custa 2x a multa")
    parser.add_argument("--nota-velocidade", type=float, default=0.0,
                        help="(08/11 23h55) andando, a nota do rastreio vezes F + (1 - F) x a nota da velocidade (0,6 no empurrao); 0: nao")
    parser.add_argument("--joelho-dentro", type=float, default=0.0,
                        help="(09/11 01h20, ele: 'evitar dele correr com os joelhos colados') m: andando, o joelho para dentro da linha "
                             "quadril-tornozelo alem do clipe (+2 cm) derruba a semelhanca: exp(-(excesso/F)^2); 0: nao")
    parser.add_argument("--idle-altura", type=float, default=0.0,
                        help="(09/11 03h40, ele: 'quando ele esta parado ... dificuldades de manter a pose') m: no idle, a pelve abaixo "
                             "da do idle alem de 1,5 cm derruba a semelhanca da pose: exp(-(excesso/F)^2); 0: nao")
    parser.add_argument("--parada-corpo", action="store_true",
                        help="(09/11 03h50, ele: 'cai muito ao frear') na parada final, o idle so com o corpo tambem abaixo de 1,5 m/s "
                             "(ou 2,5 s depois da ordem); antes a corrida ia ao idle a 4 m/s e tombava")
    parser.add_argument("--estilo-velocidade", type=float, default=0.0,
                        help="(08/11 23h55) na locomocao, o estilo do juiz vezes F + (1 - F) x a nota da velocidade; 0: nao")
    parser.add_argument("--diagonal-peito", type=float, default=0.0,
                        help="(08/11 18h40) o portao do peito alto para a vista nos diagonais retos do treino (0,4 como o --peito-vista); 0: sem")
    parser.add_argument("--fogo-alvo", type=float, default=0.0,
                        help="(08/11 18h05) > 0: o nivel de cada grupo mira este sucesso nos episodios COM empurrao (0,5: ele cai em "
                             "metade deles) - fire_step_target; 0: a razao com/sem de antes (fire_step)")
    parser.add_argument("--fogo-pontos", action="store_true",
                        help="(08/11) o ponto do empurrao sorteado no treino: metade o tronco repartido, 20%% o peito alto, 15%% a "
                             "pelve, 15%% um ombro (o fogo versatil)")
    # (08/11, ele: "trabalhe para aumentar as velocidades": a rede corria na velocidade do clipe dos motores.)
    parser.add_argument("--passos-ciclos", action="store_true",
                        help="(08/11) os passos do trote e da corrida nos motores pelos ciclos nas velocidades-meta "
                             "(ciclos_meta.py: trote 4,5/2,5/3,35, corrida 6,0/3,5/5,5) no lugar do jog (3,0 m/s), da "
                             "run_forward_medium (4,33) e dos outros; vale em todos os lotes (muda o movimento)")
    parser.add_argument("--bracos-clipe", type=float, default=-1.0,
                        help="(08/11) >= 0: andando, os motores dos bracos com o alvo e a velocidade do clipe (na fase do passo) "
                             "mais a correcao da rede vezes isto; na primeira vez, as saidas dos bracos zeradas (comeca no clipe); "
                             "vale em todos os lotes (muda o movimento); < 0: livres")
    parser.add_argument("--peito-vista", type=float, default=0.0,
                        help="(08/11) andando, a nota vezes 1 - g + g exp(-(excesso/0,6 rad)^2), o rumo da pelve contra o do clipe "
                             "alem de 20 graus - o peito virando cobrado antes do encerramento; so no treino (0: sem)")
    parser.add_argument("--imitacao-folga", type=float, default=1.0,
                        help="(08/11) andando, os expoentes da imitacao do clipe divididos por isto SO NO TREINO - a regua larga "
                             "longe do clipe; as avaliacoes seguem na regua de antes (1: sem)")
    parser.add_argument("--sincronizar-fase", action="store_true",
                        help="(08/11) no pouso do pe esquerdo nos rumos retos, a fase do clipe volta ao pouso esquerdo dele: as "
                             "comparacoes com o clipe e a fase observada no passo do corpo")
    parser.add_argument("--rastreio-encerra", type=float, default=0.0,
                        help="a media movel da nota do rastreio (~0,5 s) abaixo disto encerra, depois de 1 s (0: so a queda)")
    parser.add_argument("--simetria", type=float, default=0.0,
                        help="(05/11, o vies para a direita) o peso da perda de simetria na cabeca da locomocao: a acao no "
                             "estado espelhado e o espelho da acao (Yu, Turk e Liu 2018), nos rumos retos com o clipe "
                             "periodico (mirror_info); 0: so mede a assimetria no log")
    parser.add_argument("--pisada-largura", type=float, default=0.0,
                        help="> 0: a largura entre os pes contra a do clipe na semelhanca dos rumos retos, a folga em m (05/11)")
    parser.add_argument("--troca-freio", action="store_true",
                        help="nas trocas com a meia-volta, na corrida: freia ate o trote, gira trotando e volta a correr (05/11 21h)")
    parser.add_argument("--pegada-cruzada", type=float, default=0.0,
                        help="> 0: na corrida para a frente e nas diagonais da frente, o pe apoiado mais para dentro que o mesmo "
                             "pe do clipe (alem de 1,5 cm) derruba a semelhanca, com esta escala em m (05/11 22h55: os passos cruzados)")
    parser.add_argument("--troca-assentada", action="store_true",
                        help="de costas, freia ate 1,5 m/s antes da meia-volta (05/11 22h40)")
    parser.add_argument("--troca-arrancada", action="store_true",
                        help="na media e na forte, a saida da meia-volta pela arrancada agachada (05/11 23h20)")
    parser.add_argument("--troca-peito-fixo", action="store_true",
                        help="saindo de costas ou de lado, a media e a forte de peito fixo, sem a meia-volta (06/11 13h05)")
    parser.add_argument("--troca-assentada-costas", type=float, default=0.0,
                        help="de costas, a freada ate esta velocidade (m/s) antes da meia-volta (06/11 12h); 0: 1,5")
    parser.add_argument("--troca-assentada-frente", type=float, default=0.0,
                        help="(09/11 04h30, ele: 'mais na troca de direcao forte') de frente (0, 1, 7), a freada ate esta velocidade "
                             "(m/s) antes da meia-volta das trocas; 0: sem")
    parser.add_argument("--simetria-ciclos", action="store_true",
                        help="(10/11 02h30) o espelho da perda de simetria tambem nos clipes ciclo_* (frente e costas meio ciclo, "
                             "esquerda e direita em par) - com os nomes antigos so, ela estava desligada nos retos")
    parser.add_argument("--parado-largo", action="store_true",
                        help="(10/11 01h50) a nota do parado com gradiente no agachado: a altura e o peito alto com o nucleo fino mais "
                             "a rampa larga, e a semelhanca com o idle tambem na regua larga")
    parser.add_argument("--peso-tarefa-parado", type=float, default=None,
                        help="(10/11 01h40) o peso da tarefa nas linhas do parado com a cabeca da locomocao (o exercicio parado e o do "
                             "equilibrio); com --peso-estilo-parado. Sem: os da recuperacao (o juiz preferia o corpo dobrado)")
    parser.add_argument("--peso-estilo-parado", type=float, default=None)
    parser.add_argument("--parado-estatico", action="store_true",
                        help="(11/11 11h30, ele) o idle alerta 100%% estatico: o quadro 0 fixo como alvo do parado")
    parser.add_argument("--quieto-custos", type=float, default=0.0,
                        help="(11/11 10h30, ele: 'em definitivo') os custos de quietude do RL v1 no parado e no giro parado (0-1)")
    parser.add_argument("--parado-segundos", type=float, default=0.0,
                        help="(11/11 10h30) a duracao dos episodios fora da locomocao (o parado, o levantar); 0: a de antes (6 s)")
    parser.add_argument("--giro-variado", action="store_true",
                        help="(11/11 09h50, ele) o giro parado em giros fixos de 60/90/180/360 graus, velocidades variadas, inversao e pausas")
    parser.add_argument("--giro-coluna", type=float, default=0.0,
                        help="(11/11 09h50, ele: 'recompensar ele melhor por manter a coluna toda reta') no giro, o bonus da coluna reta")
    parser.add_argument("--idle-depois-freada", action="store_true",
                        help="(11/11 09h28, ele) a freada como era (a parada livre) e, freado, o idle alerta com a nota do parado")
    parser.add_argument("--parado-torto", type=float, nargs=3, default=[0.0, 2.0, 0.8], metavar=("FATIA", "PREMIO", "PERTO"),
                        help="(12/11 17h, ele: o parado nascendo torto - a coluna, os bracos e as pernas - e um pouco acima do chao, para "
                             "aprender a se REAJUSTAR o mais rapido e ficar firme) a fatia dos parados do treino que nascem tortos, o premio "
                             "de reajustar (cheio no comeco, zero em 2,5 s) e o perto da pose que conta como reajustado (0,5 forma + 0,5 lugar)")
    parser.add_argument("--locomocao-fogo", type=float, default=0.0,
                        help="(12/11 16h30, ele) a fatia dos episodios de locomocao do treino com o fogo da locomocao v1 (rumos, manobras, "
                             "estrela, giro e parado; o comando nao) - vault 04_locomocao/plano_fogo_da_locomocao_12_11.md")
    parser.add_argument("--locomocao-fogo-nivel", type=float, default=0.0,
                        help="o nivel L de partida dos grupos (0 sutil - 1 o fogo do levantar de hoje); so na primeira vez")
    parser.add_argument("--locomocao-fogo-liberdade", type=float, default=0.0,
                        help="a liberdade sob o fogo (o rework): a perturbacao p (0-1) vezes isto abre a janela de improviso da nota, "
                             "solta a firmeza, a inclinacao e o parado; 0 sem")
    parser.add_argument("--locomocao-fogo-faixa", type=float, nargs=2, default=[0.0, 1.0], metavar=("MIN", "MAX"),
                        help="(12/11 17h50, ele: 'um passo a mais na fase de fogo, ~30-50%% do que hoje e no de levantar') os niveis L dos "
                             "grupos ficam nesta faixa: ao carregar, os abaixo sobem ao MIN; a escada nao passa dela")
    parser.add_argument("--locomocao-fogo-fatia-inicial", type=float, default=0.0,
                        help="(12/11 20h30) a fatia do fogo da locomocao comeca aqui (a primeira vez) e sobe 0,1 a cada 3 medidas boas do "
                             "limpo ate --locomocao-fogo; 0: comeca nela")
    parser.add_argument("--parado-torto-janela", type=float, default=2.5,
                        help="(12/11 20h45) o premio da rapidez do reajuste do torto zera nestes segundos (era fixo em 2,5)")
    parser.add_argument("--parado-torto-sem-fogo", action="store_true",
                        help="(12/11 20h15) o parado que nasce torto fica sem o fogo da locomocao (exercicio de postura)")
    parser.add_argument("--locomocao-fogo-estilo-livre", action="store_true",
                        help="(12/11 20h05) enquanto a forca do fogo age (a liberdade), o peso do estilo da locomocao vai para a tarefa")
    parser.add_argument("--locomocao-fogo-piso-grupo", nargs=2, action="append", metavar=("GRUPO", "PISO"),
                        help="(12/11 22h35) o piso do nivel de um grupo do fogo da locomocao abaixo da faixa geral (ex.: parado 0.3)")
    parser.add_argument("--locomocao-fogo-intervalo", type=float, default=1.0,
                        help="(12/11 20h) a escala do intervalo depois de cada evento do fogo da locomocao (1: 1,5-3,5 s andando, 1,0-2,5 s "
                             "parado); maior deixa trechos sem forca entre os eventos")
    parser.add_argument("--locomocao-fogo-guarda-velocidade", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("TROTE", "CORRIDA", "LADO"),
                        help="(12/11 19h55) a escada do fogo da locomocao so sobe com a velocidade do limpo (capacidade, sobre a pedida) "
                             "a 0,05 destas: trote (frente/diagonais), corrida e trote de lado; zeros sem guarda")
    parser.add_argument("--locomocao-fogo-degrau-rodadas", type=int, default=0,
                        help="(13/11 17h35, ele: elevar o fogo para perto de 100%% do levantar, acelerar o aprendizado) as rodadas minimas num "
                             "nivel antes de subir o proximo degrau (e 2x antes de descer por travado); 0: as de sempre (100)")
    parser.add_argument("--locomocao-fogo-aceleracao", type=float, nargs=3, default=(0.0, 0.1, 3),
                        metavar=("TOL_TORTO", "PASSO_FATIA", "MEDIDAS_BOAS"),
                        help="(13/11 18h50, ele: o fogo perto de 100%% do levantar, acelerar o aprendizado) a tolerancia do reajuste do torto na "
                             "guarda do limpo (0: a de sempre, 0,15), o degrau da fatia do fogo e as medidas boas seguidas para ela subir")
    parser.add_argument("--locomocao-fogo-congelar", action="store_true",
                        help="a escada do fogo da locomocao sem subir (as guardas ainda descem)")
    parser.add_argument("--locomocao-fogo-prova", type=float, default=1.0,
                        help="o nivel L da prova de fogo da locomocao (na capacidade); 0 sem prova")
    parser.add_argument("--parado-na-locomocao", action="store_true",
                        help="(12/11 16h) o idle estacionado depois da parada fica na cabeca da locomocao (antes ia para a recuperacao "
                             "1-2 s depois da ordem de parar)")
    parser.add_argument("--parada-direto-idle", action="store_true",
                        help="(11/11, ele: 'nao existe mais freio... se ele para a locomocao vai para o idle alerta e pronto') a ordem de "
                             "parar poe o alvo direto no idle alerta na vista, sem clipe de parada e sem parada livre")
    parser.add_argument("--parado-micro", type=float, default=0.0,
                        help="(11/11, ele: 'Parado - penaliza micromovimentos') com --parado-unico, o custo do movimento no parado (0-1)")
    parser.add_argument("--parado-fora-pose", type=float, default=0.0,
                        help="(11/11, ele: 'penaliza fora da pose') com --parado-unico, o custo da distancia a pose no parado (0-1)")
    parser.add_argument("--levantar-imitar", type=int, default=0, metavar="MASCARA",
                        help="(14/11 v1, ele: o levantar de costas MUITO distante do clipe, 'o mais semelhante possivel') os alvos da "
                             "imitacao supervisionada (--cerebro-unico-imitar) nos levantares limpos da MASCARA (bit 0 costas, 1 brucos, "
                             "2 e 3 os correndo) = o clipe do comeco no ritmo do --levantar-ritmo (nao o relogio adaptativo); os outros "
                             "sem alvo; 0: como antes (build-base-n104)")
    parser.add_argument("--levantar-imitar-longe", type=float, nargs=2, default=(0.0, 0.6), metavar=("LIMIAR", "SEGUNDOS"),
                        help="(14/11 v1) com --levantar-imitar: a semelhanca com o clipe no ritmo abaixo de LIMIAR por SEGUNDOS encerra o "
                             "levantar (o fim cedo); 0: sem (build-base-n106)")
    parser.add_argument("--levantar-imitar-ritmo-correndo", type=float, default=0.0, metavar="SEGUNDOS",
                        help="(14/11 21h45, ele: 'cobrar mais semelhanca com o clipe para os nossos levantares correndo') com --levantar-imitar: o "
                             "ritmo proprio dos correndo (s ate o ponto de entrega; os de pe ficam no do --levantar-ritmo) (build-base-n109); 0: o mesmo")
    parser.add_argument("--levantar-imitar-nota", type=float, default=0.0, metavar="PESO",
                        help="(14/11 v1) com --levantar-imitar: a nota por passo da semelhanca da pose com o clipe do comeco no ritmo, "
                             "nos levantares limpos da mascara ate a troca; 0: sem (build-base-n105)")
    parser.add_argument("--parada-bracos-clipe", type=float, default=-1.0, metavar="CORRECAO",
                        help="(13/11 v1: na freada a rede jogava a mao ~50 cm acima do clipe em 36%% das paradas; medido, sem a correcao dela "
                             "nos bracos, 14%% e 0 queda) freando, a correcao da rede nos bracos (--bracos-clipe) vira CORRECAO; 0 = so o clipe; "
                             "< 0 desligado; muda o movimento: vale em todos os lotes (precisa da build-base-n103)")
    parser.add_argument("--virado-tolerancia", type=float, default=0.0, metavar="SEGUNDOS",
                        help="(13/11 v1: das quedas do comando e da estrela livre, ~80%% eram o fim por virado - a referencia saltando ou girando de "
                             "proposito e o corpo ainda virando) o limite do virado ganha SEGUNDOS enquanto a referencia do rumo gira a mais de "
                             "~70 graus/s e ate SEGUNDOS depois; 0: desligado (precisa da build-base-n101)")
    parser.add_argument("--parado-quieto-na-base", type=float, default=0.0, metavar="TRAVA",
                        help="(13/11 v1, a volta a base do idle: depois de parar ele ficava com os pes paralelos - o idle alerta tem um pe ~14 cm "
                             "a frente) a quietude do parado vale 1 - TRAVA x (1 - perto da base dos pes), sem fogo: quieto na base errada "
                             "vale menos e o passo de acerto se paga; 0: desligado (precisa da build-base-n100)")
    parser.add_argument("--parado-recompensa", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        help="(11/11 08h58, ele) com --parado-unico, a nota do parado: POSE x proximidade da pose alvo + QUIETO x quietude "
                             "- DESLIZE x velocidade dos pes apoiados (m/s); 0 0 0: a nota de antes")
    parser.add_argument("--parado-unico", action="store_true",
                        help="(11/11, o plano das solucoes) o parado unico: no parado (o exercicio e o de depois de frear) a nota e pose x "
                             "firmeza, na regua que nao satura e sem janelas; o parado de depois de frear conta como parado no treinador")
    parser.add_argument("--cabeca-autoridade", action="store_true",
                        help="(11/11, E5-13) com --cabeca-clipe R: a correcao da rede no pescoco e na cabeca passa a alcance 1 (autoridade "
                             "inteira sobre o clipe); na troca as saidas e o desvio da cabeca vezes R - o alvo e o ruido comecam iguais")
    parser.add_argument("--juiz-andando", action="store_true",
                        help="(11/11, E5-8) o juiz da locomocao aprende sem freio so nas linhas andando (o portao da velocidade > 0); "
                             "o parado e a pos-parada ficam com o estilo de uma copia congelada do juiz do ponto de partida, como antes")
    parser.add_argument("--parada-pose-sempre", type=float, default=0.0,
                        help="(11/11, E5-7, ele: 'a pose deve ser cobrada sempre') na parada livre, conforme ele para, a nota vira a pose do "
                             "idle alerta na regua estrita: (1-k*near)*nota + k*near*(0,25+0,75*semelhanca); 0: sem")
    parser.add_argument("--parado-ancora-obs", action="store_true",
                        help="(11/11, E5-6) a rede ve a ancora do parado: o ponto onde o parado comecou, na frente e ao lado da pelve "
                             "(em 10 cm, ate 3), que a nota ja cobra e a entrada nao tinha; as colunas novas em zero (comeca igual)")
    parser.add_argument("--linha-final-so", choices=("custo", "estilo"), default=None,
                        help="(11/11, sonda da E5-4, morre na decisao) com --linha-final: so o teto do custo ou so o portao do estilo")
    parser.add_argument("--linha-final", action="store_true",
                        help="(11/11, E5-4) a linha final de cada episodio com o feedback do passo final (a nota, o teto do custo, o "
                             "portao do estilo e as medias rastreio_*); sem: o feedback zerado do mundo recomecado, como antes")
    parser.add_argument("--parado-dobrado-encerra", action="store_true",
                        help="(10/11 01h30) parado, o peito alto a mais de 34 graus do idle por 0,5 s (fora do empurrao) encerra como a queda")
    parser.add_argument("--parado-semelhanca", type=float, default=0.0,
                        help="(10/11 00h35, ele: o parado mais parecido com o clipe) no parado livre e no parado da parada livre, o peso "
                             "da semelhanca com o idle (suavidade 2) na qualidade de pe; 0: a de antes (0,20)")
    parser.add_argument("--passo-baixo", type=float, default=0.0,
                        help="(10/11 00h35, ele: sem o 'trotar parado' nas freadas e recuperacoes) nas fases livres, o pe acima da "
                             "altura de repouso alem desta folga (m) custa ate 40%% da nota; 0: sem")
    parser.add_argument("--empurrao-livre", action="store_true",
                        help="(10/11 00h20, ele: semelhanca alta sem interferir nos momentos de sair do clipe) andando, no empurrao "
                             "e 1,5 s depois, sem imitacao: a nota e voltar ao movimento pedido, de pe, o peito na vista")
    parser.add_argument("--pose-reforco", type=float, default=0.0,
                        help="(09/11 19h15, ele: reforçar as poses) andando, a nota vezes 1 - isto x (1 - pose: a coluna, a inclinação "
                             "e a pelve contra o clipe); a coluna inteira na arrancada; no parado, a coluna contra a do idle; 0: sem")
    parser.add_argument("--parado-livre", action="store_true",
                        help="(09/11 17h30, a etapa 3a) no parado da cabeca da locomocao (parado e equilibrio com empurroes), a nota "
                             "do parado da parada livre, sem o gerador de passos e sem o fim por pose perdida")
    parser.add_argument("--parada-pose", type=float, default=0.0,
                        help="(09/11 15h40) na parada livre, ja parado, a nota vezes 1 - isto x (1 - pose de pe: a altura da pelve e o "
                             "peito alto reto contra os do idle); 0: so a parcela somada")
    parser.add_argument("--troca-livre", action="store_true",
                        help="(09/11 15h, a etapa 2) as trocas media, forte e livre e a troca de tecla do comando pelo objetivo: "
                             "por uma janela pelo tamanho da troca, a velocidade nova, o peito na vista e de pe, sem passada nem "
                             "meia-volta para seguir; depois o rastreio do ciclo novo na fase das pernas")
    parser.add_argument("--parada-livre", action="store_true",
                        help="(09/11 14h, ele: frear 'precisa ser emergente do RL') a parada final e o soltar da tecla do comando "
                             "sem referencia que freia nem clipe: a locomocao dirige ate ficar parada de pe e a nota e o objetivo "
                             "(a velocidade a zero, de pe, o peito na vista, a qualidade do parado)")
    parser.add_argument("--parada-clipes-base", action="store_true",
                        help="(10/11, ele: 'ensinar a fazer pelos clipes, para ter base e DEPOIS deixar liberdade') com --parada-clipes e "
                             "--parada-livre: a parada final segue a da Unreal do rumo (onde ha uma boa) e, assentada, a livre")
    parser.add_argument("--parada-variantes", action="store_true",
                        help="(10/11 05h, ele: 've qual dos tres fica melhor') a parada de frente sorteada entre a da Unreal, a tocha e a "
                             "virada sem o giro (Mixamo); a medida escolhe pela variavel PARADA_VARIANTE")
    parser.add_argument("--parada-variantes-mascara", type=int, default=7,
                        help="as variantes da parada de frente no sorteio (bits: 1 Unreal, 2 tocha, 4 virada, 8 Running To Stop)")
    parser.add_argument("--parada-entrada", type=float, nargs=2, default=[0.2, 0.3], metavar=("MISTURA", "JANELA"),
                        help="(10/11 05h25) a entrada no clipe de parada: a passada se mistura nele por MISTURA s e o instante de entrada "
                             "nos primeiros JANELA s dele (0,2 0,3: como antes)")
    parser.add_argument("--arrancada-unreal", action="store_true",
                        help="(10/11, ele: 'a arrancada da corrida... deveria ser a mesma arrancada igual do unreal engine') de pe nos "
                             "retos da corrida, a arrancada da Unreal do rumo (corrida_start_*) antes do ciclo, a velocidade pedida a da raiz "
                             "dela e o ciclo entrando no quadro com as pernas mais parecidas")
    parser.add_argument("--largada-clipe", type=float, nargs=2, default=[0.0, 600.0], metavar=("PARCELA", "ATUALIZACOES"),
                        help="(10/11 07h10, ele: 'executar literalmente o movimento do clipe e reforcar em largada, pelo menos ate ele "
                             "aprender') na janela da largada do treino, os motores com PARCELA do clipe, descendo a 0 em ATUALIZACOES "
                             "(as avaliacoes e as reguas sem ela)")
    parser.add_argument("--largada-mao", type=float, default=0.0,
                        help="(10/11 07h15, aprovado por ele: 'mao so na largada') na janela da largada do treino, a mao na pelve neste nivel, "
                             "descendo a 0 junto com a parcela do --largada-clipe (as avaliacoes e as reguas sem ela)")
    parser.add_argument("--largada-inclinar", type=float, default=0.0, metavar="PESO",
                        help="(14/11 20h40, ele: 'os bracos estabanados na arrancada e os passinhos parado/curtos' - visto: o clipe sai inclinado "
                             "20-26 graus e a rede reta, 0-8) na janela da largada, de pe, um custo por passo de PESO x min(1,5, (inclinacao do "
                             "clipe - a do corpo - 5 graus) / 15 graus): o tronco (peito alto contra a pelve) no rumo do movimento; so o menos "
                             "inclinado custa (build-base-n109); 0: nada")
    parser.add_argument("--largada-velocidade-parcela", type=float, default=0.0, metavar="PARCELA",
                        help="(14/11 22h10, o Codex: pagar a velocidade cedo fora dos multiplicadores de forma) na largada da corrida de frente, "
                             "com a janela da largada como envelope, a nota do rastreio vira (1 - PARCELA x janela) x ela + PARCELA x janela x "
                             "(a velocidade do centro de massa no rumo / max(1, a pedida agora)) (build-base-n110); 0: nada")
    parser.add_argument("--largada-imitar", type=float, default=0.0, metavar="NOTA",
                        help="(15/11 06h30, ele: 'basta ele seguir o clipe da arrancada para correr corretamente') na largada de pe da corrida de "
                             "frente, soma NOTA x a semelhanca do corpo com o clipe da largada no relogio dele (a nota do levantar: pose por partes, "
                             "tronco, maos e pes, centro de massa e a velocidade dele na rampa do clipe) x a janela da largada (build-base-n112); 0: nada")
    parser.add_argument("--largada-no-lugar", type=float, default=0.0,
                        help="(10/11 08h20, ele: 'pelo menos tirar os passinhos no lugar') na largada, cada pe que pousa sem ter andado 15 cm "
                             "no rumo pedido custa ate isto na nota (para tras, o maximo); 0: sem")
    parser.add_argument("--gravidade-motores", action="store_true",
                        help="(10/11 15h25) a compensacao da gravidade nos motores da locomocao (o tonus antigravitacional): o torque que "
                             "segura o peso de cada osso na pose de agora, dentro da forca maxima do motor")
    parser.add_argument("--pes-firmes", type=float, default=0.0,
                        help="(10/11 15h, ele: 'os pes deslizando... eu quero ele estatico quando parado') parado (pedido zero, corpo parado), "
                             "cada pe apoiado deslizando ou girando custa isto vezes min(2, v/5 cm/s + giro/0,5 rad/s); 0: como antes")
    parser.add_argument("--custo-batente", type=float, default=0.0,
                        help="(10/11 14h30, ele: 'pos-freio parece a coluna quebrada') o torque dos batentes da coluna alem de 20 N.m, em "
                             "100 N.m, vezes isto na nota a cada decisao: empurrar o osso contra o fim para se enrijecer custa; 0: como antes")
    parser.add_argument("--tonus-coluna", type=float, default=1.0,
                        help="(10/11 14h25, ele: 'pos-freio parece a coluna quebrada') a rigidez e o amortecimento so da coluna vezes isto "
                             "(o tronco ~150 N.m/rad contra ~250-440 in vivo pre-ativado); 1: como antes")
    parser.add_argument("--postura-cauda", type=float, default=0.0,
                        help="(10/11 14h, ele: 'ao frear e pos-freio ele fica todo molengo') a inclinacao da pelve, do peito e da cabeca "
                             "contra o clipe numa regua de cauda longa (1/(1+(d/10 graus)^2)), nesta parcela da nota do rastreio; 0: como antes")
    parser.add_argument("--tonus", type=float, default=1.0,
                        help="(10/11 13h45, ele: 'as poses ficaram molengas... ele precisa ficar mais rigido') o tonus do corpo: a rigidez e o "
                             "amortecimento de todos os motores vezes isto (a forca maxima igual); 1: como antes")
    parser.add_argument("--ruido-parado", type=float, default=1.0,
                        help="(10/11 12h55, ele: 'no treino de IDLE o corpo e MUITO instavel') o desvio da exploracao parado (pedido zero), "
                             "vezes isto, voltando ao inteiro com a velocidade do corpo ate 0,5 m/s; 1: como antes")
    parser.add_argument("--postura-base", type=float, default=0.0,
                        help="(10/11 11h45, ele: 'quando parado, sempre o IDLE alerta e o alvo... recobrar a pose assim que possivel, com o "
                             "minimo de passos') o idle alerta como atrator com o pedido zero: estar perto (regua que nao satura), o "
                             "progresso a cada decisao com este peso e cada passo de ajuste custando um pouco; 0: como antes")
    parser.add_argument("--coluna-sem-velocidade", action="store_true",
                        help="(10/11 10h40, a coluna acoplada) nos eixos acoplados da coluna, a velocidade-alvo e a do proprio alvo de "
                             "posicao da rede, nao a do canal livre (medido: a rede usava o canal para travar a coluna no batente)")
    parser.add_argument("--largada-bracos-calmos", action="store_true",
                        help="(10/11 08h20, ele: 'e os bracos estabanados na largada') na largada, os bracos da pose do idle a do clipe com a "
                             "velocidade do corpo (cheios a 1,5 m/s)")
    parser.add_argument("--largada-passo", type=float, default=0.0,
                        help="(10/11 07h05, ele: 'o maior ganho de velocidade com o menor numero de passos') na largada, cada pe que pousa paga "
                             "(tempo no ar - 0,15 s) x isto - o passinho perde; 0: sem")
    parser.add_argument("--largada-viva", action="store_true",
                        help="(10/11 06h35, ele: 'nada de passinhos nem lentidao, o IMPULSO real') na largada, o progresso inteiro na velocidade "
                             "e a imitacao pelo quadro mais parecido do clipe de largada (+-0,3 s)")
    parser.add_argument("--arrancada-unreal-escala", type=float, default=0.0,
                        help="(10/11 06h05) a escala fixa de reproducao da arrancada da Unreal (0: a que leva a saida dela a velocidade pedida)")
    parser.add_argument("--rigor-superior", type=float, default=1.0,
                        help="(10/11, ele: 'cobrar a coluna correta, cabeca, bracos com mais rigor durante as locomocoes e parado') a "
                             "regua da imitacao da coluna, da cabeca e dos bracos vezes isto (as pernas e os pes como sempre); 1: sem")
    parser.add_argument("--parada-postura", type=float, default=0.0,
                        help="(10/11, ele: 'conseguiu a freada, mas sacrificou a postura') com --parada-livre: a nota inteira da "
                             "parada vezes 1 - G + G x pose (o peito alto e a cabeca contra os do idle, folgas 5,7 e 8,6 graus); 0: sem")
    parser.add_argument("--parada-portao", type=float, default=0.0,
                        help="(09/11 10h50) freando (a parada final e o soltar da tecla), o portao da velocidade nunca abaixo disto "
                             "(0,6 como no empurrao); 0: como andando")
    parser.add_argument("--parada-clipes-comando", action="store_true",
                        help="(09/11 10h10) com --parada-clipes, o soltar da tecla do comando tambem pelo clipe de parada")
    parser.add_argument("--parada-clipes-frente", action="store_true",
                        help="(09/11 08h55) com --parada-clipes, os clipes de parada so de frente e nas diagonais da frente")
    parser.add_argument("--idle-pose", type=float, default=0.0,
                        help="(09/11 06h15, ele: 'o parado as vezes ainda fica todo estranho, abaixado e torto') no idle, fora do "
                             "empurrao, a nota inteira vezes 1 - F + F x a pose (altura e peito alto contra o idle); 0: nao")
    parser.add_argument("--parada-passada", action="store_true",
                        help="(09/11 05h) na parada final, a passada que desacelera (cadencia com a velocidade, idle abaixo de 0,5 m/s) "
                             "em todos os rumos; sem: so nos laterais")
    parser.add_argument("--troca-giro-tempo", type=float, default=1.0,
                        help="(09/11 04h35) a meia-volta das trocas tocada nesta fracao da velocidade do clipe (0,6: 1,7x mais longa); 1: como ele")
    parser.add_argument("--troca-assentada-lado", type=float, default=0.0,
                        help="de lado (2 e 6), a freada ate esta velocidade (m/s) antes da meia-volta das trocas (06/11 10h50); 0: sem")
    parser.add_argument("--largada-lado", type=float, default=0.0,
                        help="a aceleracao (m/s2) da largada dos rumos de lado 2 e 6 (06/11 08h50: o trote lateral cai na "
                             "largada a 0%%); 0: a de todos (5)")
    parser.add_argument("--arrancada-pulo", type=float, default=0.0,
                        help="na janela da arrancada da corrida, a nota cai em linha reta com a pelve acima da do clipe alem de "
                             "3 cm, ate 60%% a tantos metros alem (06/11 08h25, o salto); 0: sem")
    parser.add_argument("--penhascos-partes", nargs="*", default=None, choices=("curvas", "pe", "antebraco", "pulo"),
                        help="so estas partes da recompensa sem os penhascos (06/11 08h15, o A/B); sem: as de --sem-penhascos")
    parser.add_argument("--sem-penhascos", action="store_true",
                        help="a recompensa sem os penhascos (06/11 07h): a inclinacao de lado nas curvas (regua larga), o "
                             "antebraco nos bracos, e caudas largas no pulo da pelve e no pe cruzado")
    parser.add_argument("--capacidade-minutos", type=float, default=0.0,
                        help="a cada tantos minutos, a capacidade em forca fixa: cada exercicio num teste sempre igual a 0%% "
                             "(e as trocas e o levantar tambem a 15%%), no log como 'capacidade' (06/11 07h); 0: sem")
    parser.add_argument("--inclinacao-cauda", type=float, default=0.0,
                        help="a fracao do fator da inclinacao de lado numa regua larga (0,35 rad): gradiente para a curva "
                             "inclinada para fora (06/11 06h)")
    parser.add_argument("--faixa-velocidade", type=float, default=1.0,
                        help="(07/11, a receita nova) no treino, o pedido fixo de cada rumo vezes um sorteio por episodio "
                             "em [FAIXA, 1]: a rede treina em pedidos alcancaveis e aprende a velocidade como um controle "
                             "continuo; a avaliacao e o teste fixo sempre na meta. 1: o pedido fixo, como antes")
    parser.add_argument("--ruido-velocidade", type=float, default=1.0,
                        help="(07/11, a receita nova) o desvio da exploracao da locomocao nos alvos de velocidade dos "
                             "motores vezes isto, uma vez, e o piso dele tambem (a sonda: o ruido deles derrubava 6-9%% "
                             "dos episodios em linha reta e deixava a rede 10-25%% mais lenta). 1: como antes")
    parser.add_argument("--piso-posicao", type=float, default=0.0,
                        help="(08/11, o plato: a passada longa e a troca forte nunca descobertas - a exploracao da locomocao no "
                             "piso 0,05-0,06) o piso do desvio da locomocao nos alvos de POSICAO, e o desvio levado a ele uma vez; "
                             "a sonda de 07/11: ruido so na posicao nao derruba (98-100%% de pe). 0: o piso de sempre")
    parser.add_argument("--redo-atualizacoes", type=int, default=0,
                        help="(07/11, a sonda da rede: 41%% da 2a camada do ator e 81%% da do critico da locomocao mortas) a "
                             "cada tantas atualizacoes, as unidades que nao ativaram em NENHUMA linha da locomocao da janela "
                             "renascem (ReDo, Sokar et al. 2023: entrada nova, saida zero - a rede faz o mesmo na hora); 0: nunca")
    parser.add_argument("--troca-aceleracao", type=float, default=6.0,
                        help="(07/11, a sonda das trocas) a aceleracao da velocidade de referencia nas manobras, m/s^2 (era 6: a "
                             "forte acabava virada ou caida logo depois da troca)")
    parser.add_argument("--subida-freada", type=float, default=0.0,
                        help="(07/11, ele: o pulinho no fim do levantar) perto de pe, subir alem de 0,35 m/s custa k (vz - 0,35)^2 "
                             "por decisao, com a recuperacao no comando; 0: como antes")
    parser.add_argument("--quietude-parado", type=float, default=0.0,
                        help="(07/11, ele: a quietude do parado recompensada sem afetar o equilibrio) o custo das juntas "
                             "alem de 0,15 rad/s por elo e do sobe-desce do centro de massa no parado, e o peso da quietude "
                             "0,15 -> 0,25; 0: a quietude de antes (sugerido: 3)")
    parser.add_argument("--rumo-cauda", type=float, default=0.0,
                        help="a fracao do fator do rumo numa regua larga (0,6 rad): gradiente para a largada torta (06/11 04h55)")
    parser.add_argument("--corrida-largada", type=float, nargs=3, default=None, metavar=("ESCALA", "SEGUNDOS", "ACELERACAO"),
                        help="a largada progressiva da corrida de frente (06/11 04h40, ele: sem o pulo depois do impulso): o "
                             "impulso nesta escala, a cadencia do ciclo subindo em SEGUNDOS e o pedido a ACELERACAO m/s2")
    parser.add_argument("--troca-pratica", action="store_true",
                        help="as trocas: o rumo de partida pelas quedas de cada rumo (metade por igual), o intervalo 1-2 s e a "
                             "troca que chega no meio da anterior esperando (06/11 01h10)")
    parser.add_argument("--troca-pe", action="store_true",
                        help="a meia-volta das trocas entra no pe certo: espera o pe que o clipe planta primeiro (05/11 21h40)")
    parser.add_argument("--estab-ancora", action="store_true",
                        help="a estabilidade medida no idle fora da janela dos empurroes, a ancora recomecando depois de cada um "
                             "(o portao do levantar; a recompensa nao muda; 05/11 21h10)")
    parser.add_argument("--simetria-curvas", action="store_true",
                        help="a perda de simetria tambem nas curvas (as manobras 5-7), nao nas trocas (05/11)")
    parser.add_argument("--simetria-manobras", action="store_true",
                        help="a perda de simetria tambem nas manobras (as trocas; 05/11)")
    parser.add_argument("--equilibrio", action="store_true",
                        help="(04/11 15h) os passos de recuperacao e estabilizacao nos dois cerebros: as entradas do equilibrio "
                             "(o ponto de captura, a base), o gerador de passos no idle e os canais dele na cabeca da locomocao")
    parser.add_argument("--equilibrio-fatia", type=float, default=0.0,
                        help="a fatia dos comecos de pe com empurroes, sem a mao (o equilibrio parado; o resto e o parado)")
    parser.add_argument("--equilibrio-nivel", type=float, default=0.2,
                        help="o nivel inicial dos empurroes do equilibrio parado (0-1 das faixas --empurrao-forca e do continuo)")
    parser.add_argument("--empurrao-idle", action="store_true",
                        help="o idle (a locomocao, com o passo) segura o empurrao de pe; sem: a recuperacao, como antes")
    parser.add_argument("--chegada-premio", type=float, default=0.0,
                        help="o premio da chegada do levantar equilibrado, vezes a qualidade dela na entrega ao idle")
    parser.add_argument("--idle-alcance-ameaca", type=float, default=0.0,
                        help="(04/11 15h45) no idle, o alcance da correcao cresce do --idle-alcance ate este (fracao do cheio) com "
                             "a ameaca do ponto de captura: a pose quieto, a forca para se equilibrar perto da borda; 0: sem")
    parser.add_argument("--voo-multa", type=float, default=0.0,
                        help="a multa por decisao dos dois pes no ar quase de pe, com a recuperacao (o pulinho do fim do levantar)")
    parser.add_argument("--entrega-calma", type=float, default=0.0,
                        help="depois de levantar, a recuperacao so entrega ao idle com o ponto de captura dentro da base e o corpo "
                             "abaixo de 0,4 m/s - ou este prazo de pe (s); 0: na hora (04/11 21h35)")
    parser.add_argument("--troca-inclinar", type=float, default=0.0,
                        help="nas trocas de direcao (1-4), a referencia inclina para a aceleracao da velocidade pedida, "
                             "atan(a/g), ate este angulo (graus): a frenagem para tras e a arrancada no rumo novo; 0: sem")
    parser.add_argument("--evitar-idle", action="store_true",
                        help="o evitar a queda na cabeca da locomocao (o idle segura o empurrao); a recuperacao so caido "
                             "(04/11 23h10, ele: 'APENAS o levantar-se em cabeca separada')")
    parser.add_argument("--subida-equilibrio", type=float, default=0.0,
                        help="a multa por decisao (ate) do levantar subindo para fora dos pes: o ponto de captura fora da base e a "
                             "velocidade no plano, com a recuperacao e a cabeca acima de 1 m (04/11 16h40: ele subia explosivo, "
                             "passava do ponto e caia de costas)")
    parser.add_argument("--congelar-recuperacao", action="store_true",
                        help="com as habilidades separadas, so a cabeca da locomocao aprende (a da recuperacao fica como veio)")
    parser.add_argument("--preservar-peso", type=float, default=0.0, help="âncora do ator inicial nos exercícios antigos")
    parser.add_argument("--ancora", type=float, default=0.0,
                        help="(10/11, E5) o peso da ancora no campeao: a MSE entre a acao media da cabeca da locomocao e a do "
                             "campeao congelado nos estados do banco fixo (banco_ancora.py); 0: desligada")
    parser.add_argument("--ancora-banco", type=str, default="", help="o banco da ancora (banco_ancora.py)")
    parser.add_argument("--ancora-de", type=str, default="",
                        help="o checkpoint da rede congelada da ancora (padrao: o do banco)")
    parser.add_argument("--locomocao", type=float, default=0.0,
                        help="a fatia dos rumos (andar ou trotar num dos 8 rumos, mantendo o rumo do corpo)")
    parser.add_argument("--giro", type=float, default=0.0, help="a fatia do giro parado suave")
    parser.add_argument("--andar", type=float, default=1.0 / 3.0, help="nos rumos, a fatia do andar (o resto trota)")
    parser.add_argument("--andar-velocidade", type=float, nargs=2, default=[0.8, 1.4], help="(m/s)")
    parser.add_argument("--trote-velocidade", type=float, nargs=2, default=[2.0, 3.0], help="(m/s)")
    # (22h45, ele: "por hora tira os empurroes - so vai entrar depois que ele estiver se locomovendo bem": 0 0 nas duas
    # faixas, sem empurrao; a regua do comeco do B21 e 300 600 e 100 180.)
    parser.add_argument("--empurrao-mov-forca", type=float, nargs=2, default=[0.0, 0.0],
                        help="os empurroes curtos em movimento (N; os rumos e o giro; 0 0: sem)")
    parser.add_argument("--empurrao-mov-continuo", type=float, default=0.3, help="a fatia continua deles")
    parser.add_argument("--empurrao-mov-continuo-forca", type=float, nargs=2, default=[0.0, 0.0],
                        help="(N, por 1-2 s; 0 0: sem)")
    # (01/11, ele: o levantar volta a ser so levantar e evitar quedas, "fazendo a troca de RL no momento que recem fica
    # de pe": o episodio acaba na troca, de pe e firme por 0,5 s - a rede da locomocao assume dai.)
    parser.add_argument("--troca-de-pe", action="store_true", help="o episodio acaba ao ficar de pe e firme (a troca)")
    # (01/11, a interligacao das redes: "para que justamente um consiga aprender com o outro" - as quedas da rede da
    # locomocao, que o treino dela escreve em quedas_locomocao.npy neste treino.)
    parser.add_argument("--quedas-locomocao", type=float, default=0.0,
                        help="a fatia que comeca das quedas da rede da locomocao (quedas_locomocao.npy)")
    # (01/11, ele: "ambas as redes aprendendo em conjunto em todos os treinos... o evitar queda precisa saber operar em
    # conjunto com a outra rede nos momentos de queda iminente": a rede da locomocao exportada do treino dela como
    # parceira - depois da troca ela dirige parada por --parceiro-segundos, e se cair a troca foi ruim; e o evitar a
    # queda com ela de pe levando o empurrao, a rede do levantar assumindo na queda iminente dela.)
    parser.add_argument("--parceiro", default="", help="o treino da rede da locomocao parceira (vazio: sem)")
    parser.add_argument("--parceiro-segundos", type=float, default=2.0)
    parser.add_argument("--parceiro-punicao", type=float, default=10.0)
    parser.add_argument("--evitar-com-parceiro", action="store_true",
                        help="o evitar a queda comeca com a parceira de pe levando o empurrao")
    parser.add_argument("--toque-punicao", type=float, default=0.0,
                        help="nos comecos da queda iminente, a punicao do primeiro toque do tronco no chao")
    # (01/11 03h25, ele: "nao vi nenhum sinal do tentar evitar queda... imaginava que quando ele visse cair no chao
    # tentasse se proteger, levando as maos, joelhos, girando quadril, coluna... para se recobrar antes de cair". Medido:
    # por episodio, o custo do limite das juntas somava ~16 e o do tranco ~7, contra 1,5 da punicao do toque no chao -
    # proteger-se nao valia nada e o reflexo rapido ainda era punido como tranco. Agora, protegendo-se na queda, os custos
    # de suavidade quase somem e o primeiro apoio das maos, bracos ou joelhos vale.)
    parser.add_argument("--apoio-premio", type=float, default=0.0,
                        help="na protecao na queda, o premio do primeiro apoio das maos, bracos ou joelhos (uma vez)")
    parser.add_argument("--protecao-alivio", type=float, default=0.0,
                        help="a fracao dos custos do tranco e do limite tirada enquanto ele se protege na queda")
    # (01/11 04h40, ele, com a imagem do corpo arqueado no chao: "assim que ele cai, ele puxa os bracos e pernas dele
    # para tras... o correto seria ao contrario, para frente": na queda e ao chegar ao chao, a flexao - maos a frente do
    # peito, joelhos a frente da pelve, sem arquear a coluna para tras.)
    parser.add_argument("--flexao", type=float, default=0.0, help="o peso da flexao na queda (bracos e joelhos a frente)")
    # (01/11 04h45, ele: "nossa rede tem dificuldade de levantar-se se o cenario nao e limpo - tem algo esbarrando nele,
    # ou um boneco por cima": as paredes e caixas encostadas e os pesos caindo em cima.)
    parser.add_argument("--obstaculo", type=float, default=0.0, help="a fatia com uma parede ou caixa encostada no corpo")
    parser.add_argument("--locomocao-ambiente", type=float, default=0.0,
                        help="(16/11, foco 5) a fatia da locomocao comandada com 2-5 caixas, paredes e pilares do tamanho de um jogador em volta do caminho (build-base-n130); no jogo, a rede treinada assim le o ambiente de verdade")
    parser.add_argument("--peso-em-cima", type=float, default=0.0, help="a fatia com um peso caindo em cima do corpo")
    # (17/11, o plano de desentortar - Fase 1; build-base-n131) Os consertos do que pagava o torto e a paridade com o jogo (D2).
    parser.add_argument("--referencial-clipe", action="store_true",
                        help="o rolamento do peito, a largura dos pes e o joelho do clipe medidos no eixo de lado DO CLIPE (antes no do corpo)")
    parser.add_argument("--fase-dois-pes", action="store_true", help="a fase do clipe acertada no pouso dos dois pes (antes so no esquerdo)")
    parser.add_argument("--rumo-medio", type=float, nargs="+", default=[0.0],
                        help="PESO [LARGURA rad, 0.10]: nas retas, o rumo medio da pelve (~0,6 s) contra o do clipe derruba a semelhanca")
    parser.add_argument("--forca-contatos", action="store_true",
                        help="a entrada da forca externa soma os contatos que nao sao o apoio (outro corpo, parede) - no jogo tambem (run.json)")
    parser.add_argument("--chao-grama", type=float, default=0.0, help="a fatia dos mundos com o chao de grama do campo (o resto no concreto)")
    parser.add_argument("--reta-comando", action="store_true",
                        help="os fatores da reta (peito de lado, rumo medio, pernas, largura, pe cruzado) tambem no comando numa reta (antes so na manobra 0)")
    parser.add_argument("--simetria-comando", action="store_true",
                        help="a perda de simetria tambem nas linhas do comando (a manobra 12) com o clipe de ciclo - o modo do jogo")
    parser.add_argument("--ancora-sem-locomocao", action="store_true",
                        help="a ancora do principal (--levantar-imitar-professor-ancora) so nos levantares: a locomocao livre dela (o modo de ensino fica)")
    parser.add_argument("--loco-saidas-vivas", action="store_true",
                        help="nas linhas da marcha (clipe de ciclo) as 25 saidas sem efeito ficam fora do log da probabilidade (sonda_saidas_mortas.py)")
    parser.add_argument("--lado-medio", type=float, nargs="+", default=[0.0],
                        help="(Fase 2) PESO [LARGURA m, 0.03]: nas retas, o lado medio (~0,6 s) da cabeca, do peito alto e do meio dos pes contra o clipe")
    parser.add_argument("--imitacao-portao", type=float, nargs="+", default=[0.0],
                        help="(Fase 2) G [CHEIA, 0.6]: andando, a nota inteira vezes (1 - G) + G x min(1, semelhanca / CHEIA)")
    parser.add_argument("--velocidade-rotacao", action="store_true",
                        help="(17/11, o Codex: os bracos abertos) a velocidade-alvo do clipe nos bracos e na cabeca por junta, como o Jolt a quer "
                             "(a velocidade angular no referencial do filho), e nao a derivada de cada coordenada (build-base-n132; run.json)")
    parser.add_argument("--imitar-velocidade", action="store_true",
                        help="(17/11) na imitacao supervisionada (--imitar) as saidas de velocidade seguem a velocidade do clipe por junta (antes "
                             "eram puxadas para zero, freando os motores) e os canais do passo ficam fora (build-base-n133)")
    parser.add_argument("--projeteis", type=float, nargs="+", default=[0.0],
                        help="(17/11, Fase 3) TAXA [IMPULSO_MIN IMPULSO_MAX SEGUNDOS]: na locomocao de pe, projeteis pequenos e frequentes (DReCon) - "
                             "por segundo, um impulso de 10-40 N.s num elo sorteado, em 0,06 s (build-base-n134)")
    parser.add_argument("--ancora-parado", action="store_true",
                        help="(17/11 09h) com --ancora-sem-locomocao: a ancora tambem nas linhas de pe sem pedido (o parado e a freada) - o equilibrio "
                             "parado da ancora (a prova de fogo do parado caiu sem ela)")
    parser.add_argument("--ancora-fogo", type=float, default=0.0,
                        help="(17/11 11h) com --ancora-sem-locomocao: as linhas da locomocao com forca de fogo (fogo.forca.* != 0) agora ou nos ultimos "
                             "S segundos voltam para a ancora r130240 - a reacao a empurrao dela (a prova de fogo caiu ~5 de 71 com a ancora fora da "
                             "locomocao, no A e no C); a forma sem fogo segue livre. 0 desliga")
    parser.add_argument("--teto-custo-tarefa", action="store_true",
                        help="o teto dos custos da locomocao e a parte da tarefa (peso x nota), nao a nota inteira - com ele, melhorar a nota nunca tira premio")
    # (02/11, ele: "parede ao lado, paredes aos lados, pilares, corpos ragdoll embaixo dele... e tambem em cima dele, em
    # terrenos inclinados": as fatias de cada ambiente, sorteadas uma a uma.)
    parser.add_argument("--ambientes", type=float, nargs=6, default=[0.0] * 6,
                        metavar=("CORREDOR", "PILARES", "RAMPA", "ACIDENTADO", "CORPOS_EMBAIXO", "CORPO_EM_CIMA"),
                        help="as fatias do corredor, dos pilares, da rampa, do chao acidentado, dos corpos embaixo e do corpo em cima")
    parser.add_argument("--evitar-ambientes", type=float, default=0.0,
                        help="a fatia dos episodios do evitar a queda com um ambiente garantido (a variante ambientada)")
    # (01/11, ele: o levantar do chao "com gestos mais sutis e suaves, menos exagerados, sem o boneco ficar distorcido ou
    # fazer coisas inumanas"; a velocidade deixa de ser o foco. O discriminador separava tudo - clipes +0,90, a rede
    # -0,90 - e a nota de estilo ficava em ~0,08 para qualquer movimento: o gp maior o suaviza. E o custo dos alvos perto
    # do limite das juntas - a media de max(0, |a| - 0,8)^2 na acao media, na faixa de cada junta: 7% dos alvos estavam
    # alem de 90% da faixa.)
    parser.add_argument("--disc-gp", type=float, default=5.0, help="o peso do gradient penalty do discriminador")
    parser.add_argument("--limite", type=float, default=0.0, help="o peso do custo dos alvos perto do limite das juntas")
    parser.add_argument("--arrancada", type=float, default=0.0,
                        help="a fatia que comeca na arrancada agachada (crouched_to_sprint) virado para um alvo")
    # (01/11, a rl_final - ele: "precisamos achar uma abordagem que consigamos fazer ela se locomover! e BEM, com
    # estabilidade e velocidade e excelente auto-equilibrio"; vault 04_locomocao/plano_rl_final_01_11.md. Em 31/10 ela
    # nao deu um passo: estilo 0,7 - ficar parado em alerta ja agradava o juiz -, o custo do tranco punindo o balanco
    # rapido da perna, comecos parados e cair sem fim de episodio. Agora, nos rumos: as faixas da corrida e do sprint,
    # os comecos ja no clipe, a queda encerrando, o pedido trocando no meio e os pesos proprios da locomocao.)
    parser.add_argument("--corrida", type=float, default=0.0, help="nos rumos, a fatia da corrida")
    parser.add_argument("--sprint", type=float, default=0.0, help="nos rumos, a fatia do sprint")
    parser.add_argument("--sprint-velocidade", type=float, default=0.0,
                        help="o pedido fixo do sprint (m/s; so para a frente, no sprint_maximo largando de pe); 0: o do clipe")
    parser.add_argument("--sprint-inclinar", type=float, nargs=2, default=(0.0, 0.0), metavar=("GRAUS", "PERNAS"),
                        help="no sprint, a referencia inclinada para a frente (graus, a pelve e a coluna retas) com esta fracao "
                             "nas pernas - 05/11, ele: 'o corpo todo tombado para a frente' (a imagem CORRETA)")
    parser.add_argument("--locomocao-clipe", type=float, default=0.5,
                        help="nos rumos, a fatia que comeca ja no clipe do passo, na velocidade dele (o RSI)")
    parser.add_argument("--locomocao-segundos", type=float, default=0.0, help="o tempo dos rumos e do giro (s; 0: 6)")
    parser.add_argument("--queda-encerra", action="store_true", help="nos rumos e no giro, cair encerra o episodio")
    parser.add_argument("--peso-tarefa-loco", type=float, default=None, help="o peso da tarefa na locomocao")
    parser.add_argument("--peso-estilo-loco", type=float, default=None, help="o peso do estilo na locomocao")
    parser.add_argument("--suave-loco", type=float, default=None, help="o peso do custo do tranco na locomocao")
    parser.add_argument("--passo", type=float, default=0.0,
                        help="o premio do passo (o tempo no ar de cada pe que pousa, com o pedido de deslocamento)")
    parser.add_argument("--mistura-clipe", type=float, default=1.0,
                        help="na locomocao, o alvo = clipe x (1 - isto) + rede x isto (1: a rede livre)")
    parser.add_argument("--queda-programada", action="store_true",
                        help="a queda sem volta e a da locomocao fisica ate assentar (sem a rede); dai a rede levanta")
    parser.add_argument("--rolar-clipe", type=float, default=1.0,
                        help="deitado de costas, o rolamento do clipe de levantar de costas nos motores: alvo = clipe x (1 - isto)"
                             " + rede x isto (1: desligado)")
    parser.add_argument("--levantar-guiado", action="store_true",
                        help="deitado, o clipe de levantar mais parecido guia (nos motores e na imitacao); de pe, o idle")
    parser.add_argument("--levantar-clipe", action="store_true",
                        help="o levantar pelo clipe (05/11, ele: 'vamos fazer seguir o clipe para o levantar'): deitado, o clipe "
                             "de levantar inteiro nos motores, a cabeca da locomocao corrigindo e a mao na forca do exercicio "
                             "lev_levantar; implica --levantar-guiado no tempo do clipe")
    parser.add_argument("--levantar-segundos", type=float, default=1.7,
                        help="o levantar guiado: o clipe de levantar acelerado para durar isto")
    parser.add_argument("--estilo-chao", type=float, default=None,
                        help="o peso do estilo com a pelve abaixo de --chao-altura (a tarefa fica 1 - isto): o virar-se no chao")
    parser.add_argument("--chao-altura", type=float, default=0.6, help="a altura da pelve (m) abaixo da qual vale --estilo-chao")
    parser.add_argument("--so-clipe", default="", help="nos rumos, so os passos deste clipe (o imitador de um clipe)")
    parser.add_argument("--acao-velocidade", action="store_true",
                        help="(06/11 17h40, liberdade total nos motores) a rede da tambem a velocidade-alvo de cada eixo; a "
                             "professora copiada inteira, com a velocidade do clipe")
    parser.add_argument("--levantar-velocidade", type=float, default=None, metavar="DESVIO",
                        help="(06/11 17h55, ele: a autonomia da velocidade dos motores tambem no levantar) a cabeca da recuperacao "
                             "usa as velocidades da acao; o desvio delas ao ligar (uma vez); com --preservar-peso a ancora so nas "
                             "posicoes e nos canais")
    parser.add_argument("--manobras-pesos", type=float, nargs=7, default=None,
                        help="(06/11 22h10) os pesos do sorteio das manobras: troca suave, media, forte, livre, curva suave, S, fechada")
    parser.add_argument("--juiz-mm", action="store_true",
                        help="(07/11) o juiz da locomocao com o Motion Matching (os bancos densos do Game Animation Sample, "
                             "tools/rl/juiz_mm_bancos.json): o contexto entra no juiz e o lado real e o quadro casado; precisa "
                             "--juiz-unreal e --estilo-loco-proprio; na primeira vez o juiz da locomocao recomeca do zero")
    parser.add_argument("--levantar-unreal", type=float, default=0.0,
                        help="(11/11 14h) a fatia dos episodios no levantar pelo clipe: deitado de costas ou de brucos, parado ou "
                             "correndo, seguindo no tempo o clipe da Unreal corrigido (LEVANTAR_UNREAL), ancorado no mundo; no fim "
                             "do clipe, de pe, a troca para a locomocao, que comanda por --levantar-unreal-depois s; as avaliacoes "
                             "lu_costas_, lu_brucos_, lu_correndo_costas_ e lu_correndo_brucos_")
    parser.add_argument("--levantar-unreal-meio", type=float, default=0.5,
                        help="(11/11) a fracao dos episodios do levantar pelo clipe que comeca num instante sorteado do clipe (o RSI)")
    parser.add_argument("--levantar-unreal-depois", type=float, default=1.5,
                        help="(11/11) os segundos da locomocao depois da troca no levantar pelo clipe (cair neles encerra)")
    parser.add_argument("--levantar-unreal-ruido", type=float, default=0.2,
                        help="(11/11) a variacao da coluna e das pernas no comeco deitado (rad)")
    parser.add_argument("--levantar-unreal-largura", type=float, default=1.0,
                        help="(11/11) os nucleos da nota do DeepMimic divididos por isto (1: os do artigo)")
    parser.add_argument("--levantar-unreal-vel", type=float, default=1.5,
                        help="(11/11) a largura da nota da velocidade do centro de massa contra a do clipe (m/s)")
    parser.add_argument("--calibrar-levantar", type=str, default="",
                        help="(11/11) so a calibracao do levantar pelo clipe: os jeitos de agir (o professor nos motores, com ruido, "
                             "solto no meio, deitado parado, a acao do meio, a rede de agora) nas avaliacoes lu_*, gravados neste "
                             "JSON; nada treina e nada e salvo")
    parser.add_argument("--levantar-unreal-variante", action="store_true",
                        help="(11/11 18h05) a entrada levantar.variante (LEVANTAR_UNREAL_VARIANTS: 1 no de costas rolando) - sem ela os "
                             "dois de costas comecavam iguais e o rolando imitava o normal; a coluna nova entra no checkpoint com peso zero")
    parser.add_argument("--levantar-unreal-partes", action="store_true",
                        help="(11/11 17h40, ele: reforcar a semelhanca com o clipe, em especial o levantar de costas) a nota do levantar "
                             "pelo clipe com a pose e maos e pes metade do corpo inteiro e metade por parte, e mais peso na semelhanca "
                             "(TrackingClipGetUp3D::Widths::parts); os graficos seguem na regua de antes")
    parser.add_argument("--desvio-recuperacao", type=float, default=None,
                        help="(11/11 20h40, ele: o levantar rolando com boa exploracao para desenvolver o movimento) o desvio da acao da "
                             "cabeca da recuperacao nos eixos das juntas, uma vez por valor (guardado nos contadores; os reinicios nao "
                             "repetem): o desvio dela tinha caido a 0,09 - pouco para achar um rolamento")
    parser.add_argument("--levantar-unreal-reforco", type=float, nargs=2, default=(0.0, 200), metavar=("ALVO", "ATUALIZACOES"),
                        help="(12/11 02h50, ele: forcar melhor a semelhanca do levantar de costas com o clipe) o reforco da pose do tronco e "
                             "dos bracos so no de costas (TrackingClipGetUp3D::Widths::emphasis), subindo de 0 ao ALVO em ATUALIZACOES "
                             "(o comeco guardado nos contadores; 0: nada)")
    parser.add_argument("--levantar-unreal-reforco-correndo", type=float, default=0.0, metavar="ALVO",
                        help="(14/11 20h45, ele: 'cobrar mais semelhanca com o clipe para os nossos levantares correndo') o mesmo reforco da "
                             "pose do tronco e dos bracos do --levantar-unreal-reforco nos dois correndo (de costas e de brucos), com a rampa "
                             "propria (as ATUALIZACOES do --levantar-unreal-reforco desde a primeira com ele); 0: nada")
    parser.add_argument("--levantar-unreal-reforco-brucos", type=float, default=0.0, metavar="ALVO",
                        help="(14/11 20h55, ele: 'melhorar a semelhanca com o clipe de todos os levantares, hoje muito moles e diferentes') o "
                             "mesmo reforco da pose do tronco e dos bracos no de brucos de pe, com a rampa propria; 0: nada")
    parser.add_argument("--levantar-unreal-agilidade", type=float, nargs=3, default=(0.0, 0.0, 100), metavar=("PARADO", "CORRENDO", "ATUALIZACOES"),
                        help="(12/11 03h20, ele: recompensar a velocidade e a agilidade de levantar, e no correndo o impulso) o relogio do "
                             "clipe acompanha o corpo (ate 1,75x) e o premio e o peso x a nota x o adiantamento; o peso dos dois de pe e o "
                             "dos dois correndo, subindo ao alvo em ATUALIZACOES (o comeco nos contadores; 0 0: o relogio so com o tempo)")
    parser.add_argument("--levantar-adaptativo", action="store_true",
                        help="(12/11 04h40, ele: a fase de fogo do levantar, '100%% adaptativo, o clipe perde tanto rigor') os exercicios do "
                             "clipe sem o rastreio na nota: a do levantar livre (o potencial da subida, de pe, os custos; correndo, o "
                             "deslocamento pedido) e o juiz no estilo (--peso-estilo), sem o longe; a troca quando firme")
    parser.add_argument("--levantar-adaptativo-velocidade", type=float, default=0.0,
                        help="(12/11 05h00, ele: continuar a recompensa da velocidade e da agilidade) no levantar adaptativo, na entrega "
                             "(depois de firme): isto x a fracao do tempo do clipe ate o ponto de entrega que ele poupou")
    parser.add_argument("--levantar-tentar", action="store_true",
                        help="(12/11 08h30, ele: \"tentar se levantar novamente, APOS fracassar... (nunca desistir)\") no levantar adaptativo, o "
                             "fracasso abre uma nova tentativa (o relogio do clipe ao comeco, no lado em que ele esta; o orcamento recomeca)")
    # (12/11 12h30, a fase 2 - ele: "deixo tudo a seu criterio a elevacao da fase de fogo") As adversidades novas (so no treino).
    parser.add_argument("--levantar-fogo-serie", type=int, default=1,
                        help="ate quantas pancadas por episodio com pancada, antes de ficar de pe (1: so uma, como antes)")
    parser.add_argument("--levantar-fogo-serie-intervalo", type=float, nargs=2, default=(0.4, 1.2), metavar=("MIN", "MAX"),
                        help="(13h05) o intervalo da serie: a pancada seguinte MIN-MAX s depois do fim da anterior")
    parser.add_argument("--levantar-fogo-pancada-duracao", type=float, nargs=2, default=(0.1, 0.2), metavar=("MIN", "MAX"),
                        help="(13h30) a duracao da pancada fora da prova (s): o mesmo empurrao com menos pico")
    parser.add_argument("--levantar-fogo-pancada-newtons", type=float, nargs=2, default=(0.0, 0.0), metavar=("MIN", "MAX"),
                        help="(13h40, ele: 'as medias de 350-800') a forca da pancada fora da prova sorteada entre MIN e MAX N; 0 0: pelo nivel")
    parser.add_argument("--levantar-fogo-congelar", action="store_true",
                        help="(13h30, ele: 'nao precisamos intensificar mais') a escada do fogo nao sobe; a guarda do limpo e o travado podem descer")
    parser.add_argument("--levantar-fogo-rasteira", type=float, default=0.0, help="a fracao das pancadas nas pernas (coxa, canela, pe)")
    parser.add_argument("--levantar-fogo-prensado", type=float, nargs=3, default=(0.0, 0.5, 3.0), metavar=("FRACAO", "MIN", "MAX"),
                        help="a fracao dos episodios com o continuo com uma parede do lado para onde ele empurra, a MIN-MAX m")
    parser.add_argument("--levantar-fogo-vento", type=float, nargs=2, default=(0.0, 1.5), metavar=("FRACAO", "SEGUNDOS"),
                        help="a fracao dos episodios com o continuo em que ele troca de rumo a cada ~SEGUNDOS")
    parser.add_argument("--levantar-fogo-depois-de-pe", type=float, default=0.0,
                        help="a fracao dos episodios com pancada que leva mais uma logo depois de ficar de pe (na janela de firmeza)")
    parser.add_argument("--levantar-fogo-escorregadio", type=float, default=0.0,
                        help="a fracao dos episodios com fogo sobre a grama molhada (placa de 20 x 20 m)")
    parser.add_argument("--levantar-fogo-agarrao", type=float, nargs=2, default=(0.0, 0.3), metavar=("FRACAO", "PESO"),
                        help="a fracao dos episodios com fogo com um agarrao numa mao (PESO x o peso, 0,5-1,5 s, antes de ficar de pe)")
    parser.add_argument("--levantar-fogo-faixa", type=float, nargs=2, default=None, metavar=("BAIXO", "ALTO"),
                        help="a escada do fogo segura o levantou nesta faixa (a calibracao da fase 2: 0.30 0.50); sem: a escada de dominio")
    parser.add_argument("--levantar-paredes", type=float, nargs=2, default=(0.0, 0.0), metavar=("MIN", "MAX"),
                        help="(12/11 09h15, ele: paredes a ~20-50 m do nascimento so nos treinos de levantar) no levantar pelo clipe, quatro "
                             "paredes em quadrado a uma distancia sorteada por episodio entre MIN e MAX m; 0 0: sem")
    parser.add_argument("--levantar-fogo-contra-corrida", type=float, default=0.0,
                        help="(12/11 08h45) nos correndo, a parte da forca continua do fogo contra a corrida limitada a isto x o peso (0: sem)")
    parser.add_argument("--levantar-fogo-soltar", type=float, default=0.15,
                        help="(12/11 07h55) o soltar da forca continua do fogo depois da entrega (s, linear ate zero)")
    parser.add_argument("--levantar-tempo-alvo", type=float, nargs=2, default=(0.0, 0.0), metavar=("MIN", "MAX"),
                        help="(12/11 13h45, ele: os tempos limpos em 1-1,2 s, nem mais rapido nem mais lento) o premio da velocidade vira o da "
                             "faixa: inteiro com o tempo da tentativa entre MIN e MAX s; 0 0: o premio de antes (mais rapido, mais premio)")
    parser.add_argument("--levantar-tempo-alvo-correndo", type=float, nargs=2, default=(0.0, 0.0), metavar=("MIN", "MAX"),
                        help="(13/11 07h45, ele: de costas e de brucos em 1,3-1,4 s, os correndo em 1,2 s) a faixa do tempo dos correndo a "
                             "parte; 0 0: a de --levantar-tempo-alvo para todos")
    parser.add_argument("--levantar-fogo-pancada-cedo", type=float, nargs=2, default=(0.0, 1.0), metavar=("FRACAO", "JANELA"),
                        help="(13/11 09h35, ele: a maioria das pancadas no primeiro segundo, enquanto ainda levanta) a fracao dos episodios "
                             "com a primeira pancada pelo tempo, de 0,05 s a JANELA s; o resto pela altura da cabeca; 0: o sorteio de antes")
    parser.add_argument("--levantar-tempo-queda", type=float, default=0.5,
                        help="(13/11 07h55) em quantos segundos fora da faixa do tempo o premio dela zera (o de antes: 0,5)")
    parser.add_argument("--parado-quadril-nivel", type=float, default=0.0, metavar="PESO",
                        help="(13/11 14h, ele: no parado 'o quadril dele retinho alinhado'; medido -14,9 graus de lado, o clipe 0) no parado, "
                             "um custo por passo de PESO x (1 - e^-(inclinacao de lado da pelve / 6 graus)^2); 0: nada")
    parser.add_argument("--levantar-ritmo", type=float, nargs=2, default=(0.0, 1.5), metavar=("PESO", "SEGUNDOS"),
                        help="(13/11 17h45, ele: normalizar o tempo de levantar ~1,5 s, o de costas pelo menos 1,2) no levantar adaptativo limpo de "
                             "pe, ate a entrega, custa por passo a cabeca acima da do clipe no ritmo de SEGUNDOS ate a entrega (margem 5 cm, "
                             "escala 25 cm); precisa de --levantar-forma; 0: sem")
    parser.add_argument("--levantar-forma-mao", type=float, default=0.0, metavar="PESO",
                        help="(13/11 13h30, ele: sem melhora na mao virada; dentro da forma ela custava ~2 por levantar) com --levantar-forma, a "
                             "mao apoiada virada com peso proprio: por passo, PESO x (1 - nota da mao); 0: dentro da nota da forma")
    parser.add_argument("--levantar-fogo-soltar-firme", action="store_true",
                        help="(13/11 12h55, com a troca direta: sob fogo 42-51%% caiam 0,4-1 s depois da entrega, inclinados contra o continuo que "
                             "sumia ja no idle) o soltar do continuo (--levantar-fogo-soltar s) comeca no primeiro firme de pe, nao na entrega")
    parser.add_argument("--levantar-forma", type=float, default=0.0, metavar="PESO",
                        help="(13/11 11h30, ele: 'essa cabeca e peito torto quando levanta') na subida e na firmeza do levantar adaptativo, "
                             "um custo por passo de PESO x (1 - forma): a cabeca contra o peito (para tras, de lado) e a torcao do peito contra "
                             "a pelve, contra a media do clipe na mesma altura da cabeca, com folga de 8 graus; 0: nada")
    parser.add_argument("--levantar-forma-perna", type=float, nargs=2, default=(0.0, 1.0), metavar=("PESO", "MASCARA"),
                        help="(14/11 20h10, ele: 'o mesmo metodo... para trazer a semelhanca com o clipe' na perna do de costas) a forma da "
                             "perna esquerda como a da cabeca e do peito: limpo, na subida e na firmeza, nos exercicios da MASCARA (bit e = "
                             "exercicio e; 1: o de costas), um custo por passo de PESO x min(1,5, (erro - 5 cm) / 30 cm); o erro e o joelho e o "
                             "pe esquerdos contra a pelve, contra o clipe na mesma altura da cabeca; 0: nada")
    parser.add_argument("--levantar-tempo-fogo", type=float, default=-1.0, metavar="FOLGA",
                        help="(13/11 10h55, medido: sob fogo o levantar leva 0,2-0,8 s a mais, e a faixa nos dois prendia o tempo no meio) "
                             "no levantar com fogo a faixa vira so um teto, o maximo dela + FOLGA s, sem piso; o ritmo fica pelos limpos; "
                             "< 0: a faixa nos dois (o de antes)")
    parser.add_argument("--levantar-calma", type=float, default=0.0,
                        help="(13/11 07h45, ele: a postura nao estabanada e o auto-equilibrio no pos levantar) no levantar de pe, de pe com a "
                             "recuperacao no comando: a nota de ficar de pe vira (1 - isto) x ela + isto x a calma (centro de massa parado, peito "
                             "sem girar, captura na base, os dois pes no chao); 0: sem")
    parser.add_argument("--levantar-impulso-corrida", type=float, default=0.0,
                        help="(13h45) nos correndo, na entrega: isto x a velocidade ao alvo de 60%% (zero) a 100%% (inteiro) do pedido")
    parser.add_argument("--estilo-limpo", type=float, default=None,
                        help="(15h20, ele: 'no limpo e interessante que o juiz de estilo recompense adequadamente') o peso do estilo nas linhas "
                             "do levantar sem forca externa (a entrada do fogo zerada: o limpo e a firmeza depois da forca soltar); sem: o de sempre")
    parser.add_argument("--levantar-troca-calma", type=float, nargs=3, default=(0.0, 1.5, 1.8), metavar=("VEL", "MAX", "MAX_CORRENDO"),
                        help="(15h05, a analise: sob fogo 40-80 por cento caiam depois de ficar de pe, na troca por tempo fixo) a troca para a "
                             "locomocao so quando calmo (de pe: centro de massa < VEL m/s; correndo: ereto e >= 80 por cento do pedido), no "
                             "maximo MAX s depois da entrega (correndo, MAX_CORRENDO); 0: por tempo, como antes")
    parser.add_argument("--levantar-pose-variada", type=float, default=0.0,
                        help="(14h30, ele: levantar independente da posicao) a fracao dos comecos deitados com a pose girada 60-180 graus no "
                             "comprido do corpo; o clipe pelo lado em que ele ficou")
    parser.add_argument("--levantar-velocidade-corrida", type=float, nargs=2, default=(0.0, 0.0), metavar=("PESO", "JANELA"),
                        help="(14h05, ele: premiar a velocidade da corrida pos-levantar) nos correndo, depois de ficar de pe e antes da troca: "
                             "PESO x a velocidade ao alvo sobre a pedida a cada decisao; JANELA: a janela da rede do levantar nos correndo (s)")
    parser.add_argument("--levantar-adaptativo-firmeza", type=float, default=0.0,
                        help="(12/11 07h50) no levantar adaptativo, a rede do levantar segue no controle por isto (s) depois da entrega antes "
                             "da troca (a entrega e a forca parando seguem no de pe); 0: a troca na entrega")
    parser.add_argument("--levantar-fogo", type=float, nargs=4, default=(0.0, 0.0, 0.0, 0.0),
                        metavar=("LIMPO", "PANCADA", "CONTINUO", "COMBINADO"),
                        help="(12/11 04h40) as fatias dos cenarios do levantar com fogo (vault plano_fogo_do_levantar_12_11); 0: sem fogo")
    parser.add_argument("--levantar-fogo-entrada", action="store_true",
                        help="(12/11 06h40, ele: 'forcar a adaptacao inteligente a conseguir enfrentar as adversidades') as 6 entradas do "
                             "fogo antes da habilidade: a forca externa no corpo (em pesos) e a aceleracao do centro de massa (em g), no rumo "
                             "do corpo; na primeira vez, colunas em zero (a rede faz o mesmo de antes)")
    parser.add_argument("--levantar-fogo-de-cima", type=float, default=0.0,
                        help="(12/11 05h15, ele: 'algumas variantes que vem de CIMA para baixo, mesmo que poucas') a fracao dos cenarios com "
                             "fogo em que a forca vem de cima, inclinada 45-80 graus para baixo (a pancada no alto do tronco)")
    parser.add_argument("--levantar-fogo-niveis", type=str, default="",
                        help="(12/11 04h40) o JSON dos niveis de partida (a F0: a forca que hoje faz falhar ~100%%), uma vez nos contadores")
    parser.add_argument("--levantar-rastrear", type=int, default=0, metavar="MASCARA",
                        help="(14/11 v1) bit k: o levantar limpo do exercicio k (0 de costas, 1 de brucos, 2 e 3 correndo) pelo rastreio do "
                             "clipe (a nota e a semelhanca, o fim quando longe - o levantar de 11/11) mesmo com --levantar-adaptativo; sob "
                             "fogo e no comando seguem adaptativos; 0: nada (build-base-n108)")
    parser.add_argument("--levantar-costas-rola", nargs="*", default=None, metavar="CLIPE_ENTREGA",
                        help="(14/11 v1) o levantar de costas de pe pelo clipe que rola (o rolar do correndo de costas + o levantar de "
                             "brucos, levantar_costas_rola) ou por CLIPE com ENTREGA (s); o correndo de costas como esta (build-base-n107)")
    parser.add_argument("--levantar-brucos-marca", type=float, default=0.0, metavar="VALOR",
                        help="(14/11 17h40) a marca do de brucos de pe na entrada levantar.variante (negativa, ex. -0.4; a base, o ramo, o "
                             "critico e a ancora a veem zerada); 0: sem")
    parser.add_argument("--levantar-brucos-enxerto", default=None, metavar="CHECKPOINT",
                        help="(14/11 17h40) o especialista do de brucos (CHECKPOINT) congelado dentro do cerebro, como o do de costas "
                             "(precisa de --levantar-brucos-marca e --levantar-costas-enxerto)")
    parser.add_argument("--levantar-brucos-clipe", nargs=2, default=None, metavar=("CLIPE", "ENTREGA"),
                        help="(14/11 17h35) o levantar de brucos de pe por outro CLIPE com ENTREGA (s): levantar/levantar_brucos_rapido133 1.654")
    parser.add_argument("--levantar-costas-enxerto", default=None, metavar="CHECKPOINT",
                        help="(14/11 17h) o professor do de costas (CHECKPOINT) inteiro e congelado dentro do cerebro: a acao dele no de costas "
                             "levantando sem empurrao (precisa de --levantar-costas-marca)")
    parser.add_argument("--levantar-costas-clipe", nargs=2, default=None, metavar=("CLIPE", "ENTREGA"),
                        help="(14/11 v1) o levantar de costas de pe por outro CLIPE com ENTREGA (s), sem o lado no quadro de agora do "
                             "--levantar-costas-rola (o mesmo levantar mais rapido: levantar/levantar_costas_rapido 1.45)")
    parser.add_argument("--levantar-unreal-pesos", type=float, nargs=4, default=None, metavar=("COSTAS", "BRUCOS", "CORR_COSTAS", "CORR_BRUCOS"),
                        help="(14/11 v1, o especialista do de costas) os pesos fixos do sorteio dos 4 exercicios do levantar pelo clipe "
                             "(sem --levantar-unreal-prioridade); sem: os quatro iguais")
    parser.add_argument("--levantar-unreal-prioridade", type=float, default=0.0,
                        help="(11/11 16h25) > 0: o sorteio dos 4 exercicios do levantar pelo clipe pela dificuldade - o peso de cada um e "
                             "este piso mais a fracao que nao entrega (ultimos episodios do treino); 0: os quatro iguais")
    parser.add_argument("--congelar-locomocao", action="store_true",
                        help="(11/11) com as habilidades separadas, so a cabeca da recuperacao aprende; a da locomocao, o critico "
                             "dela, o juiz dela e a normalizacao dela ficam como vieram")
    parser.add_argument("--cerebro-unico", action="store_true",
                        help="(13/11, ele: 'um brain so para tudo... a locomocao tambem aprende a se levantar') com as habilidades separadas, "
                             "a cabeca da locomocao comanda tudo - ator, critico, normalizacao e desvio dela em todas as linhas; a ultima "
                             "entrada vira a fase (0 levantando, 1 de pe). Na primeira vez: a normalizacao dela vira a mistura das duas "
                             "(--cerebro-unico-mistura) com a primeira camada compensada (a locomocao sai igual) e o critico aquece "
                             "(--aquecer-critico); depois ela fica parada. Ao salvar, a cabeca da recuperacao vira copia dela (o laboratorio "
                             "e as sondas rodam o cerebro unico sem mudar nada)")
    parser.add_argument("--cerebro-unico-peso-levantar", type=float, default=0.0,
                        help="(13/11 15h20) com --cerebro-unico, a fracao da perda do PPO das linhas do levantar (as de pe com o resto); "
                             "0: pela contagem das linhas (~17%%)")
    parser.add_argument("--cerebro-unico-imitar", type=float, nargs=2, default=(0.0, 300), metavar=("PESO", "ATUALIZACOES"),
                        help="(13/11 15h45) com --cerebro-unico, nas linhas do levantar, a media da acao puxada (MSE, so os eixos das juntas) "
                             "para os alvos de motor do clipe uma decisao a frente, o peso caindo de PESO a zero em ATUALIZACOES a partir da "
                             "primeira vez (contadores); enquanto imita, a fase do levantar nao freia pelo KL e a taxa fica parada")
    parser.add_argument("--cerebro-unico-imitar-taxa", type=float, default=0.0,
                        help="(13/11 15h42) a taxa do ator da cabeca unica enquanto --cerebro-unico-imitar imita (0: a que estava, parada); "
                             "depois o ajuste pelo KL do levantar segue dela")
    parser.add_argument("--cerebro-unico-destilar", type=float, nargs=2, default=(0.0, 500), metavar=("PESO", "ATUALIZACOES"),
                        help="(13/11 16h, ele escolheu: copiar o cerebro do levantar) com --cerebro-unico, o kickstarting: o cerebro unico age e "
                             "o professor de duas cabecas (--cerebro-unico-professor) diz o que faria em cada estado - a cabeca do levantar no "
                             "chao, a da locomocao de pe; a perda da acao media (MSE, sem os canais do passo no levantar), metade de cada fase, "
                             "soma-se a do PPO com o peso caindo de PESO a zero em ATUALIZACOES desde a primeira vez; enquanto ensina, a fase do "
                             "levantar nao freia pelo KL e a taxa e a --cerebro-unico-destilar-taxa")
    parser.add_argument("--cerebro-unico-destilar-puro", type=int, default=0,
                        help="(13/11 16h20) com --cerebro-unico e --cerebro-unico-professor, as primeiras N rodadas so de copia (DAgger): o aluno "
                             "age, os estados das ultimas --cerebro-unico-destilar-reserva rodadas recebem a acao media e o valor do professor "
                             "(o levantar no chao, a locomocao de pe), e o ator e o critico fazem --cerebro-unico-destilar-passos passos "
                             "supervisionados por rodada (metade de cada fase), sem o PPO; o juiz segue. Depois, o PPO com a taxa da locomocao/5")
    parser.add_argument("--cerebro-unico-destilar-passos", type=int, default=200,
                        help="(13/11 16h20) os passos supervisionados por rodada do --cerebro-unico-destilar-puro (lotes de 4096)")
    parser.add_argument("--cerebro-unico-destilar-reserva", type=int, default=8,
                        help="(13/11 16h20) quantas rodadas de estados a copia pura guarda (a reserva do DAgger)")
    parser.add_argument("--cerebro-unico-destilar-taxa", type=float, default=0.0,
                        help="(13/11 16h) a taxa do ator enquanto --cerebro-unico-destilar ensina (0: a que estava)")
    parser.add_argument("--levantar-imitar-professor", nargs=2, default=None, metavar=("CHECKPOINT", "PESO"),
                        help="(14/11 v1, o DAgger do de costas) com --levantar-imitar: nas linhas do levantar da mascara (limpo, antes da "
                             "troca), a media da acao puxada (MSE) para a do CHECKPOINT (o especialista de cerebro unico, congelado) "
                             "no mesmo estado, com PESO")
    parser.add_argument("--ancora-so-locomocao", action="store_true",
                        help="(16/11, ele: os levantares na velocidade original do clipe) a ancora (--levantar-imitar-professor-ancora) so nas "
                        "linhas da locomocao: os levantares livres para mudar o ritmo (com a marca do de costas, os outros levantares tambem "
                        "ficavam presos a rede antiga)")
    parser.add_argument("--levantar-imitar-professor-ancora", nargs=2, default=None, metavar=("CHECKPOINT", "PESO"),
                        help="(14/11 13h50) com --levantar-imitar-professor: em todas as OUTRAS linhas (a locomocao, o parado, os outros "
                             "levantares, o fogo), a media da acao puxada (MSE) para a do CHECKPOINT (o principal congelado) com PESO - o "
                             "resto preso enquanto o de costas muda para o professor (sem ela, a taxa que copia arrastava a locomocao: KL 0,79)")
    parser.add_argument("--levantar-costas-adaptador", type=float, default=0.0, metavar="TAXA",
                        help="(14/11 16h25) o ramo proprio do levantar de costas (precisa de --levantar-costas-marca): uma rede pequena somada a "
                             "acao so com a marca, o otimizador dele com TAXA; com --levantar-imitar-professor, o professor ensina so o ramo "
                             "(a base destacada). 0: sem o ramo")
    parser.add_argument("--levantar-imitar-professor-pernas", type=float, default=1.0, metavar="PESO",
                        help="(14/11 15h40) com --levantar-imitar-professor: o peso dos canais das pernas (coxa, canela e pe) na puxada do "
                             "professor - na media dos 82 canais o erro da perna esquerda no giro (77 cm do clipe; o professor 10-31) pesava pouco")
    parser.add_argument("--levantar-costas-marca", type=float, default=0.0, metavar="VALOR",
                        help="(14/11 14h15) a entrada levantar.variante vale VALOR no levantar de costas de pe (os outros 0): a rede sabe "
                             "qual levantar e (o correndo de costas comeca igual) e, com --levantar-imitar-professor-ancora, a ancora prende "
                             "todas as linhas sem a marca; o professor ve 0 nela (o convenio dele). Precisa de --levantar-unreal-variante")
    parser.add_argument("--levantar-imitar-professor-taxa", type=float, default=0.0, metavar="TAXA",
                        help="(14/11 v1, 08h50) com --levantar-imitar-professor: a taxa da rede unica fica em TAXA a cada atualizacao (o ajuste pelo KL a deixava no piso 1e-6 desde 13/11 e o DAgger nao andaria); 0: a do ajuste")
    parser.add_argument("--cerebro-unico-professor", type=str, default="",
                        help="(13/11 16h) o checkpoint de duas cabecas que ensina o --cerebro-unico-destilar")
    parser.add_argument("--cerebro-unico-queda-aprende", action="store_true",
                        help="(13/11 17h, ele: evitar e melhorar as quedas iminentes, a recuperacao rapida e adaptativa) com --cerebro-unico, as "
                             "linhas do levantar dentro de um episodio de locomocao (a queda no comando, que levanta e segue) aprendem, com os "
                             "pesos e os custos do levantar; sem isto, so as de pe aprendem nesses episodios, como com duas cabecas")
    parser.add_argument("--cerebro-unico-queda-custo", type=float, default=0.1,
                        help="(13/11 17h05) com --cerebro-unico-queda-aprende, o custo por passo caido no meio da locomocao (a nota do levantar, o "
                             "premio da entrega e o estilo saem ali - cair nunca compensa)")
    parser.add_argument("--queda-vira-comando", action="store_true",
                        help="(13/11 17h, ele: 'os treinos nao encerrarem so porque houve a queda... aprender com as falhas... como cair melhor, se "
                             "recuperar melhor') com --comando-levantar, a queda num exercicio de locomocao (rumos, manobras, estrela, giro) nao "
                             "encerra: o episodio vira comando ali (o jogador emulado com a tecla e o shift do que ele fazia), levanta como no "
                             "comando e segue (pelo menos 4 s depois da queda); conta no grupo de antes")
    parser.add_argument("--cerebro-unico-mistura", type=float, default=0.3,
                        help="(13/11) a fracao das linhas do levantar na mistura das normalizacoes do --cerebro-unico")
    parser.add_argument("--recuperacao-norma", action="store_true",
                        help="(11/11) a normalizacao das entradas da recuperacao aprende com as linhas dela (a cabeca do levantar "
                             "recomecada do zero; sem isto ela fica como veio)")
    parser.add_argument("--levantar-deitado", action="store_true",
                        help="(07/11) os comecos no clipe (--comeco-clipe) deitados de costas ou de brucos e o levantar em movimento "
                             "(--movimento) deitado, das levantadas da Unreal; as avaliacoes costas_ e brucos_ (precisa --juiz-unreal)")
    parser.add_argument("--juiz-ciclos", type=float, default=0.0,
                        help="(07/11) com --juiz-unreal: os ciclos do juiz da locomocao sao os nossos (ciclo_*, a fracao F dos "
                             "pares, repetidos) e da Unreal so as transicoes; o juiz da locomocao do zero uma vez")
    parser.add_argument("--juiz-freio", type=float, default=None,
                        help="(07/11) os juizes so dao o passo no minilote em que a margem deles - a nota media dos clipes menos "
                             "a do corpo, de 0 a 2 - esta abaixo deste valor: seguros demais, nao aprendem mais - nao saturam")
    parser.add_argument("--juiz-unreal", action="store_true",
                        help="(07/11) o juiz de estilo com os clipes do Game Animation Sample (unreal_style_clips); na primeira vez "
                             "os dois juizes recomecam do zero (a marca 'juiz' nos contadores)")
    parser.add_argument("--estilo-manobras-zero", action="store_true",
                        help="(06/11 22h) nas manobras (trocas e curvas) da locomocao, so a tarefa - sem o juiz de estilo")
    parser.add_argument("--preservar-velocidade", type=float, default=0.0,
                        help="(06/11 19h25) com --levantar-velocidade, a ancora tambem nas velocidades da recuperacao, com este peso")
    parser.add_argument("--amortecimento-livre", type=float, default=0.0,
                        help="(06/11) a escala do amortecimento dos motores da locomocao sem clipe (0: a do rastreio, 0,5)")
    parser.add_argument("--velocidade-propria", action="store_true",
                        help="(06/11) na cabeca da locomocao sem clipe, a velocidade-alvo dos motores pelo proprio alvo dela")
    parser.add_argument("--imitacao", type=float, default=0.0,
                        help="na locomocao, o peso da imitacao do clipe do passo pedido (DeepMimic; 0: sem)")
    parser.add_argument("--rampa-velocidade", type=float, default=0.5,
                        help="a rampa da velocidade ate a pedida: (feita/pedida)^isto (0,5 a raiz; 1 linear)")
    return parser


def aplicar_receita(parser: argparse.ArgumentParser, args) -> None:
    """As validacoes, os padroes derivados e o estado de modulo que a receita liga (10/11: fora do main)."""
    STAR_SECONDS[0] = args.estrela_segundos
    STAR_PATH_WIDTH[0] = args.estrela_caminho[1] if args.estrela_caminho[0] > 0 else 0.0
    COMMAND_SECONDS[0] = args.comando_segundos
    if args.juiz_mm and not (args.juiz_unreal and args.estilo_loco_proprio):
        raise SystemExit("--juiz-mm precisa de --juiz-unreal e --estilo-loco-proprio")
    if args.levantar_deitado and not args.juiz_unreal:
        raise SystemExit("--levantar-deitado precisa de --juiz-unreal (os clipes deitados sao os da Unreal)")
    if args.congelar_locomocao and (args.congelar_recuperacao or not args.habilidades_separadas):
        parser.error("--congelar-locomocao exige --habilidades-separadas e nao combina com --congelar-recuperacao")
    if args.levantar_clipe:
        # (O levantar pelo clipe e o guiado no tempo do proprio clipe, com o exercicio dele na forca dinamica.)
        args.levantar_guiado, args.levantar_segundos = True, 0.0
        GETUP_CLIP[0] = True
    if args.congelar_recuperacao and not args.habilidades_separadas:
        parser.error("--congelar-recuperacao exige --habilidades-separadas")
    if args.locomocao_fogo_degrau_rodadas > 0:
        # (13/11 17h35) O degrau da escada do fogo da locomocao mais curto - a queda agora vira aula (--queda-vira-comando).
        global LOCO_FIRE_DWELL
        LOCO_FIRE_DWELL = int(args.locomocao_fogo_degrau_rodadas)
    if args.locomocao_fogo_aceleracao[0] > 0:
        # (13/11 18h50) A subida do fogo mais rapida: a tolerancia do reajuste do torto na guarda do limpo (um limiar ruidoso - 0,26-0,37
        # na mesma rede - segurava o programa inteiro com 0,28 contra 0,29), o degrau da fatia e as medidas boas para ela subir.
        global LOCO_FIRE_CROOKED_TOL, LOCO_FIRE_SHARE_STEP, LOCO_FIRE_SHARE_GOOD
        LOCO_FIRE_CROOKED_TOL, LOCO_FIRE_SHARE_STEP, LOCO_FIRE_SHARE_GOOD = (float(args.locomocao_fogo_aceleracao[0]),
                                                                             float(args.locomocao_fogo_aceleracao[1]),
                                                                             int(args.locomocao_fogo_aceleracao[2]))
    if args.cerebro_unico and (not args.habilidades_separadas or args.congelar_locomocao or args.congelar_recuperacao):
        parser.error("--cerebro-unico exige --habilidades-separadas e nao combina com --congelar-locomocao/--congelar-recuperacao")
    if args.peso_tarefa_loco is None:
        args.peso_tarefa_loco = args.peso_tarefa
    if args.peso_estilo_loco is None:
        args.peso_estilo_loco = args.peso_estilo
    if args.suave_loco is None:
        args.suave_loco = args.suave
    if args.equilibrio:
        BATCH_EXTRA.update(balance_observation=True, step_channels=True)
    if args.parado_ancora_obs:
        BATCH_EXTRA.update(idle_anchor_observation=True)
    if args.levantar_unreal_variante:
        BATCH_EXTRA.update(clip_variant_observation=True)
    if args.levantar_fogo_entrada:
        BATCH_EXTRA.update(fire_observation=True)
    if args.parado_segundos > 0:
        # (11/11 10h30: os episodios do parado de 6 para isto - o RL v1, que ficou imovel, treinava 20 s; ficar parado mais de ~5 s
        # quase nunca era treinado. As avaliacoes passam a propria duracao e ficam como estao.)
        BATCH_EXTRA.update(episode_seconds=args.parado_segundos)
        IDLE_SECONDS[0] = args.parado_segundos
    VELOCITY_ACTIONS[0] = bool(args.acao_velocidade)
    usar_costas_rola(args.levantar_costas_rola)
    if args.levantar_costas_clipe:
        if args.levantar_costas_rola is not None:
            raise SystemExit("--levantar-costas-clipe e --levantar-costas-rola juntos")
        usar_costas_rola(args.levantar_costas_clipe)
    usar_brucos_clipe(args.levantar_brucos_clipe)
    global LEVANTAR_UNREAL_VARIANTS
    if args.levantar_brucos_marca != 0.0:
        if not args.levantar_unreal_variante or not -0.5 <= args.levantar_brucos_marca < 0.0:
            raise SystemExit("--levantar-brucos-marca precisa de --levantar-unreal-variante e de um valor em [-0.5, 0)")
        LEVANTAR_UNREAL_VARIANTS = (LEVANTAR_UNREAL_VARIANTS[0], float(args.levantar_brucos_marca)) + tuple(LEVANTAR_UNREAL_VARIANTS[2:])
    # (14/11 14h15, --levantar-costas-marca) A entrada levantar.variante marca o de costas de pe (o 0 dos LEVANTAR_UNREAL_KEYS).
    if args.levantar_costas_marca > 0:
        if not args.levantar_unreal_variante:
            raise SystemExit("--levantar-costas-marca precisa de --levantar-unreal-variante (a entrada levantar.variante)")
        # (14h20) Acima de 0,5 o C++ trata o exercicio como o "rolando" de 11/11 (livre ate de brucos, fora das tentativas novas):
        # com o clipe de costas de pe isso derrubou o modulo nativo (SEGV na r115240). A marca fica so como entrada.
        if args.levantar_costas_marca > 0.5:
            raise SystemExit("--levantar-costas-marca deve ser <= 0,5 (acima disso o C++ liga o comportamento do rolando)")
        LEVANTAR_UNREAL_VARIANTS = (float(args.levantar_costas_marca),) + tuple(LEVANTAR_UNREAL_VARIANTS[1:])


def ler_receita(argv: list[str]):
    """A receita (a lista de argumentos do treinador) lida e aplicada como no treino: os args."""
    parser = build_parser()
    args = parser.parse_args(argv)
    aplicar_receita(parser, args)
    return args


def configurar_levantar(b, args, fraction: float) -> None:
    """(12/11 18h, o levantar pelo clipe num lote - o do treino, ou uma sonda/regua que precise dele: o comando ligado ao levantar) Os
    clipes da Unreal, a fracao dos episodios de levantar (0: so para o levantar do meio do episodio), o adaptativo e o nunca desistir, as
    janelas e as adversidades - como no main."""
    b.add_style_clips(LEVANTAR_UNREAL)
    b.set_clip_get_up(LEVANTAR_UNREAL, fraction, args.levantar_unreal_meio, args.levantar_unreal_depois,
                          args.levantar_unreal_ruido, args.levantar_unreal_largura, args.levantar_unreal_vel)
    b.set_clip_get_up_parts(args.levantar_unreal_partes)
    if args.levantar_rastrear:
        if not hasattr(b, "set_clip_track_mask"):
            raise SystemExit("--levantar-rastrear precisa da build com set_clip_track_mask (build-base-n108)")
        b.set_clip_track_mask(int(args.levantar_rastrear))
    if args.levantar_costas_rola is not None or args.levantar_lado_agora:
        if not hasattr(b, "set_clip_side_now"):
            raise SystemExit("--levantar-costas-rola / --levantar-lado-agora precisam da build com set_clip_side_now (build-base-n107)")
        b.set_clip_side_now(True)
    if args.levantar_chegada_de_pe != 1.0:
        if not hasattr(b, "set_get_up_stand_bonus_scale"):
            raise SystemExit("--levantar-chegada-de-pe precisa da build com set_get_up_stand_bonus_scale (build-base-n115)")
        b.set_get_up_stand_bonus_scale(args.levantar_chegada_de_pe)
    b.set_clip_get_up_variants(list(LEVANTAR_UNREAL_VARIANTS))
    if hasattr(b, "set_clip_get_up_deliver"):
        b.set_clip_get_up_deliver(list(LEVANTAR_UNREAL_DELIVER))
    if hasattr(b, "set_clip_get_up_agility"):
        b.set_clip_get_up_agility(clip_get_up_agility(args, None))
    if args.levantar_adaptativo:
        b.set_clip_get_up_adaptive(True)
        b.set_adaptive_speed_bonus(args.levantar_adaptativo_velocidade)
        if hasattr(b, "set_adaptive_timing"):
            b.set_adaptive_timing(*args.levantar_tempo_alvo, args.levantar_impulso_corrida)
        if max(args.levantar_tempo_alvo_correndo) > 0:
            b.set_adaptive_timing_run(*args.levantar_tempo_alvo_correndo)
        if args.levantar_tempo_queda != 0.5:
            b.set_adaptive_time_fall(args.levantar_tempo_queda)
        if args.levantar_tempo_fogo >= 0:
            b.set_adaptive_time_fire(args.levantar_tempo_fogo)
        if args.levantar_forma > 0:
            b.set_get_up_form(args.levantar_forma)
        if args.levantar_forma_perna[0] > 0:
            if not hasattr(b, "set_get_up_leg_form"):
                raise SystemExit("--levantar-forma-perna precisa da build com set_get_up_leg_form (build-base-n109)")
            b.set_get_up_leg_form(float(args.levantar_forma_perna[0]), int(args.levantar_forma_perna[1]))
        if args.levantar_ritmo[0] > 0:
            if not hasattr(b, "set_get_up_pace"):
                raise SystemExit("--levantar-ritmo precisa da build com set_get_up_pace (build-base-n95)")
            if args.levantar_forma <= 0:
                raise SystemExit("--levantar-ritmo precisa de --levantar-forma (a tabela da cabeca do clipe)")
            b.set_get_up_pace(*args.levantar_ritmo)
        if args.levantar_fogo_soltar_firme:
            b.set_get_up_fire_release_at_firm(True)
        if args.levantar_forma_mao > 0:
            b.set_get_up_palm(args.levantar_forma_mao)
        if args.levantar_calma > 0:
            b.set_settle_calm(args.levantar_calma)
        if hasattr(b, "set_run_post_rise"):
            b.set_run_post_rise(*args.levantar_velocidade_corrida)
        if args.levantar_pose_variada > 0:
            b.set_get_up_pose_variety(args.levantar_pose_variada)
        if args.levantar_troca_calma[0] > 0:
            b.set_calm_handoff(*args.levantar_troca_calma)
        if hasattr(b, "set_adaptive_settle"):
            b.set_adaptive_settle(args.levantar_adaptativo_firmeza, args.levantar_fogo_soltar)
        if args.levantar_correndo_troca >= 0 and hasattr(b, "set_run_handoff_at_stand"):
            b.set_run_handoff_at_stand(True, args.levantar_correndo_troca)
        if args.levantar_e_ir[0] > 0 and hasattr(b, "set_get_up_go"):
            b.set_get_up_go(*args.levantar_e_ir)
        if args.levantar_vista > 0 or args.levantar_brucos_tempo != 1.0:
            if not hasattr(b, "set_get_up_view"):
                raise SystemExit("--levantar-vista / --levantar-brucos-tempo precisam da build com set_get_up_view (build-base-n113)")
            b.set_get_up_view(args.levantar_vista)
            b.set_get_up_prone_time_scale(args.levantar_brucos_tempo)
        if args.mao_atras > 0:
            if not hasattr(b, "set_hand_behind"):
                raise SystemExit("--mao-atras precisa da build com set_hand_behind (build-base-n114)")
            b.set_hand_behind(args.mao_atras)
        if args.levantar_tentar:
            b.set_adaptive_retry(True)
        if args.levantar_fogo_contra_corrida > 0:
            b.set_fire_run_headwind_cap(args.levantar_fogo_contra_corrida)
        if max(args.levantar_paredes) > 0:
            b.set_get_up_arena(*args.levantar_paredes)
        if hasattr(b, "set_get_up_fire_extras"):
            b.set_get_up_fire_extras(*fire_extras(args))
        if args.levantar_fogo_pancada_cedo[0] > 0:
            b.set_get_up_fire_early(*args.levantar_fogo_pancada_cedo)
    b.set_clip_get_up_start_bias(list(LEVANTAR_UNREAL_START_BIAS))


def configurar(b, args, counters: dict, training: bool) -> None:
    """Os ajustes da receita num lote (o do treino com training=True; as avaliacoes e a suite com False). A unica traducao
    das flags para o ambiente (10/11: antes um fecho dentro do main, e as sondas usavam outra)."""
    # (A nota e so a v19; o 19 aqui so liga a inclinacao da referencia, que o lote nasce sem - ver o trackingReward no .hpp.)
    b.set_tracking_reward(19)
    b.set_phase_sync(args.sincronizar_fase)
    if args.clipe_sob_pelve:
        if not hasattr(b, "set_clip_under_pelvis"):
            raise SystemExit("--clipe-sob-pelve precisa da build com set_clip_under_pelvis (build-base-n117)")
        b.set_clip_under_pelvis(True)
    if args.pernas_alinhadas:
        if not hasattr(b, "set_leg_align"):
            raise SystemExit("--pernas-alinhadas precisa da build com set_leg_align (build-base-n120)")
        b.set_leg_align(float(args.pernas_alinhadas[0]), float(args.pernas_alinhadas[1]) if len(args.pernas_alinhadas) > 1 else 10.0)
    if args.largada_maos > 0:
        if not hasattr(b, "set_launch_arms_calm"):
            raise SystemExit("--largada-maos precisa da build com set_launch_arms_calm (build-base-n128)")
        b.set_launch_arms_calm(args.largada_maos)
    if args.parado_centro > 0 or args.velocidade_largura > 0:
        if not hasattr(b, "set_idle_weight_center"):
            raise SystemExit("--parado-centro/--velocidade-largura precisam da build com set_idle_weight_center (build-base-n127)")
        b.set_idle_weight_center(args.parado_centro)
        if args.velocidade_largura > 0:
            b.set_speed_width_fraction(args.velocidade_largura)
    if args.corrida_largura > 0 or args.pernas_alinhadas_comando:
        if not hasattr(b, "set_run_width"):
            raise SystemExit("--corrida-largura/--pernas-alinhadas-comando precisam da build com set_run_width (build-base-n126)")
        b.set_run_width(args.corrida_largura)
        b.set_leg_align_command(bool(args.pernas_alinhadas_comando))
    if args.largada_margem_lado:
        if not hasattr(b, "set_launch_side_margin"):
            raise SystemExit("--largada-margem-lado precisa da build com set_launch_side_margin (build-base-n125)")
        b.set_launch_side_margin(True)
    if args.referencial_clipe or args.fase_dois_pes or args.rumo_medio[0] > 0 or args.forca_contatos or args.chao_grama > 0:
        if not hasattr(b, "set_grass_ground_fraction"):
            raise SystemExit("--referencial-clipe/--fase-dois-pes/--rumo-medio/--forca-contatos/--chao-grama precisam da build-base-n131")
        b.set_reference_side_fix(bool(args.referencial_clipe))
        b.set_phase_sync_both_feet(bool(args.fase_dois_pes))
        b.set_heading_bias(float(args.rumo_medio[0]), float(args.rumo_medio[1]) if len(args.rumo_medio) > 1 else 0.10)
        b.set_contact_force_observation(bool(args.forca_contatos))
        b.set_grass_ground_fraction(float(args.chao_grama))
    if args.reta_comando:
        if not hasattr(b, "set_straight_command"):
            raise SystemExit("--reta-comando precisa da build-base-n131")
        b.set_straight_command(True)
    if args.projeteis[0] > 0:
        if not hasattr(b, "set_projectiles"):
            raise SystemExit("--projeteis precisa da build-base-n134")
        p = list(args.projeteis) + [10.0, 40.0, 0.06][len(args.projeteis) - 1:]
        b.set_projectiles(float(p[0]), float(p[1]), float(p[2]), float(p[3]))
    if args.imitar_velocidade:
        if not hasattr(b, "set_imitation_velocity_targets"):
            raise SystemExit("--imitar-velocidade precisa da build-base-n133")
        b.set_imitation_velocity_targets(True)
    if args.velocidade_rotacao:
        if not hasattr(b, "set_clip_velocity_rotation"):
            raise SystemExit("--velocidade-rotacao precisa da build-base-n132")
        b.set_clip_velocity_rotation(True)
    if args.lado_medio[0] > 0 or args.imitacao_portao[0] > 0:
        if not hasattr(b, "set_imitation_gate"):
            raise SystemExit("--lado-medio/--imitacao-portao precisam da build-base-n131")
        b.set_lateral_bias(float(args.lado_medio[0]), float(args.lado_medio[1]) if len(args.lado_medio) > 1 else 0.03)
        b.set_imitation_gate(float(args.imitacao_portao[0]), float(args.imitacao_portao[1]) if len(args.imitacao_portao) > 1 else 0.6)
    if args.locomocao_ambiente > 0:
        if not hasattr(b, "set_locomotion_scenery"):
            raise SystemExit("--locomocao-ambiente precisa da build com set_locomotion_scenery (build-base-n130)")
        b.set_locomotion_scenery(args.locomocao_ambiente if training else 0.0)
    if args.revisao_15h:
        if not hasattr(b, "set_review_fixes"):
            raise SystemExit("--revisao-15h precisa da build com set_review_fixes (build-base-n124)")
        b.set_review_fixes(True)
    if args.parado_lugar:
        if not hasattr(b, "set_idle_hold"):
            raise SystemExit("--parado-lugar precisa da build com set_idle_hold (build-base-n121)")
        b.set_idle_hold(float(args.parado_lugar[0]), float(args.parado_lugar[1]) if len(args.parado_lugar) > 1 else 0.10)
    if args.parado_preso:
        if not hasattr(b, "set_idle_clip_anchor"):
            raise SystemExit("--parado-preso precisa da build com set_idle_clip_anchor (build-base-n118)")
        b.set_idle_clip_anchor(True)
    b.set_imitation_loosen(args.imitacao_folga if training else 1.0)
    b.set_facing_gate(args.peito_vista if training else 0.0)
    if args.rumo_suave > 0:
        b.set_course_smoothing(args.rumo_suave)
    if args.rumo_limite > 0:
        b.set_course_angle_limit(math.radians(args.rumo_limite) if training else 0.0)
    if args.sem_potencial_loco:
        b.set_locomotion_shaping(False)
    if args.nota_velocidade > 0:
        b.set_speed_gate(args.nota_velocidade)
    if args.joelho_dentro > 0:
        b.set_knee_inward(args.joelho_dentro)
    if args.idle_altura > 0:
        b.set_idle_height(args.idle_altura)
    if args.parada_corpo:
        b.set_stop_wait_body(True)
    if args.diagonal_peito > 0:
        b.set_diagonal_chest(args.diagonal_peito if training else 0.0)
    b.set_arm_clip_reach(args.bracos_clipe)
    b.set_gait_cycles(args.passos_ciclos)
    b.set_head_clip_reach(1.0 if args.cabeca_autoridade else args.cabeca_clipe)
    b.set_rise_flight(args.levantar_voo if training else 0.0)
    b.set_lateral_leg_strict(args.lateral_pernas_rigor if training else 1.0)
    b.set_slow_limits([float(x) for x in counters["lento"]] if training and args.lento > 0 else [0.0, 0.0, 0.0])
    if training:
        b.set_maneuvers(args.manobras, -1)
    b.set_idle_imitation(args.imitacao_parado)
    if hasattr(b, "set_free_velocity_feedforward"):
        b.set_free_velocity_feedforward(args.velocidade_propria)
    if args.levantar_velocidade is not None:
        b.set_recovery_velocity(True)
    if args.manobras_pesos:
        b.set_maneuver_weights(list(args.manobras_pesos))
    if args.comando > 0:
        b.set_command(args.comando if training else 0.0, int(counters["comando_fase"]), args.comando_segundos)
    if args.parada > 0:
        b.set_final_stop(args.parada if training else 0.0, *args.parada_janela, args.parada_freio, False)
        if args.parada_clipes:
            b.set_stop_clips(True)
    if args.estrela > 0:
        # (08/11: a estrela so no treino - a fatia; nas avaliacoes, so as dela.)
        b.set_star(args.estrela if training else 0.0, [float(x) for x in counters["estrela_pesos"]],
                   [int(x) for x in counters["estrela_fases"]], args.estrela_segundos)
        if (args.estrela_caminho[0] > 0 or args.estrela_curvas[0] > 0) and hasattr(b, "set_star_rework"):
            b.set_star_rework(*args.estrela_caminho, *args.estrela_curvas)
    if args.comando_levantar and hasattr(b, "set_command_get_up"):
        b.set_command_get_up(True, args.comando_levantar_correndo)
        # (13/11 17h, ele: os treinos nao encerrarem so porque houve a queda) A queda nos outros exercicios de locomocao vira comando.
        # (So no treino: nas avaliacoes e na capacidade a queda segue encerrando - a regua continua comparavel.)
        if args.queda_vira_comando and training:
            if not hasattr(b, "set_fall_to_command"):
                raise SystemExit("--queda-vira-comando precisa da build com set_fall_to_command (build-base-n94)")
            b.set_fall_to_command(True)
    if args.fogo >= 0 and training:
        hit, hold = fire_bands(counters["fogo_nivel"])
        b.set_fire(args.fogo_fatia, hit, hold, FIRE_HOLD_SECONDS, FIRE_HOLD_FRACTION)
        b.set_fire_groups(fire_group_bands(counters["fogo_niveis"]))
        b.set_push_variety(args.fogo_pontos)
        b.set_push_imitation_relief(args.fogo_alivio)
    if args.amortecimento_livre > 0 and hasattr(b, "set_free_damping_scale"):
        b.set_free_damping_scale(args.amortecimento_livre)
    b.set_idle(args.idle_pos_levantar, args.parado_locomocao, args.idle_alcance)
    b.set_idle_handoff(args.idle_entrega, args.idle_entrega_alcance)
    b.set_rise_handoff(args.entrega_subindo)
    b.set_recovery_natural(args.naturalidade)
    b.set_lateral_hand_push(not args.trote_lateral_sem_puxao)
    b.set_lateral_run_leg_reach(args.corrida_lateral_pernas)
    b.set_gait_speeds(gait_speeds(args.velocidade_trote), gait_speeds(args.velocidade_corrida))
    b.set_back_start_lean(-np.radians(args.largada_costas))
    b.set_course_limit(args.curva_fora)
    b.set_curve_lean(args.curva_inclinar)
    b.set_foot_width(args.pisada_largura)
    b.set_run_turn_brake(args.troca_freio)
    b.set_run_turn_align(args.troca_pe)
    b.set_turn_settle(args.troca_assentada)
    b.set_turn_launch(args.troca_arrancada)
    b.set_run_launch_ease(*(args.corrida_largada or (0.0, 0.0, 0.0)))
    b.set_course_tail(args.rumo_cauda)
    if training and hasattr(b, "set_speed_range"):
        b.set_speed_range(args.faixa_velocidade)
    if hasattr(b, "set_idle_quiet"):
        b.set_idle_quiet(args.quietude_parado)
    if hasattr(b, "set_maneuver_acceleration"):
        b.set_maneuver_acceleration(args.troca_aceleracao)
    if hasattr(b, "set_rise_overshoot"):
        b.set_rise_overshoot(args.subida_freada)
    b.set_roll_tail(args.inclinacao_cauda)
    b.set_cliff_free(args.sem_penhascos)
    b.set_launch_rise(args.arrancada_pulo)
    if hasattr(b, "set_lateral_start"):
        b.set_lateral_start(args.largada_lado)
    if hasattr(b, "set_turn_settle_lateral"):
        b.set_turn_settle_lateral(args.troca_assentada_lado)
    if hasattr(b, "set_turn_settle_front"):
        b.set_turn_settle_front(args.troca_assentada_frente)
    if hasattr(b, "set_run_turn_time_scale"):
        b.set_run_turn_time_scale(args.troca_giro_tempo)
    if hasattr(b, "set_stop_gait_all"):
        b.set_stop_gait_all(args.parada_passada)
    if hasattr(b, "set_idle_pose_gate"):
        b.set_idle_pose_gate(args.idle_pose)
    if hasattr(b, "set_stop_clips_front"):
        b.set_stop_clips_front(args.parada_clipes_frente)
    if hasattr(b, "set_stop_clips_command"):
        b.set_stop_clips_command(args.parada_clipes_comando)
    if hasattr(b, "set_stop_gate_floor"):
        b.set_stop_gate_floor(args.parada_portao)
    if hasattr(b, "set_free_stop"):
        b.set_free_stop(args.parada_livre)
    if hasattr(b, "set_stop_posture"):
        b.set_stop_posture(args.parada_postura)
    if hasattr(b, "set_stop_idle_pose"):
        b.set_stop_idle_pose(args.parada_pose_sempre)
    if hasattr(b, "set_idle_unified"):
        b.set_idle_unified(args.parado_unico)
    if hasattr(b, "set_stop_to_idle"):
        b.set_stop_to_idle(args.parada_direto_idle)
    if hasattr(b, "set_idle_after_stop"):
        b.set_idle_after_stop(args.idle_depois_freada)
    if hasattr(b, "set_parked_on_locomotion"):
        b.set_parked_on_locomotion(args.parado_na_locomocao)
    if hasattr(b, "set_turn_variety"):
        b.set_turn_variety(args.giro_variado, args.giro_coluna)
    if hasattr(b, "set_still_cost"):
        b.set_still_cost(args.quieto_custos)
    if hasattr(b, "set_idle_static"):
        b.set_idle_static(args.parado_estatico)
    if args.parado_quadril_nivel > 0:
        b.set_idle_pelvis_level(args.parado_quadril_nivel)
    if hasattr(b, "set_idle_costs"):
        b.set_idle_costs(args.parado_micro, args.parado_fora_pose)
    if hasattr(b, "set_idle_reward"):
        b.set_idle_reward(*args.parado_recompensa)
    if args.levantar_imitar > 0:
        if not hasattr(b, "set_get_up_imitation"):
            raise SystemExit("--levantar-imitar precisa da build com set_get_up_imitation (build-base-n104)")
        b.set_get_up_imitation(args.levantar_imitar, float(args.levantar_ritmo[1]))
        if args.levantar_imitar_ritmo_correndo > 0:
            if not hasattr(b, "set_get_up_imitation_pace_run"):
                raise SystemExit("--levantar-imitar-ritmo-correndo precisa da build com set_get_up_imitation_pace_run (build-base-n109)")
            b.set_get_up_imitation_pace_run(args.levantar_imitar_ritmo_correndo)
        if args.levantar_imitar_longe[0] > 0:
            if not hasattr(b, "set_get_up_imitation_far"):
                raise SystemExit("--levantar-imitar-longe precisa da build com set_get_up_imitation_far (build-base-n106)")
            b.set_get_up_imitation_far(*[float(x) for x in args.levantar_imitar_longe])
        if args.levantar_imitar_ancora:
            if not hasattr(b, "set_get_up_imitation_anchor"):
                raise SystemExit("--levantar-imitar-ancora precisa da build com set_get_up_imitation_anchor (build-base-n117)")
            b.set_get_up_imitation_anchor(True)
        if args.levantar_imitar_nota > 0:
            if not hasattr(b, "set_get_up_imitation_reward"):
                raise SystemExit("--levantar-imitar-nota precisa da build com set_get_up_imitation_reward (build-base-n105)")
            b.set_get_up_imitation_reward(args.levantar_imitar_nota)
    if args.parada_bracos_clipe >= 0:
        if not hasattr(b, "set_stop_arms_reach"):
            raise SystemExit("--parada-bracos-clipe precisa da build com set_stop_arms_reach (build-base-n103)")
        b.set_stop_arms_reach(args.parada_bracos_clipe)
    if args.virado_tolerancia > 0:
        if not hasattr(b, "set_turned_grace"):
            raise SystemExit("--virado-tolerancia precisa da build com set_turned_grace (build-base-n101)")
        b.set_turned_grace(args.virado_tolerancia)
    if args.parado_quieto_na_base > 0:
        if not hasattr(b, "set_idle_still_base"):
            raise SystemExit("--parado-quieto-na-base precisa da build com set_idle_still_base (build-base-n100)")
        b.set_idle_still_base(args.parado_quieto_na_base)
    if hasattr(b, "set_upper_rigor"):
        b.set_upper_rigor(args.rigor_superior)
    if hasattr(b, "set_unreal_starts"):
        b.set_unreal_starts(args.arrancada_unreal)
    if hasattr(b, "set_unreal_start_scale"):
        b.set_unreal_start_scale(args.arrancada_unreal_escala)
    if hasattr(b, "set_live_launch"):
        b.set_live_launch(args.largada_viva)
    if hasattr(b, "set_launch_step_reward"):
        b.set_launch_step_reward(args.largada_passo)
    if hasattr(b, "set_launch_in_place_penalty"):
        b.set_launch_in_place_penalty(args.largada_no_lugar)
        if args.largada_velocidade_parcela > 0:
            if not hasattr(b, "set_launch_speed_share"):
                raise SystemExit("--largada-velocidade-parcela precisa da build com set_launch_speed_share (build-base-n110)")
            b.set_launch_speed_share(args.largada_velocidade_parcela)
        if args.largada_imitar > 0:
            if not hasattr(b, "set_launch_imitation"):
                raise SystemExit("--largada-imitar precisa da build com set_launch_imitation (build-base-n112)")
            b.set_launch_imitation(args.largada_imitar)
        if args.largada_inclinar > 0:
            if not hasattr(b, "set_launch_lean"):
                raise SystemExit("--largada-inclinar precisa da build com set_launch_lean (build-base-n109)")
            b.set_launch_lean(args.largada_inclinar)
    if hasattr(b, "set_launch_calm_arms"):
        b.set_launch_calm_arms(args.largada_bracos_calmos)
    if hasattr(b, "set_spine_velocity_feedforward"):
        b.set_spine_velocity_feedforward(args.coluna_sem_velocidade)
    if hasattr(b, "set_base_posture"):
        b.set_base_posture(args.postura_base)
    if hasattr(b, "set_motor_tone"):
        b.set_motor_tone(args.tonus)
    if hasattr(b, "set_posture_tail"):
        b.set_posture_tail(args.postura_cauda)
    if hasattr(b, "set_spine_tone"):
        b.set_spine_tone(args.tonus_coluna)
    if hasattr(b, "set_jam_cost"):
        b.set_jam_cost(args.custo_batente)
    if hasattr(b, "set_foot_slip_cost"):
        b.set_foot_slip_cost(args.pes_firmes)
    if hasattr(b, "set_motor_gravity"):
        b.set_motor_gravity(args.gravidade_motores)
    if hasattr(b, "set_stop_clips_base"):
        b.set_stop_clips_base(args.parada_clipes_base)
    if hasattr(b, "set_stop_entry"):
        b.set_stop_entry(*args.parada_entrada)
    if hasattr(b, "set_stop_front_variants") and args.parada_variantes:
        b.set_stop_front_variants(True, -1)
        if hasattr(b, "set_stop_front_variant_mask"):
            b.set_stop_front_variant_mask(args.parada_variantes_mascara)
    if hasattr(b, "set_free_troca"):
        b.set_free_troca(args.troca_livre)
    if hasattr(b, "set_free_stand_gate"):
        b.set_free_stand_gate(args.parada_pose)
    if hasattr(b, "set_free_idle"):
        b.set_free_idle(args.parado_livre)
    if hasattr(b, "set_pose_boost"):
        b.set_pose_boost(args.pose_reforco)
    if hasattr(b, "set_free_push"):
        b.set_free_push(args.empurrao_livre)
    if hasattr(b, "set_free_idle_match"):
        b.set_free_idle_match(args.parado_semelhanca)
    if hasattr(b, "set_free_step_height"):
        b.set_free_step_height(args.passo_baixo)
    if hasattr(b, "set_idle_bent_end"):
        b.set_idle_bent_end(args.parado_dobrado_encerra)
    b.set_final_row(args.linha_final)
    if hasattr(b, "set_idle_wide"):
        b.set_idle_wide(args.parado_largo)
    if hasattr(b, "set_mirror_cycles"):
        b.set_mirror_cycles(args.simetria_ciclos)
    if hasattr(b, "set_turn_settle_back"):
        b.set_turn_settle_back(args.troca_assentada_costas)
    if hasattr(b, "set_troca_fixed_chest"):
        b.set_troca_fixed_chest(args.troca_peito_fixo)
    if args.penhascos_partes is not None:
        b.set_cliff_parts(*(part in args.penhascos_partes for part in ("curvas", "pe", "antebraco", "pulo")))
    b.set_troca_practice(args.troca_pratica, [0.0] * 8)
    b.set_foot_cross(args.pegada_cruzada)
    b.set_idle_reanchor(args.estab_ancora)
    b.set_front_lean(*np.radians(args.trote_frente_inclinar))
    b.set_run_lean(math.radians(args.corrida_inclinar[0]), args.corrida_inclinar[1])
    b.set_run_back_lean(math.radians(args.corrida_costas_inclinar[0]), args.corrida_costas_inclinar[1])
    b.set_get_up_tracking(args.levantar_clipe)
    b.set_sprint(args.sprint_velocidade, math.radians(args.sprint_inclinar[0]), args.sprint_inclinar[1])
    b.set_upper_hand(args.mao_tronco)
    b.set_body_hand(args.mao_corpo)
    if args.equilibrio:
        # (04/11 15h, os passos: o gerador no idle, quem segura o empurrao de pe, a chegada do levantar e o equilibrio
        # parado - no treino a fatia dos comecos de pe, na avaliacao 8 todos -, no nivel do curriculo.)
        b.set_balance_steps(True, args.empurrao_idle)
        b.set_get_up_arrival(args.chegada_premio, args.voo_multa, args.subida_equilibrio, args.entrega_calma)
        b.set_idle_threat_reach(args.idle_alcance_ameaca)
        b.set_balance_drill(args.equilibrio_fatia if training else 0.0,
                            1.0 if args.fogo >= 0 and training else counters["equilibrio_nivel"])
        b.set_avoid_on_idle(args.evitar_idle)
    b.set_accel_lean(math.radians(args.troca_inclinar))


def montar_v2(args) -> dict:
    """Os ajustes de todo lote da receita, o do treino e os das avaliacoes (10/11: fora do main, para a suite)."""
    return dict(separate_skills=args.habilidades_separadas,
              tracking_termination=args.rastreio_encerra, avoid_newtons=tuple(args.evitar_forca), push_newtons=tuple(args.empurrao_forca),
              move_speed=tuple(args.velocidade), push_hold_fraction=args.empurrao_continuo,
              push_hold_newtons=tuple(args.empurrao_continuo_forca), walk_fraction=args.andar,
              walk_speed=tuple(args.andar_velocidade), trot_speed=tuple(args.trote_velocidade),
              moving_push_newtons=tuple(args.empurrao_mov_forca), moving_push_hold_fraction=args.empurrao_mov_continuo,
              moving_push_hold_newtons=tuple(args.empurrao_mov_continuo_forca),
              locomotion_seconds=args.locomocao_segundos, locomotion_termination=args.queda_encerra,
              **({"clip_blend": args.mistura_clipe} if args.mistura_clipe < 1.0 else {}),
              **({"protective_fall": True} if args.queda_programada else {}),
              **({"roll_blend": args.rolar_clipe} if args.rolar_clipe < 1.0 else {}),
              **({"scenery": tuple(args.ambientes)} if any(args.ambientes) else {}),
              **({"avoid_scenery_fraction": args.evitar_ambientes} if args.evitar_ambientes > 0 else {}),
              **({"get_up_guide": True, "get_up_seconds": args.levantar_segundos} if args.levantar_guiado else {}))


def lote_do_treino(args) -> dict:
    """Os ajustes so do lote do treino (as fatias dos comecos e dos exercicios), alem do montar_v2 (10/11: fora do main)."""
    return dict(seed=args.semente, clip_start_fraction=args.comeco_clipe,
                standing_start_fraction=args.comeco_de_pe, moving_fraction=args.movimento,
                avoid_fraction=args.evitar, sprint_start_fraction=args.arrancada,
                locomotion_fraction=args.locomocao, turn_fraction=args.giro,
                handoff_on_stand=args.troca_de_pe, locomotion_fall_fraction=args.quedas_locomocao,
                partner_seconds=args.parceiro_segundos, partner_fall_penalty=args.parceiro_punicao,
                partner_avoid=args.evitar_com_parceiro, touch_down_penalty=args.toque_punicao,
                support_bonus=args.apoio_premio, flex_reward=args.flexao,
                obstacle_fraction=args.obstaculo, weight_fraction=args.peso_em_cima,
                run_fraction=args.corrida, sprint_fraction=args.sprint,
                locomotion_clip_fraction=args.locomocao_clipe,
                step_reward=args.passo,
                **({"speed_ramp_power": args.rampa_velocidade} if args.rampa_velocidade != 0.5 else {}),
                **({"imitation_weight": args.imitacao} if args.imitacao > 0 else {}),
                **({"only_clip": args.so_clipe} if args.so_clipe else {}))


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    aplicar_receita(parser, args)

    from matter_rl.system import cpu_temperature, require_watchdog
    require_watchdog()
    torch.set_num_threads(1)

    stop = {"pedido": False, "salvar": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(pedido=True))
    # (O "Salvar" do laboratorio: SIGUSR1, que sem isto fecharia o processo sem salvar.)
    signal.signal(signal.SIGUSR1, lambda *_: stop.update(salvar=True))

    run = DATA / args.treino
    run.mkdir(parents=True, exist_ok=True)
    for name, data in (("run.json", {"kind": "levantar", "task": "levantar_livre", "de": args.de}),
                       ("tarefa.json", {"schema": "matter-getup-amp-1", "id": "levantar_livre"})):
        if not (run / name).exists():
            (run / name).write_text(json.dumps(data, ensure_ascii=False))
    # (A configuracao do v2 de agora no run.json, a cada abertura: o laboratorio mostra o levantar em movimento e o
    # evitar a queda com as mesmas forcas e velocidades do treino.)
    info = json.loads((run / "run.json").read_text())
    info["v2"] = {"movimento": args.movimento, "evitar": args.evitar, "arrancada": args.arrancada,
                  "levantar_unreal": args.levantar_unreal, "levantar_unreal_depois": args.levantar_unreal_depois,
                  "levantar_unreal_largura": args.levantar_unreal_largura, "levantar_unreal_vel": args.levantar_unreal_vel,
                  "levantar_unreal_partes": bool(args.levantar_unreal_partes),
                  "levantar_unreal_agilidade": list(args.levantar_unreal_agilidade),
                  "levantar_unreal_entrega": list(LEVANTAR_UNREAL_DELIVER), "levantar_costas_rola": args.levantar_costas_rola, "levantar_costas_clipe": args.levantar_costas_clipe, "levantar_costas_enxerto": args.levantar_costas_enxerto, "levantar_brucos_clipe": args.levantar_brucos_clipe, "levantar_brucos_marca": args.levantar_brucos_marca, "levantar_brucos_enxerto": args.levantar_brucos_enxerto,
                  "levantar_costas_marca": args.levantar_costas_marca, "levantar_forma_perna": list(args.levantar_forma_perna),
                  "levantar_adaptativo": bool(args.levantar_adaptativo), "levantar_adaptativo_firmeza": args.levantar_adaptativo_firmeza, "levantar_tempo_alvo": list(args.levantar_tempo_alvo), "levantar_tempo_alvo_correndo": list(args.levantar_tempo_alvo_correndo), "levantar_calma": args.levantar_calma, "levantar_tempo_queda": args.levantar_tempo_queda, "levantar_tempo_fogo": args.levantar_tempo_fogo, "levantar_forma": args.levantar_forma, "levantar_fogo_soltar_firme": bool(args.levantar_fogo_soltar_firme), "levantar_forma_mao": args.levantar_forma_mao, "levantar_ritmo": list(args.levantar_ritmo), "parado_quadril_nivel": args.parado_quadril_nivel, "levantar_fogo_pancada_cedo": list(args.levantar_fogo_pancada_cedo),
                  "levantar_impulso_corrida": args.levantar_impulso_corrida, "levantar_velocidade_corrida": list(args.levantar_velocidade_corrida), "levantar_pose_variada": args.levantar_pose_variada, "levantar_troca_calma": list(args.levantar_troca_calma), "levantar_fogo_soltar": args.levantar_fogo_soltar, "levantar_tentar": bool(args.levantar_tentar), "levantar_fogo_contra_corrida": args.levantar_fogo_contra_corrida, "levantar_paredes": list(args.levantar_paredes), "levantar_fogo_extras": list(fire_extras(args)), "levantar_fogo": list(args.levantar_fogo),
                  "levantar_fogo_de_cima": args.levantar_fogo_de_cima,
                  "levantar_fogo_entrada": bool(args.levantar_fogo_entrada),
                  "levantar_fogo_prova": ({k: v for k, v in json.loads(Path(args.levantar_fogo_niveis).read_text()).items()
                                           if k in LEVANTAR_UNREAL_KEYS} if args.levantar_fogo_niveis else {}),
                  "levantar_unreal_variante": bool(args.levantar_unreal_variante),
                  "congelar_locomocao": bool(args.congelar_locomocao), "recuperacao_norma": bool(args.recuperacao_norma),
                  "deitado": bool(args.levantar_deitado), "juiz_unreal": bool(args.juiz_unreal),
                  "preservar_peso": args.preservar_peso, "habilidades_separadas": args.habilidades_separadas, "cerebro_unico": bool(args.cerebro_unico),
                  "rastreio": True, "recompensa_rastreio": 19, "rastreio_encerra": args.rastreio_encerra, "residuo": 0.0,
                  "acao_velocidade": args.acao_velocidade, "levantar_velocidade": args.levantar_velocidade is not None,
                  "congelar_recuperacao": args.congelar_recuperacao,
                  "peso_tarefa": args.peso_tarefa, "peso_estilo": args.peso_estilo, "suave": args.suave,
                  "locomocao": args.locomocao, "giro": args.giro, "andar": args.andar, "troca_de_pe": args.troca_de_pe,
                  "quedas_locomocao": args.quedas_locomocao, "parceiro": args.parceiro,
                  "parceiro_segundos": args.parceiro_segundos, "parceiro_punicao": args.parceiro_punicao,
                  "evitar_com_parceiro": args.evitar_com_parceiro, "toque_punicao": args.toque_punicao,
                  "disc_gp": args.disc_gp, "limite": args.limite, "apoio_premio": args.apoio_premio,
                  "protecao_alivio": args.protecao_alivio, "flexao": args.flexao, "obstaculo": args.obstaculo,
                  "peso_em_cima": args.peso_em_cima, "ambientes": list(args.ambientes),
                  "evitar_ambientes": args.evitar_ambientes,
                  "andar_velocidade": list(args.andar_velocidade), "trote_velocidade": list(args.trote_velocidade),
                  "empurrao_mov_forca": list(args.empurrao_mov_forca), "empurrao_mov_continuo": args.empurrao_mov_continuo,
                  "empurrao_mov_continuo_forca": list(args.empurrao_mov_continuo_forca),
                  "empurrao_continuo": args.empurrao_continuo, "empurrao_continuo_forca": list(args.empurrao_continuo_forca),
                  "evitar_forca": list(args.evitar_forca),
                  "empurrao_forca": list(args.empurrao_forca), "velocidade": list(args.velocidade),
                  "corrida": args.corrida, "sprint": args.sprint, "locomocao_clipe": args.locomocao_clipe,
                  "locomocao_segundos": args.locomocao_segundos, "queda_encerra": args.queda_encerra,
                  "peso_tarefa_loco": args.peso_tarefa_loco,
                  "peso_estilo_loco": args.peso_estilo_loco, "suave_loco": args.suave_loco, "passo": args.passo,
                  "mistura_clipe": args.mistura_clipe, "queda_programada": args.queda_programada,
                  "levantar_guiado": args.levantar_guiado, "levantar_segundos": args.levantar_segundos,
                  "levantar_clipe": args.levantar_clipe,
                  "estilo_chao": args.estilo_chao, "chao_altura": args.chao_altura, "rolar_clipe": args.rolar_clipe,
                  "musculo": args.musculo,
                  "sincronizar_fase": args.sincronizar_fase, "imitacao_folga": args.imitacao_folga, "peito_vista": args.peito_vista, "bracos_clipe": args.bracos_clipe, "estrela": args.estrela, "estrela_segundos": args.estrela_segundos, "comando": args.comando, "comando_segundos": args.comando_segundos, "parada": args.parada, "parada_clipes": args.parada_clipes, "passos_ciclos": args.passos_ciclos, "levantar_voo": args.levantar_voo, "lateral_pernas_rigor": args.lateral_pernas_rigor, "lento": args.lento, "cabeca_clipe": args.cabeca_clipe, "cabeca_autoridade": args.cabeca_autoridade, "fogo_fatia": args.fogo_fatia if args.fogo >= 0 else 0.0, "fogo_pontos": args.fogo_pontos, "fogo_alvo": args.fogo_alvo, "diagonal_peito": args.diagonal_peito, "rumo_suave": args.rumo_suave, "rumo_limite": args.rumo_limite, "sem_potencial_loco": args.sem_potencial_loco, "nota_velocidade": args.nota_velocidade, "estilo_velocidade": args.estilo_velocidade, "joelho_dentro": args.joelho_dentro, "idle_altura": args.idle_altura, "parada_corpo": args.parada_corpo, "lento_faixa": args.lento_faixa,
                  "manobras": args.manobras,
                  "comeco_clipe": args.comeco_clipe, "comeco_de_pe": args.comeco_de_pe,
                  "imitacao_parado": args.imitacao_parado, "idle_pos_levantar": args.idle_pos_levantar,
                  "parado_locomocao": args.parado_locomocao, "idle_alcance": args.idle_alcance, "mao_tronco": args.mao_tronco,
                  "idle_entrega": args.idle_entrega, "idle_entrega_alcance": args.idle_entrega_alcance,
                  "entrega_subindo": args.entrega_subindo,
                  "naturalidade": args.naturalidade, "trote_lateral_sem_puxao": args.trote_lateral_sem_puxao,
                  "corrida_lateral_pernas": args.corrida_lateral_pernas,
                  "velocidades_trote": gait_speeds(args.velocidade_trote),
                  "velocidades_corrida": gait_speeds(args.velocidade_corrida),
                  "largada_costas": args.largada_costas, "curva_fora": args.curva_fora, "curva_inclinar": args.curva_inclinar, "pisada_largura": args.pisada_largura,
                  "troca_freio": args.troca_freio, "troca_aceleracao": args.troca_aceleracao, "troca_pe": args.troca_pe, "troca_assentada": args.troca_assentada, "troca_arrancada": args.troca_arrancada, "troca_pratica": args.troca_pratica, "corrida_largada": args.corrida_largada, "rumo_cauda": args.rumo_cauda, "inclinacao_cauda": args.inclinacao_cauda, "sem_penhascos": args.sem_penhascos, "penhascos_partes": args.penhascos_partes, "arrancada_pulo": args.arrancada_pulo, "largada_lado": args.largada_lado, "troca_assentada_lado": args.troca_assentada_lado, "troca_assentada_frente": args.troca_assentada_frente, "troca_giro_tempo": args.troca_giro_tempo, "parada_passada": args.parada_passada, "idle_pose": args.idle_pose, "parada_clipes_frente": args.parada_clipes_frente, "parada_clipes_comando": args.parada_clipes_comando, "parada_portao": args.parada_portao, "parada_livre": args.parada_livre, "parada_postura": args.parada_postura, "parada_pose_sempre": args.parada_pose_sempre, "parado_unico": args.parado_unico, "parada_direto_idle": args.parada_direto_idle, "idle_depois_freada": args.idle_depois_freada, "parado_na_locomocao": args.parado_na_locomocao, "parado_torto": args.parado_torto, "estrela_caminho": args.estrela_caminho, "estrela_curvas": args.estrela_curvas, "comando_levantar": args.comando_levantar, "comando_levantar_correndo": args.comando_levantar_correndo, "queda_vira_comando": bool(args.queda_vira_comando), "levantar_correndo_troca": args.levantar_correndo_troca, "levantar_e_ir": list(args.levantar_e_ir), "levantar_vista": args.levantar_vista, "levantar_brucos_tempo": args.levantar_brucos_tempo, "mao_atras": args.mao_atras, "levantar_lado_agora": bool(args.levantar_lado_agora), "levantar_imitar_ancora": bool(args.levantar_imitar_ancora), "clipe_sob_pelve": bool(args.clipe_sob_pelve), "parado_preso": bool(args.parado_preso), "pernas_alinhadas": args.pernas_alinhadas, "parado_lugar": args.parado_lugar, "revisao_15h": bool(args.revisao_15h), "largada_margem_lado": bool(args.largada_margem_lado), "corrida_largura": args.corrida_largura, "parado_centro": args.parado_centro, "largada_maos": args.largada_maos, "velocidade_largura": args.velocidade_largura, "pernas_alinhadas_comando": bool(args.pernas_alinhadas_comando), "locomocao_ambiente": args.locomocao_ambiente, "referencial_clipe": bool(args.referencial_clipe), "fase_dois_pes": bool(args.fase_dois_pes), "rumo_medio": list(args.rumo_medio), "forca_contatos": bool(args.forca_contatos), "chao_grama": args.chao_grama, "reta_comando": bool(args.reta_comando), "velocidade_rotacao": bool(args.velocidade_rotacao), "imitar_velocidade": bool(args.imitar_velocidade), "ancora_parado": bool(args.ancora_parado), "ancora_fogo": args.ancora_fogo, "projeteis": list(args.projeteis), "lado_medio": list(args.lado_medio), "imitacao_portao": list(args.imitacao_portao), "simetria_comando": bool(args.simetria_comando), "ancora_sem_locomocao": bool(args.ancora_sem_locomocao), "loco_saidas_vivas": bool(args.loco_saidas_vivas), "teto_custo_tarefa": bool(args.teto_custo_tarefa), "ppo_clipe_duplo": args.ppo_clipe_duplo, "levantar_chegada_de_pe": args.levantar_chegada_de_pe, "locomocao_fogo": args.locomocao_fogo, "locomocao_fogo_liberdade": args.locomocao_fogo_liberdade, "locomocao_fogo_intervalo": args.locomocao_fogo_intervalo, "parado_torto_sem_fogo": args.parado_torto_sem_fogo, "locomocao_fogo_prova": args.locomocao_fogo_prova, "giro_variado": args.giro_variado, "giro_coluna": args.giro_coluna, "quieto_custos": args.quieto_custos, "parado_estatico": args.parado_estatico, "parado_segundos": args.parado_segundos, "parado_micro": args.parado_micro, "parado_fora_pose": args.parado_fora_pose, "parado_recompensa": args.parado_recompensa, "rigor_superior": args.rigor_superior, "arrancada_unreal": args.arrancada_unreal, "arrancada_unreal_escala": args.arrancada_unreal_escala, "largada_viva": args.largada_viva, "largada_passo": args.largada_passo, "largada_no_lugar": args.largada_no_lugar, "largada_inclinar": args.largada_inclinar, "largada_velocidade_parcela": args.largada_velocidade_parcela, "largada_imitar": args.largada_imitar, "levantar_unreal_reforco": list(args.levantar_unreal_reforco), "levantar_unreal_reforco_correndo": args.levantar_unreal_reforco_correndo, "levantar_unreal_reforco_brucos": args.levantar_unreal_reforco_brucos, "levantar_imitar_ritmo_correndo": args.levantar_imitar_ritmo_correndo, "levantar_imitar_nota": args.levantar_imitar_nota, "levantar_imitar": args.levantar_imitar, "largada_bracos_calmos": args.largada_bracos_calmos, "coluna_sem_velocidade": args.coluna_sem_velocidade, "ruido_parado": args.ruido_parado, "tonus": args.tonus, "postura_cauda": args.postura_cauda, "tonus_coluna": args.tonus_coluna, "custo_batente": args.custo_batente, "pes_firmes": args.pes_firmes, "gravidade_motores": args.gravidade_motores, "postura_base": args.postura_base, "largada_clipe": args.largada_clipe, "largada_mao": args.largada_mao, "parada_clipes_base": args.parada_clipes_base, "parada_variantes": args.parada_variantes, "parada_variantes_mascara": args.parada_variantes_mascara, "parada_entrada": args.parada_entrada, "troca_livre": args.troca_livre, "parada_pose": args.parada_pose, "parado_livre": args.parado_livre, "pose_reforco": args.pose_reforco, "empurrao_livre": args.empurrao_livre, "parado_semelhanca": args.parado_semelhanca, "passo_baixo": args.passo_baixo, "parado_dobrado_encerra": args.parado_dobrado_encerra, "parado_largo": args.parado_largo, "simetria_ciclos": args.simetria_ciclos, "peso_tarefa_parado": args.peso_tarefa_parado, "peso_estilo_parado": args.peso_estilo_parado, "troca_assentada_costas": args.troca_assentada_costas, "troca_peito_fixo": args.troca_peito_fixo, "capacidade_minutos": args.capacidade_minutos, "pegada_cruzada": args.pegada_cruzada, "estab_ancora": args.estab_ancora,
                  "trote_frente_inclinar": list(args.trote_frente_inclinar),
                  "corrida_inclinar": list(args.corrida_inclinar),
                  "corrida_costas_inclinar": list(args.corrida_costas_inclinar),
                  "sprint_velocidade": args.sprint_velocidade, "sprint_inclinar": list(args.sprint_inclinar),
                  "mao_corpo": args.mao_corpo, "equilibrio": args.equilibrio, "parado_ancora_obs": args.parado_ancora_obs, "equilibrio_fatia": args.equilibrio_fatia,
                  "empurrao_idle": args.empurrao_idle, "chegada_premio": args.chegada_premio, "voo_multa": args.voo_multa,
                  "subida_equilibrio": args.subida_equilibrio, "entrega_calma": args.entrega_calma,
                  "evitar_idle": args.evitar_idle,
                  "troca_inclinar": args.troca_inclinar,
                  "idle_alcance_ameaca": args.idle_alcance_ameaca}
    if args.servico:
        info["comando"] = ["/usr/bin/systemctl", "--user", "start", args.servico]
        # (04/11 06h30: com o treino parado o laboratorio abre no "principal" - um A/B antigo marcado assim fazia o
        # "Continuar" religar o treino errado. O do servico e o unico principal.)
        info["principal"] = True
        for other in run.parent.glob("*/run.json"):
            try:
                config = json.loads(other.read_text())
            except (OSError, ValueError):
                continue
            if other.parent != run and isinstance(config, dict) and config.get("principal"):
                config["principal"] = False
                other.write_text(json.dumps(config, ensure_ascii=False))
    (run / "run.json").write_text(json.dumps(info, ensure_ascii=False))
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    prova = None
    if args.prova:
        from prova import Prova
        device = Prova.preparar()
        prova = Prova(args.prova, sys.argv)
    torch.manual_seed(args.semente)
    v2 = montar_v2(args)
    batch = get_up_batch(str(REPO / "assets"), environments=args.mundos, threads=args.threads, **lote_do_treino(args), **v2)
    muscle = json.loads(Path(args.musculo).read_text()) if args.musculo else None
    if muscle:
        batch.set_student_actuator(muscle["rigidez"], muscle["amortecimento"], muscle["teto"], bool(muscle["gravidade"]))
    # (31/10 23h40, ele: "verifica se... nao conseguimos mais decisoes por segundo": as avaliacoes numa thread so paravam
    # o treino ~21 s a cada 50 atualizacoes, 13% do tempo; com as threads do treino, as que esperam ficam paradas.)
    eval_threads = args.threads
    evaluation = get_up_batch(str(REPO / "assets"), environments=64, threads=eval_threads, seed=12345, evaluation=True)
    # (O v2: 32 quedas fixas com alvo e 32 empurroes fortes fixos.)
    evaluations = {"mov_": get_up_batch(str(REPO / "assets"), environments=32, threads=eval_threads, seed=777,
                                                evaluation=True, evaluation_kind=1, **v2)} if args.movimento > 0 else {}
    if args.evitar > 0:
        evaluations["evitar_"] = get_up_batch(str(REPO / "assets"), environments=32, threads=eval_threads, seed=999,
                                                      evaluation=True, evaluation_kind=2,
                                                      partner_avoid=args.evitar_com_parceiro, **v2)
    # Equilíbrio parado sem empurrões; estabilidade e IDLE ALERTA.
    evaluations["equil_"] = get_up_batch(str(REPO / "assets"), environments=32, threads=eval_threads, seed=555,
                                                 evaluation=True, evaluation_kind=3, **v2)
    # (A locomocao, com a regua fixa - nao a do treino, que sobe: os rumos a 2,5 m/s e, quando o treino tem empurroes em
    # movimento, os do comeco do B21, um por episodio. 32 nos rumos, 4 por rumo; 8 no giro, metade para cada lado; 8
    # parados, sem empurrao.)
    if args.locomocao > 0 or args.giro > 0:
        moving = max(args.empurrao_mov_forca) > 0 or max(args.empurrao_mov_continuo_forca) > 0
        ruler = dict(v2, trot_speed=(2.5, 2.5), moving_push_newtons=(300.0, 600.0) if moving else (0.0, 0.0),
                     moving_push_hold_fraction=0.3,
                     moving_push_hold_newtons=(100.0, 180.0) if moving else (0.0, 0.0))
        evaluations["loco_"] = get_up_batch(str(REPO / "assets"), environments=32, threads=eval_threads, seed=444,
                                                    evaluation=True, evaluation_kind=4, **ruler)
        evaluations["giro_"] = get_up_batch(str(REPO / "assets"), environments=8, threads=eval_threads, seed=333,
                                                    evaluation=True, evaluation_kind=6, **ruler)
        evaluations["parado_"] = get_up_batch(str(REPO / "assets"), environments=8, threads=eval_threads, seed=222,
                                                      evaluation=True, evaluation_kind=5, **ruler)
        # (A rl_final: cada faixa para a frente, de pe, na velocidade do clipe dela - andar 1,1, trote 2,6, corrida
        # 4,3 e sprint 6,0 m/s -, 16 mundos cada, sem empurrao.)
        bands = ([("andar_", 0)] if args.andar > 0 else []) + [("trote_", 1)] + ([("corrida_", 2)] if args.corrida > 0 else []) \
            + ([("sprint_", 3)] if args.sprint > 0 else [])
        for prefix, band in bands:
            evaluations[prefix] = get_up_batch(
                str(REPO / "assets"), environments=16, threads=eval_threads, seed=600 + band, evaluation=True,
                evaluation_kind=4, evaluation_band=band, **dict(v2, moving_push_newtons=(0.0, 0.0),
                                                                moving_push_hold_newtons=(0.0, 0.0)))
    if args.manobras > 0:
        # (As manobras sem forca: as 7, duas vezes cada, no trote, de pe.)
        evaluations["manobras_"] = get_up_batch(
            str(REPO / "assets"), environments=14, threads=eval_threads, seed=888, evaluation=True, evaluation_kind=4,
            evaluation_band=1, **dict(v2, moving_push_newtons=(0.0, 0.0), moving_push_hold_newtons=(0.0, 0.0)))
        evaluations["manobras_"].set_maneuvers(0.0, 0)
        evaluations["manobras_"].set_tracking_reward(19)
    if args.levantar_deitado:
        # (07/11: o levantar deitado de costas e de brucos, 16 de cada, e o levantar correndo - o mov_ - deitado.)
        for prefix, kind in (("costas_", 9), ("brucos_", 10)):
            evaluations[prefix] = get_up_batch(str(REPO / "assets"), environments=16, threads=eval_threads, seed=600 + kind,
                                               evaluation=True, evaluation_kind=kind, **v2)
        for prefix in ("mov_", "costas_", "brucos_"):
            if prefix in evaluations:
                evaluations[prefix].add_style_clips(LYING_STAND + LYING_RUN)
                evaluations[prefix].set_lying_starts(LYING_STAND, LYING_RUN)
                evaluations[prefix].set_moving_handoff(True)
    for key, other in [("", evaluation), *evaluations.items()]:
        if muscle:
            other.set_student_actuator(muscle["rigidez"], muscle["amortecimento"], muscle["teto"], bool(muscle["gravidade"]))
    falls = {"mtime": 0.0}

    def load_falls() -> None:
        """As quedas da rede da locomocao (o arquivo do treino dela, quando muda) e, das primeiras 32, para sempre, a
        avaliacao quedas_loco_ (tambem em .f32 cru, para o laboratorio)."""
        file = run / "quedas_locomocao.npy"
        if args.quedas_locomocao <= 0 or not file.exists() or file.stat().st_mtime <= falls["mtime"]:
            return
        falls["mtime"] = file.stat().st_mtime
        states = np.ascontiguousarray(np.load(file), dtype=np.float32)
        batch.set_fall_starts(states)
        fixed = run / "quedas_locomocao_aval.npy"
        if not fixed.exists() and len(states) >= 32:
            np.save(fixed, states[:32])
            states[:32].tofile(run / "quedas_locomocao_aval.f32")
        if fixed.exists() and "quedas_loco_" not in evaluations:
            other = get_up_batch(str(REPO / "assets"), environments=32, threads=eval_threads, seed=111,
                                         evaluation=True, evaluation_kind=7, **v2)
            other.set_fall_starts(np.ascontiguousarray(np.load(fixed), dtype=np.float32))
            evaluations["quedas_loco_"] = other
        print(f"quedas da locomocao: {batch.fall_start_count}", flush=True)

    load_falls()
    partner = {"mtime": 0.0}

    def load_partner() -> None:
        """A rede da locomocao parceira, a exportada por ultimo do treino dela (de novo quando ele exporta outra), no
        treino e na avaliacao do evitar a queda."""
        if not args.parceiro:
            return
        source = run.parent / args.parceiro
        policy, task = source / "exports" / "policy_latest.mrlpolicy", source / "tarefa.json"
        try:
            mtime = policy.stat().st_mtime
            if mtime <= partner["mtime"]:
                return
            iteration = batch.set_locomotion_partner(str(REPO / "assets"), str(task), str(policy))
            if "evitar_" in evaluations:
                evaluations["evitar_"].set_locomotion_partner(str(REPO / "assets"), str(task), str(policy))
            partner["mtime"] = mtime
            print(f"parceira: {args.parceiro} rodada {iteration}", flush=True)
        except (OSError, RuntimeError, ValueError) as error:
            print(f"parceira: nao carregou ({error})", flush=True)

    load_partner()
    n, obs_size, act_size, amp = batch.num_envs, batch.obs_size, batch.action_size, batch.amp_size
    # (05/11, o vies para a direita) O espelho das entradas e das acoes: mede a assimetria da cabeca da locomocao em toda
    # atualizacao e, com --simetria, a perda de simetria.
    mirror = Espelho(batch.observation_names(), batch.action_names()) if hasattr(batch, "mirror_info") else None
    loco_style_clips = list(LOCO_STYLE_CLIPS)
    mm_names = None
    if args.juiz_unreal:
        unreal_loco, unreal_recovery = unreal_style_clips(transitions_only=args.juiz_ciclos > 0)
        added = batch.add_style_clips(unreal_loco + unreal_recovery + (list(JUIZ_CICLOS) if args.juiz_ciclos > 0 else []))
        loco_style_clips = unreal_loco + list(UNREAL_KEEP_LOCO)
        if args.juiz_mm:
            spec = json.loads((HERE / "juiz_mm_bancos.json").read_text())
            groups = spec["grupos"]
            batch.add_style_clips(sorted({c for g in groups for c in g["clipes"]}))
            names = [g["nome"] for g in groups]
            mm_names = names
            frames = batch.build_motion_matching(names, [g["esquema"] for g in groups], [g["clipes"] for g in groups],
                                                 [tuple(g["vieses"]) for g in groups], [int(g["categoria"]) for g in groups],
                                                 names.index(spec["principal"]))
            # (A previsao do CMC do Game Animation Sample nas velocidades do nosso corpo, x 1,104: aceleracao 800 cm/s^2,
            # frenagem sem pedido 2000 cm/s^2 e sem atrito, atrito 5, o rumo na hora no chao.)
            batch.set_motion_movement(8.8, 22.1, 5.0, 0.0, -1.0)
            counts = ", ".join(f"{g['nome']} {len(g['clipes'])}" for g in groups)
            print(f"juiz com Motion Matching: {len(groups)} bancos, {frames} quadros ({counts})", flush=True)
        if args.levantar_deitado:
            batch.set_lying_starts(LYING_STAND, LYING_RUN)
            # (07/11, ele: "as vezes nasce em pe": o lote sorteia os comecos ja no construtor - recomecado com os deitados.)
            batch.set_moving_handoff(True)
            batch.reset(args.semente)
            print(f"comecos deitados: {LYING_STAND} (clipe) e {LYING_RUN} (levantar correndo)", flush=True)
        if args.levantar_unreal > 0:
            configurar_levantar(batch, args, args.levantar_unreal)
            if args.levantar_unreal_pesos is not None:
                if args.levantar_unreal_prioridade > 0:
                    raise SystemExit("--levantar-unreal-pesos e --levantar-unreal-prioridade nao se misturam")
                batch.set_clip_get_up_weights([float(x) for x in args.levantar_unreal_pesos])
            batch.reset(args.semente)
            ends = batch.clip_get_up_ends()
            print("levantar pelo clipe: " + "; ".join(f"{k} {ends[3 * i]:.2f} s, {ends[3 * i + 1]:.2f} m/s no fim"
                                                     for i, k in enumerate(LEVANTAR_UNREAL_KEYS)), flush=True)
        reference = torch.as_tensor(batch.reference_pairs_of(unreal_recovery + list(UNREAL_KEEP_RECOVERY)), device=device)
        print(f"juiz da Unreal: {added} clipes carregados; recuperacao {len(reference)} pares de {len(unreal_recovery)} levantadas "
              f"+ {UNREAL_KEEP_RECOVERY}", flush=True)
    else:
        reference = torch.as_tensor(batch.reference_pairs(), device=device)
    model = Policy(obs_size, act_size, batch.action_ranges(), split=args.habilidades_separadas).to(device)
    # (14/11 16h25, --levantar-costas-adaptador TAXA) O ramo do de costas, fora do grupo do ator (o Adam do ator segue o mesmo), com o
    # otimizador dele; a porta na marca normalizada (o limiar posto depois de carregar a normalizacao).
    adapter_opt = None
    if args.levantar_costas_adaptador > 0:
        if args.levantar_costas_marca <= 0:
            raise SystemExit("--levantar-costas-adaptador precisa de --levantar-costas-marca")
        model.enable_costas_adapter(obs_size, act_size, list(batch.observation_names()).index("levantar.variante"), 1.0e9)
        adapter_opt = torch.optim.Adam(model.costas_adapter.parameters(), lr=args.levantar_costas_adaptador)
    if args.levantar_costas_enxerto:
        if args.levantar_costas_marca <= 0:
            raise SystemExit("--levantar-costas-enxerto precisa de --levantar-costas-marca")
        expert_src = Policy(obs_size, act_size, batch.action_ranges(), split=True)
        expert_src.load_state_dict(torch.load(Path(args.levantar_costas_enxerto).expanduser(), map_location="cpu", weights_only=False)["model"])
        obs_names_e = list(batch.observation_names())
        model.enable_costas_expert(expert_src, obs_names_e.index("levantar.variante"), args.levantar_costas_marca,
                                   torch.tensor([n.startswith("chao.contato.") for n in obs_names_e]),
                                   torch.tensor([n.startswith("fogo.forca.") for n in obs_names_e]))
        print(f"levantar: o enxerto do de costas e {args.levantar_costas_enxerto} (congelado; no de costas levantando sem empurrao)", flush=True)
    if args.levantar_brucos_enxerto:
        if not args.levantar_costas_enxerto or args.levantar_brucos_marca >= 0.0:
            raise SystemExit("--levantar-brucos-enxerto precisa de --levantar-costas-enxerto e de --levantar-brucos-marca negativa")
        brucos_src = Policy(obs_size, act_size, batch.action_ranges(), split=True)
        brucos_src.load_state_dict(torch.load(Path(args.levantar_brucos_enxerto).expanduser(), map_location="cpu", weights_only=False)["model"])
        model.enable_brucos_expert(brucos_src, args.levantar_brucos_marca)
        print(f"levantar: o enxerto do de brucos e {args.levantar_brucos_enxerto} (marca {args.levantar_brucos_marca}; a base a ve zerada)", flush=True)
    piso_do_desvio(model, batch, args, device)
    disc = Discriminator(amp).to(device)
    disc_loco = reference_loco = disc_loco_opt = None
    mm = None
    if args.estilo_loco_proprio:
        reference_loco = torch.as_tensor(batch.reference_pairs_of(loco_style_clips), device=device)
        if args.juiz_unreal and args.juiz_ciclos > 0:
            # (Os ciclos sao curtos - ~5 s no total contra ~700 s de transicoes: repetidos ate a fracao pedida dos pares.)
            cycles = torch.as_tensor(batch.reference_pairs_of(list(JUIZ_CICLOS)), device=device)
            if len(cycles) == 0:
                raise SystemExit("--juiz-ciclos: nenhum par dos ciclos (os ciclo_*.matteranim.json existem?)")
            copies = max(1, round(args.juiz_ciclos / (1.0 - args.juiz_ciclos) * len(reference_loco) / len(cycles)))
            print(f"juiz da locomocao: {len(reference_loco)} pares das transicoes + {len(cycles)} dos ciclos x {copies}", flush=True)
            reference_loco = torch.cat([reference_loco] + [cycles] * copies)
        if args.juiz_mm:
            mm = MotionMatchingJudge(batch, device)
            mm.names = mm_names
            disc_loco = ContextDiscriminator(amp, mm.context).to(device)
        else:
            disc_loco = Discriminator(amp).to(device)
        disc_loco_opt = torch.optim.Adam(disc_loco.net.parameters(), lr=args.taxa_disc, weight_decay=1e-4)
        print(f"juiz da locomocao: {len(reference_loco)} pares de {len(loco_style_clips)} clipes"
              f"{'' if args.juiz_unreal else ' ' + str(LOCO_STYLE_CLIPS)}", flush=True)
    if args.taxa_ator_rec is not None and model.split:
        # (06/11 15h, a locomocao livre: com 5e-5 nas duas cabecas o levantar caiu de 97% para 73% em 50 atualizacoes - ela
        # aprende com ~6% dos episodios. Cada cabeca com a sua taxa: a recuperacao com a de quando avaliava 97-98%.)
        loco_params = list(model.actor.locomotion.parameters()) + [model.loco_log_std]
        loco_ids = {id(p) for p in loco_params}
        actor_opt = torch.optim.Adam([{"params": [p for p in model.actor_parameters() if id(p) not in loco_ids],
                                       "lr": args.taxa_ator_rec},
                                      {"params": loco_params}], lr=args.taxa_ator)
    elif args.taxa_adaptativa > 0 and model.split:
        # (05/11: um grupo por cabeca - a taxa de cada uma pelo KL dela.)
        loco_params = list(model.actor.locomotion.parameters()) + [model.loco_log_std]
        loco_ids = {id(p) for p in loco_params}
        actor_opt = torch.optim.Adam([{"params": [p for p in model.actor_parameters() if id(p) not in loco_ids]},
                                      {"params": loco_params}], lr=args.taxa_ator)
    else:
        actor_opt = torch.optim.Adam(model.actor_parameters(), lr=args.taxa_ator)
    critic_opt = torch.optim.Adam(model.critic.parameters(), lr=args.taxa_critico)
    disc_opt = torch.optim.Adam(disc.net.parameters(), lr=args.taxa_disc, weight_decay=1e-4)
    counters = {"atualizacao": 0, "amostras": 0, "episodios": 0}
    settings = {"mundos": n, "passos": args.passos, "residuo": 0.0, "peso_tarefa": args.peso_tarefa, "peso_estilo": args.peso_estilo,
                "comeco_clipe": args.comeco_clipe, "observacao": batch.observation_names(), "acao": batch.action_names(),
                "pares_dos_clipes": int(reference.shape[0]), "habilidades_separadas": args.habilidades_separadas}
    checkpoint = run / "ultimo.pt"
    state = None
    fresh_judges = False
    fresh_mm = False
    balance_calibration = None
    if checkpoint.exists():
        state = torch.load(checkpoint, map_location=device, weights_only=False)
        source_state = state["model"]
        first = "actor.recovery.0.weight" if "actor.recovery.0.weight" in source_state else "actor.0.weight"
        # (11/11, E5-6: as 2 entradas da ancora do parado antes da habilidade, a ultima; o Adam mantido, com zeros nelas.)
        anchor_at = source_state[first].shape[1] - 1 if args.parado_ancora_obs and source_state[first].shape[1] == obs_size - 2 else None
        if anchor_at is not None:
            source_state = insert_inputs(source_state, anchor_at, 2)
            print(f"ancora do parado na entrada: 2 colunas novas em zero na posicao {anchor_at}", flush=True)
        # (18h05: a variante do levantar pelo clipe - uma coluna antes da habilidade, peso zero: as duas cabecas fazem o mesmo.)
        variant_at = source_state[first].shape[1] - 1 if args.levantar_unreal_variante and source_state[first].shape[1] == obs_size - 1 else None
        if variant_at is not None:
            source_state = insert_inputs(source_state, variant_at, 1)
            print(f"variante do levantar na entrada: 1 coluna nova em zero na posicao {variant_at}", flush=True)
        # (12/11 06h40: as 6 entradas do fogo antes da habilidade, depois da variante; peso zero: a rede faz o mesmo de antes.)
        fire_at = source_state[first].shape[1] - 1 \
            if args.levantar_fogo_entrada and source_state[first].shape[1] == obs_size - 6 else None
        if fire_at is not None:
            source_state = insert_inputs(source_state, fire_at, 6)
            print(f"entradas do fogo: 6 colunas novas em zero na posicao {fire_at}", flush=True)
        grown = source_state[first].shape[1] < obs_size or (args.habilidades_separadas and first == "actor.0.weight")
        # (04/11 15h, os passos: a rede de antes no layout do equilibrio - as entradas antes da habilidade, os canais do passo.)
        old_inputs = source_state[first].shape[1]
        balance_grew = args.equilibrio and args.habilidades_separadas and first == "actor.recovery.0.weight" \
            and (old_inputs < obs_size or source_state["actor.recovery.4.weight"].shape[0] < act_size)
        if balance_grew:
            source_state = grow_balance(source_state, obs_size, act_size, model)
            grown = True
        # (Uma entrada nova no contrato - 31/10 22h15, o rumo pedido do corpo; 01/11, o v3, o corpo contra o chao: as
        # colunas novas em zero e o Adam do ator e do critico do zero, as formas mudaram.)
        loaded = split_state(expand_inputs(source_state, obs_size)) if args.habilidades_separadas else expand_inputs(source_state, obs_size)
        if model.has_costas_adapter:
            # (14/11 16h25) O ramo do de costas ainda nao salvo: o do comeco (a saida zero).
            for key, value in model.state_dict().items():
                if (key.startswith("costas_adapter.") or key == "costas_gate") and key not in loaded:
                    loaded[key] = value
        if model.has_costas_expert:
            for key, value in model.state_dict().items():
                if key.startswith(("costas_expert.", "expert_", "brucos_expert.", "brucos_")):
                    loaded[key] = value
        if not model.has_costas_expert and any(k.startswith(("costas_expert.", "expert_", "brucos_expert.", "brucos_")) for k in loaded):
            # (14/11 18h25, ele: "de forma alguma segunda rede") Sem o enxerto: os pesos dele no ponto ficam de fora - uma rede so.
            loaded = {k: v for k, v in loaded.items() if not k.startswith(("costas_expert.", "expert_", "brucos_expert.", "brucos_"))}
            print("levantar: sem enxerto (os pesos dele no ponto ignorados) - uma rede so", flush=True)
        if not model.has_costas_adapter and any(k.startswith("costas_adapter.") for k in loaded):
            # (14/11 17h58) O ramo desligado (sem --levantar-costas-adaptador): os pesos dele no ponto ficam de fora. Medido: o ramo aprendia
            # as correcoes do professor no limpo e as levava para debaixo do empurrao (pancadas 0,62 -> 0,25 no de costas).
            loaded = {k: v for k, v in loaded.items() if not (k.startswith("costas_adapter.") or k == "costas_gate")}
            print("levantar: o ramo do de costas desligado (os pesos dele no ponto ignorados)", flush=True)
        model.load_state_dict(loaded)
        if model.has_costas_adapter:
            # O limiar da porta: metade da marca na escala da normalizacao carregada (a de pe e os outros levantares ficam em 0).
            mark_at = int(model.costas_gate[0])
            norm_l = model.norm.locomotion if model.split else model.norm
            threshold = 0.5 * args.levantar_costas_marca / math.sqrt(float(norm_l.var[mark_at]) + 1e-8)
            model.costas_gate[1] = threshold
            print(f"levantar: o ramo do de costas na entrada {mark_at}, porta >= {threshold:.3f} (normalizada); taxa {args.levantar_costas_adaptador}",
                  flush=True)
            if "adapter_opt" in state:
                adapter_opt.load_state_dict(state["adapter_opt"])
        # (07/11 04h: a versao do conjunto do juiz - a corrida nova a 6,0/5,5 e os giros parados por angulo trocaram os clipes
        # depois de 90 atualizacoes com o conjunto antigo: os juizes de novo do zero, uma vez.)
        fresh_judges = args.juiz_unreal and state["counters"].get("juiz") != JUIZ_UNREAL_VERSAO
        fresh_mm = args.juiz_mm and state["counters"].get("juiz_loco") != "mm"
        fresh_loco = args.juiz_ciclos > 0 and state["counters"].get("juiz_loco_versao") != JUIZ_LOCO_VERSAO
        if fresh_loco:
            print("juiz da locomocao com os nossos ciclos pela primeira vez: ele (e o Adam dele) do zero; o do levantar mantido",
                  flush=True)
        if fresh_mm:
            print("juiz da locomocao com o Motion Matching pela primeira vez: ele (e o Adam dele) do zero", flush=True)
        if fresh_judges:
            print("juiz da Unreal pela primeira vez: os dois juizes (e os Adam deles) do zero; o ator e o critico mantidos", flush=True)
        else:
            disc.load_state_dict(state["disc"])
        if not grown and "actor_opt" in state and "critic_opt" in state:
            try:
                actor_opt.load_state_dict(state["actor_opt"])
            except ValueError:
                saved = state["actor_opt"]
                if len(saved["param_groups"]) == 1 and len(actor_opt.param_groups) == 2:
                    # (05/11: um grupo -> um por cabeca, os momentos do Adam mantidos - so a numeracao muda.)
                    old_ids = [id(p) for p in model.actor_parameters()]
                    new_ids = [id(p) for group in actor_opt.param_groups for p in group["params"]]
                    fresh = actor_opt.state_dict()
                    fresh["state"] = {new_ids.index(old_ids[int(i)]): v for i, v in saved["state"].items()}
                    actor_opt.load_state_dict(fresh)
                    print("adam do ator: um grupo por cabeca, os momentos mantidos", flush=True)
                else:
                    # (Os grupos do Adam mudaram: o ator do zero.)
                    print("adam do ator do zero (grupos novos)", flush=True)
            critic_opt.load_state_dict(state["critic_opt"])
            if variant_at is not None:
                print(f"adam com a variante: {insert_adam_inputs(actor_opt, variant_at, 1) + insert_adam_inputs(critic_opt, variant_at, 1)} "
                      "momentos com a coluna nova", flush=True)
            if anchor_at is not None:
                print(f"adam com a ancora: {insert_adam_inputs(actor_opt, anchor_at, 2) + insert_adam_inputs(critic_opt, anchor_at, 2)} "
                      "momentos com as colunas novas", flush=True)
            if fire_at is not None:
                print(f"adam com o fogo: {insert_adam_inputs(actor_opt, fire_at, 6) + insert_adam_inputs(critic_opt, fire_at, 6)} "
                      "momentos com as colunas novas", flush=True)
        if not fresh_judges:
            disc_opt.load_state_dict(state["disc_opt"])
        if disc_loco is not None and "disc_loco" in state and not fresh_judges and not fresh_mm and not fresh_loco:
            disc_loco.load_state_dict(state["disc_loco"])
            disc_loco_opt.load_state_dict(state["disc_loco_opt"])
        counters = state["counters"]
        if args.juiz_unreal:
            counters["juiz"] = JUIZ_UNREAL_VERSAO
        if args.juiz_ciclos > 0:
            counters["juiz_loco_versao"] = JUIZ_LOCO_VERSAO
        if args.juiz_mm:
            counters["juiz_loco"] = "mm"
        if model.split and args.piso_posicao > 0 and counters.get("piso_posicao") != args.piso_posicao:
            # (08/11) O desvio dos alvos de posicao da locomocao levado ao piso novo, uma vez por piso.
            with torch.no_grad():
                axes = torch.tensor([not (n.startswith("vel.") or n.startswith("passo.")) for n in batch.action_names()], device=device)
                before = float(model.loco_log_std[axes].exp().mean())
                model.loco_log_std[axes] = torch.maximum(model.loco_log_std[axes], torch.full_like(model.loco_log_std[axes], math.log(args.piso_posicao)))
            counters["piso_posicao"] = args.piso_posicao
            print(f"piso da posicao: desvio {before:.4f} -> {float(model.loco_log_std[axes].exp().mean()):.4f}", flush=True)
        if model.split and args.ruido_velocidade != 1.0 and counters.get("ruido_velocidade") != args.ruido_velocidade:
            # (07/11, a receita nova: o desvio da locomocao nos alvos de velocidade vezes o fator, uma vez por fator.)
            with torch.no_grad():
                axes = torch.tensor([n.startswith("vel.") for n in batch.action_names()], device=device)
                before = float(model.loco_log_std[axes].exp().mean())
                factor = args.ruido_velocidade / float(counters.get("ruido_velocidade", 1.0))
                model.loco_log_std[axes] += math.log(factor)
            counters["ruido_velocidade"] = args.ruido_velocidade
            print(f"ruido nos alvos de velocidade: desvio {before:.4f} -> {float(model.loco_log_std[axes].exp().mean()):.4f}",
                  flush=True)
        if balance_grew:
            # (A calibracao das entradas novas depois do alcance da correcao - com ele a coleta age como o treino.)
            balance_calibration = (old_inputs - 1, obs_size - 1)
            counters["equilibrio_desde_atualizacao"] = counters["atualizacao"]
    # (31/10 23h, ele: "nao consigo saber se ele esta ou nao treinando... agora essa passa a ser a nossa rede PRINCIPAL":
    # o status.json que o laboratorio le dos treinadores de processo proprio - vivo pelo pid, parar por sinal. O tempo
    # treinado vem dos contadores; sem ele, a soma dos intervalos curtos entre as linhas do log.)
    if "segundos" not in counters:
        horas = [json.loads(line).get("hora") for line in open(run / "log.jsonl")] if (run / "log.jsonl").exists() else []
        horas = [h for h in horas if isinstance(h, (int, float))]
        counters["segundos"] = float(sum(b - a for a, b in zip(horas, horas[1:]) if 0 < b - a < 600))
    opened = time.time()
    trained_before = counters["segundos"]
    last_status = {"sps": 0.0, "last": {}}

    def write_status(state: str) -> None:
        counters["segundos"] = trained_before + (time.time() - opened)
        data = {"run": args.treino, "task": args.treino, "state": state, "pid": os.getpid(),
                "taskName": "rl_final: locomocao pelo rastreio dos clipes",
                "iteration": counters["atualizacao"], "steps": counters["amostras"], "episodes": counters["episodios"],
                "trainSeconds": counters["segundos"], "sps": last_status["sps"], "threads": args.threads,
                "environments": n, "device": device.type, "cpuTemperature": cpu_temperature() or 0.0,
                "last": last_status["last"]}
        tmp = run / "status.tmp"
        tmp.write_text(json.dumps(data, ensure_ascii=False))
        tmp.replace(run / "status.json")

    write_status("treinando")
    log = open(run / "log.jsonl", "a")
    gamma, lam, clip, T = 0.99, 0.95, 0.2, args.passos
    minibatch = (T * n) // max(1, args.minilotes)
    replay = torch.zeros(0, 2 * amp, device=device)
    obs = torch.as_tensor(batch.observations(), device=device)
    window: list = []
    troca_window: list = []
    # A acao media anterior de cada mundo (o tranco) e se ela vale (nao no primeiro passo de um episodio).
    previous_action = torch.zeros(n, act_size, device=device)
    previous_valid = torch.zeros(n, dtype=torch.bool, device=device)
    # (05/11, --ruido-correlacionado: o estado do ruido de cada mundo e eixo, variancia 1.)
    colored, tau = args.ruido_correlacionado
    noise_state = torch.randn(n, act_size, device=device)
    noise_keep = math.exp(-batch.policy_dt / max(tau, 1.0e-3))
    with torch.no_grad():
        disc.norm.update(reference[:, :amp])
        if disc_loco is not None and (state is None or "disc_loco" not in state or fresh_judges or fresh_mm):
            disc_loco.norm.update(reference_loco[:, :amp])
    replay_loco = torch.zeros(0, 2 * (2 * amp + mm.context) if mm is not None else 2 * amp, device=device)
    # (11/11, E5-8, --juiz-andando: a E5-5 mostrou que o juiz aprendendo melhora a corrida e estraga o parado - o estilo do parado
    # fica com a copia congelada, a de antes; a normalizacao dela segue como a do juiz congelado do campeao.)
    # (A copia vai no checkpoint, "disc_loco_parado": continuar o treino segue com ela, nao com uma copia do juiz ja treinado andando.)
    disc_loco_idle = None
    if args.juiz_andando and disc_loco is not None:
        disc_loco_idle = copy.deepcopy(disc_loco).eval()
        if state and "disc_loco_parado" in state:
            disc_loco_idle.load_state_dict(state["disc_loco_parado"])
            print("juiz andando: o do parado vem do checkpoint (congelado)", flush=True)
        else:
            print("juiz andando: o do parado e a copia congelada do juiz do ponto de partida", flush=True)
        disc_loco_idle.requires_grad_(False)

    retained = None
    if args.preservar_peso > 0:
        retained = copy.deepcopy(model).eval()
        if state and "preservation_anchor" in state:
            retained.load_state_dict(split_state(expand_inputs(state["preservation_anchor"], obs_size)) if args.habilidades_separadas else expand_inputs(state["preservation_anchor"], obs_size))
        retained.requires_grad_(False)

    # (10/11, E5: a ancora no campeao; metodologia_rl.md, secao 8 item 2.) A acao media do campeao no banco e constante (a rede
    # e a normalizacao dele congeladas): calculada uma vez aqui. A distancia e a MSE das medias normalizadas, nao a KL: o desvio
    # da locomocao e um parametro fixo por eixo (o mesmo em todo estado, com pisos das flags), entao a KL so somaria o puxao do
    # desvio da exploracao e pesaria cada eixo por 1/(2 desvio^2) do campeao (~100x com 0,05-0,08), uma escala sem relacao com
    # a do PPO; a MSE e a forma da --preservar-peso e da imitacao, e a media e o que a suite mede.
    anchor_obs = anchor_target = anchor_generator = None
    if args.ancora > 0:
        if not args.ancora_banco:
            raise SystemExit("--ancora precisa de --ancora-banco")
        bank = torch.load(Path(args.ancora_banco).expanduser(), map_location="cpu", weights_only=False)
        if list(bank["meta"]["observacao"]) != list(batch.observation_names()):
            raise SystemExit(f"o banco da ancora tem outras entradas ({bank['obs'].shape[1]} contra {obs_size})")
        anchor_from = Path(args.ancora_de or bank["meta"]["checkpoint"]).expanduser()
        anchor_state = torch.load(anchor_from, map_location="cpu", weights_only=False)["model"]
        champion = copy.deepcopy(model).eval()
        champion.load_state_dict(split_state(anchor_state) if args.habilidades_separadas else anchor_state)
        anchor_obs = bank["obs"].to(device)
        with torch.no_grad():
            anchor_target = torch.cat([champion.actor(champion.norm(x)) for x in anchor_obs.split(8192)])
        del champion, anchor_state
        anchor_generator = torch.Generator(device=device)
        anchor_generator.manual_seed(args.semente * 104729 + counters["atualizacao"])
        info = json.loads((run / "run.json").read_text())
        info["ancora"] = {"peso": args.ancora, "banco": str(Path(args.ancora_banco).expanduser()), "estados": len(anchor_obs),
                          "de": str(anchor_from), "cenarios": bank["meta"]["cenarios"]}
        (run / "run.json").write_text(json.dumps(info, ensure_ascii=False))
        print(f"ancora no campeao: peso {args.ancora}, {len(anchor_obs)} estados do banco, rede {anchor_from}", flush=True)

    def export_policy() -> None:
        # (13/11 v1, a revisao: so depois da migracao - num treino novo com --de, a copia antes dela deixava a normalizacao da recuperacao
        # igual a da locomocao, e a mistura misturava 0 entradas sem avisar.)
        if args.cerebro_unico and counters.get("cerebro_unico_v"):
            sync_single_brain(model)
        export(model, run / "exports" / "policy_latest.mrlpolicy",
               {"run": args.treino, "iteration": counters["atualizacao"], "steps": counters["amostras"],
                "observationNames": settings["observacao"]})

    def save() -> None:
        nonlocal log
        tmp = run / "ultimo.tmp"
        if args.cerebro_unico and counters.get("cerebro_unico_v"):
            sync_single_brain(model)
        torch.save({"model": model.state_dict(), "disc": disc.state_dict(), "actor_opt": actor_opt.state_dict(),
                    **({"disc_loco": disc_loco.state_dict(), "disc_loco_opt": disc_loco_opt.state_dict()} if disc_loco else {}),
                    **({"disc_loco_parado": disc_loco_idle.state_dict()} if disc_loco_idle is not None else {}),
                    "critic_opt": critic_opt.state_dict(), "disc_opt": disc_opt.state_dict(), "counters": counters,
                    **({"adapter_opt": adapter_opt.state_dict()} if adapter_opt is not None else {}),
                    "settings": settings,
                    **({"preservation_anchor": retained.state_dict()} if retained is not None else {})}, tmp)
        tmp.replace(checkpoint)
        export_policy()
        # (As estatisticas do log condensadas por idade, como as do banco.)
        log.flush()
        if storage.condense_log(run / "log.jsonl"):
            log.close()
            log = open(run / "log.jsonl", "a")

    # (A taxa do discriminador da linha de comando tambem ao retomar - o estado do Adam guardado trazia a de antes; 01/11,
    # o estilo: o discriminador mais lento, junto com o gp maior.)
    for group in disc_opt.param_groups:
        group["lr"] = args.taxa_disc
    def copy_recovery_to_locomotion() -> None:
        names = batch.observation_names()
        # (O comando e a entrada da habilidade - a ultima, 1 na locomocao e sempre 0 na recuperacao: com ela em 1 a copia
        # nao era mais a recuperacao e caia - 22h45.)
        command = [i for i, name in enumerate(names) if name.startswith("comando.")] + [len(names) - 1]
        with torch.no_grad():
            model.actor.locomotion.load_state_dict(model.actor.recovery.state_dict())
            model.critic.locomotion.load_state_dict(model.critic.recovery.state_dict())
            model.norm.locomotion.load_state_dict(model.norm.recovery.state_dict())
            model.norm.locomotion.count.fill_(1.0e4)
            for head in (model.actor.locomotion, model.critic.locomotion):
                head[0].weight[:, command] = 0.0
            model.loco_log_std.fill_(math.log(args.desvio if args.desvio is not None else 0.3))
        for parameter in list(model.actor.locomotion.parameters()) + [model.loco_log_std]:
            actor_opt.state.pop(parameter, None)
        for parameter in model.critic.locomotion.parameters():
            critic_opt.state.pop(parameter, None)
        counters["loco_da_recuperacao"] = counters["atualizacao"]
        counters["loco_da_recuperacao_v"] = 2
        print(f"cabeca da locomocao = copia da recuperacao (comando zerado: colunas {command})", flush=True)

    if args.congelar_locomocao:
        # (11/11 14h: a locomocao congelada de verdade - sem gradiente: fora do PPO ela ja ficava pela mascara, mas a simetria e a
        # ancora tambem mexiam na cabeca dela. O Adam pula parametro sem gradiente.)
        frozen = list(model.actor.locomotion.parameters()) + list(model.critic.locomotion.parameters()) + [model.loco_log_std]
        for parameter in frozen:
            parameter.requires_grad_(False)
        print(f"locomocao congelada: {sum(x.numel() for x in frozen)} parametros sem gradiente", flush=True)

    # (04/11: num treino novo com --de a copia vem depois da rede de origem - antes ela era apagada pela carga.)
    if args.loco_da_recuperacao and model.split and counters.get("loco_da_recuperacao_v") != 2 and checkpoint.exists():
        copy_recovery_to_locomotion()
    # (As taxas da linha de comando tambem ao retomar - o Adam guardado trazia as de antes; a revisao de 03/11.)
    for i, group in enumerate(actor_opt.param_groups):
        group["lr"] = args.taxa_ator
    if args.taxa_ator_rec is not None and model.split and len(actor_opt.param_groups) == 2:
        actor_opt.param_groups[0]["lr"] = args.taxa_ator_rec
        print(f"taxas: recuperacao {args.taxa_ator_rec:.3g}, locomocao {args.taxa_ator:.3g}", flush=True)
    if args.taxa_adaptativa > 0 and len(actor_opt.param_groups) == 2:
        # (A taxa de cada cabeca que o ajuste achou, ao retomar.)
        for group, lr in zip(actor_opt.param_groups, counters.get("taxa_cabecas", [args.taxa_ator, args.taxa_ator])):
            group["lr"] = lr
        if args.taxa_adaptativa_loco == 0 and not args.cerebro_unico:
            actor_opt.param_groups[1]["lr"] = args.taxa_ator
        print(f"taxa adaptativa (KL alvo {args.taxa_adaptativa}): recuperacao {actor_opt.param_groups[0]['lr']:.3g}, "
              f"locomocao {actor_opt.param_groups[1]['lr']:.3g}", flush=True)
    for group in critic_opt.param_groups:
        group["lr"] = args.taxa_critico
    if args.desvio is not None:
        with torch.no_grad():
            (model.loco_log_std if model.split else model.log_std).fill_(math.log(args.desvio))
    # (17h40: sem as velocidades - na recuperacao, que nao as usa, o log da probabilidade fica so com os eixos.)
    preparar_politica(model, batch, args)
    if args.desvio_recuperacao is not None and counters.get("desvio_recuperacao") != args.desvio_recuperacao:
        # (20h40) Uma vez por valor: o desvio dos eixos das juntas da recuperacao (as velocidades ficam).
        counters["desvio_recuperacao"] = args.desvio_recuperacao
        joints = len(model.joint_names)
        with torch.no_grad():
            before = float(model.log_std[:joints].exp().mean())
            model.log_std[:joints] = math.log(args.desvio_recuperacao)
        print(f"desvio da recuperacao: {before:.3f} -> {args.desvio_recuperacao} nos {joints} eixos das juntas", flush=True)
    if args.levantar_velocidade is not None and counters.get("levantar_velocidade_desvio") != args.levantar_velocidade:
        # (O desvio das velocidades da recuperacao, uma vez por valor: comecam em zero - o levantar de antes - e exploram pouco.)
        counters["levantar_velocidade_desvio"] = args.levantar_velocidade
        first = len(model.joint_names) + model.step_count
        with torch.no_grad():
            model.log_std[first:] = math.log(args.levantar_velocidade)
        print(f"levantar com a velocidade: desvio {args.levantar_velocidade} nas {model.log_std.numel() - first} velocidades", flush=True)

    if "soltar" in counters:
        # shortcut: a professora do --soltar (apagada na limpeza c2) nascia aqui e gastava o gerador da CPU - uma Policy com a
        # acao sem as velocidades; este gasto so mantem a prova de equivalencia igual ao ouro. Sai quando o ouro for refeito.
        kept = act_size - sum(1 for nm in batch.action_names() if nm.startswith("vel."))
        Policy(obs_size, kept, batch.action_ranges()[:kept], split=True)

    if not checkpoint.exists() and args.de:
        source = torch.load(DATA / args.de / "ultimo.pt", map_location=device, weights_only=False)
        grown_source = expand_inputs(source["model"], obs_size)
        model.load_state_dict(split_state(grown_source) if model.split else grown_source)
        disc.load_state_dict(source["disc"])
        row = {"atualizacao": 0, "de": args.de, "de_atualizacao": source["counters"]["atualizacao"]}
        log.write(json.dumps(finite_json(row)) + "\n")
        log.flush()
        print(f"comeca da rede de {args.de} (atualizacao {row['de_atualizacao']})", flush=True)
        # (03/11 18h20: o --desvio vale tambem ao comecar de outra rede - o load_state_dict trazia o desvio dela por cima.)
        if args.desvio is not None:
            with torch.no_grad():
                (model.loco_log_std if model.split else model.log_std).fill_(math.log(args.desvio))
        if args.loco_da_recuperacao and model.split:
            copy_recovery_to_locomotion()
        if model.split and not args.loco_da_recuperacao:
            print(f"normalizacao da locomocao: {calibrate_locomotion_norm(model, batch, device)}", flush=True)
        export_policy()
    elif not checkpoint.exists() and args.professor_passos > 0:
        row = {"atualizacao": 0, **warm_start(model, args.threads, args.professor_passos, device)}
        log.write(json.dumps(finite_json(row)) + "\n")
        log.flush()
        print("aprendeu com a rede de brucos: " + " ".join(f"{k}={v}" for k, v in row.items()), flush=True)
        export_policy()

    # (31/10 23h50, ele: "aquelas paradas de sobreposicao... fazer melhor uso da gpu": a atualizacao do PPO e do
    # discriminador numa thread propria, sobreposta a coleta seguinte, como a do B21. Medido antes: a fisica e 97% da
    # coleta - 2,41 de 2,48 s - e a atualizacao 0,41 s, 14% da rodada; a GPU ficava em 1-20%. A coleta usa a rede de uma
    # atualizacao atras; o PPO e o desacoplado (Hilton et al. 2021, como o do B21): o corte contra a rede do comeco da
    # atualizacao e o peso de importancia da coleta, cortado em 2.)
    if args.cerebro_unico:
        if not counters.get("cerebro_unico_v"):
            # (13/11) Uma vez: a normalizacao da mistura com a primeira camada compensada, sem o Adam da camada antiga (os momentos eram
            # da outra parametrizacao), e o critico aquece antes do ator mexer (aquecer_desde, como no zeramento do levantar).
            gap, merged = merge_norm_single(model, args.cerebro_unico_mistura, list(batch.observation_names()))
            if gap > 1.0e-3:
                raise SystemExit(f"cerebro unico: a compensacao da normalizacao mudou a locomocao em {gap:.2e} - abortado")
            for head, opt in ((model.actor.locomotion, actor_opt), (model.critic.locomotion, critic_opt)):
                for parameter in head[0].parameters():
                    opt.state.pop(parameter, None)
            counters["cerebro_unico_v"] = 1
            counters["cerebro_unico"] = counters["atualizacao"]
            counters["aquecer_desde"] = counters["atualizacao"]
            # (A taxa da cabeca unica comeca a 1/5 da da locomocao - o KL do levantar ~1 por passo a 1,5e-5 cai ~25x - e o ajuste
            # pelo KL do levantar segue dali.)
            actor_opt.param_groups[-1]["lr"] = args.taxa_ator / 5.0
            counters["taxa_cabecas"] = [group["lr"] for group in actor_opt.param_groups]
            print(f"cerebro unico: normalizacao da locomocao = mistura ({args.cerebro_unico_mistura} do levantar) em {merged} entradas, "
                  f"primeira camada compensada (diferenca {gap:.2e}); o critico aquece {args.aquecer_critico} rodadas", flush=True)
        set_single_brain(model)
        print(f"cerebro unico: a cabeca da locomocao comanda tudo desde a atualizacao {counters['cerebro_unico']}", flush=True)
    teacher = None
    distill_reserve = []
    if args.cerebro_unico and (args.cerebro_unico_destilar[0] > 0 or args.cerebro_unico_destilar_puro > 0):
        # (13/11 16h) O professor do kickstarting: a rede de duas cabecas inteira, congelada, com as normalizacoes dela - escolhe a
        # cabeca pela ultima entrada, como no treino de antes.
        if not args.cerebro_unico_professor:
            raise SystemExit("--cerebro-unico-destilar precisa de --cerebro-unico-professor")
        teacher = Policy(obs_size, act_size, batch.action_ranges(), split=True).to(device)
        teacher.load_state_dict(torch.load(Path(args.cerebro_unico_professor).expanduser(), map_location=device,
                                           weights_only=False)["model"])
        teacher.eval()
        for parameter in teacher.parameters():
            parameter.requires_grad_(False)
        print(f"cerebro unico: o professor e {args.cerebro_unico_professor} (peso {args.cerebro_unico_destilar[0]} caindo a zero em "
              f"{int(args.cerebro_unico_destilar[1])} rodadas)", flush=True)
    costas_teacher, costas_teacher_weight = None, 0.0
    if args.levantar_imitar_professor:
        if not args.cerebro_unico or args.levantar_imitar == 0:
            raise SystemExit("--levantar-imitar-professor precisa de --cerebro-unico e --levantar-imitar")
        costas_teacher = Policy(obs_size, act_size, batch.action_ranges(), split=True).to(device)
        costas_teacher.load_state_dict(torch.load(Path(args.levantar_imitar_professor[0]).expanduser(), map_location=device,
                                                  weights_only=False)["model"])
        set_single_brain(costas_teacher)
        costas_teacher.eval()
        for parameter in costas_teacher.parameters():
            parameter.requires_grad_(False)
        costas_teacher_weight = float(args.levantar_imitar_professor[1])
        print(f"levantar: o professor do levantar da mascara e {args.levantar_imitar_professor[0]} (peso {costas_teacher_weight})", flush=True)
    costas_anchor, costas_anchor_weight = None, 0.0
    if args.levantar_imitar_professor_ancora:
        costas_anchor = Policy(obs_size, act_size, batch.action_ranges(), split=True).to(device)
        costas_anchor.load_state_dict(torch.load(Path(args.levantar_imitar_professor_ancora[0]).expanduser(), map_location=device,
                                                 weights_only=False)["model"])
        set_single_brain(costas_anchor)
        costas_anchor.eval()
        for parameter in costas_anchor.parameters():
            parameter.requires_grad_(False)
        costas_anchor_weight = float(args.levantar_imitar_professor_ancora[1])
        print(f"levantar: a ancora do resto e {args.levantar_imitar_professor_ancora[0]} (peso {costas_anchor_weight})", flush=True)
    # (14h15, --levantar-costas-marca) A coluna da marca: o professor ve 0 nela e a ancora prende as linhas sem ela.
    costas_mark_at = list(batch.observation_names()).index("levantar.variante") if args.levantar_costas_marca > 0 else None
    # (14h33) As entradas do fogo: o professor so onde elas estao zeradas. (17h00: so as da FORCA - as fogo.acel.* sao a aceleracao do
    # centro de massa, != 0 em 97% dos passos tambem no limpo: com elas o professor so ensinava o primeiro passo deitado, nunca o ajoelhar.)
    costas_push_cols = [k for k, name in enumerate(batch.observation_names()) if name.startswith("fogo.forca.")] if costas_mark_at is not None else []
    fire_force_cols = [k for k, name in enumerate(batch.observation_names()) if name.startswith("fogo.forca.")]
    # (15h40, --levantar-imitar-professor-pernas) O peso de cada canal na puxada do professor (as pernas mais pesadas).
    costas_leg_w = None
    if costas_teacher is not None and args.levantar_imitar_professor_pernas != 1.0:
        legs = ("LeftThigh", "LeftShin", "LeftFoot", "RightThigh", "RightShin", "RightFoot")
        costas_leg_w = torch.tensor([args.levantar_imitar_professor_pernas if n.split(".")[0] in legs else 1.0 for n in batch.action_names()],
                                    device=device)
        print(f"levantar: as pernas pesam {args.levantar_imitar_professor_pernas} na puxada do professor "
              f"({int((costas_leg_w != 1.0).sum())} de {len(costas_leg_w)} canais)", flush=True)
    if costas_mark_at is not None:
        print(f"levantar: a marca do de costas e {args.levantar_costas_marca} na entrada {costas_mark_at} (levantar.variante)", flush=True)
    if args.loco_saidas_vivas:
        dead_names = [n in LOCO_DEAD_OUTPUTS for n in batch.action_names()]
        if sum(dead_names) != len(LOCO_DEAD_OUTPUTS):
            raise SystemExit(f"--loco-saidas-vivas: {sum(dead_names)} das {len(LOCO_DEAD_OUTPUTS)} saidas mortas nas acoes do lote")
        model.gait_dead = torch.tensor(dead_names, dtype=torch.bool, device=device)
        print(f"saidas vivas: {len(LOCO_DEAD_OUTPUTS)} saidas mortas fora do log da probabilidade nas linhas da marcha", flush=True)
    # (17/11 09h, --ancora-parado) A entrada da velocidade pedida (zero: de pe, sem pedido).
    cmd_speed_at = list(batch.observation_names()).index("comando.velocidade") if "comando.velocidade" in list(batch.observation_names()) else None
    collector = copy.deepcopy(model)
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    update_generator = torch.Generator(device=device)
    update_generator.manual_seed(args.semente * 7919 + counters["atualizacao"])
    pending = None
    last: dict = {}

    redo_window: dict = {}
    redo_count = [0]

    def update(roll: dict) -> dict:
        """A atualizacao de uma rodada (na thread da atualizacao, com a coleta seguinte rodando)."""
        nonlocal replay, replay_loco
        start = time.time()
        with torch.no_grad():
            pairs = roll["pairs"].reshape(T * n, 2 * amp)
            # (No rastreio a locomocao nao tem estilo: o discriminador so com o levantar, como antes.)
            # (07/11: com o juiz proprio da locomocao, as linhas dos exercicios dela entram no treino dele - antes, com o
            # rastreio, a nota de estilo da locomocao (peso 0,5) vinha de um juiz que so via como "falso" o parado e o
            # pos-levantar, nunca a rede correndo.)
            style_rows = ~(roll["loco"].reshape(-1) > .5) if disc_loco is None \
                else torch.ones(T * n, dtype=torch.bool, device=device)
            head_loco = roll["obs"][..., -1].reshape(-1) > .5
            walking = roll["feedback"][..., TRACKING_TERMS.index("portao_velocidade")].reshape(-1) > 0
            loco_x = loco_real = None
            mm_stats = {}
            if disc_loco is not None:
                # (A locomocao com o juiz dela - so os clipes de locomocao; a recuperacao com o de sempre.)
                loco_style_rows = style_rows & head_loco
                # (12/11 16h30, o fogo da locomocao) As linhas sob perturbacao (a liberdade >= 0,3) fora do treino do juiz: ele nao
                # aprende que "empurrado" e falso - os movimentos proprios do equilibrio nao viram estilo ruim.
                if roll["feedback"].shape[-1] > TRACKING_TERMS.index("fogo_liberdade"):
                    free = roll["feedback"][..., TRACKING_TERMS.index("fogo_liberdade")].reshape(-1)
                    loco_style_rows = loco_style_rows & (free < 0.3)
                style_rows = style_rows & ~head_loco
                if mm is not None:
                    # (07/11, o juiz com o Motion Matching: o contexto de cada linha - a consulta do grupo principal - entra no
                    # juiz; o lado real e o par do quadro casado nos bancos que a escolha permite. As linhas sem passado
                    # gravado ainda, ou sem quadro, ficam fora do treino do juiz.)
                    queries = roll["mm"].reshape(T * n, -1)
                    context = queries[:, mm.main]
                    loco_x = torch.cat([pairs, context], 1)
                    loco_real = loco_x.clone()
                    rows = torch.nonzero(loco_style_rows).squeeze(1)
                    if len(rows):
                        matched, cost, group, valid = mm.match(queries[rows])
                        loco_real[rows, :2 * amp] = matched
                        ok = torch.zeros_like(loco_style_rows)
                        ok[rows] = valid
                        loco_style_rows = ok
                        if valid.any():
                            mm_stats["mm_custo"] = float(cost[valid].mean())
                            mm_stats["mm_validos"] = float(valid.float().mean())
                            for g, name in enumerate(mm.names or []):
                                mm_stats["mm_" + name] = round(float((group[valid] == g).float().mean()), 4)
                    if loco_style_rows.any():
                        disc_loco.norm.update(torch.cat([pairs[loco_style_rows, :amp], loco_real[loco_style_rows, :amp]]))
                        disc_loco.context_norm.update(context[loco_style_rows])
                else:
                    if disc_loco_idle is not None:
                        disc_loco_idle.norm.update(torch.cat([pairs[loco_style_rows, :amp], reference_loco[:, :amp]]))
                        loco_style_rows = loco_style_rows & walking
                    disc_loco.norm.update(torch.cat([pairs[loco_style_rows, :amp], reference_loco[:, :amp]]))
            disc.norm.update(torch.cat([pairs[style_rows, :amp], reference[:, :amp]]))
            d = disc(pairs)
            if disc_loco is not None:
                d = torch.where(head_loco, disc_loco(loco_x if loco_x is not None else pairs), d)
                if disc_loco_idle is not None:
                    d = torch.where(head_loco & ~walking, disc_loco_idle(pairs), d)
            d = d.reshape(T, n)
            style = torch.clamp(1.0 - 0.25 * (d - 1.0).pow(2), min=0.0)
            # (A nota por passo vezes 1 - gamma: o retorno fica entre 0 e 1 e o critico nao precisa correr atras da
            # escala de ~100 de um gamma 0,99.)
            # (A rl_final: na locomocao, os pesos dela - a tarefa pesando mais e o tranco menos, o passo e rapido.)
            loco = roll["loco"]
            if args.cerebro_unico and args.cerebro_unico_queda_aprende:
                # (13/11 17h, a etapa das quedas: com um cerebro so, o levantar no meio da locomocao - caiu no comando, levanta e segue - e o
                # mesmo cerebro, e aprende: as linhas da fase do levantar contam como levantar (os pesos, os custos e o aprender), nao como
                # locomocao. Com duas cabecas a recuperacao so "salvava" o episodio, sem aprender, porque o objetivo ali era outro.)
                loco = loco * (roll["obs"][..., -1] > .5).float()
            task_w = args.peso_tarefa + (args.peso_tarefa_loco - args.peso_tarefa) * loco
            style_w = args.peso_estilo + (args.peso_estilo_loco - args.peso_estilo) * loco
            jerk_w = args.suave + (args.suave_loco - args.suave) * loco
            if args.peso_tarefa_parado is not None and args.peso_estilo_parado is not None:
                # (10/11 01h40: o parado com a cabeca da locomocao - fora dos exercicios de locomocao - tinha os pesos da
                # recuperacao (tarefa 0,3, estilo 0,7), e o juiz da locomocao dava 0,70 ao corpo dobrado contra 0,48 de pe: a
                # rede se dobrava. Ali os pesos do parado.)
                idle_rows = ((roll["obs"][..., -1] > .5) & (loco < .5)).float()
                task_w = task_w + (args.peso_tarefa_parado - task_w) * idle_rows
                style_w = style_w + (args.peso_estilo_parado - style_w) * idle_rows
            if args.estilo_limpo is not None and args.levantar_fogo_entrada:
                # (12/11 15h20) O estilo no levantar limpo: a cabeca da recuperacao com a entrada da forca externa do fogo (as 3 colunas
                # antes da aceleracao, logo antes da habilidade) zerada - o limpo inteiro e a firmeza depois da forca soltar.
                force = roll["obs"][..., -1 - FIRE_OBSERVATION_SIZE: -1 - FIRE_OBSERVATION_SIZE + 3].abs().sum(-1)
                clean_rows = ((roll["obs"][..., -1] < .5) & (force < 1.0e-4)).float() * (1.0 - loco)
                style_w = style_w + (args.estilo_limpo - style_w) * clean_rows
            if args.estilo_chao is not None:
                # (02/11 00h30, ele: "a unica coisa que precisa melhorar nela e o movimento de virar-se ser mais natural": no
                # chao - a pelve abaixo de --chao-altura, onde ela gira de costas meio no ar com o braco jogado para cima -
                # o estilo, o juiz dos clipes, pesa mais; de pe e na rapidez, como estava.)
                low = (roll["obs"][..., 0] < args.chao_altura).float() * (1.0 - loco)
                style_w = style_w + (args.estilo_chao - style_w) * low
                task_w = task_w + ((1.0 - args.estilo_chao) - task_w) * low
            if args.estilo_manobras_zero:
                # (06/11 22h: o juiz so conhece os clipes em linha reta - nas curvas e trocas a rede tem de inclinar e girar;
                # a curva fechada de trote saia do percurso em 80% (desvio 95 graus) depois do PPO. Nas manobras, so a tarefa.)
                maneuver = (roll["espelho"][..., 2] > 0.5) & (roll["loco"] > .5)
                task_w = torch.where(maneuver, torch.ones_like(task_w), task_w)
                style_w = torch.where(maneuver, torch.zeros_like(style_w), style_w)
            if args.estilo_velocidade > 0:
                # (08/11 23h55: o estilo pagava ~70% do premio da locomocao e quase nao dependia da velocidade - devagar e de pe
                # rendia quase o mesmo. Na locomocao, vezes o portao da velocidade, como a nota.)
                # (A coluna do portao vale 0 onde a nota andando nao foi calculada - o idle, o parado: ali sem portao.)
                moving = roll["feedback"][..., TRACKING_TERMS.index("portao_velocidade")] > 0
                gate = args.estilo_velocidade + (1.0 - args.estilo_velocidade) * roll["feedback"][..., 8].clamp(0.0, 1.0)
                style = torch.where((loco > .5) & moving, style * gate, style)
            if args.locomocao_fogo_estilo_livre and roll["feedback"].shape[-1] > TRACKING_TERMS.index("fogo_liberdade"):
                # (12/11 20h05, a revisao do fogo com o Codex: o juiz ja nao treina nas linhas empurradas, mas a nota dele seguia com o
                # peso inteiro nelas - o passo proprio do equilibrio perdia premio de estilo, a "amarra" que o rework queria tirar.
                # Enquanto a forca age (a liberdade, que some 1,5 s depois), o peso do estilo vai para a tarefa; no limpo, nada muda.)
                free = roll["feedback"][..., TRACKING_TERMS.index("fogo_liberdade")].clamp(0.0, 1.0) * (roll["obs"][..., -1] > .5).float()
                task_w = task_w + style_w * free
                style_w = style_w * (1.0 - free)
            task_style = task_w * roll["task"] + style_w * style
            costs = (1.0 - args.protecao_alivio * roll["prot"]) * (jerk_w * roll["jerk"] + args.limite * roll["limit"])
            # (03/11: no rastreio os custos so comem a nota do passo, nunca a deixam negativa - com a nota baixa do
            # comeco, o limite x2 deixava cada passo negativo e cair voltava a compensar, o erro da locomocao livre.)
            # (17/11, --teto-custo-tarefa - a auditoria: com o teto na nota inteira e o peso 0,5 da tarefa, acima do teto o premio era
            # 0,5 nota + estilo - nota: melhorar a nota TIRAVA premio. O teto na parte da tarefa: acima dele o premio nao cai com a nota.)
            cap = task_w * roll["feedback"][..., 0] if args.teto_custo_tarefa else roll["feedback"][..., 0]
            costs = torch.where(loco > .5, torch.minimum(costs, cap), costs)
            if args.levantar_unreal > 0:
                # (11/11 14h, a regra dele - "a recompensa por ficar na pose nunca pode ser maior que o objetivo de nao cair": com a
                # recuperacao no comando os custos nunca passam da nota, que e >= 0; acabar cedo nunca compensa.)
                recovery_rows = roll["obs"][..., -1] < .5
                costs = torch.where(recovery_rows, torch.minimum(costs, roll["task"].clamp(min=0.0)), costs)
            if args.parado_unico:
                # (11/11 09h45, ele: "a recompensa por ficar na pose nunca pode ser maior que o objetivo de nao cair": no parado da
                # cabeca da locomocao os custos tambem nunca passam da nota - de pe a nota nunca fica negativa e cair nunca compensa.)
                idle_head = (roll["obs"][..., -1] > .5) & (loco < .5)
                costs = torch.where(idle_head, torch.minimum(costs, roll["feedback"][..., 0].clamp(min=0.0)), costs)
            if args.cerebro_unico and args.cerebro_unico_queda_aprende:
                # (13/11 17h05, a regra dele - cair nunca pode compensar) Caido no meio de um episodio de locomocao (o levantar do comando),
                # o tempo no chao e so custo: sem a nota do levantar, sem o premio do tempo na entrega (ate 40 x 0,01 = 0,4 de retorno,
                # mais que 1,5 s de locomocao, ~0,11-0,19) e sem estilo; -custo por passo. Levantar rapido paga por devolver a tarefa.
                # O levantar com a nota inteira fica nos exercicios dele.
                rescue = (roll["loco"] > .5) & (roll["obs"][..., -1] < .5)
                task_style = torch.where(rescue, torch.full_like(task_style, -args.cerebro_unico_queda_custo), task_style)
                costs = torch.where(rescue, torch.zeros_like(costs), costs)
            rew = (1.0 - gamma) * (task_style - costs) + gamma * roll["boot"]
            adv = torch.zeros(T, n, device=device)
            gae = torch.zeros(n, device=device)
            for t in reversed(range(T)):
                next_value = roll["last_value"] if t == T - 1 else roll["val"][t + 1]
                nonterminal = 1.0 - roll["done"][t]
                delta = rew[t] + gamma * next_value * nonterminal - roll["val"][t]
                gae = delta + gamma * lam * nonterminal * gae
                adv[t] = gae
            ret = adv + roll["val"]
            flat = lambda x: x.reshape(T * n, *x.shape[2:])  # noqa: E731
            f_obs, f_act, f_adv, f_ret = flat(roll["obs"]), flat(roll["act"]), flat(adv), flat(ret)
            f_mirror = flat(roll["espelho"])
            # (17/11 11h, --ancora-fogo) As linhas com forca de fogo agora ou nos ultimos S segundos (dentro da coleta).
            # shortcut: um empurrao antes do comeco da coleta nao marca as linhas seguintes; ~1 s em ~2 s de coleta - aceitavel.
            f_fire_recent = None
            if args.ancora_fogo > 0 and fire_force_cols:
                hit = (roll["obs"][..., fire_force_cols].abs().sum(-1) > 1.0e-6).float().cumsum(0)
                span = max(1, int(round(args.ancora_fogo / batch.policy_dt)))
                lag = torch.cat([torch.zeros_like(hit[:span]), hit[:-span]], 0) if span < T else torch.zeros_like(hit)
                f_fire_recent = flat((hit - lag) > 0)
            # (03/11 18h30, ele: "como tu espera que ele consiga se mover para os lados e costas?" - a rede de levantar
            # nunca deu um passo de lado ou de costas e o RL sozinho so achava a frente. No comeco, a cabeca da locomocao
            # aprende supervisionada os alvos de motor do clipe do rumo pedido (na acao normalizada), com o peso caindo a
            # zero em --imitar-atualizacoes; o clipe nunca vai aos motores.)
            imitate = args.imitar * max(0.0, 1.0 - counters["atualizacao"] / max(1, args.imitar_atualizacoes))
            # (13/11 15h45, --cerebro-unico-imitar) O comeco do levantar da cabeca unica: a media dela nas linhas do levantar puxada
            # para os alvos de motor do clipe uma decisao a frente (os mesmos professores da Unreal), o peso caindo a zero. Medido: a
            # cabeca da locomocao deitada manda 20% dos eixos para fora da faixa (a do levantar, 3%) e em 450 rodadas de RL a
            # semelhanca nao saiu de 0,16 - a cabeca zerada de 11/11 comecava no meio do movimento.
            rise_imitate = 0.0
            if args.cerebro_unico and args.cerebro_unico_imitar[0] > 0:
                since = counters.setdefault("cerebro_unico_imitar_desde", counters["atualizacao"])
                rise_imitate = args.cerebro_unico_imitar[0] * max(0.0, 1.0 - (counters["atualizacao"] - since)
                                                                   / max(1.0, args.cerebro_unico_imitar[1]))
            if rise_imitate > 0 and args.cerebro_unico_imitar_taxa > 0:
                # (13/11 15h42: a 2,2e-6 a MSE ate o clipe caia 0,3% por rodada - 0,99 -> 0,93 em 20 -, devagar para a janela de 300.
                # Enquanto imita, esta taxa; o freio da fase de pe (0,4) e a volta (2,0) seguem guardando a locomocao.)
                actor_opt.param_groups[-1]["lr"] = args.cerebro_unico_imitar_taxa
            f_clip = (flat(roll["clip"]) - model.action_mid) / model.action_half if imitate > 0 or rise_imitate > 0 or costas_teacher is not None else None
            # (01/11 21h50: normalizar tudo junto afogava o levantar - a imitacao x4 da locomocao varia muito mais e a
            # vantagem do levantar ia a ~0; ele: "a calibragem de recompensa dele esta bem defeituosa". Cada grupo a parte.)
            f_loco = f_obs[:, -1] > .5 if model.split else flat(loco) > .5
            # Recovery can rescue a locomotion episode without learning its different objective.
            learn = (~f_loco if args.congelar_locomocao else f_loco if args.congelar_recuperacao else ~(flat(loco) > .5) | f_loco) \
                if model.split else torch.ones_like(f_loco)
            # (13/11 15h35, o cerebro unico parado em 0,16: o critico de cada fase - a variancia explicada do retorno e o desvio da
            # vantagem antes de normalizar, nas linhas que aprendem.)
            phase_stats = {}
            if args.cerebro_unico:
                f_val = flat(roll["val"])
                for tag, mask in (("lev", learn & ~f_loco), ("pe", learn & f_loco)):
                    if int(mask.sum()) > 10:
                        r_, v_ = f_ret[mask], f_val[mask]
                        phase_stats[f"ev_{tag}"] = float(1.0 - (r_ - v_).var() / (r_.var() + 1e-12))
                        phase_stats[f"vantagem_{tag}"] = float(f_adv[mask].std())
                        phase_stats[f"retorno_{tag}"] = float(r_.mean())
            for group in (f_loco, ~f_loco):
                if int(group.sum()) > 1:
                    f_adv[group] = (f_adv[group] - f_adv[group].mean()) / (f_adv[group].std() + 1e-8)
            phase_weight = None
            if args.cerebro_unico and args.cerebro_unico_peso_levantar > 0:
                # (13/11 15h20, --cerebro-unico-peso-levantar) A parte do levantar no gradiente da cabeca unica: com 17% das linhas ele
                # tinha 17% da perda (a cabeca do levantar de 11/11 tinha 100%) e, em 300 rodadas com o KL dele no alvo, a semelhanca
                # nao saiu de 0,16. Cada fase pesa a fracao pedida da perda do PPO, nas linhas que aprendem.
                rising = learn & ~f_loco
                share = float(rising.float().sum()) / max(1.0, float(learn.float().sum()))
                if 0.0 < share < 1.0:
                    w_rise = args.cerebro_unico_peso_levantar
                    phase_weight = torch.where(~f_loco, torch.full_like(f_adv, w_rise / share),
                                               torch.full_like(f_adv, (1.0 - w_rise) / (1.0 - share)))
            distill, distill_weight = 0.0, None
            if teacher is not None:
                since = counters.setdefault("cerebro_unico_destilar_desde", counters["atualizacao"])
                distill = args.cerebro_unico_destilar[0] * max(0.0, 1.0 - (counters["atualizacao"] - since)
                                                               / max(1.0, args.cerebro_unico_destilar[1]))
                if distill > 0:
                    rising_rows = learn & ~f_loco
                    share_rise = float(rising_rows.float().sum()) / max(1.0, float(learn.float().sum()))
                    if 0.0 < share_rise < 1.0:
                        distill_weight = torch.where(~f_loco, torch.full_like(f_adv, 0.5 / share_rise),
                                                     torch.full_like(f_adv, 0.5 / (1.0 - share_rise)))
                    else:
                        distill_weight = torch.ones_like(f_adv)
                    if args.cerebro_unico_destilar_taxa > 0:
                        actor_opt.param_groups[-1]["lr"] = args.cerebro_unico_destilar_taxa
            pure = False
            if teacher is not None and args.cerebro_unico_destilar_puro > 0:
                since_pure = counters.setdefault("cerebro_unico_puro_desde", counters["atualizacao"])
                pure = counters["atualizacao"] - since_pure < args.cerebro_unico_destilar_puro
                if not pure and not counters.get("cerebro_unico_puro_fim"):
                    # (O fim da copia pura: o PPO comeca na taxa da locomocao/5 e o ajuste pelo KL do levantar segue dali.)
                    counters["cerebro_unico_puro_fim"] = counters["atualizacao"]
                    actor_opt.param_groups[-1]["lr"] = args.taxa_ator / 5.0
            # (14/11 13h35, o DAgger do de costas: o professor tambem ensina - sem isto o freio de 0,12 na fase do levantar, que o puxao
            # do professor sempre passa, parava a rede inteira em 7-11 dos 16 passos e a perda dele ficava em ~0,09; o freio e a volta
            # da fase de pe seguem valendo.)
            teaching = rise_imitate > 0 or distill > 0 or pure or costas_teacher is not None or costas_anchor is not None
            anchor = skill_logp(model, model.distribution(model.norm(f_obs)), f_act, f_obs, gait=f_mirror[:, 0] > .5)
            logp_coleta = flat(roll["logp"])
            weight = torch.exp((anchor - logp_coleta).clamp(max=math.log(2.0)))
        replay = torch.cat([replay, pairs[style_rows]])[-100_000:]
        if disc_loco is not None:
            replay_loco = torch.cat([replay_loco, torch.cat([loco_x[loco_style_rows], loco_real[loco_style_rows]], 1)
                                     if loco_x is not None else pairs[loco_style_rows]])[-100_000:]
        stats = {"pol": 0.0, "vf": 0.0, "clip": 0.0, "disc": 0.0, "d_clipe": 0.0, "d_corpo": 0.0, "gp": 0.0, "kl": 0.0}
        steps = 0
        actor_frozen = False
        skill_frozen = [False, False]
        # (04/11 22h20, a investigacao da estagnacao: quantos passos do otimizador cada cabeca deu nesta atualizacao.)
        skill_steps = [0, 0]
        skill_kl = [0.0, 0.0]
        minibatches = 0
        loco_before = loco_restore = None
        if pure:
            # (13/11 16h20, a copia pura - DAgger: os estados que o aluno visitou nas ultimas rodadas, a acao media e o valor do professor.
            # A copia junto do PPO (kickstarting, 16h00) parou em MSE 0,07 no chao sem levantar e mexeu na locomocao - trote 0,931 -> 0,872:
            # 16 passos por rodada era pouco para copiar e muito para a rede de pe.)
            skill_frozen = [True, True]
            distill_reserve.append(f_obs.detach())
            del distill_reserve[:-max(1, args.cerebro_unico_destilar_reserva)]
            data = torch.cat(distill_reserve)
            with torch.no_grad():
                t_act = torch.cat([teacher.actor(teacher.norm(c)) for c in data.split(16384)])
                t_val = torch.cat([teacher.value(teacher.norm(c)) for c in data.split(16384)])
                rise = data[:, -1] <= .5
                share = float(rise.float().mean())
                w_row = (torch.where(rise, torch.full_like(t_val, 0.5 / share), torch.full_like(t_val, 0.5 / (1.0 - share)))
                         if 0.0 < share < 1.0 else torch.ones_like(t_val))
                keep = torch.ones_like(t_act)
                if model.step_count and model.joint_names:
                    chan = torch.arange(t_act.shape[-1], device=t_act.device)
                    steps_ch = (chan >= len(model.joint_names)) & (chan < len(model.joint_names) + model.step_count)
                    keep = torch.where(rise[:, None] & steps_ch, torch.zeros_like(keep), keep)
            for group in actor_opt.param_groups:
                group["lr"] = args.cerebro_unico_destilar_taxa if args.cerebro_unico_destilar_taxa > 0 else 1.0e-4
            acc = {"destilar_lev": [0.0, 0], "destilar_pe": [0.0, 0], "destilar_valor": [0.0, 0]}
            for _ in range(args.cerebro_unico_destilar_passos):
                pick = torch.randint(len(data), (min(4096, len(data)),), device=data.device, generator=update_generator)
                x = model.norm(data[pick])
                gap = ((model.actor(x) - t_act[pick]).pow(2) * keep[pick]).mean(-1)
                loss = (gap * w_row[pick]).mean()
                actor_opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(list(model.actor.locomotion.parameters()), 1.0)
                actor_opt.step()
                value_loss = ((model.value(x) - t_val[pick]).pow(2) * w_row[pick]).mean()
                critic_opt.zero_grad()
                value_loss.backward()
                nn.utils.clip_grad_norm_(list(model.critic.locomotion.parameters()), 1.0)
                critic_opt.step()
                with torch.no_grad():
                    r = rise[pick]
                    if r.any():
                        acc["destilar_lev"][0] += float(gap[r].mean()); acc["destilar_lev"][1] += 1
                    if (~r).any():
                        acc["destilar_pe"][0] += float(gap[~r].mean()); acc["destilar_pe"][1] += 1
                    acc["destilar_valor"][0] += float(value_loss); acc["destilar_valor"][1] += 1
            phase_stats.update({k: v[0] / max(1, v[1]) for k, v in acc.items()})
            phase_stats["destilar_reserva"] = int(len(data))
        for _ in range(args.epocas):
            for idx in torch.randperm(T * n, device=device, generator=update_generator).split(minibatch):
                idx = idx[learn[idx]]
                if not len(idx): continue
                obs_n = model.norm(f_obs[idx])
                if pure:
                    vf_loss = torch.zeros((), device=obs_n.device)
                else:
                    vf_loss = (model.value(obs_n) - f_ret[idx]).pow(2).mean()
                    critic_opt.zero_grad()
                    vf_loss.backward()
                    if model.split:
                        nn.utils.clip_grad_norm_(model.critic.recovery.parameters(), 1.0)
                        nn.utils.clip_grad_norm_(model.critic.locomotion.parameters(), 1.0)
                    else:
                        nn.utils.clip_grad_norm_(model.critic.parameters(), 1.0)
                    critic_opt.step()
                dist_mb = model.distribution(obs_n)
                logp = skill_logp(model, dist_mb, f_act[idx], f_obs[idx], gait=f_mirror[idx][:, 0] > .5)
                # (03/11 21h06: com o desvio 0,05 da cabeca da locomocao a razao das linhas que a imitacao moveu estourava -
                # exp de centenas -, e o NaN do gradiente dessas linhas mascaradas envenenava a rede: o log da razao preso.)
                ratio = torch.exp((logp - anchor[idx]).clamp(-20.0, 20.0))
                w = weight[idx]
                a_mb = f_adv[idx]
                surrogate = torch.min(ratio * w * a_mb, ratio.clamp(1 - clip, 1 + clip) * w * a_mb)
                if args.ppo_clipe_duplo > 0:
                    # (15/11 15h, a revisao: a razao vai a e^20 e, com a vantagem negativa, o min deixava a perda sem limite - os picos de KL
                    # de 1e5-1e7 nas linhas do levantar dominavam o passo. O clipe duplo (Ye et al. 2020): nao passa de C x A.)
                    surrogate = torch.where(a_mb < 0, torch.maximum(surrogate, args.ppo_clipe_duplo * w * a_mb), surrogate)
                pol_loss = -(surrogate if phase_weight is None else surrogate * phase_weight[idx]).mean()
                if distill > 0:
                    with torch.no_grad():
                        target = teacher.actor(teacher.norm(f_obs[idx]))
                    gap = (dist_mb.mean - target).pow(2)
                    if model.step_count and model.joint_names:
                        # (Os canais do passo nao contam no levantar - a recuperacao nao os usa, como no skill_logp.)
                        chan = torch.arange(gap.shape[-1], device=gap.device)
                        steps_ch = (chan >= len(model.joint_names)) & (chan < len(model.joint_names) + model.step_count)
                        gap = torch.where((f_obs[idx][:, -1:] <= .5) & steps_ch, torch.zeros_like(gap), gap)
                    per_row = gap.mean(-1)
                    pol_loss = pol_loss + distill * (per_row * distill_weight[idx]).mean()
                    with torch.no_grad():
                        rise_mask = ~f_loco[idx]
                        if rise_mask.any():
                            stats["destilar_lev"] = stats.get("destilar_lev", 0.0) + float(per_row[rise_mask].mean())
                        if (~rise_mask).any():
                            stats["destilar_pe"] = stats.get("destilar_pe", 0.0) + float(per_row[~rise_mask].mean())
                if f_clip is not None and rise_imitate > 0:
                    rising = ~f_loco[idx] & (flat(roll["clip"])[idx].abs().sum(-1) > 0)
                    if rising.any():
                        joints = len(model.joint_names)
                        rise_im = (dist_mb.mean[rising][:, :joints] - f_clip[idx][rising][:, :joints]).pow(2).mean()
                        pol_loss = pol_loss + rise_imitate * rise_im
                        stats["imitacao_levantar"] = stats.get("imitacao_levantar", 0.0) + rise_im.item()
                # (14/11 18h50, ele: "o especialista nem deveria existir mais") A ancora tambem sem o professor.
                if costas_teacher is not None or costas_anchor is not None:
                    rising = ~f_loco[idx] & (flat(roll["clip"])[idx].abs().sum(-1) > 0)
                    if costas_mark_at is not None:
                        # (14h25) Com a marca: todas as linhas do de costas marcadas (limpo e sob fogo, ate a troca) - deitado o de fogo e
                        # igual ao limpo, e o PPO adaptativo puxava o rolar rapido na mesma entrada em que o professor puxava o clipe.
                        rising = ~f_loco[idx] & (f_obs[idx][:, costas_mark_at] >= 0.5 * args.levantar_costas_marca)
                        # (14h33) So sem empurrao agindo: nos estados empurrados o professor (treinado no limpo) dava acoes ruins - saltos
                        # de KL desfeitos pela volta e o puxao vazando para o correndo de costas (12,8 -> 15,9 cm). Ali fica o PPO.
                        if costas_push_cols:
                            rising = rising & (f_obs[idx][:, costas_push_cols].abs().sum(-1) < 1.0e-6)
                    if costas_teacher is not None and rising.any():
                        with torch.no_grad():
                            seen = f_obs[idx][rising]
                            if costas_mark_at is not None:
                                seen = seen.clone(); seen[:, costas_mark_at] = 0.0
                            target = costas_teacher.actor(costas_teacher.norm(seen))
                        if model.has_costas_adapter:
                            # (16h25) Com o ramo do de costas, o professor ensina so o ramo: a rede de base entra destacada (sem gradiente
                            # do professor) - nada da puxada vaza para o correndo de costas nem para a locomocao.
                            xr = obs_n[rising]
                            gap = (model.actor(xr).detach() + model.costas_adapter(xr) - target).pow(2)
                        else:
                            gap = (dist_mb.mean[rising] - target).pow(2)
                        if costas_leg_w is not None:
                            gap = gap * costas_leg_w
                        if model.step_count and model.joint_names:
                            # (Os canais do passo nao contam no levantar, como no --cerebro-unico-destilar.)
                            chan = torch.arange(gap.shape[-1], device=gap.device)
                            gap = gap[:, (chan < len(model.joint_names)) | (chan >= len(model.joint_names) + model.step_count)]
                        teach = gap.mean()
                        pol_loss = pol_loss + costas_teacher_weight * teach
                        stats["professor_levantar"] = stats.get("professor_levantar", 0.0) + teach.item()
                    # (14/11 13h50, --levantar-imitar-professor-ancora) As linhas de pe presas ao principal congelado. (14h02: so as de
                    # pe - com todas as outras, o de costas sob fogo, deitado no mesmo estado do limpo, ficava preso ao rolar antigo e
                    # brigava com o professor na mesma entrada; os levantares seguem no PPO, como no DAgger da manha.)
                    # (14h15) Com a marca: todas as linhas sem ela (os outros levantares, o correndo de costas, a locomocao) e as de pe.
                    others = f_loco[idx] | (f_obs[idx][:, costas_mark_at] < 0.5 * args.levantar_costas_marca) if costas_mark_at is not None and not args.ancora_so_locomocao else f_loco[idx]
                    # (17/11, --ancora-sem-locomocao - o plano de desentortar: a ancora segurava a locomocao na rede torta; o modo de ensino fica.)
                    if args.ancora_sem_locomocao:
                        others = others & ~f_loco[idx]
                        # (17/11 09h, --ancora-parado) O parado e a freada (de pe, sem pedido) tambem na ancora.
                        if args.ancora_parado and cmd_speed_at is not None:
                            others = others | (f_loco[idx] & (f_obs[idx][:, cmd_speed_at].abs() <= 1.0e-6))
                        if f_fire_recent is not None:
                            fired = f_loco[idx] & f_fire_recent[idx]
                            others = others | fired
                            stats["ancora_fogo_linhas"] = stats.get("ancora_fogo_linhas", 0.0) + fired.float().mean().item()
                    if costas_anchor is not None and others.any():
                        with torch.no_grad():
                            held_in = f_obs[idx][others]
                            if args.levantar_brucos_marca < 0.0:
                                held_in = held_in.clone()
                                held_in[:, costas_mark_at] = held_in[:, costas_mark_at].clamp(min=0.0)
                            held = costas_anchor.actor(costas_anchor.norm(held_in))
                        per_row = (dist_mb.mean[others] - held).pow(2).mean(-1)
                        # (14h47) Duas partes de peso igual - as de pe e os outros levantares: na media unica, as do levantar (o correndo
                        # de costas) eram poucas entre as da locomocao e quase nao pesavam (correndo de costas 11,8 -> 13,3 cm, 1,15 -> 1,35 s).
                        rise_rows = ~f_loco[idx][others]
                        hold = per_row[~rise_rows].mean() if (~rise_rows).any() else per_row.new_zeros(())
                        if rise_rows.any():
                            hold_rise = per_row[rise_rows].mean()
                            hold = hold + hold_rise
                            stats["ancora_levantares"] = stats.get("ancora_levantares", 0.0) + hold_rise.item()
                        pol_loss = pol_loss + costas_anchor_weight * hold
                        stats["ancora_resto"] = stats.get("ancora_resto", 0.0) + hold.item()
                if f_clip is not None and imitate > 0:
                    shown = f_loco[idx] & (flat(roll["clip"])[idx].abs().sum(-1) > 0)
                    if shown.any():
                        if args.imitar_velocidade and model.step_count and model.joint_names:
                            # (17/11, --imitar-velocidade) As posicoes e as velocidades do clipe; os canais do passo fora.
                            chan = torch.arange(dist_mb.mean.shape[-1], device=dist_mb.mean.device)
                            cols = (chan < len(model.joint_names)) | (chan >= len(model.joint_names) + model.step_count)
                            imitation = (dist_mb.mean[shown][:, cols] - f_clip[idx][shown][:, cols]).pow(2).mean()
                        else:
                            imitation = (dist_mb.mean[shown] - f_clip[idx][shown]).pow(2).mean()
                        pol_loss = pol_loss + imitate * imitation
                        stats["imitacao"] = stats.get("imitacao", 0.0) + imitation.item()
                if mirror is not None:
                    # (05/11, o vies para a direita: a rede corrigia o passo esquerdo e o direito de jeitos diferentes -
                    # ~1,0 na acao normalizada - e dobrava o tronco de lado de forma constante. Nos rumos retos com o
                    # clipe periodico, a acao no estado espelhado tem de ser o espelho da acao.)
                    # (05/11: a terceira coluna, a manobra - nas trocas so com --simetria-manobras.)
                    # (05/11 20h: --simetria-curvas so nas curvas - as manobras 5 a 7 -, nao nas trocas, onde a simetria
                    # dobrou as quedas; ele: "esse problema se propaga em todos os movimentos de corrida... isso inclui as curvas".)
                    man = f_mirror[idx][:, 2]
                    rows = f_loco[idx] & (f_mirror[idx][:, 0] > .5) & ((man == 0) | args.simetria_manobras
                                                                        | ((man >= 5) & args.simetria_curvas)
                                                                        | ((man == 12) & args.simetria_comando))
                    if rows.any():
                        with torch.set_grad_enabled(args.simetria > 0):
                            seen = f_obs[idx][rows]
                            twin = mirror.acao(model.actor(model.norm(mirror.obs(seen, f_mirror[idx][rows][:, 1]))))
                            gap = dist_mb.mean[rows] - twin
                            symmetry = gap.pow(2).mean()
                        if args.simetria > 0:
                            pol_loss = pol_loss + args.simetria * symmetry
                        stats["simetria"] = stats.get("simetria", 0.0) + symmetry.item()
                        stats["assimetria"] = stats.get("assimetria", 0.0) + gap.detach().abs().mean().item()
                        stats["linhas_espelho"] = stats.get("linhas_espelho", 0.0) + float(rows.float().mean())
                if retained is not None:
                    old = ~f_loco[idx] & (roll["feedback"][..., 6].reshape(-1)[idx] < .5)
                    if old.any():
                        with torch.no_grad():
                            target = retained.actor(retained.norm(f_obs[idx[old]]))
                        # (17h55: com a velocidade no levantar, a ancora so nas posicoes e nos canais - a velocidade livre.)
                        keep = len(model.joint_names) + model.step_count if model.recovery_velocity else target.shape[-1]
                        out_old = model.actor(obs_n[old])
                        pol_loss = pol_loss + args.preservar_peso * (out_old[:, :keep] - target[:, :keep]).square().mean()
                        if args.preservar_velocidade > 0 and keep < target.shape[-1]:
                            # (19h25: a pose de pe do levantar caia 0,65 -> 0,57 com as velocidades soltas - elas tambem puxadas,
                            # mais fraco, para as da rede transplantada: zero onde nao ajudam.)
                            pol_loss = pol_loss + args.preservar_velocidade * (out_old[:, keep:] - target[:, keep:]).square().mean()
                if anchor_obs is not None:
                    # (E5: um minilote do banco por passo, do tamanho do do PPO; a rede atual com a normalizacao dela.)
                    anchor_rows = torch.randint(len(anchor_obs), (min(minibatch, len(anchor_obs)),), device=device,
                                              generator=anchor_generator)
                    anchor_dist = anchor_distance(model, anchor_obs[anchor_rows], anchor_target[anchor_rows])
                    pol_loss = pol_loss + args.ancora * anchor_dist
                    stats["ancora_dist"] = stats.get("ancora_dist", 0.0) + anchor_dist.item()
                # (O KL contra a rede que coletou - somado nos eixos, o estimador k3 -, no consenso com o Codex de 03/11:
                # a ancora do recorte e intermediaria na coleta sobreposta. So o ator para; o critico e o juiz seguem.)
                with torch.no_grad():
                    # (03/11: contra a rede do comeco desta atualizacao - a ancora do recorte -, nao a da coleta: com a
                    # coleta sobreposta ela esta uma atualizacao atras, o KL ja nascia acima do teto e o freio travava a
                    # cabeca da locomocao em 98% das atualizacoes, antes do primeiro passo.)
                    log_c = (logp - anchor[idx]).clamp(-20.0, 20.0)
                    kl_rows = torch.exp(log_c) - 1 - log_c
                    kl = kl_rows.mean().item()
                    if model.split:
                        for skill, mask in enumerate((~f_loco[idx], f_loco[idx])):
                            if mask.any() and not skill_frozen[skill]:
                                skill_kl[skill] = kl_rows[mask].mean().item()
                        if loco_before is not None and f_loco[idx].any() and kl_rows[f_loco[idx]].mean() > args.kl_reverter_loco:
                            # (A volta depois do passo deste minilote - a perda ja usa os pesos de agora; a cabeca travada nao muda nele.)
                            loco_restore = loco_before
                            skill_frozen[1] = True
                            stats["revertidos_loco"] = stats.get("revertidos_loco", 0.0) + 1.0
                            skill_kl[1] = -kl_rows[f_loco[idx]].mean().item()  # (negativo: o salto que foi desfeito)
                        loco_before = None
                    if model.split and args.kl_max > 0:
                        for skill, mask in enumerate((~f_loco[idx], f_loco[idx])):
                            # (Na imitacao do comeco a cabeca da locomocao nao freia pelo KL.)
                            if skill == 1 and imitate > 0: continue
                            # (13/11, --cerebro-unico-imitar: enquanto imita, a fase do levantar nao freia - a de pe segue com o freio.)
                            if skill == 0 and teaching: continue
                            limit = args.kl_max_loco if skill == 1 and args.kl_max_loco > 0 else args.kl_max
                            if mask.any() and kl_rows[mask].mean() > limit:
                                skill_frozen[skill] = True
                if args.cerebro_unico and any(skill_frozen):
                    # (13/11) Uma cabeca so: o freio de qualquer das fases para a rede inteira.
                    skill_frozen[0] = skill_frozen[1] = True
                if model.split:
                    actor_frozen = all(skill_frozen)
                elif args.kl_max > 0 and kl > args.kl_max:
                    actor_frozen = True
                minibatches += 1
                # (11/11 14h: contado desde o zeramento da cabeca do levantar - "aquecer_desde", posto por zerar_recuperacao.py.)
                if counters["atualizacao"] - counters.get("aquecer_desde", 0) >= args.aquecer_critico and not actor_frozen:
                    if model.split:
                        for skill, mask in enumerate((~f_loco[idx], f_loco[idx])):
                            if not skill_frozen[skill] and mask.any(): skill_steps[skill] += 1
                    actor_opt.zero_grad()
                    if adapter_opt is not None:
                        adapter_opt.zero_grad()
                    pol_loss.backward()
                    if adapter_opt is not None:
                        nn.utils.clip_grad_norm_(list(model.costas_adapter.parameters()), 1.0)
                        adapter_opt.step()
                    if model.split:
                        for frozen, head, std in ((skill_frozen[0], model.actor.recovery, model.log_std),
                                                  (skill_frozen[1], model.actor.locomotion, model.loco_log_std)):
                            parameters = list(head.parameters()) + [std]
                            if frozen:
                                for parameter in parameters: parameter.grad = None
                            else:
                                nn.utils.clip_grad_norm_(parameters, 1.0)
                    else:
                        nn.utils.clip_grad_norm_(model.actor_parameters(), 1.0)
                    # (12/11 22h30/22h45, a revisao com o Codex: o freio do KL so via o salto no minilote seguinte e nao o desfazia - picos
                    # de 10-38 na cabeca da locomocao; o desvio dela 0,068 faz 0,05 de media em varias juntas virar KL ~10. Medido no
                    # proprio minilote o salto nao aparecia - ele cai nas linhas que o passo nao viu. Com --kl-reverter-loco a cabeca e
                    # guardada antes de cada passo; no minilote seguinte (e numa amostra no fim), acima do teto, ela volta ao guardado e para.)
                    loco_before = ([q.detach().clone() for q in list(model.actor.locomotion.parameters()) + [model.loco_log_std]]
                                   if model.split and args.kl_reverter_loco > 0 and not skill_frozen[1] else None)
                    actor_opt.step()
                if loco_restore is not None:
                    with torch.no_grad():
                        for q, old in zip(list(model.actor.locomotion.parameters()) + [model.loco_log_std], loco_restore):
                            q.copy_(old)
                    loco_restore = None
                # O discriminador: metade do lote de agora e metade da reserva, contra os clipes (com reposicao). Com o
                # juiz da locomocao, cada um com as linhas e os clipes dele.
                judges = [(disc, disc_opt, style_rows, replay, reference)]
                # (11/11 14h: com a locomocao congelada o juiz dela tambem - a nota dela depois da troca e a mesma o treino todo.)
                if disc_loco is not None and not args.congelar_locomocao:
                    judges.append((disc_loco, disc_loco_opt, loco_style_rows, replay_loco, reference_loco))
                for judge_index, (judge, judge_opt, rows, reserve, refs) in enumerate(judges):
                    conditional = judge_index == 1 and loco_x is not None
                    current = (loco_x if conditional else pairs)[idx[rows[idx]]]
                    if len(current) >= 2 and len(reserve):
                        half = len(current) // 2
                        pick = torch.randint(len(reserve), (half,), device=device, generator=update_generator)
                        if conditional:
                            # (O real de cada linha e o quadro casado para o contexto dela - os dois lados com o mesmo contexto.)
                            width = loco_x.shape[1]
                            body = torch.cat([current[:half], reserve[pick, :width]])
                            clips = torch.cat([loco_real[idx[rows[idx]]][:half], reserve[pick, width:]])
                        else:
                            body = torch.cat([current[:half], reserve[pick]])
                            clips = refs[torch.randint(len(refs), (len(body),), device=device, generator=update_generator)]
                        x_clip = judge.inputs(clips).detach().requires_grad_(True)
                        j_clip = judge.net(x_clip).squeeze(-1)
                        j_body = judge.net(judge.inputs(body)).squeeze(-1)
                        grad = torch.autograd.grad(j_clip.sum(), x_clip, create_graph=True)[0]
                        j_gp = grad.pow(2).sum(-1).mean()
                        last_layer = judge.net[-1].weight
                        j_loss = 0.5 * ((j_clip - 1.0).pow(2).mean() + (j_body + 1.0).pow(2).mean()) + args.disc_gp * j_gp \
                            + 0.01 * last_layer.pow(2).sum()
                        judge_opt.zero_grad()
                        j_loss.backward()
                        # (07/11 08h, --juiz-freio: o juiz da locomocao saturado desde 06/11 18h - a nota para o corpo em -0,85 a
                        # -0,96 - deixava o estilo em ~0,1 e sem direcao. Seguro demais no minilote, ele nao da o passo; o corpo
                        # chegando perto, ele volta a aprender.)
                        # (08h20: pela nota do corpo so, o juiz novo congelou no 1o passo - corpo -0,52 e clipes +0,25, sem saber
                        # ainda como os clipes sao; pela margem ele aprende os clipes e para so quando separa bem os dois.)
                        # (09h57: so o da locomocao - o do levantar congelado pelo freio foi enganado pela rede: o levantar de
                        # quedas caiu de 97% para 61% e os deitados para 0% em 1h30; o do levantar segue como era, sem freio.)
                        if args.juiz_freio is None or judge_index == 0 or disc_loco_idle is not None \
                                or (j_clip.mean() - j_body.mean()).item() < args.juiz_freio:
                            judge_opt.step()
                        else:
                            stats[f"juiz_freado_{judge_index}"] = stats.get(f"juiz_freado_{judge_index}", 0.0) + 1.0
                    else:
                        j_loss = j_clip = j_body = j_gp = torch.zeros((), device=device)
                    if judge_index == 0:
                        disc_loss, d_clip, d_body, gp = j_loss, j_clip, j_body, j_gp
                    else:
                        stats["d_loco_clipe"] = stats.get("d_loco_clipe", 0.0) + j_clip.mean().item()
                        stats["d_loco_corpo"] = stats.get("d_loco_corpo", 0.0) + j_body.mean().item()
                stats["pol"] += pol_loss.item()
                stats["vf"] += vf_loss.item()
                stats["clip"] += ((ratio - 1).abs() > clip).float().mean().item()
                stats["kl"] += kl
                stats["disc"] += disc_loss.item()
                stats["d_clipe"] += d_clip.mean().item()
                stats["d_corpo"] += d_body.mean().item()
                stats["gp"] += gp.item()
                steps += 1
        if loco_before is not None:
            # (O ultimo passo: medido numa amostra das linhas da locomocao.)
            with torch.no_grad():
                pool = torch.nonzero(f_loco & learn).squeeze(-1)
                if len(pool):
                    pick = pool[torch.randint(len(pool), (min(minibatch, len(pool)),), device=device, generator=update_generator)]
                    last = skill_logp(model, model.distribution(model.norm(f_obs[pick])), f_act[pick], f_obs[pick], gait=f_mirror[pick][:, 0] > .5)
                    log_l = (last - anchor[pick]).clamp(-20.0, 20.0)
                    if (torch.exp(log_l) - 1 - log_l).mean() > args.kl_reverter_loco:
                        for q, old in zip(list(model.actor.locomotion.parameters()) + [model.loco_log_std], loco_before):
                            q.copy_(old)
                        stats["revertidos_loco"] = stats.get("revertidos_loco", 0.0) + 1.0
            loco_before = None
        # (13/11, --cerebro-unico-imitar: a taxa parada enquanto imita - o KL do levantar ali e o da imitacao, nao o do PPO.)
        if args.taxa_adaptativa > 0 and len(actor_opt.param_groups) == 2 and not (args.cerebro_unico and teaching) \
                and counters["atualizacao"] - counters.get("aquecer_desde", 0) >= args.aquecer_critico:
            # (05/11: cada cabeca para o KL alvo no fim da atualizacao - o KL cresce ~ com o quadrado do passo, entao a raiz
            # quarta da razao, no maximo 10% por atualizacao; so a cabeca que treinou nesta.)
            for skill, group in enumerate(actor_opt.param_groups):
                target = args.taxa_adaptativa if skill == 0 or args.taxa_adaptativa_loco is None else args.taxa_adaptativa_loco
                measured = skill_kl[skill]
                if args.cerebro_unico:
                    # (13/11 14h56, medido: com a taxa fixa da locomocao, 1,5e-5, o primeiro passo ja dava KL 0,3-1,2 nas linhas do
                    # levantar - uma rede treinada responde muito mais a um passo do Adam do que a cabeca zerada de 11/11 - e o freio de
                    # 0,12 deixava 1 passo de 16. A cabeca unica pela fase do levantar, com o alvo dele (0,03, a correcao 1 de 11/11);
                    # a de pe fica com o freio e a volta dela. A recuperacao nao treina.)
                    if skill == 0:
                        continue
                    target, measured = args.taxa_adaptativa, skill_kl[0]
                if target <= 0:
                    continue
                if skill_steps[skill] > 0 or skill_frozen[skill]:
                    factor = (target / max(measured, 1.0e-6)) ** 0.25
                    group["lr"] = float(np.clip(group["lr"] * np.clip(factor, 0.9, 1.1), 1.0e-6, 3.0e-4))
            counters["taxa_cabecas"] = [group["lr"] for group in actor_opt.param_groups]
        if (costas_teacher is not None or costas_anchor is not None) and args.levantar_imitar_professor_taxa > 0:
            actor_opt.param_groups[-1]["lr"] = args.levantar_imitar_professor_taxa
        # (12/11 16h) Nunca numa cabeca congelada: de 11/11 14h48 a 12/11 o ReDo seguiu na locomocao congelada, julgando as unidades so
        # pelos 1,5 s de locomocao depois do levantar, e zerou as que a locomocao de verdade usa (vault plano_fogo_da_locomocao_12_11).
        if args.redo_atualizacoes > 0 and model.split and not args.congelar_locomocao:
            # (07/11) A janela do ReDo: o maximo de cada unidade nas linhas da locomocao desde o ultimo renascimento.
            with torch.no_grad():
                # (13/11, --cerebro-unico: as duas fases - so com as de pe, as unidades que o levantar usa pareceriam mortas.)
                rows = f_obs if args.cerebro_unico else f_obs[f_loco]
                if len(rows):
                    rows = rows[torch.randperm(len(rows), device=device, generator=update_generator)[:8192]]
                    z = model.norm(rows)
                    for tag, net in (("ator", model.actor.locomotion), ("critico", model.critic.locomotion)):
                        h1 = torch.relu(net[0](z)); h2 = torch.relu(net[2](h1))
                        for layer, h in ((0, h1), (2, h2)):
                            key = (tag, layer)
                            m = h.max(0).values
                            redo_window[key] = m if key not in redo_window else torch.maximum(redo_window[key], m)
            redo_count[0] += 1
            if redo_count[0] >= args.redo_atualizacoes:
                redo_count[0] = 0
                with torch.no_grad():
                    for tag, net, opt in (("ator", model.actor.locomotion, actor_opt), ("critico", model.critic.locomotion, critic_opt)):
                        for layer in (0, 2):
                            dead = redo_window.get((tag, layer))
                            if dead is None:
                                continue
                            dead = dead <= 0
                            k = int(dead.sum())
                            stats[f"redo_{tag}_{layer}"] = float(k) * max(1, steps)
                            if k == 0:
                                continue
                            lin, nxt = net[layer], net[layer + 2]
                            fresh = nn.Linear(lin.in_features, lin.out_features).to(device)
                            lin.weight[dead] = fresh.weight[dead]; lin.bias[dead] = fresh.bias[dead]
                            nxt.weight[:, dead] = 0.0
                            for prm in (lin.weight, lin.bias):
                                st = opt.state.get(prm, {})
                                for key in ("exp_avg", "exp_avg_sq"):
                                    if key in st: st[key][dead] = 0.0
                            st = opt.state.get(nxt.weight, {})
                            for key in ("exp_avg", "exp_avg_sq"):
                                if key in st: st[key][:, dead] = 0.0
                redo_window.clear()
        if device.type == "cuda":
            torch.cuda.synchronize()
        out = {k: v / max(1, steps) for k, v in stats.items()}
        out.update(phase_stats)
        if "revertidos_loco" in stats:
            out["revertidos_loco"] = int(stats["revertidos_loco"])
        out.update(mm_stats)
        out.update({"estilo": float(style.mean()), "peso_coleta": float(weight.mean()), "t_ppo": time.time() - start,
                    "loco_recompensa": float((task_style * loco).sum() / loco.sum().clamp_min(1)),
                    **({f"rastreio_{name}": float((roll["feedback"][..., k] * loco).sum() / loco.sum().clamp_min(1))
                        for k, name in enumerate(TRACKING_TERMS)}),
                    # (04/11, ele: "confere se isso e mesmo a 30%" - o pico da mao, o p90 da sustentacao nos passos com ela: a media
                    # esconde o "segurar" no tropeco.)
                    **({"rastreio_mao_sustenta_p90": float(torch.quantile(
                        roll["feedback"][..., TRACKING_TERMS.index("mao_sustenta_n")][(loco > .5) & (roll["feedback"][..., 0] > 0)]
                        .float(), 0.9))} if bool(((loco > .5) & (roll["feedback"][..., 0] > 0)).any()) else {}),
                    "ator_parado": float(actor_frozen), "ator_recuperacao_parado": float(skill_frozen[0]),
                    "ator_locomocao_parado": float(skill_frozen[1]),
                    "passos_ator_rec": skill_steps[0], "passos_ator_loco": skill_steps[1], "minilotes": minibatches,
                    **({"kl_rec": skill_kl[0], "kl_loco": skill_kl[1], "taxa_rec": actor_opt.param_groups[0]["lr"],
                        "taxa_loco": actor_opt.param_groups[-1]["lr"]} if model.split else {})})
        return out

    # (08/11) A estrela e a fase de fogo: as fases, os pesos e o nivel do comeco (uma vez; depois os do treino).
    counters.setdefault("comando_fase", int(args.comando_fase))
    counters.setdefault("comando_desde", counters["atualizacao"])
    command_rows: list = []
    counters.setdefault("estrela_fases", [int(x) for x in args.estrela_fases])
    counters.setdefault("estrela_pesos", [1.0] * 6)
    counters.setdefault("estrela_desde", {k: counters["atualizacao"] for k in STAR_KEYS})
    counters.setdefault("fogo_nivel", max(0.0, args.fogo))
    counters.setdefault("lento", [args.lento] * 3)
    if not isinstance(counters["lento"], list):
        # (12h25: o piso unico de antes vira o da frente e o das costas; os lados, a ~60% da meta, comecam no 0,5.)
        floor = float(counters["lento"])
        counters["lento"] = [floor, min(floor, 0.5), floor]
    counters["lento"] = [max(float(f), args.lento_minimo[i % 3]) for i, f in enumerate(counters["lento"])]
    counters.setdefault("lento_desde", counters["atualizacao"])
    if not isinstance(counters["lento_desde"], list):
        counters["lento_desde"] = [int(counters["lento_desde"])] * 3
    if args.lento_faixa and len(counters["lento"]) == 3:
        # (09/11: o piso da corrida comeca 0,10 abaixo do do grupo - ela estava abaixo do piso que o trote subiu.)
        counters["lento"] = list(counters["lento"]) + [max(m, float(f) - 0.10) for f, m in zip(counters["lento"], args.lento_minimo)]
        counters["lento_desde"] = list(counters["lento_desde"]) * 2
    elif not args.lento_faixa and len(counters["lento"]) == 6:
        counters["lento"], counters["lento_desde"] = list(counters["lento"][:3]), list(counters["lento_desde"][:3])
    slow_rows: list = [[] for _ in counters["lento"]]
    counters.setdefault("fogo_desde", counters["atualizacao"])
    # (O giro comeca em -0,5: a sonda das 12h27 - 100% de quedas com 120 N continuos.)
    counters.setdefault("fogo_niveis", [float(counters["fogo_nivel"])] * 3 + [min(-0.5, float(counters["fogo_nivel"]))]
                        + [float(counters["fogo_nivel"])] * 2)
    counters.setdefault("fogo_niveis_desde", [counters["atualizacao"]] * len(FIRE_GROUPS))
    # (08/11 18h30: os niveis no run.json desde o comeco - o laboratorio, com a caixa "Empurroes", le dali; antes so apareciam
    # na primeira mudanca.)
    if args.fogo >= 0:
        info = json.loads((run / "run.json").read_text())
        info["v2"]["fogo_niveis"] = dict(zip(FIRE_GROUPS, counters["fogo_niveis"]))
        (run / "run.json").write_text(json.dumps(info, ensure_ascii=False))
    star_rows: dict = {k: [] for k in STAR_KEYS}
    fire_rows: list = []
    fire_group_rows: list = [[] for _ in range(6)]
    if args.estrela > 0 or args.fogo >= 0 or args.comando > 0:
        info = json.loads((run / "run.json").read_text())
        info["v2"].update(comando=args.comando, comando_fase=int(counters["comando_fase"]), comando_segundos=args.comando_segundos)
        info["v2"].update(estrela=args.estrela, estrela_fases=counters["estrela_fases"], estrela_segundos=args.estrela_segundos,
                          fogo_nivel=counters["fogo_nivel"] if args.fogo >= 0 else -1.0,
                          fogo_fatia=args.fogo_fatia if args.fogo >= 0 else 0.0)
        (run / "run.json").write_text(json.dumps(info, ensure_ascii=False))
        print(f"estrela: fatia {args.estrela}, fases {counters['estrela_fases']}; fogo: "
              f"{'nivel ' + format(counters['fogo_nivel'], '.2f') + ' ' + str(fire_bands(counters['fogo_nivel'])) if args.fogo >= 0 else 'sem'}"
              f", fatia {args.fogo_fatia}", flush=True)
    if args.equilibrio:
        counters.setdefault("equilibrio_nivel", args.equilibrio_nivel)
        counters.setdefault("equilibrio_desde", counters["atualizacao"])
        evaluations["equilibrio_"] = get_up_batch(str(REPO / "assets"), environments=16, threads=eval_threads, seed=556,
                                                  evaluation=True, evaluation_kind=8, **v2)
        configurar(evaluations["equilibrio_"], args, counters, False)
        info = json.loads((run / "run.json").read_text())
        info["v2"]["equilibrio_nivel"] = counters["equilibrio_nivel"]
        (run / "run.json").write_text(json.dumps(info, ensure_ascii=False))
        print(f"equilibrio: gerador de passos no idle, empurrao de pe com {'o idle' if args.empurrao_idle else 'a recuperacao'}, "
              f"chegada {args.chegada_premio}, voo {args.voo_multa}, fatia {args.equilibrio_fatia} no nivel "
              f"{counters['equilibrio_nivel']}", flush=True)
    if args.levantar_unreal > 0 or args.calibrar_levantar:
        # (11/11 14h: o levantar pelo clipe, 16 de cada exercicio, deitado no quadro 0 sem variacao - configurados como o treino:
        # o idle e a corrida da locomocao depois da troca sao os dele.)
        # (12/11 19h10: e a do levantar e ir - o exercicio sorteado em cada mundo, a fatia cheia.)
        for exercise, prefix in list(enumerate(LEVANTAR_UNREAL_PREFIXES)) + ([(0, LEVANTAR_E_IR_PREFIX)] if args.levantar_e_ir[0] > 0 else []):
            evaluations[prefix] = get_up_batch(str(REPO / "assets"), environments=16, threads=eval_threads,
                                               seed=700 + exercise + (10 if prefix == LEVANTAR_E_IR_PREFIX else 0),
                                               evaluation=True, evaluation_kind=11 + exercise, **v2)
            if prefix == LEVANTAR_E_IR_PREFIX:
                evaluations[prefix].set_get_up_go(1.0, args.levantar_e_ir[1], args.levantar_e_ir[2])
            configurar(evaluations[prefix], args, counters, False)
            evaluations[prefix].add_style_clips(LEVANTAR_UNREAL)
            evaluations[prefix].set_clip_get_up(LEVANTAR_UNREAL, 0.0, 0.0, args.levantar_unreal_depois, 0.0,
                                                args.levantar_unreal_largura, args.levantar_unreal_vel)
            evaluations[prefix].set_clip_get_up_parts(args.levantar_unreal_partes)
            evaluations[prefix].set_clip_get_up_variants(list(LEVANTAR_UNREAL_VARIANTS))
            if hasattr(evaluations[prefix], "set_clip_get_up_deliver"):
                evaluations[prefix].set_clip_get_up_deliver(list(LEVANTAR_UNREAL_DELIVER))
            if hasattr(evaluations[prefix], "set_clip_get_up_agility"):
                evaluations[prefix].set_clip_get_up_agility(clip_get_up_agility(args, counters, full=True))
            if args.levantar_adaptativo:
                evaluations[prefix].set_clip_get_up_adaptive(True)
                if hasattr(evaluations[prefix], "set_adaptive_settle"):
                    evaluations[prefix].set_adaptive_settle(args.levantar_adaptativo_firmeza, args.levantar_fogo_soltar)
                if args.levantar_tentar:
                    evaluations[prefix].set_adaptive_retry(True)
                if hasattr(evaluations[prefix], "set_run_post_rise"):
                    evaluations[prefix].set_run_post_rise(*args.levantar_velocidade_corrida)
                if args.levantar_correndo_troca >= 0 and hasattr(evaluations[prefix], "set_run_handoff_at_stand"):
                    evaluations[prefix].set_run_handoff_at_stand(True, args.levantar_correndo_troca)
                if args.levantar_troca_calma[0] > 0:
                    evaluations[prefix].set_calm_handoff(*args.levantar_troca_calma)
                if args.levantar_fogo_contra_corrida > 0:
                    evaluations[prefix].set_fire_run_headwind_cap(args.levantar_fogo_contra_corrida)
                if max(args.levantar_paredes) > 0:
                    evaluations[prefix].set_get_up_arena(*args.levantar_paredes)
            fire = get_up_fire_state(args, counters)
            if fire is not None:
                # (A prova de fogo: o cenario pelo mundo - 4 limpos, 6 continuos, 6 pancadas - nos niveis de partida, fixos.)
                evaluations[prefix].set_get_up_fire([1.0, 0.0, 0.0, 0.0], fire_levels_flat(fire["prova"]), True)
                if exercise == 0 and prefix != LEVANTAR_E_IR_PREFIX:
                    batch.set_get_up_fire(list(args.levantar_fogo) + [args.levantar_fogo_de_cima], fire_levels_flat(fire["niveis"]), False)
                    print("fogo do levantar: fatias", list(args.levantar_fogo), "niveis", fire["niveis"], flush=True)
            if muscle:
                evaluations[prefix].set_student_actuator(muscle["rigidez"], muscle["amortecimento"], muscle["teto"],
                                                         bool(muscle["gravidade"]))
    configurar(batch, args, counters, True)
    if args.parado_torto[0] > 0 and hasattr(batch, "set_crooked_start"):
        batch.set_crooked_start(*args.parado_torto)
        if args.parado_torto_sem_fogo and hasattr(batch, "set_crooked_no_fire"):
            batch.set_crooked_no_fire(True)
        if hasattr(batch, "set_crooked_fix_span"):
            batch.set_crooked_fix_span(args.parado_torto_janela)
        print(f"parado torto: fatia {args.parado_torto[0]}, premio {args.parado_torto[1]}, perto {args.parado_torto[2]}", flush=True)
    loco_fire = loco_fire_state(args, counters)
    if loco_fire is not None and hasattr(batch, "set_locomotion_fire"):
        if args.locomocao_fogo_fatia_inicial > 0:
            counters.setdefault("fogo_protecao", {"fatia": min(args.locomocao_fogo_fatia_inicial, args.locomocao_fogo)})
        if "fogo_protecao" in counters:
            counters["fogo_protecao"]["fatia"] = min(float(counters["fogo_protecao"].get("fatia", args.locomocao_fogo)), args.locomocao_fogo)
        batch.set_locomotion_fire(counters.get("fogo_protecao", {}).get("fatia", args.locomocao_fogo), loco_fire["niveis"],
                                  args.locomocao_fogo_liberdade)
        if hasattr(batch, "set_locomotion_fire_gaps"):
            batch.set_locomotion_fire_gaps(args.locomocao_fogo_intervalo)
        print(f"fogo da locomocao: fatia {counters.get('fogo_protecao', {}).get('fatia', args.locomocao_fogo)} (a da receita "
              f"{args.locomocao_fogo}), liberdade {args.locomocao_fogo_liberdade}, niveis "
              f"{dict(zip(FIRE_GROUPS, loco_fire['niveis']))}", flush=True)
    for e in evaluations.values():
        e.set_phase_sync(args.sincronizar_fase)
        e.set_arm_clip_reach(args.bracos_clipe)
        e.set_gait_cycles(args.passos_ciclos)
        e.set_head_clip_reach(1.0 if args.cabeca_autoridade else args.cabeca_clipe)
    if args.bracos_clipe >= 0 and model.split and counters.get("bracos_clipe") != args.bracos_clipe:
        # (08/11: a saida dos bracos passa a ser correcao do clipe - zerada, a rede comeca fazendo os bracos do clipe.)
        names = batch.action_names()
        last = [m for m in model.actor.locomotion if isinstance(m, nn.Linear)][-1]
        # (O tamanho pela saida de verdade: com --acao-velocidade os nomes tambem trazem os da velocidade.)
        steps = sum(1 for nm in names if nm.startswith("passo."))
        axes = (last.bias.numel() - steps) // (2 if args.acao_velocidade else 1)
        arm = [k for k, nm in enumerate(names[:axes]) if any(t in nm.split(".")[0] for t in ("Arm", "arm", "Hand"))]
        rows = arm + ([axes + steps + k for k in arm] if args.acao_velocidade else [])
        assert max(rows) < last.bias.numel(), (rows, last.bias.numel())
        with torch.no_grad():
            last.weight[rows] = 0.0
            last.bias[rows] = 0.0
        counters["bracos_clipe"] = args.bracos_clipe
        print(f"bracos no clipe (alcance {args.bracos_clipe}): {len(arm)} eixos, saidas zeradas", flush=True)
    if args.cabeca_clipe >= 0 and model.split and counters.get("cabeca_clipe") != args.cabeca_clipe:
        # (08/11 10h40: a saida do pescoco e da cabeca passa a ser correcao do clipe - zerada, como a dos bracos.)
        names = batch.action_names()
        last = [m for m in model.actor.locomotion if isinstance(m, nn.Linear)][-1]
        steps = sum(1 for nm in names if nm.startswith("passo."))
        axes = (last.bias.numel() - steps) // (2 if args.acao_velocidade else 1)
        head = [k for k, nm in enumerate(names[:axes]) if any(t in nm.split(".")[0] for t in ("Neck", "Head"))]
        rows = head + ([axes + steps + k for k in head] if args.acao_velocidade else [])
        assert head and max(rows) < last.bias.numel(), (rows, last.bias.numel())
        with torch.no_grad():
            last.weight[rows] = 0.0
            last.bias[rows] = 0.0
        counters["cabeca_clipe"] = args.cabeca_clipe
        print(f"cabeca no clipe (alcance {args.cabeca_clipe}): {len(head)} eixos, saidas zeradas", flush=True)
    if args.cabeca_autoridade and args.cabeca_clipe > 0 and model.split and not counters.get("cabeca_autoridade"):
        # (11/11, E5-13) A troca para a autoridade inteira: o alvo e clipe + alcance*(acao - meio); com o alcance 1, as linhas de
        # saida da cabeca (so as de posicao - a velocidade dela vem do clipe) e o desvio delas vezes o alcance de antes: comeca igual.
        names = batch.action_names()
        last = [m for m in model.actor.locomotion if isinstance(m, nn.Linear)][-1]
        steps = sum(1 for nm in names if nm.startswith("passo."))
        axes = (last.bias.numel() - steps) // (2 if args.acao_velocidade else 1)
        head = [k for k, nm in enumerate(names[:axes]) if any(t in nm.split(".")[0] for t in ("Neck", "Head"))]
        assert head
        with torch.no_grad():
            last.weight[head] *= args.cabeca_clipe
            last.bias[head] *= args.cabeca_clipe
            model.loco_log_std[head] += math.log(args.cabeca_clipe)
        counters["cabeca_autoridade"] = True
        print(f"cabeca com autoridade inteira: {len(head)} eixos, saidas e desvio vezes {args.cabeca_clipe}", flush=True)
    balance_rows: list = []
    drill_rows: dict = {k: [] for k in DRILL_KEYS}

    def set_drill_assist() -> None:
        """A forca de cada exercicio no treino e no run.json do laboratorio."""
        levels = counters["ajuda_exercicios"]
        batch.set_drill_assist([float(levels[k]) for k in DRILL_KEYS])
        info = json.loads((run / "run.json").read_text())
        info["v2"]["ajuda_exercicios"] = {k: round(float(levels[k]), 4) for k in DRILL_KEYS}
        (run / "run.json").write_text(json.dumps(info, ensure_ascii=False))

    # (06/11 07h, ele: o grafico "capacidade em forca fixa" - de tempos em tempos cada exercicio num teste sempre igual (a rede
    # media, comecos fixos) sem a forca e, nas trocas e no levantar, tambem a 15%: a linha so sobe se a rede aprendeu. O de pe
    # do treino fica perto do piso pela descida da forca e nao mostra a evolucao.)
    capacity_batches: list = []
    capacity_at = [0.0]
    capacity_counted = [False]
    if args.capacidade_minutos > 0:
        plain_v2 = dict(v2, moving_push_newtons=(0.0, 0.0), moving_push_hold_newtons=(0.0, 0.0))
        for band in (1, 2):
            # (06/11 19h40, ele: "por que parece haver decaimentos e subidas assim?" - com 8 episodios por rumo e 4 por manobra
            # cada um valia 12,5-25% e a linha saltava; agora 24 por rumo e 12 por manobra.)
            rumos = get_up_batch(str(REPO / "assets"), environments=192, threads=eval_threads, seed=700 + band, evaluation=True,
                                 evaluation_kind=4, evaluation_band=band, **plain_v2)
            configurar(rumos, args, counters, False)
            capacity_batches.append(("rumos", rumos))
            moves = get_up_batch(str(REPO / "assets"), environments=84, threads=eval_threads, seed=710 + band, evaluation=True,
                                 evaluation_kind=4, evaluation_band=band, **plain_v2)
            configurar(moves, args, counters, False)
            moves.set_maneuvers(0.0, 0)
            capacity_batches.append(("manobras", moves))
        if args.parada > 0:
            # (08/11 15h20: a parada - os 8 rumos no trote e na corrida, 3 de cada, sempre com a parada no meio da janela.)
            for band in (1, 2):
                stop_batch = get_up_batch(str(REPO / "assets"), environments=24, threads=eval_threads, seed=770 + band,
                                          evaluation=True, evaluation_kind=4, evaluation_band=band, **plain_v2)
                configurar(stop_batch, args, counters, False)
                stop_batch.set_final_stop(0.0, *args.parada_janela, args.parada_freio, True)
                capacity_batches.append(("parada", stop_batch))
        if args.comando > 0:
            # (08/11 14h40: o comando na fase 3 - a regua fixa do jogo -, 24 episodios.)
            cmd = get_up_batch(str(REPO / "assets"), environments=24, threads=eval_threads, seed=760, evaluation=True,
                               evaluation_kind=4, evaluation_band=1, **plain_v2)
            configurar(cmd, args, counters, False)
            cmd.set_maneuvers(0.0, 12)
            cmd.set_command(0.0, 3, args.comando_segundos)
            capacity_batches.append(("comando", cmd))
        if args.estrela > 0:
            # (08/11: a estrela de jogo - a fase 4, a regua fixa -, 12 por peito em cada faixa.)
            for band in (1, 2):
                star = get_up_batch(str(REPO / "assets"), environments=36, threads=eval_threads, seed=730 + band,
                                    evaluation=True, evaluation_kind=4, evaluation_band=band, **plain_v2)
                configurar(star, args, counters, False)
                star.set_maneuvers(0.0, 11)
                star.set_star(0.0, [1.0] * 6, [4] * 6, args.estrela_segundos)
                capacity_batches.append(("estrela", star))
        if args.fogo >= 0:
            # (08/11, a regua fixa do fogo: um empurrao por episodio - pancada 500-900 N ou, metade, o continuo de 200-350 N por
            # 3-5 s -, 2-3 s depois do comeco nos rumos, 6 por rumo em cada faixa; e o parado, com eles a cada 1,5-3 s.)
            fire_hit, fire_hold = (500.0, 900.0), (200.0, 350.0)
            fire_v2 = dict(v2, moving_push_newtons=fire_hit, moving_push_hold_fraction=FIRE_HOLD_FRACTION,
                           moving_push_hold_newtons=fire_hold)
            for band in (1, 2):
                fire = get_up_batch(str(REPO / "assets"), environments=48, threads=eval_threads, seed=740 + band,
                                    evaluation=True, evaluation_kind=4, evaluation_band=band, **fire_v2)
                configurar(fire, args, counters, False)
                fire.set_fire(1.0, fire_hit, fire_hold, FIRE_HOLD_SECONDS, FIRE_HOLD_FRACTION)
                capacity_batches.append(("fogo", fire))
            if args.equilibrio:
                still = get_up_batch(str(REPO / "assets"), environments=16, threads=eval_threads, seed=750, evaluation=True,
                                     evaluation_kind=8, **v2)
                configurar(still, args, counters, False)
                still.set_balance_drill(0.0, 1.0)
                still.set_fire(1.0, fire_hit, fire_hold, FIRE_HOLD_SECONDS, FIRE_HOLD_FRACTION)
                capacity_batches.append(("fogo", still))
        if args.locomocao_fogo > 0 and args.locomocao_fogo_prova > 0:
            # (12/11 16h30, a prova de fogo da locomocao: uma adversidade por mundo no nivel da prova - o continuo, a serie, o agarrao, a
            # grama molhada com uma pancada -, 8 de cada nos rumos do trote e da corrida e no parado.)
            for band in (1, 2):
                exam = get_up_batch(str(REPO / "assets"), environments=32, threads=eval_threads, seed=780 + band, evaluation=True,
                                    evaluation_kind=4, evaluation_band=band, **plain_v2)
                configurar(exam, args, counters, False)
                exam.set_locomotion_fire(0.0, [0.0] * len(FIRE_GROUPS), args.locomocao_fogo_liberdade)
                exam.set_locomotion_fire_exam(True, args.locomocao_fogo_prova)
                loco_fire_extras(exam, args)
                capacity_batches.append(("prova_fogo", exam))
            exam = get_up_batch(str(REPO / "assets"), environments=32, threads=eval_threads, seed=790, evaluation=True,
                                evaluation_kind=3, **plain_v2)
            configurar(exam, args, counters, False)
            exam.set_locomotion_fire(0.0, [0.0] * len(FIRE_GROUPS), args.locomocao_fogo_liberdade)
            exam.set_locomotion_fire_exam(True, args.locomocao_fogo_prova)
            loco_fire_extras(exam, args)
            capacity_batches.append(("prova_fogo", exam))
        if args.locomocao_fogo > 0:
            # (12/11 20h05, ele: "nao depreciar velocidade e postura (semelhanca com o clipe nos exercicios limpos)") O parado limpo na
            # capacidade: 16 normais (a semelhanca com o idle) e 32 nascidos tortos (o reajuste), a rede media, sem fogo - de 18h46 a 19h50
            # o reajuste do torto foi de 27% a 0% sem ninguem ver (so a sonda viu).
            # (21h40: o torto com 32 mundos oscilava 0,25-0,50 com a mesma rede; com 96, 0,438 na das 17h55 e 0,417 na das 21h20.)
            for kind, worlds, seed in (("parado", 16, 796), ("torto", 96, 795)):
                still = get_up_batch(str(REPO / "assets"), environments=worlds, threads=eval_threads, seed=seed, evaluation=True,
                                     evaluation_kind=3, **plain_v2)
                configurar(still, args, counters, False)
                if kind == "torto" and hasattr(still, "set_crooked_start"):
                    still.set_crooked_start(1.0, args.parado_torto[1], args.parado_torto[2] if args.parado_torto[2] > 0 else 0.7)
                capacity_batches.append((kind, still))
        if GETUP_CLIP[0]:
            rise = get_up_batch(str(REPO / "assets"), environments=16, threads=eval_threads, seed=720, evaluation=True,
                                evaluation_kind=0, **v2)
            configurar(rise, args, counters, False)
            capacity_batches.append(("levantar", rise))
        print(f"capacidade em forca fixa a cada {args.capacidade_minutos:g} min: {len(capacity_batches)} lotes", flush=True)

    def capacity(model) -> dict:
        """Cada exercicio, um episodio por mundo, a rede media, a 0% e (trocas e levantar) a 15%: o de pe e a velocidade."""
        out = {}
        for tag, level in (("0", 0.0), ("15", 0.15)):
            per: dict = {}
            for kind, b in capacity_batches:
                if tag == "15" and kind in ("rumos", "estrela", "fogo", "comando", "parada", "prova_fogo", "parado", "torto"):
                    continue
                b.set_drill_assist([level] * len(DRILL_KEYS))
                b.reset(12345)
                done: dict = {}
                for _ in range(int(b.episode_seconds / b.policy_dt) + 2):
                    with torch.no_grad():
                        obs = torch.as_tensor(b.observations(), device=device)
                        action = model.env_action(model.actor(model.norm(obs)), obs).cpu().numpy().astype(np.float32)
                    b.step(action)
                    for r in b.finished_episodes().tolist():
                        done.setdefault(int(r[0]), r)
                    if len(done) >= b.num_envs:
                        break
                for r in done.values():
                    key = drill_key(r)
                    if kind == "fogo":
                        # (O fogo a parte: por faixa, e o parado.)
                        key = "fogo_parado" if r[1] == 2 else f"fogo_{'corrida' if r[25] == 2 else 'trote'}"
                    if kind == "parada":
                        key = f"parada_{'corrida' if r[25] == 2 else 'trote'}"
                    if kind in ("parado", "torto"):
                        key = f"parado_{'limpo' if kind == 'parado' else 'torto'}"
                    if kind == "prova_fogo":
                        tipo = ("continuo", "serie", "agarrao", "molhado")[(int(r[0]) // 8) % 4]
                        key = f"prova_fogo_{'parado' if r[1] == 2 else 'corrida' if r[25] == 2 else 'trote'}_{tipo}"
                    if key:
                        per.setdefault(key, []).append(r)
            res = {}
            for key, rows in per.items():
                if tag == "15" and not (key.startswith("lev_troca") or key == "lev_levantar"):
                    continue
                held_rows = np.array(rows)
                if key.startswith("prova_fogo_"):
                    kept = held_rows[:, 63] >= 0 if held_rows.shape[1] > 63 else np.zeros(len(held_rows), dtype=bool)
                    res[key] = {"sem_cair": round(float(loco_fire_ok(held_rows).mean()), 3),
                                "seguiu": round(float(held_rows[kept, 63].mean()), 3) if kept.any() else None, "n": len(rows)}
                    continue
                if key in ("parado_limpo", "parado_torto"):
                    tracked = held_rows[held_rows[:, 33] >= 0]
                    res[key] = {"de_pe": round(float(loco_fire_ok(held_rows).mean()), 3), "n": len(rows),
                                "sem": round(float(tracked[:, 33].mean()), 3) if len(tracked) else None}
                    if key == "parado_torto" and held_rows.shape[1] > 65:
                        crook, fixed = held_rows[:, 64] > 0, held_rows[:, 65] >= 0
                        took = held_rows[crook & fixed, 65]
                        res[key].update(reajustou=round(float(fixed[crook].mean()), 3) if crook.any() else None,
                                        reajuste_s=round(float(np.median(took)), 2) if len(took) else None)
                    continue
                perf = drill_performance(held_rows, args.locomocao_segundos)
                res[key] = {"de_pe": round(float(perf["fim"]), 3), "vel": round(float(perf["vel"]), 3), "n": len(rows),
                            "sem": round(float(perf["semelhanca"]), 3)}
                # (13/11 v1, a revisao: o de_pe conta qualquer fim antes da hora - no comando e na estrela ~80% eram o fim pelo virado, a regra
                # do treino, nao queda. A queda de verdade: o fim por cair ou perder o equilibrio, coluna 32 em 1 ou 2.)
                if held_rows.shape[1] > 32:
                    res[key]["caiu"] = round(float(np.isin(held_rows[:, 32], (1, 2)).mean()), 3)
                if key.startswith("fogo_") and held_rows.shape[1] > 45:
                    pushed = held_rows[held_rows[:, 38] > 0]
                    res[key]["sem_cair_empurrao"] = round(float(1.0 - pushed[:, 45].mean()), 3) if len(pushed) else None
                if key.startswith("parada_") and held_rows.shape[1] > 46:
                    stopped = held_rows[held_rows[:, 46] >= 0]
                    res[key]["erro_m"] = round(float(stopped[:, 46].mean()), 3) if len(stopped) else None
                if (key.startswith("lev_estrela") or key == COMMAND_KEY) and held_rows.shape[1] > 43:
                    res[key]["pernas"] = round(float(held_rows[:, 43].mean()), 2)
            out[tag] = res
        return out

    # (A forca por exercicio fica em zero - treino sem forca auxiliar; o --sem-mao de 06/11 virou o padrao na limpeza c3b) e o
    # portao segue medindo o dominio e as exigencias de cada exercicio; os exercicios novos entram com o estado de agora.)
    counters["ajuda_exercicios"] = {k: 0.0 for k in DRILL_KEYS}
    states = counters.setdefault("ajuda_estado", {})
    for key in DRILL_KEYS:
        states.setdefault(key, {"desde": counters["atualizacao"]})
    set_drill_assist()
    # (O teto da semelhanca de cada exercicio, medido por teto_semelhanca.py; sem ele, a regua relativa a melhor marca.)
    ceilings = json.loads((run / "teto_semelhanca.json").read_text()) if (run / "teto_semelhanca.json").exists() else {}
    speed_ceilings = json.loads((run / "teto_velocidade.json").read_text()) if (run / "teto_velocidade.json").exists() else {}
    # (A estabilidade do clipe puro com a mao cheia, teto_semelhanca.py - o teto da exigida, required_stability; 05/11 02h35,
    # todos os exercicios; antes so as trocas e o sprint, pela teto_firmeza.json da sonda.)
    stability_ceilings = json.loads((run / "teto_estabilidade.json").read_text()) \
        if (run / "teto_estabilidade.json").exists() else {}
    if ceilings:
        print(f"teto da semelhanca por exercicio: {ceilings}", flush=True)

    if balance_calibration is not None:
        # (No lote do treino com todos os ajustes - o idle, o gerador, as faixas: as entradas novas como o treino as ve.)
        print(f"rede no layout do equilibrio: {balance_calibration[0] + 1} -> {obs_size} entradas, {act_size} saidas; normalizacao "
              f"das novas {calibrate_new_columns(model, batch, device, *balance_calibration, decisions=96)}", flush=True)
        collector.load_state_dict(model.state_dict())
        batch.reset(args.semente)
        obs = torch.as_tensor(batch.observations(), device=device)
    if args.calibrar_levantar:
        calibrate_clip_get_up(model, evaluations, device, Path(args.calibrar_levantar).expanduser(), batch)
        return 0
    while (args.atualizacoes <= 0 or counters["atualizacao"] < args.atualizacoes) and not stop["pedido"]:
        begin = time.time()
        if (args.largada_clipe[0] > 0 or args.largada_mao > 0) and hasattr(batch, "set_launch_clip_share"):
            # (10/11 07h10) A parcela do clipe nos motores na largada, descendo com o treino desde a primeira atualizacao com ela.
            counters.setdefault("largada_clipe_desde", counters["atualizacao"])
            counters["largada_clipe"] = round(args.largada_clipe[0] * max(0.0, 1.0 - (counters["atualizacao"] - counters["largada_clipe_desde"])
                                                                       / max(1.0, args.largada_clipe[1])), 4)
            batch.set_launch_clip_share(counters["largada_clipe"])
            # (A mao na largada desce na mesma proporcao.)
            counters["largada_mao"] = round(args.largada_mao * max(0.0, 1.0 - (counters["atualizacao"] - counters["largada_clipe_desde"])
                                                                  / max(1.0, args.largada_clipe[1])), 4)
            if hasattr(batch, "set_launch_hand"):
                batch.set_launch_hand(counters["largada_mao"])
        physics = 0.0
        # (Os "valid" e "drove" das professoras, apagadas na limpeza c2, ficam em zero: a prova hasheia a coleta inteira.)
        roll = {k: torch.zeros(T, n, *s, device=device) for k, s in
                (("obs", (obs_size,)), ("act", (act_size,)), ("logp", ()), ("val", ()), ("task", ()), ("boot", ()),
                 ("done", ()), ("pairs", (2 * amp,)), ("jerk", ()), ("limit", ()), ("prot", ()), ("loco", ()),
                 ("feedback", (len(TRACKING_TERMS),)), ("clip", (act_size,)), ("valid", ()), ("drove", ()),
                 ("espelho", (3,)))}
        if mm is not None:
            roll["mm"] = torch.zeros(T, n, mm.width + 2, device=device)
        with torch.no_grad():
            for t in range(T):
                # (11/11 14h: com a cabeca do levantar do zero a normalizacao dela aprende (--recuperacao-norma); com a locomocao
                # congelada a dela fica parada - 13h10 medido: aprendendo os estados do levantar, a pose do parado dela caiu
                # 0,66 -> 0,36-0,57 com os pesos intactos.)
                if model.split:
                    # (13/11, --cerebro-unico: a mistura fica parada - mexer nela mexe na locomocao.)
                    if not args.cerebro_unico:
                        collector.norm.update(obs, recovery=args.recuperacao_norma, locomotion=not args.congelar_locomocao)
                else:
                    collector.norm.update(obs)
                obs_n = collector.norm(obs)
                dist = collector.distribution(obs_n)
                if colored > 0:
                    noise_state = noise_keep * noise_state + math.sqrt(1.0 - noise_keep ** 2) * torch.randn_like(noise_state)
                    action = dist.mean + dist.stddev * (math.sqrt(1.0 - colored) * torch.randn_like(noise_state)
                                                        + math.sqrt(colored) * noise_state)
                else:
                    action = dist.sample()
                roll["obs"][t], roll["act"][t] = obs, action
                if mirror is not None:
                    # (O espelho do estado destas entradas: lido antes do step, como elas.)
                    roll["espelho"][t] = torch.as_tensor(batch.mirror_info(), device=device)
                # (14/11 09h10: o DAgger do rolar tambem precisa das linhas marcadas pelos alvos do clipe - sem isto a perda do professor ficou em 0.)
                if args.imitar > 0 or (args.cerebro_unico and args.cerebro_unico_imitar[0] > 0) or args.levantar_imitar_professor:
                    roll["clip"][t] = torch.as_tensor(batch.locomotion_targets(), device=device)
                # (01/11 00h17: o tranco na acao media da rede, nao na sorteada - nas sorteadas ele tambem punia o ruido
                # da exploracao, 2 sigma^2 em media, e o PPO encolhia o desvio para fugir do custo; o Codex concordou.)
                mean = dist.mean
                roll["limit"][t] = torch.clamp(mean.abs() - 0.8, min=0.0).pow(2).mean(-1)
                roll["jerk"][t] = torch.where(previous_valid, (mean - previous_action).pow(2).mean(-1),
                                              torch.zeros_like(previous_valid, dtype=torch.float32))
                roll["logp"][t] = skill_logp(collector, dist, action, obs, gait=(roll["espelho"][t][:, 0] > .5) if mirror is not None else None)
                roll["val"][t] = collector.value(obs_n)
                env_action = collector.env_action(action, obs).cpu().numpy().astype(np.float32)
                roll["prot"][t] = torch.as_tensor(batch.protecting(), device=device).float()
                roll["loco"][t] = torch.as_tensor(batch.locomoting(), device=device).float()
                tick = time.time()
                task, term, trunc = batch.step(env_action)
                roll["feedback"][t] = torch.as_tensor(batch.locomotion_feedback(), device=device)
                physics += time.time() - tick
                roll["task"][t] = torch.as_tensor(task, device=device)
                roll["pairs"][t] = torch.as_tensor(batch.amp_pairs(), device=device)
                if mm is not None:
                    roll["mm"][t] = torch.as_tensor(batch.motion_queries(), device=device)
                trunc = torch.as_tensor(trunc, device=device).float()
                if trunc.any():
                    final = torch.as_tensor(batch.final_observations(), device=device)
                    roll["boot"][t] = trunc * collector.value(collector.norm(final))
                roll["done"][t] = torch.maximum(torch.as_tensor(term, device=device).float(), trunc)
                if args.linha_final_so:
                    # (11/11, sonda E5-4) nas linhas de termino, desfaz a metade da correcao fora do teste: "custo" deixa o
                    # portao do estilo sem agir (portao_velocidade 0); "estilo" deixa o teto do custo em 0 (nota 0), como antes.
                    ended = roll["done"][t] > .5
                    col = TRACKING_TERMS.index("portao_velocidade") if args.linha_final_so == "custo" else 0
                    roll["feedback"][t, :, col] = torch.where(ended, torch.zeros_like(ended, dtype=torch.float32), roll["feedback"][t, :, col])
                next_obs = torch.as_tensor(batch.observations(), device=device)
                if model.split:
                    # Do not bootstrap the locomotion critic from recovery's unrelated positive values.
                    # (So a troca de cabeca DENTRO do episodio: o que acabou pelo tempo e recomecou na outra cabeca guarda o
                    # bootstrap dele - a revisao de 03/11.)
                    switched = ((next_obs[:, -1] > .5) != (obs[:, -1] > .5)) & (roll["done"][t] < 0.5)
                    roll["done"][t] = torch.maximum(roll["done"][t], switched.float())
                    roll["boot"][t] *= (~switched).float()
                previous_action = mean
                previous_valid = roll["done"][t] < 0.5
                obs = next_obs
            roll["last_value"] = collector.value(collector.norm(obs))
        collected = time.time()
        if prova is not None:
            prova.coleta(counters["atualizacao"], roll)
        # A atualizacao da rodada anterior acaba; a rede dela vai para a coleta (com a normalizacao desta coleta) e esta
        # rodada vai para a atualizacao, depois da escrita (o modelo so muda na thread da atualizacao).
        if pending is not None:
            last = pending.result()
            if anchor_obs is not None:
                print(f"r{counters['atualizacao']} ancora_dist={last.get('ancora_dist', float('nan')):.4g} "
                      f"t_ppo={last['t_ppo']:.3f}", flush=True)
            if prova is not None:
                prova.rede(counters["atualizacao"], model, disc, disc_loco)
        waited = time.time() - collected
        with torch.no_grad():
            model.norm.load_state_dict(collector.norm.state_dict())
            collector.load_state_dict(model.state_dict())
        episodes = batch.finished_episodes()
        if prova is not None:
            prova.episodios(counters["atualizacao"], episodes)
        if len(episodes) and "ajuda_exercicios" in counters:
            # (Os episodios de cada exercicio depois de 4 atualizacoes - ~10 s - da ultima mudanca dele.)
            for r in episodes.tolist():
                key = drill_key(r)
                if key in counters["ajuda_estado"] and counters["atualizacao"] - counters["ajuda_estado"][key]["desde"] >= 4:
                    drill_rows[key].append(r)
        if len(episodes) and args.comando > 0:
            command_rows.extend(r for r in episodes.tolist() if len(r) > 44 and r[1] == 6 and r[26] == 12 and r[38] <= 0
                                and counters["atualizacao"] - counters["comando_desde"] >= 4)
            del command_rows[:-120]
        if len(episodes) and (args.estrela > 0 or args.fogo >= 0 or args.lento > 0):
            # (08/11: os episodios de cada exercicio da estrela e os com empurrao, 4 atualizacoes depois da ultima mudanca.)
            for r in episodes.tolist():
                key = star_key(r)
                if key and counters["atualizacao"] - counters["estrela_desde"][key] >= 4:
                    star_rows[key].append(r)
                    del star_rows[key][:-120]
                if args.fogo >= 0 and len(r) > 45:
                    g = fire_group(r)
                    if counters["atualizacao"] - counters["fogo_niveis_desde"][g] >= 4:
                        fire_group_rows[g].append(r)
                        del fire_group_rows[g][:-300]
                if args.fogo >= 0 and fire_basic(r) and counters["atualizacao"] - counters["fogo_desde"] >= 4:
                    fire_rows.append(r)
                # (15h30: so os retos sem a parada - nela a vigia do lento acaba no instante da parada e eles passavam facil.)
                if args.lento > 0 and r[1] == 6 and r[26] <= 0 and r[25] in (1, 2) and (len(r) <= 46 or r[46] < 0):
                    g = slow_group(r[2]) + (3 if args.lento_faixa and r[25] == 2 else 0)
                    if counters["atualizacao"] - counters["lento_desde"][g] >= 4:
                        slow_rows[g].append(r)
                        del slow_rows[g][:-300]
            del fire_rows[:-800]
        if len(episodes) and args.troca_pratica:
            troca_window.extend((int(r[2]), r[32] != 0) for r in episodes.tolist() if r[1] == 6 and 1 <= r[26] <= 3)
            del troca_window[:-TROCA_WINDOW]
            if counters["atualizacao"] % 10 == 0 and len(troca_window) >= 200:
                counters["troca_pesos"] = [round(x, 4) for x in troca_heading_weights(troca_window)]
                batch.set_troca_practice(True, counters["troca_pesos"])
        if len(episodes):
            window.extend(episodes.tolist())
            window = window[-3000:]
            if args.equilibrio and counters["atualizacao"] - counters["equilibrio_desde"] >= 4:
                balance_rows.extend(r for r in episodes.tolist() if r[1] == 2 and len(r) > 37 and r[37] > 0.5)
                balance_rows = balance_rows[-80:]
        counters["atualizacao"] += 1
        counters["amostras"] += T * n
        counters["episodios"] += len(episodes)
        if counters["atualizacao"] % 20 == 0:
            recent = np.array(window) if window else np.zeros((0, 27))
            row = {"atualizacao": counters["atualizacao"], "amostras": counters["amostras"], "hora": time.time(),
                   "tarefa": float(roll["task"].mean()), "estilo": last.get("estilo", float("nan")),
                   "tranco": float(roll["jerk"].mean()), "limite": float(roll["limit"].mean()),
                   "protegendo": float(roll["prot"].mean()), "locomovendo": float(roll["loco"].mean()),
                   # (12/11 16h45) A fracao das amostras na cabeca da locomocao (a habilidade, a ultima entrada) - a medida dos 80/20 dele;
                   # o "locomovendo" so conta a locomocao andando (sem o parado e o idle depois da parada).
                   "cabeca_locomocao": float((roll["obs"][..., -1] > .5).float().mean()),
                   "desvio": float(model.log_std.detach().clamp(LOG_STD_MIN, LOG_STD_MAX).exp().mean()),
                   **({"desvio_loco": float(model.loco_log_std.detach().clamp(LOG_STD_MIN, LOG_STD_MAX).exp().mean())} if model.split else {}),
                   "am_s": T * n / (time.time() - begin), "t_coleta": collected - begin, "t_fisica": physics,
                   "t_espera": waited, **{k: v for k, v in last.items() if k != "estilo"}}
            row.update(summarize(recent[recent[:, 1] == 0] if len(recent) else recent, "queda_"))
            row.update(summarize(recent[recent[:, 1] == 1] if len(recent) else recent, "clipe_"))
            # (07/11, ele: "os levantares nao estao mostrando os dados completos": o treino de costas e de brucos separado.)
            if args.levantar_deitado and len(recent):
                row.update(summarize(recent[(recent[:, 1] == 1) & (recent[:, 2] == 1)], "treino_costas_"))
                row.update(summarize(recent[(recent[:, 1] == 1) & (recent[:, 2] == 0)], "treino_brucos_"))
            balance = (recent[:, 37] > 0.5) if len(recent) and recent.shape[1] > 37 else np.zeros(len(recent), bool)
            row.update(summarize(recent[(recent[:, 1] == 2) & ~balance] if len(recent) else recent, "de_pe_"))
            if args.equilibrio:
                # (04/11 15h: o equilibrio parado a parte - o parado com empurroes, sem a mao - e o curriculo do empurrao: sobe
                # 0,05 com >= 95% ate o fim, desce com < 80%, no minimo 60 atualizacoes e 16 episodios por nivel.)
                row.update(summarize(recent[(recent[:, 1] == 2) & balance] if len(recent) else recent, "equilibrio_"))
                held = np.array(balance_rows)
                level, since = counters["equilibrio_nivel"], counters["equilibrio_desde"]
                if args.equilibrio_fatia > 0 and args.fogo < 0 and len(held) >= 16 and counters["atualizacao"] - since >= 60:
                    end = float((held[:, 32] == 0).mean())
                    target = float(np.clip(level + (0.05 if end >= 0.95 else -0.05 if end < 0.80 else 0.0), 0.1, 1.0))
                    if abs(target - level) > 1e-6:
                        counters["equilibrio_nivel"], counters["equilibrio_desde"] = target, counters["atualizacao"]
                        balance_rows = []
                        batch.set_balance_drill(args.equilibrio_fatia, target)
                        evaluations["equilibrio_"].set_balance_drill(0.0, target)
                        info = json.loads((run / "run.json").read_text())
                        info["v2"]["equilibrio_nivel"] = target
                        (run / "run.json").write_text(json.dumps(info, ensure_ascii=False))
                        print(f"equilibrio parado: nivel {level:.2f} -> {target:.2f} (ate o fim {end:.0%})", flush=True)
                row["equilibrio_nivel"] = counters["equilibrio_nivel"]
            if args.estrela > 0:
                # (08/11: a fase de cada exercicio da estrela - star_phase_step, com >= 24 episodios e 60 atualizacoes nela - e o
                # peso do sorteio pela falha, com piso.)
                changed = False
                for i, key in enumerate(STAR_KEYS):
                    # (A fase mede a estrela: so os episodios sem empurrao - o fogo tem a regua dele.)
                    held_star = np.array([r for r in star_rows[key] if r[38] <= 0]) if star_rows[key] else None
                    if held_star is None or len(held_star) < 24:
                        continue
                    perf = drill_performance(held_star, args.estrela_segundos)
                    counters["estrela_pesos"][i] = round(0.3 + (1.0 - float(perf["fim"])), 3)
                    phase = int(counters["estrela_fases"][i])
                    if counters["atualizacao"] - counters["estrela_desde"][key] < 60:
                        continue
                    target = star_phase_step(phase, float(perf["fim"]), float(held_star[:, 43].mean()))
                    if target != phase:
                        counters["estrela_fases"][i] = target
                        counters["estrela_desde"][key] = counters["atualizacao"]
                        star_rows[key] = []
                        changed = True
                        print(f"estrela: {key} fase {phase} -> {target} (ate o fim {perf['fim']:.0%}, "
                              f"{held_star[:, 43].mean():.1f} pernas)", flush=True)
                if changed or counters["atualizacao"] % 100 == 0:
                    batch.set_star(args.estrela, [float(x) for x in counters["estrela_pesos"]],
                                   [int(x) for x in counters["estrela_fases"]], args.estrela_segundos)
                if changed:
                    info = json.loads((run / "run.json").read_text())
                    info["v2"]["estrela_fases"] = counters["estrela_fases"]
                    (run / "run.json").write_text(json.dumps(info, ensure_ascii=False))
                row["estrela_fases"] = list(counters["estrela_fases"])
                row["estrela_pesos"] = list(counters["estrela_pesos"])
            if args.comando > 0:
                # (08/11 14h40: a fase do comando - sobe com >= 75% ate o fim e >= 6 eventos por episodio (os sem empurrao), desce
                # com < 40%; >= 24 episodios e 60 atualizacoes na fase.)
                held_cmd = np.array(command_rows) if command_rows else np.zeros((0, 46))
                phase = int(counters["comando_fase"])
                if len(held_cmd) >= 24 and counters["atualizacao"] - counters["comando_desde"] >= 60:
                    end = float((held_cmd[:, 32] == 0).mean())
                    events = float(held_cmd[:, 43].mean())
                    target = phase + (1 if end >= 0.75 and events >= 6 and phase < 4 else -1 if end < 0.40 and phase > 1 else 0)
                    if target != phase:
                        counters["comando_fase"], counters["comando_desde"] = target, counters["atualizacao"]
                        command_rows = []
                        batch.set_command(args.comando, target, args.comando_segundos)
                        info = json.loads((run / "run.json").read_text())
                        info["v2"]["comando_fase"] = target
                        (run / "run.json").write_text(json.dumps(info, ensure_ascii=False))
                        print(f"comando: fase {phase} -> {target} (ate o fim {end:.0%}, {events:.1f} eventos)", flush=True)
                row["comando"] = {"fase": int(counters["comando_fase"]), "n": int(len(held_cmd)),
                                  "ate_o_fim": round(float((held_cmd[:, 32] == 0).mean()), 4) if len(held_cmd) else None,
                                  "eventos": round(float(held_cmd[:, 43].mean()), 2) if len(held_cmd) else None}
            if args.lento > 0:
                # (08/11 11h40: o piso da velocidade nos retos - sobe com >= 90% sem o fim por lento; 12h25: um por grupo de
                # rumo, a frente, os lados e as costas.)
                row["lento"] = {}
                for g, name in enumerate(SLOW_GROUPS if not args.lento_faixa
                                         else tuple(f"{n}_{b}" for b in ("trote", "corrida") for n in SLOW_GROUPS)):
                    held_slow = np.array(slow_rows[g]) if slow_rows[g] else np.zeros((0, 46))
                    passed = float((held_slow[:, 32] != 7).mean()) if len(held_slow) else float("nan")
                    floor = counters["lento"][g]
                    if len(held_slow) >= 64 and counters["atualizacao"] - counters["lento_desde"][g] >= 60:
                        target = slow_step(floor, passed, args.lento_minimo[g % 3])
                        if abs(target - floor) > 1e-6:
                            counters["lento"][g], counters["lento_desde"][g] = target, counters["atualizacao"]
                            slow_rows[g] = []
                            batch.set_slow_limits([float(x) for x in counters["lento"]])
                            print(f"lento: {name} piso {floor:.2f} -> {target:.2f} (passaram {passed:.0%})", flush=True)
                    row["lento"][name] = {"piso": round(counters["lento"][g], 3),
                                          "passaram": round(passed, 4) if len(held_slow) else None, "n": int(len(held_slow))}
            if args.fogo >= 0:
                # (08/11, a fase de fogo: o nivel - fire_step, com >= 96 episodios com empurrao e 60 atualizacoes nele.)
                held_fire = np.array(fire_rows) if fire_rows else np.zeros((0, 46))
                level = counters["fogo_nivel"]
                ratio, with_push, without, n_push, n_plain = fire_ratio(held_fire)
                moving = held_fire[np.isin(held_fire[:, 1], (6, 7))] if len(held_fire) else held_fire
                still = held_fire[~np.isin(held_fire[:, 1], (6, 7))] if len(held_fire) else held_fire
                kept = ratio
                # (12h30: o nivel de cada grupo - fire_step na razao do grupo, >= 48 com e sem empurrao, 60 atualizacoes.)
                changed = []
                for g, name in enumerate(FIRE_GROUPS):
                    rows_g = np.array(fire_group_rows[g]) if fire_group_rows[g] else np.zeros((0, 46))
                    r_g, with_g, _, n_with, n_without = fire_ratio(rows_g)
                    if n_with >= 48 and n_without >= 48 and counters["atualizacao"] - counters["fogo_niveis_desde"][g] >= 60:
                        before_g = counters["fogo_niveis"][g]
                        target_g = fire_step_target(before_g, with_g, args.fogo_alvo) if args.fogo_alvo > 0 else fire_step(before_g, r_g)
                        if abs(target_g - before_g) > 1e-6:
                            counters["fogo_niveis"][g] = round(target_g, 3)
                            counters["fogo_niveis_desde"][g] = counters["atualizacao"]
                            fire_group_rows[g] = []
                            changed.append(f"{name} {before_g:.1f}->{target_g:.1f} (razao {r_g:.2f}, com empurrao {with_g:.0%})")
                if changed:
                    batch.set_fire_groups(fire_group_bands(counters["fogo_niveis"]))
                    info = json.loads((run / "run.json").read_text())
                    info["v2"]["fogo_niveis"] = dict(zip(FIRE_GROUPS, counters["fogo_niveis"]))
                    info["v2"]["fogo_nivel"] = counters["fogo_niveis"][0]
                    (run / "run.json").write_text(json.dumps(info, ensure_ascii=False))
                    print("fogo por grupo: " + ", ".join(changed), flush=True)
                hit, hold = fire_bands(counters["fogo_nivel"])
                mov_ratio = fire_ratio(moving)
                still_ratio = fire_ratio(still)
                finite = lambda x: round(float(x), 4) if np.isfinite(x) else None  # noqa: E731
                row["fogo"] = {"nivel": round(counters["fogo_nivel"], 3), "razao": finite(ratio), "com_empurrao": finite(with_push),
                               "sem_empurrao": finite(without), "n_com": n_push, "n_sem": n_plain,
                               "pancada_n": list(hit), "empurrao_n": list(hold),
                               "mov_razao": finite(mov_ratio[0]), "mov_com": finite(mov_ratio[1]), "mov_sem": finite(mov_ratio[2]),
                               "pe_razao": finite(still_ratio[0]), "pe_com": finite(still_ratio[1]), "pe_sem": finite(still_ratio[2]),
                               "caiu_no_empurrao": finite(held_fire[held_fire[:, 38] > 0, 45].mean()) if n_push else None}
                if len(held_fire):
                    # (08/11 11h10: o fogo por grupo - onde o empurrao derruba: (com, sem) o sucesso.)
                    heading = held_fire[:, 2] % 8
                    groups = {"frente": (held_fire[:, 1] == 6) & np.isin(heading, (0, 1, 7)),
                              "lado": (held_fire[:, 1] == 6) & np.isin(heading, (2, 6)),
                              "costas": (held_fire[:, 1] == 6) & np.isin(heading, (3, 4, 5)),
                              "giro": held_fire[:, 1] == 7, "parado": held_fire[:, 1] == 2,
                              "levantar": np.isin(held_fire[:, 1], (0, 1))}
                    row["fogo"]["grupos"] = {g: [finite(x) for x in fire_ratio(held_fire[m])[1:3]] for g, m in groups.items() if m.any()}
                row["fogo"]["niveis"] = dict(zip(FIRE_GROUPS, counters["fogo_niveis"]))
            row.update(summarize(recent[recent[:, 1] == 3] if len(recent) else recent, "mov_"))
            row.update(summarize(recent[recent[:, 1] == 4] if len(recent) else recent, "evitar_"))
            row.update(summarize(recent[recent[:, 1] == 5] if len(recent) else recent, "arrancada_"))
            row.update(summarize(recent[recent[:, 1] == 6] if len(recent) else recent, "loco_"))
            row.update(summarize(recent[recent[:, 1] == 7] if len(recent) else recent, "giro_"))
            row.update(summarize(recent[recent[:, 1] == 8] if len(recent) else recent, "quedas_loco_"))
            row["cenarios"] = summarize_scenery(recent)
            if args.levantar_unreal > 0 and len(recent):
                for exercise, prefix in enumerate(LEVANTAR_UNREAL_PREFIXES):
                    row.update(clip_get_up_summary(recent[(recent[:, 1] == 9) & (recent[:, 26] == exercise)], "treino_" + prefix))
                if args.levantar_e_ir[0] > 0 and recent.shape[1] >= 76:
                    row.update(get_up_go_summary(recent[recent[:, 1] == 11], "treino_" + LEVANTAR_E_IR_PREFIX))
                # (12/11 07h45, temporario: as linhas dos episodios do levantar da janela, para comparar com as sondas - os correndo
                # recuperavam 0% no treino e 40-60% nas sondas.)
                np.save(run / "exports" / "episodios_levantar.npy", recent[recent[:, 1] == 9])
                if args.levantar_unreal_prioridade > 0:
                    # (16h25, a vigia: o de brucos ja entregava 99% e os de costas travados ha 40 min com 25% do treino cada - o peso de
                    # cada exercicio e o piso mais a fracao que nao entrega.)
                    weights = []
                    for exercise in range(len(LEVANTAR_UNREAL)):
                        mine = recent[(recent[:, 1] == 9) & (recent[:, 26] == exercise)]
                        kept = float(((mine[:, 18] >= 0) & (mine[:, 19] < .5)).mean()) if len(mine) >= 20 else 0.0
                        weights.append(args.levantar_unreal_prioridade + 1.0 - kept)
                    batch.set_clip_get_up_weights(weights)
                    total = sum(weights)
                    row["levantar_pesos"] = {k: round(w / total, 3) for k, w in zip(LEVANTAR_UNREAL_KEYS, weights)}
                if args.levantar_unreal_reforco[0] > 0 and hasattr(batch, "set_clip_get_up_emphasis"):
                    # (12/11 02h50) A rampa do reforco do de costas: de 0 ao alvo em N atualizacoes desde a primeira com ele.
                    start = counters.setdefault("reforco_costas_inicio", counters["atualizacao"])
                    ramp = max(1.0, float(args.levantar_unreal_reforco[1]))
                    emphasis = args.levantar_unreal_reforco[0] * min(1.0, (counters["atualizacao"] - start) / ramp)
                    per_exercise = [emphasis] + [0.0] * (len(LEVANTAR_UNREAL) - 1)
                    # (14/11 20h45) O mesmo reforco nos correndo, com a rampa propria.
                    if args.levantar_unreal_reforco_correndo > 0:
                        start_run = counters.setdefault("reforco_correndo_inicio", counters["atualizacao"])
                        run_emphasis = args.levantar_unreal_reforco_correndo * min(1.0, (counters["atualizacao"] - start_run) / ramp)
                        # (Nao "run": e o caminho da pasta do treino neste escopo - 20h41 derrubou o treino no write_status.)
                        for k, running in enumerate(LEVANTAR_UNREAL_RUN):
                            if running:
                                per_exercise[k] = run_emphasis
                        row["reforco_correndo"] = round(run_emphasis, 3)
                    if args.levantar_unreal_reforco_brucos > 0:
                        start_prone = counters.setdefault("reforco_brucos_inicio", counters["atualizacao"])
                        prone_emphasis = args.levantar_unreal_reforco_brucos * min(1.0, (counters["atualizacao"] - start_prone) / ramp)
                        per_exercise[1] = prone_emphasis
                        row["reforco_brucos"] = round(prone_emphasis, 3)
                    batch.set_clip_get_up_emphasis(per_exercise)
                    row["reforco_costas"] = round(emphasis, 3)
                fire = get_up_fire_state(args, counters)
                if fire is not None:
                    # (12/11 04h40) O degrau do fogo do levantar (get_up_fire_step) e os niveis de agora no lote.
                    # (12/11 12h55) As duas primeiras janelas depois de comecar so tem os episodios curtos (os que deram certo): a escada
                    # lia o levantou alto demais a cada religar e subia a forca a mais.
                    FIRE_WINDOWS[0] += 1
                    for change in (get_up_fire_step(fire, recent, counters["atualizacao"], args.levantar_fogo_faixa, args.levantar_fogo_congelar,
                                                    tracked=int(args.levantar_rastrear or 0))
                                   if FIRE_WINDOWS[0] > 2 else []):
                        print("fogo do levantar:", change, flush=True)
                    batch.set_get_up_fire(list(args.levantar_fogo) + [args.levantar_fogo_de_cima], fire_levels_flat(fire["niveis"]), False)
                    row["fogo_levantar"] = fire["niveis"]
                if max(args.levantar_unreal_agilidade[:2]) > 0 and hasattr(batch, "set_clip_get_up_agility"):
                    # (12/11 03h20) A rampa da agilidade (clip_get_up_agility).
                    agility = clip_get_up_agility(args, counters)
                    batch.set_clip_get_up_agility(agility)
                    row["agilidade"] = [round(a, 3) for a in agility]
            if args.parado_torto[0] > 0 and len(recent) and recent.shape[1] >= 66:
                # (12/11 17h) O parado torto contra o normal: n, nao caiu, reajustou e em quanto tempo.
                still = recent[recent[:, 1] == 2]
                out = {}
                for name, m in (("torto", still[:, 64] > 0), ("normal", still[:, 64] <= 0)):
                    rows = still[m]
                    if not len(rows):
                        continue
                    fixed = rows[:, 65] >= 0
                    out[name] = {"n": int(len(rows)), "sem_cair": round(float(loco_fire_ok(rows).mean()), 3)}
                    if name == "torto":
                        out[name].update(reajustou=round(float(fixed.mean()), 3),
                                         reajuste_s=round(float(rows[fixed, 65].mean()), 3) if fixed.any() else None)
                row["parado_torto"] = out
            if (args.estrela_caminho[0] > 0 or args.estrela_curvas[0] > 0) and len(recent) and recent.shape[1] >= 68:
                # (12/11 17h20) A estrela por exercicio: n, ate o fim sem cair, pernas, curvas e a distancia media ao caminho (m).
                stars = recent[(recent[:, 1] == 6) & (recent[:, 26] >= 8) & (recent[:, 26] <= 10)]
                out = {}
                for band, bname in ((1, "trote"), (2, "corrida")):
                    for m, mname in ((8, "fixo"), (9, "livre"), (10, "girando")):
                        rows = stars[((stars[:, 25] >= 2) == (band == 2)) & (stars[:, 26] == m)]
                        if not len(rows):
                            continue
                        dist = rows[rows[:, 66] >= 0, 66]
                        out[f"{bname}_{mname}"] = {"n": int(len(rows)), "sem_cair": round(float(loco_fire_ok(rows).mean()), 3),
                                                   "pernas": round(float(rows[:, 43].mean()), 2), "curvas": round(float(rows[:, 67].mean()), 2),
                                                   "dist_m": round(float(dist.mean()), 2) if len(dist) else None}
                row["estrela"] = out
            if args.comando_levantar and len(recent) and recent.shape[1] >= 71:
                # (12/11 18h) O comando ligado ao levantar: n, os sem queda, as quedas por episodio, a fracao das quedas que levantou de
                # volta e o tempo medio do levantar (s).
                # (13/11, --queda-vira-comando) As quedas dos outros exercicios que viraram comando: n, levantou de volta e o tempo.
                turned = recent[np.isin(recent[:, 1], (6, 7)) & ~((recent[:, 1] == 6) & (recent[:, 26] == 12)) & (recent[:, 68] > 0)]
                if args.queda_vira_comando:
                    loco_eps = int(np.isin(recent[:, 1], (6, 7)).sum() - ((recent[:, 1] == 6) & (recent[:, 26] == 12)).sum())
                    t_falls, t_rises = float(turned[:, 68].sum()), float(turned[:, 69].sum())
                    t_timed = turned[turned[:, 70] >= 0, 70]
                    row["queda_vira_comando"] = {"episodios": loco_eps, "com_queda": int(len(turned)),
                                                 "levantou": round(t_rises / t_falls, 3) if t_falls > 0 else None,
                                                 "tempo_s": round(float(t_timed.mean()), 2) if len(t_timed) else None,
                                                 "segundos": round(float(turned[:, 4].mean()), 2) if len(turned) else None}
                cmd = recent[(recent[:, 1] == 6) & (recent[:, 26] == 12)]
                if len(cmd):
                    falls, rises = float(cmd[:, 68].sum()), float(cmd[:, 69].sum())
                    timed = cmd[cmd[:, 70] >= 0, 70]
                    runs = float(cmd[:, 71].sum()) if cmd.shape[1] > 71 else 0.0
                    row["comando_levantar"] = {"n": int(len(cmd)), "sem_queda": round(float((cmd[:, 68] <= 0).mean()), 3),
                                               "levantou_correndo": int(runs),
                                               "quedas_ep": round(falls / len(cmd), 3),
                                               "levantou": round(rises / falls, 3) if falls > 0 else None,
                                               "tempo_s": round(float(timed.mean()), 2) if len(timed) else None}
            loco_fire = loco_fire_state(args, counters)
            if loco_fire is not None and len(recent) and hasattr(batch, "set_locomotion_fire"):
                # (12/11 16h30) O degrau do fogo da locomocao (loco_fire_step; as duas primeiras janelas depois de religar puladas, como no
                # levantar) e os niveis de agora no lote.
                LOCO_FIRE_WINDOWS[0] += 1
                speed_ok, slow = loco_fire_clean_ok(counters.get("velocidade_limpo"), counters.get("limpo_base_capacidade"),
                                                    args.locomocao_fogo_guarda_velocidade)
                for change in (loco_fire_step(loco_fire, recent, counters["atualizacao"], args.locomocao_fogo_congelar, speed_ok)
                               if LOCO_FIRE_WINDOWS[0] > 2 else []):
                    print("fogo da locomocao:", change, flush=True)
                batch.set_locomotion_fire(counters.get("fogo_protecao", {}).get("fatia", args.locomocao_fogo), loco_fire["niveis"],
                                          args.locomocao_fogo_liberdade)
                row["fogo_locomocao"] = {"fatia": counters.get("fogo_protecao", {}).get("fatia", args.locomocao_fogo),"niveis": dict(zip(FIRE_GROUPS, loco_fire["niveis"])), "grupos": loco_fire_summary(recent),
                                         "velocidade_limpo": counters.get("velocidade_limpo"), "guarda_velocidade": slow or None}
            if "ajuda_exercicios" in counters:
                # (O portao, exercicio por exercicio, a cada linha do log: so mede - a forca fica em zero.)
                levels, states, dominio = counters["ajuda_exercicios"], counters["ajuda_estado"], {}
                for key in DRILL_KEYS:
                    held = np.array(drill_rows[key][-60:]) if drill_rows[key] else np.zeros((0, 42))
                    _, perf = dynamic_assist(held, float(levels[key]), states[key], args.locomocao_segundos,
                                                  ceiling=(1.0 if key in ("lev_parado", "lev_levantar") else ceilings.get(key)), key=key,
                                                  speed_ceiling=speed_ceilings.get(key),
                                                  stability_ceiling=stability_ceilings.get(key))
                    if perf is not None:
                        dominio[key] = {k: round(v, 3) for k, v in perf.items()}
                    drill_rows[key] = drill_rows[key][-60:]
                exigencias = {k: drill_requirements(float(levels[k]), k, 1.0 if k in ("lev_parado", "lev_levantar") else ceilings.get(k),
                                                    speed_ceilings.get(k), stability_ceilings.get(k))
                              for k in DRILL_KEYS}
                # (05/11 20h35: a linha da semelhanca exigida nos graficos ja com a compensacao da estabilidade - a sobra dela
                # sobre a exigida baixa a exigida da semelhanca; a sem compensar fica em semelhanca_base.)
                for k, need in exigencias.items():
                    d = dominio.get(k)
                    if d and "semelhanca" in need and "estabilidade" in d and "estab_exigida" in d:
                        need["semelhanca_base"] = need["semelhanca"]
                        need["semelhanca"] = round(need["semelhanca"] - max(0.0, d["estabilidade"] - d["estab_exigida"]), 3)
                row.update(ajuda_exercicios={k: round(float(levels[k]), 4) for k in DRILL_KEYS}, dominio=dominio,
                           exigencias=exigencias)
                if "troca_pesos" in counters:
                    row["troca_pesos"] = counters["troca_pesos"]
            log.write(json.dumps(finite_json(row)) + "\n")
            log.flush()
            print(" ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in row.items()), flush=True)
            # (As quedas: os episodios da janela que cairam depois de ja ter ficado de pe.)
            last_status["sps"] = row["am_s"]
            last_status["last"] = {"fall_fraction": float(recent[:, 13].mean()) if len(recent) else -1.0,
                                   "reward": row["tarefa"], "style": row["estilo"]}
            write_status("treinando")
        if counters["atualizacao"] % 10 == 0:
            export_policy()
            load_falls()
            load_partner()
        judge = counters["atualizacao"] % args.salvar_cada == 0
        if stop["salvar"] or judge:
            stop["salvar"] = False
            save()
        # (As copias dos juizes para a avaliacao, antes da atualizacao mexer neles na outra thread.)
        eval_judges = None
        if judge and args.juiz_unreal:
            eval_judges = (copy.deepcopy(disc).eval(),
                           copy.deepcopy(disc_loco).eval() if isinstance(disc_loco, Discriminator) else None)
        pending = executor.submit(update, roll)
        if judge:
            # (A avaliacao com a copia da coleta - a rede que acabou de ser salva -, enquanto a atualizacao roda.)
            judged = time.time()
            row = {"atualizacao": counters["atualizacao"], "amostras": counters["amostras"], "aval": True,
                   "hora": time.time(),
                   **evaluate(collector, evaluation, device, judges=eval_judges)}
            for prefix, other in evaluations.items():
                row.update(evaluate(collector, other, device, prefix, judges=eval_judges))
            # (Por exercicio, a nota do juiz de todas as avaliacoes juntas - o laboratorio le daqui.)
            merged: dict[str, list] = {}
            for key in [k for k in row if k.endswith("juiz_exercicios")]:
                for drill, value in row.pop(key).items():
                    merged.setdefault(drill, []).append(value)
            if merged:
                row["juiz_exercicios"] = {k: round(float(np.mean(v)), 4) for k, v in merged.items()}
            if capacity_batches and time.time() - capacity_at[0] >= 60.0 * args.capacidade_minutos:
                capacity_at[0] = time.time()
                row["capacidade"] = capacity(collector)
                clean = loco_fire_clean(row["capacidade"])
                if clean:
                    counters["velocidade_limpo"] = {k: round(v, 3) for k, v in clean.items() if v is not None}
                    if clean.get("torto") is not None:
                        hist = counters.setdefault("torto_historico", [])
                        hist.append(round(clean["torto"], 3))
                        del hist[:-3]
                        counters["velocidade_limpo"]["torto"] = round(float(np.mean(hist)), 3)
                    base = counters.setdefault("limpo_base_capacidade", {})
                    if any(k not in base for k in counters["velocidade_limpo"]):
                        for k, v in counters["velocidade_limpo"].items():
                            base.setdefault(k, v)
                        print(f"fogo da locomocao: a base do limpo na capacidade {base}", flush=True)
                    if args.locomocao_fogo > 0:
                        ok, why = loco_fire_clean_ok(counters["velocidade_limpo"], counters["limpo_base_capacidade"],
                                                     args.locomocao_fogo_guarda_velocidade)
                        prot = counters.setdefault("fogo_protecao", {"fatia": args.locomocao_fogo})
                        # (20h50: a medida logo depois de religar mede os mesmos pesos da ultima - nao conta de novo na protecao.)
                        change = loco_fire_protect(prot, ok, args.locomocao_fogo) if capacity_counted[0] else ""
                        capacity_counted[0] = True
                        if change:
                            print(f"fogo da locomocao: {change} - {why or 'limpo bom'}", flush=True)
                        row["fogo_protecao"] = {**prot, "guarda": why or None}
            row["t_aval"] = time.time() - judged
            log.write(json.dumps(finite_json(row)) + "\n")
            log.flush()
            print("aval " + " ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}" for k, v in row.items()),
                  flush=True)
    if pending is not None:
        pending.result()
    executor.shutdown()
    if prova is not None:
        prova.rede(counters["atualizacao"], model, disc, disc_loco)
        prova.gravar()
    save()
    write_status("parado")
    print(f"salvo na atualizacao {counters['atualizacao']}" + (" (sinal de parar)" if stop["pedido"] else ""), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
