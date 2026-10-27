#!/usr/bin/env python3
"""Motor autônomo v4 (contrato motor-v4): PPO sobre ações latentes; o ambiente aplica o
tanh fixo e o zero da ação é o IDLE alerta. A razão PPO usa o mesmo latente coletado.

O professor tem três muletas, cada uma com botão e saída:
- o zero da ação no IDLE (dado estático do corpo, permanente);
- o prior de ação: uma perda no ator que puxa a média para a pose de referência na fase,
  nos estados que a própria aluna visita (`teacher` na receita: peso e agenda até zero);
- a imitação na nota (`imitation.weight`, β, na receita).
Nenhuma força externa e nenhuma ação vinda do clipe no atuador. O ruído de exploração é
fixo por grupo de juntas, em radianos, sem bônus de entropia; o ruído correlacionado no
tempo (`--ruido-correlacao`) é uma opção para A/B, desligada por padrão.
"""
from __future__ import annotations

import argparse
import collections
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import random
import signal
import shutil
import sys
import time

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, os.environ.get('MATTER_MOTOR_NATIVE', str(ROOT / 'build-rl/python')))
from matter_rl.model import ActorCritic, export_policy  # noqa: E402
from matter_rl.control import ControlServer  # noqa: E402
import matter_motor_native as native  # noqa: E402

CONTRACT = 'motor-v4'
TASK_ID = 'motor_basico'
MODES = {'maximo': 12, 'equilibrado': 8, 'leve': 4}
METRICS = native.episode_metric_names()
COL = {name: i + 2 for i, name in enumerate(METRICS)}
# Os episódios que nasceram no movimento do clipe (só no treino) são mais fáceis: ficam fora do currículo e
# das medidas principais do log. O rumo do giro é o do corpo inteiro quando a receita pede (erro_rumo).
INMOTION = COL.get('comeco_em_movimento')
HEADING = 'erro_rumo' if 'erro_rumo' in COL else 'erro_peito'
# Ruído fixo por grupo, em radianos de alvo articular perto do IDLE (os valores medidos no corpo v2).
NOISE_RADIANS = {'pernas': .06, 'bracos': .05, 'tronco': .02}
PRIOR_RADIANS = .1  # a escala do erro do prior: 0,1 rad além da faixa livre vale 1
CAUSES = {1: 'contato', 2: 'pelve_baixa', 3: 'tronco', 4: 'numerico'}


def atomic_json(path, obj):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(obj, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    tmp.replace(path)


def noise_group(action_name):
    link = action_name.split('.')[0]
    if any(k in link for k in ('Thigh', 'Shin', 'Foot')):
        return 'pernas'
    if any(k in link for k in ('UpperArm', 'Forearm', 'Hand')):
        return 'bracos'
    return 'tronco'


def fixed_log_std(batch, scale):
    derivative = np.asarray(batch.action_derivatives(), np.float64)
    radians = np.asarray([NOISE_RADIANS[noise_group(n)] for n in batch.action_names()]) * scale
    return torch.as_tensor(np.log(radians / derivative), dtype=torch.float32)


def next_noise(eps, rho):
    """O ruído da próxima decisão: branco (rho 0) ou AR(1), com a mesma variância marginal 1."""
    fresh = torch.randn_like(eps)
    return rho * eps + math.sqrt(1 - rho * rho) * fresh if rho > 0 else fresh


def advantages(rewards, values, done, last, gamma=.99, lam=.95):
    result = torch.zeros_like(rewards)
    carry = torch.zeros_like(last)
    for t in reversed(range(len(rewards))):
        next_value = last if t == len(rewards) - 1 else values[t + 1]
        mask = 1 - done[t]
        delta = rewards[t] + gamma * next_value * mask - values[t]
        carry = delta + gamma * lam * mask * carry
        result[t] = carry
    return result


def make_model(batch, device, noise_scale):
    model = ActorCritic(batch.obs_size, batch.obs_size, batch.action_size, None,
                        actor_hidden=(256, 256), critic_hidden=(256, 256)).to(device)
    # Escalas físicas do contrato; identidade fixa evita mudar a política ao publicar estatísticas.
    model.obs_norm.mean.zero_(); model.obs_norm.var.fill_(1)
    model.critic_norm.mean.zero_(); model.critic_norm.var.fill_(1)
    with torch.no_grad():
        model.log_std.copy_(fixed_log_std(batch, noise_scale).to(device))
    model.log_std.requires_grad_(False)
    return model


def teacher_settings(recipe):
    t = recipe.get('teacher', {})
    weight, full, zero = float(t.get('priorWeight', 0)), float(t.get('fullUntilDecisions', 0)), float(t.get('zeroAtDecisions', 0))
    free = float(t.get('freeRadians', 0))
    if not all(math.isfinite(x) for x in (weight, full, zero, free)) or not 0 <= weight <= 10 or full < 0 or zero < full or not 0 <= free <= .5:
        raise ValueError('professor inválido na receita (priorWeight em [0,10], 0 <= fullUntil <= zeroAt, freeRadians em [0; 0,5])')
    # Com o clipe nos motores a ação da rede já é a correção sobre ele: o prior puxaria para o clipe outra vez.
    if float(recipe.get('clipDrive', 0)) > 0 and (weight > 0 or any(float(v) > 0 for v in t.get('permanent', {}).values())):
        raise ValueError('clipDrive > 0 com professor (priorWeight ou permanent): o clipe entraria duas vezes')
    return weight, full, zero, free


def prior_loss(mean, reference, mapping, free, weights=None):
    """O prior em radianos de alvo articular, pela transformação real do atuador (raio·tanh(latente + viés)):
    dentro da faixa livre a rede corrige o equilíbrio sem custo; fora dela a perda puxa a forma para a do clipe.
    `weights`, se dado, é o peso por eixo (a agenda mais o professor permanente por grupo)."""
    radius, bias = mapping
    difference = radius * (torch.tanh(mean + bias) - torch.tanh(reference + bias))
    excess = (difference.abs() - free).clamp_min(0)
    square = (excess / PRIOR_RADIANS).square()
    return square.mean() if weights is None else (square * weights).mean()


def permanent_prior(recipe, action_names):
    """O professor permanente, fora da agenda, por grupo de juntas (teacher.permanent = {"bracos": 0.1}): o peso por eixo."""
    groups = recipe.get('teacher', {}).get('permanent', {})
    if not all(g in NOISE_RADIANS and math.isfinite(float(w)) and 0 <= float(w) <= 10 for g, w in groups.items()):
        raise ValueError('professor permanente inválido na receita (grupos pernas/bracos/tronco, peso em [0,10])')
    return np.asarray([float(groups.get(noise_group(n), 0.)) for n in action_names], np.float32)


def prior_weight(recipe, decisions):
    weight, full, zero, _ = teacher_settings(recipe)
    if weight <= 0 or decisions >= zero:
        return 0.
    if decisions <= full:
        return weight
    return weight * (zero - decisions) / (zero - full)


class Curriculum:
    """Duas escadas. A mistura: na fase base, mais parado e giro, até os dois ficarem de pé com boa
    postura; depois as cotas finais da receita. O trote: o teto de velocidade de cada rumo sobe um
    degrau quando os últimos episódios perto do teto (alvo >= 70% dele) caem pouco, rastreiam e
    mantêm a postura."""

    def __init__(self, recipe, state=None):
        self.final_shares = [float(recipe['shares'][k]) for k in ('parado', 'giro', 'trote')]
        base = recipe['stages']['base']
        self.base_shares = [float(base['shares'][k]) for k in ('parado', 'giro', 'trote')]
        self.base_window, self.base_falls = int(base['window']), float(base['maximumFalls'])
        self.base_turn_falls, self.base_posture = float(base['turnMaximumFalls']), float(base['minimumPosture'])
        self.base_turn_error = math.radians(float(base['turnMaximumChestErrorDegrees']))
        if abs(sum(self.base_shares) - 1) > 1e-4 or min(self.base_shares) < 0 or self.base_window <= 0:
            raise ValueError('fase base inválida na receita')
        self.stage = int(state.get('stage', 0)) if state else 0
        self.base_recent = [collections.deque(maxlen=self.base_window) for _ in range(2)]
        trot = recipe['trot']
        c = trot['curriculum']
        self.target = float(trot['targetSpeed'])
        self.step = float(c['step'])
        self.window = int(c['window'])
        self.max_falls, self.min_tracking, self.min_posture = float(c['maximumFalls']), float(c['minimumTracking']), float(c['minimumPosture'])
        # O rastreio só mede a projeção no rumo: andar de lado ou com o peito virado também promovia. A receita
        # pode exigir o progresso vetorial e o rumo do peito (ausentes: sem exigência, como antes).
        self.min_progress = float(c.get('minimumProgress', 0.))
        self.max_heading = math.radians(float(c.get('maximumChestErrorDegrees', 180.)))
        if not (self.step > 0 and self.window > 0 and 0 <= self.max_falls <= 1 and self.min_tracking > 0 and 0 <= self.min_posture <= 1
                and 0 <= self.min_progress <= 1 and self.max_heading > 0):
            raise ValueError('currículo de trote inválido na receita')
        self.limits = [float(x) for x in state['limits']] if state else [float(c['start'])] * 8
        if len(self.limits) != 8:
            raise ValueError('currículo salvo inválido')
        self.recent = [collections.deque(maxlen=self.window) for _ in range(8)]

    def shares(self):
        return self.final_shares if self.stage else self.base_shares

    def observe(self, rows):
        """Devolve (rumos de trote promovidos, se a mistura passou para a fase final agora)."""
        promoted = []
        for row in rows:
            if INMOTION is not None and row[INMOTION] > .5:
                continue
            if self.stage == 0 and int(row[1]) < 2:
                self.base_recent[int(row[1])].append((row[COL['caiu']], row[COL['postura']], row[COL[HEADING]]))
            d = int(row[1]) - 2
            if d < 0 or row[COL['velocidade_alvo']] < .7 * self.limits[d]:
                continue
            self.recent[d].append((row[COL['caiu']], row[COL['rastreio']], row[COL['postura']], row[COL['progresso']], row[COL[HEADING]]))
        for d in range(8):
            q = self.recent[d]
            if len(q) < self.window or self.limits[d] >= self.target:
                continue
            falls, tracking, posture, progress, heading = np.mean(np.asarray(q), axis=0)
            if falls <= self.max_falls and tracking >= self.min_tracking and posture >= self.min_posture \
                    and progress >= self.min_progress and heading <= self.max_heading:
                self.limits[d] = min(self.target, round(self.limits[d] + self.step, 4))
                q.clear()
                promoted.append(d)
        staged = False
        if self.stage == 0 and all(len(q) >= self.base_window for q in self.base_recent):
            (stand_falls, stand_posture, _), (turn_falls, turn_posture, turn_error) = (np.mean(np.asarray(q), axis=0) for q in self.base_recent)
            # O giro só passa se girar de verdade: erro médio do peito ao alvo abaixo do limite.
            if stand_falls <= self.base_falls and turn_falls <= self.base_turn_falls and min(stand_posture, turn_posture) >= self.base_posture \
                    and turn_error <= self.base_turn_error:
                self.stage, staged = 1, True
        return promoted, staged

    def state(self):
        return {'limits': self.limits, 'stage': self.stage}


def warm_start(model, batch, device, std, derivative, iterations=4, steps=96, updates=400, log=print):
    """O ensino inicial (estilo DAgger): a aluna age com a própria média + o ruído fixo, cada estado
    visitado ganha o rótulo da pose de referência na fase, e o ator aprende por regressão a comandar
    essa pose. Repete nos estados da nova aluna. O PPO começa com a média já perto do clipe e o
    prior (λ pequeno) só segura isso enquanto a agenda o retira."""
    optimizer = torch.optim.Adam(model.actor.parameters(), lr=1e-3)
    observations, labels = [], []

    def loss_of(o, y):
        return ((derivative * (model.actor(model.obs_norm(o)) - y)) / PRIOR_RADIANS).square().mean()

    history = []
    for iteration in range(iterations):
        with torch.no_grad():
            for _ in range(steps):
                o = torch.as_tensor(batch.observations(), device=device)
                observations.append(o)
                labels.append(torch.as_tensor(batch.reference_actions(), device=device))
                action = model.actor(model.obs_norm(o)) + std * torch.randn(o.shape[0], std.shape[0], device=device)
                batch.step(action.cpu().numpy())
                batch.episodes()
            x, y = torch.cat(observations), torch.cat(labels)
            before = float(loss_of(x[:8192], y[:8192]))
        for _ in range(updates):
            idx = torch.randint(0, len(x), (4096,), device=device)
            loss = loss_of(x[idx], y[idx])
            optimizer.zero_grad(); loss.backward(); torch.nn.utils.clip_grad_norm_(model.actor.parameters(), 1); optimizer.step()
        with torch.no_grad():
            after = float(loss_of(x[-8192:], y[-8192:]))
        history.append({'iteracao': iteration + 1, 'amostras': int(len(x)), 'perda_antes': before, 'perda_depois': after})
        log(f'ensino inicial {iteration + 1}/{iterations}: {len(x)} estados, perda do prior {before:.3f} -> {after:.3f}')
    return history


def drill_summary(e):
    """O que o log e a tela mostram de um exercício numa rodada (linhas de episódio)."""
    m = lambda k: float(e[:, COL[k]].mean())  # noqa: E731
    seconds = e[:, COL['segundos']]
    return {'n': int(len(e)), 'sem_cair': 1 - m('caiu'), 'ep_segundos': m('segundos'),
            'nota': float((e[:, COL['retorno']] / np.maximum(seconds, 1e-6)).mean()),
            'erro_vel_cm': m('erro_velocidade'), 'vel_corpo': m('velocidade'), 'vel_pedida': m('velocidade_pedida'),
            'vel_alvo': m('velocidade_alvo'), 'rastreio': m('rastreio'), 'teto': m('teto_trote'), 'progresso': m('progresso'),
            'deriva': m('deriva'), 'erro_peito_graus': math.degrees(m('erro_peito')), 'erro_cabeca_graus': math.degrees(m('erro_cabeca')),
            'erro_rumo_graus': math.degrees(m(HEADING)),
            'postura': m('postura'),
            'excessos_graus': {k: math.degrees(m('excesso_' + k)) for k in ('pelve', 'tronco', 'coluna', 'torcao', 'cabeca', 'pescoco')},
            'excesso_altura': m('excesso_altura'), 'excesso_giro_pelve': m('excesso_giro_pelve'),
            'imitacao_versao': 3,
            'imitacao': {'similaridade': m('imitacao'), 'pose': m('imitacao_pose'), 'velocidade': m('imitacao_velocidade'),
                         'extremos': m('imitacao_extremos'), 'altura': m('imitacao_altura')},
            'taxa_acao': m('taxa_acao'), 'escorregar': m('escorregar'), 'quietude': m('quietude'), 'base': m('base'),
            'causas': {name: float((e[:, COL['causa_queda']] == code).mean()) for code, name in CAUSES.items()},
            'termos_por_segundo': {k: m(k) for k in ('renda_vida', 'renda_tarefa', 'renda_imitacao', 'custo_parado',
                                                     'custo_postura', 'custo_acao', 'custo_escorregar', 'custo_quietude')}}


@torch.no_grad()
def evaluate(model, task, device, limits, worlds=32, seconds=30, seed=911, threads=12):
    """A média da política sem ruído, um exercício por vez; o trote no teto atual do currículo."""
    results = {}
    for code, name in enumerate(native.drill_ids()):
        speed = float(limits[code - 2]) if code >= 2 else -1.
        batch = native.MotorBatch(str(ROOT / 'assets'), str(task), worlds=worlds, threads=threads, seed=seed,
                                  drill=code, evaluation=True, seconds=seconds, trot_speed=speed)
        found = {}
        for _ in range(int(seconds * 30) + 2):
            obs = torch.as_tensor(batch.observations(), device=device)
            batch.step(model.actor(model.obs_norm(obs)).cpu().numpy())
            for row in batch.episodes():
                found.setdefault(int(row[0]), row)
            if len(found) == worlds:
                break
        if len(found) != worlds:
            raise RuntimeError('avaliação incompleta')
        e = np.asarray(list(found.values()))
        s = drill_summary(e)
        s.update({'falls': float(e[:, COL['caiu']].mean()), 'seconds': float(e[:, COL['segundos']].mean()),
                  'velocity_error_median': float(np.median(e[:, COL['erro_velocidade']])),
                  'drift_p90': float(np.quantile(e[:, COL['deriva']], .9)),
                  'rotation_error_degrees': float(np.degrees(np.abs(e[:, COL['giro_peito']] - e[:, COL['giro_pedido']])).mean()),
                  'speed_requested': speed})
        results[name] = s
    return results


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--treino', default='motor_autonomo')
    p.add_argument('--dados', type=Path, default=Path(os.environ.get('MATTER_RL_DATA', Path.home() / '.local/share/matter-engine/rl')))
    p.add_argument('--tarefa', type=Path, default=ROOT / 'assets/rl/motor/basico.json')
    p.add_argument('--mundos', type=int, default=512)
    p.add_argument('--modo', choices=MODES, default='maximo')
    p.add_argument('--threads', type=int, default=0)
    p.add_argument('--passos', type=int, default=32)
    p.add_argument('--rodadas', type=int, default=0, help='parar ao chegar nesta rodada (0 = sem limite)')
    p.add_argument('--semente', type=int, default=1)
    p.add_argument('--preparar', action='store_true', help='cria o treino (rede nova, política e status) sem treinar')
    p.add_argument('--avaliar', action='store_true')
    p.add_argument('--sem-controle', action='store_true')
    p.add_argument('--avaliar-cada', type=int, default=500)
    p.add_argument('--avaliar-mundos', type=int, default=32)
    p.add_argument('--avaliar-segundos', type=float, default=30.)
    p.add_argument('--ruido-escala', type=float, default=None, help='multiplica o ruído fixo por grupo; omitido preserva o salvo')
    p.add_argument('--ruido-correlacao', type=float, default=None, help='correlação AR(1) do ruído entre decisões, 0 a 0,95')
    p.add_argument('--sem-ensino-inicial', action='store_true', help='pula o ensino inicial (DAgger) de um treino novo')
    args = p.parse_args()
    if not args.treino or Path(args.treino).name != args.treino or args.treino in ('.', '..'):
        p.error('nome de treino inválido')
    if min(args.mundos, args.passos, args.avaliar_mundos) <= 0 or args.threads < 0 or args.rodadas < 0 or args.avaliar_cada < 0 \
            or not args.avaliar_segundos > 0:
        p.error('dimensões/orçamento inválidos')
    if args.ruido_escala is not None and not (math.isfinite(args.ruido_escala) and 0 < args.ruido_escala <= 5):
        p.error('escala de ruído deve estar em (0, 5]')
    if args.ruido_correlacao is not None and args.ruido_correlacao != 0:
        # A razão PPO usa gaussianas independentes por decisão; com AR(1) ela fica errada (revisão do Codex, 19/10).
        p.error('ruído correlacionado ainda não suportado: falta a probabilidade condicional coerente no PPO')
    torch.set_num_threads(1)
    run = args.dados / 'runs' / args.treino
    run.mkdir(parents=True, exist_ok=True)
    lock = open(run / 'treino.lock', 'a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('já existe um treinador para este treino') from None
    task = run / 'tarefa.json'
    if not task.exists():
        task.write_bytes(args.tarefa.read_bytes())
    task_bytes = task.read_bytes()
    recipe = json.loads(task_bytes)
    teacher_settings(recipe)
    torch.manual_seed(args.semente); np.random.seed(args.semente); random.seed(args.semente)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    threads = args.threads or MODES[args.modo]
    batch = native.MotorBatch(str(ROOT / 'assets'), str(task), worlds=args.mundos, threads=threads, seed=args.semente)
    permanent_prior(recipe, batch.action_names())
    manifest = {'contract': CONTRACT, 'body': batch.identity(), 'obs': batch.observation_names(), 'actions': batch.action_names()}
    checkpoint = run / 'ultimo.pt'
    count = {'rodada': 0, 'decisoes': 0, 'episodios': 0, 'segundos': 0.0}
    evaluation = {}
    warmup = 0
    noise = {'scale': 1.0, 'rho': 0.0}
    state = None
    if checkpoint.exists():
        state = torch.load(checkpoint, map_location=device, weights_only=False)
        if state['manifest'] != manifest:
            raise RuntimeError('o checkpoint é de outro contrato/corpo; o motor-v4 começa do zero (arquive o treino antigo)')
        noise = state['settings']['noise']
    if args.ruido_escala is not None:
        noise['scale'] = args.ruido_escala
    if args.ruido_correlacao is not None:
        noise['rho'] = args.ruido_correlacao
    model = make_model(batch, device, noise['scale'])
    actor_params = list(model.actor.parameters())
    actor = torch.optim.Adam(actor_params, lr=2e-5)  # o ruído fixo é pequeno no tronco: o passo inicial do Adam cabe no KL
    critic = torch.optim.Adam(model.critic.parameters(), lr=3e-4)
    curriculum = Curriculum(recipe, state.get('curriculum') if state else None)
    if state is not None:
        saved = state['model']
        saved['log_std'] = model.log_std.detach().clone()  # o ruído é fixo: vale a escala atual
        model.load_state_dict(saved)
        actor.load_state_dict(state['actor']); critic.load_state_dict(state['critic'])
        count = state['counters']; evaluation = state.get('evaluation', {}); warmup = state.get('critic_warmup', 0)
        torch.set_rng_state(state['rng'].cpu())
        if device.type == 'cuda' and state.get('cuda_rng') is not None:
            torch.cuda.set_rng_state_all([x.cpu() for x in state['cuda_rng']])
        if state.get('recipe') != recipe:
            shutil.copyfile(checkpoint, run / f'antes_receita_{count["rodada"]}_{hashlib.sha256(task_bytes).hexdigest()[:8]}.pt')
            warmup = 5
            with open(run / 'recipes.jsonl', 'a') as recipes:
                recipes.write(json.dumps({'iteration': count['rodada'], 'hash': hashlib.sha256(task_bytes).hexdigest(), 'recipe': recipe, 'resumed': True}) + '\n')
    batch.set_trot_limits(curriculum.limits)
    batch.set_shares(curriculum.shares())
    warm = state.get('warm_start') if state else None
    std = model.log_std.detach().exp()
    derivative = torch.as_tensor(batch.action_derivatives(), device=device)
    mapping = tuple(torch.as_tensor(np.asarray(batch.action_mapping())[:, k], dtype=torch.float32, device=device) for k in (0, 1))
    status = {}
    requests = {'stop': False, 'save': False, 'evaluate': False, 'paused': False}
    signal.signal(signal.SIGTERM, lambda *_: requests.__setitem__('stop', True))
    signal.signal(signal.SIGUSR1, lambda *_: requests.__setitem__('save', True))

    def write_curriculum():
        atomic_json(run / 'curriculo.json', {'limits': curriculum.limits, 'stage': curriculum.stage, 'shares': curriculum.shares(),
                                             'iteration': count['rodada']})

    def publish(state_name='treinando', row=None):
        row = row or {}
        status.update({'run': args.treino, 'kind': 'motor', 'task': TASK_ID, 'taskName': recipe.get('displayName', 'Motor autônomo'),
                       'contract': CONTRACT, 'state': state_name, 'pid': os.getpid(), 'mode': args.modo,
                       'iteration': count['rodada'], 'steps': count['decisoes'], 'episodes': count['episodios'],
                       'trainSeconds': count['segundos'], 'sps': row.get('dec_s', status.get('sps', 0)),
                       'device': device.type, 'threads': threads, 'environments': args.mundos, 'noise': noise,
                       'prior_weight': prior_weight(recipe, count['decisoes']), 'imitation_weight': batch.imitation_weight,
                       'trot_limits': curriculum.limits, 'stage': curriculum.stage, 'shares': curriculum.shares(), 'spawn_pose': 'alert_idle', 'assistance': 0.0, 'warm_start': warm,
                       'last': {'reward': row.get('nota_passo', 0), 'episode_seconds': row.get('ep_segundos', 0), 'fall_fraction': row.get('quedas', -1)}
                       if row else status.get('last', {'reward': 0, 'episode_seconds': 0, 'fall_fraction': -1}),
                       'evaluation': {'iteration': evaluation.get('iteration', 0), 'solvers': {'jogo': evaluation.get('metrics', {})}}})
        atomic_json(run / 'status.json', status)

    last_policy_publish = 0.

    def publish_policy():
        nonlocal last_policy_publish
        export_policy(model, run / 'exports/policy_latest.mrlpolicy', {
            'observationLayout': CONTRACT, 'actionLayout': CONTRACT, 'bodyHash': batch.identity(),
            'observationNames': batch.observation_names(), 'explorationStd': std.cpu().tolist(),
            'run': args.treino, 'iteration': count['rodada'], 'steps': count['decisoes']},
            action_limit=float(np.finfo(np.float32).max))
        atomic_json(run / 'latest.json', {'iteration': count['rodada']})
        last_policy_publish = time.monotonic()

    def save():
        tmp = checkpoint.with_suffix('.tmp')
        torch.save({'manifest': manifest, 'model': model.state_dict(), 'actor': actor.state_dict(), 'critic': critic.state_dict(),
                    'counters': count, 'rng': torch.get_rng_state(),
                    'cuda_rng': torch.cuda.get_rng_state_all() if device.type == 'cuda' else None,
                    'recipe': recipe, 'evaluation': evaluation, 'critic_warmup': warmup, 'curriculum': curriculum.state(),
                    'warm_start': warm,
                    'settings': {'worlds': args.mundos, 'rollout': args.passos, 'seed': args.semente, 'gamma': .99, 'lambda': .95,
                                 'epochs': 5, 'noise': noise, 'noise_radians': NOISE_RADIANS}}, tmp)
        tmp.replace(checkpoint)
        write_curriculum()
        publish_policy()

    command = [sys.executable, str(Path(__file__).resolve()), '--treino', args.treino, '--dados', str(args.dados),
               '--mundos', str(args.mundos), '--modo', args.modo, '--semente', str(args.semente),
               '--passos', str(args.passos), '--avaliar-cada', str(args.avaliar_cada),
               '--avaliar-mundos', str(args.avaliar_mundos), '--avaliar-segundos', str(args.avaliar_segundos)]
    atomic_json(run / 'run.json', {'kind': 'motor', 'task': TASK_ID, 'contract': CONTRACT, 'manifest': manifest,
                                   'comando': command, 'pasta': str(HERE)})
    if args.preparar:
        save(); publish('parado'); return 0
    if args.avaliar:
        print(json.dumps(evaluate(model, task, device, curriculum.limits, worlds=args.avaliar_mundos, seconds=args.avaliar_segundos),
                         ensure_ascii=False)); return 0
    server = None
    numerically_valid = True
    try:
        if not args.sem_controle:
            server = ControlServer(lambda: dict(status))
        # Com o clipe nos motores a rede nasce corrigindo a partir de zero: ensiná-la a imitar dobraria o clipe.
        if warm is None and not args.sem_ensino_inicial and batch.clip_drive == 0:
            publish('ensinando')
            warm = warm_start(model, batch, device, std, derivative, log=lambda m: print(m, flush=True))
            warmup = max(warmup, 5)  # o crítico aprende os retornos da aluna ensinada antes de o ator mexer
            batch.reset(args.semente + 1)
        save()
        publish()
        n, T = args.mundos, args.passos
        observation = torch.as_tensor(batch.observations(), device=device)
        eps = torch.randn(n, batch.action_size, device=device)

        def perform_evaluation():
            nonlocal evaluation
            metrics = evaluate(model, task, device, curriculum.limits, worlds=args.avaliar_mundos, seconds=args.avaliar_segundos,
                               threads=threads)
            evaluation = {'iteration': count['rodada'], 'metrics': metrics}
            with open(run / 'avaliacoes.jsonl', 'a') as g:
                g.write(json.dumps({'rodada': count['rodada'], 'hora': time.time(), 'decisoes': count['decisoes'], 'metrics': metrics}) + '\n')
            save()
            requests['evaluate'] = False
            publish('pausado' if requests['paused'] else 'treinando')

        def controls():
            nonlocal threads
            if not server:
                return
            for cmd, arguments, answer in server.pending():
                reply = {'ok': True}
                if cmd == 'parar':
                    requests['stop'] = True
                elif cmd == 'pausar':
                    requests['paused'] = True; save(); publish('pausado')
                elif cmd == 'retomar':
                    requests['paused'] = False; publish()
                elif cmd == 'salvar':
                    save()
                elif cmd == 'avaliar':
                    perform_evaluation()
                elif cmd == 'modo':
                    mode = arguments.get('modo')
                    if mode not in MODES:
                        reply = {'ok': False, 'error': 'modo inválido'}
                    else:
                        args.modo = mode; threads = MODES[mode]; batch.set_threads(threads)
                        publish('pausado' if requests['paused'] else 'treinando')
                else:
                    reply = {'ok': False, 'error': 'comando não suportado'}
                answer.put(reply)

        with open(run / 'log.jsonl', 'a', buffering=1) as log:
            while not args.rodadas or count['rodada'] < args.rodadas:
                controls()
                if requests['stop']:
                    break
                if requests['paused']:
                    time.sleep(.05); continue
                changed = task.read_bytes()
                if changed != task_bytes:
                    new_recipe = json.loads(changed)
                    teacher_settings(new_recipe)
                    permanent_prior(new_recipe, batch.action_names())
                    save()
                    batch.set_task(str(task))  # recusa atômica: uma receita inválida não troca nada
                    shutil.copyfile(checkpoint, run / f'antes_receita_{count["rodada"]}_{hashlib.sha256(task_bytes).hexdigest()[:8]}.pt')
                    task_bytes, recipe = changed, new_recipe
                    curriculum = Curriculum(recipe, curriculum.state())
                    batch.set_trot_limits(curriculum.limits)
                    batch.set_shares(curriculum.shares())
                    observation = torch.as_tensor(batch.observations(), device=device)
                    warmup = 5  # escala/objetivo novo: aquece o crítico com o ator preservado
                    with open(run / 'recipes.jsonl', 'a') as recipes:
                        recipes.write(json.dumps({'iteration': count['rodada'], 'hash': hashlib.sha256(changed).hexdigest(), 'recipe': recipe}) + '\n')
                    save()
                lam = prior_weight(recipe, count['decisoes'])
                prior_free = teacher_settings(recipe)[3]
                permanent = torch.as_tensor(permanent_prior(recipe, batch.action_names()), device=device)
                teaching = lam > 0 or bool(permanent.any())
                rho = noise['rho']
                started = time.perf_counter(); sim = 0.; episodes = []; raw_reward = 0.
                obsbuf, actbuf, logbuf, valbuf, rewbuf, donebuf, means, refs = [], [], [], [], [], [], [], []
                with torch.no_grad():
                    for _ in range(T):
                        mean = model.actor(model.obs_norm(observation))
                        eps = next_noise(eps, rho)
                        action = mean + std * eps
                        dist = torch.distributions.Normal(mean, std, validate_args=False)
                        obsbuf.append(observation); actbuf.append(action); logbuf.append(dist.log_prob(action).sum(-1))
                        means.append(mean); valbuf.append(model.value(model.critic_norm(observation)))
                        if teaching:
                            refs.append(torch.as_tensor(batch.reference_actions(), device=device))
                        before = time.perf_counter()
                        r, t, u = batch.step(action.cpu().numpy())
                        sim += time.perf_counter() - before
                        raw_reward += float(np.mean(r))
                        reward = torch.as_tensor(r, device=device)
                        term = torch.as_tensor(t, device=device).float(); trunc = torch.as_tensor(u, device=device).float()
                        if u.any():
                            final = torch.as_tensor(batch.final_observations(), device=device)
                            reward = reward + .99 * trunc * model.value(model.critic_norm(final))
                        done = torch.maximum(term, trunc)
                        rewbuf.append(reward); donebuf.append(done)
                        if rho > 0 and bool(done.any()):
                            fresh = done.bool()
                            eps[fresh] = torch.randn_like(eps[fresh])
                        rows = batch.episodes()
                        if len(rows):
                            episodes.append(rows)
                        observation = torch.as_tensor(batch.observations(), device=device)
                    values = torch.stack(valbuf); rewards = torch.stack(rewbuf); done_t = torch.stack(donebuf)
                    adv = advantages(rewards, values, done_t, model.value(model.critic_norm(observation)))
                    returns = (adv + values).flatten(); adv = adv.flatten(); adv = (adv - adv.mean()) / (adv.std() + 1e-8)
                obs = torch.stack(obsbuf).flatten(0, 1); actions = torch.stack(actbuf).flatten(0, 1)
                oldlog = torch.stack(logbuf).flatten(); oldmean = torch.stack(means).flatten(0, 1)
                reference = torch.stack(refs).flatten(0, 1) if refs else None
                actor_on = warmup <= 0; klmax = 0.; clips = []; prior_losses = []
                for _ in range(5):
                    for idx in torch.randperm(n * T, device=device).split(4096):
                        pred = model.value(model.critic_norm(obs[idx]))
                        loss = (pred - returns[idx]).square().mean(); critic.zero_grad(); loss.backward()
                        torch.nn.utils.clip_grad_norm_(model.critic.parameters(), 1, error_if_nonfinite=True); critic.step()
                        if not actor_on:
                            continue
                        mean = model.actor(model.obs_norm(obs[idx]))
                        logp = torch.distributions.Normal(mean, std, validate_args=False).log_prob(actions[idx]).sum(-1)
                        ratio = (logp - oldlog[idx]).exp()
                        with torch.no_grad():
                            kl = ((oldmean[idx] - mean).square() / (2 * std.square())).sum(-1).mean().item()
                        klmax = max(klmax, kl)
                        if kl > .03:
                            actor_on = False; actor.param_groups[0]['lr'] = max(1e-6, actor.param_groups[0]['lr'] / 1.5); continue
                        loss = -torch.minimum(ratio * adv[idx], ratio.clamp(.8, 1.2) * adv[idx]).mean()
                        if reference is not None:
                            prior = prior_loss(mean, reference[idx], mapping, prior_free, lam + permanent)
                            loss = loss + prior; prior_losses.append(float(prior.detach()))
                        actor.zero_grad(); loss.backward(); torch.nn.utils.clip_grad_norm_(actor_params, 1, error_if_nonfinite=True); actor.step()
                        clips.append(float(((ratio - 1).abs() > .2).float().mean()))
                if klmax < .005 and warmup <= 0:
                    actor.param_groups[0]['lr'] = min(3e-4, actor.param_groups[0]['lr'] * 1.1)
                warmup = max(0, warmup - 1)
                elapsed = time.perf_counter() - started
                rows = np.concatenate(episodes) if episodes else np.zeros((0, len(METRICS) + 2), np.float32)
                count['rodada'] += 1; count['decisoes'] += n * T; count['episodios'] += len(rows); count['segundos'] += elapsed
                promoted, staged = curriculum.observe(rows)
                if promoted:
                    batch.set_trot_limits(curriculum.limits); write_curriculum()
                    print('currículo: ' + ', '.join(f'{native.drill_ids()[d + 2]} -> {curriculum.limits[d]:.2f} m/s' for d in promoted), flush=True)
                if staged:
                    batch.set_shares(curriculum.shares()); write_curriculum()
                    print(f'currículo: parado e giro passaram; mistura final {curriculum.shares()}', flush=True)
                row = {'rodada': count['rodada'], 'hora': time.time(), 'decisoes': count['decisoes'], 'dec_s': n * T / elapsed,
                       'sim_s': sim, 'total_s': elapsed, 'nota_passo': raw_reward / T, 'kl': klmax, 'clip': float(np.mean(clips)) if clips else 0,
                       'lr_ator': actor.param_groups[0]['lr'], 'prior_peso': lam, 'prior_perda': float(np.mean(prior_losses)) if prior_losses else 0.,
                       'imitacao_peso': batch.imitation_weight, 'ruido': noise, 'tetos_trote': list(curriculum.limits),
                       'promovidos': [native.drill_ids()[d + 2] for d in promoted], 'fase': curriculum.stage, 'cotas': curriculum.shares(), 'recipe': hashlib.sha256(task_bytes).hexdigest(), 'rumos': {}}
                if len(rows):
                    # Os agregados mostram o desempenho do IDLE: os começos no movimento ficam fora (estão em em_movimento).
                    rest = rows[rows[:, INMOTION] < .5] if INMOTION is not None else rows
                    if len(rest):
                        row.update(ep_segundos=float(rest[:, COL['segundos']].mean()), quedas=float(rest[:, COL['caiu']].mean()))
                    for code, name in enumerate(native.drill_ids()):
                        e = rows[rows[:, 1] == code]
                        moving = e[:, INMOTION] > .5 if INMOTION is not None else np.zeros(len(e), bool)
                        if (~moving).any():
                            row['rumos'][name] = drill_summary(e[~moving])
                        if moving.any():
                            s = drill_summary(e[moving])
                            row.setdefault('em_movimento', {})[name] = {k: s[k] for k in (
                                'n', 'sem_cair', 'ep_segundos', 'vel_corpo', 'vel_pedida', 'erro_peito_graus', 'erro_rumo_graus',
                                'postura', 'imitacao')}
                log.write(json.dumps(row, allow_nan=False) + '\n'); publish(row=row)
                print(f"rodada={count['rodada']} decisões/s={row['dec_s']:.0f} sim={sim:.3f}s KL={klmax:.4f} episódios={len(rows)}", flush=True)
                if count['rodada'] % 25 == 0 or requests['save']:
                    save(); requests['save'] = False
                elif time.monotonic() - last_policy_publish >= 2.:
                    publish_policy()
                if requests['evaluate'] or (args.avaliar_cada and count['rodada'] % args.avaliar_cada == 0):
                    perform_evaluation()
    except BaseException as error:
        numerically_valid = all(bool(torch.isfinite(x).all()) for x in model.state_dict().values())
        status['message'] = str(error)
        publish('erro'); raise
    finally:
        if numerically_valid:
            save()
        if status.get('state') != 'erro':
            publish('parado')
        if server:
            server.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
