"""Diagnóstico D0, sem política treinada e sem alterar o repositório.

D0: distance_ratio = distância mediana com ruído / distância sem ruído;
judge_ratio = nota com ruído / nota do mesmo corpo sem ruído, ambas pelo
juiz treinado contra a condição com ruído. Passa se distance_ratio <= 1,5,
judge_ratio >= 0,6 e reserved_score ausente ou >= 0,7. Denominador zero
não permite aprovação (razão indefinida). As medidas refletem a física após
o corte dos motores; não são uma estimativa do sigma efetivo dos alvos.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import numpy as np
import torch

from . import native, paths
from .amp import Judge, JudgeSettings
from .body_trainer import axis_group

GROUPS = ('pelve', 'angulos', 'velocidades_eixos', 'partes')
AXIS_GROUPS = ('legs', 'toes', 'arms', 'trunk')


def measure_group(name):
    if name.startswith('pelvis'):
        return 'pelve'
    if name.endswith('.position'):
        return 'angulos'
    if name.endswith('.velocity'):
        return 'velocidades_eixos'
    return 'partes'


def weighted_normalization(pairs, weights, min_std=0.05):
    """Mesma estatística ponderada das duas metades usada por amp.Judge."""
    pairs, weights = np.asarray(pairs), np.asarray(weights, dtype=np.float64)
    half = pairs.shape[1] // 2
    keep = weights > 0
    if not keep.any():
        raise ValueError('acervo sem pares de treino com peso positivo')
    x = np.concatenate((pairs[keep, :half], pairs[keep, half:]))
    w = np.tile(weights[keep], 2)
    w /= w.sum()
    mean = (x * w[:, None]).sum(0)
    std = np.maximum(np.sqrt(((x - mean)**2 * w[:, None]).sum(0)), min_std)
    return mean, std


def split_groups(ids, seed=7):
    """Divide grupos inteiros; nunca divide um clipe/mundo por linha."""
    ids = np.asarray(ids)
    groups = np.unique(ids)
    if len(groups) < 2:
        return np.zeros(len(ids), bool), np.zeros(len(ids), bool)
    groups = np.random.default_rng(seed).permutation(groups)
    cut = max(1, min(len(groups)-1, int(0.8 * len(groups))))
    train = np.isin(ids, groups[:cut])
    return train, ~train


def d0_criterion(distance, baseline_distance, score, baseline_score, reserved_score):
    dr = float(distance / baseline_distance) if baseline_distance > 0 else None
    jr = float(score / baseline_score) if baseline_score > 0 else None
    return {'distance_ratio': dr, 'judge_ratio': jr, 'reserved_score': reserved_score,
            'pass': bool(dr is not None and jr is not None and dr <= 1.5 and jr >= 0.6
                         and (reserved_score is None or reserved_score >= 0.7))}


def _sigma(condition, names):
    if isinstance(condition, dict):
        if set(condition) != set(AXIS_GROUPS):
            raise ValueError('sigma por grupo precisa de legs,toes,arms,trunk')
        values = np.array([condition[axis_group(n)] for n in names], np.float32)
    else:
        values = np.full(len(names), condition, np.float32)
    if not np.isfinite(values).all() or (values < 0).any():
        raise ValueError('sigma deve ser finito e não negativo')
    return values


def _collect(module, task, condition, worlds, seconds, seed, drill=''):
    rows, ids, seen = [], [], set()
    used_worlds, surviving = 0, 0
    metadata = None
    # Novos mundos, com novas sementes, fornecem trajetórias independentes.
    # Limite explícito: não inventar pares quando o corpo cai imediatamente.
    for attempt in range(64):
        batch = module.BodyBatch(str(task), environments=worlds, per_scene=min(4, worlds),
                                 threads= min(8, worlds), seed=seed + attempt,
                                 evaluation=False, drill=drill)
        names = list(batch.action_names())
        rad = np.asarray(batch.action_radians_per_unit(), np.float32)
        sigma = _sigma(condition, names) / rad
        rng = np.random.default_rng(seed + attempt)
        zero = np.zeros((worlds, batch.action_size), np.float32)
        alive = np.ones(worlds, bool)
        local_rows, local_ids = [], []
        for step in range(max(3, int(np.ceil(seconds / batch.policy_dt)))):
            action = (rng.normal(size=zero.shape) * sigma).astype(np.float32)
            batch.step(np.ascontiguousarray(action), zero)
            alive &= ~(np.asarray(batch.terminated(), bool) | np.asarray(batch.truncated(), bool))
            if metadata is None:
                metadata = (names, rad.copy(), np.array(batch.style_pairs())[0].copy(),
                            list(batch.observation_names()), np.array(batch.observations())[0].copy())
            if step >= 2:
                sp = np.array(batch.style_pairs(), np.float32)
                for world in np.flatnonzero(alive):
                    local_rows.append(sp[world].copy())
                    local_ids.append(world)
        # Descarta a trajetória inteira dos mundos que caíram/truncaram.
        for row, world in zip(local_rows, local_ids):
            if alive[world] and np.isfinite(row).all() and row.tobytes() not in seen:
                seen.add(row.tobytes())
                rows.append(row)
                ids.append(used_worlds + world)
        used_worlds += worlds
        surviving += int(alive.sum())
        del batch
        if len(rows) >= 4096:
            return np.asarray(rows), np.asarray(ids), metadata, {
                'pares_distintos': len(rows), 'mundos': used_worlds,
                'fracao_sobrevivente': surviving / used_worlds, 'segundos_por_mundo': seconds}
    raise RuntimeError(f'coleta insuficiente: {len(rows)} pares distintos; aumente mundos/segundos')


def _nearest(x, real, device):
    """Busca exata em blocos nos dois lados, sem matriz quadrática completa."""
    real = torch.as_tensor(real, dtype=torch.float32, device=device)
    distances, indices = [], []
    with torch.no_grad():
        for chunk in np.array_split(x, max(1, int(np.ceil(len(x)/256)))):
            t = torch.as_tensor(chunk, dtype=torch.float32, device=device)
            best = torch.full((len(t),), float('inf'), device=device)
            index = torch.zeros(len(t), dtype=torch.long, device=device)
            for start in range(0, len(real), 2048):
                d, i = torch.cdist(t, real[start:start+2048]).min(1)
                take = d < best
                best[take], index[take] = d[take], i[take] + start
            distances.extend(best.cpu().tolist())
            indices.extend(index.cpu().tolist())
    return np.asarray(distances), np.asarray(indices, int)


def _classifier(real, fake, real_ids, fake_ids, cols, device, seed):
    rt, rv = split_groups(real_ids, seed)
    ft, fv = split_groups(fake_ids, seed)
    if not rt.any() or not ft.any() or not len(cols):
        return {'accuracy': None, 'motivo': 'faltam dois clipes/mundos ou medidas'}
    rng = np.random.default_rng(seed)
    def balanced(a, b):
        count = min(len(a), len(b), 4096)
        a = a[rng.choice(len(a), count, replace=False)][:, cols]
        b = b[rng.choice(len(b), count, replace=False)][:, cols]
        return (torch.tensor(np.concatenate((a, b)), device=device, dtype=torch.float32),
                torch.tensor(np.r_[np.ones(count), np.zeros(count)], device=device, dtype=torch.float32))
    x, y = balanced(real[rt], fake[ft])
    vx, vy = balanced(real[rv], fake[fv])
    torch.manual_seed(seed)
    net = torch.nn.Sequential(torch.nn.Linear(len(cols), 32), torch.nn.ReLU(), torch.nn.Linear(32, 1)).to(device)
    opt = torch.optim.Adam(net.parameters(), lr=0.003)
    for _ in range(100):
        i = rng.integers(0, len(x), min(256, len(x)))
        loss = torch.nn.functional.binary_cross_entropy_with_logits(net(x[i]).flatten(), y[i])
        opt.zero_grad(); loss.backward(); opt.step()
    with torch.no_grad():
        acc = float(((net(vx).flatten() > 0) == (vy > 0)).float().mean())
    return {'accuracy': acc, 'clipes_treino': np.unique(real_ids[rt]).tolist(),
            'clipes_validacao': np.unique(real_ids[rv]).tolist(),
            'mundos_treino': np.unique(fake_ids[ft]).tolist(),
            'mundos_validacao': np.unique(fake_ids[fv]).tolist()}


def _train_judge(dataset, fake, rounds, device, seed):
    torch.manual_seed(seed)
    judge = Judge(dataset, JudgeSettings(), device, seed=seed)
    judge.remember(fake)
    health = {}
    for _ in range(rounds):
        health = judge.update()
    return judge, health


def _score(judge, rows):
    if not len(rows):
        return None
    total = 0.0
    for start in range(0, len(rows), 1024):
        chunk = rows[start:start+1024]
        total += judge.score(torch.as_tensor(chunk, device=judge.device)).sum().item()
    return total / len(rows)



def _action_report(metadata, pairs, names, pair_group):
    anames, radians, style, onames, obs = metadata
    half = len(names)
    report, grouped = {}, {}
    for i, action in enumerate(anames):
        dof = action.removesuffix('.target')
        if dof + '.position' not in names:
            report[action] = {'escala_rad': float(radians[i]), 'grupo_eixo': axis_group(action),
                              'grupos_clipes': {}, 'motivo': 'ângulo ausente das medidas do acervo (ex.: dedos)'}
            continue
        j = names.index(dof + '.position')
        base = style[half+j] - obs[onames.index(dof + '.position_from_base')]
        values = (pairs[:, j] - base) / radians[i]
        stats = {}
        for group in np.unique(pair_group):
            a = values[pair_group == group]
            stats[str(group)] = {'p5': float(np.percentile(a, 5)), 'p95': float(np.percentile(a, 95)),
                                 'max': float(np.abs(a).max())}
            grouped.setdefault((axis_group(action), str(group)), []).append(a)
        report[action] = {'escala_rad': float(radians[i]), 'grupo_eixo': axis_group(action), 'grupos_clipes': stats}
    groups = {}
    for (axis, clip), arrays in grouped.items():
        a = np.concatenate(arrays)
        groups.setdefault(axis, {})[clip] = {'p5': float(np.percentile(a, 5)),
                                            'p95': float(np.percentile(a, 95)), 'max': float(np.abs(a).max())}
    return {'eixos': report, 'grupos': groups}


def run_diagnostico(task: str, conditions: dict[str, float | dict], out_dir: Path,
                    worlds=128, seconds=3.0, judge_rounds=12, seed=7, with_controls=True) -> dict:
    """Executa D0 e grava JSON/Markdown; fórmula do critério no docstring do módulo."""
    if worlds < 2 or seconds <= 0 or judge_rounds < 1:
        raise ValueError('precisa de >=2 mundos, segundos positivos e >=1 rodada')
    conditions = {**conditions, 'sem_ruido': 0.0}
    for c in conditions.values():
        _sigma(c, ['LeftThigh.target', 'LeftToe.target', 'LeftArm.target', 'Chest.target'])
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    source = paths.task_path(task)
    config = json.loads(source.read_text())
    config['body'] = str((source.parent / config['body']).resolve())
    config['style']['dataset'] = str((source.parent / config['style']['dataset']).resolve())
    config.setdefault('initialState', {}).update(referenceFraction=0.0, movingFraction=0.0, posture='natural')
    probe = out_dir / 'tarefa_diagnostico.json'
    probe.write_text(json.dumps(config, ensure_ascii=False, indent=2))
    module = native.load()
    ds = module.motion_dataset(config['body'], config['style']['dataset'])
    pairs = np.asarray(ds['pairs'], np.float32)
    weights = np.asarray(ds['pair_weight'], np.float64)
    clip_ids = np.asarray(ds['pair_clip'], int)
    reserved = np.array([ds['clips'][c]['reserved'] for c in clip_ids], bool)
    train = (weights > 0) & ~reserved
    # Reservados nunca entram no juiz nem na estatística.
    ds = dict(ds); ds['pair_weight'] = np.where(train, weights, 0).tolist()
    mean, std = weighted_normalization(pairs, ds['pair_weight'])
    normalize = lambda x: np.clip((x - np.tile(mean, 2)) / np.tile(std, 2), -10, 10)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    names = list(ds['names']); half = len(names)
    kinds = np.array([measure_group(n) for n in names])
    agents, world_ids, meta, collection = {}, {}, {}, {}
    for label, c in conditions.items():
        agents[label], world_ids[label], meta[label], collection[label] = _collect(module, probe, c, worlds, seconds, seed)
    real_n = normalize(pairs[train])
    rd = _nearest(normalize(pairs[reserved]), real_n, device)[0] if reserved.any() else []
    result = {'tarefa': task, 'dispositivo': str(device), 'condicoes': {}, 'd0': {},
              'reservados': {'pares': int(reserved.sum()), 'distancia_mediana': float(np.median(rd)) if len(rd) else None,
                             'mensagem': None if reserved.any() else 'Não há clipes reservados'},
              'normalizacao': {'media': mean.tolist(), 'desvio': std.tolist(), 'piso': 0.05, 'corte': 10},
              'controles_mistura': None, 'acao_clipes': None,
              'limites': ['Corpo com ação média zero; não é uma política treinada.',
                         'Não inclui começos por clipe em estados de passada.',
                         'Ações dos dedos não recuperáveis: o acervo de medidas não contém seus ângulos.',
                         'Não estima sigma dos alvos após corte; mede pares após atuação física.',
                         'Classificador: uma divisão por clipe/mundo, 100 passos; acurácia exploratória.']}
    for label, x in agents.items():
        xn = normalize(x)
        med = np.median(np.abs(xn.reshape(-1, half)), axis=0)
        order = np.argsort(-med)[:10]
        distance, nearest = _nearest(xn, real_n, device)
        classifier = {}
        for group in ('todas',) + GROUPS:
            idx = np.arange(half) if group == 'todas' else np.flatnonzero(kinds == group)
            classifier[group] = _classifier(real_n, xn, clip_ids[train], world_ids[label],
                                             np.r_[idx, idx+half], device, seed)
        judge, health = _train_judge(ds, x, judge_rounds, device, seed)
        alphas = [0., 0.25, 0.5, 0.75, 0.9, 1.]
        target = pairs[train][nearest]
        result['condicoes'][label] = {
            'sigma_rad': conditions[label], 'coleta': collection[label],
            'z_mediano_grupos': {g: float(np.median(med[kinds == g])) if (kinds == g).any() else None for g in GROUPS},
            'medidas_mais_longe': [{'medida': names[i], 'z_mediano': float(med[i])} for i in order],
            'distancia_mediana': float(np.median(distance)), 'classificador': classifier,
            'juiz': {'saude': health, 'nota_corpo': _score(judge, x),
                     'nota_sem_ruido': _score(judge, agents['sem_ruido']),
                     'nota_reservados': _score(judge, pairs[reserved]),
                     'caminho': [{'alpha': a, 'nota': _score(judge, ((1-a)*x+a*target).astype(np.float32))} for a in alphas]}}
        del judge
    baseline = result['condicoes']['sem_ruido']['distancia_mediana']
    for label, c in conditions.items():
        if label == 'sem_ruido' or not np.any(_sigma(c, meta[label][0]) > 0):
            continue
        r = result['condicoes'][label]; j = r['juiz']
        result['d0'][label] = d0_criterion(r['distancia_mediana'], baseline, j['nota_corpo'], j['nota_sem_ruido'], j['nota_reservados'])
    pair_group = np.array([ds['groups'][ds['clips'][c]['group']]['id'] for c in clip_ids])
    result['acao_clipes'] = _action_report(meta['sem_ruido'], pairs, names, pair_group)
    if with_controls:
        standing = train & (pair_group == 'parado')
        selection = 'grupo parado'
        if not standing.any():
            velocity = [i for i, n in enumerate(names) if n.startswith('pelvis') and 'velocity' in n and 'angular' not in n]
            if not velocity:
                raise ValueError('sem medidas de velocidade da pelve para selecionar clipes lentos')
            speeds = {int(c): float(np.linalg.norm(pairs[clip_ids == c][:, velocity], axis=1).mean()) for c in np.unique(clip_ids[train])}
            minimum = min(speeds.values())
            chosen = [c for c, s in speeds.items() if np.isclose(s, minimum)]
            standing = train & np.isin(clip_ids, chosen)
            selection = 'clipes de menor velocidade média da pelve (norma)'
        # O controle pede explicitamente o exercício parado, mesmo no P3.
        control_config = copy.deepcopy(config)
        control_config['drills'] = [{'id': 'parado', 'weight': 1.0, 'task': {'velocity': 0.7, 'facing': 0.3},
                                    'pilot': {'kind': 'hold', 'camera': {'fixed': 1.0, 'follow': 0.0, 'turns': 0.0}}}]
        cp = out_dir / 'tarefa_controle_parado.json'
        cp.write_text(json.dumps(control_config, ensure_ascii=False, indent=2))
        stopped, _, _, collected = _collect(module, cp, 0., worlds, seconds, seed, drill='parado')
        control = {'selecao': selection, 'clipes': np.unique(clip_ids[standing]).tolist(), 'coleta': collected}
        for label, w in [('completo', np.asarray(ds['pair_weight'])), ('so_parado', np.where(standing, weights, 0))]:
            subset = dict(ds); subset['pair_weight'] = w.tolist()
            judge, health = _train_judge(subset, stopped, judge_rounds, device, seed)
            control[label] = {'nota_parado_acervo': _score(judge, pairs[standing]),
                              'nota_corpo_parado': _score(judge, stopped), 'saude': health}
            del judge
        control['diferenca_nota_parado'] = control['so_parado']['nota_parado_acervo'] - control['completo']['nota_parado_acervo']
        result['controles_mistura'] = control
    _write_report(result, out_dir)
    return result


def _write_report(result, out_dir):
    (out_dir / 'diagnostico_juiz.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    (out_dir / 'diagnostico_juiz.md').write_text(markdown_report(result))


def markdown_report(result):
    lines = [f"# Diagnóstico do juiz — {result['tarefa']}", '',
             'D0: distância com/sem ruído ≤ 1,5; nota com/sem ruído ≥ 0,6 pelo mesmo juiz; reservado ≥ 0,7 quando presente.',
             '', f"Reservados: {result['reservados']}"]
    for label, r in result['condicoes'].items():
        lines += ['', f'## {label}', '', f"Coleta: {r['coleta']}",
                  f"Distância mediana ao treino: {r['distancia_mediana']:.4f}",
                  f"|z| mediano por grupo: {r['z_mediano_grupos']}", '', 'As dez medidas mais longe:']
        lines += [f"- {v['medida']}: {v['z_mediano']:.4f}" for v in r['medidas_mais_longe']]
        lines += ['', 'Classificador (validação por clipe/mundo):']
        lines += [f"- {g}: {v['accuracy']}" for g, v in r['classificador'].items()]
        lines += ['', f"Juiz: {r['juiz']}", f"D0: {result['d0'].get(label, 'linha de base')}"]
    lines += ['', '## Controle da mistura', '', json.dumps(result['controles_mistura'], ensure_ascii=False, indent=2),
              '', '## Ação que reproduz os clipes (unidades)', '', '| eixo | grupo de clipes | p5 | p95 | max abs |', '|---|---|---:|---:|---:|']
    for axis, info in result['acao_clipes']['eixos'].items():
        for group, s in info['grupos_clipes'].items():
            lines.append(f"| {axis} | {group} | {s['p5']:.3f} | {s['p95']:.3f} | {s['max']:.3f} |")
    lines += ['', 'Agregação por grupo de eixos: ' + json.dumps(result['acao_clipes']['grupos'], ensure_ascii=False), '', '## Limites', '']
    lines += ['- ' + s for s in result['limites']]
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description='Diagnóstico D0 do juiz de estilo')
    parser.add_argument('--tarefa', required=True)
    parser.add_argument('--sigma-grupos', help='legs=0.06,toes=0.06,arms=0.05,trunk=0.03')
    parser.add_argument('--sigma-todos', default='0.05,0.15')
    parser.add_argument('--saida', required=True, type=Path)
    parser.add_argument('--mundos', type=int, default=128)
    parser.add_argument('--segundos', type=float, default=3.0)
    parser.add_argument('--rodadas', type=int, default=12)
    parser.add_argument('--semente', type=int, default=7)
    parser.add_argument('--sem-controles', action='store_true')
    args = parser.parse_args(argv)
    try:
        conditions = {f'sigma_{s}rad': float(s) for s in args.sigma_todos.split(',') if s.strip()}
        if args.sigma_grupos:
            conditions['sigma_grupos'] = dict((k.strip(), float(v)) for k, v in
                                              (item.split('=') for item in args.sigma_grupos.split(',')))
        result = run_diagnostico(args.tarefa, conditions, args.saida, args.mundos, args.segundos,
                                 args.rodadas, args.semente, not args.sem_controles)
    except ValueError as error:
        parser.error(str(error))
    print(markdown_report(result))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
