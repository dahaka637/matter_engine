"""Analise de um levantar do Kimodo (npz). uso: analisa.py [-v] a.npz b.npz ...

Mede (y para cima, metros, 30 Hz):
  - comeca de bruços (pelve baixa, frente do corpo para o chao) e termina em pe;
  - subida: do inicio do movimento ate ficar em pe (agil = curto);
  - base: centro de massa contra o poligono dos apoios no chao (maos, joelhos,
    pes) quadro a quadro; tempo "sem base" (cm fora por mais de 5 cm);
  - quantos apoios saem de uma vez (troca de apoio com 2+ saindo juntos);
  - flexao maxima do quadril e do joelho (nosso rig: quadril 120, joelho 145);
  - atravessar o chao (juntas abaixo do chao).
"""
import sys, math, numpy as np
from soma import IX, com

RATE = 30.0


def unit(v):
    return v / (np.linalg.norm(v, axis=-1, keepdims=True) + 1e-9)


def angle(a, b):
    return np.degrees(np.arccos(np.clip(np.sum(unit(a) * unit(b), -1), -1, 1)))


def hull_margin(points, q):
    pts = sorted({(round(float(a), 4), round(float(b), 4)) for a, b in points})
    if not pts:
        return -9.0
    if len(pts) < 3:
        return -min(math.hypot(q[0] - a, q[1] - b) for a, b in pts) if len(pts) == 1 else -seg_dist(pts[0], pts[1], q)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        return -seg_dist(hull[0], hull[-1], q)
    inside = all(cross(hull[i], hull[(i + 1) % len(hull)], q) >= 0 for i in range(len(hull)))
    d = min(seg_dist(hull[i], hull[(i + 1) % len(hull)], q) for i in range(len(hull)))
    return d if inside else -d


def seg_dist(a, b, q):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / L))
    return math.hypot(q[0] - a[0] - dx * t, q[1] - a[1] - dy * t)


# pontos de apoio: (nome, juntas cuja media e o ponto, altura maxima para contar)
SUPPORTS = [
    ('maoE', ('LeftHand', 'LeftHandMiddle1'), 0.08), ('maoD', ('RightHand', 'RightHandMiddle1'), 0.08),
    ('joelhoE', ('LeftShin',), 0.12), ('joelhoD', ('RightShin',), 0.12),
    ('ponta E', ('LeftToeBase',), 0.06), ('ponta D', ('RightToeBase',), 0.06),
    ('calcE', ('LeftFoot',), 0.12), ('calcD', ('RightFoot',), 0.12),
]


def analyze(path, verbose=False):
    J = np.load(path)['posed_joints'].astype(np.float64)  # (T, 77, 3)
    T = len(J)
    ground = float(np.percentile(J[:, [IX['LeftToeBase'], IX['RightToeBase']], 1].min(axis=1), 5))
    J = J - np.array([0, ground, 0])
    hips = J[:, IX['Hips']]
    up = unit(J[:, IX['Chest']] - J[:, IX['Hips']])
    right = unit(J[:, IX['RightLeg']] - J[:, IX['LeftLeg']])
    front = unit(np.cross(up, right))  # frente do peito (y para cima, mao direita)
    prone0 = hips[0, 1] < 0.32 and front[0, 1] < -0.6
    supine0 = hips[0, 1] < 0.32 and front[0, 1] > 0.6
    standing = (hips[:, 1] > 0.82) & (up[:, 1] > 0.85)
    end_ok = bool(standing[-8:].all())
    # subida: comeca quando a pelve passa 6 cm acima da altura deitada; fica em
    # pe no primeiro quadro em pe que dura ate o fim
    lie = float(np.median(hips[:max(3, int(0.3 * RATE)), 1]))
    moving = np.where(hips[:, 1] > lie + 0.06)[0]
    t0 = int(moving[0]) if len(moving) else T - 1
    st = np.where(~standing)[0]
    t1 = int(st[-1] + 1) if len(st) else 0
    rise = (t1 - t0) / RATE if t1 > t0 else float('nan')
    # base
    margins, nsup, sup_sets = [], [], []
    for k in range(T):
        pts, names = [], []
        for name, joints, hmax in SUPPORTS:
            p = np.mean([J[k, IX[j]] for j in joints], axis=0)
            if p[1] < hmax:
                pts.append((p[0], p[2])); names.append(name)
        c = com(J[k])
        margins.append(hull_margin(pts, (c[0], c[2])))
        sup_sets.append(set(names))
    margins = np.array(margins)
    window = range(t0, min(T, t1 + 1))
    bad = [k for k in window if margins[k] < -0.05]
    worst = min((margins[k] for k in window), default=0.0)
    # apoios saindo juntos (mao/joelho; os pontos do pe contam como "pe")
    def limbset(s):
        out = set()
        for n in s:
            out.add('peE' if n in ('ponta E', 'calcE') else 'peD' if n in ('ponta D', 'calcD') else n)
        return out
    multi = 0
    prev = limbset(sup_sets[0])
    for k in range(1, T):
        cur = limbset(sup_sets[k])
        if len(prev - cur) >= 2 and k in window:
            multi += 1
        prev = cur
    # flexoes
    trunk = J[:, IX['Chest']] - J[:, IX['Hips']]
    hipf = {s: 180.0 - angle(trunk, J[:, IX[f'{s}Shin']] - J[:, IX[f'{s}Leg']]) for s in ('Left', 'Right')}
    kneef = {s: 180.0 - angle(J[:, IX[f'{s}Leg']] - J[:, IX[f'{s}Shin']], J[:, IX[f'{s}Foot']] - J[:, IX[f'{s}Shin']]) for s in ('Left', 'Right')}
    hip_max = max(float(hipf[s][list(window)].max()) for s in hipf) if len(window) else 0
    knee_max = max(float(kneef[s][list(window)].max()) for s in kneef) if len(window) else 0
    body = [i for i, n in enumerate(__import__('soma').NAMES) if 'Hand' not in n or n.endswith('Hand')]
    below = float(-min(0.0, J[:, body, 1].min()))
    # sequencia de apoios (resumo): conjuntos distintos durante a subida
    seq = []
    for k in window:
        s = '+'.join(sorted(limbset(sup_sets[k])))
        if not seq or seq[-1][0] != s:
            seq.append([s, k, k])
        else:
            seq[-1][2] = k
    seq = [x for x in seq if x[2] - x[1] >= 3]
    r = dict(path=path, prone0=prone0, supine0=supine0, end=end_ok, t0=t0 / RATE, t1=t1 / RATE, rise=rise,
             bad=len(bad) / RATE, worst=worst, multi=multi, hip=hip_max, knee=knee_max, below=below,
             seq=seq, lie=lie)
    return r


def line(r):
    ok = (r['prone0'] or r['supine0']) and r['end'] and not math.isnan(r['rise'])
    return (f"{r['path'].split('/')[-1]:28s} {'OK ' if ok else '-- '} inicio {'bruços' if r['prone0'] else 'costas' if r['supine0'] else 'outro '} empe_fim {int(r['end'])} "
            f"| sobe {r['t0']:.2f}->{r['t1']:.2f} s ({r['rise']:.2f} s) | sem base {r['bad']:.2f} s pior {r['worst']:+.2f} "
            f"| saidas duplas {r['multi']} | quadril {r['hip']:.0f} joelho {r['knee']:.0f} | abaixo {r['below']*100:.1f} cm")


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('-')]
    for p in args:
        r = analyze(p)
        print(line(r))
        if '-v' in sys.argv:
            for s, a, b in r['seq']:
                print(f"     {a/RATE:5.2f}-{b/RATE:5.2f} {s}")
