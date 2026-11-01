"""Analise de uma queda com protecao (ou de um tropeco recuperado) do Kimodo (npz, SOMA77, Y para cima, 30 Hz).

uso: queda.py [-v] a.npz b.npz ...

Mede, por amostra:
  - se comeca de pe (pelve > 0,82 m e tronco a menos de ~30 graus da vertical nos 5 primeiros quadros);
  - o primeiro toque de cada apoio no chao (altura do centro da junta): mao, joelho, antebraco (cotovelo e mao
    no chao), quadril (a pelve sentando/deitando), tronco (coluna/peito) e cabeca;
  - quais apoios chegam antes do tronco e a velocidade vertical do tronco ao tocar (devagar = amorteceu);
  - o pico de velocidade para baixo do centro de massa;
  - como termina: deitado (pelve < 0,32 m) ou de pe (os 8 ultimos quadros);
  - sem queda (tropeco/empurrao): os passos (pes pousando), a maior inclinacao do tronco e se algo alem dos pes
    tocou o chao;
  - atravessar o chao (o ponto mais baixo das juntas do corpo).
As alturas foram calibradas nas poses deitadas do Kimodo (de brucos, de costas, de lado): o centro das juntas do
tronco fica a 0,06-0,12 m do chao com o corpo apoiado.
"""
import sys

import numpy as np

from soma import IX, NAMES, com

RATE = 30.0
# (nome, juntas que tem de estar TODAS abaixo da altura, altura m)
SUPPORTS = [
    ('maoE', ('LeftHand',), 0.08), ('maoD', ('RightHand',), 0.08),
    ('joelhoE', ('LeftShin',), 0.12), ('joelhoD', ('RightShin',), 0.12),
    ('antebracoE', ('LeftForeArm', 'LeftHand'), 0.09), ('antebracoD', ('RightForeArm', 'RightHand'), 0.09),
    ('quadril', ('Hips',), 0.15),
]
TRUNK = ('Spine1', 'Spine2', 'Chest')
TRUNK_HEIGHT = 0.14
HEAD_HEIGHT = 0.14


def unit(v):
    return v / (np.linalg.norm(v, axis=-1, keepdims=True) + 1e-9)


def first(mask):
    k = np.flatnonzero(mask)
    return int(k[0]) if len(k) else None


def analyze(path):
    J = np.load(path)['posed_joints'].astype(np.float64)
    T = len(J)
    y = J[..., 1]
    hips = y[:, IX['Hips']]
    up = unit(J[:, IX['Chest']] - J[:, IX['Hips']])
    tilt = np.degrees(np.arccos(np.clip(up[:, 1], -1, 1)))
    standing = (hips > 0.82) & (up[:, 1] > 0.85)
    start_ok = bool(standing[:5].all())
    end = 'de pe' if standing[-8:].all() else 'deitado' if hips[-1] < 0.32 else f'pelve {hips[-1]:.2f} m'
    touch = {}
    for name, joints, h in SUPPORTS:
        touch[name] = first(np.all([y[:, IX[j]] < h for j in joints], axis=0))
    trunk_y = np.min([y[:, IX[j]] for j in TRUNK], axis=0)
    t_trunk = first(trunk_y < TRUNK_HEIGHT)
    t_head = first(y[:, IX['Head']] < HEAD_HEIGHT)
    v_trunk = None
    if t_trunk is not None and t_trunk >= 3:
        v_trunk = float((trunk_y[t_trunk] - trunk_y[t_trunk - 3]) / 3 * RATE)
    c = np.array([com(P) for P in J])
    v_com = np.diff(c[:, 1]) * RATE
    before = sorted((k, n) for n, k in touch.items() if k is not None and (t_trunk is None or k < t_trunk))
    # passos: o pe (calcanhar ou ponta) pousa (abaixo de 4 cm depois de passar de 8 cm)
    landings = []
    for side in ('Left', 'Right'):
        foot = np.minimum(y[:, IX[f'{side}Foot']] - 0.05, y[:, IX[f'{side}ToeBase']])
        up_air = False
        for k in range(T):
            if foot[k] > 0.08:
                up_air = True
            elif foot[k] < 0.04 and up_air:
                landings.append(k)
                up_air = False
    steps = len(landings)
    k_tilt = int(np.argmax(tilt))
    # sem queda: os passos ate 1 s depois da maior inclinacao do tronco (a recuperacao)
    after = sorted(k for k in landings if k_tilt <= k <= k_tilt + RATE)
    body = [i for i, n in enumerate(NAMES) if 'Hand' not in n or n.endswith('Hand')]
    below = max(0.0, float(-y[:, body].min()))
    ground_any = [n for n, k in touch.items() if k is not None] + (['tronco'] if t_trunk is not None else [])
    return dict(path=path, T=T, start=start_ok, end=end, touch=touch, t_trunk=t_trunk, t_head=t_head,
                v_trunk=v_trunk, v_com=float(-v_com.min()), before=before, steps=steps, tilt=float(tilt.max()),
                k_tilt=k_tilt, after=after,
                below=below, ground=ground_any, hips_min=float(hips.min()))


def fmt_t(k):
    return '  -  ' if k is None else f'{k / RATE:4.2f}s'


def line(r):
    name = r['path'].split('/')[-1]
    head = f"{name:24s} {r['T'] / RATE:.1f}s empe0 {int(r['start'])} fim {r['end']:9s}"
    if r['t_trunk'] is None and r['hips_min'] > 0.5:
        extra = ('nada no chao' if not r['ground'] else 'tocou: ' + ','.join(r['ground']))
        rec = ' '.join(f"{k / RATE:.2f}" for k in r['after']) or 'nenhum'
        return (f"{head} | SEM QUEDA passos {r['steps']} tronco ate {r['tilt']:.0f} graus em {r['k_tilt'] / RATE:.2f}s, "
                f"pisadas no 1 s seguinte: {rec} | {extra} "
                f"| desce {r['v_com']:.2f} m/s | abaixo {r['below'] * 100:.1f} cm")
    vt = '  -  ' if r['v_trunk'] is None else f"{-r['v_trunk']:.2f}"
    order = ' '.join(f"{n}@{k / RATE:.2f}" for k, n in r['before']) or 'nenhum'
    return (f"{head} | tronco {fmt_t(r['t_trunk'])} (desce {vt} m/s) cabeca {fmt_t(r['t_head'])} "
            f"| antes do tronco: {order} | pico desce {r['v_com']:.2f} m/s | abaixo {r['below'] * 100:.1f} cm")


if __name__ == '__main__':
    for p in [a for a in sys.argv[1:] if not a.startswith('-')]:
        r = analyze(p)
        print(line(r))
        if '-v' in sys.argv:
            print('     ' + ' '.join(f"{n} {fmt_t(k)}" for n, k in r['touch'].items()))
