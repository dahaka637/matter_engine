"""Poses-chave para o Kimodo a partir de quadros de geracoes anteriores.

uso: chave.py saida.json fonte.npz:quadro:alvo[:dx,dz] [...] [--tipo fullbody]
  fonte.npz:quadro  - o quadro de uma geracao (rotacoes locais e raiz)
  alvo              - o quadro do clipe novo onde a pose entra (-1 = ultimo,
                      precisa de --total)
  dx,dz             - onde a raiz fica no chao (relativo a origem do clipe
                      novo); sem isso, a do quadro fonte relativa ao quadro 0
"""
import json, sys, numpy as np

def axis_angle(R):
    """(J,3,3) -> (J,3) eixo-angulo."""
    out = np.zeros((len(R), 3))
    for i, m in enumerate(R):
        c = np.clip((np.trace(m) - 1) / 2, -1, 1)
        a = np.arccos(c)
        if a < 1e-6:
            continue
        v = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
        s = np.linalg.norm(v)
        if s < 1e-6:  # ~180 graus
            w, V = np.linalg.eigh((m + np.eye(3)) / 2)
            v = V[:, np.argmax(w)]
            out[i] = v / np.linalg.norm(v) * a
        else:
            out[i] = v / s * a
    return out

args = [a for a in sys.argv[1:] if not a.startswith('--')]
total = None
if '--total' in sys.argv:
    total = int(sys.argv[sys.argv.index('--total') + 1])
    args = [a for a in args if a != str(total)]
kind = sys.argv[sys.argv.index('--tipo') + 1] if '--tipo' in sys.argv else 'fullbody'
if kind != 'fullbody':
    args = [a for a in args if a != kind]
out = args[0]
groups = {}
for spec in args[1:]:
    parts = spec.split(':')
    src, frame, target = parts[0], int(parts[1]), int(parts[2])
    if target < 0:
        target = total + target
    d = np.load(src)
    rot = axis_angle(d['local_rot_mats'][frame])
    root = d['root_positions'][frame].astype(float).copy()
    if len(parts) > 3:
        dx, dz = map(float, parts[3].split(','))
        root[0], root[2] = dx, dz
    else:
        root[0] -= d['root_positions'][0][0]
        root[2] -= d['root_positions'][0][2]
    g = groups.setdefault(kind, {'type': kind, 'frame_indices': [], 'local_joints_rot': [], 'root_positions': [], 'smooth_root_2d': []})
    g['frame_indices'].append(target)
    g['local_joints_rot'].append(rot.tolist())
    g['root_positions'].append(root.tolist())
    g['smooth_root_2d'].append([root[0], root[2]])
json.dump(list(groups.values()), open(out, 'w'))
print('ok', out, {k: v['frame_indices'] for k, v in groups.items()})
