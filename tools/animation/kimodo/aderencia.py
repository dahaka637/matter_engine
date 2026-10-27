"""O quanto uma amostra do Kimodo bate com as poses-chave que recebeu: em cada quadro de chave fullbody, a distancia
media e a pior das juntas com par (em volta do quadril, cm) e o tronco da amostra contra o da chave.

uso: aderencia.py constraints.json amostra.npz [...]
"""
import json
import sys

import numpy as np

import sprint_chaves as sc
from soma import IX, NAMES

JOINTS = [IX[n] for n in ("Spine2", "Chest", "Neck1", "Head", "LeftArm", "LeftForeArm", "LeftHand", "RightArm",
                          "RightForeArm", "RightHand", "LeftShin", "LeftFoot", "LeftToeBase", "RightShin", "RightFoot",
                          "RightToeBase")]


def lean(p):
    v = p[IX["Neck1"]] - p[IX["Hips"]]
    return np.degrees(np.arctan2(v[2], v[1]))


def exp_map(v):
    a = np.linalg.norm(v)
    if a < 1e-9:
        return np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]) / a
    return np.eye(3) + np.sin(a) * k + (1 - np.cos(a)) * k @ k


def main():
    constraints, samples = sys.argv[1], sys.argv[2:]
    parents, offsets = sc.skeleton()
    keys = []
    for group in json.load(open(constraints)):
        if group["type"] != "fullbody":
            continue
        for k, rot, root in zip(group["frame_indices"], group["local_joints_rot"], group["root_positions"]):
            local = np.array([exp_map(np.array(r)) for r in rot])
            keys.append((k, sc.forward_kinematics(parents, offsets, local, np.array(root))))
    for path in samples:
        P = np.load(path)["posed_joints"]
        errors, worst, leans = [], 0.0, []
        for k, ref in keys:
            a = P[k][JOINTS] - P[k][IX["Hips"]]
            b = ref[JOINTS] - ref[IX["Hips"]]
            d = np.linalg.norm(a - b, axis=1) * 100
            errors.append(d.mean())
            worst = max(worst, d.max())
            leans.append((lean(P[k]), lean(ref)))
        ls = np.array(leans)
        print(f"{path.rsplit('/', 1)[-1]:18s} chaves {len(keys)}: erro medio {np.mean(errors):5.1f} cm, pior junta {worst:5.1f} cm;"
              f" tronco nas chaves {ls[:, 0].mean():5.1f} graus (pedido {ls[:, 1].mean():.1f})")


if __name__ == "__main__":
    main()
