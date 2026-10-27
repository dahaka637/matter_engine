"""Poses-chave de corrida com o tronco mais inclinado, a partir de uma amostra do Kimodo.

Pega o quadro de cada pisada (o pe chegando a 3 cm do chao) da amostra, inclina a coluna para a frente mais `graus`
(divididos entre Spine1, Spine2 e Chest, em volta do eixo lateral de cada pai; o sinal e conferido pela
cinematica direta), devolve no pescoco a fracao `cabeca` (o olhar nao vai para o chao) e grava as poses como
restricao `fullbody` nos mesmos quadros, com a raiz na linha reta da velocidade (x 0, z = v * t, a altura do
quadro; sem --velocidade, a raiz da propria amostra). Junto com o caminho do lote.py, o modelo preenche o resto.

uso: sprint_chaves.py amostra.npz saida.json [--velocidade 10] --graus 12 [--cabeca 0.5] [--de 0.3]
"""
import json
import re
import sys

import numpy as np

from soma import IX, NAMES

FPS = 30.0
BVH = __import__("os").environ.get("KIMODO", "/home/dahaka/Projetos/kimodo") + \
    "/kimodo/assets/skeletons/somaskel77/somaskel77_standard_tpose.bvh"


def skeleton():
    """Pais e offsets (m) das 77 juntas do BVH da pose T padrao (sem a junta Root)."""
    parents, offsets, stack, names = [], [], [], []
    for line in open(BVH):
        token = line.split()
        if not token:
            continue
        if token[0] in ("ROOT", "JOINT"):
            names.append(token[1])
            parents.append(stack[-1] if stack else -1)
        elif token[0] == "End":
            names.append(None)
            parents.append(stack[-1])
        elif token[0] == "OFFSET":
            offsets.append([float(v) / 100.0 for v in token[1:4]])
        elif token[0] == "{":
            stack.append(len(names) - 1)
        elif token[0] == "}":
            stack.pop()
        elif token[0] == "MOTION":
            break
    keep = [i for i, n in enumerate(names) if n is not None and n != "Root"]
    index = {old: new for new, old in enumerate(keep)}
    out_parents = [index.get(parents[i], -1) for i in keep]
    assert [names[i] for i in keep] == NAMES
    return out_parents, np.array([offsets[i] for i in keep])


def forward_kinematics(parents, offsets, local, root):
    rot = np.zeros_like(local)
    pos = np.zeros((len(parents), 3))
    for j, p in enumerate(parents):
        if p < 0:
            rot[j], pos[j] = local[j], root
        else:
            rot[j] = rot[p] @ local[j]
            pos[j] = pos[p] + rot[p] @ offsets[j]
    return pos


def rot_x(degrees):
    a = np.radians(degrees)
    return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])


def axis_angle(m):
    c = np.clip((np.trace(m) - 1) / 2, -1, 1)
    a = np.arccos(c)
    if a < 1e-6:
        return np.zeros(3)
    v = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    s = np.linalg.norm(v)
    if s < 1e-6:
        w, V = np.linalg.eigh((m + np.eye(3)) / 2)
        return V[:, np.argmax(w)] * a
    return v / s * a


def trunk_lean(pos):
    v = pos[IX["Neck1"]] - pos[IX["Hips"]]
    return np.degrees(np.arctan2(v[2], v[1]))


def leaned(local, extra, give_back):
    out = local.copy()
    for name in ("Spine1", "Spine2", "Chest"):
        out[IX[name]] = rot_x(extra / 3.0) @ out[IX[name]]
    out[IX["Neck1"]] = rot_x(-extra * give_back) @ out[IX["Neck1"]]
    return out


def main():
    args = sys.argv[1:]
    opts = {"--velocidade": None, "--graus": "12", "--cabeca": "0.5", "--de": "0.3"}
    for key in list(opts):
        if key in args:
            i = args.index(key)
            opts[key] = args[i + 1]
            del args[i:i + 2]
    source, out = args
    speed, extra, give_back, start = (opts["--velocidade"] and float(opts["--velocidade"]), float(opts["--graus"]),
                                      float(opts["--cabeca"]), float(opts["--de"]))
    data = np.load(source)
    parents, offsets = skeleton()
    local, roots, joints = data["local_rot_mats"], data["root_positions"], data["posed_joints"]
    error = np.abs(forward_kinematics(parents, offsets, local[0], roots[0]) - joints[0]).max()
    assert error < 0.01, f"cinematica direta nao bate com o Kimodo ({error:.3f} m)"
    # o sinal que inclina para a frente (rumo +Z): conferido no primeiro quadro
    base = trunk_lean(joints[0])
    sign = 1.0 if trunk_lean(forward_kinematics(parents, offsets, leaned(local[0], 5.0, 0.0), roots[0])) > base else -1.0
    # as pisadas pela geometria (o rotulo de contato do modelo some nas corridas rapidas): o pe a 3 cm do chao
    frames = []
    for side in ("Left", "Right"):
        low = np.minimum(joints[:, IX[f"{side}ToeBase"], 1], joints[:, IX[f"{side}Foot"], 1])
        c = low < np.percentile(low, 2) + 0.03
        frames += list(np.flatnonzero(c[1:] & ~c[:-1]) + 1)
    frames = sorted(int(k) for k in frames if k >= start * FPS)
    assert 0 < len(frames) < 20, f"{len(frames)} pisadas (o Kimodo aceita menos de 20 chaves)"
    group = {"type": "fullbody", "frame_indices": [], "local_joints_rot": [], "root_positions": [], "smooth_root_2d": []}
    before, after = [], []
    for k in frames:
        pose = leaned(local[k], sign * extra, give_back)
        root = ([0.0, float(roots[k][1]), speed * k / FPS] if speed else
                [float(roots[k][0] - roots[0][0]), float(roots[k][1]), float(roots[k][2] - roots[0][2])])
        before.append(trunk_lean(joints[k]))
        after.append(trunk_lean(forward_kinematics(parents, offsets, pose, roots[k])))
        group["frame_indices"].append(int(k))
        group["local_joints_rot"].append([axis_angle(m).tolist() for m in pose])
        group["root_positions"].append(root)
        group["smooth_root_2d"].append([root[0], root[2]])
    json.dump([group], open(out, "w"))
    print(f"{len(frames)} chaves {frames}; tronco nas pisadas {np.mean(before):.1f} -> {np.mean(after):.1f} graus "
          f"(sinal {sign:+.0f}, cinematica {error * 1000:.1f} mm)")


if __name__ == "__main__":
    main()
