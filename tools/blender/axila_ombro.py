#!/usr/bin/env python3
"""O lado de baixo do ombro na junta da axila (26/10/2026).

Com o braco erguido, o lado de baixo do ombro (a axila e as dobras da frente e
de tras) rodava com a meia-junta (RightShoulderHelper, 65% do giro na
vertical) em volta do ombro, a distancia fixa dele: o contorno embaixo do braco
inchava para fora e a manga ficava longe do braco. Do lado de cima, o alto da
manga junto do ombro ficava para tras e o braco afinava ate o ombro (o lado da
cabeca). A junta da axila (RightArmpitHelper, no GameRig) gira como a meia-junta
ate 60 graus acima da ligacao e com o braco inteiro na vertical. Aqui a parte da
meia-junta de cada vertice passa para ela: no lado de baixo, toda ate FI_TODO
graus da direcao de baixo em volta do braco (na ligacao), nada a partir de
FI_NADA, e so ate MEIO_TODO-MEIO_NADA para dentro do ombro (o peito e a gola
ficam); no lado de cima (de SETOR_NADA a SETOR_TODO graus da direcao de baixo),
de ALEM_NADA a ALEM_TODO alem da junta ao longo do braco (a manga; o alto do
ombro e a gola ficam; na frente e atras a manga pela metade virava triangulos
no levantar do chao). E o fundo da axila (o anel da cava e os de baixo dele, no
tronco, ate VOLUME_RAIO do centro) passa parte do peso de peito para a junta do
volume (RightArmpitFill), que fica com o peito e anda um pouco para fora com o
braco erguido: a axila ganha volume em vez de um plano liso. Num vertice
que ja tem 4 pesos ela passa inteira ou nao passa (com 5, o corte mudaria o
skin com o braco baixo). Com o braco baixo as duas giram igual e o skin das
poses do jogo nao muda. O esquerdo e o espelho do direito; as regras do
desvirar (--braco-perto camisa) vem depois.

    python3 tools/blender/axila_ombro.py <entrada.troca.json> <saida.troca.json>
"""
import copy
import json
import sys
from collections import deque

import numpy as np
from scipy.spatial import cKDTree

FI_TODO, FI_NADA = 60.0, 110.0
MEIO_TODO, MEIO_NADA = 0.04, 0.08
ALEM_NADA, ALEM_TODO = -0.02, 0.03
SETOR_NADA, SETOR_TODO = 90.0, 130.0
# O centro do fundo da axila (em relacao a junta do ombro: para dentro, para baixo) e o raio do volume.
VOLUME_DENTRO, VOLUME_ABAIXO, VOLUME_RAIO = 0.019, 0.08, 0.15


def axilla_share(pos, shoulder, elbow):
    """1 no lado da axila em volta do braco (ate FI_TODO graus da direcao de baixo), 0 a partir de FI_NADA,
    so perto do ombro para dentro (o ombro direito: para dentro e y maior); e 1 no alto da manga alem da junta."""
    along = (elbow - shoulder) / np.linalg.norm(elbow - shoulder)
    up = np.array([0.0, 0.0, 1.0]) - along[2] * along
    up /= np.linalg.norm(up)
    radial = pos - shoulder
    radial -= (radial @ along)[:, None] * along
    cosine = (radial @ -up) / np.maximum(np.linalg.norm(radial, axis=1), 1e-9)
    angle = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))
    t = np.clip((angle - FI_TODO) / (FI_NADA - FI_TODO), 0.0, 1.0)
    inward = np.clip((pos[:, 1] - shoulder[1] - MEIO_TODO) / (MEIO_NADA - MEIO_TODO), 0.0, 1.0)
    below = (1.0 - t * t * (3.0 - 2.0 * t)) * (1.0 - inward * inward * (3.0 - 2.0 * inward))
    # O alto da manga, alem da junta: segue o braco erguido (sem ele o lado da cabeca afinava ate o ombro).
    s = np.clip(((pos - shoulder) @ along - ALEM_NADA) / (ALEM_TODO - ALEM_NADA), 0.0, 1.0)
    q = np.clip((angle - SETOR_NADA) / (SETOR_TODO - SETOR_NADA), 0.0, 1.0)
    sleeve = s * s * (3.0 - 2.0 * s) * q * q * (3.0 - 2.0 * q)
    return 1.0 - (1.0 - below) * (1.0 - sleeve)


def torso_rings(data, pos):
    """Os vertices da camisa do anel da cava para dentro do tronco (a busca em largura a partir do punho direito, camada 6 ou mais)."""
    parts = [p["name"] for p in data["parts"]]
    shirt, body = parts.index("camisa"), parts.index("corpo")
    in_shirt, in_body, neighbors = set(), set(), {}
    for t, p in zip(data["triangles"], data["trianglePart"]):
        (in_shirt if p == shirt else in_body if p == body else set()).update(t)
        if p == shirt:
            for k in range(3):
                neighbors.setdefault(t[k], set()).update((t[(k + 1) % 3], t[(k + 2) % 3]))
    cuff = [v for v in in_shirt & in_body if pos[v, 1] < -0.2 and 0.2 < pos[v, 2] < 0.42]
    layer = {v: 0 for v in cuff}
    queue = deque(cuff)
    while queue:
        v = queue.popleft()
        if layer[v] >= 6:
            continue
        for u in neighbors.get(v, ()):
            if u not in layer:
                layer[u] = layer[v] + 1
                queue.append(u)
    return np.array([v in in_shirt and layer.get(v, 99) >= 6 for v in range(len(pos))])


def main():
    entrada, saida = sys.argv[1], sys.argv[2]
    data = json.load(open(entrada))
    joints = data["joints"]
    for side in ("Right", "Left"):
        if not any(j["name"] == side + "ArmpitHelper" for j in joints):
            entry = copy.deepcopy(next(j for j in joints if j["name"] == side + "ShoulderHelper"))
            entry["name"] = side + "ArmpitHelper"
            joints.append(entry)
        if not any(j["name"] == side + "ArmpitFill" for j in joints):
            entry = copy.deepcopy(next(j for j in joints if j["name"] == side + "ShoulderHelper"))
            entry["name"] = side + "ArmpitFill"
            entry["helper"]["fraction"] = 0.0
            joints.append(entry)
    names = [j["name"] for j in joints]
    index = {name: i for i, name in enumerate(names)}
    pos = np.array(data["positions"])
    n = len(pos)
    weights = np.zeros((n, len(names)))
    for v, pairs in enumerate(data["weights"]):
        for j, w in pairs:
            weights[v, j] = w
    helper, armpit = index["RightShoulderHelper"], index["RightArmpitHelper"]
    share = axilla_share(pos, np.array(joints[index["RightUpperArm"]]["position"]), np.array(joints[index["RightForearm"]]["position"]))
    right = pos[:, 1] < -1e-6
    full = (weights > 1e-4).sum(1) >= 4
    share = np.where(full, (share >= 0.5).astype(float), share)
    moved = right & (weights[:, helper] > 1e-6) & (share > 1e-6)
    weights[moved, armpit] += weights[moved, helper] * share[moved]
    weights[moved, helper] *= 1.0 - share[moved]
    # O volume: o peso de peito do fundo da axila (a junta do volume e o peito com o braco baixo; 4 pesos, inteiro ou nada).
    shoulder = np.array(joints[index["RightUpperArm"]]["position"])
    center = shoulder + np.array([0.0, VOLUME_DENTRO, -VOLUME_ABAIXO])
    fill_share = np.clip(1.0 - np.linalg.norm(pos - center, axis=1) / VOLUME_RAIO, 0.0, 1.0) * torso_rings(data, pos) * right
    full = (weights > 1e-4).sum(1) >= 4
    fill_share = np.where(full, (fill_share >= 0.5).astype(float), fill_share)
    chest, fill = index["UpperChest"], index["RightArmpitFill"]
    weights[:, fill] += weights[:, chest] * fill_share
    weights[:, chest] *= 1.0 - fill_share
    distance, mirror = cKDTree(pos).query(pos * np.array([1.0, -1.0, 1.0]))
    if np.max(distance) > 1e-5:
        sys.exit("a malha nao e simetrica")
    swap = lambda s: "Left" + s[5:] if s.startswith("Right") else "Right" + s[4:] if s.startswith("Left") else s
    perm = np.array([index.get(swap(name), i) for i, name in enumerate(names)])
    for v in range(n):
        if pos[v, 1] > 1e-6:
            weights[v] = weights[mirror[v]][perm]
    out = []
    for v in range(n):
        top = [i for i in np.argsort(-weights[v])[:4] if weights[v, i] > 1e-4]
        total = sum(weights[v, i] for i in top)
        out.append([[int(i), float(weights[v, i] / total)] for i in top])
    data["weights"] = out
    json.dump(data, open(saida, "w"))
    print(f"AXILA DO OMBRO: {int(moved.sum())} vertices de cada lado passam a meia-junta para a junta da axila (o lado de baixo e o alto da manga); {int((fill_share > 1e-6).sum())} no volume")


main()
