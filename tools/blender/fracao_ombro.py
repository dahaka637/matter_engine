#!/usr/bin/env python3
"""Os pesos do ombro em fracoes do giro do braco (25/10/2026).

Cada vertice dos aneis da manga (contados a partir da costura do punho com a
pele, sem a costura da cava) com peso de meia-junta ou de braco tem uma fracao
efetiva do giro do braco: peito 0, meia-junta (RightShoulderHelper) 0,5 e braco 1. Aqui
essa parte dos pesos e reescrita nas duas juntas de fracao vizinhas, com a
mesma fracao: nos giros pequenos o skin fica quase igual (o avesso das poses do
jogo nao muda), mas o peito nao se mistura mais direto com o braco. Junto com a
rampa da meia-junta no GameRig (50% ate 60 graus acima da ligacao, 65% na
vertical), a metade de baixo da manga acompanha o braco erguido, que antes
parecia dobrar no punho dela. O esquerdo e o espelho do direito; depois vem o
axila_ombro.py (o lado de baixo do ombro na junta da axila) e o desvirar
(--braco-perto camisa).

    python3 tools/blender/fracao_ombro.py <entrada.troca.json> <saida.troca.json>
"""
import json
import sys
from collections import deque

import numpy as np
from scipy.spatial import cKDTree


# Os aneis da manga (camadas a partir da costura do punho): barra 1, apoio 2 e
# os aneis 3 a 5. A costura da cava (6, o fundo dela e a axila) e o tronco ficam
# de fora: remapeada, a axila saia com o braco erguido e a lateral do tronco
# fazia um S; com o tronco, o braco a frente esticava a lateral 2,6 vezes mais.
CAMADAS = 5


def sleeve_rings(data, pos):
    """As camadas da busca em largura pela camisa a partir da costura do punho direito com a pele."""
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
        if layer[v] >= CAMADAS:
            continue
        for u in neighbors.get(v, ()):
            if u not in layer:
                layer[u] = layer[v] + 1
                queue.append(u)
    return layer


def main():
    entrada, saida = sys.argv[1], sys.argv[2]
    data = json.load(open(entrada))
    names = [j["name"] for j in data["joints"]]
    index = {name: i for i, name in enumerate(names)}
    pos = np.array(data["positions"])
    n = len(pos)
    weights = np.zeros((n, len(names)))
    for v, pairs in enumerate(data["weights"]):
        for j, w in pairs:
            weights[v, j] = w
    levels = [index["UpperChest"], index["RightShoulderHelper"], index["RightUpperArm"]]
    share = np.array([0.0, 0.5, 1.0])
    changed = 0
    rings = sleeve_rings(data, pos)
    for v in range(n):
        if pos[v, 1] > -1e-6 or v not in rings:
            continue
        mass = weights[v, levels].sum()
        if mass < 1e-6 or weights[v, levels[1:]].sum() < 1e-6:
            continue
        f = (weights[v, levels] * share).sum() / mass
        k = 0 if f <= 0.5 else 1
        t = (f - share[k]) / (share[k + 1] - share[k])
        weights[v, levels] = 0.0
        weights[v, levels[k]] = mass * (1.0 - t)
        weights[v, levels[k + 1]] = mass * t
        changed += 1
    _, mirror = cKDTree(pos).query(pos * np.array([1.0, -1.0, 1.0]))
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
    print(f"FRACAO DO OMBRO: {changed} vertices de cada lado")


main()
