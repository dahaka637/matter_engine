#!/usr/bin/env python3
"""O canto da gola com o braco erguido (25/10/2026).

Junto do canto da gola, a camisa pesava quase so no peito e no pescoco, e o
ombro logo ao lado na clavicula: com o braco erguido o ombro sobe e o canto
fica, e a tira escondida sob a camisa dobrava (a pintinha magenta no alto do
ombro). Os vertices so da camisa (fora da faixa e da barra) a menos de RAIO do
canto da gola tem os pesos alisados com os vizinhos, PASSOS vezes; o esquerdo
e o espelho do direito. Depois vem o desvirar, com as regras de sempre
(--braco-perto camisa).

    python3 tools/blender/alisar_canto_gola.py <entrada.troca.json> <saida.troca.json>
"""
import json
import sys

import numpy as np
from scipy.spatial import cKDTree

RAIO = 0.05
PASSOS = 2


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
    parts = [p["name"] for p in data["parts"]]
    shirt = parts.index("camisa")
    tris = np.array(data["triangles"])
    part_of = np.array(data["trianglePart"])
    # A faixa: as faces da camisa que tocam a costura com o resto; a barra: os vertices delas.
    in_shirt, other = np.zeros(n, bool), np.zeros(n, bool)
    for t, p in zip(tris, part_of):
        (in_shirt if p == shirt else other)[t] = True
    seam = in_shirt & other
    strip = np.zeros(n, bool)
    for t, p in zip(tris, part_of):
        if p == shirt and seam[t].any():
            strip[t] = True
    neighbors = [set() for _ in range(n)]
    for t, p in zip(tris, part_of):
        if p == shirt:
            for k in range(3):
                neighbors[t[k]].update((int(t[(k + 1) % 3]), int(t[(k + 2) % 3])))
    # O canto da gola do lado direito: o vertice da barra da gola mais para o lado (a gola e a barra mais alta).
    hem = np.nonzero(strip & ~seam & in_shirt)[0]
    collar = hem[pos[hem, 2] > pos[hem, 2].max() - 0.08]
    corner = pos[collar[np.argmin(pos[collar, 1])]]
    chosen = [v for v in np.nonzero(in_shirt & ~strip)[0] if pos[v, 1] < 0.0 and np.linalg.norm(pos[v] - corner) < RAIO]
    if not chosen:
        sys.exit("nenhum vertice perto do canto da gola")
    _, mirror = cKDTree(pos).query(pos * np.array([1.0, -1.0, 1.0]))
    swap = lambda s: "Left" + s[5:] if s.startswith("Right") else "Right" + s[4:] if s.startswith("Left") else s
    perm = np.array([index.get(swap(name), i) for i, name in enumerate(names)])
    for _ in range(PASSOS):
        smoothed = {v: 0.5 * weights[v] + 0.5 * np.mean([weights[u] for u in neighbors[v]], axis=0) for v in chosen}
        for v, w in smoothed.items():
            weights[v] = w / w.sum()
            weights[mirror[v]] = weights[v][perm]
    out = []
    for v in range(n):
        top = [i for i in np.argsort(-weights[v])[:4] if weights[v, i] > 1e-4]
        total = sum(weights[v, i] for i in top)
        out.append([[int(i), float(weights[v, i] / total)] for i in top])
    data["weights"] = out
    json.dump(data, open(saida, "w"))
    print(f"CANTO DA GOLA: {len(chosen)} vertices de cada lado alisados ({PASSOS} passos), canto em {np.round(corner, 3).tolist()}: {sorted(int(v) for v in chosen)}")


main()
