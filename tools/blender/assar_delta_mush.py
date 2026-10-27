#!/usr/bin/env python3
"""O Delta Mush assado nos pesos lineares (22/10/2026).

Nos jogos, a tecnica de producao para as dobras de compressao do skinning
linear: calcular a deformacao lisa do Delta Mush (que nao colapsa nem dobra) nas
poses reais do jogo e ajustar, vertice a vertice, os pesos lineares (ate 4
ossos) que melhor a reproduzem. O resultado roda no shader de sempre.

    python3 tools/blender/assar_delta_mush.py <jogador.troca.json> <jogador.poses.json> <saida.troca.json>
        [--rodadas 15] [--pecas camisa,calcao] [--atual 0.02] [--vizinhos 0.05]

O troca vem do .blend (tools/blender/exportar_troca.py) e as poses do
MatterModelInspector exportar-blender --poses (o skin de cada junta, com os
auxiliares pelas regras do GameRig: o que o jogo faz). So as poses de treino
entram (as de "teste" ficam para a checagem). Ajusta os vertices livres das
pecas (as faixas do recuo e a borda delas ficam paradas); o resto da malha so
entra no alisamento.
"""
import argparse
import json
import sys

import numpy as np
from scipy.optimize import nnls

import proporcao_calcao
import proporcao_tronco


def quat_rotate(q, v):
    """q (N, 4) xyzw; v (N, 3) ou (3,)."""
    imaginary = q[..., :3]
    twice = 2.0 * np.cross(imaginary, v)
    return v + q[..., 3:4] * twice + np.cross(imaginary, twice)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("troca")
    p.add_argument("poses")
    p.add_argument("saida")
    p.add_argument("--rodadas", type=int, default=15)
    p.add_argument("--passo", type=float, default=0.5)
    p.add_argument("--pecas", default="camisa,calcao")
    p.add_argument("--atual", type=float, default=0.02)
    p.add_argument("--vizinhos", type=float, default=0.05)
    p.add_argument("--candidatos", type=int, default=8)
    p.add_argument("--costura", type=float, default=0.04, help="metros em volta das faixas que ficam parados")
    p.add_argument("--limiar", type=float, default=0.003, help="so ajusta onde o alvo se afasta do linear mais que isso")
    p.add_argument("--levante", type=float, default=0.0, help="parados os vertices da camisa com mais que isso de HipLift (a barra que a coxa levanta)")
    p.add_argument("--sem-coxa", default="", help="pecas cujos vertices nao recebem as juntas da coxa (a camisa so no tronco)")
    p.add_argument("--braco-perto", default="", help="pecas em que o braco (e a clavicula) so pesa perto dele (proporcao_tronco.arm_allowance)")
    p.add_argument("--calcao-coxa", action="store_true",
                   help="a parede de dentro da perna do calcao com o piso do peso da coxa (proporcao_calcao, etapa 07)")
    p.add_argument("--espelho", action="store_true", help="ajusta a direita e o meio (pares iguais) e espelha na esquerda")
    p.add_argument("--desvirar", type=int, default=0,
                   help="rodadas que alisam os pesos dos triangulos das pecas que viram (contra o alvo) em alguma pose de treino")
    p.add_argument("--desvirar-pecas", default="camisa", help="as pecas que o --desvirar alisa (no calcao, alisar junta as dobras da virilha)")
    a = p.parse_args()

    data = json.load(open(a.troca))
    poses = [pose for pose in json.load(open(a.poses))["poses"] if not pose["test"]]
    rest = np.array(data["positions"], dtype=np.float64)
    tris = np.array(data["triangles"], dtype=np.int64)
    n = len(rest)
    joints = len(data["joints"])
    weights = np.zeros((n, joints))
    for v, pairs in enumerate(data["weights"]):
        for j, w in pairs:
            weights[v, j] = w
    # Os vizinhos (pelas arestas dos triangulos).
    neighbors = [set() for _ in range(n)]
    for t in tris:
        for k in range(3):
            neighbors[t[k]].update((t[(k + 1) % 3], t[(k + 2) % 3]))
    neighbors = [np.array(sorted(s)) for s in neighbors]
    degree = np.array([max(1, len(s)) for s in neighbors])
    rows = np.concatenate([np.full(len(s), i) for i, s in enumerate(neighbors)])
    cols = np.concatenate(neighbors)
    from scipy.sparse import csr_matrix
    average = csr_matrix((1.0 / degree[rows], (rows, cols)), shape=(n, n))

    def smooth(x):
        for _ in range(a.rodadas):
            x = x + a.passo * (average @ x - x)
        return x

    def frames(x):
        """O referencial em cada vertice da malha alisada: normal (das faces) e tangente (para o primeiro vizinho)."""
        e1 = x[tris[:, 1]] - x[tris[:, 0]]
        e2 = x[tris[:, 2]] - x[tris[:, 0]]
        fn = np.cross(e1, e2)
        normal = np.zeros_like(x)
        for k in range(3):
            np.add.at(normal, tris[:, k], fn)
        normal /= np.maximum(1e-12, np.linalg.norm(normal, axis=1))[:, None]
        first = np.array([s[0] if len(s) else i for i, s in enumerate(neighbors)])
        tangent = x[first] - x
        tangent -= normal * np.sum(tangent * normal, axis=1)[:, None]
        tangent /= np.maximum(1e-12, np.linalg.norm(tangent, axis=1))[:, None]
        return tangent, np.cross(normal, tangent), normal

    # O delta do repouso no referencial alisado.
    smooth_rest = smooth(rest)
    t0, b0, n0 = frames(smooth_rest)
    offset = rest - smooth_rest
    delta = np.stack([np.sum(offset * t0, 1), np.sum(offset * b0, 1), np.sum(offset * n0, 1)], 1)

    # O skin de cada junta em cada pose e o alvo do Delta Mush.
    skins = []
    targets = []
    linear_all = []
    for pose in poses:
        s = np.array(pose["skin"], dtype=np.float64)  # (joints, 7): posicao + giro xyzw
        skins.append(s)
        # Todas as juntas aplicadas a todos os vertices seria (n, joints, 3); so as com peso.
        posed = np.zeros_like(rest)
        for j in np.nonzero(weights.sum(0))[0]:
            w = weights[:, j]
            mask = w > 0.0
            moved = s[j, :3] + quat_rotate(np.broadcast_to(s[j, 3:], (mask.sum(), 4)), rest[mask])
            posed[mask] += w[mask, None] * moved
        linear_all.append(posed)
        smooth_pose = smooth(posed)
        t, b, nn = frames(smooth_pose)
        targets.append(smooth_pose + delta[:, 0:1] * t + delta[:, 1:2] * b + delta[:, 2:3] * nn)
    targets = np.stack(targets)  # (poses, n, 3)
    skins = np.stack(skins)  # (poses, joints, 7)

    # Os vertices ajustados: os livres das pecas.
    names = [part["name"] for part in data["parts"]]
    pieces = {names.index(name) for name in a.pecas.split(",")}
    part_of = np.array(data["trianglePart"])
    in_piece = np.zeros(n, bool)
    other = np.zeros(n, bool)
    for t, pp in zip(tris, part_of):
        if pp in pieces:
            in_piece[t] = True
        else:
            other[t] = True
    seam = in_piece & other
    strip = np.zeros(n, bool)
    for t, pp in zip(tris, part_of):
        if pp in pieces and seam[t].any():
            strip[t] = True
    # Parados tambem os vertices perto das faixas (o punho, a barra sobre o
    # calcao, a gola: alisados juntos com a pele, eles atravessavam o corpo) e
    # os que o Delta Mush quase nao muda (so as zonas de compressao mudam).
    from scipy.spatial import cKDTree
    near_seam = np.zeros(n, bool)
    if strip.any():
        dist, _ = cKDTree(rest[strip]).query(rest)
        near_seam = dist < a.costura
    gap = np.max(np.linalg.norm(targets - np.stack(linear_all), axis=2), axis=0)
    fit = np.nonzero(in_piece & ~strip & ~near_seam & (gap > a.limiar))[0]
    # Com --levante: a barra da camisa que a coxa levanta fica como foi
    # desenhada (o alvo alisado a puxava para a barriga: HipLift 0,75 -> 0,62 na
    # borda e a coxa a 110 graus atravessava a barra).
    if a.levante > 0.0:
        lift_joints = [i for i, j in enumerate(data["joints"]) if j["name"].endswith("HipLift")]
        shirt_index = [part["name"] for part in data["parts"]].index("camisa")
        shirt_mask = np.zeros(n, bool)
        for t, pp in zip(tris, part_of):
            if pp == shirt_index:
                shirt_mask[t] = True
        held = shirt_mask & (weights[:, lift_joints].sum(1) > a.levante)
        fit = fit[~held[fit]]
        print(f"barra levantada parada: {int(held.sum())} vertices")
    # Com --espelho: so a direita (y < 0) e o meio; no meio, cada junta da
    # esquerda/direita vem com o par dela (o mesmo peso nas duas).
    jnames = [j["name"] for j in data["joints"]]
    def swap(name):
        return "Right" + name[4:] if name.startswith("Left") else "Left" + name[5:] if name.startswith("Right") else name
    partner_joint = np.array([jnames.index(swap(nm)) if swap(nm) in jnames else i for i, nm in enumerate(jnames)])
    mirror_of = None
    if a.espelho:
        mirror_of = cKDTree(rest).query(rest * np.array([1.0, -1.0, 1.0]))
        if np.max(mirror_of[0][in_piece]) > 1e-5:
            sys.exit("a malha das pecas nao e simetrica")
        mirror_of = mirror_of[1]
        fit = fit[rest[fit, 1] <= 1e-6]
    print(f"poses de treino {len(poses)}; vertices ajustados {len(fit)} (paradas as faixas: {strip.sum()})")

    # Com --sem-coxa: os vertices dessas pecas nao recebem as juntas da coxa.
    thigh_joints = [i for i, nm in enumerate(jnames)
                    if nm.endswith(("HipLift", "HipHelper", "HipHelper25", "HipHelper75", "Thigh", "GluteHelper"))]
    banned = np.zeros(n, bool)
    for nm in filter(None, a.sem_coxa.split(",")):
        k = names.index(nm)
        for t, pp in zip(tris, part_of):
            if pp == k:
                banned[t] = True
    # Com --braco-perto: o peso do braco nessas pecas cortado pela distancia ao braco (zero no
    # peito, na barriga e nas costas); onde a fracao e zero, as juntas do braco nem entram.
    arm_joints = proporcao_tronco.arm_joints(jnames)
    arm_rows = np.zeros(n, bool)
    for nm in filter(None, a.braco_perto.split(",")):
        k = names.index(nm)
        for t, pp in zip(tris, part_of):
            if pp == k:
                arm_rows[t] = True
    arm_rows = np.nonzero(arm_rows)[0]
    allowance = np.ones(n)
    if len(arm_rows):
        allowance = proporcao_tronco.arm_allowance(rest, {j["name"]: np.array(j["position"]) for j in data["joints"]})
        weights_cut = proporcao_tronco.limit_arm(weights, arm_rows, allowance, arm_joints, jnames.index("UpperChest"))
        print(f"braco perto: {len(arm_rows)} vertices; {weights_cut} cortados na entrada")
    no_arm = np.zeros(n, bool)
    no_arm[arm_rows] = allowance[arm_rows] <= 1e-6

    # Com --calcao-coxa: a parede de dentro da perna do calcao segue a coxa (o piso da etapa 07; senao o ajuste ao
    # Delta Mush devolve a cortina entre as pernas abertas).
    floor_rows = (np.array([], dtype=int), np.array([]), np.array([], dtype=int))
    front_rows = (np.array([], dtype=int), np.array([]), (0, 0))
    if a.calcao_coxa:
        floor_rows = proporcao_calcao.inner_leg_floor(weights, rest, tris, part_of, names, jnames, data["ground"])
        front_rows = proporcao_calcao.front_middle_floor(weights, rest, tris, part_of, names, jnames, data["ground"])
        print(f"calcao coxa: {len(floor_rows[0])} vertices; {proporcao_calcao.apply_floor(weights, *floor_rows)} no piso na entrada; "
              f"frente do meio {len(front_rows[0])}, {proporcao_calcao.apply_pair_floor(weights, *front_rows)} no piso")

    def keep_rules(w):
        if len(arm_rows):
            proporcao_tronco.limit_arm(w, arm_rows, allowance, arm_joints, jnames.index("UpperChest"))
        proporcao_calcao.apply_floor(w, *floor_rows)
        proporcao_calcao.apply_pair_floor(w, *front_rows)
    new = weights.copy()
    error_before, error_after = 0.0, 0.0
    for sweep in range(2):
        neighbor_avg = (average @ new)
        for i in fit:
            score = new[i] + new[neighbors[i]].sum(0)
            if banned[i]:
                score[thigh_joints] = 0.0
            if no_arm[i]:
                score[arm_joints] = 0.0
            candidates = [j for j in np.argsort(-score)[:a.candidatos] if score[j] > 0.0]
            if not candidates:
                continue
            # As colunas: o vertice levado por cada junta candidata em cada pose
            # (no meio, com --espelho, cada par junto: as duas juntas pela metade).
            midline = mirror_of is not None and abs(rest[i, 1]) <= 1e-6
            if midline:
                candidates = list(dict.fromkeys(min(j, partner_joint[j]) for j in candidates))
            cols = []
            for j in candidates:
                moved = skins[:, j, :3] + quat_rotate(skins[:, j, 3:], rest[i])
                if midline and partner_joint[j] != j:
                    moved = 0.5 * (moved + skins[:, partner_joint[j], :3] + quat_rotate(skins[:, partner_joint[j], 3:], rest[i]))
                cols.append(moved.reshape(-1))
            A = np.stack(cols, 1)
            target = targets[:, i, :].reshape(-1)
            k = len(candidates)
            if midline:
                current = np.array([new[i, j] + (new[i, partner_joint[j]] if partner_joint[j] != j else 0.0) for j in candidates])
                nbr = np.array([neighbor_avg[i, j] + (neighbor_avg[i, partner_joint[j]] if partner_joint[j] != j else 0.0) for j in candidates])
            else:
                current = new[i, candidates]
                nbr = neighbor_avg[i, candidates]
            big = 10.0
            A_full = np.vstack([A, np.sqrt(a.atual) * np.eye(k), np.sqrt(a.vizinhos) * np.eye(k), big * np.ones((1, k))])
            b_full = np.concatenate([target, np.sqrt(a.atual) * current, np.sqrt(a.vizinhos) * nbr, [big]])
            w, _ = nnls(A_full, b_full)
            if w.sum() <= 0.0:
                continue
            keep, slots = [], 0
            for kk in np.argsort(-w):
                need = 2 if midline and partner_joint[candidates[kk]] != candidates[kk] else 1
                if w[kk] > 1e-4 and slots + need <= 4:
                    keep.append(kk)
                    slots += need
            A2 = A_full[:, keep]
            w2, _ = nnls(A2, b_full)
            w2 = w2 / max(1e-12, w2.sum())
            if sweep == 1:
                error_before += np.sum((A @ current / max(1e-12, current.sum()) - target) ** 2)
                error_after += np.sum((A[:, keep] @ w2 - target) ** 2)
            new[i] = 0.0
            for kk, ww in zip(keep, w2):
                j = candidates[kk]
                if midline and partner_joint[j] != j:
                    new[i, j] = new[i, partner_joint[j]] = ww / 2.0
                else:
                    new[i, j] = ww
    print(f"erro quadratico medio ate o Delta Mush: {np.sqrt(error_before / max(1, len(fit) * len(poses))) * 1000:.2f} mm -> "
          f"{np.sqrt(error_after / max(1, len(fit) * len(poses))) * 1000:.2f} mm")
    if mirror_of is not None:
        left = np.nonzero(in_piece & (rest[:, 1] > 1e-6))[0]
        new[left] = new[mirror_of[left]][:, partner_joint]
    if len(arm_rows):
        print(f"braco perto: {proporcao_tronco.limit_arm(new, arm_rows, allowance, arm_joints, jnames.index('UpperChest'))} cortados no ajuste")
    if len(floor_rows[0]):
        print(f"calcao coxa: {proporcao_calcao.apply_floor(new, *floor_rows)} no piso depois do ajuste")
    keep_rules(new)

    # Com --desvirar: onde o linear ajustado ainda vira um triangulo das pecas
    # (a normal contra a do alvo liso) em alguma pose de treino, os pesos dos
    # cantos dele (e do espelho) se alisam com os vizinhos; ate 4 ossos.
    relax_pieces = [names.index(name) for name in a.desvirar_pecas.split(",")]
    piece_tris = tris[np.isin(part_of, relax_pieces)]
    movable = in_piece & ~strip & ~near_seam

    def normals(x):
        return np.cross(x[:, piece_tris[:, 1]] - x[:, piece_tris[:, 0]], x[:, piece_tris[:, 2]] - x[:, piece_tris[:, 0]])

    target_normals = normals(targets)

    def flipped():
        posed = np.zeros_like(targets)
        for j in np.nonzero(new.sum(0))[0]:
            mask = new[:, j] > 0.0
            moved = skins[:, j, None, :3] + quat_rotate(skins[:, j, None, 3:], rest[mask][None])
            posed[:, mask] += new[mask, j][None, :, None] * moved
        return np.any(np.sum(normals(posed) * target_normals, axis=2) < 0.0, axis=0)

    for round_ in range(a.desvirar):
        bad = flipped()
        if not bad.any():
            break
        marked = np.zeros(n, bool)
        marked[piece_tris[bad].ravel()] = True
        if mirror_of is not None:
            marked |= marked[mirror_of]
        marked &= movable
        print(f"  desvirar {round_ + 1}: {int(bad.sum())} triangulos virados, {int(marked.sum())} vertices alisados")
        idx = np.nonzero(marked)[0]
        mixed = 0.5 * new[idx] + 0.5 * (average @ new)[idx]
        mixed[np.ix_(banned[idx], thigh_joints)] = 0.0
        for row in mixed:
            row[np.argsort(-row)[4:]] = 0.0
        new[idx] = mixed / np.maximum(1e-12, mixed.sum(1))[:, None]
        keep_rules(new)
    if a.desvirar:
        if mirror_of is not None:
            new[left] = new[mirror_of[left]][:, partner_joint]
        print(f"  desvirar: {int(flipped().sum())} triangulos ainda virados nas poses de treino")
    data["weights"] = [[[int(j), float(new[v, j])] for j in np.nonzero(new[v] > 1e-4)[0]] for v in range(n)]
    json.dump(data, open(a.saida, "w"))
    print(f"gravado {a.saida}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
