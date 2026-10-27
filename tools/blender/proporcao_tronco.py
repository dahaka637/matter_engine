"""A proporcao do tronco perto da axila (22/10/2026): a deformacao da camisa.

O usuario achou a lateral do peito desproporcional (uma "asa" do tronco ate o
braco). Medido contra o ANSUR II (450 homens magros de 1,75 a 1,85 m,
escalados para 1,80 m): a axila humana fica a 1,366 m e a nossa camisa ligava o
tronco ao braco ate 1,29 m; o tronco abria de 33 cm para 37 cm logo abaixo dela
(o peito humano tem 28 cm, com a camisa ~31).

A camisa (so a superficie principal; as costuras com a pele, a gola, o punho e a
barra ficam) e deformada o mais rigida possivel (ARAP, Sorkine e Alexa 2007,
pesos de cotangente): o vale da axila sobe ate a altura humana e entra para o
encontro do braco, e a lateral do tronco abaixo dele vai ate a largura alvo. Um
lado so e resolvido; o outro e o espelho exato. A topologia nao muda (os pesos
seguem pelo ponto de origem de cada vertice, como na etapa 02).

Usado pela etapa 07 (tools/blender/etapas/07_proporcao_tronco.py) e pelo prototipo
sobre a troca exportada. Eixos do modelo: x para a frente, y para a esquerda, z
para cima; "ground" e a altura do chao.
"""
import os

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from scipy.spatial import cKDTree

# A altura humana da axila e o perfil da meia largura da camisa (m do chao, m).
APEX_HEIGHT = 1.335                # a axila alvo (o usuario, 23/10: "ligar um pouco mais a axila", entre a original 1,29 e a humana 1,366)
APEX_SIDE = 0.162
WIDTH = ((1.18, 0.157), (1.24, 0.159), (1.30, 0.163), (1.34, 0.166), (1.37, 0.168))
REGION = (1.04, 1.53, 0.10, 0.31)          # alturas e |y| da regiao que se mexe (o miolo, com os numeros, fica)
PULL = 8.0                                 # o peso dos alvos (o resto e o ARAP)
FAIR_ROUNDS = 40                           # Taubin na regiao deformada (tira os vincos das dobras)
LIFT_SMOOTH = 6                            # rodadas de alisamento do campo da subida antes de virar alvo
LIFT_ARM = (0.15, 0.30, 0.75, 0.95)        # a subida pelo peso do braco: sobe de 0,15 a 0,30, cai de 0,75 a 0,95
BLEND_SMOOTH = 6                           # rodadas de alisamento da mascara da mistura (a faixa de dentro da manga)
CUFF_FREE = 0.03                           # perto da costura do punho, sem alvo (a faixa preta do punho fica)


def cotangent_weights(pos, tris):
    """Os pesos de cotangente das arestas (simetricos, so os positivos)."""
    rows, cols, vals = [], [], []
    for k in range(3):
        i, j, o = tris[:, k], tris[:, (k + 1) % 3], tris[:, (k + 2) % 3]
        u, v = pos[i] - pos[o], pos[j] - pos[o]
        cot = (u * v).sum(1) / np.maximum(1e-12, np.linalg.norm(np.cross(u, v), axis=1))
        rows += [i, j]
        cols += [j, i]
        vals += [0.5 * cot, 0.5 * cot]
    W = sp.coo_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(len(pos), len(pos))).tocsr()
    W.data = np.maximum(W.data, 1e-4)
    return W


def arap(pos, tris, free, targets, soft, iterations=30):
    """Deforma "pos": os vertices fora de "free" ficam; "targets" (indice -> ponto) sao fixos no alvo;
    "soft" (indice -> (ponto, peso)) puxa para o alvo. Devolve as posicoes novas."""
    involved = np.unique(tris[free[tris].any(1)])
    sub = tris[np.isin(tris, involved).all(1)]
    W = cotangent_weights(pos, sub)
    n = len(pos)
    fixed = ~free.copy()
    fixed[list(targets)] = True
    new = pos.copy()
    for i, p in targets.items():
        new[i] = p
    var = np.nonzero(~fixed & np.isin(np.arange(n), involved))[0]
    index = -np.ones(n, int)
    index[var] = np.arange(len(var))
    L = sp.diags(np.asarray(W.sum(1)).ravel()) - W
    Lvv = L[var][:, var].tocsc()
    soft_idx = np.array([i for i in soft if index[i] >= 0], int)
    lam = np.zeros(len(var))
    target_pts = np.zeros((len(var), 3))
    for i in soft_idx:
        p, w = soft[i]
        lam[index[i]] = w
        target_pts[index[i]] = p
    A = (Lvv + sp.diags(lam)).tocsc()
    solve = spla.factorized(A)
    Lvc = L[var][:, np.nonzero(fixed)[0]]
    Wc = W.tocoo()
    edge_i, edge_j, edge_w = Wc.row, Wc.col, Wc.data
    for _ in range(iterations):
        # As rotacoes de cada vertice (o melhor encaixe da vizinhanca original na nova).
        e0 = pos[edge_i] - pos[edge_j]
        e1 = new[edge_i] - new[edge_j]
        S = np.zeros((n, 3, 3))
        np.add.at(S, edge_i, edge_w[:, None, None] * e0[:, :, None] * e1[:, None, :])
        U, _, Vt = np.linalg.svd(S)
        R = np.einsum("nji,nkj->nik", Vt, U)
        bad = np.linalg.det(R) < 0
        if bad.any():
            U[bad, :, -1] *= -1
            R[bad] = np.einsum("nji,nkj->nik", Vt[bad], U[bad])
        rhs_edge = 0.5 * edge_w[:, None] * np.einsum("nij,nj->ni", R[edge_i] + R[edge_j], e0)
        b = np.zeros((n, 3))
        np.add.at(b, edge_i, rhs_edge)
        rhs = b[var] - Lvc @ new[fixed] + lam[:, None] * target_pts
        for axis in range(3):
            new[var, axis] = solve(rhs[:, axis])
    return new


def merge_height(pos, tris, ground, start=1.20, stop=1.42, step=0.002):
    """A altura (do chao) em que o tronco encosta no braco direito: a primeira fatia em que o laco
    que passa pela lateral do tronco (y = -0,12 m) chega ao braco (|y| > 0,26 m)."""
    a, b, c = pos[tris[:, 0]], pos[tris[:, 1]], pos[tris[:, 2]]
    for h in np.arange(start, stop, step):
        z = h + ground
        segs = []
        da, db, dc = a[:, 2] - z, b[:, 2] - z, c[:, 2] - z
        crossing = ((da > 0) != (db > 0)) | ((db > 0) != (dc > 0))
        pts = []
        for t in np.nonzero(crossing)[0]:
            q = []
            for p0, p1, d0, d1 in ((a[t], b[t], da[t], db[t]), (b[t], c[t], db[t], dc[t]), (c[t], a[t], dc[t], da[t])):
                if (d0 > 0) != (d1 > 0):
                    q.append((p0 + d0 / (d0 - d1) * (p1 - p0))[:2])
            if len(q) == 2:
                segs.append(q)
        if not segs:
            continue
        S = np.array(segs)
        ends = S.reshape(-1, 2)
        tree = cKDTree(ends)
        parent = np.arange(len(S))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i
        for i, j in tree.query_pairs(1e-7):
            ra, rb = find(i // 2), find(j // 2)
            if ra != rb:
                parent[ra] = rb
        roots = np.array([find(i) for i in range(len(S))])
        side = np.nonzero((np.abs(S[:, :, 1] + 0.12) < 0.01).any(1) & (S[:, :, 0] < 0.0).any(1))[0]
        for r in set(roots[side]):
            if (S[roots == r][:, :, 1] < -0.26).any():
                return h
    return stop


def fair(pos, tris, movable, rounds, lam=0.5, mu=-0.53):
    """Taubin (lambda|mu) so nos vertices "movable" (o resto fica): alisa os vincos sem encolher."""
    if rounds <= 0 or not movable.any():
        return pos
    n = len(pos)
    rows = np.concatenate([tris[:, 0], tris[:, 1], tris[:, 2], tris[:, 1], tris[:, 2], tris[:, 0]])
    cols = np.concatenate([tris[:, 1], tris[:, 2], tris[:, 0], tris[:, 0], tris[:, 1], tris[:, 2]])
    A = sp.coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(n, n)).tocsr()
    A.data[:] = 1.0
    deg = np.asarray(A.sum(1)).ravel()
    out = pos.copy()
    for _ in range(rounds):
        for step in (lam, mu):
            mean = (A @ out) / np.maximum(1.0, deg)[:, None]
            out[movable] += step * (mean[movable] - out[movable])
    return out


def smooth(edge0, edge1, x):
    t = min(1.0, max(0.0, (x - edge0) / (edge1 - edge0)))
    return t * t * (3.0 - 2.0 * t)


def relax_folds(new, old, tris, movable, rounds=12):
    """Desfaz as dobras que a deformacao criou (faces vizinhas viradas uma contra a outra, que
    nao eram): os cantos delas e um anel em volta relaxam para a media dos vizinhos."""
    edges = {}
    for t, tri in enumerate(tris):
        for k in range(3):
            edges.setdefault(tuple(sorted((tri[k], tri[(k + 1) % 3]))), []).append(t)
    pairs = np.array([f for f in edges.values() if len(f) == 2])
    n = len(new)
    rows = np.concatenate([tris[:, 0], tris[:, 1], tris[:, 2], tris[:, 1], tris[:, 2], tris[:, 0]])
    cols = np.concatenate([tris[:, 1], tris[:, 2], tris[:, 0], tris[:, 0], tris[:, 1], tris[:, 2]])
    A = sp.coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(n, n)).tocsr()
    A.data[:] = 1.0
    deg = np.maximum(1.0, np.asarray(A.sum(1)).ravel())

    def cosines(P):
        f = np.cross(P[tris[:, 1]] - P[tris[:, 0]], P[tris[:, 2]] - P[tris[:, 0]])
        f /= np.maximum(1e-18, np.linalg.norm(f, axis=1))[:, None]
        return (f[pairs[:, 0]] * f[pairs[:, 1]]).sum(1)

    before = cosines(old)
    out = new.copy()
    for done in range(rounds):
        bad = (cosines(out) < 0.3) & (before > 0.5)
        if not bad.any():
            return out, done
        region = np.zeros(n, bool)
        region[tris[pairs[bad].ravel()].ravel()] = True
        region[tris[region[tris].any(1)]] = True
        region &= movable
        for _ in range(3):
            mean = (A @ out) / deg[:, None]
            out[region] += 0.5 * (mean[region] - out[region])
    return out, rounds


def deform(pos, tris, part_of_tri, names, weights, joint_names, ground, apex=None, pull=None, base=None, rounds=FAIR_ROUNDS,
           joint_positions=None):
    """As posicoes novas da malha inteira (so a camisa perto das axilas muda)."""
    shirt = names.index("camisa")
    shirt_tris = part_of_tri == shirt
    on_shirt = np.zeros(len(pos), bool)
    on_shirt[tris[shirt_tris]] = True
    on_other = np.zeros(len(pos), bool)
    on_other[tris[~shirt_tris]] = True
    seam = on_shirt & on_other
    strip = np.zeros(len(pos), bool)
    strip[tris[shirt_tris & seam[tris].any(1)]] = True      # a faixa costurada fica
    height = pos[:, 2] - ground
    h0, h1, y0, y1 = REGION
    right = on_shirt & ~strip & (height > h0) & (height < h1) & (pos[:, 1] < -y0) & (pos[:, 1] > -y1)
    arm_joints = [i for i, n in enumerate(joint_names) if n.startswith("Right") and n.endswith(("UpperArm", "ShoulderHelper"))]
    arm = np.array([sum(w for j, w in pairs if j in arm_joints) for pairs in weights])
    # A axila (as duas dobras e o vale entre elas: o que e meio tronco, meio braco, peso do
    # braco entre 0,2 e 0,85) sobe: a faixa entre o encontro medido e o alto da regiao e
    # comprimida para comecar na altura alvo. A manga fica livre (o ARAP a acomoda) e a
    # lateral do tronco abaixo da axila nova vai ate a largura alvo.
    low = merge_height(pos, tris[part_of_tri != names.index("cabeca")], ground) if base is None else base
    pull = PULL if pull is None else pull
    apex = APEX_HEIGHT if apex is None else apex
    top = REGION[1] - 0.02
    squeeze = (top - apex) / (top - low)
    hs, ws = np.array([w[0] for w in WIDTH]), np.array([w[1] for w in WIDTH])
    pit = right & (arm > 0.2) & (arm < 0.85) & (height > low - 0.03) & (height < top)
    cuff = seam & (pos[:, 1] < -0.17) & (height < 1.42)                 # a costura do punho com a pele
    far_cuff = cKDTree(pos[cuff]).query(pos)[0] > CUFF_FREE
    pit &= far_cuff
    # A subida: uma fracao continua do peso do braco (0 no tronco e no tubo da manga, 1 no meio)
    # vezes o remapeamento da altura; o campo e alisado pela malha antes de virar alvo (os pesos
    # da etapa 02 alternam entre vizinhos no lado de dentro da manga: um corte duro dava alvos
    # 7 cm diferentes lado a lado e a manga ondulava).
    def remap(h):
        return apex + (max(h, low) - low) * squeeze + min(0.0, h - low)

    def ramp(x, a, b):
        t = np.clip((x - a) / (b - a), 0.0, 1.0)
        return t * t * (3.0 - 2.0 * t)

    mirror_d, mirror = cKDTree(pos).query(pos * np.array([1.0, -1.0, 1.0]))
    left = (mirror_d < 1e-5) & right[mirror]
    ring = pit.copy()
    for _ in range(2):
        ring[tris[ring[tris].any(1)]] = True

    def solve(targets, soft, held):
        """O ARAP, o alisamento da faixa da axila e dois aneis em volta, as dobras que sobrarem
        relaxadas e o lado esquerdo como o espelho exato do direito."""
        out = arap(pos, tris, right, targets, soft)
        out = fair(out, tris, ring & right & ~held, rounds)
        out, done = relax_folds(out, pos, tris[shirt_tris], right & ~held)
        out[left] = out[mirror[left]] * np.array([1.0, -1.0, 1.0])
        return out, done

    # 1. Pela faixa (a axila de peso do braco entre 0,2 e 0,85 sobe inteira; o anel do punho
    # parado): a dobra da frente e o anel ficam bons, mas o lado de dentro da manga ondula (os
    # pesos da etapa 02 alternam entre vizinhos ali: alvos 7 cm diferentes lado a lado).
    targets, soft = {}, {}
    for i in np.nonzero(pit)[0]:
        lifted = remap(height[i])
        side = abs(pos[i, 1])
        if lifted < apex + 0.01:                 # o que fica na lateral do tronco, abaixo da axila nova
            side = min(side, np.interp(lifted, hs, ws))
        soft[int(i)] = (np.array([pos[i, 0], -side, lifted + ground]), pull)
    for i in np.nonzero(right & (arm <= 0.2) & (height < apex))[0]:
        limit = np.interp(height[i], hs, ws)
        if abs(pos[i, 1]) > limit:
            soft[int(i)] = (np.array([pos[i, 0], -limit, pos[i, 2]]), pull)
    held = np.zeros(len(pos), bool)
    if joint_positions is not None:
        shoulder, elbow = np.asarray(joint_positions["RightUpperArm"]), np.asarray(joint_positions["RightForearm"])
        along = (elbow - shoulder) / np.linalg.norm(elbow - shoulder)
        s = -((pos - (shoulder + 0.6 * (elbow - shoulder))) @ along)
        held = right & ~pit & (s < CUFF_BAND + 0.009) & (_segment_distance(pos, shoulder, elbow) < 0.075)
        held[tris[held[tris].any(1) & shirt_tris]] = True
        held &= right & ~pit
        for i in np.nonzero(held)[0]:
            targets[int(i)] = pos[i].copy()
            soft.pop(int(i), None)
    banded, relaxed = solve(targets, soft, held)
    info = {"encontro_antes": round(float(low), 3), "axila_alvo": apex, "axila": int(pit.sum()), "alvos": len(soft), "regiao": int(right.sum()),
            "rodadas_dobras": relaxed}
    if joint_positions is None:
        return banded, info
    # 2. Continua (a subida uma fracao continua do peso do braco, alisada pela malha antes de
    # virar alvo; o aperto da largura so na lateral do tronco; o anel solto): o lado de dentro
    # da manga fica liso, mas a dobra da frente da axila vira em poses de agachar e saltar.
    zone = right & far_cuff & (height > low - 0.06) & (height < top)
    share = ramp(arm, LIFT_ARM[0], LIFT_ARM[1]) * (1.0 - ramp(arm, LIFT_ARM[2], LIFT_ARM[3])) * zone
    lift = np.where(zone, np.array([remap(h) - h for h in height]) * share, 0.0)
    nbr_rows = np.concatenate([tris[:, 0], tris[:, 1], tris[:, 2], tris[:, 1], tris[:, 2], tris[:, 0]])
    nbr_cols = np.concatenate([tris[:, 1], tris[:, 2], tris[:, 0], tris[:, 0], tris[:, 1], tris[:, 2]])
    adjacency = sp.coo_matrix((np.ones(len(nbr_rows)), (nbr_rows, nbr_cols)), shape=(len(pos), len(pos))).tocsr()
    adjacency.data[:] = 1.0
    degree = np.maximum(1.0, np.asarray(adjacency.sum(1)).ravel())
    for _ in range(LIFT_SMOOTH):
        lift = np.where(right, 0.5 * lift + 0.5 * (adjacency @ lift) / degree, 0.0)
    outward = -vertex_normals(pos, tris)[:, 1] > 0.2
    soft = {}
    for i in np.nonzero(right)[0]:
        new_h = height[i] + lift[i]
        side = abs(pos[i, 1])
        if outward[i] and new_h < apex + 0.01 and arm[i] < 0.85:
            side = min(side, np.interp(new_h, hs, ws))
        if lift[i] > 5e-4 or side < abs(pos[i, 1]) - 1e-6:
            soft[int(i)] = (np.array([pos[i, 0], -side, new_h + ground]), pull)
    continuous, _ = solve({}, soft, np.zeros(len(pos), bool))
    # 3. A mistura (pedido do usuario, 22/10: com o braco aberto, o contorno de baixo da manga
    # reto e uma curva so para o tronco): a forma continua na faixa de dentro da manga (a malha
    # do tubo virada para o tronco, do punho ate a axila), a pela faixa no resto; a mascara
    # alisada pela malha da camisa e simetrica.
    mask = np.zeros(len(pos))
    normal = vertex_normals(banded, tris)
    for side, sign in (("Right", -1.0), ("Left", 1.0)):
        shoulder, elbow = np.asarray(joint_positions[side + "UpperArm"]), np.asarray(joint_positions[side + "Forearm"])
        along = (elbow - shoulder) / np.linalg.norm(elbow - shoulder)
        rel = banded - (shoulder + 0.6 * (elbow - shoulder))
        s = -(rel @ along)
        radial = rel - np.outer(rel @ along, along)
        inward = np.array([0.0, -sign, 0.0])
        inward -= (inward @ along) * along
        inward /= np.linalg.norm(inward)
        inner = on_shirt & (banded[:, 1] * sign > 0.0) & (normal @ inward > 0.3) & (np.linalg.norm(radial, axis=1) < 0.09) & (s > 0.0) & (s < 0.13)
        mask[inner] = 1.0
    for _ in range(BLEND_SMOOTH):
        mask = np.where(on_shirt, 0.5 * mask + 0.5 * (adjacency @ mask) / degree, 0.0)
    mask = np.clip(mask / max(1e-9, mask.max()), 0.0, 1.0)
    mask = np.maximum(mask, mask[mirror])
    new = banded + mask[:, None] * (continuous - banded)
    new[left] = new[mirror[left]] * np.array([1.0, -1.0, 1.0])
    info["mistura"] = int((mask > 0.5).sum())
    return new, info


# O braco so mexe a camisa perto dele (pedido do usuario, 22/10: erguer o braco deformava o
# peito, a barriga e as costas): o peso das juntas do braco (e da clavicula, que ele leva)
# pode ser inteiro ate ARM_FULL do segmento ombro-cotovelo e cai a zero em ARM_NONE.
ARM_FULL, ARM_NONE = 0.08, 0.14
ARM_CHAIN = ("UpperArm", "ShoulderHelper", "ArmpitHelper", "ClavicleHelper", "Forearm", "ElbowHelper", "Hand", "WristHelper",
             "Thumb", "Index", "Middle", "Ring", "Pinky")


def arm_joints(joint_names):
    return [i for i, n in enumerate(joint_names) if n.startswith(("Left", "Right")) and any(c in n for c in ARM_CHAIN)]


def arm_allowance(pos, joint_positions):
    """A fracao do peso do braco permitida em cada ponto: 1 perto do braco, 0 longe (os dois lados)."""
    d = np.minimum(_segment_distance(pos, np.asarray(joint_positions["RightUpperArm"]), np.asarray(joint_positions["RightForearm"])),
                   _segment_distance(pos, np.asarray(joint_positions["LeftUpperArm"]), np.asarray(joint_positions["LeftForearm"])))
    t = np.clip((ARM_NONE - d) / (ARM_NONE - ARM_FULL), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def limit_arm(W, rows, allowance, arms, fallback):
    """Corta o peso do braco das linhas "rows" de W (vertices x juntas) ate a fracao permitida; o
    que sai vai para as outras juntas do vertice, na proporcao (ou para "fallback"). Ate 4 pesos."""
    sub = W[rows]
    arm = sub[:, arms].sum(1)
    over = arm > allowance[rows] + 1e-6
    if not over.any():
        return 0
    k = np.nonzero(over)[0]
    scale = allowance[rows][k] / np.maximum(1e-12, arm[k])
    removed = arm[k] - allowance[rows][k]
    block = sub[k]
    others = block.copy()
    others[:, arms] = 0.0
    rest = others.sum(1)
    block[:, arms] *= scale[:, None]
    has = rest > 1e-9
    block[has] += others[has] * (removed[has] / rest[has])[:, None]
    block[~has, fallback] += removed[~has]
    for row in block:
        row[np.argsort(-row)[4:]] = 0.0
    block /= np.maximum(1e-12, block.sum(1))[:, None]
    sub[k] = block
    W[rows] = sub
    return int(over.sum())


# A tinta do lado da camisa repintada na forma nova (pintura.json, nas regras dos
# espacos do motor, PlayerPaintSpaces):
#  - o punho: na manga (a menos de 10 cm do segmento ombro-cotovelo), ate 2,6 cm acima
#    do plano da barra da manga, ao longo do braco (sleevePoint a 0,6 do braco);
#  - a faixa do lado: de 6 cm atras a 4 cm a frente da linha do lado, da barra ate a
#    axila, reta em cima. O alto segue a axila: a mesma distancia abaixo dela que tinha.
# Na forma antiga o fundo da axila ficava a 2,2 cm do plano da barra, dentro do punho:
# a tinta dele subiria com a axila. Por isso a tinta escura do lado sai toda (pela forma
# antiga) e as duas faixas sao pintadas de novo, com a borda lisa.
CUFF_BAND = 0.026
STRIPE = (-0.06, 0.04)                    # de 6 cm atras a 4 cm a frente da linha do lado
EDGE_PX = 1.5                              # a borda (pixels) repintada em volta de cada face da zona


def side_line(pos, tris, part_of_tri, names, ground):
    """A linha do lado do tronco (lado direito): x (para a frente) como uma reta da altura,
    pelo ponto mais de fora de cada faixa de 2 cm entre 1,00 e 1,30 m."""
    shirt = np.unique(tris[part_of_tri == names.index("camisa")])
    P = pos[shirt]
    height = P[:, 2] - ground
    hs, xs = [], []
    for h0 in np.arange(1.00, 1.30, 0.02):
        m = (P[:, 1] < -0.10) & (P[:, 1] > -0.20) & (height >= h0) & (height < h0 + 0.02)
        if m.sum() >= 3:
            k = np.argmax(np.abs(P[m, 1]))
            hs.append(height[m][k])
            xs.append(P[m, 0][k])
    slope, offset = np.polyfit(hs, xs, 1)
    return slope, offset


def _segment_distance(P, a, b):
    t = np.clip(((P - a) @ (b - a)) / ((b - a) @ (b - a)), 0.0, 1.0)
    return np.linalg.norm(P - (a + t[:, None] * (b - a)), axis=1)


def repaint_sides(pixels, uv, before, after, tris, part_of_tri, names, ground, joints, fresh=None):
    """Repinta o punho e a faixa do lado nos pixels do atlas (H x W x 3, linha 0 embaixo como
    no Blender; uv por canto [u, v]). "before"/"after": as posicoes antes e depois da
    deformacao; "joints": nome -> posicao. Devolve (pixels mudados, alto da faixa antes, depois)."""
    H, W = pixels.shape[:2]
    shoulder, elbow = np.asarray(joints["RightUpperArm"]), np.asarray(joints["RightForearm"])
    along = (elbow - shoulder) / np.linalg.norm(elbow - shoulder)
    sleeve_point = shoulder + 0.6 * (elbow - shoulder)
    no_head = tris[part_of_tri != names.index("cabeca")]
    apex_before, apex_after = merge_height(before, no_head, ground), merge_height(after, no_head, ground)
    slope, offset = side_line(after, tris, part_of_tri, names, ground)
    shirt = np.nonzero(part_of_tri == names.index("camisa"))[0]
    centre = before[tris[shirt]].mean(1)
    in_zone = (np.abs(centre[:, 1]) > 0.10) & (centre[:, 2] - ground > 0.85) & (centre[:, 2] - ground < 1.45)
    if fresh is not None:
        in_zone |= fresh[shirt]                    # as faces de mapa refeito: o lugar novo no atlas nao tem tinta nenhuma
    zone = shirt[in_zone]
    # Os pixels cobertos pelos outros triangulos (de qualquer parte): a borda alargada das faces da
    # zona nao pinta por cima deles.
    zone_set = set(zone.tolist())
    covered = np.zeros((H, W), bool)
    for t in range(len(tris)):
        if t in zone_set:
            continue
        x, y = uv[t, :, 0] * W, uv[t, :, 1] * H
        x0, x1 = int(max(0, np.floor(x.min()))), int(min(W - 1, np.ceil(x.max())))
        y0, y1 = int(max(0, np.floor(y.min()))), int(min(H - 1, np.ceil(y.max())))
        if x1 < x0 or y1 < y0:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        den = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
        if abs(den) < 1e-12:
            continue
        l1 = ((y[1] - y[2]) * (gx - x[2]) + (x[2] - x[1]) * (gy - y[2])) / den
        l2 = ((y[2] - y[0]) * (gx - x[2]) + (x[0] - x[2]) * (gy - y[2])) / den
        hit = (l1 >= 0) & (l2 >= 0) & (1.0 - l1 - l2 >= 0)
        covered[gy[hit].astype(int), gx[hit].astype(int)] = True
    rows, cols, p0, p1, n0, n1, size, new_island, depth = [], [], [], [], [], [], [], [], []
    for t in zone:
        c0, c1 = before[tris[t]], after[tris[t]]
        x, y = uv[t, :, 0] * W, uv[t, :, 1] * H
        uv_area = 0.5 * abs((x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0]))
        normal1 = np.cross(c1[1] - c1[0], c1[2] - c1[0])
        area1 = 0.5 * np.linalg.norm(normal1)
        if uv_area < 1e-9 or area1 < 1e-12:
            continue
        normal0 = np.cross(c0[1] - c0[0], c0[2] - c0[0])
        # A face e uma borda de EDGE_PX pixels em volta (o filtro do mapa le o pixel vizinho a
        # aresta: sem a borda, a tinta antiga da faixa ficava ali e virava um traco escuro).
        x0, x1 = int(max(0, np.floor(x.min() - EDGE_PX))), int(min(W - 1, np.ceil(x.max() + EDGE_PX)))
        y0, y1 = int(max(0, np.floor(y.min() - EDGE_PX))), int(min(H - 1, np.ceil(y.max() + EDGE_PX)))
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        den = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
        l1 = ((y[1] - y[2]) * (gx - x[2]) + (x[2] - x[1]) * (gy - y[2])) / den
        l2 = ((y[2] - y[0]) * (gx - x[2]) + (x[0] - x[2]) * (gy - y[2])) / den
        l3 = 1.0 - l1 - l2
        # A distancia (pixels) de fora ate a face: cada coordenada negativa vezes a altura do canto.
        heights = np.array([2.0 * uv_area / max(1e-9, np.hypot(x[(k + 2) % 3] - x[(k + 1) % 3], y[(k + 2) % 3] - y[(k + 1) % 3])) for k in range(3)])
        outside = np.maximum(0.0, np.maximum(np.maximum(-l1 * heights[0], -l2 * heights[1]), -l3 * heights[2]))
        inside = (outside <= EDGE_PX) & ((outside <= 0.0) | ~covered[gy.astype(int), gx.astype(int)])
        if not inside.any():
            continue
        lam = np.stack([l1[inside], l2[inside], l3[inside]], 1)
        rows.append(gy[inside].astype(int))
        cols.append(gx[inside].astype(int))
        depth.append(outside[inside])
        p0.append(lam @ c0)
        p1.append(lam @ c1)
        k = inside.sum()
        n0.append(np.repeat((normal0 / max(1e-12, np.linalg.norm(normal0)))[None], k, 0))
        n1.append(np.repeat((normal1 / (2.0 * area1))[None], k, 0))
        size.append(np.full(k, np.sqrt(area1 / uv_area)))
        new_island.append(np.full(k, bool(fresh[t]) if fresh is not None else False))
    rows, cols = np.concatenate(rows), np.concatenate(cols)
    p0, p1, n0, n1, size, new_island, depth = (np.concatenate(a) for a in (p0, p1, n0, n1, size, new_island, depth))
    # Cada pixel uma vez so: o da face em que ele esta mais por dentro.
    order = np.lexsort((depth, rows * W + cols))
    first = np.ones(len(order), bool)
    first[1:] = (rows * W + cols)[order][1:] != (rows * W + cols)[order][:-1]
    keep = order[first]
    rows, cols, p0, p1, n0, n1, size, new_island = (a[keep] for a in (rows, cols, p0, p1, n0, n1, size, new_island))
    # Tudo no lado direito (a camisa e o espelho): y vira -|y|, a normal junto.
    for P, N in ((p0, n0), (p1, n1)):
        flip = P[:, 1] > 0.0
        P[flip, 1] *= -1.0
        N[flip, 1] *= -1.0
    old = pixels[rows, cols].copy()
    dark = old.sum(1) < 2.6
    # O alto da faixa antiga: a tinta escura fora do punho, no tronco virado para fora.
    s0 = -((p0 - sleeve_point) @ along)
    sleeve0 = (_segment_distance(p0, shoulder, elbow) < 0.1) & (s0 < 0.12)
    # (As faces de mapa refeito estao num lugar novo do atlas, sem a tinta antiga: fora da medida.)
    stripe0 = dark & ~new_island & ~(sleeve0 & (s0 < CUFF_BAND + 0.01)) & (-n0[:, 1] > 0.3) & (np.abs(p0[:, 1]) < 0.20)
    top_before = float(np.percentile(p0[stripe0, 2] - ground, 99.5))
    top_after = top_before + (apex_after - apex_before)
    light = old[~dark & ~new_island]
    white = np.median(light, axis=0) if len(light) else np.array([0.925, 0.925, 0.925])
    deep = (old.sum(1) < 0.6) & ~new_island
    black = np.median(old[deep], axis=0) if deep.any() else np.array([0.118, 0.118, 0.129])
    edge = 0.7 * size
    # O punho na forma nova.
    s1 = -((p1 - sleeve_point) @ along)
    sleeve1 = (_segment_distance(p1, shoulder, elbow) < 0.1) & (s1 < 0.12)
    band = sleeve1 * np.clip((CUFF_BAND - s1 + edge) / (2.0 * edge), 0.0, 1.0)
    # A faixa do lado na forma nova (o tronco virado para fora; a manga fica de fora).
    h1 = p1[:, 2] - ground
    dx = p1[:, 0] - (slope * h1 + offset)
    across = np.clip((np.minimum(dx - STRIPE[0], STRIPE[1] - dx) + edge) / (2.0 * edge), 0.0, 1.0)
    below = np.clip((top_after - h1 + edge) / (2.0 * edge), 0.0, 1.0)
    # (A virada para fora so conta perto da axila: tira o lado de dentro da manga; embaixo a
    # dobra de dentro da barra tambem leva a faixa.)
    torso = ((h1 < 1.15) | (-n1[:, 1] > 0.2)) & (np.abs(p1[:, 1]) < 0.20) & ~(sleeve1 & (s1 < CUFF_BAND + 0.02))
    stripe = across * below * torso
    cover = np.maximum(band, stripe)
    new = white[None, :] * (1.0 - cover[:, None]) + black[None, :] * cover[:, None]
    if os.environ.get("PROPORCAO_DEBUG"):        # o punho vermelho, a faixa azul
        new = white[None, :] * (1.0 - cover[:, None]) + (band[:, None] * np.array([0.9, 0.1, 0.1]) + stripe[:, None] * np.array([0.1, 0.2, 0.9])) / np.maximum(1e-6, band + stripe)[:, None] * cover[:, None]
    pixels[rows, cols] = new
    return int((np.abs(new - old).sum(1) > 1e-3).sum()), top_before, top_after


def vertex_normals(pos, tris):
    n = np.zeros_like(pos)
    f = np.cross(pos[tris[:, 1]] - pos[tris[:, 0]], pos[tris[:, 2]] - pos[tris[:, 0]])
    for k in range(3):
        np.add.at(n, tris[:, k], f)
    return n / np.maximum(1e-12, np.linalg.norm(n, axis=1))[:, None]
