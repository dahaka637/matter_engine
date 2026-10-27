#!/usr/bin/env python3
"""O template da superficie do sistema de aparencia (26/10/2026; docs/SISTEMA_DE_APARENCIA.md).

Le a troca (com o molde de cada triangulo, gravado pela construcao dos moldes) e acrescenta a ela o
template: a coordenada de desenho de cada canto, as cartas, a regiao de cada face, as costuras, as
aberturas e as juncoes, as ancoras e as zonas de identificacao, o UV0 (o endereco no atlas do
uniforme) e o UV1 (o da identificacao), com a referencia 3D canonica de cada canto.

A pasta assets/characters/player_model/superficie guarda:
 - definicoes.json: as regras (os paineis, as regioes, as cartas e o metodo delas, as ancoras e as
   zonas pelos marcos) e as versoes em uso;
 - referencia_pN.json: a referencia canonica N, por poligono da malha (os vertices dele: a
   identidade estrutural, que sobrevive a outra triangulacao do quad): o painel, a regiao, a zona,
   a coordenada de desenho e as das cartas em cada vertice; a posicao canonica de cada vertice; as
   ancoras, as zonas e os cortes resolvidos;
 - layout_sN.json: o layout N do UV0.
Sem opcao, o gerador so APLICA a referencia e o layout em uso: um corpo diferente com a mesma
topologia sai com o mesmo desenho, bit a bit (a posicao de agora nao entra). Outra topologia para o
gerador. --nova-referencia replanifica pela geometria de agora (os paineis por LSCM e ARAP, as
cartas por campos de direcao ou ARAP, os cortes dos tubos: a manga pela coluna da grade, os outros
escolhidos pela direcao e gravados) e e uma versao nova; --novo-layout refaz o layout (outra versao).
As versoes novas so sao gravadas depois de tudo conferido (inclusive que nenhum mapa se sobrepoe);
as anteriores ficam.

    python3 tools/blender/superficie.py <entrada.troca.json> <saida.troca.json> [--nova-referencia] [--novo-layout]
"""
import heapq
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np
from scipy.sparse import coo_matrix, csc_matrix
from scipy.sparse.linalg import splu, spsolve
from scipy.spatial import cKDTree

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SUPERFICIE = os.path.join(ROOT, "assets", "characters", "player_model", "superficie")
# O tamanho de base do atlas do uniforme (o layout guarda a densidade nele) e a folga
# em volta de cada painel, em texels desse tamanho.
BASE_SIZE = 1024
# (64 desde o layout s2, 26/10: com 16 o nivel seguro dos mips era 2 - de longe, abaixo de ~260 px de
# altura na tela, o uniforme ia para a media da regiao; com 64 e 4, ~45 px, a camera da transmissao.
# Custa ~31% da densidade: ~405 texels/m.)
MARGIN_TEXELS = 64
# As faixas do recuo (por dentro, quase nunca a vista) com metade da densidade.
STRIP_DENSITY = 0.5
# O quanto um painel pode crescer alem da caixa gravada (m) numa reconstrucao compativel:
# ate metade da folga entre os paineis (era 16 texels a ~600 texels/m, ~2,7 cm; o s2 tem mais folga).
LAYOUT_TOLERANCE = 0.008
ARAP_ROUNDS = 80


def fail(message):
    sys.exit("superficie: " + message)


def edge_key(a, b):
    return (a, b) if a < b else (b, a)


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def local_frame(a, b, c):
    """As coordenadas 2D isometricas do triangulo (a na origem, b no eixo x), no
    sentido da normal: olhando de fora, anti-horario."""
    e1 = unit(b - a)
    n = np.cross(b - a, c - a)
    e2 = np.cross(unit(n), e1)
    return np.array([[0.0, 0.0], [np.dot(b - a, e1), 0.0], [np.dot(c - a, e1), np.dot(c - a, e2)]])


class Mesh:
    def __init__(self, data):
        self.P = np.array(data["positions"], float)
        self.T = np.array(data["triangles"], int)
        self.F = len(self.T)
        self.edge_tris = defaultdict(list)
        for t, (a, b, c) in enumerate(self.T):
            for u, v in ((a, b), (b, c), (c, a)):
                self.edge_tris[edge_key(u, v)].append(t)
        self.C = self.P[self.T].mean(1)
        tree = cKDTree(self.P)
        distance, self.mirror = tree.query(self.P * np.array([1.0, -1.0, 1.0]))
        if distance.max() > 1e-5:
            fail(f"a malha nao e simetrica ({distance.max() * 1000:.3f} mm)")
        self.by_corners = {tuple(sorted(t)): i for i, t in enumerate(self.T.tolist())}

    def mirror_triangle(self, t):
        return self.by_corners[tuple(sorted(int(self.mirror[v]) for v in self.T[t]))]


# ---------------------------------------------------------------- malha local e corte

def local_mesh(mesh, tris, cut_edges):
    """Os triangulos como malha propria: cada vertice vira uma copia por leque (os
    triangulos em volta dele ligados por arestas que nao sao de corte)."""
    tris = list(tris)
    tset = set(tris)
    incident = defaultdict(list)
    for t in tris:
        for v in mesh.T[t]:
            incident[int(v)].append(t)
    index, origin = {}, []
    for v, around in incident.items():
        parent = {t: t for t in around}

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        aset = set(around)
        for t in around:
            for w in mesh.T[t]:
                w = int(w)
                if w == v or edge_key(v, w) in cut_edges:
                    continue
                for t2 in mesh.edge_tris[edge_key(v, w)]:
                    if t2 != t and t2 in aset:
                        parent[find(t)] = find(t2)
        groups = defaultdict(list)
        for t in around:
            groups[find(t)].append(t)
        for group in sorted(groups.values(), key=min):
            for t in group:
                index[(t, v)] = len(origin)
            origin.append(v)
    faces = np.array([[index[(t, int(v))] for v in mesh.T[t]] for t in tris])
    return np.array(origin), faces, tris


def boundary_edges(faces):
    count = defaultdict(int)
    for f in faces:
        for k in range(3):
            count[edge_key(f[k], f[(k + 1) % 3])] += 1
    return [e for e, n in count.items() if n == 1]


def chains(edges):
    """As polilinhas (listas de vertices, em ordem) de um conjunto de arestas."""
    links = defaultdict(list)
    for a, b in edges:
        links[a].append(b)
        links[b].append(a)
    seen, out = set(), []
    for start in sorted(links, key=lambda v: (len(links[v]) != 1, v)):
        if start in seen:
            continue
        line = [start]
        seen.add(start)
        while True:
            nxt = next((w for w in links[line[-1]] if w not in seen), None)
            if nxt is None:
                break
            seen.add(nxt)
            line.append(nxt)
        out.append(line)
    return out


def edge_components(edges):
    """Os componentes ligados de um conjunto de arestas, cada um com todas as arestas dele (um
    anel fechado inclusive: as polilinhas de chains perdem a aresta que fecha o ciclo)."""
    parent = {}

    def find(v):
        while parent.setdefault(v, v) != v:
            parent[v] = parent[parent[v]]
            v = parent[v]
        return v

    for a, b in edges:
        parent[find(a)] = find(b)
    groups = defaultdict(list)
    for a, b in edges:
        groups[find(a)].append(edge_key(a, b))
    return [sorted(set(g)) for _, g in sorted(groups.items(), key=lambda kv: min(kv[1]))]


def cut_path(mesh, tris, sources, targets, prefer, axis_point, axis_dir):
    """O caminho de arestas de dentro dos triangulos (sem as da borda) de um vertice de
    sources a um de targets que fica mais perto da direcao prefer em volta do eixo:
    o corte de um tubo."""
    tset = set(tris)
    border = set()
    for t in tris:
        for k in range(3):
            e = edge_key(int(mesh.T[t][k]), int(mesh.T[t][(k + 1) % 3]))
            if sum(1 for u in mesh.edge_tris[e] if u in tset) == 1:
                border.add(e)
    neighbors = defaultdict(list)
    for t in tris:
        for k in range(3):
            a, b = int(mesh.T[t][k]), int(mesh.T[t][(k + 1) % 3])
            if edge_key(a, b) not in border:
                neighbors[a].append(b)
                neighbors[b].append(a)

    def misalign(p):
        radial = p - axis_point
        radial = radial - np.dot(radial, axis_dir) * axis_dir
        return 1.0 - np.dot(unit(radial), prefer)

    def cost(a, b):
        middle = (mesh.P[a] + mesh.P[b]) / 2.0
        return np.linalg.norm(mesh.P[a] - mesh.P[b]) * (1.0 + 40.0 * misalign(middle))

    best = {v: 0.4 * misalign(mesh.P[v]) for v in sources}
    back = {}
    heap = [(d, v) for v, d in best.items()]
    heapq.heapify(heap)
    targets = set(targets)
    while heap:
        d, v = heapq.heappop(heap)
        if d > best.get(v, math.inf):
            continue
        if v in targets:
            path = [v]
            while path[-1] in back:
                path.append(back[path[-1]])
            return [edge_key(path[k], path[k + 1]) for k in range(len(path) - 1)]
        for w in set(neighbors[v]):
            nd = d + cost(v, w)
            if nd < best.get(w, math.inf):
                best[w] = nd
                back[w] = v
                heapq.heappush(heap, (nd, w))
    fail("corte sem caminho entre as bordas")


# ---------------------------------------------------------------- planificacao

def lscm(V, faces):
    n = len(V)
    rows, cols, vals = [], [], []
    for r, (i, j, k) in enumerate(faces):
        p = local_frame(V[i], V[j], V[k])
        d = abs(p[1][0] * p[2][1] - p[2][0] * p[1][1])
        s = 1.0 / math.sqrt(max(d, 1e-16))
        w = [p[2] - p[1], p[0] - p[2], p[1] - p[0]]
        for idx, (a, b) in zip((i, j, k), w):
            rows += [2 * r, 2 * r, 2 * r + 1, 2 * r + 1]
            cols += [idx, n + idx, idx, n + idx]
            vals += [a * s, -b * s, b * s, a * s]
    A = coo_matrix((vals, (rows, cols)), shape=(2 * len(faces), 2 * n)).tocsc()
    edges = boundary_edges(faces)
    ring = sorted({v for e in edges for v in e})
    first = max(ring, key=lambda v: np.linalg.norm(V[v] - V[ring[0]]))
    second = max(ring, key=lambda v: np.linalg.norm(V[v] - V[first]))
    pins = {first: (0.0, 0.0), second: (np.linalg.norm(V[second] - V[first]), 0.0)}
    fixed = np.zeros(2 * n)
    mask = np.ones(2 * n, bool)
    for v, (u, w) in pins.items():
        fixed[v], fixed[n + v] = u, w
        mask[v] = mask[n + v] = False
    A_free, A_fixed = A[:, mask], A[:, ~mask]
    rhs = -A_fixed @ fixed[~mask]
    x = spsolve((A_free.T @ A_free).tocsc(), A_free.T @ rhs)
    out = fixed.copy()
    out[mask] = x
    return np.stack([out[:n], out[n:]], 1)


def arap(V, faces, U):
    """O ARAP local/global (Liu 2008) a partir de U, com os pesos de cotangente."""
    n, m = len(V), len(faces)
    X = np.array([local_frame(V[i], V[j], V[k]) for i, j, k in faces])
    cot = np.zeros((m, 3))
    for t in range(m):
        for k in range(3):
            a, b, c = X[t][k], X[t][(k + 1) % 3], X[t][(k + 2) % 3]
            u, w = b - a, c - a
            cross = abs(u[0] * w[1] - u[1] * w[0])
            cot[t, k] = max(0.5 * np.dot(u, w) / max(cross, 1e-16), 1e-4)
    # A aresta (k+1, k+2) do triangulo, oposta ao canto k, com o peso dele.
    I = np.concatenate([faces[:, (k + 1) % 3] for k in range(3)])
    J = np.concatenate([faces[:, (k + 2) % 3] for k in range(3)])
    W = np.concatenate([cot[:, k] for k in range(3)])
    L = coo_matrix((np.concatenate([W, W, -W, -W]), (np.concatenate([I, J, I, J]), np.concatenate([I, J, J, I]))), shape=(n, n)).tocsc()
    free = np.arange(1, n)
    solve = splu(csc_matrix(L[free][:, free]))
    pinned = L[free][:, [0]]
    U = U.copy()
    E = [X[:, (k + 1) % 3] - X[:, (k + 2) % 3] for k in range(3)]
    for _ in range(ARAP_ROUNDS):
        S = np.zeros((m, 2, 2))
        for k in range(3):
            eu = U[faces[:, (k + 1) % 3]] - U[faces[:, (k + 2) % 3]]
            S += cot[:, k, None, None] * np.einsum("ti,tj->tij", eu, E[k])
        A_, _, B_ = np.linalg.svd(S)
        R = A_ @ B_
        flip = np.linalg.det(R) < 0.0
        A_[flip, :, 1] *= -1.0
        R = A_ @ B_
        b = np.zeros((n, 2))
        for k in range(3):
            r = cot[:, k, None] * np.einsum("tij,tj->ti", R, E[k])
            np.add.at(b, faces[:, (k + 1) % 3], r)
            np.add.at(b, faces[:, (k + 2) % 3], -r)
        rhs = b[free] - pinned @ U[[0]]
        U[free] = np.stack([solve.solve(rhs[:, 0]), solve.solve(rhs[:, 1])], 1)
    return U


def field_parametrization(V, faces, ex, ey, weight, pin):
    """As duas funcoes cujos gradientes mais se parecem com os campos ex e ey (por face, no
    plano dela), pesadas por face, com o vertice pin na origem: (G^T W G) f = G^T W e."""
    n_verts = len(V)
    a, b, c = V[faces[:, 0]], V[faces[:, 1]], V[faces[:, 2]]
    normal = np.cross(b - a, c - a)
    double = np.linalg.norm(normal, axis=1)
    normal = normal / np.maximum(double, 1e-16)[:, None]
    rows, cols, vals = [], [], []
    for corner, (p, q) in enumerate(((b, c), (c, a), (a, b))):
        grad = np.cross(normal, q - p) / np.maximum(double, 1e-16)[:, None]
        for comp in range(3):
            rows.append(np.arange(len(faces)) * 3 + comp)
            cols.append(faces[:, corner])
            vals.append(grad[:, comp])
    G = coo_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(3 * len(faces), n_verts)).tocsc()
    W = np.repeat(weight, 3)
    GtW = G.T.multiply(W)
    L = (GtW @ G).tocsc()
    free = np.array([v for v in range(n_verts) if v != pin])
    Lf = L[free][:, free].tocsc()
    out = np.zeros((n_verts, 2))
    for k, e in enumerate((ex, ey)):
        rhs = GtW @ e.reshape(-1)
        out[free, k] = spsolve(Lf, rhs[free])
    return out


def unfold_strip(V, faces):
    """Uma fileira de triangulos (cada um com no maximo dois vizinhos, nenhum vertice de dentro)
    desdobrada triangulo a triangulo pela aresta comum: cada um com os comprimentos exatos do 3D e
    a mesma orientacao do primeiro. A tira sai curva (o cone desenrolado), sem distorcao."""
    by_edge = defaultdict(list)
    for r, f in enumerate(faces):
        for k in range(3):
            by_edge[edge_key(int(f[k]), int(f[(k + 1) % 3]))].append(r)
    if any(len(rs) > 2 for rs in by_edge.values()):
        fail("a fileira nao e uma superficie (aresta com mais de duas faces)")
    rim = {v for e, rs in by_edge.items() if len(rs) == 1 for v in e}
    if rim != {int(v) for f in faces for v in f}:
        fail("a fileira tem vertice de dentro")
    U = np.full((len(V), 2), np.nan)
    i, j, k = faces[0]
    lij = np.linalg.norm(V[j] - V[i])
    U[i], U[j] = (0.0, 0.0), (lij, 0.0)

    def third(a, b, c):
        """O vertice c do triangulo (a, b, c), na ordem da face, com a e b ja postos: o lado que
        deixa a area positiva."""
        ab = U[b] - U[a]
        lab, lac, lbc = np.linalg.norm(ab), np.linalg.norm(V[c] - V[a]), np.linalg.norm(V[c] - V[b])
        along = (lac * lac - lbc * lbc + lab * lab) / (2.0 * lab)
        up = math.sqrt(max(lac * lac - along * along, 0.0))
        e = ab / lab
        return U[a] + e * along + np.array([-e[1], e[0]]) * up

    U[k] = third(i, j, k)
    placed, queue = {0}, [0]
    while queue:
        r = queue.pop()
        f = faces[r]
        for m in range(3):
            for q in by_edge[edge_key(int(f[m]), int(f[(m + 1) % 3]))]:
                if q in placed:
                    continue
                g = [int(x) for x in faces[q]]
                missing = [x for x in range(3) if np.isnan(U[g[x]]).any()]
                if len(missing) != 1:
                    fail("a fileira fecha um ciclo (um triangulo com os tres vertices ja postos)")
                n = missing[0]
                U[g[n]] = third(g[(n + 1) % 3], g[(n + 2) % 3], g[n])
                placed.add(q)
                queue.append(q)
    if len(placed) != len(faces) or np.isnan(U).any():
        fail("a fileira nao e uma peca so")
    for (a, b) in by_edge:
        if abs(np.linalg.norm(U[a] - U[b]) - np.linalg.norm(V[a] - V[b])) > 1e-9 + 1e-6 * np.linalg.norm(V[a] - V[b]):
            fail("a fileira desdobrada nao guarda os comprimentos")
    return U


def flatten(V, faces):
    U = arap(V, faces, lscm(V, faces))
    area = 0.0
    for i, j, k in faces:
        a, b = U[j] - U[i], U[k] - U[i]
        area += a[0] * b[1] - a[1] * b[0]
    if area < 0.0:
        U[:, 0] *= -1.0
    return U


def oriented(U, origin, top):
    """O referencial do contrato: a origem no marco de baixo, y para o de cima."""
    y = unit(top - origin)
    x = np.array([y[1], -y[0]])
    return np.stack([(U - origin) @ x, (U - origin) @ y], 1)


def stretch(V, faces, U):
    """O estiramento de cada triangulo (max(s1, 1/s2) do jacobiano) e a area."""
    out, areas = [], []
    for i, j, k in faces:
        X = local_frame(V[i], V[j], V[k])
        Dx = np.array([X[1], X[2]]).T
        Du = np.array([U[j] - U[i], U[k] - U[i]]).T
        J = Du @ np.linalg.inv(Dx)
        s = np.linalg.svd(J, compute_uv=False)
        out.append(max(s[0], 1.0 / max(s[1], 1e-9)))
        areas.append(abs(np.linalg.det(Dx)) / 2.0)
    return np.array(out), np.array(areas)


def clipped_area(a, b):
    """A area da intersecao de dois triangulos anti-horarios: a recortado pelos tres lados de b
    (Sutherland-Hodgman). Dois triangulos que so dividem uma aresta ou um vertice dao zero."""
    poly = [tuple(p) for p in a]
    for k in range(3):
        (x0, y0), (x1, y1) = b[k], b[(k + 1) % 3]
        side = lambda q: (x1 - x0) * (q[1] - y0) - (y1 - y0) * (q[0] - x0)
        out = []
        for i, cur in enumerate(poly):
            nxt = poly[(i + 1) % len(poly)]
            sc, sn = side(cur), side(nxt)
            if sc >= 0.0:
                out.append(cur)
            if (sc >= 0.0) != (sn >= 0.0):
                r = sc / (sc - sn)
                out.append((cur[0] + (nxt[0] - cur[0]) * r, cur[1] + (nxt[1] - cur[1]) * r))
        poly = out
        if len(poly) < 3:
            return 0.0
    return 0.5 * abs(sum(poly[i][0] * poly[i - 1][1] - poly[i - 1][0] * poly[i][1] for i in range(len(poly))))


def overlaps(corners):
    """A injetividade exata do mapa: os triangulos degenerados (area menor que 1e-9 do quadrado da
    maior aresta) e os pares que se sobrepoem (a area da intersecao maior que 1e-6 da do menor; a
    borda compartilhada da zero). Os candidatos vem de uma grade pelas caixas."""
    tris, degenerate = {}, []
    for t, c in corners.items():
        c = np.asarray(c, float)
        area = 0.5 * ((c[1, 0] - c[0, 0]) * (c[2, 1] - c[0, 1]) - (c[2, 0] - c[0, 0]) * (c[1, 1] - c[0, 1]))
        longest = max(float(np.sum((c[k] - c[k - 1]) ** 2)) for k in range(3))
        if abs(area) <= 1e-9 * longest:
            degenerate.append(int(t))
            continue
        tris[t] = (c if area > 0.0 else c[[0, 2, 1]], abs(area))
    if not tris:
        return 0, [], degenerate
    boxes = {t: (c.min(0), c.max(0)) for t, (c, _) in tris.items()}
    size = float(np.median([np.max(hi - lo) for lo, hi in boxes.values()])) * 2.0 or 1.0
    cells = defaultdict(list)
    for t, (lo, hi) in boxes.items():
        for i in range(int(np.floor(lo[0] / size)), int(np.floor(hi[0] / size)) + 1):
            for j in range(int(np.floor(lo[1] / size)), int(np.floor(hi[1] / size)) + 1):
                cells[(i, j)].append(t)
    count, pairs, seen = 0, [], set()
    for members in cells.values():
        for x in range(len(members)):
            for y in range(x + 1, len(members)):
                t, u = members[x], members[y]
                if (t, u) in seen:
                    continue
                seen.add((t, u))
                (lt, ht), (lu, hu) = boxes[t], boxes[u]
                if np.any(lt >= hu) or np.any(lu >= ht):
                    continue
                if clipped_area(tris[t][0], tris[u][0]) > 1e-6 * min(tris[t][1], tris[u][1]):
                    count += 1
                    pairs.append((int(min(t, u)), int(max(t, u))))
    return count, sorted(pairs)[:4], degenerate


def weighted_percentile(values, weights, q):
    order = np.argsort(values)
    cum = np.cumsum(weights[order])
    return float(values[order][min(len(values) - 1, np.searchsorted(cum, q * cum[-1]))])


# ---------------------------------------------------------------- o template

class Template:
    def __init__(self, data, defs, reference=None):
        """Sem referencia, tudo sai da geometria de agora (uma referencia nova); com ela, o painel,
        a regiao e os cortes de cada poligono vem dela e nada se recalcula pela geometria."""
        self.data, self.defs = data, defs
        self.mesh = mesh = Mesh(data)
        # Cada triangulo na chave do poligono dele (os vertices, em ordem): a identidade estrutural
        # que sobrevive a outra triangulacao do quad (a ida e volta pelo Blender).
        if "trianglePolygon" not in data:
            fail("a troca nao tem trianglePolygon")
        members = defaultdict(list)
        for t, pid in enumerate(data["trianglePolygon"]):
            members[pid].append(t)
        self.polygon_key = [None] * mesh.F
        for tris in members.values():
            key = tuple(sorted({int(v) for t in tris for v in mesh.T[t]}))
            for t in tris:
                self.polygon_key[t] = key
        names = data.get("moldes")
        if names is None or "triangleMolde" not in data:
            fail("a troca nao tem os moldes (exporte do .blend com o atributo \"molde\")")
        mold = data["triangleMolde"]
        parts = [p["name"] for p in data["parts"]]
        part = data["trianglePart"]
        joints = {j["name"]: np.array(j["position"], float) for j in data["joints"]}
        self.panels = defs["panels"]
        self.regions = defs["regions"]
        self.panel_of = [names[m] if m else None for m in mold]
        for name in set(p for p in self.panel_of if p):
            if name not in self.panels:
                fail(f"o molde {name} nao esta nas definicoes")
        self.region_of = [None] * mesh.F
        self.joints = joints
        self.new_cuts = {}
        self.stored_cuts = {}
        if reference is not None:
            self.take_labels(reference)
            return
        # O meiao: a pele da perna do topo da chuteira ate o joelho; abaixo do topo, o pe escondido.
        corpo = parts.index("corpo")
        for side, prefix in (("direito", "Right"), ("esquerdo", "Left")):
            boot = parts.index("chuteira_cabedal_" + ("direita" if side == "direito" else "esquerda"))
            boot_top = max(mesh.P[v][2] for t in range(mesh.F) if part[t] == boot for v in mesh.T[t])
            knee = joints[prefix + "Shin"][2]
            for t in range(mesh.F):
                c = mesh.C[t]
                if part[t] != corpo or (c[1] < 0.0) != (side == "direito") or c[2] > knee - defs["meiao"]["belowKnee"]:
                    continue
                z = mesh.P[mesh.T[t]][:, 2]
                top = boot_top + defs["meiao"]["aboveBoot"]
                if z.max() < top:
                    self.region_of[t] = "pe_oculto"
                elif z.min() <= top:
                    # A fileira que encosta na costura com o cabedal (<=: o vertice da costura no
                    # ponto mais alto do cabedal tambem e dela; senao o anel abre num disco).
                    self.panel_of[t] = "meiao_borda_" + side
                else:
                    self.panel_of[t] = "meiao_" + side
        for t in range(mesh.F):
            p = self.panel_of[t]
            if self.region_of[t]:
                continue
            if p is None:
                self.region_of[t] = "chuteira" if parts[part[t]].startswith("chuteira") else "pele"
            elif p.startswith("recuo_"):
                self.region_of[t] = "camisa_recuo" if p in ("recuo_gola", "recuo_barra") or p.startswith("recuo_punho") else "calcao_recuo"
            else:
                self.region_of[t] = "camisa" if p.startswith(("camisa_", "manga_")) else p.split("_")[0]
        self.index_panels()

    def index_panels(self):
        self.tris_of = defaultdict(list)
        for t, p in enumerate(self.panel_of):
            if p:
                self.tris_of[p].append(t)
        self.find_seams()

    def take_labels(self, reference):
        """O painel, a regiao e os cortes pela referencia: a topologia (o conjunto dos poligonos)
        tem que ser a mesma e o molde de cada face da roupa tem que bater."""
        by_key = {tuple(p["v"]): p for p in reference["polygons"]}
        if set(self.polygon_key) != set(by_key) or len(self.mesh.P) != reference["vertexCount"]:
            fail(f"a malha nao e a da referencia p{reference['parametrization']} (outra topologia): e preciso uma referencia nova (--nova-referencia)")
        for t in range(self.mesh.F):
            entry = by_key[self.polygon_key[t]]
            # (Os paineis do meiao sao da pele do corpo, sem molde; os outros sao moldes da construcao.)
            expected = None if entry["panel"] is None or entry["panel"].startswith("meiao_") else entry["panel"]
            if self.panel_of[t] != expected:
                fail(f"o molde do triangulo {t} ({self.panel_of[t]}) nao e o da referencia ({expected})")
            self.panel_of[t] = entry["panel"]
            self.region_of[t] = entry["region"]
        self.stored_cuts = reference.get("cuts", {})
        self.index_panels()

    # -------------------------------------------------- costuras
    def label(self, t):
        return self.panel_of[t] or self.region_of[t]

    def find_seams(self):
        mesh = self.mesh
        pairs = defaultdict(list)
        for e, ts in mesh.edge_tris.items():
            if len(ts) == 2:
                a, b = self.label(ts[0]), self.label(ts[1])
                if a != b:
                    pairs[tuple(sorted((a, b)))].append(e)
        self.seams = []
        collar = {v for t in self.tris_of["recuo_gola"] for v in mesh.T[t]}
        cos = {v for t in self.tris_of["recuo_cos"] for v in mesh.T[t]}
        for (a, b), edges in sorted(pairs.items()):
            for component in edge_components(edges):
                vs = {v for e in component for v in e}
                side = "direito" if np.mean(mesh.P[sorted(vs)][:, 1]) < 0.0 else "esquerdo"
                pair = {a, b}
                strips = [p for p in pair if p.startswith("recuo_")]
                if pair == {"camisa_frente", "camisa_costas"}:
                    name, kind = ("ombro_" if vs & collar else "lado_") + side, "costura"
                elif "camisa_frente" in pair and any(p.startswith("manga_") for p in pair):
                    name, kind = "cava_frente_" + side, "costura"
                elif "camisa_costas" in pair and any(p.startswith("manga_") for p in pair):
                    name, kind = "cava_costas_" + side, "costura"
                elif all(p.startswith("calcao_") for p in pair):
                    if pair == {"calcao_frente_direita", "calcao_frente_esquerda"}:
                        name = "calcao_meio_frente"
                    elif pair == {"calcao_costas_direita", "calcao_costas_esquerda"}:
                        name = "calcao_meio_costas"
                    else:
                        name = ("calcao_lado_" if vs & cos else "calcao_dentro_") + side
                    kind = "costura"
                elif pair == {"meiao_" + side, "meiao_borda_" + side}:
                    # (O mesmo tecido: a borda so e outro painel pela planificacao.)
                    name, kind = "borda_meiao_" + side, "costura"
                elif len(strips) == 1 and all(not p.startswith("recuo_") for p in pair - set(strips)) \
                        and all(p in self.panels for p in pair):
                    name, kind = {"recuo_gola": "gola", "recuo_barra": "barra", "recuo_cos": "cos"}.get(
                        strips[0], ("punho_" if "punho" in strips[0] else "barra_perna_") + side), "abertura"
                else:
                    name, kind = f"juncao_{a}_{b}", "juncao"
                self.seams.append({"id": name, "kind": kind, "panels": [a, b], "continuous": kind == "costura",
                                   "edges": [[int(u), int(v)] for u, v in component]})
        merged = {}
        for seam in self.seams:
            if seam["id"] in merged:
                into = merged[seam["id"]]
                if seam["kind"] == "costura":
                    fail(f"a costura {seam['id']} aparece em dois lugares")
                into["panels"] = sorted(set(into["panels"]) | set(seam["panels"]))
                into["edges"] = sorted(into["edges"] + seam["edges"])
            else:
                merged[seam["id"]] = seam
        self.seams = list(merged.values())

    def seam_edges(self, seam_id):
        found = [s for s in self.seams if s["id"] == seam_id]
        if not found:
            fail(f"costura {seam_id} nao existe")
        return {tuple(e) for e in found[0]["edges"]}

    # -------------------------------------------------- cortes dos tubos
    def axis_of(self, panel):
        side = "Right" if panel.endswith(("direita", "direito")) else "Left"
        if panel.startswith(("manga_", "recuo_punho")):
            a, b = self.joints[side + "UpperArm"], self.joints[side + "Forearm"]
            axis = unit(b - a)
            down = np.array([0.0, 0.0, -1.0])
            return a, axis, unit(down - np.dot(down, axis) * axis)
        if panel.startswith(("meiao_", "recuo_barra_")):
            a, b = (self.joints[side + "Shin"], self.joints[side + "Foot"]) if panel.startswith("meiao_") else \
                (self.joints[side + "Thigh"], self.joints[side + "Shin"])
            axis = unit(b - a)
            # Atras da perna no meiao; por dentro (para o meio do corpo) na barra da perna.
            want = np.array([-1.0, 0.0, 0.0]) if panel.startswith("meiao_") else np.array([0.0, 1.0 if side == "Right" else -1.0, 0.0])
            return a, axis, unit(want - np.dot(want, axis) * axis)
        center = self.joints["Pelvis"]
        return center, np.array([0.0, 0.0, 1.0]), np.array([-1.0, 0.0, 0.0])

    def tube_cut(self, panel, tris):
        """O corte de um tubo ou anel. Na manga, a coluna da construcao (estrutural: nao muda
        quando a malha muda um pouco); nos outros, o caminho de uma borda a outra pela direcao
        do painel (no meiao, a linha de tras: o painel e a carta abrem nela)."""
        mesh = self.mesh
        loops = chains([e for e in self.border_of(tris)])
        if len(loops) != 2:
            fail(f"{panel}: {len(loops)} bordas (um tubo tem duas)")
        if panel.startswith("manga_"):
            return self.sleeve_column(panel, tris)
        # Os outros cortes sao escolhidos uma vez (pela direcao) e gravados nas definicoes da versao:
        # uma reconstrucao compativel reaplica o mesmo (escolhido de novo, ele podia pular de coluna e
        # os triangulos vizinhos trocariam de lado, uma volta inteira na coordenada).
        stored = self.stored_cuts.get(panel)
        if stored is not None:
            cut = {edge_key(int(a), int(b)) for a, b in stored}
            self.check_cut(panel, tris, cut, loops)
            return cut
        if panel.startswith("meiao_borda_"):
            # A borda continua o corte do meiao (a carta da volta passa pelos dois): a aresta da
            # fileira que desce do pe do corte do meiao ate a costura, a mais alinhada ao eixo.
            above = panel.replace("meiao_borda_", "meiao_")
            ends = {int(v) for e in self.stored_cuts.get(above, self.new_cuts.get(above, [])) for v in e}
            top = [loop for loop in loops if ends & set(loop)]
            if len(top) != 1 or len(ends & set(top[0])) != 1:
                fail(f"{panel}: o corte do meiao nao chega a borda num vertice so")
            a = (ends & set(top[0])).pop()
            seam = set(loops[1]) if top[0] is loops[0] else set(loops[0])
            _, axis, _ = self.axis_of(panel)
            down = sorted({edge_key(a, int(v)) for t in tris if a in mesh.T[t] for v in mesh.T[t] if int(v) in seam},
                          key=lambda e: -abs(np.dot(unit(mesh.P[e[0]] - mesh.P[e[1]]), axis)))
            if not down:
                fail(f"{panel}: nenhuma aresta desce do corte do meiao")
            self.new_cuts[panel] = [list(down[0])]
            return {down[0]}
        point, axis, prefer = self.axis_of(panel)
        cut = set(cut_path(mesh, tris, loops[0], loops[1], prefer, point, axis))
        self.new_cuts[panel] = sorted([int(a), int(b)] for a, b in cut)
        return cut

    def check_cut(self, panel, tris, cut, loops):
        """O corte gravado ainda serve: arestas de dentro do painel, um caminho de uma borda a outra."""
        tset = set(tris)
        for e in cut:
            if sum(1 for u in self.mesh.edge_tris.get(e, []) if u in tset) != 2:
                fail(f"o corte gravado de {panel} nao serve nesta malha: e preciso uma versao nova das definicoes")
        lines = chains(list(cut))
        a, b = set(loops[0]), set(loops[1])
        if len(lines) != 1 or not ({lines[0][0], lines[0][-1]} & a and {lines[0][0], lines[0][-1]} & b):
            fail(f"o corte gravado de {panel} nao liga as duas bordas: e preciso uma versao nova das definicoes")

    def sleeve_column(self, panel, tris):
        """A coluna da manga que desce da axila (o vertice da frente, das costas e da manga) ate a
        barra, reta pela grade de quads: em cada vertice, a aresta oposta a que chegou (nenhum quad
        em comum com ela). E a coluna 0 dos aneis da construcao."""
        mesh = self.mesh
        side = "direito" if panel.endswith("direita") else "esquerdo"
        front = {v for e in self.seam_edges("cava_frente_" + side) for v in e}
        back = {v for e in self.seam_edges("cava_costas_" + side) for v in e}
        corner = front & back & {v for e in self.seam_edges("lado_" + side) for v in e}
        if len(corner) != 1:
            fail(f"{panel}: a axila nao e um vertice so ({len(corner)})")
        hem = {v for e in self.edges_between(panel, ["recuo_punho_" + side]) for v in e}
        tset = set(tris)
        polygon = self.data["trianglePolygon"]
        quads = defaultdict(set)  # aresta -> poligonos dela
        for t in tris:
            for k in range(3):
                quads[edge_key(int(mesh.T[t][k]), int(mesh.T[t][(k + 1) % 3]))].add(polygon[t])
        # As arestas de verdade da grade (nao a diagonal de um quad) e a borda do painel.
        real = {e for e, polys in quads.items() if len(polys) == 2 or
                sum(1 for u in mesh.edge_tris[e] if u in tset) == 1}
        border = {e for e in real if sum(1 for u in mesh.edge_tris[e] if u in tset) == 1}
        links = defaultdict(set)
        for a, b in real:
            links[a].add(b)
            links[b].add(a)
        here = next(iter(corner))
        first = [w for w in links[here] if edge_key(here, w) not in border]
        if len(first) != 1:
            fail(f"{panel}: a axila tem {len(first)} arestas para dentro da manga (a grade espera uma)")
        path, previous, here = [edge_key(here, first[0])], here, first[0]
        while here not in hem:
            coming = quads[edge_key(previous, here)]
            ahead = [w for w in links[here] if w != previous and not (quads[edge_key(here, w)] & coming)]
            if len(ahead) != 1:
                fail(f"{panel}: a coluna da manga se perde num vertice com {len(ahead)} saidas")
            previous, here = here, ahead[0]
            path.append(edge_key(previous, here))
            if len(path) > 64:
                fail(f"{panel}: a coluna da manga nao chega a barra")
        return set(path)

    def border_of(self, tris):
        tset = set(tris)
        out = []
        for t in tris:
            for k in range(3):
                e = edge_key(int(self.mesh.T[t][k]), int(self.mesh.T[t][(k + 1) % 3]))
                if sum(1 for u in self.mesh.edge_tris[e] if u in tset) == 1:
                    out.append(e)
        return out

    def back_center_cut(self, tris):
        """O meio das costas da camisa: as arestas no y = 0 do painel de costas."""
        mesh = self.mesh
        tset = set(tris)
        out = set()
        for t in tris:
            if self.panel_of[t] != "camisa_costas":
                continue
            for k in range(3):
                a, b = int(mesh.T[t][k]), int(mesh.T[t][(k + 1) % 3])
                if abs(mesh.P[a][1]) < 1e-6 and abs(mesh.P[b][1]) < 1e-6 and \
                        sum(1 for u in mesh.edge_tris[edge_key(a, b)] if u in tset) == 2:
                    out.add(edge_key(a, b))
        if not out:
            fail("o meio das costas nao tem arestas no y = 0")
        return out

    # -------------------------------------------------- marcos
    def chain_midpoint(self, origin, faces, U, edges_global):
        """O ponto no meio (pelo comprimento) da polilinha 2D das arestas de borda dadas."""
        local = [e for e in boundary_edges(faces) if edge_key(int(origin[e[0]]), int(origin[e[1]])) in edges_global]
        if not local:
            return None
        lines = chains(local)
        line = max(lines, key=len)
        pts = U[line]
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        half, run = seg.sum() / 2.0, 0.0
        for k, s in enumerate(seg):
            if run + s >= half:
                return pts[k] + (pts[k + 1] - pts[k]) * ((half - run) / max(s, 1e-12))
            run += s
        return pts[-1]

    def edges_between(self, panel, others):
        out = set()
        for s in self.seams:
            if panel in s["panels"] and (set(s["panels"]) - {panel}) & set(others):
                out |= {tuple(e) for e in s["edges"]}
        return out

    def landmarks_of(self, panel, origin, faces, U):
        """O marco de baixo (o meio da barra) e o de cima de cada painel, no 2D da planificacao."""
        side = "direito" if panel.endswith(("direita", "direito")) else "esquerdo"
        fem = "direita" if side == "direito" else "esquerda"
        if panel in ("camisa_frente", "camisa_costas"):
            mesh = self.mesh
            hem = {v for e in self.edges_between(panel, ["recuo_barra"]) for v in e}
            top = {v for e in self.edges_between(panel, ["recuo_gola"]) for v in e}
            pick = lambda group: next(i for i, v in enumerate(origin) if v in group and abs(mesh.P[v][1]) < 1e-6)
            return U[pick(hem)], U[pick(top)]
        if panel.startswith("manga_"):
            return (self.chain_midpoint(origin, faces, U, self.edges_between(panel, ["recuo_punho_" + side])),
                    self.chain_midpoint(origin, faces, U, self.edges_between(panel, ["camisa_frente", "camisa_costas"])))
        if panel.startswith("calcao_"):
            return (self.chain_midpoint(origin, faces, U, self.edges_between(panel, ["recuo_barra_" + fem])),
                    self.chain_midpoint(origin, faces, U, self.edges_between(panel, ["recuo_cos"])))
        if panel.startswith("recuo_"):
            # A borda com o pano (a abertura) embaixo; a outra (a pele ou a outra faixa) em cima.
            garment = [p for p in self.panels if not p.startswith(("recuo_", "meiao_"))]
            hem = self.edges_between(panel, garment)
            other = {tuple(e) for e in self.border_of(self.tris_of[panel])} - hem
            return self.chain_midpoint(origin, faces, U, hem), self.chain_midpoint(origin, faces, U, other)
        if panel.startswith("meiao_borda_"):
            return (self.chain_midpoint(origin, faces, U, self.edges_between(panel, ["chuteira", "pe_oculto"])),
                    self.chain_midpoint(origin, faces, U, self.edges_between(panel, ["meiao_" + side])))
        if panel.startswith("meiao_"):
            return (self.chain_midpoint(origin, faces, U, self.edges_between(panel, ["meiao_borda_" + side])),
                    self.chain_midpoint(origin, faces, U, self.edges_between(panel, ["pele"])))
        return None, None

    # -------------------------------------------------- paineis
    def flatten_panel(self, panel):
        tris = self.tris_of[panel]
        if not tris:
            fail(f"o painel {panel} nao tem triangulos")
        cut = set()
        tube = panel.startswith(("manga_", "meiao_", "recuo_"))
        if tube:
            cut = self.tube_cut(panel, tris)
        origin, faces, tris = local_mesh(self.mesh, tris, cut)
        V = self.mesh.P[origin]
        if panel.startswith("meiao_borda_"):
            # A fileira de triangulos finos entre a costura com o cabedal e o meiao: o LSCM/ARAP
            # vira as lascas (dois pares se sobrepunham). Desdobrada, ela fica exata.
            U = unfold_strip(V, faces)
        else:
            U = flatten(V, faces)
        low, high = self.landmarks_of(panel, origin, faces, U)
        if low is None or high is None:
            fail(f"{panel}: sem os marcos do referencial")
        U = oriented(U, low, high)
        return {"tris": tris, "origin": origin, "faces": faces, "U": U, "V": V}

    def corners_from(self, flat):
        out = {}
        for r, t in enumerate(flat["tris"]):
            out[t] = flat["U"][flat["faces"][r]]
        return out

    def mirrored(self, corners_right):
        """O painel da esquerda pelo da direita: o espelho exato, (x, y) -> (-x, y)."""
        mesh = self.mesh
        out = {}
        for t, U in corners_right.items():
            m = mesh.mirror_triangle(t)
            for k in range(3):
                partner = int(mesh.mirror[mesh.T[t][k]])
                km = list(mesh.T[m]).index(partner)
                out.setdefault(m, np.zeros((3, 2)))[km] = U[k] * np.array([-1.0, 1.0])
        return out

    def symmetrized(self, corners):
        mesh = self.mesh
        out = {}
        for t, U in corners.items():
            m = mesh.mirror_triangle(t)
            Um = corners[m]
            new = np.zeros((3, 2))
            for k in range(3):
                partner = int(mesh.mirror[mesh.T[t][k]])
                km = list(mesh.T[m]).index(partner)
                new[k] = 0.5 * (U[k] + Um[km] * np.array([-1.0, 1.0]))
            out[t] = new
        return out

    def build_design(self):
        self.design = {}
        self.flats = {}
        self.stats = {}
        done = set()
        for panel in self.panels:
            if panel in done:
                continue
            twin = panel.replace("direita", "esquerda").replace("direito", "esquerdo")
            if twin != panel:
                flat = self.flatten_panel(panel)
                corners = self.corners_from(flat)
                self.design.update(corners)
                self.design.update(self.mirrored(corners))
                done |= {panel, twin}
                self.flats[panel] = flat
            elif panel.endswith(("esquerda", "esquerdo")):
                continue
            else:
                flat = self.flatten_panel(panel)
                corners = self.corners_from(flat)
                if panel in ("camisa_frente", "camisa_costas"):
                    corners = self.symmetrized(corners)
                self.design.update(corners)
                done.add(panel)
                self.flats[panel] = flat
        for panel in self.panels:
            tris = self.tris_of[panel]
            values, areas = [], []
            for t in tris:
                s, a = stretch(self.mesh.P[self.mesh.T[t]], np.array([[0, 1, 2]]), self.design[t])
                values.append(s[0])
                areas.append(a[0])
            values, areas = np.array(values), np.array(areas)
            self.stats[panel] = {"p50": weighted_percentile(values, areas, 0.5), "p95": weighted_percentile(values, areas, 0.95),
                                 "max": float(values.max())}

    # -------------------------------------------------- cartas
    def build_charts(self):
        """As cartas por campos de direcao (Poisson): x segue o campo de "em volta" (ou o
        lateral, na sobre_ombro) e y o de "ao longo" (para cima no tronco, para o corpo nos
        membros, por cima do ombro na sobre_ombro). Os aros ficam horizontais em volta do
        tronco e as faixas dos membros perpendiculares a eles, coisa que a planificacao
        isometrica de um tubo cortado nao garante (a orientacao gira)."""
        mesh = self.mesh
        self.chart_corners = {}
        self.chart_stats = {}
        for chart in self.defs["charts"]:
            cid = chart["id"]
            if cid.endswith("esquerda") or cid.endswith("esquerdo"):
                continue
            tris = []
            for panel in chart["panels"]:
                tris += self.tris_of[panel]
            if "torsoReach" in chart:
                tris = self.reach_band(chart, tris)
            joined = set()
            for seam in chart["joined"]:
                joined |= self.seam_edges(seam)
            tset = set(tris)
            cut = set()
            for t in tris:
                for k in range(3):
                    e = edge_key(int(mesh.T[t][k]), int(mesh.T[t][(k + 1) % 3]))
                    others = [u for u in mesh.edge_tris[e] if u in tset and u != t]
                    if others and self.panel_of[others[0]] != self.panel_of[t] and e not in joined:
                        cut.add(e)
            if chart["cut"] == "meio_costas":
                cut |= self.back_center_cut(tris)
            elif chart["cut"] in ("coluna_manga", "linha_tras"):
                for tube in chart["panels"]:
                    if tube.startswith(("manga_", "meiao_")):
                        cut |= self.tube_cut(tube, self.tris_of[tube])
            origin, faces, tris = local_mesh(mesh, tris, cut)
            self.chart_tris = tris
            V = mesh.P[origin]
            pin = self.chart_pin(cid, chart, origin, faces, V)
            if chart["method"] == "isometrica":
                # (A da cava: um desenho local; o de cima e o alto do ombro, onde a costura
                # do ombro encontra a cava.)
                U = flatten(V, faces)
                side = "direito" if cid.endswith(("direita", "direito")) else "esquerdo"
                top = {v for e in self.seam_edges("ombro_" + side) for v in e} & {v for e in self.seam_edges("cava_frente_" + side) for v in e}
                if len(top) != 1:
                    fail(f"{cid}: o alto do ombro nao e um vertice so ({len(top)})")
                U = oriented(U, U[pin], U[next(i for i, v in enumerate(origin) if v in top)])
            else:
                ex, ey, weight = self.chart_fields(cid, chart, V, faces, origin)
                U = field_parametrization(V, faces, ex, ey, weight, pin)
            corners = {t: U[faces[r]] for r, t in enumerate(tris)}
            if cid in ("volta_tronco", "sobre_ombro"):
                corners = self.symmetrized(corners)
            values, areas = stretch(V, faces, U)
            self.chart_stats[cid] = {"p50": weighted_percentile(values, areas, 0.5), "p95": weighted_percentile(values, areas, 0.95),
                                     "max": float(values.max())}
            self.chart_corners[cid] = corners
            twin = cid.replace("direita", "esquerda").replace("direito", "esquerdo")
            if twin != cid:
                self.chart_corners[twin] = self.mirrored(corners)
                self.chart_stats[twin] = self.chart_stats[cid]

    def chart_axis(self, cid):
        """O eixo de "ao longo" de uma carta de membro, apontando para o corpo."""
        side = "Right" if cid.endswith(("direita", "direito")) else "Left"
        j = self.joints
        if cid.startswith(("volta_manga_", "ombro_manga_")):
            return unit(j[side + "UpperArm"] - j[side + "Forearm"])
        if cid.startswith("volta_perna_"):
            return unit(j[side + "Thigh"] - j[side + "Shin"])
        if cid.startswith("volta_meiao_"):
            return unit(j[side + "Shin"] - j[side + "Foot"])
        return np.array([0.0, 0.0, 1.0])

    def chart_fields(self, cid, chart, V, faces, origin):
        """Os campos por face (no plano dela) e o peso de cada face (a area, menos onde o
        campo projetado quase some: o alto do ombro no tronco, os lados na sobre_ombro)."""
        a, b, c = V[faces[:, 0]], V[faces[:, 1]], V[faces[:, 2]]
        n = np.cross(b - a, c - a)
        area = 0.5 * np.linalg.norm(n, axis=1)
        n = n / np.maximum(2.0 * area, 1e-16)[:, None]
        if chart["field"] == "lateral":
            # A posicao de um lado ao outro, por cima do ombro: no meio da frente e das costas
            # e no alto do ombro, a direcao lateral (+y); nos lados, onde ela some, a de "em
            # volta" com o sinal de cada painel (na frente ela ja e +y; nas costas, -y).
            want = np.array([0.0, 1.0, 0.0])
            lat = want - (n @ want)[:, None] * n
            up = np.array([0.0, 0.0, 1.0]) - (n @ np.array([0.0, 0.0, 1.0]))[:, None] * n
            around = np.cross(up, n)
            sign = np.array([1.0 if self.panel_of[t] == "camisa_frente" else -1.0 for t in self.chart_tris])[:, None]
            mixed = lat + sign * around
            size = np.linalg.norm(mixed, axis=1)
            ex = mixed / np.maximum(size, 1e-12)[:, None]
            ey = np.cross(n, ex)
            size = np.ones(len(faces))
        else:
            axis = self.chart_axis(cid)
            along = axis - (n @ axis)[:, None] * n
            size = np.linalg.norm(along, axis=1)
            ey = along / np.maximum(size, 1e-12)[:, None]
            ex = np.cross(ey, n)
        weight = area * np.clip(size / 0.5, 0.0, 1.0) ** 2 + 1e-9
        return ex, ey, weight

    def chart_pin(self, cid, chart, origin, faces, V):
        """O vertice da origem da carta: o meio da barra da frente no tronco, o meio da
        barra (o lado oposto ao corte) nos membros."""
        mesh = self.mesh
        if cid in ("volta_tronco", "sobre_ombro"):
            hem = {v for e in self.edges_between("camisa_frente", ["recuo_barra"]) for v in e}
            return next(i for i, v in enumerate(origin) if v in hem and abs(mesh.P[v][1]) < 1e-6)
        side = "direito" if cid.endswith(("direita", "direito")) else "esquerdo"
        fem = "direita" if side == "direito" else "esquerda"
        if cid.startswith(("volta_manga_", "ombro_manga_")):
            edges = self.edges_between(chart["panels"][0], ["recuo_punho_" + side])
        elif cid.startswith("volta_perna_"):
            edges = set()
            for panel in chart["panels"]:
                edges |= self.edges_between(panel, ["recuo_barra_" + fem])
        else:
            # (O meiao: a borda de baixo e a costura da borda da meia com o cabedal.)
            edges = self.edges_between(chart["panels"][-1], ["chuteira", "pe_oculto"])
        local = [e for e in boundary_edges(faces) if edge_key(int(origin[e[0]]), int(origin[e[1]])) in edges]
        line = max(chains(local), key=len)
        seg = np.linalg.norm(np.diff(V[line], axis=0), axis=1)
        run = np.concatenate([[0.0], np.cumsum(seg)])
        return line[int(np.argmin(np.abs(run - run[-1] / 2.0)))]

    def reach_band(self, chart, tris):
        """A manga e o tronco ate torsoReach metros da cava (pela malha)."""
        mesh = self.mesh
        sleeve = chart["panels"][0]
        scye = self.edges_between(sleeve, ["camisa_frente", "camisa_costas"])
        seeds = {v for e in scye for v in e}
        torso = [t for t in tris if self.panel_of[t] in ("camisa_frente", "camisa_costas")]
        allowed = {int(v) for t in torso for v in mesh.T[t]}
        best = {v: 0.0 for v in seeds}
        heap = [(0.0, v) for v in seeds]
        heapq.heapify(heap)
        while heap:
            d, v = heapq.heappop(heap)
            if d > best[v] or d > chart["torsoReach"]:
                continue
            for w in self.vertex_neighbors(v):
                if w not in allowed:
                    continue
                nd = d + float(np.linalg.norm(mesh.P[v] - mesh.P[w]))
                if nd < best.get(w, math.inf):
                    best[w] = nd
                    heapq.heappush(heap, (nd, w))
        band = [t for t in torso if all(best.get(v, math.inf) <= chart["torsoReach"] for v in self.polygon_key[t])]
        return self.tris_of[sleeve] + band

    def vertex_neighbors(self, v):
        if not hasattr(self, "_neighbors"):
            self._neighbors = defaultdict(set)
            for a, b in self.mesh.edge_tris:
                self._neighbors[a].add(b)
                self._neighbors[b].add(a)
        return self._neighbors[v]

    # -------------------------------------------------- marcos, ancoras e zonas
    def landmark(self, name, panel):
        mesh = self.mesh
        if name in ("gola_frente", "gola_costas"):
            group = {v for e in self.edges_between(panel, ["recuo_gola"]) for v in e}
            for t in self.tris_of[panel]:
                for k, v in enumerate(mesh.T[t]):
                    if v in group and abs(mesh.P[v][1]) < 1e-6:
                        return self.design[t][k].copy()
        if name == "topo":
            flat = self.flats.get(panel) or self.flats[panel.replace("esquerda", "direita")]
            high = self.chain_midpoint(flat["origin"], flat["faces"], flat["U"],
                                       self.edges_between(panel.replace("esquerda", "direita"), ["camisa_frente", "camisa_costas"]))
            return np.array([0.0, high[1]])
        if name == "numero_calcao":
            # O numero aprovado: 14,5 cm do meio para a esquerda, 6,43 cm acima da barra da perna.
            hem = {v for e in self.edges_between(panel, ["recuo_barra_esquerda"]) for v in e}
            best = None
            for t in self.tris_of[panel]:
                for k, v in enumerate(mesh.T[t]):
                    if v in hem:
                        d = abs(mesh.P[v][1] - 0.145)
                        if best is None or d < best[0]:
                            best = (d, self.design[t][k])
            return best[1] + np.array([0.0, 0.0643])
        fail(f"marco {name} desconhecido")

    def build_anchors_and_zones(self):
        self.anchors = []
        for a in self.defs["anchors"]:
            base = self.landmark(a["landmark"], a["panel"])
            self.anchors.append({"id": a["id"], "panel": a["panel"], "position": (base + np.array(a["offset"])).round(6).tolist(),
                                 "up": [0.0, 1.0], "size": a["size"]})
        self.zones = []
        layout = self.defs["identityLayout"]
        for z in self.defs["zones"]:
            base = self.landmark(z["landmark"], z["panel"])
            r = z["rect"]
            rect = [base[0] + r[0], base[1] + r[1], base[0] + r[2], base[1] + r[3]]
            self.zones.append({"id": z["id"], "panel": z["panel"], "rect": np.round(rect, 6).tolist(), "block": z["block"],
                               "inset": layout["inset"]})

    def classify_identity(self):
        """A zona de cada triangulo (por poligono: as faces que tocam o retangulo da zona)."""
        self.identity = [-1] * self.mesh.F
        groups = defaultdict(list)
        for t in range(self.mesh.F):
            groups[self.polygon_key[t]].append(t)
        for zi, z in enumerate(self.zones):
            rx0, ry0, rx1, ry1 = z["rect"]
            for tris in groups.values():
                if self.panel_of[tris[0]] != z["panel"]:
                    continue
                D = np.concatenate([self.design[t] for t in tris])
                if D[:, 0].max() < rx0 or D[:, 0].min() > rx1 or D[:, 1].max() < ry0 or D[:, 1].min() > ry1:
                    continue
                for t in tris:
                    self.identity[t] = zi

    # -------------------------------------------------- a referencia canonica
    def per_vertex(self, tris, key, corners):
        out = {}
        for t in tris:
            for k, v in enumerate(self.mesh.T[t]):
                v, c = int(v), np.asarray(corners[t][k])
                if v in out and np.abs(out[v] - c).max() > 1e-9:
                    fail(f"o vertice {v} tem duas coordenadas no mesmo poligono")
                out[v] = c
        return [np.round(out[v], 7).tolist() for v in key]

    def to_reference(self, version):
        """A referencia canonica: por poligono (os vertices dele), o painel, a regiao, a zona, a
        coordenada de desenho e as das cartas em cada vertice; a posicao de cada vertice; as ancoras,
        as zonas, os cortes e a distorcao medida."""
        groups = defaultdict(list)
        for t in range(self.mesh.F):
            groups[self.polygon_key[t]].append(t)
        chart_ids = [c["id"] for c in self.defs["charts"]]
        polygons = []
        for key in sorted(groups):
            tris = groups[key]
            t0 = tris[0]
            if any(self.panel_of[t] != self.panel_of[t0] or self.region_of[t] != self.region_of[t0] or self.identity[t] != self.identity[t0]
                   for t in tris):
                fail(f"o poligono {key} mistura paineis, regioes ou zonas")
            entry = {"v": list(key), "panel": self.panel_of[t0], "region": self.region_of[t0],
                     "identity": self.zones[self.identity[t0]]["id"] if self.identity[t0] >= 0 else None}
            if entry["panel"] is not None:
                entry["design"] = self.per_vertex(tris, key, self.design)
                charts = {}
                for cid in chart_ids:
                    inside = [t in self.chart_corners[cid] for t in tris]
                    if any(inside) and not all(inside):
                        fail(f"a carta {cid} corta o poligono {key}")
                    if all(inside):
                        charts[cid] = self.per_vertex(tris, key, self.chart_corners[cid])
                if charts:
                    entry["charts"] = charts
            polygons.append(entry)
        return {"schema": "matter.surface.reference.v1", "parametrization": version, "vertexCount": len(self.mesh.P),
                "reference": np.round(self.mesh.P, 7).tolist(), "cuts": dict(self.stored_cuts, **self.new_cuts),
                "polygons": polygons, "anchors": self.anchors, "zones": self.zones,
                "distortion": {"panels": self.stats, "charts": self.chart_stats}}

    def apply_reference(self, reference):
        """Tudo pela referencia (as posicoes de agora nao entram): a coordenada de cada canto vem do
        vertice dele no poligono, e a referencia 3D e a canonica."""
        mesh = self.mesh
        self.reference_positions = np.array(reference["reference"], float)
        by_key = {tuple(p["v"]): p for p in reference["polygons"]}
        chart_ids = [c["id"] for c in self.defs["charts"]]
        self.anchors, self.zones = reference["anchors"], reference["zones"]
        zone_index = {z["id"]: i for i, z in enumerate(self.zones)}
        self.design, self.chart_corners, self.identity = {}, {cid: {} for cid in chart_ids}, [-1] * mesh.F
        for t in range(mesh.F):
            entry = by_key[self.polygon_key[t]]
            at = {v: i for i, v in enumerate(entry["v"])}
            if entry["panel"] is not None:
                self.design[t] = np.array([entry["design"][at[int(v)]] for v in mesh.T[t]], float)
                for cid, coords in entry.get("charts", {}).items():
                    self.chart_corners[cid][t] = np.array([coords[at[int(v)]] for v in mesh.T[t]], float)
            if entry["identity"] is not None:
                self.identity[t] = zone_index[entry["identity"]]
        self.stats = reference["distortion"]["panels"]
        self.chart_stats = reference["distortion"]["charts"]

    # -------------------------------------------------- a injetividade
    def check_overlaps(self):
        """Nenhum mapa se sobrepoe nem degenera: a coordenada de desenho de cada painel, cada carta e o
        UV0 (todos juntos), pela intersecao exata de cada par de triangulos."""
        maps = [(f"o desenho do painel {panel}", {t: self.design[t] for t in self.tris_of[panel]}) for panel in self.panels]
        maps += [(f"a carta {cid}", corners) for cid, corners in self.chart_corners.items()]
        maps.append(("o UV0", self.kit_uv))
        for name, corners in maps:
            count, pairs, degenerate = overlaps(corners)
            if degenerate:
                fail(f"{name}: triangulos degenerados {degenerate[:4]}")
            if count:
                fail(f"{name} se sobrepoe: {count} pares de triangulos ({pairs})")

    # -------------------------------------------------- UV0 e UV1
    def bbox(self, panel):
        pts = np.concatenate([self.design[t] for t in self.tris_of[panel]])
        return pts.min(0), pts.max(0)

    def density_factor(self, panel):
        return STRIP_DENSITY if panel.startswith("recuo_") else 1.0

    def new_layout(self, storage, version):
        """As caixas dos paineis em prateleiras (das mais altas para as mais baixas; o empate
        pelo nome), na maior escala que cabe."""
        boxes = {p: self.bbox(p) for p in self.panels}
        margin = MARGIN_TEXELS / BASE_SIZE
        order = sorted(self.panels, key=lambda p: (-round((boxes[p][1][1] - boxes[p][0][1]) * self.density_factor(p), 6), p))

        def place(scale):
            x, y, row, out = margin, margin, 0.0, {}
            for p in order:
                lo, hi = boxes[p]
                w, h = (hi - lo) * scale * self.density_factor(p)
                if x + w + margin > 1.0:
                    x, y, row = margin, y + row + margin, 0.0
                if x + w + margin > 1.0 or y + h + margin > 1.0:
                    return None
                out[p] = [x, y, x + w, y + h]
                x += w + margin
                row = max(row, h)
            return out

        lo_s, hi_s = 0.1, 10.0
        for _ in range(60):
            mid = 0.5 * (lo_s + hi_s)
            if place(mid) is None:
                hi_s = mid
            else:
                lo_s = mid
        rects = place(lo_s)
        return {"version": storage, "parametrization": version, "baseSize": BASE_SIZE, "texelsPerMeter": round(lo_s * BASE_SIZE, 3),
                "panels": [{"panel": p, "rect": np.round(rects[p], 6).tolist(),
                            "bbox": np.round(np.concatenate(boxes[p]), 6).tolist(),
                            "density": self.density_factor(p)} for p in self.panels]}

    def build_uv(self, layout):
        self.kit_uv = {}
        scale = layout["texelsPerMeter"] / layout["baseSize"]
        for entry in layout["panels"]:
            p = entry["panel"]
            lo, hi = self.bbox(p)
            x0, y0, x1, y1 = entry["bbox"]
            if lo[0] < x0 - LAYOUT_TOLERANCE or lo[1] < y0 - LAYOUT_TOLERANCE or hi[0] > x1 + LAYOUT_TOLERANCE or hi[1] > y1 + LAYOUT_TOLERANCE:
                fail(f"o painel {p} nao cabe mais no retangulo da versao {layout['version']} do layout: e preciso uma versao nova (--novo-layout)")
            u0, v0 = entry["rect"][0], entry["rect"][1]
            s = scale * entry["density"]
            for t in self.tris_of[p]:
                D = self.design[t]
                self.kit_uv[t] = np.stack([u0 + (D[:, 0] - x0) * s, v0 + (y1 - D[:, 1]) * s], 1)
        self.identity_uv = {}
        W, H = self.defs["identityLayout"]["size"]
        for zi, z in enumerate(self.zones):
            rx0, ry0, rx1, ry1 = z["rect"]
            bx, by, bw, bh = z["block"]
            inset = z["inset"]
            for t in self.tris_of[z["panel"]]:
                if self.identity[t] != zi:
                    continue
                D = self.design[t]
                u = (bx + inset + (D[:, 0] - rx0) / (rx1 - rx0) * (bw - 2 * inset)) / W
                v = (by + inset + (ry1 - D[:, 1]) / (ry1 - ry0) * (bh - 2 * inset)) / H
                self.identity_uv[t] = np.stack([u, v], 1)

    # -------------------------------------------------- saida
    def write(self, out_path, layout, version):
        mesh, data = self.mesh, self.data
        chart_ids = [c["id"] for c in self.defs["charts"]]
        panel_index = {p: i for i, p in enumerate(self.panels)}
        region_index = {r: i for i, r in enumerate(self.regions)}
        r7 = lambda a: np.round(np.asarray(a, float), 7).tolist()
        charts = []
        for c in self.defs["charts"]:
            entry = {"id": c["id"], "panels": c["panels"], "joined": c["joined"], "cut": c["cut"], "method": c["method"]}
            if c.get("field"):
                entry["field"] = c["field"]
            charts.append(entry)
        data["surface"] = {
            "schema": self.defs["schema"], "parametrization": version, "storage": layout["version"],
            "panels": self.panels, "regions": self.regions, "charts": charts,
            "seams": self.seams, "anchors": self.anchors, "zones": self.zones,
            "storageLayout": layout, "identityLayout": self.defs["identityLayout"],
            "distortion": {"panels": self.stats, "charts": self.chart_stats},
        }
        data["trianglePanel"] = [panel_index[p] if p else -1 for p in self.panel_of]
        data["triangleRegion"] = [region_index[r] for r in self.region_of]
        data["triangleIdentity"] = self.identity
        data["triangleChartMask"] = [sum(1 << c for c, cid in enumerate(chart_ids) if t in self.chart_corners[cid]) for t in range(mesh.F)]
        data["triangleDesign"] = [r7(self.design[t]) if t in self.design else None for t in range(mesh.F)]
        data["triangleChart"] = {cid: {str(t): r7(U) for t, U in sorted(self.chart_corners[cid].items())} for cid in chart_ids}
        data["triangleKitUv"] = [r7(self.kit_uv[t]) if t in self.kit_uv else None for t in range(mesh.F)]
        data["triangleIdentityUv"] = [r7(self.identity_uv[t]) if t in self.identity_uv else None for t in range(mesh.F)]
        # A referencia 3D canonica de cada canto (a da referencia, nao a posicao de agora).
        data["triangleReference"] = [r7(self.reference_positions[mesh.T[t]]) if t in self.design else None for t in range(mesh.F)]
        with open(out_path, "w") as handle:
            json.dump(data, handle)


def write_json(path, value, indent=None):
    with open(path, "w") as handle:
        json.dump(value, handle, indent=indent, ensure_ascii=False)
        handle.write("\n")


def files():
    """As definicoes e os arquivos versionados (a pasta pode vir de MATTER_SUPERFICIE_DIR: as provas)."""
    folder = os.environ.get("MATTER_SUPERFICIE_DIR") or SUPERFICIE
    return (os.path.join(folder, "definicoes.json"), lambda n: os.path.join(folder, f"referencia_p{n}.json"),
            lambda n: os.path.join(folder, f"layout_s{n}.json"))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) != 2:
        sys.exit(__doc__)
    new_reference, new_layout = "--nova-referencia" in sys.argv, "--novo-layout" in sys.argv
    definitions, reference_file, layout_file = files()
    data = json.load(open(args[0]))
    defs = json.load(open(definitions))
    version, storage = defs.get("parametrization", 0), defs.get("storage", 0)
    if new_reference:
        # Uma referencia nova (replanificar e intencional): pela geometria de agora, com os cortes da
        # anterior quando a topologia e a mesma.
        template = Template(data, defs)
        if version and os.path.exists(reference_file(version)):
            previous = json.load(open(reference_file(version)))
            if {tuple(p["v"]) for p in previous["polygons"]} == set(template.polygon_key):
                template.stored_cuts = previous.get("cuts", {})
        template.build_design()
        template.build_charts()
        template.build_anchors_and_zones()
        template.classify_identity()
        version += 1
        reference = template.to_reference(version)
    else:
        if not version or not os.path.exists(reference_file(version)):
            fail("sem a referencia canonica: a primeira e com --nova-referencia")
        reference = json.load(open(reference_file(version)))
    # As saidas sempre pela referencia (a nova tambem: o que se grava e o que se le).
    template = Template(data, defs, reference)
    template.apply_reference(reference)
    if new_layout:
        storage += 1
        layout = template.new_layout(storage, version)
    else:
        if not storage or not os.path.exists(layout_file(storage)):
            fail("sem o layout do UV0: o primeiro e com --novo-layout")
        layout = json.load(open(layout_file(storage)))
        if layout["parametrization"] != version:
            fail(f"o layout s{storage} e da referencia p{layout['parametrization']}: uma referencia nova pede um layout novo (--novo-layout)")
    template.build_uv(layout)
    template.check_overlaps()
    template.write(args[1], layout, version)
    # So depois de tudo conferido: as versoes novas (as anteriores ficam).
    if new_reference:
        write_json(reference_file(version), reference)
    if new_layout:
        write_json(layout_file(storage), layout, indent=1)
    if new_reference or new_layout:
        defs["parametrization"], defs["storage"] = version, storage
        write_json(definitions, defs, indent=2)
        print(f"  versoes novas: referencia p{version}, layout s{storage} ({layout['texelsPerMeter']:.1f} texels/m no atlas de {layout['baseSize']})")
    worst = max(template.stats.items(), key=lambda kv: kv[1]["p95"])
    print(f"SUPERFICIE p{version} s{storage}: {len(template.panels)} paineis, {len(template.defs['charts'])} cartas, {len(template.seams)} costuras/aberturas/juncoes, "
          f"{len(template.anchors)} ancoras, {len(template.zones)} zonas; pior painel {worst[0]} p95 {worst[1]['p95']:.3f}")
    for name, st in list(template.stats.items()) + [("carta " + k, v) for k, v in template.chart_stats.items()]:
        print(f"  {name:28s} estiramento p50 {st['p50']:.3f} p95 {st['p95']:.3f} max {st['max']:.3f}")


if __name__ == "__main__":
    main()
