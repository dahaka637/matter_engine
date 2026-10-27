"""Utilitarios de malha das etapas do jogador (22/10/2026): as voltas de
arestas, o alinhamento de duas voltas e o ziper otimo entre elas (com o teste de
cruzamento do motor), o baricentro."""
import bmesh
import mathutils
from mathutils.bvhtree import BVHTree


def loops_of(edges):
    """As voltas (listas ordenadas de vertices) de um conjunto de arestas."""
    left, loops = set(edges), []
    while left:
        start = left.pop()
        a, b = start.verts
        loop = [a, b]
        while True:
            here = loop[-1]
            nxt = next((e for e in here.link_edges if e in left), None)
            if nxt is None:
                break
            left.remove(nxt)
            other = nxt.other_vert(here)
            if other is loop[0]:
                break
            loop.append(other)
        loops.append(loop)
    return loops


def centroid(loop):
    return sum((v.co for v in loop), mathutils.Vector()) / len(loop)


def nearest_on_polyline(p, points):
    best, best_d = None, None
    for i in range(len(points)):
        a, b = points[i], points[(i + 1) % len(points)]
        ab = b - a
        t = max(0.0, min(1.0, (p - a).dot(ab) / max(1e-12, ab.dot(ab))))
        q = a + ab * t
        d = (p - q).length
        if best_d is None or d < best_d:
            best, best_d = q, d
    return best


def newell(loop):
    normal = mathutils.Vector()
    for i in range(len(loop)):
        a, b = loop[i].co, loop[(i + 1) % len(loop)].co
        normal += mathutils.Vector(((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y)))
    return normal


def aligned(loop, reference):
    """O loop no mesmo sentido de giro do de referencia (pela normal de Newell) e comecando no vertice mais perto do comeco dele."""
    if newell(loop).dot(newell(reference)) < 0.0:
        loop = loop[::-1]
    start = min(range(len(loop)), key=lambda i: (loop[i].co - reference[0].co).length)
    return loop[start:] + loop[:start]


def crossing(t, u):
    """O comprimento (m) da linha em que dois triangulos se cruzam (0 se nao se cruzam): a regua do motor."""
    na = (t[1] - t[0]).cross(t[2] - t[0])
    nb = (u[1] - u[0]).cross(u[2] - u[0])
    if na.length < 1e-14 or nb.length < 1e-14:
        return 0.0
    na.normalize()
    nb.normalize()
    line = na.cross(nb)
    if line.length_squared < 1e-6:
        return 0.0

    def span(tri, normal, origin):
        d = [normal.dot(p - origin) for p in tri]
        out = []
        for i in range(3):
            j = (i + 1) % 3
            if (d[i] > 0.0) == (d[j] > 0.0) or d[i] == d[j]:
                continue
            out.append(line.dot(tri[i] + (tri[j] - tri[i]) * (d[i] / (d[i] - d[j]))))
        return (min(out), max(out)) if out else None

    a, b = span(t, nb, u[0]), span(u, na, t[0])
    if a is None or b is None:
        return 0.0
    return max(0.0, min(a[1], b[1]) - max(a[0], b[0])) / line.length


def zipper(outer, inner, others, other_tris):
    """Os triangulos entre duas voltas alinhadas, o ziper otimo (programacao
    dinamica sobre os degraus, como na reconstrucao entre contornos): a soma das
    arestas novas minima, e cada triangulo que cruza o resto da malha (a pele,
    a outra peca, a fileira vizinha; pares sem vertice comum, como a analise do
    motor) custa mais que qualquer volta."""
    no, ni = len(outer), len(inner)

    def fraction(loop):
        steps = [(loop[k].co - loop[(k + 1) % len(loop)].co).length for k in range(len(loop))]
        total, run, out = sum(steps), 0.0, []
        for step in steps:
            out.append(run / total)
            run += step
        return out + [1.0]

    fo, fi = fraction(outer), fraction(inner)
    # Os candidatos (perto da diagonal): (i, j, 0) leva o degrau (i, j) a (i+1, j); (i, j, 1) a (i, j+1).
    cand, verts = [], []
    for i in range(no + 1):
        for j in range(ni + 1):
            if abs(fo[i] - fi[j]) > 0.2:
                continue
            if i < no:
                cand.append(((i, j, 0), (outer[i % no], outer[(i + 1) % no], inner[j % ni])))
            if j < ni:
                cand.append(((i, j, 1), (outer[i % no], inner[(j + 1) % ni], inner[j % ni])))
    index = {}
    for _, tri in cand:
        for v in tri:
            if v not in index:
                index[v] = len(verts)
                verts.append(v.co.copy())
    tree = BVHTree.FromPolygons(verts, [[index[v] for v in tri] for _, tri in cand])
    penalty = {}
    for c, o in tree.overlap(others):
        key, tri = cand[c]
        other = other_tris[o]
        if any(v is w or (v.co - w.co).length_squared < 1e-14 for v in tri for w in other):
            continue
        length = crossing([v.co for v in tri], [w.co for w in other])
        if length > 0.0:
            penalty[key] = penalty.get(key, 0.0) + 1.0 + 100.0 * length
    inf = float("inf")
    cost = [[inf] * (ni + 1) for _ in range(no + 1)]
    back = [[None] * (ni + 1) for _ in range(no + 1)]
    cost[0][0] = 0.0
    for i in range(no + 1):
        for j in range(ni + 1):
            here = cost[i][j]
            if here == inf:
                continue
            for di, dj, kind in ((1, 0, 0), (0, 1, 1)):
                a, b = i + di, j + dj
                if a > no or b > ni or abs(fo[a] - fi[b]) > 0.2:
                    continue
                step = here + (outer[a % no].co - inner[b % ni].co).length + penalty.get((i, j, kind), 0.0)
                if step < cost[a][b]:
                    cost[a][b], back[a][b] = step, (i, j, kind)
    tris, crossed, i, j = [], 0, no, ni
    while (i, j) != (0, 0):
        pi, pj, kind = back[i][j]
        tris.append((outer[pi % no], outer[(pi + 1) % no], inner[pj % ni]) if kind == 0
                    else (outer[pi % no], inner[(pj + 1) % ni], inner[pj % ni]))
        crossed += (pi, pj, kind) in penalty
        i, j = pi, pj
    return tris[::-1], cost[no][ni], crossed


def barycentric(p, a, b, c):
    v0, v1, v2 = b - a, c - a, p - a
    d00, d01, d11, d20, d21 = v0.dot(v0), v0.dot(v1), v1.dot(v1), v2.dot(v0), v2.dot(v1)
    den = d00 * d11 - d01 * d01
    if abs(den) < 1e-18:
        return 1.0, 0.0, 0.0
    v = (d11 * d20 - d01 * d21) / den
    w = (d00 * d21 - d01 * d20) / den
    return 1.0 - v - w, v, w


def untangle(mesh, movable, rounds=12):
    """Onde a peca nova se cruza ou cruza o resto (triangulada como o
    exportador; pares sem vertice comum), os vertices dela e duas voltas de
    vizinhos relaxam (a media dos vizinhos desdobra), ate nao cruzar mais."""
    for done in range(rounds):
        mesh.verts.index_update()
        mesh.verts.ensure_lookup_table()
        flat = mesh.copy()
        bmesh.ops.triangulate(flat, faces=flat.faces[:])
        tris = [tuple(v.index for v in f.verts) for f in flat.faces]
        tree = BVHTree.FromBMesh(flat)
        flat.verts.ensure_lookup_table()
        co = [v.co.copy() for v in flat.verts]
        flat.free()
        bad = set()
        for a, b in tree.overlap(tree):
            ta, tb = tris[a], tris[b]
            if a >= b or not any(mesh.verts[i] in movable for i in ta + tb):
                continue
            if any((co[x] - co[y]).length_squared < 1e-14 for x in ta for y in tb):
                continue
            if crossing([co[x] for x in ta], [co[y] for y in tb]) > 0.0:
                bad.update(ta + tb)
        if not bad:
            return done
        region = {mesh.verts[i] for i in bad}
        for _ in range(2):
            region |= {e.other_vert(v) for v in region for e in v.link_edges}
        region &= movable
        for _ in range(3):
            moved = {v: v.co.lerp(sum((e.other_vert(v).co for e in v.link_edges), mathutils.Vector()) / len(v.link_edges), 0.5)
                     for v in region if v.link_edges}
            for v, q in moved.items():
                v.co = q
    return rounds


def push_clear(mesh, movable, step=0.0015, limit=0.01, mirror=None, reach=0.0):
    """Onde a peca nova cruza o resto (ou a si mesma; triangulada como o
    exportador, pares sem vertice comum), os vertices dela nos triangulos que
    cruzam saem pela normal, step por vez, ate nao cruzar (no maximo limit): a
    malha grossa, plana entre pontos da superficie, afundava na pele logo abaixo
    (no canto da gola, a 8-11 mm). Com mirror (vertice -> par no espelho), o
    par sai junto: a malha fica simetrica. Com reach (m), os vizinhos ate essa
    distancia saem junto, menos quanto mais longe: um domo baixo em vez de um
    chifre (so os 4 vertices do canto da gola saindo 7 mm faziam um relevo
    pontudo). Devolve as rodadas e os que ainda cruzam."""
    moved = {}
    pool = [v for v in movable if v.is_valid]
    for done in range(int(limit / step) + 1):
        mesh.verts.index_update()
        mesh.verts.ensure_lookup_table()
        mesh.normal_update()
        flat = mesh.copy()
        bmesh.ops.triangulate(flat, faces=flat.faces[:])
        tris = [tuple(v.index for v in f.verts) for f in flat.faces]
        tree = BVHTree.FromBMesh(flat)
        flat.verts.ensure_lookup_table()
        co = [v.co.copy() for v in flat.verts]
        flat.free()
        bad = set()
        for a, b in tree.overlap(tree):
            ta, tb = tris[a], tris[b]
            if a >= b or not any(mesh.verts[i] in movable for i in ta + tb):
                continue
            if any((co[x] - co[y]).length_squared < 1e-14 for x in ta for y in tb):
                continue
            if crossing([co[x] for x in ta], [co[y] for y in tb]) > 0.0:
                bad.update(i for i in ta + tb if mesh.verts[i] in movable)
        if not bad:
            return done, 0
        pushed = 0
        targets = {mesh.verts[i] for i in bad}
        if mirror:
            targets |= {mirror[v] for v in list(targets) if v in mirror and mirror[v] in movable}
        share = {v: 1.0 for v in targets}
        if reach > 0.0:
            for v in pool:
                near = min((v.co - t.co).length for t in targets) / reach
                if near < 1.0:
                    share[v] = max(share.get(v, 0.0), (1.0 - near * near) ** 2)
        for v, w in share.items():
            if moved.get(v, 0.0) + step * w > limit + 1e-9 or v.normal.length < 1e-9:
                continue
            v.co = v.co + v.normal.normalized() * (step * w)
            moved[v] = moved.get(v, 0.0) + step * w
            pushed += 1
        if not pushed:
            return done, len(bad)
    return int(limit / step) + 1, -1

