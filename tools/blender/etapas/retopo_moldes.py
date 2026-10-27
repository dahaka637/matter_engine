"""A retopologia das pecas do uniforme pelos moldes de uma roupa costurada (25/10/2026).

O QuadriFlow abaixo de ~2400 faces fechava os buracos das mangas, e os quads dele
nao seguiam as barras (a camisa saia com 2500 vertices, quatro vezes o corpo).
Aqui a peca e montada como a roupa de verdade: a camisa em frente, costas e duas
mangas (as costuras do lado, do ombro e da cava); o calcao em frente e costas de
cada perna (a costura do lado, a de dentro da perna e as do meio, que se
encontram no gancho). Cada molde e uma grade de quads: a interpolacao de Coons
entre as quatro bordas, levada a superficie e relaxada. As bordas sao cortes da
superficie por planos (o meio, o lado, o ombro e a cava, perpendicular ao braco)
e as barras originais, divididas por comprimento. Tudo e feito na metade direita
(x < 0 no Blender; a frente e o y negativo) e espelhado: a peca sai simetrica,
com os anéis paralelos as barras.

    malha = moldes(peca, superficie, barras, esqueleto, densidade)

Cada face sai com o molde dela no atributo de face "molde" (26/10/2026, o
template da superficie do sistema de aparencia): o indice em MOLDES, a mesma
tabela que a etapa 01 grava no .blend (texto "matter_moldes"). O espelho leva o
molde do outro lado (a manga direita vira a esquerda); a frente e as costas da
camisa sao um molde so dos dois lados do meio.
"""
import math

import bmesh
import mathutils
from mathutils.bvhtree import BVHTree

X = mathutils.Vector((1.0, 0.0, 0.0))
Z = mathutils.Vector((0.0, 0.0, 1.0))

# As divisoes de cada costura na densidade 1 (a escala multiplica todas). A gola
# (5 + 5 por metade) e a manga (8 + 8) dao os aneis da pele do jogador (20 e
# 16) e a perna do calcao (6 + 7) os 13 da coxa: a faixa da barra vira quads.
CAMISA = {"gola_frente": 5, "gola_costas": 5, "ombro": 2, "lado": 8, "cava_frente": 8, "cava_costas": 8, "manga": 4}
CALCAO = {"perna_frente": 6, "perna_costas": 7, "dentro": 3, "gancho": 6}
# O quanto as amostras da barra que seguem a pele vao para o espacamento igual
# dentro de cada trecho (entre os cantos dos moldes).
EVEN_WEIGHT = 0.5
# O quanto a cava entra (m) por metro de subida (a axila fica embaixo da junta do
# ombro; o fim do ombro, ~2 cm para dentro dela, 18 cm acima).
SCYE_TILT = 0.1
# O anel de apoio junto das barras da manga e da perna (m).
HEM_SUPPORT = 0.008
# A altura da fileira de cima dos moldes da camisa, junto da gola e do ombro (m).
TOP_GAP = 0.012
# Os moldes com nome (o indice e o valor do atributo "molde"; 0 = nenhum, a pele e
# o resto). As faixas do recuo (o ziper entre a barra e a pele, da etapa 01) tem o
# nome da abertura. So se acrescenta no fim: o indice de um nome nao muda.
MOLDES = ["nenhum", "camisa_frente", "camisa_costas", "manga_direita", "manga_esquerda",
          "calcao_frente_direita", "calcao_costas_direita", "calcao_frente_esquerda", "calcao_costas_esquerda",
          "recuo_gola", "recuo_barra", "recuo_punho_direito", "recuo_punho_esquerdo",
          "recuo_cos", "recuo_barra_direita", "recuo_barra_esquerda"]


def divisions(table, density):
    return {k: max(2, int(round(v * density))) for k, v in table.items()}


class Surface:
    """Os triangulos de uma parte da superficie, com a busca do ponto mais perto."""

    def __init__(self, tris):
        self.tris = tris
        verts, faces, index = [], [], {}
        for tri in tris:
            face = []
            for p in tri:
                key = (round(p.x, 7), round(p.y, 7), round(p.z, 7))
                if key not in index:
                    index[key] = len(verts)
                    verts.append(p.copy())
                face.append(index[key])
            faces.append(face)
        self.verts, self.faces = verts, faces
        self.tree = BVHTree.FromPolygons(verts, faces)

    def nearest(self, p):
        location = self.tree.find_nearest(p)[0]
        return location if location is not None else p

    def section(self, co, no):
        """As polilinhas do corte pelo plano (co, no)."""
        no = no.normalized()
        dist = [no.dot(v - co) for v in self.verts]
        dist = [d if abs(d) > 1e-9 else 1e-9 for d in dist]
        point, links, crossed = {}, {}, {}
        for f, face in enumerate(self.faces):
            crossings = []
            for k in range(3):
                a, b = face[k], face[(k + 1) % 3]
                if (dist[a] > 0.0) != (dist[b] > 0.0):
                    key = (min(a, b), max(a, b))
                    if key not in point:
                        t = dist[a] / (dist[a] - dist[b])
                        point[key] = self.verts[a].lerp(self.verts[b], t)
                    crossings.append(key)
            if len(crossings) == 2:
                p, q = crossings
                links.setdefault(p, []).append(q)
                links.setdefault(q, []).append(p)
                crossed[p] = crossed.get(p, []) + [f]
                crossed[q] = crossed.get(q, []) + [f]
        chains, seen = [], set()
        for start in sorted(links, key=lambda k: len(links[k])):
            if start in seen:
                continue
            chain, here, previous = [start], start, None
            seen.add(start)
            while True:
                nxt = next((n for n in links[here] if n != previous and n not in seen), None)
                if nxt is None:
                    closed = len(chain) > 2 and chain[0] in links[here]
                    break
                seen.add(nxt)
                chain.append(nxt)
                previous, here = here, nxt
            chains.append(([point[k].copy() for k in chain], closed, {f for k in chain for f in crossed[k]}))
        return chains

    def flood(self, seeds, blocked):
        """As faces ligadas as sementes por arestas, sem passar pelas bloqueadas."""
        edge_faces = {}
        for f, face in enumerate(self.faces):
            for k in range(3):
                key = (min(face[k], face[(k + 1) % 3]), max(face[k], face[(k + 1) % 3]))
                edge_faces.setdefault(key, []).append(f)
        reached, stack = set(), [f for f in seeds if f not in blocked]
        while stack:
            f = stack.pop()
            if f in reached:
                continue
            reached.add(f)
            face = self.faces[f]
            for k in range(3):
                key = (min(face[k], face[(k + 1) % 3]), max(face[k], face[(k + 1) % 3]))
                stack += [g for g in edge_faces[key] if g not in reached and g not in blocked]
        return reached


def locate(points, closed, p):
    """O lugar do ponto mais perto na polilinha: (indice do segmento, t, ponto)."""
    best = None
    count = len(points) if closed else len(points) - 1
    for i in range(count):
        a, b = points[i], points[(i + 1) % len(points)]
        ab = b - a
        t = max(0.0, min(1.0, (p - a).dot(ab) / max(1e-14, ab.dot(ab))))
        q = a + ab * t
        d = (q - p).length
        if best is None or d < best[0]:
            best = (d, i, t, q)
    return best[1], best[2], best[3]


def portion(points, closed, a, b, via=None):
    """O pedaco da polilinha entre os pontos mais perto de a e de b (numa volta,
    o lado que passa mais perto de via)."""
    ia, ta, pa = locate(points, closed, a)
    ib, tb, pb = locate(points, closed, b)
    n = len(points)

    def walk(i0, p0, i1, p1, step):
        out = [p0]
        i = i0
        if step > 0:
            while i != i1:
                i = (i + 1) % n
                out.append(points[i])
        else:
            i = (i0 + 1) % n
            while i != (i1 + 1) % n:
                out.append(points[i])
                i = (i - 1) % n
        out.append(p1)
        return out

    if not closed:
        if (ia, ta) <= (ib, tb):
            return [pa] + [points[k] for k in range(ia + 1, ib + 1)] + [pb]
        return ([pb] + [points[k] for k in range(ib + 1, ia + 1)] + [pa])[::-1]
    forward = walk(ia, pa, ib, pb, 1)
    backward = walk(ia, pa, ib, pb, -1)
    if via is None:
        return forward if length(forward) <= length(backward) else backward
    return forward if nearness(forward, via) <= nearness(backward, via) else backward


def nearness(line, via):
    """O quanto o caminho passa perto de via (sem as pontas, que os dois lados dividem)."""
    inner = line[1:-1]
    return min((q - via).length for q in inner) if inner else float("inf")


def length(line):
    return sum((line[k + 1] - line[k]).length for k in range(len(line) - 1))


def resample(line, segments):
    """segments + 1 pontos igualmente espacados (as pontas incluidas)."""
    total = length(line)
    if total < 1e-9:
        return [line[0].copy() for _ in range(segments + 1)]
    out, run, k = [line[0].copy()], 0.0, 0
    for s in range(1, segments):
        target = total * s / segments
        while k < len(line) - 1 and run + (line[k + 1] - line[k]).length < target:
            run += (line[k + 1] - line[k]).length
            k += 1
        step = (line[k + 1] - line[k]).length
        out.append(line[k].lerp(line[k + 1], (target - run) / max(step, 1e-12)))
    out.append(line[-1].copy())
    return out


def resample_end(line, segments, gap):
    """Como resample, mas o ultimo segmento com gap metros (o resto por igual):
    a fileira junto da borda fica baixa e acompanha a curva dela."""
    total = length(line)
    if segments < 2 or total <= 2.0 * gap:
        return resample(line, segments)
    run, k = 0.0, 0
    cut_at = total - gap
    head = [line[0].copy()]
    while k < len(line) - 1 and run + (line[k + 1] - line[k]).length < cut_at:
        run += (line[k + 1] - line[k]).length
        k += 1
        head.append(line[k].copy())
    step = (line[k + 1] - line[k]).length
    split = line[k].lerp(line[k + 1], (cut_at - run) / max(step, 1e-12))
    head.append(split)
    return resample(head, segments - 1) + [line[-1].copy()]


def underarm(scye):
    """Os trechos da cava, a partir da axila, que ficam quase deitados (o fundo
    dela, embaixo do braco): ate o primeiro que sobe mais de ~37 graus."""
    for i in range(1, len(scye) - 1):
        step = scye[i + 1] - scye[i]
        if step.z > 0.6 * step.length:
            return i
    return 1


def along(line, fraction):
    """O ponto na fracao do comprimento da polilinha e o indice do trecho dele."""
    target, run = length(line) * fraction, 0.0
    for k in range(len(line) - 1):
        step = (line[k + 1] - line[k]).length
        if run + step >= target:
            return line[k].lerp(line[k + 1], (target - run) / max(step, 1e-12)), k
        run += step
    return line[-1].copy(), len(line) - 2


def cut(surface, co, no, a, b, via):
    """A curva da superficie entre a e b no plano (co, no), pelo lado de via."""
    best = None
    for points, closed, _ in surface.section(co, no):
        if len(points) < 2:
            continue
        score = (locate(points, closed, a)[2] - a).length + (locate(points, closed, b)[2] - b).length
        if best is None or score < best[0]:
            best = (score, points, closed)
    if best is None:
        raise RuntimeError("corte vazio")
    line = portion(best[1], best[2], a, b, via)
    line[0], line[-1] = a.copy(), b.copy()
    return line


def at_heights(line, heights):
    """Os pontos da polilinha (subindo) nas alturas dadas."""
    out = []
    for h in heights:
        point = line[-1].copy()
        for k in range(len(line) - 1):
            a, b = line[k], line[k + 1]
            if (a.z - h) * (b.z - h) <= 0.0 and abs(b.z - a.z) > 1e-9:
                point = a.lerp(b, (h - a.z) / (b.z - a.z))
                break
        out.append(point)
    return out


def level_rows(center, side_points, rows):
    """A linha do meio dividida com as fileiras de baixo na altura das da
    costura do lado (horizontais, como na roupa de verdade: dividida por igual,
    as fileiras desciam em V ate o meio do peito) e o resto por igual ate a gola."""
    low = at_heights(center, [p.z for p in side_points])
    low[0] = center[0].copy()
    k = next((i for i in range(len(center) - 1) if center[i + 1].z >= low[-1].z), len(center) - 2)
    rest = [low[-1]] + center[k + 1:]
    return low + resample_end(rest, rows - (len(side_points) - 1), TOP_GAP)[1:]


def fractions(line):
    """A fracao do comprimento em cada ponto da polilinha (0 a 1)."""
    run, out = 0.0, [0.0]
    for k in range(len(line) - 1):
        run += (line[k + 1] - line[k]).length
        out.append(run)
    return [x / out[-1] if out[-1] > 1e-12 else k / max(1, len(line) - 1) for k, x in enumerate(out)]


def coons(bottom, top, left, right):
    """A grade (len(bottom) colunas x len(left) linhas) entre as quatro bordas,
    no parametro de cada borda pelo comprimento (uma fileira junto da barra fica
    junto dela no meio tambem)."""
    ni, nj = len(bottom) - 1, len(left) - 1
    fb, ft, fl, fr = fractions(bottom), fractions(top), fractions(left), fractions(right)
    grid = [[None] * (nj + 1) for _ in range(ni + 1)]
    for i in range(ni + 1):
        for j in range(nj + 1):
            v = (1.0 - i / ni) * fl[j] + (i / ni) * fr[j]
            u = (1.0 - j / nj) * fb[i] + (j / nj) * ft[i]
            p = (1 - v) * bottom[i] + v * top[i] + (1 - u) * left[j] + u * right[j]
            p -= (1 - u) * (1 - v) * bottom[0] + u * (1 - v) * bottom[ni] + (1 - u) * v * top[0] + u * v * top[ni]
            grid[i][j] = p
    for i in range(ni + 1):
        grid[i][0], grid[i][nj] = bottom[i].copy(), top[i].copy()
    for j in range(nj + 1):
        grid[0][j], grid[ni][j] = left[j].copy(), right[j].copy()
    return grid


def settle(grid, surface, rounds=12):
    """O miolo da grade na superficie, relaxado (a media dos quatro vizinhos e de
    volta a superficie); as bordas paradas."""
    ni, nj = len(grid) - 1, len(grid[0]) - 1
    for i in range(1, ni):
        for j in range(1, nj):
            grid[i][j] = surface.nearest(grid[i][j])
    for _ in range(rounds):
        moved = {}
        for i in range(1, ni):
            for j in range(1, nj):
                mean = (grid[i - 1][j] + grid[i + 1][j] + grid[i][j - 1] + grid[i][j + 1]) / 4.0
                moved[(i, j)] = surface.nearest(grid[i][j].lerp(mean, 0.5))
        for (i, j), p in moved.items():
            grid[i][j] = p
    return grid


def near_edge(edge, inner, gap, surface):
    """Uma fileira de apoio a gap metros da borda, na direcao da fileira de dentro
    (a barra da manga e da perna enrola para dentro nos ultimos milimetros e
    esconde a faixa; sem ela, a ultima fileira reta deixava a faixa a vista com o
    braco levantado)."""
    out = []
    for a, b in zip(edge, inner):
        step = b - a
        out.append(surface.nearest(a + step * min(0.45, gap / max(step.length, 1e-9))))
    return out


def tube(top_ring, bottom_ring, rows, surface):
    """Os anéis entre duas voltas com o mesmo numero de pontos (a manga), com o
    anel de apoio junto da barra (a de baixo)."""
    rings = [top_ring]
    for r in range(1, rows):
        t = r / rows
        rings.append([surface.nearest(a.lerp(b, t)) for a, b in zip(top_ring, bottom_ring)])
    rings.append(bottom_ring)
    count = len(top_ring)
    for _ in range(8):
        for r in range(1, rows):
            new = []
            for k in range(count):
                mean = (rings[r][(k - 1) % count] + rings[r][(k + 1) % count] + rings[r - 1][k] + rings[r + 1][k]) / 4.0
                new.append(surface.nearest(rings[r][k].lerp(mean, 0.5)))
            rings[r] = new
    rings.insert(len(rings) - 1, near_edge(bottom_ring, rings[-2], HEM_SUPPORT, surface))
    return rings


def follow(hem_points, seam_points):
    """As amostras da barra que seguem o anel da pele: cada vertice da pele no
    ponto mais perto da linha da barra, na mesma ordem (a faixa entre as duas
    vira uma tira de quads retos)."""
    return [locate(hem_points, True, q)[2] for q in seam_points]


def even_out(samples, dense, weight=EVEN_WEIGHT):
    """As amostras de um trecho da barra (as pontas paradas) a meio caminho
    (weight) do espacamento igual pela linha densa da barra entre elas: o anel
    da pele e irregular (no pescoco, dois vertices a 1 cm e saltos de 6 cm), e a
    gola que o seguia fazia um quad de 4,8 cm no canto apertado da gola."""
    if len(samples) < 3:
        return samples
    cum = [0.0]
    for k in range(len(dense) - 1):
        cum.append(cum[-1] + (dense[k + 1] - dense[k]).length)
    total = cum[-1]
    if total < 1e-9:
        return samples

    def param(p):
        best = None
        for k in range(len(dense) - 1):
            a, b = dense[k], dense[k + 1]
            ab = b - a
            t = max(0.0, min(1.0, (p - a).dot(ab) / max(1e-14, ab.dot(ab))))
            d = (a + ab * t - p).length
            if best is None or d < best[0]:
                best = (d, cum[k] + t * ab.length)
        return best[1]

    def point(u):
        k = max(0, min(len(dense) - 2, next((j for j in range(len(dense) - 1) if cum[j + 1] >= u), len(dense) - 2)))
        step = cum[k + 1] - cum[k]
        return dense[k].lerp(dense[k + 1], (u - cum[k]) / max(step, 1e-12))

    n = len(samples) - 1
    out = [samples[0].copy()]
    for i in range(1, n):
        out.append(point((1.0 - weight) * param(samples[i]) + weight * total * i / n))
    out.append(samples[-1].copy())
    return out


def ring_between(samples, start, end, via):
    """O pedaco da volta de amostras de start ate end (indices), pelo lado que passa perto de via."""
    n = len(samples)
    forward = [samples[(start + k) % n] for k in range((end - start) % n + 1)]
    backward = [samples[(start - k) % n] for k in range((start - end) % n + 1)]
    return forward if nearness(forward, via) <= nearness(backward, via) else backward


def on_loop(loop, key):
    return max(loop, key=key)


def mid(p):
    return mathutils.Vector((0.0, p.y, p.z))


def assemble(quads, labels):
    """A malha (bmesh) das quads da metade direita e do espelho, soldada nas
    costuras (os pontos de costura sao os mesmos numeros nos dois moldes).
    labels: o (molde da direita, molde da esquerda) de cada quad."""
    bm = bmesh.new()
    molde = bm.faces.layers.int.new("molde")
    index = {}

    def vert(p):
        key = (round(p.x, 6), round(p.y, 6), round(p.z, 6))
        if key not in index:
            index[key] = bm.verts.new(p)
        return index[key]

    for quad, (right, left) in zip(quads, labels):
        for side in (1.0, -1.0):
            points = [mathutils.Vector((side * p.x, p.y, p.z)) for p in quad]
            if side < 0.0:
                points = points[::-1]
            verts = list(dict.fromkeys(vert(p) for p in points))
            if len(verts) < 3:
                continue
            try:
                face = bm.faces.new(verts)
            except ValueError:
                continue
            # (A metade feita e a direita, x < 0 no Blender; o espelho e a esquerda.)
            face[molde] = MOLDES.index(right if side > 0.0 else left)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return bm


def grid_quads(grid):
    ni, nj = len(grid) - 1, len(grid[0]) - 1
    return [(grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]) for i in range(ni) for j in range(nj)]


def points_of(loop):
    return [v.co.copy() for v in loop]


def seam_for(hem, seams):
    if not seams:
        return None
    c = sum((v.co for v in hem), mathutils.Vector()) / len(hem)
    best = min(seams, key=lambda s: (sum((v.co for v in s), mathutils.Vector()) / len(s) - c).length)
    return [v.co.copy() for v in best]


def shirt(tris, hems, arm_head, arm_tail, density=1.0, seams=None):
    """A camisa: frente, costas e mangas (a metade direita, espelhada). Com os
    aneis da pele (seams), a gola e as mangas seguem eles."""
    d = divisions(CAMISA, density)
    right = [t for t in tris if sum(p.x for p in t) / 3.0 < 0.004]
    whole = Surface(right)
    collar = max(hems, key=lambda h: sum(v.co.z for v in h))
    waist = min(hems, key=lambda h: sum(v.co.z for v in h))
    sleeve = min((h for h in hems if h is not collar and h is not waist), key=lambda h: sum(v.co.x for v in h))
    collar_p, waist_p, sleeve_p = points_of(collar), points_of(waist), points_of(sleeve)
    near_mid = lambda pts, front: min((p for p in pts if abs(p.x) < 0.02), key=lambda p: p.y if front else -p.y)
    front_hint = lambda a, b: (a + b) / 2.0 + mathutils.Vector((0.0, -0.1, 0.0))
    back_hint = lambda a, b: (a + b) / 2.0 + mathutils.Vector((0.0, 0.1, 0.0))
    hem_front, hem_back = mid(near_mid(waist_p, True)), mid(near_mid(waist_p, False))
    collar_front, collar_back = mid(near_mid(collar_p, True)), mid(near_mid(collar_p, False))
    hem_side = min(waist_p, key=lambda p: p.x)
    collar_side = min(collar_p, key=lambda p: p.x)
    gola = None
    collar_seam = seam_for(collar, seams)
    if collar_seam:
        # A gola pelas projecoes do anel do pescoco: o meio da frente, o lado e
        # o meio de tras sao amostras dele (o canto do ombro no lado mais perto).
        samples = follow(collar_p, collar_seam)
        i_front = min((i for i, q in enumerate(samples) if q.y < collar_side.y), key=lambda i: (round(abs(samples[i].x), 4), samples[i].x))
        i_back = min((i for i, q in enumerate(samples) if q.y > collar_side.y), key=lambda i: (round(abs(samples[i].x), 4), samples[i].x))
        i_side = min(range(len(samples)), key=lambda i: samples[i].x)
        collar_front, collar_back, collar_side = mid(samples[i_front]), mid(samples[i_back]), samples[i_side].copy()
        gola = {"frente": ring_between(samples, i_front, i_side, front_hint(collar_front, collar_side)),
                "costas": ring_between(samples, i_back, i_side, back_hint(collar_back, collar_side))}
        for key, start, hint in (("frente", collar_front, front_hint), ("costas", collar_back, back_hint)):
            gola[key][0], gola[key][-1] = start.copy(), collar_side.copy()
            dense = portion(collar_p, True, start, collar_side, hint(start, collar_side))
            gola[key] = even_out(gola[key], dense)
            d["gola_" + key] = len(gola[key]) - 1
    else:
        gola = {"frente": resample(portion(collar_p, True, collar_front, collar_side, front_hint(collar_front, collar_side)), d["gola_frente"]),
                "costas": resample(portion(collar_p, True, collar_back, collar_side, back_hint(collar_back, collar_side)), d["gola_costas"])}
    arm = (arm_tail - arm_head).normalized()
    # A axila: no corte de frente pelo meio do braco, o contorno que sobe da
    # barra da cintura pelo lado do tronco e desce por baixo da manga ate a
    # barra dela; o alto dele.
    chains = whole.section(mathutils.Vector((0.0, arm_head.y, 0.0)), mathutils.Vector((0.0, 1.0, 0.0)))
    side_chain = min((c for c in chains if not c[1]), key=lambda c: min(p.z for p in c[0]) + 10.0 * min(abs(p.x - hem_side.x) for p in c[0]))
    armpit = max(side_chain[0], key=lambda p: p.z)
    # A cava: o corte por um plano quase em pe pela axila (com o y: sobe pela
    # frente ate o alto do ombro e desce por tras), inclinado SCYE_TILT para
    # dentro por metro de subida: abaixo da axila ele sai do tronco (o plano
    # perpendicular ao braco, que desce em diagonal, cortava o tronco de lado a
    # lado). O alto da volta e o fim do ombro.
    scye_normal = mathutils.Vector((SCYE_TILT, 0.0, 1.0)).normalized().cross(mathutils.Vector((0.0, 1.0, 0.0))).normalized()
    ring = min((c for c in whole.section(armpit + scye_normal * 1e-4, scye_normal) if c[1]),
               key=lambda c: min((p - armpit).length for p in c[0]))
    shoulder_top = max(ring[0], key=lambda p: p.z)
    # A manga e o resto: as faces ligadas a barra da manga sem passar pela volta
    # da cava (o plano dela tambem passa pela lateral do tronco abaixo da axila).
    hem_faces = {f for f, face in enumerate(whole.faces) if min((whole.verts[i] - q).length for i in face for q in sleeve_p) < 1e-5}
    beyond = whole.flood(hem_faces, ring[2])
    beyond |= {f for f in ring[2] if (sum((whole.verts[i] for i in whole.faces[f]), mathutils.Vector()) / 3.0 - armpit).dot(scye_normal) > 0.0}
    sleeve_tris = [tuple(whole.verts[i] for i in whole.faces[f]) for f in beyond]
    torso_tris = [tuple(whole.verts[i] for i in whole.faces[f]) for f in range(len(whole.faces)) if f not in beyond]
    # A barra da manga: as pontas de baixo (na linha da axila pelo braco) e de cima
    # (na do fim do ombro); com o anel da pele, as amostras seguem ele.
    bottom_ap = min(sleeve_p, key=lambda p: ((p - armpit) - arm * (p - armpit).dot(arm)).length)
    bottom_st = min(sleeve_p, key=lambda p: ((p - shoulder_top) - arm * (p - shoulder_top).dot(arm)).length)
    sleeve_seam = seam_for(sleeve, seams)
    if sleeve_seam:
        samples = follow(sleeve_p, sleeve_seam)
        i_ap = min(range(len(samples)), key=lambda i: (samples[i] - bottom_ap).length)
        i_st = min(range(len(samples)), key=lambda i: (samples[i] - bottom_st).length)
        hem_front_half = ring_between(samples, i_ap, i_st, front_hint(bottom_ap, bottom_st))
        hem_back_half = ring_between(samples, i_st, i_ap, back_hint(bottom_ap, bottom_st))
        hem_front_half = even_out(hem_front_half, portion(sleeve_p, True, hem_front_half[0], hem_front_half[-1], front_hint(bottom_ap, bottom_st)))
        hem_back_half = even_out(hem_back_half, portion(sleeve_p, True, hem_back_half[0], hem_back_half[-1], back_hint(bottom_ap, bottom_st)))
        d["cava_frente"], d["cava_costas"] = len(hem_front_half) - 1, len(hem_back_half) - 1
        hem_ring = hem_front_half + hem_back_half[1:-1]
    else:
        hem_front_half = portion(sleeve_p, True, bottom_ap, bottom_st, front_hint(bottom_ap, bottom_st))
        hem_back_half = portion(sleeve_p, True, bottom_st, bottom_ap, back_hint(bottom_ap, bottom_st))
        hem_ring = resample(hem_front_half, d["cava_frente"]) + resample(hem_back_half, d["cava_costas"])[1:-1]
    scye_front = portion(ring[0], True, armpit, shoulder_top, front_hint(armpit, shoulder_top))
    scye_back = portion(ring[0], True, shoulder_top, armpit, back_hint(armpit, shoulder_top))
    # (A volta passa a 0,1 mm da axila: as pontas nela, para soldar com o lado.)
    scye_front[0], scye_back[-1] = armpit.copy(), armpit.copy()
    # As costuras: o ombro (plano em pe pela gola e pelo fim do ombro), o lado
    # (plano pela barra e pela axila, com o x), o meio (x = 0).
    seam_shoulder = cut(whole, collar_side, (shoulder_top - collar_side).cross(Z), collar_side, shoulder_top,
                        (collar_side + shoulder_top) / 2.0 + Z * 0.05)
    seam_side = cut(whole, hem_side, (armpit - hem_side).cross(X), hem_side, armpit,
                    (hem_side + armpit) / 2.0 - X * 0.05)
    center_front = cut(whole, hem_front, X, hem_front, collar_front, (hem_front + collar_front) / 2.0 + mathutils.Vector((0, -0.2, 0)))
    center_back = cut(whole, hem_back, X, hem_back, collar_back, (hem_back + collar_back) / 2.0 + mathutils.Vector((0, 0.2, 0)))
    for line in (center_front, center_back):
        for p in line:
            p.x = 0.0
    hem_f = portion(waist_p, True, hem_front, hem_side, front_hint(hem_front, hem_side))
    hem_b = portion(waist_p, True, hem_back, hem_side, back_hint(hem_back, hem_side))
    # As divisoes (as costuras compartilhadas com os mesmos pontos nos dois moldes).
    side = resample(seam_side, d["lado"])
    shoulder = resample(seam_shoulder, d["ombro"])
    # (A fileira de cima dos moldes baixa, TOP_GAP, junto da gola e do ombro: a
    # de ~3,5 cm cortava a curva da borda no canto da gola e a pele do pescoco
    # aparecia por cima. Na cava so a posicao muda: a manga casa igual.)
    sf = resample_end(scye_front, d["cava_frente"], TOP_GAP)
    sb = resample_end(scye_back[::-1], d["cava_costas"], TOP_GAP)[::-1]
    quads, labels = [], []

    def add(new, right, left=None):
        quads.extend(new)
        labels.extend([(right, left or right)] * len(new))

    torso = Surface(torso_tris)
    axis_y = (hem_front.y + hem_back.y) / 2.0
    for name, hem_line, center, scye in (("frente", hem_f, center_front, sf), ("costas", hem_b, center_back, sb[::-1])):
        # A tira do lado, embaixo do braco: as colunas junto da costura do lado
        # acabam no fundo da cava (quase deitado, da axila ate onde ela sobe), e
        # nao no ombro. Num molde so, a axila era uma quina no meio da borda: as
        # colunas de cima se espremiam a 1 cm junto da cava e torciam (a lasca na
        # junta da manga com o tronco).
        # (O pe da divisa na barra onde as colunas da barra ficam todas iguais.)
        k = underarm(scye)
        corner = scye[k]
        columns = d["gola_" + name] + d["ombro"]
        foot, i_foot = along(hem_line, columns / (columns + k))
        # (O plano da divisa contem a direcao radial do tronco no meio dela, em
        # pe na superficie: com o eixo do lado, nas costas ele ficava quase
        # tangente e a curva entrava ate perto do meio.)
        middle = (foot + corner) / 2.0
        radial = mathutils.Vector((middle.x, middle.y - axis_y, 0.0)).normalized()
        split = cut(torso, foot, (corner - foot).cross(radial), foot, corner, middle + radial * 0.05)
        split_points = [foot] + at_heights(split, [p.z for p in side[1:-1]]) + [corner.copy()]
        hem_main, hem_strip = hem_line[:i_foot + 1] + [foot], [foot] + hem_line[i_foot + 1:]
        strip_bottom = resample(hem_strip, k)
        strip_top = [p.copy() for p in scye[k::-1]]
        strip_bottom[0], strip_bottom[-1] = split_points[0], side[0]
        add(grid_quads(settle(coons(strip_bottom, strip_top, split_points, side), torso)), "camisa_" + name)
        bottom = resample(hem_main, columns)
        top = [p.copy() for p in gola[name]] + shoulder[1:]
        right_side = split_points + scye[k + 1:]
        rows = len(right_side) - 1
        left = level_rows(center, split_points, rows)
        bottom[0], bottom[-1] = left[0], right_side[0]
        top[0], top[-1] = left[-1], right_side[-1]
        add(grid_quads(settle(coons(bottom, top, left, right_side), torso)), "camisa_" + name)
    # A manga: a volta da cava (frente da axila ao ombro e costas de volta) ate a barra.
    scye_ring = sf + sb[1:-1]
    rings = tube(scye_ring, hem_ring, d["manga"], Surface(sleeve_tris))
    count = len(scye_ring)
    for r in range(len(rings) - 1):
        add([(rings[r][k], rings[r][(k + 1) % count], rings[r + 1][(k + 1) % count], rings[r + 1][k]) for k in range(count)],
            "manga_direita", "manga_esquerda")
    info = {"axila": armpit, "ombro": shoulder_top, "gola_lado": collar_side, "barra_lado": hem_side,
            "divisoes": dict(d)}
    return assemble(quads, labels), info


def shorts(tris, hems, density=1.0, seams=None):
    """O calcao: frente e costas de cada perna (a metade direita, espelhada).
    Com o anel da coxa (seams), a barra da perna segue ele."""
    d = divisions(CALCAO, density)
    right = [t for t in tris if sum(p.x for p in t) / 3.0 < 0.004]
    whole = Surface(right)
    waist = max(hems, key=lambda h: sum(v.co.z for v in h))
    leg = min((h for h in hems if h is not waist), key=lambda h: sum(v.co.x for v in h))
    waist_p, leg_p = points_of(waist), points_of(leg)
    near_mid = lambda pts, front: min((p for p in pts if abs(p.x) < 0.02), key=lambda p: p.y if front else -p.y)
    waist_front, waist_back = mid(near_mid(waist_p, True)), mid(near_mid(waist_p, False))
    waist_side = min(waist_p, key=lambda p: p.x)
    leg_out, leg_in = min(leg_p, key=lambda p: p.x), max(leg_p, key=lambda p: p.x)
    # O gancho: o ponto mais baixo do corte pelo meio (x = 0).
    middle = max(whole.section(mathutils.Vector(), X), key=lambda c: length(c[0]))
    crotch = mid(min(middle[0], key=lambda p: p.z))
    rise_front = cut(whole, mathutils.Vector(), X, crotch, waist_front, (crotch + waist_front) / 2.0 + mathutils.Vector((0, -0.2, 0)))
    rise_back = cut(whole, mathutils.Vector(), X, crotch, waist_back, (crotch + waist_back) / 2.0 + mathutils.Vector((0, 0.2, 0)))
    for line in (rise_front, rise_back):
        for p in line:
            p.x = 0.0
    seam_side = cut(whole, leg_out, (waist_side - leg_out).cross(X), leg_out, waist_side, (leg_out + waist_side) / 2.0 - X * 0.05)
    seam_in = cut(whole, leg_in, (crotch - leg_in).cross(X), leg_in, crotch, (leg_in + crotch) / 2.0 + X * 0.03)
    front_hint = lambda a, b: (a + b) / 2.0 + mathutils.Vector((0.0, -0.1, 0.0))
    back_hint = lambda a, b: (a + b) / 2.0 + mathutils.Vector((0.0, 0.1, 0.0))
    hem_f = portion(leg_p, True, leg_in, leg_out, front_hint(leg_in, leg_out))
    hem_b = portion(leg_p, True, leg_in, leg_out, back_hint(leg_in, leg_out))
    leg_seam = seam_for(leg, seams)
    if leg_seam:
        samples = follow(leg_p, leg_seam)
        i_out = min(range(len(samples)), key=lambda i: samples[i].x)
        i_in = max(range(len(samples)), key=lambda i: samples[i].x)
        hem_f = ring_between(samples, i_in, i_out, front_hint(samples[i_in], samples[i_out]))
        hem_b = ring_between(samples, i_in, i_out, back_hint(samples[i_in], samples[i_out]))
        hem_f = even_out(hem_f, portion(leg_p, True, hem_f[0], hem_f[-1], front_hint(hem_f[0], hem_f[-1])))
        hem_b = even_out(hem_b, portion(leg_p, True, hem_b[0], hem_b[-1], back_hint(hem_b[0], hem_b[-1])))
        d["perna_frente"], d["perna_costas"] = len(hem_f) - 1, len(hem_b) - 1
    waist_f = portion(waist_p, True, waist_front, waist_side, front_hint(waist_front, waist_side))
    waist_b = portion(waist_p, True, waist_back, waist_side, back_hint(waist_back, waist_side))
    if leg_seam:
        leg_out, leg_in = samples[i_out].copy(), samples[i_in].copy()
        seam_side = cut(whole, leg_out, (waist_side - leg_out).cross(X), leg_out, waist_side, (leg_out + waist_side) / 2.0 - X * 0.05)
        seam_in = cut(whole, leg_in, (crotch - leg_in).cross(X), leg_in, crotch, (leg_in + crotch) / 2.0 + X * 0.03)
    inner = resample(seam_in, d["dentro"])
    side = resample(seam_side, d["dentro"] + d["gancho"])
    # (O apoio junto da barra da perna: um ponto a HEM_SUPPORT dela na costura de
    # dentro e na do lado; a fileira sai pela grade.)
    inner.insert(1, near_edge([inner[0]], [inner[1]], HEM_SUPPORT, whole)[0])
    side.insert(1, near_edge([side[0]], [side[1]], HEM_SUPPORT, whole)[0])
    quads, labels = [], []
    for name, hem_line, waist_line, rise, columns in (("frente", hem_f, waist_f, rise_front, d["perna_frente"]),
                                                      ("costas", hem_b, waist_b, rise_back, d["perna_costas"])):
        bottom = [p.copy() for p in hem_line] if leg_seam else resample(hem_line, columns)
        top = resample(waist_line, columns)
        left = inner + resample(rise, d["gancho"])[1:]
        bottom[0], bottom[-1] = left[0], side[0]
        top[0], top[-1] = left[-1], side[-1]
        new = grid_quads(settle(coons(bottom, top, left, side), whole))
        quads += new
        labels += [(f"calcao_{name}_direita", f"calcao_{name}_esquerda")] * len(new)
    return assemble(quads, labels), {"gancho": crotch, "cintura_lado": waist_side}


def moldes(piece, surface_bm, hems, rig, density=1.0, seams=None):
    """A peca nova (bmesh de quads, as bordas nas barras originais; com os aneis
    da pele, as barras que encostam neles seguem os vertices deles)."""
    tris = [tuple(l.vert.co.copy() for l in f.loops) for f in surface_bm.faces]
    if piece == "camisa":
        bone = rig.data.bones["RightUpperArm"]
        head = rig.matrix_world @ bone.head_local
        tail = rig.matrix_world @ bone.tail_local
        return shirt(tris, hems, head, tail, density, seams)
    return shorts(tris, hems, density, seams)
