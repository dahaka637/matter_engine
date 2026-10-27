"""O rosto pintado a mao no mapa da cabeca (22/10/2026): a base para retocar.

Cores lisas e bem delimitadas, cada traco no lugar da malha (as medidas em mm
foram tiradas do rosto de frente e de lado, numa grade sobre os guias):
 - a pele, com mais vermelho nas bochechas e na ponta do nariz;
 - a barba rala (o buco, o queixo, a mandibula e debaixo dela) e o cabelo
   raspado acima da linha do cabelo (que contorna as tempora, a costeleta, a
   orelha e a nuca);
 - os olhos (o branco no vao da palpebra, a iris, a pupila, os cilios de cima,
   a carúncula), as sobrancelhas e as narinas;
 - os labios nas faces da boca da malha (a cor ate as arestas) e a linha da
   boca na fresta;
 - a orelha no estilo antigo: o buraco do desenho do usuario (a camada
   "orelha" da pintura.json) levado a placa da orelha desta cabeca.
Os tracos do rosto valem so onde a frente ve a pele (o z-buffer de frente); as
narinas, por regiao da cabeca; os labios e a orelha, pelos triangulos deles.
A pele e simetrica.

    blender -b cabeca.blend --python tools/blender/pintar_rosto.py -- <saida.png> [lado em px, 2048]

O mapa sai no quadrado da metade direita do atlas (preto fora da ilha); a
etapa 06 o poe no atlas: 06_textura_pele.py -- --mapa <saida.png>.
"""
import json
import math
import sys
from pathlib import Path

import bmesh
import bpy
import numpy as np
from PIL import Image

argv = sys.argv[sys.argv.index("--") + 1:]
OUT = argv[0]
SIDE = int(argv[1]) if len(argv) > 1 else 2048
rng = np.random.default_rng(7)

body = bpy.data.objects["Jogador"]
rules = json.loads(bpy.data.texts["matter_troca"].as_string())
HEAD = [p["name"] for p in rules["parts"]].index("cabeca")
bm = bmesh.new()
bm.from_mesh(body.data)
bm.normal_update()
part = bm.faces.layers.int.get("parte")
uv_layer = bm.loops.layers.uv.get("atlas")
tris = []
for f in bm.faces:
    if f[part] != HEAD:
        continue
    loops = list(f.loops)
    for k in range(1, len(loops) - 1):
        t = (loops[0], loops[k], loops[k + 1])
        tris.append(([tuple(l.vert.co) for l in t], [tuple(l.vert.normal) for l in t], [tuple(l[uv_layer].uv) for l in t],
                     [l.vert.index for l in t]))
P = np.array([t[0] for t in tris])
N = np.array([t[1] for t in tris])
U = np.array([t[2] for t in tris])
I = np.array([t[3] for t in tris])
V = np.array([v.co[:] for v in bm.verts])
corners_a, corners_b, corners_c = P[:, 0], P[:, 1], P[:, 2]
face_n = np.cross(corners_b - corners_a, corners_c - corners_a)
face_area = 0.5 * np.linalg.norm(face_n, axis=1)
face_n /= np.maximum(1e-12, 2.0 * face_area)[:, None]
face_c = (corners_a + corners_b + corners_c) / 3.0


def raster(px, py, size, slack=0.0):
    for t in range(len(px)):
        x0, x1 = int(max(0, np.floor(px[t].min()))), int(min(size[0] - 1, np.ceil(px[t].max())))
        y0, y1 = int(max(0, np.floor(py[t].min()))), int(min(size[1] - 1, np.ceil(py[t].max())))
        if x1 < x0 or y1 < y0:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        (ax, bx, cx), (ay, by, cy) = px[t], py[t]
        den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(den) < 1e-12:
            continue
        l1 = ((by - cy) * (gx - cx) + (cx - bx) * (gy - cy)) / den
        l2 = ((cy - ay) * (gx - cx) + (ax - cx) * (gy - cy)) / den
        l3 = 1.0 - l1 - l2
        inside = (l1 >= -slack) & (l2 >= -slack) & (l3 >= -slack)
        if inside.any():
            yield t, gy[inside].astype(int), gx[inside].astype(int), np.stack([l1[inside], l2[inside], l3[inside]], 1)


# Os pixels do mapa (linha de cima para baixo) com o ponto e a normal de cada um.
UPX, UPY = (U[:, :, 0] - 0.5) * 2.0 * SIDE, (1.0 - U[:, :, 1]) * SIDE
rows, cols, pos, nor, tri_of = [], [], [], [], []
for t, r, c, lam in raster(UPX, UPY, (SIDE, SIDE), 0.01):
    rows.append(r), cols.append(c), pos.append(lam @ P[t]), nor.append(lam @ N[t]), tri_of.append(np.full(len(r), t))
rows, cols, tri_of = np.concatenate(rows), np.concatenate(cols), np.concatenate(tri_of)
pos, nor = np.concatenate(pos), np.concatenate(nor)
# O tamanho do pixel na superficie (mm), pelo triangulo de cada um.
uv_area = 0.5 * np.abs((UPX[:, 1] - UPX[:, 0]) * (UPY[:, 2] - UPY[:, 0]) - (UPX[:, 2] - UPX[:, 0]) * (UPY[:, 1] - UPY[:, 0]))
texel_mm = np.sqrt(face_area * 1e6 / np.maximum(1e-9, uv_area))[tri_of]
nor /= np.maximum(1e-9, np.linalg.norm(nor, axis=1))[:, None]
X, Y, Z = pos[:, 0], pos[:, 1], pos[:, 2]
ax = np.abs(X) * 1000.0          # mm do meio (a pele simetrica)
zm = Z * 1000.0                  # mm de altura

# O z-buffer de frente (a camera em -Y olhando +Y; 0,2 mm por pixel): o que a frente ve.
RES, XL, ZL = 0.0002, -0.11, 0.55
FW, FH = int(0.22 / RES), int(0.30 / RES)
front_depth = np.full((FH, FW), np.inf, np.float32)
fx, fz = (P[:, :, 0] - XL) / RES, (P[:, :, 2] - ZL) / RES
for t, r, c, lam in raster(fx, fz, (FW, FH)):
    np.minimum.at(front_depth, (r, c), lam @ P[t][:, 1])
fi = np.clip(((X - XL) / RES).astype(int), 0, FW - 1)
fj = np.clip(((Z - ZL) / RES).astype(int), 0, FH - 1)
seen = Y <= front_depth[fj, fi] + 0.0025


def rgb(*c):
    return np.array(c, np.float32) / 255.0


def smooth(edge0, edge1, x):
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def curve(points, x):
    """A curva por pontos (x crescente em mm), interpolada liso (monotono por trechos)."""
    xs, ys = np.array([p[0] for p in points], float), np.array([p[1] for p in points], float)
    return np.interp(x, xs, ys)


def blend(mask, color, amount=1.0):
    global paint
    a = np.clip(mask * amount, 0.0, 1.0)[:, None]
    paint = paint * (1.0 - a) + color * a


def blob(center, radius):
    d = np.linalg.norm((pos - np.array(center)) / np.array(radius), axis=1)
    return np.exp(-d * d)


def coverage(chosen, inside=None, k=4):
    """A fracao de cada pixel coberta pelos triangulos "chosen" (k x k amostras), so onde inside(pontos) vale."""
    cover = np.zeros((SIDE, SIDE), np.float32)
    sub = np.nonzero(chosen)[0]
    for t, r, c, lam in raster(UPX[sub] * k, UPY[sub] * k, (SIDE * k, SIDE * k)):
        keep = np.ones(len(r), bool) if inside is None else inside(lam @ P[sub[t]])
        np.add.at(cover, (r[keep] // k, c[keep] // k), 1.0 / (k * k))
    return np.minimum(cover[rows, cols], 1.0)


def inside_polygon(points, polygon):
    """Os pontos 2D dentro do poligono (a regra par-impar dos cruzamentos)."""
    x, y = points[:, 0], points[:, 1]
    inside = np.zeros(len(points), bool)
    for (xa, ya), (xb, yb) in zip(polygon, np.roll(polygon, -1, axis=0)):
        inside ^= ((ya > y) != (yb > y)) & (x < (xb - xa) * (y - ya) / (yb - ya + 1e-30) + xa)
    return inside


def to_polyline(points, where=None):
    """A distancia de cada pixel (ou dos pontos "where") a linha quebrada pelos pontos (3D ou 2D)."""
    where = pos if where is None else where
    best = np.full(len(where), np.inf)
    for a, b in zip(points[:-1], points[1:]):
        s = np.clip(((where - a) @ (b - a)) / ((b - a) @ (b - a)), 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(where - (a + s[:, None] * (b - a)), axis=1))
    return best


def vertices_at(points, label, tolerance=0.3):
    """Os vertices da cabeca em cada ponto (mm: do meio, Y, altura), nos dois lados (um so no meio)."""
    used = np.unique(I)
    found = set()
    for side_mm, y_mm, z_mm in points:
        near = np.linalg.norm(np.stack([np.abs(V[used, 0]) * 1000.0 - side_mm, V[used, 1] * 1000.0 - y_mm,
                                        V[used, 2] * 1000.0 - z_mm], 1), axis=1) < tolerance
        if near.sum() != (1 if side_mm == 0.0 else 2):
            sys.exit(f"a malha mudou: {int(near.sum())} vertices em {label} ({side_mm}, {y_mm}, {z_mm} mm)")
        found |= set(used[near].tolist())
    return found


def made_of(allowed):
    """Os triangulos com os tres cantos no conjunto de vertices."""
    return np.array([set(t) <= allowed for t in I.tolist()])


def side_chain(vertex_set, sign):
    """Os vertices de um lado (sign: -1 o direito, +1 o esquerdo), do meio para fora."""
    return V[sorted((v for v in vertex_set if V[v, 0] * sign > 0.0), key=lambda v: abs(V[v, 0]))]


# O angulo em volta do eixo da cabeca (0 de frente, 90 do lado, 180 atras) em graus.
turn = np.degrees(np.arctan2(np.abs(X), -(Y + 0.0054)))
fine = rng.standard_normal(len(pos)).astype(np.float32)

# 1. A pele: o tom da imagem do usuario, com mais vermelho onde o sangue aparece.
paint = np.tile(rgb(214, 166, 138), (len(pos), 1))
paint *= (1.0 + 0.018 * fine)[:, None]
warm = rgb(212, 142, 122)
blend(blob((0.045, -0.072, 0.668), (0.02, 0.03, 0.018)) + blob((-0.045, -0.072, 0.668), (0.02, 0.03, 0.018)), warm, 0.45)
blend(blob((0.0, -0.112, 0.667), (0.012, 0.012, 0.01)), warm, 0.5)

# 2. A barba rala: o buco, o queixo e a mandibula (a borda de cima bem suave
# na bochecha, a de baixo sumindo no pescoco), so pontilhada, sem mancha.
top = curve([(0, 655), (15, 648), (30, 632), (55, 636), (75, 655), (92, 688), (100, 700)], turn)
low = curve([(0, 590), (40, 596), (70, 610), (95, 632), (110, 660)], turn)
beard = smooth(-2, 9, top - zm) * smooth(-2, 12, zm - low) * smooth(-104, -96, -turn)
blend(beard, rgb(150, 122, 110), np.clip(0.14 + 0.16 * fine, 0.0, 0.45))

# 3. O cabelo: um degrade curto e crespo (o estilo da imagem do usuario, menos o
# pezinho de tras dela), a linha limpa e nitida.
#  - A linha na vista de lado (Y, altura; mm): a testa como era, a quina reta da
#    tempora, a tempora descendo ate a costeleta, o contorno da orelha (uns 5 mm
#    dela) e o lado da nuca atras da orelha, descendo ate o canto do pezinho.
#  - O pezinho de tras: reto a 640 mm, com os cantos arredondados (15 mm) e a borda
#    macia. O usuario tirou a faixa rala que ia ate a fileira de arestas a 620 mm:
#    atras o cabelo comeca ja no meio do degrade.
#  - O degrade: do ralo na linha (fios esparsos e a sombra da raiz) ao cheio uns 4 cm
#    acima, dos lados; atras ele sobe em S a partir de 621 mm (cortado no pezinho,
#    a 640). A testa e cheia desde a linha.
#  - Os fios: arcos curtos e finos deitados para longe do redemoinho (no alto de
#    tras), com a curva e o rumo soltos, em camadas, um pouco mais claros que a
#    raiz escura; sorteados na superficie (continuos na costura do mapa).
HAIRLINE = [(-200.0, 766.0), (-87.5, 765.8), (-82.5, 764.3), (-77.5, 762.8), (-72.5, 760.4), (-67.5, 756.8),
            (-62.5, 754.8), (-57.5, 753.0), (-52.5, 751.1), (-47.5, 750.0), (-42.5, 748.7), (-38.0, 747.6),
            (-38.0, 722.0), (-30.0, 714.0), (-20.0, 706.0), (-11.0, 700.0), (-6.5, 692.0), (1.5, 692.0),
            (3.5, 706.0), (6.0, 713.0), (12.0, 720.0), (20.0, 724.0), (30.0, 725.0), (40.0, 723.5), (48.0, 718.5),
            (53.0, 712.0), (55.0, 703.0), (55.5, 690.0)]
FADE_START = [(-46.0, 724.0), (-38.0, 724.0), (-26.0, 718.0), (-11.0, 712.0), (3.5, 712.0), (10.0, 718.0),
              (20.0, 722.0), (36.0, 723.0), (48.0, 717.0), (54.0, 706.0), (58.0, 684.0), (64.0, 652.0), (70.0, 630.0),
              (76.0, 621.0), (200.0, 621.0)]
NAPE_Z, ROUND = 640.0, 15.0                                          # o pezinho de tras e o raio dos cantos (mm)
corner_y = 58.6                                                       # o lado atras da orelha chega ao canto aqui
behind_ear = [(57.5, 666.0), (corner_y, NAPE_Z + ROUND)]
outline = np.array(HAIRLINE + behind_ear)
side_shape = np.array(HAIRLINE + behind_ear + [(corner_y, 900.0), (-200.0, 900.0)])
y_mm = Y * 1000.0
side_view = np.stack([y_mm, zm], 1)
behind = y_mm >= corner_y
corner_x = np.median(ax[(np.abs(y_mm - corner_y) < 0.5) & (np.abs(zm - NAPE_Z - ROUND) < 0.5)])
bow = np.clip(ax - (corner_x - ROUND), 0.0, ROUND)
nape_z = np.maximum(NAPE_Z, NAPE_Z + ROUND - np.sqrt(ROUND ** 2 - bow ** 2))
inside = (inside_polygon(side_view, side_shape) & ~behind) | (behind & (zm >= nape_z))
reach = np.minimum(to_polyline(outline, side_view), np.where(behind, np.abs(zm - nape_z), np.inf))
back = smooth(52.0, 68.0, y_mm)                                       # 1 atras, 0 dos lados
soft = 0.12 + 0.6 * back
hair = smooth(-soft, soft, np.where(inside, reach, -reach))
fade_on = smooth(-47.0, -39.0, y_mm)
start = 0.3 - 0.24 * back
ramp = smooth(0.0, 40.0 + 10.0 * back, zm - curve(FADE_START, y_mm))
density = 1.0 - fade_on * (1.0 - start) * (1.0 - ramp)

from scipy.spatial import cKDTree
strand_rng = np.random.default_rng(23)
grow = np.nonzero(face_c[:, 2] > 0.605)[0]
grow = np.repeat(grow, strand_rng.poisson(face_area[grow] * 1e6 * 1.6))
weights = strand_rng.dirichlet((1.0, 1.0, 1.0), len(grow))
seeds = np.einsum("ij,ijk->ik", weights, P[grow]) * 1000.0
normal = face_n[grow]
away = seeds - np.array([0.0, 40.0, 812.0])                         # o redemoinho, no alto de tras
away -= (away * normal).sum(1)[:, None] * normal
away /= np.maximum(1e-9, np.linalg.norm(away, axis=1))[:, None]
turn_by = strand_rng.normal(0.0, np.radians(28.0), len(grow))[:, None]
across = np.cos(turn_by) * away + np.sin(turn_by) * np.cross(normal, away)
side_way = np.cross(normal, across)
radius = strand_rng.uniform(1.5, 4.0, len(grow))
half_len = 0.5 * strand_rng.uniform(2.0, 3.2, len(grow))
bright = strand_rng.uniform(0.3, 0.95, len(grow))
exists = strand_rng.uniform(0.0, 1.0, len(grow))
bend = strand_rng.choice([-1.0, 1.0], len(grow))
zone = np.nonzero(hair > 0.0)[0]
strands = np.zeros(len(pos), np.float32)
tree = cKDTree(seeds)
for start in range(0, len(zone), 100000):
    part_idx = zone[start:start + 100000]
    here = pos[part_idx] * 1000.0
    gap, near = tree.query(here, k=20, distance_upper_bound=2.0)
    valid = np.isfinite(gap)
    near = np.where(valid, near, 0)
    offset = here[:, None, :] - seeds[near]
    a = (offset * across[near]).sum(-1)
    b = (offset * side_way[near]).sum(-1) * bend[near]
    r, dens, rear = radius[near], density[part_idx][:, None], back[part_idx][:, None]
    reach_phi = half_len[near] * (0.55 + 0.45 * dens) / r
    phi = np.arctan2(a, r - b)
    on_arc = np.abs(phi) <= reach_phi
    end_a, end_b = r * np.sin(reach_phi), r * (1.0 - np.cos(reach_phi))
    off = np.where(on_arc, np.abs(np.hypot(a, b - r) - r), np.minimum(np.hypot(a - end_a, b - end_b), np.hypot(a + end_a, b - end_b)))
    along = np.where(on_arc, np.sqrt(np.clip(1.0 - (phi / reach_phi) ** 2, 0.0, 1.0)), 0.0)
    width = np.maximum(0.15, 0.42 * texel_mm[part_idx])[:, None]
    lit = np.exp(-(off / width) ** 2) * (0.3 + 0.7 * along) * bright[near] * (exists[near] < (1.0 - rear) * (0.25 + 0.75 * dens) + rear * dens ** 0.8) * valid
    strands[part_idx] = 1.0 - np.prod(1.0 - np.clip(lit, 0.0, 1.0), axis=1)
blend(hair, rgb(38, 28, 22), (1.0 - back) * (0.2 + 0.72 * density ** 1.5) + back * (0.7 * density ** 1.2 + 0.22 * np.sqrt(density)))
blend(hair * strands, rgb(122, 92, 70), 0.9)

# 4. As sobrancelhas: um arco do meio para fora, grosso no comeco e fino na ponta.
brow_mid = curve([(12, 711.0), (20, 713.5), (28, 715.5), (36, 715.8), (44, 714.0), (52, 710.0)], ax)
brow_half = curve([(12, 2.6), (20, 2.8), (30, 2.4), (40, 1.9), (48, 1.3), (53, 0.8)], ax)
brow = smooth(-0.6, 0.6, brow_half - np.abs(zm - brow_mid)) * smooth(9.5, 14.5, ax) * smooth(-54.0, -51.0, -ax)
blend(brow * seen, rgb(70, 50, 40), 0.9 + 0.1 * np.clip(fine, -1, 1))

# 5. Os olhos nas faces do olho da malha (o usuario: a pintura "nos vertices DO
# OLHO"), cada face numa classe so, achadas pelos vertices medidos (mm: do meio,
# Y, altura):
#  - o globo (6 faces por olho) e a faixa de baixo (6, a parte de baixo do globo,
#    que se curva para tras ate a borda da palpebra de baixo): o branco, mais escuro
#    nos cantos e na sombra da palpebra de cima; a iris e a pupila no meio, cortadas
#    pela palpebra de cima e encostando na borda da de baixo; uma sombra fina na borda;
#  - a faixa de cima (6, a parte de baixo da palpebra de cima): os cilios;
#  - o canto de dentro (2): a caruncula. A dobra da palpebra na aresta de cima dela.
EYE = {"globo_alto": ((22.84, -79.31, 705.37), (32.70, -80.01, 706.01), (41.91, -74.02, 704.60)),
       "globo_baixo": ((22.63, -79.10, 699.43), (32.67, -78.91, 698.24), (41.91, -73.39, 699.53)),
       "canto_dentro": ((19.36, -76.59, 700.94),), "canto_fora": ((44.42, -66.28, 701.13),),
       "borda_cima": ((21.52, -81.78, 705.73), (32.90, -81.70, 707.06), (42.68, -75.32, 705.65)),
       "borda_baixo": ((22.49, -76.40, 696.37), (32.69, -75.38, 695.58), (41.23, -71.67, 696.59)),
       "caruncula": ((16.93, -77.35, 700.92),),
       "dobra": ((21.55, -82.70, 708.54), (32.39, -80.91, 710.89), (43.12, -74.54, 709.31))}
eye = {name: vertices_at(points, name) for name, points in EYE.items()}
ends = eye["canto_dentro"] | eye["canto_fora"]
globe = made_of(eye["globo_alto"] | eye["globo_baixo"] | ends)
lid_under = made_of(eye["globo_alto"] | eye["borda_cima"] | ends) & ~globe
lid_rim = made_of(eye["globo_baixo"] | eye["borda_baixo"] | ends) & ~globe
caruncle = made_of(eye["caruncula"] | eye["canto_dentro"] | eye["borda_cima"] | eye["borda_baixo"]) & ~(globe | lid_under | lid_rim)
if (globe.sum(), lid_under.sum(), lid_rim.sum(), caruncle.sum()) != (12, 12, 12, 4):
    sys.exit(f"as faces do olho mudaram: globo {globe.sum()}, cilios {lid_under.sum()}, borda {lid_rim.sum()}, caruncula {caruncle.sum()}")
lid_under_c, caruncle_c = coverage(lid_under), coverage(caruncle)
white = coverage(globe | lid_rim)
# A iris: no meio da aresta do meio do globo, 0,9 mm abaixo (para encostar na borda de baixo).
mid = V[[v for v in eye["globo_alto"] | eye["globo_baixo"] if abs(abs(V[v, 0]) * 1000.0 - 32.7) < 0.5]] * 1000.0
eye_x, eye_z = np.abs(mid[:, 0]).mean(), mid[:, 2].mean() - 0.9
eye_d = np.hypot(ax - eye_x, zm - eye_z)
top_edge = np.minimum(*(to_polyline(side_chain(eye["canto_dentro"] | eye["globo_alto"] | eye["canto_fora"], s)) for s in (-1.0, 1.0))) * 1000.0
lid_edge = np.minimum(*(to_polyline(side_chain(eye["canto_dentro"] | eye["borda_baixo"] | eye["canto_fora"], s)) for s in (-1.0, 1.0))) * 1000.0
fold = np.minimum(*(to_polyline(side_chain(eye["dobra"], s)) for s in (-1.0, 1.0))) * 1000.0
blend(white, rgb(226, 218, 210))
blend(white * smooth(4.5, 11.0, np.abs(ax - eye_x)), rgb(198, 182, 174), 0.7)                   # o branco mais escuro nos cantos
spokes = 0.5 + 0.25 * np.sin(np.arctan2(zm - eye_z, ax - eye_x) * 29.0) + 0.25 * np.sin(np.arctan2(zm - eye_z, ax - eye_x) * 13.0 + 1.3)
blend(white * smooth(-0.1, 0.1, 5.3 - eye_d), rgb(58, 78, 94))                                   # o anel de fora da iris
blend(white * smooth(-0.1, 0.1, 4.7 - eye_d), rgb(92, 120, 138) * (0.9 + 0.2 * spokes)[:, None])  # a iris, com raios
blend(white * smooth(3.2, 2.2, eye_d), rgb(128, 150, 156), 0.6)                                   # o anel claro perto da pupila
blend(white * smooth(-0.1, 0.1, 1.8 - eye_d), rgb(14, 12, 12))                                   # a pupila
blend(white * smooth(1.5, 0.2, top_edge), rgb(120, 98, 90), 0.6)                                 # a sombra da palpebra de cima
blend(white * smooth(0.6, 0.15, lid_edge), rgb(150, 112, 102), 0.45)                             # a sombra na borda de baixo
blend(lid_under_c, rgb(34, 25, 21))                                                              # os cilios
blend(caruncle_c, rgb(206, 132, 126), 0.9)                                                       # a caruncula
blend(smooth(0.9, 0.2, fold) * seen, rgb(170, 122, 104), 0.35)                                   # a dobra da palpebra

# 6. Os labios nas faces da boca da malha, a cor ate as arestas (o usuario: "pintado
# sob o vertice"): o de cima entre o alto do labio e a fresta, o de baixo entre a
# fresta e a borda de baixo, os dois ate o canto. A linha da boca segue a fresta
# de canto a canto. Os vertices (mm do meio, altura) medidos na frente da cabeca.
MOUTH = {"canto": ((33.26, 634.81),), "alto": ((0.0, 641.88), (7.68, 641.12)),
         "fresta": ((0.0, 633.17), (8.79, 635.76)), "baixo": ((0.0, 627.40), (10.89, 626.83))}
used = np.unique(I)
ring = {}
for name, points in MOUTH.items():
    ring[name] = set()
    for side_mm, height in points:
        hit = used[(np.abs(np.abs(V[used, 0]) * 1000.0 - side_mm) < 0.5) & (np.abs(V[used, 2] * 1000.0 - height) < 0.5)
                   & (V[used, 1] < -0.07)]
        if len(hit) != (1 if side_mm == 0.0 else 2):
            sys.exit(f"a boca da malha mudou: {len(hit)} vertices em {name} ({side_mm} mm, {height} mm)")
        ring[name] |= set(hit.tolist())
upper_tris = made_of(ring["canto"] | ring["alto"] | ring["fresta"])
lower_tris = made_of(ring["canto"] | ring["fresta"] | ring["baixo"])
upper_lip, lower_lip = coverage(upper_tris), coverage(lower_tris)
blend(upper_lip, rgb(176, 104, 98), 0.85)
blend(lower_lip, rgb(194, 122, 112), 0.85)
slit = V[sorted(ring["canto"] | ring["fresta"], key=lambda v: V[v, 0])]
mouth_line = smooth(-0.15, 0.15, 0.4 - to_polyline(slit) * 1000.0) * np.minimum(1.0, upper_lip + lower_lip)
blend(mouth_line, rgb(110, 62, 58), 0.9)

# 7. As narinas (debaixo do nariz, as faces viradas para baixo).
nostrils = blob((0.0085, -0.099, 0.6612), (0.0042, 0.0075, 0.0035)) + blob((-0.0085, -0.099, 0.6612), (0.0042, 0.0075, 0.0035))
blend(smooth(0.35, 0.6, nostrils) * smooth(0.2, 0.5, -nor[:, 2]), rgb(62, 34, 30), 0.95)

# 8. A orelha no estilo antigo (o usuario preferiu): a camada "orelha" da
# pintura.json, o buraco do desenho dele (a concha e o canal) multiplicando a pele
# pelo degrade, a beira um degrau nitido. Ela foi medida na caixa da placa da
# cabeca antiga (abaixo, de historico/jogador_05_candidato.blend: x para a frente
# e altura); cada pixel da placa desta volta para la pela normalizacao afim dos
# momentos das duas placas (o centro, o eixo maior para cima, o menor para a
# frente, os semieixos). A placa e a do motor (earPlate): os triangulos virados
# para o lado (a normal da face) com o centro a mais de 7,6 cm do meio, a maior
# peca ligada; a tinta fica so neles, nos dois lados.
OLD_BOX = (-0.039196976, 0.002550676, 0.667498529, 0.730631471)
OLD_CENTER, OLD_AXIS = np.array([-0.016974366, 0.698356144]), np.array([-0.278577711, 0.960413692])
OLD_SEMI = (0.032217831, 0.017679888)
layers = json.load(open(Path(__file__).resolve().parents[2] / "assets/characters/player_model/pintura.json"))["layers"]
ear_layer = next(layer for layer in layers if layer["name"] == "orelha")


def ear_plate(sign):
    """Os triangulos da placa de um lado (a maior peca ligada) e a volta da borda dela."""
    chosen = np.nonzero((sign * face_c[:, 0] > 0.076) & (sign * face_n[:, 0] > 0.6))[0]
    users = {}
    for k, t in enumerate(chosen):
        for i in range(3):
            users.setdefault(tuple(sorted((I[t][i], I[t][(i + 1) % 3]))), []).append(k)
    piece, areas = -np.ones(len(chosen), int), []
    for seed in range(len(chosen)):
        if piece[seed] >= 0:
            continue
        piece[seed], stack = len(areas), [seed]
        areas.append(0.0)
        while stack:
            k = stack.pop()
            areas[-1] += face_area[chosen[k]]
            for i in range(3):
                for other in users[tuple(sorted((I[chosen[k]][i], I[chosen[k]][(i + 1) % 3])))]:
                    if piece[other] < 0:
                        piece[other] = piece[seed]
                        stack.append(other)
    big = int(np.argmax(areas))
    following = {}
    for edge, ks in users.items():
        if sum(piece[k] == big for k in ks) == 1:
            following.setdefault(edge[0], []).append(edge[1])
            following.setdefault(edge[1], []).append(edge[0])
    start = previous = current = min(following)
    loop = []
    for _ in range(len(following) + 1):
        loop.append(current)
        step = following[current][0] if following[current][0] != previous else following[current][-1]
        previous, current = current, step
        if current == start:
            break
    return chosen[piece == big], np.array([[-V[v, 1], V[v, 2]] for v in loop])


def frame(axis):
    """O quadro da placa: o eixo maior para cima e o menor para a frente."""
    axis = axis if axis[1] > 0.0 else -axis
    return np.stack([axis, np.array([axis[1], -axis[0]])], 1)


def moments(outline):
    """O centro, o quadro e os semieixos do poligono (a elipse de mesma area e momentos)."""
    x0, h0 = outline[:, 0], outline[:, 1]
    x1, h1 = np.roll(x0, -1), np.roll(h0, -1)
    c = x0 * h1 - x1 * h0
    area = 0.5 * c.sum()
    cx, ch = ((x0 + x1) * c).sum() / (6.0 * area), ((h0 + h1) * c).sum() / (6.0 * area)
    vx = ((x0 * x0 + x0 * x1 + x1 * x1) * c).sum() / (12.0 * area) - cx * cx
    vh = ((h0 * h0 + h0 * h1 + h1 * h1) * c).sum() / (12.0 * area) - ch * ch
    cv = ((x0 * h1 + 2.0 * x0 * h0 + 2.0 * x1 * h1 + x1 * h0) * c).sum() / (24.0 * area) - cx * ch
    w, vec = np.linalg.eigh(np.array([[vx, cv], [cv, vh]]))
    return np.array([cx, ch]), frame(vec[:, 1]), (2.0 * np.sqrt(w[1]), 2.0 * np.sqrt(w[0]))


right_plate, outline = ear_plate(-1.0)
left_plate, _ = ear_plate(1.0)
new_center, new_frame, new_semi = moments(outline)
back = frame(OLD_AXIS) @ np.diag([OLD_SEMI[0] / new_semi[0], OLD_SEMI[1] / new_semi[1]]) @ new_frame.T
if np.linalg.det(back) <= 0.0:
    sys.exit("a placa da orelha saiu espelhada")


def ear_space(points):
    """O ponto da placa no espaco "orelha" da placa antiga (0 a 1 na caixa dela)."""
    old = (np.stack([-points[:, 1], points[:, 2]], 1) - new_center) @ back.T + OLD_CENTER
    return np.stack([(old[:, 0] - OLD_BOX[0]) / (OLD_BOX[1] - OLD_BOX[0]), (old[:, 1] - OLD_BOX[2]) / (OLD_BOX[3] - OLD_BOX[2])], 1)


plate = np.zeros(len(P), bool)
plate[right_plate] = plate[left_plate] = True
hole_points = np.array(ear_layer["shape"]["points"])
hole = coverage(plate, lambda points: inside_polygon(ear_space(points), hole_points))
gradient = ear_layer["gradient"]
uv = ear_space(pos)
reach = np.hypot((uv[:, 0] - gradient["center"][0]) / gradient["radius"][0], (uv[:, 1] - gradient["center"][1]) / gradient["radius"][1])
stops = gradient["stops"]
shade = np.stack([np.interp(reach, [s[0] for s in stops], [s[1][k] / 255.0 for s in stops]) for k in range(3)], 1)
paint = paint * (1.0 - hole[:, None] * (1.0 - shade))

image = np.zeros((SIDE, SIDE, 3), np.float32)
image[rows, cols] = np.clip(paint, 0.0, 1.0)
Image.fromarray((image * 255.0 + 0.5).astype(np.uint8)).save(OUT)
print(f"ROSTO PINTADO: {len(pos)} pixels; olhos {int(globe.sum())}+{int(lid_under.sum())}+{int(lid_rim.sum())}+{int(caruncle.sum())} "
      f"triangulos (globo, cilios, borda, caruncula), {len(seeds)} fios, labios {int(upper_tris.sum())}+{int(lower_tris.sum())} "
      f"triangulos ({int(((upper_lip + lower_lip) > 0.5).sum())} px), orelha {len(right_plate)}+{len(left_plate)} triangulos "
      f"({int((hole > 0.5).sum())} px no buraco), cabelo {int((hair > 0.5).sum())}, barba {int((beard > 0.5).sum())} -> {OUT}")
bm.free()
