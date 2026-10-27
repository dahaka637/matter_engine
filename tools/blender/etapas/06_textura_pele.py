"""Etapa 6 (22/10/2026): a pele.

A cabeca e pintada de um de dois jeitos:
 - por fotos (a base): imagens geradas sobre renders ortograficos da propria
   cabeca (o rosto, o perfil direito e as costas no mesmo enquadramento, a camera
   de cada um em guias.json). Cada pixel do mapa da cabeca vira o ponto dela na
   ligacao e pega a cor das imagens que o veem, pesadas pelo quanto a superficie
   olha para cada uma (e so onde a imagem ve o ponto: o teste de profundidade
   tira a nuca da imagem do rosto e o atras da orelha). A pele sai simetrica: o
   lado esquerdo copia o direito (o lado que o perfil mostra). As imagens do lado
   e de costas tem a cor igualada a do rosto onde os dois veem bem; o que nenhuma
   ve (dentro da orelha, debaixo do queixo) pega a cor do ponto pintado mais perto;
 - por um mapa pronto (--mapa): a imagem quadrada do mapa da cabeca (a base
   retocada), no lugar da ilha dela na metade direita do atlas.
A pele do corpo (bracos, pernas, maos) ganha o tom do pescoco da cabeca e o
grao da pele da bochecha (o detalhe fino de um quadrado do mapa, sem emenda).

    blender -b jogador.blend --python tools/blender/etapas/06_textura_pele.py -- <pasta das fotos> [saida.blend]
    blender -b jogador.blend --python tools/blender/etapas/06_textura_pele.py -- --mapa <mapa.png> [--borda 1] [saida.blend]

Na pasta das fotos: guias.json, foto_frente.png, foto_direita.png, foto_costas.png.
"""
import json
import os
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Vector
from mathutils.kdtree import KDTree
from PIL import Image, ImageFilter

argv = sys.argv[sys.argv.index("--") + 1:]
# --borda: os pixels da beira da ilha que nao valem (a cor do miolo se estende
# sobre eles). 4 para imagens geradas (a beira escurece no preto de fora); 1
# para a pintura a mao (com 4, a costura da nuca virava uma faixa lisa).
EDGE = 4
if "--borda" in argv:
    k = argv.index("--borda")
    EDGE, argv = int(argv[k + 1]), argv[:k] + argv[k + 2:]
MAP = argv[1] if argv[0] == "--mapa" else None
if MAP:
    argv = argv[2:]
else:
    FOLDER, argv = argv[0], argv[1:]
SHARP = 6.0          # o quanto a superficie de frente para a imagem manda
SIZE = 1024          # a resolucao das fotos (a dos guias)

body = bpy.data.objects["Jogador"]
rules = json.loads(bpy.data.texts["matter_troca"].as_string())
names = [p["name"] for p in rules["parts"]]
HEAD, SKIN = names.index("cabeca"), names.index("corpo")
SKIN_MATERIAL = next(i for i, m in enumerate(rules["materials"]) if m["name"] == "pele")

bm = bmesh.new()
bm.from_mesh(body.data)
part = bm.faces.layers.int.get("parte")
uv_layer = bm.loops.layers.uv.get("atlas")
bm.normal_update()
texture = rules["textures"][0]
image = bpy.data.images.get(texture) or next(i for i in bpy.data.images if i.filepath.endswith(texture))
W, H = image.size
if W != 2 * H:
    sys.exit(f"o atlas {W}x{H} nao tem a metade da cabeca (rode a etapa 05 antes)")
pixels = np.array(image.pixels[:], np.float32).reshape(H, W, 4)

# A cabeca em triangulos (posicao e normal lisa por canto, o mapa por canto).
head_tris = []
for f in bm.faces:
    if f[part] != HEAD:
        continue
    loops = list(f.loops)
    for k in range(1, len(loops) - 1):
        tri = (loops[0], loops[k], loops[k + 1])
        head_tris.append(([tuple(l.vert.co) for l in tri], [tuple(l.vert.normal) for l in tri], [tuple(l[uv_layer].uv) for l in tri]))
P = np.array([t[0] for t in head_tris])   # (n, 3, 3)
N = np.array([t[1] for t in head_tris])
U = np.array([t[2] for t in head_tris])   # (n, 3, 2)


def raster(px, py, slack):
    """Os pixels de cada triangulo (x, y por canto): linha, coluna e os pesos dos cantos."""
    for t in range(len(px)):
        x0, x1 = int(max(0, np.floor(px[t].min()))), int(np.ceil(px[t].max()))
        y0, y1 = int(max(0, np.floor(py[t].min()))), int(np.ceil(py[t].max()))
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


def island_core(erode):
    """Os pixels do quadrado da cabeca dentro da ilha, menos `erode` pixels da borda."""
    core = np.zeros((H, H), bool)
    for t, rows, cols, lam in raster(U[:, :, 0] * W - H, U[:, :, 1] * H, 0.0):
        keep = (rows < H) & (cols < H)
        core[rows[keep], cols[keep]] = True
    for _ in range(erode):
        core = core & np.roll(core, 1, 0) & np.roll(core, -1, 0) & np.roll(core, 1, 1) & np.roll(core, -1, 1)
    return core


def spread(square, core, steps=12):
    """A cor do miolo se estende para fora dele (a borda da ilha, a folga e a margem)."""
    have, square = core.copy(), np.where(core[..., None], square, 0.0)
    for _ in range(steps):
        total = sum(np.roll(square, s, a) for s in (1, -1) for a in (0, 1))
        count = sum(np.roll(have, s, a).astype(np.float32) for s in (1, -1) for a in (0, 1))
        grow = ~have & (count > 0)
        square[grow] = total[grow] / count[grow, None]
        have |= grow
    return square


# Cada pixel do mapa da cabeca (com um pixel de folga em volta da ilha): o ponto e a normal.
rows_all, cols_all, pos_all, nor_all = [], [], [], []
for t, rows, cols, lam in raster(U[:, :, 0] * W, U[:, :, 1] * H, 0.02):
    keep = (rows < H) & (cols < W)
    rows_all.append(rows[keep]), cols_all.append(cols[keep])
    pos_all.append(lam[keep] @ P[t]), nor_all.append(lam[keep] @ N[t])
rows_all, cols_all = np.concatenate(rows_all), np.concatenate(cols_all)
pos_all, nor_all = np.concatenate(pos_all), np.concatenate(nor_all)
nor_all /= np.maximum(1e-9, np.linalg.norm(nor_all, axis=1))[:, None]

if MAP:
    # O mapa pronto: o quadrado da metade direita (a imagem de cima para baixo).
    color = np.asarray(Image.open(MAP).convert("RGB").resize((H, H), Image.LANCZOS), np.float32)[::-1][rows_all, cols_all - H] / 255.0
    print(f"  cabeca: {len(color)} pixels do mapa {os.path.basename(MAP)}")
else:
    guides = json.load(open(os.path.join(FOLDER, "guias.json")))
    center = np.array(guides["centro"], np.float64)
    scale = float(guides["escala"])

    def skin_of(photo):
        """A pele na imagem: quente (a pele) ou longe do fundo cinza (os cantos)."""
        corners = np.concatenate([photo[:40, :40], photo[:40, -40:], photo[-40:, :40], photo[-40:, -40:]]).reshape(-1, 3)
        return ((photo[..., 0] - photo[..., 2]) > 8.0 / 255.0) | (np.abs(photo - corners.mean(0)).sum(-1) > 45.0 / 255.0)

    def project(points, view):
        rel = points - center
        return (rel @ view["right"] / scale + 0.5) * SIZE, (0.5 - rel @ view["up"] / scale) * SIZE, rel @ view["look"]

    # As vistas: a foto (linhas de cima para baixo), os eixos dela no mundo do
    # Blender (direita e cima), para onde a camera olha e o z-buffer da cabeca.
    views = []
    for name in ("frente", "direita", "costas"):
        photo = np.asarray(Image.open(os.path.join(FOLDER, f"foto_{name}.png")).convert("RGB").resize((SIZE, SIZE), Image.LANCZOS),
                           np.float32) / 255.0
        rot = np.array(guides["vistas"][name]["rot"], np.float64)  # a matriz do mundo da camera
        view = {"name": name, "photo": photo, "skin": skin_of(photo), "right": rot[:, 0], "up": rot[:, 1], "look": -rot[:, 2]}
        zbuf = np.full((SIZE, SIZE), np.inf, np.float32)
        px, py, pd = (a.reshape(-1, 3) for a in project(P.reshape(-1, 3), view))
        for t, rows, cols, lam in raster(px, py, 1e-4):
            keep = (rows < SIZE) & (cols < SIZE)
            np.minimum.at(zbuf, (rows[keep], cols[keep]), (lam @ pd[t])[keep])
        view["zbuf"] = zbuf
        views.append(view)

    # O lado esquerdo (x do Blender positivo) pega a cor do espelho dele no direito.
    # (A 2 mm do meio pega a cor de 2 mm: o meio desenhado na imagem nao vira linha.)
    pos, nor = pos_all.copy(), nor_all.copy()
    left = pos[:, 0] > 0.0
    pos[:, 0] = -np.maximum(np.abs(pos[:, 0]), 0.002)
    nor[left, 0] *= -1.0
    colors, weights = [], []
    for view in views:
        x, y, d = project(pos, view)
        xi, yi = np.clip(x.astype(int), 0, SIZE - 1), np.clip(y.astype(int), 0, SIZE - 1)
        facing = np.clip(-(nor @ view["look"]), 0.0, 1.0)
        ok = (d <= view["zbuf"][yi, xi] + 0.003) & view["skin"][yi, xi] & (facing > 0.05)
        colors.append(view["photo"][yi, xi])
        weights.append(np.where(ok, facing ** SHARP, 0.0))
    # A cor das vistas igualada ao rosto (media e desvio por canal onde os dois pesam bem).
    for k in (1, 2):
        both = (weights[0] > 0.02) & (weights[k] > 0.02)
        if both.sum() > 200:
            a, b = colors[0][both], colors[k][both]
            gain = np.clip(a.std(0) / np.maximum(1e-4, b.std(0)), 0.7, 1.4)
            colors[k] = (colors[k] - b.mean(0)) * gain + a.mean(0)
            print(f"  {views[k]['name']}: cor igualada ao rosto em {both.sum()} pixels")
    total = sum(weights)
    color = sum(c * w[:, None] for c, w in zip(colors, weights))
    painted = total > 1e-6
    color[painted] /= total[painted, None]
    # O que nenhuma vista ve: a media dos 8 pontos pintados mais perto.
    if (~painted).any():
        tree = KDTree(int(painted.sum()))
        for i, p in enumerate(pos[painted]):
            tree.insert(Vector(p), i)
        tree.balance()
        known = color[painted]
        for i in np.nonzero(~painted)[0]:
            color[i] = np.mean([known[j] for _, j, _ in tree.find_n(Vector(pos[i]), 8)], 0)
    print(f"  cabeca: {painted.sum()} pixels pelas fotos, {(~painted).sum()} pelo vizinho pintado")
# A borda da imagem escurece (o preto de fora do mapa, o corte do pescoco da
# foto): so vale o miolo da ilha EDGE px para dentro, e a cor dali se estende.
square = np.zeros((H, H, 3), np.float32)
square[rows_all, cols_all - H] = color
color = spread(square, island_core(EDGE))[rows_all, cols_all - H]
pixels[rows_all, cols_all, :3] = np.clip(color, 0.0, 1.0)

# O tom do pescoco da cabeca (a faixa logo acima do anel do corpo) para a pele do corpo.
neck = pos_all[:, 2] < P[:, :, 2].min() + 0.02
tone = np.median(color[neck], 0)

# A pele do corpo: o tom do pescoco e o grao da bochecha (o mapa menos ele
# borrado, num quadrado em volta de um pixel da bochecha acima da barba; na testa
# o quadrado pegava a linha do cabelo), repetido em espelho sem emenda.
cheek = np.array([-0.05, P[:, :, 1].min() + 0.04, 0.672])
i = int(np.argmin(np.linalg.norm(pos_all - cheek, axis=1)))
patch = pixels[rows_all[i] - 64:rows_all[i] + 64, cols_all[i] - 64:cols_all[i] + 64, :3].copy()
blur = np.asarray(Image.fromarray((patch * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(6)), np.float32) / 255.0
grain = patch - blur
grain = np.concatenate([grain, grain[:, ::-1]], 1)
grain = np.concatenate([grain, grain[::-1]], 0)
old_skin = np.array(rules["materials"][SKIN_MATERIAL]["color"], np.float32)
count = 0
for f in bm.faces:
    if f[part] != SKIN or f.material_index != SKIN_MATERIAL:
        continue
    uv = np.array([l[uv_layer].uv for l in f.loops]) * np.array([W, H])
    x0, x1 = int(max(0, np.floor(uv[:, 0].min()) - 1)), int(min(W - 1, np.ceil(uv[:, 0].max()) + 1))
    y0, y1 = int(max(0, np.floor(uv[:, 1].min()) - 1)), int(min(H - 1, np.ceil(uv[:, 1].max()) + 1))
    block = pixels[y0:y1 + 1, x0:x1 + 1, :3]
    # So os pixels da cor de pele lisa (o resto do atlas fica: unhas, olhos...).
    near = np.abs(block - old_skin).sum(-1) < 0.12
    if not near.any():
        continue
    gy, gx = np.meshgrid(np.arange(y0, y1 + 1), np.arange(x0, x1 + 1), indexing="ij")
    g = grain[gy % grain.shape[0], gx % grain.shape[1]]
    block[near] = np.clip((block - old_skin + tone)[near] + g[near] * 0.8, 0.0, 1.0)
    count += int(near.sum())
image.pixels[:] = pixels.ravel()
image.pack()
bm.free()
print(f"PELE: cabeca {'pelo mapa' if MAP else 'pelas fotos (simetrica)'}; corpo {count} pixels no tom do pescoco {np.round(tone, 3)}")
bpy.ops.wm.save_as_mainfile(filepath=argv[0] if argv else bpy.data.filepath)
