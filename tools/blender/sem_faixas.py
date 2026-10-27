"""Tira as faixas pretas do lado da camisa e do calcao (23/10/2026, pedido do usuario: ver melhor o
contraste com o fundo). Os pixels escuros das faixas viram o branco da peca; o punho, a gola e os
numeros ficam. A faixa e o que e escuro na peca, virado para o lado (a normal da face) e fora da
manga, da gola e dos numeros; a margem da ilha (8 px) vai junto, sem passar em outra parte.

    blender -b jogador.blend --python tools/blender/sem_faixas.py -- [saida.blend]
"""
import json
import sys

import bmesh
import bpy
import numpy as np

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
MARGIN = 8
body = bpy.data.objects["Jogador"]
rules = json.loads(bpy.data.texts["matter_troca"].as_string())
names = [p["name"] for p in rules["parts"]]
ground = rules["ground"]
at = {j["name"]: np.array(j["position"]) for j in rules["joints"]}
image = bpy.data.images.get(rules["textures"][0]) or next(i for i in bpy.data.images if i.filepath.endswith(rules["textures"][0]))
W, H = image.size
px = np.array(image.pixels[:], np.float32).reshape(H, W, 4)

bm = bmesh.new()
bm.from_mesh(body.data)
part = bm.faces.layers.int.get("parte")
uvl = bm.loops.layers.uv.get("atlas")
tris = []
for f in bm.faces:
    loops = list(f.loops)
    for k in range(1, len(loops) - 1):
        t = (loops[0], loops[k], loops[k + 1])
        co = np.array([l.vert.co[:] for l in t])
        tris.append((f[part], np.stack([-co[:, 1], co[:, 0], co[:, 2]], 1), np.array([l[uvl].uv[:] for l in t]) * (W, H)))


def seg_dist(p, a, b):
    t = np.clip(((p - a) @ (b - a)) / ((b - a) @ (b - a)), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * (b - a)), axis=1)


def raster(uv, grow):
    x, y = uv[:, 0] - 0.5, uv[:, 1] - 0.5
    x0, x1 = int(max(0, np.floor(x.min() - grow))), int(min(W - 1, np.ceil(x.max() + grow)))
    y0, y1 = int(max(0, np.floor(y.min() - grow))), int(min(H - 1, np.ceil(y.max() + grow)))
    if x1 < x0 or y1 < y0:
        return None
    gx, gy = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
    den = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
    if abs(den) < 1e-12:
        return None
    l1 = ((y[1] - y[2]) * (gx - x[2]) + (x[2] - x[1]) * (gy - y[2])) / den
    l2 = ((y[2] - y[0]) * (gx - x[2]) + (x[0] - x[2]) * (gy - y[2])) / den
    l3 = 1.0 - l1 - l2
    area = abs(den) / 2.0
    hts = [2 * area / max(1e-9, np.hypot(x[(k + 2) % 3] - x[(k + 1) % 3], y[(k + 2) % 3] - y[(k + 1) % 3])) for k in range(3)]
    out = np.maximum(0.0, np.maximum(np.maximum(-l1 * hts[0], -l2 * hts[1]), -l3 * hts[2]))
    m = out <= grow
    return gy[m], gx[m], out[m] <= 0.0, np.stack([l1[m], l2[m], l3[m]], 1)


garments = {names.index("camisa"), names.index("calcao")}
owner = np.full((H, W), -1, np.int16)              # quem cobre o pixel por dentro
for p, P, uv in tris:
    r = raster(uv, 0.0)
    if r is not None:
        owner[r[0][r[2]], r[1][r[2]]] = p
stripe = np.zeros((H, W), bool)
white = {}
for p, P, uv in tris:
    if p not in garments:
        continue
    n = np.cross(P[1] - P[0], P[2] - P[0])
    n /= max(1e-12, np.linalg.norm(n))
    c = P.mean(0)
    h = c[2] - ground
    side = "Right" if c[1] < 0 else "Left"
    keep = None
    if names[p] == "camisa":
        ok = abs(c[1]) > 0.09 and h < 1.43
    else:
        # A faixa em volta da costura de fora: o angulo em volta da perna perto do lado (o numero fica na frente).
        hip = at[side + "Thigh"]
        ang = np.degrees(np.arctan2(c[0] - hip[0], abs(c[1]) - abs(hip[1])))
        ok = abs(ang) < 32.0 and abs(c[1]) > 0.06
    if not ok:
        continue
    r = raster(uv, MARGIN)
    if r is None:
        continue
    rr, cc, inside, lam = r
    free = inside | (owner[rr, cc] == -1) | (owner[rr, cc] == p)
    if names[p] == "camisa":
        # Por pixel: fora o punho (a menos de 10 cm do braco e ate 3 cm acima do plano da barra da
        # manga), a gola e os numeros (o meio).
        q = lam @ P
        a, e = at[side + "UpperArm"], at[side + "Forearm"]
        along = (e - a) / np.linalg.norm(e - a)
        s_cuff = -((q - (a + 0.6 * (e - a))) @ along)
        cuff = (seg_dist(q, a, e) < 0.10) & (s_cuff < 0.030)
        free &= ~cuff & (np.abs(q[:, 1]) > 0.10) & (q[:, 2] - ground < 1.42)
    stripe[rr[free], cc[free]] = True
for p in garments:
    m = owner == p
    light = px[m, :3][px[m, :3].sum(1) > 2.4]
    white[p] = np.median(light, axis=0)
# A cor branca: a de cada peca pelo dono do pixel (a margem: a camisa).
dark = px[..., :3].sum(-1) < 1.6
target = stripe & dark
color = np.where((owner == names.index("calcao"))[..., None], white[names.index("calcao")], white[names.index("camisa")])
px[target, :3] = color[target]
# Bordas suaves (meio-tom entre preto e branco) que sobram em volta.
mid = stripe & (px[..., :3].sum(-1) < 2.5) & ~target
px[mid, :3] = color[mid]
image.pixels[:] = px.ravel()
image.pack()
bm.free()
print(f"SEM FAIXAS: {int(target.sum())} pixels pretos e {int(mid.sum())} de borda viraram branco")
bpy.ops.wm.save_as_mainfile(filepath=argv[0] if argv else bpy.data.filepath)
