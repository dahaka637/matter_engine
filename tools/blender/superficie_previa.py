#!/usr/bin/env python3
"""A previa do template de superficie (26/10/2026; docs/SISTEMA_DE_APARENCIA.md): o jogador com um
xadrez de 5 cm na coordenada de desenho de cada painel (ou numa carta), uma cor por painel, o eixo x
(vermelho) e o y (verde) da origem de cada painel, e o layout do UV0. Um rasterizador simples (z-buffer
e baricentricas por pixel), sem o motor: para conferir a planificacao, as cartas e as costuras de olho
enquanto o compositor (etapa 2) nao existe.

    python3 tools/blender/superficie_previa.py <troca com o template> <saida.png> [carta]
"""
import json
import sys

import numpy as np
from PIL import Image, ImageDraw

troca, out = sys.argv[1], sys.argv[2]
chart = sys.argv[3] if len(sys.argv) > 3 else None
d = json.load(open(troca))
P = np.array(d["positions"]); T = np.array(d["triangles"])
S = d["surface"]
panel = np.array(d["trianglePanel"]); region = np.array(d["triangleRegion"])
design = d["triangleDesign"]; kituv = d["triangleKitUv"]
cc = d["triangleChart"][chart] if chart else None
rng = np.random.default_rng(3)
pcolor = {i: rng.uniform(0.35, 0.95, 3) for i in range(len(S["panels"]))}
skin = np.array([0.80, 0.62, 0.50]); boot = np.array([0.15, 0.15, 0.16])
regions = S["regions"]


def coords_of(t):
    if chart:
        c = cc.get(str(t))
        return np.array(c) if c is not None else None
    return np.array(design[t]) if design[t] is not None else None


def render(yaw, pitch, size, target, radius):
    yr, pr = np.radians(yaw), np.radians(pitch)
    fwd = np.array([-np.cos(pr) * np.cos(yr), -np.cos(pr) * np.sin(yr), -np.sin(pr)])
    right = np.cross(fwd, [0, 0, 1.0]); right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    p = P - np.array(target)
    sx = (p @ right) / radius * size / 2 + size / 2
    sy = -(p @ up) / radius * size / 2 + size / 2
    depth = p @ fwd
    img = np.zeros((size, size, 3)) + np.array([0.07, 0.08, 0.10])
    zb = np.full((size, size), np.inf)
    n = np.cross(P[T[:, 1]] - P[T[:, 0]], P[T[:, 2]] - P[T[:, 0]])
    n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
    light = -fwd * 0.75 + up * 0.55 - right * 0.25; light /= np.linalg.norm(light)
    for t in range(len(T)):
        if np.dot(n[t], -fwd) <= 0:
            continue
        a, b, c = T[t]
        xs, ys, zs = sx[[a, b, c]], sy[[a, b, c]], depth[[a, b, c]]
        x0, x1 = int(max(0, np.floor(xs.min()))), int(min(size - 1, np.ceil(xs.max())))
        y0, y1 = int(max(0, np.floor(ys.min()))), int(min(size - 1, np.ceil(ys.max())))
        if x1 < x0 or y1 < y0:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        det = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
        if abs(det) < 1e-12:
            continue
        w0 = ((ys[1] - ys[2]) * (gx - xs[2]) + (xs[2] - xs[1]) * (gy - ys[2])) / det
        w1 = ((ys[2] - ys[0]) * (gx - xs[2]) + (xs[0] - xs[2]) * (gy - ys[2])) / det
        w2 = 1 - w0 - w1
        inside = (w0 >= -1e-4) & (w1 >= -1e-4) & (w2 >= -1e-4)
        if not inside.any():
            continue
        z = w0 * zs[0] + w1 * zs[1] + w2 * zs[2]
        sub = zb[y0:y1 + 1, x0:x1 + 1]
        ok = inside & (z < sub)
        if not ok.any():
            continue
        shade = 0.45 + 0.55 * max(0.0, float(np.dot(n[t], light)))
        uv = coords_of(t)
        if uv is not None:
            u = w0 * uv[0][0] + w1 * uv[1][0] + w2 * uv[2][0]
            v = w0 * uv[0][1] + w1 * uv[1][1] + w2 * uv[2][1]
            check = ((np.floor(u / 0.05) + np.floor(v / 0.05)) % 2)[..., None]
            base = pcolor[panel[t]] if panel[t] >= 0 else skin
            color = base * (0.55 + 0.45 * check)
            # O eixo x (vermelho) e o y (verde) da origem do painel.
            color = np.where((np.abs(u) < 0.004)[..., None], np.array([0.9, 0.1, 0.1]), color)
            color = np.where((np.abs(v) < 0.004)[..., None], np.array([0.1, 0.8, 0.1]), color)
        else:
            base = boot if regions[region[t]] == "chuteira" else skin
            color = np.broadcast_to(base, inside.shape + (3,))
        region_img = img[y0:y1 + 1, x0:x1 + 1]
        region_img[ok] = (color * shade)[ok] if color.ndim == 3 else color[ok] * shade
        sub[ok] = z[ok]
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)


views = [(0, -5), (180, -5), (90, -5), (35, -15), (-60, 25)]
size = 560
tiles = [render(yaw, pitch, size, [0.0, 0.0, 0.05], 0.75) for yaw, pitch in views]
# O layout do UV0.
uvimg = Image.new("RGB", (size, size), (20, 22, 28))
dr = ImageDraw.Draw(uvimg)
for t in range(len(T)):
    if kituv[t] is None:
        continue
    pts = [(c[0] * size, c[1] * size) for c in kituv[t]]
    col = tuple(int(255 * x) for x in pcolor[panel[t]])
    dr.polygon(pts, fill=col, outline=(0, 0, 0))
sheet = Image.new("RGB", (size * 3, size * 2), (10, 10, 12))
for i, tile in enumerate(tiles + [np.array(uvimg)]):
    sheet.paste(Image.fromarray(tile), ((i % 3) * size, (i // 3) * size))
sheet.save(out)
print("ok", out)
