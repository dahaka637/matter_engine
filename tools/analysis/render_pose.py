#!/usr/bin/env python3
"""Folha de quadros do ragdoll a partir do XPOSEDUMP do harness (MatterAdaptiveTests)
ou do dump do MatterBallPreview (--dump).

Rasterizador proprio (numpy + PIL, sem GPU): capsulas e caixas dos colliders,
chao quadriculado de 0,5 m, pontos de contato em verde.

uso: render_pose.py dump.txt saida.png [--scenario N] [--times t0,t1,...|--from A --to B --count K]
     [--azimuth graus] [--elevation graus] [--size px] [--cols C] [--span m] [--follow 0|1] [--ghost 0|1]

Gerar o dump nos testes: XPOSEDUMP=arquivo XPOSEEVERY=6 MATTERENGINE_TEST_FILTER=... ./MatterAdaptiveTests
azimuth 0 / elevation 30 = de tras e de cima (o corpo olha para +x); 90 = de lado.
"""
import argparse, math, sys
import numpy as np
from PIL import Image, ImageDraw


def quat_rotate(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def quat_matrix(q):
    return np.stack([quat_rotate(q, e) for e in np.eye(3)], axis=1)


def parse(path):
    scenarios = []
    cur = None
    frame = None
    ghost_next = False
    for line in open(path):
        p = line.split()
        if not p:
            continue
        if p[0] == 'S':
            cur = {'name': ' '.join(p[1:]), 'frames': []}
            scenarios.append(cur)
        elif p[0] == 'G':
            ghost_next = True
        elif p[0] == 'F':
            frame = {'t': float(p[1]), 'state': int(p[2]), 'phase': int(p[3]), 'links': [], 'contacts': []}
            if cur is None:
                cur = {'name': '?', 'frames': []}
                scenarios.append(cur)
            if ghost_next:
                cur['frames'][-1]['ghost'] = frame
                ghost_next = False
            else:
                cur['frames'].append(frame)
        elif p[0] in ('C', 'B'):
            vals = list(map(float, p[2:]))
            frame['links'].append((p[0], p[1], np.array(vals[0:3]), vals[3:7], vals[7:10]))
        elif p[0] == 'P':
            frame['contacts'].append((np.array(list(map(float, p[1:4]))), int(p[4])))
    return scenarios


def capsule_mesh(h, r0, r1, rings=8, seg=12):
    # eixo x, de -h (raio r0) a +h (raio r1), com calotas
    verts = []
    profile = []
    for i in range(rings + 1):  # calota -x
        a = math.pi / 2 * (1 - i / rings)
        profile.append((-h - r0 * math.sin(a), r0 * math.cos(a)))
    for i in range(rings + 1):  # calota +x
        a = math.pi / 2 * i / rings
        profile.append((h + r1 * math.sin(a), r1 * math.cos(a)))
    for (px, pr) in profile:
        for k in range(seg):
            b = 2 * math.pi * k / seg
            verts.append((px, pr * math.cos(b), pr * math.sin(b)))
    tris = []
    n = len(profile)
    for i in range(n - 1):
        for k in range(seg):
            a = i * seg + k
            b = i * seg + (k + 1) % seg
            c = (i + 1) * seg + (k + 1) % seg
            d = (i + 1) * seg + k
            tris += [(a, b, c), (a, c, d)]
    return np.array(verts), np.array(tris)


def box_mesh(hx, hy, hz):
    v = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, c, d in faces:
        tris += [(a, b, c), (a, c, d)]
    return v, np.array(tris)


def color_of(name):
    if name.startswith('Left'):
        return np.array([0.30, 0.55, 0.95])
    if name.startswith('Right'):
        return np.array([0.95, 0.55, 0.25])
    if name == 'Ball':
        return np.array([0.98, 0.98, 0.98])
    if name in ('Head', 'Neck'):
        return np.array([0.92, 0.85, 0.75])
    return np.array([0.80, 0.80, 0.82])


def camera_basis(azimuth, elevation):
    az = math.radians(azimuth)
    el = math.radians(elevation)
    forward = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), -math.sin(el)])
    right = np.cross(forward, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return forward, right, up


def projector(size, azimuth, elevation, span, center):
    """Ponto do mundo -> pixel (x, y) na imagem de render_frame."""
    _, right, up = camera_basis(azimuth, elevation)
    scale = size / span

    def project(p):
        rel = np.asarray(p, float) - center
        return (float(rel @ right * scale + size / 2), float(size / 2 - rel @ up * scale))
    return project


def render_frame(frame, size, azimuth, elevation, span, center):
    az = math.radians(azimuth)
    el = math.radians(elevation)
    # camera olhando para 'forward'
    forward = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), -math.sin(el)])
    right = np.cross(forward, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    light = np.array([-0.3, -0.5, 0.8])
    light /= np.linalg.norm(light)
    tris_world = []
    colors = []
    for (kind, name, c, q, d) in frame['links']:
        if kind == 'C':
            v, t = capsule_mesh(d[0], d[1], d[2] if d[2] > 0 else d[1])
        else:
            v, t = box_mesh(*d)
        R = quat_matrix(q)
        w = v @ R.T + c
        col = color_of(name)
        for tri in t:
            tris_world.append(w[list(tri)])
            colors.append(col)
    # chao: quadriculado 4x4 m em volta do centro
    cx, cy = round(center[0]), round(center[1])
    for i in range(-3, 3):
        for j in range(-3, 3):
            x0, y0 = cx + i * 0.5, cy + j * 0.5
            col = np.array([0.42, 0.45, 0.42]) if (i + j) % 2 == 0 else np.array([0.36, 0.39, 0.36])
            a = np.array([x0, y0, 0]); b = np.array([x0 + 0.5, y0, 0])
            c2 = np.array([x0 + 0.5, y0 + 0.5, 0]); d2 = np.array([x0, y0 + 0.5, 0])
            tris_world.append(np.array([a, b, c2])); colors.append(col)
            tris_world.append(np.array([a, c2, d2])); colors.append(col)
    img = np.zeros((size, size, 3))
    img[:] = (0.62, 0.70, 0.78)
    zb = np.full((size, size), np.inf)
    scale = size / span
    for tri, col in zip(tris_world, colors):
        rel = tri - center
        sx = rel @ right * scale + size / 2
        sy = size / 2 - rel @ up * scale
        z = rel @ forward
        nrm = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        ln = np.linalg.norm(nrm)
        if ln < 1e-12:
            continue
        nrm /= ln
        if nrm @ forward > 0:
            nrm = -nrm
        shade = col * (0.35 + 0.65 * max(0.0, nrm @ light))
        x0 = max(int(math.floor(sx.min())), 0); x1 = min(int(math.ceil(sx.max())), size - 1)
        y0 = max(int(math.floor(sy.min())), 0); y1 = min(int(math.ceil(sy.max())), size - 1)
        if x0 > x1 or y0 > y1:
            continue
        den = (sy[1] - sy[2]) * (sx[0] - sx[2]) + (sx[2] - sx[1]) * (sy[0] - sy[2])
        if abs(den) < 1e-9:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        w0 = ((sy[1] - sy[2]) * (gx - sx[2]) + (sx[2] - sx[1]) * (gy - sy[2])) / den
        w1 = ((sy[2] - sy[0]) * (gx - sx[2]) + (sx[0] - sx[2]) * (gy - sy[2])) / den
        w2 = 1 - w0 - w1
        m = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
        if not m.any():
            continue
        zz = w0 * z[0] + w1 * z[1] + w2 * z[2]
        sub = zb[y0:y1 + 1, x0:x1 + 1]
        m &= zz < sub
        sub[m] = zz[m]
        img[y0:y1 + 1, x0:x1 + 1][m] = shade
    im = Image.fromarray((np.clip(img, 0, 1) ** (1 / 2.2) * 255).astype(np.uint8))
    dr = ImageDraw.Draw(im)
    for (p, link) in frame['contacts']:
        rel = p - center
        px = rel @ right * scale + size / 2
        py = size / 2 - rel @ up * scale
        dr.ellipse([px - 3, py - 3, px + 3, py + 3], fill=(40, 220, 60))
    dr.text((4, 4), f"t {frame['t']:.2f}  st {frame['state']} f {frame['phase']}", fill=(0, 0, 0))
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dump'); ap.add_argument('out')
    ap.add_argument('--scenario', type=int, default=0)
    ap.add_argument('--times')
    ap.add_argument('--from', dest='t0', type=float, default=0.0)
    ap.add_argument('--to', dest='t1', type=float, default=1e9)
    ap.add_argument('--count', type=int, default=16)
    ap.add_argument('--azimuth', type=float, default=90.0)
    ap.add_argument('--elevation', type=float, default=12.0)
    ap.add_argument('--size', type=int, default=300)
    ap.add_argument('--cols', type=int, default=4)
    ap.add_argument('--span', type=float, default=2.4)
    ap.add_argument('--follow', type=int, default=1)
    ap.add_argument('--ghost', type=int, default=0)
    a = ap.parse_args()
    sc = parse(a.dump)[a.scenario]
    frames = [f for f in sc['frames'] if a.t0 <= f['t'] <= a.t1]
    if a.times:
        want = [float(x) for x in a.times.split(',')]
        pick = [min(frames, key=lambda f: abs(f['t'] - t)) for t in want]
    else:
        n = min(a.count, len(frames))
        pick = [frames[round(i * (len(frames) - 1) / max(1, n - 1))] for i in range(n)]
    fixed = None
    ims = []
    for f in pick:
        pel = f['links'][0][2]
        center = np.array([pel[0], pel[1], 0.55])
        if not a.follow:
            fixed = fixed if fixed is not None else center
            center = fixed
        ims.append(render_frame(f, a.size, a.azimuth, a.elevation, a.span, center))
        if a.ghost:
            g = f.get('ghost')
            if g is not None:
                gi = render_frame(g, a.size, a.azimuth, a.elevation, a.span, center)
            else:
                gi = Image.new('RGB', (a.size, a.size), (200, 200, 200))
            ImageDraw.Draw(gi).text((4, 16), 'alvo', fill=(120, 0, 0))
            ims.append(gi)
    cols = a.cols * (2 if a.ghost else 1)
    rows = (len(ims) + cols - 1) // cols
    sheet = Image.new('RGB', (cols * a.size, rows * a.size), (255, 255, 255))
    for i, im in enumerate(ims):
        sheet.paste(im, ((i % cols) * a.size, (i // cols) * a.size))
    sheet.save(a.out)
    print(sc['name'], len(sc['frames']), 'quadros; desenhados', len(ims))


if __name__ == '__main__':
    main()
