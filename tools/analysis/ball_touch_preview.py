#!/usr/bin/env python3
"""Desenhos do visualizador de toque na bola (para analise e correcoes).

Roda o MatterBallPreview (a mesma simulacao da tela "Toque na bola" do
Workbench), le o dump e desenha:

  <prefixo>_cima.png    vista de cima, no referencial do corpo no pedido
                        (rumo para cima): caminho da pelve, dos dois pes
                        (pontos cheios = plantado), da bola, o alvo que o
                        manejo deu ao pe (vermelho), o pe de apoio do chute
                        (verde), a direcao pedida (seta) e o toque (estrela).
  <prefixo>_quadros.png folha de quadros 3D (lado e tras), do pedido ate
                        depois do toque, com a bola e o alvo do pe.

uso:
  python3 tools/analysis/ball_touch_preview.py /tmp/x [--build build-profile]
      [--quadros 8] [--de -0.1] [--ate 0.5] [--tamanho 300]
      -- --marcha trote --fase 0.25 --acao drible --direcao 45 --bola 0.55,0.12,0.11

Tudo depois de "--" vai para o MatterBallPreview (veja tools/BallTouchPreview.cpp).
Com --dump existente (--ler arquivo) nao roda a simulacao.
"""
import argparse
import math
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_pose import projector, render_frame  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SURFACES = ['peito do pe', 'parte de dentro', 'parte de fora', 'sola']
PHASES = ['livre', 'esperando', 'armando', 'indo', 'acompanhando']


def parse(path):
    info = {}
    name = ''
    frames = []
    frame = None
    for line in open(path, encoding='utf-8'):
        p = line.split()
        if not p:
            continue
        k = p[0]
        if k == 'S':
            name = ' '.join(p[1:])
        elif k == 'I':
            it = iter(p[1:])
            for key in it:
                if key in ('pelve', 'bola'):
                    info[key] = np.array([float(next(it)) for _ in range(3)])
                elif key == 'direcao':
                    info[key] = np.array([float(next(it)), float(next(it)), 0.0])
                else:
                    info[key] = float(next(it))
        elif k == 'F':
            frame = {'t': float(p[1]), 'state': int(p[2]), 'phase': int(p[3]), 'links': [], 'contacts': [],
                     'feet': [None, None], 'planted': [False, False], 'ball': None, 'goal': None,
                     'plant': None, 'touch': False}
            frames.append(frame)
        elif k in ('C', 'B'):
            vals = list(map(float, p[2:]))
            frame['links'].append((k, p[1], np.array(vals[0:3]), vals[3:7], vals[7:10]))
        elif k == 'K':
            frame['ball'] = np.array(list(map(float, p[1:4])))
        elif k == 'E':
            side = int(p[1])
            frame['feet'][side] = np.array(list(map(float, p[2:5])))
            frame['planted'][side] = p[5] == '1'
        elif k == 'Q':
            frame['goal'] = (int(p[1]), np.array(list(map(float, p[2:5]))), float(p[5]))
        elif k == 'L':
            frame['plant'] = (int(p[1]), np.array(list(map(float, p[2:5]))), float(p[5]))
        elif k == 'T':
            frame['touch'] = True
    return name, info, frames


def top_view(name, info, frames, out, size=900, before=0.35, after=0.6):
    """Vista de cima no referencial do corpo no pedido (rumo para cima), na
    janela do pedido ate um pouco depois do toque."""
    touch_at = frames[int(info['toque'])]['t'] if int(info.get('toque', -1)) >= 0 else 0.5
    window = [i for i, f in enumerate(frames) if -before <= f['t'] <= touch_at + after]
    if window:
        first = window[0]
        info = dict(info)
        info['pedido'] = int(info.get('pedido', 0)) - first
        if int(info.get('toque', -1)) >= 0:
            info['toque'] = int(info['toque']) - first
        frames = frames[first:window[-1] + 1]
    pelvis0 = info.get('pelve', np.zeros(3))
    yaw = info.get('rumo', 0.0)
    c, s = math.cos(-yaw), math.sin(-yaw)

    def local(p):
        d = p[:2] - pelvis0[:2]
        # x do corpo (frente) para cima, y (esquerda) para a esquerda.
        fx, fy = d[0] * c - d[1] * s, d[0] * s + d[1] * c
        return fx, fy

    pts = []
    for f in frames:
        for p in [f['links'][0][2]] + [q for q in f['feet'] if q is not None] + ([f['ball']] if f['ball'] is not None else []):
            pts.append(local(p))
    pts = np.array(pts)
    lo = pts.min(axis=0) - 0.4
    hi = pts.max(axis=0) + 0.4
    span = max(hi[0] - lo[0], hi[1] - lo[1], 2.0)
    mid = (lo + hi) / 2
    scale = (size - 80) / span

    def px(p):
        fx, fy = local(p)
        return (size / 2 - (fy - mid[1]) * scale, size / 2 + 20 - (fx - mid[0]) * scale)

    im = Image.new('RGB', (size, size + 40), (246, 246, 242))
    dr = ImageDraw.Draw(im)
    # Grade de 0,25 m (linha forte a cada metro) no referencial do corpo.
    for i in range(int(-span), int(span) + 1):
        for j in range(4):
            v = i + j * 0.25
            col = (205, 205, 200) if j == 0 else (228, 228, 224)
            x = size / 2 - (v - mid[1]) * scale
            y = size / 2 + 20 - (v - mid[0]) * scale
            dr.line([(x, 20), (x, size + 20)], fill=col)
            dr.line([(0, y), (size, y)], fill=col)

    def path(points, color, width=2):
        if len(points) > 1:
            dr.line([px(p) for p in points], fill=color, width=width)

    request = int(info.get('pedido', 0))
    touch = int(info.get('toque', -1))
    path([f['links'][0][2] for f in frames], (150, 150, 150), 2)
    colors = [(60, 110, 230), (235, 130, 50)]
    for side in range(2):
        path([f['feet'][side] for f in frames if f['feet'][side] is not None], colors[side], 2)
        for i, f in enumerate(frames):
            if f['feet'][side] is None or i % 3:
                continue
            x, y = px(f['feet'][side])
            r = 3 if f['planted'][side] else 1.5
            dr.ellipse([x - r, y - r, x + r, y + r], fill=colors[side])
    path([f['ball'] for f in frames if f['ball'] is not None], (20, 20, 20), 2)
    goals = [f['goal'][1] for f in frames if f['goal'] is not None]
    for g in goals[::2]:
        x, y = px(g)
        dr.line([(x - 3, y - 3), (x + 3, y + 3)], fill=(210, 30, 30))
        dr.line([(x - 3, y + 3), (x + 3, y - 3)], fill=(210, 30, 30))
    plants = [f['plant'][1] for f in frames if f['plant'] is not None]
    if plants:
        x, y = px(plants[-1])
        dr.rectangle([x - 6, y - 6, x + 6, y + 6], outline=(30, 160, 60), width=2)
    # Pedido: a pelve, a bola e a seta da direcao.
    if 0 <= request < len(frames):
        f = frames[request]
        x, y = px(f['links'][0][2])
        dr.ellipse([x - 6, y - 6, x + 6, y + 6], outline=(90, 90, 90), width=2)
        if f['ball'] is not None:
            bx, by = px(f['ball'])
            rr = 0.11 * scale
            dr.ellipse([bx - rr, by - rr, bx + rr, by + rr], outline=(20, 20, 20), width=2)
            d = info.get('direcao', np.array([1.0, 0.0, 0.0]))
            tip = px(f['ball'] + d * 0.6)
            dr.line([(bx, by), tip], fill=(160, 40, 200), width=3)
            dr.ellipse([tip[0] - 4, tip[1] - 4, tip[0] + 4, tip[1] + 4], fill=(160, 40, 200))
        for side in range(2):
            if f['feet'][side] is not None:
                fx, fy = px(f['feet'][side])
                dr.text((fx + 5, fy - 5), 'E' if side == 0 else 'D', fill=colors[side])
    if 0 <= touch < len(frames):
        f = frames[touch]
        for p in [f['ball']] + [f['feet'][int(info.get('pe', 0))]]:
            if p is None:
                continue
            x, y = px(p)
            dr.regular_polygon((x, y, 9), 5, fill=None, outline=(230, 20, 20))
    side = int(info.get('pe', -1))
    surface = int(info.get('superficie', 0))
    if touch >= 0:
        verdict = (f"toque {frames[touch]['t']:.3f} s depois, pe {'E' if side == 0 else 'D'}, "
                   f"{SURFACES[surface] if 0 <= surface < 4 else '?'}")
    else:
        verdict = f"sem toque (falha {int(info.get('falha', 0))})"
    dr.text((8, 4), name, fill=(0, 0, 0))
    dr.text((8, size + 22), verdict + "   |   azul E, laranja D (cheio = plantado), preto bola, x vermelho alvo do pe, "
            "seta roxa direcao", fill=(0, 0, 0))
    im.save(out)


def sheet(name, info, frames, out, count, t0, t1, size, views):
    request = int(info.get('pedido', 0))
    touch = int(info.get('toque', -1))
    end_t = frames[touch]['t'] + t1 if touch >= 0 else t1 + 0.4
    chosen = [f for f in frames if t0 <= f['t'] <= end_t]
    if not chosen:
        chosen = frames
    n = min(count, len(chosen))
    pick = [chosen[round(i * (len(chosen) - 1) / max(1, n - 1))] for i in range(n)]
    if touch >= 0 and frames[touch] not in pick:
        pick = sorted(pick[:-1] + [frames[touch]], key=lambda f: f['t'])
    rows = []
    for (az, el) in views:
        row = []
        for f in pick:
            g = dict(f)
            g['links'] = list(f['links'])
            if f['ball'] is not None:
                g['links'].append(('C', 'Ball', f['ball'], [0, 0, 0, 1], [0.0, 0.11, 0.11]))
            pel = f['links'][0][2]
            center = np.array([pel[0], pel[1], 0.55])
            im = render_frame(g, size, az, el, 2.4, center)
            project = projector(size, az, el, 2.4, center)
            dr = ImageDraw.Draw(im)
            if f['goal'] is not None:
                x, y = project(f['goal'][1])
                dr.line([(x - 6, y - 6), (x + 6, y + 6)], fill=(220, 20, 20), width=2)
                dr.line([(x - 6, y + 6), (x + 6, y - 6)], fill=(220, 20, 20), width=2)
            if f['plant'] is not None:
                x, y = project(f['plant'][1])
                dr.rectangle([x - 5, y - 5, x + 5, y + 5], outline=(30, 160, 60), width=2)
            label = f"{f['t']:+.3f}s {PHASES[f['phase']] if 0 <= f['phase'] < 5 else f['phase']}"
            dr.rectangle([0, 0, size, 14], fill=(255, 255, 255))
            dr.text((4, 2), label + ('  TOQUE' if f['touch'] else ''), fill=(200, 0, 0) if f['touch'] else (0, 0, 0))
            row.append(im)
        rows.append(row)
    cols = len(pick)
    out_im = Image.new('RGB', (cols * size, len(rows) * size + 18), (255, 255, 255))
    for r, row in enumerate(rows):
        for i, im in enumerate(row):
            out_im.paste(im, (i * size, 18 + r * size))
    ImageDraw.Draw(out_im).text((4, 3), name, fill=(0, 0, 0))
    out_im.save(out)


def main():
    argv = sys.argv[1:]
    passthrough = []
    if '--' in argv:
        k = argv.index('--')
        argv, passthrough = argv[:k], argv[k + 1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('prefixo')
    ap.add_argument('--build', default='build-profile')
    ap.add_argument('--ler')
    ap.add_argument('--quadros', type=int, default=8)
    ap.add_argument('--de', type=float, default=-0.1)
    ap.add_argument('--ate', type=float, default=0.35)
    ap.add_argument('--tamanho', type=int, default=280)
    ap.add_argument('--vistas', default='lado,tras', help='lado, tras, frente, cima3d')
    a = ap.parse_args(argv)
    dump = a.ler
    if dump is None:
        dump = a.prefixo + '_dump.txt'
        exe = os.path.join(ROOT, a.build, 'MatterBallPreview')
        r = subprocess.run([exe, '--dump', dump] + passthrough, capture_output=True, text=True)
        sys.stdout.write(''.join(l + '\n' for l in r.stdout.splitlines() if not l.startswith('[INFO]')))
        if r.returncode != 0:
            sys.stderr.write(r.stderr)
            sys.exit(r.returncode)
    name, info, frames = parse(dump)
    top_view(name, info, frames, a.prefixo + '_cima.png')
    table = {'lado': (90.0, 10.0), 'tras': (0.0, 25.0), 'frente': (180.0, 15.0), 'cima3d': (0.0, 70.0)}
    views = [table[v] for v in a.vistas.split(',') if v in table]
    sheet(name, info, frames, a.prefixo + '_quadros.png', a.quadros, a.de, a.ate, a.tamanho, views)
    print('desenhos:', a.prefixo + '_cima.png', a.prefixo + '_quadros.png')


if __name__ == '__main__':
    main()
