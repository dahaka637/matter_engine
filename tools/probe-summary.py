#!/usr/bin/env python3
# Uso: tools/probe-summary.py <saida do probe-grid.sh>
# Resume a grade: piores picos, saltos, folga pe-bola na forca, atraso, perna
# louca, voo e toques por pedido.
import sys
rows = []
for line in open(sys.argv[1]):
    if '| v5' not in line:
        continue
    head, v5 = line.split('| v5')
    parts = head.split()
    name = parts[0]
    def field(key):
        try:
            return float(parts[parts.index(key) + 1])
        except (ValueError, IndexError):
            return 0.0
    extra = v5.split('| garantia')
    v = extra[0].split()
    if v and v[0] != '-':
        req, tou, gmed, gmax, lmed, lmax, crazy, flip, spin, flight = (float(x) for x in v[:10])
    else:
        req = tou = gmed = gmax = lmed = lmax = crazy = flip = spin = flight = 0.0
    gua = gfar = lost = aborted = missed = merged = 0.0
    if len(extra) > 1:
        values = [float(x) for x in extra[1].split()]
        gua, gfar, lost = values[:3]
        if len(values) >= 6:
            aborted, missed, merged = values[3:6]
    rows.append(dict(name=name, legs=field('pernas'), trunk=field('tronco'), jump=field('salto'), req=req, tou=tou,
                     gmed=gmed, gmax=gmax, lmed=lmed, lmax=lmax, crazy=crazy, flip=flip, spin=spin, flight=flight,
                     gua=gua, gfar=gfar, lost=lost, aborted=aborted, missed=missed, merged=merged))
def worst(key, n=5):
    return ', '.join(f"{r['name']} {r[key]:g}" for r in sorted(rows, key=lambda r: -r[key])[:n])
print(f"cenarios {len(rows)} | pedidos {sum(r['req'] for r in rows):.0f} toques {sum(r['tou'] for r in rows):.0f}")
print('pernas (m/s2):', worst('legs'))
print('tronco (m/s2):', worst('trunk'))
print('salto (m):', worst('jump'))
print('folga pe-bola max (m):', worst('gmax'))
print('folga pe-bola mediana (m):', worst('gmed'))
print('atraso max (s):', worst('lmax'))
print('perna louca (ticks):', worst('crazy'), '| joelho virando:', worst('flip', 3))
print('giro max (rad/s):', worst('spin'))
print('voo max (s):', worst('flight'))
print('garantia (toques):', worst('gua'), '| folga med:', worst('gfar', 3))
print('perdidos:', worst('lost'), '| juntados:', worst('merged', 3))
print('desistiu:', worst('aborted', 3), '| errou:', worst('missed', 3))
print('cliques sem toque (fora os juntados):', ', '.join(f"{r['name']} {r['req']-r['tou']-r['merged']:.0f}/{r['req']:.0f}" for r in sorted(rows, key=lambda r: -(r['req']-r['tou']-r['merged']))[:8]))
lost = [r for r in rows if r['req'] > 0 and r['tou'] < r['req']]
print('pedidos sem toque:', ', '.join(f"{r['name']} {r['tou']:.0f}/{r['req']:.0f}" for r in lost[:12]))
