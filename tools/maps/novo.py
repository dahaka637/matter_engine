#!/usr/bin/env python3
"""Cria um .map Valve 220 sem sobrescrever arquivos existentes."""
import argparse
from pathlib import Path
import re

from compilar import ASSETS, add, mul, cross, unit


def brush(planes, material='matter/concrete'):
    lines = ['{']
    for normal, position in planes:
        reference = [0, 0, 0]
        reference[min(range(3), key=lambda i: abs(normal[i]))] = 1
        u = unit(cross(normal, reference))
        v = unit(cross(normal, u))
        points = [position, add(position, mul(v, 32)), add(position, mul(u, 32))]
        text = ' '.join('( ' + ' '.join(f'{n:.12g}' for n in p) + ' )' for p in points)
        uv = {0: '[ 0 1 0 0 ] [ 0 0 -1 0 ]',
              1: '[ 1 0 0 0 ] [ 0 0 -1 0 ]',
              2: '[ 1 0 0 0 ] [ 0 -1 0 0 ]'}[max(range(3), key=lambda i: abs(normal[i]))]
        lines.append(f'{text} {material} {uv} 0 0.25 0.25')
    return '\n'.join(lines + ['}'])


def box(lo, hi, material='matter/concrete', bevel=0):
    """Coordenadas e chanfro em unidades do editor; zero preserva a caixa simples."""
    if not 0 <= bevel < min(b-a for a,b in zip(lo,hi))/2:
        raise ValueError('Chanfro invalido para as dimensoes do brush')
    planes = []
    for axis in range(3):
        for sign, bound in [(-1, lo[axis]), (1, hi[axis])]:
            n, p = [0, 0, 0], [0, 0, 0]
            n[axis], p[axis] = sign, bound
            planes.append((tuple(n), tuple(p)))
    if bevel:
        for a,b in ((0,1),(0,2),(1,2)):
            for sa in (-1,1):
                for sb in (-1,1):
                    n=[0,0,0]
                    p=[(x+y)/2 for x,y in zip(lo,hi)]
                    n[a],n[b]=sa,sb
                    p[a]=(hi[a] if sa>0 else lo[a])-sa*bevel
                    p[b]=hi[b] if sb>0 else lo[b]
                    planes.append((tuple(n),tuple(p)))
    return brush(planes, material)


def template(name, demo=False):
    objects = [box((-256, -256, -16), (256, 256, 0))]
    if demo:
        objects += [box((-256, 224, 0), (256, 240, 96), 'matter/brick'),
                    box((96, -32, 0), (128, 32, 32), 'matter/wood')]
    return ('// Game: Soccer Fall\n// Format: Valve\n{\n"classname" "worldspawn"\n'
            f'"message" "{name}"\n' + '\n'.join(objects)
            + '\n}\n{\n"classname" "info_player_start"\n"origin" "0 0 8"\n"angle" "0"\n}\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('nome', help='Nome simples, sem extensao')
    args = parser.parse_args()
    if not re.fullmatch('[a-z][a-z0-9_]*', args.nome): parser.error('Use minusculas, numeros e sublinhado')
    path = ASSETS / 'src' / (args.nome + '.map')
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open('x') as file: file.write(template(args.nome))
        print(path)
    except FileExistsError: parser.exit(1, f'O mapa ja existe: {path}\n')
