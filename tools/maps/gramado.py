"""Prepara os dados PBR do gramado a partir do albedo criado para o projeto."""
from pathlib import Path
import math
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]


def generate():
    out = ROOT / 'assets/maps/textures/indoor'
    out.mkdir(parents=True, exist_ok=True)
    image = Image.open(ROOT / 'assets/maps/materials/grass/albedo_source.png').convert('RGB')
    image = image.resize((1024, 1024), Image.Resampling.LANCZOS)
    # A faixa de borda converge para a mesma cor no par oposto, fechando o tile.
    pixels = image.load()
    size, border = image.width, 24
    for axis in (0, 1):
        for t in range(size):
            for i in range(border):
                a, b = ((i, t), (size-1-i, t)) if axis == 0 else ((t, i), (t, size-1-i))
                ca, cb = pixels[a], pixels[b]
                mix = .5 * (1-i/border)**2
                pixels[a] = tuple(round(ca[k]*(1-mix)+cb[k]*mix) for k in range(3))
                pixels[b] = tuple(round(cb[k]*(1-mix)+ca[k]*mix) for k in range(3))
    image.save(out / 'gramado_simples.png')
    # Altura estimada das fibras, sem deslocar a superficie de colisao.
    height = image.convert('L')
    heights = list(height.getdata())
    normal, mr = [], []
    for y in range(size):
        for x in range(size):
            dx = (heights[y*size+(x+1)%size]-heights[y*size+(x-1)%size]) / 255 * 2.5
            dy = (heights[((y+1)%size)*size+x]-heights[((y-1)%size)*size+x]) / 255 * 2.5
            inv = 1/math.sqrt(dx*dx+dy*dy+1)
            normal.append((round((.5-dx*inv*.5)*255), round((.5-dy*inv*.5)*255),
                           round((.5+inv*.5)*255), heights[y*size+x]))
            mr.append((255, 232 + heights[y*size+x]//12, 0))
    n = Image.new('RGBA', image.size)
    n.putdata(normal)
    n.save(out / 'gramado_normal.png')
    r = Image.new('RGB', image.size)
    r.putdata(mr)
    r.save(out / 'gramado_mr.png')
    assert image.size == n.size == r.size == (1024, 1024)
    assert all(p[2] >= 128 for p in normal)
    print('Gramado: albedo repetivel, normal/altura RGBA e roughness/metallic 1024px')


if __name__ == '__main__':
    generate()
