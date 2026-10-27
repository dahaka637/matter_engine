"""Etapa 3 (22/10/2026): a margem das ilhas na textura.

O atlas vem empacotado colado (ilhas a 1 pixel, o fundo preto puro): o canto
de mapa que cai na borda de uma ilha (as pecas refeitas tem faces que nao
seguem as costuras do mapa) pegava o preto do fundo, em tracos e pontos. Como
em todo bake de textura, cada ilha se estende MARGEM pixels sobre o fundo (a
media dos vizinhos ja pintados); nenhum pixel de dentro das ilhas muda.

    blender -b jogador.blend --python tools/blender/etapas/03_textura_margem.py -- [saida.blend]
"""
import json
import sys

import bpy
import numpy as np

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
MARGIN = 8
rules = json.loads(bpy.data.texts["matter_troca"].as_string())
for name in rules["textures"]:
    image = bpy.data.images.get(name) or next(i for i in bpy.data.images if i.filepath.endswith(name))
    w, h = image.size
    px = np.array(image.pixels[:], dtype=np.float32).reshape(h, w, 4)
    background = px[..., :3].sum(-1) == 0.0
    before = int(background.sum())
    for _ in range(MARGIN):
        total = np.zeros((h, w, 3), np.float32)
        count = np.zeros((h, w), np.float32)
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            painted = ~np.roll(background, (dy, dx), axis=(0, 1))
            reach = background & painted
            total[reach] += np.roll(px[..., :3], (dy, dx), axis=(0, 1))[reach]
            count[reach] += 1.0
        fill = count > 0.0
        px[fill, :3] = total[fill] / count[fill, None]
        background &= ~fill
    image.pixels[:] = px.ravel()
    image.pack()
    print(f"MARGEM {name}: {before - int(background.sum())} pixels do fundo pintados ({MARGIN} px)")
bpy.ops.wm.save_as_mainfile(filepath=argv[0] if argv else bpy.data.filepath)
