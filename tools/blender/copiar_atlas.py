"""Copia o atlas de outro .blend para este (22/10/2026): a textura refeita sem refazer os pesos.

As etapas da textura (05 -> pintar_rosto -> 06 -> 03) so mexem na imagem e as
dos pesos (02 -> assado -> desvirar -> 04) so nos pesos; com a mesma cabeca (o
mesmo mapa), o atlas novo vai para o .blend final assim:

    blender -b jogador.blend --python tools/blender/copiar_atlas.py -- <origem.blend>

O mapa das duas malhas tem de ser o mesmo (confere os UV das faces da cabeca).
"""
import json
import sys

import bpy
import numpy as np

source = sys.argv[sys.argv.index("--") + 1]
rules = json.loads(bpy.data.texts["matter_troca"].as_string())
name = rules["textures"][0]
target = bpy.data.images.get(name) or next(i for i in bpy.data.images if i.filepath.endswith(name))
mine = bpy.data.objects["Jogador"].data
with bpy.data.libraries.load(source) as (data_from, data_to):
    data_to.images = [n for n in data_from.images]
    data_to.meshes = [n for n in data_from.meshes]
image = next(i for i in data_to.images if i is not None and (i.name == name or i.filepath.endswith(name)))
other = max((m for m in data_to.meshes if m is not None), key=lambda m: len(m.polygons))
if len(other.polygons) != len(mine.polygons):
    sys.exit(f"malhas diferentes: {len(other.polygons)} faces na origem, {len(mine.polygons)} aqui")
uv_a = np.array([d.uv[:] for d in mine.uv_layers["atlas"].data])
uv_b = np.array([d.uv[:] for d in other.uv_layers["atlas"].data])
if uv_a.shape != uv_b.shape or np.abs(uv_a - uv_b).max() > 1e-6:
    sys.exit("o mapa da origem e diferente: o atlas nao serve nesta malha")
if tuple(image.size) != tuple(target.size):
    target.scale(*image.size)
target.pixels[:] = image.pixels[:]
target.pack()
size = tuple(image.size)
# (O que veio da origem sai: so os pixels ficam.)
for block in data_to.images:
    if block is not None:
        bpy.data.images.remove(block)
for block in data_to.meshes:
    if block is not None:
        bpy.data.meshes.remove(block)
print(f"ATLAS copiado de {source} ({size[0]}x{size[1]})")
bpy.ops.wm.save_mainfile()
