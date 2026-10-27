"""Etapa 4 (22/10/2026): os pesos assados de volta no .blend.

O assado do Delta Mush (tools/blender/assar_delta_mush.py) le e grava o
.troca.json; sem esta etapa, exportar de novo do .blend jogaria o assado fora.
Os vertices vao na ordem da exportacao (a do .blend); a posicao de cada um e
conferida antes de gravar.

    blender -b jogador.blend --python tools/blender/etapas/04_pesos_assados.py -- <assado.troca.json> [saida.blend]
"""
import json
import sys

import bmesh
import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
baked = json.load(open(argv[0]))
body = bpy.data.objects["Jogador"]
names = [j["name"] for j in baked["joints"]]
bm = bmesh.new()
bm.from_mesh(body.data)
if len(bm.verts) != len(baked["positions"]):
    sys.exit(f"o .blend tem {len(bm.verts)} vertices e o assado {len(baked['positions'])}")
bm.verts.ensure_lookup_table()
for v, (x, y, z) in zip(bm.verts, baked["positions"]):
    # o espaco do modelo (x frente, y esquerda) no do Blender: X = y, Y = -x
    if abs(v.co.x - y) > 1e-5 or abs(v.co.y + x) > 1e-5 or abs(v.co.z - z) > 1e-5:
        sys.exit(f"o vertice {v.index} nao bate com o do assado")
deform = bm.verts.layers.deform.verify()
groups = {g.name: g.index for g in body.vertex_groups}
for v, pairs in zip(bm.verts, baked["weights"]):
    for g in list(v[deform].keys()):
        del v[deform][g]
    for j, w in pairs:
        if names[j] not in groups:
            groups[names[j]] = body.vertex_groups.new(name=names[j]).index
        v[deform][groups[names[j]]] = w
bm.to_mesh(body.data)
body.data.update()
# As juntas novas do assado (a da axila, 26/10) tambem nas regras do .blend: sem elas a exportacao recusa os grupos.
rules_text = bpy.data.texts["matter_troca"]
rules = json.loads(rules_text.as_string())
if [j["name"] for j in rules["joints"]] != names:
    rules["joints"] = baked["joints"]
    rules_text.clear()
    rules_text.write(json.dumps(rules))
print(f"PESOS ASSADOS: {len(bm.verts)} vertices gravados no .blend")
bpy.ops.wm.save_as_mainfile(filepath=argv[1] if len(argv) > 1 else bpy.data.filepath)
