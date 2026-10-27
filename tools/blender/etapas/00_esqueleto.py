"""Etapa 0 (22/10/2026): o esqueleto do motor no .blend.

As juntas (e as regras dos auxiliares) do texto "matter_troca" passam a ser as
do esqueleto atual do motor, na ordem dele (o exportador indexa os pesos por
essa lista e o assado le as poses nessa ordem). Os grupos de vertice que ja
existem continuam pelo nome; as juntas novas (as claviculas) entram vazias.

    MatterModelInspector exportar-blender --modelo <model.json> --saida <pasta>
    blender -b jogador.blend --python tools/blender/etapas/00_esqueleto.py -- <pasta>/jogador.troca.json [saida.blend]
"""
import json
import sys

import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
engine = json.load(open(argv[0]))
text = bpy.data.texts["matter_troca"]
rules = json.loads(text.as_string())
before = {j["name"] for j in rules["joints"]}
rules["joints"] = engine["joints"]
after = {j["name"] for j in rules["joints"]}
lost = sorted(g.name for g in bpy.data.objects["Jogador"].vertex_groups if g.name not in after)
if lost:
    sys.exit(f"grupos sem junta no esqueleto novo: {lost}")
text.clear()
text.write(json.dumps(rules))
print(f"ESQUELETO: {len(rules['joints'])} juntas; novas: {sorted(after - before)}")
bpy.ops.wm.save_as_mainfile(filepath=argv[1] if len(argv) > 1 else bpy.data.filepath)
