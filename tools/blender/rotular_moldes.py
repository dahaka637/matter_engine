"""Os moldes de uma construcao levados a uma malha que nao foi refeita (26/10/2026).

O template da superficie precisa do molde de cada face da roupa (o atributo
"molde" que a etapa 01 grava desde 26/10). A malha aprovada foi construida antes
disso: aqui a construcao e refeita com as mesmas entradas (a etapa 01 e
deterministica) e o molde de cada face vem da face com os MESMOS vertices na
refeita. Nada de proximidade: os vertices precisam ser os mesmos, na mesma
ordem e na mesma posicao, e toda face da camisa e do calcao precisa achar a sua;
qualquer diferenca para sem gravar.

    blender -b <aprovado.blend> --python tools/blender/rotular_moldes.py -- <refeito.blend> <saida.blend>
"""
import json
import sys

import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
rebuilt_path, target = argv[0], argv[1]

body = bpy.data.objects["Jogador"].data
# (Anexar a malha traz junto os materiais e a imagem dela: o que nao havia antes sai no fim.)
before = {kind: set(getattr(bpy.data, kind)) for kind in ("materials", "images")}
with bpy.data.libraries.load(rebuilt_path) as (source, loaded):
    loaded.meshes = [name for name in source.meshes if name.startswith("Jogador")][:1]
    loaded.texts = [name for name in source.texts if name == "matter_moldes"]
if not loaded.meshes or not loaded.texts:
    sys.exit("a refeita nao tem a malha Jogador com o texto matter_moldes")
rebuilt = loaded.meshes[0]
names = json.loads(loaded.texts[0].as_string())

if len(rebuilt.vertices) != len(body.vertices):
    sys.exit(f"vertices: {len(rebuilt.vertices)} na refeita, {len(body.vertices)} na aprovada")
worst = max((a.co - b.co).length for a, b in zip(rebuilt.vertices, body.vertices))
if worst > 0.0:
    sys.exit(f"a refeita nao e a aprovada: um vertice anda {worst * 1000.0:.4f} mm")

labels = rebuilt.attributes["molde"].data
by_corners = {tuple(sorted(face.vertices)): labels[face.index].value for face in rebuilt.polygons}
rules = json.loads(bpy.data.texts["matter_troca"].as_string())
parts = [part["name"] for part in rules["parts"]]
garment = {parts.index("camisa"), parts.index("calcao")}
part_of = body.attributes["parte"].data
target_layer = body.attributes.get("molde") or body.attributes.new("molde", "INT", "FACE")
values, orphans = [], 0
for face in body.polygons:
    value = by_corners.get(tuple(sorted(face.vertices)))
    if value is None:
        orphans += part_of[face.index].value in garment
        value = 0
    values.append(value)
if orphans:
    sys.exit(f"{orphans} face(s) da roupa sem a face igual na refeita")
unlabeled = sum(1 for face, value in zip(body.polygons, values) if part_of[face.index].value in garment and value == 0)
if unlabeled:
    sys.exit(f"{unlabeled} face(s) da roupa sem molde na refeita")
target_layer.data.foreach_set("value", values)
text = bpy.data.texts.get("matter_moldes") or bpy.data.texts.new("matter_moldes")
text.from_string(json.dumps(names))
# (Um texto sem usuario nao e gravado: o anexado da refeita sumia ao salvar.)
text.use_fake_user = True
bpy.data.meshes.remove(rebuilt)
for kind, existing in before.items():
    collection = getattr(bpy.data, kind)
    for block in [b for b in collection if b not in existing]:
        collection.remove(block)
bpy.ops.wm.save_as_mainfile(filepath=target)
counts = {}
for value in values:
    counts[names[value]] = counts.get(names[value], 0) + 1
print("ROTULADO", target, {name: n for name, n in sorted(counts.items())})
