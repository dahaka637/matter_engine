"""Grava o JSON de troca do motor a partir do .blend do jogador (a volta).

    blender -b assets/characters/player_model/blender/jogador.blend \
        --python tools/blender/exportar_troca.py -- \
        assets/characters/player_model/blender/jogador.troca.json

As regras (o esqueleto, as partes, os materiais, as texturas) vem do texto
"matter_troca" que importar_troca.py deixou no .blend. A malha do objeto
"Jogador" volta ao espaco do modelo (x = -Y, y = X, z = Z), triangulada; o mapa
por canto (v para baixo, como no motor); a parte de cada face pelo atributo
"parte" e o material pelo indice; os pesos pelos grupos com o nome das juntas,
os quatro maiores de cada vertice, somando 1. O molde de cada face (atributo
"molde", tabela no texto "matter_moldes") vai em "triangleMolde" e "moldes": a
base do template da superficie (tools/blender/superficie.py, depois desta).
"""
import json
import os
import sys

import bmesh
import bpy


def main():
    argv = sys.argv[sys.argv.index("--") + 1:]
    target = argv[0]
    rules = json.loads(bpy.data.texts["matter_troca"].as_string())
    body = bpy.data.objects["Jogador"]
    index = {joint["name"]: i for i, joint in enumerate(rules["joints"])}
    group_joint = {group.index: index.get(group.name, -1) for group in body.vertex_groups}
    missing = [group.name for group in body.vertex_groups if group.name not in index]
    if missing:
        sys.exit(f"grupos sem junta no esqueleto do motor: {missing}")

    mesh = bmesh.new()
    mesh.from_mesh(body.data)
    # O poligono de origem de cada triangulo (o arame do motor mostra o quad sem a diagonal).
    polygon = mesh.faces.layers.int.new("poligono_troca")
    for face in mesh.faces:
        face[polygon] = face.index
    bmesh.ops.triangulate(mesh, faces=[f for f in mesh.faces if len(f.verts) > 3])
    mesh.verts.index_update()
    uv = mesh.loops.layers.uv.get("atlas")
    part = mesh.faces.layers.int.get("parte")
    molde = mesh.faces.layers.int.get("molde")
    moldes = json.loads(bpy.data.texts["matter_moldes"].as_string()) if "matter_moldes" in bpy.data.texts else None
    if (molde is None) != (moldes is None):
        sys.exit("o atributo \"molde\" e o texto \"matter_moldes\" andam juntos")
    deform = mesh.verts.layers.deform.active
    positions, weights = [], []
    for vert in mesh.verts:
        p = vert.co
        positions.append([-p.y, p.x, p.z])
        pairs = []
        if deform is not None:
            for group, weight in vert[deform].items():
                joint = group_joint.get(group, -1)
                if joint >= 0 and weight > 0.0:
                    pairs.append([joint, weight])
        pairs = sorted(pairs, key=lambda item: -item[1])[:4]
        total = sum(w for _, w in pairs)
        weights.append([[j, w / total] for j, w in pairs] if total > 0.0 else [])
    triangles, triangle_uv, triangle_part, triangle_material, triangle_polygon, triangle_molde = [], [], [], [], [], []
    for face in mesh.faces:
        triangles.append([vert.index for vert in face.verts])
        triangle_polygon.append(face[polygon])
        if uv is not None:
            triangle_uv.append([[loop[uv].uv[0], 1.0 - loop[uv].uv[1]] for loop in face.loops])
        triangle_part.append(face[part] if part is not None else 0)
        triangle_material.append(face.material_index)
        if molde is not None:
            if not 0 <= face[molde] < len(moldes):
                sys.exit(f"molde {face[molde]} fora da tabela")
            triangle_molde.append(face[molde])
    # (Nada por triangulo nem o template velho vem das regras: a malha grava de novo.)
    document = {key: value for key, value in rules.items() if not key.startswith("triangle") and key not in ("moldes", "surface")}
    document.update({"positions": positions, "weights": weights, "triangles": triangles, "triangleUv": triangle_uv,
                     "trianglePart": triangle_part, "triangleMaterial": triangle_material, "trianglePolygon": triangle_polygon})
    if molde is not None:
        document.update({"moldes": moldes, "triangleMolde": triangle_molde})
    folder = os.path.dirname(os.path.abspath(target))
    for name in rules["textures"]:
        image = bpy.data.images.get(name) or next((i for i in bpy.data.images if os.path.basename(i.filepath) == name), None)
        if image is not None and not os.path.exists(os.path.join(folder, name)):
            image.save(filepath=os.path.join(folder, name))
    with open(target, "w") as handle:
        json.dump(document, handle)
    print(f"GRAVADO {target}: {len(positions)} vertices, {len(triangles)} triangulos")


main()
