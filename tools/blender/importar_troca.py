"""Monta o .blend do jogador a partir do JSON de troca do motor.

    blender -b --factory-startup --python tools/blender/importar_troca.py -- \
        assets/characters/player_model/blender/jogador.troca.json \
        assets/characters/player_model/blender/jogador.blend

(MatterModelInspector exportar-blender grava o JSON.) O espaco do modelo (x
frente, y esquerda, z cima) vira o do Blender (frente -Y, esquerda +X, z
cima): X = y, Y = -x, Z = z. A malha leva o mapa (v para cima no Blender), um
material por material do motor (com a textura), a parte de cada face no
atributo "parte" e os pesos em grupos com o nome de cada junta; a armadura tem
um osso por junta (os auxiliares presos ao osso de cima). O JSON inteiro, sem a
malha, fica no texto "matter_troca" do .blend: a volta (exportar_troca.py) le
as regras de la. Os triangulos de cada poligono de origem ("trianglePolygon")
voltam a ser o poligono (o quad), na ordem original das faces: a volta grava o
mesmo agrupamento (o template da superficie depende dele para achar a grade).
"""
import json
import os
import sys

import bpy
import mathutils


def to_blender(p):
    return mathutils.Vector((p[1], -p[0], p[2]))


def main():
    argv = sys.argv[sys.argv.index("--") + 1:]
    source, target = argv[0], argv[1]
    folder = os.path.dirname(os.path.abspath(source))
    with open(source) as handle:
        data = json.load(handle)
    bpy.ops.wm.read_factory_settings(use_empty=True)

    # A malha: cada poligono de origem de novo (o contorno dos triangulos dele, no sentido deles).
    triangles = data["triangles"]
    groups = {}
    for t, pid in enumerate(data.get("trianglePolygon") or range(len(triangles))):
        groups.setdefault(pid, []).append(t)
    polygons, first, corner_uv = [], [], []
    for pid in sorted(groups):
        tris = groups[pid]
        directed = {(u, v) for t in tris for u, v in zip(triangles[t], triangles[t][1:] + triangles[t][:1])}
        following = {u: v for u, v in directed if (v, u) not in directed}
        loop = [min(following)]
        while following[loop[-1]] != loop[0]:
            loop.append(following[loop[-1]])
            if len(loop) > len(following):
                sys.exit(f"o poligono {pid} nao fecha um contorno")
        polygons.append(loop)
        first.append(tris[0])
        uv = {}
        for t in tris:
            for k, v in enumerate(triangles[t]):
                if data["triangleUv"]:
                    uv[v] = data["triangleUv"][t][k]
        corner_uv.append(uv)
    mesh = bpy.data.meshes.new("Jogador")
    mesh.from_pydata([to_blender(p) for p in data["positions"]], [], [tuple(loop) for loop in polygons])
    mesh.update()
    if data["triangleUv"]:
        layer = mesh.uv_layers.new(name="atlas")
        for face in mesh.polygons:
            for vertex, loop in zip(face.vertices, face.loop_indices):
                u, v = corner_uv[face.index][vertex]
                layer.data[loop].uv = (u, 1.0 - v)
    parts = mesh.attributes.new("parte", "INT", "FACE")
    parts.data.foreach_set("value", [data["trianglePart"][t] for t in first])
    if "triangleMolde" in data:
        moldes = mesh.attributes.new("molde", "INT", "FACE")
        moldes.data.foreach_set("value", [data["triangleMolde"][t] for t in first])
        table = bpy.data.texts.new("matter_moldes")
        table.write(json.dumps(data["moldes"]))
        table.use_fake_user = True
    images = [bpy.data.images.load(os.path.join(folder, name)) for name in data["textures"]]
    for material in data["materials"]:
        mat = bpy.data.materials.new(material["name"])
        mat.use_nodes = True
        shader = mat.node_tree.nodes["Principled BSDF"]
        color = material["color"]
        shader.inputs["Base Color"].default_value = (color[0], color[1], color[2], 1.0)
        shader.inputs["Roughness"].default_value = 0.8
        if material["texture"] >= 0:
            image = mat.node_tree.nodes.new("ShaderNodeTexImage")
            image.image = images[material["texture"]]
            image.interpolation = "Closest"
            mat.node_tree.links.new(image.outputs["Color"], shader.inputs["Base Color"])
        mesh.materials.append(mat)
    mesh.polygons.foreach_set("material_index", [data["triangleMaterial"][t] for t in first])
    for face in mesh.polygons:
        face.use_smooth = True
    body = bpy.data.objects.new("Jogador", mesh)
    bpy.context.collection.objects.link(body)

    # Os pesos: um grupo por junta usada.
    joints = data["joints"]
    groups = {}
    for v, pairs in enumerate(data["weights"]):
        for joint, weight in pairs:
            if joint not in groups:
                groups[joint] = body.vertex_groups.new(name=joints[joint]["name"])
            groups[joint].add([v], weight, "REPLACE")

    # A armadura: a cabeca do osso na junta; a ponta no filho (um so) ou 4 cm
    # para cima; os auxiliares na junta de baixo, presos a de cima.
    armature = bpy.data.armatures.new("Esqueleto")
    rig = bpy.data.objects.new("Esqueleto", armature)
    bpy.context.collection.objects.link(rig)
    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode="EDIT")
    children = {i: [] for i in range(len(joints))}
    for i, joint in enumerate(joints):
        if joint["parent"] >= 0 and joint["kind"] != "auxiliar" and "helper" not in joint:
            children[joint["parent"]].append(i)
    bones = []
    for i, joint in enumerate(joints):
        bone = armature.edit_bones.new(joint["name"])
        head = to_blender(joint["position"])
        if len(children[i]) == 1:
            tail = to_blender(joints[children[i][0]]["position"])
        else:
            tail = head + mathutils.Vector((0.0, 0.0, 0.04))
        if (tail - head).length < 1e-3:
            tail = head + mathutils.Vector((0.0, 0.0, 0.02))
        bone.head, bone.tail = head, tail
        bones.append(bone)
    # O auxiliar com o mesmo referencial do osso de baixo (o giro local dele e
    # o do osso de baixo), preso ao de cima.
    for i, joint in enumerate(joints):
        if "helper" in joint:
            lower = bones[joint["helper"]["to"]]
            bones[i].head, bones[i].tail, bones[i].roll = lower.head.copy(), lower.tail.copy(), lower.roll
    for i, joint in enumerate(joints):
        parent = joint["helper"]["from"] if "helper" in joint else joint["parent"]
        if parent >= 0:
            bones[i].parent = bones[parent]
    rest = {bone.name: bone.matrix.copy() for bone in bones}
    bpy.ops.object.mode_set(mode="OBJECT")
    # A pre-visualizacao das regras do motor (o motor e quem vale; aqui so
    # para pintar e conferir no Blender): a fracao do giro local do osso de
    # baixo; com eixo, so em volta dele; com sinal, so naquele sentido.
    for joint in joints:
        helper = joint.get("helper")
        if helper is None:
            continue
        pose = rig.pose.bones[joint["name"]]
        lower = joints[helper["to"]]["name"]
        copy = pose.constraints.new("COPY_ROTATION")
        copy.target, copy.subtarget = rig, lower
        copy.owner_space = copy.target_space = "LOCAL"
        copy.mix_mode = "REPLACE"
        copy.influence = helper["fraction"]
        axis = mathutils.Vector((helper["axis"][1], -helper["axis"][0], helper["axis"][2]))
        if axis.length > 1e-6:
            frame = rest[lower].to_3x3()
            local = [frame.col[k].normalized().dot(axis.normalized()) for k in range(3)]
            k = max(range(3), key=lambda n: abs(local[n]))
            copy.use_x, copy.use_y, copy.use_z = (k == 0, k == 1, k == 2)
            if helper["sign"] != 0.0:
                limit = pose.constraints.new("LIMIT_ROTATION")
                limit.owner_space = "LOCAL"
                positive = helper["sign"] * (1.0 if local[k] > 0.0 else -1.0) > 0.0
                low, high = (0.0, 3.14159) if positive else (-3.14159, 0.0)
                for n, name in enumerate("xyz"):
                    setattr(limit, f"use_limit_{name}", True)
                    setattr(limit, f"min_{name}", low if n == k else 0.0)
                    setattr(limit, f"max_{name}", high if n == k else 0.0)
    body.parent = rig
    modifier = body.modifiers.new("Esqueleto", "ARMATURE")
    modifier.object = rig

    # As regras do motor para a volta (sem nada por vertice ou por triangulo, que a
    # volta grava de novo pela malha, sem a tabela dos moldes, que fica no texto
    # dela, e sem o template da superficie, que superficie.py refaz pela troca).
    rules = {key: value for key, value in data.items()
             if key not in ("positions", "weights", "triangles", "moldes", "surface") and not key.startswith("triangle")}
    text = bpy.data.texts.new("matter_troca")
    text.write(json.dumps(rules))
    for image in images:
        image.pack()
    bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(target))
    print(f"MONTADO {target}: {len(mesh.vertices)} vertices, {len(mesh.polygons)} faces, {len(groups)} grupos, "
          f"{len(joints)} ossos")


main()
