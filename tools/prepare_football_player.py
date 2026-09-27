"""Fit the user-supplied Mixamo Ch38 to the calibrated physical humanoid.

The physical anchors, collider shapes and animation coordinates are deliberately
preserved. Weighted segment transforms adapt the source mesh to those anchors;
this is a skin replacement, not a recalibration of the controller or gait.

Fingers are visual bones: three per finger, children of the physical hand,
exported after the eighteen physical bones (skin "visualBones"). They carry
no physics; each has a rest rotation that shapes the relaxed hand (curled in
a cascade from index to little finger, fingers together, thumb along the
index). Ch38 ships finger bones but almost no finger weights (whole fingers
ride the hand bone), so the hand is re-weighted here from geometry: each
vertex goes to the nearest bone segments - palm to the hand (wrist to every
knuckle), each phalanx to its bone, blended across the joints.

blender -b --factory-startup --disable-autoexec --python-exit-code 1 \\
  --python tools/prepare_football_player.py -- \\
  --source assets/characters/football_player/Ch38_nonPBR.fbx \\
  --output assets/characters/football_player
"""
import argparse
import importlib.util
import json
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

PRIMARY = {
    'Hips': 'Pelvis', 'Spine': 'Abdomen', 'Spine1': 'Chest',
    'Spine2': 'UpperChest', 'Neck': 'Neck', 'Head': 'Head',
}
for side in ('Left', 'Right'):
    PRIMARY.update({side + bone: side + link for bone, link in (
        ('Arm', 'UpperArm'), ('ForeArm', 'Forearm'), ('Hand', 'Hand'),
        ('UpLeg', 'Thigh'), ('Leg', 'Shin'), ('Foot', 'Foot'))})


FINGERS = ('Thumb', 'Index', 'Middle', 'Ring', 'Pinky')
# Relaxed hand. Flexion in degrees of each bone (fingers: MCP, PIP, DIP;
# thumb: CMC, MCP, IP), more toward the little finger, as in a resting hand;
# and how much of the modelled spread each finger closes toward the middle
# finger (the thumb toward the index).
RELAXED_FLEXION = {'Thumb': (9, 14, 18), 'Index': (23, 34, 17),
                   'Middle': (26, 38, 18), 'Ring': (29, 41, 18),
                   'Pinky': (32, 42, 20)}
RELAXED_CLOSE = {'Thumb': 0.6, 'Index': 0.8, 'Middle': 0.0, 'Ring': 0.8,
                 'Pinky': 0.8}
# Geometric hand weights: inverse distance to the bone segments, this power.
# Surface points sit ~1 cm from their own finger axis and ~2 cm from the
# neighbour's: (1/2)^8 leaves the neighbour under 0.5%.
HAND_WEIGHT_POWER = 8.0


def finger_bone(side, finger, index):
    return f'{side}{finger}{index}'


def physical_link(name):
    name = name.split(':')[-1]
    if name in PRIMARY:
        return PRIMARY[name]
    if name == 'HeadTop_End':
        return 'Head'
    for side in ('Left', 'Right'):
        if name == side + 'Shoulder':
            return 'UpperChest'
        if name.startswith(side + 'Hand'):
            return side + 'Hand'
        if name.startswith(side + 'Toe'):
            return side + 'Foot'
    raise ValueError('Unmapped source bone: ' + name)


def segment_transform(a, b, target_a, target_b, radial_scale):
    source = b - a
    target = target_b - target_a
    direction = source.normalized()
    rotation = direction.rotation_difference(target.normalized()).to_matrix()
    axial = target.length / source.length
    stretch = Matrix.Identity(3)
    for i in range(3):
        for j in range(3):
            stretch[i][j] = radial_scale * (i == j) + (axial - radial_scale) * direction[i] * direction[j]
    return Matrix.Translation(target_a) @ (rotation @ stretch).to_4x4() @ Matrix.Translation(-a)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    profile_path = out / 'FootballPlayerV1.ragdoll.json'
    profile = json.loads(profile_path.read_text())
    profile['id'] = 'FootballPlayerV1'
    links = {link['id']: link for link in profile['links']}
    order = list(links)
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.fbx(filepath=str(args.source.resolve()), use_anim=False)
    source_rig = next(o for o in bpy.context.scene.objects if o.type == 'ARMATURE')
    meshes = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    # Imported Mixamo faces -Y; MatterEngine faces +X, left +Y, up +Z.
    facing = Matrix.Rotation(np.pi / 2, 4, 'Z')
    heads = {b.name.split(':')[-1]: facing @ source_rig.matrix_world @ b.head_local
             for b in source_rig.data.bones}
    target = {name: Vector(link.get('joint', {}).get('anchor', [0, 0, 0]))
              for name, link in links.items()}
    reverse = {v: k for k, v in PRIMARY.items()}
    source = {name: heads[reverse[name]] for name in order}
    next_link = {'Pelvis': 'Abdomen', 'Abdomen': 'Chest', 'Chest': 'UpperChest',
                 'UpperChest': 'Neck', 'Neck': 'Head'}
    for side in ('Left', 'Right'):
        next_link.update({side + a: side + b for a, b in (
            ('UpperArm', 'Forearm'), ('Forearm', 'Hand'),
            ('Thigh', 'Shin'), ('Shin', 'Foot'))})
    transforms = {}
    for name in order:
        a, ta = source[name], target[name]
        if name in next_link:
            child = next_link[name]
            # Keep anatomical girth; only lengths change to match the rig.
            transforms[name] = segment_transform(a, source[child], ta, target[child], 1.0)
        elif name.endswith('Hand'):
            side = name.removesuffix('Hand')
            sign = 1 if side == 'Left' else -1
            b = heads[side + 'HandMiddle1']
            transforms[name] = segment_transform(a, b, ta, ta + Vector((0, sign * (b-a).length, 0)), 1.0)
        elif name == 'Head':
            transforms[name] = Matrix.Translation(ta - a)
        else:
            # Flat sole at the calibrated support plane. Keep ankle fixed.
            side = name.removesuffix('Foot')
            foot_points = [facing @ obj.matrix_world @ v.co for obj in meshes
                           if 'Shoes' in obj.name for v in obj.data.vertices
                           if (facing @ obj.matrix_world @ v.co).y * (1 if side == 'Left' else -1) > 0]
            floor = min(p.z for p in foot_points)
            scale_z = (ta.z + profile['standingRootHeightMeters']) / (a.z - floor)
            scale = Matrix.Diagonal(Vector((1.0, 1.0, scale_z, 1.0)))
            transforms[name] = Matrix.Translation(ta) @ scale @ Matrix.Translation(-a)

    tails = {b.name.split(':')[-1]: facing @ source_rig.matrix_world @ b.tail_local
             for b in source_rig.data.bones}
    hand_segments = {}
    for side in ('Left', 'Right'):
        wrist = heads[side + 'Hand']
        segments = []
        for finger in FINGERS:
            chain = [heads[f'{side}Hand{finger}{k}'] for k in (1, 2, 3, 4)]
            # Palm: wrist to the knuckle (thumb: to its base, the thenar
            # eminence belongs to the thumb's first bone).
            segments.append((side + 'Hand', wrist, chain[0]))
            for k in range(3):
                segments.append((finger_bone(side, finger, k + 1),
                                 chain[k], chain[k + 1]))
        hand_segments[side] = segments

    def hand_weights(side, point):
        """Share of the hand among palm and finger bones (source space)."""
        p = np.array(point)
        best = {}
        for bone, a, b in hand_segments[side]:
            a, b = np.array(a), np.array(b)
            ab = b - a
            t = np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0)
            distance = np.linalg.norm(p - (a + ab * t))
            weight = (distance + 0.002) ** -HAND_WEIGHT_POWER
            best[bone] = max(best.get(bone, 0.0), weight)
        top = sorted(best.items(), key=lambda x: x[1], reverse=True)[:3]
        total = sum(w for _, w in top)
        return {bone: w / total for bone, w in top if w / total > 0.01}

    # Visual finger bones in the fitted (target) space. The hand transform is
    # rigid (the hand keeps its length), so the fingers ride it unchanged.
    visual_bones, visual_heads, visual_parent_link = [], {}, {}
    for side in ('Left', 'Right'):
        hand = transforms[side + 'Hand']
        rotation = hand.to_3x3()
        source = {k: heads[k] for k in heads if k.startswith(side + 'Hand')}
        across = source[side + 'HandIndex1'] - source[side + 'HandPinky1']
        along = source[side + 'HandMiddle1'] - source[side + 'Hand']
        palm = across.cross(along).normalized()
        # Toward the palm: the thumb hangs on the palm side.
        if (source[side + 'HandThumb4'] - source[side + 'Hand']).dot(palm) < 0:
            palm = -palm
        palm = (rotation @ palm).normalized()

        def direction(finger, k):
            return (rotation @ (heads[f'{side}Hand{finger}{k + 1}']
                                - heads[f'{side}Hand{finger}{k}'])).normalized()

        def flat(v):
            return (v - palm * v.dot(palm)).normalized()

        for finger in FINGERS:
            parent = side + 'Hand'
            for k in (1, 2, 3):
                name = finger_bone(side, finger, k)
                head = hand @ heads[f'{side}Hand{finger}{k}']
                x = direction(finger, k)
                z = (palm - x * palm.dot(x)).normalized()
                y = z.cross(x)
                frame = Matrix((x, y, z)).transposed().to_quaternion()
                flex = Quaternion((0.0, -1.0, 0.0),
                                  np.radians(RELAXED_FLEXION[finger][k - 1]))
                rest = flex
                if k == 1 and RELAXED_CLOSE[finger] > 0.0:
                    toward = 'Index' if finger == 'Thumb' else 'Middle'
                    a, b = flat(x), flat(direction(toward, 1))
                    angle = np.arctan2(a.cross(b).dot(palm), a.dot(b))
                    rest = Quaternion((0.0, 0.0, 1.0),
                                      angle * RELAXED_CLOSE[finger]) @ flex
                visual_bones.append(dict(
                    name=name, parent=parent,
                    bindPosition=[float(v) for v in head],
                    bindOrientation=[frame.x, frame.y, frame.z, frame.w],
                    restRotation=[rest.x, rest.y, rest.z, rest.w]))
                visual_heads[name] = head
                visual_parent_link[name] = side + 'Hand'
                parent = name
    visual_names = [bone['name'] for bone in visual_bones]
    all_bones = order + visual_names

    # Two diffuse materials share one atlas because the character renderer uses
    # a single albedo. Embedded sources remain in the original FBX.
    atlas = np.ones((2048, 4096, 4), dtype=np.float32)
    alpha_images = {}
    material_slot = {}
    for material in bpy.data.materials:
        if not material.use_nodes:
            continue
        shader = next((n for n in material.node_tree.nodes if n.type == 'BSDF_PRINCIPLED'), None)
        if shader is None or not shader.inputs['Base Color'].is_linked:
            continue
        node = shader.inputs['Base Color'].links[0].from_node
        if node.type != 'TEX_IMAGE':
            raise RuntimeError('Expected embedded diffuse texture: ' + material.name)
        slot = 1 if 'hair' in material.name.lower() else 0
        material_slot[material.name] = slot
        img = node.image.copy()
        img.scale(2048, 2048)
        data = np.empty(2048 * 2048 * 4, dtype=np.float32)
        img.pixels.foreach_get(data)
        pixels = data.reshape(2048, 2048, 4)
        atlas[:, slot * 2048:(slot + 1) * 2048, :] = pixels
        alpha_images[slot] = pixels[:, :, 3].copy()
    image = bpy.data.images.new('Football albedo atlas', width=4096, height=2048, alpha=True)
    image.pixels.foreach_set(atlas.ravel())
    image.filepath_raw = str(out / 'albedo.png')
    image.file_format = 'PNG'
    image.save()
    image.pack()
    print('ALPHA_RANGES', {i: (float(a.min()), float(a.max())) for i, a in alpha_images.items()})

    # Fit each point using its original smooth weights, then retain the same
    # weights on the eighteen physical links. No nearest-bone guessing.
    vertices, faces, face_uvs = [], [], []
    fitted_weights = []
    for obj in meshes:
        mapping = {g.index: physical_link(g.name) for g in obj.vertex_groups}
        points, weights = [], []
        for v in obj.data.vertices:
            merged = {}
            for influence in v.groups:
                if influence.weight > 1e-6:
                    name = mapping[influence.group]
                    merged[name] = merged.get(name, 0) + influence.weight
            if not merged:
                raise RuntimeError('Unweighted source vertex')
            total = sum(merged.values())
            merged = {name: w / total for name, w in merged.items()}
            point = facing @ obj.matrix_world @ v.co
            points.append(sum((transforms[n] @ point * w for n, w in merged.items()), Vector()))
            for side in ('Left', 'Right'):
                share = merged.pop(side + 'Hand', 0.0)
                for bone, w in (hand_weights(side, point).items() if share else ()):
                    merged[bone] = merged.get(bone, 0.0) + share * w
            weights.append(np.array([merged.get(n, 0) for n in all_bones]))
        obj.data.calc_loop_triangles()
        uv_layer = obj.data.uv_layers.active
        if uv_layer is None:
            raise RuntimeError('Missing UVs: ' + obj.name)

        def alpha(v, slot):
            uv = v[1]
            return float(alpha_images[slot][min(2047, max(0, int(uv[1] * 2048))),
                                             min(2047, max(0, int(uv[0] * 2048)))])

        def mix(a, b, t):
            return tuple(x * (1-t) + y * t for x, y in zip(a, b))

        def emit(triangle, slot):
            start = len(vertices)
            for point, uv, weight in triangle:
                vertices.append(tuple(point))
                fitted_weights.append(weight)
                # Blender UV origin is bottom-left; exporter flips V once.
                face_uvs.append(((float(uv[0]) + slot) / 2, float(uv[1])))
            faces.append((start, start + 1, start + 2))

        def cutout(triangle, slot, depth=0):
            # Runtime is opaque. Convert hair/eyelash alpha cards to geometry
            # instead of drawing rectangular cards or changing global shaders.
            a, b, c = triangle
            ab, bc, ca = mix(a,b,.5), mix(b,c,.5), mix(c,a,.5)
            samples = [alpha(v, slot) for v in (a,b,c,ab,bc,ca,mix(ab,c,1/3))]
            if max(samples) < .45:
                return
            if min(samples) >= .45:
                emit(triangle, slot)
                return
            if depth < 1:
                for tri in ((a,ab,ca), (ab,b,bc), (ca,bc,c), (ab,bc,ca)):
                    cutout(tri, slot, depth+1)
                return
            polygon = []
            for u, v in ((a,b),(b,c),(c,a)):
                au, av = alpha(u,slot), alpha(v,slot)
                if au >= .45:
                    polygon.append(u)
                if (au >= .45) != (av >= .45):
                    polygon.append(mix(u,v,(.45-au)/(av-au)))
            for i in range(1,len(polygon)-1):
                emit((polygon[0],polygon[i],polygon[i+1]),slot)

        for tri in obj.data.loop_triangles:
            slot = material_slot[obj.data.materials[tri.material_index].name]
            triangle = tuple((points[index], np.array(uv_layer.data[loop].uv), weights[index])
                             for index,loop in zip(tri.vertices,tri.loops))
            if slot == 1:
                cutout(triangle, slot)
            else:
                emit(triangle, slot)

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    mesh = bpy.data.meshes.new('FootballPlayer')
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new('FootballPlayer', mesh)
    bpy.context.collection.objects.link(obj)
    uv = mesh.uv_layers.new(name='UVMap')
    for loop in mesh.loops:
        uv.data[loop.index].uv = face_uvs[loop.vertex_index]
    for i, name in enumerate(all_bones):
        group = obj.vertex_groups.new(name=name)
        for index, w in enumerate(fitted_weights):
            if w[i] > 1e-6:
                group.add([index], float(w[i]), 'REPLACE')
    # Merge triangle corners so normals are smooth across the original mesh.
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles(threshold=0.000001)
    bpy.ops.object.mode_set(mode='OBJECT')
    for polygon in mesh.polygons:
        polygon.use_smooth = True
    rig_data = bpy.data.armatures.new('FootballPhysicalRig')
    rig = bpy.data.objects.new('FootballPhysicalRig', rig_data)
    bpy.context.collection.objects.link(rig)
    obj.select_set(False)
    rig.select_set(True)
    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode='EDIT')
    for name in order:
        bone = rig_data.edit_bones.new(name)
        bone.head = target[name]
        if name in next_link:
            bone.tail = target[next_link[name]]
        else:
            bone.tail = bone.head + Vector((0,0,.08))
        parent = links[name]['parent']
        if parent >= 0:
            bone.parent = rig_data.edit_bones[order[parent]]
    for visual in visual_bones:
        bone = rig_data.edit_bones.new(visual['name'])
        bone.head = Vector(visual['bindPosition'])
        frame = Quaternion((visual['bindOrientation'][3], *visual['bindOrientation'][:3]))
        bone.tail = bone.head + frame @ Vector((0.02, 0.0, 0.0))
        bone.parent = rig_data.edit_bones[visual['parent']]
    bpy.ops.object.mode_set(mode='OBJECT')
    modifier = obj.modifiers.new('Physical skin', 'ARMATURE')
    modifier.object = rig
    material = bpy.data.materials.new('Football kit')
    material.use_nodes = True
    shader = material.node_tree.nodes.get('Principled BSDF')
    shader.inputs['Roughness'].default_value = .75
    node = material.node_tree.nodes.new('ShaderNodeTexImage')
    node.image = image
    material.node_tree.links.new(node.outputs['Color'], shader.inputs['Base Color'])
    mesh.materials.append(material)
    obj['albedo_path'] = 'albedo.png'
    obj['smooth_skin'] = True
    discarded = []
    for vertex in mesh.vertices:
        weights = sorted((g.weight for g in vertex.groups), reverse=True)
        discarded.append(sum(weights[4:]) / max(sum(weights), 1e-8))
    print('WEIGHT_REPORT', max(discarded), 'over2pct', sum(w > .02 for w in discarded), flush=True)
    obj['maximum_discarded_skin_weight'] = .04  # measured maximum: 3.785%, only 12 vertices > 2%
    profile_path.write_text(json.dumps(profile, indent=2) + '\n')
    exporter_path = Path(__file__).resolve().parent / 'export_ragdoll_skin.py'
    spec = importlib.util.spec_from_file_location('export_ragdoll_skin', exporter_path)
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    exporter.export_skin(obj, rig, profile, str(out / 'football.skin.json'),
                         strict_anchors=True, visual_bones=visual_bones)
    # Share only identical vertices, preserving UV seams, normals and weights.
    skin_path = out / 'football.skin.json'
    skin = json.loads(skin_path.read_text())
    unique, remap, indices = [], {}, []
    for vertex in skin['vertices']:
        key = tuple(x for field in ('position', 'normal', 'uv', 'color', 'joints', 'weights')
                    for x in vertex[field])
        if key not in remap:
            remap[key] = len(unique)
            unique.append(vertex)
        indices.append(remap[key])
    skin['vertices'], skin['indices'] = unique, indices
    skin_path.write_text(json.dumps(skin, separators=(',', ':')) + '\n')
    bpy.data.orphans_purge(do_recursive=True)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(out / 'football-rigged.blend'), compress=True)
    print('FOOTBALL_READY', len(mesh.vertices), 'vertices', len(mesh.polygons), 'triangles')


if __name__ == '__main__':
    main()
