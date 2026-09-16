"""Reproducible Blender preparation of the supplied static Sketchfab mesh.

The artist landmarks belong to this asset adapter. Runtime never uses them.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import zipfile
import tempfile
import importlib.util

import bpy
from mathutils import Matrix, Vector


def render_preview(obj, path):
    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_WORKBENCH'
    scene.display.shading.light = 'STUDIO'
    scene.display.shading.color_type = 'TEXTURE' if obj.get('albedo_path') else 'SINGLE'
    scene.display.shading.single_color = (0.72, 0.78, 0.82)
    scene.display.shading.show_shadows = True
    scene.display.shading.show_cavity = True
    scene.world.color = (0.10, 0.10, 0.10)
    camera_data = bpy.data.cameras.new('Inspection')
    camera = bpy.data.objects.new('Inspection', camera_data)
    scene.collection.objects.link(camera)
    camera.location = (4.0, -0.7, 0.15)
    camera.rotation_euler = (Vector((0, 0, 0))-camera.location).to_track_quat('-Z','Y').to_euler()
    camera_data.type = 'ORTHO'
    camera_data.ortho_scale = 2.2
    scene.camera = camera
    scene.render.resolution_x = 900
    scene.render.resolution_y = 900
    scene.render.resolution_percentage = 100
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source')
    parser.add_argument('--output', required=True)
    parser.add_argument('--config', required=True)
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='matter-character-') as staging:
        with zipfile.ZipFile(args.source) as archive:
            # Exact known member; no arbitrary archive path extraction.
            original = Path(staging) / 'original.blend'
            original.write_bytes(archive.read('source/MeinCharacter.blend'))
        bpy.ops.wm.open_mainfile(filepath=str(original), use_scripts=False)
    obj = next(o for o in bpy.context.scene.objects if o.type == 'MESH')
    obj.name = 'LowPolyHuman'
    # The original mesh is local Y-up. Author's object rotation must not be
    # confused with the engine basis (+X forward, +Y left, +Z up).
    obj.matrix_world = Matrix.Identity(4)
    for v in obj.data.vertices:
        x, y, z = v.co
        v.co = (z, x, y)
    render_preview(obj, output / 'source-inspection.png')
    config_path = Path(args.config).resolve()
    config = json.loads(config_path.read_text())
    profile = json.loads((config_path.parent/config['profileTemplate']).read_text())
    profile['id'] = config['id']
    profile['standingRootHeightMeters'] = -min(v.co.z for v in obj.data.vertices)
    source = {name: tuple(Vector(p) for p in pair) for name,pair in config['core'].items()}
    target = dict(source)
    for side, sign in [('Left',1),('Right',-1)]:
        arm = [Vector((p[0],sign*p[1],p[2])) for p in config['leftArm']]
        arm_target = [arm[0].copy()]
        for a,b in zip(arm,arm[1:]):
            arm_target.append(arm_target[-1]+Vector((0,sign*(b-a).length,0)))
        leg = [Vector((p[0],sign*p[1],p[2])) for p in config['leftLeg']]
        for i,name in enumerate(['UpperArm','Forearm','Hand']):
            source[side+name] = (arm[i],arm[i+1])
            target[side+name] = (arm_target[i],arm_target[i+1])
        for i,name in enumerate(['Thigh','Shin','Foot']):
            source[side+name] = target[side+name] = (leg[i],leg[i+1])

    bpy.ops.object.select_all(action='DESELECT')
    armature = bpy.data.armatures.new('LowPolyPhysicalSkeleton')
    rig = bpy.data.objects.new('LowPolyPhysicalSkeleton', armature)
    bpy.context.collection.objects.link(rig)
    rig.select_set(True)
    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode='EDIT')
    for link in profile['links']:
        bone = armature.edit_bones.new(link['id'])
        bone.head,bone.tail = source[link['id']]
        if link['parent'] >= 0:
            bone.parent = armature.edit_bones[profile['links'][link['parent']]['id']]
    bpy.ops.object.mode_set(mode='OBJECT')
    obj.select_set(True)
    bpy.ops.object.parent_set(type='ARMATURE_AUTO')
    if not obj.vertex_groups:
        raise RuntimeError('Automatic weights failed')

    # Prune/normalize once, preserving heat diffusion around shoulder/hip.
    # Fingers are rigid with the hand, not pulled by adjacent forearm bones.
    names = [link['id'] for link in profile['links']]
    weights = []
    for vertex in obj.data.vertices:
        influences = [(obj.vertex_groups[g.group].name,g.weight) for g in vertex.groups if g.weight>1e-6]
        side='Left' if vertex.co.y>0 else 'Right'
        hand_a,hand_b=source[side+'Hand']
        hand_direction=(hand_b-hand_a).normalized()
        hand_distance=(vertex.co-hand_a).dot(hand_direction)
        if abs(vertex.co.y)>0.55 and hand_distance>0.01:
            influences = [(side+'Hand',1.0)]
            relaxed=config.get('relaxedHand')
            if relaxed and hand_distance>relaxed['bendStartMeters']:
                excess=hand_distance-relaxed['bendStartMeters']
                radius=relaxed['curlRadiusMeters']
                angle=excess/radius
                vertex.co += hand_direction*(radius*math.sin(angle)-excess)
                vertex.co.x += radius*(1.0-math.cos(angle))
                lateral=hand_direction.cross(Vector((1,0,0)))
                vertex.co -= lateral*((vertex.co-hand_a).dot(lateral)
                    *(1.0-relaxed['fingerSpreadScale']))
        if vertex.co.z < -0.855:
            influences = [('LeftFoot' if vertex.co.y>0 else 'RightFoot',1.0)]
        influences = sorted(influences,key=lambda pair:pair[1],reverse=True)[:4]
        total = sum(w for _,w in influences)
        if total < 1e-6:
            raise RuntimeError(f'Unweighted vertex {vertex.index}; repair rigging before export')
        normalized = [(name,w/total) for name,w in influences]
        weights.append(normalized)
    for group in obj.vertex_groups:
        group.remove(list(range(len(obj.data.vertices))))
    for i,influences in enumerate(weights):
        for name,w in influences:
            obj.vertex_groups[name].add([i],w,'REPLACE')

    # Pose the source arm chains into the physical neutral T pose, then bake
    # the surface and make that pose the new bind. No retarget offsets remain
    # hidden in the runtime.
    for link in profile['links']:
        name=link['id']
        a,b=target[name]
        source_a,source_b=source[name]
        rotation=(source_b-source_a).rotation_difference(b-a)
        rig.pose.bones[name].matrix = Matrix.Translation(a) @ rotation.to_matrix().to_4x4() @ Matrix.Translation(-source_a) @ armature.bones[name].matrix_local
        bpy.context.view_layer.update()
    evaluated=obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    positions=[v.co.copy() for v in evaluated.data.vertices]
    for vertex,position in zip(obj.data.vertices,positions):
        vertex.co=position
    for pose in rig.pose.bones:
        pose.matrix_basis=Matrix.Identity(4)
    bpy.context.view_layer.objects.active=rig
    bpy.ops.object.mode_set(mode='EDIT')
    for name,(a,b) in target.items():
        bone=armature.edit_bones[name]
        bone.head,bone.tail=a,b
        bone.roll=0
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.view_layer.update()

    for link in profile['links']:
        name=link['id']
        a,b=target[name]
        center=(a+b)*0.5 if name!='Pelvis' else Vector((0,0,0))
        link['position']=list(center)
        link['orientation']=[0,0,0,1]
        if 'joint' in link:
            link['joint']['anchor']=list(a)
        if name.endswith('Foot'):
            # Flat sole exactly matches the source floor. Keep the existing
            # physical foot box contract and contact telemetry.
            center=Vector((0.095,a.y,-profile['standingRootHeightMeters']+0.038))
            link['position']=list(center)
            link['box']['halfExtents']=[0.14,0.047,0.038]
            link['box']['position']=[0,0,0]
        else:
            region=name.removeprefix('Left').removeprefix('Right')
            radius=config['radiusMeters'][region]
            direction=b-a
            q=Vector((1,0,0)).rotation_difference(direction)
            link['capsule']['orientation']=[q.x,q.y,q.z,q.w]
            link['capsule']['radius']=radius
            link['capsule']['length']=max(direction.length,2*radius+0.002)
    profile_path=output/'LowPolyHumanV1.ragdoll.json'
    profile_path.write_text(json.dumps(profile,indent=2)+'\n')

    spec=importlib.util.spec_from_file_location('export_ragdoll_skin',Path(__file__).with_name('export_ragdoll_skin.py'))
    exporter=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    exporter.export_skin(obj,rig,profile,output/'human.skin.json')
    manifest=dict(schema='matter-ragdoll-character-1',id='low_poly_human',
        displayName=config['displayName'],physicsProfile=profile_path.name,
        skin='human.skin.json',flatShaded=True,thumbnail='source-inspection.png',
        attribution=dict(title='Low poly Character',author='favoritelike69',license='CC Attribution'))
    (output/'character.json').write_text(json.dumps(manifest,indent=2)+'\n')
    # Prepared editable source retains the real armature and normalized weights.
    obj['source_author']='favoritelike69'
    obj['source_license']='CC Attribution (Sketchfab download)'
    obj['physics_profile']=profile['id']
    rig.show_in_front=True
    bpy.ops.wm.save_as_mainfile(filepath=str(output/'human-rigged.blend'))
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True); rig.select_set(True)
    bpy.context.view_layer.objects.active=rig
    bpy.ops.export_scene.gltf(filepath=str(output/'human-rigged.glb'),export_format='GLB',
        use_selection=True,export_animations=False,export_extras=True)
    render_preview(obj,output/'bind-inspection.png')
    print('CHARACTER_EXPORTED',len(obj.data.vertices),'vertices',len(names),'bones')


if __name__ == '__main__':
    main()
