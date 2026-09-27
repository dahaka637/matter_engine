"""Adapt the authored Unity/Mecanim rig; preserve its topology, UV and weights.

Run in Blender with --disable-autoexec. The Rigify UI scripts are not needed.
"""
import argparse
import importlib.util
import json
import sys
import tempfile
from pathlib import Path
import zipfile

import bpy
from mathutils import Matrix, Vector


def load_tool(name):
    spec=importlib.util.spec_from_file_location(name,Path(__file__).with_name(name+'.py'))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('source')
    parser.add_argument('--output',required=True)
    parser.add_argument('--profile-template',required=True)
    args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    output=Path(args.output).resolve(); output.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='matter-crash-') as staging:
        with zipfile.ZipFile(args.source) as archive:
            original=Path(staging)/'source.blend'
            original.write_bytes(archive.read('CrashTestDummy_UnityMecanim.blend'))
            (output/'albedo.png').write_bytes(archive.read('man_neutral_Texture1.png'))
            (output/'SOURCE-LICENSE.html').write_bytes(archive.read('88294 - LICENSE.html'))
        bpy.ops.wm.open_mainfile(filepath=str(original),use_scripts=False)
    obj=bpy.data.objects['man']; rig=bpy.data.objects['metarig']
    obj.name='CrashTestDummy'; rig.name='CrashDummySkeleton'
    bpy.context.scene.render.engine='BLENDER_EEVEE'
    # Normalize to 1.8 m, origin at the authored pelvis, engine Z-up/+X front.
    scale=1.8/(max(v.co.z for v in obj.data.vertices)-min(v.co.z for v in obj.data.vertices))
    origin=rig.data.bones['hips'].head_local.copy()
    basis=Matrix(((0,-1,0,0),(1,0,0,0),(0,0,1,0),(0,0,0,1)))
    transform=Matrix.Diagonal((scale,scale,scale,1))@basis@Matrix.Translation(-origin)
    obj.data.transform(transform); rig.data.transform(transform)
    rig.data.use_mirror_x=False
    bpy.context.view_layer.update()
    source_positions={b.name:(b.head_local.copy(),b.tail_local.copy()) for b in rig.data.bones}

    aliases={'hips':'Pelvis','spine':'Abdomen','chest':'Chest','neck':'Neck','head':'Head'}
    for side,suffix in [('Left','L'),('Right','R')]:
        for source,target in [('upper_arm','UpperArm'),('forearm','Forearm'),('hand','Hand'),
                              ('thigh','Thigh'),('shin','Shin'),('foot','Foot')]:
            aliases[source+'.'+suffix]=side+target
    for source,target in aliases.items():
        rig.data.bones[source].name=target
        # Blender usually propagates the rename to vertex groups; handle
        # versions where it doesn't without creating duplicate groups.
        if source in obj.vertex_groups:
            obj.vertex_groups[source].name=target
    bpy.ops.object.select_all(action='DESELECT'); rig.select_set(True)
    bpy.context.view_layer.objects.active=rig
    bpy.ops.object.mode_set(mode='EDIT')
    chest=rig.data.edit_bones['Chest']; split=chest.head.lerp(chest.tail,0.55)
    upper=rig.data.edit_bones.new('UpperChest'); upper.head=split; upper.tail=chest.tail.copy()
    upper.roll=chest.roll; upper.parent=chest; chest.tail=split
    for bone in list(rig.data.edit_bones):
        if bone.parent==chest and bone!=upper:
            bone.parent=upper
    bpy.ops.object.mode_set(mode='OBJECT')
    chest_group=obj.vertex_groups['Chest']; upper_group=obj.vertex_groups.new(name='UpperChest')
    for v in obj.data.vertices:
        original_weight=next((g.weight for g in v.groups if g.group==chest_group.index),0)
        alpha=max(0,min(1,(v.co.z-(split.z-0.075))/0.15))
        chest_group.add([v.index],original_weight*(1-alpha),'REPLACE')
        upper_group.add([v.index],original_weight*alpha,'REPLACE')

    # Keep finger/hand and toe detail in the editable skeleton. Runtime
    # influences resolve to their physical parent until those DOFs are used.
    physical_names=set(aliases.values())|{'UpperChest'}
    for bone in rig.data.bones:
        if bone.name in physical_names:
            bone['physics_link']=bone.name
        elif bone.name.startswith('shoulder.'):
            bone['physics_link']='UpperChest'
        elif bone.name.endswith(('.L','.R')):
            side='Left' if bone.name.endswith('.L') else 'Right'
            bone['physics_link']=side+('Foot' if bone.name.startswith(('toe.','heel.')) else 'Hand')
        else:
            raise RuntimeError('Unmapped bone '+bone.name)

    # Pose the authored A-pose into the canonical T-pose, retaining each
    # segment length. Descendant finger bones follow the hand unchanged.
    targets={b.name:(b.head_local.copy(),b.tail_local.copy()) for b in rig.data.bones}
    rotations={}
    for side,sign in [('Left',1),('Right',-1)]:
        shoulder=rig.data.bones[side+'UpperArm'].head_local.copy()
        print('SOURCE_SHOULDER',side,tuple(shoulder))
        for part in ['UpperArm','Forearm','Hand']:
            name=side+part; bone=rig.data.bones[name]
            length=bone.length
            direction=Vector((0,sign,0))
            q=(bone.tail_local-bone.head_local).rotation_difference(direction)
            targets[name]=(shoulder.copy(),shoulder+direction*length)
            rotations[name]=q
            shoulder=targets[name][1]
    for side in ['Left','Right']:
        for part in ['UpperArm','Forearm','Hand']:
            name=side+part; bone=rig.data.bones[name]; a,b=targets[name]
            rig.pose.bones[name].matrix=(Matrix.Translation(a)@rotations[name].to_matrix().to_4x4()
                @Matrix.Translation(-bone.head_local)@bone.matrix_local)
            bpy.context.view_layer.update()
    # Bake geometry and every visual bone's posed matrix together.
    evaluated=obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    baked=[v.co.copy() for v in evaluated.data.vertices]
    posed={b.name:(rig.pose.bones[b.name].head.copy(),rig.pose.bones[b.name].tail.copy(),
                   rig.pose.bones[b.name].matrix.copy()) for b in rig.data.bones}
    print('POSED_SHOULDERS',tuple(posed['LeftUpperArm'][0]),tuple(posed['RightUpperArm'][0]))
    for vertex,p in zip(obj.data.vertices,baked): vertex.co=p
    for pose in rig.pose.bones: pose.matrix_basis=Matrix.Identity(4)
    bpy.ops.object.mode_set(mode='EDIT')
    for name,(head,tail,matrix) in posed.items():
        bone=rig.data.edit_bones[name]
        bone.use_connect=False; bone.matrix=matrix; bone.length=(tail-head).length
    bpy.ops.object.mode_set(mode='OBJECT'); bpy.context.view_layer.update()

    profile=json.loads(Path(args.profile_template).read_text()); profile['id']='CrashTestDummyV1'
    profile['standingRootHeightMeters']=-min(v.co.z for v in obj.data.vertices)
    radii={'Pelvis':0.095,'Abdomen':0.090,'Chest':0.11,'UpperChest':0.115,'Neck':0.041,
           'Head':0.078,'UpperArm':0.045,'Forearm':0.039,'Hand':0.03,'Thigh':0.065,'Shin':0.044}
    for link in profile['links']:
        bone=rig.data.bones[link['id']]; a,b=bone.head_local,bone.tail_local
        center=(a+b)*0.5
        if link['id']=='Pelvis': center=Vector((0,0,0))
        link['position']=list(center); link['orientation']=[0,0,0,1]
        if 'joint' in link: link['joint']['anchor']=list(a)
        if link['id'].endswith('Foot'):
            center=Vector((a.x+0.035,a.y,-profile['standingRootHeightMeters']+0.036))
            link['position']=list(center); link['box']['position']=[0,0,0]
            link['box']['halfExtents']=[0.12,0.047,0.036]
        else:
            radius=radii[link['id'].removeprefix('Left').removeprefix('Right')]
            direction=b-a
            if link['id']=='Pelvis': direction=Vector((0,0.25,0))
            q=Vector((1,0,0)).rotation_difference(direction)
            link['capsule'].update(radius=radius,length=max(direction.length,2*radius+0.002),
                orientation=[q.x,q.y,q.z,q.w])
            # Get-up can use forearms/thighs/abdomen too. Only feet count as
            # walking support; all links report external resting contact.
            link['capsule']['contactSensor']=True
    profile_path=output/'CrashTestDummyV1.ragdoll.json'
    profile_path.write_text(json.dumps(profile,indent=2)+'\n')

    material=bpy.data.materials.new('Crash Dummy Yellow'); material.use_nodes=True
    texture=bpy.data.images.load(str(output/'albedo.png')); texture.pack()
    node=material.node_tree.nodes.new('ShaderNodeTexImage'); node.image=texture
    shader=material.node_tree.nodes.get('Principled BSDF')
    shader.inputs['Roughness'].default_value=0.72
    material.node_tree.links.new(node.outputs['Color'],shader.inputs['Base Color'])
    obj.data.materials.clear(); obj.data.materials.append(material)
    for polygon in obj.data.polygons: polygon.use_smooth=True
    obj['albedo_path']='albedo.png'; obj['smooth_skin']=True
    obj['maximum_discarded_skin_weight']=0.02
    obj['source_license']='CC0 1.0'; obj['physics_profile']=profile['id']
    # Keep author weights; only merge aliases and trim numerically tiny
    # surplus influences to the four GPU channels, with an explicit report.
    max_discarded=0
    for vertex in obj.data.vertices:
        groups=sorted([(g.group,g.weight) for g in vertex.groups if g.weight>1e-6],key=lambda p:p[1],reverse=True)
        # Exporter merges helper/finger aliases first, so many source weights
        # still collapse to a single physical hand/foot influence.
        merged={}
        for group,weight in groups:
            name=obj.vertex_groups[group].name
            mapped=rig.data.bones[name]['physics_link']
            merged[mapped]=merged.get(mapped,0)+weight
        ordered=sorted(merged.items(),key=lambda p:p[1],reverse=True)
        max_discarded=max(max_discarded,sum(w for _,w in ordered[4:]))
    print('AUTHORED_WEIGHTS maximum discarded after physical mapping:',max_discarded)
    load_tool('export_ragdoll_skin').export_skin(obj,rig,profile,output/'dummy.skin.json')
    manifest=dict(schema='matter-ragdoll-character-1',id='crash_test_dummy',displayName='Crash Test Dummy',
        physicsProfile=profile_path.name,skin='dummy.skin.json',albedo='albedo.png',flatShaded=False,
        thumbnail='thumbnail.png',locomotion=dict(idle='idle_standard_dummy',run='run_forward_dummy',stop='run_to_stop_dummy',
            walkForward='walk_forward_dummy',walkBackward='walk_backward_dummy',
            walkLeft='walk_strafe_left_dummy',walkRight='walk_strafe_right_dummy',runBackward='run_backward_dummy',
            standUpFront='stand_up_front_dummy',standUpBack='stand_up_back_dummy'),
        attribution=dict(title='Crash Test Dummy mark1',license='CC0-1.0',
        source='https://www.blendswap.com/blends/view/88294'))
    (output/'character.json').write_text(json.dumps(manifest,indent=2)+'\n')
    bpy.context.scene.render.engine='BLENDER_WORKBENCH'
    rig.show_in_front=True
    bpy.context.preferences.filepaths.save_version=0
    bpy.ops.object.select_all(action='DESELECT'); obj.select_set(True); rig.select_set(True)
    bpy.context.view_layer.objects.active=rig
    bpy.ops.export_scene.gltf(filepath=str(output/'dummy-rigged.glb'),export_format='GLB',
        use_selection=True,export_animations=False,export_extras=True)
    preview=load_tool('prepare_low_poly_character')
    preview.render_preview(obj,output/'thumbnail.png')
    bpy.ops.wm.save_as_mainfile(filepath=str(output/'dummy-rigged.blend'))
    print('CRASH_DUMMY_READY',len(obj.data.vertices),'vertices',len(obj.data.loop_triangles),
          'triangles',len(rig.data.bones),'visual bones',len(profile['links']),'physical links')


if __name__=='__main__': main()
