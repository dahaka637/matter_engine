"""Cook an artist-prepared Blender skin against an explicit physical profile.

blender -b --disable-autoexec human-rigged.blend --python-exit-code 1 \
  --python tools/export_ragdoll_skin.py -- --profile path/profile.json \
  --output path/human.skin.json

Input: neutral mesh already fitted to the physical anchors, named vertex
groups matching link IDs (or bones with a physics_link custom property).
This exporter never guesses anatomical correspondences.
"""
import argparse
import json
from pathlib import Path
import sys

import bpy
from mathutils import Vector


def export_skin(obj, rig, profile, output, strict_anchors=True):
    names=[link['id'] for link in profile['links']]
    mapping={bone.name:bone.get('physics_link',bone.name) for bone in rig.data.bones}
    for link in profile['links'][1:]:
        bone=rig.data.bones.get(link['id']) or next((b for b in rig.data.bones if mapping[b.name]==link['id']),None)
        if bone is None:
            raise RuntimeError('Missing physical bone: '+link['id'])
        if strict_anchors and ((rig.matrix_world@bone.head_local)-Vector(link['joint']['anchor'])).length>0.001:
            raise RuntimeError('Bone anchor does not match profile: '+link['id'])
    if any(abs(pose.matrix_basis[i][j]-(1.0 if i==j else 0.0))>1e-5
           for pose in rig.pose.bones for i in range(4) for j in range(4)):
        raise RuntimeError('Export requires neutral bind pose, not a posed animation')
    obj.data.update()
    obj.data.calc_loop_triangles()
    weights=[]
    maximum_discarded=0.0
    for vertex in obj.data.vertices:
        merged={}
        for group in vertex.groups:
            if group.weight<=1e-6:
                continue
            name=obj.vertex_groups[group.group].name
            link=mapping.get(name)
            if link not in names:
                raise RuntimeError('Unmapped weighted bone: '+name)
            merged[link]=merged.get(link,0.0)+group.weight
        influences=sorted(merged.items(),key=lambda x:x[1],reverse=True)
        if not influences:
            raise RuntimeError(f'Unweighted vertex {vertex.index}')
        discarded=sum(w for _,w in influences[4:])
        maximum_discarded=max(maximum_discarded,discarded)
        if discarded>obj.get('maximum_discarded_skin_weight',0.01):
            raise RuntimeError(f'More than four significant weights at vertex {vertex.index}')
        influences=influences[:4]
        total=sum(w for _,w in influences)
        weights.append([(names.index(n),w/total) for n,w in influences])
    normal_matrix=obj.matrix_world.to_3x3().inverted().transposed()
    vertices=[]
    textured=bool(obj.get('albedo_path'))
    uv_layer=obj.data.uv_layers.active
    for triangle in obj.data.loop_triangles:
        for index,loop_index in zip(triangle.vertices,triangle.loops):
            influences=weights[index]
            ids=[i for i,_ in influences]+[0]*(4-len(influences))
            values=[w for _,w in influences]+[0.0]*(4-len(influences))
            normal=obj.data.vertices[index].normal if obj.get('smooth_skin',False) else triangle.normal
            uv=uv_layer.data[loop_index].uv if uv_layer else (0.0,0.0)
            vertices.append(dict(position=list(obj.matrix_world@obj.data.vertices[index].co),
                normal=list((normal_matrix@normal).normalized()),
                uv=[uv[0],1.0-uv[1]],
                color=[1.0,1.0,1.0] if textured else [0.72,0.78,0.82],joints=ids,weights=values))
    skin=dict(schema='matter-ragdoll-skin-1',rigId=profile['id'],
        exportReport=dict(maximumDiscardedWeight=maximum_discarded),
        bones=[dict(link=l['id'],bindPosition=l['position'],
                    bindOrientation=l.get('orientation',[0,0,0,1])) for l in profile['links']],
        vertices=vertices,indices=list(range(len(vertices))))
    Path(output).write_text(json.dumps(skin,separators=(',',':'))+'\n')
    print('SKIN_EXPORTED',len(vertices)//3,'triangles;',len(names),'physical bones')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--profile',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--mesh',default='LowPolyHuman')
    parser.add_argument('--armature',default='LowPolyPhysicalSkeleton')
    args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    export_skin(bpy.data.objects[args.mesh],bpy.data.objects[args.armature],
        json.loads(Path(args.profile).read_text()),args.output)
