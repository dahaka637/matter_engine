"""Render do MAP autorado em Blender, sem compilar/importar na Matter Engine."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector

sys.path.insert(0,str(Path(__file__).resolve().parent))
from compilar import parse, polygons, dot, UNITS, ASSETS
from campo_materiais import specs

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--mapa',type=Path,default=ASSETS/'src/campo_grande_01.map')
parser.add_argument('--saida',type=Path,required=True)
parser.add_argument('--vista',choices=['aerea','campo','fachada','posterior','degraus','alambrado'],default='aerea')
parser.add_argument('--largura',type=int,default=1400)
parser.add_argument('--amostras',type=int,default=12)
parser.add_argument('--salvar-blend',type=Path)
args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])

bpy.ops.wm.read_factory_settings(use_empty=True)
scene=bpy.context.scene
scene.render.engine='CYCLES'
scene.cycles.device='CPU'
scene.cycles.samples=args.amostras
scene.cycles.use_denoising=True
scene.cycles.max_bounces=4
scene.cycles.diffuse_bounces=2
scene.cycles.glossy_bounces=2
scene.render.threads_mode='FIXED'
scene.render.threads=1
scene.render.resolution_x=args.largura
scene.render.resolution_y=round(args.largura*(.875 if args.vista=='aerea' else .625))
scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG'
scene.view_settings.view_transform='AgX'
scene.view_settings.look='AgX - Medium High Contrast'
scene.view_settings.exposure=.25
scene.world=bpy.data.worlds.new('Ceu de conferencia')
scene.world.use_nodes=True
background=scene.world.node_tree.nodes['Background']
background.inputs['Color'].default_value=(.52,.64,.77,1)
background.inputs['Strength'].default_value=.45
sun_data=bpy.data.lights.new('Sol somente para previa','SUN')
sun_data.energy=2.7
sun_data.angle=.055
sun=bpy.data.objects.new(sun_data.name,sun_data)
scene.collection.objects.link(sun)
sun.rotation_euler=(math.radians(32),math.radians(-26),math.radians(-35))

materials={}
for name,spec in specs().items():
    mat=bpy.data.materials.new(name)
    mat.use_nodes=True
    bsdf=mat.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Roughness'].default_value=spec['roughness']
    bsdf.inputs['Metallic'].default_value=spec['metallic']
    image=bpy.data.images.load(str(ASSETS/'textures'/(name+'.png')),check_existing=True)
    tex=mat.node_tree.nodes.new('ShaderNodeTexImage')
    tex.image=image
    tex.extension='REPEAT'
    mat.node_tree.links.new(tex.outputs['Color'],bsdf.inputs['Base Color'])
    materials[name]=mat

groups=defaultdict(lambda:([],[],[]))
brushes=triangles=0
for props,solids in parse(args.mapa.read_text()):
    name=props.get('_tb_name',props.get('classname'))
    for solid in solids:
        brushes+=1
        for face,vertices in polygons(solid):
            normal,d,material,u,v,scales=face
            coords,faces,uvs=groups[(name,material)]
            first=len(coords)
            coords.extend(tuple(c/UNITS for c in p) for p in vertices)
            faces.append(tuple(range(first,first+len(vertices))))
            uvs.extend(((dot(p,u[:3])/scales[0]+u[3])/512,1-(dot(p,v[:3])/scales[1]+v[3])/512) for p in vertices)
            triangles+=len(vertices)-2
for (layer,material),(coords,faces,uvs) in groups.items():
    collection=bpy.data.collections.get(layer)
    if collection is None:
        collection=bpy.data.collections.new(layer)
        scene.collection.children.link(collection)
    mesh=bpy.data.meshes.new(layer+' / '+material)
    mesh.from_pydata(coords,[],faces)
    mesh.materials.append(materials[material])
    uv=mesh.uv_layers.new(name='Valve220')
    for polygon in mesh.polygons:
        for loop in polygon.loop_indices:uv.data[loop].uv=uvs[mesh.loops[loop].vertex_index]
    obj=bpy.data.objects.new(mesh.name,mesh)
    collection.objects.link(obj)

camera_data=bpy.data.cameras.new('Conferencia real do MAP')
camera=bpy.data.objects.new(camera_data.name,camera_data)
scene.collection.objects.link(camera)
camera_data.clip_end=600
camera_data.clip_start=.05
if args.vista=='aerea':
    camera.location=(0,-4,160)
    camera.rotation_euler=(0,0,math.pi/2)
    camera_data.type='ORTHO'
    camera_data.ortho_scale=142
else:
    position,target,lens={
        'campo':((-43,15,2.2),(-5,-42,2.6),30),
        'fachada':((-48,-63,5.0),(-27,-42,3.1),42),
        'posterior':((40,-72,6),(0,-45,2.4),27),
        'degraus':((10,-36.5,2.0),(3,-45,2.0),28),
        'alambrado':((44,-41.3,1.7),(40,-38.1,1.3),45),
    }[args.vista]
    camera.location=position
    camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler()
    camera_data.lens=lens
scene.camera=camera
args.saida.parent.mkdir(parents=True,exist_ok=True)
scene.render.filepath=str(args.saida.resolve())
print(json.dumps(dict(source=str(args.mapa),brushes=brushes,triangles=triangles,groups=len(groups),
                      vegetation_modeled=False,engine_imported=False,threads=1,view=args.vista)),flush=True)
if args.salvar_blend:
    bpy.ops.wm.save_as_mainfile(filepath=str(args.salvar_blend.resolve()))
bpy.ops.render.render(write_still=True)
