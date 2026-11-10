"""Renderiza o GLB real para conferir o mapa; Blender e apenas visualizador."""
import argparse
from pathlib import Path
import sys

import bpy
from mathutils import Vector

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--mapa', type=Path, required=True)
parser.add_argument('--saida', type=Path, required=True)
parser.add_argument('--vista', choices=['geral','bancos','entrada','estrutura','encaixes','portao','sinalizacao'], default='geral')
parser.add_argument('--largura', type=int, default=1600)
parser.add_argument('--amostras', type=int, default=16)
args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=str(args.mapa.resolve()))
scene=bpy.context.scene
scene.render.engine='CYCLES'
scene.cycles.device='CPU'
scene.cycles.samples=args.amostras
scene.cycles.use_denoising=True
scene.cycles.max_bounces=5
scene.cycles.diffuse_bounces=3
scene.cycles.glossy_bounces=2
scene.render.threads_mode='FIXED'
scene.render.threads=1
scene.render.resolution_x=args.largura
scene.render.resolution_y=round(args.largura*9/16)
scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG'
scene.world=bpy.data.worlds.new('Ambiente interno')
scene.world.use_nodes=True
scene.world.node_tree.nodes['Background'].inputs['Color'].default_value=(.12,.15,.18,1)
scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value=.12
scene.view_settings.view_transform='AgX'
scene.view_settings.look='AgX - Medium High Contrast'
scene.view_settings.exposure=3 if args.vista=='encaixes' else (2.3 if args.vista in ('sinalizacao','bancos') else 1.8)
for light in bpy.data.lights:
    light.shadow_soft_size=.32
print('RENDER:',scene.cycles.device,'LUZES:',[round(l.energy,2) for l in bpy.data.lights][:4],flush=True)

views={
    'geral': ((-22,-12,6.8),(3,1,1.3),21),
    'bancos': ((-12.3,12,1.4),(-5,13.2,1),26),
    'entrada': ((11.8,-12.7,1.8),(17,-14.1,1.6),26),
    'estrutura': ((-19,-9,2.0),(2,4,5.4),22),
    'encaixes': ((19,-12.6,2),(21.3,-14.1,1.9),32),
    'portao': ((-16.7,14,3.2),(-13.9,11.4,1),21),
    'sinalizacao': ((17,-12.2,2.45),(16.5,-14.2,2.27),23),
}
position,target,lens=views[args.vista]
camera=bpy.data.cameras.new('Camera de conferencia')
obj=bpy.data.objects.new('Camera de conferencia',camera)
scene.collection.objects.link(obj)
obj.location=position
obj.rotation_euler=(Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()
camera.lens=lens
camera.clip_start=.05
camera.clip_end=180
scene.camera=obj
args.saida.parent.mkdir(parents=True,exist_ok=True)
scene.render.filepath=str(args.saida.resolve())
print(f'PREVIA REAL DO GLB: {len(bpy.data.objects)} objetos, {len(bpy.data.lights)} luzes',flush=True)
bpy.ops.render.render(write_still=True)
