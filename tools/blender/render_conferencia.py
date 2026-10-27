"""Folha de conferencia do .blend do jogador (sem janela).

    blender -b <jogador.blend> --python tools/blender/render_conferencia.py -- <saida_prefixo> [osso=graus_x,graus_y,graus_z ...]

Renderiza frente, 3/4 e lado com a textura (Workbench). Com ossos girados
(em graus, no referencial do osso), a pose antes de renderizar.
"""
import math
import sys

import bpy
import mathutils

argv = sys.argv[sys.argv.index("--") + 1:]
prefix = argv[0]
rig = bpy.data.objects["Esqueleto"]
for spec in argv[1:]:
    name, angles = spec.split("=")
    x, y, z = (math.radians(float(a)) for a in angles.split(","))
    bone = rig.pose.bones[name]
    bone.rotation_mode = "XYZ"
    bone.rotation_euler = (x, y, z)
bpy.context.view_layer.update()
scene = bpy.context.scene
scene.render.engine = "BLENDER_WORKBENCH"
scene.display.shading.light = "STUDIO"
scene.display.shading.color_type = "TEXTURE"
scene.render.resolution_x, scene.render.resolution_y = 700, 900
camera = bpy.data.objects.new("camera", bpy.data.cameras.new("camera"))
scene.collection.objects.link(camera)
scene.camera = camera
camera.data.type = "ORTHO"
camera.data.ortho_scale = 2.1
center = mathutils.Vector((0.0, 0.0, -0.05))
for label, direction in (("frente", (0, -1, 0)), ("34", (-0.7, -0.7, 0)), ("lado", (-1, 0, 0))):
    d = mathutils.Vector(direction).normalized()
    camera.location = center + d * 4.0
    camera.rotation_euler = (-d).to_track_quat("-Z", "Y").to_euler()
    scene.render.filepath = f"{prefix}_{label}.png"
    bpy.ops.render.render(write_still=True)
