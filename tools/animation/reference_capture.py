"""Captura as articulacoes de uma animacao de referencia (FBX/GLB) para estudo.

Referencia e para medir, nao para copiar: este script nao gera clipe. Ele
exporta, quadro a quadro, a posicao das articulacoes que interessam a uma
passada (quadril, joelho, tornozelo, bola e ponta do pe, ombro, cotovelo,
punho, pescoco, cabeca). Tudo ja vai no nosso referencial - frente +X,
esquerda +Y, cima +Z - e na escala do nosso corpo, pelo comprimento da perna.
Quem mede e gait_metrics.py, com as mesmas definicoes usadas nos nossos
clipes.

A base e anatomica, tirada da pose de repouso (quadris, cabeca), e nao dos
eixos declarados no arquivo: FBX muda de convencao entre exportadores.

Uso:
  blender --background --factory-startup --python-exit-code 1 \\
    --python tools/animation/reference_capture.py -- "Slow Run.fbx" \\
    --output /tmp/slow_run.capture.json
"""

import argparse
import json
import pathlib
import sys

import bpy
from mathutils import Matrix

# Articulacao -> osso Mixamo cuja CABECA marca o ponto.
LANDMARKS = {
    "Hips": "Hips", "Chest": "Spine2", "Neck": "Neck", "Head": "Head",
    "HeadTop": "HeadTop_End",
    "LeftHip": "LeftUpLeg", "LeftKnee": "LeftLeg", "LeftAnkle": "LeftFoot",
    "LeftBall": "LeftToeBase", "LeftToe": "LeftToe_End",
    "RightHip": "RightUpLeg", "RightKnee": "RightLeg",
    "RightAnkle": "RightFoot", "RightBall": "RightToeBase",
    "RightToe": "RightToe_End",
    "LeftShoulder": "LeftArm", "LeftElbow": "LeftForeArm",
    "LeftWrist": "LeftHand",
    "RightShoulder": "RightArm", "RightElbow": "RightForeArm",
    "RightWrist": "RightHand",
}

# Quadril ao tornozelo do FootballPlayerV1 (m): a escala do nosso corpo.
TARGET_LEG_LENGTH = 0.8277


def arguments():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    return parser.parse_args(argv)


def main():
    args = arguments()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    if args.input.suffix.lower() == ".fbx":
        bpy.ops.import_scene.fbx(filepath=str(args.input))
    else:
        bpy.ops.import_scene.gltf(filepath=str(args.input))
    armature = next(o for o in bpy.context.scene.objects
                    if o.type == "ARMATURE")
    action = armature.animation_data.action
    bones = {b.name.split(":")[-1]: b for b in armature.data.bones}
    pose = {b.name.split(":")[-1]: b for b in armature.pose.bones}
    missing = [name for name in LANDMARKS.values() if name not in bones]
    if missing:
        raise RuntimeError("Ossos ausentes: " + ", ".join(missing))

    world = armature.matrix_world
    rest = {key: world @ bones[name].head_local
            for key, name in LANDMARKS.items()}
    up = (rest["Head"] - rest["Hips"]).normalized()
    left = rest["LeftHip"] - rest["RightHip"]
    left = (left - up * left.dot(up)).normalized()
    forward = left.cross(up).normalized()
    basis = Matrix((forward, left, up))
    leg = abs((rest["LeftHip"] - rest["LeftAnkle"]).dot(up))
    scale = TARGET_LEG_LENGTH / leg

    start, end = (int(round(v)) for v in action.frame_range)
    fps = bpy.context.scene.render.fps / bpy.context.scene.render.fps_base
    positions = {key: [] for key in LANDMARKS}
    for frame in range(start, end + 1):
        bpy.context.scene.frame_set(frame)
        for key, name in LANDMARKS.items():
            point = basis @ (world @ pose[name].head) * scale
            positions[key].append([round(v, 5) for v in point])

    capture = {
        "source": args.input.name,
        # Pose de repouso, mesma base e escala: inclinacoes (tronco, cabeca,
        # pe) sao medidas em relacao a ela, porque os ossos Mixamo ja vem
        # inclinados de fabrica (a cabeca, o pe).
        "rest": {key: [round(v, 5) for v in basis @ point * scale]
                 for key, point in rest.items()},
        "action": action.name,
        "fps": fps,
        "frames": end - start + 1,
        "sourceLegLengthMeters": round(leg, 5),
        "scaleToTarget": round(scale, 5),
        "positions": positions,
    }
    args.output.write_text(json.dumps(capture) + "\n", encoding="utf-8")
    print(f"capturado: {args.input.name}, {capture['frames']} quadros a "
          f"{fps:g} fps, escala {scale:.3f}")


main()
