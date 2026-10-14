#!/usr/bin/env python3
"""O inventario das fontes de animacao (FBX do Mixamo): duracao, quanto o
quadril anda (se veio "no lugar") e quanto o corpo gira, de cada arquivo.

Executar pelo Blender (o do sistema precisa do Python dele):
  env PYTHONHOME=/usr blender --background --factory-startup --python-use-system-env \
    --python tools/animation/inventario.py -- assets/animations/source/mixamo \
    --saida assets/animations/source/INVENTARIO.md
"""

import argparse
import math
import pathlib
import sys

import bpy
from mathutils import Vector


def bone(armature, name):
    # Cada download do Mixamo pode vir com outro prefixo (mixamorig:,
    # mixamorig5:...): vale o nome depois dos dois pontos.
    for candidate in armature.pose.bones:
        if candidate.name.split(":")[-1] == name:
            return candidate
    return None


def measure(path: pathlib.Path) -> dict:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=str(path))
    armature = next((o for o in bpy.context.scene.objects if o.type == "ARMATURE"), None)
    if armature is None or armature.animation_data is None or armature.animation_data.action is None:
        return {"erro": "sem esqueleto animado"}
    action = armature.animation_data.action
    first, last = (int(round(v)) for v in action.frame_range)
    fps = bpy.context.scene.render.fps / bpy.context.scene.render.fps_base
    hips, left, right = bone(armature, "Hips"), bone(armature, "LeftUpLeg"), bone(armature, "RightUpLeg")
    if hips is None or left is None or right is None:
        return {"erro": "sem os ossos do Mixamo"}
    scene = bpy.context.scene
    world = armature.matrix_world
    positions, headings = [], []
    for frame in range(first, last + 1):
        scene.frame_set(frame)
        p = world @ hips.head
        across = (world @ left.head) - (world @ right.head)
        positions.append(Vector((p.x, p.y)))
        # O rumo do corpo: perpendicular a linha dos quadris, no chao.
        headings.append(math.atan2(across.x, -across.y))
    turned, previous = 0.0, headings[0]
    peak = 0.0
    for h in headings[1:]:
        step = math.atan2(math.sin(h - previous), math.cos(h - previous))
        turned += step
        peak = max(peak, abs(turned))
        previous = h
    travel = (positions[-1] - positions[0]).length
    farthest = max((p - positions[0]).length for p in positions)
    return {
        "quadros": last - first + 1,
        "segundos": (last - first) / fps,
        "anda": travel,
        "max": farthest,
        "giro": math.degrees(turned),
        "giro_max": math.degrees(peak),
    }


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("pasta", type=pathlib.Path)
    parser.add_argument("--saida", type=pathlib.Path, required=True)
    args = parser.parse_args(argv)
    rows = []
    for path in sorted(args.pasta.rglob("*.fbx")):
        try:
            data = measure(path)
        except Exception as error:  # um arquivo ruim nao para o inventario
            data = {"erro": str(error)[:80]}
        rows.append((path.relative_to(args.pasta), data))
        print(path.name, data, flush=True)
    lines = [
        "# Inventário das fontes de animação",
        "",
        "Gerado por `tools/animation/inventario.py` (não editar à mão; rodar de novo",
        "quando chegarem arquivos). **Anda:** quanto o quadril sai do lugar do primeiro",
        "ao último quadro (m); perto de 0 com \"máx\" pequeno = veio no lugar (In Place).",
        "**Giro:** quanto o corpo virou no fim, e o máximo no meio (graus).",
        "",
        "| Arquivo | s | Anda (m) | Máx (m) | Giro (°) | Giro máx (°) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, d in rows:
        if "erro" in d:
            lines.append(f"| `{name}` | — | — | — | — | {d['erro']} |")
        else:
            lines.append(f"| `{name}` | {d['segundos']:.2f} | {d['anda']:.2f} | {d['max']:.2f} | "
                         f"{d['giro']:.0f} | {d['giro_max']:.0f} |")
    args.saida.write_text("\n".join(lines) + "\n")
    print("inventario:", args.saida)
    return 0


if __name__ == "__main__":
    sys.exit(main())
