#!/usr/bin/env python3
"""Converte uma ação de um rig humanoide conhecido para AnimationClip3D.

O arquivo pode ser FBX ou glTF/GLB e pode conter várias ações. O preset do rig
define apenas nomes de ossos; o retarget continua sendo geométrico e limitado
pelo perfil físico do personagem.

Executar pelo Blender:
  blender --background --factory-startup --python-exit-code 1 \
    --python tools/import_humanoid_animation.py -- library.glb \
    --source-rig kaykit --action Running_A \
    --profile assets/characters/football_player/FootballPlayerV1.ragdoll.json \
    --output assets/animations/clips/run_forward.matteranim.json \
    --id run_forward --name "Correr para frente" --loop
"""

import argparse
import copy
import json
import math
import pathlib
import sys

import bpy
from mathutils import Matrix, Quaternion, Vector


SOURCE_RIGS = {
    "mixamo": {
        "Pelvis": "Hips", "Abdomen": "Spine", "Chest": "Spine1",
        "UpperChest": "Spine2", "Neck": "Neck", "Head": "Head",
        "LeftUpperArm": "LeftArm", "LeftForearm": "LeftForeArm",
        "LeftHand": "LeftHand", "RightUpperArm": "RightArm",
        "RightForearm": "RightForeArm", "RightHand": "RightHand",
        "LeftThigh": "LeftUpLeg", "LeftShin": "LeftLeg",
        "LeftFoot": "LeftFoot", "RightThigh": "RightUpLeg",
        "RightShin": "RightLeg", "RightFoot": "RightFoot",
    },
    "kaykit": {
        "Pelvis": "hips", "Abdomen": "spine", "Chest": "chest",
        "UpperChest": "chest", "Neck": "head", "Head": "head",
        "LeftUpperArm": "upperarm.l", "LeftForearm": "lowerarm.l",
        "LeftHand": "hand.l", "RightUpperArm": "upperarm.r",
        "RightForearm": "lowerarm.r", "RightHand": "hand.r",
        "LeftThigh": "upperleg.l", "LeftShin": "lowerleg.l",
        "LeftFoot": "foot.l", "RightThigh": "upperleg.r",
        "RightShin": "lowerleg.r", "RightFoot": "foot.r",
    },
    "unreal": {
        "Pelvis": "pelvis", "Abdomen": "spine_01", "Chest": "spine_02",
        "UpperChest": "spine_03", "Neck": "neck_01", "Head": "Head",
        "LeftUpperArm": "upperarm_l", "LeftForearm": "lowerarm_l",
        "LeftHand": "hand_l", "RightUpperArm": "upperarm_r",
        "RightForearm": "lowerarm_r", "RightHand": "hand_r",
        "LeftThigh": "thigh_l", "LeftShin": "calf_l",
        "LeftFoot": "foot_l", "RightThigh": "thigh_r",
        "RightShin": "calf_r", "RightFoot": "foot_r",
    },
}

# Configurados no início de main. Permanecem globais para manter as rotinas
# matemáticas independentes do parser de linha de comando.
MIXAMO_TO_RAGDOLL = SOURCE_RIGS["mixamo"]

# Juntas cuja flexão precisa ser obtida da geometria dos ossos, não do roll
# interno do FBX. Mixamo e HumanAdultV1 usam bases locais diferentes nesses
# hinges; copiar o quaternion colocaria a dobra no eixo errado.
HINGE_CHAINS = {
    "LeftForearm": ("LeftArm", "LeftForeArm", "LeftHand"),
    "RightForearm": ("RightArm", "RightForeArm", "RightHand"),
    "LeftShin": ("LeftUpLeg", "LeftLeg", "LeftFoot"),
    "RightShin": ("RightUpLeg", "RightLeg", "RightFoot"),
}

LIMB_CHAINS = (
    ("LeftUpperArm", "LeftForearm", "LeftHand",
     ("LeftArm", "LeftForeArm", "LeftHand")),
    ("RightUpperArm", "RightForearm", "RightHand",
     ("RightArm", "RightForeArm", "RightHand")),
    ("LeftThigh", "LeftShin", "LeftFoot",
     ("LeftUpLeg", "LeftLeg", "LeftFoot")),
    ("RightThigh", "RightShin", "RightFoot",
     ("RightUpLeg", "RightLeg", "RightFoot")),
)

RETARGET_DIRECTION_EDGES = (
    ("Abdomen", "Chest"),
    ("Chest", "UpperChest"),
    ("UpperChest", "Neck"),
    ("Neck", "Head"),
    ("LeftUpperArm", "LeftForearm"),
    ("LeftForearm", "LeftHand"),
    ("RightUpperArm", "RightForearm"),
    ("RightForearm", "RightHand"),
    ("LeftThigh", "LeftShin"),
    ("LeftShin", "LeftFoot"),
    ("RightThigh", "RightShin"),
    ("RightShin", "RightFoot"),
)


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=pathlib.Path)
    parser.add_argument("--source-rig", choices=tuple(SOURCE_RIGS),
                        default="mixamo")
    parser.add_argument("--action",
                        help="nome exato da ação quando o arquivo contém várias")
    parser.add_argument("--start-frame", type=int,
                        help="primeiro quadro inclusivo da ação")
    parser.add_argument("--end-frame", type=int,
                        help="último quadro inclusivo da ação")
    parser.add_argument("--profile", required=True, type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--id", default="imported_animation")
    parser.add_argument("--name", default="Animação importada")
    parser.add_argument("--loop", action="store_true",
                        help="exige fechamento e marca o clipe como cíclico")
    parser.add_argument("--loop-open", action="store_true",
                        help="o último quadro precede o primeiro; fecha o ciclo no export")
    parser.add_argument("--dynamic-projection", action="store_true",
                        help="aceita projeção limitada de poses dinâmicas extremas")
    parser.add_argument("--recovery", action="store_true",
                        help="ação de recuperação: tolera a aproximação física de poses no solo")
    parser.add_argument("--root-motion", choices=("in-place", "preserve"),
                        default="in-place")
    parser.add_argument("--nominal-speed", type=float,
                        help=("velocidade autoral para um clipe in-place; "
                              "quando omitida, usa o deslocamento horizontal "
                              "da raiz dividido pela duração"))
    parser.add_argument("--inspect-only", action="store_true")
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])


def stripped_bone_name(name):
    return name.rsplit(":", 1)[-1]


def vector_json(value):
    return [round(float(component), 7) for component in value]


def quaternion_json(value):
    # Blender usa w,x,y,z; a MatterEngine usa x,y,z,w.
    value.normalize()
    return [
        round(float(value.x), 8),
        round(float(value.y), 8),
        round(float(value.z), 8),
        round(float(value.w), 8),
    ]


def matter_quaternion(value):
    if value is None:
        return Quaternion()
    return Quaternion((value[3], value[0], value[1], value[2])).normalized()


def exponential_quaternion(rotation_vector):
    angle = rotation_vector.length
    if angle < 0.000001:
        return Quaternion()
    return Quaternion(rotation_vector / angle, angle)


def quaternion_rotation_vector(rotation):
    rotation = rotation.normalized()
    if rotation.w < 0.0:
        rotation.negate()
    imaginary = Vector((rotation.x, rotation.y, rotation.z))
    length = imaginary.length
    if length < 0.000001:
        return Vector((0.0, 0.0, 0.0))
    angle = 2.0 * math.atan2(length, max(0.0, min(1.0, rotation.w)))
    return imaginary * (angle / length)


def joint_limits(link):
    by_name = {axis["axis"]: axis for axis in link["joint"]["axes"]}
    result = []
    for axis_name in ("twist", "swing1", "swing2"):
        axis = by_name.get(axis_name)
        result.append((math.radians(axis["minimumDegrees"]),
                       math.radians(axis["maximumDegrees"]))
                      if axis is not None else (0.0, 0.0))
    return result


def clamp_joint_coordinates(coordinates, limits):
    return Vector(tuple(
        max(limits[index][0], min(limits[index][1], coordinates[index]))
        for index in range(3)
    ))


def suppress_isolated_joint_spikes(samples, loops):
    """Remove um salto isolado do solver sem amortecer o movimento real.

    O retarget por geometria pode encontrar um mínimo local diferente em um
    único quadro quando a cadeia passa muito perto de estendida. Isso aparece
    como uma tremida, e não como movimento autoral: os dois vizinhos continuam
    coerentes. Corrigimos somente esse outlier temporal, por interpolação dos
    vizinhos; transições contínuas e inversões reais permanecem intactas.
    """
    if len(samples) < 3:
        return 0
    threshold = math.radians(12.0)
    source = [sample.copy() for sample in samples]
    corrected = 0
    first = 0 if loops else 1
    last = len(samples) if loops else len(samples) - 1
    for index in range(first, last):
        previous = source[(index - 1) % len(source)]
        following = source[(index + 1) % len(source)]
        midpoint = (previous + following) * 0.5
        # Um pico precisa destoar fortemente da tendência local e os dois
        # lados precisam apontar de volta para ela. Assim não aplainamos uma
        # aceleração legítima de uma passada ou de uma ação rápida.
        toward_previous = samples[index] - previous
        toward_following = following - samples[index]
        if ((samples[index] - midpoint).length > threshold
                and toward_previous.dot(toward_following) < 0.0):
            samples[index] = midpoint
            corrected += 1
    return corrected


def hinge_flexion_radians(pose_bones, chain):
    proximal, joint, distal = (pose_bones[name] for name in chain)
    first = (joint.head - proximal.head).normalized()
    second = (distal.head - joint.head).normalized()
    return math.acos(max(-1.0, min(1.0, first.dot(second))))


def vector_pair_rotation(local_first, local_second,
                         world_first, world_second):
    """Rotação que alinha duas direções e, portanto, o plano de flexão."""
    local_primary = local_first.normalized()
    local_secondary = (local_second
                       - local_primary * local_second.dot(local_primary))
    world_primary = world_first.normalized()
    world_secondary = (world_second
                       - world_primary * world_second.dot(world_primary))
    if local_secondary.length < 0.000001 \
            or world_secondary.length < 0.000001:
        # Pose parada ou membro totalmente estendido: existe uma direção
        # anatômica confiável, mas não existe plano de dobra mensurável.
        # O solver de dois vetores ficaria subdeterminado e pode alternar a
        # torção entre quadros. O chamador conserva o delta local já extraído
        # do FBX para a cadeia, cuja torção é contínua nesse caso.
        return None
    local_secondary.normalize()
    world_secondary.normalize()
    local_normal = local_primary.cross(local_secondary).normalized()
    world_normal = world_primary.cross(world_secondary).normalized()
    local_basis = Matrix((
        local_primary, local_secondary, local_normal)).transposed()
    world_basis = Matrix((
        world_primary, world_secondary, world_normal)).transposed()
    return (world_basis @ local_basis.transposed()).to_quaternion().normalized()


def solve_constrained_limb_joint(parent_world_orientation,
                                 parent_frame, child_frame,
                                 upper_direction_local,
                                 bent_lower_direction_local,
                                 source_upper_direction,
                                 source_lower_direction,
                                 limits, desired_upper_orientation):
    """IK angular limitado para uma cadeia de dois segmentos."""
    desired_local = (parent_world_orientation.inverted()
                     @ desired_upper_orientation).normalized()
    desired_joint = (parent_frame.inverted()
                     @ desired_local @ child_frame).normalized()
    coordinates = clamp_joint_coordinates(
        quaternion_rotation_vector(desired_joint), limits)
    source_upper_direction = source_upper_direction.normalized()
    source_lower_direction = source_lower_direction.normalized()

    def cost(candidate):
        orientation = (parent_world_orientation @ parent_frame
                       @ exponential_quaternion(candidate)
                       @ child_frame.inverted()).normalized()
        upper = (orientation @ upper_direction_local).normalized()
        lower = (orientation @ bent_lower_direction_local).normalized()
        upper_error = math.acos(max(-1.0, min(1.0,
                                      upper.dot(source_upper_direction))))
        lower_error = math.acos(max(-1.0, min(1.0,
                                      lower.dot(source_lower_direction))))
        return upper_error * upper_error + lower_error * lower_error

    best_cost = cost(coordinates)
    for step_degrees in (24.0, 12.0, 6.0, 3.0, 1.5, 0.75, 0.25):
        step = math.radians(step_degrees)
        improved = True
        while improved:
            improved = False
            for axis_index in range(3):
                for direction in (-1.0, 1.0):
                    candidate = coordinates.copy()
                    candidate[axis_index] += direction * step
                    candidate = clamp_joint_coordinates(candidate, limits)
                    candidate_cost = cost(candidate)
                    if candidate_cost + 0.00000001 < best_cost:
                        coordinates = candidate
                        best_cost = candidate_cost
                        improved = True
    orientation = (parent_world_orientation @ parent_frame
                   @ exponential_quaternion(coordinates)
                   @ child_frame.inverted()).normalized()
    return coordinates, orientation


def main():
    global MIXAMO_TO_RAGDOLL, HINGE_CHAINS, LIMB_CHAINS
    args = arguments()
    if args.loop_open:
        args.loop = True
    MIXAMO_TO_RAGDOLL = SOURCE_RIGS[args.source_rig]
    HINGE_CHAINS = {
        "LeftForearm": tuple(MIXAMO_TO_RAGDOLL[target] for target in
                             ("LeftUpperArm", "LeftForearm", "LeftHand")),
        "RightForearm": tuple(MIXAMO_TO_RAGDOLL[target] for target in
                              ("RightUpperArm", "RightForearm", "RightHand")),
        "LeftShin": tuple(MIXAMO_TO_RAGDOLL[target] for target in
                          ("LeftThigh", "LeftShin", "LeftFoot")),
        "RightShin": tuple(MIXAMO_TO_RAGDOLL[target] for target in
                           ("RightThigh", "RightShin", "RightFoot")),
    }
    LIMB_CHAINS = tuple(
        (upper, lower, distal,
         tuple(MIXAMO_TO_RAGDOLL[target]
               for target in (upper, lower, distal)))
        for upper, lower, distal in (
            ("LeftUpperArm", "LeftForearm", "LeftHand"),
            ("RightUpperArm", "RightForearm", "RightHand"),
            ("LeftThigh", "LeftShin", "LeftFoot"),
            ("RightThigh", "RightShin", "RightFoot"),
        )
    )
    input_path = args.input.resolve()
    profile_path = args.profile.resolve()
    if not input_path.is_file():
        raise RuntimeError(f"Arquivo de animação não encontrado: {input_path}")
    if not profile_path.is_file():
        raise RuntimeError(f"Perfil não encontrado: {profile_path}")

    bpy.ops.wm.read_factory_settings(use_empty=True)
    if input_path.suffix.lower() == ".fbx":
        bpy.ops.import_scene.fbx(filepath=str(input_path))
    elif input_path.suffix.lower() in (".glb", ".gltf"):
        bpy.ops.import_scene.gltf(filepath=str(input_path))
    else:
        raise RuntimeError("Formato de animação não suportado: "
                           + input_path.suffix)
    armatures = [obj for obj in bpy.context.scene.objects
                 if obj.type == "ARMATURE"]
    if len(armatures) != 1:
        raise RuntimeError(
            f"Esperado exatamente um armature, encontrados {len(armatures)}")
    armature = armatures[0]
    bpy.context.view_layer.objects.active = armature

    actions = list(bpy.data.actions)
    if args.action:
        action = next((candidate for candidate in actions
                       if candidate.name == args.action), None)
        if action is None:
            raise RuntimeError("Ação ausente: " + args.action
                               + ". Disponíveis: "
                               + ", ".join(sorted(a.name for a in actions)))
    elif len(actions) == 1:
        action = actions[0]
    elif armature.animation_data is not None \
            and armature.animation_data.action is not None:
        action = armature.animation_data.action
    else:
        raise RuntimeError("Arquivo contém várias ações; informe --action")
    armature.animation_data_create()
    armature.animation_data.action = action

    bones = {stripped_bone_name(bone.name): bone
             for bone in armature.data.bones}
    pose_bones = {stripped_bone_name(bone.name): bone
                  for bone in armature.pose.bones}
    missing = sorted(set(MIXAMO_TO_RAGDOLL.values()) - set(bones))
    if missing:
        raise RuntimeError("Ossos ausentes no preset " + args.source_rig
                           + ": " + ", ".join(missing))

    # Base ortonormal extraída da anatomia, independente dos eixos declarados
    # no FBX: +Y alvo aponta para a esquerda do corpo, +Z para cima e +X para
    # frente. Isso torna o retarget robusto à conversão FBX->Blender.
    source_pelvis = MIXAMO_TO_RAGDOLL["Pelvis"]
    source_head = MIXAMO_TO_RAGDOLL["Head"]
    source_left_thigh = MIXAMO_TO_RAGDOLL["LeftThigh"]
    source_right_thigh = MIXAMO_TO_RAGDOLL["RightThigh"]
    source_left_foot = MIXAMO_TO_RAGDOLL["LeftFoot"]
    hips = bones[source_pelvis].head_local
    up = (bones[source_head].head_local - hips).normalized()
    left = (bones[source_left_thigh].head_local
            - bones[source_right_thigh].head_local).normalized()
    left = (left - up * left.dot(up)).normalized()
    forward = left.cross(up).normalized()
    source_to_target = Matrix((forward, left, up))
    if source_to_target.determinant() < 0.0:
        forward.negate()
        source_to_target = Matrix((forward, left, up))
    conversion = source_to_target.to_4x4()
    conversion_rotation = source_to_target.to_quaternion()

    with profile_path.open("r", encoding="utf-8") as profile_file:
        profile = json.load(profile_file)
    profile_links = profile["links"]
    profile_by_id = {link["id"]: link for link in profile_links}
    parent_by_id = {
        link["id"]: (profile_links[link["parent"]]["id"]
                     if link["parent"] >= 0 else None)
        for link in profile_links
    }
    target_model_rotations = {
        link["id"]: matter_quaternion(link.get("orientation"))
        for link in profile_links
    }

    source_leg_length = abs(
        (bones[source_left_foot].head_local - hips).dot(up))
    target_leg_length = abs(profile_by_id["LeftFoot"]["position"][2])
    meters_per_source_unit = target_leg_length / source_leg_length

    action_start = int(math.ceil(action.frame_range[0]))
    action_end = int(math.floor(action.frame_range[1]))
    start_frame = (action_start if args.start_frame is None
                   else max(action_start, args.start_frame))
    end_frame = (action_end if args.end_frame is None
                 else min(action_end, args.end_frame))
    fps = (bpy.context.scene.render.fps
           / bpy.context.scene.render.fps_base)
    if end_frame <= start_frame or fps <= 0.0:
        raise RuntimeError("Intervalo de quadros ou FPS inválido")

    rest_rotations = {
        name: bone.matrix_local.to_quaternion().normalized()
        for name, bone in bones.items()
    }
    frames = list(range(start_frame, end_frame + 1))
    sampled_global_rotations = []
    sampled_root_offsets = []
    sampled_hinge_flexions = []
    sampled_source_landmarks = []
    for frame in frames:
        bpy.context.scene.frame_set(frame)
        bpy.context.view_layer.update()
        frame_rotations = {}
        for target_id, source_id in MIXAMO_TO_RAGDOLL.items():
            animated = pose_bones[source_id].matrix.to_quaternion().normalized()
            source_delta = animated @ rest_rotations[source_id].inverted()
            target_delta = (conversion_rotation @ source_delta
                            @ conversion_rotation.inverted()).normalized()
            frame_rotations[target_id] = target_delta
        frame_hinge_flexions = {
            target_id: hinge_flexion_radians(pose_bones, chain)
            for target_id, chain in HINGE_CHAINS.items()
        }
        sampled_hinge_flexions.append(frame_hinge_flexions)

        # Reconstroi cada braço/perna a partir das direções dos segmentos e
        # do plano de dobra. Isso descarta completamente o bone roll do FBX,
        # que não possui correspondência estável entre rigs diferentes.
        for upper_id, lower_id, distal_id, source_chain in LIMB_CHAINS:
            upper_link = profile_by_id[upper_id]
            lower_link = profile_by_id[lower_id]
            distal_link = profile_by_id[distal_id]
            upper_model = target_model_rotations[upper_id]
            lower_model = target_model_rotations[lower_id]
            joint_frame = matter_quaternion(
                lower_link["joint"]["frameOrientation"])
            parent_frame = (upper_model.inverted()
                            @ joint_frame).normalized()
            child_frame = (lower_model.inverted()
                           @ joint_frame).normalized()
            twist = next(axis for axis in lower_link["joint"]["axes"]
                         if axis["axis"] == "twist")
            flexion = max(math.radians(twist["minimumDegrees"]),
                          min(math.radians(twist["maximumDegrees"]),
                              frame_hinge_flexions[lower_id]))
            lower_local_rotation = (parent_frame
                                    @ exponential_quaternion(
                                        Vector((flexion, 0.0, 0.0)))
                                    @ child_frame.inverted()).normalized()

            upper_anchor = Vector(upper_link["joint"]["anchor"])
            lower_anchor = Vector(lower_link["joint"]["anchor"])
            distal_anchor = Vector(distal_link["joint"]["anchor"])
            target_upper_direction = upper_model.inverted() @ (
                lower_anchor - upper_anchor)
            target_lower_direction = lower_local_rotation @ (
                lower_model.inverted() @ (distal_anchor - lower_anchor))
            source_upper, source_lower, source_distal = (
                pose_bones[name] for name in source_chain)
            source_upper_direction = source_to_target @ (
                source_lower.head - source_upper.head)
            source_lower_direction = source_to_target @ (
                source_distal.head - source_lower.head)
            desired_upper_orientation = vector_pair_rotation(
                target_upper_direction, target_lower_direction,
                source_upper_direction, source_lower_direction)
            if desired_upper_orientation is None:
                continue
            upper_parent = profile_links[upper_link["parent"]]
            upper_parent_model = target_model_rotations[upper_parent["id"]]
            upper_joint_frame = matter_quaternion(
                upper_link["joint"]["frameOrientation"])
            upper_parent_frame = (upper_parent_model.inverted()
                                  @ upper_joint_frame).normalized()
            upper_child_frame = (upper_model.inverted()
                                 @ upper_joint_frame).normalized()
            parent_world_orientation = (
                upper_parent_model
                @ frame_rotations[upper_parent["id"]]).normalized()
            _, constrained_upper_orientation = solve_constrained_limb_joint(
                parent_world_orientation,
                upper_parent_frame, upper_child_frame,
                target_upper_direction, target_lower_direction,
                source_upper_direction, source_lower_direction,
                joint_limits(upper_link), desired_upper_orientation)
            desired_lower_orientation = (constrained_upper_orientation
                                         @ lower_local_rotation).normalized()
            frame_rotations[upper_id] = (constrained_upper_orientation
                                         @ upper_model.inverted()).normalized()
            frame_rotations[lower_id] = (desired_lower_orientation
                                         @ lower_model.inverted()).normalized()

        sampled_global_rotations.append(frame_rotations)
        source_origin = pose_bones[source_pelvis].head.copy()
        sampled_source_landmarks.append({
            target_id: (source_to_target
                        @ (pose_bones[source_id].head - source_origin))
                       * meters_per_source_unit
            for target_id, source_id in MIXAMO_TO_RAGDOLL.items()
        })

        source_root_offset = pose_bones[source_pelvis].matrix.translation - hips
        target_root_offset = source_to_target @ source_root_offset
        sampled_root_offsets.append(target_root_offset * meters_per_source_unit)

    # O visualizador e o futuro controlador físico são donos do deslocamento
    # global. Remove a deriva linear completa entre as pontas do ciclo. Além
    # do avanço do FBX, isso elimina qualquer vazamento desse avanço no eixo
    # vertical causado pela inclinação da pose de referência. A oscilação da
    # pelve dentro da passada permanece intacta.
    first_root = sampled_root_offsets[0].copy()
    last_root = sampled_root_offsets[-1].copy()
    source_displacement = last_root - first_root
    if args.loop_open and len(frames) > 1:
        source_displacement *= len(frames) / (len(frames) - 1)
    cycle_drift = (source_displacement
                   if args.root_motion == "in-place" else Vector())

    duration = ((len(frames) if args.loop_open else end_frame - start_frame)
                / fps)
    tracks = []
    previous_quaternion = {}
    generated_root_rotations = []
    generated_root_translations = []
    generated_joint_coordinates = {}
    isolated_joint_spike_count = 0
    for target_id in (link["id"] for link in profile_links):
        source_id = MIXAMO_TO_RAGDOLL.get(target_id)
        if source_id is None:
            continue
        parent_id = parent_by_id[target_id]
        keyframes = []
        for sample_index, frame in enumerate(frames):
            progress = (sample_index / len(frames) if args.loop_open
                        else sample_index / (len(frames) - 1))
            global_rotation = sampled_global_rotations[sample_index][target_id]
            if parent_id is None:
                root_rotation = global_rotation
                previous = previous_quaternion.get(target_id)
                if previous is not None and previous.dot(root_rotation) < 0.0:
                    root_rotation.negate()
                previous_quaternion[target_id] = root_rotation.copy()
                translation = (sampled_root_offsets[sample_index]
                               - cycle_drift * progress)
                generated_root_rotations.append(root_rotation.copy())
                generated_root_translations.append(translation.copy())
                keyframes.append({
                    "timeSeconds": round((frame - start_frame) / fps, 7),
                    "rootTranslationOffsetMeters": vector_json(translation),
                    "rootRotationDelta": quaternion_json(root_rotation),
                })
            else:
                parent_global = sampled_global_rotations[sample_index][parent_id]
                local_rotation = (parent_global.inverted()
                                  @ global_rotation).normalized()
                # O clipe canônico é um alvo do ragdoll, não uma cópia livre
                # do esqueleto Mixamo. Converte o delta para as coordenadas
                # exponenciais usadas pela articulation PhysX, zera eixos
                # bloqueados e limita cada DOF conforme o perfil físico.
                link = profile_by_id[target_id]
                parent_model = target_model_rotations[parent_id]
                child_model = target_model_rotations[target_id]
                joint_frame = matter_quaternion(
                    link["joint"]["frameOrientation"])
                parent_frame = (parent_model.inverted()
                                @ joint_frame).normalized()
                child_frame = (child_model.inverted()
                               @ joint_frame).normalized()
                bind_local = (parent_model.inverted()
                              @ child_model).normalized()
                desired_local = (bind_local @ local_rotation).normalized()
                joint_rotation = (parent_frame.inverted()
                                  @ desired_local @ child_frame).normalized()
                if joint_rotation.w < 0.0:
                    joint_rotation.negate()
                coordinates = joint_rotation.to_exponential_map()
                limits = {axis["axis"]: axis
                          for axis in link["joint"]["axes"]}
                for coordinate_index, axis_name in enumerate(
                        ("twist", "swing1", "swing2")):
                    axis = limits.get(axis_name)
                    if axis is None:
                        coordinates[coordinate_index] = 0.0
                    else:
                        minimum = math.radians(axis["minimumDegrees"])
                        maximum = math.radians(axis["maximumDegrees"])
                        coordinates[coordinate_index] = max(
                            minimum, min(maximum,
                                         coordinates[coordinate_index]))
                if target_id in HINGE_CHAINS:
                    twist = limits["twist"]
                    coordinates.x = max(
                        math.radians(twist["minimumDegrees"]),
                        min(math.radians(twist["maximumDegrees"]),
                            sampled_hinge_flexions[sample_index][target_id]))
                generated_joint_coordinates.setdefault(target_id, []).append(
                    coordinates.copy())
                keyframes.append({
                    "timeSeconds": round((frame - start_frame) / fps, 7),
                    "jointPositionRadians": vector_json(coordinates),
                })
        if parent_id is not None:
            samples = generated_joint_coordinates[target_id]
            isolated_joint_spike_count += suppress_isolated_joint_spikes(
                samples, args.loop)
            for keyframe, coordinates in zip(keyframes, samples):
                keyframe["jointPositionRadians"] = vector_json(coordinates)
        if args.loop_open:
            closing = copy.deepcopy(keyframes[0])
            closing["timeSeconds"] = round(duration, 7)
            keyframes.append(closing)
        tracks.append({
            "targetLinkId": target_id,
            "space": "root" if parent_id is None else "joint",
            "keyframes": keyframes,
        })

    measured_edges = tuple(edge for edge in RETARGET_DIRECTION_EDGES
        if MIXAMO_TO_RAGDOLL[edge[0]] != MIXAMO_TO_RAGDOLL[edge[1]])
    direction_errors = {edge: [] for edge in measured_edges}
    retargeted_foot_positions = {"leftFoot": [], "rightFoot": []}
    for sample_index in range(len(frames)):
        target_positions = {}
        target_orientations = {}
        target_landmarks = {}
        for link in profile_links:
            target_id = link["id"]
            model_rotation = target_model_rotations[target_id]
            if link["parent"] < 0:
                target_positions[target_id] = (
                    Vector(link["position"])
                    + generated_root_translations[sample_index])
                target_orientations[target_id] = (
                    model_rotation
                    @ generated_root_rotations[sample_index]).normalized()
                target_landmarks[target_id] = target_positions[target_id]
                continue
            parent = profile_links[link["parent"]]
            parent_id = parent["id"]
            parent_model = target_model_rotations[parent_id]
            joint_frame = matter_quaternion(
                link["joint"]["frameOrientation"])
            parent_frame = (parent_model.inverted()
                            @ joint_frame).normalized()
            child_frame = (model_rotation.inverted()
                           @ joint_frame).normalized()
            local_rotation = (parent_frame
                              @ exponential_quaternion(
                                  generated_joint_coordinates[target_id][
                                      sample_index])
                              @ child_frame.inverted()).normalized()
            target_orientations[target_id] = (
                target_orientations[parent_id]
                @ local_rotation).normalized()
            anchor = Vector(link["joint"]["anchor"])
            parent_anchor_local = parent_model.inverted() @ (
                anchor - Vector(parent["position"]))
            child_anchor_local = model_rotation.inverted() @ (
                anchor - Vector(link["position"]))
            world_anchor = (target_positions[parent_id]
                            + target_orientations[parent_id]
                              @ parent_anchor_local)
            target_positions[target_id] = (
                world_anchor
                - target_orientations[target_id] @ child_anchor_local)
            target_landmarks[target_id] = world_anchor

        retargeted_foot_positions["leftFoot"].append(
            target_positions["LeftFoot"].copy())
        retargeted_foot_positions["rightFoot"].append(
            target_positions["RightFoot"].copy())

        source_landmarks = sampled_source_landmarks[sample_index]
        for edge in measured_edges:
            first, second = edge
            source_direction = (
                source_landmarks[second] - source_landmarks[first]).normalized()
            target_direction = (
                target_landmarks[second] - target_landmarks[first]).normalized()
            cosine = max(-1.0, min(1.0,
                         source_direction.dot(target_direction)))
            direction_errors[edge].append(math.degrees(math.acos(cosine)))

    all_direction_errors = [value for values in direction_errors.values()
                            for value in values]
    limb_direction_errors = [
        value for edge, values in direction_errors.items()
        if "Arm" in edge[0] or "Forearm" in edge[0]
        or "Thigh" in edge[0] or "Shin" in edge[0]
        for value in values
    ]
    direction_rms = math.sqrt(sum(value * value
                              for value in all_direction_errors)
                              / len(all_direction_errors))
    direction_maximum = max(all_direction_errors)
    limb_direction_maximum = max(limb_direction_errors)
    maximum_joint_step = 0.0
    maximum_joint_cycle_delta = 0.0
    limit_hit_count = 0
    for target_id, samples in generated_joint_coordinates.items():
        limits = joint_limits(profile_by_id[target_id])
        for sample_index, coordinates in enumerate(samples):
            previous = (samples[sample_index - 1]
                        if sample_index > 0 or args.loop else None)
            for axis_index, (minimum, maximum) in enumerate(limits):
                if previous is not None:
                    maximum_joint_step = max(maximum_joint_step,
                        abs(coordinates[axis_index] - previous[axis_index]))
                if maximum - minimum > 0.0001 and (
                        abs(coordinates[axis_index] - minimum) < 0.0002
                        or abs(coordinates[axis_index] - maximum) < 0.0002):
                    limit_hit_count += 1
        if args.loop:
            maximum_joint_cycle_delta = max(maximum_joint_cycle_delta,
                max(abs(samples[-1][axis] - samples[0][axis])
                    for axis in range(3)))
    root_rotation_dot = abs(generated_root_rotations[0].dot(
        generated_root_rotations[-1]))
    root_cycle_rotation = math.degrees(
        2.0 * math.acos(max(-1.0, min(1.0, root_rotation_dot))))
    root_cycle_translation = (
        generated_root_translations[-1]
        - generated_root_translations[0]).length
    if args.loop_open:
        # O arquivo não duplica o primeiro quadro. O export acima acrescenta
        # esse quadro exatamente em duration, portanto a costura canônica é
        # exata mesmo que a última amostra original ainda esteja em trânsito.
        maximum_joint_cycle_delta = 0.0
        root_cycle_rotation = 0.0
        root_cycle_translation = 0.0

    quality_limits = {
        "directionRmsDegrees": 6.0,
        "directionMaxDegrees": 15.0,
        "limbDirectionMaxDegrees": 8.0,
        "maximumJointStepDegrees": 45.0,
    }
    if args.recovery:
        # Levantar do chão exige grandes flexões do tronco e apoio nos membros.
        # Mantemos limites articulares e RMS estritos, mas aceitamos até 20° de
        # erro local quando a pose Mixamo excede a amplitude física do ragdoll.
        quality_limits.update({
            "directionMaxDegrees": 20.0,
            "limbDirectionMaxDegrees": 20.0,
        })
    if args.dynamic_projection:
        # Corrida, sprint e salto podem exceder a amplitude da articulação
        # física no auge do swing. A projeção continua limitada e registrada;
        # este preset não afrouxa continuidade nem permite valores ilegais.
        quality_limits.update({
            "directionRmsDegrees": 8.0,
            "directionMaxDegrees": 35.0,
            "limbDirectionMaxDegrees": 35.0,
            "maximumJointStepDegrees": 60.0,
        })
    quality_failures = []
    measured_quality = {
        "directionRmsDegrees": direction_rms,
        "directionMaxDegrees": direction_maximum,
        "limbDirectionMaxDegrees": limb_direction_maximum,
        "jointCycleDeltaDegrees": math.degrees(maximum_joint_cycle_delta),
        "rootCycleRotationDegrees": root_cycle_rotation,
        "rootCycleTranslationMeters": root_cycle_translation,
        "maximumJointStepDegrees": math.degrees(maximum_joint_step),
    }
    if args.loop:
        quality_limits.update({
            "jointCycleDeltaDegrees": 0.5,
            "rootCycleRotationDegrees": 0.5,
            "rootCycleTranslationMeters": 0.001,
        })
    for metric, limit in quality_limits.items():
        if measured_quality[metric] > limit:
            quality_failures.append(
                f"{metric}={measured_quality[metric]:.4f} > {limit:.4f}")

    print("MATTERENGINE_HUMANOID_IMPORT")
    print(f"  source rig: {args.source_rig}")
    print(f"  armature: {armature.name}")
    print(f"  action: {action.name}")
    print(f"  frames: {start_frame}..{end_frame} ({len(frames)})")
    print(f"  fps: {fps:.3f}; duration: {duration:.3f} s")
    print(f"  bones: {len(bones)}; mapped links: {len(tracks)}")
    print(f"  source leg: {source_leg_length:.5f}; scale: "
          f"{meters_per_source_unit:.7f} m/unit")
    print("  source basis:")
    print(f"    forward={tuple(round(v, 5) for v in forward)}")
    print(f"    left={tuple(round(v, 5) for v in left)}")
    print(f"    up={tuple(round(v, 5) for v in up)}")
    print("  root offset first/last (m):")
    print(f"    first={tuple(round(v, 5) for v in first_root)}")
    print(f"    last={tuple(round(v, 5) for v in last_root)}")
    print("  semantic hinge flexion ranges (degrees):")
    for target_id in HINGE_CHAINS:
        values = [math.degrees(sample[target_id])
                  for sample in sampled_hinge_flexions]
        print(f"    {target_id}: {min(values):.2f}..{max(values):.2f}")
    print("  retarget direction error ranges (degrees):")
    for edge, values in direction_errors.items():
        print(f"    {edge[0]}->{edge[1]}: "
              f"{min(values):.2f}..{max(values):.2f}")
    print(f"  direction RMS/max: {direction_rms:.2f}/"
          f"{direction_maximum:.2f} degrees; limb max: "
          f"{limb_direction_maximum:.2f}")
    print(f"  max joint step/cycle delta: "
          f"{math.degrees(maximum_joint_step):.2f}/"
          f"{math.degrees(maximum_joint_cycle_delta):.4f} degrees")
    print(f"  root cycle rotation/translation: "
          f"{root_cycle_rotation:.4f} degrees/"
          f"{root_cycle_translation:.7f} m")
    print(f"  constrained samples at a limit: {limit_hit_count}")
    print(f"  isolated joint spikes corrected: {isolated_joint_spike_count}")
    print("  joint coordinate ranges / limits (degrees):")
    for target_id, samples in generated_joint_coordinates.items():
        limits = joint_limits(profile_by_id[target_id])
        ranges = []
        for axis_index, (minimum, maximum) in enumerate(limits):
            values = [math.degrees(sample[axis_index]) for sample in samples]
            ranges.append(
                f"{min(values):.1f}..{max(values):.1f}/"
                f"{math.degrees(minimum):.1f}..{math.degrees(maximum):.1f}")
        print(f"    {target_id}: " + ", ".join(ranges))

    if quality_failures:
        raise RuntimeError("Retarget reprovado nos gates de qualidade: "
                           + "; ".join(quality_failures))

    if args.inspect_only:
        return
    if args.output is None:
        raise RuntimeError("--output é obrigatório sem --inspect-only")
    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    def contact_track(positions):
        heights = [position.z for position in positions]
        lowest = min(heights)
        highest = max(heights)
        full_contact = lowest + min(0.025, (highest - lowest) * 0.18)
        released = lowest + min(0.075, max(0.035,
                                           (highest - lowest) * 0.42))
        interval = max(0.005, released - full_contact)
        values = []
        for sample_index, position in enumerate(positions):
            amount = max(0.0, min(1.0,
                (position.z - full_contact) / interval))
            amount = amount * amount * (3.0 - 2.0 * amount)
            values.append({
                "timeSeconds": round((frames[sample_index] - start_frame)
                                     / fps, 7),
                "value": round(1.0 - amount, 6),
            })
        if args.loop_open:
            values.append({"timeSeconds": round(duration, 7),
                           "value": values[0]["value"]})
        return values

    measured_speed = (math.hypot(source_displacement.x,
                                 source_displacement.y) / duration)
    if measured_speed < 0.01:
        measured_speed = 0.0
    nominal_speed = (max(0.0, args.nominal_speed)
                     if args.nominal_speed is not None
                     else measured_speed)
    document = {
        "schema": "matter-ragdoll-animation-1",
        "id": args.id,
        "displayName": args.name,
        "targetRigId": profile["id"],
        "sourceAssetPath": input_path.name + "#" + action.name,
        "durationSeconds": round(duration, 7),
        "sourceSampleRateHz": round(fps, 4),
        "loops": args.loop,
        "sourceRootDisplacementMeters": vector_json(source_displacement),
        "nominalSpeedMetersPerSecond": round(nominal_speed, 5),
        "contacts": {
            name: contact_track(positions)
            for name, positions in retargeted_foot_positions.items()
        },
        "rootMotionPolicy": args.root_motion,
        "retargetReport": {
            "passed": True,
            **{name: round(value, 6)
               for name, value in measured_quality.items()},
            "limitHitCount": limit_hit_count,
            "thresholds": quality_limits,
            "directionEdges": [
                {
                    "from": edge[0],
                    "to": edge[1],
                    "rmsDegrees": round(math.sqrt(sum(value * value
                        for value in values) / len(values)), 6),
                    "maxDegrees": round(max(values), 6),
                }
                for edge, values in direction_errors.items()
            ],
        },
        "tracks": tracks,
    }
    with output_path.open("w", encoding="utf-8", newline="\n") as output_file:
        json.dump(document, output_file, ensure_ascii=False, indent=2)
        output_file.write("\n")
    print(f"  output: {output_path}")


if __name__ == "__main__":
    main()
