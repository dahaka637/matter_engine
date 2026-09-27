"""Build the ALS mannequin's ragdoll profile and skin from its source file.

Everything geometric comes from the model itself: each physical link owns a
set of mesh vertices (this model is rigidly skinned, one bone per vertex) and
its collider is fitted to those vertices in the bone's own frame. Nothing is
inherited from a previous character.

Torso pieces are capsules lying on the LATERAL axis. They used to be boxes on
the premise that "that is their shape", and measuring the mesh showed it is
not: each torso piece of this mannequin is a rounded barrel, narrowest at its
top and bottom (the Chest is 0.174 m half-wide at its middle and 0.104 at its
ends). A box of the widest span left corners ~6 cm proud of the body exactly
where the upper arm hangs past the chest, so a relaxed arm could not hang
without being shoved out by self-collision - the raised arms seen in game
while the animation viewer (no physics) showed them lowered. A lateral capsule
follows the barrel within a few millimetres. The feet stay boxes: a flat sole
is what they need to stand on.

The arm and leg chains are straightened before anything is measured. The
animation retarget applies a clip's absolute elbow/knee flexion on top of
whatever bend the bind pose already has, so a bent bind shows up as a
constant per-frame direction error (this model ships with 32 degrees of
elbow bend, which cost ~16 degrees of error on every arm segment).

blender --background --factory-startup --disable-autoexec \\
    --python-exit-code 1 --python tools/prepare_als_ragdoll.py -- \\
    --source "ALS Ragdoll.blend" --output assets/characters/als_ragdoll
"""
import argparse
import importlib.util
import json
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

REPO = Path(__file__).resolve().parent.parent
PROFILE_ID = "AlsRagdollV1"
TOTAL_MASS_KG = 80.0

PRIMARY = {
    "pelvis": "Pelvis", "spine_01": "Abdomen", "spine_02": "Chest",
    "spine_03": "UpperChest", "neck_01": "Neck", "head": "Head",
    "upperarm_l": "LeftUpperArm", "lowerarm_l": "LeftForearm",
    "hand_l": "LeftHand", "upperarm_r": "RightUpperArm",
    "lowerarm_r": "RightForearm", "hand_r": "RightHand",
    "thigh_l": "LeftThigh", "calf_l": "LeftShin", "foot_l": "LeftFoot",
    "thigh_r": "RightThigh", "calf_r": "RightShin", "foot_r": "RightFoot",
}

# Clavicles stay with the torso: a shoulder blade should not swing with the
# humerus. Twist bones and fingers fold into the limb they belong to.
AUXILIARY = {
    "clavicle_l": "UpperChest", "clavicle_r": "UpperChest",
    "upperarm_twist_01_l": "LeftUpperArm",
    "upperarm_twist_01_r": "RightUpperArm",
    "lowerarm_twist_01_l": "LeftForearm",
    "lowerarm_twist_01_r": "RightForearm",
    "thigh_twist_01_l": "LeftThigh", "thigh_twist_01_r": "RightThigh",
    "calf_twist_01_l": "LeftShin", "calf_twist_01_r": "RightShin",
    "ball_l": "LeftFoot", "ball_r": "RightFoot",
}
for _side, _hand in (("l", "LeftHand"), ("r", "RightHand")):
    for _finger in ("index", "middle", "pinky", "ring", "thumb"):
        for _part in ("01", "02", "03"):
            AUXILIARY[f"{_finger}_{_part}_{_side}"] = _hand

PARENT_OF = {
    "Pelvis": None, "Abdomen": "Pelvis", "Chest": "Abdomen",
    "UpperChest": "Chest", "Neck": "UpperChest", "Head": "Neck",
    "LeftUpperArm": "UpperChest", "LeftForearm": "LeftUpperArm",
    "LeftHand": "LeftForearm", "RightUpperArm": "UpperChest",
    "RightForearm": "RightUpperArm", "RightHand": "RightForearm",
    "LeftThigh": "Pelvis", "LeftShin": "LeftThigh", "LeftFoot": "LeftShin",
    "RightThigh": "Pelvis", "RightShin": "RightThigh",
    "RightFoot": "RightShin",
}
ORDER = ["Pelvis", "Abdomen", "Chest", "UpperChest", "Neck", "Head",
         "LeftUpperArm", "LeftForearm", "LeftHand",
         "RightUpperArm", "RightForearm", "RightHand",
         "LeftThigh", "LeftShin", "LeftFoot",
         "RightThigh", "RightShin", "RightFoot"]

BOX_LINKS = {"LeftFoot", "RightFoot"}
LATERAL_CAPSULE_LINKS = {"Pelvis", "Abdomen", "Chest", "UpperChest"}

# Contatos falsos da forma aproximada dos colisores, dentro da amplitude normal
# do corpo. Medido: a capsula do abdomen (raio 12,5 cm em torno de um eixo
# lateral) desce ate a altura do quadril; a coxa encosta nela com 45 graus de
# flexao e a penetra 3 cm a 90. Num corpo real a coxa so toca a barriga perto
# de 110 graus. Com o par ativo, toda passada, sprint ou chute bateria numa
# barriga que nao existe.
SELF_COLLISION_IGNORED_PAIRS = [
    {"links": ["Abdomen", side + "Thigh"],
     "reason": "capsula do abdomen desce ate o quadril; contato falso a "
               "partir de 45 graus de flexao do quadril"}
    for side in ("Left", "Right")
]

MASS_FRACTION = {
    "Pelvis": 0.142, "Abdomen": 0.102, "Chest": 0.124, "UpperChest": 0.086,
    "Neck": 0.020, "Head": 0.081,
    "LeftUpperArm": 0.027, "LeftForearm": 0.016, "LeftHand": 0.006,
    "RightUpperArm": 0.027, "RightForearm": 0.016, "RightHand": 0.006,
    "LeftThigh": 0.112, "LeftShin": 0.047, "LeftFoot": 0.0145,
    "RightThigh": 0.112, "RightShin": 0.047, "RightFoot": 0.0145,
}

# stiffness / damping / maximumTorque. Sized against the load each joint
# actually carries while standing, not copied from the previous character:
# an ankle holding a body up needs roughly bodyweight * foot lever, and at
# the old 120 N*m/rad that came out as ~30 degrees of sag which bent the
# whole leg above it. Legs carry the most, then the torso, then the arms.
DRIVE = {
    "Abdomen": (440.0, 49.5, 520.0), "Chest": (440.0, 49.5, 520.0),
    "UpperChest": (440.0, 49.5, 520.0), "Neck": (82.0, 14.0, 62.0),
    "Head": (76.0, 13.0, 58.0),
    "LeftUpperArm": (240.0, 25.5, 300.0),
    "RightUpperArm": (240.0, 25.5, 300.0),
    "LeftForearm": (90.0, 14.0, 100.0), "RightForearm": (90.0, 14.0, 100.0),
    "LeftHand": (32.0, 6.0, 35.0), "RightHand": (32.0, 6.0, 35.0),
    "LeftThigh": (750.0, 71.2, 875.0), "RightThigh": (750.0, 71.2, 875.0),
    "LeftShin": (650.0, 60.1, 750.0), "RightShin": (650.0, 60.1, 750.0),
    "LeftFoot": (360.0, 34.6, 420.0), "RightFoot": (360.0, 34.6, 420.0),
}

# Ranges are measured from the bind pose this profile defines. Spine, hips,
# knees and ankles keep the values the controller was tuned with - their bind
# direction is unchanged (spine up, thigh down, hinge sideways). Shoulders are
# symmetric on purpose: a symmetric cone cannot be silently mirrored wrong.
LIMITS = {
    "Abdomen": [("twist", -25, 25), ("swing1", -35, 45), ("swing2", -20, 20)],
    "Chest": [("twist", -22, 22), ("swing1", -28, 38), ("swing2", -18, 18)],
    "UpperChest": [("twist", -20, 20), ("swing1", -25, 35),
                   ("swing2", -18, 18)],
    "Neck": [("twist", -30, 30), ("swing1", -18, 22), ("swing2", -22, 22)],
    "Head": [("twist", -45, 45), ("swing1", -28, 34), ("swing2", -28, 28)],
    "LeftUpperArm": [("twist", -80, 80), ("swing1", -115, 115),
                     ("swing2", -95, 95)],
    "RightUpperArm": [("twist", -80, 80), ("swing1", -115, 115),
                      ("swing2", -95, 95)],
    "LeftForearm": [("twist", -5, 145), ("swing1", -80, 80)],
    "RightForearm": [("twist", -5, 145), ("swing1", -80, 80)],
    "LeftHand": [("twist", -70, 70), ("swing1", -35, 35)],
    "RightHand": [("twist", -70, 70), ("swing1", -35, 35)],
    "LeftThigh": [("twist", -45, 45), ("swing1", -120, 30),
                  ("swing2", -42, 42)],
    "RightThigh": [("twist", -45, 45), ("swing1", -120, 30),
                   ("swing2", -42, 42)],
    "LeftShin": [("twist", -5, 145)],
    "RightShin": [("twist", -5, 145)],
    "LeftFoot": [("twist", -50, 65), ("swing1", -35, 35)],
    "RightFoot": [("twist", -50, 65), ("swing1", -35, 35)],
}

VERTICAL_FRAME = [0.0, -0.70710678, 0.0, 0.70710678]   # local +X -> world +Z
SIDEWAYS_FRAME = [0.0, 0.0, 0.70710678, 0.70710678]    # local +X -> world +Y

# Elbows and knees bend about a fixed anatomical axis, and it cannot be read
# off the bind pose here: the limbs are deliberately straightened above, so
# there is no bend left to derive an axis from. Both knees hinge about the
# same sideways axis (heels travel backwards together); the elbows mirror,
# because with the arms out along Y each hand swings forward about the
# vertical axis in the opposite sense.
HINGE_FRAME = {
    "LeftShin": SIDEWAYS_FRAME,
    "RightShin": SIDEWAYS_FRAME,
    "LeftForearm": [0.0, 0.70710678, 0.0, 0.70710678],    # +X -> -Z
    "RightForearm": [0.0, -0.70710678, 0.0, 0.70710678],  # +X -> +Z
}


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path,
                        help=".blend holding CharacterMesh0 and the root rig")
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1:])


def frame_from_axis(axis, reference=(0.0, 0.0, 1.0)):
    """Orthonormal basis with local +X on `axis`; `reference` fixes the roll."""
    x = Vector(axis).normalized()
    ref = Vector(reference).normalized()
    if abs(x.dot(ref)) > 0.97:
        ref = Vector((1.0, 0.0, 0.0))
        if abs(x.dot(ref)) > 0.97:
            ref = Vector((0.0, 1.0, 0.0))
    z = (ref - x * ref.dot(x)).normalized()
    y = z.cross(x).normalized()
    return x, y, z


def quaternion_json(x, y, z):
    q = Matrix((x, y, z)).transposed().to_quaternion().normalized()
    return [round(q.x, 8), round(q.y, 8), round(q.z, 8), round(q.w, 8)]


def vector_json(v):
    return [round(float(v[0]), 7), round(float(v[1]), 7), round(float(v[2]), 7)]


def span(values, low_fraction=0.004, high_fraction=0.996):
    """A percentile span, not the extremes: one stray vertex must not inflate
    a whole limb."""
    ordered = sorted(values)
    last = len(ordered) - 1
    low = ordered[min(last, max(0, int(round(low_fraction * last))))]
    high = ordered[min(last, max(0, int(round(high_fraction * last))))]
    return low, high


def rotation_between(source, target):
    return Vector(source).normalized().rotation_difference(
        Vector(target).normalized()).to_matrix().to_4x4()


def pivot_transform(pivot, rotation):
    return (Matrix.Translation(pivot) @ rotation
            @ Matrix.Translation(-Vector(pivot)))


def main():
    args = arguments()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)

    bpy.ops.wm.open_mainfile(filepath=str(args.source.resolve()),
                             use_scripts=False)
    bpy.context.view_layer.update()
    obj = bpy.data.objects["CharacterMesh0"]
    rig = bpy.data.objects["root"]

    # The file ships posed; the exporter rightly refuses to bind anything but
    # the rest pose, and every measurement below is rest data too.
    for pose_bone in rig.pose.bones:
        pose_bone.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()

    for target in (rig, obj):
        bpy.ops.object.select_all(action="DESELECT")
        target.select_set(True)
        bpy.context.view_layer.objects.active = target
        if target.parent:
            bpy.ops.object.parent_clear(type="CLEAR_KEEP_TRANSFORM")
        bpy.ops.object.transform_apply(location=True, rotation=True,
                                       scale=True)

    # The engine expresses every link and skin vertex relative to the pelvis.
    pelvis_origin = rig.data.bones["pelvis"].head_local.copy()
    for target in (rig, obj):
        bpy.ops.object.select_all(action="DESELECT")
        target.select_set(True)
        bpy.context.view_layer.objects.active = target
        target.location = -pelvis_origin
        bpy.ops.object.transform_apply(location=True, rotation=False,
                                       scale=False)
    bpy.context.view_layer.update()
    bone_head = {b.name: b.head_local.copy() for b in rig.data.bones}

    group_link = {}
    for group in obj.vertex_groups:
        if group.name in PRIMARY:
            group_link[group.index] = PRIMARY[group.name]
        elif group.name in AUXILIARY:
            group_link[group.index] = AUXILIARY[group.name]
        else:
            raise RuntimeError(f"Vertex group without a physics link: "
                               f"{group.name}")

    vertex_link = {}
    for vertex in obj.data.vertices:
        merged = {}
        for entry in vertex.groups:
            if entry.weight > 1e-6:
                link = group_link[entry.group]
                merged[link] = merged.get(link, 0.0) + entry.weight
        if not merged:
            raise RuntimeError(f"Unweighted vertex {vertex.index}")
        vertex_link[vertex.index] = max(merged, key=merged.get)

    # ------------------------------------------------------------ straighten
    link_transform = {link: Matrix.Identity(4) for link in ORDER}
    for side, sign in (("l", 1.0), ("r", -1.0)):
        upper, lower, hand = (f"upperarm_{side}", f"lowerarm_{side}",
                              f"hand_{side}")
        thigh, calf, foot = (f"thigh_{side}", f"calf_{side}", f"foot_{side}")
        prefix = "Left" if side == "l" else "Right"

        shoulder = bone_head[upper]
        arm_out = Vector((0.0, sign, 0.0))
        upper_rotation = pivot_transform(
            shoulder, rotation_between(bone_head[lower] - shoulder, arm_out))
        elbow = upper_rotation @ bone_head[lower]
        forearm_rotation = pivot_transform(
            elbow,
            rotation_between(upper_rotation @ bone_head[hand] - elbow, arm_out))
        link_transform[f"{prefix}UpperArm"] = upper_rotation
        link_transform[f"{prefix}Forearm"] = forearm_rotation @ upper_rotation
        link_transform[f"{prefix}Hand"] = forearm_rotation @ upper_rotation

        hip = bone_head[thigh]
        leg_down = Vector((0.0, 0.0, -1.0))
        thigh_rotation = pivot_transform(
            hip, rotation_between(bone_head[calf] - hip, leg_down))
        knee = thigh_rotation @ bone_head[calf]
        shin_rotation = pivot_transform(
            knee,
            rotation_between(thigh_rotation @ bone_head[foot] - knee, leg_down))
        link_transform[f"{prefix}Thigh"] = thigh_rotation
        link_transform[f"{prefix}Shin"] = shin_rotation @ thigh_rotation
        # The foot only follows the ankle: rotating it would tip the sole off
        # the floor, and the ankle is not part of the flexion chain anyway.
        ankle = shin_rotation @ thigh_rotation @ bone_head[foot]
        link_transform[f"{prefix}Foot"] = Matrix.Translation(
            ankle - bone_head[foot])

    for vertex in obj.data.vertices:
        vertex.co = link_transform[vertex_link[vertex.index]] @ vertex.co
    obj.data.update()

    bone_link = dict(PRIMARY)
    bone_link.update(AUXILIARY)
    edit_targets = {}
    for bone in rig.data.bones:
        transform = link_transform[bone_link.get(bone.name, "Pelvis")]
        edit_targets[bone.name] = (transform @ bone.head_local,
                                   transform @ bone.tail_local)
    bpy.ops.object.select_all(action="DESELECT")
    rig.select_set(True)
    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode="EDIT")
    for name, (head, tail) in edit_targets.items():
        edit_bone = rig.data.edit_bones[name]
        edit_bone.head, edit_bone.tail = head, tail
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.context.view_layer.update()

    bone_head = {b.name: b.head_local.copy() for b in rig.data.bones}
    standing_root_height = -min(v.co.z for v in obj.data.vertices)
    link_vertices = {link: [] for link in ORDER}
    for vertex in obj.data.vertices:
        link_vertices[vertex_link[vertex.index]].append(vertex.co.copy())

    # -------------------------------------------------------------- segments
    segment = {
        "Pelvis": (bone_head["thigh_r"], bone_head["thigh_l"]),
        "Abdomen": (bone_head["spine_01"], bone_head["spine_02"]),
        "Chest": (bone_head["spine_02"], bone_head["spine_03"]),
        "UpperChest": (bone_head["spine_03"], bone_head["neck_01"]),
        "Neck": (bone_head["neck_01"], bone_head["head"]),
        "Head": (bone_head["head"], bone_head["head"] + Vector((0, 0, 0.18))),
        "LeftUpperArm": (bone_head["upperarm_l"], bone_head["lowerarm_l"]),
        "LeftForearm": (bone_head["lowerarm_l"], bone_head["hand_l"]),
        "LeftHand": (bone_head["hand_l"], bone_head["middle_01_l"]),
        "RightUpperArm": (bone_head["upperarm_r"], bone_head["lowerarm_r"]),
        "RightForearm": (bone_head["lowerarm_r"], bone_head["hand_r"]),
        "RightHand": (bone_head["hand_r"], bone_head["middle_01_r"]),
        "LeftThigh": (bone_head["thigh_l"], bone_head["calf_l"]),
        "LeftShin": (bone_head["calf_l"], bone_head["foot_l"]),
        "LeftFoot": (bone_head["foot_l"], bone_head["ball_l"]),
        "RightThigh": (bone_head["thigh_r"], bone_head["calf_r"]),
        "RightShin": (bone_head["calf_r"], bone_head["foot_r"]),
        "RightFoot": (bone_head["foot_r"], bone_head["ball_r"]),
    }

    def collider_axis(link):
        head, tail = segment[link]
        direction = tail - head
        if link in ("LeftFoot", "RightFoot"):
            direction = Vector((direction.x, direction.y, 0.0))
        if direction.length < 1e-5:
            direction = Vector((0.0, 0.0, 1.0))
        return direction.normalized()

    def fit_collider(link):
        """Half-extents of this link's own vertices, in its bone frame."""
        head = segment[link][0]
        x, y, z = frame_from_axis(collider_axis(link))
        points = link_vertices[link]
        if not points:
            raise RuntimeError(f"No vertices own {link}")
        lows, highs = [], []
        for basis in (x, y, z):
            low, high = span([(p - head).dot(basis) for p in points])
            lows.append(low)
            highs.append(high)
        center = (head + x * ((lows[0] + highs[0]) * 0.5)
                  + y * ((lows[1] + highs[1]) * 0.5)
                  + z * ((lows[2] + highs[2]) * 0.5))
        half = Vector(((highs[0] - lows[0]) * 0.5, (highs[1] - lows[1]) * 0.5,
                       (highs[2] - lows[2]) * 0.5))
        return center, (x, y, z), half

    def fit_lateral_capsule(link, center):
        """Capsule across the body for a rounded torso piece.

        Keeps the link's own origin (`center`, the same one the box used):
        moving it would move the pelvis origin every clip's root track and the
        skin bind are expressed against."""
        height = collider_axis(link)
        lateral = Vector((0.0, 1.0, 0.0))
        lateral = (lateral - height * lateral.dot(height)).normalized()
        depth = height.cross(lateral).normalized()
        points = link_vertices[link]
        halves = []
        for basis in (lateral, depth, height):
            low, high = span([(p - center).dot(basis) for p in points])
            halves.append(max(abs(low), abs(high)))
        half_lateral, half_depth, half_height = halves
        radius = (half_depth + half_height) * 0.5
        length = max(2.0 * radius + 0.012, 2.0 * half_lateral)
        # Local +X on the lateral axis (profile capsules run along local X),
        # +Z along the bone, right-handed.
        x = lateral
        z = (height - x * height.dot(x)).normalized()
        y = z.cross(x).normalized()
        return (x, y, z), radius, length

    # --------------------------------------------------------------- profile
    links, report = [], []
    for link in ORDER:
        center, (bx, by, bz), half = fit_collider(link)
        entry = {
            "id": link,
            "parent": ORDER.index(PARENT_OF[link]) if PARENT_OF[link] else -1,
            "position": vector_json(center),
            "massFraction": MASS_FRACTION[link],
        }
        if link in LATERAL_CAPSULE_LINKS:
            (lx, ly, lz), radius, length = fit_lateral_capsule(link, center)
            entry["capsule"] = {
                "orientation": quaternion_json(lx, ly, lz),
                "length": round(length, 6),
                "radius": round(radius, 6),
                "contactSensor": True,
            }
            report.append(f"{link:15} lat  r={radius:.3f} len={length:.3f}")
        elif link in BOX_LINKS:
            entry["box"] = {
                "orientation": quaternion_json(bx, by, bz),
                "halfExtents": vector_json(half),
                "position": [0.0, 0.0, 0.0],
                "contactSensor": True,
            }
            report.append(f"{link:15} box  "
                          f"half={tuple(round(c, 3) for c in half)}")
        else:
            radius = max(0.02, (half.y + half.z) * 0.5)
            # Validation demands a cylinder, not a sphere: length > 2*radius.
            length = max(2.0 * radius + 0.012, 2.0 * half.x)
            entry["capsule"] = {
                "orientation": quaternion_json(bx, by, bz),
                "length": round(length, 6),
                "radius": round(radius, 6),
                "contactSensor": True,
            }
            report.append(f"{link:15} cap  r={radius:.3f} len={length:.3f}")

        if PARENT_OF[link] is not None:
            if link in HINGE_FRAME:
                frame = HINGE_FRAME[link]
            elif link in ("LeftFoot", "RightFoot"):
                frame = SIDEWAYS_FRAME
            elif link in ("LeftUpperArm", "RightUpperArm",
                          "LeftHand", "RightHand"):
                frame = quaternion_json(*frame_from_axis(collider_axis(link)))
            else:
                frame = VERTICAL_FRAME
            stiffness, damping, torque = DRIVE[link]
            entry["joint"] = {
                "type": "spherical",
                "anchor": vector_json(segment[link][0]),
                "frameOrientation": frame,
                "axes": [{"axis": name, "minimumDegrees": float(low),
                          "maximumDegrees": float(high),
                          "stiffness": stiffness, "damping": damping,
                          "maximumTorque": torque}
                         for name, low, high in LIMITS[link]],
            }
        entry["orientation"] = [0, 0, 0, 1]
        links.append(entry)

    profile = {
        "id": PROFILE_ID,
        "totalMassKg": TOTAL_MASS_KG,
        "uniformRadiusMeters": 0.055,
        "standingRootHeightMeters": round(standing_root_height, 7),
        "links": links,
        "selfCollisionIgnoredPairs": SELF_COLLISION_IGNORED_PAIRS,
    }
    (out / f"{PROFILE_ID}.ragdoll.json").write_text(
        json.dumps(profile, indent=2) + "\n")
    print("\n".join(report))
    print("MASS_SUM", round(sum(MASS_FRACTION.values()), 6))

    # ------------------------------------------------------------------ skin
    # The exporter looks a link up by bone name first, so the 18 physical
    # bones carry the link id and everything else routes through physics_link.
    for bone in rig.data.bones:
        if bone.name in AUXILIARY:
            bone["physics_link"] = AUXILIARY[bone.name]
        elif bone.name not in PRIMARY:
            bone["physics_link"] = "Pelvis"
    for source_name, link in PRIMARY.items():
        group = obj.vertex_groups.get(source_name)
        rig.data.bones[source_name].name = link
        if group:
            group.name = link

    obj["smooth_skin"] = True
    obj["maximum_discarded_skin_weight"] = 0.02

    spec = importlib.util.spec_from_file_location(
        "export_ragdoll_skin", REPO / "tools" / "export_ragdoll_skin.py")
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    exporter.export_skin(obj, rig, profile, str(out / "als.skin.json"),
                         strict_anchors=True)

    bpy.ops.wm.save_as_mainfile(filepath=str(out / "als-rigged.blend"))
    print("PREPARED", PROFILE_ID)


if __name__ == "__main__":
    main()
