"""Uma pose de um rig do Mixamo (as posicoes de mundo das juntas, tiradas no Blender) no esqueleto SOMA77 do Kimodo.

Cada osso do SOMA aponta para onde o osso equivalente aponta na pose. A frente e a da pose (no Blender, o Mixamo
olha para -Y), nunca a linha do quadril (girada ate ~40 graus numa passada: a pose inteira entrava girada e os
membros iam para os lados). O quadril fica de frente para a corrida (sem giro, so a inclinacao da coluna), a coluna e
a cabeca numa linha reta do quadril ao pescoco (--coluna-curva mantem a da pose), e a altura do quadril vem pela razao
do comprimento das pernas. As juntas sem par (o segundo pescoco, os
dedos, os olhos) ficam como no repouso. Grava a pose e o espelho (a outra perna na frente) como rotacoes locais
(eixo-angulo) e a raiz no referencial canonico do Kimodo (Y para cima, frente +Z, esquerda +X). Confere pela cinematica
direta o angulo de cada osso contra a pose.

uso: pose_soma.py pose.json saida.json [--quadro 1] [--dump capsulas.txt] [--coluna-curva]
  pose.json: {"frames": {"1": {"mixamorigN:Hips": {"head": [x, y, z]}, ...}}} (Blender, Z para cima)
"""
import json
import sys

import numpy as np

import sprint_chaves as sc
from soma import IX, NAMES

# SOMA -> Mixamo (sem o prefixo)
PAIRS = {"Hips": "Hips", "Spine1": "Spine", "Spine2": "Spine1", "Chest": "Spine2", "Neck1": "Neck", "Head": "Head",
         "HeadEnd": "HeadTop_End"}
for s in ("Left", "Right"):
    PAIRS.update({f"{s}Shoulder": f"{s}Shoulder", f"{s}Arm": f"{s}Arm", f"{s}ForeArm": f"{s}ForeArm",
                  f"{s}Hand": f"{s}Hand", f"{s}HandMiddle1": f"{s}HandMiddle1", f"{s}Leg": f"{s}UpLeg",
                  f"{s}Shin": f"{s}Leg", f"{s}Foot": f"{s}Foot", f"{s}ToeBase": f"{s}ToeBase",
                  f"{s}ToeEnd": f"{s}Toe_End"})
# cada junta aponta para esta (pode ser um neto: as do meio ficam no repouso)
AIMS = {"Spine1": "Spine2", "Spine2": "Chest", "Chest": "Neck1", "Neck1": "Head", "Head": "HeadEnd"}
for s in ("Left", "Right"):
    AIMS.update({f"{s}Shoulder": f"{s}Arm", f"{s}Arm": f"{s}ForeArm", f"{s}ForeArm": f"{s}Hand",
                 f"{s}Hand": f"{s}HandMiddle1", f"{s}Leg": f"{s}Shin", f"{s}Shin": f"{s}Foot",
                 f"{s}Foot": f"{s}ToeBase", f"{s}ToeBase": f"{s}ToeEnd"})


def unit(v):
    return v / max(np.linalg.norm(v), 1e-9)


def frame(up, lateral):
    """Base ortonormal (lateral, cima, frente) pelas colunas."""
    u = unit(up)
    l = unit(lateral - u * np.dot(lateral, u))
    return np.stack([l, u, np.cross(l, u)], axis=1)


def align(a, b):
    """A menor rotacao que leva a direcao a para b."""
    a, b = unit(a), unit(b)
    v, c = np.cross(a, b), float(np.dot(a, b))
    if c < -0.999999:
        axis = unit(np.cross(a, [1.0, 0, 0]) if abs(a[0]) < 0.9 else np.cross(a, [0, 1.0, 0]))
        return 2 * np.outer(axis, axis) - np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


def source_points(frame_data, prefix, mirror=False, straight=True):
    """As juntas da pose no referencial do Kimodo (esquerda +X, cima +Y, frente +Z), em volta do quadril; o chao em 0.
    A frente da pose e -Y do Blender (o Mixamo importado); a esquerda, +X."""
    get = lambda n: np.array(frame_data[prefix + n]["head"], float)
    up = np.array([0.0, 0.0, 1.0])
    forward = np.array([0.0, -1.0, 0.0])
    left = np.array([1.0, 0.0, 0.0])
    hips = get("Hips")
    names = {n for n in PAIRS.values()}
    out = {}
    for n in names:
        d = get(n) - hips
        out[n] = np.array([np.dot(d, left), np.dot(d, up), np.dot(d, forward)])
    height = float(hips[2])
    if straight:
        # a coluna e a cabeca numa linha reta do quadril ao pescoco (cada ponto na sua distancia ao quadril)
        line = unit(out["Neck"] - out["Hips"])
        for n in ("Spine", "Spine1", "Spine2", "Neck", "Head", "HeadTop_End"):
            out[n] = out["Hips"] + line * np.linalg.norm(out[n] - out["Hips"])
    # os bracos no plano do ombro (o balanco e para a frente e para tras, nunca para os lados): o cotovelo e a mao a
    # no maximo 6 cm do ombro, para fora ou para dentro
    for side in ("Left", "Right"):
        shoulder_x = out[f"{side}Arm"][0]
        for n in (f"{side}ForeArm", f"{side}Hand", f"{side}HandMiddle1"):
            out[n] = out[n].copy()
            out[n][0] = shoulder_x + np.clip(out[n][0] - shoulder_x, -0.06, 0.06)
    if mirror:
        swapped = {}
        for n, p in out.items():
            other = n.replace("Left", "#").replace("Right", "Left").replace("#", "Right")
            swapped[other] = p * np.array([-1.0, 1.0, 1.0])
        out = swapped
    return out, height


def solve(points, parents, offsets):
    rest = sc.forward_kinematics(parents, offsets, np.tile(np.eye(3), (len(NAMES), 1, 1)), np.zeros(3))
    local = np.tile(np.eye(3), (len(NAMES), 1, 1))
    world = np.tile(np.eye(3), (len(NAMES), 1, 1))
    src = lambda soma: points[PAIRS[soma]]
    # a raiz: a coluna baixa e a esquerda da corrida (+X) - o quadril de frente para a corrida, sem giro
    f_src = frame(src("Spine1") - src("Hips"), np.array([1.0, 0.0, 0.0]))
    f_rest = frame(rest[IX["Spine1"]] - rest[IX["Hips"]], rest[IX["LeftLeg"]] - rest[IX["RightLeg"]])
    local[0] = world[0] = f_src @ f_rest.T
    for j in range(1, len(NAMES)):
        p = parents[j]
        world[j] = world[p] @ local[j]
        name = NAMES[j]
        if name in AIMS:
            c = IX[AIMS[name]]
            current = world[j] @ (rest[c] - rest[j])  # a direcao do repouso ate o alvo, com o local identidade
            target = src(AIMS[name]) - src(name)
            swing = align(current, target)
            world[j] = swing @ world[j]
            local[j] = world[p].T @ world[j]
    return local


def leg_length(points, side):
    return (np.linalg.norm(points[f"{side}Leg"] - points[f"{side}UpLeg"])
            + np.linalg.norm(points[f"{side}Foot"] - points[f"{side}Leg"]))


def main():
    args = sys.argv[1:]
    straight = "--coluna-curva" not in args
    args = [a for a in args if a != "--coluna-curva"]
    opts = {"--quadro": "1", "--dump": None}
    for key in list(opts):
        if key in args:
            i = args.index(key)
            opts[key] = args[i + 1]
            del args[i:i + 2]
    source, out = args
    data = json.load(open(source))["frames"][opts["--quadro"]]
    prefix = next(k for k in data if k.endswith("Hips"))[:-len("Hips")]
    parents, offsets = sc.skeleton()
    rest = sc.forward_kinematics(parents, offsets, np.tile(np.eye(3), (len(NAMES), 1, 1)), np.zeros(3))
    soma_leg = (np.linalg.norm(rest[IX["LeftShin"]] - rest[IX["LeftLeg"]])
                + np.linalg.norm(rest[IX["LeftFoot"]] - rest[IX["LeftShin"]]))
    result, dumps = {}, []
    for label, mirror in (("pose", False), ("espelho", True)):
        points, height = source_points(data, prefix, mirror, straight)
        scale = soma_leg / (0.5 * (leg_length(points, "Left") + leg_length(points, "Right")))
        local = solve(points, parents, offsets)
        root = np.array([0.0, height * scale, 0.0])
        fk = sc.forward_kinematics(parents, offsets, local, root)
        worst = 0.0
        for name, aim in AIMS.items():
            a = fk[IX[aim]] - fk[IX[name]]
            b = points[PAIRS[aim]] - points[PAIRS[name]]
            worst = max(worst, np.degrees(np.arccos(np.clip(np.dot(unit(a), unit(b)), -1, 1))))
        lean = np.degrees(np.arctan2(*(fk[IX["Neck1"]] - fk[IX["Hips"]])[[2, 1]]))
        side = {n: 100 * (fk[IX[n]][0] - fk[IX[base]][0]) for n, base in (("LeftHand", "LeftArm"), ("RightHand", "RightArm"),
                ("LeftFoot", "LeftLeg"), ("RightFoot", "RightLeg"), ("LeftShin", "LeftLeg"), ("RightShin", "RightLeg"))}
        a, b = fk[IX["Chest"]] - fk[IX["Hips"]], fk[IX["Head"]] - fk[IX["Chest"]]
        bend = np.degrees(np.arccos(np.clip(np.dot(unit(a), unit(b)), -1, 1)))
        print(f"{label}: pior osso {worst:.2f} graus; quadril a {root[1]:.3f} m (escala {scale:.3f}); tronco {lean:.1f} graus;"
              f" curvatura da coluna {bend:.1f} graus; para os lados (cm, + para fora): "
              + ", ".join(f"{n} {v * (1 if n.startswith('Left') else -1):+.0f}" for n, v in side.items()))
        result[label] = {"local_joints_rot": [sc.axis_angle(m).tolist() for m in local], "root": root.tolist()}
        dumps.append((label, fk, points, root, scale))
    json.dump(result, open(out, "w"))
    if opts["--dump"]:
        with open(opts["--dump"], "w") as f:
            f.write("S pose_soma\n")
            for k, (label, fk, points, root, scale) in enumerate(dumps):
                for which, pts in (("soma", {n: fk[IX[n]] for n in PAIRS}), ("fonte", {n: points[m] * scale + root for n, m in PAIRS.items()})):
                    f.write(f"F {k * 2 + (which == 'fonte')}.0 0 0\n")
                    for name, aim in AIMS.items():
                        a, b = pts[name], pts[aim]
                        c = 0.5 * (a + b)
                        # Kimodo (x esquerda, y cima, z frente) -> o render (z para cima)
                        cz = np.array([c[0], -c[2], c[1]])
                        d = np.array([b[0] - a[0], -(b[2] - a[2]), b[1] - a[1]])
                        q = quat_x_to(d)
                        f.write(f"C {name} {cz[0]:.4f} {cz[1]:.4f} {cz[2]:.4f} {q[0]:.5f} {q[1]:.5f} {q[2]:.5f} {q[3]:.5f} "
                                f"{np.linalg.norm(d) / 2:.4f} 0.03 0.03\n")


def quat_x_to(d):
    d = unit(d)
    x = np.array([1.0, 0, 0])
    c = np.cross(x, d)
    w = 1.0 + float(np.dot(x, d))
    if w < 1e-6:
        return np.array([0, 0, 1.0, 0])
    q = np.array([c[0], c[1], c[2], w])
    return q / np.linalg.norm(q)


if __name__ == "__main__":
    main()
