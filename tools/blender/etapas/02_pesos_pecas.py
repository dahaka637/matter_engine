"""Etapa 2 (22/10/2026): os pesos das pecas refeitas.

 - A camisa recebe os pesos v2c (o campo do ombro no referencial do braco, a
   coluna e a ponta da manga parada; medidos os melhores na malha em quads,
   historico/jogador_01_camisa_quads.blend), pelo ponto de origem de cada
   vertice (atributo "origem" da etapa 1: a forma lisa afasta o vertice da
   peca original).
 - A barra costurada na pele (punho, gola, perna do calcao) anda com a pele: a
   barra recebe os pesos da costura (o ponto mais perto dela) e a mistura
   volta aos pesos de antes ate MISTURA metros da barra. A cintura (camisa e
   calcao num laco escondido) fica como esta.
 - O flanco sem o braco; a cava com os pesos alisados; a parte de baixo da
   camisa lisa entre a barra e o tronco.
 - A camisa so no tronco (sem a cadeia da coxa).
 - Tudo espelhado no fim (a esquerda copia a direita; o meio com pares iguais).
 - No maximo quatro pesos por vertice.

    blender -b jogador.blend --python tools/blender/etapas/02_pesos_pecas.py -- <fonte_v2c.blend> [saida.blend]
"""
import json
import sys

import bmesh
import bpy
import mathutils
from mathutils.bvhtree import BVHTree
from mathutils.kdtree import KDTree

argv = sys.argv[sys.argv.index("--") + 1:]
SOURCE = argv[0]
MIXING = 0.025
SLEEVE_MIXING = 0.055

body = bpy.data.objects["Jogador"]
rules = json.loads(bpy.data.texts["matter_troca"].as_string())
names = [p["name"] for p in rules["parts"]]
SHIRT, SHORTS, SKIN = names.index("camisa"), names.index("calcao"), names.index("corpo")


def barycentric(p, a, b, c):
    v0, v1, v2 = b - a, c - a, p - a
    d00, d01, d11, d20, d21 = v0.dot(v0), v0.dot(v1), v1.dot(v1), v2.dot(v0), v2.dot(v1)
    den = d00 * d11 - d01 * d01
    if abs(den) < 1e-18:
        return 1.0, 0.0, 0.0
    v = (d11 * d20 - d01 * d21) / den
    w = (d00 * d21 - d01 * d20) / den
    return 1.0 - v - w, v, w


def top(d, count=4):
    items = [(a, w) for a, w in sorted(d.items(), key=lambda kv: -kv[1])[:count] if w > 1e-4]
    s = sum(w for _, w in items) or 1.0
    return {a: w / s for a, w in items}


bm = bmesh.new()
bm.from_mesh(body.data)
part = bm.faces.layers.int.get("parte")
deform = bm.verts.layers.deform.active
origin = bm.verts.layers.float_vector.get("origem")
groups = {g.name: g.index for g in body.vertex_groups}
group_name = {g.index: g.name for g in body.vertex_groups}


def weights_of(v):
    return {group_name[g]: w for g, w in v[deform].items() if w > 0.0}


def set_weights(v, w):
    for g in list(v[deform].keys()):
        del v[deform][g]
    for a, x in top(w).items():
        if a not in groups:
            g = body.vertex_groups.new(name=a)
            groups[a], group_name[g.index] = g.index, a
        v[deform][groups[a]] = x


def faces_of(piece):
    return [f for f in bm.faces if f[part] == piece]


def verts_of(piece):
    return {v for f in faces_of(piece) for v in f.verts}


# 1. A camisa com os pesos v2c (a superficie principal da fonte, triangulada).
with bpy.data.libraries.load(SOURCE) as (data_from, data_to):
    data_to.objects = ["Jogador"]
src = data_to.objects[0]
sb = bmesh.new()
sb.from_mesh(src.data)
spart = sb.faces.layers.int.get("parte")
sname = {g.index: g.name for g in src.vertex_groups}
sseam = {v for e in sb.edges if sum(1 for f in e.link_faces if f[spart] == SHIRT) == 1 and len(e.link_faces) == 2 for v in e.verts}
bmesh.ops.delete(sb, geom=[f for f in sb.faces if f[spart] != SHIRT or any(v in sseam for v in f.verts)], context="FACES")
bmesh.ops.triangulate(sb, faces=sb.faces[:])
sb.faces.ensure_lookup_table()
sdeform = sb.verts.layers.deform.active
stree = BVHTree.FromBMesh(sb)
shirt_seam = verts_of(SHIRT) & (verts_of(SKIN) | verts_of(SHORTS))
shirt_strip = {v for f in faces_of(SHIRT) if any(u in shirt_seam for u in f.verts) for v in f.verts}
moved = 0
for v in verts_of(SHIRT) - shirt_strip:
    location, _, index, _ = stree.find_nearest(v[origin])
    corners = [loop.vert for loop in sb.faces[index].loops]
    mixed = {}
    for corner, share in zip(corners, barycentric(location, *(c.co for c in corners))):
        for g, x in corner[sdeform].items():
            mixed[sname[g]] = mixed.get(sname[g], 0.0) + x * max(share, 0.0)
    set_weights(v, mixed)
    moved += 1
bpy.data.objects.remove(src, do_unlink=True)

# 1b. O flanco sem o braco: os pesos de calor punham braco no tronco ate 20 cm
# abaixo da axila (26% a 7 cm dela); na corrida o braco balanca e o flanco ia
# junto, dobrando. No tronco (|x| < 16,5 cm no Blender), a cadeia do braco some
# de 30 cm (a altura da axila) a 20 cm sobre a pelve; o que sai vai para as
# juntas da coluna que o vertice ja tem.
ARM_CHAIN = ("UpperArm", "ShoulderHelper", "Forearm", "ElbowHelper", "Hand", "WristHelper")
flank = 0
for v in verts_of(SHIRT) - shirt_strip:
    if abs(v.co.x) >= 0.165:
        continue
    keep = min(1.0, max(0.0, (v.co.z - 0.20) / 0.10))
    keep = keep * keep * (3.0 - 2.0 * keep)
    w = weights_of(v)
    arm = {a: x for a, x in w.items() if any(a.endswith(c) for c in ARM_CHAIN)}
    if not arm or keep >= 1.0:
        continue
    removed = sum(arm.values()) * (1.0 - keep)
    rest_w = {a: x for a, x in w.items() if a not in arm}
    total = sum(rest_w.values())
    out = {a: x * keep for a, x in arm.items()}
    if total > 0.0:
        for a, x in rest_w.items():
            out[a] = x + removed * x / total
    else:
        out["UpperChest"] = removed
    set_weights(v, out)
    flank += 1

# 1c. Os pesos alisados na cava (o ombro e a axila, ate 12 cm da junta do
# ombro, inteiros ate 8 cm; a manga perto do punho e as faixas paradas): os
# degraus do campo amassavam o peito com o braco a frente e erguido (avesso
# 51 -> 45; com 16 cm a manga do punho pegava tronco e cruzava o braco).
CAVA_ROUNDS, CAVA_RADIUS = 20, 0.12
joints_at = {j["name"]: mathutils.Vector((j["position"][1], -j["position"][0], j["position"][2])) for j in rules["joints"]}
region = {}
for v in verts_of(SHIRT) - shirt_strip:
    for side in ("Left", "Right"):
        d = (v.co - joints_at[side + "UpperArm"]).length
        if d < CAVA_RADIUS:
            t = min(1.0, (CAVA_RADIUS - d) / 0.04)
            region[v] = max(region.get(v, 0.0), t * t * (3.0 - 2.0 * t))
field_w = {v: weights_of(v) for v in verts_of(SHIRT)}
for _ in range(CAVA_ROUNDS):
    step = {}
    for v, t in region.items():
        ns = [e.other_vert(u) for u in (v,) for e in u.link_edges]
        mean = {}
        for u in ns:
            for a, x in field_w.get(u, weights_of(u)).items():
                mean[a] = mean.get(a, 0.0) + x / len(ns)
        here = field_w[v]
        step[v] = {a: here.get(a, 0.0) + 0.5 * t * (mean.get(a, 0.0) - here.get(a, 0.0)) for a in set(here) | set(mean)}
    field_w.update(step)
for v in region:
    set_weights(v, field_w[v])

# 1d. A parte de baixo da camisa com os pesos lisos: as fileiras da barra (a
# 1,5 cm dela) e o tronco acima de 14 cm ficam; no meio, a media dos vizinhos
# (Laplace). O HipLift vinha da camisa antiga pelos triangulos grandes dela e a
# borda de um deles virava, na flexao, uma caixa com um vinco em L.
waist_hem = [v for v in shirt_strip - shirt_seam if v.co.z < 0.10]
hem_tree = KDTree(len(waist_hem))
for i, v in enumerate(waist_hem):
    hem_tree.insert(v.co, i)
hem_tree.balance()
# A camisa so no tronco (pedido do usuario, 22/10: a perna nunca deforma a
# camisa; o calcao que entra na barra numa flexao grande e aceito): a cadeia da
# coxa sai de todos os vertices dela (o peso vai para as outras juntas).
THIGH_CHAIN = ("HipLift", "HipHelper", "HipHelper25", "HipHelper75", "Thigh", "GluteHelper")
for v in verts_of(SHIRT) - (shirt_seam & verts_of(SHORTS)):
    w = weights_of(v)
    thigh = sum(x for a, x in w.items() if a.endswith(THIGH_CHAIN))
    if thigh > 0.0:
        others = {a: x for a, x in w.items() if not a.endswith(THIGH_CHAIN)}
        total = sum(others.values()) or 1.0
        set_weights(v, {a: x + thigh * x / total for a, x in others.items()} if others else {"Abdomen": 1.0})
lower = [v for v in verts_of(SHIRT) - shirt_strip if v.co.z < 0.14 and hem_tree.find(v.co)[2] > 0.015 and v.link_edges]
lower_w = {v: weights_of(v) for v in verts_of(SHIRT)}
# As fileiras da barra (paradas no Laplace) primeiro alisadas ao longo dela: o
# HipLift caia de repente da frente para o lado e a lateral fazia degraus com a
# perna levantada.
hem_rows = {v for v in verts_of(SHIRT) - shirt_strip if hem_tree.find(v.co)[2] <= 0.015 and v.link_edges}
for _ in range(30):
    step = {}
    for v in hem_rows:
        ns = [e.other_vert(v) for e in v.link_edges if e.other_vert(v) in hem_rows]
        if not ns:
            continue
        mean = {}
        for u in ns:
            for a, x in lower_w[u].items():
                mean[a] = mean.get(a, 0.0) + x / len(ns)
        here = lower_w[v]
        step[v] = {a: here.get(a, 0.0) + 0.5 * (mean.get(a, 0.0) - here.get(a, 0.0)) for a in set(here) | set(mean)}
    lower_w.update(step)
for v in hem_rows:
    set_weights(v, lower_w[v])
for _ in range(200):
    step = {}
    for v in lower:
        ns = [e.other_vert(v) for e in v.link_edges]
        mean = {}
        for u in ns:
            for a, x in lower_w.get(u, weights_of(u)).items():
                mean[a] = mean.get(a, 0.0) + x / len(ns)
        step[v] = mean
    lower_w.update(step)
for v in lower:
    set_weights(v, lower_w[v])

# 1e. A clavicula (ela leva o braco): a faixa ao longo da linha da junta
# esterno-clavicular ao acromio (o alto do ombro), o trapezio e o alto do peito
# e das costas junto do ombro; CLAVICLE no meio da linha e no trecho de fora,
# sumindo ate 13 cm dela e perto do esterno; o que ela leva sai das juntas que o
# vertice ja tem (o deltoide fica com a meia-junta e o braco, que ela carrega).
CLAVICLE = 0.6


def smooth(e0, e1, x):
    u = min(1.0, max(0.0, (x - e0) / (e1 - e0)))
    return u * u * (3.0 - 2.0 * u)


# (perto da gola nada: a gola fica com o pescoco; com peso de clavicula ela
# subia e a faixa da gola virava)
collar_hem = [v for v in shirt_strip - shirt_seam if v.co.z > 0.45]
collar_tree = KDTree(len(collar_hem))
for i, v in enumerate(collar_hem):
    collar_tree.insert(v.co, i)
collar_tree.balance()
for side in ("Left", "Right"):
    joint = next((j for j in rules["joints"] if j["name"] == side + "ClavicleHelper"), None)
    if joint is None:
        continue
    shoulder = next(j for j in rules["joints"] if j["name"] == side + "UpperArm")["position"]
    a = mathutils.Vector((joint["helper"]["pivot"][1], -joint["helper"]["pivot"][0], joint["helper"]["pivot"][2]))
    b = mathutils.Vector((shoulder[1] * 0.9, -(shoulder[0] + 0.01), shoulder[2] + 0.04))
    ab = b - a
    for v in verts_of(SHIRT) - shirt_strip:
        if (v.co.x > 0.0) != (a.x > 0.0):
            continue
        t = (v.co - a).dot(ab) / ab.dot(ab)
        d = (v.co - (a + ab * min(1.0, max(0.0, t)))).length
        w = CLAVICLE * (1.0 - smooth(0.05, 0.13, d)) * smooth(0.15, 0.55, t) * (1.0 - smooth(1.1, 1.5, t)) \
            * smooth(0.03, 0.07, collar_tree.find(v.co)[2])
        if w <= 1e-3:
            continue
        current = weights_of(v)
        out = {k: x * (1.0 - w) for k, x in current.items()}
        out[side + "ClavicleHelper"] = out.get(side + "ClavicleHelper", 0.0) + w
        set_weights(v, out)

# 2. As barras costuradas na pele andam com ela (os punhos da manga e as barras
# do calcao). A gola nao: presa ao anel de baixo do pescoco (0,10 do Neck), o
# pescoco que inclina se afastava dela e o lado de dentro da gola aparecia; ela
# fica com os pesos da fonte (0,19 do Neck na borda, como na montagem).
rigid = 0
for piece in (SHIRT, SHORTS):
    seam = verts_of(piece) & verts_of(SKIN)
    # As voltas da costura (pelas arestas entre vertices dela); a da gola e a mais alta.
    loops, left_over = [], set(seam)
    while left_over:
        stack = [left_over.pop()]
        loop = set(stack)
        while stack:
            v = stack.pop()
            for e in v.link_edges:
                u = e.other_vert(v)
                if u in left_over:
                    left_over.discard(u)
                    loop.add(u)
                    stack.append(u)
        loops.append(loop)
    if piece == SHIRT and loops:
        seam -= max(loops, key=lambda loop: sum(v.co.z for v in loop) / len(loop))
    strip_faces = [f for f in faces_of(piece) if any(u in seam for u in f.verts)]
    hem = {v for f in strip_faces for v in f.verts} - seam
    if not seam:
        continue
    seam_list = list(seam)
    seam_tree = KDTree(len(seam_list))
    for i, v in enumerate(seam_list):
        seam_tree.insert(v.co, i)
    seam_tree.balance()
    hem_weights = {}
    for v in hem:
        # A costura entre os dois vertices dela mais perto (pela distancia).
        found = seam_tree.find_n(v.co, 2)
        total = sum(1.0 / max(1e-6, d) for _, _, d in found)
        mixed = {}
        for _, i, d in found:
            for a, x in weights_of(seam_list[i]).items():
                mixed[a] = mixed.get(a, 0.0) + x * (1.0 / max(1e-6, d)) / total
        hem_weights[v] = top(mixed)
    hem_list = list(hem)
    hem_tree = KDTree(len(hem_list))
    for i, v in enumerate(hem_list):
        hem_tree.insert(v.co, i)
    hem_tree.balance()
    for v in hem:
        set_weights(v, hem_weights[v])
        rigid += 1
    # (Nos punhos da manga a passagem da barra para os pesos da fonte e mais
    # longa: em 2,5 cm a manga logo acima do punho ja levava peito e, com o
    # braco erguido ou para tras, abria do braco e mostrava o lado de dentro.)
    mixing = SLEEVE_MIXING if piece == SHIRT else MIXING
    for v in verts_of(piece) - hem - seam:
        _, i, d = hem_tree.find(v.co)
        if d >= mixing:
            continue
        t = d / mixing
        keep = t * t * (3.0 - 2.0 * t)
        current = weights_of(v)
        near = hem_weights[hem_list[i]]
        set_weights(v, {a: current.get(a, 0.0) * keep + near.get(a, 0.0) * (1.0 - keep) for a in set(current) | set(near)})

# 3. O gancho do calcao: a costura do meio segue metade de cada dobra da coxa
# para a frente (as duas HipLift meio a meio: no agachamento o meio acompanha as
# pernas pela metade e nao afunda) e cada lado passa para os pesos da sua perna
# (os da parede do V original, pelo ponto de origem) ate 4,5 cm do meio. Preso a
# pelve, o meio ficava parado e as coxas o espremiam num cone; misturando as
# duas coxas inteiras, esticava de coxa a coxa como uma saia; numa faixa
# estreita, a borda dela amassava.
crotch = 0
for v in verts_of(SHORTS):
    t = min(1.0, max(0.0, (0.045 - abs(v.co.x)) / 0.035)) * min(1.0, max(0.0, (-0.10 - v.co.z) / 0.03)) \
        * min(1.0, max(0.0, (v.co.z + 0.24) / 0.03))
    if t <= 0.0:
        continue
    t = t * t * (3.0 - 2.0 * t)
    w = weights_of(v)
    out = {a: x * (1.0 - t) for a, x in w.items()}
    out["LeftHipLift"] = out.get("LeftHipLift", 0.0) + 0.5 * t
    out["RightHipLift"] = out.get("RightHipLift", 0.0) + 0.5 * t
    set_weights(v, out)
    crotch += 1

# 4. Os pesos das pecas espelhados (a malha e simetrica): a esquerda copia a
# direita (Left e Right trocados); no meio so juntas do centro ou pares
# esquerda/direita com o mesmo peso, nas quatro vagas (o par conta duas).
def swapped(name):
    return "Right" + name[4:] if name.startswith("Left") else "Left" + name[5:] if name.startswith("Right") else name


def symmetric_set(w, slots=4):
    groups = {}
    for a, x in w.items():
        key = a[4:] if a.startswith("Left") else a[5:] if a.startswith("Right") else a
        paired = a != key
        g = groups.setdefault((key, paired), 0.0)
        groups[(key, paired)] = g + x
    chosen, used = {}, 0
    for (key, paired), x in sorted(groups.items(), key=lambda kv: -kv[1]):
        need = 2 if paired else 1
        if used + need > slots or x <= 1e-4:
            continue
        used += need
        if paired:
            chosen["Left" + key] = chosen["Right" + key] = x / 2.0
        else:
            chosen[key] = x
    return chosen


piece_verts = verts_of(SHIRT) | verts_of(SHORTS)
everything = list(bm.verts)
tree = KDTree(len(everything))
for i, v in enumerate(everything):
    tree.insert(v.co, i)
tree.balance()
mirrored = 0
for v in piece_verts:
    if v.co.x > 1e-6:
        continue
    if abs(v.co.x) <= 1e-6:
        mine = weights_of(v)
        both = {a: 0.5 * (mine.get(a, 0.0) + mine.get(swapped(a), 0.0)) for a in set(mine) | {swapped(a) for a in mine}}
        set_weights(v, symmetric_set(both))
        mirrored += 1
        continue
    _, i, d = tree.find(mathutils.Vector((-v.co.x, v.co.y, v.co.z)))
    if d > 1e-5:
        sys.exit(f"sem par no espelho a {d * 1000:.3f} mm em {tuple(v.co)}")
    set_weights(everything[i], {swapped(a): x for a, x in weights_of(v).items()})
    mirrored += 1

bm.to_mesh(body.data)
body.data.update()
print(f"PESOS DAS PECAS: camisa v2c em {moved} vertices (flanco sem braco em {flank}); {rigid} vertices de barra com a pele; "
      f"{mirrored} espelhados")
bpy.ops.wm.save_as_mainfile(filepath=argv[1] if len(argv) > 1 else bpy.data.filepath)
