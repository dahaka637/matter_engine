"""Etapa 5 (22/10/2026): a cabeca nova.

A cabeca da montagem sai e entra a do usuario (MODELOS_PRONTOS/cabeca_2.0.glb:
uma cabeca low-poly com olhos, num busto de estatua):
 - no lugar: o rosto do arquivo olha para +X; o centro dela fica em y -0,106;
   a escala leva o queixo ao topo a mesma altura da cabeca de antes (23,7 cm:
   a largura com as orelhas e a profundidade batem sozinhas) e a frente e a nuca
   do pescoco caem na frente e na nuca do anel do pescoco do corpo;
 - o busto sai: o corte e CUT acima do anel do pescoco do corpo, logo abaixo do
   queixo; a base do pescoco (2 cm mais fina que o anel) alarga ate ele,
   sumindo ate o queixo;
 - simetrica (cada vertice no espelho exato do par);
 - costurada no anel do corpo pelo ziper otimo (sem fresta nem sobreposicao);
 - os pesos da cabeca de antes (o ponto mais perto dela);
 - o mapa: a cabeca numa ilha so (a costura no meio da nuca, o desenrolar de
   menor esticamento com o rosto pesando mais que a nuca; em pe e centrada pelo
   cilindro em volta da cabeca), na metade direita de um atlas com o dobro da
   largura (o atlas de antes fica inteiro na metade esquerda, pixel por pixel:
   o u de todas as outras faces vai a metade; a metade nova fica preta para a
   etapa 06 pintar e a 03 fazer a margem).

    blender -b jogador.blend --python tools/blender/etapas/05_cabeca.py -- <cabeca.glb> [saida.blend]
"""
import heapq
import json
import math
import os
import sys

import bmesh
import bpy
import mathutils
import numpy as np
from mathutils.bvhtree import BVHTree
from mathutils.kdtree import KDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from malha_comum import aligned, barycentric, loops_of, untangle, zipper  # noqa: E402

argv = sys.argv[sys.argv.index("--") + 1:]
SOURCE = argv[0]
# Medidas do arquivo (raios no contorno): o queixo em z -0,62, o topo em 1,77,
# o meio em y -0,106, o pescoco com a frente em x -0,178 e a nuca em -1,45.
SCALE = 0.237 / 2.39
CHIN_FILE, CHIN_MODEL = -0.62, 0.585
CENTER_Y = -0.106
NECK_FILE_X, NECK_MODEL_X = -0.815, -0.04
CUT = 0.581
WIDEN_TOP = 0.605

body = bpy.data.objects["Jogador"]
rules = json.loads(bpy.data.texts["matter_troca"].as_string())
names = [p["name"] for p in rules["parts"]]
HEAD, SKIN = names.index("cabeca"), names.index("corpo")
SKIN_MATERIAL = next(i for i, m in enumerate(rules["materials"]) if m["name"] == "pele")


def to_blender(fx, fy, fz):
    """Do arquivo para o Blender da troca (modelo: x frente, y esquerda; Blender X = y, Y = -x)."""
    x = SCALE * (fx - NECK_FILE_X) + NECK_MODEL_X
    y = SCALE * (fy - CENTER_Y)
    z = SCALE * (fz - CHIN_FILE) + CHIN_MODEL
    return mathutils.Vector((y, -x, z))


# 1. A cabeca do arquivo, no lugar, sem o busto.
before = set(bpy.data.objects)
bpy.ops.import_scene.gltf(filepath=SOURCE)
imported = [o for o in bpy.data.objects if o not in before and o.type == "MESH"][0]
head = bmesh.new()
head.from_mesh(imported.data)
for v in head.verts:
    w = imported.matrix_world @ v.co
    v.co = to_blender(w.x, w.y, w.z)
for o in [o for o in bpy.data.objects if o not in before]:
    bpy.data.objects.remove(o, do_unlink=True)
bmesh.ops.remove_doubles(head, verts=head.verts[:], dist=1e-6)
cut = bmesh.ops.bisect_plane(head, geom=head.verts[:] + head.edges[:] + head.faces[:], dist=1e-6,
                             plane_co=(0.0, 0.0, CUT), plane_no=(0.0, 0.0, 1.0), clear_inner=True)
# (So a peca ligada a cabeca: o que sobrar do busto acima do corte sai.)
start = max(head.verts, key=lambda v: v.co.z)
keep, stack = {start}, [start]
while stack:
    v = stack.pop()
    for e in v.link_edges:
        u = e.other_vert(v)
        if u not in keep:
            keep.add(u)
            stack.append(u)
bmesh.ops.delete(head, geom=[v for v in head.verts if v not in keep], context="VERTS")
for v in head.verts:
    if abs(v.co.z - CUT) < 1e-5:
        v.co.z = CUT

# 2. O anel do pescoco do corpo (a costura da cabeca de antes).
bm = bmesh.new()
bm.from_mesh(body.data)
part = bm.faces.layers.int.get("parte")
deform = bm.verts.layers.deform.active
uv_layer = bm.loops.layers.uv.get("atlas")
old_faces = [f for f in bm.faces if f[part] == HEAD]
old_verts = {v for f in old_faces for v in f.verts}
skin_verts = {v for f in bm.faces if f[part] != HEAD for v in f.verts}
ring = old_verts & skin_verts
ring_loop = loops_of([e for e in bm.edges if all(v in ring for v in e.verts)
                      and any(f[part] == HEAD for f in e.link_faces) and any(f[part] != HEAD for f in e.link_faces)])
if len(ring_loop) != 1:
    sys.exit(f"o anel do pescoco: {len(ring_loop)} voltas")
ring_loop = ring_loop[0]
center = sum((v.co for v in ring_loop), mathutils.Vector()) / len(ring_loop)

# A base do pescoco alarga ate o anel (pelo angulo em volta do centro dele),
# inteira no corte e nada no queixo.


def angle(p):
    return math.atan2(p.y - center.y, p.x - center.x)


def radius_at(loop, a):
    pts = sorted(((angle(v.co), (v.co - center).to_2d().length) for v in loop))
    pts = [(pts[-1][0] - 2 * math.pi, pts[-1][1])] + pts + [(pts[0][0] + 2 * math.pi, pts[0][1])]
    for (a0, r0), (a1, r1) in zip(pts, pts[1:]):
        if a0 <= a <= a1:
            return r0 + (r1 - r0) * (a - a0) / max(1e-9, a1 - a0)
    return pts[1][1]


head_loop = loops_of([e for e in head.edges if e.is_boundary])
if len(head_loop) != 1:
    sys.exit(f"o corte do pescoco: {len(head_loop)} voltas")
head_loop = head_loop[0]
for v in head.verts:
    if v.co.z >= WIDEN_TOP:
        continue
    a = angle(v.co)
    k = radius_at(ring_loop, a) / max(1e-6, radius_at(head_loop, a))
    t = min(1.0, max(0.0, (WIDEN_TOP - v.co.z) / (WIDEN_TOP - CUT)))
    t = t * t * (3.0 - 2.0 * t)
    offset = (v.co - center).to_2d() * (1.0 + (k - 1.0) * t)
    v.co.x, v.co.y = center.x + offset.x, center.y + offset.y

# Simetrica: cada vertice no espelho exato do par (o meio em x 0).
verts = list(head.verts)
tree = KDTree(len(verts))
for i, v in enumerate(verts):
    tree.insert(v.co, i)
tree.balance()
pairs = {}
for v in verts:
    _, i, d = tree.find(mathutils.Vector((-v.co.x, v.co.y, v.co.z)))
    if d > 0.003:
        sys.exit(f"a cabeca nao e simetrica: um vertice sem par a {d * 1000:.1f} mm")
    pairs[v] = verts[i]
for v, p in pairs.items():
    if p is v:
        v.co.x = 0.0
    elif v.co.x < 0.0:
        p.co = mathutils.Vector((-v.co.x, v.co.y, v.co.z))

# 3. Os pesos da cabeca de antes (o ponto mais perto dela).
old = bmesh.new()
old.from_mesh(body.data)
opart = old.faces.layers.int.get("parte")
bmesh.ops.delete(old, geom=[f for f in old.faces if f[opart] != HEAD], context="FACES")
bmesh.ops.triangulate(old, faces=old.faces[:])
old.faces.ensure_lookup_table()
odeform = old.verts.layers.deform.active
old_tree = BVHTree.FromBMesh(old)
group_name = {g.index: g.name for g in body.vertex_groups}
groups = {g.name: g.index for g in body.vertex_groups}

# 4. A cabeca de antes sai; a nova entra e se costura no anel.
bmesh.ops.delete(bm, geom=old_faces, context="FACES_ONLY")
bmesh.ops.delete(bm, geom=[v for v in old_verts - ring if not v.link_faces], context="VERTS")
copy_of = {v: bm.verts.new(v.co) for v in head.verts}
new_faces = []
for f in head.faces:
    face = bm.faces.new([copy_of[v] for v in f.verts])
    face[part], face.material_index, face.smooth = HEAD, SKIN_MATERIAL, True
    new_faces.append(face)
for v, copy in copy_of.items():
    location, _, index, _ = old_tree.find_nearest(v.co)
    corners = [loop.vert for loop in old.faces[index].loops]
    mixed = {}
    for corner, share in zip(corners, barycentric(location, *(c.co for c in corners))):
        for g, w in corner[odeform].items():
            mixed[group_name[g]] = mixed.get(group_name[g], 0.0) + w * max(share, 0.0)
    ranked = sorted(mixed.items(), key=lambda kv: -kv[1])[:4]
    total = sum(w for _, w in ranked) or 1.0
    for g, w in ranked:
        copy[deform][groups[g]] = w / total
flat = bm.copy()
bmesh.ops.triangulate(flat, faces=[f for f in flat.faces if len(f.verts) > 3])
other_tris = [tuple(f.verts) for f in flat.faces]
flat.verts.index_update()
others = BVHTree.FromPolygons([v.co.copy() for v in flat.verts], [[v.index for v in tri] for tri in other_tris])
outer = [copy_of[v] for v in head_loop]
inner = aligned(list(ring_loop), outer)
best = None
for shift in range(-3, 4):
    trial = zipper(outer, inner[shift:] + inner[:shift], others, other_tris)
    if best is None or trial[1] < best[1]:
        best = trial
tris, _, crossed = best
if crossed:
    print(f"  aviso: {crossed} triangulo(s) da costura do pescoco cruzam a malha")
for tri in tris:
    if len(set(tri)) < 3:
        continue
    face = bm.faces.new(tri)
    face[part], face.material_index, face.smooth = HEAD, SKIN_MATERIAL, True
    new_faces.append(face)
# (O que a costura do pescoco cruza na cabeca relaxa ate nao cruzar.)
print(f"  desembaraco do pescoco: {untangle(bm, set(copy_of.values()))} rodada(s)")
bm.normal_update()
bmesh.ops.recalc_face_normals(bm, faces=new_faces)

# 5. O mapa: o u de tudo a metade; a cabeca desenrolada na metade direita.
added = set(new_faces)
for f in bm.faces:
    if f in added:
        continue
    for loop in f.loops:
        loop[uv_layer].uv.x *= 0.5
bm.to_mesh(body.data)
body.data.update()
temp_mesh = bpy.data.meshes.new("CabecaTemp")
temp = bpy.data.objects.new("CabecaTemp", temp_mesh)
bpy.context.collection.objects.link(temp)
tb = bmesh.new()
index_of = {}
for f in new_faces:
    for v in f.verts:
        if v not in index_of:
            index_of[v] = tb.verts.new(v.co)
for f in new_faces:
    tb.faces.new([index_of[v] for v in f.verts])
# A costura do mapa: o meio da nuca, do anel do pescoco ao topo (o caminho mais
# curto colado no x 0). A cabeca abre numa ilha so, o rosto inteiro no meio,
# como um mapa de rosto que se pinta por cima.
tb.verts.ensure_lookup_table()
tb.verts.index_update()
start = max((v for v in tb.verts if v.is_boundary), key=lambda v: v.co.y - 10.0 * abs(v.co.x))
goal = max(tb.verts, key=lambda v: v.co.z)
cost, back, frontier = {start: 0.0}, {}, [(0.0, start.index)]
while frontier:
    c, i = heapq.heappop(frontier)
    v = tb.verts[i]
    if v is goal:
        break
    if c > cost[v]:
        continue
    for e in v.link_edges:
        u = e.other_vert(v)
        step = e.calc_length() + 10.0 * abs(u.co.x + v.co.x) * 0.5
        if c + step < cost.get(u, math.inf):
            cost[u], back[u] = c + step, (v, e)
            heapq.heappush(frontier, (c + step, u.index))
v = goal
while v is not start:
    v, e = back[v]
    e.seam = True
tb.to_mesh(temp_mesh)
tb.free()
# O rosto vale mais pixels que a nuca e o pescoco (o desenrolar de menor
# esticamento leva esse peso: a frente acima do queixo pesa 1, o resto 0).
importance = temp.vertex_groups.new(name="importancia")
middle_y = sum(v.co.y for v in temp_mesh.vertices) / len(temp_mesh.vertices)
for v in temp_mesh.vertices:
    front = min(1.0, max(0.0, (middle_y + 0.01 - v.co.y) / 0.05))
    high = min(1.0, max(0.0, (v.co.z - 0.60) / 0.04))
    importance.add([v.index], front * high, "REPLACE")
for o in bpy.context.view_layer.objects:
    o.select_set(False)
bpy.context.view_layer.objects.active = temp
temp.select_set(True)
bpy.ops.object.mode_set(mode="EDIT")
bpy.ops.mesh.select_all(action="SELECT")
bpy.ops.uv.unwrap(method="MINIMUM_STRETCH", fill_holes=True, correct_aspect=True, margin=0.0, iterations=400,
                  use_weights=True, weight_group="importancia", weight_factor=2.0)
bpy.ops.object.mode_set(mode="OBJECT")
temp_uv = temp_mesh.uv_layers.active.data
# Em pe e centrada: a rotacao que leva o mapa ao cilindro em volta da cabeca
# (o angulo em volta do eixo vertical e a altura; o rosto no meio, o queixo
# embaixo), e a escala que enche o quadrado com folga.
mid = np.array([sum(v.co.x for v in temp_mesh.vertices), sum(v.co.y for v in temp_mesh.vertices)]) / len(temp_mesh.vertices)
cyl, flat = [], []
for loop in temp_mesh.loops:
    co = temp_mesh.vertices[loop.vertex_index].co
    turn = math.atan2(co.x - mid[0], -(co.y - mid[1]))
    if abs(turn) < 2.3:
        cyl.append((turn * 0.09, co.z))
        flat.append(tuple(temp_uv[loop.index].uv))
cyl, flat = np.array(cyl), np.array(flat)
u_, s_, vt_ = np.linalg.svd((flat - flat.mean(0)).T @ (cyl - cyl.mean(0)))
turn_uv = u_ @ vt_
if np.linalg.det(turn_uv) < 0.0:
    sys.exit("o mapa da cabeca saiu espelhado")
all_uv = np.array([tuple(d.uv) for d in temp_uv]) @ turn_uv
low, high = all_uv.min(0), all_uv.max(0)
fit = (1.0 - 2.0 * 0.012) / max(high - low)
all_uv = (all_uv - (low + high) * 0.5) * fit + 0.5
for d, uv in zip(temp_uv, all_uv):
    d.uv = uv
bm = bmesh.new()
bm.from_mesh(body.data)
part = bm.faces.layers.int.get("parte")
uv_layer = bm.loops.layers.uv.get("atlas")
head_faces = [f for f in bm.faces if f[part] == HEAD]
# (As faces da cabeca na mesma ordem em que entraram no objeto temporario.)
positions = {}
for poly in temp_mesh.polygons:
    key = tuple(sorted(tuple(round(c, 6) for c in temp_mesh.vertices[i].co) for i in poly.vertices))
    positions[key] = {tuple(round(c, 6) for c in temp_mesh.vertices[temp_mesh.loops[li].vertex_index].co): temp_uv[li].uv.copy()
                      for li in poly.loop_indices}
for f in head_faces:
    key = tuple(sorted(tuple(round(c, 6) for c in v.co) for v in f.verts))
    uvs = positions[key]
    for loop in f.loops:
        uv = uvs[tuple(round(c, 6) for c in loop.vert.co)]
        loop[uv_layer].uv = mathutils.Vector((0.5 + 0.5 * uv.x, uv.y))
bm.to_mesh(body.data)
body.data.update()
bpy.data.objects.remove(temp, do_unlink=True)

# O atlas com o dobro da largura (o de antes na metade esquerda).
texture = rules["textures"][0]
image = bpy.data.images.get(texture) or next(i for i in bpy.data.images if i.filepath.endswith(texture))
w, h = image.size
old_px = np.array(image.pixels[:], dtype=np.float32).reshape(h, w, 4)
if w == h:
    # (A metade nova preta: a etapa 06 pinta a cabeca e a 03 faz a margem.)
    wide = np.zeros((h, 2 * w, 4), np.float32)
    wide[:, :w] = old_px
    wide[:, w:, 3] = 1.0
    image.scale(2 * w, h)
    image.pixels[:] = wide.ravel()
    image.pack()
print(f"CABECA: {len(head_faces)} faces ({len(tris)} na costura do pescoco); atlas {image.size[0]}x{image.size[1]}")
bpy.ops.wm.save_as_mainfile(filepath=argv[1] if len(argv) > 1 else bpy.data.filepath)
