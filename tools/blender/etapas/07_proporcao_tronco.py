"""Etapa 7 (22/10/2026): a proporcao do tronco perto da axila, depois dos pesos da etapa 2.

A camisa ligava o tronco ao braco ate 1,29 m do chao (a axila humana de 1,80 m
fica a 1,366 m, ANSUR II) e o tronco abria de 33 para 37 cm logo abaixo dela: o
usuario via uma "asa" do peito ate o braco. A deformacao (tools/blender/
proporcao_tronco.py: ARAP so na superficie principal da camisa, o vale e as
dobras da axila subindo, a lateral ate a largura alvo, um lado e o espelho) muda
so as posicoes; cada vertice leva os pesos que ja tinha (o assado e o desvirar
partem deles na forma nova). Roda depois da 02 porque as regras de posicao dela
(o flanco, a cava) foram medidas na forma antiga. O peso do braco (e da clavicula)
na camisa e cortado pela distancia ao braco (proporcao_tronco.arm_allowance: o peito, a
barriga e as costas nao andam com ele); o assado e o desvirar mantem o corte
(--braco-perto). Depois da axila, o ombro e a manga encolhem para a proporcao humana
(proporcao_ombro: a casca do ombro a ~5,4 cm da junta, a manga justa). As faces da camisa com o mapa degenerado (a area no atlas quase
zero: na axila antiga ficavam escondidas, na forma nova sao lateral a vista) sao
desdobradas de novo num lugar livre do atlas. A faixa preta do lado da camisa e o
punho sao repintados na forma nova (as regras do motor, a borda lisa); rode a 03
de novo depois desta (as margens das ilhas).

    blender -b 2_pesos.blend --python tools/blender/etapas/07_proporcao_tronco.py -- [saida.blend]
"""
import json
import os
import sys

import bmesh
import bpy
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import proporcao_cabeca  # noqa: E402
import proporcao_calcao  # noqa: E402
import proporcao_ombro  # noqa: E402
import proporcao_tronco  # noqa: E402

DEGENERATE = 4.0          # mm por pixel acima disso (a camisa tem ~1): o mapa da face e degenerado
DENSITY = 1.0             # mm por pixel do mapa novo dessas faces
GAP = 16                  # pixels livres em volta de cada ilha nova (a margem da etapa 03 e de 8)

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
body = bpy.data.objects["Jogador"]
rules = json.loads(bpy.data.texts["matter_troca"].as_string())
names = [p["name"] for p in rules["parts"]]
joints = [j["name"] for j in rules["joints"]]
joint_of = {n: i for i, n in enumerate(joints)}
group_name = {g.index: g.name for g in body.vertex_groups}
SHIRT = names.index("camisa")

bm = bmesh.new()
bm.from_mesh(body.data)
part = bm.faces.layers.int.get("parte")
deform = bm.verts.layers.deform.active

# 1. A forma nova e o peso do braco cortado pela distancia.
bm.verts.ensure_lookup_table()
bm.verts.index_update()
bm.faces.ensure_lookup_table()
blender = np.array([v.co[:] for v in bm.verts])
pos = np.stack([-blender[:, 1], blender[:, 0], blender[:, 2]], 1)      # Blender -> eixos do modelo
tris, tri_part = [], []
for f in bm.faces:
    corners = [v.index for v in f.verts]
    for k in range(1, len(corners) - 1):
        tris.append((corners[0], corners[k], corners[k + 1]))
        tri_part.append(f[part])
tris, tri_part = np.array(tris), np.array(tri_part)
weights = [[(joint_of[group_name[g]], w) for g, w in v[deform].items() if group_name[g] in joint_of] for v in bm.verts]
at = {j["name"]: np.array(j["position"]) for j in rules["joints"]}
new, info = proporcao_tronco.deform(pos, tris, tri_part, names, weights, joints, rules["ground"], joint_positions=at)
# O ombro e a manga na proporcao humana (a casca em volta da junta encolhe; proporcao_ombro).
new, shoulder = proporcao_ombro.shape_shoulder(new, tris, tri_part, names, at, rules["ground"])
print(f"OMBRO: {shoulder}")
# A boca na largura humana (proporcao_cabeca).
new, head = proporcao_cabeca.shape_head(new, tris, tri_part, names, at, rules["ground"])
print(f"CABECA: {head}")
# O vao entre as pernas do calcao em V (proporcao_calcao.v_crotch).
new, v_width, v_shift = proporcao_calcao.v_crotch(new, tris, tri_part, names, rules["ground"])
new, fly_moved, fly_max = proporcao_calcao.smooth_fly(new, tris, tri_part, names, rules["ground"])
for v, p in zip(bm.verts, new):
    v.co = (p[1], -p[0], p[2])
Wm = np.zeros((len(pos), len(joints)))
for v, pairs in enumerate(weights):
    for j, w in pairs:
        Wm[v, j] = w
shirt_rows = np.unique(tris[tri_part == SHIRT])
limited = proporcao_tronco.limit_arm(Wm, shirt_rows, proporcao_tronco.arm_allowance(new, at), proporcao_tronco.arm_joints(joints),
                                     joint_of["UpperChest"])
inner = proporcao_calcao.apply_floor(Wm, *proporcao_calcao.inner_leg_floor(Wm, new, tris, tri_part, names, joints, rules["ground"]))
front = proporcao_calcao.apply_pair_floor(Wm, *proporcao_calcao.front_middle_floor(Wm, new, tris, tri_part, names, joints, rules["ground"]))
print(f"CALCAO: o vao em V (a barra a {100 * v_width:.1f} cm do meio, a borda de dentro entra ate {100 * v_shift:.1f} cm); a parede de "
      f"dentro da perna seguindo a coxa em {inner} vertices; a frente do meio com as HipLift em {front}; a braguilha alisada em {fly_moved} vertices (ate {1000 * fly_max:.1f} mm)")
shorts_rows = proporcao_calcao.shorts_vertices(tris, tri_part, names)
groups = {g.name: g.index for g in body.vertex_groups}
for v in np.union1d(shirt_rows, shorts_rows):
    vert = bm.verts[int(v)]
    for g in list(vert[deform].keys()):
        del vert[deform][g]
    for j in np.nonzero(Wm[v] > 1e-4)[0]:
        if joints[j] not in groups:
            groups[joints[j]] = body.vertex_groups.new(name=joints[j]).index
        vert[deform][groups[joints[j]]] = float(Wm[v, j])
bm.to_mesh(body.data)
bm.free()

# 2. As faces da camisa com o mapa degenerado: desdobradas de novo (conformal) num lugar
# livre do atlas, cada grupo ligado uma ilha, a DENSITY mm por pixel.
image = bpy.data.images.get(rules["textures"][0]) or next(i for i in bpy.data.images if i.filepath.endswith(rules["textures"][0]))
W, H = image.size
mesh = body.data
uv_data = mesh.uv_layers["atlas"].data
co = np.array([v.co[:] for v in mesh.vertices])
face_part = np.array([a.value for a in mesh.attributes["parte"].data])
polys = list(mesh.polygons)


def fan_areas(p):
    vs, ls = list(p.vertices), list(p.loop_indices)
    a3 = a2 = 0.0
    for k in range(1, len(vs) - 1):
        a, b, c = co[vs[0]], co[vs[k]], co[vs[k + 1]]
        a3 += 0.5 * np.linalg.norm(np.cross(b - a, c - a)) * 1e6
        u0, u1, u2 = (np.array(uv_data[ls[i]].uv) * (W, H) for i in (0, k, k + 1))
        a2 += 0.5 * abs((u1[0] - u0[0]) * (u2[1] - u0[1]) - (u2[0] - u0[0]) * (u1[1] - u0[1]))
    return a3, a2


def worst_corner(p):
    """O pior triangulo de canto da face (cada canto com os dois vizinhos: as duas diagonais do quad)."""
    vs, ls = list(p.vertices), list(p.loop_indices)
    n, worst = len(vs), 0.0
    for k in range(n):
        i0, i1, i2 = (k - 1) % n, k, (k + 1) % n
        a, b, c = co[vs[i0]], co[vs[i1]], co[vs[i2]]
        a3 = 0.5 * np.linalg.norm(np.cross(b - a, c - a)) * 1e6
        u0, u1, u2 = (np.array(uv_data[ls[i]].uv) * (W, H) for i in (i0, i1, i2))
        a2 = 0.5 * abs((u1[0] - u0[0]) * (u2[1] - u0[1]) - (u2[0] - u0[0]) * (u1[1] - u0[1]))
        worst = max(worst, np.sqrt(a3 / max(1e-9, a2)))
    return worst


bad = [p.index for p in polys if face_part[p.index] == SHIRT and worst_corner(p) > DEGENERATE]
moved_islands = 0
if bad:
    # Os grupos ligados (por aresta).
    owner = {}
    for i in bad:
        for e in polys[i].edge_keys:
            owner.setdefault(e, []).append(i)
    parent = {i: i for i in bad}

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for faces in owner.values():
        for a, b in zip(faces, faces[1:]):
            parent[find(a)] = find(b)
    clusters = {}
    for i in bad:
        clusters.setdefault(find(i), []).append(i)
    # O desdobramento: um objeto temporario so com essas faces.
    used = sorted({v for i in bad for v in polys[i].vertices})
    index = {v: k for k, v in enumerate(used)}
    temp_mesh = bpy.data.meshes.new("desdobrar")
    temp_mesh.from_pydata([tuple(co[v]) for v in used], [], [[index[v] for v in polys[i].vertices] for i in bad])
    temp_mesh.uv_layers.new(name="atlas")
    temp = bpy.data.objects.new("desdobrar", temp_mesh)
    bpy.context.scene.collection.objects.link(temp)
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    bpy.context.view_layer.objects.active = temp
    temp.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.unwrap(method="CONFORMAL", fill_holes=True, correct_aspect=True, margin=0.02)
    bpy.ops.object.mode_set(mode="OBJECT")
    temp_uv = {}
    for k, i in enumerate(bad):
        tp = temp_mesh.polygons[k]
        temp_uv[i] = [np.array(temp_mesh.uv_layers["atlas"].data[l].uv) for l in tp.loop_indices]
    bpy.data.objects.remove(temp, do_unlink=True)
    bpy.data.meshes.remove(temp_mesh)
    # O lugar livre: o atlas ocupado pelas outras faces (em celulas de 4 pixels, com folga).
    CELL = 4
    occupied = np.zeros((H // CELL + 1, W // CELL + 1), bool)
    bad_set = set(bad)
    for p in polys:
        if p.index in bad_set:
            continue
        uvs = np.array([uv_data[l].uv for l in p.loop_indices]) * (W, H) / CELL
        x0, y0 = np.floor(uvs.min(0)).astype(int)
        x1, y1 = np.ceil(uvs.max(0)).astype(int)
        occupied[max(0, y0):y1 + 1, max(0, x0):x1 + 1] = True
    grow = int(np.ceil(GAP / CELL))
    padded = occupied.copy()
    for dy in range(-grow, grow + 1):
        for dx in range(-grow, grow + 1):
            padded |= np.roll(np.roll(occupied, dy, 0), dx, 1)
    summed = np.pad(padded.astype(np.int32).cumsum(0).cumsum(1), ((1, 0), (1, 0)))

    def free_at(r, c, h, w):
        return summed[r + h, c + w] - summed[r, c + w] - summed[r + h, c] + summed[r, c] == 0

    for members in clusters.values():
        loops = [(i, k) for i in members for k in range(len(temp_uv[i]))]
        pts = np.array([temp_uv[i][k] for i, k in loops])
        a3 = sum(fan_areas(polys[i])[0] for i in members)
        a2 = 0.0
        for i in members:
            u = temp_uv[i]
            for k in range(1, len(u) - 1):
                a2 += 0.5 * abs((u[k][0] - u[0][0]) * (u[k + 1][1] - u[0][1]) - (u[k + 1][0] - u[0][0]) * (u[k][1] - u[0][1]))
        scale = np.sqrt(a3 / max(1e-12, a2)) / DENSITY           # pixels por unidade do mapa temporario
        low = pts.min(0)
        size = (pts.max(0) - low) * scale
        h_cells, w_cells = int(np.ceil(size[1] / CELL)) + 1, int(np.ceil(size[0] / CELL)) + 1
        spot = None
        for r in range(0, padded.shape[0] - h_cells):
            for c in range(0, padded.shape[1] // 2 - w_cells):               # so a metade do corpo (a da cabeca fica)
                if free_at(r, c, h_cells, w_cells):
                    spot = (r, c)
                    break
            if spot:
                break
        if spot is None:
            sys.exit("o atlas nao tem lugar livre para as faces de mapa degenerado")
        r, c = spot
        padded[max(0, r - grow):r + h_cells + grow, max(0, c - grow):c + w_cells + grow] = True
        summed = np.pad(padded.astype(np.int32).cumsum(0).cumsum(1), ((1, 0), (1, 0)))
        for i, k in loops:
            x, y = (temp_uv[i][k] - low) * scale
            uv_data[polys[i].loop_indices[k]].uv = ((c * CELL + x) / W, (r * CELL + y) / H)
        moved_islands += 1

# 3. O punho e a faixa do lado repintados na forma nova (com o mapa das faces ja refeito).
uv, skip = [], []
bad_set = set(bad)
for p in polys:
    ls = list(p.loop_indices)
    for k in range(1, len(ls) - 1):
        uv.append([tuple(uv_data[ls[0]].uv), tuple(uv_data[ls[k]].uv), tuple(uv_data[ls[k + 1]].uv)])
        skip.append(p.index in bad_set)
pixels = np.array(image.pixels[:], np.float32).reshape(H, W, 4)
rgb = pixels[:, :, :3].copy()
changed, top_before, top_after = proporcao_tronco.repaint_sides(rgb, np.array(uv), pos, new, tris, tri_part, names, rules["ground"], at,
                                                                fresh=np.array(skip))
pixels[:, :, :3] = rgb
image.pixels[:] = pixels.ravel()
image.pack()
moved = int((np.linalg.norm(new - pos, axis=1) > 1e-6).sum())
print(f"PROPORCAO: {moved} vertices; {info}; braco cortado em {limited} vertices da camisa; {len(bad)} faces de mapa degenerado "
      f"em {moved_islands} ilhas novas; punho e faixa repintados: {changed} pixels (alto da faixa {top_before:.3f} -> {top_after:.3f} m)")
bpy.ops.wm.save_as_mainfile(filepath=argv[0] if argv else bpy.data.filepath)
