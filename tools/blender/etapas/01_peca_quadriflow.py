"""Etapa 1 (22/10/2026): uma peca do uniforme refeita em quads (QuadriFlow).

A camisa tinha 378 triangulos (enormes no ombro) e o calcao triangulos soltos,
sem lacos: nenhum peso dobra direito neles. Como na retopologia de um
modelador: a peca vira uma grade de quads (o QuadriFlow do Blender, simetrico,
~ALVO faces). As costuras ficam como o motor costura: a superficie principal da
peca (sem as faixas do recuo) e remalhada; cada borda nova e projetada na linha
da barra original (anda milimetros); tudo vai para a forma lisa da peca (a
tesselacao de Phong da original: sem as facetas dos triangulos grandes); as faixas do recuo antigas saem e uma
faixa nova e costurada em ziper otimo entre a borda nova e a borda do outro lado
(a pele do pescoco, do braco ou da coxa; o calcao na cintura), sem mexer nela. O mapa e os pesos
vem da peca original pelo ponto mais perto (o mapa dentro da ilha do atlas de
cada face: a pintura fica no lugar).

    blender -b jogador.blend --python tools/blender/etapas/01_peca_quadriflow.py -- [saida.blend] [alvo] [peca] [rodadas de Taubin] [remalha] [densidade]

remalha: quadriflow (padrao; o alvo em faces) ou moldes (25/10: a peca pelos moldes de
uma roupa costurada, retopo_moldes.py; a densidade multiplica as divisoes das costuras).

O molde de cada face nova vai para o atributo de face "molde" (26/10): o dos
moldes da roupa e, nas faixas do recuo, o da abertura (recuo_gola, recuo_punho_direito...);
a tabela dos nomes fica no texto "matter_moldes" (retopo_moldes.MOLDES).
"""
import json
import math
import os
import sys

import bmesh
import bpy
import mathutils
from mathutils.bvhtree import BVHTree

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
TARGET = int(argv[1]) if len(argv) > 1 else 2400
PIECE = argv[2] if len(argv) > 2 else "camisa"
FAIRING = int(argv[3]) if len(argv) > 3 else 10
REMESHER = argv[4] if len(argv) > 4 else "quadriflow"
DENSITY = float(argv[5]) if len(argv) > 5 else 1.0
# A peca de origem: "original" (a da montagem, grossa: a forma lisa de Phong, o
# Taubin e, no calcao, a membrana, as nadegas e o sulco) ou "final" (25/10: a
# peca ja pronta do jogador instalado, densa e lisa; so troca a malha, sem mexer
# na forma: nem Phong, nem Taubin, nem os retoques do calcao).
SOURCE = argv[6] if len(argv) > 6 else "original"
FINAL = SOURCE == "final"
# O quanto a malha grossa sai para fora da peca final (m).
FINAL_LIFT = float(argv[7]) if len(argv) > 7 else 0.002

body = bpy.data.objects["Jogador"]
rules = json.loads(bpy.data.texts["matter_troca"].as_string())
PART = [p["name"] for p in rules["parts"]].index(PIECE)


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from malha_comum import aligned, barycentric, centroid, crossing, loops_of, nearest_on_polyline, push_clear, untangle, zipper  # noqa: E402
from retopo_moldes import MOLDES  # noqa: E402


# 1. A malha, a peca e as suas costuras.
bm = bmesh.new()
bm.from_mesh(body.data)
part = bm.faces.layers.int.get("parte")
molde = bm.faces.layers.int.get("molde") or bm.faces.layers.int.new("molde")
uv_layer = bm.loops.layers.uv.get("atlas")
deform = bm.verts.layers.deform.active
seam_edges = [e for e in bm.edges if sum(1 for f in e.link_faces if f[part] == PART) == 1 and len(e.link_faces) == 2]
seam_verts = {v for e in seam_edges for v in e.verts}
strip = {f for f in bm.faces if f[part] == PART and any(v in seam_verts for v in f.verts)}
main = {f for f in bm.faces if f[part] == PART and f not in strip}
hem_loops = loops_of([e for e in bm.edges if sum(1 for f in e.link_faces if f in main) == 1])
seam_loops = loops_of(seam_edges)


def opening(center, heights):
    """A abertura de uma barra da peca, pelo centro dela (no Blender o x < 0 e a direita)."""
    if center.z >= max(heights) - 1e-9:
        return "gola" if PIECE == "camisa" else "cos"
    if PIECE == "camisa" and center.z <= min(heights) + 1e-9:
        return "barra"
    side = "direit" if center.x < 0.0 else "esquerd"
    return f"punho_{side}o" if PIECE == "camisa" else f"barra_{side}a"


# O molde das faixas do recuo de cada barra (a mesma ordem das barras e dos pares).
hem_centers = [centroid(h) for h in hem_loops]
strip_molde = [MOLDES.index("recuo_" + opening(c, [h.z for h in hem_centers])) for c in hem_centers]
material = next(iter(main)).material_index
strip_material = next(iter(strip)).material_index if strip else material

# A peca original (alvo da projecao, fonte do mapa e dos pesos), triangulada.
source = bm.copy()
source_part = source.faces.layers.int.get("parte")
bmesh.ops.delete(source, geom=[f for f in source.faces if f[source_part] != PART], context="FACES")
bmesh.ops.triangulate(source, faces=source.faces[:])
source.faces.ensure_lookup_table()
source_tree = BVHTree.FromBMesh(source)
source_uv = source.loops.layers.uv.get("atlas")
source_deform = source.verts.layers.deform.active
# A superficie principal original (sem as faixas, que dobram para dentro): o
# alvo das projecoes (as bordas e o relaxamento nao grudam na faixa).
surface = bm.copy()
surface_part = surface.faces.layers.int.get("parte")
surface_seam = {v for e in surface.edges if sum(1 for f in e.link_faces if f[surface_part] == PART) == 1 and len(e.link_faces) == 2 for v in e.verts}
bmesh.ops.delete(surface, geom=[f for f in surface.faces if f[surface_part] != PART or any(v in surface_seam for v in f.verts)], context="FACES")
bmesh.ops.triangulate(surface, faces=surface.faces[:])
surface.faces.ensure_lookup_table()
surface.normal_update()
surface_tree = BVHTree.FromBMesh(surface)
ORIGIN = [None]
# A curva de Phong em cada vertice: inteira onde as faces em volta concordam,
# nenhuma num vinco fechado (no vinco concavo da frente do calcao as duas
# laterais curvadas se cruzavam).
bend = {}
for v in surface.verts:
    spread = min((v.normal.dot(f.normal) for f in v.link_faces), default=1.0)
    t = min(1.0, max(0.0, (spread - 0.5) / (0.866 - 0.5)))
    bend[v] = 0.0 if FINAL else 0.75 * t * t * (3.0 - 2.0 * t)


def smooth_point(p):
    """O ponto na forma lisa da peca: a tesselacao de Phong da superficie
    original (passa pelos vertices dela e curva entre eles pelas normais; os
    planos grandes da peca antiga viravam facetas na malha densa). Guarda em
    ORIGIN[0] o ponto na superficie plana (de onde vem o mapa e os pesos)."""
    location, _, index, _ = surface_tree.find_nearest(p)
    ORIGIN[0] = location
    corners = [loop.vert for loop in surface.faces[index].loops]
    shares = barycentric(location, *(c.co for c in corners))
    curved = mathutils.Vector()
    for share, c in zip(shares, corners):
        curved += share * (location - c.normal * (location - c.co).dot(c.normal))
    return location.lerp(curved, sum(share * bend[c] for share, c in zip(shares, corners)))

# 2. A superficie principal remalhada: pelos moldes da roupa ou pelo QuadriFlow.
piece_obj = None
if REMESHER == "moldes":
    from retopo_moldes import moldes  # noqa: E402
    new, molde_info = moldes(PIECE, surface, hem_loops, bpy.data.objects["Esqueleto"], DENSITY, seam_loops)
    print(f"  moldes: {molde_info.get('divisoes')}")
else:
    mesh_copy = body.data.copy()
    piece_obj = bpy.data.objects.new("PecaNova", mesh_copy)
    bpy.context.collection.objects.link(piece_obj)
    tmp = bmesh.new()
    tmp.from_mesh(mesh_copy)
    tmp_part = tmp.faces.layers.int.get("parte")
    tmp_seam = {v for e in tmp.edges if sum(1 for f in e.link_faces if f[tmp_part] == PART) == 1 and len(e.link_faces) == 2 for v in e.verts}
    bmesh.ops.delete(tmp, geom=[f for f in tmp.faces if f[tmp_part] != PART or any(v in tmp_seam for v in f.verts)], context="FACES")
    tmp.to_mesh(mesh_copy)
    tmp.free()
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    bpy.context.view_layer.objects.active = piece_obj
    piece_obj.select_set(True)
    # O espelho em X do Blender (o y do modelo, esquerda/direita): sem ele o
    # use_mesh_symmetry nao faz nada e a peca sai torta (o gancho com a esquerda e a
    # direita diferentes a 1 cm).
    mesh_copy.use_mirror_x = True
    bpy.ops.object.quadriflow_remesh(target_faces=TARGET, use_preserve_sharp=False, use_preserve_boundary=True,
                                     use_mesh_symmetry=True, smooth_normals=False, mode="FACES", seed=0)
    new = bmesh.new()
    new.from_mesh(piece_obj.data)
    # Com simetria o QuadriFlow remalha cada metade e devolve as duas soltas (a
    # borda do corte no meio): o corte vai ao x 0 e as metades se soldam.

cut = [v for v in new.verts if v.is_boundary and abs(v.co.x) < 0.003]
for v in cut:
    v.co.x = 0.0
bmesh.ops.remove_doubles(new, verts=cut, dist=1e-4)
new_loops = loops_of([e for e in new.edges if e.is_boundary])
# O par de cada vertice no espelho (a saida e simetrica a 0,01 mm; o meio no x 0).
from mathutils.kdtree import KDTree
new_list = list(new.verts)
mirror_tree = KDTree(len(new_list))
for i, v in enumerate(new_list):
    mirror_tree.insert(v.co, i)
mirror_tree.balance()
partner = {}
for v in new_list:
    _, i, d = mirror_tree.find(mathutils.Vector((-v.co.x, v.co.y, v.co.z)))
    if d > 1e-4:
        sys.exit(f"QuadriFlow sem simetria: vertice sem par a {d * 1000:.2f} mm")
    partner[v] = new_list[i]


def symmetric(pairs_of, positions=None):
    """Cada par no espelho exato (o lado direito, x < 0 no Blender, manda; o meio no x 0)."""
    for v, p in pairs_of.items():
        if p is v:
            v.co.x = 0.0
        elif v.co.x < 0.0:
            p.co = mathutils.Vector((-v.co.x, v.co.y, v.co.z))


symmetric(partner)
if len(new_loops) != len(hem_loops):
    sys.exit(f"bordas: {len(new_loops)} na peca nova, {len(hem_loops)} na original")

# 3. Cada borda nova projetada na linha da barra original.
pairs = []
# A origem de cada vertice novo: o ponto da peca original de onde vem o mapa e
# os pesos (a forma lisa e a membrana afastam o vertice ate centimetros; pelo
# mais perto da posicao final, o mapa caia do outro lado do V, noutra ilha).
origin = {}
for hem in hem_loops:
    candidate = min(new_loops, key=lambda loop: (centroid(loop) - centroid(hem)).length)
    new_loops.remove(candidate)
    line = [v.co.copy() for v in hem]
    # A barra fica na linha original (fora da forma lisa): com ela curvada, a
    # faixa da gola perdia a folga e virava do avesso em quase toda pose.
    for v in candidate:
        v.co = nearest_on_polyline(v.co, line)
        smooth_point(v.co)
        origin[v] = ORIGIN[0]
    seam = min(seam_loops, key=lambda loop: (centroid(loop) - centroid(hem)).length)
    pairs.append((candidate, seam))

border_new = {v for candidate, _ in pairs for v in candidate}
for v in new.verts:
    if v not in border_new:
        v.co = smooth_point(v.co)
        origin[v] = ORIGIN[0]
# (Sobre a peca final, a malha grossa um pouco para fora: as faces planas entre
# pontos da superficie ficam abaixo dela, e no lado da gola a camisa passa a
# 6 mm do anel da pele - a faixa e a pele apareciam por cima.)
if FINAL and REMESHER == "moldes":
    for v in new.verts:
        if v not in border_new:
            location, normal, _, _ = surface_tree.find_nearest(v.co)
            if normal is not None:
                v.co = v.co + normal.normalized() * FINAL_LIFT
# E o alisamento de Taubin (sem encolher): o vinco em V da frente do calcao,
# que o Phong deixa reto, vira um vale; e as bossas do Phong nos triangulos
# grandes da peca original somem.
inside = [v for v in new.verts if v not in border_new and v.link_edges]
for step in [0.5, -0.53] * (0 if FINAL else FAIRING):
    moved = {v: v.co + step * (sum((e.other_vert(v).co for e in v.link_edges), mathutils.Vector()) / len(v.link_edges) - v.co)
             for v in inside}
    for v, q in moved.items():
        v.co = q
# No calcao, o gancho vira uma membrana (Laplace com a volta parada, na faixa
# do meio abaixo da barra da camisa, sumindo para os lados e para cima; no
# Blender o x e a esquerda/direita do modelo; ate a cintura recuada do calcao,
# ela puxava as costas para dentro; abaixo do fundo do gancho ela pegava a
# parte de dentro das pernas ate a barra): a peca original seguia o
# V da virilha do corpo (um entalhe na frente e um amassado em Y por baixo);
# o pano de verdade passa por cima, numa sela lisa.
membrane = {}
if PIECE == "calcao" and not FINAL:
    for v in inside:
        back = min(1.0, max(0.0, (v.co.y + 0.02) / 0.04))
        ceiling = -0.03 + 0.02 * back
        t = min(1.0, max(0.0, (0.065 - abs(v.co.x)) / 0.035)) * min(1.0, max(0.0, (ceiling - v.co.z) / 0.03)) \
            * min(1.0, max(0.0, (v.co.z + 0.24) / 0.03))
        if t > 0.0:
            membrane[v] = t * t * (3.0 - 2.0 * t)
    for _ in range(300):
        moved = {v: v.co + 0.5 * w * (sum((e.other_vert(v).co for e in v.link_edges), mathutils.Vector()) / len(v.link_edges) - v.co)
                 for v, w in membrane.items()}
        for v, q in moved.items():
            v.co = q
    # As nadegas: a membrana tira o V do meio e com ele o relevo - o fundo
    # virava um painel liso ("sem bunda"). Duas bossas, uma de cada lado do
    # meio (inteiras a 6,5 cm dele e 12 cm abaixo da pelve), empurram o pano
    # para fora pela normal (ate GLUTE); no meio e na cintura, nada.
    GLUTE = 0.022
    new.normal_update()
    for v in inside:
        if v.co.y <= 0.0:
            continue
        across = math.exp(-((abs(v.co.x) - 0.065) / 0.045) ** 2)
        height = math.exp(-((v.co.z + 0.12) / 0.06) ** 2)
        back = min(1.0, v.co.y / 0.04)
        v.co = v.co + v.normal * (GLUTE * across * height * back)
    # (e alisadas: pela normal de cada vertice o empurrao repetia o ruido da
    # malha e o sombreado ficava calombento)
    region = [v for v in inside if v.co.y > 0.0 and -0.25 < v.co.z < 0.0]
    for step in [0.5, -0.53] * 8:
        moved = {v: v.co + step * (sum((e.other_vert(v).co for e in v.link_edges), mathutils.Vector()) / len(v.link_edges) - v.co)
                 for v in region}
        for v, q in moved.items():
            v.co = q
    # A costura entre as pernas: um sulco suave (GROOVE no meio, 1,2 cm de
    # largura) na linha do meio abaixo de 9 cm sob a pelve - a frente, o fundo e
    # atras; sem ele o gancho liso fazia das duas pernas um bloco so.
    GROOVE = 0.008
    new.normal_update()
    for v in inside:
        depth = GROOVE * math.exp(-(v.co.x / 0.012) ** 2) * min(1.0, max(0.0, (-0.09 - v.co.z) / 0.04))
        if depth > 1e-5:
            v.co = v.co - v.normal * depth



# 4. Sai a peca antiga (principal e faixas); entra a nova; as faixas novas em ziper.
bmesh.ops.delete(bm, geom=list(main | strip), context="FACES_ONLY")
bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces and v not in seam_verts], context="VERTS")
copy_of = {v: bm.verts.new(v.co) for v in new.verts}
origin = {copy_of[v]: p for v, p in origin.items()}
added, strip_faces = [], []
new_molde = new.faces.layers.int.get("molde")
for f in new.faces:
    try:
        face = bm.faces.new([copy_of[v] for v in f.verts])
    except ValueError:
        continue
    face[part], face.material_index, face.smooth = PART, material, True
    face[molde] = f[new_molde] if new_molde is not None else 0
    added.append(face)
# As fileiras junto as bordas relaxadas (a media com os vizinhos, de volta na
# superficie original): o canto do V da gola desdobra.
border = {copy_of[v] for candidate, _ in pairs for v in candidate}
# (Nos moldes nao: a grade ja sai relaxada no pedaco certo da superficie, e a
# volta a superficie inteira levava os pontos de baixo da manga para o tronco,
# na axila.)
if border and REMESHER != "moldes":
    from mathutils.kdtree import KDTree
    border_tree = KDTree(len(border))
    for i, v in enumerate(border):
        border_tree.insert(v.co, i)
    border_tree.balance()
    # (os vertices da membrana do gancho ficam como ela os deixou: projetados
    # de volta na forma original, o degrau do cos atras voltava)
    in_membrane = {copy_of[v] for v in membrane}
    near = [v for v in set(copy_of.values()) - border - in_membrane if border_tree.find(v.co)[2] < 0.03]
    for _ in range(6):
        moved = {}
        for v in near:
            ns = [e.other_vert(v) for e in v.link_edges]
            if ns:
                moved[v] = v.co.lerp(sum((u.co for u in ns), mathutils.Vector()) / len(ns), 0.5)
        for v, p in moved.items():
            v.co = smooth_point(p)
            origin[v] = ORIGIN[0]
print(f"  desembaraco: {untangle(bm, set(copy_of.values()) - border)} rodada(s)")
symmetric({copy_of[v]: copy_of[p] for v, p in partner.items()})
for v, p in partner.items():
    if copy_of[v] in origin and copy_of[p] in origin and v.co.x < 0.0:
        o = origin[copy_of[v]]
        origin[copy_of[p]] = mathutils.Vector((-o.x, o.y, o.z))
# O resto da malha (a pele, a outra peca, a peca nova) triangulado como o
# exportador triangula: o que as faixas novas nao podem cruzar.
flat = bm.copy()
bmesh.ops.triangulate(flat, faces=[f for f in flat.faces if len(f.verts) > 3])
other_tris = [tuple(f.verts) for f in flat.faces]
flat.verts.index_update()
others = BVHTree.FromPolygons([v.co.copy() for v in flat.verts], [[v.index for v in tri] for tri in other_tris])
for (candidate, seam), label in zip(pairs, strip_molde):
    outer = [copy_of[v] for v in candidate]
    # O ziper otimo, do melhor dos comecos perto do vertice mais perto.
    inner = aligned(list(seam), outer)
    if len(inner) == len(outer):
        # (Com o mesmo numero de pontos dos dois lados - os moldes casam a barra
        # com o anel da pele -, uma faixa de quads: o giro que deixa cada par
        # mais perto.)
        n = len(outer)
        shift = min(range(n), key=lambda r: sum((outer[k].co - inner[(k + r) % n].co).length for k in range(n)))
        for k in range(n):
            a, b, c, e = outer[k], outer[(k + 1) % n], inner[(k + 1 + shift) % n], inner[(k + shift) % n]
            # (Em dois triangulos pela diagonal mais curta: no canto da gola o
            # quad da faixa e torto, e a outra diagonal furava a camisa.)
            tris = [(a, b, c), (a, c, e)] if (a.co - c.co).length <= (b.co - e.co).length else [(a, b, e), (b, c, e)]
            for tri in tris:
                try:
                    face = bm.faces.new(tri)
                except ValueError:
                    continue
                face[part], face.material_index, face.smooth = PART, strip_material, True
                face[molde] = label
                strip_faces.append(face)
        continue
    best = None
    for shift in range(-3, 4):
        trial = zipper(outer, inner[shift:] + inner[:shift], others, other_tris)
        if best is None or trial[1] < best[1]:
            best = trial
    tris, _, crossed = best
    if crossed:
        print(f"  aviso: {crossed} triangulo(s) da costura ainda cruzam a malha")
    for tri in tris:
        if len(set(tri)) < 3:
            continue
        try:
            face = bm.faces.new(tri)
        except ValueError:
            continue
        face[part], face.material_index, face.smooth = PART, strip_material, True
        face[molde] = label
        strip_faces.append(face)
bm.normal_update()
bmesh.ops.recalc_face_normals(bm, faces=added + strip_faces)
# (Nos moldes, a folga: a malha grossa, plana entre pontos da superficie,
# afundava na pele logo abaixo, no canto da gola; os vertices dos triangulos que
# cruzam saem pela normal ate nao cruzar, e os vizinhos a 3,5 cm saem junto,
# menos quanto mais longe: um domo baixo, sem chifre.)
if REMESHER == "moldes":
    pairs_of = {copy_of[v]: copy_of[p] for v, p in partner.items()}
    rounds, left = push_clear(bm, set(copy_of.values()) - border, step=0.001, limit=0.02, mirror=pairs_of, reach=0.035)
    print(f"  folga: {rounds} rodada(s), {left} vertice(s) ainda cruzando")


# 5. O mapa (dentro da ilha do atlas de cada face) e os pesos.
island = {}
for tri in source.faces:
    if tri.index in island:
        continue
    label = len(set(island.values()))
    stack = [tri]
    island[tri.index] = label
    while stack:
        here = stack.pop()
        for loop in here.loops:
            edge = loop.edge
            for other in edge.link_faces:
                if other is here or other.index in island:
                    continue
                same = True
                for vert in edge.verts:
                    uv_here = next(l[source_uv].uv for l in here.loops if l.vert is vert)
                    uv_other = next(l[source_uv].uv for l in other.loops if l.vert is vert)
                    same = same and (uv_here - uv_other).length < 1e-5
                if same:
                    island[other.index] = label
                    stack.append(other)
island_trees = {}
for label in set(island.values()):
    faces = [f for f in source.faces if island[f.index] == label]
    verts = list({v for f in faces for v in f.verts})
    position = {v: i for i, v in enumerate(verts)}
    island_trees[label] = (BVHTree.FromPolygons([v.co for v in verts], [[position[v] for v in f.verts] for f in faces]), faces)
# As bordas de cada ilha no mapa, com a normal para dentro.
INSET = 4.0 / 2048.0
island_edges = {label: [] for label in island_trees}
for tri in source.faces:
    corners_uv = {l.vert: l[source_uv].uv.copy() for l in tri.loops}
    middle = sum(corners_uv.values(), mathutils.Vector((0.0, 0.0))) / 3.0
    for loop in tri.loops:
        edge = loop.edge
        inside = False
        for other in edge.link_faces:
            if other is tri or island.get(other.index) != island[tri.index]:
                continue
            inside = all((next(l[source_uv].uv for l in other.loops if l.vert is v) - corners_uv[v]).length < 1e-5 for v in edge.verts)
        if inside:
            continue
        a, b = (corners_uv[v] for v in edge.verts)
        along = b - a
        if along.length < 1e-9:
            continue
        normal = mathutils.Vector((-along.y, along.x)).normalized()
        if normal.dot(middle - a) < 0.0:
            normal = -normal
        island_edges[island[tri.index]].append((a, b, normal))


def inset(uv, edges):
    for _ in range(2):
        best = None
        for a, b, normal in edges:
            ab = b - a
            t = max(0.0, min(1.0, (uv - a).dot(ab) / ab.dot(ab)))
            d = (uv - (a + ab * t)).length
            if best is None or d < best[0]:
                best = (d, normal)
        if best is None or best[0] >= INSET:
            return uv
        uv = uv + best[1] * (INSET - best[0])
    return uv


for face in added + strip_faces:
    # A ilha pelo voto dos cantos e do centro (uma face no fim da faixa
    # escolhia pelo centro a ilha branca no meio da preta).
    votes = {}
    for p in [origin.get(v, v.co) for v in face.verts] + [sum((origin.get(v, v.co) for v in face.verts), mathutils.Vector()) / len(face.verts)]:
        label = island[source_tree.find_nearest(p)[2]]
        votes[label] = votes.get(label, 0) + 1
    tree, faces = island_trees[max(votes, key=votes.get)]
    corners = []
    for loop in face.loops:
        location, _, local, _ = tree.find_nearest(origin.get(loop.vert, loop.vert.co))
        tri = faces[local]
        a, b, c = (l.vert.co for l in tri.loops)
        uvs = [l[source_uv].uv.copy() for l in tri.loops]
        u, v, w = barycentric(location, a, b, c)
        corners.append(uvs[0] * u + uvs[1] * v + uvs[2] * w)
    # O canto que cai a menos de INSET da borda da ilha volta para dentro: na
    # borda o filtro da textura pegava o contorno escuro do atlas (no gancho,
    # onde a membrana espalhou vertices que vem de uma tira junto da borda).
    for loop, uv in zip(face.loops, corners):
        loop[uv_layer].uv = inset(uv, island_edges[max(votes, key=votes.get)])
for vert in set(copy_of.values()):
    location, _, index, _ = source_tree.find_nearest(origin.get(vert, vert.co))
    tri = source.faces[index]
    corners = [loop.vert for loop in tri.loops]
    u, v, w = barycentric(location, *(c.co for c in corners))
    mixed = {}
    for corner, share in zip(corners, (u, v, w)):
        for group, weight in corner[source_deform].items():
            mixed[group] = mixed.get(group, 0.0) + weight * max(share, 0.0)
    ranked = sorted(mixed.items(), key=lambda item: -item[1])[:4]
    total = sum(weight for _, weight in ranked) or 1.0
    for group in list(vert[deform].keys()):
        del vert[deform][group]
    for group, weight in ranked:
        vert[deform][group] = weight / total

# A origem de cada vertice fica na malha (atributo "origem"): as etapas
# seguintes buscam o mapa e os pesos pelo ponto da peca original.
origin_layer = bm.verts.layers.float_vector.get("origem")
if origin_layer is None:
    origin_layer = bm.verts.layers.float_vector.new("origem")
    for v in bm.verts:
        v[origin_layer] = v.co
for v in bm.verts:
    if v in origin:
        v[origin_layer] = origin[v]
    elif v[origin_layer].length == 0.0:
        v[origin_layer] = v.co
bm.to_mesh(body.data)
body.data.update()
names = bpy.data.texts.get("matter_moldes") or bpy.data.texts.new("matter_moldes")
names.from_string(json.dumps(MOLDES))
names.use_fake_user = True
if piece_obj is not None:
    bpy.data.objects.remove(piece_obj, do_unlink=True)
print(f"{PIECE.upper()} {REMESHER.upper()}: {len(added)} faces novas, {len(strip_faces)} na costura; {len(pairs)} bordas; "
      f"malha {len(bm.verts)} vertices, {len(bm.faces)} faces")
bpy.ops.wm.save_as_mainfile(filepath=argv[0] if argv else bpy.data.filepath)
