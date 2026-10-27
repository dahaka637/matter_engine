"""A parede de dentro das pernas do calcao seguindo a coxa (23/10/2026, pedido do usuario: com as pernas abertas o
meio do calcao virava uma cortina entre as coxas; ele mandou a CORRETO/ERRADO e pediu a pose idle de referencia).

Os pesos da etapa 02 deixavam a parede de dentro de cada perna, perto do gancho, com metade da coxa e um pouco da
HipLift do outro lado (a divisoria da virilha): com a perna aberta ela ficava com a pelve. Aqui ela ganha um piso da
coxa do lado (inteira 5 cm abaixo do gancho); so a faixa do meio (BAND) fica com a divisoria, e a dobra de baixo do
gluteo (onde o GluteHelper pesa) fica com o corretivo dela. O assado e o desvirar mantem o piso (--calcao-coxa).
O vao entre as pernas era um U (a borda de dentro subia reta e arredondava so nos ultimos 3 cm): ela passa a subir em
linha reta da barra ate a ponta do gancho (v_crotch; a altura do gancho e a barra ficam). (Prender a faixa da costura nos pesos do meio foi testado e desfeito: o avesso foi de 12 a 59.)
Com as pernas abertas o vao
fica um V: so a costura do meio fica com a divisoria. E a frente do meio acima do gancho
segue a flexao das duas coxas como os lados (front_middle_floor), senao o leque da braguilha amassa num losango.
(Subir o gancho de 0,77 para 0,83 m foi testado e desfeito: "nao era para ficar com o cavalo mais alto, o problema
era como a malha se comportava".)
"""
import numpy as np

import proporcao_tronco

HEM = 0.70              # m do chao: a barra das pernas e a dobra dela (0,658 a 0,698)
BAND = 0.008            # m de |y|: so a costura do meio fica com a divisoria (a faixa de 4,5 cm da etapa 02 arredondava
                        # o vao entre as pernas abertas num U; ele pediu um V)
THIGH_FADE = (0.0, 0.015)   # m abaixo do gancho: a parede de dentro segue a coxa inteira logo abaixo dele
INNER = (0.06, 0.09)        # m de |y|: so o lado de dentro da perna (o de fora ja e da coxa)
GLUTE = (0.02, 0.15)        # o peso do GluteHelper que poupa o vertice (a dobra de baixo do gluteo, o corretivo da extensao)
# A frente do meio acima do gancho (a braguilha): seguia a pelve e os lados as HipLift (a flexao); com as duas coxas
# dobradas (o idle) os lados subiam e o meio ficava, amassando o leque num losango. O meio passa a ter a parte das
# HipLift dos lados (metade de cada uma).
V_DEPTH = 0.11           # m: a reta do V vai da barra (o gancho - isto) ate a ponta
V_REACH = 0.04          # m de |y| alem da borda de dentro: ate onde a perna acompanha (some ate la)
FLY_ROUNDS = 8          # o alisamento (Taubin) do leque da braguilha: afundado 3 mm no polo, fazia o losango
FLY_REGION = (0.03, 0.07, 0.03, 0.955)   # x minimo (frente), |y| maximo, m acima do gancho, m do chao (abaixo da camisa)
FRONT_LIFT = 0.5           # a soma das duas HipLift no meio (a dos lados, 0,46 a 0,54)
FRONT_SIDE = (0.02, 0.05)  # m de |y|: inteiro no meio, some ate os lados (que ja tem)
FRONT_HEIGHT = (0.03, 0.95)  # m: de 3 cm acima do gancho ate a barra da camisa


def _ramp(x, a, b):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def shorts_vertices(tris, tri_part, names):
    return np.unique(tris[tri_part == names.index("calcao")])


def crotch_height(pos, shorts, ground):
    """O fundo do U do gancho (m do chao): o ponto mais baixo do meio do calcao entre as pernas."""
    middle = shorts[(np.abs(pos[shorts, 1]) < 0.004) & (pos[shorts, 2] - ground > HEM + 0.03)]
    return float((pos[middle, 2] - ground).min())


def inner_leg_floor(W, pos, tris, tri_part, names, joints, ground):
    """A parede de dentro de cada perna abaixo do gancho: os vertices, o piso do peso da coxa do lado (0 a 1) e a coxa.
    A dobra de baixo do gluteo fica com o corretivo dela (o peso do GluteHelper de W poupa o vertice)."""
    shorts = shorts_vertices(tris, tri_part, names)
    top = crotch_height(pos, shorts, ground)
    h = pos[shorts, 2] - ground
    y = np.abs(pos[shorts, 1])
    glute = W[shorts][:, [joints.index("RightGluteHelper"), joints.index("LeftGluteHelper")]].max(1)
    floor = (_ramp(top - h, *THIGH_FADE) * (y > BAND) * (1.0 - _ramp(y, *INNER)) * (1.0 - _ramp(glute, *GLUTE)))
    keep = floor > 1e-3
    thigh = np.where(pos[shorts, 1] < 0.0, joints.index("RightThigh"), joints.index("LeftThigh"))
    return shorts[keep], floor[keep], thigh[keep]


def apply_floor(W, rows, floor, thigh):
    """Cada vertice com pelo menos 'floor' da coxa; as outras juntas encolhem na mesma proporcao (a soma fica 1).
    Devolve quantos mudaram."""
    changed = 0
    for v, f, j in zip(rows, floor, thigh):
        if W[v, j] >= f - 1e-9:
            continue
        rest = 1.0 - W[v, j]
        W[v] *= (1.0 - f) / rest if rest > 1e-12 else 0.0
        W[v, j] = f
        changed += 1
    return changed


def front_middle_floor(W, pos, tris, tri_part, names, joints, ground):
    """A frente do meio acima do gancho: os vertices e o piso da soma das duas HipLift (metade de cada)."""
    shorts = shorts_vertices(tris, tri_part, names)
    top = crotch_height(pos, shorts, ground)
    h = pos[shorts, 2] - ground
    floor = (FRONT_LIFT * (1.0 - _ramp(np.abs(pos[shorts, 1]), *FRONT_SIDE))
             * (h > top + FRONT_HEIGHT[0]) * (h < FRONT_HEIGHT[1]) * (pos[shorts, 0] > 0.02))
    keep = floor > 1e-3
    return shorts[keep], floor[keep], (joints.index("LeftHipLift"), joints.index("RightHipLift"))


def apply_pair_floor(W, rows, floor, pair):
    """Cada vertice com pelo menos 'floor' na soma das duas juntas (a falta dividida meio a meio); as outras encolhem
    na mesma proporcao. Devolve quantos mudaram."""
    a, b = pair
    changed = 0
    for v, f in zip(rows, floor):
        have = W[v, a] + W[v, b]
        if have >= f - 1e-9:
            continue
        rest = 1.0 - have
        keep_a, keep_b = W[v, a], W[v, b]
        W[v] *= (1.0 - f) / rest if rest > 1e-12 else 0.0
        W[v, a] = keep_a + 0.5 * (f - have)
        W[v, b] = keep_b + 0.5 * (f - have)
        changed += 1
    return changed


def _inner_edge(pos, tris, shorts_tris, z):
    """A menor |y| do calcao cortado no plano z (fora da costura do meio): a borda de dentro do vao."""
    d = pos[:, 2] - z
    a, b = shorts_tris, np.roll(shorts_tris, -1, axis=1)
    cross = (d[a] < 0) != (d[b] < 0)
    t = d[a][cross] / (d[a][cross] - d[b][cross])
    y = np.abs(pos[a][cross][:, 1] + t * (pos[b][cross][:, 1] - pos[a][cross][:, 1]))
    y = y[y > 1e-4]
    return float(y.min()) if len(y) else 0.0


def v_crotch(pos, tris, tri_part, names, ground):
    """O vao entre as pernas em V: a borda de dentro de cada perna em linha reta da barra ate a ponta do gancho; o que
    fica entre as bordas (a sela) aperta na mesma proporcao e a perna acompanha ate V_REACH alem da borda."""
    pos = pos.copy()
    shorts = shorts_vertices(tris, tri_part, names)
    own = tris[tri_part == names.index("calcao")]
    apex = crotch_height(pos, shorts, ground) + ground
    base = apex - V_DEPTH
    width = _inner_edge(pos, tris, own, base)
    heights = np.linspace(base, apex - 0.002, 40)
    edges = np.array([_inner_edge(pos, tris, own, z) for z in heights])
    h = pos[shorts, 2]
    inside = (h > base) & (h < apex)
    rows = shorts[inside]
    edge = np.interp(pos[rows, 2], heights, edges)
    target = np.minimum(edge, width * (apex - pos[rows, 2]) / V_DEPTH)
    y = np.abs(pos[rows, 1])
    shift = edge - target
    new = np.where(y <= edge, y * target / np.maximum(edge, 1e-6), y - shift * (1.0 - _ramp(y - edge, 0.0, V_REACH)))
    pos[rows, 1] = np.sign(pos[rows, 1]) * new
    return pos, float(width), float(np.max(shift)) if len(shift) else 0.0


def smooth_fly(pos, tris, tri_part, names, ground):
    """O leque da braguilha (a frente do meio, acima do gancho e abaixo da barra da camisa) alisado: o polo dele
    ficava afundado e o leque aparecia como um losango, parado e mais ainda com as coxas dobradas."""
    shorts = shorts_vertices(tris, tri_part, names)
    own = tris[tri_part == names.index("calcao")]
    seams = np.intersect1d(shorts, np.unique(tris[tri_part != names.index("calcao")]))
    top = crotch_height(pos, shorts, ground)
    h = pos[shorts, 2] - ground
    x0, y0, above, ceiling = FLY_REGION
    movable = np.zeros(len(pos), bool)
    movable[shorts[(pos[shorts, 0] > x0) & (np.abs(pos[shorts, 1]) < y0) & (h > top + above) & (h < ceiling)]] = True
    movable[seams] = False
    before = pos.copy()
    pos = proporcao_tronco.fair(pos, own, movable, FLY_ROUNDS)
    pos, _ = proporcao_tronco.relax_folds(pos, before, own, movable)
    return pos, int(movable.sum()), float(np.max(np.linalg.norm(pos - before, axis=1)))
