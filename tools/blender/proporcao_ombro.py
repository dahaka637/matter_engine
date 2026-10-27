"""O ombro e a manga da camisa na proporcao humana (23/10/2026).

O usuario achou o ombro "horrivel" (uma ombreira) comparado a uma imagem de referencia. Medido
na malha: a casca do ombro ficava a 7,5-8 cm da junta do ombro em cima e para fora e a 8-9,5 cm
na frente e atras; a manga tinha 8,8 cm de raio junto ao ombro, afinando ate 6,5 cm perto da
barra. No humano (ANSUR II, 1,80 m: o biacromial de 41,6 cm, o bideltoide de 48,7 cm, a altura do
acromio 1,473 m) e na referencia, o deltoide fica a ~5,3 cm da junta e a manga justa no braco.

A camisa em volta da junta encolhe no raio: na manga (abaixo da junta, ao longo do braco), so a
distancia ao eixo do braco (o comprimento da manga fica); acima da junta, a distancia a ela
(esferico). O fator sai da propria malha: o raio mediano de cada faixa ao longo do braco ate o raio
alvo. O lado virado para o tronco (a axila, a dobra) nao encolhe perto da junta; perto da gola, a barra
da manga e o que fica longe da junta tambem nao. O campo e alisado pela malha, as dobras que
sobrarem relaxam, e o lado esquerdo e o espelho exato.
"""
import numpy as np
import scipy.sparse as sp
from scipy.spatial import cKDTree

import proporcao_tronco as tronco

SLEEVE_RADIUS = 0.057      # o raio mediano alvo da manga (m), do ombro ate perto da barra
CAP_RADIUS = 0.054         # a distancia alvo da casca do ombro acima da junta (m)
HEM_FADE = (0.100, 0.125)  # ao longo do braco: as faixas de raio medidas ate aqui
TAPER = (0.04, 0.110)      # ao longo do braco: o raio alvo ate o primeiro, o raio da barra no segundo
REACH = (0.11, 0.15)       # distancia ao eixo (ou a junta): inteiro ate o primeiro, nada depois do segundo
MEDIAL = (0.05, 0.01)      # a capa: nada a mais de 5 cm para dentro da junta, inteira a menos de 1 cm
COLLAR = (0.030, 0.110)    # distancia a faixa da gola: nada ate o primeiro, inteiro depois do segundo
INNER = (0.25, 0.65)       # cosseno com a direcao do tronco: inteiro ate o primeiro, nada depois
FREE_TUBE = (0.04, 0.08)   # ao longo do braco: a partir dai a manga e um tubo solto (o lado de dentro encolhe)
SMOOTH_ROUNDS = 24         # rodadas de alisamento do deslocamento


def _ramp(x, a, b):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def shape_shoulder(pos, tris, part_of_tri, names, joint_positions, ground):
    """As posicoes novas (so a camisa em volta dos ombros muda) e um resumo."""
    shirt = names.index("camisa")
    shirt_tris = part_of_tri == shirt
    on_shirt = np.zeros(len(pos), bool)
    on_shirt[tris[shirt_tris]] = True
    on_other = np.zeros(len(pos), bool)
    on_other[tris[~shirt_tris]] = True
    seam = on_shirt & on_other
    strip = np.zeros(len(pos), bool)
    strip[tris[shirt_tris & seam[tris].any(1)]] = True      # as faixas costuradas (gola, punho, barra) ficam
    joint = np.asarray(joint_positions["RightUpperArm"])
    elbow = np.asarray(joint_positions["RightForearm"])
    along = (elbow - joint) / np.linalg.norm(elbow - joint)
    inward = np.array([0.0, 1.0, 0.0])
    inward -= (inward @ along) * along
    inward /= np.linalg.norm(inward)
    rel = pos - joint
    t = rel @ along
    radial = rel - t[:, None] * along
    r = np.linalg.norm(radial, axis=1)
    right = on_shirt & ~strip & (pos[:, 1] < -0.05)
    # O raio mediano de cada faixa de 1 cm ao longo do braco (o lado de fora, frente e tras).
    outer = right & ((radial @ inward) / np.maximum(1e-9, r) < INNER[0]) & (r < REACH[0])
    # O alvo: SLEEVE_RADIUS ate TAPER[0] ao longo do braco, abrindo ate o raio da barra em TAPER[1]
    # (a manga um pouco mais larga na barra, sem degrau no punho).
    bins = np.arange(-0.01, HEM_FADE[1] + 0.01, 0.01)
    medians = {}
    for b in bins:
        m = outer & (t >= b) & (t < b + 0.01)
        if m.sum() >= 3:
            medians[b] = np.median(r[m])
    hem_bins = [v for b, v in medians.items() if TAPER[1] - 0.015 <= b + 0.005 <= TAPER[1] + 0.005]
    hem_radius = float(np.median(hem_bins)) if hem_bins else SLEEVE_RADIUS
    centers, scales = [], []
    for b, med in medians.items():
        c = b + 0.005
        goal = np.interp(c, [TAPER[0], TAPER[1]], [SLEEVE_RADIUS, max(SLEEVE_RADIUS, hem_radius)])
        centers.append(c)
        scales.append(min(1.0, goal / med))
    scales = np.convolve(np.pad(scales, 1, mode="edge"), np.ones(3) / 3.0, mode="valid")
    cap = right & (t < 0.0) & (np.linalg.norm(rel, axis=1) < REACH[0]) & ((rel @ inward) / np.maximum(1e-9, np.linalg.norm(rel, axis=1)) < INNER[0])
    cap_scale = min(1.0, CAP_RADIUS / np.median(np.linalg.norm(rel[cap], axis=1))) if cap.sum() >= 3 else 1.0
    scale_t = np.where(t < 0.0, cap_scale, np.interp(t, centers, scales))
    # O peso de cada vertice.
    dist = np.where(t < 0.0, np.linalg.norm(rel, axis=1), r)
    direction = np.where((t < 0.0)[:, None], rel, radial)
    cos_in = (direction @ inward) / np.maximum(1e-9, np.linalg.norm(direction, axis=1))
    free = _ramp(t, FREE_TUBE[0], FREE_TUBE[1])
    w_dir = 1.0 - (1.0 - free) * _ramp(cos_in, INNER[0], INNER[1])
    collar = strip & (pos[:, 2] - ground > 1.45)
    from_collar = cKDTree(pos[collar]).query(pos)[0] if collar.any() else np.full(len(pos), 1.0)
    # (A capa encolhe so por fora da junta, o deltoide: o pano entre a junta e a gola, puxado para ela, deixava a gola
    # sobrar sobre o ombro e, com o pescoco de lado, ela mostrava o avesso. A reta de cima cuida dessa parte.)
    medial = np.where(t < 0.0, _ramp(np.abs(pos[:, 1]), abs(joint[1]) - MEDIAL[0], abs(joint[1]) - MEDIAL[1]), 1.0)
    weight = (right * w_dir * medial * (1.0 - _ramp(dist, REACH[0], REACH[1])) * _ramp(from_collar, COLLAR[0], COLLAR[1])
              * (1.0 - _ramp(t, TAPER[1], HEM_FADE[1])))
    s = 1.0 - (1.0 - scale_t) * weight
    target = joint + np.where((t < 0.0)[:, None], (t * s)[:, None] * along, t[:, None] * along) + radial * s[:, None]
    move = np.where(right[:, None], target - pos, 0.0)
    # O deslocamento alisado pela malha da camisa (as faixas e o resto ficam).
    rows = np.concatenate([tris[:, 0], tris[:, 1], tris[:, 2], tris[:, 1], tris[:, 2], tris[:, 0]])
    cols = np.concatenate([tris[:, 1], tris[:, 2], tris[:, 0], tris[:, 0], tris[:, 1], tris[:, 2]])
    adjacency = sp.coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(pos), len(pos))).tocsr()
    adjacency.data[:] = 1.0
    degree = np.maximum(1.0, np.asarray(adjacency.sum(1)).ravel())
    for _ in range(SMOOTH_ROUNDS):
        move = np.where(right[:, None], 0.5 * move + 0.5 * (adjacency @ move) / degree[:, None], 0.0)
    # (O alisamento espalha: junto da gola e da barra da manga o deslocamento volta a zero.)
    move *= (_ramp(from_collar, 0.02, COLLAR[0] + 0.02) * (1.0 - _ramp(t, TAPER[1], HEM_FADE[1])))[:, None]
    new = pos + move
    moved = right & (np.linalg.norm(move, axis=1) > 1e-4)
    zone = moved.copy()
    zone[tris[zone[tris].any(1)]] = True
    zone &= right
    new = tronco.fair(new, tris, zone, 10)
    new, relaxed = tronco.relax_folds(new, pos, tris[shirt_tris], zone)
    # O lado esquerdo: o espelho exato do direito.
    mirror_d, mirror = cKDTree(pos).query(pos * np.array([1.0, -1.0, 1.0]))
    left = (mirror_d < 1e-5) & right[mirror]
    new[left] = new[mirror[left]] * np.array([1.0, -1.0, 1.0])
    for _ in range(3):                                       # (o alisamento deixa resto; mais duas passadas o tiram)
        new, line = straighten_top(new, tris, part_of_tri, names, ground)
    info = {"linha": line, "movidos": int(moved.sum()), "fator_ombro": round(float(cap_scale), 3),
            "fator_manga": [round(float(x), 2) for x in scales], "rodadas_dobras": relaxed,
            "desloc_max_cm": round(float(np.linalg.norm(move, axis=1).max() * 100), 1)}
    return new, info


# A linha de cima do ombro reta (pedido do usuario, 23/10: da gola ate a ponta do ombro "algo mais
# linear", como a referencia): a crista de cima, vista de frente ou de costas, vira a reta entre o
# ponto junto da gola (TOP_LINE[0] do meio) e a ponta do ombro (TOP_LINE[1]); a correcao e vertical
# e vale inteira na crista, sumindo para baixo na frente e nas costas.
TOP_LINE = (0.140, 0.235)  # |y| dos dois pontos da reta (m): logo fora da gola ate a ponta do ombro
TOP_DEPTH = (0.02, 0.06)   # abaixo da crista: inteiro ate o primeiro, nada depois do segundo


def crest(pos, tris, mask_tris, bins):
    """A altura maxima da superficie (amostrada nos triangulos) em cada faixa de |y|."""
    rng = np.random.default_rng(0)
    a, b, c = pos[tris[mask_tris, 0]], pos[tris[mask_tris, 1]], pos[tris[mask_tris, 2]]
    top = np.full(len(bins) - 1, -np.inf)
    for _ in range(24):
        l = rng.dirichlet((1.0, 1.0, 1.0), len(a))
        p = l[:, :1] * a + l[:, 1:2] * b + l[:, 2:] * c
        k = np.digitize(np.abs(p[:, 1]), bins) - 1
        ok = (k >= 0) & (k < len(top)) & (p[:, 1] < 0.0)
        np.maximum.at(top, k[ok], p[ok, 2])
    return top


def straighten_top(pos, tris, part_of_tri, names, ground):
    """As posicoes com a crista de cima do ombro reta (o lado direito; o esquerdo e o espelho)."""
    shirt = names.index("camisa")
    shirt_tris = part_of_tri == shirt
    on_shirt = np.zeros(len(pos), bool)
    on_shirt[tris[shirt_tris]] = True
    on_other = np.zeros(len(pos), bool)
    on_other[tris[~shirt_tris]] = True
    seam = on_shirt & on_other
    strip = np.zeros(len(pos), bool)
    strip[tris[shirt_tris & seam[tris].any(1)]] = True
    bins = np.arange(TOP_LINE[0] - 0.01, TOP_LINE[1] + 0.03, 0.0025)
    centers = 0.5 * (bins[1:] + bins[:-1])
    top = crest(pos, tris, shirt_tris, bins)
    ok = np.isfinite(top)
    top = np.convolve(np.pad(np.where(ok, top, np.interp(centers, centers[ok], top[ok])), 2, mode="edge"), np.ones(5) / 5, mode="valid")
    y0, y1 = TOP_LINE
    h0, h1 = np.interp(y0, centers, top), np.interp(y1, centers, top)
    line = h0 + (h1 - h0) * (centers - y0) / (y1 - y0)
    delta = np.where((centers >= y0) & (centers <= y1), line - top, 0.0)
    ay = np.abs(pos[:, 1])
    dz = np.interp(ay, centers, delta)
    local_top = np.interp(ay, centers, top)
    # (Junto da gola nada: com o pescoco de lado, a gola descia por dentro do ombro.)
    collar = strip & (pos[:, 2] - ground > 1.45)
    from_collar = cKDTree(pos[collar]).query(pos)[0] if collar.any() else np.full(len(pos), 1.0)
    weight = ((on_shirt & ~strip & (pos[:, 1] < 0.0)) * (1.0 - _ramp(local_top - pos[:, 2], TOP_DEPTH[0], TOP_DEPTH[1]))
              * _ramp(from_collar, 0.03, 0.08))
    move = np.zeros_like(pos)
    move[:, 2] = dz * weight
    rows = np.concatenate([tris[:, 0], tris[:, 1], tris[:, 2], tris[:, 1], tris[:, 2], tris[:, 0]])
    cols = np.concatenate([tris[:, 1], tris[:, 2], tris[:, 0], tris[:, 0], tris[:, 1], tris[:, 2]])
    adjacency = sp.coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(pos), len(pos))).tocsr()
    adjacency.data[:] = 1.0
    degree = np.maximum(1.0, np.asarray(adjacency.sum(1)).ravel())
    free = on_shirt & ~strip & (pos[:, 1] < 0.0)
    for _ in range(6):
        move = np.where(free[:, None], 0.5 * move + 0.5 * (adjacency @ move) / degree[:, None], 0.0)
    move *= _ramp(from_collar, 0.02, 0.05)[:, None]
    new = pos + move
    mirror_d, mirror = cKDTree(pos).query(pos * np.array([1.0, -1.0, 1.0]))
    left = (mirror_d < 1e-5) & free[mirror]
    new[left] = new[mirror[left]] * np.array([1.0, -1.0, 1.0])
    return new, {"reta_de": round(float(h0 - ground), 3), "reta_ate": round(float(h1 - ground), 3),
                 "correcao_max_mm": round(float(np.abs(delta).max() * 1000), 1)}
