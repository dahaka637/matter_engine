"""A analise da marcha (05/11): os numeros da marcha, do corpo e do clipe (a referencia) nos mesmos quadros, para pegar o
defeito pelo numero e nao so pela imagem. Por captura (inspecao.captura), pulando a largada de pe (1,5 s) de cada episodio,
o levantar do chao (os quadros do clipe de levantar: no lev_levantar fica o de pe depois dele) e as quedas (do segundo antes
da pelve ficar abaixo de 45 cm ao fim do episodio; elas ficam contadas em episodios):
  - os pes: contato (a sola abaixo de 1,5 cm, a mesma regra no corpo e no clipe), pousos e saidas, os passos de cada lado
    (tempo, comprimento no rumo, largura), a passada, a cadencia, o apoio, o voo, a altura do pe no balanco (sola, ponta,
    calcanhar e a folga minima da ponta), o deslize do pe no apoio (o pivo da sola, como a firmeza do treino: cm por
    apoio e a fracao do apoio escorregando), os quiques (o pe pulando no meio do apoio, e a altura deles), os toques
    breves (bicudas), os tropecos (o pe de volta ao chao em menos de 0,12 s), o cruzamento e a distancia minima entre os pes;
  - os angulos: joelho, quadril (flexao e abducao), tornozelo, cotovelo, ombro; a inclinacao do tronco e do corpo inteiro
    (vies e balanco), a altura e o quique da pelve, a cabeca (giro e balanco), o centro de massa contra a base;
  - o deslocamento: a velocidade contra o pedido, o desvio do rumo e o rosto contra o pedido e contra o clipe;
  - a mao de fora (a forca auxiliar): o quanto ela segura (para cima) e empurra (no plano), em fracao do peso;
  - tudo de novo no clipe (a referencia, com a velocidade natural dele, a do pe de apoio) e a diferenca corpo - clipe; o
    erro de posicao de cada articulacao;
  - a simetria de cada medida por lado (SI = 2|E-D|/(|E|+|D|), nulo perto de zero ou com os lados de sinais opostos), o
    ciclo medio de cada perna desde o proprio pouso (101 pontos) e as comparacoes com o clipe puro (comparar) e com o
    exercicio espelhado (espelho: o esquerdo de um contra o direito do outro, os sinais de lado trocados).
Convencoes: graus, + = esquerda, o referencial do corpo pelo abdome (ver quadro()); o joelho e o cotovelo em flexao (0 =
esticado); o quadril e o ombro + = para a frente; a abducao + = para fora; o tornozelo + = dorsiflexao; o pe + = ponta para
cima. Nas curvas a inclinacao de lado inclui a de entrar na curva (o clipe da curva_s tambem inclina, -5,5 graus): o vies
da rede ali e o delta_referencia (corpo - clipe) ou o comparar contra o clipe puro do mesmo exercicio.
Saida: <saida>/<captura>.marcha.json, <captura>.ciclo.png, <captura>.series.png e <captura>.pegadas.png.

Uso:
  python -m inspecao.marcha A.npz [B.npz ...] --saida DIR [--contra CLIPE.npz] [--espelho OUTRO.npz]
  python -m inspecao.marcha --teste
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from inspecao import geometria as G

REPO = Path(__file__).resolve().parents[3]
AMOSTRAS = REPO / "build-profile/tmp/inspecao_amostras_0645"
SAIDA = REPO / "build-profile/tmp/inspecao_dev/marcha"
LARGADA_S = 1.5  # (a largada de pe de cada episodio fica fora da marcha ciclica)
CAIDO_M = 0.45  # (a pelve abaixo disso e queda; sem queda, a mais baixa das amostras e 0,51: o agachamento do giro de 180)
QUEDA_S = 1.0  # (de pe ate o chao a queda leva menos que isso - 0,5 s nas amostras)
# (A sola abaixo disso conta como no chao, no corpo e no clipe; o chao liso em z = 0, como os contatos das amostras. Os
# pontos de contato da fisica nao entram: vem ate 2,3 cm acima do chao, quase todos com impulso zero, e o clipe nao os tem
# - com eles o voo do corpo no trote da mao cheia caia de 21% para 11% so pela regra.)
TOQUE_M = 0.015
PISO_M = 0.01  # (o ponto da sola que apoia de fato, o SoleDown do treino: o pivo do deslize e o pe que leva a pelve do clipe)
ARRASTO_MS = 0.15  # (o pivo do pe no chao andando acima disso escorrega: a folga da firmeza do treino, SlipSlack)
TROPECO_S = 0.12  # (o pe que sai e volta ao chao em menos disso tropecou, o StumbleSeconds do treino)
PONTA_M = 0.06  # (os pontos da casca do pe com x local acima disso sao a ponta, abaixo de -isso o calcanhar: x + e a ponta)
PONTOS = 101
LADOS = (("e", "Left"), ("d", "Right"))
ARTICULACOES = ("pelve", "joelho_e", "joelho_d", "pe_e", "pe_d", "mao_e", "mao_d", "cabeca")
INCLINACOES = ("inclinacao_lado_corpo", "inclinacao_frente_corpo", "inclinacao_lado_tronco", "inclinacao_frente_tronco")
# (No espelho, o que tem lado no sinal: + = esquerda vira direita.)
LATERAIS = ("inclinacao_lado_corpo", "inclinacao_lado_tronco", "cm_lateral_cm", "desvio_rumo_graus", "rosto_rumo_graus",
            "rosto_clipe_graus")
FIM = {0: "tempo", 1: "caiu", 2: "desequilibrio", 3: "virado", 4: "desviou", 5: "outro", 6: "fora_do_percurso"}
COR = {"e": "#2a78d6", "d": "#eb6834"}
MUNDO = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")
TINTA, GRADE = "#52514e", "#e1e0d9"


# --- Numeros ----------------------------------------------------------------------------------------------------------

def _f(x):
    x = np.asarray(x, float).ravel()
    return x[np.isfinite(x)]


def _media(x):
    x = _f(x)
    return float(x.mean()) if x.size else None


def _dp(x):
    x = _f(x)
    return float(x.std()) if x.size > 1 else None


def _pct(x, q):
    x = _f(x)
    return float(np.percentile(x, q)) if x.size else None


def _amp(x):
    x = _f(x)
    return float(np.percentile(x, 95) - np.percentile(x, 5)) if x.size else None


def _vb(x):
    return {"vies": _media(x), "balanco": _dp(x)}


def _enrola(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _circ(rad):
    """A media e o desvio (graus) de angulos ja enrolados em [-pi, pi]."""
    r = _f(rad)
    if not r.size:
        return {"vies": None, "balanco": None}
    m = np.arctan2(np.sin(r).mean(), np.cos(r).mean())
    return {"vies": float(np.degrees(m)), "balanco": float(np.degrees(np.std(_enrola(r - m))))}


def _si(k, e, d):
    """SI = 2|E-D|/(|E|+|D|) da medida `k`; nenhum perto de zero (abaixo de 0,05 em s, m e fracao; de 2 em graus, cm e
    contagens), onde o SI so mede ruido, nem com os lados de sinais opostos (abducao, pouso de lado), onde ele da sempre 2."""
    if e is None or d is None or e * d < 0 or abs(e) + abs(d) < (0.05 if k.endswith(("_s", "_m", "_frac")) else 2.0):
        return None
    return 2.0 * abs(e - d) / (abs(e) + abs(d))


def _menos(a, b):
    """a - b nos numeros que os dois tem (descendo nos dicionarios)."""
    out = {}
    for k, v in a.items():
        w = b.get(k)
        if isinstance(v, dict) and isinstance(w, dict):
            out[k] = _menos(v, w)
        elif isinstance(v, (int, float)) and isinstance(w, (int, float)) and not isinstance(v, bool):
            out[k] = v - w
    return out


def _limpo(x, casas=3):
    if isinstance(x, dict):
        return {k: _limpo(v, casas) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_limpo(v, casas) for v in x]
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (int, np.integer)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        return round(float(x), casas) if np.isfinite(x) else None
    return x


def _corridas(m):
    """Os trechos [a, b) onde a mascara 1-D e verdadeira."""
    d = np.diff(np.concatenate(([0], m.astype(np.int8), [0])))
    return list(zip(np.nonzero(d == 1)[0].tolist(), np.nonzero(d == -1)[0].tolist()))


def _dot(a, b):
    return np.einsum("...c,...c->...", a, b)


def _plano(v, eixo):
    """O angulo (graus) de v com a vertical para baixo, no plano vertical do `eixo` (+ = para o lado do eixo)."""
    return np.degrees(np.arctan2(_dot(v, eixo), -v[..., 2]))


def quadro(fr: np.ndarray) -> dict:
    """O referencial do corpo pela orientacao do abdome (o x local do colisor dele aponta para a esquerda): 'esquerda',
    'frente' (no plano) e o 'rumo' (rad). O de geometria.referencial vem dos quadris, que andam com a coxa: no trote ele
    balanca +-23 graus a cada passada (no sprint +-33), no clipe tambem; o abdome, entre a pelve e o peito que giram ao
    contrario, balanca +-1,7."""
    left = G.qrot(fr[..., G.IDX["Abdomen"], 3:7].astype(float), np.array([1.0, 0.0, 0.0]))
    left[..., 2] = 0.0
    left /= np.maximum(np.linalg.norm(left, axis=-1, keepdims=True), 1e-9)
    forward = np.cross(left, G.UP)
    return {"esquerda": left, "frente": forward, "rumo": np.arctan2(forward[..., 1], forward[..., 0])}


def casca(fr: np.ndarray, geo: list, side: str) -> np.ndarray:
    """Os pontos da casca do pe `side` ("Left"/"Right") no mundo: [..., pontos, 3]."""
    foot = fr[..., G.IDX[f"{side}Foot"], :].astype(float)
    pts = np.asarray(geo[G.IDX[f"{side}Foot"]]["pontos"], float)
    return foot[..., None, :3] + G.qrot(np.broadcast_to(foot[..., None, 3:7], foot.shape[:-1] + (len(pts), 4)), pts)


# --- Os episodios, os trechos e a referencia no chao dela ----------------------------------------------------------------

def episodio_quadro(cap: G.Captura) -> np.ndarray:
    """[mundos, passos]: o episodio de cada quadro. O quadro do passo_fim ja e o recomeco do mundo (a captura grava depois
    do passo, e o salto da pelve la e de metros): ele vai para o episodio seguinte."""
    ep = cap.episodio.astype(int)
    cols = cap.meta["colunas_episodios"]
    for row in cap.episodios:
        ep[int(row[cols.index("mundo")]), int(row[cols.index("passo_fim")])] += 1
    return ep


def trechos(ep: np.ndarray, ok: np.ndarray, dt: float) -> list:
    """Os trechos da marcha (mundo, a, b): quadros [a, b) contiguos, ok, do mesmo episodio, depois da largada."""
    out = []
    for w in range(ep.shape[0]):
        for e in np.unique(ep[w]):
            idx = np.nonzero(ep[w] == e)[0]
            usar = ok[w, idx] & ((idx - idx[0]) * dt >= LARGADA_S - 1e-6)
            out += [(w, int(idx[0] + i), int(idx[0] + j)) for i, j in _corridas(usar) if j - i >= 2]
    return out


def referencia_no_chao(cap: G.Captura, tr: list) -> np.ndarray:
    """A referencia andando sozinha: gravada, ela e posta sob a pelve do corpo a cada quadro (o andar do clipe se perde).
    Aqui a pelve do clipe anda o que o pe de apoio dele pede: com o pe do clipe no chao (os pontos da sola abaixo de PISO_M
    nos dois quadros), a pelve anda o contrario do que esses pontos andam em relacao a ela (a mediana de cada apoio); no voo,
    a velocidade e interpolada (no ar ela quase nao muda). Comeca na pelve do corpo no inicio de cada trecho. Confere com a
    velocidade dos clipes autorados (o autoteste). O pedido nao serve para isso: o trote pede 4,0 m/s e o jog anda 3,0. Fora
    dos trechos, e nos trechos sem pedido (parado, giro: o pe girando no lugar engana a conta), fica como gravada (os angulos
    nao mudam com a translacao).
    shortcut: o pe do clipe e tomado como firme (um clipe que escorrega some aqui); o deslize do clipe sai zero por isso. O
    run_forward_medium (a corrida) nao e autorado: o pe dele afunda ate 4,5 cm no chao e ainda varre para tras 2-5 cm acima
    dele, entao a velocidade natural dele (3,5 m/s aqui; o pe varre a 4) e incerta. A direcao do clipe de lado tem ~2 graus
    de incerteza (o pe dele varre torto ao longo do apoio: no jog_strafe, autorado a 90 graus, os estimadores dao de +0,8 a
    -1,7); nos laterais, o desvio da rede vem do corpo e do espelho, nao do delta_referencia."""
    ref = cap.referencia.astype(float)
    for w, a, b in tr:
        if cap.info[w, a:b, 3].mean() < 0.1:
            continue
        pelvis = ref[w, a:b, 0, :2]
        acc, cnt = np.zeros((b - a - 1, 2)), np.zeros(b - a - 1)
        for _, side in LADOS:
            P = casca(ref[w, a:b], cap.geo, side)
            down = (P[:-1, :, 2] < PISO_M) & (P[1:, :, 2] < PISO_M)
            rel = P[..., :2] - pelvis[:, None, :]
            n = down.sum(-1)
            move = ((rel[1:] - rel[:-1]) * down[..., None]).sum(1) / np.maximum(n, 1)[:, None]
            # (A mediana de cada apoio deste pe: o quadro do pouso, com o pe do clipe ainda freando, puxava o voo inteiro na
            # interpolacao - a corrida dava 2,9 m/s com o pe varrendo a 4, o jog 2,94 contra os 3,0 dele.)
            for p, q in _corridas(n > 0):
                acc[p:q] -= np.median(move[p:q], 0) / cap.dt
            cnt += n > 0
        ok = cnt > 0
        v = np.zeros_like(acc)
        if ok.any():
            i = np.arange(len(v))
            v = np.stack([np.interp(i, i[ok], acc[ok, k] / cnt[ok]) for k in range(2)], -1)
        root = cap.corpo[w, a, 0, :2] + np.concatenate([[[0.0, 0.0]], np.cumsum(v * cap.dt, 0)])
        ref[w, a:b, :, :2] += (root - pelvis)[:, None, :]
    return ref


# --- As series de uma pose (o corpo ou o clipe) ---------------------------------------------------------------------------

def medir(fr: np.ndarray, geo: list, dt: float) -> dict:
    """As series [mundos, passos] de um conjunto de poses: os angulos, as inclinacoes, a sola, a ponta e o calcanhar de cada
    pe, o deslize do pivo do pe de um quadro ao seguinte, a cabeca e a distancia entre os pes."""
    j = G.articulacoes(fr, geo)
    f = quadro(fr)
    m = {"j": j, "f": f}
    for s, side in LADOS:
        hip, knee, ankle = j[f"quadril_{s}"], j[f"joelho_{s}"], j[f"tornozelo_{s}"]
        m[f"joelho_{s}"] = 180.0 - G.angulo(hip - knee, ankle - knee)
        m[f"quadril_{s}"] = _plano(knee - hip, f["frente"])
        m[f"abducao_{s}"] = _plano(knee - hip, f["esquerda"] if s == "e" else -f["esquerda"])
        toe = G.qrot(fr[..., G.IDX[f"{side}Foot"], 3:7].astype(float), np.array([1.0, 0.0, 0.0]))
        m[f"tornozelo_{s}"] = 90.0 - G.angulo(knee - ankle, toe)
        m[f"pe_{s}"] = np.degrees(np.arcsin(np.clip(toe[..., 2], -1.0, 1.0)))
        m[f"cotovelo_{s}"] = 180.0 - G.angulo(j[f"ombro_{s}"] - j[f"cotovelo_{s}"], j[f"punho_{s}"] - j[f"cotovelo_{s}"])
        m[f"ombro_{s}"] = _plano(j[f"cotovelo_{s}"] - j[f"ombro_{s}"], f["frente"])
        pts = np.asarray(geo[G.IDX[f"{side}Foot"]]["pontos"], float)
        P = casca(fr, geo, side)
        z = P[..., 2]
        m[f"casca_{s}"] = P
        m[f"sola_{s}"] = z.min(-1)
        m[f"ponta_{s}"] = z[..., pts[:, 0] > PONTA_M].min(-1)
        m[f"calcanhar_{s}"] = z[..., pts[:, 0] < -PONTA_M].min(-1)
        # (O deslize do pivo: dos pontos da casca no chao nos dois quadros, o que menos andou - rolar do calcanhar a ponta
        # nao conta, escorregar conta.)
        both = (z[..., :-1, :] < PISO_M) & (z[..., 1:, :] < PISO_M)
        step = np.linalg.norm(P[..., 1:, :, :2] - P[..., :-1, :, :2], axis=-1)
        piv = np.where(both, step, np.inf).min(-1)
        piv[~both.any(-1)] = np.nan
        m[f"pivo_{s}"] = np.concatenate([piv, np.full(piv.shape[:-1] + (1,), np.nan)], -1)
    ankles = 0.5 * (j["tornozelo_e"] + j["tornozelo_d"])
    m["inclinacao_lado_corpo"] = G.inclinacao(j["cabeca"], ankles, f["esquerda"])
    m["inclinacao_frente_corpo"] = G.inclinacao(j["cabeca"], ankles, f["frente"])
    m["inclinacao_lado_tronco"] = G.inclinacao(j["pescoco"], j["pelve"], f["esquerda"])
    m["inclinacao_frente_tronco"] = G.inclinacao(j["pescoco"], j["pelve"], f["frente"])
    head = fr[..., G.IDX["Head"], 3:7].astype(float)
    turn = 2.0 * np.arccos(np.clip(np.abs(_dot(head[..., 1:, :], head[..., :-1, :])), 0.0, 1.0))
    m["cabeca_giro"] = np.concatenate([np.degrees(turn) / dt, np.full(turn.shape[:-1] + (1,), np.nan)], -1)
    rel = j["cabeca"] - j["pescoco"]
    m["cabeca_rel"] = np.stack([_dot(rel, f["frente"]), _dot(rel, f["esquerda"]), rel[..., 2]], -1)
    m["lado_pes"] = _dot(j["pe_e"] - j["pe_d"], f["esquerda"])
    A, B = m["casca_e"], m["casca_d"]
    # (De 64 em 64 quadros: todos os pares de pontos das duas cascas de uma vez nao cabem numa captura longa.)
    m["dist_pes"] = np.concatenate([np.linalg.norm(A[:, t:t + 64, :, None] - B[:, t:t + 64, None], axis=-1).min((-1, -2))
                                    for t in range(0, A.shape[1], 64)], axis=1)
    return m


def contato(m: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """[mundos, passos, 2]: o pe no chao (a sola abaixo de TOQUE_M), os toques breves (um quadro no chao entre dois no ar: a
    bicuda, nao um apoio) e os quiques (no meio do apoio, a sola sobe um quadro acima de TOQUE_M e volta: o pe pulando no
    chao)."""
    low = np.stack([m[f"sola_{s}"] < TOQUE_M for s, _ in LADOS], -1)
    c = low.copy()
    c[:, 1:-1] |= c[:, :-2] & c[:, 2:]
    breve = np.zeros_like(c)
    breve[:, 1:-1] = c[:, 1:-1] & ~c[:, :-2] & ~c[:, 2:]
    hop = np.zeros_like(c)
    hop[:, 1:-1] = c[:, 1:-1] & ~low[:, 1:-1] & low[:, :-2] & low[:, 2:]
    return c & ~breve, breve, hop


# --- A marcha de um conjunto de poses ---------------------------------------------------------------------------------------

def marcha(m: dict, c: np.ndarray, breve: np.ndarray, hop: np.ndarray, tr: list, info: np.ndarray,
           dt: float) -> tuple[dict, dict, dict, dict]:
    """(global, lados, ciclo, eventos) de um conjunto de poses com os contatos `c` (e os toques breves e os quiques de
    contato()) nos trechos `tr`."""
    j, f = m["j"], m["f"]
    W, T = c.shape[:2]
    usar = np.zeros((W, T), bool)
    for w, a, b in tr:
        usar[w, a:b] = True
    # (As velocidades de t a t+1 so com os dois no trecho: o quadro depois do fim ja e o recomeco do mundo.)
    par = usar.copy()
    par[:, :-1] &= usar[:, 1:]
    par[:, -1] = False
    ev = {"passos": [], "repetidos": 0, "passadas": {"e": [], "d": []}, "apoios": {"e": [], "d": []},
          "balancos": {"e": [], "d": []}, "pousos": {"e": [], "d": []}}

    def eixos(w, t):
        r = info[w, t, 4] if info[w, t, 3] > 0.1 else f["rumo"][w, t]
        return np.array([np.cos(r), np.sin(r)]), np.array([-np.sin(r), np.cos(r)])

    for w, a, b in tr:
        downs = []
        for k, (s, _) in enumerate(LADOS):
            runs = [(a + i, a + jj) for i, jj in _corridas(c[w, a:b, k])]
            for p, q in runs:
                if p > a:
                    downs.append((p, s))
                    ev["pousos"][s].append((w, p))
                if p > a and q < b:
                    ev["apoios"][s].append((w, p, q))
            for (_, q), (p, _) in zip(runs[:-1], runs[1:]):
                ev["balancos"][s].append((w, q, p))
            touch = [p for p, _ in runs if p > a]
            ev["passadas"][s] += [(w, p0, p1) for p0, p1 in zip(touch[:-1], touch[1:])]
        downs.sort()
        for (t0, s0), (t1, s1) in zip(downs[:-1], downs[1:]):
            if s0 == s1:
                ev["repetidos"] += 1
                continue
            fwd, lat = eixos(w, t1)
            d = j[f"pe_{s1}"][w, t1, :2] - j[f"pe_{s0}"][w, t0, :2]
            ev["passos"].append((s1, (t1 - t0) * dt, float(d @ fwd), float(abs(d @ lat)) * 100.0))
    # (Passadas fora de 0,6-1,6 da mediana sao eventos perdidos ou tropecos: ficam fora do ciclo medio e do quique.)
    for s in "ed":
        dur = np.array([p1 - p0 for _, p0, p1 in ev["passadas"][s]], float)
        if dur.size:
            med = np.median(dur)
            ev["passadas"][s] = [x for x, d in zip(ev["passadas"][s], dur) if 0.6 * med <= d <= 1.6 * med]

    lados, ciclo = {}, {}
    for k, (s, _) in enumerate(LADOS):
        steps = [x for x in ev["passos"] if x[0] == s]
        strides = ev["passadas"][s]
        stances = ev["apoios"][s]
        swings = ev["balancos"][s]
        touches = ev["pousos"][s]
        slip = [np.nansum(m[f"pivo_{s}"][w, p:q - 1]) * 100.0 for w, p, q in stances]
        pivot = m[f"pivo_{s}"][c[..., k] & par] / dt
        pivot = pivot[np.isfinite(pivot)]
        swing_h = {"elevacao_cm": [], "ponta_max_cm": [], "calcanhar_max_cm": [], "folga_ponta_cm": []}
        for w, q, p in swings:
            swing_h["elevacao_cm"].append(m[f"sola_{s}"][w, q:p].max() * 100.0)
            swing_h["ponta_max_cm"].append(m[f"ponta_{s}"][w, q:p].max() * 100.0)
            swing_h["calcanhar_max_cm"].append(m[f"calcanhar_{s}"][w, q:p].max() * 100.0)
            n = p - q
            swing_h["folga_ponta_cm"].append(m[f"ponta_{s}"][w, q + n // 4:q + max(n // 4 + 1, 3 * n // 4)].min() * 100.0)
        land = [(j[f"pe_{s}"][w, t] - j["centro_massa"][w, t]) for w, t in touches]
        out = f["esquerda"] if s == "e" else -f["esquerda"]
        knee = m[f"joelho_{s}"]
        lados[s] = {
            "passos": len(steps),
            "passo_s": _media([x[1] for x in steps]),
            "comprimento_m": _media([x[2] for x in steps]),
            "largura_cm": _media([x[3] for x in steps]),
            "passada_s": _media([(p1 - p0) * dt for _, p0, p1 in strides]),
            "apoio_frac": float(c[..., k][usar].mean()) if usar.any() else None,
            **{key: _media(v) for key, v in swing_h.items()},
            "deslize_cm": _media(slip),
            "deslize_max_cm": float(max(slip)) if slip else None,
            "arrasto_frac": float((pivot > ARRASTO_MS).mean()) if pivot.size else None,
            "toques_breves": int(breve[..., k][usar].sum()),
            "quiques": int(hop[..., k][usar].sum()),
            "quique_cm": _media(m[f"sola_{s}"][hop[..., k] & usar] * 100.0),
            "tropecos": sum(1 for _, q, p in swings if (p - q) * dt < TROPECO_S),
            "pe_pouso_graus": _media([m[f"pe_{s}"][w, t] for w, t in touches]),
            "pouso_frente_cm": _media([v @ f["frente"][w, t] * 100.0 for v, (w, t) in zip(land, touches)]),
            "pouso_fora_cm": _media([v @ out[w, t] * 100.0 for v, (w, t) in zip(land, touches)]),
            "joelho_pico": _media([knee[w, p0:p1 + 1].max() for w, p0, p1 in strides]),
            "joelho_apoio_pico": _media([knee[w, p:q].max() for w, p, q in stances]),
            "joelho_amplitude": _amp(knee[usar]),
            "quadril_media": _media(m[f"quadril_{s}"][usar]),
            "quadril_amplitude": _amp(m[f"quadril_{s}"][usar]),
            "abducao_media": _media(m[f"abducao_{s}"][usar]),
            "tornozelo_amplitude": _amp(m[f"tornozelo_{s}"][usar]),
            "cotovelo_media": _media(m[f"cotovelo_{s}"][usar]),
            "ombro_media": _media(m[f"ombro_{s}"][usar]),
            "ombro_amplitude": _amp(m[f"ombro_{s}"][usar]),
        }
        # (O ciclo medio: cada passada do proprio pe, de pouso a pouso, em 101 pontos; a saida do pe em % do ciclo.)
        curves = {"joelho": [], "quadril": [], "tornozelo": []}
        lift = []
        for w, p0, p1 in strides:
            x = np.linspace(p0, p1, PONTOS)
            for key in curves:
                curves[key].append(np.interp(x, np.arange(p0, p1 + 1), m[f"{key}_{s}"][w, p0:p1 + 1]))
            off = [q for ww, p, q in stances if ww == w and p == p0]
            if off:
                lift.append(100.0 * (off[0] - p0) / (p1 - p0))
        ciclo[s] = {"n": len(strides), "saida_pct": _media(lift),
                    **{key: {"media": np.mean(v, 0).tolist(), "dp": np.std(v, 0).tolist()} if v else None
                       for key, v in curves.items()}}

    steps = ev["passos"]
    times = np.array([x[1] for x in steps], float)
    secs = usar.sum() * dt
    both = c[..., 0] & c[..., 1]
    none = ~c[..., 0] & ~c[..., 1]
    # (A base: os pes no chao; no voo, o meio dos dois.)
    feet = np.stack([j["pe_e"], j["pe_d"]], -2)
    wgt = c[..., None].astype(float)
    base = np.where(c.any(-1)[..., None], (feet * wgt).sum(-2) / np.maximum(wgt.sum(-2), 1.0), feet.mean(-2))
    order = np.sign(m["lado_pes"])
    cross = sum(int((np.diff(order[w, a:b]) != 0).sum()) for w, a, b in tr)
    bounce_side = "e" if ev["passadas"]["e"] else "d"
    pelvis_z = j["pelve"][..., 2]
    rel = m["cabeca_rel"][usar]
    g = {
        "passos": len(steps),
        "passos_repetidos": ev["repetidos"],
        "toques_breves": int(breve[usar].sum()),
        "quiques": int(hop[usar].sum()),
        "tropecos": lados["e"]["tropecos"] + lados["d"]["tropecos"],
        "cadencia": 60.0 / times.mean() if times.size else None,
        "passo_s": _media(times),
        "passo_cv": float(times.std() / times.mean()) if times.size > 1 else None,
        "passada_s": _media([lados[s]["passada_s"] for s in "ed"]),
        "comprimento_m": _media([x[2] for x in steps]),
        "largura_cm": _media([x[3] for x in steps]),
        "apoio_frac": _media([lados[s]["apoio_frac"] for s in "ed"]),
        "duplo_apoio_frac": float(both[usar].mean()) if usar.any() else None,
        "voo_frac": float(none[usar].mean()) if usar.any() else None,
        "elevacao_cm": _media([lados[s]["elevacao_cm"] for s in "ed"]),
        "deslize_cm": _media([lados[s]["deslize_cm"] for s in "ed"]),
        "arrasto_frac": _media([lados[s]["arrasto_frac"] for s in "ed"]),
        "cruzamentos_por_s": cross / secs if secs else None,
        "dist_pes_min_cm": _pct(m["dist_pes"][usar] * 100.0, 0),
        "dist_pes_p5_cm": _pct(m["dist_pes"][usar] * 100.0, 5),
        **{key: _vb(m[key][usar]) for key in INCLINACOES},
        "cm_lateral_cm": _vb(_dot(j["centro_massa"] - base, f["esquerda"])[usar] * 100.0),
        "pelve_altura_m": _media(pelvis_z[usar]),
        "pelve_quique_cm": _media([np.ptp(pelvis_z[w, p0:p1 + 1]) * 100.0 for w, p0, p1 in ev["passadas"][bounce_side]]),
        "cabeca_giro_p95_gs": _pct(m["cabeca_giro"][par], 95),
        "cabeca_pescoco_cm": float(np.sqrt(rel.var(0).sum()) * 100.0) if len(rel) > 1 else None,
        "rosto_rumo_graus": _circ(_enrola(f["rumo"] - info[..., 4])[usar & (info[..., 3] > 0.1)])["vies"],
    }
    return g, lados, ciclo, ev


def deslocamento(m: dict, tr: list, info: np.ndarray, dt: float) -> dict:
    """A velocidade do centro de massa contra o pedido e o desvio do rumo (como o desvio_rumo do treino: a soma da
    velocidade no chao contra o rumo pedido, + = esquerda), so com pedido."""
    if not tr:
        return {}
    al, la, sp, ped = [], [], [], []
    com = m["j"]["centro_massa"][..., :2]
    for w, a, b in tr:
        v = np.diff(com[w, a:b], axis=0) / dt
        r = info[w, a:b - 1, 4]
        al.append(v[:, 0] * np.cos(r) + v[:, 1] * np.sin(r))
        la.append(-v[:, 0] * np.sin(r) + v[:, 1] * np.cos(r))
        sp.append(np.linalg.norm(v, axis=-1))
        ped.append(info[w, a:b - 1, 3])
    al, la, sp, ped = (np.concatenate(x) for x in (al, la, sp, ped))
    moving = ped.mean() > 0.1
    return {"velocidade_ms": _media(sp), "avanco_ms": _media(al) if moving else None, "pedido_ms": float(ped.mean()),
            "razao_velocidade": float(al.mean() / ped.mean()) if moving else None,
            "desvio_rumo_graus": float(np.degrees(np.arctan2(la.sum(), al.sum()))) if moving else None}


# --- A analise inteira ------------------------------------------------------------------------------------------------------

def _analise(cap: G.Captura) -> tuple[dict, dict]:
    dt = cap.dt
    ep = episodio_quadro(cap)
    # (O levantar do chao nao e marcha: com ele, o lev_levantar dava inclinacao de +-170 graus, cadencia e desvio.)
    ok = cap.vivo() & ~np.array([[c.startswith("stand_up") for c in row] for row in cap.meta["clipes"]], bool)
    # (Nem a queda: de QUEDA_S antes da pelve passar abaixo de CAIDO_M ate o fim do episodio - 1 s de queda na janela muda o
    # vies de lado em ~3 graus, o tamanho do defeito procurado. As quedas ficam contadas em episodios.)
    for w, t in zip(*np.nonzero(ok & (cap.corpo[..., 0, 2] < CAIDO_M))):
        s = max(0, t - int(round(QUEDA_S / dt)))
        ok[w, s:] &= ep[w, s:] != ep[w, t]
    tr = trechos(ep, ok, dt)
    tr_ref = trechos(ep, ok & np.isfinite(cap.referencia[..., 0, 0]), dt)
    body = medir(cap.corpo, cap.geo, dt)
    ref = medir(referencia_no_chao(cap, tr_ref), cap.geo, dt)
    c_body, br_body, hop_body = contato(body)
    c_ref, br_ref, hop_ref = contato(ref)
    g, lados, ciclo_b, ev_b = marcha(body, c_body, br_body, hop_body, tr, cap.info, dt)
    gr, lados_r, ciclo_r, ev_r = marcha(ref, c_ref, br_ref, hop_ref, tr_ref, cap.info, dt)
    g.update(deslocamento(body, tr, cap.info, dt))
    # (No clipe, a velocidade e a natural dele, a do pe de apoio - sem pedido ele fica sob o corpo e ela seria a do corpo;
    # o deslize dele e zero por construcao: fora.)
    travel = deslocamento(ref, tr_ref, cap.info, dt)
    gr.update(travel if travel.get("avanco_ms") is not None else {}, deslize_cm=None, arrasto_frac=None)
    for s in "ed":
        lados_r[s].update(deslize_cm=None, deslize_max_cm=None, arrasto_frac=None)
    usar = np.zeros(ok.shape, bool)
    for w, a, b in tr:
        usar[w, a:b] = True
    both = usar & np.isfinite(cap.referencia[..., 0, 0])
    g["rosto_clipe_graus"] = _circ(_enrola(body["f"]["rumo"] - ref["f"]["rumo"])[both])
    # (A mao de fora nos quadros analisados, em fracao do peso: o quanto o corpo ainda se apoia nela. Linear: na forca F do
    # exercicio ela segura ate F do peso. O clipe, a referencia, nao tem mao.)
    peso = cap.meta["ragdoll"]["massa_total"] * 9.81
    for chave, termo in (("mao_sustenta_frac", "mao_sustenta_n"), ("mao_empurra_frac", "mao_empurra_n")):
        if termo in cap.meta["medidas"]:
            g[chave] = _media(cap.medida(termo)[usar] / peso)
    placed = G.articulacoes(cap.referencia, cap.geo)
    erro = {k: {"media": _media(d), "p90": _pct(d, 90)}
            for k in ARTICULACOES for d in [np.linalg.norm(body["j"][k] - placed[k], axis=-1)[both] * 100.0]}
    cols = cap.meta["colunas_episodios"]
    E = cap.episodios
    cause = E[:, cols.index("fim_causa")].astype(int)
    fell = (E[:, cols.index("caiu_de_pe")] > 0) | np.isin(cause, (1, 2))
    res = {
        "captura": cap.nome, "exercicio": cap.meta["exercicio"], "modo": cap.meta["modo"], "forca": cap.meta["forca"],
        "atualizacao": cap.meta.get("atualizacao"), "clipe_puro": cap.meta.get("clipe_puro"),
        "episodios": {"n": int(len(E)), "cairam": int(fell.sum()),
                      "causas": {FIM.get(int(k), str(int(k))): int((cause == k).sum()) for k in np.unique(cause)},
                      **{k: _media(E[:, cols.index(k)]) for k in ("desvio_rumo", "semelhanca", "estabilidade")},
                      "analisado_s": float(usar.sum() * dt)},
        "global": g,
        "referencia": {**gr, "lados": lados_r, "simetria": {k: _si(k, lados_r["e"][k], lados_r["d"][k]) for k in lados_r["e"]}},
        "delta_referencia": {**_menos(g, gr), "lados": {s: _menos(lados[s], lados_r[s]) for s in "ed"}},
        "lados": lados,
        "simetria": {k: _si(k, lados["e"][k], lados["d"][k]) for k in lados["e"]},
        "ciclo": {"pontos": PONTOS, "corpo": ciclo_b, "referencia": ciclo_r},
        "erro_referencia_cm": erro,
    }
    res = _limpo(res)
    dados = {"cap": cap, "ep": ep, "tr": tr, "tr_ref": tr_ref, "corpo": body, "ref": ref, "c": c_body, "ev": ev_b,
             "ev_ref": ev_r, "res": res}
    return res, dados


def analisar(cap: G.Captura) -> dict:
    """A analise (o dicionario do .marcha.json) de uma captura carregada."""
    return _analise(cap)[0]


def comparar(a: dict, b: dict) -> dict:
    """a - b entre duas analises (por exemplo a rede contra o clipe puro do mesmo exercicio)."""
    return _limpo({"a": a["captura"], "b": b["captura"], "global": _menos(a["global"], b["global"]),
                   "lados": {s: _menos(a["lados"][s], b["lados"][s]) for s in "ed"},
                   "simetria": _menos(a["simetria"], b["simetria"])})


def _espelhar(g: dict) -> dict:
    out = dict(g)
    for k in LATERAIS:
        v = out.get(k)
        if isinstance(v, dict):
            out[k] = {**v, "vies": None if v.get("vies") is None else -v["vies"]}
        elif v is not None:
            out[k] = -v
    return out


def espelho(a: dict, b: dict) -> dict:
    """Os exercicios espelhados (2x6, 1x7, 3x5): o lado esquerdo de a contra o direito de b (e vice-versa) e os sinais de
    lado de b trocados. Com os clipes espelhados tudo deveria dar ~0; o que sobra e o vies da rede (+ = a mais a esquerda)."""
    return _limpo({"a": a["captura"], "b": b["captura"], "global": _menos(a["global"], _espelhar(b["global"])),
                   "lados": {"e": _menos(a["lados"]["e"], b["lados"]["d"]), "d": _menos(a["lados"]["d"], b["lados"]["e"])}})


# --- Os desenhos ----------------------------------------------------------------------------------------------------------

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": "#c3c2b7", "axes.labelcolor": TINTA, "xtick.color": TINTA,
                         "ytick.color": TINTA, "axes.grid": True, "grid.color": GRADE, "grid.linewidth": 0.6,
                         "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False})
    return plt


def _fmt(x, casas=1, sinal=False):
    return "–" if x is None else f"{x:+.{casas}f}" if sinal else f"{x:.{casas}f}"


def desenhar_ciclo(d: dict, path: Path) -> None:
    plt = _plt()
    res = d["res"]
    fig, axs = plt.subplots(1, 3, figsize=(13, 4.2))
    titles = {"joelho": "Joelho: flexão (°)", "quadril": "Quadril: coxa para a frente (°)",
              "tornozelo": "Tornozelo: dorsiflexão (°)"}
    x = np.linspace(0, 100, PONTOS)
    for ax, key in zip(axs, titles):
        for s, nome in (("e", "esq"), ("d", "dir")):
            cb, cr = res["ciclo"]["corpo"][s], res["ciclo"]["referencia"][s]
            if cb[key]:
                mu, sd = np.array(cb[key]["media"]), np.array(cb[key]["dp"])
                ax.fill_between(x, mu - sd, mu + sd, color=COR[s], alpha=0.15, lw=0)
                ax.plot(x, mu, color=COR[s], lw=2, label=f"corpo {nome} (n={cb['n']})")
            if cr[key]:
                ax.plot(x, cr[key]["media"], color=COR[s], lw=1.3, ls="--", label=f"clipe {nome} (n={cr['n']})")
            for cc, ls in ((cb, "-"), (cr, "--")):
                if cc["saida_pct"] is not None:
                    ax.axvline(cc["saida_pct"], color=COR[s], lw=0.8, ls=ls, alpha=0.6)
        ax.set_title(titles[key], fontsize=10, color="#0b0b0b", loc="left")
        ax.set_xlabel("% do ciclo (0 = pouso do próprio pé; linha vertical = saída do pé)")
        ax.set_xlim(0, 100)
    # (A legenda fora dos eixos: dentro, ela cobria as curvas.)
    handles, labels = axs[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper right", ncol=len(labels), fontsize=8)
    else:
        axs[1].text(0.5, 0.5, "sem passadas completas", transform=axs[1].transAxes, ha="center", color=TINTA)
    g = res["global"]
    fig.suptitle(f"{res['captura']} · força {res['forca']:.0%} · cadência {_fmt(g['cadencia'], 0)} passos/min "
                 f"(clipe {_fmt(res['referencia']['cadencia'], 0)}) · tracejado = clipe", fontsize=11, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _janela(x, n):
    """A media movel centrada de n quadros por mundo, ignorando nan (nan onde nao ha nenhum)."""
    k = np.ones(n)
    num = np.stack([np.convolve(np.nan_to_num(r), k, "same") for r in x])
    den = np.stack([np.convolve(np.isfinite(r).astype(float), k, "same") for r in x])
    return np.where(den > 0, num / np.maximum(den, 1), np.nan)


def _velocidades(m: dict, ok: np.ndarray, info: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """[mundos, passos-1]: a velocidade do centro de massa (media de 0,3 s) e o desvio do rumo (graus, janela de 1 s, so com
    pedido), nos pares de quadros `ok`."""
    v = np.diff(m["j"]["centro_massa"][..., :2], axis=1) / dt
    v[~ok] = np.nan
    r = info[:, :-1, 4]
    along = _janela(v[..., 0] * np.cos(r) + v[..., 1] * np.sin(r), 30)
    lat = _janela(-v[..., 0] * np.sin(r) + v[..., 1] * np.cos(r), 30)
    drift = np.degrees(np.arctan2(lat, along))
    drift[(np.hypot(along, lat) < 0.3) | (info[:, :-1, 3] < 0.1)] = np.nan
    return _janela(np.linalg.norm(v, axis=-1), 9), drift


def desenhar_series(d: dict, path: Path) -> None:
    plt = _plt()
    cap, body, ref, ep, res = d["cap"], d["corpo"], d["ref"], d["ep"], d["res"]
    dt, (W, T) = cap.dt, ep.shape
    t = np.arange(T) * dt
    # (Sem ligar o ultimo quadro de um episodio ao recomeco do seguinte.)
    show = cap.vivo().copy()
    show[:, 1:] &= ep[:, 1:] == ep[:, :-1]
    pair = show[:, 1:] & show[:, :-1]
    usar = np.zeros((W, T), bool)
    for w, a, b in d["tr"]:
        usar[w, a:b] = True
    in_ref = np.zeros((W, T - 1), bool)
    for w, a, b in d["tr_ref"]:
        in_ref[w, a:b - 1] = True
    g, gr = res["global"], res["referencia"]
    speed, drift = _velocidades(body, pair, cap.info, dt)
    # (Sem pedido o clipe fica sob o corpo: a velocidade dele seria a do corpo, nao a natural.)
    speed_ref = _velocidades(ref, in_ref, cap.info, dt)[0] if gr.get("velocidade_ms") is not None else None
    panels = (("Inclinação de lado do corpo (°, + esquerda)", body["inclinacao_lado_corpo"], ref["inclinacao_lado_corpo"],
               f"viés {_fmt(g['inclinacao_lado_corpo']['vies'], 2, True)}° (clipe "
               f"{_fmt(gr['inclinacao_lado_corpo']['vies'], 2, True)}°)"),
              ("Tronco para a frente (°)", body["inclinacao_frente_tronco"], ref["inclinacao_frente_tronco"],
               f"viés {_fmt(g['inclinacao_frente_tronco']['vies'], 1, True)}° (clipe "
               f"{_fmt(gr['inclinacao_frente_tronco']['vies'], 1, True)}°)"),
              ("Desvio do rumo do deslocamento (°, + esquerda; janela de 1 s)", drift, None,
               f"médio {_fmt(g.get('desvio_rumo_graus'), 2, True)}°"),
              ("Altura da pelve (m)", body["j"]["pelve"][..., 2], ref["j"]["pelve"][..., 2],
               f"média {_fmt(g['pelve_altura_m'], 3)} m (clipe {_fmt(gr['pelve_altura_m'], 3)}) · quique "
               f"{_fmt(g['pelve_quique_cm'])} cm (clipe {_fmt(gr['pelve_quique_cm'])})"),
              ("Velocidade do centro de massa (m/s; preta = o pedido, tracejada = a natural do clipe, a do pé de apoio)",
               speed, speed_ref, f"média {_fmt(g.get('velocidade_ms'), 2)} (clipe {_fmt(gr.get('velocidade_ms'), 2)}, "
                                 f"pedido {_fmt(g.get('pedido_ms'), 2)})"))
    fig, axs = plt.subplots(len(panels), 1, figsize=(12, 12.5), sharex=True)
    for w in range(W):
        for e in np.unique(ep[w]):
            t0 = np.nonzero(ep[w] == e)[0][0] * dt
            for ax in axs:
                ax.axvspan(t0, min(t0 + LARGADA_S, t[-1]), color="#898781", alpha=0.05, lw=0)
                if t0 > 0:
                    ax.axvline(t0, color=MUNDO[w], lw=0.8, ls=":")
    axs[-1].plot(t, np.where(show, cap.info[..., 3], np.nan).T, color="#0b0b0b", lw=0.9, ls="--")
    for ax, (title, yb, yr, note) in zip(axs, panels):
        for w in range(W):
            n = yb.shape[1]
            # (Claro fora da analise - a largada, o levantar do chao -, forte dentro.)
            for mask, lw, alpha, label in ((show[w, :n], 0.8, 0.3, None), (usar[w, :n], 1.4, 1.0, f"mundo {w}")):
                ax.plot(t[:n], np.where(mask, yb[w], np.nan), color=MUNDO[w], lw=lw, alpha=alpha, label=label)
                if yr is not None:
                    ax.plot(t[:n], np.where(mask, yr[w, :n], np.nan), color=MUNDO[w], lw=0.7 * lw, ls="--",
                            alpha=0.85 * alpha)
        ax.set_title(title, fontsize=10, color="#0b0b0b", loc="left")
        # (O resumo acima dos eixos: dentro, a caixa dele cobria as curvas e a linha do pedido.)
        ax.text(1.0, 1.02, note, transform=ax.transAxes, ha="right", va="bottom", color="#0b0b0b")
        if "°" in title:
            ax.axhline(0, color="#898781", lw=0.8)
    axs[0].set_xlim(0, t[-1])
    fig.legend(*axs[0].get_legend_handles_labels(), loc="upper right", ncol=W, fontsize=8)
    axs[-1].set_xlabel("tempo (s) · cheia = corpo, tracejada = clipe · traço claro = fora da análise (faixa cinza = a "
                       "largada; o levantar do chão) · pontilhada = recomeço do mundo")
    fig.suptitle(f"{res['captura']} · força {res['forca']:.0%} · atualização {res['atualizacao']}", fontsize=11, x=0.01,
                 ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    fig.savefig(path, dpi=100)
    plt.close(fig)


def desenhar_pegadas(d: dict, path: Path) -> None:
    plt = _plt()
    cap, body, ref, res = d["cap"], d["corpo"], d["ref"], d["res"]
    worlds = sorted({w for w, _, _ in d["tr"]})
    if not worlds:
        return
    fig, axs = plt.subplots(len(worlds), 1, figsize=(13, 2.9 * len(worlds)), squeeze=False)
    for ax, w in zip(axs[:, 0], worlds):
        for w_, a, b in d["tr"]:
            if w_ != w:
                continue
            o = body["j"]["centro_massa"][w, a, :2]
            r = cap.info[w, a, 4] if cap.info[w, a, 3] > 0.1 else body["f"]["rumo"][w, a]
            rot = np.array([[np.cos(r), np.sin(r)], [-np.sin(r), np.cos(r)]])

            def local(p):
                q = (p[..., :2] - o) @ rot.T
                return q[..., 0], q[..., 1] * 100.0

            ra = cap.info[w, a:b, 4]
            line = np.cumsum(cap.info[w, a:b, 3:4] * np.stack([np.cos(ra), np.sin(ra)], -1) * cap.dt, 0) + o
            lx, ly = local(np.vstack([o, line[:-1]]))
            ax.plot(lx, ly, color="#0b0b0b", lw=1, ls="--")
            if lx[-1] > 0.5:
                ax.annotate("pedido", (lx[-1], ly[-1]), xytext=(3, 3), textcoords="offset points", fontsize=8)
            ax.plot(*local(body["j"]["centro_massa"][w, a:b]), color="#898781", lw=1)
            for k, (s, _) in enumerate(LADOS):
                xs, ys = local(body["j"][f"pe_{s}"][w, a:b])
                ax.plot(xs, ys, color=COR[s], lw=0.6, alpha=0.5)
                down = d["c"][w, a:b, k]
                ax.plot(np.where(down, xs, np.nan), np.where(down, ys, np.nan), color=COR[s], lw=3.2,
                        solid_capstyle="round")
                for ww, p in d["ev"]["pousos"][s]:
                    if ww == w and a <= p < b:
                        ax.plot(*local(body["j"][f"pe_{s}"][w, p]), "o", ms=6, color=COR[s], mec="white", mew=1)
                for ww, p in d["ev_ref"]["pousos"][s]:
                    if ww == w and a <= p < b:
                        ax.plot(*local(ref["j"][f"pe_{s}"][w, p]), "o", ms=7, mfc="none", mec=COR[s], mew=1.2)
        ax.set_ylabel("lateral (cm, + esq.)")
        # (O desvio como o global, contra o rumo pedido de cada instante: o angulo do ponto final enganava nas curvas.)
        seg = [x for x in d["tr"] if x[0] == w]
        com = body["j"]["centro_massa"][w, :, :2]
        dist = sum(float(np.linalg.norm(np.diff(com[a:b], axis=0), axis=-1).sum()) for _, a, b in seg)
        drift = deslocamento(body, seg, cap.info, cap.dt).get("desvio_rumo_graus")
        ax.set_title(f"mundo {w} · desvio {_fmt(drift, 2, True)}° · {dist:.1f} m andados em "
                     f"{sum(b - a for _, a, b in seg) * cap.dt:.1f} s", fontsize=10, color="#0b0b0b", loc="left")
    g, L = res["global"], res["lados"]
    axs[-1, 0].set_xlabel("no rumo pedido (m) · tracejada = o pedido · cinza = centro de massa · traço grosso = pé no chão "
                          "(alto = escorregando ou girando de lado) · ● pouso do corpo · ○ pouso do clipe")
    fig.suptitle(f"{res['captura']} · desvio {_fmt(g.get('desvio_rumo_graus'), 2, True)}° · deslize no apoio esq "
                 f"{_fmt(L['e']['deslize_cm'])} / dir {_fmt(L['d']['deslize_cm'])} cm · passo esq "
                 f"{_fmt(L['e']['comprimento_m'], 2)} / dir {_fmt(L['d']['comprimento_m'], 2)} m (azul = esquerdo, "
                 f"laranja = direito)", fontsize=11, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def salvar(path: str | Path, saida: Path) -> dict:
    """Analisa uma captura e grava o .marcha.json e as tres figuras em `saida`."""
    res, dados = _analise(G.carregar(path))
    saida.mkdir(parents=True, exist_ok=True)
    name = res["captura"]
    (saida / f"{name}.marcha.json").write_text(json.dumps(res, ensure_ascii=False))
    desenhar_ciclo(dados, saida / f"{name}.ciclo.png")
    desenhar_series(dados, saida / f"{name}.series.png")
    desenhar_pegadas(dados, saida / f"{name}.pegadas.png")
    return res


# --- A tabela e o terminal --------------------------------------------------------------------------------------------------

def tabela(rs: list) -> None:
    print(f"{'captura':<24}{'força':>6}{'cad':>5}{'passo E/D s':>12}{'compr E/D m':>12}{'apoio E/D':>11}"
          f"{'desl E/D cm':>12}{'elev E/D cm':>12}{'lado°':>7}{'tronco°':>8}{'desvio°':>8}{'vel/ped':>8}  pior SI")
    for r in rs:
        g, L, si = r["global"], r["lados"], r["simetria"]
        worst = max(((k, v) for k, v in si.items() if v is not None), key=lambda kv: kv[1], default=(None, None))

        def par(k, c=2):
            return f"{_fmt(L['e'][k], c)}/{_fmt(L['d'][k], c)}"

        print(f"{r['captura']:<24}{r['forca']:>6.2f}{_fmt(g['cadencia'], 0):>5}{par('passo_s'):>12}{par('comprimento_m'):>12}"
              f"{par('apoio_frac'):>11}{par('deslize_cm', 1):>12}{par('elevacao_cm', 1):>12}"
              f"{_fmt(g['inclinacao_lado_corpo']['vies'], 2, True):>7}{_fmt(g['inclinacao_frente_tronco']['vies'], 1, True):>8}"
              f"{_fmt(g.get('desvio_rumo_graus'), 2, True):>8}{_fmt(g.get('razao_velocidade'), 2):>8}"
              f"  {worst[0]} {_fmt(worst[1], 2)}")


def _mostra(titulo: str, d: dict) -> None:
    def val(v):
        return _fmt(v["vies"], 2, True) if isinstance(v, dict) else _fmt(v, 2, True)
    keys = ("inclinacao_lado_corpo", "inclinacao_lado_tronco", "desvio_rumo_graus", "cm_lateral_cm", "cadencia",
            "comprimento_m", "deslize_cm", "elevacao_cm")
    print(titulo + ": " + ", ".join(f"{k} {val(d['global'][k])}" for k in keys if k in d["global"]))
    for s in "ed":
        L = d["lados"][s]
        print(f"  lado {s}: " + ", ".join(f"{k} {_fmt(L.get(k), 2, True)}" for k in
                                         ("passo_s", "comprimento_m", "apoio_frac", "deslize_cm", "elevacao_cm",
                                          "joelho_apoio_pico", "joelho_pico")))


# --- O autoteste ------------------------------------------------------------------------------------------------------------

def autoteste(saida: Path) -> None:
    """Confere nas amostras os fatos esperados (e imprime os numeros achados)."""
    rs = {n: salvar(AMOSTRAS / f"{n}.npz", saida) for n in
          ("lev_trote_0__clipe", "lev_trote_0__0.3", "lev_trote_0__atual", "lev_parado__clipe", "lev_levantar__atual",
           "lev_trote_2__clipe", "lev_trote_6__clipe", "lev_trote_2__0.3", "lev_trote_6__0.3", "lev_corrida_2__atual",
           "lev_corrida_6__atual")}
    tabela(list(rs.values()))
    clip, low = rs["lev_trote_0__clipe"], rs["lev_trote_0__0.3"]
    for name, g, si in (("corpo", clip["global"], clip["simetria"]),
                        ("clipe", clip["referencia"], clip["referencia"]["simetria"])):
        print(f"trote_0 clipe puro ({name}): lado {g['inclinacao_lado_corpo']['vies']:+.2f}°, cadência {g['cadencia']:.0f}, "
              f"SI passo {si['passo_s']:.3f} comprimento {si['comprimento_m']:.3f}")
        assert abs(g["inclinacao_lado_corpo"]["vies"]) < 1.5, "o clipe puro deveria ser ereto de lado"
        assert si["passo_s"] < 0.15 and si["comprimento_m"] < 0.15, "o clipe puro deveria ser simetrico"
        assert 150 <= g["cadencia"] <= 220, "a cadencia do trote fora do humano"
    print(f"trote_0 a 30%: lado {low['global']['inclinacao_lado_corpo']['vies']:+.2f}°, desvio "
          f"{low['global']['desvio_rumo_graus']:+.2f}° (o treino: {low['episodios']['desvio_rumo']:+.2f}°)")
    assert low["global"]["inclinacao_lado_corpo"]["vies"] < 0, "a 30% a rede tomba para a direita"
    assert low["global"]["desvio_rumo_graus"] < 0 and low["episodios"]["desvio_rumo"] < 0, "a 30% ela vai para a direita"
    # (A mao linear: em media ela nunca segura mais que a forca do exercicio em fracao do peso - em 04/11 "30%" segurava
    # 62% -, e a rede a 30% se apoia bem menos que o clipe puro, que nao se sustenta sozinho.)
    for n in ("lev_trote_0__atual", "lev_trote_0__0.3", "lev_trote_0__clipe"):
        g = rs[n]["global"]
        print(f"{n}: a mão segura {g['mao_sustenta_frac']:.0%} do peso e empurra {g['mao_empurra_frac']:.0%} (força "
              f"{rs[n]['forca']:.0%})")
        assert 0.0 < g["mao_sustenta_frac"] <= rs[n]["forca"] + 0.01 and g["mao_empurra_frac"] >= 0.0
    assert low["global"]["mao_sustenta_frac"] < 0.5 * clip["global"]["mao_sustenta_frac"]
    parado = rs["lev_parado__clipe"]
    print(f"parado clipe: {parado['global']['passos']} passos, {parado['global']['toques_breves']} toques breves")
    assert parado["global"]["passos"] <= 2, "o parado nao deveria dar passos"
    # (O levantar do chao e as quedas ficam fora: sobra o de pe. O clipe puro cai nos 4 mundos depois de levantar.)
    for n in ("lev_levantar__atual", "lev_levantar__clipe"):
        r = rs.get(n) or analisar(G.carregar(AMOSTRAS / f"{n}.npz"))
        g = r["global"]
        print(f"{n}: {r['episodios']['analisado_s']:.1f} s de pé analisados, {r['episodios']['cairam']} quedas, pelve "
              f"{g['pelve_altura_m']:.3f} m, lado {g['inclinacao_lado_corpo']['vies']:+.2f}° ± "
              f"{g['inclinacao_lado_corpo']['balanco']:.2f}")
        assert g["pelve_altura_m"] > 0.8 and g["inclinacao_lado_corpo"]["balanco"] < 10.0, "so o de pe na analise"
    assert r["episodios"]["cairam"] == 4
    # (A pelve do clipe pelo pe de apoio: a velocidade natural bate com passo x cadencia, e o trote pede mais que ela.)
    r = clip["referencia"]
    nat = r["comprimento_m"] * r["cadencia"] / 60.0
    print(f"trote_0: o clipe anda {r['velocidade_ms']:.2f} m/s (passo x cadência {nat:.2f}), o pedido é {r['pedido_ms']:.1f}, "
          f"o corpo com a mão cheia {clip['global']['velocidade_ms']:.2f} e desliza {clip['global']['deslize_cm']:.1f} cm por apoio")
    assert abs(r["velocidade_ms"] - nat) < 0.1 * nat and r["velocidade_ms"] < 0.85 * r["pedido_ms"]
    # (E bate com a velocidade dos clipes autorados - a conferencia de fora; a corrida, nao autorada, nao cai mais para os
    # 2,9 m/s do pouso puxando o voo.)
    for n, nome in (("lev_trote_0__clipe", "jog"), ("lev_trote_4__clipe", "jog_backward"),
                    ("lev_trote_2__clipe", "jog_strafe_left"), ("lev_corrida_2__clipe", "sprint_strafe_left")):
        autorado = json.loads((REPO / f"assets/animations/clips/{nome}.matteranim.json").read_text())
        v = (rs.get(n) or analisar(G.carregar(AMOSTRAS / f"{n}.npz")))["referencia"]["avanco_ms"]
        print(f"{nome}: anda {v:.2f} m/s, o clipe diz {autorado['nominalSpeedMetersPerSecond']:.2f}")
        assert abs(v - autorado["nominalSpeedMetersPerSecond"]) < 0.05 * autorado["nominalSpeedMetersPerSecond"]
    corrida = analisar(G.carregar(AMOSTRAS / "lev_corrida_0__clipe.npz"))["referencia"]["avanco_ms"]
    print(f"run_forward_medium: anda {corrida:.2f} m/s (o pé dele varre a ~4)")
    assert corrida > 3.3
    # (O SI de lados com sinais opostos daria sempre 2: fica nulo.)
    L = low["lados"]
    assert L["e"]["abducao_media"] * L["d"]["abducao_media"] < 0 and low["simetria"]["abducao_media"] is None
    # (As convencoes: a ponta e o x + da casca do pe; o abdome aponta a esquerda como os quadris, na media.)
    cap = G.carregar(AMOSTRAS / "lev_trote_0__clipe.npz")
    j = G.articulacoes(cap.corpo, cap.geo)
    for s, side in LADOS:
        x = np.asarray(cap.geo[G.IDX[f"{side}Foot"]]["pontos"])[:, 0]
        P = casca(cap.corpo, cap.geo, side)
        toe, heel = (np.median(np.linalg.norm(P[..., sel, :].mean(-2) - j[f"tornozelo_{s}"], axis=-1))
                     for sel in (x > PONTA_M, x < -PONTA_M))
        assert toe > heel + 0.05, "a ponta do pe deveria ser o x + da casca"
    assert np.mean(_dot(quadro(cap.corpo)["esquerda"], G.referencial(j)["esquerda"])) > 0.8
    esp = espelho(rs["lev_trote_2__clipe"], rs["lev_trote_6__clipe"])
    _mostra("espelho trote 2 x 6 (clipe puro)", esp)
    assert abs(esp["global"]["inclinacao_lado_corpo"]["vies"]) < 1.0 and abs(esp["lados"]["e"]["passo_s"]) < 0.03, \
        "os clipes puros espelhados deveriam dar o mesmo"
    _mostra("espelho trote 2 x 6 (30%)", espelho(rs["lev_trote_2__0.3"], rs["lev_trote_6__0.3"]))
    _mostra("espelho corrida 2 x 6 (atual)", espelho(rs["lev_corrida_2__atual"], rs["lev_corrida_6__atual"]))
    # (O pe esquerdo da corrida lateral direita pula no chao; o espelho dele, o direito da esquerda, nao.)
    hop6, hop2 = rs["lev_corrida_6__atual"]["lados"]["e"]["quiques"], rs["lev_corrida_2__atual"]["lados"]["d"]["quiques"]
    print(f"quiques: corrida 6 pé esq {hop6}, o espelho (corrida 2 pé dir) {hop2}")
    assert hop6 > 4 * max(hop2, 1)
    _mostra("trote_0: 30% contra o clipe puro", comparar(low, clip))
    print(f"marcha ok (saida em {saida})")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capturas", nargs="*", help="as capturas (.npz) da inspecao.captura")
    ap.add_argument("--saida", default=str(SAIDA))
    ap.add_argument("--contra", help="a captura do clipe puro do mesmo exercicio: as diferencas")
    ap.add_argument("--espelho", help="a captura do exercicio espelhado: o vies de lado")
    ap.add_argument("--teste", action="store_true", help="o autoteste nas amostras")
    a = ap.parse_args()
    saida = Path(a.saida)
    if a.teste:
        autoteste(saida)
        return
    if not a.capturas:
        ap.error("informe as capturas ou --teste")
    rs = [salvar(p, saida) for p in a.capturas]
    tabela(rs)
    for flag, fn, tag in ((a.contra, comparar, "contra"), (a.espelho, espelho, "espelho")):
        if not flag:
            continue
        other = analisar(G.carregar(flag))
        for r in rs:
            out = fn(r, other)
            (saida / f"{r['captura']}__{tag}__{other['captura']}.json").write_text(json.dumps(out, ensure_ascii=False))
            _mostra(f"{r['captura']} {tag} {other['captura']}", out)
    print(f"saida: {saida}")


if __name__ == "__main__":
    main()
