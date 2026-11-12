"""O fantasma do movimento: o v2 (05/11) mostra o movimento andando no caminho de verdade e no lugar, em poses discretas
(nao uma massa), com a sequencia no tempo legivel, os rastros limpos, as pegadas e o equilibrio; o v1 (a sobreposicao de
todas as poses, a folha, o espelho) continua igual mais abaixo.

v2 - os produtos (o referencial "caminho": x no rumo pedido quando ele e fixo, senao no eixo do trajeto; + = esquerda):
  --movimento  a vista lateral grande (os fantasmas ao longo do caminho, um a cada ~0,65 m entre 0,1 e 0,25 s, cada um
               numa cor do azul ao magenta pelo tempo e com o contorno escuro; a ultima pose clara com o esqueleto amarelo;
               os rastros lisos da cabeca, pelve, joelhos e pes, esquerda cheia e direita tracejada, com os pontos nos
               fantasmas; os apoios no chao em verde, E1 D1... na ordem), a planta alinhada (as pegadas e os rastros), as
               faixas dos joelhos e da inclinacao de lado contra o clipe no mesmo x, e embaixo a de frente para o caminho e
               a obliqua 3/4 (perspectiva) e as poses no lugar de uma passada (de frente na corrida de lado). O desenho e
               o detalhe de 2 s de mais movimento (o regime, a queda, o levantar) ou, com o rumo pedido saltando (a troca),
               de 0,5 s antes do salto; com --janela, a janela dada inteira; os numeros do titulo sao da janela inteira (a
               inclinacao de lado contra o clipe, a velocidade contra o pedido, o desvio). Quem quase nao anda (parado,
               giro, levantar) e espalhado no tempo sem sobrepor (ou a --espalhar m/s); quem anda mas volta sobre o proprio
               caminho (a troca: os fantasmas se empilhavam) so na lateral e nas faixas - a planta, a de frente e a
               obliqua ficam no caminho de verdade.
  --recorte S  com --movimento: a janela em cartoes de S s (a lateral sobre a planta e, no lugar, de frente e de lado), so
               com os fantasmas, as pegadas e os rastros de cada recorte; com --pegadas e --equilibrio, as linhas da
               planta; com --parado, uma linha por recorte.
  --referencia o clipe (o alvo, sob a pelve) em contorno branco sobre cada fantasma da lateral e o destaque de frente.
  --malha      a lateral com o jogador de verdade: os quadros do .dump desenhados pelo MatterModelInspector em argila, sem o
               fundo, tingidos pelo tempo e montados com o centro na pelve (inspecao.malha; so o mundo 0).
  --parado     no lugar (a pelve travada, o rumo suavizado): poses numeradas de uma passada em cronofotografia (o esqueleto
               na cor da ordem), o rastro dos pes e da cabeca, e a simetria (a perna e o braco direitos espelhados sobre os
               esquerdos). As capturas nas linhas.
  --pegadas    de cima, na escala de verdade: a sola de cada apoio numerada (E1, D1...; esquerda azul, direita laranja; o
               pe que volta ao chao em ate 0,13 s e o mesmo apoio quicando, "D2–3"), o borrao do apoio (giro, deslize), o
               balanco, o arrasto do pivo acima de 3 cm (seta vermelha), o passo e a largura desde o pouso do outro pe (a
               cadencia entre pousos de verdade), o rumo pedido e o centro de massa. As capturas nas linhas.
  --equilibrio de cima, o mesmo detalhe do movimento (o de lado exagerado 3x em volta do rumo de fato so no trecho reto):
               a base, o centro de massa, o ponto de captura e a inclinacao em setas contra a do clipe; no tempo: a margem
               do ponto de captura (andando, a de Hof: de lado do movimento ate a borda de FORA do pe de apoio - por
               dentro dele e o normal da marcha; parado, ate a borda da base; verde a salvo, vermelho alem da borda), o
               centro de massa contra o centro da base, a inclinacao de lado e as fases dos pes. So de pe (pelve > 0,6 m).
  Tambem: --passo-fantasma S, --destaque T (a pose clara), --espalhar M_S. O pe no chao e a regra de marcha.contato (a sola
  abaixo de 1,5 cm) ou um contato da fisica com impulso. --padrao: o conjunto do v1 com os mesmos nomes (fantasma_lugar,
  fantasma_mundo, desvio, folha, espelho - o inspecao.auditor liga os achados a eles) e mais o movimento (com o clipe) e
  o equilibrio de cada captura da rede (<exercicio>__<modo>__movimento.png, __equilibrio.png) e as pegadas e o parado de
  todas (<exercicio>__pegadas.png, __parado.png).

v1 - o fantasma ("chams"): todas as poses de uma janela sobrepostas, cada elo com a silhueta de verdade (a capsula
projetada, afunilada como no colisor, e o casco do pe), a cor pelo tempo, nas vistas alinhadas ao corpo (frente, lado_dir,
cima, obliqua; costas e lado_esq sao o espelho das duas primeiras), pelo abdome (geometria.referencial). Por cima: o
clipe (a referencia) nos mesmos quadros, os rastros, os contatos no chao (cima), a vertical na base, o eixo medio do corpo
(base -> cabeca, frente/costas) e as inclinacoes medias de pe no titulo (graus, + = esquerda).
Modos: "lugar" (a pelve travada no plano e o rumo do corpo suavizado numa passada: o ciclo empilhado no lugar; o clipe
como a envoltoria do que ele visita) e "mundo" (estroboscopico ao longo do caminho, um corpo a cada metro andado entre 0,2
e 0,5 s, o clipe em contorno em cada corpo). --desvio pinta cada elo pela distancia ao clipe (verde perto, vermelho a 30
cm ou mais). --folha N: N poses lado a lado numa passada (ou espacadas pelo movimento). --espelho: a captura do exercicio
espelhado refletida no plano sagital sobre a outra, em anaglifo (ciano so a captura, vermelho so o espelho, branco onde
coincidem), com o vies lateral da rede ((A - B refletido) / 2) no titulo. --grade: cada captura com as irmas da pasta.

Uso (de dentro de tools/rl):
  python -m inspecao.fantasma A.npz --movimento [--referencia] [--recorte 1] [--espalhar 0.5] [--malha] [--saida X.png]
  python -m inspecao.fantasma A.npz [B.npz ...] --pegadas | --equilibrio | --parado [--recorte 2]
  python -m inspecao.fantasma PASTA_DAS_CAPTURAS/*.npz --padrao --saida PASTA
  python -m inspecao.fantasma A.npz [B.npz ...] [--vistas frente lado_esq cima] [--modo lugar|mundo] [--janela 1.5 8]
                              [--mundo 0] [--desvio] [--grade] [--saida X.png|PASTA]
  python -m inspecao.fantasma A.npz --folha 10 [--vistas frente lado_dir]
  python -m inspecao.fantasma lev_corrida_2__atual.npz --espelho lev_corrida_6__atual.npz [--grade]
  python -m inspecao.fantasma --teste [--malha]       (--malha: tambem a malha, com o Inspector; so com o treino parado)
A janela padrao vai de 1,5 s (o fim da largada em pe; 0 no lev_levantar) ao fim do primeiro episodio, e nunca atravessa
o fim de um episodio. A saida padrao e <pasta da primeira captura>/fantasma/.
"""
from __future__ import annotations

import argparse
import tempfile
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable

import matplotlib

matplotlib.use("Agg")
import numpy as np
from matplotlib import colormaps
from matplotlib import patheffects
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.cm import ScalarMappable
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.colors import LinearSegmentedColormap, ListedColormap, Normalize, to_rgba
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyArrowPatch, Patch, PathPatch, Polygon, Rectangle
from matplotlib.path import Path as MPath

from inspecao import comum
from inspecao import geometria as G
from inspecao import malha as M_
from inspecao import marcha as M

AMOSTRAS = comum.REPO / "build-profile/tmp/inspecao_amostras_0645"
SAIDA_TESTE = comum.REPO / "build-profile/tmp/inspecao_dev/fantasma"
BASE = comum.REPO / "build-profile/tmp/inspecao_base_0731"  # (as capturas da rede r24120, os 30 exercicios)
SAIDA_TESTE2 = comum.REPO / "build-profile/tmp/inspecao_dev/fantasma_v2"

FUNDO, TEXTO, GRADE, CHAO = "#0d1117", "#d1d7e0", "#252c38", "#8b95a5"
ESQ, DIR, CENTRO, REF, EIXO = "#ff4fd8", "#ff9a1f", "#aeb6c2", "#ffffff", "#00e5ff"
CABECA = "#8ab4ff"  # (o rastro da cabeca: branco ele se confundia com a envoltoria do clipe)
# (O viridis sem o quarto mais escuro, que some no fundo escuro.)
TEMPO = ListedColormap(colormaps["viridis"](np.linspace(0.25, 1.0, 256)))
DESVIO = colormaps["RdYlGn_r"].with_extremes(bad="#6b7280")  # (cinza: sem clipe naquele quadro)
DESVIO_MAX = 0.30  # (m: a distancia ao clipe que satura no vermelho)
RASTROS = (("ponta_e", ESQ, "-"), ("calcanhar_e", ESQ, "--"), ("joelho_e", ESQ, ":"), ("mao_e", ESQ, "-."),
           ("ponta_d", DIR, "-"), ("calcanhar_d", DIR, "--"), ("joelho_d", DIR, ":"), ("mao_d", DIR, "-."),
           ("cabeca", CABECA, "-"), ("pelve", "#9aa4b2", "--"), ("centro_massa", "#ff3d3d", "-"))
CONTORNO = [patheffects.withStroke(linewidth=2.4, foreground=FUNDO)]  # (as linhas legiveis sobre o fantasma)
LARGURA, ALTURA, ESCALA_MAX = 1800.0, 1500.0, 200.0  # (px da area dos paineis e o teto de px por metro)

_X, _Y, _Z = np.eye(3)


def _obliqua(az: float = 45.0, el: float = 25.0) -> tuple[np.ndarray, np.ndarray]:
    """A vista 3/4: a camera a `az` graus da frente para a esquerda e `el` graus acima."""
    a, e = np.radians(az), np.radians(el)
    o = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    u = np.cross(_Z, o)
    u /= np.linalg.norm(u)
    return u, np.cross(o, u)


# As vistas no referencial do corpo (x = frente, y = esquerda, z = cima): (a direita da tela, o alto da tela).
VISTAS = {"frente": (_Y, _Z), "costas": (-_Y, _Z), "lado_esq": (-_X, _Z), "lado_dir": (_X, _Z), "cima": (_X, _Y),
          "obliqua": _obliqua()}
# (Costas e lado_esq sao o espelho de frente e lado_dir na projecao ortografica: no fantasma so mudam a ordem de desenho.)
VISTAS_LUGAR = ("frente", "lado_dir", "cima", "obliqua")
TITULOS = {"frente": "frente", "costas": "costas", "lado_esq": "lado esquerdo", "lado_dir": "lado direito",
           "cima": "cima (frente →)", "obliqua": "oblíqua 3/4 (frente-esq.)"}
ELEVACAO = ("frente", "costas", "lado_esq", "lado_dir")
# O espelho: cada elo e o do outro lado.
TROCA = np.array([G.IDX[n.replace("Left", "#").replace("Right", "Left").replace("#", "Right")] for n in G.LINKS])
LADO = np.array([1 if n.startswith("Left") else -1 if n.startswith("Right") else 0 for n in G.LINKS])
_PARTES = {"Pelvis": "pelve", "Abdomen": "abdome", "Chest": "peito", "UpperChest": "peito alto", "Neck": "pescoço",
           "Head": "cabeça", "UpperArm": "braço", "Forearm": "antebraço", "Hand": "mão", "Thigh": "coxa", "Shin": "canela",
           "Foot": "pé"}
# (O contorno da caixa projetada: os 6 cantos fora do mais perto e do mais longe, em sinais relativos aos da vista.)
_HEX = np.array([(1, -1, 1), (1, -1, -1), (1, 1, -1), (-1, 1, -1), (-1, 1, 1), (-1, -1, 1)])
_ARCO = 7
_PAD = np.linspace(0, 6, 2 * _ARCO, endpoint=False).astype(int)


def _nome_elo(n: str) -> str:
    for lado, s in (("Left", " esq."), ("Right", " dir.")):
        if n.startswith(lado):
            return _PARTES[n[len(lado):]] + s
    return _PARTES[n]


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """O produto de quaternios xyzw a*b, vetorizado."""
    av, aw, bv, bw = a[..., :3], a[..., 3:], b[..., :3], b[..., 3:]
    return np.concatenate([aw * bv + bw * av + np.cross(av, bv), aw * bw - (av * bv).sum(-1, keepdims=True)], -1)


# --- A janela no referencial do corpo --------------------------------------------------------------------------------

@dataclass
class Janela:
    """Um mundo de uma captura numa janela, no referencial do corpo: x = a frente, y = a esquerda, z = cima (no lugar a frente
    suavizada de cada passo e a origem na pelve dele; no mundo a frente media da janela e a origem na pelve do primeiro)."""
    cap: G.Captura
    mundo: int
    modo: str
    t: np.ndarray              # [F] o tempo da captura (s)
    corpo: np.ndarray          # [F, 18, 7]
    ref: np.ndarray | None     # [F, 18, 7] o clipe (nan sem clipe); None sem clipe na janela
    geo: list
    juntas: dict               # _juntas, [F, 3] cada
    contatos: np.ndarray       # [N, 6]: x, y, z, elo, quadro, impulso
    desvio: np.ndarray         # [F, 18] (m) a distancia de cada elo ao clipe
    medidas: dict              # as inclinacoes medias (graus)
    pedido: float | None       # o rumo pedido contra o rumo medio do corpo (rad); None sem pedido de deslocamento
    trajeto: float             # o rumo do trajeto menos o pedido (graus, + = esquerda); nan sem pedido fixo
    passada: float | None = None  # o periodo da passada (s); None sem ciclo
    espelhada: bool = False
    rumo_mundo: float = float("nan")  # o rumo do eixo x da janela no mundo (rad)
    espalhar: float = 0.0             # (m/s) o deslocamento de mentira em x por tempo (o movimento no lugar espalhado)


# (A ponta e o calcanhar estaveis e o referencial do abdome agora vem de geometria; contra a esquerda do trajeto nos
# exercicios retos, a inclinacao de lado pelo abdome erra ~0,4 grau em media e ate 1 grau na corrida - o abdome gira +-20
# graus com os bracos: diferencas abaixo de 1 grau entre exercicios nao sao conclusivas.)
_juntas = G.articulacoes
_quadro = G.referencial


def _rumo(fr: np.ndarray) -> np.ndarray:
    """[F] o rumo do corpo pelo abdome (rad, desenrolado); deitado de lado (o eixo dele quase vertical, sem rumo) vem dos
    quadros vizinhos."""
    e = G.qrot(fr[:, G.IDX["Abdomen"], 3:7], np.array([1.0, 0.0, 0.0]))
    ok = np.hypot(e[:, 0], e[:, 1]) > 0.3
    ok = ok if ok.any() else np.ones_like(ok)
    return np.interp(np.arange(len(e)), np.flatnonzero(ok), np.unwrap(np.arctan2(-e[ok, 0], e[ok, 1])))


def _passada(fr: np.ndarray, geo: list, dt: float) -> float | None:
    """O periodo da passada (s): o primeiro pico (> 0,5) da autocorrelacao do vetor entre os tornozelos no referencial do
    corpo (a frente e a esquerda de cada passo: o giro e as curvas nao o embaralham), entre 0,3 e 1,5 s; None sem ciclo
    (os pes parados, o levantar)."""
    j, f = G.articulacoes(fr, geo), _quadro(fr)
    v = j["tornozelo_e"] - j["tornozelo_d"]
    d = np.stack([np.einsum("fc,fc->f", v, f["frente"]), np.einsum("fc,fc->f", v, f["esquerda"])], -1)
    d = d - d.mean(0)
    n = len(d)
    if np.sqrt((d ** 2).sum(1).mean()) < 0.05:  # (o parado: os pes balancam milimetros)
        return None
    ac = np.array([(d[:n - k] * d[k:]).sum() / (n - k) for k in range(n // 2)])
    ac /= ac[0]
    for k in range(1, len(ac) - 1):
        if 0.3 <= k * dt <= 1.5 and ac[k] > 0.5 and ac[k] >= max(ac[k - 1], ac[k + 1]):
            return k * dt
    return None


def _de_pe(j: dict) -> np.ndarray:
    """Os quadros de pe (a pelve acima de 0,6 m): so neles a inclinacao e o eixo medio fazem sentido (o levantar comeca
    deitado). Sem nenhum, todos."""
    m = j["pelve"][:, 2] > 0.6
    return m if m.any() else np.ones_like(m)


def _medidas(j: dict, fr: np.ndarray) -> dict:
    """As inclinacoes medias nos quadros de pe (graus), no referencial do abdome de cada passo: de lado (+ = esquerda) do
    corpo (tornozelos -> cabeca) e do tronco (pelve -> pescoco), e do tronco para a frente."""
    m = _de_pe(j)
    f = _quadro(fr)
    base = 0.5 * (j["tornozelo_e"] + j["tornozelo_d"])
    return {"lado_corpo": float(np.mean(G.inclinacao(j["cabeca"], base, f["esquerda"])[m])),
            "lado_tronco": float(np.mean(G.inclinacao(j["pescoco"], j["pelve"], f["esquerda"])[m])),
            "frente_tronco": float(np.mean(G.inclinacao(j["pescoco"], j["pelve"], f["frente"])[m]))}


def _eixo_medio(j: dict) -> tuple[np.ndarray, np.ndarray]:
    """A base media no chao (o meio dos tornozelos) e a cabeca media, nos quadros de pe."""
    m = _de_pe(j)
    return (0.5 * (j["tornozelo_e"] + j["tornozelo_d"]))[m].mean(0) * [1, 1, 0], j["cabeca"][m].mean(0)


def _desvio(fr: np.ndarray, rf: np.ndarray, geo: list) -> np.ndarray:
    """[F, 18] (m): a distancia media de cada elo ao do clipe (as pontas da capsula; os cantos do pe)."""
    out = np.empty(fr.shape[:2])
    for k, g in enumerate(geo):
        if g["forma"] == "capsule":
            a1, b1, _ = G.capsula(fr[:, k], g)
            a2, b2, _ = G.capsula(rf[:, k], g)
            out[:, k] = 0.5 * (np.linalg.norm(a1 - a2, axis=-1) + np.linalg.norm(b1 - b2, axis=-1))
        else:
            out[:, k] = np.linalg.norm(G.caixa(fr[:, k], g) - G.caixa(rf[:, k], g), axis=-1).mean(-1)
    return out


def _rumo_caminho(fr: np.ndarray, info: np.ndarray, medio: float) -> float:
    """O rumo do caminho (rad, mundo): o pedido quando ele e fixo na janela; senao o eixo principal do trajeto da pelve
    (orientado do comeco ao fim) quando ele passa de 0,5 m; senao o rumo medio do corpo (o parado, o giro)."""
    if np.nanmean(info[:, 3]) > 0.3:
        cmd = float(np.arctan2(np.nanmean(np.sin(info[:, 4])), np.nanmean(np.cos(info[:, 4]))))
        if np.nanmax(np.abs(_wrap(info[:, 4] - cmd))) < 0.1:
            return cmd
    p = fr[:, 0, :2] - fr[:, 0, :2].mean(0)
    if np.ptp(p, 0).max() > 0.5:
        e = np.linalg.svd(p, full_matrices=False)[2][0]
        e = -e if e @ (fr[-1, 0, :2] - fr[0, 0, :2]) < 0 else e
        return float(np.arctan2(e[1], e[0]))
    return medio


def janela(cap: G.Captura, mundo: int = 0, intervalo: tuple[float, float] | None = None, modo: str = "lugar") -> Janela:
    """A janela [t0, t1) s do `mundo` (padrao: de 1,5 s, ou 0 no lev_levantar, ao fim do episodio), sem atravessar o fim de
    um episodio, no referencial do corpo: no lugar o rumo de cada passo suavizado numa passada (o fantasma acompanha as
    curvas e o giro sem o balanco da passada), no mundo o rumo medio da janela, no caminho (o v2) o x no rumo do caminho
    (_rumo_caminho) e a origem na pelve do primeiro quadro."""
    if not 0 <= mundo < cap.corpo.shape[0]:
        raise SystemExit(f"{cap.nome}: nao ha o mundo {mundo} (0 a {cap.corpo.shape[0] - 1})")
    dt, n = cap.dt, cap.corpo.shape[1]
    col = cap.meta["colunas_episodios"]
    fins = sorted(int(r[col.index("passo_fim")]) for r in cap.episodios if int(r[col.index("mundo")]) == mundo)
    t0, t1 = intervalo or (0.0 if cap.meta["exercicio"] == "lev_levantar" else 1.5, np.inf)
    i0 = int(round(t0 / dt))
    # (O passo do fim ja traz o corpo novo do episodio seguinte: um episodio vai de um fim ate o proximo, sem ele.)
    limites = [0] + fins + [n]
    k = min(int(np.searchsorted(limites, i0, side="right")) - 1, len(limites) - 2)
    passos = np.arange(i0, min(limites[k + 1], int(round(t1 / dt)) if np.isfinite(t1) else n))
    passos = passos[np.isfinite(cap.corpo[mundo, passos, 0, 0])]
    if len(passos) < 2:
        raise SystemExit(f"{cap.nome}: janela vazia ({t0}-{t1} s, mundo {mundo})")
    fr = cap.corpo[mundo, passos].astype(float)
    passada = _passada(fr, cap.geo, dt)
    rumo = _rumo(fr)
    medio = float(np.arctan2(np.sin(rumo).mean(), np.cos(rumo).mean()))
    info = cap.info[mundo, passos]
    if modo == "lugar":
        # (A media movel de uma passada; a reflexao impar nas pontas segue a rampa do giro sem atraso.)
        h = min(int(round(0.5 * (passada or 0.7) / dt)), len(rumo) - 1)
        psi = np.convolve(np.pad(rumo, h, mode="reflect", reflect_type="odd"), np.full(2 * h + 1, 1.0 / (2 * h + 1)), "valid")
        origem = fr[:, 0, :2]
    else:
        eixo = _rumo_caminho(fr, info, medio) if modo == "caminho" else medio
        psi, origem = np.full(len(fr), eixo), np.repeat(fr[:1, 0, :2], len(fr), 0)
    c, s = np.cos(psi), np.sin(psi)
    gira = np.stack([np.stack([c, -s], -1), np.stack([s, c], -1)], -2)  # [F, 2, 2]
    qz = np.stack([0.0 * psi, 0.0 * psi, np.sin(-psi / 2), np.cos(-psi / 2)], -1)[:, None]

    def local(f: np.ndarray) -> np.ndarray:
        out = f.astype(float)
        out[..., :2] = np.einsum("fkc,fcd->fkd", f[..., :2] - origem[:, None], gira)
        out[..., 3:] = _qmul(qz, f[..., 3:])
        return out

    corpo = local(fr)
    rf = cap.referencia[mundo, passos]
    ref = local(rf) if np.isfinite(rf[..., 0]).any() else None
    ct = cap.contatos[cap.contatos[:, 0] == mundo]
    ct = ct[np.isin(ct[:, 1].astype(int), passos)]
    q = np.searchsorted(passos, ct[:, 1].astype(int))
    xy = np.einsum("nc,ncd->nd", ct[:, 2:4] - origem[q], gira[q])
    pedido, trajeto = None, float("nan")
    if np.nanmean(info[:, 3]) > 0.3:
        cmd = float(np.arctan2(np.nanmean(np.sin(info[:, 4])), np.nanmean(np.cos(info[:, 4]))))
        pedido = float(_wrap(cmd - psi[0])) if modo == "caminho" else float(_wrap(cmd - medio))
        if np.nanmax(np.abs(_wrap(info[:, 4] - cmd))) < 0.1:
            d = fr[-1, 0, :2] - fr[0, 0, :2]
            trajeto = float(np.degrees(_wrap(np.arctan2(d[1], d[0]) - cmd)))
    juntas = _juntas(corpo, cap.geo)
    return Janela(cap, mundo, modo, passos * dt, corpo, ref, cap.geo, juntas, np.column_stack([xy, ct[:, 4:6], q, ct[:, 9]]),
                  _desvio(corpo, ref, cap.geo) if ref is not None else np.full(corpo.shape[:2], np.nan),
                  _medidas(juntas, corpo), pedido, trajeto, passada, rumo_mundo=float(psi.mean()) if modo != "lugar"
                  else float("nan"))


def espelhar(jan: Janela) -> Janela:
    """A janela refletida no plano sagital do corpo (y -> -y) com os elos trocados de lado: o movimento como seria no
    exercicio espelhado."""
    def m(f):
        if f is None:
            return None
        out = f[:, TROCA].copy()
        out[..., 1] *= -1.0
        out[..., 3] *= -1.0  # (o quaternio refletido: (x, y, z, w) -> (-x, y, -z, w))
        out[..., 5] *= -1.0
        return out

    corpo, geo = m(jan.corpo), [jan.geo[i] for i in TROCA]
    ct = jan.contatos.copy()
    ct[:, 1] *= -1.0
    ct[:, 3] = TROCA[ct[:, 3].astype(int)]
    # (A reflexao so troca o sinal das inclinacoes de lado; o x do abdome refletido apontaria para a direita do corpo.)
    medidas = dict(jan.medidas, lado_corpo=-jan.medidas["lado_corpo"], lado_tronco=-jan.medidas["lado_tronco"])
    return replace(jan, corpo=corpo, ref=m(jan.ref), geo=geo, juntas=_juntas(corpo, geo), contatos=ct,
                   desvio=jan.desvio[:, TROCA], medidas=medidas, pedido=None if jan.pedido is None else -jan.pedido,
                   trajeto=-jan.trajeto, espelhada=True)


# --- A projecao ------------------------------------------------------------------------------------------------------

def _silhuetas(fr: np.ndarray, geo: list, u: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """As silhuetas projetadas [F, 18, 14, 2] (a capsula: o estadio afunilado entre os centros das tampas; o pe: o hexagono
    da caixa) e a profundidade de cada elo [F, 18] (maior = mais perto da camera)."""
    o = np.cross(u, v)
    P = np.stack([u, v], 1)
    out = np.empty(fr.shape[:2] + (2 * _ARCO, 2))
    t = np.linspace(0.0, 1.0, _ARCO)
    for k, g in enumerate(geo):
        if g["forma"] == "capsule":
            a, b, _ = G.capsula(fr[:, k], g)
            A, B = a @ P, b @ P
            r1, r2 = g["raio"], g.get("raio_mais_x", g["raio"])
            e = B - A
            th = np.arctan2(e[:, 1], e[:, 0])[:, None]
            # (A tangente externa de dois circulos: o arco do lado de b cobre 2*ph em volta do eixo, o de a o resto.)
            ph = np.arccos(np.clip((r1 - r2) / np.maximum(np.linalg.norm(e, axis=-1), 1e-9), -1.0, 1.0))[:, None]
            ang = np.concatenate([th + ph * (2 * t - 1), th + ph + (2 * np.pi - 2 * ph) * t], 1)
            cen = np.concatenate([np.repeat(B[:, None], _ARCO, 1), np.repeat(A[:, None], _ARCO, 1)], 1)
            rad = np.repeat([r2, r1], _ARCO)[None, :, None]
            out[:, k] = cen + rad * np.stack([np.cos(ang), np.sin(ang)], -1)
        else:
            c = G.caixa(fr[:, k], g)
            s = np.where(np.einsum("fkc,c->fk", c[:, [4, 2, 1]] - c[:, :1], o) >= 0, 1, -1)
            idx = ((_HEX[None] * s[:, None]) > 0).astype(int) @ np.array([4, 2, 1])
            out[:, k] = (np.take_along_axis(c, idx[..., None].repeat(3, -1), 1) @ P)[:, _PAD]
    return out, fr[..., :3] @ o


def _eixos(jan: Janela, vista: str) -> tuple[np.ndarray, np.ndarray]:
    """(u, v) da vista; a de cima no modo mundo segue o rumo pedido (o caminho pedido na horizontal)."""
    if vista == "cima" and jan.modo == "mundo" and jan.pedido is not None:
        a = jan.pedido
        return np.array([np.cos(a), np.sin(a), 0.0]), np.array([-np.sin(a), np.cos(a), 0.0])
    return VISTAS[vista]


def _titulo_vista(jan: Janela, vista: str) -> str:
    m = jan.medidas
    if vista in ("frente", "costas"):
        return f"{TITULOS[vista]}\nlado: corpo {m['lado_corpo']:+.1f}°, tronco {m['lado_tronco']:+.1f}°"
    if vista in ("lado_esq", "lado_dir"):
        return f"{TITULOS[vista]}\ntronco à frente {m['frente_tronco']:+.1f}°"
    if vista == "cima" and jan.modo == "mundo" and jan.pedido is not None:
        extra = f"trajeto {jan.trajeto:+.1f}° do pedido" if np.isfinite(jan.trajeto) else "rumo pedido variável"
        return f"cima (rumo pedido →)\n{extra}"
    return TITULOS[vista]


def _limites(vista: str, *pontos: np.ndarray, margem: float = 0.12) -> tuple[float, float, float, float]:
    p = np.concatenate([a.reshape(-1, 2) for a in pontos if a is not None and a.size])
    p = p[np.isfinite(p).all(1)]
    lo, hi = p.min(0), p.max(0)
    if vista in ELEVACAO:
        return lo[0] - margem, hi[0] + margem, min(lo[1], 0.0) - 0.08, hi[1] + margem
    return lo[0] - margem, hi[0] + margem, lo[1] - margem, hi[1] + margem


def _cenario(ax, jan: Janela, vista: str, u: np.ndarray, v: np.ndarray, eixo: bool = True, vertical: bool = True) -> None:
    """A grade de 0,5 m, o chao, a vertical na base e (frente/costas) o eixo medio base -> cabeca."""
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    base, cab = _eixo_medio(jan.juntas)
    if vista == "obliqua":
        pts = jan.corpo[..., :2].reshape(-1, 2)
        lo, hi = np.floor(pts.min(0) / 0.5) * 0.5 - 0.5, np.ceil(pts.max(0) / 0.5) * 0.5 + 0.5
        segs = [[(gx, lo[1], 0), (gx, hi[1], 0)] for gx in np.arange(lo[0], hi[0] + 1e-6, 0.5)]
        segs += [[(lo[0], gy, 0), (hi[0], gy, 0)] for gy in np.arange(lo[1], hi[1] + 1e-6, 0.5)]
        ax.add_collection(LineCollection(np.array(segs) @ np.stack([u, v], 1), colors=GRADE, linewidths=0.7, zorder=0))
        topo = base + [0, 0, 2.0]
        ax.plot([base @ u, topo @ u], [base @ v, topo @ v], color=CHAO, ls="--", lw=0.8, zorder=1)
        return
    gy = np.arange(np.ceil(y0 / 0.5) * 0.5, y1, 0.5)
    ax.vlines(np.arange(np.ceil(x0 / 0.5) * 0.5, x1, 0.5), 0.0 if vista in ELEVACAO else y0, y1, colors=GRADE,
              linewidths=0.7, zorder=0)
    ax.hlines(gy[gy >= 0] if vista in ELEVACAO else gy, x0, x1, colors=GRADE, linewidths=0.7, zorder=0)
    if vista == "cima":
        ax.plot(base @ u, base @ v, "+", color=CHAO, ms=12, mew=1.2, zorder=5)
        return
    ax.axhline(0.0, color=CHAO, lw=1.3, zorder=1)
    if vertical:
        ax.axvline(base @ u, color=CHAO, ls="--", lw=0.8, zorder=1)
    if eixo and vista in ("frente", "costas"):
        ax.plot([base @ u, cab @ u], [0.0, cab @ v], color=EIXO, lw=1.5, zorder=6, path_effects=CONTORNO)
        ax.plot(cab @ u, cab @ v, "o", color=EIXO, ms=4, zorder=6, path_effects=CONTORNO)


def _rastros(ax, jan: Janela, u: np.ndarray, v: np.ndarray) -> None:
    P = np.stack([u, v], 1)
    for nome, cor, ls in RASTROS:
        p = jan.juntas[nome] @ P
        ax.plot(p[:, 0], p[:, 1], color=cor, ls=ls, lw=0.9, zorder=4, path_effects=CONTORNO)


def _contatos(ax, jan: Janela, u: np.ndarray, v: np.ndarray) -> None:
    c = jan.contatos
    p = c[:, :3] @ np.stack([u, v], 1)
    elo = c[:, 3].astype(int)
    for sel, cor, mk, s in ((elo == G.IDX["LeftFoot"], ESQ, "o", 4), (elo == G.IDX["RightFoot"], DIR, "o", 4),
                            (~np.isin(elo, [G.IDX["LeftFoot"], G.IDX["RightFoot"]]), "#ffffff", "x", 14)):
        ax.scatter(p[sel, 0], p[sel, 1], s=s, c=cor, marker=mk, linewidths=0.8, alpha=0.55, zorder=5)


# --- O quadro: paineis numa escala unica -----------------------------------------------------------------------------

@dataclass
class _Painel:
    lims: tuple[float, float, float, float]
    chao: bool                     # ancora o chao na mesma altura dos outros da linha
    titulo: str
    desenhar: Callable             # (ax) -> None, com os limites ja postos


def _montar(linhas: list, titulo: str, legenda: list | None = None, barra: tuple | None = None) -> Figure:
    """A figura numa escala unica (px por metro): linhas [(cabecalho, [_Painel])]; cada coluna tem a faixa x da uniao das
    linhas, cada linha a altura do maior painel, e as vistas de elevacao o chao na mesma altura."""
    ncol = max(len(p) for _, p in linhas)
    faixa = [(min(p[c].lims[0] for _, p in linhas if c < len(p)), max(p[c].lims[1] for _, p in linhas if c < len(p)))
             for c in range(ncol)]
    larg = [b - a for a, b in faixa]
    alt = [max(q.lims[3] - q.lims[2] for q in p) for _, p in linhas]
    esc = min(LARGURA / sum(larg), ALTURA / sum(alt), ESCALA_MAX)
    # (Cada coluna com 250 px ou mais, aberta dos dois lados: o titulo de um painel estreito nao invade o do vizinho.)
    faixa = [(0.5 * (a + b - max(b - a, 250 / esc)), 0.5 * (a + b + max(b - a, 250 / esc))) for a, b in faixa]
    larg = [b - a for a, b in faixa]
    gap, cab, tit, topo, mx = 16, 22, 34, 58, 14
    W = max(2 * mx + sum(larg) * esc + gap * (ncol - 1), 900.0)  # (900 px: cabe o cabecalho mais longo, ~125 letras)
    # (A legenda a direita da barra, em colunas da largura do rotulo mais longo, ~6,5 px por letra mais a amostra; o
    # rodape cresce com as linhas dela.)
    x_leg = mx + (min(260, W / 4) + 50 if barra else 0)
    n_leg = len(legenda or [])
    larg_leg = 60 + 6.5 * max((len(h.get_label()) for h in legenda or []), default=0)
    col_leg = max(1, min(n_leg, int((W - x_leg - mx) // larg_leg)))
    rodape = max(96 if legenda or barra else 16, 24 + 17 * -(-n_leg // col_leg))
    H = topo + rodape + sum(cab + 15 * c.count("\n") + tit + h * esc for (c, _), h in zip(linhas, alt))
    fig = Figure(figsize=(W / 100, H / 100), dpi=100, facecolor=FUNDO)
    fig.text(mx / W, 1 - 12 / H, titulo, color=TEXTO, fontsize=11.5, va="top", weight="bold")
    y = H - topo
    for (cabecalho, paineis), h in zip(linhas, alt):
        y -= cab + 15 * cabecalho.count("\n")
        fig.text(mx / W, (y + 4) / H, cabecalho, color=TEXTO, fontsize=10, va="bottom", linespacing=1.25)
        y -= tit + h * esc
        x = mx
        chao = min((q.lims[2] for q in paineis if q.chao), default=0.0)
        for c, q in enumerate(paineis):
            ax = fig.add_axes((x / W, y / H, larg[c] * esc / W, h * esc / H))
            y0 = chao if q.chao else 0.5 * (q.lims[2] + q.lims[3] - h)
            ax.set_xlim(*faixa[c])
            ax.set_ylim(y0, y0 + h)
            ax.set_facecolor(FUNDO)
            ax.set_xticks([])
            ax.set_yticks([])
            for sp in ax.spines.values():
                sp.set_color(GRADE)
            ax.set_title(q.titulo, color=TEXTO, fontsize=9, loc="left", pad=4)
            q.desenhar(ax)
            x += larg[c] * esc + gap
    if barra:
        cmap, norm, rotulo, ticks = barra
        cax = fig.add_axes(((mx + 18) / W, 40 / H, min(260, W / 4) / W, 11 / H))
        cb = fig.colorbar(ScalarMappable(norm, cmap), cax=cax, orientation="horizontal")
        cax.set_title(rotulo, color=TEXTO, fontsize=8.5, loc="left", pad=3)
        if ticks:
            cb.set_ticks(list(ticks), labels=list(ticks.values()))
        cb.ax.tick_params(colors=TEXTO, labelsize=8)
        cb.outline.set_edgecolor(GRADE)
    if legenda:
        fig.legend(handles=legenda, loc="lower left", bbox_to_anchor=(x_leg / W, 4 / H), ncol=col_leg,
                   frameon=False, fontsize=8.5, labelcolor=TEXTO, handlelength=2.6, columnspacing=1.2, labelspacing=0.35)
    return fig


def _rotulo(jan: Janela) -> str:
    m = jan.cap.meta
    modo = "clipe puro (ação zero, mão cheia)" if m["clipe_puro"] else f"rede {m['modo']}"
    esp = " · REFLETIDO" if jan.espelhada else ""
    return (f"{m['exercicio']} · {modo} · força {m['forca']:.0%} · r{m['atualizacao']} · mundo {jan.mundo} · "
            f"{jan.t[0]:.1f}–{jan.t[-1] + jan.cap.dt:.1f} s{_fim(jan)}{esp}")


def _legenda(modo: str, desvio: bool) -> list:
    fixa = [Line2D([], [], color=REF, lw=1.1, label="envoltória do clipe" if modo == "lugar" else "clipe (cada quadro)"),
            Line2D([], [], color=EIXO, lw=1.4, marker="o", ms=3.5, label="eixo médio base→cabeça (de pé)"),
            Line2D([], [], color=CHAO, ls="--", label="vertical na base (grade 0,5 m)")]
    if desvio:
        return fixa
    return fixa + [Line2D([], [], color=ESQ, lw=2.5, label="esquerda"), Line2D([], [], color=DIR, lw=2.5, label="direita"),
                   Line2D([], [], color=CENTRO, ls="-", label="ponta do pé"),
                   Line2D([], [], color=CENTRO, ls="--", label="calcanhar"),
                   Line2D([], [], color=CENTRO, ls=":", lw=1.4, label="joelho"), Line2D([], [], color=CENTRO, ls="-.", label="mão"),
                   Line2D([], [], color=CABECA, label="cabeça"), Line2D([], [], color="#9aa4b2", ls="--", label="pelve"),
                   Line2D([], [], color="#ff3d3d", label="centro de massa"),
                   Line2D([], [], color="#ffffff", marker="x", ls="", label="contato fora dos pés")]


# --- As figuras ------------------------------------------------------------------------------------------------------

def _painel_fantasma(jan: Janela, vista: str, quadros: np.ndarray, desvio: bool) -> _Painel:
    u, v = _eixos(jan, vista)
    sil, prof = _silhuetas(jan.corpo[quadros], jan.geo, u, v)
    ordem = np.argsort(prof, 1)  # (do mais longe ao mais perto em cada quadro)
    polys = np.take_along_axis(sil, ordem[..., None, None], 1)
    if desvio:
        cor = DESVIO(np.clip(np.take_along_axis(jan.desvio[quadros], ordem, 1) / DESVIO_MAX, 0.0, 1.0))
    else:
        tt = (jan.t[quadros] - jan.t[0]) / max(jan.t[-1] - jan.t[0], 1e-9)
        cor = np.repeat(TEMPO(tt)[:, None], sil.shape[1], 1)
    cor = cor.reshape(-1, 4)
    cor[:, 3] = _alfa(len(quadros))
    polys = polys.reshape(-1, *sil.shape[2:])
    rpolys = None
    if jan.ref is not None:
        r, _ = _silhuetas(jan.ref[quadros], jan.geo, u, v)
        rpolys = r.reshape(-1, *r.shape[2:])
        rpolys = rpolys[np.isfinite(rpolys).all((1, 2))]
    P = np.stack([u, v], 1)
    rastros = [jan.juntas[n] @ P for n, _, _ in RASTROS]

    def desenhar(ax):
        _cenario(ax, jan, vista, u, v)
        ax.add_collection(PolyCollection(polys, facecolors=cor, edgecolors="none", zorder=2))
        if rpolys is not None and len(rpolys) and jan.modo == "lugar":
            # (No lugar, a envoltoria do clipe: o contorno do que ele visita em 2 quadros ou mais.)
            lims = (*ax.get_xlim(), *ax.get_ylim())
            occ = _ocupacao(rpolys, lims, (int(round(ax.bbox.height)), int(round(ax.bbox.width))), 0.05)
            ax.contour(occ, levels=[1.0 - 0.95 ** 2], extent=lims, origin="upper", colors=REF, linewidths=1.1,
                       zorder=3).set_path_effects(CONTORNO)
        elif rpolys is not None and len(rpolys):
            ax.add_collection(PolyCollection(rpolys, facecolors="none", edgecolors=REF, linewidths=0.6, alpha=0.55,
                                             zorder=3))
        if not desvio:
            _rastros(ax, jan, u, v)
            if vista == "cima":
                _contatos(ax, jan, u, v)

    return _Painel(_limites(vista, sil, rpolys, *rastros), vista in ELEVACAO, _titulo_vista(jan, vista), desenhar)


def _alfa(n: int) -> float:
    """O alfa de cada silhueta do fantasma: um quadro so ainda aparece, o tronco (todos os quadros) satura."""
    return float(np.clip(7.0 / n, 0.03, 0.45))


def _estrobo(jan: Janela) -> float:
    """O intervalo (s) do estroboscopio no mundo: um corpo a cada metro andado, entre 0,2 e 0,5 s."""
    p = jan.juntas["pelve"][:, :2]
    vel = np.linalg.norm(p[-1] - p[0]) / max(jan.t[-1] - jan.t[0], 1e-9)
    return float(np.clip(1.0 / max(vel, 1e-9), 0.2, 0.5))


def _quadros(jan: Janela) -> np.ndarray:
    """Todos os quadros no lugar; os do estroboscopio no mundo."""
    return np.arange(len(jan.t)) if jan.modo == "lugar" else np.arange(0, len(jan.t), max(1, round(_estrobo(jan) / jan.cap.dt)))


def vistas_mundo(jan: Janela) -> tuple[str, ...]:
    """As vistas do modo mundo: a de cima e a de elevacao em que o caminho pedido corre na horizontal."""
    lateral = jan.pedido is not None and abs(np.sin(jan.pedido)) > abs(np.cos(jan.pedido))
    return ("cima", "frente" if lateral else "lado_dir")


def fantasma(jans: list[Janela], vistas, desvio: bool = False) -> Figure:
    """O fantasma: no lugar uma linha por janela e uma coluna por vista; no mundo (faixas compridas) um painel por linha."""
    linhas = []
    for jan in jans:
        cab = _rotulo(jan)
        if jan.modo == "mundo":
            cab += f" · um corpo a cada {_estrobo(jan):.2f} s"
        if desvio and np.isfinite(jan.desvio).any():
            d = np.nanmean(jan.desvio, 0)
            pior = np.argsort(d)[::-1][:3]
            cab += (f"\ndistância ao clipe: mediana {np.nanmedian(jan.desvio) * 100:.0f} cm · piores: "
                    + ", ".join(f"{_nome_elo(G.LINKS[k])} {d[k] * 100:.0f} cm" for k in pior))
        paineis = [_painel_fantasma(jan, v, _quadros(jan), desvio) for v in vistas]
        linhas += [(cab, paineis)] if jan.modo == "lugar" else [(cab, [p]) for p in paineis]
    modo = jans[0].modo
    titulo = ("Fantasma no lugar: pelve travada no plano (o ciclo empilhado)" if modo == "lugar" else
              "Fantasma no mundo: estroboscópico ao longo do caminho")
    titulo += (f"\nvistas pelo abdome, rumo {'suavizado na passada' if modo == 'lugar' else 'médio'} · + = esquerda · "
               "grade 0,5 m")
    if desvio:
        barra = (DESVIO, Normalize(0, DESVIO_MAX * 100), "distância de cada elo ao clipe (cm)", None)
    else:
        barra = (TEMPO, Normalize(0, 1), "tempo na janela", {0: "início", 1: "fim"})
    return _montar(linhas, titulo, _legenda(modo, desvio), barra)


def _painel_folha(jan: Janela, vista: str, quadros: np.ndarray, desvio: bool, sil: np.ndarray, prof: np.ndarray,
                  passo: float) -> _Painel:
    u, v = VISTAS[vista]
    sil = sil.copy()
    dx = np.arange(len(quadros)) * passo
    sil[..., 0] += dx[:, None, None]
    ordem = np.argsort(prof, 1)
    polys = np.take_along_axis(sil, ordem[..., None, None], 1).reshape(-1, *sil.shape[2:])
    if desvio:
        cor = DESVIO(np.clip(np.take_along_axis(jan.desvio[quadros], ordem, 1) / DESVIO_MAX, 0.0, 1.0)).reshape(-1, 4)
    else:
        lado = np.array([CENTRO, ESQ, DIR])[LADO]
        cor = np.take_along_axis(np.broadcast_to(lado, prof.shape), ordem, 1).reshape(-1)
    rpolys = None
    if jan.ref is not None:
        r, _ = _silhuetas(jan.ref[quadros], jan.geo, u, v)
        r[..., 0] += dx[:, None, None]
        rpolys = r.reshape(-1, *r.shape[2:])
        rpolys = rpolys[np.isfinite(rpolys).all((1, 2))]
    c = jan.contatos
    pes = np.isin(c[:, 3].astype(int), [G.IDX["LeftFoot"], G.IDX["RightFoot"]])
    base = (0.5 * (jan.juntas["tornozelo_e"] + jan.juntas["tornozelo_d"]))[quadros] @ u + dx

    def desenhar(ax):
        _cenario(ax, jan, vista, u, v, eixo=False, vertical=False)
        ax.vlines(base, 0.0, ax.get_ylim()[1], colors=CHAO, linestyles="--", linewidths=0.8, zorder=1)
        ax.add_collection(PolyCollection(polys, facecolors=cor, edgecolors=FUNDO, linewidths=0.5, zorder=2))
        if rpolys is not None and len(rpolys):
            ax.add_collection(PolyCollection(rpolys, facecolors="none", edgecolors=REF, linewidths=0.8, alpha=0.85,
                                             zorder=3))
        for i, q in enumerate(quadros):
            ax.text(dx[i], -0.05, f"{jan.t[q]:.2f} s", color=TEXTO, fontsize=8, ha="center", va="top", zorder=6)
            sel = pes & (c[:, 4] == q)
            for elo, cor_pe in ((G.IDX["LeftFoot"], ESQ), (G.IDX["RightFoot"], DIR)):
                p = c[sel & (c[:, 3] == elo), :3]
                if len(p):
                    ax.plot(p.mean(0) @ u + dx[i], 0.0, "^", color=cor_pe, ms=6, mec=FUNDO, zorder=6)

    lims = _limites(vista, sil, rpolys)
    return _Painel((lims[0], lims[1], lims[2] - 0.1, lims[3]), True, "", desenhar)


def _poses(jan: Janela, n: int) -> np.ndarray:
    """Os quadros da folha: com ciclo, n poses numa passada desde o pouso do pe esquerdo mais perto do meio da janela (as
    linhas na mesma fase); sem ciclo, na janela inteira igualmente espacadas no movimento, nao no tempo (o levantar, 1 s
    de 6, ganha metade delas). (Espalhadas no tempo da janela, as poses de um trote caiam a 0,72 s, quase uma passada: a
    folha repetia a mesma fase.)"""
    F = len(jan.t)
    k = int(round(jan.passada / jan.cap.dt)) if jan.passada else F
    if k >= F:
        mov = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(jan.corpo[..., :3], axis=0), axis=-1).sum(-1))]
        return np.unique(np.searchsorted(mov, np.linspace(0.0, mov[-1], n)).clip(0, F - 1))
    pe = np.zeros(F, bool)
    pe[jan.contatos[jan.contatos[:, 3] == G.IDX["LeftFoot"], 4].astype(int)] = True
    pousos = np.flatnonzero(pe[1:] & ~pe[:-1]) + 1
    pousos = pousos[pousos + k <= F]
    meio = (F - k) // 2
    i0 = int(pousos[np.argmin(np.abs(pousos - meio))]) if len(pousos) else meio
    return np.unique(i0 + np.arange(n) * k // n)


def folha(jans: list[Janela], n: int = 10, vistas=("frente", "lado_dir"), desvio: bool = False) -> Figure:
    """A folha: n poses lado a lado (_poses: uma passada ou a janela inteira), uma linha por vista (o corpo solido; o clipe
    em contorno; o triangulo marca o pe com contato no chao)."""
    linhas = []
    for jan in jans:
        q = _poses(jan, n)
        sils = {v: _silhuetas(jan.corpo[q], jan.geo, *VISTAS[v]) for v in vistas}
        passo = max(float(np.nanmax(s[..., 0].max((1, 2)) - s[..., 0].min((1, 2)))) for s, _ in sils.values()) + 0.3
        ciclo = (f" (uma passada de {jan.passada:.2f} s)" if jan.passada and round(jan.passada / jan.cap.dt) < len(jan.t)
                 else " (espaçadas pelo movimento)")
        for vista in vistas:
            linhas.append((f"{_rotulo(jan)} · poses {jan.t[q[0]]:.2f}–{jan.t[q[-1]]:.2f} s{ciclo}\n"
                           f"{_titulo_vista(jan, vista).replace(chr(10), ' · ')}",
                           [_painel_folha(jan, vista, q, desvio, *sils[vista], passo)]))
    lados = [Line2D([], [], color=ESQ, lw=5, label="esquerda"), Line2D([], [], color=DIR, lw=5, label="direita"),
             Line2D([], [], color=CENTRO, lw=5, label="tronco e cabeça")]
    leg = ([] if desvio else lados) + [
           Line2D([], [], color=REF, lw=0.8, label="clipe"), Line2D([], [], color=CENTRO, marker="^", ls="", label="pé com contato"),
           Line2D([], [], color=CHAO, ls="--", label="vertical na base da pose")]
    barra = (DESVIO, Normalize(0, DESVIO_MAX * 100), "distância de cada elo ao clipe (cm)", None) if desvio else None
    return _montar(linhas, "Folha de poses (pelve travada no plano) · grade 0,5 m", leg, barra)


def _ocupacao(polys: np.ndarray, lims: tuple, forma: tuple[int, int], alfa: float) -> np.ndarray:
    """[h, w] (0-1): as silhuetas sobrepostas com o alfa, rasterizadas pelo Agg (1 - (1 - alfa)^visitas)."""
    h, w = forma
    fig = Figure(figsize=(w / 100, h / 100), dpi=100, facecolor="black")
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_axis_off()
    ax.set_xlim(lims[0], lims[1])
    ax.set_ylim(lims[2], lims[3])
    ax.add_collection(PolyCollection(polys, facecolors=(1.0, 1.0, 1.0, alfa), edgecolors="none"))
    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    return np.asarray(canvas.buffer_rgba())[..., 0] / 255.0


def _painel_espelho(a: Janela, b: Janela, vista: str, saida: dict) -> _Painel:
    u, v = _eixos(a, vista)
    sa, _ = _silhuetas(a.corpo[_quadros(a)], a.geo, u, v)
    sb, _ = _silhuetas(b.corpo[_quadros(b)], b.geo, u, v)
    sa, sb = sa.reshape(-1, *sa.shape[2:]), sb.reshape(-1, *sb.shape[2:])
    alfa = _alfa(max(len(_quadros(a)), len(_quadros(b))))

    def desenhar(ax):
        _cenario(ax, a, vista, u, v, eixo=False)
        lims = (*ax.get_xlim(), *ax.get_ylim())
        forma = (int(round(ax.bbox.height)), int(round(ax.bbox.width)))
        oa, ob = _ocupacao(sa, lims, forma, alfa), _ocupacao(sb, lims, forma, alfa)
        rgba = np.stack([ob, oa, oa, np.maximum(oa, ob)], -1) ** 0.75
        ax.imshow(rgba, extent=lims, origin="upper", interpolation="bilinear", zorder=2, aspect="auto")
        # (A coincidencia: a intersecao sobre a uniao do que cada um visita em 3 quadros ou mais.)
        lim = 1.0 - (1.0 - alfa) ** 3
        ma, mb = oa >= lim, ob >= lim
        iou = float((ma & mb).sum() / max((ma | mb).sum(), 1))
        saida[vista] = iou
        titulo = f"{TITULOS[vista]} · coincidência {iou:.0%}"
        if vista in ("frente", "costas"):
            for jan, cor in ((a, "#00e5ff"), (b, "#ff4040")):
                base, cab = _eixo_medio(jan.juntas)
                ax.plot([base @ u, cab @ u], [0.0, cab @ v], color=cor, lw=1.4, zorder=6)
                ax.plot(cab @ u, cab @ v, "o", color=cor, ms=3.5, zorder=6)
        ax.set_title(titulo, color=TEXTO, fontsize=9, loc="left", pad=4)

    return _Painel(_limites(vista, sa, sb), vista in ELEVACAO, "", desenhar)


def espelho(pares: list[tuple[Janela, Janela]], vistas=("frente", "cima", "lado_dir")) -> tuple[Figure, list[dict]]:
    """O espelho: em cada linha a janela A e a B (do exercicio espelhado) refletida. Devolve a figura e, por linha, a
    coincidencia de cada vista e as inclinacoes."""
    linhas, medidas = [], []
    for a, b in pares:
        bm = espelhar(b)
        la, lb = a.medidas["lado_corpo"], bm.medidas["lado_corpo"]
        saida = {"lado_corpo": (la, lb), "lado_tronco": (a.medidas["lado_tronco"], bm.medidas["lado_tronco"])}
        # (O vies: a parte da inclinacao de lado que nao troca de sinal com o exercicio espelhado.)
        cab = (f"A: {_rotulo(a)}\nB: {_rotulo(bm)}\ninclinação de lado do corpo: A {la:+.1f}° × B {lb:+.1f}° · "
               f"viés da rede (A − B)/2 = {(la - lb) / 2:+.1f}° (+ = esquerda)")
        linhas.append((cab, [_painel_espelho(a, bm, v, saida) for v in vistas]))
        medidas.append(saida)
    leg = [Line2D([], [], color="#00e5ff", lw=5, label="só A"), Line2D([], [], color="#ff4040", lw=5, label="só B refletido"),
           Line2D([], [], color="#ffffff", lw=5, label="coincidem"),
           Line2D([], [], color=CHAO, ls="--", label="vertical na base de A (grade 0,5 m)")]
    titulo = "Espelho: B (exercício espelhado) refletido no plano sagital sobre A\ncom a rede simétrica tudo seria branco"
    return _montar(linhas, titulo, leg), medidas


# --- v2: o movimento (05/11) -------------------------------------------------------------------------------------------
# (A critica do v1 que ele encaminhou, e a resposta do v2: (1) a pelve travada comprimia o movimento -> o "movimento" anda
# no caminho de verdade, um fantasma a cada ~0,65 m; (2) o corpo virava uma massa continua -> fantasmas discretos, uma cor
# cada e o contorno escuro; (3) sem sequencia no tempo -> a cor do azul (cedo) ao magenta (tarde), os instantes escritos e
# a pose em destaque com o esqueleto; (4) os rastros dos pes poluidos -> um rastro liso por parte (cabeca, pelve, joelhos,
# pes; esquerda cheia, direita tracejada), os apoios no chao numerados e as pegadas numa figura so delas; (5) o equilibrio
# invisivel -> o painel do equilibrio: a base, o centro de massa, o ponto de captura, a margem, a inclinacao e as fases.)

TEMPO2 = LinearSegmentedColormap.from_list("tempo2", ["#3d7bff", "#6a62ff", "#9b55f2", "#cf4fe0", "#ff59bd"])
CLARO, OSSO, VERDE, VERMELHO = "#e9ecf1", "#ffd400", "#3ddc84", "#ff4545"
EIXO_REDE = "#ff3b3b"  # (o eixo tornozelos -> cabeca do destaque: branco ele sumia no corpo claro)
_TRACO = (0, (4.0, 2.4))
# (Os rastros por parte, como na referencia dele: (junta, cor, traco, nome); a esquerda cheia, a direita tracejada.)
TRILHAS = (("cabeca", "#3fe0ff", "-", "cabeça"), ("pelve", "#ffd23f", "-", "pelve"),
           ("joelho_e", "#ff9a3c", "-", "joelho esq."), ("joelho_d", "#ff9a3c", _TRACO, "joelho dir."),
           ("pe_e", "#ff5ad1", "-", "pé esq."), ("pe_d", "#ff5ad1", _TRACO, "pé dir."))
PE = {"e": "#4da3ff", "d": "#ff9a1f"}  # (pegadas e equilibrio: o esquerdo azul, o direito laranja, como no Inspector)
LETRA = {"e": "E", "d": "D"}
OSSOS = (("pelve", "abdome"), ("abdome", "peito"), ("peito", "pescoco"), ("pescoco", "cabeca")) + tuple(
    p for s in "ed" for p in (("pelve", f"quadril_{s}"), (f"quadril_{s}", f"joelho_{s}"), (f"joelho_{s}", f"tornozelo_{s}"),
                              (f"tornozelo_{s}", f"calcanhar_{s}"), (f"calcanhar_{s}", f"ponta_{s}"),
                              (f"ponta_{s}", f"tornozelo_{s}"), ("peito", f"ombro_{s}"), (f"ombro_{s}", f"cotovelo_{s}"),
                              (f"cotovelo_{s}", f"punho_{s}"), (f"punho_{s}", f"mao_{s}")))
JUNTAS = ("cabeca", "pescoco", "peito", "abdome", "pelve") + tuple(
    f"{n}_{s}" for s in "ed" for n in ("ombro", "cotovelo", "punho", "quadril", "joelho", "tornozelo", "ponta"))
ESPACO_M = 0.65    # (m: o espaco entre dois fantasmas no caminho, o passo deles sai disso e da velocidade)
FRENTE_N = 8       # (os ultimos fantasmas na vista de frente para o caminho)
ESC_MAX = 230.0    # (px/m: o teto da escala das vistas)
LARGURA2 = 2100.0  # (px: a largura das figuras do v2)
LINHA_M = 16.0     # (m: o comprimento maximo de uma linha da planta nas pegadas e no equilibrio; a mais quebra em linhas)
ARRASTO_M = 0.03   # (m: o pe que arrasta mais que isso no apoio ganha a seta vermelha)
GRAVIDADE = 9.81


def _n(x, casas: int = 1, sinal: bool = False) -> str:
    """O numero em portugues (virgula decimal, o menos tipografico); '–' sem valor."""
    if x is None or not np.isfinite(x):
        return "–"
    return (f"{x:+.{casas}f}" if sinal else f"{x:.{casas}f}").replace("-", "−").replace(".", ",")


def _traco(lw: float) -> list:
    return [patheffects.withStroke(linewidth=lw + 1.8, foreground=FUNDO)]


def _casco(p: np.ndarray) -> np.ndarray:
    """O casco convexo 2D anti-horario dos pontos [n, 2] (a cadeia monotona de Andrew)."""
    p = np.unique(np.round(p[np.isfinite(p).all(1)], 6), axis=0)
    if len(p) < 3:
        return p

    def meia(pts):
        h = []
        for q in pts:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (q[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(q)
        return h

    return np.array(meia(p)[:-1] + meia(p[::-1])[:-1])


@dataclass
class Camera:
    """A projecao de uma vista no referencial da janela: u (a direita da tela) e v (o alto); com `olho`, em perspectiva (a
    escala 1 no plano a `ref` metros do olho, a unidade da tela continua o metro ali); sem, ortografica."""
    u: np.ndarray
    v: np.ndarray
    olho: np.ndarray | None = None
    ref: float = 1.0

    @property
    def o(self) -> np.ndarray:
        """O eixo para a camera (a profundidade: maior = mais perto)."""
        return np.cross(self.u, self.v)

    def tela(self, p: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """[..., 3] -> os pontos na tela [..., 2] e a escala local [...] (1 na ortografica)."""
        p = np.asarray(p, float)
        if self.olho is None:
            return np.stack([p @ self.u, p @ self.v], -1), np.ones(p.shape[:-1])
        r = p - self.olho
        k = self.ref / np.maximum(-(r @ self.o), 0.05)
        return np.stack([(r @ self.u) * k, (r @ self.v) * k], -1), k


LAT, PLANTA = Camera(_X, _Z), Camera(_X, _Y)  # (de lado, a camera a direita do caminho; de cima)


def _perspectiva(alvo, az: float, el: float, dist: float) -> Camera:
    """A camera em perspectiva olhando o `alvo` de `dist` metros, a `az` graus do eixo x (+ = para a esquerda) e `el`
    graus acima."""
    a, e = np.radians(az), np.radians(el)
    o = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    u = np.cross(_Z, o)
    u /= np.linalg.norm(u)
    return Camera(u, np.cross(o, u), np.asarray(alvo, float) + dist * o, dist)


def _contornos(fr: np.ndarray, geo: list, cam: Camera) -> tuple[list, np.ndarray]:
    """As silhuetas de cada quadro na vista: [quadro][elo] o poligono anti-horario [k, 2] (a capsula: o estadio afunilado
    entre as tampas, o raio na escala da perspectiva de cada ponta; o pe: o casco 2D dos pontos da casca dele), e a
    profundidade de cada elo [F, 18] (maior = mais perto). Quadros com nan (sem clipe) saem None."""
    F = len(fr)
    out = [[None] * len(geo) for _ in range(F)]
    t = np.linspace(0.0, 1.0, _ARCO)
    with np.errstate(invalid="ignore"):
        for k, g in enumerate(geo):
            if g["forma"] == "capsule":
                a, b, _ = G.capsula(fr[:, k], g)
                (A, ka), (B, kb) = cam.tela(a), cam.tela(b)
                r1, r2 = g["raio"] * ka[:, None], g.get("raio_mais_x", g["raio"]) * kb[:, None]
                e = B - A
                th = np.arctan2(e[:, 1], e[:, 0])[:, None]
                ph = np.arccos(np.clip((r1 - r2) / np.maximum(np.linalg.norm(e, axis=-1)[:, None], 1e-9), -1.0, 1.0))
                ang = np.concatenate([th + ph * (2 * t - 1), th + ph + (2 * np.pi - 2 * ph) * t], 1)
                cen = np.concatenate([np.repeat(B[:, None], _ARCO, 1), np.repeat(A[:, None], _ARCO, 1)], 1)
                rad = np.concatenate([np.repeat(r2, _ARCO, 1), np.repeat(r1, _ARCO, 1)], 1)[..., None]
                poly = cen + rad * np.stack([np.cos(ang), np.sin(ang)], -1)
                for f in range(F):
                    out[f][k] = poly[f] if np.isfinite(poly[f]).all() else None
            else:
                pts = np.asarray(g["pontos"], float)
                P = fr[:, k, None, :3] + G.qrot(np.broadcast_to(fr[:, k, None, 3:7], (F, len(pts), 4)), pts)
                S = cam.tela(P)[0]
                for f in range(F):
                    out[f][k] = _casco(S[f]) if np.isfinite(S[f]).all() else None
    return out, fr[..., :3] @ cam.o


def _uniao(polys: list) -> MPath | None:
    """Os poligonos anti-horarios num caminho so: preenchido pela regra nao-zero, a uniao leva a cor uma vez (sem escurecer
    onde os elos se cruzam)."""
    ps = [p for p in polys if p is not None and len(p) >= 3]
    if not ps:
        return None
    return MPath(np.concatenate([np.vstack([p, p[:1]]) for p in ps]),
                 np.concatenate([[MPath.MOVETO] + [MPath.LINETO] * (len(p) - 1) + [MPath.CLOSEPOLY] for p in ps]))


def _escuro(cor, f: float = 0.42) -> tuple:
    r, g, b, _ = to_rgba(cor)
    return (r * f, g * f, b * f, 1.0)


def _junta(j: dict, nome: str) -> np.ndarray:
    return j["_quadros"][..., G.IDX["Abdomen"], :3] if nome == "abdome" else j[nome]


def _desenhar_poses(ax, polys: list, prof: np.ndarray, cores, alfa: float, z: float = 2.0, lw: float = 0.7,
                    borda=None) -> None:
    """Cada quadro uma pose: a uniao preenchida numa cor so (`alfa`) e o contorno de cada elo num tom escuro dela (ou
    `borda`), em ordem (a ultima por cima); dentro da pose, os elos do mais longe ao mais perto."""
    n = max(len(polys), 1)
    for i, (ps, cor) in enumerate(zip(polys, cores)):
        elos = [ps[k] for k in np.argsort(prof[i]) if ps[k] is not None]
        path = _uniao(elos)
        if path is None:
            continue
        zi = z + 0.5 * i / n
        ax.add_patch(PathPatch(path, facecolor=to_rgba(cor, alfa), edgecolor="none", zorder=zi))
        ax.add_collection(PolyCollection(elos, facecolors="none", edgecolors=[borda or _escuro(cor)], linewidths=lw,
                                         zorder=zi + 0.001))


def _esqueleto(ax, j: dict, i: int, cam: Camera, cor=OSSO, lw: float = 1.8, s: float = 15.0, z: float = 7.0,
               ls="-", pontos: bool = True, bracos: float = 1.0) -> None:
    """O esqueleto do quadro i: os ossos (linhas; os bracos com `bracos` da largura) e as juntas (pontos), com o contorno
    escuro para ler sobre o corpo."""
    p = {n: cam.tela(_junta(j, n)[i])[0] for n in set(JUNTAS) | {x for o in OSSOS for x in o}}
    ossos = [(a, b, lw * (bracos if any(k in a + b for k in ("ombro", "cotovelo", "punho", "mao")) else 1.0))
             for a, b in OSSOS]
    ossos = [o for o in ossos if o[2] > 0]
    lc = LineCollection([(p[a], p[b]) for a, b, _ in ossos], colors=cor, linewidths=[w for *_, w in ossos],
                        linestyles=ls, zorder=z, capstyle="round")
    lc.set_path_effects(_traco(lw))
    ax.add_collection(lc)
    if pontos:
        pts = np.array([p[n] for n in JUNTAS])
        ax.scatter(pts[:, 0], pts[:, 1], s=s, c=[cor], edgecolors=FUNDO, linewidths=0.7, zorder=z + 0.01)


def _desenhar_trilhas(ax, j: dict, cam: Camera, quadros, nomes=TRILHAS, lw: float = 1.5, z: float = 6.0,
                      alfa: float = 1.0, pontos: bool = True) -> None:
    """Os rastros lisos (todos os quadros) e os pontos nos instantes dos fantasmas (cheio a esquerda, vazio a direita)."""
    for nome, cor, ls, _ in nomes:
        p = cam.tela(j[nome])[0]
        ax.plot(p[:, 0], p[:, 1], color=cor, ls=ls, lw=lw, alpha=alfa, zorder=z, path_effects=_traco(lw),
                solid_capstyle="round")
        if pontos and len(quadros):
            ax.scatter(p[quadros, 0], p[quadros, 1], s=17, facecolors=FUNDO if nome.endswith("_d") else cor,
                       edgecolors=cor, linewidths=1.1, zorder=z + 0.01)


def _rapidez(jan: Janela) -> float:
    """A rapidez media da pelve no chao (m/s): o caminho percorrido, nao o deslocamento (a troca de rumo vai e volta e
    anda)."""
    p = jan.juntas["pelve"][:, :2]
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum() / max(jan.t[-1] - jan.t[0], 1e-9))


def _anda(jan: Janela) -> bool:
    """Anda acima de 0,5 m/s; quem nao anda (parado, giro, levantar) e espalhado no tempo."""
    return _rapidez(jan) > 0.5


def _espalhado(jan: Janela, v: float, dx: np.ndarray | None = None) -> Janela:
    """A janela com o deslocamento de mentira em x: v*(t - t0) ou o `dx` de cada quadro (o movimento quase no lugar
    espalhado no tempo)."""
    if dx is None and not v:
        return jan
    dx = v * (jan.t - jan.t[0]) if dx is None else dx
    v = float(dx[-1] / max(jan.t[-1] - jan.t[0], 1e-9))

    def m(f):
        if f is None:
            return None
        out = f.copy()
        out[..., 0] += dx[:, None]
        return out

    corpo = m(jan.corpo)
    ct = jan.contatos.copy()
    ct[:, 0] += dx[ct[:, 4].astype(int)]
    return replace(jan, corpo=corpo, ref=m(jan.ref), juntas=_juntas(corpo, jan.geo), contatos=ct, espalhar=v)


def _sem_sobrepor(jan: Janela, q: np.ndarray) -> np.ndarray:
    """[F] o deslocamento em x que poe cada fantasma logo depois do anterior na vista lateral (cobrindo 1/4 do mais estreito
    dos dois, e ao menos 0,3 m), os quadros entre eles interpolados: o espalhar automatico de quem quase nao anda, que
    abre o corpo deitado do levantar tanto quanto o de pe."""
    polys, _ = _contornos(jan.corpo[q], jan.geo, LAT)
    lo = np.array([min(p[:, 0].min() for p in ps if p is not None) for ps in polys])
    hi = np.array([max(p[:, 0].max() for p in ps if p is not None) for ps in polys])
    off = np.zeros(len(q))
    for i in range(1, len(q)):
        sobra = 0.25 * min(hi[i - 1] - lo[i - 1], hi[i] - lo[i])
        off[i] = off[i - 1] + max(0.3, (hi[i - 1] + off[i - 1]) - (lo[i] + off[i - 1]) - sobra)
    return np.interp(np.arange(len(jan.t)), q, off)


def _quadros_fantasma(jan: Janela, passo: float | None = None, par: bool = False) -> np.ndarray:
    """Os quadros dos fantasmas: a cada `passo` s (padrao: ESPACO_M pela velocidade, entre 0,1 e 0,25 s), alinhados ao fim
    da janela (o ultimo e a pose em destaque). `par`: so os passos pares da captura (a gravacao do Inspector)."""
    dt = jan.cap.dt
    if passo is None:
        vel = _rapidez(jan)
        passo = 0.25 if vel < 0.3 else float(np.clip(ESPACO_M / vel, 0.1, 0.25))
    k = max(1, int(round(passo / dt)))
    fim = len(jan.t) - 1
    if par:
        k += k % 2
        fim -= int(round(jan.t[fim] / dt)) % 2
    return np.arange(fim, -1, -k)[::-1]


def _fatia(jan: Janela, i0: int, i1: int) -> Janela:
    """Os quadros [i0, i1) da janela, no mesmo referencial (um recorte)."""
    sl = slice(i0, i1)
    ct = jan.contatos[(jan.contatos[:, 4] >= i0) & (jan.contatos[:, 4] < i1)].copy()
    ct[:, 4] -= i0
    corpo = jan.corpo[sl]
    j = _juntas(corpo, jan.geo)
    return replace(jan, t=jan.t[sl], corpo=corpo, ref=None if jan.ref is None else jan.ref[sl], juntas=j, contatos=ct,
                   desvio=jan.desvio[sl], medidas=_medidas(j, corpo))


def _pedido_fixo(jan: Janela) -> bool:
    """Com pedido de deslocamento fixo no trecho e no eixo x dele (a reta do pedido e y = a primeira pelve)."""
    info = jan.cap.info[jan.mundo, np.round(jan.t / jan.cap.dt).astype(int)]
    return bool(not jan.espalhar and jan.pedido is not None and np.nanmean(info[:, 3]) > 0.3 and abs(jan.pedido) < 0.05
                and np.nanmax(np.abs(_wrap(info[:, 4] - info[0, 4]))) < 0.1)


def _no_rumo(jan: Janela) -> Janela:
    """O trecho girado para o x seguir o rumo dele (o pedido se fixo no trecho; senao o eixo principal do trajeto da pelve
    quando passa de 0,5 m): o eixo da janela inteira numa curva deixava o trecho de 2 s na diagonal (os fantasmas
    empilhados na lateral e a planta de 3 m de altura). Nos retos o giro e zero."""
    dt = jan.cap.dt
    info = jan.cap.info[jan.mundo, np.round(jan.t / dt).astype(int)]
    p = jan.juntas["pelve"][:, :2]
    if np.nanmean(info[:, 3]) > 0.3 and np.nanmax(np.abs(_wrap(info[:, 4] - info[0, 4]))) < 0.1 \
            and np.isfinite(jan.rumo_mundo):
        d = float(_wrap(np.arctan2(np.nanmean(np.sin(info[:, 4])), np.nanmean(np.cos(info[:, 4]))) - jan.rumo_mundo))
    elif np.ptp(p - p.mean(0), 0).max() > 0.5:
        e = np.linalg.svd(p - p.mean(0), full_matrices=False)[2][0]
        e = -e if e @ (p[-1] - p[0]) < 0 else e
        d = float(np.arctan2(e[1], e[0]))
    else:
        return jan
    if abs(d) < np.radians(3.0):
        return jan
    c, s_ = np.cos(-d), np.sin(-d)
    R = np.array([[c, -s_], [s_, c]])
    qz = np.array([0.0, 0.0, np.sin(-d / 2), np.cos(-d / 2)])

    def g(f):
        if f is None:
            return None
        out = f.copy()
        out[..., :2] = f[..., :2] @ R.T
        out[..., 3:7] = _qmul(np.broadcast_to(qz, f[..., 3:7].shape), f[..., 3:7])
        return out

    corpo = g(jan.corpo)
    ct = jan.contatos.copy()
    ct[:, :2] = ct[:, :2] @ R.T
    return replace(jan, corpo=corpo, ref=g(jan.ref), juntas=_juntas(corpo, jan.geo), contatos=ct,
                   rumo_mundo=float(jan.rumo_mundo + d), pedido=None if jan.pedido is None else float(_wrap(jan.pedido - d)))


def recortes(jan: Janela, segundos: float) -> list[Janela]:
    """A janela em recortes consecutivos de `segundos` (o ultimo pode ser menor; sobras abaixo de 0,3 s ficam fora)."""
    k = max(2, int(round(segundos / jan.cap.dt)))
    return [_fatia(jan, i, min(i + k, len(jan.t))) for i in range(0, len(jan.t), k)
            if min(i + k, len(jan.t)) - i >= max(2, int(round(0.3 / jan.cap.dt)))]


# --- v2: os apoios, as pegadas e o equilibrio (numeros) ------------------------------------------------------------------

def _fisica(jan: Janela) -> np.ndarray:
    """[F, 2]: o pe esquerdo e o direito com um contato da fisica com impulso naquele quadro."""
    out = np.zeros((len(jan.t), 2), bool)
    ct = jan.contatos[jan.contatos[:, 5] > 1e-4]
    for k, lado in enumerate(("Left", "Right")):
        out[ct[ct[:, 3] == G.IDX[f"{lado}Foot"], 4].astype(int), k] = True
    return out


def _no_chao(fr: np.ndarray, geo: list, fisica: np.ndarray | None = None) -> np.ndarray:
    """[F, 2]: o pe no chao pela regra de marcha.contato (a sola abaixo de marcha.TOQUE_M, sem os toques de um quadro e com
    os quiques de um quadro preenchidos), contando tambem como no chao o quadro com contato da fisica com impulso: no sprint
    o pe esquerdo da rede empurra o chao 4 quadros por passada (0,2-0,9 N.s) com a sola gravada quicando a 1,6-2 cm, e a
    regra so da sola perdia todos os apoios dele."""
    sola = {f"sola_{s}": M.casca(fr, geo, lado)[None, ..., 2].min(-1) for s, lado in M.LADOS}
    if fisica is not None:
        for k, (s, _) in enumerate(M.LADOS):
            sola[f"sola_{s}"][0, fisica[:, k]] = 0.0
    return M.contato(sola)[0][0]


def _apoio(jan: Janela) -> np.ndarray:
    """[F, 2]: o pe esquerdo e o direito no chao (_no_chao, com a fisica da captura)."""
    return _no_chao(jan.corpo, jan.geo, _fisica(jan))


def _apoios(c: np.ndarray) -> list[tuple[str, int, int, int]]:
    """Os apoios (lado, a, b, numero) em ordem de pouso: os quadros [a, b) de cada pe no chao, numerados por lado."""
    out = []
    for k, s in enumerate("ed"):
        d = np.diff(np.r_[0, c[:, k].astype(np.int8), 0])
        out += [(s, int(a), int(b)) for a, b in zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1))]
    out.sort(key=lambda x: x[1])
    conta = {"e": 0, "d": 0}
    res = []
    for s, a, b in out:
        conta[s] += 1
        res.append((s, a, b, conta[s]))
    return res


def _sola(jan: Janela, i: int, s: str) -> np.ndarray:
    """O contorno da sola do pe `s` ("e"/"d") no quadro i, no chao: o casco 2D do anel de baixo da casca do pe."""
    g = jan.geo[G.IDX[("Left" if s == "e" else "Right") + "Foot"]]
    pts = np.asarray(g["pontos"], float)
    pts = pts[pts[:, 2] < pts[:, 2].min() + 0.005]
    f = jan.corpo[i, G.IDX[("Left" if s == "e" else "Right") + "Foot"]]
    return _casco((f[:3] + G.qrot(np.broadcast_to(f[3:7], (len(pts), 4)), pts))[:, :2])


def _arrasto(P: np.ndarray, a: int, b: int) -> tuple[np.ndarray, float]:
    """O arrasto do pe no apoio [a, b) (P: os pontos da casca do pe [F, n, 3]): a soma (vetor 2D, m) e o caminho (m) do pivo,
    o ponto da sola no chao nos dois quadros que menos anda de um ao outro (como o deslize de marcha.medir)."""
    z = P[a:b, :, 2]
    both = (z[:-1] < M.PISO_M) & (z[1:] < M.PISO_M)
    if b - a < 2 or not both.any():
        return np.zeros(2), 0.0
    step = P[a + 1:b, :, :2] - P[a:b - 1, :, :2]
    k = np.where(both, np.linalg.norm(step, axis=-1), np.inf).argmin(-1)
    vec = step[np.arange(len(k)), k][both.any(-1)]
    return vec.sum(0), float(np.linalg.norm(vec, axis=-1).sum())


def _lado_corpo(fr: np.ndarray, geo: list) -> np.ndarray:
    """[F] a inclinacao de lado do corpo (tornozelos -> cabeca) no referencial do abdome (graus, + = esquerda)."""
    with np.errstate(invalid="ignore", divide="ignore"):
        j, f = G.articulacoes(fr, geo), G.referencial(fr)
        return G.inclinacao(j["cabeca"], 0.5 * (j["tornozelo_e"] + j["tornozelo_d"]), f["esquerda"])


def _joelhos(fr: np.ndarray, geo: list) -> dict:
    """A flexao dos joelhos (graus, 0 = esticado) {e, d}: [F]."""
    with np.errstate(invalid="ignore", divide="ignore"):
        j = G.articulacoes(fr, geo)
        return {s: 180.0 - G.angulo(j[f"quadril_{s}"] - j[f"joelho_{s}"], j[f"tornozelo_{s}"] - j[f"joelho_{s}"]) for s in "ed"}


def _numeros(jan: Janela) -> dict:
    """Os numeros dos titulos (nos quadros de pe): a inclinacao de lado do corpo e a do clipe nos mesmos quadros (graus, + =
    esquerda) e a diferenca, a do tronco a frente; com pedido, a velocidade do centro de massa no rumo pedido, o pedido e o
    desvio do rumo (graus, + = esquerda, como o desvio_rumo do treino). A janela sem o espalhar."""
    j = jan.juntas
    m = _de_pe(j)
    lado = _lado_corpo(jan.corpo, jan.geo)
    out = {"lado": float(np.mean(lado[m])), "tronco_frente": jan.medidas["frente_tronco"], "lado_clipe": float("nan"),
           "delta": float("nan"), "vel": float("nan"), "pedido": float("nan"), "desvio": float("nan")}
    if jan.ref is not None:
        lr = _lado_corpo(jan.ref, jan.geo)
        ok = m & np.isfinite(lr)
        if ok.any():
            out.update(lado_clipe=float(lr[ok].mean()), delta=float((lado[ok] - lr[ok]).mean()))
    info = jan.cap.info[jan.mundo, np.round(jan.t / jan.cap.dt).astype(int)]
    if np.nanmean(info[:, 3]) > 0.3 and np.isfinite(jan.rumo_mundo) and len(jan.t) > 2:
        v = np.diff(j["centro_massa"][:, :2], axis=0) / jan.cap.dt
        r = info[:-1, 4] - jan.rumo_mundo
        al, la = v[:, 0] * np.cos(r) + v[:, 1] * np.sin(r), -v[:, 0] * np.sin(r) + v[:, 1] * np.cos(r)
        out.update(vel=float(al.mean()), pedido=float(np.nanmean(info[:, 3])),
                   desvio=float(np.degrees(np.arctan2(la.sum(), al.sum()))))
    return out


def _resumo(jan: Janela, num: dict) -> str:
    """A linha dos numeros-chave: a inclinacao de lado contra o clipe, o tronco, a velocidade e o desvio."""
    txt = f"inclinação de lado do corpo {_n(num['lado'], 1, True)}°"
    if np.isfinite(num["lado_clipe"]):
        txt += f" (clipe {_n(num['lado_clipe'], 1, True)}°: Δ {_n(num['delta'], 1, True)}°)"
    txt += f" · tronco à frente {_n(num['tronco_frente'], 0, True)}°"
    if np.isfinite(num["vel"]):
        txt += (f" · velocidade {_n(num['vel'], 2)} m/s (pedido {_n(num['pedido'], 1)}) · desvio do rumo "
                f"{_n(num['desvio'], 1, True)}°")
    if jan.passada:
        txt += f" · passada {_n(jan.passada, 2)} s"
    return txt


FIM_TXT = {1: "caiu", 2: "desequilíbrio", 3: "virou", 4: "desviou do rumo", 5: "outro", 6: "saiu do percurso"}


def _fim(jan: Janela) -> str:
    """O fim do episodio no fim da janela, quando nao e o do tempo (a queda, o desequilibrio...): ' · ...' ou ''. (05/11: a
    janela da lev_corrida_6 a 30% no mundo 0 acaba em 4,6 s por desequilibrio, e o ultimo 1 s dela, a -28,7 graus, fazia a
    media de -11,5 graus parecer o regime - nos outros 3 mundos ela fica a -1 a +0,3, como o clipe.)"""
    col = jan.cap.meta["colunas_episodios"]
    fim = int(round(jan.t[-1] / jan.cap.dt)) + 1
    for r in jan.cap.episodios:
        if int(r[col.index("mundo")]) == jan.mundo and int(r[col.index("passo_fim")]) == fim:
            causa = int(r[col.index("fim_causa")])
            return f" · o episódio acaba aqui: {FIM_TXT[causa].upper()}" if causa in FIM_TXT else ""
    return ""


def _cabecalho(jan: Janela) -> str:
    m = jan.cap.meta
    modo = "clipe puro (ação zero, mão cheia)" if m["clipe_puro"] else f"rede {m['modo']}"
    return (f"{m['exercicio']} · {modo} · força {m['forca']:.0%} · r{m['atualizacao']} · mundo {jan.mundo} · "
            f"{_n(jan.t[0], 1)}–{_n(jan.t[-1] + jan.cap.dt, 1)} s{_fim(jan)}" + (" · REFLETIDO" if jan.espelhada else ""))


def _seta_corpo(jan: Janela) -> str:
    """Para onde o corpo olha na planta (uma seta), ou "todo lado" quando ele gira."""
    rumo = _rumo(jan.corpo)
    if np.degrees(np.ptp(rumo)) > 120:
        return "todo lado (gira)"
    a = np.degrees(np.arctan2(np.sin(rumo).mean(), np.cos(rumo).mean()))
    return "→↗↑↖←↙↓↘"[int(np.round(a / 45.0)) % 8]


def _como_visto(jan: Janela, cam_az: float) -> str:
    """Como o corpo aparece numa vista cuja camera esta no azimute `cam_az` (graus, no referencial da janela): "de frente",
    "de perfil direito"..., ou "girando" quando o rumo dele varre mais de 120 graus no trecho."""
    rumo = _rumo(jan.corpo)
    if np.degrees(np.ptp(rumo)) > 120:
        return "girando"
    # (O corpo da janela ja esta no referencial dela: o rumo do abdome sai direto nele.)
    a = float(np.degrees(_wrap(np.arctan2(np.sin(rumo).mean(), np.cos(rumo).mean()) - np.radians(cam_az))))
    return ("de frente" if abs(a) <= 35 else "de costas" if abs(a) >= 145 else
            ("de perfil direito" if a > 0 else "de perfil esquerdo") + ("" if 55 <= abs(a) <= 125 else " (3/4)"))


# --- v2: a figura em caixas (pixels) -------------------------------------------------------------------------------------

@dataclass
class _Caixa:
    x: float                    # px da borda esquerda da figura
    y: float                    # px do alto da figura ao alto da caixa
    w: float
    h: float
    desenhar: Callable          # (ax) -> None, com os limites ja postos
    titulo: str = ""
    lims: tuple | None = None   # (x0, x1, y0, y1) em metros (sem marcas); None = eixos com marcas (as faixas)


def _figura(caixas: list, W: float, H: float, titulo: str, subtitulo: str = "", rodape: Callable | None = None) -> Figure:
    """A figura das caixas; o titulo e o subtitulo quebrados na largura, e as linhas a mais descem as caixas."""
    tit = _quebrar(titulo, W - 32, 9.6)
    sub = "\n".join(_quebrar(lin, W - 32, 7.6) for lin in subtitulo.split("\n")) if subtitulo else ""
    extra = 21 * tit.count("\n") + 16 * (sub.count("\n") - subtitulo.count("\n"))
    H += extra
    caixas = [replace(c, y=c.y + extra) for c in caixas]
    fig = Figure(figsize=(W / 100, H / 100), dpi=100, facecolor=FUNDO)
    fig.text(16 / W, 1 - 12 / H, tit, color=TEXTO, fontsize=12.5, weight="bold", va="top", linespacing=1.3)
    if sub:
        fig.text(16 / W, 1 - (36 + 21 * tit.count("\n")) / H, sub, color=TEXTO, fontsize=10, va="top", linespacing=1.4)
    for c in caixas:
        ax = fig.add_axes((c.x / W, 1 - (c.y + c.h) / H, c.w / W, c.h / H))
        ax.set_facecolor(FUNDO)
        for sp in ax.spines.values():
            sp.set_color(GRADE)
        if c.lims is not None:
            ax.set_xlim(c.lims[0], c.lims[1])
            ax.set_ylim(c.lims[2], c.lims[3])
            ax.set_xticks([])
            ax.set_yticks([])
        else:
            ax.tick_params(colors=TEXTO, labelsize=8, length=3)
            ax.grid(color=GRADE, lw=0.6)
        if c.titulo:
            ax.set_title(c.titulo, color=TEXTO, fontsize=9.5, loc="left", pad=4)
        c.desenhar(ax)
    if rodape:
        rodape(fig, W, H)
    return fig


def _lims(*pts, margem=(0.25, 0.25, 0.25, 0.25)) -> tuple:
    p = np.concatenate([np.asarray(a, float).reshape(-1, 2) for a in pts if a is not None and np.size(a)])
    p = p[np.isfinite(p).all(1)]
    lo, hi = p.min(0), p.max(0)
    return (lo[0] - margem[0], hi[0] + margem[1], lo[1] - margem[2], hi[1] + margem[3])


def _pontos_polys(polys: list) -> np.ndarray:
    return np.concatenate([p for ps in polys for p in ps if p is not None] or [np.zeros((0, 2))])


def _grade(ax, lims: tuple, chao: bool) -> None:
    """A grade de 0,5 m (so acima do chao nas vistas de elevacao) e a linha do chao."""
    x0, x1, y0, y1 = lims
    gy = np.arange(np.ceil(y0 / 0.5) * 0.5, y1, 0.5)
    ax.vlines(np.arange(np.ceil(x0 / 0.5) * 0.5, x1, 0.5), 0.0 if chao else y0, y1, colors=GRADE, linewidths=0.6, zorder=0)
    ax.hlines(gy[gy >= 0] if chao else gy, x0, x1, colors=GRADE, linewidths=0.6, zorder=0)
    if chao:
        ax.axhline(0.0, color=CHAO, lw=1.2, zorder=1)


def _chao_perspectiva(ax, cam: Camera, x0: float, x1: float, y0: float, y1: float) -> None:
    """A grade do chao em perspectiva (de 0,5 m no lado, de 1 m no caminho) e a linha do caminho (y = 0)."""
    segs = [[(x0, y, 0.0), (x1, y, 0.0)] for y in np.arange(np.floor(y0 / 0.5) * 0.5, y1 + 1e-6, 0.5)]
    segs += [[(x, y0, 0.0), (x, y1, 0.0)] for x in np.arange(np.floor(x0), x1 + 1e-6, 1.0)]
    ax.add_collection(LineCollection(cam.tela(np.array(segs))[0], colors=GRADE, linewidths=0.7, zorder=0))
    ax.plot(*cam.tela(np.array([[x0, 0.0, 0.0], [x1, 0.0, 0.0]]))[0].T, color=CHAO, lw=0.9, ls="--", zorder=1)


def _rotulos_tempo(jan: Janela, quadros: np.ndarray) -> list[int]:
    """Os fantasmas que levam o instante escrito: os mais perto dos multiplos de 0,5 s (ou de 1 s, se 0,5 s andar menos de
    0,8 m), sempre o primeiro e o ultimo."""
    t = jan.t[quadros]
    passo = 0.5 if _rapidez(jan) * 0.5 >= 0.8 or len(t) < 12 else 1.0
    alvo = np.arange(np.ceil(t[0] / passo) * passo, t[-1] + 1e-9, passo)
    idx = {int(np.argmin(np.abs(t - a))) for a in alvo} | {0, len(t) - 1}
    return sorted(i for i in idx if i == 0 or i == len(t) - 1 or min(abs(t[i] - t[0]), abs(t[-1] - t[i])) > 0.6 * passo)


def _barra_tempo(fig, W: float, H: float, x: float, y: float, t0: float, t1: float, rotulo: str = "tempo (s)",
                 marcas: dict | None = None) -> None:
    cax = fig.add_axes((x / W, y / H, 240 / W, 10 / H))
    cb = fig.colorbar(ScalarMappable(Normalize(t0, t1), TEMPO2), cax=cax, orientation="horizontal")
    marcas = marcas or {v: _n(v, 1) for v in np.linspace(t0, t1, 5)}
    cb.set_ticks(list(marcas), labels=list(marcas.values()))
    cb.ax.tick_params(colors=TEXTO, labelsize=8)
    cb.outline.set_edgecolor(GRADE)
    cax.set_title(rotulo, color=TEXTO, fontsize=8.5, loc="left", pad=3)


# --- v2: o movimento (A), os recortes (C) e a comparacao com o clipe (F) --------------------------------------------------

def _paineis_movimento(jd: Janela, q: np.ndarray, h: int, referencia: bool, apoios: list,
                       vistas=("lateral", "planta", "frontal", "obliqua")) -> dict:
    """O conteudo das vistas do movimento (as silhuetas, os rastros e as solas em cada camera) e os limites delas; a planta
    no mesmo x da lateral. As cameras em perspectiva olham a pose em destaque: a de frente um pouco a esquerda e acima (os
    fantasmas de tras sobem em diagonal), a obliqua da frente-direita."""
    solas = [(s, a, b, n, _sola(jd, (a + b - 1) // 2, s)) for s, a, b, n in apoios]
    y_med = float(np.mean(jd.juntas["pelve"][q, 1]))
    dist = max(4.6, 0.5 * float(np.ptp(jd.juntas["pelve"][q, 0])))
    # (A de frente no eixo do caminho e so de cima: o corpo no eixo continua com a vertical de verdade - de lado, a
    # inclinacao para a frente vazava como inclinacao de lado - e os fantasmas de tras sobem na tela. Ela mira o destaque e
    # mostra os ~1 s que chegam nele; a obliqua mira o ultimo fantasma, para nenhum ficar entre ela e a camera.)
    todas = {"lateral": LAT, "planta": PLANTA,
             "frontal": _perspectiva((jd.juntas["pelve"][q[h], 0], y_med, 0.9), 0.0, 20.0, dist),
             "obliqua": _perspectiva((jd.juntas["pelve"][q[-1], 0], y_med, 0.9), -50.0, 24.0, 1.1 * dist)}
    out = {"solas": solas, "cams": {k: todas[k] for k in vistas}}
    for nome in vistas:
        cam = todas[nome]
        sel = np.arange(max(0, h + 1 - FRENTE_N), h + 1) if nome == "frontal" else np.arange(len(q))
        polys, prof = _contornos(jd.corpo[q], jd.geo, cam)
        ref = _contornos(jd.ref[q], jd.geo, cam)[0] if referencia and jd.ref is not None and nome in ("lateral", "frontal") \
            else None
        trilhas = [cam.tela(jd.juntas[n])[0] for n, *_ in TRILHAS]
        # (Na de frente so os apoios do trecho dela; os de depois do destaque ficavam entre ele e a camera.)
        sol = [cam.tela(np.column_stack([p, np.zeros(len(p))]))[0] if nome != "frontal" or (a < q[sel[-1]] + 1 and
               b > q[sel[0]]) else None for s_, a, b, n, p in solas]
        pts = (_pontos_polys([polys[i] for i in sel]), _pontos_polys([ref[i] for i in sel] if ref else []),
               *(t[q[sel[0]]:q[sel[-1]] + 1] for t in trilhas), *(p for p in sol if p is not None))
        if nome == "lateral":
            lims = _lims(*pts, margem=(0.3, 0.3, 0.0, 0.34))
            lims = (lims[0], lims[1], -0.4, lims[3])
        else:
            lims = _lims(*pts, margem=(0.3, 0.3, 0.3, 0.3) if nome == "planta" else (0.25, 0.25, 0.15, 0.25))
        out[nome] = {"polys": polys, "prof": prof, "ref": ref, "trilhas": trilhas, "solas": sol, "lims": lims, "sel": sel}
    if "lateral" in vistas and "planta" in vistas:
        x0 = min(out["lateral"]["lims"][0], out["planta"]["lims"][0])
        x1 = max(out["lateral"]["lims"][1], out["planta"]["lims"][1])
        for nome in ("lateral", "planta"):
            out[nome]["lims"] = (x0, x1, *out[nome]["lims"][2:])
    return out


def _contorno_uniao(ax, grupos: list, cor, lw: float, z: float, alfa: float = 1.0) -> None:
    """O contorno da uniao de cada grupo de poligonos (o clipe de cada fantasma: uma linha so, sem as costuras dos elos),
    rasterizado na resolucao do painel so na caixa do grupo."""
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    sx, sy = ax.bbox.width / (x1 - x0), ax.bbox.height / (y1 - y0)
    for ps in grupos:
        ps = [p for p in ps if p is not None and len(p) >= 3]
        if not ps:
            continue
        P = np.concatenate(ps)
        lo, hi = P.min(0) - 0.04, P.max(0) + 0.04
        forma = (int(np.ceil((hi[1] - lo[1]) * sy)) + 2, int(np.ceil((hi[0] - lo[0]) * sx)) + 2)
        lims = (lo[0], hi[0], lo[1], hi[1])
        occ = _ocupacao(ps, lims, forma, 1.0)
        ax.contour(occ, levels=[0.5], extent=lims, origin="upper", colors=[cor], linewidths=lw, zorder=z, alpha=alfa)


def _rotulos_apoio(ax, solas: list, pts: list, y: float, passo_y: float) -> None:
    """Os nomes dos apoios (E1, D1...) sob o chao, descendo um nivel quando colam no anterior."""
    ult = -np.inf
    nivel = 0
    for (s, a, b, n, _), p in sorted(zip(solas, pts), key=lambda x: x[1][:, 0].mean()):
        x = float(p[:, 0].mean())
        nivel = (nivel + 1) % 2 if x - ult < 0.28 else 0
        ult = x
        ax.text(x, y - nivel * passo_y, f"{LETRA[s]}{n}", color=VERDE, fontsize=7.5, ha="center", va="top", zorder=5)


def _desenhar_vista(ax, jd: Janela, vista: str, d: dict, q: np.ndarray, h: int, cores, rotulos: bool = True) -> None:
    """Uma vista do movimento: o chao, os apoios, os fantasmas, o clipe (o contorno branco de cada um), a pose em destaque
    com o esqueleto, os rastros com os pontos nos fantasmas e os instantes."""
    v = d[vista]
    cam = d["cams"][vista]
    alfa = {"lateral": 0.36, "planta": 0.13}.get(vista, 0.32)
    sel = list(v["sel"])
    if vista == "lateral":
        _grade(ax, v["lims"], True)
        for (s, *_), p in zip(d["solas"], v["solas"]):
            ax.plot([p[:, 0].min(), p[:, 0].max()], [0.0, 0.0], color=VERDE, lw=4.5, solid_capstyle="butt", zorder=5)
        # (Os nomes so nos apoios de verdade: espalhado parado, cada fantasma leva o pe no chao sem nome.)
        if rotulos and any(n for _, _, _, n, _ in d["solas"]):
            _rotulos_apoio(ax, d["solas"], v["solas"], -0.06, 0.12)
    elif vista == "planta":
        _grade(ax, v["lims"], False)
        if _pedido_fixo(jd):
            ax.axhline(float(jd.juntas["pelve"][0, 1]), color=CHAO, lw=0.9, ls="--", zorder=1)
    else:
        jx = jd.juntas["pelve"][q, 0]
        _chao_perspectiva(ax, cam, jx.min() - 1.5, jx.max() + 1.0, -1.25, 1.25)
        for p in v["solas"]:
            if p is not None:
                ax.add_patch(_poligono(p, VERDE, 0.3, 0.9, z=1.5))
    # (Do mais longe ao mais perto: nas vistas de lado e de cima, a ordem do tempo.)
    ordem = sorted(sel, key=lambda i: v["prof"][i].mean()) if vista in ("frontal", "obliqua") else sel
    ordem = [i for i in ordem if i != h]
    malha = d.get("malha") if vista == "lateral" else None
    if malha is not None:
        # (G: o jogador de verdade - cada foto do Inspector tingida na cor do instante, o centro dela na pelve.)
        pel = cam.tela(jd.juntas["pelve"][q])[0]
        for i in ordem:
            ax.imshow(M_.tingir(malha[i], cores[i], 0.62), extent=M_.extensao(*pel[i], malha[i]), zorder=2 + 0.5 * i / len(q),
                      interpolation="antialiased")
    else:
        _desenhar_poses(ax, [v["polys"][i] for i in ordem], v["prof"][ordem], cores[ordem], alfa,
                        lw=0.6 if vista != "planta" else 0.4, borda=None)
    if vista == "planta":
        for (s, a, b, n, _), p in zip(d["solas"], v["solas"]):
            ax.add_patch(_poligono(p, VERDE, 0.42, 1.0, z=4.6))
            if rotulos and n:
                c = p.mean(0)
                ax.text(c[0], c[1] + (0.16 if s == "e" else -0.16), f"{LETRA[s]}{n}", color=VERDE, fontsize=7.5,
                        ha="center", va="center", zorder=5, path_effects=_traco(0.5))
    if malha is not None:
        ax.imshow(np.dstack([np.clip(malha[h][..., :3] * 1.15, 0, 1), malha[h][..., 3]]),
                  extent=M_.extensao(*pel[h], malha[h]), zorder=4.0, interpolation="antialiased")
        ax.set_xlim(*v["lims"][:2])
        ax.set_ylim(*v["lims"][2:])
    else:
        _desenhar_poses(ax, [v["polys"][h]], v["prof"][h:h + 1], [CLARO], 0.9 if vista == "lateral" else 0.78, z=4.0,
                        borda="#4b5563")
    if v["ref"] is not None:
        # (O clipe: em cada fantasma na lateral; so no destaque na de frente, onde a inclinacao de lado se le.)
        # (Fraco nos fantasmas e forte no destaque: os 16 contornos brancos cheios eram o que mais se via na lateral.)
        if vista == "lateral":
            _contorno_uniao(ax, [v["ref"][i] for i in ordem], REF, 0.8, 5.5, 0.55)
        _contorno_uniao(ax, [v["ref"][h]], REF, 1.0, 5.5)
    trecho = slice(q[sel[0]], q[sel[-1]] + 1)
    jt = {n: jd.juntas[n][trecho] for n, *_ in TRILHAS}
    nomes = [t for t in TRILHAS if not (vista == "planta" and t[0].startswith("joelho"))]
    _desenhar_trilhas(ax, jt, cam, q[sel] - q[sel[0]], nomes=nomes, lw=1.5 if vista != "planta" else 1.2)
    _esqueleto(ax, jd.juntas, q[h], cam)
    if d.get("eixo", (None, None))[0] == vista:
        # (O eixo tornozelos -> cabeca do destaque, cheio, e o do clipe tracejado na mesma base, na vista que ve o corpo de
        # frente ou de costas: a inclinacao de lado se le ali.)
        base = 0.5 * (jd.juntas["tornozelo_e"][q[h]] + jd.juntas["tornozelo_d"][q[h]])
        clipe = d["eixo"][1]
        for fim, cor, ls, lw in ((jd.juntas["cabeca"][q[h]], EIXO_REDE, "-", 2.4),
                                 (None if clipe is None else base + clipe, REF, (0, (3.0, 2.0)), 1.7)):
            if fim is not None:
                p = cam.tela(np.stack([base, fim]))[0]
                ax.plot(p[:, 0], p[:, 1], color=cor, ls=ls, lw=lw, zorder=8.5, path_effects=_traco(lw),
                        solid_capstyle="round")
    if rotulos and vista == "lateral":
        topo = cam.tela(jd.juntas["topo_cabeca"][q])[0]
        for i in _rotulos_tempo(jd, q):
            ax.text(topo[i, 0], topo[i, 1] + 0.1, f"{_n(jd.t[q[i]], 1)} s", color=OSSO if i == h else TEXTO,
                    fontsize=8, ha="center", va="bottom", zorder=8, path_effects=_traco(0.5))


def _poligono(p: np.ndarray, cor, alfa: float, lw: float, ls="-", z: float = 2.0) -> Polygon:
    return Polygon(p, closed=True, facecolor=to_rgba(cor, alfa), edgecolor=cor, lw=lw, ls=ls, zorder=z)


def _lugar_conteudo(jl: Janela, vista: str, q: np.ndarray) -> dict:
    """As poses numeradas no lugar (a janela 'lugar'): as silhuetas, os rastros e os limites da vista."""
    cam = Camera(*VISTAS[vista])
    polys, prof = _contornos(jl.corpo[q], jl.geo, cam)
    trilhas = {n: cam.tela(jl.juntas[n])[0] for n, *_ in TRILHAS}
    lims = _lims(_pontos_polys(polys), *trilhas.values(), margem=(0.16, 0.16, 0.0, 0.16))
    lims = (lims[0], lims[1], -0.1, lims[3]) if vista in ELEVACAO else (lims[0], lims[1], lims[2] - 0.16, lims[3])
    return {"cam": cam, "polys": polys, "prof": prof, "trilhas": trilhas, "lims": lims}


def _desenhar_lugar(ax, jl: Janela, vista: str, q: np.ndarray, c: dict, numeros: bool = True) -> None:
    """As poses no lugar (a pelve travada no plano): cada pose numerada na cor da ordem dela (a silhueta clara, o contorno e
    o esqueleto), os rastros da passada fortes e os do resto da janela fracos."""
    cam, lims = c["cam"], c["lims"]
    _grade(ax, lims, vista in ELEVACAO)
    cores = TEMPO2(np.linspace(0.0, 1.0, len(q)))
    trecho = slice(q[0], q[-1] + 1)
    for nome, cor, ls, _ in TRILHAS:
        if nome.startswith("joelho") or nome == "pelve":
            continue
        # (Os pes claros: no magenta deles o rastro sumia entre os esqueletos, que vao do azul ao magenta.)
        cor = CLARO if nome.startswith("pe_") else cor
        p = c["trilhas"][nome]
        ax.plot(p[:, 0], p[:, 1], color=cor, ls=ls, lw=0.7, alpha=0.2, zorder=1.5)
        ax.plot(p[trecho, 0], p[trecho, 1], color=cor, ls=ls, lw=1.3, zorder=6, path_effects=_traco(1.3))
    # (A cronofotografia: o esqueleto de cada pose na cor da ordem dela - as pernas, o tronco e a cabeca; os bracos ficam
    # para a vista em movimento e para a simetria. Com as silhuetas, mesmo fracas, as poses viravam uma nevoa.)
    cab = cam.tela(jl.juntas["cabeca"][q])[0]
    for i, cor in enumerate(cores):
        _esqueleto(ax, jl.juntas, q[i], cam, cor=cor, lw=2.0, s=7, z=3 + 0.01 * i, pontos=False, bracos=0.0)
        ax.add_patch(Circle(cab[i], 0.085, facecolor="none", edgecolor=cor, lw=1.4, zorder=3 + 0.01 * i))
    if numeros:
        feitos = []
        pe = cam.tela(jl.juntas["pe_e"][q])[0]
        for i, (p, cor) in enumerate(zip(pe, cores)):
            if any(np.hypot(*(p - f)) < 0.09 for f in feitos):
                continue
            feitos.append(p)
            ax.text(p[0], p[1], str(i + 1), color=FUNDO, fontsize=7, weight="bold", ha="center", va="center", zorder=9,
                    bbox=dict(boxstyle="circle,pad=0.18", fc=cor, ec=FUNDO, lw=0.6))


def _quebrar(texto: str, largura_px: float, letra_px: float = 6.9) -> str:
    """O titulo quebrado em linhas que cabem na largura (a letra media de 9,5 pt ~ 6,9 px no DejaVu Sans)."""
    linhas, atual = [], ""
    for p in texto.split(" "):
        if atual and len(atual) + 1 + len(p) > largura_px / letra_px:
            linhas.append(atual)
            atual = p
        else:
            atual = f"{atual} {p}".strip()
    return "\n".join(linhas + [atual])


def _faixas_caminho(jd: Janela, q: np.ndarray, cores) -> list[tuple[str, Callable]]:
    """As faixas sob a lateral, no x do caminho (a pelve de cada quadro): a flexao dos joelhos e a inclinacao de lado do
    corpo contra a do clipe, com os fantasmas marcados (G2: os angulos alinhados com as poses)."""
    x = jd.juntas["pelve"][:, 0]
    jb = _joelhos(jd.corpo, jd.geo)
    jr = _joelhos(jd.ref, jd.geo) if jd.ref is not None else None
    # (A inclinacao de lado so de pe - a pelve acima de 0,6 m: deitada ela da +-180 graus e esmaga a escala.)
    lb = np.where(jd.juntas["pelve"][:, 2] > 0.6, _lado_corpo(jd.corpo, jd.geo), np.nan)
    lr = None
    if jd.ref is not None:
        with np.errstate(invalid="ignore"):
            lr = np.where(jd.ref[:, 0, 2] > 0.6, _lado_corpo(jd.ref, jd.geo), np.nan)

    def marcas(ax):
        for i, qi in enumerate(q):
            ax.axvline(x[qi], color=cores[i], lw=0.8, alpha=0.5, zorder=0)

    def joelhos(ax):
        marcas(ax)
        for s, ls in (("e", "-"), ("d", _TRACO)):
            if jr is not None:
                ax.plot(x, jr[s], color="#9aa4b2", ls=ls, lw=0.9)
            ax.plot(x, jb[s], color="#ff9a3c", ls=ls, lw=1.5)

    def lado(ax):
        marcas(ax)
        ax.axhline(0.0, color=CHAO, lw=0.8)
        if lr is not None:
            ax.plot(x, lr, color="#9aa4b2", lw=1.0, ls=_TRACO)
        ax.plot(x, lb, color=CLARO, lw=1.5)

    tj = "joelhos: flexão (°) · laranja = rede, cinza = clipe · esq. cheia, dir. tracejada" if jr is not None else \
        "joelhos: flexão (°) · esq. cheia, dir. tracejada"
    tl = (f"inclinação de lado do corpo (°, + = esquerda) · clara = rede (média {_n(np.nanmean(lb), 1, True)}), cinza "
          f"tracejada = clipe (média {_n(np.nanmean(lr), 1, True)})" if lr is not None else
          f"inclinação de lado do corpo (°, + = esquerda) · média {_n(np.nanmean(lb), 1, True)}")
    return [(tj, joelhos), (tl, lado)]


def _legenda_movimento(referencia: bool, h_t: float, eixo: bool = False) -> list:
    leg = [Patch(facecolor=CLARO, edgecolor="#4b5563", label=f"pose em destaque ({_n(h_t, 2)} s)"),
           Line2D([], [], color=OSSO, lw=1.8, marker="o", ms=4, mec=FUNDO, label="esqueleto (juntas)")]
    if eixo:
        leg.append(Line2D([], [], color=EIXO_REDE, lw=2.4, label="eixo tornozelos→cabeça do destaque"
                          + (" (o do clipe tracejado)" if referencia else "")))
    leg += [Line2D([], [], color=cor, ls=ls, lw=1.6, marker="o", ms=4.5, mfc=FUNDO if n.endswith("_d") else cor,
                   label=nome) for n, cor, ls, nome in TRILHAS]
    leg += [Line2D([], [], color=VERDE, lw=4.5, label="pé no chão (E1, D1…: os apoios em ordem)")]
    if referencia:
        leg.append(Line2D([], [], color=REF, lw=0.9, label="clipe (o alvo, sob a pelve) em cada fantasma"))
    return leg


def _rodape(leg: list, t0: float, t1: float, rotulo: str = "tempo (s) — a cor de cada fantasma", marcas: dict | None = None):
    def f(fig, W, H):
        _barra_tempo(fig, W, H, 18, 30, t0, t1, rotulo, marcas)
        fig.legend(handles=leg, loc="lower left", bbox_to_anchor=(300 / W, 6 / H),
                   ncol=max(1, min(len(leg), int((W - 310) // 250))), frameon=False, fontsize=8.5, labelcolor=TEXTO,
                   handlelength=2.6, columnspacing=1.4, labelspacing=0.45)
    return f


def _alargar(lims: tuple, minimo: float) -> tuple:
    """Os limites com a largura de pelo menos `minimo`, abertos em volta do meio."""
    x0, x1, y0, y1 = lims
    extra = max(0.0, minimo - (x1 - x0)) / 2
    return (x0 - extra, x1 + extra, y0, y1)


def _detalhe(jan: Janela, detalhe: float | None) -> Janela:
    """O trecho do detalhe: os `detalhe` s de mais movimento do corpo em volta da pelve (e da pelve subindo ou descendo),
    com 15% de vantagem para o fim: no regime ciclico fica o fim (e a queda, se houver), no levantar o levantar. Com um
    salto do rumo pedido acima de 30 graus (a troca), o trecho comeca 0,5 s antes de um deles (o de mais movimento): o
    de mais movimento sozinho pegava a troca forte pela metade (4,8-6,8 s com o salto em 6,0 e a recuperacao ate 7,0).
    Sem detalhe, ou com a janela curta, a janela inteira."""
    dt = jan.cap.dt
    k = int(round((detalhe or 0.0) / dt))
    if not k or len(jan.t) - k <= int(round(0.5 / dt)):
        return jan
    rel = jan.corpo[:, :, :3] - jan.corpo[:, :1, :3]
    mov = np.nan_to_num(np.linalg.norm(np.diff(rel, axis=0), axis=-1).sum(-1) + 5.0 * np.abs(np.diff(jan.corpo[:, 0, 2])))
    soma = np.convolve(mov, np.ones(k - 1), "valid")
    peso = soma * (1.0 + 0.15 * np.linspace(0.0, 1.0, len(soma)))
    saltos = _saltos(jan)
    cand = np.unique(np.clip(saltos + 1 - int(round(0.5 / dt)), 0, len(soma) - 1)) if len(saltos) else np.arange(len(soma))
    i0 = int(cand[np.argmax(peso[cand])])
    return _fatia(jan, i0, i0 + k)


def _saltos(jan: Janela) -> np.ndarray:
    """Os quadros i com o rumo pedido saltando mais de 30 graus de i a i+1 (a troca de rumo), com pedido de deslocamento."""
    info = jan.cap.info[jan.mundo, np.round(jan.t / jan.cap.dt).astype(int)]
    if not np.nanmean(info[:, 3]) > 0.3:
        return np.zeros(0, int)
    return np.flatnonzero(np.nan_to_num(np.abs(_wrap(np.diff(info[:, 4])))) > np.radians(30.0))


def _dobra(jan: Janela, q: np.ndarray) -> bool:
    """Os fantasmas se empilham na lateral do caminho de verdade: mais de 1/4 dos vizinhos avanca menos de 0,25 m no x (a
    troca de rumo vai e volta; a curva fechada anda de lado)."""
    x = jan.juntas["pelve"][q, 0]
    return len(q) > 2 and float(np.mean(np.diff(x) < 0.25)) > 0.25


def _apoios_em(apoios: list, i0: int, i1: int) -> list:
    """Os apoios da janela inteira (a numeracao dela) que caem em [i0, i1), nos quadros do trecho."""
    return [(s, max(a, i0) - i0, min(b, i1) - i0, n) for s, a, b, n in apoios if a < i1 and b > i0]


def _lugar_de(jan: Janela, trecho: Janela) -> Janela:
    """A janela no lugar (a pelve travada, o rumo suavizado) dos mesmos quadros do trecho."""
    return janela(jan.cap, jan.mundo, (float(trecho.t[0]), float(trecho.t[-1] + 0.5 * jan.cap.dt)), "lugar")


def movimento(jan: Janela, passo: float | None = None, referencia: bool = False, destaque: float | None = None,
              espalhar: float | None = None, malha=None, detalhe: float | None = 2.0) -> Figure:
    """O movimento (A): a vista lateral grande no alto (os fantasmas ao longo do caminho de verdade, a cada `passo` s), a
    planta alinhada com ela, as faixas dos joelhos e da inclinacao de lado no mesmo x, e embaixo a vista de frente para o
    caminho e a obliqua 3/4 (em perspectiva: os fantasmas se afastam) e as poses no lugar (a ultima passada, numeradas).
    O desenho e o `detalhe` (_detalhe: os s de mais movimento; None = a janela inteira), girado para o rumo dele
    (_no_rumo); os numeros do titulo sao da janela inteira. `espalhar` (m/s; None = sem sobrepor quando a pelve anda
    menos de 0,5 m/s, _anda) abre no tempo o que quase nao anda; `referencia` poe o clipe em
    contorno sobre cada fantasma; `destaque` (s) escolhe a pose clara (padrao: a ultima); `malha` (inspecao.malha) troca a
    lateral pelo jogador de verdade."""
    if jan.modo != "caminho":
        raise ValueError("o movimento pede a janela no modo 'caminho'")
    num = _numeros(jan)
    jv = _no_rumo(_detalhe(jan, detalhe))
    i0 = int(round((jv.t[0] - jan.t[0]) / jan.cap.dt))
    anda = _anda(jv)
    q = _quadros_fantasma(jv, passo, par=bool(malha))
    if espalhar is None and (not anda or _dobra(jv, q)):
        jd = _espalhado(jv, 0.0, _sem_sobrepor(jv, q))
    else:
        jd = _espalhado(jv, espalhar or 0.0)
        q = _quadros_fantasma(jd, passo, par=bool(malha))
    # (Quem anda e foi espalhado - a troca de rumo vai e volta sobre o proprio caminho e os fantasmas se empilhavam na
    # lateral - so tem a lateral e as faixas abertas no tempo: a planta, a de frente e a obliqua ficam no caminho de verdade.)
    jr = jv if anda and jd.espalhar else jd
    h = len(q) - 1 if destaque is None else int(np.argmin(np.abs(jd.t[q] - destaque)))
    cores = TEMPO2((jd.t[q] - jd.t[0]) / max(jd.t[-1] - jd.t[0], 1e-9))
    if jd.espalhar and not anda:
        # (Espalhado, o apoio de um pe dura o trecho todo: o pe no chao vai em cada fantasma, sem nome.)
        c = _apoio(jd)
        apoios = [(s_, int(qi), int(qi) + 1, 0) for qi in q for k, s_ in enumerate("ed") if c[qi, k]]
    else:
        apoios = _apoios_em(_apoios(_apoio(jan)), i0, i0 + len(jv.t))
    if jr is jd:
        d = _paineis_movimento(jd, q, h, referencia, apoios)
    else:
        d = _paineis_movimento(jr, q, h, referencia, apoios, vistas=("planta", "frontal", "obliqua"))
        dl = _paineis_movimento(jd, q, h, referencia, apoios, vistas=("lateral",))
        d["lateral"], d["cams"]["lateral"] = dl["lateral"], dl["cams"]["lateral"]
    if malha:
        if jan.mundo != 0:
            raise SystemExit("--malha: a gravacao do Inspector (o .dump) e so do mundo 0")
        # (Os quadros do dump sao os passos pares da captura, no instante (passo + 1) * dt.)
        tempos = (np.round(jd.t[q] / jan.cap.dt) + 1) * jan.cap.dt
        with tempfile.TemporaryDirectory() as tmp:
            d["malha"] = M_.renderizar(jan.cap.caminho.with_suffix(".dump"), tempos, np.degrees(jd.rumo_mundo) - 90.0,
                                       saida=Path(tmp) / "malha.png")
    # (O eixo tornozelos -> cabeca do destaque e o do clipe na vista que ve o corpo de frente ou de costas: a de frente na
    # corrida, a lateral na corrida de lado; girando, em nenhuma.)
    ve = ("frontal" if _como_visto(jr, 0.0) in ("de frente", "de costas") else
          "lateral" if _como_visto(jd, -90.0) in ("de frente", "de costas") else None)
    i_h, clipe, eixo_txt = q[h], None, ""
    if referencia and jd.ref is not None and np.isfinite(jd.ref[i_h]).all():
        jrf = G.articulacoes(jd.ref[i_h:i_h + 1], jd.geo)
        clipe = jrf["cabeca"][0] - 0.5 * (jrf["tornozelo_e"][0] + jrf["tornozelo_d"][0])
    d["eixo"] = (ve, clipe)
    if ve:
        eixo_txt = (f" · eixo tornozelos→cabeça do destaque: rede {_n(_lado_corpo(jd.corpo[i_h:i_h + 1], jd.geo)[0], 1, True)}°"
                    + (f", clipe {_n(_lado_corpo(jd.ref[i_h:i_h + 1], jd.geo)[0], 1, True)}° (tracejado)"
                       if clipe is not None else ""))
    jl = _lugar_de(jan, jv)
    ql = _poses(jl, 6)
    # (Na corrida de lado as pernas abrem de lado no corpo: no lugar, de frente; senao do lado direito.)
    vl = "frente" if jl.pedido is not None and abs(np.sin(jl.pedido)) > abs(np.cos(jl.pedido)) else "lado_dir"
    cl = _lugar_conteudo(jl, vl, ql)
    mx, mxd, gap = 54.0, 16.0, 22.0
    lat, pla = d["lateral"]["lims"], d["planta"]["lims"]
    S = min(ESC_MAX, (LARGURA2 - mx - mxd) / max(lat[1] - lat[0], pla[1] - pla[0]))
    w_lat, w_pla = (lat[1] - lat[0]) * S, (pla[1] - pla[0]) * S
    W = max(w_lat + mx + mxd, w_pla + mx + mxd, 1500.0)
    # (As tres de baixo na mesma altura em pixels, cada uma na escala dela - as duas em perspectiva nao tem escala unica
    # mesmo -, a de frente e a do lugar com pelo menos 1,3 m de largura: estreitas, o titulo quebrava em cinco linhas.)
    fre, obl, lug = _alargar(d["frontal"]["lims"], 1.3), d["obliqua"]["lims"], _alargar(cl["lims"], 1.3)
    alto = 560.0
    proporcao = [(b[1] - b[0]) / (b[3] - b[2]) for b in (fre, obl, lug)]
    alto = min(alto, (W - mx - mxd - 2 * gap) / sum(proporcao))
    d["frontal"]["lims"], d["obliqua"]["lims"], cl["lims"] = fre, obl, lug
    passo_s = (q[1] - q[0]) * jan.cap.dt if len(q) > 1 else 0.0
    esp = (f" · espalhado no tempo (~{_n(jd.espalhar, 1)} m/s; o x não é o caminho)" if jd.espalhar else "")
    caixas = []
    y = 78.0
    titulo_lat = (("O jogador de verdade (a malha do Inspector em argila, tingida pelo tempo) · " if malha else "")
                  + f"Vista lateral do caminho (a câmera à direita; o corpo {_como_visto(jd, -90.0)}) · um fantasma a "
                  f"cada {_n(passo_s, 2)} s · grade 0,5 m{esp}" + (eixo_txt if ve == "lateral" else ""))
    h_lat = (lat[3] - lat[2]) * S
    caixas.append(_Caixa(mx, y + 20, w_lat, h_lat, lambda ax: _desenhar_vista(ax, jd, "lateral", d, q, h, cores),
                         titulo_lat, lat))
    y += 20 + h_lat + 10
    h_pla = (pla[3] - pla[2]) * S
    titulo_pla = (f"Planta (de cima; o caminho →, a esquerda ↑; o corpo olha para {_seta_corpo(jr)}): as pegadas no chão e "
                  f"os rastros" + (" · tracejada = o rumo pedido" if _pedido_fixo(jr) else "")
                  + (" · o caminho de verdade (só a lateral e as faixas estão espalhadas no tempo)" if jr is not jd else ""))
    caixas.append(_Caixa(mx, y + 20, w_pla, h_pla, lambda ax: _desenhar_vista(ax, jr, "planta", d, q, h, cores),
                         titulo_pla, pla))
    y += 20 + h_pla + 8
    for titulo, desenhar in _faixas_caminho(jd, q, cores):
        def faixa(ax, desenhar=desenhar):
            desenhar(ax)
            ax.set_xlim(lat[0], lat[1])
            ax.tick_params(labelbottom=False)
        caixas.append(_Caixa(mx, y + 18, w_lat, 72, faixa, titulo))
        y += 18 + 72 + 6
    y += 8
    larg = [alto * p_ for p_ in proporcao]
    titulos = [f"De frente para o caminho (perspectiva; o corpo {_como_visto(jr, 0.0)}; os fantasmas se afastam)"
               + (eixo_txt if ve == "frontal" else ""),
               "Oblíqua 3/4 (perspectiva, da frente-direita e de cima)",
               f"No lugar (a pelve travada; {'de frente' if vl == 'frente' else 'lado direito'}): {len(ql)} poses "
               "numeradas de " + (f"uma passada ({_n(jl.passada, 2)} s)" if jl.passada else "todo o trecho")
               + "; claros, os rastros dos pés"]
    titulos = [_quebrar(t, w) for t, w in zip(titulos, larg)]
    topo = 16 + 15 * max(t.count("\n") for t in titulos)
    x = mx
    for (nome, lims), w, t in zip((("frontal", fre), ("obliqua", obl), ("lugar", lug)), larg, titulos):
        desenhar = (lambda ax: _desenhar_lugar(ax, jl, vl, ql, cl)) if nome == "lugar" else \
            (lambda ax, nome=nome: _desenhar_vista(ax, jr, nome, d, q, h, cores))
        caixas.append(_Caixa(x, y + topo + 6, w, alto, desenhar, t, lims))
        x += w + gap
    y += topo + 6 + alto + 12
    H = y + 96
    janela_txt = (f" · detalhe {_n(jv.t[0], 1)}–{_n(jv.t[-1] + jan.cap.dt, 1)} s ("
                  + ("a troca de rumo" if len(_saltos(jv)) else "o trecho de mais movimento")
                  + "; os números: a janela inteira)" if len(jv.t) < len(jan.t) else "")
    titulo = f"Movimento — {_cabecalho(jan)}{janela_txt}"
    leg = _legenda_movimento(referencia and jd.ref is not None, jd.t[q[h]], ve is not None)
    return _figura(caixas, W, H, titulo, _resumo(jan, num), _rodape(leg, jd.t[0], jd.t[-1] + jan.cap.dt))


# --- v2: os recortes (C) ---------------------------------------------------------------------------------------------------

def movimento_recortes(jan: Janela, segundos: float, passo: float | None = None, referencia: bool = False,
                       espalhar: float | None = None) -> Figure:
    """Os recortes (C): a janela em trechos consecutivos de `segundos`, um cartao por trecho com so os fantasmas, as pegadas
    e os rastros dele: a lateral em movimento sobre a planta e, ao lado, as poses no lugar de frente e de lado (a pelve
    travada). A cor do tempo recomeca em cada trecho; a numeracao dos apoios e a da janela inteira; os cartoes em uma, duas
    ou tres colunas, a que der a maior escala sem passar de ~2300 px de altura."""
    if jan.modo != "caminho":
        raise ValueError("os recortes pedem a janela no modo 'caminho'")
    dt = jan.cap.dt
    sem_sobrepor = espalhar is None and not _anda(jan)
    todos = _apoios(_apoio(jan))
    k = max(2, int(round(segundos / dt)))
    cartoes = []
    for i0 in range(0, len(jan.t), k):
        i1 = min(i0 + k, len(jan.t))
        if i1 - i0 < max(3, int(round(0.3 / dt))):
            continue
        fat = _no_rumo(_fatia(jan, i0, i1))
        if sem_sobrepor:
            q = _quadros_fantasma(fat, passo)
            jd = _espalhado(fat, 0.0, _sem_sobrepor(fat, q))
        else:
            jd = _espalhado(fat, espalhar or 0.0)
            q = _quadros_fantasma(jd, passo)
        cores = TEMPO2((jd.t[q] - jd.t[0]) / max(jd.t[-1] - jd.t[0], 1e-9))
        if jd.espalhar:
            c = _apoio(jd)
            ap = [(s_, int(qi), int(qi) + 1, 0) for qi in q for k_, s_ in enumerate("ed") if c[qi, k_]]
        else:
            ap = _apoios_em(todos, i0, i1)
        d = _paineis_movimento(jd, q, len(q) - 1, referencia, ap, vistas=("lateral", "planta"))
        jl = _lugar_de(jan, fat)
        ql = _poses(jl, 5)
        cf, cs = _lugar_conteudo(jl, "frente", ql), _lugar_conteudo(jl, "lado_dir", ql)
        cartoes.append({"fat": fat, "jd": jd, "q": q, "cores": cores, "d": d, "jl": jl, "ql": ql, "cf": cf, "cs": cs,
                        "ap": ap, "num": _numeros(fat)})
    if not cartoes:
        raise SystemExit(f"{jan.cap.nome}: janela curta demais para recortes de {segundos} s")
    # (A medida de cada cartao em metros: a lateral sobre a planta a esquerda, as duas no lugar lado a lado a direita.)
    for c in cartoes:
        lat, pla = c["d"]["lateral"]["lims"], c["d"]["planta"]["lims"]
        c["wl"], c["hl"], c["hp"] = lat[1] - lat[0], lat[3] - lat[2], pla[3] - pla[2]
        c["wd"] = (c["cf"]["lims"][1] - c["cf"]["lims"][0]) + (c["cs"]["lims"][1] - c["cs"]["lims"][0])
        c["hd"] = max(c["cf"]["lims"][3] - c["cf"]["lims"][2], c["cs"]["lims"][3] - c["cs"]["lims"][2])
    mx, gap, gapc, cab, tit = 16.0, 14.0, 18.0, 40.0, 20.0
    largura_m = max(c["wl"] + c["wd"] for c in cartoes)
    melhor = None
    for ncol in (1, 2, 3):
        linhas = [cartoes[i:i + ncol] for i in range(0, len(cartoes), ncol)]
        s_w = ((LARGURA2 - 2 * mx - (ncol - 1) * gapc) / ncol - 2 * gap - 8) / largura_m
        alt_m = sum(max(max(c["hl"] + c["hp"], c["hd"]) for c in ln) for ln in linhas)
        s_h = (2300.0 - 90 - 100 - len(linhas) * (cab + 2 * tit + 30)) / alt_m
        esc = min(ESC_MAX, s_w, s_h)
        if melhor is None or esc > melhor[0] + 1e-6:
            melhor = (esc, ncol, linhas)
    S, ncol, linhas = melhor
    larg_cartao = largura_m * S + 2 * gap + 8
    W = max(2 * mx + ncol * larg_cartao + (ncol - 1) * gapc, 1300.0)
    caixas, fundos = [], []
    y = 84.0
    for ln in linhas:
        alt = max(max(c["hl"] + c["hp"], c["hd"]) for c in ln) * S + cab + 2 * tit + 22
        for j, c in enumerate(ln):
            x = mx + j * (larg_cartao + gapc)
            fundos.append((x, y, larg_cartao, alt))
            fat, jd, q, d, cores, num = c["fat"], c["jd"], c["q"], c["d"], c["cores"], c["num"]
            lat, pla = d["lateral"]["lims"], d["planta"]["lims"]
            texto = (f"{_n(fat.t[0], 1)}–{_n(fat.t[-1] + dt, 1)} s · lado {_n(num['lado'], 1, True)}°"
                     + (f" (clipe {_n(num['lado_clipe'], 1, True)}°, Δ {_n(num['delta'], 1, True)}°)"
                        if np.isfinite(num["lado_clipe"]) else "")
                     + (f" · {_n(num['vel'], 1)} m/s" if np.isfinite(num["vel"]) else "")
                     + ("" if c["jd"].espalhar else " · apoios " + (" ".join(f"{LETRA[s_]}{n}" for s_, _, _, n in c["ap"])
                                                                     or "nenhum")))
            caixas.append(_Caixa(x + 8, y + cab + tit - 4, c["wl"] * S, c["hl"] * S,
                                 lambda ax, jd=jd, d=d, q=q, cores=cores: _desenhar_vista(ax, jd, "lateral", d, q, len(q) - 1,
                                                                                         cores),
                                 _quebrar(texto, c["wl"] * S + c["wd"] * S, 6.4), lat))
            caixas.append(_Caixa(x + 8, y + cab + 2 * tit + c["hl"] * S + 6, c["wl"] * S, c["hp"] * S,
                                 lambda ax, jd=jd, d=d, q=q, cores=cores: _desenhar_vista(ax, jd, "planta", d, q, len(q) - 1,
                                                                                         cores), "", pla))
            xd = x + 8 + c["wl"] * S + gap
            for vista, cc in (("frente", c["cf"]), ("lado_dir", c["cs"])):
                lims = cc["lims"]
                lims = (lims[0], lims[1], lims[2], lims[2] + c["hd"])
                cc["lims"] = lims
                w = (lims[1] - lims[0]) * S
                caixas.append(_Caixa(xd, y + cab + tit - 4, w, c["hd"] * S,
                                     lambda ax, c=c, vista=vista, cc=cc: _desenhar_lugar(ax, c["jl"], vista, c["ql"], cc),
                                     "", lims))
                xd += w + 6
        y += alt + 12
    H = y + 92

    def rodape(fig, W_, H_):
        for x, yy, w, hh in fundos:
            fig.add_artist(Rectangle((x / W_, 1 - (yy + hh) / H_), w / W_, hh / H_, transform=fig.transFigure,
                                     facecolor="#11161e", edgecolor=GRADE, lw=0.8, zorder=-5))
            fig.text((x + 8) / W_, 1 - (yy + 8) / H_, f"recorte {fundos.index((x, yy, w, hh)) + 1}", color=OSSO,
                     fontsize=10, weight="bold", va="top")
        leg = _legenda_movimento(referencia and jan.ref is not None, 0.0)
        leg[0] = Patch(facecolor=CLARO, edgecolor="#4b5563", label="pose em destaque (o fim do recorte)")
        _rodape(leg, 0.0, 1.0, "tempo dentro do recorte", {0.0: "início", 1.0: "fim"})(fig, W_, H_)

    titulo = f"Movimento em recortes de {_n(segundos, 1)} s — {_cabecalho(jan)}"
    sub = (_resumo(jan, _numeros(jan)) + " (a janela inteira) · em cada recorte: a lateral em movimento sobre a planta; "
           "à direita, no lugar, de frente e de lado (poses numeradas)")
    return _figura(caixas, W, H, titulo, sub, rodape)


# --- v2: no lugar (B) ------------------------------------------------------------------------------------------------------

def _desenhar_simetria(ax, jl: Janela, q: np.ndarray, lims: tuple) -> None:
    """De frente, a perna e o braco direitos espelhados (y -> -y, tracejados) sobre os esquerdos (cheios), na passada: com a
    marcha simetrica os lacos coincidem (o que sobra e assimetria)."""
    _grade(ax, lims, True)
    cam = Camera(*VISTAS["frente"])
    # (A passada inteira: um laco aberto nao se compara.)
    k = int(round(jl.passada / jl.cap.dt)) if jl.passada else len(jl.t)
    trecho = slice(q[0], min(q[0] + k + 1, len(jl.t)))
    for nome, rot in (("pe", "pé"), ("joelho", "joelho"), ("mao", "mão")):
        e = cam.tela(jl.juntas[f"{nome}_e"][trecho])[0]
        d = cam.tela(jl.juntas[f"{nome}_d"][trecho] * [1.0, -1.0, 1.0])[0]
        ax.plot(e[:, 0], e[:, 1], color=PE["e"], lw=1.6, zorder=5, path_effects=_traco(1.6))
        ax.plot(d[:, 0], d[:, 1], color=PE["d"], lw=1.6, ls=_TRACO, zorder=5, path_effects=_traco(1.6))
        xs, ys = np.r_[e[:, 0], d[:, 0]], np.r_[e[:, 1], d[:, 1]]
        # (O pe a direita do laco dele, embaixo: a esquerda, no alto, o nome colava no do joelho.)
        if nome == "pe":
            ax.text(np.nanmax(xs) + 0.04, np.nanmin(ys), rot, color=CHAO, fontsize=7.5, ha="left", va="bottom", zorder=6)
        else:
            ax.text(np.nanmin(xs) - 0.04, np.nanmax(ys), rot, color=CHAO, fontsize=7.5, ha="right", va="top", zorder=6)
    cab = cam.tela(jl.juntas["cabeca"][trecho])[0]
    ax.plot(cab[:, 0], cab[:, 1], color="#3fe0ff", lw=1.2, zorder=4)
    ax.axvline(0.0, color=CHAO, ls="--", lw=0.8, zorder=1)


def parado(jans: list[Janela], n: int = 6) -> Figure:
    """No lugar (B): a pelve travada no plano e o rumo suavizado (o v1), mas com poses discretas: `n` poses numeradas de uma
    passada (sem ciclo, espalhadas pelo movimento), cada uma na cor da ordem dela (o esqueleto sobre a silhueta fraca), o
    rastro dos pes e da cabeca forte na passada e fraco no resto; e a simetria (a direita espelhada sobre a esquerda). Uma
    linha por janela, as vistas frente, lado direito, cima e a simetria, numa escala so."""
    linhas = []
    for jl in jans:
        q = _poses(jl, n)
        paineis = []
        for vista in ("frente", "lado_dir", "cima"):
            c = _lugar_conteudo(jl, vista, q)
            titulo = {"frente": "frente", "lado_dir": "lado direito", "cima": "cima (frente →)"}[vista]
            paineis.append(_Painel(c["lims"], vista in ELEVACAO, titulo,
                                   lambda ax, jl=jl, vista=vista, q=q, c=c: _desenhar_lugar(ax, jl, vista, q, c)))
        lf = paineis[0].lims
        paineis.append(_Painel(lf, True, "simetria: a direita espelhada (tracejada) sobre a esquerda",
                               lambda ax, jl=jl, q=q, lf=lf: _desenhar_simetria(ax, jl, q, (*ax.get_xlim(), *ax.get_ylim()))))
        m = jl.medidas
        trecho = (f"uma passada de {_n(jl.passada, 2)} s" if jl.passada and round(jl.passada / jl.cap.dt) < len(jl.t)
                  else "espalhadas pelo movimento")
        cab = (f"{_cabecalho(jl)} · {len(q)} poses {_n(jl.t[q[0]], 2)}–{_n(jl.t[q[-1]], 2)} s ({trecho})\n"
               f"inclinação de lado: corpo {_n(m['lado_corpo'], 1, True)}°, tronco {_n(m['lado_tronco'], 1, True)}° · "
               f"tronco à frente {_n(m['frente_tronco'], 1, True)}° (+ = esquerda; médias da janela de pé)")
        linhas.append((cab, paineis))
    leg = [Line2D([], [], color=c_ if n_ == "cabeca" else CLARO, ls=ls, lw=1.6, label=nome) for n_, c_, ls, nome in TRILHAS
           if n_ in ("cabeca", "pe_e", "pe_d")]
    leg += [Line2D([], [], color=PE["e"], lw=1.6, label="esquerda (simetria)"),
            Line2D([], [], color=PE["d"], lw=1.6, ls=_TRACO, label="direita espelhada (simetria)"),
            Line2D([], [], color=CHAO, lw=0.9, alpha=0.6, label="rastro fraco: o resto da janela")]
    barra = (TEMPO2, Normalize(0, 1), "a ordem das poses", {0: "pose 1", 1: f"pose {n}"})
    return _montar(linhas, "No lugar: a pelve travada no plano, poses discretas numeradas\n"
                           "vistas pelo abdome, rumo suavizado na passada · + = esquerda · grade 0,5 m", leg, barra)


# --- v2: as pegadas (D) -----------------------------------------------------------------------------------------------------

def _linhas_planta(jan: Janela, recorte: float | None) -> list[tuple[int, int]]:
    """Os trechos [a, b) das linhas da planta: os recortes de `recorte` s, ou partes iguais no tempo com no maximo LINHA_M
    de caminho cada."""
    F = len(jan.t)
    if recorte:
        k = max(2, int(round(recorte / jan.cap.dt)))
        return [(i, min(i + k, F)) for i in range(0, F, k) if min(i + k, F) - i >= 2]
    n = max(1, int(np.ceil(np.ptp(jan.juntas["pelve"][:, 0]) / LINHA_M)))
    cortes = np.linspace(0, F, n + 1).round().astype(int)
    return list(zip(cortes[:-1], cortes[1:]))


def _pegadas_dados(jan: Janela) -> list[dict]:
    """Cada apoio: o lado, os quadros, o numero, o contorno da sola no meio do apoio e em cada quadro, o arrasto (o vetor e
    o caminho do pivo), o centro do pe no pouso e o passo (comprimento no caminho e largura) desde o pouso anterior do outro
    pe (None se o anterior foi do mesmo pe)."""
    c = _apoio(jan)
    P = {s: M.casca(jan.corpo, jan.geo, lado) for s, lado in M.LADOS}
    out = []
    for s, a, b, n in _apoios(c):
        vec, cam = _arrasto(P[s], a, b)
        out.append({"lado": s, "a": a, "b": b, "n": n, "sola": _sola(jan, (a + b - 1) // 2, s),
                    "solas": [_sola(jan, i, s) for i in range(a, b, 2)], "arrasto": vec, "arrasto_m": cam,
                    "pouso": jan.juntas[f"pe_{s}"][a, :2], "passo": None, "quiques": 0})
    # (O pe que volta ao chao em ate 4 quadros (0,13 s) e o mesmo apoio quicando: um nome so, "D6–7", e o arrasto somado
    # no primeiro.)
    for s in "ed":
        lado = [g for g in out if g["lado"] == s]
        for g in lado:
            g["rotulo"] = f"{LETRA[s]}{g['n']}"
        for ant, atual in zip(lado[:-1], lado[1:]):
            if atual["a"] - ant["b"] <= 4:
                dono = ant.get("junto", ant)
                atual["junto"] = dono
                dono["rotulo"] = f"{LETRA[s]}{dono['n']}–{atual['n']}"
                dono["arrasto"], dono["arrasto_m"] = dono["arrasto"] + atual["arrasto"], dono["arrasto_m"] + atual["arrasto_m"]
                dono["quiques"] += 1
                atual["arrasto"], atual["arrasto_m"] = np.zeros(2), 0.0
    # (O passo entre os pousos de verdade: medido do quique, o tempo encurtava - a rede atual da corrida_0, com o pe direito
    # quicando em quase todo apoio, dava a cadencia de 265 passos/min contra os ~180 da passada de 0,70 s.)
    donos = [g for g in out if "junto" not in g]
    for ant, atual in zip(donos[:-1], donos[1:]):
        if ant["lado"] != atual["lado"] and atual["a"] > 0:
            d = atual["pouso"] - ant["pouso"]
            atual["passo"] = (float(d[0]), abs(float(d[1])), (atual["a"] - ant["a"]) * jan.cap.dt)
    return out


def _linha_pedido(jan: Janela) -> np.ndarray | None:
    """O caminho do rumo pedido no passo do corpo (o rumo pedido de cada quadro andado na velocidade do centro de massa,
    desde a primeira pelve: com o pedido fixo, a reta y = 0); None sem pedido."""
    info = jan.cap.info[jan.mundo, np.round(jan.t / jan.cap.dt).astype(int)]
    if np.nanmean(info[:, 3]) < 0.3 or not np.isfinite(jan.rumo_mundo):
        return None
    r = info[:, 4] - jan.rumo_mundo
    vel = np.linalg.norm(np.gradient(jan.juntas["centro_massa"][:, :2], jan.cap.dt, axis=0), axis=1)
    v = vel[:, None] * np.stack([np.cos(r), np.sin(r)], -1) * jan.cap.dt
    return jan.juntas["pelve"][0, :2] + np.vstack([[0.0, 0.0], np.cumsum(v, 0)[:-1]])


def _desenhar_pegadas(ax, jan: Janela, peg: list, a: int, b: int, lims: tuple) -> None:
    _grade(ax, lims, False)
    ped = _linha_pedido(jan)
    if ped is not None:
        ax.plot(ped[a:b, 0], ped[a:b, 1], color=CHAO, lw=1.0, ls="--", zorder=1)
    com = jan.juntas["centro_massa"][a:b]
    ax.plot(com[:, 0], com[:, 1], color=CLARO, lw=0.9, alpha=0.55, zorder=2)
    c = _apoio(jan)
    for k, s in enumerate("ed"):
        p = jan.juntas[f"pe_{s}"][a:b, :2].copy()
        voo = ~c[a:b, k]
        # (O balanco: o centro do pe fora do chao, um quadro de cada lado para ligar as pegadas.)
        mostra = voo | np.r_[voo[1:], False] | np.r_[False, voo[:-1]]
        p[~mostra] = np.nan
        ax.plot(p[:, 0], p[:, 1], color=PE[s], lw=0.9, alpha=0.75, zorder=3)
    for g in peg:
        if g["b"] <= a or g["a"] >= b:
            continue
        cor = PE[g["lado"]]
        for sola in g["solas"]:
            ax.add_patch(_poligono(sola, cor, 0.06, 0.0, z=3.5))
        ax.add_patch(_poligono(g["sola"], cor, 0.5, 1.3, z=4))
        cen = g["sola"].mean(0)
        sinal = 1.0 if g["lado"] == "e" else -1.0
        if "junto" not in g:
            ax.text(cen[0], cen[1] + sinal * 0.13, g["rotulo"], color=cor, fontsize=8.5, weight="bold", ha="center",
                    va="bottom" if sinal > 0 else "top", zorder=6, path_effects=_traco(0.5))
        if g["passo"]:
            comp, larg, _ = g["passo"]
            ax.text(cen[0], cen[1] + sinal * 0.27, f"{_n(comp, 2)} m · {_n(larg * 100, 0)} cm", color=TEXTO,
                    fontsize=7.2, ha="center", va="bottom" if sinal > 0 else "top", zorder=6, path_effects=_traco(0.5))
        if g["arrasto_m"] > ARRASTO_M:
            # (A seta 2x o arrasto, no maximo 0,7 m: a 4x as de 50 cm cruzavam as pegadas vizinhas.)
            vec = 2.0 * g["arrasto"]
            fim = cen + vec * min(1.0, 0.7 / max(np.linalg.norm(vec), 1e-9))
            ax.add_patch(FancyArrowPatch(cen, fim, arrowstyle="-|>", mutation_scale=9, color=VERMELHO, lw=1.6, zorder=7))
            ax.text(fim[0], fim[1] - sinal * 0.05, f"{_n(g['arrasto_m'] * 100, 0)} cm", color=VERMELHO, fontsize=7.2,
                    ha="center", va="top" if sinal > 0 else "bottom", zorder=7, path_effects=_traco(0.5))


def pegadas(jans: list[Janela], recorte: float | None = None) -> Figure:
    """As pegadas (D), de cima e na escala de verdade: cada apoio e o contorno da sola no chao no meio dele (cheio; o
    rastro fraco e a sola em cada quadro do apoio: o pe que gira ou escorrega borra), numerado em ordem (E1, D1, E2...),
    esquerda azul e direita laranja; o balanco (o centro do pe no ar) liga as pegadas do mesmo pe; o arrasto do pivo no
    apoio acima de 3 cm vira a seta vermelha (4x o tamanho, o total escrito); o passo desde o pouso do outro pe (no caminho
    e de lado) sob cada nome; a tracejada e o caminho que o pedido daria e a clara o centro de massa. Uma linha a cada
    LINHA_M de caminho (ou a cada `recorte` s)."""
    blocos = []
    for jan in jans:
        if jan.modo != "caminho":
            raise ValueError("as pegadas pedem a janela no modo 'caminho'")
        peg = _pegadas_dados(jan)
        # (Com o pedido fixo, a reta dele entra na linha: o sprint a 30% sai 2 m para a direita e ela sumia da vista.)
        ped = _linha_pedido(jan) if _pedido_fixo(jan) else None
        partes = []
        for a, b in _linhas_planta(jan, recorte):
            pts = [jan.juntas[f"pe_{s}"][a:b, :2] for s in "ed"] + [jan.juntas["centro_massa"][a:b, :2]]
            pts += [g["sola"] for g in peg if a <= g["a"] < b] + ([] if ped is None else [ped[a:b]])
            lims = _lims(*pts, margem=(0.45, 0.45, 0.42, 0.42))
            partes.append((a, b, lims))
        blocos.append((jan, peg, partes))
    mx = 16.0
    largura = max(p[2][1] - p[2][0] for _, _, partes in blocos for p in partes)
    S = min(300.0, (LARGURA2 - 2 * mx) / largura)
    W = max(largura * S + 2 * mx, 1300.0)
    caixas, y = [], 84.0
    for jan, peg, partes in blocos:
        resumo = _resumo_pegadas(jan, peg)
        for k, (a, b, lims) in enumerate(partes):
            titulo = (f"{_cabecalho(jan)}\n{resumo}" if k == 0 else "") + \
                     f"{chr(10) if k == 0 else ''}{_n(jan.t[a], 1)}–{_n(jan.t[b - 1] + jan.cap.dt, 1)} s"
            lin = titulo.count("\n")
            h = (lims[3] - lims[2]) * S
            caixas.append(_Caixa(mx, y + 16 + 15 * lin, (lims[1] - lims[0]) * S, h,
                                 lambda ax, jan=jan, peg=peg, a=a, b=b, lims=lims: _desenhar_pegadas(ax, jan, peg, a, b, lims),
                                 titulo, lims))
            y += 16 + 15 * lin + h + 12
        y += 10
    H = y + 70
    leg = [Patch(facecolor=to_rgba(PE["e"], 0.5), edgecolor=PE["e"], label="pegada esquerda (E1, E2…)"),
           Patch(facecolor=to_rgba(PE["d"], 0.5), edgecolor=PE["d"], label="pegada direita (D1, D2…)"),
           Line2D([], [], color=PE["e"], lw=0.9, label="balanço (o pé no ar)"),
           Line2D([], [], color=VERMELHO, lw=1.6, marker=">", ms=5, label="arrasto do pé no apoio > 3 cm (seta 2×, até 0,7 m)"),
           Line2D([], [], color=CHAO, lw=1.0, ls="--", label="o rumo pedido"),
           Line2D([], [], color=CLARO, lw=0.9, alpha=0.55, label="centro de massa")]

    def rodape(fig, W_, H_):
        fig.legend(handles=leg, loc="lower left", bbox_to_anchor=(12 / W_, 6 / H_), ncol=4, frameon=False, fontsize=8.5,
                   labelcolor=TEXTO, handlelength=2.4, columnspacing=1.6)

    return _figura(caixas, W, H, "Pegadas — de cima, na escala de verdade (o caminho →, a esquerda ↑) · grade 0,5 m",
                   "cada apoio: a sola no meio do apoio (cheia) e em cada quadro dele (o borrão = giro ou deslize); sob o "
                   "nome, o passo desde o pouso do outro pé (no caminho · de lado); D6–7 = o pé quicou (voltou ao chão em "
                   "até 0,13 s); o pé no chão: a sola abaixo de 1,5 cm (marcha.contato) ou a física empurrando o chão",
                   rodape)


def _resumo_pegadas(jan: Janela, peg: list) -> str:
    txt = []
    for s in "ed":
        g = [x for x in peg if x["lado"] == s and "junto" not in x]
        passos = [x["passo"] for x in g if x["passo"]]
        arr = [x["arrasto_m"] * 100 for x in g]
        quiques = sum(x["quiques"] for x in g)
        txt.append(f"{LETRA[s]}: {len(g)} apoios" + (f" ({quiques} quicam)" if quiques else "")
                   + f", passo {_n(np.mean([p[0] for p in passos]) if passos else np.nan, 2)} m, "
                   f"largura {_n(np.mean([p[1] for p in passos]) * 100 if passos else np.nan, 0)} cm, arrasto "
                   f"{_n(np.mean(arr) if arr else np.nan, 1)} cm (máx. {_n(max(arr) if arr else np.nan, 1)})")
    tempos = [x["passo"][2] for x in peg if x["passo"]]
    cad = f" · cadência {_n(60.0 / np.mean(tempos), 0)} passos/min" if tempos else ""
    return " · ".join(txt) + cad


# --- v2: o equilibrio (E) ---------------------------------------------------------------------------------------------------

ANDA_MS = 0.5  # (m/s: o centro de massa acima disso anda, e a margem passa a ser a de lado do movimento)


def _equilibrio_dados(fr: np.ndarray, geo: list, dt: float, fisica: np.ndarray | None = None) -> dict:
    """O equilibrio de um conjunto de poses [F, 18, 7] no referencial da janela: o pe no chao, a base (o casco dos pontos da
    sola no chao dos pes apoiados) e o centro dela, o centro de massa, o ponto de captura (o centro de massa + v/omega,
    omega = raiz(g/z)) e a margem dele (m, + = a salvo). Andando (a velocidade de 0,3 s do centro de massa acima de
    ANDA_MS), a margem de estabilidade de Hof na normal do movimento: num pe so, ate a borda de FORA dele (a do lado oposto
    ao centro de massa naquele apoio); nos dois, ate a borda mais perto. Parado, a distancia com sinal a base no plano.
    (05/11: antes, andando, a distancia as duas bordas de lado do pe de apoio: o ponto de captura por dentro do pe - o
    normal da marcha, o outro pe o apanha - contava como fora, e o proprio clipe ficava "fora da base" em 43-81% dos
    quadros.) A margem, o centro de massa contra o centro da base (na esquerda do corpo, m) e a inclinacao de lado so de pe
    (a pelve acima de 0,6 m: deitado a base e o corpo, nao os pes, e a inclinacao vai a +-180 graus)."""
    with np.errstate(invalid="ignore", divide="ignore"):
        j = G.articulacoes(fr, geo)
        cas = {s: M.casca(fr, geo, lado) for s, lado in M.LADOS}
        c = _no_chao(fr, geo, fisica)
        com = j["centro_massa"]
        v = np.gradient(com, dt, axis=0)
        xcom = com[:, :2] + v[:, :2] / np.sqrt(GRAVIDADE / np.maximum(com[:, 2:3], 0.2))
        esq = G.referencial(fr)["esquerda"][:, :2]
        h = max(1, int(round(0.15 / dt)))
        vs = np.stack([np.convolve(np.pad(np.nan_to_num(v[:, k]), h, mode="edge"), np.full(2 * h + 1, 1.0 / (2 * h + 1)),
                                   "valid") for k in (0, 1)], -1)
        rapido = np.linalg.norm(vs, axis=1) > ANDA_MS
        nor = np.stack([-vs[:, 1], vs[:, 0]], -1) / np.maximum(np.linalg.norm(vs, axis=1, keepdims=True), 1e-9)
        de_pe = j["pelve"][:, 2] > 0.6
    F = len(fr)
    pts = [[None, None] for _ in range(F)]
    for i in range(F):
        for k, s in enumerate("ed"):
            if c[i, k]:
                z = cas[s][i, :, 2]
                pts[i][k] = cas[s][i, z < max(M.TOQUE_M, z.min() + 0.01), :2]
    # (O lado de fora de cada apoio: onde o pe fica contra o centro de massa na normal do movimento, a mediana do apoio.)
    fora = np.ones((F, 2))
    with np.errstate(invalid="ignore"):
        for s, a, b, _ in _apoios(c):
            k = "ed".index(s)
            fora[a:b, k] = 1.0 if np.nanmedian([(pts[i][k].mean(0) - com[i, :2]) @ nor[i] for i in range(a, b)]) >= 0 else -1.0
    bases, centro = [], np.full((F, 2), np.nan)
    margem = np.full(F, np.nan)
    for i in range(F):
        P = [p for p in pts[i] if p is not None]
        base = _casco(np.concatenate(P)) if P else None
        bases.append(base)
        if base is not None and len(base) >= 3:
            centro[i] = base.mean(0)
            if not de_pe[i] or not np.isfinite(xcom[i]).all():
                continue
            if rapido[i]:
                x = xcom[i] @ nor[i]
                k = 0 if pts[i][0] is not None else 1
                y = base @ nor[i] if len(P) == 2 else pts[i][k] @ nor[i]
                margem[i] = (min(y.max() - x, x - y.min()) if len(P) == 2 else
                             y.max() - x if fora[i, k] > 0 else x - y.min())
            else:
                margem[i] = _margem(xcom[i], base)
        elif np.isfinite(j["pe_e"][i]).all():
            centro[i] = 0.5 * (j["pe_e"][i, :2] + j["pe_d"][i, :2])
    with np.errstate(invalid="ignore"):
        com_lado = np.where(de_pe, ((com[:, :2] - centro) * esq).sum(-1), np.nan)
        lado = np.where(de_pe, _lado_corpo(fr, geo), np.nan)
    return {"c": c, "bases": bases, "centro": centro, "com": com, "xcom": xcom, "margem": margem, "com_lado": com_lado,
            "lado": lado, "j": j, "rapido": rapido, "rumo": np.arctan2(vs[:, 1], vs[:, 0])}


def _margem(p: np.ndarray, poli: np.ndarray) -> float:
    """A distancia com sinal (m) do ponto ao poligono convexo anti-horario: + dentro (ate a borda mais perto), - fora."""
    e = np.roll(poli, -1, 0) - poli
    n = np.stack([-e[:, 1], e[:, 0]], 1) / np.maximum(np.linalg.norm(e, axis=1, keepdims=True), 1e-12)
    d = ((p - poli) * n).sum(1)
    if (d >= 0).all():
        return float(d.min())
    t = np.clip(((p - poli) * e).sum(1) / np.maximum((e * e).sum(1), 1e-12), 0.0, 1.0)
    return -float(np.linalg.norm(poli + t[:, None] * e - p, axis=1).min())


def _reto(eq: dict, a: int, b: int) -> bool:
    """O trecho [a, b) anda em linha reta: o rumo do centro de massa (andando) varia menos de 30 graus nele."""
    r = eq["rumo"][a:b][eq["rapido"][a:b]]
    return len(r) > 1 and float(np.ptp(np.unwrap(r))) < np.radians(30.0)


EXAGERO = 3.0  # (o eixo de lado da planta do equilibrio: os centimetros de lado que importam ficam visiveis)


def _exagero(com: np.ndarray, ex: float = EXAGERO, girar: bool = True) -> Callable:
    """A transformacao da planta do equilibrio num trecho: com `girar`, o x no rumo de fato dele (o eixo principal do
    caminho do centro de massa, do comeco ao fim) e o de lado exagerado `ex` vezes em volta dessa reta (0 = a reta pelo
    primeiro centro de massa). Exagerar o y do caminho entortava tudo quando o corpo desviava do pedido (o sprint a 30%
    sai 1 m para a direita); a reta pelo polyfit de y em x falhava no trecho que anda em y (a troca)."""
    c = com[:, :2][np.isfinite(com[:, :2]).all(1)]
    o = c[0]
    phi = 0.0
    if girar and np.ptp(c, 0).max() > 0.3:
        e = np.linalg.svd(c - c.mean(0), full_matrices=False)[2][0]
        e = -e if e @ (c[-1] - c[0]) < 0 else e
        phi = float(np.arctan2(e[1], e[0]))
    rot = np.array([[np.cos(phi), np.sin(phi)], [-np.sin(phi), np.cos(phi)]])

    def tf(p: np.ndarray) -> np.ndarray:
        q = (np.asarray(p, float)[..., :2] - o) @ rot.T
        return q * [1.0, ex]

    tf.angulo, tf.ex = phi, ex
    return tf


def _desenhar_equilibrio_planta(ax, jan: Janela, eq: dict, eqr: dict | None, q: np.ndarray, a: int, b: int, lims: tuple,
                                peg: list, _ex: Callable) -> None:
    """A planta do equilibrio pela transformacao `_ex` (_exagero: andando, o de lado exagerado em volta do rumo de fato;
    parado, ampliada sem exagero); a grade de lado a cada 10 cm de verdade, a do caminho a cada 0,5 m (10 cm parado)."""
    x0, x1, y0, y1 = lims
    ex = _ex.ex
    # (Exagerado: 0,5 m no caminho e 10 cm de verdade de lado; na escala de verdade, 10 cm parado e 0,5 m andando.)
    passo_x = 0.5 if ex != 1.0 or x1 - x0 > 2.0 else 0.1
    passo_y = 0.1 * ex if ex != 1.0 else passo_x
    ax.vlines(np.arange(np.ceil(x0 / passo_x) * passo_x, x1, passo_x), y0, y1, colors=GRADE, linewidths=0.6, zorder=0)
    gy = np.arange(np.ceil(y0 / passo_y), np.floor(y1 / passo_y) + 1) * passo_y
    ax.hlines(gy, x0, x1, colors=GRADE, linewidths=0.6, zorder=0)
    for yy in gy[gy < y1 - 0.4 * passo_y]:
        ax.text(x0 + 0.01 * (x1 - x0), yy + 0.05 * passo_y, f"{_n(yy / ex * 100, 0, True) if abs(yy) > 1e-9 else '0'} cm",
                color=CHAO, fontsize=7, va="bottom", zorder=0.5, clip_on=True)
    ped = _linha_pedido(jan)
    if ped is not None:
        # (Desde a pelve do comeco do trecho: desde a primeira pelve da janela, a reta ia parar longe do trecho - a curva, a
        # troca, e a diagonal a 30% que desvia 10 graus. O desvio de lado acumulado fica nas pegadas.)
        ped = ped[a:b] - ped[a] + jan.juntas["pelve"][a, :2]
        ped = _ex(ped)
        ax.plot(ped[:, 0], ped[:, 1], color=CHAO, lw=0.9, ls="--", zorder=1)
    for g in peg:
        if g["b"] > a and g["a"] < b:
            ax.add_patch(_poligono(_ex(g["sola"]), PE[g["lado"]], 0.16, 0.6, z=1.5))
            if "junto" in g:
                continue
            c = _ex(g["sola"]).mean(0)
            ax.text(c[0], c[1] + (0.06 if g["lado"] == "e" else -0.06) * ex, g["rotulo"], color=PE[g["lado"]],
                    fontsize=7.5, ha="center", va="center", zorder=2, weight="bold", clip_on=True)
    com = _ex(eq["com"])
    ax.plot(com[a:b, 0], com[a:b, 1], color=CLARO, lw=1.1, zorder=4)
    xc, m = _ex(eq["xcom"])[a:b], eq["margem"][a:b]
    segs = np.stack([xc[:-1], xc[1:]], 1)
    cor = np.where(np.isnan(m[:-1]), 0, np.where(m[:-1] >= 0, 1, 2))
    ax.add_collection(LineCollection(segs, colors=np.array(["#6b7280", VERDE, VERMELHO])[cor], linewidths=1.8, zorder=5))
    cores = TEMPO2((jan.t[q] - jan.t[0]) / max(jan.t[-1] - jan.t[0], 1e-9))
    j = eq["j"]
    for i, cor_t in zip(q, cores):
        # (Deitado, a base e o corpo, nao os pes, e a seta tornozelos -> cabeca e o comprimento dele: so de pe.)
        if not a <= i < b or not j["pelve"][i, 2] > 0.6:
            continue
        base = eq["bases"][i]
        if base is not None and len(base) >= 3:
            # (O preenchimento fraco: parado, as bases de todos os instantes sao a mesma e empilhavam uma mancha.)
            ax.add_patch(Polygon(_ex(base), closed=True, facecolor=to_rgba(cor_t, 0.05), edgecolor=cor_t, lw=1.2, zorder=3))
        pe_m = 0.5 * (j["tornozelo_e"][i] + j["tornozelo_d"][i])
        pe = _ex(pe_m)
        ax.add_patch(FancyArrowPatch(pe, _ex(j["cabeca"][i]), arrowstyle="-|>", mutation_scale=8, color=cor_t, lw=1.4,
                                     zorder=6))
        if eqr is not None and np.isfinite(eqr["j"]["cabeca"][i]).all():
            # (A do clipe na mesma origem: a cabeca dele menos o meio dos tornozelos dele, somada aos do corpo.)
            jr = eqr["j"]
            d = jr["cabeca"][i] - 0.5 * (jr["tornozelo_e"][i] + jr["tornozelo_d"][i])
            ax.add_patch(FancyArrowPatch(pe, _ex(pe_m + d), arrowstyle="-|>", mutation_scale=7, color=REF, lw=0.9, ls="--",
                                         alpha=0.8, zorder=5.5))
        cor_m = VERDE if eq["margem"][i] >= 0 else VERMELHO if np.isfinite(eq["margem"][i]) else "#6b7280"
        xi = _ex(eq["xcom"][i])
        ax.plot([com[i, 0], xi[0]], [com[i, 1], xi[1]], color=cor_m, lw=0.7, alpha=0.6, zorder=5.8)
        ax.plot(*com[i], "o", color=CLARO, ms=3.4, zorder=6, mec=FUNDO, mew=0.5)
        ax.plot(*xi, "o", ms=4.0, zorder=6.5, mec=FUNDO, mew=0.5, color=cor_m)


def equilibrio(jan: Janela, passo: float | None = None, recorte: float | None = None,
               detalhe: float | None = 2.0) -> Figure:
    """O equilibrio (E): de cima, o detalhe (o mesmo trecho do movimento, _detalhe; com `recorte`, a janela inteira em
    linhas de `recorte` s): a base (o casco dos pontos da sola no chao) nos instantes marcados, o caminho do centro de
    massa (claro) e o do ponto de captura (verde com margem, vermelho alem da borda, cinza no voo; a linha fina liga os
    dois no mesmo instante: v/omega), a inclinacao do corpo em setas (o meio dos tornozelos -> a cabeca; a do clipe
    tracejada na mesma origem); embaixo, na janela inteira e no tempo (a faixa clara e o trecho de cima): a margem do ponto
    de captura (_equilibrio_dados: a de Hof andando, a da base parado), o centro de massa contra o centro da base na
    esquerda do corpo, a inclinacao de lado contra o clipe e as fases dos pes (o diagrama da marcha; o do clipe em cinza).
    O de lado da planta e exagerado EXAGERO vezes so no trecho reto (numa curva ou troca ele entortava tudo)."""
    if jan.modo != "caminho":
        raise ValueError("o equilibrio pede a janela no modo 'caminho'")
    dt = jan.cap.dt
    anda = _anda(jan)
    eq = _equilibrio_dados(jan.corpo, jan.geo, dt, _fisica(jan))
    eqr = _equilibrio_dados(jan.ref, jan.geo, dt) if jan.ref is not None else None
    peg = _pegadas_dados(jan)
    if recorte:
        partes = _linhas_planta(jan, recorte)
    else:
        # (O trecho do detalhe onde ele estiver: antes os ultimos 2 s, e o equilibrio do lev_levantar mostrava 4-6 s de pe
        # com o movimento no levantar, 0-2 s.)
        jv = _detalhe(jan, detalhe)
        i0 = int(round((jv.t[0] - jan.t[0]) / dt))
        partes = [(i0, i0 + len(jv.t))]
    q = _quadros_fantasma(jan, max(passo or 0.0, 0.2) if anda else (passo or 0.4))
    linhas = []
    for a, b in partes:
        tf = _exagero(eq["com"][a:b], EXAGERO if anda and _reto(eq, a, b) else 1.0, girar=anda)
        pts = [eq["com"][a:b], eq["xcom"][a:b], *(g["sola"] for g in peg if g["b"] > a and g["a"] < b),
               eq["j"]["cabeca"][a:b]]
        linhas.append((a, b, _lims(*(tf(p) for p in pts), margem=(0.35, 0.35, 0.3, 0.3) if anda else (0.08,) * 4), tf))
    mx, mxl = 16.0, 58.0
    largura = max(p[2][1] - p[2][0] for p in linhas)
    # (Parado, a base inteira cabe em meio metro: a planta ampliada ate 900 px/m, o estabilograma.)
    S = min(ESC_MAX if anda else 900.0, (LARGURA2 - mx - mxl) / largura, 520.0 / max(p[2][3] - p[2][2] for p in linhas))
    W = max(largura * S + mx + mxl, 1400.0)
    caixas, y = [], 84.0
    for a, b, lims, tf in linhas:
        h = (lims[3] - lims[2]) * S
        titulo = (f"de cima, no rumo de fato do trecho (→; o pedido é a tracejada: {_n(-np.degrees(tf.angulo), 1, True)}° "
                  f"dele) · o de lado exagerado {_n(EXAGERO, 0)}× (grade de 10 cm; no caminho, 0,5 m)" if tf.ex != 1.0 else
                  "de cima, na escala de verdade, no eixo do trecho (→; ele faz curva: sem exagero; grade de 0,5 m)" if anda else
                  "de cima, ampliado (o estabilograma; grade de 10 cm)")
        titulo = _quebrar(titulo + f" · {_n(jan.t[a], 1)}–{_n(jan.t[b - 1] + dt, 1)} s · a base, o centro de massa e o "
                          f"ponto de captura a cada {_n((q[1] - q[0]) * dt if len(q) > 1 else 0.0, 2)} s", W - mxl - 30)
        topo = 20 + 15 * titulo.count("\n")
        caixas.append(_Caixa(mxl, y + topo, (lims[1] - lims[0]) * S, h,
                             lambda ax, a=a, b=b, lims=lims, tf=tf: _desenhar_equilibrio_planta(ax, jan, eq, eqr, q, a, b,
                                                                                                lims, peg, tf),
                             titulo, lims))
        y += topo + h + 10
    t = jan.t
    voo = ~eq["c"].any(1)
    wf = W - mx - mxl

    def base_tempo(ax, zero=True):
        ax.set_xlim(t[0], t[-1] + dt)
        if not recorte:
            for a, b, *_ in linhas:
                ax.axvspan(t[a], t[b - 1] + dt, color="#ffffff", alpha=0.045, lw=0, zorder=0)
        for i0, i1 in _trechos_bool(voo):
            ax.axvspan(t[i0], t[i1 - 1] + dt, color="#6b7280", alpha=0.14, lw=0)
        if zero:
            ax.axhline(0.0, color=CHAO, lw=0.8)
        ax.tick_params(labelbottom=False)

    def f_margem(ax):
        base_tempo(ax)
        m = eq["margem"] * 100
        if eqr is not None:
            ax.plot(t, eqr["margem"] * 100, color="#9aa4b2", lw=0.9, ls=_TRACO)
        ax.plot(t, np.where(m >= 0, m, np.nan), color=VERDE, lw=1.6)
        ax.plot(t, np.where(m < 0, m, np.nan), color=VERMELHO, lw=1.6)

    def f_com(ax):
        base_tempo(ax)
        if eqr is not None:
            ax.plot(t, eqr["com_lado"] * 100, color="#9aa4b2", lw=0.9, ls=_TRACO)
        ax.plot(t, eq["com_lado"] * 100, color=CLARO, lw=1.5)

    def f_lado(ax):
        base_tempo(ax)
        if eqr is not None:
            ax.plot(t, eqr["lado"], color="#9aa4b2", lw=0.9, ls=_TRACO)
        ax.plot(t, eq["lado"], color=CLARO, lw=1.5)

    def f_fases(ax):
        base_tempo(ax, zero=False)
        ax.tick_params(labelbottom=True)
        for k, (s_, yy) in enumerate((("e", 1.0), ("d", 0.0))):
            for g in peg:
                if g["lado"] == s_:
                    ax.broken_barh([(t[g["a"]], (g["b"] - g["a"]) * dt)], (yy - 0.3, 0.6), facecolors=to_rgba(PE[s_], 0.8),
                                   edgecolors=PE[s_], lw=0.6)
                    if (g["b"] - g["a"]) * dt > 0.09:
                        ax.text(t[g["a"]] + 0.5 * (g["b"] - g["a"]) * dt, yy, f"{LETRA[s_]}{g['n']}", color=FUNDO,
                                fontsize=7, ha="center", va="center", weight="bold")
            if eqr is not None:
                for i0, i1 in _trechos_bool(eqr["c"][:, k]):
                    ax.plot([t[i0], t[i1 - 1] + dt], [yy - 0.42, yy - 0.42], color="#9aa4b2", lw=1.8,
                            solid_capstyle="butt")
        ax.set_yticks([0.0, 1.0], labels=["D", "E"])
        ax.set_ylim(-0.6, 1.45)
        ax.set_xlabel("tempo (s)", color=TEXTO, fontsize=8.5)

    num = _numeros(jan)
    mr = np.nanmean(eqr["margem"]) * 100 if eqr is not None else np.nan
    faixas = [
        (("margem de Hof do ponto de captura até a borda de fora do pé de apoio" if anda else
          "margem do ponto de captura à borda da base") + f" (cm; + = a salvo; cinza = voo) · média "
         f"{_n(np.nanmean(eq['margem']) * 100, 1, True)}, além da borda em {_n(_fracao(eq['margem'] < 0, eq['margem']), 0)}% "
         "dos quadros" + (f" (clipe {_n(mr, 1, True)} e {_n(_fracao(eqr['margem'] < 0, eqr['margem']), 0)}%, tracejada)"
                          if eqr is not None else ""), f_margem, 84),
        (f"centro de massa − centro da base, na esquerda do corpo (cm, + = esquerda) · média "
         f"{_n(np.nanmean(eq['com_lado']) * 100, 1, True)}"
         + (f" (clipe {_n(np.nanmean(eqr['com_lado']) * 100, 1, True)})" if eqr is not None else ""), f_com, 84),
        (f"inclinação de lado do corpo (°, + = esquerda) · rede {_n(num['lado'], 1, True)}"
         + (f", clipe {_n(num['lado_clipe'], 1, True)} (Δ {_n(num['delta'], 1, True)})" if np.isfinite(num["lado_clipe"])
            else ""), f_lado, 84),
        ("fases dos pés (o diagrama da marcha): a barra é o pé no chão" + (" · o traço cinza, o do clipe" if eqr is not None
                                                                            else ""), f_fases, 80),
    ]
    for titulo, f, h in faixas:
        caixas.append(_Caixa(mxl, y + 18, wf, h, f, titulo))
        y += 18 + h + 8
    H = y + 84
    leg = [Line2D([], [], color=CLARO, lw=1.1, marker="o", ms=3.5, label="centro de massa (no chão)"),
           Line2D([], [], color=VERDE, lw=1.8, marker="o", ms=4, label="ponto de captura a salvo (margem ≥ 0)"),
           Line2D([], [], color=VERMELHO, lw=1.8, marker="o", ms=4, label="ponto de captura além da borda (margem < 0)"),
           Patch(facecolor=to_rgba(TEMPO2(0.5), 0.18), edgecolor=TEMPO2(0.5), label="a base no instante (cor do tempo)"),
           Line2D([], [], color=TEMPO2(0.5), lw=1.4, marker=">", ms=5, label="inclinação: tornozelos → cabeça"),
           Line2D([], [], color=REF, lw=0.9, ls="--", label="a do clipe (mesma origem)"),
           Patch(facecolor=to_rgba(PE["e"], 0.4), edgecolor=PE["e"], label="pegada esquerda"),
           Patch(facecolor=to_rgba(PE["d"], 0.4), edgecolor=PE["d"], label="pegada direita"),
           Line2D([], [], color="#9aa4b2", lw=0.9, ls=_TRACO, label="o clipe nas faixas")]
    titulo = f"Equilíbrio — {_cabecalho(jan)}"
    sub = (_resumo(jan, num) + "\nponto de captura = centro de massa + v/ω (ω = √(g/z)); "
           + ("andando, a margem de Hof: de lado do movimento, até a borda de fora do pé de apoio (por dentro dele é o "
              "normal da marcha)" if anda else "parado, ele deve ficar dentro da base"))
    t0, t1 = t[linhas[0][0]], t[linhas[-1][1] - 1] + dt
    return _figura(caixas, W, H + 16, titulo, sub, _rodape(leg, t0, t1, "tempo (s) — os instantes marcados de cima"))


def _trechos_bool(m: np.ndarray) -> list[tuple[int, int]]:
    d = np.diff(np.r_[0, m.astype(np.int8), 0])
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def _fracao(m: np.ndarray, valores: np.ndarray) -> float:
    """A porcentagem de quadros com `m` entre os de `valores` finitos (nan sem nenhum)."""
    ok = np.isfinite(valores)
    return float(100.0 * m[ok].mean()) if ok.any() else float("nan")


# --- O conjunto padrao, a linha de comando e o autoteste ---------------------------------------------------------------

def _ordem_modo(modo: str) -> tuple:
    return (0, 0.0) if modo == "atual" else (2, 0.0) if modo == "clipe" else (1, -float(modo))


def irmas(caminho: str | Path) -> list[Path]:
    """As capturas do mesmo exercicio na pasta (atual, as forcas da maior a menor, clipe)."""
    p = Path(caminho)
    return sorted(p.parent.glob(f"{p.stem.split('__')[0]}__*.npz"), key=lambda q: _ordem_modo(q.stem.split("__")[1]))


def salvar(fig: Figure, caminho: Path, log=print) -> Path:
    caminho.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(caminho, dpi=100, facecolor=FUNDO)
    log(f"  {caminho}")
    return caminho


def padrao(caminhos: list[Path], saida: Path, mundo: int = 0, log=print) -> list[Path]:
    """O conjunto padrao de um exercicio (as capturas dele, p.ex. atual, 0.3 e clipe). O do v1 continua com os mesmos nomes
    (o inspecao.auditor liga os achados a eles): o fantasma no lugar e no mundo, o desvio do clipe, a folha e o espelho
    (quando as capturas do exercicio espelhado estao na pasta). O v2 soma: o movimento (com o clipe em contorno) e o
    equilibrio de cada captura da rede (as do clipe puro so se nao houver rede), as pegadas e o parado de todas."""
    caps = [G.carregar(c) for c in caminhos]
    ex = caps[0].meta["exercicio"]
    lugar = [janela(c, mundo) for c in caps]
    no_mundo = [janela(c, mundo, modo="mundo") for c in caps]
    rede = [j for j in lugar if not j.cap.meta["clipe_puro"]] or lugar
    feitos = [salvar(fantasma(lugar, VISTAS_LUGAR), saida / f"{ex}__fantasma_lugar.png", log),
              salvar(fantasma(no_mundo, vistas_mundo(no_mundo[0])), saida / f"{ex}__fantasma_mundo.png", log),
              salvar(fantasma(lugar, ("frente", "lado_dir", "cima"), desvio=True), saida / f"{ex}__desvio.png", log),
              salvar(folha(rede[:2]), saida / f"{ex}__folha.png", log)]
    for cap in [c for c in caps if not c.meta["clipe_puro"]] or caps:
        jan = janela(cap, mundo, modo="caminho")
        feitos.append(salvar(movimento(jan, referencia=True), saida / f"{cap.nome}__movimento.png", log))
        feitos.append(salvar(equilibrio(jan), saida / f"{cap.nome}__equilibrio.png", log))
    feitos.append(salvar(pegadas([janela(c, mundo, modo="caminho") for c in caps]), saida / f"{ex}__pegadas.png", log))
    feitos.append(salvar(parado(lugar), saida / f"{ex}__parado.png", log))
    par = comum.mirror_of(ex)
    pares = [(j, Path(c).with_name(f"{par}__{j.cap.meta['modo']}.npz")) for j, c in zip(lugar, caminhos)] if par else []
    pares = [(j, janela(G.carregar(o), mundo)) for j, o in pares if o.exists()]
    if pares:
        feitos.append(salvar(espelho(pares)[0], saida / f"{ex}__espelho.png", log))
    return feitos


def autoteste(amostras: Path = AMOSTRAS, saida: Path = SAIDA_TESTE) -> None:
    """Desenha nas amostras o conjunto padrao do lev_trote_0, o espelho lev_corrida_2 x lev_corrida_6, o lev_levantar (as
    poses no chao) e o lev_giro, e confere os fatos: o clipe puro ereto de lado (|vies| < 1,5 grau) e a rede a 30%
    inclinada para a direita no lev_trote_0; a passada do trote (~0,7 s), nenhuma no parado, e a folha numa passada; o
    espelho do clipe puro coincide (as inclinacoes iguais, vies ~0, coincidencia alta); a janela nao atravessa a queda e o
    recomeco (lev_troca_media 0.3, mundo 2); a do levantar comeca no chao; no giro o corpo fica de frente no lugar; o
    fantasma de 6 vistas x 3 capturas em segundos."""
    feitos = []
    t = time.time()
    trote = irmas(amostras / "lev_trote_0__atual.npz")
    assert [p.stem.split("__")[1] for p in trote] == ["atual", "0.3", "clipe"], trote
    feitos += padrao(trote, saida)
    print(f"conjunto padrao do lev_trote_0: {time.time() - t:.1f} s")
    t = time.time()
    jans = [janela(G.carregar(p)) for p in trote]
    fantasma(jans, tuple(VISTAS)).savefig(saida / "tempo_6vistas.png", dpi=100, facecolor=FUNDO)
    dt = time.time() - t
    print(f"6 vistas x 3 capturas: {dt:.1f} s")
    assert dt < 20.0, "o fantasma de 6 vistas x 3 capturas deveria sair em segundos"
    clipe, rede = jans[2].medidas, jans[1].medidas
    print(f"lev_trote_0 inclinacao de lado do corpo: clipe {clipe['lado_corpo']:+.2f}, rede a 30% {rede['lado_corpo']:+.2f} "
          f"(tronco {clipe['lado_tronco']:+.2f} / {rede['lado_tronco']:+.2f})")
    assert abs(clipe["lado_corpo"]) < 1.5, "o clipe puro deveria ser ereto de lado"
    assert rede["lado_corpo"] < 0.0, "a rede a 30% deveria inclinar para a direita (o vies achado)"
    q = _poses(jans[0], 10)
    print(f"passada lev_trote_0 (atual, 0.3, clipe): {', '.join(f'{j.passada:.2f}' for j in jans)} s; folha da rede atual "
          f"{jans[0].t[q[0]]:.2f}-{jans[0].t[q[-1]]:.2f} s")
    assert all(0.6 < j.passada < 0.8 for j in jans), "o trote deveria ter uma passada de ~0,7 s"
    assert q[-1] - q[0] < round(jans[0].passada / jans[0].cap.dt), "a folha do trote deveria cobrir uma passada"
    assert janela(G.carregar(amostras / "lev_parado__clipe.npz")).passada is None, "o parado nao tem passada"

    pares = [(janela(G.carregar(amostras / f"lev_corrida_2__{m}.npz")), janela(G.carregar(amostras / f"lev_corrida_6__{m}.npz")))
             for m in ("atual", "clipe")]
    fig, med = espelho(pares)
    feitos.append(salvar(fig, saida / "lev_corrida_2__espelho.png"))
    for m, x in zip(("atual", "clipe"), med):
        a, b = x["lado_corpo"]
        print(f"espelho lev_corrida_2 x 6 ({m}): lado do corpo {a:+.2f} x {b:+.2f} (vies {(a - b) / 2:+.2f}), "
              f"coincidencia " + ", ".join(f"{k} {x[k]:.0%}" for k in ("frente", "cima", "lado_dir")))
    a, b = med[1]["lado_corpo"]
    assert abs(a - b) < 1.0, "o clipe puro espelhado deveria ter a mesma inclinacao"
    assert min(med[1][k] for k in ("frente", "cima", "lado_dir")) > 0.75, "o clipe puro espelhado deveria coincidir"

    troca = G.carregar(amostras / "lev_troca_media__0.3.npz")
    col = troca.meta["colunas_episodios"]
    fim = min(int(r[col.index("passo_fim")]) for r in troca.episodios if int(r[col.index("mundo")]) == 2)
    jt = janela(troca, 2)
    salto = np.linalg.norm(np.diff(troca.corpo[2, np.round(jt.t / troca.dt).astype(int), 0, :3], axis=0), axis=-1).max()
    print(f"lev_troca_media 0.3 mundo 2: o episodio cai e recomeca no passo {fim}; janela {jt.t[0]:.2f}-{jt.t[-1]:.2f} s, "
          f"maior salto da pelve {salto:.2f} m")
    assert round(jt.t[-1] / troca.dt) < fim and salto < 0.3, "a janela nao deveria atravessar o recomeco"

    lev = irmas(amostras / "lev_levantar__atual.npz")
    jl = [janela(G.carregar(p)) for p in lev]
    assert all(j.t[0] == 0.0 and j.juntas["pelve"][:5, 2].max() < 0.4 for j in jl), "o levantar deveria comecar no chao"
    q = _poses(jl[0], 12)
    print(f"lev_levantar (rede atual): sem passada ({jl[0].passada}); folha pelo movimento em {np.round(jl[0].t[q], 2).tolist()} s")
    assert jl[0].passada is None and (jl[0].t[q] < 1.5).sum() >= 5, "a folha do levantar deveria se concentrar no levantar"
    feitos.append(salvar(fantasma(jl, ("frente", "lado_dir", "cima", "obliqua")), saida / "lev_levantar__fantasma_lugar.png"))
    feitos.append(salvar(folha(jl[:1], 12), saida / "lev_levantar__folha.png"))
    giro = [janela(G.carregar(p)) for p in irmas(amostras / "lev_giro__atual.npz")]
    frente = _quadro(giro[0].corpo)["frente"]
    torto = np.degrees(np.abs(np.arctan2(frente[:, 1], frente[:, 0]))).max()
    print(f"lev_giro: no lugar o corpo fica a no maximo {torto:.1f} graus da frente (o giro na janela: "
          f"{np.degrees(np.ptp(_rumo(giro[0].cap.corpo[0, np.round(giro[0].t / giro[0].cap.dt).astype(int)].astype(float)))):.0f} graus)")
    assert torto < 20.0, "no lugar o rumo suavizado deveria manter o corpo de frente durante o giro"
    feitos.append(salvar(fantasma(giro, ("frente", "lado_dir", "cima", "obliqua")), saida / "lev_giro__fantasma_lugar.png"))
    for f in feitos:
        assert f.exists() and f.stat().st_size > 30_000, f
    print(f"fantasma ok: {len(feitos)} figuras em {saida}")


def autoteste_v2(base: Path = BASE, saida: Path = SAIDA_TESTE2, malha: bool = False) -> None:
    """O v2 nas capturas da rede atual (inspecao_base_0731): o conjunto padrao do lev_corrida_0 (atual, 0.3 e clipe), o
    lev_corrida_2 x lev_corrida_6, o lev_trote_0, o lev_sprint, o lev_parado (espalhado), o lev_levantar, o lev_giro, os
    recortes de 1 s e, com `malha` (--teste --malha: o Inspector, so com o treino parado), a malha; e os fatos: o clipe
    ereto de lado no trote de frente e no sprint, a rede a 30% na lev_corrida_0 inclinada para a direita contra o clipe, o
    espalhar automatico so em quem nao anda, o detalhe do levantar no levantar, os apoios do pe esquerdo no sprint pela
    fisica (a sola quica a 1,6-2 cm) e a montagem da malha alinhada."""
    t = time.time()
    feitos = padrao(irmas(base / "lev_corrida_0__atual.npz"), saida)
    print(f"conjunto padrao do lev_corrida_0: {len(feitos)} figuras em {time.time() - t:.1f} s")
    nomes = {f.name for f in feitos}
    assert {"lev_corrida_0__fantasma_lugar.png", "lev_corrida_0__fantasma_mundo.png", "lev_corrida_0__desvio.png",
            "lev_corrida_0__folha.png", "lev_corrida_0__pegadas.png", "lev_corrida_0__parado.png",
            "lev_corrida_0__0.3__movimento.png", "lev_corrida_0__0.3__equilibrio.png"} <= nomes, nomes
    for ex in ("lev_trote_0", "lev_sprint"):
        n = _numeros(janela(G.carregar(base / f"{ex}__clipe.npz"), modo="caminho"))
        print(f"{ex} clipe puro: inclinacao de lado do corpo {n['lado']:+.2f}, do clipe {n['lado_clipe']:+.2f}")
        assert abs(n["lado"]) < 1.5 and abs(n["lado_clipe"]) < 1.5, "o clipe deveria ser ereto de lado"
    n03 = _numeros(janela(G.carregar(base / "lev_corrida_0__0.3.npz"), modo="caminho"))
    nclip = _numeros(janela(G.carregar(base / "lev_corrida_0__clipe.npz"), modo="caminho"))
    print(f"lev_corrida_0 a 30%: lado {n03['lado']:+.2f} contra o clipe {n03['lado_clipe']:+.2f} (delta {n03['delta']:+.2f}); "
          f"o clipe puro {nclip['lado']:+.2f}")
    assert n03["delta"] < -3.0 and n03["lado"] - nclip["lado"] < -3.0, "a rede a 30% deveria inclinar para a direita"
    for ex in ("lev_corrida_2", "lev_corrida_6", "lev_trote_0", "lev_sprint"):
        for modo in ("atual", "0.3"):
            jan = janela(G.carregar(base / f"{ex}__{modo}.npz"), modo="caminho")
            feitos.append(salvar(movimento(jan, referencia=True), saida / f"{jan.cap.nome}__movimento.png"))
        feitos.append(salvar(equilibrio(jan), saida / f"{jan.cap.nome}__equilibrio.png"))
    pares = [(janela(G.carregar(base / f"lev_corrida_2__{m}.npz")), janela(G.carregar(base / f"lev_corrida_6__{m}.npz")))
             for m in ("atual", "0.3", "clipe")]
    fig, med = espelho(pares)
    feitos.append(salvar(fig, saida / "lev_corrida_2__espelho.png"))
    a, b = med[2]["lado_corpo"]
    print(f"espelho lev_corrida_2 x 6: clipe {a:+.2f} x {b:+.2f}; rede atual vies {(med[0]['lado_corpo'][0] - med[0]['lado_corpo'][1]) / 2:+.2f}")
    assert abs(a - b) < 1.5, "o clipe puro espelhado deveria inclinar igual"
    jp = janela(G.carregar(base / "lev_parado__atual.npz"), modo="caminho")
    assert not _anda(jp) and _rapidez(janela(G.carregar(base / "lev_trote_0__atual.npz"), modo="caminho")) > 2.0
    feitos.append(salvar(movimento(jp, espalhar=0.8), saida / "lev_parado__atual__movimento_espalhar.png"))
    feitos.append(salvar(movimento(jp), saida / "lev_parado__atual__movimento.png"))
    jl = janela(G.carregar(base / "lev_levantar__atual.npz"), modo="caminho")
    dl = _detalhe(jl, 2.0)
    print(f"lev_levantar: o detalhe vai de {dl.t[0]:.2f} a {dl.t[-1]:.2f} s; a pelve de {dl.juntas['pelve'][0, 2]:.2f} a "
          f"{dl.juntas['pelve'][-1, 2]:.2f} m")
    assert dl.t[0] < 0.5 and dl.juntas["pelve"][0, 2] < 0.4 < dl.juntas["pelve"][-1, 2], "o detalhe do levantar e o levantar"
    feitos.append(salvar(movimento(jl, referencia=True), saida / "lev_levantar__atual__movimento.png"))
    feitos.append(salvar(parado([janela(G.carregar(p)) for p in irmas(base / "lev_levantar__atual.npz")]),
                         saida / "lev_levantar__parado.png"))
    jg = janela(G.carregar(base / "lev_giro__atual.npz"), modo="caminho")
    feitos.append(salvar(movimento(jg, referencia=True), saida / "lev_giro__atual__movimento.png"))
    feitos.append(salvar(movimento_recortes(janela(G.carregar(base / "lev_trote_0__0.3.npz"), modo="caminho"), 1.0),
                         saida / "lev_trote_0__0.3__movimento_recorte1.png"))
    feitos.append(salvar(pegadas([janela(G.carregar(p), modo="caminho") for p in irmas(base / "lev_sprint__atual.npz")],
                                 recorte=2.0), saida / "lev_sprint__pegadas_recorte2.png"))
    sp = janela(G.carregar(base / "lev_sprint__atual.npz"), modo="caminho")
    so_sola = M.contato({f"sola_{s_}": M.casca(sp.corpo, sp.geo, lado)[None, ..., 2].min(-1) for s_, lado in M.LADOS})[0][0]
    com_fisica = _apoio(sp)
    print(f"lev_sprint atual: apoios do pe esquerdo so pela sola {len(_trechos_bool(so_sola[:, 0]))}, com a fisica "
          f"{len(_trechos_bool(com_fisica[:, 0]))} (o direito {len(_trechos_bool(com_fisica[:, 1]))})")
    assert len(_trechos_bool(com_fisica[:, 0])) >= 0.7 * len(_trechos_bool(com_fisica[:, 1])), \
        "o pe esquerdo do sprint deveria apoiar quase tanto quanto o direito"
    # (A revisao de 05/11: os passos entre pousos de verdade, a margem de Hof, o detalhe da troca e do levantar.)
    ja = janela(G.carregar(base / "lev_corrida_0__atual.npz"), modo="caminho")
    pa = _pegadas_dados(ja)
    meio = 2.0 * np.mean([g["passo"][2] for g in pa if g["passo"]])
    print(f"lev_corrida_0 atual: {sum(g['quiques'] for g in pa)} quiques; a passada pelos passos {meio:.2f} s contra "
          f"{ja.passada:.2f} s pela autocorrelacao")
    assert abs(meio - ja.passada) < 0.1 * ja.passada, "o passo deveria ir de pouso a pouso de verdade (sem os quiques)"
    for modo in ("clipe", "0.3"):
        jc = janela(G.carregar(base / f"lev_corrida_0__{modo}.npz"), modo="caminho")
        m = _equilibrio_dados(jc.corpo, jc.geo, jc.cap.dt, _fisica(jc))["margem"]
        mr = _equilibrio_dados(jc.ref, jc.geo, jc.cap.dt)["margem"]
        print(f"lev_corrida_0 {modo}: margem de Hof media {np.nanmean(m) * 100:+.1f} cm, alem da borda "
              f"{_fracao(m < 0, m):.0f}% (o alvo do clipe {np.nanmean(mr) * 100:+.1f} cm, {_fracao(mr < 0, mr):.0f}%)")
        assert _fracao(mr < 0, mr) < 15.0, "o alvo do clipe correndo deveria ter margem de Hof quase sempre"
    jt = _no_rumo(_detalhe(janela(G.carregar(base / "lev_troca_forte__0.3.npz"), modo="caminho"), 2.0))
    jr_ = _no_rumo(_detalhe(ja, 2.0))
    print(f"lev_troca_forte 0.3: detalhe {jt.t[0]:.2f}-{jt.t[-1]:.2f} s com {len(_saltos(jt))} salto(s) do pedido; lateral "
          f"espalhada: troca {_dobra(jt, _quadros_fantasma(jt))}, corrida reta {_dobra(jr_, _quadros_fantasma(jr_))}")
    assert len(_saltos(jt)) and _dobra(jt, _quadros_fantasma(jt)), "o detalhe da troca deveria pegar o salto e abrir a lateral"
    assert not _dobra(jr_, _quadros_fantasma(jr_)), "a corrida reta nao deveria ser espalhada"
    cab6 = _cabecalho(janela(G.carregar(base / "lev_corrida_6__0.3.npz"), modo="caminho"))
    print(f"cabecalho: {cab6}")
    assert "DESEQUILÍBRIO" in cab6, "o cabecalho deveria dizer que o episodio acaba por desequilibrio na janela"
    feitos.append(salvar(equilibrio(jl), saida / "lev_levantar__atual__equilibrio.png"))
    feitos.append(salvar(movimento(janela(G.carregar(base / "lev_troca_forte__0.3.npz"), modo="caminho"), referencia=True),
                         saida / "lev_troca_forte__0.3__movimento.png"))
    if malha:
        t = time.time()
        jm = janela(G.carregar(base / "lev_corrida_0__0.3.npz"), modo="caminho")
        feitos.append(salvar(movimento(jm, referencia=True, malha=True), saida / "lev_corrida_0__0.3__movimento_malha.png"))
        print(f"malha: {time.time() - t:.1f} s")
    for f in feitos:
        assert f.exists() and f.stat().st_size > 50_000, f
    print(f"fantasma v2 ok: {len(feitos)} figuras em {saida}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capturas", nargs="*", help="as capturas .npz (inspecao.captura)")
    ap.add_argument("--vistas", nargs="+", choices=list(VISTAS))
    ap.add_argument("--modo", choices=("lugar", "mundo"), default="lugar")
    ap.add_argument("--janela", nargs=2, type=float, metavar=("T0", "T1"), help="segundos da captura")
    ap.add_argument("--mundo", type=int, default=0)
    ap.add_argument("--desvio", action="store_true", help="cada elo pela distancia ao clipe")
    ap.add_argument("--folha", type=int, metavar="N", help="a folha de N poses em vez do fantasma")
    ap.add_argument("--espelho", metavar="OUTRO.npz", help="a captura do exercicio espelhado (refletida por cima)")
    ap.add_argument("--grade", action="store_true", help="cada captura com as irmas da pasta nas linhas")
    ap.add_argument("--padrao", action="store_true", help="o conjunto padrao do exercicio na pasta --saida")
    ap.add_argument("--saida", help="o .png (ou a pasta); padrao: <pasta da primeira captura>/fantasma")
    ap.add_argument("--teste", action="store_true", help="o autoteste nas amostras (v1) e nas capturas da rede atual (v2)")
    v2 = ap.add_argument_group("v2: o movimento")
    v2.add_argument("--movimento", action="store_true", help="o movimento: os fantasmas ao longo do caminho de verdade")
    v2.add_argument("--parado", action="store_true", help="no lugar: poses discretas numeradas (as capturas nas linhas)")
    v2.add_argument("--pegadas", action="store_true", help="as pegadas numeradas de cima (as capturas nas linhas)")
    v2.add_argument("--equilibrio", action="store_true", help="a base, o centro de massa, o ponto de captura e as fases")
    v2.add_argument("--recorte", type=float, metavar="S", help="recortes de S s: o movimento em cartoes, as pegadas e o "
                    "equilibrio em linhas, o parado com uma linha por recorte")
    v2.add_argument("--referencia", action="store_true", help="o clipe em contorno sobre cada fantasma")
    v2.add_argument("--espalhar", type=float, metavar="M_S", help="abre no tempo o que quase nao anda, a M_S m/s (padrao: "
                    "automatico, sem sobrepor, quando a pelve anda menos de 0,5 m/s; 0 desliga)")
    v2.add_argument("--passo-fantasma", type=float, metavar="S", help="o intervalo entre os fantasmas (padrao: ~0,65 m de "
                    "caminho, entre 0,1 e 0,25 s)")
    v2.add_argument("--destaque", type=float, metavar="T", help="o instante (s) da pose clara com o esqueleto (padrao: o "
                    "ultimo)")
    v2.add_argument("--malha", action="store_true", help="o movimento com o jogador de verdade (MatterModelInspector; o "
                    "mundo 0)")
    a = ap.parse_args()
    if a.teste:
        if AMOSTRAS.exists() or a.capturas:
            autoteste(Path(a.capturas[0]) if a.capturas else AMOSTRAS, Path(a.saida) if a.saida else SAIDA_TESTE)
        autoteste_v2(malha=a.malha)
        return
    if not a.capturas:
        ap.error("faltam as capturas")
    caminhos = [Path(c) for c in a.capturas]
    if a.grade:
        caminhos = list(dict.fromkeys(q for c in caminhos for q in irmas(c)))
    saida = Path(a.saida) if a.saida else caminhos[0].parent / "fantasma"
    if a.padrao:
        grupos: dict = {}
        for c in caminhos:
            grupos.setdefault(c.stem.split("__")[0], []).append(c)
        for grupo in grupos.values():
            padrao(sorted(grupo, key=lambda q: _ordem_modo(q.stem.split("__")[1])), saida, a.mundo)
        return
    intervalo = tuple(a.janela) if a.janela else None
    if a.movimento or a.parado or a.pegadas or a.equilibrio:
        _main_v2(a, caminhos, saida, intervalo)
        return
    # (A folha e o espelho comparam poses: no mundo os estroboscopios de duas capturas nao se alinham.)
    jans = [janela(G.carregar(c), a.mundo, intervalo, "lugar" if a.folha or a.espelho else a.modo) for c in caminhos]
    nome = jans[0].cap.nome if len(jans) == 1 else f"{jans[0].cap.meta['exercicio']}__grade"
    if a.espelho:
        # (Com --grade, um par por modo; sem, cada captura contra a dada.)
        por_modo = {o.stem.split("__")[1]: o for o in irmas(a.espelho)} if a.grade else {}
        pares = [(j, janela(G.carregar(por_modo.get(j.cap.meta["modo"], a.espelho)), a.mundo, intervalo)) for j in jans
                 if not a.grade or j.cap.meta["modo"] in por_modo]
        if not pares:
            ap.error(f"sem captura de {a.espelho} nos modos das capturas dadas")
        fig = espelho(pares, a.vistas or ("frente", "cima", "lado_dir"))[0]
        sufixo = "espelho"
    elif a.folha:
        fig, sufixo = folha(jans, a.folha, a.vistas or ("frente", "lado_dir"), a.desvio), "folha"
    else:
        vistas = a.vistas or (VISTAS_LUGAR if a.modo == "lugar" else vistas_mundo(jans[0]))
        fig, sufixo = fantasma(jans, vistas, a.desvio), f"fantasma_{a.modo}" + ("_desvio" if a.desvio else "")
    salvar(fig, saida if saida.suffix == ".png" else saida / f"{nome}__{sufixo}.png")



def _main_v2(a, caminhos: list[Path], saida: Path, intervalo) -> None:
    """Os produtos do v2 pedidos na linha de comando, um .png cada (o --saida .png so quando sai um so)."""
    caps = [G.carregar(c) for c in caminhos]
    n = len(caps) * (a.movimento + a.equilibrio) + a.pegadas + a.parado
    pasta = saida.parent if saida.suffix == ".png" else saida
    nome = caps[0].nome if len(caps) == 1 else f"{caps[0].meta['exercicio']}__grade"
    detalhe = None if intervalo else 2.0

    def destino(base: str, sufixo: str) -> Path:
        return saida if saida.suffix == ".png" and n == 1 else pasta / f"{base}__{sufixo}.png"

    def caminho(cap):
        return janela(cap, a.mundo, intervalo, "caminho")

    for cap in caps if a.movimento else []:
        if a.recorte:
            fig = movimento_recortes(caminho(cap), a.recorte, a.passo_fantasma, a.referencia, a.espalhar)
            salvar(fig, destino(cap.nome, f"movimento_recorte{_n(a.recorte, 1).replace(',0', '')}"))
        else:
            fig = movimento(caminho(cap), a.passo_fantasma, a.referencia, a.destaque, a.espalhar, a.malha, detalhe)
            salvar(fig, destino(cap.nome, "movimento" + ("_malha" if a.malha else "")))
    for cap in caps if a.equilibrio else []:
        salvar(equilibrio(caminho(cap), a.passo_fantasma, a.recorte, detalhe), destino(cap.nome, "equilibrio"))
    if a.pegadas:
        salvar(pegadas([caminho(c) for c in caps], a.recorte), destino(nome, "pegadas"))
    if a.parado:
        if a.recorte:
            # (Uma linha por recorte de cada captura, a pelve travada em cada um.)
            jans = [janela(c, a.mundo, (float(f.t[0]), float(f.t[-1] + 0.5 * c.dt))) for c in caps
                    for f in recortes(caminho(c), a.recorte)]
        else:
            jans = [janela(c, a.mundo, intervalo) for c in caps]
        salvar(parado(jans), destino(nome, "parado"))


if __name__ == "__main__":
    main()
