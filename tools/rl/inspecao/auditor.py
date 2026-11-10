"""O auditor (05/11): aponta sozinho os defeitos do movimento da rede, com a gravidade, a evidencia (a rede contra o
clipe puro, a referencia e o espelho), a causa provavel, onde agir e a figura para conferir. Le as analises da marcha
(inspecao.marcha, <captura>.marcha.json) e, se dados, o painel do treino (inspecao.painel, painel.json) e as capturas.

As bases: cada captura da rede (atual, 0.3, ...) contra o clipe do mesmo exercicio de dois jeitos - a referencia (o
clipe cinematico nos mesmos quadros, a secao "referencia" da marcha) e o clipe puro (a captura "clipe": o corpo sem a
rede, com a mao cheia). O defeito e o que sai do intervalo entre os dois, ou dos limites relativos a eles: o que o
proprio clipe faz nunca acusa, e o clipe puro auditado contra si mesmo nao acusa nada (o autoteste confere). Nos
laterais o clipe puro degenera (escorrega 25-50 cm por apoio, inclina +-8,6 graus na corrida): ali o intervalo fica
largo, e o par espelhado (espelho_vies, espelho_lados) pega o que sobra.

Gravidade: cada regra da media ou alta pelos limiares dela (na docstring de cada r_*); a mao muda um nivel (escalar:
sobe na forca menor o que cresce sem a mao, desce no atual o que some sem ela, ate baixa). A ordem: a gravidade, o peso
da regra (PESO) e o tamanho. Com --anterior, cada achado sai como novo, piorou, melhorou ou persiste, e os que sumiram
como resolvidos (ou nao medidos, sem a captura).

Regras: lado, rumo, rosto, joelho, arrasto, pe_baixo, quique, base, ritmo (e os toques repetidos), assimetria,
espelho_vies, espelho_lados, cabrito, cabeca, bracos, tronco, pes_juntos, velocidade, quedas, erro_pose, parado (o
lev_parado e o de pe do lev_levantar, este contra o clipe puro do lev_parado) e levantar (so com --capturas).
--capturas (a pasta dos .npz): o tempo de levantar, a deriva de pe e as figuras do fantasma (<capturas>/fantasma/, a
saida padrao do inspecao.fantasma).
Saida: <saida>/relatorio_auditor.md e <saida>/auditor.json.

Uso (de dentro de tools/rl):
  python -m inspecao.auditor --marcha DIR [--capturas DIR] [--painel PAINEL.json] [--anterior OLD.json] --saida DIR
  python -m inspecao.auditor --teste
"""
from __future__ import annotations

import argparse
import copy
import functools
import json
import re
import time
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from inspecao import comum
from inspecao import geometria as G
from inspecao import marcha as M

AMOSTRAS = comum.REPO / "build-profile/tmp/inspecao_amostras_0645"
SAIDA = comum.REPO / "build-profile/tmp/inspecao_dev/auditor"
GRAVIDADE = {1: "baixa", 2: "media", 3: "alta"}
GRAVIDADE_MD = {1: "baixa", 2: "média", 3: "alta"}
LADO = {"e": "esquerdo", "d": "direito"}
NOMES = {"lado": "inclinação de lado", "rumo": "desvio do rumo", "rosto": "peito fora da vista", "joelho": "joelho duro",
         "arrasto": "pé arrastando", "pe_baixo": "pé rasteiro", "quique": "pé quicando", "base": "base dos pousos",
         "ritmo": "cadência", "assimetria": "assimetria esquerda/direita", "espelho_vies": "viés comum do par espelhado",
         "espelho_lados": "par espelhado com um lado pior", "cabrito": "cabrito (pelve quicando)",
         "cabeca": "cabeça instável", "bracos": "braços", "tronco": "tronco para a frente", "pes_juntos": "pés juntos",
         "velocidade": "lento", "quedas": "quedas", "erro_pose": "longe do clipe", "parado": "de pé parado",
         "levantar": "levantar"}
EXIGENCIAS = {"semelhanca": "semelhança", "forca": "força"}  # (os nomes do painel que pedem acento)
PARTES = {"pelve": "pelve", "joelho_e": "joelho esquerdo", "joelho_d": "joelho direito", "pe_e": "pé esquerdo",
          "pe_d": "pé direito", "mao_e": "mão esquerda", "mao_d": "mão direita", "cabeca": "cabeça"}
SIMETRIA = {"passo_s": "tempo do passo (s)", "comprimento_m": "comprimento do passo (m)", "apoio_frac": "fração de apoio",
            "elevacao_cm": "altura do pé no balanço (cm)", "joelho_pico": "pico do joelho (°)",
            "joelho_amplitude": "amplitude do joelho (°)", "quadril_amplitude": "amplitude do quadril (°)",
            "tornozelo_amplitude": "amplitude do tornozelo (°)", "ombro_amplitude": "balanço do braço (°)",
            "deslize_cm": "deslize por apoio (cm)", "arrasto_frac": "fração do apoio arrastando"}
# (A ordem dos achados: a gravidade, depois o peso da regra - o vies, os pares, as quedas e os pes batendo antes das
# pernas, as pernas antes da postura e do erro de pose -, depois o tamanho: o excesso de uma regra nao se compara ao de
# outra.)
PESO = {**dict.fromkeys(("lado", "rumo", "espelho_vies", "espelho_lados", "quedas", "pes_juntos", "levantar"), 3),
        **dict.fromkeys(("joelho", "arrasto", "pe_baixo", "quique", "assimetria", "velocidade", "parado", "base",
                         "ritmo"), 2),
        **dict.fromkeys(("cabeca", "bracos", "tronco", "rosto", "cabrito", "erro_pose"), 1)}
# (As figuras de cada achado: as da marcha ao lado do .marcha.json; as do fantasma pelo nome do --padrao dele.)
FANTASMA = {"fantasma_lugar": "fantasma_lugar", "fantasma_mundo": "fantasma_mundo", "fantasma_desvio": "desvio",
            "fantasma_folha": "folha", "fantasma_espelho": "espelho"}


@dataclass
class Caso:
    """A captura auditada (r, a analise da marcha) e as bases dela: o clipe puro do mesmo exercicio (c), o exercicio
    espelhado no mesmo modo (e), o atual do mesmo exercicio quando r e uma forca menor (at), as capturas .npz da rede e
    do clipe puro (cap, cap_c; so com --capturas) e a base de pe (pe, cap_pe: no lev_levantar, o clipe puro do
    lev_parado - o de pe dele segue o mesmo alert_idle, e o clipe puro do levantar cai de pe nos 4 mundos)."""
    r: dict
    c: dict | None = None
    e: dict | None = None
    at: dict | None = None
    cap: Path | None = None
    cap_c: Path | None = None
    pe: dict | None = None
    cap_pe: Path | None = None


# --- Numeros e textos --------------------------------------------------------------------------------------------------

def _v(r: dict | None, chave: str):
    """O valor de uma chave com pontos ('global.inclinacao_lado_corpo.vies', 'lados.e.joelho_pico') numa analise."""
    for k in chave.split("."):
        r = r.get(k) if isinstance(r, dict) else None
    return r


def _ref(chave: str) -> str:
    return "referencia." + chave.removeprefix("global.")


def _tres(x: Caso, chave: str, ref: bool = True) -> tuple:
    """(rede, referencia, clipe puro) de uma chave."""
    return _v(x.r, chave), _v(x.r, _ref(chave)) if ref else None, _v(x.c, chave)


def _esp(x: Caso, chave: str):
    """A chave no exercicio espelhado, ja espelhada: o outro lado nas pernas e o sinal trocado no que tem lado."""
    if x.e is None:
        return None
    p = chave.split(".")
    if p[0] == "lados":
        return _v(x.e, ".".join(["lados", "d" if p[1] == "e" else "e"] + p[2:]))
    v = _v(x.e, chave)
    return -v if isinstance(v, (int, float)) and p[1] in M.LATERAIS else v


def _ev(x: Caso, chave: str, nome: str, ref: bool = True, k: float = 1.0) -> dict:
    """{nome: {rede, clipe puro, referencia, espelho}} de uma chave (k: a escala, 100 para fracao em %)."""
    n, rf, c = _tres(x, chave, ref)
    d = {"rede": n, "clipe puro": c}
    if ref:
        d["referência"] = rf
    if x.e is not None:
        d[f"espelho ({x.e['exercicio']})"] = _esp(x, chave)
    return {nome: {a: None if b is None else b * k for a, b in d.items()}}


def _dev(x: Caso, chave: str, *extra) -> float | None:
    """Quanto a rede sai do intervalo entre o clipe, o clipe puro (e os extras) numa chave."""
    n, rf, c = _tres(x, chave)
    return _fora(n, [rf, c, *extra])


def _fora(n, bases) -> float | None:
    """Quanto n sai do intervalo das bases (com sinal, 0 dentro); None sem n ou sem base."""
    b = [v for v in bases if v is not None]
    if n is None or not b:
        return None
    return n - max(b) if n > max(b) else n - min(b) if n < min(b) else 0.0


def _razao(n, bases) -> float | None:
    """n sobre o intervalo das bases positivas: abaixo de 1 sob a menor, acima de 1 sobre a maior, 1 dentro."""
    b = [v for v in bases if v is not None and v > 0]
    if n is None or not b:
        return None
    return n / min(b) if n < min(b) else n / max(b) if n > max(b) else 1.0


def _nivel(x, media: float, alta: float) -> int:
    return 0 if x is None else 3 if x > alta else 2 if x > media else 0


def _n(x, casas: int = 1, sinal: bool = False) -> str:
    if x is None:
        return "–"
    x = round(x, casas) + 0.0  # (sem o -0,0)
    return (f"{x:+.{casas}f}" if sinal else f"{x:.{casas}f}").replace("-", "−").replace(".", ",")


def _pc(x) -> str:
    return "–" if x is None else f"{x:.0%}"


def _para(v: float) -> str:
    return "para a esquerda" if v > 0 else "para a direita"


def _sentido(ex: str) -> int:
    """O lado do deslocamento nos rumos de lado e nas diagonais do trote e da corrida: +1 esquerda (1, 2, 3), -1 direita
    (5, 6, 7), 0 nos outros."""
    m = re.fullmatch(r"lev_(trote|corrida)_([1-35-7])", ex)
    return 0 if m is None else 1 if m.group(2) in "123" else -1


def _lateral(ex: str) -> bool:
    """Os rumos de lado e as diagonais do trote e da corrida (a direcao do clipe deles tem ~2 graus de incerteza)."""
    return _sentido(ex) != 0


def _agir_pernas(x: Caso) -> str:
    return "pernas laterais" if _lateral(x.r["exercicio"]) else "pernas"


def _a(regra, detalhe, nivel, excesso, medida, o_que, causa, agir, img, ev, valor=None) -> dict:
    """Um registro de regra: nivel 0 nao e achado, mas fica para a mao e o historico. excesso: 1 = o limiar da media."""
    return {"regra": regra, "detalhe": detalhe, "nivel": nivel, "excesso": round(max(float(excesso), 0.0), 3),
            "medida": medida, "o_que": o_que, "causa": causa, "agir": agir, "img": img, "evidencia": ev, "valor": valor}


# --- As regras de uma captura ----------------------------------------------------------------------------------------

def r_lado(x: Caso) -> list:
    """Inclinacao de lado (graus, + = esquerda) do corpo inteiro (cabeca sobre os tornozelos) e do tronco (pescoco sobre
    a pelve): o vies fora do intervalo entre o clipe e o clipe puro, o maior dos dois. Media acima de 1,5 grau, alta
    acima de 3. Nos rumos de lado e nas diagonais ela e dita contra o movimento ou para o lado dele: ali o par espelhado
    inclinando cada um contra o proprio movimento nao e vies da rede (a parte comum, espelho_vies, e)."""
    d = {nome: _dev(x, f"global.inclinacao_lado_{nome}.vies") for nome in ("corpo", "tronco")}
    d = {k: v for k, v in d.items() if v is not None}
    if not d:
        return []
    nome = max(d, key=lambda k: abs(d[k]))
    v = d[nome]
    ev = (_ev(x, "global.inclinacao_lado_corpo.vies", "corpo (°)") | _ev(x, "global.inclinacao_lado_tronco.vies", "tronco (°)")
          | _ev(x, "global.cm_lateral_cm.vies", "centro de massa sobre a base (cm)"))
    o_que = f"o {nome} inclina {_n(abs(v))}° {_para(v)} além do clipe"
    causa, agir = "viés lateral da rede: o clipe e o clipe puro não inclinam assim", "viés lateral da rede"
    s = _sentido(x.r["exercicio"])
    if s and v:
        contra = np.sign(v) != s
        o_que += f", {'contra o' if contra else 'para o lado do'} movimento (vai {_para(s)})"
        causa = (("o corpo fica para trás do deslocamento de lado, como quem freia" if contra else
                  "o corpo se joga para o lado do deslocamento") + "; o viés da rede é a parte comum do par (espelho_vies)")
        agir = "pernas laterais"
    return [_a("lado", "", _nivel(abs(v), 1.5, 3.0), abs(v) / 1.5, f"{_n(v, 1, True)}°", o_que, causa, agir,
               ("series", "fantasma_lugar"), ev, v)]


def r_rumo(x: Caso) -> list:
    """Desvio do rumo do deslocamento (graus, + = esquerda; so com pedido): fora do intervalo entre o pedido (0), o clipe
    e o clipe puro. Media acima de 2 graus, alta acima de 4; +2 nos dois nos rumos de lado e nas diagonais (a direcao
    do clipe de lado tem ~2 graus de incerteza, ver marcha)."""
    v = _dev(x, "global.desvio_rumo_graus", 0.0)
    if v is None:
        return []
    folga = 2.0 if _lateral(x.r["exercicio"]) else 0.0
    return [_a("rumo", "", _nivel(abs(v), 2 + folga, 4 + folga), abs(v) / (2 + folga), f"{_n(v, 1, True)}°",
               f"o deslocamento desvia {_n(abs(v))}° {_para(v)} do rumo pedido além do clipe",
               f"deriva: a rede escorrega {_para(v)}" + (" (rumo de lado: ±2° de incerteza no clipe)" if folga else ""),
               "viés lateral da rede", ("pegadas", "series", "fantasma_mundo"),
               _ev(x, "global.desvio_rumo_graus", "desvio do rumo (°)")
               | {"desvio no treino (°)": {"rede": _v(x.r, "episodios.desvio_rumo"),
                                           "clipe puro": _v(x.c, "episodios.desvio_rumo")}},
               v)]


def r_rosto(x: Caso) -> list:
    """O peito contra o rumo pedido (graus, + = esquerda; a regra dele: o peito sempre para a vista): o vies fora do
    intervalo entre o clipe e o clipe puro (os dois trazidos para perto do da rede, o angulo da a volta em +-180). Media
    acima de 5 graus, alta acima de 10."""
    n, rf, c = _tres(x, "global.rosto_rumo_graus")
    if n is None:
        return []
    v = _fora(n, [None if b is None else n + (b - n + 180.0) % 360.0 - 180.0 for b in (rf, c)])
    if v is None:
        return []
    return [_a("rosto", "", _nivel(abs(v), 5.0, 10.0), abs(v) / 5.0, f"{_n(v, 1, True)}°",
               f"o peito gira {_n(abs(v))}° {_para(v)} além do clipe", "o corpo não fica de frente para a vista como o clipe",
               "tronco", ("fantasma_lugar", "series"), _ev(x, "global.rosto_rumo_graus", "peito contra o rumo pedido (°)"))]


def r_joelho(x: Caso) -> list:
    """Joelho duro: o pico da flexao na passada ou a amplitude dela (p95 - p5) abaixo de 75% do menor entre o clipe e o
    clipe puro (alta abaixo de 60%), por perna; so onde o joelho do clipe dobra 20 graus ou mais."""
    out = []
    for s in "ed":
        if min([v for v in _tres(x, f"lados.{s}.joelho_amplitude")[1:] if v is not None], default=0.0) < 20.0:
            continue
        q = {}
        for m in ("pico", "amplitude"):
            n, rf, c = _tres(x, f"lados.{s}.joelho_{m}")
            r = _razao(n, [rf, c])
            if r is not None:
                q[m] = (min(r, 1.0), n, min(v for v in (rf, c) if v is not None))
        if not q:
            continue
        m = min(q, key=lambda k: q[k][0])
        r, n, b = q[m]
        out.append(_a("joelho", s, 3 if r < 0.6 else 2 if r < 0.75 else 0, (1 - r) / 0.25, f"{r:.0%} do clipe",
                      f"joelho {LADO[s]} duro: {'pico de flexão' if m == 'pico' else 'amplitude'} {_n(n, 0)}° contra "
                      f"{_n(b, 0)}° do clipe ({r:.0%})", f"perna dura: a rede não dobra o joelho {LADO[s]} como o clipe",
                      _agir_pernas(x), ("ciclo", "fantasma_folha"),
                      _ev(x, f"lados.{s}.joelho_pico", "pico do joelho (°)")
                      | _ev(x, f"lados.{s}.joelho_amplitude", "amplitude do joelho (°)")))
    return out


def r_arrasto(x: Caso) -> list:
    """Pe arrastando: o deslize do pe no apoio (cm por apoio, o pivo da sola) acima de max(2x, +3 cm) o do clipe puro
    (alta acima de max(3x, +8 cm)), por pe; so contra o clipe puro (o pe da referencia e firme por construcao)."""
    out = []
    for s in "ed":
        n, c = _v(x.r, f"lados.{s}.deslize_cm"), _v(x.c, f"lados.{s}.deslize_cm")
        if n is None or c is None:
            continue
        lim, alta = max(2 * c, c + 3), max(3 * c, c + 8)
        frac = _v(x.r, f"lados.{s}.arrasto_frac")
        out.append(_a("arrasto", s, 3 if n > alta else 2 if n > lim else 0, (n - c) / (lim - c), f"{_n(n)} cm/apoio",
                      f"pé {LADO[s]} escorrega {_n(n)} cm por apoio (o clipe puro {_n(c)} cm)"
                      + (f"; arrasta em {_pc(frac)} do apoio" if frac is not None else ""),
                      "pé arrastando no apoio: o pivô da sola anda no chão", "pés", ("pegadas",),
                      _ev(x, f"lados.{s}.deslize_cm", "deslize por apoio (cm)", ref=False)
                      | _ev(x, f"lados.{s}.deslize_max_cm", "pior apoio (cm)", ref=False)
                      | _ev(x, f"lados.{s}.arrasto_frac", "apoio arrastando (%)", ref=False, k=100.0)))
    return out


def r_pe_baixo(x: Caso) -> list:
    """Pe rasteiro: a altura maxima da sola no balanco abaixo de 60% da menor entre o clipe e o clipe puro (alta abaixo
    de 40%), por pe; so onde o clipe levanta o pe 3 cm ou mais."""
    out = []
    for s in "ed":
        n, rf, c = _tres(x, f"lados.{s}.elevacao_cm")
        b = min([v for v in (rf, c) if v is not None], default=None)
        if n is None or b is None or b < 3.0:
            continue
        r = min(n / b, 1.0)
        out.append(_a("pe_baixo", s, 3 if r < 0.4 else 2 if r < 0.6 else 0, (1 - r) / 0.4, f"{r:.0%} do clipe",
                      f"pé {LADO[s]} baixo no balanço: sobe {_n(n)} cm contra {_n(b)} cm do clipe ({r:.0%})",
                      "passo rasteiro: o pé quase não sai do chão (risco de tropeço)", _agir_pernas(x),
                      ("ciclo", "fantasma_lugar"),
                      _ev(x, f"lados.{s}.elevacao_cm", "altura do pé no balanço (cm)")
                      | _ev(x, f"lados.{s}.folga_ponta_cm", "folga da ponta no meio do balanço (cm)")
                      | _ev(x, f"lados.{s}.tropecos", "tropeços")))
    return out


def r_quique(x: Caso) -> list:
    """Pe quicando no apoio (a sola sobe um quadro e volta, da marcha): por segundo analisado, acima do clipe e do
    clipe puro por mais de 0,5/s (alta 1,5/s), por pe."""
    def taxa(r, chave):
        t, q = _v(r, "episodios.analisado_s"), _v(r, chave)
        return q / t if q is not None and t else None

    out = []
    for s in "ed":
        k = f"lados.{s}.quiques"
        n, rf, c = taxa(x.r, k), taxa(x.r, _ref(k)), taxa(x.c, k)
        v = _fora(n, [rf, c])
        if v is None:
            continue
        altura = _v(x.r, f"lados.{s}.quique_cm")
        out.append(_a("quique", s, _nivel(v, 0.5, 1.5), v / 0.5, f"{_n(n, 2)}/s",
                      f"pé {LADO[s]} quica no apoio {_n(n, 2)} vezes por segundo (o clipe puro {_n(c, 2)})"
                      + (f", {_n(altura)} cm de altura" if altura is not None else ""),
                      "o pé pula no chão em vez de apoiar (perna rígida ou empurrão fora de hora)", _agir_pernas(x),
                      ("pegadas", "ciclo"),
                      {"quiques por segundo": {"rede": n, "clipe puro": c, "referência": rf}}
                      | _ev(x, f"lados.{s}.quique_cm", "altura do quique (cm)", ref=False)))
    return out


def r_base(x: Caso) -> list:
    """Base: a largura entre os pousos (cm, de traves ao rumo pedido) fora de +-30% do intervalo entre o clipe e o clipe
    puro (alta +-50%). Nos rumos de lado o traves ao rumo e a frente-tras do corpo: ali ela mede um pe pousando a frente
    do outro, nao a abertura das pernas."""
    n, rf, c = _tres(x, "global.largura_cm")
    r = _razao(n, [rf, c])
    if r is None:
        return []
    d = abs(r - 1.0)
    pct = _n(100 * (r - 1), 0, True)
    if _lateral(x.r["exercicio"]):
        o_que = (f"pousos {'desencontrados' if r > 1 else 'alinhados'} demais de través ao rumo (de lado, na frente-trás "
                 f"do corpo): {_n(n)} cm contra {_n(rf)} do clipe e {_n(c)} do clipe puro ({pct}%)")
        causa = ("um pé pousa à frente do outro: o passo de lado sai torto" if r > 1 else
                 "os pés pousam na mesma linha do passo de lado, sem o desencontro do clipe")
    else:
        o_que = f"base {'larga' if r > 1 else 'estreita'}: {_n(n)} cm entre os pousos contra {_n(rf)} do clipe e {_n(c)} do clipe puro ({pct}%)"
        causa = "os pés pousam abertos demais" if r > 1 else "os pés pousam fechados demais (um sob o outro)"
    return [_a("base", "", _nivel(d, 0.3, 0.5), d / 0.3, f"{pct}%", o_que, causa, _agir_pernas(x), ("pegadas",),
               _ev(x, "global.largura_cm", "largura entre os pousos de través ao rumo (cm)"))]


def r_ritmo(x: Caso) -> list:
    """Cadencia (passos por minuto) fora de +-15% do intervalo entre o clipe e o clipe puro (alta +-30%). E os toques
    repetidos (detalhe "repetidos": o mesmo pe pousando de novo antes do outro, por segundo analisado) acima do clipe e do
    clipe puro por mais de 0,5/s (alta 1,5/s): cada um encurta o passo medido e infla a cadencia (a corrida a 91% dava
    238 passos/min contra 171 com a mediana do passo igual a do clipe), entao com eles acusados a cadencia nao acusa."""
    def rep(r, chave):
        q, t = _v(r, chave), _v(r, "episodios.analisado_s")
        return q / t if q is not None and t else None

    out = []
    n, rf, c = (rep(x.r, "global.passos_repetidos"), rep(x.r, "referencia.passos_repetidos"),
                rep(x.c, "global.passos_repetidos"))
    v = _fora(n, [rf, c])
    if v is not None:
        out.append(_a("ritmo", "repetidos", _nivel(v, 0.5, 1.5), v / 0.5, f"{_n(n, 2)}/s",
                      f"o mesmo pé pousa de novo antes do outro {_n(n, 2)} vezes por segundo (o clipe puro {_n(c, 2)}): "
                      f"{_v(x.r, 'global.passos_repetidos')} toques repetidos em {_v(x.r, 'global.passos')} passos",
                      "o pé quica ou gagueja no apoio (pousa, sai e pousa de novo); a cadência medida sobe por isso",
                      _agir_pernas(x), ("pegadas", "ciclo"),
                      {"toques repetidos por segundo": {"rede": n, "clipe puro": c, "referência": rf}}
                      | _ev(x, "global.cadencia", "cadência (passos/min)")))
    inflada = v is not None and v > 0.5
    n, rf, c = _tres(x, "global.cadencia")
    r = _razao(n, [rf, c])
    if r is None:
        return out
    d = abs(r - 1.0)
    return out + [_a("ritmo", "", 0 if inflada else _nivel(d, 0.15, 0.3), 0.0 if inflada else d / 0.15,
                     f"{_n(100 * (r - 1), 0, True)}%",
                     f"cadência {_n(n, 0)} passos/min contra {_n(rf, 0)} do clipe e {_n(c, 0)} do clipe puro "
                     f"({_n(100 * (r - 1), 0, True)}%)" + (" — inflada pelos toques repetidos" if inflada else ""),
                     "passos curtos e rápidos demais" if r > 1 else "passos lentos demais", _agir_pernas(x),
                     ("ciclo", "pegadas"),
                     _ev(x, "global.cadencia", "cadência (passos/min)") | _ev(x, "global.comprimento_m", "comprimento do passo (m)"))]


def r_assimetria(x: Caso) -> list:
    """Assimetria esquerda/direita na marcha (o clipe da passos): o SI (2|E-D|/(|E|+|D|), o da marcha) acima de 0,15
    (alta 0,30) nas medidas em que o clipe e o clipe puro sao simetricos (SI abaixo de 0,10 nos que existem) e com a
    diferenca E-D acima do piso da unidade (0,03 s, 0,10 m, 0,10 de fracao, 3 cm, 8 graus: abaixo disso o SI de valores
    pequenos e ruido). Um registro por captura, com as medidas."""
    if not _v(x.r, "referencia.passos"):
        return []
    si = {}
    for k in SIMETRIA:
        n = _v(x.r, f"simetria.{k}")
        b = [v for v in (_v(x.r, f"referencia.simetria.{k}"), _v(x.c, f"simetria.{k}")) if v is not None]
        e, d = _v(x.r, f"lados.e.{k}"), _v(x.r, f"lados.d.{k}")
        piso = 0.03 if k.endswith("_s") else 0.10 if k.endswith(("_m", "_frac")) else 3.0 if k.endswith("_cm") else 8.0
        if n is not None and b and max(b) < 0.10 and abs(e - d) > piso:
            si[k] = n
    if not si:
        return []
    ruins = sorted((k for k in si if si[k] > 0.15), key=si.get, reverse=True)
    top = max(si.values())

    def casas(k):
        return 2 if k.endswith(("_s", "_m", "_frac")) else 1 if k.endswith("_cm") else 0

    txt = "; ".join(f"{SIMETRIA[k]} {_n(_v(x.r, f'lados.e.{k}'), casas(k))}/{_n(_v(x.r, f'lados.d.{k}'), casas(k))} "
                    f"(SI {_n(si[k], 2)})" for k in ruins)
    ev = {SIMETRIA[k]: {"rede E": _v(x.r, f"lados.e.{k}"), "rede D": _v(x.r, f"lados.d.{k}"), "SI": si[k],
                        "clipe E": _v(x.r, f"referencia.lados.e.{k}"), "clipe D": _v(x.r, f"referencia.lados.d.{k}"),
                        "clipe puro E": _v(x.c, f"lados.e.{k}"), "clipe puro D": _v(x.c, f"lados.d.{k}")} for k in ruins}
    return [_a("assimetria", "", _nivel(top, 0.15, 0.30), top / 0.15, f"SI {_n(top, 2)}",
               f"esquerda/direita diferentes em {len(ruins)} medida(s): {txt}" if ruins else "simétrica",
               "a rede usa cada perna de um jeito (o clipe é simétrico nessas medidas)", "viés lateral da rede",
               ("ciclo", "pegadas"), ev)]


def r_cabrito(x: Caso) -> list:
    """Cabrito: a pelve subindo e descendo na passada (cm, de pico a vale) acima do clipe e do clipe puro por mais de
    3 cm (alta 6 cm)."""
    n, rf, c = _tres(x, "global.pelve_quique_cm")
    v = _dev(x, "global.pelve_quique_cm")
    if v is None:
        return []
    return [_a("cabrito", "", _nivel(v, 3.0, 6.0), v / 3.0, f"{_n(n)} cm",
               f"a pelve quica {_n(n)} cm por passada contra {_n(rf)} do clipe e {_n(c)} do clipe puro",
               "cabrito: o corpo pula a cada passo", "pernas", ("series",),
               _ev(x, "global.pelve_quique_cm", "quique da pelve (cm)") | _ev(x, "global.pelve_altura_m", "altura da pelve (m)"))]


def r_cabeca(x: Caso) -> list:
    """Cabeca instavel: o giro dela (graus/s, p95) acima de 1,5x o maior entre o clipe e o clipe puro e por mais de
    25 g/s (alta: 2x e 50 g/s), ou o balanco dela sobre o pescoco (cm) acima de 2x e por mais de 0,75 cm (media)."""
    n, rf, c = _tres(x, "global.cabeca_giro_p95_gs")
    b = max([v for v in (rf, c) if v is not None], default=None)
    if n is None or b is None:
        return []
    q, dif = n / max(b, 1e-6), n - b
    nivel = 3 if q > 2 and dif > 50 else 2 if q > 1.5 and dif > 25 else 0
    exc = min((q - 1) / 0.5, dif / 25)
    giro = f"gira a {_n(n, 0)}°/s (p95) contra {_n(b, 0)} do clipe"
    np_, rp, cp = _tres(x, "global.cabeca_pescoco_cm")
    bp = max([v for v in (rp, cp) if v is not None], default=None)
    balanco = f"balança {_n(np_, 2)} cm sobre o pescoço (clipe {_n(bp, 2)})"
    medida, o_que = f"{_n(n, 0)}°/s", f"a cabeça {giro}; {balanco}"
    if np_ is not None and bp:
        exc_p = min(np_ / bp - 1, (np_ - bp) / 0.75)
        nivel = max(nivel, 2 if exc_p > 1 else 0)
        # (A medida e o texto pelo que acusa mais: o balanco sobre o pescoco ou o giro.)
        if exc_p > exc:
            medida, o_que, exc = f"{_n(np_, 2)} cm", f"a cabeça {balanco}; {giro}", exc_p
    return [_a("cabeca", "", nivel, exc, medida, o_que, "a cabeça sacode (pescoço solto ou o tronco batendo)", "cabeça",
               ("fantasma_lugar", "series"),
               _ev(x, "global.cabeca_giro_p95_gs", "giro da cabeça p95 (°/s)")
               | _ev(x, "global.cabeca_pescoco_cm", "cabeça sobre o pescoço (cm)"))]


def r_bracos(x: Caso) -> list:
    """Bracos: o balanco do braco (a amplitude do ombro, graus) fora de +-40% do intervalo entre o clipe e o clipe puro
    (alta: abaixo de 40% ou acima de 1,8x), por braco; so onde o braco do clipe balanca 8 graus ou mais."""
    out = []
    for s in "ed":
        n, rf, c = _tres(x, f"lados.{s}.ombro_amplitude")
        r = _razao(n, [rf, c])
        if r is None or min(v for v in (rf, c) if v is not None) < 8.0:
            continue
        exc = (1 - r) / 0.4 if r < 1 else (r - 1) / 0.4
        nivel = 3 if r < 0.4 or r > 1.8 else 2 if r < 0.6 or r > 1.4 else 0
        out.append(_a("bracos", s, nivel, exc, f"{r:.0%} do clipe",
                      f"braço {LADO[s]} {'quase parado' if r < 1 else 'balança demais'}: {_n(n, 0)}° contra "
                      f"{_n(rf, 0)}° do clipe e {_n(c, 0)}° do clipe puro ({r:.0%})",
                      "o braço não acompanha o clipe (trava ou abana)", "braços", ("fantasma_lugar",),
                      _ev(x, f"lados.{s}.ombro_amplitude", "balanço do braço (°)")
                      | _ev(x, f"lados.{s}.cotovelo_media", "cotovelo médio (°)")))
    return out


def r_tronco(x: Caso) -> list:
    """Tronco para a frente (graus): o vies fora do intervalo entre o clipe e o clipe puro por mais de 5 graus (alta
    10), para a frente ou para tras."""
    v = _dev(x, "global.inclinacao_frente_tronco.vies")
    if v is None:
        return []
    return [_a("tronco", "", _nivel(abs(v), 5.0, 10.0), abs(v) / 5.0, f"{_n(v, 1, True)}°",
               f"o tronco vai {_n(abs(v))}° mais {'para a frente' if v > 0 else 'para trás'} que o clipe",
               "o tronco não segue a inclinação do clipe", "tronco", ("series", "fantasma_lugar"),
               _ev(x, "global.inclinacao_frente_tronco.vies", "tronco para a frente (°)"))]


def r_pes_juntos(x: Caso) -> list:
    """Pes juntos: a menor distancia entre as cascas dos pes abaixo de min(8 cm, a menor do clipe e do clipe puro -
    2 cm) (alta abaixo de 3 cm), ou os pes trocando de lado mais que o clipe e o clipe puro por mais de 0,5 vez/s. A
    menor e de um quadro so: com o p5 no do clipe ou acima (os pes longe no resto), o quase-toque e isolado e fica
    media no limiar."""
    n, rf, c = _tres(x, "global.dist_pes_min_cm")
    b = min([v for v in (rf, c) if v is not None], default=None)
    v = _dev(x, "global.cruzamentos_por_s")
    if n is None or b is None:
        return []
    p5, b5 = _v(x.r, "global.dist_pes_p5_cm"), min([u for u in _tres(x, "global.dist_pes_p5_cm")[1:] if u is not None],
                                                    default=None)
    lim = min(8.0, b - 2.0)
    nivel = 3 if n < min(3.0, lim) else 2 if n < lim else 0
    exc = lim / max(n, 0.5) if lim > 0 else 0.0
    isolado = bool(nivel) and p5 is not None and b5 is not None and p5 >= b5
    if isolado:
        nivel, exc = 2, min(exc, 1.0)
    if v is not None and v > 0.5:
        nivel, exc = max(nivel, 2), max(exc, v / 0.5)
    return [_a("pes_juntos", "", nivel, exc, f"{_n(n)} cm",
               f"os pés chegam a {_n(n)} cm um do outro (p5 {_n(p5)}; o clipe {_n(rf)}, o clipe puro {_n(c)})"
               + (f"; quase-toque isolado: no p5 ficam tão longe quanto no clipe ({_n(b5)})" if isolado else "")
               + (f"; trocam de lado {_n(v, 2)} vez/s a mais que o clipe" if v and v > 0 else ""),
               "base estreita ou um pé cruzando por baixo do corpo: risco de chutar o outro pé", "pernas",
               ("pegadas", "fantasma_lugar"),
               _ev(x, "global.dist_pes_min_cm", "distância mínima entre os pés (cm)")
               | _ev(x, "global.dist_pes_p5_cm", "distância p5 entre os pés (cm)")
               | _ev(x, "global.cruzamentos_por_s", "pés trocando de lado (por s)"))]


def r_velocidade(x: Caso) -> list:
    """Lento: o avanco sobre o pedido abaixo de 85% do menor entre o clipe (a velocidade natural dele) e o clipe puro
    (alta abaixo de 70%)."""
    n, rf, c = _tres(x, "global.razao_velocidade")
    r = _razao(n, [rf, c])
    if r is None:
        return []
    r = min(r, 1.0)
    return [_a("velocidade", "", 3 if r < 0.7 else 2 if r < 0.85 else 0, (1 - r) / 0.15, f"{r:.0%} do clipe",
               f"anda {_n(_v(x.r, 'global.avanco_ms'), 2)} m/s ({_pc(n)} do pedido de {_n(_v(x.r, 'global.pedido_ms'), 1)}) "
               f"contra {_n(_v(x.r, 'referencia.avanco_ms'), 2)} do clipe e {_n(_v(x.c, 'global.avanco_ms'), 2)} do clipe puro",
               "a rede não sustenta a velocidade do clipe", "pernas", ("series",),
               _ev(x, "global.avanco_ms", "avanço (m/s)")
               | _ev(x, "global.razao_velocidade", "avanço sobre o pedido (%)", k=100.0))]


def r_quedas(x: Caso) -> list:
    """Quedas: a fracao dos episodios que caem acima da do clipe puro por 0,25 ou mais (alta 0,5); numa forca menor,
    tambem acima da do atual (cai sem a mao)."""
    def q(r):
        e = r and r["episodios"]
        return (e["cairam"], e["n"]) if e and e["n"] else None
    n, c, at = q(x.r), q(x.c), q(x.at)
    d = {k: n[0] / n[1] - b[0] / b[1] for k, b in (("clipe puro", c), ("atual", at)) if n and b}
    if not d:
        return []
    k = max(d, key=d.get)
    v = d[k]
    return [_a("quedas", "", 3 if v >= 0.5 - 1e-9 else 2 if v >= 0.25 - 1e-9 else 0, v / 0.25, f"{n[0]}/{n[1]}",
               f"{n[0]} de {n[1]} episódios caem (o clipe puro {c[0] if c else '–'} de {c[1] if c else '–'}"
               + (f"; o atual {at[0]} de {at[1]}" if at else "") + ")",
               "equilíbrio: cai " + ("sem a mão (o atual não cai)" if k == "atual" else "onde o clipe puro não cai"),
               "equilíbrio" + (" · régua" if k == "atual" else ""), ("series", "fantasma_mundo"),
               {"episódios que caem": {"rede": n[0] / n[1], "clipe puro": c[0] / c[1] if c else None,
                                       "atual": at[0] / at[1] if at else None}, "causas do fim": x.r["episodios"]["causas"]})]


def r_erro_pose(x: Caso) -> list:
    """Longe do clipe: o erro medio de posicao de cada parte (pelve, joelhos, pes, maos, cabeca; cm, contra a referencia)
    acima de max(1,5x, +5 cm) o do clipe puro (alta: max(2x, +10 cm)). Um registro por captura, com as partes."""
    partes = {}
    for k in PARTES:
        n, c = _v(x.r, f"erro_referencia_cm.{k}.media"), _v(x.c, f"erro_referencia_cm.{k}.media")
        if n is None or c is None:
            continue
        lim, alta = max(1.5 * c, c + 5), max(2 * c, c + 10)
        partes[k] = (3 if n > alta else 2 if n > lim else 0, (n - c) / (lim - c), n, c)
    if not partes:
        return []
    ruins = sorted((k for k in partes if partes[k][0]), key=lambda k: partes[k][1], reverse=True)
    k = max(partes, key=lambda k: partes[k][1])
    return [_a("erro_pose", "", partes[k][0], partes[k][1], f"{PARTES[k]} {_n(partes[k][2])} cm",
               "longe do clipe: " + "; ".join(f"{PARTES[p]} {_n(partes[p][2])} cm (o clipe puro {_n(partes[p][3])})"
                                              for p in ruins),
               "a rede não segue o clipe nessas partes", "nota", ("fantasma_desvio", "ciclo"),
               {f"erro médio: {PARTES[p]} (cm)": {"rede": partes[p][2], "clipe puro": partes[p][3],
                                                  "rede p90": _v(x.r, f"erro_referencia_cm.{p}.p90")} for p in partes})]


@functools.lru_cache(maxsize=None)
def _deriva(caminho: Path) -> tuple[float, str]:
    """O maior deslocamento liquido no chao (cm) da pelve ou de um pe num trecho analisado de pe, e a parte. Os trechos
    sao os da propria marcha (a _analise dela: sem a largada, o levantar do chao e as quedas), para casar com o JSON."""
    _, d = M._analise(G.carregar(caminho))
    j = d["corpo"]["j"]
    out = (0.0, "–")
    for w, a, b in d["tr"]:
        for k, nome in (("pelve", "a pelve"), ("pe_e", "o pé esquerdo"), ("pe_d", "o pé direito")):
            out = max(out, (float(np.linalg.norm(j[k][w, b - 1, :2] - j[k][w, a, :2])) * 100.0, nome), key=lambda t: t[0])
    return out


def r_parado(x: Caso) -> list:
    """De pe parado (o lev_parado e o de pe depois do levantar): os passos a cada 10 s de pe a mais que o clipe e o
    clipe puro (media acima de 1, alta acima de 3; a taxa, porque as quedas encurtam o de pe); a base (a distancia p5
    entre os pes, cm) fora de +-15% do intervalo deles (alta +-30%); com --capturas, a deriva (o maior deslocamento
    liquido da pelve ou de um pe num trecho de pe, cm) acima da do clipe puro por mais de 5 cm (alta 15). O clipe puro
    e a base de pe do caso quando ha (no lev_levantar, o do lev_parado: o do levantar cai de pe e escorrega ~50 cm)."""
    if x.r["exercicio"] not in ("lev_parado", "lev_levantar"):
        return []
    if x.pe:
        x = replace(x, c=x.pe, cap_c=x.cap_pe)
    puro = f"o clipe puro do {x.c['exercicio']}" if x.c and x.c["exercicio"] != x.r["exercicio"] else "o clipe puro"

    def por_10s(r, chave):
        p, t = _v(r, chave), _v(r, "episodios.analisado_s")
        return 10.0 * p / t if p is not None and t else None

    out = []
    n, rf, c = por_10s(x.r, "global.passos"), por_10s(x.r, "referencia.passos"), por_10s(x.c, "global.passos")
    v = _fora(n, [rf, c])
    if v is not None:
        out.append(_a("parado", "passos", _nivel(v, 1.0, 3.0), v, f"{_n(n)} passos/10 s",
                      f"dá {_v(x.r, 'global.passos')} passos em {_n(_v(x.r, 'episodios.analisado_s'), 0)} s de pé "
                      f"({_n(n)} a cada 10 s; {puro} {_n(c)})",
                      "não consegue ficar quieto: corrige o equilíbrio com passos", "equilíbrio", ("pegadas", "series"),
                      {"passos a cada 10 s de pé": {"rede": n, "clipe puro": c, "referência": rf}}
                      | _ev(x, "global.velocidade_ms", "velocidade média (m/s)")))
    n, rf, c = _tres(x, "global.dist_pes_p5_cm")
    r = _razao(n, [rf, c])
    if r is not None:
        out.append(_a("parado", "base", _nivel(abs(r - 1), 0.15, 0.3), abs(r - 1) / 0.15, f"{_n(n)} cm",
                      f"pés a {_n(n)} cm um do outro contra {_n(rf)} do clipe e {_n(c)} d{puro} "
                      f"({_n(100 * (r - 1), 0, True)}%)", "a base parada não é a do clipe", "equilíbrio",
                      ("fantasma_lugar", "series"), _ev(x, "global.dist_pes_p5_cm", "distância entre os pés p5 (cm)")
                      | {"abdução média (°)": {"rede E": _v(x.r, "lados.e.abducao_media"),
                                               "rede D": _v(x.r, "lados.d.abducao_media"),
                                               "clipe E": _v(x.r, "referencia.lados.e.abducao_media"),
                                               "clipe D": _v(x.r, "referencia.lados.d.abducao_media")}}))
    if x.cap and x.cap_c and x.cap.exists() and x.cap_c.exists():
        (dn, pn), (dc, pc) = _deriva(x.cap), _deriva(x.cap_c)
        out.append(_a("parado", "deriva", _nivel(dn - dc, 5.0, 15.0), (dn - dc) / 5.0, f"{_n(dn)} cm",
                      f"de pé, {pn} se desloca {_n(dn)} cm ({puro} {_n(dc)} cm, {pc})",
                      "o corpo escorrega ou anda devagar parado", "equilíbrio", ("pegadas", "fantasma_mundo"),
                      {"deslocamento líquido de pé (cm)": {"rede": dn, "clipe puro": dc}}))
    return out


def _tempos(caminho: Path) -> dict:
    """{mundo: de_pe_em} do primeiro episodio terminado de cada mundo (o tempo ate ficar de pe do treino; < 0 = nao
    levantou)."""
    cap = G.carregar(caminho)
    cols = cap.meta["colunas_episodios"]
    out = {}
    for row in sorted(cap.episodios, key=lambda r: r[cols.index("passo_fim")]):
        out.setdefault(int(row[cols.index("mundo")]), float(row[cols.index("de_pe_em")]))
    return out


def r_levantar(x: Caso) -> list:
    """Levantar (o lev_levantar, com --capturas): o tempo ate ficar de pe de cada mundo contra o do clipe puro no mesmo
    mundo (o mesmo clipe de levantar): media acima de +0,5 s, alta acima de +1 s ou sem levantar onde o clipe levanta."""
    if x.r["exercicio"] != "lev_levantar" or not (x.cap and x.cap_c and x.cap.exists() and x.cap_c.exists()):
        return []
    n, c = _tempos(x.cap), _tempos(x.cap_c)
    # (Nao levantar onde o clipe levanta conta como 2 s de atraso: alta.)
    d = {w: 2.0 if n[w] < 0 else n[w] - c[w] for w in n if w in c and c[w] >= 0}
    if not d:
        return []
    w = max(d, key=d.get)
    return [_a("levantar", "", _nivel(d[w], 0.5, 1.0), d[w] / 0.5,
               "não levanta" if n[w] < 0 else f"{_n(d[w], 2, True)} s",
               f"não levanta no mundo {w} (o clipe puro em {_n(c[w], 2)} s)" if n[w] < 0 else
               f"levanta até {_n(d[w], 2)} s depois do clipe puro (mundo {w}: {_n(n[w], 2)} contra {_n(c[w], 2)} s)",
               "o levantar demora mais que o do clipe", "levantar", ("series", "fantasma_folha"),
               {f"de pé em, mundo {k} (s)": {"rede": n.get(k), "clipe puro": c.get(k)} for k in sorted(n)})]


REGRAS = (r_lado, r_rumo, r_rosto, r_joelho, r_arrasto, r_pe_baixo, r_quique, r_base, r_ritmo, r_assimetria, r_cabrito,
          r_cabeca, r_bracos, r_tronco, r_pes_juntos, r_velocidade, r_quedas, r_erro_pose, r_parado, r_levantar)


# --- Os pares espelhados -----------------------------------------------------------------------------------------------

def _exf(r: dict) -> str:
    """O exercicio e a forca da captura (os dois de um par podem estar em forcas diferentes no atual)."""
    return f"{r['exercicio']} ({_pc(r['forca'])})"


def _comum(a: dict | None, b: dict | None, chave: str) -> float | None:
    """(A + B)/2 de uma chave num par espelhado: a parte que nao troca de sinal com o sentido."""
    va, vb = _v(a, chave), _v(b, chave)
    return None if va is None or vb is None else (va + vb) / 2


def r_espelho_vies(a: dict, b: dict, ca: dict | None, cb: dict | None) -> list:
    """O vies comum de um par espelhado (o da esquerda a, o da direita b) no mesmo modo: (A + B)/2 da inclinacao de
    lado e do desvio de rumo - a parte que nao troca de sinal com o sentido (+ = esquerda). Com os clipes espelhados ela
    e zero; o que sobra e o vies da rede, fora do intervalo entre o mesmo dos clipes (a referencia), o dos clipes puros
    e, no rumo, o pedido (0). Inclinacao: media acima de 1,5 grau, alta acima de 3; rumo: 2 e 4."""
    out = []
    for det, k, med, alta in (("lado", "global.inclinacao_lado_corpo.vies", 1.5, 3.0),
                              ("rumo", "global.desvio_rumo_graus", 2.0, 4.0)):
        n, rf, c = _comum(a, b, k), _comum(a, b, _ref(k)), _comum(ca, cb, k)
        v = _fora(n, [rf, c] + ([0.0] if det == "rumo" else []))
        if v is None:
            continue
        nome = "inclinam" if det == "lado" else "desviam o rumo"
        out.append(_a("espelho_vies", det, _nivel(abs(v), med, alta), abs(v) / med, f"{_n(v, 1, True)}°",
                      f"{_exf(a)} e {_exf(b)} {nome} juntos {_n(abs(v))}° {_para(v)} (a parte que não troca "
                      f"com o sentido; {_n(_v(a, k), 1, True)}° e {_n(_v(b, k), 1, True)}°)",
                      "viés lateral da rede: com os clipes espelhados essa parte é zero", "viés lateral da rede",
                      ("fantasma_espelho", "series"),
                      {("inclinação de lado (°)" if det == "lado" else "desvio do rumo (°)"):
                       {f"{a['exercicio']}": _v(a, k), f"{b['exercicio']}": _v(b, k), "comum": n, "clipe comum": rf,
                        "clipe puro comum": c}}, v))
    return out


def _perna(r: dict | None, lado: str, m: str) -> float | None:
    """A medida de uma perna para o espelho: no pico do joelho e na altura do pe, o que falta contra o proprio clipe
    (1 - rede/clipe, 0 se passa dele); os quiques por segundo analisado; o deslize por apoio como esta."""
    v = _v(r, f"lados.{lado}.{m}")
    if v is None:
        return None
    if m in ("joelho_pico", "elevacao_cm"):
        rf = _v(r, f"referencia.lados.{lado}.{m}")
        return max(0.0, 1.0 - v / rf) if rf else None
    t = _v(r, "episodios.analisado_s")
    return v if m == "deslize_cm" else v / t if t else None


# (As medidas do espelho das pernas: o nome, os limiares de pior e muito pior e como mostrar o valor da rede.)
PERNAS = (("joelho_pico", "pico do joelho", 0.15, 0.30, "°", 0), ("elevacao_cm", "altura do pé no balanço", 0.15, 0.30,
          " cm", 1), ("deslize_cm", "deslize por apoio do pé", 5.0, 15.0, " cm", 1),
          ("quiques", "quiques do pé", 0.5, 1.5, "/s", 2))


def r_espelho_lados(a: dict, b: dict, ca: dict | None, cb: dict | None) -> list:
    """Cada perna do exercicio da direita (b) contra a espelhada no da esquerda (a), no mesmo modo: o que falta ao pico
    do joelho e a altura do pe no balanco contra o proprio clipe (1 - rede/clipe; pior acima de 0,15 de diferenca, muito
    pior acima de 0,30), o deslize por apoio (cm; 5 e 15) e os quiques por segundo (0,5 e 1,5); e o exercicio inteiro
    pelo avanco sobre o pedido (o da direita mais lento: 0,10 e 0,20 do pedido). A diferenca B - A conta so fora do
    intervalo entre 0 (os clipes sao espelhados) e a mesma do par de clipes puros. Cada sinal vale 1 (pior) ou 2 (muito
    pior) para o lado pior; os pontos do lado com mais dao a gravidade (1 baixa, 2 media, 3 ou mais alta) e o saldo
    (D - E) o veredito ("D pior": o da direita pior; "misto" no empate com pontos dos dois lados)."""
    sinais = []
    va, vb = _v(a, "global.razao_velocidade"), _v(b, "global.razao_velocidade")
    if va is not None and vb is not None:
        cva, cvb = _v(ca, "global.razao_velocidade"), _v(cb, "global.razao_velocidade")
        d = _fora(va - vb, [0.0, cva - cvb if cva is not None and cvb is not None else None])
        sinais.append((d, 0.10, 0.20, f"avanço: {_n(_v(b, 'global.avanco_ms'), 2)} m/s ({_pc(vb)} do pedido) no "
                                      f"{b['exercicio']}, {_n(_v(a, 'global.avanco_ms'), 2)} m/s ({_pc(va)}) no {a['exercicio']}"))
    for s, sa in (("d", "e"), ("e", "d")):
        for m, nome, med, alta, u, casas in PERNAS:
            vb, va, cvb, cva = _perna(b, s, m), _perna(a, sa, m), _perna(cb, s, m), _perna(ca, sa, m)
            if vb is None or va is None:
                continue
            d = _fora(vb - va, [0.0, cvb - cva if cvb is not None and cva is not None else None])
            if m == "quiques":
                tb, ta = f"{_n(vb, casas)}{u}", f"{_n(va, casas)}{u}"
            else:
                nb, na = _v(b, f"lados.{s}.{m}"), _v(a, f"lados.{sa}.{m}")
                rb, ra = _v(b, f"referencia.lados.{s}.{m}"), _v(a, f"referencia.lados.{sa}.{m}")
                tb = f"{_n(nb, casas)}{u}" + (f" ({nb / rb:.0%} do clipe)" if rb else "")
                ta = f"{_n(na, casas)}{u}" + (f" ({na / ra:.0%})" if ra else "")
            sinais.append((d, med, alta, f"{nome}: {LADO[s]} {tb} no {b['exercicio']}, {LADO[sa]} {ta} no {a['exercicio']}"))
    pontos = [(2 if abs(d) > alta else 1 if abs(d) > med else 0) * (1 if d > 0 else -1) for d, med, alta, _ in sinais]
    # (Os pontos de lados opostos nao se anulam na gravidade - o lado com mais da a gravidade; o empate e misto.)
    pos, neg = sum(p for p in pontos if p > 0), -sum(p for p in pontos if p < 0)
    saldo = pos - neg
    pior_d = [t for p, (_, _, _, t) in zip(pontos, sinais) if p > 0]
    pior_e = [t for p, (_, _, _, t) in zip(pontos, sinais) if p < 0]
    veredito = "D pior" if saldo > 0 else "E pior" if saldo < 0 else "misto" if pos else "iguais"
    if veredito == "misto":
        texto = f"misto: {_exf(b)} pior em {'; '.join(pior_d)} — {_exf(a)} pior em {'; '.join(pior_e)}"
    elif saldo:
        (pior, outro), (ps, os_) = ((b, a), (pior_d, pior_e)) if saldo > 0 else ((a, b), (pior_e, pior_d))
        texto = (f"{_exf(pior)} pior que o espelho {_exf(outro)}: " + "; ".join(ps)
                 + (f" (o {outro['exercicio']} pior em: {'; '.join(os_)})" if os_ else ""))
    else:
        texto = "as pernas espelhadas fazem o mesmo"
    return [_a("espelho_lados", "", min(3, max(pos, neg)), max(pos, neg) / 2,
               f"saldo {saldo:+d}" + (f" ({pos}×{neg})" if pos and neg else ""), texto,
               "viés lateral da rede: as pernas de um lado não fazem o espelho das do outro", "pernas laterais",
               ("fantasma_espelho", "ciclo"),
               {t: {"B − A (+ = o da direita pior)": d} for d, _, _, t in sinais} | {"veredito": veredito}, saldo)]


# --- A auditoria ---------------------------------------------------------------------------------------------------------

def carregar(pasta: Path) -> dict:
    """As analises da marcha da pasta, por captura."""
    rs = [json.loads(p.read_text()) for p in sorted(Path(pasta).glob("*.marcha.json"))]
    return {r["captura"]: r for r in rs}


def _por_exercicio(rs: dict) -> dict:
    out: dict = {}
    for r in rs.values():
        out.setdefault(r["exercicio"], {})[r["modo"]] = r
    return out


def _modos(modos: dict) -> list:
    """Os modos da rede (sem o clipe puro): o atual e as forcas da maior para a menor."""
    return sorted((m for m in modos if m != "clipe"), key=lambda m: (m != "atual", -float(m) if m != "atual" else 0.0))


def regras_caso(x: Caso) -> list:
    return [a for regra in REGRAS for a in regra(x)]


def regras_par(a: dict, b: dict, ca: dict | None, cb: dict | None) -> list:
    return r_espelho_vies(a, b, ca, cb) + r_espelho_lados(a, b, ca, cb)


def _figuras(a: dict, marcha_dir: Path, capturas: Path | None) -> list:
    """As figuras do achado que existem: as da marcha da captura e as do fantasma do exercicio."""
    out = []
    for k in a["img"]:
        if k in FANTASMA:
            if capturas:
                out += [capturas / "fantasma" / f"{ex}__{FANTASMA[k]}.png" for ex in a["exercicios"]]
        else:
            out.append(marcha_dir / f"{a['captura']}.{k}.png")
    return [str(p) for p in dict.fromkeys(out) if p.exists()]


def escalar(regs: list) -> None:
    """A mao, entre o atual e cada forca menor da mesma regra: o defeito que cresce 1,5x sem a mao sobe um nivel na
    forca menor quando ja passa 1,5x do limiar dela (a mao esconde; perto do limiar so a nota); o do atual que some na
    forca menor (nivel 0 la, 1,5x menor) desce um nivel no atual, ate baixa (vem da mao). As quedas nao: a regra delas
    ja compara com o atual."""
    grupos: dict = {}
    for a in regs:
        grupos.setdefault((a["regra"], a["exercicio"], a["detalhe"]), {})[a["modo"]] = a
    for g in grupos.values():
        at = g.get("atual")
        if at is None or at["regra"] == "quedas":
            continue
        for modo, b in g.items():
            if modo == "atual":
                continue
            txt = f"atual ({_pc(at['forca'])}) {at['medida']} → {_pc(b['forca'])} {b['medida']}"
            if b["nivel"] and b["excesso"] >= 1.5 * at["excesso"]:
                if b["excesso"] >= 1.5:
                    b["nivel"] = min(3, b["nivel"] + 1)
                b["mao"] = f"a mão esconde: {txt}"
                if at["nivel"]:
                    at["mao"] = f"cresce sem a mão: {txt}"
            elif at["nivel"] and not b["nivel"] and at["excesso"] >= 1.5 * b["excesso"] and "mao" not in at:
                at["nivel"] = max(1, at["nivel"] - 1)
                at["mao"] = f"vem da mão: {txt}"


def _padrao_lado(regs: list) -> None:
    """Na causa de cada inclinacao, os outros exercicios do mesmo modo que inclinam para o mesmo lado (sem os rumos de
    lado: neles o lado do mundo nao e o vies, ver r_lado)."""
    lado = [a for a in regs if a["regra"] == "lado" and a["nivel"] and not _lateral(a["exercicio"])]
    for a in lado:
        outros = [b["exercicio"] for b in lado if b is not a and b["modo"] == a["modo"]
                  and np.sign(b["valor"]) == np.sign(a["valor"])]
        if outros:
            a["causa"] += f"; o mesmo {_para(a['valor'])} em {', '.join(outros)} (mesmo modo)"


def _treino(painel: dict | None, a: dict) -> str | None:
    """O que o painel (o log do treino) diz dos exercicios do achado: a forca, o que segura a regua, o par e o vies."""
    if not painel:
        return None
    out = []
    for ex in a["exercicios"]:
        e = painel.get("exercicios", {}).get(ex)
        if e:
            seg = ", ".join(f"{EXIGENCIAS.get(b['exigencia'], b['exigencia'])} {_n(b['valor'], 3)} < {_n(b['exigido'], 3)}"
                            for b in e.get("bloqueios") or [])
            out.append(f"{ex} na força {_pc(e['forca'])} ({e.get('estado')}" + (f": {seg}" if seg else "") + ")")
    if a["regra"] == "espelho_lados":
        for p in painel.get("pares", []):
            if [p["esquerda"], p["direita"]] == a["exercicios"]:
                dif = [f"{EXIGENCIAS.get(m, m)} {_n(v['E'], 3)} → {_n(v['D'], 3)}" for m, v in p["medidas"].items()
                       if v["nivel"]]
                out.append(f"o par no log: {p['veredito']}" + (f" ({', '.join(dif)})" if dif else ""))
    faixa = re.match(r"lev_(trote|corrida)_", a["exercicios"][0])
    v = faixa and painel.get("vies_rumo", {}).get(faixa.group(1))
    if v and a["regra"] in ("rumo", "lado", "espelho_vies"):
        out.append(f"viés de rumo no log ({faixa.group(1)}, + = esquerda): giro {_n(v['giro'], 2, True)}°, deriva "
                   f"{_n(v['deriva'], 2, True)}°")
    # (O log pode ser de outra atualizacao que a das capturas: a forca de la nao e a do "atual" daqui.)
    return f"r{painel.get('atualizacao')}: " + "; ".join(out) if out else None


def auditar(rs: dict, marcha_dir: Path, capturas: Path | None = None, painel: dict | None = None) -> dict:
    """Todos os registros (os achados sao os de nivel > 0) das capturas da rede e dos pares espelhados, com a mao, as
    figuras e o painel."""
    por = _por_exercicio(rs)
    regs = []

    def junta(lista, r, exs, modo):
        for a in lista:
            a.update(exercicio=exs[0] if len(exs) == 1 else f"{exs[0]}×{exs[1].rsplit('_', 1)[1]}",
                     exercicios=exs, modo=modo, forca=r["forca"], captura=r["captura"], atualizacao=r.get("atualizacao"))
            regs.append(a)

    def npz(r):
        return capturas / f"{r['captura']}.npz" if capturas and r else None

    for ex, modos in sorted(por.items()):
        c, par = modos.get("clipe"), comum.mirror_of(ex)
        pe = por.get("lev_parado", {}).get("clipe") if ex == "lev_levantar" else None
        for modo in _modos(modos):
            r = modos[modo]
            x = Caso(r, c, por.get(par, {}).get(modo), modos.get("atual") if modo != "atual" else None, npz(r), npz(c),
                     pe, npz(pe))
            junta(regras_caso(x), r, [ex], modo)
    for ea, eb in comum.MIRROR_PAIRS:
        ma, mb = por.get(ea, {}), por.get(eb, {})
        for modo in _modos(ma):
            if modo in mb:
                junta(regras_par(ma[modo], mb[modo], ma.get("clipe"), mb.get("clipe")), mb[modo], [ea, eb], modo)
    escalar(regs)
    _padrao_lado(regs)
    for a in regs:
        a["id"] = ":".join(p for p in (a["regra"], a["exercicio"], a["modo"], a["detalhe"]) if p)
        a["gravidade"] = GRAVIDADE.get(a["nivel"])
        a["figuras"] = _figuras(a, marcha_dir, capturas)
        a["treino"] = _treino(painel, a)
    return {"registros": regs, "achados": sorted((a for a in regs if a["nivel"]), key=lambda a: (
        -a["nivel"], -PESO[a["regra"]], -min(a["excesso"], 5.0), a["id"]))}


def historico(achados: list, registros: list, anterior: dict) -> dict:
    """Marca cada achado contra a auditoria anterior (novo, piorou, melhorou, persiste) e lista os que sumiram:
    resolvidos (a regra mediu e nao acusou) e nao medidos (a captura nao veio)."""
    velhos = {a["id"]: a for a in anterior.get("achados", [])}
    for a in achados:
        o = velhos.get(a["id"])
        a["historico"] = ("novo" if o is None else "piorou" if a["nivel"] > o["nivel"] else "melhorou"
                          if a["nivel"] < o["nivel"] else "piorou" if a["excesso"] > 1.25 * o["excesso"] else "melhorou"
                          if a["excesso"] < 0.8 * o["excesso"] else "persiste")
        if o is not None:
            a["antes"] = o["medida"]
    agora = {a["id"]: a for a in registros}
    ids = {a["id"] for a in achados}
    fora = [o for i, o in velhos.items() if i not in ids]
    return {"resolvidos": [{**o, "agora": agora[o["id"]]["medida"]} for o in fora if o["id"] in agora],
            "nao_medidos": [o for o in fora if o["id"] not in agora]}


# --- O relatorio ---------------------------------------------------------------------------------------------------------

def _modo_txt(a: dict) -> str:
    """O modo e a forca; o par no atual sem ela (cada um numa forca, as duas no texto)."""
    if a["modo"] != "atual":
        return _pc(a["forca"])
    return "atual" if len(a["exercicios"]) > 1 else f"atual ({_pc(a['forca'])})"


def _ev_txt(ev: dict) -> list:
    out = []
    for nome, d in ev.items():
        if isinstance(d, dict):
            out.append(f"{nome}: " + " · ".join(
                f"{k} {_n(v, 0 if abs(v) >= 100 else 1 if abs(v) >= 10 else 2) if isinstance(v, float) else v}"
                for k, v in d.items() if v is not None))
        else:
            out.append(f"{nome}: {d}")
    return out


def _vies(regs: list) -> list:
    """O mapa do vies lateral: por exercicio e modo, o desvio de lado e de rumo da rede contra o clipe."""
    linhas: dict = {}
    for a in regs:
        if a["regra"] in ("lado", "rumo", "espelho_vies") and a["valor"] is not None:
            chave = a["regra"] if a["regra"] != "espelho_vies" else a["detalhe"]
            linhas.setdefault(a["exercicio"], {})[f"{chave} {a['modo']}"] = a["valor"]
    return [{"exercicio": ex, **v} for ex, v in sorted(linhas.items())]


def markdown(res: dict) -> str:
    ach = res["achados"]
    cont = res["contagem"]
    L = [f"# Auditor do movimento — atualização {', '.join(map(str, res['atualizacao']))}", "",
         f"{res['gerado']} · {res['capturas_rede']} capturas da rede contra {res['clipes']} clipes puros · "
         f"**{cont['alta']} altos, {cont['media']} médios, {cont['baixa']} baixos**"
         + (f" · desde a anterior: {res['mudancas']}" if res.get("mudancas") else ""), ""]
    L += ["## Os piores", ""]
    for i, a in enumerate(ach[:12], 1):
        L.append(f"{i}. **{GRAVIDADE_MD[a['nivel']]} · {a['exercicio']} · {_modo_txt(a)} · {NOMES[a['regra']]}** — "
                 f"{a['o_que']}. *Agir:* {a['agir']}." + (f" *{a['mao']}.*" if a.get("mao") else "")
                 + (f" [{a['historico']}]" if a.get("historico") else ""))
    L += ["", "## Tabela", "", "| # | gravidade | exercício | modo | regra | medida | agir | hist. |",
          "|---|---|---|---|---|---|---|---|"]
    for i, a in enumerate(ach, 1):
        det = f" ({LADO.get(a['detalhe'], a['detalhe'])})" if a["detalhe"] else ""
        L.append(f"| {i} | {GRAVIDADE_MD[a['nivel']]} | {a['exercicio']} | {_modo_txt(a)} | {NOMES[a['regra']]}{det} | "
                 f"{a['medida']} | {a['agir']} | {a.get('historico', '')} |")
    modos = _modos({k.split(" ", 1)[1]: 0 for v in res["vies"] for k in v if k != "exercicio"})
    cols = [f"{q} {m}" for q in ("lado", "rumo") for m in modos]
    L += ["", "## O viés lateral (rede fora do clipe, graus, + = esquerda; 0 = dentro do clipe; lado: o maior entre o "
          "corpo e o tronco; os pares: a parte comum)",
          "", "| exercício | " + " | ".join(f"{q} {m if m == 'atual' else _pc(float(m))}" for q in ("lado", "rumo")
                                            for m in modos) + " |",
          "|---|" + "---|" * len(cols)]
    for v in res["vies"]:
        L.append(f"| {v['exercicio']} | " + " | ".join(_n(v.get(c), 1, True) for c in cols) + " |")
    L += ["", "## Detalhes", ""]
    for i, a in enumerate(ach, 1):
        L += [f"### {i}. {GRAVIDADE_MD[a['nivel']]} · {a['exercicio']} · {_modo_txt(a)} · {NOMES[a['regra']]}", "",
              f"- **O que:** {a['o_que']}", "- **Evidência:**"]
        L += [f"  - {t}" for t in _ev_txt(a["evidencia"])]
        L += [f"- **Causa provável:** {a['causa']}", f"- **Onde agir:** {a['agir']}"]
        if a.get("mao"):
            L.append(f"- **A mão:** {a['mao']}")
        if a.get("treino"):
            L.append(f"- **Treino (painel):** {a['treino']}")
        L.append("- **Figuras:** " + (", ".join(f"`{f}`" for f in a["figuras"]) or "nenhuma (rodar a marcha com as figuras, "
                                       "e o fantasma --padrao em <capturas>/fantasma)"))
        if a.get("historico"):
            L.append(f"- **Histórico:** {a['historico']}" + (f" (antes {a['antes']})" if a.get("antes") else ""))
        L.append("")
    if res.get("resolvidos") or res.get("nao_medidos"):
        L += ["## Sumiram desde a anterior", ""]
        L += [f"- resolvido: {o['id']} (antes {o['medida']}, agora {o['agora']})" for o in res.get("resolvidos", [])]
        L += [f"- não medido (sem a captura): {o['id']} (antes {o['medida']})" for o in res.get("nao_medidos", [])]
        L.append("")
    if res.get("treino"):
        L += [f"## Do treino (painel, atualização {res['treino']['atualizacao']})", ""]
        L += [f"- {p['gravidade']}: {p['texto']}" for p in res["treino"]["problemas"]]
        L.append("")
    L += ["## O que o auditor não vê", "",
          "- Nos laterais o clipe puro degenera (escorrega 25–50 cm por apoio): as regras contra ele ficam lenientes "
          "ali; o par espelhado cobre a diferença entre os lados, não o defeito comum aos dois.",
          "- Os episódios de uma captura são poucos (4 a 5): quedas e as notas do treino nela são ruidosas; o painel "
          "traz as do log.",
          "- No lev_levantar o clipe puro cai depois de levantar: o de pé dele (passos, base, deriva) é medido contra o "
          "clipe puro do lev_parado (o mesmo alert_idle), quando a captura dele vem; sem ela, contra o do levantar, que "
          "escorrega e cai.",
          "- Sem --capturas, não mede o tempo de levantar nem a deriva de pé, e não acha as figuras do fantasma.", ""]
    return "\n".join(L)


def gerar(marcha_dir: Path, saida: Path, capturas: Path | None = None, painel: Path | None = None,
          anterior: Path | None = None) -> dict:
    """Audita a pasta da marcha e grava o relatorio_auditor.md e o auditor.json em `saida`."""
    rs = carregar(marcha_dir)
    if not rs:
        raise SystemExit(f"nenhum .marcha.json em {marcha_dir}")
    pn = json.loads(Path(painel).read_text()) if painel else None
    au = auditar(rs, Path(marcha_dir), capturas, pn)
    ach = au["achados"]
    res = {"gerado": time.strftime("%d/%m/%Y %H:%M"), "marcha": str(marcha_dir),
           "capturas": str(capturas) if capturas else None, "painel": str(painel) if painel else None,
           "atualizacao": sorted({r.get("atualizacao") for r in rs.values() if r.get("atualizacao") is not None}),
           "capturas_rede": sum(r["modo"] != "clipe" for r in rs.values()),
           "clipes": sum(r["modo"] == "clipe" for r in rs.values()),
           "contagem": {g: sum(a["gravidade"] == g for a in ach) for g in ("alta", "media", "baixa")},
           "achados": ach, "vies": _vies(au["registros"]),
           "medidas": {a["id"]: {"nivel": a["nivel"], "excesso": a["excesso"], "medida": a["medida"]}
                       for a in au["registros"]}}
    if anterior:
        res.update(historico(ach, au["registros"], json.loads(Path(anterior).read_text())), anterior=str(anterior))
        n = {h: sum(a["historico"] == h for a in ach) for h in ("novo", "piorou", "melhorou", "persiste")}
        res["mudancas"] = (f"{n['novo']} novos, {n['piorou']} pioraram, {n['melhorou']} melhoraram, "
                           f"{len(res['resolvidos'])} resolvidos")
    if pn:
        res["treino"] = {"atualizacao": pn.get("atualizacao"), "problemas": pn.get("problemas", [])}
    for a in ach:
        a.pop("img", None)
    saida.mkdir(parents=True, exist_ok=True)
    (saida / "auditor.json").write_text(json.dumps(M._limpo(res), ensure_ascii=False, allow_nan=False, indent=1))
    (saida / "relatorio_auditor.md").write_text(markdown(res))
    return res


def imprimir(res: dict, n: int = 12) -> None:
    c = res["contagem"]
    print(f"{len(res['achados'])} achados: {c['alta']} altos, {c['media']} médios, {c['baixa']} baixos")
    for i, a in enumerate(res["achados"][:n], 1):
        print(f"{i:2d}. {a['gravidade']:<5} {a['exercicio']:<18} {a['modo']:<6} {a['regra']:<13} {a['medida']:<16} "
              f"{a['o_que']}" + (f" [{a['mao']}]" if a.get("mao") else ""))


# --- O autoteste ---------------------------------------------------------------------------------------------------------

def autoteste(saida: Path = SAIDA / "teste") -> None:
    """Nas amostras: a marcha de todas as capturas (sem figuras, mais as do lev_trote_0 0.3), a auditoria com um painel
    e uma auditoria anterior sinteticos, e os fatos: a inclinacao para a direita do lev_trote_0 (atual e 0.3) e do
    lev_sprint 0.3 acusada (e nao a do sprint atual, -0,8 grau, nem a do trote_4); nenhum clipe puro acusa contra si
    mesmo (nem os pares de clipes); um par sintetico com um lado pior em cada medida sai misto; a perna direita dura do
    lev_corrida_6 e o par corrida 2x6 com a direita pior; a mao escondendo no trote_0; a cadencia inflada pelos toques
    repetidos; o de pe do levantar contra o lev_parado; a inclinacao dos laterais contra o movimento; o quase-toque
    isolado dos pes; o historico; o JSON estrito."""
    t0 = time.time()
    md = saida / "marcha"
    md.mkdir(parents=True, exist_ok=True)
    for p in sorted(AMOSTRAS.glob("*.npz")):
        if p.stem == "lev_trote_0__0.3":
            M.salvar(p, md)
        else:
            r = M.analisar(G.carregar(p))
            (md / f"{r['captura']}.marcha.json").write_text(json.dumps(r, ensure_ascii=False))
    print(f"marcha das {len(list(AMOSTRAS.glob('*.npz')))} amostras em {time.time() - t0:.0f} s")
    rs = carregar(md)
    # (O clipe puro contra si mesmo, e os pares de clipes: nada acusa - os limiares sao relativos ao clipe.)
    por = _por_exercicio(rs)
    for ex, modos in por.items():
        c = modos["clipe"]
        e = por.get(comum.mirror_of(ex), {}).get("clipe")
        caps = [AMOSTRAS / f"{c['captura']}.npz"] * 2
        acusa = [a for a in regras_caso(Caso(c, c, e, None, *caps)) if a["nivel"]]
        assert not acusa, f"o clipe puro acusa contra si: {ex} {[(a['regra'], a['medida']) for a in acusa]}"
    for ea, eb in comum.MIRROR_PAIRS:
        if ea in por and eb in por:
            ca, cb = por[ea]["clipe"], por[eb]["clipe"]
            assert not [a for a in regras_par(ca, cb, ca, cb) if a["nivel"]], f"o par de clipes {ea}x{eb} acusa"
    print("nenhum clipe puro acusa contra si (nem os pares)")
    # (Os pontos de lados opostos nao se anulam: o joelho direito do lev_corrida_6 dobrando a metade do clipe e o pe
    # direito do lev_corrida_2 escorregando 40 cm a mais dao misto, media - antes, saldo 0 e "fazem o mesmo".)
    ca, cb = (copy.deepcopy(por[e]["clipe"]) for e in ("lev_corrida_2", "lev_corrida_6"))
    cb["lados"]["d"]["joelho_pico"] = 0.5 * cb["referencia"]["lados"]["d"]["joelho_pico"]
    ca["lados"]["d"]["deslize_cm"] += 40.0
    m = r_espelho_lados(ca, cb, por["lev_corrida_2"]["clipe"], por["lev_corrida_6"]["clipe"])[0]
    assert m["evidencia"]["veredito"] == "misto" and m["nivel"] == 2, (m["medida"], m["o_que"])
    print(f"par misto sintetico: {m['medida']}, {m['o_que']}")
    painel = {"atualizacao": 1, "exercicios": {"lev_corrida_6": {"forca": 0.779, "estado": "segura", "bloqueios": [
        {"exigencia": "semelhanca", "valor": 0.525, "exigido": 0.534}]}},
        "pares": [{"esquerda": "lev_corrida_2", "direita": "lev_corrida_6", "veredito": "D pior",
                   "medidas": {"forca": {"E": 0.5, "D": 0.779, "nivel": 2}}}],
        "vies_rumo": {"corrida": {"giro": -0.14, "deriva": -0.51}}, "problemas": [{"gravidade": "alerta", "texto": "x"}]}
    (saida / "painel_teste.json").write_text(json.dumps(painel))
    res = gerar(md, saida, AMOSTRAS, saida / "painel_teste.json")
    imprimir(res, 15)
    ach = {a["id"]: a for a in res["achados"]}
    for i in ("lado:lev_trote_0:atual", "lado:lev_trote_0:0.3", "lado:lev_sprint:0.3"):
        assert i in ach and ach[i]["valor"] < 0, f"a inclinacao para a direita deveria acusar: {i}"
        print(f"{i}: {ach[i]['gravidade']}, {ach[i]['o_que']}")
    assert "lado:lev_sprint:atual" not in ach and not any(i.startswith("lado:lev_trote_4") for i in ach)
    assert not any(a["modo"] == "clipe" for a in res["achados"])
    assert ach["lado:lev_trote_0:0.3"].get("mao", "").startswith("a mão esconde")
    assert "joelho:lev_corrida_6:atual:d" in ach, "a perna direita dura da corrida lateral direita"
    par = ach.get("espelho_lados:lev_corrida_2×6:atual")
    assert par and par["valor"] > 0 and "o par no log: D pior" in (par["treino"] or ""), "corrida 2x6: a direita pior"
    print(f"{par['id']}: {par['gravidade']}, {par['o_que']}")
    fig = ach["lado:lev_trote_0:0.3"]["figuras"]
    assert fig and fig[0].endswith("lev_trote_0__0.3.series.png"), fig
    assert {"parado:lev_parado:atual:deriva", "levantar:lev_levantar:atual"} <= set(res["medidas"]), \
        "o parado e o levantar medidos com as capturas"
    # (Os da revisao: a cadencia da corrida a 91%, inflada pelos toques repetidos (238 contra 171, a mediana do passo
    # igual a do clipe), nao acusa - acusam os toques; o de pe depois do levantar contra o clipe puro do lev_parado
    # (37 cm de deriva contra 2); a inclinacao dos laterais contra o movimento, nas pernas laterais e nao no vies; o
    # quase-toque isolado dos pes (p5 acima do clipe) fica media.)
    med = res["medidas"]
    assert "ritmo:lev_corrida_0:atual" not in ach and med["ritmo:lev_corrida_0:atual:repetidos"]["nivel"], \
        "a cadencia inflada pelos toques repetidos"
    assert ach["parado:lev_levantar:atual:deriva"]["nivel"] == 3, "o de pe depois do levantar escorrega"
    l6 = ach["lado:lev_trote_6:0.3"]
    assert "contra o movimento" in l6["o_que"] and l6["agir"] == "pernas laterais", l6["o_que"]
    assert ach["lado:lev_trote_0:0.3"]["agir"] == "viés lateral da rede"
    assert ach["pes_juntos:lev_corrida_6:0.3"]["nivel"] == 2 and ach["pes_juntos:lev_trote_6:0.3"]["nivel"] == 3
    # (O trote de lado para a direita mais lento que o espelho: 2,2 contra 2,9 m/s a 30%, os clipes puros iguais.)
    t26 = ach["espelho_lados:lev_trote_2×6:0.3"]
    assert t26["valor"] > 0 and "avanço: 2,22 m/s" in t26["o_que"], t26["o_que"]
    for i in ("parado:lev_levantar:atual:deriva", "lado:lev_trote_6:0.3", "pes_juntos:lev_corrida_6:0.3",
              "espelho_lados:lev_trote_2×6:0.3"):
        print(f"{i}: {ach[i]['gravidade']}, {ach[i]['o_que']}")
    # (O historico: uma anterior com um achado a menos, um menor, outro maior e um que sumiu.)
    velha = copy.deepcopy(res)
    a0, a1, a2 = velha["achados"][:3]
    a1["excesso"] /= 2
    a2["excesso"] *= 2
    velha["achados"] = velha["achados"][1:] + [{**a0, "id": "lado:lev_trote_4:0.3", "medida": "+5,0°"}]
    (saida / "anterior_teste.json").write_text(json.dumps(velha, ensure_ascii=False))
    res2 = gerar(md, saida, AMOSTRAS, saida / "painel_teste.json", saida / "anterior_teste.json")
    h = {a["id"]: a["historico"] for a in res2["achados"]}
    assert h[a0["id"]] == "novo" and h[a1["id"]] == "piorou" and h[a2["id"]] == "melhorou", h
    assert [o["id"] for o in res2["resolvidos"]] == ["lado:lev_trote_4:0.3"] and res2["mudancas"]
    json.loads((saida / "auditor.json").read_text())
    assert "| # | gravidade |" in (saida / "relatorio_auditor.md").read_text()
    print(f"historico: {res2['mudancas']}")
    print(f"auditor ok em {time.time() - t0:.0f} s (saida em {saida})")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--marcha", help="a pasta com os .marcha.json (inspecao.marcha)")
    ap.add_argument("--capturas", help="a pasta dos .npz (o levantar, a deriva de pe e o fantasma em <capturas>/fantasma)")
    ap.add_argument("--painel", help="o painel.json do inspecao.painel")
    ap.add_argument("--anterior", help="o auditor.json de uma auditoria anterior")
    ap.add_argument("--saida", default=str(SAIDA))
    ap.add_argument("--teste", action="store_true", help="o autoteste nas amostras")
    a = ap.parse_args()
    if a.teste:
        autoteste()
        return
    if not a.marcha:
        ap.error("informe --marcha ou --teste")
    res = gerar(Path(a.marcha), Path(a.saida), Path(a.capturas) if a.capturas else None, a.painel, a.anterior)
    imprimir(res)
    print(f"saida: {Path(a.saida) / 'relatorio_auditor.md'}")


if __name__ == "__main__":
    main()
