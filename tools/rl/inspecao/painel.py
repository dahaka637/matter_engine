"""O painel dos dados do treino (so le o log.jsonl, o treino.out, o status.json e o log do vigia termico: seguro com o treino
rodando): a forca de cada exercicio e a descida dela, o que segura cada um (qual exigencia da regua falha e ha quanto
tempo, a simulacao do dynamic_assist com o que o log guarda), as subidas, a previsao ate 0%, o ranking de dificuldade, os
pares espelhados (o lado direito pior) e o vies de rumo da rede (o giro e a deriva lateral tirados dos 8 rumos), a saude
(linhas, lacunas, vazao, KL e taxa, ruido, NaN, erros no treino.out, temperaturas), o levantar, a avaliacao sem a mao e o
rastreio.

Saida em DIR: painel.md (a checagem de hora em hora: os problemas e o ranking primeiro), painel.json, forcas.png (a forca
de cada exercicio no tempo, o fundo pela exigencia que segura), bloqueios.png (exercicio x tempo: o que segurou cada
decisao) e espelhos.png (os pares lado a lado e a diferenca de forca no tempo).

Uso:
  python -m inspecao.painel [--treino rl_final_clipe] --saida DIR [--horas 3]
  python -m inspecao.painel --treino rl_final_clipe/arquivo_antes_reset_reto_0626_0511 --saida DIR   (um arquivado)
  python -m inspecao.painel --teste
"""
from __future__ import annotations

import argparse
import json
import re
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from inspecao import comum

# (A regua de levantar_amp.dynamic_assist: sobe com o fim abaixo de 0,85 ou P abaixo de 75% da marca; desce com P >= 92%
# dela; a marca esquece 0,002 por decisao; 3 decisoes de espera depois de uma subida; o passo cai a metade abaixo de 30%.)
FIM_SOBE, P_DESCE, P_SOBE, P_ESQUECE, ESPERA, METADE = 0.85, 0.92, 0.75, 0.002, 3, 30.0
# (O piso da taxa adaptativa por cabeca - levantar_amp, --taxa-adaptativa - e o do ruido da politica - LOG_STD_MIN.)
TAXA_PISO, RUIDO_PISO = 1.0e-6, 0.05
TEMPERATURAS = Path.home() / ".local/share/matter-engine/vigia_temperatura.log"
# (NaN nestes e o treino quebrado: o log grava None no lugar - finite_json.)
NUCLEO = ("kl", "kl_loco", "kl_rec", "pol", "vf", "tarefa", "desvio", "desvio_loco", "am_s")
ERRO = re.compile(r"Traceback|double free|Segmentation fault|core dumped|Killed|Aborted|\w+Error\b|"
                  r"\b(?:" + "|".join(NUCLEO) + r")=-?(?:nan|inf)\b")
# (A ultima linha de rodada impressa: um erro depois dela e do processo de agora. As linhas [INFO] do modulo nativo nao
# servem de marca: o C as guarda e solta na saida do processo, depois do Traceback.)
RODADA = re.compile(r"^(?:aval )?atualizacao=", re.M)
RUMOS = ("frente", "fr-esq", "esq", "tr-esq", "trás", "tr-dir", "dir", "fr-dir")
ORDEM = [f"lev_trote_{i}" for i in range(8)] + [f"lev_corrida_{i}" for i in range(8)] + [
    "lev_troca_suave", "lev_troca_media", "lev_troca_forte", "lev_troca_livre", "lev_curva_suave", "lev_curva_s",
    "lev_curva_fechada", "lev_giro", "lev_curva_suave_corrida", "lev_curva_s_corrida", "lev_curva_fechada_corrida",
    "lev_sprint", "lev_parado", "lev_levantar"]
# As exigencias da regua: a letra, a cor (a paleta categorica) e o nome; a ordem e a prioridade da principal no mapa.
EXIGENCIAS = {"fim": ("F", "#2a78d6", "fim"), "estabilidade": ("E", "#1baf7a", "estabilidade"),
              "semelhanca": ("S", "#eb6834", "semelhança"), "vel": ("V", "#eda100", "velocidade"),
              "tempo": ("T", "#eda100", "tempo do levantar"), "desvio": ("D", "#e87ba4", "desvio de rumo"),
              "cruzado": ("C", "#8b5a2b", "pé cruzado na corrida"), "P": ("P", "#6250d6", "P < 92% da marca")}
ESTADOS = {"desceu": ("", "#d3ebd0", "desceu"), "subiu": ("↑", "#d03b3b", "subiu"),
           "aguardando": ("·", "#b9b8b0", "aguardando (espera ou amostra)"),
           "indefinido": ("?", "#e1e0d9", "segurou sem exigência visível"), "zerado": ("", "#f4f3ef", "0%")}
CODIGOS = list(ESTADOS) + list(EXIGENCIAS)
NOMES = {"forca": "força", **{k: v[2] for k, v in EXIGENCIAS.items()}}
TINTA, TINTA2, GRADE, FUNDO = "#0b0b0b", "#52514e", "#e1e0d9", "#fcfcfb"
# (Os limiares do lado pior num par espelhado - pior, muito pior -; forca e medidas em fracao, desvio em graus.)
PAR = {"forca": (0.05, 0.15), "semelhanca": (0.03, 0.08), "fim": (0.02, 0.05), "estabilidade": (0.03, 0.08),
       "desvio": (1.0, 2.0)}
QUEDA = ("levantou", "de_pe", "tempo", "caiu_de_pe", "brucos", "costas", "lado")
AVAL = ("trote_", "corrida_", "sprint_", "manobras_", "parado_", "equil_")
RASTREIO = ("nota", "inclinacao_graus", "coluna_graus", "arrasto_pe", "escorrega_contato", "tropeco", "pulo_pelve",
            "giro_pelve", "mao_sustenta_n", "mao_sustenta_p90", "firmeza", "margem_hof_m")
MANTER = set(NUCLEO) | {"atualizacao", "hora", "aval", "taxa_loco", "taxa_rec", "t_fisica", "ator_recuperacao_parado",
                        "ajuda_exercicios", "dominio", "exigencias"} | {f"{p}{k}" for p in ("queda_", "clipe_") for k in QUEDA} \
    | {f"rastreio_{k}" for k in RASTREIO} | {f"{p}{k}" for p in AVAL for k in ("sem_cair", "seg")}


def nome(key: str) -> str:
    m = re.fullmatch(r"lev_(trote|corrida)_([0-7])", key)
    return f"{m[1]} {m[2]} {RUMOS[int(m[2])]}" if m else key.removeprefix("lev_").replace("_", " ")


# --- A leitura -------------------------------------------------------------------------------------------------------

def ler_log(caminho: Path, desde: int = 0) -> tuple[list[dict], int]:
    """As linhas completas do log a partir do byte `desde`, so com o que o painel usa (uma linha tem ~40 KB, quase tudo os
    cenarios), e o byte depois da ultima lida (para a leitura seguinte)."""
    with open(caminho, "rb") as f:
        f.seek(desde)
        dados = f.read()
    fim = dados.rfind(b"\n") + 1
    linhas = []
    for bruta in dados[:fim].splitlines():
        try:
            r = json.loads(bruta)
        except ValueError:
            continue
        if isinstance(r, dict) and "hora" in r and "atualizacao" in r:
            out = {k: v for k, v in r.items() if k in MANTER}
            if isinstance(r.get("cenarios"), dict):
                out["cenarios"] = {k: {c: v.get(c) for c in ("desvio_rumo", "ate_o_fim", "semelhanca", "estabilidade")}
                                   for k, v in r["cenarios"].items() if isinstance(v, dict)}
            linhas.append(out)
    return linhas, desde + fim


def erros_treino(run: Path) -> list[str]:
    """As linhas de erro do treino.out (Traceback, NaN no nucleo, double free...) do processo de agora: o fim do arquivo,
    da ultima linha de rodada em diante (um processo religado que ja rodou apaga o erro do anterior)."""
    try:
        with open(run / "treino.out", "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 262144))
            texto = f.read().decode(errors="ignore")
    except OSError:
        return []
    rodadas = [m.start() for m in RODADA.finditer(texto)]
    texto = texto[rodadas[-1]:] if rodadas else texto
    return [linha.strip()[:200] for linha in texto.splitlines() if ERRO.search(linha)]


_KL: dict = {}


def kl_alvo(run: Path) -> float | None:
    """O KL alvo da taxa adaptativa (a ultima linha "taxa adaptativa (KL alvo X)" do treino.out): o arquivo inteiro na
    primeira vez, depois so o fim (um comeco novo a reimprime; o treino.out nao e condensado e cresce sempre)."""
    p = run / "treino.out"
    try:
        with open(p, "rb") as f:
            if p in _KL:
                f.seek(0, 2)
                f.seek(max(0, f.tell() - 262144))
            achados = re.findall(rb"taxa adaptativa \(KL alvo ([0-9.eE+-]+)\)", f.read())
    except OSError:
        return None
    if achados or p not in _KL:
        _KL[p] = float(achados[-1]) if achados else None
    return _KL[p]


def temperaturas(t0: float, t1: float) -> dict:
    """O vigia termico (o log dele): a ultima leitura e a idade dela, a maxima da CPU em [t0, t1], as paradas termicas na
    janela e se o treino esta parado por ele agora (a ultima parada sem um "esfriou" depois)."""
    try:
        with open(TEMPERATURAS, "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 200000))
            texto = f.read().decode(errors="ignore")
    except OSError:
        return {}
    leituras, eventos, agora = [], [], time.time()
    for linha in texto.splitlines():
        m = re.match(r"(\d\d)/(\d\d) (\d\d:\d\d:\d\d) (.*)", linha)
        if not m:
            continue
        t = datetime.strptime(f"{datetime.now().year}-{m[2]}-{m[1]} {m[3]}", "%Y-%m-%d %H:%M:%S").timestamp()
        t -= 365 * 86400 if t > agora + 86400 else 0  # (a virada do ano)
        c = re.match(r"CPU ([0-9.]+) GPU ([0-9.]+)", m[4])
        if c:
            leituras.append((t, float(c[1]), float(c[2])))
        elif m[4].startswith(("LIMITE DE PARAR", "EMERGENCIA", "esfriou")):
            eventos.append((t, m[4]))
    if not leituras:
        return {}
    # (O pico que parou o treino so aparece na linha do evento: "LIMITE DE PARAR: CPU a 84.0 graus".)
    janela = [c for t, c, _ in leituras if t0 <= t <= t1] + [
        float(c[1]) for t, e in eventos if t0 <= t <= t1 and (c := re.search(r"CPU a ([0-9.]+)", e))]
    ate = [e for t, e in eventos if t <= t1]
    return {"cpu": leituras[-1][1], "gpu": leituras[-1][2], "idade_min": (agora - leituras[-1][0]) / 60,
            "cpu_max": max(janela) if janela else None,
            "paradas": [(t, e) for t, e in eventos if t0 <= t <= t1 and not e.startswith("esfriou")],
            "pausado": bool(ate) and not ate[-1].startswith("esfriou")}


def status(run: Path) -> dict:
    try:
        s = json.loads((run / "status.json").read_text())
    except (OSError, ValueError):
        return {}
    pid = s.get("pid")
    return {"estado": s.get("state"), "pid": pid, "vivo": bool(pid) and Path(f"/proc/{pid}").exists()}


# --- A regua, decisao por decisao ------------------------------------------------------------------------------------

def _falhas(d: dict, e: dict, ref: float) -> list[str]:
    """As exigencias que falham numa decisao (as contas do dynamic_assist com os minimos que o log guarda; sem a
    exigencias.fim - os logs antigos - o fim nao entra)."""
    out = []
    if "fim" in e and d["fim"] + 1e-9 < e["fim"]:
        out.append("fim")
    if "estab_exigida" in d and d["estabilidade"] < d["estab_exigida"]:
        out.append("estabilidade")
    if "exigido" in d and d["sem_teto"] < d["exigido"]:
        out.append("semelhanca")
    if "tempo_exigido" in d:
        if d.get("tempo", 99.0) > d["tempo_exigido"]:
            out.append("tempo")
    elif "vel_exigida" in d and d["vel"] < d["vel_exigida"]:
        out.append("vel")
    if "desvio" in e and d.get("desvio", 0.0) > e["desvio"]:
        out.append("desvio")
    if "cruzado" in e and d.get("cruzado", 0.0) > e["cruzado"]:
        out.append("cruzado")
    if d["P"] < P_DESCE * max(ref, 1e-3):
        out.append("P")
    return out


def historico(treino: list[dict], key: str) -> list[dict]:
    """Cada decisao da regua de um exercicio (uma por linha de treino): a forca depois dela, o estado (desceu, subiu,
    segura, aguardando, indefinido, zerado) e as exigencias que falharam; a marca de P e a espera reconstruidas desde o
    comeco do log (o estado de verdade fica no checkpoint)."""
    out, ref, espera, antes = [], None, 0, None
    for r in treino:
        f = r["ajuda_exercicios"].get(key)
        if f is None:
            continue
        d = (r.get("dominio") or {}).get(key)
        e = (r.get("exigencias") or {}).get(key) or {}
        falhas, motivo = [], "" if d else "amostra"
        if d:
            ref = d["P"] if ref is None else max(ref - P_ESQUECE, d["P"])
            if d["fim"] < FIM_SOBE or d["P"] < P_SOBE * ref:
                espera = ESPERA
            elif espera > 0:
                espera, motivo = espera - 1, "espera"
            falhas = _falhas(d, e, ref)
        mudou = 0.0 if antes is None else f - antes
        if mudou > 1e-6:
            # (Toda subida arma a espera, mesmo sem o motivo visivel aqui: a marca reconstruida abaixo da de verdade ou
            # a linha condensada.)
            espera = ESPERA
        estado = ("desceu" if mudou < -1e-6 else "subiu" if mudou > 1e-6 else "zerado" if f <= 0.0 else
                  "aguardando" if motivo else "segura" if falhas else "indefinido")
        out.append({"hora": r["hora"], "forca": f, "estado": estado, "falhas": falhas, "motivo": motivo, "d": d, "e": e,
                    "ref": ref})
        antes = f
    return out


def _valor_exigido(q: str, d: dict, e: dict, ref: float | None = None) -> tuple:
    """(o valor, o minimo - ou o maximo, no tempo e no desvio - exigido) de uma exigencia numa decisao."""
    return {"fim": (d.get("fim"), e.get("fim")), "vel": (d.get("vel"), d.get("vel_exigida")),
            "tempo": (d.get("tempo"), d.get("tempo_exigido")), "semelhanca": (d.get("semelhanca"), e.get("semelhanca")),
            "estabilidade": (d.get("estabilidade"), d.get("estab_exigida")), "desvio": (d.get("desvio"), e.get("desvio")),
            "cruzado": (d.get("cruzado"), e.get("cruzado")), "P": (d.get("P"), P_DESCE * ref if ref else None)}[q]


def _ha_quanto(hist: list[dict], q: str) -> float:
    """Ha quantos segundos a exigencia q falha sem parar ate a ultima decisao (as sem amostra nao contam; uma descida ou
    um acerto interrompem)."""
    desde = None
    for h in reversed(hist):
        if h["estado"] == "desceu" or (h["d"] and q not in h["falhas"]):
            break
        if h["d"]:
            desde = h["hora"]
    return hist[-1]["hora"] - desde if desde is not None else 0.0


def _eta(f: float, taxa: float | None) -> float | None:
    """As horas ate 0% na taxa de agora (%/h); abaixo de 30% o passo maximo cai a metade."""
    if taxa is None or taxa < 0.05:
        return None
    p = f * 100
    return (p - METADE) / taxa + METADE / (taxa / 2) if p > METADE else p / taxa


def _exercicio(hist: list[dict], t1: float, rumo: float | None) -> dict:
    hora = np.array([h["hora"] for h in hist])
    forca = np.array([h["forca"] for h in hist])

    def em(t: float) -> float | None:
        i = int(np.searchsorted(hora, t, side="right")) - 1
        return float(forca[i]) if i >= 0 else None

    f = float(forca[-1])
    # (A taxa na ultima hora - ou desde o comeco do log, com 15 min no minimo.)
    janela = min(3600.0, t1 - hora[0])
    taxa = (em(t1 - janela) - f) * 100 / (janela / 3600) if janela >= 900 else None
    desceu = [h["hora"] for h in hist if h["estado"] == "desceu"]
    mudou = [h["hora"] for h in hist if h["estado"] in ("desceu", "subiu")]
    ultima = hist[-1]
    medida = next((h for h in reversed(hist) if h["d"]), None)
    falhas = [] if ultima["estado"] in ("desceu", "zerado") or medida is None else medida["falhas"]
    return {"forca": f, "forca_30": em(t1 - 1800), "forca_60": em(t1 - 3600), "forca_ini": em(t1 - janela),
            "taxa_h": taxa, "janela_taxa_min": janela / 60,
            "sem_descer_min": (t1 - (desceu[-1] if desceu else hora[0])) / 60, "nunca_desceu": not desceu,
            "ultima_mudanca_min": (t1 - (mudou[-1] if mudou else hora[0])) / 60,
            "subidas_1h": sum(h["estado"] == "subiu" and h["hora"] > t1 - 3600 for h in hist),
            "subidas_10min": sum(h["estado"] == "subiu" and h["hora"] > t1 - 600 for h in hist),
            "estado": ultima["estado"], "aguardando": ultima["motivo"] if ultima["estado"] == "aguardando" else "",
            "bloqueios": [dict(zip(("exigencia", "ha_min", "valor", "exigido"), (q, _ha_quanto(hist, q) / 60,
                                                                                 *_valor_exigido(q, medida["d"], medida["e"],
                                                                                                 medida["ref"]))))
                          for q in falhas],
            "eta_h": _eta(f, taxa), "medidas": medida["d"] if medida else {}, "exigencias": medida["e"] if medida else {},
            "falhas": medida["falhas"] if medida else [], "rumo": rumo}


def ranking(ex: dict) -> list[str]:
    """A dificuldade: primeiro os sem previsao de chegar a 0% (parados ou subindo: mais subidas, mais tempo sem descer,
    mais forca), depois os com previsao, a mais longa primeiro."""
    def chave(k):
        e = ex[k]
        if e["eta_h"] is None:
            return (0, -e["subidas_1h"], -e["sem_descer_min"], -e["forca"])
        return (1, -e["eta_h"], 0, 0)
    return sorted((k for k in ex if ex[k]["forca"] > 0), key=chave)


# --- Os espelhos e o vies --------------------------------------------------------------------------------------------

def pares(ex: dict) -> list[dict]:
    """Os pares espelhados (o clipe de um e o espelho do outro): a diferenca e o vies aprendido da rede. pior > 0 e nivel
    > 0: a direita pior (nivel 2, muito pior); < 0, a esquerda."""
    out = []
    for e, d in comum.MIRROR_PAIRS:
        if e not in ex or d not in ex:
            continue
        linha = {"esquerda": e, "direita": d, "medidas": {}}
        for m, (leve, forte) in PAR.items():
            a = ex[e]["forca"] if m == "forca" else ex[e]["medidas"].get(m)
            b = ex[d]["forca"] if m == "forca" else ex[d]["medidas"].get(m)
            if a is None or b is None:
                continue
            pior = (b - a) if m in ("forca", "desvio") else (a - b)
            nivel = int(np.sign(pior)) * (2 if abs(pior) >= forte else 1 if abs(pior) >= leve else 0)
            linha["medidas"][m] = {"E": a, "D": b, "pior": pior, "nivel": nivel}
        ra, rb = ex[e]["rumo"], ex[d]["rumo"]
        linha["rumo"] = {"E": ra, "D": rb, "vies": (ra + rb) / 2 if ra is not None and rb is not None else None}
        linha["nivel"] = max((v["nivel"] for v in linha["medidas"].values()), default=0)
        # (O veredito pela maioria das medidas que diferem: um lado pior numa e melhor noutra e misto.)
        dp = sum(v["nivel"] > 0 for v in linha["medidas"].values())
        ep = sum(v["nivel"] < 0 for v in linha["medidas"].values())
        linha["veredito"] = "D pior" if dp > ep else "E pior" if ep > dp else "misto" if dp else "iguais"
        out.append(linha)
    return out


def vies_rumo(cen: dict) -> dict:
    """O vies de rumo da rede por faixa (graus, + = esquerda, como o desvio de rumo e as outras ferramentas): a parte par
    do desvio nos 8 rumos - a media com o rumo espelhado; numa rede simetrica ela e zero - ajustada a
    e(th) = giro + deriva cos(th) + c cos(2 th): giro > 0, o deslocamento gira para a esquerda em todos os rumos;
    deriva > 0, o corpo (de frente para a vista) escorrega para a esquerda (desvia para a esquerda indo para a frente e
    para a direita indo para tras). Negativos: para a direita."""
    out = {}
    th = np.radians(45.0 * np.arange(8))
    base = np.stack([np.ones(8), np.cos(th), np.cos(2 * th)], 1)
    for faixa in ("trote", "corrida"):
        d = [(cen.get(f"lev_{faixa}_{h}") or {}).get("desvio_rumo") for h in range(8)]
        if any(x is None for x in d):
            continue
        d = np.array(d, float)
        par = (d + d[(8 - np.arange(8)) % 8]) / 2
        (giro, deriva, _), *_ = np.linalg.lstsq(base, par, rcond=None)
        out[faixa] = {"giro": float(giro), "deriva": float(deriva), "desvio": d.tolist(), "par": par.tolist()}
    return out


# --- A saude ---------------------------------------------------------------------------------------------------------

def _mediana(linhas: list[dict], chave: str, t0: float, t1: float) -> float | None:
    vals = [r[chave] for r in linhas if t0 < r["hora"] <= t1 and isinstance(r.get(chave), (int, float))]
    return float(np.median(vals)) if vals else None


def lacunas(treino: list[dict], desde: float) -> list[tuple[float, float, float]]:
    """As paradas do treino depois de `desde`: (inicio, fim, segundos parado) - o tempo entre duas linhas alem do que as
    atualizacoes entre elas pedem. (O treino condensa o log a cada salvamento - storage.condense_log: as linhas com mais
    de 10 min viram grupos de 1, 3, 6, 12 min... -, entao o espaco entre linhas nao serve; a atualizacao que nao anda e
    um reinicio do checkpoint.)"""
    pares = list(zip(treino[:-1], treino[1:]))
    ritmo = [(b["hora"] - a["hora"]) / (b["atualizacao"] - a["atualizacao"]) for a, b in pares[-60:]
             if b["atualizacao"] > a["atualizacao"]]
    if not ritmo:
        return []
    ritmo = float(np.median(ritmo))
    out = []
    for a, b in pares:
        esperado = max(0, b["atualizacao"] - a["atualizacao"]) * ritmo
        parado = b["hora"] - a["hora"] - esperado
        if b["hora"] > desde and (esperado == 0 or parado > max(90.0, 0.25 * esperado)):
            out.append((a["hora"], b["hora"], parado))
    return out


def saude(linhas: list[dict], run: Path, horas: float) -> dict:
    t1 = linhas[-1]["hora"]
    treino = [r for r in linhas if "am_s" in r]
    h = np.array([r["hora"] for r in treino])
    # (A cadencia: a mediana dos 10 ultimos intervalos - recentes, quase sem condensar; uma parada no meio nao a move.)
    passos = np.diff(h[-11:])
    cad = float(np.median(passos)) if len(passos) else 60.0
    agora = float(np.median([r["am_s"] for r in treino[-3:]])) if treino else None
    hora_antes = _mediana(treino, "am_s", t1 - 3600, t1)
    taxas = [r for r in treino if r["hora"] > t1 - 3600]

    def no_piso(k: str) -> float | None:
        vals = [r[k] for r in taxas if r.get(k)]
        return float(np.mean([v <= TAXA_PISO * 1.05 for v in vals])) if vals else None

    nan = [r["atualizacao"] for r in linhas if r["hora"] > t1 - horas * 3600 and any(k in r and r[k] is None for k in NUCLEO)]
    return {"ultima_linha_min": (time.time() - t1) / 60, "cadencia_s": cad,
            "lacunas": lacunas(treino, t1 - horas * 3600),
            "vazao": {"agora": agora, "hora": hora_antes,
                      "variacao": agora / hora_antes - 1 if agora and hora_antes else None},
            "t_fisica": _mediana(treino, "t_fisica", t1 - 600, t1),
            "kl": {"alvo": kl_alvo(run), "loco": _mediana(treino, "kl_loco", t1 - 1800, t1),
                   "rec": _mediana(treino, "kl_rec", t1 - 1800, t1)},
            "taxa": {"loco": _mediana(treino, "taxa_loco", t1 - 1800, t1), "rec": _mediana(treino, "taxa_rec", t1 - 1800, t1),
                     "loco_piso": no_piso("taxa_loco"), "rec_piso": no_piso("taxa_rec"),
                     "rec_parada": float(np.mean([r.get("ator_recuperacao_parado") or 0.0 for r in taxas])) if taxas else None},
            "ruido": {"loco": treino[-1].get("desvio_loco") if treino else None,
                      "rec": treino[-1].get("desvio") if treino else None},
            "nan": nan, "erros": erros_treino(run),
            # (Um treino com linha na ultima hora: a janela termica vai ate agora - a parada que o deixou sem linha nova -;
            # um arquivado, ate a ultima linha dele.)
            "temperatura": temperaturas(t1 - horas * 3600, max(t1, time.time()) if time.time() - t1 < 3600 else t1),
            "status": status(run)}


# --- A analise toda --------------------------------------------------------------------------------------------------

def analisar(linhas: list[dict], run: Path, horas: float = 3.0) -> tuple[dict, dict] | None:
    """A analise (o dicionario do painel.json) e os historicos de cada exercicio (para os graficos); None sem linha de
    treino com a forca por exercicio."""
    treino = [r for r in linhas if "ajuda_exercicios" in r]
    if not treino:
        return None
    t1 = linhas[-1]["hora"]
    ultimas = treino[-1]["ajuda_exercicios"]
    chaves = [k for k in ORDEM if k in ultimas] + [k for k in ultimas if k not in ORDEM]
    cen = next((r["cenarios"] for r in reversed(treino) if r.get("cenarios")), {})
    hist = {k: historico(treino, k) for k in chaves}
    # (O rumo so nos rumos retos e no sprint - levantar_amp.course_drill: nas trocas e nas curvas a coluna 31 mede contra o
    # rumo do comeco, 41-77 graus, e no giro e no parado nao ha deslocamento pedido.)
    ex = {k: _exercicio(hist[k], t1, (cen.get(k) or {}).get("desvio_rumo")
                        if k.startswith(("lev_trote_", "lev_corrida_")) or k == "lev_sprint" else None) for k in chaves}
    avaliacoes = [r for r in linhas if r.get("aval")]
    antes = (t1 - 4200, t1 - 3000)
    janela = max(ex[k]["janela_taxa_min"] for k in chaves)
    an = {"treino": run.name, "atualizacao": linhas[-1]["atualizacao"], "hora": t1, "horas": horas,
          "inicio": max(t1 - horas * 3600, linhas[0]["hora"]), "exercicios": ex,
          "ranking": ranking(ex), "pares": pares(ex), "vies_rumo": vies_rumo(cen), "saude": saude(linhas, run, horas),
          "levantar": {p: {k: (_mediana(treino, p + k, t1 - 600, t1), _mediana(treino, p + k, *antes)) for k in QUEDA}
                       for p in ("queda_", "clipe_")},
          "aval": {p[:-1]: {k: (_mediana(avaliacoes, p + k, t1 - 600, t1), _mediana(avaliacoes, p + k, *antes))
                            for k in ("sem_cair", "seg")} for p in AVAL},
          "rastreio": {k: (_mediana(treino, f"rastreio_{k}", t1 - 600, t1), _mediana(treino, f"rastreio_{k}", *antes))
                       for k in RASTREIO},
          "regua": {"forca_media": float(np.mean(list(ultimas.values()))),
                    "forca_media_ini": float(np.mean([ex[k]["forca_ini"] for k in chaves])), "janela_min": janela,
                    "descidas_1h": sum(h["estado"] == "desceu" and h["hora"] > t1 - 3600 for k in chaves for h in hist[k]),
                    "subidas_1h": sum(ex[k]["subidas_1h"] for k in chaves), "zerados": sum(v <= 0 for v in ultimas.values())}}
    an["problemas"] = problemas(an)
    return an, hist


def _letras(falhas: list[str]) -> str:
    return "".join(EXIGENCIAS[q][0] for q in EXIGENCIAS if q in falhas)


def problemas(an: dict) -> list[dict]:
    """Os problemas apontados automaticamente, os graves primeiro."""
    out = []

    def add(gravidade: str, texto: str) -> None:
        out.append({"gravidade": gravidade, "texto": texto})

    ex, s = an["exercicios"], an["saude"]
    st, t = s["status"], s["temperatura"]
    if st.get("estado") == "treinando" and not st.get("vivo"):
        add("grave", f"o processo do treino morreu (status.json treinando, pid {st.get('pid')} sem processo)")
    if st.get("estado") == "parado" and not t.get("pausado") and not an["treino"].startswith("arquivo"):
        add("alerta", f"o treino está parado fora da guarda térmica (status.json “parado”; última linha há "
            f"{s['ultima_linha_min']:.0f} min)")
    if st.get("vivo") and s["ultima_linha_min"] > 6:
        add("grave", f"o treino não escreve no log há {s['ultima_linha_min']:.0f} min")
    if s["nan"]:
        add("grave", f"NaN no núcleo do treino em {len(s['nan'])} linha(s) (última r{s['nan'][-1]})")
    for linha in s["erros"][:3]:
        add("grave", f"treino.out: {linha}")
    if t and t["idade_min"] > 3:
        add("grave", f"o vigia térmico não lê a temperatura há {t['idade_min']:.0f} min")
    if t.get("pausado"):
        add("alerta", f"o treino está parado pela guarda térmica ({t['paradas'][-1][1] if t['paradas'] else 'parada antiga'}); "
            "ela o religa sozinha ao esfriar")
    if t.get("cpu_max") is not None and t["cpu_max"] >= 80:
        add("alerta", f"CPU chegou a {t['cpu_max']:.0f} °C na janela (agora {t['cpu']:.0f})")
    if t.get("paradas"):
        add("alerta", f"{len(t['paradas'])} parada(s) térmica(s) na janela: "
            + ", ".join(f"{_hm(a)} {e}" for a, e in t["paradas"][-3:]))
    if s["lacunas"]:
        add("nota", f"{len(s['lacunas'])} parada(s) do treino na janela: "
            + ", ".join(f"{_hm(a)}–{_hm(b)} (~{p / 60:.0f} min)" for a, b, p in s["lacunas"][-4:]))
    v = s["vazao"]["variacao"]
    if v is not None and v < -0.15:
        add("alerta" if v < -0.3 else "nota", f"vazão {v:+.0%} contra a mediana da última hora "
            f"({s['vazao']['agora']:.0f} am/s)")
    kl, tx = s["kl"], s["taxa"]
    if kl["alvo"] and kl["loco"] and kl["loco"] > 2 * kl["alvo"]:
        add("alerta", f"KL da locomoção {kl['loco']:.3f}, mais que o dobro do alvo {kl['alvo']}")
    if tx["rec_piso"] and tx["rec_piso"] > 0.5:
        add("nota", f"a taxa da cabeça da recuperação fica no piso (1e-6) em {tx['rec_piso']:.0%} das linhas da última hora "
            f"(KL dela {_n(kl['rec'], 3)} contra o alvo {kl['alvo']}): ela quase não aprende")
    if tx["loco_piso"] and tx["loco_piso"] > 0.5:
        add("alerta", f"a taxa da cabeça da locomoção fica no piso em {tx['loco_piso']:.0%} das linhas da última hora")
    if s["ruido"]["loco"] is not None and s["ruido"]["loco"] <= RUIDO_PISO * 1.2:
        add("alerta", f"o ruído da locomoção caiu para {s['ruido']['loco']:.3f} (piso {RUIDO_PISO}): a exploração acabando")
    subindo = [k for k in an["ranking"] if ex[k]["subidas_1h"]]
    if subindo:
        add("alerta", "subiram na última hora: " + ", ".join(f"{nome(k)} ({ex[k]['subidas_1h']}×)" for k in subindo))
    parados = [k for k in an["ranking"] if ex[k]["sem_descer_min"] >= 60]
    if parados:
        add("alerta", "sem descer há 60 min ou mais: " + "; ".join(
            f"{nome(k)} {_pf(ex[k]['forca'])} {ex[k]['sem_descer_min']:.0f} min ({_letras(ex[k]['falhas']) or ex[k]['estado']})"
            for k in parados))
    for p in an["pares"]:
        piores = [m for m, v in p["medidas"].items() if v["nivel"] > 0]
        melhores = [NOMES[m] for m, v in p["medidas"].items() if v["nivel"] < 0]
        if piores:
            add("alerta" if p["veredito"] == "D pior" and p["nivel"] >= 2 else "nota",
                f"par {nome(p['esquerda'])} × {nome(p['direita'])} ({p['veredito']}): a direita pior em "
                + ", ".join(_dif(m, p["medidas"][m]) for m in piores)
                + (f" (a esquerda pior em {', '.join(melhores)})" if melhores else ""))
    for faixa, v in an["vies_rumo"].items():
        for q, txt in (("deriva", "escorrega para a"), ("giro", "gira para a")):
            if abs(v[q]) >= 1.0:
                add("alerta", f"{faixa}: o corpo {txt} {'esquerda' if v[q] > 0 else 'direita'} ({v[q]:+.1f}° de rumo, "
                    f"+ = esquerda; a parte assimétrica dos 8 rumos)")
    for p, (agora, antes) in ((k, v["sem_cair"]) for k, v in an["aval"].items()):
        if agora is not None and antes is not None and agora < antes - 0.15:
            add("alerta", f"avaliação sem a mão: {p} sem cair caiu de {antes:.0%} para {agora:.0%} em 1 h")
    lev = an["levantar"]["queda_"]["levantou"]
    if lev[0] is not None and (lev[0] < 0.9 or (lev[1] is not None and lev[0] < lev[1] - 0.05)):
        add("alerta", f"levantar das quedas: {lev[0]:.0%} levantam (1 h atrás {_pc(lev[1])})")
    ordem = {"grave": 0, "alerta": 1, "nota": 2}
    return sorted(out, key=lambda p: ordem[p["gravidade"]])


def _dif(m: str, v: dict) -> str:
    if m == "desvio":
        return f"desvio {v['E']:.1f}° → {v['D']:.1f}°"
    if m == "forca":
        return f"força {_pf(v['E'])} → {_pf(v['D'])}"
    return f"{NOMES[m]} {v['E']:.2f} → {v['D']:.2f}"


# --- O texto ---------------------------------------------------------------------------------------------------------

def _hm(t: float, dia: bool = False) -> str:
    return datetime.fromtimestamp(t).strftime("%d/%m %H:%M" if dia else "%H:%M")


def _pc(x) -> str:
    return "—" if x is None else f"{x * 100:.0f}%"


def _pf(x) -> str:
    """A forca em %, com uma casa quando ha: o passo de teste de 0,25% nao some (99,75% nao vira 100%)."""
    return "—" if x is None else f"{x * 100:.1f}".removesuffix(".0") + "%"


def _n(x, casas: int = 2) -> str:
    return "—" if x is None else f"{x:.{casas}f}"


def _nq(q: str, x) -> str:
    """Um valor de uma exigencia: as fracoes com 3 casas (o log guarda 3; 2 escondem o quase), graus e segundos com 1."""
    return _n(x, 1 if q in ("tempo", "desvio") else 3)


def _bloqueio(b: dict) -> str:
    q = b["exigencia"]
    sinal = ">" if q in ("tempo", "desvio", "cruzado") else "<"
    return f"{EXIGENCIAS[q][2]} {_nq(q, b['valor'])}{sinal}{_nq(q, b['exigido'])} ({b['ha_min']:.0f} min)"


def _segura(e: dict) -> str:
    if e["estado"] == "desceu":
        return "descendo"
    if e["estado"] == "aguardando":
        txt = "espera depois da subida" if e["aguardando"] == "espera" else "aguardando amostra"
        return txt + (f" (antes: {_letras(e['falhas'])})" if e["falhas"] else "")
    if e["estado"] == "indefinido":
        return "nada visível (P/marca?)"
    return "; ".join(_bloqueio(b) for b in e["bloqueios"]) or e["estado"]


def _eta_txt(e: dict) -> str:
    return "—" if e["eta_h"] is None else "> 2 d" if e["eta_h"] > 48 else f"{e['eta_h']:.1f} h"


def _celula(e: dict, q: str) -> str:
    """valor/exigido de uma medida, em negrito quando falha."""
    par = _valor_exigido(q, e["medidas"], e["exigencias"])
    if par[0] is None or (q == "desvio" and par[1] is None):
        return "—"
    txt = f"{_nq(q, par[0])}/{_nq(q, par[1])}"
    return f"**{txt}**" if q in e["falhas"] else txt


def markdown(an: dict) -> str:
    ex, s, rg = an["exercicios"], an["saude"], an["regua"]
    L = [f"# Painel {an['treino']} — {_hm(an['hora'], True)}, r{an['atualizacao']}", ""]
    if an["problemas"]:
        L += ["**Problemas (automático)**", ""] + [f"- **{p['gravidade']}** {p['texto']}" for p in an["problemas"]] + [""]
    jm = rg["janela_min"]
    L += [f"**Régua:** {len(ex)} exercícios, {rg['zerados']} a 0%; força média {rg['forca_media']:.0%} (há {jm:.0f} min: "
          f"{rg['forca_media_ini']:.0%}); na última hora {rg['descidas_1h']} descidas e {rg['subidas_1h']} subidas.", "",
          "## Ranking de dificuldade", "",
          f"| # | exercício | força (há {jm:.0f} min) | %/h | sem descer | o que segura agora | subidas 1 h | ETA 0% |",
          "|---|---|---|---|---|---|---|---|"]
    for i, k in enumerate(an["ranking"][:12], 1):
        e = ex[k]
        L.append(f"| {i} | {nome(k)} | {_pf(e['forca'])} ({_pf(e['forca_ini'])}) | {_n(e['taxa_h'], 1)} | "
                 f"{'≥ ' if e['nunca_desceu'] else ''}{e['sem_descer_min']:.0f} min | {_segura(e)} | {e['subidas_1h']} | "
                 f"{_eta_txt(e)} |")
    L += ["", "## Pares espelhados (E = esquerda, D = direita; o clipe de um é o espelho do outro)", "",
          "| par | força E/D | semelhança E/D | fim E/D | estab. E/D | desvio E/D (°) | rumo E/D (°, + = esq.) | viés | veredito |",
          "|---|---|---|---|---|---|---|---|---|"]
    for p in an["pares"]:
        m = p["medidas"]
        cel = lambda q, f="{:.2f}".format: f"{f(m[q]['E'])}/{f(m[q]['D'])}" if q in m else "—"  # noqa: E731
        r = p["rumo"]
        piores = [NOMES[q] for q, v in m.items() if v["nivel"] > 0]
        melhores = [NOMES[q] for q, v in m.items() if v["nivel"] < 0]
        veredito = (f"**{p['veredito']}**" if p["veredito"] == "D pior" else p["veredito"]) + "".join(
            ([f"; D pior em {', '.join(piores)}"] if piores else []) + ([f"; E pior em {', '.join(melhores)}"] if melhores else []))
        L.append(f"| {nome(p['esquerda'])} / {nome(p['direita'])} | {cel('forca', _pf)} | {cel('semelhanca')} | "
                 f"{cel('fim')} | {cel('estabilidade')} | {cel('desvio', '{:.1f}'.format)} | {_n(r['E'], 1)}/{_n(r['D'], 1)} | "
                 f"{_n(r['vies'], 1)} | {veredito} |")
    for faixa, v in an["vies_rumo"].items():
        L.append(f"\n**Viés de rumo, {faixa}:** giro {v['giro']:+.2f}° e deriva lateral {v['deriva']:+.2f}° (+ = para a "
                 f"esquerda, − = para a direita; a parte assimétrica dos 8 rumos). Desvio por rumo 0–7: "
                 + ", ".join(f"{x:+.1f}" for x in v["desvio"]) + ".")
    tp, kl, tx = s["temperatura"], s["kl"], s["taxa"]
    st = s["status"]
    L += ["", "## Saúde", "",
          f"- última linha há {s['ultima_linha_min']:.1f} min; cadência {s['cadencia_s'] / 60:.1f} min; paradas na janela: "
          + (", ".join(f"{_hm(a)}–{_hm(b)} (~{p / 60:.1f} min)" for a, b, p in s["lacunas"]) or "nenhuma"),
          f"- status: {st.get('estado', '?')} (pid {st.get('pid')} {'vivo' if st.get('vivo') else 'sem processo'})",
          f"- vazão {_n(s['vazao']['agora'], 0)} am/s (mediana 1 h {_n(s['vazao']['hora'], 0)}, "
          f"{'—' if s['vazao']['variacao'] is None else format(s['vazao']['variacao'], '+.0%')}); física {_n(s['t_fisica'])} s",
          f"- KL loco {_n(kl['loco'], 3)}, rec {_n(kl['rec'], 3)} (alvo {kl['alvo']}); taxa loco {_n(tx['loco'], 7)}, "
          f"rec {_n(tx['rec'], 7)} (no piso: loco {_pc(tx['loco_piso'])}, rec {_pc(tx['rec_piso'])}; a rec parada em "
          f"{_pc(tx['rec_parada'])} das atualizações)",
          f"- ruído (desvio da política): loco {_n(s['ruido']['loco'], 3)}, rec {_n(s['ruido']['rec'], 3)}",
          f"- temperatura: CPU {_n(tp.get('cpu'), 0)} °C agora (máx. na janela {_n(tp.get('cpu_max'), 0)}), GPU "
          f"{_n(tp.get('gpu'), 0)}; paradas térmicas na janela: {len(tp.get('paradas', []))}"
          + ("; **o treino está parado pela guarda térmica**" if tp.get("pausado") else ""),
          f"- NaN no núcleo: {len(s['nan'])} linha(s); treino.out: "
          + ("; ".join(s["erros"][:3]) if s["erros"] else "sem erros desde o último começo"), "",
          "## Levantar (entre parênteses, 1 h atrás)", "",
          "| começo | levantou | de pé no fim | tempo (s) | caiu depois | bruços | costas | lado |", "|---|---|---|---|---|---|---|---|"]
    for p, txt in (("queda_", "da queda"), ("clipe_", "do clipe")):
        v = an["levantar"][p]
        L.append(f"| {txt} | " + " | ".join(
            f"{_n(v[k][0], 2)} ({_n(v[k][1], 2)})" for k in ("levantou", "de_pe", "tempo", "caiu_de_pe", "brucos", "costas",
                                                            "lado")) + " |")
    lv = ex.get("lev_levantar")
    if lv:
        L.append(f"\nlev_levantar: força {_pf(lv['forca'])}, {_segura(lv)}; fim {_celula(lv, 'fim')}, semelhança "
                 f"{_celula(lv, 'semelhanca')}, estabilidade {_celula(lv, 'estabilidade')}, tempo {_celula(lv, 'tempo')} s "
                 "(valor/exigido na última decisão).")
    L += ["", "## Avaliação sem a mão (0%, linhas aval; entre parênteses 1 h atrás)", "",
          "| faixa | sem cair | duração média (s) |", "|---|---|---|"]
    for p, v in an["aval"].items():
        if v["sem_cair"][0] is not None:
            L.append(f"| {p} | {_pc(v['sem_cair'][0])} ({_pc(v['sem_cair'][1])}) | {_n(v['seg'][0], 1)} ({_n(v['seg'][1], 1)}) |")
    L += ["", "## Rastreio (média da coleta; entre parênteses 1 h atrás)", "",
          " · ".join(f"{k} {_n(a, 3)} ({_n(b, 3)})" for k, (a, b) in an["rastreio"].items() if a is not None), "",
          "## Todos os exercícios", "",
          "| exercício | força | 30 min | 60 min | %/h | sem descer | estado | fim | vel/tempo | semelhança | estab. | desvio | rumo (°) |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for k, e in ex.items():
        L.append(f"| {nome(k)} | {_pf(e['forca'])} | {_pf(e['forca_30'])} | {_pf(e['forca_60'])} | {_n(e['taxa_h'], 1)} | "
                 f"{'≥ ' if e['nunca_desceu'] else ''}{e['sem_descer_min']:.0f} min | {e['estado']} "
                 f"{_letras(e['falhas'])} | {_celula(e, 'fim')} | "
                 f"{_celula(e, 'tempo') if k == 'lev_levantar' else _celula(e, 'vel')} | {_celula(e, 'semelhanca')} | "
                 f"{_celula(e, 'estabilidade')} | {_celula(e, 'desvio')} | {_n(e['rumo'], 1)} |")
    L += ["", "Notas: “sem descer” conta desde a última descida (≥ quando o log não tem nenhuma); o que segura é a simulação "
          "da régua (levantar_amp.dynamic_assist) com os mínimos do log; P contra a marca é estimado (a marca de verdade fica "
          "no checkpoint); o ETA usa a taxa da última hora, com o passo pela metade abaixo de 30%. O treino condensa as "
          "linhas com mais de 10 min (grupos de 1, 3, 6, 12 min...: fica a última decisão de cada grupo), então subidas "
          "antigas no meio de um grupo não aparecem.", ""]
    return "\n".join(L)


# --- Os graficos -----------------------------------------------------------------------------------------------------

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 8, "axes.edgecolor": "#c3c2b7", "axes.labelcolor": TINTA2, "xtick.color": TINTA2,
                         "ytick.color": TINTA2, "figure.facecolor": FUNDO, "axes.facecolor": FUNDO,
                         "savefig.facecolor": FUNDO})
    return plt


def _tempo(ts) -> np.ndarray:
    import matplotlib.dates as md
    return md.date2num([datetime.fromtimestamp(t) for t in ts])


def _principal(h: dict) -> str:
    return next(q for q in EXIGENCIAS if q in h["falhas"]) if h["estado"] == "segura" else h["estado"]


def _cores():
    from matplotlib.colors import BoundaryNorm, ListedColormap
    cores = {**{k: v[1] for k, v in ESTADOS.items()}, **{k: v[1] for k, v in EXIGENCIAS.items()}}
    cmap = ListedColormap([cores[c] for c in CODIGOS])
    cmap.set_bad(alpha=0.0)
    return cmap, BoundaryNorm(np.arange(len(CODIGOS) + 1) - 0.5, len(CODIGOS)), cores


def _celulas(hs: list[dict], paradas: set, cad: float, t1: float) -> tuple[list, np.ndarray]:
    """As celulas no tempo de cada decisao (a forca dela vale ate a seguinte): as bordas e os codigos, com uma celula
    vazia (nan) nas paradas do treino."""
    bordas, cod = [hs[0]["hora"]], []
    for i, h in enumerate(hs):
        prox = hs[i + 1]["hora"] if i + 1 < len(hs) else max(t1, h["hora"] + cad)
        fim = min(prox, h["hora"] + cad) if h["hora"] in paradas else prox
        bordas.append(fim)
        cod.append(CODIGOS.index(_principal(h)))
        if fim < prox:
            bordas.append(prox)
            cod.append(np.nan)
    return bordas, np.array(cod, float)


def _janela(hist: dict, t0: float) -> dict:
    """Os historicos na janela, com a decisao de antes dela (a forca no comeco)."""
    out = {}
    for k, hs in hist.items():
        i = next((j for j, h in enumerate(hs) if h["hora"] >= t0), len(hs))
        out[k] = hs[max(0, i - 1):]
    return out


def _legenda(fig, cores: dict, y: float, ncol: int = 6) -> None:
    from matplotlib.patches import Patch
    itens = [Patch(color=cores[k], label=f"{v[0]} {v[2]}") for k, v in EXIGENCIAS.items() if k != "tempo"] + [
        Patch(color=cores[k], label=(f"{v[0]} " if v[0] else "") + v[2]) for k, v in ESTADOS.items()]
    fig.legend(handles=itens, loc="upper center", bbox_to_anchor=(0.5, y), ncol=ncol, frameon=False, fontsize=8,
               handlelength=1.4, columnspacing=1.4)


def grafico_forcas(an: dict, hist: dict, caminho: Path) -> None:
    """Pequenos multiplos: a forca de cada exercicio no tempo (a linha) sobre o fundo do estado de cada decisao (a
    exigencia principal que segurou, desceu, subiu, aguardando)."""
    import matplotlib.dates as md
    plt = _plt()
    t1, cad = an["hora"], an["saude"]["cadencia_s"]
    paradas = {a for a, _, _ in an["saude"]["lacunas"]}
    t0 = an["inicio"]
    hist = _janela(hist, t0)
    cmap, norm, cores = _cores()
    cols = 8
    linhas = -(-len(hist) // cols)
    fig, axs = plt.subplots(linhas, cols, figsize=(2.3 * cols, 1.85 * linhas + 1.0), sharex=True, sharey=True,
                            squeeze=False)
    for ax, (k, hs) in zip(axs.flat, hist.items()):
        e = an["exercicios"][k]
        bordas, cod = _celulas(hs, paradas, cad, t1)
        ax.pcolormesh(_tempo(bordas), [0, 105], cod[None, :], cmap=cmap, norm=norm, alpha=0.4, shading="flat")
        ax.step(_tempo([h["hora"] for h in hs] + [t1]), [h["forca"] * 100 for h in hs] + [hs[-1]["forca"] * 100],
                where="post", color=TINTA, lw=1.4)
        # (Acabou de descer sem a hora descer - as subidas a anulam -: o saldo, nao "parado 0 min".)
        taxa = e["taxa_h"] or 0.0
        resumo = (f"↓ {taxa:.1f}%/h" if taxa > 0.05 else f"parado {e['sem_descer_min']:.0f} min {_letras(e['falhas'])}"
                  if e["sem_descer_min"] >= 1 else f"sobe {-taxa:.1f}%/h" if taxa < -0.05 else "↓ agora")
        ax.set_title(f"{nome(k)}  {_pf(e['forca'])}\n{resumo}" + (f"  ↑{e['subidas_1h']}×" if e["subidas_1h"] else ""),
                     loc="left", fontsize=8, color=TINTA)
        ax.grid(axis="y", color=GRADE, lw=0.6)
        ax.set_axisbelow(True)
    for i, ax in enumerate(axs.flat):
        if i >= len(hist):
            ax.axis("off")
        elif i + cols >= len(hist):
            ax.tick_params(axis="x", labelbottom=True)  # (sem quadro embaixo: as horas aqui)
    ax = axs.flat[0]
    ax.set_ylim(0, 105)
    ax.set_yticks([0, 50, 100])
    ax.set_xlim(*_tempo([t0, t1]))
    ax.xaxis.set_major_locator(md.AutoDateLocator(minticks=2, maxticks=4))
    ax.xaxis.set_major_formatter(md.DateFormatter("%H:%M"))
    fig.suptitle(f"{an['treino']} — força auxiliar por exercício (%), de {_hm(t0)} a {_hm(t1, True)}; "
                 "o fundo é o estado de cada decisão da régua", x=0.01, ha="left", y=0.995, fontsize=10)
    _legenda(fig, cores, 0.965, ncol=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94), h_pad=0.6, w_pad=0.4)
    fig.savefig(caminho, dpi=110)
    plt.close(fig)


def grafico_bloqueios(an: dict, hist: dict, caminho: Path) -> None:
    """O mapa exercicio x tempo: a cor e a exigencia principal que segurou cada decisao (ou desceu, subiu, aguardando); as
    letras de todas as que falharam no comeco de cada trecho igual."""
    import matplotlib.dates as md
    plt = _plt()
    t1, cad = an["hora"], an["saude"]["cadencia_s"]
    paradas = {a for a, _, _ in an["saude"]["lacunas"]}
    t0 = an["inicio"]
    hist = _janela(hist, t0)
    cmap, norm, cores = _cores()
    n = len(hist)
    fig, ax = plt.subplots(figsize=(15, 0.3 * n + 1.9))
    largura_min = (t1 - t0) / 45
    for i, (k, hs) in enumerate(hist.items()):
        bordas, cod = _celulas(hs, paradas, cad, t1)
        ax.pcolormesh(_tempo(bordas), [i, i + 1], cod[None, :], cmap=cmap, norm=norm, shading="flat")
        trechos = []
        for h in hs:
            rotulo = _letras(h["falhas"]) if h["estado"] == "segura" else ESTADOS[h["estado"]][0]
            if not trechos or trechos[-1][1] != rotulo or trechos[-1][2] != _principal(h):
                trechos.append([max(h["hora"], t0), rotulo, _principal(h)])
        for j, (ini, rotulo, principal) in enumerate(trechos):
            fim = trechos[j + 1][0] if j + 1 < len(trechos) else t1
            if rotulo and fim - ini >= largura_min:
                rgb = np.array([int(cores[principal][c:c + 2], 16) for c in (1, 3, 5)]) / 255
                escuro = (rgb @ [0.2126, 0.7152, 0.0722]) < 0.45
                ax.text(_tempo([ini])[0], i + 0.5, " " + rotulo, va="center", ha="left", fontsize=7,
                        color="white" if escuro else TINTA, clip_on=True)
    ax.set_yticks(np.arange(n) + 0.5)
    ax.set_yticklabels([f"{nome(k)}  {_pf(an['exercicios'][k]['forca'])}" for k in hist], fontsize=8)
    ax.set_ylim(n, 0)
    ax.set_xlim(*_tempo([t0, t1]))
    ax.xaxis.set_major_locator(md.AutoDateLocator(minticks=4, maxticks=12))
    ax.xaxis.set_major_formatter(md.DateFormatter("%H:%M"))
    for y in (8, 16, 24):
        if y < n:
            ax.axhline(y, color=TINTA2, lw=0.8)
    ax.tick_params(axis="y", length=0)
    fig.suptitle(f"{an['treino']} — o que segura cada exercício, decisão por decisão (de {_hm(t0)} a {_hm(t1, True)}; "
                 "a força de agora ao lado do nome)", x=0.01, ha="left", fontsize=10)
    _legenda(fig, cores, 0.985 - 0.35 / (0.3 * n + 1.9), ncol=11)
    fig.tight_layout(rect=(0, 0, 1, 1 - 0.75 / (0.3 * n + 1.9)))
    fig.savefig(caminho, dpi=110)
    plt.close(fig)


def grafico_espelhos(an: dict, hist: dict, caminho: Path) -> None:
    """Os pares espelhados: cada medida, a esquerda (azul) e a direita (laranja) ligadas, a ligacao vermelha com a direita
    pior; o rumo assinado (numa rede simetrica os dois ficam espelhados em torno do zero); embaixo, a diferenca de forca
    direita - esquerda no tempo (acima de zero, a direita precisa de mais mao)."""
    plt = _plt()
    import matplotlib.dates as md
    ps = an["pares"]
    if not ps:
        return
    t1 = an["hora"]
    t0 = an["inicio"]
    azul, laranja, critico = "#2a78d6", "#eb6834", "#d03b3b"
    fig = plt.figure(figsize=(16, 8.2))
    gs = fig.add_gridspec(2, 6, height_ratios=[1.15, 1], hspace=0.45, wspace=0.12)
    medidas = [("forca", "força (%)", 100), ("semelhanca", "semelhança", 1), ("fim", "chegar ao fim", 1),
               ("estabilidade", "estabilidade", 1), ("desvio", "|desvio de rumo| (°)", 1), ("rumo", "rumo (°, + = esq.)", 1)]
    for j, (m, titulo, esc) in enumerate(medidas):
        ax = fig.add_subplot(gs[0, j])
        for i, p in enumerate(ps):
            if m == "rumo":
                a, b, nivel = p["rumo"]["E"], p["rumo"]["D"], 0
            else:
                v = p["medidas"].get(m)
                a, b, nivel = (v["E"], v["D"], v["nivel"]) if v else (None, None, 0)
            if a is None or b is None:
                continue
            ax.plot([a * esc, b * esc], [i, i], color=critico if nivel > 0 else "#c3c2b7", lw=2.6 if nivel > 0 else 1.6,
                    zorder=1, solid_capstyle="round")
            ax.scatter([a * esc], [i], s=52, color=azul, zorder=3, edgecolors=FUNDO, linewidths=1.5)
            ax.scatter([b * esc], [i], s=52, color=laranja, zorder=3, edgecolors=FUNDO, linewidths=1.5)
            if m == "rumo" and p["rumo"]["vies"] is not None:
                ax.scatter([p["rumo"]["vies"]], [i], marker="|", s=140, color=TINTA, zorder=4)
        if m == "rumo":
            ax.axvline(0, color=TINTA2, lw=0.8)
        ax.set_title(titulo, fontsize=9, loc="left", color=TINTA)
        ax.set_yticks(range(len(ps)))
        ax.set_yticklabels([f"{nome(p['esquerda']).split()[0]} {p['esquerda'][-1]}/{p['direita'][-1]}"
                            + ("" if p["veredito"] == "iguais" else f"  {p['veredito']}") for p in ps] if j == 0 else [],
                           fontsize=8)
        for lab, p in zip(ax.get_yticklabels(), ps):
            if p["veredito"] == "D pior":
                lab.set_color(critico)
                lab.set_fontweight("bold")
        ax.set_ylim(len(ps) - 0.5, -0.5)
        ax.grid(axis="x", color=GRADE, lw=0.6)
        ax.set_axisbelow(True)
        ax.tick_params(axis="y", length=0)
    from matplotlib.lines import Line2D
    fig.legend(handles=[Line2D([], [], marker="o", ls="", color=azul, label="esquerda (E)"),
                        Line2D([], [], marker="o", ls="", color=laranja, label="direita (D)"),
                        Line2D([], [], color=critico, lw=2.6, label="direita pior"),
                        Line2D([], [], marker="|", ls="", color=TINTA, markersize=10, label="viés (média E, D)")],
               loc="upper right", ncol=4, frameon=False, fontsize=8, bbox_to_anchor=(0.995, 0.995))
    serie = ["#2a78d6", "#eb6834", "#1baf7a"]
    for j, faixa in enumerate(("trote", "corrida")):
        ax = fig.add_subplot(gs[1, 3 * j:3 * j + 3])
        for c, p in zip(serie, [p for p in ps if faixa in p["esquerda"]]):
            fe = {h["hora"]: h["forca"] for h in hist[p["esquerda"]] if h["hora"] >= t0}
            fd = [(t, h["forca"] - fe[t]) for h in hist[p["direita"]] if (t := h["hora"]) in fe]
            if not fd:
                continue
            x = _tempo([t for t, _ in fd])
            y = [v * 100 for _, v in fd]
            ax.plot(x, y, color=c, lw=2, label=f"{p['esquerda'][-1]}/{p['direita'][-1]}")
            ax.annotate(f"{p['esquerda'][-1]}/{p['direita'][-1]}  {y[-1]:+.0f}", (x[-1], y[-1]), xytext=(4, 0),
                        textcoords="offset points", va="center", fontsize=8, color=TINTA)
        ax.axhline(0, color=TINTA2, lw=0.8)
        ax.set_xlim(*_tempo([t0, t1 + (t1 - t0) * 0.08]))
        # (Os x sao numeros do date2num: sem o localizador de datas as marcas caem em minutos quebrados.)
        ax.xaxis.set_major_locator(md.AutoDateLocator(minticks=4, maxticks=10))
        ax.xaxis.set_major_formatter(md.DateFormatter("%H:%M"))
        ax.set_title(f"{faixa}: força D − E (pontos %; acima de zero a direita precisa de mais mão)", fontsize=9, loc="left",
                     color=TINTA)
        ax.grid(color=GRADE, lw=0.6)
        ax.set_axisbelow(True)
        ax.legend(frameon=False, fontsize=8, loc="upper left")
    fig.suptitle(f"{an['treino']} — pares espelhados em {_hm(t1, True)} (o clipe de um é o espelho do outro: a diferença é "
                 "viés da rede)", x=0.01, ha="left", y=0.995, fontsize=10)
    fig.savefig(caminho, dpi=110, bbox_inches="tight")
    plt.close(fig)


# --- A saida ---------------------------------------------------------------------------------------------------------

def _limpo(x):
    if isinstance(x, float):
        return round(x, 5) if np.isfinite(x) else None
    if isinstance(x, dict):
        return {k: _limpo(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_limpo(v) for v in x]
    return x


def gerar(treino: str, saida: Path, horas: float = 3.0, log=print) -> dict:
    """Le o log do treino e escreve o painel em `saida`. Devolve a analise."""
    run = comum.run_dir(treino)
    linhas, _ = ler_log(run / "log.jsonl")
    res = analisar(linhas, run, horas)
    if res is None:
        raise SystemExit(f"{run / 'log.jsonl'}: nenhuma linha com a forca por exercicio")
    an, hist = res
    saida.mkdir(parents=True, exist_ok=True)
    (saida / "painel.md").write_text(markdown(an))
    (saida / "painel.json").write_text(json.dumps(_limpo(an), ensure_ascii=False, indent=1))
    grafico_forcas(an, hist, saida / "forcas.png")
    grafico_bloqueios(an, hist, saida / "bloqueios.png")
    grafico_espelhos(an, hist, saida / "espelhos.png")
    log(f"painel de {treino} (r{an['atualizacao']}, {len(linhas)} linhas) em {saida}")
    return an


def _teste() -> None:
    # (1) A regua num log sintetico: segura por semelhanca, desce, aguarda amostra, sobe e espera.
    exi = {"fim": 1.0, "vel": 0.72, "semelhanca": 0.7, "estabilidade": 0.6}
    ok = {"fim": 1.0, "vel": 0.9, "semelhanca": 0.8, "estabilidade": 0.8, "P": 0.9, "sem_teto": 0.9, "exigido": 0.8,
          "vel_exigida": 0.72, "estab_exigida": 0.6}
    mau = dict(ok, sem_teto=0.7, semelhanca=0.62)
    seq = [(0, 1.0, mau), (60, 0.99, ok), (120, 0.99, mau), (180, 0.99, None), (240, 0.99, mau), (300, 1.0, dict(ok, fim=0.8)),
           (360, 1.0, ok)]
    rows = [{"hora": t, "atualizacao": t, "ajuda_exercicios": {"x": f}, "dominio": {"x": d} if d else {},
             "exigencias": {"x": exi}} for t, f, d in seq]
    h = historico(rows, "x")
    assert [x["estado"] for x in h] == ["segura", "desceu", "segura", "aguardando", "segura", "subiu", "aguardando"], h
    assert h[4]["falhas"] == ["semelhanca"] and h[5]["falhas"] == ["fim"] and h[6]["motivo"] == "espera"
    e = _exercicio(h[:5], 240, None)
    assert e["bloqueios"][0]["exigencia"] == "semelhanca" and abs(e["bloqueios"][0]["ha_min"] - 2.0) < 1e-9, e
    e = _exercicio(h, 360, None)
    assert e["sem_descer_min"] == 5.0 and e["subidas_1h"] == 1 and e["aguardando"] == "espera" and e["bloqueios"] == []
    assert _eta(0.5, 10.0) == 2.0 + 6.0 and _eta(0.2, 10.0) == 2.0 and _eta(0.5, 0.0) is None
    # (Uma subida sem o motivo visivel - a marca reconstruida abaixo da de verdade - tambem arma a espera.)
    rows = [{"hora": t, "atualizacao": t, "ajuda_exercicios": {"x": f}, "dominio": {"x": ok}, "exigencias": {"x": exi}}
            for t, f in ((0, 0.5), (60, 0.52), (120, 0.52))]
    assert [x["estado"] for x in historico(rows, "x")] == ["indefinido", "subiu", "aguardando"]
    assert _pf(0.9975) == "99.8%" and _pf(0.61) == "61%" and _pf(None) == "—"
    # (O vies de rumo, + = esquerda: o corpo escorregando para a direita desvia para a direita indo para a frente e para a
    # esquerda indo para tras; um giro para a esquerda desvia todos os rumos; a parte espelhada - a do clipe - some.)
    th = np.radians(45.0 * np.arange(8))
    v = vies_rumo({f"lev_trote_{h}": {"desvio_rumo": x} for h, x in enumerate(1.0 - 2.0 * np.cos(th) + 3.0 * np.sin(th))})
    assert abs(v["trote"]["giro"] - 1.0) < 1e-9 and abs(v["trote"]["deriva"] + 2.0) < 1e-9 and "corrida" not in v, v
    # (O par: a direita com mais forca e menos semelhanca e pior, nivel 2; o vies e a media dos rumos, aqui a direita.)
    p, = pares({"lev_trote_2": {"forca": 0.5, "medidas": {"semelhanca": 0.8}, "rumo": 0.5},
                "lev_trote_6": {"forca": 0.7, "medidas": {"semelhanca": 0.7}, "rumo": -1.5}})
    assert p["veredito"] == "D pior" and p["nivel"] == 2 and p["rumo"]["vies"] == -0.5, p
    # (A parada: 11 min entre duas linhas de 20 atualizacoes a 3 s cada.)
    rows = [{"hora": 60.0 * i + (600.0 if i >= 5 else 0.0), "atualizacao": 20 * i} for i in range(10)]
    assert lacunas(rows, 0.0) == [(240.0, 900.0, 600.0)], lacunas(rows, 0.0)
    # (O erro do processo de agora mesmo com as linhas [INFO] soltas depois do Traceback; o religado que rodou o apaga; o
    # NaN do nucleo conta, o do estilo nao.)
    base = comum.REPO / "build-profile/tmp/inspecao_dev/painel"
    falso = base / "treino_falso"
    falso.mkdir(parents=True, exist_ok=True)
    caiu = ("atualizacao=10 kl=0.03 estilo=nan\nTraceback (most recent call last):\n  File \"x.py\", line 1\nRuntimeError: "
            "boom\n[INFO] Backend fisico Jolt Physics 5.6.0 inicializado\n")
    (falso / "treino.out").write_text(caiu)
    assert [x for x in erros_treino(falso)] == ["Traceback (most recent call last):", "RuntimeError: boom"]
    (falso / "treino.out").write_text(caiu + "atualizacao=20 kl=0.03 estilo=nan\n")
    assert erros_treino(falso) == []
    (falso / "treino.out").write_text(caiu + "atualizacao=20 kl=nan estilo=nan\n")
    assert erros_treino(falso) == ["atualizacao=20 kl=nan estilo=nan"]
    # (2) O log de agora e um arquivado: as forcas sao as da ultima linha; as decisoes com amostra tem o estado e, quando
    # seguram, o que falhou; a simulacao explica quase todas as que seguraram.
    for treino in ("rl_final_clipe", "rl_final_clipe/arquivo_antes_reset_reto_0626_0511"):
        run = comum.run_dir(treino)
        linhas, pos = ler_log(run / "log.jsonl")
        assert pos == (run / "log.jsonl").stat().st_size or pos > 0
        an, hist = analisar(linhas, run, 3.0)
        ultima = [r for r in linhas if "ajuda_exercicios" in r][-1]
        assert {k: v["forca"] for k, v in an["exercicios"].items()} == ultima["ajuda_exercicios"]
        assert set(an["ranking"]) == {k for k, v in ultima["ajuda_exercicios"].items() if v > 0}
        for k in ultima["dominio"]:
            ult = hist[k][-1]
            assert ult["d"] is not None and ult["estado"] in (*ESTADOS, "segura"), (k, ult)
            assert ult["estado"] != "segura" or ult["falhas"], (k, ult)
            if ult["estado"] == "segura":
                assert [b["exigencia"] for b in an["exercicios"][k]["bloqueios"]] == ult["falhas"]
        seguras = [x for hs in hist.values() for x in hs if x["estado"] in ("segura", "indefinido")]
        sem_motivo = sum(x["estado"] == "indefinido" for x in seguras) / max(1, len(seguras))
        print(f"{treino}: {len(linhas)} linhas, {len(seguras)} decisoes seguradas com amostra, {sem_motivo:.1%} sem exigencia"
              f" visivel; problemas {len(an['problemas'])}")
        assert sem_motivo < 0.10, sem_motivo
        assert {p["esquerda"] for p in an["pares"]} == {a for a, _ in comum.MIRROR_PAIRS}
        saida = base / ("teste_" + run.name)
        gerar(treino, saida)
        for arq in ("painel.md", "painel.json", "forcas.png", "bloqueios.png", "espelhos.png"):
            assert (saida / arq).stat().st_size > 1000, arq
    print("painel: teste ok")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--treino", default="rl_final_clipe", help="o treino (uma pasta de runs; arquivado: treino/arquivo_...)")
    ap.add_argument("--saida", help="a pasta do painel")
    ap.add_argument("--horas", type=float, default=3.0, help="a janela dos graficos, das lacunas e das temperaturas")
    ap.add_argument("--teste", action="store_true")
    a = ap.parse_args()
    if a.teste:
        _teste()
        return
    if not a.saida:
        ap.error("--saida e obrigatorio")
    gerar(a.treino, Path(a.saida), a.horas)


if __name__ == "__main__":
    main()
