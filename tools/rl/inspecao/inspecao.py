"""O orquestrador da inspecao (05/11): a checagem de hora em hora numa chamada so - o painel do log, as capturas do
movimento e as folhas com o jogador de verdade (com o treino parado: a regra termica), a marcha, o fantasma e o auditor
(com o treino religado), e o relatorio com o antes -> agora da inspecao anterior.

O fluxo:
  1. o painel (inspecao.painel, so le o log) e os exercicios: os TOPO primeiros do ranking de dificuldade, os espelhos
     deles e o NUCLEO (ou --exercicios), em cada modo de --modos;
  2. a regra termica, conferida na hora de cada passo pesado: o treino ativo e parado (comum.stop_training) - no comeco e
     de novo se voltou no meio (o vigia retoma o que parou quando a CPU esfria, o systemd o que caiu) -, e a CPU abaixo
     de ESFRIAR_C (comum.wait_cool, ate 5 min); sem esfriar ou sem leitura, os pesados acabam ali (o treino nao fica
     parado esperando). Os pesados: as capturas (inspecao.captura, o simulador) e as folhas do MatterModelInspector
     (quadros: as poses da folha do fantasma - uma passada desde o pouso do pe esquerdo, ou espacadas pelo movimento -,
     de frente e do lado direito do corpo; as capturas da rede, o mundo 0);
  3. religa o que parou logo depois, num finally (o erro, o Ctrl-C e o SIGTERM/SIGHUP tambem religam; comum.start_training
     confirma a linha nova no log), salvo --manter-parado;
  4. os leves, em nice 10 e nos nucleos 0-1: a marcha (e as comparacoes contra o clipe puro e o espelho, os .json
     __contra__ e __espelho__), o fantasma (fantasma.padrao de cada exercicio, com os espelhos), o antes x agora (a folha
     de cada captura da rede com a mesma da inspecao anterior por cima: antes nas duas primeiras linhas, agora nas duas
     ultimas, na mesma fase) e o auditor (--anterior: o da inspecao em <saida>/ultima);
  5. <saida>/<AAAAmmdd_HHMM>/index.md (o painel, os piores achados, cada exercicio com os numeros - entre parenteses os da
     anterior -, os achados e as figuras, as falhas, o registro termico e os tempos) e o ponteiro <saida>/ultima (so
     quando o auditor rodou: e a base da proxima).
A pasta: painel/, capturas/ (os .npz; no --simular, links para os da PASTA), capturas/fantasma/, folhas/, marcha/,
antes_agora/, auditor/ e index.md.

Uso (de dentro de tools/rl; na checagem de verdade sem nice nem taskset: as capturas e as folhas rodam com o treino
parado, e o orquestrador mesmo vai para nice 10 e os nucleos 0-1 ao religar; e sem um timeout que mate com SIGKILL - ele
nao passa pelo finally e o treino fica parado):
  .venv/bin/python -m inspecao.inspecao [--exercicios ...] [--modos atual 0.3 clipe] [--mundos 4] [--segundos 8]
                                        [--sem-folhas] [--manter-parado] [--saida PASTA]
  .venv/bin/python -m inspecao.inspecao --so-painel               (so o painel e o relatorio; nao para o treino)
  .venv/bin/python -m inspecao.inspecao --simular PASTA_CAPTURAS  (sem parar, capturar, folhas nem religar)
  .venv/bin/python -m inspecao.inspecao --teste                   (o --simular nas amostras, em inspecao_dev/orquestrador)
"""
from __future__ import annotations

import argparse
import sys
import io
import json
import os
import re
import signal
import subprocess
import time
from contextlib import contextmanager, redirect_stderr
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from inspecao import auditor as A
from inspecao import captura
from inspecao import comum
from inspecao import fantasma as F
from inspecao import geometria as G
from inspecao import marcha as M
from inspecao import painel as P

TREINO = "rl_final_clipe"
SAIDA = comum.REPO / "build-profile/tmp/inspecao"
SAIDA_TESTE = comum.REPO / "build-profile/tmp/inspecao_dev/orquestrador"
AMOSTRAS = comum.REPO / "build-profile/tmp/inspecao_amostras_0645"
NUCLEO = ("lev_trote_0", "lev_corrida_0", "lev_sprint", "lev_trote_2", "lev_trote_6", "lev_corrida_2", "lev_corrida_6",
          "lev_levantar", "lev_parado")
TOPO = 4  # (os primeiros do ranking de dificuldade do painel)
ESFRIAR_C = 70.0  # (o mesmo do vigia para retomar o treino)
POSES = 10  # (as poses de cada folha do Inspector, as da folha do fantasma)
ACHADOS = 10  # (os piores achados no index)
# (A ordem das figuras de um exercicio no index, as mais uteis primeiro; as de cada modo depois, na ordem de --modos.)
FIGURAS = ("folha", "fantasma_lugar", "espelho", "desvio", "fantasma_mundo", "pegadas", "parado")


@dataclass
class Registro:
    """O que a inspecao anota para o relatorio: as temperaturas, as falhas e os pulos, os tempos de cada fase e a parada do
    treino."""
    log: object = print
    termico: list = field(default_factory=list)  # (hora, passo, CPU, GPU, espera s)
    falhas: list = field(default_factory=list)
    tempos: list = field(default_factory=list)  # (fase, s)
    ativo: bool | None = None  # (o treino no comeco dos pesados; None sem eles)
    parado: list = field(default_factory=list)  # [de, ate] (ate None: ficou parado); vazio: nao parou
    paradas: int = 0  # (as vezes que a inspecao parou o treino; > 0: religa no fim)
    religado: int | None = None
    quente: bool = False  # (um passo pesado sem esfriar: os pesados acabaram)

    def medir(self, passo: str) -> None:
        cpu, gpu = comum.temperatures()
        self.termico.append((time.time(), passo, cpu, gpu, 0.0))

    def parar(self, passo: str) -> None:
        """O treino ativo (no comeco ou religado no meio: o vigia retoma o que parou quando a CPU esfria, o systemd o que
        caiu) e parado e fica para religar no fim."""
        if comum.service_active():
            self.parado = self.parado or [time.time(), None]
            self.paradas += 1  # (antes de parar: um sinal no meio da parada tambem religa)
            comum.stop_training(self.log)
            self.medir(f"treino parado ({passo})")

    def esfriar(self, passo: str) -> bool:
        """Antes de cada passo pesado (a regra termica, conferida na hora): o treino parado e a CPU abaixo de ESFRIAR_C
        (ate 5 min). Sem esfriar ou sem leitura, False neste e nos seguintes: os pesados acabam e o treino religa, em vez
        de ficar parado ate 5 min por passo."""
        if self.quente:
            return False
        self.parar(passo)
        t = time.time()
        cpu = comum.wait_cool(ESFRIAR_C, log=lambda m: None)
        self.termico.append((time.time(), passo, cpu, float("nan"), time.time() - t))
        if cpu < ESFRIAR_C:
            self.parar(passo)  # (voltou durante a espera)
            return True
        self.quente = True
        self.falhas.append(f"{passo} e os seguintes: pulados, " + (
            "sem leitura da CPU" if cpu != cpu else
            f"CPU {cpu:.0f} °C depois de {time.time() - t:.0f} s de espera (o limite é {ESFRIAR_C:.0f})"))
        return False

    @contextmanager
    def fase(self, nome: str):
        t = time.time()
        try:
            yield
        finally:
            self.tempos.append((nome, time.time() - t))


def _interromper(sinal, _quadro):
    # (KeyboardInterrupt e nao SystemExit: os passos seguram SystemExit e Exception, e o sinal tem de chegar ao finally.)
    raise KeyboardInterrupt(f"sinal {sinal}")


def _leve() -> None:
    """Os passos leves com o treino rodando: nice 10 (ou mais) e os nucleos 0-1, em todas as threads do processo."""
    for tid in map(int, os.listdir("/proc/self/task")):
        os.setpriority(os.PRIO_PROCESS, tid, max(10, os.getpriority(os.PRIO_PROCESS, tid)))
        os.sched_setaffinity(tid, {0, 1})


def _ordem(ex: str) -> int:
    return P.ORDEM.index(ex) if ex in P.ORDEM else len(P.ORDEM)


def escolher(an: dict) -> list[str]:
    """Os TOPO primeiros do ranking de dificuldade do painel, os espelhos deles e o NUCLEO, na ordem do painel."""
    topo = an["ranking"][:TOPO]
    todos = dict.fromkeys([*topo, *(comum.mirror_of(k) for k in topo), *NUCLEO])
    return sorted((k for k in todos if k), key=_ordem)


# --- Os passos pesados (com o treino parado) -------------------------------------------------------------------------

def capturar(exercicios: list, modos: list, a, caps: Path, reg: Registro, log) -> list[Path]:
    """As capturas (inspecao.captura) de cada exercicio em cada modo, uma por vez, cada uma depois de esfriar."""
    feitas = []
    for ex in exercicios:
        for modo in modos:
            passo = f"captura {ex} {modo}"
            if not reg.esfriar(passo):
                continue
            try:
                feitas.append(capturar_uma(ex, modo, a, caps, log))
            except (Exception, SystemExit) as e:
                reg.falhas.append(f"{passo}: {e}")
                log(f"{passo}: {e}")
    return feitas


def capturar_uma(ex: str, modo: str, a, caps: Path, log) -> Path:
    """Uma captura num processo proprio com limite de tempo (05/11: a do lev_troca_forte travou no simulador e deixou o
    treino parado 1h30 - um laco no codigo nativo nao se interrompe por dentro)."""
    cmd = [sys.executable, "-m", "inspecao.captura", "--treino", TREINO, "--exercicios", ex, "--modos", modo,
           "--mundos", str(a.mundos), "--segundos", str(a.segundos), "--saida", str(caps)]
    try:
        r = subprocess.run(cmd, cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True,
                           timeout=CAPTURA_LIMITE_S)
    except subprocess.TimeoutExpired:
        raise RuntimeError(f"passou de {CAPTURA_LIMITE_S} s (travou) - pulada") from None
    npz = caps / f"{ex}__{modo}.npz"
    if r.returncode != 0 or not npz.exists():
        raise RuntimeError(f"saiu com {r.returncode}: {r.stderr.strip()[-300:]}")
    return npz


# O limite de cada captura (s): 4 mundos x 8 s levam ~10 s; 300 s so com o simulador travado.
CAPTURA_LIMITE_S = 300


def folhas(npz: Path, destino: Path, reg: Registro) -> None:
    """As folhas do jogador de verdade (MatterModelInspector quadros) de uma captura, no mundo 0 (o .dump): as poses da
    folha do fantasma, de frente para o corpo e do lado direito dele (o rumo medio do abdome nas poses)."""
    cap = G.carregar(npz)
    jan = F.janela(cap, 0)
    passos = np.round(jan.t[F._poses(jan, POSES)] / cap.dt).astype(int)
    # (O .dump grava os passos pares, no instante (passo + 1) * dt: o impar vai para o par de baixo, ou o de cima quando o
    # de baixo ja e do episodio anterior.)
    lo = int(round(jan.t[0] / cap.dt))
    passos = np.unique(np.where(passos % 2 == 0, passos, np.where(passos - 1 >= lo, passos - 1, passos + 1)))
    rumo = G.referencial(cap.corpo[0, passos].astype(float))["rumo"]
    frente = float(np.degrees(np.arctan2(np.sin(rumo).mean(), np.cos(rumo).mean())))
    destino.mkdir(parents=True, exist_ok=True)
    # (O yaw do Inspector e no mundo do dump: a camera fica na direcao dele a partir da pelve - o rumo do corpo e a frente,
    # menos 90 graus o lado direito, como a vista "direita"; o pitch 4 das vistas dele.)
    for vista, yaw in (("frente", frente), ("lado", frente - 90.0)):
        passo = f"folha {cap.nome} {vista}"
        if not reg.esfriar(passo):
            continue
        saida = destino / f"{cap.nome}__{vista}.png"
        r = subprocess.run([str(comum.INSPECTOR), "quadros", "--dump", str(npz.with_suffix(".dump")), "--instantes",
                            ",".join(f"{(p + 1) * cap.dt:.4f}" for p in passos), "--yaw", f"{yaw:.1f}", "--pitch", "4",
                            "--colunas", "5", "--largura", "260", "--altura", "340", "--saida", str(saida)],
                           capture_output=True, text=True, timeout=300)
        if r.returncode or not saida.exists():
            reg.falhas.append(f"{passo}: o Inspector falhou ({r.returncode}): {r.stderr.strip()[-300:]}")


def pesados(exercicios: list, a, pasta: Path, reg: Registro, log) -> None:
    """Para o treino (se ativo, e de novo antes de cada passo se ele voltou no meio), captura e desenha as folhas; religa
    num finally o que parou, salvo --manter-parado. Aqui o SIGTERM e o SIGHUP viram KeyboardInterrupt (o finally religa
    tambem quando executar() e chamado de fora do main)."""
    reg.ativo = comum.service_active()
    sinais = {s: signal.signal(s, _interromper) for s in (signal.SIGTERM, signal.SIGHUP)}
    try:
        reg.parar("começo dos pesados")
        with reg.fase("capturas"):
            feitas = capturar(exercicios, a.modos, a, pasta / "capturas", reg, log)
        if not a.sem_folhas:
            with reg.fase("folhas"):
                for npz in feitas:
                    if reg.quente:
                        break
                    if npz.stem.split("__")[1] == "clipe":
                        continue
                    try:
                        folhas(npz, pasta / "folhas", reg)
                    except (Exception, SystemExit) as e:
                        reg.falhas.append(f"folhas {npz.stem}: {e}")
        reg.medir("fim dos pesados")
    finally:
        try:
            if reg.paradas and not a.manter_parado:
                with reg.fase("religar"):
                    reg.religado = comum.start_training(TREINO, log)
                reg.parado[1] = time.time()
                reg.medir(f"treino religado (r{reg.religado})")
        finally:
            for s, h in sinais.items():
                signal.signal(s, h)


def ligar(origem: Path, exercicios: list, modos: list, caps: Path, reg: Registro) -> None:
    """O --simular: links em capturas/ para as capturas prontas da `origem` (as que faltam viram falhas)."""
    caps.mkdir(parents=True, exist_ok=True)
    for ex in exercicios:
        for modo in modos:
            npz = origem / f"{ex}__{modo}.npz"
            if not npz.exists():
                reg.falhas.append(f"sem a captura {npz.name} em {origem}")
                continue
            for arq in (npz, npz.with_suffix(".json"), npz.with_suffix(".dump")):
                alvo = caps / arq.name
                alvo.unlink(missing_ok=True)
                if arq.exists():
                    alvo.symlink_to(arq.resolve())


# --- Os passos leves (com o treino religado) --------------------------------------------------------------------------

def _json(caminho: Path, dados: dict) -> None:
    caminho.write_text(json.dumps(dados, ensure_ascii=False))


def leves(exercicios: list, modos: list, pasta: Path, reg: Registro, painel_json: Path, ant: Path | None,
          log) -> tuple[dict, dict, dict | None]:
    """A marcha de cada captura (e as comparacoes com o clipe puro e o espelho), o fantasma de cada exercicio, o antes x
    agora (com a inspecao anterior `ant`) e o auditor. Devolve as analises da marcha, os espelhos ({(a, b, modo):
    espelho}) e o resultado do auditor (None se falhou)."""
    caps, md = pasta / "capturas", pasta / "marcha"
    rs, esp = {}, {}
    with reg.fase("marcha"):
        for npz in sorted(caps.glob("*.npz")):
            try:
                rs[npz.stem] = M.salvar(npz, md)
            except (Exception, SystemExit) as e:
                reg.falhas.append(f"marcha {npz.stem}: {e}")
        try:
            for nome, r in rs.items():
                c = rs.get(f"{r['exercicio']}__clipe")
                if c is not None and c is not r:
                    _json(md / f"{nome}__contra__{c['captura']}.json", M.comparar(r, c))
            for ea, eb in comum.MIRROR_PAIRS:
                for modo in modos:
                    ra, rb = rs.get(f"{ea}__{modo}"), rs.get(f"{eb}__{modo}")
                    if ra and rb:
                        esp[ea, eb, modo] = M.espelho(ra, rb)
                        _json(md / f"{ra['captura']}__espelho__{rb['captura']}.json", esp[ea, eb, modo])
        except (Exception, SystemExit) as e:
            reg.falhas.append(f"marcha (contra o clipe puro e espelhos): {e}")
    log(f"marcha: {len(rs)} capturas, {len(esp)} espelhos")
    with reg.fase("fantasma"):
        for ex in exercicios:
            grupo = F.irmas(caps / f"{ex}.npz")
            if not grupo:
                continue
            t = time.time()
            try:
                n = len(F.padrao(grupo, caps / "fantasma", log=lambda m: None))
                log(f"fantasma {ex}: {n} figuras em {time.time() - t:.0f} s")
            except (Exception, SystemExit) as e:
                reg.falhas.append(f"fantasma {ex}: {e}")
    # (O antes x agora: os numeros da anterior ja vao entre parenteses no index; aqui a pose, na mesma fase. O clipe puro
    # nao muda com a rede.)
    with reg.fase("antes × agora"):
        for ex in exercicios if ant else []:
            for modo in modos:
                antes, agora = ant / "capturas" / f"{ex}__{modo}.npz", caps / f"{ex}__{modo}.npz"
                if modo == "clipe" or not (antes.exists() and agora.exists()):
                    continue
                try:
                    F.salvar(F.folha([F.janela(G.carregar(antes)), F.janela(G.carregar(agora))]),
                             pasta / "antes_agora" / f"{ex}__{modo}.png", log=lambda m: None)
                except (Exception, SystemExit) as e:
                    reg.falhas.append(f"antes × agora {ex} {modo}: {e}")
    au = None
    with reg.fase("auditor"):
        try:
            au = A.gerar(md, pasta / "auditor", caps, painel_json, ant / "auditor" / "auditor.json" if ant else None)
        except (Exception, SystemExit) as e:
            reg.falhas.append(f"auditor: {e}")
    return rs, esp, au


# --- O relatorio ------------------------------------------------------------------------------------------------------

def _hms(t: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(t))


def _dur(s: float) -> str:
    m, r = divmod(round(s), 60)  # (arredondado antes: 119,6 s e "2 min 00 s", nao "1 min 60 s")
    return f"{m} min {r:02d} s" if m else f"{r} s"


def _rel(p, pasta: Path) -> str:
    return os.path.relpath(p, pasta)


def _nome_modo(m: str) -> str:
    return m if m == "atual" else "clipe puro" if m == "clipe" else A._pc(float(m))


def _modo(r: dict, r0: dict | None = None) -> str:
    if r["modo"] != "atual":
        return _nome_modo(r["modo"])
    return f"atual ({A._pc(r['forca'])}" + (f"; antes {A._pc(r0['forca'])}" if r0 else "") + ")"


def _meio(x: dict, chave: str) -> float | None:
    """A parte comum de um par espelhado (marcha.espelho da a + b refletido nas medidas de lado): a metade."""
    v = A._v(x, chave)
    return None if v is None else v / 2


def _cel(r: dict, r0: dict | None, chave: str, casas: int = 1, sinal: bool = False) -> str:
    """O valor de agora e, entre parenteses, o da inspecao anterior."""
    txt = A._n(A._v(r, chave), casas, sinal)
    return txt + (f" ({A._n(A._v(r0, chave), casas, sinal)})" if r0 else "")


def _mao(r: dict) -> str:
    """A mao de fora em % do peso: o que ela segura / o que empurra (a marcha, nos quadros analisados)."""
    s, e = A._v(r, "global.mao_sustenta_frac"), A._v(r, "global.mao_empurra_frac")
    return "–" if s is None or e is None else f"{100 * s:.0f}/{100 * e:.0f}"


def _ordem_figura(nome: str, modos: list) -> tuple:
    """As do exercicio primeiro (na ordem de FIGURAS), depois as de cada modo na ordem de --modos ("0.3__movimento",
    "0.3.ciclo": o modo pode ter ponto; "0.3" so, o antes x agora)."""
    k = next((i for i, m in enumerate(modos, 1) if nome == m or nome.startswith((m + "__", m + "."))), 0)
    return k, FIGURAS.index(nome) if nome in FIGURAS else len(FIGURAS), nome


def _erro(r: dict) -> float | None:
    v = [d["media"] for d in (r.get("erro_referencia_cm") or {}).values() if d.get("media") is not None]
    return float(np.mean(v)) if v else None


def _achado(a: dict, pasta: Path, com_exercicio: bool = True) -> str:
    det = f" ({A.LADO.get(a['detalhe'], a['detalhe'])})" if a["detalhe"] else ""
    fig = f" → [figura]({_rel(a['figuras'][0], pasta)})" if a.get("figuras") else ""
    hist = f" [{a['historico']}]" if a.get("historico") else ""
    onde = f"{a['exercicio']} · " if com_exercicio else ""
    return (f"**{A.GRAVIDADE_MD[a['nivel']]} · {onde}{A._modo_txt(a)} · {A.NOMES[a['regra']]}{det}** — {a['o_que']}. "
            f"*Agir:* {a['agir']}.{hist}{fig}")


def _secao_painel(an: dict, pasta: Path) -> list:
    rg = an["regua"]
    L = [f"## Painel do treino — r{an['atualizacao']}, {P._hm(an['hora'], True)}", "",
         f"Força média {A._pc(rg['forca_media'])} (há {rg['janela_min']:.0f} min: {A._pc(rg['forca_media_ini'])}); na "
         f"última hora {rg['descidas_1h']} descidas e {rg['subidas_1h']} subidas; {rg['zerados']} exercícios a 0%.", ""]
    L += [f"- **{p['gravidade']}** {p['texto']}" for p in an["problemas"]] or ["- nenhum problema apontado"]
    L += ["", "Os mais difíceis (o ranking): " + "; ".join(
        f"{i}. {P.nome(k)} {P._pf(an['exercicios'][k]['forca'])} ({P._segura(an['exercicios'][k])})"
        for i, k in enumerate(an["ranking"][:2 * TOPO], 1)) + ".", "",
        "Arquivos: " + " · ".join(f"[{n}]({_rel(pasta / 'painel' / n, pasta)})"
                                  for n in ("painel.md", "forcas.png", "bloqueios.png", "espelhos.png")), ""]
    return L


def _secao_exercicio(ex: str, modos: list, pasta: Path, rs: dict, esp: dict, au: dict | None, an: dict,
                     ant: Path | None) -> list:
    caps = pasta / "capturas"
    e = an["exercicios"].get(ex)
    L = [f"### {ex} — {P.nome(ex)}", ""]
    if e:
        pos = an["ranking"].index(ex) + 1 if ex in an["ranking"] else None
        L += [f"No treino (r{an['atualizacao']}): força {P._pf(e['forca'])} (1 h antes {P._pf(e['forca_60'])}), "
              f"{P._segura(e)}" + (f"; sem descer há {e['sem_descer_min']:.0f} min" if e["estado"] != "desceu" else "")
              + (f"; {pos}º no ranking de dificuldade" if pos else "") + ".", ""]
    linhas = [rs[f"{ex}__{m}"] for m in modos if f"{ex}__{m}" in rs]
    if linhas:
        c = rs.get(f"{ex}__clipe")
        L += ["| modo | lado (°) | lado − clipe puro | tronco à frente (°) | desvio do rumo (°) | avanço/pedido (m/s) | "
              "mão segura/empurra (% do peso) | cadência (passos/min) | quedas | semelhança | estab. | erro ao clipe (cm) |",
              "|---|" + "---|" * 11]
        com_antes = False
        for r in linhas:
            r0 = None
            if ant and (ant / "marcha" / f"{r['captura']}.marcha.json").exists():
                r0 = json.loads((ant / "marcha" / f"{r['captura']}.marcha.json").read_text())
                com_antes = True
            lado = A._v(r, "global.inclinacao_lado_corpo.vies")
            dc = A._v(c, "global.inclinacao_lado_corpo.vies") if c and r is not c else None
            q = r["episodios"]
            pedido = A._v(r, "global.pedido_ms")
            L.append(f"| {_modo(r, r0)} | "
                     f"{_cel(r, r0, 'global.inclinacao_lado_corpo.vies', 1, True)} | "
                     f"{A._n(lado - dc if lado is not None and dc is not None else None, 1, True)} | "
                     f"{_cel(r, r0, 'global.inclinacao_frente_tronco.vies', 1, True)} | "
                     f"{_cel(r, r0, 'global.desvio_rumo_graus', 1, True)} | "
                     + (f"{_cel(r, r0, 'global.avanco_ms', 2)}/{A._n(pedido, 1)} | " if pedido else "– | ") +
                     _mao(r) + (f" ({_mao(r0)})" if r0 else "") + " | "
                     f"{_cel(r, r0, 'global.cadencia', 0)} | {q['cairam']}/{q['n']}"
                     + (f" ({r0['episodios']['cairam']}/{r0['episodios']['n']})" if r0 else "") + " | "
                     f"{_cel(r, r0, 'episodios.semelhanca', 2)} | {_cel(r, r0, 'episodios.estabilidade', 2)} | "
                     f"{A._n(_erro(r), 1)}" + (f" ({A._n(_erro(r0), 1)})" if r0 else "") + " |")
        L += ["", "Graus + = esquerda; lado é o corpo inteiro (a cabeça sobre os tornozelos), fora da largada, do levantar "
              "e das quedas; mão: a força auxiliar média para cima / no plano, em % do peso (na força F ela vai até F)"
              + ("; entre parênteses, a inspeção anterior." if com_antes else "."), ""]
    outro = comum.mirror_of(ex)
    ea, eb = (ex, outro) if (ex, outro) in comum.MIRROR_PAIRS else (outro, ex)
    par = [(m, esp[ea, eb, m]) for m in modos if (ea, eb, m) in esp]
    if par:
        L += [f"Espelho {ea} × {eb}, o viés da rede (a parte comum, que não troca com o sentido; + = esquerda): " + "; ".join(
            f"{_nome_modo(m)} lado {A._n(_meio(x, 'global.inclinacao_lado_corpo.vies'), 1, True)}°, rumo "
            f"{A._n(_meio(x, 'global.desvio_rumo_graus'), 1, True)}°" for m, x in par) + ".", ""]
    if au:
        meus = [a for a in au["achados"] if ex in a["exercicios"]]
        if meus:
            n = {g: sum(a["gravidade"] == g for a in meus) for g in ("alta", "media", "baixa")}
            L += [f"Achados: {n['alta']} altos, {n['media']} médios, {n['baixa']} baixos."]
            L += [f"- {_achado(a, pasta, len(a['exercicios']) > 1)}" for a in meus[:4]]
            L.append("")
    for titulo, d in (("antes × agora", pasta / "antes_agora"), ("folhas do jogador", pasta / "folhas"),
                      ("fantasma", caps / "fantasma"), ("marcha", pasta / "marcha")):
        figs = sorted(((f.name.removeprefix(ex + "__").removesuffix(".png"), f) for f in d.glob(f"{ex}__*.png")),
                      key=lambda x: _ordem_figura(x[0], modos))
        if figs:
            L.append(f"- {titulo}: " + " · ".join(f"[{nome.replace('__', ' ')}]({_rel(f, pasta)})" for nome, f in figs))
    return L + [""]


def indice(pasta: Path, a, an: dict, exercicios: list, rs: dict, esp: dict, au: dict | None, reg: Registro,
           ant: Path | None) -> str:
    ups = sorted({r.get("atualizacao") for r in rs.values() if r.get("atualizacao") is not None})
    L = [f"# Inspeção {time.strftime('%d/%m/%Y %H:%M', time.localtime(reg.termico[0][0]))} — {TREINO}", ""]
    if a.so_painel:
        L += ["Só o painel (--so-painel): sem captura, sem parar o treino.", ""]
    else:
        origem = f"capturas prontas de `{a.simular}` (--simular: o treino não foi parado)" if a.simular else (
            f"{a.mundos} mundos × {a.segundos:g} s por captura")
        com = {r["exercicio"] for r in rs.values()}
        L += [f"**Capturas:** {len(rs)} analisadas, da rede em r{', r'.join(map(str, ups)) or '?'} — {len(com)} de "
              f"{len(exercicios)} exercícios × modos {', '.join(a.modos)}; {origem}. Anterior: "
              + (f"`{ant.name}`" if ant else "nenhuma") + ".", ""]
        if reg.parado:
            de, ate = reg.parado
            L += [f"**Treino parado** às {_hms(de)}"
                  + (f" ({reg.paradas} vezes: voltou a rodar no meio, o vigia ou o systemd, e parou de novo)"
                     if reg.paradas > 1 else "")
                  + (f", religado às {_hms(ate)} em r{reg.religado} (linha nova confirmada): {_dur(ate - de)} parado."
                     if ate else ", e ficou parado (--manter-parado)."), ""]
        elif reg.ativo is False:
            L += ["**Treino:** já estava parado no começo (não foi parado nem religado).", ""]
        if au:
            c = au["contagem"]
            L += [f"**Auditor:** {c['alta']} altos, {c['media']} médios, {c['baixa']} baixos"
                  + (f" · desde a anterior: {au['mudancas']}" if au.get("mudancas") else "") + ".", ""]
        if reg.falhas:
            L += [f"**Falhas e pulos:** {len(reg.falhas)} (ver o fim).", ""]
    L += _secao_painel(an, pasta)
    if au:
        L += [f"## Auditor — os piores ([relatório]({_rel(pasta / 'auditor' / 'relatorio_auditor.md', pasta)}))", ""]
        L += [f"{i}. {_achado(x, pasta)}" for i, x in enumerate(au["achados"][:ACHADOS], 1)]
        if au.get("resolvidos"):
            L += ["", "Resolvidos desde a anterior: " + ", ".join(f"{o['id']} ({o['medida']} → {o['agora']})"
                                                              for o in au["resolvidos"]) + "."]
        L.append("")
    if rs:
        L += ["## Exercícios", ""]
        for ex in exercicios:
            if any(r["exercicio"] == ex for r in rs.values()):
                L += _secao_exercicio(ex, a.modos, pasta, rs, esp, au, an, ant)
    if reg.falhas:
        L += ["## Falhas e pulos", ""] + [f"- {f}" for f in reg.falhas] + [""]
    L += ["## Registro térmico e tempos", "",
          "Tempos: " + " · ".join(f"{n} {_dur(s)}" for n, s in reg.tempos) + ".", "",
          "| hora | passo | CPU (°C) | GPU (°C) | espera (s) |", "|---|---|---|---|---|"]
    L += [f"| {_hms(t)} | {p} | {A._n(c if c == c else None, 0)} | {A._n(g if g == g else None, 0)} | {e:.0f} |"
          for t, p, c, g, e in reg.termico]
    return "\n".join(L) + "\n"


def _apontar(saida: Path, pasta: Path) -> None:
    """O ponteiro <saida>/ultima -> esta inspecao (relativo, trocado de uma vez)."""
    tmp = saida / ".ultima"
    tmp.unlink(missing_ok=True)
    tmp.symlink_to(pasta.name)
    os.replace(tmp, saida / "ultima")


def imprimir(pasta: Path, an: dict, au: dict | None, reg: Registro) -> None:
    mb = sum(f.stat().st_size for f in pasta.rglob("*") if f.is_file() and not f.is_symlink()) / 1e6
    print(f"\ninspeção: {pasta / 'index.md'} ({mb:.0f} MB)")
    if reg.parado:
        de, ate = reg.parado
        print((f"treino parado {_dur(ate - de)} ({_hms(de)}–{_hms(ate)}), religado r{reg.religado}" if ate
               else f"treino parado às {_hms(de)} e mantido parado")
              + (f"; parado {reg.paradas} vezes (voltou no meio)" if reg.paradas > 1 else ""))
    cpu = [c for _, _, c, _, _ in reg.termico if c == c]
    print(f"CPU {A._n(min(cpu), 0)}–{A._n(max(cpu), 0)} °C" if cpu else "CPU sem leitura")
    graves = [p for p in an["problemas"] if p["gravidade"] != "nota"]
    print(f"painel r{an['atualizacao']}: {len(an['problemas'])} problemas ({len(graves)} graves/alertas); difíceis: "
          + ", ".join(P.nome(k) for k in an["ranking"][:TOPO]))
    if au:
        c = au["contagem"]
        print(f"auditor: {c['alta']} altos, {c['media']} médios, {c['baixa']} baixos"
              + (f" (desde a anterior: {au['mudancas']})" if au.get("mudancas") else ""))
        for i, x in enumerate(au["achados"][:5], 1):
            print(f"  {i}. {x['gravidade']} {x['exercicio']} {x['modo']} {x['regra']} {x['medida']}"
                  + (f" [{x['historico']}]" if x.get("historico") else ""))
    for f in reg.falhas:
        print(f"falha: {f}")


# --- A inspecao -------------------------------------------------------------------------------------------------------

def executar(a, saida: Path = SAIDA, log=print) -> Path:
    """Uma inspecao inteira (o fluxo da docstring); devolve a pasta dela."""
    reg = Registro(log=log)
    pasta = saida / time.strftime("%Y%m%d_%H%M")
    if pasta.exists():
        pasta = saida / time.strftime("%Y%m%d_%H%M%S")
    pasta.mkdir(parents=True)
    ultima = saida / "ultima"
    ant = ultima.resolve() if (ultima / "auditor" / "auditor.json").exists() else None
    reg.medir("início")
    with reg.fase("painel"):
        an = P.gerar(TREINO, pasta / "painel", log=log)
    exercicios = list(dict.fromkeys(a.exercicios)) if a.exercicios else escolher(an)
    rs, esp, au = {}, {}, None
    if not a.so_painel:
        log("exercícios: " + ", ".join(exercicios))
        if a.simular:
            ligar(Path(a.simular), exercicios, a.modos, pasta / "capturas", reg)
        else:
            pesados(exercicios, a, pasta, reg, log)
        _leve()
        rs, esp, au = leves(exercicios, a.modos, pasta, reg, pasta / "painel" / "painel.json", ant, log)
    reg.medir("fim")
    (pasta / "index.md").write_text(indice(pasta, a, an, exercicios, rs, esp, au, reg, ant))
    if (pasta / "auditor" / "auditor.json").exists():
        _apontar(saida, pasta)
    imprimir(pasta, an, au, reg)
    return pasta


def _modo_arg(m: str) -> str:
    """atual, clipe ou uma forca de 0 a 1 (o "0,3" e o "30" param aqui, antes de parar o treino)."""
    if m in ("atual", "clipe") or 0.0 <= float(m) <= 1.0:
        return m
    raise argparse.ArgumentTypeError(f"{m}: a força vai de 0 a 1")


def _args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--so-painel", action="store_true", help="so o painel e o relatorio (nao para o treino)")
    ap.add_argument("--simular", metavar="PASTA", help="as capturas prontas da PASTA: sem parar, capturar, folhas nem religar")
    # (Os nomes conferidos aqui: um errado so falharia nas capturas, com o treino ja parado.)
    ap.add_argument("--exercicios", nargs="+", choices=P.ORDEM, metavar="EX",
                    help="os exercicios (padrao: o ranking do painel, os espelhos e o nucleo)")
    ap.add_argument("--modos", nargs="+", type=_modo_arg, default=["atual", "0.3", "clipe"],
                    help="atual | clipe | uma forca (0-1)")
    ap.add_argument("--mundos", type=int, default=4)
    ap.add_argument("--segundos", type=float, default=8.0)
    ap.add_argument("--sem-folhas", action="store_true", help="sem as folhas do MatterModelInspector")
    ap.add_argument("--manter-parado", action="store_true", help="nao religa o treino no fim")
    ap.add_argument("--saida", default=str(SAIDA), help="a pasta das inspecoes (e do ponteiro ultima)")
    ap.add_argument("--teste", action="store_true", help="o autoteste: --simular nas amostras, em inspecao_dev/orquestrador")
    return ap.parse_args(argv)


def _teste_pesados(saida: Path) -> None:
    """O caminho de verdade com dubles no lugar do servico, do simulador e do Inspector (nada disso roda): para antes de
    capturar e religa no fim, mesmo com erro numa captura ou com SIGTERM (e devolve o tratador do SIGTERM); para de novo o
    treino que voltou no meio (o vigia retoma o que parou quando esfria) e religa so o que parou; acaba os pesados na
    primeira vez sem esfriar (quente, morna ou sem leitura) e religa; nao religa com --manter-parado; nao faz folha do
    clipe puro; o Inspector pede os quadros que o .dump tem, de frente para o corpo (o lev_trote_0 da amostra olha para
    -72 graus no mundo) e do lado direito; e o --modos recusa "0,3", "30" e "nan", o --exercicios um nome errado."""
    reais = (comum.service_active, comum.stop_training, comum.start_training, comum.wait_cool, comum.Ambiente,
             captura.capturar, globals()["subprocess"], globals()["capturar_uma"])
    sigterm = signal.getsignal(signal.SIGTERM)
    feito, est = [], {}

    def capturar_falso(amb, ex, modo, mundos, segundos, caps, log=print):
        if est["erro"] == "sinal":
            os.kill(os.getpid(), signal.SIGTERM)
        if est["erro"]:
            raise est["erro"]
        feito.append("captura")
        if est["volta"]:  # (o vigia retoma o treino durante a captura)
            est.update(ativo=True, volta=False)
        caps.mkdir(parents=True, exist_ok=True)
        for suf in (".npz", ".json", ".dump"):
            (caps / f"{ex}__{modo}{suf}").unlink(missing_ok=True)
            (caps / f"{ex}__{modo}{suf}").symlink_to(AMOSTRAS / f"{ex}__{modo}{suf}")
        return caps / f"{ex}__{modo}.npz"

    def run_falso(cmd, **_):
        assert not est["ativo"], "o Inspector com o treino ativo"
        feito.append(cmd)
        Path(cmd[cmd.index("--saida") + 1]).write_bytes(b"")
        return reais[6].CompletedProcess(cmd, 0, "", "")

    def caso(*argv, ativo=True, volta=False, cpu=50.0, erro=None):
        """Os eventos (parar, captura, religar), os comandos do Inspector e o registro de uma rodada."""
        feito.clear()
        est.update(ativo=ativo, volta=volta, cpu=cpu, erro=erro)
        reg = Registro(log=lambda m: None)
        pesados(["lev_trote_0"], _args(["--modos", "atual", "clipe", *argv]), saida / "pesados", reg, lambda m: None)
        return [c for c in feito if isinstance(c, str)], [c for c in feito if isinstance(c, list)], reg

    comum.service_active = lambda: est["ativo"]
    comum.stop_training = lambda log: feito.append("parar") or est.update(ativo=False)
    comum.start_training = lambda treino, log: feito.append("religar") or est.update(ativo=True) or 99
    comum.wait_cool = lambda limit, timeout=300.0, log=print: est["cpu"]
    comum.Ambiente = lambda treino: None
    captura.capturar = capturar_falso
    globals()["capturar_uma"] = lambda ex, modo, a, caps, log: capturar_falso(None, ex, modo, a.mundos, a.segundos, caps, log)
    globals()["subprocess"] = type("Dubles", (), {"run": staticmethod(run_falso)})
    try:
        for m in ("0,3", "30", "nan"):
            try:
                _modo_arg(m)
                raise AssertionError(f"--modos {m} passou")
            except (ValueError, argparse.ArgumentTypeError):
                pass
        with redirect_stderr(io.StringIO()):
            try:
                _args(["--exercicios", "lev_trote_0", "lev_trote_9"])
                raise AssertionError("--exercicios lev_trote_9 passou")
            except SystemExit:
                pass
        ev, cmds, reg = caso()
        assert ev == ["parar", "captura", "captura", "religar"], ev
        assert len(cmds) == 2 and reg.religado == 99 and reg.parado[0] <= reg.parado[1] and not reg.falhas, reg
        quadros = {round(float(x.split()[1]), 3) for x in open(AMOSTRAS / "lev_trote_0__atual.dump") if x[:2] == "F "}
        t = [float(x) for x in cmds[0][cmds[0].index("--instantes") + 1].split(",")]
        yaw = [float(c[c.index("--yaw") + 1]) for c in cmds]
        print(f"pesados (dubles): {len(t)} quadros de {t[0]:.2f} a {t[-1]:.2f} s, yaw de frente {yaw[0]:.0f}°, de lado "
              f"{yaw[1]:.0f}°")
        assert 6 <= len(t) <= POSES and all(round(x, 3) in quadros for x in t), t
        assert abs(yaw[0] + 72.0) < 10.0 and abs(yaw[1] - (yaw[0] - 90.0)) < 0.11, yaw
        for erro in (RuntimeError("boom"), "sinal"):
            try:
                ev, _, reg = caso(erro=erro)
                assert erro != "sinal" and "boom" in reg.falhas[0], ("o SIGTERM deveria interromper", reg.falhas)
            except KeyboardInterrupt:
                ev = [c for c in feito if isinstance(c, str)]
                assert erro == "sinal"
            assert ev == ["parar", "religar"] and signal.getsignal(signal.SIGTERM) is sigterm, (erro, ev)
        for cpu in (85.0, 75.0, float("nan")):
            ev, cmds, reg = caso(cpu=cpu)
            assert ev == ["parar", "religar"] and not cmds and len(reg.falhas) == 1 and "pulados" in reg.falhas[0], (
                cpu, ev, reg.falhas)
        ev, _, reg = caso(volta=True)
        assert ev == ["parar", "captura", "parar", "captura", "religar"] and reg.paradas == 2, ev
        ev, _, reg = caso(ativo=False, volta=True)
        assert ev == ["captura", "parar", "captura", "religar"] and reg.ativo is False, ev
        ev, cmds, reg = caso(ativo=False)
        assert ev == ["captura", "captura"] and len(cmds) == 2 and not reg.parado, ev
        ev, _, _ = caso("--manter-parado")
        assert ev == ["parar", "captura", "captura"], ev
        print("pesados (dubles): religa o que parou com erro e com SIGTERM; para de novo o que voltou no meio; acaba os "
              "pesados sem esfriar (85, 75 ou sem leitura); mantém parado")
    finally:
        (comum.service_active, comum.stop_training, comum.start_training, comum.wait_cool, comum.Ambiente,
         captura.capturar, globals()["subprocess"], globals()["capturar_uma"]) = reais


def autoteste(saida: Path = SAIDA_TESTE) -> None:
    """O caminho de verdade com dubles (_teste_pesados) e, nas amostras (r23358) com o painel do log de agora, uma inspecao
    inteira pelo --simular (os exercicios do ranking sem amostra viram falhas; o nucleo inteiro tem), uma segunda menor
    contra ela (o --anterior pela ultima, o antes -> agora nos numeros e nas folhas) e um --so-painel; e os fatos: o
    relatorio com as secoes e as figuras de cada exercicio do nucleo, a mao de fora na tabela, o espelho corrida 2x6 com
    o vies da rede para a direita (e o do clipe puro ~0), o auditor acusando a inclinacao para a direita do lev_trote_0 a
    30%, e o ponteiro ultima so nas inspecoes com o auditor."""
    t = time.time()
    _teste_pesados(saida)
    p1 = executar(_args(["--simular", str(AMOSTRAS)]), saida)
    print(f"inspeção simulada em {time.time() - t:.0f} s")
    md = (p1 / "index.md").read_text()
    for secao in ("## Painel do treino", "## Auditor — os piores", "## Exercícios", "## Registro térmico e tempos"):
        assert secao in md, secao
    for ex in NUCLEO:
        assert f"### {ex} — " in md, ex
        for nome in ("fantasma_lugar", "fantasma_mundo", "desvio", "folha", "pegadas", "atual__movimento"):
            f = p1 / "capturas" / "fantasma" / f"{ex}__{nome}.png"
            assert f.stat().st_size > 30_000 and f"]({_rel(f, p1)})" in md, f
        for modo in ("atual", "0.3", "clipe"):
            for fig in ("ciclo", "series", "pegadas"):
                assert (p1 / "marcha" / f"{ex}__{modo}.{fig}.png").stat().st_size > 10_000, (ex, modo, fig)
    for ex in ("lev_trote_2", "lev_trote_6", "lev_corrida_2", "lev_corrida_6"):
        assert (p1 / "capturas" / "fantasma" / f"{ex}__espelho.png").exists(), ex
    esp = {m: json.loads((p1 / "marcha" / f"lev_corrida_2__{m}__espelho__lev_corrida_6__{m}.json").read_text())
           for m in ("atual", "clipe")}
    vies = {m: e["global"]["inclinacao_lado_corpo"]["vies"] / 2 for m, e in esp.items()}
    print(f"espelho corrida 2×6, a parte comum do lado: atual {vies['atual']:+.2f}°, clipe puro {vies['clipe']:+.2f}°")
    assert vies["atual"] < -1.0 and abs(vies["clipe"]) < 0.5, vies
    au = json.loads((p1 / "auditor" / "auditor.json").read_text())
    assert any(x["regra"] == "lado" and x["exercicio"] == "lev_trote_0" and x["modo"] == "0.3" for x in au["achados"])
    assert os.readlink(saida / "ultima") == p1.name
    n_falhas = md.count("\n- sem a captura ")
    print(f"index {len(md) // 1024} KB, {len(au['achados'])} achados, {n_falhas} capturas sem amostra (do ranking)")
    # (A segunda, menor, contra a primeira: o --anterior pela ultima e o antes -> agora na tabela.)
    p2 = executar(_args(["--simular", str(AMOSTRAS), "--exercicios", "lev_trote_2", "lev_trote_6", "--modos", "atual",
                         "clipe"]), saida)
    au2 = json.loads((p2 / "auditor" / "auditor.json").read_text())
    md2 = (p2 / "index.md").read_text()
    assert au2["anterior"] == str(p1 / "auditor" / "auditor.json") and au2.get("mudancas"), au2.get("anterior")
    assert f"Anterior: `{p1.name}`" in md2 and re.search(r"\| atual \(\d+%; antes \d+%\) \| [−+]\d,\d \([−+]\d,\d\) \|", md2), \
        md2[:3000]
    assert os.readlink(saida / "ultima") == p2.name
    # (O antes x agora so das capturas da rede que a anterior tambem tem; a mao de fora na tabela, com o antes.)
    for ex in ("lev_trote_2", "lev_trote_6"):
        f = p2 / "antes_agora" / f"{ex}__atual.png"
        assert f.stat().st_size > 30_000 and f"[atual]({_rel(f, p2)})" in md2, f
    assert not (p2 / "antes_agora" / "lev_trote_2__clipe.png").exists()
    assert re.search(r"\| \d+/\d+ \(\d+/\d+\) \|", md2) and "mão segura/empurra" in md, "a mão de fora na tabela"
    print(f"segunda inspeção contra a primeira: {au2['mudancas']}")
    # (O --so-painel nao move o ponteiro: ele e a base do auditor da proxima.)
    p3 = executar(_args(["--so-painel"]), saida)
    assert (p3 / "index.md").exists() and not (p3 / "capturas").exists() and os.readlink(saida / "ultima") == p2.name
    print(f"orquestrador ok em {time.time() - t:.0f} s: {p1}, {p2}, {p3}")


def main() -> None:
    a = _args()
    if a.teste:
        autoteste()
        return
    executar(a, Path(a.saida))


if __name__ == "__main__":
    main()
