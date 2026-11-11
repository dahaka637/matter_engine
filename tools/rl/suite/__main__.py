"""A linha de comando da suite (de dentro de tools/rl):

  .venv/bin/python -m suite medir --candidato NOME=CHECKPOINT@RECEITA [...] --saida DIR [--conjunto decisao|diagnostico]
        [--build BUILD/python] [--assets DIR] [--paralelo 2] [--margens margens.json]
      cada candidato num processo (freio nice -n 10), no maximo --paralelo de uma vez; a CPU acima de 85 graus pausa a captura
      ate 80. Depois as medidas, resultados.json, tabela.md e as figuras. RECEITA: um arquivo ou o nome em tools/rl/receitas.
  .venv/bin/python -m suite comparar --candidatos DIR/capturas/A DIR/capturas/B [...] --saida DIR [--margens margens.json]
      so as medidas e a saida, das capturas ja feitas (o primeiro e a referencia).
  .venv/bin/python -m suite aa --resultados DIR/resultados.json --saida margens.json
      o teste A/A (metodologia, secao 5): com os dois bracos iguais medidos (o primeiro e a referencia), a margem de cada medida
      e o maior desvio que o IC95 do delta pareado A/A admite, max(|lo|, |hi|). Delta degenerado (IC [0, 0], ex.: zero quedas
      nos dois) fica com a provisoria. Tambem conta o alarme falso das provisorias (quantas guardas o A/A reprovaria).
  .venv/bin/python -m suite autoteste [--checkpoint CK --receita R]
      as medidas em dados sinteticos (suite/testes) e o determinismo: o mesmo cenario duas vezes da arrays identicos.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from suite import cenarios as C

RL = Path(__file__).resolve().parents[1]
PADRAO_PRINCIPAL = "~/.local/share/matter-engine/rl/runs/rl_final_clipe/pontos/r71860_pausa_parado_1011.pt"
CPU_PARA, CPU_VOLTA = 85.0, 80.0


def _ambiente(a) -> None:
    if a.build:
        os.environ["MATTER_TRACKING_NATIVE"] = str(Path(a.build).expanduser())
    os.environ.setdefault("TMPDIR", str(RL.parents[1] / "build-profile" / "tmp" / "suite"))


def pausa_quente(log=print) -> None:
    """A regra termica: acima de 85 graus a captura espera a CPU voltar a 80."""
    from inspecao.comum import temperatures
    cpu, _ = temperatures()
    if cpu == cpu and cpu >= CPU_PARA:
        log(f"CPU {cpu:.1f} C: pausa ate {CPU_VOLTA:.0f}")
        while cpu == cpu and cpu >= CPU_VOLTA:
            time.sleep(10)
            cpu, _ = temperatures()


def cmd_capturar(a) -> int:
    """(interno) um candidato inteiro, no processo dele."""
    _ambiente(a)
    import torch
    torch.set_num_threads(a.torch_threads)
    from suite import captura
    cand = captura.carregar(a.nome, a.checkpoint, a.receita, a.assets)
    inicio = time.time()
    tempos = captura.capturar_todos(cand, a.conjunto, Path(a.saida), a.mundos, a.threads, a.so, pausa_quente=pausa_quente)
    meta = dict(cand.meta, conjunto=a.conjunto, tempos=tempos, segundos_captura=round(time.time() - inicio, 1))
    (Path(a.saida) / "candidato.json").write_text(json.dumps(meta, ensure_ascii=False, indent=1))
    print(f"{a.nome}: {len(tempos)} capturas em {meta['segundos_captura']} s", flush=True)
    return 0


def comparar(pastas: dict, saida: Path, margens_json: str | None = None, figuras: bool = True) -> dict:
    """{candidato: pasta das capturas} -> resultados.json, tabela.md e as figuras."""
    from suite import estatistica, figuras as F, medidas, relatorio
    from suite.medidas import base
    medidas_por = {}
    metas = {}
    import numpy as np
    assinaturas = {}
    for nome, pasta in pastas.items():
        t = time.time()
        caps = base.capturas(pasta)
        medidas_por[nome] = medidas.medir(caps)
        # (O par e pareado se os pedidos sao os mesmos: a velocidade e o rumo de cada mundo e o clipe no primeiro passo. O quadro
        # do clipe nao serve: ele vai sob a pelve do corpo, que ja andou um passo da rede.)
        assinaturas[nome] = {k: (c.z["info"][:, 0, 3:5].ravel(), [c.meta["clipes"][i] if i >= 0 else "" for i in c.z["clipe"][:, 0]])
                             for k, c in caps.items()}
        cj = Path(pasta) / "candidato.json"
        metas[nome] = dict(json.loads(cj.read_text()) if cj.exists() else {}, segundos_medidas=round(time.time() - t, 1))
    ref = next(iter(pastas))
    for nome in pastas:
        metas[nome]["cenarios_nao_pareados"] = sorted(
            k for k, v in assinaturas[nome].items()
            if k not in assinaturas[ref] or v[1] != assinaturas[ref][k][1]
            or not np.allclose(v[0], assinaturas[ref][k][0], atol=1e-4, equal_nan=True))
    res = relatorio.montar(medidas_por, metas, estatistica.margens(margens_json))
    relatorio.escrever(res, saida)
    if figuras:
        F.parado(pastas, saida / "figuras" / "parado.png")
        F.freada(pastas, saida / "figuras" / "freada.png")
    print(f"saida: {saida}/tabela.md, resultados.json, figuras/", flush=True)
    return res


def cmd_medir(a) -> int:
    _ambiente(a)
    saida = Path(a.saida).expanduser()
    saida.mkdir(parents=True, exist_ok=True)
    pastas, filas = {}, []
    for spec in a.candidato:
        nome, resto = spec.split("=", 1)
        ck, receita = resto.rsplit("@", 1)
        pastas[nome] = saida / "capturas" / nome
        cmd = ["freio", "nice", "-n", "10", sys.executable, "-m", "suite", "capturar", "--nome", nome, "--checkpoint", ck,
               "--receita", receita, "--saida", str(pastas[nome]), "--conjunto", a.conjunto, "--assets", a.assets,
               "--mundos", str(a.mundos), "--threads", str(a.threads)] + (["--so", *a.so] if a.so else [])
        filas.append((nome, cmd))
    vivos: list = []
    inicio = time.time()
    while filas or vivos:
        while filas and len(vivos) < a.paralelo:
            pausa_quente()
            nome, cmd = filas.pop(0)
            log = open(saida / f"captura_{nome}.log", "w")
            vivos.append((nome, subprocess.Popen(cmd, cwd=RL, stdout=log, stderr=subprocess.STDOUT), time.time()))
            print(f"{nome}: capturando ({' '.join(cmd[4:6])} ...)", flush=True)
        time.sleep(2)
        for item in list(vivos):
            nome, p, t0 = item
            if p.poll() is not None:
                vivos.remove(item)
                print(f"{nome}: terminou ({p.returncode}) em {time.time() - t0:.0f} s", flush=True)
                if p.returncode:
                    raise SystemExit(f"{nome}: a captura falhou; veja {saida}/captura_{nome}.log")
    print(f"capturas: {time.time() - inicio:.0f} s", flush=True)
    comparar(pastas, saida, a.margens)
    return 0


def cmd_comparar(a) -> int:
    pastas = {Path(p).name: Path(p) for p in a.candidatos}
    comparar(pastas, Path(a.saida), a.margens, not a.sem_figuras)
    return 0


def cmd_aa(a) -> int:
    d = json.loads(Path(a.resultados).read_text())
    nomes = list(d["candidatos"])
    if len(nomes) != 2:
        raise SystemExit(f"o A/A pede exatamente 2 candidatos, veio {nomes}")
    margens, degeneradas, alarmes = {}, [], []
    for mid, r in d["candidatos"][nomes[1]]["medidas"].items():
        c = r.get("contra_referencia")
        if not c:
            continue
        lo, hi = c["ic95"]
        if not (lo == lo and hi == hi):
            continue
        m = max(abs(lo), abs(hi))
        if m == 0.0:
            degeneradas.append(mid)
            m = r["margem_provisoria"]
        if m is not None:
            margens[mid] = round(float(m), 6)
        if c["marca"] in ("piorou", "incerto+"):
            alarmes.append(f"{mid} ({c['marca']}: delta {c['delta_pior']:+.4g}, IC [{lo:+.4g}, {hi:+.4g}], provisoria {c['margem']})")
    Path(a.saida).write_text(json.dumps(margens, indent=1, sort_keys=True) + "\n")
    print(f"A/A {nomes[0]} x {nomes[1]}: {len(margens)} margens em {a.saida}; {len(degeneradas)} degeneradas (ficaram com a provisoria)")
    print(f"alarme falso das provisorias: {len(alarmes)} de {len(margens)} guardas")
    for x in alarmes:
        print("  " + x)
    return 0


def cmd_autoteste(a) -> int:
    import unittest
    r = unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.discover(str(Path(__file__).parent / "testes"),
                                                                                     top_level_dir=str(RL)))
    if not r.wasSuccessful():
        return 1
    _ambiente(a)
    import numpy as np
    import torch
    torch.set_num_threads(2)
    from suite import captura
    cand = captura.carregar("autoteste", a.checkpoint, a.receita, a.assets)
    base = Path(os.environ["TMPDIR"]) / "autoteste"
    cens = C.por_nome()
    for nome, cond in (("reto_corrida_3", "ruido"), ("parado", "ruido"), ("parada_trote_0", "det")):
        paths = [captura.rodar(cand, cens[nome], cond, "decisao", base / str(k), mundos=4, threads=a.threads) for k in (0, 1)]
        z0, z1 = np.load(paths[0]), np.load(paths[1])
        for key in z0.files:
            if not np.array_equal(z0[key], z1[key], equal_nan=True):
                print(f"DETERMINISMO FALHOU: {nome}__{cond} {key}")
                return 1
        print(f"determinismo ok: {nome}__{cond} ({len(z0.files)} arrays identicos)")
    print("autoteste ok")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(prog="python -m suite", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    comum = argparse.ArgumentParser(add_help=False)
    comum.add_argument("--build", default=None, help="a pasta python do build congelado (MATTER_TRACKING_NATIVE)")
    comum.add_argument("--assets", default=str(RL.parents[1] / "assets"), help="os assets do corpo (padrao: os do repositorio)")
    comum.add_argument("--threads", type=int, default=4, help="as threads da fisica por lote")
    m = sub.add_parser("medir", parents=[comum])
    m.add_argument("--candidato", nargs="+", required=True, metavar="NOME=CHECKPOINT@RECEITA")
    m.add_argument("--saida", required=True)
    m.add_argument("--conjunto", choices=tuple(C.SEMENTES), default="decisao")
    m.add_argument("--mundos", type=int, default=C.MUNDOS)
    m.add_argument("--paralelo", type=int, default=2)
    m.add_argument("--margens", default=None)
    m.add_argument("--so", nargs="+", default=None, help="so estes cenarios (diagnostico; a decisao usa todos)")
    c = sub.add_parser("capturar", parents=[comum])
    c.add_argument("--nome", required=True)
    c.add_argument("--checkpoint", required=True)
    c.add_argument("--receita", required=True)
    c.add_argument("--saida", required=True)
    c.add_argument("--conjunto", choices=tuple(C.SEMENTES), default="decisao")
    c.add_argument("--mundos", type=int, default=C.MUNDOS)
    c.add_argument("--torch-threads", type=int, default=2)
    c.add_argument("--so", nargs="+", default=None)
    k = sub.add_parser("comparar")
    k.add_argument("--candidatos", nargs="+", required=True, help="as pastas das capturas (o nome do candidato e o da pasta)")
    k.add_argument("--saida", required=True)
    k.add_argument("--margens", default=None)
    k.add_argument("--sem-figuras", action="store_true")
    q = sub.add_parser("aa")
    q.add_argument("--resultados", required=True, help="o resultados.json do medir com os dois bracos iguais")
    q.add_argument("--saida", required=True, help="o margens.json")
    t = sub.add_parser("autoteste", parents=[comum])
    t.add_argument("--checkpoint", default=PADRAO_PRINCIPAL)
    t.add_argument("--receita", default="campeao")
    a = ap.parse_args()
    return {"medir": cmd_medir, "capturar": cmd_capturar, "comparar": cmd_comparar, "autoteste": cmd_autoteste, "aa": cmd_aa}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
