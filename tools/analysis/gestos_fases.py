#!/usr/bin/env python3
"""A regua dos gestos em varias fases da passada (28/10).

A bateria (gestos_bateria.py) mede cada cenario numa fase so: o gesto cai
sempre no mesmo ponto da passada, e qualquer mudanca na fase (a arrancada, a
cadencia) muda o resultado sem dizer se piorou. Aqui cada cenario roda com a
bola e os pedidos de bola deslocados (XGESTO_ATRASO, a caminhada igual) por
varias fases, nas duas variantes (a de agora e a "antes" por uma chave de A/B
da sonda), e soma os defeitos.

uso:
  python3 tools/analysis/gestos_fases.py cenario1,cenario2,...
      [--antes XSEMARRANCADA] (a variavel que liga a variante de antes; vazio: so a de agora)
      [--fases N] (padrao 12: N atrasos iguais dentro de UM ciclo da passada de cada
          cenario, medido pela sonda no ultimo pedido de bola andando; numa janela fixa
          as fases ficavam desiguais e a soma enganava - 28/10)
      [--atrasos 0,0.05,...] (s; fixos, no lugar das fases)
      [--chave XGESTO_ATRASO] (ou XATRASO: atrasa a caminhada em vez do gesto)
      [--por-atraso] (louca/pe_na_bola/pico de cada fase)
"""
import argparse
import os
import re
import subprocess
from concurrent.futures import ThreadPoolExecutor

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MEDIDAS = [("louca", r"perna louca (\d+)"), ("pe_bola", r"pe na bola (\d+)"),
           ("pico", r"\|\s*[\d.]+ /\s*[\d.]+ /\s*[\d.]+ \|\s*[\d.]+ /\s*([\d.]+) m/s2"),
           ("cruzadas", r"cruzadas (\d+) ticks"), ("perdidos", r"perdidos (\d+)"),
           ("deslize", r"\|\s*([\d.]+) /\s*[\d.]+ /\s*[\d.]+ \|")]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("cenarios")
    p.add_argument("--antes", default="XSEMARRANCADA")
    p.add_argument("--fases", type=int, default=12)
    p.add_argument("--atrasos", default="")
    p.add_argument("--chave", default="XGESTO_ATRASO")
    p.add_argument("--por-atraso", action="store_true")
    p.add_argument("--build", default=os.path.join(REPO, "build-profile"))
    a = p.parse_args()
    cenarios = a.cenarios.split(",")
    variantes = ["antes", "agora"] if a.antes else ["agora"]
    sonda = os.path.join(a.build, "MatterAthleteProbe")

    def ciclo(cenario):
        # O ciclo da passada e o primeiro pedido (a versao de agora, sem atraso).
        env = dict(os.environ)
        for chave in ("XATRASO", "XGESTO_ATRASO", a.antes):
            env.pop(chave, None)
        out = subprocess.run([sonda, "--cenario", cenario], cwd=REPO, env=env, capture_output=True, text=True).stdout
        m = re.search(r"ciclo da passada (-?[\d.]+) s", out)
        primeiro = re.search(r"sequencia p@([\d.]+)", out)
        return (float(m.group(1)) if m else -1.0), (float(primeiro.group(1)) if primeiro else -1.0)

    with ThreadPoolExecutor(6) as ex:
        ciclos = dict(zip(cenarios, ex.map(ciclo, cenarios)))
    atrasos = {}
    for c in cenarios:
        periodo, primeiro = ciclos[c]
        if a.atrasos:
            atrasos[c] = a.atrasos.split(",")
        elif periodo > 0.0:
            atrasos[c] = [f"{periodo * k / a.fases:.4f}" for k in range(a.fases)]
        else:
            atrasos[c] = []
            print(f"{c}: sem ciclo da passada nos pedidos (fica fora)")
        if 0.0 <= primeiro < 0.8:
            print(f"{c}: o primeiro pedido em {primeiro:.2f} s cai na arrancada (o atraso o leva para dentro dela)")

    def roda(job):
        cenario, variante, atraso = job
        env = dict(os.environ)
        for chave in ("XATRASO", "XGESTO_ATRASO", a.antes):
            env.pop(chave, None)
        if float(atraso) > 0.0:
            env[a.chave] = atraso
        if variante == "antes":
            env[a.antes] = "1"
        out = subprocess.run([sonda, "--cenario", cenario], cwd=REPO, env=env, capture_output=True, text=True).stdout
        medidas = {}
        for nome, rx in MEDIDAS:
            m = re.search(rx, out)
            medidas[nome] = float(m.group(1)) if m else None
        return cenario, variante, float(atraso), medidas

    jobs = [(c, v, t) for c in cenarios for v in variantes for t in atrasos[c]]
    with ThreadPoolExecutor(6) as ex:
        res = list(ex.map(roda, jobs))
    print(f"{len(cenarios)} cenarios, {len(jobs) // max(1, len(variantes))} rodadas por variante ({a.chave})")
    for v in variantes:
        col = {n: [r[3][n] for r in res if r[1] == v and r[3][n] is not None] for n, _ in MEDIDAS}
        picos = sorted(col["pico"])
        print(f"{v}: louca {sum(col['louca']):.0f} (casos {sum(1 for x in col['louca'] if x > 0)}) | "
              f"pe na bola {sum(col['pe_bola']):.0f} (casos {sum(1 for x in col['pe_bola'] if x > 0)}) | "
              f"cruzadas {sum(col['cruzadas']):.0f} | perdidos {sum(col['perdidos']):.0f} | "
              f"pico med {picos[len(picos) // 2]:.0f} max {picos[-1]:.0f} | "
              f"deslize med {sum(col['deslize']) / max(1, len(col['deslize'])):.3f}")
    for c in cenarios:
        linha = c
        for v in variantes:
            rows = [r[3] for r in res if r[0] == c and r[1] == v]
            linha += (f" | {v}: louca {sum(r['louca'] or 0 for r in rows):.0f}"
                      f" pe_bola {sum(r['pe_bola'] or 0 for r in rows):.0f}"
                      f" picomax {max(r['pico'] or 0 for r in rows):.0f}")
        print(linha)
        if a.por_atraso:
            for v in variantes:
                rows = sorted((r[2], r[3]) for r in res if r[0] == c and r[1] == v)
                print(f"  {v}: " + " ".join(f"{t:.2f}:{int(m['louca'] or 0)}/{int(m['pe_bola'] or 0)}/{int(m['pico'] or 0)}"
                                           for t, m in rows))


if __name__ == "__main__":
    main()
