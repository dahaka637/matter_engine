#!/usr/bin/env python3
"""Bateria de qualidade dos gestos de bola (manejo e chute).

Roda os cenarios de bola da sonda (MatterAthleteProbe, o toque de agora:
--manejo 1 --v6 1), le as medidas de cada um e, com --quadros, desenha uma
folha de quadros em volta do primeiro toque (MatterModelInspector quadros).
Compara com uma base aprovada e marca o que piorou: cada mudanca nos gestos
vem com o antes e depois pronto, e uma piora em outro cenario aparece na hora.

uso:
  python3 tools/analysis/gestos_bateria.py <saida> [--conjunto rapido|completo]
      [--base tools/analysis/gestos_base.json] [--salvar-base]
      [--quadros] [--cenarios a,b,c] [--trabalhos 4] [--build build-profile]

Na saida: resultado.json (as medidas), tabela.md (a tabela com a base) e, com
--quadros, <cenario>.png. Variaveis de ambiente (XQT*, ...) passam para a sonda.
"""
import argparse
import concurrent.futures
import json
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BASE_PADRAO = os.path.join(REPO, "tools", "analysis", "gestos_base.json")

# O conjunto rapido: conduzir, rajadas, toques, chutes e voleios.
RAPIDO = [
    "conduz_anda", "conduz_corre", "conduz_sprint", "conduz_lado", "conduz_lado_anda", "conduz_diagonal",
    "conduz_zigue", "conduz_corte", "conduz_inverte", "conduz_volta", "conduz_volta_sprint", "conduz_para",
    "rajada_parado", "rajada_anda", "rajada_corre", "rajada_vira_corre", "continuo_direita_parado",
    "continuo_direita_anda", "continuo_direita_corre", "intercala_corre", "intercala_parado", "para_toca_sai",
    "toque_parado", "toque_lado", "toque_corre", "toque_gira", "toque_ativo", "puxa_lado_parado", "puxa_lado_anda",
    "corte_corre", "atras_fora", "atras_tras", "chute_parado", "chute_corre", "chute_corre_cedo", "chute_lado",
    "chute_gira", "chute_alto_parado", "chute_alto_corre", "voleio_parado", "voleio_toque", "voleio_corre",
    "voleio_lado", "bola_vindo", "bola_rapida",
]

# As medidas: (nome, expressao, maior e pior?, folga para marcar piora).
MEDIDAS = [
    ("pedidos", r"pedidos (\d+)", None, 0),
    ("toques", r"toques (\d+)", False, 0),
    ("louca", r"perna louca (\d+)", True, 2),
    ("joelho", r"joelho virando (\d+)", True, 1),
    ("giro", r"giro max ([\d.]+)", True, 8.0),
    ("voo", r"voo max ([\d.]+)", True, 0.08),
    ("perdidos", r"perdidos (\d+)", True, 0),
    ("folga", r"folga pe-bola na forca med [\d.]+ max ([\d.]+)", True, 0.08),
    ("pe_na_bola", r"pe na bola (\d+)", True, 3),
    ("cruzadas", r"cruzadas (\d+) ticks", True, 2),
    ("ordem", r"ordem trocada pes (\d+)", True, 15),
    ("deslize", r"\|\s*([\d.]+) /\s*[\d.]+ /\s*[\d.]+ \|", True, 0.15),
    ("balancos", r"$^", None, 0),
    ("gestos", r"$^", None, 0),
    ("desvio", r"$^", True, 0.03),
    ("no_teto", r"$^", True, 0),
    ("pico_pernas", r"\|\s*[\d.]+ /\s*[\d.]+ /\s*[\d.]+ \|\s*[\d.]+ /\s*([\d.]+) m/s2", True, 150.0),
]


def lista(sonda):
    out = subprocess.run([sonda, "--lista"], capture_output=True, text=True).stdout
    return [nome for nome in out.split() if nome]


def medir(sonda, inspetor, cenario, saida, quadros):
    dump = os.path.join(saida, cenario + ".dump") if quadros else None
    comando = [sonda, "--manejo", "1", "--v6", "1", "--cenario", cenario]
    if dump:
        comando += ["--dump", dump, "--cada", "1"]
    # (XQT: as linhas do toque, para o tipo de gesto e o desvio do pe.)
    ambiente = dict(os.environ, XQT="1")
    texto = subprocess.run(comando, capture_output=True, text=True, env=ambiente).stdout
    linhas = [l for l in texto.splitlines() if not l.startswith("[")]
    junto = "\n".join(linhas)
    valores = {}
    for nome, expressao, _, _ in MEDIDAS:
        alvo = junto
        if nome == "pico_pernas":
            alvo = next((l for l in linhas if l.startswith(cenario)), "")
        if nome == "deslize":
            alvo = next((l for l in linhas if l.startswith(cenario)), "")
        m = re.search(expressao, alvo)
        if m:
            valores[nome] = float(m.group(1))
    # Os gestos: no balanco da passada ou o gesto inteiro, e o desvio do pe do
    # clipe no balanco (o teto e 0,40 m: no teto o pe nao chega e o corpo senta).
    balancos = [float(d) for d in re.findall(r"inicio balanco .*? desvio ([\d.]+)", junto)]
    valores["balancos"] = float(len(balancos))
    valores["gestos"] = float(len(re.findall(r"inicio gesto ", junto)))
    if balancos:
        valores["desvio"] = round(sum(balancos) / len(balancos), 3)
        valores["no_teto"] = float(sum(1 for d in balancos if d >= 0.395))
    if dump and os.path.exists(dump):
        toque = primeiro_toque(dump, cenario)
        if toque is not None:
            png = os.path.join(saida, cenario + ".png")
            subprocess.run([inspetor, "quadros", "--dump", dump, "--cenario", cenario, "--de", f"{toque - 0.15:.3f}",
                            "--ate", f"{toque + 0.4:.3f}", "--quadros", "8", "--colunas", "8", "--largura", "220",
                            "--altura", "330", "--saida", png], capture_output=True, text=True)
            valores["toque_em"] = toque
        os.remove(dump)
    return cenario, valores


def primeiro_toque(dump, cenario):
    """O primeiro toque depois de 0,4 s (o comeco assenta)."""
    atual = None
    tempo = 0.0
    with open(dump) as arquivo:
        for linha in arquivo:
            if linha.startswith("S "):
                atual = linha[2:].split()[0]
            elif linha.startswith("F "):
                tempo = float(linha.split()[1])
            elif linha.startswith("T") and atual == cenario and tempo > 0.4:
                return tempo
    return None


def tabela(resultado, base):
    nomes = [m[0] for m in MEDIDAS]
    linhas = ["| cenario | " + " | ".join(nomes) + " |", "|" + "---|" * (len(nomes) + 1)]
    pioras = []
    for cenario, valores in resultado.items():
        celulas = []
        anterior = base.get(cenario, {}) if base else {}
        for nome, _, maior_pior, folga in MEDIDAS:
            v = valores.get(nome)
            if v is None:
                celulas.append("-")
                continue
            texto = f"{v:g}"
            a = anterior.get(nome)
            if a is not None and a != v:
                texto = f"{a:g}→{v:g}"
                if maior_pior is not None:
                    pior = (v - a > folga) if maior_pior else (a - v > folga)
                    melhor = (a - v > folga) if maior_pior else (v - a > folga)
                    if pior:
                        texto = "**" + texto + "** ▲"
                        pioras.append(f"{cenario}: {nome} {a:g} → {v:g}")
                    elif melhor:
                        texto += " ✓"
            celulas.append(texto)
        linhas.append(f"| {cenario} | " + " | ".join(celulas) + " |")
    return "\n".join(linhas), pioras


def main():
    p = argparse.ArgumentParser()
    p.add_argument("saida")
    p.add_argument("--conjunto", default="rapido", choices=["rapido", "completo"])
    p.add_argument("--cenarios", default="")
    p.add_argument("--base", default=BASE_PADRAO)
    p.add_argument("--salvar-base", action="store_true")
    p.add_argument("--quadros", action="store_true")
    p.add_argument("--trabalhos", type=int, default=4)
    p.add_argument("--build", default=os.path.join(REPO, "build-profile"))
    a = p.parse_args()
    sonda = os.path.join(a.build, "MatterAthleteProbe")
    inspetor = os.path.join(a.build, "MatterModelInspector")
    os.makedirs(a.saida, exist_ok=True)
    if a.cenarios:
        cenarios = a.cenarios.split(",")
    else:
        cenarios = list(RAPIDO)
        if a.conjunto == "completo":
            cenarios += [c for c in lista(sonda) if c.startswith(("v2_", "bola_")) and c not in cenarios]
    resultado = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.trabalhos) as fila:
        for cenario, valores in fila.map(lambda c: medir(sonda, inspetor, c, a.saida, a.quadros), cenarios):
            resultado[cenario] = valores
    base = {}
    if a.base and os.path.exists(a.base):
        with open(a.base) as arquivo:
            base = json.load(arquivo)
    texto, pioras = tabela(resultado, base)
    with open(os.path.join(a.saida, "resultado.json"), "w") as arquivo:
        json.dump(resultado, arquivo, indent=1, ensure_ascii=False)
    with open(os.path.join(a.saida, "tabela.md"), "w") as arquivo:
        arquivo.write(texto + "\n")
    print(texto)
    # Os totais (o resumo de uma linha para comparar mudancas).
    for nome in ("louca", "cruzadas", "ordem", "perdidos", "pe_na_bola", "balancos", "gestos", "no_teto"):
        agora = sum(v.get(nome, 0) for v in resultado.values())
        antes = sum(base.get(c, {}).get(nome, 0) for c in resultado) if base else None
        print(f"total {nome}: {antes:g} → {agora:g}" if antes is not None else f"total {nome}: {agora:g}")
    if pioras:
        print("\nPIOROU:\n  " + "\n  ".join(pioras))
    if a.salvar_base:
        novo = dict(base)
        novo.update(resultado)
        with open(a.base, "w") as arquivo:
            json.dump(novo, arquivo, indent=1, ensure_ascii=False, sort_keys=True)
        print(f"base salva em {a.base}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
