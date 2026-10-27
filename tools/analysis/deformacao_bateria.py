#!/usr/bin/env python3
"""Bateria da deformacao do jogador (quadril e ombro).

Roda o `MatterModelInspector analisar` numa bateria de poses dos dois lados
(flexao, extensao, abertura do quadril, agachamento; braco de lado, para cima,
a frente e para tras; instantes de clipes do jogo) e mede, por pose, o que nao
depende da densidade da malha: o comprimento das linhas de cruzamento (cm;
autointersecao de cada parte e uma parte atravessando outra) e a area na
ligacao dos triangulos amassados e esticados (cm2); os virados a vista contam
um a um. Compara com uma base aprovada e marca o que piorou. Com --quadros, desenha uma folha de perto de
cada grupo nas mesmas cameras (os bracos levantados nas do quadril).

uso:
  python3 tools/analysis/deformacao_bateria.py <saida> [--grupos quadril,ombro]
      [--base tools/analysis/deformacao_base.json] [--salvar-base] [--quadros]
      [--build build-profile]

Na saida: resultado.json, tabela.md e, com --quadros, <grupo>.png.
"""
import argparse
import json
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BASE_PADRAO = os.path.join(REPO, "tools", "analysis", "deformacao_base.json")


def quadril(lado):
    coxa, canela = lado + "Thigh", lado + "Shin"
    outra = "Left" if lado == "Right" else "Right"
    abrir = "-45x" if lado == "Right" else "45x"
    return [
        (f"{lado}_flexao30", f"dobra:{coxa}=-30"),
        (f"{lado}_flexao60", f"dobra:{coxa}=-60"),
        (f"{lado}_flexao90", f"dobra:{coxa}=-90+{canela}=90"),
        (f"{lado}_flexao110", f"dobra:{coxa}=-110+{canela}=110"),
        (f"{lado}_extensao20", f"dobra:{coxa}=20"),
        (f"{lado}_extensao40", f"dobra:{coxa}=40"),
        (f"{lado}_abertura45", f"dobra:{coxa}={abrir}"),
        (f"{lado}_flexao60_abertura30", f"dobra:{coxa}=-60+{coxa}={'-30x' if lado == 'Right' else '30x'}"),
        (f"{lado}_passada", f"dobra:{coxa}=-50+{canela}=40+{outra}Thigh=25"),
    ]


def ombro(lado):
    braco = lado + "UpperArm"
    abre = -1 if lado == "Right" else 1  # o sinal do giro em x que abre o braco (-45x: a pose T do direito)
    return [
        (f"{lado}_braco_T", f"dobra:{braco}={45 * abre}x"),
        (f"{lado}_braco_lado135", f"dobra:{braco}={135 * abre}x"),
        (f"{lado}_braco_frente90", f"dobra:{braco}=-90"),
        (f"{lado}_braco_cima150", f"dobra:{braco}=-150"),
        (f"{lado}_braco_tras40", f"dobra:{braco}=40"),
    ]


GRUPOS = {
    "quadril": quadril("Right") + quadril("Left") + [
        ("agachamento", "dobra:RightThigh=-90+LeftThigh=-90+RightShin=120+LeftShin=120"),
    ] + [(f"sprint@{t:.2f}", f"sprint@{t:.2f}") for t in (0.05, 0.15, 0.25, 0.35, 0.45, 0.55)] + [
        # (O chute por dobras: os clipes bola_* sairam com o gesto antigo, 24/10.)
        ("chute_batida", "dobra:RightThigh=-25+RightShin=60+RightFoot=40"),
        ("chute_acompanhamento", "dobra:RightThigh=-85+RightShin=20+RightFoot=35"),
        ("carrinho@0.40", "tackle_slide@0.40"),
    ],
    "ombro": ombro("Right") + ombro("Left") + [
        ("repouso", "repouso"),
        ("sprint_bracos@0.15", "sprint@0.15"), ("sprint_bracos@0.45", "sprint@0.45"),
    ],
}

# As cameras de perto de cada grupo: (alvo, raio, vistas). O lado esquerdo
# espelha o y.
CAMERAS = {
    "quadril": ((-0.011, -0.091, -0.066), 0.30, ("direita", "34c", "34")),
    "ombro": ((-0.068, -0.19, 0.45), 0.25, ("frente", "34", "costas")),
}
BRACOS_ACIMA = "+RightUpperArm=-150+LeftUpperArm=-150"

LINHA = re.compile(r"(AVISO|ERRO|INFO) (\w+): (.*?) na pose (\S+) \((\d+)(?: pares)?(?:; ([\d.]+) cm2?)?\) \(parte (\w+)\)")
CRUZA = re.compile(r"(\w+) atravessa (\w+)")


def medir(inspetor, poses, modelo=()):
    """{pose: {medida: n}} pelo analisar (uma chamada para todas as poses)."""
    texto = subprocess.run([inspetor, "analisar", *modelo, "--poses", ",".join(p for _, p in poses), "--sem-simetria"],
                           capture_output=True, text=True).stdout
    por_pose = {p: n for n, p in poses}
    resultado = {n: {} for n, _ in poses}
    for m in LINHA.finditer(texto):
        _, codigo, descricao, pose, n, medida, parte = m.groups()
        nome = por_pose.get(pose)
        if nome is None:
            continue
        if codigo == "autointersecao_pose":
            chave = "auto_" + parte
        elif codigo == "partes_se_cruzam_pose":
            c = CRUZA.search(descricao)
            chave = "cruza_" + ("_".join(sorted(c.groups())) if c else parte)
        elif codigo in ("amassou_pose", "esticou_pose", "virou_pose"):
            chave = codigo.split("_")[0] + "_" + parte
        else:
            continue
        # Cruzar: cm de linha; amassar e esticar: cm2; virar: um a um.
        valor = int(n) if chave.startswith("virou_") or medida is None else round(float(medida), 1)
        resultado[nome][chave] = round(resultado[nome].get(chave, 0) + valor, 1)
    return resultado


def folha(inspetor, grupo, poses, saida, modelo=()):
    alvo, raio, vistas = CAMERAS[grupo]
    quadros = []
    for nome, pose in poses:
        y = -alvo[1] if nome.startswith("Left") else alvo[1]
        extra = BRACOS_ACIMA if grupo == "quadril" and pose.startswith("dobra:") else ""
        for vista in vistas:
            png = os.path.join(saida, f"_{nome}_{vista}.png".replace("@", "_"))
            subprocess.run([inspetor, "render", *modelo, "--pose", pose + extra, "--vista", vista, "--modo", "solido",
                            "--alvo", f"{alvo[0]},{y},{alvo[2]}", "--raio", str(raio), "--largura", "360",
                            "--altura", "360", "--sem-hud", "--sem-grade", "--sem-ossos", "--saida", png],
                           capture_output=True, text=True)
            quadros += ["-label", f"{nome} {vista}", png]
    destino = os.path.join(saida, grupo + ".png")
    subprocess.run(["montage", *quadros, "-tile", f"{len(vistas)}x", "-geometry", "300x300+2+2", "-pointsize", "13",
                    "-background", "#111", "-fill", "white", destino], capture_output=True)
    for arquivo in quadros[2::3]:
        os.remove(arquivo)
    return destino


def comparar(resultado, base):
    linhas, pioras = [], []
    for grupo, poses in resultado.items():
        chaves = sorted({k for v in poses.values() for k in v} | {k for v in base.get(grupo, {}).values() for k in v})
        linhas += [f"\n## {grupo}\n", "| pose | " + " | ".join(chaves) + " |", "|" + "---|" * (len(chaves) + 1)]
        for nome, valores in poses.items():
            antes = base.get(grupo, {}).get(nome, {})
            celulas = []
            for k in chaves:
                v, a = valores.get(k, 0), antes.get(k, 0) if base else None
                texto = f"{v}"
                if a is not None and a != v:
                    texto = f"{a}→{v}"
                    # Virar piora com qualquer aumento; cruzar com 0,5 cm de linha; amassar e esticar com 2 cm2.
                    folga = 0 if k.startswith("virou_") else 0.5 if k.startswith(("auto_", "cruza_")) else 2.0
                    if v - a > folga:
                        texto = f"**{texto}** ▲"
                        pioras.append(f"{grupo}/{nome}: {k} {a} → {v}")
                    elif a - v > folga:
                        texto += " ✓"
                celulas.append(texto)
            linhas.append(f"| {nome} | " + " | ".join(celulas) + " |")
    return "\n".join(linhas), pioras


def totais(resultado):
    soma = {}
    for grupo, poses in resultado.items():
        for valores in poses.values():
            for k, v in valores.items():
                if k.startswith(("auto_", "cruza_", "virou_")):
                    soma[f"{grupo}/{k}"] = round(soma.get(f"{grupo}/{k}", 0) + v, 1)
    return soma


def main():
    p = argparse.ArgumentParser()
    p.add_argument("saida")
    p.add_argument("--grupos", default="quadril,ombro")
    p.add_argument("--base", default=BASE_PADRAO)
    p.add_argument("--salvar-base", action="store_true")
    p.add_argument("--quadros", action="store_true")
    p.add_argument("--build", default=os.path.join(REPO, "build-profile"))
    p.add_argument("--modelo", default="", help="outro model.json (uma copia com a receita mudada)")
    a = p.parse_args()
    modelo = ("--modelo", a.modelo) if a.modelo else ()
    inspetor = os.path.join(a.build, "MatterModelInspector")
    os.makedirs(a.saida, exist_ok=True)
    grupos = [g for g in a.grupos.split(",") if g]
    resultado = {g: medir(inspetor, GRUPOS[g], modelo) for g in grupos}
    base = {}
    if a.base and os.path.exists(a.base):
        with open(a.base) as arquivo:
            base = json.load(arquivo)
    texto, pioras = comparar(resultado, base)
    with open(os.path.join(a.saida, "resultado.json"), "w") as arquivo:
        json.dump(resultado, arquivo, indent=1, ensure_ascii=False)
    with open(os.path.join(a.saida, "tabela.md"), "w") as arquivo:
        arquivo.write(texto + "\n")
    print(texto)
    agora, antes = totais(resultado), totais({g: base.get(g, {}) for g in grupos}) if base else {}
    for k in sorted(set(agora) | set(antes)):
        print(f"total {k}: {antes.get(k, 0)} → {agora.get(k, 0)}" if base else f"total {k}: {agora.get(k, 0)}")
    if a.quadros:
        for g in grupos:
            print("folha:", folha(inspetor, g, GRUPOS[g], a.saida, modelo))
    if pioras:
        print("\nPIOROU:\n  " + "\n  ".join(pioras))
    if a.salvar_base:
        novo = dict(base)
        novo.update(resultado)
        with open(a.base, "w") as arquivo:
            json.dump(novo, arquivo, indent=1, ensure_ascii=False, sort_keys=True)
        print(f"base salva em {a.base}")
    return 1 if pioras else 0


if __name__ == "__main__":
    sys.exit(main())
