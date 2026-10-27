#!/usr/bin/env python3
"""A regua do corpo variavel (26/10): as larguras da silhueta de frente por
regiao, em figuras de pe na pose A, e a razao de cada uma sobre a de uma
figura de referencia (o corpo de 80 kg).

Mede igual a folha do usuario (assets/characters/player_model/referencias/
corpo_peso_70_80_90.png, as tres figuras numa imagem) e as imagens do
inspetor (uma figura por imagem: MatterModelInspector folha --pose ligacao
--vistas frente --sem-ossos [--corpo cm,kg]). As linhas de medida sao alturas
relativas (0 no alto da cabeca, 1 no pe do meiao) e cada largura e a da
corrida de pixels que passa pelo meio da regiao (o tronco e as pernas no
centro; o braco do lado direito da imagem).

Uso:
  folha_corpo.py --folha corpo_peso_70_80_90.png --rotulos 70,80,90 --ref 80
  folha_corpo.py --imagens a.png,b.png,c.png --rotulos 70,80,90 --ref 80
  (--json saida.json grava as razoes; --comparar outro.json mostra lado a lado)
"""
import argparse
import json
import sys

import numpy as np
from PIL import Image

# (regiao, altura relativa, o que medir): 'centro' e a corrida que contem o
# meio da figura (pescoco, tronco, quadril); 'pernas' a corrida da perna
# direita da imagem; 'braco' a corrida mais a direita (o braco solto da pose A).
LINHAS = [
    ("cabeca", 0.078, "centro"),
    ("pescoco", 0.139, "centro"),
    ("ombros", 0.248, "centro"),
    ("peito", 0.342, "centro"),
    ("braco", 0.375, "braco"),
    ("cintura", 0.410, "centro"),
    ("antebraco", 0.449, "braco"),
    ("punho", 0.495, "braco"),
    ("quadril", 0.580, "centro"),
    ("coxa", 0.690, "pernas"),
    ("joelho", 0.780, "pernas"),
    ("panturrilha", 0.845, "pernas"),
    ("tornozelo", 0.965, "pernas"),
]


def fundo(img):
    borda = np.concatenate([img[0], img[-1], img[:, 0], img[:, -1]])
    valores, contagem = np.unique(borda.reshape(-1, 3) // 8, axis=0, return_counts=True)
    return valores[contagem.argmax()] * 8 + 4


def mascara(img):
    bg = fundo(img)
    diff = np.abs(img.astype(int) - bg.astype(int)).sum(axis=2)
    return diff > 60


def figuras(mask):
    cols = mask[: int(mask.shape[0] * 0.75)].sum(0)
    out, dentro = [], False
    for x, n in enumerate(cols):
        if n > 15 and not dentro:
            ini, dentro = x, True
        elif n <= 15 and dentro:
            dentro = False
            if x - ini > 60:
                out.append((ini, x))
    if dentro and mask.shape[1] - ini > 60:
        out.append((ini, mask.shape[1]))
    return out


def corridas(linha, emenda=4):
    out, dentro = [], False
    for x, v in enumerate(linha):
        if v and not dentro:
            ini, dentro = x, True
        elif not v and dentro:
            dentro = False
            if x - ini >= 2:
                out.append((ini, x))
    if dentro:
        out.append((ini, len(linha)))
    # Buracos de poucos pixels (o olho, uma costura escura) nao separam.
    juntas = []
    for a, b in out:
        if juntas and a - juntas[-1][1] <= emenda:
            juntas[-1] = (juntas[-1][0], b)
        else:
            juntas.append((a, b))
    return juntas


def medir(img, x0, x1):
    mask = mascara(img)
    branco = (img[..., 0] > 165) & (img[..., 1] > 165) & (img[..., 2] > 165)
    cx = (x0 + x1) // 2
    lo, hi = x0, x1
    cheias = mask[:, lo:hi].sum(1) >= 3
    linhas = np.where(cheias)[0]
    # O alto da cabeca: o comeco do primeiro trecho cheio de 100 linhas (o
    # rotulo de cima da folha e o traco dele ficam de fora).
    topo = int(linhas.min())
    for y in linhas:
        if cheias[y:y + 100].all():
            topo = int(y)
            break
    # O pe do meiao: a ultima linha com uma corrida branca larga (nao a grade
    # do chao, de 2 px) perto do meio (as pernas).
    perto = slice(max(lo, cx - (hi - lo) // 4), min(hi, cx + (hi - lo) // 4))
    meiao = [y for y in range(topo, mask.shape[0])
             if any(b - a >= 6 for a, b in corridas(branco[y, perto]))]
    base = max(meiao) if meiao else int(linhas.max())
    altura = base - topo
    res = {}
    for nome, rel, modo in LINHAS:
        y = int(round(topo + rel * altura))
        # A media de 5 linhas em volta (o serrilhado do facetado).
        medidas = []
        for dy in (-2, -1, 0, 1, 2):
            rs = [(a + lo, b + lo) for a, b in corridas(mask[y + dy, lo:hi]) if b - a >= 6]
            if not rs:
                continue
            if modo == "centro":
                r = [c for c in rs if c[0] <= cx < c[1]]
            elif modo == "pernas":
                r = [c for c in rs if c[0] >= cx - 5]
                r = r[:1]
            else:
                r = rs[-1:]
            if r:
                medidas.append(r[0][1] - r[0][0])
        res[nome] = float(np.median(medidas)) / altura if medidas else float("nan")
    return res, altura


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--folha")
    ap.add_argument("--imagens")
    ap.add_argument("--rotulos", required=True)
    ap.add_argument("--ref", required=True)
    ap.add_argument("--json")
    ap.add_argument("--comparar")
    a = ap.parse_args()
    rotulos = a.rotulos.split(",")
    medidas = []
    if a.folha:
        img = np.asarray(Image.open(a.folha).convert("RGB"))
        figs = figuras(mascara(img))
        if len(figs) != len(rotulos):
            sys.exit(f"{len(figs)} figuras na folha, {len(rotulos)} rotulos")
        medidas = [medir(img, x0, x1)[0] for x0, x1 in figs]
    else:
        for caminho in a.imagens.split(","):
            img = np.asarray(Image.open(caminho).convert("RGB"))
            figs = figuras(mascara(img))
            if not figs:
                sys.exit(f"sem figura em {caminho}")
            x0, x1 = max(figs, key=lambda f: f[1] - f[0])
            medidas.append(medir(img, x0, x1)[0])
    ref = medidas[rotulos.index(a.ref)]
    razoes = {r: {k: m[k] / ref[k] for k in ref} for r, m in zip(rotulos, medidas)}
    outro = json.load(open(a.comparar)) if a.comparar else None
    print("regiao        " + "".join(f"{r:>9}" for r in rotulos) + ("   | comparado" if outro else ""))
    for nome, _, _ in LINHAS:
        linha = f"{nome:<13} " + "".join(f"{razoes[r][nome]:9.3f}" for r in rotulos)
        if outro:
            linha += "   | " + "".join(f"{outro[r][nome]:7.3f}" for r in rotulos if r in outro and nome in outro[r])
        print(linha)
    if a.json:
        json.dump(razoes, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
