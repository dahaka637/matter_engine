"""Os retos (os 8 rumos no trote e na corrida, 7 s sem parada):
  T6 a velocidade travada (07/11): o avanco do centro de massa no rumo pedido dividido pelo pedido real (a linha Q do clipe, por
     passo - nao uma tabela), 2-6 s; e as quedas;
  T5 a meia-volta e o peito para a vista (01/11, 03/11): o rumo do peito alto contra o do clipe no mesmo passo, em modulo (o clipe
     ja poe o peito na vista; nos rumos de lado e nas diagonais o proprio clipe fica 13-15 graus fora dela, medido na SUITE_V1,
     entao a vista crua reprovaria o clipe); e o desvio do rumo do deslocamento (graus, + = esquerda);
  T4 o vies para a direita (05/11): nos pares espelhados 2x6, 1x7 e 3x5, por mundo, a diferenca da razao de velocidade, a soma
     dos desvios (no espelho um e o negativo do outro) e a diferenca das quedas. E o teste invariante (Ribeiro 2020);
  T12 a cabeca agitada (24/10): o p95 da velocidade angular da cabeca (graus/s), 2-6 s;
  T10 a largada (10/11), so na corrida, 0-1 s: o avanco aos 1,0 s (m/s, media de 0,9 a 1,1 s), os pousos e o tempo com os dois
     pes no ar (s).
"""
from __future__ import annotations

import numpy as np

from suite.cenarios import PARES
from suite.medidas.base import IDX, Medida, centro_massa, pousos, por_mundo, rumo_peito, velocidade_angular, wrap

def series(cap) -> dict:
    dt = cap.dt
    com = centro_massa(cap.corpo, cap.geo())
    v = np.full(com.shape[:2] + (2,), np.nan)
    v[:, 1:] = np.diff(com[..., :2], axis=1) / dt
    info = cap.z["info"].astype(float)
    r, ped = info[..., 4], info[..., 3]
    along = v[..., 0] * np.cos(r) + v[..., 1] * np.sin(r)
    lat = -v[..., 0] * np.sin(r) + v[..., 1] * np.cos(r)
    return {"along": along, "lat": lat, "pedido": ped,
            "peito": np.abs(wrap(np.degrees(rumo_peito(cap.corpo) - rumo_peito(cap.ref)))),
            "cabeca": velocidade_angular(cap.corpo, dt, (IDX["Head"],))[..., 0]}


def valores(cap) -> dict:
    s = series(cap)
    m = cap.janela(2.0, 6.0)
    along = por_mundo(s["along"], m, np.mean)
    lat = por_mundo(s["lat"], m, np.mean)
    out = {"razao": along / por_mundo(s["pedido"], m, np.mean),
           "desvio": np.degrees(np.arctan2(lat, along)),
           "peito": por_mundo(s["peito"], m, np.mean),
           "cabeca_p95": por_mundo(s["cabeca"], m, lambda x: np.percentile(x, 95)),
           "quedas": cap.caiu().astype(float)}
    if cap.meta["cenario"]["faixa"] == 2:
        out["largada_vel"] = por_mundo(s["along"], cap.janela(0.9, 1.1), np.mean)
        first = cap.janela(0.0, 1.0)
        out["largada_passos"] = pousos(cap.pes_no_chao(), first)
        no_ar = ~cap.pes_no_chao().any(-1)
        out["largada_voo"] = np.where(first, no_ar, False).sum(1) * cap.dt
    return out


DEFS = (("razao", "T6", "velocidade / pedido", "razao", "maior", "continua", 0.05),
        ("quedas", "T6", "quedas", "taxa", "menor", "taxa", 0.05),
        ("peito", "T5", "peito contra o do clipe", "graus", "menor", "continua", 3.0),
        ("desvio", "T5", "desvio do rumo", "graus", "zero", "continua", 2.0),
        ("cabeca_p95", "T12", "cabeca: p95 da velocidade angular", "graus/s", "menor", "continua", 15.0),
        ("largada_vel", "T10", "largada: avanco aos 1,0 s", "m/s", "maior", "continua", 0.2),
        ("largada_passos", "T10", "largada: pousos no 1o segundo", "por mundo", "menor", "continua", 0.5),
        ("largada_voo", "T10", "largada: tempo com os dois pes no ar no 1o segundo", "s", "menor", "continua", 0.03))


def medir(caps: dict) -> list[Medida]:
    out: dict[str, Medida] = {}
    vals = {}
    for tag, cap in caps.items():
        nome, cond = tag.split("__")
        if cap.meta["cenario"]["tipo"] != "reto":
            continue
        v = vals[tag] = valores(cap)
        marcha = nome.split("_")[1]
        for chave, teste, titulo, unidade, sentido, tipo, margem in DEFS:
            if chave not in v:
                continue
            key = f"retos.{chave}.{marcha}.{cond}"
            med = out.setdefault(key, Medida(key, teste, f"{titulo}, retos do {marcha}", unidade, sentido, tipo=tipo, margem=margem))
            med.por_cenario[nome] = v[chave]
    # (T4: os pares espelhados, por mundo - os mesmos comecos nos dois rumos do par.)
    for cond in ("det", "ruido"):
        for chave, titulo, f, margem in (("razao", "diferenca da velocidade / pedido", lambda a, b: a - b, 0.05),
                                         ("desvio", "soma dos desvios do rumo", lambda a, b: a + b, 2.0),
                                         ("quedas", "diferenca das quedas", lambda a, b: a - b, 0.07)):
            key = f"espelho.{chave}.{cond}"
            for marcha in ("trote", "corrida"):
                for a, b in PARES:
                    ta, tb = f"reto_{marcha}_{a}__{cond}", f"reto_{marcha}_{b}__{cond}"
                    if ta in vals and tb in vals:
                        med = out.setdefault(key, Medida(key, "T4", f"espelho: {titulo} (esquerda - direita)",
                                                         "graus" if chave == "desvio" else "razao" if chave == "razao" else "taxa",
                                                         "zero", margem=margem))
                        med.por_cenario[f"{marcha}_{a}x{b}"] = f(vals[ta][chave], vals[tb][chave])
    return list(out.values())
