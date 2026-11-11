"""T1 e T2, a postura (19/10; 09-10/11, o desabamento: o trote da r84461 com a coluna +29 graus e a cabeca +42 contra o clipe, o
bom tinha -2,6/+2,2): das posicoes dos elos, corpo menos clipe, no quadro do rumo do quadril (regua_postura de 10/11). Por mundo,
a mediana na janela; com sinal (+ = para a frente / para a esquerda / acima do clipe).

Janelas: retos 2-6 s (a marcha formada, sem a largada); parado 1,5-10 s; paradas o ultimo 1 s (7-8 s, o corpo ja no idle: a
postura em que ele fica depois de frear).

Pisos provisorios (por cenario, na media dos mundos, em modulo), ate o A/A e a referencia fixa, tirados da regua de 10/11 no
diario 42 (trote / corrida de frente, coluna / cabeca / pelve): o melhor ponto, a r68609, -2,6/+2,2/-7,6 cm e +0,2/+5,8; a deriva
que ninguem viu (r74699, 09/11 14h13) +2,8/+8,0 e +9,5/+17,5; o desabamento (r84461) +29,3/+42,3/-11,7 cm e +28,9/+40,4.
Coluna para a frente 6 graus e cabeca 10 passam a r68609 e reprovam a corrida da r74699; a pelve 10 cm passa a r68609 (-7,6, o
corpo corre mais baixo que o clipe) e reprova o desabamento. Coluna de lado 4 graus (a queixa dele de 10/11 14h05, "a coluna
torta para a esquerda, pouca coisa"; sem numero de referencia: o mais provisorio dos quatro). A margem provisoria: 2 graus,
1,5 cm.
"""
from __future__ import annotations

import numpy as np

from suite.medidas.base import Medida, por_mundo, postura

TRACOS = (("coluna_frente", "coluna para a frente (corpo - clipe)", "graus", 6.0, 2.0),
          ("coluna_lado", "coluna de lado (corpo - clipe)", "graus", 4.0, 2.0),
          ("cabeca", "cabeca para a frente (corpo - clipe)", "graus", 10.0, 2.0),
          ("pelve", "altura da pelve (corpo - clipe)", "cm", 10.0, 1.5))


def janela(cap) -> np.ndarray:
    tipo = cap.meta["cenario"]["tipo"]
    if tipo == "reto":
        return cap.janela(2.0, 6.0)
    if tipo == "parado":
        return cap.janela(1.5, cap.meta["cenario"]["segundos"])
    fim = cap.meta["cenario"]["segundos"]
    return cap.janela(fim - 1.0, fim)


def grupo(nome: str) -> str | None:
    if nome.startswith("reto_trote_"):
        return "retos_trote"
    if nome.startswith("reto_corrida_"):
        return "retos_corrida"
    if nome == "parado":
        return "parado"
    if nome.startswith("parada_"):
        return "paradas"
    return None


def medir(caps: dict) -> list[Medida]:
    out: dict[str, Medida] = {}
    for tag, cap in caps.items():
        nome, cond = tag.split("__")
        g = grupo(nome)
        if g is None:
            continue
        m = janela(cap)
        body, clip = postura(cap.corpo), postura(cap.ref)
        grupos = [g] + (["retos_frente"] if nome in ("reto_trote_0", "reto_corrida_0") else [])
        for traco, titulo, unidade, piso, margem in TRACOS:
            d = body[traco] - clip[traco]
            if g == "paradas" and traco == "coluna_lado":
                # (11/11, SUITE_V2: depois de frear vale o idle alerta ou o espelho dele, o mais perto - no espelho o lado troca.)
                e = body[traco] + clip[traco]
                d = np.where(np.abs(e) < np.abs(d), e, d)
            v = por_mundo(d, m)
            for gg in grupos:
                if gg == "retos_frente" and traco != "coluna_lado":
                    continue
                key = f"postura.{traco}.{gg}.{cond}"
                med = out.setdefault(key, Medida(key, "T2" if gg == "retos_frente" else "T1", f"{titulo}, {gg}", unidade,
                                                 "zero", piso=piso, margem=margem))
                med.por_cenario[nome] = v
    return list(out.values())
