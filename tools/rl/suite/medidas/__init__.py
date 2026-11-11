"""As medidas da suite: um teste por queixa dele (metodologia_rl.md, secao 3), numpy puro sobre as capturas.

  postura  T1 T2   parado  T7 T8   paradas T9   marcha T4 T5 T6 T10 T12   coluna T3   firmeza T9 (corpo mole)

Fora da SUITE_V1 (a segunda versao): T11 os giros (regua nova: o angulo girado / pedido >= 360 graus, o tremor), T13 o levantar
(chao, de costas, de brucos), T14 os empurroes (parado, trote_0, corrida_0 x 400/1200/2000 N; a forca maxima tolerada), o
direcional da velocidade (pedido x0,8/1,0/1,2), os joelhos colados, a capacidade fixa do treinador (manobras, estrela, comando,
fogo), o melhor valor ja visto e a referencia fixa (r68609) do andar lento.
"""
from __future__ import annotations

from suite.medidas import coluna, firmeza, marcha, parado, paradas, postura

MODULOS = (postura, parado, paradas, marcha, coluna, firmeza)


def medir(caps: dict) -> list:
    out = []
    for mod in MODULOS:
        out += mod.medir(caps)
    return out
