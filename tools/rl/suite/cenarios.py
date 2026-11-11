"""Os cenarios fixos da suite (SUITE_V1). Mudou um cenario, uma semente, uma janela ou um limiar das medidas: muda a versao, e o
campeao e medido de novo antes de qualquer comparacao (metodologia_rl.md, secao 3).

Os cenarios copiam os das reguas de 10/11 (regua_postura, sonda_meta_parado, sonda_assentar): os retos sao a avaliacao 4 com o
rumo fixo, 7 s sem parada; o parado e a avaliacao 8 sem empurrao, 10 s; as paradas, a avaliacao 4 com a ordem de parar aos 4 s.
O ambiente vem SO de levantar_amp.configurar (a receita do candidato), com a forca auxiliar em zero.
"""
from __future__ import annotations

from dataclasses import dataclass

SUITE_V1 = "SUITE_V1"
VERSAO = SUITE_V1
MUNDOS = 16

# Dois conjuntos de sementes: o 'decisao' so decide (nunca afinar nada nele); o 'diagnostico' e o das reguas de 10/11 (lote 61/501,
# reset 55), que ja serviu para afinar o J/J2. O ruido da exploracao tem um gerador por mundo, semeado por (ruido, mundo).
SEMENTES = {
    "decisao": {"lote": 1001, "lote_parado": 1002, "reset": 1003, "ruido": 1004},
    "diagnostico": {"lote": 61, "lote_parado": 501, "reset": 55, "ruido": 7},
}
CONDICOES = ("det", "ruido")


@dataclass(frozen=True)
class Cenario:
    nome: str
    tipo: str            # reto | parada | parado
    segundos: float
    faixa: int = 0       # 1 trote, 2 corrida
    rumo: int = -1       # 0 a frente, de 45 em 45 graus para a esquerda
    ordem: float = 0.0   # a ordem de parar (s), nas paradas
    equilibrio: tuple = (0.0, 1.0)  # set_balance_drill(fatia, nivel) no parado
    condicoes: tuple = CONDICOES


MARCHAS = {1: "trote", 2: "corrida"}


def cenarios() -> list[Cenario]:
    out = [Cenario(f"reto_{MARCHAS[f]}_{h}", "reto", 7.0, faixa=f, rumo=h) for f in (1, 2) for h in range(8)]
    out.append(Cenario("parado", "parado", 10.0))
    # (A volta a pose, T8: o exercicio de equilibrio do treino no nivel 0,2 - so a media da rede.)
    out.append(Cenario("parado_empurrao", "parado", 10.0, equilibrio=(1.0, 0.2), condicoes=("det",)))
    out += [Cenario(f"parada_{MARCHAS[f]}_{h}", "parada", 8.0, faixa=f, rumo=h, ordem=4.0)
            for f, hs in ((1, (0, 2, 4, 6)), (2, (0, 1, 4, 6))) for h in hs]
    return out


def extras() -> list[Cenario]:
    """(11/11, SUITE_V2) So por --so: as paradas longas, a ordem aos 4 s e 8 s depois dela (no V1, 4 s: nenhum braco assentava,
    a base no fim a 27-29 cm da do idle - a volta ao idle alerta ficava fora da janela)."""
    longas = [Cenario(f"parada_longa_{MARCHAS[f]}_{h}", "parada", 12.0, faixa=f, rumo=h, ordem=4.0)
              for f, hs in ((1, (0, 2, 4, 6)), (2, (0, 1, 4, 6))) for h in hs]
    # (11/11, SUITE_V2: os empurroes andando - os do treino (set_fire, as pancadas e os continuos do nivel FOGO_NIVEL), nos
    # retos de frente e de lado, 8 s: a postura e a firmeza com empurrao - o corpo mole no empurrao, o problema 4 dele.)
    fogo = [Cenario(f"fogo_{MARCHAS[f]}_{h}", "fogo", 8.0, faixa=f, rumo=h) for f in (1, 2) for h in (0, 2)]
    return longas + fogo


FOGO_NIVEL = 0.6


def por_nome() -> dict[str, Cenario]:
    return {c.nome: c for c in cenarios() + extras()}


# Os pares espelhados (o rumo da esquerda e o da direita), dentro de cada marcha.
PARES = ((2, 6), (1, 7), (3, 5))
