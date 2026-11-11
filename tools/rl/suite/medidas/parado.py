"""T7 o parado (10/11, ele: "ficar paradinho corretinho no IDLE", "os pes deslizando", "sem vontade de voltar a pose"; a meta: o
clipe fixo no mundo seguido ~99%, sem deslizar nem oscilar) e T8 a volta a pose (o parado com o empurrao leve do treino).

O erro de pose e calculado AQUI, contra o idle preso no mundo onde o parado comecou (base.ancorar), e nao pela nota do treino
(a semelhanca das sondas antigas era a coluna 0 do locomotion_feedback, a mesma nota que o J/J2 treinam). Janela 1,5-10 s.
Por mundo:
  - erro_pose: a media no tempo do angulo medio dos 18 elos contra o idle fixo (graus);
  - pes: a distancia no chao de cada pe ao do idle fixo (cm, media dos dois); deriva_pelve: o maximo da pelve no chao (cm);
  - tolerancia: a fracao do tempo com erro_pose <= 8 graus, pes <= 3 cm e pelve <= 5 cm (limiares da SUITE_V1);
  - deslize: o quanto cada pe andou no chao enquanto apoiado nos dois passos seguidos (cm em 8,5 s, media dos dois pes);
  - agitacao: a velocidade angular media dos elos (graus/s); bracos: a dos 6 elos dos bracos (graus/s) e o erro deles contra o
    idle (graus);
  - passos: os pousos de pe; quedas: o episodio que acaba antes dos 10 s;
  - minimo (o teste minimo de Ribeiro): de pe, sem passo e cada pe deslizando <= 2 cm nos 8,5 s.

Pisos provisorios (do parado sem empurrao; a meta dele e 99% e zero passo): tolerancia >= 0,80, deslize <= 3 cm, passos <= 1,
deriva <= 10 cm, quedas 0, minimo >= 0,75.
"""
from __future__ import annotations

import numpy as np

from suite.medidas.base import BRACOS, IDX, PES, Medida, ancorar, pousos, por_mundo, qang, velocidade_angular

TOL_POSE, TOL_PES, TOL_PELVE = 8.0, 3.0, 5.0
MINIMO_DESLIZE = 2.0
INICIO = 1.5

# (chave, titulo, unidade, sentido, tipo, piso, margem)
DEFS = (("tolerancia", "tempo dentro da tolerancia do idle fixo", "fracao", "maior", "continua", 0.80, 0.05),
        ("erro_pose", "erro de pose contra o idle fixo (media dos elos)", "graus", "menor", "continua", None, 1.0),
        ("pes", "pes contra os do idle fixo", "cm", "menor", "continua", None, 1.0),
        ("deriva_pelve", "deriva maxima da pelve no chao", "cm", "menor", "continua", 10.0, 2.0),
        ("deslize", "deslize de cada pe apoiado", "cm", "menor", "continua", 3.0, 1.0),
        ("agitacao", "agitacao (velocidade angular media dos elos)", "graus/s", "menor", "continua", None, 5.0),
        ("bracos_agitacao", "bracos: velocidade angular media", "graus/s", "menor", "continua", None, 5.0),
        ("bracos_erro", "bracos: erro contra o idle fixo", "graus", "menor", "continua", None, 2.0),
        ("passos", "passos (pousos de pe)", "por mundo", "menor", "continua", 1.0, 0.5),
        ("quedas", "quedas (fim antes dos 10 s)", "taxa", "menor", "taxa", 0.0, 0.07),
        ("minimo", "minimo: 8,5 s sem passo e sem deslizar", "taxa", "maior", "taxa", 0.75, 0.10))


def series(cap) -> dict:
    """As series por passo [W, T] do parado contra o idle fixo, e a mascara da janela."""
    seg = cap.meta["cenario"]["segundos"]
    m = cap.janela(INICIO, seg)
    fixo = ancorar(cap.ref, cap.vivo(), cap.corpo)
    body = cap.corpo.astype(float)
    ang = qang(body[..., 3:7], fixo[..., 3:7])
    pes = np.linalg.norm(body[:, :, PES, :2] - fixo[:, :, PES, :2], axis=-1).mean(-1) * 100.0
    pelve = np.linalg.norm(body[:, :, IDX["Pelvis"], :2] - fixo[:, :, IDX["Pelvis"], :2], axis=-1) * 100.0
    chao = cap.pes_no_chao()
    passo_pe = np.zeros(chao.shape)
    passo_pe[:, 1:] = np.linalg.norm(np.diff(body[:, :, PES, :2], axis=1), axis=-1) * 100.0
    apoiado = np.zeros(chao.shape, bool)
    apoiado[:, 1:] = chao[:, 1:] & chao[:, :-1] & (m[:, 1:] & m[:, :-1])[..., None]
    return {"m": m, "erro_pose": ang.mean(-1), "pes": pes, "pelve": pelve,
            "dentro": (ang.mean(-1) <= TOL_POSE) & (pes <= TOL_PES) & (pelve <= TOL_PELVE),
            "deslize": np.where(apoiado, passo_pe, 0.0).sum(1).mean(-1),
            "agitacao": velocidade_angular(body, cap.dt).mean(-1),
            "bracos_agitacao": velocidade_angular(body, cap.dt, BRACOS).mean(-1),
            "bracos_erro": ang[..., list(BRACOS)].mean(-1),
            "passos": pousos(chao, m), "caiu": cap.caiu().astype(float), "fixo": fixo}


def valores(cap) -> dict:
    s = series(cap)
    m = s["m"]
    caiu = s["caiu"]
    out = {"tolerancia": por_mundo(s["dentro"].astype(float), m, np.mean),
           "erro_pose": por_mundo(s["erro_pose"], m, np.mean),
           "pes": por_mundo(s["pes"], m, np.mean),
           "deriva_pelve": por_mundo(s["pelve"], m, np.max),
           "deslize": s["deslize"],
           "agitacao": por_mundo(s["agitacao"], m, np.mean),
           "bracos_agitacao": por_mundo(s["bracos_agitacao"], m, np.mean),
           "bracos_erro": por_mundo(s["bracos_erro"], m, np.mean),
           "passos": s["passos"], "quedas": caiu}
    out["minimo"] = ((caiu == 0) & (s["passos"] == 0) & (s["deslize"] <= MINIMO_DESLIZE)).astype(float)
    return out


def medir(caps: dict) -> list[Medida]:
    out = []
    for tag, cap in caps.items():
        nome, cond = tag.split("__")
        if cap.meta["cenario"]["tipo"] != "parado":
            continue
        v = valores(cap)
        empurrao = nome != "parado"
        for chave, titulo, unidade, sentido, tipo, piso, margem in DEFS:
            if empurrao and chave == "minimo":
                continue
            key = f"parado.{chave}.{nome}.{cond}"
            out.append(Medida(key, "T8" if empurrao else "T7", f"{titulo}, {nome}", unidade, sentido, {nome: v[chave]}, tipo,
                              None if empurrao else piso, margem))
    return out
