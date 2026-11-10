"""T9 o corpo mole (10/11, ele: "as poses ficaram molengas desengoncadas... ao frear e pos-freio ele fica todo molengo"; sonda_firmeza
de 10/11). Nas paradas (4-8 s), na corrida de frente (2-6 s) e no parado (1,5-10 s), por mundo:
  - tronco_mole: a velocidade angular do peito alto em relacao a pelve (graus/s, media);
  - cabeca: a velocidade angular da cabeca no mundo (graus/s, media);
  - peito_vaivem: o desvio padrao, no tempo, da inclinacao do peito alto (graus).
"""
from __future__ import annotations

import numpy as np

from suite.medidas.base import IDX, Medida, ossos, por_mundo, qang, qconj, qmul, qrot


def valores(cap, t0: float, t1: float) -> dict:
    bones = ossos(cap.corpo.astype(float), cap.meta["ragdoll"])
    rel = qmul(qconj(bones[..., IDX["Pelvis"], :]), bones[..., IDX["UpperChest"], :])
    head = bones[..., IDX["Head"], :]
    dt = cap.dt
    trunk = np.full(rel.shape[:2], np.nan)
    hd = np.full(rel.shape[:2], np.nan)
    trunk[:, 1:] = qang(rel[:, 1:], rel[:, :-1]) / dt
    hd[:, 1:] = qang(head[:, 1:], head[:, :-1]) / dt
    up = qrot(bones[..., IDX["UpperChest"], :], np.array([0.0, 0.0, 1.0]))
    tilt = np.degrees(np.arccos(np.clip(up[..., 2], -1.0, 1.0)))
    m = cap.janela(t0, t1)
    return {"tronco_mole": por_mundo(trunk, m, np.mean), "cabeca": por_mundo(hd, m, np.mean),
            "peito_vaivem": por_mundo(tilt, m, np.std)}


DEFS = (("tronco_mole", "tronco mole (peito alto contra a pelve)", "graus/s", 5.0),
        ("cabeca", "cabeca: velocidade angular media", "graus/s", 5.0),
        ("peito_vaivem", "vaivem da inclinacao do peito", "graus", 1.0))


def medir(caps: dict) -> list[Medida]:
    out: dict[str, Medida] = {}
    for tag, cap in caps.items():
        nome, cond = tag.split("__")
        tipo = cap.meta["cenario"]["tipo"]
        if tipo == "parada":
            grupo, jan = "paradas", (4.0, 8.0)
        elif nome == "reto_corrida_0":
            grupo, jan = "corrida_0", (2.0, 6.0)
        elif nome == "parado":
            grupo, jan = "parado", (1.5, 10.0)
        else:
            continue
        v = valores(cap, *jan)
        for chave, titulo, unidade, margem in DEFS:
            key = f"firmeza.{chave}.{grupo}.{cond}"
            med = out.setdefault(key, Medida(key, "T9", f"{titulo}, {grupo}", unidade, "menor", margem=margem))
            med.por_cenario[nome] = v[chave]
    return list(out.values())
