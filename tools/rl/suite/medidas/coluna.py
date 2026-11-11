"""T3 as vertebras em Z (10/11, ele: "uma coluna real nao poderia fazer isso"): a torcao de cada junta da coluna (Abdomen, Chest,
UpperChest, Neck) pela telemetria das juntas (body_joint_telemetry: a coluna 9 e o twist em rad, o vetor de rotacao do Jolt -
sonda_juntas_coluna de 10/11), a cada 3 passos. No trote e na corrida de frente (2-6 s) e no parado (1,5-10 s). Por mundo:
  - torcao_p90: o maior, entre as 4 juntas, p90 de |twist| (graus);
  - opostos: % das amostras com duas juntas vizinhas (abdome/peito/peito alto) torcendo em sentidos opostos, as duas acima de
    8 graus - o Z;
  - no_batente: % das amostras com alguma junta no batente duro (>= 97% do maximo do perfil: desde 10/11 o batente e o fim
    anatomico, 7/14/11 graus, e o pescoco 30), como a sonda_juntas_coluna.
"""
from __future__ import annotations

import numpy as np

from suite.medidas.base import COLUNA, IDX, Medida

ZIGUE = 8.0
CENARIOS = {"reto_trote_0": (2.0, 6.0), "reto_corrida_0": (2.0, 6.0), "parado": (1.5, 10.0)}


def valores(cap, t0: float, t1: float) -> dict:
    links = {lk["id"]: lk for lk in cap.meta["ragdoll"]}
    fim = 0.97 * np.array([links[j]["joint"]["axes"][0]["maximumDegrees"] for j in COLUNA], float)
    passos = cap.z["telemetria_passos"]
    tw = np.degrees(cap.z["telemetria"][:, :, [IDX[j] for j in COLUNA], 9].astype(float))
    t = (passos + 1) * cap.dt
    m = cap.vivo()[:, passos] & ((t >= t0) & (t <= t1))[None]
    W = cap.W
    out = {k: np.full(W, np.nan) for k in ("torcao_p90", "opostos", "no_batente")}
    for w in range(W):
        x = tw[w, m[w]]
        if not len(x):
            continue
        out["torcao_p90"][w] = np.percentile(np.abs(x), 90, axis=0).max()
        opp = [(x[:, j] * x[:, j + 1] < 0) & (np.abs(x[:, j]) > ZIGUE) & (np.abs(x[:, j + 1]) > ZIGUE) for j in range(2)]
        out["opostos"][w] = 100.0 * np.mean(opp[0] | opp[1])
        out["no_batente"][w] = 100.0 * np.mean((np.abs(x) >= fim).any(-1))
    return out


DEFS = (("torcao_p90", "torcao: p90 da maior junta", "graus", 2.0),
        ("opostos", "torcao: vizinhas opostas (o Z)", "% das amostras", 5.0),
        ("no_batente", "torcao: no batente duro", "% das amostras", 5.0))


def medir(caps: dict) -> list[Medida]:
    out: dict[str, Medida] = {}
    for tag, cap in caps.items():
        nome, cond = tag.split("__")
        if nome not in CENARIOS:
            continue
        v = valores(cap, *CENARIOS[nome])
        grupo = "parado" if nome == "parado" else "retos_frente"
        for chave, titulo, unidade, margem in DEFS:
            key = f"coluna.{chave}.{grupo}.{cond}"
            med = out.setdefault(key, Medida(key, "T3", f"{titulo}, {grupo}", unidade, "menor", margem=margem))
            med.por_cenario[nome] = v[chave]
    return list(out.values())
