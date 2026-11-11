"""T9 as paradas (10/11, ele: "ao frear ele fica num equilibrio amador desnecessario em vez de recobrar o idle alerta assim que
possivel, com o minimo de passos"): a ordem de parar aos 4 s, o episodio de 8 s (sonda_assentar e regua_parada de 10/11). Por
mundo:
  - quedas: o episodio que acaba antes dos 8 s (antes e depois da ordem, separadas);
  - parou: o primeiro instante depois da ordem com a pelve (media movel de 5 passos) abaixo de 0,3 m/s; o tempo ate ele (s) e
    os metros andados da ordem ate ele;
  - passos depois de parar: os pousos de pe dali em diante;
  - assentou no idle: o primeiro instante, depois de parar, mantido 0,5 s, com a base (os pes em relacao a pelve, no quadro de
    cada pelve) a menos de 8 cm da do clipe, a pelve e o peito girados menos de 10 graus e o peito inclinado menos de 8 graus
    (limiares de 10/11, versionados aqui); a taxa e o tempo desde parar (s);
  - o ultimo 1 s: a base (cm), o giro da pelve e do peito contra o clipe (graus).
Sem pisos na SUITE_V1 (so as margens provisorias); a postura do ultimo 1 s entra na T1 (postura.*.paradas).
"""
from __future__ import annotations

import numpy as np

from suite.medidas.base import IDX, PES, Medida, ossos, pousos, qrot, wrap

PAROU_MS, ASSENTA_BASE, ASSENTA_GIRO, ASSENTA_PEITO, ASSENTA_S = 0.3, 8.0, 10.0, 8.0, 0.5

DEFS = (("quedas", "quedas", "taxa", "menor", "taxa", 0.05),
        ("quedas_antes", "quedas antes da ordem", "taxa", "menor", "taxa", 0.05),
        ("quedas_depois", "quedas depois da ordem", "taxa", "menor", "taxa", 0.05),
        ("tempo_parar", "tempo ate parar", "s", "menor", "continua", 0.15),
        ("metros_parar", "metros ate parar", "m", "menor", "continua", 0.2),
        ("passos_depois", "passos depois de parar", "por mundo", "menor", "continua", 0.5),
        ("assentou", "assentou no idle", "taxa", "maior", "taxa", 0.05),
        ("tempo_assentar", "tempo de parar a assentar", "s", "menor", "continua", 0.2),
        ("base_fim", "base contra o clipe no ultimo 1 s", "cm", "menor", "continua", 1.0),
        ("pelve_fim", "giro da pelve contra o clipe no ultimo 1 s", "graus", "menor", "continua", 2.0),
        ("peito_fim", "giro do peito contra o clipe no ultimo 1 s", "graus", "menor", "continua", 2.0))


def quadro_pelve(fr, bones):
    """O centro e a base (frente, esquerda, cima) da pelve no chao, pelo eixo lateral do osso dela."""
    lat = qrot(bones[..., IDX["Pelvis"], :], np.array([0.0, 1.0, 0.0]))
    lat[..., 2] = 0.0
    lat /= np.maximum(np.linalg.norm(lat, axis=-1, keepdims=True), 1e-9)
    up = np.broadcast_to(np.array([0.0, 0.0, 1.0]), lat.shape)
    return fr[..., IDX["Pelvis"], :3], np.stack([np.cross(lat, up), lat, up], -2)


def base(fr, bones) -> np.ndarray:
    """[..., 2, 2] os pes no chao em relacao a pelve, no quadro dela."""
    c, B = quadro_pelve(fr, bones)
    rel = fr[..., list(PES), :3] - c[..., None, :]
    return np.einsum("...ij,...kj->...ki", B, rel)[..., :2]


def giro(bones, elo) -> np.ndarray:
    lat = qrot(bones[..., IDX[elo], :], np.array([0.0, 1.0, 0.0]))
    return np.degrees(np.arctan2(lat[..., 1], lat[..., 0]))


def series(cap) -> dict:
    links = cap.meta["ragdoll"]
    fb, fr = cap.corpo.astype(float), cap.ref.astype(float)
    bb, br = ossos(fb, links), ossos(fr, links)
    up = qrot(bb[..., IDX["UpperChest"], :], np.array([0.0, 0.0, 1.0]))
    # (11/11, SUITE_V2: o idle alerta e o espelho dele valem igual - o treino aceita o mais perto, e os dois tem o peito na vista
    # com a pelve a +-22 graus (diario 43). O espelho da referencia: os pes trocados com o lado negado, no quadro da pelve, e a
    # pelve refletida no rumo do peito dela. Vale o mais perto dos dois.)
    bc, br_base = base(fb, bb), base(fr, br)
    espelho = br_base[..., ::-1, :] * np.array([1.0, -1.0])
    pc, pr, cr = giro(bb, "Pelvis"), giro(br, "Pelvis"), giro(br, "UpperChest")
    return {"base": np.minimum(np.linalg.norm(bc - br_base, axis=-1).mean(-1), np.linalg.norm(bc - espelho, axis=-1).mean(-1)) * 100.0,
            "pelve": np.minimum(np.abs(wrap(pc - pr)), np.abs(wrap(pc - (2.0 * cr - pr)))),
            "peito": np.abs(wrap(giro(bb, "UpperChest") - giro(br, "UpperChest"))),
            "inclinacao": np.degrees(np.arccos(np.clip(up[..., 2], -1.0, 1.0))),
            "xy": fb[..., IDX["Pelvis"], :2]}


def valores(cap) -> dict:
    ordem, seg, dt = cap.meta["cenario"]["ordem"], cap.meta["cenario"]["segundos"], cap.dt
    s = series(cap)
    t, vivo, fim = cap.t(), cap.vivo(), cap.fim()
    chao = cap.pes_no_chao()
    W = cap.W
    out = {k[0]: np.full(W, np.nan) for k in DEFS}
    caiu = cap.caiu()
    out["quedas"] = caiu.astype(float)
    out["quedas_antes"] = (caiu & (fim < ordem)).astype(float)
    out["quedas_depois"] = (caiu & (fim >= ordem)).astype(float)
    hold = int(round(ASSENTA_S / dt))
    ok_pose = (s["base"] < ASSENTA_BASE) & (s["pelve"] < ASSENTA_GIRO) & (s["peito"] < ASSENTA_GIRO) & (s["inclinacao"] < ASSENTA_PEITO)
    for w in range(W):
        idx = np.where(vivo[w])[0]
        if idx.size < 10 or caiu[w]:
            continue
        xy = s["xy"][w, idx]
        sp = np.r_[0.0, np.linalg.norm(np.diff(xy, axis=0), axis=-1) / dt]
        sm = np.convolve(sp, np.ones(5) / 5, mode="same")
        depois = np.where((t[idx] > ordem) & (sm < PAROU_MS))[0]
        out["assentou"][w] = 0.0
        if not depois.size:
            continue
        k = depois[0]
        out["tempo_parar"][w] = t[idx[k]] - ordem
        k0 = np.searchsorted(t[idx], ordem)
        out["metros_parar"][w] = float(np.linalg.norm(np.diff(xy[k0:k + 1], axis=0), axis=-1).sum())
        mask = np.zeros_like(vivo)
        mask[w, idx[k:]] = True
        out["passos_depois"][w] = pousos(chao, mask)[w]
        ok = ok_pose[w, idx[k:]]
        run = np.convolve(ok.astype(float), np.ones(hold), mode="valid") >= hold - 1e-9 if ok.size >= hold else np.zeros(0, bool)
        if run.any():
            out["assentou"][w] = 1.0
            out["tempo_assentar"][w] = t[idx[k + int(np.argmax(run))]] - t[idx[k]]
        last = idx[t[idx] >= seg - 1.0]
        out["base_fim"][w] = s["base"][w, last].mean()
        out["pelve_fim"][w] = s["pelve"][w, last].mean()
        out["peito_fim"][w] = s["peito"][w, last].mean()
    return out


def medir(caps: dict) -> list[Medida]:
    out: dict[str, Medida] = {}
    for tag, cap in caps.items():
        nome, cond = tag.split("__")
        if cap.meta["cenario"]["tipo"] != "parada":
            continue
        v = valores(cap)
        for chave, titulo, unidade, sentido, tipo, margem in DEFS:
            key = f"paradas.{chave}.{cond}"
            med = out.setdefault(key, Medida(key, "T9", f"paradas: {titulo}", unidade, sentido, tipo=tipo, margem=margem))
            med.por_cenario[nome] = v[chave]
    return list(out.values())
