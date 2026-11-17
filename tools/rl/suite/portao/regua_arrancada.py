"""(14/11 20h35, ele: 'a arrancada da corrida: os bracos estabanados e os passinhos parado/curtos') A regua da arrancada, de pe parado com o
pedido desde o comeco, 0-1 s, acao media (modo do jogo): a velocidade da pelve no rumo pedido a 0,5 e 1 s, os pousos dos pes (contato do pe
que comeca) e quantos pousam sem ter andado 15 cm no rumo (passinho no lugar; para tras conta), e as maos contra o peito (m/s, a media das
duas) na rede e no clipe seguido (a referencia da captura). Uso: regua_arrancada.py TREINO PONTO [CHAVES] [sementes=61,62]"""
import json
import sys
from pathlib import Path

import numpy as np

W = Path("/home/dahaka/Projetos/matter_engine_wip")
D = W / "build-profile/tmp/rolar_r72889"
sys.path.insert(0, str(W / "tools/rl"))
from inspecao import captura, comum  # noqa: E402

treino, ck = sys.argv[1], sys.argv[2]
keys = (sys.argv[3] if len(sys.argv) > 3 else "lev_corrida_0,lev_trote_0,lev_corrida_2,lev_corrida_4").split(",")
seeds = [int(s) for s in (sys.argv[4] if len(sys.argv) > 4 else "61,62").split(",")]
amb = comum.Ambiente(treino, ck)
WINDOW = 1.0


def hands_speed(arr, names, idx, dt):
    """A media da velocidade das duas maos contra o peito alto (m/s) nos passos idx."""
    ch = names.index("UpperChest")
    out = []
    for h in ("LeftHand", "RightHand"):
        k = names.index(h)
        rel = arr[idx, k, :3] - arr[idx, ch, :3]
        out.append(np.linalg.norm(np.diff(rel, axis=0), axis=-1) / dt)
    return float(np.mean(out))


for key in keys:
    v05, v10, dist, lands, inplace, hb, hr = [], [], [], [], [], [], []
    for seed in seeds:
        npz = captura.capturar(amb, key, "atual", 8, WINDOW + 0.3, D / "distancia" / f"arrancada_{Path(ck).stem}_{key}_s{seed}", semente=seed,
                               log=lambda *a: None)
        d = np.load(npz)
        meta = json.loads(npz.with_suffix(".json").read_text())
        names, dt = meta["elos"], meta["dt"]
        body, ref, info, ep = d["corpo"].astype(float), d["referencia"].astype(float), d["info"].astype(float), d["episodio"]
        contacts = np.asarray(d["contatos"], float).reshape(-1, 10) if d["contatos"].size else np.zeros((0, 10))
        feet = {names.index("LeftFoot"): "e", names.index("RightFoot"): "d"}
        n = int(round(WINDOW / dt))
        for w in range(body.shape[0]):
            if (ep[w, :n] != 0).any():
                continue
            rumo = info[w, 0, 4]
            u = np.array([np.cos(rumo), np.sin(rumo)])
            pel = body[w, :n + 1, 0, :2]
            vel = np.diff(pel, axis=0) @ u / dt
            v05.append(vel[int(0.4 / dt):int(0.5 / dt)].mean())
            v10.append(vel[int(0.9 / dt):n].mean())
            dist.append(float((pel[n] - pel[0]) @ u))
            touch = {f: np.zeros(n, bool) for f in feet}
            for c in contacts:
                if int(c[0]) == w and int(c[1]) < n and int(c[5]) in feet:
                    touch[int(c[5])][int(c[1])] = True
            count, still = 0, 0
            for f in feet:
                last = body[w, 0, f, :2]
                for i in range(1, n):
                    if touch[f][i] and not touch[f][i - 1]:
                        count += 1
                        if (body[w, i, f, :2] - last) @ u < 0.15:
                            still += 1
                        last = body[w, i, f, :2]
            lands.append(count)
            inplace.append(still)
            idx = np.arange(0, n)
            hb.append(hands_speed(body[w], names, idx, dt))
            if np.isfinite(ref[w, idx]).all():
                hr.append(hands_speed(ref[w], names, idx, dt))
    print(f"{key} (pedido {info[0, 0, 3]:.1f} m/s): vel 0,5 s {np.mean(v05):.2f} m/s, 1 s {np.mean(v10):.2f} m/s, andou {np.mean(dist):.2f} m em 1 s | "
          f"pousos {np.mean(lands):.1f}, passinhos no lugar {np.mean(inplace):.1f} | maos contra o peito {np.mean(hb):.2f} m/s "
          f"(clipe {np.mean(hr) if hr else float('nan'):.2f}) | {len(v05)} largadas", flush=True)
