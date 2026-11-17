"""(15/11 06h50) A velocidade de cruzeiro nos retos: de pe parado com o pedido desde o comeco (modo do jogo), a velocidade da pelve no rumo
pedido entre 3 e 6 s (media), contra a pedida; e quantos cairam/acabaram antes de 6 s. Uso: TREINO PONTO [CHAVES] [sementes=61,62]"""
import json
import sys
from pathlib import Path

import numpy as np

W = Path("/home/dahaka/Projetos/matter_engine_wip")
D = W / "build-profile/tmp/rolar_r72889"
sys.path.insert(0, str(W / "tools/rl"))
from inspecao import captura, comum  # noqa: E402

treino, ck = sys.argv[1], sys.argv[2]
keys = (sys.argv[3] if len(sys.argv) > 3 else "lev_corrida_0,lev_corrida_1,lev_corrida_4,lev_trote_0").split(",")
seeds = [int(s) for s in (sys.argv[4] if len(sys.argv) > 4 else "61,62").split(",")]
amb = comum.Ambiente(treino, ck)
for key in keys:
    speeds, ended, wanted = [], 0, None
    for seed in seeds:
        npz = captura.capturar(amb, key, "atual", 8, 6.2, D / "distancia" / f"reta_{Path(ck).stem}_{key}_s{seed}", semente=seed, log=lambda *a: None)
        d = np.load(npz)
        meta = json.loads(npz.with_suffix(".json").read_text())
        dt = meta["dt"]
        body, info, ep = d["corpo"].astype(float), d["info"].astype(float), d["episodio"]
        n3, n6 = int(3.0 / dt), int(6.0 / dt)
        for w in range(body.shape[0]):
            wanted = info[w, 0, 3]
            if (ep[w, :n6] != 0).any():
                ended += 1
                continue
            u = np.array([np.cos(info[w, 0, 4]), np.sin(info[w, 0, 4])])
            pel = body[w, n3:n6 + 1, 0, :2]
            speeds.append(float((pel[-1] - pel[0]) @ u / (dt * (pel.shape[0] - 1))))
    s = np.array(speeds)
    print(f"{key} (pedido {wanted:.1f} m/s): cruzeiro 3-6 s {s.mean():.2f} m/s ({s.mean() / wanted:.0%} do pedido; min {s.min():.2f}, max {s.max():.2f}) | "
          f"acabaram antes de 6 s {ended}/{8 * len(seeds)}", flush=True)
