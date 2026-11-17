"""(16/11: com a saida - a velocidade da pelve 0,5 s depois de de pe.) Os quatro levantares limpos com a acao media (modo do jogo): levantares.py TREINO PONTO TAG. 4 sementes x 4 mundos limpos cada.
Por exercicio: de pe (cabeca > 1,45 m), o tempo ate ficar de pe e a distancia media ao clipe (cm)."""
import json, sys
from pathlib import Path
import numpy as np
W = Path("/home/dahaka/Projetos/matter_engine_wip"); D = W / "build-profile/tmp/rolar_r72889"
sys.path.insert(0, str(W / "tools/rl"))
from inspecao import comum, captura
treino, ck, tag = sys.argv[1], sys.argv[2], sys.argv[3]
import os
SEMENTES = [int(x) for x in os.environ.get("SEMENTES", "61,62,63,64").split(",")]
amb = comum.Ambiente(treino, ck)
for key in amb.L.LEVANTAR_UNREAL_KEYS:
    ups, times, dist, exits = 0, [], [], []
    for sem in SEMENTES:
        npz = captura.capturar(amb, key, "atual", 4, 3.0, D / "levantares" / f"{treino}_{tag}_{key}_s{sem}", semente=sem, log=lambda *a: None)
        d = np.load(npz); dt = json.loads(npz.with_suffix(".json").read_text())["dt"]
        body, ref, ep = d["corpo"].astype(float), d["referencia"].astype(float), d["episodio"]
        for w in range(4):
            idx = np.where(ep[w] == 0)[0]
            hz = body[w, idx, 5, 2]
            k = next((j for j in range(len(idx)) if hz[j] > 1.45), None)
            if k is not None:
                ups += 1; times.append((idx[k] + 1) * dt)
                # (16/11, ele: os levantares "obedecerem a velocidade dos clipes professores") A saida: a velocidade da pelve no plano nos
                # 0,5 s depois de de pe (o clipe correndo sai a ~4,4 m/s).
                n = int(round(0.5 / dt))
                if k + n < len(idx):
                    p0, p1 = body[w, idx[k], 0, :2], body[w, idx[k + n], 0, :2]
                    exits.append(float(np.linalg.norm(p1 - p0)) / (n * dt))
            ok = [i for i in idx if np.isfinite(ref[w, i]).all()]
            if ok:
                b = body[w, ok, :, :3] - body[w, ok, :1, :3]; r = ref[w, ok, :, :3] - ref[w, ok, :1, :3]
                dist.append(100 * float(np.linalg.norm(b - r, axis=-1).mean()))
    print(f"{tag} {key}: de pe {ups}/{4 * len(SEMENTES)}, tempo {np.mean(times) if times else float('nan'):.2f} s, distancia ao clipe {np.mean(dist):.1f} cm, "
          f"saida {np.mean(exits) if exits else float('nan'):.2f} m/s", flush=True)
