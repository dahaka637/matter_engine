"""(16/11 21h30, ele: "nao e questao de SIMETRIA, e seguir a porra do clipe") A distancia de cada parte do corpo ao clipe na locomocao: as
andaduras limpas com a acao media (modo do jogo), do segundo 2 ao fim, cada elo com a pelve na origem contra o quadro do clipe do mesmo
instante (o que a nota do rastreio segue). Por andadura: a media de todos os elos e as das pernas, dos bracos e do tronco (cm).
distancia_clipe_loco.py TREINO PONTO TAG. A ultima linha, "DISTLOCO media X", e a do portao."""
import json, os, sys
from pathlib import Path
import numpy as np
W = Path("/home/dahaka/Projetos/matter_engine_wip"); D = W / "build-profile/tmp/rolar_r72889"
sys.path.insert(0, str(W / "tools/rl"))
from inspecao import comum, captura
treino, ck, tag = sys.argv[1], sys.argv[2], sys.argv[3]
SEMENTES = [int(x) for x in os.environ.get("SEMENTES", "61,62").split(",")]
GAITS = ("lev_trote_0", "lev_corrida_0", "lev_corrida_1", "lev_trote_2", "lev_trote_4", "lev_corrida_4")
amb = comum.Ambiente(treino, ck)
all_means = []
for key in GAITS:
    per_link = None
    names = []
    for sem in SEMENTES:
        npz = captura.capturar(amb, key, "atual", 2, 6.0, D / "dist_loco" / f"{treino}_{tag}_{key}_s{sem}", semente=sem, log=lambda *a: None)
        d = np.load(npz); meta = json.loads(npz.with_suffix(".json").read_text()); dt = meta["dt"]
        names = meta.get("elos") or names
        body, ref, ep = d["corpo"].astype(float), d["referencia"].astype(float), d["episodio"]
        for w in range(body.shape[0]):
            idx = np.where(ep[w] == 0)[0]; idx = idx[int(2.0 / dt):]
            ok = [i for i in idx if np.isfinite(ref[w, i]).all()]
            if not ok:
                continue
            b = body[w, ok, :, :3] - body[w, ok, :1, :3]; r = ref[w, ok, :, :3] - ref[w, ok, :1, :3]
            e = 100 * np.linalg.norm(b - r, axis=-1).mean(0)
            per_link = e[None] if per_link is None else np.vstack([per_link, e[None]])
    if per_link is None:
        print(f"{tag} {key}: sem quadros", flush=True)
        continue
    e = per_link.mean(0)
    group = lambda f: float(np.mean([e[k] for k, n in enumerate(names) if f(n)])) if names else float("nan")
    legs = group(lambda n: any(s in n for s in ("Thigh", "Shin", "Foot", "Toe")))
    arms = group(lambda n: any(s in n for s in ("Arm", "Hand", "Shoulder", "Clavicle")))
    trunk = group(lambda n: any(s in n for s in ("Spine", "Chest", "Neck", "Head")))
    all_means.append(float(e.mean()))
    print(f"{tag} {key}: distancia ao clipe {e.mean():.1f} cm (pernas {legs:.1f}, bracos {arms:.1f}, tronco {trunk:.1f})", flush=True)
print(f"DISTLOCO media {np.mean(all_means):.1f}", flush=True)
