"""(12/11 17h40, ele: "porque apos frear, ele fica com a coluna torta, ao inves de ficar na pose IDLE?") Das capturas da suite (paradas
longas, ordem aos 4 s; e o parado): por segundo depois da ordem, quem dirige (locomocao), a coluna de lado e para a frente do corpo
(graus; a de lado contra o idle ou o espelho, o mais perto) e a do idle, e a velocidade. Uso (de tools/rl): python
inspecao/sondas_locomocao_fogo/coluna_freio.py PASTA_DAS_CAPTURAS"""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, '.')
from suite.medidas import base
caps = base.capturas(Path(sys.argv[1]))
for tag in sorted(caps):
    cap = caps[tag]
    body, clip = base.postura(cap.corpo), base.postura(cap.ref)
    dt = float(cap.meta["dt"])
    T = cap.corpo.shape[1]
    alive = cap.z["episodio"] == cap.z["episodio"][:, :1]
    loco = cap.z["cabeca_loco"]
    lado = body["coluna_lado"] - clip["coluna_lado"]
    lado = np.where(np.abs(body["coluna_lado"] + clip["coluna_lado"]) < np.abs(lado), body["coluna_lado"] + clip["coluna_lado"], lado)
    frente = body["coluna_frente"] - clip["coluna_frente"]
    p = cap.corpo[:, :, 0, :2].astype(float)
    v = np.linalg.norm(np.diff(p, axis=1), axis=2) / dt
    v = np.concatenate([v[:, :1], v], 1)
    out = []
    for s0 in range(3, int(T * dt)):
        m = alive[:, int(s0 / dt):int((s0 + 1) / dt)]
        if not m.any():
            continue
        sl = slice(int(s0 / dt), int((s0 + 1) / dt))
        out.append(f"{s0}s loco {loco[:, sl][m].mean():.2f} lado {np.median(np.abs(lado[:, sl][m])):4.1f} frente {np.median(frente[:, sl][m]):+5.1f} "
                   f"v {np.median(v[:, sl][m]):.2f} vivos {m.any(1).mean():.2f}")
    print(tag, "|", " ; ".join(out))
