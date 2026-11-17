"""(14/11 19h55, ele: 'atacar a estabilidade do pos levantar dos nossos levantares, sinto que estao bem fracos') O que acontece DEPOIS de
ficar de pe nos quatro levantares, com a acao media (modo do jogo): 16 mundos (0-3 limpos, 4-9 empurroes, 10-15 pancadas, como a prova de
fogo) x sementes, SEGUNDOS s. Por levantar e grupo: de pe firme (cabeca > 1,45 m por 0,5 s), quando o episodio acaba depois disso, recaida
(cabeca < 1,0 m depois de de pe), passos (pe sobe > 4 cm), quanto os pes andam, a deriva e o balanco da pelve, o tempo ate assentar e a
inclinacao do tronco.

Uso: pos_levantar.py TREINO PONTO [segundos=9] [sementes=61,62]
"""
import sys
from pathlib import Path

import numpy as np
import torch

W = Path("/home/dahaka/Projetos/matter_engine_wip")
sys.path.insert(0, str(W / "tools/rl"))
from inspecao import comum  # noqa: E402

treino, ck = sys.argv[1], sys.argv[2]
SECONDS = float(sys.argv[3]) if len(sys.argv) > 3 else 9.0
SEEDS = [int(s) for s in (sys.argv[4] if len(sys.argv) > 4 else "61,62").split(",")]
amb = comum.Ambiente(treino, ck)
import os
# (DEPOIS: o episodio segue isto (s) depois da troca, como no laboratorio; MARCA: a marca do de costas, 0 = a do laboratorio.)
if os.environ.get("DEPOIS"):
    amb.v2["levantar_unreal_depois"] = float(os.environ["DEPOIS"])
if os.environ.get("MARCA") is not None:
    amb.v2["levantar_costas_marca"] = float(os.environ["MARCA"])
KEYS = os.environ.get("CHAVES", ",".join(amb.L.LEVANTAR_UNREAL_KEYS)).split(",")
GROUPS = {"limpo": range(0, 4), "empurroes": range(4, 10), "pancadas": range(10, 16)}
LINKS = ("Head", "Pelvis", "LeftFoot", "RightFoot")


def analyse(dt, head, pel, lf, rf, phase, ep_end):
    """Um mundo: as medidas depois do primeiro de pe firme do primeiro episodio (None se nao ficou de pe)."""
    n = len(head) if ep_end is None else ep_end
    up, ts = 0.0, None
    for j in range(n):
        up = up + dt if head[j] > 1.45 else 0.0
        if up >= 0.5:
            ts = j + 1 - int(round(0.5 / dt))
            break
    if ts is None:
        return None
    a = ts + int(round(0.5 / dt))  # o fim da janela firme
    end = min(n, ts + int(round(6.0 / dt)))
    if end - a < int(round(1.5 / dt)):
        return {"curto": True, "fim_depois": (n - ts) * dt}
    refall = bool((head[ts:end] < 1.0).any())
    lifts = 0
    travel = 0.0
    for f in (lf, rf):
        z = f[a:end, 2]
        base = np.percentile(z, 10)
        hi = z > base + 0.04
        lifts += int(np.sum(hi[1:] & ~hi[:-1]))
        travel += float(np.linalg.norm(np.diff(f[a:end, :2], axis=0), axis=-1).sum())
    drift = float(np.linalg.norm(pel[end - 1, :2] - pel[a, :2]))
    v = np.linalg.norm(np.diff(pel[:, :2], axis=0), axis=-1) / dt
    vf = np.maximum(np.linalg.norm(np.diff(lf[:, :2], axis=0), axis=-1), np.linalg.norm(np.diff(rf[:, :2], axis=0), axis=-1)) / dt
    b0, b1 = ts + int(round(1.0 / dt)), min(end - 1, ts + int(round(4.0 / dt)))
    sway = float(np.sqrt(np.mean(v[b0:b1] ** 2))) if b1 > b0 else np.nan
    settle = None
    quiet = 0.0
    for j in range(a, end - 1):
        quiet = quiet + dt if (v[j] < 0.10 and vf[j] < 0.05) else 0.0
        if quiet >= 0.5:
            settle = (j + 1 - ts) * dt - 0.5
            break
    tilt = []
    for j in range(b0, b1):
        d = head[j] - pel[j, 2]
        hor = np.linalg.norm(HEADXY[j] - pel[j, :2])
        tilt.append(np.degrees(np.arctan2(hor, d)))
    hand = next((j for j in range(n) if phase[j] >= 0.5), None)
    return {"curto": False, "de_pe": ts * dt, "fim_depois": None if ep_end is None else (ep_end - ts) * dt, "recaida": refall,
            "passos": lifts, "pes_andam": travel, "deriva": drift, "balanco": sway, "assenta": settle,
            "tronco": float(np.mean(tilt)) if tilt else np.nan, "entrega": None if hand is None else hand * dt}


for key in KEYS:
    res = {g: [] for g in GROUPS}
    for seed in SEEDS:
        b = amb.batch(key, 16, seed)
        amb.setup(b, amb.level(key))
        if os.environ.get("DEPOIS") and "correndo" in key and hasattr(b, "set_run_handoff_at_stand"):
            b.set_run_handoff_at_stand(True, float(os.environ["DEPOIS"]))
        b.reset(seed)
        m = amb.model(b)
        dt = float(b.policy_dt)
        steps = int(round(SECONDS / dt))
        pos = np.full((16, steps, len(LINKS), 3), np.nan)
        phase = np.zeros((16, steps))
        count = np.zeros(16, int)
        ep_end = [None] * 16
        for i in range(steps):
            obs = torch.as_tensor(np.asarray(b.observations()))
            phase[:, i] = obs[:, -1].numpy()
            with torch.no_grad():
                a = m.env_action(m.distribution(m.norm(obs)).mean, obs).numpy().astype(np.float32)
            b.step(a)
            for w in range(16):
                links = comum.parse_lines(b.frame_lines(w))[0]
                for k, nm in enumerate(LINKS):
                    pos[w, i, k] = links[nm][:3]
            for r in b.finished_episodes().tolist():
                w = int(r[0])
                if count[w] == 0:
                    ep_end[w] = i
                count[w] += 1
        for g, ws in GROUPS.items():
            for w in ws:
                HEADXY = pos[w, :, 0, :2]
                res[g].append(analyse(dt, pos[w, :, 0, 2], pos[w, :, 1], pos[w, :, 2], pos[w, :, 3], phase[w], ep_end[w]))
    for g, rs in res.items():
        stood = [r for r in rs if r is not None]
        full = [r for r in stood if not r["curto"]]
        short = [r for r in stood if r["curto"]]
        line = f"{key} {g}: de pe {len(stood)}/{len(rs)}"
        if short:
            line += f" | episodio acaba {np.mean([r['fim_depois'] for r in short]):.1f} s depois de de pe em {len(short)}"
        if full:
            f = lambda k: np.nanmean([np.nan if r[k] is None else r[k] for r in full])
            ends = [r["fim_depois"] for r in full if r["fim_depois"] is not None]
            line += (f" | recaida {sum(r['recaida'] for r in full)}/{len(full)}, passos {f('passos'):.1f}, pes andam {100 * f('pes_andam'):.0f} cm,"
                     f" deriva {100 * f('deriva'):.0f} cm, balanco {100 * f('balanco'):.0f} cm/s, assenta em {f('assenta'):.2f} s"
                     f" ({sum(r['assenta'] is not None for r in full)}/{len(full)}), tronco {f('tronco'):.1f} graus, entrega {f('entrega'):.2f} s")
            if ends:
                line += f", episodio acaba {np.mean(ends):.1f} s depois de de pe"
        print(line, flush=True)
