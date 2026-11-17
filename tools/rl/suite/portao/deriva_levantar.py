"""(15/11 12h05, ele: o levantar de costas 'da uns passos em rumo a uma certa direcao... deveria permanecer no mesmo lugar 100% das
vezes') Quanto a pelve anda no mundo (xy) do comeco deitado ate de pe e ate SEGUNDOS, e quantos passos (pe sobe > 4 cm) depois do primeiro
apoio dos dois pes, nos mundos limpos (0-3) x sementes, com a acao media.
Uso: deriva_levantar.py TREINO PONTO [segundos=6] [sementes=61,62,63]"""
import sys, os
from pathlib import Path
import numpy as np, torch
W = Path("/home/dahaka/Projetos/matter_engine_wip")
sys.path.insert(0, str(W / "tools/rl"))
from inspecao import comum  # noqa: E402
treino, ck = sys.argv[1], sys.argv[2]
SECONDS = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0
SEEDS = [int(s) for s in (sys.argv[4] if len(sys.argv) > 4 else "61,62,63").split(",")]
amb = comum.Ambiente(treino, ck)
KEYS = os.environ.get("CHAVES", "lev_levantar_costas,lev_levantar_brucos").split(",")
LINKS = ("Head", "Pelvis", "LeftFoot", "RightFoot")
for key in KEYS:
    rows = []
    for seed in SEEDS:
        b = amb.batch(key, 16, seed); amb.setup(b, amb.level(key)); b.reset(seed)
        m = amb.model(b); dt = float(b.policy_dt); steps = int(round(SECONDS / dt))
        pos = np.full((16, steps, 4, 3), np.nan); alive = np.ones(16, bool)
        for i in range(steps):
            obs = torch.as_tensor(np.asarray(b.observations()))
            with torch.no_grad():
                a = m.env_action(m.distribution(m.norm(obs)).mean, obs).numpy().astype(np.float32)
            b.step(a)
            for w in range(4):
                if not alive[w]: continue
                L = comum.parse_lines(b.frame_lines(w))[0]
                for k, nm in enumerate(LINKS): pos[w, i, k] = L[nm][:3]
            for r in b.finished_episodes().tolist():
                alive[int(r[0])] = False
        for w in range(4):
            head = pos[w, :, 0, 2]; pel = pos[w, :, 1, :2]
            ok = ~np.isnan(head); n = int(ok.sum())
            if n < 10: continue
            up = next((j for j in range(n) if head[j] > 1.45), None)
            p0 = pel[0]
            d_up = np.linalg.norm(pel[up] - p0) if up is not None else np.nan
            d_end = np.linalg.norm(pel[n - 1] - p0)
            d_after = np.linalg.norm(pel[n - 1] - pel[up]) if up is not None else np.nan
            lifts = 0
            if up is not None:
                for f in (2, 3):
                    z = pos[w, up:n, f, 2]; base = np.percentile(z, 10); hi = z > base + 0.04
                    lifts += int(np.sum(hi[1:] & ~hi[:-1]))
            rows.append((up * dt if up is not None else np.nan, d_up, d_after, d_end, lifts))
    r = np.array(rows, float)
    print(f"{key}: n={len(r)} de pe em {np.nanmean(r[:,0]):.2f} s | pelve: ate de pe {100*np.nanmean(r[:,1]):.0f} cm, depois de de pe {100*np.nanmean(r[:,2]):.0f} cm (max {100*np.nanmax(r[:,2]):.0f}), total {100*np.nanmean(r[:,3]):.0f} cm (max {100*np.nanmax(r[:,3]):.0f}) | passos depois de de pe {np.nanmean(r[:,4]):.1f}", flush=True)
