"""(15/11 12h50, ele: "nosso parado aprender a nao se deslocar mais, por conta do seu bug anterior, que o fez aprender errado") O parado no
mundo: no exercicio de parado (lev_parado, acao media), de 1 s ao fim (SEGUNDOS), o quanto a pelve se afasta do ponto de 1 s (o maior e o do
fim), quanto os pes andam e quantos passos (pe sobe > 4 cm). Por grupo de mundos (0-3 limpos; o resto como o setup do exercicio).
Uso: deriva_parado.py TREINO PONTO [segundos=8] [sementes=61,62]"""
import sys
from pathlib import Path
import numpy as np, torch
W = Path("/home/dahaka/Projetos/matter_engine_wip")
sys.path.insert(0, str(W / "tools/rl"))
from inspecao import comum  # noqa: E402
treino, ck = sys.argv[1], sys.argv[2]
SECONDS = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0
SEEDS = [int(s) for s in (sys.argv[4] if len(sys.argv) > 4 else "61,62").split(",")]
amb = comum.Ambiente(treino, ck)
key = "lev_parado"
rows = {"limpos 0-3": [], "outros 4-15": []}
for seed in SEEDS:
    b = amb.batch(key, 16, seed); amb.setup(b, amb.level(key)); b.reset(seed)
    m = amb.model(b); dt = float(b.policy_dt); steps = int(round(SECONDS / dt))
    pos = np.full((16, steps, 3, 3), np.nan); alive = np.ones(16, bool)
    for i in range(steps):
        obs = torch.as_tensor(np.asarray(b.observations()))
        with torch.no_grad():
            a = m.env_action(m.distribution(m.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        for w in range(16):
            if not alive[w]: continue
            L = comum.parse_lines(b.frame_lines(w))[0]
            for k, nm in enumerate(("Pelvis", "LeftFoot", "RightFoot")): pos[w, i, k] = L[nm][:3]
        for r in b.finished_episodes().tolist(): alive[int(r[0])] = False
    a0 = int(round(1.0 / dt))
    for w in range(16):
        n = int((~np.isnan(pos[w, :, 0, 0])).sum())
        if n < a0 + int(2.0 / dt): continue
        p = pos[w, a0:n, 0, :2]; d = np.linalg.norm(p - p[0], axis=-1)
        lifts, walk = 0, 0.0
        for f in (1, 2):
            z = pos[w, a0:n, f, 2]; base = np.percentile(z, 10); hi = z > base + 0.04
            lifts += int(np.sum(hi[1:] & ~hi[:-1])); walk += float(np.linalg.norm(np.diff(pos[w, a0:n, f, :2], axis=0), axis=-1).sum())
        rows["limpos 0-3" if w < 4 else "outros 4-15"].append((d.max(), d[-1], walk, lifts, (n - a0) * dt))
for g, r in rows.items():
    if not r: continue
    r = np.array(r)
    print(f"{key} {g}: n={len(r)} | pelve se afasta ate {100*r[:,0].mean():.1f} cm (max {100*r[:,0].max():.1f}), no fim {100*r[:,1].mean():.1f} cm | pes andam {100*r[:,2].mean():.0f} cm | passos {r[:,3].mean():.1f} | janela {r[:,4].mean():.1f} s", flush=True)
