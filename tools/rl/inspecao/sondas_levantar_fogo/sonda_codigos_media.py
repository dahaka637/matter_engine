"""(12/11 07h45) n52: os codigos do foi ao chao (0 nao, 1 caiu e nao levantou, 2 recuperou, 3 levantou e caiu) e a firmeza (argv[2] s)."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L
cols = list(comum.EPISODE_COLUMNS); c = {n: cols.index(n) for n in cols}
levels = json.loads(open(sys.argv[1]).read()); settle = float(sys.argv[2]); release = float(sys.argv[4]) if len(sys.argv) > 4 else 0.15; keys = [L.LEVANTAR_UNREAL_KEYS[int(k)] for k in sys.argv[3].split(',')]
W = 64
for key in keys:
    b = amb.batch(key, W, 21); amb.setup(b, 0.0)
    b.set_clip_get_up_adaptive(True)
    b.set_adaptive_settle(settle, release)
    b.set_get_up_fire([0.0, 0.0, 0.5, 0.5, 0.15], [float(x) for k in L.LEVANTAR_UNREAL_KEYS for x in levels[k]], False)
    b.reset(21); model = amb.model(b); rows = []
    for s in range(int(round(22.0 / float(b.policy_dt)))):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        rows += b.finished_episodes().tolist()
    x = np.array(rows, np.float32); up = x[:, 18] >= 0; fell = np.isin(x[:, 32], (1, 2, 3, 4, 8))
    out = []
    for k, name in ((2, 'continuo'), (3, 'combinado')):
        m = x[:, 49] == k
        out.append(f"{name} n {m.sum()} levantou {up[m].mean():.2f} firme {(up & ~fell)[m].mean():.2f}")
    codes = dict(zip(*[v.tolist() for v in np.unique(x[:, 54], return_counts=True)]))
    print(f"{key} firmeza {settle} soltar {release}: {' | '.join(out)} | codigos {codes}", flush=True)
