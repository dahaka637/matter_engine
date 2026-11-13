"""(12/11 07h35) O primeiro episodio de cada mundo contra os seguintes (o treino roda em sequencia): o recuperou depois do chao nos correndo."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L
cols = list(comum.EPISODE_COLUMNS); c = {n: cols.index(n) for n in cols}
levels = json.loads(open(sys.argv[1]).read())
W = 64
for key in (L.LEVANTAR_UNREAL_KEYS[2], L.LEVANTAR_UNREAL_KEYS[0]):
    b = amb.batch(key, W, 11); amb.setup(b, 0.0)
    b.set_clip_get_up_adaptive(True)
    b.set_get_up_fire([0.0, 0.0, 0.5, 0.5, 0.15], [float(x) for k in L.LEVANTAR_UNREAL_KEYS for x in levels[k]], False)
    b.reset(11); model = amb.model(b); seen = {}; rows = []
    for s in range(int(round(26.0 / float(b.policy_dt)))):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).sample(), obs).numpy().astype(np.float32)
        b.step(a)
        for r in b.finished_episodes().tolist():
            k = int(r[0]); seen[k] = seen.get(k, 0) + 1; rows.append([seen[k]] + r)
    r = np.array(rows, np.float32); order, r = r[:, 0], r[:, 1:]
    for name, sel in (("primeiro", order == 1), ("seguintes", order > 1)):
        x = r[sel]; down = x[:, c["fogo_foi_ao_chao"]] > 0.5; up = x[:, c["entrega_em"]] >= 0; fire = x[:, c["fogo_cenario"]] > 0
        print(f"{key} {name}: {len(x)} ep, levantou sob fogo {up[fire].mean():.2f}, foi ao chao {down.mean():.2f} ({down.sum()}), "
              f"recuperou {up[down].mean() if down.any() else -1:.2f}, comeco_clipe max {x[:, c['comeco_clipe']].max():.2f}", flush=True)
