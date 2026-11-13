"""(12/11 07h25) Por que os correndo nao recuperam depois de ir ao chao sob fogo (rec 0-1% no treino): a rede de agora, os niveis de agora,
deitado, o sorteio de treino; dos que foram ao chao: o fim, a duracao, a cabeca no fim, quando levantou firme."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L
cols = list(comum.EPISODE_COLUMNS); c = {n: cols.index(n) for n in cols}
levels = json.loads(open(sys.argv[1]).read())
W = 96
for key in L.LEVANTAR_UNREAL_KEYS:
    b = amb.batch(key, W, 7); amb.setup(b, 0.0)
    b.set_clip_get_up_adaptive(True)
    b.set_get_up_fire([0.0, 0.0, 0.5, 0.5, 0.15], [float(x) for k in L.LEVANTAR_UNREAL_KEYS for x in levels[k]], False)
    b.reset(7); model = amb.model(b); done = {}
    for s in range(int(round(9.0 / float(b.policy_dt)))):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        for r in b.finished_episodes().tolist(): done.setdefault(int(r[0]), r)
        if len(done) >= W: break
    r = np.array(list(done.values()), np.float32)
    down = r[:, c["fogo_foi_ao_chao"]] > 0.5; up = r[:, c["entrega_em"]] >= 0
    print(f"{key}: {len(r)} ep, levantou {up.mean():.2f}, foi ao chao {down.mean():.2f}, recuperou {up[down].mean() if down.any() else -1:.2f}")
    for name in ("fim_causa",):
        u, n = np.unique(r[down, c[name]], return_counts=True); print("  chao fim_causa:", dict(zip(u.astype(int).tolist(), n.tolist())))
        u, n = np.unique(r[~down & ~up, c[name]], return_counts=True); print("  falhou sem chao fim_causa:", dict(zip(u.astype(int).tolist(), n.tolist())))
    if down.any():
        print("  chao: segundos", np.round(np.percentile(r[down, c["segundos"]], [10, 50, 90]), 2).tolist(),
              "cabeca fim", np.round(np.percentile(r[down, c["cabeca"]], [10, 50, 90]), 2).tolist(),
              "pelve fim", np.round(np.percentile(r[down, c["pelve"]], [10, 50, 90]), 2).tolist(),
              "momento", np.unique(r[down, c["fogo_momento"]], return_counts=True)[1].tolist(),
              "cenario", np.unique(r[down, c["fogo_cenario"]], return_counts=True)[1].tolist())
    print("  todos: segundos", np.round(np.percentile(r[:, c["segundos"]], [10, 50, 90]), 2).tolist(), flush=True)
