"""(12/11 09h00) n56: o teto do continuo e a nova tentativa pelo ponto de entrega. Sem ruido, episodio de 12 s, a firmeza e a nova
tentativa do treino, o limite contra a corrida; por exercicio: levantou sob fogo, tentativas, levantou depois, fora do mundo."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L
_gub = L.get_up_batch
L.get_up_batch = lambda *a, **k: _gub(*a, **{**k, "episode_seconds": 12.0})
levels = json.loads(open(sys.argv[1]).read()); keys = [L.LEVANTAR_UNREAL_KEYS[int(k)] for k in sys.argv[2].split(",")]; arena = (float(sys.argv[3]), float(sys.argv[4]))
W = 64
for key in keys:
    b = amb.batch(key, W, 51); amb.setup(b, 0.0)
    b.set_clip_get_up_adaptive(True); b.set_adaptive_settle(0.6, 0.4); b.set_adaptive_retry(True); b.set_fire_run_headwind_cap(0.25); b.set_get_up_arena(*arena)
    b.set_get_up_fire([0.0, 0.0, 0.5, 0.5, 0.15], [float(x) for k in L.LEVANTAR_UNREAL_KEYS for x in levels[k]], False)
    b.reset(51); model = amb.model(b); rows = []
    for s in range(int(round(30.0 / float(b.policy_dt)))):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        rows += b.finished_episodes().tolist()
    x = np.array(rows, np.float32); up = x[:, 18] >= 0; fire = x[:, 49] > 0; att = x[:, 57]; r = fire & (att > 1)
    u, n = np.unique(att[fire].astype(int), return_counts=True)
    up_all = x[:, 18] >= 0; fell = np.isin(x[:, 32], (1, 2, 3, 4, 8)); print(f"firme {(up_all & ~fell)[fire].mean():.2f}", end=" ")
    print(f"{key} paredes {arena} continuo {levels[key][0]}: levantou sob fogo {up[fire].mean():.2f} (n {fire.sum()}) continuo {up[x[:,49]==2].mean():.2f} combinado {up[x[:,49]==3].mean():.2f} | "
          f"tentou de novo {r.sum() / max(1, fire.sum()):.2f} levantou depois {up[r].mean() if r.any() else -1:.2f} | tentativas {dict(zip(u.tolist(), n.tolist()))} | "
          f"fora do mundo {(x[:, 7] < -0.5).sum()} cabeca max {x[:, 7].max():.2f} nan {int(np.isnan(x).any())}", flush=True)
