"""(12/11 08h45) n55: os correndo por rumo da forca com o limite contra a corrida (argv[2], fracao do peso; 0 sem). A rede de agora, sem
ruido, a nova tentativa e a firmeza do treino, episodio de 12 s; so o continuo e o combinado."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L
_gub = L.get_up_batch
L.get_up_batch = lambda *a, **k: _gub(*a, **{**k, "episode_seconds": 12.0})
levels = json.loads(open(sys.argv[1]).read()); cap = float(sys.argv[2]); keys = [L.LEVANTAR_UNREAL_KEYS[int(k)] for k in sys.argv[3].split(',')]
W = 64
for key in keys:
    b = amb.batch(key, W, 41); amb.setup(b, 0.0)
    b.set_clip_get_up_adaptive(True); b.set_adaptive_settle(0.6, 0.4); b.set_adaptive_retry(True)
    b.set_fire_run_headwind_cap(cap)
    b.set_get_up_fire([0.0, 0.0, 0.5, 0.5, 0.15], [float(x) for k in L.LEVANTAR_UNREAL_KEYS for x in levels[k]], False)
    b.reset(41); model = amb.model(b); rows = []
    for s in range(int(round(30.0 / float(b.policy_dt)))):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        rows += b.finished_episodes().tolist()
    x = np.array(rows, np.float32); up = x[:, 18] >= 0; fire = x[:, 49] > 0
    per = ' '.join(f"{d}:{up[fire & (x[:, 51] % 8 == d)].mean():.2f}" for d in range(8) if (fire & (x[:, 51] % 8 == d)).any())
    print(f"{key} limite {cap}: levantou sob fogo {up[fire].mean():.2f} (n {fire.sum()}) | continuo {up[x[:, 49] == 2].mean():.2f} combinado {up[x[:, 49] == 3].mean():.2f} | por rumo {per}", flush=True)
