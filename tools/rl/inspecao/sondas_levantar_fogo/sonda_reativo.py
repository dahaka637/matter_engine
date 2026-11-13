"""(12/11 09h45) A nova tentativa reativa (n58) contra a de antes (n57): por exercicio, sem ruido, 12 s, o treino de agora (firmeza,
nova tentativa, vento contra 0,25, paredes 10 m), 20% limpos."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L
_gub = L.get_up_batch
L.get_up_batch = lambda *a, **k: _gub(*a, **{**k, "episode_seconds": 12.0})
levels = json.loads(open(sys.argv[1]).read()); keys = [L.LEVANTAR_UNREAL_KEYS[int(k)] for k in sys.argv[2].split(',')]; tag = sys.argv[3]
W = 64
for key in keys:
    b = amb.batch(key, W, 61); amb.setup(b, 0.0)
    b.set_clip_get_up_adaptive(True); b.set_adaptive_settle(0.6, 0.4); b.set_adaptive_retry(True); b.set_fire_run_headwind_cap(0.25)
    b.set_get_up_arena(10.0, 10.0)
    b.set_get_up_fire([0.2, 0.0, 0.4, 0.4, 0.15], [float(x) for k in L.LEVANTAR_UNREAL_KEYS for x in levels[k]], False)
    b.reset(61); model = amb.model(b); rows = []
    for s in range(int(round(30.0 / float(b.policy_dt)))):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        rows += b.finished_episodes().tolist()
    x = np.array(rows, np.float32); up = x[:, 18] >= 0; fire = x[:, 49] > 0; clean = x[:, 49] == 0; att = x[:, 57]; r = fire & (att > 1)
    t = x[r & up, 58] if x.shape[1] > 58 else np.array([])
    print(f"{tag} {key}: sob fogo {up[fire].mean():.2f} | limpo {up[clean].mean():.2f} tentou-limpo {(att[clean] > 1).mean():.2f} | "
          f"tentou {r.sum() / max(1, fire.sum()):.2f} levantou depois {up[r].mean() if r.any() else -1:.2f} "
          f"tempo da tentativa {np.median(t) if len(t) else -1:.2f} s | tentativas med {np.median(att[r]) if r.any() else -1:.0f}", flush=True)
