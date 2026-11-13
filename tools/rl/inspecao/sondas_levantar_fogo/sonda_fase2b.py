"""(12/11 12h35) n59: as adversidades da fase 2 na rede de agora (sem ruido, 12 s, o treino de agora). Por exercicio: sob fogo e por
adversidade (bits da coluna 59), e o firme da pancada de depois de ficar de pe."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L
_gub = L.get_up_batch
L.get_up_batch = lambda *a, **k: _gub(*a, **{**k, "episode_seconds": 12.0})
levels = json.loads(open(sys.argv[1]).read()); keys = [L.LEVANTAR_UNREAL_KEYS[int(k)] for k in sys.argv[2].split(',')]
extras = [float(x) for x in sys.argv[3].split(',')]
W = 64
for key in keys:
    b = amb.batch(key, W, 71); amb.setup(b, 0.0)
    b.set_clip_get_up_adaptive(True); b.set_adaptive_settle(0.6, 0.4); b.set_adaptive_retry(True); b.set_fire_run_headwind_cap(0.25)
    b.set_get_up_arena(10.0, 10.0)
    b.set_get_up_fire_extras(int(extras[0]), *extras[1:])
    b.set_get_up_fire([0.1, 0.0, 0.45, 0.45, 0.15], [float(x) for k in L.LEVANTAR_UNREAL_KEYS for x in levels[k]], False)
    b.reset(71); model = amb.model(b); rows = []
    for s in range(int(round(30.0 / float(b.policy_dt)))):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        rows += b.finished_episodes().tolist()
    x = np.array(rows, np.float32); up = x[:, 18] >= 0; fire = x[:, 49] > 0; fell = np.isin(x[:, 32], (1, 2, 3, 4, 8)); bits = x[:, 59].astype(int)
    per = ' '.join(f"{n}:{up[fire & ((bits & bit) != 0)].mean():.2f}({(fire & ((bits & bit) != 0)).sum()})" for bit, n in ((1, 'serie'), (2, 'rasteira'), (4, 'prensado'), (8, 'vento'), (32, 'escorregadio'), (64, 'agarrao')) if (fire & ((bits & bit) != 0)).any())
    a16 = fire & ((bits & 16) != 0)
    print(f"{key}: sob fogo {up[fire].mean():.2f} (n {fire.sum()}) sem extras {up[fire & (bits == 0)].mean():.2f} | {per} | depois de pe firme "
          f"{(up & ~fell)[a16].mean() if a16.any() else -1:.2f}({a16.sum()}) | nan {int(np.isnan(x).any())} fora {(x[:, 7] < -0.5).sum()}", flush=True)
