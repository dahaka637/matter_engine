"""(12/11 14h35) n67: o comeco em pose variada (girada 60-180 graus) num lote em modo de treino, so limpos, sem os comecos no meio: a rede
de agora (sem ruido) levanta de qualquer posicao? argv: variedade exercicio."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L; S = amb.S
variety = float(sys.argv[1]); ex = int(sys.argv[2]); W = 64
b = L.get_up_batch(S.ASSETS, environments=W, threads=4, seed=17, episode_seconds=12.0, **S.TRACK)
amb.setup(b, 0.0)
b.add_style_clips(L.LEVANTAR_UNREAL)
b.set_clip_get_up(L.LEVANTAR_UNREAL, 1.0, 0.0, float(amb.v2.get("levantar_unreal_depois", 1.5)), 0.2,
                  float(amb.v2.get("levantar_unreal_largura", 2.0)), float(amb.v2.get("levantar_unreal_vel", 1.5)))
b.set_clip_get_up_variants(list(L.LEVANTAR_UNREAL_VARIANTS)); b.set_clip_get_up_deliver(list(L.LEVANTAR_UNREAL_DELIVER))
b.set_clip_get_up_adaptive(True); b.set_adaptive_settle(0.6, 0.4); b.set_adaptive_retry(True); b.set_run_post_rise(0.5, 1.0)
b.set_clip_get_up_start_bias(list(L.LEVANTAR_UNREAL_START_BIAS))
w = [0.0] * 4; w[ex] = 1.0; b.set_clip_get_up_weights(w)
b.set_get_up_pose_variety(variety)
b.set_get_up_fire([1.0, 0.0, 0.0, 0.0, 0.0], [0.0] * 16, False)
b.reset(17); model = amb.model(b); rows = []
for s in range(int(round(30.0 / float(b.policy_dt)))):
    obs = torch.as_tensor(b.observations())
    with torch.no_grad():
        a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
    b.step(a)
    rows += b.finished_episodes().tolist()
x = np.array(rows, np.float32); x = x[(x[:, 1] == 9) & (x[:, 56] <= 0.05)]
up = x[:, 18] >= 0; var = (x[:, 59].astype(int) & 128) != 0; fell = np.isin(x[:, 32], (1, 2, 3, 4, 8))
for name, m in (("normal", ~var), ("pose variada", var)):
    if m.any():
        t = x[m & up, 55]
        print(f"{L.LEVANTAR_UNREAL_KEYS[ex]} variedade {variety}: {name} n {m.sum()} levantou {up[m].mean():.2f} firme {(up & ~fell)[m].mean():.2f} "
              f"tempo {np.median(t[t >= 0]) if (t >= 0).any() else -1:.2f}s tentativas med {np.median(x[m, 57]):.0f}", flush=True)
