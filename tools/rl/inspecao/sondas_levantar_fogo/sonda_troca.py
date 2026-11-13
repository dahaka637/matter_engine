"""(12/11 15h10) A troca calma (n68) contra a por tempo: lote em modo de treino, o fogo e as adversidades do treino de agora, so deitados,
sem ruido. Por exercicio: sob fogo levantou, firme e firme dado que levantou; e o limpo."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L; S = amb.S
calm = float(sys.argv[1]); ex = int(sys.argv[2]); W = 64
b = L.get_up_batch(S.ASSETS, environments=W, threads=4, seed=23, episode_seconds=12.0, **S.TRACK)
amb.setup(b, 0.0)
b.add_style_clips(L.LEVANTAR_UNREAL)
b.set_clip_get_up(L.LEVANTAR_UNREAL, 1.0, 0.0, float(amb.v2.get("levantar_unreal_depois", 1.5)), 0.2,
                  float(amb.v2.get("levantar_unreal_largura", 2.0)), float(amb.v2.get("levantar_unreal_vel", 1.5)))
b.set_clip_get_up_variants(list(L.LEVANTAR_UNREAL_VARIANTS)); b.set_clip_get_up_deliver(list(L.LEVANTAR_UNREAL_DELIVER))
b.set_clip_get_up_adaptive(True); b.set_adaptive_settle(0.6, 0.4); b.set_adaptive_retry(True); b.set_run_post_rise(0.5, 1.0)
b.set_fire_run_headwind_cap(0.25); b.set_get_up_arena(10.0, 10.0)
b.set_get_up_fire_extras(6, 0.3, 0.25, 0.5, 3.0, 0.3, 1.5, 0.3, 0.25, 0.25, 0.3, 0.3, 0.8, 0.25, 0.4, 350.0, 800.0)
if calm > 0: b.set_calm_handoff(calm, 1.5, 1.8)
b.set_clip_get_up_start_bias(list(L.LEVANTAR_UNREAL_START_BIAS))
w = [0.0] * 4; w[ex] = 1.0; b.set_clip_get_up_weights(w)
lv = json.load(open(sys.argv[3]))
b.set_get_up_fire([0.25, 0.0, 0.0, 0.75, 0.15], [float(x) for k in L.LEVANTAR_UNREAL_KEYS for x in lv[k]], False)
b.reset(23); model = amb.model(b); rows = []
for s in range(int(round(40.0 / float(b.policy_dt)))):
    obs = torch.as_tensor(b.observations())
    with torch.no_grad():
        a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
    b.step(a)
    rows += b.finished_episodes().tolist()
x = np.array(rows, np.float32); x = x[(x[:, 1] == 9) & (x[:, 56] <= 0.05)]
up = x[:, 18] >= 0; firm = up & ~np.isin(x[:, 32], (1, 2, 3, 4, 8)); fire = x[:, 49] > 0; clean = x[:, 49] == 0
print(f"{L.LEVANTAR_UNREAL_KEYS[ex]} troca calma {calm}: sob fogo levantou {up[fire].mean():.2f} firme {firm[fire].mean():.2f} firme|lev {firm[fire & up].mean():.2f} (n {fire.sum()}) | "
      f"limpo levantou {up[clean].mean():.2f} firme {firm[clean].mean():.2f} (n {clean.sum()})", flush=True)
