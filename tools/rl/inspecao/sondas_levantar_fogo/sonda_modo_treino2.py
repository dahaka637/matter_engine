"""(12/11 07h40) O correndo de costas sob fogo num lote em modo de treino (evaluation False) contra o de avaliacao: o recuperou."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L; S = amb.S
cols = list(comum.EPISODE_COLUMNS); c = {n: cols.index(n) for n in cols}
levels = json.loads(open(sys.argv[1]).read()); noise = float(sys.argv[2]); ex = int(sys.argv[3]); conf = int(sys.argv[4]); colored = float(sys.argv[5])
W = 64
b = L.get_up_batch(S.ASSETS, environments=W, threads=4, seed=13, **S.TRACK)
amb.setup(b, 0.0)
if conf:
    import math
    rec = json.load(open('receitas/levantar_unreal.json'))['argumentos']
    args = L.build_parser().parse_args(rec + ['--treino', 'sonda'])
    counters = amb.state.get('counters', {})
    for k, v in (("comando_fase", int(args.comando_fase)), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", max(0.0, args.fogo)), ("lento", [args.lento] * 3)):
        counters.setdefault(k, v)
    counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
    L.configurar(b, args, counters, True)
    print("configurar(treino) aplicado", flush=True)
b.add_style_clips(L.LEVANTAR_UNREAL)
b.set_clip_get_up(L.LEVANTAR_UNREAL, 1.0, 0.0, float(amb.v2.get("levantar_unreal_depois", 1.5)), noise,
                  float(amb.v2.get("levantar_unreal_largura", 2.0)), float(amb.v2.get("levantar_unreal_vel", 1.5)))
b.set_clip_get_up_variants(list(L.LEVANTAR_UNREAL_VARIANTS))
b.set_clip_get_up_deliver(list(L.LEVANTAR_UNREAL_DELIVER))
b.set_clip_get_up_adaptive(True)
b.set_clip_get_up_start_bias(list(L.LEVANTAR_UNREAL_START_BIAS))
w = [0.0] * 4; w[ex] = 1.0; b.set_clip_get_up_weights(w)
b.set_get_up_fire([0.0, 0.0, 0.5, 0.5, 0.15], [float(x) for k in L.LEVANTAR_UNREAL_KEYS for x in levels[k]], False)
b.reset(13); model = amb.model(b); rows = []
import math
keep = math.exp(-float(b.policy_dt) / 0.15); state = None
for s in range(int(round(26.0 / float(b.policy_dt)))):
    obs = torch.as_tensor(b.observations())
    with torch.no_grad():
        dist = model.distribution(model.norm(obs))
        if colored > 0:
            state = torch.randn_like(dist.mean) if state is None else keep * state + math.sqrt(1 - keep ** 2) * torch.randn_like(dist.mean)
            act = dist.mean + dist.stddev * (math.sqrt(1 - colored) * torch.randn_like(dist.mean) + math.sqrt(colored) * state)
        else:
            act = dist.sample()
        a = model.env_action(act, obs).numpy().astype(np.float32)
    b.step(a)
    rows += b.finished_episodes().tolist()
r = np.array(rows, np.float32)
gu = r[:, 1] == 9; print("comecos:", dict(zip(*[x.tolist() for x in np.unique(r[:, 1], return_counts=True)])), "exercicios:", np.unique(r[gu, 26], return_counts=True))
x = r[gu & (r[:, 56] <= 0.05) & (r[:, 26] == ex)]
down = x[:, c["fogo_foi_ao_chao"]] > 0.5; up = x[:, c["entrega_em"]] >= 0; fire = x[:, c["fogo_cenario"]] > 0
print(f"{L.LEVANTAR_UNREAL_KEYS[ex]} modo treino, ruido {noise} conf {conf} colorido {colored}: {len(x)} ep, levantou sob fogo {up[fire].mean():.2f}, foi ao chao {down.mean():.2f} "
      f"({down.sum()}), recuperou {up[down].mean() if down.any() else -1:.2f}; perdidos cabeca {np.median(x[down & ~up, c['cabeca']]) if (down & ~up).any() else -1:.2f}", flush=True)
