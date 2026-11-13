"""(12/11 14h00, ele: "no limpo e interessante que o juiz de estilo recompense adequadamente") O juiz da recuperacao do checkpoint
(state["disc"]) nos levantares da prova (sem ruido, as forcas fixas): por exercicio, a nota media D (~+1 parece clipe, ~-1 nao) e o
premio de estilo do treino (1 - 0,25 (D - 1)^2, >= 0) durante o levantar (a cabeca da recuperacao), nos limpos e nos com fogo."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L
_gub = L.get_up_batch
L.get_up_batch = lambda *a, **k: _gub(*a, **{**k, "episode_seconds": 12.0})
W = 32
for key in L.LEVANTAR_UNREAL_KEYS:
    b = amb.batch(key, W, 12345); amb.setup(b, 0.0)
    disc = L.Discriminator(b.amp_size); disc.load_state_dict(amb.state["disc"]); disc.eval()
    model = amb.model(b); b.reset(12345)
    n = b.num_envs; dsum = np.zeros(n); ssum = np.zeros(n); cnt = np.zeros(n); done = {}
    for _ in range(int(b.episode_seconds / b.policy_dt) + 2):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        with torch.no_grad():
            d = disc(torch.as_tensor(b.amp_pairs())).reshape(-1).numpy()
        rec = obs[:, -1].numpy() < .5
        live = np.array([i not in done for i in range(n)]) & rec
        dsum[live] += d[live]; ssum[live] += np.clip(1 - 0.25 * (d[live] - 1) ** 2, 0, None); cnt[live] += 1
        for r in b.finished_episodes().tolist(): done.setdefault(int(r[0]), r)
        if len(done) >= n: break
    rows = {k: v for k, v in done.items()}
    out = []
    for name, sel in (("limpo", lambda r: r[49] == 0), ("fogo", lambda r: r[49] > 0)):
        ids = [k for k, r in rows.items() if sel(r) and cnt[k] > 0]
        if ids:
            out.append(f"{name}: D {np.mean([dsum[k] / cnt[k] for k in ids]):+.2f} (juiz {50 * (np.mean([dsum[k] / cnt[k] for k in ids]) + 1):.0f}%) estilo {np.mean([ssum[k] / cnt[k] for k in ids]):.2f} n {len(ids)}")
    print(f"{key}: " + " | ".join(out), flush=True)
