"""(12/11 08h20) n54: a nova tentativa depois do fracasso (settings.adaptiveRetry). argv: niveis.json tentar(0/1) exercicios(0,1,2,3).
A rede de agora, sem ruido (como no jogo), a janela de firmeza do treino; por cenario: levantou, firme, tentativas > 1 e levantou depois delas."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente("idle_alerta"); L = amb.L
_gub = L.get_up_batch
L.get_up_batch = lambda *a, **k: _gub(*a, **{**k, "episode_seconds": 12.0})
levels = json.loads(open(sys.argv[1]).read()); retry = bool(int(sys.argv[2])); keys = [L.LEVANTAR_UNREAL_KEYS[int(k)] for k in sys.argv[3].split(',')]
W = 64
for key in keys:
    b = amb.batch(key, W, 31); amb.setup(b, 0.0)
    b.set_clip_get_up_adaptive(True)
    b.set_adaptive_settle(0.6, 0.4)
    b.set_adaptive_retry(retry)
    b.set_get_up_fire([0.2, 0.0, 0.4, 0.4, 0.15], [float(x) for k in L.LEVANTAR_UNREAL_KEYS for x in levels[k]], False)
    b.reset(31); model = amb.model(b); rows = []
    for s in range(int(round(30.0 / float(b.policy_dt)))):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        rows += b.finished_episodes().tolist()
    x = np.array(rows, np.float32); up = x[:, 18] >= 0; fell = np.isin(x[:, 32], (1, 2, 3, 4, 8)); att = x[:, 57]
    out = []
    for k, name in ((0, 'limpo'), (2, 'continuo'), (3, 'combinado')):
        m = x[:, 49] == k; r = m & (att > 1)
        out.append(f"{name} n {m.sum()} levantou {up[m].mean():.2f} firme {(up & ~fell)[m].mean():.2f} tentou de novo {r.mean() / max(m.mean(), 1e-9):.2f} "
                   f"levantou depois {up[r].mean() if r.any() else -1:.2f}")
    print(f"{key} tentar {int(retry)}: {' | '.join(out)} | seg {np.median(x[:, 4]):.1f}", flush=True)
