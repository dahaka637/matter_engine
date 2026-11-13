"""F0 v4 (12/11 06h50, ele: "recalibre tudo para que apenas 10-30% das tentativas sob fogo tenham sucesso"): a rede de agora (com as entradas
do fogo), o sorteio de treino (0,8-1), deitado; por exercicio: o continuo sozinho numa grade e o nivel interpolado para ~27% de sucesso; depois
o continuo nesse nivel + a pancada numa grade e, por momento, o nivel interpolado para ~15%. Grava o JSON da versao 4."""
import sys, json, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('idle_alerta'); L = amb.L
cols = list(comum.EPISODE_COLUMNS); T, C0 = cols.index("fogo_momento"), cols.index("entrega_em")
W = 48
GRID_HOLD = {"lev_levantar_costas": (0.45, 0.55, 0.65, 0.75), "lev_levantar_brucos": (0.35, 0.45, 0.55, 0.65),
             "lev_levantar_correndo_costas": (0.55, 0.65, 0.75, 0.90), "lev_levantar_correndo_brucos": (0.65, 0.75, 0.90, 1.05)}
GRID_SHOVE = (1.5, 3.0, 4.5)
HOLD_TARGET, SHOVE_TARGET = 0.27, 0.15
print('checkpoint r', amb.update, flush=True)
def run(key, shares, levels):
    b = amb.batch(key, W, 91); amb.setup(b, 0.0)
    b.set_clip_get_up_adaptive(True)
    b.set_get_up_fire(shares, levels * 4, False)
    b.reset(91); model = amb.model(b); done = {}
    for s in range(int(round(8.0 / float(b.policy_dt)))):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        for r in b.finished_episodes().tolist(): done.setdefault(int(r[0]), r)
        if len(done) >= W: break
    return np.array(list(done.values()), np.float32)
def pick(xs, ys, target):
    """O nivel em que o sucesso (decrescente com a forca) cruza o alvo, interpolado na grade; fora dela, a ponta."""
    for (x0, y0), (x1, y1) in zip(zip(xs, ys), zip(xs[1:], ys[1:])):
        if (y0 - target) * (y1 - target) <= 0 and y0 != y1:
            return round(x0 + (target - y0) * (x1 - x0) / (y1 - y0), 3)
    return xs[-1] if ys[-1] > target else xs[0]
levels, report = {}, {}
for key in L.LEVANTAR_UNREAL_KEYS:
    xs, ys = list(GRID_HOLD[key]), []
    for c in xs:
        r = run(key, [0.0, 0.0, 1.0, 0.0, 0.0], [c, 0.0, 0.0, 0.0])
        ys.append(float((r[:, C0] >= 0).mean()))
        print(key, f"continuo {c:.2f}: levantou {ys[-1]:.2f}", flush=True)
    hold = pick(xs, ys, HOLD_TARGET)
    shove_ys = {1: [], 2: [], 3: []}
    for dv in GRID_SHOVE:
        r = run(key, [0.0, 0.0, 0.0, 1.0, 0.0], [hold, dv, dv, dv])
        ok = r[:, C0] >= 0
        for t in (1, 2, 3):
            m = r[:, T] == t
            shove_ys[t].append(float(ok[m].mean()) if m.any() else float('nan'))
        print(key, f"continuo {hold:.2f} + pancada {dv:.1f}: levantou por momento", {t: round(shove_ys[t][-1], 2) for t in (1, 2, 3)}, flush=True)
    shoves = []
    for t in (1, 2, 3):
        ys_t = shove_ys[t]
        # (Se ja abaixo do alvo na menor pancada, a menor; o continuo ja derruba.)
        shoves.append(pick(list(GRID_SHOVE), ys_t, SHOVE_TARGET) if ys_t[0] > SHOVE_TARGET else GRID_SHOVE[0])
    levels[key] = [hold] + shoves
    report[key] = {"continuo": dict(zip(xs, ys)), "combinado": {str(t): dict(zip(GRID_SHOVE, shove_ys[t])) for t in (1, 2, 3)}}
    print(key, "NIVEIS", levels[key], flush=True)
json.dump({"niveis": levels, "medidas": report}, open(sys.argv[1], 'w'), indent=1)
