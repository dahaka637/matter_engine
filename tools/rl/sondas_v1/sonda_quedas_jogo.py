"""(13/11 19h50, a revisao final da v1) As quedas do jogo sem fogo: o comando (o jogador emulado, manobra 12, fase F) e a estrela
(8 fixo, 9 livre, 10 girando, fase 4), na faixa do trote e da corrida, acao media (o jogo). Por queda: o instante, o ultimo evento
do pedido antes (o salto do rumo, a velocidade pedida antes e depois, quanto antes), a velocidade do corpo e o peito contra o rumo.
Uso: sonda_quedas_jogo.py CHECKPOINT RECEITA [N] [SEGUNDOS]"""
import json, os, sys, numpy as np, torch
sys.path.insert(0, '.')
import levantar_amp as L
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]
N = int(sys.argv[3]) if len(sys.argv) > 3 else 48; SEG = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0
amb = comum.Ambiente('cerebro_unico', ck)
args = L.build_parser().parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
counters = amb.state.get('counters', {})
v2 = L.montar_v2(args); plain = dict(v2, moving_push_newtons=(0.0, 0.0), moving_push_hold_newtons=(0.0, 0.0))
def wrap(a): return (a + 180.0) % 360.0 - 180.0
def yaw_of(q):
    x, y, z, w = q[3:7]; fx = 1 - 2 * (y * y + z * z); fy = 2 * (x * y + w * z)
    return np.degrees(np.arctan2(fy, fx))
casos = [("comando_trote_f1", 1, 12, 1), ("comando_corrida_f1", 2, 12, 1), ("comando_trote_f2", 1, 12, 2), ("comando_corrida_f2", 2, 12, 2),
         ("estrela_trote_fixo", 1, 8, 4), ("estrela_corrida_fixo", 2, 8, 4), ("comando_trote_f3", 1, 12, 3), ("comando_corrida_f3", 2, 12, 3), ("comando_trote_f4", 1, 12, 4), ("comando_corrida_f4", 2, 12, 4),
         ("estrela_trote_livre", 1, 9, 4), ("estrela_corrida_livre", 2, 9, 4), ("estrela_trote_girando", 1, 10, 4), ("estrela_corrida_girando", 2, 10, 4)]
so = os.environ.get('SO')
resumo = {}
for nome, band, man, fase in casos:
    if so and not any(x in nome for x in so.split(',')): continue
    b = L.get_up_batch(str(L.REPO / "assets"), environments=N, threads=4, seed=760 + band + int(os.environ.get('SEMENTE', '1003')) - 1003,
                       evaluation=True, evaluation_kind=4,
                       evaluation_band=band, **plain)
    L.configurar(b, args, counters, False)
    if man == 12:
        b.set_maneuvers(0.0, 12); b.set_command(0.0, fase, SEG)
    else:
        b.set_maneuvers(0.0, man); b.set_star(0.0, [1.0] * 6, [fase] * 6, SEG)
    model = amb.model(b); b.reset(int(os.environ.get('SEMENTE', '1003'))); dt = float(b.policy_dt); K = int(SEG / dt)
    nm = list(b.observation_names()); IV, IC, IS = nm.index('comando.velocidade'), nm.index('comando.rumo_cos'), nm.index('comando.rumo_sen')
    IAC, IAS = nm.index('comando.alvo_cos'), nm.index('comando.alvo_sen')
    H = [[] for _ in range(N)]; done = np.zeros(N, bool); fim = {}
    for k in range(K):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            act = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(act)
        for r in b.finished_episodes().tolist():
            w = int(r[0])
            if not done[w]: done[w] = True; fim[w] = (k * dt, int(r[32]), r)
        for w in range(N):
            if done[w]: continue
            Lb, _, q = comum.parse_lines(b.frame_lines(w))
            pv = Lb.get('Pelvis'); ch = Lb.get('UpperChest') or Lb.get('Chest')
            o = obs[w].numpy()
            H[w].append((k * dt, float(o[IV]), float(np.degrees(np.arctan2(o[IS], o[IC]))), pv[2], pv[0], pv[1],
                         yaw_of(ch) if ch else 0.0, float(o[-1]), float(np.degrees(np.arctan2(o[IAS], o[IAC])))))
    quedas = []
    for w in range(N):
        h = np.array(H[w]) if H[w] else np.zeros((0, 9))
        if not len(h): continue
        # a queda: a pelve abaixo de 0,55 m (no comando o levantar continua o episodio) ou o fim por queda (estrela)
        low = np.where(h[:, 3] < 0.55)[0]
        t_fall = h[low[0], 0] if len(low) else (fim[w][0] if w in fim and fim[w][1] != 0 else None)
        causa = fim[w][1] if w in fim else 0
        tipo = 'CAIU' if len(low) else f'fim{causa}'
        if t_fall is None: continue
        i = int(np.searchsorted(h[:, 0], t_fall)) - 1; i = max(1, min(i, len(h) - 1))
        # o ultimo evento do pedido antes: o rumo saltando > 20 graus ou a velocidade pedida mudando > 0,5
        ev = None
        for j in range(i, 0, -1):
            drumo = abs(wrap(h[j, 2] - h[j - 1, 2]))
            if drumo > 20 or abs(h[j, 1] - h[j - 1, 1]) > 0.5:
                ev = (h[i, 0] - h[j, 0], drumo, h[j - 1, 1], h[j, 1]); break
        a = max(0, i - int(0.3 / dt))
        vel = np.hypot(h[i, 4] - h[a, 4], h[i, 5] - h[a, 5]) / max(dt, h[i, 0] - h[a, 0])
        # o alvo do peito (a vista) contra o peito: o maximo no ultimo 1 s, e a pelve no comeco (o comeco deitado?)
        a1 = max(0, i - int(1.0 / dt))
        quedas.append((w, t_fall, ev, vel, float(np.max(np.abs(h[a1:i + 1, 8]))), float(h[0, 3]), float(h[0, 7]), tipo))
    n_q = len(quedas); n_c = sum(q[7] == "CAIU" for q in quedas); resumo[nome] = (N, n_q, n_c)
    print(f"== {nome}: {n_q}/{N} mundos com fim ou queda em {SEG:.0f} s; caiu de verdade (pelve < 0,55 m): {n_c}", flush=True)
    for w, t, ev, vel, alvo, z0, f0, tipo in quedas:
        e = f"evento {ev[0]:.2f}s antes: rumo {ev[1]:5.0f} graus, pedido {ev[2]:.1f} -> {ev[3]:.1f}" if ev else "sem evento antes"
        print(f"   mundo {w:2d} {tipo:5s} {t:5.1f}s | {e} | vel do corpo {vel:.1f} m/s | vista-peito max {alvo:4.0f} graus | pelve inicial {z0:.2f} fase {f0:.0f}")
    np.savez_compressed(f"{os.environ.get('SAIDA', '.')}/{nome}.npz", **{f"w{w}": np.array(H[w]) for w in range(N) if H[w]})
print("RESUMO", json.dumps(resumo))
