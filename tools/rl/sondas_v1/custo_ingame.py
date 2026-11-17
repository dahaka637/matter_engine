"""(13/11 20h, ele: "o RL foi construido unica e exclusivamente para o in-game... 22 bots/players mais o jogador... performance e leveza")
O custo por jogador hoje, medido no ambiente do treino (o teto do runtime in-game, que nao precisa da nota): 23 mundos numa thread, o
comando (o jogador emulado), a acao media; o tempo do passo nativo (fisica 4 x 120 Hz + a logica do ambiente + a observacao) e o da
rede (o ator, lote de 23, uma thread). Uso: custo_ingame.py CHECKPOINT RECEITA"""
import json, sys, time, numpy as np, torch
sys.path.insert(0, '.')
import levantar_amp as L
from inspecao import comum
torch.set_num_threads(1)
ck, receita = sys.argv[1], sys.argv[2]
N, K = 23, 300
amb = comum.Ambiente('cerebro_unico', ck)
args = L.build_parser().parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
counters = amb.state.get('counters', {})
v2 = L.montar_v2(args); plain = dict(v2, moving_push_newtons=(0.0, 0.0), moving_push_hold_newtons=(0.0, 0.0))
for modo, avaliacao in (("avaliacao (sem a nota de treino)", True), ("treino", False)):
    kw = dict(evaluation=True, evaluation_kind=4, evaluation_band=1) if avaliacao else {}
    b = L.get_up_batch(str(L.REPO / "assets"), environments=N, threads=1, seed=761, **kw, **plain)
    L.configurar(b, args, counters, not avaliacao)
    b.set_maneuvers(0.0, 12); b.set_command(0.0, 2, 20.0)
    model = amb.model(b); b.reset(1003)
    t_step = t_obs = t_net = 0.0
    for k in range(K):
        t0 = time.perf_counter(); o = b.observations(); t1 = time.perf_counter()
        obs = torch.as_tensor(o)
        with torch.no_grad():
            act = model.env_action(model.actor(model.norm(obs)), obs).numpy().astype(np.float32)
        t2 = time.perf_counter(); b.step(act); t3 = time.perf_counter()
        b.finished_episodes()
        if k >= 20: t_obs += t1 - t0; t_net += t2 - t1; t_step += t3 - t2
    n = (K - 20)
    por = lambda t: 1000.0 * t / n / N
    print(f"{modo}: por jogador e decisao (30/s): passo nativo {por(t_step):.3f} ms, observacao {por(t_obs):.3f} ms, rede {por(t_net):.3f} ms "
          f"| 23 jogadores x 30 decisoes/s = {(t_step + t_obs + t_net) / n * 30 * 1000:.0f} ms de CPU por segundo (uma thread)", flush=True)
