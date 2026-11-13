"""(12/11 21h25) A assimetria do trote de lado: na capacidade a semelhanca do rumo 6 (direita) fica em 0,01-0,03 e a do 2 (esquerda) em
0,06-0,25. Os rumos do trote (a avaliacao 4, faixa BANDA), a rede media, sem fogo: por rumo (mundo % 8), a media dos termos do rastreio
(locomotion_feedback) depois de 1,5 s. Uso (de tools/rl): python inspecao/sondas_locomocao_fogo/sonda_lado_direito.py CHECKPOINT RECEITA [BANDA]"""
import json, sys
import numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]
band = int(sys.argv[3]) if len(sys.argv) > 3 else 1
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
parser = L.build_parser(); args = parser.parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda']); L.aplicar_receita(parser, args)
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
v2 = dict(L.montar_v2(args), moving_push_newtons=(0.0, 0.0), moving_push_hold_newtons=(0.0, 0.0))
b = L.get_up_batch(str(L.REPO / "assets"), environments=48, threads=4, seed=700 + band, evaluation=True, evaluation_kind=4, evaluation_band=band, **v2)
L.configurar(b, args, counters, False)
model = amb.model(b); b.reset(12345)
dt = float(b.policy_dt); acc = {d: [] for d in range(8)}; done = set()
for step in range(int(b.episode_seconds / dt)):
    obs = torch.as_tensor(b.observations())
    with torch.no_grad():
        a = model.env_action(model.actor(model.norm(obs)), obs).numpy().astype(np.float32)
    b.step(a)
    fb = np.asarray(b.locomotion_feedback())
    for r in b.finished_episodes().tolist(): done.add(int(r[0]))
    if step * dt < 1.5: continue
    for i in range(48):
        if i not in done and fb[i, 0] > 0: acc[i % 8].append(fb[i])
T = L.TRACKING_TERMS
cols = ["nota", "pernas", "tronco", "bracos", "pes", "r_semelhanca", "r_vel", "coluna_graus", "cabeca_graus", "bracos_graus", "inclinacao_graus"]
print("rumo " + " ".join(f"{c[:10]:>10s}" for c in cols))
for d in range(8):
    if acc[d]:
        m = np.mean(acc[d], 0)
        print(f"{d:4d} " + " ".join(f"{m[T.index(c)]:10.3f}" for c in cols))
