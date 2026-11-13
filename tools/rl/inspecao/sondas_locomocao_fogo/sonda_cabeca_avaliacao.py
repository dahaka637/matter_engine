"""(12/11 19h35, ele: no laboratorio "nao troca para o da locomocao, parece estar tentando andar pelo brain do levantar") O lote em modo de
avaliacao, como o do laboratorio (a avaliacao 11+exercicio; TIPO=ir: o levantar e ir), a acao media: por mundo, a linha do tempo da
cabeca (L levantar, M locomocao, a cada 0,1 s) ate o fim do primeiro episodio. Uso (de tools/rl):
python inspecao/sondas_locomocao_fogo/sonda_cabeca_avaliacao.py CHECKPOINT RECEITA EXERCICIO"""
import json, os, sys
import numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita, ex = sys.argv[1], sys.argv[2], int(sys.argv[3])
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
parser = L.build_parser()
args = parser.parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
L.aplicar_receita(parser, args)
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
b = L.get_up_batch(str(L.REPO / "assets"), environments=6, threads=2, seed=5, evaluation=True, evaluation_kind=11 + ex, **L.montar_v2(args))
L.configurar(b, args, counters, False)
L.configurar_levantar(b, args, 0.0)
if os.environ.get("TIPO") == "ir":
    b.set_get_up_go(1.0, 5.0, 10.0)
b.reset(5)
model = amb.model(b)
lines = [""] * 6; done = set(); dt = float(b.policy_dt); every = max(1, int(round(0.1 / dt)))
for step in range(int(round(12.0 / dt))):
    obs = torch.as_tensor(b.observations())
    if step % every == 0:
        for i in range(6):
            if i not in done: lines[i] += "M" if obs[i, -1] > .5 else "L"
    with torch.no_grad():
        a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
    b.step(a)
    for r in b.finished_episodes().tolist():
        if int(r[0]) not in done:
            done.add(int(r[0])); lines[int(r[0])] += f" | fim {r[4]:.1f}s comeco {int(r[1])} causa {int(r[32])} troca {r[18]:.2f}s"
    if len(done) == 6: break
print("obs", b.observations().shape)
for l in lines: print(l)
