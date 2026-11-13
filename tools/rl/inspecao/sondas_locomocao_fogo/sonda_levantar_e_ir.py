"""(12/11 19h15, ele: o "levantar e ir" - deitado de costas ou de brucos, levanta, a troca para a locomocao e vai ate um ponto a 5-10 m;
normal ou correndo) Todos os levantares pelo clipe como levantar e ir, a acao media. Por tipo: levantou, chegou (o sucesso), caiu
depois da troca, os tempos e as causas do fim dos que nao chegaram. Uso (de tools/rl):
python inspecao/sondas_locomocao_fogo/sonda_levantar_e_ir.py CHECKPOINT RECEITA"""
import collections, json, sys
import numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
parser = L.build_parser()
args = parser.parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
L.aplicar_receita(parser, args)  # (os 12 s do episodio do --parado-segundos, como no treino)
args.locomocao, args.giro, args.comeco_de_pe, args.locomocao_fogo, args.comando = 0.0, 0.0, 0.0, 0.0, 0.0
args.levantar_e_ir = (1.0, 5.0, 10.0)
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
args.semente = 61
b = L.get_up_batch(str(L.REPO / "assets"), environments=48, threads=4, **L.lote_do_treino(args), **L.montar_v2(args))
L.configurar(b, args, counters, True)
L.configurar_levantar(b, args, 1.0)
b.reset(61)
model = amb.model(b)
rows = []
for _ in range(int(round(40.0 / float(b.policy_dt)))):
    obs = torch.as_tensor(b.observations())
    with torch.no_grad():
        a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
    b.step(a)
    rows += b.finished_episodes().tolist()
x = np.array(rows, np.float32)
print(f"comecos: {collections.Counter(x[:, 1].astype(int).tolist())}")
x = x[x[:, 1] == 11]
for name, sel in (("todos", np.ones(len(x), bool)), ("de pe", x[:, 72] < 1.5), ("correndo", x[:, 72] > 1.5)):
    out = L.get_up_go_summary(x[sel])
    miss = x[sel & (x[:, 73] < 0)]
    print(f"{name}: " + " ".join(f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}" for k, v in out.items())
          + f" | fins sem chegar: {dict(collections.Counter(miss[:, 32].astype(int).tolist()))}, faltou p25/50/75 "
          f"{np.percentile(miss[:, 75], [25, 50, 75]).round(1).tolist() if len(miss) else '-'} m, segundos med "
          f"{np.median(miss[:, 4]) if len(miss) else -1:.1f}", flush=True)
