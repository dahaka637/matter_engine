"""(12/11 17h20) A estrela reformada num lote como o do treino: so estrelas (as 6), sem fogo, a acao sorteada. Com e sem o rework
(ESTRELA="peso largura fatia min max"): ate o fim sem cair, pernas, curvas e a distancia media ao caminho, por peito e faixa.
Uso (de tools/rl): python inspecao/sondas_locomocao_fogo/sonda_estrela.py CHECKPOINT RECEITA"""
import json, os, sys
import numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]
rework = [float(x) for x in os.environ.get("ESTRELA", "0 1.2 0 0.15 0.45").split()]
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
args = L.build_parser().parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
args.levantar_unreal, args.locomocao, args.giro, args.comeco_de_pe = 0.0, 1.0, 0.0, 0.0
args.comando, args.estrela, args.manobras, args.locomocao_fogo = 0.0, 1.0, 0.0, 0.0
args.estrela_caminho, args.estrela_curvas = rework[:2], rework[2:]
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
args.semente = 41
b = L.get_up_batch(str(L.REPO / "assets"), environments=48, threads=4, **L.lote_do_treino(args), **L.montar_v2(args))
L.configurar(b, args, counters, True)
b.reset(41)
model = amb.model(b)
rows = []
for _ in range(int(round(40.0 / float(b.policy_dt)))):
    obs = torch.as_tensor(b.observations())
    with torch.no_grad():
        a = model.env_action(model.distribution(model.norm(obs)).sample(), obs).numpy().astype(np.float32)
    b.step(a)
    rows += b.finished_episodes().tolist()
r = np.array(rows, np.float32)
r = r[(r[:, 1] == 6) & (r[:, 26] >= 8) & (r[:, 26] <= 10)]
ok = L.loco_fire_ok(r)
print(f"estrela {rework}: {len(r)} episodios | sem cair {ok.mean():.2f} | fim por sair do caminho {(r[:, 32] == 6).mean():.2f} | pernas "
      f"{r[:, 43].mean():.2f} | curvas {r[:, 67].mean():.2f} | dist {r[r[:, 66] >= 0, 66].mean():.2f} m | duracao {r[:, 4].mean():.1f} s")
for m, name in ((8, "fixo"), (9, "livre"), (10, "girando")):
    x = r[r[:, 26] == m]
    if len(x):
        print(f"  {name}: n {len(x)} sem cair {L.loco_fire_ok(x).mean():.2f} pernas {x[:, 43].mean():.2f} curvas {x[:, 67].mean():.2f} "
              f"dist {x[x[:, 66] >= 0, 66].mean():.2f} m fora {(x[:, 32] == 6).mean():.2f}")
