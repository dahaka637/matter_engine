"""(12/11 16h45) A prova de fogo da locomocao como a capacidade a monta (a avaliacao 4 no trote e na corrida, a 3 no parado; 32 mundos,
uma adversidade por mundo no nivel da prova), a rede media. Por faixa e tipo: nao caiu e seguiu. Uso (de tools/rl):
python inspecao/sondas_locomocao_fogo/sonda_prova.py CHECKPOINT RECEITA [L=1.0]"""
import json, sys
import numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]
level = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
args = L.build_parser().parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
v2 = L.montar_v2(args)
plain = dict(v2, moving_push_newtons=(0.0, 0.0), moving_push_hold_newtons=(0.0, 0.0))
res = {}
for name, kind, band in (("trote", 4, 1), ("corrida", 4, 2), ("parado", 3, 0)):
    kw = dict(evaluation_kind=kind, **({"evaluation_band": band} if band else {}))
    b = L.get_up_batch(str(L.REPO / "assets"), environments=32, threads=4, seed=780 + band, evaluation=True, **kw, **plain)
    L.configurar(b, args, counters, False)
    b.set_locomotion_fire(0.0, [0.0] * 6, args.locomocao_fogo_liberdade)
    b.set_locomotion_fire_exam(True, level)
    L.loco_fire_extras(b, args)
    b.set_drill_assist([0.0] * len(L.DRILL_KEYS))
    b.reset(12345)
    model = amb.model(b)
    done = {}
    for _ in range(int(b.episode_seconds / b.policy_dt) + 2):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            a = model.env_action(model.actor(model.norm(obs)), obs).numpy().astype(np.float32)
        b.step(a)
        for r in b.finished_episodes().tolist():
            done.setdefault(int(r[0]), r)
        if len(done) >= b.num_envs:
            break
    rows = np.array(list(done.values()), np.float32)
    for t, tipo in enumerate(("continuo", "serie", "agarrao", "molhado")):
        m = (rows[:, 0].astype(int) // 8) % 4 == t
        ok = L.loco_fire_ok(rows[m])
        keep = rows[m, 63]
        res[f"{name}_{tipo}"] = (round(float(ok.mean()), 2), round(float(keep[keep >= 0].mean()), 2) if (keep >= 0).any() else None, int(m.sum()),
                                 int((rows[m, 60] > 0).sum()), round(float(rows[m, 62].mean()), 1))
print(f"prova L {level}: " + " | ".join(f"{k}: nao caiu {v[0]} seguiu {v[1]} (n {v[2]}, fogo {v[3]}, eventos {v[4]})" for k, v in res.items()))
