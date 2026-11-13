"""(12/11 18h) O comando ligado ao levantar: so o comando (o jogador emulado), como no treino, com --comando-levantar. Por episodio: as
quedas, os levantares de volta e o tempo deles; e, nos mundos que cairam, a sequencia das cabecas (L locomocao, R levantar) por
segundo. Uso (de tools/rl): python inspecao/sondas_locomocao_fogo/sonda_comando.py CHECKPOINT RECEITA [LIGADO=1]"""
import json, os, sys
import numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]
on = os.environ.get("LIGADO", "1") == "1"
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
args = L.build_parser().parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
args.levantar_unreal, args.locomocao, args.giro, args.comeco_de_pe = 0.0, 1.0, 0.0, 0.0
args.comando, args.comando_levantar = 1.0, on
args.locomocao_fogo = float(os.environ.get("FOGO", "0"))
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 4), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
args.semente = 51
b = L.get_up_batch(str(L.REPO / "assets"), environments=48, threads=4, **L.lote_do_treino(args), **L.montar_v2(args))
L.configurar(b, args, counters, True)
L.configurar_levantar(b, args, 0.0)
L.loco_fire_extras(b, args)
if args.locomocao_fogo > 0:
    b.set_locomotion_fire(args.locomocao_fogo, [0.3] * 6, args.locomocao_fogo_liberdade)
b.reset(51)
model = amb.model(b)
rows, heads = [], []
dt = float(b.policy_dt)
for step in range(int(round(45.0 / dt))):
    obs = torch.as_tensor(b.observations())
    heads.append((obs[:, -1] > .5).numpy().copy())
    with torch.no_grad():
        a = model.env_action(model.distribution(model.norm(obs)).sample(), obs).numpy().astype(np.float32)
    b.step(a)
    rows += b.finished_episodes().tolist()
r = np.array(rows, np.float32)
r = r[(r[:, 1] == 6) & (r[:, 26] == 12)]
h = np.array(heads)
print(f"comando {'LIGADO' if on else 'desligado'}: {len(r)} episodios, {r[:, 4].mean():.1f} s em media | quedas/ep {r[:, 68].mean():.2f} | "
      f"levantou de volta {r[:, 69].sum() / max(1, r[:, 68].sum()):.2f} ({int(r[:, 69].sum())}/{int(r[:, 68].sum())}) | tempo "
      f"{r[r[:, 70] >= 0, 70].mean() if (r[:, 70] >= 0).any() else -1:.2f} s | correndo {int(r[:, 71].sum()) if r.shape[1] > 71 else -1} | fim pelo tempo {(r[:, 32] == 0).mean():.2f} | causas "
      f"{dict(zip(*[x.astype(int).tolist() for x in np.unique(r[:, 32], return_counts=True)]))}")
switches = [w for w in range(h.shape[1]) if (np.diff(h[:, w].astype(int)) != 0).sum() >= 2]
for w in switches[:4]:
    seq = "".join("L" if h[int(s / dt), w] else "R" for s in np.arange(0, h.shape[0] * dt, 0.5))
    print(f"  mundo {w}: {seq}")
