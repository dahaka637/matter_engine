"""(12/11 16h30) O fogo da locomocao num lote como o do treino (vault 04_locomocao/plano_fogo_da_locomocao_12_11.md): so episodios de
locomocao (rumos, manobras, estrela, comando, giro, parado - nas fatias da receita, sem levantar), todos os elegiveis com fogo no nivel L,
a acao sorteada como no treino (ou a media com MEDIA=1). Por grupo: nao caiu, firme (nao caiu e seguiu a tarefa >= 70%), seguiu, e o
limpo (sem fogo) do mesmo lote para comparar.

Uso (de tools/rl, com MATTER_TRACKING_NATIVE do build): python inspecao/sondas_locomocao_fogo/sonda_nivel.py CHECKPOINT RECEITA L
[FATIA=1.0] [LIBERDADE=1.0] [SEGUNDOS=40] [MUNDOS=96]
"""
import json
import math
import os
import sys

import numpy as np
import torch

sys.path.insert(0, '.')
from inspecao import comum

ck, receita, level = sys.argv[1], sys.argv[2], float(sys.argv[3])
share = float(os.environ.get("FATIA", "1.0"))
relief = float(os.environ.get("LIBERDADE", "1.0"))
seconds = float(os.environ.get("SEGUNDOS", "40"))
worlds = int(os.environ.get("MUNDOS", "96"))
mean = os.environ.get("MEDIA", "0") == "1"
amb = comum.Ambiente('idle_alerta', ck)
L = amb.L
rec = json.load(open(receita))['argumentos']
args = L.build_parser().parse_args(rec + ['--treino', 'sonda'])
# (So locomocao: as fatias da receita sem o levantar, na mesma proporcao.)
total = args.locomocao + args.giro + args.comeco_de_pe
args.levantar_unreal = 0.0
args.locomocao, args.giro, args.comeco_de_pe = args.locomocao / total, args.giro / total, args.comeco_de_pe / total
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", int(args.comando_fase)), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", max(0.0, args.fogo)),
             ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6)
counters.setdefault("fogo_niveis", None)
args.semente = 17
v2 = L.montar_v2(args)
b = L.get_up_batch(comum.REPO_ASSETS if hasattr(comum, "REPO_ASSETS") else str(L.REPO / "assets"), environments=worlds, threads=4,
                   **L.lote_do_treino(args), **v2)
L.configurar(b, args, counters, True)
L.loco_fire_extras(b, args)
b.set_locomotion_fire(share, [level] * 6, relief)
if os.environ.get("INTERVALO") and hasattr(b, "set_locomotion_fire_gaps"):  # (12/11 20h) a escala do intervalo entre os eventos
    b.set_locomotion_fire_gaps(float(os.environ["INTERVALO"]))
b.reset(17)
model = amb.model(b)
rows = []
for _ in range(int(round(seconds / float(b.policy_dt)))):
    obs = torch.as_tensor(b.observations())
    with torch.no_grad():
        dist = model.distribution(model.norm(obs))
        act = dist.mean if mean else dist.sample()
        a = model.env_action(act, obs).numpy().astype(np.float32)
    b.step(a)
    rows += b.finished_episodes().tolist()
r = np.array(rows, np.float32)
if os.environ.get("SALVAR"):
    np.save(os.environ["SALVAR"], r)
print(f"L {level} fatia {share} liberdade {relief} {'media' if mean else 'sorteada'}: {len(r)} episodios; comecos "
      f"{dict(zip(*[x.astype(int).tolist() for x in np.unique(r[:, 1], return_counts=True)]))}")
summary = L.loco_fire_summary(r)
for g, e in summary.items():
    print(f"  {g:7s} limpo {e.get('limpo')} (n {e['n_limpo']}) | fogo n {e['n_fogo']} nao caiu {e.get('sem_cair')} firme {e.get('firme')} "
          f"seguiu {e.get('seguiu')} " + " ".join(f"{k} {e[k]}" for k in ("serie", "rasteira", "prensado", "vento", "molhado", "agarrao") if k in e))
fire = r[:, 60] > 0
if fire.any():
    print(f"  eventos por episodio de fogo: {r[fire, 62].mean():.2f} (parado {r[fire & (r[:, 1] == 2), 62].mean():.2f})")
print(f"  todos: fogo n {int(fire.sum())} nao caiu {float(L.loco_fire_ok(r[fire]).mean()) if fire.any() else -1:.3f} | limpo n {int((~fire).sum())} "
      f"nao caiu {float(L.loco_fire_ok(r[~fire]).mean()) if (~fire).any() else -1:.3f}")
