#!/usr/bin/env python3
"""O quanto a rede se aproxima do juiz (07/11, ele: "fica medindo ai o quanto ele esta se aproximando do juiz"): por exercicio,
a nota media dos juizes da Unreal nos episodios das avaliacoes - (D + 1) / 2 em %, 100% = parece clipe para o juiz, 0% = nao
parece -, na primeira avaliacao com a medida, ha ~1 h e na ultima; e o treino (d_corpo e d_loco_corpo contra os clipes).
Uso: tools/rl/.venv/bin/python tools/rl/juiz_tendencia.py [--treino rl_final_clipe]"""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--treino", default="rl_final_clipe")
args = parser.parse_args()
log = Path.home() / ".local/share/matter-engine/rl/runs" / args.treino / "log.jsonl"
rows = [json.loads(line) for line in log.open() if line.strip()]
evals = [r for r in rows if r.get("aval") and isinstance(r.get("juiz_exercicios"), dict)]
if not evals:
    raise SystemExit("ainda sem avaliacao com a nota do juiz")
first, last = evals[0], evals[-1]
hour = next((r for r in reversed(evals) if last["hora"] - r["hora"] >= 3600), first)
pct = lambda d: None if d is None else 50.0 * (d + 1.0)
fmt = lambda v: "  -  " if v is None else f"{v:5.1f}"
print(f"avaliacoes com o juiz: {len(evals)} (atualizacao {first['atualizacao']} -> {last['atualizacao']})")
print(f"{'exercicio':26} {'inicio':>6} {'-1 h':>6} {'agora':>6}  (% parecido com os clipes)")
for key in sorted(last["juiz_exercicios"]):
    a, h, z = (pct(r["juiz_exercicios"].get(key)) for r in (first, hour, last))
    print(f"{key:26} {fmt(a):>6} {fmt(h):>6} {fmt(z):>6}")
train = [r for r in rows if not r.get("aval") and "d_loco_corpo" in r and r["atualizacao"] >= first["atualizacao"]]
if train:
    t0, t1 = train[0], train[-1]
    print(f"treino (com ruido) - locomocao: corpo {pct(t0['d_loco_corpo']):.1f}% -> {pct(t1['d_loco_corpo']):.1f}% "
          f"(clipes {pct(t1['d_loco_clipe']):.1f}%); levantar: corpo {pct(t0['d_corpo']):.1f}% -> {pct(t1['d_corpo']):.1f}% "
          f"(clipes {pct(t1['d_clipe']):.1f}%)")
