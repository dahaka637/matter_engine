"""(14/11 v1, a correcao 7) O braco G: a imitacao do clipe de costas e o levantar (semelhanca, levantou, tempo) contra o principal."""
import json, os
def rows(run):
    return [json.loads(l) for l in open(os.path.expanduser(f'~/.local/share/matter-engine/rl/runs/{run}/log.jsonl')) if l.strip()]
ex = ('costas', 'brucos', 'correndo_costas', 'correndo_brucos')
for run in ('v1_costas_h', 'v1_costas_h2', 'cerebro_unico'):
    rs = rows(run)
    tr = [r for r in rs if not r.get('aval') and r.get('atualizacao', 0) > 115000 and 'treino_lu_costas_levantar_s' in r]
    av = [r for r in rs if r.get('aval') and r.get('atualizacao', 0) > 115000 and 'lu_costas_semelhanca' in r]
    print(f"== {run}")
    for r in tr[-3:]:
        print(f"  {r['atualizacao']} treino: sem " + ' '.join(f"{r.get(f'treino_lu_{e}_semelhanca') or 0:.3f}" for e in ex)
              + " | levantou " + ' '.join(f"{r.get(f'treino_lu_{e}_levantou') or 0:.2f}" for e in ex)
              + " | tempo " + ' '.join(f"{r.get(f'treino_lu_{e}_levantar_s') or 0:.2f}" for e in ex)
              + (f" | imit {r.get('imitacao_levantar'):.3f}" if r.get('imitacao_levantar') is not None else "")
              + f" | kl_lev {r.get('kl_rec') or 0:.3f}")
    for r in av[-2:]:
        print(f"  {r['atualizacao']} aval: sem " + ' '.join(f"{r.get(f'lu_{e}_semelhanca') or 0:.3f}" for e in ex)
              + " | firme limpo " + ' '.join(f"{r.get(f'lu_{e}_firme_limpo') or 0:.2f}" for e in ex))
