"""(14/11 v1) O especialista do de costas contra o principal: semelhanca, levantou, longe (fim cedo), tempo, kl."""
import json, os
def rows(run):
    return [json.loads(l) for l in open(os.path.expanduser(f'~/.local/share/matter-engine/rl/runs/{run}/log.jsonl')) if l.strip()]
for run in ('v1_costas_especialista', 'cerebro_unico'):
    rs = rows(run)
    tr = [r for r in rs if not r.get('aval') and r.get('atualizacao', 0) > 116000 and 'treino_lu_costas_levantar_s' in r]
    av = [r for r in rs if r.get('aval') and r.get('atualizacao', 0) > 116000 and 'lu_costas_semelhanca' in r]
    print(f"== {run}")
    for r in tr[-4:]:
        g = lambda k: r.get(f'treino_lu_costas_{k}') or 0
        print(f"  {r['atualizacao']} treino costas: sem {g('semelhanca'):.3f} levantou {g('levantou'):.2f} longe {g('longe'):.2f} "
              f"tempo {g('levantar_s'):.2f} ep {g('episodios')} | kl_lev {r.get('kl_rec') or 0:.3f} taxa {r.get('taxa_rec') or 0:.1e}")
    for r in av[-2:]:
        print(f"  {r['atualizacao']} aval costas: sem {r.get('lu_costas_semelhanca') or 0:.3f} firme_limpo {r.get('lu_costas_firme_limpo') or 0:.2f}"
              f" | brucos sem {r.get('lu_brucos_semelhanca') or 0:.3f}")
