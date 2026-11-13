"""(12/11 15h00, a analise geral do levantar) As tendencias desde o zero das 13h51, por exercicio: o limpo (sucesso e tempo), sob fogo, firme,
levantou de novo e o tempo dela, a velocidade na troca, cada adversidade; e a prova (sem ruido, niveis fixos) com o juiz."""
import json, os, datetime
rows = [json.loads(l) for l in open(os.path.expanduser('~/.local/share/matter-engine/rl/runs/idle_alerta/log.jsonl'))]
K = (('costas', 'costas'), ('brucos', 'brucos'), ('correndo_costas', 'cor_costas'), ('correndo_brucos', 'cor_brucos'))
tr = [r for r in rows if 'treino_lu_costas_sob_fogo' in r]
av = [r for r in rows if 'lu_costas_sob_fogo' in r and 'treino_lu_costas_sob_fogo' not in r]
f = lambda v, d=2: format(v, f'.{d}f') if isinstance(v, (int, float)) else '-'
def mean(rs, key):
    xs = [r[key] for r in rs if isinstance(r.get(key), (int, float))]
    return sum(xs) / len(xs) if xs else None
print(f"linhas de treino {len(tr)}, provas {len(av)}; de {datetime.datetime.fromtimestamp(tr[0]['hora']):%H:%M} a {datetime.datetime.fromtimestamp(tr[-1]['hora']):%H:%M} (r{tr[0]['atualizacao']}-r{tr[-1]['atualizacao']})")
for k, n in K:
    p = 'treino_lu_' + k + '_'
    print(f"\n{n}:")
    for name, rs in (('comeco', tr[:6]), ('fim', tr[-6:])):
        print(f"  {name:6s} limpo {f(mean(rs, p + 'fogo_limpo'))} em {f(mean(rs, p + 'levantar_s'))}s | sob fogo {f(mean(rs, p + 'sob_fogo'))} firmeF {f(mean(rs, p + 'firme_sob_fogo'))} firme|lev {f(mean(rs, p + 'firme_se_levantou'))} | "
              f"tentou {f(mean(rs, p + 'tentou_de_novo'))} de novo {f(mean(rs, p + 'recuperou'))} em {f(mean(rs, p + 'denovo_s'))}s | vel troca {f(mean(rs, p + 'vel_troca'), 1)} | "
              + ' '.join(f"{x[:4]} {f(mean(rs, p + 'fogo_' + x))}" for x in ('serie', 'rasteira', 'prensado', 'vento', 'escorregadio', 'agarrao', 'depois_de_pe')))
    q = 'lu_' + k + '_'
    print(f"  prova  sob fogo {f(mean(av[-4:], q + 'sob_fogo'))} limpo {f(mean(av[-4:], q + 'fogo_limpo'))} em {f(mean(av[-4:], q + 'levantar_s'))}s juiz {f(mean(av[-4:], q + 'juiz'))} firme {f(mean(av[-4:], q + 'firme'))}")
print('\nniveis', tr[-1].get('fogo_levantar'))
