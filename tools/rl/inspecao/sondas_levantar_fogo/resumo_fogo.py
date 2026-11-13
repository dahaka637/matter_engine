"""Resumo da fase de fogo do levantar (12/11 05h): a ultima linha de treino e a ultima prova, por exercicio - sob fogo, por cenario,
recuperou, firme, tempo limpo, juiz - e os niveis."""
import datetime, json, os
LOG = os.path.expanduser("~/.local/share/matter-engine/rl/runs/idle_alerta/log.jsonl")
P = (("costas", "lu_costas_"), ("brucos", "lu_brucos_"), ("cor_costas", "lu_correndo_costas_"), ("cor_brucos", "lu_correndo_brucos_"))
with open(LOG, "rb") as f:
    f.seek(0, 2); f.seek(max(0, f.tell() - 1_500_000)); lines = f.read().decode("utf-8", "replace").splitlines()[1:]
rows = []
for l in lines:
    if '"treino_lu_' in l or '"lu_costas_' in l:
        try: rows.append(json.loads(l))
        except json.JSONDecodeError: pass
f = lambda v, d=2: format(v, f".{d}f") if isinstance(v, (int, float)) else "-"
tr = [r for r in rows if not r.get("aval")]; av = [r for r in rows if r.get("aval")]
parts = []
if tr:
    r = tr[-1]
    parts.append(f"r{r['atualizacao']} {datetime.datetime.fromtimestamp(r['hora']).strftime('%H:%M')} kl {f(r.get('kl_rec'), 3)} estilo {f(r.get('estilo'))}")
    g = lambda k, d=2: f(r.get('treino_' + k), d)
    parts.append("TREINO " + " ".join(
        f"{n}: LIMPO {g(p + 'fogo_limpo')} em {g(p + 'levantar_s')}s vel {g(p + 'vel_troca', 1)} POSE {g(p + 'pose_variada')} em {g(p + 'pose_variada_s')}s | fogo {g(p + 'sob_fogo')} [ser {g(p + 'fogo_serie')} ras {g(p + 'fogo_rasteira')} pre {g(p + 'fogo_prensado')} ven {g(p + 'fogo_vento')} dpe {g(p + 'fogo_depois_de_pe')}] denovo {g(p + 'recuperou')} em {g(p + 'denovo_s')}s firmeF {g(p + 'firme_sob_fogo')}"
        for n, p in P))
    if r.get("fogo_levantar"):
        parts.append("NIVEIS " + " ".join(f"{k[15:]}:{v}" for k, v in r["fogo_levantar"].items()))
if av:
    r = av[-1]
    parts.append(f"PROVA r{r['atualizacao']} " + " ".join(
        f"{n}: fogo {f(r.get(p + 'sob_fogo'))} rec {f(r.get(p + 'recuperou'))} firme {f(r.get(p + 'firme'))} firmeF {f(r.get(p + 'firme_sob_fogo'))} {f(r.get(p + 'levantar_s'))}s juiz {f(50 * (r[p + 'juiz'] + 1) if isinstance(r.get(p + 'juiz'), (int, float)) else None, 0)}%"
        for n, p in P))
print(" || ".join(parts))
