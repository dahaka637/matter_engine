"""(12/11 19h45, ele: no levantar e ir normal "demora um pouco para de fato trocar o brain... precisaria ser mais preciso") O lote em
modo de avaliacao do levantar e ir (o do laboratorio), a acao media: por episodio de pe, quando a cabeca passou de 1,3 m, quando ficou
"de pe" (cabeca 1,4, pelve 0,75), a entrega (col 18) e a troca de cabeca de verdade (a flag da observacao). Uso (de tools/rl):
python inspecao/sondas_locomocao_fogo/sonda_troca_de_pe.py CHECKPOINT RECEITA [MUNDOS]"""
import json, sys
import numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]
n = int(sys.argv[3]) if len(sys.argv) > 3 else 24
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
parser = L.build_parser()
args = parser.parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
L.aplicar_receita(parser, args)
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
b = L.get_up_batch(str(L.REPO / "assets"), environments=n, threads=4, seed=7, evaluation=True, evaluation_kind=11, **L.montar_v2(args))
L.configurar(b, args, counters, False)
L.configurar_levantar(b, args, 0.0)
b.set_get_up_go(1.0, 5.0, 10.0)
b.reset(7)
model = amb.model(b)
dt = float(b.policy_dt)
def alturas(i):
    h = p = 0.0
    for ln in b.frame_lines(i).splitlines():
        s = ln.split()
        if len(s) > 4 and s[0] in "BC":
            if s[1] == "Head": h = float(s[4])
            elif s[1] == "Pelvis": p = float(s[4])
    return h, p
trace = {0: '', n - 1: ''}
t13 = [-1.0] * n; tpe = [-1.0] * n; tm = [-1.0] * n; done = {}
for step in range(int(round(12.0 / dt))):
    obs = torch.as_tensor(b.observations())
    t = step * dt
    for i in range(n):
        if i in done: continue
        h, p = alturas(i)
        if t13[i] < 0 and h > 1.3: t13[i] = t
        if tpe[i] < 0 and h > 1.4 and p > 0.75: tpe[i] = t
        if tm[i] < 0 and obs[i, -1] > .5: tm[i] = t
        if i in trace and step % max(1, int(round(0.1 / dt))) == 0: trace[i] += f" {t:.1f}:{h:.2f}/{p:.2f}{'M' if obs[i, -1] > .5 else 'L'}"
    with torch.no_grad():
        a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
    b.step(a)
    for r in b.finished_episodes().tolist():
        if int(r[0]) not in done: done[int(r[0])] = r
    if len(done) == n: break
rows = []
for i, r in done.items():
    if r[72] < 1.5:
        rows.append((t13[i], tpe[i], r[18], tm[i], r[73], r[32], r[4], r[74]))
x = np.array(rows)
print(f"de pe: {len(x)} ep | cabeca>1,3 {np.median(x[:,0]):.2f}s | 'de pe' {np.median(x[:,1]):.2f}s | entrega {np.median(x[:,2]):.2f}s | "
      f"troca (flag M) {np.median(x[:,3]):.2f}s | atraso troca-de_pe {np.median(x[:,3]-x[:,1]):.2f}s (max {np.max(x[:,3]-x[:,1]):.2f}) | "
      f"chegou {np.mean(x[:,4] >= 0):.2f}")
for row in x: print("  " + " ".join(f"{v:5.2f}" for v in row))
for i, tr in trace.items(): print('mundo', i, 'tipo', done[i][72] if i in done else '?', 'clipe', done[i][56] if i in done else '?', tr)
