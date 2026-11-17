"""(17/11 11h, o equilibrio do braco C) Os fatores da reta (o rumo medio e a reta no comando, da Fase 1) cobram o desvio que o empurrao
causa? A prova de fogo (como sonda_prova.py, trote e corrida) rodada duas vezes com a mesma rede e as mesmas sementes: a receita como
esta e a receita sem --reta-comando e sem --rumo-medio. Esses ajustes so mudam a nota (a trajetoria e a mesma - conferido pela soma das
observacoes), entao a diferenca da nota e o que eles cobram. Por andadura e tipo: a nota media com o fogo agindo (liberdade > 0) e sem.
Uso (de tools/rl): python suite/portao/sonda_reta_fogo.py CHECKPOINT RECEITA"""
import json, os, sys
import numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
args = L.build_parser().parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
v2 = L.montar_v2(args)
plain = dict(v2, moving_push_newtons=(0.0, 0.0), moving_push_hold_newtons=(0.0, 0.0))
terms = list(L.TRACKING_TERMS)
seed = int(os.environ.get("SEMENTE_PROVA", "0"))


def run(band, without):
    b = L.get_up_batch(str(L.REPO / "assets"), environments=32, threads=4, seed=780 + band + 97 * seed, evaluation=True,
                       evaluation_kind=4, evaluation_band=band, **plain)
    L.configurar(b, args, counters, False)
    b.set_locomotion_fire(0.0, [0.0] * 6, args.locomocao_fogo_liberdade)
    b.set_locomotion_fire_exam(True, 1.0)
    L.loco_fire_extras(b, args)
    b.set_drill_assist([0.0] * len(L.DRILL_KEYS))
    if without:
        b.set_straight_command(False)
        b.set_heading_bias(0.0, 0.10)
    b.reset(12345 + 1000 * seed)
    model = amb.model(b)
    nota, free, kind, check = [], [], None, 0.0
    alive = np.ones(32, bool)
    for _ in range(int(b.episode_seconds / b.policy_dt) + 2):
        o = np.asarray(b.observations())
        check += float(o[alive].sum())
        obs = torch.as_tensor(o)
        with torch.no_grad():
            a = model.env_action(model.actor(model.norm(obs)), obs).numpy().astype(np.float32)
        b.step(a)
        fb = np.asarray(b.locomotion_feedback())
        nota.append(np.where(alive, fb[:, terms.index("nota")], np.nan)); free.append(np.where(alive, fb[:, -1], np.nan))
        for r in b.finished_episodes().tolist():
            alive[int(r[0])] = False
        if not alive.any():
            break
    return np.array(nota), np.array(free), check


for name, band in (("trote", 1), ("corrida", 2)):
    na, fa, ca = run(band, False)
    nb, fb_, cb = run(band, True)
    n = min(len(na), len(nb))
    na, nb, fa = na[:n], nb[:n], fa[:n]
    kinds = (np.arange(32) // 8) % 4
    out = []
    for t, tipo in enumerate(("continuo", "serie", "agarrao", "molhado")):
        cols = kinds == t
        f = fa[:, cols]; a_ = na[:, cols]; b_ = nb[:, cols]
        on = f > 0.05; off = (f <= 0.05) & ~np.isnan(f)
        out.append(f"{tipo}: fogo {np.nanmean(a_[on]):.3f} vs {np.nanmean(b_[on]):.3f} | sem fogo {np.nanmean(a_[off]):.3f} vs {np.nanmean(b_[off]):.3f}")
    print(f"{name} (trajetoria igual: {abs(ca - cb) < 1e-3 * max(1.0, abs(ca))}) nota com/sem os fatores da reta - " + " | ".join(out), flush=True)
