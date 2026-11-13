"""(12/11 18h50, ele: no levantar correndo "assim que ele comeca ficar de pe ja trocar para a locomocao.. nunca deixar o brain do levantar ser
usado para a locomocao em si" e cortar o exercicio nos "primeiros dois passos" depois) So os levantares correndo, limpos, a acao media.
Por exercicio: levantou, firme ate o fim (sem cair depois da troca), os segundos do episodio e os da cabeca do levantar e da locomocao
por episodio. Uso (de tools/rl): python inspecao/sondas_locomocao_fogo/sonda_troca_correndo.py CHECKPOINT RECEITA
(TROCA=0.7: --levantar-correndo-troca; sem: como a receita)."""
import json, os, sys
import numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
args = L.build_parser().parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
if "TROCA" in os.environ: args.levantar_correndo_troca = float(os.environ["TROCA"])
args.locomocao, args.giro, args.comeco_de_pe, args.locomocao_fogo, args.comando = 0.0, 0.0, 0.0, 0.0, 0.0
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
for ex in (2, 3):
    args.semente = 41 + ex
    b = L.get_up_batch(str(L.REPO / "assets"), environments=48, threads=4, **L.lote_do_treino(args), **L.montar_v2(args))
    L.configurar(b, args, counters, True)
    L.configurar_levantar(b, args, 1.0)
    w = [0.0] * len(L.LEVANTAR_UNREAL_KEYS); w[ex] = 1.0; b.set_clip_get_up_weights(w)
    b.reset(41 + ex)
    model = amb.model(b)
    rows, rec, loco = [], 0, 0
    dt = float(b.policy_dt)
    for _ in range(int(round(30.0 / dt))):
        obs = torch.as_tensor(b.observations())
        flag = obs[:, -1] > .5
        rec += int((~flag).sum()); loco += int(flag.sum())
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        rows += b.finished_episodes().tolist()
    x = np.array(rows, np.float32); x = x[x[:, 1] == 9]
    up = x[:, 18] >= 0
    firm = up & ~np.isin(x[:, 32], (1, 2, 3, 4, 8))
    n = max(1, len(x))
    print(f"{L.LEVANTAR_UNREAL_KEYS[ex]} troca {args.levantar_correndo_troca}: {len(x)} ep | levantou {up.mean():.2f} firme {firm.mean():.2f} "
          f"firme|lev {firm[up].mean() if up.any() else -1:.2f} | entrega aos {np.median(x[up, 18]) if up.any() else -1:.2f} s, episodio "
          f"{np.median(x[:, 4]):.2f} s | por episodio: cabeca do levantar {rec * dt / n:.2f} s, da locomocao {loco * dt / n:.2f} s", flush=True)
