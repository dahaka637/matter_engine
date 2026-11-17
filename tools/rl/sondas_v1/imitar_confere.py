"""(13/11 18h05) O custo do ritmo chega na recompensa? O mesmo levantar limpo de costas (as mesmas acoes medias, a mesma semente) com o
ritmo ligado e desligado: a diferenca da recompensa por passo, e a altura da cabeca."""
import json, os, sys, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]; W = 8
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
args = L.build_parser().parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
args.locomocao, args.giro, args.comeco_de_pe, args.locomocao_fogo, args.comando = 0.0, 0.0, 0.0, 0.0, 0.0
args.levantar_unreal_meio, args.levantar_pose_variada = 0.0, 0.0
args.levantar_fogo = (1.0, 0.0, 0.0, 0.0)
args.levantar_e_ir = (0.0, args.levantar_e_ir[1], args.levantar_e_ir[2])
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
runs = {}
for pace in (0.0, 1.0):  # aqui: a nota de imitacao desligada e ligada
    args.levantar_ritmo = (3.0, 1.5); args.levantar_imitar = 1; args.levantar_imitar_nota = pace
    b = L.get_up_batch(str(L.REPO / "assets"), environments=W, threads=2, **L.lote_do_treino(args), **L.montar_v2(args))
    L.configurar(b, args, counters, True); L.configurar_levantar(b, args, 1.0)
    b.set_clip_get_up_weights([1.0, 0.0, 0.0, 0.0])
    model = amb.model(b); b.reset(4321); rew, heads = [], []
    acts = runs.get('acts')
    rec_acts = []
    for step in range(60):
        obs = torch.as_tensor(b.observations())
        if acts is None:
            with torch.no_grad():
                act = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
            rec_acts.append(act)
        else:
            act = acts[step]
        heads.append([b.get_up_form_now(w)[0] for w in range(W)])
        task, term, trunc = b.step(act)
        rew.append(np.array(task, np.float32))
    if acts is None: runs['acts'] = rec_acts
    runs[pace] = (np.array(rew), np.array(heads))
d = runs[1.0][0] - runs[0.0][0]
h = runs[1.0][1]
print("passo  cabeca(m)  recompensa sem - com ritmo (media dos mundos)")
for s in range(0, 60, 3):
    print(f"  {s * 1/30:4.2f}s  {h[s].mean():.2f}      {d[s].mean():+.3f}")
print(f"soma por levantar (sem - com): {d.sum(0).mean():.2f}  (a medida em python dava ~16 com peso 1)")
print("mesma trajetoria?", np.allclose(runs[0.0][1], runs[1.0][1], atol=1e-4))
