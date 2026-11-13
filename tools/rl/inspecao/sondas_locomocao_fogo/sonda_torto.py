"""(12/11 17h) O parado torto: so parados, todos nascendo tortos (crookPose), sem fogo; a acao sorteada como no treino (MEDIA=1: a media).
Por limiar de "perto" (0,5 forma + 0,5 lugar): quantos reajustaram, em quanto tempo, e os que cairam. Uso (de tools/rl):
python inspecao/sondas_locomocao_fogo/sonda_torto.py CHECKPOINT RECEITA PERTO [PERTO...]   (TORTO=0: os normais, para comparar)"""
import json, os, sys
import numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]
mean = os.environ.get("MEDIA", "0") == "1"
share = float(os.environ.get("TORTO", "1.0"))
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
for close in [float(x) for x in sys.argv[3:]]:
    args = L.build_parser().parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
    args.levantar_unreal, args.locomocao, args.giro, args.comeco_de_pe = 0.0, 0.0, 0.0, 1.0
    args.locomocao_fogo = float(os.environ.get("FOGO", "0"))  # (18h20) FOGO=0.9: com o fogo da locomocao em L 0,3
    counters = amb.state.get('counters', {})
    for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
        counters.setdefault(k, v)
    counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
    args.semente = 31
    b = L.get_up_batch(str(L.REPO / "assets"), environments=32, threads=4, **L.lote_do_treino(args), **L.montar_v2(args))
    L.configurar(b, args, counters, True)
    L.configurar_levantar(b, args, 0.0)
    L.loco_fire_extras(b, args)
    if args.locomocao_fogo > 0: b.set_locomotion_fire(args.locomocao_fogo, [0.3] * 6, args.locomocao_fogo_liberdade)
    b.set_crooked_start(share, 2.0, close)
    b.reset(31)
    model = amb.model(b)
    rows = []
    for _ in range(int(round(26.0 / float(b.policy_dt)))):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            dist = model.distribution(model.norm(obs))
            a = model.env_action(dist.mean if mean else dist.sample(), obs).numpy().astype(np.float32)
        b.step(a)
        rows += b.finished_episodes().tolist()
    r = np.array(rows, np.float32)
    r = r[r[:, 1] == 2]
    crook = r[:, 64] > 0
    fixed = r[:, 65] >= 0
    ok = L.loco_fire_ok(r)
    t = r[crook & fixed, 65]
    print(f"perto {close}: {len(r)} parados ({int(crook.sum())} tortos) | tortos: nao caiu {ok[crook].mean() if crook.any() else -1:.2f}, "
          f"reajustou {fixed[crook].mean() if crook.any() else -1:.2f}, tempo med {np.median(t) if len(t) else -1:.2f} s "
          f"(p25 {np.percentile(t, 25) if len(t) else -1:.2f}, p75 {np.percentile(t, 75) if len(t) else -1:.2f}) | normais: nao caiu "
          f"{ok[~crook].mean() if (~crook).any() else -1:.2f} (n {int((~crook).sum())})", flush=True)
