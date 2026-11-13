"""(12/11 16h45) O prensado no parado: por passo, a altura da pelve e a distancia do corpo (os centros dos colisores) a face da parede.
Uso (de tools/rl): python inspecao/sondas_locomocao_fogo/sonda_prensado.py CHECKPOINT RECEITA"""
import json, math, os, sys
import numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
ck, receita = sys.argv[1], sys.argv[2]
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
args = L.build_parser().parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
args.levantar_unreal, args.locomocao, args.giro, args.comeco_de_pe = 0.0, 0.0, 0.0, 1.0
args.levantar_fogo_prensado = [1.0, 0.5, 3.0]
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
args.semente = 23
b = L.get_up_batch(str(L.REPO / "assets"), environments=8, threads=2, **L.lote_do_treino(args), **L.montar_v2(args))
L.configurar(b, args, counters, True)
L.loco_fire_extras(b, args)
b.set_locomotion_fire(1.0, [0.0] * 6, 1.0)
b.reset(23)
model = amb.model(b)
def parse(world):
    links, boxes = {}, []
    for line in b.frame_lines(world).splitlines():
        p = line.split()
        if p[0] in ("B", "C"): links[p[1]] = np.array([float(x) for x in p[2:5]])
        elif p[0] == "O": boxes.append([float(x) for x in p[1:]])
    return links, boxes
for step in range(60):
    if step % 3 == 0:
        out = []
        for wi in range(4):
            links, boxes = parse(wi)
            pel = links.get("Pelvis", np.zeros(3))[2]
            d = None
            for bx in boxes:
                c = np.array(bx[0:3]); q = bx[3:7]; half = bx[7:10]
                if half[1] < 2.5 or half[0] > 1.0: continue  # so a parede do prensado (6 m de comprimento, 0,3 de espessura)
                x, y, z, w_ = q
                axis = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w_ * z), 2 * (x * z - w_ * y)])
                d = min(abs(np.dot(p_ - c, axis)) - half[0] for p_ in links.values())
            out.append(f"w{wi} pelve {pel:.2f} parede {('%.2f' % d) if d is not None else '-'}")
        print(f"t {step * float(b.policy_dt):.2f}: " + " | ".join(out), flush=True)
    obs = torch.as_tensor(b.observations())
    with torch.no_grad():
        a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
    b.step(a)
