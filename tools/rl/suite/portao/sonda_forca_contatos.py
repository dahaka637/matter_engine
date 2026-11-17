"""(17/11, Fase 1 - D2) A prova positiva da forca dos contatos na entrada: o parado prensado contra a parede (o fogo "prensado" do
treino, como a sonda_prensado), a rede r130240, sem e com --forca-contatos. Sem: a entrada mostra so o empurrao do treino. Com: soma a
reacao da parede (estatica, normal de lado) - parado e prensado, a soma deve cair perto de zero (a parede segura o empurrao).
Uso (de tools/rl, MATTER_TRACKING_NATIVE da build-base-n131): python suite/portao/sonda_forca_contatos.py"""
import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, ".")
from inspecao import comum  # noqa: E402

RUN = Path.home() / ".local/share/matter-engine/rl/runs/cerebro_unico"
amb = comum.Ambiente("cerebro_unico", str(RUN / "final_15h_r130240.pt"))
L = amb.L


def run(contacts: bool):
    args = L.build_parser().parse_args(json.load(open(RUN / "receita_principal_etapa15.json"))["argumentos"] + ["--treino", "sonda"])
    # (A parede ou a caixa encostada no corpo desde o comeco - o --obstaculo -, no levantar e de pe: o toque garantido. O prensado
    # da sonda_prensado comeca com a parede a 3 m e em 3 s ainda nao encostava.)
    args.obstaculo = 1.0
    args.forca_contatos = contacts
    args.semente = 23
    counters = amb.state.get("counters", {})
    for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
        counters.setdefault(k, v)
    counters.setdefault("estrela_fases", [1] * 6)
    counters.setdefault("fogo_niveis", None)
    b = L.get_up_batch(str(L.REPO / "assets"), environments=4, threads=2, **L.lote_do_treino(args), **L.montar_v2(args))
    L.configurar(b, args, counters, True)
    L.loco_fire_extras(b, args)
    b.set_locomotion_fire(1.0, [0.0] * 6, 1.0)
    b.reset(23)
    model = amb.model(b)
    names = list(b.observation_names())
    fire = [names.index(n) for n in ("fogo.forca.frente", "fogo.forca.lado", "fogo.forca.cima")]
    out = []
    for _ in range(150):
        obs_np = np.asarray(b.observations())
        out.append(obs_np[:, fire].copy())
        obs = torch.as_tensor(obs_np)
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        b.finished_episodes()
    return np.array(out)  # (passos, mundos, 3) em pesos do corpo


off, on = run(False), run(True)
print("passo | sem: |forca| por mundo (pesos do corpo) | com: |forca| por mundo")
for i in range(0, 150, 10):
    print(f"{i:3d} | " + " ".join(f"{np.linalg.norm(off[i, w]):5.2f}" for w in range(4)) + " | "
          + " ".join(f"{np.linalg.norm(on[i, w]):5.2f}" for w in range(4)), flush=True)
diff = np.abs(on - off).max(axis=2)
print(f"passos com a entrada diferente: {(diff > 1e-6).sum()} de {diff.size}; maior diferenca {diff.max():.2f} pesos do corpo")
