"""(17/11, o plano de desentortar - Fase 1, item 6) Quais saidas da rede nao mexem no corpo na locomocao: a rede r130240 na acao media,
2 mundos, 12 decisoes, o mesmo comeco (reset com a mesma semente); para cada saida k, a mesma corrida com a saida k somada de +0,8
(normalizada) em toda decisao. Morta: o corpo no fim igual ao da corrida sem mexer (todos os elos, bit a bit). Uma saida que so age
no levantar aparece morta aqui. Uso (de tools/rl, build-base-n131): python suite/portao/sonda_saidas_mortas.py [EXERCICIO ...]"""
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, ".")
from inspecao import comum  # noqa: E402

RUN = Path.home() / ".local/share/matter-engine/rl/runs/cerebro_unico"
amb = comum.Ambiente("cerebro_unico", str(RUN / "final_15h_r130240.pt"))
keys = sys.argv[1:] or ["lev_corrida_0", "lev_trote_0", "lev_comando"]
DELTA, STEPS, SEED = 0.8, 12, 61


def final_state(key, k):
    # (Um lote novo a cada corrida: o reset no mesmo lote nao repete o comeco - o estado do Jolt e dos mundos fica.)
    b = amb.batch(key, 2, SEED)
    amb.setup(b, amb.level(key))
    b.reset(SEED)
    model = amb.model(b)
    for _ in range(STEPS):
        obs = torch.as_tensor(np.asarray(b.observations()))
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        if k is not None:
            a[:, k] += DELTA
        b.step(a)
        b.finished_episodes()
    return [b.frame_lines(w) for w in range(2)]


dead_all = None
for key in keys:
    b = amb.batch(key, 2, SEED)
    names = list(b.action_names())
    del b
    ref = final_state(key, None)
    again = final_state(key, None)
    if ref != again:
        print(f"{key}: o comeco nao se repete - a sonda nao vale aqui", flush=True)
        continue
    dead = [names[k] for k in range(len(names)) if final_state(key, k) == ref]
    print(f"{key}: {len(dead)} de {len(names)} saidas sem efeito: {' '.join(dead)}", flush=True)
    dead_all = set(dead) if dead_all is None else dead_all & set(dead)
if dead_all is not None:
    print(f"MORTAS em todos ({len(dead_all)}): {' '.join(sorted(dead_all))}", flush=True)
