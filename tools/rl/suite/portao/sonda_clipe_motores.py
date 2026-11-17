"""(17/11, a imitacao com a velocidade do clipe) O clipe tocado direto nos motores, sem a rede, para conferir os alvos da imitacao
supervisionada: a acao e o alvo do clipe da decisao (locomotion_targets: as posicoes e, com --imitar-velocidade, as velocidades por junta)
- com as velocidades em zero (os alvos de antes) ou as do clipe. Sem a mao (forca 0). Por andadura: a semelhanca e a nota do segundo 0,3
ao 1,5 (o corpo sem a rede cai), a velocidade e quando o episodio acabou. Uso (de tools/rl, build-base-n133): sonda_clipe_motores.py"""
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, ".")
from inspecao import comum  # noqa: E402

CK = Path.home() / ".local/share/matter-engine/rl/runs/cerebro_unico/final_15h_r130240.pt"
amb = comum.Ambiente("cerebro_unico", str(CK))
amb.S.RUN_V2["velocidade_rotacao"] = True
amb.S.RUN_V2["imitar_velocidade"] = True
terms = list(amb.L.TRACKING_TERMS)
GAITS = tuple(os.environ.get("GAITS", "lev_corrida_0,lev_trote_0").split(","))


def run(key, seed, with_velocity):
    b = amb.batch(key, 2, seed)
    amb.setup(b, 0.0)
    b.set_imitation_velocity_targets(True)
    b.reset(seed)
    names = list(b.action_names())
    vel = np.array([n.startswith("vel.") for n in names])
    dt = float(b.policy_dt)
    sem, nota, speed, ended = [], [], [], [None, None]
    for i in range(int(round(2.0 / dt))):
        a = np.asarray(b.locomotion_targets(), np.float32).copy()
        if not with_velocity:
            a[:, vel] = 0.0
        b.step(a)
        fb = np.asarray(b.locomotion_feedback())
        if 0.3 <= i * dt <= 1.5:
            sem.append(fb[:, terms.index("r_semelhanca")])
            nota.append(fb[:, terms.index("nota")])
            speed.append(fb[:, terms.index("velocidade")])
        for r in b.finished_episodes().tolist():
            w = int(r[0])
            if ended[w] is None:
                ended[w] = round(i * dt, 2)
    return float(np.mean(sem)), float(np.mean(nota)), float(np.mean(speed)), ended


for key in GAITS:
    for with_velocity in (False, True):
        res = [run(key, s, with_velocity) for s in (61, 62)]
        print(f"{key:14s} velocidade {'do clipe' if with_velocity else 'zero    '}: semelhanca {np.mean([r[0] for r in res]):.3f} "
              f"nota {np.mean([r[1] for r in res]):.3f} velocidade {np.mean([r[2] for r in res]):.2f} | fim {[r[3] for r in res]}", flush=True)
