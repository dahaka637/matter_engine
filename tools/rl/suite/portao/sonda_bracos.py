"""(17/11, o plano de desentortar - os bracos abertos) O braco firme sem treinar: a rede r130240 na acao media com o tonus dos bracos
(rigidez e amortecimento de UpperArm, Forearm e Hand vezes T; build-base-n132), 2 mundos, sementes 61 e 62, 6 s, do segundo 2 ao fim.
Por tonus e andadura: a abertura dos cotovelos, das maos e dos pes (cm, de lado no rumo do deslocamento, a partir do peito alto), o
mesmo no clipe, a semelhanca, a nota e as quedas. Uso (de tools/rl): python suite/portao/sonda_bracos.py [TONUS ...]"""
import os
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, ".")
from inspecao import comum  # noqa: E402

CK = Path.home() / ".local/share/matter-engine/rl/runs/cerebro_unico/final_15h_r130240.pt"
amb = comum.Ambiente("cerebro_unico", str(CK))
# (MATTER_ASSETS_SONDA: outra pasta de assets - o perfil do corpo mudado so para a sonda.)
if os.environ.get("MATTER_ASSETS_SONDA"):
    amb.S.ASSETS = os.environ["MATTER_ASSETS_SONDA"]
tones = [float(x) for x in sys.argv[1:]] or [1.0, 2.0, 3.0]
# (ROTACAO=1: a velocidade do clipe por junta nos bracos e na cabeca - --velocidade-rotacao, build-base-n132.)
if os.environ.get("ROTACAO") == "1":
    amb.S.RUN_V2["velocidade_rotacao"] = True
GAITS = tuple(os.environ.get("GAITS", "lev_corrida_0,lev_trote_0").split(","))
terms = list(amb.L.TRACKING_TERMS)
PARTS = ("Forearm", "Hand", "Foot")


def run(tone, key, seed):
    b = amb.batch(key, 2, seed)
    amb.setup(b, amb.level(key))
    b.set_arm_tone(tone)
    b.reset(seed)
    model = amb.model(b)
    dt = float(b.policy_dt)
    steps = int(round(6.0 / dt))
    first = int(2.0 / dt)
    frames = {"corpo": [[], []], "clipe": [[], []]}
    sem, nota, ends = [], [], 0
    for i in range(steps):
        obs = torch.as_tensor(np.asarray(b.observations()))
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        b.step(a)
        fb = np.asarray(b.locomotion_feedback())
        if i >= first:
            sem.append(fb[:, terms.index("r_semelhanca")])
            nota.append(fb[:, terms.index("nota")])
            for w in range(2):
                links, _, _ = comum.parse_lines(b.frame_lines(w))
                rlinks, _, _ = comum.parse_lines(b.reference_lines(w))
                frames["corpo"][w].append(links)
                frames["clipe"][w].append(rlinks)
        ends += len(b.finished_episodes())
    out = {}
    for label in ("corpo", "clipe"):
        acc = {p: [] for p in PARTS}
        for w in range(2):
            fr = [f for f in frames[label][w] if all(k in f for k in ("Pelvis", "UpperChest", "LeftForearm", "RightForearm"))]
            if len(fr) < 10:
                continue
            pel = np.array([f["Pelvis"][:2] for f in fr])
            d = pel[-1] - pel[0]
            fwd = d / (np.linalg.norm(d) + 1e-9)
            left = np.array([-fwd[1], fwd[0]])
            for p in PARTS:
                L = np.array([(np.array(f["Left" + p][:2]) - np.array(f["UpperChest"][:2])) @ left for f in fr])
                R = np.array([(np.array(f["Right" + p][:2]) - np.array(f["UpperChest"][:2])) @ left for f in fr])
                acc[p].append(100 * (L - R).mean())
        out[label] = {p: float(np.mean(v)) if v else float("nan") for p, v in acc.items()}
    return out, float(np.mean(sem)), float(np.mean(nota)), ends


for key in GAITS:
    for tone in tones:
        res = [run(tone, key, s) for s in (61, 62)]
        body = {p: np.mean([r[0]["corpo"][p] for r in res]) for p in PARTS}
        clip = {p: np.mean([r[0]["clipe"][p] for r in res]) for p in PARTS}
        print(f"{key:14s} tonus {tone:3.1f}: cotovelos {body['Forearm']:5.1f} (clipe {clip['Forearm']:5.1f}) maos {body['Hand']:5.1f} "
              f"(clipe {clip['Hand']:5.1f}) pes {body['Foot']:5.1f} (clipe {clip['Foot']:5.1f}) cm | semelhanca "
              f"{np.mean([r[1] for r in res]):.3f} nota {np.mean([r[2] for r in res]):.3f} | episodios fechados {sum(r[3] for r in res)}",
              flush=True)
