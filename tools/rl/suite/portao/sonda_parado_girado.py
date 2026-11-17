"""(17/11 11h40, ele: "o boneco esta torto ainda") No jogo, parado, a r130240 e o C ficam girados ~30-40 graus com um pe atras; a
default fica de frente. No treino (lev_parado, a rede na acao media, 4 mundos, 2 a 6 s): o rumo da linha dos ombros, do quadril e
dos pes do corpo contra os do clipe (graus; positivo = girado para a esquerda) e o pe a frente (o desencontro dos pes no rumo do
clipe, cm). Uso (de tools/rl): python suite/portao/sonda_parado_girado.py CHECKPOINT"""
import sys

import numpy as np
import torch

sys.path.insert(0, ".")
from inspecao import comum  # noqa: E402

amb = comum.Ambiente("cerebro_unico", sys.argv[1])
amb.S.RUN_V2["velocidade_rotacao"] = True
b = amb.batch("lev_parado", 4, 61)
amb.setup(b, 1.0)
b.reset(61)
model = amb.model(b)
dt = float(b.policy_dt)
PAIRS = {"ombros": ("LeftUpperArm", "RightUpperArm"), "quadril": ("LeftThigh", "RightThigh"), "pes": ("LeftFoot", "RightFoot")}
acc = {k: [] for k in PAIRS}
stagger = []
absol = {}
for i in range(int(round(6.0 / dt))):
    obs = torch.as_tensor(np.asarray(b.observations()))
    with torch.no_grad():
        a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
    b.step(a)
    if i * dt < 2.0:
        continue
    for w in range(4):
        body, _, _ = comum.parse_lines(b.frame_lines(w))
        clip, _, _ = comum.parse_lines(b.reference_lines(w))
        yaw = lambda f, l, r: np.degrees(np.arctan2(*(np.array(f[l][:2]) - np.array(f[r][:2]))[::-1]))  # noqa: E731
        for k, (l, r) in PAIRS.items():
            acc[k].append(((yaw(body, l, r) - yaw(clip, l, r) + 180) % 360) - 180)
        cy = np.radians(yaw(clip, "LeftThigh", "RightThigh"))
        fwd = np.array([np.cos(cy - np.pi / 2), np.sin(cy - np.pi / 2)])
        fb = lambda f: (np.array(f["LeftFoot"][:2]) - np.array(f["RightFoot"][:2])) @ fwd  # noqa: E731
        stagger.append((100 * fb(body), 100 * fb(clip)))
        absol.setdefault(w, []).append([yaw(clip, "LeftUpperArm", "RightUpperArm"), yaw(clip, "LeftThigh", "RightThigh"),
                                        yaw(body, "LeftUpperArm", "RightUpperArm"), yaw(body, "LeftThigh", "RightThigh")])
print(" | ".join(f"{k}: corpo - clipe media {np.mean(v):+5.1f} graus (|p95| {np.percentile(np.abs(v), 95):4.1f})" for k, v in acc.items()))
s = np.array(stagger)
print(f"pe esquerdo a frente do direito: corpo {s[:, 0].mean():+5.1f} cm, clipe {s[:, 1].mean():+5.1f} cm")
for w, v in absol.items():
    v = np.array(v).mean(0)
    print(f"mundo {w}: rumo (linha esq-dir) clipe ombros {v[0]:+6.1f} quadril {v[1]:+6.1f} | corpo ombros {v[2]:+6.1f} quadril {v[3]:+6.1f} graus")
