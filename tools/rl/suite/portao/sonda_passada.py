"""(17/11 11h, a passada curta de lado) Na corrida, a coxa balanca ~74 graus contra ~99 do clipe. Quem encurta: a rede (o alvo que ela
manda) ou o corpo (o motor no limite de torque, a junta no batente)? A rede na acao media, 2 mundos (sementes 61 e 62), 6 s, do segundo 2
ao fim. Por eixo das coxas: a amplitude (p95 - p5, graus) do alvo da rede, do alvo do clipe (locomotion_targets) e da posicao alcancada
(body_joint_telemetry), e a fracao do tempo com o motor no limite de torque (>= 98%) e com torque de restricao (o batente).
Uso (de tools/rl): python suite/portao/sonda_passada.py CHECKPOINT [ANDADURA]"""
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, ".")
from inspecao import comum  # noqa: E402
from inspecao.geometria import IDX  # noqa: E402

ck = sys.argv[1]
key = sys.argv[2] if len(sys.argv) > 2 else "lev_corrida_0"
amb = comum.Ambiente("cerebro_unico", ck)
amb.S.RUN_V2["velocidade_rotacao"] = True
# (CLIPE=1: o alvo do clipe nos motores no lugar da rede - com as velocidades, --imitar-velocidade -, de 0,3 a 1,5 s, antes de cair.)
CLIPE = __import__("os").environ.get("CLIPE") == "1"
# (Sempre: o alvo do clipe com as velocidades, para comparar as velocidades-alvo da rede com as do clipe.)
amb.S.RUN_V2["imitar_velocidade"] = True


def run(seed):
    b = amb.batch(key, 2, seed)
    amb.setup(b, amb.level(key))
    b.set_imitation_velocity_targets(True)
    # (TONUS=x: a rigidez e o amortecimento de todos os motores vezes x, a forca maxima igual - --tonus.)
    b.set_motor_tone(float(__import__("os").environ.get("TONUS", "1.0")))
    b.reset(seed)
    model = amb.model(b)
    names = list(b.action_names())
    dt = float(b.policy_dt)
    rec = {"rede": [], "clipe": [], "corpo": [], "sat": [], "bat": []}
    t0, t1 = (0.3, 1.5) if CLIPE else (2.0, 6.0)
    for i in range(int(round(t1 / dt))):
        obs = torch.as_tensor(np.asarray(b.observations()))
        clip = np.asarray(b.locomotion_targets(), np.float32).copy()
        with torch.no_grad():
            a = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        if CLIPE:
            a = clip
        b.step(a)
        if i * dt >= t0:
            tel = np.stack([np.asarray(b.body_joint_telemetry(w)) for w in range(2)])
            rec["rede"].append(a); rec["clipe"].append(clip); rec["corpo"].append(tel[:, :, 9:12])
            rec["sat"].append(np.abs(tel[:, :, 0:3]) >= 0.98 * np.maximum(tel[:, :, 3:6], 1e-6))
            rec["bat"].append(np.abs(tel[:, :, 6:9]) > 1.0)
            rec.setdefault("tau", []).append(tel[:, :, 0:6])
            # (O pe a frente da pelve no rumo do deslocamento, corpo e clipe - de onde ele pousa e de onde sai.)
            for label, fn in (("pe_corpo", b.frame_lines), ("pe_clipe", b.reference_lines)):
                row = []
                for w in range(2):
                    links, _, _ = comum.parse_lines(fn(w))
                    row.append([links["Pelvis"][:2], links["LeftFoot"][:2], links["RightFoot"][:2]])
                rec.setdefault(label, []).append(row)
    return names, {k: np.array(v) for k, v in rec.items()}


names, r = run(61)
_, r2 = run(62)
r = {k: np.concatenate([r[k], r2[k]], 1) for k in r}
njoint = r["corpo"].shape[2]
print("juntas na telemetria:", njoint, "| acoes:", len(names), names[:6])
amp = lambda x: float(np.degrees(np.percentile(x, 95) - np.percentile(x, 5)))  # noqa: E731
for side in ("Left", "Right"):
    for k in range(3):
        axis = ("twist", "swing1", "swing2")[k]
        col = names.index(f"{side}Thigh.{axis}")
        j = IDX[f"{side}Thigh"]
        print(f"{side}Thigh {axis}: rede {amp(r['rede'][:, :, col]):5.1f} clipe {amp(r['clipe'][:, :, col]):5.1f} "
              f"corpo {amp(r['corpo'][:, :, j, k]):5.1f} graus | motor no limite {100 * r['sat'][:, :, j, k].mean():4.1f}% "
              f"batente {100 * r['bat'][:, :, j, k].mean():4.1f}%", flush=True)

# As velocidades-alvo (vel.*) das coxas no eixo da passada: a da rede, a do clipe e a do corpo (a derivada da posicao), em graus/s.
dt = 1.0 / 30.0
for side in ("Left", "Right"):
    col = names.index(f"vel.{side}Thigh.swing1")
    j = IDX[f"{side}Thigh"]
    body_vel = np.gradient(r["corpo"][:, :, j, 1], axis=0) / float(amb.batch("lev_corrida_0", 1, 1).policy_dt)
    net, clip = r["rede"][:, :, col], r["clipe"][:, :, col]
    corr = lambda x, y: float(np.corrcoef(x.ravel(), y.ravel())[0, 1])  # noqa: E731
    print(f"vel.{side}Thigh.swing1: rede p5/p95 {np.degrees(np.percentile(net, 5)):6.0f}/{np.degrees(np.percentile(net, 95)):5.0f} "
          f"clipe {np.degrees(np.percentile(clip, 5)):6.0f}/{np.degrees(np.percentile(clip, 95)):5.0f} "
          f"corpo {np.degrees(np.percentile(body_vel, 5)):6.0f}/{np.degrees(np.percentile(body_vel, 95)):5.0f} graus/s | "
          f"correlacao rede x corpo {corr(net, body_vel):+.2f}, clipe x corpo {corr(clip, body_vel):+.2f}", flush=True)

for side in ("Left", "Right"):
    j = IDX[f"{side}Thigh"]
    tau = r["tau"][:, :, j]
    print(f"{side}Thigh torque |motor| p50/p95/max por eixo: " + " | ".join(
        f"{np.percentile(np.abs(tau[..., k]), 50):.0f}/{np.percentile(np.abs(tau[..., k]), 95):.0f}/{np.abs(tau[..., k]).max():.0f} (limite {tau[..., 3 + k].max():.0f})"
        for k in range(3)) + f" | norma p95 {np.percentile(np.linalg.norm(tau[..., :3], axis=-1), 95):.0f}", flush=True)

# A cadencia: os ciclos por segundo da coxa (cruzamentos da media, subindo) no corpo e no alvo do clipe - com a fase presa ao passo
# do corpo (--sincronizar-fase), o relogio do clipe salta para o pouso do corpo e a cadencia mais alta nao e cobrada no tempo.
step_dt = float(amb.batch("lev_corrida_0", 1, 1).policy_dt)
def cadence(x):
    x = x - x.mean(0)
    up = ((x[:-1] < 0) & (x[1:] >= 0)).sum(0)
    return float(up.mean() / (x.shape[0] * step_dt))
for side in ("Left", "Right"):
    col = names.index(f"{side}Thigh.swing1"); j = IDX[f"{side}Thigh"]
    print(f"{side}Thigh ciclos/s: corpo {cadence(r['corpo'][:, :, j, 1]):.2f} clipe {cadence(r['clipe'][:, :, col]):.2f}", flush=True)

# O alcance do pe (m, a frente da pelve no rumo do deslocamento): p95 (o pouso mais a frente) e p5 (a saida mais atras).
for label in ("pe_corpo", "pe_clipe"):
    x = r[label]  # (tempo, mundo, [pelve, pe E, pe D], xy)
    out = []
    for w in range(x.shape[1]):
        pel = x[:, w, 0]; d = pel[-1] - pel[0]; fwd = d / (np.linalg.norm(d) + 1e-9)
        for f in (1, 2):
            rel = (x[:, w, f] - pel) @ fwd
            out.append((np.percentile(rel, 95), np.percentile(rel, 5)))
    out = np.array(out)
    print(f"{label}: a frente p95 {out[:, 0].mean():+.2f} m, atras p5 {out[:, 1].mean():+.2f} m, alcance {(out[:, 0] - out[:, 1]).mean():.2f} m", flush=True)

# As pontas da coxa no eixo da passada (graus, p5/p95): onde o corpo fica curto - na flexao (pouso) ou na extensao (saida).
pct = lambda x: f"{np.degrees(np.percentile(x, 5)):+6.1f}/{np.degrees(np.percentile(x, 95)):+6.1f}"  # noqa: E731
for side in ("Left", "Right"):
    col = names.index(f"{side}Thigh.swing1"); j = IDX[f"{side}Thigh"]
    print(f"{side}Thigh.swing1 p5/p95: corpo {pct(r['corpo'][:, :, j, 1])} clipe {pct(r['clipe'][:, :, col])} rede {pct(r['rede'][:, :, col])}", flush=True)
