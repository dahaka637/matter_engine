"""A corrida em linha reta, sem nada para os lados (regras do usuario, 24/10): por amostra do Kimodo (npz, SOMA77), entre
--de e --ate segundos, no referencial do caminho (a frente e o rumo do quadril do inicio ao fim):
  giro do quadril e dos ombros (graus em volta da vertical: a faixa p5-p95, que e o quanto vira a cada passo);
  a coluna (o tronco, quadril -> pescoco, graus para a frente, e a curvatura: o angulo entre quadril -> peito e
  peito -> cabeca; reta e perto de 0);
  para os lados (cm, o pior para fora/dentro): a mao e o cotovelo em relacao ao proprio ombro, o joelho e o pe em
  relacao ao proprio quadril; e o quanto o caminho do quadril sai da reta.

uso: linha.py [--de 0.25] [--ate 1.8] amostra.npz [...]
"""
import sys

import numpy as np

from soma import IX

FPS = 30.0


def unit(v):
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-9)


def yaw_range(vectors, lateral, forward):
    # o angulo de cada linha (esquerda -> direita) em volta da vertical, em relacao a lateral do caminho
    a = np.degrees(np.arctan2(vectors @ forward, vectors @ lateral))
    return float(np.percentile(a, 95) - np.percentile(a, 5)), float(np.mean(a))


def measure(path, start, end):
    P = np.load(path)["posed_joints"]
    P = P[int(start * FPS):int(end * FPS) + 1]
    up = np.array([0.0, 1.0, 0.0])
    hips = P[:, IX["Hips"]]
    travel = hips[-1] - hips[0]
    travel[1] = 0
    forward = unit(travel)
    lateral = np.cross(up, forward)  # a esquerda
    pelvis = yaw_range(P[:, IX["LeftLeg"]] - P[:, IX["RightLeg"]], lateral, forward)
    shoulders = yaw_range(P[:, IX["LeftArm"]] - P[:, IX["RightArm"]], lateral, forward)
    trunk = P[:, IX["Neck1"]] - hips
    lean = np.degrees(np.arctan2(trunk @ forward, trunk @ up))
    a, b = unit(P[:, IX["Chest"]] - hips), unit(P[:, IX["Head"]] - P[:, IX["Chest"]])
    bend = np.degrees(np.arccos(np.clip(np.sum(a * b, 1), -1, 1)))
    side = {}
    for s, sign in (("Left", 1.0), ("Right", -1.0)):
        for joint, base in (("Hand", "Arm"), ("ForeArm", "Arm"), ("Shin", "Leg"), ("Foot", "Leg")):
            off = ((P[:, IX[s + joint]] - P[:, IX[s + base]]) @ lateral) * sign * 100  # + para fora
            key = joint
            side[key] = max(side.get(key, 0.0), float(np.max(np.abs(off))))
    drift = float(np.max(np.abs((hips - hips[0]) @ lateral)) * 100)
    return pelvis[0], shoulders[0], float(np.mean(lean)), float(np.percentile(lean, 95) - np.percentile(lean, 5)), \
        float(np.mean(bend)), float(np.max(bend)), side, drift


def main():
    args = sys.argv[1:]
    start, end = 0.25, 1.8
    for key in ("--de", "--ate"):
        if key in args:
            i = args.index(key)
            if key == "--de":
                start = float(args[i + 1])
            else:
                end = float(args[i + 1])
            del args[i:i + 2]
    print(f"{'amostra':22s} {'giro quadril':>12s} {'giro ombros':>11s} {'tronco (faixa)':>15s} {'curva med/max':>13s}"
          f" | para os lados cm: {'mao':>4s} {'cotov':>5s} {'joelho':>6s} {'pe':>4s} | {'desvio':>6s}")
    for path in args:
        pelvis, shoulders, lean, lean_range, bend, bend_max, side, drift = measure(path, start, end)
        name = path.rsplit("/", 1)[-1].replace(".npz", "")
        print(f"{name:22s} {pelvis:10.1f}   {shoulders:9.1f}   {lean:6.1f} ({lean_range:4.1f})  {bend:5.1f} / {bend_max:4.1f}"
              f" |                  {side['Hand']:4.0f} {side['ForeArm']:5.0f} {side['Shin']:6.0f} {side['Foot']:4.0f} | {drift:5.0f}")


if __name__ == "__main__":
    main()
