"""Mede corridas do Kimodo (npz, esqueleto SOMA77, Y para cima) a partir de --de segundos.

Por amostra: velocidade (m/s), inclinacao do tronco (quadril -> pescoco, graus para a frente: media e faixa),
cabeca (pescoco -> cabeca), cadencia (passos/s), voo (fracao sem pe no chao), passo (m), joelho (a coxa
mais a frente e mais atras, graus da vertical), dobra do joelho no balanco, bracos (amplitude do braco a
frente/atras e o cotovelo medio), chao (fracao com o pe a 3 cm do chao), deslize do pe nesse apoio (m/s),
a regularidade dos passos (desvio/media) e o pico de velocidade (janelas de 0,5 s).

uso: corrida.py [--de 1.0] [--pico] arquivo.npz [...]
  --pico: so a janela de 0,7 s (uma passada) mais rapida depois de --de (de onde sai o ciclo do jogo).
"""
import sys

import numpy as np

from soma import IX

FPS = 30.0


def angle_from(vectors, axis, reference):
    """Angulo (graus) de cada vetor em relacao a `reference`, positivo para `axis` (no plano dos dois)."""
    return np.degrees(np.arctan2(vectors @ axis, vectors @ reference))


def onsets(contact):
    return np.flatnonzero(contact[1:] & ~contact[:-1]) + 1


def measure(path, start, peak=False):
    data = np.load(path)
    P = data["posed_joints"]
    k0 = int(start * FPS)
    k1 = len(P)
    if peak:
        window = int(0.7 * FPS)
        hips = P[:, IX["Hips"]][:, [0, 2]]
        speeds = [np.linalg.norm(hips[k + window] - hips[k]) for k in range(k0, len(P) - window)]
        k0 = k0 + int(np.argmax(speeds))
        k1 = k0 + window + 1
    P = P[k0:k1]
    up = np.array([0.0, 1.0, 0.0])
    hips = P[:, IX["Hips"]]
    travel = hips[-1] - hips[0]
    travel[1] = 0.0
    seconds = (len(P) - 1) / FPS
    speed = np.linalg.norm(travel) / seconds
    fwd = travel / max(np.linalg.norm(travel), 1e-9)
    side = np.cross(up, fwd)
    lateral = (hips - hips[0]) @ side
    trunk = angle_from(P[:, IX["Neck1"]] - hips, fwd, up)
    head = angle_from(P[:, IX["Head"]] - P[:, IX["Neck1"]], fwd, up)
    # o apoio pela geometria (o rotulo de contato do modelo some nas corridas rapidas): o pe a 3 cm do chao
    # (o chao: o ponto mais baixo do pe no clipe inteiro)
    def planted(side):
        whole = np.minimum(data["posed_joints"][:, IX[f"{side}ToeBase"], 1], data["posed_joints"][:, IX[f"{side}Foot"], 1])
        return whole[k0:k1] < np.percentile(whole, 2) + 0.03
    left, right = planted("Left"), planted("Right")
    flight = float(np.mean(~left & ~right))
    steps = np.sort(np.concatenate([onsets(left), onsets(right)]))
    # a cadencia pela mediana do intervalo entre pisadas no clipe inteiro depois de --de (a janela do pico e curta)
    def planted_after(side):
        whole = np.minimum(data["posed_joints"][:, IX[f"{side}ToeBase"], 1], data["posed_joints"][:, IX[f"{side}Foot"], 1])
        return whole[int(start * FPS):] < np.percentile(whole, 2) + 0.03
    all_steps = np.sort(np.concatenate([onsets(planted_after("Left")), onsets(planted_after("Right"))]))
    cadence = FPS / float(np.median(np.diff(all_steps))) if len(all_steps) > 2 else len(steps) / seconds
    intervals = np.diff(steps) / FPS
    regular = float(np.std(intervals) / np.mean(intervals)) if len(intervals) > 2 else float("nan")
    thigh, knee_bend, arm, elbow, slide, ground = [], [], [], [], [], []
    for s, contact in (("Left", left), ("Right", right)):
        t = P[:, IX[f"{s}Shin"]] - P[:, IX[f"{s}Leg"]]
        shin = P[:, IX[f"{s}Foot"]] - P[:, IX[f"{s}Shin"]]
        thigh.append(angle_from(t, fwd, -up))
        cos = np.sum(t * shin, 1) / (np.linalg.norm(t, axis=1) * np.linalg.norm(shin, axis=1))
        knee_bend.append(np.degrees(np.arccos(np.clip(cos, -1, 1))))
        u = P[:, IX[f"{s}ForeArm"]] - P[:, IX[f"{s}Arm"]]
        f = P[:, IX[f"{s}Hand"]] - P[:, IX[f"{s}ForeArm"]]
        arm.append(angle_from(u, fwd, -up))
        cos = np.sum(u * f, 1) / (np.linalg.norm(u, axis=1) * np.linalg.norm(f, axis=1))
        elbow.append(180.0 - np.degrees(np.arccos(np.clip(cos, -1, 1))))
        toe = P[:, IX[f"{s}ToeBase"]]
        on = contact
        ground.append(on.mean())
        v = np.linalg.norm(np.diff(toe[:, [0, 2]], axis=0), axis=1) * FPS
        planted = on[1:] & on[:-1]
        if planted.any():
            slide.append(v[planted])
    thigh = np.concatenate(thigh)
    arm = np.concatenate(arm)
    slide = np.concatenate(slide) if slide else np.zeros(1)
    return {
        "vel": speed, "tronco": float(np.mean(trunk)), "tronco_min": float(np.percentile(trunk, 5)),
        "tronco_max": float(np.percentile(trunk, 95)), "cabeca": float(np.mean(head)),
        "cadencia": cadence, "voo": flight, "passo": speed / cadence if cadence > 0 else float("nan"),
        "coxa_frente": float(np.percentile(thigh, 98)), "coxa_tras": float(np.percentile(thigh, 2)),
        "joelho_dobra": float(np.percentile(np.concatenate(knee_bend), 98)),
        "braco_frente": float(np.percentile(arm, 98)), "braco_tras": float(np.percentile(arm, 2)),
        "cotovelo": float(np.mean(np.concatenate(elbow))), "deslize": float(np.percentile(slide, 90)),
        "regular": regular, "desvio_lateral": float(np.max(np.abs(lateral))), "chao": float(np.mean(ground)),
        "pico": float(max(np.linalg.norm(hips[min(len(P) - 1, k + 15)][[0, 2]] - hips[k][[0, 2]]) * 2
                          for k in range(0, max(1, len(P) - 15), 5))), "de": k0 / FPS,
    }


def main():
    args = sys.argv[1:]
    start = 1.0
    peak = "--pico" in args
    args = [a for a in args if a != "--pico"]
    if "--de" in args:
        i = args.index("--de")
        start = float(args[i + 1])
        del args[i:i + 2]
    print(f"{'amostra':24s} {'vel':>5s} {'tronco (faixa)':>17s} {'cab':>4s} {'cad':>4s} {'voo':>4s} "
          f"{'passo':>5s} {'coxa +/-':>9s} {'joel':>4s} {'braco +/-':>10s} {'cotov':>5s} {'chao':>4s} {'desl':>5s} {'irreg':>5s} {'pico':>4s}")
    for path in args:
        m = measure(path, start, peak)
        name = path.rsplit("/", 1)[-1].replace(".npz", "") + (f"@{m['de']:.1f}" if peak else "")
        print(f"{name:24s} {m['vel']:5.2f} {m['tronco']:5.1f} ({m['tronco_min']:4.1f}-{m['tronco_max']:4.1f}) "
              f"{m['cabeca']:4.0f} {m['cadencia']:4.1f} {m['voo']:4.2f} {m['passo']:5.2f} "
              f"{m['coxa_frente']:4.0f}/{m['coxa_tras']:4.0f} {m['joelho_dobra']:4.0f} "
              f"{m['braco_frente']:4.0f}/{m['braco_tras']:4.0f} {m['cotovelo']:5.0f} {m['chao']:4.2f} {m['deslize']:5.2f} {m['regular']:5.2f} {m['pico']:4.1f}")


if __name__ == "__main__":
    main()
