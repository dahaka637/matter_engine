"""(14/11 v1, ele: "fazer o ROLAR para virar de bruços no levantar de costas... adaptar o proprio levantar de costas correndo, pois
ele ja faz o movimento em si") O levantar de costas que rola: o comeco do levantar_correndo_costas (de costas, gira de lado com a mao
no chao e fica de joelhos virado para o chao) emendado no levantar_brucos (de joelhos ate de pe).

A emenda: o par de quadros (rolar em [A0, A1] s, brucos em [B0, B1] s) de pose mais parecida (juntas, inclinacao e altura da raiz); o
brucos dali em diante girado no rumo e deslocado no plano para continuar o rolar (a altura e a dele, ja apoiada no chao), e uma mistura
de MISTURA s na emenda.

Uso: python3 tools/animation/clips/emendar_rolar_brucos.py [saida.matteranim.json] [clipe do fim: levantar_costas | levantar_brucos]

(05h40) O fim pelo levantar_costas casa melhor (custo 0,61 contra 1,19: o correndo a 0,80 s com o de costas a 1,13 s, juntas 0,46 rad,
inclinacao 3,6 graus, altura 3 cm) e e a pose ajoelhada com a mao no chao que ele marcou como a correta.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from matter_clip import Clip, load_clip, save_clip  # noqa: E402
from matter_rig import Rig, qangle_degrees, qfrom_axis_angle, qmul, qnormalize, qrotate  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
CLIPS = ROOT / "assets/animations/clips/levantar"
PROFILE = ROOT / "assets/characters/football_player/FootballPlayerV1.ragdoll.json"
A_RANGE, B_RANGE = (0.55, 1.15), (0.3, 1.6)
MISTURA = 0.2
Z = np.array([0.0, 0.0, 1.0])


def heading_of(rotation: np.ndarray) -> float:
    """O rumo da raiz estavel deitado de brucos e de costas (o do runtime, como no stand_up_kimodo.py)."""
    forward = qrotate(rotation, np.array([1.0, 0.0, 0.0]))
    up = qrotate(rotation, Z)
    planar = forward[:2] - up[:2] * forward[2]
    return math.atan2(planar[1], planar[0])


def tilt(rotation: np.ndarray) -> np.ndarray:
    return qnormalize(qmul(qfrom_axis_angle(Z, -heading_of(rotation)), rotation))


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else CLIPS / "levantar_costas_rola.matteranim.json"
    rig = Rig(PROFILE)
    fim = sys.argv[2] if len(sys.argv) > 2 else "levantar_costas"
    # (06h50) O comeco e as janelas da emenda tambem por argumento: comeco a0 a1 b0 b1 (o rolar do Kimodo + o brucos da Unreal).
    comeco = sys.argv[3] if len(sys.argv) > 3 else "levantar_correndo_costas"
    a_range = (float(sys.argv[4]), float(sys.argv[5])) if len(sys.argv) > 5 else A_RANGE
    b_range = (float(sys.argv[6]), float(sys.argv[7])) if len(sys.argv) > 7 else B_RANGE
    b = load_clip(CLIPS / f"{fim}.matteranim.json", rig)
    a = load_clip(CLIPS / f"{comeco}.matteranim.json", rig)
    # (07h45, ROLAR_VEL=k) O comeco k vezes mais rapido (reamostrado a 30 Hz): o rolar do Kimodo e lento para o jogo.
    import os
    k = float(os.environ.get("ROLAR_VEL", "1.0"))
    if abs(k - 1.0) > 1e-6:
        ts = np.arange(0.0, a.duration / k + 1e-6, 1.0 / a.sample_rate)
        samples = [a.sample(float(t * k), loop=False) for t in ts]
        a.times, a.duration = ts, float(ts[-1])
        a.root_offsets = np.array([s_[0] for s_ in samples])
        a.root_rotations = np.array([s_[1] for s_ in samples])
        a.coordinates = np.array([s_[2] for s_ in samples])
        for name in ("left_foot_contact", "right_foot_contact"):
            v = getattr(a, name)
            if v is not None:
                setattr(a, name, np.interp(ts * k, np.linspace(0.0, ts[-1] * k, len(v)), v))
    best = None
    for i, ta in enumerate(a.times):
        if not a_range[0] <= ta <= a_range[1]:
            continue
        for j, tb in enumerate(b.times):
            if not b_range[0] <= tb <= b_range[1]:
                continue
            joints = float(np.sqrt(((a.coordinates[i] - b.coordinates[j]) ** 2).sum(-1).mean()))
            lean = qangle_degrees(tilt(a.root_rotations[i]), tilt(b.root_rotations[j])) / 57.3
            height = abs(a.root_offsets[i, 2] - b.root_offsets[j, 2])
            cost = joints + lean + 3.0 * height
            if best is None or cost < best[0]:
                best = (cost, i, j, joints, lean * 57.3, height)
    cost, i, j, joints, lean, height = best
    print(f"emenda: rolar {a.times[i]:.2f} s -> brucos {b.times[j]:.2f} s (juntas {joints:.3f} rad, inclinacao {lean:.1f} graus, "
          f"altura {100 * height:.1f} cm)")
    # O brucos de j em diante no rumo e na posicao do rolar em i.
    turn = qfrom_axis_angle(Z, heading_of(a.root_rotations[i]) - heading_of(b.root_rotations[j]))
    rot_b = np.array([qnormalize(qmul(turn, q)) for q in b.root_rotations[j:]])
    off_b = np.array([qrotate(turn, p - b.root_offsets[j]) for p in b.root_offsets[j:]])
    off_b[:, :2] += a.root_offsets[i, :2]
    off_b[:, 2] = b.root_offsets[j:, 2]
    coords_b = b.coordinates[j:].copy()
    dt = 1.0 / a.sample_rate
    n = len(coords_b)
    # A mistura: nos primeiros quadros do brucos, o que o rolar seguiria fazendo (se tiver quadros) puxado para o brucos.
    blend = max(1, int(round(MISTURA / dt)))
    for k in range(min(blend, n, len(a.times) - 1 - i)):
        w = 0.5 - 0.5 * math.cos(math.pi * (k + 1) / (blend + 1))
        ia = i + 1 + k
        coords_b[k] = (1 - w) * a.coordinates[ia] + w * coords_b[k]
        qa = a.root_rotations[ia]
        qb = rot_b[k] if np.dot(qa, rot_b[k]) >= 0 else -rot_b[k]
        rot_b[k] = qnormalize((1 - w) * qa + w * qb)
        off_b[k] = (1 - w) * a.root_offsets[ia] + w * off_b[k]
    times = np.concatenate([a.times[:i + 1], a.times[i] + dt * np.arange(1, n + 1)])
    coords = np.concatenate([a.coordinates[:i + 1], coords_b])
    offsets = np.concatenate([a.root_offsets[:i + 1], off_b])
    rotations = np.concatenate([a.root_rotations[:i + 1], rot_b])

    def contact(ca, cb):
        if ca is None or cb is None:
            return None
        return np.concatenate([ca[:i + 1], cb[j:]])

    step = 0.0
    for f in range(1, len(times)):
        step = max(step, float(np.degrees(np.abs(coords[f] - coords[f - 1]).max())))
        step = max(step, qangle_degrees(rotations[f], rotations[f - 1]))
    clip = Clip(id="levantar_costas_rola", display_name=f"Levantar de costas (o comeco do {comeco} + {fim})",
                target_rig=a.target_rig, duration=float(times[-1]), sample_rate=a.sample_rate, loops=False, times=times,
                root_offsets=offsets, root_rotations=rotations, coordinates=coords,
                left_foot_contact=contact(a.left_foot_contact, b.left_foot_contact),
                right_foot_contact=contact(a.right_foot_contact, b.right_foot_contact),
                nominal_speed=0.0, source=f"{comeco} + {fim} (emendar_rolar_brucos.py)")
    deliver_b = {"levantar_brucos": 66 / 30, "levantar_costas": 60 / 30}[fim]  # LEVANTAR_UNREAL_DELIVER (a cabeca a 95% da final)
    report = {"passed": step < 45.0, "maximumJointStepDegrees": step, "limitHitCount": 0,
              "emenda": {"rolar_s": float(a.times[i]), "brucos_s": float(b.times[j]), "juntas_rad": joints, "inclinacao_graus": lean,
                         "altura_cm": 100 * height, "mistura_s": MISTURA},
              "entrega_s": float(a.times[i] + (deliver_b - b.times[j]))}
    save_clip(clip, rig, out, report)
    print(f"{out}: {clip.duration:.2f} s, passo maximo {step:.1f} graus, entrega {report['entrega_s']:.2f} s")


if __name__ == "__main__":
    main()
