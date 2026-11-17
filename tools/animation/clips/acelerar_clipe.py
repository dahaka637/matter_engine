"""(14/11 15h45, ele: "depois... pode começar a trabalhar já para deixar a velocidade do levantar em 1.4 - 1.5") Um clipe mais rapido, os
mesmos gestos: reamostrado a 30 Hz com o tempo dividido por K (como o ROLAR_VEL do emendar_rolar_brucos.py).

Uso: python3 tools/animation/clips/acelerar_clipe.py ORIGEM.matteranim.json SAIDA.matteranim.json K ID "NOME"
"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from matter_clip import Clip, load_clip, save_clip  # noqa: E402
from matter_rig import Rig  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / "assets/characters/football_player/FootballPlayerV1.ragdoll.json"


def main() -> None:
    src, out, k, clip_id, display = sys.argv[1], Path(sys.argv[2]), float(sys.argv[3]), sys.argv[4], sys.argv[5]
    rig = Rig(PROFILE)
    a = load_clip(src, rig)
    ts = np.arange(0.0, a.duration / k + 1e-6, 1.0 / a.sample_rate)
    samples = [a.sample(float(t * k), loop=False) for t in ts]
    contact = {}
    for name in ("left_foot_contact", "right_foot_contact"):
        v = getattr(a, name)
        contact[name] = None if v is None else np.interp(ts * k, a.times, v)
    rot = np.array([s[1] for s in samples])
    for i in range(1, len(rot)):
        if np.dot(rot[i], rot[i - 1]) < 0:
            rot[i] = -rot[i]
    clip = Clip(id=clip_id, display_name=display, target_rig=a.target_rig, duration=float(ts[-1]), sample_rate=a.sample_rate, loops=False,
                times=ts, root_offsets=np.array([s[0] for s in samples]), root_rotations=rot, coordinates=np.array([s[2] for s in samples]),
                left_foot_contact=contact["left_foot_contact"], right_foot_contact=contact["right_foot_contact"], nominal_speed=0.0,
                source=f"{Path(src).name} acelerado {k:.3f}x (acelerar_clipe.py)")
    step = float(np.degrees(np.abs(np.diff(clip.coordinates, axis=0)).max()))
    save_clip(clip, rig, out, {"passed": True, "maximumJointStepDegrees": step, "limitHitCount": 0, "acelerado": k, "origem": str(src)})
    print(f"{out}: {clip.duration:.2f} s (era {a.duration:.2f} s), passo maximo {step:.1f} graus/quadro")


if __name__ == "__main__":
    main()
