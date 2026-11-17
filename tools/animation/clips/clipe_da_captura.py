"""(14/11 v1, ele: "observe no checkpoint 72889 antigo.. o levantar de costas que ele tinha, que era um rolar melhor, em que o unico
problema era o fato da mao direita ficar enroscada atras por um periodo.. e o saltinho que ele dava no final") Um clipe a partir de
uma captura da fisica (tools/rl/inspecao/captura.py: o centro e o quaternio do colisor de cada elo por decisao): o corpo de cada
elo pelo colisor do perfil (como o frameLines do C++), as coordenadas das juntas pelo inverso da junta do Rig e a raiz pela pelve.

Uso: python3 tools/animation/clips/clipe_da_captura.py CAPTURA.npz MUNDO T0 T1 SAIDA.matteranim.json ID "NOME"
A ida e volta (as coordenadas pela cinematica direta de volta aos colisores) e conferida e impressa: o erro tem de ser de mm.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from matter_clip import Clip, save_clip  # noqa: E402
from matter_rig import Rig, qconj, qmul, qnormalize, qrotate  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / "assets/characters/football_player/FootballPlayerV1.ragdoll.json"


def collider_offsets(profile: Path):
    """O posicionamento local do colisor como o motor usa (o da casca e o dela, nao o da caixa que a envolve no Rig)."""
    out = []
    for link in json.loads(profile.read_text())["links"]:
        shape = link.get("box") or link.get("capsule") or link.get("hull")
        out.append((np.array(shape.get("position", [0, 0, 0]), float), np.array(shape.get("orientation", [0, 0, 0, 1]), float)))
    return out


def link_poses(colliders: np.ndarray, offsets) -> tuple[np.ndarray, np.ndarray]:
    """(posicoes, orientacoes) dos elos a partir dos colisores [N, 7] (xyz + xyzw)."""
    pos, rot = np.zeros((len(offsets), 3)), np.zeros((len(offsets), 4))
    for i, (lp, lq) in enumerate(offsets):
        q = qnormalize(qmul(qnormalize(colliders[i, 3:]), qconj(lq)))
        rot[i] = q
        pos[i] = colliders[i, :3] - qrotate(q, lp)
    return pos, rot


def main() -> None:
    npz, world, t0, t1, out, clip_id, display = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), \
        Path(sys.argv[5]), sys.argv[6], sys.argv[7]
    rig = Rig(PROFILE)
    offsets = collider_offsets(PROFILE)
    data = np.load(npz)
    meta = json.loads(Path(npz).with_suffix(".json").read_text())
    names = meta["elos"]
    assert names == [lk.id for lk in rig.links], "a ordem dos elos da captura difere do perfil"
    dt = float(meta["dt"])
    body = data["corpo"][world].astype(float)
    episode = data["episodio"][world]
    steps = [i for i in range(body.shape[0]) if t0 <= (i + 1) * dt <= t1 + 1e-6 and episode[i] == episode[0]
             and np.isfinite(body[i]).all()]
    root = rig.links[0]
    origin = None
    offs, rots, coords = [], [], []
    worst = 0.0
    for i in steps:
        pos, rot = link_poses(body[i], offsets)
        if origin is None:
            origin = np.array([pos[0, 0], pos[0, 1], 0.0])
        c = np.zeros((len(rig.links), 3))
        for k in range(1, len(rig.links)):
            c[k] = rig.coordinates_for(k, rot[rig.links[k].parent], rot[k])
        delta = qnormalize(qmul(qconj(root.model_orientation), rot[0]))
        off = pos[0] - origin - root.model_position
        offs.append(off)
        rots.append(delta)
        coords.append(c)
        # (A ida e volta: a cinematica direta destas coordenadas contra os elos capturados.)
        fk_pos, _ = rig.forward_kinematics(off, delta, c)
        worst = max(worst, float(np.abs(fk_pos - (pos - origin)).max()))
    print(f"{len(steps)} quadros de {t0:.2f} a {t1:.2f} s; ida e volta: erro maximo {1000 * worst:.1f} mm")
    rots = np.array(rots)
    for k in range(1, len(rots)):  # (o mesmo hemisferio quadro a quadro, para a interpolacao)
        if np.dot(rots[k], rots[k - 1]) < 0:
            rots[k] = -rots[k]
    times = np.arange(len(steps)) * dt
    coords = np.array(coords)
    step = float(np.degrees(np.abs(np.diff(coords, axis=0)).max()))
    # (Fora dos limites do perfil por mais de 1 grau: a fisica respeita os batentes, entao isto e erro da conversao.)
    hits = sum(int((np.abs(coords[:, k] - np.array([rig.clamp(k, c) for c in coords[:, k]])) > np.radians(1.0)).any(-1).sum())
               for k in range(1, len(rig.links)))
    print(f"passo maximo {step:.1f} graus/quadro; quadros-junta fora dos limites (>1 grau): {hits}")
    clip = Clip(id=clip_id, display_name=display, target_rig=rig.id, duration=float(times[-1]), sample_rate=1.0 / dt, loops=False,
                times=times, root_offsets=np.array(offs), root_rotations=rots, coordinates=coords,
                source=f"captura {Path(npz).name} mundo {world} ({meta.get('checkpoint')}, atualizacao {meta.get('atualizacao')})")
    save_clip(clip, rig, out, {"passed": hits == 0, "maximumJointStepDegrees": step, "limitHitCount": hits, "captura": str(npz), "mundo": world, "ida_e_volta_mm": 1000 * worst})
    print(f"{out}: {clip.duration:.2f} s")


if __name__ == "__main__":
    main()
