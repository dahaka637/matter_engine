#!/usr/bin/env python3
"""Simetriza um clipe ciclico (05/11, ele: "o nosso exercicio de corrida esta com o problema do personagem torto"): cada
quadro vira a media dele com o espelho (strafe.mirror_pose, a mesma cinematica do motor) do quadro meio ciclo depois.
Assim o lado esquerdo faz exatamente o que o direito fez meio ciclo antes - o clipe deixa de pender para um lado - e o
espelho meio ciclo depois (o de tools/rl/espelho.py) passa a valer para ele. Os contatos dos pes tambem: o do esquerdo
vira a media dele com o do direito meio ciclo depois, e vice-versa. O original vai para --guardar.

  python3 tools/animation/simetrizar.py run_forward_medium --guardar build-profile/tmp/run_forward_medium.original.json
  python3 tools/animation/simetrizar.py run_forward_medium --medir      # so mede a assimetria, nao grava
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "clips"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from matter_clip import load_clip  # noqa: E402
from matter_rig import REPO_ROOT, load_default_character, qnormalize  # noqa: E402
from strafe import mirror_pose  # noqa: E402

CLIPS = REPO_ROOT / "assets" / "animations" / "clips"


def asymmetry(rig, clip) -> float:
    """O erro medio (rad) entre a junta no instante t e a do espelho do instante t + meio ciclo."""
    half = clip.duration / 2.0
    errs = []
    for t in clip.times[:-1]:
        a = clip.sample(float(t))
        b = mirror_pose(rig, *clip.sample(float(t) + half))
        errs.append(float(np.abs(np.asarray(a[2]) - np.asarray(b[2])).mean()))
    return float(np.mean(errs))


def contact_at(keys: list, t: float, duration: float) -> float:
    t = t % duration
    ts = [k["timeSeconds"] for k in keys]
    vs = [k["value"] for k in keys]
    return float(np.interp(t, ts, vs))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("clipe", help="o id do clipe (em assets/animations/clips), ciclico")
    ap.add_argument("--guardar", help="onde guardar o original")
    ap.add_argument("--medir", action="store_true")
    a = ap.parse_args()
    rig, _ = load_default_character()
    path = CLIPS / f"{a.clipe}.matteranim.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    clip = load_clip(path, rig)
    if not clip.loops:
        raise SystemExit("so clipes ciclicos")
    before = asymmetry(rig, clip)
    print(f"{a.clipe}: assimetria antes {before:.4f} rad")
    if a.medir:
        return 0
    if not a.guardar:
        raise SystemExit("--guardar: onde guardar o original")
    shutil.copy2(path, a.guardar)
    half = clip.duration / 2.0
    tracks = {track["targetLinkId"]: track for track in data["tracks"]}
    root_id = rig.links[0].id
    for f, t in enumerate(clip.times):
        o1, r1, c1 = clip.sample(float(t))
        o2, r2, c2 = mirror_pose(rig, *clip.sample(float(t) + half))
        r2 = np.asarray(r2, float)
        if float(np.dot(r1, r2)) < 0:
            r2 = -r2
        offset = (np.asarray(o1, float) + np.asarray(o2, float)) / 2.0
        rotation = qnormalize((np.asarray(r1, float) + r2) / 2.0)
        coords = (np.asarray(c1, float) + np.asarray(c2, float)) / 2.0
        root_key = tracks[root_id]["keyframes"][f]
        root_key["rootTranslationOffsetMeters"] = [round(float(v), 7) for v in offset]
        root_key["rootRotationDelta"] = [round(float(v), 7) for v in rotation]
        for i, link in enumerate(rig.links):
            if i == 0 or link.id not in tracks:
                continue
            n = len(tracks[link.id]["keyframes"][f]["jointPositionRadians"])
            tracks[link.id]["keyframes"][f]["jointPositionRadians"] = [round(float(v), 7) for v in coords[i][:n]]
    contacts = data.get("contacts") or {}
    if contacts.get("leftFoot") and contacts.get("rightFoot"):
        left, right = contacts["leftFoot"], contacts["rightFoot"]
        d = clip.duration
        new_left = [{"timeSeconds": k["timeSeconds"],
                     "value": round(0.5 * (k["value"] + contact_at(right, k["timeSeconds"] + half, d)), 6)} for k in left]
        new_right = [{"timeSeconds": k["timeSeconds"],
                      "value": round(0.5 * (k["value"] + contact_at(left, k["timeSeconds"] + half, d)), 6)} for k in right]
        data["contacts"] = {"leftFoot": new_left, "rightFoot": new_right}
    data["displayName"] = data.get("displayName", a.clipe) + " (simetrizado)"
    data["sourceAssetPath"] = f"{data.get('sourceAssetPath', '')} (simetrizado: media com o espelho meio ciclo depois)"
    if isinstance(data.get("sourceRootDisplacementMeters"), list) and len(data["sourceRootDisplacementMeters"]) == 3:
        data["sourceRootDisplacementMeters"][1] = 0.0
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    after = asymmetry(rig, load_clip(path, rig))
    print(f"{a.clipe}: assimetria depois {after:.4f} rad (original em {a.guardar})")
    assert after < 0.25 * before, "a simetrizacao nao reduziu a assimetria"
    return 0


if __name__ == "__main__":
    sys.exit(main())
