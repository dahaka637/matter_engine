#!/usr/bin/env python3
"""Espelha um clipe (esquerda <-> direita) no plano do corpo: cada osso vai para
o do outro lado com a orientacao de mundo refletida (strafe.mirror_pose, a
mesma cinematica do motor), os contatos dos pes trocam e a direcao de
deslocamento inverte. O resto do arquivo (a politica de raiz, o relatorio do
retarget) fica como no original.

  python3 tools/animation/espelhar.py walk_strafe_right walk_strafe_left \\
      --name "Andar de lado para a esquerda (Mixamo, espelhado)"
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "clips"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from matter_clip import load_clip  # noqa: E402
from matter_rig import REPO_ROOT, load_default_character, qnormalize  # noqa: E402
from strafe import mirror_pose  # noqa: E402

CLIPS = REPO_ROOT / "assets" / "animations" / "clips"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("origem", help="o id do clipe de origem (em assets/animations/clips)")
    parser.add_argument("destino", help="o id do clipe espelhado")
    parser.add_argument("--name", required=True)
    args = parser.parse_args()
    rig, _ = load_default_character()
    source = CLIPS / f"{args.origem}.matteranim.json"
    data = json.loads(source.read_text(encoding="utf-8"))
    clip = load_clip(source, rig)
    out = copy.deepcopy(data)
    out["id"] = args.destino
    out["displayName"] = args.name
    out["sourceAssetPath"] = f"{data.get('sourceAssetPath', '')} (espelhado de {args.origem})"
    tracks = {track["targetLinkId"]: track for track in out["tracks"]}
    root_id = rig.links[0].id
    for f in range(len(clip.times)):
        offset, rotation, coordinates = mirror_pose(rig, clip.root_offsets[f], clip.root_rotations[f], clip.coordinates[f])
        root_key = tracks[root_id]["keyframes"][f]
        root_key["rootTranslationOffsetMeters"] = [round(float(v), 7) for v in offset]
        root_key["rootRotationDelta"] = [round(float(v), 7) for v in qnormalize(rotation)]
        for i, link in enumerate(rig.links):
            if i == 0 or link.id not in tracks:
                continue
            tracks[link.id]["keyframes"][f]["jointPositionRadians"] = [round(float(v), 7) for v in coordinates[i]]
    contacts = out.get("contacts") or {}
    if "leftFoot" in contacts or "rightFoot" in contacts:
        out["contacts"] = {"leftFoot": contacts.get("rightFoot"), "rightFoot": contacts.get("leftFoot")}
    if isinstance(out.get("sourceRootDisplacementMeters"), list) and len(out["sourceRootDisplacementMeters"]) == 3:
        x, y, z = out["sourceRootDisplacementMeters"]
        out["sourceRootDisplacementMeters"] = [x, -y, z]
    if out.get("travelDirectionDegrees") is not None:
        out["travelDirectionDegrees"] = -float(out["travelDirectionDegrees"])
    report = out.setdefault("retargetReport", {})
    report["mirroredFrom"] = args.origem
    target = CLIPS / f"{args.destino}.matteranim.json"
    target.write_text(json.dumps(out, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"espelhado: {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
