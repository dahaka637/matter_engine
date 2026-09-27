"""Gera os clipes autorados a partir de tools/animation/clips/*.py.

Para cada definicao: amostra, valida todos os quadros, grava o
.matteranim.json em assets/animations/clips (so se passar) e renderiza uma
folha de previa em PNG para revisao.

Uso:
  python3 tools/animation/build_clips.py              # todos
  python3 tools/animation/build_clips.py natural_idle # um
  python3 tools/animation/build_clips.py --preview-dir /tmp/previas

Sai com codigo 1 se algum clipe reprovar - nada reprovado e gravado.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from authoring import build_clip, write_clip  # noqa: E402
from matter_rig import load_default_character  # noqa: E402
from preview import STANDARD_VIEWS, contact_sheet  # noqa: E402

CLIP_SOURCES = HERE / "clips"


def load_definition_module(path: Path):
    spec = importlib.util.spec_from_file_location(f"clip_{path.stem}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("clips", nargs="*", help="ids (padrao: todos)")
    parser.add_argument("--preview-dir", type=Path, default=None,
                        help="onde gravar as folhas de previa")
    parser.add_argument("--preview-frames", type=int, default=4)
    args = parser.parse_args()

    rig, skin = load_default_character()
    sources = sorted(CLIP_SOURCES.glob("*.py"))
    if args.clips:
        sources = [p for p in sources if p.stem in args.clips]
    failed = False
    definitions = []
    for source in sources:
        module = load_definition_module(source)
        # Um arquivo descreve um clipe (definition) ou uma familia deles
        # (definitions: a mesma passada em varias direcoes).
        if hasattr(module, "definitions"):
            definitions.extend(module.definitions(rig))
        else:
            definitions.append(module.definition(rig))
    for definition in definitions:
        result = build_clip(rig, definition)
        report = result.report
        status = "APROVADO" if report["passed"] else "REPROVADO"
        print(f"{definition.id}: {status} - {report['frames']} quadros, "
              f"junta max {report['maximumJointRateRadiansPerSecond']} rad/s, "
              f"loop {report['loopClosureDegrees']} graus, "
              f"chao {report['groundErrorMillimeters']} mm, "
              f"deslize {report['plantedFootSlideMillimeters']} mm")
        for tight in report["tightestSelfClearance"]:
            print(f"    folga {tight['pair']}: {tight['gapMillimeters']} mm")
        for problem in report["problems"]:
            print(f"    ! {problem}")
        if report["passed"]:
            path = write_clip(rig, result)
            print(f"    gravado: {path.relative_to(HERE.parents[1])}")
        else:
            failed = True
        if args.preview_dir is not None:
            args.preview_dir.mkdir(parents=True, exist_ok=True)
            step = max(1, len(result.frames) // args.preview_frames)
            picks = result.frames[::step][:args.preview_frames]
            sheet = contact_sheet(rig, skin, [(f"t={t:.2f}s", p, o)
                                              for t, p, o in picks],
                                  views=STANDARD_VIEWS,
                                  floor_z=definition.floor_z)
            out = args.preview_dir / f"{definition.id}.png"
            sheet.save(out)
            print(f"    previa: {out}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
