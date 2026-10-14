#!/usr/bin/env python3
"""Marca um clipe de giro parado como um ciclo que gira: grava no arquivo
quanto o corpo vira de um ciclo para o outro (turnCycleDegrees; positivo para a
esquerda, como no INVENTARIO.md). Na biblioteca do RL todo clipe toca em ciclo;
com o giro marcado, a referencia emenda o fim no comeco somando a volta, em vez
de ver um pulo de 90 ou 180 graus.

  python3 tools/animation/ciclo_de_giro.py giro_esquerda_180 180
"""

import json
import pathlib
import sys

CLIPS = pathlib.Path(__file__).resolve().parents[2] / "assets" / "animations" / "clips"


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    path = CLIPS / f"{sys.argv[1]}.matteranim.json"
    text = path.read_text()
    degrees = float(sys.argv[2])
    # So uma linha do arquivo muda (o resto fica como o importador gravou).
    lines = [line for line in text.split("\n") if not line.lstrip().startswith('"turnCycleDegrees"')]
    at = next(i for i, line in enumerate(lines) if line.lstrip().startswith('"loops"'))
    indent = lines[at][: len(lines[at]) - len(lines[at].lstrip())]
    if lines[at].rstrip().endswith(","):
        lines.insert(at + 1, f'{indent}"turnCycleDegrees": {degrees},')
    else:  # "loops" era a ultima chave do objeto
        lines[at] = lines[at].rstrip() + ","
        lines.insert(at + 1, f'{indent}"turnCycleDegrees": {degrees}')
    path.write_text("\n".join(lines))
    json.loads(path.read_text())  # continua valido
    print(f"{path.name}: gira {float(sys.argv[2]):+.0f} graus por ciclo")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
