"""Arrancada agachada (pedido do usuario, 05/10 noite): Crouched To Sprinting.

Referencia (Mixamo, fornecida pelo usuario), retargeteada com
  import_humanoid_animation.py --source-rig mixamo --reference-only --root-motion preserve
  -> assets/animations/source/retargeted/crouched_to_sprint.retarget.json

  - 0,5 s: do agachamento fundo (quadril a 0,52 m, joelhos a 130-140 graus,
    tronco 52-69 graus a frente, na ponta dos pes) o impulso e duas passadas,
    terminando correndo a ~4,4 m/s com o quadril a 0,83 m, 1,5 m a frente.

No jogo: levantando com as teclas de andar, o levantar vai ate perto da
pose agachada e troca para este clipe, virado para onde as teclas pedem; no
fim a locomocao continua a passada.

A limpeza e a mesma do carrinho (tools/animation/clips/tackle.py): a raiz no
rumo do primeiro quadro, o apoio no chao pelos contatos da referencia, o
tornozelo girado para o pe nao atravessar o chao e as maos acima dele.

Uso:
  python3 tools/animation/clips/crouch_to_sprint.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from matter_rig import Rig  # noqa: E402

import stand_up_getup as g  # noqa: E402
import tackle  # noqa: E402


def main() -> None:
    rig = Rig(g.PROFILE)
    count, heights, planar, rotations, coordinates, floor = tackle.build(
        rig, "crouched_to_sprint", 0.0, 0.5, False)
    tackle.write(rig, "crouched_to_sprint", "Arrancada agachada (Crouched To Sprinting)",
                 "crouched_to_sprint", count, heights, planar, rotations, coordinates, floor)


if __name__ == "__main__":
    main()
