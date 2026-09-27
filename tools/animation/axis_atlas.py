"""Mapa semantico dos eixos articulares: o que cada eixo faz com o corpo.

"swing1 +30 graus no ombro" nao diz nada a quem autora. Este script aplica
+0,5 rad em cada eixo de cada junta, a partir da pose de repouso, e mede para
onde o segmento vai (frente/tras, esquerda/direita, cima/baixo) ou se ele gira
em torno de si (twist). A tabela sai da geometria do rig, nao de suposicao
sobre nomes - e e a base da linguagem de autoria em pose.py.

Uso: python3 axis_atlas.py
"""

from __future__ import annotations

import math

import numpy as np

from matter_rig import AXIS_NAMES, Rig, DEFAULT_CHARACTER_DIR, qrotate, quat
from validate import world_colliders

PROBE_RADIANS = 0.5
DIRECTIONS = {
    "frente": np.array([1.0, 0.0, 0.0]), "tras": np.array([-1.0, 0.0, 0.0]),
    "esquerda": np.array([0.0, 1.0, 0.0]), "direita": np.array([0.0, -1.0, 0.0]),
    "cima": np.array([0.0, 0.0, 1.0]), "baixo": np.array([0.0, 0.0, -1.0]),
}


def segment_tip(rig: Rig, index: int, positions, orientations) -> np.ndarray:
    """Ponta do segmento: ancora do primeiro filho, ou o fim do colisor."""
    children = rig.children(index)
    if children:
        child = rig.links[children[0]]
        local = qrotate(np.array([-rig.links[index].model_orientation[0],
                                  -rig.links[index].model_orientation[1],
                                  -rig.links[index].model_orientation[2],
                                  rig.links[index].model_orientation[3]]),
                        child.anchor - rig.links[index].model_position)
        return positions[index] + qrotate(orientations[index], local)
    collider = world_colliders(rig, positions, orientations)[index]
    if collider.segment is not None:
        a, b = collider.segment
        anchor = positions[index]
        return a if np.linalg.norm(a - anchor) > np.linalg.norm(b - anchor) else b
    return collider.center


def describe(delta: np.ndarray) -> str:
    if np.linalg.norm(delta) < 1e-6:
        return "-"
    delta = delta / np.linalg.norm(delta)
    ranked = sorted(DIRECTIONS.items(), key=lambda item: -(item[1] @ delta))
    first, second = ranked[0], ranked[1]
    if second[1] @ delta > 0.35:
        return f"{first[0]}+{second[0]}"
    return first[0]


def main() -> None:
    rig = Rig(DEFAULT_CHARACTER_DIR / "FootballPlayerV1.ragdoll.json")
    zero = np.zeros((len(rig), 3))
    base_p, base_o = rig.forward_kinematics(np.zeros(3), quat(), zero)
    print(f"{'junta':<14}{'eixo':<8}{'limites (graus)':<18}efeito de +{math.degrees(PROBE_RADIANS):.0f} graus")
    for i, link in enumerate(rig.links):
        if link.parent < 0:
            continue
        anchor = link.anchor
        base_tip = segment_tip(rig, i, base_p, base_o) - anchor
        for a in range(3):
            axis = link.axes[a]
            if not axis.enabled:
                continue
            coordinates = zero.copy()
            coordinates[i, a] = PROBE_RADIANS
            p, o = rig.forward_kinematics(np.zeros(3), quat(), coordinates)
            tip = segment_tip(rig, i, p, o) - (p[i] + qrotate(
                o[i], qrotate(np.array([-link.model_orientation[0],
                                        -link.model_orientation[1],
                                        -link.model_orientation[2],
                                        link.model_orientation[3]]),
                              anchor - link.model_position)))
            delta = tip - base_tip
            if np.linalg.norm(delta) < 0.02 * max(np.linalg.norm(base_tip), 1e-3):
                # Ponta parada: rotacao em torno do proprio segmento. O lado
                # para onde a frente do segmento gira identifica o sentido.
                side = qrotate(o[i], np.array([1.0, 0.0, 0.0])) - qrotate(
                    base_o[i], np.array([1.0, 0.0, 0.0]))
                effect = f"torce (frente do segmento -> {describe(side)})"
            else:
                effect = f"ponta vai para {describe(delta)}"
            limits = f"{math.degrees(axis.minimum):+.0f} / {math.degrees(axis.maximum):+.0f}"
            print(f"{link.id:<14}{AXIS_NAMES[a]:<8}{limits:<18}{effect}")


if __name__ == "__main__":
    main()
