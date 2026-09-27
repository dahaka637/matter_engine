"""Validacao cinematica de poses e clipes autorados.

Toda pose que entra num clipe precisa ser fisicamente possivel no NOSSO corpo.
O visualizador de animacao desenha o clipe sem fisica; o jogo desenha o corpo
simulado, que colide consigo mesmo. Uma pose em que o braco atravessa o peito
aparece "certa" no visualizador e empurrada para fora no jogo - foi exatamente
a divergencia vista em 26/09 com o idle antigo (tronco superior 17 graus fora,
bracos levantados; sem autocolisao o erro caia para 0,1 grau).

Checagens:
  - autocolisao: folga com sinal entre colisores que o solver deixa colidir
    (todos os pares menos pai-filho direto, a mesma regra do backend);
  - limites articulares, antes do clamp que o motor aplicaria;
  - chao: sola mais baixa em relacao ao piso.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from matter_rig import Rig, qmul, qrotate

# Folga minima exigida entre colisores do proprio corpo. Encostar ja gera
# forca de contato a cada passo; 5 mm deixa margem para a deriva do solver.
SELF_CLEARANCE_METERS = 0.005


@dataclass
class WorldCollider:
    shape: str
    center: np.ndarray
    orientation: np.ndarray
    half_extents: np.ndarray
    segment: tuple[np.ndarray, np.ndarray] | None
    radius: float                   # na ponta segment[0] (-X local)
    end_radius: float = 0.0         # na ponta segment[1] (+X local)

    def radii(self, t: np.ndarray) -> np.ndarray:
        """Raio ao longo do segmento (t = 0 na ponta -X, 1 na +X)."""
        return self.radius + (self.end_radius - self.radius) * t


def world_colliders(rig: Rig, positions: np.ndarray,
                    orientations: np.ndarray) -> list[WorldCollider]:
    result = []
    for i, link in enumerate(rig.links):
        c = link.collider
        orientation = qmul(orientations[i], c.local_orientation)
        center = positions[i] + qrotate(orientations[i], c.local_position)
        if c.shape == "capsule":
            # Capsulas de perfil sao longitudinais em X local (ver o teste
            # adversarial do peito e JoltRagdoll3D). Conica: esferas de raio
            # radius (-X) e radius_positive_x (+X); o comprimento total inclui
            # as duas tampas.
            half = max(0.0, 0.5 * (c.length - c.radius - c.radius_positive_x))
            axis = qrotate(orientation, np.array([half, 0.0, 0.0]))
            result.append(WorldCollider("capsule", center, orientation,
                                        np.zeros(3),
                                        (center - axis, center + axis),
                                        c.radius, c.radius_positive_x))
        else:
            result.append(WorldCollider("box", center, orientation,
                                        c.half_extents, None, 0.0))
    return result


def _segment_distance(p1, q1, p2, q2) -> float:
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a <= 1e-12 and e <= 1e-12:
        return float(np.linalg.norm(r))
    if a <= 1e-12:
        s, t = 0.0, min(max(f / e, 0.0), 1.0)
    else:
        c = d1 @ r
        if e <= 1e-12:
            t, s = 0.0, min(max(-c / a, 0.0), 1.0)
        else:
            b = d1 @ d2
            denom = a * e - b * b
            s = min(max((b * f - c * e) / denom, 0.0), 1.0) if denom > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(max(-c / a, 0.0), 1.0)
            elif t > 1.0:
                t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def _box_signed_distance(box: WorldCollider, points: np.ndarray) -> np.ndarray:
    local = qrotate(np.array([-box.orientation[0], -box.orientation[1],
                              -box.orientation[2], box.orientation[3]]),
                    points - box.center)
    q = np.abs(local) - box.half_extents
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
    inside = np.minimum(np.max(q, axis=-1), 0.0)
    return outside + inside


def _box_axes(box: WorldCollider) -> np.ndarray:
    return qrotate(box.orientation, np.eye(3))


def _box_box_gap(a: WorldCollider, b: WorldCollider) -> float:
    """SAT: profundidade de penetracao (negativa) ou separacao minima garantida.

    Separado, o valor e uma cota inferior da distancia real - conservador para
    exigir folga.
    """
    axes_a, axes_b = _box_axes(a), _box_axes(b)
    candidates = list(axes_a) + list(axes_b)
    for u in axes_a:
        for v in axes_b:
            w = np.cross(u, v)
            n = np.linalg.norm(w)
            if n > 1e-6:
                candidates.append(w / n)
    delta = b.center - a.center
    best_separation = -np.inf
    for axis in candidates:
        ra = np.sum(np.abs(axes_a @ axis) * a.half_extents)
        rb = np.sum(np.abs(axes_b @ axis) * b.half_extents)
        separation = abs(delta @ axis) - (ra + rb)
        best_separation = max(best_separation, separation)
    return float(best_separation)


def _tapered(c: WorldCollider) -> bool:
    return abs(c.end_radius - c.radius) > 1e-6


def collider_gap(a: WorldCollider, b: WorldCollider) -> float:
    """Distancia com sinal entre dois colisores (negativa = penetracao).

    Capsula conica e amostrada com o raio interpolado entre as esferas das
    pontas; para as conicidades do corpo (raio caindo ~3 cm em 30 cm) a
    diferenca para o casco convexo exato do Jolt fica abaixo de 0,5%."""
    if a.shape == "capsule" and b.shape == "capsule":
        if not _tapered(a) and not _tapered(b):
            return _segment_distance(*a.segment, *b.segment) - a.radius - b.radius
        t = np.linspace(0.0, 1.0, 33)
        pa = a.segment[0] * (1 - t[:, None]) + a.segment[1] * t[:, None]
        pb = b.segment[0] * (1 - t[:, None]) + b.segment[1] * t[:, None]
        distance = np.linalg.norm(pa[:, None, :] - pb[None, :, :], axis=-1)
        return float(np.min(distance - a.radii(t)[:, None]
                            - b.radii(t)[None, :]))
    if a.shape == "box" and b.shape == "box":
        return _box_box_gap(a, b)
    capsule, box = (a, b) if a.shape == "capsule" else (b, a)
    t = np.linspace(0.0, 1.0, 97)
    points = capsule.segment[0] * (1 - t[:, None]) + capsule.segment[1] * t[:, None]
    return float(np.min(_box_signed_distance(box, points) - capsule.radii(t)))


def rest_gaps(rig: Rig) -> dict[tuple[int, int], float]:
    """Folga de cada par nao pai-filho na pose de repouso (bind).

    Guardada no proprio objeto: dois perfis podem ter o mesmo id (versoes do
    mesmo corpo), e um cache por id devolveria as folgas do outro.
    """
    cached = getattr(rig, "_rest_gaps", None)
    if cached is None:
        from matter_rig import quat
        positions, orientations = rig.forward_kinematics(
            np.zeros(3), quat(), np.zeros((len(rig), 3)))
        colliders = world_colliders(rig, positions, orientations)
        cached = {}
        for i in range(len(rig)):
            for j in range(i + 1, len(rig)):
                if rig.links[j].parent == i or rig.links[i].parent == j:
                    continue
                cached[(i, j)] = collider_gap(colliders[i], colliders[j])
        rig._rest_gaps = cached
    return cached


# Par ancestral-descendente quase encostado em repouso nao colide (ver
# buildSelfCollisionFilter em JoltRagdoll3D.cpp).
CHAIN_REST_CLEARANCE_METERS = 0.01


def _is_ancestor(rig: Rig, ancestor: int, link: int) -> bool:
    current = rig.links[link].parent
    while current >= 0:
        if current == ancestor:
            return True
        current = rig.links[current].parent
    return False


def self_collision_pairs(rig: Rig) -> list[tuple[int, int]]:
    """Pares que o solver deixa colidir - a mesma regra do backend Jolt
    (buildSelfCollisionFilter): fora pai-filho direto, fora par sobreposto em
    repouso, fora par ancestral-descendente a menos de 1 cm em repouso e fora
    os pares que o perfil declara (selfCollisionIgnoredPairs)."""
    pairs = []
    for (i, j), gap in rest_gaps(rig).items():
        if frozenset((i, j)) in rig.ignored_pairs:
            continue
        same_chain = _is_ancestor(rig, i, j) or _is_ancestor(rig, j, i)
        limit = CHAIN_REST_CLEARANCE_METERS if same_chain else 0.0
        if gap >= limit:
            pairs.append((i, j))
    return pairs


def required_clearance(rig: Rig, pair: tuple[int, int]) -> float:
    """Folga exigida: 5 mm, ou a que o par ja tem em repouso se for menor
    (as coxas ficam a 2 mm uma da outra no bind; exigir 5 mm reprovaria o
    proprio corpo parado)."""
    return min(SELF_CLEARANCE_METERS, rest_gaps(rig)[pair])


def self_collision_gaps(rig: Rig, positions: np.ndarray,
                        orientations: np.ndarray) -> list[tuple[str, str, float, float]]:
    """(link a, link b, folga, folga exigida) para cada par que colide."""
    colliders = world_colliders(rig, positions, orientations)
    return [(rig.links[i].id, rig.links[j].id,
             collider_gap(colliders[i], colliders[j]),
             required_clearance(rig, (i, j)))
            for i, j in self_collision_pairs(rig)]


def limit_violations(rig: Rig, coordinates: np.ndarray,
                     tolerance: float = 1e-4) -> list[tuple[str, int, float]]:
    """(link, eixo, excesso em radianos) para coordenadas fora dos limites."""
    result = []
    for i, link in enumerate(rig.links):
        if link.parent < 0:
            continue
        for a in range(3):
            axis = link.axes[a]
            value = coordinates[i, a]
            if not axis.enabled:
                if abs(value) > tolerance:
                    result.append((link.id, a, abs(value)))
                continue
            excess = max(axis.minimum - value, value - axis.maximum, 0.0)
            if excess > tolerance:
                result.append((link.id, a, excess))
    return result


def lowest_point(rig: Rig, positions: np.ndarray, orientations: np.ndarray,
                 link_ids: list[str]) -> float:
    """Menor Z entre os colisores dados (funcao de suporte, nao o centro)."""
    lowest = np.inf
    for c, link in zip(world_colliders(rig, positions, orientations), rig.links):
        if link.id not in link_ids:
            continue
        if c.shape == "capsule":
            lowest = min(lowest, c.segment[0][2] - c.radius,
                         c.segment[1][2] - c.end_radius)
        else:
            axes = _box_axes(c)
            lowest = min(lowest, c.center[2] - np.sum(np.abs(axes[:, 2])
                                                      * c.half_extents))
    return float(lowest)
