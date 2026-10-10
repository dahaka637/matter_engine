"""Rig do MatterEngine em Python: perfil fisico, cinematica direta e skin.

Espelha exatamente o que o motor faz em RagdollProfile3D.cpp (leitura do
perfil), AnimationClip3D.cpp (sampleRagdollAnimationPose3D, a cinematica
direta de um clipe) e RagdollCharacter3D.cpp (skin com pesos). As ferramentas de
autoria, validacao e previa usam este modulo; se o motor mudar a convencao,
este arquivo muda junto - e check_engine_parity() existe para pegar a
divergencia.

Convencoes (as mesmas do motor):
  - quaternions sao numpy [x, y, z, w], produto de Hamilton;
  - mundo Z-up destro; o modelo olha para +X e a esquerda e +Y;
  - coordenada de junta e um vetor de rotacao (mapa exponencial) no frame
    articular: x = twist, y = swing1, z = swing2.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CHARACTER_DIR = REPO_ROOT / "assets" / "characters" / "football_player"


# --- quaternions --------------------------------------------------------------

def quat(x: float = 0.0, y: float = 0.0, z: float = 0.0,
         w: float = 1.0) -> np.ndarray:
    return np.array([x, y, z, w], dtype=np.float64)


def qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def qconj(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]])


def qnormalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def qrotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Roda um vetor (3,) ou um lote (N, 3), como Quaternion::rotate."""
    q = qnormalize(q)
    imaginary = q[:3]
    twice_cross = 2.0 * np.cross(imaginary, v)
    return v + q[3] * twice_cross + np.cross(imaginary, twice_cross)


def qfrom_axis_angle(axis: np.ndarray, radians: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    half = 0.5 * radians
    return np.array([*(axis * math.sin(half)), math.cos(half)])


def qexp(rotation_vector: np.ndarray) -> np.ndarray:
    """Mapa exponencial, identico a quaternionExponential no motor."""
    angle = float(np.linalg.norm(rotation_vector))
    if angle < 1e-6:
        return quat()
    return qfrom_axis_angle(rotation_vector / angle, angle)


def qlog(q: np.ndarray) -> np.ndarray:
    """Inverso de qexp: quaternion -> vetor de rotacao (menor rotacao)."""
    q = qnormalize(q)
    if q[3] < 0.0:
        q = -q
    s = float(np.linalg.norm(q[:3]))
    if s < 1e-9:
        return np.zeros(3)
    return q[:3] * (2.0 * math.atan2(s, q[3]) / s)


def qangle_degrees(a: np.ndarray, b: np.ndarray) -> float:
    d = abs(float(np.dot(qnormalize(a), qnormalize(b))))
    return math.degrees(2.0 * math.acos(min(1.0, d)))


# --- perfil fisico -------------------------------------------------------------

AXIS_NAMES = ("twist", "swing1", "swing2")


@dataclass
class JointAxis:
    enabled: bool = False
    minimum: float = 0.0   # radianos
    maximum: float = 0.0


@dataclass
class Collider:
    shape: str                      # "box" | "capsule"
    local_position: np.ndarray
    local_orientation: np.ndarray
    half_extents: np.ndarray        # box
    length: float                   # capsula: comprimento total com as tampas
    radius: float                   # capsula: raio na ponta -X local
    # Raio na ponta +X local (capsula conica); igual a radius se uniforme.
    radius_positive_x: float = 0.0


@dataclass
class Link:
    id: str
    parent: int
    model_position: np.ndarray
    model_orientation: np.ndarray
    mass_fraction: float
    center_of_mass_local: np.ndarray
    collider: Collider
    anchor: np.ndarray = field(default_factory=lambda: np.zeros(3))
    frame: np.ndarray = field(default_factory=quat)
    axes: list[JointAxis] = field(
        default_factory=lambda: [JointAxis(), JointAxis(), JointAxis()])


class Rig:
    """Perfil de ragdoll com a cinematica direta do motor."""

    def __init__(self, profile_path: Path | str):
        data = json.loads(Path(profile_path).read_text(encoding="utf-8"))
        self.id: str = data["id"]
        self.total_mass: float = float(data["totalMassKg"])
        self.standing_root_height: float = float(
            data["standingRootHeightMeters"])
        uniform_radius = float(data["uniformRadiusMeters"])
        self.links: list[Link] = []
        for source in data["links"]:
            has_box = "box" in source
            shape = source["box"] if has_box else source["capsule"]
            collider = Collider(
                shape="box" if has_box else "capsule",
                local_position=np.array(shape.get("position", [0, 0, 0]),
                                        dtype=np.float64),
                local_orientation=np.array(shape["orientation"],
                                           dtype=np.float64),
                half_extents=np.array(shape.get("halfExtents", [0, 0, 0]),
                                      dtype=np.float64),
                length=float(shape.get("length", 0.0)),
                radius=float(shape.get("radius", uniform_radius)),
                radius_positive_x=float(shape.get(
                    "radiusAtPositiveX", shape.get("radius", uniform_radius))),
            )
            link = Link(
                id=source["id"],
                parent=int(source["parent"]),
                model_position=np.array(source["position"], dtype=np.float64),
                model_orientation=np.array(source.get("orientation",
                                                      [0, 0, 0, 1]),
                                           dtype=np.float64),
                mass_fraction=float(source["massFraction"]),
                center_of_mass_local=np.array(
                    source.get("centerOfMass", [0, 0, 0]), dtype=np.float64),
                collider=collider,
            )
            if link.parent >= 0:
                joint = source["joint"]
                link.anchor = np.array(joint["anchor"], dtype=np.float64)
                link.frame = np.array(joint["frameOrientation"],
                                      dtype=np.float64)
                for axis in joint["axes"]:
                    index = AXIS_NAMES.index(axis["axis"])
                    link.axes[index] = JointAxis(
                        enabled=True,
                        minimum=math.radians(axis["minimumDegrees"]),
                        maximum=math.radians(axis["maximumDegrees"]))
            self.links.append(link)
        self._index = {link.id: i for i, link in enumerate(self.links)}
        # Pares que o perfil declara sem colisao (motivo no proprio perfil).
        self.ignored_pairs: set[frozenset[int]] = {
            frozenset(self._index[name] for name in pair["links"])
            for pair in data.get("selfCollisionIgnoredPairs", [])}

    def __len__(self) -> int:
        return len(self.links)

    def index(self, link_id: str) -> int:
        return self._index[link_id]

    def children(self, index: int) -> list[int]:
        return [i for i, link in enumerate(self.links) if link.parent == index]

    def clamp(self, index: int, coordinates: np.ndarray) -> np.ndarray:
        """Projeta nos limites, como jointLocalOrientation no motor."""
        link = self.links[index]
        result = np.zeros(3)
        for a in range(3):
            axis = link.axes[a]
            result[a] = (min(max(coordinates[a], axis.minimum), axis.maximum)
                         if axis.enabled else 0.0)
        return result

    def joint_local_orientation(self, index: int,
                                coordinates: np.ndarray) -> np.ndarray:
        link = self.links[index]
        parent = self.links[link.parent]
        parent_frame = qnormalize(qmul(qconj(parent.model_orientation),
                                       link.frame))
        child_frame = qnormalize(qmul(qconj(link.model_orientation),
                                      link.frame))
        clamped = self.clamp(index, coordinates)
        return qnormalize(qmul(qmul(parent_frame, qexp(clamped)),
                               qconj(child_frame)))

    def forward_kinematics(self, root_offset: np.ndarray,
                           root_rotation_delta: np.ndarray,
                           coordinates: np.ndarray):
        """Identico a sampleRagdollAnimationPose3D.

        coordinates: (N, 3) por link; a linha da raiz e ignorada.
        Devolve (posicoes (N, 3), orientacoes (N, 4)) em espaco de modelo.
        """
        count = len(self.links)
        positions = np.zeros((count, 3))
        orientations = np.zeros((count, 4))
        for i, link in enumerate(self.links):
            if link.parent < 0:
                positions[i] = link.model_position + root_offset
                orientations[i] = qnormalize(
                    qmul(link.model_orientation, root_rotation_delta))
                continue
            p = link.parent
            parent = self.links[p]
            local = self.joint_local_orientation(i, coordinates[i])
            orientations[i] = qnormalize(qmul(orientations[p], local))
            parent_anchor_local = qrotate(
                qconj(parent.model_orientation),
                link.anchor - parent.model_position)
            child_anchor_local = qrotate(
                qconj(link.model_orientation),
                link.anchor - link.model_position)
            world_anchor = positions[p] + qrotate(orientations[p],
                                                  parent_anchor_local)
            positions[i] = world_anchor - qrotate(orientations[i],
                                                  child_anchor_local)
        return positions, orientations

    def coordinates_for(self, index: int, parent_world: np.ndarray,
                        child_world: np.ndarray) -> np.ndarray:
        """Inverso da junta: orientacoes de mundo -> coordenada articular.

        Nao aplica limites; quem chama decide se clampa ou reprova.
        """
        link = self.links[index]
        parent = self.links[link.parent]
        parent_frame = qnormalize(qmul(qconj(parent.model_orientation),
                                       link.frame))
        child_frame = qnormalize(qmul(qconj(link.model_orientation),
                                      link.frame))
        local = qmul(qconj(parent_world), child_world)
        # local = parent_frame * exp(q) * child_frame^-1
        return qlog(qmul(qmul(qconj(parent_frame), local), child_frame))


# --- skin com pesos -----------------------------------------------------------------

class Skin:
    """Malha com ate quatro influencias por vertice, como no runtime.

    Ossos: primeiro um por link fisico, depois os visuais (dedos), que seguem
    um osso pai e nao tem fisica. Cada osso visual tem uma rotacao de
    repouso (a mao relaxada); pose(..., rest=False) desenha a malha com os
    dedos retos.

    Versao 2 (matter-ragdoll-skin-2, o jogador montado): cada osso guarda o
    repouso (o do perfil; os visuais se prendem ao pai por ele) e a ligacao da
    malha (a malha fica na pose A dela), e os auxiliares (joelho, quadril,
    tornozelo, ombro, cotovelo, pulso) vem depois dos visuais - giram a fracao
    do giro do osso de baixo em relacao ao de cima, em volta do pivo de
    verdade desse giro (buildRagdollSkinMatrices3D)."""

    def __init__(self, skin_path: Path | str, rig: Rig):
        data = json.loads(Path(skin_path).read_text(encoding="utf-8"))
        self.version = 2 if data["schema"] == "matter-ragdoll-skin-2" else 1
        rest_position = "restPosition" if self.version == 2 else "bindPosition"
        rest_orientation = "restOrientation" if self.version == 2 else "bindOrientation"
        bones = data["bones"]
        visual = data.get("visualBones", [])
        helpers = data.get("helperBones", []) if self.version == 2 else []
        names = [b["link"] for b in bones] + [b["name"] for b in visual]
        # Link fisico de cada osso (o visual herda o do ancestral fisico).
        links = [rig.index(b["link"]) for b in bones]
        self.visual_parent: list[int] = []
        for b in visual:
            parent = names.index(b["parent"])
            self.visual_parent.append(parent)
            links.append(links[parent])
        self.physical_count = len(bones)
        self.rest_position = np.array(
            [b[rest_position] for b in bones + visual], dtype=np.float64)
        self.rest_orientation = np.array(
            [b[rest_orientation] for b in bones + visual], dtype=np.float64)
        self.bind_position = np.array(
            [b["bindPosition"] for b in bones + visual], dtype=np.float64)
        self.bind_orientation = np.array(
            [b["bindOrientation"] for b in bones + visual], dtype=np.float64)
        self.rest_rotation = np.array(
            [b.get("restRotation", [0.0, 0.0, 0.0, 1.0]) for b in visual],
            dtype=np.float64).reshape(-1, 4)
        # Os auxiliares: os dois ossos, a fracao, o pivo e o eixo (so a parte
        # do giro em volta dele; sem, o giro inteiro). O link e o do de baixo,
        # para os diagnosticos.
        self.helper_upper = [names.index(h["upper"]) for h in helpers]
        self.helper_lower = [names.index(h["lower"]) for h in helpers]
        self.helper_fraction = [float(h.get("fraction", 0.5)) for h in helpers]
        self.helper_pivot = [np.array(h["pivot"], dtype=np.float64) for h in helpers]
        self.helper_axis = [np.array(h.get("axis", [0.0, 0.0, 0.0]), dtype=np.float64) for h in helpers]
        for h, lower in zip(helpers, self.helper_lower):
            names.append(h["name"])
            links.append(links[lower])
        self.bone_link = np.array(links)
        self.bone_names = names
        vertices = data["vertices"]
        self.positions = np.array([v["position"] for v in vertices],
                                  dtype=np.float64)
        self.joints = np.array([v["joints"] for v in vertices], dtype=np.int64)
        self.weights = np.array([v["weights"] for v in vertices], dtype=np.float64)
        # Dominante continua disponivel para rotulos/diagnosticos.
        self.bone = np.array([
            v["joints"][int(np.argmax(v["weights"]))] for v in vertices])
        self.indices = np.array(data["indices"], dtype=np.int64).reshape(-1, 3)

    def bone_transforms(self, positions: np.ndarray, orientations: np.ndarray,
                        rest: bool = True, visual: np.ndarray | None = None):
        """Posicao e orientacao no mundo de cada osso fisico e visual, como
        buildRagdollSkinMatrices3D. visual: a rotacao de cada osso visual no
        lugar da de repouso (a pose dos dedos e dos dedos do pe)."""
        count = self.physical_count + len(self.visual_parent)
        world_p = np.zeros((count, 3))
        world_q = np.zeros((count, 4))
        for bone in range(self.physical_count):
            world_p[bone] = positions[self.bone_link[bone]]
            world_q[bone] = orientations[self.bone_link[bone]]
        for k, parent in enumerate(self.visual_parent):
            bone = self.physical_count + k
            inverse = qconj(self.rest_orientation[parent])
            local_p = qrotate(inverse, self.rest_position[bone]
                              - self.rest_position[parent])
            local_q = qmul(inverse, self.rest_orientation[bone])
            if visual is not None and k < len(visual):
                local_q = qmul(local_q, np.asarray(visual[k], dtype=np.float64))
            elif rest:
                local_q = qmul(local_q, self.rest_rotation[k])
            world_p[bone] = world_p[parent] + qrotate(world_q[parent], local_p)
            world_q[bone] = qnormalize(qmul(world_q[parent], local_q))
        return world_p, world_q

    def skin_transforms(self, positions: np.ndarray, orientations: np.ndarray,
                        rest: bool = True, visual: np.ndarray | None = None):
        """O skin de cada osso da paleta (translacao, rotacao): o mundo vezes
        o inverso da ligacao da malha; os auxiliares saem dos dois ossos."""
        world_p, world_q = self.bone_transforms(positions, orientations, rest, visual)
        skin_q = [qnormalize(qmul(world_q[b], qconj(self.bind_orientation[b])))
                  for b in range(len(world_p))]
        skin_p = [world_p[b] - qrotate(skin_q[b], self.bind_position[b])
                  for b in range(len(world_p))]
        for upper, lower, fraction, pivot_hint, axis in zip(
                self.helper_upper, self.helper_lower, self.helper_fraction,
                self.helper_pivot, self.helper_axis):
            uq, up = skin_q[upper], skin_p[upper]
            relative_q = qnormalize(qmul(qconj(uq), skin_q[lower]))
            relative_p = qrotate(qconj(uq), skin_p[lower] - up)
            part = _partial_rotation(_twist_about(relative_q, axis), fraction)
            pivot = _rotation_pivot(relative_q, relative_p, pivot_hint)
            skin_q.append(qnormalize(qmul(uq, part)))
            skin_p.append(up + qrotate(uq, pivot - qrotate(part, pivot)))
        return np.array(skin_p), np.array(skin_q)

    def pose(self, positions: np.ndarray, orientations: np.ndarray,
             rest: bool = True, visual: np.ndarray | None = None) -> np.ndarray:
        """Vertices no mundo para uma pose de links."""
        skin_p, skin_q = self.skin_transforms(positions, orientations, rest, visual)
        result = np.zeros_like(self.positions)
        for bone in range(len(skin_p)):
            weight = np.sum(np.where(self.joints == bone, self.weights, 0), axis=1)
            mask = weight > 0
            if not np.any(mask):
                continue
            transformed = skin_p[bone] + qrotate(skin_q[bone], self.positions[mask])
            result[mask] += transformed * weight[mask, None]
        return result


def _partial_rotation(q: np.ndarray, fraction: float) -> np.ndarray:
    """A fracao do giro q (o mesmo eixo, pelo caminho mais curto)."""
    q = qnormalize(q)
    if q[3] < 0:
        q = -q
    half = math.acos(min(1.0, max(-1.0, float(q[3]))))
    s = math.sin(half)
    if s < 1e-9:
        return quat()
    axis = q[:3] / s
    h = half * fraction
    return np.array([*(axis * math.sin(h)), math.cos(h)])


def _twist_about(q: np.ndarray, axis: np.ndarray) -> np.ndarray:
    """A parte do giro q em volta do eixo (balanco e torcao); sem eixo, o giro
    inteiro (twistAbout no motor e na modelagem)."""
    length = float(np.linalg.norm(axis))
    if length < 1e-6:
        return q
    a = axis / length
    along = float(np.dot(q[:3], a))
    twist = np.array([*(a * along), q[3]])
    norm = float(np.linalg.norm(twist))
    if norm < 1e-6:
        return quat()
    return twist / norm


def _rotation_pivot(q: np.ndarray, t: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    """O ponto que o giro relativo (q, t) deixa parado; quase sem giro, o da
    ligacao (rotationPivot em Skeleton.cpp e no motor)."""
    q = qnormalize(q)
    if q[3] < 0:
        q = -q
    half = math.acos(min(1.0, max(-1.0, float(q[3]))))
    s = math.sin(half)
    if s < 0.004:
        return fallback
    axis = q[:3] / s
    across_t = t - axis * float(np.dot(t, axis))
    across = (across_t + np.cross(axis, across_t) * (math.cos(half) / s)) * 0.5
    return across + axis * float(np.dot(fallback, axis))


def load_default_character() -> tuple[Rig, Skin]:
    """O personagem do laboratorio: o perfil e a pele do manifesto."""
    manifest = json.loads((DEFAULT_CHARACTER_DIR / "character.json").read_text(encoding="utf-8"))
    rig = Rig(DEFAULT_CHARACTER_DIR / manifest["physicsProfile"])
    return rig, Skin(DEFAULT_CHARACTER_DIR / manifest["skin"], rig)
