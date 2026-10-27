"""Parado em alerta - a única pose parada do jogador.

A pedido do usuario, esta e a unica postura parada: base larga e escalonada,
joelhos dobrados, peso baixo - "base forte". Como no strafe, a base vem do
proprio Idle.fbx retargeteado para o nosso rig
(tools/import_humanoid_animation.py; o retarget cru fica em
assets/animations/source/retargeted/alert_idle.retarget.json), com o minimo
que o corpo e o jogo pedem:

  - RUMO: na referencia o corpo inteiro esta virado para a direita (peito e
    cabeca a -21 graus, pelve a -43). O personagem olha para onde a camera
    aponta, entao o clipe gira ate o peito ficar de frente; a pelve fica a
    ~-22 graus, a base escalonada com o pe esquerdo a frente.
  - TRONCO, BRACOS E CABECA: permanecem como no Idle.fbx. Nao se mistura a
    pelve inclinada da fonte com uma coluna ou bracos redesenhados.
  - PES: a referencia tem um pe 0,8 cm dentro do chao e o outro 2 cm acima.
    Os dois ficam planos, parados no mundo (a media da referencia), na sola;
    os joelhos absorvem o balanco da respiracao.
  - ALTURA (03/10, pedido do usuario: postura mais elegante, nao "sempre
    semi-agachado"): a pelve sobe 3 cm e vai 5,5 cm para o pe de tras, com os
    pes no mesmo lugar. Os joelhos passam de 41/28 graus para ~17/15 - a
    base continua larga e escalonada, so menos agachada. (Subir sem deslocar
    estica a perna de tras de vez; mais alto, ~9/10 graus, a perna de tras
    trava reta em parte da respiracao.)

  - RETO DE LADO (23/10, pedido do usuario: "a coluna e torta, fica
    inclinada; no idle a coluna, a linha da cintura e a cabeca ficam retas"):
    o Idle.fbx inclina a pelve e curva a coluna para o lado. A pelve, a
    coluna, o pescoco e a cabeca ficam sem a inclinacao de lado (cada elo
    guarda o rumo e a inclinacao para a frente/tras); os pes sao plantados
    de novo por baixo.
  - POSTURA (09/10, pedido do usuario: "o pescoco reto, tipo postura
    ergonomica"): o Idle.fbx curva o alto das costas e baixa a cabeca 20
    graus; a correcao e a de pose.ergonomic_posture (a mesma dos giros): o
    pescoco reto sobre o tronco e o olhar no horizonte. A respiracao e o
    resto continuam por cima.

O resto e da referencia: a respiracao e o peso mudando de lado em 10 s, os
bracos soltos com os cotovelos dobrados.
"""

from __future__ import annotations

import math

import numpy as np

from authoring import ClipDefinition
from matter_clip import load_clip
from matter_rig import REPO_ROOT, Rig, qconj, qfrom_axis_angle, qmul, qrotate
from matter_rig import qlog, qnormalize
from pose import LegSpec, Pose, PoseBuilder, ergonomic_posture
from validate import lowest_point

ID = "alert_idle"
DISPLAY_NAME = "Parado em alerta"
SOURCE = (REPO_ROOT / "assets" / "animations" / "source" / "retargeted"
          / "alert_idle.retarget.json")
SIDES = ("Left", "Right")
PELVIS_RISE = 0.034   # (23/10: +3,5 mm, o quadril que desceu ao nivelar a cintura volta a altura de antes)
PELVIS_TOWARD_REAR_FOOT = 0.055


def _rz(degrees: float) -> np.ndarray:
    return qfrom_axis_angle([0.0, 0.0, 1.0], math.radians(degrees))


STRAIGHT = ("Abdomen", "Chest", "UpperChest", "Neck", "Head")


def _without_side_tilt(delta: np.ndarray) -> np.ndarray:
    """A rotacao (do mundo, em relacao a ligacao) sem a inclinacao de lado:
    o rumo do elo e a inclinacao dele para a frente ou para tras ficam."""
    forward = qrotate(delta, np.array([1.0, 0.0, 0.0]))
    up = qrotate(delta, np.array([0.0, 0.0, 1.0]))
    yaw = math.atan2(forward[1], forward[0])
    c, s = math.cos(-yaw), math.sin(-yaw)
    pitch = math.atan2(c * up[0] - s * up[1], up[2])
    return qnormalize(qmul(qfrom_axis_angle([0.0, 0.0, 1.0], yaw),
                           qfrom_axis_angle([0.0, 1.0, 0.0], pitch)))


def straighten(rig: Rig, builder) -> None:
    """A pelve, a coluna, o pescoco e a cabeca sem a inclinacao de lado (no
    builder, antes das pernas)."""
    pelvis = rig.links[0]
    world = qmul(qmul(pelvis.model_orientation, builder.root_rotation),
                 qconj(pelvis.model_orientation))
    builder.root_rotation = qnormalize(qmul(qmul(
        qconj(pelvis.model_orientation), _without_side_tilt(world)),
        pelvis.model_orientation))
    for link_id in STRAIGHT:
        i = rig.index(link_id)
        link = rig.links[i]
        parent = rig.links[link.parent]
        _, orientations = rig.forward_kinematics(builder.root_offset,
                                                 builder.root_rotation,
                                                 builder.coordinates)
        delta = qmul(orientations[i], qconj(link.model_orientation))
        wanted = qmul(_without_side_tilt(delta), link.model_orientation)
        local = qmul(qconj(orientations[link.parent]), wanted)
        parent_frame = qnormalize(qmul(qconj(parent.model_orientation),
                                       link.frame))
        child_frame = qnormalize(qmul(qconj(link.model_orientation),
                                      link.frame))
        builder.coordinates[i] = rig.clamp(i, qlog(qmul(qmul(
            qconj(parent_frame), local), child_frame)))


class AlertIdle:
    def __init__(self, rig: Rig):
        self.rig = rig
        self.index = {link.id: i for i, link in enumerate(rig.links)}
        builder = PoseBuilder(rig)
        self.bind = builder.bind_orientations
        self.floor = lowest_point(rig, builder.bind_positions, self.bind,
                                  ["LeftFoot", "RightFoot"])
        clip = load_clip(SOURCE, rig)
        # O ultimo quadro repete o primeiro.
        self.frames = len(clip.coordinates) - 1
        self.duration = clip.duration
        self.sample_rate = self.frames / self.duration
        # Rumo: o peito de frente, em media.
        chest = self.index["UpperChest"]
        yaws = []
        for f in range(self.frames):
            _, o = rig.forward_kinematics(clip.root_offsets[f],
                                          clip.root_rotations[f],
                                          clip.coordinates[f])
            yaws.append(self._yaw(o[chest], chest))
        turn = _rz(-float(np.mean(yaws)))
        pelvis = rig.links[0].model_position
        offsets = np.array([qrotate(turn, pelvis + o) - pelvis
                            for o in clip.root_offsets[:self.frames]])
        # A pelve sobre a capsula, em media (so no plano).
        offsets[:, :2] -= offsets[:, :2].mean(axis=0)
        self.offsets = offsets
        self.rotations = np.array([qnormalize(qmul(turn, r))
                                   for r in clip.root_rotations[:self.frames]])
        self.coordinates = clip.coordinates[:self.frames].copy()
        self.reference = [rig.forward_kinematics(self.offsets[f],
                                                 self.rotations[f],
                                                 self.coordinates[f])
                          for f in range(self.frames)]
        self.feet = {side: self._planted(side) for side in SIDES}
        # Postura mais alta: a pelve sobe e vai para o pe de tras (o direito),
        # com os pes plantados onde estavam.
        rear = self.feet["Right"][0] - self.feet["Left"][0]
        rear[2] = 0.0
        rear /= np.linalg.norm(rear)
        self.offsets = self.offsets + rear * PELVIS_TOWARD_REAR_FOOT \
            + np.array([0.0, 0.0, PELVIS_RISE])

    def _yaw(self, orientation, index) -> float:
        delta = qmul(orientation, qconj(self.bind[index]))
        forward = qrotate(delta, np.array([1.0, 0.0, 0.0]))
        return math.degrees(math.atan2(forward[1], forward[0]))

    def anchor(self, positions, orientations, link_id: str) -> np.ndarray:
        i = self.index[link_id]
        link = self.rig.links[i]
        local = qrotate(qconj(link.model_orientation),
                        link.anchor - link.model_position)
        return positions[i] + qrotate(orientations[i], local)

    def _planted(self, side: str):
        """Pe parado: tornozelo medio da referencia, plano, com a sola no
        chao, no rumo medio dela."""
        foot = self.index[f"{side}Foot"]
        ankles, yaws = [], []
        for positions, orientations in self.reference:
            ankles.append(self.anchor(positions, orientations, f"{side}Foot"))
            yaws.append(self._yaw(orientations[foot], foot))
        ankle = np.mean(ankles, axis=0)
        yaw = float(np.degrees(np.arctan2(np.mean(np.sin(np.radians(yaws))),
                                          np.mean(np.cos(np.radians(yaws))))))
        ankle[2] = self.floor + PoseBuilder(self.rig).floor_ankle_height(side)
        return ankle, yaw

    def pose(self, f: int) -> Pose:
        f %= self.frames
        offset = self.offsets[f]
        rotation = self.rotations[f]
        positions, orientations = self.reference[f]
        builder = PoseBuilder(self.rig)
        builder.root_offset = np.array(offset, dtype=float)
        builder.root_rotation = np.array(rotation, dtype=float)
        builder.coordinates = self.coordinates[f].copy()
        # A postura ergonomica e o reto de lado (ver o topo).
        ergonomic_posture(self.rig, builder.coordinates)
        straighten(self.rig, builder)
        for side in SIDES:
            hip = self.anchor(positions, orientations, f"{side}Thigh")
            knee = self.anchor(positions, orientations, f"{side}Shin")
            ankle, yaw = self.feet[side]
            axis = (ankle - hip) / np.linalg.norm(ankle - hip)
            pole = (knee - hip) - axis * ((knee - hip) @ axis)
            s = 1.0 if side == "Left" else -1.0
            builder.leg(side, LegSpec(ankle=ankle,
                                      knee_direction=pole / np.linalg.norm(pole),
                                      toe_out_degrees=s * yaw))
        return builder.solve()


def definition(rig: Rig) -> ClipDefinition:
    idle = AlertIdle(rig)
    poses = [idle.pose(f) for f in range(idle.frames)]
    closed = poses + poses[:1]

    def pose_at(t: float) -> Pose:
        x = (t % idle.duration) * idle.sample_rate
        a = int(math.floor(x)) % idle.frames
        b = a + 1
        w = x - math.floor(x)
        pa, pb = closed[a], closed[b]
        rotation = pa.root_rotation if w < 1e-6 else _slerp(
            pa.root_rotation, pb.root_rotation, w)
        return Pose(pa.root_offset * (1 - w) + pb.root_offset * w, rotation,
                    pa.coordinates * (1 - w) + pb.coordinates * w)

    return ClipDefinition(
        id=ID, display_name=DISPLAY_NAME, duration=idle.duration, loops=True,
        pose_at=pose_at, sample_rate=idle.sample_rate, floor_z=idle.floor,
        source="tools/animation/clips/alert_idle.py")


def _slerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    if np.dot(a, b) < 0:
        b = -b
    angle = math.acos(min(1.0, abs(float(np.dot(a, b)))))
    if angle < 1e-6:
        return a.copy()
    return qnormalize((math.sin((1 - t) * angle) * a
                       + math.sin(t * angle) * b) / math.sin(angle))
