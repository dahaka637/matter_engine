"""Pulo parado pela referencia do usuario (03/10): Jumping(3).fbx, o mesmo
movimento para o pulo tocado e o carregado (a carga so muda a altura e o
agachamento). Retarget cru em assets/animations/source/retargeted
(jump_standing_reference, importado com --root-motion preserve).

Quem voa e a fisica: os clipes so trazem a pose. A raiz sai sem deslocamento
(a altura do voo e do agachamento e do corpo e da pelve de referencia no
runtime) e sem o rumo da referencia (so a inclinacao).

  - jump_standing: o voo do Jumping(3), da decolagem ao toque (pes acima de
    2 cm do chao). Duracao nominal de 1 s: o runtime amostra pela fase do voo
    (0 decola, 0,5 apice, 1 toca).
  - jump_prep: o agachamento antes do impulso (de em pe ao
    mais fundo), percorrido pela carga do pulo enquanto o jogador segura.
"""

from __future__ import annotations

import math

import numpy as np

from authoring import ClipDefinition
from matter_clip import load_clip
from matter_rig import REPO_ROOT, Rig, qconj, qfrom_axis_angle, qmul, qnormalize, qrotate
from pose import Pose
from validate import lowest_point

SOURCES = REPO_ROOT / "assets" / "animations" / "source" / "retargeted"
CLEARANCE = 0.02   # pe acima disto: no ar
PREP_PITCH = 0.5   # do avanco da pelve no agachamento da referencia
ARM_DROP_START = 0.42  # fase do voo em que os bracos comecam a descer
ARM_DROP_SPAN = 0.28   # e em quanto chegam a posicao de pe


class Reference:
    def __init__(self, rig: Rig, name: str):
        self.rig = rig
        self.clip = load_clip(SOURCES / f"{name}.retarget.json", rig)
        frames = len(self.clip.times)
        soles, pelvis = [], []
        for f in range(frames):
            pos, ori = rig.forward_kinematics(self.clip.root_offsets[f],
                                              self.clip.root_rotations[f],
                                              self.clip.coordinates[f])
            soles.append((lowest_point(rig, pos, ori, ["LeftFoot"]),
                          lowest_point(rig, pos, ori, ["RightFoot"])))
            pelvis.append(pos[0][2])
        floor = min(min(s) for s in soles)
        airborne = [min(s) - floor > CLEARANCE for s in soles]
        times = self.clip.times
        first_air = next(f for f in range(frames) if airborne[f])
        self.takeoff = float(times[first_air])
        landing = next((f for f in range(first_air, frames) if not airborne[f]), None)
        self.touchdown = float(times[landing]) if landing is not None else float(times[-1])
        self.lands = landing is not None
        # Agachamento: da pelve comecando a descer ao ponto mais baixo antes
        # da decolagem.
        standing = pelvis[0]
        deepest = int(np.argmin(pelvis[:first_air]))
        start = next((f for f in range(deepest) if pelvis[f] < standing - 0.01), 0)
        self.prep_start = float(times[max(0, start - 1)])
        self.prep_end = float(times[deepest])

    def pose(self, time: float, yaw_reference: float, pitch_share: float = 0.0) -> Pose:
        offset, rotation, coordinates = self.clip.sample(time, loop=False)
        # So uma parte da inclinacao para a frente da raiz, sem rolagem nem
        # rumo: o rumo e o do runtime, e a referencia inclina de lado (ate 14
        # graus no voo do pulo forte) e para tras no voo - no corpo fisico o
        # pulo saia girando.
        unturn = qfrom_axis_angle(np.array([0.0, 0.0, 1.0]), -yaw_reference)
        up = qrotate(qnormalize(qmul(unturn, rotation)), np.array([0.0, 0.0, 1.0]))
        pitch = math.asin(max(-1.0, min(1.0, up[0])))
        rotation = qfrom_axis_angle(np.array([0.0, 1.0, 0.0]), pitch * pitch_share)
        return Pose(np.zeros(3), np.array(rotation), np.array(coordinates))

    def yaw(self, time: float) -> float:
        _, rotation, _ = self.clip.sample(time, loop=False)
        forward = qrotate(rotation, np.array([1.0, 0.0, 0.0]))
        return math.atan2(forward[1], forward[0])


def definitions(rig: Rig) -> list[ClipDefinition]:
    normal = Reference(rig, "jump_standing_reference")
    normal_yaw = normal.yaw(normal.takeoff)
    # Bracos de pe (antes do pulo): para onde eles voltam na descida.
    standing = normal.pose(normal.clip.times[0], normal_yaw).coordinates
    arms = [i for i, link in enumerate(rig.links)
            if any(part in link.id for part in ("UpperArm", "Forearm", "Hand"))]

    def flight(t: float) -> Pose:
        source = normal.takeoff + t * (normal.touchdown - normal.takeoff)
        pose = normal.pose(source, normal_yaw)
        # Os bracos sobem no impulso e descem logo depois do apice (pedido do
        # usuario: "so levanta na hora que pula"); na referencia eles ficavam
        # no alto o voo inteiro.
        drop = min(max((t - ARM_DROP_START) / ARM_DROP_SPAN, 0.0), 1.0)
        drop = drop * drop * (3.0 - 2.0 * drop)
        for i in arms:
            pose.coordinates[i] = pose.coordinates[i] + (standing[i] - pose.coordinates[i]) * drop
        return pose

    def prep(t: float) -> Pose:
        source = normal.prep_start + t * (normal.prep_end - normal.prep_start)
        return normal.pose(source, normal_yaw, PREP_PITCH)

    common = dict(duration=1.0, loops=False, contact_at=lambda t: (0.0, 0.0),
                  sample_rate=30.0, floor_z=-10.0,
                  source="tools/animation/clips/jump_reference.py")
    return [
        ClipDefinition(id="jump_standing", display_name="Pulo parado",
                       pose_at=flight, **common),
        ClipDefinition(id="jump_prep", display_name="Preparo do pulo",
                       pose_at=prep, **common),
    ]
