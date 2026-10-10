"""Trote em oito direcoes e o sprint para a frente - fieis as referencias.

Pedido do usuario (05/10): o trote de antes (clips/jog.py) parecia
caminhada - medido contra o Jog Forward da Mixamo, a pelve quase nao subia
e descia (3 contra 10 cm), o pe pousava 19 cm a frente do quadril (8 na
referencia), o calcanhar subia pouco no balanco (21 contra 33 cm) e o tronco
ia reto (8 contra 18 graus). Ele fica guardado (o cansaco, no futuro); estes
sao os clipes do trote:

  - trot_forward       <- Jog Forward
  - trot_backward      <- Jog Backward
  - trot_forward_left  <- Jog Forward Diagonal (direita: o mesmo espelhado)
  - trot_backward_left <- Jog Backward Diagonal (direita: espelhado)
  - sprint_backward_left/right: a diagonal de costas mais rapida (a cadencia
    sobe; o usuario pediu que de costas na diagonal nao perdesse tanta
    velocidade)
  - sprint_forward     <- Running (o sprint reto; o tronco da referencia,
    que balanca e solta os ombros da pelve - o nosso sprint autoral era
    robotico -, com a cabeca mais erguida: na referencia ela vai baixa)

Como o strafe (clips/strafe.py, de onde vem a mecanica): o retarget cru
(tools/import_humanoid_animation.py, em assets/animations/source/retargeted)
e so as correcoes que o corpo exige - o pe de apoio parado e plano no mundo,
nada abaixo do chao, coxas, pes e bracos sem se encostar. O ciclo e girado
para andar exatamente no rumo do clipe. Nas diagonais o usuario pediu para
ignorar os bracos da referencia: neles (e no trote inteiro) os bracos ficam
na postura padrao de corrida no tempo da referencia; no sprint, os da
referencia.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import strafe as base  # noqa: E402

from authoring import ClipDefinition  # noqa: E402
from gait import (RUNNING_ARM_INWARD_ROLL, RUNNING_ARM_LOWERED,  # noqa: E402
                  RUNNING_WRIST_FLEXION)
from matter_clip import load_clip  # noqa: E402
from matter_rig import (Rig, qconj, qfrom_axis_angle, qmul,  # noqa: E402
                        qnormalize, qrotate)
from pose import ArmSpec, LegSpec, Pose, PoseBuilder  # noqa: E402
from validate import self_collision_gaps  # noqa: E402

# Bracos do trote: um pouco mais de balanco e cotovelo mais dobrado que no
# strafe (a referencia do trote dobra ate 44 graus).
TROT_ARM_SWING_SHARE = 0.45
TROT_ARM_ELBOW = 76.0
TROT_ARM_ELBOW_SHARE = 0.35
# Sprint: quanto a cabeca sobe (graus) - na referencia ela vai 17 graus
# inclinada para a frente, olhando o chao.
SPRINT_HEAD_RAISE = 11.0
# Diagonal de costas rapida: cadencia (a mesma passada, mais rapida).
BACKWARD_SPRINT_CADENCE = 1.45
# Pe mais inclinado que isto (graus): o joelho fica no polo da referencia.
# No balanco do trote o calcanhar sobe atras e o pe fica quase vertical; o
# rumo da ponta do pe (que o IK acerta girando o joelho) vira ruido, e o
# joelho da direita cruzava para a esquerda - as coxas se encostavam 8 cm.
SWIVEL_PITCH = 35.0
# Quadros gravados por quadro da referencia. A referencia tem 30 quadros/s;
# entre eles os alvos (pelve, juntas, tornozelos, pe plantado, correcoes)
# seguem uma curva Catmull-Rom periodica e as pernas sao refeitas por IK. Com
# a reta do Clip.sample a velocidade quebrava a cada quadro e, a 120 Hz, o pe
# do sprint dava picos de 1200 m/s2; a curva direto nas juntas tirava o pe
# plantado do lugar entre os quadros (8 cm).
UPSAMPLE = 4


def _spline(values, k: int, a: float, n: int):
    """Catmull-Rom periodica entre os quadros k e k + 1 (a em 0..1)."""
    p0, p1, p2, p3 = (np.asarray(values[(k + j) % n], dtype=float) for j in (-1, 0, 1, 2))
    return 0.5 * (2 * p1 + (p2 - p0) * a + (2 * p0 - 5 * p1 + 4 * p2 - p3) * a * a
                  + (3 * p1 - p0 - 3 * p2 + p3) * a ** 3)


def _spline_quaternion(values, k: int, a: float, n: int):
    keys = [np.asarray(values[(k + j) % n], dtype=float) for j in (-1, 0, 1, 2)]
    keys = [q if q @ keys[1] >= 0 else -q for q in keys]
    return qnormalize(_spline(keys, 1, a, 4))


class ReferenceGait(base.Strafe):
    """Uma passada retargeteada (o ciclo fechado do arquivo), girada para o
    rumo pedido, com as correcoes do strafe."""

    def __init__(self, rig: Rig, source: str, heading: float, keep_arms: bool = False,
                 head_raise: float = 0.0, plant_at_landing: bool = False,
                 arms_from_legs: float = 0.0, arm_swing_share: float = TROT_ARM_SWING_SHARE,
                 arm_elbow: float = TROT_ARM_ELBOW, blend_frames: int = base.BLEND_FRAMES,
                 reach_margin: float = base.REACH_MARGIN, stance_pitch: float = base.STANCE_PITCH):
        self.source = source
        # Pe mais inclinado que isto (graus) no apoio da referencia rola em vez
        # de plantar.
        self.stance_pitch = stance_pitch
        # Quadros da volta do pe plantado para a referencia e a folga minima
        # do alcance da perna de apoio (m).
        self.blend_frames = blend_frames
        self.reach_margin = reach_margin
        self.plant_at_landing = plant_at_landing
        # Bracos pelas pernas (a referencia segura algo - o fuzil do Sprint
        # Backward - e nao balanca): cada braco vai a frente com a coxa do
        # outro lado, esta fracao do angulo dela.
        self.arms_from_legs = arms_from_legs
        self.arm_swing_share = arm_swing_share
        self.arm_elbow = arm_elbow
        self.target_heading = heading
        self.keep_arms = keep_arms
        self.head_raise = head_raise
        super().__init__(rig, "Left")
        # Os contatos do apoio pelo lado da referencia (o espelho troca os
        # nomes de self.contacts, nao os dos alvos).
        self.plant_contacts = self.contacts

    def _load(self) -> None:
        path = base.SOURCES / f"{self.source}.retarget.json"
        clip = load_clip(path, self.rig)
        displacement = np.array(json.loads(path.read_text(encoding="utf-8"))
                                ["sourceRootDisplacementMeters"])
        # O retarget fecha o ciclo: o ultimo quadro repete o primeiro.
        frames = len(clip.times) - 1
        period = frames / clip.sample_rate
        offsets = clip.root_offsets[:frames].copy()
        drift = (clip.root_offsets[frames] - offsets[0]) * np.array([1.0, 1.0, 0.0])
        offsets -= np.outer(np.arange(frames) / frames, drift)
        velocity = displacement / clip.duration + drift / period
        velocity[2] = 0.0
        heading = math.degrees(math.atan2(velocity[1], velocity[0]))
        self.heading_correction = self.target_heading - heading
        turn = base._rz(self.heading_correction)
        pelvis = self.rig.links[0].model_position
        self.offsets = np.array([qrotate(turn, pelvis + o) - pelvis for o in offsets])
        self.rotations = np.array([qnormalize(qmul(turn, r)) for r in clip.root_rotations[:frames]])
        self.coordinates = clip.coordinates[:frames].copy()
        self.contacts = {"Left": clip.left_foot_contact[:frames].copy(),
                         "Right": clip.right_foot_contact[:frames].copy()}
        self.frames = frames
        self.period = period
        self.speed = float(np.linalg.norm(velocity[:2]))
        angle = math.radians(self.target_heading)
        self.velocity = np.array([math.cos(angle) * self.speed, math.sin(angle) * self.speed, 0.0])
        self.reference = [self.rig.forward_kinematics(self.offsets[f], self.rotations[f], self.coordinates[f])
                          for f in range(frames)]

    def _plant_place(self, targets, side: str, core: list[int], dt: float):
        # No sprint o pe da referencia escorrega no apoio: parado na media,
        # ele pousava, andava 10 cm a 5 m/s e so entao plantava. Parado onde
        # pousou, o escorregao fica na saida (o pe ja rolando na ponta).
        if not self.plant_at_landing:
            return super()._plant_place(targets, side, core, dt)
        return targets[core[0] % self.frames][side].ankle + self.velocity * core[0] * dt

    def _legs(self, offset, rotation, coordinates, targets):
        builder = PoseBuilder(self.rig)
        builder.root_offset = np.array(offset, dtype=float)
        builder.root_rotation = np.array(rotation, dtype=float)
        builder.coordinates = coordinates.copy()
        for side in base.SIDES:
            t = targets[side]
            delta = qmul(t.foot, qconj(self.bind[self.index[f"{side}Foot"]]))
            forward = qrotate(delta, np.array([1.0, 0.0, 0.0]))
            yaw = math.degrees(math.atan2(forward[1], forward[0]))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, -forward[2]))))
            steep = abs(pitch) > SWIVEL_PITCH
            builder.leg(side, LegSpec(ankle=t.ankle, knee_direction=t.pole,
                                      toe_out_degrees=base._sign(side) * yaw,
                                      foot_pitch_degrees=pitch,
                                      ankle_follow=1.0 if steep else 0.0))
        pose = builder.solve()
        _, orientations = builder._forward()
        for side in base.SIDES:
            builder._solve_foot(side, targets[side].foot, orientations, pose)
        return builder.coordinates.copy()

    def fractional_pose(self, u: float, twist: float, lift: dict, arms: dict):
        """A pose no instante u (em quadros), com as correcoes dele: entre dois
        quadros, os alvos na curva e o corpo resolvido como num quadro (o pe
        plantado anda na reta dele)."""
        n = self.frames
        k = int(math.floor(u + 1e-6))
        a = u - k
        if a < 1e-6:
            return self.pose(k % n, twist, lift, arms)
        offset = _spline(self.offsets, k, a, n)
        rotation = _spline_quaternion(self.rotations, k, a, n)
        coordinates = _spline(self.coordinates, k, a, n)
        targets = {}
        for side in base.SIDES:
            ankles = [self.targets[i][side].ankle for i in range(n)]
            poles = [self.targets[i][side].pole for i in range(n)]
            feet = [self.targets[i][side].foot for i in range(n)]
            pole = _spline(poles, k, a, n)
            ankle = _spline(ankles, k, a, n)
            foot = _spline_quaternion(feet, k, a, n)
            # Entre dois quadros de apoio, a reta do pe plantado (a curva
            # puxaria pelo vizinho fora do apoio).
            if min(self.plant_contacts[side][k % n], self.plant_contacts[side][(k + 1) % n]) >= 0.99:
                ankle = (1 - a) * ankles[k % n] + a * ankles[(k + 1) % n]
                foot = _spline_quaternion([feet[k % n]] * 4, 1, 0.0, 4)
            targets[side] = base.Targets(ankle, pole / np.linalg.norm(pole), foot)
        # Um quadro a mais, no fim, so durante a montagem.
        saved = (self.offsets, self.rotations, self.coordinates, self.arm_swing, self.elbow_swing)
        self.offsets = np.vstack([self.offsets, offset])
        self.rotations = np.vstack([self.rotations, rotation])
        self.coordinates = np.concatenate([self.coordinates, coordinates[None]])
        self.arm_swing = {s: np.append(v, _spline(v, k, a, n)) for s, v in saved[3].items()}
        self.elbow_swing = {s: np.append(v, _spline(v, k, a, n)) for s, v in saved[4].items()}
        self.reference.append(self.rig.forward_kinematics(offset, rotation, coordinates))
        self.targets.append(targets)
        try:
            return self.pose(n, twist, lift, arms)
        finally:
            self.offsets, self.rotations, self.coordinates, self.arm_swing, self.elbow_swing = saved
            self.reference.pop()
            self.targets.pop()

    def _arm_rhythm(self) -> None:
        if not self.arms_from_legs:
            super()._arm_rhythm()
            return
        pitch = {}
        for side in base.SIDES:
            values = []
            for f in range(self.frames):
                positions, orientations = self.reference[f]
                thigh = (self.anchor(positions, orientations, f"{side}Shin")
                         - self.anchor(positions, orientations, f"{side}Thigh"))
                pelvis = qrotate(qconj(qmul(orientations[0], qconj(self.bind[0]))), thigh)
                values.append(math.degrees(math.atan2(pelvis[0], -pelvis[2])))
            pitch[side] = np.array(values) - np.mean(values)
        self.arm_swing = {"Left": self.arms_from_legs * pitch["Right"],
                          "Right": self.arms_from_legs * pitch["Left"]}
        self.elbow_swing = {side: np.zeros(self.frames) for side in base.SIDES}

    def _calm_arms(self, f: int, offset, rotation, coordinates):
        if self.keep_arms:
            return coordinates
        builder = PoseBuilder(self.rig)
        builder.root_offset = np.array(offset, dtype=float)
        builder.root_rotation = np.array(rotation, dtype=float)
        builder.coordinates = coordinates.copy()
        for side in base.SIDES:
            builder.arm(side, ArmSpec(
                lowered_degrees=RUNNING_ARM_LOWERED,
                forward_degrees=base.ARM_FORWARD + self.arm_swing_share * self.arm_swing[side][f],
                inward_roll_degrees=RUNNING_ARM_INWARD_ROLL,
                elbow_flexion_degrees=self.arm_elbow + TROT_ARM_ELBOW_SHARE * self.elbow_swing[side][f],
                wrist_flexion_degrees=RUNNING_WRIST_FLEXION))
        return builder.solve().coordinates.copy()

    def _steady_head(self, offset, rotation, coordinates):
        coordinates = super()._steady_head(offset, rotation, coordinates)
        if not self.head_raise:
            return coordinates
        # A cabeca mais erguida: pescoco 40%, cabeca o resto (para tras em
        # volta da esquerda de cada um).
        _, now = self.rig.forward_kinematics(offset, rotation, coordinates)
        parent = now[self.index["UpperChest"]]
        for link, share in (("Neck", 0.4), ("Head", 1.0)):
            i = self.index[link]
            left = qrotate(qmul(now[i], qconj(self.bind[i])), np.array([0.0, 1.0, 0.0]))
            world = qnormalize(qmul(qfrom_axis_angle(left, -math.radians(self.head_raise * share)), now[i]))
            coordinates[i] = self.rig.clamp(i, self.rig.coordinates_for(i, parent, world))
            _, now = self.rig.forward_kinematics(offset, rotation, coordinates)
            parent = now[i]
        return coordinates


class MirroredReferenceGait(ReferenceGait):
    """A passada espelhada no plano sagital (o lado direito da diagonal),
    com as correcoes procuradas ja na pose espelhada."""

    def __init__(self, rig: Rig, source: str, heading: float, **options):
        super().__init__(rig, source, heading, **options)
        self.contacts = {"Left": self.contacts["Right"], "Right": self.contacts["Left"]}

    def pose(self, f: int, twist: float = 0.0, lift=None, arms=None):
        lift = {base._mirror_name(k): v for k, v in (lift or {}).items()}
        arms = {base._mirror_name(k): v for k, v in (arms or {}).items()}
        return base.mirror_pose(self.rig, *ReferenceGait.pose(self, f, -twist, lift, arms))


def _gait(rig: Rig, kind, source: str, heading: float, **options):
    """A passada e as correcoes dela."""
    gait = kind(rig, source, heading, **options)
    return gait, base.corrections(gait)


def _legs_touch(pairs) -> bool:
    """Um pe encostando no pe ou na canela da outra perna."""
    def side(link: str) -> str:
        return "Left" if link.startswith("Left") else "Right"
    return any((a.endswith("Foot") or b.endswith("Foot")) and side(a) != side(b)
               and all(x.endswith(("Foot", "Shin")) for x in (a, b)) for a, b in pairs)


def _fine_contact(gait, side: str, j: int) -> float:
    """Contato na amostra j da grade fina: entre dois quadros, o menor."""
    n = gait.frames
    k, r = divmod(j, UPSAMPLE)
    c = gait.contacts[side]
    return float(c[k % n]) if r == 0 else float(min(c[k % n], c[(k + 1) % n]))


def _fine_corrections(gait, twist, lift, arms):
    """As correcoes na grade fina (UPSAMPLE por quadro): as dos quadros na
    curva e, entre eles, o pe em balanco erguido o minimo para nao encostar
    no pe ou na canela de apoio (a passagem entre dois quadros)."""
    n = gait.frames
    m = UPSAMPLE * n

    def at(values, j: int) -> float:
        k, r = divmod(j, UPSAMPLE)
        return float(values[k % n]) if r == 0 else float(_spline(values, k, r / UPSAMPLE, n))

    fine_twist = np.array([at(twist, j) for j in range(m)])
    fine_lift = {s: np.array([max(0.0, at(lift[s], j)) for j in range(m)]) for s in base.SIDES}
    fine_arms = {s: np.array([max(0.0, at(arms[s], j)) for j in range(m)]) for s in base.SIDES}

    def touching(j: int, extra: dict) -> bool:
        offset, rotation, coordinates = gait.fractional_pose(
            j / UPSAMPLE, fine_twist[j], {s: fine_lift[s][j] + extra.get(s, 0.0) for s in base.SIDES},
            {s: fine_arms[s][j] for s in base.SIDES})
        positions, orientations = gait.rig.forward_kinematics(offset, rotation, coordinates)
        return _legs_touch([(a, b) for a, b, gap, required in self_collision_gaps(gait.rig, positions, orientations)
                            if gap < required])

    need = {s: np.zeros(m) for s in base.SIDES}
    for j in range(m):
        if j % UPSAMPLE == 0 or not touching(j, {}):
            continue
        for side in base.SIDES:
            if _fine_contact(gait, side, j) >= 0.5:
                continue
            for step in range(1, int(round(base.MAXIMUM_LIFT / 0.005)) + 1):
                if not touching(j, {side: 0.005 * step}):
                    need[side][j] = 0.005 * step
                    break
    for side in base.SIDES:
        fine_lift[side] = fine_lift[side] + base._cover(need[side])
    return fine_twist, fine_lift, fine_arms


def _clip(rig: Rig, prepared, clip_id: str, name: str, travel: float,
          cadence: float = 1.0) -> ClipDefinition:
    gait, (twist, lift, arms) = prepared
    n = gait.frames
    m = UPSAMPLE * n
    duration = gait.period / cadence
    fine_twist, fine_lift, fine_arms = _fine_corrections(gait, twist, lift, arms)

    def pose_at(t: float) -> Pose:
        j = int(round(t / duration * m)) % m
        offset, rotation, coordinates = gait.fractional_pose(
            j / UPSAMPLE, fine_twist[j], {s: fine_lift[s][j] for s in base.SIDES},
            {s: fine_arms[s][j] for s in base.SIDES})
        return Pose(np.array(offset), np.array(rotation), np.array(coordinates))

    def contact_at(t: float):
        j = int(round(t / duration * m)) % m
        return tuple(_fine_contact(gait, side, j) for side in base.SIDES)

    return ClipDefinition(
        id=clip_id, display_name=name, duration=duration, loops=True,
        pose_at=pose_at, contact_at=contact_at, sample_rate=UPSAMPLE * n / duration,
        nominal_speed=round(gait.speed * cadence, 4),
        travel_direction_degrees=travel, floor_z=gait.floor,
        source="tools/animation/clips/trot.py")


def definitions(rig: Rig) -> list[ClipDefinition]:
    result = [
        _clip(rig, _gait(rig, ReferenceGait, "trot_forward", 0.0), "trot_forward", "Trote", 0.0),
        _clip(rig, _gait(rig, ReferenceGait, "trot_backward", 180.0), "trot_backward", "Trote de costas", 180.0),
        _clip(rig, _gait(rig, ReferenceGait, "trot_forward_diagonal", 45.0), "trot_forward_left",
              "Trote diagonal (esquerda)", 45.0),
        _clip(rig, _gait(rig, MirroredReferenceGait, "trot_forward_diagonal", 45.0), "trot_forward_right",
              "Trote diagonal (direita)", -45.0),
        _clip(rig, _gait(rig, ReferenceGait, "running_reference", 0.0, keep_arms=True,
                         head_raise=SPRINT_HEAD_RAISE, plant_at_landing=True), "sprint_forward", "Sprint", 0.0),
    ]
    for side, mirrored, travel in (("left", False, 135.0), ("right", True, -135.0)):
        gait_type = MirroredReferenceGait if mirrored else ReferenceGait
        prepared = _gait(rig, gait_type, "trot_backward_diagonal", 135.0)
        where = "esquerda" if side == "left" else "direita"
        result.append(_clip(rig, prepared, f"trot_backward_{side}", f"Trote de costas diagonal ({where})", travel))
        result.append(_clip(rig, prepared, f"sprint_backward_{side}", f"Sprint de costas diagonal ({where})",
                            travel, cadence=BACKWARD_SPRINT_CADENCE))
    return result
