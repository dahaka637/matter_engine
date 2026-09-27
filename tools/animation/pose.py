"""Linguagem de autoria de pose.

Autorar em coordenadas cruas de junta e fragil: alguns eixos do ALS nao sao
espelhados entre os lados (swing2 da coxa move as DUAS pernas para a direita;
swing2 do braco leva o esquerdo para tras e o direito para a frente - ver
axis_atlas.py). Aqui a pose e descrita como alguem descreveria um corpo:

  - coluna, pescoco e cabeca em graus anatomicos (flexao, inclinacao, giro).
    Sao linha media, o sinal nao depende de lado;
  - bracos por anatomia: quanto o braco desce, vai para frente, gira; quanto o
    cotovelo dobra. O lado espelha sozinho;
  - pernas por IK de dois ossos: onde o tornozelo fica no chao, para onde o
    joelho aponta, quanto o pe abre. E o que mantem o pe plantado enquanto a
    pelve se move - sem isso, o proprio idle faria o pe deslizar.

Tudo em espaco de modelo do ALS: olha para +X, esquerda +Y, cima +Z.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from matter_rig import (Rig, qconj, qfrom_axis_angle, qmul, qnormalize, qrotate,
                        quat)

SIDES = ("Left", "Right")


def _sign(side: str) -> float:
    """+1 para o lado esquerdo (+Y), -1 para o direito."""
    return 1.0 if side == "Left" else -1.0


def quat_from_matrix(m: np.ndarray) -> np.ndarray:
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        return qnormalize(np.array([(m[2, 1] - m[1, 2]) / s,
                                    (m[0, 2] - m[2, 0]) / s,
                                    (m[1, 0] - m[0, 1]) / s, 0.25 * s]))
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return qnormalize(np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s,
                                    (m[0, 2] + m[2, 0]) / s,
                                    (m[2, 1] - m[1, 2]) / s]))
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return qnormalize(np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s,
                                    (m[1, 2] + m[2, 1]) / s,
                                    (m[0, 2] - m[2, 0]) / s]))
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return qnormalize(np.array([(m[0, 2] + m[2, 0]) / s,
                                (m[1, 2] + m[2, 1]) / s, 0.25 * s,
                                (m[1, 0] - m[0, 1]) / s]))


def _frame(primary: np.ndarray, reference: np.ndarray) -> np.ndarray:
    a = primary / np.linalg.norm(primary)
    b = reference - a * (reference @ a)
    b /= np.linalg.norm(b)
    return np.stack([a, b, np.cross(a, b)], axis=1)


def aligned_rotation(from_primary, to_primary, from_reference,
                     to_reference) -> np.ndarray:
    """Rotacao que leva o eixo primario ao alvo e alinha o secundario o mais
    perto possivel da referencia (o secundario controla o giro do segmento)."""
    source = _frame(np.asarray(from_primary, float), np.asarray(from_reference, float))
    target = _frame(np.asarray(to_primary, float), np.asarray(to_reference, float))
    return quat_from_matrix(target @ source.T)


def _nlerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    if float(a @ b) < 0.0:
        b = -b
    return qnormalize(a * (1.0 - t) + b * t)


def degrees_vector(*components: float) -> np.ndarray:
    return np.radians(np.array(components, dtype=np.float64))


@dataclass
class Pose:
    root_offset: np.ndarray
    root_rotation: np.ndarray
    coordinates: np.ndarray            # (N, 3) radianos
    # Diagnostico do solver: juntas em que a geometria pedida nao coube num
    # eixo habilitado (residuo em graus).
    residuals: dict[str, float] = field(default_factory=dict)


@dataclass
class ArmSpec:
    # Quanto o braco desce a partir da pose em T (90 = colado ao lado).
    lowered_degrees: float = 80.0
    # Frente (+) / tras (-), no plano sagital.
    forward_degrees: float = 0.0
    # Giro do braco em torno de si; + leva a frente do braco para dentro.
    inward_roll_degrees: float = 0.0
    elbow_flexion_degrees: float = 10.0
    # Antebraco: + prona (palma para tras), - supina (palma para frente).
    pronation_degrees: float = 0.0
    wrist_flexion_degrees: float = 0.0
    wrist_deviation_degrees: float = 0.0


@dataclass
class LegSpec:
    # Tornozelo alvo em espaco de modelo (o chao do bind e a altura do pe no
    # modelo; ver PoseBuilder.floor_ankle_height).
    ankle: np.ndarray
    # Para onde o joelho aponta (vetor aproximado, espaco de modelo).
    knee_direction: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    # Abertura do pe (+ = ponta para fora) e inclinacao (+ = ponta para baixo).
    toe_out_degrees: float = 0.0
    foot_pitch_degrees: float = 0.0
    # 0 = pe orientado no mundo (toe_out/pitch acima); 1 = pe acompanhando a
    # canela, com o tornozelo em ankle_flexion_degrees (+ = ponta para baixo).
    # Na recuperacao de uma corrida a canela fica quase horizontal: um pe
    # "chato no mundo" pediria 90 graus de tornozelo.
    ankle_follow: float = 0.0
    ankle_flexion_degrees: float = 0.0


class PoseBuilder:
    """Monta uma pose a partir de uma descricao anatomica."""

    def __init__(self, rig: Rig):
        self.rig = rig
        zero = np.zeros((len(rig), 3))
        self.bind_positions, self.bind_orientations = rig.forward_kinematics(
            np.zeros(3), quat(), zero)
        self.root_offset = np.zeros(3)
        self.root_rotation = quat()
        self.coordinates = zero.copy()
        self.arms: dict[str, ArmSpec] = {}
        self.legs: dict[str, LegSpec] = {}

    # --- pontos de referencia no bind ---------------------------------------
    def bind_anchor(self, link_id: str) -> np.ndarray:
        return self.rig.links[self.rig.index(link_id)].anchor.copy()

    def floor_ankle_height(self, side: str = "Left") -> float:
        """Altura do tornozelo acima da sola, no bind (pe plano)."""
        from validate import lowest_point
        sole = lowest_point(self.rig, self.bind_positions,
                            self.bind_orientations, [f"{side}Foot"])
        return float(self.bind_anchor(f"{side}Foot")[2] - sole)

    # --- descricao ------------------------------------------------------------
    def root(self, offset=(0.0, 0.0, 0.0), pitch_degrees: float = 0.0,
             roll_degrees: float = 0.0, yaw_degrees: float = 0.0) -> None:
        """Pelve: deslocamento e orientacao em relacao ao bind.
        pitch + inclina para a frente, roll + tomba para a direita."""
        self.root_offset = np.asarray(offset, dtype=np.float64)
        q = qfrom_axis_angle([0, 0, 1], math.radians(yaw_degrees))
        q = qmul(q, qfrom_axis_angle([0, 1, 0], math.radians(pitch_degrees)))
        q = qmul(q, qfrom_axis_angle([1, 0, 0], math.radians(roll_degrees)))
        self.root_rotation = qnormalize(q)

    def spine(self, flexion: float = 0.0, lateral: float = 0.0,
              axial: float = 0.0,
              weights=(0.30, 0.35, 0.35)) -> None:
        """Coluna inteira: flexao (+ para frente), inclinacao (+ para a
        esquerda), giro (+ para a esquerda), distribuidos entre Abdomen,
        Chest e UpperChest."""
        for link, w in zip(("Abdomen", "Chest", "UpperChest"), weights):
            # twist + gira a frente do segmento para a esquerda, como no
            # pescoco (medido pela FK; o atlas mostra a ancora do filho, que
            # fica atras do eixo e por isso anda para a direita).
            self.coordinates[self.rig.index(link)] = degrees_vector(
                axial * w, flexion * w, lateral * w)

    def head(self, nod: float = 0.0, tilt: float = 0.0, turn: float = 0.0,
             neck_share: float = 0.4) -> None:
        """nod + baixa o queixo, tilt + inclina para a esquerda, turn + vira
        para a esquerda. O pescoco leva neck_share, a cabeca o resto."""
        for link, share in (("Neck", neck_share), ("Head", 1.0 - neck_share)):
            self.coordinates[self.rig.index(link)] = degrees_vector(
                turn * share, nod * share, tilt * share)

    def arm(self, side: str, spec: ArmSpec) -> None:
        self.arms[side] = spec

    def leg(self, side: str, spec: LegSpec) -> None:
        self.legs[side] = spec

    # --- solucao ----------------------------------------------------------------
    def _forward(self) -> tuple[np.ndarray, np.ndarray]:
        return self.rig.forward_kinematics(self.root_offset,
                                           self.root_rotation,
                                           self.coordinates)

    def _set_world(self, link_id: str, world: np.ndarray,
                   positions, orientations, pose: Pose) -> None:
        """Coordenada que leva o link a esta orientacao de mundo."""
        index = self.rig.index(link_id)
        parent = self.rig.links[index].parent
        raw = self.rig.coordinates_for(index, orientations[parent], world)
        clamped = self.rig.clamp(index, raw)
        residual = math.degrees(float(np.linalg.norm(raw - clamped)))
        if residual > 0.5:
            pose.residuals[link_id] = residual
        self.coordinates[index] = clamped

    def _solve_arm(self, side: str, spec: ArmSpec, pose: Pose) -> None:
        s = _sign(side)
        positions, orientations = self._forward()
        chest = self.rig.index("UpperChest")
        # Espaco do tronco: a rotacao do UpperChest desde o bind. O braco
        # segue o tronco, como segue num corpo.
        torso = qmul(orientations[chest], qconj(self.bind_orientations[chest]))

        # Em T o braco aponta para o lado, (0, s, 0), com a frente anterior em
        # +X. Tres rotacoes, nesta ordem:
        #   1. abaixar no plano frontal, em torno de +X: angulo -s * lowered
        #      leva (0, s, 0) a (0, s cos, -sin);
        #   2. ir para frente no plano sagital, em torno de +Y: angulo
        #      -forward leva "para baixo" para a frente;
        #   3. girar o braco em torno de si mesmo: angulo +s * roll leva a
        #      frente do braco para a linha media, nos dois lados.
        lower = qfrom_axis_angle([1.0, 0.0, 0.0],
                                 -s * math.radians(spec.lowered_degrees))
        flex = qfrom_axis_angle([0.0, 1.0, 0.0],
                                -math.radians(spec.forward_degrees))
        placement = qmul(flex, lower)
        direction = qrotate(placement, np.array([0.0, s, 0.0]))
        roll = qfrom_axis_angle(direction,
                                s * math.radians(spec.inward_roll_degrees))
        world_local = qmul(roll, placement)

        upper = f"{side}UpperArm"
        bind_upper = self.bind_orientations[self.rig.index(upper)]
        world = qnormalize(qmul(torso, qmul(world_local, bind_upper)))
        self._set_world(upper, world, positions, orientations, pose)

        forearm = self.rig.index(f"{side}Forearm")
        hand = self.rig.index(f"{side}Hand")
        # Cotovelo: hinge no eixo "twist" do antebraco (+ = flexao nos dois
        # lados); pronacao no swing1. Punho: twist espelhado entre os lados,
        # swing1 + desvia nos dois (axis_atlas).
        self.coordinates[forearm] = self.rig.clamp(forearm, degrees_vector(
            spec.elbow_flexion_degrees, spec.pronation_degrees, 0.0))
        self.coordinates[hand] = self.rig.clamp(hand, degrees_vector(
            -s * spec.wrist_flexion_degrees, spec.wrist_deviation_degrees, 0.0))

    def _solve_leg(self, side: str, spec: LegSpec, pose: Pose) -> None:
        s = _sign(side)
        thigh, shin, foot = (self.rig.index(f"{side}{part}")
                             for part in ("Thigh", "Shin", "Foot"))
        hip_bind = self.rig.links[thigh].anchor
        knee_bind = self.rig.links[shin].anchor
        ankle_bind = self.rig.links[foot].anchor
        upper_length = float(np.linalg.norm(knee_bind - hip_bind))
        lower_length = float(np.linalg.norm(ankle_bind - knee_bind))

        positions, orientations = self._forward()
        hip = positions[0] + qrotate(orientations[0], qrotate(
            qconj(self.rig.links[0].model_orientation),
            hip_bind - self.rig.links[0].model_position))

        target = np.asarray(spec.ankle, dtype=np.float64)
        reach = target - hip
        distance = float(np.linalg.norm(reach))
        maximum = upper_length + lower_length - 1e-4
        if distance > maximum:
            pose.residuals[f"{side}Leg alcance"] = (distance - maximum) * 1000
            target = hip + reach * (maximum / distance)
            reach = target - hip
            distance = maximum
        axis = reach / distance
        # Lei dos cossenos: distancia do quadril ate a projecao do joelho.
        along = (upper_length ** 2 - lower_length ** 2 + distance ** 2) / (
            2.0 * distance)
        height = math.sqrt(max(upper_length ** 2 - along ** 2, 0.0))
        base_pole = np.asarray(spec.knee_direction, dtype=np.float64)
        base_pole = base_pole - axis * (base_pole @ axis)
        base_pole /= np.linalg.norm(base_pole)

        # O tornozelo do ALS nao gira o pe em torno da vertical (swing2
        # desligado): o rumo do pe e o da canela. Com a perna varrendo de
        # lado (passo lateral) a canela gira e o pe iria junto - medido, 4
        # graus num apoio, o que arrasta o pe no chao. Como numa pessoa, quem
        # corrige e o giro do joelho em torno da linha quadril-tornozelo: o
        # polo gira ate a ponta do pe apontar para onde foi pedido.
        wanted_yaw = s * math.radians(spec.toe_out_degrees)
        swivel = 0.0
        for _ in range(6):
            trial = Pose(pose.root_offset, pose.root_rotation, pose.coordinates)
            pole = qrotate(qfrom_axis_angle(axis, swivel), base_pole)
            self._place_leg(side, spec, hip, target, axis, along, height, pole,
                            trial)
            if spec.ankle_follow >= 0.5:
                break
            forward = qrotate(self._forward()[1][foot],
                              np.array([1.0, 0.0, 0.0]))
            error = math.atan2(math.sin(wanted_yaw - math.atan2(
                forward[1], forward[0])), math.cos(wanted_yaw - math.atan2(
                    forward[1], forward[0])))
            if abs(error) < math.radians(0.1):
                break
            # O eixo quadril-tornozelo aponta para baixo: girar +swivel em
            # torno dele leva a ponta do pe para a direita.
            swivel = min(max(swivel - error, -0.6), 0.6)
        pose.residuals.update(trial.residuals)

    def _place_leg(self, side: str, spec: LegSpec, hip, target, axis, along,
                   height, pole, pose: Pose) -> None:
        s = _sign(side)
        thigh, shin, foot = (self.rig.index(f"{side}{part}")
                             for part in ("Thigh", "Shin", "Foot"))
        hip_bind = self.rig.links[thigh].anchor
        knee_bind = self.rig.links[shin].anchor
        ankle_bind = self.rig.links[foot].anchor
        upper_length = float(np.linalg.norm(knee_bind - hip_bind))
        lower_length = float(np.linalg.norm(ankle_bind - knee_bind))
        knee = hip + axis * along + pole * height

        # Coxa e canela: eixo do osso para o alvo, "frente" (rotula) para o
        # polo. Mantem o joelho como dobradica pura.
        bind_thigh_axis = (knee_bind - hip_bind) / upper_length
        bind_shin_axis = (ankle_bind - knee_bind) / lower_length
        forward = np.array([1.0, 0.0, 0.0])
        thigh_world = qmul(aligned_rotation(bind_thigh_axis, knee - hip,
                                            forward, pole),
                           self.bind_orientations[thigh])
        shin_world = qmul(aligned_rotation(bind_shin_axis, target - knee,
                                           forward, pole),
                          self.bind_orientations[shin])
        positions, orientations = self._forward()
        self._set_world(f"{side}Thigh", qnormalize(thigh_world), positions,
                        orientations, pose)
        positions, orientations = self._forward()
        self._set_world(f"{side}Shin", qnormalize(shin_world), positions,
                        orientations, pose)
        positions, orientations = self._forward()

        # Pe: plano no chao, aberto pelo toe-out, inclinado pelo pitch.
        yaw = qfrom_axis_angle([0, 0, 1], s * math.radians(spec.toe_out_degrees))
        pitch = qfrom_axis_angle([0, 1, 0], math.radians(spec.foot_pitch_degrees))
        foot_world = qnormalize(qmul(qmul(yaw, pitch),
                                     self.bind_orientations[foot]))
        if spec.ankle_follow > 0.0:
            # Pe preso a canela: a mesma rotacao que levou a canela desde o
            # bind, mais a flexao pedida do tornozelo em torno do eixo lateral
            # da canela.
            shin_delta = qmul(orientations[shin],
                              qconj(self.bind_orientations[shin]))
            lateral = qrotate(shin_delta, qrotate(yaw, np.array([0.0, 1.0, 0.0])))
            following = qnormalize(qmul(qmul(qfrom_axis_angle(
                lateral, math.radians(spec.ankle_flexion_degrees)),
                shin_delta), self.bind_orientations[foot]))
            foot_world = _nlerp(foot_world, following, spec.ankle_follow)
        self._solve_foot(side, foot_world, orientations, pose)

    def _solve_foot(self, side: str, target: np.ndarray, orientations,
                    pose: Pose) -> None:
        """Orienta o pe com os dois eixos que o tornozelo tem.

        O ALS nao tem o terceiro eixo no tornozelo (swing2 desligado). Com a
        canela inclinada e a perna um pouco aberta, um pe "chato e apontando
        para a frente" pede 2-4 graus nesse eixo e sai recortado - torto, com
        a quina da sola fora do chao. Aqui a prioridade e a sola (o vetor
        normal a ela, que decide o contato com o chao); a direcao da ponta
        vem em segundo e pode desviar alguns graus.
        """
        index = self.rig.index(f"{side}Foot")
        parent_world = orientations[self.rig.links[index].parent]
        link = self.rig.links[index]
        free = [a for a in range(3) if link.axes[a].enabled]
        up_target = qrotate(target, np.array([0.0, 0.0, 1.0]))
        forward_target = qrotate(target, np.array([1.0, 0.0, 0.0]))

        def residual(c: np.ndarray) -> np.ndarray:
            world = qmul(parent_world, self.rig.joint_local_orientation(
                index, c))
            up = qrotate(world, np.array([0.0, 0.0, 1.0]))
            forward = qrotate(world, np.array([1.0, 0.0, 0.0]))
            return np.concatenate([up - up_target,
                                   0.25 * (forward - forward_target)])

        c = self.rig.clamp(index, self.rig.coordinates_for(
            index, parent_world, target))
        for _ in range(10):
            r = residual(c)
            jacobian = np.zeros((len(r), len(free)))
            for column, axis in enumerate(free):
                probe = c.copy()
                probe[axis] += 1e-5
                jacobian[:, column] = (residual(probe) - r) / 1e-5
            step = np.linalg.lstsq(jacobian, -r, rcond=None)[0]
            for column, axis in enumerate(free):
                c[axis] += step[column]
            c = self.rig.clamp(index, c)
            if np.linalg.norm(step) < 1e-7:
                break
        world = qmul(parent_world, self.rig.joint_local_orientation(index, c))
        up = qrotate(world, np.array([0.0, 0.0, 1.0]))
        tilt = math.degrees(math.acos(min(1.0, float(up @ up_target))))
        if tilt > 0.5:
            pose.residuals[f"{side}Foot sola"] = tilt
        self.coordinates[index] = c

    def solve(self) -> Pose:
        pose = Pose(self.root_offset.copy(), self.root_rotation.copy(),
                    self.coordinates)
        for side in SIDES:
            if side in self.legs:
                self._solve_leg(side, self.legs[side], pose)
        for side in SIDES:
            if side in self.arms:
                self._solve_arm(side, self.arms[side], pose)
        pose.coordinates = self.coordinates.copy()
        return pose
