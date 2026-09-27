"""Strafe - correr de lado olhando para a frente, fiel as referencias.

A pedido do usuario ("o mais proximo possivel do modelo"), o strafe NAO e
gerado: e o proprio Jog Strafe Left/Right.fbx retargeteado para o nosso rig
(tools/import_humanoid_animation.py; os retargets crus ficam em
assets/animations/source/retargeted). Aqui so entra o que o nosso corpo
exige, e sempre a menor mudanca que resolve:

  - EMENDA: o retarget da direita termina num quadro de pausa em que a raiz
    volta 4,5 cm de uma vez; o ciclo fecha no quadro anterior e a deriva
    que sobra vira velocidade. O ciclo e girado para andar exatamente a +-90
    graus (a referencia desvia menos de 1 grau).
  - PE DE APOIO: o pe Mixamo tem dedos; o nosso e uma caixa rigida. No fim
    do apoio a referencia ergue o calcanhar com os dedos no chao, e a nossa
    caixa afundaria 4 cm; e o pe de apoio da referencia escorrega 5-9 cm.
    Durante o contato o pe fica plano e parado no mundo, onde a referencia o
    pos em media; o joelho absorve. Nos quadros vizinhos a pose volta a da
    referencia aos poucos.
  - CHAO: nenhum pe abaixo do piso; se a referencia afunda, o tornozelo sobe
    o necessario.
  - COXAS: na passada cruzada as coxas se encostam no alto, perto do
    quadril. Nesses quadros a pelve gira alguns graus a mais (um quadril vai
    para tras do outro); a coluna devolve o giro, o tronco e o olhar nao
    mudam, e as pernas sao refeitas por IK ate os mesmos pes.
  - PES e BRACOS: se o pe em balanco rela no de apoio, ele sobe o minimo;
    se o braco encosta no tronco, ele abre o minimo.
  - CABECA: olha para a frente em todo quadro (o personagem olha para onde
    a camera aponta); na referencia ela varre 23 graus com o tronco.
  - BRACOS: a referencia balanca os bracos largos, cruzando o corpo - o
    usuario achou "muito espalhafatoso". Aqui eles ficam na postura padrao
    de corrida (gait.RUNNING_ARM_*: junto ao corpo, cotovelo dobrado, mao
    relaxada) e balancam no tempo da referencia (o braco acompanha as
    pernas), com ARM_SWING_SHARE da amplitude dela.

O que fica da referencia, de proposito: a pelve e o tronco girados para o
lado de tras (-20 e -13 graus em media no strafe para a esquerda, +9 e -7 no
para a direita). E o estilo do Jog Strafe - a perna de tras cruza por tras
da outra, e o quadril dela vai junto.

As correcoes sao suavizadas no ciclo. O sprint de lado (pedido do usuario:
"correr rapido andando para o lado") e o mesmo movimento com cadencia maior.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

import numpy as np

from authoring import ClipDefinition
from gait import (RUNNING_ARM_INWARD_ROLL, RUNNING_ARM_LOWERED,
                  RUNNING_WRIST_FLEXION)
from matter_clip import Clip, load_clip
from matter_rig import (REPO_ROOT, Rig, qconj, qfrom_axis_angle, qmul,
                        qnormalize, qrotate)
from pose import ArmSpec, LegSpec, Pose, PoseBuilder
from validate import lowest_point, self_collision_gaps

SOURCES = REPO_ROOT / "assets" / "animations" / "source" / "retargeted"
SIDES = ("Left", "Right")
# Quadros do ciclo em cada retarget e se a emenda precisa ser extrapolada:
# o da esquerda repete o primeiro quadro no fim; o da direita tem ainda um
# quadro de pausa com a raiz fora do lugar antes dessa repeticao.
CYCLES = {"Left": (21, False), "Right": (22, True)}
BLEND_FRAMES = 3              # volta do pe plantado para a referencia
STANCE_PITCH = 20.0           # graus: mais inclinado que isto, o pe rola
MAXIMUM_STANCE_PITCH = 15.0   # pe parado na ponta, no maximo
ROLLING_CONTACT = 0.45
REACH_MARGIN = 0.01           # joelho de apoio nunca travado reto
SPRINT_CADENCE = 1.4          # sprint de lado: a mesma passada, mais rapida
# Giro extra da pelve devolvido pela coluna: fracao que cada segmento ainda
# carrega (o peito superior volta a orientacao da referencia).
SPINE = (("Abdomen", 0.70), ("Chest", 0.35), ("UpperChest", 0.0))
MAXIMUM_TWIST = 25            # graus
MAXIMUM_LIFT = 0.08           # metros
MAXIMUM_ARM = 30              # graus
ARM_PARTS = ("UpperArm", "Forearm", "Hand")
# Bracos de lado: postura padrao de corrida, balanco no tempo da referencia.
ARM_SWING_SHARE = 0.3         # da amplitude da referencia (frente/tras)
ARM_FORWARD = -12.0           # graus: o braco vive um pouco atras do tronco
# Cotovelo medio. Longe de 90 graus: parado perto de 90 (82-88 com 85) o
# antebraco do ALS gira em volta do braco no corpo fisico - medido, 86 graus
# fora do alvo no strafe para a esquerda. Nas passadas o cotovelo so passa
# por 90, balancando 44 graus.
ARM_ELBOW = 72.0
ARM_ELBOW_SHARE = 0.3         # da variacao do cotovelo da referencia


def _sign(side: str) -> float:
    return 1.0 if side == "Left" else -1.0


def _rz(degrees: float) -> np.ndarray:
    return qfrom_axis_angle([0.0, 0.0, 1.0], math.radians(degrees))


def _slerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    if np.dot(a, b) < 0:
        b = -b
    angle = math.acos(min(1.0, abs(float(np.dot(a, b)))))
    if angle < 1e-6:
        return a.copy()
    return qnormalize((math.sin((1 - t) * angle) * a
                       + math.sin(t * angle) * b) / math.sin(angle))


@dataclass
class Targets:
    """Pe pedido: tornozelo, polo do joelho e orientacao do pe no mundo."""
    ankle: np.ndarray
    pole: np.ndarray
    foot: np.ndarray


class Strafe:
    def __init__(self, rig: Rig, side: str):
        self.rig = rig
        self.side = side
        self.index = {link.id: i for i, link in enumerate(rig.links)}
        builder = PoseBuilder(rig)
        self.bind = builder.bind_orientations
        self.floor = lowest_point(rig, builder.bind_positions, self.bind,
                                  ["LeftFoot", "RightFoot"])
        anchors = {link.id: link.anchor for link in rig.links}
        self.leg_length = {s: float(
            np.linalg.norm(anchors[f"{s}Shin"] - anchors[f"{s}Thigh"])
            + np.linalg.norm(anchors[f"{s}Foot"] - anchors[f"{s}Shin"]))
            for s in SIDES}
        self._load()
        self.targets = self._plant()
        self._arm_rhythm()

    # --- referencia --------------------------------------------------------------
    def _load(self) -> None:
        path = SOURCES / f"jog_strafe_{self.side.lower()}.retarget.json"
        clip = load_clip(path, self.rig)
        displacement = np.array(json.loads(path.read_text(encoding="utf-8"))
                                ["sourceRootDisplacementMeters"])
        frames, extrapolate = CYCLES[self.side]
        period = frames / clip.sample_rate
        offsets = clip.root_offsets[:frames].copy()
        # Deriva da raiz no plano ao longo do ciclo: vira velocidade.
        closing = (2 * offsets[-1] - offsets[-2] if extrapolate
                   else clip.root_offsets[frames])
        drift = (closing - offsets[0]) * np.array([1.0, 1.0, 0.0])
        offsets -= np.outer(np.arange(frames) / frames, drift)
        velocity = displacement / clip.duration + drift / period
        velocity[2] = 0.0
        # Gira o ciclo inteiro para o deslocamento cair exatamente no lado.
        heading = math.degrees(math.atan2(velocity[1], velocity[0]))
        self.heading_correction = 90.0 * _sign(self.side) - heading
        turn = _rz(self.heading_correction)
        pelvis = self.rig.links[0].model_position
        self.offsets = np.array([qrotate(turn, pelvis + o) - pelvis
                                 for o in offsets])
        self.rotations = np.array([qnormalize(qmul(turn, r))
                                   for r in clip.root_rotations[:frames]])
        self.coordinates = clip.coordinates[:frames].copy()
        self.contacts = {"Left": clip.left_foot_contact[:frames].copy(),
                         "Right": clip.right_foot_contact[:frames].copy()}
        self.frames = frames
        self.period = period
        self.speed = float(np.linalg.norm(velocity[:2]))
        self.velocity = np.array([0.0, _sign(self.side) * self.speed, 0.0])
        self.reference = [self.rig.forward_kinematics(
            self.offsets[f], self.rotations[f], self.coordinates[f])
            for f in range(frames)]

    def anchor(self, positions, orientations, link_id: str) -> np.ndarray:
        i = self.index[link_id]
        link = self.rig.links[i]
        local = qrotate(qconj(link.model_orientation),
                        link.anchor - link.model_position)
        return positions[i] + qrotate(orientations[i], local)

    def reference_targets(self, f: int, side: str) -> Targets:
        positions, orientations = self.reference[f]
        hip = self.anchor(positions, orientations, f"{side}Thigh")
        knee = self.anchor(positions, orientations, f"{side}Shin")
        ankle = self.anchor(positions, orientations, f"{side}Foot")
        axis = (ankle - hip) / np.linalg.norm(ankle - hip)
        pole = (knee - hip) - axis * ((knee - hip) @ axis)
        return Targets(ankle, pole / np.linalg.norm(pole),
                       orientations[self.index[f"{side}Foot"]].copy())

    # --- pe de apoio -------------------------------------------------------------
    def windows(self, side: str) -> list[list[int]]:
        """Quadros de contato consecutivos, desenrolados (o tempo continua
        depois do fim do ciclo)."""
        down = self.contacts[side] >= 0.5
        start = int(np.argmin(down))          # um quadro sem contato
        windows, current = [], []
        for k in range(1, self.frames + 1):
            if down[(start + k) % self.frames]:
                current.append(start + k)
            elif current:
                windows.append(current)
                current = []
        if current:
            windows.append(current)
        return windows

    def _angles(self, orientation: np.ndarray, side: str):
        """(rumo, inclinacao + ponta para baixo, rolamento) do pe, graus."""
        delta = qmul(orientation, qconj(self.bind[self.index[f"{side}Foot"]]))
        forward = qrotate(delta, np.array([1.0, 0.0, 0.0]))
        up = qrotate(delta, np.array([0.0, 0.0, 1.0]))
        return (math.degrees(math.atan2(forward[1], forward[0])),
                math.degrees(math.asin(max(-1.0, min(1.0, -forward[2])))),
                math.degrees(math.atan2(up[1], up[2])))

    def _sole_depth(self, side: str, foot: np.ndarray) -> float:
        """Distancia do tornozelo a sola (ponto mais baixo) com o pe nesta
        orientacao."""
        i = self.index[f"{side}Foot"]
        link = self.rig.links[i]
        positions = np.zeros((len(self.rig), 3))
        orientations = np.tile(self.bind[0], (len(self.rig), 1))
        orientations[i] = foot
        positions[i] = -qrotate(foot, qrotate(qconj(link.model_orientation),
                                              link.anchor - link.model_position))
        return -lowest_point(self.rig, positions, orientations, [link.id])

    def _hip(self, side: str, f: int) -> np.ndarray:
        positions, orientations = self.reference[f]
        return self.anchor(positions, orientations, f"{side}Thigh")

    def _reach(self, side: str, f: int, ankle: np.ndarray) -> float:
        """Quanto o tornozelo pedido passa do alcance da perna (m)."""
        return float(np.linalg.norm(ankle - self._hip(side, f))
                     - self.leg_length[side])

    def _plant(self):
        """Alvos por quadro e contatos: a referencia, e no apoio o pe parado
        no mundo, sem rolar, com a sola no chao.

        O apoio da referencia inclui o toque e a saida, com o pe inclinado
        20-40 graus rolando sobre os dedos. A caixa rigida nao rola sobre
        dedos: esses quadros ficam como na referencia (so sem afundar) e
        viram "rolando" (contato 0,45: o runtime mantem a trava do pe se ja
        travou, mas nao trava nele). O pe parado fica no rumo e na inclinacao
        medianos da referencia (a da direita corre na ponta do pe, ~12
        graus), sem rolamento lateral, e onde ela o pos em media. Fora do
        apoio a diferenca para a referencia some em BLEND_FRAMES quadros.
        """
        n = self.frames
        dt = self.period / n
        targets = [{s: self.reference_targets(f, s) for s in SIDES}
                   for f in range(n)]
        contacts = {s: self.contacts[s].copy() for s in SIDES}
        identity = np.array([0.0, 0.0, 0.0, 1.0])
        for side in SIDES:
            bind = self.bind[self.index[f"{side}Foot"]]
            for window in self.windows(side):
                angles = [self._angles(targets[u % n][side].foot, side)
                          for u in window]
                core = [u for u, a in zip(window, angles)
                        if a[1] <= STANCE_PITCH]
                if not core:
                    core = [window[int(np.argmin([a[1] for a in angles]))]]
                core = list(range(core[0], core[-1] + 1))
                while True:
                    chosen = [a for u, a in zip(window, angles) if u in core]
                    yaw = float(np.median([a[0] for a in chosen]))
                    pitch = min(max(float(np.median([a[1] for a in chosen])),
                                    0.0), MAXIMUM_STANCE_PITCH)
                    foot = qnormalize(qmul(qmul(_rz(yaw), qfrom_axis_angle(
                        [0.0, 1.0, 0.0], math.radians(pitch))), bind))
                    place = np.median([targets[u % n][side].ankle
                                       + self.velocity * u * dt for u in core],
                                      axis=0)
                    place[2] = self.floor + self._sole_depth(side, foot)
                    # A referencia deixa o pe escorregar junto com o corpo;
                    # parado no meio, ele pode ficar fora do alcance da perna
                    # numa ponta do apoio. Essa ponta vira "rolando" (o pe
                    # sai do chao um quadro antes, ou toca um depois).
                    ends = [u for u in (core[0], core[-1]) if self._reach(
                        side, u % n, place - self.velocity * u * dt)
                        > -REACH_MARGIN]
                    if not ends or len(core) <= 2:
                        break
                    core = [u for u in core if u != ends[0]]

                def planted(u, place=place):
                    return place - self.velocity * u * dt

                for u in window:
                    contacts[side][u % n] = 1.0 if u in core else min(
                        contacts[side][u % n], ROLLING_CONTACT)
                reference = {u: self.reference_targets(u % n, side)
                             for u in range(core[0] - BLEND_FRAMES,
                                            core[-1] + BLEND_FRAMES + 1)}
                for u, ref in reference.items():
                    if core[0] <= u <= core[-1]:
                        targets[u % n][side] = Targets(planted(u), ref.pole,
                                                       foot)
                        continue
                    edge = core[0] if u < core[0] else core[-1]
                    w = 1.0 - abs(u - edge) / (BLEND_FRAMES + 1)
                    w = w * w * (3 - 2 * w)
                    turn = _slerp(identity, qmul(foot, qconj(
                        reference[edge].foot)), w)
                    targets[u % n][side] = Targets(
                        ref.ankle + w * (planted(edge) - reference[edge].ankle),
                        ref.pole, qnormalize(qmul(turn, ref.foot)))
        self.contacts = contacts
        return targets

    # --- bracos -------------------------------------------------------------------
    def _arm_rhythm(self) -> None:
        """Balanco dos bracos da referencia, em relacao ao tronco: quanto o
        braco vai a frente (graus, sem a media) e quanto o cotovelo dobra a
        mais que a media, por quadro."""
        chest = self.index["UpperChest"]
        self.arm_swing, self.elbow_swing = {}, {}
        for side in SIDES:
            forward, elbow = [], []
            for f in range(self.frames):
                positions, orientations = self.reference[f]
                torso = qmul(orientations[chest], qconj(self.bind[chest]))
                arm = (self.anchor(positions, orientations, f"{side}Forearm")
                       - self.anchor(positions, orientations, f"{side}UpperArm"))
                local = qrotate(qconj(torso), arm)
                forward.append(math.degrees(math.atan2(local[0], -local[2])))
                elbow.append(math.degrees(
                    self.coordinates[f][self.index[f"{side}Forearm"]][0]))
            self.arm_swing[side] = np.array(forward) - np.mean(forward)
            self.elbow_swing[side] = np.array(elbow) - np.mean(elbow)

    def _calm_arms(self, f: int, offset, rotation, coordinates):
        builder = PoseBuilder(self.rig)
        builder.root_offset = np.array(offset, dtype=float)
        builder.root_rotation = np.array(rotation, dtype=float)
        builder.coordinates = coordinates.copy()
        for side in SIDES:
            builder.arm(side, ArmSpec(
                lowered_degrees=RUNNING_ARM_LOWERED,
                forward_degrees=ARM_FORWARD
                + ARM_SWING_SHARE * self.arm_swing[side][f],
                inward_roll_degrees=RUNNING_ARM_INWARD_ROLL,
                elbow_flexion_degrees=ARM_ELBOW
                + ARM_ELBOW_SHARE * self.elbow_swing[side][f],
                wrist_flexion_degrees=RUNNING_WRIST_FLEXION))
        return builder.solve().coordinates.copy()

    # --- montagem ------------------------------------------------------------------
    def pose(self, f: int, twist: float = 0.0, lift=None, arms=None):
        """(offset, rotation, coordinates) do quadro f com as correcoes:
        giro extra da pelve (graus), pe erguido (m) e braco aberto (graus)."""
        lift = lift or {}
        arms = arms or {}
        offset = self.offsets[f].copy()
        rotation = self.rotations[f].copy()
        coordinates = self.coordinates[f].copy()
        _, orientations = self.reference[f]
        if twist:
            rotation = qnormalize(qmul(_rz(twist), rotation))
            parent = rotation
            for link, share in SPINE:
                i = self.index[link]
                world = qnormalize(qmul(_rz(twist * share), orientations[i]))
                coordinates[i] = self.rig.clamp(
                    i, self.rig.coordinates_for(i, parent, world))
                parent = world
        targets = {}
        for side in SIDES:
            t = self.targets[f][side]
            targets[side] = Targets(
                t.ankle + np.array([0.0, 0.0, lift.get(side, 0.0)]),
                t.pole, t.foot)
        coordinates = self._legs(offset, rotation, coordinates, targets)
        # Sola abaixo do piso: sobe o tornozelo o que falta e refaz (o giro
        # do joelho que acerta o rumo do pe muda um pouco a sola; poucas
        # voltas bastam).
        for _ in range(4):
            positions, now = self.rig.forward_kinematics(offset, rotation,
                                                         coordinates)
            raised = False
            for side in SIDES:
                sole = lowest_point(self.rig, positions, now, [f"{side}Foot"])
                if sole < self.floor:
                    targets[side].ankle = targets[side].ankle + np.array(
                        [0.0, 0.0, self.floor - sole + 0.0005])
                    raised = True
            if not raised:
                break
            coordinates = self._legs(offset, rotation, coordinates, targets)
        coordinates = self._calm_arms(f, offset, rotation, coordinates)
        if any(arms.values()):
            positions, now = self.rig.forward_kinematics(offset, rotation,
                                                         coordinates)
            chest = self.index["UpperChest"]
            forward = qrotate(qmul(now[chest], qconj(self.bind[chest])),
                              np.array([1.0, 0.0, 0.0]))
            for side, angle in arms.items():
                if not angle:
                    continue
                # Abre o braco no plano frontal do tronco: em torno da frente
                # do peito, + leva o braco esquerdo para a esquerda.
                i = self.index[f"{side}UpperArm"]
                world = qnormalize(qmul(qfrom_axis_angle(
                    forward, _sign(side) * math.radians(angle)), now[i]))
                coordinates[i] = self.rig.clamp(
                    i, self.rig.coordinates_for(i, now[chest], world))
        return offset, rotation, self._steady_head(offset, rotation,
                                                   coordinates)

    def _steady_head(self, offset, rotation, coordinates):
        """Cabeca olhando para a frente em todo quadro.

        Na referencia a cabeca acompanha o tronco e varre 23 graus por ciclo;
        no jogo o personagem olha para onde a camera aponta, e nas nossas
        passadas a cabeca fica parada no rumo (como a de quem corre olhando
        para algo). Pescoco (40%) e cabeca (60%) devolvem o rumo; a
        inclinacao da referencia fica."""
        _, now = self.rig.forward_kinematics(offset, rotation, coordinates)
        head = self.index["Head"]
        delta = qmul(now[head], qconj(self.bind[head]))
        forward = qrotate(delta, np.array([1.0, 0.0, 0.0]))
        yaw = math.degrees(math.atan2(forward[1], forward[0]))
        parent = now[self.index["UpperChest"]]
        for link, share in (("Neck", 0.4), ("Head", 1.0)):
            i = self.index[link]
            world = qnormalize(qmul(_rz(-yaw * share), now[i]))
            coordinates[i] = self.rig.clamp(
                i, self.rig.coordinates_for(i, parent, world))
            parent = world
        return coordinates

    def _legs(self, offset, rotation, coordinates, targets):
        builder = PoseBuilder(self.rig)
        builder.root_offset = np.array(offset, dtype=float)
        builder.root_rotation = np.array(rotation, dtype=float)
        builder.coordinates = coordinates.copy()
        for side in SIDES:
            t = targets[side]
            delta = qmul(t.foot, qconj(self.bind[self.index[f"{side}Foot"]]))
            forward = qrotate(delta, np.array([1.0, 0.0, 0.0]))
            yaw = math.degrees(math.atan2(forward[1], forward[0]))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, -forward[2]))))
            builder.leg(side, LegSpec(ankle=t.ankle, knee_direction=t.pole,
                                      toe_out_degrees=_sign(side) * yaw,
                                      foot_pitch_degrees=pitch))
        pose = builder.solve()
        # O pe com a orientacao inteira pedida, inclusive o rolamento, que o
        # LegSpec nao descreve.
        _, orientations = builder._forward()
        for side in SIDES:
            builder._solve_foot(side, targets[side].foot, orientations, pose)
        return builder.coordinates.copy()

    def violations(self, f: int, twist=0.0, lift=None, arms=None):
        offset, rotation, coordinates = self.pose(f, twist, lift, arms)
        positions, orientations = self.rig.forward_kinematics(
            offset, rotation, coordinates)
        return [(a, b) for a, b, gap, required in self_collision_gaps(
            self.rig, positions, orientations) if gap < required]


def _thighs(pairs) -> bool:
    return any(a.endswith("Thigh") and b.endswith("Thigh") for a, b in pairs)


def _feet(pairs) -> bool:
    return any(a.endswith("Foot") and b.endswith("Foot") for a, b in pairs)


def _arm(pairs, side: str) -> bool:
    return any(link == side + part for pair in pairs for link in pair
               for part in ARM_PARTS)


def _cover(need: np.ndarray) -> np.ndarray:
    """Suaviza uma correcao no ciclo sem ficar abaixo do necessario em
    nenhum quadro: maximo numa janela de +-2 quadros, depois media de +-1."""
    n = len(need)
    peak = np.array([max(need[(i + k) % n] for k in range(-2, 3))
                     for i in range(n)])
    return np.array([np.mean([peak[(i + k) % n] for k in (-1, 0, 1)])
                     for i in range(n)])


def corrections(strafe: Strafe):
    """Menores correcoes que tiram as autocolisoes, quadro a quadro, ja
    suavizadas: giro da pelve, pe em balanco erguido, braco aberto."""
    n = strafe.frames
    twist = np.zeros(n)
    frames = [f for f in range(n) if _thighs(strafe.violations(f))]
    if frames:
        # Um sentido so para o ciclo: o que pede menos giro.
        best = None
        for sign in (1.0, -1.0):
            need = np.zeros(n)
            for f in frames:
                need[f] = np.inf
                for magnitude in range(1, MAXIMUM_TWIST + 1):
                    if not _thighs(strafe.violations(f, sign * magnitude)):
                        need[f] = magnitude
                        break
            if best is None or need.max() < best[1].max():
                best = (sign, need)
        if not np.isfinite(best[1].max()):
            raise RuntimeError(f"strafe {strafe.side}: coxas se tocam mesmo "
                               f"com {MAXIMUM_TWIST} graus de giro")
        twist = best[0] * _cover(best[1])
    lift = {s: np.zeros(n) for s in SIDES}
    for f in range(n):
        remaining = strafe.violations(f, twist[f])
        if not _feet(remaining):
            continue
        for side in SIDES:
            if strafe.contacts[side][f] >= 0.5:
                continue
            for step in range(1, int(round(MAXIMUM_LIFT / 0.005)) + 1):
                if not _feet(strafe.violations(f, twist[f],
                                               {side: 0.005 * step})):
                    lift[side][f] = 0.005 * step
                    break
    lift = {s: _cover(v) for s, v in lift.items()}
    arms = {s: np.zeros(n) for s in SIDES}
    for f in range(n):
        lifted = {s: lift[s][f] for s in SIDES}
        remaining = strafe.violations(f, twist[f], lifted)
        for side in SIDES:
            if not _arm(remaining, side):
                continue
            for angle in range(1, MAXIMUM_ARM + 1):
                if not _arm(strafe.violations(f, twist[f], lifted,
                                              {side: float(angle)}), side):
                    arms[side][f] = angle
                    break
    arms = {s: _cover(v) for s, v in arms.items()}
    return twist, lift, arms


# Correcoes do ultimo build, para diagnostico.
LAST_CORRECTIONS: dict = {}


def _definitions(rig: Rig, side: str) -> list[ClipDefinition]:
    strafe = Strafe(rig, side)
    twist, lift, arms = corrections(strafe)
    LAST_CORRECTIONS[side] = (strafe, twist, lift, arms)
    n = strafe.frames
    poses = [strafe.pose(f, twist[f], {s: lift[s][f] for s in SIDES},
                         {s: arms[s][f] for s in SIDES}) for f in range(n)]
    closed = poses + poses[:1]
    result = []
    for sprint in (False, True):
        scale = SPRINT_CADENCE if sprint else 1.0
        duration = strafe.period / scale
        clip = Clip(
            id="", display_name="", target_rig=rig.id, duration=duration,
            sample_rate=n / duration, loops=True,
            times=np.arange(n + 1) * duration / n,
            root_offsets=np.array([p[0] for p in closed]),
            root_rotations=np.array([p[1] for p in closed]),
            coordinates=np.array([p[2] for p in closed]))

        def pose_at(t: float, clip=clip) -> Pose:
            offset, rotation, coordinates = clip.sample(t)
            return Pose(np.array(offset), np.array(rotation),
                        np.array(coordinates))

        def contact_at(t: float, clip=clip):
            f = int(round(t / clip.duration * n)) % n
            return (float(strafe.contacts["Left"][f]),
                    float(strafe.contacts["Right"][f]))

        kind = "sprint" if sprint else "jog"
        name = "Sprint de lado" if sprint else "Corrida de lado"
        where = "esquerda" if side == "Left" else "direita"
        result.append(ClipDefinition(
            id=f"{kind}_strafe_{side.lower()}",
            display_name=f"{name} ({where})",
            duration=duration, loops=True, pose_at=pose_at,
            contact_at=contact_at, sample_rate=n / duration,
            nominal_speed=round(strafe.speed * scale, 4),
            travel_direction_degrees=90.0 * _sign(side),
            floor_z=strafe.floor,
            source="tools/animation/clips/strafe.py"))
    return result


def definitions(rig: Rig) -> list[ClipDefinition]:
    return [d for side in SIDES for d in _definitions(rig, side)]
