"""Gerador parametrico de passada: andar, recuar e correr.

Uma passada e descrita pelo que se mede num corredor ou num andador:
velocidade, cadencia, fracao de apoio, largura da base, altura do passo,
oscilacao da pelve e balanco dos bracos. O gerador transforma isso numa pose
por instante; ninguem posiciona junta a junta.

Como funciona (espaco de modelo: olha para +X, esquerda +Y, cima +Z):

  - PES. Cada pe tem um apoio e um balanco por ciclo. No apoio, a ponta do pe
    fica parada no mundo - no clipe, que e in-place, ela anda para tras a
    exatamente a velocidade nominal. O pe gira sobre a ponta no fim do apoio
    (o calcanhar sobe). No balanco, o tornozelo vai da decolagem ao proximo
    toque por um Hermite com as velocidades das pontas casadas, e a sola nunca
    desce abaixo do chao.
  - DIRECAO. A passada anda na direcao `direction_degrees` em relacao a
    pelve: 0 para a frente, 180 para recuar. Diagonal em relacao a pelve nao
    existe numa passada longa: 30 graus ja levam cada pe 23 cm para o lado,
    alem dos quadris (a 9 cm), e a perna de tras cruza por baixo do corpo.
    Quem vira para a diagonal e a pelve (o runtime faz isso).
  - RECUO. E a passada para a frente invertida no tempo: a ponta toca
    primeiro e o calcanhar desce (roll_at_touchdown), e o pe sai chato.
  - PELVE. Sobe e desce duas vezes por ciclo, balanca para o pe de apoio, gira
    com a perna que avanca e o quadril sem apoio cai.
  - TRONCO. Os ombros giram com os bracos, contra a pelve, e a cabeca segura
    o olhar para a frente (o runtime ainda soma o giro para a camera).
  - BRACOS. Balancam contra as pernas: o braco esquerdo vai a frente quando a
    perna direita esta a frente.

Contato por pe: 1 no apoio com o pe parado no mundo, 0 no resto (inclusive
durante a rolagem sobre a ponta, em que o tornozelo sobe). O runtime so trava
o pe onde o contato e 1.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from authoring import ClipDefinition
from matter_rig import Rig, qfrom_axis_angle, qmul, qrotate
from pose import ArmSpec, LegSpec, Pose, PoseBuilder
from validate import lowest_point, world_colliders, _box_axes

SIDES = (("Left", 1.0), ("Right", -1.0))


# Postura padrao dos bracos correndo, a mesma em todas as passadas: junto ao
# corpo, antebraco na linha da passada, levemente para dentro, cotovelo quase
# constante (o balanco vem do ombro) e a mao relaxada (os dedos sao da skin).
# Abertura de 16 graus (arm_lowered 74): e o minimo que deixa o braco a 5 mm
# do colisor do peito no balanco do sprint (76 encosta). Antes eram 30-32
# graus - os bracos arqueados, abertos, que o usuario apontou. Mais giro
# para dentro poe a mao na frente da pelve (o colisor dela e quase uma
# esfera de 14 cm de raio): com 8 graus a mao ja encosta nela.
RUNNING_ARM_LOWERED = 74.0
RUNNING_ARM_INWARD_ROLL = 3.0
RUNNING_WRIST_FLEXION = 4.0


@dataclass
class GaitSpec:
    id: str
    display_name: str
    speed: float                    # m/s
    cadence: float                  # passos por segundo (os dois pes)
    duty: float                     # fracao do ciclo em apoio, por pe
    direction_degrees: float = 0.0  # direcao de deslocamento em relacao a pelve
    sample_rate: float = 60.0
    # Pes.
    stance_width: float = 0.105     # do tornozelo a linha media da passada (m)
    stance_center: float = 0.0      # meio do apoio a frente (+) do quadril (m)
    swing_height: float = 0.07      # folga maxima da sola no balanco (m)
    swing_peak: float = 0.5         # onde (fracao do balanco) a folga e maxima
    stance_pitch: float = 0.0       # graus, + ponta para baixo (antepe)
    roll_fraction: float = 0.3      # fracao do apoio rolando sobre a ponta
    toe_off_pitch: float = 28.0     # graus de ponta para baixo na rolagem
    # Rolagem no inicio do apoio (recuo: a ponta toca e o calcanhar desce) em
    # vez do fim (frente: o calcanhar sobe e a ponta sai por ultimo).
    roll_at_touchdown: bool = False
    # Passo lateral (strafe), sem cruzar: cada pe fica do seu lado do corpo e
    # anda ao longo da propria passada. stance_width vira a distancia de cada
    # pe ao centro NO SENTIDO DO MOVIMENTO; os pes nunca chegam mais perto que
    # 2 * stance_width - stance_length. (Passada cruzada, como nas
    # referencias Mixamo, nao cabe no ALS: a coxa e um cilindro de 18 cm de
    # diametro ate o joelho, e as pernas se atravessariam.)
    side_step: bool = False
    landing_dorsiflex: float = 6.0  # ponta para cima no fim do balanco
    # Quanto o pe acompanha a canela no meio do balanco (0 = orientado no
    # mundo). Na corrida a canela fica quase horizontal na recuperacao.
    swing_ankle_follow: float = 0.0
    swing_ankle_flexion: float = 10.0
    # Recuperacao (m): quanto o pe fica para tras no comeco do balanco - o
    # calcanhar sobe atras antes de a perna vir para a frente. Sem isso o pe
    # vem para a frente cedo e a coxa sobe demais.
    swing_recovery: float = 0.0
    toe_out: float = 5.0
    foot_follow: float = 0.35       # quanto o pe gira para a direcao da passada
    # Pelve.
    pelvis_drop: float = 0.03       # m abaixo do bind, em media
    pelvis_bob: float = 0.015       # + sobe no meio do apoio (andar), - desce (corrida)
    pelvis_sway: float = 0.012      # m para o lado do pe de apoio
    pelvis_yaw: float = 5.0         # graus, pico
    pelvis_list: float = 2.0        # graus, quadril sem apoio cai
    lean_forward: float = 2.0       # graus, sempre para a frente
    lean_travel: float = 2.0        # graus, na direcao do deslocamento
    # Tronco e cabeca.
    spine_flexion: float = 2.0
    # Giro dos ombros (graus, pico), em fase com os bracos: o ombro do braco
    # que vai a frente avanca. E o braco que gira o torax, nao a pelve - nas
    # referencias de corrida os ombros giram 10-35 graus com a pelve em 3-6.
    shoulder_turn: float = 3.0
    head_nod: float = 4.0
    # Bracos. A postura de corrida e uma so em todas as passadas (ver
    # RUNNING_ARM_*); cada passada muda o balanco, o cotovelo e o quanto o
    # braco vive atras do tronco.
    arm_lowered: float = 76.0
    arm_forward: float = 4.0
    arm_swing: float = 20.0         # graus para frente/tras no pico
    arm_inward_roll: float = 0.0
    elbow: float = 45.0
    elbow_swing: float = 12.0       # dobra a mais a frente, a menos atras
    # Zero: com o cotovelo dobrado perto de 90 graus, a "pronacao" do ALS
    # (swing1 do antebraco) vira o antebraco girando em volta do braco, com
    # motor fraco; pedida diferente de zero o alvo cai no eixo travado e a
    # mao gira sozinha (medido: sprint lateral 88 graus, pulo saindo do
    # sprint 47 graus). O mesmo vale para o cotovelo parado perto de 90 graus
    # (ver ARM_ELBOW em clips/strafe.py).
    pronation: float = 0.0
    wrist_flexion: float = 6.0

    @property
    def cycle(self) -> float:
        return 2.0 / self.cadence

    @property
    def stance_time(self) -> float:
        return self.duty * self.cycle

    @property
    def stance_length(self) -> float:
        """Quanto o corpo anda durante um apoio."""
        return self.speed * self.stance_time


def _smooth(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def _hermite(p0, v0, p1, v1, u: float):
    h00 = 2 * u ** 3 - 3 * u ** 2 + 1
    h10 = u ** 3 - 2 * u ** 2 + u
    h01 = -2 * u ** 3 + 3 * u ** 2
    h11 = u ** 3 - u ** 2
    return h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1


def _rotation_matrix(q: np.ndarray) -> np.ndarray:
    return np.stack([qrotate(q, e) for e in np.eye(3)], axis=1)


class FootGeometry:
    """Caixa do pe em relacao ao tornozelo, medida no bind."""

    def __init__(self, rig: Rig, builder: PoseBuilder, side: str):
        colliders = world_colliders(rig, builder.bind_positions,
                                    builder.bind_orientations)
        box = colliders[rig.index(f"{side}Foot")]
        ankle = builder.bind_anchor(f"{side}Foot")
        self.center = box.center - ankle
        self.axes = _box_axes(box)          # linhas = eixos da caixa (mundo)
        self.half = box.half_extents
        # Ponto de apoio da ponta: meio da aresta inferior da frente.
        self.toe = self.center + self.axes[0] * self.half[0] \
            - self.axes[2] * self.half[2]

    def sole_offset(self, rotation: np.ndarray) -> float:
        """Altura do tornozelo acima do ponto mais baixo da caixa."""
        center = rotation @ self.center
        axes = (rotation @ self.axes.T).T
        return float(-(center[2] - np.sum(np.abs(axes[:, 2]) * self.half)))

    def toe_offset(self, rotation: np.ndarray) -> np.ndarray:
        return rotation @ self.toe


class Gait:
    def __init__(self, rig: Rig, spec: GaitSpec):
        self.rig = rig
        self.spec = spec
        self.builder = PoseBuilder(rig)
        self.floor = lowest_point(rig, self.builder.bind_positions,
                                  self.builder.bind_orientations,
                                  ["LeftFoot", "RightFoot"])
        self.feet = {side: FootGeometry(rig, self.builder, side)
                     for side, _ in SIDES}
        a = math.radians(spec.direction_degrees)
        self.travel = np.array([math.cos(a), math.sin(a), 0.0])
        lateral = np.array([-math.sin(a), math.cos(a), 0.0])
        if spec.side_step:
            # A base fica ao longo da passada, com o pe esquerdo do lado
            # esquerdo: indo para a esquerda ele e o pe da frente.
            self.lateral = self.travel if self.travel[1] >= 0 \
                else -self.travel
        else:
            # A base lateral e perpendicular a passada, com o pe esquerdo
            # sempre do lado esquerdo do corpo.
            self.lateral = lateral if lateral[1] >= 0 else -lateral
        # Eixo da fase que governa o giro da pelve e o balanco dos bracos:
        # a frente do corpo para andar e recuar, a propria passada no strafe.
        self.phase_axis = self.travel if spec.side_step \
            else np.array([1.0, 0.0, 0.0])
        backward = abs(a) > math.pi / 2
        # Quanto o pe acompanha a direcao: em relacao a frente para a
        # passada para a frente, e em relacao a tras para o recuo.
        relative = a - math.copysign(math.pi, a) if backward else a
        self.foot_yaw = spec.foot_follow * math.degrees(relative)
        # Fase que governa o giro da pelve e o balanco dos bracos: a distancia
        # entre os pes ao longo do eixo da fase, centrada e normalizada. No
        # passo lateral o pe da frente esta sempre a frente; sem centrar, a
        # pelve ficaria girada em media (medido: 5,5 graus).
        spreads = [((self._foot_state("Left", t)[0]
                     - self._foot_state("Right", t)[0]) @ self.phase_axis)
                   for t in np.linspace(0, spec.cycle, 60, endpoint=False)]
        self._spread_mean = float(np.mean(spreads))
        self._spread_norm = max(abs(v - self._spread_mean) for v in spreads)

    # --- pes ----------------------------------------------------------------
    def _foot_rotation(self, side: str, pitch_degrees: float) -> np.ndarray:
        s = 1.0 if side == "Left" else -1.0
        yaw = qfrom_axis_angle([0, 0, 1], math.radians(
            s * self.spec.toe_out + self.foot_yaw))
        pitch = qfrom_axis_angle([0, 1, 0], math.radians(pitch_degrees))
        return _rotation_matrix(qmul(yaw, pitch))

    def _rolling(self, tau: float) -> bool:
        spec = self.spec
        roll = spec.roll_fraction * spec.stance_time
        return tau < roll - 1e-6 if spec.roll_at_touchdown \
            else tau > spec.stance_time - roll + 1e-6

    def _stance_pitch(self, tau: float) -> float:
        """Inclinacao do pe no apoio: constante, com a rolagem sobre a
        ponta no fim (frente) ou no comeco (recuo)."""
        spec = self.spec
        roll = max(spec.roll_fraction * spec.stance_time, 1e-6)
        if spec.roll_at_touchdown:
            u = 1.0 - tau / roll
        else:
            u = (tau - (spec.stance_time - roll)) / roll
        return spec.stance_pitch + (spec.toe_off_pitch - spec.stance_pitch) \
            * _smooth(u)

    def _stance_ankle(self, side: str, tau: float) -> np.ndarray:
        """Tornozelo no apoio, tau segundos depois do toque. A ponta fica
        parada no mundo (anda para tras no clipe) e o pe gira sobre ela."""
        spec = self.spec
        s = 1.0 if side == "Left" else -1.0
        geometry = self.feet[side]
        flat = self._foot_rotation(side, spec.stance_pitch)
        # Onde a ponta toca: o tornozelo "no meio do apoio" fica em
        # base + centro; a ponta vai junto.
        base = s * spec.stance_width * self.lateral \
            + spec.stance_center * self.travel
        touch_ankle = base + self.travel * (spec.stance_length / 2.0)
        touch_ankle = touch_ankle + np.array(
            [0.0, 0.0, self.floor + geometry.sole_offset(flat)])
        tip = touch_ankle + geometry.toe_offset(flat) \
            - self.travel * (spec.speed * tau)
        rotation = self._foot_rotation(side, self._stance_pitch(tau))
        return tip - geometry.toe_offset(rotation)

    def _swing_follow(self, side: str, time: float) -> float:
        spec = self.spec
        offset = 0.0 if side == "Left" else 0.5
        tau = ((time / spec.cycle + offset) % 1.0) * spec.cycle
        if tau < spec.stance_time:
            return 0.0
        u = (tau - spec.stance_time) / (spec.cycle - spec.stance_time)
        return spec.swing_ankle_follow * math.sin(math.pi * u) ** 2

    def _foot_state(self, side: str, time: float):
        """(tornozelo, inclinacao do pe, contato) no instante dado."""
        spec = self.spec
        offset = 0.0 if side == "Left" else 0.5
        phase = (time / spec.cycle + offset) % 1.0
        tau = phase * spec.cycle
        if tau < spec.stance_time:
            contact = 0.0 if self._rolling(tau) else 1.0
            return self._stance_ankle(side, tau), self._stance_pitch(tau), contact
        swing_time = spec.cycle - spec.stance_time
        u = (tau - spec.stance_time) / swing_time
        eps = 1e-4
        start = self._stance_ankle(side, spec.stance_time)
        start_velocity = (start - self._stance_ankle(
            side, spec.stance_time - eps)) / eps
        end = self._stance_ankle(side, 0.0)
        end_velocity = (self._stance_ankle(side, eps) - end) / eps
        if spec.side_step:
            # De lado, casar a velocidade do apoio faria o pe sair andando
            # para o lado do outro e cruzar a perna dele: reta da decolagem
            # ao toque (o pe salta, como no passo lateral de marcacao).
            horizontal = start[:2] + (end[:2] - start[:2]) * _smooth(u)
        else:
            horizontal = _hermite(start[:2], start_velocity[:2] * swing_time,
                                  end[:2], end_velocity[:2] * swing_time, u)
        # sin^2 * (1 - u): zero e de derivada zero nas duas pontas, entao as
        # velocidades casadas do Hermite continuam valendo; pico cedo.
        horizontal = horizontal - self.travel[:2] * spec.swing_recovery \
            * math.sin(math.pi * u) ** 2 * (1.0 - u)
        # Inclinacao: sai com a da decolagem, passa pela ponta para cima e
        # chega no toque com a do comeco do apoio.
        lift_pitch = self._stance_pitch(spec.stance_time)
        land_pitch = self._stance_pitch(0.0)
        pitch = lift_pitch + (land_pitch - lift_pitch) * _smooth(u / 0.7) \
            - spec.landing_dorsiflex * math.sin(math.pi * u) ** 2
        rotation = self._foot_rotation(side, pitch)
        # Folga da sola: sobe ate swing_height no pico e volta a zero; o
        # tornozelo fica onde a sola cumpre essa folga para a inclinacao atual.
        peak = spec.swing_peak
        v = u / peak if u < peak else 1.0 + (u - peak) / (1.0 - peak)
        clearance = spec.swing_height * math.sin(math.pi * v / 2.0)
        z = self.floor + clearance + self.feet[side].sole_offset(rotation)
        return np.array([horizontal[0], horizontal[1], z]), pitch, 0.0

    # --- pose -----------------------------------------------------------------
    def pose_at(self, time: float) -> Pose:
        spec = self.spec
        states = {side: self._foot_state(side, time) for side, _ in SIDES}
        # Fase de referencia: meio do apoio esquerdo.
        mid = (time / spec.cycle - spec.duty / 2.0) * 2.0 * math.pi
        spread = (((states["Left"][0] - states["Right"][0]) @ self.phase_axis)
                  - self._spread_mean) / max(self._spread_norm, 1e-6)

        yaw = -spec.pelvis_yaw * spread
        a = math.radians(spec.direction_degrees)
        roll = spec.pelvis_list * math.cos(mid) \
            - spec.lean_travel * math.sin(a)
        pitch = spec.lean_forward + spec.lean_travel * math.cos(a) \
            * (1.0 if abs(a) <= math.pi / 2 else 0.0)
        bob = spec.pelvis_bob * math.cos(2.0 * mid)
        sway = spec.pelvis_sway * math.cos(mid)

        b = PoseBuilder(self.rig)
        b.root(offset=(0.0, sway, -spec.pelvis_drop + bob),
               pitch_degrees=pitch, roll_degrees=roll, yaw_degrees=yaw)
        # Torax girando com os bracos (braco direito a frente -> ombros para
        # a esquerda), contra a pelve; a coluna leva a diferenca e devolve a
        # queda lateral do quadril.
        chest_yaw = spec.shoulder_turn * spread
        b.spine(flexion=spec.spine_flexion,
                lateral=0.8 * roll,
                axial=chest_yaw - yaw)
        b.head(nod=spec.head_nod - 0.7 * (pitch + spec.spine_flexion),
               turn=-chest_yaw, tilt=0.2 * roll)
        for side, s in SIDES:
            ankle, foot_pitch, _ = states[side]
            foot_yaw = s * spec.toe_out + self.foot_yaw
            knee = np.array([math.cos(math.radians(foot_yaw)),
                             math.sin(math.radians(foot_yaw)), 0.0])
            b.leg(side, LegSpec(ankle=ankle, knee_direction=knee,
                                toe_out_degrees=s * foot_yaw,
                                foot_pitch_degrees=foot_pitch,
                                ankle_follow=self._swing_follow(side, time),
                                ankle_flexion_degrees=spec.swing_ankle_flexion))
            # Braco esquerdo a frente quando a perna direita esta a frente.
            swing = -s * spread
            b.arm(side, ArmSpec(
                lowered_degrees=spec.arm_lowered,
                forward_degrees=spec.arm_forward + spec.arm_swing * swing,
                inward_roll_degrees=spec.arm_inward_roll,
                # O cotovelo dobra com o braco a frente e abre atras: e o que
                # tira a mao do caminho da coxa que avanca.
                elbow_flexion_degrees=spec.elbow + spec.elbow_swing * swing,
                pronation_degrees=spec.pronation,
                wrist_flexion_degrees=spec.wrist_flexion))
        return b.solve()

    def contact_at(self, time: float) -> tuple[float, float]:
        return (self._foot_state("Left", time)[2],
                self._foot_state("Right", time)[2])


def gait_definition(rig: Rig, spec: GaitSpec) -> ClipDefinition:
    gait = Gait(rig, spec)
    return ClipDefinition(
        id=spec.id, display_name=spec.display_name, duration=spec.cycle,
        loops=True, pose_at=gait.pose_at, contact_at=gait.contact_at,
        sample_rate=spec.sample_rate, nominal_speed=spec.speed,
        travel_direction_degrees=spec.direction_degrees,
        floor_z=gait.floor)
