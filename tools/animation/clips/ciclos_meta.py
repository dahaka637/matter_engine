"""Os ciclos de trote e corrida do juiz nas velocidades-meta (07/11).

Ele: "os clipes de trote e corrida, para os lados, frente e costas, gostaria de voltar aos que nos tinhamos antes para o
juiz" - e regerar na meta. Os nossos foram feitos a 3,0 m/s (trote) e 4,1-4,4 (corrida); as metas sao trote 4,5 de
frente, 2,5 de lado e 3,35 de costas, e corrida 6,0 de frente, 3,5 de lado e 5,5 de costas (de lado desde 07/11: 3,9 e 5,5
nao cabiam no corpo humano).

A adaptacao e a dos clipes da Unreal (gasp_to_bvh.adapt_speed): com f = meta / velocidade do clipe, a cadencia vai a
f^0,25 e a passada a f^0,75 (o produto e f; o ritmo sobe pouco e a passada cresce).
  - os da passada (jog, jog_backward, run_backward_trote, sprint): a mesma especificacao com a velocidade e a cadencia
    novas; o apoio encurta com f^-0,5 (no chao a passada cresce f^0,25 e a perna alcanca);
  - os de lado (strafe.py): a referencia da Mixamo com a oscilacao de cada junta, da raiz e do giro da pelve em volta da
    media do ciclo vezes f^0,75, e o pe de apoio plantado de novo na velocidade nova (o mesmo plantio e as mesmas
    correcoes de colisao do strafe.py).
  - a corrida de frente: o sprint (inspirado no mesmo Running(2).fbx da run_forward_medium) a 6,0 - a run_forward_medium
    da Mixamo escorrega o pe 62 mm e sai 33 mm do chao ja na velocidade dela, e nao tem gerador para replantar.
Nomes proprios: o modo padrao do jogo continua com os de antes.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import jog  # noqa: E402
import jog_backward  # noqa: E402
import run_backward_trote  # noqa: E402
import sprint  # noqa: E402
import strafe  # noqa: E402

from authoring import ClipDefinition  # noqa: E402
from gait import gait_definition  # noqa: E402
from matter_clip import Clip  # noqa: E402
from matter_rig import Rig, qconj, qexp, qlog, qmul, qnormalize  # noqa: E402
from pose import Pose  # noqa: E402

SOURCE = "tools/animation/clips/ciclos_meta.py"
# (A corrida de frente perto do ritmo da run_forward_medium adaptada - 171 passos/min a 4,33 m/s, x (6/4,33)^0,25 = 186; o
# sprint pela regra ficava em 244. Abaixo de ~200 a perna nao alcancava o toque ou a coxa saia do limite: 207 passos/min, o
# toque 1 cm mais perto e a pelve 1 cm mais baixa que no sprint.)
CORRIDA_FRENTE_CADENCIA = 3.45


def _natural(rig: Rig, spec, speed: float, clip_id: str, name: str, apoio: float = 0.5,
             cadence: float | None = None, **extra) -> ClipDefinition:
    f = speed / spec.speed
    cadence = cadence or spec.cadence * f ** 0.25
    # (O apoio vezes (cadencia nova / velocidade nova)^apoio relativos ao clipe: apoio 1 = a mesma passada no chao.)
    duty = spec.duty * (f / (cadence / spec.cadence)) ** -apoio
    definition = gait_definition(rig, replace(spec, id=clip_id, display_name=name, speed=speed, cadence=cadence,
                                              duty=duty, **extra))
    definition.source = f"{SOURCE} (de {spec.id})"
    return definition


def _stretch(values: np.ndarray, stride: float) -> np.ndarray:
    mean = values.mean(axis=0)
    return mean + stride * (values - mean)


class _Stretched:
    """O strafe.Strafe com a referencia esticada: target (m/s) e base (a cadencia do clipe de antes: 1 o trote de lado,
    strafe.SPRINT_CADENCE o sprint de lado) definidos na subclasse."""
    target = 0.0
    base = 1.0

    def _load(self) -> None:
        super()._load()
        f = self.target / (self.speed * self.base)
        stride = f ** 0.75
        self.cadence = self.base * f ** 0.25
        self.offsets = _stretch(self.offsets, stride)
        self.coordinates = _stretch(self.coordinates, stride)
        first = self.rotations[0]
        aligned = np.array([q if np.dot(q, first) >= 0 else -q for q in self.rotations])
        mean = qnormalize(aligned.mean(axis=0))
        self.rotations = np.array([qnormalize(qmul(mean, qexp(stride * qlog(qmul(qconj(mean), q))))) for q in aligned])
        self.speed *= stride
        self.velocity = self.velocity * stride
        self.reference = [self.rig.forward_kinematics(self.offsets[f], self.rotations[f], self.coordinates[f])
                          for f in range(self.frames)]


def _sideways(rig: Rig, side: str, base: float, speed: float, clip_id: str, name: str) -> ClipDefinition:
    parent = strafe.MirroredStrafe if side == "Right" and strafe.MIRROR_RIGHT else strafe.Strafe
    cls = type("Esticado", (_Stretched, parent), {"target": speed, "base": base})
    planted = cls(rig) if parent is strafe.MirroredStrafe else cls(rig, side)
    twist, lift, arms = strafe.corrections(planted)
    n = planted.frames
    poses = [planted.pose(f, twist[f], {s: lift[s][f] for s in strafe.SIDES}, {s: arms[s][f] for s in strafe.SIDES})
             for f in range(n)]
    closed = poses + poses[:1]
    duration = planted.period / planted.cadence
    clip = Clip(id="", display_name="", target_rig=rig.id, duration=duration, sample_rate=n / duration, loops=True,
                times=np.arange(n + 1) * duration / n, root_offsets=np.array([p[0] for p in closed]),
                root_rotations=np.array([p[1] for p in closed]), coordinates=np.array([p[2] for p in closed]))

    def pose_at(t: float) -> Pose:
        offset, rotation, coordinates = clip.sample(t)
        return Pose(np.array(offset), np.array(rotation), np.array(coordinates))

    def contact_at(t: float) -> tuple[float, float]:
        f = int(round(t / duration * n)) % n
        return float(planted.contacts["Left"][f]), float(planted.contacts["Right"][f])

    return ClipDefinition(id=clip_id, display_name=name, duration=duration, loops=True, pose_at=pose_at,
                          contact_at=contact_at, sample_rate=n / duration,
                          nominal_speed=round(planted.speed * planted.cadence, 4),
                          travel_direction_degrees=90.0 * (1.0 if side == "Left" else -1.0), floor_z=planted.floor,
                          source=f"{SOURCE} (de strafe.py)")


def definitions(rig: Rig) -> list[ClipDefinition]:
    run = strafe.SPRINT_CADENCE
    return [
        # (O calcanhar mais alto no balanco: a 0,18 a perna nao alcancava logo depois do apoio.)
        # (09/11 02h20, ele: "o trote vamos baixar para algo em torno de 3.5 - 4.0 m/s": de frente 4,5 -> 4,0, de costas 3,35 -> 3,5.)
        _natural(rig, jog.JOG, 4.0, "ciclo_trote_frente", "Trote de frente a 4,0", apoio=1.0, swing_height=0.28),
        # (07/11, ele: as metas de lado no limite humano - 3,9 e 5,5 pediam passos de lado de 1,28 m a 184 e 258 passos/min.)
        # (09/11 02h30, ele: "no trote todas as direcoes 4 m/s".)
        _sideways(rig, "Left", 1.0, 4.0, "ciclo_trote_esquerda", "Trote de lado (esquerda) a 4,0"),
        _sideways(rig, "Right", 1.0, 4.0, "ciclo_trote_direita", "Trote de lado (direita) a 4,0"),
        _natural(rig, jog_backward.JOG_BACKWARD, 4.0, "ciclo_trote_costas", "Trote de costas a 4,0"),
        _natural(rig, sprint.SPRINT, 6.0, "ciclo_corrida_frente", "Corrida de frente a 6,0", apoio=1.0,
                 cadence=CORRIDA_FRENTE_CADENCIA, stance_center=-0.12, pelvis_drop=sprint.SPRINT.pelvis_drop + 0.01),
        # (09/11 02h20, ele: "as velocidades do corrida laterais, deve subir para 4.5 m/s".)
        _sideways(rig, "Left", run, 4.5, "ciclo_corrida_esquerda", "Corrida de lado (esquerda) a 4,5"),
        _sideways(rig, "Right", run, 4.5, "ciclo_corrida_direita", "Corrida de lado (direita) a 4,5"),
        _natural(rig, run_backward_trote.RUN_BACKWARD_TROTE, 5.5, "ciclo_corrida_costas", "Corrida de costas a 5,5",
                 apoio=1.25),
    ]
