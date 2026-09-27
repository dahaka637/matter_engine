"""Clipes matter-ragdoll-animation-1: leitura, amostragem e escrita.

A amostragem espelha sampleAnimationTrack3D (AnimationClip3D.cpp): linear nas
coordenadas de junta e na translacao da raiz, nlerp na rotacao da raiz, com o
mesmo tratamento de loop e das pontas.
"""

from __future__ import annotations

import bisect
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from matter_rig import Rig, qnormalize, quat

SCHEMA = "matter-ragdoll-animation-1"


@dataclass
class Clip:
    id: str
    display_name: str
    target_rig: str
    duration: float
    sample_rate: float
    loops: bool
    # Por quadro, em espaco de modelo (a pose de bind e a origem).
    times: np.ndarray                     # (F,)
    root_offsets: np.ndarray              # (F, 3)
    root_rotations: np.ndarray            # (F, 4)
    coordinates: np.ndarray               # (F, N, 3)
    left_foot_contact: np.ndarray | None = None   # (F,) 1 = apoio
    right_foot_contact: np.ndarray | None = None
    nominal_speed: float = 0.0
    # Direcao de deslocamento em relacao a pelve, em graus (0 = frente).
    travel_direction_degrees: float = 0.0
    source: str = ""
    extra: dict = field(default_factory=dict)

    def sample(self, time: float, loop: bool | None = None):
        """(root_offset, root_rotation, coordinates) no instante pedido."""
        loop = self.loops if loop is None else loop
        t = max(0.0, time)
        t = t % self.duration if (loop and self.duration > 0) else min(
            t, self.duration)
        k = bisect.bisect_right(self.times.tolist(), t)
        if k == 0:
            return (self.root_offsets[0], qnormalize(self.root_rotations[0]),
                    self.coordinates[0])
        if k >= len(self.times):
            return (self.root_offsets[-1], qnormalize(self.root_rotations[-1]),
                    self.coordinates[-1])
        t0, t1 = self.times[k - 1], self.times[k]
        a = 0.0 if t1 - t0 <= 1e-6 else min(max((t - t0) / (t1 - t0), 0), 1)
        r0, r1 = self.root_rotations[k - 1], self.root_rotations[k]
        if np.dot(r0, r1) < 0:
            r1 = -r1
        return (self.root_offsets[k - 1] * (1 - a) + self.root_offsets[k] * a,
                qnormalize(r0 * (1 - a) + r1 * a),
                self.coordinates[k - 1] * (1 - a) + self.coordinates[k] * a)


def load_clip(path: Path | str, rig: Rig) -> Clip:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data["schema"] != SCHEMA:
        raise ValueError(f"schema nao suportado: {data['schema']}")
    tracks = {track["targetLinkId"]: track for track in data["tracks"]}
    root_id = rig.links[0].id
    times = np.array([key["timeSeconds"]
                      for key in tracks[root_id]["keyframes"]])
    count = len(rig)
    coordinates = np.zeros((len(times), count, 3))
    for i, link in enumerate(rig.links):
        if i == 0 or link.id not in tracks:
            continue
        keys = tracks[link.id]["keyframes"]
        if len(keys) != len(times):
            raise ValueError(f"{link.id}: amostragem diferente da raiz")
        coordinates[:, i] = [key["jointPositionRadians"] for key in keys]
    root_keys = tracks[root_id]["keyframes"]

    def contact(name: str) -> np.ndarray | None:
        values = data.get("contacts", {}).get(name)
        return None if values is None else np.array(
            [v["value"] for v in values])

    return Clip(
        id=data["id"], display_name=data["displayName"],
        target_rig=data["targetRigId"],
        duration=float(data["durationSeconds"]),
        sample_rate=float(data["sourceSampleRateHz"]),
        loops=bool(data.get("loops", True)),
        times=times,
        root_offsets=np.array([k["rootTranslationOffsetMeters"]
                               for k in root_keys]),
        root_rotations=np.array([k["rootRotationDelta"] for k in root_keys]),
        coordinates=coordinates,
        left_foot_contact=contact("leftFoot"),
        right_foot_contact=contact("rightFoot"),
        nominal_speed=float(data.get("nominalSpeedMetersPerSecond", 0.0)),
        travel_direction_degrees=float(data.get("travelDirectionDegrees", 0.0)),
        source=data.get("sourceAssetPath", ""),
    )


def save_clip(clip: Clip, rig: Rig, path: Path | str,
              authoring_report: dict) -> None:
    """Grava no schema do motor.

    O runtime so aceita clipe com retargetReport aprovado. Clipe autorado nao
    tem fonte para comparar, entao as metricas de direcao sao zero por
    construcao e "passed" reflete a validacao de autoria, registrada inteira em
    authoringReport - e ela que decide.
    """
    def r(value: float) -> float:
        return round(float(value), 7)

    tracks = []
    for i, link in enumerate(rig.links):
        if i == 0:
            keys = [{
                "timeSeconds": r(t),
                "rootTranslationOffsetMeters": [r(v) for v in clip.root_offsets[f]],
                "rootRotationDelta": [r(v) for v in qnormalize(clip.root_rotations[f])],
            } for f, t in enumerate(clip.times)]
            tracks.append({"targetLinkId": link.id, "space": "root",
                           "keyframes": keys})
        else:
            keys = [{
                "timeSeconds": r(t),
                "jointPositionRadians": [r(v) for v in clip.coordinates[f, i]],
            } for f, t in enumerate(clip.times)]
            tracks.append({"targetLinkId": link.id, "space": "joint",
                           "keyframes": keys})

    def contact(values: np.ndarray | None) -> list:
        if values is None:
            values = np.ones(len(clip.times))
        return [{"timeSeconds": r(t), "value": r(v)}
                for t, v in zip(clip.times, values)]

    data = {
        "schema": SCHEMA,
        "id": clip.id,
        "displayName": clip.display_name,
        "targetRigId": clip.target_rig,
        "sourceAssetPath": clip.source,
        "durationSeconds": r(clip.duration),
        "sourceSampleRateHz": r(clip.sample_rate),
        "loops": clip.loops,
        "sourceRootDisplacementMeters": [0.0, 0.0, 0.0],
        "nominalSpeedMetersPerSecond": r(clip.nominal_speed),
        "travelDirectionDegrees": r(clip.travel_direction_degrees),
        "contacts": {"leftFoot": contact(clip.left_foot_contact),
                     "rightFoot": contact(clip.right_foot_contact)},
        "rootMotionPolicy": "in-place",
        "retargetReport": {
            "origin": "authored",
            "passed": bool(authoring_report["passed"]),
            "directionRmsDegrees": 0.0,
            "directionMaxDegrees": 0.0,
            "limbDirectionMaxDegrees": 0.0,
            "maximumJointStepDegrees": r(authoring_report["maximumJointStepDegrees"]),
            "limitHitCount": int(authoring_report["limitHitCount"]),
        },
        "authoringReport": authoring_report,
        "tracks": tracks,
    }
    Path(path).write_text(json.dumps(data, indent=1, ensure_ascii=False)
                          + "\n", encoding="utf-8")
