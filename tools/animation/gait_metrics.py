"""Mede uma passada: referencia capturada ou clipe nosso, com as mesmas regras.

Serve para se inspirar numa referencia sem copia-la: mede-se o que caracteriza
a passada dela (velocidade, cadencia, apoio, voo, altura do joelho, balanco dos
bracos, inclinacao...) e esses numeros viram parametros do gerador (gait.py).
Medir os nossos clipes com o mesmo codigo e o que permite comparar.

Referencial: frente +X, esquerda +Y, cima +Z, metros, corpo do tamanho do
FootballPlayerV1 (reference_capture.py ja entrega assim).

Uso:
  python3 tools/animation/gait_metrics.py slow_run.capture.json \\
      assets/animations/clips/jog.matteranim.json
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from matter_clip import load_clip  # noqa: E402
from matter_rig import load_default_character, qconj, qrotate, quat  # noqa: E402


def rig_identity():
    return quat()


# --- captura dos nossos clipes -----------------------------------------------
def capture_from_clip(path: Path, samples: int = 60) -> dict:
    """Mesmos pontos que reference_capture.py, tirados das ancoras das juntas
    do nosso rig. O clipe e in-place: o deslocamento nominal e somado para
    que os pes de apoio fiquem parados, como numa captura com raiz."""
    rig, _ = load_default_character()
    clip = load_clip(path, rig)
    direction = math.radians(clip.travel_direction_degrees)
    velocity = clip.nominal_speed * np.array(
        [math.cos(direction), math.sin(direction), 0.0])

    def anchor(positions, orientations, link_id):
        i = rig.index(link_id)
        link = rig.links[i]
        local = qrotate(qconj(link.model_orientation),
                        link.anchor - link.model_position)
        return positions[i] + qrotate(orientations[i], local)

    def foot_point(positions, orientations, side, forward):
        # Na sola, ao longo da caixa do pe: 0 = tornozelo, 1 = ponta.
        i = rig.index(f"{side}Foot")
        local = np.array([0.24 * forward, 0.0, -0.13])
        return positions[i] + qrotate(orientations[i], local)

    zero = np.zeros((len(rig), 3))
    bind_p, bind_o = rig.forward_kinematics(np.zeros(3), rig_identity(),
                                            zero)
    points = {key: [] for key in (
        "Hips", "Chest", "Neck", "Head", "HeadTop",
        "LeftHip", "LeftKnee", "LeftAnkle", "LeftBall", "LeftToe",
        "RightHip", "RightKnee", "RightAnkle", "RightBall", "RightToe",
        "LeftShoulder", "LeftElbow", "LeftWrist",
        "RightShoulder", "RightElbow", "RightWrist")}
    def landmarks(p, o):
        head = rig.index("Head")
        values = {
            "Hips": p[0], "Chest": anchor(p, o, "UpperChest"),
            "Neck": anchor(p, o, "Neck"), "Head": anchor(p, o, "Head"),
            "HeadTop": anchor(p, o, "Head")
            + qrotate(o[head], np.array([0.0, 0.0, 0.2])),
        }
        for side in ("Left", "Right"):
            values[f"{side}Hip"] = anchor(p, o, f"{side}Thigh")
            values[f"{side}Knee"] = anchor(p, o, f"{side}Shin")
            values[f"{side}Ankle"] = anchor(p, o, f"{side}Foot")
            values[f"{side}Ball"] = foot_point(p, o, side, 0.7)
            values[f"{side}Toe"] = foot_point(p, o, side, 1.0)
            values[f"{side}Shoulder"] = anchor(p, o, f"{side}UpperArm")
            values[f"{side}Elbow"] = anchor(p, o, f"{side}Forearm")
            values[f"{side}Wrist"] = anchor(p, o, f"{side}Hand")
        return values

    for t in np.linspace(0.0, clip.duration, samples + 1):
        offset, rotation, coordinates = clip.sample(float(t))
        p, o = rig.forward_kinematics(offset, rotation, coordinates)
        for key, value in landmarks(p, o).items():
            points[key].append((value + velocity * t).tolist())
    rest = {key: value.tolist()
            for key, value in landmarks(bind_p, bind_o).items()}
    return {"source": path.name, "fps": samples / clip.duration,
            "frames": samples + 1, "positions": points, "rest": rest}


# --- medidas -----------------------------------------------------------------
def _angle(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    cos = np.sum(a * b, axis=-1) / (np.linalg.norm(a, axis=-1)
                                    * np.linalg.norm(b, axis=-1))
    return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))


def _amplitude(values: np.ndarray) -> float:
    return float((values.max() - values.min()) / 2.0)


def _pitch(vectors: np.ndarray) -> np.ndarray:
    """Inclinacao para a frente em relacao a vertical (graus)."""
    return np.degrees(np.arctan2(vectors[..., 0], vectors[..., 2]))


def measure(capture: dict) -> dict:
    P = {k: np.asarray(v, dtype=float) for k, v in capture["positions"].items()}
    R = {k: np.asarray(v, dtype=float) for k, v in capture["rest"].items()}
    # O corpo nem sempre olha para +X da pose de repouso (Run Backward.fbx
    # esta virado 180 graus): gira tudo para o rumo medio dos quadris.
    hip_line = (P["LeftHip"] - P["RightHip"]).mean(axis=0)
    heading = math.atan2(-hip_line[0], hip_line[1])
    c, s = math.cos(-heading), math.sin(-heading)
    turn = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    P = {k: v @ turn.T for k, v in P.items()}
    fps = float(capture["fps"])
    frames = len(P["Hips"])
    period = (frames - 1) / fps
    mid_hip = (P["LeftHip"] + P["RightHip"]) / 2.0
    travel = mid_hip[-1] - mid_hip[0]
    travel[2] = 0.0
    speed = float(np.linalg.norm(travel)) / period
    direction = math.degrees(math.atan2(travel[1], travel[0]))
    # Fechamento: a ultima amostra repete a primeira, entao mede-se sem ela.
    body = slice(0, frames - 1)
    dt = 1.0 / fps

    result = {
        "velocidade (m/s)": speed,
        "direcao (graus)": direction,
        "ciclo (s)": period,
        "cadencia (passos/min)": 120.0 / period,
        "passo (m)": speed * period / 2.0,
    }

    contact = {}
    for side in ("Left", "Right"):
        ball = P[f"{side}Ball"][body]
        ankle = P[f"{side}Ankle"][body]
        low = np.minimum(ball[:, 2], ankle[:, 2] - 0.06)
        # Apoio: pe perto do chao e quase parado no mundo.
        velocity = np.gradient(ball[:, :2], dt, axis=0)
        still = np.linalg.norm(velocity, axis=1) < max(0.35 * speed, 0.25)
        contact[side] = (low - low.min() < 0.025) & still
    both = contact["Left"] | contact["Right"]
    result["apoio por pe (%)"] = 100.0 * float(
        (contact["Left"].mean() + contact["Right"].mean()) / 2.0)
    result["voo (%)"] = 100.0 * float(1.0 - both.mean())

    hip_height = mid_hip[body, 2]
    ankle_floor = min(P["LeftAnkle"][:, 2].min(), P["RightAnkle"][:, 2].min())
    result["quadril acima do tornozelo (m)"] = float(hip_height.mean()
                                                     - ankle_floor)
    result["pelve sobe-desce (cm)"] = 100.0 * _amplitude(hip_height)
    in_stance = hip_height[both[: len(hip_height)]]
    result["pelve no apoio x media (cm)"] = 100.0 * float(
        in_stance.mean() - hip_height.mean()) if len(in_stance) else 0.0
    lateral = mid_hip[body, 1] - np.polyval(np.polyfit(
        np.arange(frames - 1), mid_hip[body, 1], 1), np.arange(frames - 1))
    result["pelve lateral (cm)"] = 100.0 * _amplitude(lateral)

    hip_line = P["LeftHip"][body] - P["RightHip"][body]
    result["pelve giro (graus)"] = _amplitude(np.degrees(
        np.arctan2(-hip_line[:, 0], hip_line[:, 1])))
    result["pelve queda lateral (graus)"] = _amplitude(np.degrees(
        np.arctan2(hip_line[:, 2], np.linalg.norm(hip_line[:, :2], axis=1))))
    shoulder_line = P["LeftShoulder"][body] - P["RightShoulder"][body]
    shoulder_yaw = np.degrees(np.arctan2(-shoulder_line[:, 0],
                                         shoulder_line[:, 1]))
    result["ombros giro (graus)"] = _amplitude(shoulder_yaw)
    pelvis_yaw = np.degrees(np.arctan2(-hip_line[:, 0], hip_line[:, 1]))
    # Torax contra a pelve: correlacao negativa = gira ao contrario.
    result["ombros x pelve (correlacao)"] = float(np.corrcoef(
        shoulder_yaw, pelvis_yaw)[0, 1])

    rest_mid_hip = (R["LeftHip"] + R["RightHip"]) / 2.0
    trunk_rest = _pitch(R["Neck"] - rest_mid_hip)
    trunk = _pitch(P["Neck"][body] - mid_hip[body]) - trunk_rest
    result["tronco inclinado (graus)"] = float(trunk.mean())
    result["tronco balanca (graus)"] = _amplitude(trunk)
    head_rest = _pitch(R["HeadTop"] - R["Head"])
    head = _pitch(P["HeadTop"][body] - P["Head"][body]) - head_rest
    result["cabeca inclinada (graus)"] = float(head.mean())

    stance_width, touch, lift, knee_max, knee_touch, knee_stance = \
        [], [], [], [], [], []
    swing_height, heel_to_hip = [], []
    thigh_forward, thigh_back = [], []
    foot_touch, foot_lift = [], []
    for side in ("Left", "Right"):
        hip = P[f"{side}Hip"][body]
        knee = P[f"{side}Knee"][body]
        ankle = P[f"{side}Ankle"][body]
        ball = P[f"{side}Ball"][body]
        c = contact[side]
        flexion = 180.0 - _angle(hip - knee, ankle - knee)
        relative = ankle - mid_hip[body]
        stance_width.append(np.abs(relative[c, 1]).mean())
        starts = np.where(c & ~np.roll(c, 1))[0]
        ends = np.where(c & ~np.roll(c, -1))[0]
        touch.extend(relative[starts, 0])
        lift.extend(relative[ends, 0])
        knee_touch.extend(flexion[starts])
        knee_max.append(flexion[~c].max() if (~c).any() else 0.0)
        knee_stance.append(flexion[c].max() if c.any() else 0.0)
        swing_height.append((ankle[:, 2] - ankle[c, 2].min()).max())
        heel_to_hip.append(np.linalg.norm(ankle - hip, axis=1).min())
        thigh = knee - hip
        pitch = np.degrees(np.arctan2(thigh[:, 0], -thigh[:, 2]))
        thigh_forward.append(pitch.max())
        thigh_back.append(pitch.min())
        foot = ball - ankle
        rest_foot = R[f"{side}Ball"] - R[f"{side}Ankle"]
        foot_rest = math.degrees(math.atan2(-rest_foot[2],
                                            np.linalg.norm(rest_foot[:2])))
        foot_pitch = np.degrees(np.arctan2(-foot[:, 2],
                                           np.linalg.norm(foot[:, :2], axis=1))) \
            - foot_rest
        foot_touch.extend(foot_pitch[starts])
        foot_lift.extend(foot_pitch[ends])
    result["base: tornozelo ao centro (cm)"] = 100.0 * float(np.mean(stance_width))
    result["toque: tornozelo a frente do quadril (cm)"] = 100.0 * float(np.mean(touch))
    result["saida: tornozelo a frente do quadril (cm)"] = 100.0 * float(np.mean(lift))
    result["joelho no toque (graus)"] = float(np.mean(knee_touch))
    result["joelho max no apoio (graus)"] = float(np.mean(knee_stance))
    result["joelho max no balanco (graus)"] = float(np.mean(knee_max))
    result["tornozelo sobe no balanco (cm)"] = 100.0 * float(np.mean(swing_height))
    result["tornozelo mais perto do quadril (cm)"] = 100.0 * float(np.mean(heel_to_hip))
    result["coxa a frente max (graus)"] = float(np.mean(thigh_forward))
    result["coxa atras max (graus)"] = float(np.mean(thigh_back))
    result["pe no toque (graus, + ponta abaixo)"] = float(np.mean(foot_touch))
    result["pe na saida (graus, + ponta abaixo)"] = float(np.mean(foot_lift))

    arm_forward, arm_back, elbow_min, elbow_max, arm_out = [], [], [], [], []
    for side in ("Left", "Right"):
        s = 1.0 if side == "Left" else -1.0
        upper = P[f"{side}Elbow"][body] - P[f"{side}Shoulder"][body]
        lower = P[f"{side}Wrist"][body] - P[f"{side}Elbow"][body]
        pitch = np.degrees(np.arctan2(upper[:, 0], -upper[:, 2]))
        arm_forward.append(pitch.max())
        arm_back.append(pitch.min())
        elbow = 180.0 - _angle(-upper, lower)
        elbow_min.append(elbow.min())
        elbow_max.append(elbow.max())
        arm_out.append(np.degrees(np.arctan2(s * upper[:, 1],
                                             -upper[:, 2])).mean())
    result["braco a frente max (graus)"] = float(np.mean(arm_forward))
    result["braco atras max (graus)"] = float(np.mean(arm_back))
    result["braco aberto (graus)"] = float(np.mean(arm_out))
    result["cotovelo min (graus)"] = float(np.mean(elbow_min))
    result["cotovelo max (graus)"] = float(np.mean(elbow_max))
    return result


def load_any(path: Path) -> dict:
    if path.name.endswith(".matteranim.json"):
        return capture_from_clip(path)
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    paths = [Path(p) for p in sys.argv[1:]]
    if not paths:
        print(__doc__)
        return 1
    measured = [(p.name.split(".")[0], measure(load_any(p))) for p in paths]
    width = max(len(k) for k in measured[0][1]) + 2
    print("".join([" " * width] + [f"{name[:16]:>17}" for name, _ in measured]))
    for key in measured[0][1]:
        print(f"{key:<{width}}" + "".join(f"{m[key]:17.2f}" for _, m in measured))
    return 0


if __name__ == "__main__":
    sys.exit(main())
