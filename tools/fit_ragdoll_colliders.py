#!/usr/bin/env python3
"""Ajusta os colisores do ragdoll a malha do personagem.

O perfil fisico do jogador (FootballPlayerV1) herdou os colisores do manequim
ALS: membros cilindricos grossos ate o joelho e o cotovelo e um tronco largo.
A malha do jogador e anatomica - coxa e canela afinam, a panturrilha fica
atras do osso, o tronco e mais estreito. Com os colisores velhos as pernas nao
passavam uma pela outra onde passam num corpo real e o braco batia num tronco
que nao estava la.

Aqui os colisores sao medidos da propria malha (vertices com peso dominante
no segmento, na pose de bind):

  - MEMBROS (braco, antebraco, coxa, canela): capsula conica. Eixo do osso,
    deslocado pelo centro medio da malha no meio do segmento; raio perto da
    articulacao de cima e perto da de baixo; as tampas chegam as pontas da
    malha. Nas pernas o raio e a mediana da secao: o jogador usa short, e a
    malha do short se abre ate perto do joelho (16-20 cm de largura) - o
    colisor e o corpo, o pano pode sobrar 1-2 cm como numa pessoa; o raio de
    baixo da coxa e medido no joelho, depois da barra do short. Nos bracos
    a manga e justa: percentil 70. O comeco da coxa fica de fora da medida
    (gluteo e virilha sao volume da pelve).
  - COXAS: o raio de cima e limitado para as duas ficarem a 2,4 cm uma da
    outra em repouso. Sobrepostas no repouso, a regra de autocolisao
    desligaria o par e as pernas poderiam se atravessar em qualquer situacao.
    E a folga e maior que o minimo porque a carne da parte interna da coxa
    cede: na passada cruzada do strafe (Jog Strafe) as coxas passam uma
    rente a outra, e com 3 mm de folga o nosso corpo so repetia a referencia
    girando a pelve 19-25 graus a mais; com 2,4 cm, 4-10 graus. O colisor e
    o nucleo da coxa, nao a malha do short.
  - MAO: capsula uniforme, so o raio e medido, na mao aberta (a malha como
    modelada; na tela os dedos ficam dobrados pelos ossos visuais). Medida
    na mao relaxada ela encolhe de 20 para 15 cm, e o antebraco deixa de
    acompanhar o alvo: rastreio do braco no trote de 16 para 25 graus. A
    inercia da mao cai e os ganhos do braco foram calibrados com a maior.
  - TRONCO (pelve, abdomen, peito, peito superior): capsulas laterais, o
    mesmo metodo do gerador do ALS - raio = media da meia profundidade e da
    meia altura do segmento, comprimento = largura.
  - PES: caixa com o comprimento e a largura do tenis; a altura (a sola) nao
    muda, porque o chao do personagem e medido por ela.

Cabeca e pescoco ficam como estao. Ancoras, limites, massas e centros de
massa nao mudam - as animacoes continuam valendo.

Uso (depois de tools/prepare_football_player.py):
  python3 tools/fit_ragdoll_colliders.py \\
      assets/characters/football_player/FootballPlayerV1.ragdoll.json \\
      assets/characters/football_player/football.skin.json
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "animation"))

from matter_rig import Rig, Skin, qrotate, quat  # noqa: E402
from validate import collider_gap, world_colliders  # noqa: E402

TAPERED = ("UpperArm", "Forearm", "Thigh", "Shin")
TORSO = ("Pelvis", "Abdomen", "Chest", "UpperChest")
RADIUS_PERCENTILE = {"Thigh": 50.0, "Shin": 50.0, "UpperArm": 70.0,
                     "Forearm": 70.0, "Hand": 70.0}
# Faixas do segmento (fracao do osso) onde cada raio e medido. A da coxa
# embaixo e o joelho, depois da barra do short: acima dela a malha e pano.
PROXIMAL_BAND = (0.15, 0.35)
DISTAL_BAND = {"Thigh": (0.90, 0.99)}
DEFAULT_DISTAL_BAND = (0.75, 0.92)
# Faixa usada para o desvio do eixo (centro da malha em relacao ao osso).
OFFSET_BAND = (0.25, 0.75)
THIGH_REST_CLEARANCE = 0.024


def _rotation_from_x(direction: np.ndarray) -> list[float]:
    """Quaternion (x, y, z, w) que leva +X a direction."""
    x = np.array([1.0, 0.0, 0.0])
    d = direction / np.linalg.norm(direction)
    axis = np.cross(x, d)
    s = np.linalg.norm(axis)
    c = float(x @ d)
    if s < 1e-9:
        return [0.0, 0.0, 0.0, 1.0] if c > 0 else [0.0, 0.0, 1.0, 0.0]
    angle = math.atan2(s, c)
    axis /= s
    h = math.sin(angle / 2)
    return [float(axis[0] * h), float(axis[1] * h), float(axis[2] * h),
            float(math.cos(angle / 2))]


def _rounded(values) -> list[float]:
    return [round(float(v), 6) for v in values]


def fit_limb(rig: Rig, index: int, mesh: np.ndarray, capsule: dict,
             part: str) -> str:
    link = rig.links[index]
    children = [j for j, other in enumerate(rig.links) if other.parent == index]
    start = link.anchor
    if children:
        end = rig.links[children[0]].anchor
    else:
        # Mao: sem filho; o eixo e o da capsula atual, apontando para a malha.
        direction = qrotate(np.array(capsule["orientation"]),
                            np.array([1.0, 0.0, 0.0]))
        if direction @ (mesh.mean(axis=0) - start) < 0:
            direction = -direction
        end = start + direction * 0.18
    bone = end - start
    length = float(np.linalg.norm(bone))
    axis = bone / length
    t = (mesh - start) @ axis
    radial_vectors = (mesh - start) - np.outer(t, axis)
    middle = (t > OFFSET_BAND[0] * length) & (t < OFFSET_BAND[1] * length)
    offset = radial_vectors[middle].mean(axis=0) if middle.any() else np.zeros(3)
    radial = np.linalg.norm(radial_vectors - offset, axis=1)
    percentile = RADIUS_PERCENTILE[part]

    def band_radius(band):
        sel = (t > band[0] * length) & (t < band[1] * length)
        return float(np.percentile(radial[sel], percentile))

    t_low = float(np.percentile(t, 1))
    t_high = float(np.percentile(t, 99))
    if part in TAPERED:
        proximal = band_radius(PROXIMAL_BAND)
        distal = band_radius(DISTAL_BAND.get(part, DEFAULT_DISTAL_BAND))
    else:
        proximal = distal = float(np.percentile(radial, percentile))
    first, last = t_low + proximal, t_high - distal
    if last <= first:
        first = last = 0.5 * (t_low + t_high)
    center = start + offset + axis * (0.5 * (first + last))
    capsule["position"] = _rounded(center - link.model_position)
    capsule["orientation"] = [round(v, 7) for v in _rotation_from_x(axis)]
    capsule["length"] = round(float((last - first) + proximal + distal), 6)
    capsule["radius"] = round(proximal, 6)
    if part in TAPERED:
        capsule["radiusAtPositiveX"] = round(distal, 6)
    else:
        capsule.pop("radiusAtPositiveX", None)
    return (f"{link.id:14s} conica {proximal * 100:4.1f} -> {distal * 100:4.1f}"
            f" cm, {capsule['length'] * 100:5.1f} cm, eixo deslocado "
            f"{np.linalg.norm(offset) * 100:.1f} cm")


def fit_torso(rig: Rig, index: int, mesh: np.ndarray, capsule: dict) -> str:
    """Capsula lateral (eixo X local = lateral do corpo), como no ALS."""
    link = rig.links[index]
    orientation = np.array(capsule["orientation"])
    center = link.model_position + np.array(capsule.get("position", [0, 0, 0]))
    frame = np.stack([qrotate(orientation, e) for e in np.eye(3)])  # linhas
    local = (mesh - center) @ frame.T
    low, high = np.percentile(local, 3, axis=0), np.percentile(local, 97, axis=0)
    half = (high - low) / 2.0
    middle = (high + low) / 2.0
    # Largura (x local) centrada no corpo; profundidade e altura podem
    # deslocar o centro.
    middle[0] = 0.0
    radius = float((half[1] + half[2]) / 2.0)
    length = float(max(2.0 * radius + 0.012, 2.0 * half[0]))
    new_center = center + middle @ frame
    capsule["position"] = _rounded(new_center - link.model_position)
    capsule["length"] = round(length, 6)
    capsule["radius"] = round(radius, 6)
    capsule.pop("radiusAtPositiveX", None)
    return (f"{link.id:14s} lateral r {radius * 100:4.1f} cm, "
            f"largura {length * 100:5.1f} cm")


def fit_foot(rig: Rig, index: int, mesh: np.ndarray, box: dict) -> str:
    """Comprimento e largura do tenis; a altura (e a sola) nao mudam."""
    link = rig.links[index]
    orientation = np.array(box["orientation"])
    center = link.model_position + np.array(box.get("position", [0, 0, 0]))
    frame = np.stack([qrotate(orientation, e) for e in np.eye(3)])
    local = (mesh - center) @ frame.T
    low, high = np.percentile(local, 1, axis=0), np.percentile(local, 99, axis=0)
    half = np.array(box["halfExtents"], dtype=float)
    middle = np.zeros(3)
    for a in (0, 1):
        half[a] = (high[a] - low[a]) / 2.0
        middle[a] = (high[a] + low[a]) / 2.0
    box["position"] = _rounded(center + middle @ frame - link.model_position)
    box["halfExtents"] = _rounded(half)
    return (f"{link.id:14s} caixa {half[0] * 200:4.1f} x {half[1] * 200:4.1f} "
            f"x {half[2] * 200:4.1f} cm")


def main() -> int:
    profile_path, skin_path = Path(sys.argv[1]), Path(sys.argv[2])
    rig = Rig(profile_path)
    skin = Skin(skin_path, rig)
    zero = np.zeros((len(rig), 3))
    positions, orientations = rig.forward_kinematics(np.zeros(3), quat(), zero)
    vertices = skin.pose(positions, orientations, rest=False)   # mao aberta
    rows = np.arange(len(skin.joints))
    strongest = np.argmax(skin.weights, axis=1)
    dominant_link = skin.bone_link[skin.joints[rows, strongest]]
    dominant_weight = skin.weights[rows, strongest]

    data = json.loads(profile_path.read_text(encoding="utf-8"))
    report = []
    for index, link in enumerate(rig.links):
        part = link.id.removeprefix("Left").removeprefix("Right")
        entry = data["links"][index]
        mesh = vertices[(dominant_link == index) & (dominant_weight > 0.5)]
        if part in TAPERED or part == "Hand":
            report.append(fit_limb(rig, index, mesh, entry["capsule"], part))
        elif part in TORSO:
            report.append(fit_torso(rig, index, mesh, entry["capsule"]))
        elif part == "Foot":
            report.append(fit_foot(rig, index, mesh, entry["box"]))

    # Coxas: afastadas no repouso (so o raio de cima encolhe).
    thighs = [i for i, link in enumerate(rig.links) if link.id.endswith("Thigh")]
    for _ in range(20):
        profile_path.write_text(json.dumps(data, indent=2) + "\n",
                                encoding="utf-8")
        fitted = Rig(profile_path)
        p, o = fitted.forward_kinematics(np.zeros(3), quat(), zero)
        colliders = world_colliders(fitted, p, o)
        gap = collider_gap(colliders[thighs[0]], colliders[thighs[1]])
        if gap >= THIGH_REST_CLEARANCE - 1e-4:
            break
        for i in thighs:
            capsule = data["links"][i]["capsule"]
            shrink = (THIGH_REST_CLEARANCE - gap) / 2.0 + 0.0005
            capsule["radius"] = round(capsule["radius"] - shrink, 6)
            capsule["length"] = round(capsule["length"] - shrink, 6)
    report.append(f"coxas em repouso: folga {gap * 1000:.1f} mm, raio de cima "
                  f"{data['links'][thighs[0]]['capsule']['radius'] * 100:.1f} cm")
    profile_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    print("\n".join(report))
    return 0


if __name__ == "__main__":
    sys.exit(main())
