"""Previa renderizada de poses e clipes: folhas de contato em PNG.

Renderiza a malha real do personagem (skin com pesos, mesma matematica do motor)
em projecao ortografica, sem dependencia de GPU nem do aplicativo. Serve para
quem autora olhar o resultado - pose por pose, de varios lados - antes de
qualquer coisa chegar ao visualizador do jogo.

Segmentos que violam a folga de autocolisao saem em vermelho: e o que o
visualizador do jogo nao mostra, porque ele desenha o clipe sem fisica.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from PIL import Image, ImageDraw

from matter_rig import Rig, Skin
from validate import self_collision_gaps


@dataclass(frozen=True)
class View:
    name: str
    # Direcao DA camera PARA o personagem, no espaco de modelo (Z-up, o personagem
    # olha para +X, esquerda e +Y).
    direction: tuple[float, float, float]


FRONT = View("frente", (-1.0, 0.0, 0.0))
BACK = View("costas", (1.0, 0.0, 0.0))
LEFT = View("perfil esq.", (0.0, -1.0, 0.0))
RIGHT = View("perfil dir.", (0.0, 1.0, 0.0))
THREE_QUARTER = View("3/4", (-0.7, -0.55, -0.35))
TOP = View("topo", (0.0, 0.0, -1.0))
STANDARD_VIEWS = (FRONT, LEFT, THREE_QUARTER, BACK)

_SKIN_COLOR = np.array([196, 188, 176], dtype=np.float64)
_ALERT_COLOR = np.array([214, 64, 52], dtype=np.float64)
_BACKGROUND = (34, 36, 40)
_GROUND = (82, 86, 92)
_LIGHT = np.array([-0.45, -0.35, 0.82])


def _view_basis(view: View) -> np.ndarray:
    forward = np.array(view.direction, dtype=np.float64)
    forward /= np.linalg.norm(forward)
    up_hint = np.array([0.0, 0.0, 1.0]) if abs(forward[2]) < 0.95 \
        else np.array([1.0, 0.0, 0.0])
    right = np.cross(forward, up_hint)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return np.stack([right, up, forward])


def render_pose(rig: Rig, skin: Skin, positions: np.ndarray,
                orientations: np.ndarray, view: View, size: int = 360,
                extent: float = 2.2, floor_z: float | None = None,
                alert_links: set[str] | None = None,
                label: str = "",
                center: np.ndarray | None = None) -> Image.Image:
    """Uma vista da pose. extent: altura do mundo coberta pela imagem (m);
    center: ponto do mundo no meio da imagem (padrao: corpo inteiro)."""
    vertices = skin.pose(positions, orientations)
    basis = _view_basis(view)
    if center is None:
        center = np.array([positions[0][0], positions[0][1], -0.05])
    projected = (vertices - center) @ basis.T
    scale = size / extent
    xs = size * 0.5 + projected[:, 0] * scale
    ys = size * 0.5 - projected[:, 1] * scale
    depth = projected[:, 2]

    triangles = skin.indices
    a, b, c = (vertices[triangles[:, k]] for k in range(3))
    normals = np.cross(b - a, c - a)
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1e-12
    normals[valid] /= lengths[valid, None]
    light = _LIGHT / np.linalg.norm(_LIGHT)
    # A skin nao garante orientacao das faces para o nosso lado; iluminacao de
    # duas faces evita triangulos pretos sem mudar a leitura da forma.
    shade = 0.28 + 0.72 * np.abs(normals @ light)
    order = np.argsort(-depth[triangles].mean(axis=1))

    triangle_link = skin.bone_link[skin.bone[triangles[:, 0]]]
    alert = np.zeros(len(triangles), dtype=bool)
    if alert_links:
        alert_indices = [rig.index(name) for name in alert_links]
        alert = np.isin(triangle_link, alert_indices)

    image = Image.new("RGB", (size, size), _BACKGROUND)
    draw = ImageDraw.Draw(image)
    if floor_z is not None and abs(basis[1][2]) > 0.3:
        floor_y = size * 0.5 - ((np.array([0, 0, floor_z]) - center)
                                @ basis[1]) * scale
        draw.line([(0, floor_y), (size, floor_y)], fill=_GROUND, width=2)
    for t in order:
        if not valid[t]:
            continue
        color = (_ALERT_COLOR if alert[t] else _SKIN_COLOR) * shade[t]
        i0, i1, i2 = triangles[t]
        draw.polygon([(xs[i0], ys[i0]), (xs[i1], ys[i1]), (xs[i2], ys[i2])],
                     fill=tuple(int(v) for v in color))
    caption = f"{view.name}  {label}".strip()
    draw.text((8, 6), caption, fill=(230, 230, 230))
    return image


def contact_sheet(rig: Rig, skin: Skin, poses: list[tuple[str, np.ndarray, np.ndarray]],
                  views=STANDARD_VIEWS, size: int = 320,
                  floor_z: float | None = None,
                  check_self_collision: bool = True) -> Image.Image:
    """Linhas = poses (rotulo, posicoes, orientacoes); colunas = vistas."""
    rows = []
    for label, positions, orientations in poses:
        alert: set[str] = set()
        if check_self_collision:
            for a, b, gap, required in self_collision_gaps(
                    rig, positions, orientations):
                if gap < required - 1e-4:
                    alert.update((a, b))
        rows.append([render_pose(rig, skin, positions, orientations, view,
                                 size=size, floor_z=floor_z,
                                 alert_links=alert, label=label)
                     for view in views])
    sheet = Image.new("RGB", (size * len(views), size * len(rows)), _BACKGROUND)
    for r, row in enumerate(rows):
        for c, image in enumerate(row):
            sheet.paste(image, (c * size, r * size))
    return sheet
