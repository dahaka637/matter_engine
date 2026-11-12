"""Os colisores da mao do jogador (11/11, ele: "os COLISORES das maos... um cilindro bem simples, que nao leva em consideracao os
DEDOS e a forma da mao... mecanicas de GOLEIRO... palma e dedos"; "ou ate so um colisor por polegar"; "talvez 2 cilindros por dedo se
voce ver que ficaria melhor"; e, com a mao fechando: "Mas os colisores nao acompanham").

Por mao, ajustados a malha do jogador:
- a palma: a casca convexa dos vertices do osso da mao, no link da mao (o colisor principal - quem le so ele, o apoio no chao, ve a
  palma);
- os dedos ("extraColliders" com "bone"): capsulas conicas presas aos ossos visuais dos dedos ("posableBones": a cadeia de cada dedo,
  o repouso relativo ao pai e a rotacao relaxada), que a fisica move com a mesma pose de mao do visual (RagdollVisualPose3D); sem pose
  informada (o treino), a relaxada. Duas por dedo (--segmentos 2): a falange da base no osso 1 e o resto no osso 2. Medido com os dedos
  acompanhando (vertices a ate 3 mm): aberta 96%, relaxada 93%, 50% fechada 96%, punho 97%, o pior a 6 mm; uma por dedo cai a 73%
  meio fechada (pior 3 cm); tres por dedo ~100%, com 50% mais capsulas.
A forma e ajustada na mao aberta (os dedos retos), no referencial de cada osso.

No repouso do corpo (o T) a clavicula ergue o braco visual ~22 graus acima dos links (o carry do ombro, so visual): o ajuste posa a
pele sem ele, com a mao onde o link esta.

uso: colisores_mao.py [--folha PNG] [--fechada F] [--aplicar PERFIL] [--segmentos 1|2|3]
  sem --aplicar so mede (a cobertura em quatro poses) e, com --folha, desenha a conferencia na mao fechada F (0 aberta... 1 punho).
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import ConvexHull

REPO = Path(__file__).resolve().parents[2]
CHAR = REPO / "assets" / "characters" / "football_player"
sys.path.insert(0, str(REPO / "tools" / "animation"))
from matter_rig import load_default_character  # noqa: E402
from preview import render_pose, View, _view_basis  # noqa: E402

FINGERS = ("Thumb", "Index", "Middle", "Ring", "Pinky")
# A mao aberta do motor (src/Engine/Animation/RagdollVisualPose3D.cpp, OpenHand): a flexao (graus) de cada junta e o "close" da base.
OPEN_FLEXION = {"Thumb": (0.0, 4.0, 4.0), "Index": (4.0, 3.0, 2.0), "Middle": (4.0, 3.0, 2.0), "Ring": (4.0, 3.0, 2.0), "Pinky": (4.0, 3.0, 2.0)}
OPEN_CLOSE = {"Thumb": -0.3, "Index": -0.25, "Middle": 0.0, "Ring": -0.25, "Pinky": -0.25}
FIST_FLEXION = {"Thumb": (21.0, 6.0, -2.0), "Index": (85.0, 100.0, 70.0), "Middle": (85.0, 100.0, 70.0), "Ring": (85.0, 100.0, 70.0),
                "Pinky": (85.0, 100.0, 70.0)}
FIST_CLOSE = {"Thumb": 0.3, "Index": 1.0, "Middle": 0.0, "Ring": 1.0, "Pinky": 1.0}
FIST_ACROSS = {"Thumb": (52.0, 27.0)}
PALM_REACH = 0.16  # so os vertices do osso da mao perto do link (a pele do pulso tambem pesa nele)
CONVEX_RADIUS = 0.004
DECIMALS = 6


def qrot(q, v):
    q = np.asarray(q, dtype=float); v = np.asarray(v, dtype=float)
    u, w = q[:3], q[3]
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def qconj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def axis_angle(axis, angle):
    return np.array([*(np.asarray(axis, dtype=float) * math.sin(angle / 2)), math.cos(angle / 2)])


def quat_from_x(axis):
    """O quaternio (x, y, z, w) que leva o X local ao eixo dado."""
    b = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    c = float(b[0])
    if c > 1 - 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0])
    if c < -1 + 1e-9:
        return np.array([0.0, 0.0, 1.0, 0.0])
    v = np.cross([1.0, 0.0, 0.0], b); s = math.sqrt((1 + c) * 2)
    return np.array([v[0] / s, v[1] / s, v[2] / s, s / 2])


def visual_bones():
    manifest = json.loads((CHAR / "character.json").read_text())
    return json.loads((CHAR / manifest["skin"]).read_text())["visualBones"]


def finger_rotation(bone, flexion, close):
    q = axis_angle([0.0, -1.0, 0.0], flexion)
    if bone["joint"] == 1:
        q = qmul(axis_angle([0.0, 0.0, 1.0], bone["closeAngle"] * close), q)
    return q / np.linalg.norm(q)


def hand_rotations(bones, o=1.0, f=0.0):
    """A rotacao de cada osso visual com a mao aberta o e fechada f (0-1), como o motor (RagdollVisualPose3D); os dedos do pe no
    repouso. Confere a formula na relaxada: ela tem de dar a restRotation exportada."""
    out = []
    for b in bones:
        if b.get("kind") != "finger":
            out.append(np.asarray(b.get("restRotation", [0, 0, 0, 1.0]), dtype=float))
            continue
        relaxed = finger_rotation(b, b["flexion"], b["close"]); r = np.asarray(b["restRotation"])
        assert min(np.abs(relaxed - r).max(), np.abs(relaxed + r).max()) < 2e-3, ("a mao relaxada nao bate com a exportada", b["name"])
        j = b["joint"] - 1
        flex = (b["flexion"] + (math.radians(OPEN_FLEXION[b["finger"]][j]) - b["flexion"]) * o
                + (math.radians(FIST_FLEXION[b["finger"]][j]) - b["flexion"]) * f)
        q = axis_angle([0.0, -1.0, 0.0], flex)
        if b["joint"] == 1:
            close = b["close"] + (OPEN_CLOSE[b["finger"]] - b["close"]) * o + (FIST_CLOSE[b["finger"]] - b["close"]) * f
            q = qmul(axis_angle([0.0, 0.0, 1.0], b["closeAngle"] * close), q)
        else:
            # os ossos da esquerda sao o espelho dos da direita: para dentro e o giro ao contrario
            across = FIST_ACROSS.get(b["finger"], (0.0, 0.0))[j - 1] * f * (-1.0 if b["name"].startswith("Left") else 1.0)
            if across:
                q = qmul(axis_angle([0.0, 0.0, 1.0], math.radians(across)), q)
        out.append(q / np.linalg.norm(q))
    return np.array(out)


def posed(rig, skin, visual):
    """O repouso fisico (o T) com a mao aberta e sem o carry da clavicula."""
    skin.helper_carry = [None] * len(skin.helper_carry)
    zero = np.zeros((len(rig), 3))
    p, o = rig.forward_kinematics(np.zeros(3), np.array([0, 0, 0, 1.0]), zero)
    return p, o, skin.pose(p, o, visual=visual)


def farthest_points(points, k):
    chosen = [int(np.argmax(np.linalg.norm(points - points.mean(0), axis=1)))]
    d = np.linalg.norm(points - points[chosen[0]], axis=1)
    while len(chosen) < min(k, len(points)):
        i = int(np.argmax(d)); chosen.append(i); d = np.minimum(d, np.linalg.norm(points - points[i], axis=1))
    return points[chosen]


POSES = (("aberta", 1.0, 0.0), ("relaxada", 0.0, 0.0), ("50% fechada", 0.0, 0.5), ("punho", 0.0, 1.0))
COLORS = {"Thumb": (255, 140, 30), "Index": (60, 170, 255), "Middle": (90, 230, 140), "Ring": (60, 170, 255), "Pinky": (90, 230, 140)}


class Hand:
    """Uma mao: a palma no link e as capsulas dos dedos nos ossos (ajustadas na mao aberta, no referencial de cada osso)."""

    def __init__(self, rig, skin, side, bones_json, segments):
        self.rig, self.skin, self.side, self.bones_json, self.segments = rig, skin, side, bones_json, segments
        self.names = list(skin.bone_names)
        self.dominant = np.asarray(skin.bone)
        self.link = rig.index(f"{side}Hand")
        self.finger_bones = [b for b in bones_json if b.get("kind") == "finger" and b["name"].startswith(side)]
        self._fit()

    def posed(self, o, f):
        rots = hand_rotations(self.bones_json, o, f)
        p, q, verts = posed(self.rig, self.skin, rots)
        bp, bq = self.skin.bone_transforms(p, q, visual=rots)
        return p, q, verts, bp, bq, rots

    def _fit(self):
        p, q, verts, bp, bq, _ = self.posed(1.0, 0.0)
        hand_p, hand_q = p[self.link], q[self.link]
        local = np.array([qrot(qconj(hand_q), v - hand_p) for v in verts])
        palm = local[self.dominant == self.names.index(f"{self.side}Hand")]
        palm = palm[np.linalg.norm(palm, axis=1) < PALM_REACH]
        pts = palm[ConvexHull(palm).vertices]
        if len(pts) > 64:
            pts = farthest_points(pts, 64)
        hull = ConvexHull(pts)
        self.palm_center = pts.mean(0)
        self.palm_points, self.palm_faces = pts - self.palm_center, hull.simplices
        self.capsules = []  # o osso, as pontas (no referencial do osso), os raios e o dedo
        for f in FINGERS:
            idx = [self.names.index(f"{self.side}{f}{k}") for k in (1, 2, 3)]
            joints = [bp[i] for i in idx]
            pts_f = verts[np.isin(self.dominant, idx)]
            last = joints[2] - joints[1]
            tip = pts_f[np.argmax((pts_f - joints[2]) @ (last / np.linalg.norm(last)))]
            pieces = {1: [(idx[0], joints[0], tip)], 2: [(idx[0], joints[0], joints[1]), (idx[1], joints[1], tip)],
                      3: [(idx[0], joints[0], joints[1]), (idx[1], joints[1], joints[2]), (idx[2], joints[2], tip)]}[self.segments]
            for n, (bone, a, b) in enumerate(pieces):
                axis = (b - a) / np.linalg.norm(b - a)
                L = float(np.linalg.norm(b - a))
                t = (pts_f - a) @ axis
                radial = np.linalg.norm((pts_f - a) - np.outer(t, axis), axis=1)
                last_piece = n == len(pieces) - 1
                near = (t > -0.01) & (t < L + (0.03 if last_piece else 0.005))
                lo, hi = near & (t < 0.5 * L), near & (t >= 0.5 * L)
                r0 = float(np.percentile(radial[lo], 90)) if lo.sum() >= 3 else float(np.percentile(radial[near], 90))
                r1 = float(np.percentile(radial[hi], 90)) if hi.sum() >= 3 else r0
                end = b - axis * r1 if last_piece else b  # a tampa da ponta cobre a ponta da malha
                to_bone = lambda w: qrot(qconj(bq[bone]), w - bp[bone])
                self.capsules.append({"bone": self.names[bone], "a": to_bone(a), "b": to_bone(end), "r0": r0, "r1": r1, "finger": f})

    def rest_bones(self):
        """A cadeia dos ossos dos dedos para o perfil: o repouso de cada um relativo ao pai (o link da mao: ""), sem rotacao, e a
        rotacao relaxada (a de quando nada informa a pose)."""
        identity = np.tile([0.0, 0.0, 0.0, 1.0], (len(self.bones_json), 1))
        p, q, _ = posed(self.rig, self.skin, identity)
        bp, bq = self.skin.bone_transforms(p, q, visual=identity)
        out = []
        for b in self.finger_bones:
            k = self.names.index(b["name"])
            parent = "" if b["parent"] == f"{self.side}Hand" else b["parent"]
            pp, pq = (p[self.link], q[self.link]) if parent == "" else (bp[self.names.index(parent)], bq[self.names.index(parent)])
            out.append({"name": b["name"], "parent": parent, "position": qrot(qconj(pq), bp[k] - pp),
                        "orientation": qmul(qconj(pq), bq[k]), "restRotation": np.asarray(b["restRotation"], dtype=float)})
        return out

    def placed(self, o, f):
        """No mundo, na pose (o, f): a palma, as capsulas (a, b, r0, r1, dedo) e a malha."""
        p, q, verts, bp, bq, rots = self.posed(o, f)
        caps = []
        for c in self.capsules:
            k = self.names.index(c["bone"])
            caps.append((bp[k] + qrot(bq[k], c["a"]), bp[k] + qrot(bq[k], c["b"]), c["r0"], c["r1"], c["finger"]))
        palm = p[self.link] + np.array([qrot(q[self.link], v + self.palm_center) for v in self.palm_points])
        return p, q, verts, caps, palm, rots

    def coverage(self, o, f):
        p, q, verts, caps, palm, _ = self.placed(o, f)
        ids = [self.names.index(f"{self.side}Hand")] + [self.names.index(f"{self.side}{fi}{k}") for fi in FINGERS for k in (1, 2, 3)]
        region = verts[np.isin(self.dominant, ids)]
        region = region[np.linalg.norm(region - p[self.link], axis=1) < PALM_REACH]
        hull = ConvexHull(palm)
        d = (region @ hull.equations[:, :3].T + hull.equations[:, 3]).max(1)
        for a, b, r0, r1, _ in caps:
            ax = b - a; L = max(1e-9, float(np.linalg.norm(ax))); ax = ax / L
            t = np.clip((region - a) @ ax, 0, L)
            d = np.minimum(d, np.linalg.norm(region - (a + np.outer(t, ax)), axis=1) - (r0 + (r1 - r0) * t / L))
        return {"inside_3mm": float((d <= 0.003).mean()), "p95_cm": float(np.percentile(d, 95) * 100), "max_cm": float(d.max() * 100),
                "outside": region[d > 0.003]}


def rounded(values):
    return [round(float(v), DECIMALS) for v in values]


def profile_entries(hand, previous):
    """As chaves do perfil para a mao: a palma ("hull"), os ossos dos dedos ("posableBones") e as capsulas presas a eles
    ("extraColliders" com "bone"); o material e o sensor de contato continuam os do colisor de antes."""
    material, sensor = previous.get("material", "body"), previous.get("contactSensor", False)
    hull = {"contactSensor": sensor, "convexRadius": CONVEX_RADIUS, "faces": [[int(k) for k in f] for f in hand.palm_faces],
            "material": material, "orientation": [0.0, 0.0, 0.0, 1.0], "points": [rounded(q) for q in hand.palm_points],
            "position": rounded(hand.palm_center)}
    bones = [{"name": b["name"], "orientation": rounded(b["orientation"] / np.linalg.norm(b["orientation"])), "parent": b["parent"],
              "position": rounded(b["position"]), "restRotation": rounded(b["restRotation"])} for b in hand.rest_bones()]
    extras = []
    for c in hand.capsules:
        axis = c["b"] - c["a"]
        extras.append({"bone": c["bone"], "capsule": {
            "length": round(float(np.linalg.norm(axis)) + c["r0"] + c["r1"], DECIMALS), "material": material,
            "orientation": rounded(quat_from_x(axis)), "position": rounded(0.5 * (c["a"] + c["b"])),
            "radius": round(c["r0"], DECIMALS), "radiusAtPositiveX": round(c["r1"], DECIMALS)}})
    return hull, bones, extras


def apply(profile_path, hands):
    """Troca, no texto do perfil, o colisor de cada mao (o primeiro bloco do link, ate a chave "id") pelos novos, no mesmo recuo; o
    resto do arquivo fica byte a byte. Confere que o JSON resultante e o de antes com so as maos trocadas."""
    text = Path(profile_path).read_text()
    before = json.loads(text)
    expected = json.loads(text)
    for hand in hands:
        link_id = f"{hand.side}Hand"
        entry = next(e for e in expected["links"] if e["id"] == link_id)
        previous = entry.get("capsule") or entry.get("hull")
        hull, bones, extras = profile_entries(hand, previous)
        for key in ("capsule", "box", "hull", "extraColliders", "posableBones"):
            entry.pop(key, None)
        entry.update({"extraColliders": extras, "hull": hull, "posableBones": bones})
        at = text.index(f'"id": "{link_id}"')
        start = text.rindex("\n  {\n", 0, at) + len("\n  {\n")
        if not re.match(r'^   "(capsule|extraColliders|hull|posableBones)": ', text[start:at]):
            raise SystemExit(f"{link_id}: o link nao comeca pelo colisor")
        indent = "   "
        block = json.dumps({"extraColliders": extras, "hull": hull, "posableBones": bones}, indent=1, sort_keys=True, ensure_ascii=False)
        inner = "\n".join(indent + line[1:] if line.startswith(" ") else line for line in block.splitlines()[1:-1])
        text = text[:start] + inner + ",\n" + indent + text[at:]
    after = json.loads(text)
    assert after == expected, "o perfil escrito nao e o esperado"
    assert sum(1 for x, y in zip(before["links"], after["links"]) if x != y) == len(hands), "mudou outro link"
    Path(profile_path).write_text(text)


def draw_sheet(rig, skin, hands, closed, out_png):
    views = [View("palma (de baixo)", (0.0, 0.0, 1.0)), View("dorso (de cima)", (0.0, 0.0, -1.0)), View("frente", (-1.0, 0.0, 0.0)),
             View("3/4", (-0.6, 0.0, 0.8))]
    size, extent = 420, 0.30
    sheet = Image.new("RGB", (size * len(views), size * len(hands) + 24), (20, 22, 26))
    pose_with_hand = skin.pose
    o = 1.0 if closed <= 0.0 else 0.0
    try:
        for r, hand in enumerate(hands):
            p, q, _, caps, palm, rots = hand.placed(o, closed)
            outside = hand.coverage(o, closed)["outside"]
            # O preview posa sem a pose dos dedos: a mesma mao no desenho.
            skin.pose = lambda pp, oo, rest=True, visual=None: pose_with_hand(pp, oo, rest, rots if visual is None else visual)
            center = palm.mean(0) + qrot(q[hand.link], np.array([0.0, 0.035 if hand.side == "Left" else -0.035, 0.0]))
            for c, view in enumerate(views):
                label = f"{hand.side} {view.name} (mao {'aberta' if o else f'{int(100 * closed)}% fechada'})"
                im = render_pose(rig, skin, p, q, view, size=size, extent=extent, label=label, center=center)
                basis = _view_basis(view); scale = size / extent; draw = ImageDraw.Draw(im)
                proj = lambda w: tuple(((np.asarray(w) - center) @ basis.T)[:2] * np.array([scale, -scale]) + size * 0.5)
                for w in outside:
                    x, y = proj(w); draw.ellipse([x - 2.5, y - 2.5, x + 2.5, y + 2.5], fill=(255, 40, 40))
                for face in ConvexHull(palm).simplices:
                    for i, j in ((0, 1), (1, 2), (2, 0)):
                        draw.line([proj(palm[face[i]]), proj(palm[face[j]])], fill=(240, 220, 40), width=1)
                for a, b, r0, r1, finger in caps:
                    ax = (b - a) / max(1e-9, np.linalg.norm(b - a))
                    ref = np.array([0, 0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0, 0])
                    u = np.cross(ax, ref); u /= np.linalg.norm(u); v = np.cross(ax, u)
                    for t in (0.0, 1.0):
                        rr = r0 + (r1 - r0) * t
                        draw.line([proj(a + (b - a) * t + rr * (math.cos(k) * u + math.sin(k) * v)) for k in np.linspace(0, 2 * math.pi, 17)],
                                  fill=COLORS[finger], width=1)
                    for k in np.linspace(0, 2 * math.pi, 5)[:-1]:
                        ring = math.cos(k) * u + math.sin(k) * v
                        draw.line([proj(a + r0 * ring), proj(b + r1 * ring)], fill=COLORS[finger], width=1)
                        for end, sign, rr in ((a, -1, r0), (b, 1, r1)):
                            draw.line([proj(end + rr * math.cos(t) * ring + sign * ax * rr * math.sin(t)) for t in np.linspace(0, math.pi / 2, 7)],
                                      fill=COLORS[finger], width=1)
                sheet.paste(im, (c * size, 24 + r * size))
    finally:
        skin.pose = pose_with_hand
    ImageDraw.Draw(sheet).text((8, 6), f"dedos presos aos ossos ({hands[0].segments} capsulas por dedo): palma (amarelo), polegar (laranja), "
                                       "dedos (azul/verde); vermelho = vertice a mais de 3 mm fora", fill=(230, 230, 230))
    sheet.save(out_png)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--folha", help="o .png de conferencia (a malha com os colisores por cima, 4 vistas por mao)")
    ap.add_argument("--fechada", type=float, default=0.5, help="a mao da folha: 0 aberta, ate 1 o punho")
    ap.add_argument("--aplicar", metavar="PERFIL", help="grava no perfil (o .ragdoll.json) a palma, os ossos e os dedos das duas maos")
    ap.add_argument("--segmentos", type=int, choices=(1, 2, 3), default=2, help="capsulas por dedo")
    args = ap.parse_args()
    rig, skin = load_default_character()
    bones_json = visual_bones()
    hands = [Hand(rig, skin, side, bones_json, args.segmentos) for side in ("Left", "Right")]
    for hand in hands:
        cells = []
        for label, o, f in POSES:
            cov = hand.coverage(o, f)
            cells.append(f"{label} {100 * cov['inside_3mm']:.0f}% (pior {cov['max_cm']:.1f} cm)")
        print(f"{hand.side}: palma {len(hand.palm_points)} pontos, {len(hand.capsules)} capsulas; cobertura a ate 3 mm: " + "; ".join(cells))
    if args.folha:
        draw_sheet(rig, skin, hands, args.fechada, args.folha)
        print("folha:", args.folha)
    if args.aplicar:
        apply(args.aplicar, hands)
        print("aplicado:", args.aplicar)


if __name__ == "__main__":
    main()
