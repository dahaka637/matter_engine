"""Poses de levantar a partir de apoios (base do levantar desenhado, 04/10).

Uma pose e: a pelve (posicao e inclinacao), a curva da coluna e, para cada
membro, onde ele apoia no chao (ou para onde vai). Os membros sao resolvidos
por cinematica inversa (minimos quadrados com os limites das juntas); a pose
devolve tambem o centro de massa e o poligono de apoio, para conferir que
cada passo e estatico (o centro de massa sobre os apoios que ficam).

Convencoes do rig (FootballPlayerV1): coxa swing1 negativo = flexao do
quadril, swing2 negativo = abrir (esquerda); joelho/cotovelo twist positivo =
dobrar; pe twist positivo = ponta para baixo; coluna swing1 positivo =
flexao; braco swing2 negativo = para a frente, swing1 positivo = para baixo.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from matter_rig import Rig, qconj, qfrom_axis_angle, qmul, qnormalize, qrotate  # noqa: E402

import stand_up_getup as g  # noqa: E402

UP = np.array([0.0, 0.0, 1.0])
HAND_REST = 0.05    # punho sobre o chao com a palma apoiada
KNEE_REST = 0.07    # centro do joelho sobre o chao, ajoelhado
SPINE_LINKS = ("Abdomen", "Chest", "UpperChest")
SPINE_SHARES = (0.36, 0.34, 0.30)


def pitch_roll_rotation(pitch: float, roll: float, yaw: float = 0.0) -> np.ndarray:
    """Rotacao da raiz: inclina para a frente (pitch, em torno de y) e de lado
    (roll, em torno de x), no rumo yaw."""
    q = qmul(qfrom_axis_angle(np.array([0.0, 1.0, 0.0]), pitch),
             qfrom_axis_angle(np.array([1.0, 0.0, 0.0]), roll))
    return qnormalize(qmul(qfrom_axis_angle(UP, yaw), q))


@dataclass
class Limb:
    """Alvo de um membro. kind: 'plant' (apoiado em `point`), 'move' (o ponto
    vai a `point`, no ar) ou 'pose' (so os angulos dados)."""
    kind: str = "pose"
    point: np.ndarray | None = None
    # mao: palma no chao; pe: 'flat' (sola no chao), 'toes' (dedos dobrados,
    # apoio na ponta), 'point' (peito do pe no chao, deitado), None (livre).
    foot: str | None = None
    weight: float = 1.0


@dataclass
class Pose:
    root: np.ndarray
    rotation: np.ndarray
    spine: float = 0.0          # flexao total da coluna (rad)
    spine_side: float = 0.0
    neck: float = 0.0           # flexao do pescoco + cabeca (rad)
    limbs: dict = field(default_factory=dict)  # 'LeftHand', 'LeftKnee', 'LeftFoot', ...
    base: np.ndarray | None = None  # angulos de partida (o quadro anterior)


class Solver:
    def __init__(self, rig: Rig):
        self.rig = rig
        self.floor = float(rig.links[0].model_position[2]) - rig.standing_root_height
        self.masses = np.array([l.mass_fraction for l in rig.links])
        self.masses = self.masses / self.masses.sum()
        self.idx = {l.id: i for i, l in enumerate(rig.links)}

    # --- geometria ------------------------------------------------------------
    def fk(self, pose_root, rotation, coordinates):
        return self.rig.forward_kinematics(pose_root, rotation, coordinates)

    def joint(self, p, o, child: str) -> np.ndarray:
        return g.joint_world(self.rig, p, o, child)

    def wrist(self, p, o, side: str) -> np.ndarray:
        return g.wrist_world(self.rig, p, o, side)

    def palm(self, p, o, side: str) -> np.ndarray:
        """Centro da palma (centro do colisor da mao)."""
        h = self.idx[f"{side}Hand"]
        return p[h] + qrotate(o[h], self.rig.links[h].collider.local_position)

    def com(self, p, o) -> np.ndarray:
        c = np.zeros(3)
        for i, link in enumerate(self.rig.links):
            c += self.masses[i] * (p[i] + qrotate(o[i], link.center_of_mass_local))
        return c

    def foot_points(self, p, o, side: str) -> dict:
        """Ponta, bola (dedos), calcanhar da sola, no mundo."""
        f = self.idx[f"{side}Foot"]
        link = self.rig.links[f]
        c, h = link.collider.local_position, link.collider.half_extents
        lo = link.collider.local_orientation
        pts = {}
        for name, x in (("toe", h[0]), ("ball", h[0] * 0.55), ("heel", -h[0])):
            local = c + qrotate(lo, np.array([x, 0.0, -h[2]]))
            pts[name] = p[f] + qrotate(o[f], local)
        return pts

    def lowest(self, p, o, links) -> float:
        from validate import lowest_point
        return lowest_point(self.rig, p, o, links)

    def depth(self, p, o, links) -> np.ndarray:
        """Quanto cada link atravessa o chao (0 se nao atravessa)."""
        return np.array([max(0.0, self.floor - self.lowest(p, o, [l])) for l in links])

    # --- cinematica inversa ---------------------------------------------------
    def _solve(self, root, rotation, coordinates, free, residual, iterations=10, drift=0.03):
        rest = coordinates.copy()
        c = coordinates.copy()
        values = np.array([c[l][a] for l, a in free])

        def full(vals):
            trial = c.copy()
            for (l, a), v in zip(free, vals):
                trial[l][a] = v
            p, o = self.fk(root, rotation, trial)
            r = residual(p, o)
            reg = np.array([trial[l][a] - rest[l][a] for l, a in free]) * drift
            return np.concatenate([r, reg])

        lam = 1e-3
        for _ in range(iterations):
            r = full(values)
            jac = np.zeros((len(r), len(values)))
            for k in range(len(values)):
                probe = values.copy()
                probe[k] += 1e-4
                jac[:, k] = (full(probe) - r) / 1e-4
            a = jac.T @ jac + lam * np.eye(len(values))
            step = np.linalg.solve(a, -jac.T @ r)
            values = values + step
            for k, (l, ax) in enumerate(free):
                lim = self.rig.links[l].axes[ax]
                values[k] = min(max(values[k], lim.minimum), lim.maximum)
            if np.linalg.norm(step) < 1e-5:
                break
        for (l, a), v in zip(free, values):
            c[l][a] = v
        return c

    def _free(self, *names):
        out = []
        for n in names:
            i = self.idx[n]
            out += [(i, a) for a in range(3) if self.rig.links[i].axes[a].enabled]
        return out

    def solve_arm(self, pose: Pose, coordinates, side: str, limb: Limb):
        hand = self.idx[f"{side}Hand"]
        arm_links = [f"{side}UpperArm", f"{side}Forearm"]
        free = self._free(f"{side}UpperArm", f"{side}Forearm")

        def reach(c, wrist_target):
            def residual(p, o):
                return np.concatenate([(self.wrist(p, o, side) - wrist_target) * 4.0,
                                       self.depth(p, o, arm_links) * 6.0])
            return self._solve(pose.root, pose.rotation, c, free, residual)

        if limb.kind != "plant":
            return reach(coordinates, limb.point)
        # Apoiada: o punho sobre o ponto; a mao o mais plana que o punho deixa
        # (com o antebraco em pe, ~40 graus) e encostando no chao - o punho
        # sobe ou desce ate a mao tocar.
        lift = 0.08
        c = coordinates
        hl = self.rig.links[hand]
        for _ in range(4):
            c = reach(c, limb.point + np.array([0.0, 0.0, lift]))
            best = None
            for tw in np.linspace(hl.axes[0].minimum, hl.axes[0].maximum, 15):
                for s1 in np.linspace(hl.axes[1].minimum, hl.axes[1].maximum, 7):
                    trial = c.copy()
                    trial[hand] = [tw, s1, 0.0]
                    p, o = self.fk(pose.root, pose.rotation, trial)
                    score = qrotate(o[hand], UP)[2]
                    if best is None or score > best[0]:
                        best = (score, trial[hand].copy())
            c = c.copy()
            c[hand] = best[1]
            p, o = self.fk(pose.root, pose.rotation, c)
            gap = self.lowest(p, o, [f"{side}Hand"]) - self.floor
            if abs(gap) < 0.004:
                break
            lift -= gap
        return c

    def solve_leg(self, pose: Pose, coordinates, side: str, knee: Limb | None, foot: Limb | None):
        f = self.idx[f"{side}Foot"]
        free = self._free(f"{side}Thigh", f"{side}Shin", f"{side}Foot")

        leg_links = [f"{side}Thigh", f"{side}Shin", f"{side}Foot"]

        def residual(p, o):
            r = [self.depth(p, o, leg_links) * 6.0]
            if knee is not None and knee.point is not None:
                r.append((self.joint(p, o, f"{side}Shin") - knee.point) * 3.0 * knee.weight)
            if foot is not None and foot.point is not None:
                r.append((self.joint(p, o, f"{side}Foot") - foot.point) * 3.0 * foot.weight)
            if foot is not None and foot.foot is not None:
                pts = self.foot_points(p, o, side)
                up = qrotate(o[f], UP)
                if foot.foot == "flat":
                    r.append((up - UP) * 0.8)
                    r.append(np.array([pts["toe"][2] - self.floor, pts["heel"][2] - self.floor]) * 2.0)
                elif foot.foot == "toes":
                    # dedos dobrados: a bola do pe no chao, calcanhar para cima
                    r.append(np.array([pts["ball"][2] - self.floor, pts["toe"][2] - self.floor]) * 2.0)
                elif foot.foot == "point":
                    # peito do pe no chao: a sola para cima/tras
                    r.append(np.array([max(0.0, self.floor + 0.02 - min(pts["toe"][2], pts["heel"][2]))]) * 2.0)
            return np.concatenate(r)

        return self._solve(pose.root, pose.rotation, coordinates, free, residual)

    # --- pose inteira ---------------------------------------------------------
    def build(self, pose: Pose, start: np.ndarray) -> np.ndarray:
        c = start.copy()
        for name, share in zip(SPINE_LINKS, SPINE_SHARES):
            i = self.idx[name]
            c[i] = self.rig.clamp(i, np.array([0.0, pose.spine * share, pose.spine_side * share]))
        for name, share in (("Neck", 0.5), ("Head", 0.5)):
            i = self.idx[name]
            c[i] = self.rig.clamp(i, np.array([0.0, pose.neck * share, 0.0]))
        for side in ("Left", "Right"):
            hand = pose.limbs.get(f"{side}Hand")
            if hand is not None and hand.point is not None:
                c = self.solve_arm(pose, c, side, hand)
            knee = pose.limbs.get(f"{side}Knee")
            foot = pose.limbs.get(f"{side}Foot")
            if (knee is not None and knee.point is not None) or foot is not None:
                c = self.solve_leg(pose, c, side, knee, foot)
        return c

    def report(self, pose: Pose, c: np.ndarray) -> dict:
        p, o = self.fk(pose.root, pose.rotation, c)
        out = {"com": self.com(p, o)}
        support = []
        err = {}
        for side in ("Left", "Right"):
            hand = pose.limbs.get(f"{side}Hand")
            if hand is not None and hand.point is not None:
                if hand.kind == "plant":
                    palm = self.palm(p, o, side)
                    err[f"{side}Hand"] = float(np.hypot(*(self.wrist(p, o, side)[:2] - hand.point[:2])))
                    err[f"{side}HandZ"] = float(self.lowest(p, o, [f"{side}Hand"]) - self.floor)
                    support.append(palm)
                else:
                    err[f"{side}Hand"] = float(np.linalg.norm(self.wrist(p, o, side) - hand.point))
            knee = pose.limbs.get(f"{side}Knee")
            if knee is not None and knee.point is not None:
                err[f"{side}Knee"] = float(np.linalg.norm(self.joint(p, o, f"{side}Shin") - knee.point))
                if knee.kind == "plant":
                    support.append(self.joint(p, o, f"{side}Shin"))
            foot = pose.limbs.get(f"{side}Foot")
            if foot is not None and foot.kind == "plant":
                pts = self.foot_points(p, o, side)
                for name in ("toe", "heel", "ball"):
                    if pts[name][2] - self.floor < 0.04:
                        support.append(pts[name])
                if foot.point is not None:
                    err[f"{side}Foot"] = float(np.linalg.norm(self.joint(p, o, f"{side}Foot") - foot.point))
        out["support"] = support
        out["margin"] = hull_margin([s[:2] for s in support], out["com"][:2]) if len(support) else -1.0
        out["err"] = err
        body = [l.id for l in self.rig.links]
        depth = self.depth(p, o, body)
        out["penetration"] = float(depth.max())
        out["worst"] = body[int(depth.argmax())] if depth.max() > 0.005 else "-"
        out["positions"] = p
        out["orientations"] = o
        return out


def hull_margin(points, q) -> float:
    pts = sorted({(float(a), float(b)) for a, b in points})
    if len(pts) < 3:
        if not pts:
            return -1.0
        d = min(math.hypot(q[0] - a, q[1] - b) for a, b in pts)
        return -d

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for pt in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], pt) <= 0:
            lower.pop()
        lower.append(pt)
    for pt in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], pt) <= 0:
            upper.pop()
        upper.append(pt)
    hull = lower[:-1] + upper[:-1]
    inside = True
    best = 1e9
    for k in range(len(hull)):
        a, b = hull[k], hull[(k + 1) % len(hull)]
        if cross(a, b, q) < 0:
            inside = False
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / length)) if length > 0 else 0.0
        best = min(best, math.hypot(q[0] - a[0] - dx * t, q[1] - a[1] - dy * t))
    return best if inside else -best
