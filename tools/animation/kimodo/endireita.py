"""Endireita uma corrida do Kimodo (npz, SOMA77) pelas regras do usuario (24/10): linha reta, nada para os lados.

Quadro a quadro, no referencial do caminho (a frente e o rumo do quadril do inicio ao fim):
  - o caminho vira uma reta: a deriva lenta do quadril para o lado (a media de uma passada, 0,5 s) sai da raiz; o
    balanco de cada passo fica;
  - o giro do quadril e o dos ombros em volta da vertical caem para a fracao --giro (o quadril pela raiz; os ombros
    pelo peito, em volta do eixo do tronco);
  - os bracos: cada segmento (braco, antebraco) com no maximo a fracao --bracos da direcao para o lado (o balanco
    para a frente e para tras fica o mesmo);
  - as pernas: o joelho e o pe numa linha fixa de cada lado do caminho (--passada m do centro), a frente e a altura
    as do Kimodo; o pe chega exato por IK de duas juntas (sem escorregar para os lados); no apoio (o pe a 3 cm do
    chao), o tornozelo e o dedo ficam parados na frente do meio do apoio (sem escorregar para a frente), com um quadro
    de transicao antes e depois;
  - com --coluna-reta, a coluna, o pescoco e a cabeca numa linha do quadril ao pescoco (a inclinacao do tronco fica;
    a curvatura sai); sem ela, como vieram. Os dedos ficam como vieram.
Grava o npz corrigido com as rotacoes locais, globais e as juntas recalculadas pela cinematica direta do SOMA (a mesma
do Kimodo: confere 0,0 mm), e a interface mostra igual.

uso: endireita.py entrada.npz saida.npz [--giro 0.25] [--bracos 0.15] [--passada 0.09] [--coluna-reta]
"""
import sys

import numpy as np

import sprint_chaves as sc
from soma import IX, NAMES

UP = np.array([0.0, 1.0, 0.0])


def unit(v):
    return v / max(np.linalg.norm(v), 1e-9)


def rotation(axis, angle):
    a = unit(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def align(a, b):
    a, b = unit(a), unit(b)
    v, c = np.cross(a, b), float(np.dot(a, b))
    if c < -0.999999:
        axis = unit(np.cross(a, [1.0, 0, 0]) if abs(a[0]) < 0.9 else np.cross(a, [0, 1.0, 0]))
        return 2 * np.outer(axis, axis) - np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


class Body:
    def __init__(self, parents, offsets, local, root):
        self.parents, self.offsets = parents, offsets
        self.local, self.root = local.copy(), root.copy()
        self.update()

    def update(self):
        n = len(self.parents)
        self.world = np.zeros((n, 3, 3))
        self.pos = np.zeros((n, 3))
        for j, p in enumerate(self.parents):
            if p < 0:
                self.world[j], self.pos[j] = self.local[j], self.root
            else:
                self.world[j] = self.world[p] @ self.local[j]
                self.pos[j] = self.pos[p] + self.world[p] @ self.offsets[j]

    def turn(self, joint, rot):
        """Gira a junta (e tudo abaixo) por rot no mundo."""
        j = IX[joint]
        p = self.parents[j]
        new_world = rot @ self.world[j]
        self.local[j] = new_world if p < 0 else self.world[p].T @ new_world
        self.update()

    def aim(self, joint, child, target):
        """Gira a junta para o filho (pode ser neto) apontar para o alvo."""
        j, c = IX[joint], IX[child]
        self.turn(joint, align(self.pos[c] - self.pos[j], target - self.pos[j]))


def yaw(vector, lateral, forward):
    return np.arctan2(vector @ forward, vector @ lateral)


def two_bone(hip, knee, ankle, target, pole):
    """O joelho para a perna (comprimentos atuais) chegar ao alvo, no plano do alvo e do polo."""
    l1, l2 = np.linalg.norm(knee - hip), np.linalg.norm(ankle - knee)
    d = target - hip
    dist = np.clip(np.linalg.norm(d), 1e-6, l1 + l2 - 1e-5)
    axis = d / np.linalg.norm(d)
    along = (l1 * l1 - l2 * l2 + dist * dist) / (2 * dist)
    height = np.sqrt(max(l1 * l1 - along * along, 0.0))
    bend = pole - hip
    bend = unit(bend - axis * np.dot(bend, axis))
    return hip + axis * along + bend * height, hip + axis * dist


def main():
    args = sys.argv[1:]
    straight_spine = "--coluna-reta" in args
    args = [a for a in args if a != "--coluna-reta"]
    opts = {"--giro": "0.25", "--bracos": "0.15", "--passada": "0.09"}
    for key in list(opts):
        if key in args:
            i = args.index(key)
            opts[key] = args[i + 1]
            del args[i:i + 2]
    source, out = args
    keep, lateral_share, width = float(opts["--giro"]), float(opts["--bracos"]), float(opts["--passada"])
    data = dict(np.load(source))
    parents, offsets = sc.skeleton()
    local, roots = data["local_rot_mats"].astype(float), data["root_positions"].astype(float)
    hips = data["posed_joints"][:, IX["Hips"]]
    forward = hips[-1] - hips[0]
    forward[1] = 0
    forward = unit(forward)
    lateral = np.cross(UP, forward)  # a esquerda
    center = hips[0]
    # a deriva para o lado (a media movel de 0,5 s do quadril no eixo lateral) sai da raiz: o caminho vira uma reta
    side = (hips - center) @ lateral
    half = 7
    padded = np.pad(side, half, mode="edge")
    drift = np.convolve(padded, np.ones(2 * half + 1) / (2 * half + 1), mode="valid")
    roots = roots - np.outer(drift, lateral)
    # os apoios de cada pe (o mais baixo do calcanhar e do dedo a 3 cm do chao) e onde o pe fica parado neles
    joints0 = data["posed_joints"].astype(float)
    locks = {}
    for s in ("Left", "Right"):
        low = np.minimum(joints0[:, IX[f"{s}ToeBase"], 1], joints0[:, IX[f"{s}Foot"], 1])
        on = low < np.percentile(low, 2) + 0.03
        weight = np.zeros(len(on))
        ankle_at = np.zeros(len(on))
        toe_at = np.zeros(len(on))
        k = 0
        while k < len(on):
            if not on[k]:
                k += 1
                continue
            j = k
            while j + 1 < len(on) and on[j + 1]:
                j += 1
            if j > k:
                mid = (k + j) // 2
                for i in range(max(0, k - 1), min(len(on), j + 2)):
                    w = 1.0 if k <= i <= j else 0.5
                    if w > weight[i]:
                        weight[i] = w
                        ankle_at[i] = float((joints0[mid, IX[f"{s}Foot"]] - center) @ forward)
                        toe_at[i] = float((joints0[mid, IX[f"{s}ToeBase"]] - center) @ forward)
            k = j + 1
        locks[s] = (weight, ankle_at, toe_at)
    new_local = np.zeros_like(local)
    new_pos = np.zeros_like(data["posed_joints"], dtype=float)
    new_world = np.zeros_like(local)
    check = turn_check = 0.0
    for t in range(len(local)):
        b = Body(parents, offsets, local[t], roots[t])
        check = max(check, float(np.abs(b.pos - data["posed_joints"][t]).max()))
        turn_check = max(turn_check, float(np.abs(b.world - data["global_rot_mats"][t]).max()))
        # o giro do quadril (pela raiz, em volta da vertical) e o dos ombros (pelo peito, em volta do tronco)
        # (girar em volta de +Y leva a esquerda para tras: o giro psi vira psi - angulo)
        b.turn("Hips", rotation(UP, (1 - keep) * yaw(b.pos[IX["LeftLeg"]] - b.pos[IX["RightLeg"]], lateral, forward)))
        target_yaw = keep * yaw(b.pos[IX["LeftArm"]] - b.pos[IX["RightArm"]], lateral, forward)
        for _ in range(3):
            axis = b.pos[IX["Neck1"]] - b.pos[IX["Chest"]]
            error = yaw(b.pos[IX["LeftArm"]] - b.pos[IX["RightArm"]], lateral, forward) - target_yaw
            b.turn("Chest", rotation(axis if axis @ UP > 0 else -axis, error / max(0.3, unit(axis) @ UP)))
        if straight_spine:
            line = unit(b.pos[IX["Neck1"]] - b.pos[IX["Hips"]])
            for joint, child in (("Spine1", "Spine2"), ("Spine2", "Chest"), ("Chest", "Neck1"), ("Neck1", "Head"),
                                 ("Head", "HeadEnd")):
                b.aim(joint, child, b.pos[IX[joint]] + line * np.linalg.norm(b.pos[IX[child]] - b.pos[IX[joint]]))
        # os bracos no plano do ombro: cada segmento (braco, antebraco, mao) com no maximo esta fracao de lado; o resto
        # da direcao (para a frente/tras e para baixo) fica (todo de lado: para baixo, junto ao corpo)
        for s in ("Left", "Right"):
            for joint, child, limit in ((f"{s}Arm", f"{s}ForeArm", lateral_share), (f"{s}ForeArm", f"{s}Hand", lateral_share),
                                        (f"{s}Hand", f"{s}HandMiddle1", 0.3)):
                v = b.pos[IX[child]] - b.pos[IX[joint]]
                d = unit(v)
                c = float(d @ lateral)
                if abs(c) <= limit:
                    continue
                rest = d - lateral * c
                rest = unit(rest) if np.linalg.norm(rest) > 1e-3 else -UP
                d_new = rest * np.sqrt(1 - limit * limit) + lateral * np.sign(c) * limit
                b.aim(joint, child, b.pos[IX[joint]] + d_new * np.linalg.norm(v))
        # as pernas numa linha de cada lado do caminho; o pe exato por IK
        for s, sign in (("Left", 1.0), ("Right", -1.0)):
            line = lambda p, w=width: p - lateral * (float((p - center) @ lateral) - sign * w)
            hip, knee, ankle = b.pos[IX[f"{s}Leg"]], b.pos[IX[f"{s}Shin"]], b.pos[IX[f"{s}Foot"]]
            toe, toe_end = b.pos[IX[f"{s}ToeBase"]], b.pos[IX[f"{s}ToeEnd"]]
            weight, ankle_at, toe_at = locks[s]
            pin = lambda p, at: p + forward * weight[t] * (at - float((p - center) @ forward))
            ankle_target = pin(line(ankle), ankle_at[t])
            # o polo do joelho: a frente do joelho do Kimodo, trazida para a linha
            pole = line(knee) + unit(knee - 0.5 * (hip + ankle) - lateral * float((knee - 0.5 * (hip + ankle)) @ lateral)) * 0.2
            knee_target, reach = two_bone(hip, knee, ankle, ankle_target, pole)
            toe_target = pin(line(toe), toe_at[t])
            toe_end_target = line(toe_end) + (toe_target - line(toe))
            b.aim(f"{s}Leg", f"{s}Shin", knee_target)
            b.aim(f"{s}Shin", f"{s}Foot", reach)
            b.aim(f"{s}Foot", f"{s}ToeBase", toe_target)
            b.aim(f"{s}ToeBase", f"{s}ToeEnd", toe_end_target)
        new_local[t], new_pos[t], new_world[t] = b.local, b.pos, b.world
    data["local_rot_mats"] = new_local.astype(data["local_rot_mats"].dtype)
    data["global_rot_mats"] = new_world.astype(data["global_rot_mats"].dtype)
    data["posed_joints"] = new_pos.astype(data["posed_joints"].dtype)
    np.savez(out, **data)
    print(f"{out.rsplit('/', 1)[-1]}: {len(local)} quadros; cinematica contra o Kimodo {check * 1000:.1f} mm,"
          f" rotacoes {turn_check:.4f}")


if __name__ == "__main__":
    main()
