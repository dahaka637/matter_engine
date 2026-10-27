#!/usr/bin/env python3
"""O avesso do motor zerado nos pesos (22/10/2026).

O laco com o proprio motor: o MatterModelInspector avesso (todas as animacoes
do jogo, a mao solta e o punho; a mesma regra do teste do modelo) diz quais
triangulos viram do avesso a vista. Cada grupo deles (os ligados por vertice,
com o espelho) recebe uma proposta: os pesos dos cantos e de um anel em volta
alisados com os vizinhos (ate 4 ossos), mais forte a cada tentativa; o avesso
roda de novo e a proposta so fica se o total cair. Ficam parados os vertices
das costuras com outra parte e a camisa nunca recebe as juntas da coxa; com
--braco-perto, o braco so pesa perto dele (o corte da etapa 07).

    python3 tools/blender/desvirar_avesso.py <entrada.troca.json> <saida.troca.json> --modelo <pasta do modelo de teste> [--passadas 3]
        [--corpos 180,80,170,100,180,100]

Com --corpos (o corpo variavel, 26/10), o avesso e a soma dos corpos (cm,kg aos pares; o
primeiro e o de referencia) e a proposta so fica se a soma cair sem o primeiro piorar: o
encorpado virava a camisa nos agachamentos do levantar.

A pasta do modelo de teste e uma copia de assets/characters/player_model (o
model.json e o blender/): o troca de cada tentativa vai para blender/jogador.troca.json dela.
"""
import argparse
import json
import re
import shutil
import subprocess
import sys

import numpy as np
from scipy.spatial import cKDTree

import proporcao_calcao
import proporcao_tronco

THIGH = ("HipLift", "HipHelper", "HipHelper25", "HipHelper75", "Thigh", "GluteHelper")
# As tentativas para cada grupo: (aneis em volta, rodadas de alisamento); com
# rodadas 0, o trecho todo com um peso so (a media dele: anda rigido, nao vira
# por dentro).
# Com rodadas negativas, o trecho passa a seguir o que fica logo acima dele (a
# media dos pesos dos vertices ate 4 cm acima): metade (-1) ou inteiro (-2); na
# barra esmagada entre a barriga e a coxa, a dobra sai da camisa.
TRIES = ((1, 1), (1, 3), (2, 3), (3, 6), (4, 10), (2, 0), (3, 0), (2, -1), (2, -2), (3, -2))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("troca")
    p.add_argument("saida")
    p.add_argument("--modelo", required=True)
    p.add_argument("--passadas", type=int, default=3)
    p.add_argument("--inspetor", default="build-profile/MatterModelInspector")
    p.add_argument("--braco-perto", default="", help="pecas em que o braco so pesa perto dele (proporcao_tronco.arm_allowance)")
    p.add_argument("--calcao-coxa", action="store_true", help="a parede de dentro da perna do calcao com o piso do peso da coxa")
    p.add_argument("--corpos", default="", help="cm,kg aos pares (o primeiro e a referencia que nao pode piorar)")
    a = p.parse_args()

    data = json.load(open(a.troca))
    rest = np.array(data["positions"])
    tris = np.array(data["triangles"])
    part_of = np.array(data["trianglePart"])
    names = [part["name"] for part in data["parts"]]
    jnames = [j["name"] for j in data["joints"]]
    n, joints = len(rest), len(jnames)
    weights = np.zeros((n, joints))
    for v, pairs in enumerate(data["weights"]):
        for j, w in pairs:
            weights[v, j] = w
    neighbors = [set() for _ in range(n)]
    for t in tris:
        for k in range(3):
            neighbors[t[k]].update((t[(k + 1) % 3], t[(k + 2) % 3]))
    neighbors = [np.array(sorted(s)) for s in neighbors]
    # Os vertices de mais de uma parte (as costuras) ficam parados.
    parts_of_vertex = [set() for _ in range(n)]
    for t, pp in zip(tris, part_of):
        for v in t:
            parts_of_vertex[v].add(pp)
    held = np.array([len(s) > 1 for s in parts_of_vertex])
    in_shirt = np.array([names.index("camisa") in s for s in parts_of_vertex])
    thigh = [i for i, nm in enumerate(jnames) if nm.endswith(THIGH)]

    def swap(name):
        return "Right" + name[4:] if name.startswith("Left") else "Left" + name[5:] if name.startswith("Right") else name

    partner = np.array([jnames.index(swap(nm)) if swap(nm) in jnames else i for i, nm in enumerate(jnames)])
    distance, mirror = cKDTree(rest).query(rest * np.array([1.0, -1.0, 1.0]))
    if np.max(distance) > 1e-5:
        sys.exit("a malha nao e simetrica")
    left = rest[:, 1] > 1e-6
    target = f"{a.modelo}/blender/jogador.troca.json"
    # O braco so pesa perto dele nas pecas do --braco-perto (o corte da etapa 07 e do assado).
    arm_joints = proporcao_tronco.arm_joints(jnames)
    arm_rows = np.zeros(n, bool)
    for nm in filter(None, a.braco_perto.split(",")):
        arm_rows |= np.array([names.index(nm) in s for s in parts_of_vertex])
    arm_rows = np.nonzero(arm_rows)[0]
    allowance = proporcao_tronco.arm_allowance(rest, {j["name"]: np.array(j["position"]) for j in data["joints"]})
    upper_chest = jnames.index("UpperChest")

    floor_rows = (np.array([], dtype=int), np.array([]), np.array([], dtype=int))
    front_rows = (np.array([], dtype=int), np.array([]), (0, 0))
    if a.calcao_coxa:
        floor_rows = proporcao_calcao.inner_leg_floor(weights, rest, tris, part_of, names, jnames, data["ground"])
        front_rows = proporcao_calcao.front_middle_floor(weights, rest, tris, part_of, names, jnames, data["ground"])

    def keep_rules(w):
        if len(arm_rows):
            proporcao_tronco.limit_arm(w, arm_rows, allowance, arm_joints, upper_chest)
        proporcao_calcao.apply_floor(w, *floor_rows)
        proporcao_calcao.apply_pair_floor(w, *front_rows)

    numbers = [x for x in a.corpos.split(",") if x]
    bodies = [f"{numbers[i]},{numbers[i + 1]}" for i in range(0, len(numbers), 2)] or [None]
    reference = [None]

    def avesso(w):
        data["weights"] = [[[int(j), float(w[v, j])] for j in np.nonzero(w[v] > 1e-4)[0]] for v in range(n)]
        json.dump(data, open(target, "w"))
        totals, bad = [], set()
        for body in bodies:
            out = subprocess.run([a.inspetor, "avesso", "--modelo", f"{a.modelo}/model.json"]
                                 + (["--corpo", body] if body else []), capture_output=True, text=True).stdout
            total = re.search(r"triangulos do avesso a vista: (\d+)", out)
            totals.append(int(total.group(1)) if total else 10 ** 6)
            bad |= {int(t) for t in re.findall(r"^\s+T(\d+)\s", out, re.M)}
        # (O primeiro corpo nao piora: a soma so vale com ele igual ou melhor.)
        if reference[0] is not None and totals[0] > reference[0]:
            return 10 ** 6, sorted(bad)
        reference[0] = totals[0] if reference[0] is None else reference[0]
        if len(bodies) > 1:
            print("    por corpo: " + ", ".join(f"{b} {t}" for b, t in zip(bodies, totals)), flush=True)
        return sum(totals), sorted(bad)

    def proposal(w, group, rings, rounds):
        region = set(group)
        for _ in range(rings):
            region |= {int(u) for v in list(region) for u in neighbors[v]}
        idx = np.array(sorted(v for v in region if not held[v]), dtype=int)
        if len(idx) == 0:
            return None
        out = w.copy()
        if rounds < 0:
            top = rest[idx, 2].max()
            near = cKDTree(rest[idx]).query(rest, distance_upper_bound=0.04)[0] < 0.04
            above = np.nonzero(near & (rest[:, 2] > top) & (rest[:, 2] < top + 0.04) & ~held)[0]
            if len(above) == 0:
                return None
            mean = out[above].mean(0)
            if in_shirt[idx].any():
                mean[thigh] = 0.0
            share = 0.5 if rounds == -1 else 1.0
            mixed = (1.0 - share) * out[idx] + share * mean
            for row in mixed:
                row[np.argsort(-row)[4:]] = 0.0
            out[idx] = mixed / np.maximum(1e-12, mixed.sum(1))[:, None]
            rounds = 0
            out[left] = out[mirror[left]][:, partner]
            keep_rules(out)
            return out
        if rounds == 0:
            mean = out[idx].mean(0)
            if in_shirt[idx].any():
                mean[thigh] = 0.0
            mean[np.argsort(-mean)[4:]] = 0.0
            out[idx] = mean / max(1e-12, mean.sum())
        for _ in range(rounds):
            mixed = np.array([0.5 * out[i] + 0.5 * out[neighbors[i]].mean(0) for i in idx])
            mixed[np.ix_(in_shirt[idx], thigh)] = 0.0
            for row in mixed:
                row[np.argsort(-row)[4:]] = 0.0
            out[idx] = mixed / np.maximum(1e-12, mixed.sum(1))[:, None]
        # O lado esquerdo e o espelho do direito (as juntas trocadas de lado).
        out[left] = out[mirror[left]][:, partner]
        keep_rules(out)
        return out

    if len(arm_rows):
        print(f"  braco perto: {proporcao_tronco.limit_arm(weights, arm_rows, allowance, arm_joints, upper_chest)} cortados na entrada", flush=True)
    if len(floor_rows[0]):
        print(f"  calcao coxa: {len(floor_rows[0])} vertices; {proporcao_calcao.apply_floor(weights, *floor_rows)} no piso na entrada", flush=True)
    keep_rules(weights)
    best, bad = avesso(weights)
    print(f"  avesso no comeco: {best}", flush=True)
    for pass_ in range(a.passadas):
        if not bad:
            break
        # Os grupos: os cantos dos triangulos ligados por vertice (com o espelho).
        corners = {t: set(tris[t]) | {int(mirror[v]) for v in tris[t]} for t in bad}
        groups = []
        for t in bad:
            merged = [g for g in groups if g & corners[t]]
            union = set(corners[t]).union(*merged) if merged else set(corners[t])
            groups = [g for g in groups if not (g & corners[t])] + [union]
        improved = False
        for group in groups:
            for rings, rounds in TRIES:
                trial = proposal(weights, group, rings, rounds)
                if trial is None:
                    break
                total, trial_bad = avesso(trial)
                if total < best:
                    weights, best, bad, improved = trial, total, trial_bad, True
                    print(f"  passada {pass_ + 1}: grupo de {len(group)} vertices, aneis {rings} x {rounds} -> avesso {best}", flush=True)
                    break
            if best == 0:
                break
        if not improved:
            break
    total, _ = avesso(weights)
    shutil.copyfile(target, a.saida)
    print(f"AVESSO {total}; gravado {a.saida}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
