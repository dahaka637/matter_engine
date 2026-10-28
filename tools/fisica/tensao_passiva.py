"""As paredes passivas das juntas (a tensao nos limites) no perfil do corpo, pela pesquisa de
biomecanica de 28/10 (vault 05_fisica/pesquisa_tensao_passiva_juntas, a secao "Valores por eixo";
os valores "sem" as unidades biarticulares, que ficam para depois).

Cada linha da tabela e um lado anatomico de um eixo: onde o tecido comeca a resistir (graus a
partir do neutro, positivo rumo aquele fim), o fim anatomico, a escala da exponencial (os graus em
que o momento se multiplica por e) e o momento no fim (N.m). O mapa de sinais leva o lado
anatomico ao sinal do eixo da junta - medido pela geometria do perfil (X frente, Y esquerda, Z cima):
nos eixos espelhados, esquerda e direita tem sinais opostos.

uso: tensao_passiva.py PERFIL [--saida OUTRO] [--ligar] [--neutro]
  --ligar   liga a tensao no perfil (limitTension.enabled)
  --neutro  as paredes de extensao do quadril e de hiperextensao do joelho comecam no neutro
            (na pesquisa elas ja empurram em pe, como no corpo real)
"""
import argparse
import json
import sys

# (junta, eixo, lado anatomico): (comeco, fim, escala, momento no fim)
LEGS = {
    ("Thigh", "swing1", "flexao"): (50.0, 125.0, 25.0, 15.0),
    ("Thigh", "swing1", "extensao"): (-24.5, 25.0, 16.5, 34.5),
    ("Thigh", "swing2", "abducao"): (32.0, 50.0, 6.0, 40.0),
    ("Thigh", "swing2", "aducao"): (12.0, 30.0, 6.0, 40.0),
    ("Thigh", "twist", "rot_interna"): (23.0, 38.0, 5.0, 40.0),
    ("Thigh", "twist", "rot_externa"): (27.0, 42.0, 5.0, 40.0),
    ("Shin", "twist", "flexao"): (73.0, 145.0, 24.0, 29.5),
    ("Shin", "twist", "hiperextensao"): (-16.0, 5.0, 7.0, 22.0),
    ("Foot", "twist", "dorsiflexao"): (2.0, 35.0, 11.0, 30.0),
    ("Foot", "twist", "flexao_plantar"): (41.5, 58.0, 5.5, 30.0),
    ("Foot", "swing1", "inversao"): (19.5, 30.0, 3.5, 40.0),
    ("Foot", "swing1", "eversao"): (9.5, 20.0, 3.5, 20.0),
}
SPINE = {  # o mesmo momento nas tres (em serie), comecando no neutro
    "Abdomen": {"flexao": (55, 21.5), "extensao": (20, 10.0), "inclinacao": (25, 6.8), "rotacao": (7, 2.9)},
    "Chest": {"flexao": (11, 4.3), "extensao": (6, 3.0), "inclinacao": (10, 2.7), "rotacao": (14, 5.7)},
    "UpperChest": {"flexao": (7, 2.7), "extensao": (4, 2.0), "inclinacao": (6, 1.6), "rotacao": (11, 4.5)},
}
SPINE_MOMENT = {"flexao": 100.0, "extensao": 20.0, "inclinacao": 50.0, "rotacao": 12.0}
NECK = {
    ("Head", "swing1", "flexao"): (0.0, 15.0, 11.6, 8.0),
    ("Head", "swing1", "extensao"): (17.7, 30.0, 4.1, 6.0),
    ("Head", "swing2", "inclinacao"): (0.0, 12.0, 4.0, 6.0),
    ("Head", "twist", "rotacao"): (30.0, 45.0, 5.0, 4.0),
    ("Neck", "swing1", "flexao"): (10.0, 75.0, 21.6, 8.0),
    ("Neck", "swing1", "extensao"): (0.0, 45.0, 28.5, 6.0),
    ("Neck", "swing2", "inclinacao"): (5.0, 35.0, 10.0, 6.0),
    ("Neck", "twist", "rotacao"): (2.0, 32.0, 10.0, 4.0),
}
ARMS = {
    ("UpperArm", "swing1", "elevacao"): (70.0, 90.0, 7.0, 10.0),
    ("UpperArm", "swing2", "horizontal_frente"): (105.0, 130.0, 8.0, 10.0),
    ("UpperArm", "swing2", "horizontal_tras"): (30.0, 45.0, 5.0, 10.0),
    ("UpperArm", "twist", "rot_externa"): (70.0, 90.0, 7.0, 5.0),
    ("UpperArm", "twist", "rot_interna"): (45.0, 62.0, 5.0, 5.0),
    ("Forearm", "twist", "flexao"): (120.0, 145.0, 8.0, 10.0),
    ("Forearm", "twist", "hiperextensao"): (-1.0, 5.0, 2.0, 10.0),
    ("Hand", "twist", "pro_supinacao"): (65.0, 85.0, 5.0, 2.0),
    ("Hand", "swing1", "flexao_extensao"): (75.0, 90.0, 5.0, 1.0),
}

# O sinal do eixo para cada lado anatomico; (junta sem lado) ou ("Left"/"Right", junta).
# Medido: Thigh swing1 + = extensao; Shin twist + = flexao; Foot twist + = flexao plantar;
# coluna/pescoco swing1 + = flexao; Thigh swing2 (eixo -X) + = aducao na esquerda, abducao na
# direita; Thigh twist (eixo +Z) + = rotacao externa na esquerda, interna na direita; Foot swing1
# (eixo -X) + = inversao na esquerda, eversao na direita; UpperArm swing1 + = abaixar (os dois);
# UpperArm swing2 (eixo +Z) + = para tras na esquerda, para a frente na direita; UpperArm twist
# (eixo ao longo do braco) + = rotacao interna na esquerda, externa na direita; Forearm + = flexao.
SIGNS = {
    ("Thigh", "flexao"): -1, ("Thigh", "extensao"): 1,
    ("LeftThigh", "aducao"): 1, ("LeftThigh", "abducao"): -1,
    ("RightThigh", "aducao"): -1, ("RightThigh", "abducao"): 1,
    ("LeftThigh", "rot_externa"): 1, ("LeftThigh", "rot_interna"): -1,
    ("RightThigh", "rot_externa"): -1, ("RightThigh", "rot_interna"): 1,
    ("Shin", "flexao"): 1, ("Shin", "hiperextensao"): -1,
    ("Foot", "flexao_plantar"): 1, ("Foot", "dorsiflexao"): -1,
    ("LeftFoot", "inversao"): 1, ("LeftFoot", "eversao"): -1,
    ("RightFoot", "inversao"): -1, ("RightFoot", "eversao"): 1,
    ("UpperArm", "elevacao"): -1,
    ("LeftUpperArm", "horizontal_tras"): 1, ("LeftUpperArm", "horizontal_frente"): -1,
    ("RightUpperArm", "horizontal_tras"): -1, ("RightUpperArm", "horizontal_frente"): 1,
    ("LeftUpperArm", "rot_interna"): 1, ("LeftUpperArm", "rot_externa"): -1,
    ("RightUpperArm", "rot_interna"): -1, ("RightUpperArm", "rot_externa"): 1,
    ("Forearm", "flexao"): 1, ("Forearm", "hiperextensao"): -1,
    ("Head", "flexao"): 1, ("Head", "extensao"): -1, ("Neck", "flexao"): 1, ("Neck", "extensao"): -1,
}


def sign(link: str, side_name: str) -> int:
    base = link.replace("Left", "").replace("Right", "")
    for key in ((link, side_name), (base, side_name)):
        if key in SIGNS:
            return SIGNS[key]
    raise KeyError(f"sem sinal para {link} {side_name}")


def walls(link: str, axis: str, neutral: bool) -> dict:
    """As paredes do eixo no sinal da junta: {"lower": {...}, "upper": {...}}."""
    base = link.replace("Left", "").replace("Right", "")
    rows = []  # (sinal, comeco, fim, escala, momento)
    if base in SPINE:
        plane = {"swing1": ("flexao", "extensao"), "swing2": ("inclinacao", "inclinacao"),
                 "twist": ("rotacao", "rotacao")}[axis]
        for name, s in zip(plane, (1, -1)):
            end, scale = SPINE[base][name]
            rows.append((s, 0.0, float(end), scale, SPINE_MOMENT[name]))
    else:
        table = {**LEGS, **NECK, **ARMS}
        for (joint, ax, name), (start, end, scale, moment) in table.items():
            if joint != base or ax != axis:
                continue
            if neutral and (base, name) in (("Thigh", "extensao"), ("Shin", "hiperextensao")):
                start = 0.0
            if name in ("inclinacao", "rotacao", "pro_supinacao", "flexao_extensao") or (base == "Head" and name == "inclinacao"):
                rows += [(1, start, end, scale, moment), (-1, start, end, scale, moment)]  # simetrico
            else:
                rows.append((sign(link, name), start, end, scale, moment))
    out = {}
    for s, start, end, scale, moment in rows:
        key = "upper" if s > 0 else "lower"
        if key in out:
            raise ValueError(f"{link} {axis}: duas paredes no lado {key}")
        out[key] = {"startDegrees": round(s * start, 3), "endDegrees": round(s * end, 3),
                    "scaleDegrees": scale, "endTorque": moment}
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("perfil"); ap.add_argument("--saida", default="")
    ap.add_argument("--ligar", action="store_true"); ap.add_argument("--neutro", action="store_true")
    a = ap.parse_args()
    profile = json.load(open(a.perfil, encoding="utf-8"))
    count = 0
    for link in profile["links"]:
        for axis in (link.get("joint") or {}).get("axes", []):
            w = walls(link["id"], axis["axis"], a.neutro)
            axis.pop("passive", None)
            if w:
                axis["passive"] = w
                count += len(w)
    if a.ligar:
        profile["limitTension"] = {"enabled": True}
    open(a.saida or a.perfil, "w", encoding="utf-8").write(json.dumps(profile, indent=1, ensure_ascii=False) + "\n")
    print(f"{count} paredes em {sum(1 for l in profile['links'] for x in (l.get('joint') or {}).get('axes', []))} eixos"
          f"{' (ligada)' if a.ligar else ''}{' (neutro)' if a.neutro else ''} -> {a.saida or a.perfil}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
