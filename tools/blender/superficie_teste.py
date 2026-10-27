#!/usr/bin/env python3
"""As provas do gerador do template de superficie (tools/blender/superficie.py), na troca instalada do
jogador (26/10/2026; docs/SISTEMA_DE_APARENCIA.md):

 - a referencia nova e deterministica: duas replanificacoes da mesma malha dao a mesma referencia;
 - a referencia e canonica: um corpo diferente com a mesma topologia (0,5% mais largo e um calombo
   liso de 2 mm) sai com o template IGUAL bit a bit (desenho, cartas, zonas, regioes, UV0, UV1, a
   referencia 3D, as ancoras e as costuras): a posicao de agora nao entra;
 - outra topologia para o gerador; uma face da roupa sem o molde tambem (nao e restaurada da referencia);
 - o desdobramento da borda da meia so aceita a fileira (um leque com vertice de dentro e um anel fechado
   param) e guarda os comprimentos; a injetividade e exata (um triangulo pequeno dentro de outro e pego,
   os vizinhos pela aresta nao, o degenerado e recusado);
 - as versoes pelo comando de verdade: --novo-layout grava o layout s2 e guarda o s1; uma referencia
   nova sem layout novo para sem gravar nada; as duas juntas gravam p2 e s2 e guardam p1 e s1; com
   outro layout a coordenada de desenho e a mesma e uma ancora cai no mesmo ponto 3D pelos dois UV0;
 - a ida e volta pelo Blender (troca -> importar_troca -> exportar_troca -> gerador) da o mesmo
   desenho em cada poligono (a outra ordem dos triangulos nao importa).

    python3 tools/blender/superficie_teste.py
"""
import copy
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import superficie  # noqa: E402

TROCA = os.path.join(superficie.ROOT, "assets", "characters", "player_model", "blender", "jogador.troca.json")
SURFACE_KEYS = ("surface", "trianglePanel", "triangleRegion", "triangleIdentity", "triangleChartMask", "triangleDesign",
                "triangleChart", "triangleKitUv", "triangleIdentityUv", "triangleReference")
checks = 0


def require(condition, message):
    global checks
    checks += 1
    if not condition:
        sys.exit("FALHOU: " + message)


def expect_failure(function, fragment, message):
    try:
        function()
    except SystemExit as stop:
        require(fragment in str(stop.code), f"{message}: a mensagem \"{stop.code}\" nao diz \"{fragment}\"")
        return
    require(False, message + ": nao parou")


def definitions():
    return json.load(open(superficie.files()[0]))


def referenced(data, defs, reference, layout):
    """O template aplicado (as saidas como a troca grava)."""
    template = superficie.Template(copy.deepcopy(data), defs, reference)
    template.apply_reference(reference)
    template.build_uv(layout)
    template.check_overlaps()
    return template


def outputs(template):
    """Tudo o que o template grava, comparavel."""
    out = {}
    for t in range(template.mesh.F):
        out[t] = (template.panel_of[t], template.region_of[t], template.identity[t],
                  None if t not in template.design else template.design[t].tobytes(),
                  None if t not in template.kit_uv else template.kit_uv[t].tobytes(),
                  None if t not in template.identity_uv else template.identity_uv[t].tobytes(),
                  None if t not in template.design else template.reference_positions[template.mesh.T[t]].tobytes(),
                  tuple((cid, c[t].tobytes()) for cid, c in sorted(template.chart_corners.items()) if t in c))
    return out, json.dumps([template.anchors, template.zones, template.seams], sort_keys=True)


def main():
    data = json.load(open(TROCA))
    for key in SURFACE_KEYS:
        data.pop(key, None)
    defs = definitions()
    _, reference_file, layout_file = superficie.files()
    reference = json.load(open(reference_file(defs["parametrization"])))
    layout = json.load(open(layout_file(defs["storage"])))

    # 1. A referencia nova e deterministica.
    def fresh():
        template = superficie.Template(copy.deepcopy(data), defs)
        template.stored_cuts = reference.get("cuts", {})
        template.build_design()
        template.build_charts()
        template.build_anchors_and_zones()
        template.classify_identity()
        return json.dumps(template.to_reference(99), sort_keys=True)
    require(fresh() == fresh(), "duas replanificacoes da mesma malha dao referencias diferentes")

    # 2. A referencia e canonica: o corpo muda, o template nao.
    base = referenced(data, defs, reference, layout)
    moved = copy.deepcopy(data)
    p = np.array(moved["positions"])
    bump = 0.002 * np.exp(-((p[:, 2] - 0.25) / 0.15) ** 2)
    radial = np.stack([p[:, 0], p[:, 1], np.zeros(len(p))], 1)
    radial /= np.maximum(np.linalg.norm(radial, axis=1), 1e-9)[:, None]
    moved["positions"] = (p * np.array([1.005, 1.005, 1.0]) + radial * bump[:, None]).tolist()
    other = referenced(moved, defs, reference, layout)
    a, b = outputs(base), outputs(other)
    for t in range(base.mesh.F):
        require(a[0][t] == b[0][t], f"o corpo diferente muda o template no triangulo {t}")
    require(a[1] == b[1], "o corpo diferente muda as ancoras, as zonas ou as costuras")

    # 3. Outra topologia para o gerador.
    # (Um triangulo de um quad vira um poligono sozinho: o quad deixa de existir.)
    broken = copy.deepcopy(data)
    counts = {}
    for pid in broken["trianglePolygon"]:
        counts[pid] = counts.get(pid, 0) + 1
    t = next(i for i, m in enumerate(broken["triangleMolde"]) if m and counts[broken["trianglePolygon"][i]] == 2)
    broken["trianglePolygon"][t] = max(broken["trianglePolygon"]) + 1
    expect_failure(lambda: superficie.Template(broken, defs, reference), "outra topologia", "outra topologia")

    # 3b. O molde de cada face da roupa e obrigatorio: sem ele, o gerador para (nao restaura da referencia).
    unlabeled = copy.deepcopy(data)
    t = next(i for i, m in enumerate(unlabeled["triangleMolde"]) if m)
    for i, pid in enumerate(unlabeled["trianglePolygon"]):
        if pid == unlabeled["trianglePolygon"][t]:
            unlabeled["triangleMolde"][i] = 0
    expect_failure(lambda: superficie.Template(unlabeled, defs, reference), "o molde do triangulo", "um molde da roupa removido")

    # 3c. O desdobramento so aceita a fileira: um leque com vertice de dentro e um anel fechado param.
    ring = np.array([[np.cos(a), np.sin(a), 0.0] for a in np.linspace(0, 2 * np.pi, 7)[:6]])
    fan = np.vstack([[0.0, 0.0, 0.1], ring])
    expect_failure(lambda: superficie.unfold_strip(fan, np.array([[0, 1 + k, 1 + (k + 1) % 6] for k in range(6)])), "vertice de dentro",
                   "um leque no desdobramento")
    tube = np.vstack([ring, ring + [0.0, 0.0, 0.3]])
    closed = np.array([f for k in range(6) for f in ([k, (k + 1) % 6, 6 + k], [(k + 1) % 6, 6 + (k + 1) % 6, 6 + k])])
    expect_failure(lambda: superficie.unfold_strip(tube, closed), "ciclo", "um anel fechado no desdobramento")
    strip = closed[:-2]
    U = superficie.unfold_strip(tube, strip)
    for a, b in {tuple(sorted((int(f[k]), int(f[(k + 1) % 3])))) for f in strip for k in range(3)}:
        require(abs(np.linalg.norm(U[a] - U[b]) - np.linalg.norm(tube[a] - tube[b])) < 1e-12, "a fileira desdobrada guarda os comprimentos")

    # 3d. A injetividade exata: um triangulo pequeno inteiro dentro de outro (a grade de antes o perdia),
    # vizinhos pela aresta sem conflito, e o degenerado recusado.
    require(superficie.overlaps({0: [[0, 0], [1, 0], [0, 1]], 1: [[0.1, 0.1], [0.1001, 0.1], [0.1, 0.1001]]})[0] == 1,
            "o triangulo pequeno dentro do grande")
    require(superficie.overlaps({0: [[0, 0], [1, 0], [0, 1]], 1: [[1, 0], [1, 1], [0, 1]]})[0] == 0, "os vizinhos pela aresta")
    require(superficie.overlaps({0: [[0, 0], [1, 0], [2, 0]]})[2] == [0], "o triangulo degenerado")

    # 4. As versoes pelo comando de verdade, numa copia das definicoes.
    folder = tempfile.mkdtemp(prefix="superficie_teste_")
    try:
        work = os.path.join(folder, "superficie")
        shutil.copytree(os.path.dirname(superficie.files()[0]), work)
        troca_in, troca_out = os.path.join(folder, "in.troca.json"), os.path.join(folder, "out.troca.json")
        json.dump(data, open(troca_in, "w"))
        env = dict(os.environ, MATTER_SUPERFICIE_DIR=work)
        run = lambda *flags: subprocess.run([sys.executable, os.path.join(HERE, "superficie.py"), troca_in, troca_out, *flags],
                                            env=env, capture_output=True, text=True)
        p0, s0 = defs["parametrization"], defs["storage"]
        done = run("--novo-layout")
        require(done.returncode == 0, "o --novo-layout falhou: " + done.stdout + done.stderr)
        after = json.load(open(os.path.join(work, "definicoes.json")))
        require(after["storage"] == s0 + 1 and after["parametrization"] == p0, "o --novo-layout nao subiu so a versao do layout")
        require(os.path.exists(os.path.join(work, f"layout_s{s0}.json")) and os.path.exists(os.path.join(work, f"layout_s{s0 + 1}.json")),
                "o --novo-layout nao guardou o layout anterior")
        second = json.load(open(troca_out))
        require(second["surface"]["storage"] == s0 + 1, "a troca nao diz o layout novo")
        before = sorted(os.listdir(work))
        refused = run("--nova-referencia")
        require(refused.returncode != 0 and "pede um layout novo" in refused.stdout + refused.stderr,
                "uma referencia nova sem layout novo nao parou")
        require(sorted(os.listdir(work)) == before and json.load(open(os.path.join(work, "definicoes.json"))) == after,
                "a recusa gravou alguma coisa")
        done = run("--nova-referencia", "--novo-layout")
        require(done.returncode == 0, "a referencia nova com layout novo falhou: " + done.stdout + done.stderr)
        final = json.load(open(os.path.join(work, "definicoes.json")))
        require(final["parametrization"] == p0 + 1 and final["storage"] == s0 + 2, "as versoes novas")
        require(all(os.path.exists(os.path.join(work, name)) for name in
                    (f"referencia_p{p0}.json", f"referencia_p{p0 + 1}.json", f"layout_s{s0}.json")), "as versoes anteriores ficam")
    finally:
        shutil.rmtree(folder)
    # Com outro layout: a mesma coordenada de desenho e uma ancora no mesmo ponto 3D pelos dois UV0.
    superficie.BASE_SIZE, size = 2048, superficie.BASE_SIZE
    try:
        relaid = base.new_layout(layout["version"] + 1, reference["parametrization"])
    finally:
        superficie.BASE_SIZE = size
    shifted = referenced(data, defs, reference, relaid)
    require(any(not np.allclose(base.kit_uv[t], shifted.kit_uv[t]) for t in base.kit_uv), "o segundo layout e igual ao primeiro")
    P = base.reference_positions
    for anchor in base.anchors:
        hit = None
        for t in base.tris_of[anchor["panel"]]:
            A, B, C = base.design[t]
            m = np.array([[B[0] - A[0], C[0] - A[0]], [B[1] - A[1], C[1] - A[1]]])
            u, v = np.linalg.solve(m, np.asarray(anchor["position"]) - A)
            if u >= -1e-9 and v >= -1e-9 and u + v <= 1 + 1e-9:
                hit = (t, np.array([1 - u - v, u, v]))
                break
        require(hit is not None, f"a ancora {anchor['id']} fora do painel")
        t, w = hit
        point = w @ P[base.mesh.T[t]]
        for template in (base, shifted):
            A, B, C = template.kit_uv[t]
            m = np.array([[B[0] - A[0], C[0] - A[0]], [B[1] - A[1], C[1] - A[1]]])
            u, v = np.linalg.solve(m, (w @ template.kit_uv[t]) - A)
            back = np.array([1 - u - v, u, v]) @ P[base.mesh.T[t]]
            require(np.linalg.norm(back - point) < 1e-9, f"a ancora {anchor['id']} muda de lugar pelo UV0")

    # 5. A ida e volta pelo Blender: o mesmo desenho em cada poligono.
    blender = shutil.which("blender")
    if blender is None:
        print("superficie_teste: sem o Blender, a ida e volta nao foi conferida")
    else:
        folder = tempfile.mkdtemp(prefix="superficie_volta_")
        try:
            # (Isolado das preferencias e dos complementos do usuario, sem audio, com tempo limite.)
            env = dict(os.environ, PATH="/usr/bin:/bin", PYTHONHOME="/usr", BLENDER_USER_CONFIG=folder, BLENDER_USER_SCRIPTS=folder)
            source, blend, back = (os.path.join(folder, n) for n in ("ida.troca.json", "volta.blend", "volta.troca.json"))
            json.dump(data, open(source, "w"))
            for name in data["textures"]:
                shutil.copy(os.path.join(os.path.dirname(TROCA), name), folder)
            subprocess.run([blender, "-b", "--factory-startup", "-noaudio", "--python", os.path.join(HERE, "importar_troca.py"), "--", source, blend],
                           env=env, capture_output=True, check=True, timeout=600)
            subprocess.run([blender, "-b", "--factory-startup", "-noaudio", blend, "--python", os.path.join(HERE, "exportar_troca.py"), "--", back],
                           env=env, capture_output=True, check=True, timeout=600)
            returned = json.load(open(back))
            again = referenced(returned, defs, reference, layout)
            per_vertex = lambda tpl: {(tpl.polygon_key[t], int(v)): tuple(np.round(tpl.design[t][k], 9))
                                      for t in tpl.design for k, v in enumerate(tpl.mesh.T[t])}
            require(per_vertex(again) == per_vertex(base), "a ida e volta pelo Blender muda o desenho de algum poligono")
            require(np.array_equal(np.array(returned["positions"]), np.array(data["positions"])), "a ida e volta muda as posicoes")
        finally:
            shutil.rmtree(folder)
    print(f"superficie_teste: ok ({checks} checagens)")


if __name__ == "__main__":
    main()
