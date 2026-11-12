"""O atlas das quedas (05/11 21h, as trocas de direcao): em capturas com muitos mundos (inspecao.captura --ruido --mundos 64),
cada troca - o rumo pedido saltando ou a meia-volta entrando - e se o corpo caiu antes da proxima (ate --janela s depois),
por clipe de antes -> clipe da troca; e cada queda com o que veio antes (a troca, quanto tempo antes, o clipe na queda, a
pelve, a cabeca, a velocidade). So le os .npz: nao pesa, mas a regra termica dele vale - rodar com o treino parado.

  python -m inspecao.atlas PASTA/lev_troca_forte_corrida__atual.npz [...] [--quedas] [--janela 2.5]
  python -m inspecao.atlas --teste
"""
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def carregar(npz: Path) -> dict:
    z = np.load(npz)
    m = json.loads(Path(npz).with_suffix(".json").read_text())
    return dict(meta=m, dt=m["dt"], info=z["info"], epi=z["episodio"], rows=z["episodios"], clips=m["clipes"],
                corpo=z["corpo"], cols=m["colunas_episodios"])


def eventos(c: dict, w: int) -> tuple[list, list]:
    """As trocas do mundo w (passo, salto do rumo em rad, clipe novo, clipe de antes) e os passos das quedas. O passo do fim de
    um episodio ja mostra o corpo recomecado (o lote recomeca sozinho), com o indice do episodio velho: fica de fora."""
    rows = c["rows"][c["rows"][:, 0] == w]
    resets = {int(r[-1]) for r in rows}
    falls = [int(r[-1]) for r in rows if int(r[c["cols"].index("fim_causa")]) != 0]
    epi, info, clips = c["epi"], c["info"], c["clips"]
    ev = []
    for i in range(1, epi.shape[1]):
        if epi[w, i] != epi[w, i - 1] or i in resets or (i - 1) in resets:
            continue
        d = float(wrap(info[w, i, 4] - info[w, i - 1, 4]))
        turn_in = clips[w][i].startswith("running_turn") and not clips[w][i - 1].startswith("running_turn")
        if abs(d) > 0.2 or turn_in:
            ev.append((i, d, clips[w][i], clips[w][i - 1]))
    return ev, falls


def taxa_por_evento(c: dict, janela: float = 2.5) -> dict:
    """{(clipe de antes, "giro" ou o clipe novo): [trocas, quedas]} - a queda conta se vier antes da proxima troca e ate
    `janela` s depois; a troca perto do fim da captura sem tempo de janela fica de fora."""
    T = c["epi"].shape[1]
    stats = defaultdict(lambda: [0, 0])
    for w in range(c["epi"].shape[0]):
        ev, falls = eventos(c, w)
        for k, (i, _d, clip, before) in enumerate(ev):
            end = min(ev[k + 1][0] if k + 1 < len(ev) else T, i + int(janela / c["dt"]))
            fell = any(i < f <= end for f in falls)
            if end >= T and not fell:
                continue
            key = (before, "giro" if clip.startswith("running_turn") else clip)
            stats[key][0] += 1
            stats[key][1] += fell
    return dict(stats)


def quedas(c: dict) -> list[dict]:
    """Cada queda com a ultima troca do episodio antes dela."""
    out, dt, corpo = [], c["dt"], c["corpo"]
    for w in range(c["epi"].shape[0]):
        ev, falls = eventos(c, w)
        for f in falls:
            i0 = max(f - 1, 0)
            ult = [e for e in ev if e[0] < f and c["epi"][w, e[0]] == c["epi"][w, i0]]
            ult = ult[-1] if ult else None
            v = np.linalg.norm(corpo[w, i0, 0, :2] - corpo[w, max(i0 - 3, 0), 0, :2]) / (3 * dt)
            out.append(dict(mundo=w, t=round(f * dt, 2), desde_troca=None if not ult else round((f - ult[0]) * dt, 2),
                            graus=None if not ult else round(float(np.degrees(ult[1]))), antes=None if not ult else ult[3],
                            troca=None if not ult else ult[2], clipe_queda=c["clips"][w][i0],
                            pelve=round(float(corpo[w, i0, 0, 2]), 2), cabeca=round(float(corpo[w, i0, 5, 2]), 2),
                            vel=round(float(v), 2)))
    return out


def resumo(npz: Path, janela: float, listar: bool) -> None:
    c = carregar(npz)
    m, ep = c["meta"], c["rows"]
    n_quedas = int((ep[:, c["cols"].index("fim_causa")] != 0).sum())
    st = taxa_por_evento(c, janela)
    tot = [sum(v[0] for v in st.values()), sum(v[1] for v in st.values())]
    print(f"\n== {npz.stem}: forca {m['forca']:.0%}, {m['mundos']} mundos x {m['passos'] * m['dt']:.0f} s, {len(ep)} episodios "
          f"acabados, {n_quedas} quedas; trocas {tot[0]}, quedas depois de troca {tot[1]} ({tot[1] / max(tot[0], 1):.1%})")
    for (a, b), (n, k) in sorted(st.items(), key=lambda kv: -kv[1][0]):
        print(f"   {a:>26} -> {b:<22} trocas {n:4d}  quedas {k:3d} ({k / max(n, 1):.0%})")
    if listar:
        for q in quedas(c):
            print("   queda", q)


def teste() -> None:
    assert abs(float(wrap(3 * np.pi)) - np.pi) < 1e-6 or abs(float(wrap(3 * np.pi)) + np.pi) < 1e-6
    W, T = 1, 40
    info = np.zeros((W, T, 5), np.float32)
    info[0, 10:, 4] = np.pi / 2  # a troca no passo 10
    clips = [["jog"] * 10 + ["running_turn_180"] * 10 + ["jog"] * 20]
    rows = np.zeros((1, 6), np.float32)
    rows[0, 0], rows[0, 4], rows[0, -1] = 0, 1.0, 20  # mundo 0, queda (fim_causa=1) no passo 20
    c = dict(dt=1 / 30, info=info, epi=np.zeros((W, T), np.int16), rows=rows, clips=clips, corpo=np.zeros((W, T, 18, 7)),
             cols=["mundo", "a", "b", "c", "fim_causa"], meta={})
    ev, falls = eventos(c, 0)
    assert [e[0] for e in ev] == [10] and falls == [20], (ev, falls)
    assert taxa_por_evento(c) == {("jog", "giro"): [1, 1]}
    q = quedas(c)
    assert len(q) == 1 and q[0]["desde_troca"] == round(10 / 30, 2) and q[0]["graus"] == 90
    print("atlas ok")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capturas", nargs="*", type=Path)
    ap.add_argument("--janela", type=float, default=2.5)
    ap.add_argument("--quedas", action="store_true", help="lista cada queda")
    ap.add_argument("--teste", action="store_true")
    a = ap.parse_args()
    if a.teste:
        teste()
        return
    for p in a.capturas:
        resumo(p, a.janela, a.quedas)


if __name__ == "__main__":
    main()
