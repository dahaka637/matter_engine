"""A saida da suite: resultados.json (por medida e candidato: o valor, o IC95, a IQM, o pior mundo, os valores por mundo e por
cenario, o piso e a comparacao com o primeiro candidato) e tabela.md (os candidatos lado a lado, o delta contra o primeiro com a
marca contra a margem, os pisos e os cenarios que passavam e passaram a falhar, um a um).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from suite import cenarios as C
from suite import estatistica as E

# As linhas do resumo (o que decide primeiro): a postura, o parado, as paradas e o espelho, a velocidade e o rumo.
RESUMO = ("postura.coluna_frente.retos_trote", "postura.cabeca.retos_trote", "postura.coluna_frente.retos_corrida",
          "postura.cabeca.retos_corrida", "postura.pelve.retos_trote", "postura.coluna_lado.retos_frente",
          "postura.coluna_frente.parado", "postura.cabeca.parado", "postura.coluna_frente.paradas", "postura.cabeca.paradas",
          "parado.tolerancia.parado", "parado.erro_pose.parado", "parado.pes.parado", "parado.deslize.parado",
          "parado.deriva_pelve.parado", "parado.agitacao.parado", "parado.bracos_agitacao.parado", "parado.passos.parado",
          "parado.quedas.parado", "parado.minimo.parado", "parado.tolerancia.parado_empurrao", "parado.passos.parado_empurrao",
          "paradas.quedas", "paradas.tempo_parar", "paradas.passos_depois", "paradas.assentou", "paradas.base_fim",
          "firmeza.tronco_mole.paradas",
          "retos.razao.trote", "retos.razao.corrida", "retos.quedas.trote", "retos.quedas.corrida", "retos.peito.trote",
          "retos.peito.corrida", "espelho.razao", "espelho.desvio", "espelho.quedas")


def montar(medidas: dict, metas: dict, margens: dict) -> dict:
    """{candidato: [Medida]} -> o resultados.json. O primeiro candidato e a referencia das comparacoes."""
    nomes = list(medidas)
    por_id = {n: {m.id: m for m in medidas[n]} for n in nomes}
    out = {"suite": C.VERSAO, "candidatos": {}, "referencia": nomes[0] if nomes else None,
           "margens": "margens.json" if margens else "provisorias (sem A/A)"}
    for n in nomes:
        res = {}
        for mid, m in por_id[n].items():
            r = E.resumo(m)
            if n != nomes[0] and mid in por_id[nomes[0]]:
                r["contra_referencia"] = E.comparar(por_id[nomes[0]][mid], m, margens.get(mid, m.margem))
            res[mid] = r
        out["candidatos"][n] = {"meta": metas.get(n, {}), "medidas": res}
    return out


def _fmt(v, unidade: str) -> str:
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "—"
    if unidade in ("taxa", "fracao"):
        return f"{100 * v:.0f}%"
    a = abs(v)
    return f"{v:.3f}" if a < 1 else f"{v:.2f}" if a < 10 else f"{v:.1f}"


def _celula(r: dict) -> str:
    u = r["unidade"]
    v = r.get("valor_com_sinal", r["valor"]) if r["sentido"] == "zero" else r["valor"]
    lo, hi = r["ic95"]
    pre = "|·| " if r["sentido"] == "zero" else ""
    s = f"{_fmt(v, u)} ({pre}{_fmt(r['valor'], u)} [{_fmt(lo, u)}, {_fmt(hi, u)}])" if r["sentido"] == "zero" \
        else f"{_fmt(v, u)} [{_fmt(lo, u)}, {_fmt(hi, u)}]"
    if "passa_piso" in r and not r["passa_piso"]:
        s = f"**{s} ✗piso**"
    return s


def _delta(c: dict, u: str) -> str:
    if not c:
        return ""
    lo, hi = c["ic95"]
    return f"Δ {_fmt(c['delta_pior'], u)} [{_fmt(lo, u)}, {_fmt(hi, u)}] **{c['marca']}**"


def tabela(res: dict) -> str:
    nomes = list(res["candidatos"])
    if not nomes:
        return "sem candidatos\n"
    ref = nomes[0]
    L = [f"# Suite {res['suite']}", "",
         f"Referencia das comparacoes: **{ref}**. Margens: {res['margens']}. Δ no sentido 'pior' (positivo = piorou), IC95 "
         "pareado por bootstrap nos mundos; a marca confere o limite do intervalo contra a margem (estatistica.py). "
         "Medidas 'zero' (com sinal): o valor com sinal e, entre parenteses, a media dos modulos por cenario (o que decide). "
         "✗piso: o valor do grupo passou do piso provisorio (os cenarios acima dele, um a um, estao na secao dos pisos).", "", "## Candidatos", "",
         "| candidato | checkpoint | atualizacao | receita | conjunto | build | assets | setters ausentes no build | captura (s) |",
         "|---|---|---|---|---|---|---|---|---|"]
    for n in nomes:
        m = res["candidatos"][n]["meta"]
        L.append(f"| {n} | `{Path(m.get('checkpoint', '?')).name}` | {m.get('atualizacao', '?')} | `{Path(m.get('receita', '?')).name}` "
                 f"| {m.get('conjunto', '?')} | `{m.get('build', '?')}` | `{m.get('assets', '?')}` "
                 f"| {', '.join(m.get('setters_ausentes_no_build', [])) or '—'} | {m.get('segundos_captura', '?')} |")
    soltos = {n: res["candidatos"][n]["meta"].get("cenarios_nao_pareados") for n in nomes[1:]}
    L += ["", "Pareamento com a referencia (a velocidade, o rumo e o clipe pedidos a cada mundo no primeiro passo): "
          + "; ".join(f"{n}: " + ("todos pareados" if not v else "NAO pareados " + ", ".join(v)) for n, v in soltos.items())]
    ids = sorted({i for n in nomes for i in res["candidatos"][n]["medidas"]})

    def linhas(sel):
        out = ["| medida | cond | unidade | piso | " + " | ".join(nomes) + " |", "|---|---|---|---|" + "---|" * len(nomes)]
        for mid in sel:
            rs = [res["candidatos"][n]["medidas"].get(mid) for n in nomes]
            r0 = next(r for r in rs if r)
            base, cond = mid.rsplit(".", 1)
            cells = []
            for n, r in zip(nomes, rs):
                if not r:
                    cells.append("—")
                    continue
                c = _celula(r)
                if n != ref:
                    c += "<br>" + _delta(r.get("contra_referencia"), r["unidade"])
                cells.append(c)
            piso = "—" if r0["piso"] is None else ("≥ " if r0["sentido"] == "maior" else "≤ ") + _fmt(r0["piso"], r0["unidade"])
            out.append(f"| {r0['teste']} {base} | {cond} | {r0['unidade']} | {piso} | " + " | ".join(cells) + " |")
        return out

    resumo = [i for key in RESUMO for i in ids if i.rsplit(".", 1)[0] == key]
    L += ["", "## Resumo (postura, parado, paradas, espelho e velocidade)", ""] + linhas(resumo)
    L += ["", "## Pisos (no valor do grupo) e os cenarios acima do piso", ""]
    for n in nomes:
        ms = res["candidatos"][n]["medidas"]
        falhas = [mid for mid, r in ms.items() if "passa_piso" in r and not r["passa_piso"]]
        acima = [f"`{mid}`: {', '.join(r['cenarios_falham'])}" for mid, r in ms.items() if r.get("cenarios_falham")]
        L.append(f"- **{n}**: reprova {len(falhas)} pisos" + (f" ({', '.join(f'`{m}`' for m in falhas)})" if falhas else "")
                 + (f"; cenarios acima do piso: {'; '.join(acima)}" if acima else ""))
    if len(nomes) > 1:
        L += ["", f"## Cenarios que passavam no {ref} e passaram a falhar (pisos, um a um)", ""]
        base = res["candidatos"][ref]["medidas"]
        for n in nomes[1:]:
            novos = []
            for mid, r in res["candidatos"][n]["medidas"].items():
                if "cenarios_falham" in r and mid in base:
                    cs = [c for c in r["cenarios_falham"] if c not in base[mid].get("cenarios_falham", [])]
                    novos += [f"`{mid}`: {', '.join(cs)}"] if cs else []
            L.append(f"- **{n}**: " + ("nenhum" if not novos else "; ".join(novos)))
    L += ["", "## Todas as medidas", ""] + linhas(ids)
    return "\n".join(L) + "\n"


def escrever(res: dict, saida: Path) -> None:
    saida.mkdir(parents=True, exist_ok=True)
    (saida / "resultados.json").write_text(json.dumps(res, ensure_ascii=False, indent=1, default=float))
    (saida / "tabela.md").write_text(tabela(res))
