"""O vigia do treino: um laco leve (so le: o log, o fim do treino.out, o log do vigia termico e o servico) que imprime UMA
linha por alerta novo, para o Monitor: o treino inativo fora da guarda termica (ou parado por ela), o log sem linha nova
ha mais de 6 min, a CPU a 80 C ou mais (some abaixo de 76) ou o vigia termico sem leitura, erro no treino.out (Traceback,
NaN, double free...), NaN no log, a vazao 30% abaixo da mediana da ultima hora (some acima de -20%), 5 ou mais exercicios
subindo em 10 min, um exercicio sem descer ha mais de 90 min e um par espelhado com a direita muito pior (some so quando
ela nao e mais pior). Sem repetir: um alerta sai uma vez e so de novo depois de sumir e voltar. As contas sao as do painel
(inspecao.painel).

Uso:
  python -m inspecao.vigia_treino [--treino rl_final_clipe] [--intervalo 60]
  python -m inspecao.vigia_treino --uma-vez      (uma passada, imprime os alertas de agora e sai)
  python -m inspecao.vigia_treino --teste
"""
from __future__ import annotations

import argparse
import copy
import sys
import time
from pathlib import Path

from inspecao import comum, painel

JANELA = 3 * 3600.0  # (as linhas guardadas: a ultima hora da vazao e os 90 min sem descer cabem)
SEM_LINHA_MIN, CPU_QUENTE, CPU_VOLTA, PARADO_MIN, SUBINDO = 6.0, 80.0, 76.0, 90.0, 5
VAZAO_QUEDA, VAZAO_VOLTA = -0.30, -0.20


class Leitor:
    """As linhas do log (as das ultimas 3 h), relidas so quando ele muda: o fim novo quando cresce, tudo quando o treino
    o troca (a condensacao reescreve o arquivo - outro inode - a cada salvamento)."""

    def __init__(self, caminho: Path):
        self.caminho, self.marca, self.pos, self.linhas = caminho, None, 0, []

    def ler(self) -> list[dict]:
        try:
            st = self.caminho.stat()
        except OSError:
            self.marca, self.pos, self.linhas = None, 0, []
            return []
        if self.marca != (st.st_ino, st.st_size):
            if self.marca and self.marca[0] == st.st_ino and st.st_size > self.pos:
                novas, self.pos = painel.ler_log(self.caminho, self.pos)
                self.linhas += novas
            else:
                self.linhas, self.pos = painel.ler_log(self.caminho)
            self.marca = (st.st_ino, st.st_size)
            if self.linhas:
                self.linhas = [r for r in self.linhas if r["hora"] > self.linhas[-1]["hora"] - JANELA]
        return self.linhas


def alertas(an: dict | None, servico: bool, ativos: set) -> dict[str, str]:
    """Os alertas de agora, {chave: texto}; a CPU, a vazao e os espelhos com histerese (um alerta ativo so some alem do
    limiar de volta)."""
    if an is None:
        return {"log": "o log não tem linha de treino com a força por exercício"}
    out = {}
    s, ex = an["saude"], an["exercicios"]
    t = s["temperatura"]
    if not servico:
        out["servico"] = (f"treino parado pela guarda térmica (CPU {t['cpu']:.0f} °C)" if t.get("pausado")
                          else f"treino inativo ({comum.SERVICE}) fora da guarda térmica")
    elif s["ultima_linha_min"] > SEM_LINHA_MIN:
        out["linha"] = f"sem linha nova no log há {s['ultima_linha_min']:.0f} min (última r{an['atualizacao']})"
    if not t:
        out["termico"] = "sem o log do vigia térmico"
    else:
        if t["idade_min"] > 3:
            out["termico"] = f"o vigia térmico não lê a temperatura há {t['idade_min']:.0f} min"
        if t["cpu"] >= (CPU_VOLTA if "cpu" in ativos else CPU_QUENTE):
            out["cpu"] = f"CPU a {t['cpu']:.0f} °C (GPU {t['gpu']:.0f})"
    if s["erros"]:
        out["erro"] = "treino.out: " + s["erros"][-1]
    if s["nan"]:
        out["nan"] = f"NaN no núcleo do treino no log (r{s['nan'][-1]})"
    v = s["vazao"]["variacao"]
    if v is not None and v < (VAZAO_VOLTA if "vazao" in ativos else VAZAO_QUEDA):
        out["vazao"] = f"vazão {v:+.0%} contra a mediana da última hora ({s['vazao']['agora']:.0f} am/s)"
    subindo = [k for k in an["ranking"] if ex[k]["subidas_10min"]]
    if len(subindo) >= SUBINDO:
        out["subidas"] = f"{len(subindo)} exercícios subiram a força em 10 min: " + ", ".join(map(painel.nome, subindo))
    for k in an["ranking"]:
        if ex[k]["sem_descer_min"] > PARADO_MIN:
            out[f"parado:{k}"] = (f"{painel.nome(k)} sem descer há {'≥ ' if ex[k]['nunca_desceu'] else ''}"
                                  f"{ex[k]['sem_descer_min']:.0f} min (força {painel._pf(ex[k]['forca'])}; "
                                  f"{painel._segura(ex[k])})")
    for p in an["pares"]:
        # (Histerese: entra com a direita muito pior e so sai quando ela nao e mais pior - nem no veredito nem numa medida
        # no nivel 2: as medidas de uma decisao vem de 12-60 episodios e o veredito oscila; refeita a regra linha a linha
        # no log arquivado das 06:26 - 3 h -, o par 2/6 do trote saia 16 vezes sem ela e 4 com ela.)
        pior, k = p["veredito"] == "D pior", f"espelho:{p['esquerda']}"
        if (pior and p["nivel"] >= 2) or (k in ativos and (pior or p["nivel"] >= 2)):
            out[k] = (
                f"par {painel.nome(p['esquerda'])} × {painel.nome(p['direita'])}: a direita muito pior em "
                + ", ".join(painel._dif(m, v) for m, v in p["medidas"].items() if v["nivel"] > 0))
    return out


def novos(ativos: set, atuais: dict) -> list[str]:
    """As chaves que nao estavam ativas na passada anterior (um alerta que some e volta sai de novo)."""
    return [k for k in atuais if k not in ativos]


def passada(run: Path, leitor: Leitor, ativos: set) -> dict[str, str]:
    try:
        linhas = leitor.ler()
        res = painel.analisar(linhas, run, 1.0) if linhas else None
        return alertas(res[0] if res else None, comum.service_active(), ativos)
    except Exception as erro:  # (o vigia nao cai por uma linha estranha: vira um alerta)
        return {"vigia": f"o vigia falhou nesta passada: {type(erro).__name__}: {erro}"}


def _teste() -> None:
    # (1) Sem repetir: sai uma vez, some, volta e sai de novo.
    ativos, saidas = set(), []
    for atuais in ({"a": "x"}, {"a": "x"}, {}, {"a": "x", "b": "y"}):
        saidas.append(novos(ativos, atuais))
        ativos = set(atuais)
    assert saidas == [["a"], [], [], ["a", "b"]], saidas
    # (2) As regras sobre a analise de agora, alterada: a histerese da CPU e da vazao, o servico, as subidas e o parado.
    run = comum.run_dir("rl_final_clipe")
    linhas, _ = painel.ler_log(run / "log.jsonl")
    base, _ = painel.analisar(linhas, run, 1.0)
    an = copy.deepcopy(base)
    an["saude"]["temperatura"].update(cpu=81.0, gpu=60.0, idade_min=0.5, pausado=False)
    assert "cpu" in alertas(an, True, set())
    an["saude"]["temperatura"]["cpu"] = 78.0
    assert "cpu" in alertas(an, True, {"cpu"}) and "cpu" not in alertas(an, True, set())
    an["saude"]["temperatura"]["cpu"] = 75.0
    assert "cpu" not in alertas(an, True, {"cpu"})
    an["saude"]["vazao"].update(agora=3000.0, variacao=-0.35)
    assert "vazao" in alertas(an, True, set())
    an["saude"]["vazao"]["variacao"] = -0.25
    assert "vazao" in alertas(an, True, {"vazao"}) and "vazao" not in alertas(an, True, set())
    an["saude"]["temperatura"]["pausado"] = True
    assert "guarda térmica" in alertas(an, False, set())["servico"]
    an["saude"]["ultima_linha_min"] = 9.0
    assert "linha" in alertas(an, True, set()) and "linha" not in alertas(an, False, set())
    for k in an["ranking"][:5]:
        an["exercicios"][k]["subidas_10min"] = 1
    assert "subidas" in alertas(an, True, set())
    k = an["ranking"][0]
    an["exercicios"][k]["sem_descer_min"] = 120.0
    assert f"parado:{k}" in alertas(an, True, set())
    # (O espelho com histerese: entra com a direita muito pior; misto com uma medida no nivel 2 o mantem; iguais o tira.)
    p, k = an["pares"][0], f"espelho:{an['pares'][0]['esquerda']}"
    p.update(veredito="D pior", nivel=2)
    assert k in alertas(an, True, set())
    p.update(veredito="misto", nivel=2)
    assert k in alertas(an, True, {k}) and k not in alertas(an, True, set())
    p.update(veredito="iguais", nivel=0)
    assert k not in alertas(an, True, {k})
    # (3) Uma passada de verdade no treino de agora e num arquivado (parado ha horas: sem linha nova, ou o servico parado).
    agora = passada(run, Leitor(run / "log.jsonl"), set())
    print("treino de agora:", *[f"  {v}" for v in agora.values()] or ["  sem alertas"], sep="\n")
    assert all(isinstance(v, str) for v in agora.values()) and "vigia" not in agora, agora
    velho = comum.run_dir("rl_final_clipe/arquivo_antes_reset_reto_0626_0511")
    arquivado = passada(velho, Leitor(velho / "log.jsonl"), set())
    assert ("linha" in arquivado) != ("servico" in arquivado) and "vigia" not in arquivado, arquivado
    print("arquivado:", arquivado.get("linha") or arquivado.get("servico"))
    # (4) O leitor relê tudo quando o arquivo e trocado e so o fim quando cresce.
    tmp = comum.REPO / "build-profile/tmp/inspecao_dev/vigia_treino"
    tmp.mkdir(parents=True, exist_ok=True)
    origem = (run / "log.jsonl").read_bytes().splitlines(keepends=True)
    log = tmp / "log.jsonl"
    log.write_bytes(b"".join(origem[:-2]))
    leitor = Leitor(log)
    n = len(leitor.ler())
    with open(log, "ab") as f:
        f.write(origem[-2])
    assert len(leitor.ler()) == n + 1
    (tmp / "novo.jsonl").write_bytes(b"".join(origem[-4:-1]))  # (a ultima pode estar no meio da escrita)
    (tmp / "novo.jsonl").replace(log)
    assert len(leitor.ler()) == 3
    print("vigia_treino: teste ok")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--treino", default="rl_final_clipe")
    ap.add_argument("--intervalo", type=float, default=60.0, help="segundos entre as passadas")
    ap.add_argument("--uma-vez", action="store_true", help="uma passada e sai")
    ap.add_argument("--teste", action="store_true")
    a = ap.parse_args()
    if a.teste:
        _teste()
        return
    run = comum.run_dir(a.treino)
    leitor, ativos = Leitor(run / "log.jsonl"), set()
    print(f"vigia do treino {a.treino}: a cada {a.intervalo:g} s", file=sys.stderr)
    while True:
        atuais = passada(run, leitor, ativos)
        for k in novos(ativos, atuais):
            print(f"{time.strftime('%d/%m %H:%M')} {atuais[k]}", flush=True)
        ativos = set(atuais)
        if a.uma_vez:
            return
        time.sleep(a.intervalo)


if __name__ == "__main__":
    main()
