"""A captura do movimento: roda exercicios com a rede do treino (na forca dela, numa forca dada ou o clipe puro com a mao
cheia) e grava, a cada decisao e em cada mundo, o corpo (o centro e a orientacao do colisor de cada elo), a referencia do
clipe seguida naquele instante (posta sob a pelve, como a nota compara), os contatos, as medidas do rastreio, as acoes e os
episodios terminados. Saida por exercicio e modo: <saida>/<exercicio>__<modo>.npz, o .json com os metadados e a gravacao
do MatterModelInspector (<...>.dump, o mundo 0) para as folhas com o jogador de verdade.

Regra termica: recusa rodar com o treino ativo (use o orquestrador inspecao.inspecao, que para e religa).

Uso:
  python -m inspecao.captura --exercicios lev_trote_0 lev_trote_6 --modos atual 0.3 clipe --mundos 4 --segundos 8 --saida DIR
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from inspecao import comum


def capturar(amb: comum.Ambiente, key: str, modo: str, mundos: int, segundos: float, saida: Path, ruido: bool = False,
             semente: int = 61, log=print, troca: str | None = None,
            largada: tuple | None = None, rigidez: float | None = None) -> Path:
    """Uma captura: o exercicio `key` no `modo` ("atual" = a forca do exercicio no checkpoint; um numero = essa forca;
    "clipe" = o clipe puro, acao zero, mao cheia). Devolve o caminho do .npz."""
    torch = amb.torch
    # (06/11 09h25: "clipe0" - o clipe puro, acao zero, SEM a mao: os motores seguindo o clipe conseguem sozinhos?)
    zero = modo in ("clipe", "clipe0")
    level = 0.0 if modo == "clipe0" else 1.0 if zero else amb.level(key) if modo == "atual" else float(modo)
    b = amb.batch(key, mundos, semente)
    amb.setup(b, level)
    # (05/11 21h: o A/B das trocas - nada, freio (a freada ate o trote na corrida), pe (a meia-volta no pe certo) ou
    # freio_pe; None: a do treino, pelo run.json.)
    if troca is not None:
        b.set_run_turn_brake("freio" in troca)
        b.set_run_turn_align("pe" in troca)
        b.set_turn_settle("saida" in troca)
        b.set_turn_launch("arrancada" in troca)
    # (06/11: a largada progressiva da corrida - (escala, segundos, aceleracao); (0, 0, 0) a de antes; None: a do treino.)
    if largada is not None:
        b.set_run_launch_ease(*largada)
    # (06/11 09h40: a rigidez dos motores no levantar - o teste do limite estrutural.)
    if rigidez is not None:
        b.set_rise_stiffness(rigidez)
    b.reset(semente)
    model = amb.model(b)
    names = [lk["id"] for lk in comum.ragdoll()["links"]]
    profile_names = list(b.action_names())  # (so para conferir que o lote carregou)
    nl = len(names)
    dt = float(b.policy_dt)
    steps = int(round(segundos / dt))
    feedback_names = list(amb.L.TRACKING_TERMS)
    body = np.full((mundos, steps, nl, 7), np.nan, np.float32)
    ref = np.full((mundos, steps, nl, 7), np.nan, np.float32)
    feedback = np.full((mundos, steps, len(feedback_names)), np.nan, np.float32)
    actions = np.zeros((mundos, steps, b.action_size), np.float32)
    episode = np.zeros((mundos, steps), np.int16)
    info = np.full((mundos, steps, 5), np.nan, np.float32)  # t do clipe, forca, controle, pedido, rumo
    clips = [["" for _ in range(steps)] for _ in range(mundos)]
    contacts = []  # (mundo, passo, x, y, z, elo, nx, ny, nz, impulso)
    rows = []
    count = np.zeros(mundos, np.int16)
    gen = torch.Generator().manual_seed(semente)
    started = time.time()
    for i in range(steps):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            if zero:
                a = np.zeros((mundos, b.action_size), np.float32)
            else:
                dist = model.distribution(model.norm(obs))
                mean = dist.mean
                sample = mean + dist.stddev * torch.randn(mean.shape, generator=gen) if ruido else mean
                a = model.env_action(sample, obs).numpy().astype(np.float32)
        b.step(a)
        actions[:, i] = a
        fb = np.asarray(b.locomotion_feedback())
        feedback[:, i, :min(fb.shape[1], len(feedback_names))] = fb[:, :len(feedback_names)]
        for w in range(mundos):
            links, cts, _ = comum.parse_lines(b.frame_lines(w))
            for k, n in enumerate(names):
                if n in links:
                    body[w, i, k] = links[n]
            for c in cts:
                contacts.append([w, i] + c[:8])
            rlinks, _, q = comum.parse_lines(b.reference_lines(w)) if hasattr(b, "reference_lines") else ({}, [], {})
            for k, n in enumerate(names):
                if n in rlinks:
                    ref[w, i, k] = rlinks[n]
            if q:
                info[w, i] = [q["t"], q["forca"], q["controle"], q["pedido"], q["rumo"]]
                clips[w][i] = q["clipe"]
            episode[w, i] = count[w]
        for r in b.finished_episodes().tolist():
            rows.append([float(x) for x in r] + [float(i)])
            count[int(r[0])] += 1
    saida.mkdir(parents=True, exist_ok=True)
    tag = f"{key}__{modo}" + ("" if troca is None else f"__troca_{troca}") \
        + ("" if largada is None else "__largada_" + "_".join(f"{x:g}" for x in largada)) \
        + ("" if rigidez is None else f"__rigidez_{rigidez:g}")
    path = saida / f"{tag}.npz"
    np.savez_compressed(path, corpo=body, referencia=ref, medidas=feedback, acoes=actions, episodio=episode, info=info,
                        contatos=np.array(contacts, np.float32).reshape(-1, 10),
                        episodios=np.array(rows, np.float32).reshape(-1, len(comum.EPISODE_COLUMNS) + 1))
    meta = {"exercicio": key, "modo": modo, "troca": troca, "forca": level, "clipe_puro": zero, "ruido": ruido, "mundos": mundos, "dt": dt,
            "passos": steps, "treino": amb.treino, "checkpoint": str(amb.checkpoint), "atualizacao": amb.update,
            "elos": names, "medidas": feedback_names, "acoes": list(b.action_names()),
            "colunas_episodios": list(comum.EPISODE_COLUMNS) + ["passo_fim"], "clipes": clips,
            "info": ["t_clipe", "forca", "controle_locomocao", "pedido_ms", "rumo_rad"],
            "contatos": ["mundo", "passo", "x", "y", "z", "elo", "nx", "ny", "nz", "impulso"],
            "corpo": "centro do colisor xyz + quaternio do colisor xyzw (o eixo da capsula e o x local)",
            "referencia": "o quadro do clipe seguido, posto sob a pelve, no mesmo formato (nan sem clipe)",
            "ragdoll": comum.ragdoll(), "segundos_reais": round(time.time() - started, 2)}
    (saida / f"{tag}.json").write_text(json.dumps(meta, ensure_ascii=False))
    # (A gravacao do Inspector: o mundo 0, um quadro a cada 2 decisoes, como o laboratorio.)
    # (Como o frameLines: "B" so a caixa; a casca e a capsula, "C".)
    kinds = ["B" if lk["forma"] == "box" else "C" for lk in meta["ragdoll"]["links"]]
    with open(saida / f"{tag}.dump", "w") as dump:
        dump.write(f"S {key} {modo}\n")
        for i in range(0, steps, 2):
            if not np.isfinite(body[0, i, 0, 0]):
                continue
            dump.write(f"F {(i + 1) * dt:.4f} 0\n")
            for k, n in enumerate(names):
                dump.write(f"{kinds[k]} {n} " + " ".join(f"{v:.5f}" for v in body[0, i, k]) + "\n")
    log(f"{tag}: {steps} passos x {mundos} mundos, forca {level:.0%}, episodios terminados {len(rows)}, "
        f"{time.time() - started:.1f} s")
    return path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--treino", default="rl_final_clipe")
    ap.add_argument("--checkpoint", default=None)
    ap.add_argument("--exercicios", nargs="+", required=True)
    ap.add_argument("--modos", nargs="+", default=["atual", "clipe"], help="atual | clipe | uma forca (0-1)")
    ap.add_argument("--mundos", type=int, default=4)
    ap.add_argument("--segundos", type=float, default=8.0)
    ap.add_argument("--ruido", action="store_true", help="a acao com o ruido do treino (padrao: a media)")
    ap.add_argument("--saida", required=True)
    ap.add_argument("--semente", type=int, default=61, help="a semente dos comecos e do ruido (padrao 61)")
    ap.add_argument("--forcar", action="store_true", help="roda mesmo com o treino ativo (NAO: a regra termica)")
    ap.add_argument("--rigidez", type=float, default=None, help="a rigidez dos motores no levantar guiado (escala; o rastreio: 0,2)")
    ap.add_argument("--largada", type=float, nargs=3, default=None, metavar=("ESCALA", "SEGUNDOS", "ACELERACAO"),
                    help="a largada progressiva da corrida (0 0 0: a de antes; padrao: a do treino); o nome ganha __largada_...")
    ap.add_argument("--troca", choices=("nada", "freio", "pe", "freio_pe", "freio_pe_saida", "freio_pe_saida_arrancada"), default=None,
                    help="as trocas: a freada ate o trote na corrida e/ou a meia-volta no pe certo (padrao: a do treino); o "
                         "nome ganha __troca_<isto>")
    a = ap.parse_args()
    comum.require_stopped(a.forcar)
    amb = comum.Ambiente(a.treino, a.checkpoint)
    for key in a.exercicios:
        for modo in a.modos:
            capturar(amb, key, modo, a.mundos, a.segundos, Path(a.saida), a.ruido, a.semente,
                     troca=a.troca, largada=tuple(a.largada) if a.largada else None, rigidez=a.rigidez)


if __name__ == "__main__":
    main()
