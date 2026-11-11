#!/usr/bin/env python3
"""Roda a prova de equivalencia de uma receita a partir de um treino (10/11; prova.py e a metodologia, secao 8).

Copia os arquivos do treino de origem para uma pasta de dados separada (MATTER_RL_DATA), roda o levantar_amp.py da arvore
de codigo dada com a receita e --prova por K atualizacoes, na CPU e sem salvar no meio, e deixa o hash em --saida.

  prova_rodar.py --receita receitas/principal_1011.json --de ~/.local/share/matter-engine/rl/runs/rl_final_clipe \
                 --nativo build-base-n1/python --saida /tmp/a.json [--atualizacoes 2] [--threads 1] [--passos 16]
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
PYTHON = Path("/home/dahaka/Projetos/matter_engine/tools/rl/.venv/bin/python")
COPY = ("ultimo.pt", "run.json", "log.jsonl", "tarefa.json")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--receita", required=True, help="o JSON da receita (tools/rl/receitas/*.json, chave 'argumentos')")
    p.add_argument("--de", required=True, help="a pasta do treino de origem (com o ultimo.pt)")
    p.add_argument("--nativo", required=True, help="a pasta python do build (o matter_tracking_native)")
    p.add_argument("--saida", required=True)
    p.add_argument("--codigo", default=str(HERE), help="a pasta tools/rl da arvore de codigo a provar")
    p.add_argument("--atualizacoes", type=int, default=2)
    p.add_argument("--threads", type=int, default=1)
    p.add_argument("--passos", type=int, default=16)
    p.add_argument("--rotulo", default="prova")
    p.add_argument("--extra", nargs=argparse.REMAINDER, default=[], help="argumentos a mais no fim (depois da receita)")
    a = p.parse_args()

    source = Path(a.de).expanduser()
    root = REPO / "build-profile" / "tmp" / "rl_prova" / a.rotulo
    run = root / "runs" / "prova"
    if root.exists():
        shutil.rmtree(root)
    run.mkdir(parents=True)
    for name in COPY:
        if (source / name).exists():
            shutil.copy2(source / name, run / name)
    for teto in source.glob("teto_*.json"):
        shutil.copy2(teto, run / teto.name)

    import torch  # noqa: E402 (so para ler o contador do ponto de partida)
    r0 = int(torch.load(run / "ultimo.pt", map_location="cpu", weights_only=False)["counters"]["atualizacao"])
    recipe = json.loads(Path(a.receita).read_text())["argumentos"]
    argv = [*recipe, "--treino", "prova", "--atualizacoes", str(r0 + a.atualizacoes), "--prova", str(Path(a.saida).resolve()),
            "--threads", str(a.threads), "--passos", str(a.passos), "--salvar-cada", "1000000", *a.extra]
    env = dict(os.environ, MATTER_RL_DATA=str(root), MATTER_TRACKING_NATIVE=str(Path(a.nativo).resolve()), PYTHONHASHSEED="0",
               MATTER_RL_SOCKET=str(root / "control.sock"), CUDA_VISIBLE_DEVICES="", TMPDIR=str(REPO / "build-profile" / "tmp"))
    for key in [k for k in env if k.startswith("T_") or k in ("PARADA_VARIANTE", "MATTER_MM_SEM_TROCA")]:
        env.pop(key)
    with open(run / "prova.out", "w") as out:
        rc = subprocess.run([str(PYTHON), "levantar_amp.py", *argv], cwd=a.codigo, env=env, stdout=out, stderr=subprocess.STDOUT).returncode
    print(f"prova {a.rotulo}: rc={rc}, de r{r0} ate r{r0 + a.atualizacoes}, saida {a.saida} (log {run / 'prova.out'})")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
