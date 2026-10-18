#!/usr/bin/env python3
"""Exporta um checkpoint do rastreador para o .mrlpolicy que o motor le
(<treino>/exports/policy_latest.mrlpolicy; o treino ja exporta a cada vez que salva).

  .venv/bin/python rastreador_exportar.py --treino rast_g1 [--checkpoint ultimo.pt]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from rastreador_treino import DATA, Policy, export_policy, write_run_files  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", required=True)
    parser.add_argument("--checkpoint", default="ultimo.pt")
    args = parser.parse_args()
    run = DATA / args.treino
    state = torch.load(run / args.checkpoint, map_location="cpu", weights_only=False)
    settings = state["settings"]
    model = Policy(len(settings["observacao"]), len(settings["acao"]))
    model.load_state_dict(state["model"])
    write_run_files(run, settings.get("cenario", "frente"))
    out = run / "exports" / "policy_latest.mrlpolicy"
    export_policy(model, out, {"run": args.treino, "iteration": int(state["counters"]["rodada"]),
                               "steps": int(state["counters"]["decisoes"]), "observationNames": settings["observacao"]})
    (run / "latest.json").write_text(json.dumps({"iteration": int(state["counters"]["rodada"])}))
    print(f"politica: {out} (rodada {state['counters']['rodada']})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
