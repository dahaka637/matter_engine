#!/usr/bin/env python3
"""As folhas de quadros do rastreador: o corpo fisico e o lider (a animacao do
modo padrao que ele segue), lado a lado, de um checkpoint (ou sem rede, --zero).

  MATTER_TRACKING_NATIVE=../../build-prova/python .venv/bin/python rastreador_previa.py \
      --treino rast_g1 --velocidade 3 --segundos 8 --saida /tmp/rast
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from matter_rl import paths  # noqa: E402
from rastreador_treino import DATA, REPO, Policy, native  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", default="")
    parser.add_argument("--checkpoint", default="ultimo.pt")
    parser.add_argument("--zero", action="store_true")
    parser.add_argument("--velocidade", type=float, default=3.0)
    parser.add_argument("--segundos", type=float, default=8.0)
    parser.add_argument("--quadros", type=int, default=16)
    parser.add_argument("--vista", default="direita")
    parser.add_argument("--saida", required=True)
    parser.add_argument("--rumo", type=int, default=0, help="0..7 (de 45 em 45 graus a partir da frente, para a esquerda)")
    args = parser.parse_args()
    out = Path(args.saida)
    out.mkdir(parents=True, exist_ok=True)
    batch = native.TrackingBatch(str(REPO / "assets"), environments=8, threads=2, seed=20261018, scenario="frente",
                                 evaluation=True, evaluation_speed=args.velocidade,
                                 evaluation_mode="rumos" if args.rumo else "frente")
    w = args.rumo
    model = None
    who = "sem rede"
    if not args.zero:
        state = torch.load(DATA / args.treino / args.checkpoint, map_location="cpu", weights_only=False)
        model = Policy(batch.obs_size, batch.action_size)
        model.load_state_dict(state["model"])
        model.eval()
        who = f"{args.treino} rodada {state['counters']['rodada']}"
    dumps = {"corpo": ["S corpo"], "lider": ["S lider"]}
    ended = None
    steps = int(args.segundos / batch.policy_dt)
    for step in range(steps):
        obs = torch.as_tensor(batch.observations())
        with torch.no_grad():
            action = np.zeros((8, batch.action_size), np.float32) if model is None \
                else model.actor(model.norm(obs)).numpy().astype(np.float32)
        batch.step(action)
        for row in batch.finished_episodes():
            if int(row[0]) == w and ended is None:
                ended = f"{'caiu' if row[2] else 'tempo'} aos {row[1]:.1f} s"
        if ended is not None:
            break
        if step % 2 == 0:
            t = (step + 1) * batch.policy_dt
            dumps["corpo"] += [f"F {t:.4f} 0", batch.frame_lines(w, False).rstrip()]
            dumps["lider"] += [f"F {t:.4f} 0", batch.frame_lines(w, True).rstrip()]
    inspector = paths.inspector()
    for name, lines in dumps.items():
        (out / f"{name}.dump").write_text("\n".join(lines) + "\n")
        if inspector is not None:
            subprocess.run([str(inspector), "quadros", "--dump", str(out / f"{name}.dump"), "--quadros",
                            str(args.quadros), "--colunas", "8", "--largura", "180", "--altura", "260", "--vista",
                            args.vista, "--saida", str(out / f"{name}.png")], check=False, capture_output=True)
            print(f"folha: {out / f'{name}.png'}")
    print(f"{who} · {args.velocidade} m/s · {ended or f'de pe ate {args.segundos:.0f} s'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
