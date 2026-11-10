"""A folha de quadros do rastreio nas condicoes do treino (03/11): a rede de um treino da rl_final com --rastreio,
nascendo dentro do clipe (o RSI), com a acao media, um episodio por clipe; grava o dump para o MatterModelInspector
quadros (uma linha "S <nome>" por clipe). O laboratorio mostra a largada de pe; isto mostra o que o treino ve.

    folha_rastreio.py --treino rl_final_rastreio --saida <pasta> [--clipes jog jog_strafe_left ...] [--giro] [--segundos 4]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from levantar_amp import Policy  # noqa: E402
from rastreador_treino import DATA, REPO, native  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", default="rl_final_rastreio")
    parser.add_argument("--saida", required=True)
    parser.add_argument("--clipes", nargs="*", default=["jog", "jog_strafe_left", "jog_backward", "jog_strafe_right"])
    parser.add_argument("--giro", action="store_true", help="tambem o giro parado")
    parser.add_argument("--segundos", type=float, default=4.0)
    parser.add_argument("--semente", type=int, default=5)
    args = parser.parse_args()
    run = DATA / args.treino
    v2 = json.loads((run / "run.json").read_text())["v2"]
    state = torch.load(run / "ultimo.pt", map_location="cpu", weights_only=False)
    common = dict(environments=1, threads=1, seed=args.semente, clip_start_fraction=0.0, standing_start_fraction=0.0,
                  locomotion_clip_fraction=1.0, locomotion_seconds=float(v2.get("locomocao_segundos", 8.0)),
                  locomotion_termination=bool(v2.get("queda_encerra", True)), separate_skills=True,
                  tracking_termination=float(v2.get("rastreio_encerra", 0.0)),
                  locomotion_residual=float(v2.get("residuo", 0.0)) > 0,
                  clip_blend=float(v2.get("mistura_clipe", 1.0)), walk_fraction=0.0)
    jobs = [(name, dict(common, locomotion_fraction=1.0, only_clip=name)) for name in args.clipes]
    if args.giro:
        jobs.append(("giro", dict(common, turn_fraction=1.0)))
    out = Path(args.saida)
    out.mkdir(parents=True, exist_ok=True)
    dump = open(out / f"{args.treino}_r{state['counters']['atualizacao']}.dump", "w")
    for name, settings in jobs:
        batch = native.GetUpAmpBatch(str(REPO / "assets"), **settings)
        model = Policy(batch.obs_size, batch.action_size, batch.action_ranges(), split=True)
        model.load_state_dict(state["model"])
        model.residual = float(v2.get("residuo", 0.0))
        model.eval()
        dump.write(f"S {name}\n")
        decisions = int(args.segundos * 30)
        scores, ended = [], None
        with torch.no_grad():
            for k in range(decisions):
                obs = torch.as_tensor(batch.observations())
                action = model.env_action(model.actor(model.norm(obs)), obs).numpy().astype(np.float32)
                _, term, trunc = batch.step(action)
                scores.append(float(np.asarray(batch.locomotion_feedback())[0][0]))
                if term[0] or trunc[0]:
                    ended = (k + 1) / 30.0
                    break
                if k % 2 == 0:
                    dump.write(f"F {(k + 1) / 30.0:.4f} 0\n" + batch.frame_lines(0))
        print(f"{name}: {'acabou em %.2f s' % ended if ended else 'de pe o tempo todo'}, nota media {np.mean(scores):.3f}")
    dump.close()
    print(dump.name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
