"""Muda o alcance da correcao da cabeca da locomocao num checkpoint sem mudar o movimento (04/11 22h50, a estagnacao: a
rede pedia mais que o alcance 22-34% do tempo nas pernas). A acao no motor e a media x o alcance: as linhas da ultima
camada dessas juntas vao divididas por novo/velho e o desvio da exploracao delas tambem - a correcao em radianos fica a
mesma, so o teto sobe.

  tools/rl/.venv/bin/python tools/rl/migrar_alcance.py ultimo.pt --de 0.10 0.12 0.08 0.05 --para 0.15 0.18 0.08 0.05
  (o resto/pernas e pelve, coluna, bracos, cabeca - como --residuo e --residuo-partes)
"""
import argparse
import math
import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import levantar_amp as L  # noqa: E402


def reach_of(names, rest, spine, arms, head):
    out = []
    for name in names:
        joint = name.split(".")[0]
        out.append(1.0 if joint == "passo" else spine if joint in ("Abdomen", "Chest", "UpperChest")
                   else arms if ("Arm" in joint or "Hand" in joint) else head if joint in ("Neck", "Head") else rest)
    return torch.tensor(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("checkpoint")
    ap.add_argument("--de", type=float, nargs=4, required=True)
    ap.add_argument("--para", type=float, nargs=4, required=True)
    ap.add_argument("--treino", default="rl_final_clipe")
    a = ap.parse_args()
    import json
    v2 = json.loads((L.DATA / a.treino / "run.json").read_text()).get("v2", {})
    L.BATCH_EXTRA.update(L.batch_layout(v2))
    b = L.get_up_batch(str(L.REPO / "assets"), environments=1, threads=1, seed=1)
    names = b.action_names()
    ratio = reach_of(names, *a.para) / reach_of(names, *a.de)
    path = Path(a.checkpoint)
    state = torch.load(path, map_location="cpu", weights_only=False)
    m = state["model"]
    keys = sorted(k for k in m if k.startswith("actor.locomotion.") and k.endswith(".weight"))
    last = keys[-1].rsplit(".", 1)[0]
    assert m[f"{last}.weight"].shape[0] == len(names), (m[f"{last}.weight"].shape, len(names))
    m[f"{last}.weight"] = m[f"{last}.weight"] / ratio[:, None]
    m[f"{last}.bias"] = m[f"{last}.bias"] / ratio
    m["loco_log_std"] = m["loco_log_std"] - torch.log(ratio)
    state.setdefault("settings", {})["alcance_migrado"] = {"de": a.de, "para": a.para}
    torch.save(state, path)
    changed = [(n, round(float(r), 3)) for n, r in zip(names, ratio) if abs(float(r) - 1.0) > 1e-6]
    print(f"{path}: {len(changed)} eixos com o alcance novo (camada {last}); ex.: {changed[:4]}")


if __name__ == "__main__":
    main()
