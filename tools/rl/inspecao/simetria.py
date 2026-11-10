"""A assimetria da propria rede (05/11): nos estados que ela mesma visita (os exercicios retos, o clipe periodico nos
motores), a acao no estado espelhado contra o espelho da acao - |mu(s) - E_a mu(E_s s)| na acao normalizada. Mede o vies
na fonte, sem a mao e sem a fisica no meio: a parte antissimetrica media por junta diz para que lado a rede puxa.

Uso (com o treino PARADO - roda o simulador):
  python -m inspecao.simetria [--exercicios ...] [--modos atual 0.3] [--mundos 2] [--segundos 6] [--saida DIR]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from inspecao import comum

sys.path.insert(0, str(comum.RL))
from espelho import Espelho  # noqa: E402

# (Os clipes periodicos e o deslocamento da fase no espelho: meio ciclo nos que servem aos dois lados, zero nos laterais.)
SHIFT = {"jog": math.pi, "jog_backward": math.pi, "run_forward_medium": math.pi, "run_backward_trote": math.pi,
         "sprint_maximo": math.pi, "jog_strafe_left": 0.0, "jog_strafe_right": 0.0, "sprint_strafe_left": 0.0,
         "sprint_strafe_right": 0.0}
DEFAULT = ["lev_trote_0", "lev_trote_2", "lev_trote_6", "lev_trote_1", "lev_trote_7", "lev_corrida_0", "lev_corrida_2",
           "lev_corrida_6", "lev_corrida_1", "lev_corrida_7", "lev_corrida_4", "lev_sprint"]
GROUPS = {"tronco": ("Abdomen", "Chest", "UpperChest"), "cabeca": ("Neck", "Head"),
          "bracos": ("UpperArm", "Forearm", "Hand"), "pernas": ("Thigh", "Shin", "Foot")}


def medir(amb: comum.Ambiente, key: str, modo: str, mundos: int, segundos: float, semente: int = 61) -> dict:
    torch = amb.torch
    level = amb.level(key) if modo == "atual" else float(modo)
    b = amb.batch(key, mundos, semente)
    amb.setup(b, level)
    b.reset(semente)
    model = amb.model(b)
    esp = Espelho(b.observation_names(), b.action_names())
    steps = int(round(segundos / float(b.policy_dt)))
    rows, shifts = [], []
    for _ in range(steps):
        obs = np.asarray(b.observations(), np.float32)
        with torch.no_grad():
            t = torch.as_tensor(obs)
            a = model.env_action(model.distribution(model.norm(t)).mean, t).numpy().astype(np.float32)
        for w in range(mundos):
            _, _, q = comum.parse_lines(b.reference_lines(w))
            if q and q["clipe"] in SHIFT and obs[w, -1] > 0.5 and q["controle"] == 1:
                rows.append(obs[w])
                shifts.append(SHIFT[q["clipe"]])
        b.step(a)
    if not rows:
        return {"exercicio": key, "modo": modo, "forca": level, "linhas": 0}
    x = torch.as_tensor(np.stack(rows))
    s = torch.as_tensor(np.array(shifts, np.float32))
    with torch.no_grad():
        mu = model.distribution(model.norm(x)).mean
        mm = esp.acao(model.distribution(model.norm(esp.obs(x, s))).mean)
    d = (mu - mm).numpy()
    names = esp.act_names
    joints = [i for i, n in enumerate(names) if not n.startswith("passo.")]
    out = {"exercicio": key, "modo": modo, "forca": round(level, 4), "linhas": int(len(rows)),
           "assimetria": round(float(np.abs(d[:, joints]).mean()), 4),
           # (A metade da diferenca media: o que a rede poe a mais num lado - o espelho da antissimetrica.)
           "vies_por_eixo": {names[i]: round(float(0.5 * d[:, i].mean()), 4) for i in joints},
           "assimetria_por_grupo": {g: round(float(np.abs(d[:, [i for i in joints if names[i].split(".")[0].replace(
               "Left", "").replace("Right", "") in parts]]).mean()), 4) for g, parts in GROUPS.items()}}
    top = sorted(out["vies_por_eixo"].items(), key=lambda kv: -abs(kv[1]))[:6]
    out["maiores"] = top
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--treino", default="rl_final_clipe")
    ap.add_argument("--checkpoint", default=None)
    ap.add_argument("--exercicios", nargs="+", default=DEFAULT)
    ap.add_argument("--modos", nargs="+", default=["atual", "0.3"])
    ap.add_argument("--mundos", type=int, default=2)
    ap.add_argument("--segundos", type=float, default=6.0)
    ap.add_argument("--saida", default=None)
    ap.add_argument("--forcar", action="store_true")
    a = ap.parse_args()
    comum.require_stopped(a.forcar)
    amb = comum.Ambiente(a.treino, a.checkpoint)
    res = []
    for key in a.exercicios:
        for modo in a.modos:
            r = medir(amb, key, modo, a.mundos, a.segundos)
            res.append(r)
            if r["linhas"]:
                print(f"{key:16s} {modo:5s} forca {r['forca']:.0%} linhas {r['linhas']:4d} assimetria {r['assimetria']:.3f} "
                      f"grupos {r['assimetria_por_grupo']} maiores " +
                      ", ".join(f"{n} {v:+.3f}" for n, v in r["maiores"]), flush=True)
            else:
                print(f"{key:16s} {modo:5s} sem linhas no clipe periodico", flush=True)
    if a.saida:
        Path(a.saida).mkdir(parents=True, exist_ok=True)
        (Path(a.saida) / "simetria.json").write_text(json.dumps({"atualizacao": amb.update, "medidas": res},
                                                               ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
