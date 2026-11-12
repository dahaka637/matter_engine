"""O teto da semelhanca de cada exercicio (04/11 06h20): a semelhanca (a media do episodio, a da nota do rastreio) que o
proprio clipe consegue no corpo fisico - puro nos motores, sem a correcao da rede, com a forca auxiliar em 100% e a mao no
corpo todo. O portao da forca dinamica cobra a semelhanca da rede como fracao desse teto (95% a 100% de forca ... 70% a 0%).

Uso: MATTER_TRACKING_NATIVE=build-rl-n1/python tools/rl/.venv/bin/python tools/rl/teto_semelhanca.py --treino rl_final_clipe
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import levantar_amp as L  # noqa: E402

MANOBRAS = {name: k + 1 for k, name in enumerate(L.MANEUVERS)}
MANOBRAS.update({name: k + 5 for k, name in enumerate(L.RUN_CURVES)})
# (04/11 12h40: o ambiente como o do treino - a nota, o puxao de lado e os pedidos fixos do run.json; antes a nota 8 fixa.)
RUN_V2: dict = {}


def medir(key: str, mundos: int, segundos: float, idle_alcance: float) -> tuple[float, float, float]:
    common = dict(separate_skills=True, locomotion_tracking=True, locomotion_residual=True, locomotion_termination=True,
                  locomotion_seconds=segundos, moving_push_newtons=(0.0, 0.0), moving_push_hold_newtons=(0.0, 0.0))
    A = str(L.REPO / "assets")
    if key == "lev_parado":
        b = L.get_up_batch(A, environments=mundos, threads=4, seed=61, clip_start_fraction=0.0, standing_start_fraction=1.0,
                                   moving_fraction=0.0, avoid_fraction=0.0, locomotion_fraction=0.0, turn_fraction=0.0,
                                   episode_seconds=6.0, **common)
    elif key == "lev_giro":
        b = L.get_up_batch(A, environments=mundos, threads=4, seed=61, evaluation=True, evaluation_kind=6, **common)
    elif key in MANOBRAS:
        b = L.get_up_batch(A, environments=mundos, threads=4, seed=61, evaluation=True, evaluation_kind=4,
                                   evaluation_band=2 if key in L.RUN_CURVES else 1, **common)
        b.set_maneuvers(0.0, MANOBRAS[key])
    elif key == "lev_levantar":
        # (05/11: o levantar pelo clipe - as quedas da avaliacao.)
        b = L.get_up_batch(A, environments=mundos, threads=4, seed=61, evaluation=True, evaluation_kind=0, **common)
    elif key == "lev_sprint":
        # (05/11: o sprint e a faixa 3, so para a frente.)
        b = L.get_up_batch(A, environments=mundos, threads=4, seed=61, evaluation=True, evaluation_kind=4,
                                   evaluation_band=3, heading=0, **common)
    else:
        band = 2 if key.startswith("lev_corrida_") else 1
        b = L.get_up_batch(A, environments=mundos, threads=4, seed=61, evaluation=True, evaluation_kind=4,
                                   evaluation_band=band, heading=int(key[-1]), **common)
    b.set_tracking_reward(int(RUN_V2.get("recompensa_rastreio", 8)))
    L.run_tracking_settings(b, RUN_V2)
    b.set_idle_imitation(2.0)
    b.set_idle(True, True, idle_alcance)
    b.set_upper_hand(True)
    b.set_body_hand(True)
    b.set_drill_assist([1.0] * len(L.DRILL_KEYS))
    b.reset(61)
    rows = []
    zero = np.zeros((b.num_envs, b.action_size), np.float32)
    for _ in range(int((segundos + 0.5) / b.policy_dt) + 2):
        obs = np.asarray(b.observations())
        # (A recuperacao, nos primeiros instantes do parado, com a acao zero tambem: so o clipe conta.)
        b.step(zero)
        rows += b.finished_episodes().tolist()
        if len(rows) >= b.num_envs:
            break
    ep = np.array(rows[: b.num_envs])
    tracked = ep[ep[:, 33] >= 0, 33]
    # (04/11 10h40: e o teto da velocidade - a fracao do pedido que o proprio clipe faz no rumo, como no dominio; nas trocas
    # ele mesmo so faz 44-73%: a medida no rumo novo conta a frenagem e a virada.)
    moving = ep[ep[:, 10] > 0.05] if key not in ("lev_giro", "lev_parado") else ep[:0]
    speed = float(np.mean(moving[:, 12] / np.maximum(moving[:, 10], 0.1))) if len(moving) else 1.0
    # (04/11 13h50: e o teto da estabilidade, a media do episodio do clipe puro.)
    stable = ep[ep[:, 33] >= 0, 34]
    return (float(tracked.mean()) if len(tracked) else float("nan")), speed, (float(stable.mean()) if len(stable) else float("nan"))


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--treino", default="rl_final_clipe")
    p.add_argument("--mundos", type=int, default=8)
    p.add_argument("--segundos", type=float, default=8.0)
    p.add_argument("--idle-alcance", type=float, default=0.2)
    args = p.parse_args()
    RUN_V2.update(json.loads((L.DATA / args.treino / "run.json").read_text()).get("v2", {}))
    L.BATCH_EXTRA.update(L.batch_layout(RUN_V2))
    print("pedidos fixos:", RUN_V2.get("velocidades_trote"), RUN_V2.get("velocidades_corrida"), "nota",
          RUN_V2.get("recompensa_rastreio"), flush=True)
    out, speeds, stabilities = {}, {}, {}
    for key in L.DRILL_KEYS:
        similarity, speed, stability = medir(key, args.mundos, args.segundos, args.idle_alcance)
        out[key], speeds[key], stabilities[key] = round(similarity, 4), round(min(1.0, speed), 4), round(stability, 4)
        print(f"{key}: semelhanca {out[key]}, velocidade {speeds[key]}, estabilidade {stabilities[key]}", flush=True)
    path = L.DATA / args.treino / "teto_semelhanca.json"
    path.write_text(json.dumps(out, ensure_ascii=False, indent=1))
    (L.DATA / args.treino / "teto_velocidade.json").write_text(json.dumps(speeds, ensure_ascii=False, indent=1))
    (L.DATA / args.treino / "teto_estabilidade.json").write_text(json.dumps(stabilities, ensure_ascii=False, indent=1))
    print("gravado em", path, "e teto_velocidade.json")


if __name__ == "__main__":
    main()
