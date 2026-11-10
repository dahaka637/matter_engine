"""A professora B21 dentro da cena da rl_final (03/11 18h27, a ordem dele: gravar os movimentos da B21 nos 8 rumos e
FORCAR a rl_final a fazer exatamente os mesmos). Por rumo, episodios da avaliacao (de pe, no rumo, sem empurrao):

- modo b21: ela no volante (beta 1) - a regua: quedas, velocidade no rumo, desvio do rumo, peito, ajuda de fora;
- modo oraculo1 / oraculo2: a aluna-oraculo - o rotulo da professora em sombra nos motores da aluna (no tick 0 de cada
  decisao e segurado, como a rede a 30 Hz; ou a cada tick). Se ela nao anda como a professora, nenhuma rede treinada anda.

Uso: MATTER_TRACKING_NATIVE=build-rl-n1/python tools/rl/.venv/bin/python tools/rl/professor_b21.py --modos b21 oraculo1
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
RUNS = Path.home() / ".local/share/matter-engine/rl/runs"
NOMES = ["frente", "diag_frente_esq", "esquerda", "diag_costas_esq", "costas", "diag_costas_dir", "direita",
         "diag_frente_dir"]


def native():
    sys.path.insert(0, str(REPO / os.environ.get("MATTER_TRACKING_NATIVE", "build-rl-n1/python")))
    return importlib.import_module("matter_tracking_native")


EXERCICIOS = {"rumos": 4, "parado": 5, "giro": 6}


def medir(mod, modo: str, rumo: int, mundos: int, segundos: float, professor: Path, pedidos, aluna, threads: int,
          dumps: Path | None, exercicio: str = "rumos") -> dict:
    batch = mod.GetUpAmpBatch(str(REPO / "assets"), environments=mundos, threads=threads, seed=444 + rumo,
                              evaluation=True, evaluation_kind=EXERCICIOS[exercicio], heading=rumo, locomotion_seconds=segundos,
                              episode_seconds=segundos, locomotion_termination=True, separate_skills=True,
                              moving_push_newtons=(0.0, 0.0), moving_push_hold_newtons=(0.0, 0.0))
    base = modo.removesuffix("_m")
    share = 1.0 if base == "b21" else 0.0
    batch.set_locomotion_teacher(str(REPO / "assets"), str(professor / "tarefa.json"),
                                 str(professor / "b21_82578.mrlpolicy"), share=share, speeds=tuple(pedidos),
                                 student_speeds=tuple(aluna), takeover=(15.0, 45.0, 25.0, 20.0, 0.0))
    batch.set_teacher_oracle({"b21": 0, "oraculo1": 1, "oraculo2": 2, "oraculo3": 3, "oraculo4": 4}[base])
    batch.set_teacher_still_clip(PARADO[0])
    if modo.endswith("_m") and MUSCULO:
        batch.set_student_actuator(MUSCULO["rigidez"], MUSCULO["amortecimento"], MUSCULO["teto"], MUSCULO["gravidade"])
    batch.reset(444 + rumo)
    n = batch.num_envs
    zeros = np.zeros((n, batch.action_size), dtype=np.float32)
    done: dict[int, list] = {}
    rotulos = []
    lines = []
    for step in range(int(segundos / batch.policy_dt) + 2):
        batch.step(zeros)
        t = batch.teacher_targets()
        rotulos.append(t)
        if dumps is not None and 0 not in done:
            lines.append(batch.frame_lines(0))
        for row in batch.finished_episodes().tolist():
            done.setdefault(int(row[0]), row)
        if len(done) >= n:
            break
    if base == "b21":
        STATS.append(np.asarray(batch.teacher_drive_stats()))
    ep = np.array([done[i] for i in sorted(done)])
    caiu = ep[:, 4] < segundos - 0.05
    lab = np.concatenate(rotulos)
    a = batch.action_size
    valid = lab[:, 2 * a] > 0.5
    diff = np.abs(lab[valid, :a] - lab[valid, a:2 * a]) if valid.any() else np.zeros((1, a))
    if dumps is not None and lines:
        dumps.mkdir(parents=True, exist_ok=True)
        (dumps / f"{modo}_{rumo}.dump").write_text("".join(lines))
    return {
        "modo": modo, "rumo": rumo, "nome": NOMES[rumo] if exercicio == "rumos" else exercicio, "mundos": n,
        "pose_fim": float(ep[:, 15].mean()), "coluna_fim": float(ep[:, 17].mean()), "de_pe_fim": float(ep[:, 9].mean()),
        "quedas": int(caiu.sum()), "segundos_medio": float(ep[:, 4].mean()),
        "vel_rumo": float(ep[:, 12].mean()), "vel_rumo_min": float(ep[:, 12].min()),
        "desvio_rumo": float(ep[:, 31].mean()), "desvio_rumo_abs_max": float(np.abs(ep[:, 31]).max()),
        "peito_graus": float(ep[:, 16].mean()), "ajuda_max": float(ep[:, 30].max()),
        "professora": float(ep[:, 27].mean()), "rotulados": float(ep[:, 29].mean()),
        "rotulo_menos_cru_graus": float(np.degrees(diff.mean())),
    }


MUSCULO: dict = {}
PARADO = [2]
STATS: list = []


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--modos", nargs="+", default=["b21", "oraculo1"])
    p.add_argument("--rumos", nargs="+", type=int, default=list(range(8)))
    p.add_argument("--mundos", type=int, default=8)
    p.add_argument("--segundos", type=float, default=8.0)
    p.add_argument("--threads", type=int, default=4)
    p.add_argument("--professor", type=Path, default=RUNS / "rl_final_b21" / "professor")
    p.add_argument("--pedidos", nargs=8, type=float, default=[3.0, 3.0, 1.5, 1.5, 1.5, 1.5, 1.5, 3.0])
    p.add_argument("--aluna", nargs=8, type=float, default=[1.9, 1.45, 1.05, 1.2, 1.25, 1.2, 1.1, 1.45])
    p.add_argument("--saida", type=Path, default=None)
    p.add_argument("--exercicio", choices=list(EXERCICIOS), default="rumos")
    p.add_argument("--parado", type=int, default=2, help="o parado: 0 a B21, 1 tronco no clipe, 2 tudo no clipe")
    p.add_argument("--dumps", type=Path, default=None)
    p.add_argument("--musculo", type=Path, default=None,
                   help="o musculo da aluna nos modos *_m (json: rigidez, amortecimento, teto por eixo e gravidade); "
                        "sem o arquivo, o medido da B21 nos modos b21 desta rodada")
    args = p.parse_args()
    PARADO[0] = args.parado
    if args.musculo and args.musculo.exists():
        MUSCULO.update(json.loads(args.musculo.read_text()))
    mod = native()
    out = []
    for modo in args.modos:
        for rumo in args.rumos:
            t0 = time.time()
            r = medir(mod, modo, rumo, args.mundos, args.segundos, args.professor, args.pedidos, args.aluna,
                      args.threads, args.dumps, args.exercicio)
            r["s"] = round(time.time() - t0, 1)
            out.append(r)
            if modo == "b21" and rumo == args.rumos[-1] and STATS and not MUSCULO:
                m = np.mean(np.stack(STATS), axis=0)
                MUSCULO.update({"rigidez": np.clip(m[:, 0], 0, 2).tolist(), "amortecimento": np.clip(m[:, 1], 0, 2.5).tolist(),
                                "teto": np.clip(m[:, 2], 0, 3).tolist(), "gravidade": bool(m[:, 3].mean() > 0.5),
                                "antecipacao_nm": m[:, 4].tolist()})
                if args.musculo:
                    args.musculo.parent.mkdir(parents=True, exist_ok=True)
                    args.musculo.write_text(json.dumps(MUSCULO, indent=1))
                print("musculo medido da B21 (media por eixo): rigidez", np.round(m[:, 0], 2).tolist(), flush=True)
                print("  amortecimento", np.round(m[:, 1], 2).tolist(), flush=True)
                print("  teto", np.round(m[:, 2], 2).tolist(), " gravidade", np.round(m[:, 3], 2).tolist(), flush=True)
                print("  |antecipacao| N.m", np.round(m[:, 4], 1).tolist(), flush=True)
            print(f"{modo:9s} {r['nome']:16s} quedas {r['quedas']}/{r['mundos']}  vel {r['vel_rumo']:.2f} (min "
                  f"{r['vel_rumo_min']:.2f})  desvio {r['desvio_rumo']:+.1f} (|max| {r['desvio_rumo_abs_max']:.1f})  "
                  f"peito {r['peito_graus']:.1f}  ajuda {r['ajuda_max']:.4f}  prof {r['professora']:.2f}  "
                  f"rot {r['rotulados']:.0f}  rot-cru {r['rotulo_menos_cru_graus']:.1f}g  pose {r['pose_fim']:.2f}  "
                  f"coluna {r['coluna_fim']:.2f}  de_pe {r['de_pe_fim']:.2f}  [{r['s']}s]", flush=True)
    if args.saida:
        args.saida.parent.mkdir(parents=True, exist_ok=True)
        args.saida.write_text(json.dumps(out, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
