#!/usr/bin/env python3
"""A instrumentacao do risco de queda (etapa 1 da prevencao; vault
04_locomocao/ajuda_auxiliar.md): so le - roda o ultimo checkpoint de um treino em
avaliacao (nada se escreve nele) e mede, por decisao, os sinais de risco que a
entrada ja tem, para calibrar o potencial de risco antes de mexer na nota.

Para cada sinal e para o risco composto: quanto ele separa os instantes de
AVISO (de 1,5 s a 0,5 s antes de uma queda) dos normais (os episodios sem queda
e o comeco dos que cairam, mais de 2 s antes), pela area sob a curva (AUC: 0,5 =
nada, 1 = separa tudo), e o perfil medio do risco nos 2 s antes da queda.

  .venv/bin/python risco_queda.py --treino loco_v3 --exercicios estrela_trote,estrela
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from matter_rl import paths, storage  # noqa: E402
from matter_rl.locomotion_trainer import native  # noqa: E402
from matter_rl.model import ActorCritic  # noqa: E402

GAMMA = 0.99


def smoothstep(edge0: float, edge1: float, x: np.ndarray) -> np.ndarray:
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def signals(obs: np.ndarray, index: dict[str, int]) -> dict[str, np.ndarray]:
    """Os sinais de risco (cada um de 0 a 1, 1 = perigoso) das observacoes cruas."""
    col = lambda name: obs[:, index[name]]  # noqa: E731
    capture = np.clip(col("exec.captura.fora") / 0.08, 0.0, 1.0)
    margin = 1.0 - smoothstep(0.0, 0.06, col("exec.margem"))
    tilt_rate = np.clip(np.hypot(col("corpo.Chest.giro.x"), col("corpo.Chest.giro.y")) / 3.0, 0.0, 1.0)
    pelvis = np.clip((0.60 - col("corpo.pelve.altura")) / 0.15, 0.0, 1.0)
    fall_speed = np.clip(np.maximum(0.0, -col("corpo.Pelvis.vel.z")) / 1.5, 0.0, 1.0)
    support = np.clip(1.0 - (col("pe_esq.apoio") + col("pe_dir.apoio")), 0.0, 1.0)
    return {"captura_fora": capture, "margem_com": margin, "giro_peito": tilt_rate, "altura_pelve": pelvis,
            "queda_pelve": fall_speed, "apoio": support}


def raw_signals(obs: np.ndarray, index: dict[str, int]) -> dict[str, np.ndarray]:
    """Os mesmos sinais SEM o corte (as unidades do corpo), para escolher as escalas."""
    col = lambda name: obs[:, index[name]]  # noqa: E731
    up = np.clip(col("corpo.Chest.cima.z"), -1.0, 1.0)
    return {
        "captura_fora_m": col("exec.captura.fora"),
        "margem_com_m": col("exec.margem"),
        "inclinacao_peito_graus": np.degrees(np.arccos(up)),
        "giro_peito_rad_s": np.hypot(col("corpo.Chest.giro.x"), col("corpo.Chest.giro.y")),
        "queda_pelve_m_s": -col("corpo.Pelvis.vel.z"),
        "apoio_peso": col("pe_esq.apoio") + col("pe_dir.apoio"),
        "vmc_resto_g": np.hypot(col("exec.vmc.resto.x"), col("exec.vmc.resto.y")),
        "vmc_pedida_g": np.hypot(col("exec.vmc.pedida.x"), col("exec.vmc.pedida.y")),
        "urgencia_recupera": col("exec.recupera.urgencia"),
        "ajuda_forca_peso": np.hypot(col("ajuda.forca.x"), col("ajuda.forca.y")),
    }


def risk_v2(obs: np.ndarray, index: dict[str, int]) -> np.ndarray:
    """O risco recalibrado pelos dados (a v1 saturava: a captura fora e a margem do
    COM ficam acima das escalas de 0,08 e 0,06 m no andar normal): as escalas vem
    das medianas de aviso e normal medidas no loco_v3 (17/10)."""
    raw = raw_signals(obs, index)
    capture = np.clip(raw["captura_fora_m"] / 0.8, 0.0, 1.0)
    margin = np.clip(-raw["margem_com_m"] / 0.30, 0.0, 1.0)
    chest = np.clip(raw["giro_peito_rad_s"] / 2.5, 0.0, 1.0)
    residual = np.clip(raw["vmc_resto_g"] / 0.40, 0.0, 1.0)
    fall = np.clip(np.maximum(0.0, raw["queda_pelve_m_s"]) / 0.8, 0.0, 1.0)
    return np.clip(0.25 * capture + 0.20 * margin + 0.20 * chest + 0.20 * residual + 0.15 * fall, 0.0, 1.0)


WEIGHTS = {"captura_fora": 0.35, "margem_com": 0.25, "giro_peito": 0.12, "altura_pelve": 0.12, "queda_pelve": 0.08,
           "apoio": 0.05}


def risk(parts: dict[str, np.ndarray]) -> np.ndarray:
    return np.clip(sum(WEIGHTS[k] * parts[k] for k in WEIGHTS), 0.0, 1.0)


def auc(positive: np.ndarray, negative: np.ndarray) -> float:
    if len(positive) == 0 or len(negative) == 0:
        return float("nan")
    values = np.concatenate([positive, negative])
    ranks = values.argsort().argsort().astype(np.float64) + 1.0
    # (Empates: o rank medio.)
    _, inverse, counts = np.unique(values, return_inverse=True, return_counts=True)
    sums = np.zeros(len(counts))
    np.add.at(sums, inverse, ranks)
    ranks = (sums / counts)[inverse]
    n_pos, n_neg = len(positive), len(negative)
    return float((ranks[:n_pos].sum() - n_pos * (n_pos + 1) / 2.0) / (n_pos * n_neg))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", required=True)
    parser.add_argument("--exercicios", default="estrela_trote,estrela")
    parser.add_argument("--mundos", type=int, default=64)
    parser.add_argument("--segundos", type=float, default=20.0)
    parser.add_argument("--threads", type=int, default=3)
    args = parser.parse_args()
    run = paths.runs_dir() / args.treino if hasattr(paths, "runs_dir") else Path.home() / ".local/share/matter-engine/rl/runs" / args.treino
    state = storage.load_checkpoint(storage.latest_checkpoint(run), "cpu")
    net = state["network"]
    model = ActorCritic(len(state["obs_names"]), len(state["critic_names"]), len(state["action_names"]),
                        np.full(len(state["action_names"]), 0.3, dtype=np.float32), actor_hidden=net["actor"],
                        critic_hidden=net["critic"])
    model.load_state_dict(state["model"])
    model.eval()
    scale = torch.as_tensor(np.asarray(state["action_scale"], dtype=np.float32))
    nat = native()
    task = str(run / "tarefa.json")
    print(f"{args.treino} rodada {state.get('counters', {}).get('iteration')} · {args.mundos} mundos x {args.segundos:.0f} s")
    for drill in args.exercicios.split(","):
        batch = nat.LocomotionBatch(task, str(paths.ASSETS), environments=args.mundos, threads=args.threads, seed=424242,
                                    evaluation=True, evaluation_seconds=args.segundos, drill=drill)
        index = {name: i for i, name in enumerate(batch.observation_names())}
        columns = {n: i for i, n in enumerate(batch.episode_columns())}
        batch.set_authority(1.0, 1.0, 1.0, 1.0)
        batch.reset(424242)
        worlds = batch.num_envs
        dt = batch.policy_dt
        buffers: list[list[dict[str, float]]] = [[] for _ in range(worlds)]
        done = np.zeros(worlds, dtype=bool)
        raw_names = list(raw_signals(np.zeros((1, len(index))), {n: i for n, i in index.items()}).keys())
        keys = [*WEIGHTS, "risco", "risco_v2", *raw_names]
        warn: dict[str, list[float]] = {k: [] for k in keys}
        normal: dict[str, list[float]] = {k: [] for k in keys}
        profile = np.zeros(int(2.0 / dt) + 1)
        profile2 = np.zeros(int(2.0 / dt) + 1)
        profile_n = 0
        falls = 0
        with torch.no_grad():
            for _ in range(int(args.segundos / dt) + 2):
                obs = np.array(batch.observations())
                parts = signals(obs, index)
                composite = risk(parts)
                parts_raw = raw_signals(obs, index)
                composite2 = risk_v2(obs, index)
                for w in range(worlds):
                    if not done[w]:
                        buffers[w].append({**{k: float(v[w]) for k, v in parts.items()}, "risco": float(composite[w]), "risco_v2": float(composite2[w]),
                                           **{k: float(v[w]) for k, v in parts_raw.items()}})
                mean = model.actor(model.obs_norm(torch.as_tensor(obs)))
                batch.step(np.ascontiguousarray((mean * scale).numpy(), dtype=np.float32))
                for row in batch.finished_episodes():
                    w = int(row[columns["mundo"]])
                    if done[w]:
                        continue
                    done[w] = True
                    series = buffers[w]
                    fell = int(row[columns["razao"]]) == 1
                    n = len(series)
                    far = int(2.0 / dt)
                    early_lo, early_hi = n - int(1.5 / dt), n - int(0.5 / dt)
                    for i, item in enumerate(series):
                        bucket = None
                        if not fell:
                            bucket = normal
                        elif i < n - far:
                            bucket = normal
                        elif early_lo <= i < early_hi:
                            bucket = warn
                        if bucket is not None:
                            for k, v in item.items():
                                bucket[k].append(v)
                    if fell:
                        falls += 1
                        tail = [item["risco"] for item in series[-len(profile):]]
                        profile[len(profile) - len(tail):] += np.array(tail)
                        tail2 = [item["risco_v2"] for item in series[-len(profile2):]]
                        profile2[len(profile2) - len(tail2):] += np.array(tail2)
                        profile_n += 1
                if done.all():
                    break
        print(f"\n{drill}: {falls}/{worlds} quedas")
        print("  sinal            AUC aviso x normal   (aviso medio / normal medio)")
        for k in keys:
            if k == raw_names[0]:
                print("  --- os mesmos sinais crus (AUC acima de 0,5: sobe antes da queda; abaixo: cai) ---")
            a = auc(np.array(warn[k]), np.array(normal[k]))
            wm = np.median(warn[k]) if warn[k] else float("nan")
            nm = np.median(normal[k]) if normal[k] else float("nan")
            print(f"  {k:24s} {a:6.3f}              (mediana aviso {wm:.3f} / normal {nm:.3f})")
        if profile_n:
            avg = profile / profile_n
            marks = [0, len(avg) // 4, len(avg) // 2, 3 * len(avg) // 4, len(avg) - 1]
            print("  risco v1 medio nos 2 s antes da queda (-2,0 -1,5 -1,0 -0,5 0 s): "
                  + " ".join(f"{avg[m]:.2f}" for m in marks))
            avg2 = profile2 / profile_n
            print("  risco v2 medio nos 2 s antes da queda (-2,0 -1,5 -1,0 -0,5 0 s): "
                  + " ".join(f"{avg2[m]:.2f}" for m in marks))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
