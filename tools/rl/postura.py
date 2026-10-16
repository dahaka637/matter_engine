#!/usr/bin/env python3
"""A postura funcional na mesma regua para humanos e politicas: a inclinacao do
tronco inteiro (pelve-cabeca) e da parte baixa (pelve-UpperChest), pelos centros
dos colisores, no rumo do deslocamento, contra a aceleracao (cruzeiro, frear,
acelerar; a reta contra atan(a/g)). Os humanos sao os clipes retargetados no
corpo do RL (o padrao de referencia de 04_locomocao/postura_funcional.md).

  .venv/bin/python -I postura.py clipes
  .venv/bin/python -I postura.py politica RUN TAREFA EXERCICIO[,EXERCICIO...]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build-rl" / "python"))
import matter_rl_native as native  # noqa: E402

BODY = str(REPO / "assets" / "rl" / "bodies" / "football_player_v2.json")
CLIPS = REPO / "assets" / "animations" / "clips"
DATA = Path.home() / ".local" / "share" / "matter-engine" / "rl"
G = 9.81
DEG = 180.0 / np.pi


def parse(text: str) -> list[dict]:
    frames, cur = [], None
    for line in text.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "F":
            cur = {"t": float(p[1])}
            frames.append(cur)
        elif p[0] in ("C", "B") and cur is not None:
            cur[p[1]] = np.array([float(v) for v in p[2:5]])
        elif p[0] == "V" and cur is not None:
            cur["V"] = np.array([float(v) for v in p[1:4]])
        elif p[0] == "L" and cur is not None:
            cur["L"] = float(p[1])
    return frames


WRAP = False


def smooth(x: np.ndarray, n: int) -> np.ndarray:
    if n <= 1 or len(x) < n:
        return x
    k = np.ones(n) / n
    pad = n // 2
    xp = np.pad(x, ((pad, pad), (0, 0)), mode="wrap" if WRAP else "edge")
    return np.stack([np.convolve(xp[:, j], k, mode="valid")[: len(x)] for j in range(x.shape[1])], axis=1)


def series(frames: list[dict], dt: float, velocity: np.ndarray | None) -> dict:
    P = np.array([f["Pelvis"] for f in frames])
    U = np.array([f["UpperChest"] for f in frames])
    H = np.array([f["Head"] for f in frames])
    if velocity is None:
        velocity = np.gradient(P, dt, axis=0)
    v = smooth(velocity[:, :2], max(1, int(round(0.2 / dt))))
    a = smooth(np.gradient(v, dt, axis=0), max(1, int(round(0.25 / dt))))
    speed = np.linalg.norm(v, axis=1)
    u = v / np.maximum(speed, 1e-6)[:, None]
    left = np.stack([-u[:, 1], u[:, 0]], axis=1)
    a_long = np.sum(a * u, axis=1)
    a_lat = np.sum(a * left, axis=1)

    def lean(base, tip):
        d = tip - base
        return (np.degrees(np.arctan2(np.sum(d[:, :2] * u, axis=1), d[:, 2])),
                np.degrees(np.arctan2(np.sum(d[:, :2] * left, axis=1), d[:, 2])))

    trunk_long, trunk_lat = lean(P, H)
    low_long, _ = lean(P, U)
    up_long, _ = lean(U, H)
    return dict(speed=speed, a_long=a_long, a_lat=a_lat, trunk=trunk_long, trunk_lat=trunk_lat, low=low_long, up=up_long)


def fit(x: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    if len(x) < 5 or np.ptp(x) < 1e-3:
        return float("nan"), float(np.mean(y)) if len(y) else float("nan")
    s, b = np.polyfit(x, y, 1)
    return float(s), float(b)


def report(name: str, s: dict, moving: float = 0.6) -> None:
    m = s["speed"] > moving
    if m.sum() < 5:
        print(f"{name:28s} parado: tronco(base) n/a")
        return
    t, lo, up = s["trunk"][m], s["low"][m], s["up"][m]
    al, at, tl = s["a_long"][m], s["a_lat"][m], s["trunk_lat"][m]
    cruise = np.abs(al) < 0.7
    brake = al < -1.5
    accel = al > 1.5
    want_long = np.degrees(np.arctan(al / G))
    want_lat = np.degrees(np.arctan(at / G))
    slope_long, off_long = fit(want_long, t)
    slope_lat, _ = fit(want_lat, tl)
    f = lambda arr, mask: f"{arr[mask].mean():+5.1f}" if mask.sum() >= 3 else "  n/a"
    print(f"{name:28s} vel {s['speed'][m].mean():4.1f} | tronco cruzeiro {f(t, cruise)} frear {f(t, brake)} acelerar {f(t, accel)}"
          f" | baixo/alto cruzeiro {f(lo, cruise)}/{f(up, cruise)} | inclinacao~atan(a/g): frente x{slope_long:+.2f}"
          f" lado x{slope_lat:+.2f} | a_long [{np.percentile(al, 5):+.1f},{np.percentile(al, 95):+.1f}]"
          f" a_lat [{np.percentile(at, 5):+.1f},{np.percentile(at, 95):+.1f}] | n {m.sum()}")


def trim(s: dict, dt: float, loops: bool) -> dict:
    # Clipe sem ciclo: fora 0,3 s de cada ponta (o filtro deforma as bordas).
    if loops:
        return s
    k = int(round(0.3 / dt))
    return {key: v[k:len(v) - k] for key, v in s.items()}


def clips() -> None:
    global WRAP
    names = ["natural_idle", "alert_idle", "ual_walk", "jog", "trot_forward", "run_forward_medium", "mixamo_running",
             "sprint", "sprint_forward", "walk_backward", "trot_backward", "run_backward", "sprint_backward",
             "run_forward_arc_left", "run_forward_arc_right", "running_turn_180", "change_direction", "running_to_turn",
             "turn_to_running", "idle_to_sprint", "crouched_to_sprint", "jog_strafe_left"]
    for name in names:
        path = CLIPS / f"{name}.matteranim.json"
        meta = json.loads(path.read_text())
        seconds = float(meta["durationSeconds"]) * (3.0 if meta.get("loops") else 1.0)
        frames = parse(native.reference_dump(BODY, [str(path)], seconds))
        dt = frames[1]["t"] - frames[0]["t"]
        WRAP = bool(meta.get("loops"))
        s = trim(series(frames, dt, None), dt, WRAP)
        if name.endswith("idle"):
            P = np.array([f["Pelvis"] for f in frames]); H = np.array([f["Head"] for f in frames])
            d = (H - P).mean(axis=0)
            print(f"{name:28s} parado: linha pelve-cabeca {np.degrees(np.arctan2(np.hypot(d[0], d[1]), d[2])):.1f} graus da vertical"
                  f" (x {np.degrees(np.arctan2(d[0], d[2])):+.1f})")
            continue
        report(name, s)


def policy(run: str, task: str, drills: list[str]) -> None:
    pol = native.Policy(str(DATA / "runs" / run / "exports" / "policy_latest.mrlpolicy"))
    path = str(REPO / "assets" / "rl" / "tasks" / f"{task}.json")
    worlds = 16
    for drill in drills:
        batch = native.BodyBatch(path, environments=worlds, per_scene=4, threads=8, seed=20261015, evaluation=True,
                                 evaluation_seconds=32.0, solver_position=20, solver_velocity=20, drill=drill)
        dt = batch.policy_dt
        frames = [[] for _ in range(worlds)]
        for _ in range(int(30.0 / dt)):
            obs = np.ascontiguousarray(batch.observations(), dtype=np.float32)
            act = np.ascontiguousarray(pol.evaluate(obs), dtype=np.float32)
            batch.step(act, act)
            for e in range(worlds):
                fr = parse("F 0 0\n" + batch.frame_lines(e))
                if fr and "Pelvis" in fr[0] and "V" in fr[0]:
                    frames[e].append(fr[0])
        agg: dict[str, list] = {}
        for e in range(worlds):
            if len(frames[e]) < 20:
                continue
            vel = np.array([f["V"] for f in frames[e]])
            s = series(frames[e], dt, vel)
            for k, v in s.items():
                agg.setdefault(k, []).append(v[15:])  # sem o primeiro meio segundo
        s = {k: np.concatenate(v) for k, v in agg.items()}
        report(f"{run}:{drill}", s)
        # Parado: no rumo pedido para o peito (a camera).
        rows = []
        for e in range(worlds):
            for f in frames[e][15:]:
                if np.hypot(f["V"][0], f["V"][1]) > 0.3 or "L" not in f:
                    continue
                u = np.array([np.cos(f["L"]), np.sin(f["L"])])
                d1 = f["Head"] - f["Pelvis"]; d2 = f["UpperChest"] - f["Pelvis"]
                rows.append((np.degrees(np.arctan2(d1[:2] @ u, d1[2])), np.degrees(np.arctan2(d2[:2] @ u, d2[2]))))
        if len(rows) > 20:
            r = np.array(rows)
            print(f"{run}:{drill:18s} parado ({len(r)} quadros): tronco inteiro {r[:, 0].mean():+5.1f} parte baixa {r[:, 1].mean():+5.1f}")


if __name__ == "__main__":
    if sys.argv[1] == "clipes":
        clips()
    else:
        policy(sys.argv[2], sys.argv[3], sys.argv[4].split(","))
