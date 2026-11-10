"""Traz a media da cabeca da locomocao para dentro do alcance (04/11 09h50): sem o limite a media da rede saiu (os bracos a
2,8x o alcance) e o corpo se afastou do clipe. Destila a cabeca nela mesma com a media presa a [-1, 1] (dentro, igual; fora,
na borda), em observacoes de todos os exercicios do rastreio na forca do checkpoint, e grava no ultimo.pt (arquive antes).

  MATTER_TRACKING_NATIVE=build-rl-n1/python tools/rl/.venv/bin/python tools/rl/trazer_ao_alcance.py --treino rl_final_clipe
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sondas_rl_final as S  # noqa: E402

L = S.L


def colher(state, levels, decisoes: int) -> torch.Tensor:
    obs_all = []
    casos = [(k, dict(evaluation_kind=4, **kw), m) for k, kw, m in S.CASES] + [("lev_giro", dict(evaluation_kind=6), 0)]
    for key, kw, man in casos:
        b = L.native.GetUpAmpBatch(S.ASSETS, environments=8, threads=4, seed=91, evaluation=True, **kw, **S.TRACK)
        S.setup(b, levels)
        if man:
            b.set_maneuvers(0.0, man)
        b.reset(91)
        act = S.policy(b, state, 0.1, (0.12, 0.08, 0.05))
        for _ in range(decisoes):
            obs = np.array(b.observations())
            obs_all.append(obs[obs[:, -1] > .5])
            b.step(act())
    # (O parado e o idle.)
    b = L.native.GetUpAmpBatch(S.ASSETS, environments=8, threads=4, seed=91, clip_start_fraction=0.0, standing_start_fraction=1.0,
                               moving_fraction=0.0, avoid_fraction=0.0, locomotion_fraction=0.0, turn_fraction=0.0,
                               **{k: v for k, v in S.TRACK.items() if not k.startswith("moving")})
    S.setup(b, levels)
    b.reset(91)
    act = S.policy(b, state, 0.1, (0.12, 0.08, 0.05))
    for _ in range(decisoes):
        obs = np.array(b.observations())
        obs_all.append(obs[obs[:, -1] > .5])
        b.step(act())
    return torch.as_tensor(np.concatenate(obs_all), dtype=torch.float32)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--treino", default="rl_final_clipe")
    ap.add_argument("--decisoes", type=int, default=150)
    ap.add_argument("--passos", type=int, default=3000)
    ap.add_argument("--taxa", type=float, default=1e-4)
    a = ap.parse_args()
    path = S.RUNS / a.treino / "ultimo.pt"
    state = torch.load(path, map_location="cpu", weights_only=False)
    levels = [float(state["counters"].get("ajuda_exercicios", {}).get(k, 1.0)) for k in L.DRILL_KEYS]
    obs = colher(state, levels, a.decisoes)
    print(f"{len(obs)} observacoes da locomocao", flush=True)
    b = L.native.GetUpAmpBatch(S.ASSETS, environments=1, threads=1, seed=1, evaluation=True, evaluation_kind=4, **S.TRACK)
    model = L.Policy(b.observations().shape[1], b.action_size, b.action_ranges(), split=True)
    model.load_state_dict(state["model"])
    with torch.no_grad():
        obs_n = model.norm(obs)
        target = model.actor(obs_n).clamp(-1.0, 1.0)
        before = (model.actor(obs_n).abs() - 1).clamp_min(0)
    print(f"antes: fora do alcance em {(before > 0).float().mean():.0%} das saidas, excesso medio {before.mean():.3f}, max {before.max():.2f}")
    params = list(model.actor.locomotion.parameters())
    opt = torch.optim.Adam(params, lr=a.taxa)
    for step in range(a.passos):
        idx = torch.randint(0, len(obs_n), (4096,))
        loss = (model.actor(obs_n[idx]) - target[idx]).pow(2).mean()
        opt.zero_grad()
        loss.backward()
        opt.step()
        if step % 500 == 0 or step == a.passos - 1:
            print(f"passo {step}: erro {loss.item():.5f}", flush=True)
    with torch.no_grad():
        out = model.actor(obs_n)
        after = (out.abs() - 1).clamp_min(0)
        inside = before.sum(-1) == 0
        drift = (out[inside] - target[inside]).abs().mean() if inside.any() else torch.tensor(0.0)
    print(f"depois: fora do alcance em {(after > 0).float().mean():.0%}, excesso medio {after.mean():.4f}, max {after.max():.2f};"
          f" mudanca nas linhas que ja estavam dentro {drift:.4f}")
    state["model"] = model.state_dict()
    state["counters"]["trazido_ao_alcance"] = state["counters"]["atualizacao"]
    torch.save(state, path)
    print("gravado em", path)


if __name__ == "__main__":
    main()
