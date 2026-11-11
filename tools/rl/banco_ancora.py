#!/usr/bin/env python3
"""O banco fixo da ancora no campeao (E5, experimento 1; vault 04_locomocao/metodologia_rl.md, secao 8 item 2).

Roda o campeao com a maquinaria da suite (suite.captura: carregar, lote, politica - o lote e a politica montados como no treino)
nos cenarios APROVADOS: os retos (trote e corrida, os 8 rumos) e as paradas. O parado fica de fora (e o alvo do proximo
experimento) e, nas paradas, os passos depois de o corpo estar assentado ha mais de 1 s (o pedido zero e o centro de massa
abaixo de PARADO_MS): a ancora protege a freada, nao o parado. So as linhas da cabeca da locomocao. As duas condicoes da suite
(a media da rede e o ruido do treino). Um conjunto de sementes proprio, fora do 'decisao' e do 'diagnostico'.

Grava as observacoes cruas (as que a Policy recebe) e a acao media do campeao (a saida do ator, normalizada), subamostradas:

  MATTER_TRACKING_NATIVE=../../build-base-n2/python freio .venv/bin/python banco_ancora.py [--estados 40000]
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np

from suite import captura
from suite import cenarios as C

PADRAO_CK = "~/.local/share/matter-engine/rl/runs/rl_final_clipe/pontos/r71860_pausa_parado_1011.pt"
PADRAO_SAIDA = "~/.local/share/matter-engine/rl/ancora/banco_r71860.pt"
SEMENTES_BANCO = {"lote": 2001, "lote_parado": 2002, "reset": 2003, "ruido": 2004}
ASSENTADO_S = 1.0
PARADO_MS = 0.2  # o centro de massa abaixo disto (m/s) com o pedido zero: assentado

assert all(SEMENTES_BANCO[k] not in {s[k] for s in C.SEMENTES.values()} for k in SEMENTES_BANCO), "o banco nao treina no teste"


def coletar(cand, cen, condicao: str, mundos: int, threads: int):
    """As observacoes cruas das linhas da locomocao de um cenario (o laco de captura.rodar, sem as poses)."""
    import torch
    b = captura.lote(cand, cen, SEMENTES_BANCO, mundos, threads)
    model = captura.politica(cand, b)
    nomes = list(b.observation_names())
    i_cmd, i_vx, i_vy = (nomes.index(n) for n in ("comando.velocidade", "eq.vel_cm.frente", "eq.vel_cm.lado"))
    dt = float(b.policy_dt)
    T, W, A = int(round(cen.segundos / dt)), mundos, b.action_size
    colored, tau = cand.args.ruido_correlacionado
    keep = math.exp(-dt / max(tau, 1.0e-3))
    geradores = [np.random.default_rng([SEMENTES_BANCO["ruido"], w]) for w in range(W)]
    estado = np.stack([g.standard_normal(A) for g in geradores]).astype(np.float32)
    assentado = np.zeros(W)
    out = []
    for _ in range(T):
        obs = torch.as_tensor(b.observations())
        o = obs.numpy()
        parado = (np.abs(o[:, i_cmd]) <= 1e-6) & (np.hypot(o[:, i_vx], o[:, i_vy]) < PARADO_MS)
        assentado = np.where(parado, assentado + dt, 0.0)
        usar = (o[:, -1] > .5) & ~((cen.tipo == "parada") & (assentado > ASSENTADO_S))
        out.append(o[usar].copy())
        with torch.no_grad():
            obs_n = model.norm(obs)
            if condicao == "ruido":  # o ruido do treino, como captura.rodar
                dist = model.distribution(obs_n)
                e = np.stack([g.standard_normal(A) for g in geradores]).astype(np.float32)
                if colored > 0:
                    e2 = np.stack([g.standard_normal(A) for g in geradores]).astype(np.float32)
                    estado = keep * estado + math.sqrt(1.0 - keep ** 2) * e2
                    e = math.sqrt(1.0 - colored) * e + math.sqrt(colored) * estado
                a = dist.mean + dist.stddev * torch.as_tensor(e)
            else:
                a = model.actor(obs_n)
            b.step(model.env_action(a, obs).numpy().astype(np.float32))
        b.finished_episodes()
    return np.concatenate(out), model


def main() -> int:
    import torch
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", default=PADRAO_CK)
    p.add_argument("--receita", default=str(captura.RL / "receitas" / "campeao.json"))
    p.add_argument("--assets", default=str(captura.RL.parents[1] / "assets"))
    p.add_argument("--saida", default=PADRAO_SAIDA)
    p.add_argument("--estados", type=int, default=40000)
    p.add_argument("--mundos", type=int, default=C.MUNDOS)
    p.add_argument("--threads", type=int, default=6)
    a = p.parse_args()
    torch.set_num_threads(1)
    inicio = time.time()
    cand = captura.carregar("campeao", a.checkpoint, a.receita, a.assets)
    cens = [c for c in C.cenarios() if c.tipo in ("reto", "parada")]
    blocos, origem, nomes = [], [], []
    model = None
    for k, cen in enumerate(cens):
        for cond in C.CONDICOES:
            t = time.time()
            x, model = coletar(cand, cen, cond, a.mundos, a.threads)
            blocos.append(x)
            origem.append(np.full(len(x), len(nomes), np.int16))
            nomes.append(f"{cen.nome}__{cond}")
            print(f"{nomes[-1]}: {len(x)} estados, {time.time() - t:.1f} s", flush=True)
    obs, origem = np.concatenate(blocos), np.concatenate(origem)
    total = len(obs)
    pick = np.sort(np.random.default_rng(SEMENTES_BANCO["lote"]).choice(total, min(a.estados, total), replace=False))
    obs = torch.as_tensor(obs[pick])
    with torch.no_grad():
        acao = torch.cat([model.actor(model.norm(x)) for x in obs.split(8192)])
    meta = dict(cand.meta, cenarios=nomes, sementes=SEMENTES_BANCO, estados=len(obs), estados_coletados=int(total),
                assentado_s=ASSENTADO_S, parado_ms=PARADO_MS, observacao=cand.estado["settings"]["observacao"],
                acao="a saida do ator do campeao (a media, normalizada na faixa de cada junta)",
                segundos=round(time.time() - inicio, 1))
    saida = Path(a.saida).expanduser()
    saida.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"obs": obs, "acao": acao, "origem": torch.as_tensor(origem[pick]), "meta": meta}, saida)
    print(f"banco: {len(obs)} de {total} estados, {len(nomes)} capturas, {meta['segundos']} s -> {saida}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
