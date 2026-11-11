"""A captura: roda um candidato (checkpoint + receita) nos cenarios fixos e grava, por passo e mundo, as poses dos elos do corpo e
da referencia (o clipe seguido), os contatos de cada elo, o pedido (clipe, velocidade, rumo), a cabeca que dirige, a acao, a
telemetria das juntas (a cada 3 passos) e, no fim, os episodios terminados. Um npz por cenario e condicao.

O ambiente sai SO de levantar_amp.configurar com os contadores do checkpoint (a traducao unica das flags) e a forca auxiliar em
zero. A politica como o main() do treinador a monta (o residuo, o alcance por parte, a autoridade, o piso do desvio e o ruido
parado). O ruido da condicao 'ruido' e o do treino: o desvio da politica, o --ruido-correlacionado e o --ruido-parado, com um
gerador por mundo. As medidas nao leem a nota: as colunas do locomotion_feedback ficam gravadas so para diagnostico.
"""
from __future__ import annotations

import hashlib
import inspect
import json
import math
import os
import re
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from suite import cenarios as C

RL = Path(__file__).resolve().parents[1]
TELEMETRIA_CADA = 3


def sha1(path: Path) -> str:
    h = hashlib.sha1()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _treinador():
    """O levantar_amp (importado tarde: o MATTER_TRACKING_NATIVE tem de estar no ambiente antes)."""
    if "MATTER_TRACKING_NATIVE" not in os.environ:
        raise SystemExit("defina MATTER_TRACKING_NATIVE (o build congelado do candidato) ou passe --build")
    import sys
    sys.path.insert(0, str(RL))
    import levantar_amp as L
    return L


@dataclass
class Candidato:
    nome: str
    checkpoint: Path
    receita: Path
    assets: Path
    args: object = None
    estado: dict = field(default_factory=dict)
    meta: dict = field(default_factory=dict)


def carregar(nome: str, checkpoint: str | Path, receita: str | Path, assets: str | Path) -> Candidato:
    """O candidato: a receita lida e aplicada como no treino (ler_receita: o estado de modulo que ela liga) e o checkpoint."""
    import torch
    L = _treinador()
    checkpoint, receita, assets = Path(checkpoint).expanduser(), Path(receita).expanduser(), Path(assets).expanduser()
    if not receita.exists():
        receita = RL / "receitas" / f"{receita.name}.json" if not receita.suffix else RL / "receitas" / receita.name
    argv = json.loads(receita.read_text())["argumentos"]
    # (As receitas nao trazem o --treino, que o parser exige: o nome do treino so serve para as pastas do main.)
    args = L.ler_receita(list(argv) + ([] if "--treino" in argv else ["--treino", "suite"]))
    estado = torch.load(checkpoint, map_location="cpu", weights_only=False)
    construido = set(dir(L.native.GetUpAmpBatch))
    pedidos = sorted(set(re.findall(r'hasattr\(b, "(set_\w+)"\)', inspect.getsource(L.configurar))))
    ragdoll = assets / "characters/football_player/FootballPlayerV1.ragdoll.json"
    meta = {"nome": nome, "checkpoint": str(checkpoint), "checkpoint_sha1": sha1(checkpoint),
            "atualizacao": int(estado["counters"].get("atualizacao", -1)), "receita": str(receita),
            "receita_sha1": sha1(receita), "build": os.environ["MATTER_TRACKING_NATIVE"], "assets": str(assets),
            "ragdoll_sha1": sha1(ragdoll),
            # (configurar pula calado o setter que o build nao tem: a lista fica registrada no resultado.)
            "setters_ausentes_no_build": [s for s in pedidos if s not in construido]}
    return Candidato(nome, checkpoint, receita, assets, args, estado, meta)


def politica(cand: Candidato, b):
    """A politica do checkpoint montada como no treino (levantar_amp.piso_do_desvio e preparar_politica)."""
    L = _treinador()
    args = cand.args
    model = L.Policy(b.obs_size, b.action_size, b.action_ranges(), split=args.habilidades_separadas)
    L.piso_do_desvio(model, b, args)
    model.load_state_dict(cand.estado["model"])
    L.preparar_politica(model, b, args)
    model.eval()
    return model


def lote(cand: Candidato, cen: C.Cenario, sementes: dict, mundos: int, threads: int):
    """O lote de um cenario: a avaliacao com os ajustes de todo lote (montar_v2, sem empurrao andando), o configurar da receita,
    a forca auxiliar em zero e o que o cenario fixa (a ordem de parar, o equilibrio do parado)."""
    L = _treinador()
    plain = dict(L.montar_v2(cand.args), moving_push_newtons=(0.0, 0.0), moving_push_hold_newtons=(0.0, 0.0))
    if cen.nome.startswith("parada_longa_"):
        # (SUITE_V2: a parada longa precisa do episodio da locomocao inteiro - a receita o corta em --locomocao-segundos.)
        plain["locomotion_seconds"] = cen.segundos
    assets = str(cand.assets)
    if cen.tipo == "parado":
        b = L.get_up_batch(assets, environments=mundos, threads=threads, seed=sementes["lote_parado"], evaluation=True,
                           evaluation_kind=8, episode_seconds=cen.segundos, **plain)
    else:
        b = L.get_up_batch(assets, environments=mundos, threads=threads, seed=sementes["lote"], evaluation=True,
                           evaluation_kind=4, evaluation_band=cen.faixa, heading=cen.rumo, episode_seconds=cen.segundos, **plain)
    L.configurar(b, cand.args, cand.estado["counters"], False)
    b.set_drill_assist([0.0] * len(L.DRILL_KEYS))
    if cen.tipo == "parado":
        b.set_balance_drill(*cen.equilibrio)
        b.set_fire(0.0, (1.0, 1.0), (1.0, 1.0), L.FIRE_HOLD_SECONDS, 0.0)
    elif cen.tipo == "fogo":
        hit, hold = L.fire_bands(C.FOGO_NIVEL)
        b.set_fire(1.0, hit, hold, L.FIRE_HOLD_SECONDS, L.FIRE_HOLD_FRACTION)
    elif cen.tipo == "parada":
        b.set_final_stop(1.0, cen.ordem, cen.ordem, cand.args.parada_freio, True)
    b.reset(sementes["reset"])
    nomes = cand.estado.get("settings", {}).get("observacao")
    if nomes is not None and list(nomes) != list(b.observation_names()):
        raise SystemExit(f"{cand.nome}: as entradas do lote nao batem com as do checkpoint ({b.obs_size} contra {len(nomes)})")
    return b


def ler_linhas(text: str, idx: dict, out: np.ndarray, contato: np.ndarray | None = None) -> tuple:
    """Uma frame_lines/reference_lines: os elos em out[elo] = (x y z qx qy qz qw); os contatos marcam contato[elo]; devolve a
    linha Q (clipe, t, forca, controle, pedido, rumo) ou None."""
    q = None
    for line in text.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] in ("B", "C"):
            k = idx.get(p[1])
            if k is not None:
                out[k] = [float(x) for x in p[2:9]]
        elif p[0] == "P" and contato is not None:
            contato[int(float(p[4]))] = True
        elif p[0] == "Q":
            q = (p[1], float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6]))
    return q


def colunas_episodios(b, linhas: list) -> list[str]:
    """Os nomes das colunas dos episodios: os do lote (episode_columns, 35) completados pelos da binding (inspecao.comum, 47 - as
    linhas tem mais colunas que os nomes que o lote declara) e o passo do fim."""
    from inspecao.comum import EPISODE_COLUMNS
    nomes = list(b.episode_columns())
    largura = len(linhas[0]) - 1 if linhas else len(EPISODE_COLUMNS)
    nomes += list(EPISODE_COLUMNS[len(nomes):largura])
    nomes += [f"c{k}" for k in range(len(nomes), largura)]
    return nomes + ["passo_fim"]


def rodar(cand: Candidato, cen: C.Cenario, condicao: str, conjunto: str, saida: Path, mundos: int = C.MUNDOS,
          threads: int = 4, log=print) -> Path:
    """Um cenario numa condicao ('det' a media da rede, 'ruido' o ruido do treino). Devolve o npz."""
    import torch
    L = _treinador()
    sementes = C.SEMENTES[conjunto]
    inicio = time.time()
    b = lote(cand, cen, sementes, mundos, threads)
    model = politica(cand, b)
    from inspecao import comum
    comum.RAGDOLL = cand.assets / "characters/football_player/FootballPlayerV1.ragdoll.json"
    ragdoll = json.loads(comum.RAGDOLL.read_text())
    elos = [lk["id"] for lk in ragdoll["links"]]
    idx = {n: i for i, n in enumerate(elos)}
    dt = float(b.policy_dt)
    T = int(round(cen.segundos / dt))
    W, A, E = mundos, b.action_size, len(elos)
    corpo = np.full((W, T, E, 7), np.nan, np.float32)
    ref = np.full((W, T, E, 7), np.nan, np.float32)
    contato = np.zeros((W, T, E), bool)
    info = np.full((W, T, 5), np.nan, np.float32)
    clipe = np.full((W, T), -1, np.int16)
    clipes: list[str] = []
    episodio = np.zeros((W, T), np.int16)
    cabeca_loco = np.zeros((W, T), bool)
    acao = np.zeros((W, T, A), np.float32)
    feedback = np.full((W, T, len(L.TRACKING_TERMS)), np.nan, np.float32)
    tele_passos = np.arange(0, T, TELEMETRIA_CADA)
    telemetria = np.full((W, len(tele_passos), E, 12), np.nan, np.float32)
    linhas = []
    conta = np.zeros(W, np.int16)
    # (O ruido do treino, levantar_amp.py no laco da coleta: o estado correlacionado de cada mundo e eixo, variancia 1.)
    colored, tau = cand.args.ruido_correlacionado
    keep = math.exp(-dt / max(tau, 1.0e-3))
    geradores = [np.random.default_rng([sementes["ruido"], w]) for w in range(W)]
    estado = np.stack([g.standard_normal(A) for g in geradores]).astype(np.float32)
    for i in range(T):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            obs_n = model.norm(obs)
            if condicao == "ruido":
                dist = model.distribution(obs_n)
                e1 = np.stack([g.standard_normal(A) for g in geradores]).astype(np.float32)
                if colored > 0:
                    e2 = np.stack([g.standard_normal(A) for g in geradores]).astype(np.float32)
                    estado = keep * estado + math.sqrt(1.0 - keep ** 2) * e2
                    e = math.sqrt(1.0 - colored) * e1 + math.sqrt(colored) * estado
                else:
                    e = e1
                a = dist.mean + dist.stddev * torch.as_tensor(e)
            else:
                a = model.actor(obs_n)
            env = model.env_action(a, obs).numpy().astype(np.float32)
        cabeca_loco[:, i] = (obs[:, -1] > 0.5).numpy()
        b.step(env)
        acao[:, i] = env
        for r in b.finished_episodes().tolist():
            linhas.append([float(x) for x in r] + [float(i)])
            conta[int(r[0])] += 1
        fb = np.asarray(b.locomotion_feedback())
        feedback[:, i, :min(fb.shape[1], feedback.shape[2])] = fb[:, :feedback.shape[2]]
        tele = i % TELEMETRIA_CADA == 0
        for w in range(W):
            episodio[w, i] = conta[w]
            ler_linhas(b.frame_lines(w), idx, corpo[w, i], contato[w, i])
            q = ler_linhas(b.reference_lines(w), idx, ref[w, i])
            if q:
                if q[0] not in clipes:
                    clipes.append(q[0])
                clipe[w, i] = clipes.index(q[0])
                info[w, i] = q[1:]
            if tele:
                telemetria[w, i // TELEMETRIA_CADA] = np.asarray(b.body_joint_telemetry(w))
    # (Os episodios que acabam no mesmo passo chegam na ordem das threads: por passo e mundo, a captura fica deterministica.)
    linhas.sort(key=lambda r: (r[-1], r[0]))
    saida.mkdir(parents=True, exist_ok=True)
    tag = f"{cen.nome}__{condicao}"
    colunas = colunas_episodios(b, linhas)
    path = saida / f"{tag}.npz"
    np.savez_compressed(path, corpo=corpo, referencia=ref, contato=contato, info=info, clipe=clipe, episodio=episodio,
                        cabeca_loco=cabeca_loco, acao=acao, feedback=feedback, telemetria=telemetria, telemetria_passos=tele_passos,
                        episodios=np.array(linhas, np.float32).reshape(len(linhas), len(colunas)))
    meta = {"suite": C.VERSAO, "conjunto": conjunto, "sementes": sementes, "cenario": cen.__dict__, "condicao": condicao,
            "mundos": W, "dt": dt, "passos": T, "elos": elos, "clipes": clipes, "acoes": list(b.action_names()),
            "colunas_episodios": colunas, "feedback": list(L.TRACKING_TERMS),
            "info": ["t_clipe", "forca", "controle_locomocao", "pedido_ms", "rumo_rad"],
            "ragdoll": ragdoll["links"], "geo": comum.ragdoll()["links"], "candidato": cand.meta, "segundos_reais": round(time.time() - inicio, 2),
            "corpo": "centro do colisor xyz + quaternio xyzw; o eixo da capsula e o x local do colisor",
            "referencia": "o quadro do clipe seguido no mesmo formato (nan sem clipe)",
            "contato": "o elo com algum ponto de contato no passo (frame_lines, linhas P)",
            "episodio": "quantos episodios do mundo ja terminaram ate este passo (0 = o primeiro, o medido)"}
    (saida / f"{tag}.json").write_text(json.dumps(meta, ensure_ascii=False, default=str))
    log(f"{cand.nome} {tag}: {T} passos x {W} mundos, {len(linhas)} episodios terminados, {time.time() - inicio:.1f} s")
    return path


def capturar_todos(cand: Candidato, conjunto: str, saida: Path, mundos: int = C.MUNDOS, threads: int = 4,
                   so: list[str] | None = None, condicoes: tuple = C.CONDICOES, log=print, pausa_quente=None) -> dict:
    """Todos os cenarios e condicoes de um candidato; devolve {tag: segundos}."""
    tempos = {}
    for cen in C.cenarios() + (C.extras() if so else []):
        if so and cen.nome not in so:
            continue
        for cond in cen.condicoes:
            if cond not in condicoes:
                continue
            if pausa_quente:
                pausa_quente()
            t = time.time()
            rodar(cand, cen, cond, conjunto, saida, mundos, threads, log)
            tempos[f"{cen.nome}__{cond}"] = round(time.time() - t, 2)
    return tempos
