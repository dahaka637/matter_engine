#!/usr/bin/env python3
"""O treino do levantar livre (31/10; vault 04_locomocao/plano_compostos_31_10.md, "O levantar livre"): uma rede so
para levantar de qualquer queda (de brucos, de costas, de lado), pelo AMP (Peng et al. 2021, "AMP: Adversarial Motion
Priors for Stylized Physics-Based Character Control"): PPO com a nota da tarefa (ficar de pe, do lote) mais a do
estilo, de um discriminador que separa as transicoes do corpo das dos clipes (os de levantar de brucos e de costas -
com o rolamento -, os espelhados e o parado). Nada de seguir o clipe quadro a quadro.

Do artigo (secao 6 e o material suplementar): discriminador 1024-512 ReLU por minimos quadrados (alvo +1 nos clipes,
-1 no corpo), penalidade do gradiente nos clipes, nota do estilo max(0, 1 - 0,25 (d - 1)^2), nota total 0,5 tarefa +
0,5 estilo (aqui 0,7 tarefa + 0,3 estilo, 31/10), uma reserva de transicoes recentes do corpo para o discriminador; ator e critico 1024-512.
Declarado: gamma 0,99 (o levantar leva segundos), lambda 0,95; a acao normalizada na faixa de cada junta (o meio
mais a metade vezes a saida, como no ASE), o desvio dela aprendido (comeca em 0,1, preso entre 0,05 e 0,5; --desvio troca); Adam (ator
5e-5, critico 2e-4, discriminador 1e-4); penalidade do gradiente 5 e regularizacao da ultima camada 0,01 (os do ASE).
Num treino novo, a rede comeca imitando a de brucos do DeepMimic (levantar_frente, 100% de brucos; warm_start) e as
primeiras 20 atualizacoes so ensinam o critico e o discriminador.

A nota por passo entra vezes 1 - gamma (o retorno entre 0 e 1).
A medida honesta (o log, linha "aval", e o laboratorio): a rede media em 64 quedas fixas, sem os comecos nos clipes -
de pe no fim, se ficou de pe alguma vez e quando, por lado.

  .venv/bin/python levantar_amp.py --treino levantar_amp --threads 3
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from matter_rl import storage  # noqa: E402
from rastreador_treino import DATA, REPO, RunningNorm, native  # noqa: E402

LOG_STD_MIN, LOG_STD_MAX = math.log(0.05), math.log(0.5)
SIDES = ("brucos", "costas", "lado")
# Os rumos da locomocao (31/10 22h35, os basicos do B21) por exercicio: o rumo pedido (0 a frente, de 45 em 45 graus
# para a esquerda) - as diagonais com os dois lados, como no B21.
HEADINGS = {"frente": (0,), "diagonal_frente": (1, 7), "esquerda": (2,), "diagonal_costas": (3, 5), "tras": (4,),
            "direita": (6,)}


def mlp(inputs: int, outputs: int) -> nn.Sequential:
    return nn.Sequential(nn.Linear(inputs, 1024), nn.ReLU(), nn.Linear(1024, 512), nn.ReLU(), nn.Linear(512, outputs))


class Policy(nn.Module):
    """A acao da rede e normalizada na faixa de cada junta (como no AMP/ASE): o alvo e o meio da faixa mais a metade
    dela vezes a saida - a saida zero e o meio do movimento (nao a pose em T) e o ruido e proporcional a cada junta."""

    def __init__(self, obs: int, actions: int, ranges: np.ndarray):
        super().__init__()
        self.norm = RunningNorm(obs)
        self.actor = mlp(obs, actions)
        self.critic = mlp(obs, 1)
        self.log_std = nn.Parameter(torch.full((actions,), math.log(0.1)))
        low, high = torch.as_tensor(ranges[:, 0]), torch.as_tensor(ranges[:, 1])
        self.register_buffer("action_mid", 0.5 * (low + high))
        self.register_buffer("action_half", torch.clamp(0.5 * (high - low), min=0.05))

    def env_action(self, a: torch.Tensor) -> torch.Tensor:
        return self.action_mid + self.action_half * a

    def distribution(self, obs_n: torch.Tensor) -> torch.distributions.Normal:
        mean = self.actor(obs_n)
        std = self.log_std.clamp(LOG_STD_MIN, LOG_STD_MAX).exp().expand_as(mean)
        return torch.distributions.Normal(mean, std)

    def value(self, obs_n: torch.Tensor) -> torch.Tensor:
        return self.critic(obs_n).squeeze(-1)


class Discriminator(nn.Module):
    """D(s, s'): as duas metades normalizadas pela mesma media (as caracteristicas de um instante)."""

    def __init__(self, size: int):
        super().__init__()
        self.size = size
        self.norm = RunningNorm(size)
        self.net = mlp(2 * size, 1)

    def inputs(self, pairs: torch.Tensor) -> torch.Tensor:
        return torch.cat([self.norm(pairs[:, :self.size]), self.norm(pairs[:, self.size:])], dim=1)

    def forward(self, pairs: torch.Tensor) -> torch.Tensor:
        return self.net(self.inputs(pairs)).squeeze(-1)


def export(model: Policy, path: Path, meta: dict) -> None:
    import struct
    from datetime import datetime, timezone
    floats, offset = [], 0

    def push(array: torch.Tensor) -> int:
        nonlocal offset
        data = array.detach().to("cpu", torch.float32).contiguous().numpy().reshape(-1)
        floats.append(data)
        start = offset
        offset += data.size
        return start

    mean_at = push(model.norm.mean.float())
    inv_at = push(1.0 / torch.sqrt(model.norm.var.float() + 1e-8))
    # (A ultima camada ja sai em radianos: o meio e a metade da faixa de cada junta embutidos nela.)
    linears = [m for m in model.actor if isinstance(m, nn.Linear)]
    layers = []
    for k, m in enumerate(linears):
        weight, bias = m.weight, m.bias
        if k == len(linears) - 1:
            weight = model.action_half[:, None] * weight
            bias = model.action_mid + model.action_half * bias
        layers.append({"inputs": m.in_features, "outputs": m.out_features, "weights": push(weight), "bias": push(bias)})
    header = dict(meta)
    header.update({"schema": "matter-rl-policy-1", "observationSize": layers[0]["inputs"],
                   "actionSize": layers[-1]["outputs"], "activation": "relu",
                   "normalizer": {"mean": mean_at, "invStd": inv_at, "clip": 5.0}, "actionLimit": 3.14159,
                   "layers": layers, "observationLayout": "levantar-v2", "actionLayout": "levantar-v1",
                   "createdUtc": datetime.now(timezone.utc).isoformat(timespec="seconds")})
    text = json.dumps(header, ensure_ascii=False).encode("utf-8")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "wb") as handle:
        handle.write(b"MRLPOL01")
        handle.write(struct.pack("<I", len(text)))
        handle.write(text)
        handle.write(b"\0" * ((-(12 + len(text))) % 4))
        for data in floats:
            handle.write(data.astype("<f4").tobytes())
    tmp.replace(path)


def summarize(rows: np.ndarray, prefix: str) -> dict:
    """As medidas de um conjunto de episodios (as colunas de finished_episodes)."""
    out: dict = {}
    if not len(rows):
        return out
    stood = rows[:, 5] >= 0
    out[f"{prefix}de_pe"] = float(rows[:, 9].mean())
    out[f"{prefix}levantou"] = float(stood.mean())
    out[f"{prefix}tempo"] = float(rows[stood, 5].mean()) if stood.any() else None
    for k, side in enumerate(SIDES):
        sel = rows[:, 2] == k
        out[f"{prefix}{side}"] = float(rows[sel, 9].mean()) if sel.any() else None
        out[f"{prefix}n_{side}"] = int(sel.sum())
    if rows.shape[1] > 10:
        # O levantar v2: chegou ao alvo e a velocidade rumo a ele (os com pedido), caiu depois de ja ter ficado de pe
        # e, no evitar a queda, evitou (nao caiu), se recompos e em quanto tempo.
        moving = rows[:, 10] > 0
        if moving.any():
            out[f"{prefix}chegou"] = float((rows[moving, 11] >= 0).mean())
            out[f"{prefix}vel_rumo"] = float(rows[moving, 12].mean())
        out[f"{prefix}caiu_de_pe"] = float(rows[:, 13].mean())
        out[f"{prefix}sem_cair"] = 1.0 - out[f"{prefix}caiu_de_pe"]
        # Os rumos: por exercicio (o rumo pedido no lado da linha), a velocidade no rumo, ela sobre a pedida (os que
        # comecaram pedindo), o sem cair e o erro do rumo do corpo; e o giro, o sem cair e o erro do rumo.
        loco = rows[:, 1] == 6
        for name, headings in HEADINGS.items():
            sel = loco & np.isin(rows[:, 2], headings)
            if sel.any():
                out[f"{prefix}{name}_vel"] = float(rows[sel, 12].mean())
                out[f"{prefix}{name}_razao"] = float((rows[sel, 12] / np.maximum(rows[sel, 10], 0.1)).mean())
                out[f"{prefix}{name}_sem_cair"] = float(1.0 - rows[sel, 13].mean())
                if rows.shape[1] > 16:
                    out[f"{prefix}{name}_rumo"] = float(rows[sel, 16].mean())
        turn = rows[:, 1] == 7
        if turn.any() and rows.shape[1] > 16:
            out[f"{prefix}rumo"] = float(rows[turn, 16].mean())
        if rows.shape[1] > 15:
            # (A pose de pe no fim, perto do parado em alerta: os que terminaram de pe.)
            up = rows[:, 9] > 0.5
            out[f"{prefix}pose"] = float(rows[up, 15].mean()) if up.any() else None
        avoid = rows[:, 1] == 4
        if avoid.any():
            ok = avoid & (rows[:, 14] >= 0)
            out[f"{prefix}evitou"] = float((rows[avoid, 13] == 0).mean())
            out[f"{prefix}recompos"] = float(ok.sum() / avoid.sum())
            out[f"{prefix}recompos_s"] = float(rows[ok, 14].mean()) if ok.any() else None
    return out


def evaluate(model: Policy, batch, device: torch.device, prefix: str = "") -> dict:
    """A rede media nos comecos fixos da avaliacao (a semente nao muda): um episodio de cada mundo."""
    batch.reset(12345)
    n = batch.num_envs
    done: dict[int, list] = {}
    for _ in range(int(batch.episode_seconds / batch.policy_dt) + 2):
        with torch.no_grad():
            obs = torch.as_tensor(batch.observations(), device=device)
            action = model.env_action(model.actor(model.norm(obs))).cpu().numpy().astype(np.float32)
        batch.step(action)
        for row in batch.finished_episodes().tolist():
            done.setdefault(int(row[0]), row)
        if len(done) >= n:
            break
    return summarize(np.array(list(done.values())), prefix)


def expand_inputs(state: dict, observations: int) -> dict:
    """A rede de outro treino do levantar numa entrada maior (o v2: o comando depois do corpo): as colunas novas da
    primeira camada do ator e do critico em zero (a rede faz o mesmo de antes) e a normalizacao delas neutra."""
    out = dict(state)
    for key in ("actor.0.weight", "critic.0.weight"):
        w = out[key]
        if observations > w.shape[1]:
            out[key] = torch.cat([w, torch.zeros(w.shape[0], observations - w.shape[1], dtype=w.dtype, device=w.device)], 1)
    for key, fill in (("norm.mean", 0.0), ("norm.var", 1.0)):
        v = out[key]
        if observations > v.shape[0]:
            out[key] = torch.cat([v, torch.full((observations - v.shape[0],), fill, dtype=v.dtype, device=v.device)])
    return out


def warm_start(model: Policy, threads: int, steps: int, device: torch.device) -> dict:
    """O comeco pela rede de brucos do DeepMimic (levantar_frente, que levanta 100% de brucos): a rede nova (sem a fase
    do clipe) aprende a acao dela nos estados dela, com o ruido do DART (Laskey et al. 2017: o professor age com um pouco
    de ruido, a aluna ve estados um pouco fora do caminho dele). O treino parte sabendo levantar de brucos."""
    from levantar_treino import GetUpPolicy as Teacher
    state = torch.load(DATA / "levantar_frente" / "ultimo.pt", map_location=device, weights_only=False)
    source = native.GetUpBatch(str(REPO / "assets"), clip="frente", environments=256, threads=threads, seed=7)
    teacher = Teacher(source.obs_size, source.action_size).to(device)
    teacher.load_state_dict(state["model"])
    teacher.eval()
    rows_x, rows_y = [], []
    with torch.no_grad():
        for _ in range(steps):
            obs = torch.as_tensor(source.observations(), device=device)
            act = teacher.actor(teacher.norm(obs))
            rows_x.append(obs[:, :-1].clone())
            rows_y.append((act - model.action_mid) / model.action_half)
            source.step((act + 0.05 * torch.randn_like(act)).cpu().numpy().astype(np.float32))
            source.finished_episodes()
    x, y = torch.cat(rows_x), torch.cat(rows_y)
    for chunk in x.split(16384):
        model.norm.update(chunk)
    optimizer = torch.optim.Adam(model.actor.parameters(), lr=3e-4)
    loss = torch.zeros(())
    for _ in range(30):
        for idx in torch.randperm(len(x), device=device).split(4096):
            loss = (model.actor(model.norm(x[idx])) - y[idx]).pow(2).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
    # A conferencia: a aluna no lote do professor, comecando deitada de brucos (o inicio do clipe).
    check = native.GetUpBatch(str(REPO / "assets"), clip="frente", environments=64, threads=threads, seed=9,
                              evaluation=True)
    ended: dict[int, list] = {}
    for _ in range(int(check.clip_seconds / check.policy_dt) + 2):
        with torch.no_grad():
            obs = torch.as_tensor(check.observations(), device=device)[:, :-1]
            action = model.env_action(model.actor(model.norm(obs))).cpu().numpy().astype(np.float32)
        check.step(action)
        for row in check.finished_episodes().tolist():
            ended.setdefault(int(row[0]), row)
    standing = float(np.mean([row[6] for row in ended.values()])) if ended else 0.0
    return {"aprendeu": True, "amostras_professor": int(len(x)), "perda": float(loss.item()), "de_pe_brucos": standing}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--treino", required=True)
    parser.add_argument("--mundos", type=int, default=512)
    parser.add_argument("--passos", type=int, default=32)
    parser.add_argument("--threads", type=int, default=3)
    parser.add_argument("--semente", type=int, default=1)
    parser.add_argument("--atualizacoes", type=int, default=0, help="0: ate parar")
    parser.add_argument("--salvar-cada", type=int, default=50)
    parser.add_argument("--epocas", type=int, default=4)
    parser.add_argument("--minilotes", type=int, default=4)
    parser.add_argument("--taxa-ator", type=float, default=5e-5)
    parser.add_argument("--taxa-critico", type=float, default=2e-4)
    parser.add_argument("--taxa-disc", type=float, default=1e-4)
    # (31/10, ordem dele: o RL com liberdade para criar os proprios movimentos de levantar - a tarefa pesa mais que o
    # estilo, que so puxa para ficar parecido com os clipes.)
    parser.add_argument("--peso-tarefa", type=float, default=0.7)
    parser.add_argument("--peso-estilo", type=float, default=0.3)
    # (31/10 21h35, ele: "humanizar e suavizar um pouco os movimentos do levantar-se... bem funcionais, mas nitidamente
    # artificiais e recontorcidos": o custo do tranco - a diferenca da acao (na faixa das juntas) de uma decisao para a
    # seguinte, ao quadrado, media nos eixos - vezes este peso, na mesma escala da tarefa.)
    parser.add_argument("--suave", type=float, default=0.0, help="o peso do custo do tranco da acao")
    parser.add_argument("--desvio", type=float, default=None,
                        help="o desvio da acao (na faixa das juntas) a partir de agora, tambem ao retomar")
    parser.add_argument("--comeco-clipe", type=float, default=0.25, help="a fatia dos comecos nos clipes de levantar")
    parser.add_argument("--comeco-de-pe", type=float, default=0.15, help="a fatia que comeca de pe (o clipe parado)")
    parser.add_argument("--professor-passos", type=int, default=800,
                        help="num treino novo: os passos da rede de brucos (256 mundos cada) para a aluna aprender")
    parser.add_argument("--aquecer-critico", type=int, default=20,
                        help="as primeiras atualizacoes so do critico e do discriminador (a acao aprendida fica)")
    # O levantar v2 (31/10 17h, ele): levantar ja indo para um alvo e evitar a queda depois de um empurrao forte.
    parser.add_argument("--de", default="", help="num treino novo: comecar da rede de outro treino do levantar")
    parser.add_argument("--movimento", type=float, default=0.0, help="a fatia que levanta ja indo para um alvo")
    parser.add_argument("--evitar", type=float, default=0.0, help="a fatia do evitar a queda (de pe, o empurrao forte)")
    parser.add_argument("--evitar-forca", type=float, nargs=2, default=[1200.0, 1800.0], help="o empurrao curto (N)")
    parser.add_argument("--empurrao-forca", type=float, nargs=2, default=[400.0, 900.0], help="o de pe (N)")
    parser.add_argument("--velocidade", type=float, nargs=2, default=[2.5, 5.0], help="a pedida no movimento (m/s)")
    parser.add_argument("--empurrao-continuo", type=float, default=0.0,
                        help="a fatia continua dos empurroes de pe (o autoequilibrio parado)")
    parser.add_argument("--empurrao-continuo-forca", type=float, nargs=2, default=[150.0, 350.0], help="(N, por 1-3 s)")
    # (31/10 22h35, ele: so esta rede daqui em diante, com os basicos do B21 - os rumos e o giro parado suave -, o
    # autoequilibrio o principal: "se locomover resistindo a impactos e adversidades".)
    parser.add_argument("--locomocao", type=float, default=0.0,
                        help="a fatia dos rumos (andar ou trotar num dos 8 rumos, mantendo o rumo do corpo)")
    parser.add_argument("--giro", type=float, default=0.0, help="a fatia do giro parado suave")
    parser.add_argument("--andar", type=float, default=1.0 / 3.0, help="nos rumos, a fatia do andar (o resto trota)")
    parser.add_argument("--andar-velocidade", type=float, nargs=2, default=[0.8, 1.4], help="(m/s)")
    parser.add_argument("--trote-velocidade", type=float, nargs=2, default=[2.0, 3.0], help="(m/s)")
    parser.add_argument("--empurrao-mov-forca", type=float, nargs=2, default=[300.0, 600.0],
                        help="os empurroes curtos em movimento (N; os rumos e o giro)")
    parser.add_argument("--empurrao-mov-continuo", type=float, default=0.3, help="a fatia continua deles")
    parser.add_argument("--empurrao-mov-continuo-forca", type=float, nargs=2, default=[100.0, 180.0],
                        help="(N, por 1-2 s)")
    parser.add_argument("--arrancada", type=float, default=0.0,
                        help="a fatia que comeca na arrancada agachada (crouched_to_sprint) virado para um alvo")
    args = parser.parse_args()

    from matter_rl.system import require_watchdog
    require_watchdog()
    torch.set_num_threads(1)

    stop = {"pedido": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(pedido=True))

    run = DATA / args.treino
    run.mkdir(parents=True, exist_ok=True)
    for name, data in (("run.json", {"kind": "levantar", "task": "levantar_livre", "de": args.de}),
                       ("tarefa.json", {"schema": "matter-getup-amp-1", "id": "levantar_livre"})):
        if not (run / name).exists():
            (run / name).write_text(json.dumps(data, ensure_ascii=False))
    # (A configuracao do v2 de agora no run.json, a cada abertura: o laboratorio mostra o levantar em movimento e o
    # evitar a queda com as mesmas forcas e velocidades do treino.)
    info = json.loads((run / "run.json").read_text())
    info["v2"] = {"movimento": args.movimento, "evitar": args.evitar, "arrancada": args.arrancada,
                  "peso_tarefa": args.peso_tarefa, "peso_estilo": args.peso_estilo, "suave": args.suave,
                  "locomocao": args.locomocao, "giro": args.giro, "andar": args.andar,
                  "andar_velocidade": list(args.andar_velocidade), "trote_velocidade": list(args.trote_velocidade),
                  "empurrao_mov_forca": list(args.empurrao_mov_forca), "empurrao_mov_continuo": args.empurrao_mov_continuo,
                  "empurrao_mov_continuo_forca": list(args.empurrao_mov_continuo_forca),
                  "empurrao_continuo": args.empurrao_continuo, "empurrao_continuo_forca": list(args.empurrao_continuo_forca),
                  "evitar_forca": list(args.evitar_forca),
                  "empurrao_forca": list(args.empurrao_forca), "velocidade": list(args.velocidade)}
    (run / "run.json").write_text(json.dumps(info, ensure_ascii=False))
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(args.semente)
    v2 = dict(avoid_newtons=tuple(args.evitar_forca), push_newtons=tuple(args.empurrao_forca),
              move_speed=tuple(args.velocidade), push_hold_fraction=args.empurrao_continuo,
              push_hold_newtons=tuple(args.empurrao_continuo_forca), walk_fraction=args.andar,
              walk_speed=tuple(args.andar_velocidade), trot_speed=tuple(args.trote_velocidade),
              moving_push_newtons=tuple(args.empurrao_mov_forca), moving_push_hold_fraction=args.empurrao_mov_continuo,
              moving_push_hold_newtons=tuple(args.empurrao_mov_continuo_forca))
    batch = native.GetUpAmpBatch(str(REPO / "assets"), environments=args.mundos, threads=args.threads,
                                 seed=args.semente, clip_start_fraction=args.comeco_clipe,
                                 standing_start_fraction=args.comeco_de_pe, moving_fraction=args.movimento,
                                 avoid_fraction=args.evitar, sprint_start_fraction=args.arrancada,
                                 locomotion_fraction=args.locomocao, turn_fraction=args.giro, **v2)
    evaluation = native.GetUpAmpBatch(str(REPO / "assets"), environments=64, threads=1, seed=12345, evaluation=True)
    # (O v2: 32 quedas fixas com alvo e 32 empurroes fortes fixos.)
    evaluations = {"mov_": native.GetUpAmpBatch(str(REPO / "assets"), environments=32, threads=1, seed=777,
                                                evaluation=True, evaluation_kind=1, **v2)} if args.movimento > 0 else {}
    if args.evitar > 0:
        evaluations["evitar_"] = native.GetUpAmpBatch(str(REPO / "assets"), environments=32, threads=1, seed=999,
                                                      evaluation=True, evaluation_kind=2, **v2)
    # (O equilibrio parado: 32 de pe, um empurrao cada com a regua do parado com empurrao forte do B21.)
    evaluations["equil_"] = native.GetUpAmpBatch(str(REPO / "assets"), environments=32, threads=1, seed=555,
                                                 evaluation=True, evaluation_kind=3, **v2)
    # (A locomocao, com a regua fixa - nao a do treino, que sobe: os rumos a 2,5 m/s e os empurroes em movimento do
    # comeco do B21, um por episodio. 32 nos rumos, 4 por rumo; 8 no giro, metade para cada lado; 8 parados, sem
    # empurrao.)
    if args.locomocao > 0 or args.giro > 0:
        ruler = dict(v2, trot_speed=(2.5, 2.5), moving_push_newtons=(300.0, 600.0), moving_push_hold_fraction=0.3,
                     moving_push_hold_newtons=(100.0, 180.0))
        evaluations["loco_"] = native.GetUpAmpBatch(str(REPO / "assets"), environments=32, threads=1, seed=444,
                                                    evaluation=True, evaluation_kind=4, **ruler)
        evaluations["giro_"] = native.GetUpAmpBatch(str(REPO / "assets"), environments=8, threads=1, seed=333,
                                                    evaluation=True, evaluation_kind=6, **ruler)
        evaluations["parado_"] = native.GetUpAmpBatch(str(REPO / "assets"), environments=8, threads=1, seed=222,
                                                      evaluation=True, evaluation_kind=5, **ruler)
    n, obs_size, act_size, amp = batch.num_envs, batch.obs_size, batch.action_size, batch.amp_size
    reference = torch.as_tensor(batch.reference_pairs(), device=device)
    model = Policy(obs_size, act_size, batch.action_ranges()).to(device)
    disc = Discriminator(amp).to(device)
    actor_opt = torch.optim.Adam(list(model.actor.parameters()) + [model.log_std], lr=args.taxa_ator)
    critic_opt = torch.optim.Adam(model.critic.parameters(), lr=args.taxa_critico)
    disc_opt = torch.optim.Adam(disc.net.parameters(), lr=args.taxa_disc, weight_decay=1e-4)
    counters = {"atualizacao": 0, "amostras": 0, "episodios": 0}
    settings = {"mundos": n, "passos": args.passos, "peso_tarefa": args.peso_tarefa, "peso_estilo": args.peso_estilo,
                "comeco_clipe": args.comeco_clipe, "observacao": batch.observation_names(), "acao": batch.action_names(),
                "pares_dos_clipes": int(reference.shape[0])}
    checkpoint = run / "ultimo.pt"
    if checkpoint.exists():
        state = torch.load(checkpoint, map_location=device, weights_only=False)
        grown = state["model"]["actor.0.weight"].shape[1] < obs_size
        # (Uma entrada nova no contrato - 31/10 22h15, o rumo pedido do corpo: as colunas novas em zero e o Adam do ator e
        # do critico do zero, as formas mudaram.)
        model.load_state_dict(expand_inputs(state["model"], obs_size) if grown else state["model"])
        disc.load_state_dict(state["disc"])
        if not grown:
            actor_opt.load_state_dict(state["actor_opt"])
            critic_opt.load_state_dict(state["critic_opt"])
        disc_opt.load_state_dict(state["disc_opt"])
        counters = state["counters"]
    log = open(run / "log.jsonl", "a")
    gamma, lam, clip, T = 0.99, 0.95, 0.2, args.passos
    minibatch = (T * n) // max(1, args.minilotes)
    replay = torch.zeros(0, 2 * amp, device=device)
    obs = torch.as_tensor(batch.observations(), device=device)
    window: list = []
    # A acao anterior de cada mundo (o tranco) e se ela vale (nao no primeiro passo de um episodio).
    previous_action = torch.zeros(n, act_size, device=device)
    previous_valid = torch.zeros(n, dtype=torch.bool, device=device)
    with torch.no_grad():
        disc.norm.update(reference[:, :amp])

    def export_policy() -> None:
        export(model, run / "exports" / "policy_latest.mrlpolicy",
               {"run": args.treino, "iteration": counters["atualizacao"], "steps": counters["amostras"],
                "observationNames": settings["observacao"]})

    def save() -> None:
        nonlocal log
        tmp = run / "ultimo.tmp"
        torch.save({"model": model.state_dict(), "disc": disc.state_dict(), "actor_opt": actor_opt.state_dict(),
                    "critic_opt": critic_opt.state_dict(), "disc_opt": disc_opt.state_dict(), "counters": counters,
                    "settings": settings}, tmp)
        tmp.replace(checkpoint)
        export_policy()
        # (As estatisticas do log condensadas por idade, como as do banco.)
        log.flush()
        if storage.condense_log(run / "log.jsonl"):
            log.close()
            log = open(run / "log.jsonl", "a")

    if args.desvio is not None:
        with torch.no_grad():
            model.log_std.fill_(math.log(args.desvio))
    if not checkpoint.exists() and args.de:
        source = torch.load(DATA / args.de / "ultimo.pt", map_location=device, weights_only=False)
        model.load_state_dict(expand_inputs(source["model"], obs_size))
        disc.load_state_dict(source["disc"])
        row = {"atualizacao": 0, "de": args.de, "de_atualizacao": source["counters"]["atualizacao"]}
        log.write(json.dumps(row) + "\n")
        log.flush()
        print(f"comeca da rede de {args.de} (atualizacao {row['de_atualizacao']})", flush=True)
        export_policy()
    elif not checkpoint.exists() and args.professor_passos > 0:
        row = {"atualizacao": 0, **warm_start(model, args.threads, args.professor_passos, device)}
        log.write(json.dumps(row) + "\n")
        log.flush()
        print("aprendeu com a rede de brucos: " + " ".join(f"{k}={v}" for k, v in row.items()), flush=True)
        export_policy()

    while (args.atualizacoes <= 0 or counters["atualizacao"] < args.atualizacoes) and not stop["pedido"]:
        begin = time.time()
        buf = {k: torch.zeros(T, n, *s, device=device) for k, s in
               (("obs", (obs_size,)), ("act", (act_size,)), ("logp", ()), ("val", ()), ("task", ()), ("boot", ()),
                ("done", ()), ("pairs", (2 * amp,)), ("jerk", ()))}
        with torch.no_grad():
            for t in range(T):
                model.norm.update(obs)
                obs_n = model.norm(obs)
                dist = model.distribution(obs_n)
                action = dist.sample()
                buf["obs"][t], buf["act"][t] = obs, action
                buf["jerk"][t] = torch.where(previous_valid, (action - previous_action).pow(2).mean(-1),
                                             torch.zeros_like(previous_valid, dtype=torch.float32))
                buf["logp"][t] = dist.log_prob(action).sum(-1)
                buf["val"][t] = model.value(obs_n)
                task, term, trunc = batch.step(model.env_action(action).cpu().numpy().astype(np.float32))
                buf["task"][t] = torch.as_tensor(task, device=device)
                buf["pairs"][t] = torch.as_tensor(batch.amp_pairs(), device=device)
                trunc = torch.as_tensor(trunc, device=device).float()
                if trunc.any():
                    final = torch.as_tensor(batch.final_observations(), device=device)
                    buf["boot"][t] = trunc * model.value(model.norm(final))
                buf["done"][t] = torch.maximum(torch.as_tensor(term, device=device).float(), trunc)
                previous_action = action
                previous_valid = buf["done"][t] < 0.5
                obs = torch.as_tensor(batch.observations(), device=device)
            last_value = model.value(model.norm(obs))
            pairs = buf["pairs"].reshape(T * n, 2 * amp)
            disc.norm.update(torch.cat([pairs[:, :amp], reference[:, :amp]]))
            d = disc(pairs).reshape(T, n)
            style = torch.clamp(1.0 - 0.25 * (d - 1.0).pow(2), min=0.0)
            # (A nota por passo vezes 1 - gamma: o retorno fica entre 0 e 1 e o critico nao precisa correr atras da
            # escala de ~100 de um gamma 0,99.)
            rew = (1.0 - gamma) * (args.peso_tarefa * buf["task"] + args.peso_estilo * style - args.suave * buf["jerk"]) \
                + gamma * buf["boot"]
        adv = torch.zeros(T, n, device=device)
        gae = torch.zeros(n, device=device)
        for t in reversed(range(T)):
            next_value = last_value if t == T - 1 else buf["val"][t + 1]
            nonterminal = 1.0 - buf["done"][t]
            delta = rew[t] + gamma * next_value * nonterminal - buf["val"][t]
            gae = delta + gamma * lam * nonterminal * gae
            adv[t] = gae
        ret = adv + buf["val"]
        flat = lambda x: x.reshape(T * n, *x.shape[2:])  # noqa: E731
        f_obs, f_act, f_logp, f_adv, f_ret = flat(buf["obs"]), flat(buf["act"]), flat(buf["logp"]), flat(adv), flat(ret)
        f_adv = (f_adv - f_adv.mean()) / (f_adv.std() + 1e-8)
        replay = torch.cat([replay, pairs])[-100_000:]
        stats = {"pol": 0.0, "vf": 0.0, "clip": 0.0, "disc": 0.0, "d_clipe": 0.0, "d_corpo": 0.0, "gp": 0.0}
        steps = 0
        for _ in range(args.epocas):
            for idx in torch.randperm(T * n, device=device).split(minibatch):
                obs_n = model.norm(f_obs[idx])
                vf_loss = (model.value(obs_n) - f_ret[idx]).pow(2).mean()
                critic_opt.zero_grad()
                vf_loss.backward()
                nn.utils.clip_grad_norm_(model.critic.parameters(), 1.0)
                critic_opt.step()
                logp = model.distribution(obs_n).log_prob(f_act[idx]).sum(-1)
                ratio = torch.exp(logp - f_logp[idx])
                pol_loss = -torch.min(ratio * f_adv[idx], ratio.clamp(1 - clip, 1 + clip) * f_adv[idx]).mean()
                if counters["atualizacao"] >= args.aquecer_critico:
                    actor_opt.zero_grad()
                    pol_loss.backward()
                    nn.utils.clip_grad_norm_(list(model.actor.parameters()) + [model.log_std], 1.0)
                    actor_opt.step()
                # O discriminador: metade do lote de agora e metade da reserva, contra os clipes (com reposicao).
                half = len(idx) // 2
                body = torch.cat([pairs[idx[:half]], replay[torch.randint(len(replay), (half,), device=device)]])
                clips = reference[torch.randint(len(reference), (len(body),), device=device)]
                x_clip = disc.inputs(clips).detach().requires_grad_(True)
                d_clip = disc.net(x_clip).squeeze(-1)
                d_body = disc.net(disc.inputs(body)).squeeze(-1)
                grad = torch.autograd.grad(d_clip.sum(), x_clip, create_graph=True)[0]
                gp = grad.pow(2).sum(-1).mean()
                last = disc.net[-1].weight
                disc_loss = 0.5 * ((d_clip - 1.0).pow(2).mean() + (d_body + 1.0).pow(2).mean()) + 5.0 * gp \
                    + 0.01 * last.pow(2).sum()
                disc_opt.zero_grad()
                disc_loss.backward()
                disc_opt.step()
                stats["pol"] += pol_loss.item()
                stats["vf"] += vf_loss.item()
                stats["clip"] += ((ratio - 1).abs() > clip).float().mean().item()
                stats["disc"] += disc_loss.item()
                stats["d_clipe"] += d_clip.mean().item()
                stats["d_corpo"] += d_body.mean().item()
                stats["gp"] += gp.item()
                steps += 1
        episodes = batch.finished_episodes()
        if len(episodes):
            window.extend(episodes.tolist())
            window = window[-3000:]
        counters["atualizacao"] += 1
        counters["amostras"] += T * n
        counters["episodios"] += len(episodes)
        if counters["atualizacao"] % 20 == 0:
            recent = np.array(window) if window else np.zeros((0, 17))
            row = {"atualizacao": counters["atualizacao"], "amostras": counters["amostras"], "hora": time.time(),
                   "tarefa": float(buf["task"].mean()), "estilo": float(style.mean()), "tranco": float(buf["jerk"].mean()),
                   "desvio": float(model.log_std.detach().clamp(LOG_STD_MIN, LOG_STD_MAX).exp().mean()),
                   "am_s": T * n / (time.time() - begin), **{k: v / max(1, steps) for k, v in stats.items()}}
            row.update(summarize(recent[recent[:, 1] == 0] if len(recent) else recent, "queda_"))
            row.update(summarize(recent[recent[:, 1] == 1] if len(recent) else recent, "clipe_"))
            row.update(summarize(recent[recent[:, 1] == 2] if len(recent) else recent, "de_pe_"))
            row.update(summarize(recent[recent[:, 1] == 3] if len(recent) else recent, "mov_"))
            row.update(summarize(recent[recent[:, 1] == 4] if len(recent) else recent, "evitar_"))
            row.update(summarize(recent[recent[:, 1] == 5] if len(recent) else recent, "arrancada_"))
            row.update(summarize(recent[recent[:, 1] == 6] if len(recent) else recent, "loco_"))
            row.update(summarize(recent[recent[:, 1] == 7] if len(recent) else recent, "giro_"))
            log.write(json.dumps(row) + "\n")
            log.flush()
            print(" ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in row.items()), flush=True)
        if counters["atualizacao"] % 10 == 0:
            export_policy()
        if counters["atualizacao"] % args.salvar_cada == 0:
            save()
            row = {"atualizacao": counters["atualizacao"], "amostras": counters["amostras"], "aval": True,
                   "hora": time.time(),
                   **evaluate(model, evaluation, device)}
            for prefix, other in evaluations.items():
                row.update(evaluate(model, other, device, prefix))
            log.write(json.dumps(row) + "\n")
            log.flush()
            print("aval " + " ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}" for k, v in row.items()),
                  flush=True)
    save()
    print(f"salvo na atualizacao {counters['atualizacao']}" + (" (sinal de parar)" if stop["pedido"] else ""), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
