"""As sondas da rl_final (04/11): medem a rede de um checkpoint fora do treino, deterministica, para o monitor achar as falhas
do catalogo (vault 04_locomocao/catalogo_de_falhas_rl.md). Leves (4 threads), mas pesam: uma de cada vez ao lado do treino.

  MATTER_TRACKING_NATIVE=build-rl-n1/python tools/rl/.venv/bin/python tools/rl/sondas_rl_final.py levantar
  ... sondas_rl_final.py evitar --forca-empurrao 1400 2200
  ... sondas_rl_final.py parado --forcas 0.3 0.17 1.0 --segundos 30
  ... sondas_rl_final.py velocidade              (o % do pedido por exercicio, na forca de cada um)
  ... sondas_rl_final.py partes --forcas 1.0 0   (por parte, a rede contra o clipe puro, nos rumos)
  --checkpoint outro.pt compara com um arquivado (ex.: rl_final_clipe/arquivo_*/ultimo.pt).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import levantar_amp as L  # noqa: E402

# (04/11 09h50: o torch usava todos os nucleos - a destilacao levou a CPU a 83,6 graus. As sondas com 2.)
torch.set_num_threads(2)

ASSETS = str(L.REPO / "assets")
RUNS = Path.home() / ".local/share/matter-engine/rl/runs"
TRACK = dict(separate_skills=True, locomotion_tracking=True, locomotion_residual=True, locomotion_termination=True,
             locomotion_seconds=8.0, moving_push_newtons=(0.0, 0.0), moving_push_hold_newtons=(0.0, 0.0))


IDLE_REACH = [0.2]
REWARD = [10]
HANDOFF = [0.6, -1.0]
# (04/11 12h40: o puxao de lado, as pernas da corrida de lado e os pedidos fixos do treino, do run.json dele.)
RUN_V2: dict = {}


def setup(b, levels, idle=True):
    """O ambiente como o treino de agora (a nota v7, o idle na locomocao, as maos)."""
    b.set_tracking_reward(REWARD[0])
    b.set_idle_imitation(2.0)
    b.set_idle(idle, idle, IDLE_REACH[0])
    b.set_idle_handoff(*HANDOFF)
    b.set_upper_hand(True)
    b.set_body_hand(True)
    b.set_drill_assist(levels)
    L.run_tracking_settings(b, RUN_V2)


def policy(b, state, residual, parts):
    model = L.Policy(b.observations().shape[1], b.action_size, b.action_ranges(), split=True)
    model.load_state_dict(state["model"])
    model.residual = residual
    spine, arms, head = parts
    model.joint_names = tuple(n for n in b.action_names() if not n.startswith("passo."))
    L.configure_threat(model, b, float(RUN_V2.get("autoridade_ameaca", 1.0)), float(RUN_V2.get("autoridade_demanda", 1.0)))
    model.residual_joint = torch.as_tensor([
        1.0 if n.startswith("passo.") else spine if n.split(".")[0] in ("Abdomen", "Chest", "UpperChest") else arms if ("Arm" in n or "Hand" in n)
        else head if n.split(".")[0] in ("Neck", "Head") else residual for n in b.action_names()])
    model.eval()

    def act(zero=False):
        if zero:
            return np.zeros((b.observations().shape[0], b.action_size), np.float32)
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            return model.env_action(model.actor(model.norm(obs)), obs).numpy().astype(np.float32)
    return act


def episodes(b, act, seconds, count, zero=False):
    rows = []
    for _ in range(int(seconds / b.policy_dt) + 2):
        b.step(act(zero))
        rows += b.finished_episodes().tolist()
        if len(rows) >= count:
            break
    return np.array(rows[:count]) if rows else np.zeros((0, 35))


def levantar(a, state, levels):
    """O levantar no chao liso: levantou (ficou de pe alguma vez), o tempo e de pe no fim."""
    b = L.get_up_batch(ASSETS, environments=a.mundos, threads=4, seed=7, clip_start_fraction=0.0, standing_start_fraction=0.0,
                               moving_fraction=0.0, avoid_fraction=0.0, locomotion_fraction=0.0, turn_fraction=0.0, **TRACK)
    setup(b, [a.forcas[0]] * 25)
    b.reset(7)
    ep = episodes(b, policy(b, state, a.residuo, a.partes), 9.0, a.mundos)
    stood = ep[:, 5] >= 0
    print(f"levantar: {len(ep)} quedas, levantou {stood.mean():.0%} em {ep[stood, 5].mean() if stood.any() else float('nan'):.2f} s,"
          f" de pe no fim {ep[:, 9].mean():.0%}")


def evitar(a, state, levels):
    """O evitar a queda (o empurrao de pe): evitou (o tronco nunca no chao) e de pe no fim, com e sem o idle na locomocao."""
    for idle in (False, True):
        b = L.get_up_batch(ASSETS, environments=24, threads=4, seed=11, clip_start_fraction=0.0, standing_start_fraction=0.0,
                                   moving_fraction=0.0, avoid_fraction=1.0, locomotion_fraction=0.0, turn_fraction=0.0,
                                   **{**TRACK, "avoid_newtons": tuple(a.forca_empurrao)})
        setup(b, [0.0] * len(L.DRILL_KEYS), idle)
        b.reset(11)
        ep = episodes(b, policy(b, state, a.residuo, a.partes), 9.0, 24)
        av = ep[ep[:, 1] == 4]
        print(f"evitar (idle na locomocao {idle}): {len(av)} empurroes, evitou {(av[:, 21] == 0).mean():.0%}, de pe no fim {av[:, 9].mean():.0%}")


def parado(a, state, levels):
    """O parado em alerta por muito mais que os 6 s do treino: os pes contra o idle, a pelve, a semelhanca e quem controla."""
    for level in a.forcas:
        b = L.get_up_batch(ASSETS, environments=16, threads=4, seed=5, clip_start_fraction=0.0, standing_start_fraction=1.0,
                                   moving_fraction=0.0, avoid_fraction=0.0, locomotion_fraction=0.0, turn_fraction=0.0,
                                   episode_seconds=a.segundos, **{k: v for k, v in TRACK.items() if not k.startswith("moving")})
        setup(b, [level] * len(L.DRILL_KEYS))
        b.reset(5)
        act = policy(b, state, a.residuo, a.partes)
        per, ends = {}, []
        for i in range(int(a.segundos / b.policy_dt)):
            b.step(act())
            ends += [(round(r[4], 1), int(r[32])) for r in b.finished_episodes().tolist()]
            f = np.array(b.locomotion_feedback())
            per.setdefault(int(i * b.policy_dt), []).append(
                (np.sqrt(np.maximum(f[:, 5], 0)), f[:, 6], f[:, 9], np.array(b.observations())[:, -1] > .5))
        print(f"parado com forca {level:.0%}: encerrados {len(ends)} {sorted(ends)[:6]}")
        print("  seg | pes fora do idle (cm, max) | pelve vs idle (cm) | semelhanca | na locomocao")
        for s in sorted(per)[::max(1, len(per) // 10)]:
            feet, pelvis, match, loco = (np.mean([p[k] for p in per[s]], axis=0) for k in range(4))
            print(f"  {s:3d} | {100 * feet.mean():5.1f} ({100 * feet.max():5.1f}) | {100 * pelvis.mean():6.1f} | {match.mean():.2f} | {loco.mean():.0%}")


CASES = [(f"lev_trote_{h}", dict(evaluation_band=1, heading=h), 0) for h in range(8)] \
    + [(f"lev_corrida_{h}", dict(evaluation_band=2, heading=h), 0) for h in range(8)] \
    + [(k, dict(evaluation_band=1), m + 1) for m, k in enumerate(L.MANEUVERS)] \
    + [(k, dict(evaluation_band=2), m + 5) for m, k in enumerate(L.RUN_CURVES)]


def velocidade(a, state, levels):
    """O % do pedido (a velocidade no rumo sobre a que a referencia pede nos mesmos instantes) por exercicio, na forca dele."""
    for key, kw, man in CASES:
        b = L.get_up_batch(ASSETS, environments=8, threads=4, seed=21, evaluation=True, evaluation_kind=4, **kw, **TRACK)
        setup(b, levels)
        if man:
            b.set_maneuvers(0.0, man)
        b.reset(21)
        ep = episodes(b, policy(b, state, a.residuo, a.partes), 8.0, 8)
        print(f"{key[4:]:14s} {np.mean(ep[:, 12] / np.maximum(ep[:, 10], 0.1)):4.0%} do pedido (pede {ep[:, 10].mean():.2f},"
              f" faz {ep[:, 12].mean():.2f} m/s), forca {levels[L.DRILL_KEYS.index(key)]:.0%}, ate o fim {np.mean(ep[:, 32] == 0):.0%}")


def partes(a, state, levels):
    """Por parte, a rede contra o clipe puro (acao zero), na mesma forca: onde a correcao da rede estraga o clipe."""
    keys = ("r_semelhanca", "pernas", "pes", "coluna_graus", "bracos_graus", "vel_rumo", "r_estabilidade", "altura_pelve",
            "pulo_pelve")
    T = L.TRACKING_TERMS
    for key, kw, man in CASES[:16]:
        if int(key[-1]) not in (a.rumos or ((0, 2, 4) if not a.todos else range(8))):
            continue
        for level in a.forcas:
            for who in ("clipe", "rede"):
                b = L.get_up_batch(ASSETS, environments=8, threads=4, seed=58, evaluation=True, evaluation_kind=4, **kw, **TRACK)
                setup(b, [level] * len(L.DRILL_KEYS))
                b.reset(58)
                act = policy(b, state, a.residuo, a.partes)
                acc = []
                for i in range(int(8.0 / b.policy_dt)):
                    b.step(act(who == "clipe"))
                    if i > 15:
                        acc.append(np.array(b.locomotion_feedback()))
                f = np.concatenate(acc)
                f = f[f[:, 0] > 0]
                print(f"{key[4:]:12s} {level:4.0%} {who:5s} " + " ".join(f"{k[:8]} {f[:, T.index(k)].mean():6.2f}" for k in keys)
                      + f" p90 altura {np.percentile(f[:, T.index('altura_pelve')], 90):.3f} pulo {np.percentile(f[:, T.index('pulo_pelve')], 90):.2f}",
                      flush=True)


def largada(a, state, levels):
    """A largada de costas (04/11 13h40, ele: "arranca com a coluna para frente ao inves de para tras"): o peito e a cabeca
    contra a pelve no rumo do deslocamento (graus, + para tras, o rumo), a velocidade no rumo e a distancia em 3 s, a rede
    na forca dela e a 0%. O rumo de cada mundo: o deslocamento do clipe puro com a mao cheia, na mesma semente (as coxas
    nao servem - o parado em alerta tem os pes desencontrados)."""
    def lines(b):
        out = []
        for w in range(b.num_envs):
            f = {p[1]: np.array([float(x) for x in p[2:5]]) for p in (l.split() for l in b.frame_lines(w).splitlines())
                 if p and p[0] in ("B", "C")}
            pelvis = next(v for k, v in f.items() if "Pelvis" in k or "Hips" in k)
            out.append((pelvis, next(v for k, v in f.items() if "UpperChest" in k), next(v for k, v in f.items() if k.startswith("Head"))))
        return out

    def run(kw, level, zero):
        b = L.get_up_batch(ASSETS, environments=8, threads=4, seed=58, evaluation=True, evaluation_kind=4, **kw, **TRACK)
        setup(b, [level] * len(L.DRILL_KEYS))
        b.reset(58)
        act = policy(b, state, a.residuo, a.partes)
        track, speed = [], []
        for _ in range(int(3.0 / b.policy_dt)):
            track.append(lines(b))
            b.step(act(zero))
            speed.append(np.asarray(b.locomotion_feedback())[:, L.TRACKING_TERMS.index("vel_rumo")])
        return np.array(track), np.array(speed), b.policy_dt

    for h in (a.rumos or (4, 3, 5)):
        for band, name in ((1, "trote"), (2, "corrida")):
            kw = dict(evaluation_band=band, heading=h)
            key = f"lev_{name}_{h}"
            ref, _, _ = run(kw, 1.0, True)
            moved = ref[-1, :, 0, :2] - ref[0, :, 0, :2]
            travel = moved / (np.linalg.norm(moved, axis=1, keepdims=True) + 1e-9)
            for level in (levels[L.DRILL_KEYS.index(key)], 0.0):
                track, speed, dt = run(kw, level, False)
                d = track[:, :, 1, :] - track[:, :, 0, :]
                lean = np.degrees(np.arctan2((d[:, :, :2] * travel[None]).sum(-1), d[:, :, 2]))
                cells = " | ".join(f"{lean[int(x / dt):int(z / dt)].mean():+5.1f} graus {speed[int(x / dt):int(z / dt)].mean():4.2f} m/s"
                                   for x, z in ((0.0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 3.0)))
                fell = (track[:, :, 2, 2].min(axis=0) < 0.6).mean()
                print(f"{key[4:]:10s} forca {level:4.0%} | {cells} | 3 s: {speed.mean() * 3.0:4.2f} m, cairam {fell:.0%}", flush=True)
    print("colunas: o peito contra a pelve no rumo (+ para tras) e a velocidade no rumo em 0-0,5 / 0,5-1 / 1-2 / 2-3 s")


def curva(a, state, levels):
    """A coluna nas curvas (04/11 16h, ele: "no trote curva ele ainda leva a coluna para tras, apesar de estar correto no trote
    frente"): o peito contra a pelve no rumo do deslocamento (graus, + para a FRENTE) e a velocidade, no trote de frente e nas
    curvas 5-7, o clipe puro com a mao cheia, a rede na forca dela e a 0%, de 1 a 6 s."""
    def lines(b):
        out = []
        for w in range(b.num_envs):
            f = {p[1]: np.array([float(x) for x in p[2:5]]) for p in (l.split() for l in b.frame_lines(w).splitlines())
                 if p and p[0] in ("B", "C")}
            out.append((next(v for k, v in f.items() if "Pelvis" in k or "Hips" in k), next(v for k, v in f.items() if "UpperChest" in k)))
        return out

    for key, maneuver in (("lev_trote_0", None), ("lev_curva_suave", 5), ("lev_curva_s", 6), ("lev_curva_fechada", 7)):
        for level, zero in ((1.0, True), (levels[L.DRILL_KEYS.index(key)], False), (0.0, False)):
            b = L.get_up_batch(ASSETS, environments=8, threads=4, seed=59, evaluation=True, evaluation_kind=4,
                               evaluation_band=1, **({"heading": 0} if maneuver is None else {}), **TRACK)
            setup(b, [level] * len(L.DRILL_KEYS))
            if maneuver is not None:
                b.set_maneuvers(0.0, maneuver)
            b.reset(59)
            act = policy(b, state, a.residuo, a.partes)
            track = []
            for _ in range(int(6.0 / b.policy_dt)):
                track.append(lines(b))
                b.step(act(zero))
            t = np.array(track)
            k = max(1, int(0.25 / b.policy_dt))
            move = t[2 * k:, :, 0, :2] - t[:-2 * k, :, 0, :2]
            travel = move / (np.linalg.norm(move, axis=-1, keepdims=True) + 1e-9)
            d = (t[:, :, 1, :] - t[:, :, 0, :])[k:-k]
            lean = np.degrees(np.arctan2((d[..., :2] * travel).sum(-1), d[..., 2]))
            speed = np.linalg.norm(move, axis=-1) / (2 * k * b.policy_dt)
            ok = t[k:-k, :, 0, 2] > 0.6
            s0 = int(1.0 / b.policy_dt)
            name = "clipe" if zero else f"rede {level:4.0%}"
            print(f"{key[4:]:14s} {name:10s} coluna {lean[s0:][ok[s0:]].mean():+5.1f} graus (p10 {np.percentile(lean[s0:][ok[s0:]], 10):+5.1f})"
                  f" vel {speed[s0:][ok[s0:]].mean():4.2f} m/s, cairam {(t[:, :, 0, 2].min(axis=0) < 0.6).mean():.0%}", flush=True)


def firmeza(a, state, levels):
    """A firmeza (a estabilidade v13, 04/11 16h30) contra a estabilidade de antes: por exercicio, o clipe puro com a mao
    cheia, a rede na forca dela e a 0%, de 1 s ate o fim (8 s) ou a queda - os pilares, a margem de Hof (p10, cm), o giro do
    corpo alem do clipe (p90), a estabilidade de antes (a media) e a firmeza (a media e a do episodio, 1 - raiz da media do
    quadrado da falta)."""
    T = L.TRACKING_TERMS
    cols = [T.index(k) for k in ("r_estabilidade", "firme_equilibrio", "firme_base", "firme_corpo", "firme_ritmo", "margem_hof_m",
                                 "giro_corpo_excesso", "firmeza", "escorrega_contato", "tropeco")]
    drills = a.exercicios or ["lev_trote_0", "lev_trote_2", "lev_trote_4", "lev_trote_6", "lev_corrida_0", "lev_corrida_2",
                              "lev_corrida_6", "lev_curva_s", "lev_troca_media", "lev_giro"]
    print(f"{'exercicio':18s} {'quem':9s} antes | equil base corpo ritmo | margem_p10 giro_p90 | firmeza (episodio) | cairam")
    for key in drills:
        # (05/11: o sprint - a faixa 3, para a frente.)
        kw = dict(evaluation_band=3 if key == "lev_sprint" else 2 if "corrida" in key else 1)
        if key.startswith(("lev_trote_", "lev_corrida_")):
            kw["heading"] = int(key[-1])
        if key == "lev_sprint":
            kw["heading"] = 0
        maneuver = L.MANEUVERS.index(key) + 1 if key in L.MANEUVERS else None
        for level, zero in ((1.0, True), (levels[L.DRILL_KEYS.index(key)], False), (0.0, False)):
            b = L.get_up_batch(ASSETS, environments=8, threads=4, seed=61, evaluation=True,
                               evaluation_kind=6 if key == "lev_giro" else 4, **kw, **TRACK)
            setup(b, [level] * len(L.DRILL_KEYS))
            b.set_tracking_reward(12)
            if maneuver is not None:
                b.set_maneuvers(0.0, maneuver)
            b.reset(61)
            act = policy(b, state, a.residuo, a.partes)
            alive = np.ones(b.num_envs, bool)
            got = []
            fell = np.zeros(b.num_envs, bool)
            for step in range(int(8.0 / b.policy_dt)):
                b.step(act(zero))
                f = np.asarray(b.locomotion_feedback())
                down = f[:, 0] == 0
                alive &= ~down
                if step * b.policy_dt >= 1.0:
                    got.append(f[alive][:, cols])
            g = np.concatenate(got) if got else np.zeros((0, len(cols)))
            if len(g) < 10:
                print(f"{key[4:]:18s} {'clipe' if zero else f'rede {level:3.0%}':9s} (todos cairam antes de 1 s)", flush=True)
                continue
            firm = g[:, 7]
            ep = 1.0 - np.sqrt(np.mean((1.0 - firm) ** 2))
            name = "clipe" if zero else f"rede {level:3.0%}"
            print(f"{key[4:]:18s} {name:9s} {g[:, 0].mean():.2f} | {g[:, 1].mean():.2f} {g[:, 2].mean():.2f} {g[:, 3].mean():.2f} "
                  f"{g[:, 4].mean():.2f} | {100 * np.percentile(g[:, 5], 10):+5.1f} cm {np.percentile(g[:, 6], 90):.3f} | "
                  f"{firm.mean():.2f} ({ep:.2f}) | escorrega p50 {np.percentile(g[:, 8], 50):.2f} p90 {np.percentile(g[:, 8], 90):.2f} m/s, tropeco {g[:, 9].mean():.2f} "
                  f"| vivos {alive.mean():.0%}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sonda", choices=("levantar", "evitar", "parado", "velocidade", "partes", "largada", "curva", "firmeza"))
    ap.add_argument("--treino", default="rl_final_clipe")
    ap.add_argument("--checkpoint", default=None, help="outro ultimo.pt (padrao: o do treino)")
    ap.add_argument("--residuo", type=float, default=0.15, help="o alcance da correcao nas pernas e na pelve (o --residuo do treino)")
    ap.add_argument("--partes", type=float, nargs=3, default=(0.18, 0.08, 0.05), metavar=("COLUNA", "BRACOS", "CABECA"))
    ap.add_argument("--forcas", type=float, nargs="+", default=[1.0])
    ap.add_argument("--segundos", type=float, default=30.0)
    ap.add_argument("--forca-empurrao", type=float, nargs=2, default=(1400.0, 2200.0))
    ap.add_argument("--idle-alcance", type=float, default=0.2, help="o alcance da correcao no idle (o --idle-alcance do treino)")
    ap.add_argument("--entrega", type=float, nargs=2, default=(0.6, -1.0), metavar=("SEGUNDOS", "ALCANCE"),
                    help="a entrega ao idle depois de levantar (o --idle-entrega e --idle-entrega-alcance; 0 = a de antes)")
    ap.add_argument("--mundos", type=int, default=24, help="levantar: quantas quedas")
    ap.add_argument("--nota", type=int, default=10, help="a versao da nota do rastreio (o --recompensa-rastreio do treino)")
    ap.add_argument("--todos", action="store_true", help="partes: os 16 rumos (padrao: frente, esquerda e costas)")
    ap.add_argument("--rumos", type=int, nargs="+", default=None, help="partes: so estes rumos (0 a 7)")
    ap.add_argument("--exercicios", nargs="+", default=None, help="firmeza: so estes")
    a = ap.parse_args()
    import freio_treino  # (08/11: a sonda freia o treino enquanto roda.)
    freio_treino.acquire()
    IDLE_REACH[0] = a.idle_alcance
    REWARD[0] = a.nota
    HANDOFF[:] = a.entrega
    if (RUNS / a.treino / "run.json").exists():
        RUN_V2.update(json.loads((RUNS / a.treino / "run.json").read_text()).get("v2", {}))
    # (23h15: o A/B de configuracoes sem mexer no treino - SONDA_V2='{"autoridade_ameaca": 4}'.)
    import os
    if os.environ.get("SONDA_V2"):
        RUN_V2.update(json.loads(os.environ["SONDA_V2"]))
    # (04/11 15h: o layout do treino - o do equilibrio com --equilibrio.)
    L.BATCH_EXTRA.update(L.batch_layout(RUN_V2))
    path = Path(a.checkpoint) if a.checkpoint else RUNS / a.treino / "ultimo.pt"
    state = torch.load(path, map_location="cpu", weights_only=False)
    levels = [float(state["counters"].get("ajuda_exercicios", {}).get(k, 1.0)) for k in L.DRILL_KEYS]
    print(f"{path} (atualizacao {state['counters']['atualizacao']})", flush=True)
    globals()[a.sonda](a, state, levels)


if __name__ == "__main__":
    main()
