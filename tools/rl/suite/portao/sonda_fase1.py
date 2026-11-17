"""(17/11, o plano de desentortar - Fase 1) Confere cada conserto da build-base-n131 antes de qualquer treino, com a rede r130240
(a acao media, 2 mundos, sementes 61 e 62, 6 s; medidas do segundo 2 ao fim):
  base      - tudo desligado: tem de dar o mesmo que a n130 (rodar com MATTER_TRACKING_NATIVE de cada build e comparar a linha BASE);
  referencial, rumo, fase2pes - so a nota (a fase tambem muda a entrada): a semelhanca de cada um contra a base;
  forca     - no chao plano sem nada em volta a entrada tem de ficar IGUAL a da base (o apoio fora da conta);
  grama     - o chao de grama (o atrito do campo): a trajetoria muda.
Tambem o vies do rumo da pelve (a media com sinal do giro da pelve contra o do clipe, em graus) para calibrar o --rumo-medio.
sonda_fase1.py [CONFIG ...] (sem nada: todas)."""
import os
import sys
from pathlib import Path

import numpy as np
import torch

W = Path("/home/dahaka/Projetos/matter_engine_wip")
sys.path.insert(0, str(W / "tools/rl"))
from inspecao import comum  # noqa: E402

CK = Path.home() / ".local/share/matter-engine/rl/runs/cerebro_unico/final_15h_r130240.pt"
CONFIGS = {
    "base": {},
    "referencial": {"referencial_clipe": True},
    "rumo": {"rumo_medio": [0.5, 0.10]},
    "fase2pes": {"fase_dois_pes": True},
    "forca": {"forca_contatos": True},
    "grama": {"chao_grama": 1.0},
    "reta_comando": {"reta_comando": True},
    "reta_rumo": {"reta_comando": True, "rumo_medio": [0.5, 0.10], "referencial_clipe": True},
    "lado": {"lado_medio": [0.5, 0.03], "reta_comando": True},
    "portao": {"imitacao_portao": [0.6, 0.6]},
    "bracos07": {"bracos_clipe": 0.07},
    "bracos05": {"bracos_clipe": 0.05},
}
# (GAITS=lev_comando ... troca as andaduras; o comando e o modo do jogo.)
GAITS = tuple(os.environ.get("GAITS", "lev_corrida_0,lev_trote_0").split(","))
SEEDS = (61, 62)
SECONDS = 6.0

amb = comum.Ambiente("cerebro_unico", str(CK))
names = [lk["id"] for lk in comum.ragdoll()["links"]]
PELVIS = names.index("Pelvis")
base_v2 = dict(amb.S.RUN_V2)
terms = list(amb.L.TRACKING_TERMS)


def yaw_between(qa, qb):
    """O giro em torno do z do mundo de qa contra qb (quaternions x y z w), em graus."""
    ax, ay, az, aw = qa
    bx, by, bz, bw = -qb[0], -qb[1], -qb[2], qb[3]  # o conjugado de qb
    x = aw * bx + ax * bw + ay * bz - az * by
    y = aw * by - ax * bz + ay * bw + az * bx
    z = aw * bz + ax * by - ay * bx + az * bw
    w = aw * bw - ax * bx - ay * by - az * bz
    return np.degrees(2.0 * np.arctan2(z, w) if w >= 0 else 2.0 * np.arctan2(-z, -w))


def run(config: str, key: str, seed: int):
    v2 = dict(base_v2)
    v2.update(CONFIGS[config])
    amb.S.RUN_V2.clear()
    amb.S.RUN_V2.update(v2)
    os.environ.pop("MATTER_CHAO_GRAMA", None)
    b = amb.batch(key, 2, seed)
    amb.setup(b, amb.level(key))
    b.reset(seed)
    model = amb.model(b)
    obs_names = list(b.observation_names())
    fire = [obs_names.index(n) for n in ("fogo.forca.frente", "fogo.forca.lado", "fogo.forca.cima")] if "fogo.forca.frente" in obs_names else []
    phase = [obs_names.index("fase.sen"), obs_names.index("fase.cos")]
    dt = float(b.policy_dt)
    steps = int(round(SECONDS / dt))
    first = int(2.0 / dt)
    sem, nota, bias, obs_log, pelvis_xy, fire_max, jumps = [], [], [[], []], [], [], 0.0, 0
    prev_phase = None
    mirror_rows = [0, 0, 0]
    for i in range(steps):
        obs_np = np.asarray(b.observations())
        obs = torch.as_tensor(obs_np)
        with torch.no_grad():
            dist = model.distribution(model.norm(obs))
            a = model.env_action(dist.mean, obs).numpy().astype(np.float32)
        obs_log.append(obs_np.copy())
        if fire:
            fire_max = max(fire_max, float(np.abs(obs_np[:, fire]).max()))
        ph = np.arctan2(obs_np[:, phase[0]], obs_np[:, phase[1]])
        if prev_phase is not None:
            step_jump = np.abs((ph - prev_phase + np.pi) % (2 * np.pi) - np.pi)
            jumps += int((step_jump > 0.6).sum())
        prev_phase = ph
        b.step(a)
        fb = np.asarray(b.locomotion_feedback())
        if i >= first:
            sem.append(fb[:, terms.index("r_semelhanca")])
            nota.append(fb[:, terms.index("nota")])
        for w in range(2):
            links, _, _ = comum.parse_lines(b.frame_lines(w))
            rlinks, _, _ = comum.parse_lines(b.reference_lines(w))
            if i >= first and "Pelvis" in links and "Pelvis" in rlinks:
                bias[w].append(yaw_between(links["Pelvis"][3:7], rlinks["Pelvis"][3:7]))
            if "Pelvis" in links:
                pelvis_xy.append(links["Pelvis"][:2])
        mi = np.asarray(b.mirror_info())
        mirror_rows[0] += int((mi[:, 0] > .5).sum())
        mirror_rows[1] += int(((mi[:, 0] > .5) & (mi[:, 2] == 12)).sum())
        mirror_rows[2] += len(mi)
        b.finished_episodes()
    print(f"    linhas do espelho: {mirror_rows[0]} de {mirror_rows[2]} (no comando: {mirror_rows[1]})", flush=True)
    window = max(1, int(round(0.6 / dt)))
    smooth = [np.convolve(np.array(v), np.ones(window) / window, mode="valid") for v in bias if len(v) > window]
    return (float(np.mean(sem)), float(np.mean(nota)), (float(np.mean([np.mean(v) for v in bias])),
            float(np.mean([np.mean(np.abs(v)) for v in smooth])) if smooth else float("nan")),
            np.array(obs_log), np.array(pelvis_xy), fire_max, jumps)


wanted = sys.argv[1:] or list(CONFIGS)
results = {}
for config in wanted:
    for key in GAITS:
        for seed in SEEDS:
            results[(config, key, seed)] = run(config, key, seed)
            s, n, bias, _, _, fmax, jumps = results[(config, key, seed)]
            print(f"{config:12s} {key:14s} s{seed}: semelhanca {s:.4f} nota {n:.4f} vies_rumo {bias[0]:+6.2f} graus "
                  f"(|media de 0,6 s| {bias[1]:5.2f}) fogo_max {fmax:.3f} saltos_fase {jumps}", flush=True)
if "base" in wanted:
    print("BASE " + " ".join(f"{results[('base', k, s)][0]:.6f}/{results[('base', k, s)][1]:.6f}" for k in GAITS for s in SEEDS), flush=True)
    for config in wanted:
        if config == "base":
            continue
        for key in GAITS:
            for seed in SEEDS:
                o0, o1 = results[("base", key, seed)][3], results[(config, key, seed)][3]
                p0, p1 = results[("base", key, seed)][4], results[(config, key, seed)][4]
                same_obs = o0.shape == o1.shape and np.array_equal(o0, o1)
                drift = float(np.abs(p0 - p1).max()) if p0.shape == p1.shape else float("nan")
                print(f"  {config:12s} {key:14s} s{seed}: entrada igual a base {same_obs}, pelve afastou {100 * drift:.2f} cm "
                      f"semelhanca {results[(config, key, seed)][0] - results[('base', key, seed)][0]:+.4f}", flush=True)
