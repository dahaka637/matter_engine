"""(13/11 07h40, ele: os levantares a 1,3-1,4 s (de pe) e 1,2 s (correndo), o juiz de estilo mais forte ("menos mole") e a postura
nao estabanada e o auto-equilibrio depois de levantar.) Os 4 levantares LIMPOS (sem fogo), deitados, com a media da politica e com o
ruido do treino. Por exercicio:
- o tempo para levantar (de pe na postura: cabeca > 1,4 m, pelve > 0,75 m; correndo: cabeca > 1,3 m) e quando o corpo comecou a se
  mexer (a cabeca 8 cm acima do comeco) - o "esperar deitado e levantar rapido" apareceria como comecou tarde;
- o juiz da recuperacao (D medio, ~+1 parece clipe) e o premio de estilo durante o levantar;
- nos 1,5 s depois de ficar de pe: a velocidade do centro de massa, o giro do peito, as maos (velocidade relativa a pelve), os passos
  (pes que saem do chao), a margem do ponto de captura (> 0 dentro da base) e a inclinacao do peito;
- caiu depois (o fim do episodio em queda depois de levantar).
O lote e o do treino (lote_do_treino + configurar + configurar_levantar com a receita), so levantares deitados e limpos (sem o meio,
a pose variada, o levantar e ir e o fogo); o primeiro episodio de cada mundo.
Uso (de tools/rl): python sonda_pos_levantar.py CHECKPOINT RECEITA [mundos] [media|amostra|ambos]"""
import json, sys, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum

ck, receita = sys.argv[1], sys.argv[2]
W = int(sys.argv[3]) if len(sys.argv) > 3 else 32
modes = ("media", "amostra") if (len(sys.argv) <= 4 or sys.argv[4] == "ambos") else (sys.argv[4],)
amb = comum.Ambiente('idle_alerta', ck); L = amb.L
args = L.build_parser().parse_args(json.load(open(receita))['argumentos'] + ['--treino', 'sonda'])
args.locomocao, args.giro, args.comeco_de_pe, args.locomocao_fogo, args.comando = 0.0, 0.0, 0.0, 0.0, 0.0
args.levantar_unreal_meio, args.levantar_pose_variada = 0.0, 0.0
import os
if "TROCA" in os.environ: args.levantar_correndo_troca = float(os.environ["TROCA"])  # (o correndo depois da troca, s)
if "DEPOIS" in os.environ: args.levantar_unreal_depois = float(os.environ["DEPOIS"])  # (o de pe depois da troca, s)
if "PAREDES" in os.environ: args.levantar_paredes = (float(os.environ["PAREDES"]), float(os.environ["PAREDES"]))  # (a arena, m)
if "CALMA" in os.environ: args.levantar_calma = float(os.environ["CALMA"])  # (so a nota muda)
if "FIRMEZA" in os.environ: args.levantar_adaptativo_firmeza = float(os.environ["FIRMEZA"])  # (13/11 12h35: a janela de firmeza, s)
if "TROCA_CALMA" in os.environ: args.levantar_troca_calma = tuple(float(v) for v in os.environ["TROCA_CALMA"].split())  # (13/11 14h: VEL MAX MAX_CORRENDO)
only = [int(x) for x in os.environ["EXERCICIOS"].split(",")] if "EXERCICIOS" in os.environ else [0, 1, 2, 3]
args.levantar_e_ir = (0.0, args.levantar_e_ir[1], args.levantar_e_ir[2])
counters = amb.state.get('counters', {})
for k, v in (("comando_fase", 1), ("estrela_pesos", [1.0] * 6), ("fogo_nivel", 0.0), ("lento", [args.lento] * 3)):
    counters.setdefault(k, v)
counters.setdefault("estrela_fases", [1] * 6); counters.setdefault("fogo_niveis", None)
names = None
POST = 1.5
for mode in modes:
    for ex, key in enumerate(L.LEVANTAR_UNREAL_KEYS):
        if ex not in only:
            continue
        run = ex >= 2
        args.semente = 4242 + ex
        b = L.get_up_batch(str(L.REPO / "assets"), environments=W, threads=4, **L.lote_do_treino(args), **L.montar_v2(args))
        L.configurar(b, args, counters, True)
        L.configurar_levantar(b, args, 1.0)
        wts = [0.0] * len(L.LEVANTAR_UNREAL_KEYS); wts[ex] = 1.0; b.set_clip_get_up_weights(wts)
        if names is None:
            names = list(b.observation_names()); ix = {n: i for i, n in enumerate(names)}
        disc = L.Discriminator(b.amp_size); disc.load_state_dict(amb.state["disc"]); disc.eval()
        model = amb.model(b); b.reset(4242 + ex)
        dt = float(b.policy_dt); n = b.num_envs
        g = torch.Generator().manual_seed(7 + ex)
        steps = int(round(9.0 / dt))
        hist = {k: np.full((steps, n), np.nan, np.float32) for k in ("head", "pelvis", "rew", "vcm", "wchest", "hands", "margin", "upc", "lf", "rf", "loco", "D")}
        done = {}
        alive = np.ones(n, bool)
        for s in range(steps):
            obs = torch.as_tensor(b.observations())
            with torch.no_grad():
                dist = model.distribution(model.norm(obs))
                a = dist.mean if mode == "media" else dist.mean + dist.stddev * torch.randn(dist.mean.shape, generator=g)
                act = model.env_action(a, obs).numpy().astype(np.float32)
            o = obs.numpy()
            hist["head"][s] = np.where(alive, o[:, ix["chao.altura.Head"]], np.nan)
            hist["pelvis"][s] = np.where(alive, o[:, ix["chao.altura.Pelvis"]], np.nan)
            hist["vcm"][s] = np.where(alive, np.hypot(o[:, ix["eq.vel_cm.frente"]], o[:, ix["eq.vel_cm.lado"]]), np.nan)
            hist["wchest"][s] = np.where(alive, np.linalg.norm(o[:, [ix["UpperChest.ang.x"], ix["UpperChest.ang.y"], ix["UpperChest.ang.z"]]], axis=1), np.nan)
            pv = o[:, [ix["Pelvis.vel.x"], ix["Pelvis.vel.y"], ix["Pelvis.vel.z"]]]
            hv = [np.linalg.norm(o[:, [ix[f"{h}Hand.vel.x"], ix[f"{h}Hand.vel.y"], ix[f"{h}Hand.vel.z"]]] - pv, axis=1) for h in ("Left", "Right")]
            hist["hands"][s] = np.where(alive, 0.5 * (hv[0] + hv[1]), np.nan)
            hist["margin"][s] = np.where(alive, o[:, ix["eq.margem"]], np.nan)
            q = o[:, [ix["UpperChest.rot.x"], ix["UpperChest.rot.y"], ix["UpperChest.rot.z"], ix["UpperChest.rot.w"]]]
            # (o eixo z do peito no mundo: 1 - 2(x^2 + y^2) - a componente vertical; 1 = ereto)
            hist["upc"][s] = np.where(alive, 1.0 - 2.0 * (q[:, 0] ** 2 + q[:, 1] ** 2), np.nan)
            hist["lf"][s] = np.where(alive, o[:, ix["chao.contato.LeftFoot"]], np.nan)
            hist["rf"][s] = np.where(alive, o[:, ix["chao.contato.RightFoot"]], np.nan)
            hist["loco"][s] = np.where(alive, o[:, -1], np.nan)
            task = np.asarray(b.step(act)[0], np.float32).reshape(-1)
            hist["rew"][s] = np.where(alive & (o[:, -1] < .5), task, np.nan)
            with torch.no_grad():
                d = disc(torch.as_tensor(b.amp_pairs())).reshape(-1).numpy()
            hist["D"][s] = np.where(alive & (o[:, -1] < .5), d, np.nan)
            for r in b.finished_episodes().tolist():
                wi = int(r[0])
                if wi not in done:
                    done[wi] = (s, r); alive[wi] = False
            if not alive.any():
                break
        res = {k: [] for k in ("t", "inicio", "caiu", "D", "estilo", "vcm", "wchest", "maos", "passos", "margem_fora", "peito_incl", "troca", "caiu_aos", "codigo", "nota_de_pe")}
        for wi in range(n):
            end = done[wi][0] if wi in done else steps - 1
            r = done[wi][1] if wi in done else None
            head = hist["head"][:end + 1, wi]; pel = hist["pelvis"][:end + 1, wi]
            up = np.nonzero((head > 1.3) if run else ((head > 1.4) & (pel > 0.75)))[0]
            D = hist["D"][:end + 1, wi]
            if np.isfinite(D).any():
                res["D"].append(np.nanmean(D)); res["estilo"].append(np.nanmean(np.clip(1 - 0.25 * (D - 1) ** 2, 0, None)))
            mv = np.nonzero(head > head[0] + 0.08)[0]
            if mv.size:
                res["inicio"].append(mv[0] * dt)
            if r is not None and int(r[1]) != 9:
                continue
            fell = r is not None and int(r[32]) in (1, 2, 3, 4, 8)
            if not up.size:
                res["caiu"].append(1.0); continue
            u = up[0]
            res["t"].append(u * dt)
            res["caiu"].append(1.0 if fell else 0.0)
            lo = np.nonzero(hist["loco"][:end + 1, wi] > .5)[0]
            if lo.size:
                res["troca"].append((lo[0] - u) * dt)
                if fell:
                    res["caiu_aos"].append((end - lo[0]) * dt); res["codigo"].append(int(r[32]))
            w = slice(u, min(end + 1, u + int(round(POST / dt))))
            rw = hist["rew"][w, wi]
            if np.isfinite(rw).any():
                res["nota_de_pe"].append(np.nanmean(rw))
            res["vcm"].append(np.nanmean(hist["vcm"][w, wi])); res["wchest"].append(np.nanmean(hist["wchest"][w, wi]))
            res["maos"].append(np.nanmean(hist["hands"][w, wi]))
            res["margem_fora"].append(np.nanmean(hist["margin"][w, wi] < 0))
            res["peito_incl"].append(np.degrees(np.arccos(np.clip(np.nanmean(hist["upc"][w, wi]), -1, 1))))
            lifts = 0
            for f in ("lf", "rf"):
                c = hist[f][w, wi]; c = c[np.isfinite(c)]
                lifts += int(((c[:-1] > .5) & (c[1:] < .5)).sum())
            res["passos"].append(lifts)
        m = lambda k, f=".2f": format(np.mean(res[k]), f) if res[k] else "-"  # noqa: E731
        print(f"{mode:7s} {key[4:]:24s} n{n} t {m('t')}s (comeca {m('inicio')}s) caiu {m('caiu')} | juiz D {m('D', '+.2f')} estilo {m('estilo')} | "
              f"1,5 s depois: vcm {m('vcm')} m/s giro peito {m('wchest')} rad/s maos {m('maos')} m/s passos {m('passos', '.1f')} "
              f"nota de pe (levantar) {m('nota_de_pe')} captura fora {m('margem_fora')} peito {m('peito_incl', '.0f')} graus | troca +{m('troca')}s caiu {m('caiu_aos')}s depois dela "
              f"({sorted(round(float(x), 1) for x in res['caiu_aos'])}) codigos {sorted(res['codigo'])}", flush=True)
