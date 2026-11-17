"""(16/11 17h20) A volta dos treinos (ele: "estruture a melhoria do RL, vamos voltar os treinos... tentar fazer em MENOS tempo"; o plano no
vault, 04_locomocao/plano_melhoria_rl_16_11.md). Um braco por foco: cada um parte da melhor rede ate agora (a final_15h_r130240 ou a do ultimo
aprovado), treina TREINO_MIN minutos escondido do laboratorio (socket proprio), mede com a checagem do portao e o peso do parado, e passa so se
atingir o alvo dele sem piorar o resto alem do ruido. O aprovado vira a base do proximo e vai ao laboratorio e ao jogo (policy_latest do
cerebro_unico; a anterior guardada). A referencia e medida enquanto o primeiro braco treina. Estado e relatorio em
~/.local/share/matter-engine/rl/ab16/ (fora do scratchpad). Uso: ab16.py [--so-medir] [--testes 1,2,...]
(Adaptado de rolar_r72889/noite_ab.py, a noite de 15/11, que nao chegou a rodar.)"""
import json, os, re, shutil, subprocess, sys, threading, time
from datetime import datetime
from pathlib import Path

RUNS = Path.home() / ".local/share/matter-engine/rl/runs"
MAIN = RUNS / "cerebro_unico"
W = Path("/home/dahaka/Projetos/matter_engine_wip")
G = W / "tools/rl/suite/portao"
S = Path.home() / ".local/share/matter-engine/rl/ab16"
P = "/home/dahaka/Projetos/matter_engine/tools/rl/.venv/bin/python"
BUILD = os.environ.get("BUILD", "build-base-n129")
TREINO_MIN = float(os.environ.get("TREINO_MIN", "25"))  # (17h40: 35 -> 25, ele: "MENOS tempo")
THREADS = os.environ.get("THREADS", "8")
SOCKET = "/tmp/matter_rl_ab16.sock"
REPORT = S / "relatorio.md"
STATE = S / "estado.json"


def log(msg):
    line = f"- {datetime.now().strftime('%H:%M')} {msg}"
    print(line, flush=True)
    with REPORT.open("a") as f:
        f.write(line + "\n")


def num(pattern, text, cast=float, default=None):
    m = re.search(pattern, text)
    return cast(m.group(1)) if m else default


def parse(check, peso):
    m = {}
    arr = check.split("== cruzeiro")[0]
    m["largada_1s"] = num(r"lev_corrida_0 \(pedido [^)]*\): vel 0,5 s [0-9.]+ m/s, 1 s ([0-9.]+)", arr)
    m["maos"] = num(r"lev_corrida_0 \(pedido [^)]*\):.*?maos contra o peito ([0-9.]+)", arr)
    for k in ("corrida_0", "corrida_1", "corrida_7", "corrida_4", "trote_0"):
        m["cru_" + k] = num(r"lev_%s \(pedido [^)]*\): cruzeiro 3-6 s [0-9.]+ m/s \(([0-9]+)%%" % k, check)
        m["vel_" + k] = num(r"lev_%s \(pedido [^)]*\): cruzeiro 3-6 s ([0-9.]+) m/s" % k, check)
    for k in ("costas", "brucos", "correndo_costas", "correndo_brucos"):
        m["lev_pe_" + k] = num(r"lev_levantar_%s: de pe ([0-9]+)/32" % k, check, int)
        m["lev_tempo_" + k] = num(r"lev_levantar_%s: de pe [0-9]+/32, tempo ([0-9.]+) s" % k, check)
        m["lev_dist_" + k] = num(r"lev_levantar_%s: de pe [0-9]+/32, tempo [0-9.na]+ s, distancia ao clipe ([0-9.]+)" % k, check)
        m["lev_saida_" + k] = num(r"lev_levantar_%s: de pe .*?saida ([0-9.]+) m/s" % k, check)
    errs, width = [], None
    for line in check.splitlines():
        mm = re.match(r"(lev_\w+) \| esq: pe ([-+0-9.]+)/([-+0-9.]+) joelho ([-+0-9.]+)/([-+0-9.]+) largura ([-+0-9.]+)/([-+0-9.]+).*\| dir: pe ([-+0-9.]+)/([-+0-9.]+) joelho ([-+0-9.]+)/([-+0-9.]+) largura ([-+0-9.]+)/([-+0-9.]+)", line)
        if mm:
            g = [float(x) for x in mm.groups()[1:]]
            errs += [abs(g[0] - g[1]), abs(g[2] - g[3]), abs(g[6] - g[7]), abs(g[8] - g[9])]
            if mm.group(1) == "lev_corrida_0":
                width = 0.5 * (g[4] + g[10])
    m["pernas_erro"] = sum(errs) / len(errs) if errs else None
    m["largura_frente"] = width
    m["parado"] = num(r"lev_parado limpos 0-3: n=\d+ \| pelve se afasta ate ([0-9.]+) cm", check)
    der, pas = 0.0, 0.0
    for k in ("costas", "brucos"):
        der += num(r"lev_levantar_%s: n=\d+ de pe em [0-9.]+ s \| pelve: ate de pe [0-9.]+ cm, depois de de pe ([0-9.]+) cm" % k, check, float, 0.0)
        pas += num(r"lev_levantar_%s: n=\d+ de pe em .*?passos depois de de pe ([0-9.]+)" % k, check, float, 0.0)
    m["deriva_pos"] = der
    m["passos_pos"] = pas
    tot = 0
    for k in ("costas", "brucos"):
        for g in ("empurroes", "pancadas"):
            tot += num(r"lev_levantar_%s %s: de pe ([0-9]+)/12" % (k, g), check, int, 0)
    m["fogo_de_pe"] = tot
    m["peso"] = num(r"diferenca do peso contra o clipe ([0-9.]+) cm", peso)
    # (21h30, ele: "e seguir a porra do clipe") A distancia do corpo ao clipe na locomocao (cm, a media dos elos nas seis andaduras).
    m["dist_loco"] = num(r"DISTLOCO media ([0-9.]+)", check)
    for k in ("trote_0", "corrida_0", "corrida_1", "trote_2", "trote_4", "corrida_4"):
        m["dist_" + k] = num(r"lev_%s: distancia ao clipe ([0-9.]+) cm" % k, check)
    return m


# (As tolerancias de ruido de 15/11: o que conta como piorar.)
def regressions(b, t, skip=()):
    bad = []

    def worse(key, tol, higher_is_better=True, label=None):
        if b.get(key) is None or t.get(key) is None or key in skip:
            return
        d = (t[key] - b[key]) if higher_is_better else (b[key] - t[key])
        if d < -tol:
            bad.append(f"{label or key} {b[key]:g} -> {t[key]:g}")
    for k in ("corrida_0", "corrida_1", "corrida_7"):
        worse("cru_" + k, 3, True, "cruzeiro " + k + " %")
    worse("cru_corrida_4", 4, True, "cruzeiro costas %")
    if t.get("vel_trote_0") is not None and not (3.5 <= t["vel_trote_0"] <= 4.2):
        bad.append(f"trote {t['vel_trote_0']:g} m/s fora de 3,5-4,2")
    worse("largada_1s", 0.25, True, "largada 1 s")
    for k in ("costas", "brucos", "correndo_costas", "correndo_brucos"):
        worse("lev_pe_" + k, 1, True, "de pe " + k)
        worse("lev_dist_" + k, 3.0 if "correndo" not in k else 4.0, False, "distancia " + k)
    # (20h20) A saida dos correndo (o braco 9 passou com o de costas 3,5 -> 2,2 m/s sem guarda; no jogo a corrida depois do levantar ficou igual).
    worse("lev_saida_correndo_costas", 0.6, True, "saida do correndo de costas m/s")
    worse("lev_saida_correndo_brucos", 0.6, True, "saida do correndo de brucos m/s")
    worse("lev_tempo_costas", 0.15, False, "tempo de pe costas s")
    worse("lev_tempo_brucos", 0.15, False, "tempo de pe brucos s")
    worse("parado", 2.0, False, "parado cm")
    worse("deriva_pos", 12.0, False, "deriva depois de levantar cm")
    worse("pernas_erro", 3.0, False, "pernas erro graus")
    worse("largura_frente", 3.0, False, "largura corrida cm")
    worse("fogo_de_pe", 5, True, "de pe sob fogo /48")
    worse("peso", 1.5, False, "peso do parado cm")
    worse("maos", 0.2, False, "maos na largada")
    worse("dist_loco", 1.5, False, "distancia ao clipe na locomocao cm")
    return bad


# (16/11 17h40) A regua do jogo (regua_jogo.sh): a suavidade, as pancadas e as colisoes na fisica do jogo, com a rede exportada do braco.
JOGO_CAMPEAO = ("JOGO suave p95 cima 28.3 maos 89.4 cabeca 15.7 | pancadas de pe 22/30 (parado 10/12 trote 7/9 corrida 5/9) volta 1.62 s"
                " | colisoes de volta ao padrao 5/7, quedas 4 | obstaculos: quedas 1 | SUAVE30 cima 21.7 maos 65.0 cabeca 12.5")  # (17h26, 17h39 e 18h03, a r130240 - a policy_latest do cerebro_unico; a regua do jogo e deterministica: a de 18h03 repetiu a de 17h26)


def parse_jogo(out):
    line = next((x for x in out.splitlines() if x.startswith("JOGO")), "")
    return {"j_cima": num(r"cima ([0-9.]+)", line), "j_maos": num(r"maos ([0-9.]+)", line), "j_cabeca": num(r"cabeca ([0-9.]+)", line),
            "j_pancadas": num(r"pancadas de pe ([0-9]+)/", line, int), "j_pancadas_corrida": num(r"corrida ([0-9]+)/", line, int),
            "j_volta": num(r"volta ([-0-9.]+) s", line), "j_colisoes": num(r"padrao ([0-9]+)/", line, int), "j_quedas": num(r", quedas ([0-9]+)", line, int),
            "j_obst": num(r"obstaculos: quedas ([0-9]+)", line, int),
            # (18h10) A suavidade a 30 Hz (o tranco que se ve; a de 120 Hz e sobretudo vibracao de milimetros).
            "j30_cima": num(r"SUAVE30 cima ([0-9.]+)", line), "j30_maos": num(r"SUAVE30 cima [0-9.]+ maos ([0-9.]+)", line),
            "j30_cabeca": num(r"SUAVE30 cima [0-9.]+ maos [0-9.]+ cabeca ([0-9.]+)", line),
            # (21h) O torto no jogo (ele: "corrida e trote ele esta bem torto"): a assimetria dos passos (tempo e comprimento, %) e a
            # inclinacao de lado do peito (graus) no trote e na corrida de frente.
            "j_torto_trote": num(r"TORTO trote assim ([0-9.]+)%", line), "j_peito_trote": num(r"TORTO trote assim [0-9.]+% peito ([-+0-9.]+)", line),
            "j_torto_corrida": num(r"corrida assim ([0-9.]+)%", line), "j_peito_corrida": num(r"corrida assim [0-9.]+% peito ([-+0-9.]+)", line)}


def regressions_jogo(b, t):
    bad = []
    for k, label in (("j_cima", "jogo suave corpo de cima p95"), ("j_maos", "jogo suave maos p95"),
                     ("j30_cima", "jogo suave 30 Hz corpo de cima p95"), ("j30_maos", "jogo suave 30 Hz maos p95")):
        if b.get(k) and t.get(k) is not None and t[k] > b[k] * 1.15:
            bad.append(f"{label} {b[k]:g} -> {t[k]:g}")
    for k, tol, label in (("j_pancadas", 3, "jogo pancadas de pe /30"), ("j_colisoes", 1, "jogo colisoes de volta /7")):
        if b.get(k) is not None and t.get(k) is not None and t[k] < b[k] - tol:
            bad.append(f"{label} {b[k]} -> {t[k]}")
    if b.get("j_quedas") is not None and t.get("j_quedas") is not None and t["j_quedas"] > b["j_quedas"] + 2:
        bad.append(f"jogo quedas nas colisoes {b['j_quedas']} -> {t['j_quedas']}")
    for k in ("j_torto_trote", "j_torto_corrida"):
        if b.get(k) is not None and t.get(k) is not None and t[k] > b[k] + 5:
            bad.append(f"jogo {k} % {b[k]:g} -> {t[k]:g}")
    for k in ("j_peito_trote", "j_peito_corrida"):
        if b.get(k) is not None and t.get(k) is not None and abs(t[k]) > abs(b[k]) + 1.5:
            bad.append(f"jogo {k} graus {b[k]:+g} -> {t[k]:+g}")
    if b.get("j_obst") is not None and t.get("j_obst") is not None and t["j_obst"] > b["j_obst"] + 1:
        bad.append(f"jogo quedas no corredor/meio-fio/parede {b['j_obst']} -> {t['j_obst']}")
    return bad


def mean(*xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


# Os bracos, um por foco (o plano de 16/11). Alvo e guardas escritos antes.
TESTS = [
    (1, "levantar_correndo",
     "o levantar correndo no ritmo natural do clipe (entrega 1,57-1,70 s; o treino comprimia a 1,2 s) com a saida cobrada: "
     "--levantar-imitar-ritmo-correndo 1.2 -> 1.6, --levantar-tempo-alvo-correndo 1.15 1.25 -> 1.5 1.8, --levantar-velocidade-corrida 0.5 -> 1.0",
     {"set": {"--levantar-imitar-ritmo-correndo": ["1.6"], "--levantar-tempo-alvo-correndo": ["1.5", "1.8"],
              "--levantar-velocidade-corrida": ["1.0", "1.0"]}},
     lambda b, t: mean(t.get("lev_saida_correndo_costas"), t.get("lev_saida_correndo_brucos")) is not None
     and mean(t["lev_saida_correndo_costas"], t["lev_saida_correndo_brucos"])
     >= mean(b["lev_saida_correndo_costas"], b["lev_saida_correndo_brucos"]) + 0.5,
     "a saida dos correndo +0,5 m/s (media dos dois)"),
    (2, "levantares_ritmo_clipe",
     "(ele, 16/11 17h50: 'obedecer a velocidade dos clipes e esse ~2,0 s... achei muito rapido... isso para todos os levantares, a "
     "velocidade original do clipe') os quatro no ritmo original: o relogio do clipe so com o tempo (--levantar-unreal-agilidade 0.5 1.0 "
     "-> 0 0: o premio por ficar adiantado do clipe sai), de pe --levantar-ritmo 1.35 -> 2.0 s e a faixa 1.30-1.40 -> 1.9-2.1 s, o de "
     "brucos x1,1 (2,2 s, --levantar-brucos-tempo), os correndo --levantar-imitar-ritmo-correndo 1.2 -> 1.65 e a faixa 1.5-1.8 s com a "
     "saida cobrada (--levantar-velocidade-corrida 1.0); a ancora so na locomocao (--ancora-so-locomocao: ela prendia os levantares de "
     "brucos e correndo a rede rapida)",
     {"set": {"--levantar-unreal-agilidade": ["0", "0"], "--levantar-ritmo": ["3.0", "2.0"], "--levantar-tempo-alvo": ["1.9", "2.1"],
              "--levantar-imitar-ritmo-correndo": ["1.65"], "--levantar-tempo-alvo-correndo": ["1.5", "1.8"],
              "--levantar-velocidade-corrida": ["1.0", "1.0"]},
      "add": ["--levantar-brucos-tempo", "1.1", "--ancora-so-locomocao"],
      "ignorar": ["lev_tempo_costas", "lev_tempo_brucos"]},
     lambda b, t: None not in (t.get("lev_tempo_costas"), t.get("lev_tempo_brucos")) and t["lev_tempo_costas"] >= 1.7
     and t["lev_tempo_brucos"] >= 1.85 and t["lev_pe_costas"] >= 31 and t["lev_pe_brucos"] >= 31,
     "os de pe no ritmo do clipe: costas >= 1,7 s e brucos >= 1,85 s (de 1,29/1,27; o clipe ~2,0/2,2), 31/32 de pe"),
    (7, "levantares_ritmo_clipe_b",
     "o braco 2 continuado (18h30: o ritmo do clipe veio - costas 1,91 s, brucos 2,23 s - mas a diagonal 81 -> 76%, a deriva depois de "
     "levantar 104 -> 126 cm e o corredor do jogo 1 -> 3 quedas): mais 25 min do ponto final dele, a ancora da locomocao 0,5 -> 1,0",
     {"set": {"--levantar-unreal-agilidade": ["0", "0"], "--levantar-ritmo": ["3.0", "2.0"], "--levantar-tempo-alvo": ["1.9", "2.1"],
              "--levantar-imitar-ritmo-correndo": ["1.65"], "--levantar-tempo-alvo-correndo": ["1.5", "1.8"],
              "--levantar-velocidade-corrida": ["1.0", "1.0"]},
      "add": ["--levantar-brucos-tempo", "1.1", "--ancora-so-locomocao"],
      "ignorar": ["lev_tempo_costas", "lev_tempo_brucos"],
      "partida": str(RUNS / "ab16_2_levantares_ritmo_clipe" / "braco2_final.pt"), "ancora_peso": "1.0"},
     lambda b, t: None not in (t.get("lev_tempo_costas"), t.get("lev_tempo_brucos")) and t["lev_tempo_costas"] >= 1.7
     and t["lev_tempo_brucos"] >= 1.85 and t["lev_pe_costas"] >= 31 and t["lev_pe_brucos"] >= 31,
     "os de pe no ritmo do clipe (costas >= 1,7 s, brucos >= 1,85 s, 31/32) sem as pioras do braco 2"),
    (3, "poses_corrida",
     "as poses da corrida: os pes na largura do clipe (--corrida-largura 0.3: correndo, o custo dos pes mais abertos que no clipe alem de "
     "4 cm) e as pernas alinhadas tambem no comando, que e como o jogo dirige a rede (--pernas-alinhadas-comando)",
     {"add": ["--corrida-largura", "0.3", "--pernas-alinhadas-comando"]},
     lambda b, t: t.get("largura_frente") is not None and t["largura_frente"] <= b["largura_frente"] - 4.0,
     "a largura da corrida de frente -4 cm (clipe 12 cm)"),
    (8, "levantares_ritmo_clipe_c",
     "o ritmo do clipe de novo (18h45: o 2b caiu no lancamento - sem a pasta exports): os ajustes do braco 2 com a ancora da locomocao "
     "0,5 -> 1,0; continua do ponto final do braco 2 se a melhor rede ainda e a r130240, senao parte da melhor (o braco 3 aprovado)",
     {"set": {"--levantar-unreal-agilidade": ["0", "0"], "--levantar-ritmo": ["3.0", "2.0"], "--levantar-tempo-alvo": ["1.9", "2.1"],
              "--levantar-imitar-ritmo-correndo": ["1.65"], "--levantar-tempo-alvo-correndo": ["1.5", "1.8"],
              "--levantar-velocidade-corrida": ["1.0", "1.0"]},
      "add": ["--levantar-brucos-tempo", "1.1", "--ancora-so-locomocao"],
      "ignorar": ["lev_tempo_costas", "lev_tempo_brucos"],
      "partida_se_campeao": str(RUNS / "ab16_2_levantares_ritmo_clipe" / "braco2_final.pt"), "ancora_peso": "1.0"},
     lambda b, t: None not in (t.get("lev_tempo_costas"), t.get("lev_tempo_brucos")) and t["lev_tempo_costas"] >= 1.7
     and t["lev_tempo_brucos"] >= 1.85 and t["lev_pe_costas"] >= 31 and t["lev_pe_brucos"] >= 31,
     "os de pe no ritmo do clipe (costas >= 1,7 s, brucos >= 1,85 s, 31/32) sem as pioras do braco 2"),
    (10, "levantares_ritmo_clipe_d",
     "o braco 8 continuado (19h20: ritmo e muito mais vieram - costas 1,96 s, brucos 2,13 s, pernas 10,1 -> 7,3 graus, cruzeiro 84 -> 87%, "
     "largada 2,11 m/s, fogo 31/48, quedas nas colisoes 4 -> 2 - mas o brucos limpo anda 1 m depois de ficar de pe: deriva 104 -> 121 cm): "
     "mais 25 min do ponto final dele com o lugar do parado mais forte (--parado-lugar 0.5 -> 0.7, posto na troca do levantar). As "
     "distancias ao clipe dos levantares nao contam: com a agilidade, o relogio do clipe seguia o corpo e a base media so a forma; "
     "agora o clipe corre no tempo dele e a distancia mede forma e ritmo (a folha confere a forma)",
     {"set": {"--levantar-unreal-agilidade": ["0", "0"], "--levantar-ritmo": ["3.0", "2.0"], "--levantar-tempo-alvo": ["1.9", "2.1"],
              "--levantar-imitar-ritmo-correndo": ["1.65"], "--levantar-tempo-alvo-correndo": ["1.5", "1.8"],
              "--levantar-velocidade-corrida": ["1.0", "1.0"], "--parado-lugar": ["0.7", "0.10"]},
      "add": ["--levantar-brucos-tempo", "1.1", "--ancora-so-locomocao"],
      "ignorar": ["lev_tempo_costas", "lev_tempo_brucos", "lev_dist_costas", "lev_dist_brucos", "lev_dist_correndo_costas",
                  "lev_dist_correndo_brucos"],
      "partida": str(RUNS / "ab16_8_levantares_ritmo_clipe_c" / "braco8_final.pt"), "ancora_peso": "1.0"},
     lambda b, t: None not in (t.get("lev_tempo_costas"), t.get("lev_tempo_brucos")) and t["lev_tempo_costas"] >= 1.7
     and t["lev_tempo_brucos"] >= 1.85 and t["lev_pe_costas"] >= 31 and t["lev_pe_brucos"] >= 31,
     "os de pe no ritmo do clipe (costas >= 1,7 s, brucos >= 1,85 s, 31/32) e a deriva depois de levantar de volta a tolerancia"),
    (9, "poses_corrida_b",
     "as poses da corrida de novo, por cima do ritmo do clipe (18h53: o braco 3 atingiu o alvo - largura 22,2 -> 17,2 cm, colisoes 5/7 -> "
     "7/7 - e so o levantar de brucos passou da tolerancia, 8,2 -> 11,5 cm contra 3 cm): --corrida-largura 0.3 + --pernas-alinhadas-comando",
     {"add": ["--corrida-largura", "0.3", "--pernas-alinhadas-comando"]},
     lambda b, t: t.get("largura_frente") is not None and t["largura_frente"] <= b["largura_frente"] - 4.0,
     "a largura da corrida de frente -4 cm (clipe 12 cm)"),
    (4, "suavidade",
     "o tranco da acao na locomocao 3x (--suave-loco 0.1 -> 0.3; no levantar segue 2.5), a velocidade como guarda dura",
     {"set": {"--suave-loco": ["0.3"]}},
     lambda b, t: None not in (t.get("j30_maos"), b.get("j30_maos")) and t["j30_maos"] <= 0.85 * b["j30_maos"] and t["j30_cima"] <= b["j30_cima"],
     "no jogo, a 30 Hz (o tranco que se ve): as maos p95 -15% e o corpo de cima sem piorar (campeao: maos 65, a corrida diagonal 144)"),
    (5, "equilibrio",
     "as pancadas mais fortes na locomocao: o piso do nivel do fogo 0.6 -> 0.8 (--locomocao-fogo-faixa; os niveis estavam presos no piso)",
     {"set": {"--locomocao-fogo-faixa": ["0.8", "1.0"]}},
     lambda b, t: t.get("j_pancadas") is not None and t["j_pancadas"] >= b["j_pancadas"] + 3,
     "no jogo, as pancadas de pe +3/30"),
    (14, "imitar_clipe",
     "(21h30, ele: 'nao e questao de SIMETRIA, e seguir a porra do clipe') a imitacao supervisionada dos alvos do clipe na locomocao, "
     "constante: --imitar 1.0 (a acao da rede puxada para a do clipe em cada passo; a de antes decaia a zero em 300 atualizacoes e o "
     "treino esta na 131 mil - nada mandava seguir o clipe pose a pose). Base: a distancia do corpo ao clipe 9,2 cm na media das seis "
     "andaduras (corrida: pernas 18,8, bracos 14,3)",
     {"set": {"--imitar": ["1.0"]}, "add": ["--imitar-atualizacoes", "1000000000"]},
     lambda b, t: t.get("dist_loco") is not None and b.get("dist_loco") is not None and t["dist_loco"] <= 0.85 * b["dist_loco"],
     "a distancia do corpo ao clipe na locomocao -15%"),
    (15, "nota_rastreio",
     "(22h, a causa raiz de ele nao seguir o clipe: a nota da locomocao e 25% a semelhanca com o clipe e 75% a velocidade - "
     "--nota-velocidade 0.25 - numa regua frouxa - --imitacao-folga 2, os expoentes da semelhanca pela metade; o juiz tambem 0,25. O "
     "braco 14 - a imitacao dos alvos do clipe - deixou as pernas 12,8 -> 6,6 graus do clipe mas a corrida caiu 83 -> 72%: copiar o alvo "
     "tira a propulsao). Aqui a rede tem de correr rapido FAZENDO a pose do clipe: a semelhanca 25 -> 60% da nota "
     "(--nota-velocidade 0.6) e a regua 2 -> 1,3 (--imitacao-folga)",
     {"set": {"--nota-velocidade": ["0.6"], "--imitacao-folga": ["1.3"]}},
     lambda b, t: t.get("dist_loco") is not None and b.get("dist_loco") is not None and t["dist_loco"] <= 0.9 * b["dist_loco"],
     "a distancia do corpo ao clipe na locomocao -10% sem perder velocidade"),
    (16, "rastreio_e_imitacao",
     "(22h15) o braco 15 continuado (a semelhanca 60% da nota e a regua 1,3: aos 15 min a corrida ja 17 -> 5% de assimetria no jogo, mas a "
     "distancia ao clipe ainda 9,4 cm - a nota muda devagar) com a imitacao leve dos alvos do clipe (--imitar 0.3; com 1,0 as pernas "
     "foram a 6,6 graus do clipe mas a corrida caiu 11 pontos): a pose do clipe pelas duas vias, a velocidade ainda na nota",
     {"set": {"--nota-velocidade": ["0.6"], "--imitacao-folga": ["1.3"], "--imitar": ["0.3"]}, "add": ["--imitar-atualizacoes", "1000000000"],
      "partida": str(RUNS / "ab16_15_nota_rastreio" / "braco15_final.pt")},
     lambda b, t: t.get("dist_loco") is not None and b.get("dist_loco") is not None and t["dist_loco"] <= 0.9 * b["dist_loco"],
     "a distancia do corpo ao clipe na locomocao -10% sem perder velocidade"),
    (13, "juiz_estilo",
     "(21h15) o juiz de estilo saturado (a revisao de 15/11, achado C: a margem da locomocao 1,85 de 2 - d_clipe +0,92, d_corpo -0,92 - "
     "em todas as atualizacoes: o estilo vale 0,5 na locomocao e nao da gradiente; ele: 'todos os movimentos estao errados'): a "
     "penalidade de gradiente do discriminador 10 -> 40 (a fronteira mais suave, as notas sem saturar) e o freio do juiz em 1,4 (so "
     "aprende com a margem abaixo disso)",
     {"set": {"--disc-gp": ["40"]}, "add": ["--juiz-freio", "1.4"]},
     lambda b, t: t.get("juiz_loco_margem") is not None and t["juiz_loco_margem"] <= 1.5,
     "o juiz da locomocao desaturado: a margem <= 1,5 (de 1,85)"),
    (6, "colisoes",
     "a locomocao com coisas em volta do caminho (--locomocao-ambiente 0.25: 2-5 caixas, paredes curtas e pilares do tamanho de um "
     "jogador, 12% das da frente na linha; build n130) - e no jogo esta rede le o ambiente de verdade (o run.json com locomocao_ambiente)",
     {"add": ["--locomocao-ambiente", "0.25"], "build": "build-base-n130"},
     lambda b, t: t.get("j_obst") is not None and t["j_obst"] <= b["j_obst"]
     and (t["j_quedas"] <= b["j_quedas"] - 2 or t["j_colisoes"] >= b["j_colisoes"] + 1),
     "no jogo, vendo o ambiente: o corredor sem mais quedas e as colisoes melhores (quedas -2 ou +1 de volta ao padrao)"),
]


def recipe(base_ck, accepted, test_mod):
    r = json.load(open(MAIN / "receita_principal_etapa14.json"))
    a = r["argumentos"]
    # (18h30) O peso da ancora do braco (ancora_peso; 0,5 de sempre).
    i = a.index("--levantar-imitar-professor-ancora"); a[i + 1] = str(base_ck); a[i + 2] = str((test_mod or {}).get("ancora_peso", "0.5"))
    for mod in accepted + ([test_mod] if test_mod else []):
        for k, vals in mod.get("set", {}).items():
            i = a.index(k)
            for j, v in enumerate(vals):
                a[i + 1 + j] = v
        a += [x for x in mod.get("add", [])]
    return r


def juiz(treino):
    """(21h15) As margens dos juizes de estilo na ultima linha do treino (a nota media dos clipes menos a do corpo, 0-2; 1,85 e saturado:
    o estilo nao da gradiente - a revisao de 15/11, achado C)."""
    out = {}
    f = RUNS / treino / "treino.out"
    if f.exists():
        last = next((x for x in reversed(f.read_text(errors="ignore").splitlines()) if x.startswith("atualizacao=")), "")
        for k in ("d_clipe", "d_corpo", "d_loco_clipe", "d_loco_corpo"):
            m = re.search(r"\b%s=([-0-9.e+]+)" % k, last)
            if m:
                out[k] = float(m.group(1))
        if "d_loco_clipe" in out and "d_loco_corpo" in out:
            out["juiz_loco_margem"] = round(out["d_loco_clipe"] - out["d_loco_corpo"], 3)
        if "d_clipe" in out and "d_corpo" in out:
            out["juiz_margem"] = round(out["d_clipe"] - out["d_corpo"], 3)
    return {k: v for k, v in out.items() if k.startswith("juiz")}


def measure(treino, ck, tag, build=BUILD):
    env = dict(os.environ, TREINO=treino, PONTO_FONTE=str(ck), BUILD=build)
    check = subprocess.run(["bash", str(G / "checagem_etapa.sh"), tag], capture_output=True, text=True, env=env, timeout=3600).stdout
    peso = subprocess.run(["nice", "-n", "15", P, str(G / "peso_parado.py"), treino, str(ck)], capture_output=True, text=True, timeout=1200,
                          cwd=str(W / "tools/rl"), env=dict(os.environ, MATTER_TRACKING_NATIVE=str(W / build / "python"),
                                                            TMPDIR=str(W / "build-profile/tmp"))).stdout
    (S / f"checagem_{tag}.txt").write_text(check + "\n" + peso)
    m = parse(check, peso)
    m.update(juiz(treino))
    return m


def measure_jogo(treino, ck, tag, build=BUILD):
    if treino == "cerebro_unico":
        return parse_jogo(JOGO_CAMPEAO)
    exp = RUNS / treino / "exports"; exp.mkdir(parents=True, exist_ok=True)
    subprocess.run(["nice", "-n", "15", P, str(G / "exportar.py"), treino, str(ck), str(exp / "policy_latest.mrlpolicy")], capture_output=True,
                   text=True, cwd=str(W / "tools/rl"), env=dict(os.environ, MATTER_TRACKING_NATIVE=str(W / build / "python"),
                                                                TMPDIR=str(W / "build-profile/tmp")))
    out = subprocess.run(["bash", str(G / "regua_jogo.sh"), str(RUNS / treino), tag], capture_output=True, text=True, timeout=3600).stdout
    (S / f"jogo_{tag}.txt").write_text(out)
    return parse_jogo(out)


def unit_active(unit):
    return subprocess.run(["systemctl", "--user", "is-active", unit], capture_output=True, text=True).stdout.strip() == "active"


def unit_treino(unit):
    """(18h10) O treino que a unidade roda (o --treino da linha de comando dela), para retomar um braco ja treinando."""
    out = subprocess.run(["systemctl", "--user", "show", unit, "-p", "ExecStart"], capture_output=True, text=True).stdout
    m = re.search(r"--treino (\S+)", out)
    return m.group(1) if m else None


def main():
    S.mkdir(parents=True, exist_ok=True)
    only = None
    if "--testes" in sys.argv:
        only = {int(x) for x in sys.argv[sys.argv.index("--testes") + 1].split(",")}
    state = json.loads(STATE.read_text()) if STATE.exists() else {"base_ck": str(MAIN / "final_15h_r130240.pt"), "base_treino": "cerebro_unico",
                                                                    "accepted": [], "done": []}

    def measure_base():
        log(f"medindo a referencia: {Path(state['base_ck']).name}")
        state["base"] = measure(state["base_treino"], Path(state["base_ck"]), "base")
        STATE.write_text(json.dumps(state, indent=1))
        log("referencia: " + json.dumps(state["base"]))

    if "--so-medir" in sys.argv:
        measure_base()
        return
    if "base" in state and state["base"].get("j_maos") is not None and state["base"].get("j30_maos") is None:
        # (18h10) A base medida antes da suavidade a 30 Hz: os numeros dela da linha JOGO guardada (a regua e deterministica).
        m = re.match(r"ab16_(\d+)_", state["base_treino"])
        f = S / f"jogo_b{m.group(1)}.txt" if m else None
        vals = parse_jogo(JOGO_CAMPEAO) if state["base_treino"] == "cerebro_unico" else parse_jogo(f.read_text()) if f and f.exists() else {}
        state["base"].update({k: v for k, v in vals.items() if k.startswith("j30_") and v is not None})
        STATE.write_text(json.dumps(state, indent=1))
        log("referencia a 30 Hz: " + json.dumps({k: v for k, v in state["base"].items() if k.startswith("j30_")}))
    if "base" in state and state["base"].get("j_torto_trote") is None:
        # (21h) O torto da base: a linha TORTO da regua da base (a mesma regua deterministica; a da r131350 medida as 20h55).
        f = Path(os.environ.get("TORTO_BASE", str(S / "jogo/torto_b9/regua.txt")))
        if f.exists():
            line = "JOGO | " + next((x for x in f.read_text().splitlines() if x.startswith("TORTO")), "")
            state["base"].update({k: v for k, v in parse_jogo(line).items() if k.startswith(("j_torto", "j_peito")) and v is not None})
            STATE.write_text(json.dumps(state, indent=1))
            log("referencia do torto: " + json.dumps({k: v for k, v in state["base"].items() if k.startswith(("j_torto", "j_peito"))}))
    if "base" in state and state["base"].get("juiz_loco_margem") is None and state["base_treino"] != "cerebro_unico":
        state["base"].update(juiz(state["base_treino"]))
        STATE.write_text(json.dumps(state, indent=1))
        log("referencia dos juizes: " + json.dumps({k: v for k, v in state["base"].items() if k.startswith("juiz")}))
    if "base" in state and state["base"].get("dist_loco") is None and os.environ.get("DIST_BASE"):
        # (21h30) A distancia ao clipe da base, medida a parte (distancia_clipe_loco.py; a mesma regua da checagem).
        txt = Path(os.environ["DIST_BASE"]).read_text()
        state["base"]["dist_loco"] = num(r"DISTLOCO media ([0-9.]+)", txt)
        for k in ("trote_0", "corrida_0", "corrida_1", "trote_2", "trote_4", "corrida_4"):
            state["base"]["dist_" + k] = num(r"lev_%s: distancia ao clipe ([0-9.]+) cm" % k, txt)
        STATE.write_text(json.dumps(state, indent=1))
        log(f"referencia da distancia ao clipe: {state['base']['dist_loco']} cm")
    if "base" in state and state["base"].get("j_maos") is None:
        log(f"a regua do jogo da referencia: {Path(state['base_ck']).name}")
        state["base"].update(measure_jogo(state["base_treino"], Path(state["base_ck"]), "base_jogo"))
        STATE.write_text(json.dumps(state, indent=1))
        log("referencia no jogo: " + json.dumps({k: v for k, v in state["base"].items() if k.startswith("j_")}))
    base_thread = None
    for n, name, desc, mod, target, target_desc in TESTS:
        if n in state["done"] or (only and n not in only):
            continue
        treino = f"ab16_{n}_{name}"
        # (17h50) A build do braco: a dele, ou a do ultimo aprovado que pedia uma (as builds novas so somam ajustes desligados).
        build = mod.get("build") or next((m["build"] for m in reversed(state["accepted"]) if m.get("build")), BUILD)
        run = RUNS / treino
        unit = "rl-ab16"
        if unit_active(unit) and unit_treino(unit) == treino:
            # (18h10) Retomado: o orquestrador foi reiniciado com o braco treinando - nada de copiar o ponto nem relancar; o relogio
            # conta da receita gravada no lancamento.
            t0 = (run / "receita.json").stat().st_mtime
            log(f"braco {n} ({name}) retomado: treinando desde {datetime.fromtimestamp(t0).strftime('%H:%M')}")
        else:
            run.mkdir(parents=True, exist_ok=True)
            # (18h45) O exports antes do lancamento: o 2b caiu no comeco ao gravar exports/episodios_levantar.npy sem a pasta.
            (run / "exports").mkdir(exist_ok=True)
            # (18h30) O ponto de partida: o do braco (partida - continuar um braco) ou a melhor rede; a ancora segue na melhor rede.
            partida = mod.get("partida") or (mod.get("partida_se_campeao") if state["base_treino"] == "cerebro_unico" else None) or state["base_ck"]
            if not Path(partida).exists():
                log(f"braco {n}: a partida {partida} nao existe - parte da melhor rede")
                partida = state["base_ck"]
            shutil.copy(partida, run / "ultimo.pt")
            r = recipe(state["base_ck"], state["accepted"], mod)
            r["receita"] = treino
            r["descricao"] = f"16/11, braco {n}: {desc}, da {Path(partida).name}"
            json.dump(r, open(run / "receita.json", "w"), indent=1, ensure_ascii=False)
            subprocess.run(["systemctl", "--user", "reset-failed", unit], capture_output=True)
            out = subprocess.run(["bash", str(G / "lancar.sh"), unit, str(run / "receita.json"), treino, build, THREADS, "0", SOCKET],
                                 capture_output=True, text=True)
            log(f"braco {n} ({name}) comecou: {(out.stdout + out.stderr).strip()[-100:]}")
            t0 = time.time()
        # (A referencia mede enquanto o primeiro braco treina.)
        if "base" not in state and base_thread is None:
            base_thread = threading.Thread(target=measure_base)
            base_thread.start()
        died = False
        while time.time() - t0 < TREINO_MIN * 60:
            time.sleep(30)
            if not unit_active(unit):
                died = True
                break
        subprocess.run(["systemctl", "--user", "stop", unit], capture_output=True)
        time.sleep(5)
        if base_thread is not None:
            base_thread.join()
            base_thread = None
        if died:
            log(f"braco {n}: o treino caiu antes do tempo (ver {run}/treino.out) - registrado, segue")
            state["done"].append(n); STATE.write_text(json.dumps(state, indent=1))
            continue
        ck = run / f"braco{n}_final.pt"
        shutil.copy(run / "ultimo.pt", ck)
        t = measure(treino, ck, f"b{n}", build)
        t.update(measure_jogo(treino, ck, f"b{n}", build))
        b = state["base"]
        bad = regressions(b, t, tuple(mod.get("ignorar", ()))) + regressions_jogo(b, t)
        hit = target(b, t)
        ok = hit and not bad
        log(f"braco {n} ({name}): alvo [{target_desc}] {'ATINGIDO' if hit else 'nao atingido'}; pioras: {', '.join(bad) if bad else 'nenhuma'}"
            f" -> {'PASSOU' if ok else 'NAO PASSOU'}")
        log("  numeros: " + json.dumps({k: t[k] for k in t}))
        if ok and (S / "sem_promover").exists():
            # (22h30) Ele esta olhando no laboratorio uma rede que pediu (o braco 14): aprovado, guardado e anotado, mas sem trocar o
            # laboratorio, o jogo nem a base da fila ate ele decidir (apagar o arquivo sem_promover libera).
            log(f"  -> aprovado, mas SEM PROMOVER (ele esta vendo o braco 14 no laboratorio): o ponto em {ck}")
            ok = False
        if ok:
            state["base_ck"] = str(ck); state["base_treino"] = treino; state["base"] = t; state["accepted"].append(mod)
            lab = MAIN / "exports/policy_latest.mrlpolicy"
            arch = MAIN / "exports/arquivo_ab16"; arch.mkdir(exist_ok=True)
            shutil.copy(lab, arch / f"policy_antes_braco{n}_{datetime.now().strftime('%H%M')}.mrlpolicy")
            # (17h50) O run.json do braco no principal tambem: o jogo e o laboratorio leem dele as chaves da rede (ex.: a velocidade depois
            # do levantar correndo, o ambiente de verdade); o anterior guardado.
            shutil.copy(MAIN / "run.json", arch / f"run_antes_braco{n}_{datetime.now().strftime('%H%M')}.json")
            shutil.copy(run / "run.json", MAIN / "run.json")
            e = subprocess.run(["nice", "-n", "15", P, str(G / "exportar.py"), treino, str(ck), str(lab)], capture_output=True, text=True,
                               cwd=str(W / "tools/rl"), env=dict(os.environ, MATTER_TRACKING_NATIVE=str(W / build / "python"),
                                                                 TMPDIR=str(W / "build-profile/tmp")))
            log(f"  -> no laboratorio e no jogo: {(e.stdout + e.stderr).strip()[-120:]}")
        state["done"].append(n); STATE.write_text(json.dumps(state, indent=1))
    log(f"FIM desta rodada. A melhor rede: {state['base_ck']} (aprovados: {len(state['accepted'])})")


if __name__ == "__main__":
    main()
