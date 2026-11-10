"""O que as ferramentas de inspecao compartilham: os caminhos, o treino (run.json e o checkpoint), o lote de cada exercicio
como no treino, a politica, a guarda termica e o servico do treino."""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

RL = Path(__file__).resolve().parents[1]
REPO = RL.parents[1]
sys.path.insert(0, str(RL))
os.environ.setdefault("MATTER_TRACKING_NATIVE", str(REPO / "build-rl-n1" / "python"))
os.environ.setdefault("MATTER_RL_NATIVE", str(REPO / "build-rl-n1" / "python"))
os.environ.setdefault("TMPDIR", str(REPO / "build-profile" / "tmp"))

RUNS = Path.home() / ".local/share/matter-engine/rl/runs"
SERVICE = "matter-rl-final-ajuda"
RAGDOLL = REPO / "assets/characters/football_player/FootballPlayerV1.ragdoll.json"
INSPECTOR = REPO / "build-profile" / "MatterModelInspector"
# As colunas dos episodios terminados (bindings/python/MatterTrackingNative.cpp, finished_episodes).
EPISODE_COLUMNS = ("mundo", "comeco", "lado", "cabeca_comeco", "segundos", "de_pe_em", "tarefa", "cabeca", "pelve", "de_pe",
                   "pedido", "chegou_em", "vel_alvo", "caiu_de_pe", "recompos_em", "pose_fim", "erro_rosto", "coluna_fim",
                   "entrega_em", "parceiro_caiu", "parceiro_s", "tocou_chao", "apoiou", "flexao", "obstruido", "faixa",
                   "cenario", "professora", "tomadas", "rotuladas", "ajuda_professora", "desvio_rumo", "fim_causa",
                   "semelhanca", "estabilidade", "passos_rec", "passos_estab", "equilibrio", "empurroes", "captura_fora",
                   "quietou_em", "chegada", "cruzado", "estrela_pernas", "estrela_fase", "caiu_no_empurrao",
                   "erro_parada")
MANEUVERS = ("lev_troca_suave", "lev_troca_media", "lev_troca_forte", "lev_troca_livre", "lev_curva_suave", "lev_curva_s",
             "lev_curva_fechada")
RUN_CURVES = ("lev_curva_suave_corrida", "lev_curva_s_corrida", "lev_curva_fechada_corrida")
# Os pares espelhados (o clipe de um e o espelho do outro): a diferenca entre eles e o vies da rede.
MIRROR_PAIRS = (("lev_trote_2", "lev_trote_6"), ("lev_trote_1", "lev_trote_7"), ("lev_trote_3", "lev_trote_5"),
                ("lev_corrida_2", "lev_corrida_6"), ("lev_corrida_1", "lev_corrida_7"), ("lev_corrida_3", "lev_corrida_5"))


def mirror_of(key: str) -> str | None:
    for a, b in MIRROR_PAIRS:
        if key == a:
            return b
        if key == b:
            return a
    return None


def run_dir(treino: str) -> Path:
    return RUNS / treino


def run_v2(treino: str) -> dict:
    return json.loads((run_dir(treino) / "run.json").read_text()).get("v2", {})


def ragdoll() -> dict:
    """O perfil fisico do jogador: os elos (id, pai, a capsula ou a casca e a junta) - as medidas para desenhar e analisar."""
    data = json.loads(RAGDOLL.read_text())
    links = []
    for link in data["links"]:
        shape = "capsule" if "capsule" in link else "hull" if "hull" in link else "box" if "box" in link else "?"
        geo = link.get(shape, {})
        entry = {"id": link["id"], "pai": int(link.get("parent", -1)), "forma": shape, "massa": float(link.get("massFraction", 0.0))}
        if shape == "capsule":
            entry.update(comprimento=float(geo.get("length", 0.1)), raio=float(geo.get("radius", 0.05)),
                         raio_mais_x=float(geo.get("radiusAtPositiveX", geo.get("radius", 0.05)) or geo.get("radius", 0.05)))
        elif shape == "hull":
            pts = np.array(geo.get("points", [[0, 0, 0]]), float)
            entry.update(meia_caixa=((pts.max(0) - pts.min(0)) / 2).tolist(), centro_caixa=((pts.max(0) + pts.min(0)) / 2).tolist(),
                         pontos=pts.tolist())
        elif shape == "box":
            entry.update(meia_caixa=list(geo.get("halfExtents", [0.05, 0.05, 0.05])))
        links.append(entry)
    return {"links": links, "massa_total": float(data.get("totalMassKg", 75.0))}


# --- O servico do treino e a guarda termica -------------------------------------------------------------------------

def service_active() -> bool:
    return subprocess.run(["systemctl", "--user", "is-active", SERVICE], capture_output=True, text=True).stdout.strip() == "active"


def temperatures() -> tuple[float, float]:
    """(CPU Tctl, GPU) em graus C; nan quando nao der para ler."""
    cpu = gpu = float("nan")
    try:
        out = subprocess.run(["sensors"], capture_output=True, text=True, timeout=10).stdout
        m = re.search(r"Tctl:\s*\+?([0-9.]+)", out)
        cpu = float(m.group(1)) if m else cpu
    except Exception:
        pass
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"], capture_output=True,
                             text=True, timeout=10).stdout
        gpu = float(out.strip().splitlines()[0])
    except Exception:
        pass
    return cpu, gpu


def wait_cool(limit: float = 70.0, timeout: float = 300.0, log=print) -> float:
    """Espera a CPU ficar abaixo de `limit` (a regra termica: um trabalho pesado por vez e esfriar entre eles)."""
    start = time.time()
    cpu, _ = temperatures()
    while cpu == cpu and cpu >= limit and time.time() - start < timeout:
        time.sleep(5)
        cpu, _ = temperatures()
    log(f"CPU {cpu:.1f} C")
    return cpu


def require_stopped(force: bool = False) -> None:
    """A regra termica: sonda, captura ou folha so com o treino parado."""
    if service_active() and not force:
        raise SystemExit(f"o treino ({SERVICE}) esta ativo: pare antes (systemctl --user stop {SERVICE}) ou use o orquestrador")


def last_log_row(treino: str) -> dict | None:
    path = run_dir(treino) / "log.jsonl"
    if not path.exists():
        return None
    with open(path, "rb") as f:
        f.seek(0, 2)
        size = f.tell()
        f.seek(max(0, size - 200000))
        lines = f.read().decode(errors="ignore").splitlines()
    for line in reversed(lines):
        try:
            return json.loads(line)
        except Exception:
            continue
    return None


def stop_training(log=print) -> None:
    if service_active():
        log(f"parando o treino ({SERVICE})")
        subprocess.run(["systemctl", "--user", "stop", SERVICE], check=False)
    if service_active():
        raise SystemExit("o treino nao parou")


def start_training(treino: str, log=print, timeout: float = 420.0) -> int:
    """Religa e espera uma linha nova no log (a regra: confirmar que escreve). Devolve a atualizacao da linha nova."""
    before = last_log_row(treino)
    before_update = before.get("atualizacao", -1) if before else -1
    subprocess.run(["systemctl", "--user", "start", SERVICE], check=False)
    start = time.time()
    while time.time() - start < timeout:
        time.sleep(5)
        if not service_active():
            raise SystemExit("o treino caiu ao religar: veja o treino.out")
        row = last_log_row(treino)
        if row and row.get("atualizacao", -1) > before_update:
            log(f"treino religado: linha nova r{row['atualizacao']}")
            return int(row["atualizacao"])
    raise SystemExit("o treino religou mas nao escreveu linha nova a tempo")


# --- O lote de cada exercicio, como no treino ------------------------------------------------------------------------

class Ambiente:
    """O treino carregado: o modulo nativo, as configuracoes do run.json e o checkpoint."""

    def __init__(self, treino: str = "rl_final_clipe", checkpoint: str | None = None):
        # (08/11, pedido dele: a ferramenta freia o treino enquanto roda - tools/rl/freio_treino.py.)
        import freio_treino
        freio_treino.acquire()
        import torch
        import sondas_rl_final as S
        self.S, self.L, self.torch = S, S.L, torch
        self.treino = treino
        self.v2 = run_v2(treino)
        S.RUN_V2.update(self.v2)
        # (08/11: desde 06/11 a rede livre - residuo 0, --acao-velocidade, 82 saidas - nao carregava aqui: o lote saia com 43
        # saidas e sempre como residuo do clipe. Agora o lote como o do treino.)
        self.L.VELOCITY_ACTIONS[0] = bool(self.v2.get("acao_velocidade", False))
        S.TRACK["locomotion_residual"] = self.v2.get("residuo") is None or float(self.v2["residuo"]) > 0
        S.REWARD[0] = int(self.v2.get("recompensa_rastreio", 13))
        self.L.BATCH_EXTRA.update(self.L.batch_layout(self.v2))
        self.L.GETUP_CLIP[0] = bool(self.v2.get("levantar_clipe", False))
        path = Path(checkpoint) if checkpoint else run_dir(treino) / "ultimo.pt"
        self.checkpoint = path
        self.state = torch.load(path, map_location="cpu", weights_only=False)
        self.update = int(self.state["counters"].get("atualizacao", -1))
        self.levels = {k: float(v) for k, v in self.state["counters"].get("ajuda_exercicios", {}).items()}
        torch.set_num_threads(2)
        self.residual = 0.15 if self.v2.get("residuo") is None else float(self.v2["residuo"])
        parts = self.v2.get("residuo_partes") or (0.18, 0.08, 0.05)
        self.parts = tuple(float(x) for x in parts)

    def level(self, key: str) -> float:
        # (As trocas correndo tem a forca da troca: o treino guarda uma por manobra, nas duas faixas.)
        # (06/11 00h25: so as trocas - as curvas de corrida, "lev_curva_s_corrida", sao exercicios proprios e cortar o sufixo
        # pegava a forca da curva do trote.)
        if key.startswith("lev_troca_") and key.endswith("_corrida"):
            key = key[:-len("_corrida")]
        return float(self.levels.get(key, 1.0))

    def batch(self, key: str, worlds: int, seed: int = 61):
        """O lote da avaliacao de um exercicio (como portao_100/teto_semelhanca): os rumos, as manobras, o giro, o parado, o
        sprint e o levantar."""
        L, S = self.L, self.S
        A = S.ASSETS
        if key == "lev_parado":
            return L.get_up_batch(A, environments=worlds, threads=4, seed=seed, clip_start_fraction=0.0, standing_start_fraction=1.0,
                                  moving_fraction=0.0, avoid_fraction=0.0, locomotion_fraction=0.0, turn_fraction=0.0,
                                  episode_seconds=6.0, **S.TRACK)
        if key == "lev_levantar":
            return L.get_up_batch(A, environments=worlds, threads=4, seed=seed, evaluation=True, evaluation_kind=0, **S.TRACK)
        if key == "lev_giro":
            return L.get_up_batch(A, environments=worlds, threads=4, seed=seed, evaluation=True, evaluation_kind=6, **S.TRACK)
        # (05/11 21h: as trocas tambem correndo - "lev_troca_media_corrida"; no treino elas caem na corrida, nao no trote.)
        run_maneuver = key.startswith("lev_troca_") and key.endswith("_corrida")
        if run_maneuver:
            key = key[:-len("_corrida")]
        if key in MANEUVERS or key in RUN_CURVES:
            code = (MANEUVERS.index(key) + 1) if key in MANEUVERS else (RUN_CURVES.index(key) + 5)
            b = L.get_up_batch(A, environments=worlds, threads=4, seed=seed, evaluation=True, evaluation_kind=4,
                               evaluation_band=2 if key in RUN_CURVES or run_maneuver else 1, **S.TRACK)
            b.set_maneuvers(0.0, code)
            return b
        # (08/11 14h40: o comando - o jogador emulado -, na fase do treino ou na dada, lev_comando_f3.)
        m = re.fullmatch(r"lev_comando(?:_f([1-4]))?", key)
        if m:
            b = L.get_up_batch(A, environments=worlds, threads=4, seed=seed, evaluation=True, evaluation_kind=4, evaluation_band=1,
                               **S.TRACK)
            b.set_maneuvers(0.0, 12)
            b.set_command(0.0, int(m.group(1) or self.v2.get("comando_fase", 1)), float(self.v2.get("comando_segundos", 20.0)))
            return b
        # (08/11: a estrela - lev_estrela_<trote|corrida>_<fixo|livre|girando>, na fase do treino ou na dada.)
        m = re.fullmatch(r"lev_estrela_(trote|corrida)_(fixo|livre|girando)(?:_f([1-4]))?", key)
        if m:
            code = 8 + ("fixo", "livre", "girando").index(m.group(2))
            phases = [int(x) for x in self.v2.get("estrela_fases", [1] * 6)]
            if m.group(3):
                phases = [int(m.group(3))] * 6
            b = L.get_up_batch(A, environments=worlds, threads=4, seed=seed, evaluation=True, evaluation_kind=4,
                               evaluation_band=2 if m.group(1) == "corrida" else 1, **S.TRACK)
            b.set_maneuvers(0.0, code)
            b.set_star(0.0, [1.0] * 6, phases, float(self.v2.get("estrela_segundos", 14.0)))
            return b
        if key == "lev_sprint":
            return L.get_up_batch(A, environments=worlds, threads=4, seed=seed, evaluation=True, evaluation_kind=4, evaluation_band=3,
                                  heading=0, **S.TRACK)
        m = re.fullmatch(r"lev_(trote|corrida)_([0-7])", key)
        if not m:
            raise SystemExit(f"exercicio desconhecido: {key}")
        return L.get_up_batch(A, environments=worlds, threads=4, seed=seed, evaluation=True, evaluation_kind=4,
                              evaluation_band=2 if m.group(1) == "corrida" else 1, heading=int(m.group(2)), **S.TRACK)

    def setup(self, b, level: float) -> None:
        self.S.setup(b, [level] * len(self.L.DRILL_KEYS))

    def model(self, b):
        """A politica do checkpoint (como sondas_rl_final.policy), devolvendo o modelo para o ruido e as acoes."""
        L, torch = self.L, self.torch
        model = L.Policy(b.observations().shape[1], b.action_size, b.action_ranges(), split=True)
        model.load_state_dict(self.state["model"])
        if float(self.v2.get("ruido_parado", 1.0)) < 1.0:
            L.configure_idle_noise(model, b, float(self.v2["ruido_parado"]))
        model.residual = self.residual
        spine, arms, head = self.parts
        model.joint_names = tuple(n for n in b.action_names() if not n.startswith("passo."))
        L.configure_threat(model, b, float(self.v2.get("autoridade_ameaca", 1.0)), float(self.v2.get("autoridade_demanda", 1.0)))
        model.residual_joint = torch.as_tensor([
            1.0 if n.startswith("passo.") else spine if n.split(".")[0] in ("Abdomen", "Chest", "UpperChest")
            else arms if ("Arm" in n or "Hand" in n) else head if n.split(".")[0] in ("Neck", "Head") else self.residual
            for n in b.action_names()])
        model.eval()
        return model


def parse_lines(text: str) -> tuple[dict, list, dict]:
    """As linhas do frame_lines/reference_lines: {elo: [x y z qx qy qz qw]}, os contatos [(x y z elo nx ny nz impulso)] e a
    linha Q (clipe, instante, forca, controle, pedido, rumo)."""
    links, contacts, q = {}, [], {}
    for line in text.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] in ("B", "C"):
            links[p[1]] = [float(x) for x in p[2:9]]
        elif p[0] == "P":
            contacts.append([float(x) for x in p[1:9]])
        elif p[0] == "Q":
            q = {"clipe": p[1], "t": float(p[2]), "forca": float(p[3]), "controle": int(p[4]), "pedido": float(p[5]),
                 "rumo": float(p[6])}
    return links, contacts, q
