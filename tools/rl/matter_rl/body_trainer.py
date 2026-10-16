"""O treino do Corpo v2 (plano_claude_rl.md): o lote v2 (o contrato v2, o piloto
e as notas novas), o PPO com as redes do plano, o juiz de estilo (AMP) com o
botao beta e a atualizacao da GPU sobreposta a coleta seguinte.

O banco, o canal de controle, os checkpoints e a exportacao para o C++ ficam
nos modulos de apoio (storage, control, model).

A sobreposicao (Experimento A: a GPU leva ~0,6 s por rodada, quase o mesmo que
a fisica): enquanto a GPU atualiza com a rodada k, a fisica coleta a k+1 com a
politica de antes (uma copia). O PPO ja guarda as probabilidades de quem
coletou, entao a conta continua certa, uma rodada fora da politica.
"""
from __future__ import annotations

import concurrent.futures
import copy
import json
import math
import os
import random
import signal
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import torch

from . import control, native, paths, storage, system
from .amp import Judge, JudgeSettings, style_score
from .model import ActorCritic, RunningNorm, export_policy, migrate_inputs
from .ppo import PPO, PPOSettings, Rollout
from .symmetry import MirrorLoss

RESTART_CODE = system.RESTART_CODE
TASK_SCHEMA = "matter-rl-task-2"
NETWORK = {"actor": [1024, 512, 256], "critic": [1024, 512, 256]}
# Os canais que passam crus pela normalizacao: os comandos e os dados do
# jogador (o contrato ja os escala) e, no critico, o exercicio e a camera.
RAW_PREFIXES = ("command.", "player.", "drill.", "pilot.", "world.solver")

# Os ajustes do PPO do Corpo v2 (plano de recuperacao, 5.3): o critico a parte,
# a taxa do ator entre 1e-6 e 3e-4 pelo KL, a parada das atualizacoes do ator
# quando o KL passa de 3x o alvo e sem bonus de entropia (o desvio e fixo).
BODY_PPO = {"separate_critic": True, "critic_learning_rate": 3e-4, "critic_max_grad_norm": 1.0, "kl_stop_factor": 3.0,
            "learning_rate": 5e-6, "min_learning_rate": 1e-6, "max_learning_rate": 3e-4, "entropy_coef": 0.0,
            # O alvo do rsl_rl: com 37 eixos o recorte ja chega a ~0,4 com KL 0,02 (a razao soma 37 termos).
            "desired_kl": 0.01}
# O desvio fixo da exploracao por grupo de eixos (rad; plano 5.3). O tronco em 0,02 e o resultado do D0
# (14/10): com 0,03 o corpo com ruido ficava a 1,64x da distancia do corpo sem ruido ao acervo; com 0,02, 1,44x.
FIXED_SIGMA = {"legs": 0.06, "toes": 0.06, "arms": 0.05, "trunk": 0.02}


def axis_group(action_name: str) -> str:
    """O grupo de um eixo atuado pelo nome (LeftThigh.swing1.target -> legs)."""
    link = action_name.split(".")[0]
    if "Toe" in link:
        return "toes"
    if any(part in link for part in ("Thigh", "Shin", "Foot")):
        return "legs"
    if any(part in link for part in ("Arm", "Forearm", "Hand")):
        return "arms"
    return "trunk"


def initial_std(action_names: list[str], radians_per_unit: np.ndarray, exploration: dict) -> np.ndarray:
    """O desvio inicial da acao (em unidades): fixo por grupo (rad / rad por
    unidade) ou, no modo aprendido, 0,15 rad em todos os eixos como no v1."""
    if exploration.get("mode", "fixed") == "fixed":
        sigma = {**FIXED_SIGMA, **exploration.get("sigma", {})}
        rad = np.array([sigma[axis_group(name)] for name in action_names], dtype=np.float32)
        return np.clip(rad / radians_per_unit, 0.02, 2.0)
    return np.clip(float(exploration.get("initial_rad", 0.15)) / radians_per_unit, 0.1, 1.5)


def is_body_task(path: Path) -> bool:
    try:
        return json.loads(path.read_text()).get("schema") == TASK_SCHEMA
    except (OSError, ValueError):
        return False


def is_body_run(run: str) -> bool:
    config = paths.run_dir(run) / "run.json"
    try:
        return config.exists() and int(json.loads(config.read_text()).get("contract", 1)) == 2
    except (OSError, ValueError):
        return False


@dataclass
class BodyRunConfig:
    contract: int = 2
    task: str = "corpo2"
    environments: int = 512
    seed: int = 1
    mode: str = "equilibrado"
    # Por que o ultimo treino fechou ("usuario" impede a retomada automatica).
    end_reason: str = ""
    # O juiz de estilo (B2) ou so a tarefa e a fisica (B1); escolhido ao criar.
    style: bool = True
    # A coleta e a atualizacao em serie (o PPO verificavel): a sobreposicao so
    # volta depois do P3, com o teste de equivalencia (plano 5.1).
    overlap: bool = False
    # O peso da perda do espelho no ator (0 = desligada; o plano compara com e
    # sem depois do Experimento C).
    mirror_loss: float = 0.0
    checkpoint_minutes: float = 10.0
    export_minutes: float = 1.0
    # A avaliacao: cada exercicio com a acao media (como no jogo), em mundos
    # fixos, a cada evaluation_minutes (0 = nunca), no solver do jogo.
    evaluation_minutes: float = 10.0
    evaluation_worlds: int = 32
    evaluation_seconds: float = 20.0
    disk_cap_gb: float = 10.0
    ppo: dict = field(default_factory=lambda: {**PPOSettings().to_dict(), **BODY_PPO})
    # A exploracao: desvio fixo por grupo de eixos ("fixed", com sigma em rad
    # por grupo) ou aprendido ("learned", initial_rad).
    exploration: dict = field(default_factory=lambda: {"mode": "fixed", "sigma": dict(FIXED_SIGMA)})
    # Depois de trocar o juiz (a transferencia entre degraus): rodadas so do
    # juiz e do critico antes de o ator voltar a aprender (minimo, maximo; sai
    # antes se a variancia explicada do critico passa de 0,5 depois do minimo).
    warmup_rounds: int = 0
    warmup_min_rounds: int = 20
    # De onde o treino nasceu (a transferencia entre degraus), para o manifesto.
    transferred_from: str = ""
    # O corte do recorte: com a media acima de 0,35 por 5 rodadas, 3 epocas.
    clip_streak_limit: int = 5
    # O ganho fixo do canal da velocidade pedida (command.move.*, que entra como v/6): 3
    # leva 3 m/s a 1,5. Sem ele, a rede nao usava o comando (14/10: a acao media
    # mudava 0,002 rad do pedido 0,5 ao 3,0). 1 = o antigo (treinos de antes).
    command_gain: float = 3.0
    # O curriculo da velocidade (a tarefa traz curriculum.speedMax e speedCap): sobe
    # curriculum_step quando a nota de progresso media passa de curriculum_threshold
    # em curriculum_rounds rodadas seguidas e ha ao menos curriculum_gap rodadas
    # desde a ultima subida (a regra do v3-v6).
    curriculum_threshold: float = 0.7
    curriculum_step: float = 0.25
    curriculum_rounds: int = 3
    curriculum_gap: int = 50
    network: dict = field(default_factory=lambda: dict(NETWORK))
    judge: dict = field(default_factory=lambda: JudgeSettings().to_dict())

    @staticmethod
    def load(path: Path) -> "BodyRunConfig":
        data = json.loads(path.read_text())
        known = {k: v for k, v in data.items() if k in BodyRunConfig.__dataclass_fields__}
        if "command_gain" not in known:
            known["command_gain"] = 1.0
        if "exploration" not in known:
            # Um treino de antes do desvio fixo aprendia o desvio: continua assim.
            known["exploration"] = {"mode": "learned", "initial_rad": 0.15}
        return BodyRunConfig(**known)

    def save(self, path: Path) -> None:
        storage.write_json_atomic(path, asdict(self))


def choose_device(preference: str = "auto") -> torch.device:
    if preference == "cpu" or not torch.cuda.is_available():
        return torch.device("cpu")
    try:
        x = torch.randn(64, 64, device="cuda")
        float((x @ x).sum())
        return torch.device("cuda")
    except Exception:  # noqa: BLE001 - sem kernel para a GPU: segue na CPU
        return torch.device("cpu")


def wilson_upper(failures: int, total: int, z: float = 1.96) -> float:
    """O limite de cima (95%) da fatia de falhas: 0 em 32 ainda e ~11%."""
    if total <= 0:
        return 1.0
    p = failures / total
    center = p + z * z / (2 * total)
    margin = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total))
    return min(1.0, (center + margin) / (1 + z * z / total))


def drill_summary(rows: np.ndarray, columns: list[str], drills: list[str]) -> dict:
    """As medidas dos episodios que acabaram, por exercicio."""
    out: dict = {}
    if not len(rows):
        return out
    col = {name: i for i, name in enumerate(columns)}
    for index, drill in enumerate(drills):
        part = rows[rows[:, col["exercicio"]] == index]
        if not len(part):
            continue
        reasons = part[:, col["razao"]].astype(int)
        reaction = part[:, col["reacao"]]
        reaction = reaction[reaction >= 0.0]
        falls = int(np.isin(reasons, [1, 2]).sum())
        velocity = part[:, col["erro_velocidade"]]
        out[drill] = {
            "episodes": int(len(part)),
            "falls": falls / len(part),
            "falls_upper95": wilson_upper(falls, len(part)),
            "velocity_error_median": float(np.median(velocity)),
            "velocity_error_p90": float(np.percentile(velocity, 90)),
            "seconds": float(part[:, col["segundos"]].mean()),
            "velocity_error": float(part[:, col["erro_velocidade"]].mean()),
            "velocity_score": float(part[:, col["nota_velocidade"]].mean()),
            "facing_error": float(part[:, col["erro_olhar"]].mean()),
            "reaction": float(reaction.mean()) if len(reaction) else None,
            "slip": float(part[:, col["escorregar"]].mean()),
            "work": float(part[:, col["trabalho"]].mean()),
            "saturation": float(part[:, col["no_teto"]].mean()),
            "reversal": float(part[:, col["reversao"]].mean()),
            "cadence": float(part[:, col["cadencia"]].mean()),
            # O passo util (plano 5.9): por segundo, a partida (so entre os que
            # deram um passo, com a fatia deles), a sequencia e a caminhada
            # sustentada (>= 6 passos seguidos e 8 s sem cair).
            "useful_steps": float(part[:, col["passos_uteis"]].mean()),
            "first_step_median": float(np.median(part[part[:, col["primeiro_passo"]] >= 0, col["primeiro_passo"]]))
            if (part[:, col["primeiro_passo"]] >= 0).any() else None,
            "first_step_fraction": float((part[:, col["primeiro_passo"]] >= 0).mean()),
            "step_run_median": float(np.median(part[:, col["sequencia"]])),
            "sustained": float(((part[:, col["sequencia"]] >= 6) & (part[:, col["segundos"]] >= 8.0)
                                & ~np.isin(reasons, [1, 2])).mean()),
            "requested": float(part[:, col["pedido_medio"]].mean()),
            "stand_gate": float(part[:, col["porta_media"]].mean()),
            "both_feet_air": float(part[:, col["voo_duplo"]].mean()),
        }
        # A estrela (colunas que um modulo nativo antigo nao tem).
        if "alvos_por_minuto" in col:
            out[drill]["targets_per_minute"] = float(part[:, col["alvos_por_minuto"]].mean())
            drift = part[:, col["deriva_espera"]]
            drift = drift[drift >= 0.0]
            out[drill]["hold_drift"] = float(drift.mean()) if len(drift) else None
        if "postura" in col:
            out[drill]["posture"] = float(part[:, col["postura"]].mean())
        if "afastamento_curva" in col:
            arc = part[:, col["afastamento_curva"]]
            arc = arc[arc >= 0.0]
            out[drill]["arc_error"] = float(arc.mean()) if len(arc) else None
        if "largura_base" in col:
            width = part[:, col["largura_base"]]
            width = width[width > -1.0]
            out[drill]["stance_width"] = float(width.mean()) if len(width) else None
            out[drill]["feet_crossed"] = float(part[:, col["pes_cruzados"]].mean())
        if "parado_pes" in col:
            out[drill]["still_feet"] = float(part[:, col["parado_pes"]].mean())
        if "pivo_pe" in col:
            out[drill]["foot_pivot"] = float(part[:, col["pivo_pe"]].mean())
        if "tronco_cruzeiro" in col:
            # A postura funcional (graus; NaN = sem amostra no episodio).
            for key, name in (("trunk_cruise", "tronco_cruzeiro"), ("lower_trunk_cruise", "baixo_cruzeiro"),
                              ("trunk_back_fraction", "tronco_tras"), ("lower_trunk_back_fraction", "baixo_tras"),
                              ("trunk_braking", "tronco_freando"), ("trunk_accelerating", "tronco_acelerando"),
                              ("curve_inward", "curva_para_dentro")):
                values = part[:, col[name]]
                values = values[np.isfinite(values)]
                out[drill][key] = float(np.median(values)) if len(values) else None
    return out


def drill_difficulty(metrics: dict) -> float:
    """A dificuldade de um exercicio na avaliacao (0 = perfeito): as quedas mais o
    erro mediano de velocidade (1 m/s conta como cair sempre)."""
    return float(metrics.get("falls", 0.0)) + min(1.0, float(metrics.get("velocity_error_median", 0.0)))


def balance_drills(drills: list[dict], evaluation: dict, rule: dict) -> dict[str, float]:
    """As fatias seguem a dificuldade (exercicios_do_corpo_v2): o peso de cada
    exercicio e o de base vezes 1 + gain x (a dificuldade dele menos a do
    melhor), ate maximumFactor x o de base; depois nenhum fica abaixo de
    floor da soma. Sem medida, o exercicio fica no peso de base."""
    gain = float(rule.get("gain", 2.0))
    factor = float(rule.get("maximumFactor", 3.0))
    floor = float(rule.get("floor", 0.08))
    base = {d["id"]: float(d.get("baseWeight", d.get("weight", 1.0))) for d in drills}
    difficulty = {d: drill_difficulty(evaluation[d]) for d in base if d in evaluation}
    best = min(difficulty.values()) if difficulty else 0.0
    weights = {d: base[d] * min(factor, 1.0 + gain * (difficulty[d] - best)) if d in difficulty else base[d]
               for d in base}
    total = sum(weights.values())
    if total <= 0.0:
        return base
    # O piso: quem fica abaixo dele vai para o piso, e o resto se divide entre os
    # outros na proporcao deles (repete ate ninguem novo cair abaixo).
    floor = min(floor, 1.0 / len(weights))
    pinned: set[str] = set()
    while True:
        free = {d: w for d, w in weights.items() if d not in pinned}
        room = 1.0 - floor * len(pinned)
        free_total = sum(free.values())
        shares = {d: floor for d in pinned} | {d: room * w / free_total for d, w in free.items()}
        low = {d for d, share in shares.items() if share < floor - 1e-12 and d not in pinned}
        if not low:
            return {d: shares[d] * total for d in shares}
        pinned |= low


class BodyTrainer:
    def __init__(self, run: str, task: str | None = None, mode: str | None = None, environments: int | None = None,
                 device: str = "auto", seed: int | None = None, socket_path: Path | None = None,
                 iterations: int | None = None, style: bool | None = None, overlap: bool | None = None,
                 transfer_from: str | None = None, transfer: tuple = ("ator", "normalizacao"),
                 warmup: int | None = None, exploration: dict | None = None, mirror_loss: float | None = None,
                 command_gain: float | None = None):
        self.run = run
        self.directory = paths.run_dir(run)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.lock = storage.RunLock(self.directory)
        storage.clean_temporaries(self.directory)
        config_path = self.directory / "run.json"
        fresh = not config_path.exists()
        if not fresh and not is_body_run(run):
            raise RuntimeError(f"o treino {run} e do contrato 1: o Corpo v2 abre num treino novo")
        self.config = BodyRunConfig() if fresh else BodyRunConfig.load(config_path)
        if task:
            self.config.task = task
        if mode:
            self.config.mode = mode
        if environments:
            self.config.environments = environments
        if seed is not None:
            self.config.seed = seed
        # O juiz e escolhido ao criar (B1 ou B2); a sobreposicao pode mudar.
        if style is not None and fresh:
            self.config.style = style
        if overlap is not None:
            self.config.overlap = overlap
        if fresh and exploration is not None:
            self.config.exploration = exploration
        if fresh and command_gain is not None:
            self.config.command_gain = command_gain
        if fresh and mirror_loss is not None:
            self.config.mirror_loss = mirror_loss
        if fresh and warmup is not None:
            self.config.warmup_rounds = warmup
        elif fresh and transfer_from and self.config.style:
            # Uma transferencia com juiz novo: o ator espera o juiz e o critico
            # (20 a 60 rodadas; sai antes se o critico ja explica o retorno).
            self.config.warmup_rounds = 60
        if fresh and transfer_from:
            self.config.transferred_from = transfer_from
        self.config.save(config_path)
        self.seed_everything(self.config.seed)
        self.max_iterations = iterations

        self.mode = system.mode(self.config.mode)
        system.apply_priority(self.mode)
        self.threads = system.threads_for(self.mode)
        torch.set_num_threads(2)
        self.device = choose_device(device)
        self.native = native.load()
        self.task_path = paths.task_path(self.config.task)
        if not is_body_task(self.task_path):
            raise RuntimeError(f"a tarefa {self.config.task} nao e do Corpo v2 ({TASK_SCHEMA})")
        # Uma copia da tarefa no treino: os pesos mudados com ele andando (o
        # beta, os custos) ficam aqui.
        self.live_task = self.directory / "tarefa.json"
        source = json.loads(self.task_path.read_text())
        if self.live_task.exists() and json.loads(self.live_task.read_text()).get("id") != source.get("id"):
            raise RuntimeError("a tarefa deste treino e outra: o Corpo v2 abre num treino novo")
        if not self.live_task.exists():
            source["body"] = str((self.task_path.parent / source["body"]).resolve())
            style_section = source.setdefault("style", {})
            style_section.setdefault("dataset", "../style/acervo_corpo.json")
            style_section["dataset"] = str((self.task_path.parent / style_section["dataset"]).resolve())
            style_section.setdefault("beta", 1.0)
            storage.write_json_atomic(self.live_task, source)
        live = json.loads(self.live_task.read_text())
        self.beta = float(live.get("style", {}).get("beta", 1.0))
        # A nota se compoe aqui (o estilo so existe no treinador): ver compose_rewards.
        self.positive_only = bool(live.get("reward", {}).get("positiveOnly", False))
        self.batch = self.native.BodyBatch(str(self.live_task), environments=self.config.environments, per_scene=4,
                                           threads=self.threads, seed=self.config.seed)
        self.obs_names = self.batch.observation_names()
        self.critic_names = self.batch.critic_names()
        self.action_names = self.batch.action_names()
        self.reward_names = self.batch.reward_names()
        names = list(self.reward_names)
        self.ri = {name: names.index(name) for name in names}
        self.cost_index = [names.index(name) for name in
                           ("work", "torque", "slip", "impact", "head", "smooth", "limits", "contacts", "posture", "still", "athletic", "pivot")]
        self.episode_columns = self.batch.episode_columns()
        self.drill_ids = list(self.batch.drill_ids)
        # O juiz so em alguns exercicios (style.drills; vazio = todos): so os
        # mundos deles dao amostras ao juiz e recebem o estilo.
        style_drills = live.get("style", {}).get("drills", [])
        unknown = [d for d in style_drills if d not in self.drill_ids]
        if unknown:
            raise RuntimeError(f"style.drills cita exercicios que a tarefa nao tem: {unknown}")
        self.style_drills = np.array([self.drill_ids.index(d) for d in style_drills], dtype=np.int64)

        radians = np.array(self.batch.action_radians_per_unit(), dtype=np.float32)
        std0 = initial_std(self.action_names, radians, self.config.exploration)
        network = self.config.network
        self.model = ActorCritic(self.batch.obs_size, self.batch.critic_size, self.batch.action_size, std0,
                                 actor_hidden=network["actor"], critic_hidden=network["critic"]).to(self.device)
        # O desvio fixo: nao aprende e nao ha bonus de entropia (nada o puxa).
        self.fixed_sigma = self.config.exploration.get("mode", "fixed") == "fixed"
        if self.fixed_sigma:
            self.model.log_std.requires_grad_(False)
            self.config.ppo["entropy_coef"] = 0.0
        self.settings = PPOSettings.from_dict(self.config.ppo)
        self.ppo = PPO(self.model, self.settings)
        self.judge: Judge | None = None
        if self.config.style:
            dataset = self.native.motion_dataset(live["body"], live["style"]["dataset"])
            self.judge = Judge(dataset, JudgeSettings.from_dict(self.config.judge), self.device, self.config.seed)
        self.mirror = MirrorLoss(self.batch.mirror_maps(), self.device) if self.config.mirror_loss > 0.0 else None
        self.counters = {"iteration": 0, "steps": 0, "train_seconds": 0.0, "episodes": 0,
                         "warmup_left": int(self.config.warmup_rounds) if self.judge is not None else 0,
                         "warmup_done": 0, "curriculum_streak": 0, "curriculum_last": 0, "epochs": self.config.ppo.get("epochs", PPOSettings().epochs), "clip_streak": 0}
        self.db = storage.Database(paths.DATABASE)
        self.db.ensure_run(run, self.config.task, self.batch.body_hash, asdict(self.config))
        self.restore()
        self.ppo.settings.epochs = int(self.counters["epochs"])
        if transfer_from and self.counters["iteration"] == 0 and storage.latest_checkpoint(self.directory) is None:
            self.transfer_weights(transfer_from, transfer)
        self.fix_raw_channels()
        self.reset_candidates()
        # A copia que coleta (a politica de antes, durante a atualizacao) e a
        # copia do juiz que da a nota na coleta.
        self.collector = copy.deepcopy(self.model)
        self.scorer = copy.deepcopy(self.judge.net) if self.judge else None
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=1, initializer=self._init_worker) \
            if self.config.overlap else None
        self.stream = torch.cuda.Stream() if self.config.overlap and self.device.type == "cuda" else None

        self.state = "treinando"
        self.message = ""
        self.stop_requested = False
        self.stop_reason = "sinal"
        self.config.end_reason = ""
        self.config.save(config_path)
        self.restart_requested = False
        self.pause_requested = False
        self.started = time.time()
        self.recent_sps = 0.0
        self.last_iteration_stats: dict = {}
        self.last_judge: dict = {}
        self.last_evaluation: dict = {}
        self.evaluation_batches: dict = {}
        self.pending = None
        self.failures = 0
        self.session = self.db.begin_session(run, self.counters["iteration"], self.mode.name, str(self.device))
        self.db.event(run, "sessao", f"Corpo v2 aberto no modo {self.mode.label} ({self.threads} threads, "
                                     f"{self.device}; juiz {'ligado' if self.judge else 'desligado'}, beta {self.beta})")
        self.server = control.ControlServer(self.status, socket_path or paths.SOCKET)
        signal.signal(signal.SIGTERM, self._signal_stop)
        signal.signal(signal.SIGINT, self._signal_stop)

    # --- estado ------------------------------------------------------------------
    def _init_worker(self) -> None:
        # A thread da atualizacao com o contexto da GPU desde o comeco.
        if self.device.type == "cuda":
            torch.cuda.set_device(self.device.index or 0)

    def _signal_stop(self, *_):
        self.stop_requested = True
        self.stop_reason = "sinal"

    def fix_raw_channels(self) -> None:
        for norm, names in ((self.model.obs_norm, self.obs_names), (self.model.critic_norm, self.critic_names)):
            gain = torch.tensor([self.config.command_gain if name.startswith("command.move") else 1.0 for name in names])
            norm.fix_channels(torch.tensor([name.startswith(RAW_PREFIXES) for name in names]), gain)

    def seed_everything(self, seed: int) -> None:
        """A semente de cada gerador: Python, NumPy, PyTorch (CPU e CUDA). O lote
        e o juiz recebem a deles. Nao promete a mesma trajetoria bit a bit (o
        solver e os kernels da GPU), mas "semente 1" passa a ser uma
        aleatoriedade conhecida."""
        random.seed(seed)
        np.random.seed(seed % (2 ** 32))
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)

    def reset_candidates(self) -> None:
        """A estatistica candidata da normalizacao: acumula durante a coleta e so
        vale na fronteira da rodada (sync). Durante a coleta e a atualizacao a
        estatistica e a mesma, e o que o PPO guarda e o que a politica viu."""
        self.obs_candidate = copy.deepcopy(self.model.obs_norm)
        self.critic_candidate = copy.deepcopy(self.model.critic_norm)

    def resolve_checkpoint(self, source: str):
        """Um checkpoint de outro treino: "treino", "treino:rodada" ou o caminho de uma pasta de treino."""
        name, _, iteration = source.partition(":")
        directory = Path(name) if Path(name).is_dir() else paths.run_dir(name)
        references = storage.list_checkpoints(directory)
        if iteration:
            references = [r for r in references if r.iteration == int(iteration)]
        if not references:
            raise RuntimeError(f"nao achei checkpoint para a transferencia: {source}")
        return references[-1]

    def transfer_weights(self, source: str, parts) -> None:
        """A transferencia entre degraus (um treino NOVO que nasce de outro): so o
        ator e a normalizacao passam. O critico e o juiz recomecam (a nota e o
        acervo mudaram) e o contador volta a zero. A retomada do mesmo treino e
        outra coisa (restore)."""
        reference = self.resolve_checkpoint(source)
        state = storage.load_checkpoint(reference, str(self.device))
        if state["action_names"] != self.action_names:
            raise RuntimeError("a transferencia pede as mesmas acoes (o contrato da acao mudou)")
        network = state.get("network", self.config.network)
        saved = ActorCritic(len(state["obs_names"]), len(state["critic_names"]), len(self.action_names),
                            actor_hidden=network["actor"], critic_hidden=network["critic"]).to(self.device)
        saved.load_state_dict(state["model"])
        if state["obs_names"] != self.obs_names or state["critic_names"] != self.critic_names:
            saved = migrate_inputs(saved, list(state["obs_names"]), list(self.obs_names),
                                   list(state["critic_names"]), list(self.critic_names))
        parts = set(parts)
        if "ator" in parts:
            self.model.actor.load_state_dict(saved.actor.state_dict())
        if "normalizacao" in parts:
            self.model.obs_norm.load_state_dict(saved.obs_norm.state_dict())
        self.db.event(self.run, "transferencia", f"{'+'.join(sorted(parts))} de {source} "
                                                 f"(rodada {reference.iteration}); o critico e o juiz recomecam")

    def manifest(self) -> dict:
        return {"run": self.run, "task": self.config.task, "contract": 2, "body_id": self.batch.body_id,
                "body_hash": self.batch.body_hash, "task_hash": self.batch.task_hash, "steps": self.counters["steps"],
                "time": time.time(), "obs_names": self.obs_names, "critic_names": self.critic_names,
                "action_names": self.action_names, "beta": self.beta, "style": self.config.style,
                "seed": self.config.seed, "exploration": self.config.exploration, "ppo": self.config.ppo,
                "judge": self.config.judge, "overlap": self.config.overlap, "transferred_from": self.config.transferred_from,
                "warmup_left": self.counters.get("warmup_left", 0), "positive_only": self.positive_only,
                "torch": torch.__version__}

    def checkpoint_state(self) -> dict:
        return {"model": self.model.state_dict(), "ppo": self.ppo.state_dict(),
                "judge": self.judge.state_dict() if self.judge else None, "network": self.config.network,
                "counters": dict(self.counters), "obs_names": self.obs_names, "critic_names": self.critic_names,
                "action_names": self.action_names, "body_hash": self.batch.body_hash, "beta": self.beta,
                "rng_torch": torch.get_rng_state()}

    def restore(self) -> None:
        reference = storage.latest_checkpoint(self.directory)
        if reference is None:
            return
        state = storage.load_checkpoint(reference, str(self.device))
        if (state["action_names"] != self.action_names or state["obs_names"] != self.obs_names
                or state["critic_names"] != self.critic_names):
            raise RuntimeError("o checkpoint e de outro contrato: abra um treino novo")
        self.model.load_state_dict(state["model"])
        self.ppo.load_state_dict(state["ppo"])
        if self.judge is not None and state.get("judge"):
            self.judge.load_state_dict(state["judge"])
        self.counters.update(state.get("counters", {}))
        lost = self.db.rewind(self.run, self.counters["iteration"])
        if lost:
            self.db.event(self.run, "retomada", f"retomado da rodada {self.counters['iteration']}; "
                                                f"{lost} rodadas depois do checkpoint se perderam")
        if "rng_torch" in state:
            torch.set_rng_state(state["rng_torch"].cpu())

    def status(self) -> dict:
        return {
            "run": self.run, "task": self.config.task, "taskName": self.batch.task_name, "contract": 2,
            "state": self.state, "mode": self.mode.name, "modeLabel": self.mode.label, "threads": self.threads,
            "device": str(self.device), "environments": self.config.environments,
            "iteration": self.counters["iteration"], "steps": self.counters["steps"], "sps": self.recent_sps,
            "trainSeconds": self.counters["train_seconds"], "sessionSeconds": time.time() - self.started,
            "episodes": self.counters["episodes"], "last": self.last_iteration_stats,
            "evaluation": self.last_evaluation, "best": {},
            "cpuTemperature": system.cpu_temperature(), "memoryMB": system.memory_mb(),
            "gpuMemoryMB": torch.cuda.memory_allocated() / 1e6 if self.device.type == "cuda" else 0.0,
            "policy": str(self.directory / "exports" / "policy_latest.mrlpolicy"), "message": self.message,
            "commands": {}, "imitation": {}, "kickstart": {},
            "style": {"enabled": self.judge is not None, "beta": self.beta, "judge": self.last_judge},
            "pid": os.getpid(), "updated": time.time(),
        }

    def write_status(self) -> None:
        storage.write_json_atomic(self.directory / "status.json", self.status())

    # --- persistencia --------------------------------------------------------------
    def save(self, kind: str = "periodico") -> None:
        reference = storage.save_checkpoint(self.directory, self.counters["iteration"], self.checkpoint_state(),
                                            self.manifest())
        self.db.log_checkpoint(self.run, self.counters["iteration"], self.counters["steps"], reference.path, kind, None)
        self.export("policy_latest")
        for iteration in storage.prune_checkpoints(self.directory, set()):
            self.db.forget_checkpoint(self.run, iteration)
        self.db.compact(self.run)
        size = storage.directory_size_mb(paths.DATA) / 1024.0
        if size > self.config.disk_cap_gb:
            self.db.event(self.run, "disco", f"os treinos ocupam {size:.1f} GB (teto {self.config.disk_cap_gb} GB)")
        self.last_save = time.time()

    def export(self, name: str) -> None:
        export_policy(self.model, self.directory / "exports" / f"{name}.mrlpolicy", {
            "observationLayout": "matter.rl.observation", "observationVersion": 2, "actionLayout": "matter.rl.action",
            "bodyId": self.batch.body_id, "bodyHash": self.batch.body_hash, "taskId": self.config.task,
            "run": self.run, "iteration": self.counters["iteration"], "steps": self.counters["steps"],
            "observationNames": list(self.obs_names),
        })

    # --- avaliacao -------------------------------------------------------------------
    def evaluate(self) -> dict:
        """Cada exercicio com a acao media, os mesmos mundos e o solver do jogo:
        o primeiro episodio de cada mundo. As medidas vao para o banco."""
        self.drain()
        previous, self.state = self.state, "avaliando"
        self.write_status()
        results: dict = {}
        col = {name: i for i, name in enumerate(self.episode_columns)}
        # O solver do jogo (20/20) e o do treino (o do corpo).
        for drill, solver in [(d, s) for s in ("jogo", "treino") for d in self.drill_ids]:
            batch = self.evaluation_batches.get((drill, solver))
            if batch is None:
                iterations = 20 if solver == "jogo" else -1
                batch = self.native.BodyBatch(str(self.live_task), environments=self.config.evaluation_worlds,
                                              per_scene=4, threads=self.threads, seed=20261014, evaluation=True,
                                              evaluation_seconds=self.config.evaluation_seconds,
                                              solver_position=iterations, solver_velocity=iterations, drill=drill)
                self.evaluation_batches[(drill, solver)] = batch
            else:
                batch.update_task(str(self.live_task))
                batch.reset(20261014)
            worlds = batch.num_envs
            first: dict[int, np.ndarray] = {}
            steps = int(self.config.evaluation_seconds / batch.policy_dt) + 2
            with torch.no_grad():
                for _ in range(steps):
                    obs = self.tensor(batch.observations())
                    mean = self.model.actor(self.model.obs_norm(obs))
                    action = np.ascontiguousarray(mean.cpu().numpy(), dtype=np.float32)
                    batch.step(action, action)
                    for row in batch.finished_episodes():
                        first.setdefault(int(row[col["mundo"]]), np.array(row))
                    if len(first) >= worlds:
                        break
            rows = np.array(list(first.values())) if first else np.zeros((0, len(self.episode_columns)))
            summary = drill_summary(rows, self.episode_columns, self.drill_ids).get(drill, {})
            summary["worlds"] = worlds
            results.setdefault(solver, {})[drill] = summary
            self.db.log_body_evaluation(self.run, self.counters["iteration"], self.counters["steps"], drill, solver,
                                        summary)
        self.last_evaluation = {"iteration": self.counters["iteration"], "time": time.time(), "solvers": results}
        self.adjust_drill_weights(results.get("jogo", {}))
        self.state = previous
        return self.last_evaluation

    def adjust_drill_weights(self, evaluation: dict) -> None:
        """Com "drillWeights": {"auto": true} na tarefa, as fatias seguem a
        avaliacao do solver do jogo (o peso de base fica em baseWeight)."""
        live = json.loads(self.live_task.read_text())
        rule = live.get("drillWeights") or {}
        drills = live.get("drills") or []
        if not rule.get("auto") or len(drills) < 2 or not evaluation:
            return
        for drill in drills:
            drill.setdefault("baseWeight", drill.get("weight", 1.0))
        weights = balance_drills(drills, evaluation, rule)
        if all(abs(drill.get("weight", 1.0) - weights[drill["id"]]) < 1e-3 for drill in drills):
            return
        for drill in drills:
            drill["weight"] = round(weights[drill["id"]], 4)
        storage.write_json_atomic(self.live_task, live)
        self.batch.update_task(str(self.live_task))
        total = sum(weights.values())
        self.db.event(self.run, "exercicios", "fatias: " + ", ".join(
            f"{d['id']} {100 * weights[d['id']] / total:.0f}%" for d in drills))

    # --- comandos -------------------------------------------------------------------
    def handle_commands(self) -> None:
        for command, arguments, answer in self.server.pending():
            try:
                payload = self.command(command, arguments)
            except Exception as error:  # noqa: BLE001
                payload = {"ok": False, "error": str(error)}
            control.reply(answer, payload)

    def command(self, command: str, arguments: dict) -> dict:
        author = arguments.get("autor", "usuario")
        if command == "pausar":
            self.pause_requested = True
            self.db.event(self.run, "pausa", "pausado", author)
            return {"ok": True}
        if command == "retomar":
            self.pause_requested = False
            self.db.event(self.run, "retomada", "retomado", author)
            return {"ok": True}
        if command == "parar":
            self.stop_requested = True
            self.stop_reason = "usuario"
            self.db.event(self.run, "parada", "treino parado", author)
            return {"ok": True}
        if command == "salvar":
            self.drain()
            self.save("manual")
            return {"ok": True, "iteration": self.counters["iteration"]}
        if command == "avaliar":
            return {"ok": True, "evaluation": self.evaluate()}
        if command == "modo":
            name = arguments["modo"]
            system.mode(name)
            if name != self.mode.name:
                self.config.mode = name
                self.config.save(self.directory / "run.json")
                self.db.event(self.run, "modo", f"modo {system.MODES[name].label}", author)
                self.restart_requested = True
            return {"ok": True}
        if command == "peso":
            return self.set_weight(arguments["componente"], float(arguments["valor"]), author,
                                   arguments.get("motivo", ""))
        raise ValueError(f"comando desconhecido no Corpo v2: {command}")

    def set_weight(self, component: str, value: float, author: str, reason: str) -> dict:
        """O botao beta ("estilo") e os pesos da nota (custos e queda), na copia da
        tarefa do treino. Cada mudanca de beta salva um checkpoint antes."""
        data = json.loads(self.live_task.read_text())
        if component in ("estilo", "beta"):
            if self.judge is None:
                raise ValueError("este treino nao tem o juiz de estilo (B1)")
            if value < 0.0:
                raise ValueError("beta nao pode ser negativo")
            self.drain()
            self.save("antes_beta")
            old = self.beta
            data.setdefault("style", {})["beta"] = value
            storage.write_json_atomic(self.live_task, data)
            self.beta = value
            self.db.event(self.run, "beta", f"estilo: {old} -> {value}", author, {"motivo": reason})
            return {"ok": True}
        reward = data.setdefault("reward", {})
        costs = reward.setdefault("costs", {})
        if component == "fall":
            old = reward.get("fall", "padrao")
            reward["fall"] = value
        elif component in ("work", "torque", "slip", "impact", "head", "smooth", "limits", "contacts", "posture", "still", "athletic", "pivot"):
            old = costs.get(component, "padrao")
            costs[component] = value
        else:
            raise ValueError(f"parcela desconhecida: {component}")
        storage.write_json_atomic(self.live_task, data)
        self.batch.update_task(str(self.live_task))
        self.db.event(self.run, "peso", f"{component}: {old} -> {value}", author, {"motivo": reason})
        return {"ok": True}

    # --- o laco ---------------------------------------------------------------------
    def tensor(self, array) -> torch.Tensor:
        # (Uma copia: os buffers do lote valem so ate o proximo passo.)
        return torch.tensor(np.asarray(array), device=self.device, dtype=torch.float32)

    def compose_rewards(self, components: torch.Tensor, style: torch.Tensor | None) -> torch.Tensor:
        """A nota de cada mundo na decisao, pelas parcelas do lote (plano 5.4):
            dt x max(0, T + beta x G x S + A - custos) - F
        T, A, G e F vem do C++ (a tarefa ja com a porta de pe, a muleta do pe no
        ar, a porta do estilo e a queda ja aplicada); S e o estilo do juiz, que
        so o treinador tem. Sem o max (positiveOnly desligado) e a conta do C++.
        Nunca se soma isto a batch.rewards(): duplicaria a tarefa e a queda."""
        ri = self.ri
        rate = components[:, ri["task"]] + components[:, ri["air"]] - components[:, self.cost_index].sum(-1)
        if style is not None:
            rate = rate + self.beta * components[:, ri["style_gate"]] * style
        if self.positive_only:
            rate = rate.clamp(min=0.0)
        return self.batch.policy_dt * rate - components[:, ri["fall"]]

    def collect(self) -> tuple[Rollout, dict]:
        """Uma rodada de coleta com a copia que coleta (a politica de antes). A
        normalizacao fica congelada na rodada: as observacoes guardadas sao as que
        a politica viu, e a estatistica nova (a candidata) so vale na fronteira."""
        s = self.settings
        batch, net = self.batch, self.collector
        envs = batch.num_envs
        begin = time.time()
        context = torch.cuda.stream(self.stream) if self.stream is not None else _Null()
        components = np.zeros(batch.reward_count, dtype=np.float64)
        episodes: list[np.ndarray] = []
        pairs: list[np.ndarray] = []
        style_sum = paid_sum = bootstrap_sum = 0.0
        beta = self.beta
        if self.stream is not None:
            # Os pesos que o sync copiou (no fluxo padrao) antes da coleta ler.
            self.stream.wait_stream(torch.cuda.current_stream())
        with context, torch.no_grad():
            rollout = Rollout(s.steps_per_env, envs, batch.obs_size, batch.critic_size, batch.action_size, self.device)
            rollout.norm = (net.obs_norm.mean.clone(), net.obs_norm.inverse_std().clone())
            obs, critic = self.tensor(batch.observations()), self.tensor(batch.critic_observations())
            for _ in range(s.steps_per_env):
                self.obs_candidate.update(obs)
                self.critic_candidate.update(critic)
                obs_n, critic_n = net.obs_norm(obs), net.critic_norm(critic)
                dist = net.distribution(obs_n)
                action = dist.sample()
                log_prob = dist.log_prob(action).sum(-1)
                value = net.value(critic_n)
                batch.step(np.ascontiguousarray(action.cpu().numpy(), dtype=np.float32),
                           np.ascontiguousarray(dist.mean.cpu().numpy(), dtype=np.float32))
                parts = self.tensor(batch.reward_components())
                style = None
                if self.scorer is not None:
                    step_pairs = np.array(batch.style_pairs(), dtype=np.float32)
                    style = style_score(self.scorer(self.judge.normalize(self.tensor(step_pairs))))
                    if len(self.style_drills):
                        judged = np.isin(np.asarray(batch.world_drills()), self.style_drills)
                        step_pairs = step_pairs[judged]
                        style = style * self.tensor(judged.astype(np.float32))
                    pairs.append(step_pairs)
                    style_sum += float(style.sum())
                    paid_sum += float((beta * parts[:, self.ri["style_gate"]] * style).sum())
                reward = self.compose_rewards(parts, style)
                terminated = self.tensor(batch.terminated())
                truncated_np = np.asarray(batch.truncated())
                truncated = self.tensor(truncated_np)
                if truncated_np.any():
                    # Fim pelo tempo nao e fracasso: o valor do estado final entra.
                    index = np.nonzero(truncated_np)[0]
                    final = self.tensor(np.asarray(batch.terminal_critic_observations())[index])
                    credit = s.gamma * net.value(net.critic_norm(final))
                    reward[torch.as_tensor(index, device=self.device)] += credit
                    bootstrap_sum += float(credit.sum())
                rollout.add(obs_n, critic_n, action, log_prob, value, reward, torch.maximum(terminated, truncated),
                            dist.mean, dist.stddev, raw=obs)
                components += np.asarray(batch.reward_components()).sum(axis=0)
                finished = batch.finished_episodes()
                if len(finished):
                    episodes.append(np.array(finished))
                obs, critic = self.tensor(batch.observations()), self.tensor(batch.critic_observations())
            last_values = net.value(net.critic_norm(critic))
            rollout.compute_returns(last_values, s.gamma, s.lam)
        if self.stream is not None:
            self.stream.synchronize()
        samples = s.steps_per_env * envs
        rows = np.concatenate(episodes) if episodes else np.zeros((0, len(self.episode_columns)))
        info = {"collect": time.time() - begin, "samples": samples, "components": components / samples,
                "rows": rows, "pairs": np.concatenate(pairs) if pairs else None,
                "style": style_sum / samples if self.scorer is not None else None,
                "style_paid": paid_sum / samples if self.scorer is not None else 0.0,
                "bootstrap": bootstrap_sum / samples,
                "reward": float(rollout.rewards.mean())}
        return rollout, info

    def update(self, rollout: Rollout) -> dict:
        """A atualizacao (na thread da GPU, com a sobreposicao)."""
        begin = time.time()
        if self.config.overlap:
            # O KL da taxa contra a politica do comeco desta atualizacao.
            with torch.no_grad():
                dist = self.model.distribution(rollout.obs.reshape(-1, rollout.obs.shape[-1]))
                rollout.anchor_mean = dist.mean.reshape(rollout.mean.shape)
                rollout.anchor_std = dist.stddev.reshape(rollout.std.shape)
        # A variancia explicada do critico (antes de aprender com esta rodada).
        with torch.no_grad():
            returns, values = rollout.returns.reshape(-1), rollout.values.reshape(-1)
            explained = float(1.0 - (returns - values).var() / returns.var().clamp(min=1e-8))
        auxiliary = None
        if self.mirror is not None:
            raw = rollout.raw.reshape(-1, rollout.raw.shape[-1])
            weight = self.config.mirror_loss

            def auxiliary():
                index = torch.randint(0, raw.shape[0], (min(4096, raw.shape[0]),), device=raw.device)
                return weight * self.mirror(self.model, raw[index], raw=True, norm=rollout.norm)
        # O aquecimento depois de trocar o juiz: so o juiz e o critico aprendem.
        frozen = self.counters.get("warmup_left", 0) > 0
        stats = self.ppo.update(rollout, actor_frozen=frozen, auxiliary=auxiliary)
        stats["explained_variance"] = explained
        stats["warmup"] = float(frozen)
        if self.judge is not None:
            stats.update(self.judge.update())
        if self.device.type == "cuda":
            torch.cuda.synchronize()
        stats["update"] = time.time() - begin
        return stats

    def sync(self) -> None:
        """Depois de uma atualizacao: a normalizacao (que a coleta acumulou) vai
        para a rede treinada, e a rede treinada vai para a copia que coleta."""
        with torch.no_grad():
            # A estatistica candidata (acumulada na coleta) passa a valer aqui.
            self.model.obs_norm.load_state_dict(self.obs_candidate.state_dict())
            self.model.critic_norm.load_state_dict(self.critic_candidate.state_dict())
            self.collector.load_state_dict(self.model.state_dict())
            if self.scorer is not None:
                self.scorer.load_state_dict(self.judge.net.state_dict())

    def drain(self) -> None:
        """Espera a atualizacao em andamento (salvar, mudar beta, parar)."""
        if getattr(self, "pending", None) is not None:
            future, info = self.pending
            self.pending = None
            self.finish(future.result(), info)

    def finish(self, stats: dict, info: dict) -> None:
        """Uma rodada atualizada: sincroniza, registra e confere a saude."""
        self.sync()
        self.counters["iteration"] += 1
        self.counters["steps"] += info["samples"]
        self.counters["train_seconds"] += info["collect"] + stats["update"]
        rows = info["rows"]
        self.counters["episodes"] += len(rows)
        col = {name: i for i, name in enumerate(self.episode_columns)}
        reasons = rows[:, col["razao"]].astype(int) if len(rows) else np.zeros(0, dtype=int)
        names = self.native.BODY_END_REASONS
        ends = {names[r]: int((reasons == r).sum()) for r in set(reasons.tolist())}
        # A rodada leva o maior dos dois tempos com a sobreposicao.
        seconds = max(info["collect"], stats["update"]) if self.config.overlap else info["collect"] + stats["update"]
        sps = info["samples"] / max(1e-6, seconds)
        self.recent_sps = sps if self.recent_sps == 0.0 else 0.9 * self.recent_sps + 0.1 * sps
        judge = {k: v for k, v in stats.items() if k.startswith(("judge_", "style_"))}
        if judge:
            self.last_judge = judge
        # As parcelas da nota, por decisao (taxa por segundo na tarefa, na muleta e nos
        # custos; a queda e o bootstrap em valor), sem misturar o estilo e o bootstrap
        # com o retorno do episodio.
        ci = info["components"]
        parts = {"task": float(ci[self.ri["task"]]), "air": float(ci[self.ri["air"]]),
                 "costs": float(sum(ci[i] for i in self.cost_index)), "fall": float(ci[self.ri["fall"]]),
                 "style": info["style"], "style_paid": info["style_paid"], "bootstrap": info["bootstrap"],
                 "stand_gate": float(ci[self.ri["stand_gate"]]), "style_gate": float(ci[self.ri["style_gate"]]),
                 "total": info["reward"]}
        ppo = {key: stats[key] for key in ("kl", "kl_max", "kl_halted", "actor_steps", "clip_fraction", "grad_norm",
                                           "critic_grad_norm", "explained_variance", "value_loss", "learning_rate",
                                           "action_std", "warmup") if key in stats}
        details = {"beta": self.beta, "style": info["style"], "judge": judge, "mirror": stats.get("auxiliary"),
                   "parts": parts, "ppo": ppo, "drills": drill_summary(rows, self.episode_columns, self.drill_ids)}
        result = {
            "run": self.run, "iteration": self.counters["iteration"], "time": time.time(),
            "steps": self.counters["steps"], "sps": sps, "reward": info["reward"], "episodes": int(len(rows)),
            "episode_return": float(rows[:, col["retorno"]].mean()) if len(rows) else None,
            "episode_seconds": float(rows[:, col["segundos"]].mean()) if len(rows) else None,
            "fall_fraction": float(np.isin(reasons, [1, 2, 3]).mean()) if len(rows) else None,
            "components": json.dumps({n: float(v) for n, v in zip(self.reward_names, info["components"])}),
            "ends": json.dumps(ends), "policy_loss": stats["policy_loss"], "value_loss": stats["value_loss"],
            "entropy": stats["entropy"], "kl": stats["kl"], "clip_fraction": stats["clip_fraction"],
            "learning_rate": stats["learning_rate"], "action_std": stats["action_std"],
            "grad_norm": stats["grad_norm"], "collect_seconds": info["collect"], "update_seconds": stats["update"],
            "details": json.dumps(details),
        }
        if not self.healthy(result):
            self.failures += 1
            self.recover()
            if self.failures >= 3:
                self.pause_requested = True
                self.message = "a rede ficou invalida 3 vezes seguidas: pausado"
                self.db.event(self.run, "erro", self.message)
            return
        self.failures = 0
        # O aquecimento do juiz novo: sai no maximo de rodadas ou, depois do
        # minimo, quando o critico ja explica o retorno (variancia explicada > 0,5).
        if stats.get("warmup"):
            self.counters["warmup_left"] -= 1
            self.counters["warmup_done"] += 1
            if self.counters["warmup_left"] > 0 and self.counters["warmup_done"] >= self.config.warmup_min_rounds \
                    and stats["explained_variance"] > 0.5:
                self.counters["warmup_left"] = 0
            if self.counters["warmup_left"] == 0:
                self.db.event(self.run, "aquecimento", f"o ator volta a aprender depois de {self.counters['warmup_done']} "
                                                       f"rodadas (variancia explicada {stats['explained_variance']:.2f})")
        self.update_curriculum(rows, col)
        # O recorte alto por muitas rodadas: 3 epocas em vez de 5.
        self.counters["clip_streak"] = self.counters["clip_streak"] + 1 if stats["clip_fraction"] > 0.35 else 0
        if self.counters["clip_streak"] >= self.config.clip_streak_limit and self.ppo.settings.epochs > 3:
            self.ppo.settings.epochs = self.counters["epochs"] = 3
            self.db.event(self.run, "epocas", "o recorte ficou acima de 0,35 por "
                                              f"{self.config.clip_streak_limit} rodadas: 3 epocas")
        self.last_iteration_stats = {k: result[k] for k in (
            "iteration", "sps", "reward", "episodes", "episode_return", "episode_seconds", "fall_fraction", "kl",
            "entropy", "learning_rate", "action_std", "value_loss")}
        self.last_iteration_stats["style"] = info["style"]
        self.db.log_iteration(result)
        self.db.commit()

    def update_curriculum(self, rows: np.ndarray, col: dict) -> None:
        """Sobe a janela de velocidade pedida (curriculum.speedMax na copia da tarefa)
        quando o corpo ja segue bem o pedido de agora."""
        if not len(rows):
            return
        live = json.loads(self.live_task.read_text())
        curriculum = live.get("curriculum")
        if not curriculum or not curriculum.get("speedCap"):
            return
        top, cap = float(curriculum.get("speedMax", 0.0)), float(curriculum["speedCap"])
        if top >= cap:
            return
        c, cfg = self.counters, self.config
        score = float(rows[:, col["nota_velocidade"]].mean())
        c["curriculum_streak"] = c["curriculum_streak"] + 1 if score >= cfg.curriculum_threshold else 0
        if c["curriculum_streak"] >= cfg.curriculum_rounds and c["iteration"] - c["curriculum_last"] >= cfg.curriculum_gap:
            curriculum["speedMax"] = min(cap, top + cfg.curriculum_step)
            storage.write_json_atomic(self.live_task, live)
            self.batch.update_task(str(self.live_task))
            c["curriculum_streak"], c["curriculum_last"] = 0, c["iteration"]
            self.db.event(self.run, "curriculo", f"velocidade pedida ate {curriculum['speedMax']:.2f} m/s "
                                                 f"(progresso medio {score:.2f})")

    def healthy(self, result: dict) -> bool:
        values = [result["policy_loss"], result["value_loss"], result["kl"]]
        if not all(math.isfinite(v) for v in values):
            return False
        return all(torch.isfinite(p).all() for p in self.model.parameters())

    def recover(self) -> None:
        """NaN na rede: volta ao ultimo checkpoint bom com a taxa pela metade."""
        reference = storage.latest_checkpoint(self.directory)
        if reference is None:
            raise RuntimeError("a rede ficou invalida antes do primeiro checkpoint")
        state = storage.load_checkpoint(reference, str(self.device))
        self.model.load_state_dict(state["model"])
        self.ppo.load_state_dict(state["ppo"])
        if self.judge is not None and state.get("judge"):
            self.judge.load_state_dict(state["judge"])
        self.fix_raw_channels()
        self.reset_candidates()
        self.collector.load_state_dict(self.model.state_dict())
        if self.scorer is not None:
            self.scorer.load_state_dict(self.judge.net.state_dict())
        self.counters.update(state.get("counters", {}))
        self.ppo.settings.epochs = int(self.counters["epochs"])
        self.db.rewind(self.run, self.counters["iteration"])
        self.ppo.learning_rate = max(self.settings.min_learning_rate, self.ppo.learning_rate * 0.5)
        for group in self.ppo.optimizer.param_groups:
            group["lr"] = self.ppo.learning_rate
        self.batch.reset(int(time.time()))
        self.db.event(self.run, "recuperacao", f"valores invalidos: voltou ao checkpoint {reference.iteration}")

    def round(self) -> None:
        """Coleta uma rodada e atualiza (com a sobreposicao, a atualizacao desta
        rodada roda enquanto a proxima coleta)."""
        rollout, info = self.collect()
        if self.executor is None:
            if self.judge is not None and info["pairs"] is not None:
                self.judge.remember(info["pairs"])
            self.finish(self.update(rollout), info)
            return
        self.drain()
        # (A memoria do juiz so muda com a atualizacao parada.)
        if self.judge is not None and info["pairs"] is not None:
            self.judge.remember(info["pairs"])
        self.pending = (self.executor.submit(self.update, rollout), info)

    def loop(self) -> int:
        self.last_save = time.time()
        self.pending = None
        self.failures = 0
        last_status = 0.0
        last_export = time.time()
        last_evaluation = time.time()
        code = 0
        try:
            while True:
                self.handle_commands()
                if self.stop_requested or self.restart_requested:
                    break
                if self.pause_requested:
                    if self.state != "pausado":
                        self.drain()
                        self.save("pausa")
                        self.state = "pausado"
                        self.write_status()
                    time.sleep(0.2)
                    continue
                # (Com a sobreposicao, a atualizacao em andamento ja conta.)
                if self.max_iterations and \
                        self.counters["iteration"] + (1 if self.pending else 0) >= self.max_iterations:
                    break
                self.state = "treinando"
                self.round()
                now = time.time()
                if self.config.evaluation_minutes > 0.0 and now - last_evaluation >= self.config.evaluation_minutes * 60.0:
                    self.evaluate()
                    last_evaluation = time.time()
                if now - self.last_save >= self.config.checkpoint_minutes * 60.0:
                    self.drain()
                    self.save()
                    last_export = now
                elif now - last_export >= self.config.export_minutes * 60.0:
                    self.drain()
                    self.export("policy_latest")
                    last_export = now
                if now - last_status >= 2.0:
                    self.write_status()
                    last_status = now
            code = RESTART_CODE if self.restart_requested and not self.stop_requested else 0
            if self.max_iterations and not self.stop_requested:
                self.stop_reason = "usuario"
        except Exception as error:  # noqa: BLE001
            self.message = f"erro: {error}"
            self.state = "erro"
            self.db.event(self.run, "erro", str(error))
            code = 1
            raise
        finally:
            self.state = "parando"
            try:
                self.drain()
            finally:
                self.config.end_reason = "erro" if code == 1 else ("" if code == RESTART_CODE else self.stop_reason)
                self.config.save(self.directory / "run.json")
                try:
                    if all(torch.isfinite(p).all() for p in self.model.parameters()):
                        self.save("final")
                finally:
                    self.state = "parado"
                    self.write_status()
                    self.db.end_session(self.session, "reinicio" if code == RESTART_CODE else
                                        ("erro" if code else "parado"))
                    self.server.close()
                    if self.executor is not None:
                        self.executor.shutdown(wait=True)
                    self.lock.release()
        return code


class _Null:
    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False
