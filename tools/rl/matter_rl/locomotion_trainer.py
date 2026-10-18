"""O treino da locomocao (a etapa B do plano, vault 04_locomocao/plano_fases_1_2.md):
o PPO sobre o lote novo (matter_locomotion_native.LocomotionBatch), com a rede
dentro dos modulos de locomocao do jogo (o contrato v2). Reaproveita o PPO, a
rede, o espelho, o banco, o status e o socket do Corpo v2; a nota vem pronta do
C++ (sem juiz de estilo: o estilo e o do padrao a sombra).

O curriculo da fase: a autoridade das cabecas sobe de 0 a 1 nas primeiras
rodadas (a rampa de entrada); os tetos sao os da tarefa; as fatias dos
exercicios seguem a dificuldade na avaliacao (com piso). A avaliacao mede cada
exercicio com a acao media, e tambem sem a pose e a rigidez (a ablacao do
portao: a melhora nao pode vir so delas) e o classico (autoridade zero).
"""
from __future__ import annotations

import concurrent.futures
import copy
import json
import math
import os
import random
import signal
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import torch

from . import control, paths, storage, system
from .body_trainer import balance_drills, choose_device, wilson_upper
from .curriculum import SpeedCurriculum
from .model import ActorCritic, export_policy, migrate_inputs, rescale_actions
from .ppo import PPO, PPOSettings, Rollout
from .symmetry import MirrorLoss

TASK_SCHEMA = "matter-rl-locomocao-1"
KIND = "locomocao"
NETWORK = {"actor": [512, 512, 256], "critic": [512, 512, 256]}
# Os canais da entrada que passam crus pela normalizacao: os do critico que so o
# treino sabe (o exercicio, um-quente) - os outros normalizam.
RAW_PREFIXES = ("drill.",)
# Os grupos das saidas (a telemetria da saturacao e do desvio; vault
# .../pesquisas/03_otimizacao_ppo.md). Os prefixos mais longos primeiro.
ACTION_GROUPS = ("pose", "pes", "passada", "equilibrio", "ganho", "turno")


def action_group(name: str) -> str:
    head = name.split(".")[0]
    return "pes" if head == "passada" else head


def channel_values(names: list[str], table: dict, default: float) -> np.ndarray:
    """Um valor por canal pelo prefixo mais longo da tabela ("*" = o resto)."""
    keys = sorted((k for k in table if k != "*"), key=len, reverse=True)
    out = []
    for name in names:
        match = next((k for k in keys if name.startswith(k)), None)
        out.append(float(table[match]) if match else float(table.get("*", default)))
    return np.asarray(out, dtype=np.float32)
# O PPO da locomocao: o critico a parte; o KL somado nos 61 canais (em unidades
# do teto, desvio inicial 0,5).
PPO_DEFAULTS = {"steps_per_env": 32, "epochs": 5, "minibatches": 4, "gamma": 0.99, "lam": 0.95, "clip": 0.2,
                "entropy_coef": 0.0005, "learning_rate": 3e-4, "desired_kl": 0.02, "separate_critic": True,
                "critic_learning_rate": 3e-4, "max_std": 1.0}


def native():
    directory = str(paths.NATIVE_DIR)
    if directory not in sys.path:
        sys.path.insert(0, directory)
    try:
        import matter_locomotion_native  # type: ignore
    except ImportError as error:  # pragma: no cover
        raise SystemExit(f"modulo nativo da locomocao ausente em {paths.NATIVE_DIR} ({error}); "
                         f"rode tools/rl/setup.sh") from error
    return matter_locomotion_native


def is_locomotion_task(path: Path) -> bool:
    try:
        return json.loads(path.read_text()).get("schema") == TASK_SCHEMA
    except (OSError, ValueError):
        return False


def is_locomotion_run(run: str) -> bool:
    config = paths.run_dir(run) / "run.json"
    try:
        return config.exists() and json.loads(config.read_text()).get("kind") == KIND
    except (OSError, ValueError):
        return False


@dataclass
class LocomotionRunConfig:
    kind: str = KIND
    contract: str = "locomocao-2"
    task: str = "locomocao_f1f2"
    environments: int = 512
    seed: int = 1
    mode: str = "maximo"
    network: dict = field(default_factory=lambda: dict(NETWORK))
    ppo: dict = field(default_factory=lambda: dict(PPO_DEFAULTS))
    # O desvio inicial da acao, em unidades do teto da fase (0,5: metade do
    # teto de cada canal).
    initial_std_of_cap: float = 0.5
    # A rampa de entrada da autoridade (todas as cabecas, de 0 a 1 em pelo
    # menos estas rodadas): ela so sobe numa rodada cujo risco de queda - a
    # fracao de episodios inteiros que acabariam em queda, as quedas da rodada
    # sobre os episodios-equivalentes dela (segundos de mundo / duracao do
    # episodio) - fica ate authority_max_falls; acima, segura. (A fracao das
    # quedas entre os episodios que acabaram nao serve: nas primeiras rodadas
    # so as quedas acabam, e 2 em 512 mundos davam 100%.)
    authority_ramp_rounds: int = 60
    authority_max_falls: float = 0.15
    mirror_loss: float = 0.3
    evaluation_minutes: float = 15.0
    evaluation_worlds: int = 24
    evaluation_seconds: float = 20.0
    checkpoint_minutes: float = 10.0
    # Todos os mundos num exercicio (o treino curto sozinho de uma novidade).
    drill: str = ""
    disk_cap_gb: float = 40.0
    # A atualizacao do PPO sobreposta a coleta seguinte (vault
    # 02_estrutura/estudo_desempenho.md, pipeline-16; o padrao do body_trainer):
    # a coleta usa a politica de uma atualizacao atras; a razao do PPO e contra
    # quem coletou e o KL da taxa contra a politica do comeco da atualizacao.
    overlap: bool = True
    # Os limites do desvio da exploracao por grupo, na unidade do teto (vault
    # .../pesquisas/08_plano_consenso.md, Q1): {prefixo: valor, "*": o resto}.
    # Vazios (o padrao): so o max_std do PPO, o de sempre.
    std_floor: dict = field(default_factory=dict)
    std_ceiling: dict = field(default_factory=dict)
    # Canais da entrada que passam crus alem do exercicio (as bandeiras 0/1; Q3) e
    # a memoria da normalizacao (amostras; 0 = sem limite, o de sempre).
    raw_extra_prefixes: list = field(default_factory=list)
    norm_max_count: float = 0.0
    end_reason: str = ""

    @staticmethod
    def load(path: Path) -> "LocomotionRunConfig":
        data = json.loads(path.read_text())
        known = {k: v for k, v in data.items() if k in LocomotionRunConfig.__dataclass_fields__}
        config = LocomotionRunConfig(**known)
        config.ppo = {**PPO_DEFAULTS, **config.ppo}
        return config

    def save(self, path: Path) -> None:
        storage.write_json_atomic(path, asdict(self))


def drill_summary(rows: np.ndarray, columns: list[str], drills: list[str], reasons: list[str]) -> dict:
    """As medidas dos episodios que acabaram, por exercicio (as chaves que a
    regra das fatias, o status e o relatorio leem)."""
    out: dict = {}
    if not len(rows):
        return out
    col = {name: i for i, name in enumerate(columns)}
    fall_codes = [reasons.index(name) for name in ("queda", "numerico", "falha") if name in reasons]
    for index, drill in enumerate(drills):
        part = rows[rows[:, col["exercicio"]] == index]
        if not len(part):
            continue
        ended = part[:, col["razao"]].astype(int)
        falls = int(np.isin(ended, fall_codes).sum())
        velocity = part[:, col["erro_velocidade"]]

        def median_of(name):
            if name not in col:
                return None
            values = part[:, col[name]]
            values = values[values >= 0.0]
            return float(np.median(values)) if len(values) else None

        def mean_of(name, valid=lambda v: v >= 0.0):
            values = part[:, col[name]]
            values = values[valid(values)]
            return float(values.mean()) if len(values) else None

        out[drill] = {
            "episodes": int(len(part)),
            "falls": falls / len(part),
            "falls_upper95": wilson_upper(falls, len(part)),
            "seconds": float(part[:, col["segundos"]].mean()),
            "return": float(part[:, col["retorno"]].mean()),
            "velocity_error_median": float(np.median(velocity)),
            "velocity_error": float(velocity.mean()),
            "velocity_error_window_median": median_of("erro_vel_janela"),
            "velocity_error_cruise_median": median_of("erro_vel_cruzeiro"),
            "speed": float(part[:, col["velocidade_media"]].mean()),
            "requested": float(part[:, col["pedido_medio"]].mean()),
            "facing_error": float(part[:, col["erro_rumo"]].mean()),
            "style_error": mean_of("erro_estilo"),
            "contact": mean_of("contato_por_pouso"),
            "touchdowns_per_second": float(part[:, col["pousos_por_segundo"]].mean()),
            "slip": float(part[:, col["escorregar"]].mean()),
            "power": float(part[:, col["potencia"]].mean()),
            "assist": float(part[:, col["ajuda"]].mean()),
            "posture": float(part[:, col["postura"]].mean()),
            "residual": float(part[:, col["residuo"]].mean()),
            "jerk": float(part[:, col["trancos"]].mean()),
            "tremor": mean_of("tremor"),
            "targets_per_minute": float(part[:, col["alvos_por_minuto"]].mean()),
            "hold_drift": mean_of("deriva_espera"),
            "arc_error": mean_of("afastamento_curva"),
        }
        if "postura_tronco" in col:
            seconds = np.maximum(part[:, col["segundos"]], 1e-6)
            fell_rows = part[np.isin(ended, fall_codes)]
            since = fell_rows[:, col["queda_desde_troca"]] if len(fell_rows) else np.zeros(0)
            since = since[since >= 0.0]
            out[drill].update({
                "posture_trunk": float(part[:, col["postura_tronco"]].mean()),
                "posture_neck": float(part[:, col["postura_pescoco"]].mean()),
                "neck_body": float(part[:, col["pescoco_corpo"]].mean()),
                "neck_shadow": mean_of("pescoco_sombra"),
                "neck_relative": mean_of("pescoco_relativo"),
                "feasible": float(part[:, col["viavel_medio"]].mean()),
                "jerk_cap": float(part[:, col["trancos_teto"]].mean()),
                "fall_since_change_median": float(np.median(since)) if len(since) else None,
                "saved_start_fraction": float(part[:, col["comeco_guardado"]].mean()),
                "assist_level": float(part[:, col["ajuda_nivel"]].mean()),
                "terms_per_second": {name[5:]: float(part[:, col[name]].sum() / seconds.sum())
                                     for name in columns if name.startswith("nota_")},
                # As quedas no primeiro segundo dos episodios que comecaram guardados
                # (o instante e aproximado: o resolvedor recomeca frio).
                "saved_start_early_falls": (float((np.isin(ended, fall_codes)
                                                   & (part[:, col["segundos"]] < 1.0))[part[:, col["comeco_guardado"]] > 0.5].mean())
                                            if (part[:, col["comeco_guardado"]] > 0.5).any() else None),
            })
            cells = {}
            fall_cell = part[:, col["celula_queda"]].astype(int)
            for k in range(16):
                decisions = float(part[:, col[f"cel{k:02d}.decisoes"]].sum())
                if decisions <= 0.0:
                    continue
                cruise = float(part[:, col[f"cel{k:02d}.cruzeiro"]].sum())
                cells[str(k)] = {
                    "decisions": decisions, "cruise": cruise,
                    "tracking": float(part[:, col[f"cel{k:02d}.rastreio"]].sum()) / cruise if cruise > 0 else None,
                    "error": float(np.sqrt(part[:, col[f"cel{k:02d}.erro2"]].sum() / cruise)) if cruise > 0 else None,
                    "asked": float(part[:, col[f"cel{k:02d}.pedida"]].sum()) / decisions,
                    "made": float(part[:, col[f"cel{k:02d}.feita"]].sum()) / decisions,
                    "falls": int((fall_cell == k).sum()),
                    "episodes": int((part[:, col[f"cel{k:02d}.decisoes"]] > 0).sum()),
                }
            out[drill]["cells"] = cells
    return out


class LocomotionTrainer:
    def __init__(self, run: str, task: str | None = None, mode: str | None = None, environments: int | None = None,
                 device: str = "auto", seed: int | None = None, socket_path: Path | None = None,
                 iterations: int | None = None, drill: str | None = None):
        self.run = run
        self.directory = paths.run_dir(run)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.lock = storage.RunLock(self.directory)
        storage.clean_temporaries(self.directory)
        config_path = self.directory / "run.json"
        fresh = not config_path.exists()
        if not fresh and not is_locomotion_run(run):
            raise RuntimeError(f"o treino {run} nao e da locomocao: abra um treino novo")
        self.config = LocomotionRunConfig() if fresh else LocomotionRunConfig.load(config_path)
        if task:
            self.config.task = task
        if mode:
            self.config.mode = mode
        if environments:
            self.config.environments = environments
        if seed is not None:
            self.config.seed = seed
        if drill is not None:
            self.config.drill = drill
        if fresh and os.environ.get("MATTER_RL_PPO"):
            self.config.ppo.update(json.loads(os.environ["MATTER_RL_PPO"]))
        self.config.end_reason = ""
        # (Nenhuma constante T_* do motor no ambiente sem querer.)
        self.tuned_environment = system.refuse_tuned_environment("treino da locomocao")
        self.config.save(config_path)
        self.max_iterations = iterations
        random.seed(self.config.seed)
        np.random.seed(self.config.seed % (2 ** 32))
        torch.manual_seed(self.config.seed)

        self.mode = system.mode(self.config.mode)
        system.apply_priority(self.mode)
        self.threads = system.threads_for(self.mode)
        torch.set_num_threads(2)
        self.device = choose_device(device)
        self.native = native()
        self.task_path = paths.task_path(self.config.task)
        if not is_locomotion_task(self.task_path):
            raise RuntimeError(f"a tarefa {self.config.task} nao e da locomocao ({TASK_SCHEMA})")
        # Uma copia da tarefa no treino (as fatias e os pesos mudados andando).
        # A troca de tarefa (matter-rl tarefa: a fase seguinte, o mesmo cerebro)
        # troca a copia; o que a rede sabe continua (restore migra o critico e
        # reescala a acao se os tetos abriram).
        self.live_task = self.directory / "tarefa.json"
        self.switched_from = ""
        source_task = json.loads(self.task_path.read_text())
        if self.live_task.exists():
            previous = json.loads(self.live_task.read_text()).get("id", "")
            if previous != source_task.get("id"):
                storage.write_json_atomic(self.live_task, source_task)
                self.switched_from = previous
        else:
            storage.write_json_atomic(self.live_task, source_task)
        self.assets = str(paths.ASSETS)
        self.batch = self.native.LocomotionBatch(str(self.live_task), self.assets, environments=self.config.environments,
                                                 threads=self.threads, seed=self.config.seed, drill=self.config.drill)
        self.obs_names = self.batch.observation_names()
        self.critic_names = self.batch.critic_names()
        self.action_names = self.batch.action_names()
        self.reward_names = self.batch.reward_names()
        self.episode_columns = self.batch.episode_columns()
        self.end_reasons = self.batch.end_reasons()
        self.drill_ids = list(self.batch.drill_ids)
        self.episode_seconds = float(json.loads(self.live_task.read_text()).get("episodeSeconds", 20.0))
        # A rede age em unidades do teto da fase (+-1 = o teto de cada canal); o
        # lote recebe a acao na escala maxima do contrato (vezes a fracao do
        # teto). Assim o desvio e o KL ficam bem condicionados, e abrir um teto
        # num portao e reescalar a ultima camada (model.rescale_actions).
        fractions = np.asarray(self.batch.action_cap_fractions(), dtype=np.float32)
        self.action_scale = torch.as_tensor(fractions, device=self.device)
        std0 = np.full(len(fractions), self.config.initial_std_of_cap, dtype=np.float32)
        network = self.config.network
        self.model = ActorCritic(self.batch.obs_size, self.batch.critic_size, self.batch.action_size, std0,
                                 actor_hidden=network["actor"], critic_hidden=network["critic"]).to(self.device)
        self.settings = PPOSettings.from_dict(self.config.ppo)
        self.ppo = PPO(self.model, self.settings)
        self.mirror = MirrorLoss(self.batch.mirror_maps(), self.device) if self.config.mirror_loss > 0.0 else None
        # Os limites de cada canal na unidade da rede (a acao aplicada para de mudar
        # fora deles) e os grupos das saidas (a telemetria; a perda de limites).
        lower, upper = self.batch.action_bounds()
        self.action_lower = torch.as_tensor(np.asarray(lower, dtype=np.float32), device=self.device)
        self.action_upper = torch.as_tensor(np.asarray(upper, dtype=np.float32), device=self.device)
        self.action_groups = [action_group(name) for name in self.action_names]
        self.std_floor = (torch.as_tensor(np.log(channel_values(self.action_names, self.config.std_floor, 1e-3)),
                                          device=self.device) if self.config.std_floor else None)
        self.std_ceiling = (torch.as_tensor(np.log(channel_values(self.action_names, self.config.std_ceiling, 10.0)),
                                            device=self.device) if self.config.std_ceiling else None)
        self.counters = {"iteration": 0, "steps": 0, "train_seconds": 0.0, "episodes": 0, "authority": 0.0}
        self.db = storage.Database(paths.DATABASE)
        self.db.ensure_run(run, self.config.task, self.batch.contract_hash, asdict(self.config))
        if self.tuned_environment:
            self.db.event(run, "ambiente", "rodando com T_* definidas: " + json.dumps(self.tuned_environment))
        self.restore()
        self.fix_raw_channels()
        self.configure_ppo()
        self.apply_std_bounds()
        self.reset_candidates()
        self.collector = copy.deepcopy(self.model)
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=1, initializer=self._init_worker) \
            if self.config.overlap else None
        self.stream = torch.cuda.Stream() if self.config.overlap and self.device.type == "cuda" else None
        self.pending = None
        # (Geradores proprios da atualizacao: com a sobreposicao, a ordem em que
        # coleta e atualizacao consomem o gerador padrao dependeria do tempo.)
        self.update_generator = torch.Generator(device=self.device)
        self.update_generator.manual_seed(self.config.seed * 7919 + 17 + self.counters["iteration"])
        self.apply_authority()
        self.load_drill_weights()
        # O curriculo de velocidade por celulas (os tetos moram na copia da tarefa).
        live = json.loads(self.live_task.read_text())
        self.curriculum = SpeedCurriculum(live, self.drill_ids)
        if self.curriculum.rule.enabled:
            before = json.dumps(live.get("speedCurriculum", {}).get("caps", {}), sort_keys=True)
            self.curriculum.initial_caps(live)
            if json.dumps(live["speedCurriculum"]["caps"], sort_keys=True) != before:
                storage.write_json_atomic(self.live_task, live)
                self.batch.update_task(str(self.live_task))

        self.state = "treinando"
        self.message = ""
        self.stop_requested = False
        self.stop_reason = "sinal"
        self.restart_requested = False
        self.pause_requested = False
        self.started = time.time()
        self.recent_sps = 0.0
        self.last_iteration_stats: dict = {}
        self.last_evaluation: dict = {}
        self.evaluation_batches: dict = {}
        self.failures = 0
        self.session = self.db.begin_session(run, self.counters["iteration"], self.mode.name, str(self.device))
        if self.switched_from:
            self.db.event(run, "tarefa", f"{self.switched_from} -> {self.config.task} (o mesmo cerebro)")
        if self.restore_notes:
            self.db.event(run, "retomada", "; ".join(self.restore_notes))
        self.db.event(run, "sessao", f"locomocao aberta no modo {self.mode.label} ({self.threads} threads, {self.device}; "
                                     f"contrato v{self.batch.contract_version} {self.batch.contract_hash}, "
                                     f"tarefa {self.batch.task_name})")
        self.server = control.ControlServer(self.status, socket_path or paths.SOCKET)
        signal.signal(signal.SIGTERM, self._signal_stop)
        signal.signal(signal.SIGINT, self._signal_stop)

    # --- estado ------------------------------------------------------------------
    def _signal_stop(self, *_):
        self.stop_requested = True
        self.stop_reason = "sinal"

    def fix_raw_channels(self) -> None:
        prefixes = RAW_PREFIXES + tuple(self.config.raw_extra_prefixes)
        for norm, names in ((self.model.obs_norm, self.obs_names), (self.model.critic_norm, self.critic_names)):
            norm.fix_channels(torch.tensor([name.startswith(prefixes) for name in names]))
            norm.max_count = float(self.config.norm_max_count)

    def configure_ppo(self) -> None:
        """O que o PPO precisa saber do contrato (depois de todo PPO novo)."""
        self.ppo.bounds = (self.action_lower, self.action_upper)

    @torch.no_grad()
    def apply_std_bounds(self) -> None:
        """O piso e o teto do desvio por grupo (Q1; sem tabelas, nada muda)."""
        if self.std_floor is not None:
            self.model.log_std.copy_(torch.maximum(self.model.log_std, self.std_floor))
        if self.std_ceiling is not None:
            self.model.log_std.copy_(torch.minimum(self.model.log_std, self.std_ceiling))

    def reset_candidates(self) -> None:
        self.obs_candidate = copy.deepcopy(self.model.obs_norm)
        self.critic_candidate = copy.deepcopy(self.model.critic_norm)

    def authority(self) -> float:
        return float(min(1.0, max(0.0, self.counters.get("authority", 0.0))))

    def advance_authority(self, fall_risk: float) -> None:
        """Um degrau da rampa de entrada, se o risco de queda da rodada deixa."""
        if self.authority() >= 1.0:
            return
        if fall_risk > self.config.authority_max_falls:
            return
        step = 1.0 / max(1, int(self.config.authority_ramp_rounds))
        self.counters["authority"] = min(1.0, self.authority() + step)

    def apply_authority(self) -> None:
        a = self.authority()
        self.batch.set_authority(a, a, a, a)

    def load_drill_weights(self) -> None:
        live = json.loads(self.live_task.read_text())
        weights = [float(d.get("weight", 1.0)) for d in live.get("drills", [])]
        if len(weights) == len(self.drill_ids):
            self.batch.set_drill_weights(weights)

    def manifest(self) -> dict:
        return {"run": self.run, "kind": KIND, "task": self.config.task, "contract": self.batch.contract_version,
                "contract_hash": self.batch.contract_hash, "task_hash": self.batch.task_hash,
                "body_id": self.batch.body_id, "steps": self.counters["steps"], "time": time.time(),
                "obs_names": self.obs_names, "critic_names": self.critic_names, "action_names": self.action_names,
                "seed": self.config.seed, "ppo": self.config.ppo, "network": self.config.network,
                "authority": self.authority(), "torch": torch.__version__}

    def checkpoint_state(self) -> dict:
        return {"model": self.model.state_dict(), "ppo": self.ppo.state_dict(), "network": self.config.network,
                "action_scale": self.action_scale.cpu().tolist(),
                "counters": dict(self.counters), "obs_names": self.obs_names, "critic_names": self.critic_names,
                "action_names": self.action_names, "contract_hash": self.batch.contract_hash,
                "rng_torch": torch.get_rng_state(),
                "rng_cuda": torch.cuda.get_rng_state() if self.device.type == "cuda" else None}

    def restore(self) -> None:
        self.restore_notes: list[str] = []
        reference = storage.latest_checkpoint(self.directory)
        if reference is None:
            return
        state = storage.load_checkpoint(reference, str(self.device))
        # Um contrato novo so com canais ACRESCENTADOS (a entrada e a saida da
        # virada brusca, v3) migra: os que ja existiam levam os pesos, os novos
        # entram com peso zero (a politica antiga segue igual no primeiro
        # instante). Canal que some ou muda de nome nao migra.
        additive = set(state["obs_names"]) <= set(self.obs_names) and set(state["action_names"]) <= set(self.action_names)
        io_changed = list(state["obs_names"]) != list(self.obs_names) or list(state["action_names"]) != list(self.action_names)
        if (state.get("contract_hash") != self.batch.contract_hash or io_changed) and not additive:
            raise RuntimeError("o checkpoint e de outro contrato: abra um treino novo")
        fresh_optimizer = False
        old_scale = state.get("action_scale")
        if io_changed or state["critic_names"] != self.critic_names:
            # Outros exercicios (outra tarefa) ou canais novos: o critico e o ator
            # ganham as entradas (e as saidas) novas com peso zero e guardam as
            # que ja tinham.
            network = state.get("network", self.config.network)
            old = ActorCritic(len(state["obs_names"]), len(state["critic_names"]), len(state["action_names"]),
                              actor_hidden=network["actor"], critic_hidden=network["critic"]).to(self.device)
            old.load_state_dict(state["model"])
            fresh = migrate_inputs(old, list(state["obs_names"]), list(self.obs_names), list(state["critic_names"]),
                                   list(self.critic_names), list(state["action_names"]), list(self.action_names),
                                   self.model.log_std.detach().cpu())
            self.model.load_state_dict(fresh.state_dict())
            fresh_optimizer = True
            if io_changed:
                added_in = sorted(set(self.obs_names) - set(state["obs_names"]))
                added_out = sorted(set(self.action_names) - set(state["action_names"]))
                self.restore_notes.append(f"o contrato ganhou {len(added_in)} entradas e {len(added_out)} saidas (peso zero)")
                if old_scale is not None:
                    # (A escala de cada saida pelo nome: as novas ficam como a de agora.)
                    previous = {name: float(v) for name, v in zip(state["action_names"], np.asarray(old_scale).tolist())}
                    current = self.action_scale.detach().cpu().numpy()
                    old_scale = np.asarray([previous.get(name, float(current[j])) for j, name in enumerate(self.action_names)],
                                           dtype=np.float32)
            else:
                self.restore_notes.append("o critico migrou para os exercicios da tarefa nova")
        else:
            self.model.load_state_dict(state["model"])
        # Um ajuste fora do treino (o recentramento, a normalizacao refeita) pede o
        # Adam de novo: o embalo era do ator de antes.
        if state.get("fresh_optimizer"):
            fresh_optimizer = True
            self.restore_notes.append("ajuste fora do treino: o otimizador recomeca")
        if old_scale is not None:
            # Os tetos mudaram: a mesma acao fisica (u novo = u antigo x teto antigo / teto novo).
            factor = torch.as_tensor(np.asarray(old_scale, dtype=np.float32), device=self.device) / self.action_scale
            if float((factor - 1.0).abs().max()) > 1e-6:
                rescale_actions(self.model, factor)
                fresh_optimizer = True
                self.restore_notes.append("os tetos mudaram: a acao foi reescalada (a mesma acao fisica)")
        if fresh_optimizer:
            # (O embalo do Adam era das formas e escalas de antes: recomeca; a taxa fica.)
            self.ppo = PPO(self.model, self.settings)
            self.ppo.learning_rate = float(state["ppo"].get("learning_rate", self.settings.learning_rate))
            if state.get("learning_rate_override"):
                self.ppo.learning_rate = float(state["learning_rate_override"])
            for group in self.ppo.optimizer.param_groups:
                group["lr"] = self.ppo.learning_rate
        else:
            self.ppo.load_state_dict(state["ppo"])
        self.counters.update(state.get("counters", {}))
        lost = self.db.rewind(self.run, self.counters["iteration"])
        if lost:
            self.db.event(self.run, "retomada", f"retomado da rodada {self.counters['iteration']}; "
                                                f"{lost} rodadas depois do checkpoint se perderam")
        if "rng_torch" in state:
            torch.set_rng_state(state["rng_torch"].cpu())
        # Retomada: o ruido da GPU segue de onde parou e os mundos recomecam com
        # uma semente nova (antes cada sessao repetia as sementes da primeira).
        if self.device.type == "cuda" and state.get("rng_cuda") is not None:
            torch.cuda.set_rng_state(state["rng_cuda"].cpu())
        elif self.device.type == "cuda":
            torch.cuda.manual_seed_all(self.config.seed * 1000003 + self.counters["iteration"])
        self.batch.reset(self.config.seed * 1000003 + self.counters["iteration"])

    def status(self) -> dict:
        return {
            "run": self.run, "kind": KIND, "task": self.config.task, "taskName": self.batch.task_name,
            "contract": self.batch.contract_version, "state": self.state, "mode": self.mode.name,
            "modeLabel": self.mode.label, "threads": self.threads, "device": str(self.device),
            "environments": self.config.environments, "iteration": self.counters["iteration"],
            "steps": self.counters["steps"], "sps": self.recent_sps, "trainSeconds": self.counters["train_seconds"],
            "sessionSeconds": time.time() - self.started, "episodes": self.counters["episodes"],
            "last": self.last_iteration_stats, "evaluation": self.last_evaluation, "authority": self.authority(),
            "cpuTemperature": system.cpu_temperature(), "memoryMB": system.memory_mb(),
            "gpuMemoryMB": torch.cuda.memory_allocated() / 1e6 if self.device.type == "cuda" else 0.0,
            "message": self.message, "pid": os.getpid(), "updated": time.time(),
        }

    def write_status(self) -> None:
        storage.write_json_atomic(self.directory / "status.json", self.status())

    def export(self, name: str = "policy_latest") -> Path:
        """A politica para o jogo (o .mrlpolicy do PolicyRuntime3D): o ator com a
        escala do teto embutida na ultima camada (a saida ja e a acao do
        contrato), a normalizacao, e no cabecalho o contrato, a autoridade e os
        tetos com que ela treinou."""
        model = copy.deepcopy(self.model).cpu()
        with torch.no_grad():
            last = [m for m in model.actor if isinstance(m, torch.nn.Linear)][-1]
            scale = self.action_scale.cpu()
            last.weight.mul_(scale.unsqueeze(1))
            last.bias.mul_(scale)
        live = json.loads(self.live_task.read_text())
        path = self.directory / "exports" / f"{name}.mrlpolicy"
        export_policy(model, path, {
            "observationLayout": "locomocao.observacao", "observationVersion": int(self.batch.contract_version),
            "actionLayout": "locomocao.acao", "bodyId": self.batch.body_id, "bodyHash": self.batch.contract_hash,
            "taskId": self.config.task, "run": self.run, "iteration": self.counters["iteration"],
            "steps": self.counters["steps"], "observationNames": list(self.obs_names),
            "contractHash": self.batch.contract_hash, "authority": self.authority(), "caps": live.get("caps", {}),
            "turnMove": live.get("turnMove", {}),
        })
        return path

    def save(self, kind: str = "periodico") -> None:
        self.drain()
        reference = storage.save_checkpoint(self.directory, self.counters["iteration"], self.checkpoint_state(),
                                            self.manifest())
        self.export()
        self.db.log_checkpoint(self.run, self.counters["iteration"], self.counters["steps"], reference.path, kind, None)
        for iteration in storage.prune_checkpoints(self.directory, set()):
            self.db.forget_checkpoint(self.run, iteration)
        self.db.compact(self.run)
        size = storage.directory_size_mb(paths.DATA) / 1024.0
        if size > self.config.disk_cap_gb:
            self.db.event(self.run, "disco", f"os treinos ocupam {size:.1f} GB (teto {self.config.disk_cap_gb} GB)")
        self.last_save = time.time()

    # --- avaliacao -------------------------------------------------------------------
    def evaluate(self) -> dict:
        """Cada exercicio com a acao media, o primeiro episodio de cada mundo: a
        rede inteira, a rede sem a pose e a rigidez (a ablacao do portao) e o
        classico (autoridade zero). As medidas vao para o banco."""
        self.drain()
        previous, self.state = self.state, "avaliando"
        self.write_status()
        results: dict = {}
        a = self.authority()
        # (autoridades, virada brusca: 0 a da tarefa, 1 desligada, 2 ligada). Com a
        # virada na tarefa: a rede com ela, a rede sem ela, ela sem a rede e o classico
        # sem ela; sem a virada na tarefa, os tres de sempre.
        turn_task = bool(json.loads(self.live_task.read_text()).get("turnMove", {}).get("enabled", False))
        solvers = {"rede": ((a, a, a, a), 0), "sem_pose_rigidez": ((a, a, 0.0, 0.0), 0),
                   "classico": ((0.0, 0.0, 0.0, 0.0), 1 if turn_task else 0)}
        if turn_task:
            solvers["rede_sem_virada"] = ((a, a, a, a), 1)
            solvers["virada_sem_rede"] = ((0.0, 0.0, 0.0, 0.0), 2)
        col = {name: i for i, name in enumerate(self.episode_columns)}
        # (O classico nao depende da rede: o resumo dele fica em cache pela
        # tarefa - o arquivo inteiro, com as ajudas - e pelo contrato.)
        import hashlib
        task_key = hashlib.sha256(Path(self.live_task).read_bytes()).hexdigest()[:16] + self.batch.contract_hash
        if not hasattr(self, "classic_cache"):
            self.classic_cache = {}
        for drill in self.drill_ids:
            batch = self.evaluation_batches.get(drill)
            if batch is None:
                batch = self.native.LocomotionBatch(str(self.live_task), self.assets,
                                                    environments=self.config.evaluation_worlds, threads=self.threads,
                                                    seed=20261016, evaluation=True,
                                                    evaluation_seconds=self.config.evaluation_seconds, drill=drill)
                self.evaluation_batches[drill] = batch
            else:
                try:
                    batch.update_task(str(self.live_task))
                except Exception:  # noqa: BLE001 (outras ajudas ou exercicios: um lote novo)
                    batch = self.native.LocomotionBatch(str(self.live_task), self.assets,
                                                        environments=self.config.evaluation_worlds,
                                                        threads=self.threads, seed=20261016, evaluation=True,
                                                        evaluation_seconds=self.config.evaluation_seconds, drill=drill)
                    self.evaluation_batches[drill] = batch
            for solver, (authority, turn_mode) in solvers.items():
                cached = self.classic_cache.get((task_key, drill)) if solver == "classico" else None
                if cached is not None:
                    results.setdefault(solver, {})[drill] = cached
                    self.db.log_body_evaluation(self.run, self.counters["iteration"], self.counters["steps"], drill,
                                                solver, cached)
                    continue
                batch.set_authority(*authority)
                if turn_task:
                    batch.set_turn_move(turn_mode)
                batch.reset(20261016)
                worlds = batch.num_envs
                first: dict[int, np.ndarray] = {}
                steps = int(self.config.evaluation_seconds / batch.policy_dt) + 2
                with torch.no_grad():
                    for _ in range(steps):
                        if solver == "classico":
                            # (Acao zero: com a da rede, os trancos e o residuo do
                            # classico mediam a rede, embora ela nao agisse.)
                            batch.step(np.zeros((worlds, batch.action_size), dtype=np.float32))
                        else:
                            obs = self.tensor(batch.observations())
                            mean = self.model.actor(self.model.obs_norm(obs))
                            batch.step(np.ascontiguousarray((mean * self.action_scale).cpu().numpy(),
                                                            dtype=np.float32))
                        for row in batch.finished_episodes():
                            first.setdefault(int(row[col["mundo"]]), np.array(row))
                        if len(first) >= worlds:
                            break
                rows = np.array(list(first.values())) if first else np.zeros((0, len(self.episode_columns)))
                summary = drill_summary(rows, self.episode_columns, self.drill_ids, self.end_reasons).get(drill, {})
                summary["worlds"] = worlds
                if solver == "classico":
                    self.classic_cache[(task_key, drill)] = summary
                results.setdefault(solver, {})[drill] = summary
                self.db.log_body_evaluation(self.run, self.counters["iteration"], self.counters["steps"], drill, solver,
                                            summary)
        self.db.commit()
        self.last_evaluation = {"iteration": self.counters["iteration"], "time": time.time(), "solvers": results}
        self.adjust_drill_weights(results.get("rede", {}))
        self.state = previous
        self.write_status()
        return self.last_evaluation

    def adjust_drill_weights(self, evaluation: dict) -> None:
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
        self.load_drill_weights()
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
        if command in ("peso", "tarefa", "modo"):
            self.drain()
        if command == "peso":
            return self.set_weight(arguments["componente"], float(arguments["valor"]), author,
                                   arguments.get("motivo", ""))
        raise ValueError(f"comando desconhecido na locomocao: {command}")

    def set_weight(self, component: str, value: float, author: str, reason: str) -> dict:
        """Um peso da nota (as chaves de "reward" da tarefa) na copia do treino."""
        data = json.loads(self.live_task.read_text())
        reward = data.setdefault("reward", {})
        known = ("velocity", "facing", "style", "contact", "slip", "work", "smooth", "assist", "posture", "residual",
                 "still", "fall")
        if component not in known:
            raise ValueError(f"parcela desconhecida: {component} (as da locomocao: {', '.join(known)})")
        old = reward.get(component, "padrao")
        reward[component] = value
        storage.write_json_atomic(self.live_task, data)
        self.batch.update_task(str(self.live_task))
        self.db.event(self.run, "peso", f"{component}: {old} -> {value}", author, {"motivo": reason})
        return {"ok": True}

    # --- o laco ---------------------------------------------------------------------
    def tensor(self, array) -> torch.Tensor:
        # (Uma copia: os buffers do lote valem so ate o proximo passo.)
        return torch.tensor(np.asarray(array), device=self.device, dtype=torch.float32)

    def collect(self) -> tuple[Rollout, dict]:
        s = self.settings
        batch, net = self.batch, self.collector
        envs = batch.num_envs
        begin = time.time()
        components = np.zeros(batch.reward_count, dtype=np.float64)
        episodes: list[np.ndarray] = []
        bootstrap_sum = 0.0
        context = torch.cuda.stream(self.stream) if self.stream is not None else _Null()
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
                batch.step(np.ascontiguousarray((action * self.action_scale).cpu().numpy(), dtype=np.float32))
                reward = self.tensor(batch.rewards())
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
        # O ganho da rede sobre a virada brusca (1 + a acao fisica, por familia): se
        # nao cai a zero onde o professor ajuda, e se a rede o mexe.
        gates = [j for j, name in enumerate(self.action_names) if name.startswith("turno.ganho.")]
        turn_gate = ([float(1.0 + (rollout.actions[..., j] * self.action_scale[j]).mean()) for j in gates]
                     if gates else [])
        # A saturacao da media por grupo (a acao aplicada para de mudar fora dos
        # limites) e os trancos da acao media (sem o ruido), na unidade do teto.
        with torch.no_grad():
            mean = rollout.mean
            outside = (mean < self.action_lower) | (mean > self.action_upper)
            saturation = {}
            for group in ACTION_GROUPS:
                index = [j for j, g in enumerate(self.action_groups) if g == group]
                if index:
                    saturation[group] = float(outside[..., index].float().mean())
            applied = torch.maximum(torch.minimum(mean, self.action_upper), self.action_lower)
            alive = (1.0 - rollout.dones[:-1]).unsqueeze(-1)
            jerk_mean = float(((applied[1:] - applied[:-1]).pow(2) * alive).mean() / alive.mean().clamp(min=1e-6))
        return rollout, {"collect": time.time() - begin, "samples": samples, "components": components / samples,
                         "rows": rows, "bootstrap": bootstrap_sum / samples, "reward": float(rollout.rewards.mean()),
                         "turn_gate": turn_gate, "saturation": saturation, "jerk_mean_action": jerk_mean}

    def _init_worker(self) -> None:
        # A thread da atualizacao com o contexto da GPU desde o comeco.
        if self.device.type == "cuda":
            torch.cuda.set_device(self.device.index or 0)

    def update(self, rollout: Rollout) -> dict:
        """A atualizacao (na thread da GPU, com a sobreposicao)."""
        begin = time.time()
        rollout.generator = self.update_generator
        if self.config.overlap:
            # O KL da taxa contra a politica do comeco desta atualizacao.
            with torch.no_grad():
                dist = self.model.distribution(rollout.obs.reshape(-1, rollout.obs.shape[-1]))
                rollout.anchor_mean = dist.mean.reshape(rollout.mean.shape)
                rollout.anchor_std = dist.stddev.reshape(rollout.std.shape)
                # (O PPO desacoplado: o log-prob das acoes na politica de agora.)
                rollout.anchor_log_probs = dist.log_prob(
                    rollout.actions.reshape(-1, rollout.actions.shape[-1])).sum(-1).reshape(rollout.log_probs.shape)
        with torch.no_grad():
            returns, values = rollout.returns.reshape(-1), rollout.values.reshape(-1)
            explained = float(1.0 - (returns - values).var() / returns.var().clamp(min=1e-8))
        auxiliary = None
        if self.mirror is not None:
            raw = rollout.raw.reshape(-1, rollout.raw.shape[-1])
            weight = self.config.mirror_loss

            def auxiliary():
                index = torch.randint(0, raw.shape[0], (min(4096, raw.shape[0]),), device=raw.device,
                                      generator=self.update_generator)
                return weight * self.mirror(self.model, raw[index], raw=True, norm=rollout.norm)
        stats = self.ppo.update(rollout, auxiliary=auxiliary)
        self.apply_std_bounds()
        stats["explained_variance"] = explained
        if self.device.type == "cuda":
            torch.cuda.synchronize()
        stats["update"] = time.time() - begin
        return stats

    def sync(self) -> None:
        with torch.no_grad():
            self.model.obs_norm.load_state_dict(self.obs_candidate.state_dict())
            self.model.critic_norm.load_state_dict(self.critic_candidate.state_dict())
            self.collector.load_state_dict(self.model.state_dict())

    def finish(self, stats: dict, info: dict) -> None:
        self.sync()
        self.counters["iteration"] += 1
        self.counters["steps"] += info["samples"]
        self.counters["train_seconds"] += info["collect"] + stats["update"]
        rows = info["rows"]
        self.counters["episodes"] += len(rows)
        col = {name: i for i, name in enumerate(self.episode_columns)}
        reasons = rows[:, col["razao"]].astype(int) if len(rows) else np.zeros(0, dtype=int)
        ends = {self.end_reasons[r]: int((reasons == r).sum()) for r in set(reasons.tolist())}
        fall_codes = [self.end_reasons.index(n) for n in ("queda", "numerico", "falha")]
        falls = int(np.isin(reasons, fall_codes).sum())
        world_seconds = info["samples"] * self.batch.policy_dt
        fall_risk = falls * self.episode_seconds / max(1e-6, world_seconds)
        # A rodada leva o maior dos dois tempos com a sobreposicao.
        seconds = max(info["collect"], stats["update"]) if self.config.overlap else info["collect"] + stats["update"]
        sps = info["samples"] / max(1e-6, seconds)
        self.recent_sps = sps if self.recent_sps == 0.0 else 0.9 * self.recent_sps + 0.1 * sps
        ppo = {key: stats[key] for key in ("kl", "kl_max", "clip_fraction", "grad_norm", "critic_grad_norm",
                                           "explained_variance", "value_loss", "learning_rate", "action_std",
                                           "auxiliary") if key in stats}
        details = {"authority": self.authority(), "fall_risk": fall_risk, "bootstrap": info["bootstrap"], "ppo": ppo,
                   "turn_gate": info.get("turn_gate", []),
                   "drills": drill_summary(rows, self.episode_columns, self.drill_ids, self.end_reasons)}
        # O diagnostico de 18/10 (vault .../pesquisas/08_plano_consenso.md, Q0).
        std = self.model.log_std.detach().exp().cpu().numpy()
        kl_channels = stats.get("kl_per_channel") or []
        top = sorted(zip(self.action_names, kl_channels), key=lambda item: -item[1])[:5]
        details["diag"] = {
            "saturation": info.get("saturation", {}),
            "jerk_mean_action": info.get("jerk_mean_action"),
            "kl_top5": [[name, value] for name, value in top],
            "clip": ({group: float(np.mean([v for v, g in zip(stats["clip_per_channel"], self.action_groups) if g == group]))
                      for group in ACTION_GROUPS if group in self.action_groups} if stats.get("clip_per_channel") else {}),
            "kl_adapt": stats.get("kl_adapt"), "bound_loss": stats.get("bound_loss"),
            "std": {group: {"p50": float(np.median(std[[j for j, g in enumerate(self.action_groups) if g == group]])),
                            "p95": float(np.percentile(std[[j for j, g in enumerate(self.action_groups) if g == group]], 95))}
                    for group in ACTION_GROUPS if group in self.action_groups},
            "saved_starts": {d: int(self.batch.saved_start_count(i)) for i, d in enumerate(self.drill_ids)},
        }
        if self.curriculum.rule.enabled:
            self.curriculum.observe(rows, self.episode_columns)
            live = json.loads(self.live_task.read_text())
            opened = self.curriculum.advance(live)
            if opened:
                storage.write_json_atomic(self.live_task, live)
                self.batch.update_task(str(self.live_task))
                self.db.event(self.run, "curriculo", "; ".join(opened))
            details["curriculum"] = self.curriculum.snapshot(live)
        result = {
            "run": self.run, "iteration": self.counters["iteration"], "time": time.time(),
            "steps": self.counters["steps"], "sps": sps, "reward": info["reward"], "episodes": int(len(rows)),
            "episode_return": float(rows[:, col["retorno"]].mean()) if len(rows) else None,
            "episode_seconds": float(rows[:, col["segundos"]].mean()) if len(rows) else None,
            "fall_fraction": float(np.isin(reasons, fall_codes).mean()) if len(rows) else None,
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
        self.advance_authority(fall_risk)
        self.apply_authority()
        self.last_iteration_stats = {k: result[k] for k in (
            "iteration", "sps", "reward", "episodes", "episode_return", "episode_seconds", "fall_fraction", "kl",
            "entropy", "learning_rate", "action_std", "value_loss")}
        self.last_iteration_stats["authority"] = self.authority()
        self.last_iteration_stats["fall_risk"] = fall_risk
        self.db.log_iteration(result)
        self.db.commit()

    def drain(self) -> None:
        """Espera a atualizacao em andamento (salvar, avaliar, pausar, parar)."""
        if getattr(self, "pending", None) is not None:
            future, info = self.pending
            self.pending = None
            self.finish(future.result(), info)

    def round(self) -> None:
        """Coleta uma rodada e atualiza (com a sobreposicao, a atualizacao desta
        rodada roda enquanto a proxima coleta)."""
        rollout, info = self.collect()
        if self.executor is None:
            self.finish(self.update(rollout), info)
            return
        self.drain()
        self.pending = (self.executor.submit(self.update, rollout), info)

    def healthy(self, result: dict) -> bool:
        values = [result["policy_loss"], result["value_loss"], result["kl"]]
        if not all(math.isfinite(v) for v in values):
            return False
        return all(torch.isfinite(p).all() for p in self.model.parameters())

    def recover(self) -> None:
        reference = storage.latest_checkpoint(self.directory)
        if reference is None:
            raise RuntimeError("a rede ficou invalida antes do primeiro checkpoint")
        state = storage.load_checkpoint(reference, str(self.device))
        self.model.load_state_dict(state["model"])
        self.ppo.load_state_dict(state["ppo"])
        self.fix_raw_channels()
        self.apply_std_bounds()
        self.reset_candidates()
        self.collector.load_state_dict(self.model.state_dict())
        self.counters.update(state.get("counters", {}))
        self.db.rewind(self.run, self.counters["iteration"])
        self.ppo.learning_rate = max(self.settings.min_learning_rate, self.ppo.learning_rate * 0.5)
        for group in self.ppo.optimizer.param_groups:
            group["lr"] = self.ppo.learning_rate
        self.batch.reset(int(time.time()))
        self.db.event(self.run, "recuperacao", f"valores invalidos: voltou ao checkpoint {reference.iteration}")

    def loop(self) -> int:
        self.last_save = time.time()
        last_status = 0.0
        last_evaluation = time.time()
        code = 0
        try:
            self.save("inicio")
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
                    self.save()
                if now - last_status >= 2.0:
                    self.write_status()
                    last_status = now
            code = system.RESTART_CODE if self.restart_requested and not self.stop_requested else 0
            if self.max_iterations and not self.stop_requested:
                self.stop_reason = "rodadas"
        except Exception as error:  # noqa: BLE001
            self.message = f"erro: {error}"
            self.state = "erro"
            self.db.event(self.run, "erro", str(error))
            code = 1
            raise
        finally:
            self.state = "parando"
            if code != 1:
                self.drain()
            self.config.end_reason = "erro" if code == 1 else ("" if code == system.RESTART_CODE else self.stop_reason)
            self.config.save(self.directory / "run.json")
            try:
                if all(torch.isfinite(p).all() for p in self.model.parameters()):
                    self.save("final")
            finally:
                self.state = "parado"
                self.write_status()
                self.db.end_session(self.session, "reinicio" if code == system.RESTART_CODE else
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
