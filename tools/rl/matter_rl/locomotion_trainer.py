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
from .model import ActorCritic
from .ppo import PPO, PPOSettings, Rollout
from .symmetry import MirrorLoss

TASK_SCHEMA = "matter-rl-locomocao-1"
KIND = "locomocao"
NETWORK = {"actor": [512, 512, 256], "critic": [512, 512, 256]}
# Os canais da entrada que passam crus pela normalizacao: os do critico que so o
# treino sabe (o exercicio, um-quente) - os outros normalizam.
RAW_PREFIXES = ("drill.",)
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
    # A rampa de entrada da autoridade (rodadas de 0 a 1, todas as cabecas).
    authority_ramp_rounds: int = 30
    mirror_loss: float = 0.3
    evaluation_minutes: float = 15.0
    evaluation_worlds: int = 24
    evaluation_seconds: float = 20.0
    checkpoint_minutes: float = 10.0
    disk_cap_gb: float = 40.0
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
            "targets_per_minute": float(part[:, col["alvos_por_minuto"]].mean()),
            "hold_drift": mean_of("deriva_espera"),
            "arc_error": mean_of("afastamento_curva"),
        }
    return out


class LocomotionTrainer:
    def __init__(self, run: str, task: str | None = None, mode: str | None = None, environments: int | None = None,
                 device: str = "auto", seed: int | None = None, socket_path: Path | None = None,
                 iterations: int | None = None):
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
        if fresh and os.environ.get("MATTER_RL_PPO"):
            self.config.ppo.update(json.loads(os.environ["MATTER_RL_PPO"]))
        self.config.end_reason = ""
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
        self.live_task = self.directory / "tarefa.json"
        if not self.live_task.exists():
            storage.write_json_atomic(self.live_task, json.loads(self.task_path.read_text()))
        self.assets = str(paths.ASSETS)
        self.batch = self.native.LocomotionBatch(str(self.live_task), self.assets, environments=self.config.environments,
                                                 threads=self.threads, seed=self.config.seed)
        self.obs_names = self.batch.observation_names()
        self.critic_names = self.batch.critic_names()
        self.action_names = self.batch.action_names()
        self.reward_names = self.batch.reward_names()
        self.episode_columns = self.batch.episode_columns()
        self.end_reasons = self.batch.end_reasons()
        self.drill_ids = list(self.batch.drill_ids)
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
        self.counters = {"iteration": 0, "steps": 0, "train_seconds": 0.0, "episodes": 0}
        self.db = storage.Database(paths.DATABASE)
        self.db.ensure_run(run, self.config.task, self.batch.contract_hash, asdict(self.config))
        self.restore()
        self.fix_raw_channels()
        self.reset_candidates()
        self.collector = copy.deepcopy(self.model)
        self.apply_authority()
        self.load_drill_weights()

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
        for norm, names in ((self.model.obs_norm, self.obs_names), (self.model.critic_norm, self.critic_names)):
            norm.fix_channels(torch.tensor([name.startswith(RAW_PREFIXES) for name in names]))

    def reset_candidates(self) -> None:
        self.obs_candidate = copy.deepcopy(self.model.obs_norm)
        self.critic_candidate = copy.deepcopy(self.model.critic_norm)

    def authority(self) -> float:
        rounds = max(1, int(self.config.authority_ramp_rounds))
        return min(1.0, (self.counters["iteration"] + 1) / rounds)

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
                "rng_torch": torch.get_rng_state()}

    def restore(self) -> None:
        reference = storage.latest_checkpoint(self.directory)
        if reference is None:
            return
        state = storage.load_checkpoint(reference, str(self.device))
        if state.get("contract_hash") != self.batch.contract_hash or state["action_names"] != self.action_names \
                or state["obs_names"] != self.obs_names or state["critic_names"] != self.critic_names:
            raise RuntimeError("o checkpoint e de outro contrato: abra um treino novo")
        self.model.load_state_dict(state["model"])
        self.ppo.load_state_dict(state["ppo"])
        self.counters.update(state.get("counters", {}))
        lost = self.db.rewind(self.run, self.counters["iteration"])
        if lost:
            self.db.event(self.run, "retomada", f"retomado da rodada {self.counters['iteration']}; "
                                                f"{lost} rodadas depois do checkpoint se perderam")
        if "rng_torch" in state:
            torch.set_rng_state(state["rng_torch"].cpu())

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

    def save(self, kind: str = "periodico") -> None:
        reference = storage.save_checkpoint(self.directory, self.counters["iteration"], self.checkpoint_state(),
                                            self.manifest())
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
        previous, self.state = self.state, "avaliando"
        self.write_status()
        results: dict = {}
        a = self.authority()
        solvers = {"rede": (a, a, a, a), "sem_pose_rigidez": (a, a, 0.0, 0.0), "classico": (0.0, 0.0, 0.0, 0.0)}
        col = {name: i for i, name in enumerate(self.episode_columns)}
        for drill in self.drill_ids:
            batch = self.evaluation_batches.get(drill)
            if batch is None:
                batch = self.native.LocomotionBatch(str(self.live_task), self.assets,
                                                    environments=self.config.evaluation_worlds, threads=self.threads,
                                                    seed=20261016, evaluation=True,
                                                    evaluation_seconds=self.config.evaluation_seconds, drill=drill)
                self.evaluation_batches[drill] = batch
            else:
                batch.update_task(str(self.live_task))
            for solver, authority in solvers.items():
                batch.set_authority(*authority)
                batch.reset(20261016)
                worlds = batch.num_envs
                first: dict[int, np.ndarray] = {}
                steps = int(self.config.evaluation_seconds / batch.policy_dt) + 2
                with torch.no_grad():
                    for _ in range(steps):
                        obs = self.tensor(batch.observations())
                        mean = self.model.actor(self.model.obs_norm(obs))
                        batch.step(np.ascontiguousarray((mean * self.action_scale).cpu().numpy(), dtype=np.float32))
                        for row in batch.finished_episodes():
                            first.setdefault(int(row[col["mundo"]]), np.array(row))
                        if len(first) >= worlds:
                            break
                rows = np.array(list(first.values())) if first else np.zeros((0, len(self.episode_columns)))
                summary = drill_summary(rows, self.episode_columns, self.drill_ids, self.end_reasons).get(drill, {})
                summary["worlds"] = worlds
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
        if command == "peso":
            return self.set_weight(arguments["componente"], float(arguments["valor"]), author,
                                   arguments.get("motivo", ""))
        raise ValueError(f"comando desconhecido na locomocao: {command}")

    def set_weight(self, component: str, value: float, author: str, reason: str) -> dict:
        """Um peso da nota (as chaves de "reward" da tarefa) na copia do treino."""
        data = json.loads(self.live_task.read_text())
        reward = data.setdefault("reward", {})
        known = ("velocity", "facing", "style", "contact", "slip", "work", "smooth", "assist", "posture", "residual",
                 "fall")
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
        with torch.no_grad():
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
        samples = s.steps_per_env * envs
        rows = np.concatenate(episodes) if episodes else np.zeros((0, len(self.episode_columns)))
        return rollout, {"collect": time.time() - begin, "samples": samples, "components": components / samples,
                         "rows": rows, "bootstrap": bootstrap_sum / samples, "reward": float(rollout.rewards.mean())}

    def update(self, rollout: Rollout) -> dict:
        begin = time.time()
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
        stats = self.ppo.update(rollout, auxiliary=auxiliary)
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
        self.apply_authority()
        rows = info["rows"]
        self.counters["episodes"] += len(rows)
        col = {name: i for i, name in enumerate(self.episode_columns)}
        reasons = rows[:, col["razao"]].astype(int) if len(rows) else np.zeros(0, dtype=int)
        ends = {self.end_reasons[r]: int((reasons == r).sum()) for r in set(reasons.tolist())}
        fall_codes = [self.end_reasons.index(n) for n in ("queda", "numerico", "falha")]
        seconds = info["collect"] + stats["update"]
        sps = info["samples"] / max(1e-6, seconds)
        self.recent_sps = sps if self.recent_sps == 0.0 else 0.9 * self.recent_sps + 0.1 * sps
        ppo = {key: stats[key] for key in ("kl", "kl_max", "clip_fraction", "grad_norm", "critic_grad_norm",
                                           "explained_variance", "value_loss", "learning_rate", "action_std",
                                           "auxiliary") if key in stats}
        details = {"authority": self.authority(), "bootstrap": info["bootstrap"], "ppo": ppo,
                   "drills": drill_summary(rows, self.episode_columns, self.drill_ids, self.end_reasons)}
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
        self.last_iteration_stats = {k: result[k] for k in (
            "iteration", "sps", "reward", "episodes", "episode_return", "episode_seconds", "fall_fraction", "kl",
            "entropy", "learning_rate", "action_std", "value_loss")}
        self.last_iteration_stats["authority"] = self.authority()
        self.db.log_iteration(result)
        self.db.commit()

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
                        self.save("pausa")
                        self.state = "pausado"
                        self.write_status()
                    time.sleep(0.2)
                    continue
                if self.max_iterations and self.counters["iteration"] >= self.max_iterations:
                    break
                self.state = "treinando"
                rollout, info = self.collect()
                self.finish(self.update(rollout), info)
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
                self.lock.release()
        return code
