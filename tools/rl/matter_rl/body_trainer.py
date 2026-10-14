"""O treino do Corpo v2 (plano_claude_rl.md): o lote v2 (o contrato v2, o piloto
e as notas novas), o PPO com as redes do plano, o juiz de estilo (AMP) com o
botao beta e a atualizacao da GPU sobreposta a coleta seguinte.

O contrato 1 (a imitacao residual, o professor, o curriculo) continua no
trainer.py. Os dois dividem o banco, o canal de controle, os checkpoints e a
exportacao para o C++.

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
import signal
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import torch

from . import control, native, paths, storage, system
from .amp import Judge, JudgeSettings, style_score
from .model import ActorCritic, export_policy
from .ppo import PPO, PPOSettings, Rollout

RESTART_CODE = system.RESTART_CODE
TASK_SCHEMA = "matter-rl-task-2"
NETWORK = {"actor": [1024, 512, 256], "critic": [1024, 512, 256]}
# Os canais que passam crus pela normalizacao: os comandos e os dados do
# jogador (o contrato ja os escala) e, no critico, o exercicio e a camera.
RAW_PREFIXES = ("command.", "player.", "drill.", "pilot.")


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
    overlap: bool = True
    checkpoint_minutes: float = 10.0
    export_minutes: float = 1.0
    disk_cap_gb: float = 10.0
    ppo: dict = field(default_factory=lambda: PPOSettings().to_dict())
    network: dict = field(default_factory=lambda: dict(NETWORK))
    judge: dict = field(default_factory=lambda: JudgeSettings().to_dict())

    @staticmethod
    def load(path: Path) -> "BodyRunConfig":
        data = json.loads(path.read_text())
        known = {k: v for k, v in data.items() if k in BodyRunConfig.__dataclass_fields__}
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
        out[drill] = {
            "episodes": int(len(part)),
            "falls": float(np.isin(reasons, [1, 2]).mean()),
            "seconds": float(part[:, col["segundos"]].mean()),
            "velocity_error": float(part[:, col["erro_velocidade"]].mean()),
            "facing_error": float(part[:, col["erro_olhar"]].mean()),
            "reaction": float(reaction.mean()) if len(reaction) else None,
            "slip": float(part[:, col["escorregar"]].mean()),
            "work": float(part[:, col["trabalho"]].mean()),
            "saturation": float(part[:, col["no_teto"]].mean()),
            "reversal": float(part[:, col["reversao"]].mean()),
            "cadence": float(part[:, col["cadencia"]].mean()),
        }
    return out


class BodyTrainer:
    def __init__(self, run: str, task: str | None = None, mode: str | None = None, environments: int | None = None,
                 device: str = "auto", seed: int | None = None, socket_path: Path | None = None,
                 iterations: int | None = None, style: bool | None = None, overlap: bool | None = None):
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
        self.config.save(config_path)
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
        self.batch = self.native.BodyBatch(str(self.live_task), environments=self.config.environments, per_scene=4,
                                           threads=self.threads, seed=self.config.seed)
        self.obs_names = self.batch.observation_names()
        self.critic_names = self.batch.critic_names()
        self.action_names = self.batch.action_names()
        self.reward_names = self.batch.reward_names()
        self.episode_columns = self.batch.episode_columns()
        self.drill_ids = list(self.batch.drill_ids)

        radians = np.array(self.batch.action_radians_per_unit(), dtype=np.float32)
        initial_std = np.clip(0.15 / radians, 0.1, 1.5)
        network = self.config.network
        self.model = ActorCritic(self.batch.obs_size, self.batch.critic_size, self.batch.action_size, initial_std,
                                 actor_hidden=network["actor"], critic_hidden=network["critic"]).to(self.device)
        self.settings = PPOSettings.from_dict(self.config.ppo)
        self.ppo = PPO(self.model, self.settings)
        self.judge: Judge | None = None
        if self.config.style:
            dataset = self.native.motion_dataset(live["body"], live["style"]["dataset"])
            self.judge = Judge(dataset, JudgeSettings.from_dict(self.config.judge), self.device, self.config.seed)
        self.counters = {"iteration": 0, "steps": 0, "train_seconds": 0.0, "episodes": 0}
        self.db = storage.Database(paths.DATABASE)
        self.db.ensure_run(run, self.config.task, self.batch.body_hash, asdict(self.config))
        self.restore()
        self.fix_raw_channels()
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
            norm.fix_channels(torch.tensor([name.startswith(RAW_PREFIXES) for name in names]))

    def manifest(self) -> dict:
        return {"run": self.run, "task": self.config.task, "contract": 2, "body_id": self.batch.body_id,
                "body_hash": self.batch.body_hash, "task_hash": self.batch.task_hash, "steps": self.counters["steps"],
                "time": time.time(), "obs_names": self.obs_names, "critic_names": self.critic_names,
                "action_names": self.action_names, "beta": self.beta, "style": self.config.style}

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
            "episodes": self.counters["episodes"], "last": self.last_iteration_stats, "evaluation": {}, "best": {},
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
        elif component in ("work", "torque", "slip", "impact", "head", "smooth", "limits", "contacts"):
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

    def collect(self) -> tuple[Rollout, dict]:
        """Uma rodada de coleta com a copia que coleta (a politica de antes)."""
        s = self.settings
        batch, net = self.batch, self.collector
        envs = batch.num_envs
        begin = time.time()
        context = torch.cuda.stream(self.stream) if self.stream is not None else _Null()
        components = np.zeros(batch.reward_count, dtype=np.float64)
        episodes: list[np.ndarray] = []
        pairs: list[np.ndarray] = []
        style_sum = 0.0
        dt, beta = batch.policy_dt, self.beta
        with context, torch.no_grad():
            rollout = Rollout(s.steps_per_env, envs, batch.obs_size, batch.critic_size, batch.action_size, self.device)
            obs, critic = self.tensor(batch.observations()), self.tensor(batch.critic_observations())
            for _ in range(s.steps_per_env):
                net.obs_norm.update(obs)
                net.critic_norm.update(critic)
                obs_n, critic_n = net.obs_norm(obs), net.critic_norm(critic)
                dist = net.distribution(obs_n)
                action = dist.sample()
                log_prob = dist.log_prob(action).sum(-1)
                value = net.value(critic_n)
                batch.step(np.ascontiguousarray(action.cpu().numpy(), dtype=np.float32),
                           np.ascontiguousarray(dist.mean.cpu().numpy(), dtype=np.float32))
                reward = self.tensor(batch.rewards())
                if self.scorer is not None:
                    step_pairs = np.array(batch.style_pairs(), dtype=np.float32)
                    pairs.append(step_pairs)
                    style = style_score(self.scorer(self.judge.normalize(self.tensor(step_pairs))))
                    reward = reward + dt * beta * style
                    style_sum += float(style.sum())
                terminated = self.tensor(batch.terminated())
                truncated_np = np.asarray(batch.truncated())
                truncated = self.tensor(truncated_np)
                if truncated_np.any():
                    # Fim pelo tempo nao e fracasso: o valor do estado final entra.
                    index = np.nonzero(truncated_np)[0]
                    final = self.tensor(np.asarray(batch.terminal_critic_observations())[index])
                    reward[torch.as_tensor(index, device=self.device)] += s.gamma * net.value(net.critic_norm(final))
                rollout.add(obs_n, critic_n, action, log_prob, value, reward, torch.maximum(terminated, truncated),
                            dist.mean, dist.stddev)
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
        stats = self.ppo.update(rollout)
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
            self.model.obs_norm.load_state_dict(self.collector.obs_norm.state_dict())
            self.model.critic_norm.load_state_dict(self.collector.critic_norm.state_dict())
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
        details = {"beta": self.beta, "style": info["style"], "judge": judge,
                   "drills": drill_summary(rows, self.episode_columns, self.drill_ids)}
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
        self.last_iteration_stats = {k: result[k] for k in (
            "iteration", "sps", "reward", "episodes", "episode_return", "episode_seconds", "fall_fraction", "kl",
            "entropy", "learning_rate", "action_std", "value_loss")}
        self.last_iteration_stats["style"] = info["style"]
        self.db.log_iteration(result)
        self.db.commit()

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
        self.collector.load_state_dict(self.model.state_dict())
        self.counters.update(state.get("counters", {}))
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
