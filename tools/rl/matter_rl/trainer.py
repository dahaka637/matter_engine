"""O laco do treino: coletar nos mundos C++, atualizar o PPO na GPU, avaliar,
salvar, atender comandos e cuidar do PC (prioridade, temperatura, disco)."""
from __future__ import annotations

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
from .evaluation import BASE_BATTERY, EPISODE_COLUMNS, Battery, Evaluator, battery_posture
from .model import ActorCritic, export_policy, migrate_inputs
from .ppo import PPO, PPOSettings, Rollout

RESTART_CODE = system.RESTART_CODE


@dataclass
class RunConfig:
    task: str = "h1_ficar_em_pe"
    environments: int = 512
    seed: int = 1
    mode: str = "equilibrado"
    # Por que o ultimo treino fechou: "usuario" (Parar), "sinal" (o PC
    # desligando), "erro" ou "" (ainda aberto ou caiu). So "usuario" impede a
    # retomada automatica ao ligar o PC.
    end_reason: str = ""
    checkpoint_minutes: float = 10.0
    evaluation_minutes: float = 10.0
    # A politica que a tela mostra (policy_latest) sai a cada minuto.
    export_minutes: float = 1.0
    disk_cap_gb: float = 10.0
    ppo: dict = field(default_factory=lambda: PPOSettings().to_dict())

    @staticmethod
    def load(path: Path) -> "RunConfig":
        data = json.loads(path.read_text())
        known = {k: v for k, v in data.items() if k in RunConfig.__dataclass_fields__}
        return RunConfig(**known)

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


class Trainer:
    def __init__(self, run: str, task: str | None = None, mode: str | None = None, environments: int | None = None,
                 device: str = "auto", seed: int | None = None, socket_path: Path = paths.SOCKET,
                 iterations: int | None = None):
        self.run = run
        self.directory = paths.run_dir(run)
        self.lock = storage.RunLock(self.directory)
        storage.clean_temporaries(self.directory)
        config_path = self.directory / "run.json"
        self.config = RunConfig.load(config_path) if config_path.exists() else RunConfig()
        if task:
            self.config.task = task
        if mode:
            self.config.mode = mode
        if environments:
            self.config.environments = environments
        if seed is not None:
            self.config.seed = seed
        self.config.save(config_path)
        self.max_iterations = iterations

        self.mode = system.mode(self.config.mode)
        system.apply_priority(self.mode)
        self.threads = system.threads_for(self.mode)
        torch.set_num_threads(2)
        self.device = choose_device(device)
        self.native = native.load()
        self.task_path = paths.task_path(self.config.task)
        # Uma copia da tarefa no treino: os pesos mudados com ele andando ficam aqui.
        # Outra habilidade (a etapa seguinte) troca a copia pela do repositorio.
        self.live_task = self.directory / "tarefa.json"
        self.switched_from: str | None = None
        if self.live_task.exists():
            previous = json.loads(self.live_task.read_text()).get("id")
            if previous != self.config.task:
                self.switched_from = previous
                self.live_task.unlink()
        if not self.live_task.exists():
            data = json.loads(self.task_path.read_text())
            data["body"] = str((self.task_path.parent / data["body"]).resolve())
            storage.write_json_atomic(self.live_task, data)
        self.batch = self.native.Batch(str(self.live_task), environments=self.config.environments, per_scene=4,
                                       threads=self.threads, seed=self.config.seed)
        self.obs_names = self.batch.observation_names()
        self.critic_names = self.batch.critic_names()
        self.action_names = self.batch.action_names()
        self.reward_names = self.batch.reward_names()

        # O ruido inicial: ~0,15 rad em cada junta (desvio ~0,6 na escala da acao).
        radians = np.array(self.batch.action_radians_per_unit(), dtype=np.float32)
        initial_std = np.clip(0.15 / radians, 0.1, 1.5)
        self.model = ActorCritic(self.batch.obs_size, self.batch.critic_size, self.batch.action_size,
                                 initial_std).to(self.device)
        self.settings = PPOSettings.from_dict(self.config.ppo)
        self.ppo = PPO(self.model, self.settings)
        self.counters = {"iteration": 0, "steps": 0, "train_seconds": 0.0, "episodes": 0}
        # O melhor da habilidade de agora e o de cada uma (os checkpoints deles
        # ficam protegidos da limpeza).
        self.best = {"score": -1e9, "iteration": -1}
        self.bests: dict[str, dict] = {}
        self.last_evaluation: dict = {}
        self.db = storage.Database()
        self.db.ensure_run(run, self.config.task, self.batch.body_hash, asdict(self.config))
        if self.switched_from:
            self.db.event(run, "tarefa", f"habilidade {self.switched_from} -> {self.config.task}")
        self.restore()
        self.fix_commands()

        self.state = "treinando"
        self.message = ""
        self.stop_requested = False
        self.stop_reason = "sinal"
        self.config.end_reason = ""
        self.config.save(self.directory / "run.json")
        self.restart_requested = False
        self.pause_requested = False
        self.started = time.time()
        self.recent_sps = 0.0
        self.last_iteration_stats: dict = {}
        self.hot_since: float | None = None
        self.evaluator: Evaluator | None = None
        self.session = self.db.begin_session(run, self.counters["iteration"], self.mode.name, str(self.device))
        self.db.event(run, "sessao", f"treino aberto no modo {self.mode.label} ({self.threads} threads, "
                                     f"{self.device})")
        self.server = control.ControlServer(self.status, socket_path)
        signal.signal(signal.SIGTERM, self._signal_stop)
        signal.signal(signal.SIGINT, self._signal_stop)

    # --- estado ------------------------------------------------------------------
    def _signal_stop(self, *_):
        self.stop_requested = True
        self.stop_reason = "sinal"

    def manifest(self) -> dict:
        return {
            "run": self.run, "task": self.config.task, "body_id": self.batch.body_id,
            "body_hash": self.batch.body_hash, "task_hash": self.batch.task_hash,
            "steps": self.counters["steps"], "time": time.time(), "best": self.best, "bests": self.bests,
            "obs_names": self.obs_names, "critic_names": self.critic_names, "action_names": self.action_names,
        }

    def checkpoint_state(self) -> dict:
        return {
            "model": self.model.state_dict(),
            "ppo": self.ppo.state_dict(),
            "counters": dict(self.counters),
            "best": dict(self.best),
            "bests": {task: dict(best) for task, best in self.bests.items()},
            "obs_names": self.obs_names,
            "critic_names": self.critic_names,
            "action_names": self.action_names,
            "body_hash": self.batch.body_hash,
            "rng_torch": torch.get_rng_state(),
        }

    def restore(self) -> None:
        reference = storage.latest_checkpoint(self.directory)
        if reference is None:
            return
        state = storage.load_checkpoint(reference, str(self.device))
        if state["action_names"] != self.action_names:
            raise RuntimeError("o checkpoint comanda outras juntas: abra um treino novo para este corpo")
        if state["obs_names"] != self.obs_names or state["critic_names"] != self.critic_names:
            # Entrada nova (etapa nova): a rede cresce sem mudar o que faz.
            old = ActorCritic(len(state["obs_names"]), len(state["critic_names"]), len(self.action_names))
            old.load_state_dict(state["model"])
            self.model = migrate_inputs(old, state["obs_names"], self.obs_names, state["critic_names"],
                                        self.critic_names).to(self.device)
            self.ppo = PPO(self.model, self.settings)
            self.db.event(self.run, "migracao", "a entrada da rede cresceu; pesos antigos preservados")
        else:
            self.model.load_state_dict(state["model"])
            self.ppo.load_state_dict(state["ppo"])
        self.counters.update(state.get("counters", {}))
        # (Checkpoints de antes da lista: o melhor era o da tarefa deles.)
        bests = state.get("bests") or {reference.manifest.get("task", self.config.task): state.get("best", {})}
        self.bests = {task: dict(best) for task, best in bests.items() if best and best.get("iteration", -1) >= 0}
        self.best = dict(self.bests.get(self.config.task, {"score": -1e9, "iteration": -1}))
        lost = self.db.rewind(self.run, self.counters["iteration"])
        if lost:
            self.db.event(self.run, "retomada", f"retomado da rodada {self.counters['iteration']}; "
                                                f"{lost} rodadas depois do checkpoint se perderam")
        self.last_evaluation = self.db.last_evaluation(self.run)
        if "rng_torch" in state:
            torch.set_rng_state(state["rng_torch"].cpu())
        if state.get("body_hash") != self.batch.body_hash:
            self.db.event(self.run, "corpo", "o corpo mudou desde o checkpoint: o treino segue adaptando")

    def fix_commands(self) -> None:
        """Os canais de comando passam crus pela normalizacao (valem 0 ou 1)."""
        for norm, names in ((self.model.obs_norm, self.obs_names), (self.model.critic_norm, self.critic_names)):
            norm.fix_channels(torch.tensor([name.startswith("command.") for name in names]))

    def status(self) -> dict:
        evaluation = self.last_evaluation
        return {
            "run": self.run, "task": self.config.task, "taskName": self.batch.task_name, "state": self.state,
            "mode": self.mode.name, "modeLabel": self.mode.label, "threads": self.threads, "device": str(self.device),
            "environments": self.config.environments, "iteration": self.counters["iteration"],
            "steps": self.counters["steps"], "sps": self.recent_sps, "trainSeconds": self.counters["train_seconds"],
            "sessionSeconds": time.time() - self.started, "episodes": self.counters["episodes"],
            "last": self.last_iteration_stats, "evaluation": evaluation, "best": self.best,
            "cpuTemperature": system.cpu_temperature(), "memoryMB": system.memory_mb(),
            "gpuMemoryMB": torch.cuda.memory_allocated() / 1e6 if self.device.type == "cuda" else 0.0,
            "policy": str(self.directory / "exports" / "policy_latest.mrlpolicy"), "message": self.message,
            "pid": os.getpid(), "updated": time.time(),
        }

    def write_status(self) -> None:
        storage.write_json_atomic(self.directory / "status.json", self.status())

    # --- persistencia --------------------------------------------------------------
    def save(self, kind: str = "periodico") -> None:
        previous = self.state
        self.state = "salvando"
        reference = storage.save_checkpoint(self.directory, self.counters["iteration"], self.checkpoint_state(),
                                            self.manifest())
        self.db.log_checkpoint(self.run, self.counters["iteration"], self.counters["steps"], reference.path, kind,
                               self.last_evaluation.get("score"))
        self.export("policy_latest")
        protected = {best["iteration"] for best in [self.best, *self.bests.values()] if best.get("iteration", -1) >= 0}
        for iteration in storage.prune_checkpoints(self.directory, protected):
            self.db.forget_checkpoint(self.run, iteration)
        self.db.compact(self.run)
        size = storage.directory_size_mb(paths.DATA) / 1024.0
        if size > self.config.disk_cap_gb:
            self.db.event(self.run, "disco", f"os treinos ocupam {size:.1f} GB (teto {self.config.disk_cap_gb} GB)")
        self.last_save = time.time()
        self.state = previous

    def export(self, name: str) -> None:
        export_policy(self.model, self.directory / "exports" / f"{name}.mrlpolicy", {
            "observationLayout": "matter.rl.observation", "observationVersion": 1, "actionLayout": "matter.rl.action",
            "bodyId": self.batch.body_id, "bodyHash": self.batch.body_hash, "taskId": self.config.task,
            "run": self.run, "iteration": self.counters["iteration"], "steps": self.counters["steps"],
        })

    # --- avaliacao -----------------------------------------------------------------
    def evaluate(self) -> None:
        previous = self.state
        self.state = "avaliando"
        self.write_status()
        if self.evaluator is None:
            self.evaluator = Evaluator(self.native, self.live_task, self.threads, Battery())
        record = self.directory / "episodes" / f"aval-{self.counters['iteration']:09d}"
        started = time.time()
        results = self.evaluator.run(self.model, self.device, record)
        for battery, solvers in results.items():
            for solver, metrics in solvers.items():
                self.db.log_evaluation({
                    "run": self.run, "iteration": self.counters["iteration"], "time": time.time(),
                    "steps": self.counters["steps"], "battery": battery, "solver": solver,
                    "stand_fraction": metrics["stand_fraction"], "mean_seconds": metrics["mean_seconds"],
                    "sway": metrics["sway"], "foot_drift": metrics["foot_drift"],
                    "action_rate": metrics["action_rate"], "effort": metrics["effort"], "upright": metrics["upright"],
                    "turn": metrics["turn"], "joint_speed": metrics["joint_speed"], "foot_turn": metrics["foot_turn"],
                    "score": metrics["score"], "ends": json.dumps(metrics["ends"]),
                })
        # A nota e a da pior bateria (uma habilidade nova nao apaga a antiga).
        combined = min(m["score"] for solvers in results.values() for m in solvers.values())
        self.last_evaluation = {
            "iteration": self.counters["iteration"], "score": combined,
            "solvers": results.get(BASE_BATTERY) or next(iter(results.values())),
            "batteries": {battery: {"posture": battery_posture(battery), "solvers": solvers}
                          for battery, solvers in results.items()},
            "seconds": time.time() - started, "time": time.time()}
        if combined > self.best["score"] + 1e-6:
            self.best = {"score": combined, "iteration": self.counters["iteration"], "steps": self.counters["steps"]}
            self.bests[self.config.task] = dict(self.best)
            self.export("policy_best")
            self.save("melhor")
            self.db.event(self.run, "melhor", f"novo melhor: nota {combined:.3f}", data=results)
        elif self.best["score"] - combined > 0.25:
            self.db.event(self.run, "regressao", f"a nota caiu de {self.best['score']:.3f} para {combined:.3f}",
                          data=results)
        episodes = sorted((self.directory / "episodes").glob("aval-*.dump"))
        for old in episodes[:-40]:
            old.unlink(missing_ok=True)
        self.last_evaluation_time = time.time()
        self.state = previous

    # --- comandos ------------------------------------------------------------------
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
            self.evaluate()
            return {"ok": True, "evaluation": self.last_evaluation}
        if command == "modo":
            name = arguments["modo"]
            system.mode(name)
            if name != self.mode.name:
                self.config.mode = name
                self.config.save(self.directory / "run.json")
                self.db.event(self.run, "modo", f"modo {system.MODES[name].label}", author)
                self.restart_requested = True
            return {"ok": True}
        if command == "tarefa":
            # Outra habilidade no mesmo cerebro: o treino reabre com a tarefa dela.
            task = arguments["tarefa"]
            if not paths.task_path(task).exists():
                raise ValueError(f"tarefa desconhecida: {task}")
            if task != self.config.task:
                self.config.task = task
                self.config.save(self.directory / "run.json")
                self.restart_requested = True
            return {"ok": True}
        if command == "peso":
            data = json.loads(self.live_task.read_text())
            component, value = arguments["componente"], float(arguments["valor"])
            if component not in data["reward"] and component not in self.reward_names:
                raise ValueError(f"parcela desconhecida: {component}")
            old = data["reward"].get(component, "padrao")
            data["reward"][component] = value
            storage.write_json_atomic(self.live_task, data)
            self.batch.update_task(str(self.live_task))
            if self.evaluator is not None:
                for batch in self.evaluator.batches.values():
                    batch.update_task(str(self.live_task))
            self.db.event(self.run, "peso", f"{component}: {old} -> {value}", author,
                          {"motivo": arguments.get("motivo", "")})
            return {"ok": True}
        if command == "empurroes":
            data = json.loads(self.live_task.read_text())
            pushes = data.setdefault("pushes", {})
            for key in ("enabled", "intervalSeconds", "durationSeconds", "forceNewtons"):
                if key in arguments:
                    pushes[key] = arguments[key]
            storage.write_json_atomic(self.live_task, data)
            self.batch.update_task(str(self.live_task))
            self.db.event(self.run, "empurroes", json.dumps(pushes), author, {"motivo": arguments.get("motivo", "")})
            return {"ok": True}
        raise ValueError(f"comando desconhecido: {command}")

    # --- o laco ----------------------------------------------------------------------
    def tensor(self, array: np.ndarray) -> torch.Tensor:
        return torch.tensor(array, device=self.device, dtype=torch.float32)

    def iteration(self) -> dict:
        s = self.settings
        batch, model = self.batch, self.model
        envs = batch.num_envs
        rollout = Rollout(s.steps_per_env, envs, batch.obs_size, batch.critic_size, batch.action_size, self.device)
        components = np.zeros(batch.reward_count, dtype=np.float64)
        episodes: list[np.ndarray] = []
        begin = time.time()
        obs, critic = self.tensor(batch.observations()), self.tensor(batch.critic_observations())
        with torch.no_grad():
            for _ in range(s.steps_per_env):
                model.obs_norm.update(obs)
                model.critic_norm.update(critic)
                obs_n, critic_n = model.obs_norm(obs), model.critic_norm(critic)
                dist = model.distribution(obs_n)
                action = dist.sample()
                log_prob = dist.log_prob(action).sum(-1)
                value = model.value(critic_n)
                batch.step(np.ascontiguousarray(action.cpu().numpy(), dtype=np.float32))
                reward = self.tensor(batch.rewards())
                terminated = self.tensor(batch.terminated())
                truncated_np = np.asarray(batch.truncated())
                truncated = self.tensor(truncated_np)
                if truncated_np.any():
                    # Fim pelo tempo nao e fracasso: o valor do estado final entra.
                    index = np.nonzero(truncated_np)[0]
                    final = self.tensor(np.asarray(batch.terminal_critic_observations())[index])
                    reward[torch.as_tensor(index, device=self.device)] += s.gamma * model.value(model.critic_norm(final))
                rollout.add(obs_n, critic_n, action, log_prob, value, reward, torch.maximum(terminated, truncated),
                            dist.mean, dist.stddev)
                components += np.asarray(batch.reward_components()).sum(axis=0)
                finished = batch.finished_episodes()
                if len(finished):
                    episodes.append(np.array(finished))
                obs, critic = self.tensor(batch.observations()), self.tensor(batch.critic_observations())
            last_values = model.value(model.critic_norm(critic))
        rollout.compute_returns(last_values, s.gamma, s.lam)
        collect = time.time() - begin
        begin = time.time()
        stats = self.ppo.update(rollout)
        update = time.time() - begin

        samples = s.steps_per_env * envs
        self.counters["iteration"] += 1
        self.counters["steps"] += samples
        self.counters["train_seconds"] += collect + update
        rows = np.concatenate(episodes) if episodes else np.zeros((0, EPISODE_COLUMNS))
        self.counters["episodes"] += len(rows)
        reasons = rows[:, 1].astype(int) if len(rows) else np.zeros(0, dtype=int)
        names = self.native.END_REASONS
        ends = {names[r]: int((reasons == r).sum()) for r in set(reasons.tolist())}
        sps = samples / max(1e-6, collect + update)
        self.recent_sps = sps if self.recent_sps == 0.0 else 0.9 * self.recent_sps + 0.1 * sps
        result = {
            "run": self.run, "iteration": self.counters["iteration"], "time": time.time(),
            "steps": self.counters["steps"], "sps": sps,
            "reward": float(rollout.rewards.mean()),
            "episodes": int(len(rows)),
            "episode_return": float(rows[:, 6].mean()) if len(rows) else None,
            "episode_seconds": float(rows[:, 5].mean()) if len(rows) else None,
            "fall_fraction": float(np.isin(reasons, [1, 2, 3]).mean()) if len(rows) else None,
            "components": json.dumps({n: float(v / samples) for n, v in zip(self.reward_names, components)}),
            "ends": json.dumps(ends),
            "policy_loss": stats["policy_loss"], "value_loss": stats["value_loss"], "entropy": stats["entropy"],
            "kl": stats["kl"], "clip_fraction": stats["clip_fraction"], "learning_rate": stats["learning_rate"],
            "action_std": stats["action_std"], "grad_norm": stats["grad_norm"],
            "collect_seconds": collect, "update_seconds": update,
        }
        return result

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
        self.fix_commands()
        self.counters.update(state.get("counters", {}))
        self.db.rewind(self.run, self.counters["iteration"])
        self.ppo.learning_rate = max(self.settings.min_learning_rate, self.ppo.learning_rate * 0.5)
        for group in self.ppo.optimizer.param_groups:
            group["lr"] = self.ppo.learning_rate
        self.batch.reset(int(time.time()))
        self.db.event(self.run, "recuperacao", f"valores invalidos: voltou ao checkpoint {reference.iteration}")

    def check_temperature(self) -> None:
        temperature = system.cpu_temperature()
        if temperature is None or temperature < 90.0:
            self.hot_since = None
            return
        self.hot_since = self.hot_since or time.time()
        if time.time() - self.hot_since < 60.0:
            return
        index = system.ORDER.index(self.mode.name)
        if index + 1 < len(system.ORDER):
            lighter = system.ORDER[index + 1]
            self.config.mode = lighter
            self.config.save(self.directory / "run.json")
            self.db.event(self.run, "temperatura", f"CPU a {temperature:.0f} °C por 1 min: modo {lighter}")
            self.restart_requested = True
        self.hot_since = None

    def loop(self) -> int:
        self.last_save = time.time()
        # O relogio da avaliacao segue o da ultima (trocar de modo ou reabrir nao
        # adia a proxima); um treino sem nenhuma, ou numa habilidade nova, avalia logo.
        self.last_evaluation_time = 0.0 if self.switched_from else self.last_evaluation.get("time", 0.0)
        last_status = 0.0
        last_temperature = 0.0
        last_export = time.time()
        failures = 0
        code = 0
        try:
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
                self.state = "treinando"
                result = self.iteration()
                if not self.healthy(result):
                    failures += 1
                    self.recover()
                    if failures >= 3:
                        self.pause_requested = True
                        self.message = "a rede ficou invalida 3 vezes seguidas: pausado"
                        self.db.event(self.run, "erro", self.message)
                    continue
                failures = 0
                self.last_iteration_stats = {k: result[k] for k in (
                    "iteration", "sps", "reward", "episodes", "episode_return", "episode_seconds", "fall_fraction",
                    "kl", "entropy", "learning_rate", "action_std", "value_loss")}
                self.db.log_iteration(result)
                self.db.commit()
                now = time.time()
                if now - self.last_evaluation_time >= self.config.evaluation_minutes * 60.0:
                    self.evaluate()
                if now - self.last_save >= self.config.checkpoint_minutes * 60.0:
                    self.save()
                    last_export = now
                elif now - last_export >= self.config.export_minutes * 60.0:
                    self.export("policy_latest")
                    last_export = now
                if now - last_status >= 2.0:
                    self.write_status()
                    last_status = now
                if now - last_temperature >= 30.0:
                    self.check_temperature()
                    last_temperature = now
                if self.max_iterations and self.counters["iteration"] >= self.max_iterations:
                    break
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
                self.lock.release()
        return code
