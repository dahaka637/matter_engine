"""Os ajustes de um treino fora do laco (vault 04_locomocao/referencia/pesquisas/
08_plano_consenso.md): a normalizacao refeita sem mudar a funcao (Q3) e o
recentramento do ator (a media saturada de volta para dentro dos limites, a acao
aplicada igual). Cada um le o ultimo checkpoint do treino (parado), mede a prova de
equivalencia e grava um checkpoint novo (a rodada + 1) que o treinador retoma com o
otimizador novo. O treino tem de estar fechado (a trava do treino garante).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch

from . import paths, storage
from .locomotion_trainer import NETWORK, RAW_PREFIXES, LocomotionRunConfig, native
from .model import ActorCritic, recenter_actor, refit_normalizer

# As bandeiras 0/1 da entrada que passam a cruas (Q3; analise de 18/10: 35 canais com
# desvio < 0,02, o comando.sprint a 1e-4).
FLAG_PREFIXES = ("exec.estado.", "exec.marcha.", "comando.sprint", "ref.no_ar", "ref.valida", "ajuda.",
                 "comando.olhar.inclina")


class RunContext:
    def __init__(self, run: str, environments: int = 512, threads: int = 12, seed: int = 20261018):
        self.run = run
        self.directory = paths.run_dir(run)
        self.lock = storage.RunLock(self.directory)
        self.config = LocomotionRunConfig.load(self.directory / "run.json")
        self.reference = storage.latest_checkpoint(self.directory)
        if self.reference is None:
            raise RuntimeError(f"o treino {run} nao tem checkpoint")
        self.state = storage.load_checkpoint(self.reference, "cpu")
        self.module = native()
        self.batch = self.module.LocomotionBatch(str(self.directory / "tarefa.json"), str(paths.ASSETS),
                                                 environments=environments, threads=threads, seed=seed)
        if list(self.batch.observation_names()) != list(self.state["obs_names"]) \
                or list(self.batch.action_names()) != list(self.state["action_names"]):
            raise RuntimeError("o checkpoint e de outro contrato: retome o treino uma vez antes do ajuste")
        network = self.state.get("network", NETWORK)
        self.model = ActorCritic(len(self.state["obs_names"]), len(self.state["critic_names"]),
                                 len(self.state["action_names"]), actor_hidden=network["actor"],
                                 critic_hidden=network["critic"])
        self.model.load_state_dict(self.state["model"])
        self.fix_raw(self.config.raw_extra_prefixes)
        self.scale = torch.as_tensor(np.asarray(self.state["action_scale"], dtype=np.float32))
        lower, upper = self.batch.action_bounds()
        self.lower = torch.as_tensor(np.asarray(lower, dtype=np.float32))
        self.upper = torch.as_tensor(np.asarray(upper, dtype=np.float32))
        authority = float(self.state.get("counters", {}).get("authority", 1.0))
        self.batch.set_authority(authority, authority, authority, authority)
        self.batch.reset(seed)

    def fix_raw(self, extra) -> None:
        prefixes = RAW_PREFIXES + tuple(extra)
        for norm, names in ((self.model.obs_norm, self.state["obs_names"]),
                            (self.model.critic_norm, self.state["critic_names"])):
            norm.fix_channels(torch.tensor([name.startswith(prefixes) for name in names]))

    @torch.no_grad()
    def collect(self, steps: int) -> tuple[torch.Tensor, torch.Tensor]:
        """As entradas cruas (a da rede e a do critico) dos estados que a politica de agora visita
        (a acao media: os estados do treino sem o ruido)."""
        observations, critics = [], []
        for _ in range(steps):
            obs = torch.as_tensor(np.array(self.batch.observations(), dtype=np.float32))
            critic = torch.as_tensor(np.array(self.batch.critic_observations(), dtype=np.float32))
            observations.append(obs)
            critics.append(critic)
            mean = self.model.actor(self.model.obs_norm(obs))
            self.batch.step(np.ascontiguousarray((mean * self.scale).numpy(), dtype=np.float32))
        return torch.cat(observations), torch.cat(critics)

    def save(self, note: str, extra_state: dict | None = None) -> Path:
        state = dict(self.state)
        state["model"] = self.model.state_dict()
        state["fresh_optimizer"] = True
        counters = dict(state.get("counters", {}))
        counters["iteration"] = int(counters.get("iteration", self.reference.iteration)) + 1
        state["counters"] = counters
        if extra_state:
            state.update(extra_state)
        manifest = dict(self.reference.manifest)
        manifest["ajuste"] = note
        reference = storage.save_checkpoint(self.directory, counters["iteration"], state, manifest)
        return reference.path

    def close(self) -> None:
        self.lock.release()


def unclipped(norm, raw: torch.Tensor) -> torch.Tensor:
    """As linhas que a normalizacao nao corta em canal nenhum."""
    scaled = (raw - norm.mean) * norm.inverse_std()
    return (scaled.abs() < norm.clip).all(dim=-1)


def renormalize(run: str, steps: int = 100, count: float = 5.0e6) -> dict:
    """A normalizacao refeita (Q3): estatistica nova dos estados de agora, as bandeiras cruas,
    a memoria limitada, a primeira camada do ator e do critico absorvendo a troca."""
    context = RunContext(run)
    try:
        observations, critics = context.collect(steps)
        model = context.model
        with torch.no_grad():
            before_mean = model.actor(model.obs_norm(observations))
            before_value = model.value(model.critic_norm(critics))
            old_rows = unclipped(model.obs_norm, observations)
            old_critic_rows = unclipped(model.critic_norm, critics)
            old_cut = float((((observations - model.obs_norm.mean) * model.obs_norm.inverse_std()).abs()
                             >= model.obs_norm.clip).float().mean())
            obs_raw = torch.tensor([n.startswith(RAW_PREFIXES + FLAG_PREFIXES) for n in context.state["obs_names"]])
            critic_raw = torch.tensor([n.startswith(RAW_PREFIXES + FLAG_PREFIXES)
                                       for n in context.state["critic_names"]])
            limit = float(count) if count > 0 else float(observations.shape[0])
            refit_normalizer(model.obs_norm, model.actor[0], observations, obs_raw, count=limit)
            refit_normalizer(model.critic_norm, model.critic[0], critics, critic_raw, count=limit)
            context.fix_raw(FLAG_PREFIXES)
            after_mean = model.actor(model.obs_norm(observations))
            after_value = model.value(model.critic_norm(critics))
            rows = old_rows & unclipped(model.obs_norm, observations)
            critic_rows = old_critic_rows & unclipped(model.critic_norm, critics)
            new_cut = float((((observations - model.obs_norm.mean) * model.obs_norm.inverse_std()).abs()
                             >= model.obs_norm.clip).float().mean())
        report = {
            "samples": int(observations.shape[0]),
            "rows_unclipped_both": float(rows.float().mean()),
            "actor_max_change_unclipped": float((after_mean[rows] - before_mean[rows]).abs().max()) if rows.any() else None,
            "actor_max_change_all": float((after_mean - before_mean).abs().max()),
            "critic_max_change_unclipped": (float((after_value[critic_rows] - before_value[critic_rows]).abs().max())
                                            if critic_rows.any() else None),
            "inputs_cut_before": old_cut, "inputs_cut_after": new_cut,
            "raw_channels": int(obs_raw.sum()),
        }
        report["equivalent"] = report["actor_max_change_unclipped"] is not None \
            and report["actor_max_change_unclipped"] < 1e-4
        if report["equivalent"]:
            context.config.raw_extra_prefixes = list(FLAG_PREFIXES)
            context.config.norm_max_count = float(count)
            context.config.save(context.directory / "run.json")
            report["checkpoint"] = str(context.save("normalizacao refeita (Q3)"))
        return report
    finally:
        context.close()


def recenter(run: str, steps: int = 100, iterations: int = 4000, tolerance: float = 0.05) -> dict:
    """O recentramento (rodada 3): a media fora dos limites volta para 0,05 para dentro deles, a
    acao aplicada quase igual (a tolerancia: o RMS da mudanca na unidade do teto; a mudanca
    maxima esperada e a margem, 5% do teto, nos canais saturados)."""
    context = RunContext(run)
    try:
        observations, _ = context.collect(steps)
        model = context.model
        with torch.no_grad():
            normalized = model.obs_norm(observations)
        generator = torch.Generator().manual_seed(20261018)
        report = recenter_actor(model, normalized, context.lower, context.upper, iterations=iterations,
                                generator=generator)
        report["samples"] = int(observations.shape[0])
        report["equivalent"] = report["applied_rms_change"] < tolerance
        if report["equivalent"]:
            report["checkpoint"] = str(context.save("recentramento do ator"))
        return report
    finally:
        context.close()


def main(argv: list[str] | None = None) -> int:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("acao", choices=["renormalizar", "recentrar"])
    parser.add_argument("--run", required=True)
    parser.add_argument("--passos", type=int, default=100)
    args = parser.parse_args(argv)
    report = renormalize(args.run, args.passos) if args.acao == "renormalizar" else recenter(args.run, args.passos)
    print(json.dumps(report, indent=1, ensure_ascii=False))
    return 0 if report.get("equivalent") else 2


if __name__ == "__main__":
    raise SystemExit(main())
