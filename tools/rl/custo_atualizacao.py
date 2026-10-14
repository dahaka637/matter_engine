"""O Experimento A da reformulacao do RL (vault: Aprendizado_por_Reforco/
novo_plano/plano_claude_rl.md, secao 14.1): quanto custa, na GPU, uma rodada
do PPO com as redes de hoje e com as do plano (o ator e o critico maiores, o
juiz de estilo e o espelho), contra o tempo da coleta da fisica.

Usa o PPO de verdade do treinador (matter_rl.ppo) com dados sorteados; o juiz
e um MLP 1024-512 treinado como no AMP (minimos quadrados e penalidade de
gradiente). Imprime Markdown.

    cd tools/rl && .venv/bin/python custo_atualizacao.py
"""
from __future__ import annotations

import statistics
import time

import torch
from torch import nn

from matter_rl.model import ActorCritic, mlp
from matter_rl.ppo import PPO, PPOSettings, Rollout

ENVS = 512
STEPS = 24
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def make_model(obs: int, critic: int, actions: int, hidden: tuple[int, ...]) -> ActorCritic:
    model = ActorCritic(obs, critic, actions)
    model.actor = mlp(obs, actions, hidden=hidden, last_gain=0.01)
    model.critic = mlp(critic, 1, hidden=hidden, last_gain=1.0)
    return model.to(DEVICE)


def make_rollout(model: ActorCritic, steps: int) -> Rollout:
    rollout = Rollout(steps, ENVS, model.obs_size, model.critic_size, model.action_size, DEVICE)
    for _ in range(steps):
        obs = torch.randn(ENVS, model.obs_size, device=DEVICE)
        critic = torch.randn(ENVS, model.critic_size, device=DEVICE)
        actions, log_probs, values = model.act(obs, critic)
        dist = model.distribution(obs)
        rollout.add(obs, critic, actions, log_probs, values, torch.randn(ENVS, device=DEVICE),
                    torch.zeros(ENVS, device=DEVICE), dist.mean.detach(), dist.stddev.detach())
    rollout.compute_returns(torch.zeros(ENVS, device=DEVICE), 0.99, 0.95)
    return rollout


class Judge(nn.Module):
    def __init__(self, features: int):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(features, 1024), nn.ReLU(), nn.Linear(1024, 512), nn.ReLU(),
                                 nn.Linear(512, 1))

    def forward(self, x):
        return self.net(x)


def judge_update(judge: Judge, optimizer, features: int, samples: int, epochs: int = 5, minibatches: int = 4) -> None:
    size = samples // minibatches
    for _ in range(epochs):
        for _ in range(minibatches):
            policy = torch.randn(size, features, device=DEVICE)
            dataset = torch.randn(size, features, device=DEVICE, requires_grad=True)
            real, fake = judge(dataset), judge(policy)
            loss = ((real - 1.0) ** 2).mean() + ((fake + 1.0) ** 2).mean()
            grad = torch.autograd.grad(real.sum(), dataset, create_graph=True)[0]
            loss = loss + 5.0 * grad.pow(2).sum(-1).mean()
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()


def timed(function, repeats: int = 4) -> float:
    function()  # aquecimento
    if DEVICE.type == "cuda":
        torch.cuda.synchronize()
    times = []
    for _ in range(repeats):
        start = time.perf_counter()
        function()
        if DEVICE.type == "cuda":
            torch.cuda.synchronize()
        times.append(time.perf_counter() - start)
    return statistics.median(times)


def inference(model: ActorCritic) -> float:
    """A conta da politica numa coleta (24 decisoes de 512 mundos), com a ida e
    a volta da CPU como no treinador."""
    host = torch.randn(ENVS, model.obs_size).pin_memory() if DEVICE.type == "cuda" else torch.randn(ENVS, model.obs_size)
    critic = torch.randn(ENVS, model.critic_size)

    def run():
        for _ in range(STEPS):
            obs = host.to(DEVICE, non_blocking=True)
            actions, _, _ = model.act(obs, critic.to(DEVICE, non_blocking=True))
            actions.cpu()
    return timed(run)


def main() -> None:
    print(f"# Experimento A: o custo de uma rodada do PPO ({DEVICE}, "
          f"{torch.cuda.get_device_name(0) if DEVICE.type == 'cuda' else 'cpu'})\n")
    print(f"Uma rodada: {ENVS} mundos x {STEPS} decisoes = {ENVS * STEPS} decisoes; 5 epocas x 4 lotes.\n")
    print("| Arranjo | Atualizacao (s) | Politica na coleta (s) | Total da GPU (s) |")
    print("|---|---:|---:|---:|")
    settings = PPOSettings()
    # O espelho: "lote" dobra o lote do PPO (o critico tambem); "ator" e uma
    # perda a mais so no ator (a acao no estado espelhado contra a acao
    # espelhada), uma passada do ator por lote. O juiz: "lote" treina com o
    # lote inteiro (2 x 12.288 por epoca); "AMP" com 2.048 + 2.048 por lote,
    # na ordem do que o AMP usa.
    cases = [
        ("Hoje: 261 entradas, 39 eixos, 512-256-128", 261, 286, 39, (512, 256, 128), None, None),
        ("Plano: 528 entradas, 37 eixos, 1024-512-256", 528, 570, 37, (1024, 512, 256), None, None),
        ("Plano + espelho (lote dobrado)", 528, 570, 37, (1024, 512, 256), "lote", None),
        ("Plano + espelho (perda so no ator)", 528, 570, 37, (1024, 512, 256), "ator", None),
        ("Plano + juiz (lote inteiro)", 528, 570, 37, (1024, 512, 256), None, "lote"),
        ("Plano + juiz (lote do AMP, 2.048)", 528, 570, 37, (1024, 512, 256), None, "amp"),
        ("Plano + espelho no ator + juiz do AMP", 528, 570, 37, (1024, 512, 256), "ator", "amp"),
    ]
    for name, obs, critic, actions, hidden, mirror, judge_mode in cases:
        model = make_model(obs, critic, actions, hidden)
        ppo = PPO(model, settings)
        rollout = make_rollout(model, STEPS * (2 if mirror == "lote" else 1))
        judge = Judge(198).to(DEVICE) if judge_mode else None
        judge_optimizer = torch.optim.Adam(judge.parameters(), lr=1e-4) if judge else None
        minibatch = ENVS * STEPS // settings.minibatches
        mirrored = torch.randn(minibatch, obs, device=DEVICE)
        wanted = torch.randn(minibatch, actions, device=DEVICE)
        auxiliary = (lambda: ((model.actor(mirrored) - wanted) ** 2).mean()) if mirror == "ator" else None

        def update():
            ppo.update(rollout, auxiliary=auxiliary)
            if judge is not None:
                samples = ENVS * STEPS * 2 if judge_mode == "lote" else 2048 * 4 * 2
                judge_update(judge, judge_optimizer, 198, samples)
        update_seconds = timed(update, repeats=3)
        inference_seconds = inference(model)
        print(f"| {name} | {update_seconds:.3f} | {inference_seconds:.3f} | {update_seconds + inference_seconds:.3f} |")
    print("\nA coleta da fisica sozinha (12/10, solver 10/6, 10 threads): ~22.500 decisoes/s, ou ~0,55 s por rodada.")


if __name__ == "__main__":
    main()
