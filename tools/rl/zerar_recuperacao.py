"""A cabeca do levantar (a recuperacao) do zero num checkpoint de duas cabecas (11/11 14h, ele: "Zerar absolutamente o brain do
levantar-se, esta poluido demais").

Recomeca como uma rede nova (o mlp do levantar_amp.py - 1024 e 512, ReLU -, a inicializacao do PyTorch, a mesma da Policy): o ator, o critico, a
normalizacao das entradas (media 0, variancia 1, contagem 1e-4) e o desvio (log 0,1) da recuperacao. Saem os Adam do ator e do
critico (o treino os recomeca), a ancora de preservacao (puxaria a cabeca nova de volta ao levantar antigo) e marca
counters["aquecer_desde"] = a rodada de agora (o aquecimento do critico conta dali). A locomocao, o juiz dela, os contadores e o
resto ficam como vieram; o script confere que a locomocao saiu identica.

Uso: python zerar_recuperacao.py <ultimo.pt de origem> <saida.pt> [semente]
"""
import math
import sys
from pathlib import Path

import torch

from torch import nn


def mlp(inputs: int, outputs: int) -> nn.Sequential:
    """A mesma de levantar_amp.mlp (importar o treinador puxaria o modulo nativo)."""
    return nn.Sequential(nn.Linear(inputs, 1024), nn.ReLU(), nn.Linear(1024, 512), nn.ReLU(), nn.Linear(512, outputs))


def main() -> int:
    source, target = Path(sys.argv[1]).expanduser(), Path(sys.argv[2]).expanduser()
    torch.manual_seed(int(sys.argv[3]) if len(sys.argv) > 3 else 1111)
    state = torch.load(source, map_location="cpu", weights_only=False)
    model = state["model"]
    if "actor.recovery.0.weight" not in model:
        raise SystemExit("o checkpoint nao tem as duas cabecas (actor.recovery.*)")
    obs = model["actor.recovery.0.weight"].shape[1]
    actions = model["actor.recovery.4.weight"].shape[0]
    before = {k: v.clone() for k, v in model.items() if ".locomotion." in k or k == "loco_log_std"}
    actor, critic = mlp(obs, actions), mlp(obs, 1)
    for prefix, net in (("actor.recovery.", actor), ("critic.recovery.", critic)):
        fresh = net.state_dict()
        for name, value in fresh.items():
            key = prefix + name
            if key not in model or model[key].shape != value.shape:
                raise SystemExit(f"{key}: forma diferente no checkpoint")
            model[key] = value.to(model[key].dtype)
    model["norm.recovery.mean"] = torch.zeros_like(model["norm.recovery.mean"])
    model["norm.recovery.var"] = torch.ones_like(model["norm.recovery.var"])
    model["norm.recovery.count"] = torch.tensor(1e-4, dtype=model["norm.recovery.count"].dtype)
    model["log_std"] = torch.full_like(model["log_std"], math.log(0.1))
    for key in ("actor_opt", "critic_opt", "preservation_anchor"):
        state.pop(key, None)
    counters = state["counters"]
    counters["aquecer_desde"] = counters["atualizacao"]
    counters["levantar_zerado"] = {"rodada": counters["atualizacao"], "de": str(source)}
    for key, value in before.items():
        if not torch.equal(value, model[key]):
            raise SystemExit(f"a locomocao mudou: {key}")
    target.parent.mkdir(parents=True, exist_ok=True)
    torch.save(state, target)
    print(f"cabeca do levantar do zero ({obs} entradas, {actions} acoes) na rodada {counters['atualizacao']}; "
          f"locomocao identica ({len(before)} tensores); salvo em {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
