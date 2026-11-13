"""As faixas da acao num checkpoint de duas cabecas, depois de uma mudanca nos limites das juntas do perfil (11/11 17h15: o pulso
anatomico - Hand.swing1 de +-35 para -70..+80 graus; vault 04_locomocao/plano_levantar_unreal_11_11.md).

A acao da rede e normalizada: o alvo e action_mid + action_half * saida. O checkpoint guarda o meio e a metade antigos, e a carga
do treino os mantem; com a faixa nova do perfil, cada eixo que mudou tem a ultima camada de cada cabeca reescrita para o alvo
absoluto sair o MESMO - W' = W * h0 / h1, b' = (b * h0 + m0 - m1) / h1 - e o desvio na mesma escala (log_std += log(h0 / h1)).
As duas cabecas fazem exatamente o mesmo no primeiro instante; a do levantar aprende a usar a faixa nova.

Uso (com MATTER_TRACKING_NATIVE do build): python migrar_faixa_acao.py <ultimo.pt> <saida.pt> <pasta do treino (run.json)>
"""
import math
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).parent))
import levantar_amp as L  # noqa: E402


def main() -> int:
    source, target = Path(sys.argv[1]).expanduser(), Path(sys.argv[2]).expanduser()
    state = torch.load(source, map_location="cpu", weights_only=False)
    model = state["model"]
    # (O lote no layout do treino - as acoes de velocidade e os canais do passo pelo v2 do run.json, como inspecao.comum.)
    import json
    v2 = json.loads((Path(sys.argv[3]).expanduser() / "run.json").read_text())["v2"]
    L.VELOCITY_ACTIONS[0] = bool(v2.get("acao_velocidade", False))
    L.BATCH_EXTRA.update(L.batch_layout(v2))
    b = L.get_up_batch(str(L.REPO / "assets"), environments=1, threads=1, seed=1, evaluation=True)
    names = list(b.action_names())
    ranges = np.asarray(b.action_ranges(), dtype=np.float64)
    if model["action_mid"].shape[0] != len(names):
        raise SystemExit(f"o checkpoint tem {model['action_mid'].shape[0]} acoes e o lote {len(names)}")
    low, high = torch.as_tensor(ranges[:, 0]), torch.as_tensor(ranges[:, 1])
    m1 = (0.5 * (low + high)).to(model["action_mid"].dtype)
    h1 = torch.clamp(0.5 * (high - low), min=0.05).to(model["action_half"].dtype)
    m0, h0 = model["action_mid"].clone(), model["action_half"].clone()
    changed = [k for k in range(len(names)) if abs(float(m1[k] - m0[k])) > 1e-6 or abs(float(h1[k] - h0[k])) > 1e-6]
    if not changed:
        print("nenhuma faixa mudou")
        return 0
    probe = torch.randn(512, model["actor.recovery.4.weight"].shape[1])
    before = {}
    for head in ("recovery", "locomotion"):
        w, bias = model[f"actor.{head}.4.weight"], model[f"actor.{head}.4.bias"]
        before[head] = m0 + h0 * (probe @ w.T + bias)
    for head in ("recovery", "locomotion"):
        w, bias = model[f"actor.{head}.4.weight"], model[f"actor.{head}.4.bias"]
        for k in changed:
            w[k] = w[k] * h0[k] / h1[k]
            bias[k] = (bias[k] * h0[k] + m0[k] - m1[k]) / h1[k]
    for key in ("log_std", "loco_log_std"):
        if key in model:
            for k in changed:
                model[key][k] = model[key][k] + math.log(float(h0[k] / h1[k]))
    model["action_mid"], model["action_half"] = m1, h1
    worst = 0.0
    for head in ("recovery", "locomotion"):
        w, bias = model[f"actor.{head}.4.weight"], model[f"actor.{head}.4.bias"]
        after = m1 + h1 * (probe @ w.T + bias)
        worst = max(worst, float((after - before[head]).abs().max()))
    if worst > 1e-4:
        raise SystemExit(f"o alvo absoluto mudou {worst:.2e} rad - nada gravado")
    state.setdefault("counters", {})["faixa_acao_migrada"] = {names[k]: [round(math.degrees(float(m0[k] - h0[k])), 1),
                                                                         round(math.degrees(float(m0[k] + h0[k])), 1),
                                                                         round(math.degrees(float(m1[k] - h1[k])), 1),
                                                                         round(math.degrees(float(m1[k] + h1[k])), 1)]
                                                              for k in changed}
    torch.save(state, target)
    print(f"faixas migradas: {', '.join(f'{names[k]} {math.degrees(float(m0[k] - h0[k])):+.0f}..{math.degrees(float(m0[k] + h0[k])):+.0f} -> '
                                         f'{math.degrees(float(m1[k] - h1[k])):+.0f}..{math.degrees(float(m1[k] + h1[k])):+.0f} graus' for k in changed)}; "
          f"o alvo absoluto das duas cabecas igual ate {worst:.1e} rad; salvo em {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
