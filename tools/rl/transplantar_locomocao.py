"""A cabeca da locomocao de um checkpoint antigo (o ator, o critico, o desvio dela e a normalizacao dela) posta num checkpoint de agora,
que segue com a cabeca do levantar dele (12/11 16h, vault 04_locomocao/plano_fogo_da_locomocao_12_11.md).

Por que: de 11/11 14h48 a 12/11 a locomocao ficou "congelada" no idle_alerta, mas o ReDo seguiu rodando nela e julgava as unidades
mortas so pelos 1,5 s de locomocao depois do levantar: reiniciou unidades e zerou a saida de 48 de 512 da segunda camada do ator
(~58% da norma da ultima camada) e de 172 de 512 do critico. A capacidade a 0% caiu de 0,78/0,72 (trote/corrida) para 0,52/0,38.

O que faz, nesta ordem:
- as entradas que a fonte nao tem (as que entraram antes da habilidade depois dela: a variante do levantar e as 6 do fogo) entram
  como colunas em zero na primeira camada do ator e do critico, e neutras na normalizacao (levantar_amp.insert_inputs);
- as faixas da acao migradas da fonte para o alvo como o migrar_faixa_acao (W' = W h0/h1, b' = (b h0 + m0 - m1)/h1 e o desvio
  log += log(h0/h1)), para o alvo absoluto sair o mesmo;
- confere: em entradas sorteadas, o alvo absoluto da cabeca e o valor do critico iguais aos da fonte (ate 1e-4); senao nada grava.
O otimizador: o Adam da locomocao do alvo nao tem estado (o zerar_recuperacao apagou os dois, e congelada ela nao ganhou outro);
se tiver, ele e apagado (o Adam recomeca nela).

Uso (com MATTER_TRACKING_NATIVE de um build): python transplantar_locomocao.py FONTE.pt ALVO.pt SAIDA.pt
"""
import math
import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).parent))
import levantar_amp as L  # noqa: E402

HEAD_KEYS = ("actor.locomotion.", "critic.locomotion.", "norm.locomotion.")


def main() -> int:
    source_path, target_path, out_path = (Path(a).expanduser() for a in sys.argv[1:4])
    source = torch.load(source_path, map_location="cpu", weights_only=False)
    target = torch.load(target_path, map_location="cpu", weights_only=False)
    s, t = source["model"], target["model"]
    n_source, n_target = s["actor.locomotion.0.weight"].shape[1], t["actor.locomotion.0.weight"].shape[1]
    if n_target < n_source:
        raise SystemExit(f"o alvo tem menos entradas ({n_target}) que a fonte ({n_source})")
    head = {k: v.clone() for k, v in s.items() if k.startswith(HEAD_KEYS) or k == "loco_log_std"}
    if n_target > n_source:
        head = L.insert_inputs(head, n_source - 1, n_target - n_source)
    m0, h0 = s["action_mid"].double(), s["action_half"].double()
    m1, h1 = t["action_mid"].double(), t["action_half"].double()
    changed = [k for k in range(len(m0)) if abs(float(m1[k] - m0[k])) > 1e-6 or abs(float(h1[k] - h0[k])) > 1e-6]
    w, b = head["actor.locomotion.4.weight"], head["actor.locomotion.4.bias"]
    for k in changed:
        w[k] = (w[k].double() * h0[k] / h1[k]).to(w.dtype)
        b[k] = ((b[k].double() * h0[k] + m0[k] - m1[k]) / h1[k]).to(b.dtype)
        head["loco_log_std"][k] = head["loco_log_std"][k] + math.log(float(h0[k] / h1[k]))
    # (A conferencia: entradas sorteadas na escala da normalizacao; a da fonte sem as colunas novas.)
    torch.manual_seed(0)
    z = torch.randn(1024, n_target, dtype=torch.float64)
    keep = list(range(n_source - 1)) + [n_target - 1]
    z[:, n_source - 1:n_target - 1] = 0.0

    def mlp(state, prefix, x):
        x = torch.relu(x @ state[prefix + "0.weight"].double().T + state[prefix + "0.bias"].double())
        x = torch.relu(x @ state[prefix + "2.weight"].double().T + state[prefix + "2.bias"].double())
        return x @ state[prefix + "4.weight"].double().T + state[prefix + "4.bias"].double()

    before = m0 + h0 * mlp(s, "actor.locomotion.", z[:, keep])
    after = m1 + h1 * mlp(head, "actor.locomotion.", z)
    worst_actor = float((after - before).abs().max())
    worst_critic = float((mlp(head, "critic.locomotion.", z) - mlp(s, "critic.locomotion.", z[:, keep])).abs().max())
    std_before = (s["loco_log_std"].double().exp() * h0)
    std_after = (head["loco_log_std"].double().exp() * h1)
    worst_std = float((std_after - std_before).abs().max())
    if max(worst_actor, worst_critic, worst_std) > 1e-4:
        raise SystemExit(f"a cabeca transplantada difere da fonte (ator {worst_actor:.2e}, critico {worst_critic:.2e}, "
                         f"desvio {worst_std:.2e}) - nada gravado")
    for k, v in head.items():
        if t[k].shape != v.shape:
            raise SystemExit(f"{k}: forma {tuple(v.shape)} no transplante contra {tuple(t[k].shape)} no alvo")
        t[k] = v.to(t[k].dtype)
    # (O Adam: o estado das camadas da locomocao, se houver, e apagado - o grupo da locomocao e o segundo do ator; no critico,
    # os parametros da locomocao sao os do critic.locomotion. Os indices seguem a ordem do model.parameters do treino.)
    cleared = 0
    for key in ("actor_opt", "critic_opt"):
        opt = target.get(key)
        if not opt:
            continue
        if key == "actor_opt" and len(opt["param_groups"]) > 1:
            for i in opt["param_groups"][1]["params"]:
                cleared += opt["state"].pop(i, None) is not None
    counters = target.setdefault("counters", {})
    counters["transplante_locomocao"] = {"fonte": str(source_path), "atualizacao_fonte": source["counters"].get("atualizacao"),
                                         "entradas_novas": n_target - n_source, "faixas_migradas": len(changed),
                                         "adam_apagado": cleared}
    torch.save(target, out_path)
    print(f"cabeca da locomocao de {source_path.name} (r{source['counters'].get('atualizacao')}) em {out_path}: {n_target - n_source} "
          f"entradas novas em zero, {len(changed)} faixas migradas, alvo absoluto igual ate {worst_actor:.1e} rad, critico ate "
          f"{worst_critic:.1e}, desvio ate {worst_std:.1e}; Adam da locomocao apagado em {cleared} parametros")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
