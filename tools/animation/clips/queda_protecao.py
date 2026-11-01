"""Quedas com protecao e tropecos recuperados (01/11) para o estilo (AMP) do levantar: ao cair, maos, joelhos,
quadril e coluna amortecem; no tropeco e no empurrao, passos rapidos recuperam a postura.

As fontes sao gerações do Kimodo (Kimodo-SOMA-RP-v1.1, escolhidas por tools/animation/kimodo/queda.py) e dois
tropecos do Mixamo, retargeteadas pelo import_humanoid_animation.py com --root-motion preserve (a queda e os passos
andam: no lugar, os pes escorregariam) e --reference-only. A limpeza e a do levantar do Kimodo (stand_up_kimodo.py:
raiz inteira sem o rumo do primeiro quadro, tornozelo contra o chao, cada quadro apoiado no chao, maos sem
atravessar, canais de apoio); as quedas terminam no chao (sem a mistura final para a pose de pe).

Uso (depois, os espelhados: tools/animation/espelhar.py <id> <id>_espelho --name "...")
  python3 tools/animation/clips/queda_protecao.py todos | frente tras lado tropeco empurrao_lado ...
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import stand_up_kimodo as s  # noqa: E402

KIMODO = "Kimodo-SOMA-RP-v1.1 (texto + poses-chave), BVH retargeteado"
MIXAMO = "Mixamo (Adobe), FBX retargeteado"

VARIANTS = {
    # lote1/frente_a_k_02: "A person standing upright loses balance and falls forward, reaching both arms forward to
    # break the fall, landing on the hands and then on the knees, and slowly lowers the chest to the ground." (de pe
    # no quadro 0, de brucos no ultimo)
    "frente": dict(source="queda_protecao_frente.retarget.json", id="queda_protecao_frente",
                   name="Queda para a frente amortecida nos joelhos, maos e antebracos (Kimodo)", start=0.3, stop=2.6,
                   end_blend=False),
    # lote1/tras_a_0_01: "A person standing upright loses balance backward, bends the knees and sits down on the
    # ground with both hands behind to break the fall, then rolls back onto their back with the chin tucked to the
    # chest." (de pe no quadro 0)
    "tras": dict(source="queda_protecao_tras.retarget.json", id="queda_protecao_tras",
                 name="Queda para tras: senta, maos atras e rola nas costas (Kimodo)", start=0.3, stop=2.8,
                 end_blend=False),
    # lote2/lado_b_kb_04: "A person bends down on their legs and falls down on the floor to their left side." (de pe
    # no quadro 0, de costas no ultimo com a cabeca para a esquerda: cai de lado e rola para as costas)
    "lado": dict(source="queda_protecao_lado.retarget.json", id="queda_protecao_lado",
                 name="Queda de lado no joelho, antebraco e quadril, com rolamento (Kimodo)", start=0.4, stop=2.9,
                 end_blend=False),
    # lote2/tropeco_a_07: "A person walking forward trips on an object, stumbles forward and takes two quick steps to
    # regain balance, then stands upright." (so o texto)
    "tropeco": dict(source="queda_protecao_tropeco.retarget.json", id="queda_protecao_tropeco",
                    name="Tropeca andando e recupera com passos rapidos (Kimodo)", start=0.3, end_blend=False),
    # lote3/emp_lado_b_0_00: "A person standing upright stumbles sideways to the right, losing their balance
    # momentarily and then regaining balance and standing upright." (de pe no quadro 0; cruza a perna esquerda)
    "empurrao_lado": dict(source="queda_protecao_empurrao_lado.retarget.json", id="queda_protecao_empurrao_lado",
                          name="Desequilibra de lado e recupera com passo cruzado (Kimodo)", start=0.3,
                          end_blend=False),
    # lote3/emp_tras_b_0_01: "A person standing upright loses balance backward, their legs and arms move outward and
    # they step backward quickly to regain balance, then stand upright." (de pe no quadro 0)
    "empurrao_tras": dict(source="queda_protecao_empurrao_tras.retarget.json", id="queda_protecao_empurrao_tras",
                          name="Desequilibra para tras e recupera com passos para tras (Kimodo)", start=0.6,
                          end_blend=False),
    "tropeco_corrida": dict(source="queda_protecao_tropeco_corrida.retarget.json", id="queda_protecao_tropeco_corrida",
                            name="Tropeco correndo e recupera (Mixamo Jogging Stumble)", start=0.0, end_blend=False,
                            source_note=MIXAMO + " (tropecos_e_reacoes/Jogging Stumble.fbx)"),
    "tropeco_queda": dict(source="queda_protecao_tropeco_queda.retarget.json", id="queda_protecao_tropeco_queda",
                          name="Tropeca e cai para a frente (Mixamo Tripping)", start=0.0, end_blend=False,
                          source_note=MIXAMO + " (tropecos_e_reacoes/Tripping.fbx)"),
}
for v in VARIANTS.values():
    v.setdefault("full_root", True)
    v.setdefault("source_note", KIMODO)
    v.setdefault("hand_snap", 0.08)  # a mao que apoiava no Kimodo encosta no chao (stand_up_kimodo.py)
    v.setdefault("head_fix", v["source_note"] == KIMODO)  # o pescoco do SOMA volta ao neutro do jogo


GROUPS = {  # (o que toca, links do corpo do jogo; o ponto mais baixo do colisor)
    "maoE": ["LeftHand"], "maoD": ["RightHand"], "antebracoE": ["LeftForearm"], "antebracoD": ["RightForearm"],
    "joelhoE": ["LeftShin"], "joelhoD": ["RightShin"], "quadril": ["Pelvis"],
    "tronco": ["Abdomen", "Chest", "UpperChest"], "cabeca": ["Neck", "Head"],
}
TOUCH = 0.03  # m acima do chao: encostou


def measure(clip_id: str) -> str:
    """No corpo do jogo (colisores do perfil): comeca de pe, o primeiro toque de cada parte no chao, o que chega
    antes do tronco, a velocidade do tronco descendo ao tocar e como termina."""
    import numpy as np
    from matter_clip import load_clip
    from validate import lowest_point
    rig = s.Rig(s.g.PROFILE)
    clip = load_clip(s.CLIPS / f"{clip_id}.matteranim.json", rig)
    floor = float(rig.links[0].model_position[2]) - rig.standing_root_height
    low = {k: [] for k in GROUPS}
    pelvis, upright = [], []
    for f in range(len(clip.times)):
        p, o = rig.forward_kinematics(clip.root_offsets[f], clip.root_rotations[f], clip.coordinates[f])
        for k, ids in GROUPS.items():
            low[k].append(lowest_point(rig, p, o, ids) - floor)
        pelvis.append(p[0][2] - floor)
        up = p[rig.index("Neck")] - p[0]
        upright.append(up[2] / (np.linalg.norm(up) + 1e-9))
    rate = clip.sample_rate
    low = {k: np.array(v) for k, v in low.items()}
    first = {k: (int(np.flatnonzero(v < TOUCH)[0]) if (v < TOUCH).any() else None) for k, v in low.items()}
    stand0 = pelvis[0] > 0.8 and upright[0] > 0.85
    end = ("de pe" if pelvis[-1] > 0.8 and upright[-1] > 0.85 else "no chao" if pelvis[-1] < 0.35
           else f"pelve a {pelvis[-1]:.2f} m")
    t = first["tronco"]
    speed = (low["tronco"][t - 3] - low["tronco"][t]) / 3 * rate if t is not None and t >= 3 else None
    before = sorted((k_, n) for n, k_ in first.items() if k_ is not None and n != "tronco" and (t is None or k_ < t))
    touched = " ".join(f"{n}@{k_ / rate:.2f}" for k_, n in before) or "nada"
    # o que chega perto (3-8 cm) sem encostar antes do tronco (o joelho apoiado fica a ~4-5 cm pelo colisor)
    span = slice(0, t if t is not None else len(clip.times))
    near = " ".join(f"{n} {low[n][span].min() * 100:.0f} cm@{int(np.argmin(low[n][span])) / rate:.2f}"
                    for n in GROUPS if n != "tronco" and (first[n] is None or (t is not None and first[n] >= t))
                    and low[n][span].min() < 0.08)
    trunk = ("tronco nao toca" if t is None else f"tronco toca em {t / rate:.2f} s"
             + ("" if speed is None else f" (descendo a {speed:.2f} m/s)"))
    return (f"{clip_id}: {clip.duration:.2f} s | comeca de pe: {'sim' if stand0 else 'nao'} | {trunk} | "
            f"antes do tronco: {touched}" + (f" | perto sem encostar: {near}" if near else "") + f" | termina {end}")


def main() -> None:
    if sys.argv[1:2] == ["--medir"]:
        for clip_id in sys.argv[2:]:
            print(measure(clip_id))
        return
    names = sys.argv[1:] or ["todos"]
    if names == ["todos"]:
        names = list(VARIANTS)
    s.VARIANTS.update(VARIANTS)
    for name in names:
        sys.argv = [sys.argv[0], name]
        s.main()


if __name__ == "__main__":
    main()
