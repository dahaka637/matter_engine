# Movimentos gerados no Kimodo (NVIDIA)

Modelo `Kimodo-SOMA-RP-v1.1`, gerados com `tools/animation/kimodo/gera.sh`
(BVH no esqueleto SOMA, T-pose padrão) e retargeteados com
`tools/import_humanoid_animation.py --source-rig soma --reference-only`.

| Arquivo | Texto | Duração | Poses-chave | Semente / variação |
|---|---|---|---|---|
| `levantar_frente.bvh` | A person lying on their stomach on the ground quickly gets up and stands facing forward. | 3,0 s | de bruços no quadro 0, em pé no 89 (`levantar_frente.constraints.json`) | 4 / 08 |
| `levantar_costas.bvh` | A person lying flat on their back swiftly rises to a standing position, using their right hand for support. | 3,2 s | de costas no quadro 0, em pé no 95, 0,4 m à frente (`levantar_costas.constraints.json`) | 6 / 05 |

Clipes: `tools/animation/clips/stand_up_kimodo.py front|back`.
| `passe_esquerdo.bvh` | A soccer player standing behind a ball steps forward with the left foot and passes the ball with the inside of the right foot, arms relaxed and swinging for balance. (saiu com o pé esquerdo) | 2,0 s | — | 5 / 04 |
| `chute_forte_direito.bvh` | A soccer player shoots the ball hard with the right foot, swinging the leg back and through, arms out for balance. | 2,0 s | — | 7 / 00 |

Clipes do jogo (gestos com a bola, 20/10; **removidos em 24/10** com o gesto antigo - os gestos agora sao
procedurais, `src/Engine/Gesture`): `bola_passe_e` (o passe acima) e o espelho `bola_passe_d`;
`bola_chute_d` (o chute acima) e o espelho `bola_chute_e` (`tools/animation/espelhar.py`). Importados com
`--source-rig soma --root-motion in-place --reference-only` (o Blender precisa de `env -i HOME=$HOME PATH=/usr/bin:/bin`:
o ambiente do shell aponta o Python dele para o uv e quebra a biblioteca padrão). Os outros candidatos convertidos
estão em `estudos_bola/` (fora da pasta de clipes do jogo). O Kimodo não gerou um chute correndo: os textos de
"jogging ... kicks" saíram só corrida; o chute correndo usa o chute parado com o pé de apoio da corrida.

## Quedas com proteção e tropeços (01/11)

Referências de estilo (AMP) para o levantar: ao cair, mãos, joelhos, quadril e coluna amortecem; no tropeço e no
empurrão, passos rápidos recuperam. Gerados em lote (`tools/animation/kimodo/lote.py`, planos em
`~/Projetos/kimodo/saidas/protecao/lote1..4.json`, 6–8 amostras por variação) e escolhidos por
`tools/animation/kimodo/queda.py` e pelas folhas. Todos 3,0 s e em pé no quadro 0 (a pose de pé do fim de
`e3_08`, `saidas/e3/e3_08.npz:89`). Importados com `--source-rig soma --root-motion preserve --reference-only` (a
queda e os passos andam) e limpos por `tools/animation/clips/queda_protecao.py`.

| Arquivo | Texto | Poses-chave | Semente / variação |
|---|---|---|---|
| `queda_protecao_frente.bvh` | A person standing upright loses balance and falls forward, reaching both arms forward to break the fall, landing on the hands and then on the knees, and slowly lowers the chest to the ground. | de pé no 0, de bruços no 89 a 0,8 m à frente (`queda_protecao_frente.constraints.json`) | 11 / `frente_a_k_02` |
| `queda_protecao_tras.bvh` | A person standing upright loses balance backward, bends the knees and sits down on the ground with both hands behind to break the fall, then rolls back onto their back with the chin tucked to the chest. | de pé no 0 | 22 / `tras_a_0_01` |
| `queda_protecao_lado.bvh` | A person bends down on their legs and falls down on the floor to their left side. | de pé no 0, de costas no 89 com a cabeça para a esquerda, a 0,8 m (`chave.py` com giro −90°) | 35 / `lado_b_kb_04` |
| `queda_protecao_tropeco.bvh` | A person walking forward trips on an object, stumbles forward and takes two quick steps to regain balance, then stands upright. | — | 41 / `tropeco_a_07` |
| `queda_protecao_empurrao_lado.bvh` | A person standing upright stumbles sideways to the right, losing their balance momentarily and then regaining balance and standing upright. | de pé no 0 | 54 / `emp_lado_b_0_00` |
| `queda_protecao_empurrao_tras.bvh` | A person standing upright loses balance backward, their legs and arms move outward and they step backward quickly to regain balance, then stand upright. | de pé no 0 | 64 / `emp_tras_b_0_01` |
