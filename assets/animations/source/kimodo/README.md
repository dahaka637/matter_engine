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
