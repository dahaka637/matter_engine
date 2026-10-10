# Movimentos gerados no Kimodo (NVIDIA)

Modelo `Kimodo-SOMA-RP-v1.1`, gerados com `tools/animation/kimodo/gera.sh`
(BVH no esqueleto SOMA, T-pose padrão) e retargeteados com
`tools/import_humanoid_animation.py --source-rig soma --reference-only`.

| Arquivo | Texto | Duração | Poses-chave | Semente / variação |
|---|---|---|---|---|
| `levantar_frente.bvh` | A person lying on their stomach on the ground quickly gets up and stands facing forward. | 3,0 s | de bruços no quadro 0, em pé no 89 (`levantar_frente.constraints.json`) | 4 / 08 |
| `levantar_costas.bvh` | A person lying flat on their back swiftly rises to a standing position, using their right hand for support. | 3,2 s | de costas no quadro 0, em pé no 95, 0,4 m à frente (`levantar_costas.constraints.json`) | 6 / 05 |

Clipes: `tools/animation/clips/stand_up_kimodo.py front|back`.
