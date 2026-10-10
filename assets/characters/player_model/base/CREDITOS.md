# Créditos das malhas base

O jogador é montado a partir de três malhas de terceiros, todas sob a
licença Creative Commons Atribuição 4.0 Internacional (CC BY 4.0):
<https://creativecommons.org/licenses/by/4.0/>.

| Arquivo | Obra | Autor | Origem |
| --- | --- | --- | --- |
| `corpo_base.glb` | "Male Lowpoly Base Mesh" | CarvMad | <https://sketchfab.com/3d-models/male-lowpoly-base-mesh-44d9a06186fa4068a50ab421bfcc38b2> |
| `cabeca_base.glb` | "lowpoly man's head" | Jelveh 3D | <https://sketchfab.com/3d-models/lowpoly-mans-head-3a823d8ee4114ecbba1b9a354f980a52> |
| `molde_cabeca.glb` | "Zbrush head base" | hexmeff | <https://sketchfab.com/3d-models/zbrush-head-base-7ff989b57b6041c89c406cf51d88b770> |

Os arquivos aqui são cópias sem alteração dos originais baixados.

## Alterações feitas no jogador

O modelo do jogo (a montagem, `montagem.json`) altera as duas obras:

- o corpo é escalado para 1,80 m e montado a partir da metade direita,
  espelhada;
- a cabeça é escalada, cortada no pescoço e costurada no corpo;
- o queixo, a mandíbula e o pescoço da cabeça base vão para a superfície da
  "Zbrush head base", usada só como molde de forma: a malha dela não entra
  no jogo;
- os pés do corpo são trocados por chuteiras geradas pela engine;
- a camisa e o calção são feitos de cópias da pele do corpo;
- o modelo recebe pesos de skin, um mapa de textura e uma textura pintada
  pela engine (uniforme, meião, cadarço, olhos, sobrancelhas e boca).
