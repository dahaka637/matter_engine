# Personagem visual dirigido pelo ragdoll

O personagem padrão é o jogador de futebol `Ch38_nonPBR.fbx`, fornecido pelo
usuário. Manifesto: `assets/characters/football_player/character.json`.
O jogador é usado tanto no controle do personagem quanto nos ragdolls criados
no laboratório e no visualizador de animações. O modelo ALS anterior foi removido.

## Compatibilidade com o trabalho de animação

O perfil `FootballPlayerV1.ragdoll.json` conserva os anchors, frames, massas
e limites do rig físico anteriormente calibrado. O importador ajusta a malha
do jogador a esse rig, por segmento e com os pesos suaves do FBX. Isso
preserva as animações já autoradas. Os tracks dos clipes não foram
reamostrados: apenas `targetRigId` foi atualizado para o novo nome.

Uma junta mudou: o **antebraço é uma dobradiça** (27/09). O eixo de
"pronação" herdado do ALS (`swing1` do antebraço) girava o antebraço inteiro
em volta do braço com o cotovelo dobrado, e o motor dele era fraco. No corpo
físico, o braço sacudia. Nenhum clipe usava esse eixo, fora os 10° do parado,
que agora vêm do giro do braço. Detalhes em `ANIMATION_AUTHORING.md`.

Os **colisores** são medidos da malha do jogador por
`tools/fit_ragdoll_colliders.py` (cápsulas cônicas nos membros, cápsulas
laterais no tronco, caixa da chuteira no pé), com as regras e os motivos
descritos no script e em "Colisores do jogador", em
`docs/ANIMATION_AUTHORING.md`. Rodar depois do preparador:

```bash
python3 tools/fit_ragdoll_colliders.py \
  assets/characters/football_player/FootballPlayerV1.ragdoll.json \
  assets/characters/football_player/football.skin.json
```

O ajuste modifica as proporções da malha de origem para casar com o rig;
não é um novo perfil físico medido do FBX. Se o perfil mudar futuramente,
será necessário retargetear e validar os clipes, não apenas renomeá-los.

## Arquivos e reprodução

- `Ch38_nonPBR.fbx`: fonte original, incluindo todas as texturas embutidas.
- `FootballPlayerV1.ragdoll.json`: contrato dos 18 links físicos.
- `football.skin.json`: skin indexada com até quatro influências por vértice.
- `albedo.png`: atlas das texturas de corpo/uniforme e cabelo, 4096 × 2048.
- `football-rigged.blend`: malha ajustada, material e rig físico editáveis.
- `character.json`: identidade, caminhos, procedência e papéis das animações.

```bash
blender --background --factory-startup --disable-autoexec --python-exit-code 1 \
  --python tools/prepare_football_player.py -- \
  --source assets/characters/football_player/Ch38_nonPBR.fbx \
  --output assets/characters/football_player
```

O importador usa o perfil calibrado que já está no diretório de saída. Ele
não precisa dos arquivos do personagem antigo. Frente +X, esquerda +Y, cima +Z.
Dedos dos pés seguem o pé e clavículas seguem UpperChest. O mapeamento é
explícito; ossos com pesos sem correspondência causam erro.

## Dedos: ossos visuais

Os dedos são **ossos visuais**: 3 por dedo (polegar, indicador, médio,
anelar, mínimo), 15 por mão. São filhos do link da mão e não têm física. Na
skin, ficam em `visualBones`, depois dos 18 ossos físicos na paleta, cada um
com o osso pai, a pose de ligação (`bindPosition`, `bindOrientation`: eixo X
ao longo do dedo, Z para o lado da palma) e a rotação de repouso
(`restRotation`), que dá a mão relaxada: flexão em cascata do indicador ao
mínimo, dedos juntos e polegar encostado no indicador. Os ângulos ficam em
`RELAXED_FLEXION` e `RELAXED_CLOSE`, no preparador.

O Ch38 tem os ossos dos dedos, mas quase nenhum peso neles: o dedo inteiro
segue o osso da mão (só a ponta do anelar usa o osso dele). Por isso o
preparador refaz os pesos da mão pela geometria: cada vértice vai para os
segmentos de osso mais próximos (inverso da distância à 8ª potência), a
palma para a mão (segmentos do punho até cada articulação dos dedos) e cada
falange para o seu osso, misturando nas juntas.

No motor, `buildRagdollSkinMatrices3D` calcula o osso visual como o pai
vezes a ligação local vezes a rotação de repouso (`RagdollVisualBone3D`);
`matter_rig.Skin` faz o mesmo nas ferramentas Python. Animar a mão, para o
goleiro no futuro, é trocar essas rotações.

O runtime de personagem usa um único albedo opaco. Por isso o importador
compõe as duas texturas difusas em um atlas e converte os recortes alpha de
cabelo/cílios em geometria. Não altera shaders ou transparência de outros objetos.
Normal/specular/glossiness originais ficam preservados no FBX, mas não são
usados pelo material atual do personagem.

Após combinar influências (com os dedos), o descarte máximo medido para
quatro pesos foi 3,785% (12 vértices acima de 2%); o limite do asset é 4%. Pesos restantes são
renormalizados. Vértices idênticos são compartilhados, preservando costuras UV,
normais e pesos: 51.841 vértices de runtime e 76.518 triângulos nesta exportação.

## Edição e verificação

Para alterações de geometria, preferir ajustar o importador e regenerar a
partir do FBX. O Blender preparado permite editar os pesos e reexportar em
pose neutra com `tools/export_ragdoll_skin.py`, mesh `FootballPlayer`, armature
`FootballPhysicalRig`. Esse exportador genérico grava triângulos sem compartilhar
vértices; a etapa final do preparador faz a indexação.

O exportador valida alinhamento dos anchors e qualidade dos pesos. O loader
valida bind, índices, pesos, identidade do perfil e ossos visuais (pai antes
do filho, quaternions unitários). O teste de skin verifica repouso (só os
dedos saem da malha modelada, até o comprimento de um dedo), transformação
global e deformação durante animação. As ferramentas
Python em `tools/animation/matter_rig.py` também fazem blend das quatro
influências, em vez da aproximação antiga pelo osso dominante.

A variável `MATTERENGINE_CHARACTER` pode selecionar outro manifesto relativo
a `assets` ou absoluto. O personagem low-poly continua como fixture de testes.

Os papéis de locomoção estão em `character.json`, com os de strafe
(`strafeLeft`/`strafeRight` e os de sprint), que são a referência Jog
Strafe retargeteada e corrigida (`tools/animation/clips/strafe.py`).
