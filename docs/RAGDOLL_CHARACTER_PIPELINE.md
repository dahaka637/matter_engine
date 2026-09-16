# Personagens visuais dirigidos por ragdoll

O personagem principal é `CrashTestDummyV1`, recebido em
`Crash Test Dummy mark1.zip`. O pacote contém duas versões: Rigify (431 ossos
de controle/deformação) e Unity/Mecanim (63 ossos). Usamos a segunda, com os
pesos, UV e topologia do autor. A divisão do peito acrescenta um osso visual:
64 ossos no Blender, 18 links físicos e 41 graus de liberdade na engine.

## Contratos

- `character.json` (`matter-ragdoll-character-1`) escolhe perfil físico,
  superfície com pesos, albedo, thumbnail, nome e procedência.
- `CrashTestDummyV1.ragdoll.json` define proporções, anchors, frames, massas,
  colisores e limites. Raios de cápsulas podem variar por link; o campo legado
  `uniformRadiusMeters` é somente o valor padrão quando `radius` é omitido.
- `dummy.skin.json` (`matter-ragdoll-skin-1`) contém vértices, UV, normais,
  triângulos e quatro influências normalizadas por vértice. Nomes de ossos são
  resolvidos para IDs físicos durante a exportação.
- `dummy-rigged.blend` é a fonte editável normalizada. Conserva os ossos de
  dedos e pés. `dummy-rigged.glb` é a exportação de intercâmbio.
- `run_forward_dummy.matteranim.json` continua sendo **alvo articular**,
  validado contra o perfil novo. A malha visual não modifica os limites.

O Workbench carrega `characters/crash_test_dummy/character.json` por padrão.
Para experimentar outro asset compatível, definir `MATTERENGINE_CHARACTER`
como caminho de manifesto relativo a `assets` (ou absoluto). Preview e spawn
usam o mesmo asset; o catálogo seleciona apenas clipes do rig escolhido.

## Preparação desta fonte

```bash
blender --background --factory-startup --disable-autoexec --python-exit-code 1 \
  --python tools/prepare_crash_dummy.py -- \
  '/home/dahaka/Área de trabalho/Nova pasta/Crash Test Dummy mark1.zip' \
  --profile-template assets/physics/ragdolls/HumanAdultV1.ragdoll.json \
  --output assets/characters/crash_test_dummy
```

O adaptador normaliza altura para 1,80 m, eixo vertical +Z, frente +X e esquerda
+Y; converte a A-pose autoral em T-pose preservando os comprimentos. Desativa
o espelhamento de edição do Blender antes de mover os ossos: deixá-lo ligado
invertia os lados durante o bake e foi detectado pelos gates do retarget.

Os pesos do autor são preservados no Blender. No formato físico, ossos de
dedos/palma são associados à mão; dedos/calcanhares do pé são associados ao
pé. Isso mantém os detalhes e a pose original, sem inventar graus de liberdade
dinâmicos que ainda não têm consumidor. Pesos de peito são distribuídos entre
Chest/UpperChest. Após combinar aliases, o máximo descartado para quatro
influências é 1,274%; o limite explícito deste asset é 2%, com renormalização
e relatório de exportação. O exportador genérico usa limite padrão de 1%.

O pacote inclui sua licença CC0 1.0 em `SOURCE-LICENSE.html`; textura amarela
original em `albedo.png`. Nenhum script embutido do Rigify precisa executar.

## Editar e reexportar

Depois de editar pesos ou geometria na fonte preparada, exportar em pose
neutra, com os anchors alinhados ao perfil:

```bash
blender --background --factory-startup --disable-autoexec \
  assets/characters/crash_test_dummy/dummy-rigged.blend \
  --python-exit-code 1 --python tools/export_ragdoll_skin.py -- \
  --mesh CrashTestDummy --armature CrashDummySkeleton \
  --profile assets/characters/crash_test_dummy/CrashTestDummyV1.ragdoll.json \
  --output assets/characters/crash_test_dummy/dummy.skin.json
```

O exportador rejeita ossos sem mapeamento, anchors divergentes, vértices sem
peso, pose não neutra e descarte excessivo de influências. O runtime rejeita
schema/rig divergentes, bind obsoleto, pesos inválidos e índices fora da malha.

Se proporções, posição dos links ou frames físicos mudarem, reexportar a pele
e reimportar as animações. Não trocar só o ID do rig em um clipe antigo:

```bash
blender --background --factory-startup --disable-autoexec --python-exit-code 1 \
  --python tools/import_mixamo_animation.py -- /home/dahaka/Downloads/Running.fbx \
  --profile assets/characters/crash_test_dummy/CrashTestDummyV1.ragdoll.json \
  --output assets/animations/clips/run_forward_dummy.matteranim.json \
  --id run_forward_dummy --name 'Correr para frente' --loop --root-motion in-place
```

## Render e física

`RagdollCharacter3D` é independente de Blender, Workbench e Vulkan. Cada
matriz visual é `transformação física atual × inversa do bind físico`.
As instâncias compartilham buffers de malha/textura; só as paletas atual e
anterior mudam. Skinning ocorre na GPU, com o mesmo cálculo nos passes de cor,
profundidade e sombra. O histórico de ossos alimenta os vetores de movimento.
Paletas e descritores usam slots protegidos pelos fences dos frames em voo.

O preview amostra alvos limitados e reconstrói anchors coincidentes. O jogo
usa os transforms **resultantes do PhysX**, depois da simulação; a pele jamais
escreve transforms no controlador. O envelope de spawn é derivado da malha
e dos colisores, sem dimensões fixas do boneco antigo.

## Aceitação

- Testes `animation`: bind sem deformação, transformação global, ciclo real,
  superfície finita e anchors contínuos.
- Testes `character`: 22 corpos em contato e postura ativa por seis segundos
  sem assistência de raiz/coluna, raycast e agarrar/erguer/soltar pela Physgun.
  Disponível também como CTest
  `MatterEngine.Character`.
- Inspecionar frente, costas e perfis na timeline, principalmente ombro,
  cotovelo, virilha, joelho e sola. Aprovação numérica não substitui a visual.
- Conferir spawn, Physgun, sombra e ciclo de resize/minimize/fullscreen com
  validação Vulkan ativa.

A corrida do dummy mede RMS de 4,12°, máximo global de 9,96° e máximo nos
membros de 6,02°. Limitações atuais: um material/albedo por superfície; dedos
ainda seguem rigidamente seus links físicos; playback da corrida continua
exclusivo do visualizador. A regressão histórica de recuperação de impacto
do controlador ativo permanece separada desta integração.

Na entrega, Character/Audio passaram em Debug; Foundation reproduziu a
falha conhecida de recuperação após rajada esquerda no perfil antigo.
Smokes de janela/preview/spawn com validação Vulkan não registraram erros.
A reexportação da fonte preparada reproduziu a pele runtime byte a byte.
O spawn deriva a altura no chão do envelope real e usa a mesma margem de
segurança na busca e na checagem final. Spawns aéreos ainda podem cair sem
recuperação automática: não confundir estabilidade do bind com equilíbrio
dinâmico ou aprovação visual do usuário.
