# Personagem visual dirigido pelo ragdoll

O personagem padrão é o **jogador montado pela própria engine**
(`assets/characters/player_model`, ver `MODELAGEM_DO_PERSONAGEM.md`),
exportado para o jogo em `assets/characters/football_player` (09/10/2026).
Manifesto: `assets/characters/football_player/character.json`. O jogador é
usado no controle do personagem, nos ragdolls do laboratório, no visualizador
de animações e no de toque na bola. O Ch38 (o FBX fornecido em 26/09), os
arquivos dele e as ferramentas que o preparavam
(`tools/prepare_football_player.py`, `tools/fit_ragdoll_colliders.py`) foram
removidos; o ALS anterior também.

## Exportar

```bash
build-profile/MatterModelInspector exportar-jogo --destino assets/characters/football_player
# opcional: --quinas 45 (graus) --facetado 0.5
```

Lê o modelo montado (o mesmo da tela "Inspeção de jogador"), mede o corpo e
grava quatro arquivos:

- `character.json`: identidade, papéis das animações, procedência e o
  sombreamento (`flatShaded` falso, `facet` 0,5);
- `FootballPlayerV1.ragdoll.json`: o perfil físico (18 links);
- `jogador.skin.json`: a pele (versão 2);
- `jogador_atlas.png`: o atlas da montagem (2048 px).

O código é `src/Engine/Modeling/Rig/GameCharacterExport.cpp`. Exportar de
novo no mesmo corpo dá os mesmos arquivos (ponto fixo); o teste
`MatterPlayerModelTests` falha se o modelo mudou e o personagem do jogo ficou
para trás ("o personagem do jogo está desatualizado"). Depois de mudar o
modelo, exporte de novo.

## Compatibilidade com as animações

O rig continua o calibrado: as juntas (os frames, os eixos twist/swing, os
limites, as rigidezes, os amortecimentos e os torques) e as massas são os do
perfil anterior. Só mudam as âncoras, as posições dos links e os colisores,
medidos no corpo novo na **pose T dele** - a pose de repouso do jogo com as
medidas do esqueleto medido (`esqueleto.json`). Os clipes guardam os giros
das juntas, então continuam valendo sem reamostrar.

- **A pelve na origem.** O espaço do jogo tem a pelve em z = 0 e o chão em
  `-standingRootHeightMeters` (0,9684 m), como o código da locomoção conta
  (`rootPositionWorld.z - standingRootHeightMeters` é o chão). O espaço da
  modelagem é outro (o chão fixo em `model.json`, `skeleton.ground`); o
  exportador sobe tudo a altura que a pelve tem abaixo da origem.
- **O repouso em pé.** Na pose T do jogo as pernas ficam retas e verticais (na
  pose A da malha elas abrem e inclinam): o rig da modelagem sobe a pelve até
  a sola tocar o chão (`GameRig::rootRestLift`, 2,7 cm), e a altura de pé é a
  da pelve nesse repouso.
- **O movimento da raiz não é escalado.** A pelve do repouso fica 1,5% abaixo
  da do corpo antigo (0,9684 contra 0,9826 m); as animações ficam como
  estão. A modelagem passa a usar a escala 1 (a mesma pose do jogo).
- **Os links** ficam no mesmo lugar relativo do perfil anterior, na escala de
  cada osso (da junta até a do filho; na mão e no pé, até o dedo médio e os
  dedos do pé; na cabeça, a do pescoço). No Jolt o centro de massa de cada
  link é a origem dele.
- **A coluna nas juntas medidas** (09/10/2026, a postura reta pedida pelo
  usuário). No repouso, cada osso da malha vai da direção medida no corpo
  para a do osso do jogo (de âncora a âncora). As âncoras da coluna vinham
  do corpo antigo, ~1 cm à frente das juntas medidas, e a da cabeça 1,4 cm:
  em toda animação o pescoço saía 11–14° para a frente e o alto do peito
  ~5° para trás. As âncoras e os links do abdômen, do peito, do alto do
  peito, do pescoço e da cabeça foram para as juntas medidas, a partir da
  pelve. Com isso o repouso do jogo é a ligação da malha e o olhar não muda
  (a orientação dos links é a mesma). O `exportar-jogo` mantém essas âncoras
  (ponto fixo). Os clipes não mudam. O comando `MatterModelInspector
  postura` mede o tronco, o pescoço e o olhar em cada clipe.
- **Pares do corpo que colidem:** os mesmos 133 do perfil anterior. O par
  que era desligado (sobreposto em repouso) e passaria a colidir fica
  declarado em `selfCollisionIgnoredPairs` (alto do peito × cabeça: o queixo
  no peito ao olhar a bola).
- **Constantes do corpo antigo no código:** a altura do tornozelo no levantar
  (era 0,135 m fixo) agora sai do perfil; o torque de atitude da pelve no
  carrinho (`T_TackleAttitude`) passou de 1,0 para 1,5 - na saída do deslize o
  centro de massa do corpo novo fica mais à frente dos pés e o tronco
  assentava a ~36° (o limite de "de pé" é 35°): os 5 carrinhos em sprint da
  varredura viravam queda; com 1,5, nenhum, e o deslize no chão fica o mesmo.

Uma junta mudou em 27/09 e continua assim: o **antebraço é uma dobradiça** (o
eixo de "pronação" herdado do ALS girava o antebraço inteiro em volta do
braço). Detalhes em `ANIMATION_AUTHORING.md`.

## Colisores

Medidos na malha em repouso por pontos espalhados por igual na superfície (a
malha low poly tem poucos vértices: o tronco é uma volta na cintura e outra
no peito). Cada ponto é do link que mais pesa nele quando o peso passa de 0,6;
a pele de transição (axila, virilha, ponta do ombro) fica com os dois
vizinhos e com nenhum colisor.

- **Membros** (braço, antebraço, coxa, canela): cápsula cônica no eixo do
  osso, deslocado pelo centro da malha no meio do segmento; o raio de cima
  (15 a 35% do osso) e o de baixo (75 a 92%; na coxa, 90 a 99%, o joelho
  depois da barra do calção) - mediana nas pernas, percentil 70 nos braços.
  As tampas vão até as pontas da malha e pelo menos de uma junta à outra
  (sem isso sobrava um vão no joelho).
- **Mão, pescoço e cabeça:** cápsula uniforme (percentil 70; a mão aberta,
  até a ponta do dedo médio; a cabeça, até o alto). Toda cápsula tem pelo
  menos 2 mm de cilindro: o osso do pescoço é mais curto que o diâmetro
  dele, e as duas esferas sozinhas o perfil recusa (o comprimento tem de
  passar a soma dos raios). Antes era o arredondamento do arquivo que
  decidia, e em 09/10/2026 o perfil exportado saiu inválido.
- **Tronco** (pelve, abdômen, peito, alto do peito): cápsulas deitadas (o x
  local é o lado), em faixas de altura entre as juntas (o skin do tronco
  espalha o peso: nenhum ponto é do peito); raio a média da meia
  profundidade e da meia altura, mas nunca mais que a meia profundidade (na
  pelve a média passava 3 cm do glúteo); comprimento a largura.
- **Pé:** caixa no rumo da chuteira (o eixo maior visto de cima: 8,4° para
  fora; a antiga tinha 2,9°), comprimento e largura pelos percentis 1 e 99, a
  altura do perfil (13 cm) com o fundo no chão do repouso.
- **Coxas:** 2,4 cm uma da outra em repouso (só o raio de cima encolhe).
- **Folga nas animações de locomoção** (as do manifesto, fora levantar,
  carrinho e arrancada agachada; 902 poses): o braço solto não encosta no
  tronco (0,4 cm) e as coxas não se cruzam na passada de lado (0,8 cm). Na
  malha o braço encosta no tronco (axila, latíssimo); o colisor é o núcleo.
  Estreita primeiro o tronco (até 70% da largura medida: o peito fica com
  27,7 cm, a largura das costelas) e depois o membro na ponta do contato
  (até 70% do raio), igual dos dois lados. Sem isso a cápsula do braço
  entrava 5 a 7 cm na do peito e o corpo físico parado abria os braços 35 a
  41°.

Os números de cada colisor (o novo e o anterior, lado a lado) saem no
relatório do `exportar-jogo`.

Como os pontos são divididos pelos pesos do skin, mexer nos pesos do
uniforme muda os colisores. Em 09-10/10/2026, o quadril (`hipCrease`,
`hipLift`), a gola no peito e o calção subdividido
(`MODELAGEM_DO_PERSONAGEM.md`) mudaram estes:

| Link | Raio | Comprimento |
|---|---|---|
| Pelve | 11,8 → 12,5 cm | 24,8 → 26,3 cm |
| Alto do peito | 10,8 → 11,1 cm | 33,8 → 33,1 cm (centro 1 cm acima) |
| Coxas (no alto) | 8,0 → 8,2 cm | 40,3 → 40,8 cm |

A pelve no repouso e as folgas nas animações não mudaram.

## Pele (versão 2)

`matter-ragdoll-skin-2`: cada osso guarda o **repouso** (`restPosition`,
`restOrientation`: o link do perfil; os ossos visuais se prendem ao pai por
ele) e a **ligação da malha** (`bindPosition`, `bindOrientation`). A malha
fica na pose A dela e deforma direto dessa pose: a ligação de um osso físico
é `A T⁻¹ L` (A a junta na pose A, T a mesma junta no repouso, L o link). O
loader confere o repouso contra o perfil (pele velha é erro).

- **Ossos visuais:** os 30 dedos e os 2 dedos do pé, com os dados da mão
  relaxada e do punho aprovados (`flexion`, `close`, `closeAngle`,
  `restRotation`) - os mesmos do personagem anterior; o comprimento dos dedos
  do pé medido no corpo novo.
- **Auxiliares** (`helperBones`, depois dos visuais na paleta), 9 de cada
  lado: joelho, quadril (1/4, 1/2 e 3/4 do giro, e a `HipLift`), tornozelo,
  ombro, cotovelo e pulso. Giram a fração (`fraction`; 0,5 nas juntas) do
  giro do osso de baixo em relação ao de cima, em volta do pivô de verdade
  desse giro (com giro quase nulo, o `pivot` gravado) - a quina não perde
  volume. Com `"axis"` (no espaço da malha), só a parte do giro em volta do
  eixo (balanço e torção): a `HipLift` gira 45% só da flexão do quadril (o
  eixo de lado), e a coxa que abre de lado ou torce não mexe nela - a cintura
  do calção e a barra da camisa dobram para trás quando a coxa sobe. Sem
  física nem animação: `buildRagdollSkinMatrices3D` os tira dos dois ossos a
  cada pose (`twistAbout`, o mesmo da modelagem).
- **Vértices:** um por (vértice, grupo do sombreamento, coordenada do
  atlas): a quina (mais de 45° entre as faces) e a costura do atlas separam,
  o resto é liso; a normal é a lisa do grupo (`ShadingNormals`). 5.464
  vértices, 5.816 triângulos, 68 ossos (18 físicos, 32 dedos, 18
  auxiliares).
- **Sombreamento:** `facet` no manifesto (0 liso, 1 facetado); o shader
  mistura a normal lisa com a da face (`scene3d_mesh.frag`, bits 5 a 12 das
  flags do material). O padrão é 0,5 com quinas de 45° - o mesmo da tela de
  inspeção.

`matter_rig.Skin` (ferramentas Python) lê as duas versões, com os auxiliares;
confere com o motor a 0,0003 mm.

## Maciez (removida em 09/10/2026)

A maciez visual (as molas de carne e tecido no `src/Engine/Softness`) foi
testada e retirada a pedido do usuário. Saíram:

- o módulo e o alvo `MatterSoftness`;
- o `Rig/SoftTissueExport` e o arquivo `jogador.maciez.json` (com a chave
  `softTissue` do manifesto);
- os ossos macios da paleta e os toques da bola no estado dela
  (`ragdollTouches`);
- o menu MACIEZ do laboratório.

`buildRagdollSkinMatrices3D` voltou a ser só os ossos do jogo.

## Verificação

- `MatterPlayerModelTests`: exporta para uma pasta temporária, compara com o
  personagem do jogo (em dia) e compara a pele do jogo
  com a malha posada da modelagem em cinco poses (repouso, parado, sprint,
  carrinho, levantar: menos de 0,5 mm; medido 0,02 mm), e a sola no chão do
  perfil.
