# Modelagem do personagem: inspeção e ferramentas

O jogador vai deixar de ser o modelo gratuito (`Ch38`) e passar a ser gerado
pela engine, parte por parte, para depois ficar customizável (peso, altura,
pele, uniforme, cabelo, rosto). Este documento descreve a infraestrutura
pronta para isso: a tela **Inspeção de jogador** do Workbench e as
ferramentas de linha de comando que o Claude e o Codex usam para modelar.

Estado em 08/10/2026: o jogador é **montado de duas malhas base prontas**
(seção "A montagem do jogador"), no estilo facetado delas e leve (4 664
triângulos com as chuteiras):

- o corpo, com as mãos como vieram: "Male Lowpoly Base Mesh" (CarvMad),
  levado a 1,80 m;
- a cabeça original: "lowpoly man's head" (Jelveh 3D), costurada no pescoço;
- as chuteiras procedurais (seção "A chuteira"), sem o cadarço em malha;
- o uniforme (camisa e calção) feito da própria pele do corpo, com o recuo
  para dentro nas barras;
- os detalhes por textura num atlas único: as faixas, os punhos, a gola e os
  números do uniforme, o meião, o cadarço e o rosto (olhos, sobrancelhas e
  boca).

As duas malhas base são CC BY 4.0: os créditos ficam em
`assets/characters/player_model/base/CREDITOS.md`. O corpo procedural
anterior (todas as peças da seção "Peças procedurais") está guardado em
`model_antigo.json` para reaproveitar partes.

É **uma superfície só e fechada**: as peças são costuradas vértice com
vértice (seção "Conexão das peças") e nenhuma borda fica aberta.

**É o personagem do jogo** desde 09/10/2026: `MatterModelInspector
exportar-jogo --destino assets/characters/football_player` grava o perfil
físico medido nele, a pele e o atlas (seção "Para o jogo"); o Ch38 saiu.

## Arquitetura

```
MatterCore
  └─ MatterModeling        src/Engine/Modeling/{Geometry,Raster,Skeleton,Mesh,Analysis,Inspection,Parts,
                                                Import,Skin,Texture,Paint,Assembly}
       └─ MatterModelingRig  src/Engine/Modeling/Rig  (+ MatterEngine: personagem e animações do jogo)
            ├─ MatterModelInspector  tools/ModelInspector  (um arquivo por comando)
            └─ MatterWorkbench       src/Workbench/PlayerInspector (a tela)
```

- **`MatterModeling`** depende só do núcleo. Roda sem janela e tem, cada
  coisa no seu módulo:
  - geometria (com a triangulação de Delaunay restrita, `Triangulation2D`);
  - malha e kit de modelagem (com as costuras, o recorte, o relevo, os
    puxões finos e os vértices levados a lugares exatos, `Mesh/MeshSculpt`,
    o encaixe numa referência densa, `Mesh/SurfaceFit`: o alinhamento de
    Horn com ICP e a projeção na superfície, e as normais do sombreamento
    liso, `Mesh/ShadingNormals`);
  - o paralelismo (`Geometry/Parallel`: o agendador de tarefas da engine,
    `Core/TaskScheduler`, um só para a modelagem, com os workers vivos entre
    as chamadas);
  - esqueleto e regiões;
  - análise;
  - renderizador por software (com textura);
  - sessão interativa;
  - `Import/GltfImport` (glTF/GLB pelo cgltf, ligado só aqui dentro);
  - `Skin/BoneHeatSkin` (os pesos pelo calor dos ossos) e
    `Skin/SkinSmoothing` (espalhar, misturar com o espelho, prender);
  - `Texture/UvAtlas` (o mapa de textura, com o arranjo das ilhas pelo
    formato em `Texture/IslandPacking`) e `Texture/TextureBake` (a pintura
    para o atlas);
  - `Paint/` (a pintura em dados: formas vetoriais, fontes, caminhos de SVG,
    imagens, os espaços na superfície e o desenho em JSON);
  - `Assembly/` (a montagem do jogador: ver a seção dela; o pescoço mais
    curto em `NeckShortening`, a faixa do pescoço com os pesos dela em
    `NeckBand` e o molde da cabeça de referência em `ReferenceFit`).

  `tools/check-architecture.sh` proíbe incluir a engine, o ImGui ou o SDL aqui;
  as camadas de baixo (`Mesh`, `Skin`, `Import`, `Texture`, `Paint`,
  `Geometry`, `Raster`, `Skeleton`, `Analysis`) não incluem a montagem, as
  peças, o rig
  nem a inspeção, e a montagem não inclui o rig nem a inspeção.
- **`MatterModelingRig`** é a única ponte com o jogo:
  - `GameRig`: o esqueleto a partir do `RagdollCharacter3D` e as poses das
    animações;
  - `GameCharacterExport`: o jogador levado ao jogo (o perfil físico, a pele
    e o atlas);
  - `PlayerModelAsset`: o manifesto.
- **A tela** só desenha a interface e repassa o mouse para a
  `InspectionSession`. Ela fala com a GPU só pela interface `CanvasTarget`
  (também uma regra do `check-architecture`). Os testes rodam a tela sem
  janela, com um canvas falso.
- **Um renderizador só.** A imagem do viewport da tela e as imagens da
  linha de comando saem do mesmo `renderInspection`. O que o Claude vê nas
  imagens é o que o usuário vê na tela.

## O modelo: `assets/characters/player_model/`

- **`model.json`** (manifesto):
  - `character`: o personagem do jogo de onde vem o esqueleto (hoje
    `../football_player/character.json`);
  - `regions`;
  - `mesh.source`: `montagem` (o modelo novo, com `mesh.assembly` =
    `montagem.json`), `procedural` (o corpo antigo das peças, com
    `mesh.parts`) ou `nenhuma`;
  - `skeleton`: os ajustes do modelo novo sobre o esqueleto do jogo (ver
    "Esqueleto"); no modelo montado, `fit` = `esqueleto.json` e `ground` = o
    chão do espaço do modelo (fixo: as peças, a pintura e os ajustes do rosto
    foram postos nele; o perfil do jogo tem a pelve na origem e outro chão);
  - `references`: as imagens de referência e a vista de cada uma
    (`frente`, `costas`, `direita`, `detalhe`).
- **`regions.json`**: as 18 partes do menu.
  - Cada parte tem `id`, `label`, `roots` (juntas raiz), `side`, `view`
    (yaw/pitch do close-up) e `margin`.
  - Uma junta pertence à região da raiz mais próxima entre os seus
    ancestrais. Um osso pertence à região da junta de onde ele sai: o
    antebraço é o osso do cotovelo ao pulso.
- **`montagem.json`**: a receita da montagem (os campos da
  `AssemblyRecipe`; o que não estiver no arquivo fica no padrão do código).
- **`esqueleto.json`**: as juntas medidas no corpo novo.
- **`base/`**: as duas malhas base (`corpo_base.glb`, `cabeca_base.glb`) e os
  créditos delas (`CREDITOS.md`).
- **`model_antigo.json`**: o manifesto do corpo procedural anterior
  (`--modelo assets/characters/player_model/model_antigo.json`).
- **`partes/`**: as receitas das peças (JSON); a montagem usa a chuteira.
- **`referencias/`**: cópias das imagens do usuário. Os originais ficam em
  `~/Área de trabalho/modelagem/`.

A tela recarrega sozinha quando esses arquivos mudam (verifica a cada 1 s).

## Esqueleto (o mesmo do jogo)

- **Juntas:** 81 no total:
  - os 18 ossos físicos;
  - 30 dedos das mãos e 2 dedos dos pés (ossos visuais);
  - 13 pontas (`HeadTop`, `<Dedo>Tip`, `<Lado>ToeTip`), só para desenhar o
    osso até o fim. No modelo montado vêm do esqueleto medido;
  - 18 **auxiliares do skin** (`JointKind::Helper`), 9 de cada lado:
    - `KneeHelper` (coxa → canela);
    - `HipHelper25`, `HipHelper`, `HipHelper75` (pelve → coxa: 1/4, 1/2 e
      3/4 do giro);
    - `HipLift` (pelve → coxa: 45% só da flexão; abaixo);
    - `AnkleHelper` (canela → pé);
    - `ShoulderHelper` (alto do peito → braço);
    - `ElbowHelper` (braço → antebraço);
    - `WristHelper` (antebraço → mão).

    Elas não têm osso próprio: a pose vem de duas juntas. Cada uma gira uma
    fração (`derivedFraction`) do giro da de baixo em relação à de cima, em
    volta do eixo de parafuso do movimento de verdade (numa dobradiça pura, a
    própria junta; `updateDerivedJoints`). Com a frente do joelho presa na
    auxiliar, a quina da dobra fica com o raio cheio, sem o "cano de PVC
    dobrado" do skin linear. Com eixo (`derivedAxis`, no espaço do modelo),
    a auxiliar gira só a parte do giro em volta dele (a decomposição balanço
    e torção, `twistAbout`): a `HipLift` usa o eixo de lado (y), então segue
    só a flexão do quadril; a coxa que abre de lado ou torce não mexe nela.
    No jogo elas são os ossos auxiliares da pele (`helperBones`, com
    `"axis"` quando há eixo), calculados do mesmo jeito.
- **Dedos:** cada osso dobra em volta do −y dele (o dorso, para onde a unha
  olha, é o −z do osso), como em `RagdollVisualPose3D`. O polegar é girado em
  relação aos dedos: em volta do −y ele iria para baixo da palma, então dobra
  em volta do meio entre o −y e o +z dele e fecha atravessando a palma, rente
  a ela (`fingerFlexionAxis`). Os ossos da mão esquerda são o espelho dos da
  direita (`mirrored`: o y do osso troca de sinal, o x e o z não). Por isso os
  dedos usam o −y dos dois lados, o polegar esquerdo usa o meio entre o −y e
  o −z, e todo giro em volta do z do osso (o `closeAngle` do jogo, o giro para
  dentro do punho) troca de sinal na esquerda. No esqueleto medido, o y das
  falanges vem do plano em que cada dedo curva na malha nova (as três
  falanges e a ponta; quase reto, o lado da mão). O indicador da malha base
  curva 26° fora do lado da mão; com o lado, ele fechava torto.
- **O esqueleto medido** (`skeleton.fit` = `esqueleto.json`, o modelo
  montado): a posição de cada junta do lado direito e do meio, `[x, y, altura
  do chão]`, medida no corpo novo (o cotovelo no meio dos três anéis do
  braço, o pulso no mais fino, os dedos pelos anéis de quatro vértices); o
  lado esquerdo é o espelho. Cada osso aponta para a junta filha medida e a
  orientação da ligação é a do jogo levada até ela (`O_j`: o eixo do osso e
  um secundário - o de um lado ao outro da mão nos dedos, a normal da palma
  no polegar, +z se o osso corre em x, senão +x). Nas poses, cada junta gira
  como a do jogo (a mesma orientação no mundo) e fica onde o pai a põe. O
  repouso (a pose T do jogo: as pernas retas e verticais) fica em pé no chão
  - a pelve sobe até a sola tocar (`rootRestLift`, 2,7 cm) - e o movimento
  da raiz escala pela altura da pelve nesse repouso sobre a do jogo
  (`rootScale`: 1 com o jogador exportado; era 0,958 com o Ch38). Faltando
  uma junta no arquivo, o modelo não carrega.
- **Ajustes do corpo antigo** (`model_antigo.json`, bloco `skeleton`), só no
  esqueleto da modelagem; **o jogo precisa receber os mesmos** se esse
  modelo entrar nele:
  - `linkShift`: links físicos deslocados (hoje a corrente do braço,
    `UpperArm`/`Forearm`/`Hand`, 3,5 cm para a frente: o ombro no meio do
    braço das referências). Nas poses, cada junta fica onde o pai a põe (a
    ligação relativa do esqueleto do modelo); o jogo só dá os giros dos links.
    Sem isso, o cotovelo dobrado saía 4,5 cm do lugar;
  - `mirrorFingersFrom: "Right"`: os dedos da mão esquerda (ossos e pontas)
    são o espelho exato dos da direita. A malha é espelhada; no jogo os dedos
    dos dois lados diferem 1 a 5 mm, o bastante para torcer os dedos
    esquerdos ao dobrar.
- **Espaço do modelo:** z para cima, x para a frente, y para a esquerda, em
  metros. Chão em z = −0,9776 (`skeleton.ground`, a altura de pé do perfil
  do Ch38, mantida). O jogo tem a pelve na origem: o exportador sobe tudo
  0,9 cm (a pelve do repouso fica em z = −0,009 aqui).
- **Onde ficam as juntas:** nas do corpo medido (`esqueleto.json`), que viram
  as âncoras das juntas físicas do perfil exportado: no repouso, quadril a
  93,9 cm do chão, joelho a 52,1 cm, tornozelo a 10,7 cm.
- **Pose de ligação:** a do corpo base no modelo montado: pose A, com o braço
  56° abaixo da horizontal e a mão aberta. O modelo antigo usa a pose T do
  FBX.
- **Altura:** 1,80 m do chão ao alto da cabeça no modelo montado (o
  esqueleto do jogo, sem o ajuste, tem 1,906 m, com o cabelo do modelo
  antigo).
- **O punho** (`RagdollVisualPose3D`, a tabela `FistHand`): o polegar dobra
  pouco (21°, 6°, −2°), quase não gira na base (0,3 do giro até o indicador) e
  as juntas do meio e da ponta giram para dentro (52° e 27° em volta do z do
  osso): deita na diagonal na frente das falanges do meio do indicador e do
  médio. Antes ele ia para baixo da palma (nos dois esqueletos). Na mão
  esquerda o giro para dentro tem o sinal trocado, porque os ossos são o
  espelho; com o mesmo sinal, o polegar esquerdo saía para fora do punho. O
  punho só é usado pela mão manual do Laboratório e pela inspeção.

Usar o esqueleto do jogo (as juntas e os eixos dele) mantém física, ragdoll e
animações funcionando. As proporções novas vão para o perfil físico pelo
`exportar-jogo` (ver `RAGDOLL_CHARACTER_PIPELINE.md`); os clipes continuam os
mesmos.

## Linha de comando: `build-profile/MatterModelInspector`

`MatterModelInspector ajuda` lista tudo. Os comandos:

| Comando | Para quê |
| --- | --- |
| `info [--dedos] [--animacoes]` | juntas (posição, osso, região), regiões, malha, referências e o tempo de cada etapa da carga |
| `render` | uma imagem |
| `folha --vistas a,b --regioes x,y --modos m,n` | várias numa grade, com legendas |
| `volta --passos 8` | volta completa em torno |
| `analisar [--imagens N] [--json arq] [--poses a,b]` | checagens automáticas, com close-up de cada achado; o JSON traz os índices dos triângulos de cada achado (`triangleIds`, os primeiros 200) para o `pesos --triangulos` |
| `medir [--junta J --t 0.5]` | medidas do esqueleto e cortes ao longo dos ossos |
| `corte --junta J --t 0.5 \| --altura m \| --plano frontal\|sagital [--deslocamento m]` | o contorno da malha num plano, com os limites x/y/z de cada volta. Com `--pose`, a malha cortada na pose (o perfil da dobra do quadril com a coxa levantada); `--partes camisa,calcao` dá um contorno por parte, cada um numa cor; `--alvo x,y,z --raio m` enquadra a vista de frente num ponto |
| `comparar [--referencia png --vista v]` | o modelo sobre as referências, na mesma escala; `comparar_<vista>_lado_a_lado.png` põe a referência e o modelo sólido lado a lado, na mesma câmera |
| `poses --poses repouso,jog@0.25,...` | o modelo deformado pelas animações do jogo, com a checagem de deformação |
| `textura [--ilhas] [--fundo]` | o atlas de textura num PNG (com as arestas do mapa por cima; o fundo fora das ilhas cinza) |
| `pintura [--espaco tronco] [--lado esquerdo] [--passo 3] [--escala 2000]` | os moldes da pintura: cada espaço (`PlayerPaintSpaces`) desenrolado num PNG, com a cor final de cada ponto, a grade de 5 cm e as âncoras com nome (para desenhar no `pintura.json` sem adivinhar coordenada) |
| `molde` | o molde da cabeça de referência: o alinhamento (pontos, erro médio, escala, inclinação), o recuo da cabeça e a distância de cada vértice do queixo, da mandíbula e do pescoço à referência |
| `uniforme` | o uniforme sobre o corpo (a mesma montagem sem ele), comparado ao corpo da referência. Mostra a folga de verdade do pano até a pele por região (média, p90 e máximo, pela superfície), as larguras do tronco com e sem o uniforme e a linha do ombro de frente. Grava `uniforme.png` (as silhuetas de frente, de lado e de costas, com o pano além do corpo em vermelho, e os cortes do ombro, do peito, da cintura e do quadril, com o corpo em ciano e o uniforme em laranja) e `uniforme_ombro.png` (o ombro de perto, o corpo e o uniforme nos mesmos ângulos) |
| `postura [--clipes a,b] [--amostras 24]` | a postura nas animações do jogo: em cada clipe, a inclinação do tronco e do pescoço (da vertical e sobre o tronco), o quanto a cabeça vai à frente da base do pescoço e o olhar (média, mínimo e máximo), com o repouso do perfil e a ligação para comparar, e `postura.png` (cada clipe de lado) |
| `avesso [--instantes 6] [--raio 0.02]` | o avesso à vista em todas as animações do jogo (`--instantes` de cada, mão solta e punho: 528 poses em ~4 s, em paralelo): cada triângulo virado, em quantas poses, a primeira e onde fica na ligação; e os pontos do `weightRelax` da montagem prontos para colar (os da receita junto, na metade direita, cada um só na parte do triângulo). Sai com 0 quando não há avesso |
| `pesos --perto x,y,z [--raio 0.02] [--parte calcao]` ou `pesos --triangulos T2174,T2265` | os pesos do skin dos vértices perto de um ponto (espaço do modelo, na ligação) ou dos triângulos que o `avesso` aponta: o índice, a posição, as partes e cada junta com o peso. Serve para achar a causa de um avesso (um vértice com o peso de outro lugar) |

Opções comuns:

- **Sombreamento** (modo sólido): `--quinas <graus>` (as faces que dobram
  mais que isso ficam marcadas; padrão 45) e `--facetado <0..1>` (o quanto
  volta às faces chapadas; 0 liso; 0,5 o padrão, o mesmo do jogo; 1 o
  facetado de antes).
- **Modelo:** `--modelo <model.json>` (o corpo antigo:
  `assets/characters/player_model/model_antigo.json`), `--malha montagem|procedural|nenhuma`,
  `--pose <animação>@<s>` (`repouso`, `ligacao`, `sprint@0.3`...).
- **Dobras de teste** (em `--pose` e `--poses`):
  `dobra:RightShin=90+RightThigh=-60+LeftThigh=20x` gira juntas da ligação,
  em graus, em volta de um eixo do modelo (y se não disser; `x`/`z` no fim do
  número; `d` é a dobradiça da própria junta, `jointHingeAxis`:
  `RightIndex1=85d` fecha o dedo para a palma, `RightThumb3=80d` fecha o
  polegar atravessando ela, `RightForearm=120d` fecha o cotovelo no plano do
  braço e `RightShin=120d` dobra o joelho). Cada eixo é o da partida e gira
  junto com o pai (`followParent`): com o braço levantado, o cotovelo continua
  dobrando no plano do braço, e com o pulso dobrado os dedos ainda fecham para
  a palma. Joelho: + dobra. Quadril: − leva a coxa para a
  frente; em x, abre a perna. Tornozelo: − levanta a ponta do pé. Braço
  direito, para −y (na ligação do modelo montado, 56° abaixo da horizontal; no
  antigo, na horizontal): `RightUpperArm=30x` baixa, `=-56x` leva à
  horizontal, `=70z` leva para a frente;
  `RightForearm=90d` dobra o cotovelo (dos dois lados com o mesmo sinal).
  `RightFist=60` fecha a mão direita 60% da partida até o punho do jogo (o
  controle "Dedos" da tela; `LeftFist` a esquerda).
  Mão direita (palma para baixo): `RightHand=70x` dobra o pulso para a palma,
  `=-60x` para o dorso, `=25z` para o lado do polegar; os dedos
  `RightIndex1=90x+RightIndex2=100x+RightIndex3=70x` fecham. Coluna:
  `Abdomen=30` dobra para a frente, `=25z` torce. Pescoço e cabeça:
  `Neck=30` (para a frente), `Neck=30x` (de lado), `Neck=45z` (girando), o
  mesmo com `Head`.
- **A mão da pose** (no fim): `#reta` (a da ligação), `#repouso` (o padrão),
  `#aberta` ou `#punho` (as tabelas do jogo). Ex.: `ligacao#punho`,
  `sprint@0.3#aberta`. Numa dobra, só os dedos tomam a mão do jogo (o giro
  de cada um na mão; o corpo fica na ligação, como na tela; o padrão é a
  reta), e o eixo `d` é o dela:
  `dobra:RightThumb1=32d+RightThumb2=64d+RightThumb3=80d#punho`.
- **Saída:** `--saida <png>` ou `--saida-dir <pasta>` (padrão `./inspecao`,
  ignorada pelo git).
- **Desenho:**
  - `--regiao <id>`;
  - `--vista frente|costas|direita|esquerda|cima|baixo|34|34e|34c`, ou
    `--yaw/--pitch` em graus (0 = de frente; +90 = lado esquerdo);
  - `--zoom`, `--orto`;
  - `--alvo x,y,z --raio m`: enquadra um ponto do modelo num raio (um detalhe
    de perto, como se audita o modelo);
  - `--modo ossos|solido|solido_arame|arame|raiox|normais|partes|pesos|argila`,
    com `--junta J` para o modo `pesos`;
  - `--rotulos`, `--isolar`, `--largura/--altura`, `--ssaa 1..4`;
  - `--sem-grade --sem-hud --sem-ossos --sem-avesso`;
  - `--luz-frontal`: a luz principal de frente, como nos renders das
    referências (para comparar formas sem a sombra lateral do estúdio);
  - `--tempo` (só no `render`): quanto o desenho levou.

Exemplos:

```bash
B=build-profile/MatterModelInspector
$B render --regiao mao_direita --rotulos --saida mao.png
$B folha --vistas frente,direita,costas,34 --modo solido_arame
$B analisar --imagens 6 --saida-dir inspecao/analise
$B comparar                       # frente, costas e lado das referências
$B corte --junta LeftThigh --t 0.5
$B poses --poses repouso,sprint@0.3 --modo raiox
$B poses --poses dobra:RightShin=45,dobra:RightShin=90,dobra:RightShin=140 --vista direita
$B render --alvo 0.0,-0.09,-0.44 --raio 0.08 --yaw -60 --modo solido_arame   # o joelho de perto
$B analisar --poses natural_idle@0.5,sprint@0.25,dobra:RightUpperArm=30x,dobra:Abdomen=30
$B analisar --sem-intersecoes --poses "ligacao#punho,ligacao#aberta,dobra:RightHand=70x"   # a mão
$B render --alvo 0.07,-0.034,0.81 --raio 0.03 --orto --luz-frontal   # o olho direito de frente
$B analisar --poses "dobra:Neck=25+Head=25,dobra:Neck=30x,dobra:Neck=45z"   # a cabeça
$B render --regiao mao_esquerda --vista baixo --pose "dobra:LeftThumb1=32d+LeftThumb2=64d+LeftThumb3=80d#aberta"   # o polegar
```

### Leitura das imagens

- **Cores dos ossos:**
  - direito laranja, esquerdo azul, centro branco;
  - a região em foco destacada (juntas amarelas) e as outras apagadas.
- **Ossos dentro da malha:** aparecem fracos ("fantasma"). No raio-x, por cima.
- **Avesso em magenta:** um triângulo visto por trás, ou seja, normal
  invertida numa peça fechada.
- **Achados:** erro em vermelho, aviso em amarelo e observação em azul.
- **Régua:** a escala no alvo da câmera. O gizmo mostra os eixos (X frente,
  Y esquerda, Z cima).

## O que a análise checa (`analisar`, `ModelValidator`)

- **Conexão** (`ConnectivityCheck`, a regra de que nada fica solto):
  - **peça solta:** um pedaço ligado (por arestas) que não chega a nenhuma
    parte raiz (`MeshPart::root`) é erro;
  - **costura faltando:** partes declaradas ligadas (`MeshPart::connectedTo`)
    sem nenhuma aresta em comum são erro;
  - **borda aberta:** toda volta de borda além das aberturas declaradas da
    parte (`MeshPart::openings`, ex.: `cintura`) é erro; as declaradas
    aparecem como observação;
  - **costura sem solda:** vértices no mesmo lugar que não são o mesmo
    (entre quaisquer partes) são erro;
  - **vértice pinçado:** a superfície se tocando só por um vértice (aviso).
- **Topologia:**
  - arestas não-manifold e vizinhos com a frente trocada;
  - triângulos degenerados;
  - agulhas: a que forma uma faceta própria é aviso; a que fica no plano de
    uma vizinha (o pedaço de uma face cortada) é só observação;
  - peças fechadas com a normal para dentro.
- **Interseções:** BVH com teste triângulo × triângulo. Na ligação é **erro**
  (as peças se ligam por costura, nunca por sobreposição); numa pose é aviso.
  Dois triângulos quase no mesmo plano (até ~1°) só se cruzam se um cobrir o
  outro visto de frente: vizinhos lado a lado, a uma fresta, não contam (a
  reta de interseção de planos assim é só ruído).
- **Simetria:** cada vértice espelhado (y → −y) contra a superfície,
  tolerância de 3 mm. Um decalque no plano da superfície (o número do calção)
  não acusa.
- **Skin:**
  - soma diferente de 1, negativos, sem peso, junta inexistente, peso em ponta;
  - peso dominante de uma região não vizinha;
  - vértice longe do osso.
- **Ossos dentro do corpo:** corte perpendicular em cada junta com malha na
  região. Avisa quando a junta fica fora, e observa quando o osso passa a mais
  de 55% do raio do centro.
- **Deformação** (`--poses`): triângulos esticados (> 1,8×), amassados
  (< 0,35×) ou virados na pose, e partes que passam a se cruzar só na pose.
  - **Virou** compara a normal na pose com a que o skin do triângulo pede: a
    normal da ligação girada pelas juntas dos três vértices, nos pesos deles.
    (Só o osso dominante acusava à toa as juntas 50/50 dobradas a 100°, e
    não via as dobras de verdade, como atrás do joelho na corrida.)
  - **Dobrou** pega o avesso que a normal esperada não vê: a mistura dos
    giros vira a normal esperada junto (a barra da camisa nas nádegas com
    pelve e coxa, a pele na boca da chuteira). O triângulo com a normal
    contra a de um vizinho de aresta é testado do mesmo jeito; é erro só se
    o verso dele não aparecia já na ligação (uma borda aberta, numa malha
    aberta).
  - **Virado antes de amassado:** o lado de dentro de uma junta que fecha
    amassa e vira junto, e o que aparece é o avesso. (Antes o amassado
    tapava o virado, e o punho, o polegar e a camisa passavam limpos com o
    avesso à vista.)
  - Só é **erro** (`virou_pose`) se der para ver o verso: de 24 raios saindo
    do verso num cone em volta da normal contrária à da pose, 2 ou mais
    escapam da malha. De onde se vê a frente do triângulo não é avesso. Os
    raios partem rente ao triângulo (0,01 mm): a outra pele de um vinco
    encosta a centésimos de milímetro e tampa a lasca virada. Partindo de
    meio milímetro, o raio passava dela e acusava lascas que nenhum render
    mostra, como a dobra de dentro do cotovelo.
  - Os que ficam fechados dentro de uma prega (atrás do joelho, na axila, na
    dobra do cotovelo, entre os dedos) são observação (`prega_pose`).

## Kit de modelagem (`MeshBuilder`)

A malha é construída com `src/Engine/Modeling/Mesh/MeshBuilder.hpp`:

- **Parte e material:** `part(nome, região)`, `material(nome, cor)`. Toda
  geometria vai para a parte e o material correntes.
- **Ligações declaradas:** `connectTo(outra)` (as duas formam uma superfície:
  a análise cobra a costura), `opening(nome)` (uma borda aberta de
  propósito), `root()` (o corpo ao qual tudo precisa chegar).
- **Elementos:** `vertex`, `triangle`, `quad` (corta na diagonal mais curta),
  `polygon`.
- **Anéis e loft:**
  - `ring` recebe os pontos do perfil no plano (y, z) de um referencial;
  - `ellipseRing`, `bridge` e `loft(anéis, tampa inicial, tampa final)`;
  - tampas: `Open`, `Flat` ou `Point`.
  - Os anéis vão em sentido anti-horário vistos de +x do referencial; assim
    as normais saem para fora.
- **Primitivas:** `box`, `ellipsoid`, `tube`.
- **Edição:**
  - `mirrorParts` cria o outro lado, trocando esquerda/direita nos nomes e
    nas juntas do skin. Vértices divididos entre partes continuam divididos;
    os do plano y = 0 são os mesmos dos dois lados (a costura do meio); uma
    parte do centro (sem lado no nome) recebe a própria metade espelhada;
  - `weld`, `transformPart`, `flipPart`, `removePart`, `compact`;
  - `subdivideTriangles` (`Mesh/MeshSubdivision`): cada triângulo vira
    quatro pelos meios das arestas, sem mudar a forma. Os pesos vão na média
    das pontas e as bordas dos cortes ganham os meios. Pode ser só numa
    região (o filtro por triângulo): o vizinho com duas arestas partidas vira
    quatro, o com uma vira dois, sem rachadura. Do segundo nível em diante
    só viram quatro os filhos de um que virou quatro com os vizinhos também
    assim; um da borda que ficaria com duas arestas partidas vira três (o
    canto entre elas e o resto pela diagonal mais curta). A metade de uma
    metade nunca parte de novo: viraria uma agulha comprida (uma de 11 cm na
    cintura do calção virava do avesso em 436 das 2112 poses).
- **Skin:** `computeAutoSkin` (`MeshSkinning.hpp`) pondera pela distância aos
  ossos da região da parte, mais os vizinhos, e suaviza pela malha.

### Conexão das peças (a regra de que nada fica solto)

Toda peça nova sai da borda de outra, vértice com vértice. As ferramentas:

| Módulo | O que faz |
| --- | --- |
| `MeshTopology` | quem usa cada aresta, as voltas de borda (no sentido da superfície), os pedaços ligados, os vértices pinçados |
| `MeshStitch` | `stitchLoops` (a faixa entre duas voltas de qualquer tamanho, pela programação dinâmica das diagonais), `stitchStrips` (duas linhas abertas, com a ordem em leque se preciso), `stitchCorresponding` (a volta recortada trecho a trecho no anel de cima), `fillLoop` |
| `MeshImprint` | `imprintLoops`: corta voltas desenhadas sobre a superfície nos triângulos de baixo (o arranjo das arestas, Delaunay restrito com todas as arestas antigas obrigatórias). A superfície continua exatamente a mesma; as voltas viram furos (a peça continua dali) ou regiões (outro material) |
| `MeshRelief` | peças em relevo: a pegada recortada como furo e a peça subindo dela (paredes e topo), sem fresta nem sobreposição |
| `Triangulation2D` | Delaunay incremental com arestas obrigatórias e domínio com furos |

Os tipos de ligação usados no corpo:

- **Costura de anéis:** a peça seguinte começa no anel da anterior (a perna
  na gola da chuteira, o calção na barra da coxa, o quadril no assento das
  pernas, a dobra da barra da camisa no alto do calção, o braço na abertura
  do punho, o pescoço na volta de dentro da gola).
- **Borda compartilhada:** a peça seguinte usa os próprios vértices da borda
  da anterior como o primeiro anel (a manga sai da cava dos painéis; a faixa
  da gola sai da borda do V e da volta de trás).
- **Furo recortado:** a peça sai de um furo na superfície (as travas da sola,
  as pontas do cadarço nos ilhoses).
- **Relevo:** a peça deitada é uma elevação da própria superfície (a
  almofada da língua).
- **Região:** só o material muda, dentro de uma volta desenhada (os ilhoses,
  os números do calção e da camisa).

O modelo é descrito por receitas JSON (os números do desenho) lidas por
geradores C++ (a construção), em `src/Engine/Modeling/Parts/`.

## A montagem do jogador (`montagem.json`, `Assembly/`)

`assemblePlayer` monta a metade direita e espelha (simetria exata; a costura
do meio é a mesma dos dois lados). As etapas (`PlayerAssembly`):

1. **O corpo** (`GltfImport`): o GLB no espaço do modelo (x para a frente, z
   para cima), escalado até `bodyHeight` (1,80 m) com a sola no chão do
   esqueleto; fica a metade direita (`MeshEdit::keepRightHalf`).
2. **O pescoço** (`MeshCut`, `NeckJoin`): o corpo e a cabeça são cortados no
   mesmo plano (`neckHeight`). A cabeça (soldada a 5×10⁻⁴ do tamanho: a
   fresta de 0,14 mm da nuca fecha), escalada (`headScale`) com o alto em
   `headTop`, vai para cima do pescoço do corpo; a borda de cima toma a forma
   da de baixo (`neckBlend`) e as duas fecham como um zíper (vértices a menos
   de `neckWeld` viram um; as arestas de cada lado ganham os vértices do
   outro).
3. **O queixo, a mandíbula e o pescoço** (`NeckShortening`, `MeshSculpt`,
   `ReferenceFit`):
   - `headShift` (−19 mm): a cabeça recuada sobre o pescoço do corpo. A junta
     `Head` e o alto da cabeça (`HeadTop`) andam junto (`PlayerModelAsset`),
     então a cabeça continua girando no lugar certo. A medida veio da
     referência alinhada: o pescoço dela ficava 19 mm à frente do nosso, na
     garganta e na nuca (+19,3 e +18,8 mm entre 1,52 e 1,56 m). Era isso que
     fazia a curva de "dois pescoços" na emenda. A linha de cima do pescoço
     e a linha da mandíbula também recuam (a orelha pintada é medida na
     malha, já no lugar);
   - `neck` (`shorten`, `from`, `topFront`, `topBack`): o pescoço fica mais
     curto sem mudar a altura. A cabeça fica onde está e o corpo sobe por
     baixo dela:
     - abaixo de `from` (a base do pescoço), tudo estica por igual até subir
       `shorten` (1,8 cm) ali;
     - entre `from` e a linha de cima do pescoço (sob a mandíbula, mais alta
       atrás), o pescoço encolhe;
     - acima dessa linha, nada muda.

     A altura nunca dobra sobre si. O esqueleto medido e todas as alturas do
     corpo na receita (barras do uniforme, gola, números, meião) passam pela
     mesma conta (`liftedHeight`). Com a cabeça alinhada, o pescoço da
     referência tem o mesmo comprimento que o nosso;
   - `faceShape`: a parte de cima do rosto, com ajustes leves de poucos
     milímetros (as maçãs, a ponte e a ponta do nariz, a testa mais reta, o
     crânio um pouco mais comprido atrás). São vértices levados a lugares
     exatos (`moveVertices`, `[x, y, altura]` finais da metade direita: o mais
     perto de `at` vai para `to`), sem vértice novo. Um `at` sem vértice a
     2 mm é erro (a malha mudou);
   - `referenceFit`: o queixo, a mandíbula e o pescoço no molde da cabeça de
     referência (`base/molde_cabeca.glb`, "Zbrush head base", 8 464
     vértices). São medidos dela, sem cópia a olho e sem vértice novo
     (`Assembly/ReferenceFit`, `Mesh/SurfaceFit`):
     - **alinhamento:** os vértices da cabeça acima da boca (`alignAbove`
       1,635 m), sem as orelhas (`alignMaxSide`), puxam a referência para a
       nossa cabeça. É uma semelhança (escala, giro em y e translação),
       achada por mínimos quadrados de Horn repetidos por ICP, descartando
       os 15% piores pares. O erro médio fica em 1,9 mm;
     - **projeção:** abaixo da linha da mandíbula (`jawLine`, `[x, altura]`
       da frente para trás), cada vértice vai para a superfície da referência
       ao longo da própria normal (`maxDistance` 5 cm);
     - **transição:** logo abaixo da linha, a mudança cresce em `topBlend`
       (1,5 cm). No pescoço ela sobe em linha reta: zero 1 cm acima da gola
       (`collarClearance`, a pele em que a gola encosta) até inteira a 70%
       do caminho até a linha de cima do pescoço (`neckRamp` 0,7). Com a
       rampa a 0,5, a torção extrema (95°) virava um triângulo;
     - **resultado:** 25 vértices da metade direita ficam exatamente na
       referência (0,00 mm) e 16 na transição. Os mais longe (1 a 1,2 cm)
       são as laterais da base do pescoço, onde o nosso abre para o
       trapézio e a gola é feita dessa pele. O comando `molde` mede isso.
   - Tentativas reprovadas pelo usuário, que levaram a esse caminho:
     - os ajustes à mão (`headSculpt`, puxões; vértices do queixo movidos um
       a um), feios;
     - um rosto "chad" exagerado;
     - um queixo arredondado com diagonais trocadas.

     Ele pediu uma ferramenta que deixe o queixo, a mandíbula e o pescoço
     iguais à referência.
4. **A chuteira**: a da receita (`partes/chuteira.json`) nas juntas do pé, sem
   o cadarço em malha, na escala `bootScale` sobre o calcanhar; a perna é
   cortada `bootCollarGap` acima da gola e desce costurada até ela (a meia
   entrando na chuteira).
5. **Os pesos** (`BoneHeatSkin`): o corpo e a cabeça pelo calor dos ossos
   (cada vértice no osso mais perto que ele enxerga por dentro da malha;
   ossos a até 3% da mesma distância dividem; os pesos se espalham pela
   superfície). Os vértices já pesados de outra parte (a borda da chuteira)
   ficam como estavam e seguram o calor. Só as juntas do jogo e os dedos
   pesam (nenhuma auxiliar). Mais estas regras:
   - um osso que continua outro (a junta tem pai) não pega o que fica atrás
     do começo dele além de 2% do comprimento (`behindStart`). Por isso a
     cintura em cima da junta da coxa fica com a pelve; antes, o empate com o
     começo da coxa esticava o calção junto com a perna;
   - os pinos (`pin`), na cabeça: acima da linha de cima do pescoço mais
     `headPinAbove` (1,2 cm), o rosto pesa só na cabeça e não deforma quando
     o pescoço gira ou inclina;
   - os pinos na gola: a faixa logo abaixo dela (`collarPinDepth` 4 cm,
     `collarPinWidth` 7,5 cm do meio) pesa só no alto do peito, e o pescoço
     inclinado não solta a gola;
   - a faixa do pescoço (`NeckBand`, ligada por `neckGradient`): vai da base
     do pescoço até a linha da cabeça (a dos pinos do rosto). A base é a
     gola, mas na frente nunca abaixo de `neckFrontBase` (1,50 m): a pele do
     V entre a gola e essa base é peito e fica presa no alto do peito. Ao
     longo da faixa os pesos passam em linha do alto do peito ao pescoço
     (até o meio dela) e dele à cabeça. A malha base tem só umas três
     fileiras no pescoço; pelo calor, o giro caía quase inteiro na fileira da
     gola e o pescoço torcia como corda (girando 50°, um talo fino; com a
     cabeça junto, 95°, uma corda). Anéis a mais no pescoço (`splitNear`)
     foram testados: com o calor o fundo do V virava, e com o gradiente não
     somavam nada.

   - os dedos e o polegar (`fingerWeightSmoothing`, 2 voltas): os pesos
     espalhados só nos vértices com peso neles (a palma em volta segura os
     dela). A malha do dedo tem um triângulo comprido por falange do lado de
     dentro; pelo calor, ele ia de um nó quase inteiro no osso de cima ao
     outro no de baixo e, fechando a mão, amassava e virava. Eram lascas em
     magenta no punho do jogo, no sprint com punho e no polegar contraído
     (32/64/80°). Com 2 voltas nada vira do avesso em nenhuma partida
     (reta, solta, aberta, punho), com o polegar sozinho ou com a mão
     fechando até 107% do punho. Sobra só o polegar já fechado pelo punho e
     dobrado mais 16–24° por cima: aí ele entra nos dedos, pose que não
     existe. Mais voltas não dão folga além do punho (120% vira com 2, 3 ou
     4) e esticam o polegar.

   Anéis a mais no cotovelo, no joelho e no pulso, com os pesos passando
   pela auxiliar da junta, foram feitos e desfeitos: o usuário achou as
   dobras feias. As juntas ficam com a malha do corpo base.
6. **O uniforme** (`Uniform`, `Garment`): abaixo.
7. **O avesso que sobra** (`weightRelax`, `relaxWeights`): em volta de pontos
   medidos (na metade direita, com o raio e, opcional, a parte), os pesos
   espalham `iterations` voltas. Com a parte, só os vértices que são só dela:
   a costura com outra parte fica, e a chuteira, rígida, não pega peso da
   canela. Os pontos saem do `MatterModelInspector avesso`, que varre as
   animações do jogo e imprime o que colar. Hoje são 5 de 2 cm, 2 voltas:
   - três na axila da camisa (lascas no jog, na corrida de costas, no
     `idle_to_sprint` e num levantar);
   - um na pele da boca da chuteira (o agachamento fundo abria a meia);
   - um no gancho do calção (o uniforme justo).

   Com eles, `shirtBackThighWeight`, o quadril (`hipCrease`, `hipLift`) e a
   gola no peito (abaixo), as 528 poses (264 com mão solta, 264 com punho)
   ficam sem nenhum avesso à vista. Mais voltas na axila inteira
   (`armpitWeightSmoothing`) pioravam: 24 a 33 triângulos. Os pontos só
   servem para lascas de peso: numa dobra de forma (a barra do calcão, a
   gola) os pontos sugeridos pioraram (7 para 14 triângulos).

   Com `--instantes 24` (2112 poses) sobram 2 lascas pequenas na axila
   esquerda do `stand_up_front_crouch` (2,62 e 3,42 s), entre os instantes
   do teste; a de 3,42 s já existia antes da coluna nova.
8. **O espelho**: perto do meio (`midlineBlend`, 3,5 cm) os pesos vão para a
   média com o espelho (o gancho segue as duas coxas); `mirrorParts` e
   `compact`.
9. **A textura** (`UvAtlas`, `TextureBake`, `PlayerPaint`, `FacePaint`):
   abaixo.

Os olhos modelados (`EyeSocket`, `modeledEyes`) ficam desligados: a cabeça
fica como veio e os olhos são pintados.

### O uniforme

- **As barras** (`UniformLines`, as mesmas para o corte e para a pintura):
  - a da camisa (`shirtHem`) e a manga (`sleeveAt` do ombro ao cotovelo,
    perpendicular ao braço);
  - a gola em volta do pescoço: o fundo do V em `collarDepth`, o lado do
    pescoço em `collarSide` a `collarOpening` graus da frente e as costas em
    `collarBack`;
  - o alto do calção (`shortsTop`) e a barra da perna (`shortsHem`,
    perpendicular à coxa).
- **As peças** são cópias da pele cortadas nas barras (`keepRegion`: cada
  corte segue a curva inteira do plano - ou da superfície da gola - que passa
  perto do centro dele; um trecho aberto tem que terminar na borda do meio).
  A pele coberta sai (`removeCovered`), cortada `recess` (2 cm) para dentro
  de cada barra.
- **O pano** (`inflateShell`, `ClothShape`):
  - Taubin arredonda os vincos fundos (`clothRelax` voltas). Cada vértice
    entra numa parte delas (`relaxWeight`):
    - do ombro para cima a camisa fica com a forma do corpo (o deltoide e o
      trapézio por baixo do pano; `shoulderRelax` 0,3);
    - a axila, abaixo, e a nuca (atrás da gola, perto do meio) arredondam
      inteiras;
  - a folga sobe em passos; cada passo alisa só para fora e empurra para fora
    do corpo (o pano passa por cima da axila, da virilha e do glúteo);
  - a queda (`ClothHang`): abaixo de uma saliência, o pano não volta para
    dentro mais que `clothHang` m por metro na camisa (0,6: ela segue a
    lombar) e `shortsHang` no calção (0,35: ele cai do glúteo);
  - do lado de dentro (para o meio), a folga é no máximo metade do espaço até
    ele;
  - no fim, o que ainda se cruza alisa até soltar.
- **As folgas** (09/10/2026, o uniforme justo, medido com `uniforme`): a
  camisa a 6 mm (a barra 14), a manga a 8 mm, a axila 14 (`armpitLoose`: o
  lado de dentro da manga perto do ombro e o flanco embaixo dele, para o pano
  não dobrar quando o braço sobe), a gola 10, o calção 14 e o alto dele 2
  (fica dentro da camisa). A folga média medida caiu de 12,4 para 7,5 mm no
  ombro, de 21,5 para 12,2 mm na lombar, de 14,1 para 10,2 mm na manga e de
  24,9 para 12,9 mm na barra do calção. A largura de deltoide a deltoide é
  +1,3 cm sobre o corpo (antes +2,2 cm). Os limites de cada folga saíram das
  checagens:
  - o `avesso` nas 528 poses do jogo;
  - o cruzamento camisa × calção nas dobras do `MatterPlayerModelTests`;
  - o V da gola com o pescoço girando.
- **A triangulação do pano** (`MeshRepair`): as arestas curtas demais para a
  vizinhança juntam (`clothShortEdge`), a faixa colada na costura do gancho
  vai para ela (`crotchBand`) e as diagonais trocam por triângulos melhores
  (`improveTriangles`). Sem lascas, que o skin vira do avesso.
- **Os pesos do pano**:
  - os da pele;
  - na axila, espalhados (`armpitWeightSmoothing`);
  - **o quadril** (10/10/2026; `hipCrease*`, `hipLift*`, aprovado na
    referência do usuário ERRADO × CORRETO: a perna do calção reta e
    rígida, a listra num arco curto e limpo em volta do quadril, o glúteo
    redondo e compacto, a dobra da frente fechada). A passagem da pelve para
    a coxa é uma volta em torno da coxa; o centro e a largura mudam em volta
    dela (a mistura das quatro direções, cada uma pela parte positiva ao
    quadrado). Fora da passagem, só a pelve ou só a coxa: a perna do calção
    anda inteira com a coxa (sem banana, sem arco).
    - **Na frente:** o centro segue a linha da virilha que o usuário
      desenhou de frente, do lado de fora do quadril (`hipCreaseOuter`
      [-0,149, 0,933]: y e altura do chão) ao gancho (`hipCreaseInner`
      [0, 0,822]); estreita (`hipCreaseFront` 3,5 cm). A coxa levantada
      entra na barriga num vinco fechado. Larga, o canto virava um arco de
      ~10 cm em volta da junta (o "C" que o usuário reprovou). Perto do meio
      (3 a 8 cm) a frente passa para a de dentro.
    - **Do lado de fora:** na altura da junta, 14 cm (`hipCreaseSide`). A
      listra faz o arco curto em volta dela. Com 12 cm sobrava um degrau no
      canto de dentro da listra; com 15, um triângulo virado no levantar.
    - **Atrás:** na dobra do glúteo, 9 cm abaixo da junta
      (`hipCreaseBackCenter`), estreita (`hipCreaseBack` 7 cm) e direta da
      pelve para a coxa, sem os anéis. O glúteo fica redondo na pelve e o
      pano estica reto dele até a coxa, como pano esticado. Girando em volta
      da junta (os anéis, largos), o volume do glúteo descia junto e virava
      um saco 5 cm abaixo da perna levantada.
    - **Do lado de dentro:** na altura do gancho, 12 cm (`hipCreaseInside`).
    - **Fora de trás, os anéis:** pelve → 1/4 → 1/2 → 3/4 → coxa
      (`chainWeights`). Cada anel gira inteiro em volta da junta, sem a
      média dos pesos puxar o pano para dentro.
    - **A dobra da frente** (`hipLiftRise` [0,02, 0,2], `hipLiftFront`
      [0,3, 0,8], `hipLiftMidline` [0,02, 0,08]). A perna rígida levantada a
      90° sobe ~8 cm acima da junta, e a barra da camisa fica a 3,8 cm: a
      perna atravessava a barra (58 pares; o forro de dentro da barra
      aparecia em magenta). Acima da linha, a cintura do calção e a barra da
      camisa passam parte do peso da pelve para a `HipLift` (45% só da
      flexão): a coxa que sobe dobra o pano para trás e a barra fica em cima
      da perna. Os dois têm o mesmo peso no mesmo lugar, então andam juntos.
      A parte vale inteira até 2 cm acima da barra, some até 20 cm; em volta
      da coxa vale na frente (cosseno com a frente de 0,3 a 0,8); some perto
      do meio (2 a 8 cm). Com a fração do giro inteiro (sem o eixo), a coxa
      abrindo de lado no trote lateral virava a barra. Com menos de ~43% a
      perna a 90° atravessa a barra; com mais (0,55-0,7) a camisa virava no
      levantar.
    - **Nos vértices finais do pano** (depois de inflar, cair e juntar as
      arestas): o peso é o do lugar onde o vértice ficou.
    - **`shirtBackThighWeight` 0:** atrás a camisa fica toda na pelve, como
      o glúteo por baixo dela.

    O caminho até aqui, medido a cada passo (todos reprovados ou
    desfeitos):
    - **Faixa longa em volta toda** (09/10): o calção encurvava ao longo da
      coxa ("banana").
    - **Os pesos da pele na frente:** a dobra afundava num "C" e amassava.
    - **Um osso que empurra a frente da dobra para fora:** deformava o pano
      para fabricar uma dobra e cruzava a camisa. Saiu do código.
    - **Dobradiça curta em volta toda:** virava o gancho e a barra de trás
      da camisa.
    - **Anéis de arestas imprimidos na volta** (`splitNear`): lascas no
      gancho (78 avessos).
    - **A junta do quadril mais alta** (o "osso muito para baixo"): a junta
      medida fica 9 cm acima do gancho, dentro do anatômico; 3 cm mais alta
      não mudava a curva.
  - **o calção subdividido** (`shortsSubdivision` 2, `subdivideTriangles`
    em `Mesh/MeshSubdivision`). Com 4 ou 5 anéis da cintura à barra, a dobra
    virava uma quina de polígonos grandes.
    - **No pano pronto** (depois de inflar e juntar as arestas), a forma
      parada é a mesma. Subdividido logo depois do corte, o pano denso
      inflado cruzava ele mesmo no gancho.
    - **Ficam com a malha base** o gancho (`shortsCoarseCrotch`: a menos de
      6 cm do meio, abaixo da junta) e os 6 cm de cima (`shortsCoarseTop`),
      por baixo da barra da camisa. Denso, o gancho virava no agachamento
      fundo; o alto denso atravessava a barra grossa da camisa.
    - **A camisa fica grossa:** subdividida, a axila desandava.

    Resultado: 5816 triângulos, 0 erros e 0 avisos na ligação. No avesso,
    nada nas 528 poses do jogo e, nas 2112, só as 2 lascas antigas da axila
    da camisa. Nas dobras do teste, nada se cruza nem vira. Nas poses de
    quadril, só 2 pares camisa × calção por dentro da barra no
    `tackle_slide` (sem aparecer).
  - a costura do gancho presa em parte na pelve (`crotchPin` 0,3,
    `crotchPinRadius`) e a faixa colada nela indo para a costura
    (`crotchBand` 1,8 cm). Os dois valores saíram do `avesso` com a faixa
    do quadril: com 0,4 e 2,2 cm sobrava uma lasca no gancho no agachamento
    fundo do levantar;
  - **a gola no peito** (`shirtNeckWeight` 0,5): metade do peso do pescoço
    na camisa vai para o alto do peito; o pescoço mexe por dentro da gola.
    O corte da gola herdava 27-52% de pescoço da pele de cima. Com o pescoço
    reto (a coluna nova), deitado de bruços ele estendia, descia a gola de
    trás sobre a pele de dentro, e a borda virava do avesso;
  - atrás do quadril, o peso das coxas na camisa vai para a pelve
    (`shirtBackThighWeight` 0: nenhum fica; a passagem em ±3 cm em volta da
    junta do quadril, `moveWeights`). Assim a barra de trás pende do tronco,
    como o glúteo por baixo dela: na largada agachada (`crouched_to_sprint`)
    a coxa dobrada virava o pano das nádegas do avesso. Na frente, a barra
    segue a `HipLift` (a dobra da frente, acima).
- **O recuo** (`stitchRecess`): em cada barra, uma faixa liga a borda da peça
  (inflada) à borda da pele por baixo dela. Olhando por dentro da manga ou
  da perna do calção, o pano continua para dentro, sem corte seco. A barra da
  camisa costura no alto do calção.

### A textura

- **O mapa** (`unwrapAtlas`): ilhas de triângulos vizinhos da mesma parte com
  a normal até 60° da média, projetadas no plano delas (a mesma densidade em
  toda parte) e giradas para a menor caixa. O arranjo é pelo formato
  (`Texture/IslandPacking`), não pela caixa: um mapa de ocupação em células
  de 4 texels; cada ilha, da maior para a menor, vai para a primeira posição
  livre, deitada ou em pé (a que termina mais acima), e uma ilha pequena cabe
  no vão de uma grande. Entre os triângulos de duas ilhas ficam pelo menos
  8 texels (2 × `padding`). A escala é a maior que cabe (busca binária, ~0,3 s).
  Com isso o atlas (`atlasSize`, 2048) fica 77% cheio; nas prateleiras de
  antes, 52%.
- **As densidades** (fatores sobre a base, na receita): ×2 na cabeça
  (`faceDensity`), ×1,5 na camisa e no calção (`garmentDensity`) e ×2 no
  cabedal e na língua da chuteira (`laceDensity`: a tira do cadarço tem
  9,5 mm). Medido na malha montada (texels por metro de superfície):

  | Parte | Antes (prateleiras; roupa ×1, chuteira ×3) | Agora |
  | --- | --- | --- |
  | camisa, calção | 720 | 1 160 |
  | cabeça | 1 430 | 1 530 |
  | cabedal da chuteira | 2 200 | 1 580 |
  | corpo (pele, meião) | 730 | 780 |

  O mapa (`Mesh::triangleUv`, um por canto) entra por último; compactar,
  espelhar e tirar parte o mantêm.
- **A pintura** (`bakeTexture`): uma função do ponto 3D na ligação, 3×3
  amostras por texel e 4 texels de sangria. Em paralelo (faixas de 8 linhas:
  cada texel é de uma thread só) e adaptativa: primeiro uma amostra no
  centro de cada texel; as 3×3 só onde a cor muda para um vizinho (a borda
  de um desenho), na beira de uma ilha e nas partes de traço mais fino que
  um texel (`fineParts`: o rosto e o cadarço). O resultado é o mesmo do bake
  completo (0/255 de diferença no jogador) em 0,3 s; o completo numa thread
  levava 6,3 s. A borda de um desenho tem
  `paintEdge` texels (1,5; `TextureBakeSettings::edgeTexels`): com a borda de
  um texel, uma borda inclinada no atlas virava uma escadinha de dentes de um
  texel quando a textura é ampliada (o filtro bilinear mostra os degraus);
  com 1,5 a borda continua firme (~1,3 mm na roupa) e sem dentes. Todos os
  materiais usam o atlas (`MeshMaterial::texture`); o renderizador o amostra
  sob a mesma luz (`Rasterizer::triangleTextured`).
- **O que é pintado** (`PlayerPaint`, nesta ordem): a cor de base de cada
  parte (a do desenho ou a do material), o rosto (`FacePaint`, o bloco `face`
  da receita), a chuteira e o cadarço (no código) e as camadas do desenho
  (`pintura.json`, a seção seguinte):
  - o rosto:
    - os olhos: a amêndoa relaxada, com a íris inteira à vista (`eyeUpper`,
      `eyeLower`, `irisRadius`), o canto de fora um pouco mais alto
      (`eyeTilt`), a pálpebra de cima levemente reta (`upperLidCurve`), a
      linha dos cílios e uma sombra leve da dobra (`creaseWidth`,
      `creaseShade`); o centro achado por um raio na cabeça;
    - as sobrancelhas castanho-escuras (`brow`), grossas e retas, um pouco
      mais baixas por dentro; ficam por cima da sombra da dobra;
    - a boca nas arestas da cabeça base: a fresta no vinco entre a faixa de
      cima (o lábio de cima, com o arco do cupido) e a face clara, que não é
      lábio (fica pele) - `mouthSlit`, `upperLipTop`: os vértices da malha,
      do meio ao canto (com a boca um pouco mais larga, o canto em 2,86 cm);
  - a chuteira mais escura (`bootShade`), com o cadarço desenhado no lugar do
    da malha (as tiras, o cruzado, a borda escura, a sombra no cabedal e as
    fendas dos ilhoses, nas posições de `BootLaces`).
- **Ver o atlas:** `MatterModelInspector textura --ilhas --fundo`.

### A pintura em dados (`pintura.json`, `Paint/`)

O uniforme, a orelha e qualquer detalhe novo são **dados**: camadas de
formas vetoriais desenhadas em espaços medidos na malha (os moldes das
peças). Nenhum campo de código por detalhe: um uniforme novo é um arquivo
novo. A receita aponta o desenho em `paintFile`.

Os módulos (`src/Engine/Modeling/Paint/`, só o núcleo):

| Módulo | O que faz |
| --- | --- |
| `Shape2D` | formas em contornos (polígono, retângulo com canto chanfrado ou redondo, elipse, linha grossa); cheias pela regra das voltas (furos no sentido contrário); `PreparedShape` guarda só a borda de verdade da união e acha a distância exata até ela: a cobertura de cada pixel sai dela (nítida e lisa em qualquer resolução, sem depender dos vértices da malha) |
| `PaintFont` | texto: a fonte de bloco dos números (os algarismos medidos na referência) e qualquer TrueType (stb_truetype: a altura das maiúsculas é 1, com o kerning da fonte) |
| `SvgPath` | o `d` de um `<path>` de SVG (M, L, H, V, C, S, Q, T, A, Z) em contornos (curvas e arcos em segmentos com desvio de um milésimo do desenho) e o arquivo .svg simples (o viewBox e os caminhos pelo id) |
| `PaintImage` | um PNG para a pintura: a transparência pré-multiplicada (sem franja escura na beira) e as reduções pela metade (a amostra lê o nível do tamanho dela) |
| `PaintLayer` | a tinta (cor, degradê linear ou radial, imagem), a opacidade, o jeito de misturar (por cima ou multiplicando), o contorno por fora e os lados |
| `PaintSpace` | os espaços: o plano (a projeção, só as faces viradas para ele) e a volta desenrolada (abaixo) |
| `PaintDesign` | o `pintura.json` lido e conferido (o erro diz a camada) |
| `Assembly/PlayerPaintSpaces` | os espaços do jogador, com as âncoras |
| `Assembly/PlayerPaint` | a função do bake: a base, o rosto, a chuteira e as camadas |

**A volta desenrolada** (`unrolledSpace`, o tronco e a perna do calção): a
peça fatiada a cada 2 mm em planos perpendiculares ao eixo; a coordenada é o
arco na fatia a partir da costura (o lado de fora, ajustado numa reta ao
longo do eixo) e a distância ao longo do eixo. É o molde de costura de
verdade: uma faixa de 7 cm tem 7 cm medidos no pano em qualquer altura, e a
borda dela não depende de onde a malha tem vértices. Os cuidados:

- só o pano de fora (`outerOnly`): a dobra recuada por dentro da barra
  emendaria a volta de fora com a de dentro;
- a costura é seguida de fatia em fatia (o cruzamento mais perto do da
  vizinha, nunca a mais de 3 cm); um pedaço partido pela cava da manga, sem
  costura, fica sem coordenada;
- a amostra usa a fatia que corta o triângulo dela; se só uma das duas em
  volta corta (a ponta do triângulo entre as fatias), só essa. Na barra a
  fatia de baixo se parte longe dali, e o vizinho dela dava o arco da ponta
  do outro pedaço: era a mancha de 3,7 cm na barra de trás;
- entre duas fatias, o ponto mais perto na corrente da fatia (o trecho
  vizinho, quando a amostra passa da ponta de um trecho curtinho perto de um
  vértice);
- numa volta fechada (a perna) o arco vai de meia volta para trás a meia
  volta para a frente, com o corte do lado oposto à costura (a costura de
  dentro da coxa). A fatia aberta numa brecha pequena (o recorte da barra)
  conta como fechada, e a média entre duas fatias vai pelo caminho curto,
  nunca atravessando o corte. Antes, a média de +0,33 com -0,33 caía no zero,
  em cima da faixa preta: pontinhos pretos por dentro da coxa.

**Os espaços do jogador** (metros, alturas do chão; o lado esquerdo é o
espelho do direito):

| Espaço | x | y | Âncoras |
| --- | --- | --- | --- |
| `tronco` | arco a partir da costura do lado (+ para a frente) | altura | `barra`, `axila`, `gola_frente`, `gola_costas`, `inicio`, `fim` |
| `frente` | +y do jogador (a direita de quem olha) | altura | as do tronco |
| `costas` | -y do jogador | altura | as do tronco |
| `manga` | em volta do braço (do alto) | da barra da manga para o ombro | `barra` (0) |
| `gola` | em volta do pescoço (da frente) | abaixo da borda da gola | `borda` (0) |
| `calcao` | arco a partir da costura de fora (+ para a frente) | ao longo da coxa, do quadril para baixo | `barra`, `inicio`, `fim` |
| `calcao_frente` | +y do jogador | altura | `barra` |
| `perna` | em volta da canela (da frente) | altura | `joelho` |
| `orelha` | frações da placa de fora medida na malha: de trás (0) para a frente (1) | de baixo (0) para cima (1) | - |

**O formato:**

```json
{
  "colors": { "branco": [236, 236, 236], "preto": [30, 30, 33], "marinho": [20, 34, 74], "ouro": "#d6ac42" },
  "base": { "camisa": "branco", "calcao": "branco" },
  "layers": [
    { "name": "faixa do lado", "space": "tronco",
      "shape": { "type": "rectangle", "from": [-0.06, "barra-0.06"], "to": [0.04, "axila"] },
      "color": "preto" },
    { "name": "numero", "space": "costas",
      "shape": { "type": "text", "text": "10", "font": "block", "height": 0.165, "at": [0, "gola_costas-0.2003"] },
      "color": "preto", "outline": { "width": 0.004, "color": "ouro" } },
    { "name": "escudo", "space": "frente", "sides": "left",
      "shape": { "type": "path", "file": "escudo.svg", "id": "fundo", "at": [0.085, "gola_frente-0.075"], "height": 0.075 },
      "color": "marinho" },
    { "name": "patrocinio", "space": "frente",
      "image": { "file": "patrocinio.png", "at": [0, "gola_frente-0.235"], "height": 0.045 } }
  ]
}
```

- **Formas:** `polygon` (points), `rectangle` (from, to, chamfer, round),
  `ellipse` (center, radius), `line` (points, width, closed), `text` (text;
  font `block` ou um .ttf ao lado do desenho; height, at, align
  center/left/right, vertical middle/baseline, spacing, slant em graus,
  stretch) e `path` (um caminho de SVG: `d`, ou `file` e `id`; o quadro, que
  é o `viewBox` dado, o do arquivo ou a caixa do caminho, centrado em `at`
  com a altura `height`). Todas aceitam `rotate` (graus, em volta de `pivot`
  ou do centro).
- **Tinta:** `color` ([r, g, b(, a)] de 0 a 255, `#rrggbb` ou um nome de
  `colors`), `gradient` (linear: from, to; radial: center, radius, angle; e
  `stops` [[0, cor], [1, cor]]) ou `image` (um PNG: `from` e `to`, ou `at` e
  `height` com a largura pela proporção dele, `stretch`, `rotate`; sem
  `shape`, a forma é a caixa da imagem).
- **Mais:** `opacity`, `blend` (`normal` ou `multiply`: a sombra de uma
  cavidade), `outline` {width, color} (por fora, por baixo da tinta) e
  `sides` (`both`, `right`, `left`).
- **Coordenadas:** um número ou uma âncora do espaço com deslocamento
  (`"axila"`, `"barra-0.06"`).

**Um uniforme novo, passo a passo:**

1. `MatterModelInspector pintura --espaco tronco` (e `calcao`, `frente`,
   `manga`...): o molde do espaço num PNG, com a pintura de agora, a grade
   de 5 cm e as âncoras com nome (`--lado esquerdo`, `--passo`, `--escala`).
   É onde se lê a coordenada de cada coisa, sem adivinhar.
2. As camadas no `pintura.json` (uma cópia com outro nome e `paintFile`
   apontando para ela). A ordem é de baixo para cima.
3. Escudo e logos: desenhados num editor vetorial e salvos em SVG simples
   (as transformações aplicadas). Uma camada por cor, todas com o mesmo
   `file`, `at` e `height`: o quadro é o do SVG, então elas ficam alinhadas.
   Uma peça pronta em PNG com transparência entra como `image`.
4. Ver: `MatterModelInspector render`/`folha` (closes com `--alvo` e
   `--raio`), `textura --ilhas --fundo` (o atlas) e de novo `pintura` (os
   moldes).
5. Mais nitidez onde importa: `garmentDensity` (a roupa) e `paintEdge` (a
   borda) na receita.

Um exemplo pronto fica em `exemplos/pintura_exemplo.json`: o uniforme mais um
escudo de três cores em SVG (`exemplos/escudo.svg`) no peito esquerdo e um
patrocínio em PNG (`exemplos/patrocinio.png`). Para ver:
`"paintFile": "exemplos/pintura_exemplo.json"` no `montagem.json`.

**A orelha** é uma camada do desenho: o polígono do buraco (a concha e o
canal) e o degradê, os dois medidos no desenho do usuário. O desenho e a
placa medida na malha (`earPlate`: os triângulos da cabeça virados para fora
além de `earOutside`, a maior peça ligada deles) foram levados um ao outro
pela normalização afim dos momentos de área das duas placas (o centro, os
eixos e as escalas). O contorno do buraco saiu de 64 raios no meio-tom da
beira; o degradê radial (o centro no canal, os raios, 0,64 da pele no fundo e
0,876 na beira) foi ajustado aos tons do desenho por mínimos quadrados (erro
rms 0,029). A beira é um degrau nítido, o buraco fica para a frente e o
escuro na frente, como no desenho; multiplica a pele, dos dois lados.

**Campos que saíram da receita** (viraram camadas do desenho): `trimColor`,
`collarTrim`, `cuffWidth`, `sidePanel`, `shortsStripe`,
`shortsNumberAcross`, `sockTop`, `sockBand`, `sockColor`, `sockBandColor` e
os da orelha (`earShade`, `earCanal`, `earCavity`, `earSoftness`,
`earFunnel`).

### O sombreamento liso (`Mesh/ShadingNormals`)

Só a luz muda: a malha, os vértices, o contorno e a pintura são os mesmos.
Em cada canto de triângulo, a normal é a média das faces em volta do vértice
que se ligam a ele sem atravessar uma quina - a aresta entre duas faces com
mais de `creaseAngle` graus entre as normais (45°: a ponta do nariz, a boca, a
linha do queixo, a barra dobrada para dentro da roupa). A média pesa o
ângulo de cada canto. Os grupos saem da pose de ligação e são os mesmos em
qualquer pose (nenhuma quina acende e apaga na animação); as normais saem
das posições da pose. `facet` mistura com a normal da própria face (0 liso,
1 o facetado de antes; o padrão é 0,5 com quinas de 45°, na inspeção e no
jogo).

- **Na inspeção:** o modo Sólido acende cada pixel pela normal interpolada
  (`Rasterizer::triangleTexturedLit`, `PixelLight`: a mesma luz de estúdio de
  antes, agora num lugar só). A sessão guarda os grupos (refeitos com o modelo
  ou a quina).
- **No jogo:** a pele exportada leva essas normais (lisas) e um vértice da
  GPU por (vértice, grupo, coordenada do atlas): 4 721 com quinas de 45°
  (seriam 12 728 facetado, porque cada canto vira um vértice). O `facet` vai
  no manifesto e o shader mistura a normal lisa com a da face
  (`scene3d_mesh.frag`).
- Liso total (sem quinas) não serve: o rosto perde a definição e a dobra do
  recuo das barras fica com uma mancha escura.

### Para o jogo (`Rig/GameCharacterExport`)

`MatterModelInspector exportar-jogo --destino assets/characters/football_player`
(`--quinas`, `--facetado`) leva o modelo ao jogo: o perfil físico (o rig
calibrado com as âncoras, os links e os colisores medidos no corpo novo, na
pose T dele, com a pelve na origem), a pele versão 2 (a malha na pose A, cada
osso com o repouso e a ligação; os auxiliares; os dados da mão relaxada e do
punho) e o atlas. O relatório mostra cada colisor ao lado do anterior, as
folgas nas animações e a altura de pé. As regras e o formato estão em
`RAGDOLL_CHARACTER_PIPELINE.md`.

- **Ponto fixo:** exportar de novo no mesmo corpo dá os mesmos arquivos; o
  `MatterPlayerModelTests` cobra o personagem do jogo em dia com o modelo
  (mudou o modelo, exporte de novo) e a pele do jogo igual à malha posada da
  modelagem (medido: 0,02 mm no pior vértice).
- **A modelagem não muda com a troca:** o mesmo esqueleto e as mesmas poses
  (os dedos giram no máximo 0,125° na ligação; a malha posada, 0,024 mm).

## Desempenho

Medidas em 09/10/2026 (12 threads; ~10 workers no agendador):

| O quê | Antes | Agora | Como |
| --- | --- | --- | --- |
| Montar o jogador (carga inteira) | 7,1 s | ~0,9 s | o bake em paralelo e adaptativo (6,3 s → 0,3 s), o atlas sem linhas cheias na busca e com menos tentativas (0,29 → 0,12 s), os pesos em paralelo (0,12 → 0,03 s) |
| Entrar na tela de inspeção | travava ~7 s | não trava | a montagem numa thread própria |
| Recarregar a tela | relia o personagem do jogo (0,33 s) | não relê | o personagem guardado por processo, de novo só se a pasta dele mudar |
| Quadro parado, 1400×900 (2× supersampling) | ~60 ms | ~33-45 ms | as faixas paralelas com a grade e a redução em paralelo |
| `MatterModelInspector avesso` (528 poses) | 28 s | 3,7 s | as poses em paralelo |
| `MatterPlayerModelTests` | 58 s | 18 s | a carga e a checagem de deformação em paralelo |

`MatterModelInspector info` mostra o tempo de cada etapa da carga; a tela
avisa o total quando o modelo entra; os testes reclamam se a carga passar de
5 s.

**O modelo na GPU:** 4 656 triângulos e 4 721 vértices (com o sombreamento
liso), contra 76 518 triângulos e 51 841 vértices do jogador antigo do jogo
(o Ch38) - 16 vezes menos triângulos. O atlas é um só, de 2048.

**Para muitos jogadores em campo** (quando o modelo for para o jogo):

- uma malha e um skin para todos com o mesmo corpo (o jogo já deforma na
  GPU e desenha por instância);
- um atlas por uniforme e o número/nome de cada jogador por cima (um
  pedaço do atlas por jogador ou um decalque), em vez de um bake por
  jogador;
- a montagem guardada em disco (cozida no build) em vez de montar ao abrir;
- um nível de detalhe para longe: as chuteiras são 2 600 dos 4 656
  triângulos (as travas e a sola).

## Peças procedurais (`Parts/`)

O corpo anterior, guardado (`model_antigo.json`) para reaproveitar partes; a
chuteira continua em uso na montagem.

As receitas em `assets/characters/player_model/partes/` têm a forma inteira;
o código só interpreta. Cada módulo faz uma coisa:

| Módulo | O que faz |
| --- | --- |
| `ProfileCurve` | curva `[t, valor]` Hermite monótona (sem overshoot): as curvas das receitas |
| `RecipeJson` | a leitura comum das receitas (curvas, valores, cores) |
| `FacetNoise` | o ruído determinístico do facetado low-poly |
| `BootRecipe`, `BootShape` | a chuteira em números e a forma lisa (seção em t, pontas, abertura, contraforte, canal do cadarço, o anel de 15 pontos) |
| `BootSole` | a sola costurada na base do cabedal e as travas saindo de furos nela |
| `BootTongue` | a almofada da língua, em relevo |
| `SurfaceDrape` | tiras deitadas sobre os triângulos de verdade (folga exata); a tira com as pontas entrando na superfície (`drapeWeldedStrap`) |
| `BootLaces` | o cadarço: as diagonais (uma por cima da outra) e as tiras de cima em arco, cordões com as duas pontas entrando no cabedal; os ilhoses desenhados em volta das pontas |
| `BootGenerator` | monta a chuteira e os pesos; devolve a gola para a perna |
| `LegRecipe`, `LegGenerator` | o meião, a faixa, o joelho e a coxa: da gola da chuteira até a barra do calção |
| `ShortsRecipe`, `ShortsGenerator` | o calção (metade direita): a barra na coxa, a perna do calção com a faixa, a virilha, o quadril e o cós (ou, com a camisa, o anel onde ela costura); o número na perna esquerda |
| `SuperSection` | a seção de quatro quadrantes com dois expoentes (frente e costas, lados): tronco, pescoço e o campo da camisa |
| `ShapeField` | a forma implícita de uma peça: pilha de seções, cone arredondado, corrente de esferas e elipsoide unidos com mistura suave; valor (distância aproximada), gradiente, projeção na superfície e o raio que acha a superfície |
| `LimbFrame` | o referencial de um membro (ao longo do osso, para a frente e para cima): a manga e o braço |
| `NumberGlyphs` | os algarismos do uniforme (fonte de bloco com chanfros: o 1 com bandeira e base, o 0 octogonal) e o número montado num plano para o recorte: calção e camisa |
| `ShirtRecipe`, `ShirtGenerator` | a camisa (metade direita): o campo, os anéis do tronco com a faixa, os painéis da frente e de trás, a cava, a manga com o punho, a gola e a dobra da barra no calção; os números no peito e nas costas |
| `ArmRecipe`, `ArmGenerator` | a pele do braço e do antebraço, da saída do punho até o pulso (o olecrano, a dobra do cotovelo, o pulso) |
| `HandRecipe`, `HandGenerator` | a mão: a palma do pulso aos nós (com a janela do polegar), as membranas, os quatro dedos e o polegar, as unhas |
| `NeckRecipe`, `NeckGenerator` | a pele do pescoço da volta de dentro da gola (no V, o alto do peito) até a base da cabeça |
| `HeadRecipe` | a cabeça em números: as seções do crânio, o perfil do meio, o nariz, a boca, o olho, as sobrancelhas, a mandíbula, a orelha |
| `HeadFace` | a superfície da cabeça (a seção mais os campos do rosto) e os traços fixos do rosto (vértices, arestas obrigatórias, furos, regiões de material) |
| `HeadEye`, `HeadBrow`, `HeadEar` | o olho na abertura amendoada, a sobrancelha em bloco e a orelha, cada um subindo do seu furo na pele |
| `HeadGenerator` | a cabeça: o domínio desenrolado, o preenchimento, a triangulação restrita, o alto em anéis, o olho, a sobrancelha, a orelha e os pesos |
| `PartLibrary` | lê as receitas do manifesto e monta a metade direita de baixo para cima (cada peça na borda da anterior), espelha e desenha os números |
| `Mesh/MeshRepair` | `flipFlatNeedles`: troca a diagonal das agulhas chatas (as do recorte dos números) |

**A montagem** (`PartLibrary`): chuteira direita → perna direita (na gola) →
metade direita do calção (na barra; com a camisa, ele para na dobra da barra
dela) → metade direita da camisa (a dobra da barra costurada no alto do
calção) → braço direito (no punho da manga) → mão direita (no pulso) →
metade direita do pescoço (na volta de dentro da gola) → espelho de tudo (o
lado esquerdo e as outras metades, com os vértices do meio soldados) → os
números da camisa e do calção → `compact` → `flipFlatNeedles` (troca a
diagonal das agulhas chatas que o recorte dos números deixa: a do meio fica
sobre a aresta longa, quase no plano da vizinha, e a troca só vale se os
dois triângulos novos ficarem bons e com a mesma frente). A raiz é a peça de
cima do tronco (a camisa; sem ela, o calção, a perna ou a chuteira). A
cabeça entra depois do pescoço, na volta de cima dele. Sem a peça seguinte, a
borda dela fica declarada aberta (`gola`, `barra`, `cintura`, `punho`,
`pulso`, `pescoco`, `cabeca`); com tudo, nada fica aberto.

### A chuteira (`partes/chuteira.json`)

- **Espaço da chuteira:** origem no chão atrás do calcanhar, x para o bico, y
  para dentro do pé, z para cima; `t` vai de 0 (calcanhar) a 1 (bico). O
  calcanhar fica `heelBehindAnkle` atrás de `RightFoot`; o rumo vai até
  `RightToeTip`; a dobra fica em `RightToe`.
- **Cabedal:** loft de anéis nas `stations`, aberto embaixo (a sola costura
  na base: os pontos 0 e 14 de cada anel) e sem o interior da abertura: a
  borda de dentro da gola, do alto da aba do calcanhar ao alto da língua, é a
  volta onde o meião começa. As pontas fecham em "zíper" (calcanhar) ou em
  cone (bico). O facetado é o ruído com `seed`, mais fraco nas pontas.
- **Uma superfície só:** a sola sai da base do cabedal pela borda que aparece;
  as travas saem de furos recortados na face de baixo; a língua é um relevo do
  cabedal; os ilhoses são regiões escuras desenhadas nele; cada tira do
  cadarço é um cordão cujas pontas entram em furos (as de cima no meio da
  fenda do ilhós, as diagonais escondidas debaixo das tiras). A diagonal de
  cima passa por cima da de baixo, e as tiras por cima de tudo, com a folga
  mínima (`laceClearance`).
- **Skin:** tudo no pé; a frente passa para os dedos numa transição de 2,6 cm.
- **Padrões = receita:** `BootRecipe::defaults()` e os inicializadores de
  `BootRecipe.hpp` repetem o JSON. O teste `MatterEngine.ModelingParts` exige
  os dois iguais. Depois de editar o JSON, rode
  `~/Área de trabalho/modelagem/ferramentas_chuteira/sincronizar_padroes.py`.
- **Comparação com as referências:** `~/Área de trabalho/modelagem/ferramentas_chuteira/`
  tem os scripts com câmeras ajustadas a cada painel das referências da
  chuteira. O guia está em `~/Área de trabalho/modelagem/CHUTEIRA_PARA_CODEX.md`.

### A perna (`partes/perna.json`)

- **Seção:** por altura do chão, a distância da superfície até o eixo do
  osso para a frente, para trás, para dentro e para fora (`front`, `back`,
  `medial`, `lateral`) e o expoente da superelipse. As medidas vieram das
  referências de frente e de lado.
- **Saída da gola:** os anéis `collarRings` começam na forma da borda da gola
  (`collarLift` acima dela) e viram a da perna aos poucos.
- **Meião, faixa e pele:** `sockTop` e `bandTop` são anéis (as bordas ficam
  retas). Os anéis alternados ficam desencontrados meio passo e os vértices
  tremem para dentro/fora (`jitter`) e para cima/baixo (`verticalJitter`): o
  facetado irregular da referência.
- **Joelho:** a patela salta na frente (`patella*`), a tuberosidade logo
  abaixo e o oco atrás afunda (`poplitealDepth`), em volta da junta do joelho.
- **Skin:** pé na gola, a auxiliar do tornozelo no meio da subida do meião,
  canela, e no joelho: na frente canela → auxiliar → coxa (a quina dobra a
  45° numa dobra de 90°, sem perder volume; as transições cobrem o arco da
  quina, a pele estica aos poucos); atrás, a dobra entre a coxa e a canela.
- **Joelho muito dobrado** (a perna que volta na corrida): uns 14 triângulos
  da dobra de trás viram, mas ficam fechados dentro da prega: a checagem os
  separa como `prega_pose` (observação), não aparecem.

### O calção (`partes/calcao.json`)

- **Uma camada só** (sem pele por baixo): a parte de baixo da barra é um anel
  da coxa até a borda do calção (`insideColor`, o lado de dentro aparecendo).
- **Perna do calção:** em volta do eixo da coxa (`legFront`...), virando o
  alto da perna perto da virilha.
- **Bifurcação:** o alto de cada perna é o arco de fora (o primeiro anel do
  quadril, `seatHeight`) mais a costura da virilha no plano do meio (do
  assento na frente, descendo até `crotchHeight`, subindo até o assento atrás),
  dividida pelas duas pernas.
- **Quadril e cós:** em volta do centro do corpo (`hipFront`, `hipBack`,
  `hipSide`); o cós salta `waistbandStep` para fora e a borda de cima fica
  aberta (`cintura`).
- **Faixa:** os anéis têm pontos exatamente nas bordas da faixa (x = centro ±
  meia faixa), do cós até a barra.
- **Número:** os dígitos (`number`) desenhados como regiões na frente da perna
  esquerda, depois do espelho (`numberAngle`, `numberAboveHem`).
- **Com a camisa** (`ShortsPlacement::coverHeight`): o quadril para no anel
  da dobra da barra (`tuckHeight` da camisa), sem o cós, e não declara a
  cintura aberta: a camisa costura ali.
- **Skin:** a perna do calção segue a coxa, passando pela auxiliar do quadril
  perto do assento; no meio da virilha a coxa fica limitada (65% abaixo do
  assento, 50% nele), senão o carrinho dobrava a frente da virilha do
  avesso; os vizinhos do alto da perna colados no meio (a menos de 1,6 cm, na
  altura da virilha) vão a meio caminho dos pesos dele (o triângulo entre
  eles não dobra quando uma coxa sobe muito: agachando, no carrinho); o
  quadril segue a pelve; o cós já puxa o abdômen. Com a
  camisa, a cintura passa da pelve ao abdômen na mesma faixa que a barra dela
  (os dois andam juntos quando a coluna dobra: o calção não atravessa a
  camisa). Os vértices do meio dividem as duas auxiliares por igual
  (simétricos).

### A camisa (`partes/camisa.json`)

- **A forma é um campo** (`ShapeField`): o tronco em seções horizontais
  (`torsoCenterX` é a costura do lado, `torsoFront/Back/Side` e os dois
  expoentes por altura) unido com mistura suave (`sleeveBlend`) a uma
  corrente de esferas em volta do braço (`sleeveRadius/Lift/Forward`, pela
  distância desde a junta do ombro). A axila e o alto do ombro saem da
  mistura. O lado do tronco é mais fechado que uma elipse
  (`torsoSideExponent` ~1,5): a faixa preta aparece ~2 cm de frente e de
  costas e ~7 cm de lado, como nas referências.
- **A malha é um molde de costura** (os vértices projetados no campo):
  - anéis do tronco da barra (`hemHeight`) até a axila (`armpitHeight`), com
    colunas no meio de trás, nas costas, no canto de trás da cava, nas bordas
    da faixa e na costura, no canto da frente da cava, na frente e no meio
    da frente;
  - a cava: o contorno do corpo no plano `armholeDepth` (o centro no eixo da
    manga), da axila até o ponto do ombro, refeito em `armholeRows` passos
    iguais na frente e atrás;
  - a costura do ombro, do ponto do pescoço (`neckPointX/Side`, visto de
    cima) até o ponto do ombro;
  - a borda da gola: o V de frente (`vNeckHeight`, altura pela distância do
    meio) e a volta de trás (`backNeckHeight`), subindo por cima do ombro até
    o ponto do pescoço;
  - os painéis da frente e de trás: Coons entre o anel da axila, o meio (até
    o V ou a gola), a cava e a costura do ombro, projetados no campo e
    alisados (`smoothing`).
- **A manga:** a cava (anel 0) e anéis em volta do braço (`sleeveRings`),
  passando dos ângulos da cava para ângulos iguais; o punho preto
  (`cuffWidth`) e a ponta inclinada (`sleeveEndTilt`, o alto mais longe); a
  ponta dobra para dentro até a abertura do braço (`cuffOpening`).
- **A gola:** a borda de fora (a dos painéis) sobe num chanfro
  (`collarRaise`), a faixa (`collarWidth`) vai para o pescoço e a borda de
  dentro desce até a pele (`collarDepth`): a volta onde o pescoço começa.
- **A barra:** a dobra por dentro (`camisa_dentro`) costura no alto do
  calção (`tuckHeight`). Até a dobra, cada ponto do tronco fica ao menos
  `hemClearance` por fora do calção (o raio de dentro mede o calção de
  verdade).
- **Números:** `number` no peito (`frontNumberHeight/Center`) e nas costas
  (`backNumberHeight/Center`), recortados como regiões depois do espelho (o
  número cruza o meio). A fonte é a de `NumberGlyphs` (a mesma do calção).
- **Facetado:** para dentro/fora (`jitter`), para cima/baixo nos anéis
  (`slideJitter`, até 35% do passo) e ao longo do anel (`ringSlide`); nunca
  no meio, nas bordas da faixa, na barra ou na axila.
- **Skin:**
  - o tronco pela altura: abaixo do abdômen a faixa da cintura (a mesma do
    calção), acima linear de junta em junta (a frente do peito não amassa
    quando a coluna enrola);
  - em volta da cava a auxiliar do ombro, entrando ~14 cm no tronco (com a
    rampa curta, a frente da cava se cruza quando o braço vai para a frente)
    e mais fraca embaixo do eixo (quando o braço desce, o que fica sob o
    ombro gira para dentro do corpo);
  - a manga passa da auxiliar ao braço em ~10 cm em cima e ~20 cm embaixo,
    na axila: a dobra quando o braço desce ou vai para trás se espalha (com
    10 cm em volta toda, um triângulo de trás da cava virava à vista no
    sprint e a manga se cruzava com o tronco em quase toda pose);
  - a gola é da camisa: não gira com o pescoço.
- **Comportamento conhecido:** com o braço colado ao corpo, o lado de baixo
  da manga encosta no lado do tronco (compressão, aviso; escondido debaixo do
  braço); com o cotovelo muito dobrado (110° ou mais) o antebraço encosta no
  punho da manga.

### O braço (`partes/braco.json`)

- Da abertura do punho da manga até o pulso (a junta da mão), em anéis com os
  mesmos ângulos do punho (sem torcer). Duas partes do menu: `braco_*` até
  `forearmFrom` (~4 cm depois do cotovelo: a dobra inteira fica nela, como o
  joelho na perna) e `antebraco_*` dali ao pulso. O último anel (liso, sem
  facetado) é o do pulso, onde a mão costura; sem a mão, a abertura `pulso`.
- **Seção:** superelipse de quatro quadrantes em volta do osso (`front`,
  `back`, `top`, `bottom`), com o centro um pouco à frente (`forward`) e o
  olecrano saltando atrás do cotovelo (`olecranon*`). Na pose T com a palma
  para baixo, perto do cotovelo o lado largo é o vertical; no pulso, o do
  polegar ao mínimo; a massa dos flexores fica embaixo.
- **Skin:**
  - atrás do cotovelo (o lado de fora da dobra) a auxiliar segura a quina numa
    faixa que some uns 3 cm depois;
  - na frente a pele dobra para dentro: braço e antebraço se dividem numa
    faixa larga (de 4,5 cm acima a 7 cm abaixo da junta), a dobra se espalha
    e nada vira até uns 90° (a corrida dobra 76° a 98°). A 110° viram uns 7
    triângulos dentro da prega;
  - os últimos 3,5 cm antes do pulso passam do antebraço à auxiliar do pulso
    (o anel do pulso gira a metade da mão).

### A mão (`partes/mao.json`)

- **Referencial da mão:** do pulso (a junta `RightHand`) para o dedo médio,
  para o lado do polegar e para o dorso. A mão é a direita; a esquerda sai no
  espelho (com os dedos do esqueleto espelhados, ver "Esqueleto").
- **A palma:** seções por estação (`radial`, `ulnar`, `back`, `palm`,
  `exponent`; na estação 0 é o anel do pulso do braço). O anel do pulso (20
  pontos) costura no primeiro anel da palma (32 pontos, `stitchLoops`). Os
  anéis da palma (`palmRings`) põem os pontos nas mesmas frações do contorno
  que a linha dos nós e, chegando nela, vão tomando a forma dela (o arco dos
  nós, a prega da palma).
- **A linha dos nós** (o último anel da palma, 32 pontos): de cada dedo, o
  alto (os dois pontos do nó, `knuckleBump` acima do dorso, e os lados) na
  estação da junta e a palma na prega da base do dedo (um pouco antes da
  forquilha, descendo até perto da espessura do dedo: sem degrau); o
  indicador e o mínimo também dão a borda da palma (`palmEdge` além dos nós).
  Entre os dedos, um vale no dorso e outro na palma.
- **Os dedos:** octógonos com uma face em cima (onde fica a unha) e outra
  embaixo, nas medidas de `fingers` (largura e espessura na base, na junta do
  meio, na da ponta e perto da ponta). A volta da base de cada dedo são os
  pontos dele na linha dos nós mais os da **membrana**: o lado de baixo,
  entre dois dedos, na forquilha (`webDepth` da primeira falange); os vales do
  dorso e da palma descem até a aresta da forquilha (6 triângulos por
  membrana). Depois: o meio da primeira falange, a junta do meio (com o nó
  `jointBump` em cima e a prega embaixo), o meio da segunda, a junta da
  ponta, e a ponta arredondada (`tipStart`, `tipScale`) fechada num ápice um
  pouco para a palma (a polpa).
- **As unhas:** a face de cima das duas últimas faixas de cada dedo, no
  material `unha` (`colors.nail`): uma região da própria pele, não uma peça.
- **O polegar:** sai de uma **janela** na lateral da palma: três anéis
  seguidos a partir de `thumbFromRing` e três pontos em volta, centrados onde
  o eixo do metacarpo (`Thumb1` → `Thumb2`) cruza a palma; sem os quatro quads
  em volta do ponto do meio, a borda da janela tem 8 pontos e o polegar
  começa nela (os anéis seguintes começam nos ângulos da janela e chegam ao
  octógono na junta IP). A eminência tenar salta em volta do metacarpo
  (`thenarBulge`). `thumbRoll` gira a unha do polegar a partir do dorso do
  osso (−0,6 rad: ela olha para a frente, como na referência).
- **Skin:**
  - a palma na mão; perto do pulso, a auxiliar do pulso;
  - a eminência tenar anda com o metacarpo do polegar (`Thumb1`, ~60% na
    janela, sumindo uns 3,5 cm em volta);
  - a linha dos nós entre a mão e a base de cada dedo (o alto 55/45, a palma
    40/60, os vales repartidos com os dois dedos), a membrana 35/65;
  - nos dedos, cada falange no seu osso e as juntas 50/50.
- **Deformação:** nada vira do avesso com a mão aberta, no punho do jogo
  (85°/100°/70°), com os dedos dobrados à mão, o pulso dobrado nos dois
  sentidos e de lado e nas poses do jogo. No punho, as palmas dos dedos se
  apertam (autointerseção, aviso). O punho do jogo (`RagdollVisualPose3D`)
  fecha o polegar por cima dos dedos (10°/35°/55°, fechando 1,7): antes ele
  dobrava para baixo da palma.

### O pescoço (`partes/pescoco.json`)

- Da volta de dentro da gola (no V, a pele do alto do peito) até a abertura
  `cabeca`: um plano inclinado (`headFrontHeight` sob o queixo,
  `headBackHeight` na nuca). Seções por altura como o tronco.
- **Skin pela volta** (0 na gola, 1 na cabeça), não pela altura: a pele passa
  do alto do peito ao pescoço subindo da gola (no V ainda é peito; atrás, logo
  acima da gola) e a cabeça puxa a volta de cima. Com a cabeça, dobrando o
  pescoço e a cabeça (para a frente e para trás até 50° somados, de lado 30°,
  girando 45°) nada vira à vista; ficam avisos de pele esticada ou amassada
  na junta e pequenas autointerseções escondidas do lado que comprime.

### A cabeça (`partes/cabeca.json`)

As medidas vêm de `~/Área de trabalho/modelagem/rosto.png` (frente, lado e
cima do rosto) e das referências do corpo (`frente.png`, `costas.png`, o
lado). O contorno e o tamanho seguem as do corpo; os traços do rosto, o
`rosto.png` na escala proporcional (o alto na linha 170 = 1,905 m, o queixo
na 685 = 1,672 m: 0,4524 mm por pixel; de lado,
x = 0,096 + (coluna − 940) · 0,0004524).

- **A forma base:** seções de quatro quadrantes por altura em volta de
  `centerX` (`front`, `back`, `side`, os expoentes), da volta de cima do
  pescoço até o alto (`crownHeight`). A metade da frente tem expoentes
  próprios (`frontExponent`, `frontSideExponent`): o rosto redondo visto de
  cima, como na referência (os cantos do olho e da boca bem para trás do
  meio).
- **Os campos do rosto** (`HeadFace`), somados à seção na metade da frente:
  - o arco da testa (`ridge*`), a órbita (`socket*`), as almofadas dos lábios
    (`upperPad*`, `lowerPad*`), as maçãs (`cheek*`) e a bochecha cavada abaixo
    delas (`hollow*`);
  - o **perfil do meio** (`profile`): o x exato no meio, sem o nariz (queixo,
    lábios, base do nariz, raiz entre os olhos, glabela, testa); a diferença
    para o que o meio já tem se espalha até `profileWidth`. Perto da boca o
    perfil é lido numa altura apertada para a linha (`lipTaper`): os lábios
    afinam até os cantos;
  - o **nariz** por cima: `noseProfile` (o meio, da base entre os lábios à
    raiz), `noseWidth` (onde encontra o rosto) e `noseShape` (o corte: frente
    reta, lados a pique); as **asas** são cúpulas (`ala*`) unidas ao nariz com
    a emenda arredondada; as **narinas** entram por baixo da ponta
    (`nostril*`, material `narina`);
  - a **mandíbula**: `jawLine` (a altura pelo ângulo em volta, do meio do
    queixo ao canto e subindo até a orelha), a borda saltando logo acima
    (`jawRidge`) e, embaixo, a pele indo da seção do alto do pescoço até a
    linha (`jawCurve`: reta no pescoço, virando para baixo perto da quina).
- **Os traços fixos** (`faceLayout`): o meio da frente (a borda do domínio),
  a abertura do olho (8 pontos, a amêndoa medida), a linha da pálpebra de
  baixo, a pegada da sobrancelha, a quina e a base da ponte, a volta da asa,
  a narina, o sulco do nariz à boca, a linha da boca, os contornos e a linha
  mais cheia dos lábios, o filtro, a linha da mandíbula e a raiz da orelha.
  São vértices e arestas obrigatórias; o resto da malha é triangulado em
  volta deles.
- **A malha** (`HeadGenerator`): a pele da metade direita é desenrolada num
  domínio plano — em volta, o ângulo vezes o raio médio da seção; na
  vertical, o comprimento da pele ao longo do meridiano (no alto, onde a pele
  deita, um centímetro de altura é vários de pele). A borda é a volta do
  pescoço, o meio de trás, a fileira do alto e o meio da frente; os furos são
  o olho, a sobrancelha e a raiz da orelha. Pontos de preenchimento (`faceSpacing`
  no rosto, `skullSpacing` no crânio, `detailSpacing` perto dos traços e da
  orelha) e a Delaunay restrita (`Triangulation2D`). Acima da fileira do alto
  (`capHeight`, `capColumns`) o crânio fecha em anéis menores até o ponto do
  alto.
- **O olho** (`HeadEye`): da abertura, a borda da pálpebra entra
  (`lidThickness`); o globo acompanha a borda — o contato logo atrás dela
  (`eyeGap`; na de cima a linha dos cílios escura, na de baixo o globo encosta
  direto), a íris escura em domo (`eyeBulge`), com os lados quase retos e
  encostando nas pálpebras em cima e embaixo (`irisRadius`). Visto de frente
  cada faixa fica dentro da anterior: a pálpebra cobre o globo e nada se
  cruza.
- **A sobrancelha** (`HeadBrow`): a pegada (`browTop`, `browBottom`, a ponta
  `browTip*`) vira furo e o bloco sobe dela: a parede, a quina ao longo dela
  (`browThickness`) com a face de cima e a da frente, o fim de dentro fechado.
- **A orelha** (`HeadEar`): o contorno visto de lado (`earOutline`: para a
  frente, para cima e o quanto se afasta da cabeça, anti-horário visto de
  fora) em volta do centro da concha (`earX`, `earHeight`). Da raiz
  (`earRoot` vezes a borda da concha) ela sobe pelas costas lisas até a borda
  da hélice (`earRim`), dobra por cima e desce pela frente — a aba, o sulco, a
  anti-hélice — até a concha (`earConcha*`) e o fundo na sombra
  (`orelha_canal`). No lóbulo o relevo some.
- **Skin:** a pele perto do pescoço passa da cabeça (45%) para o pescoço; uns
  4 cm acima da volta já é só a cabeça. O olho, a sobrancelha e a orelha são
  da cabeça.
- **Comparação:** os scripts da sessão de modelagem põem o modelo
  (ortográfico, `--luz-frontal`) ao lado do `rosto.png` na mesma escala, de
  frente e de lado, e sobrepõem os contornos de perfil.

## Tela "Inspeção de jogador"

Menu principal → **Inspeção de jogador**. A antiga tela "Animações" do menu
principal virou uma aba desta.

- **Lateral**, com duas abas:
  - **Partes do corpo:** as 18 partes, mais as dobras de teste. Clicar numa
    parte faz um close-up com transição.
  - **Animações:** os clipes que o personagem usa no jogo, agrupados (Parado,
    Corrida, Sprint, Pulo, No chão), com nomes curtos. Os clipes carregam
    numa thread própria (`PlayerAnimations`). A malha toma a pose do jogo
    (`sampleGameRigPose`, a mão de repouso, sem andar). Ao entrar na aba, a
    câmera volta ao corpo inteiro; ao voltar às partes, voltam as dobras.
- **Barra:**
  - vistas (frente, costas, lados, 3/4, cima) e **Recentrar**;
  - modo de desenho;
  - malha (o modelo montado ou só os ossos);
  - **Referência**: a imagem do usuário com o modelo por cima, na mesma escala;
  - **Salvar imagem**, que grava em `inspecao/capturas/`.
- **Opções:** ossos, juntas, nomes, grade, só a parte, avesso em magenta,
  ortográfica.
- **Sombreamento** (nos modos Sólido e Sólido com arame): **Facetado** (0
  liso, 1 as faces de verdade) e **Quinas** (as faces que dobram mais que o
  ângulo ficam marcadas), e **Padrão** (facetado 0,5 com quinas de 45°, o
  sombreamento do jogo). A argila e os modos de análise continuam mostrando
  as faces.
- **Carga:** o modelo é montado numa thread própria. A tela não trava:
  mostra "CARREGANDO O MODELO..." (ou o modelo de antes, numa recarga) e
  avisa "Modelo montado em 0,9 s" quando entra. Recarrega sozinha quando
  muda qualquer arquivo lido: os do manifesto, as malhas base, a chuteira, o
  molde, o `pintura.json` e as fontes, os SVG e as imagens dele.
- **Dobrar (teste):** quatro grupos, em graus; a malha aparece deformada
  (com as auxiliares) e **Zerar dobras** volta à ligação.
  - **Pernas:** joelho, quadril (para a frente e abrindo) e tornozelo de cada
    lado.
  - **Tronco:** baixar o braço, braço à frente e cotovelo de cada lado e a
    coluna. Baixar o braço mostra o ângulo dele abaixo da horizontal: 56° na
    ligação, 90° colado ao corpo, 0° na pose T. O cotovelo fecha na
    dobradiça dele (o plano do braço).
  - **Cabeça:** o pescoço (para a frente, de lado, girando) e a cabeça (para
    a frente, de lado, girando).
  - **Mãos:** a mão de partida (reta, solta = a de repouso do jogo, aberta,
    punho: só os dedos mudam, o corpo fica na ligação) e, por cima, o pulso
    (para a palma e de lado, pelos eixos da mão), os dedos e o polegar de
    cada lado. "Dedos" vai de 0 a 100%: a mão fecha da partida até o punho do
    jogo (`blendJointBends`), e de qualquer partida termina no mesmo punho,
    sem passar da palma. O polegar fecha atravessando a palma
    (`fingerFlexionAxis`).
  - Todos os eixos são os da partida e giram junto com o pai
    (`followParent`): com o braço levantado o cotovelo não torce, e com o
    pescoço inclinado a cabeça gira em volta dele.
- **Tocador** (embaixo do viewport, só na aba Animações):
  - **Tocar/Pausar** e **Do início**;
  - **Repetir**;
  - velocidade ¼×, ½×, 1× e 2×;
  - a linha do tempo, que pode ser arrastada, com o instante e a duração.

  Um clipe que não repete para no último quadro. Tocando, a imagem sai sem
  supersampling (~12 ms por quadro em 1610×716) e a completa vem quando
  pausa.
- **Sem texto por cima:** a imagem não tem título, notas nem a legenda dos
  eixos, o rodapé só mostra avisos (como "Imagem salva") e o cabeçalho não
  repete o nome do modelo.
- **Mouse:**
  - arrastar com o esquerdo gira;
  - direito ou do meio move;
  - a roda aproxima;
  - clicar numa junta escolhe a parte dela;
  - o duplo clique recentraliza.
- **Desempenho:** a imagem só é refeita quando algo muda. Girando, sai sem
  supersampling. As superfícies e a grade são desenhadas em faixas de linhas
  paralelas (o agendador da engine), e a redução do supersampling também:
  o jogador em 1400×900 leva ~14 ms girando e ~33 ms (o corpo) a ~45 ms (o
  rosto de perto, liso) parado. A seção "Desempenho" tem as medidas.

## Testes

- **`MatterEngine.Modeling`** (`tests/Modeling/ModelingTests.cpp`) é linkado
  só contra o núcleo. Cobre:
  - geometria e kit de modelagem;
  - a triangulação restrita, a costura de voltas, o recorte (a trava saindo
    do furo) e o espelho que leva a costura;
  - a checagem de conexão (peça solta, costura faltando, sem solda, borda);
  - detecção de furo, avesso, interseção e assimetria;
  - circunferência de corte;
  - enquadramento e render;
  - o sombreamento liso (a caixa com a normal da face em cada canto, o tubo
    liso no lado e chapado nas tampas, a mistura com a face, a pose girando
    as normais junto, o render liso com a luz contínua no lado do tubo);
  - PNG, fonte e silhueta;
  - skin e deformação (o avesso a vista e na prega; a tampa de uma caixa
    puxada através dela, do avesso para fora sem a normal virar contra a
    esperada, acusada pela dobra; a caixa parada sem avesso);
  - as dobras de teste (o eixo preso ao pai: a coxa aberta e o joelho ainda
    dobrando; a dobradiça da junta, reta e dobrada; a mistura de poses
    inteira e pela metade);
  - o encaixe numa referência (Horn achando uma semelhança conhecida e a
    inversa desfazendo; o ICP achando de volta um alinhamento conhecido a
    partir de um chute errado; a projeção na superfície pela normal e nada
    além da distância; as normais dos vértices);
  - sessão.
- **`MatterEngine.ModelingAssembly`** (`tests/Modeling/AssemblyTests.cpp`),
  só contra o núcleo, cobre os módulos da montagem:
  - o corte (por plano: uma volta, o lado da normal; de perto: a superfície
    fechada, o pedaço entre dois cortes saindo; por superfície: a curva
    inteira);
  - os puxões (o centro anda o puxão inteiro, nada fora do raio, o meio no
    meio) e o pescoço mais curto (o chão e a cabeça parados, a base subindo
    tudo, a altura sempre crescendo);
  - a faixa do pescoço (a base acima do fundo do V na frente e na gola do
    lado; os pesos do peito ao pescoço e dele à cabeça, somando 1; nada fora
    dela nem longe do meio);
  - os vértices levados a lugares exatos (o mais perto anda, o do meio fica
    no meio, sem vértice perto é erro);
  - a importação da cabeça base (344 triângulos; a fresta da nuca fechando
    com a solda maior; orientada para fora) e o arquivo que falta;
  - o calor dos ossos (os pesos somando 1, o cotovelo dividido, a borda
    costurada mantendo o peso dela, a cintura em cima da junta da coxa com a
    raiz, o vértice preso por um pino);
  - o espalhamento, a mistura com o espelho no meio e o peso preso; a
    fração de umas juntas passando para outra; o espalhamento em volta de um
    ponto (só perto dele; com a parte, a costura e a outra parte paradas);
  - o pano: a manga entre as barras, a pele coberta saindo, a folga fora do
    corpo e os recuos fechando uma superfície só;
  - os reparos (a diagonal trocando, a aresta curta juntando sem furo);
  - o atlas (uma ilha por face da caixa, sem sobreposição, a mesma densidade)
    e o bake (a cor de cada face; a borda de um desenho mais larga com
    `edgeTexels`; o adaptativo igual ao completo nas listras de borda suave e
    numa linha mais fina que um texel na parte fina), a amostra bilinear e o
    espelho levando o mapa;
  - o arranjo das ilhas pelo formato: o quadrado que só cabe no vão de um L,
    uma ilha em pé, cada texel com uma ilha só, a folga dupla entre duas, o
    lugar só movendo e girando e a ilha maior que o atlas;
  - o rosto (as pupilas dos dois lados, a sobrancelha, a linha da boca e o
    lábio de cima; abaixo da fresta, a testa e a nuca sem tinta) e a placa
    da orelha medida numa malha (a elipse inclinada: o contorno da maior
    peça, o centro, os semieixos e a inclinação; uma peça menor, um
    triângulo virado para a cabeça, um perto do meio e um de outra parte de
    fora).
- **`MatterEngine.ModelingPaint`** (`tests/Modeling/PaintTests.cpp`), só
  contra o núcleo, cobre a pintura em dados:
  - as formas (dentro e fora, a distância com sinal, a cobertura na borda, a
    união sem costura, o furo e o espelho dele, o chanfro, o canto redondo, a
    linha e o anel) e as fontes (o 10 de bloco na largura medida, uma
    TrueType com o H de altura 1 e o furo do A);
  - as tintas e as camadas (os degradês, multiplicar, a opacidade, o
    contorno, os lados);
  - os espaços: o plano e a volta desenrolada num cilindro (o arco a partir
    da costura em cinco ângulos, a altura, o espelho) e um tubo de um lado só
    com os anéis inclinados e uma fenda embaixo (a fatia aberta embaixo e
    fechada em cima, no mesmo triângulo): o arco certo em volta inteira,
    sem borrar no corte nem na fenda;
  - os caminhos de SVG (o quadrado, relativo com H e V, números colados, o
    furo, o círculo em arcos com as bandeiras coladas, o quarto de círculo
    numa cúbica, Q e T, os erros) e o arquivo SVG (o viewBox e os caminhos
    pelo id);
  - a imagem (a cor, o transparente, a beira sem franja, a média no nível
    baixo) e no desenho (a caixa pela proporção, o vermelho pintado e o
    transparente deixado; o caminho no quadro com o y virado);
  - o desenho em JSON (as âncoras com deslocamento, o texto centrado com o
    contorno, o degradê multiplicando; os erros com o nome da camada).
- **`MatterEngine.ModelingParts`** (`tests/Modeling/PartsTests.cpp`), só
  contra o núcleo, cobre as peças procedurais:
  - as curvas e a validação das receitas (chuteira, perna, calção, camisa,
    braço, mão, pescoço, cabeça);
  - a chuteira uma superfície só (só a gola aberta), sem interseção, no chão,
    com as travas tocando o chão;
  - a cintura para baixo montada (chuteira, perna, calção, espelho): uma
    superfície, só o cós aberto, o meio soldado;
  - o corpo inteiro (com camisa, braço, mão, pescoço e cabeça, a mão nas
    juntas do jogo): uma superfície fechada (sem a cabeça, só o alto do
    pescoço aberto; sem as mãos, os dois pulsos), os materiais do rosto, sem
    interseção, a barra na altura da receita e o calção parando na dobra dela;
  - o campo de forma (valor, projeção, mistura), a seção de dois expoentes e
    os algarismos (contornos simples, furos dentro);
  - os padrões iguais às receitas;
  - a folga exata do drapeado (inclusive numa sela).
- **`MatterEngine.PlayerModel`** (`tests/Modeling/PlayerModelTests.cpp`)
  cobre:
  - o manifesto real e o esqueleto do jogo (com as auxiliares, a do joelho
    girando a metade; os dedos da esquerda no espelho dos da direita; numa
    pose do jogo, toda junta onde o pai a põe);
  - o modelo montado: as 12 partes (corpo, cabeça, camisa, calção e as
    chuteiras), menos de 6 000 triângulos, 1,80 m, uma superfície fechada
    (nada aberto), simétrico; o atlas de 2048 em todos os materiais e a
    pintura no lugar (o 1 do peito, o vão entre o 1 e o 0, o número das
    costas, a faixa do meião abaixo do joelho, as faixas do lado, as pupilas
    e a testa; a placa da orelha medida do tamanho e na altura de uma
    orelha e o escuro dela na metade da frente, igual dos dois lados);
  - a deformação nas poses do jogo (parado, trote, corrida) e dobrando a
    coluna, o ombro, o cotovelo (90°), o pulso, os dedos, o pescoço e a
    cabeça (e os dois girando juntos até o fim dos controles, 95°), com a
    mão aberta e o punho: nenhum triângulo virado à vista e o calção nunca
    atravessando a camisa;
  - o polegar fechando atravessando a palma e as duas mãos espelhadas (o
    polegar dobrado pelo eixo da junta e as mãos solta, aberta e punho do
    jogo);
  - as falanges dobrando no plano de cada dedo (a menos de 6°), a mão de
    partida sem mexer no corpo e toda partida fechando no mesmo punho;
  - o exemplo da pintura (`exemplos/pintura_exemplo.json`) válido nos espaços
    medidos no modelo de agora, com o escudo no peito esquerdo; a carga com
    a lista dos arquivos lidos e o tempo de cada etapa (menos de 5 s);
  - o chão do espaço do modelo (o do `model.json`) e o repouso em pé nele
    com a pelve na altura de pé do perfil;
  - o jogador para o jogo (`exportar-jogo` numa pasta temporária): o
    personagem do jogo em dia com o modelo, os 76 ossos (26 auxiliares, 32
    dedos), o sombreamento 0,5, a pele do jogo igual à malha posada da
    modelagem em sete poses, duas com os braços erguidos (de lado e à
    frente: a rampa e o volume da axila e a clavícula; menos de 0,5 mm) e a
    sola no chão do perfil. Roda antes das outras checagens;
  - o molde da cabeça de referência: o alinhamento (erro médio abaixo de
    4 mm, a referência quase de pé), os vértices que vão inteiros para o
    molde em cima da superfície dela (a menos de 0,5 mm) e a junta da cabeça
    recuada junto com ela;
  - as dobras fortes como na tela (cotovelo 145°, joelho 150°, pulso ±70°, o
    braço levantado com o cotovelo a 120°, a coxa à frente com o joelho a
    130°, a mão fechando da solta; o polegar contraído até o fim do
    controle, 32/64/80°, de cada mão de partida, sozinho e com a mão
    fechando 5% além do punho): nenhum triângulo virado à vista;
  - **todas as animações do jogo** (seis instantes de cada, com a mão solta
    e com o punho, ~528 poses): nenhum triângulo virado à vista em parte
    nenhuma;
  - a malha antiga e as poses;
  - a tela rodando sem janela (a montagem em segundo plano: o primeiro
    quadro sai sem esperar), com fonte e tema reais (inclusive o punho nas
    dobras das mãos e a cabeça girada no grupo da cabeça). Qualquer erro que o
    ImGui recupera vira falha;
  - a aba Animações:
    - os clipes do personagem carregados, todos com nome limpo;
    - a pose do jogo na malha e o corpo inteiro na câmera;
    - o sprint tocando, pausar segurando o instante e a linha do tempo;
    - o pulo parado, que não repete, parando no último quadro;
    - a volta às partes, com a pose de ligação.
- **Foto da tela sem janela:**
  `MATTERENGINE_UI_SNAPSHOT=<pasta> build-profile/MatterPlayerModelTests`
  grava fotos da tela de verdade: os triângulos do ImGui rasterizados em
  software. O contexto, o canvas falso e a foto ficam em
  `tests/Support/HeadlessUi` (a biblioteca `MatterTestHeadlessUi`). A tela
  "Objetos" usa o mesmo apoio (`MatterObjectInspectorTests`).

## A fonte Blender (22/10/2026)

A malha e os pesos do jogador passaram a ser modelados no Blender.
- `assets/characters/player_model/blender/jogador.blend` é a fonte e `jogador.troca.json` é o que o motor lê (`model.json`: `"source": "blender"`; leitor em `Engine/Modeling/Import/BlenderExchange`).
- **Ida:** `MatterModelInspector exportar-blender --saida <pasta>` e depois `tools/blender/importar_troca.py`.
- **Volta e checagens:** `tools/blender/sincronizar.sh`.
- **No Blender:** os auxiliares são pré-visualização (restrições). As regras que valem são as do `GameRig`.
- **A montagem por código** continua para gerar trocas novas (uniforme, pintura).
- **O template de superfície** (26/10, o sistema de aparência novo): a etapa 01 grava o molde de cada face (atributo `molde`), a troca leva os moldes e `tools/blender/superficie.py` acrescenta o template (painéis, cartas, regiões, costuras, âncoras, zonas, UV0 e UV1). O contrato está em `docs/SISTEMA_DE_APARENCIA.md`; o `exportar-jogo` leva o template ao jogo.
