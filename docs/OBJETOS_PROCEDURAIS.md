# Objetos procedurais

Os objetos físicos do laboratório não são mais modelos GLB. Cada objeto é
**gerado por código a partir de uma receita**: a malha, a textura em pixel
art e a descrição física (material, massa, colisores). Hoje existem dois
tipos: a **caixa de madeira** e a **trave** (com a rede de fios, desde 25/10).

Estado em 09/10/2026: cadeira, bigorna, barril, caixa e pneu foram removidos
junto com os GLB, os caches `.mecollider` e o cozinhador V-HACD
(`MatterPhysicsAssetCooker`). No menu de spawn ficam a caixa procedural, a bola
de futebol (inflada) e o boneco.

## Arquitetura

```
MatterModeling   src/Engine/Modeling/Objects/   o núcleo: receita, tipos, geradores, pixel art
  (sem engine, sem ImGui, sem SDL; regra no check-architecture)
MatterPhysics    src/Engine/Physics/Net/         a rede de fios (só matemática; regra no check-architecture)
                 src/Engine/Physics/Jolt/JoltNets3D  a rede na cena (nos subpassos da bola)
MatterObjects    src/Engine/Objects/             a ponte com o jogo: malha da GPU, corpo físico, laboratórios
MatterWorkbench  src/Workbench/ObjectInspector/  a tela "Objetos" (só pelo CanvasTarget)
                 src/Workbench/Props/PropCatalog  o catálogo do laboratório (spawn)
```

- **`ObjectRecipe`**: a receita em JSON. Lê e grava com cores em `#rrggbb` e
  números arredondados a 4 casas.
- **`ObjectType`**: um gerador com os parâmetros dele. Cada parâmetro tem
  chave, rótulo, grupo na tela, tipo (número, inteiro, liga/desliga, cor),
  padrão, mínimo, máximo e unidade. O que a receita não traz fica no padrão.
  `objectTypes()` lista os tipos registrados.
- **`ProceduralObject`**: o resultado da geração. Traz a malha da modelagem
  (com material, UV por canto de triângulo e a textura já ampliada), a pixel
  art no tamanho real, a física e a medida da caixa que contém o objeto.
- **`PixelCanvas`**: o pincel da pixel art. Tem retângulos, contornos, linhas
  grossas, sombra e luz, variação por ruído determinístico e a ampliação por
  vizinho mais próximo (cada texel vira um bloco nítido).
- **`buildTexturedBox`**: a caixa com 8 cantos, uma região do atlas por face e
  o mapa por canto de triângulo.
- **`objectRenderData3D`** (`Engine/Objects`): a malha da GPU. Os vértices
  se separam pela quina (45°) e pelo mapa, e a textura vai em RGBA.
- **`objectBodyData3D`**: o corpo do Jolt. Usa a massa da receita, as formas
  e o material do catálogo (um material desconhecido vira `default`). O
  cilindro vira cápsula e um objeto `fixed` vira estático.
- **`ObjectPhysicsLab3D`**: uma cena física própria com chão de concreto e os
  roteiros, cada um com as suas medidas:

  | Roteiro | O que faz | Medidas |
  |---|---|---|
  | Queda | Solta o objeto de uma altura, inclinado | velocidade no toque, quique |
  | Arremesso | Lança o objeto com giro | percurso, maior velocidade |
  | Rampa | Apoia o objeto numa rampa com a inclinação dada | quanto escorregou |
  | Pilha | Empilha N cópias | desvio do topo, se caiu |
  | Bola | A bola de futebol inflada bate no objeto | velocidade da bola e do objeto depois do toque |

  Em todos os roteiros mede também quando parou e a face em que ficou
  apoiado. Roda ao vivo (`advance`) ou de uma vez (`run`).

## A receita: `assets/objects/<id>.json`

```json
{
  "schema": "matter.object.v1",
  "id": "wood_crate",
  "displayName": "Caixa de madeira",
  "type": "caixa",
  "material": "wood",
  "parameters": { "largura": 0.6, "altura": 0.6, "massa": 14, "pixels": 32, "cor_madeira": "#d98e3a" }
}
```

Toda receita da pasta aparece no inspetor e no menu de spawn do laboratório.

## A caixa de madeira (tipo `caixa`)

Um cubo só, sem tábuas em malha: o desenho fica todo na textura.

- **Forma:** largura, profundidade e altura.
- **Física:** massa e material.
- **Textura:**
  - resolução (pixels no lado maior);
  - número de tábuas;
  - largura da moldura;
  - reforço em diagonal e pregos;
  - veios, variação de tom e semente.
- **Cores:** madeira, moldura, realce, sombra e prego.

A densidade de pixels por metro é a mesma em todas as faces. O atlas tem três
ladrilhos: o lado X, o lado Y e o tampo.

## A trave (tipo `trave`, receita `assets/objects/trave.json`)

O tipo dos desenhos do usuário: a rede presa só na estrutura da própria trave
(a rede em mastros externos é um modelo futuro). Tubos de metal redondos,
varridos por `buildTube` (`ObjectMesh`: anéis sem torção, canto soldado no
corte exato da bissetriz ou dobrado em arco, tampas; cada tubo fechado):

- **O quadro da frente:** o vão interno da Regra 1 (7,32 × 2,44 m entre as
  faces de dentro), tubo de 12 cm, os cantos soldados (ou curvados).
- **O quadro de trás:** de cada lado, a barra de cima (0,80 m), a dobra, a
  diagonal até o chão (2,00 m de fundo) e a barra do chão até o pé do poste; a
  barra de trás no chão. As medidas de trás são as dos fabricantes que seguem a
  DIN EN 748 (a regra não define essa parte).
- **Física:** aço, fixa, um cilindro por trecho de tubo.
- **A rede** (`ProceduralObject::net`, `ObjectNet`): o teto, o fundo inclinado
  e as laterais, cortados na malha (quadrada, 12 cm) com o debrum nas bordas e
  presos nos tubos (a presilha a cada 2 malhas, 24 cm); o fio (6 mm desde
  26/10, 7 g/m, EA 10 kN; folga de 6% no fundo, 2,5% no teto e 8% nos lados desde 26/10, era 5% em todos); o cabo tensor de 6 mm entre o teto e o fundo, esticado
  0,75% (~225 N; era 0,5%, ~150 N, até 26/10). Em duas malhas da mesma rede:
  - **a da física** (a que anda), com a célula de `malha_fisica` malhas (2:
    24 cm). Cada fio dela vale pelos fios paralelos que representa (EA × 2);
    o peso e o fio contra o ar de cada nó são os da malha de verdade em volta,
    repartidos pelos pesos. As células em triângulos (a bola toca a rede por
    elas), com a diagonal espelhada no meio do pano e as colunas em número par
    (a rede igual dos dois lados). A escala de contato vem com ela (6 na
    malha de verdade, 4 na de 2, 3 na de 3: os nós grossos já são mais
    pesados). 623 nós, 1.176 fios, 1.096 células;
  - **a desenhada** (a de verdade, 12 cm): cada nó é uma média fixa de até
    quatro nós da física, os pesos baricêntricos do triângulo de colisão que
    o contém, como a pele nos ossos. Como os fios ficam nos triângulos em que
    a bola e os membros batem, eles nunca entram neles: nos chutes da grade,
    o fio encosta na bola (1,9 mm do centro do fio à superfície) sem entrar.
    `drawnNetKnots` dá a posição de agora. 2.311 nós, 4.476 fios, 4,1 kg.
- **Parâmetros:** vão, tubos, cantos, fundos, dobra; malha, fio, folga,
  rigidez e peso do fio, presilhas, cabo, física a cada (1 a 3 malhas: com 1,
  a física é a própria malha de verdade; com 3, um terço do custo da de 2, mas
  o bolso da bola fica facetado); massa; cores.

## A rede de fios (`Engine/Physics/Net/NetSolver3D`)

Pura (testável sem o Jolt). XPBD com muitos subpassos e uma iteração:

- **O fio** só puxa (estica pouco, dobra livre); acima de 15% ele não cede
  mais (a malha não abre a ponto de a bola passar). As passadas repetidas no
  mesmo subpasso (as do cabo, as rodadas da bola) continuam a mesma solução: o
  multiplicador elástico de cada fio se acumula no subpasso (XPBD) e o total
  nunca empurra. Antes, cada passada repetida encolhia o fio de novo e o
  deixava mais rígido (5 passadas num fio isolado: 1,556 mm de alongamento
  viravam 0,001 mm).
- **As amarras:** cada nó solto tem uma mola até o nó preso mais perto pelo
  caminho dos fios (Long Range Attachments), com a rigidez desse caminho.
- **O cabo:** a tração puxa o nó para a reta entre as pontas com a rigidez
  lateral de uma corda esticada, T(1/a + 1/b).
- **O ar** (arrasto por nó), um atrito interno pequeno (o movimento lento
  para) e os tubos e o chão.
- **O amortecimento do fio** é viscoso (EA × 8,5e-7 s × a taxa de
  alongamento), o mesmo em qualquer malha e passo. A fração fixa por fio
  amortecia 4× mais a malha de 24 cm.
- **O atrito interno** (o fio trançado, as presilhas) é 5/s desde 26/10 (era
  1,5). Com a bola nas massas de verdade, o chute mexe mais a rede, e com 1,5
  o teto ficava balançando ±5 mm sem dormir. Isso acontecia porque o nó do
  cabo perto da ponta tem um vaivém de ~0,17 mm por subpasso entre a reta do
  cabo e os elos, que já existia e alimentava o balanço. Com 5 a rede assenta
  em ~3,7 s, como antes.
- **Os corpos** (proxies): a bola e as cápsulas (os membros dos jogadores;
  a cápsula afunilada e a casca convexa da chuteira viram cápsula) batem nas
  células (triângulos: o ponto do eixo mais perto da célula); a caixa, nos
  nós virtuais (os nós da malha de verdade sobre a da física; a correção vai
  para os nós pelos pesos). Se o caminho do centro desde o começo do tick
  passa por uma célula, o corpo volta para o lado de onde veio: a rede não é
  atravessada.
  - **A bola, desde 26/10** (o pedido dele: a freada pela física da própria
    rede, não "algo por trás"):
    - **as massas de verdade:** o impulso na bola e nos nós é o mesmo. A escala
      de colisão tirava 3/4 da reação dos nós, e a bola perdia metade da
      velocidade nos primeiros centímetros, antes de a rede esticar;
    - **o nó apoiado num tubo** não cede na direção do tubo: a bola bate na
      moldura através da rede;
    - **as rodadas do acoplamento fio-bola** (`ballContactRounds`, 2): com a
      bola encostando, os fios, os tubos e o contato dela passam de novo no
      subpasso. Com as massas de verdade, a bola empurra os nós leves mais
      rápido que a tensão corre pela malha numa passada só; nas rodadas a
      tensão chega à moldura e o apoio aparece sozinho. Nos chutes de perto no
      canto do poste (30–45 m/s) o fio esticava 42% sem elas; com 2, 14%. Elas
      substituíram a escala que contava o nó de fio esticado como 8 vezes mais
      pesado;
    - **a correção:** só a parte que a aproximação do subpasso cria vira
      velocidade: a velocidade da bola contra o ponto material da rede × o
      subpasso, no máximo a profundidade. O resto é recuperação de penetração
      funda, só posição, na bola e nos nós (nos nós o `m_previous` anda junto,
      para a recuperação não virar velocidade). Os cruzamentos de tubo e de
      chão são medidos da posição do começo do subpasso (`m_start`): medidos
      do `m_previous` deslocado, a recuperação passava o nó para o outro lado
      do tubo;
    - **o atrito:** entra nas velocidades, depois das correções de posição,
      com reação nos nós. É só o Coulomb no deslizamento do ponto da bola
      contra o ponto material da rede, giro incluso. (A resistência ao
      rolamento de 0,4 que existia saiu na revisão: era freio sem física.)
    - **saíram** a desaceleração constante e o freio de giro desacoplado (ele
      zerava o giro em 2 ticks). Hoje o giro só muda pelo atrito: no impacto
      vira rolamento rede acima e volta.

    **Medido (final de 26/10, folgas 6/2,5/8% e cabo a 0,75%):**
    - o chute de 25 m/s no meio estufa ~0,95 m, freia ao longo de ~1 m e volta
      a ~1,7 m/s;
    - o giro: de topspin, 29 → 36 rad/s 10 ticks depois do contato (vira
      rolamento rede acima); em espiral, 29 → 25;
    - grade forte de 816 chutes (com os lances de duas bolas): nenhum
      atravessa e 535 de 688 gols voltam pela boca (sair pela boca pode,
      atravessar nunca). A travessia é julgada por fora do contato: a paridade
      do segmento da bola até a boca contra as células da física (cada
      cruzamento numa aresta ou vértice comum conta uma vez), além das caixas;
      um valor não finito na bola ou na rede reprova sempre;
    - o fio no máximo: 14,3% nos chutes de perto no canto do poste (30–45 m/s,
      4 m); 4,3% no chute de 25 m/s no meio.

    O revisor (Codex) aprovou na terceira revisão. Ficam duas limitações
    conhecidas, sem caso reproduzido: a lista de candidatos de cada corpo não
    varre o movimento das células no tick (uma célula rápida que começa fora
    da margem pode alcançar a bola sem estar na lista), e o teste do nó contra
    o tubo confere o caminho do subpasso em 7 amostras, não de forma contínua
    (`closestSegmentSegment` faria o caminho contra o eixo). A grade é finita:
    0 em 816 não prova "nunca atravessa" em qualquer lance.
  - **Caixa e cápsula** dividem a correção pela massa com a escala de colisão
    e levam a desaceleração constante ao longo da normal.
  O lado de onde o corpo veio é medido na célula do começo do tick (`m_tickStart`):
  com a célula de agora, a rede voltando do estufado mais rápido que a bola
  passava por cima dela, a bola ficava do lado de fora e saía empurrada (o
  "rechicote": chutes de 2-3 m a 45-60°, 40-45 m/s, na rede do lado).
  Caixa e cápsula giram dentro do tick. O contato delas dá no
  máximo 0,5 m/s de separação além de parar o corpo: um ponto que não cede,
  achado fundo na caixa que girou, a lançava a 23 m; a bola espremendo a rede
  contra o tubo do lado (de perto, na diagonal, a 30-45 m/s) saía a 40-60 m/s
  pela rede.
- **Os tubos de novo depois dos corpos:** a bola espremendo a rede contra um
  tubo passava o nó para o outro lado dele; ele ficava preso lá (o fio a 33%
  para sempre) e a rede explodia (nós a 500 m/s). Os tubos passam de novo
  depois do contato final.
- **O corpo guiado** (o boneco no modo padrão, que segue a animação): a rede
  sai inteira do caminho dele e relata o impulso que ele sentiria solto, com
  a massa dele (no máximo o de pará-lo): contra a rede frouxa, quase nada;
  contra a esticada, o de parar o corpo. Na cena (`JoltNets3D`), a reação da
  rede no link guiado vai como empurrão de fora (`externalForceMagnitude` e a
  força no link no passo seguinte): o atleta vê como qualquer empurrão - de
  leve, nada; forte, vira físico e a rede o segura. O link físico (o boneco já
  solto) recebe só o impulso, sem contar como empurrão: senão o boneco caído
  encostado na rede lia um empurrão contínuo e não voltava ao padrão.
- **A barreira invisível dos jogadores** (`ObjectNet::barrier` e
  `capsuleBarrier`, `objectNetBarrier3D`, `PhysicsBodyDefinition3D::barrier`):
  placas estáticas no lugar da rede, que só os jogadores sentem. A bola, os
  objetos e as consultas (chão, raios, espaço livre) passam; a bola continua
  batendo só na rede. Cada placa vai de 3 cm para dentro da rede a 12 cm para
  fora (a mão a 10 m/s anda 8 cm por tick: entra menos que a metade e volta
  pelo lado de onde veio). Duas camadas do Jolt (`JoltObjectLayers`):
  - `PlayerBarrier` (o corpo dos bonecos e a cápsula): as laterais inteiras
    (do poste até a parede de trás e do chão ao teto: só até a diagonal, perto
    do canto de trás a placa tinha ~40 cm, a cápsula subia nela e o corpo
    entrava no gol por cima; por fora, acima da diagonal, ele bate numa parede
    no ar), o teto e uma parede reta na barra de trás, da altura da trave;
  - `CapsuleBarrier` (só a cápsula): o fundo inclinado. Ele fica por cima do
    chão de dentro: com o corpo batendo nele, o boneco caído embaixo
    levantava com a cabeça na placa e caía de novo (141 trocas de modo em
    10 s). O corpo, ali, afunda o alto da rede até a parede reta. Por trás,
    para onde a rede toca o chão. Um meio-fio de 35 cm no lugar da parede
    deixava o corpo físico empurrando passar o pé por cima e sair por baixo
    da rede, e por trás ele tropeçava nele.
  O atleta vê a barreira como parede (`ragdollPoseWorldContacts`). Por
  dentro, contra o fundo, a cápsula para com o eixo a 0,38 m da placa
  inclinada: a parte de cima dela é redonda (raio 0,34), e a cabeça fica a
  ~20 cm da rede (inclinado para frente, encosta). Substituiu o núcleo da
  cápsula na rede e as linhas no chão (com elas, de fora pela lateral, os
  membros passavam até 57 cm; no carrinho, 86 cm além da barra de trás). Os
  bonecos sem cápsula (os não controlados) batem nas placas do corpo no modo
  físico; no padrão (guiado) atravessam: medido em 25/10, o boneco guiado
  andando a 3 m/s contra o lado passou o link mais fundo 1,06 m da placa (o
  físico para 5 cm antes dela). Só o atleta lendo o contato da pose os
  pararia.
- **No objeto solto**, o impulso e o empurrão de posição (sem ele, o Jolt,
  que andou o tick sem a rede, deixava o corpo rápido fundo nela). Um corpo
  rápido perto (a caixa atirada, a perna no chute) pede até 16 subpassos.
- **O sono:** parada (nenhum nó sai 3 mm do lugar em 0,3 s), ela não custa
  nada e segura o que está apoiado nela. Acorda com um corpo rápido (acima de
  0,25 m/s) a menos do raio dele, mais o que ele anda no tick, de uma célula:
  a mesma conta do contato. Antes bastava entrar na caixa da rede (o gol
  inteiro), e o jogador dentro do gol ou encostado na barreira, sem tocar
  nela, a mantinha acordada (0,1-0,25 ms por tick); agora ela dorme (0,004).
  Dormindo, só contam o corpo com massa a menos de 1 cm (mais o que ele anda
  no tick) de uma célula, que ela segura parada, e o rápido, que a acorda; o
  boneco parado dentro do gol sem encostar (o goleiro) contava como apoiado e
  custava 0,06 ms por tick (a lista do tick e os subpassos para nada); agora
  0,009.
- **O custo:** os fios numa passada de Gauss-Seidel reordenada por níveis
  (cada nó vê os seus fios na mesma ordem: o resultado é o mesmo bit a bit, e
  os fios de um nível não dividem nós, então o processador adianta vários); o
  amortecimento dentro da mesma passada; os pares nó-tubo fixos, e a lista dos
  perto refeita só quando os nós já podem ter andado 5 cm; os candidatos
  numa passada só pelas células contra a caixa de todos os corpos perto (a
  lista curta que cada membro testa; o boneco tem ~20 e cada um passava por
  todos os nós e células), com as caixas das células guardadas enquanto ela
  dorme; a célula cujo plano está mais longe que o alcance do membro sai antes
  da conta exata do eixo contra o triângulo; os laços só nos nós soltos; o
  cabo com 1 passada a mais. Essas otimizações dão o mesmo resultado bit a bit
  (a impressão digital do `MatterNetBenchmark` e da grade forte não muda).
  Sem desvio no fio frouxo, ou em duas passadas por nível, os fios ficaram
  mais lentos (metade dos fios está frouxa e pular é mais barato); SIMD não
  compensa (a troca de arranjo dos nós custa o que a conta economiza).

Na cena (`PhysicsScene3D::createNet`, `JoltNets3D.cpp`), ela anda nos subpassos
da bola: acoplada, 16 por tick no impacto (bola acima de 8 m/s), 8 até 3 m/s e
4 com a bola devagar; sozinha e acordada, 4. (Até 26/10, com uma bola rolando
longe da rede, que pede 16 subpassos à cena, a rede sozinha e acordada andava
nos 16: o `stride` contava só os subpassos que as redes pediam. Depois do chute
que volta pela boca, a rede assentando custava 4 vezes mais.) O `stride` sempre
divide os subpassos do tick (uma bola com 18 e a rede pedindo 4 dão 6 passos de
3), para a rede andar o tick inteiro. Os corpos do Jolt perto viram
proxies e os dinâmicos levam o impulso de volta. (No impacto a 8, bolas
voltavam para fora do gol; na cauda a 2, a rede balançava o dobro.)

**O custo medido** (`MatterNetBenchmark`, Ryzen 5 3600, tick de 120 Hz; a rede
dormindo custa 0):

| cenário | antes (só a malha de 12 cm) | agora (26/10, o contato físico da bola) |
|---|---|---|
| montar a cena (assentar a rede) | 218–242 ms | 14 ms |
| chute de 25 m/s no meio: ms por tick acordada, média / máx | 0,80 / 2,3 | 0,10 / 0,66 |
| o lance inteiro (até a rede dormir) | ~333 ms | 30 ms |
| chute de 35 m/s no canto | 0,83 / 2,1 | 0,11 / 0,78 |
| rajada de 6 bolas | 1,01 / 3,6 | 0,21 / 1,2 |
| caixa atirada | 0,66 / 1,2 | 0,10 / 0,22 |
| duas traves, um chute em cada | 1,89 / 4,3 | 0,17 / 1,4 |

(Com o contato físico da bola, o lance acaba antes e a média por tick sobe; o
custo do lance inteiro, a média × os ticks, ficou igual ou menor: a rajada,
0,13 × 426 ticks antes e 0,21 × 240 agora.) A rede parada: o cabo do alto com
flecha de 7,4 mm, o teto 78 mm abaixo do plano e o pano virando 95° no cabo, no
meio do vão (com o cabo a 0,5% e as folgas de 5%: 11,0 mm, 108 mm e 99,5°). A
dobra é a folga do teto: só ela em 5% dá 100°; o cabo a 0,5% dá 94,9°.

Com o boneco guiado (as linhas do boneco no `MatterNetBenchmark`, 25/10, ms
por tick antes → depois da otimização acima, o mesmo resultado bit a bit):
parado no gol sem encostar 0,060 → 0,009; encostado no fundo balançando de
leve 0,21 → 0,145; correndo a 9 m/s rente ao fundo, por dentro, 0,375 →
0,287. Os casos da bola não mudam (0,08 o chute no meio). Com o atleta de
verdade e a barreira (`MatterAthleteProbe`, últimos 5 s): empurrando o fundo
0,173 → 0,112, andando contra ele 0,106, de carrinho 0,129; correndo e
voltando, a rede dorme 88% do tempo (0,020).

O chute no meio estufa 0,75 m e a rede dorme 3,95 s depois, como na malha de
12 cm. Numa grade de 27 chutes (18 a 32 m/s, três alturas, três lados), a
rede estufa 0,71 m na média (igual) e 4 bolas voltam para fora do gol (6 na
malha de 12 cm). O teto, no chute a meia altura, anda mais que na malha de 12
cm (o nó que mais anda, 17 cm contra 5); nos chutes altos, ~35% menos na
média (a célula grossa não faz um bolso menor que 48 cm). Uso:
`MatterNetBenchmark [parametro valor]...` (`malha_fisica 1` compara com a
física na malha de verdade). Cada linha termina com a impressão digital da
cena até ali (os nós e os corpos a cada tick, entre colchetes): uma
otimização que não muda nada a deixa igual.

## A tela "Objetos"

Menu principal → **Objetos**.

- **Catálogo:** as receitas da pasta.
- **Aba Propriedades:**
  - os parâmetros do tipo, em grade, agrupados (Forma, Física, Textura,
    Cores);
  - o objeto é gerado de novo a cada mudança;
  - **Salvar receita** grava o JSON e **Desfazer** relê o arquivo.
- **Aba Teste físico:**
  - escolha o roteiro e os ajustes dele;
  - o ritmo pode ser 1×, ½× ou ¼×;
  - **Rodar/Reiniciar**, **Pausar** e **Parar**;
  - as medidas aparecem ao vivo;
  - cada corpo tem uma sombra no chão (o contorno de cima), que mostra a
    altura;
  - mudar o objeto com o roteiro rodando recomeça o roteiro.
- **Visualizador:**
  - o mesmo renderizador da inspeção de jogador;
  - vistas (frente, lados, costas, cima, 3/4) e **Recentrar**;
  - modos Sólido, Sólido com arestas, Arestas, Normais, Argila e Textura (a
    pixel art inteira, ampliada sem borrar);
  - opções colisor, grade e ortográfica;
  - **Salvar imagem** grava em `inspecao/capturas/`.
- **Mouse:** o botão esquerdo gira, o direito ou o do meio move, a roda
  aproxima e o duplo clique recentra.
- **A trave** (um objeto com rede, `NetLabView` + `ObjectNetLab3D`): o
  visualizador mostra a cena dela (o campo, a trave e a rede assentada) já nas
  Propriedades, e a aba **Trave** substitui o teste físico:
  - **Chute:** velocidade, distância, ângulo, mira (lado e altura), efeito
    e giro por cima; ritmo 1×, ½×, ¼× e 1/8×;
  - **Chutar**, **Chute aleatório**, **Atirar caixa** (o primeiro objeto
    sem rede), **Pausar**, **Limpar** e **Reassentar**;
  - **medidas:** velocidade na linha, gol, quanto a rede estufou, quando
    parou, o fio mais esticado e o custo da rede por tick;
  - mexer num parâmetro reassenta a rede 0,3 s depois.
- **No laboratório** (`Workbench/Laboratory/GoalRuntime.cpp`): a trave fica
  no mapa, no lugar do arco e da viga (que foram para x = 22), com a linha do
  gol em y = -18 e a boca para o centro (o nascimento do jogador), ~11 m
  livres na frente. O corpo fixo com os tubos, a barreira dos jogadores (dois
  corpos estáticos na pose da trave), a rede na cena do laboratório
  (assentada ao criar, ~20 ms) e o desenho no Vulkan: a trave como objeto
  procedural e os fios como malha com esqueleto (`objectNetRenderData3D`: um
  tubo de 3 lados por fio, os pesos de 4 nós; `objectNetSkin3D`: a matriz de
  cada nó é a translação de agora menos o repouso), interpolada entre os
  passos e com o quadro anterior para o TAA. **A sombra dos fios** (26/10): o
  fio de milímetros, mais fino que o texel da sombra (~2 cm na primeira
  cascata), virava pontilhado piscando. No passe de sombra ele engrossa até
  ~0,7 texel da cascata (`MeshRender3D::thinShadowRadius`), e um pipeline
  próprio (`scene3d_mesh_shadow_thin.frag`) deixa só a fração do fio de verdade
  bloquear a luz (a metade dela: a sombra da rede é sutil), sorteada por um
  hash da posição no mundo. O filtro a junta numa linha fraca e macia. O fio
  desenhado tem 6 mm: em 1080p, 1 pixel a ~4,5 m e 0,5 px a 11 m. Sem o TAA (o perfil Desempenho, o padrão), o fio mais fino
  que um pixel pode piscar e falhar; o Equilibrado liga o TAA, que deve
  suavizar (não visto). Ela não entra no menu de props (Q): parada no chão e com a rede,
  não é um prop solto.

## Um tipo novo (a trave, a rede...)

1. Crie um gerador em `src/Engine/Modeling/Objects/<Tipo>.cpp`. Ele declara
   os parâmetros (`ObjectParameterSpec`) e gera a malha, a pixel art e a
   física (`ObjectCollider`; `fixed` para o que fica fincado). Registre o
   tipo em `objectTypes()`.
2. Crie a receita em `assets/objects/<id>.json`.
3. Escreva os testes do núcleo em `tests/Modeling/ObjectModelingTests.cpp`:
   a malha fechada, o volume, o mapa, o determinismo e os limites dos
   parâmetros.
4. Escreva os testes de física em `tests/ObjectPhysicsTests.cpp`, num roteiro
   do laboratório que faça sentido para o tipo.
5. Se nenhum roteiro servir, crie um em `ObjectPhysicsLab3D`. A trave, por
   exemplo, pede um "chute na trave": a bola contra um objeto fixo.

Uma peça que não for caixa (o poste redondo da trave, a rede) precisa de um
construtor de malha novo em `ObjectMesh`, como o cilindro com mapa ou a
malha de fios. Um corpo com várias formas usa mais de um `ObjectCollider`.

## Testes da trave

- **`MatterEngine.NetSolver`** (`tests/Physics/NetSolverTests.cpp`): a rede sem
  o Jolt; parada não estica, a bola a 35 m/s em quatro pontos não atravessa,
  determinismo, a bola apoiada continua apoiada com a rede dormindo, a
  recuperação de penetração não passa o nó para o outro lado do tubo,
  repetir a passada num fio isolado não o endurece (o alongamento do XPBD,
  também além do limite de 15%), e o fio que afrouxa no subpasso devolve o
  que puxou.
- **`MatterEngine.ObjectModeling`**: a trave mede o vão da regra, os pés no
  chão, cada tubo fechado e para fora, a rede presa, o cabo esticado, o peso;
  a malha desenhada sai da física parada (cada nó a menos de 0,1 mm), as
  células cobrem a mesma área com 1 ou 2 malhas, e com 1 cada nó desenhado é
  um nó da física.
- **`MatterEngine.ObjectPhysics`**: com o Jolt e a bola inflada: a rede dormindo
  não custa; o chute no meio é gol, estufa a rede e para dentro do gol; na
  trave a bola volta; no teto ela fica; a grade de 30 chutes não passa da
  rede; a caixa atirada não atravessa; o giro de topspin e o em espiral se
  mantêm no contato (sem freio de giro); a rede sozinha e acordada, com uma
  bola rolando longe, anda 4 subpassos por tick, e com uma bola de 18
  subpassos os dela dividem os 18. A pele da rede na GPU (a conta do
  `scene3d_mesh.vert` na CPU) põe cada ponta de fio no nó desenhado (erro de
  0,03 mm). A rede forte: o boneco físico atirado a 9 m/s, a caixa a 30 m/s,
  a caixa de 100 kg a 15 m/s e a bola a 40 m/s não passam; o boneco guiado
  recebe o empurrão (0 parado no gol, 0,09 N s de leve, ~250 N s contra a rede
  esticada), o
  físico encostado nela, nenhum; e o custo dela com o boneco
  parado no gol (abaixo de 0,03 ms por tick), empurrando e correndo pelas
  laterais.
  A barreira: a cápsula contra o fundo 4 s e de lado 4 s para na placa do
  fundo (o eixo a 0,376 m dela), desliza até o canto e para na do lado; o
  chute no meio passa por ela e estufa a rede igual (0,78 m); o boneco físico
  atirado a 9 m/s contra o lado para na placa. Os três chutes de perto, na
  diagonal, no canto do poste (30-45 m/s, 4 m) que saíam pela rede do lado
  ficam dentro, com o fio no máximo a 5%.
- **`MatterAthleteProbe --cenario rede_*`** (a sonda, fora das suítes): o atleta
  no modo padrão contra a trave com rede. `rede_corre`: sprint para o fundo,
  solta aos 2 s. `rede_anda` e `rede_empurra`: andando e correndo com a tecla
  segura 10 s. `rede_canto`: na diagonal para o canto. `rede_atras`: a trave
  de costas, a rede do fundo por fora. `parede_empurra` é a mesma tecla contra
  a parede, como referência. `rede_max`: o sprint máximo (o Shift duplo,
  `Keys::maxSprint`) para o fundo. `rede_lado_fora` e `rede_lado_max`: de fora
  contra a rede do lado, correndo e no sprint máximo. `rede_carrinho`: de
  carrinho para o fundo (`XCARRINHO` = quando agacha). `rede_lado_fundo`,
  `rede_lado_canto`, `rede_lado_canto_max`, `rede_lado_diag` e
  `rede_lado_carrinho`: de fora perto do canto de trás (a 1,4, 1,6 e 1,8 m do
  poste, no sprint máximo, na diagonal para o canto e de carrinho). `XDESLOCA` desloca a
  largada (a sensibilidade caótica). A linha REDE julga pelo link mais fundo
  (não pela pelve: na lateral a pelve parava e as pernas passavam) e pela
  cápsula; nos roteiros de fora, por um link ou a cápsula dentro do gol
  (com a cunha debaixo do fundo) e pela cápsula subindo. Mostra a rede nos
  últimos 5 s (dormindo, tocada, ms por tick). Medido em 25/10 com a
  barreira, os 14 roteiros e a parede com a largada deslocada em 0, 3, 7 e
  13 mm (60 rodadas): nada entrou nem saiu, a cápsula nunca subiu. Trocas de
  modo em 10 s com a tecla segura: 0 a 4 pela frente (uma ida ao físico com
  volta inteira; com o núcleo eram 16 a 20), 1 a 7 por fora pela lateral, 7
  a 10 por trás, 1 na parede. O sprint máximo contra o fundo vira físico e
  volta animado, sem cair; só os carrinhos terminam no chão.
- **`MatterNetBenchmark --grade-forte`**: 480 chutes de 30 a 45 m/s, de 4 a
  16 m, pela largura toda, do chão ao canto do teto, de ângulo dos dois lados
  e com efeito, e 288 de muito perto (2 e 3 m, 45 e 60°, 35 a 45 m/s, nas duas
  redes do lado); aponta cada bola que esteve dentro do gol e saiu pela rede.
  Antes das correções do contato, 3 dos 480 saíam (catapultados a 130 m) e 10
  dos 288 (a rede voltando por cima da bola); agora nenhum. Desde 26/10, 816
  chutes: os 768 e 24 lances de duas bolas (a segunda na rede ainda estufada), com a
  travessia julgada também pela paridade do segmento da bola até a boca
  contra as células, independente do contato (com desempate nas arestas e
  vértices comuns, conferido num quadrado e num leque antes da grade), e
  qualquer valor não finito na bola ou na rede reprovando; e onde os gols
  terminam.
  `XCHUTE="velocidade distância mira_lado mira_altura ângulo efeito_lado
  efeito_cima"` imprime um lance tick a tick.
- **`MatterEngine.NetSolver`** também: a cápsula fina (o braço guiado, o
  cinemático) encostando não atravessa; a perna de 6 kg a 15 e 30 m/s é
  parada; o guiado relata 0,02 N s contra a rede frouxa e o de parar o braço
  contra a esticada.
- **`MatterEngine.ObjectInspector`**: a trave pela tela, com as fotos do impacto
  de lado e de trás em câmera lenta (`MATTERENGINE_UI_SNAPSHOT`).

## Testes

- **`MatterEngine.ObjectModeling`**
  (`tests/Modeling/ObjectModelingTests.cpp`) é linkado só ao núcleo. Cobre a
  leitura, a gravação e os erros da receita, a caixa (fechada, o volume, o
  mapa dentro do atlas, a densidade por face), o determinismo pela semente,
  a caixa comprida e o `PixelCanvas`.
- **`MatterEngine.ObjectPhysics`** (`tests/ObjectPhysicsTests.cpp`) cobre:
  - a malha da GPU, com o mesmo mapa e a mesma posição da modelagem em cada
    canto;
  - o corpo, com massa, material e colisor;
  - o laboratório com o Jolt de verdade:
    - a queda para numa face e quica pouco (madeira no concreto);
    - na rampa de 40° a caixa escorrega, na de 12° fica;
    - a pilha de 3 fica de pé;
    - a bola a 15 m/s empurra a caixa;
    - o arremesso anda e para.
- **`MatterEngine.ObjectInspector`** (`tests/ObjectInspectorTests.cpp`) roda
  a tela sem janela. Testa:
  - as vistas, os modos e o mouse;
  - os parâmetros ao vivo, salvos numa cópia da receita;
  - todos os roteiros rodando pela tela, pausar e recomeçar.

  Com `MATTERENGINE_UI_SNAPSHOT=<pasta>`, grava as fotos da tela.
