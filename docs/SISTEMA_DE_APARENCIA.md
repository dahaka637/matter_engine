# O sistema de aparência (uniforme e corpo)

O plano e as decisões estão no vault (`06_render/plano_sistema_de_aparencia.md`,
Claude + Codex, 26/10/2026). Este documento é o contrato técnico: o que cada
arquivo guarda, as convenções e as provas. Ele cresce etapa por etapa; hoje
cobre a **etapa 1: o template de superfície, a referência canônica e a
exportação de ida e volta**.

## A ideia em uma linha
A pintura mora numa superfície feita para recebê-la (o template, que nasce dos
moldes de costura da roupa), o uniforme é uma receita composta numa textura, e
o shader do jogo só amostra. Nada do sistema de pintura antigo é reaproveitado.

## A cadeia da etapa 1
```
retopo_moldes.py + etapa 01   o molde de cada face (atributo "molde", texto "matter_moldes")
        │                      (a malha aprovada de 25/10 recebeu os rótulos por
        │                       tools/blender/rotular_moldes.py: correspondência exata de faces)
exportar_troca.py             "moldes", "triangleMolde" e "trianglePolygon" na troca
        │                      (importar_troca.py refaz os polígonos: a ida e volta guarda os quads)
tools/blender/superficie.py   o template, pela referência canônica da versão em uso:
        │                      coordenadas de desenho, cartas, regiões, costuras, âncoras, zonas,
        │                      UV0, UV1 e a referência 3D ("surface" + listas por triângulo)
        │                      os arquivos: assets/characters/player_model/superficie/
jogador.troca.json
        │
Engine/Appearance/Surface     lê e confere o template (SurfaceTemplate)
Modeling/Import               a malha da modelagem com os atributos por canto e por face
Rig/GameCharacterExport       a pele do jogo com UV0, UV1, região e zona por vértice
Animation/RagdollCharacter3D  o personagem do jogo com esses dados, cada canto conferido
                               contra o template (o material vem na etapa 3)
```

## As convenções
- **Referência 3D:** o espaço do modelo da troca, em metros, na pose de ligação.
  X aponta para a frente, Y para a esquerda do jogador e Z para cima; o chão
  fica em z = −0,9776. É canônica: guardada na referência da versão
  (`referencia_pN.json`), e não a posição de agora. Um corpo diferente com a
  mesma construção não muda nada do desenho; a posição física acompanha o corpo,
  a de referência não.
- **Coordenada de desenho** (por canto, uma por painel), em metros no pano
  planificado:
  - **olhando a face de fora, com y para cima, x cresce para a direita de quem
    olha.** Na frente da camisa, x cresce para a esquerda do jogador; nas costas,
    para a direita; na manga direita, para a frente; na esquerda, para trás;
  - a origem é o meio da barra do painel (a borda de baixo: a barra da camisa,
    a do punho, a da perna; no meião, a borda da meia);
  - y vai do meio da barra para o meio da borda oposta (a gola e o ombro, a
    cava, o cós, o joelho).
- **O espelho:** um ponto p do lado direito e o espelho dele no esquerdo têm
  coordenadas (x, y) e (−x, y). Os painéis dos dois lados do meio (a frente e as
  costas da camisa) são simétricos por construção: o meio fica em x = 0.
- **UV0 e UV1:** v para baixo (a linha da imagem é v × altura), como o resto do
  motor.
- **As distâncias** são em metros; as densidades, em texels por metro de pano.

## Os IDs (sempre pelo nome; o número é só o índice da lista no arquivo)

### Os painéis (moldes)
| Painel | O que é |
|---|---|
| `camisa_frente` | a frente da camisa, dos dois lados do meio (a tira embaixo do braço inclusa) |
| `camisa_costas` | as costas, dos dois lados do meio |
| `manga_direita`, `manga_esquerda` | as mangas (tubos, cortados embaixo, na coluna que desce da axila) |
| `calcao_frente_direita`, `calcao_costas_direita`, `calcao_frente_esquerda`, `calcao_costas_esquerda` | as quatro partes do calção |
| `recuo_gola`, `recuo_barra`, `recuo_punho_direito`, `recuo_punho_esquerdo`, `recuo_cos`, `recuo_barra_direita`, `recuo_barra_esquerda` | as faixas por dentro de cada abertura (o zíper da barra até a pele; a da barra da camisa encontra a do cós) |
| `meiao_direito`, `meiao_esquerdo` | a perna da borda da meia ao joelho (a pele por baixo da meia; a altura da meia é do desenho) |
| `meiao_borda_direito`, `meiao_borda_esquerdo` | a fileira da perna que encosta na costura com o cabedal da chuteira (os 1 a 4 cm de meia logo acima dela, à vista) |

A tabela dos moldes (`retopo_moldes.MOLDES`) só cresce no fim: o índice de um nome nunca muda.

### As regiões (por face)
| Região | Faces |
|---|---|
| `camisa` | os painéis da camisa (o pano de fora) |
| `camisa_recuo` | as faixas do recuo da camisa |
| `calcao` | os painéis do calção |
| `calcao_recuo` | as faixas do recuo do calção |
| `meiao` | os painéis do meião e da borda da meia |
| `pele` | o resto do corpo (a cabeça, as mãos, os braços, as coxas) |
| `pe_oculto` | a pele dentro da chuteira |
| `chuteira` | as peças da chuteira |

Painel, região, a validade de cada carta e a classificação da identificação
são dados **discretos por face**: nunca se interpolam.

### As cartas (desenhos que atravessam painéis)
Cada carta é a parametrização da união de painéis, unidos por costuras
escolhidas, com o corte declarado. Dentro dela a coordenada é contínua por
construção; a largura no pano é medida (a distorção de cada triângulo), não
prometida. O método e o campo de cada carta vão na troca e no documento do
jogo, e o leitor os exige (nunca supõe). Há dois métodos:
- **campo** (Poisson pelos campos de direção): x e y são as funções cujos
  gradientes mais se parecem com dois campos no plano de cada face. No tronco,
  y sobe e x dá a volta: os aros ficam horizontais em toda a volta. Na
  `sobre_ombro`, x é a posição de um lado ao outro (a lateral no meio e no alto
  do ombro, a "volta" nos lados) e y sobe pela frente, passa pelo ombro e desce
  pelas costas: a listra vertical continua reta nos dois lados. Nos membros, y
  vai ao longo do eixo (para o corpo) e x dá a volta: as faixas do punho, da
  perna e da meia (até a chuteira) ficam perpendiculares ao membro;
- **isométrica** (LSCM e depois ARAP, orientada por marcos): a travessia da cava,
  um desenho local.

A planificação isométrica de um tubo cortado foi descartada para as cartas que
dão a volta: ela guarda o tamanho dos quadrados, mas a orientação gira (o aro
saía inclinado nas costas, a listra da `sobre_ombro` 40° torta, e no meião o
"para cima" desviava até 14° da canela).

| Carta | Painéis | Unidos por | Corte | Método |
|---|---|---|---|---|
| `volta_tronco` | frente e costas da camisa | os lados | os ombros e o meio das costas | campo (tronco) |
| `sobre_ombro` | frente e costas da camisa | os ombros | os lados e o meio das costas | campo (lateral) |
| `ombro_manga_direita`, `ombro_manga_esquerda` | a manga e a frente e as costas até 12 cm da cava | a cava e o ombro | a coluna de baixo da manga e o lado abaixo da axila | isométrica |
| `volta_manga_direita`, `volta_manga_esquerda` | a manga | — | a coluna de baixo | campo (membro) |
| `volta_perna_direita`, `volta_perna_esquerda` | a frente e as costas de uma perna do calção | o lado de fora | a costura de dentro | campo (membro) |
| `volta_meiao_direito`, `volta_meiao_esquerdo` | o meião e a borda da meia | a borda (`borda_meiao_*`) | a linha de trás, até a costura com o cabedal | campo (membro) |

As cartas `volta_tronco` e `sobre_ombro` são simétricas (o corte é o meio das
costas): um desenho simétrico fecha no corte; um assimétrico mostra a emenda ali.

**Os painéis** são planificados perto da isometria (LSCM e depois ARAP), no
referencial do contrato (origem no meio da barra, y para o marco de cima). A
**borda da meia** é a exceção:
- **o que ela é:** uma fileira só de triângulos finos, entre a costura com o
  cabedal e o meião: a fileira inteira que encosta na costura, o vértice da
  costura no ponto mais alto do cabedal inclusive (senão o anel abre num disco);
- **por que outro método:** o LSCM e o ARAP viravam as lascas dela (dois pares se
  sobrepunham), e um retângulo de Tutte a achatava 4× na vertical (o xadrez saía
  em listras);
- **como é planificada:** desdobrada triângulo a triângulo pela aresta comum,
  cada um com os comprimentos exatos do corpo. O estiramento é 1,00, e a peça sai
  em arco (o cone desenrolado);
- **no atlas:** ela fica à vista e tem a densidade cheia, como o resto do pano;
  só as faixas do recuo têm a metade;
- **na carta:** a `volta_meiao` passa por ela, unida pela costura `borda_meiao_*`
  (o mesmo tecido). O desenho em volta da perna desce até a chuteira.

**Os cortes dos tubos:**
- na manga, a coluna da construção: do vértice da axila (frente, costas e manga)
  reto pela grade de quads até a barra, a coluna 0 dos anéis;
- no meião e nas faixas do recuo, um caminho escolhido uma vez pela direção
  (atrás da perna, por dentro, atrás do tronco) e **gravado** na referência
  (`cuts`). Escolhido de novo, ele podia pular de coluna, e os triângulos
  vizinhos trocariam de lado (uma volta inteira na coordenada);
- na borda da meia, a aresta da fileira que continua o corte do meião até a
  costura com o cabedal (a carta da volta abre nos dois numa linha só), também
  gravada.

### As costuras e as aberturas
- **Costuras** (entre dois painéis; a continuidade da pintura é permitida):
  `lado_*`, `ombro_*`, `cava_frente_*`, `cava_costas_*` (camisa),
  `calcao_lado_*`, `calcao_dentro_*`, `calcao_meio_frente`, `calcao_meio_costas`
  (calção), `borda_meiao_*` (o meião e a borda da meia), com `*` =
  `direito`/`esquerdo`. Uma costura com o mesmo nome em
  mais de um par de painéis é uma só (a gola, por exemplo, passa da frente para
  as costas).
- **Aberturas** (painel × faixa do recuo; a pintura não continua):
  `gola`, `barra`, `punho_*`, `cos`, `barra_perna_*`.
- **Junções com a pele e entre faixas**: não continuam.
- Cada costura guarda as arestas (pares de vértices), os dois painéis e a
  permissão de continuidade (adjacência geométrica não é permissão).
- **A fronteira é completa:** toda aresta entre dois rótulos diferentes (o
  painel, ou a região de um triângulo sem painel) está em exatamente uma
  costura, abertura ou junção, o fechamento de cada anel inclusive. O leitor
  confere a igualdade, não só que cada aresta declarada exista: uma fronteira
  esquecida é erro.

### As âncoras e as zonas de identificação
As âncoras são pontos nomeados, cada um com painel, posição, direção de "cima"
e tamanho máximo. As regras (o marco da construção e o deslocamento) estão em
`definicoes.json`; os valores resolvidos, na referência. As zonas de
identificação são retângulos canônicos em coordenada de desenho:
| Zona | Painel | Retângulo |
|---|---|---|
| `costas` | `camisa_costas` | 40 cm de largura no meio, do topo a 6 cm abaixo da gola até 45 cm abaixo dele |
| `frente` | `camisa_frente` | 20 × 20 cm no meio, centrado 14,4 cm abaixo do ponto da gola |
| `calcao` | `calcao_frente_esquerda` | 14 × 14 cm, centrado 6,4 cm acima da barra |

**A zona não garante o tecido.** Os cantos do retângulo podem cair fora do
contorno do painel: na zona das costas, os quatro cantos ficam fora; na do
calção, os dois de baixo. Os tamanhos máximos das âncoras de hoje cabem. Antes
de liberar o posicionamento no editor (etapa 4), a conferência tem que ser a
cobertura real da identificação sobre o painel, com o contorno e a sombra. "Está
dentro da zona", sozinho, deixaria voltar o número cortado.

## Os arquivos (`assets/characters/player_model/superficie/`)
- **`definicoes.json`**: as regras fixas e as versões em uso. Contém:
  - os painéis e as regiões;
  - as cartas, com o método e o campo;
  - as âncoras e as zonas, como marco mais deslocamento;
  - a regra do meião e o layout da identificação;
  - `parametrization` e `storage`, as versões que o gerador aplica.
- **`referencia_pN.json`**: a referência canônica da versão N. Contém:
  - por polígono da construção, chaveado pelo conjunto dos vértices dele:
    - o painel, a região e a zona;
    - em cada vértice, a coordenada de desenho e a de cada carta;
  - a posição 3D de cada vértice;
  - os cortes;
  - as âncoras e as zonas resolvidas;
  - a distorção medida.
- **`layout_sN.json`**: o layout do UV0, feito sobre uma referência (ela vai no
  arquivo). Guarda o tamanho de base, a densidade e, por painel, o retângulo no
  atlas e a caixa em coordenada de desenho.

## A troca (o que `superficie.py` acrescenta)
- `"surface"`, com:
  - `schema` = `matter.surface.v1`;
  - `parametrization` (a versão da referência) e `storage` (a versão do layout do UV0);
  - as listas `panels`, `regions`, `charts` (com `method` e `field`), `seams`, `anchors` e `zones`;
  - os layouts `storageLayout` (o tamanho de base, a densidade e o retângulo de cada painel) e `identityLayout` (608×448 e os blocos).
- **Por triângulo:**
  - `trianglePanel`, `triangleRegion` e `triangleIdentity` (índices; −1 = nenhum);
  - `triangleChartMask` (um bit por carta).
- **Por canto:**
  - `triangleDesign` (a coordenada de desenho);
  - `triangleChart` (por carta, só nos triângulos válidos);
  - `triangleKitUv` (o UV0);
  - `triangleIdentityUv` (o UV1; pode passar do bloco, e o material testa o limite por fragmento);
  - `triangleReference` (a posição 3D canônica, nos triângulos com painel).

## Versões e compatibilidade
- **O modo padrão do gerador só aplica a referência em uso:**
  - a coordenada de cada canto vem do polígono dele, e o painel e a região também
    (o molde de cada face da roupa tem que existir e bater com a referência);
  - a posição 3D é a da referência.

  Um corpo diferente com a mesma construção (os mesmos polígonos e vértices) sai
  com o template igual bit a bit. Outra topologia para o gerador, que pede uma
  referência nova.
- **`--nova-referencia`** planifica de novo pela geometria de agora (com os cortes
  da anterior, quando a topologia é a mesma) e grava `referencia_p{N+1}`. Ela
  exige `--novo-layout` junto: um layout é de uma referência só.
- **`--novo-layout`** grava `layout_s{M+1}`.
- **A gravação:** os arquivos novos e as versões em `definicoes.json` só são
  gravados depois de tudo conferido, a injetividade inclusive. As versões
  anteriores ficam.
- **Os limites do layout:** um painel pode crescer até 8 mm além da caixa gravada
  (metade da folga entre os painéis). Além disso o gerador para e pede um layout
  novo; ele nunca reempacota sozinho.
- **Mudança de painéis, regiões ou cartas** é outra `parametrization`.

## As provas da etapa 1 (todas passando em 26/10)
- **A malha aprovada fica:** a troca com o template tem as mesmas posições, triângulos,
  partes, materiais, juntas e UV do atlas (bit a bit) que a de antes do sistema de
  aparência, e os mesmos pesos até 1,4e-8 (o arredondamento do .blend).
- **O leitor rejeita** (`MatterAppearanceTests`), cada caso com mensagem própria:
  - campo ausente, nome desconhecido, coordenada inválida;
  - UV0 fora do atlas, bloco fora da textura de identificação;
  - carta unindo o que não é costura, costura com uma aresta que não separa os lados dela;
  - fronteira que nenhuma costura declara;
  - referência 3D ausente num triângulo de painel;
  - carta sem o método, carta do campo sem o campo, carta isométrica com campo;
  - lista mais curta que os triângulos.
- **O carregador do jogo** (`RagdollCharacter3D`) confere cada canto da pele contra
  o template:
  - a região e a zona;
  - o UV0, presente só onde há painel, e o UV1, só onde há zona;
  - a faixa, antes de estreitar os tipos.

  `MatterPlayerModelTests` mostra as recusas: atributo faltando, região trocada,
  zona onde não há, UV0 e UV1 divergentes, região fora da faixa, UV0 fora do
  atlas, zona sem UV1 e superfície sem template.
- **Os atributos chegam** (a exportação): passam pela triangulação e pela separação dos
  vértices da pele do jogo. Em cada canto de cada triângulo, a origem, a região, a zona,
  o UV0 e o UV1 relidos são os do template.
- **A reindexação não muda nada:** com os triângulos permutados, o template é o mesmo
  por nome; o documento gravado de volta lê igual, inclusive o método e o campo das cartas.
- **A injetividade exata, par a par de triângulos** (só a borda compartilhada é permitida):
  - o método é a área da interseção de cada par candidato (recorte de Sutherland–Hodgman),
    com a tolerância de 1e-6 da área do menor; os triângulos degenerados são recusados;
  - zero sobreposição no desenho de cada painel, em cada carta e no UV0 inteiro, conferida
    no gerador antes de gravar e no teste do arquivo instalado;
  - o detector pega até um triângulo a 1% do vizinho do mesmo painel, menor que um texel de
    2048, que a amostragem por grade de antes perdia; pega também o triângulo degenerado;
  - a troca de antes da correção tinha 287 texels sobrepostos no meião, nos pares
    2012/2014 e 2569/2571.
- **As cartas são contínuas:** coordenadas iguais nas bordas unidas (até 1e-5 m) e
  nenhum triângulo virado.
- **As cartas de volta seguem o eixo:** o ângulo entre o "para cima" da carta e o eixo
  (o vertical no tronco, o osso nos membros), com p95 abaixo de 4°:

  | Carta | média | p95 |
  |---|---:|---:|
  | `volta_tronco` | 0,6° | 2,2° |
  | `volta_manga_*` | 0,7° | 2,9° |
  | `volta_perna_*` | 0,4° | 1,2° |
  | `volta_meiao_*` (com a borda) | 0,1° | 0,4° |
- **A distorção medida** (o estiramento p95 do jacobiano, pela área):

  | | p95 |
  |---|---:|
  | frente, costas, mangas, calção | 1,03–1,09 |
  | meião (ARAP) | 1,10 |
  | borda da meia (desdobrada) | 1,00 |
  | `volta_tronco`, `sobre_ombro` | 1,14 e 1,11 |
  | `ombro_manga` | 1,18 |
  | `volta_manga`, `volta_perna`, `volta_meiao` | 1,21, 1,09 e 1,16 |
- **O espelho é exato:** cada painel da esquerda é o da direita com x trocado de sinal.
- **O UV0** fica no retângulo de cada painel. O layout s1 dá 587 texels/m no atlas
  de 1024 (~6 texels/cm; 12 no de 2048).
- **A referência 3D** de cada canto é a posição do corpo instalado (o de referência).
- **O gerador** (`tools/blender/superficie_teste.py`):
  - **determinismo:** duas referências novas da mesma malha são iguais;
  - **canonicidade:** com a malha 0,5% mais larga e um calombo de 2 mm, o template
    sai igual bit a bit. Isso vale para o desenho, as cartas, as zonas, as regiões,
    o UV0, o UV1, a referência 3D, as âncoras e as costuras;
  - **topologia:** com outra topologia, o gerador para. Uma face da roupa sem o molde também:
    o molde não é restaurado da referência;
  - **a borda da meia:** o desdobramento só aceita a fileira (um leque com vértice de dentro e
    um anel fechado param) e confere os comprimentos;
  - **versões, pelo comando de verdade, numa cópia:**
    - o `--novo-layout` grava o s2 e guarda o s1;
    - uma referência nova sem layout novo para sem gravar nada;
    - as duas juntas gravam p2 e s2;
  - **layout:** com outro layout, a coordenada de desenho é a mesma e uma âncora
    cai no mesmo ponto 3D pelos dois UV0;
  - **ida e volta pelo Blender** (troca → importar → exportar → gerador): o mesmo
    desenho em cada polígono. O Blender roda isolado: sem as preferências e os complementos
    do usuário, sem áudio e com tempo limite.
- **A física não mudou:**
  - a exportação do jogo guardou o perfil físico (`FootballPlayerV1.ragdoll.json`)
    da manhã de 26/10, byte a byte;
  - os testes do atleta dão as mesmas falhas de antes do sistema de aparência,
    iguais até nos números.

## As ferramentas
- `tools/blender/superficie.py`: o gerador (troca → troca), com `--nova-referencia` e
  `--novo-layout`.
- `tools/blender/superficie_teste.py`: as provas dele.
- `tools/blender/superficie_previa.py`: o jogador com o xadrez de 5 cm (painéis ou uma
  carta) e o layout do UV0, para o olho. Ela avalia o xadrez pela coordenada
  interpolada direto na superfície, sem passar pelo atlas: mostra a parametrização,
  mas não prova o caminho pelo UV0 (a imagem podia parecer certa com o UV0
  sobreposto). A prova pelo atlas é da etapa 2, com o compositor: rasterizar no UV0
  e amostrar essa textura no jogador (o comando `molde` do inspetor).
- `tools/blender/rotular_moldes.py`: os moldes de uma construção refeita levados a
  uma malha aprovada, por correspondência exata de faces.

## Etapa 2: os documentos do desenho (26/10)
O contrato das etapas 2 e 3 está no vault (`06_render/plano_sistema_de_aparencia.md`,
revisão 3: o mip pela caixa 2×2 com as margens da ilha). Os arquivos ficam em `assets/appearance/`.

**A convenção de cor (compatibilidade):** `#RRGGBB`/255 são os valores de trabalho; a
mistura, os degradês, a pré-multiplicação, os mips e a quantização (ao mais perto) são
feitos neles, sem conversão sRGB. É o que o shader de hoje lê como linear.

### Os glifos (`fonts/<id>.json`, `matter.glyphs.v1`)
```json
{ "schema": "matter.glyphs.v1", "id": "numeros_bloco",
  "glyphs": { "0": { "advance": 0.598, "d": "M 0 0 L ... Z M ... Z" } } }
```
- O espaço do glifo: x para a direita, y para cima, a linha de base em y = 0 e a
  altura das maiúsculas e dos números em y = 1 (a altura do texto escala tudo).
- `d`: o subconjunto do SVG (M L H V C Q Z, absolutos e relativos), fechado; os furos
  pela regra do texto (padrão `nonzero`, então os furos andam ao contrário).
- `advance`: a largura do glifo, nas mesmas unidades.

### O desenho (`designs/<id>.json`, `matter.design.v1`)
```json
{ "schema": "matter.design.v1", "id": "branco", "name": "Branco",
  "templates": [ { "parametrization": 1, "storage": 1 } ],
  "slots": [ { "id": "base", "name": "Base", "default": "#ececec" } ],
  "layers": [ { "id": "fundo", "target": { "regions": ["camisa"], "panels": [] },
                "space": "panel", "content": { "type": "fill" }, "color": "base" } ],
  "identity": { "font": "numeros_bloco", "color": "detalhes",
                "zones": { "frente": { "height": 0.097, "offset": [0, 0], "spacing": 0.072,
                                       "stretch": 1.0, "rotate": 0 } } } }
```
- **`templates`:** as versões de parametrização e de armazenamento compatíveis.
  Um template de outra versão é recusado.
- **`slots`:** as vagas de cor (o id estável, o nome e a cor padrão).
  - Uma camada que cita uma vaga desconhecida é erro.
  - Uma cor do kit para uma vaga que não existe é erro.
- **A camada:**
  - `target`: regiões e painéis, a união dos dois (vazio é erro);
  - `space`: em que coordenada o conteúdo é avaliado:
    - `panel`: a coordenada de desenho, em metros; a origem é o meio da barra, e y vai para a gola, a cava, o cós e o joelho;
    - `chart:<id>`: a da carta, só nas faces em que ela vale;
    - `reference`: a referência 3D vista de frente, com x = −Y (a direita do jogador para a direita de quem olha) e y = Z;
  - `anchor` (opcional): `{ "anchor": "escudo" }` ou `{ "zone": "frente" }`. A origem passa ao centro da âncora ou da zona, no painel dela, e a camada só vale nesse painel;
  - `transform` (opcional): `offset` [x, y], `scale` [x, y] e `rotate` (graus), aplicados ao conteúdo nessa ordem (escala, giro, deslocamento);
  - `content`:
    - `fill`;
    - `bands`: `axis` (`x` ou `y`), `from` (`base` ou `topo`; com `topo`, os valores contam do maior y do painel, negativos para dentro) e `bands` [[a, b], ...];
    - `stripes`: `period`, `width`, `angle` (graus) e `phase`;
    - `gradient`: `from` [x, y], `to` [x, y] e `colors` [vaga, vaga] (a camada não usa `color`). A interpolação é pré-multiplicada (como no CSS): do transparente ao branco, o meio é branco com metade do alfa, sem escurecer;
    - `path`: `d` e `rule` (`nonzero` ou `evenodd`);
    - `text`: `font`, `text`, `height`, `spacing` (o espaço entre glifos, em fração da altura), `stretch` (a largura) e `align` (`center`, `left` ou `right`);
    - `border`: `seam` (uma abertura do template, como `gola` ou `punho_direito`) e `width`. É a faixa de largura fixa a partir das arestas da abertura, medida no pano do painel de cada face. É assim que se faz a gola V e o punho: a faixa de dentro, `recuo_*`, quase nunca fica à vista. Só vale no espaço `panel` e sem máscara;
  - `mask` (opcional): `{ "d": "...", "rule": "nonzero" }`, no mesmo espaço e com a mesma transformação. A cobertura é a conjunta, por amostra;
  - `color` (a vaga) e `opacity` (0 a 1).
- **`identity`:** o estilo do número por zona.
  - É desenhado na textura de identificação, não no atlas.
  - `offset` conta a partir do centro da zona, em metros do painel.
  - `height`, `spacing`, `stretch` e `rotate` (graus) como no texto.

### O uniforme escolhido (`kits/<id>.json`, `matter.kit.v1`)
```json
{ "schema": "matter.kit.v1", "id": "branco_10", "design": "branco",
  "colors": { "base": "#ececec" }, "identity": { "number": "10" } }
```
As vagas omitidas usam o padrão do desenho. Nesta entrega, a identificação é estática
no kit; na etapa 4, o número e o nome passam a ser do jogador.
