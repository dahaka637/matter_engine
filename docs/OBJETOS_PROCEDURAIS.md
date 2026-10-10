# Objetos procedurais

Os objetos físicos do laboratório não são mais modelos GLB. Cada objeto é
**gerado por código a partir de uma receita**: a malha, a textura em pixel
art e a descrição física (material, massa, colisores). Hoje existe um tipo, a
**caixa de madeira**. A estrutura já foi feita para os próximos, como a trave,
a rede e o que mais o campo pedir.

Estado em 09/10/2026: cadeira, bigorna, barril, caixa e pneu foram removidos
junto com os GLB, os caches `.mecollider` e o cozinhador V-HACD
(`MatterPhysicsAssetCooker`). No menu de spawn ficam a caixa procedural, a bola
de futebol (inflada) e o boneco.

## Arquitetura

```
MatterModeling   src/Engine/Modeling/Objects/   o núcleo: receita, tipos, geradores, pixel art
  (sem engine, sem ImGui, sem SDL; regra no check-architecture)
MatterObjects    src/Engine/Objects/             a ponte com o jogo: malha da GPU, corpo físico, laboratório
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
