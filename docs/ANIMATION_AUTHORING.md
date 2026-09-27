# Autoria de animações próprias

Atualizado em 27/09/2026. As animações do personagem passam a ser **nossas**,
criadas do zero. Não vêm de captura de movimento, de biblioteca nem de
retarget. O pipeline de retarget (`ANIMATION_RETARGET_PIPELINE.md`) continua
existindo para estudo, mas não é mais a fonte dos clipes de jogo.

As exceções são o **strafe** e o **parado em alerta**: a pedido do usuário,
eles são referências Mixamo retargeteadas, com as correções que o nosso corpo
exige. O parado em alerta preserva tronco, braços e cabeça de `Idle.fbx`; só o
rumo global e o plantio dos pés são adaptados ao motor.

Motivos, em ordem:

1. **Identidade.** Um jogo de futebol com movimento de biblioteca parece
   qualquer outro jogo.
2. **Os clipes importados não cabiam no nosso corpo.** Foram feitos para
   outro esqueleto e outras proporções, e o retarget só transfere direções.
   O resultado era braço dentro do peito, mão dentro da coxa e pé no chão
   por acaso. No jogo, a autocolisão corrigia isso e o jogador via outra
   pose (seção "A pose tem de caber no corpo").
3. **Controle.** Cada decisão de pose fica escrita, com o porquê, e pode ser
   ajustada por parâmetro.

## Metodologia: animação é código

Uma animação é uma **função do tempo que devolve uma pose**, escrita em
Python em `tools/animation/clips/<id>.py`. A pose é descrita em linguagem
anatômica, e nunca em coordenadas cruas de junta:

- coluna, pescoço e cabeça em graus anatômicos: flexão, inclinação e giro;
- braços por anatomia: quanto descem, quanto vão à frente, quanto giram,
  quanto o cotovelo dobra, pronação e punho. O lado espelha sozinho;
- pernas por IK de dois ossos: onde o tornozelo fica, para onde o joelho
  aponta e quanto o pé abre. É o que mantém o pé plantado enquanto a pelve
  se move.

Coordenadas cruas enganam. Alguns eixos do ALS não são espelhados entre os
lados: o swing2 da coxa leva as **duas** pernas para a direita, e o swing2
do braço leva o esquerdo para trás e o direito para a frente. O mapa de cada
eixo sai da geometria do rig (`axis_atlas.py`), não do nome do eixo.

O ciclo de trabalho:

1. **Descrever** a pose e o movimento em `clips/<id>.py`, com constantes
   nomeadas e comentário do porquê de cada escolha.
2. **Gerar e validar** com `build_clips.py`. Todos os quadros passam pelos
   gates abaixo, e clipe reprovado não é gravado.
3. **Olhar a prévia.** Uma folha PNG com a malha real, em quatro vistas e
   vários instantes. Segmento em vermelho é autocolisão.
4. **Aceitação física no motor** com `testIdlePhysicalFidelity` e os demais
   testes de personagem. O corpo simulado precisa reproduzir a pose.
5. **Revisão do usuário** no visualizador de animação e no jogo.

## Ferramentas (`tools/animation/`)

| arquivo | papel |
|---|---|
| `matter_rig.py` | Rig (FK igual à do motor, paridade 0,0003 mm / 0,00003°) e skin com pesos e ossos visuais (dedos) |
| `matter_clip.py` | Leitura, amostragem e escrita de `matter-ragdoll-animation-1` |
| `pose.py` | Linguagem de autoria: `PoseBuilder`, `ArmSpec`, `LegSpec` |
| `gait.py` | Gerador paramétrico de passada: correr, recuar, sprint (`GaitSpec`) e a postura padrão dos braços |
| `jump.py` | Pulos: poses-chave ao longo do voo, interpoladas (`JumpKey`) |
| `validate.py` | Colisores no mundo, folgas de autocolisão (espelho da regra do Jolt), limites e chão |
| `authoring.py` | `ClipDefinition`, amostragem, gates e gravação |
| `preview.py` | Render por software da malha real: vistas e folhas de contato |
| `axis_atlas.py` | Tabela do que cada eixo de cada junta faz com o corpo |
| `build_clips.py` | Gera todos os clipes (ou os pedidos), com prévia opcional |
| `reference_capture.py` | (Blender) Articulações de uma referência FBX/GLB, no nosso referencial e escala |
| `gait_metrics.py` | Mede uma passada - referência ou clipe nosso - com as mesmas regras |
| `clips/*.py` | As animações |

Uso:

```sh
python3 tools/animation/build_clips.py natural_idle --preview-dir /tmp/previas
python3 tools/animation/axis_atlas.py
```

Dependências: Python 3 com `numpy` e `Pillow`. Não precisa de Blender.

## Gates de validação (`authoring.py`)

| gate | tolerância | protege contra |
|---|---|---|
| resíduo do solver | 0,5° | pose pedida que o rig não alcança (e sairia recortada) |
| limites articulares | 0 (antes de qualquer clamp) | clamp silencioso no motor |
| folga de autocolisão | 5 mm, ou a folga de repouso do par se for menor | a física empurrando o membro para fora e mudando a pose no jogo |
| sola no chão durante o apoio | 4 mm | pé enterrado (a física reage com milhares de newtons) ou flutuando |
| sola abaixo do chão em qualquer quadro | 4 mm | ponta do pé raspando no balanço |
| deslize do pé de apoio, **no mundo** | 3 mm | patinação (clipe in-place: o mundo anda para trás na velocidade nominal) |
| velocidade angular de junta | 30 rad/s | salto de pose (o joelho do sprint chega a 20,6 por construção) |
| fechamento do loop | 0,5° e 1 mm | emenda visível |

O clipe gravado leva o relatório inteiro em `authoringReport`. O runtime
exige um `retargetReport` aprovado; clipe autorado o preenche com
`"origin": "authored"` e as métricas de direção zeradas, porque não há fonte
para comparar.

## A pose tem de caber no corpo

O visualizador de animação desenha o clipe **sem física**. O jogo desenha o
**corpo simulado**, levado aos alvos pelos motores e sujeito à autocolisão.
Se a pose não cabe no corpo, a física a corrige e o jogador vê outra pose.

Foi a divergência relatada em 26/09 (braços mais erguidos no jogo que no
visualizador). Três causas, medidas:

1. **Caixas do tronco sobrepostas já no repouso.** Abdomen×UpperChest
   −13,3 mm, UpperChest×Head −8,4 mm, Pelvis×Chest −2,4 mm. A autocolisão
   tentava separá-las a cada passo e entortava o tronco superior ~17°, e os
   braços iam junto. Isso já acontecia no PhysX.
2. **O idle antigo punha o braço dentro da caixa do peito (−22 mm) e a mão
   dentro da coxa (−22 mm).**
3. **Caixa era a forma errada.** Os segmentos do tronco da malha são barris,
   e a quina da caixa segurava o braço longe do corpo.

Correções:

- **Regra de autocolisão** (`buildSelfCollisionFilter`, `JoltRagdoll3D.cpp`,
  espelhada em `validate.py`). Não colidem: pai e filho; pares que já se
  sobrepõem no repouso; e ancestral/descendente a menos de 1 cm no repouso.
  Continuam colidindo Chest×UpperArm, que impede o braço de atravessar o
  peito, e coxa×coxa.
- **Tronco em cápsulas laterais** (`prepare_als_ragdoll.py`). O braço agora
  pende a ~15° da vertical sem tocar o tronco, contra ~33° com as caixas.
- **Gate de folga na autoria e teste físico no motor.** O teste
  `testIdlePhysicalFidelity` mede cada segmento em relação à pelve, físico
  contra alvo, com o personagem controlado e parado. Resultado do
  `natural_idle`: 1,8° no pior segmento. Controle negativo, o mesmo idle
  com os braços colados: braços 10–17° fora do alvo e tronco superior 4,8°,
  e o teste reprova.

## Clipes autorais

| clipe | velocidade | cadência | apoio | notas |
|---|---|---|---|---|
| `alert_idle` Parado em alerta | parado | loop de 10 s | pés plantados | pose original de `Idle.fbx`, com rumo e contato adaptados |
| `jog` Corrida leve (papel `walk`) | 3,0 m/s | 164 passos/min | 26%, voo 33% | pisa perto de baixo do corpo, joelho em mola |
| `run_backward` Corrida de costas | 2,9 m/s | 189 passos/min | 23%, voo 43% | a ponta toca atrás com o joelho a ~60° |
| `sprint_backward` Sprint de costas | 4,4 m/s | 216 passos/min | 21%, voo 45% | recuo com ritmo: pelve mais baixa, braços mais amplos |
| `sprint` Sprint | 7,5 m/s | 258 passos/min | 23%, voo 53% | antepé, joelho alto, braço bombeando |
| `jog_strafe_left/right` Corrida de lado | 2,9 / 2,4 m/s | 171 / 164 passos/min | ~30% | Jog Strafe retargeteado, passada cruzada |
| `sprint_strafe_left/right` Sprint de lado | 4,1 / 3,4 m/s | 240 / 229 passos/min | ~30% | o mesmo movimento, cadência 1,4× |
| `jump_standing` / `_forward` / `_backward` / `_left` / `_right` | — | voo | — | tocados pela fase do voo (ver "Pulos") |

As velocidades da cápsula saem dos próprios clipes (`nominalSpeedMetersPerSecond`),
lidas em `loadAnimationCatalog`. A primeira versão tinha uma caminhada
rápida (2,4 m/s) e uma caminhada de costas (1,9 m/s); o usuário pediu uma
corridinha como passada padrão e correr de costas, e as duas foram
substituídas.

### `natural_idle` (referência inativa)

4 s em loop. Braços a ~15° da vertical, palmas para as coxas e polegar à
frente (pronação 10°), cotovelos levemente dobrados (16°) e joelhos soltos
(pelve 1,2 cm abaixo do bind). Pés um pouco além da largura do quadril, com
as pontas abertas 7°. Movimento:

- respiração: o peito estende ~1°, os ombros abrem e a cabeça sobe;
- transferência de peso: a pelve desliza 1 cm e o quadril sem apoio cai
  1,2°. A coluna compensa e os pés ficam parados por IK (deslize medido de
  0,1 mm);
- a cabeça deriva ±2°, fora de fase;
- pêndulo sutil dos braços, atrasado em relação ao balanço.

Este clipe autoral permanece como exemplo do pipeline, mas não pertence mais
ao manifesto do jogador e não toca no runtime.

### `alert_idle` (Parado em alerta)

Única pose parada do personagem, ativa desde o surgimento. É um retarget
direto de `Idle.fbx`, preservando a respiração, o tronco inclinado, a cabeça,
os braços, os joelhos dobrados e a base escalonada com o pé esquerdo à frente.
O clipe inteiro é girado rigidamente para o peito olhar para a câmera; por isso
a pelve conserva cerca de 22° de abertura. Os dois pés ficam planos e
plantados no mundo por IK, a única correção interna necessária para o contato
físico.

### Passadas (`gait.py`)

Corrida leve, corrida de costas e sprint saem do mesmo gerador. A passada é descrita pelo
que se mede em quem anda ou corre: velocidade, cadência, fração de apoio,
largura da base, altura do passo, oscilação da pelve e balanço dos braços.

- **Pés.** No apoio, a ponta do pé fica parada no mundo. No clipe in-place,
  ela anda para trás exatamente na velocidade nominal, e o pé gira sobre
  ela: no fim do apoio para a frente (o calcanhar sobe), no começo para o
  recuo (a ponta toca primeiro). No balanço, o tornozelo percorre um
  Hermite com as velocidades das pontas casadas, e a sola nunca desce abaixo
  do chão. O contato é 1 só com o pé chato e parado, que é onde o runtime
  trava o pé.
- **Tornozelo com dois eixos.** O ALS não tem o terceiro eixo no tornozelo.
  O pé é resolvido pela sola (o vetor normal decide o contato), e a direção
  da ponta pode desviar alguns graus. No balanço da corrida o pé acompanha
  a canela, que fica quase horizontal na recuperação.
- **Pelve, tronco e braços.** A pelve sobe e desce (na corrida, desce no
  apoio), balança para o pé de apoio, gira com a perna que avança e o
  quadril sem apoio cai. Os ombros giram com os braços (o braço que vai à
  frente leva o ombro), contra a pelve, e a cabeça segura o olhar. Os braços
  balançam contra as pernas, com o cotovelo abrindo atrás; é o que tira a
  mão do caminho do quadril.
- **Recuperação.** No começo do balanço o pé fica para trás (o calcanhar
  sobe) antes de a perna vir para a frente; sem isso a coxa sobe cedo e alto
  demais.

O que os números ensinaram, e ficou nos parâmetros:

- No sprint, é a pelve inclinada (9°) que deixa o quadril estender na saída
  do pé: o limite do rig é 30° de extensão em relação à pelve. Pelve mais
  em pé piorava.
- **Postura padrão dos braços** (`RUNNING_ARM_*` em `gait.py`), a mesma em
  todas as passadas: braço junto ao corpo, 16° para fora, antebraço na linha
  da passada e 3° para dentro, cotovelo quase constante (trote 90° ± 10°,
  sprint 82° ± 12°; o balanço vem do ombro), punho quase neutro e mão
  relaxada (os dedos são da skin, ver "Mãos"). Antes eram 25–35° para fora e
  o cotovelo abria e fechava 44°; o usuário achou os braços "muito
  arqueados, abertos" e, no jogo, moles. O limite é físico: com menos de 16°
  o braço encosta no colisor do peito, e girado mais para dentro a mão
  encosta na pelve (o colisor dela é quase uma esfera de 14 cm de raio).
- **O corpo físico tem de acompanhar, não só o clipe caber.** No sprint, com
  o cotovelo da referência (73–117°), o braço atrasa um pouco na física, a
  mão cola no quadril no balanço para trás e engancha na outra mão: o
  antebraço travou a 150° do alvo, com o clipe passando em todos os gates.
  Abrir o cotovelo atrás (62°) resolveu. O mesmo cotovelo funciona na
  corrida leve, mais lenta. Por isso o teste físico existe.

### Inspiração em referências, sem cópia

O usuário traz animações de referência (Mixamo) para inspirar o estilo. Elas
ficam em `assets/animations/source/mixamo/study/` e **não viram clipe**:

1. `reference_capture.py` (Blender) exporta as articulações quadro a quadro,
   no nosso referencial e na escala do nosso corpo (pelo comprimento da
   perna), com a pose de repouso junto.
2. `gait_metrics.py` mede a passada com as mesmas regras para a referência e
   para os nossos clipes: velocidade, cadência, apoio, voo, onde o pé toca,
   joelho no toque/apoio/balanço, pé no toque e na saída, ombros, tronco,
   braços e cotovelo. Inclinações são relativas à pose de repouso, porque os
   ossos Mixamo vêm inclinados de fábrica, e tudo é medido pelo rumo real
   dos quadris (um dos arquivos está virado 180°).
3. Os números que **caracterizam** a passada viram parâmetros do gerador. O
   que é exagero da referência, ou não cabe no nosso corpo, fica de fora:
   pelve subindo 23 cm, quadril caindo 8–9° e pés na linha do meio (aqui as
   coxas já se tocam no repouso).

Comparação final (referência → nosso):

| | Slow Run → `jog` | Running(2) → `sprint` | Run Backward → `run_backward` |
|---|---|---|---|
| velocidade | 2,95 → 3,0 m/s | 4,1 → 7,5 m/s (é sprint) | 3,0 → 2,9 m/s |
| cadência | 164 → 164/min | 171 → 258/min | 189 → 189/min |
| voo | 41 → 33% | 62 → 53% | 47 → 43% |
| pé toca à frente do quadril | 17 → 22 cm | 19 → 29 cm (eram 38) | −14 → −10 cm |
| joelho no toque | 41 → 39° | 34 → 38° | 62 → 64° |
| joelho no balanço | 100 → 98° | 125 → 136° | 94 → 101° |
| ombros girando | 17 → 15° | 36 → 16° (eram 4) | 11 → 10° |
| cotovelo | 63–116 → 66–110° | 73–117 → 62–98° | 45–105 → 50–100° |

### `sprint_backward`: o recuo com sprint

Pedido do usuário: correr para trás mais rápido, com movimentos mais
amplos. Não há referência; é o `run_backward` com o corpo pedindo
velocidade. Correr de costas rápido é questão de ritmo, não de passada: o
quadril só estende 30° em relação à pelve e o pé que toca atrás não pode ir
longe. Em relação ao recuo:

| | `run_backward` | `sprint_backward` |
|---|---|---|
| velocidade | 2,9 m/s | 4,4 m/s |
| cadência | 189/min | 216/min |
| passo | 0,92 m | 1,22 m |
| voo | 43% | 45% |
| pé toca atrás do quadril | 10 cm | 18 cm |
| joelho no toque | 63° | 75° |
| joelho no balanço | 100° | 113° |
| pelve abaixo do repouso (media autoral) | 7,7 cm | 10,8 cm |
| tronco à frente | 7° | 7° |
| braço (balanço, cotovelo) | 35°, 75±25° | 46°, 80±20° |
| braço à frente / atrás | 7° / 66° | 23° / 73° |
| ombros girando | 10° | 14° |

(medido com `gait_metrics.py` no clipe gravado)

Duas tentativas reprovaram no gate de alcance (tornozelo além do
comprimento da perna): com o tronco a 10° e o toque 6 cm atrás do quadril,
19 mm; com 7° e 3 cm, 4 mm. A pelve 9 cm abaixo do repouso (uma postura mais
atlética, que combina com o recuo rápido) resolveu. No corpo físico: rastreio
de tronco/braços 10,6° (recuo e na diagonal para trás), sem quedas.

### Strafe: a referência, com as correções mínimas

O primeiro strafe foi um passo lateral de marcação gerado por código (os pés
não cruzavam) e o usuário o descartou: "extremamente errado e feio". Pediu
o mais perto possível de `Jog Strafe Left/Right.fbx`. O strafe é então a
própria referência retargeteada (`import_humanoid_animation.py`; os
retargets crus ficam em `assets/animations/source/retargeted/`), e
`clips/strafe.py` aplica só o que o corpo exige, cada correção a menor
possível e suavizada no ciclo:

- **Emenda.** O retarget da direita termina num quadro de pausa em que a
  raiz volta 4,5 cm de uma vez. O ciclo fecha no quadro anterior, e a deriva
  que sobra vira velocidade. O ciclo é girado para andar exatamente a ±90°.
- **Pé de apoio.** O pé Mixamo tem dedos; o nosso é uma caixa rígida. No
  fim do apoio a referência ergue o calcanhar com os dedos no chão, e a
  caixa afundaria 4 cm; o pé de apoio da referência também escorrega
  5–9 cm. No apoio o pé fica parado no mundo, onde a referência o pôs em
  média, no rumo e na inclinação medianos dela (o strafe para a direita
  corre na ponta do pé, ~12°), e sem rolar de lado. Os quadros de toque e de
  saída, com o pé inclinado mais de 20°, ficam como na referência e viram
  "rolando" (contato 0,45: o runtime mantém a trava do pé se já travou, mas
  não trava nele). Ponta do apoio fora do alcance da perna também vira
  "rolando".
- **Chão.** Nenhum pé abaixo do piso: se a referência afunda, o tornozelo
  sobe o necessário.
- **Coxas.** Na passada cruzada as coxas se encostam no alto, perto do
  quadril. Nesses quadros a pelve gira alguns graus a mais (4° na esquerda,
  10° na direita), a coluna devolve o giro e as pernas são refeitas por IK
  até os mesmos pés. Com a folga de repouso das coxas em 3 mm eram 19–25°;
  ver "Colisores do jogador".
- **Pés.** O pé em balanço sobe até 4,5 cm quando passa pela frente do de
  apoio (afastar para a frente pediria 8,5 cm).
- **Cabeça.** Olha para a frente em todo quadro; na referência ela varria
  23° com o tronco.
- **Braços.** A referência balança os braços largos, cruzando o corpo, e o
  usuário achou "muito espalhafatoso". Eles ficam na postura padrão de
  corrida e balançam no tempo da referência (o braço acompanha as pernas),
  com 30% da amplitude dela. O cotovelo fica em ~72°, longe de 90° (ver "O
  antebraço do ALS").

Fica da referência, de propósito: a pelve e o tronco girados para o lado de
trás (−20° e −13° em média no strafe para a esquerda, +9° e −7° no para a
direita). É o estilo do Jog Strafe: a perna de trás cruza por trás da outra
e o quadril dela vai junto. O sprint de lado é o mesmo movimento com
cadência 1,4× (4,1 e 3,4 m/s).

As duas referências andam em velocidades diferentes (2,9 e 2,4 m/s). De lado,
a cápsula anda na média das duas, e cada clipe acompanha pelo ritmo
(`playbackRate`).

### Giro do joelho para manter o rumo do pé

O tornozelo do ALS não gira o pé em torno da vertical: o rumo do pé é o da
canela. Com a perna varrendo de lado, a canela gira e o pé ia junto (4° num
apoio de strafe, arrastando o pé no chão). O IK da perna agora gira o joelho
em torno da linha quadril–tornozelo até a ponta apontar para onde foi
pedido, como numa pessoa. Isso melhorou todos os ciclos: o deslize do pé de
apoio caiu para 0,03–0,18 mm.

### O antebraço do ALS e a pronação

A junta do antebraço tem o cotovelo como eixo "twist" e a pronação como
"swing1", que é o eixo longo do **braço**. Com o braço esticado, os dois
coincidem. Com o cotovelo perto de 90°, esse eixo passa a girar o antebraço
inteiro em volta do braço, e o motor dele é fraco (90 N·m/rad; a coxa tem
750). Pedida diferente de zero, a pronação ainda põe o alvo no eixo travado
(swing2). O resultado medido foi o antebraço girando sozinho: 88° do alvo
no sprint lateral e 41–68° no pulo saindo do sprint.

Regras que ficaram:

- pronação 0 em passadas e pulos. Melhorou o rastreio de todas: corrida
  leve 21 → 16°, costas 17 → 11°, sprint 24 → 21°;
- cotovelo longe de 90° quando o braço bombeia rápido (pulo correndo até
  68°);
- cotovelo que **fica parado** perto de 90° também gira: no strafe com os
  braços calmos, entre 82° e 88°, o antebraço ficou 86° fora do alvo. Com
  ~72°, 7–8°. Nas passadas o cotovelo só passa por 90°, balançando 44°.

**27/09: o cotovelo virou dobradiça.** O eixo de "pronação" foi tirado do
perfil (`swing1` do antebraço). Nenhum clipe o usava, além dos 10° do
parado, que agora vêm do giro do braço. Medido no corpo físico, esse eixo
carregava quase todo o erro do braço: 15–17° de pico no antebraço, contra
2–5° do cotovelo. Era o antebraço sacudindo, os braços "moles" que o
usuário via. Sem ele, o rastreio de tronco e braços caiu de 21° para 4° no
trote e de 24° para 14° no sprint, e os braços puderam ficar junto ao
corpo também no sprint. A pronação de verdade (girar a mão em torno do
antebraço), se um dia for preciso, fica no punho.

### Autocolisão declarada: abdômen × coxa

A cápsula do abdômen (raio 12,5 cm em torno de um eixo lateral) desce até a
altura do quadril. A coxa encostava nela com 45° de flexão e a penetrava 3 cm
a 90°; num corpo real a coxa só toca a barriga perto de 110°. Toda passada,
sprint ou chute bateria numa barriga que não existe. O par foi declarado sem
colisão no perfil (`selfCollisionIgnoredPairs`, com o motivo), gerado por
`prepare_als_ragdoll.py`. O Jolt e o validador Python leem a mesma lista.

## Colisores do jogador

Os colisores do `FootballPlayerV1` são medidos da malha dele
(`tools/fit_ragdoll_colliders.py`), e não herdados do manequim ALS:
cápsulas cônicas nos membros (`radiusAtPositiveX`, `TaperedCapsuleShape` no
Jolt), cápsulas laterais no tronco e caixa do tamanho da chuteira no pé.
Âncoras, limites e massas não mudam.

- **Coxas:** o raio de cima é limitado para as duas ficarem a 2,4 cm uma da
  outra em repouso. A carne da parte interna da coxa cede, e o colisor é o
  núcleo dela, não a malha do short. Com 3 mm, repetir a passada cruzada do
  strafe pedia girar a pelve 19–25° a mais.
- **Mão:** medida na mão **aberta**, como modelada (20 cm). Medida na mão
  relaxada ela encolhe para 15 cm, a inércia cai e o antebraço deixa de
  acompanhar o alvo: rastreio do braço no trote de 16° para 25°. Os ganhos
  do braço foram calibrados com a mão maior.

## Mãos

A mão relaxada, semifechada e com os dedos juntos, é da **skin**: os dedos
são ossos visuais, 3 por dedo e 15 por mão. Eles seguem a mão e não têm
física (ver `RAGDOLL_CHARACTER_PIPELINE.md`). Os clipes não mexem nos dedos.
Cada osso tem uma rotação de repouso: a flexão em cascata do indicador ao
mínimo, como numa mão em repouso, os dedos fechando o afastamento modelado e
o polegar próximo do indicador. Em 27/09 a flexão foi reduzida em cerca de
25% para a mão ficar relaxada e menos fechada. Animar a mão, para o goleiro
no futuro, é
trocar essas rotações.

## Olhar × movimento (implementado)

Requisito do usuário: o jogo é de futebol, então o personagem **sempre olha
para onde a câmera aponta**. Nada de "câmera para a frente, comando para
trás, ele dá meia-volta". Para andar e correr de lado, movimento próprio de
strafe; para frente e para trás, a cintura gira para o movimento.

- **Setores.** O movimento, em relação à câmera, cai num de quatro
  setores: frente, esquerda, direita ou costas. Cada setor tem o seu ciclo:
  corrida leve (ou sprint), strafe (ou sprint de lado), corrida de costas.
  **A direção decide o ciclo, nunca a ordem das teclas.** As fronteiras
  ficam no meio entre as oito direções do teclado, em 67,5° e 112,5°:

  | teclas | direção | ciclo |
  |---|---|---|
  | W | 0° | corrida (ou sprint) |
  | W+A, W+D | ±45° | corrida (ou sprint), pelve girada 45° |
  | A, D | ±90° | strafe (ou sprint de lado) |
  | S+A, S+D | ±135° | recuo, pelve girada 45° |
  | S | 180° | recuo |

  Antes as fronteiras ficavam em 45° e 135°, em cima das diagonais, e quem
  decidia era a histerese: W e depois A dava corrida; A e depois W, strafe.
  A histerese (10°) ficou só para o analógico perto da fronteira; nenhuma
  direção do teclado chega nela.
- **Pelve.** Dentro do setor, a pelve gira para o movimento: fica em
  `movimento − centro do setor` (até ~78° da câmera na borda do setor),
  mais o giro que o próprio ciclo tem (o strafe, −20° e +9°). Correndo na
  diagonal para a frente, a cintura vira para a diagonal; andando de lado,
  o corpo fica de frente para a câmera e as pernas fazem o strafe.
- **Tronco:** a coluna devolve 80% do ângulo entre pelve e câmera, até 45°
  (20° no sprint: correndo, o tronco fica com as pernas e quem olha é a
  cabeça).
- **Cabeça:** completa o resto, até 75° (pescoço 30 + cabeça 45). A cabeça
  acompanha a câmera rápido e o tronco vem atrás, mais devagar.
- **Sprint** vale em todos os setores. Recuando, é o `sprint_backward`
  (papel `sprintBackward`, opcional: sem ele, recuando, segurar sprint não
  muda nada).
- **Velocidade:** `characterGaitSpeed3D` dá a velocidade do setor (3,0 / 7,5
  frente, 2,7 / 3,7 lado, 2,9 / 4,4 costas) e passa de uma para a outra
  dentro da faixa de histerese. O jogo passa isso como `speedScale`.

Por que sem ciclos diagonais: com passada longa, 30° de desvio entre pelve e
movimento levam cada pé 23 cm para o lado, além dos quadris (a 9 cm). A perna
de trás cruza por baixo do corpo, e o validador mostrou coxa × coxa −17 mm.
Nenhum humano anda assim; ele gira a pelve. Por isso a diagonal é o ciclo do
setor com a pelve girada.

Detalhes do runtime (`CharacterLocomotion3D`):

- A rotação inteira da pelve do ciclo é preservada (giro de até 8° com a
  perna, queda do quadril). Só do levantar importado sai o rumo da fonte.
- O balanço lateral da pelve do clipe entra só no personagem controlado,
  ancorado na cápsula. Num boneco solto, a posição-alvo da raiz é a própria
  pelve simulada: somar o balanço realimentava e integrava (medido 2,6 m/s
  em bonecos parados encostados).
- O limitador de velocidade de junta (12 rad/s) deixa passar o que a amostra
  do clipe anda (o joelho do sprint) e segura IK, olhar e a mistura do
  crossfade.
- Troca de setor tem crossfade de 0,35 s, contra 0,14 s nas demais trocas.
  Desde 27/09, a mistura preserva a velocidade de saida por uma tangente
  Hermite que desaparece ao concluir a troca. Isso vale para juntas, translacao
  e rotacao da raiz, inclusive em transicoes interrompidas; olhar e IK ficam
  fora do historico para nao serem somados duas vezes.
- A inclinacao lateral usa a aceleracao da curva (`velocidade * taxa de giro`),
  com entrada progressiva e mola criticamente amortecida. A inclinacao frontal
  usa a aceleracao ja filtrada, sem uma segunda mola: atrasar o quadril novamente
  prejudicava o apoio na descida de escadas.

### Amplitude vertical revisada em 27/09

Jog: 4,4 cm pico a pico; sprint: 3,6 cm; recuo: 3,6 cm; sprint de costas:
4,4 cm. A altura minima da pelve no apoio foi preservada, diminuindo a subida
durante o voo. As pernas foram regeneradas junto com a pelve, mantendo contato,
cadencia e velocidade. Nao filtrar somente a altura no runtime: isso atrasa a
pelve em relacao aos pes e faz a correcao de chao produzir tremor.

## Pés no terreno, passos, freada, giro parado e tremor (runtime)

O pé pisando na altura certa de cada degrau, com a previsão de onde vai
pisar e passando por cima de cada espelho; a pelve descendo para o pé mais
baixo; o ciclo acompanhando a cápsula (passo mais curto devagar); os passos
de acomodação e o jogo de pés do giro parado (a base em alerta foi estudada
em Right Turn(4) e Left Turn); a reação à freada (estudada em Run To
Stop); e a causa do tremor da pelve. Tudo em `CHARACTER_FOOTWORK.md`.

## Pulos

Um pulo é o **voo**, da decolagem (fase 0) ao ápice (0,5) e ao toque no chão
(1). Não há preparação agachada: no jogo o pulo sai no instante do botão.

- **Escolha:** na decolagem, pelo setor em que ele corria (frente, costas,
  esquerda, direita), ou parado se a velocidade estava abaixo de 1 m/s. O
  pulo escolhido vale até o fim da aterrissagem.
- **Fase:** vem da velocidade vertical da cápsula, `(v0 − v)/(2·v0)`. Serve
  para qualquer altura de pulo. Sem impulso para cima (caiu de um degrau), o
  voo já começa descendo, na segunda metade do pulo parado.
- **Aterrissagem:** parte da pose de toque, a pelve desce 8 cm e os pés são
  plantados pela trava de pé. É o IK que dobra os joelhos, absorvendo.

Os pulos (`clips/jumps.py`) são poses-chave anatômicas (onde cada pé está em
relação ao quadril, pelve, coluna, braços) interpoladas por Catmull-Rom. As
referências `Jumping(2)`, `Jump` e `Jump(2)` deram o caráter; o exagero ficou
de fora, como o usuário pediu (braços acima da cabeça, giros largos):

- **parado:** pernas quase esticadas saindo do chão, recolhe no ápice e
  estica de novo; braços sobem à frente, sem passar da cabeça;
- **correndo:** salto de passada, em tesoura. A perna de impulso sai
  esticada para trás, a outra sobe com o joelho à frente e é a que recebe o
  chão. Braços com a abertura da corrida (saindo do trote com os braços
  juntos, abrir no voo deixava o antebraço 32° atrás do alvo);
- **para trás** (refeito em 27/09 a partir de `Jump(2).fbx`): salto em
  tesoura de costas. O impulso é na perna da frente, que sai esticada (quem
  salta para trás empurra com o pé adiante); a de trás sobe dobrada, estica
  no voo e recebe o chão; a da frente sobe dobrada à frente, menos que na
  referência. Tronco −2° no impulso, +8° no ápice e +16° na queda
  (referência −10, 0 e +12). Os braços são nossos: à frente e pouco abertos,
  o esquerdo mais à frente, contra a perna da frente. Na referência eles
  sobem acima dos ombros e se debatem;
- **para os lados** (sem referência, pedido do usuário): impulso da perna de
  fora, a do lado do salto abre e recebe o chão, tronco inclinado para o
  lado, braços abertos.

Observação: o pulo da cápsula (`jumpSpeed` 4,6 m/s) eleva o quadril 1,08 m,
altura de atleta de salto em altura. O pulo parado da referência sobe 30 cm.
Não foi alterado.

## Aceitação física

`testLookAndTravel` e `testJumps` (filtros `look` e `character`) medem no
corpo simulado, com a câmera fixa e a cápsula como no jogo. O tronco é medido
pela direção **média**, porque os ombros giram com os braços de propósito; a
cabeça, pelo pior instante.

| caso | setor | pelve (esperado) | tronco × olhar | cabeça | pior segmento |
|---|---|---|---|---|---|
| frente (W) | frente | 0,1° (0) | 0° | 1° | 4° |
| diagonal frente-esquerda (W+A) | frente | 45,1° (45) | 7° | 2° | 5° |
| esquerda (A, strafe) | esquerda | −20,0° (−19,9) | 13° | 2° | 3° |
| direita (D, strafe) | direita | 9,1° (9,4) | 8° | 3° | 4° |
| diagonal trás-esquerda (S+A) | costas | −45,3° (−45) | 8° | 2° | 12° |
| para trás (S) | costas | −0,3° (0) | 1° | 2° | 7° |
| sprint frente | frente | 0,1° (0) | 0° | 7° | 14° |
| sprint diagonal (W+D) | frente | −44,9° (−45) | 25° | 7° | 14° |
| sprint lateral | esquerda | −19,9° (−19,9) | 14° | 4° | 5° |
| sprint pedido para trás | costas | −0,3° (0) | 1° | 2° | 7° |

A ordem das teclas não muda o ciclo: A→W e W→A dão corrida, A→S e S→A dão
recuo, e soltar o W de W+D dá strafe. Na varredura, com a direção dando a
volta completa em torno do olhar, há quatro trocas de setor e nenhuma queda.
O membro mais rápido chega a 8,6 m/s correndo e 13,5 no sprint.

| pulo | pior segmento | cabeça | volta à passada |
|---|---|---|---|
| parado | 6° | 2° | sim |
| correndo | 4° | 3° | sim |
| em sprint | 16° | 10° | sim |
| correndo de costas | 12° | 3° | sim |
| de lado (esquerda) | 5° | 3° | sim |
| sprint de lado (direita) | 7° | 5° | sim |

Pés no terreno (escada), passos coerentes e o tremor da pelve têm os
próprios testes e números em `CHARACTER_FOOTWORK.md`.

Controles negativos, cada um registrado no teste:

- coluna girando 45° no sprint: o braço bate na coxa, 61°;
- braço fechado na corrida de costas em diagonal: 60°;
- cotovelo sem abrir atrás no sprint: o antebraço trava a 150°;
- pronação ≠ 0 com o cotovelo perto de 90°: 88° no sprint lateral e 68° no
  pulo saindo do sprint;
- cotovelo parado perto de 90° no strafe: 86°;
- colisor da mão medido na mão relaxada: trote 25°;
- o eixo de "pronação" do ALS no antebraço (antes de tirado do perfil):
  trote 21°, sprint 24°, com 15–17° só nesse eixo.

## Próximo

- **Mão animada** (goleiro): os ossos dos dedos existem; falta o canal de
  animação deles. Outros ossos do FBX que ainda seguem um link físico:
  clavícula (segue UpperChest) e dedos do pé (seguem o pé). Candidatos ao
  mesmo tratamento de osso visual.
- **Pronação no punho**, se a mão precisar girar (goleiro).
- **Altura do pulo:** decidir com o usuário (hoje 1,08 m de elevação).
- Retirar `cc0_idle`, `mixamo_running` e `mixamo_sprint` do catálogo depois
  da aprovação.
