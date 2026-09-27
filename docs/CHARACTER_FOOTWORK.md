# Pés do personagem: terreno, passos e pelve

Atualizado em 27/09/2026. Implementação em
`src/Engine/Character/CharacterLocomotion3D.cpp`, bloco "Pés no terreno".

O que o usuário pediu:

- o pé sempre pisando na altura certa do terreno, cada pé na sua altura;
- a previsão de onde o pé vai pisar, para o alvo de altura já estar certo
  antes de o pé chegar;
- sem atrapalhar movimento rápido e preciso;
- o corpo não pode andar sem as pernas andarem;
- sem as tremidas pequenas no corpo andando;
- na escada, sem a perna dando coice para trás;
- parar correndo com uma reação natural do corpo, que não trave nada;
- girar parado com jogo de pés de verdade (o de antes era "extremamente
  feio").

## Como o corpo é movido (resumo)

A cápsula (o controlador de personagem do Jolt) anda pela entrada do
jogador. A pelve do ragdoll segue a cápsula mais o deslocamento do próprio
clipe. As juntas vão aos alvos por motores. O que o jogador vê é o corpo
físico. Tudo abaixo decide os **alvos** das pernas e da pelve.

## Chão de cada pé

`CharacterLocomotionInput3D::groundAt(ponto)` devolve o chão em qualquer
ponto do mundo. No jogo, só o personagem controlado a recebe; é uma sonda
esférica de 3 cm, de 55 cm acima da base da cápsula até 1,5 m para baixo,
só em geometria estática. Os bonecos soltos continuam com `footGround`, a
sonda embaixo de cada pé físico. Sem nenhuma das duas, vale o chão da
cápsula.

Cada consulta mede calcanhar, meio e ponta, e vale o mais alto: o pé apoia
no ponto mais alto embaixo dele e nunca afunda na quina de um degrau.

## Apoio: o pé travado

Quando o clipe diz que o pé está no chão (contato acima de 0,55) e o pé
animado está perto do chão e quase parado, o pé trava no mundo:

- sobre o chão medido ali, com a sola paralela a ele (a normal do chão) e o
  rumo do clipe. A inclinação de aceleração do corpo (até 12°) não passa
  para o pé. Antes, a quina de um pé inclinado afundava até 3 cm;
- o pé travado ainda **rola**: quando o clipe ergue o calcanhar, ele gira
  sobre a ponta, e quando pousa de calcanhar, sobre o calcanhar. O ponto de
  apoio e o rumo ficam parados no mundo. Congelado plano, o pé só rolava ao
  soltar, de uma vez (chicote de 8,9 m/s no strafe);
- solta quando o contato do clipe cai abaixo de 0,55, ou quando o pé
  animado sobe mais de 5,5 cm. Segurando até 0,30, o pé ficava preso
  enquanto a pose já o levava embora, e na soltura corria atrás dela
  (12 m/s no sprint de lado). A soltura é uma mistura de 0,12 s;
- solta também quando a perna não alcança mais o ponto da trava: pelo
  comprimento da perna ou porque o IK ficou mais de 6 cm longe dele (o
  limite de extensão do quadril, 30°). Descendo escada, o degrau de trás
  ficava longe e o pé travado ficava pendurado no limite do IK, 26 cm acima
  do degrau.

## Balanço: onde o pé vai pisar

Cada clipe de passada tem, por pé, os instantes em que ele levanta e toca
o chão e onde está no toque (calculado uma vez por clipe, a partir das
curvas de contato). No balanço:

1. **Previsão.** Com o tempo até o próximo toque e a velocidade da cápsula,
   dá para saber onde a pelve estará; com o pé do clipe naquele instante,
   onde o pé vai pisar. A sonda mede o chão ali antes de o pé chegar.
2. **Encaixe no degrau.** O pé tem 28 cm, o degrau do laboratório também.
   Se calcanhar e ponta cairiam em alturas diferentes, o ponto de pouso
   anda o mínimo ao longo do pé (até 20 cm) para caber inteiro num degrau.
   O pé em balanço vai sendo levado para lá. No toque o encaixe é refeito,
   até 10 cm, porque a previsão erra alguns centímetros quando a cápsula
   freia a cada degrau. Esse cálculo é caro e só é refeito quando o pouso
   previsto anda mais de 3 cm.
3. **Altura: o terreno à frente do pé.** O pé passa por cima do que tem
   pela frente, não sobe antes para o degrau de pouso. Quatro sondas à
   frente da ponta, no rumo do caminho até o pouso (2 a 20 cm), transformam
   cada espelho de degrau numa rampa (inclinação 1,1, folga de 3 cm na
   quina); uma sonda dentro do calcanhar segura o pé na altura de onde saiu
   até ele passar a borda (descendo), sem nunca erguê-lo. O terreno depois
   do pouso não conta (o pé pousava pendurado sobre o degrau seguinte). A
   altura sobe na hora (no máximo 6 m/s) e desce no ritmo que chega ao
   pouso no toque; nos últimos 20% do balanço ela vai para o chão do pouso
   de qualquer jeito.

   A regra de antes subia a altura do degrau de pouso cedo, pelo tempo e
   depois pelo caminho. Na passada de escada cada pé sobe 4 degraus (72 cm)
   por balanço, e subir isso cedo levava o pé, ainda atrás do quadril, a
   48–72 cm acima do degrau embaixo dele: o coice que o usuário viu. Um
   erro ajudava: o ponto de saída do balanço não era regravado ao soltar a
   trava, e ficava o de um passo antigo (2,5 m para trás).
4. **Orientação.** Perto do toque, o pé se alinha ao chão de destino.
5. **Até travar.** A trava vem alguns ticks depois do toque do clipe
   (contato acima de 0,55, pé animado parado). Nesse meio-tempo o pé ainda
   está pousando naquele toque; antes ele já passava para o toque seguinte,
   jogava fora o encaixe no degrau (voltava 12 cm, para a quina) e travava
   pendurado.

## Pelve

A pelve desce o que for preciso para o quadril alcançar cada pé apoiado,
ou pousando, com a perna a 94% de esticada. Conta só o que o terreno pede
a mais que a própria animação: no chão plano o trote já deixa a perna quase
esticada, e isso não é motivo para descer. Com os dois pés apoiados acima
da base da cápsula, a pelve sobe. É uma mola criticamente amortecida, sem
degrau.

A primeira versão descia até o pé mais baixo. Na escada isso agachava o
corpo sem necessidade (as pernas contorcidas que o usuário viu). E, com a
cápsula subindo as quinas de forma contínua, 8–16 cm acima do degrau de
apoio, o pé recém-pousado ficava pendurado.

O pé em balanço também conta, antes do toque: a pelve já desce o que a
perna vai precisar para alcançar o ponto de pouso, com o quadril onde a
cápsula o terá levado (a partir de 30% do balanço). Descendo o último
degrau, o pé pousava no chão com a pelve ainda alta: fora do alcance, a
trava soltava e voltava em loop (pé flutuando 17 cm).

O aterramento de segurança olha só os pés apoiados. O pé em balanço tem a
própria folga sobre o chão. Contar com ele erguia o corpo inteiro quando
ele passava rente à quina do degrau seguinte (pelve 16 cm acima).

O controlador do Jolt sobe e desce degrau num salto de altura. A pelve vai
junto, suavizada em ~0,1 s.

## Pernas por IK

`solveFootPose`: a coxa e a canela levam o tornozelo onde o pé pedido
precisa dele (mínimos quadrados amortecidos, dentro dos limites); parado, a
perna inteira gira em volta da linha quadril–tornozelo até o pé apontar para
onde deve (o giro do quadril, ±45°); e o tornozelo, com os dois eixos que
tem, orienta o pé, com prioridade para a sola.

O giro da perna é novo. O tornozelo não tem giro: o rumo do pé em relação à
pelve vem da rotação da coxa, que o IK de posição nunca usava. O pé
"travado" girava junto com a pelve, esfregando no chão (medido: 32° no
primeiro passo de um giro parado, os mesmos da pelve).

Só parado (giro no lugar, acomodação, idle). Andando, o pé segue o rumo da
pelve como sempre seguiu: com a abertura de 7° da pose respeitada, o pé de
28 cm não cabia mais no degrau de 28 cm e a quina afundava (descendo a
escada, 7 amostras a até 11 cm).

## Passada de escada

O chão à frente, a 70 cm e a 1,05 m na direção do movimento, dá a
inclinação do terreno. Mais perto que isso, um único espelho de degrau já
dava 0,5–1. O passo fica curto o bastante para subir ou descer no máximo
34 cm por passo (dois degraus). O ritmo sobe até 15% acima do clipe, e a
velocidade que cabe no terreno vai para o jogo (`terrainSpeedLimit` na
telemetria), que a usa como teto da cápsula.

No trote a passada inteira tem 1,1 m, ou seja, 4 degraus por passo, e o pé
pousava 72 cm acima ou abaixo do de trás. Com a passada de escada, na
escada do laboratório:

- subindo: passo a 55% do clipe, 2,1 m/s;
- descendo: passo a 35%, 1,5 m/s.

## Parado na escada

A base redonda da cápsula, parada na quina de um degrau, encostava no
espelho do degrau de cima e escorregava escada abaixo (24 cm em 4 s). Agora,
sem comando de movimento e com chão pisável até um degrau abaixo dos pés, a
cápsula fica parada: quem segura o corpo são as pernas. Ela ainda assenta
7 cm ao sair do espelho e para. No ar ela nunca conta como apoiada, ou
ficaria pairando. O mesmo critério tira os instantes "sem chão" da cápsula
descendo a escada.

## O corpo só anda com as pernas

- **Ritmo e passo.** O ciclo avança com a velocidade da cápsula. O ritmo
  fica entre 0,55× e 1,45× o do clipe; o resto da velocidade vira passo mais
  curto ou mais longo (*stride warping*: o pé é levado ao longo da direção
  do ciclo, em torno do quadril, e a previsão do pouso usa o mesmo fator).
  Antes o ritmo sozinho ia até 0,35×, e devagar o pé andava mais que o
  corpo.
- **Passos de acomodação.** Parado, os pés também travam. Se um pé travado
  fica a mais de 12 cm de onde a pose parada o quer, ou 34° fora do rumo,
  ele dá um passo de 0,22 s até lá, um pé por vez. Isso vale para quem parou
  no meio da passada, levou um esbarrão ou girou a pelve. O passo do giro no
  lugar (a partir de 29°) é o mesmo mecanismo.

## Giro parado

O caso atual parte da base escalonada de `alert_idle` e foi estudado quadro a
quadro em Right Turn(4) e Left Turn
(`assets/animations/source/mixamo/study`):

1. o tronco começa o giro e a pelve vem junto, girando continuamente;
2. para qualquer lado, o pé de **trás** sai primeiro e contorna o da frente,
   que sustenta o peso;
3. o pé da frente sai em seguida e recompõe a mesma base escalonada no rumo
   novo;
4. a distância e o ângulo definem a duração de cada passo, entre 0,24 e
   0,48 s. Um giro de 90° termina em dois passos.

Na base lado a lado do surgimento, o padrão antigo das referências Left Turn
90, Right Turn(3) e Happy Right Turn continua válido: sai primeiro o pé do
lado do giro, abrindo sobre a bola do pé.

O de antes: a perna torcia com o pé travado até 29° de giro acumulado,
então um passo reto de 0,22 s, com o pé da vez escolhido por alternância.

Agora:

- **Quando.** A base (o rumo dos dois pés travados, menos a abertura que a
  pose parada dá a cada um) fica para trás da pelve. O giro começa quando a
  perna torce mais de 24° ou faltam mais de 24° para a base alcançar o alvo.
  Isso mantém giros lentos da câmera ativos entre um par de passos e outro.
- **Para onde.** Os dois passos miram a pose parada com a pelve já no rumo
  final do giro, até 72° além do rumo em que o passo começou (o quadril só
  gira 45°: mirando 115° de uma vez, na meia-volta, o pé pousava onde a
  perna não chegava e a trava o torcia de volta). Uma meia-volta são dois
  pares de passos.
- **Ordem:** na base em alerta, o pé de trás contorna primeiro e o da frente
  recompõe a base. Na base lado a lado, o primeiro pé gira sobre a bola do
  pé e o segundo contorna o apoio.
- **Duração:** 0,24–0,48 s, calculada pela distância e pelo giro do pé.
- **Peso:** enquanto um pé dá o passo, a pelve vai um quarto do caminho
  para o pé de apoio (até 5 cm) e baixa 1,5 cm.
- **Pelve:** depois de começar, o giro vai até os 18° do olhar (a coluna e
  a cabeça levam o resto). Antes parava ao entrar nos 31°, e o alvo do giro
  mudava no meio dos passos.

A acomodação (pé longe de onde a pose parada o quer, sem giro) continua,
com passo de 0,26 s em linha reta.

## Freada

Estudada em Run To Stop (4 m/s até parar em ~0,35 s): a perna da frente
trava esticada lá na frente, a pelve desce 25 cm, o tronco continua indo
para a frente pela inércia (de 12° para 32°) e tudo volta em ~0,7 s, com a
pelve girando para uma base de lado.

Aqui é uma reação do corpo à desaceleração da cápsula, nunca um estado:

- a inércia do tronco é uma mola subamortecida (ω 11, amortecimento 0,55),
  empurrada pela desaceleração no sentido do movimento, na proporção da
  velocidade com que ele vinha (de 1,5 a 6,5 m/s): parar do sprint é forte,
  do trote é leve;
- o tronco segue a inércia (até 22° de flexão, ou inclinação de lado), a
  cabeça devolve parte para o olhar ficar no horizonte, os cotovelos abrem
  até 20°;
- a pelve desce até 12 cm (as pernas dobram pelo IK dos pés);
- a inclinação de corpo inteiro pela aceleração continua, mas freando só
  15% dela vai para trás (saturava: com uma freada de sprint dava os 12,6°
  inteiros, e o tronco ia para trás em vez de para a frente);
- vale em qualquer direção: recuando, o tronco vai para trás; de lado,
  tomba;
- a cápsula não é segurada. Voltar a correr no meio da freada só desliga o
  empurrão, e a mola volta ao zero.

Os passos da freada são os do próprio ciclo: o ciclo segue a velocidade da
cápsula e encurta o passo enquanto ela desacelera.

## Tremor

**Girando a câmera.** A cabeça e o tronco tremiam: parado, a velocidade de
giro da cabeça sacudia 3,7 rad/s no corpo físico, com picos de 50 rad/s no
alvo. Toda troca de estado começava um crossfade a partir da pose do tick
anterior, que já tinha o giro do olhar, e o olhar era somado de novo:
cabeça e tronco pulavam pelo dobro do giro. Girando a câmera parado, o
estado alternava entre parado e girando no lugar a cada tick.

- O crossfade agora parte da pose do clipe, antes do olhar e do IK.
- O estado "girando no lugar" tem histerese.
- O ângulo da câmera, que chega em degraus (um por quadro da tela, com a
  física a 120 Hz), passa por uma mola criticamente amortecida antes do
  olhar. A escolha do setor do movimento continua com o ângulo cru.

Resultado: parado, cabeça 0,09 e peito 0,08 rad/s; correndo, 0,19 e 0,14.

**Andando.** A tremida do corpo andando tinha uma causa principal, medida
no alvo da pelve:

- a altura da pelve vinda do clipe passava por um filtro (16/s) e ficava
  atrasada em relação às pernas;
- o pé de apoio do alvo afundava no chão;
- o aterramento empurrava a pelve de volta num tick.

O resultado era um dente de serra de até 2,6 cm em 8 ms no sprint. Agora a
altura e o balanço da pelve saem direto do clipe, no mesmo tempo das
pernas, com o crossfade das juntas nas trocas de ciclo. O aterramento que
sobra é uma rede de segurança suavizada (sobe rápido, desce devagar).

Resíduo de alta frequência no alvo da pelve (amostra menos a média de
75 ms): trote 4,3 → 2,9 mm RMS (pico 9,8 → 6,6), sprint 9,6 → 5,4 mm (pico
25 → 11). O que sobra é a oscilação legítima da passada.

## Testes (`MATTERENGINE_TEST_FILTER=feet`, também em `character`)

`testTerrainFootwork`: escada de 8 degraus de 18 × 28 cm, subindo e
descendo, com o controlador de cápsula real, a 3 m/s. Mede a sola física de
cada pé travado contra o chão embaixo dele. O controle negativo é o mesmo
percurso sem a sonda de terreno.

| | afundado > 2 cm | flutuando > 3 cm | rastreio do corpo | passo, velocidade |
|---|---|---|---|---|
| parado 4 s | 0 | 0 | — | cápsula assenta 7 cm e para |
| subindo | 0 de 285 (pior 15 mm) | 0 (pior 15 mm) | 10° | 55%, 2,1 m/s |
| subindo, sem sonda (controle) | 5 (pior 15 cm) | 4 | 35° | passada inteira, 3 m/s |
| descendo | 0 de 279 | 0 (pior 22 mm) | 12° | 35%, 1,5 m/s |
| descendo, sem sonda (controle) | 2 (pior 15 cm) | 3 (pior 45 cm) | 14° | passada inteira, 3 m/s |

Antes do encaixe no degrau, subindo, o rastreio chegava a 62°: o pé
pousava com a ponta dentro do espelho do degrau seguinte.

`testCameraTurnSmoothness` (`look` e `character`): a câmera gira em
degraus de 60 quadros por segundo, e mede o tremor do giro da cabeça e do
peito, parado e correndo.

Coice (pé em balanço atrás do quadril, acima do chão embaixo dele): pior
29 cm subindo e descendo (o espelho de 18 cm que ele tem de passar, a folga
e a própria passada); no plano, o trote dá 17 cm. Antes, 48–72 cm.

`testRunToStop` (`stop`, `feet` e `character`), com a cápsula real:

| | reação | tronco além da corrida | pelve |
|---|---|---|---|
| sprint, para | 1,09 | +7,3° | −15 cm |
| trote, para | 0,09 | — | −1,5 mm |
| sprint de costas, para | 0,43 (para trás) | −8,8° | −3,3 cm |
| sprint, para e volta a correr em 0,2 s | 0,72 | +10,2° | −6,3 cm; 0,6 s depois a cápsula está a 7,5 m/s e a reação em 0,007 |

`testTurnInPlace` (`turn`, `feet` e `character`): a câmera vira de uma vez
(90° para cada lado, meia-volta) ou gira a 57°/s:

| | primeiro pé | passos | torção máx. | membro mais rápido |
|---|---|---|---|---|
| 90° esquerda | direito (trás) | 2 em 0,73 s | 3,9° além da pose | 2,6 m/s |
| 90° direita | direito (trás) | 2 em 0,73 s | 3,4° além da pose | 2,6 m/s |
| meia-volta | direito (trás) | 4 em 1,59 s | 4,5° além da pose | 2,7 m/s |
| câmera girando a 57°/s | direito (trás) | 5 em 2,58 s | 4,1° além da pose | 2,2 m/s |
| 90° logo ao surgir | esquerdo | 2 em 0,64 s | 3,9° além da pose | 3,4 m/s |

No fim, cada pé fica a no máximo 3,6° da relação pé–pelve autorada para a
pose. A medição desconta os cerca de 18° de abertura deliberada da base de
alerta; tratá-la como torção da perna produzia o falso pico de 56,9° que
interrompeu a primeira tentativa. Em nenhum caso os dois pés saem do chão e
não há queda. Antes da torção do quadril no IK e do limite por par, a
meia-volta tinha os dois pés no ar num tick e um pé a 8,9 m/s.

`testFootCoherence`: arrancar, parar, toques curtos e inversão, com a
cápsula real. A cápsula nunca anda com os dois pés travados. O pé travado
anda até 55 mm no mundo nas arrancadas e freadas bruscas (limite 60), que é
o corpo físico chegando no alvo; eram 42–48 antes da reação de freada, do
giro da perna parado e do passo de acomodação de 0,26 s, que somam um pouco
cada.

## Escada e controlador

A escada do laboratório tem degraus de 18 cm, e a cápsula só subia 12 cm.
Agora sobe até 19 cm (`maximumStepHeight`), e os meios-fios de 20/30 cm
continuam obstáculo. Descendo, a cápsula fica colada ao chão até um degrau
mais 5 cm; com 15 cm fixos, cada degrau de 18 cm virava um instante no ar.

## Limites conhecidos

- A passada de escada encurta o passo do próprio trote. Não é uma
  animação de escada: o joelho e o tronco são os do trote.
- A sonda só vê geometria estática.
- O rumo do pé travado depende do giro da perna; o tornozelo tem só dois
  eixos.
