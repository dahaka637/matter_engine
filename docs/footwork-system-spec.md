# Sistema de Footwork — Especificação Técnica e Base Teórica

**Projeto:** Engine própria (C++ / NVIDIA PhysX 5)
**Módulo:** `footwork` — planejamento e execução de posicionamento de pés
**Etapa atual:** Fase 0 — apenas dois pés isolados, sem esqueleto, sem IK
**Documento:** v1.0 — spec para implementação assistida (Codex)

---

## 0. Como ler este documento

Este documento tem três camadas:

1. **Parte I — Base teórica.** Revisão do estado da arte em biomecânica da marcha, planejamento de passos em robótica bípede e locomoção procedural em jogos. Serve para justificar cada decisão de design e fornecer números de referência reais em vez de valores inventados.
2. **Parte II — Arquitetura.** O desenho do módulo: camadas, contratos, estruturas de dados, algoritmos e regras.
3. **Parte III — Execução.** Estrutura de arquivos, fases de implementação, critérios de aceitação, testes e **pontos deliberadamente em aberto**.

> **Nota para o agente implementador:** as seções marcadas com **[ABERTO]** são decisões que devem ser tomadas conforme as convenções, tipos matemáticos, sistema de módulos, ciclo de vida e camada de renderização já existentes na engine. Não invente uma arquitetura paralela; encaixe-se na existente. As seções marcadas com **[INVARIANTE]** são requisitos que não devem ser negociados.

---

## 1. Objetivo e escopo

### 1.1 O que este módulo é

Um **planejador e executor de passos** que, a partir de uma intenção de locomoção (direção desejada + direção do olhar + modo de marcha), produz continuamente:

- Uma **pose-alvo 6-DoF por pé** (posição + orientação) no mundo.
- Um **estado de fase** por pé (apoio / balanço / transição) com progresso normalizado.
- Uma **trajetória de balanço** amostrável, para visualização e para posterior consumo por IK.
- Uma **base de suporte** (polígono de apoio) e métricas de estabilidade derivadas.

### 1.2 O que este módulo NÃO é

- **Não é** um sistema de animação. Não toca em esqueleto, skinning ou blending.
- **Não é** um solver de IK. Ele produz alvos; quem resolve membros é outro módulo, depois.
- **Não é** um controlador de física do corpo. Na Fase 0, os pés são cinemáticos.
- **Não é** um sistema de navegação. Não faz pathfinding global; só planeja passos locais a partir de um vetor de intenção.

### 1.3 Requisitos funcionais (do briefing)

| # | Requisito | Onde é atendido |
|---|---|---|
| R1 | Movimento fluido dos pés em qualquer direção (frente, trás, lateral, diagonal) | §8, §9 |
| R2 | Controle por WASD, **relativo à direção do olhar** | §7.1, §8.1 |
| R3 | Curvas — pés acompanham mudança de direção do olhar durante a caminhada | §9.4 |
| R4 | Overlay do arco do passo na tela | §14 |
| R5 | Detecção de terreno; escadas; terreno acidentado; objetos físicos | §11 |
| R6 | Proibição de cruzar um pé sobre o outro | §10.2 |
| R7 | Proibição de movimentos biomecanicamente impossíveis | §10 |
| R8 | Correr — arco maior e velocidade maior | §12 |
| R9 | Soltar a tecla no meio do trajeto para e planta o pé na posição atual | §9.6 |
| R10 | **Sem passo mínimo fixo** — micro-ajustes são passos válidos | §9.2 **[INVARIANTE]** |
| R11 | Módulo isolado, sem monólito | §6, Parte III |
| R12 | Botão na toolbar do laboratório abre popup de entrar/sair do modo footwork | §15 |
| R13 | Base reutilizável para equilíbrio de ragdoll ativo no futuro | §13 |

---

# PARTE I — BASE TEÓRICA

O objetivo desta parte não é academicismo. É que cada constante do sistema tenha uma origem defensável, e que os problemas difíceis (quando dar um passo, onde pisar, como não travar) sejam resolvidos com soluções que já se provaram, em vez de heurísticas ad hoc que quebram no primeiro caso de borda.

## 2. Biomecânica da marcha humana

### 2.1 O ciclo de marcha

O ciclo de marcha (*gait cycle*) é definido do contato inicial de um pé até o próximo contato do **mesmo** pé. Ele se divide em duas fases globais:

- **Apoio (stance)** — pé em contato com o solo. Ocupa ~**60%** do ciclo.
- **Balanço (swing)** — pé no ar. Ocupa ~**40%** do ciclo.

Dentro do apoio, a subdivisão relevante para nós é por número de membros em contato:

| Sub-fase | % do ciclo | Significado para o módulo |
|---|---|---|
| Duplo apoio inicial | ~10% | Ambos os pés no chão; transferência de peso |
| Apoio simples | ~40% | Um pé só; o outro está em balanço |
| Duplo apoio terminal | ~10% | Ambos no chão; preparação para o próximo balanço |

Ou seja: **em marcha normal há duplo apoio em ~20% do ciclo (2 × 10%)**, e nunca há fase de voo. Isso é o que separa caminhar de correr — na corrida o duplo apoio desaparece e surge uma fase aérea em que nenhum pé toca o solo.

**Consequência de design:** o gait profile precisa de um parâmetro `duty_factor` (fração do ciclo em apoio, por pé). `duty_factor > 0.5` ⇒ existe duplo apoio (caminhada). `duty_factor < 0.5` ⇒ existe fase de voo (corrida). Isso unifica caminhar e correr numa única parametrização em vez de dois sistemas separados.

### 2.2 Parâmetros espaço-temporais

| Parâmetro | Definição | Valor adulto típico |
|---|---|---|
| **Step length** (comprimento do passo) | Contato inicial de um pé → contato inicial do pé oposto | ~0,70 m |
| **Stride length** (comprimento da passada) | Contato inicial → próximo contato do mesmo pé = 2 passos | ~1,40 m |
| **Step width** (largura do passo) | Distância lateral entre os pés | ~0,05–0,10 m |
| **Cadência** | Passos por minuto | ~100–120 (caminhada) |
| **Velocidade** | cadência × step length | ~1,2–1,4 m/s |
| **Toe clearance mínima** | Altura mínima do dedo sobre o solo em mid-swing | ~0,01–0,02 m (muito baixa!) |

Dois pontos merecem atenção:

1. **A largura do passo é pequena (5–10 cm), mas nunca zero.** É essa faixa que impede o cruzamento dos pés e define a base de suporte lateral. Ela é o fundamento numérico da regra R6.
2. **A folga real do pé sobre o solo é minúscula.** Humanos passam com ~1–2 cm de folga. Para um jogo isso é apertado demais (jitter de raycast, tunneling, terreno poligonal), então usaremos folga inflada — mas é bom saber que estamos exagerando de propósito, e não perseguir "realismo" aumentando o arco indefinidamente.

### 2.3 Relação velocidade / comprimento / cadência

Ao acelerar, humanos aumentam **simultaneamente** o comprimento do passo e a cadência — a relação entre comprimento da passada e velocidade é aproximadamente linear numa faixa ampla. Na transição para corrida, o comprimento da passada cresce bastante (de ~0,76 m de passada de caminhada para 1,07–1,52 m em corrida, para um adulto de ~1,83 m), e a cadência de corrida costuma ficar em 150–180 passos/min.

**Consequência de design:** o comprimento do passo **não** deve ser uma constante nem um valor por gait. Deve ser `f(velocidade)`, com saturação:

```
step_length = clamp(k_len * speed, 0, max_step_length_for_gait)
step_duration = step_length / max(speed, eps)   // com clamp em [min_dur, max_dur]
```

Isso já entrega o requisito R8 (correr = arco maior e mais rápido) quase de graça, sem código especial de corrida.

### 2.4 Estratégias de curva: *step turn* vs. *spin turn*

Ao mudar de direção durante a marcha, humanos usam duas estratégias distintas:

- **Step turn** — o giro é feito pisando com o membro **contralateral** (externo) na nova direção. Ex.: virando para a esquerda, o pé direito pisa para a esquerda. **Alarga a base de suporte**, facilita o deslocamento do centro de massa, é a estratégia **mais estável** e de menor custo biomecânico. É a preferida por adultos jovens saudáveis.
- **Spin turn** — o giro é feito pivotando sobre o membro **ipsilateral** (interno). Ex.: virando para a direita, pivota sobre o pé direito. Mantém a propulsão, mas o centro de massa fica fora da base de suporte durante boa parte do apoio, exigindo mais amplitude de rotação no plano transverso e mais coordenação. É **menos estável** e associado a maior risco de tropeço.

Além disso, o spin turn se subdivide em **pivô ipsilateral** e **cruzamento ipsilateral** (*ipsilateral crossover*) — ou seja, o único caso em que humanos realmente cruzam as pernas em locomoção normal é justamente uma variante de spin turn.

**Consequência de design (importante para R3 e R6):**
- A regra padrão do planejador de curvas deve ser **step turn**: quando o yaw desejado muda, prefira dar o próximo passo com o pé do lado **oposto** ao sentido do giro. Isso é uma simples regra de seleção de pé, e resolve curvas de forma naturalmente estável.
- O crossover não deve ser proibido *em absoluto* na arquitetura — deve ser proibido **por política configurável**, porque ele existe na natureza e pode ser desbloqueado depois para movimentos especiais (esquiva lateral, footwork de luta). Na Fase 0: `allow_crossover = false`.

### 2.5 Planejamento antecipado e replanejamento

Humanos planejam colocação de pés com vários passos de antecedência quando o terreno é complexo; quando um obstáculo invalida o alvo já planejado, ocorre um **redirecionamento rápido** do pé em pleno voo, com custo em estabilidade e progressão.

**Consequência de design:** a trajetória de balanço **não pode ser um caminho rígido calculado no lift-off**. Ela precisa ser um caminho que persegue um alvo possivelmente móvel, com replanejamento a cada tick e suavização. Isso é também o que torna R9 (parar no meio) implementável de forma limpa.

---

## 3. Robótica bípede: planejamento de passos

### 3.1 Footstep planning como busca discreta

O trabalho seminal de Kuffner et al. (planejamento online de passos, robô H7, CMU) estabeleceu o formato canônico: dado um **conjunto discreto de posições de passo plausíveis**, busca-se uma sequência de passos até um objetivo usando programação dinâmica sobre um grafo de transições de passos, com funções de custo heurísticas que penalizam o número e a complexidade dos passos. A vantagem sobre navegação de robôs com rodas é justamente poder **pisar sobre ou passar por cima de** obstáculos.

Formulações modernas representam cada passo como **um ponto 3D + um vetor de direção (yaw)** — a pose SE(2) ou SE(3) do pé. Anexar o heading ao passo é o que permite, sem casos especiais, **caminhada lateral e giro no lugar**.

**Consequência de design [INVARIANTE]:** o tipo fundamental do sistema é `Footstep { position: vec3, yaw/orientation, foot: Left|Right, timing, confidence }`. Tudo — andar de lado, andar de costas, curvar, girar parado — é uma sequência desse mesmo tipo. Não existirá "modo lateral" ou "modo ré" no código.

Na Fase 0 não precisamos de busca em grafo (não há objetivo global; há um vetor de intenção contínuo). Mas a estrutura de dados deve já ser a de um **plano de passos** (fila de 1–3 passos futuros), para que um planejador de horizonte maior possa ser encaixado depois sem reescrever nada.

### 3.2 Modelos de estabilidade: ZMP, LIPM, Capture Point

Embora a Fase 0 não simule dinâmica do corpo, a arquitetura precisa deixar espaço para estes conceitos porque eles são exatamente o que R13 (equilíbrio do ragdoll ativo) vai precisar.

- **Polígono de suporte** — área convexa delimitada pelos pontos de contato dos pés. Em apoio simples é a sola de um pé; em duplo apoio, o fecho convexo dos dois.
- **ZMP (Zero Moment Point)** — ponto no solo onde o momento resultante das forças de gravidade e inércia é nulo. Se o ZMP está dentro do polígono de suporte, o movimento é dinamicamente estável.
- **LIPM (Linear Inverted Pendulum Model)** — aproxima o corpo por uma massa pontual a altura constante sobre uma perna telescópica. Fornece dinâmica linear e solução analítica.
- **Capture Point / ICP (Instantaneous Capture Point)** — sob o LIPM, o ponto no solo onde o bípede deve pisar para que o centro de massa **pare**. Formulação clássica:

  ```
  ω  = sqrt(g / z_com)            // frequência natural do pêndulo
  ξ  = x_com + v_com / ω          // capture point (2D, no plano do solo)
  ```

  Se o capture point cai **dentro** do polígono de suporte atual, dá para recuperar o equilíbrio **sem** dar um passo (estratégia de tornozelo/quadril). Se cai fora, é preciso **pisar** de forma que a nova base de suporte intersecte a região de captura.

**Consequência de design:** o `FootstepPlanner` deve calcular o alvo de passo como

```
alvo = alvo_nominal(intenção) + termo_de_equilíbrio(estado_do_corpo)
```

onde, na Fase 0, `termo_de_equilíbrio` é uma função que retorna zero (não há corpo dinâmico), mas cuja **assinatura e ponto de injeção já existem**. Na fase do ragdoll ativo, essa função passa a retornar a correção derivada do capture point. Isso é a diferença entre "adaptar" e "reescrever" o sistema depois.

### 3.3 Realimentação de colocação de pé (SIMBICON)

Em animação baseada em física, o SIMBICON (SIMple BIped locomotion CONtrol) mostrou que uma **realimentação linear simples** de colocação de pé é suficiente para manter equilíbrio durante locomoção, com robustez notável a empurrões e a variações não antecipadas de terreno. A ideia, em forma reduzida:

```
offset_do_pé = c_d * d + c_v * v
```

onde `d` é o deslocamento horizontal do centro de massa em relação ao pé de apoio e `v` sua velocidade. O sucessor GENBICON substituiu essa heurística por um modelo de pêndulo invertido explícito para a colocação do pé.

**Consequência de design:** é exatamente a mesma injeção do §3.2. Dois ganhos (`c_d`, `c_v`) expostos na config, zerados na Fase 0.

### 3.4 Alcançabilidade (reachability) e viabilidade cinemática

Trabalhos recentes formalizam a viabilidade de um passo através de **mapas de alcançabilidade** em SE(2)/SE(3): dada a pose do pé de apoio, qual é a região alcançável para o pé em balanço, e como ela varia com a orientação do pé. A ideia central é que a região viável **não é um círculo** — é uma região anular, assimétrica, dependente do yaw relativo.

**Consequência de design:** modelaremos a região viável analiticamente (§10.1) como interseção de restrições simples, o que é barato, depurável e ajustável. A estrutura fica pronta para, no futuro, ser substituída por uma tabela/mapa amostrado do esqueleto real, sem mudar o contrato do validador.

### 3.5 Percepção de terreno e seleção de footholds

A linha de pesquisa de locomoção perceptiva (mapas de elevação, segmentação em regiões planares, mapas poligonais) converge em um padrão de **critérios de avaliação de foothold** aplicados a candidatos:

- **Rugosidade do terreno (terrain roughness)** — avalia média e desvio-padrão da inclinação em relação aos vizinhos no heightmap; candidatos acima de um limiar são descartados. Isso rejeita quinas e arestas.
- **Colisão da perna (leg collision)** — verifica se a configuração da perna correspondente ao candidato colide com o terreno durante todo o ciclo (lift-off → balanço → touchdown → próximo lift-off).
- **Margem de segurança de borda** — as regiões planares detectadas são **encolhidas para dentro** antes de serem usadas, criando margem contra pisar na quina ou cair da beirada.

Além disso, uma limitação conhecida de heightmaps 2.5D é que eles armazenam **uma única altura por (x, y)** — o que falha em escadas com espelho vazado, plataformas sobrepostas e passagens sob mesas.

**Consequência de design:**
- Não usaremos heightmap. Usaremos **consulta direta ao PhysX** (raycast/sweep/overlap), que naturalmente lida com geometria 3D arbitrária e objetos dinâmicos — e é o que a engine já tem.
- Adotaremos os três critérios acima como o **núcleo do avaliador de foothold** (§11.3), com o encolhimento de borda como conceito de primeira classe.
- Adotaremos **busca por candidato alternativo** quando o alvo nominal reprova, em vez de simplesmente falhar.

### 3.6 Trajetória do pé em balanço

Duas famílias dominam a literatura:

- **Ciclóide** — usada classicamente (p.ex. no gerador de marcha do HUBO/KHR-3) para a posição do tornozelo em balanço. A propriedade decisiva: a ciclóide tem **velocidade e aceleração nulas nas extremidades**, o que produz lift-off e touchdown suaves (sem escorregar na saída, sem impacto na chegada) e mantém continuidade quando o período e a amplitude do passo mudam.
- **Bézier / splines cúbicas** — derivadas de ordem superior contínuas, controle fácil de folga sobre obstáculo via pontos de controle, e amplamente usadas em robôs com pernas.

Para degraus e diferenças de altura, uma técnica prática é inserir **waypoints intermediários** na trajetória quando é detectada diferença de altura entre passos consecutivos, posicionados em frações pré-definidas do caminho entre origem e destino, garantindo folga na subida e transição suave na descida.

**Consequência de design:** trajetória **separável** — perfil horizontal e perfil vertical desacoplados, ambos parametrizados por uma **fase normalizada `s ∈ [0,1]`**:

- Horizontal: interpolação suave (ease-in-out estilo ciclóide) do ponto de lift-off ao alvo corrente.
- Vertical: perfil de folga (sino) + termo de degrau + waypoints quando há Δh.

Parametrizar por `s` — e não por tempo absoluto — é o que torna possível **congelar, desacelerar, reverter ou abortar** o passo no meio (R9), e perseguir um alvo móvel sem descontinuidade.

---

## 4. Locomoção em jogos: o que a indústria resolveu

### 4.1 Motion matching e o problema da responsividade

*Motion matching* (Ubisoft, GDC 2016) substituiu grafos de animação por uma busca contínua: a cada frame, busca-se no banco de mocap o quadro que simultaneamente casa com a **pose atual** e com a **trajetória futura desejada**, e faz-se um blend curto. Resultado praticamente indistinguível de mocap cru, com responsividade suficiente para controle confortável. A Unreal implementa isso via *Pose Search*.

### 4.2 Warping: o combate ao foot skating

As técnicas complementares que a indústria usa sobre animação:

- **Distance matching** — sincroniza o quadro da animação com a distância física realmente percorrida.
- **Stride warping** — escala o comprimento da passada para casar com a velocidade real.
- **Orientation warping** — injeta/remove rotação para casar com a direção real.
- **Foot IK / foot planting** — pós-processo que fixa o pé no ponto de contato detectado por raycast e alinha à normal da superfície.

Um detalhe reportado no trabalho de *Gears of War 4*: uma heurística barata que **casa a fase do pé** entre animações em transição melhora sensivelmente a qualidade das transições.

**Consequência de design:** nós estamos, deliberadamente, indo pelo caminho oposto — em vez de corrigir animação para casar com o movimento, **geramos o movimento primeiro e a animação (via IK) depois**. Isso elimina foot skating por construção: o pé de apoio simplesmente **não se move**, porque é o planejador que decide onde ele está. Esse é o argumento central a favor da abordagem escolhida, e a razão de validar o footwork isolado antes do esqueleto.

Mas há uma lição a importar: **fase é um conceito de primeira classe**. O `phase ∈ [0,1]` por pé deve ser exposto na API pública desde o início — é o que o sistema de animação futuro vai consumir.

### 4.3 Disparo de passo procedural (padrão "spider IK")

O padrão consagrado em animação procedural de criaturas: cada perna tem uma **posição de repouso (home)** ancorada ao corpo; quando a distância entre o pé plantado e sua home ultrapassa um limiar, dispara-se um passo até a nova home projetada no terreno; um bloqueio impede que pernas adjacentes/opostas deem passo simultaneamente.

Isso é simples, robusto e **naturalmente sem passo mínimo** — o passo tem exatamente o tamanho do erro acumulado. É precisamente o mecanismo que atende R10 e R9.

**Consequência de design [INVARIANTE]:** o disparo de passo é **baseado em erro**, não em relógio. Não existe "timer de passo" no núcleo. O tempo entra apenas como *duração* do balanço, uma vez que o passo já foi disparado, e como *histerese* para evitar oscilação.

---

## 5. Síntese: as dez decisões que a teoria dita

| # | Decisão | Origem |
|---|---|---|
| D1 | Corpo se move **continuamente**; pés se movem **discretamente**. Existe um *root virtual* mesmo sem esqueleto. | §2.1, §4.3 |
| D2 | Passo = `posição 3D + yaw`. Nenhum modo direcional especial. | §3.1 |
| D3 | Disparo de passo por **erro em relação à home**, não por timer. Sem passo mínimo. | §4.3, R10 |
| D4 | `duty_factor` unifica caminhar e correr. | §2.1 |
| D5 | `step_length = f(velocidade)`, saturado. | §2.3 |
| D6 | Curva usa **step turn** (pé contralateral) por padrão. Crossover é política, não constante. | §2.4 |
| D7 | Alvo = `nominal(intenção) + correção_de_equilíbrio(estado)`, com a segunda parcela zerada na Fase 0. | §3.2, §3.3 |
| D8 | Trajetória parametrizada por **fase `s`**, horizontal e vertical desacoplados, alvo **móvel**. | §3.6, §2.5 |
| D9 | Foothold é **avaliado e reprojetado**, nunca apenas aceito: rugosidade, colisão, margem de borda. | §3.5 |
| D10 | Consulta de terreno é uma **porta abstrata**; PhysX é apenas uma implementação. | §6 |

---

# PARTE II — ARQUITETURA

## 6. Princípios arquiteturais

**[INVARIANTE] O módulo `footwork` não conhece:** PhysX, o renderizador, o sistema de input, o esqueleto, o loop principal da engine.

Ele expõe **quatro portas**:

```
                      ┌──────────────────────────────┐
   LocomotionInput ──▶ │                              │ ──▶ FootworkOutput
   (struct pura)       │      FootworkSystem          │     (poses-alvo, fases,
                       │        (núcleo puro)         │      polígono de suporte)
   ITerrainProbe  ◀──▶ │                              │ ──▶ DebugDrawBuffer
   (interface)         └──────────────────────────────┘     (primitivas puras)
```

- **Entrada de intenção:** `LocomotionInput` — struct simples, sem dependências. Quem preenche é a camada de input do laboratório.
- **Porta de terreno:** `ITerrainProbe` — interface abstrata. `PhysXTerrainProbe` é a implementação, e vive na camada de **adaptadores**, fora do núcleo.
- **Saída de estado:** `FootworkOutput` — o que o resto da engine consome (hoje: o renderizador de debug; amanhã: o IK).
- **Saída de debug:** buffer de primitivas geométricas (linhas, polilinhas, quads, texto). O núcleo **não desenha**; ele descreve o que desenhar.

**Por que assim:** essa é a única configuração em que o núcleo é (a) testável sem PhysX e sem janela, (b) reutilizável pelo ragdoll ativo depois, (c) imune a virar monólito. Se o núcleo incluir `PxScene.h`, o design falhou.

**[ABERTO]** Convenções da engine: sistema de coordenadas (Y-up vs Z-up), handedness, tipos matemáticos (`Vec3`/`Quat` próprios ou `PxVec3`), unidades (assumir metros), estilo de nomes, sistema de build/módulos, alocação de memória. **Siga o que a engine já usa.** As assinaturas abaixo são conceituais.

---

## 7. Modelo de dados

### 7.1 Entrada

```cpp
struct LocomotionInput {
    Vec2  move_axis;        // WASD normalizado, espaço local: x = strafe, y = forward
    float look_yaw;         // radianos, direção do olhar no mundo — REFERENCIAL PRINCIPAL
    float look_yaw_rate;    // derivada, usada para antecipar curvas
    bool  run;              // modificador de gait
    bool  crouch;           // reservado
    float dt;
};
```

**[INVARIANTE]** `move_axis` é interpretado **no frame do olhar**. A direção de deslocamento no mundo é:

```
world_dir = Rot(look_yaw) * normalize(vec3(move_axis.x, 0, move_axis.y))
```

Andar de lado, de costas e em diagonal caem todos nessa mesma expressão. Não há ramificação por direção — isso é R1/R2 resolvido em uma linha.

### 7.2 Estado do "root virtual"

Sem esqueleto, ainda precisamos de um referencial contínuo que os pés seguem. Chamemos de `VirtualRoot` — conceitualmente a projeção do quadril no solo.

```cpp
struct VirtualRoot {
    Vec3  position;         // no solo (ou a uma altura nominal)
    float yaw;              // segue look_yaw com suavização
    Vec3  velocity;
    float height;           // altura nominal do quadril — usada nos limites de alcance
};
```

O `VirtualRoot` se move de forma **contínua e suave** (aceleração/desaceleração limitadas), enquanto os pés se movem de forma **discreta**. Essa separação é a espinha dorsal de todo o sistema (D1).

**[ABERTO]** Se o root virtual deve ser puramente cinemático nesta fase ou já um corpo PhysX cinemático/dinâmico. Recomendação: **cinemático na Fase 0** — validar footwork sem acoplar dinâmica.

### 7.3 Passo e pé

```cpp
enum class FootSide { Left, Right };

enum class FootPhase {
    Planted,        // apoiado, estático
    LiftOff,        // saindo do solo (janela curta)
    Swing,          // em voo, perseguindo alvo
    TouchDown,      // encostando (janela curta)
    Aborting        // abortando para o ponto seguro mais próximo (R9)
};

struct Footstep {
    Vec3      position;         // ponto de contato no mundo
    Vec3      normal;           // normal da superfície no contato
    float     yaw;              // orientação do pé no mundo
    FootSide  side;
    float     quality;          // [0,1] score do avaliador de foothold
    TerrainClass terrain;       // Flat / Slope / StepUp / StepDown / Edge / Gap / Unstable
};

struct FootState {
    FootSide   side;
    FootPhase  phase;
    float      s;               // fase normalizada do balanço [0,1]  <- CHAVE
    Footstep   planted;         // último passo confirmado
    Footstep   target;          // alvo corrente (pode mudar durante o balanço!)
    Vec3       lift_off_pos;
    float      lift_off_yaw;
    SwingArc   arc;             // trajetória amostrável corrente
    float      swing_duration;
};
```

### 7.4 Saída

```cpp
struct FootworkOutput {
    FootState    left, right;
    SupportPolygon support;     // fecho convexo dos contatos ativos
    Vec3         support_center;
    float        stability_margin;  // distância do ponto de referência à borda do polígono
    GaitMode     gait;
    // Futuro: capture_point, zmp
};
```

---

## 8. Pipeline por tick

**[INVARIANTE]** Ordem fixa, uma passada, sem recursão:

```
1. IntentResolver     — input + look_yaw       → direção/velocidade desejadas no mundo
2. RootIntegrator     — atualiza VirtualRoot (posição, yaw, velocidade) com limites de acel.
3. GaitPlanner        — decide gait, duty_factor, step_length, swing_duration
4. HomeSolver         — calcula a posição "home" (repouso) de cada pé para o root atual
5. StepTrigger        — decide SE e QUAL pé deve iniciar um passo (baseado em erro)
6. FootstepPlanner    — calcula o alvo nominal do passo
7. ConstraintValidator— valida e REPROJETA o alvo (alcance, crossover, yaw, colisão)
8. TerrainResolver    — consulta ITerrainProbe, avalia footholds, escolhe o final
9. SwingSolver        — (re)gera/atualiza o arco; avança s; detecta touchdown
10. SupportSolver     — recalcula polígono de suporte e margem
11. DebugEmitter      — preenche o DebugDrawBuffer
```

Cada etapa é uma função/classe própria, com entrada e saída explícitas. Etapas 6–8 formam um sub-loop de **propor → validar → corrigir** que roda também durante o balanço (alvo móvel, §3.6).

**[ABERTO]** Se o sistema roda em fixed timestep próprio (recomendado: 60–120 Hz, sincronizado com o passo de simulação do PhysX) ou no tick variável da engine. Recomendação: **fixed step com interpolação para render** — o comportamento fica determinístico e depurável.

### 8.1 IntentResolver

```
desired_dir_world = Rot(look_yaw) * normalize(vec3(move.x, 0, move.y))
desired_speed     = |move| * gait.max_speed
desired_yaw       = look_yaw     // pés seguem o olhar, não o movimento
```

**[INVARIANTE]** O yaw do corpo/pés segue o **olhar**, não a direção de deslocamento. É isso que permite andar de lado e de costas mantendo a orientação — R2.

---

## 9. Planejamento de passos

### 9.1 Home position (posição de repouso)

Para cada pé, a home é onde ele "deveria" estar dado o estado atual do root:

```
home_local(side) = vec3(±step_width/2, 0, 0)     // offset lateral, sinal por lado
home_world(side) = root.position
                 + Rot(root.yaw) * home_local(side)
                 + desired_dir_world * (step_length * 0.5)      // antecipação
                 + Rot(root.yaw) * balance_offset(side)          // §3.3 — ZERO na Fase 0
```

O termo de antecipação faz o pé mirar à frente do root em vez de sob ele — o que é o comportamento humano e evita o efeito de "pés arrastados atrás do corpo".

**[ABERTO]** Se a antecipação deve escalar com `look_yaw_rate` para antecipar curvas (recomendado: sim, com ganho pequeno) — isso melhora bastante R3.

### 9.2 Disparo de passo (StepTrigger) — **[INVARIANTE]**

```
erro_pos(side) = |plantado(side).position - home_world(side)|
erro_yaw(side) = |wrap(plantado(side).yaw - (root.yaw + toe_out(side)))|

deve_passar(side) =
      erro_pos > limiar_pos(velocidade)
   || erro_yaw > limiar_yaw
   || !ainda_valido(plantado(side))         // terreno mudou, ficou inválido, cruzou
   || equilíbrio_exige_passo(side)          // Fase futura: capture point fora do suporte
```

**Não existe passo mínimo.** Se o erro é de 4 cm, o passo é de 4 cm. Se é de 60 cm, o passo é de 60 cm (limitado pelo alcance). Essa é a diferença entre um sistema que serve para caminhar e um sistema que serve **também** para equilibrar-se (R13) — micro-ajustes são a essência do equilíbrio.

`limiar_pos` deve ser **função da velocidade** (baixo em velocidade baixa, permitindo ajustes finos; maior em velocidade alta, evitando passos picotados) e deve ter **histerese** (limiar de disparo > limiar de re-disparo) para não oscilar.

### 9.3 Seleção de qual pé mover

```
1. Se um pé já está em Swing → nenhum outro passo é iniciado (a menos que gait permita voo, §12).
2. Se ambos podem passar → escolhe o de MAIOR custo:
       custo(side) = w_pos * erro_pos + w_yaw * erro_yaw + w_turn * bônus_step_turn(side)
3. bônus_step_turn: se está girando para a esquerda, o pé DIREITO ganha bônus (§2.4).
4. Histerese/alternância: penaliza mover o mesmo pé duas vezes seguidas, mas NÃO proíbe
   (dois passos seguidos do mesmo pé são legítimos em ajustes laterais e em correções).
5. Nunca mover o pé que é o único apoio válido se o outro estiver em estado inválido.
```

**[ABERTO]** Pesos e forma exata da função de custo. Deixe-os na config e calibre no laboratório.

### 9.4 Alvo nominal do passo

```
step_len   = clamp(k_len * speed, 0, gait.max_step_length)
alvo_pos   = home_world(side) + desired_dir_world * (step_len * 0.5)
alvo_yaw   = root.yaw + toe_out(side)     // toe-out ~5–8°, sinal por lado
```

Durante curvas, como `root.yaw` já está seguindo `look_yaw`, os pés naturalmente giram passo a passo — R3 emerge sem código especial. O bônus de step turn (§9.3) é o que dá o **caráter** correto à curva.

### 9.5 Alvo móvel durante o balanço

A cada tick, enquanto `phase == Swing`:

```
novo_alvo = planejar(side)                     // etapas 6–8 novamente
alvo_corrente = lerp(alvo_corrente, novo_alvo, k_chase * dt * (1 - s))
```

O fator `(1 - s)` faz o alvo **congelar progressivamente** conforme o pé se aproxima do solo: nos primeiros 20% do balanço o pé é muito redirecionável; nos últimos 20%, praticamente comprometido. Isso reproduz o comportamento de replanejamento humano (§2.5) e evita jitter no touchdown.

### 9.6 Parada no meio do trajeto — R9 **[INVARIANTE]**

Quando o input é solto no meio de um passo:

```
Se s < s_commit (padrão 0.5):
    → estado Aborting
    → novo alvo = ponto seguro mais próximo entre lift_off_pos e alvo, validado pelo terreno
    → o pé desce imediatamente pelo perfil vertical restante (encurtado)
Se s >= s_commit:
    → completa o passo, mas com o alvo ENCURTADO para o novo (menor) erro de home
    → swing_duration é reescalado, não o s
```

Em ambos os casos o pé **planta em posição válida** — nunca fica congelado no ar, nunca desliza para a posição original. É isso que dá o controle preciso e personalizado descrito no briefing.

O root, em paralelo, desacelera com limite de desaceleração. Como o disparo é por erro (§9.2), assim que o root para o erro deixa de crescer e o sistema entra em repouso naturalmente — sem estado especial de "idle".

---

## 10. Restrições — o "impossível não acontece"

Todas as restrições operam em **reprojeção**, não em rejeição. Um alvo inválido é **empurrado para dentro** da região viável; só se recorre a fallback quando não há solução.

### 10.1 Alcance (reachability) — R7

No frame do pé de apoio (ou do root):

```
r = |alvo - ancoragem_do_quadril(side)|
R1  <= r <= R2                       // região ANULAR, não círculo (§3.4)
R2 ≈ 0.85–0.90 * comprimento_da_perna
R1 ≈ pequeno, evita o pé colapsar sob o quadril
|alvo.z_local| <= max_step_up / max_step_down    // limites verticais assimétricos
```

Reprojeção: clamp radial + clamp vertical.

### 10.2 Anti-crossover — R6 **[INVARIANTE]**

No frame do **pé de apoio** (eixo lateral = direita do root):

```
lateral = dot(alvo - apoio.position, right_vector(root.yaw))
sinal_esperado = (side == Right) ? +1 : -1

Exigir:  sinal_esperado * lateral >= step_width_min      // padrão 0.06 m
```

Isto é a codificação direta do fato biomecânico de que a largura do passo humana fica em 5–10 cm e **nunca inverte de sinal** em marcha normal (§2.2). Reprojeção: empurrar o alvo lateralmente até satisfazer o mínimo.

Adicionalmente, testar **sobreposição dos volumes dos pés** (§11.4) — a restrição lateral resolve o caso geral, o teste de sobreposição resolve os cantos.

**[ABERTO]** `allow_crossover` na config, default `false`. A arquitetura deve permitir ligá-lo sem cirurgia (§2.4).

### 10.3 Yaw relativo

```
|wrap(alvo.yaw - apoio.yaw)| <= max_foot_yaw_delta        // padrão ~45°
```

Impede a "torção impossível" entre os pés. Reprojeção: clamp do yaw.

### 10.4 Ordem de aplicação e fallback

```
alvo := nominal
alvo := clamp_alcance(alvo)
alvo := empurra_anticrossover(alvo)
alvo := clamp_yaw(alvo)
alvo := resolve_terreno(alvo)          // §11 — pode MOVER o alvo
se ainda inválido:
    alvo := busca_candidato_alternativo(alvo)      // §11.3
se ainda inválido:
    alvo := passo_de_ajuste(side)      // passo curto e conservador para a home atual
se ainda inválido:
    NÃO dá o passo; mantém o plantado e sinaliza no debug  // nunca crashar, nunca teleportar
```

**[INVARIANTE]** O sistema nunca produz um alvo inválido. Em pior caso, ele não dá o passo — e isso aparece visivelmente no overlay de debug.

---

## 11. Terreno

### 11.1 A porta abstrata

```cpp
struct GroundSample {
    bool  hit;
    Vec3  position;
    Vec3  normal;
    float slope_rad;
    uint32_t material_id;
    bool  is_dynamic;        // objeto físico móvel — afeta confiança
};

class ITerrainProbe {
public:
    // Raycast vertical (de cima para baixo) num ponto
    virtual GroundSample sampleGround(const Vec3& xz, float search_up, float search_down) = 0;

    // Amostragem multi-ponto sob a sola → plano ajustado + estatística de rugosidade
    virtual FootPatch sampleFootPatch(const Vec3& center, float yaw, const Vec2& foot_size) = 0;

    // Sweep do volume do pé ao longo de um caminho → primeira obstrução
    virtual SweepResult sweepFoot(const Vec3& from, const Vec3& to, float yaw) = 0;

    // Overlap estático do volume do pé numa pose
    virtual bool overlapFoot(const Vec3& at, float yaw) = 0;
};
```

**[ABERTO]** Mapeamento para PhysX 5. O SDK oferece `PxScene::raycast/sweep/overlap`; a estrutura de pruning é separada para estáticos e dinâmicos, o que é relevante para filtragem e custo. Considere `PxQueryFilterData` para excluir os próprios pés e o root das consultas, e avalie batching (`PxBatchQueryExt`) apenas se o profiling mostrar necessidade — a documentação do próprio SDK observa que batching existe majoritariamente por compatibilidade e não garante ganho.

### 11.2 Amostragem do apoio (FootPatch)

Para cada foothold candidato, amostrar **5 pontos** (centro + 4 cantos da sola, no yaw do pé):

```
- Ajustar um plano por mínimos quadrados aos hits → normal do apoio
- rugosidade = desvio-padrão dos resíduos verticais
- inclinação  = ângulo entre a normal e o "up"
- cobertura   = fração de raios que acertaram (detecta borda/vão)
```

Isso é a versão barata e direta do critério de rugosidade da literatura de locomoção perceptiva (§3.5) — sem construir heightmap.

### 11.3 Avaliação e busca de foothold

```
score(candidato) =
      w1 * (1 - normalizado(rugosidade))
    + w2 * (1 - normalizado(inclinação))
    + w3 * cobertura
    + w4 * margem_de_borda
    - w5 * penalidade_de_desvio(|candidato - alvo_nominal|)
    - w6 * penalidade_dinâmica(is_dynamic)

rejeitar se: inclinação > max_slope (~35°)
           ou cobertura < min_coverage (~0.8)
           ou margem_de_borda < min_edge_margin (~0.03 m)
```

Se o nominal reprova, buscar em **anéis concêntricos** (p.ex. raios 0,05 / 0,10 / 0,15 m, 8 amostras angulares por anel), escolher o de maior score que satisfaça também as restrições do §10. Essa é a estratégia de "região segura mais próxima" da literatura de adaptação visual de foothold.

**Margem de borda:** amostrar alguns pontos ao redor do candidato; se algum deles cai muito abaixo (ou não acerta nada), o candidato está perto de uma quina/beirada e recebe penalidade forte. É o análogo direto do "encolhimento" das regiões planares (§3.5).

### 11.4 Escadas e degraus — R5

Detecção:

```
Δh = alvo.y - apoio.y

|Δh| < step_tolerance (~0.03 m)            → Flat
Δh > 0 e Δh <= max_step_up  (~0.20 m)      → StepUp
Δh < 0 e |Δh| <= max_step_down (~0.25 m)   → StepDown
inclinação entre 5° e max_slope             → Slope
cobertura parcial + salto vertical brusco   → Edge
sem hit                                     → Gap → foothold rejeitado
```

Os limites vêm da geometria real de escadas (espelho padrão ~17–19 cm), o que dá `max_step_up = 0.20 m` como valor defensável.

**Ajuste da trajetória em degraus (§3.6):** quando `terrain != Flat`, inserir **dois waypoints** na trajetória em frações configuráveis do caminho — o primeiro logo após o lift-off, elevado o suficiente para limpar a quina; o segundo próximo ao alvo, acima dele, para descida vertical limpa. Isso resolve tanto subir (não bater o pé na quina) quanto descer (não "raspar" a borda).

### 11.5 Objetos físicos dinâmicos

Objetos dinâmicos são footholds válidos, mas com **confiança reduzida**: penalizar no score, reduzir o limiar de commit (§9.5) e re-validar o pé plantado a cada N ticks (se a caixa em que ele pisou se moveu, `ainda_valido` retorna falso e um novo passo é disparado — §9.2). Essa re-validação periódica do apoio é o que faz o sistema reagir a plataformas móveis e caixas empurradas sem nenhum código específico para isso.

---

## 12. Gaits: caminhar e correr — R8

```cpp
struct GaitProfile {
    float max_speed;
    float k_step_length;        // step_length = k * speed
    float max_step_length;
    float duty_factor;          // >0.5 = duplo apoio (walk); <0.5 = fase de voo (run)
    float base_clearance;       // altura do arco
    float swing_speed_scale;
    float step_trigger_scale;
    bool  allow_flight;         // permite ambos os pés em Swing
    float toe_out_angle;
};
```

Valores iniciais sugeridos (escala humana, metros/segundos):

| Parâmetro | Walk | Run |
|---|---|---|
| `max_speed` | 1,4 | 4,5–6,0 |
| `k_step_length` | 0,50 | 0,45 |
| `max_step_length` | 0,75 | 1,40 |
| `duty_factor` | 0,60 | 0,35 |
| `base_clearance` | 0,06 | 0,14 |
| `allow_flight` | false | true |
| `toe_out_angle` | ~7° | ~5° |

Como `step_length` já é `f(velocidade)` (D5), **correr é literalmente só um perfil de parâmetros diferente** — não há um "sistema de corrida". Isso satisfaz R8 sem duplicar lógica. A transição entre perfis deve ser **interpolada** ao longo de ~0,2–0,3 s, não instantânea.

**[ABERTO]** Se `allow_flight` deve realmente permitir dois pés em Swing na Fase 0. Recomendação: implementar o flag mas manter `false` até a Fase 4 — fase de voo introduz casos de borda que atrapalham a calibração inicial.

---

## 13. Gancho para o ragdoll ativo — R13

Nada disso é implementado agora. O que é implementado agora é o **espaço** para isso:

```cpp
// Interface opcional injetada; null na Fase 0
class IBalanceProvider {
public:
    virtual Vec3  comPosition() const = 0;
    virtual Vec3  comVelocity() const = 0;
    virtual float comHeight()   const = 0;
};

// No FootstepPlanner:
Vec3 balanceOffset(FootSide side) const {
    if (!balance_) return Vec3::zero();          // <-- Fase 0
    const float w  = sqrt(GRAVITY / balance_->comHeight());
    const Vec3  cp = balance_->comPosition() + balance_->comVelocity() / w;   // capture point
    // ...deslocar o alvo em direção ao capture point, com ganho e clamp
}

// No StepTrigger:
bool balanceRequiresStep() const {
    if (!balance_) return false;                 // <-- Fase 0
    return !support_polygon.contains(capturePoint());   // §3.2
}
```

Esses dois pontos de injeção — **um no alvo, um no disparo** — são suficientes para transformar este planejador de footwork num controlador de equilíbrio por passo (§3.2, §3.3), sem reescrever nada. É por isso que R10 (sem passo mínimo) é inegociável: recuperar equilíbrio é feito com passos de 3 cm tanto quanto com passos de 50 cm.

---

## 14. Overlay de debug — R4

O núcleo emite primitivas; a engine desenha. Elementos:

| Elemento | Representação | Propósito |
|---|---|---|
| **Arco do passo** | Polilinha de ~24–32 amostras da trajetória corrente | R4 — o pedido central |
| Arco **planejado vs. percorrido** | Cores distintas, separadas em `s` | Ver o replanejamento acontecendo |
| Footprint alvo | Retângulo orientado no solo, cor por `quality` | Ver a decisão do planejador |
| Footprint plantado | Retângulo preenchido | Ver o estado real |
| Home position | Cruz / círculo pequeno | Ver o erro que dispara o passo |
| Vetor de erro | Linha plantado → home | Entender por que um passo disparou |
| Região de alcance | Anel/setor no solo | Ver por que um alvo foi reprojetado |
| Linha média (anti-crossover) | Linha no frame do apoio | Ver a restrição R6 atuando |
| Polígono de suporte | Polígono preenchido translúcido | Preparar R13 |
| Candidatos de foothold rejeitados | Pontos vermelhos com motivo | Depurar terreno |
| Normais amostradas | Pequenos vetores nos 5 pontos | Depurar leitura de terreno |
| HUD | Texto: gait, speed, s, fase, step_length, duração, motivo do último disparo | Calibração |

**[ABERTO]** Como isso se conecta ao debug renderer existente da engine. Não crie um renderizador novo.

Cada categoria deve ter um **toggle independente** — na calibração, ver tudo ao mesmo tempo é inútil.

---

## 15. Integração com o laboratório — R12

```
Toolbar → botão "Footwork" → popup com:
    [ ] Modo Footwork ativo (entrar/sair)
    Gait: (o) Walk  ( ) Run  — ou automático por modificador
    Toggles de debug (lista do §14)
    Sliders dos parâmetros principais (§16), com aplicação ao vivo
    Botões: Reset pés | Colocar no ponto do cursor | Congelar/Passo a passo
    Leitura: velocidade, cadência efetiva, comprimento do último passo, taxa de rejeição
```

**[INVARIANTE]** Entrar/sair do modo footwork não pode vazar estado: sair deve desregistrar o tick, limpar o debug buffer e liberar os atores criados. O modo é um **plugin do laboratório**, não uma modificação do laboratório.

O botão "Congelar / Passo a passo" (avançar um tick por clique) vale muito mais do que parece durante a calibração — permite inspecionar exatamente o tick em que uma decisão errada foi tomada.

---

## 16. Configuração

Tudo o que abaixo tem número deve ser **data-driven** (JSON/TOML/o que a engine já usa), recarregável a quente, e ajustável pelos sliders do popup.

```
FootworkConfig
├── body
│   ├── leg_length              0.90
│   ├── hip_width               0.20
│   ├── foot_size               (0.26 × 0.10)
│   └── root_height             0.95
├── stepping
│   ├── step_width_min          0.06
│   ├── step_width_nominal      0.10
│   ├── trigger_pos_threshold   0.12    (escalado por velocidade)
│   ├── trigger_yaw_threshold   0.35 rad
│   ├── trigger_hysteresis      0.6
│   ├── s_commit                0.5
│   └── target_chase_gain       8.0
├── limits
│   ├── max_reach_ratio         0.88    (× leg_length)
│   ├── min_reach               0.05
│   ├── max_step_up             0.20
│   ├── max_step_down           0.25
│   ├── max_slope               35°
│   ├── max_foot_yaw_delta      45°
│   └── allow_crossover         false
├── terrain
│   ├── probe_up / probe_down   0.60 / 1.20
│   ├── patch_samples           5
│   ├── max_roughness           0.02
│   ├── min_coverage            0.80
│   ├── min_edge_margin         0.03
│   ├── search_rings            [0.05, 0.10, 0.15]
│   └── search_samples_per_ring 8
├── swing
│   ├── vertical_profile        cycloid | bezier
│   ├── clearance_base          0.06
│   ├── clearance_per_dh        1.20
│   ├── step_waypoint_fracs     [0.25, 0.80]
│   └── touchdown_tolerance     0.01
└── gaits[]   (§12)
```

**[ABERTO]** Formato e mecanismo de hot-reload. Use o que a engine já tem.

---

## 17. Trajetória de balanço — detalhe

```
Dado: lift_off_pos P0, alvo corrente P1, fase s ∈ [0,1]

// Horizontal — perfil ciclóide (vel. e acel. nulas nas extremidades, §3.6)
u  = s - sin(2πs) / (2π)
xz = lerp(P0.xz, P1.xz, u)

// Vertical — base + folga + degrau
h_base   = lerp(P0.y, P1.y, smoothstep(s))
Δh       = P1.y - P0.y
clearance = clearance_base
          + clearance_per_dh * max(0, Δh)
          + obstacle_clearance                  // vindo de sweepFoot ao longo do caminho
h_arc    = clearance * sin(π * s)               // ou perfil assimétrico: pico em s≈0.4
y        = h_base + h_arc

// Waypoints de degrau (§11.4) quando terrain != Flat:
//   inserir P_wp1 em s=frac0 e P_wp2 em s=frac1, e passar uma spline pelos 4 pontos
```

Um detalhe que vale calibrar: humanos têm o pico de folga **antes** da metade do balanço. Um perfil assimétrico (pico em `s ≈ 0.4`) parece perceptivelmente mais natural que o simétrico. Deixe isso como parâmetro.

**Yaw durante o balanço:** interpolar `lift_off_yaw → target.yaw` por caminho curto (slerp/shortest-arc), com a mesma curva de easing.

**Detecção de touchdown:** quando `s → 1` **ou** quando um `sweepFoot` do tick detecta contato antecipado (terreno mais alto que o esperado). O segundo caso é o que faz o sistema não afundar em geometria — trate-o como touchdown antecipado válido, ajustando `planted` para o ponto de contato real.

---

# PARTE III — EXECUÇÃO

## 18. Estrutura de arquivos sugerida

**[ABERTO]** Adapte à convenção de módulos da engine. A separação em três anéis (núcleo / adaptadores / laboratório) é que é **[INVARIANTE]**.

```
modules/footwork/
├── include/footwork/
│   ├── Types.h                 // FootSide, FootPhase, Footstep, FootState, TerrainClass
│   ├── Config.h                // FootworkConfig, GaitProfile
│   ├── Input.h                 // LocomotionInput
│   ├── Output.h                // FootworkOutput, SupportPolygon
│   ├── ITerrainProbe.h         // porta de terreno
│   ├── IBalanceProvider.h      // gancho futuro (§13)
│   ├── DebugDraw.h             // primitivas puras de debug
│   └── FootworkSystem.h        // fachada — a ÚNICA classe pública
├── src/
│   ├── FootworkSystem.cpp      // orquestra o pipeline do §8; sem lógica própria
│   ├── IntentResolver.{h,cpp}
│   ├── VirtualRoot.{h,cpp}
│   ├── GaitPlanner.{h,cpp}
│   ├── HomeSolver.{h,cpp}
│   ├── StepTrigger.{h,cpp}
│   ├── FootstepPlanner.{h,cpp}
│   ├── ConstraintValidator.{h,cpp}
│   ├── TerrainResolver.{h,cpp} // usa ITerrainProbe; NÃO conhece PhysX
│   ├── SwingSolver.{h,cpp}
│   ├── SupportSolver.{h,cpp}
│   └── DebugEmitter.{h,cpp}
├── adapters/
│   └── PhysXTerrainProbe.{h,cpp}   // ÚNICO arquivo do módulo que inclui PhysX
├── lab/
│   ├── FootworkLabMode.{h,cpp}     // registro na toolbar, popup, ciclo de vida
│   └── FootworkDebugRenderer.{h,cpp} // traduz DebugDrawBuffer → renderer da engine
└── tests/
    ├── test_constraints.cpp
    ├── test_step_trigger.cpp
    ├── test_swing.cpp
    └── FakeTerrainProbe.h          // terreno sintético: plano, rampa, escada, buraco
```

**Teste de sanidade da arquitetura:** `grep -r "Px" src/` no núcleo deve retornar **zero** resultados. Se retornar, o acoplamento vazou.

---

## 19. Fases de implementação

Cada fase é entregável e testável isoladamente. Não avance sem os critérios de aceitação.

### Fase 1 — Esqueleto do módulo e plano
- Tipos, config, fachada, pipeline vazio, registro no laboratório, popup entrar/sair.
- `FakeTerrainProbe` retornando plano infinito em y=0.
- Dois pés renderizados como caixas, plantados em posições nominais.
- **Aceitação:** entrar/sair do modo sem vazamento; dois pés visíveis; nada se move.

### Fase 2 — Locomoção em plano
- `VirtualRoot`, `IntentResolver`, `HomeSolver`, `StepTrigger`, `FootstepPlanner`, `SwingSolver`.
- Restrições de alcance, anti-crossover, yaw.
- Overlay do arco (R4) e footprints.
- **Aceitação:** WASD relativo ao olhar move os pés em frente, ré, lateral e diagonal; curvas usam step turn; nenhum cruzamento visível; nenhum passo com comprimento acima do limite; overlay do arco visível e coerente.

### Fase 3 — Parada precisa e micro-passos
- Lógica de `Aborting`, `s_commit`, alvo móvel, reescalonamento de duração.
- **Aceitação:** soltar a tecla em qualquer ponto do balanço sempre resulta em pé plantado em posição válida; passos de 2–5 cm funcionam; nenhum "passo mínimo" observável; sem jitter em repouso.

### Fase 4 — Terreno
- `PhysXTerrainProbe`, `FootPatch`, avaliação e busca de foothold, classificação, degraus, waypoints, objetos dinâmicos.
- **Aceitação:** rampas até 35° OK; escada de degraus de até 20 cm sobe e desce sem atravessar geometria; pé nunca pisa a menos de 3 cm de uma borda; caixa dinâmica empurrada sob um pé plantado dispara re-passo; nenhum foothold aceito em vão.

### Fase 5 — Corrida
- Perfis de gait, interpolação de perfil, `allow_flight`.
- **Aceitação:** transição walk↔run suave; arco e velocidade escalam visivelmente; fase de voo (se habilitada) sem instabilidade; comprimento de passo satura no limite do perfil.

### Fase 6 — Preparação para o ragdoll ativo
- `SupportSolver` completo, `IBalanceProvider` (interface + implementação nula), pontos de injeção do §13, capture point desenhado no overlay.
- **Aceitação:** polígono de suporte correto em apoio simples e duplo; capture point calculado e desenhado a partir de um provedor de teste; nenhuma mudança de comportamento com provedor nulo.

---

## 20. Testes

### Unitários (sem PhysX, com `FakeTerrainProbe`)
- Anti-crossover: alvos deliberadamente cruzados são sempre reprojetados com largura ≥ mínimo.
- Alcance: alvos fora do anel são clampados dentro; nenhum alvo aprovado excede `max_reach`.
- Disparo: erro abaixo do limiar não dispara; acima, dispara; histerese não oscila.
- Swing: `s=0` ⇒ posição de lift-off; `s=1` ⇒ alvo; velocidade nas extremidades ≈ 0; folga ≥ mínima em todo o percurso.
- Abort: para todo `s ∈ [0,1]` amostrado, abortar resulta em pose plantada válida.
- Determinismo: mesma sequência de input ⇒ mesma sequência de passos, bit a bit.

### Cenários de terreno sintético
Plano · rampa 10°/25°/40° · escada 15 cm/20 cm/25 cm · degrau isolado · borda de plataforma · vão de 40 cm · terreno ruidoso · caixa dinâmica.

### Calibração no laboratório (o que olhar)
1. **Jitter em repouso** — pés não devem micro-passar parados. Se acontecer: histerese insuficiente.
2. **Passos picotados em velocidade alta** — `trigger_pos_threshold` não está escalando com a velocidade.
3. **Pés "atrás" do corpo** — falta antecipação na home (§9.1).
4. **Curvas com sensação errada** — ajustar `w_turn` do bônus de step turn (§9.3).
5. **Pé bate na quina da escada** — `clearance_per_dh` ou fração do primeiro waypoint.
6. **Alvo tremendo no fim do balanço** — `target_chase_gain` alto ou falta o fator `(1-s)`.
7. **Taxa de rejeição de foothold alta** — critérios do §11.3 apertados demais para a densidade de geometria da cena.

---

## 21. Erros a não cometer

| Anti-padrão | Por quê |
|---|---|
| Timer de passo no núcleo | Quebra R10 e inviabiliza equilíbrio (§4.3) |
| Ramificação por direção ("modo lateral", "modo ré") | Passo é `posição + yaw`; direção não é modo (§3.1) |
| Trajetória rígida calculada no lift-off | Impede replanejamento e a parada precisa (§2.5, §3.6) |
| Incluir PhysX no núcleo | Mata testabilidade e reuso (§6) |
| Constantes mágicas no código | Impossibilita calibração (§16) |
| Rejeitar alvos inválidos em vez de reprojetar | Produz travamentos visíveis do movimento (§10.4) |
| Teletransportar o pé quando não há solução | Rompe a ilusão irreversivelmente; melhor não dar o passo |
| Interpolar o pé de apoio | Foot skating — o pé plantado é **imóvel** por definição (§4.2) |
| Desenhar direto do núcleo | Acopla ao renderizador (§6) |
| Passo mínimo de N cm | Explicitamente proibido — R10 **[INVARIANTE]** |

---

## 22. Pontos deliberadamente em aberto

Estes são os pontos em que o conhecimento da engine existente deve prevalecer sobre este documento:

1. Convenções matemáticas, de nomes, de módulos, de build e de alocação.
2. Sistema de coordenadas e unidades (o documento assume metros e Y-up).
3. Como o `VirtualRoot` se materializa: puramente cinemático, `PxRigidDynamic` cinemático, ou capsule controller já existente.
4. Representação visual dos pés (caixa, cápsula, mesh de sapato como na referência).
5. Mecanismo de config e hot-reload.
6. Integração com o debug renderer e com a toolbar/popup do laboratório.
7. Frequência de tick e estratégia de interpolação para render.
8. Estratégia de filtragem de query PhysX (layers/masks) e orçamento de queries por frame.
9. Formato exato dos pesos e da função de custo de seleção de pé (§9.3) e de score de foothold (§11.3) — calibrar empiricamente.
10. Se `allow_flight` entra já na Fase 5 ou fica para depois.
11. Se a região de alcance analítica (§10.1) é suficiente ou se vale amostrar um mapa do esqueleto real quando ele existir.
12. Como o `FootworkOutput` será consumido pelo IK na fase seguinte (mas o contrato de saída já deve ser desenhado pensando nisso).

---

## 23. Referências

**Biomecânica da marcha**
- *The Gait Cycle* — Physiopedia. https://www.physio-pedia.com/The_Gait_Cycle
- *The Gait Cycle: Phases, Parameters to Evaluate & Technology* — Tekscan. https://www.tekscan.com/blog/medical/gait-cycle-phases-parameters-evaluate-technology
- *Understanding Phases of the Gait Cycle* — ProtoKinetics. https://protokinetics.com/understanding-phases-of-the-gait-cycle/
- *Gait Cycle: Phases & Biomechanics* — Orthofixar. https://orthofixar.com/basic-science/gait-cycle/
- Umberger, B. R. *Stance and swing phase costs in human walking* — J. R. Soc. Interface. https://pmc.ncbi.nlm.nih.gov/articles/PMC2894890/

**Curvas e estratégias de giro**
- Hase, K. & Stein, R. B. *Turning Strategies During Human Walking* — J. Neurophysiology (1999). https://journals.physiology.org/doi/full/10.1152/jn.1999.81.6.2914
- Taylor, M. J. D. et al. *A three-dimensional biomechanical comparison between turning strategies during the stance phase of walking* — Human Movement Science (2005). https://pubmed.ncbi.nlm.nih.gov/16129503/
- *Near-gaze fixation promotes use of spin turns during walking* — Frontiers in Aging Neuroscience (2026). https://www.frontiersin.org/journals/aging-neuroscience/articles/10.3389/fnagi.2026.1818850/full
- *Bases for the selection of alternate foot placement during straight- and turning-gait* — bioRxiv (2024). https://www.biorxiv.org/content/10.1101/2024.08.20.608530v1.full

**Planejamento de passos (robótica)**
- Kuffner, J. et al. *Online Footstep Planning for Humanoid Robots* — CMU Robotics Institute. https://publications.ri.cmu.edu/online-footstep-planning-for-humanoid-robots
- *Learning Bipedal Walking On Planned Footsteps For Humanoid Robots* — arXiv:2207.12644. https://arxiv.org/pdf/2207.12644
- *FootstepNet: an Efficient Actor-Critic Method for Fast On-line Bipedal Footstep Planning* — arXiv:2403.12589. https://arxiv.org/pdf/2403.12589
- *Learning Differentiable Reachability Maps for Optimization-based Humanoid Motion Generation* — arXiv:2508.11275. https://arxiv.org/pdf/2508.11275
- Argo-Robot, *footsteps_planning* (tutorial: 2D footstep planning, trajetórias 3D, ZMP/CoM). https://github.com/Argo-Robot/footsteps_planning

**Equilíbrio, capture point, controle**
- Pratt, J. et al. *Capture Point: A Step toward Humanoid Push Recovery*. https://www.researchgate.net/publication/224060461_Capture_Point_A_Step_toward_Humanoid_Push_Recovery
- *Determination of foot placement for humanoid push recovery* — descrição de capture point e capture region. https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7949430
- Joe, H.-M. & Oh, J.-H. *Balance recovery through model predictive control based on capture point dynamics* — Robotics and Autonomous Systems (2018). https://www.sciencedirect.com/science/article/abs/pii/S0921889017305493
- Yin, K. *Biped Controller for Character Animation* (SIMBICON / GENBICON). https://www.cs.sfu.ca/~kkyin/papers/BipedController.pdf
- *Watch Your Step: Real-Time Adaptive Character Stepping* — arXiv:2210.14730. https://arxiv.org/pdf/2210.14730
- *A Model Predictive Capture Point Control Framework for Robust Humanoid Balancing* — arXiv:2307.13243. https://arxiv.org/pdf/2307.13243

**Trajetória do pé em balanço**
- Park, I.-W. et al. *Online Free Walking Trajectory Generation for Biped Humanoid Robot KHR-3 (HUBO)* — CMU. https://www.ri.cmu.edu/pub_files/pub4/park_ill_woo_2006_1/park_ill_woo_2006_1.pdf
- *A Swing-foot Trajectory Generation Method For Biped Walking* — ICARM 2021. https://ieeexplore.ieee.org/document/9536057/
- *Anticipatory and Adaptive Footstep Streaming for Teleoperated Bipedal Robots* (waypoints para diferença de altura) — arXiv:2508.11802. https://arxiv.org/pdf/2508.11802

**Terreno e seleção de footholds**
- *On Terrain-Aware Locomotion for Legged Robots* (critérios de avaliação de foothold) — arXiv:2212.00683. https://arxiv.org/pdf/2212.00683
- *Load-bearing Assessment for Safe Locomotion of Quadruped Robots on Collapsing Terrain* (Visual Foothold Adaptation) — arXiv:2510.21369. https://arxiv.org/pdf/2510.21369
- *Real-time Multi-Plane Segmentation Based on GPU Accelerated High-Resolution 3D Voxel Mapping* (limitações de heightmaps 2.5D) — arXiv:2510.01592. https://arxiv.org/pdf/2510.01592
- *PolygMap: A Perceptive Locomotion Framework for Humanoid Robot Stair Climbing* — arXiv:2510.12346. https://arxiv.org/pdf/2510.12346
- *Motion control of quadruped robots in complex terrain* (encolhimento de bordas / margem de segurança). https://link.springer.com/article/10.1007/s44443-025-00292-z

**Locomoção em jogos**
- *Motion Matching and The Road to Next-Gen Animation* — GDC Vault. https://www.gdcvault.com/play/1023280/Motion-Matching-and-The-Road
- *Motion Matching in Unreal Engine* — Epic Developer Community. https://dev.epicgames.com/documentation/unreal-engine/motion-matching-in-unreal-engine
- *Balancing Of Active Ragdolls in Games* — Jan Schneider. https://medium.com/@jacasch/balancing-of-active-ragdolls-in-games-367f146b25fb
- *Real-Time Locomotion on Soft Grounds With Dynamic Footprints* — arXiv:2209.10215. https://arxiv.org/pdf/2209.10215

**PhysX 5**
- *Scene Queries* — NVIDIA PhysX SDK Documentation. https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/docs/SceneQueries.html
- *Geometry Queries* — NVIDIA PhysX SDK Documentation. https://nvidia-omniverse.github.io/PhysX/physx/5.2.1/docs/GeometryQueries.html

---

## 24. Resumo executivo em uma página

O sistema separa **movimento contínuo do corpo** (um root virtual dirigido por WASD no frame do olhar) de **movimento discreto dos pés** (passos disparados por erro em relação a uma posição de repouso ancorada ao root).

Um passo é sempre a mesma coisa: **posição 3D + yaw**. Andar de lado, de costas, em diagonal e girar são todos a mesma operação com vetores diferentes — não existem modos direcionais no código.

Um passo é disparado quando o erro do pé em relação à sua home ultrapassa um limiar dependente da velocidade — **nunca por relógio, e nunca com tamanho mínimo**. Isso é o que permite tanto caminhar quanto, no futuro, equilibrar-se com micro-ajustes.

O alvo do passo passa por uma cadeia de **reprojeção**: alcance anular, largura lateral mínima (anti-crossover), yaw relativo máximo, e então avaliação de terreno (rugosidade, cobertura, margem de borda, inclinação) com busca em anéis por alternativa. O sistema **nunca** produz um alvo inválido; no pior caso ele não dá o passo.

A trajetória de balanço é parametrizada por uma **fase `s ∈ [0,1]`**, com perfis horizontal e vertical desacoplados, e persegue um **alvo móvel** cujo peso decai conforme `s` cresce. É isso que torna possível redirecionar o pé em pleno voo, e abortar limpamente quando a tecla é solta — plantando sempre em posição válida.

Correr não é um sistema; é um **perfil de parâmetros** com `duty_factor < 0.5`.

O núcleo não conhece PhysX, nem o renderizador, nem o input, nem o esqueleto. Ele fala com o mundo por uma interface `ITerrainProbe` e devolve poses-alvo, fases e um buffer de primitivas de debug. É essa fronteira que permite que este mesmo módulo, sem reescrita, vire o planejador de passos do ragdoll ativo — bastando preencher dois ganchos: um no cálculo do alvo, outro no disparo do passo.
