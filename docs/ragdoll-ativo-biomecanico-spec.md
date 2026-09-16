# Ragdoll Ativo Biomecânico — Base Teórica e Arquitetura

**Projeto:** MatterEngine (C++ / NVIDIA PhysX 5)
**Módulo:** controle ativo do ragdoll — ficar em pé, equilibrar-se, locomover-se
**Premissa inegociável:** nenhuma força mágica. Toda ação vem de torques nas juntas.
**Estado de partida:** ragdoll PhysX (18 links, 17 juntas, 41 DOFs, 75 kg, base flutuante) + FootWork cinemático validado, ambos com contratos neutros e testes.
**Documento:** v1.0

---

## 0. Como ler este documento

- **Parte I — Base teórica.** O que a robótica de humanoides, a biomecânica e a animação baseada em física já resolveram. Cada conceito vem com a razão de existir e a consequência prática para o seu caso.
- **Parte II — O que o PhysX 5 já entrega.** Levantamento do que a API oferece nativamente. Isso muda a arquitetura de forma substancial — parte do que normalmente se escreve à mão já existe no SDK.
- **Parte III — Achados críticos sobre o estado atual.** Coisas que devem mudar **antes** de começar o controlador, porque senão o trabalho é construído sobre areia.
- **Parte IV — Arquitetura proposta.**
- **Parte V a X — Roadmap, parâmetros, telemetria, testes, preparação para levantar-se, armadilhas.**

Marcações usadas:
- **[INVARIANTE]** — requisito de projeto que não deve ser negociado.
- **[ABERTO]** — decisão que deve seguir as convenções já existentes na MatterEngine.
- **[CRÍTICO]** — se ignorado, o sistema não funciona; não é questão de qualidade, é questão de possibilidade.

---

# PARTE I — BASE TEÓRICA

## 1. O problema fundamental: subatuação

Este é o conceito que organiza todo o resto. Se apenas uma seção deste documento for lida, que seja esta.

A equação de movimento do ragdoll é:

```
M(q) q̈  +  C(q, q̇) q̇  +  G(q)  =  Sᵀ τ  +  Σ Jᶜᵢᵀ λᵢ
```

- `q` — posição da raiz flutuante (6) + posições das juntas (41) = 47 coordenadas generalizadas
- `M(q)` — matriz de massa generalizada
- `C(q,q̇) q̇` — forças de Coriolis e centrífugas
- `G(q)` — gravidade
- `τ` — torques das 41 juntas
- `λᵢ` — forças de contato
- `S` — matriz de seleção da atuação

**O ponto decisivo:** as **6 primeiras linhas** dessa equação — as que correspondem à raiz flutuante — têm `S = 0`. Nenhum torque de junta aparece ali. Elas dizem, literalmente:

```
[ m c̈  ]     [ m g ]     [   Σ fᵢ                    ]
[      ]  =  [     ]  +  [                           ]
[  l̇   ]     [  0  ]     [ Σ (pᵢ − c) × fᵢ + Σ τᵢ    ]
```

Ou seja: **o movimento do centro de massa e do momento angular do corpo é determinado exclusivamente pela gravidade e pelas forças de contato.** Nenhum torque interno pode mover o CoM. Você pode contorcer o corpo à vontade no ar e o CoM continuará em queda livre.

Três consequências que valem por metade do projeto:

1. **Aplicar força na pelve é trapaça e sempre parecerá trapaça.** É exatamente o que Gang Beasts / Human: Fall Flat fazem — forças externas para equilibrar. Funciona, mas é a antítese do que você pediu.
2. **Equilibrar-se é, na prática, o problema de escolher onde e com que intensidade os pés empurram o chão.** Todo o resto do corpo existe para posicionar o CoM e para modular o momento angular de forma que a força de contato desejada seja fisicamente realizável.
3. **Existe um teto rígido de autoridade.** Se a força de contato necessária estiver fora do cone de atrito, ou se o ponto de aplicação necessário estiver fora do pé, o objetivo é **inadmissível** e o controlador precisa saber disso e degradar de forma controlada — não insistir e explodir.

## 2. Pontos de referência no solo

### 2.1 CoP — Centro de Pressão

O ponto no solo onde a resultante das forças de contato pode ser considerada aplicada, com momento tangencial nulo:

```
CoP = Σ (pᵢ · fᵢ,normal) / Σ fᵢ,normal
```

Só existe **dentro do polígono de suporte**, por definição — não é uma escolha, é uma restrição física. É a variável observável mais importante do controlador: ela é medida diretamente dos contatos do solver.

### 2.2 ZMP — Zero Moment Point

O ponto onde o momento resultante de gravidade e inércia é nulo. Enquanto o pé não gira, **ZMP e CoP coincidem**. A condição clássica de estabilidade dinâmica é: **ZMP dentro do polígono de suporte**. Quando o ZMP "quer" sair, o pé começa a girar sobre a aresta e o controle de tornozelo satura.

### 2.3 CMP — Centroidal Moment Pivot

O ponto onde a força de reação do solo teria que atuar para **não produzir nenhum momento angular em torno do CoM**. A relação essencial:

- Se o momento angular centroidal está sendo conservado (`l̇ = 0`), então **CMP = CoP**.
- A distância entre CMP e CoP é proporcional à taxa de variação do momento angular centroidal.

**É isto que formaliza a "estratégia de quadril".** O CMP pode estar **fora** do polígono de suporte enquanto o CoP não pode. Girar tronco e braços move o CMP para fora sem violar nenhuma restrição de contato — e é exatamente assim que um humano se recupera de um empurrão sem dar um passo.

Popovic, Goswami e Herr estudaram ZMP, FRI e CMP na marcha humana real e observaram que o momento angular de spin permanece pequeno durante o ciclo de marcha normal — ou seja, humanos usam essa margem de forma econômica, como reserva de emergência.

### 2.4 Capture Point / DCM

Sob o modelo do pêndulo invertido linear (LIPM):

```
ω  = √(g / z_com)
ξ  = c_horizontal + ċ_horizontal / ω        (capture point / DCM)
```

Interpretação operacional, que é a que você vai codificar:

| Situação | Estratégia |
|---|---|
| `ξ` bem dentro do polígono de suporte | Nada a fazer, ou ajuste de tornozelo |
| `ξ` próximo da borda | Tornozelo (mover CoP) |
| `ξ` fora, mas alcançável por counter-rotation | Quadril / tronco / braços (mover CMP) |
| `ξ` fora e não recuperável sem novo apoio | **Passo** — pisar de modo que a nova base de suporte intersecte a região de captura |
| `ξ` fora e nenhum passo alcança | Queda controlada |

A extensão LIPM + volante (flywheel) de Pratt et al. permite calcular a região de captura levando em conta a inércia rotacional disponível — é a formalização de "quanto o counter-rotation compra".

**[INVARIANTE]** O capture point é o **sinal de comando único** que unifica as quatro estratégias. Ele deve existir desde a primeira linha de código do estimador de equilíbrio.

## 3. Dinâmica centroidal e momento angular

O momento centroidal `h_G = [linear; angular]` relaciona-se com as velocidades generalizadas pela **Matriz de Momento Centroidal (CMM)**:

```
h_G = A_G(q) q̇
ḣ_G = A_G q̈ + Ȧ_G q̇
```

O trabalho de Orin, Goswami e Lee estabeleceu a estrutura e as propriedades da CMM; Lee e Goswami construíram sobre ela um controlador de equilíbrio que regula **linear e angular simultaneamente**, determinando diretamente a GRF e o CoP em cada pé de apoio — o que permite lidar com solo não plano, não estacionário e com coeficientes de atrito diferentes por pé.

A contribuição decisiva desse trabalho para nós é o conceito de **admissibilidade**: antes de tentar realizar as taxas desejadas de momento linear e angular, o controlador **verifica se elas são fisicamente possíveis** dadas as restrições de contato. Quando não são, ele resolve o conflito por prioridade — tipicamente preservando o objetivo de momento linear e sacrificando o angular (ou o contrário, conforme a situação). Isso é o oposto de "mandar torque e torcer".

Experimentalmente, esse tipo de controlador recupera de empurrões laterais gerando momento angular pela rotação do tronco **e dos braços**, o que é reportado como comparável ao comportamento humano.

**Consequência de design:** braços não são decoração. São o atuador de momento angular mais barato do corpo, porque têm inércia razoável e nenhuma responsabilidade de contato.

## 4. As estratégias humanas de equilíbrio

Horak e Nashner estabeleceram a taxonomia clássica a partir de experimentos com plataforma móvel:

| Estratégia | Quando | Mecânica |
|---|---|---|
| **Tornozelo** | Perturbação pequena, superfície ampla e firme | Corpo se move como pêndulo invertido de segmento único; torque no tornozelo desloca o CoP |
| **Quadril** | Perturbação maior, ou base curta/complacente que limita o torque de tornozelo | Pêndulo de dois segmentos com movimento em contrafase entre tornozelo e quadril; gera cisalhamento horizontal |
| **Passo** | Perturbação além da capacidade das anteriores | Nova base de suporte |
| **Suspensória** | Estabilização vertical | Flexão de joelhos e quadris, abaixando o CoM |

Detalhes que importam para a implementação:

- A ativação muscular da estratégia de tornozelo é **distal → proximal**; a de quadril é **proximal → distal**. Isso sugere a ordem em que os ganhos devem ser escalonados.
- A escolha entre elas correlaciona-se com **minimização de esforço mecânico** — o que é uma justificativa direta para implementar a seleção como um custo, e não como um `if`.
- Superfície de apoio menor empurra o humano para a estratégia de quadril. No seu caso isso significa: **a geometria do pé determina qual estratégia é possível** (ver §11, achado crítico).
- Frameworks modernos (MPC sobre capture point) usam **peso variável**: suprimem a estratégia de quadril para erros pequenos, porque usá-la em excesso produz movimento não natural, e a liberam para erros grandes. Tornozelo e passo primeiro, quadril como necessário — igual ao comportamento humano.
- A estratégia de quadril tem **orçamento finito**: depois de usar a rotação, o corpo precisa voltar à postura, o que leva tempo e temporariamente indisponibiliza a estratégia. Isso deve estar no modelo.

## 5. Viabilidade de contato

Não basta calcular a força de contato desejada. Ela precisa ser realizável.

Para um pé retangular em solo plano com atrito uniforme, Caron et al. derivaram uma forma fechada do **Contact Wrench Cone (CWC)** que se decompõe em três condições simples:

1. **Atrito de Coulomb** sobre a força resultante (`‖f_tangencial‖ ≤ μ f_normal`);
2. **CoP dentro da área de suporte**;
3. **Limites superior e inferior no torque de yaw** — condição menos conhecida e diretamente relevante para o problema de o pé girar em torno do próprio eixo vertical, que é um defeito recorrente em bípedes.

**Consequência de design [INVARIANTE]:** essas três desigualdades são o **teste de admissibilidade** do controlador, e são baratas — não exigem programação cônica nem enumerar forças por vértice. Elas se aplicam diretamente porque os pés do seu ragdoll são (ou devem ser — ver §11) retângulos.

O critério de ZMP sozinho impede o tombamento do pé, mas **não** impede escorregar nem girar. Se o pé de apoio deslizar durante o equilíbrio, o diagnóstico quase certo é: o wrench desejado violou a condição 1 e o controlador não verificou.

## 6. Como se converte intenção em torque

Este é o catálogo de mecanismos disponíveis, do mais simples ao mais completo. A arquitetura proposta usa **vários deles somados**, não um só.

### 6.1 Controle PD nas juntas

O básico: `τ = k_p (q_d − q) + k_d (q̇_d − q̇)`. Simples, mas com um problema conhecido: ganhos altos com passo de tempo discreto geram instabilidade.

**Stable PD (SPD)** de Tan, Liu e Turk resolve isso computando forças e torques levando em conta posições e velocidades **do próximo passo de tempo**, e é estável mesmo com integração de Euler simples e ganhos arbitrariamente altos. A condição empírica de estabilidade reportada é `k_d ≥ k_p Δt`.

**Boa notícia:** os drives de articulation do PhysX 5 já são **implícitos** — a restrição que o solver resolve é em relação ao **fim** do passo de tempo, não ao início. A documentação afirma explicitamente que a vantagem chave dessa formulação é suportar ganhos muito altos sem instabilidade da junta. Ou seja, **você já tem SPD de graça** nos drives. A ressalva também está documentada: isso vale para drives isolados; combinados com contatos, limites de junta ou mimic joints muito rígidos, pode não haver solução estável, e o remédio é reduzir rigidez, reduzir força máxima ou reduzir o passo.

### 6.2 Compensação de gravidade (feedforward)

`τ_g = G(q)` — os torques exatos para segurar o corpo parado contra a gravidade na pose atual.

Este é, de longe, o **maior ganho por linha de código** de todo o projeto. Sem ele, o PD precisa de erro de posição para produzir torque, então o ragdoll sempre "cede" um pouco e você compensa com ganho absurdo, o que traz oscilação e explosão. Com ele, o PD passa a corrigir apenas **desvios**, e pode ser suave — o que é justamente o regime de baixo ganho que caracteriza movimento natural.

GENBICON (Coros, Beaudoin, van de Panne) usa exatamente isso — compensação de gravidade computada via Jacobiana transposta para todos os membros — como um dos quatro pilares do controlador.

### 6.3 Compensação de Coriolis/centrífuga

`τ_c = C(q,q̇) q̇`. Importa quando os membros se movem rápido (perna em swing na corrida, chute, braço em contrabalanço). Sem ela, movimentos rápidos "arrastam" o resto do corpo.

A documentação do PhysX inclusive alerta que a integração explícita de forças de Coriolis e centrífugas é a razão pela qual articulations precisam de clamps/damping em velocidades angulares altas.

### 6.4 Jacobiana transposta / Virtual Model Control

Pratt et al. formalizaram o **Virtual Model Control**: imaginar molas, amortecedores e forças virtuais atuando em pontos do corpo, e convertê-las em torques reais pela Jacobiana transposta:

```
τ = Jᵀ F_virtual
```

É intuitivo, barato e não requer solver. GENBICON usa VMC/JT para dois papéis: compensação de gravidade e **aplicação de uma força virtual no centro de massa para regular sua velocidade**. Geijtenbeek et al. mostraram que PD + uma forma específica de JT + otimização offline de parâmetros produz agachamentos, reverências, chutes e danças com robustez a perturbações, rodando em tempo real em um único núcleo.

**Este é o mecanismo central recomendado para a Fase 2 do seu projeto.** Uma força virtual aplicada ao CoM, roteada pela cadeia da perna de apoio, **é** a estratégia de tornozelo e parte da de quadril, sem escrever nenhum QP.

Limitação a conhecer: a faixa de forças virtuais realizáveis é limitada, justamente pela subatuação e pelo contato. Daí a necessidade do teste de admissibilidade de §5.

### 6.5 Whole-Body Control por QP

A formulação canônica, usada em Atlas, Valkyrie, HRP-2, TORO e praticamente todo humanoide moderno controlado por torque:

```
min   Σ wᵢ ‖ Aᵢ(q,q̇) q̈ − bᵢ(q,q̇) ‖²  + ε_τ‖τ‖² + ε_λ‖λ‖²
q̈,τ,λ

s.a.  M q̈ + C q̇ + G = Sᵀτ + Jᶜᵀλ        (dinâmica)
      λ ∈ cone de atrito, CoP ∈ suporte    (contato)
      τ_min ≤ τ ≤ τ_max, limites de junta  (atuação)
```

Tarefas típicas empilhadas por peso ou por hierarquia: taxa de momento centroidal, orientação do tronco, posição do pé em swing, postura de repouso. Herzog et al. demonstraram controle de momento com dinâmica inversa hierárquica em humanoide real controlado por torque.

**Recomendação:** isto é o destino, não o ponto de partida. Um QP com ~50 variáveis a 120 Hz × 22 personagens é viável, mas é a última coisa a construir, depois que tudo o mais estiver correto e instrumentado. A arquitetura deve deixar o lugar dele pronto (§17).

## 7. Controladores consagrados em animação

### 7.1 SIMBICON

Yin, Loken e van de Panne. Uma máquina de estados de poses + PD + **uma realimentação linear simples para colocação do pé**:

```
offset_do_pé = c_d · d + c_v · v
```

onde `d` é o deslocamento horizontal do CoM em relação ao pé de apoio e `v` sua velocidade. Com isso obtiveram, em tempo real, caminhada em todas as direções (frente, trás, lateral, giro), corrida, skipping e hopping, robustez a empurrões em todas as direções, degraus e rampas inesperados, e variações de parâmetros cinemáticos e dinâmicos. O controlador de caminhada resistiu a empurrões de até 600 N para frente e 500 N para trás com duração de 0,1 s, em todos os pontos amostrados do ciclo.

**Por que isso importa para você:** é a prova de que **não é preciso um QP para andar de forma robusta**. Uma lei de realimentação de duas constantes, alimentando o alvo do passo, já entrega a maior parte da robustez. E é exatamente o formato do gancho que você já deixou previsto no FootWork.

Eles também aplicaram *feedback-error learning* para aprender modelos preditivos de torque, o que permite o controle de baixo ganho típico de movimento natural — uma alternativa a compensação analítica.

### 7.2 GENBICON

Coros, Beaudoin e van de Panne. Quatro componentes:
1. gerador de movimento (trajetórias desejadas);
2. **modelo de pêndulo invertido para colocação preditiva do pé** (substitui a heurística linear do SIMBICON);
3. **compensação de gravidade** por Jacobiana transposta para todos os links;
4. **ajuste fino de velocidade** por força virtual no CoM.

Os autores demonstram que cada um dos quatro componentes é necessário para caminhada flexível e ágil. É o mapa de estrada mais próximo do que você deve construir.

### 7.3 Momentum Control for Balance

Macchietto, Zordan e Shelton — controle explícito de CoM e momento angular para equilíbrio em personagens animados. Junto com Lee e Goswami, forma a base do que na Parte IV será a camada de "resolução centroidal".

## 8. A via neuromuscular

Uma alternativa legítima e mais literalmente "biomecânica": em vez de PD nas juntas, modelar unidades músculo-tendão (tipo Hill) e controlá-las por **reflexos** de comprimento e força.

Geyer e Herr mostraram que um modelo controlado apenas por reflexos musculares que codificam princípios da mecânica de pernas **estabiliza em marcha a partir da interação dinâmica com o solo**, reproduz dinâmica e cinemática humanas, tolera perturbações do terreno e se adapta a rampas sem intervenção de parâmetros. Geijtenbeek et al. estenderam para locomoção muscular flexível em bípedes genéricos; a ferramenta SCONE tornou esse tipo de modelagem acessível.

Trabalhos recentes mostram que uma rede simples de reflexos proprioceptivos produz uma variedade de marchas — caminhar e saltitar, para frente e para trás, correr em variações e velocidades — **sem entradas rítmicas nem máquinas de estado de alto nível modulando os ganhos**.

**Avaliação para o seu caso:** é a via de maior fidelidade e a de pior custo-benefício agora. Exige modelar dezenas de músculos por perna, calibrar parâmetros por otimização offline (CMA-ES ou similar) e ainda assim o resultado depende criticamente de morfologia. Com 22 jogadores em tempo real, é inviável na primeira iteração.

**Mas há um empréstimo barato e valioso:** a ideia de **reflexo** — realimentação local, rápida e específica de fase — é implementável diretamente sobre juntas, sem músculos. Exemplos que valem a pena copiar:
- realimentação positiva de força no extensor do tornozelo durante o apoio (produz push-off emergente);
- realimentação que impede a hiperextensão do joelho;
- realimentação de orientação do tronco durante o apoio;
- controle da perna em swing por ângulo-alvo da perna + clearance + torque de reação no quadril.

Isso é o "sabor biomecânico" sem o custo de um modelo musculoesquelético.

O PhysX 5 oferece **spatial tendons**, que impõem restrições de distância entre pontos de ancoragem em links e são explicitamente descritos como forma de modelar atuadores hidráulicos ou músculos artificiais em robôs biomiméticos. Isso é uma porta aberta para o futuro, não para agora.

## 9. Aprendizado por reforço — onde ele entra

- **DeepMimic** (Peng et al.) — RL guiado por exemplo; aprende habilidades de personagem físico imitando clipes.
- **AMP** (Peng et al.) — substitui a função de imitação manual por um discriminador adversarial; o estilo vem de um conjunto não estruturado de clipes e a tarefa vem de uma recompensa simples. A composição de habilidades emerge sem planejador de alto nível.
- **ASE**, **MaskedMimic** e derivados — embeddings de habilidades reutilizáveis.

Um detalhe de arquitetura que é o que realmente importa aqui: **essas políticas tipicamente emitem ângulos-alvo de controladores PD**, não torques diretos. Um trabalho representativo aprende inclusive a **modular dinamicamente os ganhos** dos PDs, com os alvos vindos do clipe.

**Consequência de design [INVARIANTE]:** se a sua camada de drives aceitar, por DOF, `(alvo, velocidade-alvo, escala de rigidez, escala de damping, escala de torque)` — que é exatamente o `RagdollAxisTarget3D` que você já esboçou — então uma política aprendida é um **substituto plug-in** da camada analítica no futuro, sem reescrever nada abaixo. Construa a camada analítica primeiro; ela será o baseline, a referência de comparação e o fallback.

Não comece por RL: você não tem infraestrutura de treino, precisa de determinismo para testes, precisa de 22 personagens simultâneos e — principalmente — precisa **entender** o sistema, o que uma política não te dá.

## 10. O que a indústria fez

**Euphoria / Dynamic Motion Synthesis (NaturalMotion)** — usado em GTA IV, GTA V, Red Dead Redemption 1 e 2, e nos títulos Star Wars: The Force Unleashed. A tecnologia é descrita como animação em tempo real baseada em simulação completa do personagem, incluindo corpo, músculos e sistema nervoso motor. Em vez de animações predefinidas, ações e reações são sintetizadas em tempo real e **diferem a cada execução, mesmo repetindo a mesma cena**. Cobre praticamente todos os comportamentos animados: uso de armas, combate corpo a corpo, saltos, escaladas, recuperações e interação com objetos. A NaturalMotion encerrou o licenciamento comercial em 2017.

**Backbreaker** — jogo de futebol americano feito pela própria NaturalMotion, cujos tackles são gerados em tempo real em vez de reproduzidos. Relatos descrevem NPCs com senso de autopreservação e comportamentos contextuais: o portador da bola estende o braço para afastar quem se aproxima, jogadores agarram e carregam a bola apropriadamente, e um jogador derrubado perto da linha estende o braço para dentro da zona.

**Isto é literalmente o seu objetivo final de 22 jogadores.** Vale saber que já foi feito, com hardware de 2010, e que a arquitetura pública descrita é: comportamentos ("behaviors") componíveis sobre um corpo fisicamente simulado — não uma política monolítica.

## 11. Levantar-se

Deixado para depois, mas vale registrar o mapa, porque ele determina decisões estruturais de agora.

Três famílias:
1. **Trajetórias predefinidas / grafo de configurações** — nós armazenados entre deitado e em pé, com transições. Robusto e barato, mas cobre um número limitado de estados de queda e generaliza mal.
2. **Síntese por RL** — Frezzato et al. sintetizam movimentos de levantar para personagens físicos a partir de posição supina ou prona, sem imitar clipes individuais, guiados por curvas de estilo, com um espaço latente de poses naturais extraído de mocap. Na robótica, HoST, FRASA e HiFAR aprendem levantar-se com currículo multi-estágio e transferem para hardware real.
3. **Imitação com recuperação embutida** — Chentanez et al. controlam o personagem para levantar e voltar a rastrear o movimento após perturbações grandes que causam queda.

**Decisão estrutural derivada disto [INVARIANTE]:** o modelo de contato do controlador **não pode ser específico de pés**. Levantar-se usa mãos, antebraços, joelhos, quadril e tronco como pontos de apoio. Se o polígono de suporte e o estimador de equilíbrio forem escritos assumindo "dois pés", tudo terá que ser reescrito. Escreva-os sobre um **conjunto genérico de contatos** desde o primeiro dia. O custo adicional agora é quase nulo; o custo de refatorar depois é alto.

## 12. Síntese — a escada de complexidade

Ordenada por razão valor/esforço. Cada degrau é testável isoladamente e nenhum invalida o anterior.

| # | Mecanismo | O que compra | Custo |
|---|---|---|---|
| 1 | Telemetria: CoM, ĊoM, contatos, CoP, polígono, capture point | Visibilidade. Sem isso você depura no escuro | Baixo |
| 2 | Compensação de gravidade feedforward | O corpo para de ceder; PD pode ser suave | Muito baixo (1 chamada) |
| 3 | Drives por DOF com ganhos por estado + postura de referência | Postura mantida | Baixo |
| 4 | Compensação de Coriolis | Membros rápidos param de arrastar o corpo | Muito baixo |
| 5 | Força virtual no CoM via Jacobiana transposta (VMC) | **Estratégia de tornozelo** | Médio |
| 6 | Teste de admissibilidade (CWC) + saturação | Para de escorregar e explodir; degradação controlada | Baixo |
| 7 | Amortecimento de momento angular centroidal (tronco + braços) | **Estratégia de quadril** | Médio |
| 8 | Capture point → correção do alvo do FootWork | **Estratégia de passo** | Baixo (o gancho já existe) |
| 9 | Distribuição de GRF entre dois pés | Duplo apoio correto, terreno irregular | Médio |
| 10 | Reflexos específicos de fase (push-off, anti-hiperextensão, tronco) | Naturalidade | Médio |
| 11 | QP de corpo inteiro | Otimalidade, prioridades explícitas, multi-contato | Alto |
| 12 | Política aprendida substituindo camadas 5–10 | Robustez e naturalidade além do analítico | Muito alto |

**A recomendação é implementar 1 a 8 nesta ordem e parar para avaliar.** É muito provável que 1–8 já entreguem "ragdoll que spawna em pé, se equilibra e anda", que é o objetivo declarado.

---

# PARTE II — O QUE O PHYSX 5 JÁ ENTREGA

Esta parte muda a arquitetura de forma substancial. Boa parte do que normalmente se escreve à mão (ou se importa do Pinocchio/RBDL) já está no SDK, para articulations reduced-coordinate.

## 13. Dinâmica inversa nativa

O PhysX documenta explicitamente que articulations reduced-coordinate oferecem funcionalidade de dinâmica inversa baseada na equação de movimento `τ = M(q) q̈ + C(q,q̇) q̇ + G(q)`. Disponível via `PxArticulationCache`:

| Chamada | O que dá | Uso no nosso controlador |
|---|---|---|
| `computeGravityCompensation(cache)` | `G(q)` — forças de junta para anular a gravidade. Para base flutuante, retorna também força e torque da raiz | **Degrau 2 da escada.** Feedforward obrigatório |
| `computeCoriolisCompensation(cache)` | `C(q,q̇)q̇` | Degrau 4 |
| `computeMassMatrix(cache)` | `M(q)`. Base flutuante ⇒ `(nDofs+6)²` | Base do QP e do SPD explícito |
| `computeCentroidalMomentumMatrix(cache)` | **`A_G` e o viés `Ȧ_G q̇`** | **Controle de momento centroidal — §3.** A doc cita Wensing et al. e Orin et al. e diz que se aplica a WBC de humanoides |
| `computeDenseJacobian(cache, nRows, nCols)` | Jacobiana espacial no mundo, incluindo DOFs de base flutuante | **VMC / Jacobiana transposta — §6.4** |
| `computeGeneralizedExternalForce(cache)` | Torques de junta para contrapor forças externas nos links | Compensar carga conhecida |
| `computeJointForce(cache)` | `M(q)q̈` para acelerações desejadas | Feedforward de trajetória |
| `computeJointAcceleration(cache)` | `q̈` dadas forças, incluindo gravidade | Predição / verificação |
| `computeArticulationCOM(rootFrame)` | CoM da articulation | Telemetria (mas ver nota abaixo) |

**Isto é enorme.** A existência nativa da **matriz de momento centroidal** significa que o controlador de equilíbrio momentum-based de Lee & Goswami é implementável sem escrever uma biblioteca de dinâmica de corpos rígidos.

### 13.1 Protocolo obrigatório

A documentação é explícita quanto à sequência:

1. aplicar a pose (posições de junta + transform da base) via cache e `applyCache`;
2. chamar `commonInit()`;
3. preencher os valores de entrada no cache conforme o método;
4. chamar o método de dinâmica inversa.

**[CRÍTICO]** Essas chamadas **não podem ser feitas durante a simulação**. Elas se encaixam exatamente no safe point que a MatterEngine já usa antes de `simulate()`. Isso valida a "ordem fixa de atualização" que já está documentada.

Notas de armadilha:
- `computeJointForce()` **ignora** posições de junta e transform de raiz setados no cache; usa a pose corrente. Para outra pose, é preciso aplicar antes.
- `computeCentroidalMomentumMatrix` **assume** que a matriz de massa e a compensação de Coriolis já estão atualizadas no cache. A ordem importa.
- `computeGeneralizedGravityForce` e `computeGeneralizedMassMatrix` foram **depreciados** em favor de `computeGravityCompensation` e `computeMassMatrix`; a razão declarada é que os antigos não permitiam obter as forças da raiz para base flutuante — que é justamente o nosso caso. **Use os novos.**
- **[ABERTO]** Verificar a versão do PhysX que a MatterEngine usa. As APIs de compensação com suporte a raiz flutuante e a CMM são de versões recentes. Se a engine estiver em 5.1/5.3, a CMM pode não existir e o degrau 7 da escada precisa ser implementado à mão a partir de `M(q)` e da Jacobiana — possível, mas mais trabalho.

## 14. Drives, torques e medição

### 14.1 Aplicar torque puro

`PxArticulationCacheFlag::eFORCE` permite aplicar **torque/força de junta diretamente pelo usuário**, e essa contribuição é contabilizada na força transmitida pela junta. É o canal para o feedforward (`τ_g`, `τ_c`, `Jᵀ F`).

**[INVARIANTE]** Este é o único canal legítimo de atuação, junto com os drives. Nenhuma `addForce` em link, nunca — exceto para eventos externos genuínos do mundo (explosão, Physgun, colisão com a bola).

### 14.2 Drives

`PxArticulationDrive` com `stiffness`, `damping`, `maxForce`, `driveType`. Unidades documentadas assumindo SI: saída em Nm, `stiffness` em Nm/rad, `damping` em Nm/(rad/s).

Dois pontos de alto valor:

- **`PxArticulationDriveType::eACCELERATION`** — a saída do drive é uma **aceleração de junta**, o que a doc descreve como útil para obter comportamento **independente da massa e da inércia dos links**. Para controle de postura em um corpo com massas muito diferentes (mão de 0,45 kg vs. coxa de 8,6 kg), isso elimina boa parte do trabalho de tunar ganho por junta.
- **`PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES`** — se setada, `maxForce` é força/torque; caso contrário, é **impulso** (força × dt). **[CRÍTICO]** O perfil `HumanAdultV1` define "torque máximo" por eixo. Se essa flag não estiver setada, o limite está sendo interpretado como impulso e os torques efetivos são `dt` vezes menores do que o pretendido — a 120 Hz, um fator de ~120. Verificar isto é possivelmente a coisa mais barata e mais impactante desta lista inteira.

### 14.3 Medir força transmitida

`PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE` reporta a força espacial transmitida do pai para o filho, **no frame da junta de entrada do link** — o que a doc destaca como conveniente porque extrair, por exemplo, o torque do drive vira uma projeção direta. Ela inclui atrito de junta, clamp de velocidade, torque aplicado pelo usuário, drives e limites; exclui tendões e armature.

Nota: `PxArticulationCacheFlag::eJOINT_SOLVER_FORCES` está **depreciado e reporta valores incorretos** — usar `eLINK_INCOMING_JOINT_FORCE`.

**Uso:** detecção de saturação por junta, telemetria de esforço, e uma aproximação de "carga" para o modelo de fadiga futuro.

### 14.4 Estabilidade

A doc reconhece que articulations reduced-coordinate são adequadas para humanoides/ragdolls em jogos, mas que **clamps ou damping de velocidade podem ser necessários** para estabilidade em velocidades angulares altas, por causa da integração explícita de Coriolis e centrífuga. As opções listadas: limites/damping de velocidade de corpo rígido, atrito de junta não nulo, ou drive com damping não nulo.

Há também uma discussão detalhada sobre **conflito entre drive rígido, limite de junta e contato**: a ordem de resolução é `drive → limite de posição → limite de velocidade → contato estático`, e a última restrição resolvida "vence". Se nada ceder, o resultado é instabilidade ou restrição visivelmente violada. **A solução recomendada pelo próprio SDK é reduzir rigidez/damping/força máxima ou reduzir o timestep.**

**Consequência de design [INVARIANTE]:** um controlador que depende de rigidez muito alta é frágil por construção neste solver. É mais uma razão para investir em feedforward (gravidade, Coriolis, VMC) em vez de ganho.

### 14.5 Armature

`setArmature()` adiciona inércia à junta. É o truque padrão para estabilizar juntas leves acionadas por drives rígidos (mão, pé, pescoço). Barato e eficaz. Recomendo expor no perfil por eixo.

## 15. Custo e escala

Ordens de grandeza para o perfil atual (41 DOFs, base flutuante ⇒ 47):

| Cálculo | Tamanho | Frequência recomendada |
|---|---|---|
| `computeGravityCompensation` | 47 valores, O(n) | **Todo tick, todo personagem ativo** |
| `computeCoriolisCompensation` | 47 valores, O(n) | Todo tick, personagens em locomoção |
| `computeDenseJacobian` | 6·18 × 47 ≈ 5.076 floats | Todo tick para personagens em equilíbrio ativo (ou apenas as colunas das pernas — ver nota) |
| `computeMassMatrix` | 47² = 2.209 floats, O(n²)–O(n³) | Só quem precisa de CMM ou QP; considerar taxa reduzida |
| `computeCentroidalMomentumMatrix` | 6 × 47 = 282 floats | Só personagens em equilíbrio ativo |

**Nota sobre a Jacobiana:** a doc adverte que a Jacobiana é esparsa/triangular e que a representação densa **não é um uso ótimo de memória** — o PhysX não usa essa representação internamente. Para VMC você precisa apenas das colunas correspondentes aos DOFs da cadeia da perna de apoio. Extraia a submatriz e descarte o resto, ou considere calcular a Jacobiana da cadeia analiticamente (é uma cadeia de 6 DOFs: 3 quadril + 1 joelho + 2 tornozelo) — muito mais barato.

**[INVARIANTE]** LOD de controle desde o início. Um jogador longe da câmera, sem bola por perto e sem contato iminente não precisa de CMM nem de matriz de massa. Ele precisa de gravidade + postura + polígono de suporte. Sem LOD, 22 jogadores não fecham o orçamento.

---

# PARTE III — ACHADOS CRÍTICOS SOBRE O ESTADO ATUAL

Ler antes de escrever qualquer linha do controlador.

## 16. Achados

### 16.1 [CRÍTICO] O pé é uma cápsula

O documento de estado registra: *"Todos os 18 links físicos usam cápsulas do PhysX com raio uniforme de 5,5 cm. Isso inclui fisicamente: cabeça; mãos; pés; tronco."* E, no visual, os pés são "caixas achatadas e alongadas".

**Isto inviabiliza o equilíbrio.** A razão é direta:

- Uma cápsula deitada toca o solo plano em uma **linha**, não em uma superfície. O fecho convexo dos pontos de contato é um segmento de reta.
- Logo, o polígono de suporte de apoio simples tem **largura lateral zero**. O CoP não pode se deslocar lateralmente. **A estratégia de tornozelo no plano frontal deixa de existir.**
- No plano sagital, a cápsula tem extremidades arredondadas: o contato **rola** conforme o corpo inclina, em vez de pivotar sobre uma aresta. O CoP se desloca de forma não linear e a "borda" do suporte é mal definida.
- As três condições do CWC (§5) foram derivadas para **superfícies retangulares**. Com cápsula, a condição (ii) degenera e a (iii) — limites de torque de yaw — perde sentido, porque um contato de linha praticamente não resiste a torque de yaw. O pé vai girar.

**Recomendação [CRÍTICO]:** trocar o collider dos pés por uma **caixa** (`PxBoxGeometry`) com dimensões próximas às do FootWork cinemático — 0,28 × 0,115 × 0,055 m — alinhada ao frame anatômico do pé. Isso:
- cria um polígono de suporte real e retangular;
- torna as fórmulas fechadas do CWC diretamente aplicáveis;
- alinha o pé físico ao pé que o FootWork já planeja (hoje eles têm geometrias diferentes, o que produziria discrepância sistemática entre suporte planejado e suporte físico);
- faz a apresentação visual passar a coincidir com a física, em vez de mascará-la.

O requisito de "espessura uniforme para não ter tronco gordo" é legítimo, mas é sobre **estética do tronco**, não sobre o pé. Vale abrir exceção explícita no perfil: `colliderShape: capsule | box` por link.

**[ABERTO]** Avaliar também trocar as **mãos** por caixas, pela mesma razão, mas com prioridade menor — mãos importam para levantar-se, não para ficar em pé.

### 16.2 [CRÍTICO] Verificar `eDRIVE_LIMITS_ARE_FORCES`

Ver §14.2. Se a flag não estiver setada, o `torque máximo` do JSON está sendo aplicado como impulso e os torques disponíveis são ~120× menores do que o perfil declara a 120 Hz. Sintoma típico: "o ragdoll não consegue se sustentar nem com rigidez 100%".

### 16.3 O pé é um único link, sem dedos

O documento registra explicitamente: *"Não existem links separados para os dedos dos pés."* Consequências reais:

- **Não há push-off.** A propulsão humana na fase terminal do apoio vem majoritariamente do rolamento sobre o antepé e da flexão plantar com o calcanhar já elevado. Sem articulação metatarsal, a velocidade máxima de caminhada e a qualidade da corrida ficam limitadas.
- O CoP não pode avançar além da borda anterior do pé rígido, o que **reduz a autoridade da estratégia de tornozelo para frente**.

Não é bloqueante para "ficar em pé e equilibrar". É uma limitação conhecida a documentar, e um candidato natural a evolução do perfil quando chegar a corrida (etapa 7 do roadmap existente). Adicionar 2 links (um por pé) custa 2 DOFs e resolve a maior parte disso.

### 16.4 Rigidez global escalar é insuficiente

O modelo atual (`stiffness = base × rigidez^2,2`, etc., com um escalar global) foi certo para o laboratório de ragdoll passivo. Para controle, os ganhos precisam variar **por junta, por estado e por fase** — perna de apoio firme, perna em swing leve, tronco com damping alto, braços leves. Isso já está corretamente antecipado no documento de estado (§38 de lá).

A recomendação adicional é: usar `eACCELERATION` para os drives de **postura** (independente de massa) e reservar `eFORCE` para onde o limite de torque anatômico precisa morder. O perfil continua sendo o **teto**; o controlador envia escalas.

### 16.5 Contatos: generalizar desde já

O `FootContactState3D` esboçado é ótimo, mas está tipado para pés. Ver §11: levantar-se exige contatos de mãos, antebraços, joelhos e tronco. Generalize para um `ContactSet` indexado por link, com os pés apenas marcados como "links de contato prioritários". O custo agora é uma indireção; o custo depois é reescrever o estimador de equilíbrio.

### 16.6 Iterações do solver

4 posição / 1 velocidade é adequado para ragdoll passivo. Com drives ativos + contatos + limites de junta, a doc do PhysX indica que convergência pode exigir mais iterações de posição. **[ABERTO]** Medir. Provavelmente 8/2 ou 8/4 para personagens em controle ativo, mantendo 4/1 para os passivos (via LOD).

### 16.7 Spawn

Hoje a pelve fica "a aproximadamente 98 cm do solo na pose neutra". Para spawn automático em pé:
- raycast para achar o solo sob a posição de spawn;
- posicionar a raiz de modo que **as solas** fiquem no solo, não a pelve a 98 cm;
- aplicar a pose de postura de referência via cache **antes** de adicionar à cena ou no primeiro safe point;
- zerar velocidades;
- ativar compensação de gravidade e ganhos de postura **desde o frame 0**;
- período curto de "assentamento" (~0,2 s) com damping elevado e ganho de tarefa reduzido, para o solver acomodar contatos sem impulso inicial.

---

# PARTE IV — ARQUITETURA PROPOSTA

## 17. Princípios

**[INVARIANTE] 1 — Toda atuação é interna.** Torques de junta (drives + `eFORCE` no cache). As únicas forças externas legítimas são gravidade, contatos e eventos genuínos do mundo. Nenhuma força de estabilização na pelve, jamais.

**[INVARIANTE] 2 — Nenhum controlador escreve transform.** Já está no documento de estado e deve permanecer.

**[INVARIANTE] 3 — Cada camada tem structs de entrada e saída e é testável isolada.** O núcleo de controle não conhece PhysX; conhece `RagdollProfile3D`, `RagdollState3D`, `ContactSet` e devolve `RagdollAxisTarget3D[]` + torques por DOF.

**[INVARIANTE] 4 — O modelo de contato é genérico.** Nunca "dois pés".

**[INVARIANTE] 5 — Nada é comandado sem passar por admissibilidade e saturação.** O controlador sempre sabe quando está pedindo o impossível, e degrada com prioridade explícita em vez de saturar em silêncio.

**[INVARIANTE] 6 — O FootWork é conselheiro, não autoridade.** Ele diz *onde seria bom pisar*. Quem decide se e como o pé chega lá é o controlador, via torques. Um contato antecipado ou um pé que não alcança o alvo são resultados válidos, não erros.

## 18. Camadas

Refinamento das camadas já esboçadas no documento de estado, agora com o conteúdo teórico das Partes I e II.

```
┌───────────────────────────────────────────────────────────────┐
│ L0  TELEMETRIA FÍSICA                                         │
│     snapshot → CoM, ĊoM, contatos, CoP por contato e global,  │
│     polígono de suporte físico, momento centroidal, capture   │
│     point, saturação por junta                                │
└───────────────────────────┬───────────────────────────────────┘
                            ▼
┌───────────────────────────────────────────────────────────────┐
│ L1  ESTIMADOR DE EQUILÍBRIO                                   │
│     classifica: estável / recuperável por tornozelo /         │
│     por quadril / exige passo / queda                         │
│     produz: urgência, direção, margem, estratégia sugerida    │
└───────────────────────────┬───────────────────────────────────┘
                            ▼
┌───────────────────────────────────────────────────────────────┐
│ L2  MÁQUINA DE ESTADOS DO PERSONAGEM                          │
│     Passive · Settling · Standing · Balancing · Stepping ·    │
│     Walking · Running · Stumbling · Falling · Grounded ·      │
│     GettingUp                                                 │
│     define política de passos, conjunto de ganhos, prioridade │
│     de tarefas, contatos esperados                            │
└───────────────────────────┬───────────────────────────────────┘
                            ▼
┌──────────────────────┬────────────────────────────────────────┐
│ L3a FOOTWORK         │ L3b GERADOR DE REFERÊNCIA CORPORAL     │
│  (já existe)         │  postura de tronco, pelve, braços,     │
│  + feedback físico   │  cabeça; altura do CoM; orientação     │
│  + correção capture  │                                        │
└──────────┬───────────┴───────────────┬────────────────────────┘
           ▼                           ▼
┌───────────────────────────────────────────────────────────────┐
│ L4  RESOLUÇÃO CENTROIDAL                                      │
│     ḣ_desejado (linear + angular) a partir de CoM/capture     │
│     → ADMISSIBILIDADE (CWC) → ḣ_admissível                    │
│     → distribuição de wrench entre contatos ativos            │
└───────────────────────────┬───────────────────────────────────┘
                            ▼
┌───────────────────────────────────────────────────────────────┐
│ L5  COMPOSIÇÃO DE TORQUE                                      │
│     τ = τ_grav + τ_cor + τ_wrench(Jᵀ) + τ_swing + τ_postura   │
│         + τ_reflexos       →  saturação por junta             │
└───────────────────────────┬───────────────────────────────────┘
                            ▼
┌───────────────────────────────────────────────────────────────┐
│ L6  CAMADA DE DRIVES (backend-neutra)                         │
│     RagdollAxisTarget3D[] + jointForce[] → cache → PhysX      │
└───────────────────────────────────────────────────────────────┘
```

**Onde o QP entra depois:** L4 e L5 são exatamente as duas camadas que um QP de corpo inteiro unifica. Se elas tiverem contratos limpos, trocar as duas por um QP é uma substituição, não uma refatoração. **[ABERTO]** Manter L4/L5 atrás de uma interface `IBodyResolver` com duas implementações previstas: `AnalyticResolver` (agora) e `QpResolver` (depois).

**Onde a política aprendida entra depois:** substituindo L3b+L4+L5 e emitindo direto para L6. Ver §9.

## 19. L0 — Telemetria

```cpp
struct ContactPoint3D {
    std::uint32_t linkIndex;
    Vec3  position;
    Vec3  normal;
    float normalImpulse;
    float tangentialSpeed;
};

struct ContactSet3D {
    // genérico: qualquer link pode ser contato (§16.5)
    std::span<const ContactPoint3D> points;
    std::uint32_t contactingLinkMask;
};

struct BodyTelemetry3D {
    Vec3  com, comVelocity;
    float comHeight;              // acima do plano de suporte
    Vec3  centroidalAngularMomentum;
    Vec3  copGlobal;
    Vec3  copPerLink[/*contatos*/];
    ConvexPolygon2D supportPolygon;   // fecho convexo dos contatos, projetado
    float supportMargin;              // distância do CoP à borda
    Vec3  capturePoint;
    float captureMargin;              // distância do capture point à borda
    Vec3  bodyUp, bodyForward;
    float torqueSaturation[41];       // [0,1] por DOF
};
```

Cálculos:

```
CoM        = Σ mᵢ (pᵢ + Rᵢ · comLocalᵢ) / Σ mᵢ
ĊoM        = Σ mᵢ vᵢ / Σ mᵢ
CoP        = Σ (posᵢ · impulsoNormalᵢ) / Σ impulsoNormalᵢ
ω          = √(g / comHeight)
ξ          = CoM_xz + ĊoM_xz / ω
```

**[ABERTO]** `computeArticulationCOM()` do PhysX pode substituir o cálculo manual do CoM, mas note que ele usa a pose corrente da articulation e não dá velocidade. Calcular manualmente a partir do `RagdollState3D` — que já publica posições, velocidades lineares e angulares de todos os links — mantém o módulo neutro e dá as duas grandezas. Recomendo o cálculo manual.

**Sobre contatos:** o documento de estado registra que contatos de ragdoll hoje não geram stream detalhado, por otimização. A solução ali proposta — marcar links como sensores semânticos e solicitar dados mínimos apenas quando participam — é a correta. Generalize a marcação para "links de contato relevantes ao estado atual": em `Standing` são os pés; em `GettingUp` são mãos, antebraços, joelhos, pelve e tronco.

**Nota importante:** um raycast sob o pé informa geometria, mas **não** informa se o pé está sustentando peso, escorregando ou recebendo impacto. Só o solver sabe. Manter os dois: raycast para o FootWork (planejamento), contato real para o equilíbrio (execução). Isso já está corretamente identificado no documento de estado e vale reforçar como invariante.

## 20. L1 — Estimador de equilíbrio

Saída:

```cpp
enum class BalanceVerdict {
    Stable,          // capture point folgado dentro do suporte
    AnkleRecovery,   // dentro, mas perto da borda
    HipRecovery,     // fora do suporte, dentro da região de captura ampliada por counter-rotation
    StepRequired,    // fora; existe passo alcançável
    Falling          // fora; nenhum passo alcança
};

struct BalanceState3D {
    BalanceVerdict verdict;
    float   urgency;              // [0,1] normalizado pela margem
    Vec3    errorDirection;
    Vec3    desiredComPosition;
    Vec3    desiredComVelocity;
    Vec3    capturePoint;
    float   hipBudgetRemaining;   // [0,1] — §4
    bool    doubleSupport;
};
```

Regras de decisão:

```
margem = distância(ξ, borda do polígono de suporte)     // negativa se fora

margem > m_folga                      → Stable
0 < margem ≤ m_folga                  → AnkleRecovery
margem ≤ 0 e |margem| ≤ r_counterrot  → HipRecovery   (se hipBudget > 0)
margem ≤ 0 e passo alcança            → StepRequired
caso contrário                        → Falling
```

`r_counterrot` é o raio de ampliação da região de captura pelo momento angular disponível — a versão barata é uma constante calibrada (algo como 6–10 cm para escala humana); a versão correta vem do modelo LIPM+flywheel.

**Peso variável [INVARIANTE]:** suprimir a estratégia de quadril para erros pequenos e liberá-la progressivamente para erros grandes (§4). Usar quadril para micro-correções produz movimento visivelmente errado — o boneco fica "se contorcendo" parado.

**Histerese obrigatória** entre veredictos. Sem ela, o personagem oscila entre `AnkleRecovery` e `HipRecovery` a 120 Hz.

## 21. L4 — Resolução centroidal

### 21.1 Momento desejado

```
ḣ_desejado   = k_p · m · (c_d − c)  +  k_d · m · (ċ_d − ċ)  +  m·g
l̇_desejado   = −k_l · l                                          // amortecimento de CAM
```

O termo `−k_l · l` é o controle de momento angular: **empurra o momento angular centroidal para zero**. É ele que faz o tronco e os braços se moverem de forma coordenada para absorver um empurrão. Ativado proporcionalmente à urgência e ao `hipBudget`.

### 21.2 Admissibilidade

O passo que quase todo mundo pula e que é a diferença entre um controlador robusto e um que escorrega. Dado `(ḣ_desejado, l̇_desejado)`, calcular a GRF resultante e o CoP implícito, e verificar as três condições do CWC (§5):

```
1. ‖f_tangencial‖ ≤ μ · f_normal                   (não escorrega)
2. CoP ∈ polígono de suporte (com margem)          (não tomba)
3. τ_yaw ∈ [limite_inf, limite_sup]                (não gira)
```

Se inadmissível: **projetar** sobre o conjunto admissível com prioridade declarada. A prioridade padrão, seguindo Lee & Goswami, é **preservar o objetivo de momento linear e comprometer o angular** — porque perder o controle do CoM leva à queda, enquanto perder controle do momento angular leva apenas a postura feia.

Registrar o comprometimento na telemetria. Um personagem que fica cronicamente inadmissível é um personagem que precisa dar um passo — este é, aliás, um excelente sinal complementar para disparar `StepRequired`.

### 21.3 Distribuição entre contatos

Em duplo apoio, o wrench total precisa ser dividido entre os dois pés. A abordagem barata e suficiente:

```
peso_pé = f(distância do CoP desejado ao centro de cada pé, área de contato, atrito local)
```

com normalização e clamp para que cada pé receba apenas o que consegue (CoP dentro do seu próprio retângulo, atrito local respeitado). A abordagem correta é um pequeno QP com restrições de cone — ~12 variáveis. Fica para a etapa 9 da escada.

**Terreno irregular:** o método de Lee & Goswami determina GRF e CoP **por pé**, o que é justamente o que permite lidar com solo não plano e atritos diferentes. Vale adotar essa estrutura desde já, mesmo com a heurística simples, para que a troca por QP seja local.

## 22. L5 — Composição de torque

```
τ_total = τ_grav                     // G(q), feedforward — SEMPRE ativo
        + τ_cor                      // C(q,q̇)q̇, feedforward — locomoção
        + τ_wrench                   // Jᵀ · wrench admissível, roteado pelas cadeias de apoio
        + τ_swing                    // rastreio do pé em swing (task-space PD via Jᵀ, ganho moderado)
        + τ_postura                  // PD nas juntas para a pose de referência (via drives)
        + τ_reflexos                 // termos locais específicos de fase
```

Notas por termo:

**`τ_grav`** — obrigatório, todo tick, todo personagem ativo, inclusive `Passive`? Não: em `Passive` e `Grounded` deve ser zerado, senão o corpo "flutua" morto. Em `Falling`, reduzir progressivamente.

**`τ_wrench`** — este é o coração. A força virtual no CoM (§6.4) é convertida em torques pela Jacobiana transposta da cadeia que vai do pé de apoio ao CoM. Em duplo apoio, dividir conforme §21.3. **É este termo que implementa tornozelo + parte de quadril.**

**`τ_swing`** — a perna em swing rastreia o alvo do FootWork com **ganho baixo e torque limitado**. Deliberadamente frouxo: uma perna em swing rígida atrapalha o equilíbrio (ela troca momento angular com o resto do corpo) e produz colisões duras. E lembre-se do invariante 6: se o pé encostar antes, aceite.

**`τ_postura`** — via drives, não via `eFORCE`. É o que mantém a forma do corpo. Ganhos por estado. Recomendo `eACCELERATION` aqui (§14.2).

**`τ_reflexos`** — pequeno conjunto de termos locais inspirados em §8, adicionados por último:
- anti-hiperextensão de joelho (torque flexor crescente perto do limite de extensão);
- estabilização do tronco durante o apoio (realimentação de orientação roteada para o quadril de apoio);
- push-off no tornozelo de apoio no fim da fase (realimentação positiva de força, saturada);
- reação de quadril à aceleração da perna em swing.

Cada um é 5–15 linhas e cada um resolve um artefato visual específico. Adicione um de cada vez, com toggle.

### 22.1 Saturação

```
τ_final[i] = clamp(τ_total[i], −τ_max[i]·escala_estado, +τ_max[i]·escala_estado)
```

E registrar `saturação[i] = |τ_total[i]| / τ_max[i]`. **[INVARIANTE]** Saturação persistente em uma junta é um sinal de diagnóstico de primeira classe, não um detalhe. Exponha no HUD.

Adicionalmente, conforme já previsto no documento de estado: limite de variação do alvo por tick, limite de potência, redução de rigidez após impactos fortes, proteção contra wind-up.

## 23. Máquina de estados

```
                      ┌──────────┐
                      │ Passive  │◄─────────────┐
                      └────┬─────┘              │
                     spawn │                    │ desativar
                           ▼                    │
                     ┌──────────┐               │
                     │ Settling │               │
                     └────┬─────┘               │
                          ▼                     │
   ┌────────────────► ┌──────────┐              │
   │                  │ Standing │              │
   │                  └────┬─────┘              │
   │      perturbação      │      comando       │
   │        ┌──────────────┼──────────────┐     │
   │        ▼              │              ▼     │
   │  ┌───────────┐        │        ┌──────────┐│
   │  │ Balancing │        │        │ Walking  ││
   │  └─────┬─────┘        │        └────┬─────┘│
   │        │ StepRequired │             │      │
   │        ▼              │             ▼      │
   │  ┌─────────────┐      │        ┌──────────┐│
   │  │RecoveryStep │      │        │ Running  ││
   │  └─────┬───────┘      │        └────┬─────┘│
   │        │ sucesso      │             │      │
   └────────┘              │             │      │
            │ falha        │             │      │
            ▼              ▼             ▼      │
      ┌───────────┐                             │
      │ Stumbling │                             │
      └─────┬─────┘                             │
            ▼                                   │
      ┌──────────┐    ┌───────────┐             │
      │ Falling  │───►│ Grounded  │─────────────┘
      └──────────┘    └─────┬─────┘
                            │ (etapa futura)
                            ▼
                      ┌───────────┐
                      │ GettingUp │
                      └───────────┘
```

Cada estado define, como já apontado no documento de estado: política de passos, alvos articulares, conjunto de ganhos, escala de torque, contatos esperados e condições de transição. Acrescento:

- **`Settling`** — 0,2 s após spawn. Damping alto, ganho de tarefa reduzido, sem passos. Existe para o solver acomodar os contatos sem impulso.
- **`Falling`** — não é "desligar tudo". É: reduzir drives progressivamente, **manter** proteção de cabeça e tronco (torque para o pescoço flexionar e os braços virem à frente), e preparar o `ContactSet` para receber contatos de mãos e antebraços. Isso é o que torna a queda "viva" em vez de um boneco caindo.
- **`Grounded`** — estado terminal estável, com drives mínimos mantendo uma pose relaxada plausível. É daqui que `GettingUp` partirá.

**[INVARIANTE]** Transições disparadas por **contato e por veredito de equilíbrio**, nunca por tempo puro. Tempo entra apenas como timeout de segurança e como histerese.

## 24. Integração com o FootWork

O documento de estado já descreve corretamente o `FootworkFeedback3D`. Complemento com o essencial:

```cpp
struct FootworkFeedback3D {
    Vec3  pelvisPosition;
    Vec3  centerOfMassPosition;
    Vec3  centerOfMassVelocity;
    float bodyYawRadians;
    FootPose3D actualLeftFoot, actualRightFoot;
    FootContactState3D leftContact, rightContact;
    // acréscimos:
    Vec3  capturePoint;
    float balanceUrgency;       // [0,1] — modula agressividade do passo
    BalanceVerdict verdict;
    float legReachActual;       // comprimento real da perna, para alcance
};
```

E a correção do alvo, que é o gancho que você já previu:

```
alvo_final = alvo_de_marcha
           + k_cp · (ξ − ξ_nominal)          // correção de capture point
           + c_d·d + c_v·v                   // realimentação estilo SIMBICON (§7.1)
```

com clamp, suavização e validação de terreno — exatamente como o documento de estado já prescreve.

Mudanças de comportamento do FootWork que passam a valer:
- a raiz virtual deixa de ser autoritativa e passa a ser **intenção**, ancorada na pelve/CoM real;
- uma passada **só termina com contato confirmado**, nunca por relógio;
- perda de contato do pé de apoio dispara recuperação imediata;
- atraso do pé físico em relação ao plano modifica a cadência;
- o alcance usa o comprimento real da perna.

**[INVARIANTE]** O `supporte planejado` (FootWork) e o `suporte físico` (contatos) são grandezas **diferentes** e ambas devem existir. O equilíbrio usa o físico. O planejamento usa o planejado. Divergência entre os dois é sinal de recuperação necessária.

## 25. Ordem de atualização

Refinamento da ordem já estabelecida:

```
 1. ler snapshot + ContactSet do passo anterior
 2. L0  telemetria (CoM, CoP, polígono, capture point, saturação)
 3. L1  estimador de equilíbrio → veredito
 4. L2  máquina de estados → política, ganhos, prioridades
 5. L3a FootWork (com feedback físico + correção de capture point)
 6. L3b gerador de referência corporal
 7.     applyCache(pose) + commonInit()             ← protocolo §13.1
 8.     computeGravityCompensation
 9.     computeCoriolisCompensation                 (se necessário no estado)
10.     computeDenseJacobian / Jacobiana analítica  (se necessário)
11.     computeMassMatrix + computeCentroidalMomentumMatrix  (LOD)
12. L4  resolução centroidal + admissibilidade + distribuição
13. L5  composição de torque + saturação
14. L6  enfileirar RagdollAxisTarget3D[] + jointForce[]
15.     applyCache(eFORCE) + setDriveTargets
16.     PhysicsScene3D::simulate(1/120)
17.     fetchResults
18.     publicar snapshots + ContactSet
19.     renderizar / interpolar
```

**[CRÍTICO]** Os passos 7–11 só podem ocorrer fora da simulação. Isso é uma restrição do SDK, não uma preferência.

**Atraso de um passo:** o feedback tem ~8,33 ms de atraso a 120 Hz. Isso já está identificado no documento de estado. Na prática significa: use os ganhos derivativos com parcimônia, e considere um preditor simples (extrapolar CoM e ĊoM por um passo) no L0. Um preditor de um passo custa quase nada e melhora sensivelmente a estabilidade de laços com ganho alto.

## 26. Estrutura de módulos

**[ABERTO]** Adaptar à convenção existente. A separação em anéis é que é invariante.

```
src/Engine/Control/                    ← núcleo neutro, NÃO conhece PhysX
├── BodyTelemetry3D.{hpp,cpp}          // L0 (calcula de Profile + State + ContactSet)
├── BalanceEstimator3D.{hpp,cpp}       // L1
├── CharacterStateMachine3D.{hpp,cpp}  // L2
├── BodyReferenceGenerator3D.{hpp,cpp} // L3b
├── IBodyResolver3D.hpp                // interface L4+L5
├── AnalyticBodyResolver3D.{hpp,cpp}   // implementação analítica
├── ContactWrenchCone3D.{hpp,cpp}      // admissibilidade (§5)
├── SupportPolygon3D.{hpp,cpp}         // fecho convexo genérico
├── TorqueComposer3D.{hpp,cpp}         // L5
├── Reflexes3D.{hpp,cpp}               // §22 τ_reflexos
├── GainProfiles3D.{hpp,cpp}           // ganhos por estado/junta
└── ControlTypes3D.hpp

src/Engine/Physics/
├── PhysicsScene3D.hpp                 // + API de drives por eixo, joint force, ID
└── PhysX/
    ├── PhysXScene3D.cpp
    └── PhysXArticulationDynamics.cpp  // ÚNICO lugar que chama computeGravityCompensation etc.

src/Workbench/Laboratory/
├── ActiveRagdollLab.{cpp,hpp}         // modo, popup, sliders, spawn
└── BalanceDebugRenderer.{cpp,hpp}     // overlay (§28)

tests/
├── ControlUnitTests.cpp
├── BalanceFunctionalTests.cpp
└── FakeArticulation.hpp               // dinâmica sintética para testes sem PhysX
```

Teste de sanidade: `grep -r "Px" src/Engine/Control/` deve retornar zero.

---

# PARTE V — ROADMAP

Cada etapa é entregável, testável e não invalida a anterior. Não avance sem os critérios.

### Etapa 0 — Correções de fundação
- Collider dos pés → caixa (§16.1).
- Verificar/setar `eDRIVE_LIMITS_ARE_FORCES` (§16.2).
- Armature nos links leves.
- Iterações do solver ajustadas para personagens ativos.
- Spawn com raycast + pose de referência + `Settling`.

**Critério:** ragdoll passivo solto de 10 cm sobre o solo assenta com os dois pés apoiados em superfície plana; polígono de suporte medido tem área > 0,03 m² em duplo apoio (hoje seria ≈ 0).

### Etapa 1 — Telemetria (L0)
- CoM, ĊoM, contatos genéricos, CoP por contato e global, polígono de suporte físico, capture point, momento centroidal, saturação.
- Overlay de debug (§28).

**Critério:** empurrar o ragdoll com a Physgun move CoM e capture point de forma coerente e visível; CoP permanece sempre dentro do polígono; nenhum valor não finito em 10.000 passos.

### Etapa 2 — Drives por eixo + compensação de gravidade
- API batch dos 41 DOFs (`RagdollAxisTarget3D`), aplicada no safe point.
- `computeGravityCompensation` a cada tick, aplicada via `eFORCE`.
- Telemetria de saturação por junta.

**Critério:** com o ragdoll **suspenso no ar** (base travada temporariamente ou segurado pela Physgun) e ganhos de postura **zerados**, ele mantém a pose contra a gravidade apenas com o feedforward, sem cair e sem estalo. Este é o teste isolado mais valioso do projeto inteiro — se ele passa, a metade difícil da física está correta.

### Etapa 3 — Postura em pé
- Gerador de referência corporal, ganhos por junta, `eACCELERATION` para postura.
- Pés ainda sem controle de equilíbrio ativo; apenas postura + gravidade.

**Critério:** ficar em pé em piso plano por 30 s sem intervenção. Provavelmente vai oscilar — isso é esperado e é o que a Etapa 4 resolve.

### Etapa 4 — Equilíbrio estático (tornozelo)
- Estimador de equilíbrio, força virtual no CoM via Jacobiana transposta, admissibilidade CWC, distribuição simples entre dois pés.

**Critério:** resistir a impulsos definidos em 8 direções sem dar passos e sem escorregar; CoP se move visivelmente na direção correta; nenhuma violação do cone de atrito registrada.

### Etapa 5 — Estratégia de quadril
- Amortecimento de momento angular centroidal via CMM; braços e tronco entram como atuadores; orçamento de quadril e peso variável.

**Critério:** impulsos maiores são absorvidos com contrarrotação visível de tronco e braços, e o corpo **retorna** à postura depois. Nenhuma contrarrotação para perturbações pequenas.

### Etapa 6 — Passo de recuperação
- FootWork recebe correção de capture point; passada termina por contato; transição de apoio; reduzir rigidez do membro em swing.

**Critério:** recuperar-se de empurrões grandes com um passo, em 8 direções; contato confirma o fim da passada; nenhum pé fica suspenso.

### Etapa 7 — Caminhada
- Velocidade desejada, alternância, rastreio contínuo do pé em swing, reflexos de tronco e joelho, braços em contrabalanço.

**Critério:** caminhar, parar, inverter direção e virar sem foot skating e sem queda; pé de apoio não desliza (velocidade tangencial de contato abaixo do limiar); determinismo preservado.

### Etapa 8 — Corrida
- Fase aérea real, push-off por reflexo no tornozelo, absorção de aterrissagem, cadência derivada do estado físico, transição caminhada↔corrida.

**Critério:** correr sem depender da raiz cinemática; fase aérea com ambos os pés sem contato confirmada na telemetria.

### Etapa 9 — Levantar-se
Ver §29.

### Etapa 10 — Futebol e contatos
- Tackles, disputa de ombro, chute, recuperação de tropeço, controle orientado à bola, 22 jogadores ativos com LOD.

**Critério:** estabilidade e orçamento de tempo sob contatos simultâneos.

---

# PARTE VI — PARÂMETROS INICIAIS

Ponto de partida para calibração, escala humana adulta, 75 kg, perna ≈ 0,9 m.

```
telemetria
  com_height_nominal          0.95 m
  omega = sqrt(9.81/0.95)     ≈ 3.21 rad/s
  contact_impulse_threshold   0.5 N·s      (considerar "tocando")

equilíbrio
  margem_folga                0.04 m
  raio_counterrotation        0.08 m
  histerese_veredito          0.02 m
  k_p_com                     ~40  1/s²    (freq. natural ≈ 1 Hz)
  k_d_com                     ~12  1/s
  k_l_cam                     ~8   1/s
  hip_budget_recharge         1.5 s

contato
  mu_assumido                 0.7  (usar o material real quando disponível)
  margem_cop                  0.015 m  (encolhimento do retângulo do pé)
  margem_cone_atrito          0.85     (usar 85% do cone)

ganhos de postura (escala relativa ao teto do perfil)
  perna de apoio: quadril     0.85   joelho 0.85   tornozelo 0.60
  perna em swing: quadril     0.35   joelho 0.35   tornozelo 0.20
  tronco (abdômen/peito)      0.60   damping alto
  pescoço/cabeça              0.30
  braços                      0.25
  mãos                        0.10

torque
  escala por estado: Standing 1.0 · Balancing 1.0 · Walking 0.9
                     Stumbling 0.7 · Falling 0.3 → 0.05 · Grounded 0.1
  taxa máxima de variação do alvo   6 rad/s
  redução pós-impacto               ×0.6 por 0.15 s

drives
  driveType postura           eACCELERATION
  driveType apoio             eFORCE (limitado pelo perfil)
  eDRIVE_LIMITS_ARE_FORCES    true                       ← §16.2
  armature (mãos, pés, pescoço)  0.01–0.05

solver (personagens ativos)
  iterações posição / velocidade   8 / 2
  passo                            1/120 s
```

**[ABERTO]** Todos os valores acima devem ser data-driven e ajustáveis por slider no laboratório, com hot-reload.

---

# PARTE VII — TELEMETRIA E DEBUG

Sem isto, calibrar é impossível. Cada item com toggle independente.

| Elemento | Representação |
|---|---|
| CoM | esfera; rastro dos últimos 0,5 s |
| Vetor ĊoM | seta a partir do CoM |
| Capture point | cruz no solo, cor por veredito |
| Polígono de suporte **físico** | polígono preenchido translúcido |
| Polígono de suporte **planejado** (FootWork) | contorno pontilhado |
| CoP global | ponto; CoP por pé em cor distinta |
| Cone de atrito por contato | cone wireframe no ponto de contato |
| GRF por contato | seta proporcional ao impulso normal |
| Momento angular centroidal | seta a partir do CoM |
| Força virtual no CoM (`τ_wrench`) | seta em cor distinta |
| Torque por junta | barra ou cor no link, com **vermelho ao saturar** |
| Veredito + urgência | HUD textual |
| Orçamento de quadril | barra |
| Admissibilidade | indicador; realçar quando houve projeção (§21.2) |
| Alvos do FootWork vs. pose real do pé | par de contornos + linha de erro |

Botão **"congelar / avançar um tick"** — indispensável.

Gráficos temporais (últimos 5 s) de: margem de captura, saturação máxima, `|l|`, velocidade tangencial máxima de contato. É nesses gráficos que os problemas aparecem antes de virarem quedas.

---

# PARTE VIII — TESTES

### Unitários (sem PhysX, com `FakeArticulation`)
- CoM e ĊoM corretos para pose e velocidades conhecidas.
- CoP para distribuição de contatos conhecida.
- Fecho convexo de suporte, incluindo casos degenerados (1 ponto, colinear).
- Capture point para estado conhecido.
- CWC: aceita wrench admissível, rejeita e projeta os três tipos de violação (atrito, CoP, yaw).
- Conversão de alvo articular para os eixos anatômicos `TWIST/SWING1/SWING2`.
- Clamps de torque e de taxa.
- Determinismo bit a bit para a mesma sequência de fixed steps.

### Funcionais
- Ficar em pé 30 s (plano, rampa 5°, rampa 15°).
- Empurrões calibrados em 8 direções, três magnitudes (recuperável por tornozelo / por quadril / por passo).
- Perda súbita de um contato (remover o chão sob um pé).
- Plataforma que se move sob os pés.
- Caminhar, parar, inverter, virar.
- Corrida.
- Tropeço (obstáculo baixo no caminho do pé em swing).
- Queda completa → `Grounded` estável.

### Escala
- 22 ragdolls ativos equilibrando simultaneamente.
- Contatos entre jogadores.
- Nenhum estado não finito.
- Tempo de física + controle dentro do orçamento, com LOD ativo e com LOD desativado (para medir o ganho).

### Métricas de regressão (registrar em cada build)
- Impulso máximo recuperado sem passo, por direção.
- Impulso máximo recuperado com um passo.
- Velocidade tangencial máxima do pé de apoio durante equilíbrio (proxy de escorregamento).
- Saturação média e máxima por junta.
- Fração de ticks com wrench inadmissível.

---

# PARTE IX — PREPARAÇÃO PARA LEVANTAR-SE

Não implementar agora. Garantir agora que estas quatro coisas sejam verdade:

1. **`ContactSet` genérico.** Qualquer link pode ser contato (§16.5). Mãos, antebraços, joelhos, pelve, tronco.
2. **`SupportPolygon3D` genérico.** Fecho convexo de um conjunto arbitrário de contatos projetados no plano de suporte local. Nada de "dois retângulos".
3. **Fonte de pose de referência abstraída.** `IPoseProvider` com implementações previstas: `ProceduralPosture` (agora), `ClipSequence` (levantar por trajetória predefinida), `LearnedPolicy` (futuro). A máquina de estados escolhe o provider; o resto do pipeline não muda.
4. **`Grounded` é um estado real e estável**, com pose relaxada plausível e drives mínimos — não "drives desligados". Levantar-se parte de um estado conhecido, não do caos.

Com isso, `GettingUp` na Etapa 9 vira: uma sequência de fases com `IPoseProvider` de clipes ou curvas, transições **condicionadas por contato**, e o mesmo `TorqueComposer` de sempre. É por isso que a estrutura importa mais do que a feature.

Sobre a abordagem: comece por trajetória predefinida a partir de supino e prono, que é o que cobre a maioria esmagadora dos casos, já que o corpo naturalmente rola para uma dessas configurações após uma queda. Generalizar para posturas arbitrárias é o problema que os trabalhos de RL atacam, e fica para muito depois.

---

# PARTE X — ARMADILHAS

| Armadilha | Por que mata o projeto |
|---|---|
| Aplicar força na pelve para equilibrar | Viola o invariante 1; viola conservação de momento; parece errado e é errado (§1) |
| Escrever transform de link | Já proibido no documento de estado; quebra o solver reduced-coordinate |
| Pé como cápsula | Polígono de suporte degenerado; estratégia de tornozelo impossível (§16.1) |
| Compensar falta de feedforward com ganho alto | O SDK alerta que drives rígidos + contatos + limites não convergem (§14.4) |
| Pular o teste de admissibilidade | O pé escorrega, gira ou tomba e você não sabe por quê (§5, §21.2) |
| Transições de estado por relógio | Contato antecipado/atrasado é a norma, não a exceção |
| Perna em swing rígida | Troca momento angular com o corpo e desestabiliza o apoio |
| Estratégia de quadril sempre ligada | O boneco se contorce parado; humanos usam como reserva (§4) |
| Assumir dois pés no equilíbrio | Impossibilita levantar-se sem reescrever (§11, §29) |
| Chamar dinâmica inversa durante `simulate()` | Ilegal no SDK (§13.1) |
| Usar `computeGeneralizedGravityForce` | Depreciado; não retorna forças da raiz para base flutuante — que é o nosso caso (§13.1) |
| Usar `eJOINT_SOLVER_FORCES` | Depreciado; valores incorretos (§14.3) |
| Ignorar `eDRIVE_LIMITS_ARE_FORCES` | Torques ~120× menores do que o perfil declara (§16.2) |
| Jacobiana densa completa por tick × 22 | Desperdício; use a submatriz ou a cadeia analítica (§15) |
| Começar por RL | Sem infra, sem determinismo, sem entendimento, sem orçamento (§9) |
| Constantes mágicas no código | Impossibilita calibração (Parte VI) |

---

# 30. Referências

### Subatuação, dinâmica e controle de corpo inteiro
- Khatib, O. — *A unified approach for motion and force control of robot manipulators: the operational space formulation* (origem do controle em espaço operacional).
- Herzog, A. et al. — *Momentum control with hierarchical inverse dynamics on a torque-controlled humanoid*, Autonomous Robots 40(3), 2015.
- *Whole-Body Control Framework for Humanoid Robots with Heavy Limbs: A Model-Based Approach* — arXiv:2506.14278. https://arxiv.org/pdf/2506.14278
- *Assessing Whole-Body Operational Space Control in a Point-Foot Series Elastic Biped* — arXiv:1501.02855. https://arxiv.org/pdf/1501.02855
- *Humanoid Whole-Body Controllers* (panorama da formulação QP canônica). https://www.emergentmind.com/topics/humanoid-whole-body-controllers
- *First do not fall: learning to exploit a wall with a damaged humanoid robot* (formulação QP explícita) — arXiv:2203.00316. https://arxiv.org/pdf/2203.00316

### Pontos de referência no solo, capture point, DCM
- Vukobratović, M. & Borovac, B. — *Zero-moment point: thirty five years of its life*, IJHR 1(1), 2004.
- Popovic, M. B., Goswami, A. & Herr, H. — *Ground reference points in legged locomotion: definitions, biological trajectories and control implications*, IJRR 24(12), 2005.
- Pratt, J., Carff, J., Drakunov, S. & Goswami, A. — *Capture Point: A Step toward Humanoid Push Recovery*, Humanoids 2006. https://www.researchgate.net/publication/224060461_Capture_Point_A_Step_toward_Humanoid_Push_Recovery
- *Determination of foot placement for humanoid push recovery* (descrição operacional de capture point e capture region, incluindo LIPM+flywheel). https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7949430
- Joe, H.-M. & Oh, J.-H. — *Balance recovery through model predictive control based on capture point dynamics*, RAS 105, 2018. https://www.sciencedirect.com/science/article/abs/pii/S0921889017305493
- *A Model Predictive Capture Point Control Framework for Robust Humanoid Balancing via Ankle, Hip, and Stepping Strategies* — arXiv:2307.13243. https://arxiv.org/pdf/2307.13243
- *Push Recovery of a Humanoid Robot Based on Model Predictive Control and Capture Point* — arXiv:1612.08034. https://arxiv.org/pdf/1612.08034

### Dinâmica centroidal e momento angular
- Orin, D. E., Goswami, A. & Lee, S.-H. — *Centroidal dynamics of a humanoid robot*, Autonomous Robots 35(2), 2013. https://www.researchgate.net/publication/257523149_Centroidal_dynamics_of_a_humanoid_robot
- Orin, D. & Goswami, A. — *Centroidal momentum matrix of a humanoid robot: structure and properties*, IROS 2008. https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=4650772
- Wensing, P. M. & Orin, D. E. — *Improved computation of the humanoid centroidal dynamics and application for whole-body control*, IJHR 2016. https://www.cs.cmu.edu/~cga/z/Wensing_IJHR_2016.pdf
- Lee, S.-H. & Goswami, A. — *A momentum-based balance controller for humanoid robots on non-level and non-stationary ground*, Autonomous Robots 33(4), 2012. https://link.springer.com/article/10.1007/s10514-012-9294-z
- Lee, S.-H. & Goswami, A. — *A momentum-based humanoid balance controller for non-level and non-stationary ground*, IROS 2010. https://lava.kaist.ac.kr/wp-content/uploads/2017/06/Lee_Goswami_PosturalBalance_IROS2010.pdf
- Lee, S.-H. & Goswami, A. — *Reaction Mass Pendulum (RMP)*, ICRA 2007. http://www.ambarish.com/angular.html
- Macchietto, A., Zordan, V. & Shelton, C. R. — *Momentum control for balance*, ACM TOG 28(3), 2009.
- Herr, H. & Popovic, M. — *Angular momentum in human walking*, J. Exp. Biol. 211(4), 2008.
- *Learning Humanoid Arm Motion via Centroidal Momentum Regularized Multi-Agent RL* (papel do braço na regulação de CAM) — arXiv:2507.04140. https://arxiv.org/html/2507.04140
- *Balance Stabilization with Angular Momentum Damping*. http://crlab.cs.columbia.edu/humanoids_2018_proceedings/media/files/0027.pdf

### Estratégias humanas de equilíbrio
- Horak, F. B. & Nashner, L. M. — *Central programming of postural movements: adaptation to altered support-surface configurations*, J. Neurophysiol. 55, 1986.
- Runge, C. F., Shupert, C. L., Horak, F. B. & Zajac, F. E. — *Ankle and hip postural strategies defined by joint torques*, Gait & Posture 10(2), 1999. https://www.sciencedirect.com/science/article/abs/pii/S0966636299000326
- *Mechanical effort predicts the selection of ankle over hip strategies in nonstepping postural responses*, J. Neurophysiol. https://journals.physiology.org/doi/full/10.1152/jn.00127.2016
- *Integrating ankle and hip strategies for the stabilization of upright standing: an intermittent control model*, Frontiers Comput. Neurosci. https://www.frontiersin.org/journals/computational-neuroscience/articles/10.3389/fncom.2022.956932/full
- *Balance Recovery Prediction with Multiple Strategies for Standing Humans*, PLOS One. https://journals.plos.org/plosone/article?id=10.1371%2Fjournal.pone.0151166

### Viabilidade de contato
- Caron, S., Pham, Q.-C. & Nakamura, Y. — *Stability of surface contacts for humanoid robots: closed-form formulae of the Contact Wrench Cone for rectangular support areas*, ICRA 2015. https://arxiv.org/abs/1501.04719
- Implementação de referência das fórmulas. https://github.com/stephane-caron/analytical-wrench-cone
- Caron, S. et al. — *ZMP support areas for multi-contact mobility under frictional constraints* — arXiv:1510.03232. https://arxiv.org/pdf/1510.03232
- *Application of Wrench based Feasibility Analysis to the Online Trajectory Optimization of Legged Robots* — arXiv:1712.06833. https://arxiv.org/pdf/1712.06833
- *Balance and Walking Control for Biped Robot Based on DCM and Contact Force Optimization*, Mathematics 12(14), 2024. https://doi.org/10.3390/math12142188

### Controle PD e conversão para torque
- Tan, J., Liu, C. K. & Turk, G. — *Stable Proportional-Derivative Controllers*, IEEE CG&A 31(4), 2011. https://www.jie-tan.net/project/spd.pdf
- *Linear Time Stable PD Controllers*. https://www.cs.sfu.ca/~kkyin/papers/fastSPD.pdf
- Pratt, J., Chew, C., Torres, A. & Dilworth, P. — *Virtual Model Control: an intuitive approach for bipedal locomotion*, IJRR 20(2), 2001.

### Controladores em animação baseada em física
- Yin, K., Loken, K. & van de Panne, M. — *SIMBICON: Simple Biped Locomotion Control*, ACM TOG 26(3), 2007. https://www.cs.sfu.ca/~kkyin/papers/Yin_SIG07.pdf
- Coros, S., Beaudoin, P. & van de Panne, M. — *Generalized Biped Walking Control*, ACM TOG 29(4), 2010. https://www.cs.ubc.ca/~van/papers/2010-TOG-gbwc/paper.pdf
- Yin, K. — *Biped Controller for Character Animation* (capítulo comparando SIMBICON e GENBICON). https://www.cs.sfu.ca/~kkyin/papers/BipedController.pdf
- Geijtenbeek, T., Pronost, N., Egges, A. & Overmars, M. — *Interactive Character Animation using Simulated Physics: a state-of-the-art review*, CGF 31(8), 2012. http://graphics.cs.cmu.edu/nsp/course/15-869/2012/papers/PhysicsAnimation_EG11.pdf
- Geijtenbeek, T., Pronost, N. & van der Stappen, A. F. — *Simple Data-Driven Control for Simulated Bipeds*, SCA 2012. https://diglib.eg.org/server/api/core/bitstreams/b4f7ab08-6874-4c59-9f89-536616c61f74/content
- *Watch Your Step: Real-Time Adaptive Character Stepping* — arXiv:2210.14730. https://arxiv.org/pdf/2210.14730

### Controle neuromuscular e reflexos
- Geyer, H. & Herr, H. — *A muscle-reflex model that encodes principles of legged mechanics produces human walking dynamics and muscle activities*, IEEE TNSRE 18(3), 2010.
- Geijtenbeek, T., van de Panne, M. & van der Stappen, A. F. — *Flexible Muscle-Based Locomotion for Bipedal Creatures*, ACM TOG 32(6), 2013.
- SCONE — software aberto para simulação preditiva baseada em reflexos. https://scone.software
- *A simple network of proprioceptive reflexes can produce a variety of bipedal gaits*, bioRxiv 2025. https://www.biorxiv.org/content/10.1101/2025.02.05.636668v1.full
- *A Neuromuscular Model of Human Locomotion Combines Spinal Reflex Circuits with Voluntary Movements*, Sci. Rep. 12, 2022. https://www.biorxiv.org/content/10.1101/2021.09.26.461864v1.full

### Aprendizado
- Peng, X. B., Abbeel, P., Levine, S. & van de Panne, M. — *DeepMimic*, ACM TOG 37(4), 2018.
- Peng, X. B., Ma, Z., Abbeel, P., Levine, S. & Kanazawa, A. — *AMP: Adversarial Motion Priors for Stylized Physics-Based Character Control*, ACM TOG 40(4), 2021. https://xbpeng.github.io/projects/AMP/index.html
- Peng, X. B. et al. — *ASE: Large-Scale Reusable Adversarial Skill Embeddings*, ACM TOG 41(4), 2022.
- Chentanez, N. et al. — *Physics-based motion capture imitation with deep reinforcement learning*, MIG 2018. https://matthias-research.github.io/pages/publications/a1-chentanez.pdf

### Levantar-se
- Frezzato, A. et al. — *Synthesizing Get-Up Motions for Physics-based Characters*, CGF 2022. https://onlinelibrary.wiley.com/doi/abs/10.1111/cgf.14636
- *Learning Humanoid Standing-up Control across Diverse Postures* (HoST) — arXiv:2502.08378. https://arxiv.org/pdf/2502.08378
- *Learning Getting-Up Policies for Real-World Humanoid Robots* — arXiv:2502.12152. https://arxiv.org/pdf/2502.12152
- *FRASA: An End-to-End RL Agent for Fall Recovery and Stand Up* — arXiv:2410.08655. https://arxiv.org/pdf/2410.08655
- *HiFAR: Multi-Stage Curriculum Learning for High-Dynamics Humanoid Fall Recovery* — arXiv:2502.20061. https://arxiv.org/pdf/2502.20061

### Indústria
- Euphoria / Dynamic Motion Synthesis (NaturalMotion). https://en.wikipedia.org/wiki/Euphoria_(software)
- *The Wonders of Euphoria; Physics in Games* (incluindo Backbreaker). https://entropicdomain.net/formats/misc/pig-euphoria-engine/
- *Balancing of Active Ragdolls in Games* — Jan Schneider. https://medium.com/@jacasch/balancing-of-active-ragdolls-in-games-367f146b25fb

### PhysX 5
- *Articulations* — PhysX SDK Documentation (drives implícitos, cache, dinâmica inversa, CMM, Jacobiana, link incoming joint force, armature, tendões, estabilidade). https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/docs/Articulations.html
- `PxArticulationReducedCoordinate` — referência de API. https://nvidia-omniverse.github.io/PhysX/physx/5.3.0/_api_build/class_px_articulation_reduced_coordinate.html
- `PxArticulationCache` — referência de API. https://nvidia-omniverse.github.io/PhysX/physx/5.1.2/_build/physx/latest/class_px_articulation_cache.html
- CHANGELOG (depreciações de `computeGeneralizedGravityForce`, `computeGeneralizedMassMatrix`, `eJOINT_SOLVER_FORCES`). https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/CHANGELOG.md
- *Implicit Spring Joint Drives* — documento anexo do SDK sobre o modelo implícito de drive.

---

# 31. Resumo executivo

**O problema é um só:** o ragdoll tem base flutuante, então nenhum torque de junta move o centro de massa. Só gravidade e contato movem. Portanto **equilibrar-se é escolher onde e com que intensidade os pés empurram o chão** — e o resto do corpo existe para tornar essa força realizável.

**A arquitetura é uma cascata de seis camadas:** telemetria física → estimador de equilíbrio (capture point vs. polígono de suporte físico) → máquina de estados → referências (FootWork + postura corporal) → resolução centroidal com teste de admissibilidade → composição de torque → drives. O FootWork continua sendo conselheiro; quem decide é o controlador; quem executa é o PhysX.

**A composição de torque é uma soma, não uma escolha:**
`τ = gravidade + Coriolis + wrench via Jacobiana transposta + swing + postura + reflexos`, tudo saturado pelos limites anatômicos que o perfil já define. Nenhuma parcela é externa ao corpo.

**As quatro estratégias humanas caem naturalmente nessa estrutura:** tornozelo é a força virtual no CoM deslocando o CoP; quadril é o amortecimento de momento angular centroidal usando tronco e braços; passo é a correção de capture point entregue ao FootWork; queda é a degradação controlada dos drives com proteção de cabeça e tronco.

**O PhysX 5 já entrega mais do que parece:** compensação de gravidade e de Coriolis, matriz de massa, Jacobiana densa, **matriz de momento centroidal** (documentada como sendo para whole-body control de humanoides), aplicação direta de torque de junta por cache, medição da força transmitida por junta, e drives implícitos que já dão o comportamento do Stable PD. Você não precisa escrever uma biblioteca de dinâmica de corpos rígidos.

**Três correções de fundação vêm antes de tudo:** o collider dos pés precisa virar caixa — com cápsula, o polígono de suporte tem largura zero e a estratégia de tornozelo é matematicamente impossível; a flag `eDRIVE_LIMITS_ARE_FORCES` precisa ser verificada, sob pena de os torques serem ~120× menores do que o perfil declara; e o modelo de contato precisa nascer genérico, porque levantar-se usa mãos, joelhos e tronco.

**O teste isolado mais valioso do projeto** é a Etapa 2: com o ragdoll suspenso e ganhos de postura zerados, a compensação de gravidade sozinha deve segurar a pose. Se isso passa, a metade difícil está resolvida.

**Não comece por aprendizado por reforço.** Construa a cascata analítica, que é determinística, testável, barata para 22 personagens e — principalmente — compreensível. Se a camada de drives aceitar por DOF `(alvo, velocidade, escala de rigidez, damping, torque)`, uma política aprendida será, mais tarde, uma substituição plug-in das camadas do meio, com o baseline analítico servindo de referência e de fallback.
