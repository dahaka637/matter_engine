# Aprendizado por Reforço no Matter Engine

**Desde 13/10/2026 a metodologia é a das 3 áreas** (Corpo, Bola, Goleiro), com
os exercícios de cada área misturados no mesmo treino: ver "As 3 áreas e os
exercícios" abaixo. O que vem antes disso conta como as etapas da área 1.

O personagem aprende do zero, por etapas sob demanda. A primeira é a H1:
ficar em pé, estável, sem ajuda nenhuma. A H1b é ficar em pé na postura de
alerta (o `alert_idle`, o parado do jogo), e a H3 é andar seguindo um comando
de velocidade, até o trote da locomoção padrão (4,5 m/s). Todas no mesmo
cérebro. O andar pode aprender também imitando a locomoção física do jogo (o
aprendizado guiado). O plano completo e o diário ficam no
vault (`30_Projects/matter_engine/Aprendizado_por_Reforco/`); aqui fica o que
está implementado e como usar.

## As peças

| Peça | Onde | O que faz |
|---|---|---|
| Mundos de treino | `src/Engine/RL/Environment` | `TrainingBatch3D`: centenas de mundos Jolt em paralelo, sem janela; observação, recompensa por parcela, fim de episódio e recomeço no mesmo passo |
| Corpo de RL | `src/Engine/RL/Body`, `assets/rl/bodies` | o perfil do jogo mais os atuadores do RL (tetos de atleta, pose base do `natural_idle`, solver 10/6) |
| Tarefas | `assets/rl/tasks` | `h1_ficar_em_pe.json`: pesos da recompensa, término, início, variação do mundo, empurrões (desligados na H1) |
| Política no C++ | `src/Engine/RL/Policy` | `PolicyRuntime3D` lê o `.mrlpolicy` que o treinador exporta (a mesma conta do PyTorch, com teste de paridade) |
| Ligação com o treinador | `src/Engine/RL/Supervisor` | abrir o processo, socket local e o histórico (`rl.db`, só leitura) |
| Professor | `src/Engine/RL/Teacher` | a locomoção física do jogo conduzindo o corpo nos mundos de treino (o aprendizado guiado) |
| Ponte Python | `bindings/python` | `matter_rl_native` (nanobind): o lote como arrays numpy sem cópia; a única peça com Python |
| Treinador | `tools/rl` | PPO (PyTorch na GPU), checkpoints, SQLite, avaliação, canal de controle e a CLI `matter-rl` |
| Tela | `src/Workbench/ReinforcementLearning` | "Aprendizado por Reforço" no menu: controles, modos, avaliação, o visualizador ao vivo e os gráficos |
| Calibração do corpo | `src/Engine/RL/Calibration`, `MatterRLAudit`, `MatterRLDynamics`, `MatterRLJointProbe` | relatórios e regressões do corpo (massas, limites, queda) |

## Preparar (uma vez)

```bash
tools/rl/setup.sh
```

Ele faz o ambiente Python (uv, Python 3.14 fixo e o `torch` cu126, a variante
que roda na GTX 1070 Ti), compila a ponte em `build-rl/` (Release) e confere
tudo (`matter-rl doctor`).

## Usar

- **Pela tela:** "Aprendizado por Reforço" no menu.
  - "Iniciar treino" abre o treinador em segundo plano.
  - Pausar, Retomar, Parar, Salvar e Avaliar.
  - Modos Máximo, Equilibrado e Leve.
  - "Retomar ao ligar o PC".
- **Pelo terminal:** `tools/rl/matter-rl` com:
  - `iniciar`, `status`, `pausar`, `retomar`, `parar`, `salvar`, `avaliar`;
  - `modo maximo|equilibrado|leve`;
  - `tarefa <id>`: o treino aberto passa para outra habilidade (o mesmo
    cérebro, sem perder o que sabe);
  - `peso <parcela> <valor>`;
  - `relatorio --desde 1h`, `graficos`, `episodios`, `folha --cena <nome>`;
  - `comparar A B`: a bateria em dois checkpoints (rodada, `ultimo` ou
    `melhor`), lado a lado, fora do treino;
  - `folha --checkpoint X --mundo N [--de S --ate S]`: um mundo qualquer da
    bateria, num checkpoint, no trecho pedido;
  - `voltar --checkpoint N`: o treino (fechado) volta àquele checkpoint sem
    apagar nada (os mais novos vão para `protegidos/`, e a tarefa e as
    políticas voltam com ele);
  - `servico instalar|remover`.
- **A prévia de um exercício:** `tools/rl/previa.py` roda uma cópia de uma
  política numa tarefa e num exercício, mundo a mundo (de pé ou caiu, os alvos,
  os clipes, o erro de velocidade e o do peito), e grava os dumps das folhas
  de quadros. Nada se escreve no treino:

  ```bash
  cd tools/rl && .venv/bin/python previa.py --tarefa corpo --exercicio estrela --peito 1 \
      --politica ~/.local/share/matter-engine/rl/runs/principal/exports/policy_latest.mrlpolicy \
      --segundos 30 --saida /tmp/previa --mundos 0,4
  ../../build-profile/MatterModelInspector quadros --dump /tmp/previa/m4.dump --de 0 --ate 6 \
      --quadros 12 --vista 34 --seguir 1 --distancia 5 --colunas 6 --saida /tmp/previa/m4.png
  ```

## Onde ficam os treinos

`~/.local/share/matter-engine/rl/` (fora do Git):

- `rl.db`: o histórico;
- `runs/<treino>/checkpoints/`: o estado inteiro, gravado de forma atômica, com
  limpeza automática;
- `runs/<treino>/exports/`: `policy_latest.mrlpolicy` e `policy_best.mrlpolicy`;
- `runs/<treino>/episodes/`: episódios gravados nas avaliações, no formato do
  `MatterModelInspector quadros`;
- `runs/<treino>/tarefa.json`: a tarefa com os pesos de agora.

## Velocidade (12/10/2026, Ryzen 5 3600, 10 threads)

| Solver | Decisões/s (física) |
|---|---:|
| 10/6 (treino) | ~22.500 |
| 20/20 (jogo) | ~14.300 |

Medir: `MatterRLBenchmark --envs 256 --solver 10/6`.

O treino inteiro (física, rede na GTX 1070 Ti e PPO), com 512 mundos:

| Modo | Threads | Prioridade | Decisões/s |
|---|---:|---|---:|
| Máximo | 10 | nice 10 | ~17.500 |
| Equilibrado (padrão) | 8 | ociosa | ~15.500 |
| Leve | 4 | ociosa | menos |

## O pé do corpo (12/10/2026, noite)

O pé do corpo de RL é o do jogo: a casca convexa da chuteira, com a sola plana
no contorno dela (antes, uma caixa de 26,9 × 9,6 × 13 cm), e o boneco ficou
2,1 cm mais baixo com o pé nivelado. A sola real é menor que a caixa (o
calcanhar tem ~6 cm de largura): com a ação zero, o corpo cai em ~1,9 s, contra
~2,8 s com a caixa. As políticas treinadas antes seguem adaptando ("o corpo
mudou desde o checkpoint").

## A H1 do zero (validação de 12/10/2026)

- **Em ~1 min** ele já fica de pé no treino.
- **Em ~10 min** (11 milhões de decisões): a bateria `equilibrio_v1` dá 100%
  de pé por 60 s nos dois solvers. A oscilação do centro de massa fica em
  0,8 cm no 10/6 e 0,3 cm no 20/20.
- **Linha de base:** a ação zero (o PD segurando a pose base) fica 0% de pé e
  caía em 2,6–2,8 s com a caixa no pé (~1,9 s com a chuteira).
- **O tronco girando:** sem a parcela `trunkTurn`, 15 dos 32 mundos ficavam
  com a pelve rodando de um lado para o outro (até 46° de ponta a ponta), com
  os pés plantados. O centro de massa quase não se move, então a oscilação
  não via. Agora a recompensa cobra a rotação da pelve e do peito
  (−0,2·ω², quadrática para ensinar também com o ruído do treino), e a
  bateria mede o giro do tronco (`turn`), que entra na nota.
- **O pé que pivota** conta como escorregar (o centro do pé + 0,1 m × o giro
  em torno da vertical), e a bateria mede o giro dos pés no chão.
- **Empurrões:** a H1 não treina empurrões (é a H2). Ela aguenta 100 N por
  0,2 s no peito em todos os mundos, 150 N em metade e 200 N em nenhum. O
  empurrão padrão da tela é 100 N.

## Habilidades no mesmo cérebro

- **Uma habilidade é uma tarefa** (`assets/rl/tasks`). A tela lista as
  tarefas; escolher uma mostra a postura dela no mundo, e "Iniciar" treina a
  escolhida. Com um treino aberto, "Treinar esta habilidade" troca a dele.
- **Posturas:** o corpo tem uma biblioteca (`postures` no arquivo do corpo).
  A natural é a pose base, de onde a ação parte. A tarefa diz quais pede e a
  fatia de cada uma (a H1b: 70% alerta, 30% natural, para não esquecer a H1).
- **O comando** da postura entra no fim da observação. Uma política de antes
  dele lê o começo igual, e a migração entra com peso zero. Os canais de
  comando passam crus pela normalização.
- **Avaliação:** uma bateria por postura (`equilibrio_v1`,
  `postura_alerta_v1`). A nota é a da pior, e desconta o erro da postura
  pedida. Há um "melhor" por habilidade.

## As 3 áreas e os exercícios (13/10/2026)

Combinado com o usuário. O guia de estudo é o `Arquitetura_RL_MatterEngine.html`
(na área de trabalho dele), e o processo de trabalho está no Vault
(`Aprendizado_por_Reforco/Arquitetura_RL.md`).

- **Três áreas, três cérebros:**
  - **Corpo:** a locomoção ampla e os gestos sem bola. É o único que mexe nos
    músculos.
  - **Bola:** a cabeça que decide o lance (conduzir, chutar, passar, dominar,
    cabecear, disputar).
  - **Goleiro:** a cabeça própria, e o corpo é uma cópia do Corpo com os
    mergulhos.

  As cabeças mandam no Corpo (onde ir, para onde olhar, qual gesto, com que
  força) e treinam com ele congelado, então ensinar passe não mexe na corrida.
- **Dentro de cada área, exercícios misturados, não em fila.** Uma tarefa por
  área (a do Corpo é `assets/rl/tasks/corpo.json`). Cada mundo sorteia o seu
  exercício pelo peso a cada episódio, e toda atualização da rede vê todos os
  exercícios, então nenhum é esquecido.
- **O exercício muda o mundo, nunca a rede.** Ele pode mudar como os alvos
  nascem, as notas, os empurrões, a duração e o planejador. Não pode mudar o
  que a rede vê ou faz: o corpo, os clipes, o alvo ligado, o `lookahead`, o
  `facing` existir, as posturas. O carregador recusa se mudar.
  - **A rede nunca sabe o nome do exercício,** só os comandos (alvos, rumo
    do peito, velocidade). No jogo não há exercício, há comandos.
- **O exame geral:** cada exercício tem a sua bateria (`"drill"` na entrada
  da bateria), todas rodam a cada 10 min, e a nota é a da pior. Cada bateria
  tem de medir o objetivo do exercício. A lição da H6 v1: o peito virado
  não entrava na nota, e ele trapaceava.
- **As fatias seguem o exame** (`"drillWeights": {"auto": true}`). O
  exercício atrás do melhor ganha `gain` × o atraso (em pontos de nota) a mais
  de peso, até `maximumFactor` × o de base. O que alcança os outros volta ao
  peso de base (`baseWeight` na cópia do treino). Cada mudança vira um evento
  `exercicios` no banco.

O formato, na tarefa da área:

```json
"drills": [
  { "id": "estrela", "weight": 1.0, "overrides": { "target": {
      "pattern": "star", "starDirections": 8, "starJitterDegrees": 8.0,
      "facingFraction": 0.6, "facingDriftFraction": 0.4, "facingDriftRadians": 0.3 } } }
],
"drillWeights": { "auto": true, "gain": 10.0, "maximumFactor": 3.0 },
"evaluation": [
  { "name": "estrela_livre_v1", "drill": "estrela", "seconds": 40.0, "referenceSpeed": 3.0, "facing": 0 },
  { "name": "estrela_peito_v1", "drill": "estrela", "seconds": 40.0, "referenceSpeed": 3.0, "facing": 1 }
]
```

- `overrides` é um merge patch do JSON sobre a tarefa base: objetos se
  misturam; listas e valores se trocam.
- Na avaliação, `"drill"` fixa o exercício de todos os mundos e `"facing"`
  liga (1) ou desliga (0) o comando de olhar. Sem `"drill"`, a avaliação usa o
  primeiro exercício.
- O episódio grava o exercício na coluna 26 da linha (`drill`), e a vista de
  um mundo também. A 25 é o erro do peito, a 27 o de velocidade e a 28 a sola
  escorregando (29 colunas).

**Exercício novo, passo a passo:**
1. A ideia com o usuário: o que ensina e como medir.
2. Os clipes na biblioteca da área, se faltarem. A biblioteca é a mesma para
   todos: clipe novo é canal novo, e a migração entra com peso zero.
3. O exercício em `drills`, com a bateria dele (`"drill"`).
4. Um teste em C++ do que ele gera (como `testShuttle`).
5. A prévia: a política do treino nele, numa cópia, com as folhas de quadros
   (`tools/rl/previa.py`), antes de entrar na mistura.
6. Entrar na mistura com um peso pequeno, e o exame mostra se algum antigo
   caiu.

## A estrela e o comando de velocidade (14/10/2026)

O primeiro exercício do Corpo (o 3 do `treino_locomocao_claude.md`, na pasta
de treinos do usuário). Ensina a sair do centro para as 8 direções em quatro
velocidades, com o peito livre (ele vira) ou fixo (vai de costas, de lado, na
diagonal), e a voltar.

- **O desenho** (`"pattern": "star"`): os alvos pares vão do centro a uma das
  8 direções (`starDirections`), medidas do rumo do começo (o "norte"), com
  ±8° de sorteio no treino (`starJitterDegrees`). Os ímpares voltam ao centro.
- **O comando de velocidade** (`target.speedCommand`): cada ida sorteia uma
  faixa, e dela a velocidade e a distância.

  | Faixa | m/s | m | Peso |
  |---|---|---|---|
  | andar | 0,8–1,6 | 6–9 | 1 |
  | trote | 2,0–3,2 | 8–12 | 1,5 |
  | corrida | 4,0–5,0 | 12–17 | 1 |
  | sprint | 5,5–6,5 | 16–22 | 0,7 |

  - **A volta ao centro** repete a velocidade da ida. No treino, em metade
    das vezes sorteia outra (`redrawOnReturn`).
  - **A pedida de agora freia antes do ponto:**
    v = min(v do trecho, √(v_chegada² + 2·a·(d − raio))), com a =
    `brakeAcceleration` (4 m/s²) e v_chegada = max(`minimumArrivalSpeed`,
    min(v, v do próximo)·cos(virada/2)). Numa volta de 180° ele chega a 1 m/s.
  - **A rede vê** `command.speed` (a do trecho), `command.speed.now` (a de
    agora) e `command.speed.next0/1` (a dos próximos), divididas por 6.
  - **A nota `speedMatch`:** um sino na velocidade em direção ao alvo menos a
    pedida de agora (sigma 0,6 m/s). A chegada (`targetReached`) também mede
    pela pedida.
  - **Com o peito fixo, a pedida não passa do que os clipes fazem naquela
    direção em relação ao peito** (`capByDirection`: o mais rápido a até 25°,
    senão a até 50°). De lado vale o sprint de lado (4,1 m/s).
- **Os três modos no mesmo exercício:** livre em 40% dos episódios, fixo em
  30% e girando em 30% (`facingFraction` 0,6, e metade desses gira:
  `facingDriftFraction` 0,5). Girando, o rumo do peito gira em trechos: cada
  um sorteia de meia a uma volta e meia (`facingDriftTurns`), a velocidade
  (0,4–1,2 rad/s, `facingDriftSpeed`) e o sentido; o sentido pode repetir, e
  aí ele dá mais de uma volta seguida (como o jogador girando a câmera).
  Na avaliação, `"facing": 2` pede o girando.
  - **Peito fixo:** o planejador mede a direção dos clipes pelo peito
    (`directionWeightFacing` 2): andar de costas, de lado, as diagonais de trás.
  - **Peito livre:** até 4 m (`freeBackwardDistance`) valem todos os clipes.
    Mais longe, só os de frente, e ele vira.
  - **Enquanto vira,** a velocidade da escolha cai com o cosseno do ângulo
    até a mira (no mínimo a de chegada): vira andando e acelera ao alinhar.
    Antes, o sprint de costas para o alvo levava o corpo para longe dele, e o
    alvo se perdia em 1,7 s.
- **As baterias:** `estrela_livre_v1` (peito livre), `estrela_peito_v1`
  (fixo) e `estrela_girando_v1` (girando), 40 s cada.
  - O mundo i vai na direção (3i + 5·ida) mod 8 e na faixa (i + ida) mod 4,
    então 16 mundos passam por todas.
  - A nota da corrida desconta o erro de velocidade (`speed_error`) e o do
    peito.
  - A sola escorregando (`foot_slip`: a velocidade média do pé no chão, m/s)
    é só medida, fora da nota, até mostrar que precisa entrar.
- **Na tela:** Corpo → Estrela → Peito livre/fixo/girando (ramos na lista de habilidades).
  - O rótulo mostra a velocidade de agora e a pedida ("1,0 m/s · pede 1,2").
  - As 8 pontas da estrela aparecem em cinza, no tamanho da ida de agora.
  - A câmera fica em 3ª pessoa presa ao mapa: atrás dele (um pouco de lado)
    no rumo até o alvo, não no do corpo, a 7 m e 18° de cima. O mouse gira e
    aproxima por cima disso, e o duplo clique volta ao padrão. Um teste da
    tela confere que, em 40 s de estrela, ele fica sempre no meio do quadro.
  - Cada recomeço (uma queda) segue o roteiro do mundo seguinte da bateria
    (`evaluationCycle`; `cycle=True` no `Batch` do Python), então o mundo da
    tela passa por todas as direções e faixas. Sem isso, ele recomeçava
    sempre no andar para a frente.
- **Os clipes novos da biblioteca (19 no total):**
  - andar de costas (`walk_backward`);
  - andar de lado (`walk_strafe_right`, e o espelho `walk_strafe_left` feito
    por `tools/animation/espelhar.py`);
  - a corrida média (`run_forward_medium`, 4,3 m/s).

  O andar de frente é o `ual_walk`.
- **A política grava os nomes das entradas** (`observationNames`, desde
  14/10). A tela e a prévia montam a entrada pelos nomes, e as que a tarefa
  não tem entram com zero, como na migração do treinador. Assim um cérebro
  antigo roda numa tarefa mais nova.

## Não cair: o peito girando + força do Frear e parar (14/10/2026)

O exercício 9 não é um exercício separado: por pedido do usuário, as forças
ficam no modo peito girando do Frear e parar ("Peito girando + força").

- **As forças** (`pushes` do exercício, `whenRotating`: só nos episódios de
  peito girando), sem aviso, a cada 2–4 s, sorteadas entre:
  - **o tranco:** 100–800 N por 0,1–0,25 s, na pelve ou no peito, de qualquer
    lado;
  - **a força contínua** (`sustainedFraction` 0,3): 100–300 N por 1–3 s;
  - **o tropeço** (`tripFraction` 0,2): um puxão de 150–400 N por 0,08–0,15 s
    no pé que está no ar, contra o rumo do corpo (sem pé no ar, um tranco).
- **Se recobrar ao quase cair** (`termination.recoverySeconds` 3,
  `fallGraceSeconds` 0,75): por 3 s depois do começo de uma força, a mão no
  chão não encerra, e a cabeça ou a pelve baixas só são queda se ficarem
  baixas por 0,75 s.
- **A prova:** `frear_forca_v1` (peito 2). O tempo até voltar, em pé e na
  pedida, depois de cada força (`push_recovery`, a coluna 40, 41 no total), só
  medido.
- **Na tela:** o ramo "Peito girando + força"; a seta da força onde ela entra.
- **A prévia com o cérebro de 14/10 (rodada 60068):** de pé em 4 de 16 em
  40 s (sem forças, 16 de 16); a volta depois de uma força, 2,2 s.

## Pulos (14/10/2026)

O exercício 8, como a seção 8 do `treino_locomocao_claude.md` pede. A primeira
versão do mesmo dia (alvos no ar no lugar dos destinos) estava errada e foi
revertida: com a freada antes de cada volta, todo pulo virava um pulo parado.

- **O mundo:** a estrela de 9 pontos, o peito livre ou fixo, a velocidade só
  de trote ou corrida (as faixas do exercício).
- **O pulo correndo:** na ida do centro à ponta, a zona de pulo no meio do
  caminho (uma faixa de 1 m ao longo e 1,6 m de largura), uma barreira
  imaginária na altura pedida. Ele passa por ela com a pelve acima da altura
  e segue até a ponta.
- **O pulo parado:** na volta, o centro é a zona (o disco de parar). Ele fica
  0,3 s quieto no disco e pula na direção pedida: para cima (sem sair mais de
  25 cm do lugar) ou uma das 8 em relação ao corpo (pelo menos 25 cm para lá).
  Sem pular em 5 s desde a entrada no disco, perdido, e o seguinte assume.
- **A altura:** a subida da pelve sobre a de pé no alerta
  (`standingPelvisHeight`): parado 15–45 cm (`standJumpRise`), correndo
  20–50 cm (`runJumpRise`). Na avaliação, três alturas fixas (1/6, 1/2 e 5/6
  da faixa).
- **O comando de pulo** (`target.jumps`, na observação de todos os exercícios,
  ligado só nos pulos): `jump.active`, `jump.standing`, a direção e a distância
  até a zona, `jump.rise` e a direção pedida no parado.
- **A nota** `jumpZone` (1,0 nos pulos): só a zona vencida paga, máxima até
  10 cm acima da pedida e caindo depois. Pular fora da zona não rende nada.
- **A prova** (`pulos_livre_v1`, `pulos_peito_v1`; colunas 35–39, 40 no total,
  só medidas): as zonas vencidas correndo (`jump_run`) e paradas
  (`jump_stand`), a subida menos a pedida, as quedas na aterrissagem (no ar ou
  até 1,5 s depois) e a volta a 80% da velocidade depois do pulo correndo.
- **Na tela:** o disco azul da zona e a barreira (dois postes e a barra na
  altura pedida); no pulo parado, a seta da direção pedida.
- **Fora, de propósito:** os clipes de pulo (a biblioteca só aceita ciclos; a
  muleta se decide na calibração), a altura subindo com o currículo
  (calibração) e a barreira física (mais tarde, como no plano).
- **A prévia com o cérebro de 14/10 (rodada 60068):** 16 de 16 de pé, todos
  passam pelas duas zonas, 0% vencidas (ele não pula; ~30 cm abaixo da pedida).

## O tremor parado e a estrela contínua (14/10/2026)

**O tremor parado.** O usuário viu o cérebro tremer os pés no girar parado. A
causa (ele lembrou): a estabilização foi treinada no começo (H1 e H1b, com as
notas de ficar de pé) e depois nunca foi revisada; na Corpo essas notas
estavam em zero, inclusive nos parados.

- **Os parados revisam a estabilização do começo:** `parado` e `empurroes`
  com as notas de ficar de pé do H1b (quieto, centro de massa sobre os pés,
  pé sem sair do lugar nem escorregar, tronco sem girar, esforço, mudança de
  ação). O `girar` sem "tronco sem girar" e "pé sem sair do lugar" (o giro
  precisa das duas).
- **Sem limites artificiais:** um filtro de "músculo" (um atraso na ação) e
  duas penalidades novas foram feitos e desfeitos no mesmo dia. Um atraso não
  é uma limitação humana: um humano reage rápido.
- **Só medido, na prova:** a cadência (pisadas por segundo de cada pé,
  coluna 31), os passinhos (pé menos de 0,1 s no ar, coluna 32) e a reversão
  (a mudança da ação contra a anterior, o índice de tremor, coluna 33).
  `human_penalty` desconta os passinhos e a cadência alta da nota.
- **Os giros no lugar em ciclo** (`turnCycleDegrees`, os giros do Mixamo na
  biblioteca) ficaram, como muleta do girar parado: a direção combinada com o
  usuário é tirar os clipes aos poucos depois (o corpo fazendo os próprios
  movimentos antes da bola).

**A estrela contínua (o exercício 6, cortes, virou isto).**

- `starContinuousFraction` (0,5 na estrela): depois de uma ponta, metade das
  vezes ele vai direto a outra ponta (outra direção) em vez de voltar ao
  centro. Os cortes de todos os ângulos aparecem sem a parada da volta.
- `hideNextFraction` (0,3): em parte dos trechos os próximos ficam escondidos
  até a chegada (a rede, a mira, o olhar livre e a tela só veem o de agora;
  a pedida não freia para uma virada que ele não conhece). Na avaliação, um
  trecho em três.
- **O tempo de cada corte** (`cut_seconds`, coluna 34, 35 colunas): da
  chegada num alvo com virada de 45° ou mais até andar a 80% da pedida rumo
  ao seguinte. Só medido. Com o cérebro de 14/10: 2,3 s em média.

## Parado, girar parado e empurrões (14/10/2026)

Os exercícios 1 e 2 do `treino_locomocao_claude.md`, no Corpo. Os três usam
o desenho `"hold"`: o alvo é o lugar onde ele começa, de "pare aqui" para
sempre, com a pedida zero (o planejador dá o clipe `parado`). O peito tem o
comando sempre (`facingFraction` 1).

- **Parado** (`parado`, peso 0,5): o peito fixo no rumo do começo. Bateria
  `parado_v1` (de pé: o desvio, o pé saindo do lugar, o tronco girando, a
  postura).
- **Girar parado** (`girar`): o rumo do peito gira em trechos (meia a uma
  volta e meia, 0,4–1,2 rad/s, o sentido sorteado) e ele gira junto, no
  lugar. A referência é o parado girando junto (os giros do Mixamo não são
  ciclos e a biblioteca só aceita ciclos). Bateria `giro_parado_v1`, do tipo
  novo `giro` (`turn_score`: de pé, menos até 0,5 pelo peito fora do rumo
  e até 0,15 pela sola escorregando).
- **Empurrões** (`empurroes`): parado, com `pushes` ligado só nele (80–250 N
  por 0,1–0,25 s a cada 1,5–3,5 s, na pelve ou no peito, de qualquer lado; na
  avaliação também). Bateria `empurroes_v1` (de pé).
- **Na tela,** os três são ramos do Corpo sem os modos do peito (cada um tem
  um só).
- **A prévia com o cérebro de 14/10 (rodada 60068):** de pé em 16/16
  (parado), 14/16 (girar, o peito 21° fora) e 15/16 (empurrões). A sola
  escorrega muito nos três (0,7–0,9 m/s): ele nunca treinou ficar parado.

## O peito livre olha para onde vai (14/10/2026)

Com o peito livre, nada cobrava para onde o peito olhava, e o cérebro ia de
lado (41–45° fora do caminho na estrela, no frear e nas curvas, rodada 60068).
O usuário viu no laboratório.

- **`target.freeLook`:** sem o comando de olhar, o peito é medido contra a
  direção do próximo alvo, puxada `freeLookNext` (0,5) para o seguinte. A
  menos de 1,5 raio do alvo, vale o seguinte.
- **A mesma nota e o mesmo erro do modo fixo:** a nota `facingHold` e o erro
  do peito do episódio. A bateria do peito livre passa a descontar o peito
  fora do caminho.
- **De costas ou de lado com o peito livre,** só no último metro
  (`freeBackwardDistance` 1).

## Frear e parar (14/10/2026)

O segundo exercício do Corpo (o 4 do `treino_locomocao_claude.md`): a estrela,
com duas coisas a mais.

- **A pedida muda no caminho** (`speedCommand.changesPerLeg` 2): em cada
  trecho, de 0 a 2 vezes, em pontos sorteados do caminho (entre 85% e 25% do
  que falta), a velocidade pedida é sorteada de novo entre as faixas. Ele
  arranca e freia sem esperar o disco.
- **Metade dos discos é de "pare aqui"** (`stopFraction` 0,5; laranja na
  tela).
  - A chegada pedida é zero: a freada leva a pedida a zero na borda do disco.
  - Ele tem de ficar dentro do disco, mais devagar que `stopSpeed`
    (0,35 m/s), por `stopHoldSeconds` (0,8–1,5 s). Só então o alvo conta, e
    ele arranca dali para o próximo.
  - O planejador usa o clipe parado (`alert_idle`, o `parado` da biblioteca)
    quando a pedida cai abaixo de `idleBelow` (0,5 m/s).
- **Na avaliação,** os discos alternam parar e passar, cada mundo começando
  de um lado, e as mudanças no caminho vão de 0 a 2 pelo mundo e pelo trecho.
- **As baterias:** `frear_livre_v1` e `frear_peito_v1`. O quanto ele passa do
  disco antes de parar (`stop_overshoot`, a coluna 29 do episódio, 30 colunas)
  é só medido, fora da nota.
- **Na tela:** os exercícios são ramos do Corpo na lista de habilidades, e os
  modos do peito, ramos de cada exercício (Corpo → Estrela / Frear e parar →
  Peito livre / fixo / girando). O nome vem de `"name"` no exercício.

## Curvas (14/10/2026)

O terceiro exercício do Corpo (o 5 do `treino_locomocao_claude.md`).

- **O desenho** (`"pattern": "arc"`): alvos pequenos (raio 0,6 m) a cada
  `arcSpacing` (2 m) ao longo de um arco, os dois próximos no mapa.
  - Cada curva sorteia o tipo de raio (`arcRadii`: aberta 13–17, média
    6,5–9,5, fechada 2,5–3,5 m) e o quanto gira (`arcSweepDegrees`, 60–180°).
  - O sentido troca em metade das curvas (`arcSwitchFraction`): as curvas em S.
  - A velocidade é uma por curva, da faixa sorteada, no máximo
    √(`arcLateralAcceleration` × raio) (5 m/s²: na fechada, ~3,9 m/s).
- **Os clipes:** os de ciclo (trotes, sprint, andar) com a referência girando
  pelo rumo do alvo. Os clipes de curva do Mixamo (não são ciclos) ficam para
  depois, se a imagem pedir.
- **O afastamento da linha** (`line_error`, a coluna 30 do episódio, 31
  colunas): a distância do centro de massa ao trecho entre o alvo anterior e o
  de agora, em RMS. Desconta até 0,20 da nota da corrida (1 m ou mais); nas
  outras baterias é -1 e não conta (`SCORE_VERSION` 4).
- **As baterias:** `curvas_livre_v1` e `curvas_peito_v1`. Na avaliação, o
  tipo de raio e a troca de lado seguem o mundo e a curva.
- **A prévia com o cérebro de 14/10 (rodada 60068, antes de treinar):** em
  30 s, de pé em 15 de 16 (livre) e 14 de 16 (fixo), ~19 alvos, 0,6–0,7 m fora
  da linha, 2–2,8 m/s abaixo da pedida (devagar nas curvas).

## Da H4 à H7 (13/10/2026): a biblioteca de clipes e o planejador

- **A biblioteca** (`imitation.library`): todos os clipes em ciclo, e o
  planejador do ambiente escolhe qual seguir a cada decisão. A troca entra no
  quadro mais parecido (a tabela de emendas). A observação tem um canal por
  clipe e o giro pedido.
- **O planejador escolhe o clipe:**
  - pela direção da mira (a partir do corpo, ou do comando de olhar), com
    histerese;
  - com `speedByDistance` e `speedWeight`, também pela velocidade pedida pela
    distância (longe, o sprint; perto, o trote);
  - `directionWeight` < 1: longe, vale virar e correr de frente em vez de ir
    de costas.
- **Vários pontos** (`target.lookahead`): os próximos alvos ficam no mapa e na
  observação.
  - Na avaliação, os percursos fixos de `evaluationCourses`, um por mundo.
  - Na tela e nas folhas: o alvo cheio, os próximos mais fracos e ligados, os
    alcançados verdes.
  - `nextAngleOffsetDegrees` sorteia o próximo em volta de uma volta (o tiro
    com volta).
- **O traçado de corrida** (`target.racingLine`): o planejador e o giro miram
  o ponto do disco do lado do próximo alvo. O ponto fica dentro do disco, e ele
  entra já virado.
- **O comando de olhar** (`target.facing`):
  - `facingLink` `chest` segura o peito, e o quadril fica livre;
  - `facingFraction` é a fatia dos episódios com o comando (canal
    `command.facing.active`);
  - `facingHold` é a nota direta de segurar o rumo;
  - a bateria mede o erro do peito (`facing_error`, e a nota desconta até 0,25).
- **As tarefas:**

  | Tarefa | O que é |
  |---|---|
  | `h4_direcoes` | 8 trotes, livre |
  | `h5_pontos` | vários pontos e o traçado de corrida |
  | `h6_peito` | peito para a câmera |
  | `h7_tiro` | tiro com volta, com o sprint |

  Hoje viram os exercícios do `corpo`.

## Correr até o alvo (H3 v7+, imitação)

Desde 13/10 a H3 é uma corrida até um alvo a 10 m, imitando clipes de
captura (o DeepMimic). O andar só com recompensas (v1–v6, abaixo) não deu um
passo de gente: em cada versão ele achou um atalho.

- **A referência** (`src/Engine/RL/Imitation/ReferenceMotion3D`): os trechos
  de `imitation.clips` em sequência, o último em ciclo (hoje o parado→sprint
  e o sprint do Mixamo).
  - Amostrada no corpo do RL a 120 Hz: juntas, velocidades, a altura, a
    velocidade e a inclinação da pelve, e cabeça, mãos e pés no referencial
    do rumo.
  - O vazamento da inclinação do chão do Mixamo (~5°) sai da altura da
    pelve, e o pé de apoio fica no chão em cada quadro.
  - Na troca de trecho, o ciclo entra na fase mais parecida.
- **A ação corrige a pose do clipe** (`imitation.residualActions`): o alvo de
  cada junta é o da referência no fim da decisão (com a velocidade dela)
  mais `actionScale` vezes a ação. Ação zero segue o clipe com os motores de
  sempre; nada empurra o corpo.
- **A recompensa:**
  - os sinos do DeepMimic contra a referência: pose, velocidade das juntas,
    partes-chave e raiz (`imitation.scales`);
  - a velocidade em direção ao alvo, linear: mais rápido paga mais;
  - a chegada, paga pela velocidade média do trecho;
  - ao chegar, nasce o próximo alvo 10 m adiante.

  O episódio tem duração fixa: quem corre mais chega a mais alvos.
- **O começo** (o RSI do DeepMimic): metade dos episódios sai do parado, a
  outra metade numa fase sorteada do sprint, já com a pose e as velocidades
  do clipe.
- **A observação cresce no fim:** o trecho, a fase, as juntas da referência e
  a direção e a distância do alvo. Os pesos antigos ficam.
- **A bateria `corrida_alvo_v1`:** sai do parado, com os alvos em linha reta.
  Mede sem cair, chegou, alvos, o tempo até o 1º, a velocidade média e o erro
  da pose (RMS, rad).
- **Folhas:**
  - `matter-rl folha --checkpoint ultimo --bateria corrida_alvo_v1`;
  - `matter-rl folha --referencia` (o clipe no corpo do RL, nos mesmos
    instantes);
  - o alvo sai no dump (linha `A`) e na folha como o disco vermelho.
- **Na tela:** "Correr até o alvo" mostra o alvo (o disco vermelho com o
  brilho subindo), o chão de 30 m e a câmera de lado, com o jogador e o alvo
  no quadro.
- **Versão 8:** na 7, numa validação, ele voltou a ficar parado. Longe do
  clipe a imitação valia ~0, o castigo de mudar a ação era maior, e com o
  total nunca negativo tudo virava 0. Por isso a 8 tem a ação corrigindo o
  clipe, os sinos mais largos e o total livre.

## Andar por comando (H3 v1–v6)

Guardado em `tests/RL/data/andar_comando.json` para os testes (e para a
etapa de velocidade comandada, no futuro).


- **O comando** (frente, lado, giro, no referencial do rumo) entra no fim da
  observação, junto com o relógio de passada (seno e cosseno da fase). Na
  tarefa `h3_andar`, agora só a frente está ligada (até 4,5 m/s).
- **O currículo** começa em 1,0 m/s e sobe 0,25 quando ele acompanha bem; ele
  aparece no TREINO da tela. Os limites ficam em `commands.limits` na cópia da
  tarefa do treino.
- **Costas, lados e giro já estão prontos.** Para ligar, na tarefa:
  1. um máximo acima de 0 em `commands.backward`, `lateral` e `yawRate`
     (sugestão em `commands.prontos`: 3, 3 e 2);
  2. `"enabled": true` nas baterias `andar_tras_v1`, `andar_lado_v1` e
     `girar_v1`;
  3. reabrir o treino (`matter-rl tarefa h3_andar`).
- **A recompensa** é a dos humanoides do Isaac Lab:
  - seguir a velocidade e o giro;
  - o pé no ar em apoio simples;
  - a sola sem escorregar;
  - o tronco sem balançar de lado;
  - o tronco e os braços perto do alerta.

  As parcelas de parado só valem com comando 0.
- **Ficar parado não paga** (versões 2 a 6 da tarefa):
  - com pedido de andar, o "estar bem" (vivo, postura, cabeça, pose) vale
    multiplicado pelo acompanhamento;
  - o acompanhamento é medido contra ficar parado (`relativeToStanding`):
    parado vale 0 em qualquer velocidade pedida (no sino puro, parado com
    0,3 m/s pedidos levava 70%);
  - o total de cada passo, menos a queda, nunca fica negativo
    (`reward.positiveOnly`, o `only_positive_rewards` do legged_gym): tentar
    andar e errar nunca vale menos que ficar parado;
  - os episódios com pedido de andar começam com o corpo já na velocidade
    pedida (`startMoving`).

  Com a versão 4, numa validação, ele passou a tirar os pés do chão e o
  currículo subiu de 1,0 para 1,25 m/s em ~25 min.
- **Versão 6** (pedido do usuário: "ficar parado não deve recompensar quando o
  treino é andar"):
  - a tarefa só pede andar: nada de trechos parados, e um pedido pequeno
    demais (abaixo de 0,3 m/s) é sorteado de novo;
  - parado sob pedido de andar vale exatamente 0: o relógio de passada só
    paga o pé de balanço no ar (no chão, desconta), e o "não girar" só vale
    andando;
  - a queda custa 5 (tentar e cair é barato);
  - metade dos episódios sai do parado e metade já em movimento;
  - as baterias de ficar em pé são medidas, mas não entram na nota do andar
    (`"scored": false`).
- **O relógio de passada** (`gait`): cada pé no chão ou no ar na hora certa
  da fase e o pé de balanço subindo 8 cm. Há também uma referência de
  passada opcional (`gait.reference`, a do humanoid-gym), desligada.
- **Baterias:**
  - `andar_frente_v1` (parado, 1,5, 3, 4,5 m/s, parar);
  - `ir_ate_10m_v1` (o comando calculado de um ponto a 10 m);
  - as de pé.
- **Na tela** (com uma tarefa de comando): o visualizador faz sozinho um ciclo
  de demonstração (parado, metade do limite, o limite, parado), e o chão corre
  sob os pés com a câmera seguindo.

## Aprendizado guiado (o professor)

O RL aprende imitando a locomoção física do jogo e depois segue pelo treino
normal com o que extraiu. **Desligado por ora** (`teacher.enabled: false` na
`h3_andar`, decisão do usuário em 12/10): o "0%" de ajuda do jogo ainda
sustenta metade do peso andando, e o aluno imitava um corpo mais leve. O
código fica; para religar, `"enabled": true` na seção `teacher` da tarefa.

- **O professor** (`src/Engine/RL/Teacher`, `LocomotionTeacher3D`):
  - o mesmo runtime dos atletas do laboratório (`AthleteRuntime3D`), no modo
    físico total, com as ajudas parado, andando e nos pés em 0%;
  - conduz o corpo de cada mundo de treino, um boneco por cena (ele precisa
    da cápsula da cena), com a coleira e o corte da locomoção física v2;
  - segue um roteiro sorteado: parado, girando no lugar, andando para
    frente, para trás, de lado e na diagonal (às vezes em curva), com
    empurrões no tronco;
  - o roteiro fica na seção `teacher` da tarefa.
- **O que é "sem força auxiliar":** no 0% do jogo, andando, a pelve ainda
  leva metade do peso (a sustentação de base da locomoção física v2, ver o
  vault). Com as pernas levando tudo (`T_WeightWalk=1`), o professor anda
  mais devagar e cai de lado e na inversão. Ele é o modelo da passada;
  carregar o peso inteiro o aluno aprende no treino normal.
- **O rótulo:** para cada junta, o alvo do RL que, com a rigidez e o
  amortecimento do RL, daria o torque que o professor aplicou (motor mais
  antecipação), em média nos passos de física da decisão. O comando é o que
  ele tenta (a cápsula, colada ao corpo), e o relógio de passada é o dele
  (o pé direito no ar de 0 a 0,5, o esquerdo de 0,5 a 1).
- **A fase** (`tools/rl/matter_rl/imitation.py`):
  1. os mundos do professor (128) andam pelo roteiro, e as decisões dele vão
     para uma memória na GPU (sem o meio segundo antes de cada queda);
  2. a média da política imita a ação dele (erro quadrático), com 20% de
     cada lote vindo do parado da própria política (para a H1 e a H1b
     ficarem);
  3. no fim: o desvio da ação vai a 0,45, o otimizador recomeça, 20 rodadas
     só do crítico (o ator parado) e um termo de imitação no PPO que some em
     300 rodadas.
- **Usar:**
  - na tela, com o andar em treino: "Aprender com a locomoção física"
    (HABILIDADES);
  - no terminal: `matter-rl professor [--minutos N]` (padrão: 15);
  - para ver o professor: "Professor" no seletor acima do mundo, ou
    `matter-rl folha --professor --de 0 --ate 10`.
- **Medido** (12/10/2026): o professor roda a ~5.400 decisões/s com 8
  threads, anda a 0,65 m/s em média no roteiro e cai em ~7% dos episódios.
  O erro da imitação cai de 0,26 para 0,07 em 2 min.

## Semanas ligado

- **Desligar o PC** (SIGTERM): o treino grava o checkpoint e fecha em ~2 s.
  Com "Retomar ao ligar o PC", ele volta sozinho da mesma rodada. "Parar"
  pela tela não volta.
- **O processo morreu** (`kill -9`, falta de luz): o supervisor reabre em
  ~12 s, a partir do último checkpoint (no máximo 10 min perdidos). A sessão
  morta fica "interrompida" e as rodadas perdidas saem do histórico.
- **Trocar de modo** reabre o treino em ~4 s, sem perder rodadas.
- **NaN na rede:** volta ao último checkpoint bom, com a taxa de aprendizado
  pela metade.
- **CPU acima de 90 °C por 1 min:** cai um modo.
- **Disco:**
  - checkpoints de ~6 MB: os 3 últimos, o melhor, 1 por hora nas últimas
    24 h e 1 por dia depois;
  - as 40 gravações de episódio mais recentes (~1 MB cada);
  - rodadas detalhadas por 3 dias (~100 MB por dia), depois uma a cada
    10 min;
  - teto de 10 GB, com aviso.
- **Memória:** o treinador fica em ~1,9 GB e o supervisor em ~22 MB (ele não
  carrega o PyTorch).

## Testes

- **C++:**
  - `MatterEngine.RLTraining`: corpo, cinemática contra o Jolt, contrato,
    invariância ao rumo da observação, determinismo, quedas, limite de tempo;
  - `MatterEngine.ReinforcementLearningScreen`: a tela sem janela.
    `MATTER_RL_TEST_POLICY=<.mrlpolicy>` roda uma política de verdade e salva
    a foto de 30 s;
  - as calibrações (`RLCalibration`, `RLJointLimits`, `RLDynamics`).
- **Python:** `cd tools/rl && .venv/bin/python -m pytest`:
  - PPO (GAE, aprende uma tarefa simples);
  - rede (normalização, crescer sem esquecer, reescala da ação);
  - checkpoints atômicos e a limpeza;
  - trava, banco, canal de controle;
  - a retomada depois de uma queda e a amostra das curvas por tempo;
  - o treinador inteiro: continuar do checkpoint, a recuperação de NaN e a
    troca de habilidade (H1 → H1b);
  - as posturas pela ponte e uma bateria por postura;
  - o aprendizado guiado: a memória do professor, a janela de queda, a fase
    no treinador, as rodadas só do crítico e o termo de imitação sumindo;
  - ponte nativa e paridade com o C++ (< 1e-5).
- **O professor (C++):**
  - `MatterEngine.RLTraining`: o rótulo de um professor que manda nas
    juntas como o RL devolve a ação dele;
  - `MatterEngine.RLTeacher`: a locomoção física nos mundos de treino anda
    sem cair, o rótulo é finito e a fase da passada corre.
