# Aprendizado por Reforço no Matter Engine

O Corpo (o jogador com 37 eixos atuados, `assets/rl/bodies/football_player_v2.json`)
aprende a se mover sozinho: PPO com o juiz de estilo (AMP) como peso que o
usuário abaixa até zero. O contrato 1 (a imitação residual, o professor, as
tarefas H1 a H7 e o Corpo v15) foi removido em 15/10/2026: só o Corpo v2
existe. O plano, a escada de recuperação e o diário ficam no vault
(`30_Projects/matter_engine/Aprendizado_por_Reforco/` e `diario_atualizado/`);
aqui fica o que está implementado e como usar.

## A recuperação: o R1 (14/10/2026, noite)

Depois dos testes que mostraram o Corpo v2 de pé e parado, a escada de
recuperação do vault (`novo_plano/v2/plano_de_recuperacao_rl_claude.md`) começa
pelo R1, as correções. O que ficou diferente no código:

- **Tarefas da escada:** `assets/rl/tasks/aquisicao_p0.json` (ficar de pé),
  `aquisicao_p1.json` (andar sem juiz, com a muleta do pé no ar),
  `aquisicao_p2.json` (o juiz sozinho num clipe de trote),
  `aquisicao_p3a.json` e `aquisicao_p3b.json` (andar com o juiz, e depois
  trotar). Acervos em `assets/rl/style/acervo_p2|p3a|p3b.json`.
- **Nota:** `velocityForm: "progress"`, `standGate`, `air`, `positiveOnly`,
  `taskScale` em `reward`; `curriculum`, `solverMix` e
  `initialState.movingFraction` na tarefa; `forwardOnly` no piloto. O lote
  exporta as parcelas `stand_gate`, `style_gate` e `air`, e o treinador compõe
  `dt x max(0, T + beta x G x S + A - custos) - F`.
- **Treinador:** serial por padrão (`--sobreposicao` liga a sobreposição),
  desvio fixo por grupo (`--desvio-aprendido` volta ao aprendido), dois
  otimizadores, parada do ator pelo KL, normalização congelada por rodada,
  sementes. `matter-rl iniciar --run R --tarefa T --de-checkpoint RUN[:RODADA]
  [--transferir ator,normalizacao] [--aquecimento N]` abre um treino novo que
  nasce de outro (o ator e a normalização passam; o crítico e o juiz
  recomeçam).
- **Medidas por episódio** (colunas novas de `finished_episodes`):
  `passos_uteis`, `primeiro_passo`, `sequencia`, `pedido_medio`,
  `porta_media`; a cadência não conta os toques dos primeiros 0,3 s.
- **Centro de massa:** `rlLinkCenterOfMass3D`, `rlBodyCenterOfMass3D`,
  `rlBodyCenterOfMassVelocity3D` e `rlLinkPointVelocity3D`
  (`Engine/RL/Body/RLBody3D.hpp`): a posição publicada é a origem do link e a
  velocidade é a do centro de massa.

## A reformulação do Corpo (14/10/2026)

O RL do Corpo foi refeito: o clipe saiu da ação e da entrada da rede e virou
só um juiz de estilo, com um peso que o usuário abaixa até zero. O plano
completo está no vault (`Aprendizado_por_Reforco/novo_plano/plano_claude_rl.md`)
e o resultado de cada experimento ao lado dele.

- **O corpo v2** (`assets/rl/bodies/football_player_v2.json`; o perfil do jogo não
  muda):
  - **a junta dos dedos:** o carregador divide a casca da chuteira na linha dos
    dedos (70% do comprimento a partir do calcanhar, 2,5 cm acima da sola; o
    osso dos dedos do modelo visual fica ali). Ele cria `LeftToe` e `RightToe`,
    filhos dos pés, com 20% da massa do pé e um eixo de −60° a 25° (positivo
    dobra a ponta para baixo). O pé mantém a massa total e o centro de massa
    (`Body/RLFootSplit3D`);
  - **os pulsos travados:** `lockedAxes` tira os eixos da ação e da entrada, e o
    motor os segura na postura base. Destravar é tirar da lista (o eixo volta
    como canal novo, com peso zero);
  - **as posturas como dados:** a base e o alerta são os ângulos do primeiro
    quadro de `natural_idle` e `alert_idle`, guardados no arquivo, sem carregar
    clipe;
  - **o resultado:** 37 eixos atuados (39 + 2 das pontas − 4 dos pulsos) e 20
    links.
- **O contrato v2** (`src/Engine/RL/Contract`, biblioteca `MatterRLContract`, que
  vai para o jogo junto com a política):
  - **`BodyCommand3D`:** a velocidade desejada no mundo, o rumo e o giro da
    câmera e o pulo (pendente, a subida pedida, o tempo desde o aperto);
  - **`PlayerAttributes3D`:** altura, massa, a força por grupo (pernas, tronco,
    braços) e os tetos de velocidade;
  - **`BodyContract3D`:**
    - a entrada tem 528 canais: o próprio corpo, o contato de cada pé resumido
      nos 4 passos de física (fração, força média e pico, pisou, saiu, a
      ponta), os comandos, os dados do jogador e a memória das 3 decisões
      anteriores;
    - nada de clipe, alvo, futuro ou exercício;
    - a ação é o alvo absoluto de cada eixo. Nos passos de física, o alvo anda
      em linha reta de uma decisão à outra, e a velocidade pedida sai da reta da
      ação média da política (`BodyVelocityTarget3D::MeanSlope`, a decisão do
      Experimento A). Os eixos travados ficam na base.
  - **Os testes:** `tests/RL/BodyContractTests.cpp`.
  - **A regra de arquitetura:** o contrato e o corpo não incluem mundos de
    treino, estilo, notas, piloto, imitação nem treinador.
- **`ConvexHull3D`** passou para a `MatterCore`: a divisão do pé usa a casca sem
  a engine gráfica.
- **O Experimento A** (o relatório completo está no vault, `experimento_A.md`):
  - **`MatterRLMotorProbe`:** o balanço do quadril em cinco opções de velocidade
    pedida, com e sem ruído de exploração, e a ponta do pé nos dois solvers;
  - **`tools/rl/custo_atualizacao.py`:** o custo de uma rodada do PPO na GPU
    (redes do plano, juiz, espelho);
  - **`MatterRLPolicyBench`:** a política com 22 jogadores a 30 Hz.
- **`PolicyRuntime3D`:**
  - os pesos são transpostos na carga e a conta anda pelas saídas: vetoriza sem
    reordenar as somas, com o mesmo resultado;
  - `-O3` na biblioteca;
  - `evaluateBatch` faz vários jogadores numa conta só;
  - a rede do plano caiu de 876 para 153 µs por decisão. Os 22 jogadores custam
    ~10% de um núcleo a 30 Hz.
- **O adaptador do controle e o piloto** (`src/Engine/RL/Commands`, biblioteca
  `MatterRLCommands`, dados puros sem física):
  - **`ControlAdapter3D`:** o que o jogador aperta vira o comando do Corpo. O
    direcional fica no referencial da câmera (as teclas dão as 8 direções com as
    diagonais normalizadas; o analógico, qualquer ângulo). A velocidade é a do
    modo do jogo, presa no teto da direção em relação à câmera (`SpeedCaps3D`:
    frente 1, diagonal da frente 0,9, lado 0,65, diagonal de trás 0,6, costas
    0,55 do sprint). O pulo apertado espera a decolagem por até 0,6 s. O jogo e
    a IA usam o mesmo adaptador;
  - **`CommandPilot3D`:** o "jogador de mentira" dos exercícios. Ele segura o
    direcional pelo adaptador: solto (`hold`), uma direção em relação à câmera
    por 0,5–4 s e troca (`stick`), ou a estrela, indo aos pontos e soltando
    aos poucos para parar (`star`). A câmera fica parada, acompanha o movimento
    ou gira em trechos (45–180°, 0,4–1,2 rad/s), e as trocas são só da câmera,
    só da direção ou das duas;
  - **a regra de arquitetura:** sem física, Jolt nem peças do treino.
- **As notas do Corpo** (`src/Engine/RL/Rewards/BodyReward3D`):
  `r = dt × (tarefa − custos) − queda`, e o treinador soma `dt × β × estilo`.
  - **A tarefa:** a velocidade compara o vetor inteiro do centro de massa com o
    pedido, na cauda longa `1/(1 + (e/tol)²)`, no sino, ou na mista (metade
    cauda longa, metade linear `max(0, 1 − e/(|pedido| + 1))`, com inclinação
    longe do pedido; no teste de 24 milhões, com a cauda longa ele ficou
    parado com pedido de andar). A tolerância é 0,2 m/s + 10% do pedido. O olhar é o sino do erro do peito contra a câmera
    (0,4 rad). Os pesos são do exercício (parado 0,5/0,5; andando 0,7/0,3).
  - **Os custos** são somados nos passos de física: o trabalho positivo dos
    motores por peso, o uso do motor `(τ/teto)²`, o escorregar no ponto de
    contato dos pés (pesado pela carga), a força do pé acima de 2 pesos, o giro
    e o tranco da cabeça, a mudança do alvo com as reversões (4×), as juntas no
    fim do curso e o contato que não é pé. O tempo no teto do torque é medido,
    sem peso.
  - **Ainda não:** os eventos (o pulo, o "de pé") entram com os exercícios de
    pulo e de levantar.
- **A tarefa do Corpo v2** (`Environment/BodyTask3D`, o esquema
  `matter-rl-task-2`): o corpo, a decisão do contrato, o jogador, os tetos, a
  nota, o começo, o fim, a variação escondida do mundo e os exercícios como
  roteiros do piloto. `assets/rl/tasks/corpo2.json` é o conjunto do
  Experimento B: parado (20% dos mundos), deslocamento (50%) e estrela (30%),
  em andar, trote e corrida.
- **O lote v2** (`Environment/BodyTrainingBatch3D`) só orquestra o contrato, o
  piloto pelo adaptador e as notas:
  - `step(ações, médias)`: a média da política vira a velocidade pedida aos
    motores;
  - **as cotas:** cada mundo fica no seu exercício pelo peso, então a fatia das
    decisões é a fatia dos mundos;
  - **o crítico** vê a entrada sem ruído e o que o ator não vê: o atrito, a
    massa, a força, o atraso, o contato de cada link, o exercício, o modo da
    câmera e a idade do pedido;
  - **o atraso** de um passo de física desloca a reta inteira (o contrato tem
    `writeTargetsAt`);
  - **os começos pelo acervo** (a muleta da seção 10.1 do plano;
    `initialState.referenceFraction`, 0,4 no `corpo2`): um quadro de um clipe
    de treino, sorteado pelo peso do grupo, com a pose e as velocidades dele.
    Só quadros de pé (a pelve a 60% da altura de pé ou mais; o levantar tem o
    currículo dele). A câmera começa no rumo do peito do clipe, e o primeiro
    pedido segue o movimento (`CommandPilot3D::follow`: o direcional nessa
    direção e velocidade, ou a ponta da estrela dessa direção). Só no treino;
    a avaliação começa de pé;
  - **os estados guardados** (a versão simples da seção 10.1;
    `initialState.bankFraction`, zero no `corpo2`): cada cena guarda até
    `bankCapacity` estados de pé da própria experiência, um a cada ~1 s de
    cada mundo, com prioridade para os interessantes (velocidade, giro, pelve
    baixa). Uma fatia dos episódios recomeça deles, girada num rumo sorteado,
    com o pedido seguindo o movimento. É o que troca os começos pelo acervo no
    Experimento C. Cada episódio diz de onde começou (de pé, quadro de clipe
    ou estado guardado; a coluna `comeco`);
  - **as medidas de cada episódio:** as notas, os erros de velocidade e de
    olhar, a reação a cada troca do pedido, os custos, a telemetria da ação
    (tamanho, presa no limite, alvo preso no curso, reversão) e a cadência;
  - **na ponte:** `matter_rl_native.BodyBatch`, com `episode_columns()` e
    `frame_lines()` para as folhas de quadros;
  - **a velocidade medida:** ~16.900 decisões/s com 512 mundos e ações
    aleatórias (os 20 links e as pontas no chão).
  - **Os testes** (`tests/RL/BodyBatchTests.cpp` e `tools/rl/tests/test_native.py`):
    o adaptador, o piloto (determinístico, dentro dos tetos, os modos da
    câmera), a ordem da nota e as trapaças, a conta das parcelas, o
    determinismo com qualquer número de threads e a paridade de informação (a
    entrada montada só com o que o jogo tem é a do treino).
- **O acervo do juiz de estilo** (`src/Engine/RL/Style`, só no treino):
  - **`StyleFeatures3D`:** o que o juiz vê de um instante, no referencial do
    rumo da pelve: a altura, a inclinação, a velocidade e o giro da pelve, os
    35 eixos (ângulo e velocidade) e a cabeça, as mãos e os pés em relação à
    pelve (95 números). Tudo sai só do estado dos links, pela mesma conta no
    corpo e no acervo: os dois lados nunca diferem por convenção numérica. As
    pontas dos pés ficam de fora (os clipes não têm a junta dos dedos; o juiz
    ensinaria a nunca usá-la). As posições se dividem pela escala da altura e
    as velocidades seguem a escala de Froude;
  - **`MotionDataset3D`** (`assets/rl/style/acervo_corpo.json`): cada clipe
    tocado no corpo vira pares de instantes separados por uma decisão (4
    quadros), com o peso por grupo e os reservados fora do treino. O espelho
    dos clipes entra com o `Mirror3D` (E8);
  - **o acervo de hoje:** 47 clipes em 8 grupos, 12 deles reservados. Ficam de
    fora (o motivo está no arquivo):
    - os 6 pulos: no corpo, eles ficam em pé sem sair do chão, porque eram
      poses do pulo animado em que a física fazia a subida;
    - o `sprint_backward_long`: de costas a 7 m/s;
    - os carrinhos (fase Bola);
  - **o lote v2** entrega, de cada mundo, o par da decisão que acabou
    (`stylePairs`, `BodyBatch.style_pairs()`), e a ponte tem
    `matter_rl_native.motion_dataset(corpo, acervo)`;
  - **os testes** (`tests/RL/StyleTests.cpp`): o layout sem as pontas, a mesma
    medida em qualquer lugar e rumo, as medidas batendo com o quadro do clipe,
    o teste do acervo (o clipe posto no corpo da física dá as medidas que o
    acervo calcula, e a velocidade de cada eixo é a do backend), os pesos por
    grupo e o par do lote;
  - **a regra de arquitetura:** o estilo só entra nos mundos de treino.
- **`rlBodyLinkStates3D`** passou a calcular as velocidades com a raiz na
  origem: longe dela, a diferença das posições perdia a precisão do float (a
  375 m, onde ficam mundos de treino, ~0,03 m/s de erro).
- **O juiz de estilo** (`tools/rl/matter_rl/amp.py`):
  - **a rede:** 1024-512 com ReLU e uma saída, por mínimos quadrados (o acervo
    vale +1 e o corpo −1);
  - **o que segura o juiz:** a penalidade de gradiente nos pares do acervo (5),
    uma regularização leve na última camada e a memória dos pares recentes do
    corpo (1 milhão, na CPU). A entrada é normalizada pelo acervo de treino;
  - **a nota:** `max(0, 1 − 0,25 (D − 1)²)`, e o treinador soma `dt × β × nota`;
  - **a saúde:** a nota média no acervo e no corpo, a acurácia (sadia por
    volta de 60–80%), a penalidade e a nota nos clipes reservados;
  - **ainda não:** o juiz de avaliação congelado, que precisa de checkpoints
    de vários momentos do treino (vem com o exame, E9).
- **O treinador do Corpo v2** (`tools/rl/matter_rl/body_trainer.py`):
  - **as redes do plano:** ator e crítico 1024-512-256 (os tamanhos ficam no
    `run.json` e no checkpoint);
  - **a sobreposição:** enquanto a GPU atualiza com a rodada k, a física
    coleta a k+1 com uma cópia da política de antes, num fluxo próprio da GPU.
    O PPO já guarda as probabilidades de quem coletou. O KL que ajusta a taxa
    de aprendizado é medido contra a política do começo da atualização
    (`Rollout.anchor_mean`): medido contra quem coletou, ele somava a mudança
    da atualização anterior, e a taxa caía pela metade (visto no treino curto
    de teste);
  - **a velocidade medida** (512 mundos, modo Equilibrado): ~9.000 decisões/s
    em série e ~12.200 com a sobreposição. O gargalo é a física do corpo v2:
    as pontas dos pés custam ~19% (16.800 contra 20.800 decisões/s com o corpo
    v1, no mesmo lote, 10 threads). Cascas mais simples no calcanhar e na
    ponta quase não ajudam (6 mm de tolerância: +4%): o custo vem dos corpos,
    das juntas e dos pares de contato a mais;
  - **o β** fica na cópia da tarefa do treino (`style.beta`). `matter-rl peso
    estilo 0.5` muda com o treino andando e salva um checkpoint antes;
    `matter-rl peso slip 0.5` (e os outros custos) também;
  - **a variante:** `--sem-estilo` cria o treino sem o juiz (a B1 do
    Experimento B); `--sem-sobreposicao` desliga a sobreposição;
  - **a abertura:** `matter-rl treinar/iniciar --tarefa <tarefa do esquema 2>`
    cria o treino; um treino que já existe reabre do último checkpoint;
  - **cada rodada no banco:** a coluna `details` (JSON), com o β, a nota de
    estilo média, a saúde do juiz e as medidas de cada exercício (quedas,
    duração, erros de velocidade e de olhar, reação, escorregar, trabalho,
    tempo no teto, reversão e cadência);
  - **a avaliação** (a cada `evaluation_minutes`, 10 por padrão, ou com
    `matter-rl avaliar`): cada exercício roda com a ação média (como no jogo),
    em 32 mundos fixos, no solver do jogo (20/20) e no do treino. O primeiro
    episódio de cada mundo vira as medidas do exercício, na tabela
    `body_evaluations` e no status. As quedas vêm com o limite de cima do
    intervalo de 95% (zero em 32 ainda é compatível com ~11%), e o erro de
    velocidade com a mediana e o percentil 90.
- **A prévia do Corpo v2** (`tools/rl/previa_corpo.py`): uma política exportada
  no lote v2 com a física do jogo, mais os dumps para as folhas de quadros.
- **O teste de dependência** (`tools/rl/tests/test_dependency.py`, seção 12.6):
  num empacotamento sem os arquivos de animação (o corpo, o perfil e a tarefa
  numa pasta temporária; nada se apaga), o lote roda, o treino sem o juiz
  treina e exporta, e a política exportada roda no runtime em C++. A entrada
  exportada não tem canal de clipe, de alvo ou de exercício. Com a fatia do
  acervo ligada, o mesmo pacote falha (a prova de que ele não alcança os
  clipes).
- **A afinidade com o acervo** (`tools/rl/matter_rl/affinity.py`, uma peça do
  exame da E9; `matter-rl afinidade treinar --politicas a,b` e `medir`):
  - **um juiz de avaliação congelado,** treinado uma vez: o acervo (+1)
    contra amostras de políticas e movimentos estragados de propósito (ruído
    grande, metades de pares diferentes e a pose exagerada). Congelar ou
    inverter o tempo não entram: em 33 ms o par estragado fica quase o de
    verdade;
  - **mede compatibilidade com o acervo, não naturalidade,** e nunca decide
    sozinho;
  - **no teste de 14/10,** com 4 políticas de teste como negativas: os clipes
    de treino tiram 0,98, os reservados 0,75 e os negativos 0,13. Duas
    políticas de fora do treino, ambas com posturas ruins, ficam no meio (0,60
    e 0,46).
- **O cérebro do Corpo no jogo** (`src/Engine/RL/Brain/BodyBrain3D`, biblioteca
  `MatterRLBrain`: o contrato v2 mais a política exportada, sem nada do
  treino):
  - a cada decisão, `decide` (a entrada e a ação média) ou `decideMany` (os
    22 numa conta só); a cada passo de física, `targets` antes e `afterStep`
    depois;
  - recusa uma política de outro contrato (os tamanhos ou os nomes da
    entrada);
  - **a paridade do runtime** (`tests/RL/BodyBrainTests.cpp`): numa cena só
    dele, com o estado inicial e os comandos do mundo 0 do lote, o cérebro dá
    a mesma entrada a cada decisão e a mesma trajetória, bit a bit, por 24
    decisões.
- **O espelho** (`src/Engine/RL/Contract/Mirror3D`, a mecânica da E8 adiantada;
  a comparação com e sem ele depende de treino):
  - **o mapa sai das poses:** para cada eixo de cada junta, o sinal do eixo do
    outro lado é o que reproduz a pose refletida. A medida é nas superfícies
    dos colisores (a distância com sinal de cada ponto até o próprio colisor
    contra a do refletido até o do outro lado), então a triangulação das
    cascas não conta. O corpo v2 é simétrico até 1,5 mm, e todos os eixos se
    decidem;
  - **os mapas dos layouts** pelos nomes dos canais: a entrada, o crítico, a
    ação e as medidas do juiz. Os vetores trocam o y, os giros trocam o x e o
    z, os eixos vão para o outro lado com o sinal medido (a posição desde a
    base e a ação levam também um deslocamento, porque a base não é
    simétrica), e o seno e o giro do olhar e o y do movimento trocam de sinal;
  - **os testes** (`tests/RL/MirrorTests.cpp`):
    - o espelho do espelho é o original;
    - as poses dos clipes espelhadas são as refletidas;
    - a física espelhada se afasta 2,4 mm em 0,5 s sem chão e 2,6 mm em
      0,1 s de pé. Depois o boneco sem política tomba e o afastamento cresce
      com o tombo (27 mm em 0,5 s; um trote caindo no chão chega a 155 mm em
      0,25 s);
  - **o acervo** (`"mirror": true`) ganha a cópia espelhada de cada clipe;
  - **a perda do espelho no ator** (`tools/rl/matter_rl/symmetry.py`;
    `mirror_loss` no `run.json`, desligada por padrão). A ponte entrega os
    mapas (`BodyBatch.mirror_maps()`).

## As peças

| Peça | Onde | O que faz |
|---|---|---|
| Mundos de treino | `src/Engine/RL/Environment` | `BodyTrainingBatch3D`: centenas de mundos Jolt em paralelo, sem janela; entrada, nota por parcela, fim de episódio e recomeço no mesmo passo; o visualizador do laboratório usa o mesmo lote, com o comando na mão e empurrões |
| Corpo de RL | `src/Engine/RL/Body`, `assets/rl/bodies` | o perfil do jogo mais os atuadores do RL, as pontas dos pés e os pulsos travados |
| Contrato | `src/Engine/RL/Contract` | a entrada, a ação, o comando e o espelho (vão para o jogo com a política) |
| Tarefas | `assets/rl/tasks` | `aquisicao_p0..p3b.json`, `corpo2.json`: a nota, o término, o começo, a variação do mundo e os exercícios |
| Política no C++ | `src/Engine/RL/Policy` | `PolicyRuntime3D` lê o `.mrlpolicy` que o treinador exporta (a mesma conta do PyTorch, com teste de paridade) |
| Cérebro no jogo | `src/Engine/RL/Brain` | `BodyBrain3D`: o contrato mais a política, sem nada do treino |
| Ligação com o treinador | `src/Engine/RL/Supervisor` | abrir o processo, socket local e o histórico (`rl.db`, só leitura) |
| Ponte Python | `bindings/python` | `matter_rl_native` (nanobind): o lote como arrays numpy sem cópia; a única peça com Python |
| Treinador | `tools/rl` | PPO e o juiz (PyTorch), checkpoints, SQLite, avaliação, canal de controle e a CLI `matter-rl` |
| Tela | `src/Workbench/ReinforcementLearning` | "Aprendizado por Reforço" no menu: o treino escolhido, os controles, os modos, os exercícios, a avaliação, o visualizador ao vivo e os gráficos |
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
  - O seletor no alto escolhe o treino (o mais recente abre sozinho; com um
    treinador aberto, é o dele).
  - "Continuar treino" abre o treinador em segundo plano; Pausar, Retomar,
    Parar, Salvar e Avaliar; modos Máximo, Equilibrado e Leve; "Retomar ao
    ligar o PC".
  - O visualizador roda a política exportada com a física do jogo (20/20):
    o comando do piloto do exercício ou na mão (andar para a frente na
    velocidade escolhida), empurrões, vistas presas ao rumo do jogador.
- **Pelo terminal:** `tools/rl/matter-rl` com:
  - `iniciar --run R --tarefa T [--de-checkpoint RUN[:RODADA]]`, `status`,
    `pausar`, `retomar`, `parar`, `salvar`, `avaliar`;
  - `modo maximo|equilibrado|leve`;
  - `tarefa <id>`: o treino aberto passa para outra tarefa (o mesmo cérebro);
  - `peso <parcela> <valor>` (`estilo` é o β do juiz);
  - `relatorio --desde 1h`, `graficos`, `folha --episodio <dump>`;
  - `afinidade treinar|medir`;
  - `servico instalar|remover`.
- **A avaliação fora do treino:** `tools/rl/avaliar.py RUN --pedido V
  [--solver jogo|treino|ambos]` mede quedas, erro de velocidade, passos úteis
  e escorregar.
- **A prévia e as folhas de quadros:** `tools/rl/previa_corpo.py --tarefa T
  --politica <.mrlpolicy> --segundos S --saida DIR --folha --vista direita`.

## Onde ficam os treinos

`~/.local/share/matter-engine/rl/` (fora do Git; `MATTER_RL_DATA` troca a pasta):

- `rl.db`: o histórico;
- `runs/<treino>/checkpoints/`: o estado inteiro, gravado de forma atômica, com
  limpeza automática;
- `runs/<treino>/exports/policy_latest.mrlpolicy`: a política que a tela e o
  jogo leem;
- `runs/<treino>/tarefa.json`: a tarefa com os pesos de agora;
- `protegidos/`: checkpoints guardados à mão.

## Velocidade (15/10/2026, Ryzen 5 3600, corpo v2, 512 mundos)

| Modo | Threads | Prioridade | Decisões/s |
|---|---:|---|---:|
| Máximo | 10 | nice 10 | ~9.500 |
| Equilibrado (padrão) | 8 | ociosa | ~8.300 |
| Leve | 4 | ociosa | menos |

O gargalo é a física na CPU (a GPU quase não trabalha com essas redes).

## O pé do corpo (12/10/2026, noite)

O pé do corpo de RL é o do jogo: a casca convexa da chuteira, com a sola plana
no contorno dela (antes, uma caixa de 26,9 × 9,6 × 13 cm), e o boneco ficou
2,1 cm mais baixo com o pé nivelado. A sola real é menor que a caixa (o
calcanhar tem ~6 cm de largura): com a ação zero, o corpo cai em ~1,9 s, contra
~2,8 s com a caixa.

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
    lote, estilo, espelho, cérebro, centro de massa e a nota de aquisição;
  - `MatterEngine.ReinforcementLearningScreen`: a tela sem janela num treino
    de mentira. `MATTER_RL_TEST_REAL=<pasta>` (com `MATTER_RL_DATA`,
    `MATTER_RL_TEST_RUN` e `MATTER_RL_TEST_SPEED`) roda o treino de verdade
    sem janela, imprime o pedido e a velocidade e grava o visualizador de
    lado e de frente;
  - as calibrações (`RLCalibration`, `RLJointLimits`, `RLDynamics`).
- **Python:** `cd tools/rl && .venv/bin/python -m pytest`:
  - PPO, rede, checkpoints, trava, banco, canal de controle, a retomada
    depois de uma queda e a amostra das curvas por tempo;
  - o treinador do Corpo, o juiz, o espelho, a dependência dos arquivos de
    animação e a paridade da ponte nativa com o C++ (< 1e-5).
