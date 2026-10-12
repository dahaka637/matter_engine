# Aprendizado por Reforço no Matter Engine

O personagem aprende do zero, por etapas sob demanda. A primeira é a H1:
ficar em pé, estável, sem ajuda nenhuma. O plano completo e o diário ficam no
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
  - `peso <parcela> <valor>`;
  - `relatorio --desde 1h`, `graficos`, `episodios`, `folha --cena <nome>`;
  - `comparar A B`: a bateria em dois checkpoints (rodada, `ultimo` ou
    `melhor`), lado a lado, fora do treino;
  - `folha --checkpoint X --mundo N [--de S --ate S]`: um mundo qualquer da
    bateria, num checkpoint, no trecho pedido;
  - `servico instalar|remover`.

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

## A H1 do zero (validação de 12/10/2026)

- **Em ~1 min** ele já fica de pé no treino.
- **Em ~10 min** (11 milhões de decisões): a bateria `equilibrio_v1` dá 100%
  de pé por 60 s nos dois solvers. A oscilação do centro de massa fica em
  0,8 cm no 10/6 e 0,3 cm no 20/20.
- **Linha de base:** a ação zero (o PD segurando a pose base) fica 0% de pé e
  cai em 2,6–2,8 s.
- **O tronco girando:** sem a parcela `trunkTurn`, 15 dos 32 mundos ficavam
  com a pelve rodando de um lado para o outro (até 46° de ponta a ponta), com
  os pés plantados. O centro de massa quase não se move, então a oscilação
  não via. Agora a recompensa cobra a rotação da pelve e do peito
  (−0,2·ω², quadrática para ensinar também com o ruído do treino), e a
  bateria mede o giro do tronco (`turn`), que entra na nota.

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
  - o treinador inteiro: continuar do checkpoint e a recuperação de NaN;
  - ponte nativa e paridade com o C++ (< 1e-5).
