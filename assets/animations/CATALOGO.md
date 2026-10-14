# Catálogo dos clipes de animação

Gerado por `tools/animation/catalogo.py` (não editar à mão; rodar de novo depois
de importar um clipe). Os arquivos ficam em `assets/animations/clips/`. As fontes
(os FBX, por tipo) ficam em `assets/animations/source/` e o inventário delas em
`assets/animations/source/INVENTARIO.md`.

**Ciclo:** o clipe repete (trotes, sprints) ou começa e termina (cortes, pulos,
levantar). **Vel.:** a velocidade dele (m/s). **Dir.:** a direção em que anda,
em relação à frente (0° frente, 90° esquerda, 180° costas). **Usado em:** as
tarefas do RL e o personagem do jogo que citam o arquivo.

## Parado

| Clipe | Nome | s | Ciclo | Vel. | Dir. | Fonte | Usado em |
|---|---|---:|---|---:|---:|---|---|
| `alert_idle` | Parado em alerta | 10.00 | sim | — | 0° | autoral (`tools/animation/clips/alert_idle.py`) | personagem (football_player) |
| `cc0_idle` | Parado natural | 2.50 | sim | — | — | `UAL1_Standard_RM.glb` | — |
| `natural_idle` | Parado natural | 4.00 | sim | — | 0° | autoral (`tools/animation/clips/natural_idle.py`) | — |

## Giros parado

| Clipe | Nome | s | Ciclo | Vel. | Dir. | Fonte | Usado em |
|---|---|---:|---|---:|---:|---|---|
| `turn_left` | Giro parado para a esquerda (Mixamo) | 1.00 | não | — | — | autoral (`assets/animations/source/mixamo/giros_parado/LeftTurn.fbx`) | personagem (football_player) |
| `turn_right` | Giro parado para a direita (Mixamo) | 1.00 | não | — | — | autoral (`assets/animations/source/mixamo/giros_parado/RightTurn.fbx`) | personagem (football_player) |

## Andar

| Clipe | Nome | s | Ciclo | Vel. | Dir. | Fonte | Usado em |
|---|---|---:|---|---:|---:|---|---|
| `ual_walk` | Andar (UAL1 CC0) | 1.33 | sim | 1.1 | 0° | `UAL1_Standard_RM.glb` | corpo, h3b_caminhar_alvo |
| `walk_backward` | Andar de costas (Mixamo) | 1.20 | sim | 1.2 | -179° | `Walking Backwards.fbx` | corpo |
| `walk_strafe_left` | Andar de lado para a esquerda (Mixamo, espelhado) | 1.47 | sim | 0.6 | 89° | `Walk Strafe Right.fbx` | corpo |
| `walk_strafe_right` | Andar de lado para a direita (Mixamo) | 1.47 | sim | 0.6 | -89° | `Walk Strafe Right.fbx` | corpo |

## Arrancar e frear

| Clipe | Nome | s | Ciclo | Vel. | Dir. | Fonte | Usado em |
|---|---|---:|---|---:|---:|---|---|
| `crouched_to_sprint` | Arrancada agachada (Crouched To Sprinting) | 0.50 | não | — | -2° | autoral (`crouched_to_sprint`) | personagem (football_player) |
| `idle_to_sprint` | Idle para sprint | 0.80 | não | 3.9 | 1° | `Idle To Sprint.fbx` | personagem (football_player), h3_andar |

## Curvas e cortes

| Clipe | Nome | s | Ciclo | Vel. | Dir. | Fonte | Usado em |
|---|---|---:|---|---:|---:|---|---|
| `change_direction` | Chegar correndo, cortar e sair correndo para trás (Mixamo) | 2.17 | não | 0.0 | — | `ChangeDirection.fbx` | — |
| `run_backward_arc_right` | Corrida de costas em arco para direita | 0.67 | não | 3.0 | -128° | `Run Backward Arc Right.fbx` | personagem (football_player) |
| `run_forward_arc_left` | Trote em arco para esquerda (referencia inferior) | 0.73 | não | 3.0 | 32° | `Run Forward Arc Left.fbx` | personagem (football_player) |
| `run_forward_arc_right` | Corrida em arco para direita | 0.90 | não | 4.0 | -18° | `Run Forward Arc Right.fbx` | personagem (football_player) |
| `running_to_turn` | Correr, frear e virar (Mixamo) | 2.27 | não | 1.8 | 6° | `RunningToTurn.fbx` | — |
| `running_turn_180` | Meia-volta correndo, no lugar (Mixamo) | 0.67 | não | 0.1 | — | `RunningTurn180.fbx` | — |
| `running_turn_180_loop` | Vai e volta correndo, em ciclo, no lugar (Mixamo) | 1.87 | sim | — | — | `RunningTurn180Longo.fbx` | — |
| `turn_to_running` | Virar e sair correndo (Mixamo) | 1.70 | não | 1.9 | -90° | `TurnToRunning.fbx` | — |

## Trote

| Clipe | Nome | s | Ciclo | Vel. | Dir. | Fonte | Usado em |
|---|---|---:|---|---:|---:|---|---|
| `jog` | Corrida leve | 0.73 | sim | 3.0 | 0° | autoral (`tools/animation/clips/jog.py`) | personagem (football_player) |
| `jog_strafe_left` | Corrida de lado (esquerda) | 0.70 | sim | 2.9 | 90° | autoral (`tools/animation/clips/strafe.py`) | personagem (football_player), corpo, h4_direcoes, h5_pontos, h6_peito, h7_tiro |
| `jog_strafe_right` | Corrida de lado (direita) | 0.70 | sim | 2.9 | -90° | autoral (`tools/animation/clips/strafe.py`) | personagem (football_player), corpo, h4_direcoes, h5_pontos, h6_peito, h7_tiro |
| `mixamo_running` | Corrida (analise Mixamo) | 0.83 | sim | 2.5 | 1° | `Running.fbx` | corpo, h4_direcoes, h5_pontos, h6_peito, h7_tiro |
| `trot_backward` | Trote de costas | 0.80 | sim | 2.4 | 180° | autoral (`tools/animation/clips/trot.py`) | corpo, h4_direcoes, h5_pontos, h6_peito, h7_tiro |
| `trot_backward_left` | Trote de costas diagonal (esquerda) | 0.83 | sim | 2.6 | 135° | autoral (`tools/animation/clips/trot.py`) | corpo, h4_direcoes, h5_pontos, h6_peito, h7_tiro |
| `trot_backward_right` | Trote de costas diagonal (direita) | 0.83 | sim | 2.6 | -135° | autoral (`tools/animation/clips/trot.py`) | corpo, h4_direcoes, h5_pontos, h6_peito, h7_tiro |
| `trot_forward` | Trote | 0.83 | sim | 2.6 | 0° | autoral (`tools/animation/clips/trot.py`) | — |
| `trot_forward_left` | Trote diagonal (esquerda) | 0.83 | sim | 2.8 | 45° | autoral (`tools/animation/clips/trot.py`) | corpo, h4_direcoes, h5_pontos, h6_peito, h7_tiro |
| `trot_forward_right` | Trote diagonal (direita) | 0.83 | sim | 2.8 | -45° | autoral (`tools/animation/clips/trot.py`) | corpo, h4_direcoes, h5_pontos, h6_peito, h7_tiro |

## Corrida e sprint

| Clipe | Nome | s | Ciclo | Vel. | Dir. | Fonte | Usado em |
|---|---|---:|---|---:|---:|---|---|
| `mixamo_sprint` | Sprint (analise Mixamo) | 0.57 | sim | 6.0 | 1° | `Sprint.fbx` | corpo, h3_andar, h7_tiro |
| `run_backward` | Corrida de costas | 0.63 | sim | 2.9 | 180° | autoral (`tools/animation/clips/run_backward.py`) | personagem (football_player) |
| `run_forward_medium` | Corrida media (Mixamo, 4,2 m/s) | 0.70 | sim | 4.3 | 1° | `Running(2).fbx` | corpo |
| `sprint` | Sprint | 0.47 | sim | 7.5 | 0° | autoral (`tools/animation/clips/sprint.py`) | personagem (football_player), corpo, h3_andar, h7_tiro |
| `sprint_backward` | Sprint de costas | 0.56 | sim | 4.4 | 180° | autoral (`tools/animation/clips/sprint_backward.py`) | personagem (football_player), corpo |
| `sprint_backward_left` | Sprint de costas diagonal (esquerda) | 0.57 | sim | 3.8 | 135° | autoral (`tools/animation/clips/trot.py`) | corpo |
| `sprint_backward_long` | Sprint de costas (longo) | 0.43 | sim | 7.0 | 180° | autoral (`tools/animation/clips/sprint_backward_long.py`) | personagem (football_player) |
| `sprint_backward_right` | Sprint de costas diagonal (direita) | 0.57 | sim | 3.8 | -135° | autoral (`tools/animation/clips/trot.py`) | corpo |
| `sprint_forward` | Sprint | 0.63 | sim | 6.0 | 0° | autoral (`tools/animation/clips/trot.py`) | — |
| `sprint_strafe_left` | Sprint de lado (esquerda) | 0.50 | sim | 4.1 | 90° | autoral (`tools/animation/clips/strafe.py`) | personagem (football_player), corpo |
| `sprint_strafe_right` | Sprint de lado (direita) | 0.50 | sim | 4.1 | -90° | autoral (`tools/animation/clips/strafe.py`) | personagem (football_player), corpo |

## Pulos

| Clipe | Nome | s | Ciclo | Vel. | Dir. | Fonte | Usado em |
|---|---|---:|---|---:|---:|---|---|
| `jump_backward` | Pulo para tras | 1.00 | não | — | 180° | autoral (`tools/animation/clips/jump_backward.py`) | personagem (football_player) |
| `jump_forward` | Pulo correndo | 1.00 | não | — | 0° | autoral (`tools/animation/clips/jump_forward.py`) | personagem (football_player) |
| `jump_left` | Pulo para a esquerda | 1.00 | não | — | 90° | autoral (`tools/animation/clips/jump_left.py`) | personagem (football_player) |
| `jump_prep` | Preparo do pulo | 1.00 | não | — | 0° | autoral (`tools/animation/clips/jump_reference.py`) | personagem (football_player) |
| `jump_right` | Pulo para a direita | 1.00 | não | — | -90° | autoral (`tools/animation/clips/jump_right.py`) | personagem (football_player) |
| `jump_standing` | Pulo parado | 1.00 | não | — | 0° | autoral (`tools/animation/clips/jump_reference.py`) | personagem (football_player) |

## Chão e levantar

| Clipe | Nome | s | Ciclo | Vel. | Dir. | Fonte | Usado em |
|---|---|---:|---|---:|---:|---|---|
| `stand_up_back` | Levantar de costas | 2.67 | não | 0.0 | — | `Getting Up.fbx` | — |
| `stand_up_back_kimodo` | Levantar de costas (Kimodo) | 2.30 | não | — | — | autoral (`Kimodo-SOMA-RP-v1.1`) | personagem (football_player) |
| `stand_up_front` | Levantar de bruços | 2.97 | não | 0.0 | — | `Standing Up.fbx` | — |
| `stand_up_front_crouch` | Levantar de bruços pelo agachado | 4.83 | não | — | — | autoral (`Prone`) | — |
| `stand_up_front_getup` | Levantar de bruços (Getting Up limpo) | 4.30 | não | — | — | autoral (`Getting`) | — |
| `stand_up_front_kimodo` | Levantar de bruços (Kimodo) | 2.10 | não | — | — | autoral (`Kimodo-SOMA-RP-v1.1`) | personagem (football_player) |

## Futebol

| Clipe | Nome | s | Ciclo | Vel. | Dir. | Fonte | Usado em |
|---|---|---:|---|---:|---:|---|---|
| `tackle_slide` | Carrinho sentado (mais rapido) | 1.50 | não | — | -2° | autoral (`running_slide`) | personagem (football_player) |
| `tackle_soccer` | Carrinho de lado (mais devagar) | 1.77 | não | — | -2° | autoral (`soccer_tackle`) | personagem (football_player) |
