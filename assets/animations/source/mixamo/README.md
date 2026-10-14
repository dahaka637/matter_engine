# Fontes do Mixamo

Os FBX do Mixamo (Adobe) que o usuário baixou, **organizados por tipo de
movimento** (13/10/2026). Licença: os termos padrão do Mixamo/Adobe (não CC0).

| Pasta | O que tem |
|---|---|
| `parado/` | parado (Idle, Stand) |
| `giros_parado/` | giros no lugar (90°, 96°, 120°) |
| `andar/` | andar de costas, de lado, andar virando |
| `trote/` | trote e corrida leve (até ~3,5 m/s): frente, costas, lados, diagonais |
| `corrida/` | corrida (3,5–5,5 m/s) |
| `sprint/` | sprint (acima de 5,5 m/s), inclusive de costas |
| `curvas_e_cortes/` | arcos, viradas correndo (36°–60°), meias-voltas e cortes de 180° |
| `arrancar_e_frear/` | arrancadas (parado → sprint) e paradas |
| `pulos/` | pulos parado, para a frente, correndo, aterrissagem |
| `tropecos_e_reacoes/` | tropeços |
| `chao_e_levantar/` | deitar, rolar, levantar |
| `futebol/` | carrinho, deslizar |
| `goleiro/` | reposição do goleiro |

- **Quanto cada um dura, anda e gira:** `../INVENTARIO.md`, gerado por
  `tools/animation/inventario.py`.
- **Os clipes já importados** (`assets/animations/clips`) e onde cada um é
  usado: `assets/animations/CATALOGO.md`.

**Arquivo novo:**
1. Entra na pasta do tipo dele, com o nome do Mixamo. Se o nome já existir,
   ganha a duração no fim, ex.: `Getting Up [7,6s].fbx`.
2. Rode o inventário de novo.
3. Escreva aqui embaixo, na história, o que ele é e para que serve.

Os "(1)", "(2)"... são downloads diferentes do mesmo nome, com outro conteúdo
(as duplicatas exatas foram tiradas).

## A história das fontes

Os textos abaixo são os READMEs de antes da organização, sem mudança. Onde
dizem `study/`, `viradas/` ou `biblioteca/`, os arquivos agora estão na pasta
do tipo deles.

### A primeira fonte (Running.fbx, 24/09)

```text
Fonte: Mixamo (Adobe), download avulso "Running.fbx" em 24/09/2026.

Licença: termos padrão do Mixamo/Adobe (não CC0). Por isso este clipe não
entra no manifesto do personagem (assets/characters/*/character.json) nem na
biblioteca ativa CC0 descrita em assets/animations/README.md. Ele existe
apenas para o workspace procedural do menu ANIMAÇÕES (inspeção de canais,
retarget e velocidade), regenerável a qualquer momento a partir deste FBX via
tools/import_humanoid_animation.py.
```

### As referências de estudo (antiga `study/`)

```text
Fontes: Mixamo (Adobe), baixadas avulsas em 24/09/2026 (mesmos termos de
licença do README.txt em assets/animations/source/mixamo/).

Diferente de Running.fbx e Sprint.fbx, estes sete arquivos NUNCA foram
importados como clipe (.matteranim.json) e não devem ser. Eles serviram só
de referência visual/numérica (inspecionados com
tools/import_humanoid_animation.py --inspect-only) para calibrar sistemas
procedurais escritos à mão em CharacterLocomotion3D: inclinação do corpo ao
virar correndo, ideia de passo ao virar parado, pose de salto sem o floreio
de braço das fontes, e pose de levantar do chão. Mantidos aqui só para
permitir reconferir os números originais se os sistemas procedurais
precisarem de reajuste.

- Running Right Turn.fbx: inclinação/torção do tronco correndo em curva.
- Right Turn.fbx, Right Turn(1).fbx: ideia de reposicionamento dos pés ao
  virar parado.
- Running Jump.fbx: pose de salto em corrida.
- Jumping Up.fbx: pose de salto parado/lento (nota: tem preparação antes do
  salto; o salto do jogo é instantâneo, então essa antecipação foi
  descartada, só a pose de voo/aterrissagem foi usada como referência).
- Getting Up.fbx: levantar de costas no chão (supino). Revisado em
  25/09/2026: a primeira versão desta nota dizia "referência", mas
  `applyGetUpPose` (CharacterLocomotion3D.cpp) só reaproveitava números
  antigos do BiomechanicalBipedExperiment3D — o arquivo nunca tinha sido
  de fato inspecionado. Corrigido: rodado com `--inspect-only`/`--recovery`
  (ambos passam nos gates de qualidade do perfil `CrashTestDummyV1` mesmo
  com `--recovery`, então os ângulos abaixo já saem limitados aos limites
  físicos reais da junta) e também com `--output` para um clipe de
  referência temporário (não commitado — só para ler `jointPositionRadians`
  quadro a quadro; o clipe em si não é usado no jogo, mesma diretriz de
  sempre). Âncoras usadas por fase (fração do clipe → graus, já
  simetrizado E/D): Assessing (compartilhada, não vem do clipe) → SupineTuck
  0,33–0,40 (quadril -73°..-83°, joelho 135°..122°) → SupineSit 0,50→0,60
  (quadril -50°→-22°, joelho 89°→47°) → GatherFeet 0,70 (quadril -14°,
  joelho 29°, com Standing Up.fbx 0,65 em média).
- Standing Up.fbx: levantar de bruços/peito no chão (prono). Mesma correção
  de 25/09/2026 acima. Âncoras: ProneBrace 0,20→0,30 (cotovelo dobrado
  ~130°→~114°, apoiando nos antebraços) → PronePush 0,46→0,55 (quadril
  -83°→-66°, quadril "em pique" acima do tronco enquanto os braços
  esticam) → GatherFeet 0,65 (ver acima, compartilhado com Getting Up.fbx).

Adicionados em 26/09/2026, trazidos pelo usuário como INSPIRAÇÃO para as
passadas autorais (não para copiar): medidos com
tools/animation/reference_capture.py + gait_metrics.py, e os números que
caracterizam cada passada viraram parâmetros do gerador
(tools/animation/gait.py). Nenhuma pose destes arquivos entra em clipe.

- Slow Run.fbx: corridinha lenta -> corrida leve (clips/jog.py). 2,95 m/s,
  164 passos/min, 41% de voo, pé tocando 17 cm à frente do quadril, joelho
  41° no toque e 100° no balanço, cotovelo 63-116°, ombros girando 17°.
- Running(2).fbx: corrida de 4,1 m/s -> inspiração leve do sprint
  (clips/sprint.py): pisar mais embaixo do corpo, calcanhar alto atrás,
  ombros girando (36° na referência).
- Running Backward.fbx, Run Backward.fbx: corrida de costas, as duas
  concordam -> clips/run_backward.py. ~3 m/s, 189 passos/min, ponta tocando
  atrás do quadril com o joelho a ~60°, base ~12 cm do centro. Run
  Backward.fbx tem o corpo virado 180° em relação à pose de repouso;
  gait_metrics.py mede pelo rumo real dos quadris.


Adicionados em 26-27/09/2026:

- Jog Strafe Left.fbx, Jog Strafe Right.fbx: correr de lado. EXCEÇÃO à
  regra acima, a pedido do usuário ("o mais próximo possível do modelo"):
  retargeteados (assets/animations/source/retargeted/) e usados como
  strafe, com as correções mínimas que o corpo exige
  (tools/animation/clips/strafe.py).
- Jumping(2).fbx: pulo parado -> clips/jumps.py (STANDING), sem os braços
  acima da cabeça.
- Jump.fbx: pulo correndo para a frente -> FORWARD, salto de passada em
  tesoura, sem o giro largo de braços.
- Jump(1).fbx: pulo para trás da primeira versão (joelhos recolhidos).
- Jump(2).fbx (27/09): pulo para trás atual -> BACKWARD. Medido com
  reference_capture.py: impulso na perna da frente (esticada, joelho 10°),
  a de trás sobe dobrada (joelho 110-122°), estica no voo e pousa primeiro;
  a da frente sobe dobrada à frente. Tronco -10° no impulso, 0° no ápice,
  +12° na queda. Os braços da referência sobem acima dos ombros e se
  debatem: ficaram de fora.

Adicionados em 27/09/2026, trazidos pelo usuário como INSPIRAÇÃO (não para
copiar): capturados com tools/animation/reference_capture.py e medidos
quadro a quadro. Nenhuma pose destes arquivos entra em clipe; viraram
sistemas procedurais em CharacterLocomotion3D (ver
docs/CHARACTER_FOOTWORK.md, "Freada" e "Giro parado").

- Run To Stop.fbx: parar correndo (4 m/s até parar em ~0,35 s). A perna da
  frente trava esticada à frente (joelho ~40°), a de trás dobra até ~108°,
  a pelve desce 25 cm, o tronco segue a inércia de 12° para 32° à frente e
  tudo volta em ~0,7 s, com a pelve girando ~40° para uma base de lado.
  -> reação de freada (mola do tronco empurrada pela desaceleração, pelve
  descendo, cotovelos abrindo), que nunca segura o movimento.
- Left Turn 90.fbx, Right Turn(3).fbx, Happy Right Turn.fbx: girar parado
  (90°, 95°, 120°). Nas três, o tronco começa e a pelve gira contínua; o pé
  do lado do giro sai primeiro e abre quase no lugar, sobre a bola do pé
  (calcanhar 3–7 cm acima); o outro contorna o de apoio em arco (0,3–0,45
  m); ~0,3 s por passo, 90° em dois passos. -> jogo de pés do giro parado.

- Idle.fbx: EXCEÇÃO à regra de somente inspiração. Foi retargeteado como
  alert_idle, a única pose parada, preservando a base forte e escalonada; a
  pose original de tronco, braços e cabeça é preservada. Só o rumo global e
  o plantio físico dos pés são adaptados.
- Right Turn(4).fbx, Left Turn.fbx: referência do giro procedural partindo
  dessa base em alerta. Para os dois lados, o pé de trás contorna primeiro o
  da frente, que sustenta o peso; o pé da frente recompõe a base no segundo
  passo. As poses dos FBX não entram no runtime.
```

### As viradas correndo (antiga `viradas/`, 13/10)

```text
Fonte: Mixamo (Adobe), baixados pelo usuario em 13/10/2026 para as viradas
correndo do RL (o tiro com volta, etapa 4a do plano de imitacao).

- RunningTurn180.fbx  ("Running Turn 180"): a meia-volta repentina. Veio
  no lugar (In Place): gira 180 graus sem andar (0,04 m em 0,67 s).
- RunningToTurn.fbx   ("Running To Turn"): corre, freia, vira (174 graus) e
  termina parado; anda 4,1 m em 2,27 s.
- TurnToRunning.fbx   ("Turn To Running"): parado, vira e sai correndo;
  anda 3,3 m em 1,70 s.
- RunningTurn180Longo.fbx ("Running Turn 180" estendido, baixado depois; o
  Mixamo nao oferece tirar o In Place nele): o vai e volta inteiro em ciclo
  (base aberta, gira, alguns passos, planta, gira de volta, volta), 1,87 s,
  fecha o ciclo (clipe running_turn_180_loop, com --loop). Tambem no lugar:
  o deslocamento tera de ser reconstruido pelo pe de apoio (contacts).
- ChangeDirection.fbx ("Change Direction"): chega correndo, planta de lado,
  desce bem baixo para cortar e sai correndo para tras (180 graus), 2,17 s,
  sem parar no meio (clipe change_direction). Tambem no lugar (0,08 m).

Licenca: termos padrao do Mixamo/Adobe (nao CC0), como os outros desta pasta.
Clipes gerados (assets/animations/clips/running_turn_180, running_to_turn,
turn_to_running, running_turn_180_loop e change_direction) com:

  env PYTHONHOME=/usr blender --background --factory-startup \
    --python-use-system-env --python-exit-code 1 \
    --python tools/import_humanoid_animation.py -- <fbx> --source-rig mixamo \
    --profile assets/characters/football_player/FootballPlayerV1.ragdoll.json \
    --root-motion preserve --dynamic-projection --output <clipe> --id <id> --name <nome>

O --dynamic-projection foi preciso (como no Sprint): sem ele o joelho de um
quadro passa do teto de 8 graus de erro de membro (14,8 a 17,7 graus).
```

### Os downloads guardados (antiga `biblioteca/`, 13/10)

```text
Fonte: Mixamo (Adobe). Os FBX que o usuario baixou ao longo de setembro e
outubro de 2026 e que so existiam na pasta Downloads dele: copiados para ca em
13/10/2026 (sem duplicatas pelo conteudo; os "(1)" sao downloads diferentes do
mesmo nome, com outro conteudo). Licenca: termos padrao do Mixamo/Adobe (nao
CC0), como os outros desta pasta.

O inventario de todas as fontes (duracao, quanto o quadril anda, giro) esta em
assets/animations/source/INVENTARIO.md, gerado por tools/animation/inventario.py.
Rode de novo quando chegarem arquivos.
```

### Giros parados (14/10)

```text
Left Turn(2).fbx, Right Turn(6).fbx: giros de 180 graus no lugar (2,1 s), os
clipes giro_esquerda_180 e giro_direita_180. Quick 180 Turn (rifle).fbx e Turn
90 Right (rifle).fbx: giros com um rifle nas maos (o que vale e o movimento dos
pes), ainda nao importados.
```
