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
