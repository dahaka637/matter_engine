# Ragdoll ativo V2 — arquitetura e continuidade

Atualizado em 17/09/2026. **Etapa experimental, não locomoção aprovada.**
O relato de resultados da sessão fica em `DESENVOLVIMENTO_ATUAL.md`.
Passagem final detalhada no brain: `30_Projects/matter_engine/HANDOFF_2026-09-17_Ragdoll_Ativo.md`.
O estudo que fundamenta as decisões está em
[`ACTIVE_RAGDOLL_ROBOTICS_STUDY.md`](ACTIVE_RAGDOLL_ROBOTICS_STUDY.md).
Este documento descreve o código novo; não reativa os controladores arquivados.

## Contrato que não pode regredir

A animação é um alvo muscular limitado pelo perfil, não uma transformação
aplicada aos links físicos. A simulação continua autoritativa. O corpo
observado, a gravidade e os contatos externos atuais definem o contexto do
controle. Não há mola de raiz para uma posição mundial antiga, integração
de velocidade para formar um trilho, nem dívida de deslocamento após arraste.
O registro da posição inicial serve somente para medir distância percorrida.

Forças auxiliares podem agir em XYZ, inclusive verticalmente. O auxílio de
pose retira sua resultante de força/momento; intenção de velocidade e torque
de postura têm orçamento externo explícito e compartilhado entre links.
Valores de força/gain são calibração experimental, não garantias anatômicas.
Elevar assistência para esconder uma falha de apoio viola o objetivo desta
etapa, ainda que faça um teste visual parecer estável.

O contrato anterior de deixar o corpo mole durante a Physgun foi substituído
por solicitação do usuário: durante o arraste, permanecem motores internos
limitados, com redução de autoridade no membro agarrado. Intenção de caminhar
e forças auxiliares externas são suspensas. A soltura inicia transição da
pose medida na localização presente, sem retomar o percurso anterior.

## Responsabilidades implementadas

| Camada | Responsabilidade | Não significa |
| --- | --- | --- |
| `ContactFootwork3D` | Observa COM/cargas/apoios, calcula indicador de captura e propõe transferência/passada em referencial corporal | Resolver a dinâmica completa de contato ou garantir que a passada é realizável |
| `RagdollPoseMotor3D` | FK com anchors coincidentes e ajuste numérico limitado de alvos dos pés nas juntas | Teletransportar pés ou impor uma restrição mundial à pelve |
| `AnimatedRagdollController3D` | Estados, seleção de clipes, alvos suaves, reação articular, classificação de queda e tentativa de recuperação | Caminhada ou recuperação fisicamente aprovadas |
| `BodyRelativeRagdollAssist3D` | Auxílio corporal sem destino mundial, com orçamento global | Fonte principal de sustentação ou substituto dos apoios |
| Backend PhysX | Motores, esforço interno, colisões e snapshots da simulação | Reproduzir diretamente a pose cinemática do visualizador |
| Workbench | Configuração pelo manifesto, comandos experimentais e telemetria | Controlador físico independente da Engine |

A ordem fixa é preservada a 120 Hz: observar o snapshot anterior, decidir
alvos/forças, avançar e buscar os resultados físicos, ler os novos contatos.
O visualizador de animações continua sendo uma inspeção cinemática distinta
do teste com corpo dinâmico no laboratório.

### Apoios e footwork

Os pontos com impulso externo e normal favorável formam a região de apoio.
O COM e sua velocidade são calculados por massa; o indicador de captura é
uma aproximação de pêndulo invertido para apoiar decisões, não uma prova de
estabilidade do humano articulado. O observador trabalha com a geometria
presente relativamente à raiz física atual, sem guardar um destino mundial.

Os estados são `Supported`, `WeightShift`, `Swing`, `Touchdown` e
`Unsupported`. A escolha da passada possui limites de alcance, separação
lateral e tempo; transferência precisa de carga medida na perna de apoio.
Contato ausente não pode deixar uma busca ou espera infinita dentro do tick.
O alvo do pé livre é uma entrada para IK articular, não uma força direta para
arrastar o pé até uma posição global.

O perfil do dummy habilita sensores nos 18 links. O backend exclui
autocontato do próprio ragdoll da telemetria
de apoio; as colisões internas continuam existindo fisicamente. Mão no chão
ajuda a reconhecer acomodação após queda, mas não conta como pé de apoio.

### Animações e músculos

Os papéis vêm de `assets/characters/football_player/character.json`:

| Papel | Clipe |
| --- | --- |
| Idle | `idle_standard_dummy` |
| Corrida/freada | `run_forward_dummy`, `run_to_stop_dummy` |
| Caminhada frente/trás | `walk_forward_dummy`, `walk_backward_dummy` |
| Caminhada esquerda/direita | `walk_strafe_left_dummy`, `walk_strafe_right_dummy` |
| Corrida para trás | `run_backward_dummy` |
| Levantar de frente/costas | `stand_up_front_dummy`, `stand_up_back_dummy` |

O carregador admite papéis novos opcionais. O contrato atual de compatibilidade
do controlador ainda exige a base Idle/corrida/freada; não assumir que um
manifesto somente com caminhada habilita todos os comandos. Clipes são
validados ao carregar o catálogo. A compatibilidade do conjunto é armazenada
para a interface não percorrer todos os keyframes a cada frame.

Há seleção cardinal de caminhada e adaptação corporal da passada. O casamento
de fase usa uma estimativa dos picos de elevação de cada pé do clipe. Isso
ainda precisa ser substituído/refinado por fases semânticas verificadas de
apoio e balanço. **Mistura diagonal robusta e corrida com fase aérea não estão
concluídas.** O papel `runBackward` já é resolvido, mas não constitui comando
de corrida para trás implementado no laboratório.

Os alvos são limitados nas juntas e em taxa angular; IK tem trabalho limitado
por iteração. A reação de quadris/tornozelos e os ganhos musculares continuam
em calibração. O backend usa um envelope de atuação para incluir drive e
esforço interno no orçamento do motor. A compensação gravitacional descarta
os seis termos de base flutuante: não aplica uma sustentação externa oculta
na raiz. Medir saturação, esforço e estabilidade continua necessário.

Na revisão final, drives usam `eFORCE`: ganho do perfil × escala ×5 para
stiffness e ×2 para damping. O antigo `eACCELERATION` dimensionava ganho pela
inércia livre, subdimensionando o tornozelo apoiado. Músculo padrão 1,8×,
2,34× ao levantar, com slider de 0,5–3×; isso melhorou esforço, não solucionou
equilíbrio. Torque transmitido observado inclui reação dos limites e não
deve ser chamado de torque isolado do motor.

### Queda e recuperação

`Falling` deixa de ser terminal. O fluxo novo distingue queda, acomodação no
chão, tentativa de levantar de frente/costas e retorno ao Idle. A seleção
usa a direção anterior do tronco relativamente à gravidade, não o yaw da
pelve quando ela está deitada. Orientação lateral ambígua não é inventada
como frente ou costas.

A tentativa exige contato externo e acomodação, desacelera o relógio se o
erro articular estiver alto e tem timeout e número limitado de tentativas.
Terminar o clipe não é sucesso: a conclusão exige postura, altura, apoio e
velocidade física adequados mantidos por um intervalo. Não há rotação ou
translação livre da raiz aplicada do clipe ao corpo. A força secundária mais
forte permitida pelo usuário para levantar ainda não constitui uma solução
calibrada; recuperação física precisa de ensaios próprios.

Último incremento: modo `recoveryMode` explícito, orçamento de força padrão
45%/teto 55% do peso (normal até 15%), torque configurado 15%/teto 20% de
peso×altura. Feedforward vertical até 25% da gravidade, somente com contato e
trecho de subida, suavizado e limitado a 4 s acumulados por tentativa; entra
no mesmo orçamento, sem perseguir altura mundial. Sliders expõem os valores.

## Integração e ensaios

O painel do ragdoll oferece `CAMINHAR POR 15 SEGUNDOS` e mantém a corrida de
5 segundos como experimental. Os comandos não reiniciam uma ação já ativa.
São exibidos fase, tempo restante, apoio nos pés, erro de captura, erro
articular, assistência e tentativas de recuperação. A Physgun passa ao
controlador o índice físico do membro agarrado.

Autostart de diagnóstico: `MATTERENGINE_AUTOSTART=laboratory-walk-smoke`
solicita caminhada após 2 segundos e fecha aproximadamente aos 22 segundos.
O sucesso de abrir/fechar esse smoke não aprova o movimento observado.

Os gates separados são:

- `MatterEngine.ContactFootwork`: geometria, referenciais, contatos,
  transferência, limites e timeouts do planejador.
- `MatterEngine.ActiveRagdollContracts`: comandos limitados, invariância,
  Physgun reativa, seleção de recuperação e contratos de estado.
- `MatterEngine.ActiveRagdollStanding`: postura física por 30 segundos.
- `MatterEngine.ActiveRagdollWalking`: acomodação, comando de 15 segundos e
  parada, com progresso real, limites de queda/deriva e juntas coesas.
- `MatterEngine.BodyRelativeAssistance` e `MatterEngine.AnimatedRagdoll`:
  regressões anteriores, sem afrouxar a corrida para ocultar falhas físicas.

Build nativo e suíte completa continuam pelo procedimento de `AGENTS.md`.
Scripts Linux e Windows incluem os executáveis novos. Testes de contrato
aprovados não equivalem a passar os gates físicos ou à aprovação visual.

## Estado e próxima etapa

Base estrutural implementada, mas ensaios finais ainda mostram **deriva
espontânea em Idle e queda**. Build otimizado final passou; bateria direcionada
4/6, reprovando postura e caminhada (~2,13 s ativos antes da queda). Marco
Debug anterior ao último reforço: 7/10, falhando também corrida. Recuperação
segue sem sucesso comprovado. Logs e números estão no handoff e no documento
de desenvolvimento. Não houve smoke visual após o último reforço/sliders.
Não anunciar locomoção natural concluída.

Prioridade: estabilizar apoio bilateral e extensão das pernas com resposta
limitada a perturbações; depois transferência de carga e passo reativo; só
então caminhada de 15 segundos/parada e recuperação frente/costas. Validar
trás, laterais, diagonais, corrida e outros terrenos separadamente. Preservar
os gates de ausência de ancoragem, queda livre, impacto e limites articulares
durante toda calibração. Inspeção da sola e reação à Physgun continuam parte
obrigatória da aprovação humana.
