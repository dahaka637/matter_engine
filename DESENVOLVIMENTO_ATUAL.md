# MatterEngine — estado atual do desenvolvimento

> Documento vivo de continuidade técnica. Deve ser atualizado sempre que uma
> etapa material for iniciada, concluída, alterada ou bloqueada. A intenção é
> registrar não apenas *o que* mudou, mas também *por que*, como foi validado e
> quais limitações ainda existem.

**Última atualização:** 15 de setembro de 2026  
**Foco atual:** corrigir o contrato da assistência corporal antes de validar
Idle → corrida → freada. A locomoção completa ainda não está pronta.

## Atualização de 15 de setembro — correção conceitual exigida pelo usuário

- Forças auxiliares podem atuar em XYZ, **inclusive verticalmente**. O que não
  podem fazer é perseguir uma posição mundial salva. Rotacionar esse erro
  para eixos locais não elimina a ancoragem. A proibição de Z foi descartada.
- O protótipo de forças fortes por link/trajetória e a tentativa seguinte de
  auxílio só horizontal estão arquivados; não são bases aprovadas.
- `BodyRelativeRagdollAssist3D` separa auxílio de pose corporal, intenção de
  velocidade e pequeno torque de equilíbrio. É uma rotina sem estado e sem
  parâmetro de destino mundial. Pose e derivadas são construídas diretamente
  no referencial da pelve física atual, sem histórico de alvos mundiais.
- O auxílio de pose remove resultantes de força/momento; o de movimento pode
  produzir resultante XYZ explícita. Orçamento global inclui todos os links
  e reações. Isso impede que várias ajudas pequenas escondam uma grande força.
- Raiz física atual segue autoritativa; origem do comando é só telemetria.
  Physgun suspende também feedforward interno e reinicia na pose/local atuais.
  Queda suspende auxílio e reduz rigidez. Ajustes experimentais de tornozelo/
  tronco foram retirados, sem reativar WBC/footwork anteriores.
- Interface identifica auxílio XYZ e mostra resultante, força/torque totais
  e estado de queda; não afirma mais uma proibição de sustentação vertical.

Referência obrigatória: `docs/BODY_RELATIVE_ANIMATION_CONTROL.md` e nota
`Body_Relative_Animation_Control.md` no brain. Estudo de SIMBICON/DeepMimic
orientou a separação de objetivos, não constitui uma implementação desses
controladores ou prova automática de qualidade.

Validação: builds Debug e RelWithDebInfo, checagem de arquitetura e smoke de
abertura/fechamento do laboratório passaram. Os contratos de assistência XYZ
passaram, incluindo mudança de histórico de posição durante corrida/freada,
soltura, transporte rígido, forças/momentos e orçamento global. Caixa de 25 kg
a 8 m/s contra a canela deslocou o COM 0,368 m frente ao controle e derrubou o
boneco (up ≈ -0,002; controle ≈ 0,964). Sem chão, o COM caiu 1,238 m em 0,5 s:
o auxílio de pose/feedforward interno não estava sustentando o peso no ar.

A suíte Debug teve **4/5 testes aprovados**: Foundation, BodyRelativeAssistance,
Character e Audio passaram; **AnimatedRagdoll reprovou** porque a sequência
física não se completa (queda logo após começar a corrida). O smoke também
mostrou essa queda. Não reduzir esse critério nem restaurar ancoragem para
ocultá-la. Idle prolongado, corrida completa, adaptação da passada à velocidade/
apoios reais e pé torto continuam pendentes. Não anunciar pronto para jogar.

A camada de validação Vulkan não está instalada neste ambiente; o smoke foi
funcional, não uma nova certificação da validação gráfica de 10/09. Não houve
alterações Vulkan nesta etapa.

## Atualização de 14 de setembro — animação física assistida (em validação)

Registro histórico: a assistência deste protótipo foi rejeitada pelo usuário;
os resultados de completar a trajetória não aprovam seu conceito de controle.

- Idle (60 frames/1,967 s) e Run To Stop (28 frames/0,9 s) importados para
  CrashTestDummyV1, com todos os gates aprovados. Freada é one-shot e conserva
  a trajetória fonte; Idle e corrida são ciclos in-place.
- Os módulos anteriores e seus testes/consumidores foram preservados em
  `archive/2026-09-14-footwork-balance/`, fora dos alvos CMake e do runtime.
  A falha antiga do WBC não foi resolvida: seu sistema foi aposentado.
- `AnimatedRagdollController3D` substitui o controlador no Workbench. Alvos
  angulares continuam limitados pelo perfil; PhysX continua autoritativo.
  Assistência distribuída é limitada e sua força aparece na interface.
- Botão `CORRER POR 5 SEGUNDOS`; direção capturada da orientação corporal.
  Freada usa a curva de deslocamento do FBX, com tempo ajustado à velocidade
  de entrada. Não se arrasta o boneco a velocidade constante durante a parada.
- Physgun suspende motores/assistência; ao soltar, o alvo retorna ao Idle na
  localização atual. Obstáculo frontal/erro excessivo interrompe a corrida.
- Testes dedicados já exercitam três direções, sequência completa, botão
  repetido, assistência desligada, suspensão pela Physgun e juntas coesas.
  Smoke interativo, critérios finais e documentação detalhada em conclusão.

Limite deliberado: primeiro teste assistido em piso plano, sem novo footwork
procedural, navegação de escadas ou recuperação biomecânica autônoma.

## Atualização de 10 de setembro — personagem rigado e renderer

O usuário substituiu a primeira fonte low-poly estática pelo pacote
`Crash Test Dummy mark1.zip`. A versão Unity/Mecanim já possui pesos, UV e
63 ossos; a adaptação preserva esses dados e divide o peito, produzindo 64
ossos visuais na fonte Blender e 18 links/41 DOF no perfil físico.

- Novo manifesto `assets/characters/crash_test_dummy/character.json` e
  `CrashTestDummyV1.ragdoll.json`, com proporções/pivôs próprios, raios por link
  e os limites articulares herdados do contrato humano.
- `RagdollCharacter3D` valida bind, pesos e mapeamento; o renderer usa paletas
  atual/anterior na GPU em cor, profundidade e sombras. A pele segue a pose
  física, sem alterar a ordem fixa de controle/simulação.
- Preview e spawn compartilham a definição; `MATTERENGINE_CHARACTER` seleciona
  outros manifestos. Envelope de spawn é derivado da malha e dos colisores.
- Fontes Blender/GLB, textura, licença e scripts de preparação/exportação
  reproduzíveis acompanham o asset. A inspeção procedural agora mostra os
  colisores reais, como modo de diagnóstico.
- Corrida reimportada para o novo perfil: RMS 4,12°, máximo global 9,96°,
  máximo dos membros 6,02°. O espelhamento de edição do Blender foi desativado
  durante o bake após os gates detectarem uma inversão de lados.
- A validação Vulkan revelou e motivou correções de recursos já existentes:
  semáforos de apresentação por imagem do swapchain, descritores de sprites
  por frame, habilitação explícita das features usadas pelos shaders e
  separação entre layout e inicialização do conteúdo da exposição automática.

Validação: build Debug/RelWithDebInfo, arquitetura e testes do personagem
passaram. Cobertura inclui bind, corrida, 22 corpos em contato (separação
máxima dos anchors de 0,000000985 m), Physgun e seis segundos de postura ativa
sem força auxiliar. A reexportação do Blender preparado reproduziu a pele
canônica byte a byte. Preview, spawnmenu, spawn, resize, minimizar/restaurar,
tela cheia e fechamento foram exercitados; os smokes com a camada Vulkan
temporária não registraram erros, somente avisos de atributos não consumidos.
A camada foi extraída em `/tmp`, não instalada no sistema.

A suíte Debug geral teve 2/3 testes aprovados: Character e Audio passaram;
Foundation reproduziu a regressão histórica de recuperação da rajada esquerda
do `HumanAdultV1`. Não foi alterado o teste para esconder essa falha.
O spawn aéreo junto à parede também pode terminar com o dummy caído: a troca
de pele não implementa recuperação física automática de quedas. Aprovação
visual final permanece com o usuário.

Corrigida ainda a margem divergente entre busca de espaço de spawn (25 mm)
e checagem final (35 mm); o spawn humano iguala essas margens e deriva a altura
de colocação no chão do envelope real da malha/colisores.

Metodologia completa: `docs/RAGDOLL_CHARACTER_PIPELINE.md`. A primeira fonte
low-poly permanece apenas como protótipo alternativo. Dedos ainda acompanham
mãos/pés no runtime, e a corrida continua sendo playback cinemático de preview,
sem prometer locomoção física já implementada.

## Atualização de 9 de setembro — retarget físico validado

A primeira conversão visualmente plausível não foi aceita como solução final.
O pipeline foi reformulado para que todo clipe seja convertido e validado no
espaço real das juntas do ragdoll, sem correções particulares para a corrida.

- O formato canônico agora é `matter-ragdoll-animation-1`: apenas a raiz tem
  transformação livre; os outros 17 canais armazenam diretamente
  `twist/swing1/swing2` em radianos.
- A geometria dos segmentos do FBX, e não o *bone roll*, dirige um IK angular
  limitado para braços e pernas. Cotovelos e joelhos recebem sua flexão no
  hinge físico correto.
- Os frames dos cotovelos no `HumanAdultV1` foram corrigidos e espelhados. Um
  teste anatômico prova que a mesma flexão positiva dobra ambos os antebraços
  para a direção corporal equivalente.
- Filhos são sempre reconstruídos pela coincidência dos anchors pai/filho;
  interpolação e playback reaplicam limites como defesa adicional.
- O importador falha antes de gravar quando reprova erro direcional, erro dos
  membros, salto angular, fechamento das juntas ou fechamento da raiz.
- O Workbench recusa clipes sem relatório aprovado ou incompatíveis com o
  perfil físico. O visualizador exibe o estado `RETARGET VALIDADO` e as
  métricas principais.

Na corrida atual, os gates medem RMS direcional de `4,67°`, máximo global de
`13,17°`, máximo nos membros de `6,02°`, maior passo de `36,60°`, fechamento
articular de `0,00014°` e fechamento translacional da raiz abaixo da precisão
registrada. Todos ficam dentro dos limites documentados em
`docs/ANIMATION_RETARGET_PIPELINE.md`.

Validação final: build `RelWithDebInfo`, suíte filtrada `animation` e
`tools/check-architecture.sh` passaram.

## Histórico de 8 de setembro — primeira corrida retargeteada

### Fonte analisada

- `/home/dahaka/Downloads/Running.fbx`: FBX Binary 7700 do Mixamo, com um
  armature de 65 ossos, 20 quadros a 30 fps e duração de `0,633 s`.
- Embora baixado como corrida para visualização, o arquivo possui cerca de
  `3,50 m` de avanço da raiz durante o ciclo. O pipeline detecta e retira essa
  deriva; não foi necessário pedir outro download.

### Implementação

- `tools/import_mixamo_animation.py` usa o importador FBX do Blender para
  mapear 18 ossos Mixamo nos 18 links do `HumanAdultV1`, converter base e
  escala e gerar o formato físico versionado
  `matter-ragdoll-animation-1`.
- A deriva linear completa da pelve é removida entre o primeiro e o último
  quadro. Isso mantém o personagem centralizado, preserva balanço/oscilação da
  passada e também evita vazamento do avanço para o eixo vertical.
- O resultado está em
  `assets/animations/clips/run_forward.matteranim.json`, com nome exibido
  `Correr para frente`, 18 canais e os 20 quadros originais a 30 fps.
- `loadAnimationClip3D` desserializa e valida o contrato canônico. Ao iniciar,
  o Workbench enumera `assets/animations/clips`, ordena e carrega os clipes
  compatíveis com `HumanAdultV1`; FBX e Blender não entram no runtime.
- O visualizador agora reproduz a corrida sobre as peças reais do ragdoll.
  Arrastar com o botão esquerdo sobre o viewport orbita a câmera, a roda do
  mouse controla zoom e `Redefinir câmera` restaura o enquadramento inicial.
- `assets/animations/README.md` documenta o comando reproduzível para importar
  os próximos FBXs do Mixamo.

### Validação

- Build `RelWithDebInfo` de aplicativo e testes: passou.
- `MATTERENGINE_TEST_FILTER=animation ./build-profile/MatterEngineTests`:
  passou; além da interpolação anterior, abre o clipe real, verifica rig,
  metadados, 18 canais e ausência de deriva ao fechar o ciclo.
- `./tools/check-architecture.sh`: passou.
- Smoke `MATTERENGINE_AUTOSTART=animation-viewer-smoke`: catálogo carregou um
  clipe, Vulkan renderizou a corrida por 6 segundos e encerrou normalmente.
- Inspeção visual: pose de corrida articulada, personagem centralizado, chão,
  timeline, metadados e instruções de câmera apareceram corretamente.

### Limite atual

O playback desta etapa é cinemático e exclusivo do visualizador, conforme o
objetivo atual. Ele ainda não conduz o ragdoll dinâmico por torques nem se
mistura ao equilíbrio/footwork; essa integração permanece para a etapa híbrida
posterior.

### Correção após a primeira inspeção interativa

A primeira versão do preview estava incorreta: calculava a posição de cada
filho rotacionando o vetor entre centros das peças. Como esses centros não são
as âncoras da articulation, ombros, cotovelos, quadris e joelhos se separavam
visualmente durante a corrida.

A pose agora usa a mesma equação de junta do PhysX: calcula a âncora mundial no
pai e posiciona o filho de modo que sua âncora local coincida exatamente com
ela. Antes disso, a rotação desejada é convertida para o vetor exponencial
`twist/swing1/swing2`, eixos bloqueados são zerados e cada componente é limitada
pelo intervalo do `HumanAdultV1`. O importador grava keyframes já projetados e
o runtime repete a projeção depois da interpolação.

O teste de animação percorre 65 instantes do ciclo e verifica duas invariantes
para todos os 17 vínculos: distância entre âncoras menor que `0,01 mm` e todo
alvo angular — tanto no clipe quanto na pose final — dentro dos limites do
perfil. Uma inspeção visual adicional em quatro fases da passada confirmou a
cadeia corporal contínua.

Uma segunda inspeção revelou cotovelos retos apesar de o FBX possuir flexão de
`63–122°`. O problema não era somente o *bone roll* do Mixamo: os frames dos
cotovelos no perfil físico apontavam o hinge para um plano anatomicamente
incorreto. Os frames foram corrigidos e espelhados, e braços/pernas passaram a
usar IK angular limitado pelas direções geométricas dos segmentos. Assim a
solução pertence ao rig e ao adaptador semântico, não a números mágicos deste
clipe.

---

## Histórico de 8 de setembro — fundação do Visualizador de Animações

### Objetivo desta etapa

Criar no Workbench o lugar em que as animações futuras serão selecionadas,
reproduzidas em loop e conferidas diretamente sobre o rig físico
`HumanAdultV1`, sem iniciar ainda importação, retargeting, forças articulares,
assistência física ou integração com footwork.

### O que foi implementado

- O menu principal agora separa `Objetos` de uma nova entrada `Animações`.
- A tela `AnimationViewer` possui biblioteca lateral, viewport 3D, indicação
  do rig alvo, controles de reproduzir/reiniciar/loop/velocidade e timeline.
- Como nenhum arquivo foi fornecido ainda, a biblioteca começa honestamente
  vazia, os controles ficam desativados e o viewport mostra o mesmo rig
  `HumanAdultV1` de 18 links/41 DOF em pose de referência.
- O preview reutiliza as malhas procedurais e o perfil reais do ragdoll; não
  cria um esqueleto visual paralelo que poderia divergir da física.
- Foi criado `Engine/Animation/AnimationClip3D`: contrato neutro do formato de
  origem, com canais endereçados pelos IDs estáveis dos links do ragdoll,
  translação/rotação locais relativas à pose neutra, procedência do arquivo,
  validação, busca por canal, interpolação e repetição em loop.
- `assets/animations/README.md` registra a metodologia: cada futuro importador
  adapta seu formato de origem para `AnimationClip3D`; a UI e o futuro
  controlador físico consomem apenas esse formato canônico.
- Foi adicionado o autostart `animation-viewer[-smoke]` para validar a tela sem
  navegação manual.

### Validação desta etapa

- `./tools/check-architecture.sh`: passou.
- Build `RelWithDebInfo` de `MatterEngineApp`: passou com GCC 16.1.1.
- Smoke `MATTERENGINE_AUTOSTART=animation-viewer-smoke`: abriu Vulkan 1.4 na
  GTX 1070 Ti, renderizou o rig e encerrou normalmente após 6 segundos.
- Inspeção visual local: biblioteca vazia, viewport, pose neutra, timeline e
  estado desativado dos controles apareceram corretamente.
- `MATTERENGINE_TEST_FILTER=animation ./build-linux/MatterEngineTests`:
  passou; cobre validação do clipe, busca de canal, interpolação de
  translação/rotação e wrap de loop.

### Limitações e próximo ponto de entrada

- Nenhum parser de FBX/BVH/glTF foi escolhido antes de existir um arquivo real;
  isso evita cristalizar prematuramente um pipeline incompatível com a fonte
  que será fornecida.
- Ainda não existe clipe no catálogo. A próxima etapa começa ao receber o
  primeiro arquivo: inspecionar esqueleto, unidade, eixos, pose-base e canais;
  então implementar o adaptador e o mapa para `HumanAdultV1`.
- O playback atual é somente cinemático no preview. Aplicar a pose alvo por
  torques e assistência sutil pertence à etapa posterior definida no TODO.
- A regressão já conhecida nos testes `active-core` de recuperação de impacto
  não foi alterada nem considerada resolvida por este trabalho de UI/animação.

---

## Histórico da etapa anterior — equilíbrio estacionário

As seções abaixo preservam o estado registrado em 2 de agosto de 2026. Elas
não substituem a atualização mais recente acima.

## Objetivo da etapa de equilíbrio (registro de 2 de agosto)

Construir uma base biomecânica em que o ragdoll ativo:

- mantenha uma postura bilateral firme quando parado;
- sustente a pose sem depender da força auxiliar na raiz ou na coluna;
- use tornozelos, joelhos, quadris, coluna e braços para contrariar
  perturbações;
- escolha o pé **externo** ao movimento numa recuperação lateral, evitando
  cruzar ou aproximar os pés de forma desestabilizadora;
- replante rapidamente uma sola que perdeu contato;
- use uma reação de proteção com joelhos e braços quando a queda já não puder
  ser recuperada;
- possa ser submetido a impactos reproduzíveis dentro do próprio Workbench.

O comando “caminhar 5 metros” continua existindo no projeto, mas foi
deliberadamente removido do perfil de validação desta etapa. Ele será retomado
somente depois de o equilíbrio estacionário ser aprovado visualmente.

## O que foi implementado

### 1. Controlador de equilíbrio e footwork reativo

- Assistência de bacia/coluna desativada por padrão. Os testes de impacto
  também falham caso detectem força auxiliar clandestina.
- Recuperação lateral seleciona a perna externa usando a velocidade inesperada
  do centro de massa, em vez de escolher automaticamente o pé interno que
  perdeu carga.
- A passada de recuperação possui transferência de peso, liberação física do
  apoio, alvo anticruzamento e replantio urgente quando a perna que deveria
  sustentar o corpo perde contato.
- A coluna recebe alvo dinâmico de contra-inclinação e os braços participam da
  compensação do momento corporal.
- Quando a recuperação normal deixa de ser viável, a proteção de queda abre os
  braços, estende os antebraços e flexiona os joelhos na direção provável de
  contato.
- A reparação de postura permanece ativa durante toda a vida do ragdoll: uma
  base estreita, uma sola suspensa ou a perda unilateral de apoio não podem se
  tornar a nova pose de repouso.

### 2. Whole-Body Controller

- O controle principal usa dinâmica inversa de corpo inteiro e tarefas de
  contato/COM/pé, preservando a ordem fixa de simulação em 120 Hz.
- A passada reativa é tratada como tarefa de locomoção mesmo sem comando de
  caminhada; isso evita que uma perna em voo arraste o corpo inteiro.
- Existe uma tarefa explícita de reaquisição de apoio para a sola interna que
  perde contato antes da saída do pé externo.
- Durante manipulação com a Physgun, apenas a cadeia da perna agarrada cede. O
  restante do corpo continua controlado, e o WBC não disputa o membro com o
  handle físico.
- Ao soltar a Physgun há uma pequena janela de amortecimento antes de autorizar
  uma passada reativa; isso impede que a velocidade residual do handle seja
  confundida com um novo impacto.

### 3. Laboratório de impactos no menu RAGDOLL

A aba `RAGDOLL` agora possui as subabas `CONTROLE` e `IMPULSOS`.

Em `IMPULSOS` há três modos:

- **Direto:** rajada curta com força, direção, duração e intervalo definidos.
- **Contínuo:** empurrão sustentado, semelhante a vento constante.
- **Aleatório:** sorteia direção, força, duração e intervalo dentro das faixas
  configuradas.

A direção é relativa à orientação do corpo:

- `0°`: frente;
- `90°`: esquerda;
- `180°`: trás;
- `270°`: direita.

Controles disponíveis: iniciar, pausar/continuar, aplicar uma rajada imediata,
parar e restaurar os valores padrão. O painel também mostra força/direção
atuais, contagem de eventos e tempo até o próximo evento.

A força não é aplicada como uma “mão invisível” na raiz. Ela é distribuída
fisicamente entre bacia, peito e parte superior do tórax.

### 4. Gerador reutilizável de testes

`RagdollImpactTest3D` é independente de ImGui, PhysX e Workbench. A mesma
sequência que alimenta a interface pode ser usada em testes automatizados,
inclusive com semente determinística no modo aleatório.

Cenários automatizados atuais, todos com assistência desativada:

- rajada lateral esquerda: `330 N` por `0,12 s`;
- rajada lateral direita: `330 N` por `0,12 s`;
- rajada frontal: `260 N` por `0,12 s`;
- força lateral contínua: `70 N` por `0,75 s`;
- sequência aleatória: faixa de `90–180 N` durante `6 s`;
- perturbações estacionárias em oito direções;
- reparação bilateral suave após deslocamento de uma perna pela Physgun.

As forças acima são pontos de partida técnicos, não valores finais de design.
O limite humano desejado será calibrado pelos testes visuais dentro da engine.

## Problemas encontrados e soluções aplicadas

### Pé interno iniciava a recuperação lateral

**Causa:** a seleção usava o ponto de captura em relação ao apoio remanescente.
Esse referencial podia apontar para o pé descarregado e gerar uma passada
cruzada.

**Solução:** selecionar a perna pela velocidade corporal não comandada. Em uma
queda para a esquerda, o pé esquerdo amplia a base; numa queda para a direita,
o pé direito faz o mesmo.

### Controlador esperava demais para reagir

**Causa:** limiares antigos só permitiam a passada quando o ponto de captura já
estava muito fora da base.

**Solução:** antecipação do gatilho, transferência curta em emergência,
redirecionamento do alvo durante a perturbação e replantio urgente da perna de
apoio caso ela perca contato.

### Membro agarrado pela Physgun derrubava o corpo inteiro

**Causa:** Physgun, tarefas cartesianas, preferências de pose e drives locais
disputavam simultaneamente a mesma cadeia articulada.

**Solução:** informar ao controlador exatamente qual link está sendo
manipulado, retirar a perna correspondente das tarefas/contato do WBC, zerar os
drives concorrentes dessa cadeia e manter o restante do corpo ativo.

### Teste antigo media um cenário fisicamente inválido

**Causa:** um teste prendia o tornozelo com capacidade de dezenas de kN,
arrastava-o através do corpo e interpretava qualquer queda como falha de
equilíbrio humano.

**Solução:** cruzamento extremo permanece coberto pelo planejador anticruzamento
e o teste físico da Physgun passou a usar uma perturbação de intensidade humana.

### Testes de caminhada contaminavam esta etapa

**Causa:** os perfis integrados ainda exigiam completar 5 metros, apesar da
decisão atual de focar somente postura e impactos.

**Solução:** o perfil `MATTERENGINE_WBC_BALANCE_ONLY=1` agora encerra após os
testes estacionários. Os testes de caminhada foram preservados no código para
serem retomados no ciclo correto.

### Um ragdoll ativo derrubava a execução interativa para cerca de 3 FPS

**Causa real:** o atalho da Área de Trabalho ainda executava diretamente
`build-linux/MatterEngine`, configurado como `Debug` puro (`-g`, sem otimização).
O controlador ativo calcula em 120 Hz a dinâmica articulada, matrizes densas e
um QP de corpo inteiro. Executar Eigen, ProxQP e essas rotinas sem otimização
transformava cada passo de controle em um gargalo extremo. Era uma divergência
entre o binário de testes internos e o binário apropriado para teste interativo,
não um custo aceitável do ragdoll.

**Solução:** foi criado o build oficial `build-profile` em
`RelWithDebInfo`, que preserva símbolos de diagnóstico e ativa otimizações. O
atalho agora chama `tools/run.sh`, que prioriza esse perfil. O Debug permanece
disponível somente para testes e depuração, não para avaliar FPS ou resposta do
controlador.

**Trade-off/pendência:** um ragdoll voltou a acompanhar o tempo real, mas a
escalabilidade do WBC para dezenas de personagens ainda precisa ser medida com
telemetria que inclua explicitamente o custo do controlador, e não apenas o
passo interno do PhysX.

## Validação executada

Última validação local concluída:

```text
Arquitetura: passou
Compilação Debug de testes: passou
Compilação RelWithDebInfo interativa: passou
MatterEngine.Foundation: passou (224,90 s)
MatterEngine.Audio: passou (0,08 s)
Smoke otimizado do Workbench com ragdoll ativo: passou em tempo real
```

Comandos usados:

```bash
./tools/check-architecture.sh
cmake --build build-linux -j 8
MATTERENGINE_WBC_BALANCE_ONLY=1 \
  ctest --test-dir build-linux --output-on-failure
cmake -S . -B build-profile -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMATTERENGINE_BUILD_TESTS=OFF
cmake --build build-profile --target MatterEngineApp -j 8
MATTERENGINE_AUTOSTART=laboratory-ragdoll-smoke \
  ./tools/run.sh
```

O smoke otimizado abriu Vulkan 1.4 na GTX 1070 Ti, criou o ragdoll ativo,
encerrou limpamente e completou 10 segundos de simulação em `10,35 s` reais.
O contador amostrado no fim indicou aproximadamente `829 FPS`, mas essa medida
isolada **não é aceita como custo total do ragdoll**, pois o quadro capturado
não necessariamente continha um passo fixo de controle. A validação anterior
que citava aproximadamente 700 FPS no Debug foi invalidada: o mesmo smoke
levava mais de 40 segundos reais para avançar apenas 10 segundos simulados.

O atalho da Área de Trabalho executa:

```text
/home/dahaka/Projetos/matter_engine/tools/run.sh
```

O launcher seleciona `/home/dahaka/Projetos/matter_engine/build-profile/MatterEngine`,
o mesmo binário otimizado recompilado e validado acima.

## Como testar manualmente agora

1. Abra `MatterEngine` pelo atalho da Área de Trabalho.
2. Crie um ragdoll ativo.
3. Abra `RAGDOLL > IMPULSOS`.
4. Comece com uma rajada lateral de `330 N`, `0,12 s`, em `90°`.
5. Observe se o pé esquerdo (externo) reage primeiro, se ambas as solas voltam
   ao chão e se a coluna/braços contrariam o movimento.
6. Repita em `270°` e confirme que o pé direito reage primeiro.
7. Teste o modo contínuo em `70 N` e depois o aleatório entre `90–180 N`.
8. Anote o menor valor que expõe uma falha e o maior valor em que a recuperação
   ainda parece humana.

## Pendências e dificuldades conhecidas

- A aprovação visual ainda depende do teste manual do usuário; passar nos
  testes numéricos não garante movimento natural.
- O contador de FPS do smoke ainda não separa nem acumula explicitamente o
  custo do WBC. Antes de otimizar múltiplos ragdolls, adicionar telemetria do
  controlador e medir percentis de tempo por passo fixo.
- O footwork estacionário está aprovado apenas para os cenários automatizados
  atuais. Mudanças bruscas, contatos com props e terrenos inclinados ainda
  precisam de uma matriz própria de testes.
- A proteção de queda existe, mas ainda precisa ser avaliada visualmente para
  evitar poses artificiais dos braços.
- A manipulação extrema e prolongada de um membro pela Physgun pode derrubar o
  ragdoll, como ocorreria fisicamente. O objetivo atual é não criar explosões
  nem controladores concorrentes, e reconstruir a postura depois de uma
  perturbação razoável — não auto-levantar do chão.
- A caminhada de 5 metros, corrida e transição equilíbrio-locomoção estão fora
  do escopo atual e continuam pendentes.
- Os valores de força aprovados em laboratório ainda precisam virar critérios
  formais por categoria: leve, moderado, severo e irrecuperável.

## Próximas etapas previstas

1. Receber feedback visual do laboratório de impactos.
2. Corrigir poses, escolha de pé, tempo de reação ou replantio apontados pelo
   teste manual.
3. Definir os limiares mínimos de resistência a impacto que serão bloqueadores
   de regressão.
4. Ampliar a matriz estacionária para contatos com props e solo inclinado.
5. Somente após aprovação, retomar caminhada/locomoção e integrar o footwork ao
   ciclo de passos sem usar a assistência como muleta.

## Mapa dos arquivos principais desta etapa

- `src/Engine/Control/ActiveRagdollController3D.*` — orquestra equilíbrio,
  footwork reativo, postura e proteção de queda.
- `src/Engine/Control/NaturalBalanceSystem3D.*` — estima COM, apoio, ponto de
  captura e urgência.
- `src/Engine/Control/WholeBodyController3D.*` — dinâmica inversa, contato,
  tarefas do corpo e torques articulares.
- `src/Engine/Control/RagdollImpactTest3D.*` — gerador independente de impactos.
- `src/Workbench/Laboratory/RagdollRuntime.*` — aplica os eventos ao ragdoll no
  laboratório.
- `src/Workbench/Screens/LaboratoryScreen.cpp` — interface `RAGDOLL > IMPULSOS`.
- `tests/EngineFoundationTests.cpp` — regressões físicas e cenários de impacto.

## Regra de manutenção deste documento

Ao concluir qualquer mudança relevante, atualizar no mínimo:

1. data e foco atual;
2. o que foi implementado ou alterado;
3. problema encontrado e causa real;
4. solução aplicada e trade-offs;
5. testes executados e resultado;
6. pendências, limitações e próximo ponto de entrada.
