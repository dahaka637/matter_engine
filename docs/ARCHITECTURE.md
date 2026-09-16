# Arquitetura da MatterEngine

## Grafo de dependências

```text
MatterEngineApp
  -> MatterWorkbench
       -> MatterEngine
            |-> MatterCore
            |-> MatterRHI
            |-> MatterPhysics
            |    `-> backend privado PhysX
            `-> MatterVulkan
                 |-> MatterCore
                 `-> MatterRHI
```

Dependências apontam apenas para baixo. `src/Engine` não conhece Workbench;
`src/Workbench` não acessa SDL, Vulkan, PhysX ou tipos nativos de plataforma.
As APIs públicas de RHI e física são neutras quanto ao backend.

## Core e ciclo de vida

`Application` possui plataforma, janela, renderer, input e loop. A simulação é
executada em passos fixos de 120 Hz. Um limite de recuperação descarta backlog
excessivo após stalls e impede a *spiral of death*.

Ordem de desligamento:

```text
Workbench libera cenas, atores e recursos próprios
  -> Render2D e ImGui
  -> backend Vulkan
  -> janela SDL
  -> subsistemas SDL
```

## Sistema de tarefas

`TaskScheduler` é o pool central do processo. Workers persistentes possuem
filas locais LIFO e roubam trabalho antigo das demais filas quando ficam
ociosos. Submissões externas entram por uma fila de injeção; `TaskGroup` e
`parallelFor` fornecem barreiras sem criar threads durante o frame. A thread
que aguarda também pode executar trabalho disponível.

O pool é injetado nas fachadas que paralelizam trabalho. Middlewares não criam
pools privados: o backend PhysX implementa `PxCpuDispatcher` sobre o mesmo
scheduler, evitando oversubscription e disputa de cache.

## RHI e Vulkan

Headers públicos da RHI expõem apenas descritores e handles geracionais. Todo
tipo `Vk*`, Volk e VMA permanece em `Engine/RHI/Vulkan`.

O backend exige Vulkan 1.4 e usa dynamic rendering, synchronization2, dois
frames em voo, fence por imagem da swapchain e VMA. Resize, fullscreen,
minimização e `OUT_OF_DATE` recriam a swapchain pelo mesmo caminho controlado.

## Renderização

`Renderer` é a fachada da engine. O Workbench submete `Scene3DFrame`, formado
por câmera, luz e spans de renderables neutros. Meshes glTF são enviadas uma vez
para buffers de vértice/índice e reutilizadas por frame.

O caminho atual possui depth buffer, shadow map direcional, céu procedural e
materiais metallic/roughness básicos. Instâncias com a mesma geometria e
texturas são agrupadas em um draw indexado instanciado. Frustum culling da
câmera e da luz é preparado em paralelo; props invisíveis não entram no passe
correspondente. Passes de sombra desativados são omitidos após a inicialização.
PBR completo, streaming e Render Graph continuam como trabalho posterior.

O TAA usa jitter Halton, vetores de movimento por objeto e dois históricos HDR.
Cada caminho possui descriptor sets imutáveis por frame em voo; nenhum binding
referenciado por uma submissão pendente é reescrito. Histórico é opt-in e só se
torna válido após um frame realmente gravado, sendo descartado em resize, troca
de tela, camera cut ou retomada. O céu usa a VP inversa sem jitter. A geometria
é rasterizada com a VP jitterada, mas seus vetores de movimento com as VPs
estáveis atual/anterior: o jitter amostra subpixels e nunca entra na
reprojeção do histórico. Transforms anteriores avançam na cadência de
renderização, independentemente dos passos físicos de 120 Hz.

Cada frame Vulkan publica timestamps da GPU e tempos CPU de fence, acquire e
present por uma estrutura neutra da RHI. O loop mede eventos, fixed update,
preparação/render e UI. O painel `Performance` do laboratório expõe tudo sem
depender apenas do contador de FPS.

## Física

### Limite do backend

`MatterPhysics` expõe `PhysicsEngine3D`, `PhysicsScene3D`, descritores, eventos e
handles geracionais. Nenhum `Px*`, ponteiro nativo ou header do SDK atravessa
`Engine/Physics/PhysX`. O backend está fixado no commit oficial da versão
PhysX 5.9.0; builds existentes não mudam quando o repositório remoto avança.

`PhysicsEngine3D` possui Foundation, SDK e recursos cozidos. Cada
`PhysicsScene3D` possui dispatcher, cena, materiais, CCT, juntas e atores. A
ordem de destruição é determinística e o PIMPL da cena também libera recursos
se sua construção falhar pela metade.

### Passo de simulação

Todas as grandezas usam metros, quilogramas e segundos. A ordem invariável do
tick de 120 Hz é:

```text
controladores enviam forças, torques, comandos e alvos
  -> forças externas, como arrasto do ar, são acumuladas
  -> PhysX simula contatos, restrições e ilhas
  -> fetchResults finaliza o passo
  -> áudio e gameplay leem eventos/snapshots
  -> renderer consome poses concluídas
```

Controladores nunca corrigem poses de rigid bodies diretamente. A Physgun usa
um `PxD6Joint` privado com drives de mola-amortecedor limitados por força e
torque. O corpo agarrado continua dinâmico e precisa obedecer aos contatos.

### Configuração de desempenho

A cena usa solver TGS em 4/1 iterações, broad phase ABP, manifolds persistentes,
CCD seletivo, estabilização, ilhas e sleeping nativos. O dispatcher compartilha
o job system da engine, mas executa a cadeia PhysX diretamente na thread
chamadora quando a cena é pequena: acordar workers para tarefas minúsculas
custava mais do que resolvê-las. A quantidade de workers cresce somente com
DOFs ativos, atores e contatos. Um scratch buffer alinhado e reutilizado evita
alocações temporárias recorrentes no solver.

Cada humano é uma reduced-coordinate articulation dentro de um `PxAggregate`.
Assim, seus 18 links ocupam uma entrada coerente no broad phase em vez de 18
proxies independentes. Self-collision não é integral nem globalmente
desativada: cada shape carrega uma máscara anatômica imutável. Pai/filho e
é descartado diretamente no filter shader; todos os demais pares continuam
ativos. Isso é necessário porque o PhysX sempre elimina contato entre links
diretamente conectados: o `Chest`, avô do `UpperArm`, torna-se a barreira que
impede o braço de atravessar o tórax pai `UpperChest`. Membros irmãos e lados
diferentes também colidem. Colisões entre jogadores, bola, props e mundo
continuam normais.

Cada link limita a velocidade que o solver pode introduzir para corrigir
penetração e o impulso máximo de um único contato. Durante manipulação, o D6 da
Physgun limita força por aceleração proporcional à massa do membro; os limites
globais altos continuam disponíveis para props pesados, sem serem aplicados a
um antebraço leve. Speculative CCD nos links antecipa contato pela velocidade
linear/angular das cápsulas, evitando que membros rápidos atravessem uns aos
outros entre dois ticks sem impor swept CCD completo ao esqueleto.

CCD completo não é o padrão global. Objetos comuns usam detecção discreta a
120 Hz; a bola e corpos explicitamente pequenos/rápidos ativam CCD. O filtro
pede `eDETECT_CCD_CONTACT` somente quando ao menos um corpo do par exige isso.

Sleeping é parte do estado autoritativo: corpos em equilíbrio deixam de ser
simulados até contato, força, junta ou ação explícita despertá-los. A engine não
executa um solver paralelo próprio nem reconstrói colisores no spawn. Active
actors alimentam arrasto e snapshots incrementais, então a renderização não
consulta novamente todos os corpos adormecidos. Estatísticas
de corpos ativos, sleeping, pares discretos, CCD, tarefas, callbacks e fases
do passo ficam disponíveis em `PhysicsStepDiagnostics3D`. Streams detalhados
de contato são pedidos apenas para corpos que alimentam áudio; links de
ragdoll não criam strings, extração de pontos ou eventos descartados.

O perfil atual é CPU estático. GPU rigid bodies exigem CUDA, buffers e limites
de cena diferentes; serão um perfil separado apenas se medições representativas
provarem vantagem. Isso evita transformar hardware NVIDIA em requisito para o
runtime básico.

### Representação de colisão

O mapa estático é cozido uma vez como `PxTriangleMesh` com midphase BVH34. Antes
do cooking, arestas acima de 50 metros são subdivididas pela maior aresta. A
tesselação não desloca a superfície e evita triângulos gigantes que prejudicam
a estabilidade de contatos no PhysX. Essa representação é apropriada porque o
ator nunca se move.

Corpos dinâmicos nunca recebem triangle meshes côncavas. Eles usam:

- box, sphere ou capsule analíticas quando declaradas;
- um hull convexo;
- um compound com pequeno número de hulls convexos.

No modo automático, V-HACD converte a malha visual em até 16 hulls, com até 64
vértices por hull. A ferramenta offline grava um cache versionado e vinculado ao
hash da geometria; o runtime o valida antes de criar as meshes PhysX. As meshes
cozidas imutáveis são compartilhadas por todas as instâncias. Colliders
autorados aceitam até 32 partes e 64 shapes finais; ultrapassar o orçamento é
erro de importação, nunca degradação silenciosa.

Assets automáticos podem reduzir o budget por `collision_hulls`. O catálogo usa
budgets por forma e reserva o padrão de 16 apenas para objetos ainda não
calibrados. Primitivas analíticas continuam preferidas quando são verdadeiras.

A complexidade visual é independente desse contrato. Props com malhas densas
geram dois index buffers simplificados no carregamento, compartilhando o mesmo
vertex buffer e materiais. O render escolhe LOD por distância relativa ao raio
do objeto, enquanto depth de sombra usa o proxy mais compacto. Isso mantém a
malha-fonte em close e impede que detalhes subpixel sejam repetidos nas quatro
cascatas; o frame nunca simplifica geometria.

Essa separação é essencial: triangle mesh dinâmica preserva cada detalhe, mas
produz narrow phase caro e contatos instáveis. Compounds convexos preservam
assentos, vãos e apoios relevantes com custo limitado. Objetos que exigem uma
tolerância específica devem receber colliders autorados no Blender.

### Personagem

O personagem usa o Capsule Controller oficial do PhysX. O Workbench entrega
somente `CharacterMotorCommand3D`; o módulo físico calcula aceleração, gravidade,
pulo, coyote time, degraus, rampas e contatos. Agachar preserva os pés. Levantar
exige overlap livre para a cápsula normal.

O voo não possui inércia enquanto ativo e filtra todas as colisões para funcionar
como noclip. Ao desativá-lo, a velocidade instantânea é preservada, mas a saída é
recusada se a cápsula ocupar uma parede, teto ou prop.

### Contato e áudio

O callback publica impactos somente em `TOUCH_FOUND`, usando pontos e impulso
normal por massa efetiva. Um buffer na stack substitui a alocação por par; isso
afeta somente a telemetria acústica, nunca os contatos do solver. Contatos
persistentes em repouso não geram eventos a cada tick.
`ImpactAcousticResolver` transforma energia, massa efetiva, materiais e
estrutura em volume, pitch, abafamento, duração e prioridade. O mixer continua
fora do callback de física.

## Materiais e assets

`SurfaceMaterial` separa contato, estrutura, acústica e deformação. O catálogo
registra IDs canônicos em inglês e rejeita valores inválidos. Massa total
autorada prevalece; densidade vezes volume é usada somente quando o asset opta
por massa automática.

O contrato completo do Blender, dos colliders e dos sons está em
`docs/PHYSICAL_ASSET_AUTHORING.md`.

## Workbench

O Workbench mantém menu principal, laboratório, visualizador de props e
configurações. O catálogo `Q` instancia corpos a partir de descritores e recursos
cozidos compartilhados. `Z` remove a última entidade criada. Renderização, áudio,
Physgun e personagem consultam a mesma `PhysicsScene3D`; não existem mundos de
colisão paralelos com resultados divergentes.

O antigo solver, o jogo SoccerFall, o rig Footwork legado, o bípede e o editor
de animação foram removidos da árvore ativa e são proibidos pelas verificações
arquiteturais. O módulo atual de footwork é uma implementação nova e neutra de
backend: produz alvos cinemáticos para os pés, sem conhecer PhysX, Workbench,
renderizador ou esqueleto.

### Ragdoll ativo e laboratório de equilíbrio

`NaturalBalanceSystem3D` estima polígono de apoio, ponto de captura, urgência e
direção de recuperação. `FootworkSystem3D` planeja transferência de peso,
trajetória e replantio; `WholeBodyController3D` resolve os objetivos de centro
de massa, tronco, solas e contrarrotação em torques articulares. O
`ActiveRagdollController3D` apenas coordena esses três níveis. Em particular,
uma passada de recuperação é tratada como locomoção dinâmica mesmo sem comando
de caminhada, e a escolha lateral usa a velocidade real do corpo para sempre
priorizar o pé externo em vez de cruzar a base.

Forças auxiliares na raiz e na coluna são opcionais e ficam fora da estratégia
biomecânica. Com a opção desligada, postura, passos, braços e proteção de queda
continuam ativos exclusivamente por contatos e torques internos.

`RagdollImpactTest3D` é uma máquina de estados determinística e neutra de
backend. Ela publica rajadas diretas, vento contínuo ou sequências aleatórias
como força, direção relativa e duração; não conhece PhysX nem move links. O
Workbench distribui a força física na bacia e no tronco pelo contrato de
`PhysicsScene3D`, e os testes de fundação consomem exatamente o mesmo gerador.

## Benchmarks

`MatterPhysicsBenchmark` cobre cena vazia, props separados/em contato, 1–3
ragdolls em colisão e os 22 jogadores dispersos ou comprimidos no campo. Ele
publica P50/P95/pior passo e decompõe pré-simulação, dispatch, espera do PhysX,
callbacks, sincronização e estatísticas. O teste de fundação mantém ainda 1.024
corpos até sleep e um stress de 22 articulations por 240 passos. Performance
deve ser medida em `Release` ou `RelWithDebInfo`; `Debug` é destinado a
diagnóstico.
