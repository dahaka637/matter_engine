# Roadmap técnico

Qualidade de grande estúdio é um resultado mensurável. Cada marco precisa de
testes, profiling e critérios claros de saída.

## Marco 0 — Fundação limpa

Estado: concluído.

- identidade MatterEngine e build moderno;
- Vulkan 1.4, SDL3 encapsulada e Workbench minimalista;
- jogo antigo, Footwork, bípede e editor de animação removidos;
- catálogo glTF, materiais físicos, impactos e Physgun funcional;
- contrato Blender/glTF documentado e validado.

## Marco 1 — Backend físico PhysX

Estado: fundação integrada; calibração contínua.

Implementado:

- solver antigo completamente removido;
- NVIDIA PhysX 5.9.0 fixado por commit e isolado atrás de API neutra;
- timestep de 120 Hz, TGS, PABP, multithreading, sleeping, ilhas e scratch;
- static triangle mesh BVH34 para mundo;
- primitivas e compounds convexos V-HACD para corpos dinâmicos;
- cooking compartilhado por asset, sem trabalho de colisão no spawn;
- handles geracionais, scene queries e telemetria de desempenho;
- Physgun por D6 drive e Capsule Controller oficial;
- teste de regressão com 1.024 corpos, apoio composto e CCT;
- eventos de impacto somente no início do contato.

Ainda necessário:

- ampliar o cache persistente para colliders autorados compostos;
- benchmarks Release reproduzíveis com cenas de 1k, 5k e 10k props;
- captura Omni-PVD e budgets de frame para cenas de estresse;
- batch de poses ativas para renderização/streaming em escala maior;
- políticas de CCD por classe de objeto baseadas em velocidade e tamanho;
- testes longos de empilhamento, grande diferença de massa e tunneling.

Critério de saída: cenários representativos mantêm orçamento de frame definido,
sem explosão de energia, penetração crescente ou spikes no spawn.

## Marco 2 — Personagens e ragdolls ativos

- esqueleto e constraints autorados por asset;
- reduced-coordinate articulations do PhysX;
- controlador biomecânico por targets/torques, nunca escrita de transforms;
- pés e bola com CCD e materiais calibrados;
- transição controlada entre locomoção, desequilíbrio, queda e recuperação;
- testes de conservação de energia, limites articulares e estabilidade;
- LOD físico e aggregates para multidões.

Critério de saída: personagens interagem com bola, terreno e outros corpos sem
dependência de animações que atravessem a física.

## Marco 3 — Assets e cenas

- AssetManager com handles, cache e fallback;
- cooking e upload assíncronos;
- cenas versionadas e entidades persistentes;
- undo/redo de criação, remoção e propriedades;
- KTX2/Basis Universal e distinção sRGB/linear.

## Marco 4 — Renderização de produção

- PBR metallic/roughness completo;
- iluminação Forward+ ou clustered escolhida por profiling;
- sombras estáveis, atmosfera e pós-processamento HDR;
- instancing, frustum/occlusion culling e LOD;
- profiling CPU/GPU e regressões com RenderDoc.

Ray tracing, mesh shaders e bindless entram somente quando um consumidor real
e medições justificarem a complexidade.

## Marco 5 — Ferramentas

- outliner e propriedades sobre o sistema de cenas;
- import pipeline com relatório de validação física/visual;
- editor de materiais e preview físico/acústico;
- console e profiler acionáveis, ocultos por padrão.

Um editor de animação só será reconstruído depois de existir runtime, formato de
asset e requisitos de personagem aprovados.

## Marco 6 — Escala

- job system e budgets por subsistema;
- streaming de mundos e assets;
- render graph baseado em passes reais;
- telemetria de memória e frame pacing;
- builds automatizados, sanitizers e matriz de GPUs/drivers.
