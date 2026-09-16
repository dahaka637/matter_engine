# PhysX — contrato de desempenho da MatterEngine

Este documento registra as decisões adotadas para a carga-alvo inicial:
simulação determinística a 120 Hz, 22 humanos biomecânicos ativos, bola, props
e contatos simultâneos no campo. Ele não substitui profiling por cena.

## Diagnóstico da regressão original

O custo não era inerente a ragdolls. A integração acumulava quatro problemas:

1. cada humano de 18 links mantinha self-collision indiscriminada, embora
   várias cápsulas conectadas nascessem próximas/sobrepostas; um único
   ragdoll parado produzia dezenas de pares inúteis;
2. todo par pedia pontos e notificações persistentes para áudio, inclusive
   links que nem possuíam registro acústico;
3. PABP e dez workers eram acionados também para ilhas pequenas, onde mutex,
   wake-up e troca de contexto superavam o trabalho do solver;
4. o executável do atalho era uma build `Debug` do PhysX (`PX_DEBUG` e
   `PX_CHECKED`), inadequada para números de FPS.

## Configuração adotada

- Reduced-coordinate articulation do PhysX; não existe solver paralelo
  artesanal.
- TGS, passo fixo de 1/120 s e 4/1 iterações. Uma ilha inteira paga a maior
  contagem solicitada por qualquer ator, portanto iterações maiores devem ser
  aplicadas somente ao corpo que provar necessidade.
- ABP como broad phase geral. PABP só volta mediante benchmark de uma cena
  grande que amortize sua memória e paralelização.
- Um `PxAggregate` por humano, com self-collision interna seletiva. Uma matriz
  anatômica imutável elimina no filter shader apenas a cadeia direta
  pai/filho, que o próprio PhysX já exclui de articulations. Avô/neto, membros
  irmãos e lados diferentes continuam colidindo: assim o `Chest` bloqueia o
  `UpperArm`, embora o pai direto dele seja `UpperChest`. Isso impede braços e
  pernas de atravessarem o corpo sem recriar pares adjacentes inúteis.
  Colisões entre aggregates e com o mundo permanecem habilitadas.
- Dispatcher adaptativo: execução inline abaixo do limiar medido; pool
  compartilhado liberado gradualmente para cargas maiores.
- Contato detalhado somente para corpos com consumidor acústico. Ragdolls
  participam integralmente do solver sem gerar callbacks descartados.
- Scratch buffer reutilizado, primitives convexas simples, CCD seletivo,
  sleeping e snapshots apenas de atores ativos.
- Links limitam velocidade de despenetração e impulso máximo por contato.
  A Physgun usa teto de aceleração proporcional à massa quando segura um
  membro, em vez de aplicar ao antebraço o mesmo limite absoluto de força
  reservado a props pesados.
- Ragdolls usam speculative CCD por link. Ele antecipa contatos a partir da
  velocidade linear e angular das cápsulas e evita tunneling entre ticks, sem
  executar swept CCD completo em todo o esqueleto.
- Build interativa padrão `RelWithDebInfo`; `Debug` serve para investigação.

## O que não foi adotado

- GPU rigid bodies: 22 articulations ainda são uma carga pequena para justificar
  requisito CUDA, buffers próprios e sincronização CPU/GPU.
- Redução de DOFs, frequência física ou colisão entre jogadores: o orçamento
  foi atingido corrigindo a integração, sem essas perdas.
- Aumento global de solver iterations: TGS 4/1 passou o teste de estabilidade.
- Scene-query ou physics LOD agressivo: permanecem opções futuras, não
  necessárias para o orçamento atual.

## Guardas contra regressão

`MatterEngineTests` mantém:

- 1.024 corpos dinâmicos até sleep;
- 22 ragdolls completos por 240 passos;
- estados finitos;
- juntas com separação de âncora abaixo de 1,5 cm;
- contatos reais presentes;
- zero relatórios acústicos vindos dos links.

`MatterPhysicsBenchmark` mede P50, P95, máximo, número de contatos, tarefas,
workers e tempo de cada fase. O orçamento é 8,33 ms por tick; o resultado deve
ser avaliado em `build-profile`.

Na medição de 24/07/2026, com self-collision anatômica ativa, 22 ragdolls
dispersos ficaram em 1,47 ms P50 / 2,05 ms P95 e 22 ragdolls comprimidos no
campo em 2,09 ms P50 / 2,60 ms P95. Um ragdoll isolado ficou em 0,12 ms P50.
Esses números já incluem colisão avô/neto, limites anti-explosão e speculative
CCD em todos os links.

## Complexidade visual não é complexidade física

O asset `soccer_ball.glb` tem 81.920 triângulos visuais, mas sua representação
física é uma única `PxSphereGeometry`. O perfil integrado confirmou cerca de
0,05 ms de PhysX; a perda anterior vinha de desenhar a malha completa no
depth-prepass e novamente nas quatro cascatas de sombra.

Props densos agora geram dois index buffers de LOD com `meshoptimizer` no
carregamento. O vertex buffer, materiais e texturas continuam compartilhados;
a geometria integral permanece perto da câmera, o LOD é escolhido pelo tamanho
aparente e a sombra usa sempre o proxy mais compacto. Nenhuma simplificação é
feita durante o frame. A bola passou de aproximadamente 380 para 621 FPS no
smoke integrado, e o pneu de 517 para 577 FPS, sem mudar seus colliders.

## Documentação oficial usada

- [PhysX Best Practices](https://nvidia-omniverse.github.io/PhysX/physx/5.3.0/docs/BestPractices.html)
- [PhysX Threading](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/Threading.html)
- [PhysX Simulation](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/Simulation.html)
- [PhysX Articulations](https://nvidia-omniverse.github.io/PhysX/physx/5.1.1/docs/Articulations.html)
- [PhysX Rigid Body Collision](https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/docs/RigidBodyCollision.html)
- [PhysX Scene Queries](https://nvidia-omniverse.github.io/PhysX/physx/5.3.1/docs/SceneQueries.html)
- [meshoptimizer](https://github.com/zeux/meshoptimizer)
