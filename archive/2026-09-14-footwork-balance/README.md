# Arquivo de estudo — footwork e equilíbrio anteriores

Arquivado em 14/09/2026 por solicitação do usuário. Não é código ativo e não
participa de nenhum alvo CMake. Não recolocar estes controladores no runtime
como dependência do novo sistema de animação.

Preservados integralmente os módulos de Locomotion, ActiveRagdollController,
NaturalBalance, WholeBodyController, laboratório de footwork e snapshots dos
consumidores Workbench/CMake/testes antes da substituição.

Os testes antigos de equilíbrio/impacto estão no snapshot completo de
`tests/EngineFoundationTests.cpp`; a falha histórica de recuperação da rajada
esquerda **não foi corrigida**. Esses critérios pertencem ao controlador
aposentado. Testes genéricos de PhysX, limites, contatos, Physgun e coesão das
juntas continuam ativos. `MatterEngine.AnimatedRagdoll` testa a substituição.

A nova implementação está em `src/Engine/Control/AnimatedRagdollController3D.*`
fora deste arquivo. Ela usa somente alvos articulares e forças/torques externos
limitados, instrumentados e desligáveis; não escreve transforms de corpos.
