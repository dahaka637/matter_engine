# Calibração antropométrica do RL (Codex, 11–12/10/2026)

A cadeia de antropometria que o Codex criou no início do RL: a referência de
de Leva (1996) no schema v3, a referência articular dos 39 eixos (33 ainda
pendentes), os landmarks anatômicos (candidatos na malha, resolver e o comando
`MatterModelInspector landmarks`) e os testes deles.

Saiu do build em 12/10/2026, no plano de Aprendizado por Reforço do Claude
(vault: `30_Projects/matter_engine/Aprendizado_por_Reforco/Plano_Claude.md`,
seção 3). Ela bloqueava o treino até uma "versão aprovada" da calibração, e as
pendências dela não mudam se o personagem aprende a ficar em pé.

O que ela apontou ficou como decisão no plano:

- massa por segmento contra de Leva (cabeça + pescoço 10,1% contra 6,9%,
  coxas 11,2% contra 14,2%, abdômen 10,2% contra 16,3%): o corpo do jogo foi
  mantido por ora (D3);
- tetos de torque constantes e altos: o RL usa tetos de atleta por cima do
  perfil, sem mudar o perfil do jogo (D2).

Ficaram ativos, fora daqui: `RagdollCalibrationAudit3D`, `RagdollDynamicLab3D`,
`RagdollJointLimitLab3D` (agora sobre os eixos do perfil),
`RagdollColliderWireframe3D` e as ferramentas `MatterRLAudit`,
`MatterRLDynamics`, `MatterRLJointProbe` e `MatterModelInspector colisores`.

Os caminhos aqui dentro repetem os de origem no repositório.
