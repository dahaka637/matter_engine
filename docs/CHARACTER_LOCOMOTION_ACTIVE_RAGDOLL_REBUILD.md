# Reconstrução de personagem, locomoção e active ragdoll

**Data do estudo:** 23–24 de setembro de 2026  
**Estado:** integração CC0 controlável, aguardando avaliação visual

## Escopo

Este trabalho substitui o caminho rejeitado de locomoção, footwork, suporte
vertical e equilíbrio. Os protótipos `AnimatedRagdollController3D`,
`HybridRagdollAssist3D` e `ContactFootwork3D` não participam mais do build nem
do runtime. Os clipes retargeteados continuam sendo assets de entrada; sua
seleção, combinação, sincronização e correção foram reimplementadas.

O código novo foi escrito para os contratos neutros da MatterEngine. Nenhum
código dos projetos estudados foi copiado.

## Fontes estudadas

### Overgrowth

- Repositório: <https://github.com/WolfireGames/overgrowth>
- Revisão estudada: `245fe4828631c84c0023d29d1525f5716ccb6106`
- Licença: Apache 2.0.

O personagem animado mantém um objeto de movimento separado e atualiza o
`char_sphere` persistente. A entrada em ragdoll liga a física do esqueleto ao
mundo; a saída a desliga e retorna ao estado animado. No modo físico, cada
junta recebe alvo e força. O IK dos pés adapta a pose renderizada ao solo; ele
não substitui a navegação por um planejador autônomo de passos.

Decisão adotada: locomoção normal possui representação de navegação persistente
e pose exata. A articulação ganha liberdade somente em um modo físico explícito.

### ALS Community

- Repositório: <https://github.com/PanicPetal/ALS-Community>
- Revisão estudada: `d044fcd9da572212be8de76f4d248063695a38d2`

ALS separa estado de movimento, ação, postura, marcha e modo de rotação. A
cápsula e o Character Movement resolvem a travessia; o ragdoll é um estado
separado. A animação normaliza diagonais, separa comprimento da passada de
velocidade de reprodução e trava o pé quando a curva indica apoio completo.
Cada pé consulta o solo e a pelve acompanha o apoio mais baixo.

Decisões adotadas:

- entrada em coordenadas da câmera;
- caminhada em quatro bases cardinais, combinadas para oito direções;
- caminhada e corrida cardinais relativas à câmera;
- postura agachada e estado aéreo independentes da marcha;
- travamento do pé somente quando ele já entrou em apoio, sem arrastá-lo até
  uma posição plantada falsa.

### Lyra e Game Animation Sample

- Lyra Animation: <https://dev.epicgames.com/documentation/unreal-engine/animation-in-lyra-sample-game-in-unreal-engine>
- Lyra Sample Game: <https://dev.epicgames.com/documentation/unreal-engine/lyra-sample-game-in-unreal-engine>
- Game Animation Sample: <https://dev.epicgames.com/documentation/unreal-engine/game-animation-sample-project-in-unreal-engine>

Lyra usa lógica por estado, distance matching nas partidas e paradas, stride
warping nos ciclos, orientation warping para locomoção completa e compensação
de yaw da raiz. O Game Animation Sample mantém o modelo de movimento por
cápsula mesmo com seleção avançada de poses; motion matching escolhe animação,
não resolve a trajetória física.

Decisão adotada nesta etapa: ciclo baseado em distância e mistura cardinal.
Motion matching fica para quando existir uma biblioteca de animações grande e
anotada o suficiente para uma busca de pose real.

### Kickback

- Repositório: <https://github.com/blugart-dev/kickback>
- Revisão estudada: `1d2c5ae3617eb676dc86368a30eb31f80ab52419`
- Licença: MIT.

A separação entre rig de animação, rig físico e skin é útil, assim como frames
anatômicos, perfis e sincronização física para render. O próprio
`docs/AUDIT_2026-09-12.md` do projeto rejeita a base dinâmica atual: sobrescrita
de velocidades, redução de gravidade, pinos de pelve/pés e teleporte da raiz
apagam impulsos e brigam com juntas e contatos.

Decisão adotada: aproveitar a organização e a matemática de rig, sem copiar o
resolvedor por molas nem o comportamento de tropeço do projeto.

### PhysX

- Articulations: <https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/Articulations.html>
- Reduced-coordinate joint API: <https://nvidia-omniverse.github.io/PhysX/physx/latest/_api_build/classPxArticulationJointReducedCoordinate.html>

Reduced coordinate articulations possuem drives PD implícitos, limites de
força/torque e alvos de posição e velocidade. São a base física correta já
existente na engine. O modo guiado não tenta simular rigidez infinita: o
backend resolve uma restrição de pose completa após o solver. Autoridade menor
usa os drives físicos com tetos reais.

## Arquitetura implementada

```text
entrada do jogador
       │
       ▼
CharacterMotorCommand3D
       │
       ▼
cápsula PhysX persistente ──► trajetória resolvida, chão, salto, agachamento
       │
       ▼
CharacterLocomotion3D ──────► pose, fase, blend direcional, apoio dos pés
       │
       ├────────► alvos dos drives articulares
       └────────► guia completo da raiz e das juntas
                         │
                         ▼
              articulation PhysX e skin
```

### Ordem fixa a 120 Hz

1. A entrada move a cápsula persistente.
2. A locomoção lê o resultado da cápsula e o snapshot anterior da articulation.
3. O controlador enfileira drives e guia de pose.
4. `PhysicsScene3D` simula e resolve contatos.
5. O backend aplica a autoridade solicitada e publica o snapshot.
6. O Workbench atualiza a skin e a câmera.

Nenhum controlador escreve transforms nativos.

## Locomoção e footwork

- Caminhada frontal, traseira e lateral são amostradas na mesma fase
  normalizada e combinadas pelos componentes da velocidade local.
- A fase avança pela distância percorrida e pelo comprimento original da
  passada. Isso conserva a relação entre pé e deslocamento e evita slow motion.
- Caminhada e corrida mantêm a referência da câmera e combinam bases frontal,
  traseira e laterais. Assim, as oito direções usam movimento corporal próprio.
- Diagonais não somam duas velocidades; a entrada e a mistura são normalizadas.
- O arco de swing vem do clipe. Não existe um gerador procedural tentando
  arrastar o pé até um alvo.
- Cada clipe traz curvas de contato de ambos os pés. As fases são alinhadas no
  apoio esquerdo antes da combinação direcional. O pé só trava quando a curva
  indica stance e ele está baixo, lento e sobre uma superfície caminhável. Ao
  entrar em swing, o lock é removido para conservar o arco do clipe.
- A correção usa IK limitada às três juntas da perna e respeita todos os
  limites do perfil físico.
- Agachamento parado/em movimento e início/loop/aterrissagem do salto usam
  clipes dedicados. A altura da pelve e o movimento dos membros vêm desses
  clipes; a cápsula continua responsável pela trajetória balística.

## Biblioteca de animações

O catálogo antigo foi removido do caminho ativo e preservado apenas em
`archive/2026-09-24-retired-animation-library/`. A biblioteca atual contém 14
clipes derivados de duas fontes CC0:

- Universal Animation Library, de Quaternius: idle, caminhada/corrida frontal
  e agachamento;
- KayKit Character Animations: trás, laterais e os três estágios do salto.

Os arquivos de origem e as licenças acompanham o projeto em
`assets/animations/source/cc0/`. O runtime consome somente o formato canônico
`matter-ragdoll-animation-1`.

## Autoridade física

O guia da articulation contém uma raiz mundial completa: XYZ, orientação,
velocidade linear e velocidade angular. Juntas e raiz compartilham a mesma
decisão de autoridade.

- **100%:** pose e trajetória exatas, sem erro de servo ou oscilação.
- **1–99%:** mistura explícita com a simulação e músculos limitados.
- **0%:** sem guia; permanecem somente os drives musculares escolhidos.
- **Impacto externo:** o próprio passo de contato reduz a autoridade antes de
  publicar a pose; uma envolvente curta conserva a reação e retorna ao modo
  guiado com histerese.
- **Physgun:** o guia é desligado enquanto um link é manipulado.

Contato estático normal das solas é excluído dessa detecção. A telemetria exibe
a força da reação e a autoridade efetiva da raiz.

## Controles integrados

A aba **PERSONAGEM** da toolbar assume o último humano ativo:

- `WASD`: oito direções relativas à câmera;
- `Shift`: corrida;
- `Ctrl`: agachamento;
- `Espaço`: salto;
- mouse: câmera e orientação do corpo.

A câmera é terceira pessoa e faz raycast contra cenário estático para não
atravessar paredes. A cápsula ignora shapes de ragdoll enquanto controla o
avatar, evitando colisão do personagem com sua própria representação física.

## Limitações conhecidas desta integração

- As bibliotecas CC0 não oferecem todas as direções com a mesma família de
  movimento. A caminhada lateral usa o strafe disponível no KayKit e a corrida
  para trás usa a marcha traseira com cadência aumentada. A avaliação visual
  deve determinar se esses dois casos exigem novos assets.
- A reação curta a impacto está integrada. Stagger com passos de captura,
  queda consciente e recuperação procedural ainda não foram implementados
  sobre a base nova.
- As curvas de contato são derivadas da altura retargeteada dos pés. Marcação
  manual de heel strike e toe off ainda pode melhorar casos específicos.
- Compilar confirma integração de código; a aprovação visual e física depende
  da avaliação manual solicitada pelo usuário.
