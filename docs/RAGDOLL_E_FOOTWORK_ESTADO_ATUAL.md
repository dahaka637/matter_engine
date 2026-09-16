# Ragdoll e FootWork da MatterEngine

## Estado técnico atual e base para locomoção biomecânica

**Data deste levantamento:** 26 de julho de 2026  
**Escopo:** implementação atualmente ativa na MatterEngine  
**Objetivo:** registrar o que já existe, como funciona e qual arquitetura
deve ligar o planejador de passos ao ragdoll físico no futuro.

---

## 1. Visão geral

A MatterEngine possui hoje dois sistemas diferentes e propositalmente
separados:

1. **Ragdoll físico:** corpo humano articulado, simulado pelo PhysX, capaz de
   cair, colidir, receber forças e resistir a movimentos por meio de drives.
2. **FootWork:** planejador cinemático de posicionamento dos pés, capaz de
   escolher onde e como cada pé deve tocar o chão durante caminhada, corrida,
   curvas e mudanças de direção.

Eles ainda **não formam um personagem locomovendo-se fisicamente**.

O ragdoll sabe obedecer à física, conservar suas juntas e aplicar rigidez a
uma pose. Ele ainda não sabe escolher uma pose para caminhar ou equilibrar-se.

O FootWork sabe produzir poses-alvo dos pés, mas não possui pernas, massa,
centro de gravidade ou contatos físicos. Ele ainda não sabe transformar um
alvo de pé em torques nas articulações.

Essa separação é positiva. Ela permite validar independentemente:

- a estabilidade e o desempenho do corpo físico;
- a qualidade do planejamento dos passos;
- a futura camada de equilíbrio e controle corporal.

```text
               IMPLEMENTADO HOJE

 Input ──> FootWork ──> poses desejadas para os pés

 Mundo ──> PhysX ──> ragdoll físico ──> poses físicas dos 18 links


                 PONTE FUTURA

 FootWork ──> equilíbrio ──> IK/pose corporal ──> drives do ragdoll
                     ▲                                │
                     └──── contatos + COM + pose ◀────┘
```

---

# Parte I — Ragdoll físico

## 2. Arquivos principais

- Perfil e tipos públicos:
  [`RagdollProfile3D.hpp`](../src/Engine/Physics/RagdollProfile3D.hpp)
- Leitura e validação do perfil:
  [`RagdollProfile3D.cpp`](../src/Engine/Physics/RagdollProfile3D.cpp)
- Perfil humano atual:
  [`HumanAdultV1.ragdoll.json`](../assets/physics/ragdolls/HumanAdultV1.ragdoll.json)
- API física independente de backend:
  [`PhysicsScene3D.hpp`](../src/Engine/Physics/PhysicsScene3D.hpp)
- Implementação PhysX:
  [`PhysXScene3D.cpp`](../src/Engine/Physics/PhysX/PhysXScene3D.cpp)
- Apresentação e spawn no laboratório:
  [`RagdollRuntime.cpp`](../src/Workbench/Laboratory/RagdollRuntime.cpp)
- Testes funcionais e de escala:
  [`EngineFoundationTests.cpp`](../tests/EngineFoundationTests.cpp)

## 3. Representação física

O ragdoll é criado como uma
`PxArticulationReducedCoordinate` com base flutuante.

Isso significa:

- os segmentos pertencem a uma única estrutura articulada;
- a pelve não está fixada ao mundo;
- o solver trabalha diretamente com os graus de liberdade das juntas;
- as restrições internas são tratadas de modo mais robusto que uma coleção
  solta de rigid bodies conectados por joints externos;
- o ragdoll pode ser passivo ou parcialmente motorizado.

O identificador público é um `RagdollHandle3D` geracional. Nenhum ponteiro ou
tipo do PhysX atravessa a API pública da engine.

Se um ragdoll for destruído e o slot interno for reutilizado, a geração muda.
Um handle antigo, portanto, não passa a controlar acidentalmente uma nova
instância.

## 4. Perfil HumanAdultV1

O perfil atual possui:

- **18 links físicos**;
- **17 articulações**;
- **41 graus de liberdade**;
- **75 kg de massa total**;
- **5,5 cm de raio em todos os segmentos físicos**;
- pelve a aproximadamente **98 cm do solo** na pose neutra.

### 4.1 Hierarquia

```text
Pelvis
├── Abdomen
│   └── Chest
│       └── UpperChest
│           ├── Neck
│           │   └── Head
│           ├── LeftUpperArm
│           │   └── LeftForearm
│           │       └── LeftHand
│           └── RightUpperArm
│               └── RightForearm
│                   └── RightHand
├── LeftThigh
│   └── LeftShin
│       └── LeftFoot
└── RightThigh
    └── RightShin
        └── RightFoot
```

Não existem links separados para os dedos dos pés. Cada pé é um único link.

### 4.2 Distribuição de massa

| Segmento | Fração | Massa aproximada |
|---|---:|---:|
| Pelve | 14,0% | 10,50 kg |
| Abdômen | 10,0% | 7,50 kg |
| Peito | 12,0% | 9,00 kg |
| Peito superior | 8,0% | 6,00 kg |
| Pescoço | 2,0% | 1,50 kg |
| Cabeça | 8,0% | 6,00 kg |
| Cada braço superior | 2,5% | 1,875 kg |
| Cada antebraço | 1,6% | 1,20 kg |
| Cada mão | 0,6% | 0,45 kg |
| Cada coxa | 11,5% | 8,625 kg |
| Cada canela | 4,7% | 3,525 kg |
| Cada pé | 2,1% | 1,575 kg |

As frações são validadas e precisam somar 1,0.

O tensor de inércia de cada link é calculado pelo PhysX a partir de sua
geometria, massa e centro de massa local.

### 4.3 Graus de liberdade

| Região | DOFs por lado/link | Total |
|---|---:|---:|
| Abdômen, peito, peito superior, pescoço e cabeça | 3 cada | 15 |
| Braço superior | 3 por lado | 6 |
| Antebraço | 2 por lado | 4 |
| Mão | 2 por lado | 4 |
| Coxa | 3 por lado | 6 |
| Joelho/canela | 1 por lado | 2 |
| Tornozelo/pé | 2 por lado | 4 |
| **Total** |  | **41** |

As juntas esféricas usam os eixos:

- `TWIST`;
- `SWING1`;
- `SWING2`.

Os joelhos são juntas revolutas e habilitam somente `TWIST`.

Cada eixo possui no JSON:

- limite mínimo e máximo;
- stiffness base;
- damping base;
- torque máximo.

## 5. Frames articulares

Cada link armazena no perfil:

- pose no espaço de modelo;
- collider no espaço local do link;
- índice do pai;
- âncora anatômica no espaço de modelo;
- orientação do frame anatômico.

Ao criar a articulation, a engine converte a mesma âncora do espaço de modelo
para os espaços locais do pai e do filho.

O eixo X do frame anatômico corresponde a `TWIST`; Y e Z correspondem a
`SWING1` e `SWING2`.

Esse contrato é importante para o futuro controlador: um alvo de junta não
deve depender de Euler global nem da orientação da câmera. Ele deve ser
expresso nos eixos anatômicos definidos pelo perfil.

## 6. Colliders e apresentação visual

### 6.1 Física

Todos os 18 links físicos usam cápsulas do PhysX com raio uniforme de 5,5 cm.

Isso inclui fisicamente:

- cabeça;
- mãos;
- pés;
- tronco.

O requisito de espessura uniforme evita um tronco visualmente gordo e mantém
o perfil simples para os primeiros testes.

### 6.2 Visual

A representação visual não é idêntica em forma, embora acompanhe exatamente
a pose física do link:

- cabeça: superelipsoide arredondado e levemente quadrado;
- mãos: caixas achatadas;
- pés: caixas achatadas e alongadas;
- demais links: cápsulas;
- juntas: pequenas esferas visuais;
- coluna: cápsulas visualmente alongadas para parecer contínua.

As esferas das juntas são somente apresentação. Elas não adicionam corpos ou
colliders.

As malhas são geradas uma vez e seus buffers são compartilhados por todas as
instâncias. Cada ragdoll envia transforms, sem reconstruir a geometria.

O renderizador realiza culling grosso por ragdoll e calcula uma máscara das
cascatas de sombra antes de percorrer os links.

## 7. Colisões internas

A self-collision está habilitada na articulation.

Entretanto, nem todo par interno deve colidir:

- um link não colide consigo mesmo;
- pai e filho diretos não colidem, pois suas cápsulas se sobrepõem
  intencionalmente na articulação;
- os demais pares internos colidem.

Assim:

- antebraço pode colidir com tronco;
- braço pode colidir com peito quando não é pai direto daquele link;
- pernas podem colidir entre si;
- membros distantes não atravessam livremente o corpo.

A engine grava para cada shape uma máscara topológica de até 32 links.
Pares internos desnecessários são mortos pelo filter shader antes de chegar ao
solver.

Ragdolls diferentes continuam colidindo normalmente entre si e com o mundo.

### 7.1 Proteções contra instabilidade

Cada link usa:

- `contactOffset = 0,012 m`;
- `restOffset = 0`;
- velocidade máxima de despenetração de `3 m/s`;
- impulso máximo por contato de `massa × 12`;
- velocidade angular máxima de `24 rad/s`;
- speculative CCD.

Esses limites existem para impedir que uma penetração profunda ou a Physgun
transforme a correção de contato em uma explosão de velocidade.

O speculative CCD considera também o movimento angular das cápsulas finas,
sem pagar o custo de CCD completo em todos os links.

## 8. Broad phase e solver

Cada ragdoll é colocado em um `PxAggregate`.

Consequências:

- o conjunto ocupa uma entrada coerente no broad phase;
- os pares internos continuam filtrados;
- colisões contra o mundo e contra outros ragdolls continuam normais.

A cena atual utiliza:

- passo fixo de **120 Hz**;
- solver **TGS**;
- broad phase **ABP**;
- 4 iterações de posição;
- 1 iteração de velocidade;
- estabilização habilitada;
- active actors habilitados;
- scratch buffer reutilizável de 1 MiB.

O dispatcher é adaptativo:

- cenas pequenas executam tarefas PhysX na thread chamadora;
- o pool compartilhado é ativado gradualmente quando atores, contatos e DOFs
  ativos justificam o custo de paralelização.

## 9. Drives e rigidez

O ragdoll não possui hoje um controlador de animação. Ele possui drives
capazes de manter um conjunto de ângulos-alvo.

A rigidez de 0% a 100% escala os parâmetros por curvas diferentes:

```text
stiffness = stiffness_base × rigidez^2,2
damping   = damping_base   × rigidez^1,4
torque    = torque_base    × rigidez^1,25
```

Os drives usam `eFORCE`, com limites interpretados como força/torque.

Essa curva torna os valores baixos realmente flexíveis e faz o ganho crescer
progressivamente, em vez de transformar 10% em um boneco quase rígido.

### 9.1 Ações existentes

**Capturar pose**

- lê a posição atual de cada DOF;
- limita o valor à faixa anatômica;
- torna essa configuração o novo alvo.

**Pose neutra**

- coloca o alvo de todos os eixos habilitados em zero;
- o zero também é limitado à faixa anatômica.

**Soltar drives**

- desativa os motores;
- preserva juntas, limites e colisões;
- o ragdoll torna-se passivo.

Ao elevar a rigidez de zero para um valor positivo, a pose física atual é
capturada primeiro. Isso evita um estalo imediato em direção à pose neutra.

As alterações são enfileiradas e aplicadas no safe point anterior a
`simulate()`.

## 10. Snapshots e estado público

`RagdollState3D` publica:

- pose de cada link;
- velocidade linear de cada link;
- velocidade angular de cada link;
- estado de sleep;
- rigidez global atual.

Links adormecidos não são relidos do PhysX a cada passo. O último snapshot é
preservado.

Esse estado já é suficiente para calcular externamente:

- centro de massa global;
- velocidade do centro de massa;
- orientação do tronco;
- momento linear aproximado;
- posições reais dos pés.

Ainda não são publicados:

- contatos específicos de cada pé;
- forças normais por pé;
- centro de pressão;
- coordenadas e velocidades dos 41 DOFs em lote;
- alvos individuais dos drives;
- torques efetivamente aplicados pelos drives.

## 11. Physgun

A Physgun seleciona links por consultas exclusivas de ragdoll:

- raycast;
- sphere sweep.

O link selecionado é conectado a um alvo por um D6 joint do PhysX.

Para ragdolls, força e torque são limitados por aceleração e inércia, não
apenas por um teto global. Isso evita que a configuração usada para um prop
pesado seja aplicada integralmente a um antebraço leve.

## 12. Testes atuais do ragdoll

Os testes automatizados verificam:

- perfil com 18 links e 41 DOFs;
- massa, hierarquia e raio uniforme;
- ausência de links separados para os dedos;
- abertura mínima do quadril;
- gravidade na raiz flutuante;
- raycast e manipulação com a Physgun;
- rigidez, captura, pose neutra e soltura;
- destruição segura e invalidação do handle;
- 22 ragdolls completos durante 240 passos;
- poses sempre finitas;
- separação das âncoras inferior a 1,5 cm no teste de escala;
- contatos físicos reais sem relatórios acústicos desperdiçados;
- tentativa adversarial de empurrar braço para dentro do peito;
- ausência de velocidades explosivas nesse caso.

---

# Parte II — Protótipo de FootWork

## 13. Arquivos principais

- Contratos e configuração:
  [`FootworkTypes3D.hpp`](../src/Engine/Locomotion/FootworkTypes3D.hpp)
- Orquestração:
  [`FootworkSystem3D.cpp`](../src/Engine/Locomotion/FootworkSystem3D.cpp)
- Escolha do contato:
  [`FootstepPlanner3D.cpp`](../src/Engine/Locomotion/FootstepPlanner3D.cpp)
- Trajetória:
  [`FootworkTrajectory3D.cpp`](../src/Engine/Locomotion/FootworkTrajectory3D.cpp)
- Adaptador e interface do laboratório:
  [`FootworkLab.cpp`](../src/Workbench/Laboratory/FootworkLab.cpp)
- Base conceitual anterior à implementação:
  [`footwork-system-spec.md`](../footwork-system-spec.md)

## 14. Limite arquitetural

`Engine/Locomotion` não conhece:

- PhysX;
- Workbench;
- SDL/input global;
- renderizador;
- ragdoll;
- esqueleto.

O módulo recebe structs e uma interface semântica de consulta ao terreno. Ele
retorna estado e poses.

No laboratório existe um adaptador
`StaticFootworkTerrainProbe`, que implementa a consulta usando
`PhysicsScene3D::raycastStatic`.

Essa separação permite futuramente trocar:

- PhysX por outro backend;
- raycast por heightfield ou navmesh;
- pés de teste por pernas com IK;
- input manual por IA;

sem reescrever o planejador.

## 15. O que o protótipo representa

O FootWork possui:

- uma raiz virtual no chão;
- orientação virtual do corpo;
- dois pés cinemáticos;
- uma casa ideal para cada pé;
- no máximo uma passada ativa;
- polígono de suporte planejado;
- trajetória visual da passada.

Ele não possui:

- pelve física;
- pernas;
- joelhos;
- massa;
- gravidade própria;
- colisão dos pés;
- feedback de contato;
- equilíbrio;
- IK.

Portanto, a raiz virtual atual representa uma **intenção locomotora**, não o
centro de massa de um personagem.

## 16. Entrada

```cpp
struct FootworkInput3D {
    Vec2 movementLocal;
    float lookYawRadians;
    bool fast;
};
```

- `movementLocal.x`: esquerda/direita;
- `movementLocal.y`: frente/trás;
- `lookYawRadians`: direção de referência;
- `fast`: perfil de corrida, acionado por Shift no laboratório.

A direção local é convertida para o mundo usando o yaw da câmera.

Mudanças no WASD e mudanças na câmera são tratadas de forma diferente:

- trocar W por S ou W por A invalida o destino antigo e replaneja a passada;
- girar a câmera mantendo W curva o destino gradualmente sem zerar o relógio
  da passada.

## 17. Configuração atual

| Parâmetro | Valor padrão |
|---|---:|
| Comprimento do pé | 0,28 m |
| Largura do pé | 0,115 m |
| Espessura do pé | 0,055 m |
| Largura da base | 0,27 m |
| Velocidade caminhando | 1,05 m/s |
| Velocidade correndo | 3,15 m/s |
| Aceleração | 4,8 m/s² |
| Multiplicador de aceleração na corrida | 1,65× |
| Desaceleração | 7,2 m/s² |
| Velocidade angular máxima da raiz | 3,8 rad/s |
| Erro que dispara um passo | 0,19 m |
| Erro de liberação | 0,115 m |
| Erro angular que dispara um passo | 0,22 rad |
| Alcance máximo | 0,78 m |
| Rotação máxima de um passo | 0,58 rad |
| Folga anticruzamento | 0,025 m |
| Duração base caminhando | 0,34 s |
| Duração base correndo | 0,215 s |
| Clearance caminhando | 0,105 m |
| Clearance correndo | 0,155 m |
| Look-ahead caminhando | 0,16 s |
| Look-ahead correndo | 0,205 s |
| Inclinação de saída caminhando | +0,28 rad |
| Inclinação de contato caminhando | −0,22 rad |
| Inclinação de saída correndo | +0,44 rad |
| Inclinação de contato correndo | −0,34 rad |
| Inclinação máxima do terreno | 50° |
| Irregularidade máxima em relação ao plano | 0,035 m |
| Distância máxima raiz–apoio | 0,42 m |

Os valores são sanitizados antes do uso.

## 18. Estados do pé

```text
Planted ──> LiftOff ──> Swing ──> TouchDown ──> Planted
                         │
                         └──> Aborting ──> Planted
```

### Planted

Pé assentado. Sua pose não desliza junto com a raiz.

### LiftOff

Início da saída do chão. O calcanhar começa a levantar.

### Swing

Pé percorre a trajetória até o novo contato.

### TouchDown

Ponta elevada antes do contato e orientação convergindo para o terreno.

### Aborting

Quando o jogador solta o comando durante a passada, o sistema procura um
contato seguro próximo e encerra a passada sem deixar o pé suspenso.

## 19. Atualização da raiz virtual

A cada atualização:

1. a entrada é normalizada;
2. o yaw da raiz aproxima-se do yaw desejado;
3. a velocidade desejada é calculada;
4. aceleração ou desaceleração aproxima a velocidade atual;
5. a raiz avança;
6. a raiz é limitada em relação ao apoio real;
7. as casas ideais dos pés são atualizadas.

Durante uma passada, o limite usa o pé contralateral plantado como âncora.

Sem passada ativa, usa o centro dos dois pés.

Isso impede que a câmera e a raiz virtual escapem enquanto um pé tenta
alcançá-las.

## 20. Casas dos pés

A casa de cada pé é calculada a partir de:

- posição da raiz;
- eixo frontal da raiz;
- lado esquerdo/direito;
- metade da largura da base;
- pequeno offset frontal;
- antecipação pela velocidade desejada.

```text
home = raiz
     + lateral_do_lado
     + offset_frontal
     + velocidade_desejada × look_ahead
```

Na corrida, velocidade e look-ahead maiores produzem naturalmente um passo
mais longo.

As casas são intenção. O pé plantado não é arrastado até a casa. Um passo é
disparado quando o erro fica grande o suficiente.

## 21. Escolha do pé

Quando não existe uma passada ativa, cada pé recebe uma pontuação baseada em:

- distância até sua casa;
- erro angular;
- direção da curva;
- alternância em relação ao último pé usado.

O pé com maior necessidade é escolhido.

Durante uma curva, o pé contralateral recebe prioridade adicional.

## 22. Planejamento do contato

O planejador recebe:

- lado do pé;
- casa desejada;
- pose do pé de apoio;
- velocidade desejada;
- yaw da raiz.

Ele:

1. limita o alcance horizontal;
2. aplica a separação anticruzamento;
3. limita a mudança angular;
4. consulta o terreno;
5. produz posição, normal, yaw e orientação do contato.

### 22.1 Separação dos pés

A separação lateral mínima entre os centros é:

```text
largura do pé + margem
= 0,115 + 0,025
= 0,14 m
```

A trajetória também aplica uma zona de exclusão em torno do pé apoiado.

Isso impede que os dois pés ocupem a mesma posição durante translações ou
giros agressivos.

## 23. Leitura do terreno

Cada contato candidato usa cinco raios:

- centro;
- quatro pontos próximos aos cantos do pé.

Regras atuais:

- a amostra central é obrigatória;
- ao menos quatro das cinco amostras devem existir;
- as normais são combinadas;
- a inclinação não pode exceder 50°;
- a superfície deve ser aproximadamente planar.

Uma melhoria importante foi separar:

- variação natural de altura numa rampa;
- irregularidade em relação ao plano local.

Uma rampa inclinada pode ter grande diferença entre ponta e calcanhar e ainda
ser perfeitamente plana. O sistema rejeita o resíduo em relação ao plano, não
a diferença bruta de altura.

## 24. Trajetória

O movimento horizontal usa progressão cicloidal. Isso evita velocidade
constante artificial e suaviza saída e chegada.

O arco vertical usa:

```text
altura = clearance × sen(π × progresso)
```

Normal do terreno e yaw são interpolados suavemente.

### 24.1 Inclinação do pé

O pé não permanece paralelo ao solo durante toda a passada.

Existem quatro momentos:

1. neutro;
2. toe-off, com o calcanhar elevado;
3. passagem aproximadamente neutra;
4. aproximação com a ponta elevada;
5. contato novamente neutro.

Como o pivô visual fica no centro inferior do pé, a posição é compensada ao
inclinar. O extremo mais baixo continua sobre o plano, evitando que ponta ou
calcanhar atravessem visualmente o terreno.

## 25. Replanejamento

Uma mudança brusca no vetor local de entrada replaneja imediatamente a partir
da pose atual.

Isso resolve o caso:

```text
W durante uma passada ──> jogador pressiona S
```

O pé não termina primeiro o passo antigo para frente.

Já uma rotação contínua da câmera não reinicia o progresso. O destino
persegue a nova curva durante a parte inicial do swing e a passada sempre
termina.

## 26. Corrida

Shift ativa um perfil mecânico próprio, não apenas uma velocidade maior.

A corrida possui:

- velocidade maior;
- aceleração maior;
- look-ahead maior;
- passos mais longos;
- swing mais curto;
- clearance maior;
- toe-off mais forte;
- contato com maior inclinação;
- transição imediata para o pé contralateral.

Quando um pé conclui a passada, o outro pode iniciar no mesmo tick.

### Limitação atual

Ainda existe somente **uma passada ativa por vez**.

Portanto, a corrida atual tem apoio muito curto, mas não representa
explicitamente:

- dois pés simultaneamente em swing;
- fase aérea real;
- dois contatos parcialmente sobrepostos;
- forças de impulsão;
- cadência derivada do estado físico do corpo.

O “impulso” atual é representado pelo avanço mais agressivo da raiz virtual e
pela troca imediata de pé. Não é um impulso físico aplicado a uma pelve.

## 27. Saída e depuração

`FootworkDebugState3D` publica:

- raiz e velocidade;
- yaw da raiz;
- pose, casa e alvo de cada pé;
- fase e progresso;
- erro de posição e yaw;
- qualidade do terreno;
- pé ativo;
- trajetória amostrada;
- polígono de suporte;
- número de passos;
- bloqueio do terreno.

No laboratório:

- pés sólidos: poses atuais;
- contornos azuis: casas ideais;
- contorno laranja: alvo da passada;
- linha amarela: trajetória;
- contorno verde/azulado: polígono de suporte;
- cruz: raiz virtual.

Os alvos azuis ficam desativados por padrão e podem ser habilitados no painel.

## 28. Testes atuais do FootWork

Os testes automatizados cobrem:

- inicialização em terreno plano;
- pé esquerdo e direito no lado correto;
- caminhada e cadência;
- arco vertical;
- inclinação do pé para trás e para frente;
- raiz limitada ao apoio;
- parada durante swing;
- replanejamento frente–trás no próximo tick;
- não sobreposição durante giros agressivos;
- rotação contínua da câmera sem pé preso;
- corrida mais rápida que caminhada;
- ausência de pausa artificial entre passadas rápidas;
- determinismo para a mesma sequência de fixed steps;
- alinhamento a rampas;
- descida contínua de uma rampa íngreme.

---

# Parte III — Como unir os sistemas

## 29. O erro arquitetural que deve ser evitado

O FootWork não deve escrever transforms nos links do ragdoll.

Fazer isso:

- quebraria a conservação de momento;
- atravessaria colisões;
- disputaria com o solver;
- produziria foot skating;
- tornaria tackles e empurrões imprevisíveis.

Também não é correto mandar a pelve física seguir diretamente a raiz virtual
atual.

A raiz atual foi criada para testar planejamento. No personagem final, ela
deve tornar-se uma intenção ou referência, enquanto a pelve e o centro de
massa reais vêm do PhysX.

O controlador futuro deve produzir **alvos e forças**, nunca teletransportes.

## 30. Camadas recomendadas

```text
1. Comando locomotor
   velocidade, direção, corrida, ação tática
                         │
                         ▼
2. Planejador de contatos / FootWork
   qual pé, onde tocar, quando sair e quando chegar
                         │
                         ▼
3. Estimador de equilíbrio
   COM, velocidade do COM, contatos, suporte, capture point
                         │
                         ▼
4. Gerador de pose corporal
   pelve, tronco, pernas, braços e orientação desejada
                         │
                         ▼
5. IK / solução articular
   converte alvos mundiais em referências dos 41 DOFs
                         │
                         ▼
6. Controlador do ragdoll ativo
   stiffness, damping, velocidade-alvo e torque por eixo
                         │
                         ▼
7. PhysX
   contatos, gravidade, impulsos, colisões e integração
                         │
                         ▼
8. Feedback físico
   pose, velocidades, contatos e forças voltam ao estimador
```

Cada camada deve possuir structs de entrada e saída, sem conhecer UI ou
backend nativo.

## 31. Primeira extensão necessária: drives por eixo

A API atual controla apenas:

- rigidez global;
- captura global;
- pose neutra;
- soltura global.

Para locomoção, será necessário comandar os 41 DOFs individualmente.

Um contrato possível:

```cpp
struct RagdollAxisTarget3D {
    std::uint32_t linkIndex;
    RagdollAxis3D axis;
    float positionRadians;
    float velocityRadiansPerSecond;
    float stiffnessScale;
    float dampingScale;
    float maximumTorqueScale;
};

void setRagdollDriveTargets(
    RagdollHandle3D,
    std::span<const RagdollAxisTarget3D>);
```

Requisitos:

- backend-neutral;
- aplicado no safe point antes de `simulate()`;
- valores sempre limitados pelo perfil anatômico;
- sem alocação por eixo a cada tick;
- comando em lote para os 22 jogadores;
- possibilidade de deixar um eixo passivo;
- torque limitado por articulação.

O perfil atual de stiffness, damping e torque deve continuar sendo o teto
anatômico. O controlador envia escalas e alvos, não substitui os limites.

## 32. Segunda extensão: contatos sem custo acústico

Hoje contatos envolvendo ragdolls não geram o stream detalhado usado pelo
áudio. Isso foi uma otimização importante.

O equilíbrio, porém, precisará de telemetria específica dos pés:

```cpp
struct FootContactState3D {
    bool touching;
    Vec3 averagePosition;
    Vec3 averageNormal;
    float normalImpulse;
    float tangentialSpeed;
    float contactAreaEstimate;
};
```

Não é necessário reativar relatórios completos para todos os links.

Uma solução adequada é:

- marcar `LeftFoot` e `RightFoot` como sensores semânticos;
- solicitar dados mínimos somente quando esses links participarem;
- acumular os contatos durante o passo;
- publicar um snapshot compacto por pé após `fetchResults()`;
- manter esses eventos separados do áudio.

Um raycast sob o pé pode auxiliar o planejamento, mas não substitui contato
real. Somente o solver sabe se o pé está sustentando peso, escorregando ou
recebendo um impacto.

## 33. Centro de massa

O centro de massa global pode ser calculado com os dados já existentes:

```text
COM = soma(massa_i × posição_COM_i) / massa_total
```

Onde:

```text
posição_COM_i =
    posição_link_i
  + orientação_link_i × centroDeMassaLocal_i
```

A velocidade do COM pode ser obtida pela média ponderada das velocidades nos
centros de massa dos links.

Inicialmente, isso pode ser calculado em um módulo neutro usando:

- `RagdollProfile3D`;
- `RagdollState3D`.

Não é necessário expor PhysX.

## 34. Polígono de suporte real

O FootWork atual calcula um polígono a partir das poses planejadas.

O controlador final deve manter dois conceitos:

1. **suporte planejado:** onde o FootWork espera apoio;
2. **suporte físico:** contatos realmente confirmados pelo solver.

O equilíbrio deve usar o suporte físico.

O planejado serve para previsão e para decidir a próxima passada.

Se o pé deveria estar apoiado, mas o contato físico desapareceu, o sistema
precisa entrar em recuperação, não continuar acreditando no polígono ideal.

## 35. Capture point

Para uma primeira aproximação de equilíbrio, pode ser usado o modelo do
pêndulo invertido linear:

```text
ω₀ = sqrt(g / h)
capturePoint = COM_horizontal + velocidade_COM_horizontal / ω₀
```

Onde `h` é a altura do COM em relação ao suporte.

Interpretação:

- COM dentro do suporte e capture point dentro: situação recuperável sem
  passo;
- COM dentro, capture point saindo: um passo provavelmente será necessário;
- ambos fora: recuperação urgente ou queda.

O capture point pode alimentar o FootWork como correção do alvo:

```text
alvo_final =
    alvo_de_marcha
  + correção_de_equilíbrio
```

Essa correção deve ser limitada, suavizada e sujeita a terreno válido.

## 36. Estratégias de equilíbrio

Uma progressão robusta:

### 36.1 Estratégia do tornozelo

Para erros pequenos:

- ajustar tornozelos;
- inclinar levemente o corpo;
- manter os dois pés plantados.

### 36.2 Estratégia do quadril

Para erros médios:

- mover pelve e tronco em sentidos coordenados;
- usar braços como contrapeso;
- aumentar torque de quadris e coluna.

### 36.3 Estratégia do passo

Para erros grandes:

- escolher o pé livre;
- deslocar o alvo usando capture point;
- reduzir temporariamente rigidez do membro em swing;
- aumentar controle do pé de apoio e tronco.

### 36.4 Queda

Se não houver solução alcançável:

- abandonar a tentativa de manter postura;
- reduzir drives de maneira controlada;
- proteger cabeça e tronco;
- entrar em ragdoll passivo ou parcialmente ativo.

## 37. IK como gerador de referência

O IK não deve mover o ragdoll.

Ele deve responder:

> Quais ângulos articulares aproximam os pés, a pelve e o tronco dos alvos?

Esses ângulos tornam-se referências dos drives.

Para cada perna:

- alvo mundial do pé;
- orientação desejada da sola;
- posição desejada da pelve;
- direção preferida do joelho;
- limites anatômicos;
- pose física atual como ponto inicial.

O solver pode começar analítico para coxa–canela e evoluir para um solver de
corpo inteiro.

O resultado deve ser convertido para `TWIST`, `SWING1` e `SWING2` nos frames
anatômicos do perfil.

## 38. Controle corporal

O futuro controlador não deve usar 100% de rigidez em todos os eixos.

Uma distribuição inicial razoável:

- pé de apoio: tornozelo responsivo, mas com torque limitado;
- perna de apoio: quadril e joelho firmes;
- perna em swing: rigidez moderada e torque baixo;
- tronco: damping alto, stiffness suficiente para orientação;
- braços: controle leve e contrabalanço;
- cabeça: orientação amortecida.

Os ganhos devem variar por estado e por fase.

Também são necessários:

- limite de variação do alvo por tick;
- limite de potência/energia;
- limite de torque já existente;
- redução de rigidez após impactos fortes;
- detecção de saturação;
- proteção contra wind-up.

## 39. Feedback que o FootWork precisará receber

O FootWork atual integra internamente sua raiz e assume que a pose enviada foi
executada.

Para o personagem físico, ele deve receber feedback:

```cpp
struct FootworkFeedback3D {
    Vec3 pelvisPosition;
    Vec3 centerOfMassPosition;
    Vec3 centerOfMassVelocity;
    float bodyYawRadians;
    FootPose3D actualLeftFoot;
    FootPose3D actualRightFoot;
    FootContactState3D leftContact;
    FootContactState3D rightContact;
};
```

Mudanças futuras esperadas:

- casas baseadas na pelve/COM real;
- uma passada só termina após contato confirmado;
- perda de contato dispara recuperação;
- atraso do pé físico modifica a cadência;
- alcance considera comprimento real das pernas;
- raiz virtual deixa de ser fonte autoritativa.

## 40. Estados recomendados do personagem

```text
Passive
GettingUp
Standing
Balancing
Walking
Running
RecoveringStep
Stumbling
Falling
Grounded
```

O estado não deve ser apenas uma animação. Ele define:

- política de passos;
- alvos articulares;
- ganhos;
- torques;
- contatos esperados;
- condição de transição.

## 41. Ordem fixa de atualização

A arquitetura da engine exige:

```text
1. ler snapshot e contatos do passo anterior
2. atualizar intenção locomotora
3. atualizar FootWork
4. estimar equilíbrio
5. resolver IK/pose
6. enfileirar drives e forças
7. PhysicsScene3D::simulate(1/120)
8. fetchResults
9. publicar novos snapshots e contatos
10. renderizar/interpolar
```

Controllers nunca devem escrever transforms físicos.

O feedback possui naturalmente um passo de atraso, de aproximadamente
8,33 ms a 120 Hz. O controlador deve ser projetado para isso.

## 42. Roadmap recomendado

### Etapa 1 — Telemetria física

- calcular COM e velocidade do COM;
- publicar contatos compactos dos pés;
- exibir COM, capture point e suporte real no laboratório.

**Critério:** empurrar o ragdoll deve mover corretamente COM e capture point.

### Etapa 2 — Drives por eixo

- API batch dos 41 DOFs;
- alvos, velocidades e escalas individuais;
- telemetria de saturação.

**Critério:** reproduzir uma pose de referência sem estalo e sem
teletransportes.

### Etapa 3 — Postura em pé

- IK de duas pernas;
- alvo de pelve;
- pés inicialmente fixos;
- controle de tronco.

**Critério:** ficar em pé por 30 segundos em piso plano.

### Etapa 4 — Equilíbrio estático

- suporte real;
- capture point;
- estratégias de tornozelo e quadril;
- recuperação de pequenos empurrões.

**Critério:** resistir a impulsos definidos sem dar passos.

### Etapa 5 — Passo de recuperação

- FootWork recebe correção do capture point;
- contato confirma o fim do passo;
- transição de apoio.

**Critério:** recuperar-se de empurrões maiores com um passo.

### Etapa 6 — Caminhada

- velocidade desejada;
- alternância;
- IK contínuo;
- foot locking físico;
- braços como contrapeso.

**Critério:** caminhar, parar, inverter e virar sem foot skating.

### Etapa 7 — Corrida

- scheduler com fase aérea real;
- impulsão física;
- cadência dependente da velocidade;
- aterrissagem e absorção;
- transição caminhada–corrida.

**Critério:** correr sem depender da raiz cinemática do protótipo.

### Etapa 8 — Futebol e contatos

- tackles;
- disputa de ombro;
- chutes;
- recuperação de tropeços;
- controle orientado à bola;
- 22 jogadores ativos.

**Critério:** estabilidade e desempenho sob contatos simultâneos.

## 43. Testes que devem existir antes da integração completa

### Unitários

- COM conhecido para uma pose conhecida;
- capture point;
- hull do suporte físico;
- conversão de alvo articular;
- clamps de torque e limites;
- determinismo.

### Funcionais

- ficar em pé;
- inclinação estática;
- piso inclinado;
- perda de um contato;
- empurrões em oito direções;
- caminhada e parada;
- inversão brusca;
- curvas;
- corrida;
- tropeço.

### Escala

- 22 ragdolls ativos;
- todos equilibrando;
- contatos entre jogadores;
- contatos com bola;
- nenhum estado não finito;
- tempo de física dentro do orçamento.

## 44. O que já pode ser aproveitado

### Do ragdoll

- articulation reduced-coordinate;
- perfil anatômico;
- limites;
- massas;
- self-collision;
- torque máximo;
- safe point de comandos;
- snapshots;
- Physgun;
- solver e testes de escala.

### Do FootWork

- planejamento independente;
- casas dos pés;
- escolha do pé;
- terreno;
- anticruzamento;
- trajetórias;
- caminhada;
- corrida prototipada;
- replanejamento;
- polígono planejado;
- determinismo.

## 45. O que ainda não existe

- contato semântico dos pés;
- suporte físico;
- COM/capture point em runtime;
- equilíbrio;
- IK;
- controle por DOF;
- controle de pelve;
- torque ou força por link;
- pose corporal completa;
- fase aérea física;
- recuperação de queda;
- integração FootWork–ragdoll.

---

## 46. Conclusão

A base atual é adequada para evoluir para um personagem biomecânico:

- o ragdoll não é uma pilha improvisada de rigid bodies;
- o FootWork não está acoplado à física ou ao render;
- ambos possuem contratos neutros e testes;
- a simulação já trabalha em 120 Hz;
- os 22 ragdolls completos já fazem parte dos testes de escala.

O próximo passo correto não é ligar diretamente os pés do FootWork às
transforms do ragdoll.

O próximo passo correto é construir, nesta ordem:

1. telemetria física dos pés e do centro de massa;
2. comandos de drive por DOF;
3. estimador de equilíbrio;
4. IK como gerador de referência;
5. controlador de ragdoll ativo;
6. adaptação do FootWork para receber feedback físico.

Quando essas camadas existirem, o FootWork decidirá **onde o corpo precisa
pisar**, o equilíbrio decidirá **por que e com que urgência**, o IK decidirá
**quais articulações devem se mover**, e o PhysX decidirá **o que realmente
acontece sob forças, contatos e colisões**.
