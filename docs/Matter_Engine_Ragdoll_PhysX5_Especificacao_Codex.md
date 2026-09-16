# Matter Engine — Especificação do Ragdoll Humano Biomecânico com PhysX 5

**Documento de implementação para o Codex**  
**Projeto:** Matter Engine / Soccer Fall  
**Escopo desta etapa:** ragdoll humano passivo, limites articulares, arquitetura modular, ferramenta de rigidez global e testes de estresse  
**Data da especificação:** 24 de julho de 2026

---

## 1. Objetivo

Implementar na Matter Engine um sistema de ragdoll humano robusto, modular e orientado a dados, usando **NVIDIA PhysX 5** e `PxArticulationReducedCoordinate`.

Esta etapa não deve implementar ainda:

- footwork;
- equilíbrio ativo;
- caminhada;
- controle muscular completo;
- recuperação de quedas;
- tomada de decisão;
- animação procedural de futebol;
- lesões, ruptura de juntas ou dano biomecânico.

O resultado desta etapa deve ser um corpo passivo que:

1. caia e role naturalmente;
2. respeite limites articulares humanos aproximados;
3. não apresente separação entre membros;
4. não permita hiperextensões absurdas;
5. possa ser configurado sem recompilar a engine;
6. tenha um controle de rigidez global de `0` a `100`;
7. esteja preparado para receber controladores biomecânicos posteriormente;
8. não seja implementado dentro de um arquivo monolítico ou acoplado ao editor.

---

## 2. Decisões obrigatórias

### 2.1 Representação física

Usar:

```cpp
physx::PxArticulationReducedCoordinate
```

Não construir o ragdoll principal com uma coleção de `PxRigidDynamic` conectados por `PxD6Joint`, salvo em experimentos isolados.

Motivos:

- articulações em coordenadas reduzidas evitam separação de juntas por construção;
- suportam melhor cadeias articuladas;
- lidam melhor com relações de massa diferentes;
- oferecem drives, limites, atrito e estados por grau de liberdade;
- serão mais adequadas ao futuro controlador de equilíbrio e locomoção.

### 2.2 Pelve como raiz

A pelve será o root link flutuante:

```text
Pelvis [root]
```

Não fixar a raiz ao mundo. O personagem precisa cair, receber impulsos e ser transportado integralmente pela simulação.

### 2.3 Limites e rigidez são sistemas diferentes

Os limites anatômicos devem existir mesmo com rigidez `0`.

- **Limite articular:** impede posições incompatíveis com a anatomia proposta.
- **Rigidez:** resistência ativa/passiva ao movimento dentro dos limites.
- **Travamento:** não faz parte do slider. Caso seja necessário, criar uma ação de debug separada.

O slider de rigidez nunca deve:

- transformar eixos em `eLOCKED`;
- reduzir o intervalo angular;
- teleportar links;
- alterar diretamente poses dos links;
- recriar a articulation a cada mudança;
- usar a pose visual do mesh como fonte de verdade.

### 2.4 Configuração orientada a dados

Todas as seguintes informações devem ficar em um perfil serializável:

- hierarquia;
- frames de juntas;
- formas de colisão;
- massas;
- centros de massa;
- limites por eixo;
- tipo de junta;
- drive máximo por eixo;
- filtros de autocolisão;
- nomes dos ossos visuais associados;
- pose neutra;
- multiplicadores de rigidez.

Criar inicialmente:

```text
Assets/Physics/Ragdolls/HumanAdultV1.ragdoll.json
```

O nome e o caminho podem ser adaptados ao padrão já usado na Matter Engine, mas os dados não devem ficar espalhados em código C++.

---

## 3. Estrutura física inicial

Usar **20 links físicos**, sem dedos das mãos e sem clavículas físicas separadas.

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
│           └── LeftToes
└── RightThigh
    └── RightShin
        └── RightFoot
            └── RightToes
```

### 3.1 Links

| ID lógico | Link | Pai | Forma inicial |
|---|---|---|---|
| 0 | Pelvis | nenhum | convex simples ou caixa arredondada |
| 1 | Abdomen | Pelvis | cápsula curta ou convex |
| 2 | Chest | Abdomen | convex simples |
| 3 | UpperChest | Chest | convex simples |
| 4 | Neck | UpperChest | cápsula curta |
| 5 | Head | Neck | esfera, cápsula curta ou convex |
| 6 | LeftUpperArm | UpperChest | cápsula |
| 7 | LeftForearm | LeftUpperArm | cápsula |
| 8 | LeftHand | LeftForearm | caixa arredondada |
| 9 | RightUpperArm | UpperChest | cápsula |
| 10 | RightForearm | RightUpperArm | cápsula |
| 11 | RightHand | RightForearm | caixa arredondada |
| 12 | LeftThigh | Pelvis | cápsula |
| 13 | LeftShin | LeftThigh | cápsula |
| 14 | LeftFoot | LeftShin | convex ou caixa arredondada |
| 15 | LeftToes | LeftFoot | caixa curta |
| 16 | RightThigh | Pelvis | cápsula |
| 17 | RightShin | RightThigh | cápsula |
| 18 | RightFoot | RightShin | convex ou caixa arredondada |
| 19 | RightToes | RightFoot | caixa curta |

### 3.2 Clavículas

Não criar clavículas físicas no perfil inicial.

O movimento escapular e clavicular será inicialmente absorvido pelo intervalo do ombro. O rig visual da clavícula pode usar skinning ou uma regra visual derivada de `UpperChest` e `UpperArm`.

Preparar a arquitetura para um perfil futuro:

```text
HumanAdultDetailed
UpperChest → Clavicle → UpperArm
```

Não adicionar esse detalhe antes de validar estabilidade, custo e benefício visual.

---

## 4. Sistema de coordenadas anatômicas

Não espalhar dependências de `X`, `Y` e `Z` diretamente nas regras anatômicas.

Criar e usar eixos semânticos:

```cpp
enum class AnatomicalAxis : uint8_t {
    FlexionExtension,
    AbductionAdduction,
    InternalExternalRotation
};
```

O adaptador PhysX deve converter esses eixos para:

```cpp
physx::PxArticulationAxis::eTWIST
physx::PxArticulationAxis::eSWING1
physx::PxArticulationAxis::eSWING2
```

A associação exata depende dos frames locais de pai e filho.

### 4.1 Regra para frames

Cada junta precisa possuir:

- `parentFrame`;
- `childFrame`;
- orientação neutra;
- origem coincidente com o centro articular;
- eixo primário alinhado à rotação anatômica principal;
- versão espelhada corretamente para o lado direito e esquerdo.

Não assumir que espelhar apenas a posição é suficiente. A orientação e os sinais dos limites também precisam ser espelhados.

### 4.2 Convenção recomendada no perfil

Armazenar limites semanticamente, em graus:

```cpp
struct AngularRangeDeg {
    float negative;
    float positive;
};
```

Exemplo:

```text
Hip.FlexionExtension:
negative = 20   // extensão
positive = 125  // flexão
```

O compilador de perfil converte graus para radianos apenas ao criar ou atualizar o backend PhysX.

### 4.3 Pose neutra

O sistema deve aceitar uma pose neutra autorada. Para o perfil inicial:

- tronco ereto;
- cabeça alinhada;
- braços com aproximadamente `15°` a `25°` de abdução;
- cotovelos com `3°` a `8°` de flexão;
- quadris neutros;
- joelhos com `2°` a `5°` de flexão;
- tornozelos em posição neutra;
- dedos do pé neutros.

Não exigir T-pose como pose física de referência.

---

## 5. Limites articulares

## 5.1 Interpretação dos valores

Os valores anatômicos encontrados na literatura variam conforme:

- movimento ativo ou passivo;
- idade;
- sexo;
- posição usada na medição;
- estabilização dos segmentos adjacentes;
- população estudada;
- método de medição;
- flexibilidade individual.

Portanto, este documento apresenta:

1. **faixa anatômica típica:** referência humana aproximada;
2. **limite Matter V1:** valor recomendado para o primeiro ragdoll;
3. **observações de implementação:** simplificações necessárias.

Os valores Matter V1 não são uma especificação clínica. Eles são uma aproximação de engenharia para um personagem de jogo fisicamente plausível.

---

## 5.2 Coluna adaptada

A coluna humana real possui múltiplos segmentos. O ragdoll usará três juntas toracolombares:

```text
Pelvis → Abdomen
Abdomen → Chest
Chest → UpperChest
```

### Distribuição recomendada

| Junta | Flexão | Extensão | Inclinação lateral | Rotação axial |
|---|---:|---:|---:|---:|
| Pelvis → Abdomen | 25° | 15° | 12° por lado | 8° por lado |
| Abdomen → Chest | 20° | 12° | 10° por lado | 12° por lado |
| Chest → UpperChest | 15° | 10° | 10° por lado | 15° por lado |
| **Total aproximado** | **60°** | **37°** | **32° por lado** | **35° por lado** |

### Regras

- Usar juntas esféricas com três DOFs limitados.
- Não permitir translação.
- Não concentrar toda a flexão em uma única junta.
- O segmento inferior deve suportar mais flexão/extensão.
- O segmento superior pode contribuir mais para rotação.
- Começar com esses limites conservadores e ajustar visualmente.
- A flexão total do corpo também depende do quadril; não tentar reproduzir toda a inclinação para frente apenas pela coluna.

### Limites compostos futuros

Limites independentes podem permitir combinações extremas, como flexão máxima, inclinação máxima e rotação máxima simultaneamente. Preparar uma interface opcional para um envelope de pose, mas não bloquear a primeira entrega por isso.

```cpp
struct JointLimitEnvelope {
    bool enabled;
    float cornerScale;
};
```

Na primeira versão, usar limites independentes conservadores.

---

## 5.3 Pescoço e cabeça

Usar duas juntas:

```text
UpperChest → Neck
Neck → Head
```

Valores de referência cervical publicados frequentemente ficam próximos de:

- flexão total: aproximadamente `50°`;
- extensão total: aproximadamente `50°`;
- inclinação lateral total: aproximadamente `40°` por lado;
- rotação total: aproximadamente `64°` por lado.

### Distribuição Matter V1

| Junta | Flexão | Extensão | Inclinação lateral | Rotação |
|---|---:|---:|---:|---:|
| UpperChest → Neck | 15° | 15° | 15° por lado | 20° por lado |
| Neck → Head | 35° | 35° | 25° por lado | 45° por lado |
| **Total** | **50°** | **50°** | **40° por lado** | **65° por lado** |

### Regras

- As duas juntas serão esféricas.
- A cabeça não deve girar livremente sobre o próprio eixo.
- Evitar collider do pescoço grande demais, pois ele pode empurrar cabeça e tórax.
- Desativar colisão entre `UpperChest × Neck` e `Neck × Head`.
- Avaliar também `Chest × Head` para evitar contatos internos em flexão extrema.

---

## 5.4 Ombro

No perfil inicial, o ombro representa de forma agregada:

- articulação glenoumeral;
- contribuição escapulotorácica;
- parte do movimento clavicular.

### Referência anatômica típica

| Movimento | Faixa típica |
|---|---:|
| Flexão | 160° a 180° |
| Extensão | 45° a 60° |
| Abdução | 150° a 180° |
| Adução | 30° a 50° |
| Rotação interna | 70° a 90° |
| Rotação externa | aproximadamente 90° |

### Matter V1

| Movimento | Limite |
|---|---:|
| Flexão | 165° |
| Extensão | 45° |
| Abdução | 155° |
| Adução | 25° |
| Rotação interna | 70° |
| Rotação externa | 90° |

### Tipo PhysX

```text
PxArticulationJointType::eSPHERICAL
3 DOFs limitados
```

### Regras

- Usar um frame que deixe o twist alinhado ao eixo longitudinal do úmero.
- Verificar a pose com debug draw, não apenas pelo mesh.
- Desativar colisão imediata entre `UpperChest × UpperArm`.
- Manter colisão entre braço e tronco quando houver distância suficiente.
- Os limites independentes do PhysX não reproduzem perfeitamente o envelope real do ombro.
- Se combinações de elevação e rotação causarem poses absurdas, reduzir inicialmente a rotação interna em elevações altas ou aplicar um envelope futuro.
- Não resolver esse problema aumentando exageradamente a rigidez.

---

## 5.5 Cotovelo e rotação do antebraço

Anatomicamente, flexão/extensão e pronação/supinação não pertencem a uma única dobradiça simples. Para evitar mais um link físico, a versão inicial agregará os movimentos na junta entre braço e antebraço.

### Referência anatômica típica

| Movimento | Faixa típica |
|---|---:|
| Flexão do cotovelo | 130° a 154° |
| Hiperextensão | aproximadamente 0° a 6° em parte da população |
| Pronação | 75° a 85° |
| Supinação | 80° a 95° |

### Matter V1

| Movimento | Limite |
|---|---:|
| Flexão | 145° |
| Hiperextensão | 2° |
| Pronação | 80° |
| Supinação | 90° |
| Movimento lateral | bloqueado |

### Tipo PhysX

```text
PxArticulationJointType::eSPHERICAL
2 DOFs liberados e limitados
1 DOF bloqueado
```

### Alternativa simplificada

Caso a junta esférica de dois DOFs apresente instabilidade ou pose visual ruim:

```text
V1 simplificada: cotovelo revoluto com flexão apenas
V1.1: adicionar rotação do antebraço
```

Não criar um link minúsculo e quase sem massa apenas para a pronação sem antes testar relações de massa e inércia.

---

## 5.6 Punho

### Referência anatômica típica

| Movimento | Faixa típica |
|---|---:|
| Flexão | 70° a 80° |
| Extensão | 60° a 75° |
| Desvio radial | 15° a 20° |
| Desvio ulnar | 30° a 40° |
| Twist axial | não deve vir do punho |

### Matter V1

| Movimento | Limite |
|---|---:|
| Flexão | 70° |
| Extensão | 60° |
| Desvio radial | 20° |
| Desvio ulnar | 30° |
| Twist axial | bloqueado |

### Tipo PhysX

```text
PxArticulationJointType::eSPHERICAL
2 DOFs limitados
1 DOF bloqueado
```

A rotação da palma deve ocorrer principalmente na rotação do antebraço.

---

## 5.7 Quadril

### Referência anatômica típica

| Movimento | Faixa típica |
|---|---:|
| Flexão | 120° a 135° |
| Extensão | 15° a 25° |
| Abdução | 40° a 50° |
| Adução | 25° a 30° |
| Rotação interna | 30° a 45° |
| Rotação externa | 40° a 60° |

### Matter V1

| Movimento | Limite |
|---|---:|
| Flexão | 125° |
| Extensão | 20° |
| Abdução | 45° |
| Adução | 25° |
| Rotação interna | 35° |
| Rotação externa | 45° |

### Tipo PhysX

```text
PxArticulationJointType::eSPHERICAL
3 DOFs limitados
```

### Regras

- A origem deve ficar no centro aproximado da cabeça femoral.
- O twist deve seguir o eixo longitudinal da coxa.
- Os limites precisam ser espelhados entre os lados.
- Desativar colisão imediata entre `Pelvis × Thigh`.
- Manter colisão entre coxa esquerda e direita, salvo se a geometria inicial gerar sobreposição contínua.
- O quadril é crítico para chutes, equilíbrio e recuperação; não reduzir os DOFs para uma dobradiça.

---

## 5.8 Joelho

O primeiro joelho deve priorizar estabilidade.

### Referência anatômica típica

| Movimento | Faixa típica |
|---|---:|
| Flexão | aproximadamente 138° a 145° em adultos jovens |
| Extensão | 0° |
| Hiperextensão | pequena e variável |
| Rotação axial | dependente da flexão do joelho |
| Varo/valgo | muito pequeno em um joelho saudável |

### Matter V1

| Movimento | Limite |
|---|---:|
| Flexão | 145° |
| Hiperextensão | 2° |
| Rotação axial | bloqueada |
| Varo/valgo | bloqueado |

### Tipo PhysX

```text
PxArticulationJointType::eREVOLUTE
1 DOF limitado
```

### Justificativa

A rotação axial do joelho depende da pose e não deve ser representada inicialmente por um limite independente constante. Uma junta esférica permitiria rotação excessiva com a perna estendida.

Preparar uma extensão futura:

```text
KneeAdvanced:
twist permitido conforme o ângulo de flexão
```

Não implementar isso nesta etapa.

---

## 5.9 Tornozelo

O tornozelo inicial agregará articulação talocrural e parte do movimento subtalar.

### Referência anatômica típica

| Movimento | Faixa típica |
|---|---:|
| Dorsiflexão | 13° a 20° |
| Flexão plantar | 45° a 60° |
| Inversão | 25° a 35° |
| Eversão | 10° a 20° |

### Matter V1

| Movimento | Limite |
|---|---:|
| Dorsiflexão | 20° |
| Flexão plantar | 45° |
| Inversão | 25° |
| Eversão | 15° |
| Twist axial | bloqueado |

### Tipo PhysX

```text
PxArticulationJointType::eSPHERICAL
2 DOFs limitados
1 DOF bloqueado
```

### Regras

- O pé precisa ter uma base plantar estável.
- Não usar cápsula como collider principal do pé.
- Verificar o eixo de inversão/eversão nos dois lados.
- Desativar colisão imediata entre `Shin × Foot`.
- O tornozelo será uma das principais juntas do futuro controlador de equilíbrio; manter seus eixos e frames rigorosamente definidos.

---

## 5.10 Dedos do pé

Usar um único link para o antepé/dedos.

### Referência anatômica

A primeira metatarsofalângica pode apresentar amplitude passiva grande. A dorsiflexão funcional usada durante a marcha é menor que alguns valores passivos máximos, mas a literatura frequentemente considera aproximadamente `60°` a `65°` importante para a fase final da passada.

### Matter V1

| Movimento | Limite |
|---|---:|
| Extensão/dorsiflexão | 60° |
| Flexão plantar | 25° |
| Movimentos laterais | bloqueados |

### Tipo PhysX

```text
PxArticulationJointType::eREVOLUTE
1 DOF limitado
```

### Regras

- A junta deve ficar próxima à linha das cabeças metatarsais.
- O collider dos dedos não deve penetrar o collider do pé na pose neutra.
- Desativar colisão imediata entre `Foot × Toes`.

---

## 5.11 Resumo dos tipos e DOFs

| Junta por lado/região | Tipo | DOFs |
|---|---|---:|
| Pelvis → Abdomen | esférica | 3 |
| Abdomen → Chest | esférica | 3 |
| Chest → UpperChest | esférica | 3 |
| UpperChest → Neck | esférica | 3 |
| Neck → Head | esférica | 3 |
| UpperChest → UpperArm | esférica | 3 |
| UpperArm → Forearm | esférica | 2 |
| Forearm → Hand | esférica | 2 |
| Pelvis → Thigh | esférica | 3 |
| Thigh → Shin | revoluta | 1 |
| Shin → Foot | esférica | 2 |
| Foot → Toes | revoluta | 1 |

Estimativa total:

```text
43 DOFs articulares
6 DOFs da raiz flutuante
49 DOFs de estado físico total
```

---

## 6. Configuração de limites no PhysX

A configuração deve usar a API não obsoleta da versão instalada.

Exemplo conceitual:

```cpp
joint->setJointType(physx::PxArticulationJointType::eREVOLUTE);
joint->setMotion(axis, physx::PxArticulationMotion::eLIMITED);
joint->setLimitParams(axis, physx::PxArticulationLimit(toRad(-2.0f), toRad(145.0f)));
```

Para junta esférica:

```cpp
joint->setJointType(physx::PxArticulationJointType::eSPHERICAL);
joint->setMotion(physx::PxArticulationAxis::eTWIST, physx::PxArticulationMotion::eLIMITED);
joint->setMotion(physx::PxArticulationAxis::eSWING1, physx::PxArticulationMotion::eLIMITED);
joint->setMotion(physx::PxArticulationAxis::eSWING2, physx::PxArticulationMotion::eLOCKED);
```

Não usar APIs marcadas como deprecated quando `setDriveParams` ou equivalentes atuais estiverem disponíveis.

### 6.1 Validação obrigatória

Antes de adicionar a articulation à cena:

- validar `lower < upper`;
- validar valores finitos;
- validar intervalos esféricos dentro de `[-π, π]`;
- validar que frames são normalizados;
- validar que pai e filho existem;
- validar que não há ciclos;
- validar uma única raiz;
- validar massas positivas;
- validar inércias não nulas;
- validar colliders sem escala zero;
- emitir erro claro com nome da junta e do perfil.

---

## 7. Sistema de rigidez global `0–100`

## 7.1 Comportamento esperado

Adicionar um botão no toolbar superior do editor. Ao abrir:

```text
Ragdoll
Rigidez global: [ 0 ---------------- 100 ]  valor
[Capturar pose] [Pose neutra] [Soltar]
```

### Estados semânticos

| Valor | Interpretação |
|---:|---|
| 0 | flácido/passivo |
| 25 | solto, pouca resistência |
| 50 | resistência média |
| 75 | rígido |
| 100 | muito rígido, mas ainda físico |

O valor `100` não deve transformar o corpo em objeto cinemático nem travar as juntas.

### Alvo da rigidez

Ao ativar rigidez:

1. capturar as posições articulares atuais;
2. usar essas posições como `driveTarget`;
3. aumentar stiffness, damping e torque máximo conforme o slider;
4. manter limites anatômicos ativos.

Isso evita que o corpo seja puxado violentamente para uma T-pose.

### Botões

- **Capturar pose:** substitui os targets pela pose física atual.
- **Pose neutra:** usa os targets autorados do perfil.
- **Soltar:** define rigidez `0`, desativa drives e mantém limites.
- Uma ação futura de **Travar** deve ser separada e não faz parte desta tarefa.

---

## 7.2 Modelo de drive

Usar `PxArticulationDrive` por eixo.

Preferência inicial:

```cpp
physx::PxArticulationDriveType::eFORCE
```

Isso permite trabalhar com limites de torque explícitos. A flag que define se `maxForce` representa força/torque ou impulso deve ser configurada de maneira consistente para a articulation.

Exemplo:

```cpp
physx::PxArticulationDrive drive;
drive.stiffness = stiffness;
drive.damping = damping;
drive.maxForce = maxTorque;
drive.driveType = physx::PxArticulationDriveType::eFORCE;
joint->setDriveParams(axis, drive);
joint->setDriveTarget(axis, targetRadians);
joint->setDriveVelocity(axis, 0.0f);
```

### 7.3 Curva do slider

Não usar interpolação linear simples para stiffness. Uma escala linear tornaria a região baixa do slider sensível demais.

```cpp
float normalized = std::clamp(value / 100.0f, 0.0f, 1.0f);
float stiffnessScale = std::pow(normalized, 2.2f);
float dampingScale = std::pow(normalized, 1.4f);
float torqueScale = std::pow(normalized, 1.25f);
```

Aplicação:

```cpp
float stiffness = tuning.stiffnessAt100 * stiffnessScale;
float damping = tuning.dampingAt100 * dampingScale;
float maxTorque = tuning.maxTorqueAt100 * torqueScale;
```

Quando `value == 0`:

```text
stiffness = 0
damping = 0
maxForce = 0
drive target preservado, mas drive sem efeito
```

Manter apenas:

- damping mínimo dos links, se necessário;
- atrito articular passivo baixo;
- limites anatômicos;
- contatos.

### 7.4 Parâmetros iniciais de debug

Valores abaixo são somente pontos de partida para um personagem de aproximadamente `75 kg`, usando metros, quilogramas e segundos.

| Região | Stiffness em 100 | Damping em 100 | Torque máximo em 100 |
|---|---:|---:|---:|
| Coluna inferior | 1000 Nm/rad | 100 Nm/(rad/s) | 350 Nm |
| Coluna média | 800 | 80 | 280 Nm |
| Coluna superior | 650 | 65 | 220 Nm |
| Pescoço base | 180 | 20 | 65 Nm |
| Cabeça | 140 | 18 | 55 Nm |
| Ombro | 550 | 55 | 180 Nm |
| Cotovelo | 380 | 38 | 130 Nm |
| Punho | 110 | 14 | 45 Nm |
| Quadril | 1000 | 100 | 380 Nm |
| Joelho | 850 | 85 | 330 Nm |
| Tornozelo | 550 | 55 | 210 Nm |
| Dedos do pé | 120 | 14 | 45 Nm |

Armazenar esses valores por eixo no perfil. Não codificá-los em um grande `switch`.

### 7.5 Multiplicadores por eixo

Cada eixo deve poder possuir um multiplicador:

```cpp
struct JointDriveTuning {
    float stiffnessAt100;
    float dampingAt100;
    float maxTorqueAt100;
    float rigidityMultiplier;
};
```

Exemplos:

- quadril flexão/extensão: `1.0`;
- quadril twist: `0.65`;
- tornozelo inversão/eversão: `0.65`;
- punho: `0.4` a `0.6` em relação aos membros principais;
- rotação da coluna: menor que flexão/extensão.

### 7.6 Estabilidade

Drives muito fortes em contato com o chão e limites rígidos podem gerar conflito entre constraints.

Ao detectar:

- vibração;
- ganho artificial de energia;
- limite sendo ultrapassado;
- contato explosivo;
- membros tremendo em rigidez alta;

ajustar nesta ordem:

1. reduzir `maxTorque`;
2. reduzir stiffness;
3. aumentar damping com moderação;
4. reduzir timestep;
5. aumentar iterações de posição;
6. revisar sobreposição de colliders;
7. revisar relações de massa e inércia;
8. verificar targets fora dos limites.

Não usar `100` como sinônimo de stiffness infinita.

---

## 8. Toolbar e comunicação com a física

## 8.1 Separação obrigatória

A UI não pode possuir ou manipular diretamente:

```cpp
physx::PxArticulationReducedCoordinate*
physx::PxArticulationJointReducedCoordinate*
physx::PxArticulationLink*
```

A UI envia comandos de alto nível.

Exemplo:

```cpp
struct SetRagdollRigidityCommand {
    EntityId entity;
    float value;
};

struct CaptureRagdollPoseCommand {
    EntityId entity;
};

struct SetRagdollNeutralPoseCommand {
    EntityId entity;
};
```

### Fluxo

```text
Toolbar
→ Editor command/event
→ Ragdoll debug service
→ fila do physics world
→ safe point após fetchResults e antes do próximo simulate
→ RagdollRigidityController
→ backend PhysX
```

Várias funções de configuração do PhysX não podem ser chamadas enquanto a simulação está rodando. Todas as alterações devem ocorrer em um ponto de sincronização seguro.

### Seleção

Comportamento recomendado:

- se houver um ragdoll selecionado, afetar apenas o selecionado;
- se o modo de teste tiver um ragdoll principal, usá-lo como fallback;
- não alterar todos os personagens do mundo silenciosamente;
- permitir posteriormente um checkbox explícito para aplicar aos selecionados.

---

## 9. Arquitetura recomendada

Adaptar os caminhos ao padrão da Matter Engine, preservando as responsabilidades.

```text
Source/Matter/Character/Ragdoll/Core/
    RagdollTypes.h
    RagdollDefinition.h
    RagdollDefinition.cpp
    RagdollProfileLoader.h
    RagdollProfileLoader.cpp
    RagdollValidator.h
    RagdollValidator.cpp

Source/Matter/Character/Ragdoll/Runtime/
    RagdollComponent.h
    RagdollInstance.h
    RagdollInstance.cpp
    RagdollPose.h
    RagdollPose.cpp
    RagdollRigidityController.h
    RagdollRigidityController.cpp
    RagdollRuntimeService.h
    RagdollRuntimeService.cpp

Source/Matter/Physics/PhysX/Ragdoll/
    PhysXRagdollBuilder.h
    PhysXRagdollBuilder.cpp
    PhysXRagdollInstance.h
    PhysXRagdollInstance.cpp
    PhysXRagdollJointAdapter.h
    PhysXRagdollJointAdapter.cpp
    PhysXRagdollCollisionFilter.h
    PhysXRagdollCollisionFilter.cpp
    PhysXRagdollMassProperties.h
    PhysXRagdollMassProperties.cpp

Source/Matter/Editor/Ragdoll/
    RagdollToolbarController.h
    RagdollToolbarController.cpp
    RagdollToolbarView.h
    RagdollToolbarView.cpp
    RagdollDebugDraw.h
    RagdollDebugDraw.cpp
    RagdollInspectorPanel.h
    RagdollInspectorPanel.cpp

Assets/Physics/Ragdolls/
    HumanAdultV1.ragdoll.json

Tests/Ragdoll/
    RagdollDefinitionTests.cpp
    RagdollMirrorTests.cpp
    RagdollLimitTests.cpp
    RagdollRigidityTests.cpp
    RagdollStressScene.cpp
```

### 9.1 Responsabilidades

#### `RagdollDefinition`

Dados puros, sem PhysX e sem editor.

```cpp
struct RagdollDefinition {
    std::string name;
    std::vector<RagdollBodyDefinition> bodies;
    std::vector<RagdollJointDefinition> joints;
    RagdollPose neutralPose;
};
```

#### `RagdollProfileLoader`

- carregar JSON;
- converter para estruturas internas;
- emitir erros com contexto;
- não criar objetos PhysX.

#### `RagdollValidator`

- validar hierarquia;
- validar limites;
- validar massas;
- validar colliders;
- validar espelhamento;
- validar nomes de ossos;
- validar pose neutra.

#### `PhysXRagdollBuilder`

- receber `RagdollDefinition` validado;
- criar articulation;
- criar links;
- criar shapes;
- aplicar massa e inércia;
- criar joints e frames;
- configurar limites;
- configurar filtros;
- retornar uma instância backend.

#### `RagdollInstance`

- representar o ragdoll no runtime;
- manter IDs, estado, targets e handles abstratos;
- não expor detalhes do editor.

#### `RagdollRigidityController`

- armazenar rigidez global;
- capturar pose;
- aplicar pose neutra;
- calcular parâmetros por eixo;
- enviar atualização ao backend;
- nunca editar colliders ou hierarquia.

#### `RagdollToolbarView`

- desenhar botão, popup, slider e comandos;
- não conter matemática de drives;
- não conter parâmetros anatômicos;
- não incluir headers do PhysX.

#### `RagdollDebugDraw`

Desenhar opcionalmente:

- colliders;
- centros de massa;
- centro da junta;
- frames de pai e filho;
- eixos liberados;
- arco dos limites;
- valor angular atual;
- target do drive;
- torque/força recebida;
- contatos;
- nomes dos links.

---

## 10. Interfaces sugeridas

```cpp
using RagdollBodyId = uint16_t;
using RagdollJointId = uint16_t;

enum class RagdollJointKind : uint8_t {
    Revolute,
    Spherical
};

enum class JointMotionMode : uint8_t {
    Locked,
    Limited,
    Free
};

struct JointAxisDefinition {
    AnatomicalAxis semanticAxis;
    JointMotionMode motion;
    AngularRangeDeg limit;
    JointDriveTuning drive;
};

struct RagdollJointDefinition {
    std::string name;
    RagdollBodyId parent;
    RagdollBodyId child;
    RagdollJointKind kind;
    Transform parentFrame;
    Transform childFrame;
    std::array<JointAxisDefinition, 3> axes;
    float passiveFriction;
    float maxAngularVelocity;
};
```

Backend abstrato:

```cpp
class IRagdollPhysicsBackend {
public:
    virtual ~IRagdollPhysicsBackend() = default;
    virtual void setDrive(RagdollJointId joint, AnatomicalAxis axis, const JointDriveState& state) = 0;
    virtual float getJointPosition(RagdollJointId joint, AnatomicalAxis axis) const = 0;
    virtual void setJointTarget(RagdollJointId joint, AnatomicalAxis axis, float radians) = 0;
};
```

Controlador:

```cpp
class RagdollRigidityController {
public:
    void setRigidity(float value);
    void captureCurrentPose();
    void useNeutralPose();
    void apply(IRagdollPhysicsBackend& backend, const RagdollDefinition& definition);

private:
    float m_rigidity = 0.0f;
    RagdollPose m_targetPose;
};
```

Não criar uma classe `HumanRagdoll` que carregue arquivo, crie PhysX, desenhe UI, controle drives, sincronize mesh e execute testes.

---

## 11. Perfil JSON

Exemplo reduzido:

```json
{
  "name": "HumanAdultV1",
  "units": "meters-kilograms-seconds",
  "rootBody": "Pelvis",
  "bodies": [
    {
      "name": "LeftThigh",
      "parent": "Pelvis",
      "visualBone": "thigh_l",
      "massFraction": 0.10,
      "shape": {
        "type": "capsule",
        "radius": 0.09,
        "halfHeight": 0.20
      }
    }
  ],
  "joints": [
    {
      "name": "LeftHip",
      "parent": "Pelvis",
      "child": "LeftThigh",
      "type": "spherical",
      "axes": {
        "flexionExtension": {
          "motion": "limited",
          "negativeDeg": 20.0,
          "positiveDeg": 125.0,
          "stiffnessAt100": 1000.0,
          "dampingAt100": 100.0,
          "maxTorqueAt100": 380.0
        },
        "abductionAdduction": {
          "motion": "limited",
          "negativeDeg": 25.0,
          "positiveDeg": 45.0,
          "stiffnessAt100": 800.0,
          "dampingAt100": 80.0,
          "maxTorqueAt100": 300.0
        },
        "internalExternalRotation": {
          "motion": "limited",
          "negativeDeg": 35.0,
          "positiveDeg": 45.0,
          "stiffnessAt100": 650.0,
          "dampingAt100": 65.0,
          "maxTorqueAt100": 220.0
        }
      }
    }
  ]
}
```

Os nomes dos ossos visuais devem ser configuráveis. Não assumir nomes específicos do Blender.

---

## 12. Massas e inércias

Usar percentuais antropométricos como referência, mas normalizar o perfil para a massa total configurada.

Referência inicial aproximada:

| Segmento | Fração por segmento |
|---|---:|
| Cabeça | 6% a 7% |
| Tronco completo | 42% a 50% |
| Braço superior | 2,5% a 2,8% por lado |
| Antebraço | 1,4% a 1,6% por lado |
| Mão | 0,5% a 0,7% por lado |
| Coxa | aproximadamente 10% a 14% por lado, conforme conjunto antropométrico |
| Canela | aproximadamente 4,3% a 4,8% por lado |
| Pé | aproximadamente 1,3% a 1,5% por lado |

Como o perfil divide o tronco em pelve, abdômen, tórax e tórax superior, distribuir a fração total do tronco entre esses links e renormalizar para `1.0`.

### Regras

- permitir massa total configurável;
- usar densidade/forma para estimar inércia inicial;
- ajustar massa do link sem criar tensores de inércia irreais;
- evitar links com massa quase zero;
- evitar saltos extremos de massa entre pai e filho;
- manter centros de massa dentro ou muito próximos dos colliders;
- registrar warning se qualquer componente do tensor de inércia for não finito ou quase zero.

Para a primeira calibração, usar um personagem de:

```text
altura: 1,75 m a 1,80 m
massa: 75 kg
```

---

## 13. Colliders e autocolisão

### 13.1 Formas

Usar formas simples.

- cápsulas para membros longos;
- convexes simples para pelve e tórax;
- caixa arredondada ou convex para pés;
- caixa curta para dedos;
- caixa arredondada para mãos;
- esfera/cápsula curta para cabeça.

Não usar a malha visual completa como collider dinâmico.

### 13.2 Pares inicialmente desativados

Desativar colisão entre links diretamente conectados:

```text
Pelvis × Abdomen
Abdomen × Chest
Chest × UpperChest
UpperChest × Neck
Neck × Head

UpperChest × UpperArm
UpperArm × Forearm
Forearm × Hand

Pelvis × Thigh
Thigh × Shin
Shin × Foot
Foot × Toes
```

Avaliar também:

```text
Chest × Head
Pelvis × Shin
UpperChest × Forearm
```

somente se houver contatos internos inevitáveis em poses válidas.

### 13.3 Pares importantes a preservar

Tentar preservar:

```text
mão × cabeça
mão × tronco
antebraço × tronco
pé × perna oposta
joelho × perna oposta
braço esquerdo × braço direito
perna esquerda × perna direita
```

Ativar autocolisão por grupos progressivamente.

---

## 14. Configuração da simulação

Começar com:

```text
solver: TGS
timestep fixo: 1/120 s para desenvolvimento do ragdoll
iterações de posição da articulation: 12
iterações de velocidade: 2
```

Esses valores são ponto de partida, não regra definitiva.

A documentação de migração do PhysX 5.2 para 5.3 informa que o comportamento das iterações de velocidade do TGS mudou; evitar números altos de velocity iterations sem justificativa.

### Regras

- simulação com timestep fixo;
- não usar `deltaTime` de render diretamente;
- permitir vários substeps quando o frame atrasar;
- limitar catch-up para evitar espiral de atraso;
- validar em `60 Hz` e `120 Hz`;
- armazenar presets de solver para teste;
- medir custo por ragdoll;
- não otimizar para centenas de bots antes de o ragdoll individual estar correto.

---

## 15. Sincronização visual

A pose física é a fonte de verdade durante o modo ragdoll.

Fluxo:

```text
PhysX link transforms
→ Ragdoll pose buffer
→ conversão para espaço do esqueleto visual
→ skinning
```

Não escrever poses visuais de volta para os links físicos durante a simulação passiva.

Separar:

- bind pose;
- pose neutra física;
- frames articulares;
- transforms de colliders;
- transforms dos ossos visuais.

O collider não precisa ter a mesma origem do osso visual.

---

## 16. Testes obrigatórios

## 16.1 Testes automatizados

### Hierarquia

- possui uma única raiz;
- não possui ciclos;
- todos os pais existem;
- IDs são únicos;
- todos os joints conectam links válidos.

### Limites

- `lower < upper`;
- limites convertidos corretamente para radianos;
- joelho não dobra para trás além de `2°`;
- cotovelo não dobra para trás além de `2°`;
- left/right possuem amplitude equivalente;
- sinais espelhados produzem movimento anatômico equivalente.

### Rigidez

- valor abaixo de `0` vira `0`;
- valor acima de `100` vira `100`;
- rigidez `0` produz drives sem força;
- rigidez `100` usa exatamente os máximos do perfil;
- curva é monotônica;
- `Capturar pose` copia os valores atuais;
- `Pose neutra` copia os targets autorados;
- nenhuma atualização ocorre durante `simulate`.

### Serialização

- carregar e salvar sem perda relevante;
- erro claro em perfil inválido;
- versão de schema;
- suporte futuro a migração.

---

## 16.2 Cena de testes de estresse

Criar uma cena específica, sem depender do Soccer Fall completo.

Controles:

```text
spawn/reset ragdoll
altura de queda
massa total
rigidez 0–100
capturar pose
pose neutra
aplicar impulso
puxar membro
mostrar limites
mostrar contatos
mostrar COM
mostrar torque
slow motion
pausar e avançar um frame
```

### Cenários

1. queda vertical de `1 m`;
2. queda vertical de `3 m`;
3. queda de costas;
4. queda de frente;
5. queda lateral sobre ombro;
6. queda sobre joelho;
7. queda sobre cabeça;
8. rolamento em escada;
9. impacto lateral na pelve;
10. puxar pela mão;
11. puxar pelo pé;
12. colisão entre dois ragdolls;
13. rigidez `0`, `25`, `50`, `75` e `100`;
14. mudar rigidez gradualmente durante uma queda;
15. capturar pose irregular e aumentar rigidez;
16. pressionar o corpo contra uma parede;
17. apoiar o pé e aumentar rigidez;
18. repetir a cena por vários minutos para procurar NaN e ganho de energia.

### Critérios

- nenhuma separação de junta;
- nenhuma pose impossível persistente;
- sem NaN;
- sem velocidades infinitas;
- sem explosão ao aumentar rigidez;
- sem snapping para pose neutra ao usar captura atual;
- limits visuais correspondem ao comportamento;
- left/right simétricos;
- ausência de tremor contínuo em repouso;
- nenhuma alteração da UI chama PhysX durante simulação;
- reset determinístico com a mesma seed/configuração.

---

## 17. Debug draw obrigatório

Adicionar opções independentes:

```text
[ ] Colliders
[ ] Skeleton físico
[ ] Joint frames
[ ] Limites angulares
[ ] Centros de massa
[ ] Targets dos drives
[ ] Ângulos atuais
[ ] Velocidades articulares
[ ] Forças/torques recebidos
[ ] Contatos
[ ] Pares sem autocolisão
```

Cores podem seguir o tema do editor, mas precisam distinguir:

- eixo primário;
- eixo secundário;
- eixo terciário;
- limite mínimo;
- limite máximo;
- target;
- posição atual.

Não depender apenas de cores; incluir labels e setas.

---

## 18. Logging e telemetria

Criar categoria própria:

```text
Ragdoll
Ragdoll.PhysX
Ragdoll.Validation
Ragdoll.Editor
```

Mensagens úteis:

- perfil carregado;
- número de links;
- número de DOFs;
- massa total final;
- par de colisão desativado;
- target fora do limite e clamp realizado;
- drive aplicado fora do safe point;
- tensor de inércia inválido;
- frame não normalizado;
- erro de espelhamento;
- rigidez alterada;
- articulation dormindo/acordando;
- velocidade articular máxima excedida.

Não registrar uma linha por joint a cada frame em build normal.

---

## 19. Ordem de implementação para o Codex

### Fase 0 — Inspeção

Antes de editar:

1. identificar o physics world e seu ciclo `simulate/fetchResults`;
2. localizar o toolbar superior;
3. localizar o sistema de entidades/componentes;
4. localizar serialização de assets;
5. localizar seleção do editor;
6. localizar debug draw;
7. localizar convenção de eixos e unidades;
8. localizar wrappers existentes do PhysX.

Não criar uma segunda arquitetura paralela se a engine já possuir serviços equivalentes.

### Fase 1 — Dados e validação

Implementar:

- tipos de dados;
- loader;
- validator;
- profile schema;
- perfil `HumanAdultV1`;
- testes unitários de dados.

Ainda não criar UI.

### Fase 2 — Backend PhysX

Implementar:

- builder;
- links;
- shapes;
- massas;
- joints;
- frames;
- limites;
- collision filtering;
- safe destruction;
- instância runtime.

Validar ragdoll passivo com rigidez `0`.

### Fase 3 — Sincronização e debug

Implementar:

- pose física;
- sincronização visual;
- debug draw;
- inspector de joints;
- reset da cena.

### Fase 4 — Rigidez

Implementar:

- pose target;
- captura da pose atual;
- pose neutra;
- drive tuning;
- curva `0–100`;
- command queue;
- testes.

### Fase 5 — Toolbar

Implementar:

- botão;
- popup;
- slider;
- valor numérico;
- `Capturar pose`;
- `Pose neutra`;
- `Soltar`;
- target correto conforme seleção.

### Fase 6 — Stress tests

Implementar a cena de testes e corrigir:

- frames;
- sinais;
- colliders;
- massas;
- limites;
- drives;
- filtros.

### Fase 7 — Documentação final

Documentar:

- como criar perfil;
- como alterar limites;
- como adicionar novo link;
- como depurar um eixo invertido;
- como calibrar rigidez;
- como criar um perfil simplificado;
- limitações conhecidas.

---

## 20. Restrições de arquitetura

O Codex não deve:

- adicionar toda a implementação em um arquivo existente;
- colocar lógica de ragdoll no renderer;
- colocar parâmetros anatômicos no toolbar;
- colocar código ImGui dentro do componente físico;
- expor ponteiros PhysX à UI;
- reconstruir o ragdoll ao mover o slider;
- usar nomes de ossos hardcoded no builder;
- modificar sistemas não relacionados sem necessidade;
- usar transform visual como centro articular automaticamente;
- ignorar o thread/safe point da física;
- esconder erros de perfil com valores silenciosos;
- usar stiffness infinita;
- implementar footwork nesta tarefa.

Preferir arquivos pequenos e coesos. Quando uma unidade começar a misturar dados, PhysX, UI, sincronização e testes, separá-la.

---

## 21. Critérios de conclusão

A tarefa estará concluída quando:

1. `HumanAdultV1` cria 20 links físicos;
2. a hierarchy corresponde a este documento;
3. o ragdoll usa `PxArticulationReducedCoordinate`;
4. cada junta possui frames e limites configuráveis;
5. os limites principais funcionam nos dois lados;
6. os joelhos e cotovelos não hiperestendem de forma absurda;
7. pés e dedos possuem links separados;
8. o ragdoll cai naturalmente com rigidez `0`;
9. o botão aparece no toolbar;
10. o slider altera rigidez sem recriar a articulation;
11. a pose atual pode ser capturada como target;
12. a pose neutra pode ser aplicada como target;
13. `Soltar` retorna ao estado passivo;
14. a UI não acessa PhysX diretamente;
15. alterações são aplicadas em safe point;
16. existe debug draw de frames e limites;
17. existe uma cena de testes de estresse;
18. existem testes automatizados essenciais;
19. não há arquivo monolítico concentrando a mecânica;
20. o código deixa pontos claros para footwork e equilíbrio futuros.

---

## 22. Observações para as etapas futuras

A arquitetura criada agora deverá permitir posteriormente:

```text
Ragdoll passivo
→ active ragdoll
→ controle de pose
→ controle de centro de massa
→ suporte dos pés
→ foot placement
→ equilíbrio
→ caminhada
→ corrida
→ chute físico
→ contato jogador-jogador
→ recuperação de quedas
```

A rigidez global desta etapa é uma ferramenta de debug e calibração. Ela não será o controlador biomecânico final.

No controlador futuro:

- cada joint terá targets dinâmicos;
- torque será limitado por capacidade muscular;
- força dependerá de pose, velocidade, fadiga e lesão;
- tornozelo, quadril e coluna atuarão em conjunto;
- o controle não deverá depender de aumentar a rigidez global até `100`.

---

## 23. Fontes de pesquisa

Acesso em 24 de julho de 2026.

1. NVIDIA. **PhysX 5.6 — Articulations**.  
   https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/docs/Articulations.html

2. NVIDIA. **PxArticulationJointReducedCoordinate API**.  
   https://nvidia-omniverse.github.io/PhysX/physx/latest/_api_build/classPxArticulationJointReducedCoordinate.html

3. NVIDIA. **Migrating From PhysX SDK 5.2 to 5.3**.  
   https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/docs/MigrationTo53.html

4. CDC. **Normal Joint Range of Motion Study**.  
   https://archive.cdc.gov/www_cdc_gov/ncbddd/jointrom/index.html

5. Soucie JM et al. **Range of motion measurements: reference values and a database for comparison studies**. Haemophilia.  
   https://pubmed.ncbi.nlm.nih.gov/21070485/

6. Gill TK et al. **Shoulder range of movement in the general population**.  
   https://pmc.ncbi.nlm.nih.gov/articles/PMC7549223/

7. Zwerus EL et al. **Normative values and affecting factors for the elbow range of motion**.  
   https://pmc.ncbi.nlm.nih.gov/articles/PMC6555111/

8. Thoomes-de Graaf M et al. **Normative values of cervical range of motion: systematic review**.  
   https://pubmed.ncbi.nlm.nih.gov/32861355/

9. Vishal K et al. **Cervical spine range of motion values**.  
   https://pmc.ncbi.nlm.nih.gov/articles/PMC10423670/

10. Wilke HJ et al. **Range of Motion and Neutral Zone of All Human Spinal Segments**.  
    https://pmc.ncbi.nlm.nih.gov/articles/PMC11881816/

11. Wu G et al. **ISB recommendation — ankle, hip and spine**.  
    https://pubmed.ncbi.nlm.nih.gov/11934426/

12. Wu G et al. **ISB recommendation — shoulder, elbow, wrist and hand**.  
    https://pubmed.ncbi.nlm.nih.gov/15844264/

13. Koutsouradis P et al. **Arthrodesis of the first metatarsophalangeal joint**, incluindo discussão da amplitude passiva.  
    https://pmc.ncbi.nlm.nih.gov/articles/PMC8316842/

14. Menz HB et al. **First MTP range and gait**, incluindo referência de dorsiflexão funcional.  
    https://pmc.ncbi.nlm.nih.gov/articles/PMC4291455/

15. de Leva / Visual3D documentation. **Adjusted Zatsiorsky–Seluyanov segment inertia parameters**.  
    https://www.has-motion.com/wiki/doku.php?id=visual3d:documentation:definitions:adjusted_zatsiorsky-seluyanov_s_segment_inertia_parameters

---

## 24. Nota final ao Codex

Implementar em incrementos compiláveis.

Após cada fase:

1. compilar;
2. executar testes;
3. abrir a cena de stress;
4. validar visualmente os frames;
5. registrar mudanças;
6. não avançar com eixos invertidos ou perfis inválidos.

Priorizar primeiro:

```text
correção da hierarquia
→ correção dos frames
→ correção dos limites
→ correção de massas/colliders
→ estabilidade passiva
→ rigidez
→ UI
```

Não tentar esconder problemas estruturais usando damping alto ou stiffness alto.
