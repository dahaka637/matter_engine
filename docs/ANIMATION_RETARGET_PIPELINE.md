# Pipeline de retarget de animações

Este documento define a metodologia obrigatória para inserir animações
esqueléticas no `HumanAdultV1`. O objetivo é impedir ajustes manuais por clipe,
poses visualmente enganosas e alvos incompatíveis com a articulation física.

## Princípios invariantes

1. O FBX é referência de movimento, não um rig que o runtime reproduz
   diretamente.
2. O `RagdollProfile3D` é a autoridade sobre hierarquia, âncoras, frames, eixos
   bloqueados e limites angulares.
3. A pose canônica é armazenada no espaço físico das juntas:
   `twist/swing1/swing2`. Apenas a raiz usa translação e quaternion.
4. Links filhos nunca possuem translação animada. A posição deles é sempre
   reconstruída fazendo as âncoras pai e filho coincidirem.
5. Um clipe reprovado pelos gates não pode aparecer na biblioteca.

## Etapas do importador Mixamo

### 1. Inspeção e normalização

O Blender lê o FBX e determina armature, ação, intervalo, FPS, ossos, escala e
base anatômica. A base é inferida por quadris, cabeça e pernas; assim, a
conversão não depende dos flags de eixo escritos pelo exportador.

### 2. Correspondência semântica

Os 18 links do `HumanAdultV1` são associados a ossos Mixamo conhecidos. Para
braços e pernas, o importador usa posições de ombro/cotovelo/punho e
quadril/joelho/tornozelo. Bone roll não participa do plano de flexão.

### 3. IK angular limitado

Cada cadeia de dois segmentos é resolvida contra as direções observadas no
FBX. O solver procura a melhor orientação dentro dos limites do ombro/quadril;
cotovelo e joelho recebem a flexão geométrica no hinge correto. Os demais
links são convertidos ao frame articular e projetados nos DOFs permitidos.

Os frames de cotovelo são espelhados no perfil: a flexão positiva leva os dois
antebraços para a direção anatômica equivalente. Corrigir isso no rig elimina
a necessidade de exceções por animação.

### 4. Root motion

- `--root-motion in-place`: remove a deriva linear da raiz, preservando a
  oscilação interna. É o modo de locomoção dirigida pela física.
- `--root-motion preserve`: conserva a trajetória fornecida pelo arquivo.
- `--loop`: exige fechamento de raiz e juntas; deve ser omitido em ações
  one-shot.

### 5. Formato canônico

`matter-ragdoll-animation-1` contém:

- canal `root` para a pelve;
- 17 canais `joint`, sincronizados e expressos em radianos;
- procedência, duração, FPS e comportamento de loop;
- relatório numérico dos gates de retarget.

Interpolar escalares articulares limitados mantém os quadros entre keyframes
dentro da região válida. Antes de exibir, o runtime ainda reaplica os limites e
reconstrói a cadeia pelas âncoras como defesa adicional.

## Gates obrigatórios

O importador falha com código diferente de zero quando algum limite é
ultrapassado:

| Gate | Limite atual |
|---|---:|
| Erro direcional RMS fonte × alvo | `6°` |
| Maior erro direcional global | `15°` |
| Maior erro em braços/pernas | `8°` |
| Maior passo angular entre amostras | `45°` |
| Diferença articular ao fechar loop | `0,5°` |
| Diferença de rotação da raiz no loop | `0,5°` |
| Diferença de posição da raiz no loop | `1 mm` |

O runtime também rejeita schema incorreto, rig divergente, links ausentes,
canais com tempos diferentes, eixos bloqueados diferentes de zero e valores
fora dos limites do perfil.

## Fluxo para toda animação nova

1. Baixar FBX Binary, Without Skin, 30 fps e sem redução de keyframes.
2. Escolher conscientemente `loop` e política de root motion.
3. Executar o importador com `--python-exit-code 1`.
4. Só aceitar a saída se todos os gates passarem.
5. Rodar `MATTERENGINE_TEST_FILTER=animation`.
6. Conferir o relatório exibido no Visualizador de Animações e inspecionar o
   movimento em frente, perfil, costas e vista superior.

Se um gate falhar, a correção deve ocorrer no adaptador semântico ou no perfil
anatômico. Não se criam números mágicos específicos para um clipe.

## Referência atual: corrida para frente

`Running.fbx` produz 20 quadros a 30 fps. Após a correção dos frames dos
cotovelos e o IK limitado, o resultado atual apresenta:

- erro direcional RMS: `4,67°`;
- maior erro em membros: `6,02°`;
- fechamento articular: `0,0001°`;
- fechamento de raiz: abaixo de `0,000001 m`;
- todas as 17 juntas conectadas e dentro dos limites físicos.
