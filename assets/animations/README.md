# Biblioteca de animações

Os clipes ativos, normalizados para `FootballPlayerV1`, ficam em `clips/`.

**Autorais**, criados do zero em código (`tools/animation/`, metodologia em
`docs/ANIMATION_AUTHORING.md`):

- `alert_idle`: única pose parada, baseada diretamente no `Idle.fbx`;
- `jog`: Corrida leve, a passada padrão (3,0 m/s);
- `run_backward`: Corrida de costas, o recuo (2,9 m/s);
- `sprint`: Sprint (7,5 m/s);
- `jump_standing`, `jump_forward`, `jump_backward`, `jump_left`,
  `jump_right`: pulos, tocados pela fase do voo.

**Strafe**, a exceção: `jog_strafe_left/right` e `sprint_strafe_left/right`
são o Jog Strafe Left/Right da Mixamo retargeteado
(`source/retargeted/`), com as correções mínimas que o corpo exige
(`tools/animation/clips/strafe.py`), a pedido do usuário.

**Importados:** `stand_up_back` e `stand_up_front` (levantar de costas e de
frente), que continuam em uso. `cc0_idle` (CC0, Quaternius), `mixamo_running`
e `mixamo_sprint` (Mixamo/Adobe) ficam como referência antiga; o personagem
não os usa mais.

`natural_idle` também fica somente como referência/autoria antiga; não é
declarado pelo personagem nem carregado como pose inicial.

O manifesto do personagem (`locomotion` em `character.json`) diz qual clipe
cumpre cada papel: `idle`, `walk`, `walkBackward`, `sprint`, os de strafe,
os de pulo, `standUpBack` e `standUpFront`. O personagem olha para a câmera;
a direção do movimento em relação a ela escolhe o ciclo (frente e diagonais
da frente: corrida; lados: strafe; trás e diagonais de trás: recuo). Ver
"Olhar × movimento" em `docs/ANIMATION_AUTHORING.md`. A coleção CC0 anterior
(14 clipes) está preservada fora do runtime em
`archive/2026-09-24-retired-animation-library/`.

O menu **ANIMAÇÕES** não enumera este diretório. Ele mostra os clipes do
personagem, na ordem em que o jogo os usa: parado, corrida leve (o papel
`walk`), corrida de costas, sprint, strafes e pulos. As passadas foram
calibradas medindo referências de estudo (`source/mixamo/study/`, ver o
README de lá), sem copiar pose.

## Fontes CC0

- `source/cc0/quaternius_ual1/`: Universal Animation Library, de Quaternius;
- `source/cc0/kaykit_character_animations/`: KayKit Character Animations, de
  Kay Lousberg.

Cada diretório conserva a licença recebida com o pacote. O runtime não lê FBX,
GLB ou glTF diretamente: esses arquivos são fontes auditáveis para regenerar o
formato canônico `matter-ragdoll-animation-1`.

## Importação

O importador humanoide aceita FBX, GLB e glTF, seleciona uma ação nomeada e
retargeteia seus ossos para os IDs estáveis do perfil físico:

```sh
blender --background --factory-startup --python-exit-code 1 \
  --python tools/import_humanoid_animation.py -- ARQUIVO.glb \
  --source-rig kaykit --action Walking_A \
  --profile assets/characters/als_ragdoll/AlsRagdollV1.ragdoll.json \
  --output assets/animations/clips/ID.matteranim.json \
  --id ID --name "Nome exibido" --loop --loop-open \
  --root-motion in-place --nominal-speed 1.4
```

Presets disponíveis: `mixamo`, `kaykit` e `unreal`. `--loop-open` é usado
quando o último quadro da ação ainda precede o primeiro; o exportador acrescenta
o fechamento exato. Para clipes in-place sem deslocamento mensurável, informe
`--nominal-speed`. Quando há root motion horizontal, a velocidade é calculada
automaticamente.

O importador converte eixos e escala, resolve as rotações globais como
coordenadas das juntas físicas, projeta cada amostra nos DOFs e limites do
perfil e remove a viagem da raiz no modo in-place. Cotovelos e joelhos são
medidos pela geometria das cadeias, sem depender do bone roll da fonte.

Além da pose, o arquivo grava o deslocamento original, a velocidade autoral e
curvas escalares de contato dos pés derivadas da altura retargeteada. O runtime
usa essas curvas para liberar o swing antes do arco do passo e permitir foot
lock somente durante stance.

O exportador só grava um clipe após verificar cobertura do rig, fidelidade das
direções, continuidade, limites e fechamento. `--dynamic-projection` permite
projetar poses dinâmicas extremas nos limites anatômicos, mantendo o relatório
de quantas amostras atingiram esses limites.
