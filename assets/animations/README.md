# Biblioteca de animações

Os clipes normalizados para o rig `HumanAdultV1` ficam em `clips/`. O primeiro
é `run_forward.matteranim.json`, retargeteado do `Running.fbx` do Mixamo.

O runtime não dependerá diretamente de FBX, BVH, glTF ou outro formato de
origem. Cada importador converte o arquivo para `AnimationClip3D`, cujos canais
usam os IDs estáveis dos links do perfil físico e guardam translação e rotação
locais em relação à pose neutra. O caminho do arquivo-fonte permanece registrado
no clipe para permitir refazer e auditar o mapeamento.

Para importar outro FBX do Mixamo (sem skin, 30 fps), use o Blender:

```sh
blender --background --factory-startup --python-exit-code 1 \
  --python tools/import_mixamo_animation.py -- ARQUIVO.fbx \
  --profile assets/physics/ragdolls/HumanAdultV1.ragdoll.json \
  --output assets/animations/clips/ID.matteranim.json \
  --id ID --name "Nome exibido" --loop --root-motion in-place
```

O importador mapeia os 18 links, converte eixos e escala, transforma rotações
globais do Mixamo em deltas locais do rig físico, projeta cada alvo nos DOFs e
limites articulares do perfil e remove a deriva linear da raiz. Assim,
inclusive animações baixadas com deslocamento são exibidas paradas no centro,
preservando o movimento interno da passada.

O playback reconstrói cada peça pelas âncoras compartilhadas da articulation:
uma animação nunca pode introduzir distância entre o lado pai e o lado filho
de uma junta. A projeção de limites também é repetida no runtime depois da
interpolação, para manter essa garantia entre keyframes.

Cotovelo e joelho são tratados como hinges semânticos. A flexão é medida pela
geometria das cadeias braço–antebraço e coxa–canela no FBX e então escrita no
eixo `twist` permitido pelo perfil. Isso evita depender do bone roll particular
do Mixamo, que não coincide com a base local do `HumanAdultV1`.

O formato `matter-ragdoll-animation-1` grava a raiz separadamente e, para cada
filho, os escalares físicos `twist/swing1/swing2` — não quaternions locais
livres. O importador executa IK angular limitado e só grava o arquivo se os
gates de fidelidade, continuidade, cobertura, limites e fechamento passarem.
Clipes reprovados não entram na biblioteca.

Use `--loop` apenas para ações cíclicas. `--root-motion in-place` remove a
deriva da raiz para locomover via física; `--root-motion preserve` conserva a
trajetória original. Para ações não cíclicas, omita `--loop`.
