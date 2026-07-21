# Autoria física de assets

Este documento define o contrato entre Blender, glTF e a MatterEngine. As
unidades são metros, quilogramas e segundos. O exportador precisa incluir
`extras` para que custom properties sejam preservadas no GLB.

## Exemplo mínimo recomendado

Uma cadeira de madeira com massa total estimada em 8,8 kg:

```text
physical_material = wood
mass = 8.8
collision_mode = auto
```

As propriedades podem estar no objeto ou no material do Blender. Quando ambas
existirem, a propriedade do objeto prevalece.

## Material físico

A chave canônica é `physical_material`. IDs registrados:

| Material | ID | Densidade de referência |
|---|---|---:|
| Padrão/fallback | `default` | 1000 kg/m³ |
| Areia | `sand` | 1600 kg/m³ |
| Concreto | `concrete` | 2400 kg/m³ |
| Grama/solo gramado | `grass` | 1050 kg/m³ |
| Plástico genérico | `plastic` | 950 kg/m³ |
| Tijolo | `brick` | 1900 kg/m³ |
| Madeira genérica | `wood` | 700 kg/m³ |
| Aço | `steel` | 7850 kg/m³ |
| Borracha | `rubber` | 1100 kg/m³ |
| Bola de futebol composta | `soccer_ball` | 78 kg/m³ efetivos |
| Vidro, compatibilidade | `glass` | 2500 kg/m³ |
| Solo, compatibilidade | `soil` | 1600 kg/m³ |

`material_type` é um alias legado; nele, `metal` é migrado para `steel`. Assets
novos devem usar apenas `physical_material`.

## Massa

Props autorados devem declarar a massa total real ou estimada:

```text
mass = 11.0
```

A engine usa esse valor diretamente e calcula o tensor de inércia a partir das
shapes. Isso é mais correto do que inferir massa pelo volume externo: uma cadeira
não é maciça, um barril pode estar vazio e uma roda possui materiais diferentes.

Se `mass` estiver ausente, a massa automática é `densidade × volume dos hulls`.
Esse modo é útil para sólidos homogêneos fechados, geração procedural ou assets
em migração. Para peças ocas ou compostas, a massa explícita é obrigatória para
um resultado fisicamente defensável.

Compatibilidade antiga ainda aceita:

```text
mass_mode = automatic
```

ou:

```text
mass_mode = override
mass_kg = 8.8
```

## Colisão dinâmica

### Regra principal

Triangle meshes côncavas não são usadas em rigid bodies dinâmicos. Elas têm
custo e estabilidade inadequados para props que colidem, empilham e são
manipulados. A forma dinâmica é sempre uma primitiva analítica ou um compound de
hulls convexos compartilhados pelo asset.

### Geração automática

Ausente, `yes` ou `true`:

```text
generate_collision = yes
collision_mode = auto
```

A malha visual é a fonte do cooking V-HACD. O resultado padrão possui no máximo
16 hulls, 64 vértices por hull e resolução de 200.000 voxels. O processamento é
preparado pela ferramenta offline e todos os spawns reutilizam os mesmos recursos
PhysX imutáveis.

Depois de alterar um GLB ou os parâmetros de decomposição, execute:

```powershell
.\tools\cook-physics-assets.cmd -Configuration Release
```

A ferramenta grava `<asset>.glb.mecollider` ao lado do modelo. O arquivo possui
versão, marcador de endianness e hash da geometria/parâmetros. Cache ausente,
antigo ou corrompido é ignorado e reconstruído; nunca é usado para outra revisão
do asset. O runtime ainda sabe reconstruir como fallback de desenvolvimento,
mas builds distribuídas devem incluir os caches aprovados.

O modo automático é uma aproximação de produção, não uma promessa de reproduzir
cada triângulo artístico. Ele deve preservar volumes, apoios, vãos e assentos
relevantes dentro de um orçamento previsível. Se uma região precisa de forma ou
tolerância específica, use collider autorado.

O orçamento pode ser reduzido por asset:

```text
collision_hulls = 8
```

Valores aceitos: 1 a 32. Use o menor valor que preserve as interações reais do
objeto. O catálogo atual usa caixa analítica para a crate, 4 hulls para o barril,
8 para a bigorna e 12 para cadeira e pneu. Aumentar o valor para conservar
detalhes sem função física prejudica contatos e escala da cena.

### Primitivas explícitas

```text
collision_mode = box
collision_mode = sphere
collision_mode = capsule
```

Esses modos criam primitivas analíticas usando os bounds do asset. São a opção
mais rápida e numericamente precisa quando a forma realmente corresponde à
primitiva. A bola de futebol deve usar `sphere`.

### Compound autorado no Blender

Use:

```text
generate_collision = no
collision_mode = manual_compound
```

Inclua no mesmo GLB objetos separados que definam a colisão. Eles são
reconhecidos por uma destas convenções:

- custom property `collision_geometry = true`;
- nome iniciado por `COLLIDER_`, `UCX_`, `UBX_`, `USP_` ou `UCP_`.

Esses objetos não são renderizados. Cada parte é validada e cozida como hulls
convexos; a engine combina as partes no mesmo rigid body. O limite é 32 objetos
autorados e 64 shapes convexas finais. Exceder o orçamento aborta a importação
com erro explícito.

Modele cada parte simples e preferencialmente convexa. Uma cadeira, por exemplo,
pode usar um collider para o assento, quatro para as pernas e dois ou três para
o encosto. Isso permite apoiar uma caixa no assento sem simular milhares de
triângulos do modelo visual.

`Generate Collision` é aceito como alias, mas `generate_collision` é a chave
canônica. Se a geração estiver desativada e nenhum collider autorado existir, o
asset é rejeitado; não há fallback silencioso.

### Cenários estáticos

`collision_mode = static_mesh` é reservado para geometria que nunca se move.
O mapa do laboratório é cozido como `PxTriangleMesh` com BVH34 e preserva a
superfície triangular. Triângulos com arestas acima de 50 metros são tessellados
no mesmo plano para estabilidade, sem aproximar a forma. O catálogo de props
rejeita esse modo.

## Aerodinâmica opcional

Na maioria dos assets, os valores automáticos são suficientes. Overrides:

```text
drag_coefficient = 0.47
drag_area_m2 = 0.038
```

O runtime aplica arrasto quadrático `Fd = ½ρCdAv²`, com densidade padrão do ar
de 1,225 kg/m³. Atualmente a área automática é a maior projeção dos bounds do
asset, portanto constante; ainda não varia com a orientação. Overrides devem
representar medidas ou estimativas físicas, não ajustes visuais arbitrários.

## Acústica opcional

```text
acoustic_structure = hollow
acoustic_gain = 1.0
acoustic_damping = 1.0
```

Estruturas aceitas: `automatic`, `solid`, `hollow`, `thin_shell`, `soft` e
`inflated`. Impactos usam energia transferida, impulso, velocidade pré-solver,
massa efetiva e os dois materiais para calcular volume, pitch, abafamento e
duração. Um contato em repouso não dispara som repetido, pois somente o início
do contato físico publica um evento.

Som de arrasto não é sintetizado a partir de impacto. Ele só deverá retornar
quando houver gravações e regras próprias para cada material.

## Banco de impactos

O runtime procura arquivos em:

```text
assets/audio/materials/<material>/impact/
```

Convenção para expansão:

```text
soft_01.wav
soft_02.wav
medium_01.wav
medium_02.wav
hard_01.wav
hard_02.wav
```

Use WAV PCM, 48 kHz, preferencialmente 24-bit, seco, sem clipping e sem silêncio
inicial. O mínimo recomendado é duas variações por intensidade; quatro reduzem
repetição perceptível. Mantenha a intensidade relativa coerente dentro de cada
material, pois a engine fará a modulação dinâmica final.
