# Mapas com TrenchBroom

Configurado em 31/10/2026 para TrenchBroom **2026.2**, Linux. O perfil do jogo
se chama **Soccer Fall**. O codigo fonte da integracao fica em `tools/maps/`.
O TrenchBroom edita o cenario; o campo continuara sendo gerado pela engine.

O segundo mapa, aberto e inspirado em Abelardo Luz, foi rejeitado na primeira
autoria e esta sendo refeito do zero: **somente campo, arquibancada/bloco
5x20m e cercas**. Ver [escopo vigente](MAPA_CAMPO_GRANDE_01.md). Sem engine/RL.

## Abrir e criar

No menu de aplicativos, abra **Soccer Fall - Mapas**. Com o primeiro mapa
presente, abre `assets/maps/src/galpao_01.map` (ver `docs/MAPA_GALPAO_01.md`).
Em uma instalacao inicial abre `integracao.map`, a pequena prova com piso,
parede, bloco e nascimento.

Para criar um mapa novo, a partir da raiz do repositorio:

```bash
python3 tools/maps/novo.py primeiro_mapa
~/.local/bin/trenchbroom assets/maps/src/primeiro_mapa.map
```

O comando recusa sobrescrever um arquivo existente. Tambem e possivel usar
**File > New**, escolher **Soccer Fall / Valve** e salvar em `assets/maps/src/`.
O modelo inicial inclui piso e nascimento. Para mudar o layout das vistas,
use as preferencias do proprio editor.

## Compilar e validar

No menu **Run > Compile**, escolha **Matter - Compilar GLB**. O perfil exporta
o documento atual, inclusive alteracoes ainda nao salvas, para uma copia em
`assets/maps/compiled/`, converte-a para GLB e executa a verificacao da engine.
Salve o `.map` original para preservar a fonte no git. Use nomes de arquivo
distintos para mapas distintos: a saida e identificada pelo nome sem extensao.

O mesmo fluxo por terminal, sobre a versao salva:

```bash
python3 tools/maps/compilar.py assets/maps/src/primeiro_mapa.map
./tools/maps/verificar_engine.sh assets/maps/compiled/primeiro_mapa.glb
```

Antes da entrega, confira tambem `python3 tools/maps/auditar.py caminho.map`.
Ele aponta pecas sem contato com a fundacao (primeiro brush), duplicatas e
candidatos de faces coplanares. Monte suportes para objetos suspensos e confira
as faces visiveis no editor: encontros internos tambem aparecem no relatorio.

O compilador usa somente a biblioteca padrao do Python 3. O GLB inclui malhas,
normais, UVs, texturas PNG e metadados fisicos, sem caminhos externos. Agrupa a
geometria por material. Uma falha nao substitui o GLB anterior.

O verificador usa o **GltfLoader, MaterialLibrary e Jolt reais**: carrega o
arquivo, verifica os triangulos/texturas e constroi colisores estaticos. Para
cada nascimento, procura piso nos 4 m abaixo e deixa uma esfera de prova cair
por 2 s a 120 Hz, verificando se continua apoiada. Isso nao prova que o espaco
esta livre para o jogador inteiro ou que um mapa esta aprovado visualmente.

O verificador e o target CMake `MatterMapVerify`. Prepare uma build Debug com
`CMAKE_BUILD_PARALLEL_LEVEL=1 ./tools/build.sh Debug --skip-tests`; o script
`verificar_engine.sh` usa `build-linux/MatterMapVerify`. Outra build pode ser
selecionada com `MATTERENGINE_MAP_BUILD_DIR`. Nao liga contra bibliotecas antigas
em outra build. A compilacao da aplicacao prepara os materiais e o GLB do galpao
automaticamente, sem alterar preferencias ou perfis instalados do TrenchBroom.

## Usar na engine

Abra **Laboratorio**, escolha o card **Dev Map** ou **Galpao 01**, e clique em
**Abrir mapa**. O primeiro mantem a plataforma de testes; o segundo carrega o
GLB autorado, colisao, materiais, nascimentos, luminarias e campo procedural.
Os previews dos cards sao capturas reais do Vulkan em `assets/maps/previews/`.
Os cards ficam centralizados, com uma coluna em janelas estreitas.
O `info_player_start` determina a posicao/direcao inicial; erros de carga nao
publicam uma cena parcialmente pronta. Trocar de mapa libera recursos e redes.
O nascimento consulta o piso perto do marcador, abaixo da cobertura: lancar
um raio de cima do edificio colocava o personagem sobre o telhado.

Depois dos desligamentos de 02/11, validar uma etapa por vez: compilacao -j1,
um processo de teste e smokes curtos a 30 FPS (`MATTERENGINE_MAX_FPS=30`). A
aplicacao dorme entre quadros quando ha limite ativo; por pedido posterior do
usuario, nao ha teto fixo de 60 FPS. `MATTERENGINE_MAX_FPS=0` desliga o limite
adicional (padrao); o V-sync continua seguindo a tela. A simulacao continua a
120 Hz. Nao executar suite completa junto de render ou compilacao.

## Contrato de autoria

| Item | Convencao |
|---|---|
| Formato | `.map` Valve 220, planos de brushes convexos |
| Escala | **32 unidades = 1 metro**; jogador de 1,80 m = 57,6 unidades |
| Eixos | X/Y no chao, Z para cima, iguais aos da engine |
| Grade | 16 unidades = 0,5 m; 8 = 0,25 m; 1 = 3,125 cm |
| Orientacao | `angle` em graus: 0 aponta +X, 90 aponta +Y |
| Texturas base | 128x128; escala UV 0,25 cobre 1 metro |
| Geometria | `worldspawn`, `func_detail`, grupos/camadas do TrenchBroom |
| Colisao | Malha estatica, material fisico por parte |

O GLB faz a rotacao inversa do importador: Z-up em unidades do editor vira
Y-up em metros no glTF, e volta corretamente ao entrar na Matter Engine.
Nao altere a escala manualmente no glTF ou na importacao.

### Materiais

`matter/concrete`, `matter/brick`, `matter/wood`, `matter/steel`,
`matter/grass` e `matter/rubber` aparecem no navegador de materiais.
Sao texturas procedurais pequenas, de base para construir o mapa, com o ID
fisico correspondente do catalogo da engine. Nao representam arte final.

Os PNGs sao gerados por `configurar.py` em `assets/maps/textures/matter/`.
O galpao acrescenta `indoor/*`: 24 superficies, normalmente de 512x512, geradas por
`tools/maps/galpao_materiais.py`. Variantes visuais compartilham IDs fisicos
do catalogo, incluindo `glass` nos vidros escuros. No gerador, escala 0,125
mantem a repeticao de 2 m dos materiais de 512px; placas retangulares usam UVs
ajustadas a uma unica face. Laterais nao devem repetir a sinalizacao.
UVs usam a dimensao real do PNG, igual ao editor. Emissao, normal e roughness sao consumidos pelo render. Albedo/emissao usam
sRGB; mapas de dados permanecem lineares. O gramado usa tile de 1024px a 1m
(escala 0,03125), normal/altura RGBA e parallax com reducao a distancia.

Para acrescentar um material, ele precisa existir no catalogo fisico e no
contrato do compilador; nao basta copiar uma imagem. Os fontes e scripts
entram no git; `textures/` e `compiled/` sao reconstruiveis e ficam ignorados.

### Entidades

| Entidade | Significado |
|---|---|
| `worldspawn` | Geometria estatica do cenario; um por mapa |
| `func_detail` | Agrupa mais geometria estatica |
| `info_player_start` | Nascimento, origem nos pes, com direcao; pelo menos um |
| `matter_field` | Centro, direcao, `length_m` e `width_m`; campo procedural no galpao |
| `light` | Luz point/spot, `_color`, candelas, alcance e orientacao/cones |

`matter_field` preserva centro, orientacao e dimensoes em metros. No Galpao 01,
o runtime gera demarcacoes de futsal e duas traves de 3x2m com redes fisicas;
a superficie de grama pertence ao MAP. O parser preserva campos maiores,
mas a implementacao de outras modalidades ainda deve ser acrescentada antes
de usar suas demarcacoes. `info_player_start` e consumido para o nascimento.

O compilador rejeita entidades desconhecidas, materiais desconhecidos,
brushes abertos/degenerados, planos redundantes e escalas UV zero. Aceita
brushes de 4 a 64 faces, inclusive rampas e formas giradas. Os grupos/camadas
do editor nao alteram a posicao da geometria exportada. Luzes pontuais e spot sao exportadas em `KHR_lights_punctual` e chegam ao
Vulkan com intensidade em candelas, decaimento pelo quadrado da distancia e
alcance. Spots usam `type=spot`, `angle`, `pitch` (-90=baixo), `inner_cone` e
`outer_cone` (aberturas totais em graus, interna menor que externa). Portas, triggers, agua e props
dinamicos nao tem entidade funcional neste contrato.

## Trabalho dos agentes

Claude e Codex podem editar o `.map` como texto ou usar `box`, `brush` e
`template` de `tools/maps/novo.py` para gerar partes com medidas exatas.
Depois, compile, valide e abra no TrenchBroom para conferir o resultado.
Nao use substituicoes globais de numeros para mover brushes; mova os pontos
dos planos, mantendo as coordenadas UV conforme o efeito desejado.

```bash
python3 -m unittest discover -s tools/maps -v
```

Os testes cobrem escala/eixos, orientacao, topologia, rampa, brush girado,
entidades, entradas invalidas, preservacao da saida e reinstalacao do perfil.

## Instalar novamente

Feche o TrenchBroom antes de reinstalar, para ele nao salvar preferencias
antigas sobre as novas:

```bash
python3 tools/maps/configurar.py
```

O instalador copia `GameConfig.cfg`, `SoccerFall.fgd`, mapa inicial e perfil
de compilacao para `~/.TrenchBroom/games/SoccerFall/`, configura o caminho
`assets/maps`, gera texturas e o atalho no menu de aplicativos. Preserva as
preferencias dos outros jogos e cria uma copia inicial de `Preferences.json`.
Nao edite os arquivos instalados para mudar o contrato: edite as fontes em
`tools/maps/trenchbroom/` e reinstale. Os outros perfis de compilacao sao
preservados. Nao ha necessidade de compilar BSP ou lightmaps de Quake: a saida
e GLB para o render e a fisica da Matter Engine.

## Limites desta entrega

O Galpao 01 ja tem integracao no Laboratorio. As luzes locais iluminam por
cone e distancia, mas ainda nao possuem mapas de sombra point/spot. Nao ha
CSG/PVS, interacao com portas/portao ou gerador de demarcacoes de futebol de
onze nesta entrega. O portao autorado permanece fechado e estatico.

Faces internas entre brushes permanecem no GLB; nao ha CSG, BSP, PVS ou LOD.
O teto de 64 planos por brush limita o custo de reconstrucao. Esses limites
devem ser reavaliados com o primeiro mapa real, com medidas de carga/render.
A conferencia visual e a medicao de custo continuam necessarias para cada mapa novo.

Fontes oficiais usadas para o perfil e a compilacao:
[manual do TrenchBroom](https://trenchbroom.github.io/manual/latest/),
[perfil Generic 2026.2](https://github.com/TrenchBroom/TrenchBroom/blob/v2026.2/app/TrenchBroom/resources/games/Generic/GameConfig.cfg),
[formato dos perfis de compilacao](https://github.com/TrenchBroom/TrenchBroom/blob/v2026.2/lib/TbMdlLib/src/CompilationConfig.cpp).
