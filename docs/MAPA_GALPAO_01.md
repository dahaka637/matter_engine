# Galpao 01 — primeiro mapa fechado

Primeira versao de 31/10/2026, integrada ao Laboratorio em 02/11/2026, a partir da imagem enviada
pelo usuario. Ele esclareceu que a referencia foi gerada pelo ChatGPT e contem
erros: serve para a linguagem visual, com liberdade para inventar e corrigir
pela logica construtiva e pelo uso do espaco. As dimensoes sao escolhas de projeto.
Galpao industrial coberto, com estrutura aparente, protecao verde nas paredes
e cerca metalica independente no limite do gramado.

## Dimensoes e elementos

- Reserva de jogo: **40 x 20 m**, eixo longitudinal X, centro `(0,0,0)`.
- Gramado sintetico texturizado: **45 x 23,2 m**, com folga para a area de jogo.
- Casca: aproximadamente **49,6 x 29,2 m**; cobertura em duas aguas, cumeeira
  a aproximadamente **9,9 m**. O volume central fica livre ate 7,2 m.
- Chapa corrugada, porticos a cada 6 m, pilares claros, tesouras, tercas,
  chapas de ligacao, bases e parafusos.
- Protecao inferior verde nas paredes e drenos junto ao gramado.
- Cerca galvanizada em torno do gramado, com portao junto aos bancos.
- Abrigo lateral verde com duas alturas de bancos de madeira.
- Dois acessos cenograficos com portas duplas, barras e placas de saida;
  portao de servico com persiana e faixas de alerta.
- Calhas, descidas, conduites, bandejas de cabos, quadro eletrico, grelhas de
  ventilacao, placas de setor, caixas de seguranca e lixeiras.
- 24 luminarias superiores e 3 luzes nos bancos. Luzes em glTF
  `KHR_lights_punctual`; difusores em `KHR_materials_emissive_strength`.
- Tres nascimentos e um marcador `matter_field` de 40x20.

O MAP contem a arquitetura e a superficie do gramado. Na engine,
`matter_field` gera linhas, areas, circulo central, penaltis de 6/10 m,
escanteios e zonas de substituicao, alem de **duas traves procedurais de
3 x 2 m**, tubos de 8 cm e redes fisicas. As medidas seguem a
[Lei 1 do futsal FIFA 2025/26](https://digitalhub.fifa.com/m/3eb3e1b60c5f1e1c/original/FUTSAL-Laws-of-the-Game-2025-2026_FR.pdf).
O sintetico foi mantido conforme pedido do usuario; isso nao constitui
homologacao do piso como quadra oficial de competicao.
Portas e equipamentos continuam estaticos; o portao da cerca fica fechado.

## Cerca e acesso ao campo (31/10)

- Fechamento de **44,8 x 23 m**, a 10 cm da borda do gramado, antes das
  canaletas. Preserva os 40x20 de jogo e folgas de 2,4 m nos fundos e 1,5 m
  nas laterais para o futuro campo com gols.
- Malha soldada de aco galvanizado, quadrados de aproximadamente **15 cm**,
  fios de 6 mm modelados. Altura da cerca **5,4 m**; postes com tampas ate
  5,48 m, bases aparafusadas e travessas. Paineis com ate 2,5 m entre postes.
- Portao na lateral dos bancos, entre X=-14,9 e -13,5 m, Y=11,5 m. Vao
  nominal de 1,4 m, sem soleira. A malha continua acima da travessa a 2,3 m.
- Folha com dobradicas, diagonal e fecho, **fechada por pedido posterior do
  usuario**, com 1 cm de folga de cada lado e trinco no batente. Os testes
  verificam a pose fechada e a circulacao externa desimpedida. A antiga rede
  baixa dos bancos foi removida.
- O portao e geometria estatica fechada nesta versao e bloqueia a passagem:
  ainda nao abre/fecha por interacao ou dobradica fisica. O estado aberto da
  revisao anterior foi substituido por ordem explicita do usuario.

## Polimento de materiais e objetos (31/10)

- Placas e equipamentos recebem arte **somente na face frontal**. Laterais,
  verso e chanfros possuem material de carcaca; UVs encaixam o PNG inteiro,
  incluindo as proporcoes retangulares. Placa de saida com caixa clara,
  parafusos e seta para o acesso, sem repetir letras nas laterais.
- Caixa vermelha refeita como armario de hidrante: painel identificado,
  dobradicas, puxador e fechadura. Portas com juntas estreitas, vedacoes,
  vidro escuro, molduras, barras com suportes, dobradicas e chapas inferiores.
  Quadro eletrico com frente identificada e ferragens em geometria.
  Os armarios vermelhos foram afastados dos pilares; a verificacao do layout
  impede a interpenetracao que a conferencia em close revelou.
- Madeira dos bancos dividida em comprimentos plausiveis, com emendas nos
  apoios, bordas chanfradas, topo de madeira, arruelas e parafusos. Sapatas
  dos pes e bases do abrigo apoiadas, com arremates no telhado.
- Calhas com canal aberto e tampas de extremidade; descidas redondas com
  luvas e abracadeiras. Lixeiras ocas com borda e alcas. Luminarias com
  tampas, presilhas e carcaca dos difusores.
- Paleta industrial com metais neutros, madeira menos alaranjada e variacao
  discreta. Texturas repetiveis de 512px, mantendo a repeticao de 2 m com
  escala 0,125; placas usam resolucoes/proporcoes proprias. Gramado uniforme
  preservado. O vidro escuro e opaco; transparencia nao foi adicionada.

## Fontes e uso

Fonte editavel: `assets/maps/src/galpao_01.map`, Valve 220, nove camadas no
TrenchBroom. Abra pelo atalho **Soccer Fall - Mapas** ou pelo caminho acima.
Fios e ferragens usam fracoes de unidade: nao aplique snap global na geometria.

Construido com `tools/maps/galpao_01.py`. **Depois de editar no TrenchBroom, o
`.map` e a fonte de autoria.** O gerador recusa sobrescreve-lo por padrao;
`--substituir` descarta as edicoes manuais e so deve ser usado intencionalmente.

As texturas sao preparadas por `galpao_materiais.py` (23 materiais usados no
mapa atual), com corrugacoes,
granulado, veios, costuras e sinalizacao. As placas usam Roboto do repositorio.
Gerar texturas usa o Pillow instalado; compilar mapas usa apenas Python padrao.
O albedo original da grama, criado por imagegen, esta preservado em
`assets/maps/materials/grass/albedo_source.png`; prompt e preparo documentados
no README ao lado. Normal/altura e roughness derivam dessa fonte. O material
usa UV de 1 m, normal mapping e parallax de ate 12 passos, reduzido a distancia.
O relevo e visual: o chao de colisao continua plano.

```bash
python3 tools/maps/galpao_materiais.py
python3 tools/maps/auditar.py assets/maps/src/galpao_01.map
python3 tools/maps/compilar.py assets/maps/src/galpao_01.map
./tools/maps/verificar_engine.sh assets/maps/compiled/galpao_01.glb --fechado
```

`--fechado` acrescenta 396 raios nos seis sentidos, em uma grade do volume
da quadra. O teste do mapa tambem exige os 40x20 livres entre piso e 7,2 m.

## Conferencia visual

As imagens em `inspecao/mapas/galpao_01/` sao renders do **GLB exportado**,
com Blender como visualizador, com a geometria, texturas e luzes do mapa.
Nao sao imagens conceituais geradas por IA. Para reproduzir:

```bash
blender -b -t 1 --python tools/maps/previa_blender.py -- \
  --mapa assets/maps/compiled/galpao_01.glb \
  --saida inspecao/mapas/galpao_01/polimento_geral.png --vista geral --largura 1600
```

Vistas: `geral`, `bancos`, `entrada`, `estrutura`, `encaixes`, `portao`, `sinalizacao`.
As previas da revisao atual usam `polimento_*.png`;
as demais imagens preservam as revisoes anteriores. Cycles em CPU, uma thread,
preservando o RL e evitando o CUDA 13 local, que nao compila para a GTX 1070 Ti.
O Blender e so um visualizador; a autoria continua no TrenchBroom.

## Integracao e limites

Abra **Laboratorio > Galpao 01 > Abrir mapa**. O card Dev Map continua ao lado,
com a plataforma anterior; os cards ficam centralizados. O nascimento local
consulta o gramado abaixo da cobertura, preservando a direcao do marcador.
A aplicacao consome o GLB, imagens sRGB e mapas de
dados lineares, materiais fisicos, tres nascimentos e o campo procedural.
A build prepara o GLB automaticamente; o MAP editavel continua sendo a fonte.

As 24 luminarias principais sao spots de 900 cd, voltados para baixo, com
cones totais de 90/120 graus. As tres dos bancos usam 60 cd, 70/110 graus,
inclinadas 75 graus para baixo e na direcao do passeio. Intensidade cai com
o quadrado da distancia e suavemente no limite do alcance. Nao ha Sol/Lua
iluminando o interior. Difusores e placas emissivas sao consumidos no Vulkan.
A indireta atual e uma aproximacao ambiental; nao e iluminacao global calculada.
O preenchimento interior usa irradiancia 4,5 e visibilidade minima 0,45:
areas fora dos cones conservam leitura de forma, com contraste mais escuro.
**Sombras locais de point/spot ainda nao existem no renderer.**

A colisao conserva `physical_material`: concreto, grama, aco, madeira,
borracha e vidro. O Jolt recebe o material na propria malha cozida; a bola
usa amortecimento e resistencia de rolamento da superficie, alem do atrito.
Teste no GLB real: rolamento **7,394 m**, retorno **0,776 m** na queda de 2 m;
solado deslizou **0,263 m** na grama e **0,286 m** no concreto no mesmo ensaio.
O personagem ficou apoiado no gramado. Coeficientes do catalogo foram
preservados; nao foram reajustados por aparencia nem alterados treinos de RL.
O ar interior nao recebe o vento do cenario externo; o arrasto da bola em ar
parado continua ativo.

Capturas e logs de integracao ficam em `inspecao/mapas/galpao_01/integracao/`.
Conferencias Vulkan usam 30 FPS, uma por vez, depois do relato de desligamento.
A engine passou a dormir entre quadros quando ha limite ativo. Por pedido do
usuario, o teto adicional de 60 FPS foi retirado: `MATTERENGINE_MAX_FPS=0` e o
padrao, mantendo V-sync conforme a tela; a simulacao segue 120 Hz.
As temperaturas e os smokes curtos nao identificam a causa do desligamento.

O lag de colisao foi reproduzido na Debug sem otimizacao, que o launcher
escolhia por ser mais recente. `tools/run.sh` agora prefere RelWithDebInfo;
atualize essa build para jogar e use a Debug diretamente para diagnostico.
No mesmo GLB, com corpo fisico real (180ticks/caso): cerca P95 **8,843 ->
0,321ms**, trave **8,030 -> 0,293ms** (Debug -> otimizada). Bola nos postes:
maximo **0,539 -> 0,023ms**. Sao custos do ensaio de fisica/consultas, sem
render nem controle completo de locomocao; nao prometem FPS fixo em toda carga.
Rolamento, quique e atrito do solado permaneceram equivalentes nas duas builds.

Faces internas permanecem, agrupadas por material. O GLB autorado tem 83.688
triangulos, 5.418 brushes e aproximadamente 15,37 MiB com os mapas PBR.
A geometria procedural do campo e das traves e acrescentada em runtime.
Portas/portao nao possuem interacao. O gerador de linhas desta integracao e
futsal; o campo grande permanece apenas em pre-producao documental.

## Revisao anterior dos encaixes (31/10, antes da troca das redes)

O usuario apontou pecas afastadas da parede e superficies coincidentes no
editor. Os paineis verdes agora encostam na casca; tubos ficam adiante deles,
com bracadeiras e suportes ate a parede. Quadros, placas, caixas, portas e
ventilacoes possuem fundo apoiado. O passeio foi elevado ate a cota do gramado,
com bases de pilares, pernas dos bancos, postes e lixeiras apoiados no piso.
As ripas centrais dos assentos ganharam travessas. Redes, bandejas e luminarias
receberam fixacoes; fios excedentes foram cortados. As almas dos perfis I
deixaram de invadir as mesas, e os nos da trelica receberam cortes de encontro;
ferragens deixam de terminar na mesma face do pilar. As redes e seus trilhos
param nos acessos, com postes nas bordas. Caixas foram instaladas acima dos
paineis, atras da rede; esta nao atravessa o quadro eletrico.

`auditar.py` verifica brushes convexos por SAT, incluindo arestas inclinadas,
e procura um caminho de contatos de cada peca ate o primeiro brush (fundacao).
Tolerancia: 0,1 mm. Tambem detecta volumes duplicados e lista sobreposicoes
de faces voltadas ao mesmo lado, acima de 2 cm2. Faces opostas que apenas se
tocam sao encaixes. **Candidatos coplanares exigem conferencia visual:** faces
internas no piso, nas colunas e nos cortes entre pecas continuam no arquivo;
o auditor nao determina oclusao nem substitui a revisao no editor.

Resultado: nenhum grupo sem contato com a fundacao, nenhum brush duplicado.
Teste de regressao impede canos/instalacoes coplanares com as protecoes verdes.
Na revisao atual, doze testes Python e o verificador GltfLoader/Jolt
passaram; os testes verificam chanfros, UVs/rótulos frontais e a pose fechada
do portao solicitada pelo usuario, preservando a circulacao externa.
Nenhuma peca solta ou duplicada e nenhum candidato de face coplanar envolvendo
a nova cerca. Candidatos internos das demais camadas permanecem. O backup anterior,
relatorios e a vista de detalhe ficam em `inspecao/mapas/galpao_01/` (ignorados
no git). Nunca regenerar sobre futuras edicoes manuais sem preserva-las.
