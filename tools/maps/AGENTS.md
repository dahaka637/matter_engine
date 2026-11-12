# Mapas Soccer Fall — Claude e Codex

Leia `docs/MAPAS_TRENCHBROOM.md` na raiz antes de criar/editar mapas.
O `campo_grande_01` foi INTEGRALMENTE REJEITADO pelo usuario em 09/11/2026.
A primeira geometria foi removida. Reconstrucao do zero, por etapas:
**SOMENTE campo, arquibancada descoberta/bloco adjacente 2 andares 5x20m e cercas**.
Seguir as fotografias o mais proximo possivel: concreto aparente atras dos degraus,
bloco verde com empenas/vazios triangulares, mureta baixa e postes inclinados.
Sem cobertura metalica inventada, ruas, calcadas externas, casas, rio ou mobiliario.
Nao alterar engine/runtime/menu/builds/RL nem importar; Claude trabalha no RL.
Vegetacao vira de pack estilizado do usuario: nao modelar arvores/arbustos/mato.
Planta R01/imagem conceitual e a primeira autoria NAO SAO base aprovada.
Leia `docs/MAPA_CAMPO_GRANDE_01.md`. Testes tecnicos nao comprovam qualidade visual.
O primeiro mapa real e `galpao_01.map`; detalhes em `docs/MAPA_GALPAO_01.md`.
Sua referencia foi gerada por IA: o usuario autorizou corrigir e inventar pela
logica construtiva. Redes verdes foram substituidas por cerca galvanizada
na borda do gramado. Por pedido posterior do usuario, o portao junto aos bancos
agora esta **fechado**, com trinco e folgas; nao reabrir automaticamente.
E cenario estatico: abrir/fechar por interacao continua para a integracao futura.
Fontes em `assets/maps/src/*.map`, formato **Valve 220**, **32 unidades = 1 m**,
Z para cima. O `.map` e o original editavel; o GLB e gerado, nunca o contrario.

Use `python3 tools/maps/novo.py nome` para iniciar, `compilar.py` para exportar
e `./tools/maps/verificar_engine.sh assets/maps/compiled/nome.glb` para validar
com o GltfLoader e o Jolt reais. Os helpers `box`, `brush` e `template` de
`novo.py` permitem gerar a geometria em texto sem controlar o mouse.
Excecao vigente do campo grande: somente MAP, auditoria Python e previa Blender
direta do MAP (`previa_campo.py`), sem compilar GLB/importar na engine. A paleta
`beirario/*` tem descritores proprios de autoria e ainda nao foi incorporada ao
catalogo do compilador/runtime. Nao alterar esse contrato sem autorizacao futura.

O Laboratorio carrega `galpao_01` com spawn, malhas estaticas, materiais fisicos,
luzes point/spot, normal/roughness e emissao. `matter_field` gera demarcacoes e
as duas traves do galpao (futsal 3x2 m, tubos/linhas 8 cm), com redes fisicas.
O gramado continua um brush do MAP; relevo e visual, colisao plana. Campos de
outras modalidades ainda precisam de seu gerador de demarcacoes no runtime.
Nunca deduzir o material fisico pela cor: `physical_material` e obrigatorio;
`cookStaticTriangleMesh(mesh, materialId)` fixa esse material na malha Jolt.

Luzes spot: `type=spot`, `angle` em graus no plano, `pitch=-90` aponta para
baixo, `inner_cone`/`outer_cone` sao aberturas TOTAIS em graus. Intensidade em
candelas e alcance em metros. A engine converte para os semiangulos do glTF.
Verificar o resultado Vulkan: o Blender e apenas uma conferencia complementar.
Sombras locais de spot/point ainda nao sao renderizadas; nao prometa esse recurso.

Preserve a frente de RL do outro agente: nenhum treino, processo, tarefa ou
build `build-rl*` pertence a esta frente. O verificador agora e um target CMake (`MatterMapVerify`) da build Debug.
O script usa `build-linux` ou `MATTERENGINE_MAP_BUILD_DIR`, sem compilar escondido.
Depois dos desligamentos de 02/11: build -j1, testes em sequencia e previas
curtas com `MATTERENGINE_MAX_FPS=30`; conferir temperaturas entre etapas.
Nao sobrescreva mapas existentes ao criar variantes.

Antes de entregar, use `python3 tools/maps/auditar.py caminho.map`: nenhuma
peca deve ficar sem contato com a fundacao (primeiro brush) nem duplicada.
Confira os candidatos de faces coplanares no editor: faces internas podem
aparecer no relatorio, mas superficies visiveis nao devem disputar o mesmo plano.
Equipamentos suspensos precisam de suportes reais, e pecas no chao de apoio.
