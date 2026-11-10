# Campo grande 01 — reconstrução por etapas

Atualizado em **09/11/2026**. A primeira autoria foi **integralmente rejeitada**
pelo usuário: proporções, pilares, orientações e acabamentos ruins. A geometria,
os backups e as prévias dessa versão foram removidos. Não a tratar como aprovada;
testes de topologia/contato não demonstram qualidade visual.

Escopo vigente: **somente campo, arquibancada, bloco adjacente de dois andares
5 × 20 m e cercas**, o mais próximo possível das fotografias de Abelardo Luz.
Sem ruas, calçadas externas, casas, rio, mobiliário ou refletores nesta etapa.
Não alterar/importar na engine, menu, builds ou RL do Claude. Galpão01 preservado.
Vegetação virá do pack estilizado do usuário em outra etapa; não modelar plantas.

## Referência e leitura

- Arquibancada **descoberta** de concreto, degraus cinza, acessos amarelos;
  lajes e vigas inclinadas sustentadas por pilares aparentes, inclusive atrás.
  Sem a cobertura metálica inventada da versão rejeitada.
- Bloco verde-claro com base verde escura, dois pavimentos, vãos horizontais,
  grades, varanda recuada e grandes empenas triangulares com vazios laterais.
  Cobertura cinza de fibrocimento, como nas fotos, sem telhas cerâmicas vermelhas.
- **5 × 20 m informado pelo usuário**. Nesta reconstrução:20m paralelos à lateral
  do campo e5m de profundidade. Alturas e aberturas são estimadas das imagens;
  não apresentar essas estimativas como medidas reais.
- Alambrado de tela losangular em mureta baixa, postes de concreto claros com
  ponta inclinada e encontros em meia-esquadria. Portão junto ao bloco.

As [dez referências originais](</home/dahaka/Área de trabalho/IA_Brain/08_mapa/referencias/campo_grande_01/README.md>)
continuam preservadas. As novas fotos da conversa orientam esta revisão.
A planta vetorial R01 e a imagem conceitual anteriores também foram rejeitadas;
não são a base da reconstrução.

## Medidas e organização

| Elemento | Autoria |
|---|---|
| Campo de jogo |105 × 68m, marcador `matter_field`|
| Gramado plano contínuo |115 × 76m|
| Folga até a mureta |5m nos fundos e4m nas laterais|
| Arquibancada |45 × 8m, estimativa de projeto|
| Patamares |10; piso0,75m, desnível0,34m|
| Escadas entre setores |2 lances de1,20m, meios-degraus de0,17m|
| Bloco de apoio |20m ao longo do campo ×5m de profundidade|
| Coordenadas |X comprimento, Y largura, Z altura;32u/m|

A camada de bases contém apenas o gramado, fundação e acesso imediato da
arquibancada. Não representa o terreno/entorno final. Linhas, traves e redes
dos gols continuam reservadas à futura geração procedural; não duplicadas no MAP.

105×68m é escolha normalizada apoiada na
[recomendação FIFA](https://inside.fifa.com/innovation/stadium-guidelines/technical-guidelines/stadiums-guidelines/pitch-dimensions-and-surrounding-areas),
não medição do lugar nem dimensão única obrigatória. A pesquisa histórica e os
intervalos oficiais permanecem no [plano do vault](</home/dahaka/Área de trabalho/IA_Brain/08_mapa/plano_campo_grande_01.md>).

## Fonte e conferência fora da engine

```bash
./assets/maps/autoria/campo_grande_01/abrir.sh
python3 tools/maps/campo_grande_01.py --substituir
python3 -m unittest discover -s tools/maps -p test_campo_grande.py -v
python3 tools/maps/auditar.py assets/maps/src/campo_grande_01.map
blender -b -t 1 --python tools/maps/previa_campo.py -- --vista campo --saida assets/maps/autoria/campo_grande_01/previews/campo.png
```

Fonte: `assets/maps/src/campo_grande_01.map`, Valve220, sete camadas separando
bases, degraus, estrutura, alvenaria, esquadrias/cobertura, pilares e tela.
Regerar exige `--substituir` e preserva backup **da nova reconstrução** em
`assets/maps/autoria/campo_grande_01/versoes/`; avaliar edições manuais antes.

Texturas próprias em `tools/maps/campo_materiais.py`, descritores em
`assets/maps/materials/beirario/superficies.json`. UVs em escala métrica.
A paleta `beirario/*` ainda **não está integrada** ao compilador/runtime;
não executar o perfil de compilação da engine nesta etapa.

Prévias Blender leem o MAP diretamente: vistas `aerea`, `campo`, `fachada`,
`posterior`, `degraus` e `alambrado`. Sem cenário decorativo acrescentado no render.
Uma thread CPU, carga sequencial e temperatura monitorada. Não são imagens in-game.

Testes técnicos verificam a fonte salva, topologia, arquivos de textura,
footprint real5×20 do bloco e faixa livre do gramado. Comparação visual com as
fotos e aprovação do usuário são avaliações distintas; o mapa não tem aprovação
visual automática. Controle da reconstrução: `matter_engine-niz` (reaberto).
