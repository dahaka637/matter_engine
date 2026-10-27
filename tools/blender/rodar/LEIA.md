# Como rodar o jogador do zero (23/10/2026)

    export TRABALHO=/tmp/jogador            # uma pasta fora do repositorio
    mkdir -p $TRABALHO/final $TRABALHO/esq85 $TRABALHO/fonte_blender $TRABALHO/previa
    cp -r assets/characters/player_model assets/characters/football_player $TRABALHO/fonte_blender/
    cp -r assets/characters/player_model assets/characters/football_player $TRABALHO/previa/
    tools/blender/rodar/pipeline.sh       # 00 -> 02 (~1 min)
    tools/blender/rodar/previa.sh         # a etapa 07 com os pesos do instalado (~35 s) para julgar a forma
    tools/blender/rodar/de07.sh           # 07 -> margens -> sem faixas -> assado -> desvirar (~25 min) -> 04

Depois: copiar $TRABALHO/final/4_final.blend para assets/characters/player_model/blender/jogador.blend,
apagar jogador_textura_0.png e exportar a troca (tools/blender/exportar_troca.py) no lugar.

## A camisa e o calcao em quadrados (25/10/2026)

    tools/blender/rodar/moldes.sh [densidade]   # sobre o jogador instalado (~30-45 min com o desvirar)

Refaz as duas pecas pelos moldes da roupa (`etapas/retopo_moldes.py`, etapa 01 com `moldes ... final`) a partir
da peca densa aprovada (`historico/jogador_12_calcao_v.blend`; outra com FONTE=...): a forma aprovada fica (mapa e pesos pelo ponto mais perto da peca instalada), as barras seguem os aneis da pele (a faixa
vira quads) e a folga empurra para fora so onde a malha grossa cruza a pele. Antes do desvirar, `alisar_canto_gola.py` alisa os pesos junto do
canto da gola (com o braco erguido, a tira escondida ali dobrava) e `fracao_ombro.py` reescreve os pesos do ombro em fracoes do giro do braco
(com a rampa da meia-junta do GameRig, a manga acompanha o braco erguido); `axila_ombro.py` passa a parte da meia-junta do lado
de baixo do ombro (a axila e as dobras) e do alto da manga para a junta da axila do GameRig, que segue o braco inteiro na vertical
(sem ela o contorno embaixo do braco erguido inchava para fora e o lado da cabeca afinava ate o ombro), e o peso de peito do fundo
da axila para a junta do volume, que anda um pouco para fora com o braco erguido. Depois: copiar $TRABALHO/moldes/4_final.blend
para assets/characters/player_model/blender/jogador.blend, apagar jogador_textura_0.png e exportar a troca no lugar.
No pipeline do zero, `REMALHA=moldes pipeline.sh` usa os moldes na peca grossa original (o Phong dela afunda o
canto da gola; o caminho sobre o instalado e o que vale).

## O template da superficie (26/10/2026)

O sistema de aparencia novo (docs/SISTEMA_DE_APARENCIA.md) precisa do molde de cada face da roupa. A etapa 01
grava o atributo de face `molde` (e a tabela no texto `matter_moldes`); o `exportar_troca.py` leva para a troca
(`moldes`, `triangleMolde`) e o `moldes.sh` termina com o gerador:

    python3 tools/blender/superficie.py <troca> <troca>      # o template dentro da troca, pela referencia em uso
    python3 tools/blender/superficie.py <troca> <troca> --nova-referencia --novo-layout   # outra construcao: versoes novas
    python3 tools/blender/superficie.py <troca> <troca> --novo-layout   # so um layout novo do UV0
    python3 tools/blender/superficie_teste.py                # as provas do gerador (determinismo, canonicidade, versoes, layout, ida e volta)
    python3 tools/blender/superficie_previa.py <troca> <png> [carta]   # o xadrez de 5 cm no jogador, para o olho
    build-profile/MatterModelInspector exportar-jogo --destino <pasta>   # e copiar a pele, a superficie, o manifesto e o atlas

Os arquivos ficam em `assets/characters/player_model/superficie/`: `definicoes.json` (as regras e as versoes em uso),
`referencia_pN.json` (a referencia canonica: o desenho, as cartas, as regioes e a posicao 3D por poligono da construcao)
e `layout_sN.json` (o UV0). O modo padrao so aplica a referencia: o mesmo corpo ou outro com a mesma construcao da o
mesmo template; outra topologia para o gerador, que pede `--nova-referencia --novo-layout`. Os arquivos novos so sao
gravados depois de tudo conferido, e as versoes anteriores ficam. Ao exportar o jogo, o perfil fisico instalado fica
(a reexportacao deriva alguns micrometros): copie so a pele, a superficie, o manifesto e o atlas.
Uma malha feita antes dos moldes com nome recebe os rotulos por `tools/blender/rotular_moldes.py` (a etapa 01
refeita com as mesmas entradas e a correspondencia exata das faces, sem mexer em nada da malha aprovada).

