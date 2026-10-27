#!/bin/bash
# A camisa e o calcao do jogador instalado refeitos pelos moldes da roupa (25/10/2026): quadrados em aneis, a
# forma aprovada intacta (mapa e pesos pelo ponto mais perto da peca instalada), as barras seguindo os aneis da
# pele; depois as margens, o desvirar (o avesso do motor nos pesos, ~25-40 min) e a etapa 04.
# uso: moldes.sh [densidade=1] (TRABALHO: a pasta de trabalho, com a copia do player_model e do football_player em
# $TRABALHO/fonte_blender, como no LEIA.md)
cd /home/dahaka/Projetos/matter_engine
D=${TRABALHO:?defina TRABALHO, a pasta de trabalho fora do repositorio}
F=$D/moldes; M=$D/fonte_blender/player_model; B="env PATH=/usr/bin:/bin PYTHONHOME=/usr blender -b"
# (A fonte e a peca densa aprovada, a de 24/10: depois de instalado, o jogador.blend ja e o grosso.)
I=${FONTE:-assets/characters/player_model/blender/historico/jogador_12_calcao_v.blend}
mkdir -p $F
step(){ echo "== $(date +%H:%M:%S) $1"; }
step camisa; $B $I --python tools/blender/etapas/01_peca_quadriflow.py -- $F/1_camisa.blend 0 camisa 10 moldes ${1:-1} final 2>&1 | grep -E "aviso|MOLDES|moldes:|folga|Error|Trace"
step calcao; $B $F/1_camisa.blend --python tools/blender/etapas/01_peca_quadriflow.py -- $F/1_ambos.blend 0 calcao 10 moldes ${1:-1} final 2>&1 | grep -E "aviso|MOLDES|folga|Error|Trace"
step margens; $B $F/1_ambos.blend --python tools/blender/etapas/03_textura_margem.py -- $F/3_margem.blend 2>&1 | grep -E "Error|Traceback"
step exporta; rm -f "${M:?}/blender/jogador_textura_0.png"; $B $F/3_margem.blend --python tools/blender/exportar_troca.py -- $M/blender/jogador.troca.json 2>&1 | grep -E "Error"
cp $M/blender/jogador.troca.json $F/moldes.troca.json
# (O canto da gola com o braco erguido: os pesos de perto dele alisados, antes do desvirar.)
python3 tools/blender/alisar_canto_gola.py $F/moldes.troca.json $F/moldes.troca.json
# (Os pesos do ombro em fracoes do giro do braco: com a rampa da meia-junta, a manga acompanha o braco erguido.)
python3 tools/blender/fracao_ombro.py $F/moldes.troca.json $F/moldes.troca.json
# (O lado de baixo do ombro na junta da axila: o braco erguido sem o contorno inchado embaixo dele.)
python3 tools/blender/axila_ombro.py $F/moldes.troca.json $F/moldes.troca.json
timeout 300 ./build-profile/MatterModelInspector analisar --modelo $M/model.json 2>&1 | grep -E "Resumo|ERRO|AVISO"
step desvirar; python3 tools/blender/desvirar_avesso.py $F/moldes.troca.json $F/desvirado.troca.json --modelo $M --passadas 6 --braco-perto camisa --calcao-coxa 2>&1 | grep -E "AVESSO|avesso no comeco|Trace|Error"
step 04; $B $F/3_margem.blend --python tools/blender/etapas/04_pesos_assados.py -- $F/desvirado.troca.json $F/4_final.blend 2>&1 | grep -E "PESOS|Error|Traceback"
step fim; cp $F/desvirado.troca.json $M/blender/jogador.troca.json
# (O template da superficie do sistema de aparencia, pela referencia canonica em uso: com outra
# topologia o gerador para e pede uma referencia nova, --nova-referencia --novo-layout.)
step superficie; python3 tools/blender/superficie.py $M/blender/jogador.troca.json $M/blender/jogador.troca.json 2>&1 | grep -E "SUPERFICIE|superficie:|Trace"
timeout 300 ./build-profile/MatterModelInspector analisar --modelo $M/model.json 2>&1 | grep -E "Resumo|ERRO|AVISO"
./build-profile/MatterModelInspector avesso --modelo $M/model.json 2>&1 | grep -E "Poses do jogo"
