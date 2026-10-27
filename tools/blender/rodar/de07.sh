#!/bin/bash
# Da etapa 07 ate o fim: 07, margens, sem faixas, exportacao, assado, desvirar, 04 e as medidas.
cd /home/dahaka/Projetos/matter_engine
D=${TRABALHO:?defina TRABALHO, a pasta de trabalho fora do repositorio}
F=$D/final; M=$D/fonte_blender/player_model; B="env PATH=/usr/bin:/bin PYTHONHOME=/usr blender -b"
step(){ echo "== $(date +%H:%M:%S) $1"; }
step 07; $B $F/2_pesos.blend --python tools/blender/etapas/07_proporcao_tronco.py -- $F/7_proporcao.blend 2>&1 | grep -E "PROPORCAO|OMBRO|CABECA|CALCAO|Error|Traceback|line "
step 03b; $B $F/7_proporcao.blend --python tools/blender/etapas/03_textura_margem.py -- $F/7_proporcao.blend 2>&1 | grep -E "Error|Traceback"
step faixas; $B $F/7_proporcao.blend --python tools/blender/sem_faixas.py -- $F/7_proporcao.blend 2>&1 | grep -E "SEM|Error|Traceback"
step exporta; $B $F/7_proporcao.blend --python tools/blender/exportar_troca.py -- $F/antes.troca.json 2>&1 | grep -E "Error"
[ "$1" = "07" ] && exit 0
step assado; python3 tools/blender/assar_delta_mush.py $F/antes.troca.json $D/esq85/jogador.poses.json $F/assado.troca.json --atual 0.005 --vizinhos 0.02 --espelho --sem-coxa camisa --braco-perto camisa --calcao-coxa --desvirar 3 2>&1 | grep -E "erro|desvirar:|braco perto|calcao coxa|Trace|Error"
step textura; rm -f "${M:?}/blender/jogador_textura_0.png"; $B $F/7_proporcao.blend --python tools/blender/exportar_troca.py -- $M/blender/lixo.troca.json 2>&1 | grep -E "Error"; rm -f "${M:?}/blender/lixo.troca.json"
step desvirar; python3 tools/blender/desvirar_avesso.py $F/assado.troca.json $F/desvirado.troca.json --modelo $M --passadas 6 --braco-perto camisa --calcao-coxa 2>&1 | grep -E "AVESSO|avesso no comeco|braco perto|calcao coxa|Trace|Error"
step 04; $B $F/7_proporcao.blend --python tools/blender/etapas/04_pesos_assados.py -- $F/desvirado.troca.json $F/4_final.blend 2>&1 | grep -E "PESOS|Error|Traceback"
step fim; cp $F/desvirado.troca.json $M/blender/jogador.troca.json
timeout 300 ./build-profile/MatterModelInspector analisar --modelo $M/model.json 2>&1 | grep -E "Resumo|ERRO|AVISO"
./build-profile/MatterModelInspector avesso --modelo $M/model.json 2>&1 | grep -E "Poses do jogo"
