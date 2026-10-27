#!/bin/bash
# O jogador do zero ate a etapa 02 (pipeline.sh) e o resto (de07.sh). ALVO_CAMISA/ALVO_CALCAO: as faces de cada
# peca no QuadriFlow (padrao 2400/1600). REMALHA=moldes: as pecas pelos moldes da roupa (retopo_moldes.py),
# DENSIDADE multiplica as divisoes das costuras.
cd /home/dahaka/Projetos/matter_engine
D=${TRABALHO:?defina TRABALHO, a pasta de trabalho fora do repositorio}
F=$D/final; B="env PATH=/usr/bin:/bin PYTHONHOME=/usr blender -b"
H=$(pwd)/assets/characters/player_model/blender/historico; G="/home/dahaka/Área de trabalho/modelagem/MODELOS_PRONTOS/cabeca_2.0.glb"
step(){ echo "== $(date +%H:%M:%S) $1"; }
[ -f $D/esq85/jogador.poses.json ] || ./build-profile/MatterModelInspector exportar-blender --modelo assets/characters/player_model/model.json --saida $D/esq85 --poses 6 --amplitude 2>&1 | tail -1
step 00; $B $H/jogador_00_paridade.blend --python tools/blender/etapas/00_esqueleto.py -- $D/esq85/jogador.troca.json $F/0_esqueleto.blend 2>&1 | grep -E "ESQUELETO|Error|Traceback"
step 01c; $B $F/0_esqueleto.blend --python tools/blender/etapas/01_peca_quadriflow.py -- $F/1_camisa.blend ${ALVO_CAMISA:-2400} camisa 10 ${REMALHA:-quadriflow} ${DENSIDADE:-1} 2>&1 | grep -E "aviso|CAMISA|Error|Traceback"
step 01s; $B $F/1_camisa.blend --python tools/blender/etapas/01_peca_quadriflow.py -- $F/1_ambos.blend ${ALVO_CALCAO:-1600} calcao 10 ${REMALHA:-quadriflow} ${DENSIDADE:-1} 2>&1 | grep -E "aviso|CALCAO|Error|Traceback"
step 05; $B $F/1_ambos.blend --python tools/blender/etapas/05_cabeca.py -- "$G" $F/5_cabeca.blend 2>&1 | grep -E "CABECA|Error|Traceback"
step rosto; $B $F/5_cabeca.blend --python tools/blender/pintar_rosto.py -- $F/rosto.png 2>&1 | grep -E "ROSTO|Error|Traceback" | cut -c1-80
step 06; $B $F/5_cabeca.blend --python tools/blender/etapas/06_textura_pele.py -- --mapa $F/rosto.png --borda 1 $F/6_pele.blend 2>&1 | grep -E "PELE|Error|Traceback"
step 03; $B $F/6_pele.blend --python tools/blender/etapas/03_textura_margem.py -- $F/3_margem.blend 2>&1 | grep -E "Error|Traceback"
step 02; $B $F/3_margem.blend --python tools/blender/etapas/02_pesos_pecas.py -- $H/jogador_01_camisa_quads.blend $F/2_pesos.blend 2>&1 | grep -E "PESOS|Error|Traceback"
$B $F/2_pesos.blend --python tools/blender/exportar_troca.py -- $F/pre.troca.json 2>&1 | grep -E "Error"
