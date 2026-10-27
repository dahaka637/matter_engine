#!/bin/bash
# Previa rapida (sem o laco do avesso): a etapa 07 de verdade, as margens, sem faixas e a exportacao para a pasta da
# previa; os pesos sao os do modelo instalado (mesmos vertices). previa.sh
cd /home/dahaka/Projetos/matter_engine
D=${TRABALHO:?defina TRABALHO, a pasta de trabalho fora do repositorio}
F=$D/final; P=$D/previa/player_model; B="env PATH=/usr/bin:/bin PYTHONHOME=/usr blender -b"; X=$F/previa; mkdir -p $X
rm -f "${X:?}/7.blend"   # (uma 07 que falha nao deixa a previa antiga passar por nova)
$B $F/2_pesos.blend --python tools/blender/etapas/07_proporcao_tronco.py -- $X/7.blend 2>&1 | grep -E "PROPORCAO|OMBRO|CABECA|CALCAO|Error|Traceback|line " | cut -c1-260
$B $X/7.blend --python tools/blender/etapas/03_textura_margem.py -- $X/7.blend 2>&1 | grep -E "Error|Traceback"
$B $X/7.blend --python tools/blender/sem_faixas.py -- $X/7.blend 2>&1 | grep -E "Error|Traceback"
rm -f "${P:?}/blender/jogador_textura_0.png"
$B $X/7.blend --python tools/blender/exportar_troca.py -- $X/novo.troca.json 2>&1 | grep -E "Error"
mv $X/jogador_textura_0.png $P/blender/jogador_textura_0.png 2>/dev/null || cp $(dirname $X/novo.troca.json)/jogador_textura_0.png $P/blender/ 2>/dev/null
python3 - "$X/novo.troca.json" "$P/blender/jogador.troca.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1])); inst = json.load(open("/home/dahaka/Projetos/matter_engine/assets/characters/player_model/blender/jogador.troca.json"))
assert len(d["positions"]) == len(inst["positions"])
# (O calcao fica com os pesos da etapa 07: a regra da parede de dentro da perna muda pesos, nao so a forma.)
shorts = d["parts"].index(next(p for p in d["parts"] if p["name"] == "calcao"))
keep = {v for tri, part in zip(d["triangles"], d["trianglePart"]) if part == shorts for v in tri}
d["weights"] = [d["weights"][v] if v in keep else w for v, w in enumerate(inst["weights"])]
json.dump(d, open(sys.argv[2], "w")); print(f"previa gravada (pesos do instalado; os do calcao da etapa 07, {len(keep)} vertices)")
PY
