#!/bin/bash
# O jogador do Blender para o motor e as checagens (22/10/2026).
#   tools/blender/sincronizar.sh [pasta_da_regua]
# Grava assets/characters/player_model/blender/jogador.troca.json do .blend e
# roda: o avesso nas poses dos clipes do jogo (tem que ser 0) e a regua do
# quadril e do ombro (tools/analysis/deformacao_bateria.py, contra a base).
# (Nesta maquina o Blender acha o Python do uv no PATH e quebra: o PATH e o
# PYTHONHOME do sistema.)
set -e
cd "$(dirname "$0")/../.."
B=assets/characters/player_model/blender
SAIDA=${1:-/tmp/matter_regua_blender}
PATH=/usr/bin:/bin PYTHONHOME=/usr blender -b "$B/jogador.blend" --python tools/blender/exportar_troca.py -- "$B/jogador.troca.json" 2>&1 | grep -E "GRAVADO|Error|grupos sem"
./build-profile/MatterModelInspector avesso 2>&1 | grep -E "Poses do jogo"
python3 tools/analysis/deformacao_bateria.py "$SAIDA" 2>&1 | grep -E "^total|PIOROU|^  (quadril|ombro)/"
