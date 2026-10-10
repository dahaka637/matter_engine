#!/usr/bin/env bash
# Gera movimentos no Kimodo (container "demo" do docker-compose em
# ~/Projetos/kimodo) e analisa cada variacao (analisa.py).
#
# uso: gera.sh nome "prompt1. prompt2." "d1 d2" [args do kimodo]
#   ex.: gera.sh e3 "A person lying on their stomach on the ground quickly gets up and stands facing forward." "3.0" \
#          --num_samples 10 --seed 4 --constraints saidas/c3.json
# Saida: ~/Projetos/kimodo/saidas/<nome>/<nome>_NN.{npz,bvh}. O texto e
# codificado na CPU (Llama 3 8B): cada rodada leva ~4-10 min; gere varias
# variacoes por rodada.
KIMODO=${KIMODO:-/home/dahaka/Projetos/kimodo}
nome=$1; prompts=$2; dur=$3; shift 3
docker exec demo bash -lc "cd /workspace && rm -rf saidas/$nome && python -m kimodo.scripts.generate \"$prompts\" --duration \"$dur\" --bvh --bvh_standard_tpose --output saidas/$nome $* 2>&1 | grep -v '%|' | tail -3" >&2
cd "$(dirname "$0")" && python3 analisa.py -v "$KIMODO"/saidas/$nome/*.npz
