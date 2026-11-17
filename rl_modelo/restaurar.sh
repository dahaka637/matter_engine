#!/usr/bin/env bash
# Põe a rede r130240 onde o jogo e o treino procuram. Não sobrescreve arquivos que já existam.
set -euo pipefail
aqui="$(cd "$(dirname "$0")" && pwd)"
destino="${MATTER_RL_DATA:-$HOME/.local/share/matter-engine/rl}"
mkdir -p "$destino"
cp -r --update=none "$aqui/dados/runs" "$destino/"
echo "rede em $destino/runs/cerebro_unico (arquivos que já existiam foram mantidos)"
