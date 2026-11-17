#!/bin/bash
# lancar.sh UNIDADE RECEITA.json TREINO BUILD THREADS ATUALIZACOES [SOCKET] - um treino de RL do cerebro unico pelo systemd: nice 15, com o
# vigia de temperatura, THREADS de simulacao (ele: no maximo ~8 no total), sem dormir. SOCKET: o do controle (o braco escondido do
# laboratorio, que so ve o principal). (16/11: recriado no repositorio; os de 15/11 ficavam no scratchpad e foram apagados.)
set -e
U=$1; REC=$2; T=$3; B=$4; TH=$5; N=$6; SOCK=${7:-}
W=/home/dahaka/Projetos/matter_engine_wip
P=/home/dahaka/Projetos/matter_engine/tools/rl/.venv/bin/python
mapfile -t ARGS < <($P -c "
import json
a=json.load(open('$REC'))['argumentos']
i=a.index('--threads'); a[i+1]='$TH'
print('\n'.join(a))")
EXTRA=()
if [ -n "$SOCK" ]; then EXTRA=(--setenv=MATTER_RL_SOCKET=$SOCK); fi
systemd-run --user --unit=$U -p Nice=15 -p Requires=vigia-temperatura.service -p After=vigia-temperatura.service "${EXTRA[@]}" \
  --working-directory=$W/tools/rl \
  --setenv=MATTER_TRACKING_NATIVE=$W/$B/python --setenv=TMPDIR=$W/build-profile/tmp --setenv=PYTHONUNBUFFERED=1 \
  -p StandardOutput=append:$HOME/.local/share/matter-engine/rl/runs/$T/treino.out -p StandardError=append:$HOME/.local/share/matter-engine/rl/runs/$T/treino.out \
  systemd-inhibit --what=sleep:idle --why="treino RL $T" \
  $P levantar_amp.py "${ARGS[@]}" --treino $T --atualizacoes $N --semente 3
