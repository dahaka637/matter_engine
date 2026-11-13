#!/bin/bash
# lancar.sh UNIDADE RECEITA.json TREINO BUILD THREADS ATUALIZACOES [args extras...]
set -e
U=$1; REC=$2; T=$3; B=$4; TH=$5; N=$6; shift 6
W=/home/dahaka/Projetos/matter_engine_wip
P=/home/dahaka/Projetos/matter_engine/tools/rl/.venv/bin/python
mapfile -t ARGS < <($P -c "
import json,sys
a=json.load(open('$REC'))['argumentos']
i=a.index('--threads'); a[i+1]='$TH'
print('\n'.join(a))")
systemd-run --user --unit=$U -p Requires=vigia-temperatura.service -p After=vigia-temperatura.service \
  --working-directory=$W/tools/rl \
  --setenv=MATTER_TRACKING_NATIVE=$W/$B/python --setenv=TMPDIR=$W/build-profile/tmp --setenv=PYTHONUNBUFFERED=1 \
  -p StandardOutput=append:$HOME/.local/share/matter-engine/rl/runs/$T/treino.out -p StandardError=append:$HOME/.local/share/matter-engine/rl/runs/$T/treino.out \
  systemd-inhibit --what=sleep:idle --why="treino RL $T" \
  $P levantar_amp.py "${ARGS[@]}" --treino $T --atualizacoes $N --semente 3 "$@"
