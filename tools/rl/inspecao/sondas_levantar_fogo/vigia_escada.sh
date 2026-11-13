#!/bin/bash
# Vigia da escada do fogo do levantar (12/11 07h15): queda do treino, degraus (sobe/desce), erros, e a cada 10 min o resumo + a temperatura.
D=/tmp/claude-1000/-home-dahaka--rea-de-trabalho-IA-Brain/8fe86884-eddc-41e6-ada9-9fcf6d9b523b/scratchpad
R=~/.local/share/matter-engine/rl/runs/idle_alerta
P=/home/dahaka/Projetos/matter_engine/tools/rl/.venv/bin/python
U=rl-levantar-unreal.service
seen=$(wc -l < $R/treino.out); n=0
while true; do
  if ! systemctl --user is-active -q $U; then
    sleep 60
    if ! systemctl --user is-active -q $U; then echo "TREINO PAROU $(date +%H:%M): $(tail -3 $R/treino.out | tr '\n' ' ' | cut -c1-300)"; sleep 300; fi
  fi
  now=$(wc -l < $R/treino.out)
  if [ "$now" -gt "$seen" ]; then
    tail -n $((now - seen)) $R/treino.out | grep -a -E "fogo do levantar: lev_|Traceback|Error|nan|Killed" | sed 's/^/DEGRAU\/ERRO: /'
    seen=$now
  fi
  n=$((n + 1))
  if [ $((n % 20)) -eq 0 ]; then
    t=$(sensors 2>/dev/null | awk '/Tctl/{print $2; exit}')
    echo "$(date +%H:%M) $t | $($P $D/resumo_fogo.py 2>&1 | tr '\n' ' ' | cut -c1-900)"
  fi
  sleep 30
done
