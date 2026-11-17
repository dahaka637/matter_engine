#!/bin/bash
# bateria_conducao.sh: roda os cenarios conduz_auto* da sonda e imprime o resumo de cada um.
cd /home/dahaka/Projetos/matter_engine/build-profile
run() { out=$(nice -n 10 ./MatterAthleteProbe --cenario "$1" 2>/dev/null | grep "^AUTO" | sed 's/^AUTO *//'); printf "%-26s %s\n" "$1" "$out"; }
export -f run
printf "%s\n" conduz_auto conduz_auto_corre conduz_auto_curva conduz_auto_vira90 conduz_auto_inverte conduz_auto_zigue \
  conduz_auto_para conduz_auto_lado conduz_auto_tras conduz_auto_sprint_vira conduz_auto_seco90 conduz_auto_seco180 conduz_auto_sprint_seco90 conduz_auto_sprint_para conduz_auto_gira_parado conduz_auto_livre1 conduz_auto_livre2 conduz_auto_livre3 conduz_auto_livre4 conduz_auto_livre5 conduz_auto_livre6 \
  | xargs -P 6 -I{} bash -c 'run {}' | sort
