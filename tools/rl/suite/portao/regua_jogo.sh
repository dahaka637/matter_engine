#!/bin/bash
# regua_jogo.sh RUN TAG - as reguas do jogo para um treino (16/11, o plano de melhoria do RL): a rede exportada do RUN (exports/
# policy_latest.mrlpolicy) na fisica do jogo - a suavidade e as pancadas (MatterRlBrainProbe regua) e as colisoes (MatterAthleteProbe com
# o cerebro: parede, caixas, corpo por cima, encostao). A ultima linha, "JOGO ...", tem os numeros do portao. nice 15.
# (16/11 20h) Os binarios congelados do portao (portao_bin): o laboratorio in-game muda enquanto a fila roda e a regua tem de ser a
# mesma para todos os bracos; recongelar so junto com uma nova medida da referencia.
RUN=$1; T=$2; M=/home/dahaka/Projetos/matter_engine/build-profile/portao_bin; O=$HOME/.local/share/matter-engine/rl/ab16/jogo/$T; mkdir -p $O
MATTER_PROBE_RUN=$RUN nice -n 15 $M/MatterRlBrainProbe regua > $O/regua.txt 2>&1 &
for c in queda_parede caixa_debruca caixa_empurra caixa_baixa_lado boneco_por_cima boneco_encosta boneco_tombo; do
  XRL=1 XRL_RUN=$RUN XRL_REGISTRO=1 nice -n 15 $M/MatterAthleteProbe --cenario $c --ajuda 0 > $O/$c.txt 2>&1 &
done
wait
# (17h50) O corredor a 0,7 m, o meio-fio de 20 cm e a parede a frente, sem empurrao (o foco 5): depois do lote, um processo so.
MATTER_PROBE_RUN=$RUN nice -n 15 $M/MatterRlBrainProbe obst > $O/obst.txt 2>&1
grep -E "^suave|^SUAVE30|^torto|^TORTO|^REGUA" $O/regua.txt
ok=0; n=0; quedas=0
for c in queda_parede caixa_debruca caixa_empurra caixa_baixa_lado boneco_por_cima boneco_encosta boneco_tombo; do
  fim=$(grep -oE "fim (animado|volta|fisico|ponte|ativo)" $O/$c.txt | tail -1 | awk '{print $2}')
  bon=$(grep -oE "boneco: .*fim (animado|volta|fisico)" $O/$c.txt | grep -oE "fim [a-z]+" | awk '{print $2}')
  q=$(grep -c "padrao: modo do corpo FISICO\|ASSUME" $O/$c.txt)
  caiu=$(grep -E "andou .* recaidas" $O/$c.txt | grep -c "CAIU")
  n=$((n+1)); if [ "$fim" = "animado" ] || [ "$fim" = "volta" ]; then ok=$((ok+1)); fi
  quedas=$((quedas+caiu))
  echo "colisao $c: jogador fim $fim${bon:+, boneco fim $bon}, idas ao fisico $q, caiu $caiu"
done
echo "JOGO $(grep -oE "suave p95 cima [0-9.]+ maos [0-9.]+ cabeca [0-9.]+" $O/regua.txt) | $(grep -oE "pancadas de pe [0-9]+/[0-9]+ \\(.*\\) volta [-0-9.]+ s" $O/regua.txt) | colisoes de volta ao padrao $ok/$n, quedas $quedas | $(grep -oE "obstaculos: quedas [0-9]+" $O/obst.txt) | $(grep -oE "SUAVE30 cima [0-9.]+ maos [0-9.]+ cabeca [0-9.]+" $O/regua.txt) | $(grep -oE "TORTO .*" $O/regua.txt)"
