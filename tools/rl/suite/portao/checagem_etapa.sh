#!/bin/bash
# checagem_etapa.sh TAG - a bateria do portao A/B do cerebro unico (16/11, recriada no repositorio; a de 15/11 ficava no scratchpad): a
# largada, o cruzeiro, os quatro levantares (32 limpos: de pe, tempo, distancia ao clipe e a saida), as pernas, o parado, o pos-levantar
# (deriva e passos) e o fogo depois de levantar (/48). As reguas rodam em paralelo (nice 15). Env: TREINO (o run), PONTO_FONTE (o .pt).
T=$1; W=/home/dahaka/Projetos/matter_engine_wip; G=$W/tools/rl/suite/portao; P=/home/dahaka/Projetos/matter_engine/tools/rl/.venv/bin/python
TR=${TREINO:-cerebro_unico}; CK=$PONTO_FONTE; B=${BUILD:-build-base-n129}
O=$HOME/.local/share/matter-engine/rl/ab16/checagens/$T; mkdir -p $O
cd $W/tools/rl; export MATTER_TRACKING_NATIVE=$W/$B/python TMPDIR=$W/build-profile/tmp
U=$($P -c "import torch; print(torch.load('$CK',map_location='cpu',weights_only=False)['counters']['atualizacao'])" 2>/dev/null)
nice -n 15 $P $G/regua_arrancada.py $TR $CK lev_corrida_0,lev_trote_0,lev_corrida_4 > $O/arrancada.txt 2>&1 &
nice -n 15 $P $G/velocidade_reta.py $TR $CK lev_corrida_0,lev_corrida_1,lev_corrida_7,lev_corrida_4,lev_trote_0 > $O/cruzeiro.txt 2>&1 &
SEMENTES=61,62,63,64,65,66,67,68 nice -n 15 $P $G/levantares.py $TR $CK e$T > $O/levantares.txt 2>&1 &
nice -n 15 $P $G/alinhamento_pernas.py $TR $CK > $O/pernas.txt 2>&1 &
nice -n 15 $P $G/deriva_parado.py $TR $CK > $O/parado.txt 2>&1 &
nice -n 15 $P $G/deriva_levantar.py $TR $CK > $O/deriva_levantar.txt 2>&1 &
DEPOIS=8 CHAVES=lev_levantar_costas,lev_levantar_brucos nice -n 15 $P $G/pos_levantar.py $TR $CK 10 61,62 > $O/fogo.txt 2>&1 &
nice -n 15 $P $G/distancia_clipe_loco.py $TR $CK e$T > $O/dist_loco.txt 2>&1 &
wait
echo "== ponto r$U $T ($(date +%H:%M))"
echo "== arrancada"; grep -E "pedido|largadas" $O/arrancada.txt
echo "== cruzeiro"; grep "cruzeiro" $O/cruzeiro.txt
echo "== levantares (32 limpos)"; grep "de pe" $O/levantares.txt
echo "== pernas"; grep -E "^lev_.* \| esq:" $O/pernas.txt
echo "== parado"; grep -E "^lev_parado" $O/parado.txt
echo "== deriva do levantar"; grep -E "^lev_levantar" $O/deriva_levantar.txt
echo "== fogo depois de levantar"; grep -E "^lev_" $O/fogo.txt
echo "== distancia ao clipe na locomocao"; grep -E "distancia ao clipe|DISTLOCO" $O/dist_loco.txt
echo "== fim $(date +%H:%M)"
