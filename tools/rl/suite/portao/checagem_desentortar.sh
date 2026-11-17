#!/bin/bash
# checagem_desentortar.sh [BRACO ...] - (17/11, o plano de desentortar, vault 04_locomocao/plano_desentortar_17_11.md) A checagem do A/B:
# para cada braco (padrao: desentortar_a desentortar_b) guarda o ponto, exporta num run a parte e faz:
#   - as faixas do jogo (o MatterAthleteProbe com o cerebro, XRL_RUN; o corpo inteiro) no trote e na corrida, de tras (yaw 180) e de lado
#     (yaw 90), lado a lado com a locomocao default (os clipes) e a r130240 - o PORTAO e a imagem;
#   - as medidas de guarda: a distancia ao clipe (DISTLOCO) e a regua do jogo (o torto, a suavidade, as velocidades).
# A referencia (a default e a r130240) e feita uma vez em ~/.local/share/matter-engine/rl/desentortar/ref. Saida: .../desentortar/HHMM/.
# Um processo por vez, nice 15 (os bracos treinam com 8 threads).
set -u
# (17/11 05h40, ele: o protocolo de respeito a atividade dele - a checagem so com a CPU ociosa: SCHED_IDLE, os filhos herdam.)
# (So com OCIOSO=1 - fora do protocolo, a checagem roda normal com nice 15.)
if [ "${OCIOSO:-0}" = "1" ] && [ -z "${CHRT_IDLE:-}" ]; then CHRT_IDLE=1 exec chrt -i 0 bash "$0" "$@"; fi
RUNS=$HOME/.local/share/matter-engine/rl/runs
W=/home/dahaka/Projetos/matter_engine_wip
G=/home/dahaka/Projetos/matter_engine
P=$G/tools/rl/.venv/bin/python
OUT=$HOME/.local/share/matter-engine/rl/desentortar
REF=$OUT/ref
T=$(date +%H%M)
O=$OUT/$T
mkdir -p $O $REF
# (05h35: a n132 - a n131 mais a velocidade do clipe por junta (--velocidade-rotacao, o braco C); com os ajustes desligados, a mesma.)
export MATTER_TRACKING_NATIVE=$W/build-base-n134/python TMPDIR=$W/build-profile/tmp
# (As medidas no concreto, o chao do laboratorio e o da referencia r130240 - os bracos treinam metade em grama.)
export MATTER_CHAO_GRAMA=0
ARMS=("$@")
[ ${#ARMS[@]} -eq 0 ] && ARMS=(desentortar_c desentortar_h)

strips() {  # strips NOME DIR_DO_RUN(vazio: a default) -> $DEST/NOME_{trota,sprint}_{180,90}.png
  local name=$1 run=$2 c y
  for c in trota sprint; do
    if [ -n "$run" ]; then
      XRL=1 XRL_FULL_ATE=100 XRL_RUN=$run timeout 400 nice -n 15 $G/build-profile/MatterAthleteProbe --cenario $c --ajuda 0 \
        --dump $DEST/${name}_$c.dump --cada 2 > $DEST/${name}_$c.txt 2>&1
    else
      timeout 400 nice -n 15 $G/build-profile/MatterAthleteProbe --cenario $c --ajuda 0 --dump $DEST/${name}_$c.dump --cada 2 \
        > $DEST/${name}_$c.txt 2>&1
    fi
    for y in 180 90; do
      timeout 300 nice -n 15 $G/build-profile/MatterModelInspector quadros --dump $DEST/${name}_$c.dump --cenario $c --de 2.9 --ate 3.45 \
        --quadros 6 --colunas 6 --yaw $y --pitch 6 --seguir 1 --distancia 3.4 --largura 300 --altura 420 \
        --saida $DEST/${name}_${c}_$y.png > /dev/null 2>&1
    done
  done
}

if [ ! -f $REF/r130240_sprint_90.png ]; then
  DEST=$REF
  strips padrao ""
  strips r130240 $RUNS/cerebro_unico
  (cd $W/tools/rl && nice -n 15 $P $W/tools/rl/suite/portao/distancia_clipe_loco.py cerebro_unico $RUNS/cerebro_unico/final_15h_r130240.pt r130240 \
    > $REF/r130240_dist.txt 2>&1)
  MATTER_PROBE_RUN=$RUNS/cerebro_unico nice -n 15 $G/build-profile/MatterRlBrainProbe regua > $REF/r130240_regua.txt 2>&1
fi
# (--so-ref: so a referencia, enquanto os bracos comecam.)
if [ "${ARMS[0]}" = "--so-ref" ]; then rmdir $O 2>/dev/null; ls $REF/*.png; exit 0; fi

DEST=$O
for arm in "${ARMS[@]}"; do
  A=$RUNS/$arm
  M=$RUNS/${arm}_ver_$T
  mkdir -p $M/exports
  cp -p $A/ultimo.pt $A/ver_$T.pt
  cp -p $A/ultimo.pt $M/ultimo.pt
  cp -p $A/run.json $A/receita.json $M/ 2>/dev/null
  (cd $W/tools/rl && nice -n 15 $P $W/tools/rl/suite/portao/exportar.py $arm $M/ultimo.pt $M/exports/policy_latest.mrlpolicy > $O/${arm}_exportar.txt 2>&1)
  strips $arm $M
  (cd $W/tools/rl && nice -n 15 $P $W/tools/rl/suite/portao/distancia_clipe_loco.py $arm $M/ultimo.pt ${arm}_$T > $O/${arm}_dist.txt 2>&1)
  MATTER_PROBE_RUN=$M nice -n 15 $G/build-profile/MatterRlBrainProbe regua > $O/${arm}_regua.txt 2>&1
  # (09h) A prova de fogo (32 mundos por andadura, 96 tentativas): o equilibrio com menos ruido que as 30 pancadas da regua.
  # (09h50: 4 sementes do comeco - uma so varia +-4 de 96; a r130240 da 70, 67, 76, 71: media 71.)
  for sp in 0 1 2 3; do
    (cd $W/tools/rl && SEMENTE_PROVA=$sp nice -n 15 timeout 900 $P inspecao/sondas_locomocao_fogo/sonda_prova.py $M/ultimo.pt $A/receita.json >> $O/${arm}_prova.txt 2>&1)
  done
done

# A comparacao: por andadura e vista, as linhas default / r130240 / bracos (a mesma janela 2,9-3,45 s).
$P - "$REF" "$O" "${ARMS[@]}" <<'EOF'
import sys
from pathlib import Path
from PIL import Image, ImageDraw
ref, out, arms = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3:]
for c in ("trota", "sprint"):
    for y in ("180", "90"):
        rows = [("default (clipe)", ref / f"padrao_{c}_{y}.png"), ("r130240 (lab)", ref / f"r130240_{c}_{y}.png")]
        rows += [(a, out / f"{a}_{c}_{y}.png") for a in arms]
        imgs = [(label, Image.open(p).convert("RGB")) for label, p in rows if p.exists()]
        if not imgs:
            continue
        w = max(i.width for _, i in imgs); h = sum(i.height + 28 for _, i in imgs)
        sheet = Image.new("RGB", (w, h), "white"); draw = ImageDraw.Draw(sheet); top = 0
        for label, img in imgs:
            draw.text((8, top + 6), label, fill="black"); sheet.paste(img, (0, top + 28)); top += img.height + 28
        sheet.save(out / f"comparacao_{c}_{y}.png")
        print(out / f"comparacao_{c}_{y}.png")
EOF
for arm in "${ARMS[@]}"; do
  echo "== $arm ($(grep -oE '[0-9]{6}$' $O/${arm}_exportar.txt | tail -1))"
  grep -a "^atualizacao=" $RUNS/$arm/treino.out | tail -1 | tr ' ' '\n' | grep -E "^(atualizacao|tarefa|rastreio_nota|clip|kl|simetria|assimetria|linhas_espelho)=" | tr '\n' ' '; echo
  grep -E "DISTLOCO|distancia ao clipe" $O/${arm}_dist.txt
  grep -E "^torto|^TORTO|^SUAVE30|^REGUA" $O/${arm}_regua.txt
  for c in trota sprint; do echo "$c: $(grep -oE 'andou [0-9.]+ m, recaidas [0-9]+' $O/${arm}_$c.txt | head -1)"; done
  echo "prova de fogo (media de 4 sementes): $(grep -oE 'nao caiu [0-9.]+' $O/${arm}_prova.txt | awk '{s+=$3} END {printf "%.1f/96 (a r130240: 71,0)", s*8/4}') parado $(grep -oE 'parado_[a-z]+: nao caiu [0-9.]+' $O/${arm}_prova.txt | awk '{s+=$4} END {printf "%.1f/32 (r130240 19,3)", s*8/4}')"
done
