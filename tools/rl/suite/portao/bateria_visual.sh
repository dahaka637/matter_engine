#!/bin/bash
# bateria_visual.sh SAIDA NOME:RUN ... - (17/11 11h35, ele: "o boneco esta torto ainda") O corpo do jogo em todos os movimentos do
# MatterAthleteProbe (as teclas, o modo do jogo), de tras e de lado, por rede (RUN vazio: a locomocao default, os clipes), e a folha
# por cenario e vista com uma linha por rede. Nice 15, uma coisa por vez.
set -u
G=/home/dahaka/Projetos/matter_engine
P=$G/tools/rl/.venv/bin/python
O=$1; shift
mkdir -p $O
declare -A DUR=([parado]=3.0 [anda]=3.5 [trota]=3.5 [sprint]=4.0 [lados]=4.0 [diagonal]=3.0 [costas]=3.0 [para]=3.5 [gira]=3.0 [corte]=4.0)
NETS=()
for spec in "$@"; do
  name=${spec%%:*}; run=${spec#*:}; NETS+=($name)
  for c in "${!DUR[@]}"; do
    if [ -n "$run" ]; then
      XRL=1 XRL_FULL_ATE=100 XRL_RUN=$run timeout 400 nice -n 15 $G/build-profile/MatterAthleteProbe --cenario $c --ajuda 0 --dump $O/${name}_$c.dump --cada 2 > $O/${name}_$c.txt 2>&1
    else
      timeout 400 nice -n 15 $G/build-profile/MatterAthleteProbe --cenario $c --ajuda 0 --dump $O/${name}_$c.dump --cada 2 > $O/${name}_$c.txt 2>&1
    fi
    end=$(python3 -c "print(${DUR[$c]} - 0.1)")
    for y in 180 90; do
      timeout 300 nice -n 15 $G/build-profile/MatterModelInspector quadros --dump $O/${name}_$c.dump --cenario $c --de 1.0 --ate $end \
        --quadros 8 --colunas 8 --yaw $y --pitch 6 --seguir 1 --distancia 3.4 --largura 240 --altura 360 --saida $O/${name}_${c}_$y.png > /dev/null 2>&1
    done
  done
done
$P - "$O" "${NETS[@]}" <<'PY'
import sys
from pathlib import Path
from PIL import Image, ImageDraw
out, nets = Path(sys.argv[1]), sys.argv[2:]
for c in ("parado", "anda", "trota", "sprint", "lados", "diagonal", "costas", "para", "gira", "corte"):
    for y in ("180", "90"):
        imgs = [(n, Image.open(out / f"{n}_{c}_{y}.png").convert("RGB")) for n in nets if (out / f"{n}_{c}_{y}.png").exists()]
        if not imgs:
            continue
        w = max(i.width for _, i in imgs); h = sum(i.height + 24 for _, i in imgs)
        sheet = Image.new("RGB", (w, h), "white"); d = ImageDraw.Draw(sheet); top = 0
        for n, i in imgs:
            d.text((8, top + 5), f"{n} - {c} - {'de tras' if y == '180' else 'de lado'}", fill="black"); sheet.paste(i, (0, top + 24)); top += i.height + 24
        sheet.save(out / f"folha_{c}_{y}.png")
print("ok")
PY
