#!/bin/bash
# folhas_levantar.sh TREINO PONTO TAG [BUILD] - (16/11 18h10) as folhas de 8 poses dos quatro levantares (a acao media, 1 mundo, 0-2,4 s) e o
# tempo ate a cabeca passar de 1,45 m, para conferir a imagem do portao. Saida: build-profile/tmp/ab16_folhas/TAG/. Um processo, nice 15.
set -e
T=$1; CK=$2; TAG=$3; B=${4:-build-base-n129}
W=/home/dahaka/Projetos/matter_engine_wip; P=/home/dahaka/Projetos/matter_engine/tools/rl/.venv/bin/python
O=$W/build-profile/tmp/ab16_folhas/$TAG; mkdir -p $O
export MATTER_TRACKING_NATIVE=$W/$B/python TMPDIR=$W/build-profile/tmp
cd $W/tools/rl
K="lev_levantar_costas lev_levantar_brucos lev_levantar_correndo_costas lev_levantar_correndo_brucos"
nice -n 15 $P -m inspecao.captura --treino $T --checkpoint $CK --exercicios $K --modos atual --mundos 1 --segundos 3.0 --saida $O --forcar > $O/captura.log 2>&1
for k in $K; do
  nice -n 15 $P -m inspecao.fantasma $O/${k}__atual.npz --folha 8 --janela 0 2.4 --saida $O/folha_${k}.png > $O/folha_${k}.log 2>&1
done
cd $O && $P - <<'EOF'
import json, numpy as np
from pathlib import Path
for key in ("lev_levantar_costas", "lev_levantar_brucos", "lev_levantar_correndo_costas", "lev_levantar_correndo_brucos"):
    npz = Path(f"{key}__atual.npz"); d = np.load(npz); dt = json.loads(npz.with_suffix(".json").read_text())["dt"]
    body, ep = d["corpo"].astype(float), d["episodio"]
    idx = np.where(ep[0] == 0)[0]; hz = body[0, idx, 5, 2]
    k = next((j for j in range(len(idx)) if hz[j] > 1.45), None)
    t = (idx[k] + 1) * dt if k is not None else float("nan")
    print(f"{key}: cabeca > 1,45 m em {t:.2f} s")
EOF
ls $O/*.png
