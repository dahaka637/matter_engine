#!/bin/bash
# Uso: tools/probe-grid.sh <saida> [filtro-regex] [args extras da sonda...]
# Roda os cenarios da sonda (MatterAthleteProbe --manejo 1) que casam com o
# filtro, em paralelo, e escreve uma linha por cenario com os numeros
# principais (picos, salto, toques, folga pe-bola na forca, atraso, perna
# louca, voo, perna por fase). Resumo: tools/probe-summary.py <saida>.
M="$(cd "$(dirname "$0")/.." && pwd)"
out=$1; filtro=${2:-^mov_}; shift 2 2>/dev/null; extra="$*"
"$M/build-profile/MatterAthleteProbe" --lista | grep -E "$filtro" | xargs -P 12 -I{} bash -c '
c={}
"'"$M"'/build-profile/MatterAthleteProbe" --manejo 1 '"$extra"' --cenario $c 2>/dev/null | python3 -c "
import sys,re
c=sys.argv[1]; txt=sys.stdin.read()
def g(p,d=\"?\"):
    m=re.search(p,txt); return m.group(1) if m else d
pico=g(r\"\|\s+[0-9.]+\s+/\s+([0-9.]+) m/s2\"); tronco=g(r\"\|\s+([0-9.]+)\s+/\s+[0-9.]+ m/s2\"); salto=g(r\"\| ([0-9.]+) m \|\")
v5=re.search(r\"v5: pedidos (\d+) toques (\d+) \| folga pe-bola na forca med ([0-9.]+) max ([0-9.]+) m \| atraso med ([0-9.]+) max ([0-9.]+) s \| perna louca (\d+) joelho virando (\d+) giro max ([0-9.]+) rad/s \| voo max ([0-9.]+) s\",txt)
v5s=\" \".join(v5.groups()) if v5 else \"- - - - - - - - - -\"
gar=re.search(r\"garantia (\d+) folga med ([0-9.]+) \| perdidos (\d+)\",txt)
v5s+=\" | garantia \"+\" \".join(gar.groups()) if gar else \"\"
cruz=g(r\"cruzou (\d+)\"); sobg=g(r\"no gesto ([0-9.]+)\"); caiu=\"CAIU\" if \"CAIU\" in txt else \"\"
print(f\"{c:26s} tronco {tronco} pernas {pico} salto {salto} cruzou {cruz} sobg {sobg} {caiu} | v5 {v5s}\")
" $c' > "$out.tmp"
sort "$out.tmp" > "$out"; rm -f "$out.tmp"
