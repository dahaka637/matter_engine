#!/usr/bin/env bash
# Varreduras da locomocao no harness (build-profile), em 8 processos.
#
#   tools/locomotion_sweep.sh gait <rotulo> [VAR=valor ...]
#   tools/locomotion_sweep.sh push <rotulo> [VAR=valor ...]
#
# gait: sweepgait (8 direcoes x trote/sprint); resumo por estilo.
# push: sweep (216 empurroes parado); resumo de quedas.
# As linhas cruas ficam em $SWEEP_OUT (padrao /tmp/matter_sweeps).
# Exemplo: tools/locomotion_sweep.sh gait e5_1ms XVMC=31 XGAITSPEED=1.0
set -Eeuo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
kind="${1:?gait|push}"
label="${2:?rotulo}"
shift 2
out="${SWEEP_OUT:-/tmp/matter_sweeps}"
mkdir -p "$out"
cd "$root/build-profile"

case "$kind" in
gait)
    for k in 0 1 2 3 4 5 6 7; do
        (env "$@" XSWEEPPART=$k/8 MATTERENGINE_TEST_FILTER=sweepgait ./MatterAdaptiveTests 2>/dev/null \
            | grep -E "^(GAIT|PASSADA|TRANSICAO|BALANCO|APOIO) " > "$out/ga_${label}_$k.txt") &
    done
    wait
    cat "$out"/ga_"${label}"_*.txt | sort -k2,2 -k3,3n > "$out/ga_${label}.txt"
    rm -f "$out"/ga_"${label}"_*.txt
    grep "^GAIT " "$out/ga_${label}.txt" | awk -v l="$label" '{k=$2; c[k]++; f[k]+=($5>0); r[k]+=$7;
        if ($17>=0) {g[k]+=$17/$7; ng[k]++; lat[k]+=($19<0?-$19:$19)}; y[k]+=$13; ph[k]=ph[k] ($5>0?$21:"-");
        if ($23>=0) {iv[k]+=$23; ni[k]++}; fr[k]+=$25; lt[k]+=$27}
        END{for (k in c) printf "%s %s: %d/%d quedas (fases %s), regime %.0f%% do pedido (%.1f m/s), desvio lateral medio %.2f m/s, rumo pior %.0f graus%s | soltos carregados %d, carga por prazo %d\n",
            l, k, f[k], c[k], ph[k], ng[k]?100*g[k]/ng[k]:-1, r[k]/c[k], ng[k]?lat[k]/ng[k]:-1, y[k]/c[k],
            ni[k]?sprintf(", inverteu em %.2f s (%d de %d)", iv[k]/ni[k], ni[k], c[k]):"", fr[k], lt[k]}' | sort
    grep "^GAIT " "$out/ga_${label}.txt" | awk -v l="$label" 'NF>=41{c++; t+=$39; if ($39>tw) tw=$39; a+=$41; if ($41>aw) aw=$41}
        END{if (c>0) printf "%s inclinacao max media %.1f graus (pior %.1f) | afastamento corpo x referencia medio %.3f m (pior %.3f)\n", l, t/c, tw, a/c, aw}'
    if grep -q "^TRANSICAO " "$out/ga_${label}.txt"; then
        grep "^TRANSICAO " "$out/ga_${label}.txt" | awk -v l="$label" '{n+=$5; g+=$11*$5; c+=$14*$5; a+=$17*$5; s+=$23*$5; e+=$30*$5}
            END{if (n>0) printf "%s transicoes: %d | ganho antes do toque %+.3f | colisao %+.3f | aceitacao %+.3f | saldo -0,1 s do toque -> saida %+.3f | 0,1 s depois da saida %+.3f m/s (medias)\n", l, n, g/n, c/n, a/n, s/n, e/n}'
    fi
    if grep -q "^BALANCO " "$out/ga_${label}.txt"; then
        grep "^BALANCO " "$out/ga_${label}.txt" | awk -v l="$label" '{n+=$5; rv+=$11*$5; rc+=$14*$5; if ($16>w) w=$16; r20+=$19; a+=$26*$5; if ($28>aw) aw=$28; tr+=$32*$5; le+=$37*$5; fl+=$41; fs+=$43; rj+=$50*$5; lb+=$55; lr+=$57; lf+=$64; sv+=$68; sa+=$71; sd+=$73; sc+=$75; c++}
            END{if (n>0) printf "%s balancos: %d | revisao depois de 35%% %.3f m | recuo %.3f m (pior %.3f, >=20 cm: %d) | pico de aceleracao do alvo %.0f m/s2 (pior %.0f) | rastreio %.3f m | erro no pouso %.3f m (medias) | voo %.1f, apoio ficticio %.1f ticks por corrida | soltura fora da ancora %.3f m | revisoes limitadas %d, recusadas %d | pe da frente pela agenda %d\n%s estrito (por corrida): voo %.1f | apoio ficticio no apoio %.1f, na descarga %.1f, na carga %.1f ticks\n", l, n, rv/n, rc/n, w, r20, a/n, aw, tr/n, le/n, fl/c, fs/c, rj/n, lb, lr, lf, l, sv/c, sa/c, sd/c, sc/c}'
    fi
    if grep -q "^APOIO " "$out/ga_${label}.txt"; then
        grep "^APOIO " "$out/ga_${label}.txt" | awk -v l="$label" '{c++; fa+=$6; fc+=$8; fs+=$10; pl+=$12; vt+=$14; ve+=$16;
            for (i=21; i<=33; i+=2) r[i]+=$i}
            END{printf "%s apoio (por corrida): ficticio no apoio %.1f, aceitando %.1f, saindo %.1f ticks | perdido %.1f | voo inesperado %.1f ticks em %.1f eventos | solturas (total): carga %d, parcial %d, geometria %d, agenda %d, no ar %d, recuperacao %d, falha %d\n",
                l, fa/c, fc/c, fs/c, pl/c, vt/c, ve/c, r[21], r[23], r[25], r[27], r[29], r[31], r[33]}'
    fi
    ;;
push)
    for k in 0 1 2 3 4 5 6 7; do
        (env "$@" XSWEEPPART=$k/8 MATTERENGINE_TEST_FILTER=sweep ./MatterAdaptiveTests 2>/dev/null \
            | grep "^SWEEP " > "$out/sw_${label}_$k.txt") &
    done
    wait
    cat "$out"/sw_"${label}"_*.txt | sort -k2,2 -k3,3n -k4,4n > "$out/sw_${label}.txt"
    rm -f "$out"/sw_"${label}"_*.txt
    awk -v l="$label" '{c++; if ($6>0) {f++; k[$2]++}; t+=($8>90?90:$8); h+=$12; r+=$14; lv+=$16; d+=$18}
        END{printf "%s: %d casos, %d quedas (peito %d, cintura %d, impulso %d), tilt medio %.1f, segurado fora da base %.2f s | recuperacao %d passos, %d levantados, %d arrastados\n",
            l, c, f, k["peito"], k["cintura"], k["impulso"], t/c, h, r, lv, d}' "$out/sw_${label}.txt"
    ;;
*)
    echo "tipo desconhecido: $kind (gait|push)" >&2
    exit 1
    ;;
esac
