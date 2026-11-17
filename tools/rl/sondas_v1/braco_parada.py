"""(13/11 v1, a correcao 5) O braco estabanado na freada, nas capturas da suite: por parada (cenario x mundo), o maior desvio da altura das maos
(contra o ombro) do corpo contra a referencia entre a ordem de parar e +2 s. Uso: braco_parada.py PASTA_CAPTURAS [PASTA2 ...]"""
import sys, glob, numpy as np
sys.path.insert(0, '/home/dahaka/Projetos/matter_engine_wip/tools/rl')
from suite.medidas.base import carregar, IDX
for pasta in sys.argv[1:]:
    picos = []
    for f in sorted(glob.glob(pasta.rstrip('/') + '/parada_*__det.npz')):
        cap = carregar(f); ordem = cap.meta['cenario']['ordem']; t = cap.t()
        fb, fr = cap.corpo.astype(float), cap.ref.astype(float)
        jan = (t >= ordem) & (t <= ordem + 2.0)
        for w in range(cap.W):
            d = []
            for lado in ('Left', 'Right'):
                hb = fb[w, :, IDX[f'{lado}Hand'], 2] - fb[w, :, IDX[f'{lado}UpperArm'], 2]
                hr = fr[w, :, IDX[f'{lado}Hand'], 2] - fr[w, :, IDX[f'{lado}UpperArm'], 2]
                d.append(np.nanmax(np.abs(hb - hr)[jan]))
            picos.append(max(d))
    p = 100 * np.array(picos)
    print(f"{pasta.rstrip('/').split('/')[-1]:16s} paradas {len(p)}: desvio maximo da mao contra o clipe na freada - mediana {np.median(p):.0f} cm, "
          f"p90 {np.percentile(p, 90):.0f} cm, acima de 30 cm em {np.mean(p > 30):.0%}")
