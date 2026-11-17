"""(15/11 16h50, ele: 'o parado sempre distribui o peso em cima da perna esquerda') O centro de massa do parado contra o meio dos pes (frente,
esquerda; cm), corpo e clipe do idle, de 1 a 5 s, 8 mundos. Uso: peso_parado.py TREINO PONTO"""
import sys, numpy as np
from pathlib import Path
sys.path.insert(0, "/home/dahaka/Projetos/matter_engine_wip/tools/rl")
from inspecao import comum, captura
from inspecao import geometria as G
amb = comum.Ambiente(sys.argv[1], sys.argv[2])
D = Path("/home/dahaka/Projetos/matter_engine_wip/build-profile/tmp/rolar_r72889/distancia/peso_" + Path(sys.argv[2]).stem)
cap = G.carregar(captura.capturar(amb, "lev_parado", "atual", 8, 5.0, D, semente=61, log=lambda *a: None))
def off(frames):
    j = G.articulacoes(frames, cap.geo); f = G.referencial(frames)
    d = j["centro_massa"] - 0.5 * (j["pe_e"] + j["pe_d"])
    return np.stack([np.einsum("...c,...c->...", d, f["frente"]), np.einsum("...c,...c->...", d, f["esquerda"])], -1)
a0 = int(1.0 / cap.dt); b = off(cap.corpo[:, a0:].astype(float)); r = off(cap.referencia[:, a0:].astype(float))
ok = ~np.isnan(b[..., 0]) & ~np.isnan(r[..., 0]); d = np.linalg.norm(b - r, axis=-1)[ok]
print(f"{Path(sys.argv[2]).stem}: diferenca do peso contra o clipe {100*d.mean():.1f} cm (nota {np.mean(1/(1+(d/0.025)**2)):.2f}) | corpo frente/lado {np.round(100*np.nanmean(b[ok],0),1)} clipe {np.round(100*np.nanmean(r[ok],0),1)}")
