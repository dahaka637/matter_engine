"""(15/11 12h50, ele: "notei um leve desalinhamento das pernas nas locomocoes frontais pelo menos, as outras nao vi mas acredito que
tambem") O alinhamento das pernas no cruzeiro (depois de 2 s), corpo contra o clipe seguido no mesmo quadro (a referencia posta sob a
pelve), no referencial de cada um (o abdome), so com o pe apoiado (o mais baixo da ponta e do calcanhar a menos de 4 cm do chao):
  - pe: o giro da ponta no chao contra a frente (+ = ponta para fora);
  - joelho: para onde o joelho sai da linha quadril-tornozelo, no chao, contra a frente (+ = para fora; 0 = para a frente);
  - largura: o pe ate a linha do meio do corpo (cm, + = para fora do seu lado);
  - abducao: a coxa contra a vertical no plano de lado (graus, + = para fora).
Uso: alinhamento_pernas.py TREINO PONTO [CHAVES] [segundos=5]"""
import sys
from pathlib import Path
import numpy as np
W = Path("/home/dahaka/Projetos/matter_engine_wip"); D = W / "build-profile/tmp/rolar_r72889"
sys.path.insert(0, str(W / "tools/rl"))
from inspecao import comum, captura  # noqa: E402
from inspecao import geometria as G  # noqa: E402
treino, ck = sys.argv[1], sys.argv[2]
KEYS = (sys.argv[3] if len(sys.argv) > 3 else "lev_corrida_0,lev_trote_0,lev_corrida_1,lev_corrida_7,lev_corrida_4,lev_corrida_2,lev_corrida_6").split(",")
SECS = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0
amb = comum.Ambiente(treino, ck)
tag = Path(ck).stem
def medidas(frames, geo):
    j = G.articulacoes(frames, geo); f = G.referencial(frames); out = {}
    for s, sg in (("e", 1.0), ("d", -1.0)):
        fr, le = f["frente"], f["esquerda"] * sg  # le: o "para fora" deste lado
        toe = j[f"ponta_{s}"] - j[f"calcanhar_{s}"]; toe[..., 2] = 0
        out[f"pe_{s}"] = np.degrees(np.arctan2(np.einsum("...c,...c->...", toe, le), np.einsum("...c,...c->...", toe, fr)))
        hip, knee, ank = j[f"quadril_{s}"], j[f"joelho_{s}"], j[f"tornozelo_{s}"]
        line = ank - hip; u = line / np.maximum(np.linalg.norm(line, axis=-1, keepdims=True), 1e-9)
        off = (knee - hip) - np.einsum("...c,...c->...", knee - hip, u)[..., None] * u; off[..., 2] = 0
        out[f"joelho_{s}"] = np.degrees(np.arctan2(np.einsum("...c,...c->...", off, le), np.einsum("...c,...c->...", off, fr)))
        out[f"largura_{s}"] = 100 * np.einsum("...c,...c->...", j[f"pe_{s}"] - j["pelve"], le)
        out[f"abducao_{s}"] = G.inclinacao(knee, hip, le) * -1.0  # a coxa para fora: o joelho para o lado de fora do quadril
        low = np.minimum(j[f"ponta_{s}"][..., 2], j[f"calcanhar_{s}"][..., 2])
        out[f"apoio_{s}"] = low < 0.04
    return out
print(f"ponto {tag}: corpo / clipe (pe apoiado, cruzeiro depois de 2 s)", flush=True)
for key in KEYS:
    npz = captura.capturar(amb, key, "atual", 4, SECS, D / "distancia" / f"alinha_{tag}_{key}", semente=61, log=lambda *a: None)
    cap = G.carregar(npz)
    a0 = int(round(2.0 / cap.dt))
    body, ref = cap.corpo[:, a0:].astype(float), cap.referencia[:, a0:].astype(float)
    ok = ~np.isnan(ref[..., 0, 0]) & ~np.isnan(body[..., 0, 0]) & (body[..., G.IDX["Pelvis"], 2] > 0.6)
    mb, mr = medidas(body, cap.geo if hasattr(cap, "geo") else cap.links), medidas(ref, cap.geo if hasattr(cap, "geo") else cap.links)
    line = [key]
    for s in ("e", "d"):
        sb = ok & mb[f"apoio_{s}"]; sr = ok & mr[f"apoio_{s}"]
        line.append(f"{'esq' if s == 'e' else 'dir'}: pe {np.nanmean(mb[f'pe_{s}'][sb]):+.1f}/{np.nanmean(mr[f'pe_{s}'][sr]):+.1f}"
                    f" joelho {np.nanmean(mb[f'joelho_{s}'][sb]):+.1f}/{np.nanmean(mr[f'joelho_{s}'][sr]):+.1f}"
                    f" largura {np.nanmean(mb[f'largura_{s}'][sb]):+.1f}/{np.nanmean(mr[f'largura_{s}'][sr]):+.1f}"
                    f" abducao {np.nanmean(mb[f'abducao_{s}'][ok]):+.1f}/{np.nanmean(mr[f'abducao_{s}'][ok]):+.1f}")
    print(" | ".join(line), flush=True)
