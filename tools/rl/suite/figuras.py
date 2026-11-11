"""As figuras da suite (o que ele ve), das mesmas capturas das medidas (sem simular de novo):
  parado.png - por candidato, 4 mundos do parado sem empurrao (a media da rede): o rastro da pelve e dos pes visto de cima (cm,
               desde onde o parado comecou; o x marca os pes do idle fixo) e o corpo de frente e de lado a cada 1 s,
               sobreposto (estatico = um corpo so) - o figura_parado.py de 10/11;
  freada.png - por candidato, a parada da corrida de frente (a ordem aos 4 s), 2 mundos, de lado - o filme.py de 10/11.
O corpo e desenhado pelas articulacoes (inspecao.geometria.articulacoes: o eixo da capsula e o x local do colisor).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from inspecao.geometria import articulacoes
from suite.medidas.base import IDX, PES, ancorar, carregar, qrot

TEMPOS_FREADA = (3.6, 4.2, 4.6, 5.0, 5.4, 5.8, 6.4, 7.4)
SEGMENTOS = {"coluna": ("pelve", "abdome", "torax", "peito", "pescoco", "cabeca", "topo_cabeca"),
             "e": ("ponta_e", "tornozelo_e", "joelho_e", "quadril_e", "pelve", "quadril_d", "joelho_d", "tornozelo_d", "ponta_d"),
             "be": ("pescoco", "ombro_e", "cotovelo_e", "punho_e", "mao_e"), "bd": ("pescoco", "ombro_d", "cotovelo_d", "punho_d", "mao_d")}
CORES = {"coluna": "#d62728", "e": "#555555", "be": "#1f77b4", "bd": "#2ca02c"}


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def pontos(fr: np.ndarray, geo: list) -> dict:
    j = articulacoes(fr, geo)
    j["abdome"], j["torax"], j["peito"] = (fr[..., IDX[n], :3] for n in ("Abdomen", "Chest", "UpperChest"))
    return j


def desenhar(ax, j: dict, origem: np.ndarray, frente: np.ndarray, vista: str, alpha: float = 1.0, lw: float = 2.0) -> None:
    """O corpo (as articulacoes j de um quadro) de frente (o eixo esquerda na horizontal) ou de lado (o eixo frente)."""
    esquerda = np.array([-frente[1], frente[0], 0.0])
    eixo = frente if vista == "lado" else -esquerda
    for k, seq in SEGMENTOS.items():
        p = np.array([j[n] for n in seq]) - origem
        ax.plot(p @ eixo, p[:, 2], color=CORES[k], lw=lw + (1.0 if k == "coluna" else 0.0), alpha=alpha, solid_capstyle="round")


def _frente(fr: np.ndarray) -> np.ndarray:
    hip = qrot(fr[IDX["Pelvis"], 3:7].astype(float), np.array([0.0, 0.0, 1.0]))
    yaw = np.arctan2(hip[1], hip[0]) - np.pi / 2
    return np.array([np.cos(yaw), np.sin(yaw), 0.0])


def parado(pastas: dict, saida: Path, mundos: int = 4) -> Path:
    """pastas: {candidato: pasta das capturas}. Usa parado__det."""
    plt = _plt()
    caps = {n: carregar(Path(p) / "parado__det.npz") for n, p in pastas.items()}
    fig, axes = plt.subplots(len(caps), 3 * mundos, figsize=(3.0 * 3 * mundos, 3.4 * len(caps)), squeeze=False)
    for r, (nome, cap) in enumerate(caps.items()):
        vivo = cap.vivo()
        fixo = ancorar(cap.ref, vivo, cap.corpo)
        geo = cap.geo()
        passo_1s = int(round(1.0 / cap.dt))
        for w in range(min(mundos, cap.W)):
            idx = np.where(vivo[w])[0]
            p0 = cap.corpo[w, idx[0], IDX["Pelvis"], :2].astype(float)
            ax = axes[r, 3 * w]
            for k, cor, rot in ((IDX["Pelvis"], "k", "pelve"), (PES[0], "#1f77b4", "pe esq"), (PES[1], "#2ca02c", "pe dir")):
                xy = (cap.corpo[w, idx, k, :2] - p0) * 100.0
                ax.plot(xy[:, 0], xy[:, 1], color=cor, lw=1.2, label=rot)
                ax.plot(xy[0, 0], xy[0, 1], "o", color=cor, ms=4)
                if k in PES:
                    alvo = (fixo[w, idx[0], k, :2] - p0) * 100.0
                    ax.plot(alvo[0], alvo[1], "x", color=cor, ms=9, mew=2)
            andou = [np.linalg.norm(np.diff(cap.corpo[w, idx, k, :2], axis=0), axis=-1).sum() * 100.0 for k in PES]
            ax.set_aspect("equal")
            ax.set_xlim(-45, 45)
            ax.set_ylim(-45, 45)
            ax.grid(alpha=0.3)
            ax.set_title(f"{nome} m{w}: de cima (cm)\npes andaram {andou[0]:.0f} / {andou[1]:.0f} cm", fontsize=8)
            if w == 0:
                ax.legend(fontsize=6, loc="lower left")
            frente = _frente(cap.corpo[w, idx[0]])
            origem = cap.corpo[w, idx[0], IDX["Pelvis"], :3].astype(float).copy()
            origem[2] = 0.0
            quadros = idx[passo_1s - 1::passo_1s]
            for v, vista in enumerate(("frente", "lado")):
                ax = axes[r, 3 * w + 1 + v]
                desenhar(ax, pontos(fixo[w, idx[0]], geo), origem, frente, vista, alpha=0.35, lw=5.0)
                for q, i in enumerate(quadros):
                    desenhar(ax, pontos(cap.corpo[w, i].astype(float), geo), origem, frente, vista,
                             alpha=0.15 + 0.85 * q / max(1, len(quadros) - 1), lw=1.2)
                ax.set_xlim(-0.7, 0.7)
                ax.set_ylim(-0.05, 1.95)
                ax.set_aspect("equal")
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_title(f"{vista}: a cada 1 s sobreposto\n(largo e claro: o idle fixo)", fontsize=8)
    fig.suptitle("o parado sem empurrao, 10 s, a media da rede: estatico = o rastro num ponto so, os pes no x, um corpo so",
                 fontsize=10)
    fig.tight_layout()
    saida.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(saida, dpi=80)
    plt.close(fig)
    return saida


def freada(pastas: dict, saida: Path, mundos: int = 2, cenario: str = "parada_corrida_0__det") -> Path:
    plt = _plt()
    caps = {n: carregar(Path(p) / f"{cenario}.npz") for n, p in pastas.items()}
    linhas = len(caps) * mundos
    fig, axes = plt.subplots(linhas, len(TEMPOS_FREADA), figsize=(1.9 * len(TEMPOS_FREADA), 2.7 * linhas), squeeze=False)
    for c, (nome, cap) in enumerate(caps.items()):
        geo = cap.geo()
        vivo = cap.vivo()
        for w in range(mundos):
            r = c * mundos + w
            rumo = float(np.nanmedian(cap.z["info"][w, :, 4]))
            frente = np.array([np.cos(rumo), np.sin(rumo), 0.0])
            for k, t in enumerate(TEMPOS_FREADA):
                ax = axes[r, k]
                i = int(round(t / cap.dt)) - 1
                if i < vivo.shape[1] and vivo[w, i]:
                    fr = cap.corpo[w, i].astype(float)
                    origem = fr[IDX["Pelvis"], :3].copy()
                    origem[2] = 0.0
                    desenhar(ax, pontos(fr, geo), origem, frente, "lado", lw=2.0)
                else:
                    ax.text(0.5, 0.5, "caiu", ha="center", transform=ax.transAxes)
                ax.axhline(0.0, color="k", lw=0.6)
                ax.set_xlim(-0.8, 0.8)
                ax.set_ylim(-0.05, 1.95)
                ax.set_aspect("equal")
                ax.set_xticks([])
                ax.set_yticks([])
                if r == 0:
                    ax.set_title(f"{t:.1f} s", fontsize=8)
                if k == 0:
                    ax.set_ylabel(f"{nome}\nmundo {w}", fontsize=8)
    fig.suptitle("a parada da corrida de frente, de lado (correndo para a direita); a ordem de parar aos 4,0 s | vermelho: coluna "
                 "e cabeca, azul: braco esquerdo, verde: direito", fontsize=9)
    fig.tight_layout()
    saida.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(saida, dpi=90)
    plt.close(fig)
    return saida
