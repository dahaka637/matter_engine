"""O jogador de verdade nos fantasmas (o --malha do fantasma): os quadros da gravacao do Inspector (o .dump da captura, so o
mundo 0, um quadro a cada 2 decisoes) desenhados pelo MatterModelInspector com a malha em argila, recortados da folha e sem
o fundo, para o fantasma montar na vista lateral do movimento no lugar dos fantasmas de capsula.

Como funciona: a camera do Inspector mira sempre a pelve do quadro, 4 cm abaixo dela (o --seguir so prende a grade do
chao), entao cada quadro e uma foto centrada na pelve, na perspectiva dele, com o mesmo yaw e pitch para todos. A
montagem poe o centro de cada foto no lugar da pelve no referencial do caminho (ortografico): a pelve casa exata; o resto
erra o que a perspectiva de 7 m mexe (~3% da distancia a pelve, 2-3 cm no pe). O fundo e o degrade vertical do Inspector,
medido em cada linha nas bordas da celula (o corpo nao chega nelas); o alfa e a diferenca ao fundo (a argila mais escura
fica 60 acima dele). Medido 05/11 num quadro de pe do lev_parado: o alto da cabeca na linha 270 contra 270,6 pelas
capsulas (a escala 1:1), a sola 5 px abaixo (a chuteira da malha um pouco abaixo da casca do pe).

Regra termica: o treino parado (comum.require_stopped), um Inspector por vez e a CPU abaixo de LIMITE_C antes dele
(comum.wait_cool).
"""
from __future__ import annotations

import subprocess
from pathlib import Path

import numpy as np
from PIL import Image

from inspecao import comum

# (A folha do Inspector, ContactSheet.cpp: a margem, o titulo e a legenda de cada celula.)
MARGEM, TITULO, LEGENDA = 12, 44, 28
FOCO = 1.0 / np.tan(np.radians(14.0))  # (Camera.hpp: a lente vertical de 28 graus)
ALVO_Z = 0.04                          # (CommandFrames.cpp: a camera mira 4 cm abaixo da pelve)
DISTANCIA = 7.0
ESCALA = 220.0                         # (px/m no plano da pelve)
LARGURA = 640
LIMITE_C = 78.0


def altura_celula(escala: float = ESCALA, distancia: float = DISTANCIA) -> int:
    """A altura (px) da celula que da `escala` px/m no plano da pelve a `distancia` m."""
    return int(round(2.0 * escala * distancia / FOCO))


def renderizar(dump: Path, tempos, yaw: float, pitch: float = 0.0, distancia: float = DISTANCIA, escala: float = ESCALA,
               largura: int = LARGURA, saida: Path | None = None, log=print) -> list[np.ndarray]:
    """Os quadros do dump nos `tempos` (s, os do dump) com a camera a `yaw` e `pitch` graus e `distancia` m da pelve, em
    argila: [quadro] RGBA float 0-1 [altura, largura, 4], o alfa pela diferenca ao fundo. Um Inspector so, depois de a CPU
    esfriar abaixo de LIMITE_C, e so com o treino parado (a regra termica)."""
    comum.require_stopped()
    if not comum.INSPECTOR.exists():
        raise SystemExit(f"--malha: o Inspector nao existe ({comum.INSPECTOR})")
    if not Path(dump).exists():
        raise SystemExit(f"--malha: a captura nao tem a gravacao do Inspector ({dump})")
    tempos = [float(t) for t in tempos]
    altura = altura_celula(escala, distancia)
    colunas = min(16, len(tempos))
    saida = Path(saida or Path(dump).with_suffix(".malha.png"))
    comum.wait_cool(LIMITE_C, timeout=600.0, log=lambda m: log(f"  malha: {m}"))
    cmd = [str(comum.INSPECTOR), "quadros", "--dump", str(dump), "--instantes", ",".join(f"{t:.4f}" for t in tempos),
           "--seguir", "0", "--yaw", f"{yaw:.3f}", "--pitch", f"{pitch:.3f}", "--distancia", f"{distancia:.3f}",
           "--grade", "0.001", "--passo", "0.001", "--modo", "argila", "--largura", str(largura), "--altura", str(altura),
           "--colunas", str(colunas), "--saida", str(saida)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    if r.returncode != 0 or not saida.exists():
        raise SystemExit(f"--malha: o Inspector falhou ({r.returncode}): {r.stderr.strip()[-400:]}")
    folha = np.asarray(Image.open(saida).convert("RGB"), float) / 255.0
    celulas = []
    for i in range(len(tempos)):
        x = MARGEM + (i % colunas) * (largura + MARGEM)
        y = TITULO + (i // colunas) * (altura + LEGENDA + MARGEM)
        c = folha[y:y + altura, x:x + largura]
        if c.shape[:2] != (altura, largura):
            raise SystemExit(f"--malha: a folha do Inspector nao tem a celula {i} ({folha.shape})")
        fundo = np.median(np.concatenate([c[:, :4], c[:, -4:]], 1), 1)
        alfa = np.clip(((c - fundo[:, None]).max(-1) - 0.06) / 0.16, 0.0, 1.0)
        celulas.append(np.dstack([c, alfa]))
    return celulas


def tingir(celula: np.ndarray, cor, alfa: float) -> np.ndarray:
    """A celula na cor do fantasma: a cor vezes o sombreado da argila (o claro dela vira o claro da cor), o alfa vezes
    `alfa`."""
    luz = np.clip(celula[..., :3].max(-1) / 0.73, 0.0, 1.0)[..., None]
    rgb = np.asarray(cor, float)[:3] * (0.28 + 0.72 * luz)
    return np.dstack([rgb, celula[..., 3] * alfa])


def extensao(centro_x: float, centro_z: float, celula: np.ndarray, escala: float = ESCALA) -> tuple:
    """Os limites (x0, x1, z0, z1) em metros da celula com o centro dela no ponto da camera (a pelve 4 cm abaixo)."""
    h, w = celula.shape[:2]
    return (centro_x - 0.5 * w / escala, centro_x + 0.5 * w / escala, centro_z - ALVO_Z - 0.5 * h / escala,
            centro_z - ALVO_Z + 0.5 * h / escala)
