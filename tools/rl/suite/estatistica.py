"""A estatistica da suite (metodologia_rl.md, secao 5): intervalos, nao pontos.

Uma medida tem um valor por cenario e mundo. O valor do grupo e a media, entre os cenarios, da media dos mundos (nas medidas
'zero', com sinal, o modulo da media de cada cenario: o rumo da esquerda nao cancela o da direita). O IC95 e por bootstrap nos
mundos (o mesmo sorteio de mundos em todos os cenarios: os mundos sao os comecos, e os candidatos tem os mesmos); nas taxas de
um cenario so, o de Wilson. Tambem: a media interquartil e o pior mundo (Agarwal 2021; Chan 2020).

A comparacao com o primeiro candidato e pareada (os mesmos cenarios e mundos): o delta no sentido "pior" (positivo = piorou) e o
IC95 dele por bootstrap. A marca contra a margem m (conferida no limite do intervalo, Breck 2017):
  piorou  - o intervalo inteiro acima de +m;      melhorou - inteiro abaixo de -m;
  igual   - inteiro dentro de [-m, +m];            incerto+ / incerto- - o intervalo cruza a margem do lado pior / melhor.
Uma guarda com 'piorou' ou 'incerto+' nao passa (o limite pessimista passou da margem).
O piso (provisorio) e conferido no valor do grupo; os cenarios acima dele sao listados um a um.
As margens vem do margens.json (o teste A/A as preenche: {id da medida: margem}); sem ele, as provisorias de cada medida.
"""
from __future__ import annotations

import json
import warnings
from pathlib import Path

import numpy as np

BOOT = 2000
SEMENTE_BOOT = 20261110


def _matriz(por_cenario: dict, cenarios: list | None = None) -> tuple[list, np.ndarray]:
    nomes = cenarios or sorted(por_cenario)
    W = max(len(np.atleast_1d(v)) for v in por_cenario.values())
    X = np.full((len(nomes), W), np.nan)
    for i, n in enumerate(nomes):
        if n in por_cenario:
            v = np.asarray(por_cenario[n], float)
            X[i, :len(v)] = v
    return nomes, X


def _media(X: np.ndarray, axis) -> np.ndarray:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmean(X, axis=axis)


def estat(X: np.ndarray, sentido: str) -> np.ndarray | float:
    """O valor do grupo de uma matriz [cenarios, mundos] (ou [cenarios, sorteios, mundos] -> [sorteios])."""
    m = _media(X, -1)
    m = np.abs(m) if sentido == "zero" else m
    v = _media(m, 0)
    return float(v) if np.ndim(v) == 0 else v


def bootstrap(X: np.ndarray, sentido: str, n: int = BOOT, seed: int = SEMENTE_BOOT, Y: np.ndarray | None = None) -> tuple:
    """O IC95 do valor do grupo (ou, com Y, do delta pareado Y - X no sentido 'pior') sorteando os mundos com reposicao."""
    rng = np.random.default_rng(seed)
    W = X.shape[1]
    idx = rng.integers(0, W, (n, W))
    out = estat(X[:, idx], sentido)
    if Y is not None:
        out = pior(estat(Y[:, idx], sentido) - out, sentido)
    out = out[np.isfinite(out)]
    return (float(np.percentile(out, 2.5)), float(np.percentile(out, 97.5))) if out.size else (float("nan"), float("nan"))


def pior(delta: float, sentido: str) -> float:
    """O delta com o sinal de 'piorou' positivo."""
    return -delta if sentido == "maior" else delta


def wilson(k: float, n: float, z: float = 1.96) -> tuple:
    if n <= 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (float(max(0.0, c - h)), float(min(1.0, c + h)))


def iqm(x: np.ndarray) -> float:
    x = np.sort(x[np.isfinite(x)])
    if not x.size:
        return float("nan")
    lo, hi = int(np.floor(0.25 * x.size)), int(np.ceil(0.75 * x.size))
    return float(x[lo:max(hi, lo + 1)].mean())


def resumo(med) -> dict:
    """Valor, IC95, IQM, pior mundo, os valores por mundo e por cenario, e o piso (no grupo e por cenario)."""
    nomes, X = _matriz(med.por_cenario)
    valor = estat(X, med.sentido)
    flat = X.ravel()
    ok = np.isfinite(flat)
    if med.tipo == "taxa" and len(nomes) == 1:
        ic = wilson(float(np.nansum(flat)), float(ok.sum()))
    else:
        ic = bootstrap(X, med.sentido)
    if ok.any():
        chave = np.abs(flat) if med.sentido == "zero" else flat if med.sentido == "menor" else -flat
        k = int(np.nanargmax(np.where(ok, chave, -np.inf)))
        pior_mundo = {"valor": float(flat[k]), "cenario": nomes[k // X.shape[1]], "mundo": int(k % X.shape[1])}
    else:
        pior_mundo = None
    cen = {n: float(v) for n, v in zip(nomes, _media(X, 1))}
    out = {"id": med.id, "teste": med.teste, "titulo": med.titulo, "unidade": med.unidade, "sentido": med.sentido,
           "tipo": med.tipo, "valor": valor, "ic95": ic, "iqm": iqm(np.abs(flat) if med.sentido == "zero" else flat),
           "n": int(ok.sum()), "pior_mundo": pior_mundo, "por_cenario": cen,
           "por_mundo": {n: [None if not np.isfinite(v) else round(float(v), 4) for v in X[i]] for i, n in enumerate(nomes)},
           "piso": med.piso, "margem_provisoria": med.margem}
    if med.sentido == "zero":
        out["valor_com_sinal"] = float(np.nanmean(list(cen.values()))) if cen else float("nan")
    if med.piso is not None:
        # (O piso pega o desabamento no valor do grupo; cenario a cenario ele so lista, para os "passavam e passaram a falhar"
        # contra a referencia - metodologia, secao 3. Os pisos vieram dos rumos de frente: num cenario so, as diagonais e as
        # paradas de costas passam dele ate no principal.)
        out["passa_piso"] = passa(valor, med.piso, med.sentido)
        out["cenarios_falham"] = sorted(n for n, v in cen.items() if np.isfinite(v) and not passa(v, med.piso, med.sentido))
    return out


def passa(v: float, piso: float, sentido: str) -> bool:
    if not np.isfinite(v):
        return True
    if sentido == "maior":
        return v >= piso
    return (abs(v) if sentido == "zero" else v) <= piso


def comparar(a, b, margem: float | None) -> dict:
    """O delta pareado de b contra a (Medida, Medida), no sentido 'pior', com o IC95 e a marca contra a margem."""
    nomes = sorted(set(a.por_cenario) | set(b.por_cenario))
    _, X = _matriz(a.por_cenario, nomes)
    _, Y = _matriz(b.por_cenario, nomes)
    d = pior(estat(Y, a.sentido) - estat(X, a.sentido), a.sentido)
    lo, hi = bootstrap(X, a.sentido, Y=Y)
    return {"delta_pior": d, "ic95": (lo, hi), "margem": margem, "marca": marca(lo, hi, margem)}


def marca(lo: float, hi: float, m: float | None) -> str:
    if m is None or not (np.isfinite(lo) and np.isfinite(hi)):
        return "?"
    if lo > m:
        return "piorou"
    if hi < -m:
        return "melhorou"
    if lo >= -m and hi <= m:
        return "igual"
    return "incerto+" if hi > m else "incerto-"


def margens(path: str | Path | None) -> dict:
    return json.loads(Path(path).read_text()) if path and Path(path).exists() else {}
