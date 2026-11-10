"""A geometria compartilhada das ferramentas de inspecao: a captura carregada, as pontas das capsulas, as articulacoes
(quadril, joelho, tornozelo, ombro, cotovelo, punho, a ponta e o calcanhar do pe), o centro de massa e o referencial do
corpo (frente e esquerda pelo quadril, z para cima). Convencoes: metros, z para cima, + = esquerda; os quaternios xyzw do
colisor; o eixo da capsula e o x local do colisor.

Uso: cap = carregar("x.npz"); j = articulacoes(cap.corpo, cap.geo); f = referencial(cap.corpo)
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

UP = np.array([0.0, 0.0, 1.0])
# Os elos (a ordem do perfil FootballPlayerV1, a mesma da captura).
LINKS = ("Pelvis", "Abdomen", "Chest", "UpperChest", "Neck", "Head", "LeftUpperArm", "LeftForearm", "LeftHand",
         "RightUpperArm", "RightForearm", "RightHand", "LeftThigh", "LeftShin", "LeftFoot", "RightThigh", "RightShin",
         "RightFoot")
IDX = {n: i for i, n in enumerate(LINKS)}


def qrot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Gira v pelo quaternio q (xyzw), vetorizado: q [..., 4], v [..., 3] (ou [3])."""
    q = np.asarray(q, float)
    v = np.broadcast_to(np.asarray(v, float), q.shape[:-1] + (3,))
    u, w = q[..., :3], q[..., 3:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


@dataclass
class Captura:
    """Uma captura (inspecao.captura): corpo e referencia [mundos, passos, elos, 7], medidas, acoes, info, contatos,
    episodios e os metadados."""
    caminho: Path
    meta: dict
    corpo: np.ndarray
    referencia: np.ndarray
    medidas: np.ndarray
    acoes: np.ndarray
    info: np.ndarray
    episodio: np.ndarray
    contatos: np.ndarray
    episodios: np.ndarray
    geo: list

    @property
    def dt(self) -> float:
        return float(self.meta["dt"])

    @property
    def nome(self) -> str:
        return f"{self.meta['exercicio']}__{self.meta['modo']}"

    def medida(self, nome: str) -> np.ndarray:
        """Uma medida do rastreio por nome ([mundos, passos])."""
        return self.medidas[..., self.meta["medidas"].index(nome)]

    def vivo(self) -> np.ndarray:
        """[mundos, passos]: o corpo registrado (fora do fim)."""
        return np.isfinite(self.corpo[..., 0, 0])

    def pe_no_chao(self) -> np.ndarray:
        """[mundos, passos, 2]: o pe esquerdo e o direito com contato (pelos pontos de contato da captura)."""
        out = np.zeros(self.corpo.shape[:2] + (2,), bool)
        c = self.contatos
        for s, link in enumerate((IDX["LeftFoot"], IDX["RightFoot"])):
            sel = c[c[:, 5] == link]
            out[sel[:, 0].astype(int), sel[:, 1].astype(int), s] = True
        return out


def carregar(path: str | Path) -> Captura:
    path = Path(path)
    z = np.load(path)
    meta = json.loads(path.with_suffix(".json").read_text())
    return Captura(path, meta, z["corpo"], z["referencia"], z["medidas"], z["acoes"], z["info"], z["episodio"],
                   z["contatos"], z["episodios"], meta["ragdoll"]["links"])


def capsula(frames: np.ndarray, geo: dict) -> tuple[np.ndarray, np.ndarray, float]:
    """As duas pontas do segmento da capsula (os centros das tampas) [..., 3] cada, e o raio medio. A ponta "a" e a do -x
    local, a "b" a do +x."""
    half = max(0.0, 0.5 * geo["comprimento"] - 0.5 * (geo["raio"] + geo.get("raio_mais_x", geo["raio"])))
    c, q = frames[..., :3], frames[..., 3:7]
    axis = qrot(q, np.array([1.0, 0.0, 0.0]))
    return c - half * axis, c + half * axis, 0.5 * (geo["raio"] + geo.get("raio_mais_x", geo["raio"]))


def caixa(frames: np.ndarray, geo: dict) -> np.ndarray:
    """Os 8 cantos da caixa (a caixa justa da casca do pe) no mundo: [..., 8, 3]."""
    h = np.array(geo["meia_caixa"], float)
    m = np.array(geo.get("centro_caixa", [0.0, 0.0, 0.0]), float)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float) * h + m
    c, q = frames[..., None, :3], frames[..., None, 3:7]
    return c + qrot(np.broadcast_to(q, q.shape[:-2] + (8, 4)), corners)


def _ordered(frames: np.ndarray, geo: list, child: str, parent_point: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """As pontas da capsula do elo `child` na ordem (proxima ao pai, distante)."""
    a, b, _ = capsula(frames[..., IDX[child], :], geo[IDX[child]])
    near_a = np.linalg.norm(a - parent_point, axis=-1) <= np.linalg.norm(b - parent_point, axis=-1)
    prox = np.where(near_a[..., None], a, b)
    dist = np.where(near_a[..., None], b, a)
    return prox, dist


def articulacoes(frames: np.ndarray, geo: list) -> dict:
    """As articulacoes no mundo ([..., 3] cada) de quadros [..., 18, 7]: pelve, quadris, joelhos, tornozelos, ponta e
    calcanhar dos pes, ombros, cotovelos, punhos, pescoco, cabeca e o centro de massa."""
    out = {}
    pelvis = frames[..., IDX["Pelvis"], :3]
    out["pelve"] = pelvis
    for side, s in (("Left", "e"), ("Right", "d")):
        hip, knee_t = _ordered(frames, geo, f"{side}Thigh", pelvis)
        knee_s, ankle = _ordered(frames, geo, f"{side}Shin", knee_t)
        out[f"quadril_{s}"] = hip
        out[f"joelho_{s}"] = 0.5 * (knee_t + knee_s)
        out[f"tornozelo_{s}"] = ankle
        corners = caixa(frames[..., IDX[f"{side}Foot"], :], geo[IDX[f"{side}Foot"]])
        # (A ponta e o calcanhar: o meio da aresta da sola - o -z local - na ponta (+x local) e no calcanhar (-x). 05/11:
        # antes o canto mais longe no rumo do pe, que trocava de canto quase a cada passo - a revisao do fantasma.)
        out[f"ponta_{s}"] = 0.5 * (corners[..., 4, :] + corners[..., 6, :])
        out[f"calcanhar_{s}"] = 0.5 * (corners[..., 0, :] + corners[..., 2, :])
        out[f"pe_{s}"] = frames[..., IDX[f"{side}Foot"], :3]
        shoulder, elbow_u = _ordered(frames, geo, f"{side}UpperArm", frames[..., IDX["UpperChest"], :3])
        elbow_f, wrist = _ordered(frames, geo, f"{side}Forearm", elbow_u)
        out[f"ombro_{s}"] = shoulder
        out[f"cotovelo_{s}"] = 0.5 * (elbow_u + elbow_f)
        out[f"punho_{s}"] = wrist
        out[f"mao_{s}"] = frames[..., IDX[f"{side}Hand"], :3]
    neck_base, _ = _ordered(frames, geo, "Neck", frames[..., IDX["UpperChest"], :3])
    _, head_top = _ordered(frames, geo, "Head", neck_base)
    out["pescoco"] = neck_base
    out["cabeca"] = frames[..., IDX["Head"], :3]
    out["topo_cabeca"] = head_top
    out["peito"] = frames[..., IDX["UpperChest"], :3]
    masses = np.array([g["massa"] for g in geo], float)
    out["centro_massa"] = np.einsum("...kc,k->...c", frames[..., :3], masses) / masses.sum()
    out["_quadros"] = frames
    return out


def referencial(frames: np.ndarray) -> dict:
    """O referencial do corpo em cada passo pela orientacao do abdome (o x local do colisor dele aponta para a esquerda):
    'esquerda' e 'frente' (esquerda x cima) unitarios no plano [..., 3], e o 'rumo' do corpo (rad). (05/11: antes pela linha
    dos quadris - as pontas das coxas, que andam com a perna: balancava 60-90 graus na passada e trocava o sinal da
    inclinacao de lado no sprint; o abdome, entre a pelve e o peito que giram ao contrario, balanca +-1,7 grau. O mesmo de
    marcha.quadro e fantasma._quadro.) Aceita tambem o dict de articulacoes antigo se ele trouxer '_quadros'."""
    if isinstance(frames, dict):
        frames = frames["_quadros"]
    left = qrot(frames[..., IDX["Abdomen"], 3:7].astype(float), np.array([1.0, 0.0, 0.0]))
    left[..., 2] = 0.0
    left /= np.maximum(np.linalg.norm(left, axis=-1, keepdims=True), 1e-9)
    forward = np.cross(left, UP)
    return {"esquerda": left, "frente": forward, "rumo": np.arctan2(forward[..., 1], forward[..., 0])}


def inclinacao(top: np.ndarray, bottom: np.ndarray, axis: np.ndarray) -> np.ndarray:
    """O angulo (graus) do segmento bottom->top em relacao a vertical, no plano vertical do `axis` (+ = para o lado de
    `axis`): a inclinacao de lado com axis = esquerda, para a frente com axis = frente."""
    v = top - bottom
    return np.degrees(np.arctan2(np.einsum("...c,...c->...", v, axis), v[..., 2]))


def angulo(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """O angulo (graus) entre os vetores a e b."""
    na = np.linalg.norm(a, axis=-1)
    nb = np.linalg.norm(b, axis=-1)
    return np.degrees(np.arccos(np.clip(np.einsum("...c,...c->...", a, b) / np.maximum(na * nb, 1e-12), -1.0, 1.0)))


def autoteste(amostras: str) -> None:
    """Confere a geometria nas amostras: o clipe puro ereto de lado (|vies| < 1,5 grau), os joelhos entre 0 e 150 graus,
    o centro de massa entre os pes no clipe puro parado."""
    base = Path(amostras)
    for name in ("lev_trote_0__clipe", "lev_parado__clipe"):
        cap = carregar(base / f"{name}.npz")
        alive = cap.vivo()
        j = articulacoes(cap.corpo, cap.geo)
        f = referencial(cap.corpo)
        side = inclinacao(j["cabeca"], 0.5 * (j["tornozelo_e"] + j["tornozelo_d"]), f["esquerda"])[alive]
        knee = angulo(j["quadril_e"] - j["joelho_e"], j["tornozelo_e"] - j["joelho_e"])[alive]
        print(f"{name}: inclinacao de lado do corpo vies {np.nanmean(side):+.2f} balanco {np.nanstd(side):.2f}; "
              f"joelho esq (angulo interno) {np.nanmin(knee):.0f}-{np.nanmax(knee):.0f} graus")
        assert abs(np.nanmean(side)) < 1.5, "o clipe puro deveria ser ereto de lado"
        assert 20.0 < np.nanmin(knee) and np.nanmax(knee) <= 181.0
    print("geometria ok")


if __name__ == "__main__":
    import sys
    autoteste(sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parents[3] / "build-profile/tmp/inspecao_amostras_0645"))
