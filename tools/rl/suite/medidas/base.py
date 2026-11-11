"""O que as medidas compartilham: a captura carregada, as janelas de tempo, os quaternios, os ossos (o colisor sem a capsula), a
postura pelas posicoes dos elos (a da regua_postura de 10/11) e a Medida (os valores por cenario e mundo, o sentido, o piso).
Numpy puro: nada aqui le a nota do treino."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from inspecao.geometria import IDX, LINKS, UP, articulacoes, qrot  # noqa: F401

PES = (IDX["LeftFoot"], IDX["RightFoot"])
BRACOS = tuple(IDX[n] for n in ("LeftUpperArm", "LeftForearm", "LeftHand", "RightUpperArm", "RightForearm", "RightHand"))
COLUNA = ("Abdomen", "Chest", "UpperChest", "Neck")


@dataclass
class Medida:
    """Uma medida da suite: por cenario, um valor por mundo (nan = sem amostra, ex. caiu antes da janela).
    sentido: 'menor' (menor e melhor), 'maior' (maior e melhor) ou 'zero' (com sinal; |valor| menor e melhor).
    tipo: 'continua' ou 'taxa' (0/1 por mundo). piso: o limite absoluto provisorio (no sentido da medida), conferido no grupo e
    em cada cenario (os cenarios criticos). margem: a provisoria (o A/A a substitui pelo margens.json)."""
    id: str
    teste: str
    titulo: str
    unidade: str
    sentido: str
    por_cenario: dict = field(default_factory=dict)
    tipo: str = "continua"
    piso: float | None = None
    margem: float | None = None


@dataclass
class Cap:
    caminho: Path
    meta: dict
    z: dict

    @property
    def dt(self) -> float:
        return float(self.meta["dt"])

    @property
    def corpo(self) -> np.ndarray:
        return self.z["corpo"]

    @property
    def ref(self) -> np.ndarray:
        return self.z["referencia"]

    @property
    def W(self) -> int:
        return self.corpo.shape[0]

    def t(self) -> np.ndarray:
        """O instante de cada passo (s): o quadro do passo i e o estado depois do i-esimo step."""
        return (np.arange(self.corpo.shape[1]) + 1) * self.dt

    def vivo(self) -> np.ndarray:
        """[W, T]: o primeiro episodio do mundo, com o corpo registrado."""
        return (self.z["episodio"] == 0) & np.isfinite(self.corpo[..., 0, 0])

    def janela(self, t0: float, t1: float) -> np.ndarray:
        t = self.t()
        return self.vivo() & ((t >= t0) & (t <= t1))[None]

    def caiu(self) -> np.ndarray:
        """[W] 0/1: o primeiro episodio acabou antes do fim do cenario (queda ou outro fim antecipado)."""
        return self.fim() < self.meta["cenario"]["segundos"] - 0.1

    def fim(self) -> np.ndarray:
        """[W] o instante do fim do primeiro episodio (s; o fim do cenario se nao acabou)."""
        out = np.full(self.W, float(self.meta["cenario"]["segundos"]))
        e, c = self.z["episodios"], self.meta["colunas_episodios"]
        vistos = set()
        for r in e:
            w = int(r[0])
            if w not in vistos:
                vistos.add(w)
                out[w] = float(r[c.index("segundos")])
        return out

    def pes_no_chao(self) -> np.ndarray:
        return self.z["contato"][..., PES]

    def geo(self) -> list:
        return self.meta["geo"]


def carregar(path: str | Path) -> Cap:
    path = Path(path)
    z = np.load(path)
    return Cap(path, json.loads(path.with_suffix(".json").read_text()), {k: z[k] for k in z.files})


def capturas(pasta: str | Path) -> dict[str, Cap]:
    """{'<cenario>__<condicao>': Cap} de uma pasta de candidato."""
    return {p.stem: carregar(p) for p in sorted(Path(pasta).glob("*.npz"))}


# --- quaternios (xyzw) ---------------------------------------------------------------------------------------------------

def qconj(q):
    return np.concatenate([-q[..., :3], q[..., 3:]], -1)


def qmul(a, b):
    ax, ay, az, aw = (a[..., i] for i in range(4))
    bx, by, bz, bw = (b[..., i] for i in range(4))
    return np.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], -1)


def qang(a, b):
    """O angulo (graus) entre duas orientacoes."""
    d = np.abs(np.sum(np.asarray(a, float) * np.asarray(b, float), -1))
    return 2.0 * np.degrees(np.arccos(np.clip(d, 0.0, 1.0)))


def qz(angle):
    """O quaternio de um giro em volta de z."""
    h = 0.5 * np.asarray(angle, float)
    return np.stack([np.zeros_like(h), np.zeros_like(h), np.sin(h), np.cos(h)], -1)


def wrap(deg):
    return (np.asarray(deg) + 180.0) % 360.0 - 180.0


def ossos(fr: np.ndarray, links: list) -> np.ndarray:
    """[..., 18, 4]: a orientacao de cada osso = o colisor vezes o inverso da orientacao da capsula no osso (os sem capsula, o
    proprio colisor) - como sonda_coluna.bone."""
    caps = np.array([lk["capsule"]["orientation"] if "capsule" in lk else [0.0, 0.0, 0.0, 1.0] for lk in links], float)
    return qmul(fr[..., 3:7].astype(float), qconj(caps))


# --- a postura pelas posicoes (regua_postura.feats, vetorizada) ------------------------------------------------------------

def postura(fr: np.ndarray) -> dict:
    """De quadros [..., 18, 7], no quadro do rumo do quadril: a coluna (pelve -> pescoco) para a frente e de lado (graus), o arco
    do peito (cm, o peito fora da linha pelve-pescoco), a torcao ombros-quadril (graus), a cabeca (pescoco -> cabeca) para a
    frente (graus) e a altura da pelve (cm)."""
    fr = fr.astype(float)
    hip = qrot(fr[..., IDX["Pelvis"], 3:7], np.array([0.0, 0.0, 1.0]))
    sh = qrot(fr[..., IDX["UpperChest"], 3:7], np.array([1.0, 0.0, 0.0]))
    yh = np.arctan2(hip[..., 1], hip[..., 0]) - np.pi / 2
    ys = np.arctan2(sh[..., 1], sh[..., 0]) - np.pi / 2
    c, s = np.cos(yh)[..., None], np.sin(yh)[..., None]

    def loc(v):
        return np.concatenate([c * v[..., :1] + s * v[..., 1:2], -s * v[..., :1] + c * v[..., 1:2], v[..., 2:3]], -1)

    pel, neck, head, chest = (fr[..., IDX[n], :3] for n in ("Pelvis", "Neck", "Head", "Chest"))
    sp, ch, hd = loc(neck - pel), loc(chest - pel), loc(head - neck)
    t = np.sum(ch * sp, -1, keepdims=True) / np.sum(sp * sp, -1, keepdims=True)
    arc = ch - t * sp
    return {"coluna_frente": np.degrees(np.arctan2(sp[..., 0], sp[..., 2])),
            "coluna_lado": np.degrees(np.arctan2(sp[..., 1], sp[..., 2])),
            "arco": arc[..., 0] * 100.0, "torcao": wrap(np.degrees(ys - yh)),
            "cabeca": np.degrees(np.arctan2(hd[..., 0], hd[..., 2])), "pelve": pel[..., 2] * 100.0}


def rumo_peito(fr: np.ndarray) -> np.ndarray:
    """O rumo (rad) para onde o peito (o peito alto) aponta no chao."""
    sh = qrot(fr[..., IDX["UpperChest"], 3:7].astype(float), np.array([1.0, 0.0, 0.0]))
    return np.arctan2(sh[..., 1], sh[..., 0]) - np.pi / 2


def centro_massa(fr: np.ndarray, geo: list) -> np.ndarray:
    m = np.array([g["massa"] for g in geo], float)
    return np.einsum("...kc,k->...c", fr[..., :3].astype(float), m) / m.sum()


def velocidade_angular(fr: np.ndarray, dt: float, elos=None) -> np.ndarray:
    """[W, T, elos] graus/s entre passos consecutivos (nan no primeiro)."""
    q = fr[..., 3:7].astype(float) if elos is None else fr[:, :, list(elos), 3:7].astype(float)
    out = np.full(q.shape[:-1], np.nan)
    out[:, 1:] = qang(q[:, 1:], q[:, :-1]) / dt
    return out


def pousos(no_chao: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """[W] o numero de pousos (o pe saindo do ar para o chao) dentro da mascara, somados os dois pes."""
    land = no_chao[:, 1:] & ~no_chao[:, :-1]
    m = mask[:, 1:] & mask[:, :-1]
    return (land & m[..., None]).sum((1, 2)).astype(float)


def por_mundo(x: np.ndarray, mask: np.ndarray, func=np.nanmedian) -> np.ndarray:
    """[W]: func de x [W, T] sobre os passos da mascara (nan sem amostra)."""
    out = np.full(x.shape[0], np.nan)
    for w in range(x.shape[0]):
        v = x[w][mask[w] & np.isfinite(x[w])]
        if v.size:
            out[w] = func(v)
    return out


def ancorar(ref: np.ndarray, vivo: np.ndarray, corpo: np.ndarray) -> np.ndarray:
    """O idle preso no mundo onde o primeiro episodio comecou: o clipe tocado no tempo, no lugar do comeco.

    Medido na SUITE_V1 (build-base-n1), o reference_lines do parado poe o clipe sob a pelve do corpo a cada passo - so a translacao
    no chao (a pelve do clipe fica a ~4-5 cm constantes da do corpo, mais o balanco do proprio clipe), com o rumo preso na vista -
    e isso tambem com --parado-fixo (o idle fixo da nota nao aparece nas linhas). O inverso: tirar do quadro do clipe o quanto a
    pelve do corpo andou no chao desde o primeiro passo. Se a pelve do clipe nao seguir a do corpo (o desvio dela contra a do
    corpo passar de 5 cm no p95), a referencia ja e fixa e volta como esta."""
    out = np.array(ref, float)
    for w in range(ref.shape[0]):
        ok = np.where(vivo[w] & np.isfinite(ref[w, :, 0, 0]))[0]
        if not ok.size:
            continue
        off = ref[w, ok, IDX["Pelvis"], :2].astype(float) - corpo[w, ok, IDX["Pelvis"], :2]
        if np.percentile(np.linalg.norm(off - np.median(off, 0), axis=-1), 95) > 0.05:
            continue
        anda = corpo[w, :, IDX["Pelvis"], :2].astype(float) - corpo[w, ok[0], IDX["Pelvis"], :2]
        out[w, :, :, :2] -= anda[:, None, :]
    return out
