"""O espelho esquerda <-> direita das entradas e das acoes da rl_final (05/11, o vies para a direita: a rede aprendeu a
inclinar e derivar para a direita, escondida pela mao; a perda de simetria de Yu, Turk e Liu 2018 - a acao no estado
espelhado tem de ser o espelho da acao).

O espelho e a reflexao no plano frente-cima do referencial da pelve (y -> -y), trocando os elos e os canais de lado:
- posicao e velocidade: y troca de sinal; a velocidade angular (vetor axial): x e z trocam;
- o quaternio de cada elo (no referencial da pelve): (x, y, z, w) -> (-x, y, -z, w) - a constante do elo e a identidade
  nos 18 (conferido no clipe puro, erro 0,001);
- comando: os senos do alvo e do rumo trocam; o chao: os .y trocam; os raios: k*45 -> (8-k)*45; o equilibrio: os .lado
  trocam, menos o giro (axial: o .frente troca, o .lado nao); o passo: o .lado troca, no_ar_esq <-> no_ar_dir;
- a fase do clipe: deslocada pelo mundo (meio ciclo nos clipes que servem aos dois lados - frente, costas, diagonais,
  sprint -; zero nos laterais, em que o clipe da direita e o espelho exato do da esquerda no mesmo instante);
- a acao (normalizada, as faixas iguais dos dois lados): nas juntas esfericas o twist e o swing2 trocam de sinal e o
  swing1 nao; as dobradicas (joelho, cotovelo, tornozelo = os .twist do Shin, do Forearm e do Foot) nao; o swing1 do pe
  troca; a mao: twist troca, swing1 nao; o canal de passo: pouso_esquerda troca. (Os eixos das juntas do perfil preveem
  os mesmos sinais que os alvos do clipe meio ciclo depois - os dois conferidos em 05/11.)

Uso: e = Espelho(obs_names, act_names); e.obs(x, fase) e e.acao(a) (numpy ou torch).
Autoteste: python espelho.py --teste COLETA.npz (a coleta do clipe puro: build-profile/tmp/espelho/clipe_puro.npz).
"""
from __future__ import annotations

import argparse
import math

import numpy as np

# (Os canais trocados de lado fora dos elos.)
_PAIRS = (("no_ar_esq", "no_ar_dir"), ("pe_esq", "pe_dir"))
_FLIP_OBS = {"comando.alvo_sen", "comando.rumo_sen", "amb.inclinacao.lado", "eq.giro.frente"}
_KEEP_OBS = {"eq.giro.lado"}
_FLIP_ACT = {"passo.pouso_esquerda"}
_HINGES = ("Shin.twist", "Forearm.twist", "Foot.twist")


def swap(name: str) -> str:
    """O nome do canal do outro lado (ele mesmo no centro)."""
    for a, b in (("Left", "Right"), ("Right", "Left")):
        if a in name:
            return name.replace(a, b)
    for a, b in _PAIRS:
        if a in name:
            return name.replace(a, b)
        if b in name:
            return name.replace(b, a)
    if name.startswith("amb.raio."):
        head, k = name.rsplit(".", 1)
        return f"{head}.{(360 - int(k)) % 360}"
    return name


def obs_sign(name: str) -> float:
    if name in _KEEP_OBS:
        return 1.0
    if name in _FLIP_OBS:
        return -1.0
    part = name.split(".")
    if len(part) == 3 and part[1] in ("pos", "vel") and part[2] == "y":
        return -1.0
    if len(part) == 3 and part[1] in ("ang", "rot") and part[2] in ("x", "z"):
        return -1.0
    if name.startswith("chao.") and name.endswith(".y"):
        return -1.0
    if name.endswith(".lado"):
        return -1.0
    return 1.0


def act_sign(name: str) -> float:
    # (06/11 17h40: a velocidade-alvo de um eixo espelha como o eixo.)
    if name.startswith("vel."):
        return act_sign(name[4:])
    if name in _FLIP_ACT:
        return -1.0
    if name.endswith(_HINGES):
        return 1.0
    if name.endswith("Foot.swing1"):
        return -1.0
    axis = name.rsplit(".", 1)[-1]
    if axis in ("twist", "swing2"):
        return -1.0
    return 1.0


class Espelho:
    """O espelho das entradas e das acoes de um lote (os nomes do ambiente)."""

    def __init__(self, obs_names, act_names):
        self.obs_names = [str(n) for n in obs_names]
        self.act_names = [str(n) for n in act_names]
        oi = {n: i for i, n in enumerate(self.obs_names)}
        ai = {n: i for i, n in enumerate(self.act_names)}
        # (A forma de juntar: saida[j] = sinal[j] * entrada[fonte[j]]; a troca e uma involucao, a fonte de j e o par de j.)
        self.obs_src = np.array([oi[swap(n)] if swap(n) in oi else i for i, n in enumerate(self.obs_names)])
        self.obs_sign = np.array([obs_sign(self.obs_names[s]) for s in self.obs_src], np.float32)
        self.act_src = np.array([ai[swap(n)] if swap(n) in ai else i for i, n in enumerate(self.act_names)])
        self.act_sign = np.array([act_sign(self.act_names[s]) for s in self.act_src], np.float32)
        assert (self.obs_src[self.obs_src] == np.arange(len(self.obs_src))).all(), "o espelho das entradas nao e involucao"
        assert (self.act_src[self.act_src] == np.arange(len(self.act_src))).all(), "o espelho das acoes nao e involucao"
        self.fase = (oi["fase.sen"], oi["fase.cos"]) if "fase.sen" in oi else None
        self._torch = {}

    def _t(self, x):
        import torch
        key = (x.device, x.dtype)
        if key not in self._torch:
            self._torch[key] = tuple(torch.as_tensor(v, device=x.device, dtype=dt) for v, dt in (
                (self.obs_src, torch.long), (self.obs_sign, x.dtype), (self.act_src, torch.long), (self.act_sign, x.dtype)))
        return self._torch[key]

    def obs(self, x, shift=None):
        """As entradas espelhadas [..., n]; shift (rad, [...] ou escalar): o deslocamento da fase do clipe."""
        if isinstance(x, np.ndarray):
            out = x[..., self.obs_src] * self.obs_sign
        else:
            src, sign, _, _ = self._t(x)
            out = x.index_select(-1, src) * sign
        if self.fase is not None and shift is not None:
            i, j = self.fase
            s, c = out[..., i], out[..., j]
            if isinstance(x, np.ndarray):
                cs, sn = np.cos(shift), np.sin(shift)
                out[..., i], out[..., j] = s * cs + c * sn, c * cs - s * sn
            else:
                import torch
                shift = torch.as_tensor(shift, device=x.device, dtype=x.dtype)
                cs, sn = torch.cos(shift), torch.sin(shift)
                out = out.clone()
                out[..., i], out[..., j] = s * cs + c * sn, c * cs - s * sn
        return out

    def acao(self, a):
        """A acao espelhada (normalizada) [..., m]."""
        if isinstance(a, np.ndarray):
            return a[..., self.act_src] * self.act_sign
        _, _, src, sign = self._t(a)
        return a.index_select(-1, src) * sign


def autoteste(path: str) -> None:
    """Na coleta do clipe puro: (1) o trote para a frente meio ciclo depois e o espelho do de agora (as acoes do clipe e as
    entradas); (2) os laterais 2x6 no mesmo instante e as diagonais 1x7 / 3x5 meio ciclo depois."""
    z = np.load(path)
    e = Espelho(z["obs_names"], z["act_names"])
    dt = float(z["dt"])
    t0 = int(round(1.5 / dt))
    on = e.obs_names

    def lag_of(alvo):
        l, r = alvo[:, e.act_names.index("LeftThigh.swing1")], alvo[:, e.act_names.index("RightThigh.swing1")]
        l, r = l - l.mean(), r - r.mean()
        # (O primeiro meio ciclo: ate 0,8 s - o mais lento, o trote, tem 0,37 s; um multiplo depois acumula a deriva.)
        return max(range(1, min(len(l) // 2, int(0.8 / dt))),
                   key=lambda k: float(np.dot(l[:-k], r[k:]) / (np.linalg.norm(l[:-k]) * np.linalg.norm(r[k:]) + 1e-9)))

    def rel(m, b):
        return float((np.abs(m - b).mean(0) / np.maximum(np.abs(b).mean(0), 1e-3)).mean())

    # (1) o clipe que serve aos dois lados: t contra t + meio ciclo
    for key, tol_a, tol_o in (("lev_trote_0", 0.002, 0.08), ("lev_trote_4", 0.01, 0.15), ("lev_corrida_4", 0.01, 0.15)):
        obs, alvo = z[key + "__obs"][0][t0:], z[key + "__alvo"][0][t0:]
        lag = lag_of(alvo)
        ea = float(np.abs(e.acao(alvo[:-lag]) - alvo[lag:]).mean())
        eo = rel(e.obs(obs[:-lag], math.pi), obs[lag:])
        print(f"{key}: meio ciclo {lag * dt:.3f} s; erro da acao do clipe {ea:.5f} rad; erro relativo das entradas {eo:.3f}")
        assert ea < tol_a and eo < tol_o, key
    # (2) os pares de lado
    for a_key, b_key, half, tol_a, tol_o in (("lev_trote_2", "lev_trote_6", False, 1e-4, 0.12),
                                             ("lev_corrida_2", "lev_corrida_6", False, 1e-4, 0.12),
                                             ("lev_trote_1", "lev_trote_7", True, 0.002, 0.08),
                                             ("lev_trote_3", "lev_trote_5", True, 0.02, 0.15)):
        oa, ob = z[a_key + "__obs"][0][t0:], z[b_key + "__obs"][0][t0:]
        aa, ab = z[a_key + "__alvo"][0][t0:], z[b_key + "__alvo"][0][t0:]
        n = min(len(oa), len(ob)) - 40
        d = min(range(40), key=lambda k: float(np.abs(e.acao(aa[:n]) - ab[k:k + n]).mean()))
        ea = float(np.abs(e.acao(aa[:n]) - ab[d:d + n]).mean())
        eo = rel(e.obs(oa[:n], math.pi if half else 0.0), ob[d:d + n])
        print(f"{a_key} x {b_key}: deslocamento {d * dt:.3f} s; erro da acao {ea:.5f} rad; erro relativo das entradas {eo:.3f}")
        assert ea < tol_a and eo < tol_o, (a_key, b_key)
        assert (d == 0) != half, "os laterais no mesmo instante, as diagonais meio ciclo depois"
    # (3) as regras: a involucao e o espelho do espelho e a identidade
    x = np.random.default_rng(0).normal(size=(5, len(on))).astype(np.float32)
    assert np.allclose(e.obs(e.obs(x, 0.7), -0.7), x, atol=1e-5)
    a = np.random.default_rng(1).normal(size=(5, len(e.act_names))).astype(np.float32)
    assert np.allclose(e.acao(e.acao(a)), a)
    import torch
    assert np.allclose(e.obs(torch.as_tensor(x), torch.full((5,), 0.7)).numpy(), e.obs(x, 0.7), atol=1e-5)
    assert np.allclose(e.acao(torch.as_tensor(a)).numpy(), e.acao(a))
    print("espelho ok")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--teste", required=True, help="a coleta do clipe puro (.npz)")
    autoteste(ap.parse_args().teste)
