"""As medidas e a estatistica conferidas em dados sinteticos, com a resposta conhecida (sem simulacao)."""
import unittest
from pathlib import Path

import numpy as np

from suite import estatistica as E
from suite.medidas import base, marcha, parado, paradas
from suite.medidas.base import IDX, LINKS

DT = 1.0 / 30.0
PELVE_Q = np.array([-np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])  # o z local da pelve para +y (a esquerda): rumo do quadril +x
PEITO_Q = np.array([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])   # o x local do peito alto para +y


def giro_z(fr, ang):
    """Gira quadros [..., 18, 7] em volta de z (posicoes e orientacoes)."""
    c, s = np.cos(ang), np.sin(ang)
    out = fr.copy()
    out[..., 0], out[..., 1] = c * fr[..., 0] - s * fr[..., 1], s * fr[..., 0] + c * fr[..., 1]
    out[..., 3:7] = base.qmul(np.broadcast_to(base.qz(ang), fr[..., 3:7].shape), fr[..., 3:7])
    return out


def corpo(coluna=0.0, cabeca=0.0):
    """Um quadro [18, 7]: a pelve a 1 m, o pescoco a 0,6 m dela inclinado `coluna` graus para a frente (+x), a cabeca `cabeca`."""
    fr = np.zeros((18, 7))
    fr[:, 6] = 1.0
    fr[IDX["Pelvis"], :3] = (0, 0, 1)
    a, b = np.radians(coluna), np.radians(cabeca)
    fr[IDX["Neck"], :3] = fr[IDX["Pelvis"], :3] + 0.6 * np.array([np.sin(a), 0, np.cos(a)])
    fr[IDX["Chest"], :3] = fr[IDX["Pelvis"], :3] + 0.3 * np.array([np.sin(a), 0, np.cos(a)])
    fr[IDX["Head"], :3] = fr[IDX["Neck"], :3] + 0.15 * np.array([np.sin(b), 0, np.cos(b)])
    fr[IDX["Pelvis"], 3:] = PELVE_Q
    fr[IDX["UpperChest"], 3:] = PEITO_Q
    fr[IDX["LeftFoot"], :3] = (0, 0.12, 0.05)
    fr[IDX["RightFoot"], :3] = (0, -0.12, 0.05)
    return fr


def cap_falso(body, ref, tipo, segundos, contato=None, info=None, rumo=0, ordem=0.0, faixa=1, fins=None):
    W, T = body.shape[:2]
    if contato is None:
        contato = np.zeros((W, T, 18), bool)
        contato[..., IDX["LeftFoot"]] = contato[..., IDX["RightFoot"]] = True
    episodio = np.zeros((W, T), np.int16)
    linhas = []
    for w, t in (fins or {}).items():
        episodio[w, int(round(t / DT)) - 1:] = 1
        linhas.append([w, 0, 0, 0, t] + [0] * 3)
    cols = ["mundo", "comeco", "lado", "cabeca_comeco", "segundos", "x", "y", "passo_fim"]
    z = {"corpo": body.astype(np.float32), "referencia": ref.astype(np.float32), "contato": contato, "episodio": episodio,
         "info": np.zeros((W, T, 5), np.float32) if info is None else info,
         "episodios": np.array(linhas, np.float32).reshape(-1, len(cols))}
    meta = {"dt": DT, "cenario": {"tipo": tipo, "segundos": segundos, "rumo": rumo, "ordem": ordem, "faixa": faixa},
            "colunas_episodios": cols, "ragdoll": [{"id": n} for n in LINKS], "geo": [{"massa": 1.0}] * 18}
    return base.Cap(Path("falso.npz"), meta, z)


class Postura(unittest.TestCase):
    def test_angulos_conhecidos_e_invariancia_ao_rumo(self):
        for psi in (0.0, 1.0, -2.5):
            p = base.postura(giro_z(corpo(coluna=12.0, cabeca=-7.0), psi))
            self.assertAlmostEqual(float(p["coluna_frente"]), 12.0, places=6)
            self.assertAlmostEqual(float(p["coluna_lado"]), 0.0, places=6)
            self.assertAlmostEqual(float(p["cabeca"]), -7.0, places=6)
            self.assertAlmostEqual(float(p["arco"]), 0.0, places=6)
            self.assertAlmostEqual(float(p["pelve"]), 100.0, places=6)


class Parado(unittest.TestCase):
    def setUp(self):
        self.T, self.W = int(10.0 / DT), 3
        self.ref = np.broadcast_to(corpo(), (self.W, self.T, 18, 7)).copy()

    def test_estatico_e_perfeito(self):
        v = parado.valores(cap_falso(self.ref.copy(), self.ref, "parado", 10.0))
        np.testing.assert_allclose(v["tolerancia"], 1.0)
        np.testing.assert_allclose(v["erro_pose"], 0.0, atol=0.01)  # (float32: ~0,003 grau)
        np.testing.assert_allclose(v["deslize"], 0.0)
        np.testing.assert_allclose(v["passos"], 0.0)
        np.testing.assert_allclose(v["minimo"], 1.0)

    def test_pe_deslizando_e_referencia_que_anda_junto(self):
        body = self.ref.copy()
        # o pe esquerdo do mundo 1 desliza 1 cm por passo, apoiado; a referencia anda junto com a pelve (como sem --parado-fixo)
        body[1, :, IDX["LeftFoot"], 0] += 0.01 * np.arange(self.T)
        body[2, :, :, 0] += 0.002 * np.arange(self.T)[:, None]
        ref = self.ref.copy()
        ref[2, :, :, 0] += 0.002 * np.arange(self.T)[:, None]
        v = parado.valores(cap_falso(body, ref, "parado", 10.0))
        janela = np.sum((np.arange(self.T) + 1) * DT >= 1.5) - 1
        self.assertAlmostEqual(v["deslize"][1], 100 * 0.01 * janela / 2, delta=1.0)  # a media dos dois pes
        self.assertEqual(v["minimo"][1], 0.0)
        # a referencia presa no comeco: a pelve do mundo 2 se afasta 0,2 cm por passo dela
        self.assertAlmostEqual(v["deriva_pelve"][2], 100 * 0.002 * (self.T - 2), delta=1.0)
        fixo = base.ancorar(ref, np.ones((self.W, self.T), bool), body)
        np.testing.assert_allclose(fixo[2, :, :, :2], ref[2, :1, :, :2].repeat(self.T, 0), atol=1e-9)

    def test_referencia_ja_fixa_nao_muda(self):
        body = self.ref.copy()
        body[0, :, :, 0] += 0.003 * np.arange(self.T)[:, None]  # o corpo anda 90 cm e a referencia fica
        fixo = base.ancorar(self.ref, np.ones((self.W, self.T), bool), body)
        np.testing.assert_allclose(fixo, self.ref)

    def test_passos_e_queda(self):
        cont = np.ones((self.W, self.T, 18), bool)
        cont[0, 100:110, IDX["RightFoot"]] = False  # um passo
        v = parado.valores(cap_falso(self.ref.copy(), self.ref, "parado", 10.0, contato=cont, fins={2: 5.0}))
        self.assertEqual(v["passos"][0], 1.0)
        self.assertEqual(list(v["quedas"]), [0.0, 0.0, 1.0])


class Paradas(unittest.TestCase):
    def test_tempo_ate_parar(self):
        T = int(8.0 / DT)
        t = (np.arange(T) + 1) * DT
        x = np.minimum(t, 5.0) * 3.0  # 3 m/s ate os 5 s, parado depois
        body = np.broadcast_to(corpo(), (2, T, 18, 7)).copy()
        body[..., 0] += x[None, :, None]
        v = paradas.valores(cap_falso(body, body.copy(), "parada", 8.0, ordem=4.0))
        np.testing.assert_allclose(v["tempo_parar"], 1.0, atol=0.15)
        np.testing.assert_allclose(v["metros_parar"], 3.0, atol=0.5)
        np.testing.assert_allclose(v["assentou"], 1.0)
        np.testing.assert_allclose(v["base_fim"], 0.0, atol=1e-6)


class Marcha(unittest.TestCase):
    def test_velocidade_desvio_e_peito(self):
        T, rumo_ex = int(7.0 / DT), 2  # o rumo 2: andar para a esquerda com o peito na vista
        t = (np.arange(T) + 1) * DT
        r = 0.7
        body = np.broadcast_to(giro_z(corpo(), r - rumo_ex * np.pi / 4), (2, T, 18, 7)).copy()
        body[..., 0] += (4.0 * t * np.cos(r))[None, :, None]
        body[..., 1] += (4.0 * t * np.sin(r))[None, :, None]
        info = np.zeros((2, T, 5), np.float32)
        info[..., 3], info[..., 4] = 5.0, r
        ref = body.copy()
        ref[..., 3:7] = giro_z(ref, np.radians(10.0))[..., 3:7]  # o peito do clipe 10 graus a esquerda do corpo
        v = marcha.valores(cap_falso(body, ref, "reto", 7.0, info=info, rumo=rumo_ex))
        np.testing.assert_allclose(v["razao"], 0.8, atol=1e-4)
        np.testing.assert_allclose(v["desvio"], 0.0, atol=1e-3)
        np.testing.assert_allclose(v["peito"], 10.0, atol=1e-3)


class Estatistica(unittest.TestCase):
    def test_wilson_regra_do_tres(self):
        lo, hi = E.wilson(0, 16)
        self.assertEqual(lo, 0.0)
        self.assertAlmostEqual(hi, 0.194, places=2)

    def test_bootstrap_e_comparacao_pareada(self):
        rng = np.random.default_rng(1)
        a = base.Medida("x", "T", "x", "graus", "menor", {"c1": rng.normal(5, 1, 16), "c2": rng.normal(3, 1, 16)}, margem=0.5)
        b = base.Medida("x", "T", "x", "graus", "menor", {k: v + 2.0 for k, v in a.por_cenario.items()}, margem=0.5)
        r = E.resumo(a)
        self.assertLess(r["ic95"][0], r["valor"])
        self.assertGreater(r["ic95"][1], r["valor"])
        c = E.comparar(a, b, 0.5)
        self.assertAlmostEqual(c["delta_pior"], 2.0, places=9)
        self.assertAlmostEqual(c["ic95"][0], 2.0, places=9)
        self.assertEqual(c["marca"], "piorou")
        self.assertEqual(E.comparar(b, a, 0.5)["marca"], "melhorou")
        self.assertEqual(E.comparar(a, a, 0.5)["marca"], "igual")

    def test_zero_nao_cancela_os_lados(self):
        m = base.Medida("x", "T", "x", "graus", "zero", {"esq": np.full(4, 5.0), "dir": np.full(4, -5.0)}, piso=4.0)
        r = E.resumo(m)
        self.assertAlmostEqual(r["valor"], 5.0)
        self.assertAlmostEqual(r["valor_com_sinal"], 0.0)
        self.assertFalse(r["passa_piso"])
        self.assertEqual(r["cenarios_falham"], ["dir", "esq"])
        m.por_cenario["meio"] = np.zeros(4)  # o grupo: (5 + 5 + 0) / 3 = 3,3 < 4, os dois cenarios ainda listados
        r = E.resumo(m)
        self.assertTrue(r["passa_piso"])
        self.assertEqual(r["cenarios_falham"], ["dir", "esq"])

    def test_marcas(self):
        self.assertEqual(E.marca(0.1, 0.9, 1.0), "igual")
        self.assertEqual(E.marca(0.5, 1.5, 1.0), "incerto+")
        self.assertEqual(E.marca(-1.5, -0.5, 1.0), "incerto-")
        self.assertEqual(E.marca(1.1, 2.0, 1.0), "piorou")
        self.assertEqual(E.marca(-2.0, -1.1, 1.0), "melhorou")


if __name__ == "__main__":
    unittest.main()
