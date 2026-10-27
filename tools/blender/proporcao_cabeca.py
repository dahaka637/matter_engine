"""A boca na largura humana e a cabeca um pouco mais larga (23/10/2026, pedidos do usuario: a "estranheza" da cabeca;
depois "deixe a cabeca um pouco mais larga, pouca coisa").

A boca tinha 66,5 mm de canto a canto (o adulto ~54), mais larga que as pupilas (65 mm). Os dois cantos andam para o
meio pela aresta ate a fresta (ficam na superficie) ate a largura alvo. Muda so as posicoes; os pesos e o mapa vao
junto (a pintura do rosto acompanha as faces).
A cabeca alarga WIDEN so na largura (y), inteira acima de WIDEN_FADE e sumindo ate o queixo: o pescoco fica como esta.
(O pescoco mais estreito dos lados - 13,8 para 12 cm, ANSUR II - foi testado e desfeito: o usuario preferiu o de antes.)
"""
import numpy as np

MOUTH_WIDTH = 0.0546            # m de canto a canto
MOUTH_CORNER = (0.03326, 0.63481)   # |y|, z do canto (pintar_rosto.MOUTH, na troca)
MOUTH_SLIT = (0.00879, 0.63576)     # o vertice da fresta do mesmo lado
WIDEN = 0.04                    # a cabeca 4% mais larga (~6 mm no cranio)
WIDEN_FADE = (1.565, 1.610)     # m do chao: do queixo (nada) ate abaixo das macas (inteiro)


def _ramp(x, a, b):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def shape_head(pos, tris, tri_part, names, joint_positions, ground):
    pos = pos.copy()
    skin = np.unique(tris[np.isin(tri_part, [names.index("corpo"), names.index("cabeca")])])
    front = skin[pos[skin, 0] > 0.06]

    def at(side_y, z):
        hit = front[(np.abs(np.abs(pos[front, 1]) - side_y) < 0.0005) & (np.abs(pos[front, 2] - z) < 0.0005)]
        if len(hit) != 2:
            raise RuntimeError(f"a boca da malha mudou: {len(hit)} vertices em ({side_y}, {z})")
        return hit[np.argsort(pos[hit, 1])]

    corners, slits = at(*MOUTH_CORNER), at(*MOUTH_SLIT)
    for c, s in zip(corners, slits):
        # (O alvo antes do alargamento da cabeca: a boca termina em MOUTH_WIDTH.)
        half = MOUTH_WIDTH / 2.0 / (1.0 + WIDEN * _ramp(pos[c, 2] - ground, *WIDEN_FADE))
        t = (abs(pos[c, 1]) - half) / (abs(pos[c, 1]) - abs(pos[s, 1]))
        pos[c] = pos[c] + t * (pos[s] - pos[c])
    # (Depois da boca: os cantos sao achados pelas coordenadas de antes.)
    before = 2.0 * np.abs(pos[skin, 1][pos[skin, 2] - ground > 1.70]).max()
    pos[skin, 1] *= 1.0 + WIDEN * _ramp(pos[skin, 2] - ground, *WIDEN_FADE)
    after = 2.0 * np.abs(pos[skin, 1][pos[skin, 2] - ground > 1.70]).max()
    return pos, (f"boca {1000 * abs(pos[corners[1], 1] - pos[corners[0], 1]):.1f} mm; cabeca acima de 1,70 m de "
                 f"{100 * before:.1f} a {100 * after:.1f} cm de largura")
