#pragma once

#include "Engine/Gesture/GestureTypes3D.hpp"
#include "Engine/Math/Vec3.hpp"

namespace MatterEngine {

// O alcance (a area em que o pedido vale) e as divisas dos baldes, no
// referencial do corpo: a pelve no chao, x para a frente (o rumo do corpo), y
// para a esquerda, z a altura do centro da bola acima do chao (m).
struct GestureReach3D {
    float front = 0.85f;
    float back = 0.55f;
    float side = 0.80f;
    // O teto (o centro da bola; 24/10, pedido do usuario: quase na linha da
    // cintura) e, acima de highBall, o gesto alto (de pouco abaixo do joelho
    // para cima, o pe na altura do joelho).
    float height = 0.95f;
    float highBall = 0.38f;
    // O fundo: a area desce isto abaixo da sola (24/10, pedido do usuario: a
    // base presa aos pes do corpo, um pouco abaixo deles, para a bola no ar).
    float below = 0.15f;
    // Alcance lateral (|y| da bola): curto ate reachMid, medio ate reachFar,
    // longe dali ate side.
    float reachMid = 0.26f;
    float reachFar = 0.46f;
    // Frente-tras (x da bola): atras abaixo de depthBack, a frente acima de
    // depthFront.
    float depthBack = -0.05f;
    float depthFront = 0.30f;
    // Bola no meio (|y| ate isto): qualquer perna serve - vai a preferida. Na
    // recepcao o meio e mais largo (parado, a preferida e o pe da frente).
    float centerBand = 0.06f;
    float receiveBand = 0.16f;
    // Conduzindo (toques em sequencia com a bola a frente): ate isto do meio,
    // as pernas se alternam.
    float alternateBand = 0.22f;
    // Puxar para dentro quando o rumo pedido volta contra a bola (o cosseno
    // entre o rumo e a direcao do corpo ate ela abaixo disto).
    float inwardCosine = -0.2f;
    // O peito e a cabeca (24/10, pedido do usuario), fora do alcance dos pes:
    // a bola perto do corpo (x ate bodyFront a frente e bodyBack atras, |y| ate
    // bodySide) e a altura do centro dela acima da pelve - o peito de
    // chestBottom a chestTop, a cabeca dali a headTop (m; sobem com o pulo).
    float bodyFront = 0.60f;
    float bodyBack = 0.25f;
    float bodySide = 0.45f;
    float chestBottom = -0.05f;
    float chestTop = 0.55f;
    float headTop = 1.15f;
    // A caixa da cabeca no plano (26/10, o corpo variavel: a dela, para so a
    // posicao acompanhar a altura - o alcance da cabeca nao cresce).
    float headFront = 0.60f;
    float headBack = 0.25f;
    float headSide = 0.45f;
};

// O alcance num corpo (26/10, pedido do usuario): os pes na mesma proporcao
// da altura (scale = altura/1,80); o peito acompanha o corpo (as alturas x
// scale; no plano, x scale mais o quanto o tronco engrossou, chestGrowth = a
// grossura do peito - 1); a cabeca so sobe com o pescoco, do mesmo tamanho.
// scale 1 e chestGrowth 0 devolvem base.
[[nodiscard]] GestureReach3D gestureReachForBody3D(const GestureReach3D& base, float scale, float chestGrowth);

struct GestureChoice3D {
    bool inReach = false;
    GestureSpec3D spec;
};

// A escolha no comando: ball e a bola e direction o rumo pedido para ela (no
// plano), os dois no referencial do corpo; preferredSide e a perna livre (a
// que esta no ar na passada; -1 nenhuma) para a bola no meio; alternateSide,
// a perna da vez num toque em sequencia (a outra do ultimo; -1 nenhuma), que
// vale com a bola a frente ate alternateBand do meio; receive, a recepcao
// (a bola vindo para ele). Fora do alcance, inReach falso. A forca, o tempo e
// a semente ficam com quem chama.
[[nodiscard]] GestureChoice3D chooseGesture3D(GestureKind3D kind, Vec3 ball, Vec3 direction, int preferredSide,
    const GestureReach3D& reach, int alternateSide = -1, bool receive = false);

} // namespace MatterEngine
