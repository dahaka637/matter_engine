#include "Engine/Gesture/GestureChoice3D.hpp"

#include "Engine/Gesture/GestureCatalog3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

GestureChoice3D chooseGesture3D(GestureKind3D kind, Vec3 ball, Vec3 direction, int preferredSide,
    const GestureReach3D& reach, int alternateSide, bool receive) {
    GestureChoice3D choice;
    if (ball.x > reach.front || ball.x < -reach.back || std::abs(ball.y) > reach.side || ball.z > reach.height
        || ball.z < -reach.below)
        return choice;
    choice.inReach = true;
    GestureSpec3D& spec = choice.spec;
    spec.kind = kind;
    const float lateral = std::abs(ball.y);
    const float band = receive ? std::max(reach.centerBand, reach.receiveBand) : reach.centerBand;
    const bool rear = kind == GestureKind3D::Shot && !receive && ball.x > reach.shotAhead && preferredSide >= 0
        && (preferredSide == 0 ? ball.y : -ball.y) >= -reach.shotAcross;
    // (Conduzindo, as pernas se alternam: a bola a frente, perto do meio.)
    const bool alternating = alternateSide >= 0 && lateral <= reach.alternateBand && ball.x > reach.depthBack;
    if (alternating)
        spec.side = alternateSide == 1 ? 1 : 0;
    else if (rear || lateral <= band)
        spec.side = preferredSide == 1 ? 1 : 0;
    else
        spec.side = ball.y > 0.0f ? 0 : 1;
    // (O alcance para o lado da perna escolhida: do outro lado, curto.)
    const float outward = std::max(0.0f, spec.side == 0 ? ball.y : -ball.y);
    spec.reach = outward < reach.reachMid ? 0 : outward < reach.reachFar ? 1 : 2;
    spec.depth = ball.x < reach.depthBack ? 0 : ball.x < reach.depthFront ? 1 : 2;
    // O chute no meio (28/10: a perna de tras na bola do meio - o balde curto
    // ficava 17 cm para fora dela): a bola mais perto do meio do corpo que do
    // balde curto vai ao balde do meio.
    if (kind == GestureKind3D::Shot && spec.reach == 0) {
        const float bucket = std::abs(gestureNominalBall3D(spec).y) * (reach.reachMid / GestureReach3D {}.reachMid);
        spec.inward = (spec.side == 0 ? ball.y : -ball.y) < 0.5f * bucket ? 1.0f : 0.0f;
    }
    // A bola alta: o gesto com o pe na altura do joelho, e atras vira o meio (o
    // calcanhar na altura do joelho fica feio).
    spec.high = ball.z > reach.highBall;
    if (spec.high && spec.depth == 0) spec.depth = 1;
    spec.receive = receive;
    // Para dentro: o rumo pedido volta contra a bola (ela vem para o corpo ou
    // atravessa a linha dele); senao, para fora.
    const Vec3 flat { direction.x, direction.y, 0.0f };
    const Vec3 out { ball.x, ball.y, 0.0f };
    if (flat.lengthSquared() > 1e-6f && out.lengthSquared() > 1e-6f) {
        const float cosine = dot(flat.normalized(), out.normalized());
        spec.move = cosine < reach.inwardCosine ? GestureMove3D::In : GestureMove3D::Out;
    }
    // (Com a bola no meio, quem decide o movimento e a perna: jogar para o
    // lado dela e para fora, cruzar o corpo e para dentro.)
    if ((alternating || lateral <= band) && flat.lengthSquared() > 1e-6f) {
        const float toward = spec.side == 0 ? flat.y : -flat.y;
        if (std::abs(toward) > 0.3f * flat.length())
            spec.move = toward > 0.0f ? GestureMove3D::Out : GestureMove3D::In;
    }
    return choice;
}

float gestureBallArrival3D(Vec3 offset, Vec3 velocity, float radius) {
    offset.z = velocity.z = 0.0f;
    const float c = dot(offset, offset) - radius * radius;
    if (c <= 0.0f) return 0.0f;
    const float vv = dot(velocity, velocity), ov = dot(offset, velocity);
    if (vv < 1e-6f || ov >= 0.0f) return -1.0f;
    const float disc = ov * ov - vv * c;
    return disc >= 0.0f ? (-ov - std::sqrt(disc)) / vv : -ov / vv;
}

GestureReach3D gestureReachForBody3D(const GestureReach3D& base, float scale, float chestGrowth) {
    if (scale == 1.0f && chestGrowth == 0.0f) return base;
    GestureReach3D reach = base;
    for (float* field : { &reach.front, &reach.back, &reach.side, &reach.height, &reach.highBall, &reach.below, &reach.reachMid,
             &reach.reachFar, &reach.depthBack, &reach.depthFront, &reach.centerBand, &reach.receiveBand, &reach.shotAhead,
             &reach.shotAcross, &reach.alternateBand })
        *field *= scale;
    // O tronco do corpo de hoje: a meia profundidade e a meia largura (m).
    constexpr float HalfDepth = 0.12f, HalfWidth = 0.17f;
    reach.bodyFront = base.bodyFront * scale + HalfDepth * scale * chestGrowth;
    reach.bodyBack = base.bodyBack * scale + HalfDepth * scale * chestGrowth;
    reach.bodySide = base.bodySide * scale + HalfWidth * scale * chestGrowth;
    reach.chestBottom = base.chestBottom * scale;
    reach.chestTop = base.chestTop * scale;
    reach.headTop = reach.chestTop + (base.headTop - base.chestTop);
    return reach;
}

} // namespace MatterEngine
