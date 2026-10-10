#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"
#include "Engine/Modeling/Parts/ProfileCurve.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

// Uma secao horizontal de quatro quadrantes: a superelipse
// |x/a|^p + |y/b|^q = 1 em volta de (centerX, 0), com a = front para +x e
// back para -x, b = side; q = exponent (frente e costas mais retas quanto
// maior) e p = sideExponent (os lados mais fechados quanto menor; 0 = o
// mesmo q). O tronco, o pescoco e o campo da camisa usam a mesma. A metade
// da frente pode ter os seus expoentes (o rosto em V e a nuca redonda: as
// duas metades se encontram no lado com a mesma tangente).
struct SuperSection {
    float centerX = 0.0f;
    float front = 0.1f;
    float back = 0.1f;
    float side = 0.1f;
    float exponent = 2.0f;
    float z = 0.0f;
    float sideExponent = 0.0f;
    float frontExponent = 0.0f;
    float frontSideExponent = 0.0f;

    // O raio na direcao angle (0 = +x, pi/2 = +y).
    [[nodiscard]] float radius(float angle) const {
        const float c = std::cos(angle), s = std::sin(angle);
        const float along = c >= 0.0f ? front : back;
        const bool frontHalf = c >= 0.0f && frontExponent > 0.0f;
        const float q = frontHalf ? frontExponent : exponent;
        const float p = frontHalf && frontSideExponent > 0.0f ? frontSideExponent
            : !frontHalf && sideExponent > 0.0f               ? sideExponent
                                                               : q;
        const float ax = std::pow(std::abs(c) / along, p), ay = std::pow(std::abs(s) / side, q);
        float r = std::pow(std::max(ax + ay, 1e-20f), -2.0f / (p + q));
        if (p == q) return std::pow(std::max(ax + ay, 1e-20f), -1.0f / q);
        // Newton em ax r^p + ay r^q = 1 (convexa: converge em poucos passos).
        for (int i = 0; i < 12; ++i) {
            const float rp = std::pow(r, p), rq = std::pow(r, q);
            const float f = ax * rp + ay * rq - 1.0f;
            const float d = (p * ax * rp + q * ay * rq) / std::max(r, 1e-9f);
            if (d <= 0.0f) break;
            const float next = std::max(r * 0.25f, r - f / d);
            if (std::abs(next - r) < 1e-7f) {
                r = next;
                break;
            }
            r = next;
        }
        return r;
    }
    [[nodiscard]] Vec3 point(float angle) const {
        const float r = radius(angle);
        return { centerX + r * std::cos(angle), r * std::sin(angle), z };
    }
};

// As curvas de uma pilha de secoes (por altura do chao); sideExponent
// vazio = o mesmo expoente nos dois termos; os da frente vazios = os mesmos.
struct SectionCurves {
    const ProfileCurve& centerX;
    const ProfileCurve& front;
    const ProfileCurve& back;
    const ProfileCurve& side;
    const ProfileCurve& exponent;
    const ProfileCurve* sideExponent = nullptr;
    const ProfileCurve* frontExponent = nullptr;
    const ProfileCurve* frontSideExponent = nullptr;

    [[nodiscard]] SuperSection at(float height, float ground) const {
        SuperSection section { centerX(height), std::max(1e-3f, front(height)), std::max(1e-3f, back(height)),
            std::max(1e-3f, side(height)), std::max(1.0f, exponent(height)), ground + height };
        if (sideExponent != nullptr && !sideExponent->empty()) section.sideExponent = std::max(1.0f, (*sideExponent)(height));
        if (frontExponent != nullptr && !frontExponent->empty())
            section.frontExponent = std::max(1.0f, (*frontExponent)(height));
        if (frontSideExponent != nullptr && !frontSideExponent->empty())
            section.frontSideExponent = std::max(1.0f, (*frontSideExponent)(height));
        return section;
    }
};

} // namespace MatterEngine::Modeling
