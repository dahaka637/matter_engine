#pragma once

#include "Engine/Math/Vec3.hpp"

namespace MatterEngine {

// O espaco entre os pes (plano v3, pedido do usuario de 10/10 noite: "ao
// girar ou correr girando bruscamente ele acaba cruzando as pernas,
// atravessando elas"). Duas regras, no rumo do corpo:
//  - Cada pe do seu lado: lado a lado (o que anda pouco a frente ou atras do
//    outro) o pe fica pelo menos passGap do outro, do lado dele; mais longe
//    na frente ou atras, pelo menos lineGap - nunca do lado errado.
//  - O pe que anda nunca entra no disco de clearRadius em volta do outro: ele
//    contorna (o pe que comeca do lado errado, depois de um giro, da a volta
//    por fora em vez de atravessar).
struct FootSpacingSettings3D {
    float passGap = 0.15f;
    float lineGap = 0.04f;
    // Lado a lado ate passZone (m, a frente ou atras); a regra de passGap some
    // ate fadeZone.
    float passZone = 0.16f;
    float fadeZone = 0.36f;
    // As solas (do calcanhar a ponta) nunca mais perto que isto (m); e a sola
    // que passa no alto, da canela do outro (a linha do tornozelo ao quadril,
    // na altura dela).
    float clearRadius = 0.12f;
    float legClearRadius = 0.15f;
    // O pe que ja esta do lado errado (o corpo girou com os dois apoiados):
    // ate este tanto (m) ele e empurrado de volta de lado; mais que isso, so o
    // disco (ele contorna). O pe do lado certo nunca passa para o errado.
    float wrongSideSlack = 0.06f;
    // A correcao entra e sai nesta mola (s), no maximo nesta velocidade (m/s).
    float pushSeconds = 0.02f;
    float maximumPushSpeed = 6.0f;
};

// A sola de um pe no mundo: do calcanhar a ponta; e a perna dele (o
// tornozelo e o quadril), a canela que o outro pe contorna.
struct FootSole3D {
    Vec3 heel;
    Vec3 toe;
    Vec3 ankle;
    Vec3 hip;
    bool leg = false;
    [[nodiscard]] Vec3 center() const { return (heel + toe) * 0.5f; }
};

// O quanto o pe `moving` (lado: 0 esquerdo, 1 direito) precisa ir, no plano,
// para respeitar as regras contra o pe `fixed`, com o corpo olhando para
// `forward` (no plano, unitario). crossed: o pe ja estava do lado errado
// (so ele pode ficar la, contornando).
[[nodiscard]] Vec3 footSpacingPush3D(const FootSole3D& moving, const FootSole3D& fixed, int side, Vec3 forward,
    bool crossed, const FootSpacingSettings3D& settings);
// O pe esta do lado errado do outro (alem da folga; pelo meio das solas).
[[nodiscard]] bool footSpacingCrossed3D(const FootSole3D& moving, const FootSole3D& fixed, int side, Vec3 forward,
    const FootSpacingSettings3D& settings);

// Mais um passo da correcao aplicada (a mola com a velocidade maxima).
[[nodiscard]] Vec3 footSpacingStep3D(Vec3 applied, Vec3 wanted, float deltaTime, const FootSpacingSettings3D& settings);

} // namespace MatterEngine
