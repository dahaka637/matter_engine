#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"

#include <array>

// O pescoco mais curto sem mudar a altura: a cabeca fica onde esta e o corpo
// sobe por baixo dela. Abaixo de "from" (a base do pescoco) o corpo estica
// por igual ate subir "amount" ali; entre "from" e a linha de cima do pescoco
// (sob a mandibula, inclinada: mais alta atras, na nuca) o pescoco encolhe;
// acima dela (o rosto e o cranio) nada muda. Alturas do chao; a linha de cima
// e uma reta no plano (x, altura) pelos dois pontos dados. O esqueleto medido
// (as juntas no corpo base) passa pela mesma conta.
namespace MatterEngine::Modeling {

struct NeckShortening {
    float amount = 0.0f;
    float from = 1.46f;
    // A linha de cima: [x, altura] na frente (sob o queixo) e atras (a nuca).
    std::array<float, 2> topFront { 0.034f, 1.564f };
    std::array<float, 2> topBack { -0.10f, 1.605f };

    // A altura da linha de cima no x dado.
    [[nodiscard]] float top(float x) const;
    // Quanto um ponto (x, altura do chao) sobe.
    [[nodiscard]] float lift(float x, float height) const;
    [[nodiscard]] float liftedHeight(float x, float height) const { return height + lift(x, height); }
    // Um ponto do espaco do modelo (z = chao + altura).
    [[nodiscard]] Vec3 apply(Vec3 point, float ground) const;
};

} // namespace MatterEngine::Modeling
