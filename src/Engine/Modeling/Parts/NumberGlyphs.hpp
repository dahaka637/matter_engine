#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"
#include "Engine/Modeling/Mesh/MeshImprint.hpp"

#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

// Os algarismos do uniforme (fonte de bloco com chanfros, como nas
// referencias): contornos num quadrado de altura 1 (x para a direita, y para
// cima), o de fora pintado e os furos de volta na cor da peca.
struct DigitGlyph {
    std::vector<std::vector<Point2>> fills;
    std::vector<std::vector<Point2>> holes;
    float width = 0.6f;
};

// stroke: a grossura do traco (fracao da altura, ~0,15 a 0,2).
[[nodiscard]] DigitGlyph digitGlyph(char digit, float stroke);

// O numero montado num plano em frente da peca: centro, o "direita" de quem
// olha e o "cima", a altura dos digitos e o espaco entre eles (fracao da
// altura). As voltas saem prontas para imprintLoops (so cor: o numero e os
// furos de volta).
struct NumberLayout {
    Vec3 center;
    Vec3 right { 0.0f, 1.0f, 0.0f };
    Vec3 up { 0.0f, 0.0f, 1.0f };
    float height = 0.1f;
    float stroke = 0.19f;
    float gap = 0.1f;
};
[[nodiscard]] std::vector<ImprintLoop> numberLoops(std::string_view text, const NumberLayout& layout, int numberMaterial,
    int pieceMaterial);

} // namespace MatterEngine::Modeling
