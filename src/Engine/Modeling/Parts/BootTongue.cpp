#include "Engine/Modeling/Parts/BootTongue.hpp"

#include "Engine/Modeling/Parts/BootShape.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace MatterEngine::Modeling {

namespace {

// O passo entre os aneis da almofada (facetas do tamanho das do cabedal).
constexpr float TongueStep = 0.004f;
// O chanfro dos cantos (de cima) e das bordas (de lado).
constexpr float CornerChamfer = 0.004f;
constexpr float EdgeBevel = 0.003f;

} // namespace

ReliefPiece buildBootTongue(const BootRecipe& recipe, const DrapeField& upper) {
    ReliefPiece piece;
    const auto& outline = recipe.tongueHalfWidth.points();
    if (outline.size() < 2) return piece;
    const float x0 = outline.front().first * recipe.length, x1 = outline.back().first * recipe.length;
    const int count = std::max(4, static_cast<int>(std::ceil((x1 - x0) / TongueStep)));
    const auto surface = [&](float x, float v) { return upper.surfaceHeight(x, v).value_or(0.0f); };
    // Por anel: a base (na pegada) e o topo, da direita (-v) para a esquerda:
    // a borda chanfrada, o terco e o terco do outro lado, a outra borda.
    std::vector<Vec3> right, left;
    std::vector<std::array<Vec3, 4>> top;
    for (int i = 0; i <= count; ++i) {
        const float f = static_cast<float>(i) / static_cast<float>(count);
        const float x = x0 + (x1 - x0) * f, t = x / recipe.length;
        // Os cantos chanfrados (vistos de cima): os aneis das pontas estreitam.
        const float fromEnd = std::min(f, 1.0f - f) * (x1 - x0);
        const float half = recipe.tongueHalfWidth(t) * (0.55f + 0.45f * bootSmoothstep(0.0f, CornerChamfer, fromEnd));
        // A almofada afina para a frente (nunca abaixo de 2 mm).
        const float thick = std::max(std::min(0.002f, recipe.tongueThickness),
            recipe.tongueThickness * (1.0f - 0.5f * bootSmoothstep(0.3f, 1.0f, f)));
        const float bevel = std::min(EdgeBevel, half * 0.3f);
        right.push_back({ x, -half, surface(x, -half) });
        left.push_back({ x, half, surface(x, half) });
        // O topo das pontas recua o chanfro (a parede da frente e a de tras
        // inclinam como as do lado).
        const float topX = i == 0 ? x + recipe.reliefBevel : i == count ? x - recipe.reliefBevel : x;
        top.push_back({ Vec3 { topX, -(half - bevel), surface(topX, -(half - bevel)) + thick },
            Vec3 { topX, -half / 3.0f, surface(topX, -half / 3.0f) + thick },
            Vec3 { topX, half / 3.0f, surface(topX, half / 3.0f) + thick },
            Vec3 { topX, half - bevel, surface(topX, half - bevel) + thick } });
    }
    for (const Vec3& p : right) piece.footprint.push_back(p);
    for (std::size_t i = left.size(); i-- > 0;) piece.footprint.push_back(left[i]);
    // O topo: a grade dos aneis (4 pontos de lado), a borda dando a volta.
    const std::size_t rings = top.size();
    const auto index = [](std::size_t ring, std::size_t k) { return static_cast<std::uint32_t>(ring * 4 + k); };
    for (const auto& ring : top)
        for (const Vec3& p : ring) piece.topPositions.push_back(p);
    for (std::size_t i = 0; i + 1 < rings; ++i)
        for (std::size_t k = 0; k + 1 < 4; ++k) {
            const std::uint32_t a = index(i, k), b = index(i + 1, k), c = index(i + 1, k + 1), d = index(i, k + 1);
            piece.topTriangles.push_back({ a, b, c });
            piece.topTriangles.push_back({ a, c, d });
        }
    for (std::size_t i = 0; i < rings; ++i) piece.rim.push_back(index(i, 0));
    for (std::size_t k = 1; k < 4; ++k) piece.rim.push_back(index(rings - 1, k));
    for (std::size_t i = rings - 1; i-- > 0;) piece.rim.push_back(index(i, 3));
    for (std::size_t k = 3; k-- > 1;) piece.rim.push_back(index(0, k));
    // A pegada: a direita sob a coluna de fora do topo, a esquerda sob a
    // outra; a frente e a de tras sobem para as pontas do topo.
    for (std::size_t i = 0; i < rings; ++i) piece.footprintToRim.push_back(static_cast<std::uint32_t>(i));
    for (std::size_t i = rings; i-- > 0;)
        piece.footprintToRim.push_back(static_cast<std::uint32_t>(rings + 2 + (rings - 1 - i)));
    return piece;
}

} // namespace MatterEngine::Modeling
