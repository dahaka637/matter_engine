#pragma once

#include <initializer_list>
#include <utility>
#include <vector>

namespace MatterEngine::Modeling {

// Uma curva por pontos de controle (t, valor), suave entre eles
// (Catmull-Rom sem passar dos vizinhos): os perfis das pecas ao longo do
// comprimento (largura, altura) escritos como numeros nas receitas.
class ProfileCurve {
public:
    ProfileCurve() = default;
    ProfileCurve(std::initializer_list<std::pair<float, float>> points);

    void set(std::vector<std::pair<float, float>> points);
    [[nodiscard]] bool empty() const { return m_points.empty(); }
    [[nodiscard]] const std::vector<std::pair<float, float>>& points() const { return m_points; }
    [[nodiscard]] float operator()(float t) const;

private:
    std::vector<std::pair<float, float>> m_points;
};

} // namespace MatterEngine::Modeling
