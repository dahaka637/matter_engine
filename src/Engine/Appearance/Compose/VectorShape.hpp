#pragma once

#include "Engine/Math/Vec2.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

// As formas vetoriais do desenho (docs/SISTEMA_DE_APARENCIA.md, etapa 2): o
// subconjunto do SVG (M L H V C Q Z, absolutos e relativos) achatado em
// contornos fechados, e o teste de dentro pela regra (nonzero ou evenodd). A
// cobertura e por amostra (o compositor integra), nunca pela distancia a todas as
// curvas.
namespace MatterEngine::Appearance {

struct VectorShape {
    std::vector<std::vector<Vec2>> contours;
    bool evenOdd = false;
    Vec2 min { 0.0f, 0.0f };
    Vec2 max { 0.0f, 0.0f };

    [[nodiscard]] bool empty() const { return contours.empty(); }
    [[nodiscard]] bool contains(Vec2 point) const;
    // A linha horizontal y: os cruzamentos (x e o passo da volta), em ordem de
    // x - dentro e o que a soma dos passos a esquerda diz pela regra.
    [[nodiscard]] std::vector<std::pair<float, int>> crossings(float y) const;
    [[nodiscard]] bool inside(int winding) const { return evenOdd ? (winding & 1) != 0 : winding != 0; }
    // A forma levada por x' = a.x * x + b.x * y + c.x (e o mesmo em y).
    [[nodiscard]] VectorShape transformed(Vec2 a, Vec2 b, Vec2 c) const;
    void append(const VectorShape& other);
    void updateBounds();
};

// O atributo "d" de um caminho. Cada curva vira 16 segmentos; todo subcaminho e
// fechado. Erro: a primeira falha, dita.
[[nodiscard]] std::optional<VectorShape> parsePathData(std::string_view data, bool evenOdd, std::string* error);

} // namespace MatterEngine::Appearance
