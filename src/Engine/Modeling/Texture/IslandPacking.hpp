#pragma once

#include <array>
#include <optional>
#include <vector>

// O arranjo das ilhas de um atlas pelo formato delas (nao pela caixa): um
// mapa de ocupacao em celulas de poucos texels; cada ilha, da maior para a
// menor, vai para a primeira posicao livre (de cima para baixo, da esquerda
// para a direita), deitada ou em pe - a que termina mais acima. Uma ilha
// pequena cabe no vao de uma grande (entre as pernas do calcao, debaixo dos
// bracos da camisa). Entre os triangulos de duas ilhas ficam pelo menos
// 2 x padding texels (a sangria do bake e os mipmaps nao misturam as cores).
namespace MatterEngine::Modeling {

struct PackPoint {
    float x = 0.0f;
    float y = 0.0f;
};

// Uma ilha em texels: os triangulos com o canto da caixa em (0, 0).
struct PackIsland {
    std::vector<std::array<PackPoint, 3>> triangles;
    float width = 0.0f;
    float height = 0.0f;
};

// Onde a ilha foi: o ponto (x, y) dela vai para at(x, y).
struct PackPlacement {
    float originX = 0.0f;
    float originY = 0.0f;
    bool turned = false;
    // A largura da regiao da ilha sem girar (o giro e dentro dela).
    float span = 0.0f;
    float padding = 0.0f;

    [[nodiscard]] PackPoint at(PackPoint p) const {
        const PackPoint q { padding + p.x, padding + p.y };
        return turned ? PackPoint { originX + q.y, originY + span - q.x } : PackPoint { originX + q.x, originY + q.y };
    }
};

struct PackSettings {
    int size = 2048;
    int padding = 4;
    // O lado da celula do mapa de ocupacao (texels).
    int cell = 4;
};

// Todas as ilhas no quadrado, ou nada se nao couberem.
[[nodiscard]] std::optional<std::vector<PackPlacement>> packIslands(const std::vector<PackIsland>& islands, const PackSettings& settings);

} // namespace MatterEngine::Modeling
