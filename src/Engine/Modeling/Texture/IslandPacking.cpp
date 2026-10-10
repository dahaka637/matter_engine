#include "Engine/Modeling/Texture/IslandPacking.hpp"

#include "Engine/Modeling/Geometry/Parallel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

namespace MatterEngine::Modeling {

namespace {

// As celulas de uma ilha (com a folga): o desenho e os trechos de cada linha.
struct Span {
    int from = 0;
    int length = 0;
};

struct Mask {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> marked;
    std::vector<std::vector<Span>> rows;
    // O maior trecho de cada linha (a linha do mapa precisa de um livre
    // desse tamanho).
    std::vector<int> need;
    int cells = 0;
};

void buildRows(Mask& mask) {
    mask.rows.assign(static_cast<std::size_t>(mask.height), {});
    mask.need.assign(static_cast<std::size_t>(mask.height), 0);
    mask.cells = 0;
    for (int y = 0; y < mask.height; ++y)
        for (int x = 0; x < mask.width;) {
            if (!mask.marked[static_cast<std::size_t>(y * mask.width + x)]) {
                ++x;
                continue;
            }
            Span span { x, 0 };
            while (x < mask.width && mask.marked[static_cast<std::size_t>(y * mask.width + x)]) {
                ++span.length;
                ++x;
            }
            mask.rows[static_cast<std::size_t>(y)].push_back(span);
            mask.need[static_cast<std::size_t>(y)] = std::max(mask.need[static_cast<std::size_t>(y)], span.length);
            mask.cells += span.length;
        }
}

// O triangulo encosta no retangulo (eixos separadores: x, y e as normais das
// arestas).
bool touches(const std::array<PackPoint, 3>& t, float x0, float y0, float x1, float y1) {
    if (std::max({ t[0].x, t[1].x, t[2].x }) < x0 || std::min({ t[0].x, t[1].x, t[2].x }) > x1) return false;
    if (std::max({ t[0].y, t[1].y, t[2].y }) < y0 || std::min({ t[0].y, t[1].y, t[2].y }) > y1) return false;
    for (int i = 0; i < 3; ++i) {
        const PackPoint a = t[static_cast<std::size_t>(i)], b = t[static_cast<std::size_t>((i + 1) % 3)];
        const float nx = b.y - a.y, ny = a.x - b.x;
        float low = std::numeric_limits<float>::max(), high = std::numeric_limits<float>::lowest();
        for (const PackPoint p : t) {
            low = std::min(low, nx * p.x + ny * p.y);
            high = std::max(high, nx * p.x + ny * p.y);
        }
        const float c0 = nx * x0 + ny * y0, c1 = nx * x1 + ny * y0, c2 = nx * x0 + ny * y1, c3 = nx * x1 + ny * y1;
        if (std::max({ c0, c1, c2, c3 }) < low || std::min({ c0, c1, c2, c3 }) > high) return false;
    }
    return true;
}

// As celulas a ate "padding" texels de algum triangulo (a ilha com a folga
// de "padding" em volta, comecando em (padding, padding)).
Mask buildMask(const PackIsland& island, int cell, int padding) {
    Mask mask;
    const float c = static_cast<float>(cell), pad = static_cast<float>(padding);
    mask.width = std::max(1, static_cast<int>(std::ceil((island.width + 2.0f * pad) / c)));
    mask.height = std::max(1, static_cast<int>(std::ceil((island.height + 2.0f * pad) / c)));
    mask.marked.assign(static_cast<std::size_t>(mask.width * mask.height), 0);
    for (const auto& triangle : island.triangles) {
        std::array<PackPoint, 3> t;
        for (std::size_t i = 0; i < 3; ++i) t[i] = { triangle[i].x + pad, triangle[i].y + pad };
        const float minX = std::min({ t[0].x, t[1].x, t[2].x }) - pad, maxX = std::max({ t[0].x, t[1].x, t[2].x }) + pad;
        const float minY = std::min({ t[0].y, t[1].y, t[2].y }) - pad, maxY = std::max({ t[0].y, t[1].y, t[2].y }) + pad;
        const int x0 = std::max(0, static_cast<int>(std::floor(minX / c))), x1 = std::min(mask.width - 1, static_cast<int>(std::floor(maxX / c)));
        const int y0 = std::max(0, static_cast<int>(std::floor(minY / c))), y1 = std::min(mask.height - 1, static_cast<int>(std::floor(maxY / c)));
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                std::uint8_t& cellMark = mask.marked[static_cast<std::size_t>(y * mask.width + x)];
                if (!cellMark && touches(t, static_cast<float>(x) * c - pad, static_cast<float>(y) * c - pad, static_cast<float>(x + 1) * c + pad,
                                     static_cast<float>(y + 1) * c + pad))
                    cellMark = 1;
            }
    }
    buildRows(mask);
    return mask;
}

// A ilha em pe: a celula (x, y) vai para (y, largura - 1 - x) - o mesmo giro
// de PackPlacement::at.
Mask turnedMask(const Mask& mask) {
    Mask turned;
    turned.width = mask.height;
    turned.height = mask.width;
    turned.marked.assign(mask.marked.size(), 0);
    for (int y = 0; y < mask.height; ++y)
        for (int x = 0; x < mask.width; ++x)
            if (mask.marked[static_cast<std::size_t>(y * mask.width + x)])
                turned.marked[static_cast<std::size_t>((mask.width - 1 - x) * turned.width + y)] = 1;
    buildRows(turned);
    return turned;
}

// O mapa de ocupacao: em cada celula livre, quantas livres seguem para a
// direita (ela inclusive); na ocupada, menos a distancia ate a proxima livre.
// E o maior trecho livre de cada linha (as linhas cheias saem da busca sem
// percorrer as colunas).
class Occupancy {
public:
    explicit Occupancy(int side)
        : m_side(side), m_run(static_cast<std::size_t>(side) * static_cast<std::size_t>(side)), m_widest(static_cast<std::size_t>(side), side) {
        for (int y = 0; y < side; ++y)
            for (int x = 0; x < side; ++x) m_run[index(x, y)] = static_cast<std::int16_t>(side - x);
    }

    // 0 se a ilha cabe em (x, y); senao, quantas colunas andar.
    [[nodiscard]] int skip(const Mask& mask, int x, int y) const {
        for (int r = 0; r < mask.height; ++r)
            for (const Span& span : mask.rows[static_cast<std::size_t>(r)]) {
                const int run = m_run[index(x + span.from, y + r)];
                if (run <= 0) return -run;
                if (run < span.length) return run + 1;
            }
        return 0;
    }

    // A primeira posicao livre (de cima para baixo, da esquerda para a direita).
    [[nodiscard]] std::optional<std::array<int, 2>> firstFit(const Mask& mask) const {
        for (int y = 0; y + mask.height <= m_side; ++y) {
            bool possible = true;
            for (int r = 0; r < mask.height && possible; ++r) possible = m_widest[static_cast<std::size_t>(y + r)] >= mask.need[static_cast<std::size_t>(r)];
            if (!possible) continue;
            for (int x = 0; x + mask.width <= m_side;) {
                const int step = skip(mask, x, y);
                if (step == 0) return std::array<int, 2> { x, y };
                x += step;
            }
        }
        return std::nullopt;
    }

    void take(const Mask& mask, int x, int y) {
        for (int r = 0; r < mask.height; ++r) {
            const auto& spans = mask.rows[static_cast<std::size_t>(r)];
            if (spans.empty()) continue;
            for (const Span& span : spans)
                for (int i = 0; i < span.length; ++i) m_run[index(x + span.from + i, y + r)] = 0;
            // A linha refeita da direita para a esquerda.
            const int row = y + r;
            int widest = 0;
            for (int column = m_side - 1; column >= 0; --column) {
                const bool used = m_run[index(column, row)] <= 0;
                const int next = column + 1 < m_side ? m_run[index(column + 1, row)] : 0;
                if (used) m_run[index(column, row)] = static_cast<std::int16_t>(next > 0 ? -1 : next - 1);
                else m_run[index(column, row)] = static_cast<std::int16_t>(next > 0 ? next + 1 : 1);
                widest = std::max(widest, static_cast<int>(m_run[index(column, row)]));
            }
            m_widest[static_cast<std::size_t>(row)] = widest;
        }
    }

private:
    int m_side = 0;
    std::vector<std::int16_t> m_run;
    std::vector<int> m_widest;

    [[nodiscard]] std::size_t index(int x, int y) const { return static_cast<std::size_t>(y) * static_cast<std::size_t>(m_side) + static_cast<std::size_t>(x); }
};

} // namespace

std::optional<std::vector<PackPlacement>> packIslands(const std::vector<PackIsland>& islands, const PackSettings& settings) {
    const int cell = std::max(1, settings.cell);
    const int padding = std::max(0, settings.padding);
    const int side = std::max(1, settings.size / cell);
    std::vector<Mask> masks(islands.size());
    parallelFor(islands.size(), 8, [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) masks[i] = buildMask(islands[i], cell, padding);
    });
    std::vector<std::size_t> order(islands.size());
    std::iota(order.begin(), order.end(), std::size_t { 0 });
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return masks[a].cells > masks[b].cells; });
    Occupancy occupancy(side);
    std::vector<PackPlacement> placements(islands.size());
    for (const std::size_t i : order) {
        const Mask& lying = masks[i];
        const Mask standing = turnedMask(lying);
        const auto a = occupancy.firstFit(lying);
        const auto b = occupancy.firstFit(standing);
        if (!a && !b) return std::nullopt;
        // A que termina mais acima (empate: a mais a esquerda).
        const bool turn = !a || (b && ((*b)[1] + standing.height < (*a)[1] + lying.height
                                         || ((*b)[1] + standing.height == (*a)[1] + lying.height && (*b)[0] < (*a)[0])));
        const auto at = turn ? *b : *a;
        occupancy.take(turn ? standing : lying, at[0], at[1]);
        PackPlacement& placement = placements[i];
        placement.originX = static_cast<float>(at[0] * cell);
        placement.originY = static_cast<float>(at[1] * cell);
        placement.turned = turn;
        placement.span = static_cast<float>(lying.width * cell);
        placement.padding = static_cast<float>(padding);
    }
    return placements;
}

} // namespace MatterEngine::Modeling
