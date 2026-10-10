#include "Engine/Modeling/Texture/TextureBake.hpp"

#include "Engine/Modeling/Geometry/Parallel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace MatterEngine::Modeling {

namespace {

// Um triangulo pronto para o bake: os cantos no atlas (texels), a amostra
// base (a normal, a parte, a borda) e a caixa de texels dele.
struct BakeTriangle {
    float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f, cx = 0.0f, cy = 0.0f;
    float area = 0.0f;
    float tolerance = 0.0f;
    Vec3 pa, pb, pc;
    SurfaceSample sample;
    int x0 = 0, x1 = 0, y0 = 0, y1 = 0;
    bool fine = false;
};

// O ponto da superficie em (px, py) do atlas, se cai no triangulo. Uma folga
// minuscula: a amostra em cima da aresta conta para os dois lados (sem furo
// entre triangulos da mesma ilha).
bool locate(const BakeTriangle& t, float px, float py, Vec3& position) {
    float w0 = (t.bx - px) * (t.cy - py) - (t.by - py) * (t.cx - px);
    float w1 = (t.cx - px) * (t.ay - py) - (t.cy - py) * (t.ax - px);
    float w2 = (t.ax - px) * (t.by - py) - (t.ay - py) * (t.bx - px);
    if (t.area < 0.0f) {
        w0 = -w0;
        w1 = -w1;
        w2 = -w2;
    }
    if (w0 < t.tolerance || w1 < t.tolerance || w2 < t.tolerance) return false;
    position = (t.pa * w0 + t.pb * w1 + t.pc * w2) / (w0 + w1 + w2);
    return true;
}

float difference(Color a, Color b) { return std::max({ std::abs(a.r - b.r), std::abs(a.g - b.g), std::abs(a.b - b.b) }); }

constexpr int BandRows = 8;

} // namespace

Image bakeTexture(const Mesh& mesh, const SurfacePaint& paint, const TextureBakeSettings& settings) {
    const int size = std::max(1, settings.size);
    const int samples = std::max(1, settings.samples);
    Image image(size, size, { 0.0f, 0.0f, 0.0f, 0.0f });
    if (!mesh.hasUv()) return image;
    const std::size_t texels = static_cast<std::size_t>(size) * static_cast<std::size_t>(size);
    const float side = static_cast<float>(size);
    const auto at = [size](int x, int y) { return static_cast<std::size_t>(y) * static_cast<std::size_t>(size) + static_cast<std::size_t>(x); };
    std::vector<bool> finePart(mesh.parts.size(), false);
    for (const int part : settings.fineParts)
        if (part >= 0 && static_cast<std::size_t>(part) < finePart.size()) finePart[static_cast<std::size_t>(part)] = true;

    std::vector<BakeTriangle> triangles;
    triangles.reserve(mesh.triangles.size());
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const std::array<Uv, 3>& uv = mesh.triangleUv[t];
        BakeTriangle bake;
        bake.ax = uv[0].u * side;
        bake.ay = uv[0].v * side;
        bake.bx = uv[1].u * side;
        bake.by = uv[1].v * side;
        bake.cx = uv[2].u * side;
        bake.cy = uv[2].v * side;
        bake.area = (bake.bx - bake.ax) * (bake.cy - bake.ay) - (bake.by - bake.ay) * (bake.cx - bake.ax);
        if (std::abs(bake.area) < 1e-12f) continue;
        bake.tolerance = -1e-5f * std::abs(bake.area);
        const Triangle& triangle = mesh.triangles[t];
        bake.pa = mesh.positions[triangle[0]];
        bake.pb = mesh.positions[triangle[1]];
        bake.pc = mesh.positions[triangle[2]];
        bake.sample.normal = mesh.normal(t);
        // O texel na superficie (a area do triangulo sobre a dele no atlas) e
        // a borda de cada amostra nele.
        const float edge = settings.edgeTexels > 0.0f ? settings.edgeTexels : 1.0f / static_cast<float>(samples);
        bake.sample.footprint = std::sqrt(triangleArea(bake.pa, bake.pb, bake.pc) / std::max(1e-12f, 0.5f * std::abs(bake.area))) * edge;
        bake.sample.part = mesh.partOf(t);
        bake.sample.material = t < mesh.triangleMaterial.size() ? mesh.triangleMaterial[t] : -1;
        bake.sample.triangle = t;
        bake.x0 = std::max(0, static_cast<int>(std::floor(std::min({ bake.ax, bake.bx, bake.cx }))));
        bake.x1 = std::min(size - 1, static_cast<int>(std::ceil(std::max({ bake.ax, bake.bx, bake.cx }))));
        bake.y0 = std::max(0, static_cast<int>(std::floor(std::min({ bake.ay, bake.by, bake.cy }))));
        bake.y1 = std::min(size - 1, static_cast<int>(std::ceil(std::max({ bake.ay, bake.by, bake.cy }))));
        bake.fine = bake.sample.part >= 0 && static_cast<std::size_t>(bake.sample.part) < finePart.size()
            && finePart[static_cast<std::size_t>(bake.sample.part)];
        if (bake.x0 <= bake.x1 && bake.y0 <= bake.y1) triangles.push_back(bake);
    }
    // As faixas de linhas e os triangulos que passam em cada uma: uma faixa
    // por vez em cada thread, cada texel escrito por uma so.
    const int bands = (size + BandRows - 1) / BandRows;
    std::vector<std::vector<std::uint32_t>> inBand(static_cast<std::size_t>(bands));
    for (std::uint32_t i = 0; i < triangles.size(); ++i)
        for (int band = triangles[i].y0 / BandRows; band <= triangles[i].y1 / BandRows; ++band) inBand[static_cast<std::size_t>(band)].push_back(i);
    const auto eachBand = [&](auto&& work) {
        parallelFor(static_cast<std::size_t>(bands), 1, [&](std::size_t begin, std::size_t end) {
            for (std::size_t band = begin; band < end; ++band) {
                const int rowBegin = static_cast<int>(band) * BandRows, rowEnd = std::min(size, rowBegin + BandRows);
                for (const std::uint32_t i : inBand[band]) work(triangles[i], std::max(triangles[i].y0, rowBegin), std::min(triangles[i].y1, rowEnd - 1));
            }
        });
    };
    const auto eachRow = [&](auto&& work) {
        parallelFor(static_cast<std::size_t>(size), 16, [&](std::size_t begin, std::size_t end) {
            for (std::size_t y = begin; y < end; ++y) work(static_cast<int>(y));
        });
    };

    std::vector<Color> color(texels, Color { 0.0f, 0.0f, 0.0f, 0.0f });
    std::vector<std::uint8_t> covered(texels, 0), fine(texels, 0), refine(texels, 1);
    if (settings.adaptive) {
        // 1. Uma amostra no centro de cada texel (o primeiro triangulo que o cobre).
        eachBand([&](const BakeTriangle& t, int rowFirst, int rowLast) {
            SurfaceSample sample = t.sample;
            for (int y = rowFirst; y <= rowLast; ++y)
                for (int x = t.x0; x <= t.x1; ++x) {
                    const std::size_t k = at(x, y);
                    if (covered[k] || !locate(t, static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f, sample.position)) continue;
                    color[k] = paint(sample);
                    covered[k] = 1;
                    fine[k] = t.fine ? 1 : 0;
                }
        });
        // 2. Todas as amostras so sem o centro, na beira da ilha, nas partes de
        // detalhe fino e onde a cor muda para um vizinho.
        eachRow([&](int y) {
            for (int x = 0; x < size; ++x) {
                const std::size_t k = at(x, y);
                if (!covered[k] || fine[k]) continue;
                bool edge = false;
                for (int dy = -1; dy <= 1 && !edge; ++dy)
                    for (int dx = -1; dx <= 1 && !edge; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        const int nx = x + dx, ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= size || ny >= size) {
                            edge = true;
                            continue;
                        }
                        const std::size_t other = at(nx, ny);
                        edge = !covered[other] || difference(color[other], color[k]) > settings.adaptiveThreshold;
                    }
                refine[k] = edge ? 1 : 0;
            }
        });
    }
    // 3. As amostras inteiras (samples x samples) nos texels marcados.
    std::vector<Color> sum(texels, Color { 0.0f, 0.0f, 0.0f, 0.0f });
    std::vector<int> hits(texels, 0);
    eachBand([&](const BakeTriangle& t, int rowFirst, int rowLast) {
        SurfaceSample sample = t.sample;
        for (int y = rowFirst; y <= rowLast; ++y)
            for (int x = t.x0; x <= t.x1; ++x) {
                const std::size_t k = at(x, y);
                if (!refine[k]) continue;
                for (int sy = 0; sy < samples; ++sy)
                    for (int sx = 0; sx < samples; ++sx) {
                        const float px = static_cast<float>(x) + (static_cast<float>(sx) + 0.5f) / static_cast<float>(samples);
                        const float py = static_cast<float>(y) + (static_cast<float>(sy) + 0.5f) / static_cast<float>(samples);
                        if (!locate(t, px, py, sample.position)) continue;
                        const Color c = paint(sample);
                        sum[k].r += c.r;
                        sum[k].g += c.g;
                        sum[k].b += c.b;
                        ++hits[k];
                    }
            }
    });
    // A cor de cada texel: a media das amostras, ou a do centro.
    std::vector<std::uint8_t> filled(texels, 0);
    eachRow([&](int y) {
        for (int x = 0; x < size; ++x) {
            const std::size_t k = at(x, y);
            if (hits[k] > 0) {
                const float n = static_cast<float>(hits[k]);
                color[k] = { sum[k].r / n, sum[k].g / n, sum[k].b / n, 1.0f };
                filled[k] = 1;
            } else if (covered[k]) {
                filled[k] = 1;
            }
        }
    });
    // A sangria: cada volta pinta o texel vazio com a media dos vizinhos
    // pintados (as faixas juntam o que cresceu e so depois aplicam).
    for (int round = 0; round < settings.bleed; ++round) {
        std::vector<std::vector<std::pair<std::size_t, Color>>> grown(static_cast<std::size_t>(size));
        eachRow([&](int y) {
            for (int x = 0; x < size; ++x) {
                const std::size_t k = at(x, y);
                if (filled[k]) continue;
                Color total { 0.0f, 0.0f, 0.0f, 0.0f };
                int n = 0;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int nx = x + dx, ny = y + dy;
                        if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 || nx >= size || ny >= size) continue;
                        const std::size_t other = at(nx, ny);
                        if (!filled[other]) continue;
                        total.r += color[other].r;
                        total.g += color[other].g;
                        total.b += color[other].b;
                        ++n;
                    }
                if (n > 0) grown[static_cast<std::size_t>(y)].push_back({ k, { total.r / n, total.g / n, total.b / n, 1.0f } });
            }
        });
        bool any = false;
        for (const auto& row : grown)
            for (const auto& [k, c] : row) {
                color[k] = c;
                filled[k] = 1;
                any = true;
            }
        if (!any) break;
    }
    eachRow([&](int y) {
        for (int x = 0; x < size; ++x)
            if (const std::size_t k = at(x, y); filled[k]) image.setPixel(x, y, { color[k].r, color[k].g, color[k].b, 1.0f });
    });
    return image;
}

} // namespace MatterEngine::Modeling
