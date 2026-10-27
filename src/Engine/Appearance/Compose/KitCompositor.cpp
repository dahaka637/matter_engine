#include "Engine/Appearance/Compose/KitCompositor.hpp"

#include "Engine/Appearance/Compose/FootprintCoverage.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <unordered_map>

namespace MatterEngine::Appearance {
namespace {

using Clock = std::chrono::steady_clock;
double millisecondsSince(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

constexpr float Pi = 3.14159265358979f;
constexpr int Samples = 4;

// Os blocos de 64 linhas, cada um com um trabalhador so (o resultado nao depende
// de quantas threads ha).
template <typename Work> void forRows(int rows, int threads, const Work& work) {
    constexpr int Block = 64;
    const int blocks = (rows + Block - 1) / Block;
    std::atomic<int> next { 0 };
    const auto run = [&] {
        for (int b = next++; b < blocks; b = next++) work(b * Block, std::min(rows, (b + 1) * Block));
    };
    const int count = std::max(1, std::min(threads, blocks));
    std::vector<std::thread> pool;
    for (int i = 1; i < count; ++i) pool.emplace_back(run);
    run();
    for (std::thread& thread : pool) thread.join();
}

// Um triangulo de roupa: os cantos no atlas (texels do nivel 0) e o inverso da
// base deles (o texel para as baricentricas).
struct Tri {
    int panel = -1;
    int region = -1;
    std::array<float, 6> corner {};
    std::array<float, 4> inverse {};
    bool valid = false;
};

struct Bary {
    float b1 = 0.0f, b2 = 0.0f;
};

Bary baryOf(const Tri& tri, float x, float y) {
    const float dx = x - tri.corner[0], dy = y - tri.corner[1];
    return { tri.inverse[0] * dx + tri.inverse[2] * dy, tri.inverse[1] * dx + tri.inverse[3] * dy };
}

bool inside(Bary b) {
    return b.b1 >= 0.0f && b.b2 >= 0.0f && b.b1 + b.b2 <= 1.0f;
}

// A distancia (texels) de um ponto ao segmento.
float segmentDistance(float px, float py, float ax, float ay, float bx, float by) {
    const float ex = bx - ax, ey = by - ay;
    const float length = ex * ex + ey * ey;
    const float t = length > 0.0f ? std::clamp(((px - ax) * ex + (py - ay) * ey) / length, 0.0f, 1.0f) : 0.0f;
    const float dx = px - (ax + t * ex), dy = py - (ay + t * ey);
    return std::sqrt(dx * dx + dy * dy);
}

struct Rgbaf {
    float r = 0.0f, g = 0.0f, b = 0.0f, a = 0.0f;
};

// A cobertura da janela [c - w/2, c + w/2] num intervalo [a, b].
class Shader {
public:
    Shader(const SurfaceTemplate& surface, const KitDesign& design, const Kit& kit, const std::vector<Tri>& tris,
        std::span<const std::array<std::uint32_t, 3>> sources)
        : m_surface(surface), m_design(design), m_kit(kit), m_tris(tris) {
        m_panelTop.assign(surface.panels.size(), -1e9f);
        for (std::size_t t = 0; t < surface.trianglePanel.size(); ++t)
            if (const int panel = surface.trianglePanel[t]; panel >= 0)
                for (const Vec2 corner : surface.triangleDesign[t])
                    m_panelTop[static_cast<std::size_t>(panel)] = std::max(m_panelTop[static_cast<std::size_t>(panel)], corner.y);
        // As bordas: as arestas da abertura no pano de cada painel (a coordenada
        // de desenho dos cantos do triangulo daquele painel que tem a aresta).
        m_border.resize(design.layers.size());
        for (std::size_t l = 0; l < design.layers.size(); ++l) {
            const DesignLayer& layer = design.layers[l];
            if (layer.content != LayerContent::Border) continue;
            m_border[l].resize(surface.panels.size());
            for (const auto& edge : surface.seams[static_cast<std::size_t>(layer.seam)].edges)
                for (std::size_t t = 0; t < sources.size(); ++t) {
                    const int panel = surface.trianglePanel[t];
                    if (panel < 0) continue;
                    for (int k = 0; k < 3; ++k) {
                        const std::uint32_t a = sources[t][k], b = sources[t][(k + 1) % 3];
                        if ((a == edge[0] && b == edge[1]) || (a == edge[1] && b == edge[0]))
                            m_border[l][static_cast<std::size_t>(panel)].push_back(
                                { surface.triangleDesign[t][k], surface.triangleDesign[t][(k + 1) % 3] });
                    }
                }
        }
    }

    // A cor do desenho (pre-multiplicada) no triangulo t, nas baricentricas b
    // (podem estar fora: o desenho continua), com o texel de lado scale (texels
    // do nivel 0).
    [[nodiscard]] Rgbaf shade(std::size_t t, Bary b, float scale) const {
        const Tri& tri = m_tris[t];
        Rgbaf out;
        for (const DesignLayer& layer : m_design.layers) {
            if (!layer.regions[static_cast<std::size_t>(tri.region)] && !layer.panels[static_cast<std::size_t>(tri.panel)]) continue;
            if (layer.anchorPanel >= 0 && layer.anchorPanel != tri.panel) continue;
            std::array<Vec2, 3> c {};
            if (layer.space == LayerSpace::Panel) c = m_surface.triangleDesign[t];
            else if (layer.space == LayerSpace::Chart) {
                if (!m_surface.inChart(static_cast<std::size_t>(layer.chart), t)) continue;
                c = m_surface.triangleChart[static_cast<std::size_t>(layer.chart)][t];
            } else {
                for (int k = 0; k < 3; ++k) c[k] = { -m_surface.triangleReference[t][k].y, m_surface.triangleReference[t][k].z };
            }
            // O ponto e o jacobiano (coordenada da camada por texel deste nivel).
            const float e1x = c[1].x - c[0].x, e1y = c[1].y - c[0].y, e2x = c[2].x - c[0].x, e2y = c[2].y - c[0].y;
            float px = c[0].x + b.b1 * e1x + b.b2 * e2x, py = c[0].y + b.b1 * e1y + b.b2 * e2y;
            float jxx = (e1x * tri.inverse[0] + e2x * tri.inverse[1]) * scale;
            float jxy = (e1x * tri.inverse[2] + e2x * tri.inverse[3]) * scale;
            float jyx = (e1y * tri.inverse[0] + e2y * tri.inverse[1]) * scale;
            float jyy = (e1y * tri.inverse[2] + e2y * tri.inverse[3]) * scale;
            if (layer.content == LayerContent::Border) {
                // (A distancia a aresta mais perto da abertura, no pano; a
                // cobertura pelo footprint.)
                const std::vector<std::array<Vec2, 2>>& edges =
                    m_border[static_cast<std::size_t>(&layer - m_design.layers.data())][static_cast<std::size_t>(tri.panel)];
                float d = 1e9f;
                for (const auto& e : edges) {
                    const float ex = e[1].x - e[0].x, ey = e[1].y - e[0].y, length = ex * ex + ey * ey;
                    const float s = length > 0.0f ? std::clamp(((px - e[0].x) * ex + (py - e[0].y) * ey) / length, 0.0f, 1.0f) : 0.0f;
                    d = std::min(d, std::hypot(px - (e[0].x + s * ex), py - (e[0].y + s * ey)));
                }
                const float w = std::max(1e-7f, std::max(std::hypot(jxx, jyx), std::hypot(jxy, jyy)));
                const float coverage = std::clamp((layer.borderWidth - d) / w + 0.5f, 0.0f, 1.0f);
                const Rgba& color = m_kit.colors[static_cast<std::size_t>(layer.slot)];
                const float alpha = coverage * layer.opacity * color[3];
                if (alpha <= 0.0f) continue;
                out.r = color[0] * alpha + out.r * (1.0f - alpha);
                out.g = color[1] * alpha + out.g * (1.0f - alpha);
                out.b = color[2] * alpha + out.b * (1.0f - alpha);
                out.a = alpha + out.a * (1.0f - alpha);
                continue;
            }
            // A ancora, a transformacao (escala, giro, deslocamento: o inverso aqui).
            px -= layer.origin.x + layer.offset.x;
            py -= layer.origin.y + layer.offset.y;
            if (layer.rotateDegrees != 0.0f) {
                const float a = -layer.rotateDegrees * Pi / 180.0f, cs = std::cos(a), sn = std::sin(a);
                const auto turn = [&](float& x, float& y) {
                    const float nx = cs * x - sn * y;
                    y = sn * x + cs * y;
                    x = nx;
                };
                turn(px, py);
                float ax = jxx, ay = jyx, bx = jxy, by = jyy;
                turn(ax, ay);
                turn(bx, by);
                jxx = ax; jyx = ay; jxy = bx; jyy = by;
            }
            px /= layer.scale.x; jxx /= layer.scale.x; jxy /= layer.scale.x;
            py /= layer.scale.y; jyx /= layer.scale.y; jyy /= layer.scale.y;

            // A cor da camada, pre-multiplicada.
            Rgba color { 0.0f, 0.0f, 0.0f, 0.0f };
            // O degrade: a posicao ao longo dele (0 a 1, presa) e a cor dela,
            // interpolada pre-multiplicada (como no CSS): linear na posicao, a
            // media no footprint e a cor da posicao media.
            const Vec2 gradientAxis = layer.to - layer.from;
            const float gradientLength2 = layer.content == LayerContent::Gradient ? gradientAxis.lengthSquared() : 1.0f;
            const auto gradientAt = [&](float x, float y) {
                return ((x - layer.from.x) * gradientAxis.x + (y - layer.from.y) * gradientAxis.y) / gradientLength2;
            };
            const auto gradientColor = [&](float s) {
                const Rgba& a = m_kit.colors[static_cast<std::size_t>(layer.gradientSlots[0])];
                const Rgba& z = m_kit.colors[static_cast<std::size_t>(layer.gradientSlots[1])];
                Rgba out {};
                for (std::size_t k = 0; k < 3; ++k) out[k] = a[k] * a[3] + (z[k] * z[3] - a[k] * a[3]) * s;
                out[3] = a[3] + (z[3] - a[3]) * s;
                return out;
            };
            if (layer.content == LayerContent::Gradient) {
                // (A media da rampa presa no footprint do texel.)
                color = gradientColor(static_cast<float>(rampMean(gradientAt(px, py),
                    (gradientAxis.x * jxx + gradientAxis.y * jyx) / gradientLength2, (gradientAxis.x * jxy + gradientAxis.y * jyy) / gradientLength2)));
            } else {
                const Rgba& slot = m_kit.colors[static_cast<std::size_t>(layer.slot)];
                color = { slot[0] * slot[3], slot[1] * slot[3], slot[2] * slot[3], slot[3] };
            }
            const float top = layer.fromTop ? m_panelTop[static_cast<std::size_t>(tri.panel)] : 0.0f;
            // Um ponto dentro do conteudo (para as amostras).
            const auto hit = [&](float x, float y) {
                switch (layer.content) {
                case LayerContent::Fill:
                case LayerContent::Gradient: return true;
                case LayerContent::Bands: {
                    const float v = (layer.axis == 0 ? x : y) - top;
                    for (const auto& band : layer.bands)
                        if (v >= band[0] && v < band[1]) return true;
                    return false;
                }
                case LayerContent::Stripes: {
                    const float a = layer.angleDegrees * Pi / 180.0f;
                    const float u = std::cos(a) * x + std::sin(a) * y - layer.phase;
                    return u - std::floor(u / layer.period) * layer.period < layer.width;
                }
                case LayerContent::Path:
                case LayerContent::Text: return layer.shape.contains({ x, y });
                case LayerContent::Border: return false;
                }
                return false;
            };
            float coverage = 0.0f;
            const bool sampled = layer.mask.has_value() || layer.content == LayerContent::Path || layer.content == LayerContent::Text;
            if (sampled) {
                // (A forma longe do texel: nada.)
                const float reach = std::abs(jxx) + std::abs(jxy) + std::abs(jyx) + std::abs(jyy);
                const VectorShape* bounds = layer.content == LayerContent::Path || layer.content == LayerContent::Text
                    ? &layer.shape : layer.mask ? &*layer.mask : nullptr;
                if (bounds != nullptr && (px < bounds->min.x - reach || px > bounds->max.x + reach
                        || py < bounds->min.y - reach || py > bounds->max.y + reach))
                    continue;
                int count = 0;
                float gradientSum = 0.0f;
                for (int i = 0; i < Samples; ++i)
                    for (int j = 0; j < Samples; ++j) {
                        const float ox = (static_cast<float>(i) + 0.5f) / Samples - 0.5f;
                        const float oy = (static_cast<float>(j) + 0.5f) / Samples - 0.5f;
                        const float sx = px + jxx * ox + jxy * oy, sy = py + jyx * ox + jyy * oy;
                        if (hit(sx, sy) && (!layer.mask || layer.mask->contains({ sx, sy }))) {
                            ++count;
                            if (layer.content == LayerContent::Gradient) gradientSum += std::clamp(gradientAt(sx, sy), 0.0f, 1.0f);
                        }
                    }
                coverage = static_cast<float>(count) / (Samples * Samples);
                // (O degrade com mascara: a cor das amostras aceitas - a cor e a
                // cobertura juntas, pre-multiplicadas.)
                if (layer.content == LayerContent::Gradient && count > 0) color = gradientColor(gradientSum / static_cast<float>(count));
            } else if (layer.content == LayerContent::Bands) {
                const float v = (layer.axis == 0 ? px : py) - top;
                const double a = layer.axis == 0 ? jxx : jyx, b = layer.axis == 0 ? jxy : jyy;
                for (const auto& band : layer.bands) coverage += static_cast<float>(stepMean(v - band[0], a, b) - stepMean(v - band[1], a, b));
                coverage = std::clamp(coverage, 0.0f, 1.0f);
            } else if (layer.content == LayerContent::Stripes) {
                const float a = layer.angleDegrees * Pi / 180.0f, cs = std::cos(a), sn = std::sin(a);
                const float u = cs * px + sn * py - layer.phase;
                coverage = static_cast<float>(stripeMean(u, cs * jxx + sn * jyx, cs * jxy + sn * jyy, layer.period, layer.width));
            } else {
                coverage = 1.0f;
            }
            const float amount = std::clamp(coverage, 0.0f, 1.0f) * layer.opacity, alpha = amount * color[3];
            if (alpha <= 0.0f) continue;
            out.r = color[0] * amount + out.r * (1.0f - alpha);
            out.g = color[1] * amount + out.g * (1.0f - alpha);
            out.b = color[2] * amount + out.b * (1.0f - alpha);
            out.a = alpha + out.a * (1.0f - alpha);
        }
        return out;
    }

private:
    const SurfaceTemplate& m_surface;
    const KitDesign& m_design;
    const Kit& m_kit;
    const std::vector<Tri>& m_tris;
    std::vector<float> m_panelTop;
    // [camada][painel]: as arestas da abertura (as bordas).
    std::vector<std::vector<std::vector<std::array<Vec2, 2>>>> m_border;
};

// Um nivel do atlas em ponto flutuante: a cor (pre-multiplicada), a cobertura
// de tecido e a ilha (o painel; -1 vazio) de cada texel.
struct Level {
    int size = 0;
    std::vector<Rgbaf> color;
    std::vector<float> coverage;
    std::vector<int> island;
};

std::uint8_t quantize(float value) {
    return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0f, 1.0f) * 255.0f));
}

} // namespace

std::optional<ComposedKit> composeKit(const SurfaceTemplate& surface, std::span<const std::array<std::uint32_t, 3>> sourceTriangles,
    const KitDesign& design, const Kit& kit, const ComposeOptions& options, std::string* error) {
    const Clock::time_point started = Clock::now();
    const auto fail = [&](const std::string& message) -> std::optional<ComposedKit> {
        if (error != nullptr) *error = "compositor: " + message;
        return std::nullopt;
    };
    const std::size_t triangleCount = surface.trianglePanel.size();
    if (sourceTriangles.size() != triangleCount) return fail("os triangulos nao sao os do template");
    if (kit.colors.size() != design.slotIds.size()) return fail("o uniforme nao tem as vagas do desenho");
    const int size = surface.storageLayout.baseSize;
    if (size <= 0 || (size & (size - 1)) != 0) return fail("o atlas nao e potencia de 2");
    const int threads = options.threads > 0 ? options.threads : std::max(1u, std::thread::hardware_concurrency());
    const int margin = std::max(1, options.margin);
    ComposedKit result;

    // Os triangulos de roupa no atlas.
    std::vector<Tri> tris(triangleCount);
    for (std::size_t t = 0; t < triangleCount; ++t) {
        Tri& tri = tris[t];
        tri.panel = surface.trianglePanel[t];
        tri.region = surface.triangleRegion[t];
        if (tri.panel < 0) continue;
        for (int k = 0; k < 3; ++k) {
            tri.corner[2 * k] = surface.triangleKitUv[t][k].x * static_cast<float>(size);
            tri.corner[2 * k + 1] = surface.triangleKitUv[t][k].y * static_cast<float>(size);
        }
        const float ax = tri.corner[2] - tri.corner[0], ay = tri.corner[3] - tri.corner[1];
        const float bx = tri.corner[4] - tri.corner[0], by = tri.corner[5] - tri.corner[1];
        const float det = ax * by - ay * bx;
        if (std::abs(det) < 1e-12f) continue;
        tri.inverse = { by / det, -ay / det, -bx / det, ax / det };
        tri.valid = true;
    }
    // As costuras continuas: cada aresta de borda de um triangulo e o par dela do
    // outro lado (o triangulo e a aresta), pelos vertices da malha de origem.
    const auto key = [](std::uint32_t a, std::uint32_t b) {
        return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
    };
    std::unordered_map<std::uint64_t, std::vector<std::pair<std::uint32_t, int>>> edgeTriangles;
    for (std::size_t t = 0; t < triangleCount; ++t) {
        if (!tris[t].valid) continue;
        for (int k = 0; k < 3; ++k)
            edgeTriangles[key(sourceTriangles[t][k], sourceTriangles[t][(k + 1) % 3])].push_back({ static_cast<std::uint32_t>(t), k });
    }
    std::unordered_map<std::uint64_t, bool> continuousEdge;
    for (const SurfaceSeam& seam : surface.seams)
        if (seam.kind == SeamKind::Seam && seam.continuous)
            for (const auto& edge : seam.edges) continuousEdge[key(edge[0], edge[1])] = true;
    // Por triangulo e aresta: borda da ilha? e o par pela costura continua (-1 sem).
    std::vector<std::array<bool, 3>> boundary(triangleCount, { false, false, false });
    std::vector<std::array<std::pair<int, int>, 3>> across(triangleCount, { std::pair { -1, -1 }, { -1, -1 }, { -1, -1 } });
    for (std::size_t t = 0; t < triangleCount; ++t) {
        if (!tris[t].valid) continue;
        for (int k = 0; k < 3; ++k) {
            const std::uint64_t edge = key(sourceTriangles[t][k], sourceTriangles[t][(k + 1) % 3]);
            bool samePanel = false;
            for (const auto& [other, otherEdge] : edgeTriangles[edge]) {
                if (other == t) continue;
                if (tris[other].panel == tris[t].panel) samePanel = true;
                else if (continuousEdge.count(edge) != 0 && across[t][k].first < 0)
                    across[t][k] = { static_cast<int>(other), otherEdge };
            }
            boundary[t][k] = !samePanel;
        }
    }
    const Shader shader(surface, design, kit, tris, sourceTriangles);
    result.milliseconds.prepare = millisecondsSince(started);

    // O texel de margem: o triangulo mais perto (ate margin texels; no empate, o
    // menor painel) e a cor dele: pela costura continua, do outro lado; senao, o
    // proprio triangulo estendido.
    std::vector<int> ordered;
    // As caixas dos triangulos (x0, x1, y0, y1 em texels do nivel 0).
    std::vector<std::array<float, 4>> boxes(triangleCount);
    for (std::size_t t = 0; t < triangleCount; ++t) {
        if (!tris[t].valid) continue;
        ordered.push_back(static_cast<int>(t));
        const Tri& tri = tris[t];
        boxes[t] = { std::min({ tri.corner[0], tri.corner[2], tri.corner[4] }), std::max({ tri.corner[0], tri.corner[2], tri.corner[4] }),
            std::min({ tri.corner[1], tri.corner[3], tri.corner[5] }), std::max({ tri.corner[1], tri.corner[3], tri.corner[5] }) };
    }
    const auto marginLevel = [&](Level& level, float scale) {
        // A grade dos triangulos (celulas de margin texels deste nivel).
        const int cells = std::max(1, (level.size + margin - 1) / margin);
        std::vector<std::vector<int>> grid(static_cast<std::size_t>(cells * cells));
        for (const int t : ordered) {
            const Tri& tri = tris[static_cast<std::size_t>(t)];
            float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
            for (int k = 0; k < 3; ++k) {
                x0 = std::min(x0, tri.corner[2 * k] / scale); x1 = std::max(x1, tri.corner[2 * k] / scale);
                y0 = std::min(y0, tri.corner[2 * k + 1] / scale); y1 = std::max(y1, tri.corner[2 * k + 1] / scale);
            }
            const int cx0 = std::clamp(static_cast<int>(std::floor((x0 - margin) / margin)), 0, cells - 1);
            const int cx1 = std::clamp(static_cast<int>(std::floor((x1 + margin) / margin)), 0, cells - 1);
            const int cy0 = std::clamp(static_cast<int>(std::floor((y0 - margin) / margin)), 0, cells - 1);
            const int cy1 = std::clamp(static_cast<int>(std::floor((y1 + margin) / margin)), 0, cells - 1);
            for (int cy = cy0; cy <= cy1; ++cy)
                for (int cx = cx0; cx <= cx1; ++cx) grid[static_cast<std::size_t>(cy * cells + cx)].push_back(t);
        }
        forRows(level.size, threads, [&](int row0, int row1) {
            for (int y = row0; y < row1; ++y)
                for (int x = 0; x < level.size; ++x) {
                    const std::size_t index = static_cast<std::size_t>(y) * level.size + x;
                    if (level.coverage[index] > 0.0f) continue;
                    const float px = static_cast<float>(x) + 0.5f, py = static_cast<float>(y) + 0.5f;
                    const std::vector<int>& near = grid[static_cast<std::size_t>(std::min(cells - 1, y / margin) * cells + std::min(cells - 1, x / margin))];
                    int best = -1, bestEdge = -1;
                    float bestDistance = static_cast<float>(margin), bestEdgeDistance = 1e9f;
                    for (const int t : near) {
                        const Tri& tri = tris[static_cast<std::size_t>(t)];
                        // (A caixa do triangulo mais longe que o melhor ate aqui: nem mede.)
                        const std::array<float, 4>& box = boxes[static_cast<std::size_t>(t)];
                        const float bx = std::max({ 0.0f, box[0] / scale - px, px - box[1] / scale });
                        const float by = std::max({ 0.0f, box[2] / scale - py, py - box[3] / scale });
                        if (bx * bx + by * by > bestDistance * bestDistance) continue;
                        // (A distancia ao triangulo: a menor das arestas, e a aresta de borda mais perto.)
                        float distance = 1e9f, edgeDistance = 1e9f;
                        int edge = -1;
                        for (int k = 0; k < 3; ++k) {
                            const int m = (k + 1) % 3;
                            const float d = segmentDistance(px, py, tri.corner[2 * k] / scale, tri.corner[2 * k + 1] / scale,
                                tri.corner[2 * m] / scale, tri.corner[2 * m + 1] / scale);
                            distance = std::min(distance, d);
                            if (boundary[static_cast<std::size_t>(t)][k] && d < edgeDistance) {
                                edgeDistance = d;
                                edge = k;
                            }
                        }
                        if (distance < bestDistance
                            || (distance == bestDistance && best >= 0 && tri.panel < tris[static_cast<std::size_t>(best)].panel)) {
                            bestDistance = distance;
                            best = t;
                            bestEdge = edge;
                            bestEdgeDistance = edgeDistance;
                        }
                    }
                    (void)bestEdgeDistance;
                    if (best < 0) continue;
                    const Tri& tri = tris[static_cast<std::size_t>(best)];
                    std::size_t shadeTriangle = static_cast<std::size_t>(best);
                    Bary b = baryOf(tri, px * scale, py * scale);
                    if (bestEdge >= 0) {
                        const auto [other, otherEdge] = across[static_cast<std::size_t>(best)][bestEdge];
                        if (other >= 0) {
                            // A posicao ao longo da aresta e a distancia para fora,
                            // levadas a aresta pareada (os mesmos vertices) e para
                            // dentro do triangulo de la, na metrica dele.
                            const int k = bestEdge, m = (k + 1) % 3;
                            const float ax = tri.corner[2 * k] / scale, ay = tri.corner[2 * k + 1] / scale;
                            const float bx = tri.corner[2 * m] / scale, by = tri.corner[2 * m + 1] / scale;
                            const float ex = bx - ax, ey = by - ay, length = std::sqrt(ex * ex + ey * ey);
                            const float s = ((px - ax) * ex + (py - ay) * ey) / (length * length);
                            const float d = std::abs((px - ax) * ey - (py - ay) * ex) / length;
                            const Tri& there = tris[static_cast<std::size_t>(other)];
                            // (Os cantos de la na ordem dos vertices daqui.)
                            const std::uint32_t va = sourceTriangles[static_cast<std::size_t>(best)][k];
                            int ka = otherEdge, kb = (otherEdge + 1) % 3;
                            if (sourceTriangles[static_cast<std::size_t>(other)][ka] != va) std::swap(ka, kb);
                            const float cx = there.corner[2 * ka] / scale, cy = there.corner[2 * ka + 1] / scale;
                            const float dx = there.corner[2 * kb] / scale - cx, dy = there.corner[2 * kb + 1] / scale - cy;
                            const float otherLength = std::sqrt(dx * dx + dy * dy);
                            // A normal para dentro de la: para o terceiro canto.
                            const int kc = 3 - ka - kb;
                            float nx = -dy / otherLength, ny = dx / otherLength;
                            const float tx = there.corner[2 * kc] / scale - cx, ty = there.corner[2 * kc + 1] / scale - cy;
                            if (nx * tx + ny * ty < 0.0f) { nx = -nx; ny = -ny; }
                            const float reach = d * otherLength / std::max(1e-6f, length);
                            const float qx = cx + s * dx + nx * reach, qy = cy + s * dy + ny * reach;
                            shadeTriangle = static_cast<std::size_t>(other);
                            b = baryOf(there, qx * scale, qy * scale);
                        }
                    }
                    level.color[index] = shader.shade(shadeTriangle, b, scale);
                    level.island[index] = tri.panel;
                }
        });
    };

    // O nivel 0: a rasterizacao (o dono de cada texel e a cobertura em 4x4
    // amostras) e a cor do tecido.
    const Clock::time_point composeStart = Clock::now();
    std::vector<Level> levels;
    levels.push_back({ size, std::vector<Rgbaf>(static_cast<std::size_t>(size) * size),
        std::vector<float>(static_cast<std::size_t>(size) * size, 0.0f), std::vector<int>(static_cast<std::size_t>(size) * size, -1) });
    std::vector<int> owner(static_cast<std::size_t>(size) * size, -1);
    std::vector<std::uint8_t> hitTriangle(triangleCount, 0);
    {
        Level& level = levels.front();
        std::vector<float> ownerSamples(static_cast<std::size_t>(size) * size, 0.0f);
        // (Por faixa de linhas: cada trabalhador percorre todos os triangulos na
        // ordem e escreve so as linhas dele.)
        std::vector<std::array<int, 4>> texelBoxes(triangleCount);
        for (const int t : ordered) {
            const std::array<float, 4>& b = boxes[static_cast<std::size_t>(t)];
            texelBoxes[static_cast<std::size_t>(t)] = { std::max(0, static_cast<int>(std::floor(b[0]))), std::min(size - 1, static_cast<int>(std::floor(b[1]))),
                std::max(0, static_cast<int>(std::floor(b[2]))), std::min(size - 1, static_cast<int>(std::floor(b[3]))) };
        }
        std::vector<std::atomic<std::uint8_t>> hits(triangleCount);
        forRows(size, threads, [&](int row0, int row1) {
            for (const int t : ordered) {
                const std::array<int, 4>& box = texelBoxes[static_cast<std::size_t>(t)];
                if (box[3] < row0 || box[2] >= row1) continue;
                const Tri& tri = tris[static_cast<std::size_t>(t)];
                for (int y = std::max(row0, box[2]); y <= std::min(row1 - 1, box[3]); ++y)
                    for (int x = box[0]; x <= box[1]; ++x) {
                        int count = 0;
                        for (int i = 0; i < Samples; ++i)
                            for (int j = 0; j < Samples; ++j)
                                if (inside(baryOf(tri, static_cast<float>(x) + (i + 0.5f) / Samples, static_cast<float>(y) + (j + 0.5f) / Samples)))
                                    ++count;
                        if (count == 0) continue;
                        hits[static_cast<std::size_t>(t)].store(1, std::memory_order_relaxed);
                        const std::size_t index = static_cast<std::size_t>(y) * size + x;
                        const float share = static_cast<float>(count) / (Samples * Samples);
                        level.coverage[index] = std::min(1.0f, level.coverage[index] + share);
                        const bool center = inside(baryOf(tri, static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f));
                        // (O dono: o triangulo do centro; sem ele, o de mais amostras.)
                        const float rank = center ? 2.0f : share;
                        if (rank > ownerSamples[index]) {
                            ownerSamples[index] = rank;
                            owner[index] = t;
                        }
                    }
            }
        });
        for (std::size_t t = 0; t < triangleCount; ++t) {
            hitTriangle[t] = hits[t].load();
            if (tris[t].valid && hitTriangle[t] == 0) ++result.trianglesWithoutTexel;
        }
        forRows(size, threads, [&](int row0, int row1) {
            for (int y = row0; y < row1; ++y)
                for (int x = 0; x < size; ++x) {
                    const std::size_t index = static_cast<std::size_t>(y) * size + x;
                    if (owner[index] < 0) continue;
                    const Tri& tri = tris[static_cast<std::size_t>(owner[index])];
                    level.color[index] = shader.shade(static_cast<std::size_t>(owner[index]),
                        baryOf(tri, static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f), 1.0f);
                    level.island[index] = tri.panel;
                }
        });
        // A media por regiao e os buracos do desenho.
        result.regionAverage.assign(surface.regions.size(), { 0.0f, 0.0f, 0.0f, 0.0f });
        result.transparentTexels.assign(surface.regions.size(), 0);
        std::vector<double> weight(surface.regions.size(), 0.0), sum(4 * surface.regions.size(), 0.0);
        for (std::size_t index = 0; index < owner.size(); ++index) {
            if (owner[index] < 0) continue;
            const std::size_t region = static_cast<std::size_t>(tris[static_cast<std::size_t>(owner[index])].region);
            const Rgbaf& c = level.color[index];
            const double w = level.coverage[index];
            weight[region] += w;
            sum[4 * region] += c.r * w; sum[4 * region + 1] += c.g * w; sum[4 * region + 2] += c.b * w; sum[4 * region + 3] += c.a * w;
            if (c.a < 0.999f) ++result.transparentTexels[region];
        }
        for (std::size_t r = 0; r < weight.size(); ++r)
            if (weight[r] > 0.0)
                for (int k = 0; k < 4; ++k) result.regionAverage[r][static_cast<std::size_t>(k)] = static_cast<float>(sum[4 * r + k] / weight[r]);
    }
    result.milliseconds.compose = millisecondsSince(composeStart);
    const Clock::time_point marginStart = Clock::now();
    marginLevel(levels.front(), 1.0f);
    result.milliseconds.margins = millisecondsSince(marginStart);

    // Os outros niveis: a ilha pelo filho de mais cobertura, a cor pela media
    // dos filhos dela (margens inclusive), e as margens refeitas pela mesma regra.
    const Clock::time_point mipStart = Clock::now();
    for (int s = size / 2; s >= 1; s /= 2) {
        const Level& child = levels.back();
        Level level { s, std::vector<Rgbaf>(static_cast<std::size_t>(s) * s), std::vector<float>(static_cast<std::size_t>(s) * s, 0.0f),
            std::vector<int>(static_cast<std::size_t>(s) * s, -1) };
        forRows(s, threads, [&](int row0, int row1) {
            for (int y = row0; y < row1; ++y)
                for (int x = 0; x < s; ++x) {
                    float coverage = 0.0f, best = 0.0f;
                    int island = -1;
                    for (int j = 0; j < 2; ++j)
                        for (int i = 0; i < 2; ++i) {
                            const std::size_t c = static_cast<std::size_t>(2 * y + j) * child.size + (2 * x + i);
                            const float w = child.coverage[c];
                            if (w <= 0.0f) continue;
                            coverage += w;
                            if (w > best || (w == best && child.island[c] < island)) {
                                best = w;
                                island = child.island[c];
                            }
                        }
                    if (coverage <= 0.0f) continue;
                    // A cor: a caixa 2x2 inteira, com os filhos de margem da mesma
                    // ilha (o outro lado da costura, ja levado para ca). So os
                    // cobertos puxariam a borda para dentro, e os dois lados de uma
                    // costura se afastariam nos niveis altos.
                    Rgbaf sum;
                    int count = 0;
                    for (int j = 0; j < 2; ++j)
                        for (int i = 0; i < 2; ++i) {
                            const std::size_t c = static_cast<std::size_t>(2 * y + j) * child.size + (2 * x + i);
                            if (child.island[c] != island) continue;
                            sum.r += child.color[c].r; sum.g += child.color[c].g; sum.b += child.color[c].b; sum.a += child.color[c].a;
                            ++count;
                        }
                    const std::size_t index = static_cast<std::size_t>(y) * s + x;
                    level.coverage[index] = 0.25f * coverage;
                    const float inverse = 1.0f / static_cast<float>(count);
                    level.color[index] = { sum.r * inverse, sum.g * inverse, sum.b * inverse, sum.a * inverse };
                    level.island[index] = island;
                }
        });
        marginLevel(level, static_cast<float>(size / s));
        levels.push_back(std::move(level));
    }
    // O nivel seguro: o ultimo L em que o bilinear em L e em L+1 de todo texel
    // de tecido so le a propria ilha (a vizinhanca 3x3, com a borda repetida).
    const auto islandAt = [&](const Level& level, int x, int y) {
        x = std::clamp(x, 0, level.size - 1);
        y = std::clamp(y, 0, level.size - 1);
        return level.island[static_cast<std::size_t>(y) * level.size + x];
    };
    result.kitSafeLevel = 0;
    for (std::size_t l = 0; l + 1 < levels.size(); ++l) {
        const Level& here = levels[l];
        const Level& next = levels[l + 1];
        std::atomic<bool> unsafe { false };
        forRows(here.size, threads, [&](int row0, int row1) {
            for (int y = row0; y < row1 && !unsafe.load(std::memory_order_relaxed); ++y)
                for (int x = 0; x < here.size; ++x) {
                    const std::size_t index = static_cast<std::size_t>(y) * here.size + x;
                    if (here.coverage[index] <= 0.0f) continue;
                    const int own = here.island[index];
                    // (Em L, o ponto pode estar em qualquer lugar do texel: os 3x3. Em
                    // L+1, ele fica no quadrante do filho dentro do pai: o pai e os
                    // tres vizinhos daquele lado.)
                    bool safe = true;
                    for (int j = -1; j <= 1 && safe; ++j)
                        for (int i = -1; i <= 1 && safe; ++i) safe = islandAt(here, x + i, y + j) == own;
                    const int sx = (x & 1) != 0 ? 1 : -1, sy = (y & 1) != 0 ? 1 : -1;
                    for (int j = 0; j <= 1 && safe; ++j)
                        for (int i = 0; i <= 1 && safe; ++i) safe = islandAt(next, x / 2 + sx * i, y / 2 + sy * j) == own;
                    if (!safe) {
                        unsafe.store(true, std::memory_order_relaxed);
                        break;
                    }
                }
        });
        if (unsafe.load()) break;
        result.kitSafeLevel = static_cast<int>(l);
    }
    for (const Level& level : levels) {
        KitTexture& out = result.kit.emplace_back();
        out.width = out.height = level.size;
        out.rgba.resize(static_cast<std::size_t>(level.size) * level.size * 4);
        forRows(level.size, threads, [&](int row0, int row1) {
            for (std::size_t i = static_cast<std::size_t>(row0) * level.size; i < static_cast<std::size_t>(row1) * level.size; ++i) {
                const Rgbaf& c = level.color[i];
                out.rgba[4 * i] = quantize(c.r);
                out.rgba[4 * i + 1] = quantize(c.g);
                out.rgba[4 * i + 2] = quantize(c.b);
                out.rgba[4 * i + 3] = quantize(c.a);
            }
        });
    }
    result.milliseconds.mips = millisecondsSince(mipStart);

    // A identificacao: cada zona com estilo desenha o numero no bloco dela. O
    // texel vira coordenada de desenho pela transformacao afim que o template
    // deu ao UV1 (ajustada pelos cantos da zona); a cor pre-multiplicada.
    const Clock::time_point identityStart = Clock::now();
    const IdentityLayout& layout = surface.identityLayout;
    KitTexture identity { layout.width, layout.height, std::vector<std::uint8_t>(static_cast<std::size_t>(layout.width) * layout.height * 4, 0) };
    std::vector<Rgbaf> identityColor(static_cast<std::size_t>(layout.width) * layout.height);
    if (design.identityFont && !kit.number.empty() && layout.width > 0) {
        for (const IdentityStyle& style : design.identity) {
            const SurfaceZone& zone = surface.zones[static_cast<std::size_t>(style.zone)];
            // O ajuste (minimos quadrados) de desenho = A * texel + b nos cantos da zona.
            double m[3][3] {}, rx[3] {}, ry[3] {};
            for (std::size_t t = 0; t < triangleCount; ++t) {
                if (surface.triangleIdentity[t] != style.zone) continue;
                for (int k = 0; k < 3; ++k) {
                    const double u = surface.triangleIdentityUv[t][k].x * layout.width, v = surface.triangleIdentityUv[t][k].y * layout.height;
                    const double row[3] { u, v, 1.0 };
                    for (int a = 0; a < 3; ++a) {
                        for (int c = 0; c < 3; ++c) m[a][c] += row[a] * row[c];
                        rx[a] += row[a] * surface.triangleDesign[t][k].x;
                        ry[a] += row[a] * surface.triangleDesign[t][k].y;
                    }
                }
            }
            const auto solve = [&](const double (&rhs)[3], double (&out)[3]) {
                double a[3][4];
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) a[r][c] = m[r][c];
                    a[r][3] = rhs[r];
                }
                for (int c = 0; c < 3; ++c) {
                    int pivot = c;
                    for (int r = c + 1; r < 3; ++r) if (std::abs(a[r][c]) > std::abs(a[pivot][c])) pivot = r;
                    std::swap(a[c], a[pivot]);
                    if (std::abs(a[c][c]) < 1e-12) return false;
                    for (int r = 0; r < 3; ++r) {
                        if (r == c) continue;
                        const double f = a[r][c] / a[c][c];
                        for (int k = c; k < 4; ++k) a[r][k] -= f * a[c][k];
                    }
                }
                for (int r = 0; r < 3; ++r) out[r] = a[r][3] / a[r][r];
                return true;
            };
            double fx[3], fy[3];
            if (!solve(rx, fx) || !solve(ry, fy)) return fail("a zona " + zone.id + " sem UV1 para o ajuste");
            std::string message;
            std::optional<VectorShape> text = layoutText(*design.identityFont, kit.number, style.height, style.spacing, style.stretch, 0, &message);
            if (!text) return fail(message);
            const float angle = style.rotateDegrees * Pi / 180.0f, cs = std::cos(angle), sn = std::sin(angle);
            const Vec2 center = (zone.min + zone.max) * 0.5f + style.offset;
            const VectorShape shape = text->transformed({ cs, sn }, { -sn, cs }, center);
            const Rgba& color = kit.colors[static_cast<std::size_t>(design.identitySlot)];
            const int x0 = zone.block[0], y0 = zone.block[1], x1 = zone.block[0] + zone.block[2], y1 = zone.block[1] + zone.block[3];
            forRows(y1 - y0, threads, [&](int row0, int row1) {
            for (int y = y0 + row0; y < y0 + row1; ++y) {
                // (O UV1 e o desenho sem giro entre eles - o caso do template: a linha
                // de amostras e uma linha horizontal do desenho, e os cruzamentos dela
                // valem para a linha toda.)
                const bool aligned = std::abs(fx[1]) < 1e-6 * std::abs(fx[0]) && std::abs(fy[0]) < 1e-6 * std::abs(fy[1]);
                std::array<std::vector<std::pair<float, int>>, Samples> rows;
                if (aligned)
                    for (int j = 0; j < Samples; ++j) rows[static_cast<std::size_t>(j)] = shape.crossings(static_cast<float>(fy[1] * (y + (j + 0.5) / Samples) + fy[2]));
                for (int x = x0; x < x1; ++x) {
                    int count = 0;
                    for (int i = 0; i < Samples; ++i)
                        for (int j = 0; j < Samples; ++j) {
                            const double u = x + (i + 0.5) / Samples, v = y + (j + 0.5) / Samples;
                            const Vec2 q { static_cast<float>(fx[0] * u + fx[1] * v + fx[2]), static_cast<float>(fy[0] * u + fy[1] * v + fy[2]) };
                            // (So dentro do retangulo da zona: o limite e exato.)
                            if (q.x < zone.min.x || q.x > zone.max.x || q.y < zone.min.y || q.y > zone.max.y) continue;
                            if (aligned) {
                                int winding = 0;
                                for (const auto& [cx, step] : rows[static_cast<std::size_t>(j)]) {
                                    if (cx > q.x) break;
                                    winding += step;
                                }
                                // (Os passos a esquerda contam ao contrario do teste por raio a direita: o mesmo dentro.)
                                if (shape.inside(-winding)) ++count;
                            } else if (shape.contains(q)) {
                                ++count;
                            }
                        }
                    if (count == 0) continue;
                    const float a = static_cast<float>(count) / (Samples * Samples) * color[3];
                    identityColor[static_cast<std::size_t>(y) * layout.width + x] = { color[0] * a, color[1] * a, color[2] * a, a };
                }
            }
            });
        }
    }
    // Os mips da identificacao (pre-multiplicados, a caixa 2x2) e o nivel seguro:
    // a menor folga entre os blocos (os recuos transparentes) cobre o bilinear
    // em L e em L+1.
    {
        int width = layout.width, height = layout.height;
        std::vector<Rgbaf> current = identityColor;
        while (true) {
            KitTexture& out = result.identity.emplace_back();
            out.width = width;
            out.height = height;
            out.rgba.resize(static_cast<std::size_t>(width) * height * 4);
            for (std::size_t i = 0; i < current.size(); ++i) {
                out.rgba[4 * i] = quantize(current[i].r);
                out.rgba[4 * i + 1] = quantize(current[i].g);
                out.rgba[4 * i + 2] = quantize(current[i].b);
                out.rgba[4 * i + 3] = quantize(current[i].a);
            }
            if (width == 1 && height == 1) break;
            const int w = std::max(1, width / 2), h = std::max(1, height / 2);
            std::vector<Rgbaf> next(static_cast<std::size_t>(w) * h);
            for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x) {
                    Rgbaf sum;
                    int count = 0;
                    for (int j = 0; j < 2; ++j)
                        for (int i = 0; i < 2; ++i) {
                            const int cx = std::min(width - 1, 2 * x + i), cy = std::min(height - 1, 2 * y + j);
                            const Rgbaf& c = current[static_cast<std::size_t>(cy) * width + cx];
                            sum.r += c.r; sum.g += c.g; sum.b += c.b; sum.a += c.a;
                            ++count;
                        }
                    next[static_cast<std::size_t>(y) * w + x] = { sum.r / count, sum.g / count, sum.b / count, sum.a / count };
                }
            current = std::move(next);
            width = w;
            height = h;
        }
        int gap = 1 << 20;
        for (std::size_t a = 0; a < surface.zones.size(); ++a)
            for (std::size_t b = a + 1; b < surface.zones.size(); ++b) {
                const SurfaceZone& p = surface.zones[a];
                const SurfaceZone& q = surface.zones[b];
                const int ax0 = p.block[0] + p.inset, ax1 = p.block[0] + p.block[2] - p.inset;
                const int ay0 = p.block[1] + p.inset, ay1 = p.block[1] + p.block[3] - p.inset;
                const int bx0 = q.block[0] + q.inset, bx1 = q.block[0] + q.block[2] - q.inset;
                const int by0 = q.block[1] + q.inset, by1 = q.block[1] + q.block[3] - q.inset;
                const int dx = std::max({ 0, bx0 - ax1, ax0 - bx1 }), dy = std::max({ 0, by0 - ay1, ay0 - by1 });
                gap = std::min(gap, std::max(dx, dy));
            }
        // (Seguro em L: a folga >= 2^(L+2), dois texels de cada lado em L+1.)
        result.identitySafeLevel = 0;
        while ((gap >> (result.identitySafeLevel + 3)) >= 1 && result.identitySafeLevel + 1 < static_cast<int>(result.identity.size()))
            ++result.identitySafeLevel;
    }
    result.milliseconds.identity = millisecondsSince(identityStart);
    result.milliseconds.total = millisecondsSince(started);
    return result;
}

} // namespace MatterEngine::Appearance
