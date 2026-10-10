#include "Engine/Modeling/Assembly/FacePaint.hpp"

#include <algorithm>
#include <cmath>
#include <map>

namespace MatterEngine::Modeling {

namespace {

// A mistura com t limitado a [0, 1].
Color blend(Color a, Color b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return { a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, 1.0f };
}

// A altura da linha [|y|, altura] em |y| (reta entre os pontos; fora das
// pontas, a ponta).
float alongPolyline(const std::vector<std::array<float, 2>>& points, float y) {
    if (points.empty()) return 0.0f;
    if (y <= points.front()[0]) return points.front()[1];
    for (std::size_t i = 1; i < points.size(); ++i)
        if (y <= points[i][0]) {
            const float span = std::max(1e-9f, points[i][0] - points[i - 1][0]);
            return points[i - 1][1] + (points[i][1] - points[i - 1][1]) * (y - points[i - 1][0]) / span;
        }
    return points.back()[1];
}

Color tinted(Color c, float r, float g, float b) {
    return { std::clamp(c.r * r, 0.0f, 1.0f), std::clamp(c.g * g, 0.0f, 1.0f), std::clamp(c.b * b, 0.0f, 1.0f), 1.0f };
}

} // namespace

std::optional<Color> paintFace(const FaceLayout& layout, Vec3 point, Vec3 normal, Color skin) {
    const FacePaintShape& s = layout.shape;
    // O lado direito (o esquerdo e o espelho).
    const Vec3 q { point.x, -std::abs(point.y), point.z };
    const Vec3 n { normal.x, point.y > 0.0f ? -normal.y : normal.y, normal.z };
    const float h = q.z - layout.ground;

    // ---- O olho e a sobrancelha, no plano do olho (olhando para a frente e
    // um pouco para fora): u para fora, v para cima ----
    const float yaw = s.eyeYaw * 3.14159265f / 180.0f;
    const Vec3 facing { std::cos(yaw), -std::sin(yaw), 0.0f };
    const Vec3 outward { -std::sin(yaw), -std::cos(yaw), 0.0f };
    const Vec3 center { layout.eyeSurfaceX, s.eyeY, layout.ground + s.eyeHeight };
    const Vec3 d = q - center;
    const float u = dot(d, outward), v = dot(d, { 0.0f, 0.0f, 1.0f }), depth = dot(d, facing);
    if (dot(n, facing) > 0.15f && depth > -0.02f && depth < 0.015f && std::abs(v) < 0.03f && u > -0.035f && u < 0.035f) {
        // A amendoa (o meio inclinado: o canto de fora sobe).
        const float across = u / s.eyeHalfWidth;
        const float rise = s.eyeTilt * u;
        if (std::abs(across) < 1.0f) {
            const float span = 1.0f - across * across;
            // A palpebra de cima mais alta para dentro do meio; a de baixo rasa.
            const float top = rise + s.eyeUpper * std::pow(span, s.upperLidCurve) * (1.0f - 0.18f * across);
            const float bottom = rise - s.eyeLower * std::pow(span, 0.8f);
            if (v < top && v > bottom) {
                // A iris (um pouco para dentro e para cima) e a pupila; o branco
                // escurece perto dos cantos.
                const float iu = u + 0.0006f, iv = v - rise - 0.0003f;
                const float r = std::sqrt(iu * iu + iv * iv);
                if (r < s.pupilRadius) return s.pupil;
                if (r < s.irisRadius) {
                    // O brilho (pequeno, em cima para dentro) e a borda escura.
                    const float gu = iu + 0.0016f, gv = iv - 0.0019f;
                    if (gu * gu + gv * gv < 0.0007f * 0.0007f) return rgb(236, 232, 226);
                    return blend(s.iris, s.pupil, (r - 0.72f * s.irisRadius) / (0.28f * s.irisRadius));
                }
                // Sob a palpebra de cima, a sombra no branco.
                const float shade = std::clamp((top - v) / 0.0016f, 0.0f, 1.0f);
                return blend(tinted(s.sclera, 0.78f, 0.74f, 0.72f), blend(tinted(s.sclera, 0.92f, 0.88f, 0.86f), s.sclera, std::abs(across) < 0.8f ? 1.0f : (1.0f - std::abs(across)) / 0.2f),
                    shade);
            }
            // A linha dos cilios em cima (mais grossa para fora) e a sombra da
            // dobra (por baixo da sobrancelha, se ela descer ate ali).
            const float lash = s.lashWidth * (0.8f + 0.5f * std::clamp(across, 0.0f, 1.0f));
            if (v >= top && v < top + lash) return s.lash;
            if (v >= top + lash && v < top + lash + s.creaseWidth && !(u > -s.browInner && u < s.browOuter && v > s.browLowInner + (s.browLowOuter - s.browLowInner) * (u + s.browInner) / (s.browInner + s.browOuter))) {
                const float c = s.creaseShade;
                return blend(tinted(skin, 1.0f - c, 1.0f - 1.3f * c, 1.0f - 1.4f * c), skin, (v - top - lash) / s.creaseWidth);
            }
            // Embaixo, uma linha fina e mais escura.
            if (v <= bottom && v > bottom - 0.0009f) return tinted(skin, 0.78f, 0.70f, 0.68f);
        } else if (std::abs(across) < 1.1f && std::abs(v - rise) < 0.0009f * (1.1f - std::abs(across)) / 0.1f && across > 0.0f) {
            // O canto de fora estica a linha dos cilios um pouco.
            return s.lash;
        }
        // A sobrancelha: da ponta de dentro (u negativo) a de fora.
        if (u > -s.browInner && u < s.browOuter) {
            const float t = (u + s.browInner) / (s.browInner + s.browOuter);
            const float low = s.browLowInner + (s.browLowOuter - s.browLowInner) * t;
            const float thick = s.browThickInner + (s.browThickOuter - s.browThickInner) * t;
            // As pontas arredondadas (a de fora afina).
            const float endIn = std::clamp((u + s.browInner) / 0.003f, 0.0f, 1.0f);
            const float endOut = std::clamp((s.browOuter - u) / 0.006f, 0.0f, 1.0f);
            const float top = low + thick * std::min(endIn, std::sqrt(endOut));
            if (v > low && v < top) {
                // A borda de baixo bem marcada, a de cima mais macia.
                const float soft = std::clamp((top - v) / 0.0015f, 0.0f, 1.0f);
                return blend(skin, s.brow, 0.35f + 0.65f * soft);
            }
        }
    }

    // ---- A boca, de frente: a fresta e o labio de cima ate o alto dele (as
    // linhas nas arestas da malha); abaixo da fresta, pele ----
    if (n.x > 0.3f && q.x > 0.06f && s.mouthSlit.size() >= 2) {
        const float y = -q.y;
        const float corner = s.mouthSlit.back()[0];
        if (y < corner) {
            const float line = alongPolyline(s.mouthSlit, y);
            const float top = alongPolyline(s.upperLipTop, y);
            const float across = y / corner;
            // A fresta (mais fina nos cantos).
            if (std::abs(h - line) < 0.0007f * (0.45f + 0.55f * (1.0f - across * across))) return s.mouthLine;
            // O labio de cima, macio na borda de cima.
            if (h > line && h < top) {
                const float edge = std::clamp((top - h) / 0.0012f, 0.0f, 1.0f);
                return blend(skin, tinted(skin, 0.90f, 0.79f, 0.77f), edge);
            }
        }
    }
    return std::nullopt;
}

EarPlate earPlate(const Mesh& mesh, int headPart, float minSide, float ground) {
    EarPlate ear;
    // Os triangulos da placa e quem divide aresta com quem.
    std::vector<std::size_t> plate;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        if (mesh.partOf(t) != headPart) continue;
        const Vec3 c = mesh.center(t), n = mesh.normal(t);
        if (c.y < -minSide && n.y < -0.6f) plate.push_back(t);
    }
    if (plate.empty()) return ear;
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::size_t>> edges;
    for (std::size_t k = 0; k < plate.size(); ++k)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = mesh.triangles[plate[k]][static_cast<std::size_t>(i)];
            const std::uint32_t b = mesh.triangles[plate[k]][static_cast<std::size_t>((i + 1) % 3)];
            edges[{ std::min(a, b), std::max(a, b) }].push_back(k);
        }
    // A maior peca ligada (pela area).
    std::vector<int> piece(plate.size(), -1);
    std::vector<float> area;
    for (std::size_t seed = 0; seed < plate.size(); ++seed) {
        if (piece[seed] >= 0) continue;
        const int id = static_cast<int>(area.size());
        area.push_back(0.0f);
        std::vector<std::size_t> stack { seed };
        piece[seed] = id;
        while (!stack.empty()) {
            const std::size_t k = stack.back();
            stack.pop_back();
            area[static_cast<std::size_t>(id)] += mesh.area(plate[k]);
            for (int i = 0; i < 3; ++i) {
                const std::uint32_t a = mesh.triangles[plate[k]][static_cast<std::size_t>(i)];
                const std::uint32_t b = mesh.triangles[plate[k]][static_cast<std::size_t>((i + 1) % 3)];
                for (const std::size_t other : edges[{ std::min(a, b), std::max(a, b) }])
                    if (piece[other] < 0) {
                        piece[other] = id;
                        stack.push_back(other);
                    }
            }
        }
    }
    const int biggest = static_cast<int>(std::max_element(area.begin(), area.end()) - area.begin());
    // A borda dela (arestas de um triangulo so da peca), em volta.
    std::map<std::uint32_t, std::vector<std::uint32_t>> next;
    for (const auto& [edge, users] : edges) {
        int inPiece = 0;
        for (const std::size_t k : users) inPiece += piece[k] == biggest ? 1 : 0;
        if (inPiece != 1) continue;
        next[edge.first].push_back(edge.second);
        next[edge.second].push_back(edge.first);
    }
    if (next.size() < 3) return ear;
    std::vector<std::array<float, 2>>& outline = ear.outline;
    const std::uint32_t start = next.begin()->first;
    std::uint32_t previous = start, current = start;
    for (std::size_t guard = 0; guard <= next.size(); ++guard) {
        const Vec3 p = mesh.positions[current];
        outline.push_back({ p.x, p.z - ground });
        const std::vector<std::uint32_t>& around = next[current];
        std::uint32_t step = around.front();
        if (step == previous && around.size() > 1) step = around[1];
        previous = current;
        current = step;
        if (current == start) break;
    }
    if (outline.size() < 3) {
        outline.clear();
        return ear;
    }
    // A elipse: a area, o centro e os momentos de segunda ordem do poligono;
    // numa elipse cheia a variancia no eixo e (semieixo / 2)^2.
    double polygonArea = 0.0, sx = 0.0, sh = 0.0, xx = 0.0, hh = 0.0, xh = 0.0;
    for (std::size_t i = 0; i < outline.size(); ++i) {
        const double x0 = outline[i][0], h0 = outline[i][1];
        const double x1 = outline[(i + 1) % outline.size()][0], h1 = outline[(i + 1) % outline.size()][1];
        const double c = x0 * h1 - x1 * h0;
        polygonArea += c;
        sx += (x0 + x1) * c;
        sh += (h0 + h1) * c;
        xx += (x0 * x0 + x0 * x1 + x1 * x1) * c;
        hh += (h0 * h0 + h0 * h1 + h1 * h1) * c;
        xh += (x0 * h1 + 2.0 * x0 * h0 + 2.0 * x1 * h1 + x1 * h0) * c;
    }
    polygonArea *= 0.5;
    if (std::abs(polygonArea) < 1e-12) {
        outline.clear();
        return ear;
    }
    const double cx = sx / (6.0 * polygonArea), chh = sh / (6.0 * polygonArea);
    const double varX = xx / (12.0 * polygonArea) - cx * cx, varH = hh / (12.0 * polygonArea) - chh * chh,
                 cov = xh / (24.0 * polygonArea) - cx * chh;
    const double mean = 0.5 * (varX + varH), spread = std::sqrt(std::max(0.0, 0.25 * (varX - varH) * (varX - varH) + cov * cov));
    const double big = mean + spread, small = std::max(1e-12, mean - spread);
    const double angle = 0.5 * std::atan2(2.0 * cov, varX - varH);
    ear.center = { static_cast<float>(cx), static_cast<float>(chh) };
    ear.axis = { static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)) };
    ear.major = static_cast<float>(2.0 * std::sqrt(big));
    ear.minor = static_cast<float>(2.0 * std::sqrt(small));
    return ear;
}

} // namespace MatterEngine::Modeling
