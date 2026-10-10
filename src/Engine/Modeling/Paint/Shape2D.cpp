#include "Engine/Modeling/Paint/Shape2D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

namespace {

constexpr float Pi2 = 6.2831853f;

Point2 add(Point2 a, Point2 b) { return { a.x + b.x, a.y + b.y }; }
Point2 sub(Point2 a, Point2 b) { return { a.x - b.x, a.y - b.y }; }
Point2 mul(Point2 a, float s) { return { a.x * s, a.y * s }; }
float dot2(Point2 a, Point2 b) { return a.x * b.x + a.y * b.y; }
float length2(Point2 a) { return std::sqrt(a.x * a.x + a.y * a.y); }
Point2 unit(Point2 a) {
    const float l = length2(a);
    return l > 1e-12f ? Point2 { a.x / l, a.y / l } : Point2 { 1.0f, 0.0f };
}
// A esquerda da direcao (girada 90 graus no sentido anti-horario).
Point2 leftOf(Point2 d) { return { -d.y, d.x }; }

float segmentDistance(Point2 p, Point2 a, Point2 b) {
    const Point2 ab = sub(b, a);
    const float t = std::clamp(dot2(sub(p, a), ab) / std::max(1e-20f, dot2(ab, ab)), 0.0f, 1.0f);
    return length2(sub(p, add(a, mul(ab, t))));
}

// Quanto o ponto esta a esquerda da reta a->b (positivo: a esquerda).
float isLeft(Point2 a, Point2 b, Point2 p) { return (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y); }

// A contribuicao de uma aresta para as voltas em volta do ponto (Sunday:
// o raio horizontal para a direita).
int crossing(Point2 a, Point2 b, Point2 p) {
    if (a.y <= p.y) {
        if (b.y > p.y && isLeft(a, b, p) > 0.0f) return 1;
    } else if (b.y <= p.y && isLeft(a, b, p) < 0.0f) {
        return -1;
    }
    return 0;
}

int contourWinding(const std::vector<Point2>& contour, Point2 point) {
    int total = 0;
    for (std::size_t i = 0, n = contour.size(); i < n; ++i) total += crossing(contour[i], contour[(i + 1) % n], point);
    return total;
}

// A linha deslocada "offset" para a esquerda de cada trecho, com as juntas
// em quina (limitadas: angulo muito fechado vira chanfro).
std::vector<Point2> offsetLine(const std::vector<Point2>& points, float offset, bool closed) {
    std::vector<Point2> out;
    const std::size_t n = points.size();
    for (std::size_t i = 0; i < n; ++i) {
        const bool first = i == 0, last = i + 1 == n;
        const Point2 p = points[i];
        const bool hasIn = closed || !first, hasOut = closed || !last;
        const Point2 in = hasIn ? unit(sub(p, points[(i + n - 1) % n])) : Point2 {};
        const Point2 outDir = hasOut ? unit(sub(points[(i + 1) % n], p)) : Point2 {};
        if (!hasIn) {
            out.push_back(add(p, mul(leftOf(outDir), offset)));
            continue;
        }
        if (!hasOut) {
            out.push_back(add(p, mul(leftOf(in), offset)));
            continue;
        }
        const Point2 n0 = leftOf(in), n1 = leftOf(outDir);
        const Point2 miter = unit(add(n0, n1));
        const float along = dot2(miter, n0);
        if (along > 0.5f) {
            out.push_back(add(p, mul(miter, offset / along)));
        } else {
            out.push_back(add(p, mul(n0, offset)));
            out.push_back(add(p, mul(n1, offset)));
        }
    }
    return out;
}

} // namespace

void Box2::include(Point2 point) {
    minimum.x = std::min(minimum.x, point.x);
    minimum.y = std::min(minimum.y, point.y);
    maximum.x = std::max(maximum.x, point.x);
    maximum.y = std::max(maximum.y, point.y);
}

void Box2::include(const Box2& other) {
    if (!other.valid()) return;
    include(other.minimum);
    include(other.maximum);
}

float Box2::distanceOutside(Point2 point) const {
    if (!valid()) return std::numeric_limits<float>::max();
    const float dx = std::max({ minimum.x - point.x, 0.0f, point.x - maximum.x });
    const float dy = std::max({ minimum.y - point.y, 0.0f, point.y - maximum.y });
    return std::sqrt(dx * dx + dy * dy);
}

Affine2 Affine2::then(const Affine2& next) const {
    Affine2 out;
    out.a = next.a * a + next.b * c;
    out.b = next.a * b + next.b * d;
    out.c = next.c * a + next.d * c;
    out.d = next.c * b + next.d * d;
    out.tx = next.a * tx + next.b * ty + next.tx;
    out.ty = next.c * tx + next.d * ty + next.ty;
    return out;
}

Affine2 Affine2::translation(float x, float y) {
    Affine2 out;
    out.tx = x;
    out.ty = y;
    return out;
}

Affine2 Affine2::scaling(float sx, float sy) {
    Affine2 out;
    out.a = sx;
    out.d = sy;
    return out;
}

Affine2 Affine2::rotation(float radians) {
    Affine2 out;
    out.a = std::cos(radians);
    out.b = -std::sin(radians);
    out.c = std::sin(radians);
    out.d = std::cos(radians);
    return out;
}

Box2 Shape2D::bounds() const {
    Box2 box;
    for (const std::vector<Point2>& contour : contours)
        for (const Point2& p : contour) box.include(p);
    return box;
}

int Shape2D::winding(Point2 point) const {
    int total = 0;
    for (const std::vector<Point2>& contour : contours) total += contourWinding(contour, point);
    return total;
}

void Shape2D::add(const Shape2D& other) {
    contours.insert(contours.end(), other.contours.begin(), other.contours.end());
}

Shape2D Shape2D::transformed(const Affine2& transform) const {
    Shape2D out;
    const bool mirrors = transform.determinant() < 0.0f;
    for (const std::vector<Point2>& contour : contours) {
        std::vector<Point2> moved;
        moved.reserve(contour.size());
        for (const Point2& p : contour) moved.push_back(transform.apply(p));
        if (mirrors) std::reverse(moved.begin(), moved.end());
        out.contours.push_back(std::move(moved));
    }
    return out;
}

float contourArea(const std::vector<Point2>& contour) {
    double area = 0.0;
    for (std::size_t i = 0, n = contour.size(); i < n; ++i) {
        const Point2 a = contour[i], b = contour[(i + 1) % n];
        area += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    }
    return static_cast<float>(0.5 * area);
}

Shape2D polygonShape(std::vector<Point2> points) {
    Shape2D shape;
    if (points.size() < 3) return shape;
    if (contourArea(points) < 0.0f) std::reverse(points.begin(), points.end());
    shape.contours.push_back(std::move(points));
    return shape;
}

Shape2D rectangleShape(Point2 minimum, Point2 maximum, float chamfer, float round) {
    const float x0 = std::min(minimum.x, maximum.x), x1 = std::max(minimum.x, maximum.x);
    const float y0 = std::min(minimum.y, maximum.y), y1 = std::max(minimum.y, maximum.y);
    const float half = 0.5f * std::min(x1 - x0, y1 - y0);
    std::vector<Point2> points;
    if (round > 0.0f) {
        const float r = std::min(round, half);
        const Point2 centers[4] { { x1 - r, y0 + r }, { x1 - r, y1 - r }, { x0 + r, y1 - r }, { x0 + r, y0 + r } };
        constexpr int Steps = 10;
        for (int corner = 0; corner < 4; ++corner)
            for (int k = 0; k <= Steps; ++k) {
                const float angle = (static_cast<float>(corner) - 1.0f + static_cast<float>(k) / Steps) * 0.25f * Pi2;
                points.push_back({ centers[corner].x + r * std::cos(angle), centers[corner].y + r * std::sin(angle) });
            }
    } else if (chamfer > 0.0f) {
        const float c = std::min(chamfer, half);
        points = { { x0 + c, y0 }, { x1 - c, y0 }, { x1, y0 + c }, { x1, y1 - c }, { x1 - c, y1 }, { x0 + c, y1 }, { x0, y1 - c }, { x0, y0 + c } };
    } else {
        points = { { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 } };
    }
    return polygonShape(std::move(points));
}

Shape2D ellipseShape(Point2 center, Point2 radii, int segments) {
    std::vector<Point2> points;
    const int n = std::max(8, segments);
    for (int k = 0; k < n; ++k) {
        const float angle = Pi2 * static_cast<float>(k) / static_cast<float>(n);
        points.push_back({ center.x + radii.x * std::cos(angle), center.y + radii.y * std::sin(angle) });
    }
    return polygonShape(std::move(points));
}

Shape2D strokeShape(const std::vector<Point2>& points, float width, bool closed) {
    Shape2D shape;
    if (points.size() < 2 || width <= 0.0f) return shape;
    const float half = 0.5f * width;
    if (!closed) {
        std::vector<Point2> left = offsetLine(points, half, false), right = offsetLine(points, -half, false);
        std::reverse(right.begin(), right.end());
        left.insert(left.end(), right.begin(), right.end());
        return polygonShape(std::move(left));
    }
    std::vector<Point2> a = offsetLine(points, half, true), b = offsetLine(points, -half, true);
    if (std::abs(contourArea(a)) < std::abs(contourArea(b))) std::swap(a, b);
    if (contourArea(a) < 0.0f) std::reverse(a.begin(), a.end());
    if (contourArea(b) > 0.0f) std::reverse(b.begin(), b.end());
    shape.contours.push_back(std::move(a));
    shape.contours.push_back(std::move(b));
    return shape;
}

PreparedShape::PreparedShape(const Shape2D& shape) {
    m_bounds = shape.bounds();
    if (!m_bounds.valid()) return;
    const float diagonal = std::max(1e-6f, length2(sub(m_bounds.maximum, m_bounds.minimum)));
    const float step = diagonal / 256.0f;
    for (std::size_t k = 0; k < shape.contours.size(); ++k) {
        const std::vector<Point2>& contour = shape.contours[k];
        if (contour.size() < 3) continue;
        const float area = contourArea(contour);
        if (std::abs(area) < 1e-14f) continue;
        const int sign = area > 0.0f ? 1 : -1;
        for (std::size_t i = 0, n = contour.size(); i < n; ++i) {
            const Point2 a = contour[i], b = contour[(i + 1) % n];
            if (a.x == b.x && a.y == b.y) continue;
            m_edges.push_back({ a, b });
            // A aresta e borda onde as voltas dos outros contornos deixam um
            // lado dentro e o outro fora (o proprio contorno soma "sign" de um
            // lado so).
            const int pieces = std::max(1, static_cast<int>(std::ceil(length2(sub(b, a)) / step)));
            bool open = false;
            Point2 start {};
            for (int piece = 0; piece < pieces; ++piece) {
                const Point2 from = add(a, mul(sub(b, a), static_cast<float>(piece) / pieces));
                const Point2 to = add(a, mul(sub(b, a), static_cast<float>(piece + 1) / pieces));
                const Point2 middle = mul(add(from, to), 0.5f);
                int others = 0;
                for (std::size_t j = 0; j < shape.contours.size(); ++j)
                    if (j != k) others += contourWinding(shape.contours[j], middle);
                const bool boundary = ((others + sign) != 0) != (others != 0);
                if (boundary && !open) {
                    open = true;
                    start = from;
                } else if (!boundary && open) {
                    open = false;
                    m_segments.push_back({ start, from });
                }
                if (open && piece + 1 == pieces) m_segments.push_back({ start, to });
            }
        }
    }
    // As faixas de altura.
    const float height = std::max(1e-6f, m_bounds.maximum.y - m_bounds.minimum.y);
    const int bands = std::clamp(static_cast<int>(m_edges.size() / 6), 1, 64);
    m_bandHeight = height / static_cast<float>(bands);
    m_edgeBands.assign(static_cast<std::size_t>(bands), {});
    m_segmentBands.assign(static_cast<std::size_t>(bands), {});
    const auto spread = [&](const std::vector<Segment>& list, std::vector<std::vector<std::uint32_t>>& target) {
        for (std::size_t i = 0; i < list.size(); ++i) {
            const int low = band(std::min(list[i].a.y, list[i].b.y)), high = band(std::max(list[i].a.y, list[i].b.y));
            for (int bandIndex = low; bandIndex <= high; ++bandIndex)
                target[static_cast<std::size_t>(bandIndex)].push_back(static_cast<std::uint32_t>(i));
        }
    };
    spread(m_edges, m_edgeBands);
    spread(m_segments, m_segmentBands);
}

int PreparedShape::band(float y) const {
    if (m_edgeBands.empty()) return 0;
    const int count = static_cast<int>(m_edgeBands.size());
    return std::clamp(static_cast<int>(std::floor((y - m_bounds.minimum.y) / m_bandHeight)), 0, count - 1);
}

bool PreparedShape::contains(Point2 point) const {
    if (m_edges.empty() || m_bounds.distanceOutside(point) > 0.0f) return false;
    int total = 0;
    for (const std::uint32_t i : m_edgeBands[static_cast<std::size_t>(band(point.y))]) total += crossing(m_edges[i].a, m_edges[i].b, point);
    return total != 0;
}

float PreparedShape::edgeDistance(Point2 point, float limit) const {
    float best = limit;
    if (m_segments.empty()) return best;
    const bool all = !(limit < m_bounds.maximum.y - m_bounds.minimum.y);
    if (all) {
        for (const Segment& s : m_segments) best = std::min(best, segmentDistance(point, s.a, s.b));
        return best;
    }
    if (m_bounds.distanceOutside(point) >= limit) return limit;
    const int low = band(point.y - limit), high = band(point.y + limit);
    for (int b = low; b <= high; ++b)
        for (const std::uint32_t i : m_segmentBands[static_cast<std::size_t>(b)])
            best = std::min(best, segmentDistance(point, m_segments[i].a, m_segments[i].b));
    return best;
}

float PreparedShape::signedDistance(Point2 point) const {
    const float d = edgeDistance(point);
    return contains(point) ? -d : d;
}

float PreparedShape::coverage(Point2 point, float footprint, float grow) const {
    if (m_segments.empty()) return 0.0f;
    const float half = 0.5f * std::max(0.0f, footprint);
    if (m_bounds.distanceOutside(point) > grow + half) return 0.0f;
    const bool inside = contains(point);
    if (footprint <= 0.0f) {
        if (inside) return 1.0f;
        return grow > 0.0f && edgeDistance(point, grow) < grow ? 1.0f : 0.0f;
    }
    if (inside) {
        // Ate a borda engordada: grow mais a distancia ate a borda.
        if (grow >= half) return 1.0f;
        const float reach = half - grow;
        const float d = edgeDistance(point, reach);
        if (d >= reach) return 1.0f;
        return std::clamp(0.5f + (d + grow) / footprint, 0.0f, 1.0f);
    }
    const float reach = grow + half;
    const float d = edgeDistance(point, reach);
    if (d >= reach) return 0.0f;
    return std::clamp(0.5f - (d - grow) / footprint, 0.0f, 1.0f);
}

} // namespace MatterEngine::Modeling
