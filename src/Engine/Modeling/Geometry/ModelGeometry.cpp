#include "Engine/Modeling/Geometry/ModelGeometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace MatterEngine::Modeling {

Vec3 lerp(Vec3 a, Vec3 b, float t) {
    return a + (b - a) * t;
}

Vec3 componentMin(Vec3 a, Vec3 b) {
    return { std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z) };
}

Vec3 componentMax(Vec3 a, Vec3 b) {
    return { std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z) };
}

Vec3 safeNormalized(Vec3 vector, Vec3 fallback) {
    const float length = vector.length();
    return length > 1e-9f ? vector / length : fallback;
}

Vec3 anyPerpendicular(Vec3 vector) {
    const Vec3 reference = std::abs(vector.z) < 0.9f ? Vec3 { 0.0f, 0.0f, 1.0f } : Vec3 { 1.0f, 0.0f, 0.0f };
    return safeNormalized(cross(vector, reference), { 0.0f, 1.0f, 0.0f });
}

float distance(Vec3 a, Vec3 b) {
    return (a - b).length();
}

void Box3::include(Vec3 point) {
    minimum = componentMin(minimum, point);
    maximum = componentMax(maximum, point);
}

void Box3::include(const Box3& box) {
    if (!box.valid()) return;
    include(box.minimum);
    include(box.maximum);
}

void Box3::include(Vec3 center, float radius) {
    include(center - Vec3 { radius, radius, radius });
    include(center + Vec3 { radius, radius, radius });
}

bool Box3::valid() const {
    return minimum.x <= maximum.x && minimum.y <= maximum.y && minimum.z <= maximum.z;
}

Vec3 Box3::center() const {
    return valid() ? (minimum + maximum) * 0.5f : Vec3 {};
}

Vec3 Box3::size() const {
    return valid() ? maximum - minimum : Vec3 {};
}

float Box3::radius() const {
    return size().length() * 0.5f;
}

Box3 Box3::expanded(float margin) const {
    if (!valid()) return *this;
    return { minimum - Vec3 { margin, margin, margin }, maximum + Vec3 { margin, margin, margin } };
}

bool Box3::overlaps(const Box3& other) const {
    return minimum.x <= other.maximum.x && maximum.x >= other.minimum.x
        && minimum.y <= other.maximum.y && maximum.y >= other.minimum.y
        && minimum.z <= other.maximum.z && maximum.z >= other.minimum.z;
}

bool Box3::contains(Vec3 point) const {
    return point.x >= minimum.x && point.x <= maximum.x && point.y >= minimum.y && point.y <= maximum.y
        && point.z >= minimum.z && point.z <= maximum.z;
}

Vec3 Transform::apply(Vec3 point) const {
    return position + orientation.rotate(point);
}

Vec3 Transform::applyVector(Vec3 vector) const {
    return orientation.rotate(vector);
}

Transform Transform::inverse() const {
    const Quaternion inverseOrientation = orientation.normalized().conjugate();
    return { inverseOrientation.rotate(-position), inverseOrientation };
}

Transform Transform::operator*(const Transform& child) const {
    return { apply(child.position), (orientation * child.orientation).normalized() };
}

Plane Plane::fromPointNormal(Vec3 point, Vec3 normal) {
    const Vec3 unit = safeNormalized(normal, { 0.0f, 0.0f, 1.0f });
    return { unit, -dot(unit, point) };
}

float Plane::signedDistance(Vec3 point) const {
    return dot(normal, point) + offset;
}

Quaternion quaternionFromBasis(Vec3 xAxis, Vec3 yAxis, Vec3 zAxis) {
    const float m00 = xAxis.x, m01 = yAxis.x, m02 = zAxis.x;
    const float m10 = xAxis.y, m11 = yAxis.y, m12 = zAxis.y;
    const float m20 = xAxis.z, m21 = yAxis.z, m22 = zAxis.z;
    const float trace = m00 + m11 + m22;
    Quaternion q;
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        q = { (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s };
    } else if (m00 > m11 && m00 > m22) {
        const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q = { 0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s };
    } else if (m11 > m22) {
        const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q = { (m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s };
    } else {
        const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q = { (m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s };
    }
    return q.normalized();
}

Quaternion frameAlong(Vec3 xAxis, Vec3 upHint) {
    const Vec3 x = safeNormalized(xAxis, { 1.0f, 0.0f, 0.0f });
    Vec3 z = upHint - x * dot(upHint, x);
    z = z.length() > 1e-6f ? z.normalized() : anyPerpendicular(x);
    const Vec3 y = cross(z, x);
    return quaternionFromBasis(x, y, z);
}

Vec3 axisX(Quaternion orientation) {
    return orientation.rotate({ 1.0f, 0.0f, 0.0f });
}

Vec3 axisY(Quaternion orientation) {
    return orientation.rotate({ 0.0f, 1.0f, 0.0f });
}

Vec3 axisZ(Quaternion orientation) {
    return orientation.rotate({ 0.0f, 0.0f, 1.0f });
}

float angleBetween(Quaternion a, Quaternion b) {
    const Quaternion p = a.normalized();
    const Quaternion q = b.normalized();
    const float d = std::abs(p.x * q.x + p.y * q.y + p.z * q.z + p.w * q.w);
    return 2.0f * std::acos(std::min(1.0f, d));
}

Quaternion partialRotation(Quaternion q, float fraction) {
    q = q.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const float half = std::acos(std::clamp(q.w, -1.0f, 1.0f));
    const float sine = std::sin(half);
    if (sine < 1e-6f) return {};
    const Vec3 axis { q.x / sine, q.y / sine, q.z / sine };
    return Quaternion::fromAxisAngle(axis, 2.0f * half * fraction);
}

Quaternion twistAbout(Quaternion q, Vec3 axis) {
    const float length = axis.length();
    if (length < 1e-6f) return q;
    const Vec3 a = axis / length;
    const float along = q.x * a.x + q.y * a.y + q.z * a.z;
    const Quaternion twist { a.x * along, a.y * along, a.z * along, q.w };
    const float norm = std::sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z + twist.w * twist.w);
    if (norm < 1e-6f) return {};
    return { twist.x / norm, twist.y / norm, twist.z / norm, twist.w / norm };
}

float pointSegmentDistance(Vec3 point, Vec3 a, Vec3 b, float* t) {
    const Vec3 ab = b - a;
    const float lengthSquared = ab.lengthSquared();
    float u = lengthSquared > 1e-12f ? dot(point - a, ab) / lengthSquared : 0.0f;
    u = std::clamp(u, 0.0f, 1.0f);
    if (t != nullptr) *t = u;
    return (point - (a + ab * u)).length();
}

Vec3 closestPointOnTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
    // Ericson, Real-Time Collision Detection, 5.1.5.
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;
    const Vec3 bp = p - b;
    const float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) return a + ab * (d1 / (d1 - d3));
    const Vec3 cp = p - c;
    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) return a + ac * (d2 / (d2 - d6));
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const float denominator = 1.0f / (va + vb + vc);
    return a + ab * (vb * denominator) + ac * (vc * denominator);
}

Vec3 triangleNormal(Vec3 a, Vec3 b, Vec3 c) {
    return safeNormalized(cross(b - a, c - a), {});
}

float triangleArea(Vec3 a, Vec3 b, Vec3 c) {
    return cross(b - a, c - a).length() * 0.5f;
}

namespace {

float component(Vec3 v, int axis) {
    return axis == 0 ? v.x : axis == 1 ? v.y : v.z;
}

// O intervalo do triangulo na reta de intersecao dos planos (Moller, sem
// divisao): false quando o triangulo esta no plano do outro.
bool computeIntervals(float vv0, float vv1, float vv2, float d0, float d1, float d2, float d0d1, float d0d2,
    float& a, float& b, float& c, float& x0, float& x1) {
    if (d0d1 > 0.0f) {
        a = vv2; b = (vv0 - vv2) * d2; c = (vv1 - vv2) * d2; x0 = d2 - d0; x1 = d2 - d1;
    } else if (d0d2 > 0.0f) {
        a = vv1; b = (vv0 - vv1) * d1; c = (vv2 - vv1) * d1; x0 = d1 - d0; x1 = d1 - d2;
    } else if (d1 * d2 > 0.0f || d0 != 0.0f) {
        a = vv0; b = (vv1 - vv0) * d0; c = (vv2 - vv0) * d0; x0 = d0 - d1; x1 = d0 - d2;
    } else if (d1 != 0.0f) {
        a = vv1; b = (vv0 - vv1) * d1; c = (vv2 - vv1) * d1; x0 = d1 - d0; x1 = d1 - d2;
    } else if (d2 != 0.0f) {
        a = vv2; b = (vv0 - vv2) * d2; c = (vv1 - vv2) * d2; x0 = d2 - d0; x1 = d2 - d1;
    } else {
        return false;
    }
    return true;
}

} // namespace

namespace {

// Os dois triangulos projetados no plano perpendicular a normal: os
// interiores se sobrepoem? (eixos separadores: as arestas dos dois).
bool overlapAlong(Vec3 normal, const std::array<Vec3, 3>& first, const std::array<Vec3, 3>& second) {
    const Vec3 u = safeNormalized(anyPerpendicular(normal), { 1.0f, 0.0f, 0.0f });
    const Vec3 v = cross(normal, u);
    std::array<Point2, 3> a {}, b {};
    for (std::size_t i = 0; i < 3; ++i) {
        a[i] = { dot(first[i], u), dot(first[i], v) };
        b[i] = { dot(second[i], u), dot(second[i], v) };
    }
    constexpr float Margin = 1e-7f;
    const auto separated = [&](const std::array<Point2, 3>& edges) {
        for (std::size_t i = 0; i < 3; ++i) {
            const Point2 p = edges[i], q = edges[(i + 1) % 3];
            const Point2 axis { q.y - p.y, p.x - q.x };
            const float length = std::hypot(axis.x, axis.y);
            if (length < 1e-12f) continue;
            float minA = std::numeric_limits<float>::max(), maxA = -minA, minB = minA, maxB = -minA;
            for (const Point2& point : a) {
                const float d = (point.x * axis.x + point.y * axis.y) / length;
                minA = std::min(minA, d);
                maxA = std::max(maxA, d);
            }
            for (const Point2& point : b) {
                const float d = (point.x * axis.x + point.y * axis.y) / length;
                minB = std::min(minB, d);
                maxB = std::max(maxB, d);
            }
            if (maxA <= minB + Margin || maxB <= minA + Margin) return true;
        }
        return false;
    };
    return !separated(a) && !separated(b);
}

} // namespace

bool trianglesIntersect(Vec3 v0, Vec3 v1, Vec3 v2, Vec3 u0, Vec3 u1, Vec3 u2) {
    // Distancias de verdade (normais unitarias): o limiar e de 1 micrometro.
    constexpr float Epsilon = 1e-6f;
    const Vec3 n1 = safeNormalized(cross(v1 - v0, v2 - v0), {});
    if (n1.lengthSquared() == 0.0f) return false;
    const float plane1 = -dot(n1, v0);
    float du0 = dot(n1, u0) + plane1, du1 = dot(n1, u1) + plane1, du2 = dot(n1, u2) + plane1;
    if (std::abs(du0) < Epsilon) du0 = 0.0f;
    if (std::abs(du1) < Epsilon) du1 = 0.0f;
    if (std::abs(du2) < Epsilon) du2 = 0.0f;
    const float du0du1 = du0 * du1, du0du2 = du0 * du2;
    if (du0du1 > 0.0f && du0du2 > 0.0f) return false;

    const Vec3 n2 = safeNormalized(cross(u1 - u0, u2 - u0), {});
    if (n2.lengthSquared() == 0.0f) return false;
    const float plane2 = -dot(n2, u0);
    float dv0 = dot(n2, v0) + plane2, dv1 = dot(n2, v1) + plane2, dv2 = dot(n2, v2) + plane2;
    if (std::abs(dv0) < Epsilon) dv0 = 0.0f;
    if (std::abs(dv1) < Epsilon) dv1 = 0.0f;
    if (std::abs(dv2) < Epsilon) dv2 = 0.0f;
    const float dv0dv1 = dv0 * dv1, dv0dv2 = dv0 * dv2;
    if (dv0dv1 > 0.0f && dv0dv2 > 0.0f) return false;

    const Vec3 direction = cross(n1, n2);
    // Planos quase paralelos (menos de ~0,06 grau) que nao se separaram acima:
    // estao no mesmo plano, e coplanares nao contam (a reta de intersecao
    // seria so ruido numerico).
    if (direction.lengthSquared() < 1e-6f) return false;
    // Ate ~1 grau a reta ainda e instavel: so cruzam se os dois se cobrem
    // vistos de frente (vizinhos de lado, a uma fresta, nao se cruzam).
    if (direction.lengthSquared() < 3e-4f && !overlapAlong(n1, { v0, v1, v2 }, { u0, u1, u2 })) return false;
    int axis = 0;
    float largest = std::abs(direction.x);
    if (std::abs(direction.y) > largest) { largest = std::abs(direction.y); axis = 1; }
    if (std::abs(direction.z) > largest) axis = 2;

    float a = 0, b = 0, c = 0, x0 = 0, x1 = 0;
    if (!computeIntervals(component(v0, axis), component(v1, axis), component(v2, axis), dv0, dv1, dv2, dv0dv1,
            dv0dv2, a, b, c, x0, x1))
        return false;
    float d = 0, e = 0, f = 0, y0 = 0, y1 = 0;
    if (!computeIntervals(component(u0, axis), component(u1, axis), component(u2, axis), du0, du1, du2, du0du1,
            du0du2, d, e, f, y0, y1))
        return false;

    const float xx = x0 * x1, yy = y0 * y1, xxyy = xx * yy;
    float first0 = a * xxyy + b * x1 * yy, first1 = a * xxyy + c * x0 * yy;
    float second0 = d * xxyy + e * xx * y1, second1 = d * xxyy + f * xx * y0;
    if (first0 > first1) std::swap(first0, first1);
    if (second0 > second1) std::swap(second0, second1);
    // So encostar (intervalos que se tocam num ponto) nao conta.
    return !(first1 <= second0 || second1 <= first0);
}

} // namespace MatterEngine::Modeling
