#include "Engine/Modeling/Parts/ShapeField.hpp"

#include "Engine/Modeling/Parts/SuperSection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace MatterEngine::Modeling {

namespace {

using Section = SuperSection;

Section sectionAt(const SectionStack& stack, float height) {
    return SectionCurves { stack.centerX, stack.front, stack.back, stack.side, stack.exponent, &stack.sideExponent }.at(height,
        0.0f);
}

float sectionRadius(const Section& section, float angle) {
    return section.radius(angle);
}

struct StackSample {
    float value = 0.0f;
    float radius = 0.0f;
    float rho = 0.0f;
    float slope = 0.0f;
};

// rho - R(angulo, altura) no ponto, sem normalizar.
StackSample stackRaw(const SectionStack& stack, Vec3 point, bool withSlope) {
    const Section section = sectionAt(stack, point.z - stack.ground);
    const float dx = point.x - section.centerX, dy = point.y;
    StackSample sample;
    sample.rho = std::sqrt(dx * dx + dy * dy);
    const float angle = std::atan2(dy, dx);
    sample.radius = sectionRadius(section, angle);
    sample.value = sample.rho - sample.radius;
    if (withSlope) {
        constexpr float Step = 1e-3f;
        sample.slope = (sectionRadius(section, angle + Step) - sectionRadius(section, angle - Step)) / (2.0f * Step);
    }
    return sample;
}

// A distancia aproximada: rho - R dividido pelo tamanho do gradiente (o
// lado inclinado da secao e a variacao com a altura).
float stackValue(const SectionStack& stack, Vec3 point) {
    constexpr float Step = 1e-3f;
    const StackSample center = stackRaw(stack, point, true);
    const float above = stackRaw(stack, point + Vec3 { 0.0f, 0.0f, Step }, false).value;
    const float below = stackRaw(stack, point - Vec3 { 0.0f, 0.0f, Step }, false).value;
    const float vertical = (above - below) / (2.0f * Step);
    const float rho = std::max(center.rho, 0.5f * center.radius);
    const float tangential = center.slope / std::max(rho, 1e-4f);
    return center.value / std::sqrt(1.0f + tangential * tangential + vertical * vertical);
}

float signOf(float value) {
    return value > 0.0f ? 1.0f : (value < 0.0f ? -1.0f : 0.0f);
}

// A distancia exata ao cone arredondado (duas esferas e o casco).
float coneValue(const RoundCone& cone, Vec3 point) {
    const Vec3 ba = cone.b - cone.a;
    const float l2 = dot(ba, ba);
    const float rr = cone.radiusA - cone.radiusB;
    const float a2 = l2 - rr * rr;
    if (l2 < 1e-12f || a2 <= 1e-12f)
        return std::min((point - cone.a).length() - cone.radiusA, (point - cone.b).length() - cone.radiusB);
    const float il2 = 1.0f / l2;
    const Vec3 pa = point - cone.a;
    const float y = dot(pa, ba);
    const float z = y - l2;
    const Vec3 xv = pa * l2 - ba * y;
    const float x2 = dot(xv, xv);
    const float y2 = y * y * l2;
    const float z2 = z * z * l2;
    const float k = signOf(rr) * rr * rr * x2;
    if (signOf(z) * a2 * z2 > k) return std::sqrt(x2 + z2) * il2 - cone.radiusB;
    if (signOf(y) * a2 * y2 < k) return std::sqrt(x2 + y2) * il2 - cone.radiusA;
    return (std::sqrt(x2 * a2 * il2) + y * rr) * il2 - cone.radiusA;
}

// A distancia aproximada ao elipsoide (boa perto da superficie).
float ellipsoidValue(const FieldEllipsoid& ellipsoid, Vec3 point) {
    const Vec3 p = ellipsoid.frame.inverse().apply(point);
    const Vec3 r { std::max(ellipsoid.radii.x, 1e-4f), std::max(ellipsoid.radii.y, 1e-4f), std::max(ellipsoid.radii.z, 1e-4f) };
    const Vec3 scaled { p.x / r.x, p.y / r.y, p.z / r.z };
    const Vec3 scaled2 { p.x / (r.x * r.x), p.y / (r.y * r.y), p.z / (r.z * r.z) };
    const float k0 = scaled.length();
    const float k1 = scaled2.length();
    if (k1 < 1e-9f) return -std::min({ r.x, r.y, r.z });
    return k0 * (k0 - 1.0f) / k1;
}

} // namespace

float smoothMinimum(float a, float b, float blend) {
    if (blend <= 0.0f) return std::min(a, b);
    const float h = std::max(blend - std::abs(a - b), 0.0f) / blend;
    return std::min(a, b) - h * h * blend * 0.25f;
}

void ShapeField::add(SectionStack stack, float blend) {
    m_primitives.push_back({ Kind::Stack, m_stacks.size(), blend });
    m_stacks.push_back(std::move(stack));
}

void ShapeField::add(RoundCone cone, float blend) {
    m_primitives.push_back({ Kind::Cone, m_cones.size(), blend });
    m_cones.push_back(cone);
}

void ShapeField::add(ConeChain chain, float blend) {
    m_primitives.push_back({ Kind::Chain, m_chains.size(), blend });
    m_chains.push_back(std::move(chain));
}

void ShapeField::add(FieldEllipsoid ellipsoid, float blend) {
    m_primitives.push_back({ Kind::Ellipsoid, m_ellipsoids.size(), blend });
    m_ellipsoids.push_back(ellipsoid);
}

float ShapeField::primitiveValue(const Primitive& primitive, Vec3 point) const {
    switch (primitive.kind) {
    case Kind::Stack: return stackValue(m_stacks[primitive.index], point);
    case Kind::Cone: return coneValue(m_cones[primitive.index], point);
    case Kind::Chain: {
        const ConeChain& chain = m_chains[primitive.index];
        const std::size_t count = std::min(chain.centers.size(), chain.radii.size());
        if (count == 0) return 1.0f;
        if (count == 1) return (point - chain.centers.front()).length() - chain.radii.front();
        float result = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i + 1 < count; ++i)
            result = std::min(result,
                coneValue({ chain.centers[i], chain.centers[i + 1], chain.radii[i], chain.radii[i + 1] }, point));
        return result;
    }
    case Kind::Ellipsoid: return ellipsoidValue(m_ellipsoids[primitive.index], point);
    }
    return 1.0f;
}

float ShapeField::value(Vec3 point) const {
    if (m_primitives.empty()) return 1.0f;
    float result = primitiveValue(m_primitives.front(), point);
    for (std::size_t i = 1; i < m_primitives.size(); ++i)
        result = smoothMinimum(result, primitiveValue(m_primitives[i], point), m_primitives[i].blend);
    return result;
}

Vec3 ShapeField::gradient(Vec3 point) const {
    constexpr float Step = 1e-4f;
    const auto along = [&](Vec3 axis) {
        return (value(point + axis * Step) - value(point - axis * Step)) / (2.0f * Step);
    };
    return { along({ 1.0f, 0.0f, 0.0f }), along({ 0.0f, 1.0f, 0.0f }), along({ 0.0f, 0.0f, 1.0f }) };
}

Vec3 ShapeField::normal(Vec3 point) const {
    return safeNormalized(gradient(point), { 0.0f, 0.0f, 1.0f });
}

Vec3 ShapeField::project(Vec3 point, int iterations) const {
    for (int i = 0; i < iterations; ++i) {
        const float v = value(point);
        if (std::abs(v) < 1e-6f) break;
        const Vec3 g = gradient(point);
        const float g2 = dot(g, g);
        if (g2 < 1e-12f) break;
        Vec3 step = g * (v / g2);
        // Passos curtos: longe da superficie o gradiente da mistura engana.
        const float length = step.length();
        if (length > 0.02f) step = step * (0.02f / length);
        point = point - step;
    }
    return point;
}

std::optional<Vec3> ShapeField::cross(Vec3 origin, Vec3 direction, float length, float step) const {
    const Vec3 d = safeNormalized(direction, { 1.0f, 0.0f, 0.0f });
    float t0 = 0.0f;
    float v0 = value(origin);
    while (t0 < length) {
        const float t1 = std::min(length, t0 + step);
        const float v1 = value(origin + d * t1);
        if ((v0 <= 0.0f) != (v1 <= 0.0f)) {
            float low = t0, high = t1, lowValue = v0;
            for (int i = 0; i < 40; ++i) {
                const float middle = 0.5f * (low + high);
                const float v = value(origin + d * middle);
                if ((v <= 0.0f) == (lowValue <= 0.0f)) {
                    low = middle;
                    lowValue = v;
                } else {
                    high = middle;
                }
            }
            return origin + d * (0.5f * (low + high));
        }
        t0 = t1;
        v0 = v1;
    }
    return std::nullopt;
}

} // namespace MatterEngine::Modeling
