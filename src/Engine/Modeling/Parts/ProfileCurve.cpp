#include "Engine/Modeling/Parts/ProfileCurve.hpp"

#include <algorithm>

namespace MatterEngine::Modeling {

ProfileCurve::ProfileCurve(std::initializer_list<std::pair<float, float>> points) {
    set(std::vector<std::pair<float, float>>(points));
}

void ProfileCurve::set(std::vector<std::pair<float, float>> points) {
    std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    m_points = std::move(points);
}

float ProfileCurve::operator()(float t) const {
    if (m_points.empty()) return 0.0f;
    if (t <= m_points.front().first) return m_points.front().second;
    if (t >= m_points.back().first) return m_points.back().second;
    std::size_t i = 1;
    while (i < m_points.size() && m_points[i].first < t) ++i;
    const auto& p1 = m_points[i - 1];
    const auto& p2 = m_points[i];
    const auto& p0 = i >= 2 ? m_points[i - 2] : p1;
    const auto& p3 = i + 1 < m_points.size() ? m_points[i + 1] : p2;
    const float span = p2.first - p1.first;
    const float s = span > 1e-6f ? (t - p1.first) / span : 0.0f;
    // Tangentes pelas diferencas dos vizinhos (por unidade de t).
    const float m1 = (p2.second - p0.second) / std::max(1e-6f, p2.first - p0.first);
    const float m2 = (p3.second - p1.second) / std::max(1e-6f, p3.first - p1.first);
    const float s2 = s * s, s3 = s2 * s;
    float value = (2 * s3 - 3 * s2 + 1) * p1.second + (s3 - 2 * s2 + s) * span * m1 + (-2 * s3 + 3 * s2) * p2.second
        + (s3 - s2) * span * m2;
    // Sem passar dos vizinhos (nada de ondas entre dois pontos).
    const float low = std::min(p1.second, p2.second), high = std::max(p1.second, p2.second);
    value = std::clamp(value, low, high);
    return value;
}

} // namespace MatterEngine::Modeling
