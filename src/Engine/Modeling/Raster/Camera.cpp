#include "Engine/Modeling/Raster/Camera.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace MatterEngine::Modeling {

Vec3 OrbitCamera::eye() const {
    const float c = std::cos(pitch);
    return target + Vec3 { c * std::cos(yaw), c * std::sin(yaw), std::sin(pitch) } * distance;
}

Vec3 OrbitCamera::forward() const {
    const float c = std::cos(pitch);
    return -Vec3 { c * std::cos(yaw), c * std::sin(yaw), std::sin(pitch) };
}

Vec3 OrbitCamera::right() const {
    // Independe do pitch: a vista de cima nao degenera.
    return { -std::sin(yaw), std::cos(yaw), 0.0f };
}

Vec3 OrbitCamera::up() const {
    return cross(right(), forward());
}

Projector::Projector(const OrbitCamera& camera, int width, int height)
    : m_width(std::max(1, width)), m_height(std::max(1, height)), m_orthographic(camera.orthographic),
      m_eye(camera.eye()), m_forward(camera.forward()), m_right(camera.right()), m_up(camera.up()) {
    m_aspect = static_cast<float>(m_width) / static_cast<float>(m_height);
    m_focal = 1.0f / std::tan(std::clamp(camera.verticalFov, 0.05f, 2.8f) * 0.5f);
    m_orthoHalfHeight = std::max(1e-4f, camera.orthoHalfHeight);
    m_near = m_orthographic ? 1e-4f : std::clamp(camera.distance * 0.01f, 0.002f, 0.05f);
}

Vec3 Projector::toCamera(Vec3 world) const {
    const Vec3 relative = world - m_eye;
    return { dot(relative, m_right), dot(relative, m_up), dot(relative, m_forward) };
}

ScreenPoint Projector::projectCamera(Vec3 cameraSpace) const {
    ScreenPoint point;
    point.cameraZ = cameraSpace.z;
    point.visible = cameraSpace.z >= m_near;
    float ndcX = 0.0f, ndcY = 0.0f;
    if (m_orthographic) {
        ndcX = cameraSpace.x / (m_orthoHalfHeight * m_aspect);
        ndcY = cameraSpace.y / m_orthoHalfHeight;
    } else {
        const float z = std::max(cameraSpace.z, m_near);
        ndcX = cameraSpace.x * m_focal / (m_aspect * z);
        ndcY = cameraSpace.y * m_focal / z;
    }
    point.x = (ndcX + 1.0f) * 0.5f * static_cast<float>(m_width);
    point.y = (1.0f - ndcY) * 0.5f * static_cast<float>(m_height);
    point.depth = depthKey(cameraSpace.z);
    return point;
}

ScreenPoint Projector::project(Vec3 world) const {
    return projectCamera(toCamera(world));
}

float Projector::depthKey(float cameraZ) const {
    // Perspectiva: 1/z e linear na tela; ortografica: o proprio z.
    return m_orthographic ? -cameraZ : 1.0f / std::max(cameraZ, m_near);
}

float Projector::towardCamera(float key, float meters) const {
    if (m_orthographic) return key + meters;
    const float z = 1.0f / std::max(key, 1e-6f);
    return 1.0f / std::max(z - meters, m_near);
}

float Projector::pixelsPerMeter(float cameraZ) const {
    if (m_orthographic) return 0.5f * static_cast<float>(m_height) / m_orthoHalfHeight;
    return 0.5f * static_cast<float>(m_height) * m_focal / std::max(cameraZ, m_near);
}

void Projector::ray(float pixelX, float pixelY, Vec3& origin, Vec3& direction) const {
    const float ndcX = pixelX / static_cast<float>(m_width) * 2.0f - 1.0f;
    const float ndcY = 1.0f - pixelY / static_cast<float>(m_height) * 2.0f;
    if (m_orthographic) {
        origin = m_eye + m_right * (ndcX * m_orthoHalfHeight * m_aspect) + m_up * (ndcY * m_orthoHalfHeight);
        direction = m_forward;
    } else {
        origin = m_eye;
        direction = safeNormalized(m_forward + m_right * (ndcX * m_aspect / m_focal) + m_up * (ndcY / m_focal),
            m_forward);
    }
}

bool Projector::barycentricAt(Vec3 a, Vec3 b, Vec3 c, float pixelX, float pixelY, std::array<float, 3>& out) const {
    const Vec3 e1 = b - a, e2 = c - a, normal = cross(e1, e2);
    const float d11 = dot(e1, e1), d12 = dot(e1, e2), d22 = dot(e2, e2), gram = d11 * d22 - d12 * d12;
    if (gram <= 0.0f) return false;
    Vec3 origin, direction;
    ray(pixelX, pixelY, origin, direction);
    const float facing = dot(normal, direction);
    if (std::abs(facing) <= 1e-6f * std::sqrt(gram)) return false;
    const Vec3 p = origin + direction * (dot(normal, a - origin) / facing) - a;
    const float p1 = dot(p, e1), p2 = dot(p, e2);
    const float u = (d22 * p1 - d12 * p2) / gram, v = (d11 * p2 - d12 * p1) / gram;
    out = { 1.0f - u - v, u, v };
    return true;
}

Vec3 Projector::viewDirection(Vec3 world) const {
    return m_orthographic ? m_forward : safeNormalized(world - m_eye, m_forward);
}

void frameBox(OrbitCamera& camera, const Box3& box, float aspect, float margin) {
    if (!box.valid()) return;
    const Vec3 center = box.center();
    camera.target = center;
    const Vec3 right = camera.right(), up = camera.up(), forward = camera.forward();
    std::array<Vec3, 8> corners {};
    for (int i = 0; i < 8; ++i)
        corners[static_cast<std::size_t>(i)] = { i & 1 ? box.maximum.x : box.minimum.x,
            i & 2 ? box.maximum.y : box.minimum.y, i & 4 ? box.maximum.z : box.minimum.z };
    aspect = std::max(0.1f, aspect);
    if (camera.orthographic) {
        float halfHeight = 0.01f;
        float depth = 0.0f;
        for (const Vec3& corner : corners) {
            const Vec3 offset = corner - center;
            halfHeight = std::max({ halfHeight, std::abs(dot(offset, up)), std::abs(dot(offset, right)) / aspect });
            depth = std::max(depth, std::abs(dot(offset, forward)));
        }
        camera.orthoHalfHeight = halfHeight * margin;
        camera.distance = depth + 1.0f;
        return;
    }
    const float focal = 1.0f / std::tan(camera.verticalFov * 0.5f);
    float required = 0.05f;
    for (const Vec3& corner : corners) {
        const Vec3 offset = corner - center;
        const float depth = dot(offset, forward);
        // (E o olho fora da caixa.)
        required = std::max({ required, std::abs(dot(offset, right)) * focal * margin / aspect - depth,
            std::abs(dot(offset, up)) * focal * margin - depth, 0.02f - depth });
    }
    camera.distance = required;
}

void framePoints(OrbitCamera& camera, std::span<const FramePoint> points, float aspect, float margin) {
    if (points.empty()) return;
    const Vec3 right = camera.right(), up = camera.up(), forward = camera.forward();
    // O centro: o meio do que se ve (nos eixos da camera).
    float minRight = 1e9f, maxRight = -1e9f, minUp = 1e9f, maxUp = -1e9f, minDepth = 1e9f, maxDepth = -1e9f;
    for (const FramePoint& point : points) {
        const float r = dot(point.position, right), u = dot(point.position, up), d = dot(point.position, forward);
        minRight = std::min(minRight, r - point.radius);
        maxRight = std::max(maxRight, r + point.radius);
        minUp = std::min(minUp, u - point.radius);
        maxUp = std::max(maxUp, u + point.radius);
        minDepth = std::min(minDepth, d - point.radius);
        maxDepth = std::max(maxDepth, d + point.radius);
    }
    const Vec3 center = right * ((minRight + maxRight) * 0.5f) + up * ((minUp + maxUp) * 0.5f)
        + forward * ((minDepth + maxDepth) * 0.5f);
    camera.target = center;
    aspect = std::max(0.1f, aspect);
    if (camera.orthographic) {
        camera.orthoHalfHeight = std::max({ 0.01f, (maxUp - minUp) * 0.5f, (maxRight - minRight) * 0.5f / aspect }) * margin;
        camera.distance = (maxDepth - minDepth) * 0.5f + 1.0f;
        return;
    }
    const float focal = 1.0f / std::tan(camera.verticalFov * 0.5f);
    float required = 0.05f;
    for (const FramePoint& point : points) {
        const Vec3 offset = point.position - center;
        const float depth = dot(offset, forward) - point.radius;
        required = std::max({ required, (std::abs(dot(offset, right)) + point.radius) * focal * margin / aspect - depth,
            (std::abs(dot(offset, up)) + point.radius) * focal * margin - depth, 0.02f - depth });
    }
    camera.distance = required;
}

namespace {

constexpr std::array<ViewPreset, 9> Presets { {
    { "frente", "Frente", 0.0f, 4.0f },
    { "costas", "Costas", 180.0f, 4.0f },
    { "direita", "Lado direito", -90.0f, 4.0f },
    { "esquerda", "Lado esquerdo", 90.0f, 4.0f },
    { "cima", "Cima", 0.0f, 89.0f },
    { "baixo", "Baixo", 0.0f, -89.0f },
    { "34", "3/4 direita", -38.0f, 12.0f },
    { "34e", "3/4 esquerda", 38.0f, 12.0f },
    { "34c", "3/4 costas", -142.0f, 12.0f },
} };

} // namespace

std::span<const ViewPreset> viewPresets() {
    return Presets;
}

const ViewPreset* findViewPreset(std::string_view id) {
    for (const ViewPreset& preset : Presets)
        if (preset.id == id) return &preset;
    return nullptr;
}

} // namespace MatterEngine::Modeling
