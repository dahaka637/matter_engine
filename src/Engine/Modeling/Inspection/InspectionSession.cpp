#include "Engine/Modeling/Inspection/InspectionSession.hpp"

#include "Engine/Modeling/Inspection/ViewFraming.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

namespace {

constexpr float InteractionHoldSeconds = 0.18f;

float wrapAngle(float angle) {
    while (angle > Pi) angle -= 2.0f * Pi;
    while (angle < -Pi) angle += 2.0f * Pi;
    return angle;
}

} // namespace

void InspectionSession::setPose(std::optional<SkeletonPose> pose) {
    m_pose = std::move(pose);
    m_posedPositions.clear();
    if (m_pose && m_model && m_model->mesh.hasSkin())
        m_posedPositions = m_model->mesh.posedPositions(skinningTransforms(m_model->skeleton, *m_pose));
    m_dirty = true;
}

InspectionScene InspectionSession::scene() const {
    InspectionScene scene;
    if (!m_model) return scene;
    scene.skeleton = &m_model->skeleton;
    scene.mesh = &m_model->mesh;
    scene.regions = &m_model->regions;
    scene.jointRegion = m_model->jointRegion;
    scene.surface = m_model->surface.get();
    scene.kit = m_model->kit.get();
    if (m_pose) {
        scene.pose = &*m_pose;
        if (!m_posedPositions.empty()) scene.meshPositions = m_posedPositions;
    }
    return scene;
}

void InspectionSession::setModel(std::shared_ptr<const InspectionModel> model) {
    const bool first = !m_model;
    m_model = std::move(model);
    ++m_modelGeneration;
    if (!m_model) return;
    if (m_region >= static_cast<int>(m_model->regions.regions.size())) m_region = 0;
    // Recarregar o modelo nao mexe na camera (o usuario continua onde estava)
    // nem na pose (a malha nova deforma igual).
    if (first) selectRegion(m_region, false);
    m_style.focusRegion = m_region;
    if (m_pose && m_pose->joints.size() != m_model->skeleton.joints.size()) m_pose.reset();
    setPose(m_pose);
    m_dirty = true;
}

void InspectionSession::selectRegion(int region, bool animate) {
    if (!m_model) return;
    m_region = std::clamp(region, 0, std::max(0, static_cast<int>(m_model->regions.regions.size()) - 1));
    m_style.focusRegion = m_region;
    const bool orthographic = m_goal.orthographic;
    m_goal = frameRegion(scene(), m_region, m_aspect, nullptr, orthographic);
    if (!animate) m_camera = m_goal;
    m_animating = animate;
    m_userAdjusted = false;
    m_dirty = true;
}

void InspectionSession::applyView(const ViewPreset& view, bool animate) {
    if (!m_model) return;
    const bool orthographic = m_goal.orthographic;
    m_goal = frameRegion(scene(), m_region, m_aspect, &view, orthographic);
    if (!animate) m_camera = m_goal;
    m_animating = animate;
    m_userAdjusted = false;
    m_dirty = true;
}

void InspectionSession::resetView(bool animate) {
    selectRegion(m_region, animate);
}

void InspectionSession::orbit(float deltaX, float deltaY) {
    m_goal.yaw = wrapAngle(m_goal.yaw - deltaX * 0.008f);
    m_goal.pitch = std::clamp(m_goal.pitch + deltaY * 0.006f, -1.55f, 1.55f);
    m_camera.yaw = m_goal.yaw;
    m_camera.pitch = m_goal.pitch;
    interacted();
}

void InspectionSession::pan(float deltaX, float deltaY, int viewportHeight) {
    const float height = static_cast<float>(std::max(1, viewportHeight));
    const float metersPerPixel = m_goal.orthographic
        ? 2.0f * m_goal.orthoHalfHeight / height
        : 2.0f * m_goal.distance * std::tan(m_goal.verticalFov * 0.5f) / height;
    const Vec3 move = m_goal.right() * (-deltaX * metersPerPixel) + m_goal.up() * (deltaY * metersPerPixel);
    m_goal.target += move;
    m_camera.target = m_goal.target;
    interacted();
}

void InspectionSession::zoom(float wheelSteps) {
    const float factor = std::pow(0.87f, wheelSteps);
    m_goal.distance = std::clamp(m_goal.distance * factor, 0.04f, 30.0f);
    m_goal.orthoHalfHeight = std::clamp(m_goal.orthoHalfHeight * factor, 0.005f, 10.0f);
    m_camera.distance = m_goal.distance;
    m_camera.orthoHalfHeight = m_goal.orthoHalfHeight;
    interacted();
}

bool InspectionSession::click(float x, float y, int width, int height) {
    if (!m_model) return false;
    const int joint = pickJoint(scene(), m_style, m_camera, width, height, x, y, 14.0f);
    if (joint < 0 || static_cast<std::size_t>(joint) >= m_model->jointRegion.size()) return false;
    const int region = m_model->jointRegion[static_cast<std::size_t>(joint)];
    if (region < 0) return false;
    selectRegion(region, true);
    return true;
}

void InspectionSession::keepFast() {
    m_interactionSeconds = std::max(m_interactionSeconds, InteractionHoldSeconds);
}

void InspectionSession::interacted() {
    m_animating = false;
    m_userAdjusted = true;
    m_interactionSeconds = InteractionHoldSeconds;
    m_dirty = true;
}

void InspectionSession::update(float deltaSeconds) {
    if (m_interactionSeconds > 0.0f) {
        m_interactionSeconds -= deltaSeconds;
        if (m_interactionSeconds <= 0.0f) m_dirty = true;
    }
    if (!m_animating) return;
    const float alpha = 1.0f - std::exp(-std::max(0.0f, deltaSeconds) * 9.0f);
    m_camera.target = lerp(m_camera.target, m_goal.target, alpha);
    m_camera.yaw = wrapAngle(m_camera.yaw + wrapAngle(m_goal.yaw - m_camera.yaw) * alpha);
    m_camera.pitch += (m_goal.pitch - m_camera.pitch) * alpha;
    m_camera.distance = std::exp(std::log(m_camera.distance) + (std::log(m_goal.distance) - std::log(m_camera.distance)) * alpha);
    m_camera.orthoHalfHeight = std::exp(std::log(m_camera.orthoHalfHeight)
        + (std::log(m_goal.orthoHalfHeight) - std::log(m_camera.orthoHalfHeight)) * alpha);
    m_camera.orthographic = m_goal.orthographic;
    const bool settled = distance(m_camera.target, m_goal.target) < 1e-4f
        && std::abs(wrapAngle(m_goal.yaw - m_camera.yaw)) < 1e-3f && std::abs(m_goal.pitch - m_camera.pitch) < 1e-3f
        && std::abs(m_goal.distance - m_camera.distance) < 1e-4f * std::max(1.0f, m_goal.distance);
    if (settled) {
        m_camera = m_goal;
        m_animating = false;
    }
    m_interactionSeconds = std::max(m_interactionSeconds, m_animating ? InteractionHoldSeconds : 0.0f);
    m_dirty = true;
}

InspectionStyle& InspectionSession::editStyle() {
    m_dirty = true;
    return m_style;
}

void InspectionSession::setOrthographic(bool orthographic) {
    if (m_goal.orthographic == orthographic) return;
    m_goal.orthographic = orthographic;
    m_camera.orthographic = orthographic;
    // A mesma altura visivel no alvo nas duas projecoes.
    const float halfHeight = m_goal.distance * std::tan(m_goal.verticalFov * 0.5f);
    if (orthographic) {
        m_goal.orthoHalfHeight = halfHeight;
        m_camera.orthoHalfHeight = halfHeight;
    }
    m_dirty = true;
}

bool InspectionSession::render(int width, int height) {
    width = std::max(16, width);
    height = std::max(16, height);
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const bool resized = width != m_image.width() || height != m_image.height();
    if (resized && std::abs(aspect - m_aspect) > 1e-3f) {
        m_aspect = aspect;
        // O enquadramento depende da proporcao: sem o usuario ter mexido,
        // reenquadra no mesmo angulo.
        if (!m_userAdjusted && m_model) {
            framePoints(m_goal, regionPoints(scene(), m_region), m_aspect,
                m_model->regions.regions.empty() ? 1.2f
                    : m_model->regions.regions[static_cast<std::size_t>(m_region)].margin);
            if (!m_animating) m_camera = m_goal;
        }
    }
    const bool fast = m_interactionSeconds > 0.0f;
    if (!m_dirty && !resized && fast == m_lastFast) return false;
    InspectionStyle style = m_style;
    if (fast) style.supersample = 1;
    InspectionScene current = scene();
    if (m_model && style.facet < 0.999f) {
        if (!m_shading || m_shadingGeneration != m_modelGeneration || std::abs(m_shading->creaseDegrees() - style.creaseAngle) > 1e-3f) {
            m_shading = std::make_shared<const ShadingNormals>(m_model->mesh, style.creaseAngle);
            m_shadingGeneration = m_modelGeneration;
        }
        current.shading = m_shading.get();
    }
    m_image = renderInspection(current, style, m_camera, width, height);
    m_lastFast = fast;
    m_dirty = false;
    return true;
}

} // namespace MatterEngine::Modeling
