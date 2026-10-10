#include "Engine/Objects/ObjectPhysicsLab3D.hpp"

#include "Engine/Objects/ObjectAsset3D.hpp"

#include <algorithm>
#include <cmath>
#include <span>

namespace MatterEngine {

namespace {

constexpr float Step = 1.0f / 120.0f;
constexpr float Pi = 3.14159265f;
constexpr float DegreesToRadians = Pi / 180.0f;
// Parado: abaixo disto (m/s e rad/s) por StillSeconds.
constexpr float StillLinear = 0.04f;
constexpr float StillAngular = 0.12f;
constexpr float StillSeconds = 0.5f;

} // namespace

ObjectPhysicsLab3D::ObjectPhysicsLab3D(PhysicsEngine3D& engine, const MaterialLibrary& materials)
    : m_engine(engine), m_materials(materials) {}

ObjectPhysicsLab3D::~ObjectPhysicsLab3D() {
    stop();
}

void ObjectPhysicsLab3D::stop() {
    m_scene.reset();
    m_objects.clear();
    m_hasBall = false;
    m_bodies.clear();
    m_statics.clear();
}

float ObjectPhysicsLab3D::lowestPoint(const PhysicsBodyState3D& state) const {
    float lowest = 1e9f;
    for (const Modeling::ObjectCollider& collider : m_colliders) {
        const Quaternion orientation = (state.orientation * collider.orientation).normalized();
        const Vec3 center = state.position + state.orientation.rotate(collider.center);
        switch (collider.shape) {
        case Modeling::ObjectColliderShape::Box:
            for (int k = 0; k < 8; ++k) {
                const Vec3 corner { (k & 1) ? collider.halfExtents.x : -collider.halfExtents.x,
                    (k & 2) ? collider.halfExtents.y : -collider.halfExtents.y, (k & 4) ? collider.halfExtents.z : -collider.halfExtents.z };
                lowest = std::min(lowest, (center + orientation.rotate(corner)).z);
            }
            break;
        case Modeling::ObjectColliderShape::Sphere:
            lowest = std::min(lowest, center.z - collider.radius);
            break;
        case Modeling::ObjectColliderShape::Capsule:
        case Modeling::ObjectColliderShape::Cylinder: {
            const Vec3 axis = orientation.rotate({ 0.0f, 0.0f, collider.halfHeight });
            lowest = std::min({ lowest, (center + axis).z - collider.radius, (center - axis).z - collider.radius });
            break;
        }
        }
    }
    return lowest;
}

bool ObjectPhysicsLab3D::start(const Modeling::ProceduralObject& object, const ObjectTestSettings3D& settings, std::string* error) {
    stop();
    m_settings = settings;
    m_metrics = {};
    m_accumulator = 0.0f;
    m_stillSeconds = 0.0f;
    m_touched = false;
    m_ballHit = false;
    m_colliders = object.physics.colliders;
    ObjectBodyData3D data;
    try {
        data = objectBodyData3D(object, m_materials);
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = exception.what();
        return false;
    }
    PhysicsSceneSettings3D sceneSettings;
    sceneSettings.enableContinuousCollision = true;
    m_scene = m_engine.createScene(sceneSettings, m_materials);
    // O chao (a grama nao: o concreto, a referencia dos testes) e a rampa.
    const auto addStatic = [&](Vec3 center, Quaternion orientation, Vec3 half, const char* material) {
        PhysicsBodyDefinition3D body;
        body.motionType = PhysicsMotionType3D::Static;
        body.position = center;
        body.orientation = orientation;
        body.materialId = material;
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = half;
        shape.materialId = material;
        static_cast<void>(m_scene->createBody(body, std::span<const PhysicsShape3D>(&shape, 1)));
        m_statics.push_back({ center, orientation, half });
    };
    addStatic({ 0.0f, 0.0f, -0.25f }, {}, { 12.0f, 12.0f, 0.25f }, "concrete");
    const Vec3 half = object.size * 0.5f;
    const auto spawn = [&](Vec3 position, Quaternion orientation, Vec3 velocity, Vec3 spin) {
        PhysicsBodyDefinition3D body = data.body;
        body.position = position;
        body.orientation = orientation;
        body.linearVelocity = velocity;
        body.angularVelocity = spin;
        body.collisionMode = PhysicsCollisionMode3D::Continuous;
        m_objects.push_back(m_scene->createBody(body, data.shapes));
    };
    switch (settings.kind) {
    case ObjectTestKind3D::Drop: {
        // A base a "height" do chao, inclinada para cair numa quina.
        const Quaternion tilt = (Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, settings.tiltDegrees * DegreesToRadians)
            * Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, settings.tiltDegrees * 0.6f * DegreesToRadians))
                                     .normalized();
        spawn({ 0.0f, 0.0f, std::max(0.0f, settings.height) + half.z + 0.25f * object.size.length() * std::sin(settings.tiltDegrees * DegreesToRadians) }, tilt, {}, {});
        break;
    }
    case ObjectTestKind3D::Throw:
        spawn({ -2.0f, 0.0f, 1.0f + half.z }, {}, { settings.speed, 0.0f, 1.5f }, { 0.0f, settings.spin, settings.spin * 0.3f });
        break;
    case ObjectTestKind3D::Ramp: {
        // A rampa em volta do y: desce para o +x. O objeto parado em cima.
        const float angle = std::clamp(settings.rampDegrees, 0.0f, 60.0f) * DegreesToRadians;
        const Quaternion ramp = Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, angle);
        const Vec3 rampHalf { 2.5f, 1.2f, 0.1f };
        const Vec3 rampCenter { 0.0f, 0.0f, 2.5f * std::sin(angle) + 0.1f };
        addStatic(rampCenter, ramp, rampHalf, "concrete");
        const Vec3 normal = ramp.rotate({ 0.0f, 0.0f, 1.0f });
        m_rampDown = ramp.rotate({ 1.0f, 0.0f, 0.0f });
        const Vec3 top = rampCenter - m_rampDown * 1.6f + normal * (rampHalf.z + half.z + 0.005f);
        spawn(top, ramp, {}, {});
        break;
    }
    case ObjectTestKind3D::Stack: {
        const int count = std::clamp(settings.count, 1, 8);
        for (int k = 0; k < count; ++k) {
            // Um centimetro de desalinhado (sempre o mesmo: o teste repete).
            const float offset = (k % 2 == 0 ? 1.0f : -1.0f) * 0.01f * static_cast<float>(k);
            spawn({ offset, -offset * 0.5f, half.z + static_cast<float>(k) * (object.size.z + 0.002f) }, {}, {}, {});
        }
        // O centro do de cima, de pe (cai se descer meia altura disso).
        m_stackHeight = (static_cast<float>(count) - 0.5f) * object.size.z;
        m_stackPiece = object.size.z;
        break;
    }
    case ObjectTestKind3D::BallShot: {
        spawn({ 0.0f, 0.0f, half.z }, {}, {}, {});
        InflatedBallDefinition3D ball;
        m_ballRadius = ball.profile.radiusMeters;
        ball.position = { -3.0f, 0.0f, std::max(m_ballRadius + 0.01f, std::min(half.z, 0.5f)) };
        ball.linearVelocity = { settings.speed, 0.0f, 0.0f };
        m_ball = m_scene->createInflatedBall(ball);
        m_hasBall = true;
        m_ballSpeedBefore = settings.speed;
        break;
    }
    }
    if (!m_objects.empty()) m_start = m_scene->bodyState(m_objects.front()).position;
    refresh();
    return true;
}

void ObjectPhysicsLab3D::refresh() {
    m_bodies.clear();
    if (!m_scene) return;
    for (const PhysicsBodyHandle3D handle : m_objects) {
        const PhysicsBodyState3D state = m_scene->bodyState(handle);
        m_bodies.push_back({ state.position, state.orientation, false, 0.0f });
    }
    if (m_hasBall) {
        const InflatedBallState3D state = m_scene->inflatedBallState(m_ball);
        m_bodies.push_back({ state.position, state.orientation, true, m_ballRadius });
    }
}

void ObjectPhysicsLab3D::step() {
    m_scene->simulate(Step);
    m_metrics.elapsed += Step;
    bool still = true;
    for (std::size_t k = 0; k < m_objects.size(); ++k) {
        const PhysicsBodyState3D state = m_scene->bodyState(m_objects[k]);
        const float speed = state.linearVelocity.length();
        if (!state.sleeping && (speed > StillLinear || state.angularVelocity.length() > StillAngular)) still = false;
        if (k != 0) continue;
        m_metrics.peakSpeed = std::max(m_metrics.peakSpeed, speed);
        const float lowest = lowestPoint(state);
        if (!m_touched && lowest <= 0.004f) {
            m_touched = true;
            m_metrics.firstImpactSpeed = speed;
        } else if (m_touched) {
            m_metrics.bounceHeight = std::max(m_metrics.bounceHeight, lowest);
        }
        const Vec3 moved = state.position - m_start;
        m_metrics.travel = std::hypot(moved.x, moved.y);
        if (m_settings.kind == ObjectTestKind3D::Ramp) m_metrics.slide = dot(moved, m_rampDown);
        // A face para baixo: o eixo do objeto mais para baixo.
        const Vec3 down = state.orientation.conjugate().rotate({ 0.0f, 0.0f, -1.0f });
        const float ax = std::abs(down.x), ay = std::abs(down.y), az = std::abs(down.z);
        m_metrics.restFace = az >= ax && az >= ay ? (down.z < 0.0f ? "base" : "tampa") : "lado";
        if (m_hasBall) m_metrics.objectSpeedAfter = std::max(m_metrics.objectSpeedAfter, speed);
    }
    if (m_settings.kind == ObjectTestKind3D::Stack && m_objects.size() > 1) {
        const Vec3 bottom = m_scene->bodyState(m_objects.front()).position;
        const Vec3 top = m_scene->bodyState(m_objects.back()).position;
        m_metrics.stackDrift = std::max(m_metrics.stackDrift, std::hypot(top.x - bottom.x, top.y - bottom.y));
        m_metrics.toppled = m_metrics.toppled || top.z < m_stackHeight - 0.5f * m_stackPiece;
    }
    if (m_hasBall) {
        const InflatedBallState3D ball = m_scene->inflatedBallState(m_ball);
        const float speed = ball.linearVelocity.length();
        if (!m_ballHit && ball.contactCount > 0 && ball.position.x > -1.0f) m_ballHit = true;
        // A velocidade dela logo depois do toque: quando ele acaba.
        if (m_ballHit && m_metrics.ballSpeedAfter < 0.0f && ball.contactCount == 0) m_metrics.ballSpeedAfter = speed;
        if (speed > StillLinear * 3.0f) still = false;
    }
    m_stillSeconds = still ? m_stillSeconds + Step : 0.0f;
    if (m_stillSeconds >= StillSeconds && m_metrics.settleTime < 0.0f) {
        m_metrics.settleTime = m_metrics.elapsed - StillSeconds;
        m_metrics.settled = true;
    }
}

void ObjectPhysicsLab3D::advance(float seconds) {
    if (!m_scene) return;
    m_accumulator += std::clamp(seconds, 0.0f, 0.1f);
    while (m_accumulator >= Step) {
        m_accumulator -= Step;
        step();
    }
    refresh();
}

ObjectTestMetrics3D ObjectPhysicsLab3D::run(const Modeling::ProceduralObject& object, const ObjectTestSettings3D& settings,
    float maximumSeconds) {
    if (!start(object, settings)) return {};
    while (m_metrics.elapsed < maximumSeconds && !(m_metrics.settled && m_metrics.elapsed > m_metrics.settleTime + 1.0f)) step();
    refresh();
    return m_metrics;
}

} // namespace MatterEngine
