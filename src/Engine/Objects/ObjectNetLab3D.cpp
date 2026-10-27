#include "Engine/Objects/ObjectNetLab3D.hpp"

#include "Engine/Objects/ObjectAsset3D.hpp"

#include <algorithm>
#include <cmath>
#include <span>

namespace MatterEngine {

namespace {

constexpr float Step = 1.0f / 120.0f;
constexpr float Pi = 3.14159265f;
// Bolas e objetos na cena ao mesmo tempo (os mais antigos saem).
constexpr std::size_t MaximumBalls = 12;
constexpr std::size_t MaximumThrown = 4;
// A bola parada: abaixo disto (m/s) por meio segundo.
constexpr float StillSpeed = 0.15f;

float horizontalLength(Vec3 v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

} // namespace

ObjectNetLab3D::ObjectNetLab3D(PhysicsEngine3D& engine, const MaterialLibrary& materials)
    : m_engine(engine), m_materials(materials) {}

ObjectNetLab3D::~ObjectNetLab3D() {
    stop();
}

void ObjectNetLab3D::stop() {
    m_scene.reset();
    m_net = {};
    m_balls.clear();
    m_thrownBodies.clear();
    m_thrown.clear();
    m_bodies.clear();
    m_rest.clear();
}

bool ObjectNetLab3D::start(const Modeling::ProceduralObject& object, std::string* error) {
    stop();
    if (!object.net) {
        if (error != nullptr) *error = object.id + ": o objeto nao tem rede";
        return false;
    }
    ObjectBodyData3D data;
    try {
        data = objectBodyData3D(object, m_materials);
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = exception.what();
        return false;
    }
    m_object = std::make_unique<Modeling::ProceduralObject>(object);
    m_metrics = {};
    m_accumulator = 0.0f;
    PhysicsSceneSettings3D settings;
    settings.enableContinuousCollision = true;
    m_scene = m_engine.createScene(settings, m_materials);
    // O campo: grama.
    {
        PhysicsBodyDefinition3D ground;
        ground.motionType = PhysicsMotionType3D::Static;
        ground.position = { 0.0f, 0.0f, -0.25f };
        ground.materialId = "grass";
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = { 40.0f, 40.0f, 0.25f };
        shape.materialId = "grass";
        static_cast<void>(m_scene->createBody(ground, std::span<const PhysicsShape3D>(&shape, 1)));
    }
    // A trave com a frente em x = 0 e o pe no chao.
    m_objectPosition = { -0.5f * object.size.x, 0.0f, 0.5f * object.size.z };
    PhysicsBodyDefinition3D body = data.body;
    body.position = m_objectPosition;
    body.motionType = PhysicsMotionType3D::Static;
    static_cast<void>(m_scene->createBody(body, data.shapes));
    // A linha do gol (o fundo dela alinhado com o fundo dos postes), o vao.
    m_goalLineX = 0.0f;
    m_postInnerY = 0.5f * object.size.y;
    m_crossbarBottom = object.size.z;
    float thickest = 0.0f;
    for (const Modeling::ObjectCollider& collider : object.physics.colliders) thickest = std::max(thickest, collider.radius);
    for (const Modeling::ObjectCollider& collider : object.physics.colliders) {
        if (collider.radius < thickest - 1e-4f) continue;
        const Vec3 axis = collider.orientation.rotate({ 0.0f, 0.0f, 1.0f });
        const Vec3 center = m_objectPosition + collider.center;
        if (std::abs(axis.z) > 0.99f) {
            m_goalLineX = center.x - collider.radius;
            m_postInnerY = std::min(m_postInnerY, std::abs(center.y) - collider.radius);
        } else if (std::abs(axis.y) > 0.99f) {
            m_crossbarBottom = std::min(m_crossbarBottom, center.z - collider.radius);
        }
    }
    m_net = m_scene->createNet(objectNetDescription3D(object, m_objectPosition, {}, 0.0f));
    m_scene->settleNet(m_net, 8.0f);
    const std::span<const Vec3> rest = m_scene->netPositions(m_net);
    m_rest.assign(rest.begin(), rest.end());
    refresh();
    return true;
}

void ObjectNetLab3D::reset() {
    if (!m_object) return;
    const Modeling::ProceduralObject object = *m_object;
    start(object);
}

void ObjectNetLab3D::clearBodies() {
    if (!m_scene) return;
    for (const InflatedBallHandle3D ball : m_balls) m_scene->destroyInflatedBall(ball);
    for (const PhysicsBodyHandle3D body : m_thrownBodies) m_scene->destroyBody(body);
    m_balls.clear();
    m_thrownBodies.clear();
    m_thrown.clear();
    refresh();
}

Vec3 ObjectNetLab3D::launchVelocity(Vec3 from, Vec3 target, float speed, Vec3 spin) {
    const InflatedBallProfile3D profile;
    Vec3 horizontal { target.x - from.x, target.y - from.y, 0.0f };
    const float reach = horizontalLength(horizontal);
    if (reach < 1e-4f || speed <= 0.0f) return {};
    horizontal = horizontal * (1.0f / reach);
    const Vec3 gravity { 0.0f, 0.0f, -9.81f };
    // A altura com que a bola chega a frente do gol saindo com a elevacao
    // dada (o ar do modelo da bola).
    const auto arrival = [&](float elevation) {
        Vec3 p = from;
        Vec3 v = horizontal * (speed * std::cos(elevation)) + Vec3 { 0.0f, 0.0f, speed * std::sin(elevation) };
        constexpr float Dt = 1.0f / 600.0f;
        for (int k = 0; k < 6000; ++k) {
            const Vec3 force = InflatedBallModel3D::aerodynamicForce(profile, v, spin);
            v = v + (gravity + force * (1.0f / profile.massKg)) * Dt;
            p = p + v * Dt;
            if (dot(p - from, horizontal) >= reach) break;
        }
        return p.z;
    };
    float low = -0.35f, high = 0.8f;
    for (int k = 0; k < 40; ++k) {
        const float middle = 0.5f * (low + high);
        if (arrival(middle) < target.z) low = middle;
        else high = middle;
    }
    const float elevation = 0.5f * (low + high);
    return horizontal * (speed * std::cos(elevation)) + Vec3 { 0.0f, 0.0f, speed * std::sin(elevation) };
}

void ObjectNetLab3D::shoot(const NetShotSettings3D& shot) {
    if (!m_scene) return;
    InflatedBallDefinition3D ball;
    const float radius = ball.profile.radiusMeters;
    const float angle = shot.angleDegrees * Pi / 180.0f;
    const Vec3 from { m_goalLineX + shot.distance * std::cos(angle), shot.distance * std::sin(angle), radius };
    const float across = std::clamp(shot.aimAcross, -1.0f, 1.0f) * std::max(0.0f, m_postInnerY - radius);
    const float height = radius + std::clamp(shot.aimHeight, 0.0f, 1.0f) * std::max(0.0f, m_crossbarBottom - 2.0f * radius);
    const Vec3 target { 0.0f, across, height };
    // O efeito: em volta da vertical (a curva) e do eixo lateral (por cima).
    Vec3 direction = target - from;
    direction.z = 0.0f;
    const float length = horizontalLength(direction);
    const Vec3 forward = length > 1e-4f ? direction * (1.0f / length) : Vec3 { -1.0f, 0.0f, 0.0f };
    const Vec3 lateral = cross(Vec3 { 0.0f, 0.0f, 1.0f }, forward);
    const Vec3 spin = Vec3 { 0.0f, 0.0f, shot.sideSpin } + lateral * shot.topSpin;
    ball.position = from;
    ball.linearVelocity = launchVelocity(from, target, shot.speed, spin);
    ball.angularVelocity = spin;
    if (m_balls.size() >= MaximumBalls) {
        m_scene->destroyInflatedBall(m_balls.front());
        m_balls.erase(m_balls.begin());
    }
    m_balls.push_back(m_scene->createInflatedBall(ball));
    m_metrics.shots += 1;
    m_metrics.speedAtLine = -1.0f;
    m_metrics.goal = false;
    m_metrics.crossed = false;
    m_metrics.bulge = 0.0f;
    m_metrics.stopTime = -1.0f;
    m_metrics.elapsed = 0.0f;
    m_metrics.netPeakMilliseconds = 0.0f;
    m_metrics.peakStrain = 0.0f;
    m_costSum = 0.0f;
    m_costTicks = 0;
    m_stillSeconds = 0.0f;
    refresh();
}

void ObjectNetLab3D::throwObject(const Modeling::ProceduralObject& object, const NetShotSettings3D& shot) {
    if (!m_scene || object.net) return;
    ObjectBodyData3D data;
    try {
        data = objectBodyData3D(object, m_materials);
    } catch (const std::exception&) {
        return;
    }
    const float angle = shot.angleDegrees * Pi / 180.0f;
    const Vec3 from { m_goalLineX + shot.distance * std::cos(angle), shot.distance * std::sin(angle), 0.5f * object.size.z + 0.4f };
    const float across = std::clamp(shot.aimAcross, -1.0f, 1.0f) * std::max(0.0f, m_postInnerY - 0.5f * object.size.y);
    const float height = 0.5f * object.size.z
        + std::clamp(shot.aimHeight, 0.0f, 1.0f) * std::max(0.0f, m_crossbarBottom - object.size.z);
    // Sem o ar (a caixa): a parabola que chega ao ponto.
    Vec3 horizontal { 0.0f - from.x, across - from.y, 0.0f };
    const float reach = horizontalLength(horizontal);
    horizontal = horizontal * (1.0f / std::max(reach, 1e-4f));
    const float time = reach / std::max(1.0f, shot.speed);
    const float rise = (height - from.z + 0.5f * 9.81f * time * time) / std::max(time, 1e-3f);
    PhysicsBodyDefinition3D body = data.body;
    body.position = from;
    body.linearVelocity = horizontal * shot.speed + Vec3 { 0.0f, 0.0f, rise };
    body.angularVelocity = { 0.6f, -1.2f, 0.3f };
    body.collisionMode = PhysicsCollisionMode3D::Continuous;
    if (m_thrownBodies.size() >= MaximumThrown) {
        m_scene->destroyBody(m_thrownBodies.front());
        m_thrownBodies.erase(m_thrownBodies.begin());
        m_thrown.erase(m_thrown.begin());
    }
    m_thrownBodies.push_back(m_scene->createBody(body, data.shapes));
    m_thrown.push_back(object);
    refresh();
}

void ObjectNetLab3D::advance(float seconds) {
    if (!m_scene) return;
    m_accumulator += std::clamp(seconds, 0.0f, 0.1f);
    while (m_accumulator >= Step) {
        m_accumulator -= Step;
        step();
    }
}

void ObjectNetLab3D::step() {
    if (!m_scene) return;
    m_scene->simulate(Step);
    m_metrics.elapsed += Step;
    if (!m_balls.empty()) {
        const InflatedBallState3D ball = m_scene->inflatedBallState(m_balls.back());
        const float radius = InflatedBallProfile3D {}.radiusMeters;
        const float speed = ball.linearVelocity.length();
        if (!m_metrics.crossed && ball.position.x < 0.0f) {
            m_metrics.crossed = true;
            m_metrics.speedAtLine = speed;
        }
        // Gol: a bola inteira passou do fundo da linha, entre os postes e
        // abaixo do travessao.
        if (!m_metrics.goal && ball.position.x < m_goalLineX - radius && std::abs(ball.position.y) <= m_postInnerY
            && ball.position.z <= m_crossbarBottom)
            m_metrics.goal = true;
        if (m_metrics.crossed && m_metrics.stopTime < 0.0f) {
            m_stillSeconds = speed < StillSpeed ? m_stillSeconds + Step : 0.0f;
            if (m_stillSeconds >= 0.5f) m_metrics.stopTime = m_metrics.elapsed - 0.5f;
        }
    }
    const NetStatus3D status = m_scene->netStatus(m_net);
    if (!status.sleeping) {
        const std::span<const Vec3> positions = m_scene->netPositions(m_net);
        float farthest = 0.0f;
        for (std::size_t i = 0; i < positions.size() && i < m_rest.size(); ++i)
            farthest = std::max(farthest, (positions[i] - m_rest[i]).lengthSquared());
        m_metrics.bulge = std::max(m_metrics.bulge, std::sqrt(farthest));
    }
    m_metrics.netMilliseconds = status.stepMilliseconds;
    m_metrics.netPeakMilliseconds = std::max(m_metrics.netPeakMilliseconds, status.stepMilliseconds);
    m_costSum += status.stepMilliseconds;
    ++m_costTicks;
    m_metrics.netAverageMilliseconds = m_costSum / static_cast<float>(m_costTicks);
    m_metrics.peakStrain = std::max(m_metrics.peakStrain, status.maximumStrain);
    m_metrics.netSleeping = status.sleeping;
    refresh();
}

std::span<const Vec3> ObjectNetLab3D::netPositions() const {
    return m_scene ? m_scene->netPositions(m_net) : std::span<const Vec3> {};
}

NetStatus3D ObjectNetLab3D::netStatus() const {
    return m_scene ? m_scene->netStatus(m_net) : NetStatus3D {};
}

void ObjectNetLab3D::refresh() {
    m_bodies.clear();
    if (!m_scene) return;
    const float radius = InflatedBallProfile3D {}.radiusMeters;
    for (const InflatedBallHandle3D ball : m_balls) {
        const InflatedBallState3D state = m_scene->inflatedBallState(ball);
        m_bodies.push_back({ state.position, state.orientation, true, radius, 0 });
    }
    for (std::size_t k = 0; k < m_thrownBodies.size(); ++k) {
        const PhysicsBodyState3D state = m_scene->bodyState(m_thrownBodies[k]);
        m_bodies.push_back({ state.position, state.orientation, false, 0.0f, k });
    }
}

} // namespace MatterEngine
