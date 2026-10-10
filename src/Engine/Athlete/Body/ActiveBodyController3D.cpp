#include "Engine/Athlete/Body/ActiveBodyController3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

float component(Vec3 value, std::size_t axis) { return axis == 0 ? value.x : axis == 1 ? value.y : value.z; }

Vec3 horizontal(Vec3 value) { return { value.x, value.y, 0.0f }; }

Vec3 clampLength(Vec3 value, float maximum) {
    const float length = value.length();
    return length > maximum && length > 1e-9f ? value * (maximum / length) : value;
}

// O vetor de rotacao (eixo vezes angulo) de um quaternion, pelo caminho curto.
Vec3 rotationVector(Quaternion q) {
    q = q.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const Vec3 v { q.x, q.y, q.z };
    const float n = v.length();
    if (n < 1e-7f) return {};
    return v * (2.0f * std::atan2(n, q.w) / n);
}

} // namespace

void ActiveBodyController3D::reset(const RagdollProfile3D& profile) {
    m_region.assign(profile.links.size(), 1);
    m_parent.assign(profile.links.size(), -1);
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const std::string& id = profile.links[i].id;
        m_parent[i] = profile.links[i].parentIndex;
        if (id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
            || id.find("Foot") != std::string::npos)
            m_region[i] = 0;
        else if (id.find("Arm") != std::string::npos || id.find("Forearm") != std::string::npos
            || id.find("Hand") != std::string::npos)
            m_region[i] = 2;
    }
    m_previousCoordinates.clear();
    m_havePrevious = false;
    m_haveRoot = false;
    m_compliance = 1.0f;
    m_hitLocal = 0.0f;
    m_assistImpulse = 0.0f;
    enter();
}

void ActiveBodyController3D::enter(bool fromPhysical) {
    m_handoverPending = fromPhysical;
    m_handoverLeft = 0.0f;
    m_seconds = 0.0f;
    m_calmSeconds = 0.0f;
    m_closeSeconds = 0.0f;
    m_tiltSeconds = 0.0f;
    m_havePrevious = false;
    m_haveRoot = false;
}

void ActiveBodyController3D::update(const RagdollProfile3D& profile, const RagdollState3D& state,
    const BodyObservation3D& observation, const ActiveBodyTarget3D& target, const ActiveBodySettings3D& settings,
    float deltaTime, ActiveBodyOutput3D& output) {
    output = {};
    if (m_region.size() != profile.links.size()) {
        // (A primeira vez: o reset nao pode apagar a passagem pedida no enter.)
        const bool pending = m_handoverPending;
        reset(profile);
        m_handoverPending = pending;
    }
    if (!observation.valid || target.pose == nullptr || target.coordinates == nullptr
        || target.pose->linkPositions.size() != profile.links.size()
        || target.coordinates->size() != profile.links.size())
        return;
    const float dt = std::max(deltaTime, 1e-4f);
    if (m_handoverPending) {
        m_handoverLeft = settings.handoverSeconds;
        m_handoverPending = false;
    }
    const float handover = settings.handoverSeconds > 0.0f
        ? std::clamp(m_handoverLeft / settings.handoverSeconds, 0.0f, 1.0f) : 0.0f;
    m_handoverLeft = std::max(0.0f, m_handoverLeft - dt);
    m_seconds += dt;
    const std::vector<Vec3>& coordinates = *target.coordinates;

    // A velocidade e a aceleracao da raiz-alvo (a pose anda com o plano).
    Vec3 rootVelocity, rootAngularVelocity;
    if (m_haveRoot) {
        rootVelocity = (target.rootPosition - m_previousRoot) * (1.0f / dt);
        rootAngularVelocity = rotationVector(target.rootOrientation * m_previousRootOrientation.conjugate()) * (1.0f / dt);
        // (Filtrada e limitada: a capsula indo atras do corpo e o peso
        // mudando de pe mexem a raiz-alvo, e isso nao e intencao.)
        const Vec3 acceleration = (rootVelocity - m_previousRootVelocity) * (1.0f / dt);
        m_rootAcceleration += (acceleration - m_rootAcceleration) * std::min(1.0f, dt / settings.rootFeedforwardSeconds);
        m_rootAcceleration = clampLength(m_rootAcceleration, settings.maximumRootFeedforward);
    } else {
        rootVelocity = observation.pelvisVelocity;
        m_rootAcceleration = {};
    }

    // Pancada: o tonus e os ganhos caem pelo impulso e voltam com o tempo; a
    // regiao atingida cede mais.
    if (observation.externalImpulse > settings.impactMinimum) {
        const float drop = std::clamp((observation.externalImpulse - settings.impactMinimum)
                / std::max(0.1f, settings.impactFull - settings.impactMinimum), 0.0f, 1.0f)
            * settings.impactMaximumDrop;
        m_compliance = std::min(m_compliance, 1.0f - drop);
        if (drop >= m_hitLocal) m_hitLink = observation.strongestExternalLink;
        m_hitLocal = std::max(m_hitLocal, drop);
    }
    const float recover = dt / std::max(0.05f, settings.impactRecoverSeconds);
    m_compliance = std::min(1.0f, m_compliance + recover);
    m_hitLocal = std::max(0.0f, m_hitLocal - recover);
    output.compliance = m_compliance;

    // Musculos: a pose-alvo e a velocidade dela em cada eixo.
    output.drives.reserve(profile.links.size() * 3);
    const float toneByCompliance = 0.55f + 0.45f * m_compliance;
    for (std::size_t i = 1; i < profile.links.size(); ++i) {
        const float regionTone = m_region[i] == 0 ? settings.legTone : m_region[i] == 2 ? settings.armTone
            : settings.torsoTone;
        float tone = regionTone * toneByCompliance;
        const bool nearHit = i == m_hitLink || static_cast<int>(m_hitLink) == m_parent[i]
            || (m_hitLink < m_parent.size() && m_parent[m_hitLink] == static_cast<int>(i));
        if (nearHit) tone *= 1.0f - 0.6f * m_hitLocal;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!profile.links[i].inboundJoint.axes[axis].enabled) continue;
            RagdollDriveTarget3D drive;
            drive.linkIndex = static_cast<std::uint32_t>(i);
            drive.axis = static_cast<RagdollAxis3D>(axis);
            drive.positionRadians = component(coordinates[i], axis);
            if (m_havePrevious && i < m_previousCoordinates.size())
                drive.velocityRadiansPerSecond = std::clamp(
                    (component(coordinates[i], axis) - component(m_previousCoordinates[i], axis)) / dt
                        * settings.velocityFeedforward,
                    -settings.maximumJointVelocity, settings.maximumJointVelocity);
            drive.stiffnessScale = tone;
            drive.dampingScale = std::sqrt(tone);
            drive.maximumTorqueScale = tone;
            drive.gravityCompensationScale = 1.0f;
            output.drives.push_back(drive);
        }
    }

    // A raiz: a pelve ao plano, pelos pes apoiados.
    std::vector<Vec3> centers(profile.links.size());
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        centers[i] = state.links[i].position + state.links[i].orientation.rotate(profile.links[i].centerOfMassLocal);
    const float mass = observation.massKilograms;
    const float omega = settings.rootFrequency * (0.4f + 0.6f * m_compliance);
    const Vec3 positionError = target.rootPosition - observation.pelvisPosition;
    const Vec3 velocityError = rootVelocity - observation.pelvisVelocity;
    const Vec3 rootAcceleration = clampLength(m_rootAcceleration + positionError * (omega * omega)
            + velocityError * (2.0f * settings.rootDamping * omega),
        settings.maximumRootAcceleration);
    const Vec3 rootForce = rootAcceleration * mass;
    const Vec3 orientationError = rotationVector(target.rootOrientation * observation.pelvisOrientation.conjugate());
    const float turn = settings.orientationFrequency * (0.4f + 0.6f * m_compliance);
    const Vec3 angular = orientationError * (turn * turn)
        + (rootAngularVelocity - observation.pelvisAngularVelocity) * (2.0f * settings.orientationDamping * turn);
    const Vec3 rootTorque { angular.x * settings.bodyInertiaHorizontal, angular.y * settings.bodyInertiaHorizontal,
        angular.z * settings.bodyInertiaVertical };
    SupportWrenchRequest3D request;
    request.bodyLink = 0;
    request.bodyPoint = centers.front();
    request.force = rootForce;
    request.torque = rootTorque;
    request.weightNewtons = mass * 9.81f;
    SupportWrenchResult3D routed = routeSupportWrench3D(request, observation, centers, settings.support);
    output.wrenches = std::move(routed.wrenches);
    output.requestedForce = rootForce;
    output.deliveredForce = routed.deliveredForce;
    output.droppedForce = routed.droppedForce;
    output.supportCount = observation.supportCount;
    // Ajuda de jogo: o que o chao nao deu, no plano, ate o teto, com pe apoiado
    // e so com o plano andando (a resposta da corrida): parado nao ha ajuda -
    // o corpo empurrado fica onde foi e a pose vai ate ele.
    const float travel = std::clamp(target.intendedSpeed / std::max(0.1f, settings.assistFullSpeed), 0.0f, 1.0f);
    const float assistCap = std::max(settings.gameplayAssistAcceleration * travel,
        settings.handoverAssistAcceleration * handover);
    if (observation.supportCount > 0 && assistCap > 0.0f) {
        output.assistForce = clampLength(horizontal(routed.droppedForce), mass * assistCap);
        m_assistImpulse += output.assistForce.length() * dt;
    }
    output.assistImpulseTotal = m_assistImpulse;

    // Os erros contra a pose-alvo.
    output.targetRoot = target.rootPosition;
    output.rootError = positionError.length();
    output.orientationError = orientationError.length();
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        output.linkError = std::max(output.linkError, (state.links[i].position - target.pose->linkPositions[i]).length());

    // Nocaute: postura perdida, corpo baixo demais, pancada forte.
    const bool tilted = observation.pelvisUp.z < std::cos(settings.knockoutPelvisTilt)
        || observation.chestUp.z < std::cos(settings.knockoutChestTilt);
    m_tiltSeconds = tilted ? m_tiltSeconds + dt : 0.0f;
    float floor = observation.feet[0].soleHeight;
    for (const FootObservation3D& foot : observation.feet)
        if (foot.valid) floor = std::min(floor, foot.soleHeight);
    const float height = observation.pelvisPosition.z - floor;
    if (m_tiltSeconds > settings.knockoutSeconds) {
        output.knockout = true;
        output.knockoutReason = "postura";
    } else if (height < settings.knockoutHeightShare * profile.standingRootHeightMeters) {
        output.knockout = true;
        output.knockoutReason = "corpo baixo";
    } else if (observation.externalImpulse > settings.knockoutImpulse) {
        output.knockout = true;
        output.knockoutReason = "pancada forte";
    }

    // Pronto para o animado: calmo e colado a pose-alvo.
    m_calmSeconds = observation.externalImpulse < 0.3f ? m_calmSeconds + dt : 0.0f;
    const bool close = output.rootError < settings.returnRootError
        && output.orientationError < settings.returnOrientationError
        && output.linkError < settings.returnLinkError
        && (observation.pelvisVelocity - rootVelocity).length() < settings.returnVelocityError
        && m_compliance > 0.9f;
    m_closeSeconds = close ? m_closeSeconds + dt : 0.0f;
    output.readyToAnimate = m_seconds > settings.returnMinimumSeconds && m_calmSeconds > settings.returnCalmSeconds
        && m_closeSeconds > 0.08f;

    m_previousCoordinates = coordinates;
    m_havePrevious = true;
    m_previousRoot = target.rootPosition;
    m_previousRootOrientation = target.rootOrientation;
    m_previousRootVelocity = rootVelocity;
    m_haveRoot = true;
}

} // namespace MatterEngine
