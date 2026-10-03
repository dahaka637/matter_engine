#include "Engine/Character/CharacterStateEstimator3D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace MatterEngine {
namespace {

bool finiteVec(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

Vec3 clampLength(Vec3 v, float maximum) {
    const float length = v.length();
    return length > maximum && length > 0.0f ? v * (maximum / length) : v;
}

// Ponto mais baixo do colisor abaixo do centro dele, na orientacao atual.
float lowestOffset(const RagdollLinkDefinition3D& link, Quaternion colliderWorld) {
    const Vec3 up = colliderWorld.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
    if (link.collider.shape == RagdollColliderShape3D::Box) {
        const Vec3 half = link.collider.boxHalfExtents;
        return std::abs(up.x) * half.x + std::abs(up.y) * half.y
            + std::abs(up.z) * half.z;
    }
    return std::abs(up.x) * std::max(0.0f,
        link.collider.lengthMeters * 0.5f - link.collider.radiusMeters)
        + link.collider.radiusMeters;
}

} // namespace

float planarDistanceOutsideHull3D(std::vector<std::array<float, 2>> points,
    float x, float y) {
    if (points.empty()) return 10.0f;
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    const auto cross = [](const std::array<float, 2>& o,
        const std::array<float, 2>& a, const std::array<float, 2>& b) {
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    };
    std::vector<std::array<float, 2>> hull;
    if (points.size() >= 3) {
        for (int pass = 0; pass < 2; ++pass) {
            const std::size_t start = hull.size();
            for (std::size_t i = 0; i < points.size(); ++i) {
                const auto& p = pass == 0 ? points[i]
                    : points[points.size() - 1 - i];
                while (hull.size() >= start + 2
                    && cross(hull[hull.size() - 2], hull.back(), p) <= 0.0f) {
                    hull.pop_back();
                }
                hull.push_back(p);
            }
            hull.pop_back();
        }
    } else {
        hull = points;
    }
    if (hull.size() == 1) return std::hypot(x - hull[0][0], y - hull[0][1]);
    bool inside = hull.size() >= 3;
    float nearest = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const auto& a = hull[i];
        const auto& b = hull[(i + 1) % hull.size()];
        if (cross(a, b, { x, y }) < 0.0f) inside = false;
        const float dx = b[0] - a[0], dy = b[1] - a[1];
        const float length = dx * dx + dy * dy;
        const float u = length > 1e-9f ? std::clamp(((x - a[0]) * dx
            + (y - a[1]) * dy) / length, 0.0f, 1.0f) : 0.0f;
        nearest = std::min(nearest,
            std::hypot(x - (a[0] + u * dx), y - (a[1] + u * dy)));
    }
    return inside ? 0.0f : nearest;
}

void CharacterStateEstimator3D::reset(const RagdollProfile3D& profile) {
    m_state = {};
    m_previousComVelocity = {};
    m_filteredComAcceleration = {};
    m_previousComVelocityValid = false;
    m_linkCount = profile.links.size();
    m_feet = { profile.links.size(), profile.links.size() };
    m_legLink.assign(profile.links.size(), false);
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const std::string& id = profile.links[i].id;
        if (id == "LeftFoot") m_feet[0] = i;
        if (id == "RightFoot") m_feet[1] = i;
        m_legLink[i] = id.find("Thigh") != std::string::npos
            || id.find("Shin") != std::string::npos
            || id.find("Foot") != std::string::npos;
    }
    m_externalForce = {};
    m_sinceContactSupport = { 10.0f, 10.0f };
    m_supportingSeconds = {};
    m_lastContactEvidence = {};
    m_initialized = true;
}

const CharacterPhysicalState3D& CharacterStateEstimator3D::update(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    const std::array<GroundProbeResult3D, 2>& footGround, float deltaTime) {
    if (!m_initialized || m_linkCount != profile.links.size()) reset(profile);
    CharacterPhysicalState3D& out = m_state;
    const Vec3 previousExternal = m_externalForce;
    out = {};
    if (state.links.size() != profile.links.size() || state.links.empty()
        || !std::isfinite(deltaTime) || deltaTime <= 0.0f) {
        return out;
    }
    const float totalMass = std::max(0.1f, profile.totalMassKg);

    // Centro de massa e momento linear pela massa de cada link. A velocidade
    // publicada ja e a do centro de massa do link.
    float fractionSum = 0.0f;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const float fraction = std::max(0.0f, profile.links[i].massFraction);
        const PhysicsBodyState3D& link = state.links[i];
        out.centerOfMassWorld += (link.position
            + link.orientation.rotate(profile.links[i].centerOfMassLocal)) * fraction;
        out.centerOfMassVelocityWorld += link.linearVelocity * fraction;
        fractionSum += fraction;
    }
    if (fractionSum > 0.0f) {
        out.centerOfMassWorld = out.centerOfMassWorld * (1.0f / fractionSum);
        out.centerOfMassVelocityWorld = out.centerOfMassVelocityWorld * (1.0f / fractionSum);
    } else {
        out.centerOfMassWorld = state.links.front().position;
        out.centerOfMassVelocityWorld = state.links.front().linearVelocity;
    }
    if (!finiteVec(out.centerOfMassWorld) || !finiteVec(out.centerOfMassVelocityWorld))
        return out;
    out.massKilograms = totalMass;
    if (m_previousComVelocityValid) {
        const Vec3 raw = (out.centerOfMassVelocityWorld - m_previousComVelocity) * (1.0f / deltaTime);
        if (finiteVec(raw)) {
            out.centerOfMassAccelerationRawWorld = raw;
            m_filteredComAcceleration += (raw - m_filteredComAcceleration) * (deltaTime / (0.04f + deltaTime));
        }
    }
    out.centerOfMassAccelerationWorld = m_filteredComAcceleration;
    m_previousComVelocity = out.centerOfMassVelocityWorld;
    m_previousComVelocityValid = true;

    const PhysicsBodyState3D& root = state.links.front();
    out.rootPositionWorld = root.position;
    out.rootOrientationWorld = root.orientation;
    out.rootLinearVelocityWorld = root.linearVelocity;
    out.rootAngularVelocityWorld = root.angularVelocity;
    out.rootUpright = std::clamp(root.orientation.rotate({ 0.0f, 0.0f, 1.0f }).z, -1.0f, 1.0f);
    out.rootTiltRadians = std::acos(out.rootUpright);

    // Apoio de cada pe: contatos do pe (evento com impulso) ou, sem evento, a
    // sola rente ao chao sondado e quase parada (evidencia geometrica: um pe
    // quase sem carga some dos contatos por alguns ticks).
    std::vector<std::array<float, 2>> support;
    float sumX = 0.0f, sumY = 0.0f;
    float supportHeight = std::numeric_limits<float>::infinity();
    for (std::size_t side = 0; side < 2; ++side) {
        FootSupportEstimate3D& foot = out.feet[side];
        const std::size_t index = m_feet[side];
        if (index >= profile.links.size()) continue;
        foot.linkIndex = static_cast<std::uint32_t>(index);
        const RagdollLinkDefinition3D& link = profile.links[index];
        const PhysicsBodyState3D& body = state.links[index];
        const Quaternion shape = body.orientation * link.collider.localOrientation;
        const Vec3 center = body.position + body.orientation.rotate(link.collider.localPosition);
        const Vec3 half = link.collider.shape == RagdollColliderShape3D::Box
            ? link.collider.boxHalfExtents
            : Vec3 { link.collider.radiusMeters, link.collider.radiusMeters,
                link.collider.radiusMeters };
        std::size_t corner = 0;
        for (const float sx : { -1.0f, 1.0f }) {
            for (const float sy : { -1.0f, 1.0f }) {
                foot.soleCornersWorld[corner++] = center + shape.rotate({ sx * half.x, sy * half.y, -half.z });
            }
        }
        foot.soleCenterWorld = center + shape.rotate({ 0.0f, 0.0f, -half.z });
        foot.soleLowestHeight = center.z - lowestOffset(link, shape);
        const Vec3 linkCom = body.position + body.orientation.rotate(link.centerOfMassLocal);
        foot.soleVelocityWorld = body.linearVelocity
            + cross(body.angularVelocity, foot.soleCenterWorld - linkCom);

        bool estimated = false;
        for (const RagdollContactPoint3D& contact : state.contacts) {
            if (contact.linkIndex != index || contact.normalImpulseNewtonSeconds <= 0.00001f) continue;
            foot.touching = true;
            if (contact.normal.z < 0.40f) continue;
            foot.supporting = true;
            foot.onDynamicBody = foot.onDynamicBody || contact.otherBodyDynamic;
            foot.loadNewtons += contact.normalImpulseNewtonSeconds * contact.normal.z / deltaTime;
            estimated = estimated || contact.impulseEstimated;
        }
        if (foot.supporting) {
            foot.evidence = estimated ? ContactEvidence3D::EstimatedImpulse
                : ContactEvidence3D::SolvedImpulse;
            m_sinceContactSupport[side] = 0.0f;
            m_lastContactEvidence[side] = foot.evidence;
        } else if ((m_sinceContactSupport[side] += deltaTime) < 0.05f
            && foot.soleVelocityWorld.length() < 0.3f) {
            // O evento piscou (pe com pouca carga): segue o ultimo.
            foot.supporting = true;
            foot.heldFromMemory = true;
            foot.evidence = m_lastContactEvidence[side];
        } else if (footGround[side].hasSurface
            && foot.soleLowestHeight < footGround[side].pointWorld.z + 0.015f
            && foot.soleVelocityWorld.length() < 0.3f) {
            // Rente ao chao e parada: um pe em balanco baixo, ainda a 2-3 cm
            // e andando, nao e apoio.
            foot.supporting = true;
            foot.evidence = ContactEvidence3D::Geometric;
        }
        foot.contactObserved = m_sinceContactSupport[side] <= 2.5f * deltaTime;
        m_supportingSeconds[side] = foot.supporting ? m_supportingSeconds[side] + deltaTime : 0.0f;
        foot.supportingSeconds = m_supportingSeconds[side];
        if (!foot.supporting) continue;
        for (const Vec3& p : foot.soleCornersWorld) {
            support.push_back({ p.x, p.y });
            sumX += p.x;
            sumY += p.y;
            if (out.supportPointCount < out.supportPointsWorld.size())
                out.supportPointsWorld[out.supportPointCount++] = p;
        }
        supportHeight = std::min(supportHeight, foot.soleLowestHeight);
    }
    out.supported = !support.empty();
    // Outro segmento (joelho/canela, mao, tronco) apoiado em algo por baixo.
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (contact.linkIndex == m_feet[0] || contact.linkIndex == m_feet[1]
            || contact.normalImpulseNewtonSeconds <= 0.0f || contact.normal.z < 0.62f) continue;
        out.nonFootGroundContact = true;
        break;
    }

    // Ponto de captura de altura constante sobre o apoio atual.
    if (out.supported) {
        out.supportHeight = supportHeight;
        const float height = std::max(0.30f, out.centerOfMassWorld.z - supportHeight);
        out.captureOmega = std::sqrt(9.81f / height);
        out.capturePointWorld = { out.centerOfMassWorld.x + out.centerOfMassVelocityWorld.x / out.captureOmega,
            out.centerOfMassWorld.y + out.centerOfMassVelocityWorld.y / out.captureOmega, supportHeight };
        out.captureOutsideMeters = planarDistanceOutsideHull3D(support,
            out.capturePointWorld.x, out.capturePointWorld.y);
        const float count = static_cast<float>(support.size());
        Vec3 direction { out.capturePointWorld.x - sumX / count,
            out.capturePointWorld.y - sumY / count, 0.0f };
        out.captureDirectionWorld = direction.lengthSquared() > 1e-6f ? direction.normalized() : Vec3 {};
    } else {
        out.capturePointWorld = { out.centerOfMassWorld.x, out.centerOfMassWorld.y, out.centerOfMassWorld.z };
    }

    // Forca externa: contatos do tronco e dos bracos (pernas nao), fora o
    // chao embaixo de maos e tronco. Ate 800 N por tick - pressao de verdade
    // (100-400 N) passa inteira, o pico de uma pancada nao - e filtrada ~0,1 s.
    Vec3 external {};
    bool externalEstimated = false;
    bool externalSeen = false;
    for (const RagdollInteraction3D& contact : state.interactions) {
        if (contact.linkIndex >= profile.links.size()) continue;
        const bool staticGround = contact.otherMotion == RagdollContactMotion3D::Static
            && contact.normalWorld.z > 0.5f;
        if (m_legLink[contact.linkIndex] || staticGround) continue;
        external += contact.normalWorld * (contact.normalImpulseNewtonSeconds / deltaTime);
        externalEstimated = externalEstimated || contact.impulseEstimated;
        externalSeen = true;
    }
    external.z = 0.0f;
    m_externalForce = previousExternal + (clampLength(external, 800.0f) - previousExternal)
        * (1.0f - std::exp(-deltaTime / 0.10f));
    out.externalForceWorld = m_externalForce;
    out.externalForceEvidence = !externalSeen ? ContactEvidence3D::None
        : externalEstimated ? ContactEvidence3D::EstimatedImpulse : ContactEvidence3D::SolvedImpulse;
    out.valid = true;
    return out;
}

} // namespace MatterEngine
