#include "Engine/Athlete/Modes/ContactClasses3D.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace MatterEngine {

namespace {

bool finitePositive(float value) { return std::isfinite(value) && value > 0.0f; }

bool isIntentional(const ContactDeclarations3D& declared, std::uint32_t link) {
    return link < 64 && (declared.intentionalLinks >> link & 1ull) != 0;
}

// O impulso passa nos limiares da parte? (Bracos e maos so pelo corpo: a mao
// encosta em coisas o tempo todo.)
struct Strength {
    float body = 0.0f;
    float link = 0.0f;
    bool impact = false;
};

bool moving(RagdollContactMotion3D motion) { return motion != RagdollContactMotion3D::Static; }

float bodyThreshold(RagdollContactMotion3D motion, const ContactClassParams3D& params) {
    return moving(motion) ? params.bodyDeltaV : params.staticBodyDeltaV;
}

Strength strengthOf(float impulse, BodyRegion3D region, float linkMass, float bodyLimit, const ContactBody3D& body,
    const ContactClassParams3D& params) {
    Strength out;
    out.body = impulse / body.massKg;
    out.link = linkMass > 0.0f ? impulse / linkMass : 0.0f;
    out.impact = out.body >= bodyLimit || (region != BodyRegion3D::Arm && out.link >= params.linkDeltaV);
    return out;
}

// Antes da forca: apoio, chao, contato da acao ou aproximacao lenta.
ContactClass3D preClassify(const RagdollInteraction3D& contact, const ContactBody3D& body,
    const ContactDeclarations3D& declared, float baseHeight, const ContactClassParams3D& params, bool& candidate) {
    candidate = false;
    if (contact.linkIndex >= body.regions.size()) return ContactClass3D::Graze;
    if (isIntentional(declared, contact.linkIndex)) return ContactClass3D::Intentional;
    const BodyRegion3D region = body.regions[contact.linkIndex];
    const float above = contact.positionWorld.z - baseHeight;
    const bool facingUp = contact.normalWorld.z >= params.supportNormalZ;
    if (region == BodyRegion3D::Foot
        && (contact.normalWorld.z > params.footSupportNormalZ || above < params.footStepBandMeters))
        return ContactClass3D::Support;
    if (facingUp && above <= params.groundBandMeters) return ContactClass3D::Ground;
    if (contactApproachSpeed3D(contact) < params.grazeSpeed) return ContactClass3D::Graze;
    candidate = true;
    return ContactClass3D::Graze;
}

} // namespace

bool ContactClassParams3D::valid() const {
    return supportNormalZ > 0.0f && supportNormalZ < 1.0f && footStepBandMeters >= 0.0f && footStepBandMeters <= 1.0f
        && footSupportNormalZ >= -1.0f && footSupportNormalZ < 1.0f && groundBandMeters >= 0.0f
        && groundBandMeters <= 0.3f && grazeSpeed >= 0.0f && grazeSpeed < 5.0f && finitePositive(bodyDeltaV)
        && finitePositive(staticBodyDeltaV) && finitePositive(linkDeltaV) && finitePositive(penetrationMeters) && penetrationMeters < 0.2f;
}

BodyRegion3D bodyRegion3D(const std::string& linkId) {
    const auto has = [&](const char* part) { return linkId.find(part) != std::string::npos; };
    if (has("Foot") || has("Toe")) return BodyRegion3D::Foot;
    // (O antebraco e "Forearm": o "arm" minusculo tambem conta.)
    if (has("Hand") || has("Arm") || has("arm")) return BodyRegion3D::Arm;
    if (has("Thigh") || has("Shin")) return BodyRegion3D::Leg;
    return BodyRegion3D::Trunk;
}

ContactBody3D contactBody3D(const RagdollProfile3D& profile) {
    ContactBody3D body;
    body.massKg = profile.totalMassKg;
    body.regions.reserve(profile.links.size());
    body.linkMassKg.reserve(profile.links.size());
    for (const auto& link : profile.links) {
        body.regions.push_back(bodyRegion3D(link.id));
        body.linkMassKg.push_back(std::max(0.0f, link.massFraction) * profile.totalMassKg);
    }
    return body;
}

float contactApproachSpeed3D(const RagdollInteraction3D& contact) {
    return std::max(0.0f, -dot(contact.relativeVelocityWorld, contact.normalWorld));
}

ContactClass3D classifyInteraction3D(const RagdollInteraction3D& contact, const ContactBody3D& body,
    const ContactDeclarations3D& declared, float baseHeight, const ContactClassParams3D& params) {
    bool candidate = false;
    const ContactClass3D kind = preClassify(contact, body, declared, baseHeight, params, candidate);
    if (!candidate) return kind;
    const Strength strength = strengthOf(contact.normalImpulseNewtonSeconds, body.regions[contact.linkIndex],
        body.linkMassKg[contact.linkIndex], bodyThreshold(contact.otherMotion, params), body, params);
    return strength.impact ? ContactClass3D::Impact : ContactClass3D::Graze;
}

ContactClass3D classifyPenetration3D(const RagdollWorldContact3D& contact, const ContactBody3D& body, float baseHeight,
    const ContactClassParams3D& params) {
    if (contact.linkIndex >= body.regions.size()) return ContactClass3D::Graze;
    const BodyRegion3D region = body.regions[contact.linkIndex];
    const float above = contact.pointWorld.z - baseHeight;
    const bool facingUp = contact.normalWorld.z >= params.supportNormalZ;
    if (region == BodyRegion3D::Foot
        && (contact.normalWorld.z > params.footSupportNormalZ || above < params.footStepBandMeters))
        return ContactClass3D::Support;
    if (facingUp && above <= params.groundBandMeters) return ContactClass3D::Ground;
    // Os bracos e as maos nao viram impacto por penetracao (a mao encosta na
    // parede, no chao, num boneco o tempo todo; o goleiro vai usa-las).
    if (region == BodyRegion3D::Arm) return ContactClass3D::Graze;
    return contact.depthMeters >= params.penetrationMeters ? ContactClass3D::Impact : ContactClass3D::Graze;
}

ContactAssessment3D assessContacts3D(std::span<const RagdollInteraction3D> interactions,
    std::span<const RagdollWorldContact3D> penetrations, const ContactBody3D& body,
    const ContactDeclarations3D& declared, float baseHeight, const ContactClassParams3D& params,
    float pushImpulse) {
    ContactAssessment3D out;
    if (!body.valid()) return out;
    const float push = std::isfinite(pushImpulse) ? std::max(0.0f, pushImpulse) : 0.0f;
    // Os candidatos (vieram contra o corpo com forca): a soma do tick contra o
    // limiar do corpo (o que se move e o mundo parado, cada um com o seu) e,
    // por contato, o Dv do link.
    float movingSum = 0.0f;
    float staticSum = 0.0f;
    for (const RagdollInteraction3D& contact : interactions) {
        bool candidate = false;
        const ContactClass3D kind = preClassify(contact, body, declared, baseHeight, params, candidate);
        if (kind == ContactClass3D::Intentional) {
            out.intentional = true;
            out.intentionalImpulse += contact.normalImpulseNewtonSeconds;
        }
        if (!candidate) continue;
        (moving(contact.otherMotion) ? movingSum : staticSum) += contact.normalImpulseNewtonSeconds;
    }
    movingSum += push;
    const bool movingImpact = movingSum / body.massKg >= params.bodyDeltaV;
    if (push > 0.0f && movingImpact) {
        out.impact = true;
        out.fromPush = true;
        out.pushImpulse = push;
    }
    const bool staticImpact = staticSum / body.massKg >= params.staticBodyDeltaV;
    float strongest = -1.0f;
    for (const RagdollInteraction3D& contact : interactions) {
        bool candidate = false;
        preClassify(contact, body, declared, baseHeight, params, candidate);
        if (!candidate) continue;
        const BodyRegion3D region = body.regions[contact.linkIndex];
        const float linkMass = body.linkMassKg[contact.linkIndex];
        const float impulse = contact.normalImpulseNewtonSeconds;
        const bool byLink = region != BodyRegion3D::Arm && linkMass > 0.0f && impulse / linkMass >= params.linkDeltaV;
        const bool bySum = moving(contact.otherMotion) ? movingImpact : staticImpact;
        if (!byLink && !bySum) {
            ++out.grazes;
            continue;
        }
        out.impact = true;
        out.impactImpulse += impulse;
        out.linkDeltaV = std::max(out.linkDeltaV, linkMass > 0.0f ? impulse / linkMass : 0.0f);
        if (impulse > strongest) {
            strongest = impulse;
            out.link = contact.linkIndex;
            out.fromPenetration = false;
        }
    }
    out.bodyDeltaV = std::max(movingImpact ? movingSum : 0.0f, staticImpact ? staticSum : 0.0f) / body.massKg;
    for (const RagdollWorldContact3D& contact : penetrations) {
        const ContactClass3D kind = classifyPenetration3D(contact, body, baseHeight, params);
        if (kind == ContactClass3D::Graze) ++out.grazes;
        if (kind != ContactClass3D::Impact) continue;
        out.penetrationMeters = std::max(out.penetrationMeters, contact.depthMeters);
        if (!out.impact) {
            out.link = contact.linkIndex;
            out.fromPenetration = true;
        }
        out.impact = true;
    }
    return out;
}

float ImpulseWindow3D::add(float impulse, float deltaTime, float timeConstantSeconds) {
    const float keep = timeConstantSeconds > 0.0f ? std::exp(-std::max(0.0f, deltaTime) / timeConstantSeconds) : 0.0f;
    value = value * keep + (std::isfinite(impulse) ? std::max(0.0f, impulse) : 0.0f);
    return value;
}

const char* contactClassName3D(ContactClass3D value) {
    switch (value) {
    case ContactClass3D::Support: return "apoio";
    case ContactClass3D::Ground: return "chao";
    case ContactClass3D::Graze: return "raspao";
    case ContactClass3D::Impact: return "impacto";
    case ContactClass3D::Intentional: return "intencional";
    }
    return "?";
}

} // namespace MatterEngine
