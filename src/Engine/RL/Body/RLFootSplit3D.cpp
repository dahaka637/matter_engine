#include "Engine/RL/Body/RLFootSplit3D.hpp"

#include "Engine/Geometry/ConvexHull3D.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <stdexcept>

namespace MatterEngine {
namespace {

std::size_t findLink(const RagdollProfile3D& profile, const std::string& id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return i;
    throw std::runtime_error("divisao do pe: o perfil nao tem o link " + id);
}

// A casca de um lado do corte (o x do colisor menor ou maior que cut): os
// vertices daquele lado e os pontos onde as arestas cruzam o plano.
ConvexHull3D halfHull(const RagdollCapsuleDefinition3D& collider, float cut, bool front,
    const std::vector<Vec3>& crossings) {
    std::vector<Vec3> points = crossings;
    for (const Vec3& p : collider.hullPoints)
        if (front ? p.x >= cut : p.x <= cut) points.push_back(p);
    return buildConvexHull3D(points, 0.0f, 128);
}

} // namespace

void splitRagdollToes3D(RagdollProfile3D& profile, const RLToeSettings3D& settings, std::vector<std::string>* report) {
    if (!(settings.jointFromHeel > 0.3f && settings.jointFromHeel < 0.95f))
        throw std::runtime_error("divisao do pe: jointFromHeel fora de 0,3..0,95");
    if (!(settings.massShare > 0.0f && settings.massShare < 0.6f))
        throw std::runtime_error("divisao do pe: massShare fora de 0..0,6");
    for (const char* side : { "Left", "Right" }) {
        const std::size_t footIndex = findLink(profile, std::string(side) + "Foot");
        // (A copia: o push_back abaixo pode mover os links.)
        RagdollLinkDefinition3D foot = profile.links[footIndex];
        const RagdollCapsuleDefinition3D& boot = foot.collider;
        if (boot.shape != RagdollColliderShape3D::ConvexHull || boot.hullPoints.size() < 8)
            throw std::runtime_error("divisao do pe: o pe " + foot.id + " nao e uma casca convexa");

        // O corte: x do colisor (o comprimento da chuteira, a ponta em +x).
        float heel = std::numeric_limits<float>::max(), tip = -heel, sole = heel;
        for (const Vec3& p : boot.hullPoints) {
            heel = std::min(heel, p.x);
            tip = std::max(tip, p.x);
            sole = std::min(sole, p.z);
        }
        const float cut = heel + settings.jointFromHeel * (tip - heel);
        std::vector<Vec3> crossings;
        Vec3 crossingSum {};
        for (const auto& face : boot.hullFaces) {
            for (int e = 0; e < 3; ++e) {
                const Vec3 a = boot.hullPoints[face[static_cast<std::size_t>(e)]];
                const Vec3 b = boot.hullPoints[face[static_cast<std::size_t>((e + 1) % 3)]];
                if ((a.x - cut) * (b.x - cut) >= 0.0f) continue;
                const float t = (cut - a.x) / (b.x - a.x);
                const Vec3 q = a + (b - a) * t;
                crossings.push_back(q);
                crossingSum += q;
            }
        }
        if (crossings.size() < 3) throw std::runtime_error("divisao do pe: o corte nao atravessa a chuteira de " + foot.id);
        const Vec3 section = crossingSum / static_cast<float>(crossings.size());
        const ConvexHull3D rear = halfHull(boot, cut, false, crossings);
        const ConvexHull3D front = halfHull(boot, cut, true, crossings);

        // Do colisor para o link do pe e para o modelo.
        const auto toFoot = [&](Vec3 p) { return boot.localPosition + boot.localOrientation.rotate(p); };
        const auto toModel = [&](Vec3 p) { return foot.modelPosition + foot.modelOrientation.rotate(toFoot(p)); };
        // O eixo: na linha do corte, no meio da secao de lado, a jointHeight da sola.
        const Vec3 anchorCollider { cut, section.y, sole + settings.jointHeight };
        const Vec3 anchorModel = toModel(anchorCollider);
        // O centro da ponta (a origem do link novo, onde fica a massa dele).
        Vec3 frontCenter {};
        for (const Vec3& p : front.points) frontCenter += p;
        frontCenter = frontCenter / static_cast<float>(front.points.size());
        const Vec3 toeOriginModel = toModel(frontCenter);

        RagdollLinkDefinition3D toe;
        toe.id = std::string(side) + "Toe";
        toe.parentIndex = static_cast<int>(footIndex);
        toe.modelPosition = toeOriginModel;
        toe.modelOrientation = foot.modelOrientation;
        toe.massFraction = foot.massFraction * settings.massShare;
        toe.centerOfMassLocal = {};
        toe.collider = boot;
        toe.collider.hullPoints = front.points;
        toe.collider.hullFaces = front.faces;
        // O colisor no link da ponta: o mesmo giro do da chuteira; a posicao
        // relativa a origem nova (o quadro do link e o do pe, deslocado).
        toe.collider.localPosition = boot.localPosition
            + foot.modelOrientation.conjugate().rotate(foot.modelPosition - toeOriginModel);
        prepareRagdollConvexHull3D(toe.collider);
        toe.inboundJoint.type = RagdollJointType3D::Spherical;
        toe.inboundJoint.anchorModelPosition = anchorModel;
        // O quadro do tornozelo: o twist e o eixo de lado (dobrar a ponta).
        toe.inboundJoint.frameModelOrientation = foot.inboundJoint.frameModelOrientation;
        toe.inboundJoint.axes = {};
        RagdollAxisDefinition3D& bend = toe.inboundJoint.axes[static_cast<std::size_t>(RagdollAxis3D::Twist)];
        bend.enabled = true;
        bend.minimumRadians = settings.minimumRadians;
        bend.maximumRadians = settings.maximumRadians;
        bend.stiffness = settings.stiffness;
        bend.damping = settings.damping;
        bend.maximumTorque = settings.maximumTorque;

        // O pe fica com o calcanhar e com o resto da massa; o centro de massa
        // dele recua para o do conjunto continuar onde estava (a origem do pe).
        RagdollLinkDefinition3D& heelPart = profile.links[footIndex];
        heelPart.collider.hullPoints = rear.points;
        heelPart.collider.hullFaces = rear.faces;
        prepareRagdollConvexHull3D(heelPart.collider);
        const float heelMass = foot.massFraction - toe.massFraction;
        const Vec3 toeInFoot = foot.modelOrientation.conjugate().rotate(toeOriginModel - foot.modelPosition);
        heelPart.centerOfMassLocal = foot.centerOfMassLocal * (foot.massFraction / heelMass)
            - toeInFoot * (toe.massFraction / heelMass);
        heelPart.massFraction = heelMass;
        profile.links.push_back(std::move(toe));

        if (report) {
            char line[256];
            std::snprintf(line, sizeof line,
                "%s: corte a %.1f cm do calcanhar (de %.1f cm), eixo a %.1f cm da sola; calcanhar %zu vertices, "
                "ponta %zu vertices, ponta %.0f g",
                foot.id.c_str(), (cut - heel) * 100.0f, (tip - heel) * 100.0f, settings.jointHeight * 100.0f,
                rear.points.size(), front.points.size(), 1000.0f * profile.totalMassKg * foot.massFraction
                    * settings.massShare);
            report->push_back(line);
        }
    }
    validateRagdollProfileOrThrow3D(profile);
}

} // namespace MatterEngine
