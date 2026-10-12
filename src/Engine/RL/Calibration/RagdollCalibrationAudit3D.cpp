#include "Engine/RL/Calibration/RagdollCalibrationAudit3D.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <string_view>
#include <utility>

namespace MatterEngine {
namespace {

constexpr float RadiansToDegrees = 180.0f / std::numbers::pi_v<float>;

float capsuleVolume(const RagdollCapsuleDefinition3D& collider) {
    const float negativeRadius = collider.radiusMeters;
    const float positiveRadius = ragdollCapsuleRadiusAtPositiveX3D(collider);
    const float middleLength = std::max(0.0f, collider.lengthMeters - negativeRadius - positiveRadius);
    const float frustum = std::numbers::pi_v<float> * middleLength
        * (negativeRadius * negativeRadius + negativeRadius * positiveRadius + positiveRadius * positiveRadius) / 3.0f;
    const float caps = 2.0f * std::numbers::pi_v<float>
        / 3.0f * (negativeRadius * negativeRadius * negativeRadius + positiveRadius * positiveRadius * positiveRadius);
    return frustum + caps;
}

float colliderVolume(const RagdollCapsuleDefinition3D& collider) {
    if (collider.shape == RagdollColliderShape3D::Box) {
        return 8.0f * collider.boxHalfExtents.x * collider.boxHalfExtents.y * collider.boxHalfExtents.z;
    }
    return capsuleVolume(collider);
}

const char* axisName(RagdollAxis3D axis) {
    switch (axis) {
        case RagdollAxis3D::Twist: return "twist";
        case RagdollAxis3D::Swing1: return "swing1";
        case RagdollAxis3D::Swing2: return "swing2";
    }
    return "unknown";
}

const char* severityName(CalibrationFindingSeverity3D severity) {
    switch (severity) {
        case CalibrationFindingSeverity3D::Information: return "informacao";
        case CalibrationFindingSeverity3D::Warning: return "aviso";
        case CalibrationFindingSeverity3D::Error: return "erro";
    }
    return "desconhecido";
}

void addFinding(RagdollCalibrationAudit3D& audit, CalibrationFindingSeverity3D severity, std::string code,
    std::string path, std::string message) {
    audit.findings.push_back({ severity, std::move(code), std::move(path), std::move(message) });
}

Vec3 mirrorAcrossSagittalPlane(Vec3 value) {
    value.y = -value.y;
    return value;
}

float maximumComponentDifference(Vec3 first, Vec3 second) {
    return std::max({ std::abs(first.x - second.x), std::abs(first.y - second.y), std::abs(first.z - second.z) });
}

float colliderSizeDifference(const RagdollCapsuleDefinition3D& first, const RagdollCapsuleDefinition3D& second) {
    if (first.shape != second.shape) {
        return std::numeric_limits<float>::infinity();
    }
    if (first.shape == RagdollColliderShape3D::Box) {
        return maximumComponentDifference(first.boxHalfExtents, second.boxHalfExtents);
    }
    return std::max(
        { std::abs(first.lengthMeters - second.lengthMeters), std::abs(first.radiusMeters - second.radiusMeters),
            std::abs(ragdollCapsuleRadiusAtPositiveX3D(first) - ragdollCapsuleRadiusAtPositiveX3D(second)) });
}

float jointParameterDifference(
    const RagdollJointDefinition3D& first, const RagdollJointDefinition3D& second, bool& axesMatch) {
    axesMatch = first.type == second.type;
    float maximum = 0.0f;
    for (std::size_t index = 0; index < first.axes.size(); ++index) {
        const RagdollAxisDefinition3D& a = first.axes[index];
        const RagdollAxisDefinition3D& b = second.axes[index];
        axesMatch = axesMatch && a.enabled == b.enabled;
        if (!a.enabled || !b.enabled) continue;
        maximum = std::max({ maximum, std::abs(a.minimumRadians - b.minimumRadians),
            std::abs(a.maximumRadians - b.maximumRadians), std::abs(a.stiffness - b.stiffness),
            std::abs(a.damping - b.damping), std::abs(a.maximumTorque - b.maximumTorque) });
    }
    return maximum;
}

std::string counterpartId(std::string_view id) {
    constexpr std::string_view Prefix = "Left";
    if (!id.starts_with(Prefix)) return {};
    return "Right" + std::string(id.substr(Prefix.size()));
}

std::size_t findLink(const RagdollProfile3D& profile, std::string_view id) {
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        if (profile.links[index].id == id) return index;
    }
    return profile.links.size();
}

std::string fixed(float value, int precision = 4) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(precision) << value;
    return output.str();
}

} // namespace

std::size_t RagdollCalibrationAudit3D::findingCount(CalibrationFindingSeverity3D severity) const {
    return static_cast<std::size_t>(std::count_if(findings.begin(), findings.end(),
        [severity](const CalibrationFinding3D& finding) { return finding.severity == severity; }));
}

bool RagdollCalibrationAudit3D::hasErrors() const {
    return findingCount(CalibrationFindingSeverity3D::Error) > 0;
}

RagdollCalibrationAudit3D auditRagdollCalibration3D(const RagdollProfile3D& profile) {
    RagdollCalibrationAudit3D audit;
    audit.profileId = profile.id;
    audit.totalMassKg = profile.totalMassKg;
    audit.standingRootHeightMeters = profile.standingRootHeightMeters;
    audit.linkCount = profile.links.size();
    audit.ignoredSelfCollisionPairCount = profile.selfCollisionIgnoredPairs.size();
    audit.degreesOfFreedom = ragdollDegreesOfFreedom3D(profile);

    for (const RagdollProfileValidationIssue3D& issue : validateRagdollProfile3D(profile)) {
        addFinding(audit, CalibrationFindingSeverity3D::Error, "profile_validation", issue.path, issue.message);
    }

    Vec3 weightedCenter;
    bool everyCenterOfMassIsAtLinkOrigin = !profile.links.empty();
    audit.links.reserve(profile.links.size());
    audit.axes.reserve(audit.degreesOfFreedom);
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        const RagdollLinkDefinition3D& link = profile.links[index];
        RagdollLinkCalibration3D linkAudit;
        linkAudit.index = index;
        linkAudit.id = link.id;
        if (link.parentIndex >= 0 && static_cast<std::size_t>(link.parentIndex) < profile.links.size()) {
            linkAudit.parentId = profile.links[static_cast<std::size_t>(link.parentIndex)].id;
            ++audit.jointCount;
        }
        linkAudit.massFraction = link.massFraction;
        linkAudit.massKg = profile.totalMassKg * link.massFraction;
        linkAudit.centerOfMassModel = link.modelPosition + link.modelOrientation.rotate(link.centerOfMassLocal);
        linkAudit.colliderShape = link.collider.shape;
        linkAudit.approximateColliderVolumeCubicMeters = colliderVolume(link.collider);
        if (linkAudit.approximateColliderVolumeCubicMeters > 0.0f) {
            linkAudit.approximateBulkDensityKgPerCubicMeter =
                linkAudit.massKg / linkAudit.approximateColliderVolumeCubicMeters;
        }
        linkAudit.capsuleLengthMeters = link.collider.lengthMeters;
        linkAudit.capsuleRadiusAtNegativeXMeters = link.collider.radiusMeters;
        linkAudit.capsuleRadiusAtPositiveXMeters = ragdollCapsuleRadiusAtPositiveX3D(link.collider);
        linkAudit.boxHalfExtents = link.collider.boxHalfExtents;
        linkAudit.contactSensor = link.collider.contactSensor;
        if (linkAudit.contactSensor) ++audit.contactSensorCount;

        for (std::size_t axisIndex = 0; axisIndex < link.inboundJoint.axes.size(); ++axisIndex) {
            const RagdollAxisDefinition3D& axis = link.inboundJoint.axes[axisIndex];
            if (!axis.enabled) continue;
            ++linkAudit.degreesOfFreedom;
            audit.axes.push_back({ index, link.id, static_cast<RagdollAxis3D>(axisIndex), axis.minimumRadians,
                axis.maximumRadians, axis.stiffness, axis.damping, axis.maximumTorque });
        }

        audit.massFractionSum += link.massFraction;
        weightedCenter += linkAudit.centerOfMassModel * link.massFraction;
        everyCenterOfMassIsAtLinkOrigin =
            everyCenterOfMassIsAtLinkOrigin && link.centerOfMassLocal.length() < 0.000001f;
        audit.links.push_back(std::move(linkAudit));
    }
    if (audit.massFractionSum > 0.0f) {
        audit.centerOfMassModel = weightedCenter / audit.massFractionSum;
    }

    for (std::size_t leftIndex = 0; leftIndex < profile.links.size(); ++leftIndex) {
        const RagdollLinkDefinition3D& left = profile.links[leftIndex];
        const std::string rightId = counterpartId(left.id);
        if (rightId.empty()) continue;
        const std::size_t rightIndex = findLink(profile, rightId);
        if (rightIndex == profile.links.size()) {
            addFinding(audit, CalibrationFindingSeverity3D::Warning, "missing_symmetry_peer",
                "links[" + std::to_string(leftIndex) + "]", "nao existe o link correspondente " + rightId);
            continue;
        }

        const RagdollLinkDefinition3D& right = profile.links[rightIndex];
        RagdollSymmetryCalibration3D pair;
        pair.leftLinkId = left.id;
        pair.rightLinkId = right.id;
        pair.massDifferenceKg = profile.totalMassKg * std::abs(left.massFraction - right.massFraction);
        pair.mirroredLinkPositionErrorMeters =
            (mirrorAcrossSagittalPlane(left.modelPosition) - right.modelPosition).length();
        const Vec3 leftCenter = left.modelPosition + left.modelOrientation.rotate(left.centerOfMassLocal);
        const Vec3 rightCenter = right.modelPosition + right.modelOrientation.rotate(right.centerOfMassLocal);
        pair.mirroredCenterOfMassErrorMeters = (mirrorAcrossSagittalPlane(leftCenter) - rightCenter).length();
        pair.colliderShapesMatch = left.collider.shape == right.collider.shape;
        pair.colliderSizeDifferenceMeters = colliderSizeDifference(left.collider, right.collider);
        pair.jointParameterMaximumDifference =
            jointParameterDifference(left.inboundJoint, right.inboundJoint, pair.jointAxesMatch);

        if (!pair.colliderShapesMatch || !pair.jointAxesMatch || pair.massDifferenceKg > 0.01f
            || pair.mirroredCenterOfMassErrorMeters > 0.005f || pair.colliderSizeDifferenceMeters > 0.005f
            || pair.jointParameterMaximumDifference > 0.01f) {
            addFinding(audit, CalibrationFindingSeverity3D::Warning, "bilateral_asymmetry", left.id + "/" + right.id,
                "o par bilateral excede uma tolerancia preliminar; "
                "confirmar se a assimetria e deliberada");
        }
        audit.symmetryPairs.push_back(std::move(pair));
    }

    if (everyCenterOfMassIsAtLinkOrigin) {
        addFinding(audit, CalibrationFindingSeverity3D::Warning, "segment_com_unverified", "links[*].centerOfMass",
            "todos os centros de massa locais sao zero; confirmar se cada "
            "origem de link representa deliberadamente o COM do segmento");
    }
    if (!profile.links.empty() && audit.contactSensorCount == profile.links.size()) {
        addFinding(audit, CalibrationFindingSeverity3D::Information, "all_links_are_contact_sensors",
            "links[*].collider",
            "todos os links publicam contato; separar depois sensores da "
            "politica, da recompensa e apenas de diagnostico");
    }
    addFinding(audit, CalibrationFindingSeverity3D::Information, "backend_inertia_not_measured", "inertia",
        "este relatorio mede o perfil declarado; a inercia efetiva calculada "
        "e estabilizada pelo Jolt exige uma sonda dinamica posterior");
    addFinding(audit, CalibrationFindingSeverity3D::Information, "constant_torque_limits", "links[*].joint.axes",
        "os torques atuais sao tetos constantes; envelopes por angulo e "
        "velocidade ainda nao fazem parte do perfil");
    return audit;
}

std::string renderRagdollCalibrationMarkdown3D(const RagdollCalibrationAudit3D& audit) {
    std::ostringstream output;
    output << "# Auditoria de calibracao do ragdoll `" << audit.profileId << "`\n\n"
           << "> Relatorio declarativo. Nao certifica ainda antropometria, "
              "inercia efetiva do Jolt nem comportamento dinamico.\n\n"
           << "## Resumo\n\n"
           << "- Status estrutural: **" << (audit.hasErrors() ? "REPROVADO" : "VALIDO") << "**\n"
           << "- Massa total: " << fixed(audit.totalMassKg, 3) << " kg\n"
           << "- Soma das fracoes de massa: " << fixed(audit.massFractionSum, 6) << "\n"
           << "- Links / juntas / graus de liberdade: " << audit.linkCount << " / " << audit.jointCount << " / "
           << audit.degreesOfFreedom << "\n"
           << "- Sensores de contato: " << audit.contactSensorCount << " / " << audit.linkCount << "\n"
           << "- Pares de autocolisao ignorados: " << audit.ignoredSelfCollisionPairCount << "\n"
           << "- Altura da raiz em pe: " << fixed(audit.standingRootHeightMeters, 4) << " m\n"
           << "- COM global declarado na pose de modelo: (`" << fixed(audit.centerOfMassModel.x) << "`, `"
           << fixed(audit.centerOfMassModel.y) << "`, `" << fixed(audit.centerOfMassModel.z) << "`) m\n"
           << "- Achados: " << audit.findingCount(CalibrationFindingSeverity3D::Error) << " erros, "
           << audit.findingCount(CalibrationFindingSeverity3D::Warning) << " avisos, "
           << audit.findingCount(CalibrationFindingSeverity3D::Information) << " informacoes\n\n";

    output << "## Segmentos\n\n"
           << "| # | Link | Pai | Massa | Fracao | COM modelo (m) | Collider | "
              "Volume aprox. | Densidade aprox. | Sensor | DOF |\n"
           << "|---:|---|---|---:|---:|---|---|---:|---:|:---:|---:|\n";
    for (const RagdollLinkCalibration3D& link : audit.links) {
        output << "| " << link.index << " | " << link.id << " | " << (link.parentId.empty() ? "-" : link.parentId)
               << " | " << fixed(link.massKg, 3) << " kg | " << fixed(link.massFraction, 5) << " | `"
               << fixed(link.centerOfMassModel.x) << ", " << fixed(link.centerOfMassModel.y) << ", "
               << fixed(link.centerOfMassModel.z) << "` | ";
        if (link.colliderShape == RagdollColliderShape3D::Box) {
            output << "caixa `" << fixed(2.0f * link.boxHalfExtents.x, 3) << " x "
                   << fixed(2.0f * link.boxHalfExtents.y, 3) << " x " << fixed(2.0f * link.boxHalfExtents.z, 3)
                   << "` m";
        } else {
            output << "capsula `L=" << fixed(link.capsuleLengthMeters, 3)
                   << ", r-=" << fixed(link.capsuleRadiusAtNegativeXMeters, 3)
                   << ", r+=" << fixed(link.capsuleRadiusAtPositiveXMeters, 3) << "` m";
        }
        output << " | " << fixed(link.approximateColliderVolumeCubicMeters * 1000.0f, 3) << " L | "
               << fixed(link.approximateBulkDensityKgPerCubicMeter, 1) << " kg/m3 | "
               << (link.contactSensor ? "sim" : "nao") << " | " << link.degreesOfFreedom << " |\n";
    }

    output << "\nO volume e a densidade sao aproximacoes geometricas para "
           << "comparacao. Colliders se sobrepoem e nao representam o volume "
           << "anatomico exato.\n\n"
           << "## Eixos atuados\n\n"
           << "| Link | Eixo | Min | Max | Rigidez | Amortecimento | Torque max. |\n"
           << "|---|---|---:|---:|---:|---:|---:|\n";
    for (const RagdollAxisCalibration3D& axis : audit.axes) {
        output << "| " << axis.linkId << " | " << axisName(axis.axis) << " | "
               << fixed(axis.minimumRadians * RadiansToDegrees, 1) << " deg | "
               << fixed(axis.maximumRadians * RadiansToDegrees, 1) << " deg | " << fixed(axis.stiffness, 1) << " | "
               << fixed(axis.damping, 1) << " | " << fixed(axis.maximumTorqueNewtonMeters, 1) << " N.m |\n";
    }

    output << "\n## Simetria bilateral\n\n"
           << "| Esquerdo / direito | Diferenca de massa | Erro espelhado do "
              "link | Erro espelhado do COM | Diferenca de collider | "
              "Eixos iguais |\n"
           << "|---|---:|---:|---:|---:|:---:|\n";
    for (const RagdollSymmetryCalibration3D& pair : audit.symmetryPairs) {
        output << "| " << pair.leftLinkId << " / " << pair.rightLinkId << " | " << fixed(pair.massDifferenceKg, 4)
               << " kg | " << fixed(pair.mirroredLinkPositionErrorMeters * 1000.0f, 2) << " mm | "
               << fixed(pair.mirroredCenterOfMassErrorMeters * 1000.0f, 2) << " mm | "
               << fixed(pair.colliderSizeDifferenceMeters * 1000.0f, 2) << " mm | "
               << (pair.jointAxesMatch ? "sim" : "nao") << " |\n";
    }

    output << "\n## Achados\n\n"
           << "| Severidade | Codigo | Caminho | Mensagem |\n"
           << "|---|---|---|---|\n";
    if (audit.findings.empty()) {
        output << "| informacao | none | - | Nenhum achado. |\n";
    } else {
        for (const CalibrationFinding3D& finding : audit.findings) {
            output << "| " << severityName(finding.severity) << " | `" << finding.code << "` | `" << finding.path
                   << "` | " << finding.message << " |\n";
        }
    }

    output << "\n## Proximas medicoes\n\n"
           << "1. Expor por uma API neutra as massas e inercias efetivas depois "
              "de `RagdollSettings::Stabilize`.\n"
           << "2. Comparar massa/COM segmentar com a referencia antropometrica "
              "selecionada.\n"
           << "3. Medir sobreposicoes e folgas dos colliders na pose neutra.\n"
           << "4. Executar testes de queda, apoio, simetria e torque a 120 Hz.\n"
           << "5. Substituir tetos constantes por um modelo documentado, caso "
              "os ensaios mostrem necessidade.\n";
    return output.str();
}

} // namespace MatterEngine
