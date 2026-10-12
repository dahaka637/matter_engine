#include "Engine/RL/Calibration/RagdollCalibrationAudit3D.hpp"
#include "Engine/RL/Calibration/RagdollColliderWireframe3D.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using namespace MatterEngine;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

RagdollProfile3D loadFootballPlayer() {
    return loadRagdollProfile3D(
        std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
}

void testFootballPlayerBaseline() {
    const RagdollCalibrationAudit3D audit = auditRagdollCalibration3D(loadFootballPlayer());
    require(!audit.hasErrors(), "O perfil atual falhou na validacao estrutural");
    require(audit.profileId == "FootballPlayerV1", "Perfil inesperado");
    require(std::abs(audit.totalMassKg - 80.0f) < 0.001f, "O jogador nao tem 80 kg");
    require(std::abs(audit.massFractionSum - 1.0f) < 0.00001f, "As fracoes de massa nao somam 1");
    require(audit.linkCount == 18, "Quantidade de links mudou");
    require(audit.jointCount == 17, "Quantidade de juntas mudou");
    require(audit.degreesOfFreedom == 39, "Quantidade de DOFs mudou");
    require(audit.axes.size() == audit.degreesOfFreedom, "Tabela de eixos incompleta");
    require(audit.contactSensorCount == audit.linkCount, "Baseline de sensores de contato mudou");
    require(audit.symmetryPairs.size() == 6, "Pares bilaterais nao foram todos auditados");
    require(std::isfinite(audit.centerOfMassModel.x) && std::isfinite(audit.centerOfMassModel.y)
            && std::isfinite(audit.centerOfMassModel.z),
        "COM global nao finito");
    require(audit.findingCount(CalibrationFindingSeverity3D::Warning) >= 1,
        "O COM segmentar ainda nao verificado deveria estar explicito");

    float auditedMass = 0.0f;
    for (const RagdollLinkCalibration3D& link : audit.links) {
        auditedMass += link.massKg;
        require(link.approximateColliderVolumeCubicMeters > 0.0f, "Collider sem volume auditavel");
        require(link.approximateBulkDensityKgPerCubicMeter > 0.0f, "Segmento sem densidade aproximada");
    }
    require(std::abs(auditedMass - 80.0f) < 0.001f, "Massas auditadas nao preservam o total");
}

void testAuditFindsStructuralAndBilateralChanges() {
    RagdollProfile3D profile = loadFootballPlayer();
    profile.links[12].massFraction += 0.001f;
    profile.links[0].massFraction -= 0.001f;
    profile.links[3].inboundJoint.axes[0].minimumRadians = profile.links[3].inboundJoint.axes[0].maximumRadians;

    const RagdollCalibrationAudit3D audit = auditRagdollCalibration3D(profile);
    require(audit.hasErrors(), "Limite estrutural invalido nao foi detectado");
    require(audit.findingCount(CalibrationFindingSeverity3D::Warning) >= 2, "Assimetria bilateral nao foi detectada");
}

void testMarkdownReport() {
    const RagdollCalibrationAudit3D audit = auditRagdollCalibration3D(loadFootballPlayer());
    const std::string markdown = renderRagdollCalibrationMarkdown3D(audit);
    require(markdown.find("FootballPlayerV1") != std::string::npos, "Relatorio sem o perfil");
    require(markdown.find("## Segmentos") != std::string::npos, "Relatorio sem segmentos");
    require(markdown.find("## Eixos atuados") != std::string::npos, "Relatorio sem eixos");
    require(markdown.find("inercia efetiva") != std::string::npos, "Relatorio escondeu a limitacao de inercia");
}

void testColliderWireframeCoversProfile() {
    const RagdollProfile3D profile = loadFootballPlayer();
    const RagdollColliderWireframe3D wireframe = buildRagdollColliderWireframe3D(profile);
    require(wireframe.profileId == profile.id, "Wireframe perdeu a identidade do perfil");
    require(wireframe.links.size() == profile.links.size(), "Wireframe nao cobre todos os colliders");

    std::size_t boxCount = 0;
    std::size_t jointCount = 0;
    for (const RagdollColliderWireframeLink3D& link : wireframe.links) {
        require(!link.segments.empty(), "Collider sem linhas para inspecao");
        if (link.shape == RagdollColliderShape3D::Box) {
            ++boxCount;
            require(link.segments.size() == 12, "Caixa nao possui as 12 arestas");
        }
        if (link.hasInboundJoint) ++jointCount;
        for (const auto& [a, b] : link.segments) {
            require(std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z) && std::isfinite(b.x)
                    && std::isfinite(b.y) && std::isfinite(b.z),
                "Wireframe produziu coordenada nao finita");
        }
    }
    require(boxCount == 2, "Os dois pes deveriam continuar usando caixas");
    require(jointCount == 17, "Wireframe nao publicou todas as juntas");

    bool rejected = false;
    try {
        (void)buildRagdollColliderWireframe3D(profile, 4, 4);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "Amostragem visual insuficiente nao foi rejeitada");
}

void testColliderWireframeFollowsPhysicalPose() {
    const RagdollProfile3D profile = loadFootballPlayer();
    const RagdollColliderWireframe3D rest = buildRagdollColliderWireframe3D(profile);
    std::vector<Vec3> positions;
    std::vector<Quaternion> orientations;
    positions.reserve(profile.links.size());
    orientations.reserve(profile.links.size());
    for (const RagdollLinkDefinition3D& link : profile.links) {
        positions.push_back(link.modelPosition);
        orientations.push_back(link.modelOrientation);
    }

    const RagdollColliderWireframe3D identity = poseRagdollColliderWireframe3D(profile, rest, positions, orientations);
    require((identity.links[6].centerOfMassModel - rest.links[6].centerOfMassModel).length() < 0.000001f,
        "Pose de repouso alterou o COM do wireframe");

    const Vec3 movement { 0.7f, -0.3f, 0.2f };
    for (Vec3& position : positions) position += movement;
    const RagdollColliderWireframe3D moved =
        poseRagdollColliderWireframe3D(profile, rest, positions, orientations, false);
    require((moved.links[6].centerOfMassModel - rest.links[6].centerOfMassModel - movement).length() < 0.000001f,
        "Translacao da pose nao acompanhou o collider");

    const RagdollColliderWireframe3D inPlace =
        poseRagdollColliderWireframe3D(profile, rest, positions, orientations, true);
    const Vec3 expectedInPlace { 0.0f, 0.0f, movement.z };
    require(
        (inPlace.links[6].centerOfMassModel - rest.links[6].centerOfMassModel - expectedInPlace).length() < 0.000001f,
        "Modo in-place nao removeu apenas o movimento horizontal");

    bool rejected = false;
    positions.pop_back();
    try {
        (void)poseRagdollColliderWireframe3D(profile, rest, positions, orientations);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "Pose fisica incompleta nao foi rejeitada");
}

} // namespace

int main() {
    try {
        testFootballPlayerBaseline();
        testAuditFindsStructuralAndBilateralChanges();
        testMarkdownReport();
        testColliderWireframeCoversProfile();
        testColliderWireframeFollowsPhysicalPose();
        std::cout << "Ragdoll calibration audit tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Ragdoll calibration audit tests failed: " << error.what() << '\n';
        return 1;
    }
}
