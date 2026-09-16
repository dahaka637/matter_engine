#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Environment/WindSystem.hpp"
#include "Engine/Control/ActiveRagdollController3D.hpp"
#include "Engine/Control/RagdollImpactTest3D.hpp"
#include "Engine/Environment/OceanSurface.hpp"
#include "Engine/Geometry/GltfAcousticZone3D.hpp"
#include "Engine/Geometry/GltfPhysicsMetadata3D.hpp"
#include "Engine/Geometry/PhysicalAsset3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicalBodyBuilder3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/Physics/WindShelter3D.hpp"
#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Math/Frustum3D.hpp"
#include "Engine/Math/Hash.hpp"
#include "Engine/Math/JitterSequence.hpp"
#include "Engine/Math/ShadowCascade.hpp"
#include "Engine/Locomotion/FootworkSystem3D.hpp"
#include "Engine/Locomotion/PhysicalFootworkController3D.hpp"
#include "Engine/Render/SceneLightPacking.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <span>
#include <utility>
#include <vector>

namespace {

using namespace MatterEngine;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void testRagdollCharacterSkin() {
    const auto character=loadRagdollCharacter3D(std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/characters/crash_test_dummy/character.json");
    require(character.profile.id == "CrashTestDummyV1", "Wrong default humanoid rig");
    require(character.mesh.indices.size() >= 4500, "Low-poly mesh unexpectedly missing triangles");
    const auto bind=sampleRagdollAnimationPose3D(character.profile,nullptr,0.0f,false);
    const auto palette=buildRagdollSkinMatrices3D(character,bind.linkPositions,bind.linkOrientations);
    for (const auto& vertex : character.mesh.vertices) {
        require((skinVertexPosition3D(vertex,palette)-vertex.position).length() < 0.00001f,
            "Skin bind changes the authored surface");
    }
    auto moved=bind;
    const Quaternion rotation=Quaternion::fromAxisAngle({0,0,1},0.72f);
    const Vec3 translation{3,-2,1};
    for (std::size_t i=0;i<moved.linkPositions.size();++i) {
        moved.linkPositions[i]=translation+rotation.rotate(moved.linkPositions[i]);
        moved.linkOrientations[i]=rotation*moved.linkOrientations[i];
    }
    const auto movedPalette=buildRagdollSkinMatrices3D(character,moved.linkPositions,moved.linkOrientations);
    for (const auto& vertex : character.mesh.vertices) {
        require((skinVertexPosition3D(vertex,movedPalette)
            -(translation+rotation.rotate(vertex.position))).length()<0.00001f,
            "Skin does not follow world rotation/translation of physics");
    }
    const auto clip=loadAnimationClip3D(std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/animations/clips/run_forward_dummy.matteranim.json");
    require(validateAnimationClipForRagdoll3D(clip,character.profile).empty(),
        "Low-poly running clip violates physical rig");
    float maximumTravel=0;
    for (int frame=0;frame<=64;++frame) {
        const auto pose=sampleRagdollAnimationPose3D(character.profile,&clip,
            clip.durationSeconds*static_cast<float>(frame)/64.0f,false);
        const auto matrices=buildRagdollSkinMatrices3D(character,pose.linkPositions,pose.linkOrientations);
        for (const auto& vertex : character.mesh.vertices) {
            const auto point=skinVertexPosition3D(vertex,matrices);
            require(std::isfinite(point.x)&&std::isfinite(point.y)&&std::isfinite(point.z)
                && point.length()<2.5f,"Running skin exploded or became non-finite");
            maximumTravel=std::max(maximumTravel,(point-vertex.position).length());
        }
        for (std::size_t i=1;i<character.profile.links.size();++i) {
            const auto& link=character.profile.links[i];
            const auto parent=static_cast<std::size_t>(link.parentIndex);
            const auto& parentLink=character.profile.links[parent];
            const auto anchor=link.inboundJoint.anchorModelPosition;
            const auto a=pose.linkPositions[parent]+pose.linkOrientations[parent].rotate(
                parentLink.modelOrientation.conjugate().rotate(anchor-parentLink.modelPosition));
            const auto b=pose.linkPositions[i]+pose.linkOrientations[i].rotate(
                link.modelOrientation.conjugate().rotate(anchor-link.modelPosition));
            require((a-b).length()<0.00001f,"Low-poly animation disconnected physical anchors");
        }
    }
    require(maximumTravel>0.3f,"Skin remained in bind pose during running");
    auto invalid=character.profile;
    invalid.links[0].collider.radiusMeters=-1;
    require(!validateRagdollProfile3D(invalid).empty(),"Negative per-link radius accepted");
}

void testAnimationClipSampling() {
    AnimationClip3D clip;
    clip.id = "test_turn";
    clip.displayName = "Test turn";
    clip.targetRigId = "HumanAdultV1";
    clip.durationSeconds = 1.0f;
    clip.sourceSampleRateHz = 30.0f;
    AnimationTrack3D track;
    track.targetLinkId = "Pelvis";
    track.space = AnimationTrackSpace3D::Root;
    track.keyframes = {
        { 0.0f, {}, {}, {} },
        { 1.0f, { 2.0f, 0.0f, 0.0f },
            Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                3.14159265358979323846f * 0.5f), {} }
    };
    clip.tracks.push_back(track);

    require(validateAnimationClip3D(clip).empty(),
        "Clipe canônico válido foi rejeitado");
    require(findAnimationTrack3D(clip, "Pelvis") != nullptr,
        "Canal canônico não foi encontrado pelo link do ragdoll");

    const AnimationTransformSample3D halfway = sampleAnimationTrack3D(
        clip.tracks.front(), 0.5f, clip.durationSeconds, true);
    require(std::abs(halfway.translationOffsetMeters.x - 1.0f) < 0.0001f,
        "Interpolação de translação do clipe está incorreta");
    const Vec3 halfwayDirection =
        halfway.rotationDelta.rotate({ 1.0f, 0.0f, 0.0f });
    constexpr float SquareRootHalf = 0.70710678118f;
    require(std::abs(halfwayDirection.x - SquareRootHalf) < 0.0002f
            && std::abs(halfwayDirection.y - SquareRootHalf) < 0.0002f,
        "Interpolação de rotação do clipe está incorreta");

    const AnimationTransformSample3D looped = sampleAnimationTrack3D(
        clip.tracks.front(), 1.25f, clip.durationSeconds, true);
    require(std::abs(looped.translationOffsetMeters.x - 0.5f) < 0.0001f,
        "Amostragem em loop não voltou ao início do clipe");

    clip.tracks.push_back(track);
    require(!validateAnimationClip3D(clip).empty(),
        "Validação aceitou dois canais para o mesmo link");

    const std::string runningPath =
        std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/animations/clips/run_forward.matteranim.json";
    const AnimationClip3D running = loadAnimationClip3D(runningPath);
    require(running.id == "run_forward"
            && running.displayName == "Correr para frente",
        "Clipe de corrida canônico não foi carregado corretamente");
    require(running.targetRigId == "HumanAdultV1"
            && running.tracks.size() == 18,
        "Retarget da corrida não cobre os 18 links do ragdoll");
    require(running.retargetReport.available
            && running.retargetReport.passed
            && running.retargetReport.directionRmsDegrees <= 6.0f
            && running.retargetReport.limbDirectionMaxDegrees <= 8.0f,
        "Corrida não possui relatório de retarget aprovado");
    require(std::abs(running.durationSeconds - 0.6333333f) < 0.0001f
            && std::abs(running.sourceSampleRateHz - 30.0f) < 0.001f,
        "Metadados temporais da corrida foram alterados");
    const AnimationTrack3D* runningPelvis =
        findAnimationTrack3D(running, "Pelvis");
    require(runningPelvis != nullptr && !runningPelvis->keyframes.empty(),
        "Retarget da corrida não contém o canal da pelve");
    const Vec3 rootStart =
        runningPelvis->keyframes.front().translationOffsetMeters;
    const Vec3 rootEnd =
        runningPelvis->keyframes.back().translationOffsetMeters;
    require((rootEnd - rootStart).length() < 0.00001f,
        "Corrida importada ainda possui deriva global no fechamento do ciclo");

    const RagdollProfile3D runningRig = loadRagdollProfile3D(
        std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/physics/ragdolls/HumanAdultV1.ragdoll.json");
    require(validateAnimationClipForRagdoll3D(running, runningRig).empty(),
        "Clipe canônico não respeita o perfil físico alvo");
    const auto linkIndex = [&](std::string_view id) {
        const auto found = std::find_if(runningRig.links.begin(),
            runningRig.links.end(), [&](const RagdollLinkDefinition3D& link) {
                return link.id == id;
            });
        require(found != runningRig.links.end(),
            "Link anatômico esperado não existe no perfil");
        return static_cast<std::size_t>(found - runningRig.links.begin());
    };
    AnimationClip3D elbowFlexion;
    elbowFlexion.durationSeconds = 1.0f;
    for (std::string_view id : { "LeftForearm", "RightForearm" }) {
        AnimationTrack3D elbowTrack;
        elbowTrack.targetLinkId = id;
        elbowTrack.space = AnimationTrackSpace3D::Joint;
        elbowTrack.keyframes.push_back({ 0.0f, {}, {},
            { 3.14159265358979323846f * 0.5f, 0.0f, 0.0f } });
        elbowFlexion.tracks.push_back(std::move(elbowTrack));
    }
    const RagdollAnimationPose3D flexedPose = sampleRagdollAnimationPose3D(
        runningRig, &elbowFlexion, 0.0f, false);
    const auto worldAnchor = [&](std::size_t index) {
        const RagdollLinkDefinition3D& link = runningRig.links[index];
        return flexedPose.linkPositions[index]
            + flexedPose.linkOrientations[index].rotate(
                link.modelOrientation.conjugate().rotate(
                    link.inboundJoint.anchorModelPosition
                        - link.modelPosition));
    };
    const Vec3 leftForearmDirection = (worldAnchor(linkIndex("LeftHand"))
        - worldAnchor(linkIndex("LeftForearm"))).normalized();
    const Vec3 rightForearmDirection = (worldAnchor(linkIndex("RightHand"))
        - worldAnchor(linkIndex("RightForearm"))).normalized();
    require(leftForearmDirection.x > 0.99f
            && rightForearmDirection.x > 0.99f,
        "Frames espelhados dos cotovelos não flexionam para a mesma frente");
    const auto rotationVector = [](Quaternion rotation) {
        rotation = rotation.normalized();
        if (rotation.w < 0.0f) {
            rotation = { -rotation.x, -rotation.y, -rotation.z, -rotation.w };
        }
        const Vec3 imaginary { rotation.x, rotation.y, rotation.z };
        const float length = imaginary.length();
        if (length < 0.000001f) return Vec3 {};
        return imaginary * (2.0f * std::atan2(length,
            std::clamp(rotation.w, 0.0f, 1.0f)) / length);
    };
    for (int sampleIndex = 0; sampleIndex <= 64; ++sampleIndex) {
        const float time = running.durationSeconds
            * static_cast<float>(sampleIndex) / 64.0f;
        const RagdollAnimationPose3D pose = sampleRagdollAnimationPose3D(
            runningRig, &running, time, true);
        require(pose.linkPositions.size() == runningRig.links.size(),
            "Pose da corrida não cobre todos os links do ragdoll");
        for (std::size_t linkIndex = 1;
                linkIndex < runningRig.links.size(); ++linkIndex) {
            const RagdollLinkDefinition3D& link =
                runningRig.links[linkIndex];
            const std::size_t parentIndex =
                static_cast<std::size_t>(link.parentIndex);
            const RagdollLinkDefinition3D& parent =
                runningRig.links[parentIndex];
            const Vec3 parentAnchor = pose.linkPositions[parentIndex]
                + pose.linkOrientations[parentIndex].rotate(
                    parent.modelOrientation.conjugate().rotate(
                        link.inboundJoint.anchorModelPosition
                            - parent.modelPosition));
            const Vec3 childAnchor = pose.linkPositions[linkIndex]
                + pose.linkOrientations[linkIndex].rotate(
                    link.modelOrientation.conjugate().rotate(
                        link.inboundJoint.anchorModelPosition
                            - link.modelPosition));
            require((parentAnchor - childAnchor).length() < 0.00001f,
                "Pose da corrida separou as âncoras de uma articulação");

            const Quaternion localOrientation =
                (pose.linkOrientations[parentIndex].conjugate()
                    * pose.linkOrientations[linkIndex]).normalized();
            const Quaternion parentFrame =
                (parent.modelOrientation.conjugate()
                    * link.inboundJoint.frameModelOrientation).normalized();
            const Quaternion childFrame =
                (link.modelOrientation.conjugate()
                    * link.inboundJoint.frameModelOrientation).normalized();
            const AnimationTrack3D* sourceTrack =
                findAnimationTrack3D(running, link.id);
            require(sourceTrack != nullptr,
                "Clipe de corrida não cobre um link articulado");
            const AnimationTransformSample3D sourceSample =
                sampleAnimationTrack3D(*sourceTrack, time,
                    running.durationSeconds, true);
            const Vec3 sourceCoordinates = sourceSample.jointPositionRadians;
            const Vec3 coordinates = rotationVector(
                parentFrame.conjugate() * localOrientation * childFrame);
            const float values[] = {
                coordinates.x, coordinates.y, coordinates.z
            };
            const float sourceValues[] = {
                sourceCoordinates.x, sourceCoordinates.y, sourceCoordinates.z
            };
            for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
                const RagdollAxisDefinition3D& axis =
                    link.inboundJoint.axes[axisIndex];
                require(axis.enabled
                        ? sourceValues[axisIndex]
                                >= axis.minimumRadians - 0.0001f
                            && sourceValues[axisIndex]
                                <= axis.maximumRadians + 0.0001f
                        : std::abs(sourceValues[axisIndex]) < 0.0001f,
                    "Clipe canônico contém alvo fora dos limites físicos");
                require(axis.enabled
                        ? values[axisIndex] >= axis.minimumRadians - 0.0001f
                            && values[axisIndex]
                                <= axis.maximumRadians + 0.0001f
                        : std::abs(values[axisIndex]) < 0.0001f,
                    "Pose da corrida excedeu os limites de uma articulação");
            }
        }
    }
}

void testRagdollImpactGenerator() {
    RagdollImpactTest3D pulse;
    pulse.config().forceNewtons = 350.0f;
    pulse.config().directionDegrees = 90.0f;
    pulse.config().pulseDurationSeconds = 0.10f;
    pulse.config().intervalSeconds = 3.0f;
    pulse.triggerNow();
    pulse.update(1.0f / 120.0f);
    require(pulse.output().applying
            && std::abs(pulse.output().forceNewtons - 350.0f) < 0.01f
            && std::abs(pulse.output().directionDegrees - 90.0f) < 0.01f,
        "Gerador de impacto direto nao publicou a rajada configurada");
    for (int tick = 0; tick < 20; ++tick) {
        pulse.update(1.0f / 120.0f);
    }
    require(!pulse.output().applying && pulse.output().eventCount == 1,
        "Rajada direta nao terminou de forma deterministica");

    pulse.config().mode = RagdollImpactMode3D::Continuous;
    pulse.setRunning(true);
    pulse.update(1.0f / 120.0f);
    require(pulse.output().applying
            && pulse.output().secondsRemaining < 0.0f,
        "Empurrao continuo nao permaneceu ativo");
    pulse.setRunning(false);
    pulse.update(1.0f / 120.0f);
    require(!pulse.output().applying,
        "Empurrao continuo continuou aplicando forca apos pausa");

    RagdollImpactTest3D randomA;
    RagdollImpactTest3D randomB;
    randomA.config().mode = RagdollImpactMode3D::Random;
    randomB.config().mode = RagdollImpactMode3D::Random;
    randomA.reset(0x12345678u);
    randomB.reset(0x12345678u);
    randomA.config().mode = RagdollImpactMode3D::Random;
    randomB.config().mode = RagdollImpactMode3D::Random;
    randomA.triggerNow();
    randomB.triggerNow();
    randomA.update(1.0f / 120.0f);
    randomB.update(1.0f / 120.0f);
    require(randomA.output().applying && randomB.output().applying
            && randomA.output().forceNewtons
                == randomB.output().forceNewtons
            && randomA.output().directionDegrees
                == randomB.output().directionDegrees
            && randomA.output().secondsRemaining
                == randomB.output().secondsRemaining,
        "Modo aleatorio do laboratorio nao e reproduzivel por seed");
}

class TestFootworkTerrain final : public FootworkTerrainProbe3D {
public:
    bool raycastGround(Vec3 candidatePosition, float distanceAboveMeters,
        float distanceBelowMeters,
        FootworkTerrainHit3D& hit) const override {
        const float groundZ = slopeX * candidatePosition.x;
        const float originZ = candidatePosition.z + distanceAboveMeters;
        if (originZ < groundZ
            || originZ - groundZ
                > distanceAboveMeters + distanceBelowMeters) {
            return false;
        }
        hit.position = {
            candidatePosition.x, candidatePosition.y, groundZ
        };
        hit.normal = Vec3 { -slopeX, 0.0f, 1.0f }.normalized();
        hit.distanceMeters = originZ - groundZ;
        return true;
    }

    float slopeX = 0.0f;
};

// Réplica local de Workbench/Laboratory/StaticFootworkTerrainProbe.hpp — o
// núcleo de testes não pode depender do Workbench, mas o comportamento
// precisa ser idêntico ao que o laboratório real usa: um raycast físico
// contra a malha estática real, não uma fórmula analítica de plano perfeito
// como TestFootworkTerrain acima. Colisão malha-vs-cápsula/caixa no PhysX
// tem um caminho de geração de contato diferente (e mais ruidoso nas
// costuras dos triângulos) do que caixa-vs-caixa; um controlador só validado
// contra TestFootworkTerrain/createStaticBox nunca é exercitado contra essa
// fonte real de ruído.
class RealisticFootworkTerrain final : public FootworkTerrainProbe3D {
public:
    explicit RealisticFootworkTerrain(const PhysicsScene3D& scene)
        : m_scene(scene) {
    }

    bool raycastGround(Vec3 candidatePosition, float distanceAboveMeters,
        float distanceBelowMeters,
        FootworkTerrainHit3D& hit) const override {
        const float above = std::max(0.01f, distanceAboveMeters);
        const float below = std::max(0.01f, distanceBelowMeters);
        PhysicsRayHit3D physicsHit;
        if (!m_scene.raycastStatic(
                { candidatePosition + Vec3 { 0.0f, 0.0f, above },
                    { 0.0f, 0.0f, -1.0f } },
                above + below, physicsHit)) {
            return false;
        }
        hit.position = physicsHit.position;
        hit.normal = physicsHit.normal;
        hit.distanceMeters = physicsHit.distance;
        return true;
    }

private:
    const PhysicsScene3D& m_scene;
};

void testFootworkPlanner() {
    TestFootworkTerrain flat;
    FootworkSystem3D system;
    require(system.reset({}, 0.0f, flat),
        "Footwork nao inicializou em terreno plano");
    const FootworkDebugState3D& initial = system.state();
    require(initial.initialized
            && initial.supportPolygonCount >= 4,
        "Footwork nao publicou a base de suporte inicial");
    require(initial.feet[0].pose.position.y
            > initial.feet[1].pose.position.y,
        "Footwork trocou ou cruzou os pes na inicializacao");

    FootworkInput3D forward;
    forward.movementLocal.y = 1.0f;
    forward.lookYawRadians = 0.0f;
    float maximumSwingHeight = 0.0f;
    float minimumSwingForwardZ = 0.0f;
    float maximumSwingForwardZ = 0.0f;
    for (int step = 0; step < 360; ++step) {
        system.update(forward, 1.0f / 120.0f, flat);
        const FootworkDebugState3D& state = system.state();
        require(state.feet[0].pose.position.y
                > state.feet[1].pose.position.y - 0.02f,
            "Planejador permitiu cruzamento dos pes andando reto");
        if (state.hasActiveSwing) {
            const std::size_t active =
                footIndex3D(state.activeSwingSide);
            const std::size_t support = footIndex3D(
                oppositeFoot3D(state.activeSwingSide));
            Vec3 rootToSupport = state.rootGroundPosition
                - state.feet[support].pose.position;
            rootToSupport.z = 0.0f;
            require(rootToSupport.length()
                    <= system.config().rootSupportLimitMeters + 1.0e-4f,
                "Raiz virtual escapou do apoio durante a passada");
            maximumSwingHeight = std::max(maximumSwingHeight,
                state.feet[active].pose.position.z);
            const Vec3 footForward =
                state.feet[active].pose.orientation.rotate(
                    { 1.0f, 0.0f, 0.0f });
            minimumSwingForwardZ = std::min(
                minimumSwingForwardZ, footForward.z);
            maximumSwingForwardZ = std::max(
                maximumSwingForwardZ, footForward.z);
        }
    }
    require(system.state().completedStepCount >= 4,
        "Footwork nao produziu cadencia andando para frente");
    require(system.state().rootGroundPosition.x > 1.5f,
        "Raiz virtual do Footwork nao avancou");
    require(maximumSwingHeight > 0.05f,
        "Trajetoria do pe nao produziu clearance");
    require(minimumSwingForwardZ < -0.08f
            && maximumSwingForwardZ > 0.06f,
        "Pe permaneceu artificialmente reto durante a passada");

    // Soltar a direcao durante o balanço deve procurar contato seguro e
    // terminar plantado, nunca deixar um estado pendurado.
    bool foundSwing = false;
    for (int step = 0; step < 180 && !foundSwing; ++step) {
        system.update(forward, 1.0f / 120.0f, flat);
        foundSwing = system.state().hasActiveSwing;
    }
    require(foundSwing,
        "Teste de aborto nao conseguiu iniciar uma passada");
    FootworkInput3D stopped;
    stopped.lookYawRadians = 0.0f;
    for (int step = 0; step < 180; ++step) {
        system.update(stopped, 1.0f / 120.0f, flat);
    }
    require(!system.state().hasActiveSwing
            && system.state().feet[0].phase == FootPhase3D::Planted
            && system.state().feet[1].phase == FootPhase3D::Planted,
        "Soltar input deixou o Footwork fora de um apoio estavel");

    // Inverter o comando no meio do balanço precisa trocar o alvo já no
    // próximo tick, sem terminar primeiro a passada da direção antiga.
    FootworkSystem3D reversalSystem;
    require(reversalSystem.reset({}, 0.0f, flat),
        "Footwork de reversao nao inicializou");
    bool reversalSwingStarted = false;
    for (int step = 0; step < 180 && !reversalSwingStarted; ++step) {
        reversalSystem.update(forward, 1.0f / 120.0f, flat);
        reversalSwingStarted =
            reversalSystem.state().hasActiveSwing;
    }
    require(reversalSwingStarted,
        "Teste de reversao nao iniciou uma passada");
    const std::size_t reversalFoot = footIndex3D(
        reversalSystem.state().activeSwingSide);
    const float oldTargetX =
        reversalSystem.state().feet[reversalFoot].target.position.x;
    FootworkInput3D backward;
    backward.movementLocal.y = -1.0f;
    backward.lookYawRadians = 0.0f;
    reversalSystem.update(backward, 1.0f / 120.0f, flat);
    require(reversalSystem.state().feet[reversalFoot]
                .target.position.x < oldTargetX - 0.02f,
        "Footwork conservou o alvo antigo apos inverter a direcao");

    // Mesmo sob mudanças agressivas de direção e orientação, as caixas dos
    // pés não podem compartilhar a mesma posição.
    FootworkSystem3D turningSystem;
    require(turningSystem.reset({}, 0.0f, flat),
        "Footwork de giro nao inicializou");
    const float minimumFootDistance =
        turningSystem.config().footWidthMeters
        + turningSystem.config().antiCrossingMarginMeters;
    for (int step = 0; step < 900; ++step) {
        FootworkInput3D input;
        input.movementLocal = {
            (step / 45) % 2 == 0 ? 0.85f : -0.85f, 0.45f
        };
        input.lookYawRadians =
            static_cast<float>(step) * 0.021f;
        turningSystem.update(input, 1.0f / 120.0f, flat);
        Vec3 feetDelta = turningSystem.state().feet[0].pose.position
            - turningSystem.state().feet[1].pose.position;
        feetDelta.z = 0.0f;
        require(feetDelta.length()
                >= minimumFootDistance - 1.0e-4f,
            "Footwork permitiu sobreposicao dos pes durante giro");
    }

    // Girar a câmera continuamente mantendo W altera a curva do caminho,
    // mas jamais pode reiniciar a mesma passada indefinidamente.
    FootworkSystem3D curvedSystem;
    require(curvedSystem.reset({}, 0.0f, flat),
        "Footwork curvo nao inicializou");
    for (int step = 0; step < 720; ++step) {
        FootworkInput3D curved;
        curved.movementLocal.y = 1.0f;
        curved.lookYawRadians = static_cast<float>(step) * 0.018f;
        curvedSystem.update(curved, 1.0f / 120.0f, flat);
    }
    require(curvedSystem.state().completedStepCount >= 8,
        "Girar a camera deixou um pe preso no ar");

    // Shift seleciona uma corrida de passos mais longos e apoio fugaz. Depois
    // de embalar, o próximo pé já deve estar em preparação no contato.
    FootworkSystem3D walkingComparison;
    FootworkSystem3D runningSystem;
    require(walkingComparison.reset({}, 0.0f, flat)
            && runningSystem.reset({}, 0.0f, flat),
        "Comparativo de corrida nao inicializou");
    std::uint32_t runningIdleTicks = 0;
    float maximumRunningTrajectoryCrossTrackMeters = 0.0f;
    for (int step = 0; step < 480; ++step) {
        FootworkInput3D walking;
        walking.movementLocal.y = 1.0f;
        walkingComparison.update(walking, 1.0f / 120.0f, flat);
        FootworkInput3D running = walking;
        running.fast = true;
        runningSystem.update(running, 1.0f / 120.0f, flat);
        if (runningSystem.state().completedStepCount > 0
            && !runningSystem.state().hasActiveSwing) {
            ++runningIdleTicks;
        }
        const FootworkDebugState3D& runningState = runningSystem.state();
        require(runningState.feet[0].pose.position.y
                > runningState.feet[1].pose.position.y,
            "Corrida cruzou os corredores laterais dos pes");
        if (runningState.hasActiveSwing
            && runningState.swingTrajectoryCount >= 2) {
            const Vec3 start = runningState.swingTrajectory.front();
            const Vec3 finish = runningState.swingTrajectory[
                runningState.swingTrajectoryCount - 1];
            Vec3 direction = finish - start;
            direction.z = 0.0f;
            const float length = direction.length();
            if (length > 1.0e-5f) {
                direction *= 1.0f / length;
                for (std::size_t sample = 1;
                        sample + 1
                            < runningState.swingTrajectoryCount;
                        ++sample) {
                    Vec3 offset =
                        runningState.swingTrajectory[sample] - start;
                    offset.z = 0.0f;
                    const Vec3 crossTrack =
                        offset - direction * dot(offset, direction);
                    maximumRunningTrajectoryCrossTrackMeters = std::max(
                        maximumRunningTrajectoryCrossTrackMeters,
                        crossTrack.length());
                }
            }
        }
    }
    require(runningSystem.state().rootGroundPosition.x
            > walkingComparison.state().rootGroundPosition.x + 2.0f,
        "Shift nao produziu deslocamento de corrida");
    require(runningSystem.state().completedStepCount
            > walkingComparison.state().completedStepCount
            && runningIdleTicks <= 1,
        "Corrida manteve pausa artificial entre as passadas");
    require(maximumRunningTrajectoryCrossTrackMeters < 0.012f,
        "Planejador desenhou uma trajetoria lateral torta durante a corrida");

    // A mesma sequência fixa deve produzir exatamente o mesmo resultado:
    // o futuro rollback/replay não pode depender do framerate de render.
    FootworkSystem3D deterministicA;
    FootworkSystem3D deterministicB;
    require(deterministicA.reset({ 2.0f, 1.0f, 0.0f }, 0.2f, flat)
            && deterministicB.reset(
                { 2.0f, 1.0f, 0.0f }, 0.2f, flat),
        "Footwork deterministico nao inicializou");
    for (int step = 0; step < 480; ++step) {
        FootworkInput3D input;
        input.movementLocal = step < 260
            ? Vec2 { 0.35f, 0.85f } : Vec2 { -0.25f, 0.55f };
        input.lookYawRadians = step < 180 ? 0.2f : 0.65f;
        input.fast = step > 300;
        deterministicA.update(input, 1.0f / 120.0f, flat);
        deterministicB.update(input, 1.0f / 120.0f, flat);
    }
    const FootworkDebugState3D& resultA = deterministicA.state();
    const FootworkDebugState3D& resultB = deterministicB.state();
    require((resultA.rootGroundPosition
                - resultB.rootGroundPosition).lengthSquared() < 1.0e-12f
            && (resultA.feet[0].pose.position
                - resultB.feet[0].pose.position).lengthSquared() < 1.0e-12f
            && resultA.completedStepCount == resultB.completedStepCount,
        "Footwork divergiu sob entradas e fixed steps identicos");

    TestFootworkTerrain slope;
    slope.slopeX = 0.75f;
    FootworkSystem3D slopedSystem;
    require(slopedSystem.reset({}, 0.0f, slope),
        "Footwork rejeitou uma inclinacao caminhavel");
    require(slopedSystem.state().feet[0].pose.groundNormal.x < -0.50f,
        "Pe nao se alinhou a normal do terreno");
    FootworkInput3D downhill;
    downhill.movementLocal.y = -1.0f;
    downhill.lookYawRadians = 0.0f;
    for (int step = 0; step < 480; ++step) {
        slopedSystem.update(downhill, 1.0f / 120.0f, slope);
    }
    require(slopedSystem.state().completedStepCount >= 4
            && slopedSystem.state().rootGroundPosition.x < -1.0f,
        "Footwork nao conseguiu descer uma rampa ingreme");
}

void testTaskScheduler() {
    TaskScheduler scheduler(TaskSchedulerSettings { 4 });
    std::vector<std::atomic<std::uint32_t>> visits(4096);
    for (auto& visit : visits) visit.store(0, std::memory_order_relaxed);
    scheduler.parallelFor(visits.size(), 32,
        [](std::size_t begin, std::size_t end, void* context) noexcept {
            auto& counters = *static_cast<
                std::vector<std::atomic<std::uint32_t>>*>(context);
            for (std::size_t index = begin; index < end; ++index) {
                counters[index].fetch_add(1, std::memory_order_relaxed);
            }
        }, &visits);
    require(std::all_of(visits.begin(), visits.end(),
        [](const std::atomic<std::uint32_t>& visit) {
            return visit.load(std::memory_order_relaxed) == 1;
        }), "Job system perdeu ou duplicou itens do parallelFor");

    // Reproduz a carga que fazia o benchmark fisico congelar: muitos
    // parallelFor pequenos (71 props, grain 64) disparados em sequencia. A
    // antiga janela de notificacao perdida deixava tarefas enfileiradas e
    // todos os workers dormindo depois de algumas centenas de iteracoes.
    std::atomic<std::uint64_t> stressVisits { 0 };
    for (std::uint32_t round = 0; round < 4000; ++round) {
        scheduler.parallelFor(71, 64,
            [](std::size_t begin, std::size_t end,
                void* counterContext) noexcept {
                auto& counter = *static_cast<std::atomic<std::uint64_t>*>(
                    counterContext);
                counter.fetch_add(static_cast<std::uint64_t>(end - begin),
                    std::memory_order_relaxed);
            }, &stressVisits);
    }
    require(stressVisits.load(std::memory_order_relaxed) == 4000ull * 71ull,
        "Job system travou ou perdeu trabalho em rajadas pequenas");

    // Um worker tambem pode abrir e aguardar trabalho filho. Esta situacao e
    // comum em middlewares e precisa funcionar ate na configuracao minima de
    // um unico worker, sem depender de oversubscription para evitar deadlock.
    TaskScheduler singleWorker(TaskSchedulerSettings { 1 });
    std::atomic<std::uint32_t> nestedVisits { 0 };
    struct NestedContext {
        TaskScheduler* scheduler = nullptr;
        std::atomic<std::uint32_t>* visits = nullptr;
    } nestedContext { &singleWorker, &nestedVisits };
    TaskGroup outerGroup;
    singleWorker.submit({ [](void* rawContext) noexcept {
        auto& nested = *static_cast<NestedContext*>(rawContext);
        nested.scheduler->parallelFor(256, 8,
            [](std::size_t begin, std::size_t end,
                void* counterContext) noexcept {
                auto& counter = *static_cast<std::atomic<std::uint32_t>*>(
                    counterContext);
                counter.fetch_add(static_cast<std::uint32_t>(end - begin),
                    std::memory_order_relaxed);
            }, nested.visits);
    }, &nestedContext }, &outerGroup);
    singleWorker.wait(outerGroup);
    require(nestedVisits.load(std::memory_order_relaxed) == 256,
        "Job system bloqueou ou perdeu tarefas filhas em um unico worker");
}

void testFrustumCulling() {
    const Mat4 view = Mat4::lookAt({}, { 1.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f });
    // Mat4::perspective() produz profundidade INVERTIDA (perto=1, longe=0,
    // ver comentario na propria funcao) - reversedDepth=true e obrigatorio
    // aqui, senao os planos proximo/distante saem trocados.
    constexpr float NearPlane = 0.1f;
    constexpr float FarPlane = 100.0f;
    const Mat4 projection = Mat4::perspective(
        1.0471975512f, 16.0f / 9.0f, NearPlane, FarPlane);
    const Frustum3D frustum = Frustum3D::fromViewProjection(
        projection * view, /*reversedDepth=*/true);
    require(frustum.intersectsSphere({ 5.0f, 0.0f, 0.0f }, 0.5f),
        "Frustum removeu objeto visivel");
    require(!frustum.intersectsSphere({ -5.0f, 0.0f, 0.0f }, 0.5f),
        "Frustum preservou objeto atras da camera");
    require(!frustum.intersectsSphere({ 5.0f, 20.0f, 0.0f }, 0.5f),
        "Frustum preservou objeto fora da lateral");

    // Planos proximo/distante especificamente - e exatamente aqui que um
    // bug de profundidade invertida (formulas de perto/longe trocadas)
    // apareceria: um objeto atras do plano distante ou na frente do
    // proximo continuaria sendo considerado visivel.
    require(!frustum.intersectsSphere({ NearPlane * 0.3f, 0.0f, 0.0f }, 0.01f),
        "Frustum preservou objeto antes do plano proximo (bug de Z invertido)");
    require(!frustum.intersectsSphere({ FarPlane * 1.5f, 0.0f, 0.0f }, 0.5f),
        "Frustum preservou objeto depois do plano distante (bug de Z invertido)");
    require(frustum.intersectsSphere({ FarPlane * 0.5f, 0.0f, 0.0f }, 0.5f),
        "Frustum removeu objeto bem no meio do intervalo proximo-distante");
}

void testPackSceneLights() {
    std::vector<LightRender3D> lights(3);
    lights[0].type = LightType3D::Directional;
    lights[0].direction = { 0.0f, 0.0f, 1.0f };
    lights[0].intensity = 2.0f;
    lights[0].range = 50.0f; // nao se aplica a Directional - deve ser ignorado
    lights[0].coneOuterDegrees = 10.0f; // idem
    lights[0].castsShadow = true;

    lights[1].type = LightType3D::Point;
    lights[1].position = { 1.0f, 2.0f, 3.0f };
    lights[1].intensity = -5.0f; // deve clampar para 0
    lights[1].range = -1.0f; // deve clampar para 0

    lights[2].type = LightType3D::Spot;
    lights[2].position = { 4.0f, 5.0f, 6.0f };
    lights[2].direction = { 1.0f, 0.0f, 0.0f };
    lights[2].intensity = 3.0f;
    lights[2].range = 10.0f;
    lights[2].coneOuterDegrees = 200.0f; // meio-angulo deve clampar para <=89 graus

    const std::vector<GpuLightData3D> packed = packSceneLights(lights);
    require(packed.size() == 3, "packSceneLights nao preservou a contagem de luzes");

    // Ordem preservada.
    require(packed[0].colorIntensity[3] == 2.0f,
        "packSceneLights nao preservou a ordem/intensidade da luz 0");
    require(packed[2].positionRange[0] == 4.0f,
        "packSceneLights nao preservou a ordem/posicao da luz 2");

    // Comportamento default por tipo: Directional ignora alcance e cone.
    require(packed[0].positionRange[3] == 0.0f,
        "Luz Directional deveria zerar o alcance, que nao se aplica a ela");
    require(packed[0].directionOuterCosine[3] == -1.0f
        && packed[0].parameters[0] == -1.0f,
        "Luz Directional deveria usar o cosseno-sentinela (-1) no cone");

    // Comportamento default por tipo: Point tambem ignora cone (mas usa alcance).
    require(packed[1].directionOuterCosine[3] == -1.0f
        && packed[1].parameters[0] == -1.0f,
        "Luz Point deveria usar o cosseno-sentinela (-1) no cone");

    // Clamping: intensidade e alcance negativos viram 0.
    require(packed[1].colorIntensity[3] == 0.0f,
        "packSceneLights nao clampou intensidade negativa para 0");
    require(packed[1].positionRange[3] == 0.0f,
        "packSceneLights nao clampou alcance negativo para 0");

    // Clamping: meio-angulo do cone fica dentro de [1,89] graus mesmo com
    // um coneOuterDegrees absurdo (200 graus).
    const float outerHalfAngleRadians = std::acos(packed[2].directionOuterCosine[3]);
    const float outerHalfAngleDegrees = outerHalfAngleRadians * (180.0f / 3.14159265358979323846f);
    require(outerHalfAngleDegrees <= 89.0f + 0.01f,
        "packSceneLights nao clampou o meio-angulo do cone da luz Spot");

    // Overflow: uma lista bem maior que qualquer capacidade fixa antiga
    // continua sendo empacotada inteira, sem truncamento.
    std::vector<LightRender3D> manyLights(64);
    for (std::size_t index = 0; index < manyLights.size(); ++index) {
        manyLights[index].type = LightType3D::Point;
        manyLights[index].position = { static_cast<float>(index), 0.0f, 0.0f };
    }
    const std::vector<GpuLightData3D> packedMany = packSceneLights(manyLights);
    require(packedMany.size() == 64,
        "packSceneLights truncou uma lista grande de luzes");
    require(packedMany[63].positionRange[0] == 63.0f,
        "packSceneLights perdeu a ordem numa lista grande de luzes");
}

void testHaltonJitter() {
    // Valores conhecidos da sequencia de Halton (radical inverse), calculados
    // a mao - ver JitterSequence.cpp.
    require(std::abs(haltonSequence(1, 2) - 0.5f) < 1e-6f,
        "Halton(1,2) deveria ser 0.5");
    require(std::abs(haltonSequence(2, 2) - 0.25f) < 1e-6f,
        "Halton(2,2) deveria ser 0.25");
    require(std::abs(haltonSequence(3, 2) - 0.75f) < 1e-6f,
        "Halton(3,2) deveria ser 0.75");
    require(std::abs(haltonSequence(1, 3) - (1.0f / 3.0f)) < 1e-6f,
        "Halton(1,3) deveria ser 1/3");
    require(std::abs(haltonSequence(2, 3) - (2.0f / 3.0f)) < 1e-6f,
        "Halton(2,3) deveria ser 2/3");

    const Vec2 jitter0 = haltonJitter(0);
    require(std::abs(jitter0.x - 0.5f) < 1e-6f
        && std::abs(jitter0.y - (1.0f / 3.0f)) < 1e-6f,
        "haltonJitter(0) deveria usar Halton(1,2)/Halton(1,3)");
    const Vec2 jitter1 = haltonJitter(1);
    require(std::abs(jitter1.x - 0.25f) < 1e-6f
        && std::abs(jitter1.y - (2.0f / 3.0f)) < 1e-6f,
        "haltonJitter(1) deveria usar Halton(2,2)/Halton(2,3)");

    // Limites: a sequencia nunca sai de [0,1) para nenhum indice pequeno.
    for (std::uint32_t index = 0; index < 64; ++index) {
        const Vec2 sample = haltonJitter(index);
        require(sample.x >= 0.0f && sample.x < 1.0f
            && sample.y >= 0.0f && sample.y < 1.0f,
            "haltonJitter saiu dos limites [0,1)");
    }
}

void testHash() {
    // Deterministico: mesma entrada sempre produz a mesma saida - a base de
    // tudo que depende deste hash (WindSystem, ProceduralNoise) precisar
    // ser reproduzivel.
    require(pcgHash(42u) == pcgHash(42u),
        "pcgHash deveria ser deterministico para a mesma entrada");
    // Avalanche minimo: mudar 1 bit da entrada deveria mudar bastante a
    // saida, nao so um bit (senao nao serviria de hash de verdade). Nao
    // exigimos um numero exato de bits diferentes, so que nao seja quase
    // identico.
    const std::uint32_t hashA = pcgHash(1000u);
    const std::uint32_t hashB = pcgHash(1001u);
    require(hashA != hashB,
        "Entradas vizinhas produziram o mesmo hash");

    // hashToUnitFloat nunca sai de [0,1) para nenhuma entrada testada.
    for (std::uint32_t seed = 0; seed < 200; ++seed) {
        const float value = hashToUnitFloat(seed);
        require(value >= 0.0f && value < 1.0f,
            "hashToUnitFloat saiu dos limites [0,1)");
    }
}

void testWindSystem() {
    // Deterministico: duas instancias avancadas com os mesmos passos de
    // tempo devem concordar exatamente (a mesma direcao/rajada) - e o que
    // garante ceu, fisica e audio nunca dessincronizarem entre si (ver
    // WindSystem.hpp).
    WindSystem windA;
    WindSystem windB;
    for (int step = 0; step < 37; ++step) {
        windA.advance(0.1f);
        windB.advance(0.1f);
    }
    const Vec3 velocityA = windA.velocityAtHeight(10.0f);
    const Vec3 velocityB = windB.velocityAtHeight(10.0f);
    require(std::abs(velocityA.x - velocityB.x) < 1e-6f
        && std::abs(velocityA.y - velocityB.y) < 1e-6f,
        "WindSystem deveria ser deterministico para o mesmo tempo decorrido");

    // deltaTime invalido (zero, negativo, NaN) nao avanca o relogio interno.
    WindSystem windStatic;
    const Vec3 beforeInvalidAdvance = windStatic.velocityAtHeight(10.0f);
    windStatic.advance(0.0f);
    windStatic.advance(-1.0f);
    windStatic.advance(std::numeric_limits<float>::quiet_NaN());
    const Vec3 afterInvalidAdvance = windStatic.velocityAtHeight(10.0f);
    require(std::abs(beforeInvalidAdvance.x - afterInvalidAdvance.x) < 1e-6f
        && std::abs(beforeInvalidAdvance.y - afterInvalidAdvance.y) < 1e-6f,
        "advance() com deltaTime invalido nao deveria alterar o vento");

    // Perfil por altura: com o mesmo estado (nenhum advance() entre as
    // amostras), uma altura maior sempre produz vento igual ou mais forte
    // que uma altura menor - rampa chao->ceu monotonica crescente (ver
    // WindSettings3D::groundHeightMeters/referenceHeightMeters).
    WindSystem windProfile;
    windProfile.advance(12.3f);
    const float lowSpeed = windProfile.velocityAtHeight(2.0f).length();
    const float referenceSpeed = windProfile.velocityAtHeight(10.0f).length();
    const float highSpeed = windProfile.velocityAtHeight(200.0f).length();
    require(lowSpeed <= referenceSpeed + 1e-5f
        && referenceSpeed <= highSpeed + 1e-5f,
        "Velocidade do vento deveria crescer (ou empatar) com a altura");

    // Acima da altura de referencia (default 10 m) a rampa satura: 200 m nao
    // deveria soprar mais forte que exatamente na altura de referencia.
    require(std::abs(highSpeed - referenceSpeed) < 1e-5f,
        "Rampa do vento deveria saturar na altura de referencia, nao crescer alem dela");

    // Resquicio raro de rajada forte perto do chao (ver WindSettings3D::
    // groundGustEventFrequencyHz): substitui a antiga garantia de zero
    // absoluto no chao - agora o chao pode ocasionalmente deixar passar um
    // resquicio de rajada forte, entao a garantia vira estatistica (quase
    // sempre praticamente parado, raramente um pico perceptivel) em vez de
    // um valor fixo.
    WindSystem windGround;
    std::vector<float> groundFractions;
    bool sawStrongResidue = false;
    for (int step = 0; step < 5000; ++step) {
        windGround.advance(10.0f); // 50000s simulados (~13h53min)
        const float groundSpeed = windGround.velocityAtHeight(0.0f).length();
        const float fullSpeed = windGround.velocityAtHeight(10.0f).length();
        require(groundSpeed <= fullSpeed + 1.0e-4f,
            "Resquicio no chao nao deveria superar a velocidade em altura plena");
        if (fullSpeed > 1.0e-4f) {
            const float fraction = groundSpeed / fullSpeed;
            groundFractions.push_back(fraction);
            if (fraction > 0.3f) sawStrongResidue = true;
        }
    }
    require(!groundFractions.empty(),
        "Amostragem do vento no chao nao deveria ficar vazia");
    std::vector<float> sortedFractions = groundFractions;
    std::sort(sortedFractions.begin(), sortedFractions.end());
    const float medianFraction = sortedFractions[sortedFractions.size() / 2];
    require(medianFraction < 0.05f,
        "Na mediana, o vento no chao (0-30cm) deveria estar praticamente parado");
    require(sawStrongResidue,
        "Deveria existir pelo menos um resquicio de rajada forte perceptivel no chao");

    // Envelope de calmaria/vento forte (ver WindSettings3D::
    // calmEnvelopeFrequencyHz): ao longo de tempo suficiente, deveria existir
    // pelo menos um trecho de calmaria clara E um trecho perto do pico
    // (base+rajada) na altura de referencia - nao so oscilar em torno da
    // media o tempo todo, que era o comportamento antigo (sem envelope).
    WindSystem windCalm;
    float minReferenceSpeed = std::numeric_limits<float>::max();
    float maxReferenceSpeed = 0.0f;
    for (int step = 0; step < 5000; ++step) {
        windCalm.advance(10.0f); // 50000s simulados
        const float speed = windCalm.velocityAtHeight(10.0f).length();
        minReferenceSpeed = std::min(minReferenceSpeed, speed);
        maxReferenceSpeed = std::max(maxReferenceSpeed, speed);
    }
    require(minReferenceSpeed < 1.0f,
        "Deveria existir pelo menos um trecho de calmaria clara");
    require(maxReferenceSpeed > 3.5f,
        "Deveria existir pelo menos um trecho de vento proximo do pico");

    // Velocidade sempre finita ao longo de uma janela de tempo razoavel -
    // nunca deveria produzir NaN/infinito, mesmo depois de muitos passos.
    WindSystem windSweep;
    for (int step = 0; step < 500; ++step) {
        windSweep.advance(0.37f);
        const Vec3 sample = windSweep.velocityAtHeight(10.0f);
        require(std::isfinite(sample.x) && std::isfinite(sample.y),
            "Velocidade do vento deveria ser sempre finita");
    }
}

void testOceanSurface() {
    const std::array<Vec2, 5> positions {{
        { 0.0f, 0.0f },
        { 12.5f, -8.0f },
        { -117.0f, 63.0f },
        { 480.0f, 205.0f },
        { -900.0f, -750.0f }
    }};
    bool changedOverTime = false;
    for (const Vec2 position : positions) {
        const OceanSurfaceSample3D atStart =
            evaluateOceanSurface(position, 0.0f, 0.0f);
        const OceanSurfaceSample3D later =
            evaluateOceanSurface(position, 0.0f, 1.25f);
        require(std::isfinite(atStart.heightMeters)
                && std::isfinite(atStart.verticalSpeedMetersPerSecond)
                && std::isfinite(atStart.normal.x)
                && std::isfinite(atStart.normal.y)
                && std::isfinite(atStart.normal.z),
            "Campo de ondas produziu valor nao finito");
        require(std::abs(atStart.heightMeters)
                <= OceanMaximumDisplacementMeters + 1e-5f,
            "Superficie oceanica ultrapassou seu deslocamento declarado");
        require(std::abs(atStart.normal.length() - 1.0f) < 1e-5f
                && atStart.normal.z > 0.98f,
            "Normal do oceano deveria ser unitaria e suave");
        changedOverTime = changedOverTime
            || std::abs(atStart.heightMeters
                    - later.heightMeters) > 1e-4f;
    }
    require(changedOverTime,
        "Campo de ondas deveria evoluir ao longo do tempo");
}

void testPerspectiveJittered() {
    const float fov = 60.0f * 3.14159265358979323846f / 180.0f;
    const Mat4 base = Mat4::perspective(fov, 16.0f / 9.0f, 0.1f, 100.0f);
    const Mat4 zeroJitter = Mat4::perspectiveJittered(fov, 16.0f / 9.0f,
        0.1f, 100.0f, { 0.0f, 0.0f });
    for (std::size_t index = 0; index < base.values.size(); ++index) {
        require(std::abs(base.values[index] - zeroJitter.values[index]) < 1e-6f,
            "perspectiveJittered com jitter zero deveria bater com perspective()");
    }

    const Vec2 jitter { 0.02f, -0.015f };
    const Mat4 jittered = Mat4::perspectiveJittered(fov, 16.0f / 9.0f,
        0.1f, 100.0f, jitter);
    // O offset cai exatamente nos elementos (0,2) e (1,2) - o termo que
    // multiplica viewZ nas linhas x/y, com sinal invertido (ver comentario
    // em Mat4::perspectiveJittered).
    require(std::abs(jittered.at(0, 2) - (-jitter.x)) < 1e-6f,
        "Jitter X nao caiu no elemento (0,2) esperado da matriz");
    require(std::abs(jittered.at(1, 2) - (-jitter.y)) < 1e-6f,
        "Jitter Y nao caiu no elemento (1,2) esperado da matriz");
    // Nenhum outro elemento deveria ter mudado.
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            if ((row == 0 && column == 2) || (row == 1 && column == 2)) continue;
            require(std::abs(jittered.at(row, column) - base.at(row, column)) < 1e-6f,
                "perspectiveJittered mudou um elemento fora de (0,2)/(1,2)");
        }
    }

    // inverse() continua fazendo round-trip com a matriz jitterada.
    const Mat4 roundTrip = jittered.inverse() * jittered;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            const float expected = row == column ? 1.0f : 0.0f;
            require(std::abs(roundTrip.at(row, column) - expected) < 1e-3f,
                "inverse() nao fez round-trip com a matriz jitterada");
        }
    }
}

MeshData3D makePlane(float halfExtent) {
    MeshData3D mesh;
    mesh.vertices = {
        { { -halfExtent, -halfExtent, 0.0f } },
        { { halfExtent, -halfExtent, 0.0f } },
        { { halfExtent, halfExtent, 0.0f } },
        { { -halfExtent, halfExtent, 0.0f } }
    };
    mesh.indices = { 0, 1, 2, 0, 2, 3 };
    recomputeBounds(mesh);
    return mesh;
}

void appendBox(MeshData3D& mesh, Vec3 center, Vec3 halfExtents) {
    const std::uint32_t base = static_cast<std::uint32_t>(
        mesh.vertices.size());
    const Vec3 p[] {
        center + Vec3 { -halfExtents.x, -halfExtents.y, -halfExtents.z },
        center + Vec3 { halfExtents.x, -halfExtents.y, -halfExtents.z },
        center + Vec3 { halfExtents.x, halfExtents.y, -halfExtents.z },
        center + Vec3 { -halfExtents.x, halfExtents.y, -halfExtents.z },
        center + Vec3 { -halfExtents.x, -halfExtents.y, halfExtents.z },
        center + Vec3 { halfExtents.x, -halfExtents.y, halfExtents.z },
        center + Vec3 { halfExtents.x, halfExtents.y, halfExtents.z },
        center + Vec3 { -halfExtents.x, halfExtents.y, halfExtents.z }
    };
    for (Vec3 position : p) mesh.vertices.push_back({ position });
    constexpr std::uint32_t indices[] {
        0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
        0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5,
        2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7
    };
    for (std::uint32_t index : indices) mesh.indices.push_back(base + index);
    recomputeBounds(mesh);
}

PhysicsBodyHandle3D createStaticGround(PhysicsEngine3D& engine,
    PhysicsScene3D& scene) {
    const auto mesh = engine.cookStaticTriangleMesh(makePlane(80.0f));
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::TriangleMesh;
    shape.mesh = mesh;
    shape.materialId = "concrete";
    PhysicsBodyDefinition3D body;
    body.motionType = PhysicsMotionType3D::Static;
    body.materialId = "concrete";
    return scene.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
}

PhysicsBodyHandle3D createBox(PhysicsScene3D& scene, Vec3 position,
    Vec3 halfExtents, float mass = 1.0f) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = halfExtents;
    shape.materialId = "wood";
    PhysicsBodyDefinition3D body;
    body.position = position;
    body.massKg = mass;
    body.materialId = "wood";
    body.characteristicSizeMeters = std::max({ halfExtents.x,
        halfExtents.y, halfExtents.z }) * 2.0f;
    return scene.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
}

PhysicsBodyHandle3D createStaticBox(PhysicsScene3D& scene, Vec3 position,
    Vec3 halfExtents) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = halfExtents;
    shape.materialId = "concrete";
    PhysicsBodyDefinition3D body;
    body.motionType = PhysicsMotionType3D::Static;
    body.position = position;
    body.materialId = "concrete";
    return scene.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
}

void testMetadataAndMaterials() {
    MaterialLibrary materials;
    require(materials.find("concrete") != nullptr,
        "Material concrete ausente");
    require(materials.find("soccer_ball") != nullptr,
        "Material soccer_ball ausente");

    GltfExtras extras;
    extras.values = {
        { "physical_material", "wood" },
        { "mass", "8.8" },
        { "collision_mode", "auto" },
        { "collision_hulls", "12" }
    };
    const GltfPhysicsMetadata3D metadata =
        parseGltfPhysicsMetadata(extras);
    require(metadata.materialId == "wood", "Material glTF incorreto");
    require(metadata.bodyBinding.massMode
        == BodyMassMode3D::OverrideKilograms, "mass nao ativou override");
    require(std::abs(metadata.bodyBinding.massOverrideKg - 8.8f) < 0.001f,
        "Massa glTF incorreta");
    require(metadata.maximumCollisionHulls == 12u,
        "Budget de hulls glTF incorreto");

    const PhysicsBodyDefinition3D body = buildDynamicBodyDefinition(
        *materials.find("wood"), metadata.bodyBinding, 0.2f,
        { 0.6f, 0.6f, 1.0f });
    require(std::abs(body.massKg - 8.8f) < 0.001f,
        "Builder alterou massa explicita");
    require(body.aerodynamicReferenceAreaSquareMeters > 0.0f,
        "Area aerodinamica nao foi preparada");
    require(body.collisionMode == PhysicsCollisionMode3D::Discrete,
        "Prop comum ativou CCD completo sem necessidade");
    const PhysicsBodyDefinition3D ball = buildDynamicBodyDefinition(
        *materials.find("soccer_ball"), {}, 0.005575f,
        { 0.22f, 0.22f, 0.22f });
    require(ball.collisionMode == PhysicsCollisionMode3D::Continuous,
        "Bola rapida perdeu a protecao de CCD");
}

void testAcousticZoneParsing() {
    LoadedGltfEntity spawnpoint;
    spawnpoint.name = "PlayerSpawn";
    spawnpoint.extras.values = { { "entity_type", "spawnpoint" } };

    LoadedGltfEntity zoneEntity;
    zoneEntity.name = "CaveZone";
    zoneEntity.position = { 4.0f, -2.0f, 1.5f };
    zoneEntity.extras.values = {
        { "entity_type", "acoustic_zone" },
        { "reverb_preset", "cave" },
        { "radius", "6.0" },
        { "blend_distance", "2.5" },
        { "wet_send", "0.8" }
    };

    const std::vector<AcousticZoneDefinition3D> zones =
        parseAcousticZones({ spawnpoint, zoneEntity });
    require(zones.size() == 1,
        "parseAcousticZones nao ignorou a entidade spawnpoint");
    require(zones[0].name == "CaveZone", "Nome da zona acustica perdido");
    require(zones[0].preset == AcousticReverbPreset3D::Cave,
        "reverb_preset nao foi lido corretamente");
    require(std::abs(zones[0].radiusMeters - 6.0f) < 0.001f,
        "radius da zona acustica incorreto");
    require(std::abs(zones[0].blendDistanceMeters - 2.5f) < 0.001f,
        "blend_distance da zona acustica incorreto");
    require(std::abs(zones[0].wetSendGain - 0.8f) < 0.001f,
        "wet_send da zona acustica incorreto");

    // Zona sem nenhum extra usa os defaults documentados.
    LoadedGltfEntity defaultZone;
    defaultZone.extras.values = { { "entity_type", "acoustic_zone" } };
    const std::vector<AcousticZoneDefinition3D> defaults =
        parseAcousticZones({ defaultZone });
    require(defaults.size() == 1, "Zona sem extras opcionais nao foi lida");
    require(defaults[0].preset == AcousticReverbPreset3D::Generic,
        "Preset default incorreto");

    LoadedGltfEntity malformedZone;
    malformedZone.extras.values = {
        { "entity_type", "acoustic_zone" }, { "radius", "abc" }
    };
    bool threw = false;
    try {
        static_cast<void>(parseAcousticZones({ malformedZone }));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "radius invalido nao lancou std::invalid_argument");
}

void testCookingConvexo(PhysicsEngine3D& engine) {
    // Tres barras formam um U concavo. Um hull unico fecharia o vazio; a
    // decomposicao precisa publicar mais de uma shape convexa.
    MeshData3D concave;
    appendBox(concave, { -0.8f, 0.0f, 0.5f }, { 0.2f, 0.25f, 0.5f });
    appendBox(concave, { 0.8f, 0.0f, 0.5f }, { 0.2f, 0.25f, 0.5f });
    appendBox(concave, { 0.0f, 0.0f, 0.1f }, { 0.8f, 0.25f, 0.1f });
    ConvexDecompositionSettings3D settings;
    settings.maximumHulls = 12;
    settings.voxelResolution = 50000;
    const std::filesystem::path cachePath =
        std::filesystem::current_path() / "matterengine_test.mecollider";
    std::error_code fileError;
    std::filesystem::remove(cachePath, fileError);
    const CookedDynamicCollision3D cooked =
        engine.cookDynamicCollision(concave, settings, cachePath.string());
    require(cooked.shapes.size() >= 2,
        "V-HACD fechou uma concavidade em um hull unico");
    require(cooked.shapes.size() <= settings.maximumHulls,
        "V-HACD excedeu o orcamento de hulls");
    require(std::filesystem::exists(cachePath),
        "Cooking nao publicou o cache versionado");
    const CookedDynamicCollision3D cached =
        engine.cookDynamicCollision(concave, settings, cachePath.string());
    require(cached.shapes.size() == cooked.shapes.size(),
        "Cache alterou o numero de hulls do collider");
    std::filesystem::remove(cachePath, fileError);
}

void testDiagnoseHullFit(PhysicsEngine3D& engine) {
    // Diagnostico opcional, sem efeito nas execucoes normais (so age se a
    // variavel de ambiente estiver definida): mede o volume do casco de
    // colisao cozido contra o volume da caixa delimitadora da malha visual,
    // para UM prop e UM valor de shrinkWrap por vez. Cada chamada roda
    // isolada (um processo por combinacao), porque um shrinkWrap problematico
    // pode falhar de um jeito nativo do V-HACD que nao e uma excecao C++
    // capturavel — testar tudo num processo so esconderia justamente o prop
    // culpado atras dos que rodaram antes dele.
    const char* fileName = std::getenv("MATTERENGINE_DIAGNOSE_PROP");
    if (fileName == nullptr) return;
    const bool shrinkWrap = std::getenv("MATTERENGINE_DIAGNOSE_SHRINKWRAP")
        != nullptr;

    const std::string path = std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/models/props/" + std::string(fileName);
    const LoadedGltfModel model = loadGltfModel(path);
    MeshData3D mesh;
    for (const LoadedGltfPart& part : model.parts) {
        const std::uint32_t base =
            static_cast<std::uint32_t>(mesh.vertices.size());
        mesh.vertices.insert(mesh.vertices.end(),
            part.mesh.vertices.begin(), part.mesh.vertices.end());
        for (const std::uint32_t index : part.mesh.indices) {
            mesh.indices.push_back(base + index);
        }
    }
    recomputeBounds(mesh);
    const Vec3 dimensions = mesh.boundsMax - mesh.boundsMin;
    const float boundingBoxVolume =
        dimensions.x * dimensions.y * dimensions.z;

    ConvexDecompositionSettings3D settings;
    settings.shrinkWrap = shrinkWrap;
    const CookedDynamicCollision3D cooked =
        engine.cookDynamicCollision(mesh, settings);

    std::cout << "[DIAGNOSTICO] prop=" << fileName
        << " shrinkWrap=" << (shrinkWrap ? "true" : "false")
        << " hulls=" << cooked.hullCount
        << " volumeCasco=" << cooked.volumeCubicMeters
        << " volumeCaixaVisual=" << boundingBoxVolume
        << " razao=" << (boundingBoxVolume > 0.0f
            ? cooked.volumeCubicMeters / boundingBoxVolume : 0.0f)
        << '\n';
}

void testRealPropCooking(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    // Regressao: cada prop publicado em assets/models/props/ (o catalogo real
    // que o Laboratorio cozinha ao abrir, via PropCatalog::load) precisa
    // decompor sua colisao dinamica sem excecao nem falha nativa do V-HACD.
    // Uma falha nativa nao e capturavel por try/catch em C++ e antes so
    // aparecia como a engine inteira travando em jogo, silenciosamente; este
    // teste roda o mesmo cozimento fora do jogo para pegar isso mais cedo.
    const std::vector<std::string> propFiles {
        "wood_chair.glb", "anvil.glb", "plastic_barrel.glb",
        "soccer_ball.glb", "wood_crate.glb", "car_tire.glb"
    };
    for (const std::string& fileName : propFiles) {
        const std::string path = std::string(MATTERENGINE_TEST_ASSETS_DIR)
            + "/models/props/" + fileName;
        const PhysicalAsset3D asset = loadPhysicalAsset3D(path, materials,
            engine);
        require(!asset.collision.shapes.empty(),
            ("Prop sem forma de colisao valida: " + fileName).c_str());
    }
}

void testRigidBodiesAndEvents(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    auto scene = engine.createScene({}, materials);
    createStaticGround(engine, *scene);
    const PhysicsBodyHandle3D falling = createBox(*scene,
        { 0.0f, 0.0f, 4.0f }, { 0.35f, 0.35f, 0.35f }, 3.0f);
    require(scene->overlapsBox({ 0.0f, 0.0f, 4.0f },
            { 0.40f, 0.40f, 0.40f }),
        "Consulta de spawn nao detectou corpo dinamico ocupado");
    require(scene->overlapsBox({ 4.0f, 0.0f, 0.12f },
            { 0.20f, 0.20f, 0.20f }),
        "Consulta de spawn nao detectou mundo estatico ocupado");
    require(!scene->overlapsBox({ 4.0f, 0.0f, 3.0f },
            { 0.20f, 0.20f, 0.20f }),
        "Consulta de spawn marcou volume realmente livre");
    bool receivedImpact = false;
    for (int step = 0; step < 720; ++step) {
        scene->simulate(1.0f / 120.0f);
        receivedImpact = receivedImpact || !scene->contactImpacts().empty();
    }
    const PhysicsBodyState3D state = scene->bodyState(falling);
    require(std::abs(state.position.z - 0.35f) < 0.08f,
        "Corpo nao repousou sobre a triangle mesh estatica");
    require(state.sleeping, "Corpo em repouso nao entrou em sleep");
    require(receivedImpact, "Callback nao publicou o impacto inicial");
    scene->simulate(1.0f / 120.0f);
    require(scene->contactImpacts().empty(),
        "Contato em repouso gerou spam de impactos");

    PhysicsRayHit3D hit;
    require(scene->raycast({ { 0.0f, 0.0f, 5.0f },
            { 0.0f, 0.0f, -1.0f } }, 10.0f, hit),
        "Raycast PhysX falhou");
    require(hit.body == falling, "Raycast nao retornou o corpo mais proximo");

    PhysicsRayHit3D dynamicHit;
    require(scene->raycastDynamic({ { 0.0f, 0.0f, 5.0f },
            { 0.0f, 0.0f, -1.0f } }, 10.0f, dynamicHit)
            && dynamicHit.body == falling,
        "Raycast dinamico nao encontrou o prop");
    PhysicsRayHit3D staticHit;
    require(scene->raycastStatic({ { 0.0f, 0.0f, 5.0f },
            { 0.0f, 0.0f, -1.0f } }, 10.0f, staticHit)
            && staticHit.body != falling,
        "Raycast estatico deveria encontrar somente o mundo");

    // Regressao da Physgun: o raio central passa ao lado da caixa e encontra
    // o piso, mas a esfera de assistencia ainda deve selecionar o prop
    // dinamico, sem permitir que o proprio piso roube o resultado.
    PhysicsRayHit3D assistedHit;
    require(scene->sweepSphereDynamic({ { 0.37f, 0.0f, 5.0f },
            { 0.0f, 0.0f, -1.0f } }, 0.06f, staticHit.distance,
            assistedHit)
            && assistedHit.body == falling,
        "Sweep dinamica da Physgun foi bloqueada pelo mundo estatico");

    scene->destroyBody(falling);
    require(!scene->contains(falling), "Handle destruido permaneceu valido");
    const PhysicsBodyHandle3D replacement = createBox(*scene,
        { 0.0f, 0.0f, 1.0f }, { 0.2f, 0.2f, 0.2f });
    require(replacement.index == falling.index
            && replacement.generation != falling.generation,
        "Slot reutilizado nao incrementou a geracao");
}

void testOceanPhysicsLifecycle(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    auto scene = engine.createScene({}, materials);
    const OceanVolume3D ocean {
        { 0.0f, 0.0f },
        { 10.0f, 10.0f },
        0.0f,
        5.0f,
        1025.0f
    };
    scene->setOcean(ocean);

    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = { 0.5f, 0.5f, 0.5f };
    shape.materialId = "wood";
    PhysicsBodyDefinition3D body;
    body.position = { 0.0f, 0.0f, -0.1f };
    body.massKg = 500.0f;
    body.bodyVolumeCubicMeters = 1.0f;
    body.materialId = "wood";
    body.aerodynamicReferenceAreaSquareMeters = 1.0f;
    const PhysicsBodyHandle3D floating =
        scene->createBody(body, std::span<const PhysicsShape3D>(&shape, 1));

    for (int step = 0; step < 24; ++step) {
        scene->setOceanTimeSeconds(static_cast<float>(step) / 120.0f);
        scene->simulate(1.0f / 120.0f);
    }
    const PhysicsBodyState3D state = scene->bodyState(floating);
    require(std::isfinite(state.position.x)
            && std::isfinite(state.position.y)
            && std::isfinite(state.position.z)
            && std::isfinite(state.linearVelocity.z),
        "Flutuacao produziu estado fisico invalido");

    // Regressao do crash original: o vetor de corpos flutuantes guardava um
    // ponteiro cru depois de destroyBody(), causando use-after-free no passo
    // seguinte da simulacao.
    scene->destroyBody(floating);
    scene->setOceanTimeSeconds(1.0f);
    scene->simulate(1.0f / 120.0f);
    require(!scene->contains(floating),
        "Corpo destruido permaneceu registrado na agua");

    scene->createCharacter({ 2.0f, 0.0f, -1.8f }, {});
    CharacterMotorCommand3D swimUp;
    swimUp.moveDirection = { 0.0f, 0.0f, 1.0f };
    scene->moveCharacter(swimUp, {}, 1.0f / 60.0f);
    require(scene->characterState().swimming,
        "Personagem submerso nao entrou no estado de natacao");
    const float initialSwimmerHeight = scene->characterState().position.z;
    for (int step = 0; step < 10; ++step) {
        scene->setOceanTimeSeconds(1.0f
            + static_cast<float>(step) / 60.0f);
        scene->moveCharacter(swimUp, {}, 1.0f / 60.0f);
    }
    require(scene->characterState().position.z > initialSwimmerHeight,
        "Comando vertical de natacao nao elevou o personagem");
}

void testWindShelterExposure(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    auto scene = engine.createScene({}, materials);
    // Parede estatica do lado +X: halfExtents pequeno em X (parede fina) e
    // generoso em Y/Z (cobre qualquer raio de amostragem sem precisar mirar
    // exato).
    createStaticBox(*scene, { 2.0f, 0.0f, 1.0f }, { 0.1f, 3.0f, 3.0f });
    // Vento soprando em -X, ou seja, vindo do lado +X - de onde a parede
    // esta. "A barlavento" de uma posicao e o lado de ONDE o vento vem, o
    // oposto do vetor de velocidade (ver windShelterExposure3D) - por isso o
    // vetor aponta para -X mesmo a parede estando em +X.
    const Vec3 windBlowingTowardNegativeX { -3.0f, 0.0f, 0.0f };

    // Bem perto da face da parede (0.2m) - smoothstep(0.2/4=0.05) ~ 0.7%,
    // bem dentro do limiar apertado abaixo. Mais longe (mas ainda dentro do
    // alcance de 4m) daria uma exposicao parcial pelo proprio design do
    // falloff suave, entao o teste precisa ficar perto o bastante da parede
    // pra provar "quase totalmente abrigado", nao so "existe alguma sombra".
    const float exposureBehindWall = windShelterExposure3D(*scene,
        { 1.7f, 0.0f, 1.0f }, windBlowingTowardNegativeX, 4.0f);
    require(exposureBehindWall < 0.05f,
        "Posicao encostada numa parede a barlavento deveria estar quase "
        "totalmente protegida do vento");

    const float exposureOpenAir = windShelterExposure3D(*scene,
        { -10.0f, 0.0f, 1.0f }, windBlowingTowardNegativeX, 4.0f);
    require(exposureOpenAir > 0.95f,
        "Posicao sem geometria a barlavento deveria ter exposicao quase "
        "total ao vento");

    const float exposureDisabled = windShelterExposure3D(*scene,
        { 0.0f, 0.0f, 1.0f }, windBlowingTowardNegativeX, 0.0f);
    require(exposureDisabled > 0.999f,
        "shelterDistanceMeters=0 deveria desligar a checagem de abrigo");

    const float exposureNoWind = windShelterExposure3D(*scene,
        { 0.0f, 0.0f, 1.0f }, Vec3 {}, 4.0f);
    require(exposureNoWind > 0.999f,
        "Vento com velocidade zero nao deveria disparar nenhum raycast");
}

void testContactSlideEvents(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    // eNOTIFY_TOUCH_PERSISTS (ver PhysXScene3D.cpp) reporta a cada passo
    // fixo enquanto dois corpos continuam se tocando - diferente de
    // contactImpacts(), que so dispara no instante em que o toque comeca.
    // Este teste cobre so a plumbing fisica (popular/esvaziar
    // contactSlides()); a formula de intensidade forca*velocidade em si e
    // testada isoladamente em AudioFoundationTests.cpp com eventos
    // sinteticos, no mesmo espirito de ImpactAcousticResolver.
    auto scene = engine.createScene({}, materials);
    createStaticGround(engine, *scene);
    const PhysicsBodyHandle3D box = createBox(*scene,
        { 0.0f, 0.0f, 4.0f }, { 0.35f, 0.35f, 0.35f }, 3.0f);

    // So checa ENQUANTO o corpo simula (dentro do loop, mesmo padrao de
    // "receivedImpact" em testRigidBodiesAndEvents) - depois de assentar,
    // o corpo entra em sleep (ver esse mesmo teste) e o PhysX para de
    // reportar TOUCH_PERSISTS para pares dormindo, entao conferir so no
    // final sempre daria vazio independente da funcionalidade estar certa.
    bool sawBoxInSlide = false;
    for (int step = 0; step < 720; ++step) {
        scene->simulate(1.0f / 120.0f);
        for (const ContactSlideEvent3D& slide : scene->contactSlides()) {
            if (slide.bodyA == box.index || slide.bodyB == box.index) {
                sawBoxInSlide = true;
            }
        }
    }
    require(sawBoxInSlide,
        "contactSlides() nunca incluiu o corpo enquanto ele assentava sobre "
        "geometria estatica");

    scene->destroyBody(box);
    scene->simulate(1.0f / 120.0f);
    require(scene->contactSlides().empty(),
        "contactSlides() deveria esvaziar apos o corpo ser removido");
}

void testCompoundSupport(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    auto scene = engine.createScene({}, materials);
    createStaticGround(engine, *scene);

    std::vector<PhysicsShape3D> chair;
    PhysicsShape3D seat;
    seat.type = PhysicsShapeType3D::Box;
    seat.halfExtents = { 0.65f, 0.65f, 0.10f };
    seat.localPosition = { 0.0f, 0.0f, 1.0f };
    seat.materialId = "wood";
    chair.push_back(seat);
    for (float x : { -0.52f, 0.52f }) {
        for (float y : { -0.52f, 0.52f }) {
            PhysicsShape3D leg;
            leg.type = PhysicsShapeType3D::Box;
            leg.halfExtents = { 0.07f, 0.07f, 0.45f };
            leg.localPosition = { x, y, 0.45f };
            leg.materialId = "wood";
            chair.push_back(leg);
        }
    }
    PhysicsBodyDefinition3D chairBody;
    chairBody.motionType = PhysicsMotionType3D::Static;
    chairBody.materialId = "wood";
    const PhysicsBodyHandle3D chairHandle = scene->createBody(chairBody, chair);
    require(chairHandle.valid(), "O corpo composto da cadeira deve ser criado.");
    const PhysicsBodyHandle3D crate = createBox(*scene,
        { 0.0f, 0.0f, 2.4f }, { 0.25f, 0.25f, 0.25f }, 2.0f);
    for (int step = 0; step < 720; ++step) {
        scene->simulate(1.0f / 120.0f);
    }
    const PhysicsBodyState3D state = scene->bodyState(crate);
    require(std::abs(state.position.z - 1.35f) < 0.08f,
        "Compound collider nao sustentou a caixa sobre o assento");
}

void testPhysGunDrive(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    auto scene = engine.createScene({}, materials);
    const PhysicsBodyHandle3D body = createBox(*scene,
        { 0.0f, 0.0f, 2.0f }, { 0.25f, 0.25f, 0.25f }, 4.0f);
    PhysicsGrabTarget3D target;
    target.position = { 2.0f, 0.0f, 2.0f };
    target.lockOrientation = true;
    PhysicsHandleSettings3D settings;
    settings.maximumForce = 20000.0f;
    require(scene->beginGrab(body, {}, target, settings),
        "D6 joint da Physgun nao foi criado");
    for (int step = 0; step < 240; ++step) {
        scene->updateGrabTarget(target, settings);
        scene->simulate(1.0f / 120.0f);
    }
    const PhysicsBodyState3D state = scene->bodyState(body);
    require(state.position.x > 1.5f && std::isfinite(state.position.x),
        "Drive da Physgun nao convergiu para o alvo");
    scene->endGrab();
}

void testCharacterController(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    auto scene = engine.createScene({}, materials);
    createStaticGround(engine, *scene);
    CharacterMotorSettings3D settings;
    scene->createCharacter({ 0.0f, 0.0f, 0.02f }, settings);
    CharacterMotorCommand3D command;
    command.moveDirection = { 1.0f, 0.0f, 0.0f };
    for (int step = 0; step < 120; ++step) {
        scene->moveCharacter(command, settings, 1.0f / 120.0f);
        scene->simulate(1.0f / 120.0f);
    }
    require(scene->characterState().position.x > 2.0f,
        "CCT nao moveu o personagem em terra");
    command = {};
    command.toggleFlight = true;
    scene->moveCharacter(command, settings, 1.0f / 120.0f);
    require(scene->characterState().flying,
        "CCT nao ativou o modo de voo");

    // Uma cobertura baixa permite agachar, mas nao levantar. Depois o CCT
    // atravessa a cobertura em voo sem colisao e recusa reativar o corpo na
    // posicao penetrada.
    scene->placeCharacter({ 0.0f, 0.0f, 0.02f }, settings);
    const PhysicsBodyHandle3D ceiling = createStaticBox(*scene,
        { 0.0f, 0.0f, 1.35f }, { 0.8f, 0.8f, 0.10f });
    require(ceiling.valid(), "Cobertura de teste nao foi criada");
    command = {};
    command.crouch = true;
    scene->moveCharacter(command, settings, 1.0f / 120.0f);
    require(scene->characterState().crouched,
        "CCT nao entrou em agachamento");
    command.crouch = false;
    scene->moveCharacter(command, settings, 1.0f / 120.0f);
    require(scene->characterState().crouched,
        "CCT levantou atravessando a cobertura");

    command = {};
    command.crouch = true;
    command.toggleFlight = true;
    scene->moveCharacter(command, settings, 1.0f / 120.0f);
    command.toggleFlight = false;
    command.moveDirection = { 1.0f, 0.0f, 0.0f };
    for (int step = 0; step < 20; ++step) {
        scene->moveCharacter(command, settings, 1.0f / 120.0f);
        scene->simulate(1.0f / 120.0f);
    }
    command = {};
    scene->moveCharacter(command, settings, 1.0f / 120.0f);
    require(!scene->characterState().crouched,
        "CCT nao levantou no espaco livre");
    command.moveDirection = { -1.0f, 0.0f, 0.0f };
    for (int step = 0; step < 20; ++step) {
        scene->moveCharacter(command, settings, 1.0f / 120.0f);
        scene->simulate(1.0f / 120.0f);
    }
    command = {};
    command.toggleFlight = true;
    scene->moveCharacter(command, settings, 1.0f / 120.0f);
    require(scene->characterState().flying
            && scene->characterState().flightExitBlocked,
        "CCT saiu do voo dentro da cobertura");
}

void testThousandSleepingBodies(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    auto scene = engine.createScene({}, materials);
    createStaticGround(engine, *scene);
    std::vector<PhysicsBodyHandle3D> bodies;
    bodies.reserve(1024);
    for (int y = 0; y < 32; ++y) {
        for (int x = 0; x < 32; ++x) {
            bodies.push_back(createBox(*scene,
                { static_cast<float>(x) * 0.72f,
                    static_cast<float>(y) * 0.72f, 0.22f },
                { 0.20f, 0.20f, 0.20f }, 1.0f));
        }
    }
    const auto start = std::chrono::steady_clock::now();
    std::uint32_t maximumWorkers = 0;
    for (int step = 0; step < 360; ++step) {
        scene->simulate(1.0f / 120.0f);
        maximumWorkers = std::max(maximumWorkers,
            scene->diagnostics().physicsWorkerCount);
    }
    const float seconds = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - start).count();
    const PhysicsStepDiagnostics3D& diagnostics = scene->diagnostics();
    require(diagnostics.dynamicBodyCount >= 1024,
        "Diagnostico perdeu corpos dinamicos");
    require(diagnostics.sleepingDynamicBodyCount >= 1000,
        "Islands em repouso nao foram desativadas pelo PhysX");
    require(maximumWorkers > 1,
        "Carga de 1024 props nao escalou o dispatcher adaptativo");
    require(diagnostics.ccdPairs == 0,
        "Props discretos pagaram pelo caminho CCD reservado a corpos rapidos");
    // Limite amplo para Debug/CI: detecta regressao catastrofica, nao tenta
    // transformar uma maquina especifica em benchmark universal.
    require(seconds < 30.0f,
        "Simulacao de 1024 props excedeu o limite de regressao");
}

void testBackendResourceLifetime() {
    MaterialLibrary materials;
    auto engine = std::make_unique<PhysicsEngine3D>();
    auto scene = engine->createScene({}, materials);
    const auto groundMesh = engine->cookStaticTriangleMesh(makePlane(4.0f));
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::TriangleMesh;
    shape.mesh = groundMesh;
    shape.materialId = "concrete";
    PhysicsBodyDefinition3D body;
    body.motionType = PhysicsMotionType3D::Static;
    body.materialId = "concrete";
    require(scene->createBody(body,
        std::span<const PhysicsShape3D>(&shape, 1)).valid(),
        "Cena de lifetime nao criou o mundo estatico");

    // Cenas e recursos cozidos conservam internamente o contexto nativo. A
    // fachada pode sair de escopo sem deixar ponteiros pendentes no backend.
    engine.reset();
    scene->simulate(1.0f / 120.0f);
}

void testShadowCascadeSplits() {
    // Parametros invalidos produzem vetor vazio, sem travar.
    require(computeCascadeSplits(0.1f, 300.0f, 0, 0.5f).empty(),
        "cascadeCount==0 deveria produzir vetor vazio");
    require(computeCascadeSplits(0.0f, 300.0f, 4, 0.5f).empty(),
        "nearPlane<=0 deveria produzir vetor vazio");
    require(computeCascadeSplits(10.0f, 5.0f, 4, 0.5f).empty(),
        "farPlane<=nearPlane deveria produzir vetor vazio");

    // lambda=0 (so uniforme): splits igualmente espacados.
    const auto uniformSplits = computeCascadeSplits(10.0f, 110.0f, 4, 0.0f);
    require(uniformSplits.size() == 4, "Contagem de splits nao bateu");
    require(std::abs(uniformSplits[0] - 35.0f) < 1e-3f,
        "Split uniforme 1/4 deveria ser near + (far-near)*0.25");
    require(std::abs(uniformSplits[3] - 110.0f) < 1e-3f,
        "Ultimo split deveria ser exatamente farPlane");

    // lambda=1 (so logaritmico): splits[i] = near * (far/near)^(i/N).
    const auto logSplits = computeCascadeSplits(10.0f, 160.0f, 4, 1.0f);
    require(std::abs(logSplits[0] - 10.0f * std::pow(16.0f, 0.25f)) < 1e-2f,
        "Split logaritmico 1/4 nao bateu com a formula esperada");
    require(std::abs(logSplits[3] - 160.0f) < 1e-2f,
        "Ultimo split logaritmico deveria ser exatamente farPlane");

    // Qualquer lambda: monotonico crescente, e o ultimo bate com farPlane.
    const auto blendedSplits = computeCascadeSplits(0.5f, 300.0f, 4, 0.5f);
    for (std::size_t i = 1; i < blendedSplits.size(); ++i) {
        require(blendedSplits[i] > blendedSplits[i - 1],
            "Splits deveriam ser estritamente crescentes");
    }
    require(std::abs(blendedSplits.back() - 300.0f) < 1e-2f,
        "Ultimo split deveria ser exatamente farPlane");
}

// Transforma um ponto pela matriz e devolve coordenadas NDC (divisao por w
// ja aplicada) - so usado por este teste, pra verificar que o frustum
// ajustado realmente contem os cantos que deveria cobrir.
Vec3 transformToNdc(const Mat4& viewProjection, Vec3 point) {
    const float x = viewProjection.at(0, 0) * point.x
        + viewProjection.at(0, 1) * point.y + viewProjection.at(0, 2) * point.z
        + viewProjection.at(0, 3);
    const float y = viewProjection.at(1, 0) * point.x
        + viewProjection.at(1, 1) * point.y + viewProjection.at(1, 2) * point.z
        + viewProjection.at(1, 3);
    const float z = viewProjection.at(2, 0) * point.x
        + viewProjection.at(2, 1) * point.y + viewProjection.at(2, 2) * point.z
        + viewProjection.at(2, 3);
    const float w = viewProjection.at(3, 0) * point.x
        + viewProjection.at(3, 1) * point.y + viewProjection.at(3, 2) * point.z
        + viewProjection.at(3, 3);
    const float safeW = std::abs(w) > 1e-8f ? w : 1.0f;
    return { x / safeW, y / safeW, z / safeW };
}

void testFitCascadeFrustumToCamera() {
    CameraFrustumParameters3D camera;
    camera.position = { 0.0f, 0.0f, 2.0f };
    camera.forward = { 1.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 0.0f, 1.0f };
    camera.verticalFovRadians = 67.0f * 3.14159265358979323846f / 180.0f;
    camera.aspectRatio = 16.0f / 9.0f;
    const Vec3 lightDirection = Vec3 { -0.3f, -0.2f, -0.9f }.normalized();

    const FittedShadowCascade fitted = fitCascadeFrustumToCamera(camera,
        0.5f, 40.0f, lightDirection, 2048.0f);
    const Mat4 viewProjection = fitted.viewProjection;
    require(fitted.texelWorldSizeMeters > 0.0f,
        "Cascata ajustada deveria informar tamanho de texel positivo");
    require(fitted.depthRangeMeters > 40.0f - 0.5f,
        "Volume da sombra deveria reservar profundidade alem da fatia "
        "visivel para casters posicionados na direcao da luz");

    // Reconstroi os mesmos 8 cantos que a funcao deveria ter coberto e
    // confirma que TODOS caem dentro da caixa NDC (x,y em [-1,1], z em
    // [0,1] - Mat4::orthographic fica em Z padrao, nao invertido, ver
    // Mat4.hpp) - a propriedade que realmente importa: o frustum ajustado
    // de fato contem o sub-frustum que foi pedido pra cobrir. Tolerancia de
    // 0.01 absorve so erro de ponto flutuante, nao folga de projeto.
    const float tanHalfFov = std::tan(camera.verticalFovRadians * 0.5f);
    const Vec3 forward = camera.forward;
    const Vec3 right = cross(forward, camera.up).normalized();
    const Vec3 up = cross(right, forward).normalized();
    for (float distance : { 0.5f, 40.0f }) {
        const float halfHeight = tanHalfFov * distance;
        const float halfWidth = halfHeight * camera.aspectRatio;
        for (float rightSign : { -1.0f, 1.0f }) {
            for (float upSign : { -1.0f, 1.0f }) {
                const Vec3 corner = camera.position + forward * distance
                    + right * (halfWidth * rightSign)
                    + up * (halfHeight * upSign);
                const Vec3 ndc = transformToNdc(viewProjection, corner);
                require(ndc.x >= -1.01f && ndc.x <= 1.01f,
                    "Canto do sub-frustum caiu fora da caixa ajustada em X");
                require(ndc.y >= -1.01f && ndc.y <= 1.01f,
                    "Canto do sub-frustum caiu fora da caixa ajustada em Y");
                require(ndc.z >= -0.01f && ndc.z <= 1.01f,
                    "Canto do sub-frustum caiu fora da caixa ajustada em Z");
            }
        }
    }

    // Movimento menor que um texel na base da luz nao pode deslizar a
    // projecao continuamente. Escolhemos o sentido que permanece dentro da
    // mesma celula de arredondamento para tornar o teste independente da
    // posicao absoluta da camera.
    const Vec3 lightForward = (-lightDirection).normalized();
    const Vec3 lightRight = cross(lightForward,
        Vec3 { 0.0f, 0.0f, 1.0f }).normalized();
    const Vec3 sliceCenter = camera.position
        + camera.forward.normalized() * ((0.5f + 40.0f) * 0.5f);
    const float centerInTexels = dot(sliceCenter, lightRight)
        / fitted.texelWorldSizeMeters;
    const float roundingResidual = centerInTexels
        - std::round(centerInTexels);
    CameraFrustumParameters3D movedCamera = camera;
    const float movementSign = roundingResidual >= 0.0f ? -1.0f : 1.0f;
    movedCamera.position += lightRight
        * (fitted.texelWorldSizeMeters * 0.20f * movementSign);
    const FittedShadowCascade moved = fitCascadeFrustumToCamera(movedCamera,
        0.5f, 40.0f, lightDirection, 2048.0f);
    const Vec3 referencePoint { 5.0f, 2.0f, 0.0f };
    const Vec3 originalNdc = transformToNdc(fitted.viewProjection,
        referencePoint);
    const Vec3 movedNdc = transformToNdc(moved.viewProjection,
        referencePoint);
    require(std::abs(originalNdc.x - movedNdc.x) < 1e-5f
            && std::abs(originalNdc.y - movedNdc.y) < 1e-5f,
        "Movimento sub-texel da camera deslizou a projecao da sombra; "
        "o centro precisa ser quantizado em coordenadas absolutas da luz");
}

void testCascadeFrustumContainsNearbyProp() {
    // Reproduz o cenario real que expos um bug de convencao de sinal: a
    // camera do Laboratorio (posicao/orientacao tipicas) e a direcao real
    // do sol (m_laboratorySunDirection em WorkbenchApp.hpp) - lightDirection
    // aqui segue a MESMA convencao do resto do motor (aponta PRA luz, Z
    // positivo = sol acima), nao a direcao que a luz viaja. Um objeto bem
    // na frente da camera, a poucos metros, PRECISA cair dentro do frustum
    // da cascata mais proxima - senao ele nunca projeta sombra (exatamente
    // o sintoma relatado: "nem os objetos pegam sombra").
    CameraFrustumParameters3D camera;
    camera.position = { 0.0f, -10.0f, 1.7f };
    camera.forward = { 0.0f, 1.0f, 0.0f };
    camera.up = { 0.0f, 0.0f, 1.0f };
    camera.verticalFovRadians = 67.0f * 3.14159265358979323846f / 180.0f;
    camera.aspectRatio = 16.0f / 9.0f;
    const Vec3 sunDirection = Vec3 { -0.44f, -0.31f, 0.84f }.normalized();

    const Mat4 nearestCascade = fitCascadeFrustumToCamera(camera, 0.08f,
        40.0f, sunDirection, 2048.0f).viewProjection;
    const Frustum3D frustum = Frustum3D::fromViewProjection(nearestCascade,
        /*reversedDepth=*/false);

    // Um prop tipico (~0.5m de raio) a 3m na frente da camera, bem dentro
    // do intervalo [0.08, 40] que a cascata deveria cobrir.
    const Vec3 propPosition = camera.position + camera.forward * 3.0f;
    require(frustum.intersectsSphere(propPosition, 0.5f),
        "Prop bem na frente da camera, dentro do alcance da cascata, "
        "deveria estar coberto pelo frustum ajustado - se isto falhar, "
        "a camera de sombra provavelmente esta posicionada do lado errado "
        "da cena (ver o comentario sobre inversao de lightDirection em "
        "ShadowCascade.cpp)");

    // A propria camera (o "centro" do sub-frustum, aproximadamente) tambem
    // precisa estar coberta - outra checagem barata contra o mesmo tipo de
    // inversao.
    const Vec3 midPoint = camera.position + camera.forward * 20.0f;
    require(frustum.intersectsSphere(midPoint, 1.0f),
        "Ponto no meio do sub-frustum da camera deveria estar coberto "
        "pelo frustum ajustado");
}

void testRagdollProfileAndRuntime(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    const std::string path = std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/physics/ragdolls/HumanAdultV1.ragdoll.json";
    const RagdollProfile3D profile = loadRagdollProfile3D(path);
    require(profile.links.size() == 18,
        "HumanAdultV1 deveria conter dezoito links");
    require(ragdollDegreesOfFreedom3D(profile) == 41,
        "HumanAdultV1 deveria conter quarenta e um DOFs");
    require(validateRagdollProfile3D(profile).empty(),
        "HumanAdultV1 não passou na validação");
    const auto findLink = [&](std::string_view id) {
        return std::find_if(profile.links.begin(), profile.links.end(),
            [&](const RagdollLinkDefinition3D& link) {
                return link.id == id;
            });
    };
    require(findLink("LeftToes") == profile.links.end()
            && findLink("RightToes") == profile.links.end(),
        "Perfil polido não deveria recriar ossos separados nos dedos");
    const auto leftThigh = findLink("LeftThigh");
    const auto rightThigh = findLink("RightThigh");
    require(leftThigh != profile.links.end()
            && rightThigh != profile.links.end()
            && leftThigh->modelPosition.y >= 0.089f
            && rightThigh->modelPosition.y <= -0.089f,
        "Abertura mínima do quadril foi perdida");
    for (const RagdollLinkDefinition3D& link : profile.links) {
        if (link.collider.shape == RagdollColliderShape3D::Capsule) {
            require(std::abs(link.collider.radiusMeters
                    - profile.uniformRadiusMeters) < 1.0e-6f,
                "Uma cápsula do HumanAdultV1 perdeu o raio uniforme");
        } else {
            require((link.id == "LeftFoot" || link.id == "RightFoot")
                    && link.collider.boxHalfExtents.x >= 0.14f
                    && link.collider.boxHalfExtents.y >= 0.055f
                    && link.collider.contactSensor,
                "Collider funcional dos pés não preservou suporte/sensor");
        }
    }

    PhysicsSceneSettings3D settings;
    settings.solverPositionIterations = 4;
    settings.solverVelocityIterations = 1;
    auto scene = engine.createScene(settings, materials);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 9001;
    spawn.pelvisPosition = { 0.0f, 0.0f, 2.0f };
    spawn.active = false;
    const RagdollHandle3D handle = scene->createRagdoll(profile, spawn);
    require(handle && scene->contains(handle),
        "Cena não registrou o ragdoll criado");
    RagdollState3D state = scene->ragdollState(handle);
    require(state.links.size() == 18,
        "Snapshot do ragdoll perdeu links");

    PhysicsRagdollRayHit3D ragdollHit;
    require(scene->raycastRagdoll(
            { { 0.0f, -2.0f, 2.0f }, { 0.0f, 1.0f, 0.0f } },
            4.0f, ragdollHit)
            && ragdollHit.ragdoll == handle,
        "Raycast dedicado não encontrou um link do ragdoll");
    require(ragdollHit.linkIndex < state.links.size(),
        "Raycast publicou índice de link inválido");
    const PhysicsBodyState3D& grabbedLink =
        state.links[ragdollHit.linkIndex];
    const Vec3 localGrabPoint =
        grabbedLink.orientation.conjugate().rotate(
            ragdollHit.position - grabbedLink.position);
    PhysicsGrabTarget3D grabTarget;
    grabTarget.position = ragdollHit.position;
    grabTarget.orientation = grabbedLink.orientation;
    PhysicsHandleSettings3D grabSettings;
    require(scene->beginRagdollGrab(handle, ragdollHit.linkIndex,
            localGrabPoint, grabTarget, grabSettings),
        "D6 da Physgun não foi anexado ao link do ragdoll");
    require(scene->grabbing() && scene->grabbedRagdoll() == handle
            && scene->grabbedRagdollLink() == ragdollHit.linkIndex,
        "Cena não publicou o link de ragdoll agarrado");
    grabTarget.position.z += 0.20f;
    scene->updateGrabTarget(grabTarget, grabSettings);
    scene->simulate(1.0f / 120.0f);
    scene->endGrab();
    require(!scene->grabbing(),
        "D6 da Physgun não foi removido ao soltar o ragdoll");

    state = scene->ragdollState(handle);
    const float initialPelvisHeight = state.links.front().position.z;
    for (int step = 0; step < 12; ++step) scene->simulate(1.0f / 120.0f);
    state = scene->ragdollState(handle);
    require(state.links.front().position.z < initialPelvisHeight,
        "Raiz flutuante do ragdoll não respondeu à gravidade");

    scene->setRagdollRigidity(handle, 55.0f);
    scene->simulate(1.0f / 120.0f);
    require(std::abs(scene->ragdollState(handle).rigidityPercent - 55.0f)
            < 0.001f,
        "Comando seguro não aplicou rigidez ao ragdoll");
    scene->captureRagdollPose(handle);
    scene->setRagdollNeutralPose(handle);
    scene->releaseRagdollDrives(handle);
    scene->simulate(1.0f / 120.0f);
    require(scene->ragdollState(handle).rigidityPercent == 0.0f,
        "Soltar não removeu os drives do ragdoll");

    state = scene->ragdollState(handle);
    grabTarget.position = state.links.front().position;
    grabTarget.orientation = state.links.front().orientation;
    require(scene->beginRagdollGrab(handle, 0, {}, grabTarget,
            grabSettings),
        "Não foi possível repetir o grab antes da destruição");
    scene->destroyRagdoll(handle);
    require(!scene->contains(handle),
        "Handle de ragdoll continuou válido após destruição");
    require(!scene->grabbing(),
        "Destruir ragdoll não liberou o D6 ativo da Physgun");

    // Contrato de escala mínimo do jogo: os 22 jogadores continuam sendo
    // articulations completas, em contato com o chão e entre si. Este teste
    // não usa tolerância de FPS dependente da máquina; ele protege aquilo
    // que interessa à correção: solver finito, juntas coesas, contatos
    // reais e nenhum stream acústico desperdiçado nos links dos ragdolls.
    createStaticGround(engine, *scene);
    std::vector<RagdollHandle3D> team;
    team.reserve(22);
    for (std::size_t index = 0; index < 22; ++index) {
        const std::size_t column = index % 11;
        const std::size_t row = index / 11;
        RagdollSpawnDefinition3D player;
        player.entityId = 10'000 + index;
        player.pelvisPosition = {
            (static_cast<float>(column) - 5.0f) * 0.78f,
            (static_cast<float>(row) - 0.5f) * 0.78f,
            profile.standingRootHeightMeters + 0.04f
        };
        player.rigidityPercent = 20.0f;
        player.active = false;
        team.push_back(scene->createRagdoll(profile, player));
    }

    std::size_t maximumContacts = 0;
    float maximumAnchorSeparation = 0.0f;
    for (int step = 0; step < 240; ++step) {
        scene->simulate(1.0f / 120.0f);
        maximumContacts = std::max(maximumContacts,
            scene->diagnostics().discreteContactPairs);
        require(scene->diagnostics().reportedContactPairs == 0,
            "Links de ragdoll voltaram a produzir relatórios acústicos");

        for (RagdollHandle3D player : team) {
            const RagdollState3D playerState =
                scene->ragdollState(player);
            require(playerState.links.size() == profile.links.size(),
                "Stress de 22 ragdolls perdeu links");
            for (const PhysicsBodyState3D& link : playerState.links) {
                require(std::isfinite(link.position.x)
                        && std::isfinite(link.position.y)
                        && std::isfinite(link.position.z)
                        && std::isfinite(link.orientation.x)
                        && std::isfinite(link.orientation.y)
                        && std::isfinite(link.orientation.z)
                        && std::isfinite(link.orientation.w),
                    "Stress de 22 ragdolls produziu pose não finita");
            }
            for (std::size_t linkIndex = 1;
                    linkIndex < profile.links.size(); ++linkIndex) {
                const RagdollLinkDefinition3D& childDefinition =
                    profile.links[linkIndex];
                const std::size_t parentIndex =
                    static_cast<std::size_t>(
                        childDefinition.parentIndex);
                const RagdollLinkDefinition3D& parentDefinition =
                    profile.links[parentIndex];
                const PhysicsBodyState3D& parentState =
                    playerState.links[parentIndex];
                const PhysicsBodyState3D& childState =
                    playerState.links[linkIndex];
                const Vec3 parentLocalAnchor =
                    parentDefinition.modelOrientation.conjugate().rotate(
                        childDefinition.inboundJoint.anchorModelPosition
                            - parentDefinition.modelPosition);
                const Vec3 childLocalAnchor =
                    childDefinition.modelOrientation.conjugate().rotate(
                        childDefinition.inboundJoint.anchorModelPosition
                            - childDefinition.modelPosition);
                const Vec3 parentAnchor = parentState.position
                    + parentState.orientation.rotate(parentLocalAnchor);
                const Vec3 childAnchor = childState.position
                    + childState.orientation.rotate(childLocalAnchor);
                maximumAnchorSeparation = std::max(
                    maximumAnchorSeparation,
                    (parentAnchor - childAnchor).length());
            }
        }
    }
    require(maximumContacts > 0,
        "Stress de 22 ragdolls não exerceu contatos");
    require(maximumAnchorSeparation < 0.015f,
        "TGS 4/1 não manteve as juntas coesas no stress de 22 ragdolls");

    // Caso adversarial da Physgun: tenta arrastar o centro do braço superior
    // esquerdo para dentro do peito por um segundo. UpperArm e UpperChest sao
    // pai/filho e nunca colidem no PhysX; o Chest (avo) precisa impedir a
    // travessia sem responder com uma explosao de velocidade.
    auto penetrationScene = engine.createScene(settings, materials);
    RagdollSpawnDefinition3D penetrationSpawn;
    penetrationSpawn.entityId = 20'001;
    penetrationSpawn.pelvisPosition = { 0.0f, 0.0f, 2.0f };
    penetrationSpawn.active = false;
    const RagdollHandle3D penetrationRagdoll =
        penetrationScene->createRagdoll(profile, penetrationSpawn);
    const std::size_t chestIndex = static_cast<std::size_t>(
        std::distance(profile.links.begin(), findLink("Chest")));
    const std::size_t upperArmIndex = static_cast<std::size_t>(
        std::distance(profile.links.begin(), findLink("LeftUpperArm")));
    const auto capsuleAxis = [&](const RagdollState3D& ragdollState,
            std::size_t linkIndex) {
        const RagdollLinkDefinition3D& definition =
            profile.links[linkIndex];
        const PhysicsBodyState3D& state = ragdollState.links[linkIndex];
        const float halfHeight = std::max(0.0f,
            definition.collider.lengthMeters * 0.5f
                - definition.collider.radiusMeters);
        const Vec3 localAxis = definition.collider.localOrientation.rotate(
            { halfHeight, 0.0f, 0.0f });
        const Vec3 center = state.position + state.orientation.rotate(
            definition.collider.localPosition);
        const Vec3 worldAxis = state.orientation.rotate(localAxis);
        return std::pair { center - worldAxis, center + worldAxis };
    };
    const auto segmentDistance = [](Vec3 p1, Vec3 q1,
            Vec3 p2, Vec3 q2) {
        constexpr float Epsilon = 1.0e-8f;
        const Vec3 d1 = q1 - p1;
        const Vec3 d2 = q2 - p2;
        const Vec3 r = p1 - p2;
        const float a = dot(d1, d1);
        const float e = dot(d2, d2);
        const float f = dot(d2, r);
        float s = 0.0f;
        float t = 0.0f;
        if (a <= Epsilon && e <= Epsilon) return r.length();
        if (a <= Epsilon) {
            t = std::clamp(f / e, 0.0f, 1.0f);
        } else {
            const float c = dot(d1, r);
            if (e <= Epsilon) {
                s = std::clamp(-c / a, 0.0f, 1.0f);
            } else {
                const float b = dot(d1, d2);
                const float denominator = a * e - b * b;
                if (denominator > Epsilon) {
                    s = std::clamp((b * f - c * e) / denominator,
                        0.0f, 1.0f);
                }
                t = (b * s + f) / e;
                if (t < 0.0f) {
                    t = 0.0f;
                    s = std::clamp(-c / a, 0.0f, 1.0f);
                } else if (t > 1.0f) {
                    t = 1.0f;
                    s = std::clamp((b - c) / a, 0.0f, 1.0f);
                }
            }
        }
        return ((p1 + d1 * s) - (p2 + d2 * t)).length();
    };
    RagdollState3D penetrationState =
        penetrationScene->ragdollState(penetrationRagdoll);
    PhysicsGrabTarget3D penetrationTarget;
    penetrationTarget.position = penetrationState.links[chestIndex].position;
    PhysicsHandleSettings3D adversarialGrab;
    adversarialGrab.maximumForce = 500'000.0f;
    require(penetrationScene->beginRagdollGrab(penetrationRagdoll,
            static_cast<std::uint32_t>(upperArmIndex), {},
            penetrationTarget, adversarialGrab),
        "Teste de penetracao nao conseguiu agarrar o braco");
    float maximumLinkSpeed = 0.0f;
    float maximumLinkAngularSpeed = 0.0f;
    for (int step = 0; step < 120; ++step) {
        penetrationState =
            penetrationScene->ragdollState(penetrationRagdoll);
        penetrationTarget.position =
            penetrationState.links[chestIndex].position;
        penetrationScene->updateGrabTarget(
            penetrationTarget, adversarialGrab);
        penetrationScene->simulate(1.0f / 120.0f);
        penetrationState =
            penetrationScene->ragdollState(penetrationRagdoll);
        for (const PhysicsBodyState3D& link : penetrationState.links) {
            maximumLinkSpeed = std::max(maximumLinkSpeed,
                link.linearVelocity.length());
            maximumLinkAngularSpeed = std::max(maximumLinkAngularSpeed,
                link.angularVelocity.length());
        }
    }
    penetrationScene->endGrab();
    require(maximumLinkSpeed < 20.0f,
        "Physgun injetou velocidade linear explosiva no ragdoll");
    require(maximumLinkAngularSpeed < 30.0f,
        "Contato interno produziu giro explosivo no ragdoll");
    require(penetrationScene->diagnostics().discreteContactPairs > 0,
        "Barreira interna do torax nao produziu contato no teste adversarial");
    const auto chestAxis = capsuleAxis(penetrationState, chestIndex);
    const auto upperArmAxis = capsuleAxis(
        penetrationState, upperArmIndex);
    const float axisDistance = segmentDistance(chestAxis.first,
        chestAxis.second, upperArmAxis.first, upperArmAxis.second);
    const float combinedRadius =
        profile.links[chestIndex].collider.radiusMeters
        + profile.links[upperArmIndex].collider.radiusMeters;
    require(axisDistance >= combinedRadius - 0.012f,
        "Physgun conseguiu manter o braco penetrado dentro do peito");

    // Primeiro marco do controlador ativo: todas as ações chegam ao PhysX
    // como drives/torques articulares; o teste não prende nem força a raiz.
    // Chão de malha triangular real (como o mapa do laboratório), não uma
    // caixa: colisão malha-vs-forma tem geração de contato mais ruidosa que
    // caixa-vs-caixa, e um controlador validado só contra createStaticBox
    // nunca foi exercitado contra essa fonte real de ruído.
    auto activeScene = engine.createScene(settings, materials);
    createStaticGround(engine, *activeScene);
    RagdollSpawnDefinition3D activeSpawn;
    activeSpawn.entityId = 30'001;
    activeSpawn.pelvisPosition = {
        0.0f, 0.0f, profile.standingRootHeightMeters + 0.055f
    };
    activeSpawn.active = true;
    // No laboratório real o personagem nasce virado para onde a câmera
    // olhava — um yaw arbitrário, quase nunca alinhado aos eixos do mundo.
    // Testar sempre com orientação identidade (yaw 0) nunca exercitou essa
    // caixa delimitadora alinhada aos eixos do mundo (supportMinimum/
    // supportMaximum) fora do caso alinhado.
    activeSpawn.orientation = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, 0.61f);
    const RagdollHandle3D activeRagdoll =
        activeScene->createRagdoll(profile, activeSpawn);
    ActiveRagdollController3D controller;
    controller.reset(profile, activeSpawn.orientation);
    controller.config().useMagicPelvisStabilization = false;
    controller.config().useWholeBodyDynamics = true;
    RealisticFootworkTerrain activeTerrain(*activeScene);
    const auto updateActiveController =
        [&](const RagdollState3D& controllerState) {
            const RagdollDynamics3D dynamics =
                activeScene->ragdollDynamics(activeRagdoll);
            controller.update(profile, controllerState,
                1.0f / 120.0f, &activeTerrain, &dynamics);
        };
    for (int step = 0; step < 720; ++step) {
        updateActiveController(
            activeScene->ragdollState(activeRagdoll));
        const ActiveRagdollControlOutput3D& output =
            controller.output();
        if (output.applyRootForce) {
            activeScene->applyRagdollRootForce(activeRagdoll,
                output.rootForceNewtons,
                output.rootTorqueNewtonMeters);
        }
        if (output.applySpineForce) {
            activeScene->applyRagdollLinkForce(activeRagdoll,
                output.spineLinkIndex, output.spineForceNewtons,
                output.spineTorqueNewtonMeters);
        }
        activeScene->setRagdollActiveDriveTargets(activeRagdoll,
            output.driveTargets, output.gravityCompensationEnabled);
        activeScene->simulate(1.0f / 120.0f);
        if (std::getenv("MATTERENGINE_WBC_TRACE") != nullptr
            && step % 60 == 0) {
            const ActiveRagdollTelemetry3D& trace =
                controller.telemetry();
            std::cerr << "[wbc " << step << "] solved="
                      << trace.wholeBodySolved
                      << " residual=" << trace.wholeBodyDynamicsResidual
                      << " maxTau="
                      << trace.wholeBodyMaximumTorqueNewtonMeters
                      << " grf=(" << trace.leftGroundReactionNewtons
                      << ", " << trace.rightGroundReactionNewtons << ")"
                      << " up=" << trace.bodyUpDot
                      << " neck=" << trace.neckUpDot
                      << " head=" << trace.headUpDot
                      << " comZ=" << trace.centerOfMass.z
                      << " contacts=" << trace.supportContactCount
                      << " phase=" << static_cast<int>(trace.phase)
                      << " recovery=" << trace.recoveryStepActive
                      << " margin=" << trace.captureMarginMeters
                      << " comXY=(" << trace.centerOfMass.x << ", "
                      << trace.centerOfMass.y << ") supportXY=("
                      << trace.centerOfPressure.x << ", "
                      << trace.centerOfPressure.y << ") vXY=("
                      << trace.centerOfMassVelocity.x << ", "
                      << trace.centerOfMassVelocity.y << ")"
                      << " rootA*=("
                      << trace.desiredRootLinearAcceleration.x << ", "
                      << trace.desiredRootLinearAcceleration.y << ", "
                      << trace.desiredRootLinearAcceleration.z << ")"
                      << " rootA=("
                      << trace.solvedRootLinearAcceleration.x << ", "
                      << trace.solvedRootLinearAcceleration.y << ", "
                      << trace.solvedRootLinearAcceleration.z << ")"
                      << '\n';
        }
    }
    const RagdollState3D activeState =
        activeScene->ragdollState(activeRagdoll);
    const ActiveRagdollTelemetry3D& activeTelemetry =
        controller.telemetry();
    require(activeState.active && !activeState.links.empty(),
        "Modo ativo não foi preservado no snapshot");
    if (!(activeTelemetry.leftFootSupported
            && activeTelemetry.rightFootSupported
            && activeTelemetry.bodyUpDot > 0.92f
            && activeTelemetry.torsoUpDot > 0.92f
            && activeTelemetry.neckUpDot > 0.97f
            && activeTelemetry.headUpDot > 0.97f
            && activeTelemetry.centerOfMass.z > 0.80f
            && activeTelemetry.leftSoleUpDot > 0.985f
            && activeTelemetry.rightSoleUpDot > 0.985f
            && activeTelemetry.stanceWidthMeters > 0.225f
            && controller.output().rootForceNewtons.z
                < profile.totalMassKg * 9.81f * 0.28f)) {
        std::cerr << "[active-stance] bodyUp="
                  << activeTelemetry.bodyUpDot
                  << " torsoUp=" << activeTelemetry.torsoUpDot
                  << " neckUp=" << activeTelemetry.neckUpDot
                  << " headUp=" << activeTelemetry.headUpDot
                  << " comZ=" << activeTelemetry.centerOfMass.z
                  << " stance=" << activeTelemetry.stanceWidthMeters
                  << " soleUp=(" << activeTelemetry.leftSoleUpDot
                  << ", " << activeTelemetry.rightSoleUpDot << ")"
                  << " contacts=" << activeState.contacts.size()
                  << " supportFlags=("
                  << activeTelemetry.leftFootSupported << ", "
                  << activeTelemetry.rightFootSupported << ")"
                  << " recovery=" << activeTelemetry.recoveryStepActive
                  << " urgency=" << activeTelemetry.recoveryUrgency
                  << " phase=" << static_cast<int>(activeTelemetry.phase)
                  << " footZ=(" << activeState.links[14].position.z
                  << ", " << activeState.links[17].position.z << ")"
                  << " pelvisAssist="
                  << activeTelemetry.pelvisAssistForceNewtons
                  << " spineAssist="
                  << activeTelemetry.spineAssistForceNewtons
                  << " wbc=" << activeTelemetry.wholeBodySolved
                  << " residual="
                  << activeTelemetry.wholeBodyDynamicsResidual
                  << " maxTau="
                  << activeTelemetry.wholeBodyMaximumTorqueNewtonMeters
                  << " grf=("
                  << activeTelemetry.leftGroundReactionNewtons << ", "
                  << activeTelemetry.rightGroundReactionNewtons << ")\n";
    }
    require(activeTelemetry.leftFootSupported
            && activeTelemetry.rightFootSupported,
        "Ragdoll ativo não estabeleceu apoio bilateral");
    require(activeTelemetry.bodyUpDot > 0.92f
            && activeTelemetry.torsoUpDot > 0.92f
            && activeTelemetry.neckUpDot > 0.97f
            && activeTelemetry.headUpDot > 0.97f
            && activeTelemetry.centerOfMass.z > 0.80f,
        "Controlador ativo não sustentou tronco, pescoco e cabeca verticais");
    require(activeTelemetry.leftSoleUpDot > 0.985f
            && activeTelemetry.rightSoleUpDot > 0.985f,
        "Ragdoll ativo apoiou na ponta/quina em vez da planta dos pés");
    require(activeTelemetry.stanceWidthMeters > 0.225f,
        "Ragdoll ativo permaneceu com uma base de apoio estreita e frágil");
    require(controller.output().rootForceNewtons.z
            < profile.totalMassKg * 9.81f * 0.28f,
        "Auxílio vertical da bacia voltou a sustentar peso demais");
    if (std::getenv("MATTERENGINE_WBC_STANCE_ONLY") != nullptr) return;

    // Uma perturbação lateral no tronco deve acionar um passo real. O
    // controlador só pode responder por drives/torques articulares; o grab
    // abaixo representa a força externa que tirou o corpo da base de apoio.
    if (std::getenv("MATTERENGINE_WBC_WALK_ONLY") == nullptr) {
    PhysicsGrabTarget3D balanceTarget;
    balanceTarget.position = activeState.links[3].position;
    balanceTarget.orientation = activeState.links[3].orientation;
    PhysicsHandleSettings3D balanceGrab;
    balanceGrab.linearStiffness = 900.0f;
    balanceGrab.maximumForce = 2'500.0f;
    require(activeScene->beginRagdollGrab(activeRagdoll, 3, {},
            balanceTarget, balanceGrab),
        "Teste de equilíbrio não conseguiu perturbar o tronco");
    for (int step = 0; step < 12; ++step) {
        balanceTarget.position.y =
            activeState.links[3].position.y + 0.18f;
        activeScene->updateGrabTarget(balanceTarget, balanceGrab);
        updateActiveController(
            activeScene->ragdollState(activeRagdoll));
        const ActiveRagdollControlOutput3D& output =
            controller.output();
        if (output.applyRootForce) {
            activeScene->applyRagdollRootForce(activeRagdoll,
                output.rootForceNewtons,
                output.rootTorqueNewtonMeters);
        }
        if (output.applySpineForce) {
            activeScene->applyRagdollLinkForce(activeRagdoll,
                output.spineLinkIndex, output.spineForceNewtons,
                output.spineTorqueNewtonMeters);
        }
        activeScene->setRagdollActiveDriveTargets(activeRagdoll,
            output.driveTargets, output.gravityCompensationEnabled);
        activeScene->simulate(1.0f / 120.0f);
    }
    activeScene->endGrab();
    for (int step = 0; step < 360; ++step) {
        updateActiveController(
            activeScene->ragdollState(activeRagdoll));
        const ActiveRagdollControlOutput3D& output =
            controller.output();
        if (output.applyRootForce) {
            activeScene->applyRagdollRootForce(activeRagdoll,
                output.rootForceNewtons,
                output.rootTorqueNewtonMeters);
        }
        if (output.applySpineForce) {
            activeScene->applyRagdollLinkForce(activeRagdoll,
                output.spineLinkIndex, output.spineForceNewtons,
                output.spineTorqueNewtonMeters);
        }
        activeScene->setRagdollActiveDriveTargets(activeRagdoll,
            output.driveTargets, output.gravityCompensationEnabled);
        activeScene->simulate(1.0f / 120.0f);
    }
    const ActiveRagdollTelemetry3D recoveredTelemetry =
        controller.telemetry();
    require(std::isfinite(recoveredTelemetry.bodyUpDot)
            && recoveredTelemetry.bodyUpDot > 0.92f
            && recoveredTelemetry.torsoUpDot > 0.92f
            && recoveredTelemetry.centerOfMass.z > 0.80f,
        "Equilíbrio ativo não recuperou a vertical após empurrão lateral");

    // O teste lateral sozinho favorece um controlador que só domina o
    // tornozelo no plano frontal. Repetimos a perturbação no sentido da
    // marcha para validar quadris, joelhos e a estratégia de captura.
    const RagdollState3D beforeForwardPush =
        activeScene->ragdollState(activeRagdoll);
    balanceTarget.position = beforeForwardPush.links[3].position;
    balanceTarget.orientation = beforeForwardPush.links[3].orientation;
    require(activeScene->beginRagdollGrab(activeRagdoll, 3, {},
            balanceTarget, balanceGrab),
        "Teste de equilíbrio não conseguiu aplicar empurrão frontal");
    for (int step = 0; step < 12; ++step) {
        balanceTarget.position.x =
            beforeForwardPush.links[3].position.x + 0.18f;
        activeScene->updateGrabTarget(balanceTarget, balanceGrab);
        updateActiveController(
            activeScene->ragdollState(activeRagdoll));
        const ActiveRagdollControlOutput3D& output =
            controller.output();
        if (output.applyRootForce) {
            activeScene->applyRagdollRootForce(activeRagdoll,
                output.rootForceNewtons,
                output.rootTorqueNewtonMeters);
        }
        if (output.applySpineForce) {
            activeScene->applyRagdollLinkForce(activeRagdoll,
                output.spineLinkIndex, output.spineForceNewtons,
                output.spineTorqueNewtonMeters);
        }
        activeScene->setRagdollActiveDriveTargets(activeRagdoll,
            output.driveTargets, output.gravityCompensationEnabled);
        activeScene->simulate(1.0f / 120.0f);
    }
    activeScene->endGrab();
    for (int step = 0; step < 360; ++step) {
        updateActiveController(
            activeScene->ragdollState(activeRagdoll));
        const ActiveRagdollControlOutput3D& output =
            controller.output();
        if (output.applyRootForce) {
            activeScene->applyRagdollRootForce(activeRagdoll,
                output.rootForceNewtons,
                output.rootTorqueNewtonMeters);
        }
        if (output.applySpineForce) {
            activeScene->applyRagdollLinkForce(activeRagdoll,
                output.spineLinkIndex, output.spineForceNewtons,
                output.spineTorqueNewtonMeters);
        }
        activeScene->setRagdollActiveDriveTargets(activeRagdoll,
            output.driveTargets, output.gravityCompensationEnabled);
        activeScene->simulate(1.0f / 120.0f);
    }
    const ActiveRagdollTelemetry3D forwardRecoveredTelemetry =
        controller.telemetry();
    require(std::isfinite(forwardRecoveredTelemetry.bodyUpDot)
            && forwardRecoveredTelemetry.bodyUpDot > 0.92f
            && forwardRecoveredTelemetry.torsoUpDot > 0.92f
            && forwardRecoveredTelemetry.centerOfMass.z > 0.80f,
        "Equilíbrio ativo não recuperou a vertical após empurrão frontal");

    // Laboratorio automatizado de impactos. Diferente dos grabs acima, estas
    // perturbacoes sao forcas puras aplicadas aos mesmos tres links usados
    // pelo painel RAGDOLL/IMPULSOS. A assistencia externa permanece desligada:
    // a recuperacao precisa vir dos contatos, passos e torques articulares.
    const auto runImpactScenario = [&](const char* scenarioName,
            RagdollImpactMode3D mode, float forceNewtons,
            float directionDegrees, float durationSeconds,
            std::optional<FootSide3D> expectedFirstRecoveryFoot) {
        RagdollImpactTest3D impactLab;
        impactLab.config().mode = mode;
        impactLab.config().forceNewtons = forceNewtons;
        impactLab.config().directionDegrees = directionDegrees;
        impactLab.config().pulseDurationSeconds = durationSeconds;
        if (mode == RagdollImpactMode3D::Random) {
            impactLab.config().randomMinimumForceNewtons =
                forceNewtons * 0.50f;
            impactLab.config().randomMaximumForceNewtons = forceNewtons;
            impactLab.config().randomMinimumIntervalSeconds = 0.85f;
            impactLab.config().randomMaximumIntervalSeconds = 1.45f;
            impactLab.config().randomMinimumDurationSeconds = 0.05f;
            impactLab.config().randomMaximumDurationSeconds = 0.14f;
        }
        impactLab.triggerNow();
        if (mode == RagdollImpactMode3D::Continuous
                || mode == RagdollImpactMode3D::Random) {
            impactLab.setRunning(true);
        }

        bool sawRecoveryStep = false;
        bool firstRecoveryFootCorrect = false;
        bool sampledFirstRecoveryFoot = false;
        bool assistanceWasUsed = false;
        float maximumHorizontalSpeed = 0.0f;
        const int forceTicks = std::max(1,
            static_cast<int>(std::ceil(durationSeconds * 120.0f)));
        constexpr int RecoveryTicks = 600;
        for (int tick = 0; tick < forceTicks + RecoveryTicks; ++tick) {
            if ((mode == RagdollImpactMode3D::Continuous
                    || mode == RagdollImpactMode3D::Random)
                    && tick >= forceTicks) {
                impactLab.setRunning(false);
            }

            const RagdollState3D state =
                activeScene->ragdollState(activeRagdoll);
            updateActiveController(state);
            const ActiveRagdollControlOutput3D& output =
                controller.output();
            assistanceWasUsed = assistanceWasUsed
                || output.applyRootForce || output.applySpineForce;
            activeScene->setRagdollActiveDriveTargets(activeRagdoll,
                output.driveTargets, output.gravityCompensationEnabled);

            impactLab.update(1.0f / 120.0f);
            const RagdollImpactTestOutput3D& impact = impactLab.output();
            if (impact.applying) {
                Vec3 forward = state.links.front().orientation.rotate(
                    { 1.0f, 0.0f, 0.0f });
                forward.z = 0.0f;
                forward = forward.lengthSquared() > 0.000001f
                    ? forward.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
                const Vec3 left { -forward.y, forward.x, 0.0f };
                const float radians = impact.directionDegrees
                    * 3.14159265358979323846f / 180.0f;
                const Vec3 totalForce =
                    (forward * std::cos(radians)
                        + left * std::sin(radians))
                    * impact.forceNewtons;
                activeScene->applyRagdollLinkForce(activeRagdoll,
                    0, totalForce * 0.20f, {});
                activeScene->applyRagdollLinkForce(activeRagdoll,
                    2, totalForce * 0.32f, {});
                activeScene->applyRagdollLinkForce(activeRagdoll,
                    3, totalForce * 0.48f, {});
            }
            activeScene->simulate(1.0f / 120.0f);

            const ActiveRagdollTelemetry3D& telemetry =
                controller.telemetry();
            maximumHorizontalSpeed = std::max(maximumHorizontalSpeed,
                std::sqrt(telemetry.centerOfMassVelocity.x
                        * telemetry.centerOfMassVelocity.x
                    + telemetry.centerOfMassVelocity.y
                        * telemetry.centerOfMassVelocity.y));
            if (telemetry.recoveryStepActive) {
                sawRecoveryStep = true;
                if (!sampledFirstRecoveryFoot) {
                    sampledFirstRecoveryFoot = true;
                    firstRecoveryFootCorrect = !expectedFirstRecoveryFoot
                        || telemetry.recoverySwingSide
                            == *expectedFirstRecoveryFoot;
                }
            }
            if (std::getenv("MATTERENGINE_WBC_IMPACT_TRACE") != nullptr
                    && tick % 4 == 0) {
                std::cerr << "[impact-trace " << scenarioName << " "
                          << tick << "] force=" << impact.forceNewtons
                          << " up=(" << telemetry.bodyUpDot << ","
                          << telemetry.torsoUpDot << ") com=("
                          << telemetry.centerOfMass.x << ","
                          << telemetry.centerOfMass.y << ","
                          << telemetry.centerOfMass.z << ") v=("
                          << telemetry.centerOfMassVelocity.x << ","
                          << telemetry.centerOfMassVelocity.y << ") wv=("
                          << telemetry.dynamicsCenterOfMassVelocity.x << ","
                          << telemetry.dynamicsCenterOfMassVelocity.y
                          << ") margin="
                          << telemetry.captureMarginMeters << " support=("
                          << telemetry.leftFootSupported << ","
                          << telemetry.rightFootSupported << ") step="
                          << telemetry.recoveryStepActive << "/"
                          << static_cast<int>(telemetry.recoverySwingSide)
                          << " p=" << telemetry.recoveryStepProgress
                          << " stance=" << telemetry.stanceWidthMeters
                          << " feetY=("
                          << state.links[14].position.y << ","
                          << state.links[17].position.y << ") feetX=("
                          << state.links[14].position.x << ","
                          << state.links[17].position.x << ") footZ=("
                          << state.links[14].position.z << ","
                          << state.links[17].position.z << ") sole=("
                          << telemetry.leftSoleUpDot << ","
                          << telemetry.rightSoleUpDot << ") target=("
                          << telemetry.recoveryStepTarget.position.x << ","
                          << telemetry.recoveryStepTarget.position.y << ")"
                          << " pose=("
                          << telemetry.recoveryStepPose.position.x << ","
                          << telemetry.recoveryStepPose.position.y << ","
                          << telemetry.recoveryStepPose.position.z << ")"
                          << " footA=("
                          << telemetry.desiredSwingFootAcceleration.x << ","
                          << telemetry.desiredSwingFootAcceleration.y << ","
                          << telemetry.desiredSwingFootAcceleration.z << ")/("
                          << telemetry.solvedSwingFootAcceleration.x << ","
                          << telemetry.solvedSwingFootAcceleration.y << ","
                          << telemetry.solvedSwingFootAcceleration.z << ")"
                          << " urgency=" << telemetry.recoveryUrgency
                          << " axial=(" << telemetry.neckUpDot << ","
                          << telemetry.headUpDot << ") tau="
                          << telemetry.wholeBodyMaximumTorqueNewtonMeters
                          << " upperW=("
                          << state.links[3].angularVelocity.x << ","
                          << state.links[3].angularVelocity.y << ","
                          << state.links[3].angularVelocity.z << ")"
                          << " pelvisW=("
                          << state.links[0].angularVelocity.x << ","
                          << state.links[0].angularVelocity.y << ","
                          << state.links[0].angularVelocity.z << ")"
                          << " fall=" << telemetry.fallProtectionActive
                          << "/" << telemetry.fallen
                          << " wbc=" << telemetry.wholeBodySolved
                          << "/" << telemetry.wholeBodySolverOptimal
                          << " residual="
                          << telemetry.wholeBodyDynamicsResidual
                          << " qp=("
                          << telemetry.wholeBodySolverPrimalResidual << ","
                          << telemetry.wholeBodySolverDualResidual << ")"
                          << " rootA=("
                          << telemetry.desiredRootLinearAcceleration.x
                          << ","
                          << telemetry.desiredRootLinearAcceleration.y
                          << ") solved=("
                          << telemetry.solvedRootLinearAcceleration.x
                          << ","
                          << telemetry.solvedRootLinearAcceleration.y
                          << ") comA=("
                          << telemetry.solvedCenterOfMassAcceleration.x
                          << ","
                          << telemetry.solvedCenterOfMassAcceleration.y
                          << ") contactA=("
                          << telemetry.desiredContactAcceleration.x
                          << ","
                          << telemetry.desiredContactAcceleration.y
                          << ") grf=("
                          << telemetry.leftGroundReactionNewtons << ","
                          << telemetry.rightGroundReactionNewtons << ")"
                          << " fxy=("
                          << telemetry.leftContactForceNewtons.x
                              + telemetry.rightContactForceNewtons.x << ","
                          << telemetry.leftContactForceNewtons.y
                              + telemetry.rightContactForceNewtons.y << ")"
                          << '\n';
            }
        }

        const ActiveRagdollTelemetry3D& recovered =
            controller.telemetry();
        if (!(recovered.bodyUpDot > 0.92f
                && recovered.torsoUpDot > 0.92f
                && recovered.centerOfMass.z > 0.80f
                && recovered.leftFootSupported
                && recovered.rightFootSupported
                && recovered.leftSoleUpDot > 0.97f
                && recovered.rightSoleUpDot > 0.97f)) {
            std::cerr << "[impact " << scenarioName << "] up=("
                      << recovered.bodyUpDot << ","
                      << recovered.torsoUpDot << ") comZ="
                      << recovered.centerOfMass.z << " support=("
                      << recovered.leftFootSupported << ","
                      << recovered.rightFootSupported << ") sole=("
                      << recovered.leftSoleUpDot << ","
                      << recovered.rightSoleUpDot << ") speedMax="
                      << maximumHorizontalSpeed << " stance="
                      << recovered.stanceWidthMeters << " footZ=("
                      << activeScene->ragdollState(activeRagdoll)
                            .links[14].position.z << ","
                      << activeScene->ragdollState(activeRagdoll)
                            .links[17].position.z << ") step="
                      << sawRecoveryStep << "/"
                      << recovered.recoveryStepActive
                      << " firstCorrect="
                      << firstRecoveryFootCorrect << '\n';
        }
        require(!assistanceWasUsed,
            "Laboratorio de impacto acionou forca auxiliar de raiz/coluna");
        require(std::isfinite(maximumHorizontalSpeed)
                && maximumHorizontalSpeed < 8.0f,
            "Impacto provocou velocidade explosiva no ragdoll");
        if (mode == RagdollImpactMode3D::Random) {
            require(impactLab.output().eventCount >= 3,
                "Laboratorio aleatorio nao gerou impactos suficientes");
        }
        require(recovered.bodyUpDot > 0.92f
                && recovered.torsoUpDot > 0.92f
                && recovered.centerOfMass.z > 0.80f,
            "Ragdoll nao recuperou postura perfeita apos impacto padrao");
        require(recovered.leftFootSupported
                && recovered.rightFootSupported
                && recovered.leftSoleUpDot > 0.97f
                && recovered.rightSoleUpDot > 0.97f,
            "Ragdoll nao replantou as duas solas apos impacto padrao");
        if (expectedFirstRecoveryFoot) {
            require(sawRecoveryStep,
                "Impacto lateral nao acionou footwork de recuperacao");
            require(firstRecoveryFootCorrect,
                "Footwork lateral moveu primeiro o pe interno/cruzado");
        }
    };

    runImpactScenario("rajada esquerda",
        RagdollImpactMode3D::DirectPulse,
        1'000.0f, 90.0f, 0.12f, FootSide3D::Left);
    runImpactScenario("rajada direita",
        RagdollImpactMode3D::DirectPulse,
        1'000.0f, 270.0f, 0.12f, FootSide3D::Right);
    runImpactScenario("rajada frontal",
        RagdollImpactMode3D::DirectPulse,
        1'000.0f, 0.0f, 0.12f, std::nullopt);
    runImpactScenario("vento lateral continuo",
        RagdollImpactMode3D::Continuous,
        350.0f, 90.0f, 3.0f, FootSide3D::Left);
    runImpactScenario("sequencia aleatoria",
        RagdollImpactMode3D::Random,
        180.0f, 0.0f, 6.0f, std::nullopt);
    if (std::getenv("MATTERENGINE_WBC_BALANCE_ONLY") != nullptr) return;
    }

    controller.requestWalkDistance(5.0f);
    for (int step = 0; step < 42'000
            && (controller.telemetry().walking
                || controller.telemetry().walkPending
                || step == 0); ++step) {
        updateActiveController(
            activeScene->ragdollState(activeRagdoll));
        const ActiveRagdollControlOutput3D& output =
            controller.output();
        if (output.applyRootForce) {
            activeScene->applyRagdollRootForce(activeRagdoll,
                output.rootForceNewtons,
                output.rootTorqueNewtonMeters);
        }
        if (output.applySpineForce) {
            activeScene->applyRagdollLinkForce(activeRagdoll,
                output.spineLinkIndex, output.spineForceNewtons,
                output.spineTorqueNewtonMeters);
        }
        activeScene->setRagdollActiveDriveTargets(activeRagdoll,
            output.driveTargets, output.gravityCompensationEnabled);
        activeScene->simulate(1.0f / 120.0f);
        if (std::getenv("MATTERENGINE_WBC_WALK_TRACE") != nullptr
            && step % 30 == 0) {
            const ActiveRagdollTelemetry3D& trace =
                controller.telemetry();
            std::cerr << "[walk " << step << "] d="
                      << trace.walkDistanceMeters
                      << " steps=" << trace.completedWalkSteps
                      << " up=(" << trace.bodyUpDot << ","
                      << trace.torsoUpDot << ") support=("
                      << trace.leftFootSupported << ","
                      << trace.rightFootSupported << ") swing="
                      << trace.recoveryStepActive << "/"
                      << static_cast<int>(trace.recoverySwingSide)
                      << " p=" << trace.recoveryStepProgress
                      << " target=("
                      << trace.recoveryStepTarget.position.x << ","
                      << trace.recoveryStepTarget.position.y << ","
                      << trace.recoveryStepTarget.position.z << ")"
                      << " margin=" << trace.captureMarginMeters
                      << " com=(" << trace.centerOfMass.x << ","
                      << trace.centerOfMass.y << ")"
                      << " com*=(" << trace.desiredCenterOfMass.x
                      << "," << trace.desiredCenterOfMass.y << ")"
                      << " dynCom=(" << trace.dynamicsCenterOfMass.x
                      << "," << trace.dynamicsCenterOfMass.y << ")"
                      << " dynV=("
                      << trace.dynamicsCenterOfMassVelocity.x << ","
                      << trace.dynamicsCenterOfMassVelocity.y << ")"
                      << " v*=("
                      << trace.wholeBodyDesiredHorizontalVelocity.x << ","
                      << trace.wholeBodyDesiredHorizontalVelocity.y << ")"
                      << " transfer="
                      << trace.footworkWeightTransferProgress
                      << " released=" << trace.wholeBodySwingReleased
                      << " toe=("
                      << trace.toeOffSagittalErrorMeters << ","
                      << trace.toeOffLateralErrorMeters << ")"
                      << " base=("
                      << (trace.supportMinimum.x
                            + trace.supportMaximum.x) * 0.5f << ","
                      << (trace.supportMinimum.y
                            + trace.supportMaximum.y) * 0.5f << ")"
                      << " comV=(" << trace.centerOfMassVelocity.x
                      << "," << trace.centerOfMassVelocity.y << ")"
                      << " comA*=("
                      << trace.desiredRootLinearAcceleration.x << ","
                      << trace.desiredRootLinearAcceleration.y << ")"
                      << " comA=("
                      << trace.solvedCenterOfMassAcceleration.x << ","
                      << trace.solvedCenterOfMassAcceleration.y << ")"
                      << " sole=(" << trace.leftSoleUpDot << ","
                      << trace.rightSoleUpDot << ") grf=("
                      << trace.leftGroundReactionNewtons << ","
                      << trace.rightGroundReactionNewtons << ")"
                      << " fxyL=("
                      << trace.leftContactForceNewtons.x << ","
                      << trace.leftContactForceNewtons.y << ") fxyR=("
                      << trace.rightContactForceNewtons.x << ","
                      << trace.rightContactForceNewtons.y << ")"
                      << " maxTau="
                      << trace.wholeBodyMaximumTorqueNewtonMeters
                      << " wbc="
                      << trace.wholeBodySolved
                      << " optimal=" << trace.wholeBodySolverOptimal
                      << " iter=" << trace.wholeBodySolverIterations
                      << " qpRes=("
                      << trace.wholeBodySolverPrimalResidual << ","
                      << trace.wholeBodySolverDualResidual << ")\n";
        }
    }
    const ActiveRagdollTelemetry3D walkTelemetry =
        controller.telemetry();
    if (walkTelemetry.walking
            || walkTelemetry.walkDistanceMeters < 4.90f) {
        std::cerr << "[active-walk] walking=" << walkTelemetry.walking
                  << " distance=" << walkTelemetry.walkDistanceMeters
                  << " steps=" << walkTelemetry.completedWalkSteps
                  << " bodyUp=" << walkTelemetry.bodyUpDot
                  << " torsoUp=" << walkTelemetry.torsoUpDot
                  << " fallen=" << walkTelemetry.fallen
                  << " phase=" << static_cast<int>(walkTelemetry.phase)
                  << " stance=" << walkTelemetry.stanceWidthMeters
                  << " soleUp=(" << walkTelemetry.leftSoleUpDot
                  << ", " << walkTelemetry.rightSoleUpDot << ")\n";
    }
    require(!walkTelemetry.walking
            && walkTelemetry.walkDistanceMeters >= 4.90f,
        "Ragdoll ativo não concluiu a caminhada física de 5 metros");
    require(walkTelemetry.bodyUpDot > 0.90f
            && walkTelemetry.torsoUpDot > 0.90f,
        "Ragdoll perdeu a postura durante a caminhada de 5 metros");
    // Trava de regressão para "anda, mas se arrasta": nada aqui garantia
    // que os 5 m fossem cobertos com passadas plausíveis em vez de um
    // arrastar de pés picotado. Medido nesta configuração: ~97 passos,
    // ~5,1 cm de avanço médio por passo. O piso de 4 cm fica bem acima do
    // comportamento anterior (~3,2 cm/passo) — se isso regredir para lá,
    // o teste deve falhar mesmo que a distância total ainda feche em 5 m.
    require(walkTelemetry.completedWalkSteps > 0,
        "Telemetria de passos da caminhada ausente");
    const float averageStepAdvanceMeters = walkTelemetry.walkDistanceMeters
        / static_cast<float>(walkTelemetry.completedWalkSteps);
    require(averageStepAdvanceMeters > 0.040f,
        "Caminhada de 5 m regrediu para passos arrastados/picotados");
    require(walkTelemetry.completedWalkSteps < 130,
        "Caminhada de 5 m precisou de passos demais (marcha não fluida)");

    // Contrato independente do planejador: uma velocidade que projeta o
    // ponto de captura para fora da base precisa produzir imediatamente uma
    // trajetória de recuperação e comandos articulares limitados.
    RagdollState3D projectedFall =
        activeScene->ragdollState(activeRagdoll);
    for (PhysicsBodyState3D& link : projectedFall.links) {
        link.linearVelocity.y += 2.0f;
    }
    updateActiveController(projectedFall);
    require(controller.telemetry().recoveryStepActive,
        "Ponto de captura externo não iniciou passada de recuperação");
    const bool hasSwingCommand = std::any_of(
        controller.output().driveTargets.begin(),
        controller.output().driveTargets.end(),
        [](const RagdollDriveTarget3D& target) {
            return std::abs(target.velocityRadiansPerSecond) > 0.01f
                || std::abs(target.feedforwardTorqueNewtonMeters) > 0.01f;
        });
    require(hasSwingCommand,
        "Passada de recuperação não gerou torque/velocidade articular");
}

void testPhysicalFootworkController(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    const std::string path = std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/physics/ragdolls/FootworkLowerBodyV1.ragdoll.json";
    const RagdollProfile3D profile = loadRagdollProfile3D(path);
    require(profile.links.size() == 7,
        "FootworkLowerBodyV1 deveria conter sete corpos");
    require(ragdollDegreesOfFreedom3D(profile) == 12,
        "FootworkLowerBodyV1 deveria conter doze DOFs");
    require(validateRagdollProfile3D(profile).empty(),
        "FootworkLowerBodyV1 não passou na validação");

    PhysicsSceneSettings3D settings;
    auto scene = engine.createScene(settings, materials);
    createStaticGround(engine, *scene);
    RealisticFootworkTerrain terrain(*scene);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 90'701;
    spawn.pelvisPosition = {
        0.0f, 0.0f, profile.standingRootHeightMeters + 0.025f
    };
    spawn.active = true;
    const RagdollHandle3D handle =
        scene->createRagdoll(profile, spawn);
    PhysicalFootworkController3D controller;
    require(controller.reset(profile, scene->ragdollState(handle),
            0.0f, terrain, PhysicalFootworkMode3D::Manual),
        "Controlador físico de Footwork não inicializou");

    const auto simulate = [&](const FootworkInput3D& input) {
        controller.setInput(input);
        controller.update(profile, scene->ragdollState(handle),
            1.0f / 120.0f, terrain);
        const PhysicalFootworkOutput3D& output = controller.output();
        if (output.applyRootForce) {
            scene->applyRagdollRootForce(handle,
                output.rootForceNewtons, output.rootTorqueNewtonMeters);
        }
        scene->setRagdollActiveDriveTargets(handle,
            output.driveTargets, output.gravityCompensationEnabled);
        scene->simulate(1.0f / 120.0f);
    };
    FootworkInput3D idle;
    for (int tick = 0; tick < 180; ++tick) simulate(idle);
    const RagdollState3D standing = scene->ragdollState(handle);
    require(standing.links.front().position.z > 0.45f
            && standing.links.front().position.z < 1.45f,
        "Assistência da bacia não sustentou o meio-esqueleto");
    require(controller.telemetry().uprightDot > 0.45f,
        "Bacia física perdeu verticalidade em repouso");

    FootworkInput3D forward;
    forward.movementLocal.y = 1.0f;
    for (int tick = 0; tick < 540; ++tick) simulate(forward);
    const RagdollState3D moved = scene->ragdollState(handle);
    require(controller.planner().state().completedStepCount > 0,
        "Footwork físico não concluiu nenhuma passada");
    require(moved.links.front().position.x > standing.links.front().position.x
            + 0.15f,
        "Meio-esqueleto não respondeu ao comando de avanço");
    for (const PhysicsBodyState3D& link : moved.links) {
        require(std::isfinite(link.position.x)
                && std::isfinite(link.position.z)
                && link.linearVelocity.length() < 35.0f,
            "Footwork físico produziu estado explosivo/não finito");
    }

    // Um corpo deitado e sem contato não pode ser sustentado no ar pela
    // assistência de laboratório.
    RagdollState3D fallen = moved;
    fallen.contacts.clear();
    fallen.links.front().position.z = 0.18f;
    fallen.links.front().orientation = Quaternion::fromAxisAngle(
        { 1.0f, 0.0f, 0.0f }, 1.57079632679f);
    controller.update(profile, fallen, 1.0f / 120.0f, terrain);
    require(!controller.output().applyRootForce
            || controller.output().rootForceNewtons.length() < 1.0f,
        "Assistência da bacia tentou levitar o corpo caído");
}

// Modo de teste: estabilização mágica da pelve (ActiveRagdollConfig3D::
// useMagicPelvisStabilization). Isola "os passos parecem uma marcha
// agressiva" de "o corpo fica de pé sozinho" — a pelve é sustentada por
// força/torque externos direto na raiz (PhysicsScene3D::
// applyRagdollRootForce), então este teste é o único do arquivo que chama
// esse método. Critério de aceitação: cobre os 5 m com poucos passos
// grandes (não uma centena de passos arrastados) e nunca produz estado não
// finito, mesmo com o passo fixo e agressivo e nenhuma checagem de
// equilíbrio no gatilho.
void testActiveRagdollMagicPelvisStabilization(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    const std::string path = std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/physics/ragdolls/HumanAdultV1.ragdoll.json";
    const RagdollProfile3D profile = loadRagdollProfile3D(path);
    PhysicsSceneSettings3D settings;
    auto scene = engine.createScene(settings, materials);
    createStaticGround(engine, *scene);
    RealisticFootworkTerrain terrain(*scene);

    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 44'001;
    spawn.pelvisPosition = {
        0.0f, 0.0f, profile.standingRootHeightMeters + 0.055f
    };
    spawn.active = true;
    const RagdollHandle3D handle = scene->createRagdoll(profile, spawn);
    ActiveRagdollController3D controller;
    controller.reset(profile, spawn.orientation);
    controller.config().useMagicPelvisStabilization = true;

    const auto stepOnce = [&]() {
        controller.update(profile, scene->ragdollState(handle),
            1.0f / 120.0f, &terrain);
        const ActiveRagdollControlOutput3D& output = controller.output();
        if (output.applyRootForce
                && std::getenv("MATTER_DEBUG_MAGIC_NOFORCE") == nullptr) {
            scene->applyRagdollRootForce(handle, output.rootForceNewtons,
                output.rootTorqueNewtonMeters);
        }
        if (output.applySpineForce
                && std::getenv("MATTER_DEBUG_MAGIC_NOFORCE") == nullptr) {
            scene->applyRagdollLinkForce(handle, output.spineLinkIndex,
                output.spineForceNewtons,
                output.spineTorqueNewtonMeters);
        }
        scene->setRagdollActiveDriveTargets(handle, output.driveTargets,
            output.gravityCompensationEnabled);
        scene->simulate(1.0f / 120.0f);
    };
    std::size_t leftThighLink = 12;
    std::size_t leftShinLink = 13;
    std::size_t leftFootLink = 14;
    std::size_t rightFootLink = 17;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftThigh") leftThighLink = i;
        if (profile.links[i].id == "LeftShin") leftShinLink = i;
        if (profile.links[i].id == "LeftFoot") leftFootLink = i;
        if (profile.links[i].id == "RightFoot") rightFootLink = i;
    }
    for (int step = 0; step < 120; ++step) {
        stepOnce();
        if (std::getenv("MATTER_DEBUG_MAGIC") != nullptr
                && step % 5 == 0) {
            const auto& t = controller.telemetry();
            const auto& output = controller.output();
            const RagdollState3D debugState = scene->ragdollState(handle);
            const auto& pelvis = debugState.links.front();
            float hipSwing1Target = -999.0f;
            for (const RagdollDriveTarget3D& drive : output.driveTargets) {
                if (drive.linkIndex == leftThighLink
                        && drive.axis == RagdollAxis3D::Swing1) {
                    hipSwing1Target = drive.positionRadians;
                }
            }
            std::cerr << "settle step=" << step
                << " phase=" << static_cast<int>(t.phase)
                << " up=" << t.bodyUpDot
                << " hipSwing1=" << debugState.joints[leftThighLink].positionRadians[1]
                << " hipSwing1Target=" << hipSwing1Target
                << " kneeTwist=" << debugState.joints[leftShinLink].positionRadians[0]
                << " pelvisZ=" << pelvis.position.z
                << " leftFootZ=" << debugState.links[leftFootLink].position.z
                << " rightFootZ=" << debugState.links[rightFootLink].position.z
                << " contacts=" << t.supportContactCount
                << " totalContacts=" << debugState.contacts.size()
                << " lSup=" << t.leftFootSupported
                << " rSup=" << t.rightFootSupported
                << " applyRootForce=" << output.applyRootForce
                << " forceZ=" << output.rootForceNewtons.z
                << '\n';
        }
    }

    const RagdollState3D settled = scene->ragdollState(handle);
    require(std::isfinite(settled.links.front().position.z)
            && settled.links.front().position.z > 0.5f
            && settled.links.front().position.z < 1.5f,
        "Estabilização mágica não sustentou a pelve numa altura plausível");

    // Nesta etapa a assistência é validada apenas como reforço estacionário;
    // caminhada volta a este perfil quando o ciclo de locomoção for retomado.
    if (std::getenv("MATTERENGINE_WBC_BALANCE_ONLY") != nullptr) return;

    controller.requestWalkDistance(5.0f);
    int step = 0;
    for (; step < 6'000
            && (controller.telemetry().walking
                || controller.telemetry().walkPending
                || step == 0); ++step) {
        stepOnce();
        if (std::getenv("MATTER_DEBUG_MAGIC") != nullptr
                && step % 30 == 0) {
            const auto& t = controller.telemetry();
            const RagdollState3D debugState = scene->ragdollState(handle);
            const auto& pelvis = debugState.links.front();
            const std::size_t swingLink = t.recoverySwingSide == FootSide3D::Left
                ? leftFootLink : rightFootLink;
            std::cerr << "step=" << step
                << " phase=" << static_cast<int>(t.phase)
                << " dist=" << t.walkDistanceMeters
                << " steps=" << t.completedWalkSteps
                << " up=" << t.bodyUpDot
                << " pelvisZ=" << pelvis.position.z
                << " recov=" << t.recoveryStepActive
                << " prog=" << t.recoveryStepProgress
                << " swing=" << static_cast<int>(t.recoverySwingSide)
                << " swingFootZ=" << debugState.links[swingLink].position.z
                << " targetZ=" << t.recoveryStepTarget.position.z
                << " targetX=" << t.recoveryStepTarget.position.x
                << " swingFootX=" << debugState.links[swingLink].position.x
                << " lSup=" << t.leftFootSupported
                << " rSup=" << t.rightFootSupported
                << '\n';
        }
    }
    const ActiveRagdollTelemetry3D& walkTelemetry = controller.telemetry();
    require(!walkTelemetry.walking
            && walkTelemetry.walkDistanceMeters >= 4.90f,
        "Modo mágico não concluiu a caminhada de 5 metros");
    // O objetivo do modo é justamente NÃO arrastar os pés: com passo fixo
    // de 0,55 m, 5 m deveriam caber em bem menos de 20 passos.
    require(walkTelemetry.completedWalkSteps > 0
            && walkTelemetry.completedWalkSteps < 20,
        "Modo mágico regrediu para passos pequenos/numerosos — perdeu o "
        "propósito do teste");
    const RagdollState3D finalState = scene->ragdollState(handle);
    for (const PhysicsBodyState3D& link : finalState.links) {
        require(std::isfinite(link.position.x)
                && std::isfinite(link.orientation.x)
                && link.linearVelocity.length() < 30.0f,
            "Modo mágico produziu estado não finito/explosivo");
    }
}

// Reproduz o mecanismo de empurrão de testActiveRagdollBalanceRobustness
// (grab no tronco, arrastado e solto) para o modo mágico especificamente —
// nenhum outro teste deste arquivo verifica se um empurrão dispara um passo
// de recuperação com useMagicPelvisStabilization ligado. Escrito em resposta
// a relato direto: "nem quando eu empurro ele, ele não tenta ter um jogo de
// pé para tentar manter a pose".
void testActiveRagdollMagicPushRecovery(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    const std::string path = std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/physics/ragdolls/HumanAdultV1.ragdoll.json";
    const RagdollProfile3D profile = loadRagdollProfile3D(path);
    PhysicsSceneSettings3D settings;
    auto scene = engine.createScene(settings, materials);
    createStaticGround(engine, *scene);
    RealisticFootworkTerrain terrain(*scene);

    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 45'001;
    spawn.pelvisPosition = {
        0.0f, 0.0f, profile.standingRootHeightMeters + 0.055f
    };
    spawn.active = true;
    const RagdollHandle3D handle = scene->createRagdoll(profile, spawn);
    ActiveRagdollController3D controller;
    controller.reset(profile, spawn.orientation);
    controller.config().useMagicPelvisStabilization = true;

    const auto stepOnce = [&]() {
        controller.update(profile, scene->ragdollState(handle),
            1.0f / 120.0f, &terrain);
        const ActiveRagdollControlOutput3D& output = controller.output();
        if (output.applyRootForce) {
            scene->applyRagdollRootForce(handle, output.rootForceNewtons,
                output.rootTorqueNewtonMeters);
        }
        if (output.applySpineForce) {
            scene->applyRagdollLinkForce(handle, output.spineLinkIndex,
                output.spineForceNewtons,
                output.spineTorqueNewtonMeters);
        }
        scene->setRagdollActiveDriveTargets(handle, output.driveTargets,
            output.gravityCompensationEnabled);
        scene->simulate(1.0f / 120.0f);
    };
    for (int step = 0; step < 240; ++step) stepOnce();

    const RagdollState3D settled = scene->ragdollState(handle);
    require(controller.telemetry().phase != ActiveRagdollPhase3D::Falling
            && controller.telemetry().bodyUpDot > 0.90f
            && controller.telemetry().torsoUpDot > 0.90f
            && controller.telemetry().supportContactCount > 0,
        "Equilíbrio natural não assentou ereto e apoiado antes do empurrão");
    if (std::getenv("MATTER_DEBUG_MAGIC") != nullptr) {
        std::cerr << "settled.sleeping=" << settled.sleeping << '\n';
    }

    // Medido: 900/2500 (mesmos valores de testActiveRagdollBalanceRobustness)
    // mal desloca o tronco (~9 mm em 12 ticks contra um alvo que se afasta
    // 220 mm) — a rigidez articular real já resiste tão bem que o "empurrão"
    // nem chega a ser sentido, então nenhuma margem de captura é abalada e
    // nenhum passo de recuperação é necessário (bodyUpDot nunca sai de
    // ~0,999). Ganhos bem mais altos aqui garantem uma perturbação real.
    PhysicsGrabTarget3D pushTarget;
    pushTarget.position = settled.links[3].position;
    pushTarget.orientation = settled.links[3].orientation;
    PhysicsHandleSettings3D pushSettings;
    pushSettings.linearStiffness = 4'000.0f;
    pushSettings.maximumForce = 9'000.0f;
    require(scene->beginRagdollGrab(handle, 3, {}, pushTarget, pushSettings),
        "Não foi possível empurrar o tronco (modo mágico)");
    constexpr float dragMeters = 0.45f;
    for (int step = 0; step < 12; ++step) {
        pushTarget.position.y = settled.links[3].position.y
            + dragMeters * (static_cast<float>(step + 1) / 12.0f);
        scene->updateGrabTarget(pushTarget, pushSettings);
        stepOnce();
        if (std::getenv("MATTER_DEBUG_MAGIC") != nullptr) {
            const auto& t = controller.telemetry();
            const RagdollState3D debugState = scene->ragdollState(handle);
            std::cerr << "push drag step=" << step
                << " phase=" << static_cast<int>(t.phase)
                << " up=" << t.bodyUpDot
                << " captureMargin=" << t.captureMarginMeters
                << " admissibility=" << t.admissibilityProjectionScale
                << " recov=" << t.recoveryStepActive
                << " torsoY=" << debugState.links[3].position.y
                << " targetY=" << pushTarget.position.y
                << '\n';
        }
    }
    scene->endGrab();

    bool sawRecoveryStep = false;
    bool recovered = false;
    for (int step = 0; step < 600 && !recovered; ++step) {
        stepOnce();
        const ActiveRagdollTelemetry3D& telemetry = controller.telemetry();
        sawRecoveryStep = sawRecoveryStep || telemetry.recoveryStepActive;
        recovered = telemetry.bodyUpDot > 0.90f
            && telemetry.torsoUpDot > 0.90f
            && !telemetry.recoveryStepActive
            && (telemetry.phase == ActiveRagdollPhase3D::Standing
                || telemetry.phase == ActiveRagdollPhase3D::Balancing);
        if (std::getenv("MATTER_DEBUG_MAGIC") != nullptr && step % 15 == 0) {
            const auto& t = controller.telemetry();
            const RagdollState3D debugState = scene->ragdollState(handle);
            std::cerr << "push recover step=" << step
                << " phase=" << static_cast<int>(t.phase)
                << " up=" << t.bodyUpDot
                << " pelvisZ=" << debugState.links.front().position.z
                << " captureMargin=" << t.captureMarginMeters
                << " admissibility=" << t.admissibilityProjectionScale
                << " recov=" << t.recoveryStepActive << '\n';
        }
    }
    require(sawRecoveryStep,
        "Modo mágico nunca disparou um passo de recuperação após o "
        "empurrão — personagem não tenta se reposicionar");
    require(recovered,
        "Modo mágico não recuperou o equilíbrio após o empurrão");
    const RagdollState3D afterRecovery = scene->ragdollState(handle);
    for (const PhysicsBodyState3D& link : afterRecovery.links) {
        require(std::isfinite(link.position.x)
                && link.linearVelocity.length() < 15.0f,
            "Estado não finito/explosivo após empurrão (modo mágico)");
    }
}

// Cobertura ampliada de robustez do controlador ativo. Os testes de
// testRagdollProfileAndRuntime provam que o sistema funciona; estes provam
// que ele não é frágil: compensação de gravidade isolada (o teste mais
// valioso da especificação — Etapa 2), empurrões nas oito direções em duas
// magnitudes sobre o MESMO ragdoll (perturbação repetida, como acontece de
// fato em jogo) e uma perturbação no meio de uma passada em andamento.
void testActiveRagdollBalanceRobustness(PhysicsEngine3D& engine,
    const MaterialLibrary& materials) {
    const std::string path = std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/physics/ragdolls/HumanAdultV1.ragdoll.json";
    const RagdollProfile3D profile = loadRagdollProfile3D(path);
    PhysicsSceneSettings3D settings;
    TestFootworkTerrain terrain;

    // --- Etapa 2 isolada: compensação de gravidade sozinha sustenta a pose.
    // Pelve presa por um grab rígido (equivalente a "base travada" da
    // especificação), todos os ganhos de postura e as estratégias de
    // tornozelo/quadril zeradas. Só o feedforward de gravidade resta.
    // Controle negativo no mesmo laço: com a compensação desligada, os
    // mesmos ganhos zerados devem deixar o tronco cair sob o próprio peso —
    // provando que o teste realmente mede alguma coisa.
    {
        auto scene = engine.createScene(settings, materials);
        createStaticBox(*scene, { 0.0f, 0.0f, -0.05f },
            { 20.0f, 20.0f, 0.05f });
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 41'001;
        spawn.pelvisPosition = {
            0.0f, 0.0f, profile.standingRootHeightMeters + 0.055f
        };
        spawn.active = true;
        const RagdollHandle3D handle = scene->createRagdoll(profile, spawn);
        ActiveRagdollController3D controller;
        controller.reset(profile, spawn.orientation);
        for (int step = 0; step < 60; ++step) {
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain);
            scene->setRagdollActiveDriveTargets(handle,
                controller.output().driveTargets,
                controller.output().gravityCompensationEnabled);
            scene->simulate(1.0f / 120.0f);
        }
        const RagdollState3D settled = scene->ragdollState(handle);
        PhysicsGrabTarget3D pin;
        pin.position = settled.links.front().position;
        pin.orientation = settled.links.front().orientation;
        pin.lockOrientation = true;
        PhysicsHandleSettings3D pinSettings;
        pinSettings.linearStiffness = 8'000.0f;
        pinSettings.maximumForce = 40'000.0f;
        pinSettings.angularStiffness = 4'000.0f;
        pinSettings.maximumTorque = 20'000.0f;
        require(scene->beginRagdollGrab(handle, 0, {}, pin, pinSettings),
            "Não foi possível prender a pelve para o teste isolado");
        const std::size_t abdomenIndex = 1;
        float startAbdomenPitch =
            settled.joints[abdomenIndex].positionRadians[1];

        auto& config = controller.config();
        config.standingStiffnessScale = 0.0f;
        config.standingDampingScale = 0.0f;
        config.ankleTorqueResponseNewtonMetersPerMeter = 0.0f;
        config.maximumAnkleBalanceTorqueNewtonMeters = 0.0f;
        config.torsoUprightTorqueNewtonMetersPerRadian = 0.0f;
        config.pelvisUprightTorqueNewtonMetersPerRadian = 0.0f;
        config.angularMomentumDampingGainPerSecond = 0.0f;
        config.armCounterRotationResponse = 0.0f;
        config.minimumStableStanceWidthMeters = 0.0f;
        config.unsupportedFootReplantDelaySeconds = 1'000.0f;
        config.recoveryStepTriggerUrgency = 2.0f;
        config.enableFreeBaseGravityCompensation = true;
        // Ao desligar todos os ganhos, deixamos morrer o momento residual da
        // pose que acabou de ser presa. A medição começa no equilíbrio livre
        // da compensação, em vez de atribuir esse transiente ao feedforward.
        for (int step = 0; step < 180; ++step) {
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain);
            scene->setRagdollActiveDriveTargets(handle,
                controller.output().driveTargets,
                controller.output().gravityCompensationEnabled);
            scene->simulate(1.0f / 120.0f);
        }
        startAbdomenPitch = scene->ragdollState(handle)
            .joints[abdomenIndex].positionRadians[1];
        for (int step = 0; step < 240; ++step) {
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain);
            scene->setRagdollActiveDriveTargets(handle,
                controller.output().driveTargets,
                controller.output().gravityCompensationEnabled);
            scene->simulate(1.0f / 120.0f);
        }
        const RagdollState3D withGravityComp = scene->ragdollState(handle);
        for (const PhysicsBodyState3D& link : withGravityComp.links) {
            require(std::isfinite(link.position.x)
                    && std::isfinite(link.angularVelocity.x)
                    && link.angularVelocity.length() < 20.0f,
                "Compensação de gravidade isolada produziu estado não "
                "finito ou giro explosivo");
        }
        const float driftWithCompensation = std::abs(
            withGravityComp.joints[abdomenIndex].positionRadians[1]
                - startAbdomenPitch);
        if (driftWithCompensation >= 0.12f) {
            std::cerr << "[gravity-isolation] drift="
                      << driftWithCompensation
                      << " start=" << startAbdomenPitch
                      << " end="
                      << withGravityComp.joints[abdomenIndex]
                            .positionRadians[1]
                      << '\n';
        }
        require(driftWithCompensation < 0.12f,
            "Compensação de gravidade sozinha não sustentou o tronco "
            "(ganhos de postura zerados, só feedforward)");

        config.enableFreeBaseGravityCompensation = false;
        for (int step = 0; step < 240; ++step) {
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain);
            scene->setRagdollActiveDriveTargets(handle,
                controller.output().driveTargets,
                controller.output().gravityCompensationEnabled);
            scene->simulate(1.0f / 120.0f);
        }
        const RagdollState3D withoutGravityComp =
            scene->ragdollState(handle);
        const float driftWithoutCompensation = std::abs(
            withoutGravityComp.joints[abdomenIndex].positionRadians[1]
                - startAbdomenPitch);
        require(driftWithoutCompensation > driftWithCompensation + 0.05f,
            "Controle negativo falhou: tronco não caiu sem compensação de "
            "gravidade, então o teste positivo acima não prova nada");
        scene->endGrab();
    }

    // --- Base livre sem assistência externa.
    // A compensação de gravidade e os motores articulares continuam sendo
    // biomecânicos, mas nenhuma força/torque pode ser aplicada diretamente
    // à pelve ou à coluna. Parado, o ragdoll deve sustentar a própria postura
    // por tornozelos, quadris e cadeia da coluna durante vários segundos.
    {
        auto scene = engine.createScene(settings, materials);
        createStaticBox(*scene, { 0.0f, 0.0f, -0.05f },
            { 20.0f, 20.0f, 0.05f });
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 41'101;
        spawn.pelvisPosition = {
            0.0f, 0.0f, profile.standingRootHeightMeters + 0.055f
        };
        spawn.orientation = Quaternion::fromAxisAngle(
            { 0.0f, 0.0f, 1.0f }, 0.41f);
        spawn.active = true;
        const RagdollHandle3D handle = scene->createRagdoll(profile, spawn);
        ActiveRagdollController3D controller;
        controller.reset(profile, spawn.orientation);
        controller.config().useMagicPelvisStabilization = false;
        float minimumSettledBodyUp = 1.0f;
        for (int step = 0; step < 1'200; ++step) {
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain);
            const ActiveRagdollControlOutput3D& output =
                controller.output();
            require(!output.applyRootForce && !output.applySpineForce,
                "Assistencia externa foi aplicada mesmo desativada");
            scene->setRagdollActiveDriveTargets(handle,
                output.driveTargets,
                output.gravityCompensationEnabled);
            scene->simulate(1.0f / 120.0f);
            if (step >= 240) {
                minimumSettledBodyUp = std::min(
                    minimumSettledBodyUp,
                    controller.telemetry().bodyUpDot);
            }
            if (std::getenv("MATTER_DEBUG_NO_ASSIST") != nullptr
                && step % 60 == 0) {
                const ActiveRagdollTelemetry3D& debug =
                    controller.telemetry();
                std::cerr << "[no-assist:" << step << "] phase="
                          << static_cast<int>(debug.phase)
                          << " up=" << debug.bodyUpDot
                          << " torso=" << debug.torsoUpDot
                          << " capture=" << debug.captureMarginMeters
                          << " urgency=" << debug.recoveryUrgency
                          << " support=(" << debug.leftFootSupported
                          << ", " << debug.rightFootSupported << ")"
                          << " recovery=" << debug.recoveryStepActive
                          << " stance=" << debug.stanceWidthMeters
                          << " sole=(" << debug.leftSoleUpDot
                          << ", " << debug.rightSoleUpDot << ")\n";
            }
        }
        const ActiveRagdollTelemetry3D& telemetry =
            controller.telemetry();
        if (!(telemetry.phase != ActiveRagdollPhase3D::Falling
                && telemetry.bodyUpDot > 0.90f
                && telemetry.torsoUpDot > 0.90f
                && telemetry.centerOfMass.z > 0.78f
                && minimumSettledBodyUp > 0.78f)) {
            std::cerr << "[active-no-assist] phase="
                      << static_cast<int>(telemetry.phase)
                      << " bodyUp=" << telemetry.bodyUpDot
                      << " torsoUp=" << telemetry.torsoUpDot
                      << " minBodyUp=" << minimumSettledBodyUp
                      << " comZ=" << telemetry.centerOfMass.z
                      << " support=(" << telemetry.leftFootSupported
                      << ", " << telemetry.rightFootSupported << ")"
                      << " stance=" << telemetry.stanceWidthMeters
                      << " sole=(" << telemetry.leftSoleUpDot
                      << ", " << telemetry.rightSoleUpDot << ")\n";
        }
        require(telemetry.phase != ActiveRagdollPhase3D::Falling
                && telemetry.bodyUpDot > 0.90f
                && telemetry.torsoUpDot > 0.90f
                && telemetry.centerOfMass.z > 0.78f
                && minimumSettledBodyUp > 0.78f,
            "Ragdoll dependeu da assistencia externa para ficar em pe "
            "parado");
    }

    // --- Reparo persistente da postura passiva.
    // Estreitamos e levantamos deliberadamente um pé com uma perturbação de
    // intensidade humana. Depois de solto, o controlador não pode aceitar
    // essa configuração como um novo repouso: precisa replantar e reconstruir
    // uma base bilateral. Cruzamento extremo continua coberto no planejador;
    // prender o tornozelo com dezenas de kN mede ruptura, não equilíbrio.
    {
        auto scene = engine.createScene(settings, materials);
        createStaticBox(*scene, { 0.0f, 0.0f, -0.05f },
            { 20.0f, 20.0f, 0.05f });
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 41'201;
        spawn.pelvisPosition = {
            0.0f, 0.0f, profile.standingRootHeightMeters + 0.055f
        };
        spawn.orientation = Quaternion::fromAxisAngle(
            { 0.0f, 0.0f, 1.0f }, 0.33f);
        spawn.active = true;
        const RagdollHandle3D handle = scene->createRagdoll(profile, spawn);
        ActiveRagdollController3D controller;
        controller.reset(profile, spawn.orientation);
        const auto stepOnce = [&]() {
            const RagdollDynamics3D dynamics =
                scene->ragdollDynamics(handle);
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain, &dynamics);
            const ActiveRagdollControlOutput3D& output =
                controller.output();
            if (output.applyRootForce) {
                scene->applyRagdollRootForce(handle,
                    output.rootForceNewtons,
                    output.rootTorqueNewtonMeters);
            }
            if (output.applySpineForce) {
                scene->applyRagdollLinkForce(handle,
                    output.spineLinkIndex, output.spineForceNewtons,
                    output.spineTorqueNewtonMeters);
            }
            scene->setRagdollActiveDriveTargets(handle,
                output.driveTargets,
                output.gravityCompensationEnabled);
            scene->simulate(1.0f / 120.0f);
        };
        for (int step = 0; step < 360; ++step) stepOnce();
        const RagdollState3D before = scene->ragdollState(handle);
        const Vec3 forward =
            spawn.orientation.rotate({ 1.0f, 0.0f, 0.0f });
        const Vec3 lateral { -forward.y, forward.x, 0.0f };
        PhysicsGrabTarget3D footTarget;
        footTarget.position = before.links[14].position
            - lateral * 0.03f + Vec3 { 0.0f, 0.0f, 0.035f };
        footTarget.orientation = before.links[14].orientation;
        footTarget.lockOrientation = true;
        PhysicsHandleSettings3D footGrab;
        footGrab.linearStiffness = 1'500.0f;
        footGrab.maximumForce = 3'000.0f;
        footGrab.angularStiffness = 450.0f;
        footGrab.maximumTorque = 900.0f;
        require(scene->beginRagdollGrab(handle, 14, {},
                footTarget, footGrab),
            "Teste de postura nao conseguiu deslocar o pe");
        controller.setExternalManipulationActive(true, 14);
        for (int step = 0; step < 12; ++step) {
            scene->updateGrabTarget(footTarget, footGrab);
            stepOnce();
            if (std::getenv("MATTERENGINE_POSTURE_REPAIR_TRACE") != nullptr
                    && step % 6 == 0) {
                const ActiveRagdollTelemetry3D& telemetry =
                    controller.telemetry();
                std::cerr << "[posture-grab " << step << "] up="
                          << telemetry.bodyUpDot << " support=("
                          << telemetry.leftFootSupported << ","
                          << telemetry.rightFootSupported << ") step="
                          << telemetry.recoveryStepActive << "/"
                          << static_cast<int>(
                              telemetry.recoverySwingSide)
                          << " p=" << telemetry.recoveryStepProgress
                          << " margin=" << telemetry.captureMarginMeters
                          << " urgency=" << telemetry.recoveryUrgency
                          << '\n';
            }
        }
        scene->endGrab();
        controller.setExternalManipulationActive(false);

        bool recoveredBase = false;
        for (int step = 0; step < 1'200 && !recoveredBase; ++step) {
            stepOnce();
            const RagdollState3D current = scene->ragdollState(handle);
            const float signedWidth = dot(
                current.links[14].position
                    - current.links[17].position,
                lateral);
            const ActiveRagdollTelemetry3D& telemetry =
                controller.telemetry();
            recoveredBase =
                telemetry.leftFootSupported
                && telemetry.rightFootSupported
                && !telemetry.recoveryStepActive
                && telemetry.bodyUpDot > 0.88f
                && signedWidth
                    > controller.config().minimumStableStanceWidthMeters
                && telemetry.leftSoleUpDot > 0.97f
                && telemetry.rightSoleUpDot > 0.97f;
            if (std::getenv("MATTERENGINE_POSTURE_REPAIR_TRACE") != nullptr
                    && step % 60 == 0) {
                std::cerr << "[posture-repair " << step << "] up="
                          << telemetry.bodyUpDot << " support=("
                          << telemetry.leftFootSupported << ","
                          << telemetry.rightFootSupported << ") step="
                          << telemetry.recoveryStepActive << "/"
                          << static_cast<int>(
                              telemetry.recoverySwingSide)
                          << " p=" << telemetry.recoveryStepProgress
                          << " signedWidth=" << signedWidth
                          << " stance=" << telemetry.stanceWidthMeters
                          << " sole=(" << telemetry.leftSoleUpDot
                          << "," << telemetry.rightSoleUpDot << ")"
                          << " footZ=(" << current.links[14].position.z
                          << "," << current.links[17].position.z << ")"
                          << " comV=("
                          << telemetry.centerOfMassVelocity.x << ","
                          << telemetry.centerOfMassVelocity.y << ")\n";
            }
        }
        require(recoveredBase,
            "Ragdoll aceitou pe suspenso/cruzado em vez de reconstruir "
            "a postura bilateral");
    }

    // --- Empurrões nas oito direções, duas magnitudes, ragdoll único.
    // Testa perturbação repetida (como realmente acontece em jogo) em vez
    // de um único empurrão isolado por direção.
    {
        auto scene = engine.createScene(settings, materials);
        createStaticBox(*scene, { 0.0f, 0.0f, -0.05f },
            { 20.0f, 20.0f, 0.05f });
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 42'001;
        spawn.pelvisPosition = {
            0.0f, 0.0f, profile.standingRootHeightMeters + 0.055f
        };
        spawn.active = true;
        const RagdollHandle3D handle = scene->createRagdoll(profile, spawn);
        ActiveRagdollController3D controller;
        controller.reset(profile, spawn.orientation);
        const auto applyControlOutput = [&]() {
            const ActiveRagdollControlOutput3D& output =
                controller.output();
            if (output.applyRootForce) {
                scene->applyRagdollRootForce(handle,
                    output.rootForceNewtons,
                    output.rootTorqueNewtonMeters);
            }
            if (output.applySpineForce) {
                scene->applyRagdollLinkForce(handle,
                    output.spineLinkIndex, output.spineForceNewtons,
                    output.spineTorqueNewtonMeters);
            }
            scene->setRagdollActiveDriveTargets(handle,
                output.driveTargets,
                output.gravityCompensationEnabled);
        };
        for (int step = 0; step < 240; ++step) {
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain);
            applyControlOutput();
            scene->simulate(1.0f / 120.0f);
        }

        struct PushDirection3D { float x; float y; const char* label; };
        const std::array<PushDirection3D, 8> directions {{
            { 1.0f, 0.0f, "frente" }, { 0.7071f, 0.7071f, "frente-direita" },
            { 0.0f, 1.0f, "direita" }, { -0.7071f, 0.7071f, "trás-direita" },
            { -1.0f, 0.0f, "trás" }, { -0.7071f, -0.7071f, "trás-esquerda" },
            { 0.0f, -1.0f, "esquerda" },
            { 0.7071f, -0.7071f, "frente-esquerda" }
        }};
        for (const float dragMeters : { 0.10f, 0.22f }) {
            for (const PushDirection3D& direction : directions) {
                const RagdollState3D before = scene->ragdollState(handle);
                PhysicsGrabTarget3D pushTarget;
                pushTarget.position = before.links[3].position;
                pushTarget.orientation = before.links[3].orientation;
                PhysicsHandleSettings3D pushSettings;
                pushSettings.linearStiffness = 900.0f;
                pushSettings.maximumForce = 2'500.0f;
                require(scene->beginRagdollGrab(handle, 3, {}, pushTarget,
                        pushSettings),
                    (std::string("Não foi possível empurrar o tronco (")
                        + direction.label + ")").c_str());
                for (int step = 0; step < 12; ++step) {
                    pushTarget.position.x = before.links[3].position.x
                        + direction.x * dragMeters
                            * (static_cast<float>(step + 1) / 12.0f);
                    pushTarget.position.y = before.links[3].position.y
                        + direction.y * dragMeters
                            * (static_cast<float>(step + 1) / 12.0f);
                    scene->updateGrabTarget(pushTarget, pushSettings);
                    controller.update(profile, scene->ragdollState(handle),
                        1.0f / 120.0f, &terrain);
                    applyControlOutput();
                    scene->simulate(1.0f / 120.0f);
                }
                scene->endGrab();
                bool recovered = false;
                for (int step = 0; step < 600 && !recovered; ++step) {
                    controller.update(profile, scene->ragdollState(handle),
                        1.0f / 120.0f, &terrain);
                    const ActiveRagdollControlOutput3D& output =
                        controller.output();
                    static_cast<void>(output);
                    applyControlOutput();
                    scene->simulate(1.0f / 120.0f);
                    const ActiveRagdollTelemetry3D& telemetry =
                        controller.telemetry();
                    recovered = telemetry.bodyUpDot > 0.90f
                        && telemetry.torsoUpDot > 0.90f
                        && !telemetry.recoveryStepActive
                        && (telemetry.phase
                                == ActiveRagdollPhase3D::Standing
                            || telemetry.phase
                                == ActiveRagdollPhase3D::Balancing);
                }
                require(recovered,
                    (std::string("Ragdoll ativo não recuperou o equilíbrio "
                        "após empurrão de ")
                        + std::to_string(dragMeters) + " m na direção "
                        + direction.label).c_str());
                const RagdollState3D afterRecovery =
                    scene->ragdollState(handle);
                for (const PhysicsBodyState3D& link : afterRecovery.links) {
                    require(std::isfinite(link.position.x)
                            && link.linearVelocity.length() < 15.0f,
                        (std::string("Estado não finito/explosivo após "
                            "empurrão na direção ") + direction.label)
                            .c_str());
                }
            }
        }
    }

    // --- Perturbação no meio de uma passada em andamento. Os testes de
    // empurrão acima partem sempre de Standing parado; este cobre o caso
    // real de jogo — o personagem já está caminhando quando é tocado.
    // O ciclo atual deliberadamente não inclui locomoção de 5 metros; o
    // perfil de validação de equilíbrio encerra aqui, depois dos impactos
    // estacionários nas oito direções.
    if (std::getenv("MATTERENGINE_WBC_BALANCE_ONLY") != nullptr) return;
    {
        auto scene = engine.createScene(settings, materials);
        createStaticBox(*scene, { 0.0f, 0.0f, -0.05f },
            { 40.0f, 40.0f, 0.05f });
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 43'001;
        spawn.pelvisPosition = {
            0.0f, 0.0f, profile.standingRootHeightMeters + 0.055f
        };
        spawn.active = true;
        const RagdollHandle3D handle = scene->createRagdoll(profile, spawn);
        ActiveRagdollController3D controller;
        controller.reset(profile, spawn.orientation);
        const auto applyControlOutput = [&]() {
            const ActiveRagdollControlOutput3D& output =
                controller.output();
            if (output.applyRootForce) {
                scene->applyRagdollRootForce(handle,
                    output.rootForceNewtons,
                    output.rootTorqueNewtonMeters);
            }
            if (output.applySpineForce) {
                scene->applyRagdollLinkForce(handle,
                    output.spineLinkIndex, output.spineForceNewtons,
                    output.spineTorqueNewtonMeters);
            }
            scene->setRagdollActiveDriveTargets(handle,
                output.driveTargets,
                output.gravityCompensationEnabled);
        };
        for (int settleStep = 0; settleStep < 60; ++settleStep) {
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain);
            applyControlOutput();
            scene->simulate(1.0f / 120.0f);
        }
        controller.requestWalkDistance(5.0f);
        int step = 0;
        for (; step < 15'000
                && controller.telemetry().completedWalkSteps < 3; ++step) {
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain);
            applyControlOutput();
            scene->simulate(1.0f / 120.0f);
        }
        require(controller.telemetry().completedWalkSteps >= 3,
            "Caminhada não estabilizou passos suficientes para o teste "
            "de perturbação em andamento");
        const RagdollState3D midWalk = scene->ragdollState(handle);
        PhysicsGrabTarget3D midWalkPush;
        midWalkPush.position = midWalk.links[3].position;
        midWalkPush.orientation = midWalk.links[3].orientation;
        PhysicsHandleSettings3D midWalkPushSettings;
        midWalkPushSettings.linearStiffness = 900.0f;
        midWalkPushSettings.maximumForce = 2'500.0f;
        require(scene->beginRagdollGrab(handle, 3, {}, midWalkPush,
                midWalkPushSettings),
            "Não foi possível empurrar o tronco durante a caminhada");
        for (int pushStep = 0; pushStep < 10; ++pushStep) {
            midWalkPush.position.y = midWalk.links[3].position.y
                + 0.16f * (static_cast<float>(pushStep + 1) / 10.0f);
            scene->updateGrabTarget(midWalkPush, midWalkPushSettings);
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain);
            applyControlOutput();
            scene->simulate(1.0f / 120.0f);
        }
        scene->endGrab();
        for (; step < 24'000
                && (controller.telemetry().walking
                    || controller.telemetry().walkPending
                    || step == 0); ++step) {
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain);
            applyControlOutput();
            scene->simulate(1.0f / 120.0f);
        }
        // Depois de um empurrão lateral em pleno passo, mais 480 passos
        // (4 s) de estabilização — sem exigir que a caminhada de 5 m
        // originalmente pedida ainda seja concluída à risca.
        for (int settleStep = 0; settleStep < 480; ++settleStep) {
            controller.update(profile, scene->ragdollState(handle),
                1.0f / 120.0f, &terrain);
            applyControlOutput();
            scene->simulate(1.0f / 120.0f);
        }
        const ActiveRagdollTelemetry3D& afterPush = controller.telemetry();
        require(afterPush.phase != ActiveRagdollPhase3D::Falling,
            "Empurrão em pleno passo derrubou o ragdoll ativo");
        require(afterPush.bodyUpDot > 0.85f && afterPush.torsoUpDot > 0.85f,
            "Ragdoll ativo não manteve postura plausível após empurrão "
            "em pleno passo");
    }
}

void testCharacterActiveStanding() {
    const auto character=loadRagdollCharacter3D(std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/characters/crash_test_dummy/character.json");
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    auto scene=engine.createScene({},materials);
    createStaticGround(engine,*scene);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId=70000; spawn.active=true;
    spawn.pelvisPosition={0,0,character.profile.standingRootHeightMeters+0.055f};
    spawn.orientation=Quaternion::fromAxisAngle({0,0,1},0.61f);
    const auto handle=scene->createRagdoll(character.profile,spawn);
    ActiveRagdollController3D controller;
    controller.reset(character.profile,spawn.orientation);
    controller.config().useMagicPelvisStabilization=false;
    RealisticFootworkTerrain terrain(*scene);
    for (int step=0;step<720;++step) {
        const auto state=scene->ragdollState(handle);
        const auto dynamics=scene->ragdollDynamics(handle);
        controller.update(character.profile,state,1.0f/120.0f,&terrain,&dynamics);
        const auto& output=controller.output();
        require(!output.applyRootForce&&!output.applySpineForce,"Character standing used external assistance");
        scene->setRagdollActiveDriveTargets(handle,output.driveTargets,output.gravityCompensationEnabled);
        scene->simulate(1.0f/120.0f);
        if (step%120==0) {
            const auto& t=controller.telemetry();
            std::cout<<"Character standing "<<step<<" up="<<t.bodyUpDot<<" torso="<<t.torsoUpDot
                <<" COMz="<<t.centerOfMass.z<<" support="<<t.supportContactCount
                <<" solved="<<t.wholeBodySolved<<" torque="<<t.wholeBodyMaximumTorqueNewtonMeters<<std::endl;
        }
    }
    require(controller.telemetry().bodyUpDot>0.9f && controller.telemetry().torsoUpDot>0.9f,
        "Character did not maintain unassisted standing");
}

void testCharacterPhysicalRuntime() {
    const auto character=loadRagdollCharacter3D(std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/characters/crash_test_dummy/character.json");
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D settings;
    auto scene=engine.createScene(settings,materials);
    createStaticGround(engine,*scene);
    std::vector<RagdollHandle3D> handles;
    for (int i=0;i<22;++i) {
        RagdollSpawnDefinition3D spawn;
        spawn.entityId=60000+static_cast<std::uint64_t>(i);
        spawn.active=false;
        spawn.rigidityPercent=15;
        spawn.pelvisPosition={static_cast<float>(i%6)*0.8f,
            static_cast<float>(i/6)*0.8f,character.profile.standingRootHeightMeters+0.08f};
        spawn.orientation=Quaternion::fromAxisAngle({0,0,1},static_cast<float>(i)*0.31f);
        handles.push_back(scene->createRagdoll(character.profile,spawn));
    }
    float maxSeparation=0;
    std::size_t contacts=0;
    for (int step=0;step<240;++step) {
        scene->simulate(1.0f/120.0f);
        contacts=std::max(contacts,scene->diagnostics().discreteContactPairs);
        for (const auto handle : handles) {
            const auto state=scene->ragdollState(handle);
            require(state.links.size()==character.profile.links.size(),"New physical rig lost links");
            std::vector<Vec3> positions;
            std::vector<Quaternion> orientations;
            for (const auto& link : state.links) {
                require(std::isfinite(link.position.x)&&std::isfinite(link.position.y)
                    && std::isfinite(link.position.z)&&link.position.z>-0.5f
                    && link.position.z<4.0f,"Low-poly physical body escaped simulation bounds");
                positions.push_back(link.position); orientations.push_back(link.orientation);
            }
            for (std::size_t i=1;i<state.links.size();++i) {
                const auto& link=character.profile.links[i];
                const auto parent=static_cast<std::size_t>(link.parentIndex);
                const auto anchor=link.inboundJoint.anchorModelPosition;
                const auto a=positions[parent]+orientations[parent].rotate(
                    character.profile.links[parent].modelOrientation.conjugate().rotate(
                        anchor-character.profile.links[parent].modelPosition));
                const auto b=positions[i]+orientations[i].rotate(
                    link.modelOrientation.conjugate().rotate(anchor-link.modelPosition));
                maxSeparation=std::max(maxSeparation,(a-b).length());
            }
            if (step%60==0) {
                const auto palette=buildRagdollSkinMatrices3D(character,positions,orientations);
                for (const auto& vertex : character.mesh.vertices) {
                    const auto p=skinVertexPosition3D(vertex,palette);
                    require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z),
                        "Physical pose produced a non-finite surface");
                }
            }
        }
    }
    require(contacts>0,"Low-poly crowd test did not exercise collisions");
    require(maxSeparation<0.015f,"Low-poly crowd disconnected physical joints");
    std::cout<<"Low-poly 22-body stress: max anchor gap "<<maxSeparation<<" m\n";

    // Exercise the same articulation handle consumed by the Physgun.
    const auto handle=handles.front();
    const auto before=scene->ragdollState(handle);
    const auto& pelvis=before.links.front();
    PhysicsRagdollRayHit3D hit;
    require(scene->raycastRagdoll({pelvis.position+Vec3{-2,0,0},{1,0,0}},4,hit)
        && hit.ragdoll==handle,"Physgun ray did not find the new character");
    PhysicsGrabTarget3D target;
    target.position=pelvis.position;
    target.orientation=pelvis.orientation;
    PhysicsHandleSettings3D grabSettings;
    require(scene->beginRagdollGrab(handle,0,{},target,grabSettings),
        "Physgun could not attach to the new character");
    target.position.z+=0.3f;
    scene->updateGrabTarget(target,grabSettings);
    for (int step=0;step<60;++step) scene->simulate(1.0f/120.0f);
    require(scene->ragdollState(handle).links.front().position.z>pelvis.position.z+0.1f,
        "Physgun drive did not lift the new character");
    scene->endGrab();
    require(!scene->grabbing(),"Physgun did not release the new character");
}

} // namespace

int main() {
    try {
        testAnimationClipSampling();
        testRagdollCharacterSkin();
        if (const char* filter=std::getenv("MATTERENGINE_TEST_FILTER");
            filter && std::string_view(filter)=="character-active") {
            testCharacterActiveStanding();
            std::cout<<"MatterEngine character active standing passed\n";
            return 0;
        }
        if (const char* filter=std::getenv("MATTERENGINE_TEST_FILTER");
            filter && std::string_view(filter)=="character") {
            testCharacterPhysicalRuntime();
            testCharacterActiveStanding();
            std::cout<<"MatterEngine character tests passed\n";
            return 0;
        }
        if (const char* filter = std::getenv("MATTERENGINE_TEST_FILTER");
            filter != nullptr
            && std::string_view(filter) == "animation") {
            std::cout << "MatterEngine animation foundation tests passed\n";
            return 0;
        }
        testRagdollImpactGenerator();
        testCharacterPhysicalRuntime();
        if (const char* filter = std::getenv("MATTERENGINE_TEST_FILTER");
            filter != nullptr) {
            const std::string_view selected(filter);
            if (selected == "active-core") {
                MaterialLibrary materials;
                PhysicsEngine3D engine;
                testRagdollProfileAndRuntime(engine, materials);
                std::cout
                    << "MatterEngine active-ragdoll core tests passed\n";
                return 0;
            }
            if (selected == "active-robustness") {
                MaterialLibrary materials;
                PhysicsEngine3D engine;
                testActiveRagdollBalanceRobustness(engine, materials);
                std::cout
                    << "MatterEngine active-ragdoll robustness tests passed\n";
                return 0;
            }
        }
        testFootworkPlanner();
        testTaskScheduler();
        testFrustumCulling();
        testPackSceneLights();
        testHash();
        testWindSystem();
        testOceanSurface();
        testHaltonJitter();
        testPerspectiveJittered();
        testShadowCascadeSplits();
        testFitCascadeFrustumToCamera();
        testCascadeFrustumContainsNearbyProp();
        testMetadataAndMaterials();
        testAcousticZoneParsing();
        {
            MaterialLibrary materials;
            PhysicsEngine3D engine;
            testCookingConvexo(engine);
            testDiagnoseHullFit(engine);
            testRealPropCooking(engine, materials);
            testRigidBodiesAndEvents(engine, materials);
            testOceanPhysicsLifecycle(engine, materials);
            testWindShelterExposure(engine, materials);
            testContactSlideEvents(engine, materials);
            testCompoundSupport(engine, materials);
            testPhysGunDrive(engine, materials);
            testCharacterController(engine, materials);
            testThousandSleepingBodies(engine, materials);
            testRagdollProfileAndRuntime(engine, materials);
            testPhysicalFootworkController(engine, materials);
            testActiveRagdollBalanceRobustness(engine, materials);
            testActiveRagdollMagicPushRecovery(engine, materials);
            testActiveRagdollMagicPelvisStabilization(engine, materials);
        }
        testBackendResourceLifetime();
        std::cout << "MatterEngine PhysX foundation tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MatterEngine test failure: " << error.what() << '\n';
        return 1;
    }
}
