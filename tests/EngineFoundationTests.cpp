#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Environment/WindSystem.hpp"
#include "Engine/Control/AnimatedRagdollController3D.hpp"
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
            filter && std::string_view(filter)=="character") {
            testCharacterPhysicalRuntime();
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
        }
        testBackendResourceLifetime();
        std::cout << "MatterEngine PhysX foundation tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MatterEngine test failure: " << error.what() << '\n';
        return 1;
    }
}
