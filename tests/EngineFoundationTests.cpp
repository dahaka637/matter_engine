#include "Engine/Environment/WindSystem.hpp"
#include "Engine/Geometry/GltfAcousticZone3D.hpp"
#include "Engine/Geometry/GltfPhysicsMetadata3D.hpp"
#include "Engine/Geometry/PhysicalAsset3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicalBodyBuilder3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
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
#include <stdexcept>
#include <string>
#include <span>
#include <vector>

namespace {

using namespace MatterEngine;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
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

    // Marcos explicitos da rampa: na linha do chao (altura 0, o default de
    // groundHeightMeters) a influencia do vento deveria ser EXATAMENTE zero
    // - nao so "bem menor", zero de verdade (fisica e audio dependem dessa
    // garantia para simular abrigo total ao nivel do chao). Acima da altura
    // de referencia (default 10 m) a rampa satura: 200 m nao deveria soprar
    // mais forte que exatamente na altura de referencia.
    const float groundSpeed = windProfile.velocityAtHeight(0.0f).length();
    require(groundSpeed < 1e-6f,
        "Vento na linha do chao deveria ter influencia zero");
    require(std::abs(highSpeed - referenceSpeed) < 1e-5f,
        "Rampa do vento deveria saturar na altura de referencia, nao crescer alem dela");

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

} // namespace

int main() {
    try {
        testTaskScheduler();
        testFrustumCulling();
        testPackSceneLights();
        testHash();
        testWindSystem();
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
            testCompoundSupport(engine, materials);
            testPhysGunDrive(engine, materials);
            testCharacterController(engine, materials);
            testThousandSleepingBodies(engine, materials);
        }
        testBackendResourceLifetime();
        std::cout << "MatterEngine PhysX foundation tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MatterEngine test failure: " << error.what() << '\n';
        return 1;
    }
}
