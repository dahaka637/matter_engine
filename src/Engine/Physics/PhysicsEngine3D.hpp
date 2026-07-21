#pragma once

#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Physics/PhysicsTypes3D.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine {

class MaterialLibrary;
class PhysicsScene3D;
class TaskScheduler;

enum class PhysicsMeshType3D : std::uint8_t {
    Convex,
    Triangle
};

// Recurso imutavel produzido pelo cooking. O objeto publico carrega apenas um
// PIMPL compartilhado; PxConvexMesh/PxTriangleMesh permanecem confinados ao
// backend e podem ser reutilizados por milhares de instancias.
class PhysicsMesh3D final {
public:
    struct Impl;

    ~PhysicsMesh3D();
    PhysicsMesh3D(const PhysicsMesh3D&) = delete;
    PhysicsMesh3D& operator=(const PhysicsMesh3D&) = delete;

    [[nodiscard]] PhysicsMeshType3D type() const { return m_type; }
    [[nodiscard]] std::uint32_t sourceTriangleCount() const {
        return m_sourceTriangleCount;
    }

private:
    PhysicsMesh3D(std::shared_ptr<Impl> implementation,
        PhysicsMeshType3D type, std::uint32_t sourceTriangleCount);

    std::shared_ptr<Impl> m_impl;
    PhysicsMeshType3D m_type = PhysicsMeshType3D::Convex;
    std::uint32_t m_sourceTriangleCount = 0;

    friend class PhysicsEngine3D;
    friend class PhysicsScene3D;
};

enum class PhysicsShapeType3D : std::uint8_t {
    Box,
    Sphere,
    Capsule,
    ConvexMesh,
    TriangleMesh
};

struct PhysicsShape3D {
    PhysicsShapeType3D type = PhysicsShapeType3D::Box;
    Vec3 localPosition;
    Quaternion localOrientation;
    Vec3 halfExtents { 0.5f, 0.5f, 0.5f };
    float radius = 0.5f;
    float capsuleHalfHeight = 0.5f;
    std::shared_ptr<const PhysicsMesh3D> mesh;
    std::string materialId = "default";
};

struct ConvexDecompositionSettings3D {
    // Orçamentos conservadores para props de gameplay. Mais hulls aumentam a
    // fidelidade, mas tambem o numero de pares no narrow phase.
    std::uint32_t maximumHulls = 16;
    std::uint32_t voxelResolution = 200000;
    std::uint32_t maximumVerticesPerHull = 64;
    float maximumVolumeErrorPercent = 1.0f;
    // ATENCAO: shrinkWrap=false foi tentado como correcao para a folga de
    // colisao em partes finas (ver historico), mas causou uma falha nativa do
    // V-HACD/PhysX ao cozinhar props reais (crash fora do alcance do
    // try/catch de PropCatalog::load, nao uma excecao C++ comum). Revertido
    // para o valor original ate essa hipotese ser investigada com mais
    // cuidado (testando prop a prop, fora do caminho critico do jogo).
    bool shrinkWrap = true;
};

struct CookedDynamicCollision3D {
    std::vector<PhysicsShape3D> shapes;
    Vec3 localCenter;
    Vec3 dimensionsMeters;
    float volumeCubicMeters = 0.0f;
    std::uint32_t hullCount = 0;
};

struct StaticTriangleMeshCookingSettings3D {
    // Triangulos enormes pioram a geracao de contatos no PhysX. A tessellacao
    // divide apenas a superficie, sem deslocar ou aproximar a geometria.
    float maximumEdgeLengthMeters = 50.0f;
    std::uint32_t maximumOutputTriangles = 2'000'000;
};

struct PhysicsSceneSettings3D {
    Vec3 gravity { 0.0f, 0.0f, -9.81f };
    Vec3 airVelocity;
    float airDensityKgPerCubicMeter = 1.225f;
    // Alcance (metros) do raio lancado a barlavento de cada corpo com arrasto
    // aerodinamico ligado, checando se ha parede/geometria estatica entre o
    // corpo e de onde o vento sopra (ver PhysXScene3D::simulate). Uma parede
    // dentro desse alcance abriga o corpo - o ar parado atras dela nao
    // deveria empurrar com a mesma forca do vento livre. 0 desliga a
    // checagem (comportamento antigo: vento sempre uniforme na cena).
    float windShelterDistanceMeters = 4.0f;
    std::uint32_t workerThreadCount = 0;
    std::uint32_t solverPositionIterations = 8;
    std::uint32_t solverVelocityIterations = 2;
    // Memoria temporaria reutilizada pelo solver. O PhysX exige blocos
    // alinhados e multiplos de 16 KiB; a cena normaliza o valor informado.
    std::uint32_t scratchBufferSizeBytes = 1024u * 1024u;
    bool enableContinuousCollision = true;
    bool enableStabilization = true;
};

// Dono do SDK, dispatcher e recursos cozidos. Existe uma instancia por
// processo; cenas independentes compartilham o mesmo PhysX sem singletons.
class PhysicsEngine3D final {
public:
    struct Impl;

    PhysicsEngine3D();
    explicit PhysicsEngine3D(std::shared_ptr<TaskScheduler> scheduler);
    ~PhysicsEngine3D();
    PhysicsEngine3D(const PhysicsEngine3D&) = delete;
    PhysicsEngine3D& operator=(const PhysicsEngine3D&) = delete;

    [[nodiscard]] std::unique_ptr<PhysicsScene3D> createScene(
        const PhysicsSceneSettings3D& settings,
        const MaterialLibrary& materials);

    [[nodiscard]] CookedDynamicCollision3D cookDynamicCollision(
        const MeshData3D& mesh,
        const ConvexDecompositionSettings3D& settings = {},
        std::string_view cachePath = {});
    [[nodiscard]] std::shared_ptr<const PhysicsMesh3D>
        cookStaticTriangleMesh(const MeshData3D& mesh,
            const StaticTriangleMeshCookingSettings3D& settings = {});

private:
    // O contexto e compartilhado apenas dentro do backend: cenas e meshes
    // cozidas podem terminar de liberar recursos mesmo se a fachada publica
    // PhysicsEngine3D for destruida primeiro.
    std::shared_ptr<Impl> m_impl;

    friend class PhysicsScene3D;
};

} // namespace MatterEngine
