#include "Engine/Core/Log.hpp"
#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Physics/Jolt/JoltConversions.hpp"
#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include "Engine/Physics/Jolt/JoltShapes3D.hpp"

#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>

#define ENABLE_VHACD_IMPLEMENTATION 1
#include <VHACD.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace MatterEngine {
namespace {

constexpr float GeometryEpsilon = 1.0e-6f;

// Cabecalho e hash sao IDENTICOS aos do backend PhysX, de proposito: o cache
// `.mecollider` guarda os PONTOS dos hulls do V-HACD, que sao geometria
// backend-neutra, e nao binario cozido de SDK. Manter magic, versao e hash
// significa que todo cache ja gravado em disco continua valido depois da troca
// de motor fisico - a incompatibilidade que se supunha existir nao existe.
constexpr std::array<char, 8> CollisionCacheMagic {
    'M', 'E', 'C', 'O', 'L', '3', 'D', '\0'
};
constexpr std::uint32_t CollisionCacheVersion = 1;
constexpr std::uint32_t CollisionCacheEndianMarker = 0x01020304u;
constexpr std::uint32_t MaximumCachedHulls = 128;
constexpr std::uint32_t MaximumCachedVerticesPerHull = 64;

struct CachedCollisionData {
    float volumeCubicMeters = 0.0f;
    std::vector<std::vector<Vec3>> hulls;
};

struct TrianglePositions3D {
    Vec3 a;
    Vec3 b;
    Vec3 c;
};

struct TessellatedTriangleMesh3D {
    std::vector<Vec3> positions;
    std::vector<std::uint32_t> indices;
};

void hashU32(std::uint64_t& hash, std::uint32_t value) {
    // FNV-1a e suficiente aqui: o hash detecta mudancas de autoria, nao e um
    // mecanismo de seguranca. A ordem explicita dos bytes torna o resultado
    // independente do padding das structs C++.
    constexpr std::uint64_t FnvPrime = 1099511628211ull;
    for (unsigned shift = 0; shift < 32; shift += 8) {
        hash ^= static_cast<std::uint8_t>(value >> shift);
        hash *= FnvPrime;
    }
}

void hashFloat(std::uint64_t& hash, float value) {
    hashU32(hash, std::bit_cast<std::uint32_t>(value));
}

std::uint64_t collisionSourceHash(const MeshData3D& mesh,
    const ConvexDecompositionSettings3D& settings) {
    std::uint64_t hash = 14695981039346656037ull;
    hashU32(hash, CollisionCacheVersion);
    hashU32(hash, static_cast<std::uint32_t>(mesh.vertices.size()));
    hashU32(hash, static_cast<std::uint32_t>(mesh.indices.size()));
    for (const MeshVertex3D& vertex : mesh.vertices) {
        hashFloat(hash, vertex.position.x);
        hashFloat(hash, vertex.position.y);
        hashFloat(hash, vertex.position.z);
    }
    for (const std::uint32_t index : mesh.indices) hashU32(hash, index);
    hashU32(hash, settings.maximumHulls);
    hashU32(hash, settings.voxelResolution);
    hashU32(hash, settings.maximumVerticesPerHull);
    hashFloat(hash, settings.maximumVolumeErrorPercent);
    hashU32(hash, settings.shrinkWrap ? 1u : 0u);
    return hash;
}

template <typename T>
bool readValue(std::ifstream& stream, T& value) {
    stream.read(reinterpret_cast<char*>(&value), sizeof(T));
    return stream.good();
}

template <typename T>
bool writeValue(std::ofstream& stream, const T& value) {
    stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
    return stream.good();
}

[[nodiscard]] bool isFinite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z);
}

std::optional<CachedCollisionData> readCollisionCache(
    const std::filesystem::path& path, std::uint64_t expectedHash) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) return std::nullopt;

    std::array<char, CollisionCacheMagic.size()> magic {};
    stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
    std::uint32_t version = 0;
    std::uint32_t endian = 0;
    std::uint64_t sourceHash = 0;
    std::uint32_t hullCount = 0;
    CachedCollisionData data;
    if (!stream.good() || magic != CollisionCacheMagic
        || !readValue(stream, version)
        || !readValue(stream, endian)
        || !readValue(stream, sourceHash)
        || !readValue(stream, data.volumeCubicMeters)
        || !readValue(stream, hullCount)
        || version != CollisionCacheVersion
        || endian != CollisionCacheEndianMarker
        || sourceHash != expectedHash
        || !std::isfinite(data.volumeCubicMeters)
        || data.volumeCubicMeters <= 0.0f
        || hullCount == 0 || hullCount > MaximumCachedHulls) {
        return std::nullopt;
    }

    data.hulls.reserve(hullCount);
    for (std::uint32_t hullIndex = 0; hullIndex < hullCount; ++hullIndex) {
        std::uint32_t vertexCount = 0;
        if (!readValue(stream, vertexCount) || vertexCount < 4
            || vertexCount > MaximumCachedVerticesPerHull) {
            return std::nullopt;
        }
        std::vector<Vec3> points(vertexCount);
        for (Vec3& point : points) {
            if (!readValue(stream, point.x)
                || !readValue(stream, point.y)
                || !readValue(stream, point.z)
                || !isFinite(point)) {
                return std::nullopt;
            }
        }
        data.hulls.push_back(std::move(points));
    }
    // Bytes adicionais indicam formato inesperado ou arquivo corrompido.
    char trailing = 0;
    if (stream.read(&trailing, 1)) return std::nullopt;
    return data;
}

bool writeCollisionCache(const std::filesystem::path& path,
    std::uint64_t sourceHash, const CachedCollisionData& data) {
    try {
        const std::filesystem::path parent = path.parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent);
        std::filesystem::path temporary = path;
        temporary += ".tmp";
        std::ofstream stream(temporary,
            std::ios::binary | std::ios::trunc);
        if (!stream) return false;
        const auto discardTemporary = [&]() {
            stream.close();
            std::error_code removeError;
            std::filesystem::remove(temporary, removeError);
            return false;
        };
        stream.write(CollisionCacheMagic.data(),
            static_cast<std::streamsize>(CollisionCacheMagic.size()));
        const std::uint32_t hullCount = static_cast<std::uint32_t>(
            data.hulls.size());
        if (!stream.good()
            || !writeValue(stream, CollisionCacheVersion)
            || !writeValue(stream, CollisionCacheEndianMarker)
            || !writeValue(stream, sourceHash)
            || !writeValue(stream, data.volumeCubicMeters)
            || !writeValue(stream, hullCount)) {
            return discardTemporary();
        }
        for (const std::vector<Vec3>& points : data.hulls) {
            const std::uint32_t vertexCount = static_cast<std::uint32_t>(
                points.size());
            if (!writeValue(stream, vertexCount)) return discardTemporary();
            for (const Vec3& point : points) {
                if (!writeValue(stream, point.x)
                    || !writeValue(stream, point.y)
                    || !writeValue(stream, point.z)) {
                    return discardTemporary();
                }
            }
        }
        stream.flush();
        if (!stream.good()) return discardTemporary();
        stream.close();
        std::error_code error;
        std::filesystem::remove(path, error);
        error.clear();
        std::filesystem::rename(temporary, path, error);
        if (error) {
            std::filesystem::remove(temporary, error);
            return false;
        }
        return true;
    } catch (const std::filesystem::filesystem_error&) {
        return false;
    }
}

void validateTriangleMesh(const MeshData3D& mesh) {
    if (mesh.vertices.empty() || mesh.indices.empty()
        || mesh.indices.size() % 3 != 0) {
        throw std::runtime_error(
            "Cooking de fisica recebeu uma malha triangular vazia ou invalida");
    }
    if (mesh.vertices.size() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error("Malha excede o limite de vertices suportado");
    }
    for (const std::uint32_t index : mesh.indices) {
        if (index >= mesh.vertices.size()) {
            throw std::runtime_error(
                "Malha contem indice fora do intervalo de vertices");
        }
    }
}

TessellatedTriangleMesh3D tessellateStaticMesh(const MeshData3D& mesh,
    const StaticTriangleMeshCookingSettings3D& settings) {
    if (!std::isfinite(settings.maximumEdgeLengthMeters)
        || settings.maximumEdgeLengthMeters <= 0.0f
        || settings.maximumOutputTriangles == 0) {
        throw std::invalid_argument(
            "Parametros de tessellacao estatica invalidos");
    }
    const float maximumEdgeSquared = settings.maximumEdgeLengthMeters
        * settings.maximumEdgeLengthMeters;
    std::vector<TrianglePositions3D> pending;
    pending.reserve(mesh.indices.size() / 3);
    for (std::size_t index = 0; index < mesh.indices.size(); index += 3) {
        pending.push_back({
            mesh.vertices[mesh.indices[index]].position,
            mesh.vertices[mesh.indices[index + 1]].position,
            mesh.vertices[mesh.indices[index + 2]].position
        });
    }

    std::vector<TrianglePositions3D> output;
    while (!pending.empty()) {
        const TrianglePositions3D triangle = pending.back();
        pending.pop_back();
        const float edgeAB = (triangle.a - triangle.b).lengthSquared();
        const float edgeBC = (triangle.b - triangle.c).lengthSquared();
        const float edgeCA = (triangle.c - triangle.a).lengthSquared();
        const float longestEdge = std::max({ edgeAB, edgeBC, edgeCA });
        if (longestEdge <= maximumEdgeSquared) {
            output.push_back(triangle);
            continue;
        }
        if (output.size() + pending.size() + 2
            > settings.maximumOutputTriangles) {
            throw std::runtime_error(
                "Tessellacao estatica excedeu o orcamento de triangulos");
        }

        // Dividir sempre a maior aresta evita triangulos longos e finos. A
        // ordem dos vertices e preservada nos dois filhos, portanto normais e
        // face frontal continuam identicas a superficie importada.
        if (longestEdge == edgeAB) {
            const Vec3 midpoint = (triangle.a + triangle.b) * 0.5f;
            pending.push_back({ triangle.a, midpoint, triangle.c });
            pending.push_back({ midpoint, triangle.b, triangle.c });
        } else if (longestEdge == edgeBC) {
            const Vec3 midpoint = (triangle.b + triangle.c) * 0.5f;
            pending.push_back({ triangle.a, triangle.b, midpoint });
            pending.push_back({ triangle.a, midpoint, triangle.c });
        } else {
            const Vec3 midpoint = (triangle.c + triangle.a) * 0.5f;
            pending.push_back({ triangle.a, triangle.b, midpoint });
            pending.push_back({ midpoint, triangle.b, triangle.c });
        }
    }

    TessellatedTriangleMesh3D result;
    result.positions.reserve(output.size() * 3);
    result.indices.reserve(output.size() * 3);
    for (const TrianglePositions3D& triangle : output) {
        for (const Vec3& point : { triangle.a, triangle.b, triangle.c }) {
            result.indices.push_back(static_cast<std::uint32_t>(
                result.positions.size()));
            result.positions.push_back(point);
        }
    }
    return result;
}

std::shared_ptr<PhysicsMesh3D::Impl> createConvexHullMesh(
    const std::shared_ptr<PhysicsEngine3D::Impl>& engine,
    std::span<const Vec3> points) {
    if (points.size() < 4) {
        throw std::runtime_error("Hull convexo possui menos de quatro pontos");
    }
    JPH::Array<JPH::Vec3> hullPoints;
    hullPoints.reserve(points.size());
    for (const Vec3& point : points) hullPoints.push_back(toJolt(point));

    // O V-HACD ja respeita o limite de vertices por hull pedido nas settings;
    // aqui o raio convexo e o unico parametro do Jolt que importa, e o default
    // e o que o SDK recomenda para geometria em escala de metros.
    JPH::ConvexHullShapeSettings settings(hullPoints,
        JPH::cDefaultConvexRadius);
    const JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError()) {
        throw std::runtime_error(
            std::string("Jolt recusou um hull produzido pelo V-HACD: ")
            + result.GetError().c_str());
    }
    auto implementation = std::make_shared<PhysicsMesh3D::Impl>();
    implementation->engineLifetime = engine;
    implementation->shape = result.Get();
    return implementation;
}

} // namespace

CookedDynamicCollision3D PhysicsEngine3D::cookDynamicCollision(
    const MeshData3D& mesh,
    const ConvexDecompositionSettings3D& settings,
    std::string_view cachePath) {
    validateTriangleMesh(mesh);
    if (settings.maximumHulls == 0 || settings.maximumHulls > 128
        || settings.voxelResolution < 10000
        || settings.maximumVerticesPerHull < 8
        || settings.maximumVerticesPerHull > 64) {
        throw std::invalid_argument(
            "Parametros de decomposicao convexa fora dos limites seguros");
    }

    const Vec3 center = (mesh.boundsMin + mesh.boundsMax) * 0.5f;
    const Vec3 dimensions = mesh.boundsMax - mesh.boundsMin;
    if (dimensions.x <= GeometryEpsilon
        || dimensions.y <= GeometryEpsilon
        || dimensions.z <= GeometryEpsilon) {
        throw std::runtime_error("Malha dinamica possui volume degenerado");
    }

    const std::uint32_t sourceTriangleCount =
        static_cast<std::uint32_t>(mesh.indices.size() / 3);
    const std::uint64_t sourceHash = collisionSourceHash(mesh, settings);
    if (!cachePath.empty()) {
        const std::filesystem::path path { std::string(cachePath) };
        if (auto cached = readCollisionCache(path, sourceHash)) {
            Log::info("Collider carregado do cache: " + path.string());
            CookedDynamicCollision3D cooked;
            cooked.localCenter = center;
            cooked.dimensionsMeters = dimensions;
            cooked.volumeCubicMeters = cached->volumeCubicMeters;
            cooked.hullCount = static_cast<std::uint32_t>(
                cached->hulls.size());
            cooked.shapes.reserve(cooked.hullCount);
            for (const std::vector<Vec3>& hull : cached->hulls) {
                PhysicsShape3D shape;
                shape.type = PhysicsShapeType3D::ConvexMesh;
                shape.mesh = std::shared_ptr<const PhysicsMesh3D>(
                    new PhysicsMesh3D(createConvexHullMesh(m_impl, hull),
                        PhysicsMeshType3D::Convex, sourceTriangleCount));
                cooked.shapes.push_back(std::move(shape));
            }
            return cooked;
        }
    }

    // O V-HACD recebe posicoes recentradas no centro fisico: e o mesmo
    // referencial em que os hulls sao gravados no cache e em que as shapes
    // finais vivem.
    std::vector<float> flatPoints;
    flatPoints.reserve(mesh.vertices.size() * 3);
    for (const MeshVertex3D& vertex : mesh.vertices) {
        const Vec3 centered = vertex.position - center;
        flatPoints.insert(flatPoints.end(),
            { centered.x, centered.y, centered.z });
    }

    struct VhacdDeleter {
        void operator()(VHACD::IVHACD* instance) const {
            if (instance != nullptr) instance->Release();
        }
    };
    std::unique_ptr<VHACD::IVHACD, VhacdDeleter> decomposer(
        VHACD::CreateVHACD());
    if (!decomposer) {
        throw std::runtime_error("Nao foi possivel inicializar o V-HACD");
    }
    VHACD::IVHACD::Parameters parameters;
    parameters.m_maxConvexHulls = settings.maximumHulls;
    parameters.m_resolution = settings.voxelResolution;
    parameters.m_maxNumVerticesPerCH = settings.maximumVerticesPerHull;
    parameters.m_minimumVolumePercentErrorAllowed =
        settings.maximumVolumeErrorPercent;
    parameters.m_shrinkWrap = settings.shrinkWrap;
    parameters.m_asyncACD = false;
    parameters.m_fillMode = VHACD::FillMode::RAYCAST_FILL;

    const bool started = decomposer->Compute(flatPoints.data(),
        static_cast<std::uint32_t>(mesh.vertices.size()),
        mesh.indices.data(), sourceTriangleCount, parameters);
    if (!started || decomposer->GetNConvexHulls() == 0) {
        throw std::runtime_error(
            "V-HACD nao conseguiu decompor a malha dinamica");
    }

    CookedDynamicCollision3D cooked;
    cooked.localCenter = center;
    cooked.dimensionsMeters = dimensions;
    cooked.hullCount = decomposer->GetNConvexHulls();
    cooked.shapes.reserve(cooked.hullCount);
    CachedCollisionData cacheData;
    cacheData.hulls.reserve(cooked.hullCount);
    for (std::uint32_t hullIndex = 0; hullIndex < cooked.hullCount;
            ++hullIndex) {
        VHACD::IVHACD::ConvexHull hull;
        if (!decomposer->GetConvexHull(hullIndex, hull)
            || hull.m_points.size() < 4) {
            throw std::runtime_error("V-HACD publicou um hull invalido");
        }
        std::vector<Vec3> hullPoints;
        hullPoints.reserve(hull.m_points.size());
        for (const VHACD::Vertex& point : hull.m_points) {
            hullPoints.push_back({ static_cast<float>(point.mX),
                static_cast<float>(point.mY),
                static_cast<float>(point.mZ) });
        }
        cacheData.hulls.push_back(hullPoints);

        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::ConvexMesh;
        shape.mesh = std::shared_ptr<const PhysicsMesh3D>(new PhysicsMesh3D(
            createConvexHullMesh(m_impl, hullPoints),
            PhysicsMeshType3D::Convex, sourceTriangleCount));
        cooked.shapes.push_back(std::move(shape));
        cooked.volumeCubicMeters += static_cast<float>(
            std::max(0.0, hull.m_volume));
    }
    cacheData.volumeCubicMeters = cooked.volumeCubicMeters;
    if (!cachePath.empty()) {
        const std::filesystem::path path { std::string(cachePath) };
        if (writeCollisionCache(path, sourceHash, cacheData)) {
            Log::info("Collider cache atualizado: " + path.string());
        } else {
            Log::warn("Nao foi possivel gravar collider cache: "
                + path.string());
        }
    }
    return cooked;
}

std::shared_ptr<const PhysicsMesh3D>
PhysicsEngine3D::cookStaticTriangleMesh(const MeshData3D& mesh,
    const StaticTriangleMeshCookingSettings3D& settings) {
    validateTriangleMesh(mesh);
    // A tessellacao continua: o motivo original (triangulos enormes pioram a
    // geracao de contatos) vale para qualquer narrow phase, e ela e geometria
    // pura, nao especifica de SDK.
    const TessellatedTriangleMesh3D tessellated =
        tessellateStaticMesh(mesh, settings);

    JPH::VertexList vertices;
    vertices.reserve(tessellated.positions.size());
    for (const Vec3& position : tessellated.positions) {
        vertices.push_back(JPH::Float3(position.x, position.y, position.z));
    }
    JPH::IndexedTriangleList triangles;
    triangles.reserve(tessellated.indices.size() / 3);
    for (std::size_t index = 0; index + 2 < tessellated.indices.size();
            index += 3) {
        triangles.push_back(JPH::IndexedTriangle(tessellated.indices[index],
            tessellated.indices[index + 1],
            tessellated.indices[index + 2]));
    }

    // O material fica vazio: a resolucao de material por ponto de contato
    // acontece pela shape do corpo (ver JoltSurfaceMaterial), e uma malha
    // estatica do mundo carrega um material so, atribuido na criacao do corpo.
    // O construtor com lista indexada ja chama Sanitize(), que remove
    // triangulos duplicados e degenerados - o papel que eWELD_VERTICES tinha no
    // cooking do PhysX para superficies importadas com costuras repetidas.
    const JPH::MeshShapeSettings shapeSettings(std::move(vertices),
        std::move(triangles));
    const JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) {
        throw std::runtime_error(
            std::string("Jolt nao conseguiu construir a malha estatica: ")
            + result.GetError().c_str());
    }

    auto implementation = std::make_shared<PhysicsMesh3D::Impl>();
    implementation->engineLifetime = m_impl;
    implementation->shape = result.Get();
    return std::shared_ptr<const PhysicsMesh3D>(new PhysicsMesh3D(
        std::move(implementation), PhysicsMeshType3D::Triangle,
        static_cast<std::uint32_t>(mesh.indices.size() / 3)));
}

} // namespace MatterEngine
