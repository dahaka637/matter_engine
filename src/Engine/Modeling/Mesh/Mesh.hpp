#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"
#include "Engine/Modeling/Raster/Color.hpp"
#include "Engine/Modeling/Raster/Image.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

using Triangle = std::array<std::uint32_t, 3>;

// Ate quatro juntas por vertice (o mesmo limite do jogo).
struct SkinWeights {
    std::array<std::uint16_t, 4> joints {};
    std::array<float, 4> weights {};
};

// Pedaco nomeado da malha (pele do antebraco, camisa, chuteira direita...),
// com a regiao do corpo dele (close-up, isolamento, checagens).
struct MeshPart {
    std::string name;
    std::string region;
    // As partes com que esta forma uma superficie so: as duas dividem arestas
    // (a costura, vertice com vertice). A analise cobra a costura.
    std::vector<std::string> connectedTo;
    // As bordas abertas de proposito (a cintura, ate o tronco existir): cada
    // nome aceita uma volta de borda nesta parte.
    std::vector<std::string> openings;
    // Raiz: toda peca precisa chegar a uma raiz pela superficie; a que nao
    // chega e peca solta.
    bool root = false;

    [[nodiscard]] bool connectedWith(const MeshPart& other) const;
};

struct MeshMaterial {
    std::string name;
    Color color = Palette::Surface;
    // A textura dele (indice em Mesh::textures) ou -1: a cor chapada.
    int texture = -1;
};

struct MeshTexture {
    std::string name;
    Image image;
};

// A malha do personagem no espaco do modelo, na pose de ligacao. Triangulos
// anti-horarios vistos de fora; cada um com parte e material.
struct Mesh {
    std::vector<Vec3> positions;
    // Vazio, ou um por vertice.
    std::vector<SkinWeights> skin;
    std::vector<Triangle> triangles;
    std::vector<std::uint16_t> trianglePart;
    std::vector<std::uint16_t> triangleMaterial;
    std::vector<MeshPart> parts;
    std::vector<MeshMaterial> materials;
    // O mapa de textura: vazio, ou os tres cantos de cada triangulo (a
    // costura de UV fica livre: dois triangulos dividem o vertice e nao a
    // coordenada). Entra por ultimo, com a geometria pronta: os cortes e
    // costuras nao o carregam (so compactar, espelhar e tirar parte).
    std::vector<std::array<Uv, 3>> triangleUv;
    std::vector<MeshTexture> textures;

    [[nodiscard]] bool empty() const { return triangles.empty(); }
    [[nodiscard]] bool hasSkin() const { return !skin.empty() && skin.size() == positions.size(); }
    [[nodiscard]] bool hasUv() const { return !triangleUv.empty() && triangleUv.size() == triangles.size(); }
    [[nodiscard]] Box3 bounds() const;
    [[nodiscard]] Vec3 normal(std::size_t triangle) const;
    [[nodiscard]] float area(std::size_t triangle) const;
    [[nodiscard]] Vec3 center(std::size_t triangle) const;
    [[nodiscard]] int findPart(std::string_view name) const;
    [[nodiscard]] int findMaterial(std::string_view name) const;
    [[nodiscard]] int partOf(std::size_t triangle) const;
    [[nodiscard]] Box3 partBounds(int part) const;
    [[nodiscard]] std::vector<std::uint32_t> partVertices(int part) const;
    [[nodiscard]] std::vector<std::uint32_t> partTriangles(int part) const;
    [[nodiscard]] std::size_t partTriangleCount(int part) const;
    // Volume com sinal: positivo numa malha fechada orientada para fora.
    [[nodiscard]] float signedVolume() const;
    [[nodiscard]] float surfaceArea() const;
    // As posicoes deformadas pelo skin linear (uma transformacao por junta).
    [[nodiscard]] std::vector<Vec3> posedPositions(std::span<const Transform> skinning) const;
    // Tamanhos e indices consistentes.
    [[nodiscard]] std::vector<std::string> validateLayout() const;
};

} // namespace MatterEngine::Modeling
