#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <optional>
#include <span>
#include <string>
#include <vector>

// Recorte de pegadas na superficie: a ligacao de verdade de uma peca que
// sai de outra (a trava da sola, a almofada da lingua, a tira do cadarco) e
// o desenho de uma regiao com outro material (o ilhos escuro, o numero do
// calcao). As arestas das voltas sao cortadas nos triangulos de baixo: a
// superficie continua exatamente a mesma (as facetas nao mudam), so ganha
// as linhas novas. Nada fica por cima, nada fica solto.
namespace MatterEngine::Modeling {

// Uma volta sobre a superficie (pontos 3D em ordem, sem repetir o primeiro).
struct ImprintLoop {
    std::vector<Vec3> points;
    // Furo: a peca continua da volta (a volta devolvida). Regiao: os
    // triangulos de dentro ganham o material/parte dela, sem furo.
    bool hole = true;
    int material = -1;
    int part = -1;
};

struct ImprintSettings {
    // De onde se olha a superficie (a normal de fora dela): as voltas sao
    // projetadas ao longo dessa direcao.
    Vec3 viewDirection { 0.0f, 0.0f, 1.0f };
    // Ponto de volta mais perto que isso de um vertice da superficie vira o
    // proprio vertice (sem triangulos agulha).
    float snap = 0.00025f;
};

struct ImprintResult {
    bool ok = false;
    std::string error;
    // Cada volta como ficou na malha (na ordem dada): os pontos dados mais
    // os cortes nas arestas de baixo, ja na superficie.
    std::vector<std::vector<std::uint32_t>> loops;
    // So os vertices dos pontos dados, na ordem.
    std::vector<std::vector<std::uint32_t>> inputVertices;
};

// Corta as voltas nos triangulos "host" (indices na malha; so os de frente
// para a vista contam). Os pontos sao levados ate a superficie ao longo da
// vista. Os triangulos cortados saem do lugar e os pedacos vao para o fim da
// lista (os indices de triangulo antigos deixam de valer); os vertices nao
// mudam de indice (os de dentro dos furos ficam soltos ate compactar). Os
// pesos de skin dos vertices novos vem do triangulo de baixo.
[[nodiscard]] ImprintResult imprintLoops(Mesh& mesh, std::span<const std::uint32_t> host,
    std::span<const ImprintLoop> loops, const ImprintSettings& settings = {});

// O ponto levado ate a superficie dos triangulos ao longo de "direction" (o
// mais perto, entre os que ficam debaixo dele), ou nada.
[[nodiscard]] std::optional<Vec3> projectOntoTriangles(const Mesh& mesh, std::span<const std::uint32_t> triangles,
    Vec3 point, Vec3 direction);

// Pesos misturados (as 4 juntas de maior soma, normalizados).
[[nodiscard]] SkinWeights blendSkin(std::span<const SkinWeights> weights, std::span<const float> factors);

} // namespace MatterEngine::Modeling
