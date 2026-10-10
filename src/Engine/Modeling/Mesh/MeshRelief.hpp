#pragma once

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshImprint.hpp"

#include <span>
#include <string>
#include <vector>

// Pecas em relevo sobre a superficie (a almofada da lingua, as tiras do
// cadarco): a pegada e recortada como furo e a peca sobe dela, parede e
// topo, vertice com vertice. Por baixo do topo e cheio: nao existe fresta
// nem sobreposicao.
namespace MatterEngine::Modeling {

struct ReliefPiece {
    // A pegada: volta anti-horaria vista de fora (de "viewDirection"); os
    // pontos descem ate a superficie.
    std::vector<Vec3> footprint;
    // O topo: uma malha aberta (triangulos anti-horarios vistos de fora)
    // cuja borda e "rim" (indices em topPositions, anti-horaria vista de fora).
    std::vector<Vec3> topPositions;
    std::vector<Triangle> topTriangles;
    std::vector<std::uint32_t> rim;
    // Para cada ponto da pegada, a posicao em "rim" que fica acima dele (em
    // ordem): a parede de cada trecho sobe ate o trecho certo do topo. Vazio
    // = um para um (rim do tamanho da pegada).
    std::vector<std::uint32_t> footprintToRim;
};

// O topo de uma tira: duas fileiras (a direita e a esquerda de quem anda
// pelo caminho) costuradas em quads; a borda volta pela esquerda.
void stripTop(ReliefPiece& piece, std::span<const Vec3> right, std::span<const Vec3> left);

// Recorta as pegadas nos triangulos "host" (todas de uma vez: elas nao se
// sobrepoem) e ergue cada peca na parte/material correntes do builder.
// Devolve falso (com o erro) se algum recorte falhar.
bool addReliefs(MeshBuilder& builder, std::span<const std::uint32_t> host, std::span<const ReliefPiece> pieces,
    const ImprintSettings& settings, std::string* error = nullptr);

// Divide uma volta recortada (ImprintResult::loops) nos trechos entre os
// pontos dados (ImprintResult::inputVertices): trecho i vai do ponto i ao
// i+1, com os cortes do meio.
[[nodiscard]] std::vector<std::vector<std::uint32_t>> loopSegments(std::span<const std::uint32_t> loop,
    std::span<const std::uint32_t> inputs);

} // namespace MatterEngine::Modeling
