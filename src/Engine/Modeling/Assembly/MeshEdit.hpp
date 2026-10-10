#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Mesh/MeshCut.hpp"

#include <cstdint>
#include <span>
#include <vector>

// Edicoes de malha inteira usadas na montagem: mover e escalar, ficar com a
// metade direita, juntar uma malha na outra e o plano inclinado dos cortes.
namespace MatterEngine::Modeling {

// p' = p * scale + offset (todos os vertices).
void transformPositions(Mesh& mesh, float scale, Vec3 offset);

// Fica so a metade direita (y <= 0): o corte no plano do meio. A borda do
// meio (y = 0 exato) e a que o espelho solda depois.
[[nodiscard]] CutResult keepRightHalf(Mesh& mesh);

// Junta "from" em "into": vertices (com skin, se os dois tiverem),
// triangulos, partes e materiais (pelo nome). Devolve o primeiro vertice
// novo (os de "from" vem em ordem a partir dele).
std::uint32_t appendMesh(Mesh& into, const Mesh& from);

// O plano de um corte de pescoco ou de tornozelo: passa por (centerX, 0,
// height) e sobe "tiltDegrees" para tras (a frente mais baixa); a normal
// aponta para o lado que fica (para cima ou para baixo).
[[nodiscard]] CutPlane tiltedPlane(float height, float tiltDegrees, float centerX, bool keepAbove);

// Os vertices usados pelos triangulos das partes dadas.
[[nodiscard]] std::vector<std::uint32_t> partsVertices(const Mesh& mesh, std::span<const int> parts);

} // namespace MatterEngine::Modeling
