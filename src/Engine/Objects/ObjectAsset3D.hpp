#pragma once

#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsTypes3D.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

// Os objetos procedurais (Modeling/Objects) levados ao jogo: a malha da GPU
// com a textura, e o corpo fisico - os mesmos para o laboratorio, o inspetor
// de objetos e os testes.
namespace MatterEngine {

struct ObjectRenderData3D {
    // Um vertice por (canto, grupo de sombreamento, UV): a quina (mais de 45
    // graus) separa a normal, o resto e liso. Sem pesos (malha estatica).
    MeshData3D mesh;
    // A textura do objeto em RGBA8 (a pixel art ja ampliada); vazia se ele
    // for so de cores chapadas (a cor vai no vertice).
    std::vector<std::byte> rgba;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

[[nodiscard]] ObjectRenderData3D objectRenderData3D(const Modeling::ProceduralObject& object);

struct ObjectBodyData3D {
    // O corpo (a posicao e a do centro do objeto; quem cria poe a dele) e as
    // formas no espaco do objeto. O material desconhecido vira o "default".
    PhysicsBodyDefinition3D body;
    std::vector<PhysicsShape3D> shapes;
    Vec3 dimensions;
};

// O cilindro (a trave) ainda nao existe na fisica: vira uma capsula do mesmo
// comprimento.
[[nodiscard]] ObjectBodyData3D objectBodyData3D(const Modeling::ProceduralObject& object, const MaterialLibrary& materials);

} // namespace MatterEngine
