#pragma once

#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Math/Mat4.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"
#include "Engine/Physics/Net/NetSolver3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsTypes3D.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
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

// A rede desenhada na GPU: a malha de verdade como pele dos nos da fisica.
// Um tubo fino de 3 lados por fio, no repouso da rede no mundo (o objeto na
// pose dada), cada ponta com os pesos do no desenhado dela nos nos da fisica.
// No esqueleto, a matriz de cada no e a translacao da posicao de agora menos
// o repouso (objectNetSkin3D): com os pesos somando 1, o fio cai onde
// drawnNetKnots o poe, no triangulo de colisao (fora da bola e dos membros).
struct ObjectNetRenderData3D {
    MeshData3D mesh;
    // Os nos da fisica no repouso gerado (mundo): a referencia do esqueleto
    // (nao a rede assentada).
    std::vector<Vec3> restKnots;
};

[[nodiscard]] ObjectNetRenderData3D objectNetRenderData3D(const Modeling::ProceduralObject& object, Vec3 position,
    Quaternion orientation);
void objectNetSkin3D(std::span<const Vec3> knots, std::span<const Vec3> restKnots, std::vector<Mat4>& out);

// A rede do objeto (a da trave) no mundo, com o objeto na pose dada: os nos, os
// fios (a rigidez e o peso de cada um), os presos, e os tubos da propria
// estrutura (os cilindros e capsulas dele) como as pecas que ela nao
// atravessa. O chao fica em groundHeight. Vazio se o objeto nao tem rede.
[[nodiscard]] NetDescription3D objectNetDescription3D(const Modeling::ProceduralObject& object, Vec3 position,
    Quaternion orientation, float groundHeight = 0.0f);

// A barreira dos jogadores da rede: um corpo estatico por papel (as placas do
// corpo e da capsula, ObjectNet::barrier; as so da capsula,
// capsuleBarrier), no espaco do objeto (quem cria poe a pose do objeto).
// Vazio se o objeto nao tem rede.
[[nodiscard]] std::vector<ObjectBodyData3D> objectNetBarrier3D(const Modeling::ProceduralObject& object);

} // namespace MatterEngine
