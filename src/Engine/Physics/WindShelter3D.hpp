#pragma once

#include "Engine/Math/Vec3.hpp"

namespace MatterEngine {

class PhysicsScene3D;

// Fracao continua de exposicao ao vento numa posicao especifica: 0 =
// totalmente abrigado atras de geometria estatica a barlavento, 1 = vento
// livre. Mesma formula (raycast a barlavento + smoothstep pela distancia do
// impacto) ja usada pelo arrasto aerodinamico dos props dinamicos - ver
// PhysXScene3D::simulate, que a mantem inline em vez de chamar esta funcao
// (codigo quente, por corpo por passo de fisica, hoje sem cobertura de
// teste - convergir os dois fica pra depois de existir um teste de
// regressao pra aquele bloco).
//
// Vive em Engine/Physics (nao em Engine/Environment, onde mora o
// WindSystem) porque MatterPhysics nao pode depender de MatterEngine - a
// dependencia e o oposto (ver CMakeLists.txt) - entao este e o unico lugar
// que tanto o backend PhysX quanto os consumidores de audio/gameplay em
// Engine/Workbench conseguem alcancar.
[[nodiscard]] float windShelterExposure3D(const PhysicsScene3D& scene,
    Vec3 position, Vec3 windVelocity, float shelterDistanceMeters);

} // namespace MatterEngine
