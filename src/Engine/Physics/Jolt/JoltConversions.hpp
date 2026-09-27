#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <Jolt/Jolt.h>

#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>

#include <type_traits>

namespace MatterEngine {

// Conversoes entre a matematica neutra da engine e a do Jolt.
//
// Sao copia de componente, sem troca de eixo: a engine e Z-up destro e o Jolt
// nao impoe convencao de eixo ao solver. As excecoes ficam todas nos auxiliares
// de personagem (CharacterVirtual assume "up" = +Y por PADRAO) e sao tratadas
// explicitamente onde aparecem, nao aqui.
//
// Header-only de proposito: isto e o tipo de funcao que precisa desaparecer no
// inline em build otimizada.

// O backend assume precisao simples, configurada em cmake/PhysicsBackend.cmake
// (DOUBLE_PRECISION OFF). Com precisao dupla, JPH::RVec3 deixa de ser JPH::Vec3
// e as posicoes passariam a truncar silenciosamente - este assert transforma
// isso em erro de compilacao no lugar certo.
static_assert(std::is_same_v<JPH::RVec3, JPH::Vec3>,
    "O backend Jolt do MatterEngine assume DOUBLE_PRECISION OFF");

[[nodiscard]] inline JPH::Vec3 toJolt(Vec3 value) {
    return JPH::Vec3(value.x, value.y, value.z);
}

[[nodiscard]] inline Vec3 fromJolt(JPH::Vec3Arg value) {
    return { value.GetX(), value.GetY(), value.GetZ() };
}

[[nodiscard]] inline JPH::Quat toJolt(Quaternion value) {
    // O Jolt assume quaternion unitario nas rotacoes; a engine nao garante isso
    // em todo caminho de entrada (pose autoral interpolada, por exemplo).
    const Quaternion unit = value.normalized();
    return JPH::Quat(unit.x, unit.y, unit.z, unit.w);
}

[[nodiscard]] inline Quaternion fromJolt(JPH::QuatArg value) {
    return { value.GetX(), value.GetY(), value.GetZ(), value.GetW() };
}

} // namespace MatterEngine
