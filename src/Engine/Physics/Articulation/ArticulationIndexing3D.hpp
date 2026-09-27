#pragma once

#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace MatterEngine {

// Numeracao dos graus de liberdade generalizados de uma articulacao flutuante,
// derivada somente do perfil - sem SDK, sem cena, sem estado.
//
// Existe porque a numeracao deixou de ser um artefato do backend. O PhysX
// atribuia indices de DOF internamente (a articulacao dele e simulada em
// coordenadas reduzidas); o ragdoll do Jolt e corpos + constraints e nao tem
// nocao de DOF generalizado nenhuma. Sendo propriedade do perfil, a numeracao
// passa a ser igual nos dois, verificavel sem instanciar nada.
//
// Convencao, nesta ordem e sem excecao:
//   [0..5]  os seis DOFs livres da raiz (linear XYZ, angular XYZ);
//   [6..]   os eixos habilitados de cada junta, na ordem dos links do perfil
//           (pais antes de filhos, garantido pelo contrato do perfil) e, dentro
//           de cada link, na ordem Twist, Swing1, Swing2.
//
// Eixos desabilitados nao consomem indice: um perfil com uma dobradica de um
// eixo so gasta um DOF, nao tres.
struct ArticulationIndexing3D {
    static constexpr std::uint32_t InvalidIndex =
        std::numeric_limits<std::uint32_t>::max();
    // A raiz flutuante e sempre contada, mesmo quando o guia de animacao a
    // carrega: ela e seis DOFs do sistema, e omiti-los deslocaria todo o resto.
    static constexpr std::uint32_t RootDofCount = 6u;

    // Sem a raiz.
    std::uint32_t jointDofCount = 0;
    // Com a raiz: RootDofCount + jointDofCount.
    std::uint32_t generalizedDofCount = RootDofCount;
    // Mesmo indice dos links do perfil. Eixo desabilitado, e a propria raiz,
    // permanecem InvalidIndex.
    std::vector<std::array<std::uint32_t, 3>> jointGeneralizedDof;
};

[[nodiscard]] ArticulationIndexing3D buildArticulationIndexing3D(
    const RagdollProfile3D& profile);

} // namespace MatterEngine
