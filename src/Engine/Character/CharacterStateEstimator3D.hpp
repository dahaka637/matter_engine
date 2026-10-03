#pragma once

#include "Engine/Character/CharacterControlTypes3D.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace MatterEngine {

// Quanto um ponto (no plano) esta FORA do fecho convexo dos pontos dados (m;
// 0 dentro). Com 1-2 pontos, a distancia ao ponto/segmento; sem pontos, 10.
float planarDistanceOutsideHull3D(std::vector<std::array<float, 2>> points,
    float x, float y);

// Estado fisico comum de um personagem, calculado uma vez por tick a partir do
// que o backend publicou no tick concluido (poses, velocidades, contatos) e
// das sondas de chao sob os pes. So mede: nao decide pose, passo nem forca.
// Todos os consumidores (locomocao, assistencia, planejador) leem daqui.
class CharacterStateEstimator3D {
public:
    void reset(const RagdollProfile3D& profile);
    // `footGround`: sonda do chao sob cada pe (sem superficie = sem evidencia
    // geometrica).
    const CharacterPhysicalState3D& update(const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const std::array<GroundProbeResult3D, 2>& footGround, float deltaTime);
    [[nodiscard]] const CharacterPhysicalState3D& state() const { return m_state; }

private:
    CharacterPhysicalState3D m_state;
    std::size_t m_linkCount = 0;
    std::array<std::size_t, 2> m_feet {};
    std::vector<bool> m_legLink;
    Vec3 m_externalForce;
    // Aceleracao observada do COM: velocidade do tick anterior e o filtro.
    Vec3 m_previousComVelocity;
    Vec3 m_filteredComAcceleration;
    bool m_previousComVelocityValid = false;
    // Por pe: ha quanto tempo teve contato de apoio (s) e a evidencia dele.
    std::array<float, 2> m_sinceContactSupport { 10.0f, 10.0f };
    std::array<float, 2> m_supportingSeconds {};
    std::array<ContactEvidence3D, 2> m_lastContactEvidence {};
    bool m_initialized = false;
};

} // namespace MatterEngine
