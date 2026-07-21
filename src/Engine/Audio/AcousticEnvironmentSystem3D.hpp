#pragma once

#include "Engine/Audio/AcousticZone3D.hpp"

#include <array>
#include <vector>

namespace MatterEngine {

// Resultado puro (sem OpenAL) de avaliar as zonas contra a posicao do
// ouvinte: no maximo 2 zonas ativas de cada vez, porque o contexto OpenAL
// e criado com ALC_MAX_AUXILIARY_SENDS=2 (ver AudioDevice3D::initialize).
struct AcousticEnvironmentBlend3D {
    struct Slot {
        bool active = false;
        AcousticReverbPreset3D preset = AcousticReverbPreset3D::Generic;
        // Ganho final ja considerando a mistura entre zonas sobrepostas e
        // o wetSendGain autoral de cada uma; 0 quando o slot esta inativo.
        float wetSendGain = 0.0f;
    };
    std::array<Slot, 2> slots {};
};

// Logica de mistura entre zonas acusticas, isolada do backend de audio para
// poder ser testada sem nenhum dispositivo OpenAL. Sem nenhuma nocao de
// "Laboratorio": qualquer tela com ouvinte e zonas pode reusar.
class AcousticEnvironmentSystem3D final {
public:
    void setZones(std::vector<AcousticZoneDefinition3D> zones) {
        m_zones = std::move(zones);
    }

    [[nodiscard]] AcousticEnvironmentBlend3D evaluate(
        Vec3 listenerPosition) const;

private:
    std::vector<AcousticZoneDefinition3D> m_zones;
};

} // namespace MatterEngine
