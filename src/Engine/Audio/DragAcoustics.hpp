#pragma once

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsTypes3D.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// Comando continuo (nao um disparo unico como ImpactSoundCommand3D) - o
// chamador decide como transformar "intensity" numa voz em loop (ver
// WorldAudioController), incluindo iniciar/parar o loop conforme o corpo
// aparece ou nao nas resolucoes seguintes.
struct DragSoundCommand3D {
    std::uint64_t sourceBodyId = 0;
    std::string materialId;
    std::string soundSet;
    Vec3 position;
    // 0..1, ja combinando forca normal (peso) e velocidade tangencial
    // (deslizamento) - ver DragAcousticResolver::resolve.
    float intensity = 0.0f;
};

// Resolve arrasto/atrito a partir de contatos ainda tocando
// (ContactSlideEvent3D, ver PhysicsTypes3D.hpp) - o equivalente continuo de
// ImpactAcousticResolver para eventos de impacto discretos. So o lado
// DINAMICO de cada contato emite (uma superficie estatica nao "arrasta"
// sozinha) - os exemplos que motivaram esta classe (bigorna pesada vs. bola
// leve) sao inteiramente sobre o peso do objeto arrastado, nao uma mistura
// dos dois materiais em contato.
class DragAcousticResolver final {
public:
    void resolve(std::span<const ContactSlideEvent3D> slides,
        const MaterialLibrary& materials, float deltaTime);

    [[nodiscard]] std::span<const DragSoundCommand3D> commands() const {
        return m_commands;
    }

private:
    std::vector<DragSoundCommand3D> m_commands;
};

} // namespace MatterEngine
