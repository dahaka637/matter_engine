#pragma once

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsTypes3D.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace MatterEngine {

enum class ImpactIntensity3D : std::uint8_t {
    Soft,
    Medium,
    Hard
};

// Comando backend-neutral produzido pela analise acustica. O catalogo de
// clips e o AudioDevice3D decidem qual WAV corresponde ao soundSet; portanto
// os testes da resposta fisica nao dependem de SDL nem de arquivos externos.
struct ImpactSoundCommand3D {
    std::uint64_t sourceBodyId = 0;
    std::string materialId;
    std::string soundSet;
    Vec3 position;
    ImpactIntensity3D intensity = ImpactIntensity3D::Soft;
    float volume = 0.0f;
    float pitch = 1.0f;
    float muffle = 0.0f;
    float priority = 0.0f;
    float emittedEnergyJoules = 0.0f;
    float durationSeconds = 0.18f;
};

class ImpactAcousticResolver final {
public:
    // Resolve todos os contatos de um passo publico da fisica. A funcao
    // produz no maximo uma excitacao por corpo/par durante o cooldown, mas
    // permite que os dois corpos soem quando ambos realmente responderem.
    void resolve(std::span<const ContactImpactEvent3D> impacts,
        const MaterialLibrary& materials, float deltaTime);
    void reset();

    [[nodiscard]] std::span<const ImpactSoundCommand3D> commands() const {
        return m_commands;
    }

private:
    struct EmissionKey {
        std::uint64_t pairA = 0;
        std::uint64_t pairB = 0;
        std::uint64_t source = 0;

        bool operator==(const EmissionKey&) const = default;
    };

    struct EmissionKeyHash {
        [[nodiscard]] std::size_t operator()(
            const EmissionKey& key) const noexcept;
    };

    std::vector<ImpactSoundCommand3D> m_commands;
    std::unordered_map<EmissionKey, float, EmissionKeyHash>
        m_nextAllowedTime;
    float m_timeSeconds = 0.0f;
};

} // namespace MatterEngine
