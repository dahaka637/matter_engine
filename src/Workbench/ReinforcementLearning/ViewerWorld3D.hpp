#pragma once

#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/RL/Environment/BodyTrainingBatch3D.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace MatterEngine {

// Um mundo que a tela de Aprendizado mostra (o da locomocao, o do rastreador):
// a rede exportada pelo treino, uma decisao por vez, a velocidade na mao, o
// empurrao no peito e a vista que a tela desenha (BodyWorldView3D).
class ViewerWorld3D {
public:
    virtual ~ViewerWorld3D() = default;

    // A rede (o texto diz qual e, ou o erro).
    virtual void setPolicy(const std::filesystem::path& file, std::string& label) = 0;
    virtual void clearPolicy() = 0;
    [[nodiscard]] virtual bool hasPolicy() const = 0;

    // Uma decisao (30 Hz: 4 ticks de fisica).
    virtual void step() = 0;
    [[nodiscard]] virtual float policyStepSeconds() const = 0;
    virtual void reset(std::uint64_t seed) = 0;
    // Velocidade manual para a frente (nulo devolve o piloto do treino).
    virtual void setManualSpeed(std::optional<float> metersPerSecond) = 0;
    // Um empurrao no peito (N), por alguns decimos de segundo.
    virtual void push(Vec3 force, float seconds) = 0;

    [[nodiscard]] virtual BodyWorldView3D view() const = 0;
    [[nodiscard]] virtual const RagdollProfile3D& profile() const = 0;
};

} // namespace MatterEngine
