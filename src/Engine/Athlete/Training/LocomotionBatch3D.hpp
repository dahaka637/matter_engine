#pragma once

#include "Engine/Athlete/Training/LocomotionWorld3D.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace MatterEngine {

// A cena de um mundo do lote: inline, uma thread, com as reservas de um
// atleta e o chao (a padrao reserva 16 mil corpos: 512 mundos nao caberiam na
// memoria). As reservas nao mudam a simulacao (a paridade confere).
[[nodiscard]] PhysicsSceneSettings3D locomotionBatchScene3D();

struct LocomotionBatchSettings3D {
    std::uint32_t worlds = 64;
    // As threads (0: as do hardware menos duas); a que chama step() trabalha
    // tambem.
    std::uint32_t threads = 0;
    // Os ajustes de cada mundo (a cena vira a do lote).
    LocomotionWorldSettings3D world;
};

// O lote da L0 do RL: muitos mundos de treino avancados em paralelo, cada um
// com a sua cena e o seu atleta comandado, cada mundo numa thread por vez. O
// chamador escreve o comando de cada mundo (commands()) e as correcoes do RL
// no atleta dele (world(i).athlete().corrections) e chama step().
class LocomotionBatch3D final {
public:
    LocomotionBatch3D(const AthleteRuntimeAssets3D& assets, const LocomotionBatchSettings3D& settings);
    ~LocomotionBatch3D();
    LocomotionBatch3D(const LocomotionBatch3D&) = delete;
    LocomotionBatch3D& operator=(const LocomotionBatch3D&) = delete;

    [[nodiscard]] std::size_t size() const;
    [[nodiscard]] std::uint32_t threads() const;
    [[nodiscard]] LocomotionWorld3D& world(std::size_t index);
    [[nodiscard]] std::span<AthleteCommand3D> commands();

    // Todos os mundos avancam `ticks` ticks com o comando de cada um (a
    // decisao do RL a 30 Hz sao 4 ticks). Um mundo cujo passo lancou excecao
    // para ali e fica marcado (failed); o chamador o recomeca.
    void step(int ticks = 1);
    [[nodiscard]] bool failed(std::size_t index) const;
    void clearFailure(std::size_t index);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
