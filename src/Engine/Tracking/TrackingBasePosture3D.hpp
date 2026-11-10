#pragma once

#include "Engine/Physics/PhysicsTypes3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace MatterEngine {

// A postura base (10/11, ele: "quando parado, sempre o IDLE alerta e o alvo... ao frear ele fica num equilibrio amador
// desnecessario em vez de recobrar a pose do idle alerta assim que possivel, com o minimo de passos"; "manter a postura nao
// pode ser empecilho para o auto-equilibrio, mas algo benefico"). A distancia do corpo ao idle alerta - a base (os pes em
// relacao a pelve, no quadro da pelve de cada um), o giro da pelve e do peito, a inclinacao do peito e a altura -, numa regua
// que nao satura longe dele: o atrator para onde o corpo parado sempre volta, na freada e nos empurroes, sem troca de modo.
struct TrackingBasePosture3D {
    float baseMeters = 0.0f; // a media dos dois pes, no plano
    float pelvisYaw = 0.0f;  // rad
    float chestYaw = 0.0f;   // rad
    float chestTilt = 0.0f;  // rad
    float drop = 0.0f;       // m, a pelve abaixo da do idle (so para baixo)

    // As larguras da regua: 8 cm de base, 12 graus de giro, 8 graus de inclinacao, 4 cm de altura.
    static constexpr float BaseWidth = 0.08f, YawWidth = 0.2094f, TiltWidth = 0.1396f, DropWidth = 0.04f;
    [[nodiscard]] float distance() const;
    // Estar perto: 1 no idle, 1/(1 + d^2) longe - ainda com inclinacao a 25 cm (o nucleo exponencial dava ~0 alem de 15 cm).
    [[nodiscard]] float attraction() const;
};

// O corpo contra o idle (os dois no mundo; o idle ja posto no rumo da vista): a pelve e os pes pelos links do perfil.
[[nodiscard]] TrackingBasePosture3D measureTrackingBasePosture3D(const RagdollProfile3D& profile,
    std::span<const PhysicsBodyState3D> body, std::span<const PhysicsBodyState3D> idle, std::uint32_t pelvis,
    std::uint32_t chest, std::array<std::uint32_t, 2> feet);

} // namespace MatterEngine
