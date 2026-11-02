#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace MatterEngine {

// A queda programada da locomocao fisica - a imitacao do Euphoria dela - para quem nao e ela (01/11 22h50, ele: "quando
// ele de fato cai, eu preferia aquele nosso ragdoll que nos temos na locomocao fisica, que faz os movimentos programados,
// tenta cobrir a cabeca e se encolher... fica meio mole... e ai o ato de levantar ocorresse apos isso"; "nos tentamos
// imitar o euphoria ragdoll com os movimentos que ele faz quando esta em queda... pois fica mais realista").
//
// O mesmo codigo e as mesmas constantes de CharacterLocomotion3D (o cabecalho interno dela): os reflexos (a cabeca e a
// coluna se protegem, o tronco gira para cair de lado, as pernas cedem - de frente ajoelha, de costas senta), o
// equilibrio no ar e o tonus da queda e do deitado. Sem o levantar dela: assentado,
// a vez e de quem levanta.
class ProtectiveFall3D {
public:
    explicit ProtectiveFall3D(const RagdollProfile3D& profile);

    // Uma queda nova (a variacao dos movimentos pela semente).
    void begin(std::uint32_t seed);
    // Um passo: os alvos de cada eixo a partir do corpo de agora (o chao na altura dada).
    void step(const RagdollState3D& state, float groundHeight, float deltaTime, std::vector<RagdollDriveTarget3D>& drives);
    // No chao com o tronco parado ha RestSeconds, ou no chao ha GroundLimitSeconds: a vez de levantar.
    [[nodiscard]] bool settled() const { return m_settled; }

    // A queda sem volta de quem esta de pe (o "committing" da locomocao fisica): a pelve alem de ~50 graus da vertical e
    // ainda tombando, ou alem de ~40 graus tombando rapido (tilt em rad, tiltRate em rad/s).
    [[nodiscard]] static bool committed(float tilt, float tiltRate);
    // A inclinacao da pelve (rad, da vertical).
    [[nodiscard]] static float pelvisTilt(const RagdollProfile3D& profile, const RagdollState3D& state);

private:
    // O tipo do link no tonus: 0 tronco e maos, 1 braco, antebraco, pescoco e cabeca (os reflexos), 2 perna.
    enum class Kind : std::uint8_t { Trunk, ReflexLimb, Leg };
    const RagdollProfile3D* m_profile = nullptr;
    std::array<std::size_t, 2> m_hands {};
    std::size_t m_chest = 0;
    std::vector<Kind> m_kind;
    std::vector<std::uint8_t> m_arm;
    std::vector<std::uint8_t> m_lyingTorso;
    std::vector<std::uint8_t> m_lyingArm;
    std::vector<std::uint8_t> m_body;
    std::vector<Vec3> m_coordinates;
    std::uint32_t m_seed = 0;
    float m_time = 0.0f;
    float m_reflex = 0.0f;
    float m_commit = 0.0f;
    float m_flail = 0.0f;
    Vec3 m_fallDirection { 1.0f, 0.0f, 0.0f };
    std::array<float, 2> m_catchSeconds {};
    float m_noGroundSeconds = 0.0f;
    bool m_grounded = false;
    float m_groundSeconds = 0.0f;
    float m_restSeconds = 0.0f;
    float m_settleTone = 0.0f;
    bool m_settled = false;
};

} // namespace MatterEngine
