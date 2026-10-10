#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace MatterEngine {

// O apoio medido de um pe: so o que o corpo toca conta (nunca o plano).
struct FootObservation3D {
    std::uint32_t link = 0;
    bool valid = false;
    // Contato com algo por baixo (normal para cima) neste passo.
    bool supporting = false;
    // A carga normal aproximada (N): o impulso normal do contato no passo
    // sobre o dt. No Jolt e estimativa (antes do solver).
    float loadNewtons = 0.0f;
    // O centro da sola e a direcao para onde o pe aponta (no plano), no mundo.
    Vec3 soleCenter;
    Vec3 forward { 1.0f, 0.0f, 0.0f };
    // A sola mais baixa (m).
    float soleHeight = 0.0f;
};

// O que o controle le do corpo num tick (o estado publicado no passo
// anterior): centro de massa, pelve, pes e pancadas de fora. So mede.
struct BodyObservation3D {
    bool valid = false;
    float massKilograms = 0.0f;
    Vec3 centerOfMass;
    Vec3 centerOfMassVelocity;
    Vec3 pelvisPosition;
    Quaternion pelvisOrientation;
    Vec3 pelvisVelocity;
    Vec3 pelvisAngularVelocity;
    // O "para cima" da pelve e do peito no mundo (z = 1 em pe).
    Vec3 pelvisUp { 0.0f, 0.0f, 1.0f };
    Vec3 chestUp { 0.0f, 0.0f, 1.0f };
    std::array<FootObservation3D, 2> feet {};
    int supportCount = 0;
    // Pancada deste passo: o impulso normal dos contatos com o que nao e
    // chao (outro corpo, objeto, parede), somado (N.s), e o link mais atingido.
    float externalImpulse = 0.0f;
    std::uint32_t strongestExternalLink = 0;
    float strongestExternalImpulse = 0.0f;
    // So o que se move (outro boneco, objeto), sem paredes.
    float movingImpulse = 0.0f;
};

// As constantes do perfil que a observacao usa (massas, pes, peito).
class BodyObserver3D {
public:
    void prepare(const RagdollProfile3D& profile);
    [[nodiscard]] bool ready() const { return !m_mass.empty(); }
    [[nodiscard]] BodyObservation3D observe(const RagdollProfile3D& profile, const RagdollState3D& state,
        float deltaTime) const;
    [[nodiscard]] float linkMass(std::size_t link) const { return link < m_mass.size() ? m_mass[link] : 0.0f; }
    [[nodiscard]] std::uint32_t footLink(int side) const { return m_feet[side == 0 ? 0 : 1]; }
    [[nodiscard]] std::uint32_t chestLink() const { return m_chest; }

private:
    std::vector<float> m_mass;
    std::array<std::uint32_t, 2> m_feet {};
    std::uint32_t m_chest = 0;
    float m_totalMass = 0.0f;
};

} // namespace MatterEngine
