#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace MatterEngine {

// O equilibrio do corpo de agora (04/11 15h, os passos de recuperacao - vault
// 04_locomocao/plano_passos_de_recuperacao_04_11.md): a base dos pes no chao, o ponto de captura (para onde o centro de
// massa vai: CoM + v/omega, o pendulo invertido linear) e a margem dele ate a borda da base. E o "predict" do
// autoequilibrio: com o ponto de captura dentro da base o corpo para em cima dos pes; fora, so um passo o segura.
struct TrackingBalance3D {
    Vec3 centerOfMass;
    Vec3 velocity;
    // sqrt(g / altura do centro de massa ao chao), 1/s.
    float omega = 3.13f;
    // No plano do chao (z = o chao).
    Vec3 capture;
    std::array<bool, 2> footDown {};
    // A parte da carga em cada pe (dos impulsos normais dos contatos; sem impulso, igual entre os pes no chao).
    std::array<float, 2> load { 0.5f, 0.5f };
    // O centro da sola de cada pe (no chao) e o centro da base (dos pes no chao; nenhum no chao: dos dois).
    std::array<Vec3, 2> sole {};
    std::array<std::array<Vec3, 4>, 2> corners {};
    Vec3 supportCenter;
    // A distancia com sinal do ponto de captura ate a borda da base (m; + dentro).
    float margin = 0.0f;
    // O momento angular em volta do centro de massa (a parte das velocidades lineares das partes), por massa x 1 m.
    Vec3 spin;
};

// footLinks: o esquerdo e o direito. Um pe esta no chao se toca algo por baixo (normal com z > 0,5).
[[nodiscard]] TrackingBalance3D measureTrackingBalance3D(const RagdollProfile3D& profile,
    std::span<const PhysicsBodyState3D> links, std::span<const RagdollInteraction3D> contacts,
    std::array<std::uint32_t, 2> footLinks, float groundHeight);

// Os quatro cantos da sola de um pe no mundo (z do chao), da caixa do colisor (sem caixa: 26 x 10 cm em volta do link).
void trackingSoleCorners3D(const RagdollLinkDefinition3D& link, const PhysicsBodyState3D& state, float groundHeight,
    std::span<Vec3, 4> out);

// A distancia com sinal de q ao fecho convexo dos pontos, no plano (+ dentro; 1 ou 2 pontos: menos a distancia).
[[nodiscard]] float trackingSignedHullDistance3D(std::span<const Vec3> points, Vec3 q);

} // namespace MatterEngine
