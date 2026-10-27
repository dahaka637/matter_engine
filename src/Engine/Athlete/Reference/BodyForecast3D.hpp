#pragma once

#include "Engine/Math/Vec3.hpp"

#include <array>
#include <vector>

namespace MatterEngine {

// Onde o corpo vai estar (plano v5, o manejo refeito): a capsula do modo
// padrao no futuro, pela mesma regra dela (o controle de personagem, com o giro): o
// rumo da velocidade gira ate o pedido a turnRate, o modulo vai ao pedido a
// acceleration/deceleration, e pedindo o contrario (alem de reverseRadians)
// ela freia a brake antes de virar; sem pedido, desacelera. A pelve anda com
// ela; o rumo do corpo (o olhar) segue a taxa de agora, sumindo.
struct BodyForecastInput3D {
    // A pelve no chao (x, y da pelve; z do chao) e a velocidade no plano.
    Vec3 pelvisGround;
    Vec3 velocity;
    // A velocidade pedida (no plano: a das teclas).
    Vec3 requested;
    // A resposta da capsula (m/s2, rad/s, rad).
    float acceleration = 16.0f;
    float deceleration = 22.0f;
    float brake = 30.0f;
    float turnRate = 10.0f;
    float reverseRadians = 2.0944f;
    // O rumo do corpo (rad) e quanto ele gira (rad/s; some em yawRateSeconds).
    float facingYaw = 0.0f;
    float facingRate = 0.0f;
    float facingRateSeconds = 0.35f;
    // As teclas sao relativas ao olhar (o laboratorio): o pedido gira com ele.
    bool requestedTurnsWithFacing = true;
    // Os quadris no referencial do corpo (a partir da pelve no chao: x frente,
    // y esquerda, z cima).
    std::array<Vec3, 2> hipInBody {};
    // A tabela: passo e horizonte (s).
    float step = 1.0f / 60.0f;
    float horizon = 1.0f;
};

// Um passo da capsula no chao pela regra acima (a velocidade no plano de agora
// e a pedida; a resposta sai de `response`): a previsao e o lider do
// rastreador (Engine/Tracking) usam a mesma.
[[nodiscard]] Vec3 bodyForecastCapsuleStep3D(Vec3 current, Vec3 desired, const BodyForecastInput3D& response, float dt);

class BodyForecast3D {
public:
    void build(const BodyForecastInput3D& input);
    [[nodiscard]] bool valid() const { return !m_samples.empty(); }
    // No instante t (s a partir de agora; preso no horizonte).
    [[nodiscard]] Vec3 pelvisGround(float t) const;
    [[nodiscard]] Vec3 velocity(float t) const;
    [[nodiscard]] float yaw(float t) const;
    [[nodiscard]] Vec3 hip(int side, float t) const;
    // Do referencial do corpo no instante t ao mundo e de volta (pontos).
    [[nodiscard]] Vec3 toWorld(Vec3 body, float t) const;
    [[nodiscard]] Vec3 toBody(Vec3 world, float t) const;
    // Direcoes (sem a translacao).
    [[nodiscard]] Vec3 directionToWorld(Vec3 body, float t) const;
    [[nodiscard]] Vec3 directionToBody(Vec3 world, float t) const;

private:
    struct Sample {
        Vec3 position;
        Vec3 velocity;
        float yaw = 0.0f;
    };
    [[nodiscard]] Sample at(float t) const;

    std::vector<Sample> m_samples;
    float m_step = 1.0f / 60.0f;
    std::array<Vec3, 2> m_hipInBody {};
};

} // namespace MatterEngine
