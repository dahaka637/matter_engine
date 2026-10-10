#pragma once

#include "Engine/Math/Vec3.hpp"

#include <vector>

namespace MatterEngine {

// A previsao da bola (rework do manejo, 10/10/2026, Etapa 5): no chao, rolando
// com a resistencia (constante mais uma parte pela velocidade); no ar,
// balistica com arrasto, quicando com a restituicao. Barata: o planejador
// pede uma tabela a 1/30 s ate o horizonte. Ajustada contra a bola de verdade
// (a sonda mede o rolar no chao do laboratorio).
struct BallPredictorSettings3D {
    float rollingDeceleration = 0.45f;
    float rollingDragPerSpeed = 0.06f;
    float airDragPerSpeed = 0.012f;
    float gravity = 9.81f;
    float restitution = 0.55f;
    // Subpasso da integracao (s).
    float step = 1.0f / 120.0f;
};

struct BallPrediction3D {
    Vec3 position;
    Vec3 velocity;
    bool grounded = true;
};

// A bola daqui a `seconds` (s). groundHeight: o chao embaixo dela.
[[nodiscard]] BallPrediction3D predictBall3D(Vec3 position, Vec3 velocity, float groundHeight, float radius,
    float seconds, const BallPredictorSettings3D& settings);

// A tabela: a bola em 0, interval, 2*interval, ... ate horizon (s).
void predictBallTable3D(Vec3 position, Vec3 velocity, float groundHeight, float radius, float interval,
    float horizon, const BallPredictorSettings3D& settings, std::vector<BallPrediction3D>& table);

} // namespace MatterEngine
