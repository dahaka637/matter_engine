#pragma once

#include "Engine/Athlete/Dribble/DribbleTypes3D.hpp"
#include "Engine/Athlete/Dribble/TouchPlanner3D.hpp"

#include <cstdint>

namespace MatterEngine {

// O corpo bonito do toque (plano v5, E7): as chaves de corpo do V3 (armado,
// contato e acompanhamento por superficie e jeito, a forca dando o tamanho)
// no tempo do balanco - o armado em 40% da ida, o contato no contato, o
// acompanhamento em 40% da volta, zero no comeco e bodyFade depois da volta.
// Cada toque varia cada canal ate +-20% e os punhos ate +-5 cm; correndo, o
// corpo do gesto pesa pouco (a corrida manda).
struct TouchBodyKey3D {
    float pelvisDrop = 0.0f, pelvisYaw = 0.0f, pelvisPitch = 0.0f, pelvisRoll = 0.0f, pelvisToSupport = 0.0f;
    float trunkTwist = 0.0f, trunkBend = 0.0f, trunkSide = 0.0f;
    // Punhos em relacao ao ombro (m): frente (no rumo do corpo), para fora,
    // para cima; o do lado que toca e o outro.
    Vec3 kickArm;
    Vec3 supportArm;
    float arms = 0.0f;
    // A pelve para a frente do toque (m).
    float pelvisForward = 0.0f;
};

struct TouchBodyPlan3D {
    bool valid = false;
    int side = 0;
    TouchBodyKey3D windup;
    TouchBodyKey3D contact;
    TouchBodyKey3D follow;
    // Os tempos das chaves (relogio): zero, armado, contato, acompanhamento e
    // zero de novo.
    float times[5] {};
    Vec3 push { 1.0f, 0.0f, 0.0f };
    bool moving = false;
    bool shot = false;
};

// O corpo de um toque planejado (seed: o jeito deste toque).
[[nodiscard]] TouchBodyPlan3D touchBodyPlan3D(const TouchPlan3D& plan, bool moving, std::uint32_t seed,
    float fadeSeconds);

// A pose do corpo agora (relogio), por cima do que out ja tem: share e o peso
// do gesto (1 parado, pouco correndo), forward o rumo do corpo e o caminho da
// pelve ate o tornozelo de apoio. Devolve o peso dos bracos.
float touchBodyPose3D(const TouchBodyPlan3D& body, float now, float share, Vec3 forward, Vec3 toSupport,
    DribbleBody3D& out);

} // namespace MatterEngine
