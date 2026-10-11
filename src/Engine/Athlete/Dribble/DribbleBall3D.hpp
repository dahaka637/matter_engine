#pragma once

#include "Engine/Athlete/Ball/BallOutcome3D.hpp"
#include "Engine/Athlete/Sensing/BallPredictor3D.hpp"
#include "Engine/Math/Vec3.hpp"

#include <vector>

namespace MatterEngine {

// A bola no futuro (plano v5, o manejo refeito): o previsor calibrado
// (BallPredictor3D) numa tabela. O caminho pode comecar mais tarde - a bola
// que sai de um toque ainda planejado (o encadeamento: o toque seguinte e
// planejado na bola que o anterior vai mandar).
class DribbleBallPath3D {
public:
    // A bola em `position`/`velocity` no instante `start` (s a partir de
    // agora), ate `start + horizon`.
    void build(Vec3 position, Vec3 velocity, float groundHeight, float radius, float start, float horizon,
        const BallPredictorSettings3D& settings = {});
    [[nodiscard]] bool valid() const { return !m_positions.empty(); }
    [[nodiscard]] float start() const { return m_start; }
    // No instante t (s a partir de agora; antes do comeco, o comeco).
    [[nodiscard]] Vec3 position(float t) const;
    [[nodiscard]] Vec3 velocity(float t) const;

private:
    std::vector<Vec3> m_positions;
    std::vector<Vec3> m_velocities;
    float m_start = 0.0f;
    float m_step = 1.0f / 60.0f;
};

// A velocidade da bola depois de um toque (o modelo aprovado pelo usuario).
[[nodiscard]] Vec3 dribbleTouchVelocity3D(const BallOutcomeRequest3D& intent, Vec3 ballVelocity, Vec3 bodyVelocity,
    Vec3 aheadVelocity, float ballRadius, const BallOutcomeSettings3D& settings);

} // namespace MatterEngine
