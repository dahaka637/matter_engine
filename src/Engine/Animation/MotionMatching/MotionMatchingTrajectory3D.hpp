#pragma once

#include "Engine/Animation/MotionMatching/MotionMatchingDatabase3D.hpp"

#include <deque>
#include <span>
#include <vector>

namespace MatterEngine {

// A previsao da trajetoria (vault: PoseSearchTrajectoryLibrary.cpp da UE 5.8, o modelo do CharacterMovementComponent): a
// partir do estado de agora e do pedido (a direcao e a velocidade desejadas e o rumo desejado do corpo), integra:
// a posicao com a velocidade (Euler explicito, antes de atualizar a velocidade); sem pedido, a frenagem (atrito e
// desaceleracao, em subpassos, parando na inversao); com pedido, o atrito puxa a velocidade para a direcao pedida, a
// aceleracao soma e a velocidade e presa no maximo; o rumo do corpo vai ao desejado a uma velocidade angular constante
// (ou, sem rumo desejado e com orientToMovement, ao da velocidade).
struct MotionMatchingMovementSettings3D {
    float maxAcceleration = 8.0f;
    float brakingDeceleration = 8.0f;
    float groundFriction = 8.0f;
    float brakingFriction = 8.0f;
    float brakingSubStepSeconds = 1.0f / 30.0f;
    // (Negativo: o rumo vai ao desejado na hora - o "RotationRate -1" do personagem do Game Animation Sample no chao.)
    float rotationRadiansPerSecond = 10.0f;
    bool orientToMovement = false;
    float stepSeconds = 1.0f / 60.0f;
};

struct MotionMatchingMovementState3D {
    Vec3 position;
    Vec3 velocity;
    float facingYaw = 0.0f;
    // O pedido: a direcao (XY; zero = parar), a velocidade e o rumo desejado do corpo (com o giro dele, rad/s, para o rumo
    // que gira - o giro parado); a direcao pedida pode girar tambem (a curva, rad/s).
    Vec3 desiredDirection;
    float desiredSpeed = 0.0f;
    bool hasDesiredFacing = false;
    float desiredFacingYaw = 0.0f;
    float desiredFacingYawRate = 0.0f;
    float desiredDirectionYawRate = 0.0f;
};

// Os pontos previstos nos instantes pedidos (positivos, crescentes).
[[nodiscard]] std::vector<MotionMatchingTrajectoryPoint3D> predictMotionTrajectory3D(const MotionMatchingMovementSettings3D& settings,
    const MotionMatchingMovementState3D& state, std::span<const float> offsets);

// O passado: a raiz gravada a cada atualizacao (o tempo do jogo), os mais velhos que maxSeconds descartados; um instante
// do passado e interpolado (preso no mais velho, como o PoseSearch).
class MotionMatchingTrajectoryHistory3D {
public:
    explicit MotionMatchingTrajectoryHistory3D(float maxSeconds = 1.0f) : m_maxSeconds(maxSeconds) {}
    void clear() { m_points.clear(); }
    void record(float timeSeconds, Vec3 position, float yaw, Vec3 velocity);
    // O ponto offsetSeconds (negativo) antes do ultimo gravado; sem nada gravado, false.
    [[nodiscard]] bool sample(float offsetSeconds, MotionMatchingTrajectoryPoint3D& out) const;

private:
    struct Record {
        float time;
        Vec3 position;
        float yaw;
        Vec3 velocity;
    };
    float m_maxSeconds;
    std::deque<Record> m_points;
};

} // namespace MatterEngine
