#pragma once

#include "Engine/Athlete/Actions/BallActionPlanner3D.hpp"
#include "Engine/Athlete/Actions/BallGesture3D.hpp"
#include "Engine/Athlete/Core/CriticalSpring3D.hpp"
#include "Engine/Athlete/Intent/ActionQueue3D.hpp"
#include "Engine/Physics/InflatedBall3D.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace MatterEngine {

// O que o gesto pede do corpo: a perna que joga (tornozelo, joelho e pe no
// mundo), a pelve (desce, vai para cima do apoio, gira), o tronco, os
// bracos (o punho em relacao ao ombro), o olhar e as pernas sem cruzar. O
// estilo aplica (no legado, o BallGesturePose3D do modo padrao).
struct BallBodyTargets3D {
    bool active = false;
    int side = 0;
    float weight = 0.0f;
    Vec3 ankle;
    Vec3 knee { 1.0f, 0.0f, 0.0f };
    Quaternion foot;
    float legWeight = 0.0f;
    float pelvisDrop = 0.0f;
    Vec3 pelvisShift;
    float pelvisYaw = 0.0f, pelvisPitch = 0.0f, pelvisRoll = 0.0f;
    float trunkTwist = 0.0f, trunkBend = 0.0f, trunkSide = 0.0f;
    std::array<Vec3, 2> wrist {};
    std::array<float, 2> armWeight {};
    Vec3 look;
    float lookWeight = 0.0f;
    float separateGap = 0.0f;
    Vec3 separateForward { 1.0f, 0.0f, 0.0f };
    float separateWeight = 0.0f;
};

struct BallHandlingSettings3D {
    BallPlannerSettings3D planner;
    BallGestureSettings3D gesture;
    BallPredictorSettings3D predictor;
    // O prazo de aquisicao (s: a bola ainda nao esta ao alcance de um gesto).
    float acquireSeconds = 0.25f;
    // A folga do sensor do pe (m; D3: 7 cm, testar 3 depois).
    float sensorMargin = 0.07f;
    // O corpo comeca a preparar na hora do pedido (olhar, peso no apoio) e
    // entra nestas molas (s); os alvos dele (pelve, tronco, bracos) e o
    // olhar seguem os seus nestas (s).
    float prepareSeconds = 0.20f;
    float targetSeconds = 0.16f;
    float lookSeconds = 0.14f;
    // A pelve sobre o apoio: no maximo isto (m). Ela leva esta parte da
    // massa (o resto sao as pernas, presas nos pes).
    float maximumPelvisShift = 0.15f;
    float pelvisMassShare = 0.85f;
    // O balanco dos bracos acompanha a velocidade do pe nesta mola (s).
    float armSeconds = 0.20f;
    // Depois do pouso do pe (preso pelas pegadas), a perna do gesto sai neste
    // tempo (s).
    float handOverSeconds = 0.12f;
};

// A bola-alvo neste tick (o estado publicado no passo anterior).
struct BallHandlingBall3D {
    bool valid = false;
    std::uint64_t key = 0;
    Vec3 position;
    Vec3 velocity;
    Vec3 spin;
    float radius = 0.11f;
    float mass = 0.43f;
    InflatedBallSensorTouch3D sensor;
};

struct BallHandlingInput3D {
    // O pedido (borda) e a bola mais perto na hora dele (ela fica travada).
    bool request = false;
    BallOutcomeRequest3D intent;
    std::uint64_t requestBallKey = 0;
    // A bola travada pelo pedido em andamento (o runtime busca a de
    // targetBallKey()).
    BallHandlingBall3D ball;
    BallPlannerBody3D body;
    // A velocidade do corpo agora e a que ele vai ter (o toque leva a maior
    // no rumo dele) e o centro de massa da pose anterior (o equilibrio).
    Vec3 bodyVelocity;
    Vec3 aheadVelocity;
    Vec3 centerOfMass;
    // O link do pe de cada lado (o sensor).
    std::array<std::uint32_t, 2> footLinks {};
    float deltaTime = 1.0f / 120.0f;
};

struct BallHandlingOutput3D {
    BallBodyTargets3D body;
    // O pe que joga (-1 nenhum) e o de apoio reservado (fica no chao).
    int actingSide = -1;
    int reservedSide = -1;
    // O sensor: a bola, o link do pe e a folga.
    bool sensorActive = false;
    std::uint64_t sensorBallKey = 0;
    std::uint32_t sensorLink = 0;
    float sensorMargin = 0.0f;
    // O resultado do toque: impulsos linear (N.s) e angular (N.m.s) na bola.
    bool impulse = false;
    std::uint64_t impulseBallKey = 0;
    Vec3 linearImpulse;
    Vec3 angularImpulse;
    Vec3 outcomeVelocity;
    // Interagindo (o pedido ou o gesto): vira o dono da bola.
    bool interacting = false;
    std::uint64_t interactingBallKey = 0;
    // Telemetria.
    bool touched = false;
    BallPlan3D plan;
};

class BallHandling3D {
public:
    void reset(const RagdollProfile3D& profile);
    void update(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
        const BallHandlingSettings3D& settings, BallHandlingOutput3D& output);
    // A bola que o pedido em andamento travou (0 nenhuma).
    [[nodiscard]] std::uint64_t targetBallKey() const;
    [[nodiscard]] const ActionQueue3D& queue() const { return m_queue; }
    [[nodiscard]] const BallGesture3D& gesture() const { return m_gesture; }
    [[nodiscard]] bool ready() const { return m_ready; }

private:
    bool m_ready = false;
    AthleteLegs3D m_legs;
    BallFootBox3D m_box;
    ActionQueue3D m_queue;
    BallGesture3D m_gesture;
    BallPlan3D m_plan;
    // O pedido do gesto em andamento (o desfecho dele pode ja ter saido).
    BallOutcomeRequest3D m_gestureIntent;
    std::uint64_t m_gestureBall = 0;
    bool m_gestureTouched = false;
    bool m_gestureResolved = false;
    int m_previousSide = -1;
    int m_prepareSide = 0;
    // O corpo: o peso e os alvos nas molas (vivos enquanto ha o que fazer) e
    // o deslocamento e a descida da pelve aplicados no tick anterior (ja
    // estao na pose: no centro de massa e nos quadris).
    bool m_bodyLive = false;
    SmoothSpring3D<float> m_weight;
    SmoothSpring3D<Vec3> m_shift;
    SmoothSpring3D<float> m_drop;
    SmoothSpring3D<float> m_yaw;
    SmoothSpring3D<float> m_trunkSide;
    SmoothSpring3D<float> m_trunkBend;
    SmoothSpring3D<float> m_armSwing;
    std::array<SmoothSpring3D<Vec3>, 2> m_wrist {};
    SmoothSpring3D<Vec3> m_look;
    Vec3 m_appliedShift;
    float m_appliedDrop = 0.0f;
    // A perna do gesto saindo depois do pouso: o pe, a orientacao e o joelho
    // do fim, o lado e o tempo que falta.
    Vec3 m_lastAnkle;
    Quaternion m_lastFoot;
    Vec3 m_lastKnee { 1.0f, 0.0f, 0.0f };
    int m_lastSide = 0;
    float m_settleLeft = 0.0f;
    std::vector<BallPrediction3D> m_table;
};

} // namespace MatterEngine
