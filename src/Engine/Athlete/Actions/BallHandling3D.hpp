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
    // Andando: o balanco do pe que joga pode esticar ate swingStretch dele, o
    // apoio fica preso so ate o contato e o corpo do gesto pesa
    // (1 - runningBodyShare x andando): a corrida manda nele.
    float swingStretch = 1.25f;
    float runningBodyShare = 0.8f;
    // O apoio do chute correndo pode ficar no chao ate isto (s) alem do fim
    // natural dele - a passada de apoio, com o corpo freando sobre ele -, e
    // o corpo anda no maximo supportHoldReach (m) com ele preso (mais, a
    // perna nao alcanca).
    float supportHoldSeconds = 0.18f;
    float supportHoldReach = 0.12f;
    // O polo do joelho do gesto sai do joelho da pose nesta mistura (s).
    float kneeBlendSeconds = 0.10f;
    // A velocidade do pe do estilo (o referencial do toque no balanco) vem
    // filtrada neste tempo (s).
    float styleVelocitySeconds = 0.03f;
    // No corpo fisico, o gesto so comeca com o preparo (o peso no apoio)
    // acima disto.
    float physicalPrepare = 0.75f;
    // No corpo fisico, a transferencia do peso e menor (esta parte do
    // deslocamento da pelve) e mais lenta (o preparo vezes isto).
    float physicalShiftShare = 0.5f;
    float physicalPrepareStretch = 2.0f;
    // O chute correndo freia o corpo (0..1 da velocidade, pela forca) do
    // pedido ao fim do acompanhamento, nesta mola (s): a passada fica mais
    // lenta e o apoio mais longo, como na corrida para o chute.
    float runningShotSlowdown = 0.35f;
    // (Correndo, o chute tem este prazo a mais (s) para achar o apoio: a
    // passada se ajusta ao freio.)
    float runningShotAcquireExtra = 0.60f;
    // Depois do chute correndo, o corpo segue freado por isto (s) e volta a
    // corrida nesta mola (s): acelerando de uma vez, o pe do chute ainda
    // preso no chao ficava para tras.
    float shotRecoverySeconds = 0.25f;
    float shotRecoverySpring = 0.35f;
    float slowdownSeconds = 0.12f;
};

// O ritmo da passada (do estilo, no tick anterior): andando, quanto falta
// para cada pe sair (apoiado) ou pousar (no ar) e quanto dura o balanco (s).
struct BallHandlingGait3D {
    bool cycling = false;
    float movingShare = 0.0f;
    std::array<bool, 2> planted { true, true };
    std::array<float, 2> secondsToLiftoff { -1.0f, -1.0f };
    std::array<float, 2> secondsToTouchdown { -1.0f, -1.0f };
    std::array<float, 2> swingSeconds { -1.0f, -1.0f };
    // Os pes do estilo antes do gesto (o toque no balanco vai com eles).
    std::array<Vec3, 2> styleFoot {};
    std::array<Quaternion, 2> styleFootOrientation {};
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
    BallHandlingGait3D gait;
    // A velocidade do corpo agora e a que ele vai ter (o toque leva a maior
    // no rumo dele) e o centro de massa da pose anterior (o equilibrio).
    Vec3 bodyVelocity;
    Vec3 aheadVelocity;
    Vec3 centerOfMass;
    // A velocidade que o jogador pede (m/s, sem as escalas do manejo): a
    // corrida para o chute e ajustada a partir dela.
    float requestedSpeed = 0.0f;
    // O corpo pode agir: fora do Ativo, ou ja assentado nele. Desequilibrado
    // (o embalo de uma pancada), o pedido espera - levantando o pe, a base
    // virava um pe so e ele caia.
    bool bodyReady = true;
    // O corpo e fisico (o Ativo): o pe so sai do chao com o peso ja no apoio
    // (o preparo do corpo acima de physicalPrepare).
    bool physicalBody = false;
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
    // A escala da velocidade pedida ao corpo (o chute correndo freia).
    float speedScale = 1.0f;
    // A passada de apoio do chute correndo: o pe (-1 nenhum) que pousa
    // desviado do pouso do estilo por landingShift.
    int landingSide = -1;
    Vec3 landingShift;
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
    SmoothSpring3D<float> m_lookWeight;
    SmoothSpring3D<float> m_slow;
    // O gesto em andamento e de corrida (o apoio solta no contato, o pe volta
    // as pegadas no ar).
    bool m_gestureMoving = false;
    // Os pes do estilo no tick anterior (a velocidade deles) e o joelho do
    // pe que joga no comeco do gesto (o polo sai dele aos poucos).
    std::array<Vec3, 2> m_styleFootPrevious {};
    std::array<Vec3, 2> m_styleFootVelocity {};
    bool m_styleFootKnown = false;
    // Onde o pe do estilo pousou da ultima vez, em relacao a pelve (x a
    // frente, y para fora, z acima do chao), e a velocidade do corpo entao: a
    // corrida e periodica, o proximo pouso e parecido.
    Vec3 m_landLead;
    float m_landLeadSpeed = 0.0f;
    bool m_landLeadKnown = false;
    std::array<bool, 2> m_stylePlantedPrevious { true, true };
    // A passada de apoio: o pe dela no plano anterior, se ja pousou e ha
    // quanto tempo (s).
    int m_plantSide = -1;
    bool m_plantLanded = false;
    float m_plantLandedAge = 0.0f;
    // O tempo que o pedido em andamento esperou o corpo assentar (nao conta
    // nos prazos dele).
    float m_balanceWait = 0.0f;
    std::uint32_t m_balanceRecord = 0;
    // A corrida do plano ate o pouso do apoio (m/s; < 0 nenhuma) e o freio
    // que ainda fica depois do chute correndo (s).
    float m_approachSpeed = -1.0f;
    float m_recoverLeft = 0.0f;
    float m_recoverSlow = 0.0f;
    // O apoio da passada pousou: o corpo freia ate o fim do gesto.
    bool m_shotBraking = false;
    Vec3 m_kneeStart { 1.0f, 0.0f, 0.0f };
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
