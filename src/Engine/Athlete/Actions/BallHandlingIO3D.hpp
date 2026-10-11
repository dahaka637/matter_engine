#pragma once

#include "Engine/Athlete/Actions/BallActionPlanner3D.hpp"
#include "Engine/Physics/InflatedBall3D.hpp"

#include <array>
#include <cstdint>

namespace MatterEngine {

// As entradas e saidas do manejo de bola, comuns ao manejo da Etapa 5
// (BallHandling3D, a sair na limpeza) e ao v2 (BallPlay3D): o runtime monta a
// entrada e aplica a saida do mesmo jeito para os dois.

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
    // A outra perna ainda saindo do gesto anterior (cortado por este): o
    // tornozelo, o joelho e o pe dela no mundo, com peso (-1 nenhuma).
    int releaseSide = -1;
    Vec3 releaseAnkle;
    Vec3 releaseKnee { 1.0f, 0.0f, 0.0f };
    Quaternion releaseFoot;
    float releaseWeight = 0.0f;
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
    // Os pes como a locomocao os poria sem o gesto e o joelho do estilo
    // (mundo): a volta do gesto vai a eles.
    std::array<Vec3, 2> clipFoot {};
    std::array<Quaternion, 2> clipFootOrientation {};
    std::array<Vec3, 2> clipKnee { Vec3 { 1.0f, 0.0f, 0.0f }, Vec3 { 1.0f, 0.0f, 0.0f } };
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
    // O corpo segue a animacao (o padrao): a velocidade "a frente" do toque e
    // a dele prevista (no fisico, a de agora - como no manejo V3).
    bool animatedBody = true;
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
    // O pe de apoio que pousa ja (-1 nenhum; as pegadas).
    int quickPlantSide = -1;
    // Interagindo (o pedido ou o gesto): vira o dono da bola.
    bool interacting = false;
    std::uint64_t interactingBallKey = 0;
    // Telemetria.
    bool touched = false;
    BallPlan3D plan;
};

} // namespace MatterEngine
