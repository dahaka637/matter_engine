#pragma once

#include "Engine/Athlete/Ball/BallOutcome3D.hpp"
#include "Engine/Athlete/Core/Kinematics3D.hpp"
#include "Engine/Athlete/Sensing/BallPredictor3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <vector>

namespace MatterEngine {

// A geometria do pe para o toque: a caixa dele no referencial do link
// (centro, orientacao e meia-dimensao). As superficies saem dela.
struct BallFootBox3D {
    bool valid = false;
    std::array<Vec3, 2> center {};
    std::array<Quaternion, 2> orientation {};
    std::array<Vec3, 2> half {};
};
[[nodiscard]] BallFootBox3D resolveBallFootBox3D(const RagdollProfile3D& profile, const AthleteLegs3D& legs);

// O ponto da superficie e a normal dela (para fora), no referencial do link
// do pe.
void ballFootSurface3D(const BallFootBox3D& box, int side, BallTouchSurface3D surface, Vec3& point, Vec3& normal);

// O corpo para o planejador (o do tick: a pose que o corpo segue).
// Uma opcao de passada de apoio (chute correndo): o pe que pousa, quando
// (s; negativo: ja pousou ha tanto) e onde o estilo o pousaria (ou onde ja
// esta), e o balanco do outro pe, o do chute, que acompanha esse pouso.
struct BallPlantOption3D {
    int side = -1;
    float landTime = 0.0f;
    Vec3 landPosition;
    bool landed = false;
    float kickFrom = 0.0f;
    float kickUntil = 0.0f;
};

struct BallPlannerBody3D {
    Vec3 pelvis;
    // O rumo do corpo (rad) e a velocidade dele no plano (a da capsula).
    float facingYaw = 0.0f;
    Vec3 velocity;
    std::array<Vec3, 2> hip {};
    std::array<Vec3, 2> foot {};
    std::array<Quaternion, 2> footOrientation {};
    // A velocidade de cada pe agora (o gesto parte dela) e para onde o
    // joelho aponta (o polo do gesto sai dele aos poucos).
    std::array<Vec3, 2> footVelocity {};
    std::array<Vec3, 2> knee { Vec3 { 1.0f, 0.0f, 0.0f }, Vec3 { 1.0f, 0.0f, 0.0f } };
    // Correndo, o toque vai no balanco do pe do estilo: onde ele esta e a
    // velocidade dele, quando sai do chao (0 se ja no ar), onde e quando pousa
    // (s; swingLandTime < 0: sem previsao, o toque nao vai no balanco).
    std::array<Vec3, 2> swingFoot {};
    std::array<Vec3, 2> swingVelocity {};
    std::array<float, 2> swingLift {};
    std::array<Vec3, 2> swingLand {};
    std::array<float, 2> swingLandTime { -1.0f, -1.0f };
    // Correndo, o chute pela passada de apoio: as opcoes de pouso do apoio
    // (o proximo e o seguinte de cada pe), cada uma podendo pousar ao lado
    // da bola com o outro pe chutando no balanco dele.
    bool plantStep = false;
    std::array<BallPlantOption3D, 4> plantOptions {};
    int plantOptionCount = 0;
    // A passada de apoio ajusta a corrida: a velocidade de agora (a pedida,
    // sem freio) e o pouso natural em relacao a pelve (a frente, para o lado
    // de cada pe, acima do chao), medido a landLeadSpeed (o avanco cresce
    // com a velocidade).
    float runSpeed = 0.0f;
    Vec3 landLead { 0.37f, 0.09f, 0.10f };
    float landLeadSpeed = 4.5f;
    // As janelas de cada pe (s a partir de agora): apoiado de plantedFrom a
    // plantedUntil e livre (no balanco) de freeFrom a freeUntil. Parado, os
    // dois apoiados sempre e livres ja (0 / 1e9).
    std::array<float, 2> plantedFrom {};
    std::array<float, 2> plantedUntil { 1e9f, 1e9f };
    // O fim natural do apoio (a passada; plantedUntil pode ir alem, segurando
    // o pe - com custo).
    std::array<float, 2> plantedNatural { 1e9f, 1e9f };
    std::array<float, 2> freeFrom {};
    std::array<float, 2> freeUntil { 1e9f, 1e9f };
    float groundHeight = 0.0f;
    bool moving = false;
};

struct BallPlannerSettings3D {
    // O alcance: do quadril ao tornozelo ate reachLimit da perna (com a pelve
    // do plano), mais pelvisReach (m) com custo (a pelve desce e desliza).
    float reachLimit = 0.97f;
    float pelvisReach = 0.10f;
    // A perna dobrada demais: o tornozelo a menos disto do quadril (m).
    float minimumReach = 0.30f;
    // Os instantes candidatos (s) e o horizonte do toque e do chute (s).
    float interval = 1.0f / 30.0f;
    float touchHorizon = 0.6f;
    float shotHorizon = 0.8f;
    // A perna: armado e batida minimos (s) e a parte pela forca.
    float touchWindup = 0.07f, touchWindupPower = 0.08f;
    float touchStrike = 0.07f, touchStrikePower = 0.04f;
    float shotWindup = 0.12f, shotWindupPower = 0.12f;
    float shotStrike = 0.08f, shotStrikePower = 0.04f;
    // O pe vai de onde esta ate o contato a no maximo esta velocidade media
    // (m/s) no armado: longe, o armado cresce (0,75 m em 0,1 s era um tranco).
    // Andando, o pe ja sai do chao atras do corpo: o armado minimo encolhe
    // para esta parte.
    float windupTravelSpeed = 2.5f;
    float movingWindupShare = 0.5f;
    // Os pesos do custo: o tempo, o alcance alem de 85% da perna, a perna do
    // outro lado da bola, a pelve indo buscar e a troca de pe.
    float costTime = 1.0f;
    float costReach = 4.0f;
    float costOtherSide = 0.35f;
    float costPelvis = 2.0f;
    float costSwitch = 0.25f;
    // Segurar o apoio alem do fim natural dele: custo por 0,05 s.
    float costHold = 1.0f;
    // O toque no balanco: o pe do contato longe de onde o balanco do estilo
    // passaria (custo por (d / 0,15 m)^2; acima de swingDeviation, nao).
    float costSwingPath = 1.0f;
    float swingDeviation = 0.30f;
    // A passada de apoio do chute correndo: o pe de apoio pousa ao lado da
    // bola (plantSideGap do centro dela, plantBehind atras no rumo do chute),
    // no maximo plantShift (m) do pouso do estilo (custo por (d / 0,2)^2), e
    // a batida sai entre plantLeadMin e plantLeadMax (s) depois do pouso.
    float plantSideGap = 0.24f;
    float plantBehind = 0.06f;
    float plantShift = 0.40f;
    // (O passo de apoio pode encurtar e ir para o lado, mas esticar para a
    // frente so ate isto (m): alem, a perna nao alcanca o chao.)
    float plantStretch = 0.10f;
    float costPlant = 1.0f;
    float plantLeadMin = 0.08f;
    float plantLeadMax = 0.28f;
    // A corrida para o chute: entre plantSpeedMin e plantSpeedMax da de agora,
    // para a pelve chegar ao pouso natural atras do apoio; com o apoio no
    // chao, o corpo freia a plantBrake dela.
    float plantSpeedMin = 0.45f;
    float plantSpeedMax = 1.05f;
    float plantBrake = 0.6f;
    // O pe que joga nunca cruza a linha do de apoio: fica pelo menos isto do
    // lado dele (m).
    float sideGap = 0.12f;
};

// O plano de um toque: o pe, a superficie, o instante e a pose do pe no
// contato.
struct BallPlan3D {
    bool valid = false;
    const char* reason = "";
    int side = 0;
    BallTouchSurface3D surface = BallTouchSurface3D::Inside;
    float contactTime = 0.0f;
    Vec3 ball;
    Vec3 push { 1.0f, 0.0f, 0.0f };
    Vec3 footPosition;
    Quaternion footOrientation;
    float windupSeconds = 0.0f;
    float strikeSeconds = 0.0f;
    // A fracao da perna no contato e quanto a pelve precisa ir buscar (m).
    float reach = 0.0f;
    float pelvisNeed = 0.0f;
    float cost = 0.0f;
    // A passada de apoio (chute correndo): o pe de apoio, onde ele pousa e o
    // quanto isso desvia do pouso do estilo (plantSide < 0: nenhuma).
    int plantSide = -1;
    float plantTime = 0.0f;
    Vec3 plantTarget;
    Vec3 plantShift;
    // A velocidade da corrida ate o pouso do apoio (m/s; < 0: a de agora).
    float runSpeed = -1.0f;
};

// A superficie pela direcao do empurrao em relacao ao corpo (a do V3,
// aprovada): o lado de dentro ou o de fora do pe (o peito do pe no chute e
// no toque correndo para a frente), a sola e o calcanhar atras.
[[nodiscard]] BallTouchSurface3D chooseBallSurface3D(int side, Vec3 push, Vec3 bodyForward,
    BallActionKind3D kind, bool moving, bool ballBehind, float curve);

// A pose do pe no contato: a superficie encostada na bola, de frente para o
// empurrao (nos limites do quadril).
void ballContactFootPose3D(const BallFootBox3D& box, int side, BallTouchSurface3D surface, Vec3 ball,
    float radius, Vec3 push, Vec3 bodyForward, float power, float lift, float groundHeight, Vec3& position,
    Quaternion& orientation);

// O melhor plano entre os dois pes e os instantes ate o horizonte, pela
// tabela da bola (a 1/30 s). previousSide: o pe do plano anterior (-1
// nenhum; a troca custa).
[[nodiscard]] BallPlan3D planBallAction3D(const BallOutcomeRequest3D& intent, const BallPlannerBody3D& body,
    const std::vector<BallPrediction3D>& table, float tableInterval, const AthleteLegs3D& legs,
    const BallFootBox3D& box, float ballRadius, int previousSide, const BallPlannerSettings3D& settings);

} // namespace MatterEngine
