#pragma once

#include "Engine/Athlete/Actions/BallGesture3D.hpp"
#include "Engine/Athlete/Actions/BallHandlingIO3D.hpp"
#include "Engine/Athlete/Core/CriticalSpring3D.hpp"
#include "Engine/Athlete/Intent/ActionQueue3D.hpp"
#include "Engine/Athlete/Sensing/BallPredictor3D.hpp"

#include <array>
#include <cstdint>

namespace MatterEngine {

// O manejo v2 (plano v2, fase C; pedidos do usuario de 10/10, noite): a
// acao depende so de a bola estar na area do jogador, e o pe indo ate ela e
// so o visual.
//  - Area (a da V3): a base da bola ate a cintura e, no plano, ate uma perna
//    esticada da pelve.
//  - Pedido guardado ate requestBuffer: a bola entrando na area nesse tempo,
//    ele sai; com a bola na area, sai sempre (a garantia).
//  - A forca (o resultado aprovado, BallOutcome3D) sai na fracao da regua
//    "tempo estimado ate o contato" do tempo que o pe levaria ate a bola (0:
//    na hora do pedido). A bola indo sair da area antes disso: a forca sai
//    ja, com ela ainda dentro.
//  - O gesto (BallGesture3D) e o visual: o pe, a superficie e a pose para a
//    bola de verdade; so quando o gesto nem conseguiria comecar (a bola atras
//    no chute, longe, alta, do outro lado), para uma bola imaginada logo a
//    frente, do lado da perna. Rapido, nunca cruza.
//  - Nada de mexer na locomocao: sem freio, sem passada de apoio, sem pe
//    reservado. Andando, o gesto vai no balanco do pe (o que esta no ar, ou
//    o que sai do chao logo) e volta a passada. Mudando o movimento depois
//    da forca, a perna sai do gesto numa mistura curta.
struct BallPlaySettings3D {
    BallPlaySettings3D();
    // A area: a base da bola ate isto acima da pelve (m) e, no plano, ate
    // areaRadius dela (m).
    float areaRadius = 0.95f;
    float waistAbovePelvis = 0.05f;
    // Andando, o centro da area vai a frente no rumo do movimento: ate
    // areaLead (m), cheio a areaLeadSpeed (m/s).
    float areaLead = 0.45f;
    float areaLeadSpeed = 4.5f;
    // O pedido guardado (s); a bola que esteve na area ate guaranteeGrace
    // atras ainda vale (s); o intervalo depois de cada forca (s).
    float requestBuffer = 0.25f;
    float guaranteeGrace = 0.06f;
    float cooldown = 0.25f;
    // A regua: a forca nesta fracao do tempo ate o contato visual (0: na
    // hora do pedido).
    float contactTiming = 0.0f;
    // A velocidade "a frente" do toque: a do corpo daqui a carryAhead (da aba
    // "Forca na bola"), indo a pedida nesta aceleracao (m/s2) - a do manejo V3,
    // onde a forca foi calibrada.
    float carryAcceleration = 16.0f;
    // O gesto: armado e batida (s, mais a parte pela forca); no balanco da
    // passada nao ha armado, so a batida (s). O pe longe do contato: o armado
    // cresce para ele andar no maximo windupTravelSpeed (m/s).
    float touchWindup = 0.05f, touchWindupPower = 0.04f;
    float touchStrike = 0.05f, touchStrikePower = 0.02f;
    float shotWindup = 0.10f, shotWindupPower = 0.08f;
    float shotStrike = 0.07f, shotStrikePower = 0.03f;
    float swingStrike = 0.11f;
    float windupTravelSpeed = 3.0f;
    // Andando: o pe no ar com ao menos isto ate pousar (s) faz o gesto ja; o
    // apoiado que sai do chao ate liftWait (s) espera ele sair.
    float minimumSwingLeft = 0.16f;
    float liftWait = 0.22f;
    // O contato de verdade: o tornozelo ate reachLimit da perna do quadril
    // (no contato) e pelo menos minimumReach (m); a bola ate maximumHeight
    // (m, a base dela acima do chao); do lado da perna (a bola ate sideSlack
    // m do outro lado do meio ainda vale).
    float reachLimit = 0.97f;
    float minimumReach = 0.32f;
    float maximumHeight = 0.45f;
    float sideSlack = 0.06f;
    // A bola imaginada: a frente da pelve, para o lado da perna e adiantada
    // no rumo do movimento (m).
    float imaginedAhead = 0.32f;
    float imaginedSide = 0.14f;
    float imaginedLead = 0.12f;
    // As pernas nunca se cruzam: o pe que joga fica pelo menos isto do lado
    // dele (m).
    float sideGap = 0.12f;
    // Depois da forca, o movimento pedido mudando (o rumo mais que
    // interruptTurn rad, ou a velocidade pedida mais que interruptSpeed m/s
    // diferente da de entao): a perna sai do gesto nesta mistura (s).
    float interruptTurn = 0.6f;
    float interruptSpeed = 0.8f;
    float interruptSeconds = 0.10f;
    // O gesto que acaba com a perna ainda nele (andando, o acompanhamento
    // termina no ar): ela volta a locomocao nesta mistura (s).
    float handOverSeconds = 0.14f;
    // O corpo: o preparo, os alvos, o olhar e os bracos nestas molas (s); a
    // pelve sobre o apoio ate maximumPelvisShift (m); andando, o corpo do
    // gesto pesa (1 - runningBodyShare x andando).
    float prepareSeconds = 0.16f;
    float targetSeconds = 0.12f;
    float lookSeconds = 0.12f;
    float armSeconds = 0.15f;
    float maximumPelvisShift = 0.10f;
    float pelvisMassShare = 0.85f;
    float runningBodyShare = 0.85f;
    // A perna entra no gesto nesta mistura (s: a do IK nao e identica a do
    // clipe, e o primeiro tick dava o tranco); o polo do joelho sai do joelho
    // da pose nesta (s); a velocidade do pe do estilo vem filtrada neste
    // tempo (s).
    float legBlendSeconds = 0.05f;
    float kneeBlendSeconds = 0.08f;
    float styleVelocitySeconds = 0.03f;
    BallGestureSettings3D gesture;
    BallPredictorSettings3D predictor;
};

// O gesto escolhido: o plano dele, quando comeca (s a partir de agora), o
// referencial e se e na bola imaginada.
struct BallPlayGesturePlan3D {
    BallPlan3D plan;
    float startIn = 0.0f;
    BallGestureFrame3D frame = BallGestureFrame3D::World;
    bool moving = false;
    bool imagined = false;
};

class BallPlay3D {
public:
    void reset(const RagdollProfile3D& profile);
    void update(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
        const BallPlaySettings3D& settings, BallHandlingOutput3D& output);
    // A bola do pedido ou da acao em andamento (0 nenhuma).
    [[nodiscard]] std::uint64_t targetBallKey() const;
    [[nodiscard]] const ActionQueue3D& queue() const { return m_queue; }
    [[nodiscard]] const BallGesture3D& gesture() const { return m_gesture; }
    [[nodiscard]] bool ready() const { return m_ready; }
    // Telemetria: a acao aceita (do aceite a forca), quando a forca sai (s
    // depois do aceite), o tempo ate o contato visual estimado no aceite (s)
    // e se o gesto e na bola imaginada.
    [[nodiscard]] bool acting() const { return m_action; }
    [[nodiscard]] float forceAt() const { return m_forceAt; }
    [[nodiscard]] float estimatedContact() const { return m_estimatedContact; }
    [[nodiscard]] bool imagined() const { return m_imagined; }
    // A perna saindo do gesto (a mudanca de movimento depois da forca).
    [[nodiscard]] bool interrupting() const { return m_interrupting; }

    // A bola na area em volta da pelve dada (andando a velocity, ela vai a
    // frente).
    [[nodiscard]] static bool inArea(Vec3 ball, float radius, Vec3 pelvis, const BallPlaySettings3D& settings,
        Vec3 velocity = {});

private:
    // O gesto para a bola (posicao e velocidade agora): o pe, quando, onde.
    // forcedSide: so esse pe (-1 os dois); startNow: o pe ja esta livre.
    [[nodiscard]] BallPlayGesturePlan3D planGesture(const BallHandlingInput3D& input, Vec3 ballPosition,
        Vec3 ballVelocity, const BallPlaySettings3D& settings, int forcedSide, bool startNow) const;
    void startGesture(const BallHandlingInput3D& input, const BallPlayGesturePlan3D& plan,
        const BallPlaySettings3D& settings);
    // Quando a bola sai da area (s a partir de agora; -1 fica ate horizon).
    [[nodiscard]] float areaExit(const BallHandlingInput3D& input, float horizon,
        const BallPlaySettings3D& settings) const;
    void applyForce(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
        const BallPlaySettings3D& settings, BallHandlingOutput3D& output);
    void bodyTargets(const BallHandlingInput3D& input, const BallPlaySettings3D& settings,
        BallHandlingOutput3D& output);

    bool m_ready = false;
    AthleteLegs3D m_legs;
    BallFootBox3D m_box;
    ActionQueue3D m_queue;
    BallGesture3D m_gesture;
    // A acao aceita: o pedido, a bola, o relogio desde o aceite, quando a
    // forca sai e se ja saiu; o tempo ate o contato visual estimado.
    bool m_action = false;
    BallOutcomeRequest3D m_intent;
    std::uint64_t m_ballKey = 0;
    float m_actionClock = 0.0f;
    float m_forceAt = 0.0f;
    bool m_forced = false;
    float m_estimatedContact = 0.0f;
    // O gesto planejado esperando o pe sair do chao (s ate comecar) e o lado.
    bool m_waitingFoot = false;
    float m_startIn = 0.0f;
    int m_waitSide = -1;
    bool m_imagined = false;
    // A bola de antes da forca (o gesto que comeca depois dela vai onde a
    // bola estaria sem ela).
    bool m_virtualBall = false;
    Vec3 m_virtualPosition, m_virtualVelocity;
    float m_virtualClock = 0.0f;
    // O plano do gesto em curso e o pedido dele.
    BallPlan3D m_plan;
    BallOutcomeRequest3D m_gestureIntent;
    std::uint64_t m_gestureBall = 0;
    bool m_gestureMoving = false;
    BallGestureFrame3D m_gestureFrame = BallGestureFrame3D::World;
    Vec3 m_kneeStart { 1.0f, 0.0f, 0.0f };
    // A garantia: ha quanto tempo a bola esteve na area (s); o intervalo
    // depois da forca (s).
    float m_sinceInArea = 10.0f;
    float m_cooldown = 0.0f;
    // O tempo que o pedido guardado esperou a acao anterior (nao conta no
    // tempo guardado dele).
    float m_busyWait = 0.0f;
    std::uint32_t m_busyRecord = 0;
    // A interrupcao: o movimento pedido no aceite e o peso da perna saindo.
    Vec3 m_requestedAtAccept;
    bool m_interrupting = false;
    float m_exit = 1.0f;
    int m_previousSide = -1;
    // Os pes do estilo (a velocidade deles, filtrada).
    std::array<Vec3, 2> m_styleFootPrevious {};
    std::array<Vec3, 2> m_styleFootVelocity {};
    bool m_styleFootKnown = false;
    // O corpo: o peso e os alvos nas molas; o deslocamento e a descida da
    // pelve aplicados no tick anterior (ja estao na pose).
    bool m_bodyLive = false;
    int m_bodySide = 0;
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
    Vec3 m_appliedShift;
    float m_appliedDrop = 0.0f;
    // A perna do ultimo tick do gesto (o tornozelo, o pe, o joelho, o peso e o
    // lado) e o que falta da volta dela a locomocao (s).
    Vec3 m_lastAnkle;
    Vec3 m_lastVelocity;
    Quaternion m_lastFoot;
    Vec3 m_lastKnee { 1.0f, 0.0f, 0.0f };
    float m_lastLeg = 0.0f;
    // O peso da perna no comeco do gesto (o anterior cortado na mesma perna).
    float m_legStart = 0.0f;
    int m_lastSide = 0;
    float m_settleLeft = 0.0f;
};

} // namespace MatterEngine
