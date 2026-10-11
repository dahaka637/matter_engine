#pragma once

#include "Engine/Athlete/Actions/BallHandlingIO3D.hpp"
#include "Engine/Athlete/Dribble/BodyForecast3D.hpp"
#include "Engine/Athlete/Dribble/ContactPose3D.hpp"
#include "Engine/Athlete/Dribble/DribbleBall3D.hpp"
#include "Engine/Athlete/Dribble/DribbleTypes3D.hpp"
#include "Engine/Athlete/Dribble/SwingTouch3D.hpp"
#include "Engine/Athlete/Dribble/TouchBody3D.hpp"
#include "Engine/Athlete/Dribble/TouchPlanner3D.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace MatterEngine {

// O manejo refeito (plano v5, pedido do usuario de 11/10 noite: o pe vai ate
// a bola, manejo continuo e intercalado, o tempo do pe ate a bola e a
// antecipacao, refeito do zero). O maestro: os cliques entram numa fila;
// cada pedido e planejado (TouchPlanner3D) num balanco de uma perna - andando,
// o balanco do clipe; parado, um passo - na bola prevista (a do toque anterior
// ja planejado: o encadeamento); quando a janela do balanco comeca o toque se
// compromete e o balanco (SwingTouch3D) leva o pe ate a bola; a forca sai no
// contato. A garantia continua: com a bola na area, o pedido sempre vira
// forca.
class Dribble3D {
public:
    void reset(const RagdollProfile3D& profile);
    [[nodiscard]] bool ready() const { return m_feet.valid; }
    void update(const BallHandlingInput3D& input, const DribbleGaitForecast3D& gait, const DribbleCapsule3D& capsule,
        const BallOutcomeSettings3D& outcome, const DribbleSettings3D& settings, DribbleOutput3D& output);
    // A bola dos pedidos em andamento (0 nenhuma).
    [[nodiscard]] std::uint64_t targetBallKey() const { return m_requests.empty() && !anySwing() ? 0 : m_ballKey; }
    [[nodiscard]] const ContactFeet3D& feet() const { return m_feet; }
    [[nodiscard]] const SwingTouch3D& swing(int side) const { return m_swings[static_cast<std::size_t>(side & 1)]; }

private:
    struct Request {
        int id = 0;
        BallOutcomeRequest3D intent;
        std::uint64_t ballKey = 0;
        float age = 0.0f;
        bool committed = false;
        bool done = false;
        // A bola esteve na area enquanto o pedido esperava (a garantia) e ha
        // quanto tempo ela esta fora (s).
        bool sawBall = false;
        float outside = 0.0f;
        // O pe chegou na bola (a pose do contato sai neste tick): a forca no
        // proximo, quando a pose aparece - a bola sai quando o pe encosta.
        bool armed = false;
        // O chute com a bola atras ja teve o toque de preparo.
        bool setupDone = false;
        TouchPlan3D plan;
    };

    [[nodiscard]] bool anySwing() const { return m_swings[0].active() || m_swings[1].active(); }
    // A intencao do toque planejado (o preparo do chute: um toque de sola).
    [[nodiscard]] BallOutcomeRequest3D intentOf(const Request& request, const DribbleSettings3D& settings) const;
    [[nodiscard]] SwingLive3D live(const BallHandlingInput3D& input, int side) const;
    void applyForce(const Request& request, BallTouchSurface3D surface, const BallHandlingInput3D& input,
        const BallOutcomeSettings3D& outcome, const DribbleSettings3D& settings, DribbleOutput3D& output);
    void bodyLayer(const BallHandlingInput3D& input, const DribbleSettings3D& settings, DribbleOutput3D& output);

    ContactFeet3D m_feet;
    float m_clock = 0.0f;
    std::vector<Request> m_requests;
    int m_nextId = 1;
    std::uint64_t m_ballKey = 0;
    std::array<SwingTouch3D, 2> m_swings {};
    // O pe do clipe de cada lado (o tornozelo) e a velocidade e a aceleracao
    // dele (do tick anterior para este).
    std::array<Vec3, 2> m_clipAnkle {};
    std::array<Vec3, 2> m_clipVelocity {};
    std::array<Vec3, 2> m_clipAcceleration {};
    bool m_clipKnown = false;
    float m_lastYaw = 0.0f;
    float m_yawRate = 0.0f;
    bool m_yawKnown = false;
    // A previsao do corpo deste tick.
    BodyForecast3D m_forecast;
    // Desde o ultimo toque (s): a passada de conducao fica ate dribbleLinger.
    float m_sinceTouch = 10.0f;
    // O corpo do gesto: o de cada toque (ate esmaecer) e o peso na mola.
    std::array<TouchBodyPlan3D, 2> m_bodyPlans {};
    float m_bodyWeight = 0.0f;
    // As molas do corpo (valores e velocidades: os planos mudando de uma vez -
    // desistindo, trocando de perna - nao saltam; toques seguidos partem dos
    // valores de agora).
    DribbleBody3D m_bodyValue;
    DribbleBody3D m_bodyRate;
    // A descida do alcance (na sua mola curta).
    float m_reachDrop = 0.0f;
    float m_reachDropRate = 0.0f;
    float m_lookWeight = 0.0f;
    Vec3 m_look;
    bool m_lookKnown = false;
};

} // namespace MatterEngine
