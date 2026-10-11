#pragma once

#include "Engine/Athlete/Dribble/BodyForecast3D.hpp"
#include "Engine/Athlete/Dribble/ContactPose3D.hpp"
#include "Engine/Athlete/Dribble/DribbleTypes3D.hpp"
#include "Engine/Athlete/Dribble/TouchPlanner3D.hpp"

namespace MatterEngine {

// O que o balanco le a cada tick: o corpo agora (a pelve no chao, o rumo, a
// velocidade), o quadril desta perna e o pe do clipe dela agora (o
// tornozelo com a velocidade e a aceleracao, a orientacao do pe e o joelho
// do estilo), a previsao do corpo (o contato no referencial do corpo no
// instante dele) e o chao.
struct SwingLive3D {
    Vec3 pelvisGround;
    float yaw = 0.0f;
    Vec3 bodyVelocity;
    Vec3 hip;
    Vec3 clipAnkle;
    Vec3 clipVelocity;
    Vec3 clipAcceleration;
    Quaternion clipOrientation;
    Vec3 clipKnee { 1.0f, 0.0f, 0.0f };
    const BodyForecast3D* forecast = nullptr;
    float groundHeight = 0.0f;
};

// Um balanco com toque (plano v5): o pe parte do pe do clipe (no comeco do
// balanco, com a velocidade dele: sem salto), vai ate o contato numa
// quintica replanejada a cada tick no mundo - o contato e um ponto do mundo,
// a bola; no referencial do corpo o alvo mudava a cada tick com a previsao e,
// perto do contato, um centimetro virava um salto da perna - chegando na
// velocidade do corpo mais a da batida (com um arco para nao raspar o chao),
// e depois do contato e o pe do clipe mais uma diferenca que some ate o pouso
// dele - o pe
// pousa onde e quando o clipe pousaria. Parado (o passo), o mesmo, com o pe
// do clipe parado no lugar dele.
class SwingTouch3D {
public:
    void begin(const TouchPlan3D& plan, int request, float now, const SwingLive3D& live, const ContactFeet3D& feet,
        const DribbleSettings3D& settings);
    // O plano refeito (antes da trava): o ponto e o instante do contato.
    void retarget(const TouchPlan3D& plan);
    // Avanca ate `now` (relogio). Devolve o alvo da perna (peso 0 quando o
    // balanco acabou).
    void update(float now, const SwingLive3D& live, const ContactFeet3D& feet, const DribbleSettings3D& settings,
        DribbleLeg3D& out);
    void stop() { m_active = false; }
    // Desiste do toque (o plano nao cabe mais): sem contato, o pe volta de
    // onde esta para o pe do clipe em pelo menos `follow` s.
    void abort(float now, const SwingLive3D& live, float follow);
    [[nodiscard]] bool active() const { return m_active; }
    [[nodiscard]] bool contacted() const { return m_contacted; }
    // O contato aconteceu neste update (a forca sai agora).
    [[nodiscard]] bool contactedNow() const { return m_contactedNow; }
    [[nodiscard]] int request() const { return m_request; }
    // O instante (relogio) em que o balanco comecou de fato (o compromisso).
    [[nodiscard]] float startTime() const { return m_start; }
    [[nodiscard]] int side() const { return m_plan.side; }
    [[nodiscard]] const TouchPlan3D& plan() const { return m_plan; }
    // O tornozelo no mundo agora (o ultimo alvo) e o da ida sem o arco nem o
    // armado (a quintica: de onde o pe "vem" para o plano refeito).
    [[nodiscard]] Vec3 ankle() const { return m_ankle; }
    [[nodiscard]] Vec3 approachAnkle() const { return m_contacted ? m_ankle : m_position; }
    [[nodiscard]] Quaternion orientation() const { return m_orientation; }

private:
    TouchPlan3D m_plan;
    bool m_active = false;
    bool m_contacted = false;
    bool m_contactedNow = false;
    int m_request = -1;
    float m_time = 0.0f;
    float m_start = 0.0f;
    // A aproximacao no mundo (o tornozelo, a velocidade e a aceleracao) e o
    // arco (m).
    Vec3 m_position;
    Vec3 m_velocity;
    Vec3 m_acceleration;
    float m_lift = 0.0f;
    // Depois do contato: a diferenca para o pe do clipe (posicao, velocidade e
    // aceleracao no contato) e o tempo ate o pouso.
    Vec3 m_offset;
    Vec3 m_offsetVelocity;
    Vec3 m_offsetAcceleration;
    float m_followSpan = 0.1f;
    // As orientacoes do pe: no comeco e no contato.
    Quaternion m_startOrientation;
    Quaternion m_contactOrientation;
    // A saida do ultimo update (mundo).
    Vec3 m_ankle;
    Vec3 m_ankleVelocity;
    Quaternion m_orientation;
    Vec3 m_knee { 1.0f, 0.0f, 0.0f };
    float m_lastUpdate = 0.0f;
    float m_weight = 0.0f;
    float m_lastWeightTime = 0.0f;
};

} // namespace MatterEngine
