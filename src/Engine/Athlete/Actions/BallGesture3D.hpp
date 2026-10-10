#pragma once

#include "Engine/Athlete/Actions/BallActionPlanner3D.hpp"

#include <algorithm>

namespace MatterEngine {

// O gesto da perna que joga (rework do manejo, 10/10/2026, Etapa 5): o pe sai
// de onde esta e passa pelas fases - armado (para tras e para cima, pela
// forca), batida (chega no ponto de contato no instante do plano COM
// velocidade no rumo pedido), acompanhamento (segue no mesmo rumo, freando;
// no chute, subindo) e volta (pousa no chao). Cada fase e um Hermite cubico
// do estado de agora (posicao e velocidade) ate o alvo dela: continuo, e o
// ponto de contato pode acompanhar a bola em correcoes pequenas depois do
// compromisso.
enum class BallGesturePhase3D : std::uint8_t { Idle, Windup, Strike, Follow, Return };

struct BallGestureSettings3D {
    // O armado: quanto o pe vai para tras e para cima do contato (m).
    float touchBack = 0.07f, touchBackPower = 0.14f;
    float touchUp = 0.035f, touchUpPower = 0.045f;
    float shotBack = 0.28f, shotBackPower = 0.22f;
    float shotUp = 0.12f, shotUpPower = 0.14f;
    // Com a sola (por cima da bola, trazendo-a) o pe vem de cima: esta parte
    // do recuo e pelo menos esta subida (m).
    float soleBackShare = 0.3f;
    float soleUp = 0.06f;
    // O armado ao alcance: o tornozelo a no maximo isto da perna do quadril.
    float windupReach = 0.90f;
    // A velocidade do pe no contato (m/s, no rumo pedido, alem da do corpo).
    float touchStrikeSpeed = 1.2f, touchStrikeSpeedPower = 3.5f;
    float shotStrikeSpeed = 8.0f, shotStrikeSpeedPower = 12.0f;
    // O acompanhamento: a distancia (m), a subida no chute (m) e o tempo (s).
    float touchFollow = 0.08f, touchFollowPower = 0.10f, touchFollowSeconds = 0.10f;
    float shotFollow = 0.25f, shotFollowPower = 0.30f, shotFollowUp = 0.18f, shotFollowSeconds = 0.14f;
    // A volta ao chao (s).
    float returnSeconds = 0.16f, returnSecondsPower = 0.08f;
    // O acompanhamento como arco da perna (o tornozelo) em volta do quadril:
    // quanto ela sobe (rad) alem do contato no toque e no chute (e o maximo
    // acima da horizontal no chute); no chute a perna estende ate legReach
    // dela (no toque fica como esta, ate isso).
    float touchFollowArc = 0.30f, touchFollowArcPower = 0.25f;
    float shotFollowArc = 0.55f, shotFollowArcPower = 0.45f;
    float shotFollowCeiling = 0.45f;
    float legReach = 0.97f;
    // O pouso ao alcance: o tornozelo a no maximo isto da perna (coxa +
    // canela) do quadril de repouso. (Em pe o tornozelo ja fica a 0,96 dela
    // abaixo do quadril: parado, o pe pousa no maximo ~0,2 m a frente, e a
    // acomodacao das pegadas o traz de volta.)
    float landReach = 0.975f;
    // Do contato ao arco: a velocidade do pe que o arco nao tem (a parte
    // radial, a do corpo) some neste tempo (s), sem o tranco de um tick.
    float followBlendSeconds = 0.08f;
    // O toque no balanco (correndo): a batida soma esta parte da do toque
    // parado ao pe do estilo, e o pe volta a ele em pelo menos isto (s),
    // ficando ate isto (s) depois do pouso dele (o pe do estilo ja parado no
    // chao, as pegadas travam o do gesto em cima dele).
    float swingStrikeShare = 0.5f;
    float swingMinimumFollow = 0.05f;
    float swingSettleSeconds = 0.05f;
    // As correcoes do contato depois do compromisso (m/s no maximo).
    float correctionSpeed = 2.0f;
    // A janela do contato: o pe pode encostar ate isto (s) depois do instante
    // planejado.
    float contactLate = 0.08f;
};

// O referencial do gesto: parado, o mundo. Correndo, o chute vai com o corpo
// e o toque com o pe do estilo no balanco dele (o pe segue a passada,
// desviado so o bastante para encontrar a bola, e volta a ela antes de
// pousar com ela). Andando, o gesto acaba no fim do acompanhamento e as
// pegadas pousam o pe no ritmo da passada.
enum class BallGestureFrame3D : std::uint8_t { World, Body, StyleFoot };

// O comeco de um gesto.
struct BallGestureStart3D {
    BallPlan3D plan;
    BallActionKind3D kind = BallActionKind3D::Dribble;
    float power = 0.5f;
    float lift = 0.0f;
    Vec3 bodyVelocity;
    // O pe que joga agora e o chao.
    Vec3 foot;
    Vec3 footVelocity;
    Quaternion footOrientation;
    float groundHeight = 0.0f;
    // O quadril da perna, o tornozelo no link do pe e o comprimento da perna
    // (coxa + canela): o acompanhamento e um arco em volta dele e o pouso
    // fica ao alcance.
    Vec3 hip;
    Vec3 ankleInFoot;
    float legLength = 0.83f;
    BallGestureFrame3D frame = BallGestureFrame3D::World;
    // Andando: o gesto acaba no fim do acompanhamento, com o pe no ar (as
    // pegadas o pousam no ritmo da passada) - em qualquer referencial (o
    // corte correndo vai no do mundo).
    bool moving = false;
    // No referencial do pe do estilo: onde ele esta, a velocidade dele e
    // quando ele pousa (s a partir de agora).
    Vec3 styleFoot;
    Vec3 styleFootVelocity;
    float styleTouchdown = -1.0f;
};

// O que o gesto le a cada tick.
struct BallGestureLive3D {
    // O quadril da perna (o centro do arco; a pelve desliza e abaixa no
    // gesto) e onde ele fica quando o corpo sai do gesto (o pouso fica ao
    // alcance dele).
    Vec3 hip;
    Vec3 restHip;
    Vec3 bodyVelocity;
    Vec3 styleFoot;
    Vec3 styleFootVelocity;
    Quaternion styleFootOrientation;
};

class BallGesture3D {
public:
    void start(const BallGestureStart3D& in, const BallFootBox3D& box, const BallGestureSettings3D& settings);
    // A bola mudou um pouco: o contato acompanha (limitado por tick).
    void retarget(Vec3 footPosition, Quaternion footOrientation, float deltaTime, const BallGestureSettings3D& settings);
    void update(float deltaTime, const BallGestureLive3D& live, const BallGestureSettings3D& settings);
    void stop() { m_phase = BallGesturePhase3D::Idle; }
    [[nodiscard]] bool active() const { return m_phase != BallGesturePhase3D::Idle; }
    [[nodiscard]] BallGesturePhase3D phase() const { return m_phase; }
    [[nodiscard]] int side() const { return m_side; }
    [[nodiscard]] Vec3 position() const { return m_position; }
    [[nodiscard]] Vec3 velocity() const { return m_velocity; }
    [[nodiscard]] Quaternion orientation() const { return m_orientation; }
    [[nodiscard]] float time() const { return m_time; }
    [[nodiscard]] float contactTime() const { return m_contactAt; }
    // Na volta: quanto falta para o pe pousar (s; -1 fora dela).
    [[nodiscard]] float returnLeft() const {
        return m_phase == BallGesturePhase3D::Return ? std::max(0.0f, m_returnAt - m_time) : -1.0f;
    }
    // O pe pode encostar agora (a batida, ate contactLate depois do instante).
    [[nodiscard]] bool inContactWindow(const BallGestureSettings3D& settings) const;
    // 0..1: o quanto a perna esta no gesto (sobe no armado, cai na volta).
    [[nodiscard]] float weight() const;
    [[nodiscard]] Vec3 push() const { return m_push; }

private:
    void enter(BallGesturePhase3D phase);
    void reachableLand();
    [[nodiscard]] Vec3 reachableWindup(Vec3 contact) const;
    [[nodiscard]] Vec3 arcAnkle() const;

    BallGesturePhase3D m_phase = BallGesturePhase3D::Idle;
    bool m_shot = false;
    bool m_moving = false;
    BallGestureFrame3D m_frameKind = BallGestureFrame3D::World;
    // O pe do estilo no comeco (o referencial dele) e a orientacao dele agora.
    Vec3 m_styleStart;
    Quaternion m_styleOrientation;
    int m_side = 0;
    float m_time = 0.0f;
    float m_windupAt = 0.0f, m_contactAt = 0.0f, m_followAt = 0.0f, m_returnAt = 0.0f;
    Vec3 m_position, m_velocity;
    Quaternion m_orientation;
    Vec3 m_push { 1.0f, 0.0f, 0.0f };
    // Os alvos: o contato no mundo (e a velocidade do pe nele, no
    // referencial do gesto), o armado e o acompanhamento do toque em relacao
    // ao contato (no referencial), o fim do acompanhamento e o pouso; as
    // orientacoes.
    Vec3 m_contact, m_contactVelocity, m_windupOffset, m_followOffset, m_follow, m_land;
    // O referencial do gesto: o quanto andou desde o comeco e a velocidade
    // dele (a do corpo andando; parado, zero).
    Vec3 m_frame;
    Vec3 m_frameVelocity;
    Quaternion m_windupOrientation, m_contactOrientation, m_landOrientation;
    // A orientacao no comeco da fase e o tempo dela.
    Quaternion m_phaseOrientation;
    float m_phaseStart = 0.0f;
    // O arco do acompanhamento: o quadril (no contato) e a velocidade do
    // corpo, o lado (fixo), o raio, o angulo e a velocidade dele.
    Vec3 m_hip;
    Vec3 m_bodyVelocity;
    float m_legLength = 0.83f;
    float m_landReach = 0.77f;
    Vec3 m_ankleInFoot;
    // O pouso pedido (embaixo do fim do arco) e o quadril de repouso; o
    // armado pedido e o alcance dele.
    Vec3 m_landWanted;
    Vec3 m_restHip;
    float m_windupReach = 0.79f;
    float m_arcLateral = 0.0f;
    float m_arcRadius = 0.8f;
    float m_arcRadiusEnd = 0.8f;
    float m_arcRadiusStart = 0.8f;
    float m_arcAngle = 0.0f;
    float m_arcRate = 0.0f;
    float m_arcRadiusRate = 0.0f;
    float m_arcLateralRate = 0.0f;
    float m_arcEnd = 0.0f;
    float m_followArc = 0.4f;
    float m_followCeiling = 0.45f;
    // A diferenca de velocidade no comeco do arco (some em m_followBlend s).
    Vec3 m_followSlip;
    float m_followBlend = 0.08f;
    void beginFollowArc();
};

} // namespace MatterEngine
