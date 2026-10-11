#pragma once

#include "Engine/Athlete/Ball/BallOutcome3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

namespace MatterEngine {

// Os movimentos padronizados com a bola (plano v3, pedido do usuario de 10/10
// noite: "movimentos padroes, das oito direcoes para as oito direcoes ... e se
// esquece essa merda de fazer o pe ir ate a bola"). O pe nunca procura a
// bola: cada situacao tem o mesmo movimento, rapido e definido, desenhado no
// referencial do corpo (a pelve no chao, olhando para a frente do corpo) - se
// o corpo anda, o movimento anda junto; nada fica preso no mundo nem e
// arrastado. So o pouso e no mundo: o pe para no chao onde a passada
// continua (andando, a frente no rumo do movimento; de lado, para o lado).
//
// Quem faz cada toque (a regra do usuario: a perna daquele lado):
//  - para a direita, frente-direita e tras-direita: a perna direita (fora,
//    fora, sola); o mesmo espelhado para a esquerda;
//  - para a frente (peito do pe) e para tras (sola): a perna do lado da bola;
//  - o chute: a perna do lado da bola, sempre o mesmo chute para a frente; a
//    intensidade muda a velocidade e a finalizacao (sem recuar a perna).
enum class BallMoveTouch3D : std::uint8_t { Front, FrontOut, Out, BackOut, Back, Shot };

struct BallMoveChoice3D {
    int side = 1;
    BallMoveTouch3D touch = BallMoveTouch3D::Front;
    // O setor do toque contra a frente do corpo (0 frente, 1 frente-esquerda,
    // 2 esquerda ... 7 frente-direita).
    int sector = 0;
    // A parte do pe (a da conta da forca: so a sola muda o toque).
    BallTouchSurface3D surface = BallTouchSurface3D::Laces;
};

// O movimento para um pedido: o rumo do toque (no mundo; o chute vai sempre
// para a frente do corpo), a frente do corpo e de que lado dele esta a bola
// (m, + esquerda; perto do meio, a perna livre `freeSide`, ou a direita).
[[nodiscard]] BallMoveChoice3D chooseBallMove3D(BallActionKind3D kind, Vec3 touchDirection, float facingYaw,
    float ballLateral, int freeSide = -1);

struct BallMoveSettings3D {
    // Os tempos (s): ate a passagem pela bola, o acompanhamento e o pouso; no
    // chute, a batida e o acompanhamento crescem com a intensidade.
    float touchStrikeSeconds = 0.09f;
    float touchFollowSeconds = 0.06f;
    float touchLandSeconds = 0.10f;
    float shotStrikeSeconds = 0.07f;
    float shotFollowSeconds = 0.10f, shotFollowSecondsPower = 0.08f;
    float shotLandSeconds = 0.12f;
    // A velocidade do pe na passagem (m/s, no referencial do corpo): o toque
    // e o chute, mais a parte pela intensidade.
    float touchStrikeSpeed = 1.8f, touchStrikeSpeedPower = 1.6f;
    float shotStrikeSpeed = 5.0f, shotStrikeSpeedPower = 6.0f;
    // O chute: a finalizacao sobe ate isto acima do chao (m) e vai a frente
    // (m), mais a parte pela intensidade.
    float shotFollowHeight = 0.22f, shotFollowHeightPower = 0.30f;
    float shotFollowAhead = 0.42f, shotFollowAheadPower = 0.18f;
    // A finalizacao do chute como arco em volta do quadril: a perna a esta
    // fracao do comprimento e ate este angulo a partir da vertical para baixo
    // (rad), mais a parte pela intensidade.
    float shotArcReach = 0.95f;
    float shotArcEnd = 0.9f, shotArcEndPower = 0.6f;
    // O pe ate a passagem anda no maximo isto em media (m/s, no referencial
    // do corpo): saindo de longe, a passagem demora mais. E sobe ate isto
    // acima do tornozelo plano no meio do caminho (m).
    float touchTravelSpeed = 4.0f;
    // A velocidade do pe contra o corpo no comeco entra nesta parte (o pe
    // apoiado anda para tras na velocidade do corpo).
    float startVelocityShare = 0.3f;
    float shotTravelSpeed = 6.0f;
    float liftHeight = 0.10f;
    // Andando, o pouso vai a frente no rumo do movimento: este tempo vezes a
    // velocidade (s), ate landLeadMaximum (m); e nunca mais longe do quadril
    // que landReach (m, no plano).
    float landLeadSeconds = 0.07f;
    float landLeadMaximum = 0.30f;
    float landReach = 0.38f;
    // O pe que pousa fica do lado dele: pelo menos isto para fora do meio da
    // pelve (m).
    float landSideMinimum = 0.08f;
    // E pelo menos isto para o lado dele contra o pe de apoio (m).
    float landGap = 0.14f;
    // O chute sai do chao subindo pelo menos a isto (m/s).
    float shotLiftSpeed = 1.2f;
    // O fim do acompanhamento ja anda esta parte da velocidade media ate o
    // pouso (sem parar e voltar).
    float followIntoLand = 0.5f;
    // O tornozelo a no maximo esta fracao da perna do quadril (esticada, a
    // perna travava reta e o IK dava o tranco).
    float reach = 0.90f;
    // O pouso: o tornozelo no chao a no maximo esta fracao da perna do
    // quadril (a altura dele decide o quanto vai para o lado).
    float landLegReach = 0.97f;
};

// O comeco de um movimento: a escolha, a intensidade, o pe agora (origem do
// link, velocidade e orientacao, no mundo), o corpo (a pelve no chao, o rumo,
// a velocidade, o quadril da perna) e a geometria do pe (o tornozelo no link,
// a orientacao plana olhando para +x e a altura do tornozelo com o pe plano no
// chao).
struct BallMoveStart3D {
    BallMoveChoice3D choice;
    float power = 0.5f;
    Vec3 foot;
    Vec3 footVelocity;
    Quaternion footOrientation;
    Vec3 pelvisGround;
    float facingYaw = 0.0f;
    Vec3 bodyVelocity;
    Vec3 ankleInFoot;
    Quaternion flatFoot;
    float ankleHeight = 0.08f;
    // O pe estava apoiado (anda para tras contra o corpo: so uma parte dessa
    // velocidade entra; no ar, ela inteira).
    bool footPlanted = true;
    // O comprimento da perna (coxa + canela): o tornozelo nunca alem de
    // BallMoveSettings3D::reach dela do quadril.
    float legLength = 0.86f;
    // Andando, o movimento inteiro dura isto (s; o pouso do pe no clipe): os
    // tempos dele encolhem ou crescem juntos. Negativo: os dos ajustes.
    float totalSeconds = -1.0f;
};

// O que o movimento le a cada tick: o corpo agora e o pe de apoio (o pouso
// fica do lado certo dele).
struct BallMoveLive3D {
    Vec3 pelvisGround;
    float facingYaw = 0.0f;
    Vec3 bodyVelocity;
    Vec3 hip;
    Vec3 support;
    bool supportKnown = false;
};

enum class BallMovePhase3D : std::uint8_t { Idle, Strike, Follow, Land };

class BallMove3D {
public:
    void start(const BallMoveStart3D& in, const BallMoveSettings3D& settings);
    void update(float deltaTime, const BallMoveLive3D& live, const BallMoveSettings3D& settings);
    void stop() { m_phase = BallMovePhase3D::Idle; }
    // Vai para o pouso ja (o movimento pedido mudou depois da forca).
    void hurry(const BallMoveLive3D& live, const BallMoveSettings3D& settings);
    [[nodiscard]] bool active() const { return m_phase != BallMovePhase3D::Idle; }
    [[nodiscard]] BallMovePhase3D phase() const { return m_phase; }
    [[nodiscard]] int side() const { return m_choice.side; }
    [[nodiscard]] const BallMoveChoice3D& choice() const { return m_choice; }
    // O pe (a origem do link) e o tornozelo no mundo, a velocidade do pe e a
    // orientacao dele.
    [[nodiscard]] Vec3 foot() const { return m_foot; }
    [[nodiscard]] Vec3 ankle() const { return m_foot + m_orientation.rotate(m_ankleInFoot); }
    [[nodiscard]] Vec3 velocity() const { return m_velocity; }
    [[nodiscard]] Quaternion orientation() const { return m_orientation; }
    // O tempo desde o comeco e o que falta ate o pe pousar (s).
    [[nodiscard]] float time() const { return m_time; }
    [[nodiscard]] float timeToLanding() const { return active() ? std::max(0.0f, m_landAt - m_time) : 0.0f; }
    // O rumo do toque no mundo (no plano).
    [[nodiscard]] Vec3 push() const { return m_push; }
    // Acabou de pousar neste tick.
    [[nodiscard]] bool landed() const { return m_landed; }

private:
    void beginLand(const BallMoveLive3D& live, const BallMoveSettings3D& settings);
    [[nodiscard]] Quaternion localOrientation(float yaw, float pitch, float roll) const;

    BallMoveChoice3D m_choice;
    BallMovePhase3D m_phase = BallMovePhase3D::Idle;
    float m_power = 0.5f;
    float m_time = 0.0f;
    float m_strikeAt = 0.0f, m_followAt = 0.0f, m_landAt = 0.0f;
    float m_phaseStart = 0.0f;
    // No referencial do corpo (x frente, y esquerda, z cima, a partir da pelve
    // no chao): o tornozelo e a velocidade dele; a passagem, a velocidade nela
    // e o fim do acompanhamento.
    Vec3 m_local, m_localVelocity;
    Vec3 m_strike, m_strikeVelocity, m_follow;
    // O ponto do alto da sola (o pe passa por cima da bola, nos toques para
    // tras) e quando.
    bool m_over = false;
    Vec3 m_overPoint;
    float m_overAt = 0.0f;
    // No mundo, o pouso: o tornozelo agora, a velocidade, o alvo; e o pouso
    // parado no referencial do corpo (o acompanhamento ja vai para ele).
    Vec3 m_worldAnkle, m_worldVelocity, m_landTarget;
    Vec3 m_landLocal;
    // As orientacoes do pe (no referencial do corpo: guinada, arfagem e
    // rolagem) na passagem e no pouso, e a do comeco da fase.
    std::array<float, 3> m_strikeAngles {}, m_landAngles {};
    Quaternion m_phaseOrientation;
    Quaternion m_flat;
    Vec3 m_ankleInFoot;
    float m_ankleHeight = 0.08f;
    float m_legLength = 0.86f;
    // O arco do chute (em volta do quadril, no referencial do corpo): ja
    // comecou, o lado, o raio, o angulo e a velocidade dele, o fim.
    bool m_arcLive = false;
    float m_arcSide = 0.0f, m_arcRadius = 0.8f, m_arcAngle = 0.0f, m_arcRate = 0.0f, m_arcEnd = 1.0f;
    Vec3 m_foot, m_velocity;
    Quaternion m_orientation;
    Vec3 m_push { 1.0f, 0.0f, 0.0f };
    float m_facing = 0.0f;
    bool m_landed = false;
};

} // namespace MatterEngine
