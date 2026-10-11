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
// arrastado.
//
// A volta (plano v4, pedido do usuario de 11/10: "ele acaba o movimento cedo
// demais para tentar fazer o pe/perna voltar ao movimento da locomocao, e nao
// tenta adaptar ambos para ser harmonico a volta"): o movimento termina
// inteiro e o pe se junta ao pe do clipe - a diferenca entre os dois some aos
// poucos, com o pe andando junto com o do clipe. O pe do clipe atras dele
// (andando, ainda apoiado la atras ou vindo no balanco), ele pousa a frente
// e espera ali, apoiado, ate o do clipe chegar - nunca volta para tras.
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
    // (Plano v4: um pouco mais longos - curtos, o gesto era um tranco.)
    float touchStrikeSeconds = 0.12f;
    float touchFollowSeconds = 0.08f;
    float touchLandSeconds = 0.14f;
    float shotStrikeSeconds = 0.09f;
    float shotFollowSeconds = 0.12f, shotFollowSecondsPower = 0.08f;
    float shotLandSeconds = 0.15f;
    // A volta ao pe do clipe (s): a diferenca some neste tempo; com o pe do
    // clipe no ar, ate o pouso dele, no maximo rejoinMaximumSeconds.
    float touchRejoinSeconds = 0.18f;
    float shotRejoinSeconds = 0.22f;
    float rejoinMaximumSeconds = 0.35f;
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
    float touchTravelSpeed = 2.8f;
    // O pe ja indo no rumo do toque mais depressa que a passagem: ela vai
    // adiante o que ele anda freando nisto (m/s2) e chega antes, ate
    // strikeMinimumSeconds (s).
    float strikeBrake = 20.0f;
    float strikeMinimumSeconds = 0.10f;
    // A velocidade do pe contra o corpo no comeco entra nesta parte (o pe
    // apoiado anda para tras na velocidade do corpo).
    float startVelocityShare = 0.3f;
    float shotTravelSpeed = 4.5f;
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
    // No gesto, o pe fica do lado dele contra o apoio: pelo menos moveGap de
    // lado (m) com os dois a ate moveGapZone a frente ou atras um do outro
    // (m), sumindo em moveGapFade (m).
    float moveGap = 0.15f;
    float moveGapZone = 0.18f;
    float moveGapFade = 0.15f;
    // O chute sai do chao subindo pelo menos a isto (m/s).
    float shotLiftSpeed = 1.2f;
    // O fim do acompanhamento ja anda esta parte da velocidade media ate o
    // pouso (sem parar e voltar).
    float followIntoLand = 0.5f;
    // O tornozelo a no maximo esta fracao da perna do quadril (esticada, a
    // perna travava reta e o IK dava o tranco), cedendo aos poucos na ultima
    // reachBand dela.
    float reach = 0.90f;
    float reachBand = 0.06f;
    // A sola (o calcanhar e a base dos dedos) a pelo menos isto do chao no
    // gesto e no comeco do pouso (m).
    float soleClearance = 0.025f;
    // O pouso: o tornozelo no chao a no maximo esta fracao da perna do
    // quadril (a altura dele decide o quanto vai para o lado).
    float landLegReach = 0.97f;
    // A volta so vai ao pe do clipe com ele (onde estara no fim dela) no
    // maximo isto atras do pe do gesto no rumo do movimento (m); senao o pe
    // pousa e espera. Apoiado esperando, o pe sai assim mesmo com mais que
    // holdBehindMaximum atras do quadril (m), sem alcancar nem rolando na
    // ponta ou depois de holdMaximumSeconds.
    float rejoinBehind = 0.05f;
    float holdBehindMaximum = 0.30f;
    float holdMaximumSeconds = 0.6f;
    // Esperando, a perna esticando alem desta fracao, o pe rola na ponta (ate
    // holdRollMaximum rad, no maximo a holdRollRate rad/s); nem assim, ele
    // sai.
    float holdReach = 0.97f;
    float holdRollMaximum = 0.9f;
    float holdRollRate = 10.0f;
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
    Vec3 hip;
    Vec3 ankleInFoot;
    Quaternion flatFoot;
    float ankleHeight = 0.08f;
    // O calcanhar e a base dos dedos no link do pe (a sola).
    Vec3 heel;
    Vec3 toe;
    // O pe estava apoiado (anda para tras contra o corpo: so uma parte dessa
    // velocidade entra; no ar, ela inteira).
    bool footPlanted = true;
    // O comprimento da perna (coxa + canela): o tornozelo nunca alem de
    // BallMoveSettings3D::reach dela do quadril.
    float legLength = 0.86f;
};

// O que o movimento le a cada tick: o corpo agora, o pe de apoio (o pouso
// fica do lado certo dele) e o pe do clipe daquele lado (para onde a volta
// vai).
struct BallMoveLive3D {
    Vec3 pelvisGround;
    float facingYaw = 0.0f;
    Vec3 bodyVelocity;
    Vec3 hip;
    Vec3 support;
    bool supportKnown = false;
    // O pe do clipe: o tornozelo, a velocidade e a aceleracao dele (mundo), a
    // orientacao do pe, apoiado e quanto falta para pousar (s, no ar). Sem ele, o movimento
    // termina parado no pouso.
    bool clipKnown = false;
    Vec3 clipAnkle;
    Vec3 clipVelocity;
    Vec3 clipAcceleration;
    Quaternion clipOrientation;
    bool clipPlanted = true;
    float clipSecondsToTouchdown = -1.0f;
    // A locomocao no ciclo da passada (parado, a volta vai direto ao pe do
    // clipe) e o outro pe apoiado nela.
    bool cycling = false;
    bool supportPlanted = true;
};

enum class BallMovePhase3D : std::uint8_t { Idle, Strike, Follow, Land, Hold, Rejoin };

class BallMove3D {
public:
    void start(const BallMoveStart3D& in, const BallMoveSettings3D& settings);
    void update(float deltaTime, const BallMoveLive3D& live, const BallMoveSettings3D& settings);
    void stop() { m_phase = BallMovePhase3D::Idle; }
    // Vai para a volta ja (o movimento pedido mudou depois da forca).
    void hurry(float deltaTime, const BallMoveLive3D& live, const BallMoveSettings3D& settings);
    [[nodiscard]] bool active() const { return m_phase != BallMovePhase3D::Idle; }
    // O gesto em si (a passagem e o acompanhamento): um pedido novo espera.
    [[nodiscard]] bool gesturing() const
    {
        return m_phase == BallMovePhase3D::Strike || m_phase == BallMovePhase3D::Follow;
    }
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
    [[nodiscard]] float timeToLanding() const
    {
        return m_phase == BallMovePhase3D::Strike || m_phase == BallMovePhase3D::Follow
                || m_phase == BallMovePhase3D::Land
            ? std::max(0.0f, m_landAt - m_time)
            : 0.0f;
    }
    // Na volta: o quanto ja foi (0..1).
    [[nodiscard]] float rejoinProgress() const;
    // O rumo do toque no mundo (no plano).
    [[nodiscard]] Vec3 push() const { return m_push; }
    // Acabou neste tick: parado no pouso (sem o pe do clipe) ou ja junto do
    // pe do clipe.
    [[nodiscard]] bool landed() const { return m_landed; }
    [[nodiscard]] bool joined() const { return m_joined; }

private:
    // As trocas de fase: ahead e o que falta do tick (live e o fim dele).
    void beginReturn(const BallMoveLive3D& live, const BallMoveSettings3D& settings, float ahead);
    void beginLand(const BallMoveLive3D& live, const BallMoveSettings3D& settings, float ahead);
    void beginRejoin(const BallMoveLive3D& live, const BallMoveSettings3D& settings, float ahead);
    // A volta pode comecar com o pe em from (grounded: apoiado esperando).
    [[nodiscard]] bool rejoinReady(const BallMoveLive3D& live, Vec3 from, bool grounded,
        const BallMoveSettings3D& settings) const;
    // O tornozelo e a velocidade no mundo pelo estado no referencial do corpo
    // (com o chao e o alcance); a orientacao do pe na fase de agora.
    void worldState(const BallMoveLive3D& live, const BallMoveSettings3D& settings, Quaternion orientation,
        Vec3& position, Vec3& velocity) const;
    // O quanto o tornozelo sobe para a sola ficar a soleClearance do chao.
    [[nodiscard]] float soleLift(Vec3 ankle, Quaternion orientation, float ground,
        const BallMoveSettings3D& settings) const;
    [[nodiscard]] Quaternion orientationNow(const BallMoveLive3D& live) const;
    [[nodiscard]] Quaternion localOrientation(float yaw, float pitch, float roll) const;

    BallMoveChoice3D m_choice;
    BallMovePhase3D m_phase = BallMovePhase3D::Idle;
    float m_power = 0.5f;
    float m_time = 0.0f;
    float m_strikeAt = 0.0f, m_followAt = 0.0f, m_landAt = 0.0f;
    float m_phaseStart = 0.0f;
    // No referencial do corpo (x frente, y esquerda, z cima, a partir da pelve
    // no chao): o tornozelo, a velocidade e a aceleracao dele; a passagem, a
    // velocidade nela e o fim do acompanhamento.
    Vec3 m_local, m_localVelocity, m_localAcceleration;
    Vec3 m_startLocal;
    // O alcance do pe no comeco (fracao da perna).
    float m_startReach = 0.0f;
    Vec3 m_strike, m_strikeVelocity, m_follow;
    // O ponto do alto da sola (o pe passa por cima da bola, nos toques para
    // tras) e quando.
    bool m_over = false;
    Vec3 m_overPoint;
    float m_overAt = 0.0f;
    // No mundo, o pouso: o tornozelo agora, a velocidade, a aceleracao, o
    // alvo; e o pouso parado no referencial do corpo (o acompanhamento ja vai
    // para ele).
    Vec3 m_worldAnkle, m_worldVelocity, m_worldAcceleration, m_landTarget;
    Vec3 m_landLocal;
    // A volta: depois do pouso ela espera o pe do clipe (o pouso tinha o
    // pe do clipe), desde quando, e o tempo dela.
    bool m_waitForClip = false;
    // O movimento pedido mudou: a volta nao espera o pouso do pe do clipe.
    bool m_hurried = false;
    float m_holdStart = 0.0f;
    // Apoiado esperando: o tornozelo e o pe planos do pouso (o pe rola na
    // ponta a partir deles).
    Vec3 m_holdAnkle;
    Quaternion m_holdOrientation;
    float m_holdRoll = 0.0f;
    // O outro pe ja saiu do chao durante a espera.
    bool m_supportLifted = false;
    Vec3 m_offset, m_offsetVelocity, m_offsetAcceleration;
    float m_rejoinSpan = 0.18f;
    // A altura do arco da volta saindo do chao para um pe do clipe apoiado.
    float m_rejoinLift = 0.0f;
    // As orientacoes do pe (no referencial do corpo: guinada, arfagem e
    // rolagem) na passagem e no pouso, e a do comeco da fase.
    std::array<float, 3> m_strikeAngles {}, m_landAngles {};
    Quaternion m_phaseOrientation;
    Quaternion m_flat;
    Vec3 m_ankleInFoot, m_heel, m_toe;
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
    bool m_joined = false;
};

} // namespace MatterEngine
