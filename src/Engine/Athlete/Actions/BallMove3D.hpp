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
// poucos, com o pe andando junto com o do clipe. A passada acelera um pouco
// para o pe do clipe chegar no balanco ao fim do gesto (BallPlay3D); com ele
// ainda apoiado la atras, o pe pousa a frente (o passo) e sai do chao junto
// com ele - voltar a ele dava o "chicote" para tras, e esperar o do clipe
// passar deixava a perna esticada com o corpo indo embora (usuario, 11/10).
// Sem o pe do clipe (no fisico), o pe pousa parado.
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
    // (Plano v4: perto dos da V3 - curtos e na cubica, eram um tranco; bem
    // mais longos, o gesto ficou mole, usuario 11/10. A quintica tira o
    // tranco.)
    float touchStrikeSeconds = 0.10f;
    float touchFollowSeconds = 0.07f;
    float touchLandSeconds = 0.11f;
    float shotStrikeSeconds = 0.08f;
    float shotFollowSeconds = 0.11f, shotFollowSecondsPower = 0.08f;
    float shotLandSeconds = 0.13f;
    // A volta ao pe do clipe (s): a diferenca some neste tempo; com o pe do
    // clipe no balanco, ate o pouso dele, no maximo rejoinMaximumSeconds.
    float touchRejoinSeconds = 0.14f;
    float shotRejoinSeconds = 0.16f;
    float rejoinMaximumSeconds = 0.30f;
    // Com o pe do clipe apoiado, a volta vai direto a ele so com ele a
    // frente (ou no maximo rejoinBehind atras, m) de onde o pe do gesto para
    // naturalmente (indo, ele para adiante: a velocidade dele vezes metade do
    // tempo da volta); senao, se ele so sai do chao depois do pouso do gesto
    // (o tempo dele vezes leavingShare, pela passada que acelera), o pe pousa
    // e fica apoiado ate o do clipe sair do chao - pelo menos
    // plantMinimumSeconds (s) e no maximo plantMaximumSeconds (s), ou a perna
    // esticando alem de plantReach; saindo antes, a volta vai pelo ar ao
    // balanco dele (pousando para sair logo, o pe pisava embaixo do corpo com
    // a perna reta).
    float rejoinBehind = 0.06f;
    float leavingShare = 1.5f;
    // (Sendo o pe do clipe o unico apoio, a volta so vai pelo ar ao balanco
    // dele saindo do chao em ate flightLeavingSeconds (s): mais tarde, o pe
    // pulava o apoio dele e o corpo flutuava sem pe no chao.)
    float flightLeavingSeconds = 0.06f;
    float plantMinimumSeconds = 0.03f;
    float plantMaximumSeconds = 0.35f;
    float plantReach = 0.97f;
    // Na volta, a perna no maximo tao esticada quanto a do clipe ou esta
    // fracao (o que for maior); a aceleracao da diferenca no comeco, no
    // maximo isto (m/s2).
    float rejoinReach = 0.95f;
    float rejoinAccelerationMaximum = 60.0f;
    // Apoiado atras do quadril, o calcanhar sobe em volta da base dos dedos
    // (a saida do passo) para a perna nao passar de plantReach: ate
    // plantRollMaximum (rad), no maximo a plantRollRate (rad/s).
    float plantRollMaximum = 0.9f;
    float plantRollRate = 10.0f;
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
    float touchTravelSpeed = 3.5f;
    // Andando, com o pe do clipe no balanco, o gesto (ate o fim do
    // acompanhamento) termina ate fitTolerance (s) depois do pouso dele - o
    // tempo todo na mesma proporcao, nunca menos que fitMinimumSeconds (s):
    // mais longo, o pe pulava o apoio dele e o corpo flutuava (o chute nao).
    float fitTolerance = 0.03f;
    float fitMinimumSeconds = 0.13f;
    // O pe ja indo no rumo do toque mais depressa que a passagem: ela vai
    // adiante o que ele anda freando nisto (m/s2) e chega antes, ate
    // strikeMinimumSeconds (s).
    float strikeBrake = 20.0f;
    float strikeMinimumSeconds = 0.10f;
    // A velocidade do pe contra o corpo no comeco entra nesta parte (o pe
    // apoiado anda para tras na velocidade do corpo).
    float startVelocityShare = 0.3f;
    float shotTravelSpeed = 5.5f;
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
    // (Entrando nesta parte do tempo ate a passagem.)
    float moveGapEntry = 0.6f;
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
    // Andando, o pe do clipe daquele lado no balanco: quanto falta para ele
    // pousar (s; -1 apoiado ou parado).
    float clipTouchdownSeconds = -1.0f;
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
    // Apoiado, quanto falta para ele sair do chao e o tempo do balanco (s);
    // e o outro pe do clipe apoiado (sem ele, este e o unico apoio).
    float clipSecondsToLiftoff = -1.0f;
    float clipSwingSeconds = -1.0f;
    bool otherPlanted = true;
    // A locomocao no ciclo da passada.
    bool cycling = false;
};

enum class BallMovePhase3D : std::uint8_t { Idle, Strike, Follow, Land, Plant, Rejoin };

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
    // O que falta do acompanhamento (s; fora do gesto, 0) e se o pe pousa ou
    // esta apoiado esperando o do clipe sair do chao.
    [[nodiscard]] float followTimeLeft() const { return gesturing() ? std::max(0.0f, m_followAt - m_time) : 0.0f; }
    [[nodiscard]] bool planting() const
    {
        return m_plantAfterLand && (m_phase == BallMovePhase3D::Land || m_phase == BallMovePhase3D::Plant);
    }
    // O rumo do toque no mundo (no plano).
    [[nodiscard]] Vec3 push() const { return m_push; }
    // Acabou neste tick: parado no pouso (sem o pe do clipe) ou ja junto do
    // pe do clipe.
    [[nodiscard]] bool landed() const { return m_landed; }
    [[nodiscard]] bool joined() const { return m_joined; }

private:
    // As trocas de fase: ahead e o que falta do tick (live e o fim dele).
    void beginReturn(const BallMoveLive3D& live, const BallMoveSettings3D& settings, float ahead);
    // O tempo da volta: o de sempre; com o pe do clipe no balanco (ou saindo
    // do chao antes do pouso do gesto), ate o pouso dele.
    [[nodiscard]] float rejoinSpan(const BallMoveLive3D& live, const BallMoveSettings3D& settings) const;
    // O pe do clipe apoiado sai do chao antes do pouso do gesto.
    [[nodiscard]] bool clipLeavingSoon(const BallMoveLive3D& live, const BallMoveSettings3D& settings) const;
    // O pouso ao alcance do quadril de agora (previsto para o fim dele).
    void clampLandTarget(const BallMoveLive3D& live, const BallMoveSettings3D& settings, float remaining);
    void beginLand(const BallMoveLive3D& live, const BallMoveSettings3D& settings, float ahead);
    void beginRejoin(const BallMoveLive3D& live, const BallMoveSettings3D& settings, float ahead);
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
    // A volta: a diferenca para o pe do clipe (posicao, velocidade e
    // aceleracao no comeco) e o tempo dela.
    Vec3 m_offset, m_offsetVelocity, m_offsetAcceleration;
    float m_rejoinSpan = 0.14f;
    // O limite do alcance no comeco da volta (fracao da perna) e a altura do
    // arco dela (do chao a um pe do clipe tambem no chao).
    float m_rejoinLimitStart = 0.95f;
    float m_rejoinLift = 0.0f;
    // O movimento pedido mudou: a volta vai direto, no tempo dela.
    bool m_hurried = false;
    // O pouso fica apoiado ate o pe do clipe sair do chao; desde quando, o
    // tornozelo e o pe planos do pouso e o quanto o pe rolou na ponta.
    bool m_plantAfterLand = false;
    float m_plantStart = 0.0f;
    Vec3 m_plantAnkle;
    Quaternion m_plantOrientation;
    float m_plantRoll = 0.0f;
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
