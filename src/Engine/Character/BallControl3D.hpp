#pragma once

#include "Engine/Athlete/Ball/BallOutcome3D.hpp"
#include "Engine/Character/BallGesturePose3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstdint>

namespace MatterEngine {

// Manejo de bola (rework de 06/10 e V3 de 06/10 noite, pedidos do usuario):
// drible (cada toque e um pedido, para onde ele quer ir) e chute (para onde
// ele olha), com o movimento de um jogador de verdade (BallGesturePose3D).
//
//  - Area: da cintura ate a base dos pes, num raio de uma perna esticada em
//    volta da pelve. O jogador NUNCA e levado ate a bola.
//  - Pedido (V3): guardado ate RequestBuffer (0,25 s). Prevendo o encontro
//    do corpo com a bola (os dois andando), o gesto comeca quando da para o
//    contato visual coincidir com a chegada dela; cedo demais, espera;
//    passado o tempo, e descartado - se a bola nunca esteve na area.
//  - Garantia (06/10 noite, pedido do usuario: "nunca pode ocorrer de eu dar
//    o input quando a bola estiver dentro da area e a acao nao ocorrer"): o
//    pedido com a bola dentro da area (no pedido ou enquanto guardado) sempre
//    tem a forca - correndo, de lado, levantando, o que for. Sai quando der o
//    gesto; no fim do tempo guardado ou com a bola saindo da area, sai ja: o
//    gesto ate a bola, ou - impossivel - o gesto numa bola imaginada logo a
//    frente (so o visual); sem gesto nenhum (o corpo baixo), so a forca.
//  - Contato (09/10, pedido do usuario: "o contato so ocorrer quando a
//    hitbox do pe conseguir contatar a bola", maior que o pe para ajudar a
//    jogabilidade): a forca sai quando a caixa do pe aumentada
//    (hitboxMargin) encosta na bola no gesto, vindo para ela - varrida entre
//    um tick e outro. O rumo da bola e sempre o da intencao (as teclas no
//    toque, a mira no chute), nunca o do pe. O pedido sem gesto que alcance a
//    bola e descartado (nada de gesto imaginado nem de forca sem pe). Com
//    hitboxContact falso, o contato da V3: a forca numa fracao
//    (contactTimingFactor) do tempo ate o contato, com a garantia.
//  - Pernas (V3): nunca se cruzam. A perna do lado da bola e a do menor
//    caminho; o contato que obrigaria cruzar fica do lado dela (a bola e
//    tocada "de perto" - a forca vem do tempo, nao do encosto).
//  - Superficie pela direcao em relacao ao corpo: o lado de dentro ou o de
//    fora do pe (o peito do pe so no chute e no toque correndo para a
//    frente), a sola e o calcanhar atras; a bola atras tocada para o lado:
//    a perna vai para tras e para fora, o pe de ponta para baixo (V3).
//  - Bola um pouco atras do corpo: o gesto de frente, como se ela estivesse
//    logo a frente dele (V3).
//  - O gesto: arma (perna para tras, joelho dobrado, pela forca), desdobra
//    ate a bola e acompanha; quadril, cintura e coluna giram, o corpo inclina
//    para o lado contrario da perna e os bracos equilibram.
//  - Andando, o pe que esta no balanco (ou o proximo) faz o gesto no ritmo
//    da passada.
//  - O resultado na bola (velocidade e giro) e o modelo do golpe com atrito
//    (keepBallMotion), aprovado pelo usuario; desde 06/10 noite o toque leva
//    a velocidade que o corpo vai ter (a pedida), empurra mais correndo e o
//    corpo indo para o outro lado empresta forca (a bola volta de verdade).
// (BallActionKind3D, BallTouchSurface3D e a forca na bola moraram aqui; sao
// do manejo novo agora - Engine/Athlete/Ball/BallOutcome3D.hpp.)
enum class BallControlPhase3D : std::uint8_t {
    Idle,
    // Pedido aceito (ou guardado), esperando: o pe escolhido sair do chao
    // (andando) ou a bola ficar ao alcance do gesto (o pedido guardado).
    Waiting,
    // A perna arma.
    Windup,
    // A perna indo a bola (a batida).
    Reaching,
    // Depois do contato: acompanha e volta.
    FollowThrough
};

using BallForceSettings3D = BallOutcomeSettings3D;
// Os padroes (com os T_* do ambiente).
[[nodiscard]] BallForceSettings3D defaultBallForceSettings3D();

struct BallControlInput3D {
    // Pedido (borda): o tipo, a forca (0..1, a barra) e a direcao no plano
    // (drible: a das teclas, ou a do olhar sem tecla; chute: a do olhar).
    BallActionKind3D request = BallActionKind3D::None;
    float power = 0.0f;
    Vec3 direction { 1.0f, 0.0f, 0.0f };
    // Olhar (o chute).
    float lookYawRadians = 0.0f;
    // Chute: o gesto do mouse logo antes de soltar (-1..1). Para cima/baixo a
    // altura (+ bola alta, - rasteira; 0 o padrao, um pouco para cima); para
    // os lados o efeito (+ curva para a direita, - para a esquerda).
    float shotLift = 0.0f;
    float shotCurve = 0.0f;
    // Drible: a velocidade exata da bola no toque (m/s; 0 amortece). Negativa
    // usa a da forca (o jogador). O manejo automatico usa esta.
    float touchSpeed = -1.0f;
    // Quando a forca sai (V3): a fracao do tempo estimado ate o pe chegar na
    // bola (0 na hora, 1 no contato visual). Negativa: o padrao (1,0;
    // T_BallContactTiming).
    float contactTimingFactor = -1.0f;
    // A forca na bola (o toque e o chute); nula, os padroes.
    const BallForceSettings3D* force = nullptr;
    // O contato pela hitbox do pe (09/10, o padrao): a forca so quando a
    // caixa do pe aumentada encosta na bola. Falso: pelo tempo (V3, com a
    // garantia do pedido com a bola na area).
    bool hitboxContact = true;
    // Sem gesto (laboratorio, 07/10, para testar a forca isolada): o pedido so
    // aplica a forca - na hora, com a bola na area (fora dela, quando entrar
    // no tempo guardado) -, sem perna, sem espera do pe e sem chute desenhado.
    bool forceOnly = false;
    // A bola.
    bool ballValid = false;
    Vec3 ballPosition;
    Vec3 ballVelocity;
    float ballRadius = 0.11f;
    float ballMass = 0.43f;
    // O corpo: estado (a pose do tick anterior), rumo e o chao.
    const RagdollProfile3D* profile = nullptr;
    const RagdollState3D* state = nullptr;
    float facingYawRadians = 0.0f;
    // O rumo do corpo agora (valido com bodyYawValid - senao o do olhar).
    float bodyYawRadians = 0.0f;
    bool bodyYawValid = false;
    float groundHeight = 0.0f;
    // Velocidade pedida pelo jogador (m/s, no plano).
    Vec3 requestedVelocity;
    // A velocidade do corpo pela locomocao (valida com bodyVelocityValid -
    // senao a da pelve). Corpo animado: a da capsula (a da pelve tem o
    // balanco da passada e o proprio gesto, a pelve indo ao apoio - o toque
    // levava a bola rapido demais). Fisico: a da pelve (a capsula vai a
    // frente do corpo que atrasa).
    Vec3 bodyVelocity;
    bool bodyVelocityValid = false;
    // A passada (telemetria da locomocao): progresso do balanco de cada pe
    // (-1 apoiado), se esta plantado, onde e daqui a quanto pousa.
    std::array<float, 2> swingProgress { -1.0f, -1.0f };
    std::array<bool, 2> footPlanted { true, true };
    std::array<Vec3, 2> footLanding {};
    std::array<float, 2> secondsToTouchdown { -1.0f, -1.0f };
    // Parado em pe (sem marcha).
    bool standing = false;
    // O corpo e a animacao (locomocao padrao): o gesto vai inteiro pela pose
    // (BallGesturePose3D); no fisico, os pes pelo planejador (foot/script).
    bool animatedBody = false;
    float deltaTime = 1.0f / 120.0f;
};

// Toque parado no modo fisico: o manejo dirige um pe direto. O planejador o
// tira do chao e o segue (swing); com swing falso ele pousa no ponto.
// supportShift leva a pelve essa fracao do caminho ate o outro pe.
struct BallFootScript3D {
    bool active = false;
    int side = 0;
    bool swing = false;
    Vec3 position;
    Quaternion orientation;
    float supportShift = 0.0f;
};

// O pe de apoio pousa num ponto (o pouso desse pe vai ate ele, com peso).
struct BallPlant3D {
    bool active = false;
    int side = 0;
    Vec3 position;
    float weight = 0.0f;
};

// A pose de cima do chute antiga (bracos e tronco por estagio). Mantida no
// contrato do modo fisico; o gesto novo vai por BallGesturePose3D.
struct BallKickPose3D {
    bool active = false;
    bool shot = false;
    bool quick = false;
    int side = 0;
    float power = 0.0f;
    float lift = 0.0f;
    Vec3 aim { 1.0f, 0.0f, 0.0f };
    float stage = 0.0f;
    float weight = 0.0f;
};

// O alvo do pe que vai a bola (a origem do link do pe, no mundo), com peso
// (0..1) sobre o alvo da passada (modo fisico).
struct BallFootGoal3D {
    bool active = false;
    int side = 0;
    Vec3 position;
    Quaternion orientation;
    float weight = 0.0f;
};

// Repulsao do pe pela bola: o alvo do pe no ar e empurrado para fora dela.
// O pe que joga fica de fora.
struct BallRepulsion3D {
    bool active = false;
    Vec3 position;
    float radius = 0.11f;
    int exemptSide = -1;
};

// (Contrato antigo do modo fisico: a pelve indo com um passo de apoio. O
// manejo novo nunca leva o corpo; fica sempre inativo.)
struct BallBodyGoal3D {
    bool active = false;
    Vec3 position;
    float weight = 0.0f;
};

struct BallControlOutput3D {
    // O gesto do corpo inteiro (as duas locomocoes).
    BallGesturePose3D gesture;
    // Modo fisico: o pe que joga (no balanco) ou dirigido (parado).
    BallFootGoal3D foot;
    BallFootScript3D script;
    // Desvio dos pes que nao estao jogando (somado ao alvo do pe no ar).
    std::array<Vec3, 2> avoid {};
    BallPlant3D plant;
    BallKickPose3D kick;
    BallBodyGoal3D body;
    // Ritmo da passada (abaixo de 1 o ciclo anda mais devagar) e a escala da
    // velocidade do corpo pedida (o chute correndo freia um pouco).
    float gaitRate = 1.0f;
    float speedScale = 1.0f;
    BallRepulsion3D repel;
    // Tocou neste tick: a bola passa a ter esta velocidade e giro. setMotion
    // vale tambem logo depois do toque (o pe ainda empurrando nao muda a bola).
    bool touched = false;
    bool setMotion = false;
    Vec3 ballVelocity;
    Vec3 ballSpin;
    BallControlPhase3D phase = BallControlPhase3D::Idle;
    BallActionKind3D kind = BallActionKind3D::None;
    BallTouchSurface3D surface = BallTouchSurface3D::Laces;
    int side = -1;
    // Por que o ultimo pedido nao tocou (0 nenhum, 1 descartado - a bola nao
    // ficou ao alcance de um gesto no tempo do pedido guardado -, 2 o gesto
    // acabou sem a bola entrar na area). Nunca com a bola na area no pedido
    // (a garantia), a nao ser que ela saia de repente (outro a leva).
    int missReason = 0;
    // Distancia do pe que joga (a caixa de verdade) a bola (m, superficie a
    // superficie) - so telemetria. No tick do toque pela hitbox, a do pe
    // deste tick (o desenhado).
    float footGap = 0.0f;
    // V3: o pedido guardado esperando dar para comecar.
    bool pending = false;
    // Onde o pe vai encontrar a bola (previsto; o desenho de depuracao) e
    // quanto falta para a forca sair (s; -1 sem acao ou ja saiu).
    bool contactValid = false;
    Vec3 contactPoint;
    float secondsToForce = -1.0f;
    // Reforco de estabilidade do modo fisico na janela do gesto (0..1, com
    // entrada e saida suaves): quem aplica decide se vale (nao com o corpo
    // encostando em outro jogador/objeto nem depois de um impacto).
    float stability = 0.0f;
    // A folga da hitbox dos pes neste tick (m; 0 sem o contato pela hitbox) -
    // o desenho de depuracao (a caixa do pe aumentada disto).
    float hitboxMargin = 0.0f;
};

// A area do manejo (leitura para o desenho de depuracao e os testes).
struct BallReachZone3D {
    // Raio no plano em volta da pelve (m) e a altura maxima da bola (a base
    // dela, em relacao a cintura: a pelve).
    float radius = 0.95f;
    float waistAbovePelvis = 0.05f;
};

class BallControl3D {
public:
    void reset();
    void update(const BallControlInput3D& input, BallControlOutput3D& output);
    [[nodiscard]] BallControlPhase3D phase() const { return m_phase; }
    [[nodiscard]] const BallReachZone3D& reachZone() const { return m_zone; }
    // A caixa de um pe (centro, orientacao e meia-dimensao) no estado dado.
    bool footBox(const RagdollState3D& state, int side, Vec3& center, Quaternion& orientation, Vec3& half) const;

private:
    struct FootGeometry {
        bool valid = false;
        std::size_t link = 0;
        std::size_t hip = 0;  // a coxa (a junta do quadril e a ancora dela)
        Vec3 center;          // centro da caixa (local do link)
        Vec3 half;            // meia-dimensao da caixa
        Quaternion boxOrientation;
        Vec3 hipAnchor;       // no link da coxa
        Vec3 ankleAnchor;     // no link do pe
        float thigh = 0.426f;
        float shin = 0.402f;
    };
    // Uma chave da perna: o tornozelo em relacao ao quadril em coordenadas
    // polares no referencial do gesto (balanco: o angulo a partir da vertical
    // para a frente do gesto; lado: para fora; comprimento), a orientacao do
    // pe e para onde o joelho aponta (mundo).
    struct LegKey {
        float swing = 0.0f;
        float lateral = 0.0f;
        float length = 0.8f;
        Quaternion foot;
        Vec3 knee { 1.0f, 0.0f, 0.0f };
    };
    // O corpo numa chave do gesto (ver BallGesturePose3D).
    struct BodyKey {
        float pelvisDrop = 0.0f, pelvisYaw = 0.0f, pelvisPitch = 0.0f, pelvisRoll = 0.0f, pelvisToSupport = 0.0f;
        float trunkTwist = 0.0f, trunkBend = 0.0f, trunkSide = 0.0f;
        // Punhos em relacao ao ombro: frente (no rumo do corpo), para fora,
        // para cima (m); o do lado que joga e o outro.
        Vec3 kickArm;
        Vec3 supportArm;
        float arms = 0.0f;
        // A pelve vai para a frente do gesto (m; o afundo sobre o apoio).
        float pelvisForward = 0.0f;
    };
    // O gesto pela posicao da bola em relacao ao corpo (V3).
    enum class Style : std::uint8_t {
        // A bola a frente ou do lado (o gesto da V2).
        Front,
        // A bola atras tocada para fora: a perna para tras e para fora, o pe
        // de ponta para baixo, o lado de fora dele na bola (Pro Soccer Online).
        BehindOut,
        // A bola atras tocada para dentro: a sola puxa curta, sem passar do
        // meio do corpo.
        BehindIn
    };
    struct Gesture {
        bool active = false;
        BallActionKind3D kind = BallActionKind3D::None;
        BallTouchSurface3D surface = BallTouchSurface3D::Laces;
        Style style = Style::Front;
        int side = 0;
        float power = 0.0f;
        // Para onde a bola vai (no plano) e o rumo do corpo no comeco.
        Vec3 push { 1.0f, 0.0f, 0.0f };
        Vec3 bodyForward { 1.0f, 0.0f, 0.0f };
        float time = 0.0f;
        // Tempos (s desde o comeco): armado, contato, acompanhamento e fim.
        float windupAt = 0.1f, contactAt = 0.2f, followAt = 0.35f, endAt = 0.6f;
        // Velocidade (por s) do balanco e do comprimento no contato.
        float swingRate = 0.0f, lengthRate = 0.0f;
        LegKey start, windup, contact, follow, recover;
        BodyKey bodyWindup, bodyContact, bodyFollow;
        bool touched = false;
        // A bola no instante do contato (prevista; a imaginada a frente, se
        // for o caso) e o quanto o ponto do pe foi tirado da de verdade (m: o
        // contato "de perto" para nao cruzar e a bola um pouco atras).
        Vec3 ballAtContact;
        Vec3 contactOffset;
        // A bola um pouco atras tocada para a frente: imaginada logo a frente.
        bool virtualFront = false;
        // A garantia sem gesto possivel ate a bola: o gesto inteiro numa bola
        // imaginada logo a frente (so o visual - o contato nao segue a de
        // verdade, nem o pe a acompanha depois da forca).
        bool imagined = false;
        // O tornozelo no contato quando a forca saiu (mundo): dali o contato
        // ainda acompanha a bola lenta, ate FollowMaximum.
        Vec3 contactAnkle;
        // O quanto o pe foi segurado atras da bola no tick anterior (m).
        float behindBall = 0.0f;
        // Modo fisico: o pe de verdade ja desceu no ponto de pouso.
        bool footDown = false;
        // O rumo usado na pose do contato (o empurrao; no toque esticado, para
        // a bola).
        Vec3 contactPush { 1.0f, 0.0f, 0.0f };
        // Andando: o gesto no ritmo da passada (o pe do balanco).
        bool moving = false;
        // O pe no comeco (parado, ele volta para la).
        Vec3 startAnkleWorld;
        Quaternion startFootWorld;
        Vec3 supportFootWorld;
        // A linha do meio do corpo (a pelve no comeco, andando com o corpo): as
        // pernas nao cruzam.
        Vec3 midline;
        // Custo da solucao (a escolha da perna: menor e mais limpa).
        float cost = 0.0f;
    };
    // Um pedido (o guardado e o em execucao).
    struct Request {
        BallActionKind3D kind = BallActionKind3D::None;
        float power = 0.0f;
        Vec3 direction { 1.0f, 0.0f, 0.0f };
        float lookYaw = 0.0f;
        float lift = 0.0f;
        float curve = 0.0f;
        float touchSpeed = -1.0f;
    };

    void resolve(const RagdollProfile3D& profile);
    [[nodiscard]] Vec3 hipJoint(const RagdollState3D& state, int side) const;
    [[nodiscard]] Vec3 ankleJoint(const RagdollState3D& state, int side) const;
    // A bola dentro da area em volta da pelve dada.
    [[nodiscard]] bool inArea(Vec3 ball, float radius, Vec3 pelvis) const;
    // A bola (a de verdade, ou a imaginada depois de uma forca antes do gesto)
    // e onde ela estara daqui a `seconds` (rolando com atrito, voando).
    void ballNow(const BallControlInput3D& input, Vec3& position, Vec3& velocity) const;
    [[nodiscard]] Vec3 predictBall(const BallControlInput3D& input, float seconds) const;
    // Onde a pelve estara (no plano) daqui a `seconds`: a velocidade dela indo
    // a pedida (a capsula acelera).
    [[nodiscard]] Vec3 predictPelvis(const BallControlInput3D& input, float seconds) const;
    // A velocidade do corpo (no plano) daqui a `seconds`, indo a pedida.
    [[nodiscard]] Vec3 predictBodyVelocity(const BallControlInput3D& input, float seconds) const;
    // A velocidade que o toque leva no rumo dele: a que o corpo vai ter (o
    // animado) ou a de agora (o fisico).
    [[nodiscard]] Vec3 carryVelocity(const BallControlInput3D& input) const;
    // O referencial do corpo (rumo).
    void bodyFrame(const BallControlInput3D& input, Vec3& forward, Vec3& left) const;
    // A velocidade do corpo no plano (a da locomocao; sem ela, a da pelve).
    [[nodiscard]] static Vec3 bodyVelocityOf(const BallControlInput3D& input);
    // A altura da pelve sobre o chao em fracao da de pe (1 de pe).
    [[nodiscard]] static float bodyHeightShare(const BallControlInput3D& input);
    // A superficie para empurrar `push` com o pe `side` (parado ou andando).
    [[nodiscard]] BallTouchSurface3D chooseSurface(int side, Vec3 push, Vec3 bodyForward, BallActionKind3D kind,
        bool moving, bool ballBehind) const;
    // Ponto e normal da superficie no referencial do link do pe.
    void surfaceLocal(int side, BallTouchSurface3D surface, Vec3& point, Vec3& normal) const;
    // O pe (origem e orientacao do link, mundo) no contato com a bola.
    void contactPose(int side, BallTouchSurface3D surface, Style style, Vec3 ball, float radius, Vec3 push,
        Vec3 bodyForward, float power, float groundHeight, Vec3& footPosition, Quaternion& footOrientation) const;
    // Quanto subir o pe para a caixa dele ficar `clearance` acima do chao.
    [[nodiscard]] float footGroundDeficit(int side, Vec3 position, Quaternion orientation, float groundHeight,
        float clearance) const;
    // Distancia da caixa do pe (pose dada, aumentada de margin) a bola,
    // superficie a superficie.
    [[nodiscard]] float boxGap(int side, Vec3 footPosition, Quaternion footOrientation, Vec3 ball,
        float radius, float margin) const;
    // As pernas nao se cruzam: o tornozelo que joga no lado dele da linha do
    // meio do corpo (a pelve, rumo do corpo) e, pelo peso supportWeight, longe
    // do de apoio quando lado a lado. Devolve o tornozelo corrigido (soft:
    // chega no limite sem quina).
    [[nodiscard]] Vec3 keepOwnSide(int side, Vec3 ankle, Vec3 pelvis, Vec3 bodyForward, Vec3 supportAnkle,
        float supportWeight, bool soft, float sideGap = -1.0f) const;
    // O rumo do empurrao do pedido em execucao (chute: o olhar).
    [[nodiscard]] Vec3 requestPush(Vec3 bodyForward) const;
    // O estilo e a superficie deste pe para a bola em `ball` (no contato) com
    // a pelve em `pelvis`; falso: nao da (o chute com a bola atras).
    bool styleFor(int side, Vec3 ball, Vec3 pelvis, Vec3 forward, Vec3 push, bool moving, Style& style,
        BallTouchSurface3D& surface, bool& virtualFront) const;
    // Polar <-> tornozelo em relacao ao quadril (referencial do gesto).
    [[nodiscard]] LegKey legKeyFrom(Vec3 ankleFromHip) const;
    [[nodiscard]] Vec3 legKeyVector(const LegKey& key) const;
    // A chave por angulos de junta: coxa a partir da vertical (+ a frente do
    // gesto), a flexao do joelho e o lado (+ para fora).
    [[nodiscard]] LegKey legKeyAngles(int side, float thigh, float knee, float lateral) const;
    // A orientacao do pe pela canela de uma chave (o tornozelo dobrado
    // `pitch` rad, + ponta para baixo).
    [[nodiscard]] Quaternion footAlongShin(int side, const LegKey& key, float pitch) const;
    // A bola de position/velocity daqui a `seconds` (rolando com atrito,
    // voando).
    [[nodiscard]] static Vec3 predictBallFrom(const BallControlInput3D& input, Vec3 position, Vec3 velocity,
        float seconds);
    // Planeja o gesto (pe, superficie, chaves e tempos) em `out`; falso se
    // nao da. contactAt: o instante do contato pedido (s a partir de agora;
    // negativo: o natural do gesto, pelo caminho do pe).
    bool plan(const BallControlInput3D& input, int side, bool moving, float contactAt, Gesture& out);
    // O tempo natural do contato de um gesto deste pe (s), antes de planejar.
    [[nodiscard]] float naturalContact(const BallControlInput3D& input, int side) const;
    // Parado: o instante (s) do encontro do gesto deste pe com a bola andando
    // (a melhor distancia dela a frente do quadril); -1 sem encontro valido;
    // early: o encontro bom e mais tarde que o gesto aguenta esperar (o
    // instante dele em later).
    [[nodiscard]] float standingMeeting(const BallControlInput3D& input, int side, float natural, bool& early,
        float* later = nullptr) const;
    // Quando a bola de verdade sai da area (s a partir de agora; 0 ja fora; -1
    // nao sai ate horizon).
    [[nodiscard]] float areaExit(const BallControlInput3D& input, float horizon) const;
    // Andando: o instante do contato deste pe (s a partir de agora; -1 nao da)
    // pela passada e pela bola; waitLift: quanto falta para ele sair do chao.
    [[nodiscard]] float movingContact(const BallControlInput3D& input, int side, bool kick, float& waitLift,
        float* score) const;
    // A acao ou o gesto anterior ainda ocupam (o pedido segue guardado).
    [[nodiscard]] bool busy() const;
    // O pedido guardado passa a ser o em execucao.
    void takeRequest();
    // Parado, a perna do desempate (a do lado da bola).
    [[nodiscard]] int preferredSide(const BallControlInput3D& input) const;
    // O pedido guardado: comeca (o gesto ou a espera do pe), se da agora.
    bool tryStart(const BallControlInput3D& input);
    // So a forca, ja (sem gesto): a acao acaba nela. A superficie (a sola
    // puxando tem a forca dela) pela mesma regra do gesto.
    void startBare(const BallControlInput3D& input);
    // Parado: o gesto da perna mais limpa. late: o encontro mais tarde que o
    // gesto aguenta esperar tambem serve; imagined: na bola imaginada.
    bool startStanding(const BallControlInput3D& input, bool late, bool imagined);
    // Andando: o pe e o instante pela passada (no balanco, ja; senao, a acao
    // espera ele sair do chao). imagined: na bola imaginada.
    bool startMoving(const BallControlInput3D& input, bool imagined);
    // A garantia: o pedido com a bola na area sai agora (o gesto ate ela, o
    // imaginado ou so a forca); leaving: a bola saindo da area - a forca ja.
    // Falso so com a acao anterior ainda sem a forca (este espera ela).
    bool startCommitted(const BallControlInput3D& input, bool leaving);
    // A bola imaginada logo a frente dele, no rumo do movimento e do lado da
    // perna (o gesto sempre acontece - a bola nunca sai sozinha): a forca ja
    // saiu e o gesto planejado nao cabe mais, ou a garantia sem gesto possivel
    // ate a bola de verdade.
    void imagineBallAhead(const BallControlInput3D& input, int side);
    // A acao (do aceite do pedido ao fim do gesto): o relogio da forca.
    void startAction();
    void endAction();
    // Quando a forca sai: a fracao do tempo ate o contato visual (s a partir
    // de agora), antes da bola sair da area.
    void scheduleForce(const BallControlInput3D& input, float secondsToContact);
    // Comeca o gesto planejado (o tempo da forca pelo contato dele).
    void beginGesture(const BallControlInput3D& input, const Gesture& gesture);
    // O gesto no instante atual: perna, corpo e o pe do modo fisico.
    void evaluate(const BallControlInput3D& input, BallControlOutput3D& output);
    // A forca na bola (o toque/chute) neste tick.
    void applyForce(const BallControlInput3D& input, BallControlOutput3D& output);
    // A hitbox do pe do gesto encostou na bola neste tick (varrida desde o
    // tick anterior), vindo para ela. O pe: o do gesto neste tick (corpo
    // animado) ou o de verdade (fisico).
    bool hitboxTouch(const BallControlInput3D& input, const BallControlOutput3D& output);
    // A folga da hitbox para este corpo (m).
    [[nodiscard]] float hitboxMargin(const BallControlInput3D& input) const;
    // Velocidade e giro da bola no toque/chute.
    void touchMotion(const BallControlInput3D& input, Vec3& velocity, Vec3& spin) const;
    void shotMotion(Vec3& velocity, Vec3& spin) const;

    // A forca na bola neste tick (a do pedido ou os padroes).
    BallForceSettings3D m_force;
    std::array<FootGeometry, 2> m_feet {};
    BallGestureLegs3D m_legs;
    bool m_resolved = false;
    BallReachZone3D m_zone;
    BallControlPhase3D m_phase = BallControlPhase3D::Idle;
    // O pedido em execucao.
    BallActionKind3D m_kind = BallActionKind3D::None;
    BallTouchSurface3D m_surface = BallTouchSurface3D::Laces;
    int m_side = 0;
    float m_power = 0.0f;
    Vec3 m_direction { 1.0f, 0.0f, 0.0f };
    float m_lookYaw = 0.0f;
    float m_lift = 0.0f;
    float m_curve = 0.0f;
    float m_touchSpeed = -1.0f;
    // O pedido guardado (V3): ate RequestBuffer s esperando dar para comecar.
    bool m_pending = false;
    float m_pendingSeconds = 0.0f;
    Request m_request;
    // A garantia: o pedido guardado ja viu a bola na area; a acao em curso
    // tem a forca garantida; a acao sem gesto (so a forca); ha quanto tempo a
    // bola de verdade esteve na area pela ultima vez (s).
    bool m_requestCommitted = false;
    bool m_actionCommitted = false;
    bool m_bareForce = false;
    float m_sinceInArea = 10.0f;
    // A velocidade da bola em relacao ao corpo na ultima forca (m/s): lancada
    // longe, o pedido guardado durante ela nao dispara a garantia na saida.
    float m_launchSpeed = 0.0f;
    // A acao em curso (da espera do pe ou do comeco do gesto ao fim): o
    // relogio, quando a forca sai nele e se ja saiu.
    bool m_action = false;
    float m_actionClock = 0.0f;
    float m_forceAt = 0.0f;
    bool m_forced = false;
    float m_timingFactor = 1.0f;
    // A forca saiu antes do gesto comecar (andando, esperando o pe): o gesto
    // vai onde a bola estaria sem ela (a bola de antes, prevista). Ou a bola
    // imaginada a frente (m_imagined): o gesto que comeca dela e imaginado.
    bool m_virtualBall = false;
    bool m_imagined = false;
    Vec3 m_virtualPosition;
    Vec3 m_virtualVelocity;
    float m_virtualClock = 0.0f;
    // A velocidade da bola neste tick (o golpe leva um pouco dela).
    Vec3 m_ballVelocityNow;
    // O toque define a velocidade da bola: logo depois dele o pe ainda
    // empurra; a bola fica na velocidade planejada.
    Vec3 m_touchVelocity;
    Vec3 m_touchSpin;
    float m_sinceTouch = 10.0f;
    // Intervalo minimo entre dois pedidos aceitos (s).
    float m_requestCooldown = 0.0f;
    // Andando: a acao esperando o pe escolhido sair do chao.
    bool m_waiting = false;
    int m_waitSide = 0;
    float m_waitSeconds = 0.0f;
    // O instante previsto do contato na espera (s a partir de agora).
    float m_waitContact = -1.0f;
    // Os pes apoiados no tick anterior (a saida do chao de um pe).
    std::array<bool, 2> m_lastPlanted { true, true };
    // A passada medida: apoio e balanco de cada pe (s) e o tempo na fase.
    std::array<float, 2> m_stanceSeconds { 0.16f, 0.16f };
    std::array<float, 2> m_swingSeconds { 0.42f, 0.42f };
    std::array<float, 2> m_footClock { 0.0f, 0.0f };
    Gesture m_gesture;
    // O referencial do gesto (as chaves polares): para onde o pe passa e o
    // lado de fora da perna que joga.
    Vec3 m_frameSwing { 1.0f, 0.0f, 0.0f };
    Vec3 m_frameOut { 0.0f, 1.0f, 0.0f };
    // Para onde a cabeca olha (suavizado).
    Vec3 m_lookPoint;
    // O reforco de estabilidade (sai suave depois do gesto).
    float m_stability = 0.0f;
    // A pose do gesto no tick anterior e a de um gesto cortado por outro (sai
    // suave em BlendOldSeconds).
    BallGesturePose3D m_lastPose;
    BallGesturePose3D m_carry;
    float m_carryTime = 10.0f;
    // A perna de apoio no ar afastada da que joga (0..1, suave); e o pe que
    // joga longe do apoio plantado (0..1, suave: entra e sai com o apoio).
    float m_separate = 0.0f;
    float m_supportHold = 1.0f;
    // O preparo do corpo na espera do pe (0..1).
    float m_prepare = 0.0f;
    // Modo fisico: o quanto o pe de verdade ainda segue o gesto (cai com o
    // erro grande e nao volta ate o proximo gesto).
    float m_physicalTrust = 1.0f;
    // O pe do tick anterior (a velocidade dele, telemetria).
    bool m_previousFootValid = false;
    Vec3 m_previousFootPosition;
    // O contato pela hitbox neste tick (do pedido) e a hitbox do pe que joga
    // e a bola no tick anterior (a varredura ate este).
    bool m_hitbox = true;
    bool m_hitPrevious = false;
    // O pe deste tick ate a bola (a caixa, sem a folga; m) - telemetria.
    float m_hitGap = 0.0f;
    Vec3 m_hitCenter;
    Quaternion m_hitOrientation;
    Vec3 m_hitBall;
};

} // namespace MatterEngine
