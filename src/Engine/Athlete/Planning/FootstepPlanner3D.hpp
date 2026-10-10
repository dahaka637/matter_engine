#pragma once

#include "Engine/Athlete/Core/FootGeometry3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstdint>

namespace MatterEngine {

// O que o estilo (hoje o modo padrao, pelos clipes) diz dos pes neste tick:
// onde poria cada um (a origem do link e a orientacao, no mundo), quem esta
// apoiado e os tempos do ciclo.
struct FootstepStyle3D {
    std::array<Vec3, 2> position {};
    std::array<Quaternion, 2> orientation {};
    std::array<bool, 2> planted { true, true };
    // No balanco: quanto falta para pousar (s). Apoiado e andando: quanto
    // falta para sair (s) e quanto dura o balanco (s). -1 sem ciclo.
    std::array<float, 2> secondsToTouchdown { -1.0f, -1.0f };
    std::array<float, 2> secondsToLiftoff { -1.0f, -1.0f };
    std::array<float, 2> swingSeconds { -1.0f, -1.0f };
    // A raiz (a pelve) da pose do estilo: o alcance da perna sai dela. E a
    // velocidade da capsula (no plano): o estilo parado anda junto com ela.
    Vec3 rootPosition;
    Quaternion rootOrientation;
    Vec3 velocity;
    // No chao (fora do pulo) e sem o caminho de clipe que leva o corpo
    // (carrinho, arrancada agachada): sem isso, os pes seguem o estilo.
    bool grounded = true;
    bool scripted = false;
    // A perna que o gesto da bola leva (-1 nenhuma): segue o estilo. E o pe
    // de apoio reservado pelo gesto (-1 nenhum): fica no chao ate o gesto o
    // soltar, mesmo com o estilo pedindo o balanco.
    int actingSide = -1;
    int reservedSide = -1;
    // 0 parado ... 1 andando.
    float movingShare = 0.0f;
};

struct FootstepSettings3D {
    // Parado, um pe longe do lugar do estilo (m ou rad) da um passo ate la -
    // um de cada vez, com os dois apoiados, settleDelaySeconds depois do
    // ultimo pouso. O passo (s) e a altura do arco (m).
    bool settleSteps = true;
    float settleDistance = 0.07f;
    float settleYawRadians = 0.35f;
    float settleMovingShare = 0.3f;
    float settleDelaySeconds = 0.08f;
    float settleSeconds = 0.30f;
    float settleHeight = 0.05f;
    // O pe so trava com a sola ate esta altura (m) acima do chao do estilo;
    // mais alto, um passo curto (s) ate onde o estilo o poe.
    float landingHeight = 0.025f;
    float catchSeconds = 0.18f;
    // Andando, o pe apoiado sai antes do estilo quando a perna chega a esta
    // fracao do comprimento (do quadril ao tornozelo): um pe nao fica no
    // chao alem do alcance.
    float reachLimit = 0.985f;
    // A saida do apoio: o balanco parte com a velocidade que o pe tinha (o do
    // estilo ja anda) e a diferenca some neste tempo (s).
    float liftBlendSeconds = 0.10f;
    // O pouso macio: perto dele o pe anda no plano no maximo a
    // landingDeceleration (m/s2) vezes o tempo que falta (nunca menos que
    // landingMinimumSpeed m/s).
    float landingDeceleration = 200.0f;
    float landingMinimumSpeed = 0.3f;
    // O balanco nunca anda mais rapido que um de minimumSwingSeconds (s): o
    // tempo do estilo salta quando a marcha dominante troca. Com o estilo ja
    // pousando e o pe ainda no ar, o pe pousa atrasado (sem salto).
    float minimumSwingSeconds = 0.14f;
    // Arrancada: a capsula ja anda (acima de startSpeed m/s) e o estilo ainda
    // nao comecou o ciclo (os dois pes apoiados no parado dele, deslizando
    // com ela): o pe mais para tras (alem de startLag m do lugar do estilo)
    // da um passo rapido (s, altura m) para onde o estilo o pora no fim dele.
    float startSpeed = 0.4f;
    float startLag = 0.05f;
    float startSeconds = 0.22f;
    float startHeight = 0.07f;
    // O pe que sai do apoio para o estilo livre (pulo, gesto, carrinho) chega
    // nele nesta mistura (s).
    float freeBlendSeconds = 0.15f;
    // A sola: os dedos dobram a esta distancia da ponta (m) e a diferenca de
    // altura (m) entre o calcanhar e a base dos dedos que troca o ponto em
    // que o pe rola.
    float toeRollMeters = 0.08f;
    float pivotSwitchHeight = 0.004f;
};

// O pe para a sintese: travado no apoio, no balanco, num passo de
// acomodacao, ou livre (o do estilo).
enum class FootMode3D : std::uint8_t { Free, Stance, Swing, Step };

struct FootTarget3D {
    FootMode3D mode = FootMode3D::Free;
    Vec3 position;
    Quaternion orientation;
};

struct FootstepTelemetry3D {
    // O quanto o pe estava longe do estilo ao sair do apoio (m) - o que o
    // balanco desfaz - e o maior visto; as saidas antes do estilo (alcance),
    // os passos de acomodacao e os pousos no ar (passo curto).
    std::array<float, 2> liftCorrection {};
    float worstLiftCorrection = 0.0f;
    int earlyLifts = 0;
    int settleSteps = 0;
    int catchSteps = 0;
    int startSteps = 0;
};

// O dono dos pes no modo padrao (Etapa 3, v1): o estilo diz quando cada pe
// apoia e a forma do balanco; os pes do corpo nao escorregam.
//  - Apoio: o pe fica onde pousou, no rumo em que pousou. Ele rola no
//    calcanhar (no comeco) e na base dos dedos (no fim) como o estilo o
//    inclina: o ponto que toca o chao nao sai do lugar. Andando, sai antes
//    do estilo se a perna chegar ao fim do alcance.
//  - Balanco: sai de onde o pe estava (sem salto) e a diferenca para o
//    estilo some no balanco todo, pelo tempo, numa curva de jerk minimo (a
//    trava antiga, de 05/10, soltava em 0,1 s e dava picos). Perto do pouso
//    o pe desacelera ate parar - o clipe pousa com o pe andando, ate 10 m/s
//    nas trocas de direcao, com o corpo girando. Pousa onde o estilo poe o
//    pe (atrasado o que a desaceleracao pedir).
//  - Parado: o pe longe do lugar do estilo (o fim de uma corrida, um giro,
//    a volta do fisico) da um passo curto ate la.
//  - Livre: no pulo, no gesto da bola e no carrinho os pes seguem o estilo.
class FootstepPlanner3D {
public:
    void reset(const RagdollProfile3D& profile, const FootstepSettings3D& settings);
    [[nodiscard]] bool ready() const { return m_ready; }
    [[nodiscard]] const AthleteLegs3D& legs() const { return m_legs; }
    [[nodiscard]] const AthleteFootGeometry3D& geometry() const { return m_geometry; }
    // A sola mais baixa (calcanhar ou base dos dedos) do pe nesta pose (m).
    [[nodiscard]] float soleHeight(int side, Vec3 position, Quaternion orientation) const {
        return lowestSole(side, position, orientation);
    }
    [[nodiscard]] bool live() const { return m_live; }
    // Comecando (vindo do corpo fisico): cada pe no chao fica apoiado onde
    // esta; os outros, livres.
    void seed(const std::array<Vec3, 2>& position, const std::array<Quaternion, 2>& orientation,
        const std::array<bool, 2>& grounded);
    // Fora do modo padrao: o proximo plano comeca do corpo.
    void clear() {
        m_live = false;
        m_previousActing = -1;
    }
    [[nodiscard]] std::array<FootTarget3D, 2> plan(const FootstepStyle3D& style, const FootstepSettings3D& settings,
        float deltaTime);
    [[nodiscard]] const FootstepTelemetry3D& telemetry() const { return m_telemetry; }

private:
    struct Foot {
        FootMode3D mode = FootMode3D::Free;
        // A ultima saida.
        Vec3 position;
        Quaternion orientation;
        // Apoio: o ponto que toca o chao (mundo), qual e (dedos ou calcanhar)
        // e o rumo travado.
        Vec3 pivot;
        bool toePivot = false;
        float yaw = 0.0f;
        // Balanco e livre: a diferenca para o estilo na saida (m, rad), o
        // quanto dela ja foi (0..1), o tempo desde a saida (s) e se o estilo
        // ja saiu do apoio (no balanco que comecou antes dele).
        Vec3 offset;
        float offsetYaw = 0.0f;
        float progress = 0.0f;
        float elapsed = 0.0f;
        bool styleSwung = false;
        // O andar da saida e o do pe do estilo (do tick anterior) e a
        // diferenca entre eles na saida do apoio.
        Vec3 velocity;
        Vec3 stylePrevious;
        Vec3 styleVelocity;
        bool haveHistory = false;
        Vec3 liftVelocity;
        // Passo (acomodacao, arrancada, pouso no ar): o tempo, a duracao, a
        // altura do arco e de onde.
        float stepTime = 0.0f;
        float stepSeconds = 0.3f;
        float stepHeight = 0.05f;
        Vec3 stepFrom;
        Quaternion stepFromOrientation;
    };
    void lock(int side, Vec3 position, Quaternion orientation);
    void startOffset(int side, FootMode3D mode, const FootstepStyle3D& style);
    void startStep(int side, float seconds, float height);
    [[nodiscard]] float lowestSole(int side, Vec3 position, Quaternion orientation) const;
    [[nodiscard]] float reach(int side, const FootstepStyle3D& style) const;
    [[nodiscard]] FootTarget3D stance(int side, const FootstepStyle3D& style, const FootstepSettings3D& settings);

    bool m_ready = false;
    AthleteLegs3D m_legs;
    AthleteFootGeometry3D m_geometry;
    std::array<Foot, 2> m_feet {};
    bool m_live = false;
    float m_sinceLanding = 0.0f;
    // A perna do gesto no tick anterior (a soltura dela trava o pe onde ele
    // pousou).
    int m_previousActing = -1;
    FootstepTelemetry3D m_telemetry;
};

} // namespace MatterEngine
