#pragma once

#include "Engine/Athlete/Core/Kinematics3D.hpp"
#include "Engine/Athlete/Sensing/BodyObservation3D.hpp"

#include <array>
#include <vector>

namespace MatterEngine {

// A base do Corpo Ativo (Etapa 2): a pose-alvo vem do modo padrao, que poe os
// pes onde o clipe quer, mas no corpo fisico os pes ficam onde estao no chao
// (uma pancada os tira do lugar). Sem isto os musculos brigam com o chao, o
// corpo fica torto e nunca cola na pose.
//  - Pe apoiado (o clipe diz apoio e o pe toca o chao): fica preso onde
//    esta; a perna vai a ele por IK a partir da pelve do plano.
//  - Pe no balanco do clipe: solta do lugar e volta a pose do clipe numa
//    mistura curta.
//  - Parado com um pe fora do lugar do clipe: um passo de acomodacao - o peso
//    vai para o outro pe, o pe sai num arco e pousa onde a pose quer, o peso
//    volta. Assim o corpo volta a base do clipe e o padrao pode assumir sem
//    os pes escorregarem.
struct ActiveStanceSettings3D {
    // A acomodacao (ligada) e o pe mais longe que isto (m) do lugar do clipe,
    // parado. (Desligada por padrao ate a Etapa 3: passar o peso de um pe
    // para o outro pela fisica ainda falha nas bases largas; o planejador de
    // pegadas faz isso direito.)
    bool settleSteps = false;
    float settleDistance = 0.05f;
    // A acomodacao: o peso vai ao outro pe, o pe no arco, o peso volta (s), a
    // altura do arco (m) e quanto a pelve vai para o pe de apoio (fracao).
    float shiftSeconds = 0.55f;
    float swingSeconds = 0.26f;
    float settleHeight = 0.06f;
    float shiftShare = 0.92f;
    float maximumShift = 0.30f;
    // O pe so sai com a captura dentro da sola do pe que fica (mais esta
    // folga, m); sem isso ate shiftTimeout (s) depois do comeco, a acomodacao
    // desiste.
    float liftMargin = 0.03f;
    float shiftTimeout = 1.1f;
    // Abaixo desta velocidade da pose (m/s) a base pode se acomodar.
    float stillSpeed = 0.35f;
    // O pe que sai do apoio volta a pose do clipe nesta mistura (s).
    float releaseSeconds = 0.12f;
    // Parado, a raiz-alvo fica sobre os pes que estao no chao (como a do
    // clipe sobre os dela), entrando e saindo nesta mistura (s).
    bool anchorRootToBase = true;
    float baseAnchorSeconds = 0.25f;
    // Passo de recuperacao (equilibrio): o ponto de captura do centro de massa
    // (x + v/w0) mais que recoveryMargin (m) fora da base dos pes apoiados -> o
    // pe mais longe dele (o descarregado) sai rapido e pousa recoveryOvershoot
    // (m) alem da captura, entre recoveryMinimumWidth e recoveryMaximumReach
    // (m) do pe que fica. O balanco (s) e a altura do arco (m).
    // (Desligado por padrao ate a Etapa 3: o passo de equilibrio de verdade
    // vem do planejador de pegadas; hoje a pancada que pede passo vai ao
    // fisico pleno, que tem os passos de recuperacao.)
    bool recoverySteps = false;
    float recoveryMargin = 0.03f;
    float recoveryOvershoot = 0.07f;
    float recoveryMinimumWidth = 0.16f;
    float recoveryMaximumReach = 0.75f;
    float recoverySwingSeconds = 0.22f;
    float recoveryHeight = 0.05f;
};

class ActiveStance3D {
public:
    void reset(const RagdollProfile3D& profile);
    // Entrando no Ativo: os pes prendem onde estiverem apoiados.
    void enter();
    // Ajusta a pose-alvo (raiz, coordenadas e a pose no mundo) aos pes que
    // estao no chao. clipPlanted: o clipe diz que o pe esta apoiado.
    // allowSettle: a acomodacao pode comecar (nao durante a inercia da volta,
    // quando a pose ainda esta indo do corpo ate a do clipe).
    void adapt(const RagdollProfile3D& profile, const RagdollState3D& body, const BodyObservation3D& observation,
        std::array<bool, 2> clipPlanted, float targetSpeed, bool allowSettle, const ActiveStanceSettings3D& settings,
        float deltaTime, Vec3& rootPosition, Quaternion rootOrientation, std::vector<Vec3>& coordinates,
        RagdollAnimationPose3D& pose);
    // A maior distancia (m) de um pe preso ao lugar do clipe (sem a
    // ancoragem da raiz: onde o animado o pora).
    [[nodiscard]] float mismatch() const { return m_mismatch; }
    [[nodiscard]] bool settling() const { return m_settleSide >= 0; }
    // O passo em andamento e de recuperacao (equilibrio), nao de acomodacao.
    [[nodiscard]] bool recovering() const { return m_settleSide >= 0 && m_recovery; }
    // Quanto o ponto de captura esta fora da base (m; 0 dentro).
    [[nodiscard]] float captureOutside() const { return m_captureOutside; }
    // O quanto a raiz-alvo andou (no plano) para ficar sobre a base real (a
    // capsula vai atras disso).
    [[nodiscard]] Vec3 baseShift() const { return m_baseShift; }

private:
    struct Foot {
        bool anchored = false;
        Vec3 position;
        Quaternion orientation;
        // Saindo do apoio: 1 no comeco da mistura ate a pose do clipe, 0 no fim.
        float release = 0.0f;
        Vec3 releasePosition;
        Quaternion releaseOrientation;
    };
    AthleteLegs3D m_legs;
    std::array<float, 2> m_soleToOrigin {};
    bool m_ready = false;
    std::array<Foot, 2> m_feet {};
    // A acomodacao em andamento: o pe (-1 nenhuma), o tempo, de onde e para
    // onde (a origem do link do pe e a orientacao), e o lado da pelve.
    int m_settleSide = -1;
    float m_settleTime = 0.0f;
    Vec3 m_settleFrom;
    Vec3 m_settleTo;
    Quaternion m_settleFromOrientation;
    Quaternion m_settleToOrientation;
    Vec3 m_settleShift;
    // O passo em andamento e de recuperacao: sem a ida do peso, balanco curto.
    bool m_recovery = false;
    // A acomodacao: o pe ja saiu (o peso chegou no outro) e quando (s).
    bool m_lifted = false;
    float m_liftAt = 0.0f;
    // Quanto do peso ja foi ao pe que fica (0..1, continuo: sobe na ida,
    // segura no balanco, desce na volta).
    float m_shiftLevel = 0.0f;
    float m_shiftAtLift = 1.0f;
    float m_captureOutside = 0.0f;
    float m_mismatch = 0.0f;
    // A raiz sobre a base: o peso da ancoragem (0..1), o deslocamento cheio e
    // o aplicado.
    float m_baseWeight = 0.0f;
    Vec3 m_baseTarget;
    Vec3 m_baseShift;
};

} // namespace MatterEngine
