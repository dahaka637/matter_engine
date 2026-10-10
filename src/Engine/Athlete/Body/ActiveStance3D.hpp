#pragma once

#include "Engine/Athlete/Core/Kinematics3D.hpp"
#include "Engine/Athlete/Sensing/BodyObservation3D.hpp"

#include <array>
#include <vector>

namespace MatterEngine {

// A base do Corpo Ativo (Etapas 2 e 3): a pose-alvo vem do modo padrao, que
// poe os pes onde o clipe quer, mas no corpo fisico os pes ficam onde estao
// no chao (uma pancada os tira do lugar). Sem isto os musculos brigam com o
// chao, o corpo fica torto e nunca cola na pose.
//  - Pe apoiado (o clipe diz apoio e o pe toca o chao): fica preso onde
//    esta; a perna vai a ele por IK.
//  - Pe no balanco do clipe: solta do lugar e volta a pose do clipe numa
//    mistura curta.
//  - Parado, a raiz-alvo fica sobre os pes de verdade como a do clipe fica
//    sobre os dele (filtrada: a base muda quando um pe sai ou pousa).
//  - Passo de equilibrio: o ponto de captura do centro de massa (x + v/w0)
//    saiu da base. O pe mais longe dele sai e pousa alem dele - o alvo segue
//    a captura durante o balanco, a perna vai pela pelve de verdade (com o
//    corpo inclinado, a do plano poria o pe longe de onde ele precisa) e o
//    pouso e o contato. No passo a raiz-alvo segue a pelve (o passo e quem
//    equilibra; o servo nao briga com a queda).
// (A acomodacao de volta a base do clipe e das pegadas do modo padrao, na
// volta ao animado.)
struct ActiveStanceSettings3D {
    // Parado (a pose abaixo de stillSpeed m/s), a raiz sobre a base real,
    // entrando e saindo nesta mistura (s); o salto quando muda o conjunto de
    // pes presos some neste tempo (s).
    bool anchorRootToBase = true;
    float stillSpeed = 0.35f;
    float baseAnchorSeconds = 0.25f;
    float baseFilterSeconds = 0.15f;
    // O pe que sai do apoio volta a pose do clipe nesta mistura (s).
    float releaseSeconds = 0.12f;
    // Passos de equilibrio: a captura mais que stepMargin (m) fora da base. O
    // pe pousa stepOvershoot (m) alem dela, entre stepMinimumWidth e
    // stepMaximumReach (m) do pe que fica; o balanco (s) e o arco (m). O
    // contato pousa o pe depois de stepTouchShare do balanco; em
    // stepTimeoutShare dele o passo acaba de qualquer jeito. No passo, a
    // raiz-alvo vai stepRelax do caminho ate a pelve de verdade, e volta em
    // stepRelaxSeconds depois. No maximo maximumSteps seguidos.
    bool balanceSteps = false;
    float stepMargin = 0.03f;
    float stepOvershoot = 0.08f;
    float stepMinimumWidth = 0.18f;
    float stepMaximumReach = 0.75f;
    float stepSeconds = 0.20f;
    float stepHeight = 0.06f;
    // A escolha do pe: a captura fora da base nova (o pe que fica mais o
    // pouso) pesa stepCatchWeight vezes o caminho do pe. O fim do balanco
    // mira stepPress (m) abaixo do chao.
    float stepCatchWeight = 4.0f;
    float stepPress = 0.01f;
    // O pe do passo nunca cruza a linha do outro: fica pelo menos isto (m)
    // do lado dele. E o passo so sai com a captura fora por stepTriggerTicks
    // ticks seguidos (no tranco da pancada o contato pisca).
    float stepSideGap = 0.10f;
    int stepTriggerTicks = 3;
    float stepTouchShare = 0.55f;
    float stepTimeoutShare = 1.5f;
    float stepRelax = 0.7f;
    float stepRelaxSeconds = 0.3f;
    int maximumSteps = 5;
};

class ActiveStance3D {
public:
    void reset(const RagdollProfile3D& profile);
    // Entrando no Ativo: os pes prendem onde estiverem apoiados.
    void enter();
    // Ajusta a pose-alvo (raiz, coordenadas e a pose no mundo) aos pes que
    // estao no chao. clipPlanted: o clipe diz que o pe esta apoiado.
    void adapt(const RagdollProfile3D& profile, const RagdollState3D& body, const BodyObservation3D& observation,
        std::array<bool, 2> clipPlanted, float targetSpeed, const ActiveStanceSettings3D& settings, float deltaTime,
        Vec3& rootPosition, Quaternion rootOrientation, std::vector<Vec3>& coordinates, RagdollAnimationPose3D& pose);
    // A maior distancia (m) de um pe preso ao lugar do clipe (sem a
    // ancoragem da raiz: onde o animado o pora).
    [[nodiscard]] float mismatch() const { return m_mismatch; }
    // Um passo de equilibrio em andamento e quantos ja foram seguidos.
    [[nodiscard]] bool stepping() const { return m_step.side >= 0; }
    [[nodiscard]] int stepsTaken() const { return m_stepsTaken; }
    // Quanto o ponto de captura esta fora da base (m; 0 dentro). No passo, a
    // base e o pe que fica mais o lugar do pouso.
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
    struct Step {
        int side = -1;
        float time = 0.0f;
        // De onde sai (a origem do link) e para onde vai (filtrado), a
        // orientacao e a altura do chao.
        Vec3 from;
        Vec3 target;
        Quaternion orientation;
        float ground = 0.0f;
    };
    void anchor(int side, const RagdollProfile3D& profile, const RagdollState3D& body,
        const FootObservation3D& seen);
    [[nodiscard]] Vec3 stepTarget(int side, Vec3 capture, Vec3 left, const ActiveStanceSettings3D& settings) const;

    AthleteLegs3D m_legs;
    std::array<float, 2> m_soleToOrigin {};
    bool m_ready = false;
    std::array<Foot, 2> m_feet {};
    Step m_step;
    int m_stepsTaken = 0;
    int m_outsideTicks = 0;
    float m_relax = 0.0f;
    float m_captureOutside = 0.0f;
    float m_mismatch = 0.0f;
    // A raiz sobre a base: o peso da ancoragem (0..1), o deslocamento cheio
    // (filtrado) e o aplicado.
    float m_baseWeight = 0.0f;
    Vec3 m_baseTarget;
    Vec3 m_baseShift;
    // O salto da troca dos pes presos (some no tempo) e quais estao presos.
    Vec3 m_baseBias;
    int m_baseMask = 0;
    bool m_baseFresh = true;
};

} // namespace MatterEngine
