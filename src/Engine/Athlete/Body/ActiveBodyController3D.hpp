#pragma once

#include "Engine/Athlete/Body/SupportWrench3D.hpp"
#include "Engine/Athlete/Sensing/BodyObservation3D.hpp"
#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace MatterEngine {

// Corpo Ativo: o estado fisico intermediario (rework de 10/10/2026, Etapa 2).
// O corpo e fisico de verdade - gravidade, limites de junta, colisao com o
// chao e com tudo -, com musculos fortes seguindo a pose-alvo do plano, e a
// unica ajuda e a que um atleta tem: empurrar o chao com os pes apoiados.
//  - Musculos: os motores das juntas com a pose-alvo e, sobretudo, a
//    velocidade dela (sem o alvo de velocidade o corpo atrasa a pose).
//  - Raiz: a forca e o torque que levam a pelve ao plano viram pares com os
//    pes apoiados (SupportWrench3D), cortados pelo atrito e pela sola. No ar
//    ou caido nada passa: sem guindaste invisivel, sem boneco levitando.
//  - Ajuda de jogo (opcional): o que o chao nao deu, no plano, ate um teto,
//    so com pe apoiado; contabilizada.
//  - Pancada: o tonus e os ganhos caem pelo impulso e voltam com o tempo - o
//    corpo cede, cambaleia e volta pelos proprios pes.
//  - Nocaute (vai ao fisico pleno): postura perdida, corpo baixo demais,
//    pancada forte.
//  - Volta ao animado: calmo e com o corpo colado a pose-alvo.
// Nunca escreve posicao nem velocidade de corpo: so alvos de motor e forcas.
struct ActiveBodySettings3D {
    // Servo da raiz (a pelve no plano): frequencia (rad/s) e razao de
    // amortecimento; aceleracao maxima pedida (m/s2).
    float rootFrequency = 7.0f;
    float rootDamping = 1.0f;
    float maximumRootAcceleration = 25.0f;
    // A aceleracao antecipada da raiz-alvo: filtro (s) e teto (m/s2).
    float rootFeedforwardSeconds = 0.08f;
    float maximumRootFeedforward = 8.0f;
    // Servo da orientacao da pelve (o binario com os pes apoiados).
    float orientationFrequency = 10.0f;
    float orientationDamping = 1.0f;
    float bodyInertiaHorizontal = 11.0f;
    float bodyInertiaVertical = 2.5f;
    // O chao (ver SupportWrenchSettings3D).
    SupportWrenchSettings3D support;
    // Ajuda de jogo: aceleracao maxima no plano que o que o chao nao deu pode
    // receber de fora (m/s2), so com pe apoiado. 0 desliga.
    float gameplayAssistAcceleration = 4.0f;
    // A ajuda de jogo cresce com a velocidade pedida pelo jogador ate esta
    // (m/s); parado ela e zero.
    float assistFullSpeed = 4.0f;
    // Vindo do fisico pleno (que segurava o corpo com a ajuda na pelve): uma
    // ajuda de passagem no plano que comeca neste teto (m/s2) e some no tempo
    // da passagem (s), enquanto a pose leva o corpo para cima da base.
    float handoverAssistAcceleration = 6.0f;
    float handoverSeconds = 0.5f;
    // Tonus por regiao (multiplica a rigidez do perfil) e o alvo de velocidade
    // das juntas (fracao e limite, rad/s).
    float legTone = 1.0f;
    float torsoTone = 1.0f;
    float armTone = 0.8f;
    float velocityFeedforward = 1.0f;
    float maximumJointVelocity = 30.0f;
    // Pancada: impulso (N.s) a partir do qual o corpo cede, o que leva a queda
    // maxima, a queda maxima (fracao) e o tempo da volta (s).
    float impactMinimum = 1.0f;
    float impactFull = 20.0f;
    float impactMaximumDrop = 0.65f;
    float impactRecoverSeconds = 0.9f;
    // Nocaute: pelve e peito alem destas inclinacoes (rad) por mais de
    // knockoutSeconds; pelve abaixo desta fracao da altura de pe; pancada
    // acima de knockoutImpulse (N.s).
    float knockoutPelvisTilt = 0.75f;
    float knockoutChestTilt = 0.96f;
    float knockoutSeconds = 0.15f;
    float knockoutHeightShare = 0.62f;
    float knockoutImpulse = 32.0f;
    // Volta ao animado: erros maximos contra a pose-alvo (m, rad, m/s) e o
    // tempo calmo (s) com eles.
    float returnRootError = 0.05f;
    float returnLinkError = 0.07f;
    float returnOrientationError = 0.12f;
    float returnVelocityError = 0.7f;
    float returnCalmSeconds = 0.25f;
    float returnMinimumSeconds = 0.35f;
};

// A pose-alvo do tick (a do plano: hoje a do modo padrao).
struct ActiveBodyTarget3D {
    // Os links no mundo e as coordenadas de junta (as dos motores), a raiz.
    const RagdollAnimationPose3D* pose = nullptr;
    const std::vector<Vec3>* coordinates = nullptr;
    Vec3 rootPosition;
    Quaternion rootOrientation;
    // A velocidade que o jogador pede (m/s, no plano): a ajuda de jogo cresce
    // com ela (a capsula indo atras da base nao conta como andar).
    float intendedSpeed = 0.0f;
};

struct ActiveBodyOutput3D {
    std::vector<RagdollDriveTarget3D> drives;
    std::vector<LinkWrench3D> wrenches;
    // A ajuda de jogo deste tick (fora; no centro de massa da pelve).
    Vec3 assistForce;
    // Telemetria.
    float compliance = 1.0f;
    int supportCount = 0;
    Vec3 requestedForce;
    Vec3 deliveredForce;
    Vec3 droppedForce;
    Vec3 targetRoot;
    float rootError = 0.0f;
    float orientationError = 0.0f;
    float linkError = 0.0f;
    float assistImpulseTotal = 0.0f;
    bool knockout = false;
    std::string knockoutReason;
    bool readyToAnimate = false;
};

class ActiveBodyController3D {
public:
    void reset(const RagdollProfile3D& profile);
    // Entrando no Ativo (do animado ou do fisico): zera os relogios e parte da
    // pose-alvo atual (sem salto no alvo de velocidade). fromPhysical: com a
    // ajuda de passagem (ver handoverAssistAcceleration).
    void enter(bool fromPhysical = false);
    [[nodiscard]] bool handingOver() const { return m_handoverLeft > 0.0f; }
    void update(const RagdollProfile3D& profile, const RagdollState3D& state, const BodyObservation3D& observation,
        const ActiveBodyTarget3D& target, const ActiveBodySettings3D& settings, float deltaTime,
        ActiveBodyOutput3D& output);
    [[nodiscard]] float secondsActive() const { return m_seconds; }

private:
    // Regiao de cada link: 0 perna, 1 tronco/cabeca, 2 braco.
    std::vector<std::uint8_t> m_region;
    std::vector<int> m_parent;
    std::vector<Vec3> m_previousCoordinates;
    bool m_havePrevious = false;
    Vec3 m_previousRoot;
    Quaternion m_previousRootOrientation;
    Vec3 m_previousRootVelocity;
    Vec3 m_rootAcceleration;
    bool m_haveRoot = false;
    float m_compliance = 1.0f;
    // O link mais atingido e o quanto a regiao dele cede a mais (0..1).
    std::uint32_t m_hitLink = 0;
    float m_hitLocal = 0.0f;
    float m_seconds = 0.0f;
    float m_calmSeconds = 0.0f;
    float m_closeSeconds = 0.0f;
    float m_tiltSeconds = 0.0f;
    float m_assistImpulse = 0.0f;
    float m_handoverLeft = 0.0f;
    bool m_handoverPending = false;
};

} // namespace MatterEngine
