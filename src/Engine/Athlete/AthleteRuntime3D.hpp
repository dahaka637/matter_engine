#pragma once

#include "Engine/Athlete/AthleteSettings3D.hpp"
#include "Engine/Athlete/AthleteState3D.hpp"
#include "Engine/Character/BallOwnership3D.hpp"
#include "Engine/Character/CharacterControlTypes3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace MatterEngine {

// O que o runtime dos atletas precisa do personagem: o perfil, os clipes (os
// da locomocao fisica e os do modo padrao), as poses de referencia do
// experimento biomecanico e as medidas da capsula do jogador.
struct AthleteRuntimeAssets3D {
    const RagdollProfile3D* profile = nullptr;
    const CharacterLocomotionAnimations3D* clips = nullptr;
    const CharacterLocomotionAnimations3D* defaultClips = nullptr;
    const AnimationClip3D* idleReferenceClip = nullptr;
    const AnimationClip3D* walkReferenceClip = nullptr;
    CharacterMotorSettings3D avatarSettings;
};

// Um atleta na cena (um ragdoll humano): a identidade, o corpo publicado no
// ultimo passo e o estado de controle (de quem chama; o runtime so escreve
// nele).
// O comando de um atleta sem a capsula do jogador (a reescrita, rumo ao
// treino do RL: muitos atletas fisicos comandados na mesma cena). A intencao
// (com a velocidade ja pedida pela tabela de quem chama), o olhar e o sprint;
// a capsula virtual dele anda no pedido com a aceleracao do fisico e vai
// atras do corpo como a de verdade.
struct AthleteCommand3D {
    CharacterIntent3D intent;
    float facingYawRadians = 0.0f;
    float lookPitchRadians = 0.0f;
    bool sprinting = false;
};

struct AthleteSlot3D {
    std::uint64_t entityId = 0;
    RagdollHandle3D handle;
    const RagdollState3D* state = nullptr;
    bool active = true;
    bool frozen = false;
    AthleteState3D* athlete = nullptr;
    // Comandado sem capsula (so os nao controlados; nulo: o boneco solto de
    // sempre, que so segue o proprio corpo).
    const AthleteCommand3D* command = nullptr;
};

// Uma bola inflada na cena: o handle, o estado do ultimo passo e a posse
// (de quem chama; o runtime a atualiza).
struct AthleteBall3D {
    InflatedBallHandle3D handle;
    const InflatedBallState3D* state = nullptr;
    BallOwnership3D* ownership = nullptr;
};

// O input do atleta controlado neste tick (o laboratorio, o jogo ou a sonda).
struct AthleteControlInput3D {
    std::uint64_t controlledEntityId = 0;
    // A intencao pura (nunca reescrita) e o olhar.
    CharacterIntent3D intent;
    float facingYawRadians = 0.0f;
    float lookPitchRadians = 0.0f;
    bool sprinting = false;
    // A velocidade da capsula neste tick, o quanto ela bateu numa parede e a
    // velocidade com que ela foi atras do corpo neste tick (a do tick
    // anterior do runtime: nao e intencao do jogador).
    Vec3 proxyVelocityWorld;
    float capsuleWallImpact = 0.0f;
    Vec3 capsuleFollowVelocityWorld;
    // Espaco segurado (pressa no levantar) e a carga do pulo.
    bool spaceHeld = false;
    float jumpCrouch = 0.0f;
    float jumpCharge = 0.0f;
    // Manejo de bola: o pedido (borda, consumido no tick) e os botoes.
    BallActionKind3D ballRequest = BallActionKind3D::None;
    float ballRequestPower = 0.0f;
    float ballRequestAimYaw = 0.0f;
    float ballRequestLift = 0.0f;
    float ballRequestCurve = 0.0f;
    bool ballShotHeld = false;
    bool ballDribbleHeld = false;
    // Experimento biomecanico: a velocidade pedida.
    Vec3 biomechanicalRequestedVelocity;
};

// O que volta para quem chama. Os campos opcionais so valem quando o atleta
// controlado passou pelo tick (sem ele, quem chama mantem o valor anterior).
struct AthleteRuntimeOutput3D {
    // A capsula vai atras do corpo no proximo movimento dela.
    bool followVelocitySet = false;
    Vec3 followVelocityWorld;
    // O controlado esta no modo animado (o padrao).
    bool controlledAnimatedSet = false;
    bool controlledAnimated = false;
    // A escala da velocidade da capsula do controlado no proximo movimento
    // dela (o chute correndo do manejo novo freia o corpo de verdade).
    float controlledSpeedScale = 1.0f;
    // O pedido de bola deste tick foi consumido.
    bool ballRequestConsumed = false;
};

// O runtime dos atletas: por tick fixo, antes do PhysicsScene3D::simulate, com
// o estado publicado no passo anterior. Atualiza a posse das bolas, decide o
// modo do corpo de cada atleta, roda o manejo de bola do controlado e envia
// os alvos (guia, motores, forcas) para a fisica. E a logica que morava em
// WorkbenchApp::updateActiveRagdolls, agora na engine: o laboratorio, o jogo
// e a sonda sem janela rodam o mesmo codigo.
class AthleteRuntime3D {
public:
    void update(PhysicsScene3D& scene, const AthleteRuntimeAssets3D& assets, std::span<AthleteSlot3D> athletes,
        std::span<AthleteBall3D> balls, const AthleteRuntimeSettings3D& settings, const AthleteControlInput3D& control,
        float deltaTime, AthleteRuntimeOutput3D& output);
};

// O chao embaixo de um ponto (raio no estatico; sem chao, a altura de pe do
// perfil abaixo dele).
[[nodiscard]] float athleteGroundHeight3D(const PhysicsScene3D& scene, const RagdollProfile3D& profile, Vec3 position);

// A velocidade da capsula no modo padrao para um rumo em relacao ao olhar,
// pela tabela dos ajustes (a aba "Velocidades": o trote em todos os rumos, a
// corrida a frente, de lado e de costas).
[[nodiscard]] float athleteDefaultTravelSpeed3D(const AthleteRuntimeSettings3D& settings,
    float travelRelativeToLookRadians, bool sprinting);

// A batida da capsula (m/s; AthleteControlInput3D::capsuleWallImpact): o
// quanto a velocidade no plano caiu, no rumo que tinha, alem da frenagem dela
// num tick (parede no caminho). Pelo quanto ela andou de fato (travada numa
// parede, a velocidade dela seguia dizendo a pedida). Subindo um degrau
// (rose: a capsula subiu neste tick ou no anterior, m) a queda da velocidade e
// a subida, nao parede - contada como parede, o corpo ia ao fisico no instante
// do salto da pelve e voava (plano v2, fase E).
[[nodiscard]] float athleteCapsuleWallImpact3D(Vec3 previousMoved, Vec3 moved, float braking, bool grounded,
    float rose);

} // namespace MatterEngine
