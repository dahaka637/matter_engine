#include "Engine/Tracking/TrackingGetUpAmpTraining3D.hpp"

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/Tracking/TrackingClipReference3D.hpp"
#include "Engine/Tracking/TrackingGetUp3D.hpp"
#include "Engine/Tracking/TrackingGetUpAmp3D.hpp"
#include "Engine/Tracking/TrackingScene3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979f;
// A queda: os musculos a esta fracao da rigidez do rastreador, segurando a pose sorteada com este ruido (rad), de
// 0,9 a 1,5 s; a pose comeca com o ponto mais baixo de 0,35 a 0,85 m do chao.
constexpr float FallStiffness = 0.15f;
constexpr float FallPoseNoise = 0.35f;
// Caido (31/10, o evitar a queda; ele: "nao deixar o peito ou costas ou cabeca tocar o chao... que ele use os joelhos,
// bracos enfim para evitar a queda e se recobrar"): o chao tocado pelo tronco (pelve, abdomen, peito), pelo pescoco ou
// pela cabeca; os pes, as pernas e os bracos podem apoiar. Cair depois de ja ter ficado de pe custa isto, uma vez por
// queda (alem do potencial que desce).
constexpr float FallPenalty = 10.0f;
// O levantar em movimento: a troca de pe por este tanto (s), mesmo andando, e a parceira correndo ate o alvo ate esta
// velocidade (m/s; a do treino dela).
constexpr float MovingHandoffSeconds = 0.2f;
constexpr float PartnerRunSpeed = 3.5f;
// A flexao na queda vale ate este tanto depois do primeiro toque do tronco no chao (s): o chegar ao chao.
constexpr float FlexAfterTouchSeconds = 0.75f;
// Os pares da arrancada agachada pesam isto no discriminador (a referencia do levantar em movimento).
constexpr int SprintStartPairWeight = 6;
// O levantar em movimento: chegou ao alvo neste raio (m), e dai o pedido vira ficar parado.
constexpr float ArriveRadius = 1.0f;
// Recomposto depois do empurrao do evitar: de pe, parecido com o parado e quase parado. (0,5 na pose de 21h25, o largo
// mais o estreito: a soma dos angulos^2 ~0,55; era 0,6 so no largo, a soma ~1,0.)
constexpr float RecomposedPose = 0.5f;
constexpr float RecomposedSpeed = 0.3f;
// A rl_final: os passos de cada faixa (0 andar, 1 trote, 2 corrida, 3 sprint) - o clipe em ciclo e o rumo do
// deslocamento dele em relacao ao corpo (0 a frente, de 45 em 45 graus para a esquerda). O primeiro de cada faixa e o da
// avaliacao. Na troca do pedido, parar tem esta chance.
struct GaitEntry {
    const char* clip;
    std::uint32_t band;
    std::uint32_t heading;
};
constexpr GaitEntry GaitTable[] {
    { "ual_walk", 0, 0 }, { "walk_strafe_left", 0, 2 }, { "walk_backward", 0, 4 }, { "walk_strafe_right", 0, 6 },
    { "trot_forward", 1, 0 }, { "trot_forward_left", 1, 1 }, { "jog_strafe_left", 1, 2 }, { "trot_backward_left", 1, 3 },
    { "trot_backward", 1, 4 }, { "trot_backward_right", 1, 5 }, { "jog_strafe_right", 1, 6 }, { "trot_forward_right", 1, 7 },
    { "mixamo_running", 1, 0 },
    { "run_forward_medium", 2, 0 }, { "jog", 2, 0 }, { "jog_backward", 2, 4 }, { "run_backward", 2, 4 },
    { "sprint_forward", 3, 0 }, { "mixamo_sprint", 3, 0 }, { "sprint", 3, 0 }, { "sprint_maximo", 3, 0 },
    { "sprint_strafe_left", 3, 2 }, { "sprint_backward", 3, 4 }, { "sprint_strafe_right", 3, 6 },
};
constexpr float CommandStopChance = 0.15f;

// Uma rotacao uniforme (Shoemake, "Uniform random rotations", 1992).
Quaternion uniformRotation(std::mt19937& rng) {
    std::uniform_real_distribution<float> u(0.0f, 1.0f);
    const float u1 = u(rng), u2 = 2.0f * Pi * u(rng), u3 = 2.0f * Pi * u(rng);
    const float a = std::sqrt(1.0f - u1), b = std::sqrt(u1);
    return Quaternion { a * std::sin(u2), a * std::cos(u2), b * std::sin(u3), b * std::cos(u3) }.normalized();
}

float planarAngle(Vec3 v) { return std::atan2(v.y, v.x); }

float wrapAngle(float a) {
    while (a > Pi) a -= 2.0f * Pi;
    while (a < -Pi) a += 2.0f * Pi;
    return a;
}

} // namespace

struct TrackingGetUpAmpTraining3D::Impl {
    TrackingGetUpAmpSettings3D settings;
    std::shared_ptr<const AthleteBundle3D> bundle;
    const RagdollProfile3D* profile = nullptr;
    // Os clipes do estilo (os de levantar primeiro, o parado, depois os da arrancada e da corrida) e as referencias.
    std::vector<std::unique_ptr<AnimationClip3D>> clips;
    std::vector<std::unique_ptr<TrackingClipReference3D>> references;
    std::size_t standingIndex = 0;
    std::size_t standingMirrorIndex = 0;
    // A arrancada agachada (crouched_to_sprint), quando carregada: o peso maior no estilo. E os clipes da arrancada e
    // da corrida, os comecos do tipo 5 (o RSI do DeepMimic/AMP: ele vive os estados de corrida e aprende a seguir).
    std::size_t sprintStartIndex = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> runStartIndices;
    // Os clipes do trote por rumo (os 8) e os do andar (a frente, a esquerda, de costas e a direita; as diagonais nao
    // tem), quando carregados.
    std::vector<std::size_t> headingIndices;
    std::array<std::size_t, 8> walkIndices;
    // A rl_final: os passos carregados de cada faixa (o indice do clipe e o rumo do deslocamento, GaitTable).
    struct Gait {
        std::size_t clip = 0;
        std::uint32_t heading = 0;
    };
    std::array<std::vector<Gait>, 4> gaits;
    // A velocidade de deslocamento que falta aos clipes no lugar (os pares do discriminador): constante (corrida) ou
    // em rampa desde zero (a arrancada, com o deslocamento da fonte), para a frente do clipe.
    struct Travel {
        float speed = 0.0f;
        float rampDisplacement = 0.0f;
        float direction = 0.0f;
    };
    std::vector<Travel> travel;
    std::unique_ptr<TrackingGetUpContract3D> contract;
    std::unique_ptr<TrackingAmpFeatures3D> features;
    std::unique_ptr<TrackingGetUpPostureProbe3D> posture;
    std::vector<std::string> observationNames;
    std::uint32_t rootLink = 0;
    std::uint32_t footLinks[2] { 0, 0 };
    // A flexao na queda: o peito alto, as maos e as canelas (a origem delas e o joelho).
    std::uint32_t upperChestLink = 0;
    std::uint32_t handLinks[2] { 0, 0 };
    std::uint32_t shinLinks[2] { 0, 0 };
    // As partes que podem tocar o chao sem contar como caido: as pernas e os bracos (nao o tronco, o pescoco e a cabeca).
    std::vector<bool> allowedContact;
    // O apoio de protecao: as maos, os bracos e as pernas (os joelhos), sem os pes.
    std::vector<bool> supportContact;
    std::uint32_t workers = 1;
    std::shared_ptr<TaskScheduler> scheduler;
    std::unique_ptr<PhysicsEngine3D> engine;
    MaterialLibrary materials;

    struct World {
        std::unique_ptr<PhysicsScene3D> scene;
        RagdollHandle3D handle;
        std::mt19937 rng;
        float time = 0.0f;
        TrackingClipFrame3D frame;
        std::vector<float> pose;
        std::vector<RagdollDriveTarget3D> drives;
        const RagdollState3D* body = nullptr;
        std::vector<float> observation;
        std::vector<float> finalObservation;
        std::vector<float> ampPair;
        std::uint32_t startKind = 0;
        std::uint32_t startSide = 0;
        float startHead = 0.0f;
        float stoodAt = -1.0f;
        double rewardSum = 0.0;
        std::uint32_t decisions = 0;
        // O empurrao (o de pe do jogo de pes ou o forte do evitar): a forca, os passos de fisica que faltam, o proximo e
        // quando o ultimo acaba (s).
        Vec3 pushForce;
        int pushTicks = 0;
        float nextPushAt = -1.0f;
        float lastPushEnd = -1.0e9f;
        // O comando (o levantar em movimento): o alvo no chao e a velocidade pedida (0: ficar onde esta).
        Vec3 target;
        float commandSpeed = 0.0f;
        float startCommandSpeed = 0.0f;
        float reachedAt = -1.0f;
        double towardSum = 0.0;
        std::uint32_t towardSamples = 0;
        // O evitar a queda: quando vem o empurrao forte, quando ele acaba e quando se recompos.
        float avoidAt = -1.0f;
        float avoidEnd = -1.0f;
        float recomposedAt = -1.0f;
        // Caido agora, e se ja caiu depois de ter ficado de pe.
        bool down = false;
        bool fellAfterStanding = false;
        // O rumo da arrancada agachada (rad).
        float sprintYaw = 0.0f;
        // A locomocao (os rumos e o giro): o rumo pedido (0 a 7), o do deslocamento e o do corpo (rad), quando o pedido
        // para (s; < 0: nao para), no giro a velocidade (rad/s, com o sinal), quanto falta do giro (rad) e o fim da pausa
        // (s; < 0: girando), e o erro do rumo do corpo de pe (a soma, rad).
        bool locomotion = false;
        std::uint32_t heading = 0;
        float moveYaw = 0.0f;
        float faceYaw = 0.0f;
        float stopAt = -1.0f;
        float turnRate = 0.0f;
        float turnLeft = 0.0f;
        float pauseUntil = -1.0f;
        double faceErrorSum = 0.0;
        std::uint32_t faceErrorSamples = 0;
        // A rl_final: a faixa do passo de agora e a do comeco, e a proxima troca do pedido (s; < 0: sem troca).
        std::uint32_t band = 1;
        std::uint32_t startBand = 1;
        float nextCommandAt = -1.0f;
        // A troca de rede ao ficar de pe: ha quanto tempo ele esta de pe e firme (s) e quando trocou (s; < 0: nao).
        float firmSeconds = 0.0f;
        float handoffAt = -1.0f;
        // A parceira depois da troca (quanto dirigiu, s; < 0: sem) e se o tronco ja tocou o chao.
        float partnerDrove = -1.0f;
        bool partnerFell = false;
        bool touchedDown = false;
        // A protecao na queda: se esta protegendo-se agora e se ja se apoiou.
        bool protecting = false;
        bool supported = false;
        // A flexao na queda: ate quando vale depois do toque (s; < 0: sem toque ainda), a soma e as amostras.
        float flexUntil = -1.0f;
        double flexSum = 0.0;
        std::uint32_t flexSamples = 0;
        // O cenario sujo do episodio: a parede e o peso (desfeitos no comeco seguinte).
        std::vector<PhysicsBodyHandle3D> obstacles;
        bool obstructed = false;
        // O premio do passo: o tempo no ar de cada pe (s) e se ele estava no chao.
        float footAir[2] { 0.0f, 0.0f };
        bool footDown[2] { true, true };
        std::uint32_t index = 0;
    };
    std::vector<World> worlds;
    // As quedas da locomocao (setFallStarts) e a rede da locomocao parceira (setLocomotionPartner).
    std::vector<std::vector<PhysicsBodyState3D>> fallStarts;
    std::shared_ptr<const LocomotionPartner3D> partner;
    std::vector<TrackingGetUpAmpEpisode3D> episodes;
    std::mutex episodeMutex;

    void settle(World& w, int ticks) {
        for (int k = 0; k < ticks; ++k) {
            w.scene->setRagdollActiveDriveTargets(w.handle, w.drives, false);
            if (w.pushTicks > 0) {
                // (Repartido como o do B19: pelve, peito e peito alto.)
                w.scene->applyRagdollLinkForce(w.handle, 0, w.pushForce * 0.20f, {});
                w.scene->applyRagdollLinkForce(w.handle, 2, w.pushForce * 0.32f, {});
                w.scene->applyRagdollLinkForce(w.handle, 3, w.pushForce * 0.48f, {});
                --w.pushTicks;
            }
            w.scene->simulate(TickSeconds);
        }
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // A queda: uma pose de um clipe (qualquer um do estilo) num instante sorteado, o corpo inteiro girado num sentido
    // sorteado em torno do centro de massa, erguido, com giro e deriva sorteados, e solto com os musculos fracos
    // segurando a pose com ruido ate assentar. Se nao caiu (a cabeca acima de 1 m), outra.
    void beginFall(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        std::normal_distribution<float> gauss(0.0f, 1.0f);
        for (int attempt = 0; attempt < 4; ++attempt) {
            const auto& reference = *references[std::uniform_int_distribution<std::size_t>(0, references.size() - 1)(w.rng)];
            reference.frame(u(w.rng) * reference.durationSeconds(), {}, 0.0f, w.frame);
            const Quaternion turn = uniformRotation(w.rng);
            const Vec3 center = w.frame.centerOfMass;
            float lowest = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < w.frame.links.size(); ++i) {
                PhysicsBodyState3D& s = w.frame.links[i];
                s.position = center + turn.rotate(s.position - center);
                s.orientation = (turn * s.orientation).normalized();
                lowest = std::min(lowest, rlLinkCenterOfMass3D(profile->links[i], s).z);
            }
            const Vec3 lifted = center + Vec3 { 0.0f, 0.0f, 0.35f + 0.5f * u(w.rng) - lowest };
            const Vec3 spin = Vec3 { gauss(w.rng), gauss(w.rng), gauss(w.rng) } * 1.5f;
            const Vec3 drift { 2.0f * u(w.rng) - 1.0f, 2.0f * u(w.rng) - 1.0f, 0.0f };
            for (PhysicsBodyState3D& s : w.frame.links) {
                s.position = s.position + (lifted - center);
                s.linearVelocity = drift + cross(spin, s.position - lifted);
                s.angularVelocity = spin;
            }
            w.scene->resetRagdollState(w.handle, w.frame.links);
            contract->actionOf(w.frame.coordinates, w.pose);
            for (float& value : w.pose) value += FallPoseNoise * (2.0f * u(w.rng) - 1.0f);
            contract->driveTargets(w.pose, w.drives);
            for (RagdollDriveTarget3D& drive : w.drives) drive.stiffnessScale *= FallStiffness;
            settle(w, static_cast<int>((0.9f + 0.6f * u(w.rng)) / TickSeconds));
            if (posture->measure(w.body->links).headHeight < 1.0f) return;
        }
    }

    // O evitar a queda com a parceira (01/11, ele: "o evitar queda precisa saber operar em conjunto com a outra rede nos
    // momentos de queda iminente"): de pe, ela leva o empurrao forte (o do evitar, de 0,3 a 1,0 s depois do comeco) e a
    // rede do levantar assume na queda iminente dela, com o resto do empurrao (pushLeft, s; a forca em pushForce). Se ela
    // aguenta sozinha ate 1,5 s depois do empurrao, outro (ate 3 vezes); sem a queda iminente, false.
    bool beginPartnerAvoid(World& w, float& pushLeft, Vec3& pushForce) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        for (int attempt = 0; attempt < 3; ++attempt) {
            if (attempt > 0) beginFromClip(w, true);
            LocomotionPartner3D::Push push;
            push.startSeconds = 0.3f + 0.7f * u(w.rng);
            const float angle = 2.0f * Pi * u(w.rng);
            float newtons = 0.0f;
            if (u(w.rng) < settings.avoidHoldFraction) {
                newtons = settings.avoidHoldMinimumNewtons
                    + (settings.avoidHoldMaximumNewtons - settings.avoidHoldMinimumNewtons) * u(w.rng);
                push.seconds = settings.avoidHoldMinimumSeconds
                    + (settings.avoidHoldMaximumSeconds - settings.avoidHoldMinimumSeconds) * u(w.rng);
            } else {
                newtons = settings.avoidMinimumNewtons + (settings.avoidMaximumNewtons - settings.avoidMinimumNewtons) * u(w.rng);
                push.seconds = 0.1f + 0.1f * u(w.rng);
            }
            push.force = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
            const LocomotionPartner3D::Result result =
                partner->drive(*w.scene, w.handle, {}, push.startSeconds + push.seconds + 1.5f, &push);
            w.body = w.scene->ragdollStateView(w.handle);
            if (result.fell) {
                pushLeft = result.pushLeft;
                pushForce = push.force;
                return true;
            }
        }
        return false;
    }

    // A queda da locomocao (01/11, a interligacao das redes): o corpo no instante em que a rede da locomocao caiu, com as
    // velocidades, trazido para a origem e, no treino, girado num rumo sorteado (na avaliacao, a do mundo, sem giro).
    void beginLocomotionFall(World& w) {
        const std::size_t pick = settings.evaluation
            ? w.index % fallStarts.size()
            : std::uniform_int_distribution<std::size_t>(0, fallStarts.size() - 1)(w.rng);
        const std::vector<PhysicsBodyState3D>& source = fallStarts[pick];
        const Vec3 root { source[rootLink].position.x, source[rootLink].position.y, 0.0f };
        const Quaternion turn = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
            settings.evaluation ? 0.0f : std::uniform_real_distribution<float>(-Pi, Pi)(w.rng));
        w.frame.links = source;
        for (PhysicsBodyState3D& s : w.frame.links) {
            s.position = turn.rotate(s.position - root);
            s.orientation = (turn * s.orientation).normalized();
            s.linearVelocity = turn.rotate(s.linearVelocity);
            s.angularVelocity = turn.rotate(s.angularVelocity);
        }
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // A arrancada e a corrida (31/10 18h25: so a arrancada agachada, curta, nao bastou - em 300 atualizacoes ele saia do
    // clipe e freava ate parar): um dos clipes da arrancada e da corrida, num instante sorteado (as arrancadas na
    // primeira metade), num rumo sorteado (o alvo vem perto dele), com a velocidade do clipe (a dos no lugar somada).
    void beginSprintStart(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const std::size_t index = runStartIndices[std::uniform_int_distribution<std::size_t>(0, runStartIndices.size() - 1)(w.rng)];
        const auto& reference = *references[index];
        const Travel& run = travel[index];
        const bool transition = !clips[index]->loops;
        w.sprintYaw = (2.0f * u(w.rng) - 1.0f) * Pi;
        const float t = u(w.rng) * (transition ? 0.5f : 1.0f) * reference.durationSeconds();
        reference.frame(t, {}, w.sprintYaw, w.frame);
        float speed = run.speed;
        if (run.rampDisplacement > 0.0f && reference.durationSeconds() > 0.0f)
            speed = 2.0f * run.rampDisplacement * t / (reference.durationSeconds() * reference.durationSeconds());
        if (speed > 0.0f) {
            const float heading = w.sprintYaw + run.direction;
            const Vec3 v { std::cos(heading) * speed, std::sin(heading) * speed, 0.0f };
            for (PhysicsBodyState3D& s : w.frame.links) s.linearVelocity = s.linearVelocity + v;
        }
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // Um instante sorteado de um clipe de levantar (o RSI do DeepMimic) ou, de pe, do parado, num rumo sorteado.
    void beginFromClip(World& w, bool standing) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const auto& reference = standing ? *references[u(w.rng) < 0.5f ? standingIndex : standingMirrorIndex]
            : *references[std::uniform_int_distribution<std::size_t>(0, std::size(GetUpClips) - 1)(w.rng)];
        const float last = std::max(0.0f, reference.durationSeconds() - 2.0f / 30.0f);
        reference.frame(u(w.rng) * last, {}, (2.0f * u(w.rng) - 1.0f) * Pi, w.frame);
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // Um rumo (os basicos do B21 nesta rede, 31/10 22h35): andar ou trotar no rumo pedido (em relacao ao rumo do comeco)
    // mantendo o rumo do corpo, de pe no parado em alerta ou, em metade dos trotes do treino, ja no clipe de trote do
    // rumo, na velocidade dele (o RSI; 22h55, tambem metade dos andares, nos rumos com clipe de andar: ele freava os
    // comecos em movimento e ficava parado). Em 20% do treino o pedido para no meio, de 2 a 5 s (o soltar do direcional
    // do B21: frear e ficar parado no rumo).
    // (01/11, a rl_final: as faixas. Um passo da faixa sorteada - o clipe e o rumo do deslocamento -, a velocidade pedida
    // a do clipe +-15%; em locomotionClipFraction ja no clipe, num instante sorteado e na velocidade dele, senao de pe no
    // parado em alerta. Na avaliacao com a faixa, para a frente, na velocidade do primeiro clipe dela, de pe; sem a faixa,
    // o meio do trote no rumo do mundo, como antes. Com a troca do pedido ligada, sem a parada do meio.)
    void beginHeading(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const bool changes = !settings.evaluation && settings.commandChangeMaximumSeconds > 0.0f;
        if (settings.evaluation) {
            const int band = settings.evaluationBand;
            if (band >= 0 && band < 4 && !gaits[static_cast<std::size_t>(band)].empty()) {
                const Gait& gait = gaits[static_cast<std::size_t>(band)].front();
                w.band = static_cast<std::uint32_t>(band);
                w.heading = gait.heading;
                w.commandSpeed = travel[gait.clip].speed;
            } else {
                w.band = 1;
                w.heading = settings.heading >= 0 ? static_cast<std::uint32_t>(settings.heading) % 8u : w.index % 8u;
                w.commandSpeed = 0.5f * (settings.trotMinimumSpeed + settings.trotMaximumSpeed);
            }
            beginFromClip(w, true);
        } else {
            const Gait& gait = pickGait(w);
            w.heading = gait.heading;
            w.commandSpeed = travel[gait.clip].speed * (0.85f + 0.3f * u(w.rng));
            if (u(w.rng) < settings.locomotionClipFraction) startInClip(w, gait.clip);
            else beginFromClip(w, true);
        }
        w.faceYaw = bodyYaw(w);
        w.moveYaw = w.faceYaw + 0.25f * Pi * static_cast<float>(w.heading);
        w.stopAt = !settings.evaluation && !changes && u(w.rng) < 0.2f ? 2.0f + 3.0f * u(w.rng) : -1.0f;
        w.startBand = w.band;
        w.nextCommandAt = changes ? settings.commandChangeMinimumSeconds
                + (settings.commandChangeMaximumSeconds - settings.commandChangeMinimumSeconds) * u(w.rng)
                                  : -1.0f;
    }

    // Um passo pelas fatias das faixas (andar, corrida, sprint; o resto trota), uniforme entre os carregados da faixa.
    const Gait& pickGait(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const float draw = u(w.rng);
        std::uint32_t band = 1;
        if (draw < settings.walkFraction) band = 0;
        else if (draw < settings.walkFraction + settings.runFraction) band = 2;
        else if (draw < settings.walkFraction + settings.runFraction + settings.sprintFraction) band = 3;
        if (gaits[band].empty()) band = 1;
        w.band = band;
        const std::vector<Gait>& list = gaits[band];
        return list[std::uniform_int_distribution<std::size_t>(0, list.size() - 1)(w.rng)];
    }

    // O corpo num instante sorteado de um clipe de locomocao, num rumo sorteado, na velocidade dele (o RSI).
    void startInClip(World& w, std::size_t clip) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const float yaw = (2.0f * u(w.rng) - 1.0f) * Pi;
        references[clip]->frame(u(w.rng) * references[clip]->durationSeconds(), {}, yaw, w.frame);
        const Travel& run = travel[clip];
        const Vec3 v { std::cos(yaw + run.direction) * run.speed, std::sin(yaw + run.direction) * run.speed, 0.0f };
        for (PhysicsBodyState3D& s : w.frame.links) s.linearVelocity = s.linearVelocity + v;
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // A troca do pedido no meio de um rumo (a rl_final: as transicoes): CommandStopChance para e fica no rumo do corpo,
    // senao um passo novo pelas fatias; nos passos para a frente o rumo pedido do corpo vira para um lado sorteado (ate
    // 180 graus, virar correndo), nos outros fica (o deslocamento muda em relacao a ele).
    void changeCommand(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if (u(w.rng) < CommandStopChance) {
            w.commandSpeed = 0.0f;
        } else {
            const Gait& gait = pickGait(w);
            w.heading = gait.heading;
            w.commandSpeed = travel[gait.clip].speed * (0.85f + 0.3f * u(w.rng));
            if (gait.heading == 0) w.faceYaw = wrapAngle(w.faceYaw + (2.0f * u(w.rng) - 1.0f) * Pi);
            w.moveYaw = w.faceYaw + 0.25f * Pi * static_cast<float>(w.heading);
        }
        w.nextCommandAt = w.time + settings.commandChangeMinimumSeconds
            + (settings.commandChangeMaximumSeconds - settings.commandChangeMinimumSeconds) * u(w.rng);
    }

    // O giro parado suave (o "giro" do B21: giros de 30-450 graus a 0,4-0,8 rad/s, para um lado sorteado, com pausas de
    // 1-3 s; na avaliacao, 0,6 rad/s o episodio todo, metade dos mundos para cada lado): de pe, o rumo pedido do corpo
    // gira e o deslocamento pedido e zero.
    void beginTurn(World& w) {
        beginFromClip(w, true);
        w.faceYaw = bodyYaw(w);
        w.moveYaw = w.faceYaw;
        nextTurn(w);
    }
    void nextTurn(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const bool left = settings.evaluation ? w.index % 2u == 0u : u(w.rng) < 0.5f;
        const float rate = settings.evaluation ? 0.6f : 0.4f + 0.4f * u(w.rng);
        w.turnRate = left ? rate : -rate;
        w.turnLeft = settings.evaluation ? 1.0e9f : (30.0f + 420.0f * u(w.rng)) * Pi / 180.0f;
        w.pauseUntil = -1.0f;
    }
    // O rumo pedido do giro, uma decisao adiante.
    void advanceTurn(World& w) {
        if (w.startKind != 7) return;
        if (w.pauseUntil >= 0.0f) {
            if (w.time >= w.pauseUntil) nextTurn(w);
            return;
        }
        const float step = std::min(std::abs(w.turnRate) * TicksPerDecision * TickSeconds, w.turnLeft);
        w.faceYaw = wrapAngle(w.faceYaw + std::copysign(step, w.turnRate));
        w.turnLeft -= step;
        if (w.turnLeft <= 0.0f) w.pauseUntil = w.time + 1.0f + 2.0f * std::uniform_real_distribution<float>(0.0f, 1.0f)(w.rng);
    }

    // A flexao do corpo agora (de -2 a 1): as maos a frente do peito e os joelhos a frente da pelve, cada um ate 0,25 m
    // (+1; atras, -1), na media, menos a coluna arqueada para tras (o peito inclinado para tras em relacao a pelve, ate
    // 30 graus: -1).
    float flexion(const RagdollState3D& body) const {
        const auto frame = [&](std::uint32_t i) {
            return (body.links[i].orientation * profile->links[i].modelOrientation.conjugate()).normalized();
        };
        const Quaternion pelvis = frame(rootLink), chest = frame(upperChestLink);
        const Vec3 pelvisForward = pelvis.rotate({ 1.0f, 0.0f, 0.0f }), chestForward = chest.rotate({ 1.0f, 0.0f, 0.0f });
        const auto ahead = [](float meters) { return std::clamp(meters / 0.25f, -1.0f, 1.0f); };
        float limbs = 0.0f;
        for (int side = 0; side < 2; ++side) {
            limbs += ahead(dot(body.links[handLinks[side]].position - body.links[upperChestLink].position, chestForward));
            limbs += ahead(dot(body.links[shinLinks[side]].position - body.links[rootLink].position, pelvisForward));
        }
        const float back = std::asin(std::clamp(-dot(chest.rotate({ 0.0f, 0.0f, 1.0f }), pelvisForward), -1.0f, 1.0f));
        return 0.25f * limbs - std::clamp(back / (30.0f * Pi / 180.0f), 0.0f, 1.0f);
    }

    // (01/11, o cenario sujo: so o que toca uma superficie de chao - o ombro na parede, ou o peso em cima, nao e caido.)
    bool fallen(const TrackingGetUpPosture3D& posture, const RagdollState3D& body) const {
        (void)posture;
        for (const RagdollContactPoint3D& contact : body.contacts)
            if (contact.linkIndex < allowedContact.size() && !allowedContact[contact.linkIndex] && contact.normal.z > 0.5f)
                return true;
        return false;
    }

    // O cenario sujo do comeco (so no treino): a parede ou caixa fixa encostada no corpo e o peso caindo em cima.
    void placeObstacles(World& w) {
        for (const PhysicsBodyHandle3D handle : w.obstacles)
            if (w.scene->contains(handle)) w.scene->destroyBody(handle);
        w.obstacles.clear();
        if (settings.evaluation || w.body->links.empty()) return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const Vec3 root = w.body->links[rootLink].position;
        if (u(w.rng) < settings.obstacleFraction) {
            const float angle = 2.0f * Pi * u(w.rng);
            const Vec3 direction { std::cos(angle), std::sin(angle), 0.0f };
            // (Do link mais longe nesse lado, para comecar encostada sem atravessar o corpo.)
            float reach = 0.0f;
            for (const PhysicsBodyState3D& link : w.body->links) reach = std::max(reach, dot(link.position - root, direction));
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = { 0.15f + 0.25f * u(w.rng), 0.5f + 1.0f * u(w.rng), 0.2f + 0.55f * u(w.rng) };
            shape.materialId = "concrete";
            PhysicsBodyDefinition3D wall;
            wall.motionType = PhysicsMotionType3D::Static;
            wall.position = root + direction * (reach + 0.04f + 0.4f * u(w.rng) + shape.halfExtents.x);
            wall.position.z = shape.halfExtents.z;
            wall.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, angle);
            wall.materialId = "concrete";
            w.obstacles.push_back(w.scene->createBody(wall, std::span<const PhysicsShape3D>(&shape, 1)));
        }
        if (u(w.rng) < settings.weightFraction) {
            const std::size_t link = std::uniform_int_distribution<std::size_t>(0, w.body->links.size() - 1)(w.rng);
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = { 0.12f + 0.06f * u(w.rng), 0.4f + 0.3f * u(w.rng), 0.1f + 0.05f * u(w.rng) };
            shape.materialId = "wood";
            PhysicsBodyDefinition3D weight;
            weight.motionType = PhysicsMotionType3D::Dynamic;
            weight.position = w.body->links[link].position + Vec3 { 0.0f, 0.0f, 0.3f + 0.3f * u(w.rng) };
            weight.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 2.0f * Pi * u(w.rng));
            weight.massKg = 20.0f + 40.0f * u(w.rng);
            weight.materialId = "wood";
            w.obstacles.push_back(w.scene->createBody(weight, std::span<const PhysicsShape3D>(&shape, 1)));
        }
        w.obstructed = !w.obstacles.empty();
    }

    void begin(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        // (Um sorteio so, como antes: com as fatias novas em 0, os mesmos comecos de antes.)
        // (Tambem na avaliacao, que nao usa o sorteio: as mesmas 64 quedas fixas de antes.)
        const float draw = u(w.rng);
        std::uint32_t kind = 0;
        if (settings.evaluation) {
            constexpr std::uint32_t Kinds[8] { 0u, 3u, 4u, 2u, 6u, 2u, 7u, 8u };
            kind = settings.evaluationKind < 8 ? Kinds[settings.evaluationKind] : 0u;
        } else {
            float edge = settings.standingStartFraction;
            if (draw < edge) kind = 2;
            else if (draw < (edge += settings.clipStartFraction)) kind = 1;
            else if (draw < (edge += settings.avoidFraction)) kind = 4;
            else if (draw < (edge += settings.movingFraction)) kind = 3;
            else if (draw < (edge += settings.sprintStartFraction)) kind = 5;
            else if (draw < (edge += settings.locomotionFraction)) kind = 6;
            else if (draw < (edge += settings.turnFraction)) kind = 7;
            else if (draw < (edge += settings.locomotionFallFraction)) kind = 8;
        }
        if (kind == 5 && runStartIndices.empty()) kind = 3;
        if (kind == 8 && fallStarts.empty()) kind = 0;
        // (O evitar a queda com a parceira: ela de pe primeiro; sem a queda iminente dela, uma queda sorteada.)
        float partnerPushLeft = -1.0f;
        Vec3 partnerPushForce;
        if (kind == 4 && partner && settings.partnerAvoid) {
            w.pushTicks = 0;
            beginFromClip(w, true);
            if (!beginPartnerAvoid(w, partnerPushLeft, partnerPushForce)) {
                kind = 0;
                partnerPushLeft = -1.0f;
            }
        }
        // (O que vem do episodio anterior zerado antes do comeco: a queda assenta sem o resto de um empurrao.)
        w.pushTicks = 0;
        w.commandSpeed = 0.0f;
        w.locomotion = kind == 6 || kind == 7;
        w.stopAt = -1.0f;
        w.pauseUntil = -1.0f;
        w.faceErrorSum = 0.0;
        w.faceErrorSamples = 0;
        w.firmSeconds = 0.0f;
        w.handoffAt = -1.0f;
        w.partnerDrove = -1.0f;
        w.partnerFell = false;
        w.touchedDown = false;
        w.protecting = false;
        w.supported = false;
        w.flexUntil = -1.0f;
        w.flexSum = 0.0;
        w.flexSamples = 0;
        w.footAir[0] = w.footAir[1] = 0.0f;
        w.footDown[0] = w.footDown[1] = true;
        w.startKind = kind;
        if (kind == 0 || kind == 3) beginFall(w);
        else if (kind == 5) beginSprintStart(w);
        else if (kind == 6) beginHeading(w);
        else if (kind == 7) beginTurn(w);
        else if (kind == 8) beginLocomotionFall(w);
        else if (partnerPushLeft < 0.0f) beginFromClip(w, kind != 1);
        const TrackingGetUpPosture3D start = posture->measure(w.body->links);
        w.startSide = kind == 6 ? w.heading : start.chestFacingUp < -0.5f ? 0u : start.chestFacingUp > 0.5f ? 1u : 2u;
        w.startHead = start.headHeight;
        w.time = 0.0f;
        w.stoodAt = -1.0f;
        w.rewardSum = 0.0;
        w.decisions = 0;
        w.nextPushAt = -1.0f;
        w.lastPushEnd = -1.0e9f;
        w.reachedAt = -1.0f;
        w.towardSum = 0.0;
        w.towardSamples = 0;
        w.avoidAt = -1.0f;
        w.avoidEnd = -1.0f;
        w.recomposedAt = -1.0f;
        w.down = fallen(start, *w.body);
        w.fellAfterStanding = false;
        if (kind == 3 || kind == 5) {
            // (Na arrancada, o alvo a ate 30 graus da frente do clipe: ele ja sai virado para o alvo.)
            const float angle = kind == 5 ? w.sprintYaw + (2.0f * u(w.rng) - 1.0f) * (Pi / 6.0f) : 2.0f * Pi * u(w.rng);
            const float distance =
                settings.targetMinimumMeters + (settings.targetMaximumMeters - settings.targetMinimumMeters) * u(w.rng);
            const Vec3 root = w.body->links[rootLink].position;
            w.target = { root.x + std::cos(angle) * distance, root.y + std::sin(angle) * distance, 0.0f };
            w.commandSpeed = settings.moveMinimumSpeed + (settings.moveMaximumSpeed - settings.moveMinimumSpeed) * u(w.rng);
        }
        w.startCommandSpeed = w.commandSpeed;
        if (kind == 4) w.avoidAt = 0.3f + 0.7f * u(w.rng);
        w.obstructed = false;
        placeObstacles(w);
        if (partnerPushLeft >= 0.0f) {
            w.avoidAt = -1.0f;
            w.avoidEnd = w.lastPushEnd = partnerPushLeft;
            if (partnerPushLeft > 0.0f) {
                w.pushForce = partnerPushForce;
                w.pushTicks = std::max(1, static_cast<int>(std::lround(partnerPushLeft / TickSeconds)));
            }
        }
    }

    // O empurrao de pe (31/10 09h, ele: depois de levantar falta o "jogo de pes de auto-equilibrio"): so no treino, nos
    // comecos sem alvo e sem o empurrao forte, depois de ja ter ficado de pe e enquanto esta de pe, um empurrao curto a
    // cada 1,5-3 s (o primeiro 1-2,5 s depois de ficar de pe), na faixa da configuracao por 0,1-0,2 s, num rumo
    // sorteado (150-450 N ate 12h35, 250-650 ate 15h05, 400-900 depois; cada degrau quando ele ja ficava de pe em ~100%).
    // Se cair, levanta de novo.
    // (31/10 21h25, ele: "botar um treino nele, para parado e aguentar empurroes medianos e fortes e ver como ele se sai
    // com auto-equilibrio": a faixa e o continuo da configuracao; na avaliacao do equilibrio parado, um so empurrao com a
    // regua do parado com empurrao forte do B21, para comparar as duas redes.)
    // (31/10 22h35, a locomocao: nos rumos e no giro, os empurroes em movimento, o primeiro de 1 a 3 s depois do comeco
    // e os outros a cada 2-4 s; na avaliacao, um so, de 2 a 3 s depois do comeco, com a velocidade ja pega.)
    void maybePush(World& w, const TrackingGetUpPosture3D& posture) {
        const bool balanceTest = settings.evaluation && settings.evaluationKind == 3;
        const bool movingTest = settings.evaluation && w.locomotion;
        const bool movingPushes = settings.movingPushMaximumNewtons > 0.0f || settings.movingPushHoldMaximumNewtons > 0.0f;
        if ((settings.evaluation && !balanceTest && !movingTest) || (w.startKind >= 3 && !w.locomotion)
            || (w.locomotion && !movingPushes) || w.stoodAt < 0.0f || !posture.standing || w.pushTicks > 0)
            return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if (w.nextPushAt < 0.0f) {
            w.nextPushAt = w.time + (movingTest ? 2.0f + u(w.rng) : w.locomotion ? 1.0f + 2.0f * u(w.rng) : 1.0f + 1.5f * u(w.rng));
            return;
        }
        if (w.time < w.nextPushAt) return;
        const float angle = 2.0f * Pi * u(w.rng);
        float newtons = 0.0f, seconds = 0.0f;
        if (balanceTest) {
            if (u(w.rng) < 0.3f) newtons = 205.0f + 70.0f * u(w.rng), seconds = 1.0f + 2.0f * u(w.rng);
            else newtons = 900.0f + 300.0f * u(w.rng), seconds = 0.1f + 0.1f * u(w.rng);
        } else if (w.locomotion) {
            if (u(w.rng) < settings.movingPushHoldFraction) {
                newtons = settings.movingPushHoldMinimumNewtons
                    + (settings.movingPushHoldMaximumNewtons - settings.movingPushHoldMinimumNewtons) * u(w.rng);
                seconds = 1.0f + u(w.rng);
            } else {
                newtons = settings.movingPushMinimumNewtons
                    + (settings.movingPushMaximumNewtons - settings.movingPushMinimumNewtons) * u(w.rng);
                seconds = 0.1f + 0.1f * u(w.rng);
            }
        } else if (u(w.rng) < settings.pushHoldFraction) {
            newtons = settings.pushHoldMinimumNewtons
                + (settings.pushHoldMaximumNewtons - settings.pushHoldMinimumNewtons) * u(w.rng);
            seconds = settings.pushHoldMinimumSeconds
                + (settings.pushHoldMaximumSeconds - settings.pushHoldMinimumSeconds) * u(w.rng);
        } else {
            newtons = settings.pushMinimumNewtons + (settings.pushMaximumNewtons - settings.pushMinimumNewtons) * u(w.rng);
            seconds = 0.1f + 0.1f * u(w.rng);
        }
        w.pushForce = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
        w.pushTicks = std::max(1, static_cast<int>(std::lround(seconds / TickSeconds)));
        w.lastPushEnd = w.time + seconds;
        w.nextPushAt = balanceTest || movingTest ? 1.0e9f
            : w.time + seconds + (w.locomotion ? 2.0f + 2.0f * u(w.rng) : 1.5f + 1.5f * u(w.rng));
    }

    // O empurrao forte do evitar a queda (31/10 17h, ele: "comeca de pe, leva um empurrao que com certeza vai leva-lo ao
    // chao e ai ele deve treinar conseguir evitar aquela queda e recompor-se o mais rapido possivel"): uma vez, de 0,3 a
    // 1,0 s depois do comeco, o curto ou (numa fatia) o continuo, num rumo sorteado. Tambem na avaliacao.
    void maybeAvoidPush(World& w) {
        if (w.startKind != 4 || w.avoidAt < 0.0f || w.time < w.avoidAt || w.pushTicks > 0) return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const float angle = 2.0f * Pi * u(w.rng);
        float newtons = 0.0f, seconds = 0.0f;
        if (u(w.rng) < settings.avoidHoldFraction) {
            newtons = settings.avoidHoldMinimumNewtons
                + (settings.avoidHoldMaximumNewtons - settings.avoidHoldMinimumNewtons) * u(w.rng);
            seconds = settings.avoidHoldMinimumSeconds
                + (settings.avoidHoldMaximumSeconds - settings.avoidHoldMinimumSeconds) * u(w.rng);
        } else {
            newtons = settings.avoidMinimumNewtons + (settings.avoidMaximumNewtons - settings.avoidMinimumNewtons) * u(w.rng);
            seconds = 0.1f + 0.1f * u(w.rng);
        }
        w.pushForce = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
        w.pushTicks = std::max(1, static_cast<int>(std::lround(seconds / TickSeconds)));
        w.avoidEnd = w.time + seconds;
        w.lastPushEnd = w.avoidEnd;
        w.avoidAt = -1.0f;
    }

    // O rumo do alvo em relacao ao corpo (rad) e a distancia (m), no chao.
    std::pair<float, float> targetBearing(const World& w) const {
        const PhysicsBodyState3D& root = w.body->links[rootLink];
        const Vec3 to { w.target.x - root.position.x, w.target.y - root.position.y, 0.0f };
        const float yaw = trackingBodyYaw3D(profile->links[rootLink], root);
        return { wrapAngle(planarAngle(to) - yaw), std::hypot(to.x, to.y) };
    }

    // O rumo do deslocamento pedido e o do corpo (rad, no mundo): no levantar em movimento, os dois para o alvo; nos
    // rumos, o pedido e o do comeco; no giro, o que gira.
    float moveDirection(const World& w) const {
        if (w.locomotion) return w.moveYaw;
        const PhysicsBodyState3D& root = w.body->links[rootLink];
        return planarAngle({ w.target.x - root.position.x, w.target.y - root.position.y, 0.0f });
    }
    float faceDirection(const World& w) const { return w.locomotion ? w.faceYaw : moveDirection(w); }
    float bodyYaw(const World& w) const { return trackingBodyYaw3D(profile->links[rootLink], w.body->links[rootLink]); }
    // (01/11 00h, o Codex: no giro, max(0, cos) fica plano alem de 90 graus - o alvo gira e foge, e dai ele nao tinha
    // sinal nenhum para virar; o erro ficou em ~95 graus por 1,5 h. No giro, o alinhamento continuo (1 + cos) / 2.)
    float facing(const World& w) const {
        const float c = std::cos(wrapAngle(faceDirection(w) - bodyYaw(w)));
        return w.startKind == 7 ? 0.5f * (1.0f + c) : std::max(0.0f, c);
    }

    void observe(World& w) const {
        const std::size_t b = contract->bodyObservationSize();
        contract->observeBody(w.body->links, std::span<float>(w.observation).first(b));
        // (O contrato v3: o corpo contra o chao, depois do comando; o chao do treino e o plano z = 0.)
        contract->observeGround(w.body->links, w.body->interactions, 0.0f,
            std::span<float>(w.observation).subspan(b + CommandSize, TrackingGetUpContract3D::GroundObservationSize));
        float* c = w.observation.data() + b;
        const bool moving = w.commandSpeed > 0.0f;
        if (!moving && !w.locomotion) {
            c[0] = 1.0f, c[1] = 0.0f, c[2] = 0.0f, c[3] = 0.0f, c[4] = 1.0f, c[5] = 0.0f;
            return;
        }
        // (Parado com rumo pedido - o giro, o rumo depois de parar: o deslocamento como sem pedido, o rumo do corpo sim.)
        const float yaw = bodyYaw(w);
        const float move = moving ? wrapAngle(moveDirection(w) - yaw) : 0.0f, face = wrapAngle(faceDirection(w) - yaw);
        c[0] = std::cos(move);
        c[1] = std::sin(move);
        c[2] = !moving ? 0.0f : w.locomotion ? 1.0f : std::min(targetBearing(w).second, 10.0f) / 10.0f;
        c[3] = w.commandSpeed / 5.0f;
        c[4] = std::cos(face);
        c[5] = std::sin(face);
    }

    // O potencial do rumo com pedido: virar para o rumo pedido enquanto sobe (a altura vezes o alinhamento).
    float facePotential(const World& w, const TrackingGetUpPosture3D& posture) const {
        if (w.commandSpeed <= 0.0f && !w.locomotion) return 0.0f;
        const float rise = std::clamp((posture.headHeight - 0.15f) / 1.35f, 0.0f, 1.0f);
        return 5.0f * rise * facing(w);
    }

    // A nota de estado com o pedido de ir ao alvo, so de pe: a velocidade rumo ao alvo 0,6 e, na proporcao dela, virado
    // para ele 0,25 e sem andar de lado 0,15 (eram 0,5, 0,3 e 0,2 inteiros). (31/10 18h: a velocidade era exp(-(v - pedida)^2), que nao dava gradiente parado - pedindo
    // 2,5-5 m/s, parado ou a 1 m/s dava ~0 - e em 250 atualizacoes ele ficou parado virado para o alvo. Agora uma rampa:
    // 0 parado, 1 na pedida, caindo acima dela e negativa se afasta.)
    // (Nos rumos, o rumo do corpo e o do comeco: andar e trotar de costas e de lado sem virar para a direcao.)
    float moveReward(World& w, const TrackingGetUpPosture3D& posture) {
        if (!posture.standing) return 0.0f;
        const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
        const float toward = moveDirection(w);
        const float along = v.x * std::cos(toward) + v.y * std::sin(toward);
        const float across = -v.x * std::sin(toward) + v.y * std::cos(toward);
        w.towardSum += along;
        ++w.towardSamples;
        const float ratio = along / w.commandSpeed;
        // (01/11 00h05, ele: "nenhum passo ate agora": a rampa concava ate a pedida - a raiz -, para um passo pequeno ja
        // compensar o estilo e o tranco de um andar ainda desajeitado; para tras e acima da pedida, como antes.)
        const float speed = ratio < 0.0f ? std::max(-1.0f, ratio) : ratio <= 1.0f ? std::sqrt(ratio)
                                                                                  : std::max(0.0f, 2.0f - ratio);
        // (22h55, ele: "garantir que ele consiga ter o estimulo para se mover, e nao ficar apenas parado": parado com o
        // pedido, o rumo e o sem andar de lado davam 0,5 de 1 - nos rumos a velocidade caia, 0,136 -> 0,098 m/s em 160
        // atualizacoes, e no levantar em movimento ficou ~0 por horas. Agora a velocidade 0,6 e os outros dois na
        // proporcao dela: parado com o pedido vale 0, na pedida 1.)
        const float progress = std::clamp(ratio, 0.0f, 1.0f);
        return 0.6f * speed + progress * (0.25f * facing(w) + 0.15f * std::exp(-across * across));
    }

    void decide(World& w, std::size_t index, std::span<const float> action, float& reward, std::uint8_t& terminated,
        std::uint8_t& truncated) {
        const std::size_t a = features->size();
        features->compute(w.body->links, std::span<float>(w.ampPair).first(a));
        const TrackingGetUpPosture3D was = posture->measure(w.body->links);
        const float before = posture->risePotential(was) + posture->turnPotential(was) + facePotential(w, was);
        maybePush(w, was);
        maybeAvoidPush(w);
        contract->driveTargets(action, w.drives);
        settle(w, TicksPerDecision);
        w.time += TicksPerDecision * TickSeconds;
        features->compute(w.body->links, std::span<float>(w.ampPair).subspan(a, a));
        const TrackingGetUpPosture3D now = posture->measure(w.body->links);
        // O alvo alcancado: dai o pedido e ficar parado (a nota do parado, com a pose).
        if (!w.locomotion && w.commandSpeed > 0.0f && targetBearing(w).second < ArriveRadius) {
            w.commandSpeed = 0.0f;
            w.reachedAt = w.time;
        }
        if (w.stopAt >= 0.0f && w.time >= w.stopAt) {
            w.commandSpeed = 0.0f;
            w.stopAt = -1.0f;
        }
        if (w.startKind == 6 && w.nextCommandAt >= 0.0f && w.time >= w.nextCommandAt) changeCommand(w);
        advanceTurn(w);
        // (Parado com rumo pedido: a nota do parado 0,7 e o rumo do corpo 0,3, de pe. A coluna so quieto: sem empurrao e
        // 1 s depois do ultimo.)
        const bool quiet = w.pushTicks == 0 && w.time >= w.lastPushEnd + 1.0f;
        float state = w.commandSpeed > 0.0f ? moveReward(w, now)
            : w.locomotion ? 0.7f * posture->standingReward(now, w.body->links, quiet) + (now.standing ? 0.3f * facing(w) : 0.0f)
                           : posture->standingReward(now, w.body->links, quiet);
        if (w.locomotion && now.standing) {
            w.faceErrorSum += std::abs(wrapAngle(faceDirection(w) - bodyYaw(w)));
            ++w.faceErrorSamples;
        }
        // O premio do passo (com o pedido de deslocamento, de pe): cada pe que pousa depois de um tempo no ar.
        if (settings.stepReward > 0.0f) {
            float steps = 0.0f;
            for (int f = 0; f < 2; ++f) {
                const bool down = std::any_of(w.body->contacts.begin(), w.body->contacts.end(),
                    [&](const RagdollContactPoint3D& c) { return c.linkIndex == footLinks[f]; });
                if (down && !w.footDown[f] && w.commandSpeed > 0.0f && now.standing)
                    steps += std::min(w.footAir[f], 0.5f) - 0.15f;
                w.footAir[f] = down ? 0.0f : w.footAir[f] + TicksPerDecision * TickSeconds;
                w.footDown[f] = down;
            }
            state += settings.stepReward * steps;
        }
        // (O gamma do treinador, 0,99, no potencial.)
        reward = state + 0.99f * (posture->risePotential(now) + posture->turnPotential(now) + facePotential(w, now)) - before;
        const bool down = fallen(now, *w.body);
        if (down && !w.down && w.stoodAt >= 0.0f) {
            reward -= FallPenalty;
            w.fellAfterStanding = true;
        }
        // (O evitar a queda de verdade nos comecos da queda iminente: a primeira vez do tronco no chao, antes de ficar de
        // pe - depois, a FallPenalty.)
        if (down && !w.touchedDown && w.stoodAt < 0.0f && (w.startKind == 4 || w.startKind == 8))
            reward -= settings.touchDownPenalty;
        // (A protecao: nesses comecos, antes do tronco no chao e de ficar de pe, o primeiro apoio das maos, dos bracos ou
        // dos joelhos vale uma vez.)
        const bool protecting = (w.startKind == 4 || w.startKind == 8) && !w.touchedDown && !down && w.stoodAt < 0.0f;
        if (protecting && !w.supported) {
            for (const RagdollContactPoint3D& contact : w.body->contacts)
                if (contact.linkIndex < supportContact.size() && supportContact[contact.linkIndex]) w.supported = true;
            if (w.supported) reward += settings.supportBonus;
        }
        w.protecting = protecting;
        // (A flexao: na protecao e ate FlexAfterTouchSeconds depois do primeiro toque do tronco.)
        if (down && !w.touchedDown && w.stoodAt < 0.0f && (w.startKind == 4 || w.startKind == 8))
            w.flexUntil = w.time + FlexAfterTouchSeconds;
        if (protecting || (w.flexUntil >= 0.0f && w.time <= w.flexUntil)) {
            const float flex = flexion(*w.body);
            reward += settings.flexReward * flex;
            w.flexSum += flex;
            ++w.flexSamples;
        }
        w.touchedDown = w.touchedDown || down;
        w.down = down;
        if (w.startKind == 4 && w.avoidEnd >= 0.0f && w.time >= w.avoidEnd && w.recomposedAt < 0.0f && now.standing
            && !w.fellAfterStanding && posture->idlePoseMatch(w.body->links) > RecomposedPose) {
            const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
            if (std::hypot(v.x, v.y) < RecomposedSpeed) w.recomposedAt = w.time;
        }
        w.rewardSum += reward;
        ++w.decisions;
        if (w.stoodAt < 0.0f && now.standing) w.stoodAt = w.time;
        terminated = 0;
        // A troca de rede ao ficar de pe (so no treino): de pe e firme - a pelve perto da altura de pe do perfil, a
        // pelve em pe e quase parado, como a entrega no jogo - por 0,5 s, depois do chao ou do empurrao do evitar.
        if (settings.handoffOnStand && !settings.evaluation) {
            const bool eligible = w.startKind == 4 ? w.avoidEnd >= 0.0f && w.time >= w.avoidEnd : w.stoodAt >= 0.0f;
            const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
            // (01/11, ele: no levantar em movimento, "a troca do modelo de RL assim que ele comeca a ficar em pe" e a da
            // locomocao corre para o alvo: de pe por MovingHandoffSeconds, mesmo andando; nos outros, de pe e firme.)
            const bool moving = w.startKind == 3 || w.startKind == 5;
            const bool firm = moving ? now.standing && !w.fellAfterStanding
                : now.pelvisHeight > 0.85f * profile->standingRootHeightMeters && now.upright > 0.85f
                    && std::hypot(v.x, v.y) < 0.5f && !w.fellAfterStanding;
            w.firmSeconds = eligible && firm ? w.firmSeconds + TicksPerDecision * TickSeconds : 0.0f;
            if (w.firmSeconds >= (moving ? MovingHandoffSeconds : 0.5f)) w.handoffAt = w.time;
        }
        // Depois da troca, a parceira dirige parada (01/11): se ela cair no tempo dela, a troca foi ruim - a punicao e o
        // fim do episodio -; senao, o corte de sempre, com o valor do estado da troca.
        const bool partnered = w.handoffAt >= 0.0f && partner && !settings.evaluation;
        if (partnered) {
            observe(w);
            w.finalObservation = w.observation;
            // (No levantar em movimento, ela corre ate o alvo na velocidade pedida - ate PartnerRunSpeed, a do treino dela.)
            Vec3 velocity;
            if (w.startKind == 3 || w.startKind == 5) {
                const Vec3 root = w.body->links[rootLink].position;
                const Vec3 to { w.target.x - root.x, w.target.y - root.y, 0.0f };
                const float distance = std::hypot(to.x, to.y);
                if (distance > ArriveRadius)
                    velocity = to * (std::min(w.startCommandSpeed, PartnerRunSpeed) / distance);
            }
            const LocomotionPartner3D::Result result =
                partner->drive(*w.scene, w.handle, w.drives, settings.partnerSeconds, nullptr, velocity);
            w.body = w.scene->ragdollStateView(w.handle);
            w.partnerDrove = result.seconds;
            w.partnerFell = result.fell;
            if (result.fell) reward -= settings.partnerFallPenalty;
        }
        // (A rl_final: nos rumos e no giro, o tronco no chao encerra o episodio - o early termination do DeepMimic: cair
        // custa o resto do episodio - e o tempo deles e o locomotionSeconds.)
        const bool fellMoving = settings.locomotionTermination && w.locomotion && down;
        const float limit = w.locomotion && settings.locomotionSeconds > 0.0f ? settings.locomotionSeconds
                                                                              : settings.episodeSeconds;
        terminated = w.partnerFell || fellMoving ? 1 : 0;
        truncated = !terminated && (w.time >= limit - 1e-4f || w.handoffAt >= 0.0f) ? 1 : 0;
        if (truncated || terminated) {
            if (!partnered) {
                observe(w);
                w.finalObservation = w.observation;
            }
            TrackingGetUpAmpEpisode3D episode;
            episode.world = static_cast<std::uint32_t>(index);
            episode.startKind = w.startKind;
            episode.startSide = w.startSide;
            episode.startHeadHeight = w.startHead;
            episode.seconds = w.time;
            episode.firstStandSeconds = w.stoodAt;
            episode.meanTaskReward = static_cast<float>(w.rewardSum / std::max<std::uint32_t>(1, w.decisions));
            episode.headHeight = now.headHeight;
            episode.pelvisHeight = now.pelvisHeight;
            episode.standing = now.standing;
            episode.commandSpeed = w.startCommandSpeed;
            episode.reachedSeconds = w.reachedAt;
            episode.meanTowardSpeed = w.towardSamples ? static_cast<float>(w.towardSum / w.towardSamples) : 0.0f;
            episode.fellAfterStanding = w.fellAfterStanding;
            episode.recomposedSeconds = w.recomposedAt >= 0.0f ? w.recomposedAt - w.avoidEnd : -1.0f;
            episode.endPoseMatch = now.standing ? posture->idlePoseMatch(w.body->links) : 0.0f;
            episode.endSpineMatch = now.standing ? posture->idleSpineMatch(w.body->links) : 0.0f;
            episode.handoffSeconds = w.handoffAt;
            episode.partnerFell = w.partnerFell;
            episode.partnerSeconds = w.partnerDrove;
            episode.touchedDown = w.touchedDown;
            episode.supported = w.supported;
            episode.meanFlex = w.flexSamples ? static_cast<float>(w.flexSum / w.flexSamples) : 0.0f;
            episode.obstructed = w.obstructed;
            episode.band = w.startBand;
            episode.meanFaceError = w.faceErrorSamples
                ? static_cast<float>(w.faceErrorSum / w.faceErrorSamples) * 180.0f / Pi : 0.0f;
            {
                std::lock_guard lock(episodeMutex);
                episodes.push_back(episode);
            }
            begin(w);
        }
        observe(w);
    }
};

TrackingGetUpAmpTraining3D::TrackingGetUpAmpTraining3D(const TrackingGetUpAmpSettings3D& settings)
    : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    impl.settings = settings;
    impl.walkIndices.fill(std::numeric_limits<std::size_t>::max());
    impl.bundle = AthleteBundle3D::shared(settings.assetsDirectory);
    impl.profile = &impl.bundle->profile();
    std::vector<std::string> names(std::begin(GetUpClips), std::end(GetUpClips));
    names.emplace_back(StandingClip);
    impl.standingIndex = names.size() - 1;
    names.emplace_back(StandingMirrorClip);
    impl.standingMirrorIndex = names.size() - 1;
    // (O levantar v2: os clipes da arrancada e da corrida so entram com o levantar em movimento ligado - sem ele, o
    // estilo e as quedas de antes.)
    if (settings.movingFraction > 0.0f || settings.sprintStartFraction > 0.0f || settings.evaluationKind == 1)
        names.insert(names.end(), std::begin(MovingClips), std::end(MovingClips));
    if (settings.locomotionFraction > 0.0f || settings.turnFraction > 0.0f || settings.evaluationKind == 4
        || settings.evaluationKind == 6) {
        names.insert(names.end(), std::begin(HeadingClips), std::end(HeadingClips));
        names.insert(names.end(), std::begin(LocomotionStyleClips), std::end(LocomotionStyleClips));
    }
    // (A rl_final: a corrida e o sprint so com as fatias deles - ou a avaliacao dessas faixas -, cada clipe uma vez.)
    if (settings.runFraction > 0.0f || settings.sprintFraction > 0.0f || settings.evaluationBand >= 2) {
        names.insert(names.end(), std::begin(RunClips), std::end(RunClips));
        names.insert(names.end(), std::begin(SprintClips), std::end(SprintClips));
    }
    for (std::size_t i = 0; i < names.size(); ++i)
        if (std::find(names.begin(), names.begin() + static_cast<std::ptrdiff_t>(i), names[i])
            != names.begin() + static_cast<std::ptrdiff_t>(i))
            names.erase(names.begin() + static_cast<std::ptrdiff_t>(i--));
    for (const std::string& name : names) {
        impl.clips.push_back(std::make_unique<AnimationClip3D>(
            loadAnimationClip3D(settings.assetsDirectory + "/animations/clips/" + name + ".matteranim.json")));
        impl.references.push_back(std::make_unique<TrackingClipReference3D>(*impl.profile, *impl.clips.back()));
        if (name == "crouched_to_sprint") impl.sprintStartIndex = impl.references.size() - 1;
        if (std::find(std::begin(MovingClips), std::end(MovingClips), name) != std::end(MovingClips))
            impl.runStartIndices.push_back(impl.references.size() - 1);
        const bool heading = std::find(std::begin(HeadingClips), std::end(HeadingClips), name) != std::end(HeadingClips);
        if (heading) impl.headingIndices.push_back(impl.references.size() - 1);
        constexpr std::pair<const char*, std::size_t> Walks[4] { { "ual_walk", 0 }, { "walk_strafe_left", 2 },
            { "walk_backward", 4 }, { "walk_strafe_right", 6 } };
        for (const auto& [walkName, walkHeading] : Walks)
            if (name == walkName) impl.walkIndices[walkHeading] = impl.references.size() - 1;
        Impl::Travel travel;
        const AnimationClip3D& clip = *impl.clips.back();
        travel.direction = clip.travelDirectionRadians;
        if (name == "idle_to_sprint") travel.rampDisplacement = std::hypot(clip.sourceRootDisplacementMeters.x,
            clip.sourceRootDisplacementMeters.y);
        else if (name == "mixamo_running" || name == "run_forward_medium" || name == "sprint_forward" || heading
            || std::find(std::begin(LocomotionStyleClips), std::end(LocomotionStyleClips), name)
                != std::end(LocomotionStyleClips)
            || std::find(std::begin(RunClips), std::end(RunClips), name) != std::end(RunClips)
            || std::find(std::begin(SprintClips), std::end(SprintClips), name) != std::end(SprintClips))
            travel.speed = clip.loops ? clip.nominalSpeedMetersPerSecond : 0.0f;
        impl.travel.push_back(travel);
        for (const GaitEntry& entry : GaitTable)
            if (name == entry.clip && travel.speed > 0.0f)
                impl.gaits[entry.band].push_back({ impl.references.size() - 1, entry.heading });
    }
    // (Os passos de cada faixa na ordem da GaitTable: o primeiro e o da avaliacao, para a frente.)
    for (std::vector<Impl::Gait>& band : impl.gaits) {
        std::stable_sort(band.begin(), band.end(), [&](const Impl::Gait& a, const Impl::Gait& b) {
            const auto rank = [&](const Impl::Gait& g) {
                const std::string& clip = names[g.clip];
                for (std::size_t k = 0; k < std::size(GaitTable); ++k)
                    if (clip == GaitTable[k].clip) return k;
                return std::size(GaitTable);
            };
            return rank(a) < rank(b);
        });
    }
    impl.contract = std::make_unique<TrackingGetUpContract3D>(*impl.profile);
    impl.features = std::make_unique<TrackingAmpFeatures3D>(*impl.profile);
    impl.posture = std::make_unique<TrackingGetUpPostureProbe3D>(*impl.profile);
    {
        TrackingClipFrame3D idle;
        impl.references[impl.standingIndex]->frame(0.0f, {}, 0.0f, idle);
        impl.posture->setIdlePose(idle.links);
        impl.references[impl.standingMirrorIndex]->frame(0.0f, {}, 0.0f, idle);
        impl.posture->addIdlePose(idle.links);
    }
    for (std::size_t i = 0; i < impl.profile->links.size(); ++i) {
        if (impl.profile->links[i].parentIndex < 0) impl.rootLink = static_cast<std::uint32_t>(i);
        const std::string& id = impl.profile->links[i].id;
        if (id == "LeftFoot") impl.footLinks[0] = static_cast<std::uint32_t>(i);
        if (id == "RightFoot") impl.footLinks[1] = static_cast<std::uint32_t>(i);
        if (id == "UpperChest") impl.upperChestLink = static_cast<std::uint32_t>(i);
        if (id == "LeftHand") impl.handLinks[0] = static_cast<std::uint32_t>(i);
        if (id == "RightHand") impl.handLinks[1] = static_cast<std::uint32_t>(i);
        if (id == "LeftShin") impl.shinLinks[0] = static_cast<std::uint32_t>(i);
        if (id == "RightShin") impl.shinLinks[1] = static_cast<std::uint32_t>(i);
        impl.allowedContact.push_back(id != "Pelvis" && id != "Abdomen" && id != "Chest" && id != "UpperChest"
            && id != "Neck" && id != "Head");
        impl.supportContact.push_back(impl.allowedContact.back() && id.find("Foot") == std::string::npos
            && id.find("Toe") == std::string::npos);
    }
    // (Os nomes do corpo: os do levantar do DeepMimic sem a fase, a ultima; depois os do comando.)
    impl.observationNames.assign(impl.contract->observationNames().begin(), impl.contract->observationNames().end() - 1);
    for (const char* name : { "comando.alvo_cos", "comando.alvo_sen", "comando.distancia", "comando.velocidade",
             "comando.rumo_cos", "comando.rumo_sen" })
        impl.observationNames.emplace_back(name);
    for (const std::string& name : TrackingGetUpContract3D::groundObservationNames()) impl.observationNames.push_back(name);
    const std::uint32_t hardware = std::max(1u, std::thread::hardware_concurrency());
    impl.workers = settings.threads > 0 ? settings.threads : std::max(1u, hardware > 2 ? hardware - 2 : 1u);
    impl.scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings { std::max(1u, impl.workers - 1) });
    impl.engine = std::make_unique<PhysicsEngine3D>(impl.scheduler);
    impl.worlds.resize(std::max(1u, settings.environments));
    for (std::size_t i = 0; i < impl.worlds.size(); ++i) impl.worlds[i].index = static_cast<std::uint32_t>(i);
    for (Impl::World& w : impl.worlds) {
        w.scene = impl.engine->createScene(trackingSceneSettings3D(), impl.materials);
        createTrackingGround3D(*w.scene, 20.0f);
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 1;
        spawn.active = true;
        spawn.pelvisPosition = { 0.0f, 0.0f, impl.profile->standingRootHeightMeters };
        w.handle = w.scene->createRagdoll(*impl.profile, spawn);
        w.observation.assign(impl.contract->bodyObservationSize() + CommandSize + GroundObservationSize, 0.0f);
        w.finalObservation.assign(w.observation.size(), 0.0f);
        w.ampPair.assign(2 * impl.features->size(), 0.0f);
        w.pose.assign(impl.contract->actionSize(), 0.0f);
    }
    reset(settings.seed);
}

TrackingGetUpAmpTraining3D::~TrackingGetUpAmpTraining3D() {
    if (m_impl) m_impl->worlds.clear();
}

std::size_t TrackingGetUpAmpTraining3D::environmentCount() const { return m_impl->worlds.size(); }
std::size_t TrackingGetUpAmpTraining3D::observationSize() const {
    return m_impl->contract->bodyObservationSize() + CommandSize + GroundObservationSize;
}
std::size_t TrackingGetUpAmpTraining3D::actionSize() const { return m_impl->contract->actionSize(); }
std::size_t TrackingGetUpAmpTraining3D::ampSize() const { return m_impl->features->size(); }
const std::vector<std::string>& TrackingGetUpAmpTraining3D::observationNames() const {
    return m_impl->observationNames;
}
const std::vector<std::string>& TrackingGetUpAmpTraining3D::actionNames() const {
    return m_impl->contract->actionNames();
}
float TrackingGetUpAmpTraining3D::episodeSeconds() const {
    return std::max(m_impl->settings.episodeSeconds, m_impl->settings.locomotionSeconds);
}
std::vector<float> TrackingGetUpAmpTraining3D::actionRanges() const { return m_impl->contract->actionRanges(); }

void TrackingGetUpAmpTraining3D::setFallStarts(std::vector<std::vector<PhysicsBodyState3D>> states) {
    const std::size_t links = m_impl->profile->links.size();
    std::erase_if(states, [links](const std::vector<PhysicsBodyState3D>& s) { return s.size() != links; });
    m_impl->fallStarts = std::move(states);
}

std::size_t TrackingGetUpAmpTraining3D::fallStartCount() const { return m_impl->fallStarts.size(); }

void TrackingGetUpAmpTraining3D::protecting(std::span<std::uint8_t> out) const {
    for (std::size_t i = 0; i < m_impl->worlds.size() && i < out.size(); ++i) out[i] = m_impl->worlds[i].protecting ? 1 : 0;
}

void TrackingGetUpAmpTraining3D::locomoting(std::span<std::uint8_t> out) const {
    for (std::size_t i = 0; i < m_impl->worlds.size() && i < out.size(); ++i) out[i] = m_impl->worlds[i].locomotion ? 1 : 0;
}

void TrackingGetUpAmpTraining3D::setLocomotionPartner(std::shared_ptr<const LocomotionPartner3D> partner) {
    m_impl->partner = std::move(partner);
}

void TrackingGetUpAmpTraining3D::reset(std::uint64_t seed) {
    Impl& impl = *m_impl;
    trackingPullWorlds3D(*impl.scheduler, impl.workers, impl.worlds.size(), [&impl, seed](std::size_t i) {
        Impl::World& w = impl.worlds[i];
        w.rng.seed(static_cast<std::uint32_t>(seed * 2654435761ull + i * 40503ull + 911ull));
        impl.begin(w);
        impl.observe(w);
    });
    impl.episodes.clear();
}

void TrackingGetUpAmpTraining3D::observations(std::span<float> out) const {
    const std::size_t size = observationSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].observation.begin(), m_impl->worlds[i].observation.end(), out.begin() + i * size);
}

void TrackingGetUpAmpTraining3D::finalObservations(std::span<float> out) const {
    const std::size_t size = observationSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].finalObservation.begin(), m_impl->worlds[i].finalObservation.end(),
            out.begin() + i * size);
}

void TrackingGetUpAmpTraining3D::step(std::span<const float> actions, std::span<float> rewards,
    std::span<std::uint8_t> terminated, std::span<std::uint8_t> truncated) {
    Impl& impl = *m_impl;
    const std::size_t n = impl.worlds.size();
    const std::size_t a = actionSize();
    if (actions.size() < n * a || rewards.size() < n || terminated.size() < n || truncated.size() < n)
        throw std::invalid_argument("levantar livre: buffers pequenos no step");
    trackingPullWorlds3D(*impl.scheduler, impl.workers, n, [&](std::size_t i) {
        impl.decide(impl.worlds[i], i, actions.subspan(i * a, a), rewards[i], terminated[i], truncated[i]);
    });
}

void TrackingGetUpAmpTraining3D::ampPairs(std::span<float> out) const {
    const std::size_t size = 2 * ampSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].ampPair.begin(), m_impl->worlds[i].ampPair.end(), out.begin() + i * size);
}

std::vector<float> TrackingGetUpAmpTraining3D::referencePairs() const {
    const Impl& impl = *m_impl;
    const std::size_t a = ampSize();
    const float step = TicksPerDecision * TickSeconds;
    std::vector<float> out;
    std::vector<float> now(a), next(a);
    TrackingClipFrame3D first, second;
    for (std::size_t r = 0; r < impl.references.size(); ++r) {
        const auto& reference = impl.references[r];
        const Impl::Travel& travel = impl.travel[r];
        const float duration = reference->durationSeconds();
        // (Os clipes no lugar: a velocidade do deslocamento que a fonte tinha, para a frente do clipe, somada a todos
        // os links - as caracteristicas usam a da raiz; as posicoes vao em relacao a ela.)
        const auto moving = [&](float t, TrackingClipFrame3D& frame) {
            float speed = travel.speed;
            if (travel.rampDisplacement > 0.0f && duration > 0.0f)
                speed = 2.0f * travel.rampDisplacement * std::clamp(t, 0.0f, duration) / (duration * duration);
            if (speed <= 0.0f) return;
            const Vec3 v { std::cos(travel.direction) * speed, std::sin(travel.direction) * speed, 0.0f };
            for (PhysicsBodyState3D& s : frame.links) s.linearVelocity = s.linearVelocity + v;
        };
        // (A arrancada agachada pesa mais: a referencia do levantar em movimento, "um levantar preparado ja indo".)
        const int copies = r == impl.sprintStartIndex ? SprintStartPairWeight : 1;
        for (float t = 0.0f; t + step <= duration + 1e-5f; t += 1.0f / 120.0f) {
            reference->frame(t, {}, 0.0f, first);
            reference->frame(t + step, {}, 0.0f, second);
            moving(t, first);
            moving(t + step, second);
            impl.features->compute(first.links, now);
            impl.features->compute(second.links, next);
            for (int c = 0; c < copies; ++c) {
                out.insert(out.end(), now.begin(), now.end());
                out.insert(out.end(), next.begin(), next.end());
            }
        }
    }
    return out;
}

std::vector<TrackingGetUpAmpEpisode3D> TrackingGetUpAmpTraining3D::takeEpisodes() {
    std::lock_guard lock(m_impl->episodeMutex);
    std::vector<TrackingGetUpAmpEpisode3D> out;
    out.swap(m_impl->episodes);
    return out;
}

const RagdollState3D& TrackingGetUpAmpTraining3D::body(std::size_t world) const {
    return *m_impl->worlds.at(world).body;
}
std::span<const RagdollDriveTarget3D> TrackingGetUpAmpTraining3D::driveTargets(std::size_t world) const {
    return m_impl->worlds.at(world).drives;
}

const RagdollProfile3D& TrackingGetUpAmpTraining3D::profile() const { return *m_impl->profile; }

TrackingGetUpAmpWorldView3D TrackingGetUpAmpTraining3D::worldView(std::size_t world) const {
    const Impl::World& w = m_impl->worlds.at(world);
    TrackingGetUpAmpWorldView3D view;
    view.startKind = w.startKind;
    view.hasTarget = w.startKind == 3 || w.startKind == 5;
    view.target = w.target;
    view.commandSpeed = w.commandSpeed;
    if (w.commandSpeed > 0.0f) {
        const float move = m_impl->moveDirection(w);
        view.moveVelocity = { std::cos(move) * w.commandSpeed, std::sin(move) * w.commandSpeed, 0.0f };
    }
    view.hasFace = w.locomotion;
    view.faceYaw = w.faceYaw;
    view.pushing = w.pushTicks > 0;
    view.pushForce = w.pushForce;
    return view;
}

std::string TrackingGetUpAmpTraining3D::frameLines(std::size_t world) const {
    const Impl& impl = *m_impl;
    const Impl::World& w = impl.worlds.at(world);
    const RagdollState3D& body = *w.body;
    std::ostringstream out;
    for (std::size_t i = 0; i < impl.profile->links.size() && i < body.links.size(); ++i) {
        const RagdollLinkDefinition3D& link = impl.profile->links[i];
        const PhysicsBodyState3D& state = body.links[i];
        const Vec3 center = state.position + state.orientation.rotate(link.collider.localPosition);
        const Quaternion orientation = (state.orientation * link.collider.localOrientation).normalized();
        out << (link.collider.shape == RagdollColliderShape3D::Box ? "B " : "C ") << link.id << ' ' << center.x << ' '
            << center.y << ' ' << center.z << ' ' << orientation.x << ' ' << orientation.y << ' ' << orientation.z
            << ' ' << orientation.w << '\n';
    }
    for (const RagdollContactPoint3D& contact : body.contacts)
        out << "P " << contact.position.x << ' ' << contact.position.y << ' ' << contact.position.z << ' '
            << contact.linkIndex << ' ' << contact.normal.x << ' ' << contact.normal.y << ' ' << contact.normal.z << ' '
            << contact.normalImpulseNewtonSeconds << '\n';
    // (O alvo do levantar em movimento, como o "A" das folhas da locomocao.)
    if (w.startKind == 3 || w.startKind == 5)
        out << "A " << w.target.x << ' ' << w.target.y << " 0 " << ArriveRadius << '\n';
    return out.str();
}

} // namespace MatterEngine
