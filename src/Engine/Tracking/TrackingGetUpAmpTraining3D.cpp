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
// Os pares da arrancada agachada pesam isto no discriminador (a referencia do levantar em movimento).
constexpr int SprintStartPairWeight = 6;
// O levantar em movimento: chegou ao alvo neste raio (m), e dai o pedido vira ficar parado.
constexpr float ArriveRadius = 1.0f;
// Recomposto depois do empurrao do evitar: de pe, parecido com o parado e quase parado.
constexpr float RecomposedPose = 0.6f;
constexpr float RecomposedSpeed = 0.3f;

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
    // A arrancada agachada (crouched_to_sprint), quando carregada: o comeco do tipo 5 e o peso maior no estilo.
    std::size_t sprintStartIndex = std::numeric_limits<std::size_t>::max();
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
    // As partes que podem tocar o chao sem contar como caido: as pernas e os bracos (nao o tronco, o pescoco e a cabeca).
    std::vector<bool> allowedContact;
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
        // O empurrao (o de pe do jogo de pes ou o forte do evitar): a forca, os passos de fisica que faltam e o proximo.
        Vec3 pushForce;
        int pushTicks = 0;
        float nextPushAt = -1.0f;
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
    };
    std::vector<World> worlds;
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

    // A arrancada agachada (o crouched_to_sprint): um instante da primeira metade, num rumo sorteado (o alvo vem perto
    // dele), com a velocidade do clipe.
    void beginSprintStart(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const auto& reference = *references[sprintStartIndex];
        w.sprintYaw = (2.0f * u(w.rng) - 1.0f) * Pi;
        reference.frame(u(w.rng) * 0.5f * reference.durationSeconds(), {}, w.sprintYaw, w.frame);
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

    bool fallen(const TrackingGetUpPosture3D& posture, const RagdollState3D& body) const {
        (void)posture;
        for (const RagdollContactPoint3D& contact : body.contacts)
            if (contact.linkIndex < allowedContact.size() && !allowedContact[contact.linkIndex]) return true;
        return false;
    }

    void begin(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        // (Um sorteio so, como antes: com as fatias novas em 0, os mesmos comecos de antes.)
        // (Tambem na avaliacao, que nao usa o sorteio: as mesmas 64 quedas fixas de antes.)
        const float draw = u(w.rng);
        std::uint32_t kind = 0;
        if (settings.evaluation) {
            kind = settings.evaluationKind == 1 ? 3u : settings.evaluationKind == 2 ? 4u : 0u;
        } else {
            float edge = settings.standingStartFraction;
            if (draw < edge) kind = 2;
            else if (draw < (edge += settings.clipStartFraction)) kind = 1;
            else if (draw < (edge += settings.avoidFraction)) kind = 4;
            else if (draw < (edge += settings.movingFraction)) kind = 3;
            else if (draw < (edge += settings.sprintStartFraction)) kind = 5;
        }
        if (kind == 5 && sprintStartIndex >= references.size()) kind = 3;
        if (kind == 0 || kind == 3) beginFall(w);
        else if (kind == 5) beginSprintStart(w);
        else beginFromClip(w, kind != 1);
        const TrackingGetUpPosture3D start = posture->measure(w.body->links);
        w.startKind = kind;
        w.startSide = start.chestFacingUp < -0.5f ? 0u : start.chestFacingUp > 0.5f ? 1u : 2u;
        w.startHead = start.headHeight;
        w.time = 0.0f;
        w.stoodAt = -1.0f;
        w.rewardSum = 0.0;
        w.decisions = 0;
        w.pushTicks = 0;
        w.nextPushAt = -1.0f;
        w.commandSpeed = 0.0f;
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
    }

    // O empurrao de pe (31/10 09h, ele: depois de levantar falta o "jogo de pes de auto-equilibrio"): so no treino, nos
    // comecos sem alvo e sem o empurrao forte, depois de ja ter ficado de pe e enquanto esta de pe, um empurrao curto a
    // cada 1,5-3 s (o primeiro 1-2,5 s depois de ficar de pe), na faixa da configuracao por 0,1-0,2 s, num rumo
    // sorteado (150-450 N ate 12h35, 250-650 ate 15h05, 400-900 depois; cada degrau quando ele ja ficava de pe em ~100%).
    // Se cair, levanta de novo.
    void maybePush(World& w, const TrackingGetUpPosture3D& posture) {
        if (settings.evaluation || w.startKind >= 3 || w.stoodAt < 0.0f || !posture.standing || w.pushTicks > 0) return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if (w.nextPushAt < 0.0f) {
            w.nextPushAt = w.time + 1.0f + 1.5f * u(w.rng);
            return;
        }
        if (w.time < w.nextPushAt) return;
        const float angle = 2.0f * Pi * u(w.rng);
        const float newtons = settings.pushMinimumNewtons + (settings.pushMaximumNewtons - settings.pushMinimumNewtons) * u(w.rng);
        w.pushForce = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
        w.pushTicks = std::max(1, static_cast<int>(std::lround((0.1f + 0.1f * u(w.rng)) / TickSeconds)));
        w.nextPushAt = w.time + 1.5f + 1.5f * u(w.rng);
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
        w.avoidAt = -1.0f;
    }

    // O rumo do alvo em relacao ao corpo (rad) e a distancia (m), no chao.
    std::pair<float, float> targetBearing(const World& w) const {
        const PhysicsBodyState3D& root = w.body->links[rootLink];
        const Vec3 to { w.target.x - root.position.x, w.target.y - root.position.y, 0.0f };
        const float yaw = trackingBodyYaw3D(profile->links[rootLink], root);
        return { wrapAngle(planarAngle(to) - yaw), std::hypot(to.x, to.y) };
    }

    void observe(World& w) const {
        const std::size_t b = contract->bodyObservationSize();
        contract->observeBody(w.body->links, std::span<float>(w.observation).first(b));
        float* c = w.observation.data() + b;
        if (w.commandSpeed <= 0.0f) {
            c[0] = 1.0f, c[1] = 0.0f, c[2] = 0.0f, c[3] = 0.0f;
            return;
        }
        const auto [bearing, distance] = targetBearing(w);
        c[0] = std::cos(bearing);
        c[1] = std::sin(bearing);
        c[2] = std::min(distance, 10.0f) / 10.0f;
        c[3] = w.commandSpeed / 5.0f;
    }

    // O potencial do rumo no levantar em movimento: virar para o alvo enquanto sobe (a altura vezes o alinhamento).
    float facePotential(const World& w, const TrackingGetUpPosture3D& posture) const {
        if (w.commandSpeed <= 0.0f) return 0.0f;
        const float rise = std::clamp((posture.headHeight - 0.15f) / 1.35f, 0.0f, 1.0f);
        return 5.0f * rise * std::max(0.0f, std::cos(targetBearing(w).first));
    }

    // A nota de estado com o pedido de ir ao alvo, so de pe: a velocidade rumo ao alvo 0,5, virado para ele 0,3 e sem
    // andar de lado 0,2. (31/10 18h: a velocidade era exp(-(v - pedida)^2), que nao dava gradiente parado - pedindo
    // 2,5-5 m/s, parado ou a 1 m/s dava ~0 - e em 250 atualizacoes ele ficou parado virado para o alvo. Agora uma rampa:
    // 0 parado, 1 na pedida, caindo acima dela e negativa se afasta.)
    float moveReward(World& w, const TrackingGetUpPosture3D& posture) {
        if (!posture.standing) return 0.0f;
        const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
        const auto [bearing, distance] = targetBearing(w);
        const PhysicsBodyState3D& root = w.body->links[rootLink];
        const float toward = planarAngle({ w.target.x - root.position.x, w.target.y - root.position.y, 0.0f });
        const float along = v.x * std::cos(toward) + v.y * std::sin(toward);
        const float across = -v.x * std::sin(toward) + v.y * std::cos(toward);
        w.towardSum += along;
        ++w.towardSamples;
        (void)distance;
        const float ratio = along / w.commandSpeed;
        const float speed = ratio <= 1.0f ? std::max(-1.0f, ratio) : std::max(0.0f, 2.0f - ratio);
        return 0.5f * speed + 0.3f * std::max(0.0f, std::cos(bearing)) + 0.2f * std::exp(-across * across);
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
        if (w.commandSpeed > 0.0f && targetBearing(w).second < ArriveRadius) {
            w.commandSpeed = 0.0f;
            w.reachedAt = w.time;
        }
        const float state = w.commandSpeed > 0.0f ? moveReward(w, now) : posture->standingReward(now, w.body->links);
        // (O gamma do treinador, 0,99, no potencial.)
        reward = state + 0.99f * (posture->risePotential(now) + posture->turnPotential(now) + facePotential(w, now)) - before;
        const bool down = fallen(now, *w.body);
        if (down && !w.down && w.stoodAt >= 0.0f) {
            reward -= FallPenalty;
            w.fellAfterStanding = true;
        }
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
        truncated = w.time >= settings.episodeSeconds - 1e-4f ? 1 : 0;
        if (truncated) {
            observe(w);
            w.finalObservation = w.observation;
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
    for (const std::string& name : names) {
        impl.clips.push_back(std::make_unique<AnimationClip3D>(
            loadAnimationClip3D(settings.assetsDirectory + "/animations/clips/" + name + ".matteranim.json")));
        impl.references.push_back(std::make_unique<TrackingClipReference3D>(*impl.profile, *impl.clips.back()));
        if (name == "crouched_to_sprint") impl.sprintStartIndex = impl.references.size() - 1;
        Impl::Travel travel;
        const AnimationClip3D& clip = *impl.clips.back();
        travel.direction = clip.travelDirectionRadians;
        if (name == "idle_to_sprint") travel.rampDisplacement = std::hypot(clip.sourceRootDisplacementMeters.x,
            clip.sourceRootDisplacementMeters.y);
        else if (name == "mixamo_running" || name == "run_forward_medium" || name == "sprint_forward")
            travel.speed = clip.nominalSpeedMetersPerSecond;
        impl.travel.push_back(travel);
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
        impl.allowedContact.push_back(id != "Pelvis" && id != "Abdomen" && id != "Chest" && id != "UpperChest"
            && id != "Neck" && id != "Head");
    }
    // (Os nomes do corpo: os do levantar do DeepMimic sem a fase, a ultima; depois os do comando.)
    impl.observationNames.assign(impl.contract->observationNames().begin(), impl.contract->observationNames().end() - 1);
    for (const char* name : { "comando.alvo_cos", "comando.alvo_sen", "comando.distancia", "comando.velocidade" })
        impl.observationNames.emplace_back(name);
    const std::uint32_t hardware = std::max(1u, std::thread::hardware_concurrency());
    impl.workers = settings.threads > 0 ? settings.threads : std::max(1u, hardware > 2 ? hardware - 2 : 1u);
    impl.scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings { std::max(1u, impl.workers - 1) });
    impl.engine = std::make_unique<PhysicsEngine3D>(impl.scheduler);
    impl.worlds.resize(std::max(1u, settings.environments));
    for (Impl::World& w : impl.worlds) {
        w.scene = impl.engine->createScene(trackingSceneSettings3D(), impl.materials);
        createTrackingGround3D(*w.scene, 20.0f);
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 1;
        spawn.active = true;
        spawn.pelvisPosition = { 0.0f, 0.0f, impl.profile->standingRootHeightMeters };
        w.handle = w.scene->createRagdoll(*impl.profile, spawn);
        w.observation.assign(impl.contract->bodyObservationSize() + CommandSize, 0.0f);
        w.finalObservation.assign(impl.contract->bodyObservationSize() + CommandSize, 0.0f);
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
    return m_impl->contract->bodyObservationSize() + CommandSize;
}
std::size_t TrackingGetUpAmpTraining3D::actionSize() const { return m_impl->contract->actionSize(); }
std::size_t TrackingGetUpAmpTraining3D::ampSize() const { return m_impl->features->size(); }
const std::vector<std::string>& TrackingGetUpAmpTraining3D::observationNames() const {
    return m_impl->observationNames;
}
const std::vector<std::string>& TrackingGetUpAmpTraining3D::actionNames() const {
    return m_impl->contract->actionNames();
}
float TrackingGetUpAmpTraining3D::episodeSeconds() const { return m_impl->settings.episodeSeconds; }
std::vector<float> TrackingGetUpAmpTraining3D::actionRanges() const { return m_impl->contract->actionRanges(); }

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

const RagdollProfile3D& TrackingGetUpAmpTraining3D::profile() const { return *m_impl->profile; }

TrackingGetUpAmpWorldView3D TrackingGetUpAmpTraining3D::worldView(std::size_t world) const {
    const Impl::World& w = m_impl->worlds.at(world);
    TrackingGetUpAmpWorldView3D view;
    view.startKind = w.startKind;
    view.hasTarget = w.startKind == 3 || w.startKind == 5;
    view.target = w.target;
    view.commandSpeed = w.commandSpeed;
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
