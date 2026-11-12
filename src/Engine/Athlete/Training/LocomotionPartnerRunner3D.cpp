#include "Engine/Athlete/Training/LocomotionPartnerRunner3D.hpp"

#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/AthleteTelemetry3D.hpp"
#include "Engine/Athlete/Body/GetUpPolicy3D.hpp"
#include "Engine/Athlete/Physical/PhysicalCorrections3D.hpp"
#include "Engine/Athlete/Policy/LocomotionPolicy3D.hpp"
#include "Engine/Athlete/Training/LocomotionTask3D.hpp"
#include "Engine/Athlete/Training/LocomotionTraining3D.hpp"
#include "Engine/Physique/PlayerPhysique3D.hpp"

#include <algorithm>
#include <mutex>
#include <vector>

namespace MatterEngine {
namespace {

constexpr float TickSeconds = 1.0f / 120.0f;

} // namespace

struct LocomotionPartnerRunner3D::Impl {
    std::shared_ptr<const AthleteBundle3D> bundle;
    AthleteRuntimeSettings3D settings;
    // A queda iminente do treino dela (o fim do episodio): a pelve abaixo disto (m, na escala do corpo) e, com
    // forbiddenContact, algo que nao os pes no chao.
    float fallPelvisHeight = 0.45f;
    bool forbiddenContact = false;
    std::vector<bool> footLink;
    std::filesystem::path policyFile;
    std::uint64_t iteration = 0;
    // (A rede guarda o estado da decisao: uma por chamada ao mesmo tempo, devolvida no fim.)
    std::mutex mutex;
    std::vector<std::shared_ptr<LocomotionPolicy3D>> idle;

    std::shared_ptr<LocomotionPolicy3D> acquire() {
        {
            std::lock_guard lock(mutex);
            if (!idle.empty()) {
                auto policy = std::move(idle.back());
                idle.pop_back();
                return policy;
            }
        }
        return std::make_shared<LocomotionPolicy3D>(bundle->profile(), policyFile);
    }
    void release(std::shared_ptr<LocomotionPolicy3D> policy) {
        std::lock_guard lock(mutex);
        idle.push_back(std::move(policy));
    }
    // O pedido pelo analogico do jogo (o mesmo do treino dela): o deslocamento velocity no chao, com a camera (a vista:
    // o peito e o olhar) em cameraYaw, o sprint acima do trote da tabela.
    AthleteCommand3D stickCommand(const AthleteRuntimeSettings3D& settings, Vec3 velocity, float cameraYaw) const {
        AthleteStickInput3D stick;
        const float wanted = std::hypot(velocity.x, velocity.y);
        const Vec3 direction { velocity.x / wanted, velocity.y / wanted, 0.0f };
        stick.cameraYawRadians = cameraYaw;
        stick.lookPitchRadians = -0.2f;
        stick.moveWorld = direction;
        const auto tableSpeed = [&]() {
            const Vec3 v = athleteStickCommand3D(stick, settings, bundle->gaitSpeeds(), 1.0e9f).intent.requestedVelocityWorld;
            return std::hypot(v.x, v.y);
        };
        float table = tableSpeed();
        if (wanted > table + 1.0e-3f) {
            stick.sprint = true;
            table = tableSpeed();
        }
        stick.moveWorld = direction * (table > 1.0e-3f ? std::min(1.0f, wanted / table) : 0.0f);
        return athleteStickCommand3D(stick, settings, bundle->gaitSpeeds(), 1.0e9f);
    }
    // A sessao tick a tick: o atleta fisico do zero no corpo. (A rede sai do pool a cada tick e volta: o estado da
    // decisao mora no atleta - a rampa, o historico -, e uma rede por mundo seriam ~6,5 MB x 400.)
    struct DriveSession final : LocomotionPartner3D::Session {
        AthleteRuntime3D runtime;
        AthleteState3D athlete;
        AthleteRuntimeSettings3D settings;
        AthleteCommand3D command;
        RagdollState3D body;
        bool alive = true;
        // (A queda iminente dela, como no fim do episodio do treino dela e no drive(): os ticks e o tronco da ultima decisao.)
        int ticks = 0;
        float previousTilt = 0.0f;
        bool tiltValid = false;
    };
};

LocomotionPartnerRunner3D::LocomotionPartnerRunner3D(const std::string& assetsDirectory,
    const std::filesystem::path& taskFile, const std::filesystem::path& policyFile)
    : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    impl.bundle = AthleteBundle3D::shared(assetsDirectory);
    const LocomotionTask3D task = loadLocomotionTask3D(taskFile);
    impl.settings = locomotionRuntimeSettings3D(task);
    impl.settings.historicalAssistRuler = task.historicalAssistRuler;
    impl.fallPelvisHeight = task.fallPelvisHeight;
    impl.forbiddenContact = task.reward.forbiddenContact;
    for (const RagdollLinkDefinition3D& link : impl.bundle->profile().links)
        impl.footLink.push_back(link.id.find("Foot") != std::string::npos || link.id.find("Toe") != std::string::npos);
    impl.policyFile = policyFile;
    auto first = std::make_shared<LocomotionPolicy3D>(impl.bundle->profile(), policyFile);
    impl.iteration = first->iteration();
    impl.idle.push_back(std::move(first));
}

LocomotionPartnerRunner3D::~LocomotionPartnerRunner3D() = default;

std::uint64_t LocomotionPartnerRunner3D::iteration() const { return m_impl->iteration; }

LocomotionPartner3D::Result LocomotionPartnerRunner3D::drive(PhysicsScene3D& scene, RagdollHandle3D handle,
    std::span<const RagdollDriveTarget3D> handoff, float maximumSeconds, const Push* push, Vec3 velocity) const {
    Impl& impl = *m_impl;
    const RagdollProfile3D& profile = impl.bundle->profile();
    Result result;
    RagdollState3D body = scene.ragdollState(handle);
    if (body.links.size() != profile.links.size()) return result;
    std::shared_ptr<LocomotionPolicy3D> policy = impl.acquire();
    AthleteRuntime3D runtime;
    runtime.setLocomotionPolicy(policy);
    AthleteState3D athlete;
    athleteStart3D(scene, handle, profile, body, athlete, AthleteStartMode3D::Physical);
    athlete.getUpHandoff = { handoff.empty() ? 0.0f : GetUpPolicy3D::BlendSeconds,
        std::vector<RagdollDriveTarget3D>(handoff.begin(), handoff.end()) };
    AthleteRuntimeSettings3D settings = impl.settings;
    // (Parada no lugar, olhando para onde o corpo olha; com o pedido de deslocamento, pelo analogico do jogo - o mesmo
    // do treino dela -, na direcao dele, com o sprint acima do trote da tabela.)
    AthleteCommand3D command;
    command.facingYawRadians = locomotionFrameYaw3D(profile, body.links.front().orientation);
    command.intent.requestedFacingYawRadians = command.facingYawRadians;
    if (std::hypot(velocity.x, velocity.y) > 0.05f)
        command = impl.stickCommand(settings, velocity, std::atan2(velocity.y, velocity.x));
    settings.locomotionPolicy = true;
    settings.dynamicTurnMove = policy->usesTurnMove();
    if (settings.dynamicTurnMove) settings.dynamicTurnMoveSettings.amount = policy->turnMoveAmount();
    const float pelvisFloor = impl.fallPelvisHeight * bodyScale3D(impl.bundle->body());
    const int ticks = std::max(0, static_cast<int>(maximumSeconds / TickSeconds));
    float previousTilt = 0.0f;
    bool tiltValid = false;
    for (int tick = 0; tick < ticks; ++tick) {
        AthleteSlot3D slot { 1, handle, &body, true, false, &athlete };
        slot.command = &command;
        AthleteControlInput3D control;
        control.controlledEntityId = 0;
        AthleteRuntimeOutput3D output;
        runtime.update(scene, impl.bundle->assets(), std::span<AthleteSlot3D>(&slot, 1), {}, settings, control,
            TickSeconds, output);
        const float time = static_cast<float>(tick) * TickSeconds;
        if (push != nullptr && time >= push->startSeconds && time < push->startSeconds + push->seconds) {
            scene.applyRagdollLinkForce(handle, 0, push->force * 0.20f, {});
            scene.applyRagdollLinkForce(handle, 2, push->force * 0.32f, {});
            scene.applyRagdollLinkForce(handle, 3, push->force * 0.48f, {});
        }
        scene.simulate(TickSeconds);
        body = scene.ragdollState(handle);
        result.seconds = static_cast<float>(tick + 1) * TickSeconds;
        bool fell = body.links.front().position.z - athlete.groundHeight < pelvisFloor;
        if (!fell && impl.forbiddenContact)
            for (const RagdollInteraction3D& contact : body.interactions)
                if (contact.linkIndex < impl.footLink.size() && !impl.footLink[contact.linkIndex]
                    && contact.otherMotion == RagdollContactMotion3D::Static && contact.normalWorld.z > 0.5f)
                    fell = true;
        // (No fim de cada decisao, como no treino dela: o classico caido e a queda iminente pelo tronco.)
        if ((tick + 1) % GetUpPolicy3D::TicksPerDecision == 0) {
            if (athleteTelemetry3D(athlete).fallen) fell = true;
            const float tilt = GetUpPolicy3D::trunkTilt(profile, body.links);
            if (tiltValid && tilt > GetUpPolicy3D::ImminentTiltDegrees
                && (tilt - previousTilt) / (GetUpPolicy3D::TicksPerDecision * TickSeconds) > GetUpPolicy3D::ImminentTiltRate)
                fell = true;
            previousTilt = tilt;
            tiltValid = true;
        }
        if (fell) {
            result.fell = true;
            if (push != nullptr) result.pushLeft = std::max(0.0f, push->startSeconds + push->seconds - result.seconds);
            break;
        }
    }
    impl.release(std::move(policy));
    return result;
}

std::unique_ptr<LocomotionPartner3D::Session> LocomotionPartnerRunner3D::begin(PhysicsScene3D& scene,
    RagdollHandle3D handle, Vec3 velocity, float viewYaw) const {
    Impl& impl = *m_impl;
    const RagdollProfile3D& profile = impl.bundle->profile();
    auto session = std::make_unique<Impl::DriveSession>();
    session->body = scene.ragdollState(handle);
    if (session->body.links.size() != profile.links.size()) {
        session->alive = false;
        return session;
    }
    std::shared_ptr<LocomotionPolicy3D> policy = impl.acquire();
    athleteStart3D(scene, handle, profile, session->body, session->athlete, AthleteStartMode3D::Physical);
    session->settings = impl.settings;
    session->settings.locomotionPolicy = true;
    session->settings.dynamicTurnMove = policy->usesTurnMove();
    if (session->settings.dynamicTurnMove) session->settings.dynamicTurnMoveSettings.amount = policy->turnMoveAmount();
    impl.release(std::move(policy));
    command(*session, velocity, viewYaw);
    return session;
}

void LocomotionPartnerRunner3D::command(Session& base, Vec3 velocity, float viewYaw) const {
    auto& session = static_cast<Impl::DriveSession&>(base);
    // (Parada, olhando para a vista - o giro parado e a vista girando; andando, o analogico com a camera na vista - o peito
    // nela, o corpo indo no rumo.)
    if (std::hypot(velocity.x, velocity.y) > 0.05f) {
        session.command = m_impl->stickCommand(session.settings, velocity, viewYaw);
    } else {
        session.command = {};
        session.command.facingYawRadians = viewYaw;
        session.command.intent.requestedFacingYawRadians = viewYaw;
        // (O olhar do treino dela: -0,2, como no analogico; com 0 a entrada dela sai do que ela conhece - a revisao de 03/11.)
        session.command.lookPitchRadians = -0.2f;
    }
}

LocomotionPartner3D::Tick LocomotionPartnerRunner3D::tick(Session& base, PhysicsScene3D& scene,
    RagdollHandle3D handle) const {
    Impl& impl = *m_impl;
    auto& session = static_cast<Impl::DriveSession&>(base);
    Tick result;
    if (!session.alive) return result;
    session.body = scene.ragdollState(handle);
    session.athlete.externalAssist = {};
    std::shared_ptr<LocomotionPolicy3D> policy = impl.acquire();
    session.runtime.setLocomotionPolicy(policy);
    AthleteSlot3D slot { 1, handle, &session.body, true, false, &session.athlete };
    slot.command = &session.command;
    AthleteControlInput3D control;
    control.controlledEntityId = 0;
    AthleteRuntimeOutput3D output;
    session.runtime.update(scene, impl.bundle->assets(), std::span<AthleteSlot3D>(&slot, 1), {}, session.settings,
        control, TickSeconds, output);
    session.runtime.setLocomotionPolicy(nullptr);
    impl.release(std::move(policy));
    const AthleteExternalAssist3D& assist = session.athlete.externalAssist;
    result.assistNewtons = assist.sliderNewtons() + assist.activeNewtons + assist.netTorque;
    // (Caida ou guiada - o classico no chao -, a sessao acaba: o rotulo so vale no fisico de pe. Tambem a queda iminente
    // do treino dela - a pelve baixa, um contato proibido, o tronco tombando -, como no drive(): o trecho da queda dela nao
    // vira rotulo - a revisao de 03/11.)
    bool fell = athleteTelemetry3D(session.athlete).fallen || session.body.guided;
    const RagdollProfile3D& profile = impl.bundle->profile();
    if (!fell && session.body.links.front().position.z - session.athlete.groundHeight
            < impl.fallPelvisHeight * bodyScale3D(impl.bundle->body()))
        fell = true;
    if (!fell && impl.forbiddenContact)
        for (const RagdollInteraction3D& contact : session.body.interactions)
            if (contact.linkIndex < impl.footLink.size() && !impl.footLink[contact.linkIndex]
                && contact.otherMotion == RagdollContactMotion3D::Static && contact.normalWorld.z > 0.5f)
                fell = true;
    if (++session.ticks % GetUpPolicy3D::TicksPerDecision == 0) {
        const float tilt = GetUpPolicy3D::trunkTilt(profile, session.body.links);
        if (session.tiltValid && tilt > GetUpPolicy3D::ImminentTiltDegrees
            && (tilt - session.previousTilt) / (GetUpPolicy3D::TicksPerDecision * TickSeconds) > GetUpPolicy3D::ImminentTiltRate)
            fell = true;
        session.previousTilt = tilt;
        session.tiltValid = true;
    }
    session.alive = !fell;
    result.valid = session.alive;
    return result;
}

} // namespace MatterEngine
