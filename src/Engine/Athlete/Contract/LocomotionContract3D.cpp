#include "Engine/Athlete/Contract/LocomotionContract3D.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <string>

namespace MatterEngine {

namespace {

constexpr float Gravity = 9.81f;
// O tick do jogo (o contato dos pes vira fracao do peso por ele).
constexpr float TickSeconds = 1.0f / 120.0f;
constexpr float TwoPi = 6.28318530718f;
const char* const AxisNames[3] = { "twist", "swing1", "swing2" };
const char* const KeyPartNames[6] = { "pe_esq", "pe_dir", "mao_esq", "mao_dir", "peito", "cabeca" };

int findLink(const RagdollProfile3D& profile, const char* id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return static_cast<int>(i);
    return -1;
}

// O rumo no plano (x a frente, y a esquerda) e o giro ate ele.
struct Heading {
    float c = 1.0f;
    float s = 0.0f;
    float yaw = 0.0f;
    [[nodiscard]] Vec3 in(Vec3 v) const { return { c * v.x + s * v.y, -s * v.x + c * v.y, v.z }; }
};

Heading headingOf(float yaw) {
    Heading h;
    h.yaw = yaw;
    h.c = std::cos(yaw);
    h.s = std::sin(yaw);
    return h;
}

// O rumo do corpo pela pelve (o referencial da rede, o mesmo das correcoes).
float pelvisHeading(const RagdollProfile3D& profile, Quaternion orientation) {
    return locomotionFrameYaw3D(profile, orientation);
}

class Writer {
public:
    explicit Writer(std::span<float> out) : m_out(out) {}
    void put(float value) {
        // (Alem do fim, so conta: o teste confere que o escrito e o layout.)
        if (m_index >= m_out.size()) {
            ++m_index;
            return;
        }
        if (!std::isfinite(value)) {
            ++m_nonFinite;
            value = 0.0f;
        }
        m_out[m_index++] = value;
    }
    void put(Vec3 v) {
        put(v.x);
        put(v.y);
        put(v.z);
    }
    void put2(Vec3 v) {
        put(v.x);
        put(v.y);
    }
    void zeros(std::size_t count) {
        for (std::size_t i = 0; i < count; ++i) put(0.0f);
    }
    [[nodiscard]] std::size_t index() const { return m_index; }
    [[nodiscard]] std::size_t nonFinite() const { return m_nonFinite; }

private:
    std::span<float> m_out;
    std::size_t m_index = 0;
    std::size_t m_nonFinite = 0;
};

void addChannel(RLVectorLayout3D& layout, std::string name, const char* unit) {
    layout.channels.push_back({ std::move(name), unit });
}

void addVec(RLVectorLayout3D& layout, const std::string& name, const char* unit, int components = 3) {
    static const char* const axes[3] = { "x", "y", "z" };
    for (int i = 0; i < components; ++i) addChannel(layout, name + "." + axes[i], unit);
}

std::uint64_t fnv(std::uint64_t value, const std::string& text) {
    for (unsigned char c : text) {
        value ^= c;
        value *= 1099511628211ull;
    }
    value ^= 0xFFu;
    value *= 1099511628211ull;
    return value;
}

} // namespace

LocomotionContract3D::LocomotionContract3D(const RagdollProfile3D& profile) : m_profile(&profile) {
    m_feet = { findLink(profile, "LeftFoot"), findLink(profile, "RightFoot") };
    m_toes = { findLink(profile, "LeftToes"), findLink(profile, "RightToes") };
    if (m_toes[0] < 0) m_toes = { findLink(profile, "LeftToe"), findLink(profile, "RightToe") };
    m_keyParts = { m_feet[0], m_feet[1], findLink(profile, "LeftHand"), findLink(profile, "RightHand"),
        findLink(profile, "Chest"), findLink(profile, "Head") };
    for (std::size_t side = 0; side < 2; ++side) {
        const int thigh = findLink(profile, side == 0 ? "LeftThigh" : "RightThigh");
        if (thigh >= 0) {
            const Vec3 hip = profile.links[static_cast<std::size_t>(thigh)].inboundJoint.anchorModelPosition
                - profile.links.front().modelPosition;
            m_hipInBody[side] = profile.links.front().modelOrientation.conjugate().rotate(hip);
        }
    }
    for (std::size_t i = 1; i < profile.links.size(); ++i)
        for (std::size_t axis = 0; axis < 3; ++axis)
            if (profile.links[i].inboundJoint.axes[axis].enabled)
                m_axes.push_back(static_cast<std::uint32_t>(i * 3 + axis));

    // A entrada.
    RLVectorLayout3D& o = m_observation;
    o.id = "locomocao.observacao";
    o.version = Version;
    addChannel(o, "corpo.pelve.altura", "m");
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const std::string link = "corpo." + profile.links[i].id;
        if (i > 0) addVec(o, link + ".pos", "m");
        addVec(o, link + ".frente", "1");
        addVec(o, link + ".cima", "1");
        addVec(o, link + ".vel", "m/s");
        addVec(o, link + ".giro", "rad/s");
    }
    for (std::uint32_t key : m_axes) {
        const std::string axis = "junta." + profile.links[key / 3].id + "." + AxisNames[key % 3];
        addChannel(o, axis + ".angulo", "rad");
        addChannel(o, axis + ".vel", "10 rad/s");
    }
    for (const char* side : { "esq", "dir" }) {
        addChannel(o, std::string("pe_") + side + ".contato", "1");
        addChannel(o, std::string("pe_") + side + ".apoio", "peso");
    }
    addVec(o, "comando.vel", "m/s", 2);
    addChannel(o, "comando.rapidez", "m/s");
    addChannel(o, "comando.sprint", "1");
    addChannel(o, "comando.olhar.sen", "1");
    addChannel(o, "comando.olhar.cos", "1");
    addChannel(o, "comando.olhar.inclina", "rad");
    addChannel(o, "ref.valida", "1");
    addChannel(o, "ref.fase.sen", "1");
    addChannel(o, "ref.fase.cos", "1");
    addChannel(o, "ref.andando", "1");
    addChannel(o, "ref.ciclo", "1");
    addChannel(o, "ref.no_ar", "1");
    for (const char* side : { "esq", "dir" }) {
        const std::string foot = std::string("ref.pe_") + side;
        addChannel(o, foot + ".apoiado", "1");
        addChannel(o, foot + ".sai_em", "s");
        addChannel(o, foot + ".pousa_em", "s");
        addChannel(o, foot + ".pouso_valido", "1");
        addVec(o, foot + ".pouso", "m");
    }
    for (const char* when : { "0_10", "0_25", "0_50" }) {
        addVec(o, std::string("ref.plano.") + when + ".pos", "m", 2);
        addVec(o, std::string("ref.plano.") + when + ".vel", "m/s", 2);
    }
    for (const char* part : KeyPartNames) addVec(o, std::string("ref.estilo_erro.") + part, "m");
    for (const char* side : { "esq", "dir" }) {
        const std::string foot = std::string("classico.pe_") + side;
        addChannel(o, foot + ".balanco", "1");
        addVec(o, foot + ".pouso", "m");
        addChannel(o, foot + ".falta", "1");
    }
    for (const char* part : KeyPartNames) addVec(o, std::string("classico.pose_erro.") + part, "m");
    addChannel(o, "ajuda.impacto", "1");
    addChannel(o, "ajuda.andando", "1");
    addChannel(o, "ajuda.parado", "1");
    addVec(o, "ajuda.forca", "peso");
    addVec(o, "ajuda.torque", "peso.m");

    // O executor (a entrada causal da v2).
    for (const char* state : { "parado", "andando", "correndo", "ar", "chao" })
        addChannel(o, std::string("exec.estado.") + state, "1");
    for (const char* sector : { "frente", "esq", "dir", "tras" }) addChannel(o, std::string("exec.marcha.") + sector, "1");
    addChannel(o, "exec.ciclo.sen", "1");
    addChannel(o, "exec.ciclo.cos", "1");
    for (const char* side : { "esq", "dir" }) {
        const std::string foot = std::string("exec.pe_") + side;
        addChannel(o, foot + ".fase.sen", "1");
        addChannel(o, foot + ".fase.cos", "1");
        addChannel(o, foot + ".balanco", "1");
        addChannel(o, foot + ".contato_clipe", "1");
        addChannel(o, foot + ".avanco", "ciclo");
        addChannel(o, foot + ".espera", "1");
        for (const char* phase : { "apoio", "descarrega", "balanco", "procura", "carrega" })
            addChannel(o, foot + ".plano." + phase, "1");
        addChannel(o, foot + ".dono", "1");
    }
    addChannel(o, "exec.ritmo", "1");
    addChannel(o, "exec.passada", "1");
    addChannel(o, "exec.recupera.urgencia", "1");
    addVec(o, "exec.captura", "m", 2);
    addChannel(o, "exec.captura.fora", "m");
    addVec(o, "exec.referencia", "m");
    addChannel(o, "exec.ajuda.pernas", "1");
    addChannel(o, "exec.ajuda.equilibrio", "1");
    addVec(o, "exec.vmc.pedida", "g", 2);
    addVec(o, "exec.vmc.alocada", "g", 2);
    addVec(o, "exec.vmc.resto", "g", 2);
    addChannel(o, "exec.margem", "m");

    // A saida (a unidade e a escala maxima do -1..1).
    RLVectorLayout3D& a = m_action;
    a.id = "locomocao.acao";
    a.version = Version;
    const auto action = [&](std::string name, const char* unit, Channel kind, std::uint32_t index) {
        addChannel(a, std::move(name), unit);
        m_specs.push_back({ kind, index });
    };
    for (std::uint32_t side = 0; side < 2; ++side) {
        const std::string foot = std::string("pes.") + (side == 0 ? "esq" : "dir");
        action(foot + ".pouso.x", "0.30 m", Channel::TouchdownX, side);
        action(foot + ".pouso.y", "0.30 m", Channel::TouchdownY, side);
    }
    for (std::uint32_t side = 0; side < 2; ++side)
        action(std::string("pes.") + (side == 0 ? "esq" : "dir") + ".altura", "+0.10/-0.03 m", Channel::SwingHeight, side);
    for (std::uint32_t side = 0; side < 2; ++side)
        action(std::string("pes.") + (side == 0 ? "esq" : "dir") + ".ritmo", "0.10 ciclo", Channel::PhaseLead, side);
    for (std::uint32_t side = 0; side < 2; ++side)
        action(std::string("pes.") + (side == 0 ? "esq" : "dir") + ".recupera", "0..1", Channel::Recovery, side);
    action("passada.cadencia", "0.5 oitava", Channel::Cadence, 0);
    action("passada.comprimento", "0.5 oitava", Channel::Stride, 0);
    action("equilibrio.inclina.frente", "0.30 rad", Channel::LeanForward, 0);
    action("equilibrio.inclina.lado", "0.30 rad", Channel::LeanLateral, 0);
    action("equilibrio.cm.x", "0.20 m", Channel::ComX, 0);
    action("equilibrio.cm.y", "0.20 m", Channel::ComY, 0);
    action("equilibrio.altura", "+0.05/-0.15 m", Channel::PelvisHeight, 0);
    action("equilibrio.contato.x", "4 m/s2", Channel::ContactX, 0);
    action("equilibrio.contato.y", "4 m/s2", Channel::ContactY, 0);
    for (std::uint32_t key : m_axes)
        action("pose." + profile.links[key / 3].id + "." + AxisNames[key % 3], "0.6 rad", Channel::Joint, key);
    action("ganho.pernas", "oitava", Channel::Gain, 0);
    action("ganho.tronco", "oitava", Channel::Gain, 1);
    action("ganho.bracos", "oitava", Channel::Gain, 2);

    // O historico das acoes entra na entrada.
    for (std::size_t d = 0; d < HistoryDecisions; ++d)
        for (const RLChannel3D& channel : m_action.channels)
            addChannel(o, "antes_" + std::to_string(d + 1) + "." + channel.name, channel.unit.c_str());

    std::uint64_t hash = 1469598103934665603ull;
    hash = fnv(hash, "locomocao.v" + std::to_string(Version) + "." + profile.id + "."
        + std::to_string(profile.links.size()));
    for (const RLVectorLayout3D* layout : { &m_observation, &m_action })
        for (const RLChannel3D& channel : layout->channels) hash = fnv(fnv(hash, channel.name), channel.unit);
    m_hash = hash;
}

LocomotionObservation3D LocomotionContract3D::observe(const RagdollState3D& body, const AthleteCommand3D& command,
    const AthleteControlView3D& view, std::span<const float> history, BodyForecast3D& forecast,
    std::span<float> out) const {
    const RagdollProfile3D& profile = *m_profile;
    Writer w(out);
    if (body.links.size() != profile.links.size() || body.links.empty()) {
        std::fill(out.begin(), out.end(), 0.0f);
        return {};
    }
    const PhysicsBodyState3D& pelvis = body.links.front();
    const Heading h = headingOf(pelvisHeading(profile, pelvis.orientation));
    const float weight = Gravity * std::max(1.0f, profile.totalMassKg);
    const Vec3 pelvisGround { pelvis.position.x, pelvis.position.y, view.groundHeight };

    // O corpo.
    w.put(pelvis.position.z - view.groundHeight);
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const PhysicsBodyState3D& link = body.links[i];
        if (i > 0) w.put(h.in(link.position - pelvis.position));
        const Quaternion rotation = (link.orientation * profile.links[i].modelOrientation.conjugate()).normalized();
        w.put(h.in(rotation.rotate({ 1.0f, 0.0f, 0.0f })));
        w.put(h.in(rotation.rotate({ 0.0f, 0.0f, 1.0f })));
        w.put(h.in(link.linearVelocity));
        w.put(h.in(link.angularVelocity));
    }
    for (std::uint32_t key : m_axes) {
        const std::size_t link = key / 3, axis = key % 3;
        const RagdollJointState3D* joint = link < body.joints.size() ? &body.joints[link] : nullptr;
        w.put(joint != nullptr ? joint->positionRadians[axis] : 0.0f);
        w.put(joint != nullptr ? joint->velocityRadiansPerSecond[axis] * 0.1f : 0.0f);
    }
    for (std::size_t side = 0; side < 2; ++side) {
        bool contact = false;
        float impulse = 0.0f;
        for (const RagdollInteraction3D& item : body.interactions) {
            const int link = static_cast<int>(item.linkIndex);
            if (link != m_feet[side] && link != m_toes[side]) continue;
            contact = true;
            impulse += item.normalImpulseNewtonSeconds;
        }
        w.put(contact ? 1.0f : 0.0f);
        w.put(std::clamp(impulse / (weight * TickSeconds), 0.0f, 4.0f));
    }

    // O comando.
    const Vec3 asked { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
    w.put2(h.in(asked));
    w.put(asked.length());
    w.put(command.sprinting ? 1.0f : 0.0f);
    w.put(std::sin(command.facingYawRadians - h.yaw));
    w.put(std::cos(command.facingYawRadians - h.yaw));
    w.put(command.lookPitchRadians);

    // A referencia viavel.
    const std::size_t referenceSize = 6 + 2 * 7 + ReferenceFrame3D::PlanCount * 4 + 6 * 3;
    if (view.reference.valid) {
        ReferenceFrameInput3D input;
        input.pelvisGround = pelvisGround;
        input.velocity = { pelvis.linearVelocity.x, pelvis.linearVelocity.y, 0.0f };
        input.requested = asked;
        input.facingYaw = h.yaw;
        input.facingRate = pelvis.angularVelocity.z;
        input.profile = view.navigation;
        input.hipInBody = m_hipInBody;
        const ReferenceFrame3D frame = buildReferenceFrame3D(view.reference, input, forecast);
        w.put(frame.valid ? 1.0f : 0.0f);
        w.put(std::sin(frame.phase * TwoPi));
        w.put(std::cos(frame.phase * TwoPi));
        w.put(frame.movingShare);
        w.put(frame.cycling ? 1.0f : 0.0f);
        w.put(frame.airborne ? 1.0f : 0.0f);
        for (const ReferenceFootGoal3D& foot : frame.feet) {
            w.put(foot.planted ? 1.0f : 0.0f);
            w.put(std::clamp(foot.secondsToLiftoff, -1.0f, 2.0f));
            w.put(std::clamp(foot.secondsToTouchdown, -1.0f, 2.0f));
            w.put(foot.touchdownValid ? 1.0f : 0.0f);
            w.put(foot.touchdownValid ? h.in(foot.touchdownWorld - pelvisGround) : Vec3 {});
        }
        for (int k = 0; k < ReferenceFrame3D::PlanCount; ++k) {
            w.put2(h.in(frame.planPosition[static_cast<std::size_t>(k)] - pelvisGround));
            w.put2(h.in(frame.planVelocity[static_cast<std::size_t>(k)]));
        }
        const RagdollAnimationPose3D* style = frame.pose;
        const Heading styleHeading = headingOf(frame.facingYaw);
        for (int part : m_keyParts) {
            if (style == nullptr || part < 0 || static_cast<std::size_t>(part) >= style->linkPositions.size()) {
                w.put(Vec3 {});
                continue;
            }
            const std::size_t i = static_cast<std::size_t>(part);
            const Vec3 wanted = styleHeading.in(style->linkPositions[i] - frame.rootPosition);
            const Vec3 actual = h.in(body.links[i].position - pelvis.position);
            w.put(wanted - actual);
        }
    } else {
        w.zeros(referenceSize);
    }

    // O que o classico propos.
    for (const AthleteControlView3D::ClassicFoot& foot : view.classicFeet) {
        w.put(foot.swinging ? 1.0f : 0.0f);
        w.put(foot.swinging ? h.in(foot.goalWorld - pelvisGround) : Vec3 {});
        w.put(foot.swinging ? foot.swingLeft : 0.0f);
    }
    const RagdollAnimationPose3D* classic = view.classicPose;
    const bool classicValid = classic != nullptr && classic->linkPositions.size() == profile.links.size();
    for (int part : m_keyParts) {
        if (!classicValid || part < 0) {
            w.put(Vec3 {});
            continue;
        }
        const std::size_t i = static_cast<std::size_t>(part);
        const Vec3 wanted = h.in(classic->linkPositions[i] - classic->linkPositions.front());
        const Vec3 actual = h.in(body.links[i].position - pelvis.position);
        w.put(wanted - actual);
    }

    // A ajuda.
    w.put(view.impactLevel);
    w.put(view.walkingScale);
    w.put(view.standingScale);
    w.put(h.in(view.pelvisForceWorld) * (1.0f / weight));
    w.put(h.in(view.pelvisTorqueWorld) * (1.0f / weight));

    // O executor.
    const AthleteControlView3D::Executor& exec = view.executor;
    {
        using State = CharacterLocomotionState3D;
        const State state = static_cast<State>(exec.state);
        const bool still = state == State::Idle || state == State::Turning || state == State::CrouchIdle;
        const bool walking = state == State::Walking || state == State::CrouchWalking;
        const bool running = state == State::Running;
        const bool air = state == State::JumpStarting || state == State::Airborne || state == State::Landing;
        w.put(still ? 1.0f : 0.0f);
        w.put(walking ? 1.0f : 0.0f);
        w.put(running ? 1.0f : 0.0f);
        w.put(air ? 1.0f : 0.0f);
        w.put(!still && !walking && !running && !air ? 1.0f : 0.0f);
        using Sector = CharacterGaitDirection3D;
        const Sector sector = static_cast<Sector>(exec.gaitDirection);
        w.put(sector == Sector::Forward ? 1.0f : 0.0f);
        w.put(sector == Sector::Left ? 1.0f : 0.0f);
        w.put(sector == Sector::Right ? 1.0f : 0.0f);
        w.put(sector == Sector::Backward ? 1.0f : 0.0f);
    }
    w.put(std::sin(exec.cyclePhase * TwoPi));
    w.put(std::cos(exec.cyclePhase * TwoPi));
    for (std::size_t side = 0; side < 2; ++side) {
        w.put(std::sin(exec.legPhase[side] * TwoPi));
        w.put(std::cos(exec.legPhase[side] * TwoPi));
        w.put(std::clamp(exec.swingProgress[side], -1.0f, 1.0f));
        w.put(exec.clipContact[side]);
        w.put(exec.classicLead[side]);
        w.put(exec.waitingLeg == static_cast<int>(side) ? 1.0f : 0.0f);
        for (int phase = 0; phase < 5; ++phase) w.put(exec.plannerPhase[side] == phase ? 1.0f : 0.0f);
        w.put(exec.plannerOwns[side] ? 1.0f : 0.0f);
    }
    w.put(exec.playbackRate);
    w.put(exec.strideScale);
    w.put(exec.recoveryUrgency);
    w.put2(h.in(Vec3 { exec.capturePointWorld.x - pelvis.position.x, exec.capturePointWorld.y - pelvis.position.y, 0.0f }));
    w.put(std::clamp(exec.captureOutside, 0.0f, 2.0f));
    w.put(h.in(exec.referenceRootWorld - pelvis.position));
    w.put(exec.legShare);
    w.put(exec.balanceShare);
    w.put2(h.in(exec.comAccelerationRequested) * (1.0f / Gravity));
    w.put2(h.in(exec.comAccelerationAllocated) * (1.0f / Gravity));
    w.put2(h.in(exec.comAccelerationResidual) * (1.0f / Gravity));
    w.put(std::clamp(exec.balanceMargin, -1.0f, 1.0f));

    // As decisoes anteriores.
    const std::size_t historySize = HistoryDecisions * actionSize();
    for (std::size_t i = 0; i < historySize; ++i) w.put(i < history.size() ? history[i] : 0.0f);
    // (Se faltou canal, o resto fica zero.)
    for (std::size_t i = std::min(w.index(), out.size()); i < out.size(); ++i) out[i] = 0.0f;
    return { w.index(), w.nonFinite() };
}

LocomotionCaps3D LocomotionCaps3D::full() {
    using C = LocomotionContract3D;
    LocomotionCaps3D caps;
    caps.touchdownMeters = C::TouchdownMeters;
    caps.swingUpMeters = C::SwingUpMeters;
    caps.swingDownMeters = C::SwingDownMeters;
    caps.phaseLeadCycles = C::PhaseLeadCycles;
    caps.recoveryUrgency = 1.0f;
    caps.cadenceOctaves = C::CadenceOctaves;
    caps.strideOctaves = C::StrideOctaves;
    caps.leanRadians = C::LeanRadians;
    caps.comMeters = C::ComMeters;
    caps.pelvisUpMeters = C::PelvisUpMeters;
    caps.pelvisDownMeters = C::PelvisDownMeters;
    caps.contactAcceleration = C::ContactAcceleration;
    caps.jointRadians = C::JointRadians;
    caps.gainOctaves = C::GainOctaves;
    return caps;
}

std::vector<float> LocomotionContract3D::physical(std::span<const float> action,
    const LocomotionAuthority3D& authority) const {
    const auto unit = [](float a) { return std::isfinite(a) ? std::clamp(a, 0.0f, 1.0f) : 0.0f; };
    const LocomotionCaps3D& caps = authority.caps;
    std::vector<float> out(m_specs.size(), 0.0f);
    for (std::size_t i = 0; i < m_specs.size(); ++i) {
        const float raw = i < action.size() ? action[i] : 0.0f;
        const float v = std::isfinite(raw) ? std::clamp(raw, -1.0f, 1.0f) : 0.0f;
        // A escala (assimetrica onde o canal e), o teto e a cabeca.
        float up = 0.0f, down = 0.0f, capUp = 0.0f, capDown = 0.0f, head = 0.0f;
        const auto both = [&](float scale, float cap, float a) { up = down = scale; capUp = capDown = cap; head = a; };
        switch (m_specs[i].kind) {
        case Channel::TouchdownX:
        case Channel::TouchdownY: both(TouchdownMeters, caps.touchdownMeters, authority.footwork); break;
        case Channel::SwingHeight:
            up = SwingUpMeters, down = SwingDownMeters, capUp = caps.swingUpMeters, capDown = caps.swingDownMeters;
            head = authority.footwork;
            break;
        case Channel::PhaseLead: both(PhaseLeadCycles, caps.phaseLeadCycles, authority.footwork); break;
        case Channel::Recovery:
            // (So a metade positiva: a acao no meio nao pede passo nenhum.)
            up = 1.0f, down = 0.0f, capUp = caps.recoveryUrgency, capDown = 0.0f, head = authority.footwork;
            break;
        case Channel::Cadence: both(CadenceOctaves, caps.cadenceOctaves, authority.footwork); break;
        case Channel::Stride: both(StrideOctaves, caps.strideOctaves, authority.footwork); break;
        case Channel::LeanForward:
        case Channel::LeanLateral: both(LeanRadians, caps.leanRadians, authority.balance); break;
        case Channel::ComX:
        case Channel::ComY: both(ComMeters, caps.comMeters, authority.balance); break;
        case Channel::PelvisHeight:
            up = PelvisUpMeters, down = PelvisDownMeters, capUp = caps.pelvisUpMeters, capDown = caps.pelvisDownMeters;
            head = authority.balance;
            break;
        case Channel::ContactX:
        case Channel::ContactY: both(ContactAcceleration, caps.contactAcceleration, authority.balance); break;
        case Channel::Joint: both(JointRadians, caps.jointRadians, authority.pose); break;
        case Channel::Gain: both(GainOctaves, caps.gainOctaves, authority.gain); break;
        }
        const float value = v >= 0.0f ? std::min(v * up, std::max(0.0f, capUp)) : std::max(v * down, -std::max(0.0f, capDown));
        out[i] = value * unit(head);
    }
    return out;
}

PhysicalCorrections3D LocomotionContract3D::corrections(std::span<const float> values,
    const LocomotionAuthority3D& authority) const {
    const auto on = [](float a) { return std::isfinite(a) && a > 0.0f; };
    PhysicalCorrections3D out;
    out.footwork = on(authority.footwork);
    out.balance = on(authority.balance);
    out.pose = on(authority.pose);
    out.gain = on(authority.gain);
    // (Uma cabeca que perdeu a autoridade segue ligada ate o valor dela
    // voltar a zero pelo limite de taxa: desligada de uma vez, saltava.)
    for (std::size_t i = 0; i < m_specs.size() && i < values.size(); ++i) {
        if (values[i] == 0.0f) continue;
        switch (m_specs[i].kind) {
        case Channel::Joint: out.pose = true; break;
        case Channel::Gain: out.gain = true; break;
        case Channel::LeanForward: case Channel::LeanLateral: case Channel::ComX: case Channel::ComY:
        case Channel::PelvisHeight: case Channel::ContactX: case Channel::ContactY: out.balance = true; break;
        default: out.footwork = true; break;
        }
    }
    if (out.pose) out.jointResiduals.assign(m_profile->links.size() * 3, 0.0f);
    for (std::size_t i = 0; i < m_specs.size() && i < values.size(); ++i) {
        const float value = values[i];
        const std::uint32_t index = m_specs[i].index;
        switch (m_specs[i].kind) {
        case Channel::TouchdownX: out.feet[index].touchdownOffset.x = value; break;
        case Channel::TouchdownY: out.feet[index].touchdownOffset.y = value; break;
        case Channel::SwingHeight: out.feet[index].swingHeight = value; break;
        case Channel::PhaseLead: out.feet[index].phaseLead = value; break;
        case Channel::Recovery: out.feet[index].recoveryUrgency = value; break;
        case Channel::Cadence: out.cadenceOctaves = value; break;
        case Channel::Stride: out.strideOctaves = value; break;
        case Channel::LeanForward: out.forwardLean = value; break;
        case Channel::LeanLateral: out.lateralLean = value; break;
        case Channel::ComX: out.comOffset.x = value; break;
        case Channel::ComY: out.comOffset.y = value; break;
        case Channel::PelvisHeight: out.pelvisHeight = value; break;
        case Channel::ContactX: out.contactAcceleration.x = value; break;
        case Channel::ContactY: out.contactAcceleration.y = value; break;
        case Channel::Joint:
            if (index < out.jointResiduals.size()) out.jointResiduals[index] = value;
            break;
        case Channel::Gain: out.gainScale[index] = std::exp2(value); break;
        }
    }
    return out;
}

namespace {

// O nome do outro lado: so palavras inteiras - "Left"/"Right" no comeco de
// um nome de link (seguido de maiuscula) e "esq"/"dir" entre separadores; um
// "direcao" nao vira "esqecao".
std::string mirroredName(const std::string& name) {
    const auto boundary = [&](std::size_t i) {
        return i >= name.size() || !(std::isalnum(static_cast<unsigned char>(name[i])) != 0);
    };
    const auto startsWord = [&](std::size_t i) { return i == 0 || boundary(i - 1); };
    const auto upper = [&](std::size_t i) {
        return i < name.size() && std::isupper(static_cast<unsigned char>(name[i])) != 0;
    };
    std::string out;
    out.reserve(name.size() + 1);
    for (std::size_t i = 0; i < name.size();) {
        if (startsWord(i) && name.compare(i, 4, "Left") == 0 && (upper(i + 4) || boundary(i + 4))) {
            out += "Right";
            i += 4;
        } else if (startsWord(i) && name.compare(i, 5, "Right") == 0 && (upper(i + 5) || boundary(i + 5))) {
            out += "Left";
            i += 5;
        } else if (startsWord(i) && name.compare(i, 3, "esq") == 0 && boundary(i + 3)) {
            out += "dir";
            i += 3;
        } else if (startsWord(i) && name.compare(i, 3, "dir") == 0 && boundary(i + 3)) {
            out += "esq";
            i += 3;
        } else {
            out += name[i++];
        }
    }
    return out;
}

bool endsWith(const std::string& text, const char* tail) {
    const std::size_t n = std::strlen(tail);
    return text.size() >= n && text.compare(text.size() - n, n, tail) == 0;
}

bool contains(const std::string& text, const char* part) { return text.find(part) != std::string::npos; }

} // namespace

LocomotionMirror3D LocomotionContract3D::mirror() const {
    const RagdollProfile3D& profile = *m_profile;
    LocomotionMirror3D out;
    // O sinal de cada eixo de junta: o eixo do quadro da junta do outro lado
    // contra o reflexo deste.
    const auto linkIndex = [&](const std::string& id) {
        for (std::size_t i = 0; i < profile.links.size(); ++i)
            if (profile.links[i].id == id) return static_cast<int>(i);
        return -1;
    };
    const auto axisSign = [&](std::size_t link, std::size_t axis) {
        const int other = linkIndex(mirroredName(profile.links[link].id));
        if (other < 0) return 1.0f;
        Vec3 unit {};
        (axis == 0 ? unit.x : axis == 1 ? unit.y : unit.z) = 1.0f;
        const Vec3 v = profile.links[link].inboundJoint.frameModelOrientation.rotate(unit);
        const Vec3 w = profile.links[static_cast<std::size_t>(other)].inboundJoint.frameModelOrientation.rotate(unit);
        const float alignment = -(v.x * w.x - v.y * w.y + v.z * w.z);
        if (std::abs(alignment) < out.worstAxisAlignment) {
            out.worstAxisAlignment = std::abs(alignment);
            out.worstAxis = profile.links[link].id + "." + AxisNames[axis];
        }
        return alignment >= 0.0f ? 1.0f : -1.0f;
    };
    const auto axisOf = [&](const std::string& name) {
        for (std::size_t a = 0; a < 3; ++a)
            if (contains(name, (std::string(".") + AxisNames[a]).c_str())) return a;
        return std::size_t { 0 };
    };
    // A regra de um canal (do contrato ou do historico).
    const auto rule = [&](const std::string& name) {
        float scale = 1.0f;
        const bool vector = endsWith(name, ".x") || endsWith(name, ".y") || endsWith(name, ".z");
        const bool pseudo = contains(name, ".giro.") || contains(name, "ajuda.torque.");
        if (vector) {
            if (pseudo) scale = endsWith(name, ".y") ? 1.0f : -1.0f;
            else scale = endsWith(name, ".y") ? -1.0f : 1.0f;
        }
        // (As fases andam meio ciclo no espelho: o seno e o cosseno trocam de
        // sinal. As das pernas tambem trocam de lado, pelo nome.)
        const bool phase = name == "ref.fase.sen" || name == "ref.fase.cos" || name == "exec.ciclo.sen"
            || name == "exec.ciclo.cos" || (contains(name, "exec.pe_") && (endsWith(name, ".fase.sen")
                || endsWith(name, ".fase.cos")));
        if (name == "comando.olhar.sen" || phase || endsWith(name, "equilibrio.inclina.lado"))
            scale = -1.0f;
        if (contains(name, "junta.") || contains(name, "pose.")) {
            const std::size_t start = contains(name, "junta.") ? name.find("junta.") + 6 : name.find("pose.") + 5;
            const std::string link = name.substr(start, name.find('.', start) - start);
            const int index = linkIndex(link);
            if (index > 0) scale = axisSign(static_cast<std::size_t>(index), axisOf(name.substr(start)));
        }
        return scale;
    };
    const auto build = [&](const RLVectorLayout3D& layout, LocomotionMirrorMap3D& map) {
        map.source.resize(layout.size());
        map.scale.resize(layout.size());
        map.offset.assign(layout.size(), 0.0f);
        for (std::size_t i = 0; i < layout.size(); ++i) {
            const std::string& name = layout.channels[i].name;
            const std::string other = mirroredName(name);
            std::uint32_t source = static_cast<std::uint32_t>(i);
            for (std::size_t j = 0; j < layout.size(); ++j)
                if (layout.channels[j].name == other) source = static_cast<std::uint32_t>(j);
            map.source[i] = source;
            // (A regra pelo nome de origem: o sinal do eixo e o do lado de la.)
            map.scale[i] = rule(layout.channels[source].name);
        }
    };
    build(m_observation, out.observation);
    build(m_action, out.action);
    return out;
}

void LocomotionContract3D::decide(AthleteState3D& athlete, std::span<const float> action) const {
    const std::size_t size = actionSize();
    std::vector<float>& history = athlete.policyHistory;
    history.resize(HistoryDecisions * size, 0.0f);
    std::copy_backward(history.begin(), history.end() - static_cast<std::ptrdiff_t>(size), history.end());
    // (O historico guarda a acao pedida: no -1..1, como physical a usa.)
    for (std::size_t i = 0; i < size; ++i)
        history[i] = i < action.size() && std::isfinite(action[i]) ? std::clamp(action[i], -1.0f, 1.0f) : 0.0f;
    // A rampa parte de onde a anterior estava.
    std::vector<float> from(size, 0.0f);
    if (athlete.policyTo.size() == size) {
        const float t = std::clamp(static_cast<float>(athlete.policyTicks) / static_cast<float>(TicksPerDecision), 0.0f, 1.0f);
        for (std::size_t i = 0; i < size; ++i)
            from[i] = (athlete.policyFrom.size() == size ? athlete.policyFrom[i] : 0.0f) * (1.0f - t)
                + athlete.policyTo[i] * t;
    }
    athlete.policyFrom = std::move(from);
    athlete.policyTo.assign(history.begin(), history.begin() + static_cast<std::ptrdiff_t>(size));
    athlete.policyTicks = 0;
}

void LocomotionContract3D::clear(AthleteState3D& athlete) {
    athlete.policyHistory.clear();
    athlete.policyFrom.clear();
    athlete.policyTo.clear();
    athlete.policyTicks = 0;
    athlete.policyApplied.clear();
    athlete.corrections = {};
}

void LocomotionContract3D::follow(AthleteState3D& athlete, const LocomotionAuthority3D& authority,
    float deltaTime) const {
    const std::size_t size = actionSize();
    if (athlete.policyTo.size() != size) {
        athlete.corrections = {};
        athlete.policyApplied.clear();
        return;
    }
    const float t = std::clamp(static_cast<float>(athlete.policyTicks + 1) / static_cast<float>(TicksPerDecision), 0.0f,
        1.0f);
    std::vector<float> action(size, 0.0f);
    for (std::size_t i = 0; i < size; ++i)
        action[i] = (athlete.policyFrom.size() == size ? athlete.policyFrom[i] : 0.0f) * (1.0f - t)
            + athlete.policyTo[i] * t;
    // O limite de taxa de cada canal (por segundo): o aplicado anda ate o
    // pedido no maximo isto por tick.
    const std::vector<float> wanted = physical(action, authority);
    athlete.policyApplied.resize(size, 0.0f);
    const float dt = std::isfinite(deltaTime) ? std::clamp(deltaTime, 0.0f, 0.05f) : 0.0f;
    for (std::size_t i = 0; i < size; ++i) {
        float rate = 0.0f;
        switch (m_specs[i].kind) {
        case Channel::TouchdownX: case Channel::TouchdownY: rate = TouchdownRate; break;
        case Channel::SwingHeight: rate = SwingRate; break;
        case Channel::PhaseLead: rate = PhaseLeadRate; break;
        case Channel::Recovery: rate = -1.0f; break;
        case Channel::Cadence: case Channel::Stride: rate = OctaveRate; break;
        case Channel::LeanForward: case Channel::LeanLateral: rate = LeanRate; break;
        case Channel::ComX: case Channel::ComY: rate = ComRate; break;
        case Channel::PelvisHeight: rate = PelvisRate; break;
        case Channel::ContactX: case Channel::ContactY: rate = ContactRate; break;
        case Channel::Joint: rate = JointRate; break;
        case Channel::Gain: rate = GainRate; break;
        }
        float& applied = athlete.policyApplied[i];
        if (!std::isfinite(applied)) applied = 0.0f;
        // (A urgencia de recuperacao nao tem limite: a histerese e do planejador.)
        applied = rate < 0.0f ? wanted[i] : applied + std::clamp(wanted[i] - applied, -rate * dt, rate * dt);
    }
    const Quaternion frame = athlete.corrections.frame;
    athlete.corrections = corrections(athlete.policyApplied, authority);
    athlete.corrections.frame = frame;
    ++athlete.policyTicks;
}

} // namespace MatterEngine
