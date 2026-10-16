#include "Engine/Athlete/Contract/LocomotionContract3D.hpp"

#include <algorithm>
#include <cmath>
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

// O rumo do corpo pela pelve: a frente dela no plano e a perpendicular do
// eixo do lado dela, pesadas pelo quanto cada uma esta deitada no plano (de
// pe as duas concordam; tombando para a frente ou para tras a frente fica
// vertical e o lado segura o rumo; de lado, o contrario). Todo rumo tirado de
// uma orientacao tem alguma singularidade; a desta fica de cabeca para baixo
// (a pelve virada 180 graus por um eixo do chao: as duas se anulam), onde o
// rumo vira 0 - raro, e so naquele instante.
float pelvisHeading(const RagdollProfile3D& profile, Quaternion orientation) {
    const Quaternion body = (orientation * profile.links.front().modelOrientation.conjugate()).normalized();
    const Vec3 forward = body.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 left = body.rotate({ 0.0f, 1.0f, 0.0f });
    const float fx = forward.x, fy = forward.y;
    const float lx = left.y, ly = -left.x;
    const float fw = std::hypot(fx, fy), lw = std::hypot(lx, ly);
    const float x = fx * fw + lx * lw, y = fy * fw + ly * lw;
    if (!std::isfinite(x) || !std::isfinite(y) || x * x + y * y < 1e-12f) return 0.0f;
    return std::atan2(y, x);
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

    // A saida.
    RLVectorLayout3D& a = m_action;
    a.id = "locomocao.acao";
    a.version = Version;
    addVec(a, "pes.esq.pouso", "0.15 m");
    addVec(a, "pes.dir.pouso", "0.15 m");
    addChannel(a, "equilibrio.inclina.frente", "0.2 rad");
    addChannel(a, "equilibrio.inclina.lado", "0.2 rad");
    for (std::uint32_t key : m_axes)
        addChannel(a, "pose." + profile.links[key / 3].id + "." + AxisNames[key % 3], "0.3 rad");
    addChannel(a, "ganho.pernas", "oitava");
    addChannel(a, "ganho.tronco", "oitava");
    addChannel(a, "ganho.bracos", "oitava");

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

    // As decisoes anteriores.
    const std::size_t historySize = HistoryDecisions * actionSize();
    for (std::size_t i = 0; i < historySize; ++i) w.put(i < history.size() ? history[i] : 0.0f);
    // (Se faltou canal, o resto fica zero.)
    for (std::size_t i = std::min(w.index(), out.size()); i < out.size(); ++i) out[i] = 0.0f;
    return { w.index(), w.nonFinite() };
}

PhysicalCorrections3D LocomotionContract3D::decode(std::span<const float> action,
    const LocomotionAuthority3D& authority) const {
    const auto value = [&](std::size_t i) {
        const float v = i < action.size() ? action[i] : 0.0f;
        return std::isfinite(v) ? std::clamp(v, -1.0f, 1.0f) : 0.0f;
    };
    PhysicalCorrections3D out;
    out.footworkAuthority = authority.footwork;
    out.balanceAuthority = authority.balance;
    out.poseAuthority = authority.pose;
    out.gainAuthority = authority.gain;
    std::size_t i = 0;
    for (std::size_t side = 0; side < 2; ++side) {
        out.feet[side].touchdownOffset = { value(i) * TouchdownMeters, value(i + 1) * TouchdownMeters,
            value(i + 2) * TouchdownHeightMeters };
        i += 3;
    }
    out.forwardLean = value(i++) * LeanRadians;
    out.lateralLean = value(i++) * LeanRadians;
    out.jointResiduals.assign(m_profile->links.size() * 3, 0.0f);
    for (std::uint32_t key : m_axes) out.jointResiduals[key] = value(i++) * JointRadians;
    for (std::size_t g = 0; g < 3; ++g) out.gainScale[g] = std::exp2(value(i++) * GainOctaves);
    return out;
}

void LocomotionContract3D::decide(AthleteState3D& athlete, std::span<const float> action) const {
    const std::size_t size = actionSize();
    std::vector<float>& history = athlete.policyHistory;
    history.resize(HistoryDecisions * size, 0.0f);
    std::copy_backward(history.begin(), history.end() - static_cast<std::ptrdiff_t>(size), history.end());
    // (O historico guarda a acao aplicada: no -1..1, como o decode a usa.)
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

void LocomotionContract3D::follow(AthleteState3D& athlete, const LocomotionAuthority3D& authority) const {
    const std::size_t size = actionSize();
    if (athlete.policyTo.size() != size) {
        athlete.corrections = {};
        return;
    }
    const float t = std::clamp(static_cast<float>(athlete.policyTicks + 1) / static_cast<float>(TicksPerDecision), 0.0f,
        1.0f);
    std::vector<float> action(size, 0.0f);
    for (std::size_t i = 0; i < size; ++i)
        action[i] = (athlete.policyFrom.size() == size ? athlete.policyFrom[i] : 0.0f) * (1.0f - t)
            + athlete.policyTo[i] * t;
    athlete.corrections = decode(action, authority);
    ++athlete.policyTicks;
}

} // namespace MatterEngine
