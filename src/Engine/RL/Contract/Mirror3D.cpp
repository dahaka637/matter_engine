#include "Engine/RL/Contract/Mirror3D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace MatterEngine {
namespace {

constexpr float Probe = 0.35f;
const char* const AxisNames[3] = { "twist", "swing1", "swing2" };

std::string swapSide(const std::string& token) {
    if (token.rfind("Left", 0) == 0) return "Right" + token.substr(4);
    if (token.rfind("Right", 0) == 0) return "Left" + token.substr(5);
    return token;
}

std::vector<std::string> split(const std::string& name) {
    std::vector<std::string> tokens;
    std::stringstream stream(name);
    std::string token;
    while (std::getline(stream, token, '.')) tokens.push_back(token);
    return tokens;
}

std::string join(const std::vector<std::string>& tokens) {
    std::string name;
    for (std::size_t i = 0; i < tokens.size(); ++i) name += (i ? "." : "") + tokens[i];
    return name;
}

// Um colisor numa pose: o centro, a orientacao e os pontos dele no mundo (o
// centro e os cantos).
struct Placed {
    Vec3 center;
    Quaternion orientation;
    std::vector<Vec3> points;
};

std::vector<Placed> placeColliders(const RagdollProfile3D& profile, const RLBodyPose3D& pose) {
    std::vector<Placed> placed(profile.links.size());
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const RagdollCapsuleDefinition3D& collider = profile.links[i].collider;
        Placed& p = placed[i];
        p.center = pose.positions[i] + pose.orientations[i].rotate(collider.localPosition);
        p.orientation = (pose.orientations[i] * collider.localOrientation).normalized();
        p.points.push_back(p.center);
        for (const Vec3& corner : ragdollColliderCorners3D(collider)) p.points.push_back(p.center + p.orientation.rotate(corner));
    }
    return placed;
}

Vec3 reflect(Vec3 p) { return { p.x, -p.y, p.z }; }

float signedDistance(const RagdollCapsuleDefinition3D& collider, const Placed& at, Vec3 p) {
    return ragdollColliderClosestPoint3D(collider, at.orientation.conjugate().rotate(p - at.center)).distance;
}

// Os pontos de um colisor (from, na pose dele) refletidos contra o colisor do
// outro lado (to): a distancia com sinal de cada ponto ate o proprio colisor
// tem de ser a do refletido ate o outro (a reflexao preserva distancias). Mede
// a forma, nao a triangulacao nem onde os pontos caem.
float gap(const RagdollCapsuleDefinition3D& from, const Placed& fromAt, const RagdollCapsuleDefinition3D& to,
    const Placed& toAt) {
    float worst = 0.0f;
    for (const Vec3& p : fromAt.points)
        worst = std::max(worst, std::abs(signedDistance(to, toAt, reflect(p)) - signedDistance(from, fromAt, p)));
    return worst;
}

} // namespace

void MirrorChannelMap3D::apply(const float* in, float* out) const {
    for (std::size_t c = 0; c < source.size(); ++c) out[c] = in[source[c]] * scale[c] + offset[c];
}

Mirror3D::Mirror3D(const RLBody3D& body) : m_body(body) {
    const RagdollProfile3D& profile = m_body.profile;
    const std::size_t count = profile.links.size();
    std::unordered_map<std::string, std::uint32_t> linkIndex;
    for (std::size_t i = 0; i < count; ++i) linkIndex[profile.links[i].id] = static_cast<std::uint32_t>(i);
    m_linkCounterpart.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        const auto found = linkIndex.find(swapSide(profile.links[i].id));
        if (found == linkIndex.end()) throw std::runtime_error("espelho: " + profile.links[i].id + " sem o outro lado");
        m_linkCounterpart[i] = found->second;
    }

    // As poses com a raiz na origem, na orientacao de referencia (a frente em
    // +x, a esquerda em +y): o espelho e y -> -y.
    const Quaternion root = profile.links.front().modelOrientation;
    const auto place = [&](const std::vector<Vec3>& coordinates) {
        return placeColliders(profile, rlBodyPose3D(profile, {}, root, coordinates));
    };
    // O link k de a refletido contra o do outro lado em b, nos dois sentidos.
    const auto linkMismatch = [&](const std::vector<Placed>& a, const std::vector<Placed>& b, std::size_t k) {
        const std::uint32_t other = m_linkCounterpart[k];
        const RagdollCapsuleDefinition3D& mine = profile.links[k].collider;
        const RagdollCapsuleDefinition3D& theirs = profile.links[other].collider;
        return std::max(gap(mine, a[k], theirs, b[other]), gap(theirs, b[other], mine, a[k]));
    };
    const auto mismatch = [&](const std::vector<Placed>& a, const std::vector<Placed>& b) {
        float worst = 0.0f;
        for (std::size_t k = 0; k < count; ++k) worst = std::max(worst, linkMismatch(a, b, k));
        return worst;
    };
    const std::vector<Vec3> zero(count);
    const std::vector<Placed> bind = place(zero);
    for (std::size_t k = 0; k < count; ++k) m_linkAsymmetry.push_back(linkMismatch(bind, bind, k));
    m_bindAsymmetry = *std::max_element(m_linkAsymmetry.begin(), m_linkAsymmetry.end());

    m_axes.assign(count, {});
    for (std::size_t link = 0; link < count; ++link)
        for (std::size_t a = 0; a < 3; ++a) m_axes[link][a] = { m_linkCounterpart[link], 1.0f };
    for (std::size_t link = 1; link < count; ++link) {
        const RagdollJointDefinition3D& joint = profile.links[link].inboundJoint;
        for (std::size_t a = 0; a < 3; ++a) {
            if (!joint.axes[a].enabled) continue;
            std::vector<Vec3> probe = zero;
            float* value = a == 0 ? &probe[link].x : a == 1 ? &probe[link].y : &probe[link].z;
            *value = Probe;
            const std::vector<Placed> original = place(probe);
            float error[2] {};
            for (int s = 0; s < 2; ++s) {
                std::vector<Vec3> mirrored = zero;
                Vec3& q = mirrored[m_linkCounterpart[link]];
                (a == 0 ? q.x : a == 1 ? q.y : q.z) = s == 0 ? Probe : -Probe;
                error[s] = mismatch(original, place(mirrored));
            }
            const int best = error[1] < error[0] ? 1 : 0;
            m_axes[link][a].sign = best == 0 ? 1.0f : -1.0f;
            m_worstAxisError = std::max(m_worstAxisError, error[best]);
            if (std::abs(error[0] - error[1]) < 1.0e-4f)
                m_undecided.push_back(profile.links[link].id + "." + AxisNames[a]);
        }
    }

    // Os eixos atuados.
    m_dofCounterpart.resize(m_body.dofs.size());
    m_dofSign.resize(m_body.dofs.size());
    for (std::size_t d = 0; d < m_body.dofs.size(); ++d) {
        const RLBodyDof3D& dof = m_body.dofs[d];
        const auto axis = static_cast<std::size_t>(dof.axis);
        const std::uint32_t other = m_linkCounterpart[dof.linkIndex];
        bool found = false;
        for (std::size_t e = 0; e < m_body.dofs.size(); ++e)
            if (m_body.dofs[e].linkIndex == other && static_cast<std::size_t>(m_body.dofs[e].axis) == axis) {
                m_dofCounterpart[d] = static_cast<std::uint32_t>(e);
                found = true;
            }
        if (!found) throw std::runtime_error("espelho: " + dof.name + " sem o eixo do outro lado");
        m_dofSign[d] = m_axes[dof.linkIndex][axis].sign;
    }
}

void Mirror3D::mirrorCoordinates(std::span<const Vec3> in, std::vector<Vec3>& out) const {
    out.assign(in.size(), {});
    for (std::size_t link = 0; link < in.size() && link < m_axes.size(); ++link) {
        const std::uint32_t other = m_linkCounterpart[link];
        const float values[3] = { in[link].x, in[link].y, in[link].z };
        float* target[3] = { &out[other].x, &out[other].y, &out[other].z };
        for (std::size_t a = 0; a < 3; ++a) *target[a] = m_axes[link][a].sign * values[a];
    }
}

Quaternion Mirror3D::mirrorRoot(Quaternion root) const {
    // A rotacao refletida no plano y = 0: o eixo como pseudovetor, o angulo igual.
    const Quaternion model = m_body.profile.links.front().modelOrientation;
    const Quaternion frame = (root * model.conjugate()).normalized();
    const Quaternion reflected { -frame.x, frame.y, -frame.z, frame.w };
    return (reflected * model).normalized();
}

MirrorChannelMap3D Mirror3D::layoutMap(const RLVectorLayout3D& layout) const {
    std::unordered_map<std::string, std::uint32_t> index;
    for (std::size_t c = 0; c < layout.channels.size(); ++c) index[layout.channels[c].name] = static_cast<std::uint32_t>(c);
    std::unordered_map<std::string, std::uint32_t> dofIndex;
    for (std::size_t d = 0; d < m_body.dofs.size(); ++d) dofIndex[m_body.dofs[d].name] = static_cast<std::uint32_t>(d);
    const bool style = layout.id == "matter.rl.style";
    MirrorChannelMap3D map;
    for (const RLChannel3D& channel : layout.channels) {
        std::vector<std::string> tokens = split(channel.name);
        std::vector<std::string> swapped = tokens;
        for (std::string& token : swapped) token = swapSide(token);
        const auto source = index.find(join(swapped));
        if (source == index.end()) throw std::runtime_error("espelho: " + channel.name + " sem o canal do outro lado");
        float scale = 1.0f, offset = 0.0f;
        // Um eixo atuado: <link>.<eixo>.<quantidade>.
        int dofAt = -1;
        for (std::size_t i = 0; i + 1 < tokens.size() && dofAt < 0; ++i)
            if (dofIndex.count(tokens[i] + "." + tokens[i + 1])) dofAt = static_cast<int>(i);
        if (dofAt >= 0) {
            const std::uint32_t out = dofIndex.at(tokens[static_cast<std::size_t>(dofAt)] + "." + tokens[static_cast<std::size_t>(dofAt) + 1]);
            const std::uint32_t from = m_dofCounterpart[out];
            const float s = m_dofSign[from];
            const RLBodyDof3D& a = m_body.dofs[from];
            const RLBodyDof3D& b = m_body.dofs[out];
            const std::string quantity = static_cast<std::size_t>(dofAt) + 2 < tokens.size() ? tokens[static_cast<std::size_t>(dofAt) + 2] : "";
            if (quantity == "velocity" || (style && quantity == "position")) {
                scale = s;
            } else if (quantity == "position_from_base" || quantity == "position") {
                // O desvio da base: a base pode nao ser simetrica.
                scale = s;
                offset = s * a.baseRadians - b.baseRadians;
            } else if (quantity == "previous_action" || quantity == "action" || quantity == "target") {
                scale = s * a.actionScale / b.actionScale;
                offset = (s * a.baseRadians - b.baseRadians) / b.actionScale;
            } else {
                throw std::runtime_error("espelho: quantidade desconhecida em " + channel.name);
            }
        } else {
            const std::string last = tokens.back();
            const std::string base = tokens.size() >= 2 ? tokens[tokens.size() - 2] : "";
            const bool spin = base.find("angular_velocity") != std::string::npos || base == "spin_local";
            if (channel.name == "command.face.sin" || channel.name == "command.face.rate") scale = -1.0f;
            else if (last == "x" || last == "z") scale = spin ? -1.0f : 1.0f;
            else if (last == "y") scale = spin ? 1.0f : -1.0f;
        }
        map.source.push_back(source->second);
        map.scale.push_back(scale);
        map.offset.push_back(offset);
    }
    return map;
}

} // namespace MatterEngine
