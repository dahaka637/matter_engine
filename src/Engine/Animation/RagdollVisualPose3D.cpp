#include "Engine/Animation/RagdollVisualPose3D.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace MatterEngine {

namespace {

constexpr float Degrees = 3.14159265358979323846f / 180.0f;

// A mao aberta e a fechada: a flexao (graus) de cada junta - base, meio e
// ponta; no polegar, a base dele (perto do pulso), o meio e a ponta -, a
// fracao do giro ate o dedo de referencia (o polegar ate o indicador, os
// outros ate o medio): aberta abre alem do modelado, fechada junta - e o
// giro para dentro (graus, em volta do z do osso) do meio e da ponta. No
// punho o polegar dobra pouco (pelo eixo dele, a flexao leva o polegar para
// baixo da palma) e quase nao gira na base: o meio e a ponta giram para
// dentro e ele deita na diagonal na frente das falanges do meio do
// indicador e do medio.
struct HandPoseTable {
    float flexion[5][3];
    float close[5];
    float across[5][2];
};
// (Polegar, indicador, medio, anelar, minimo.)
constexpr HandPoseTable OpenHand {
    { { 0.0f, 4.0f, 4.0f }, { 4.0f, 3.0f, 2.0f }, { 4.0f, 3.0f, 2.0f }, { 4.0f, 3.0f, 2.0f }, { 4.0f, 3.0f, 2.0f } },
    { -0.3f, -0.25f, 0.0f, -0.25f, -0.25f },
    {} };
constexpr HandPoseTable FistHand {
    { { 21.0f, 6.0f, -2.0f }, { 85.0f, 100.0f, 70.0f }, { 85.0f, 100.0f, 70.0f }, { 85.0f, 100.0f, 70.0f },
        { 85.0f, 100.0f, 70.0f } },
    { 0.3f, 1.0f, 0.0f, 1.0f, 1.0f },
    { { 52.0f, 27.0f }, {}, {}, {}, {} } };
// A automatica: o braco esticado (cotovelo ate MinimumBend) abre a mao ate
// AutomaticOpen; dobrado ate MaximumBend, relaxada.
constexpr float AutomaticOpen = 0.6f;
constexpr float AutomaticStraightBend = 25.0f * Degrees;
constexpr float AutomaticBentBend = 60.0f * Degrees;
// A transicao da mao (s).
constexpr float HandBlendSeconds = 0.12f;
// Os dedos do pe: a maior dobra (rad) e quanto volta por s soltando.
constexpr float ToeBendMaximum = 50.0f * Degrees;
constexpr float ToeReleaseRate = 4.0f;

float smooth01(float x) {
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

int linkIndex(const RagdollProfile3D& profile, const std::string& id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return static_cast<int>(i);
    return -1;
}

} // namespace

void RagdollVisualPose3D::reset() {
    m_rotations.clear();
    m_toeBend = {};
    m_open = {};
    m_fist = {};
    m_started = false;
}

const std::vector<Quaternion>& RagdollVisualPose3D::update(const RagdollCharacter3D& character,
    const RagdollVisualPoseInput3D& input) {
    const auto& visuals = character.visualBones;
    m_rotations.resize(visuals.size());
    const std::size_t linkCount = character.profile.links.size();
    if (input.linkPositions.size() < linkCount || input.linkOrientations.size() < linkCount) {
        for (std::size_t k = 0; k < visuals.size(); ++k) m_rotations[k] = visuals[k].restRotation;
        return m_rotations;
    }
    const float dt = std::max(0.0f, input.deltaTime);

    // A mao: o alvo (aberta, fechada) por lado e a transicao.
    const auto linkRotation = [&](int link) {
        // A rotacao do link desde a pose de ligacao.
        return (input.linkOrientations[static_cast<std::size_t>(link)]
            * character.profile.links[static_cast<std::size_t>(link)].modelOrientation.conjugate()).normalized();
    };
    const float blend = m_started ? 1.0f - std::exp(-dt / HandBlendSeconds) : 1.0f;
    for (int side = 0; side < 2; ++side) {
        float open = 0.0f, fist = 0.0f;
        switch (input.handPose[static_cast<std::size_t>(side)]) {
        case RagdollHandPose3D::Relaxed: break;
        case RagdollHandPose3D::Open: open = 1.0f; break;
        case RagdollHandPose3D::Fist: fist = 1.0f; break;
        case RagdollHandPose3D::Automatic: {
            // O cotovelo: o angulo entre o braco (ombro ao cotovelo) e o
            // antebraco (cotovelo ao pulso) - a torcao do antebraco nao conta.
            const std::string prefix = side == 0 ? "Left" : "Right";
            const int upper = linkIndex(character.profile, prefix + "UpperArm");
            const int fore = linkIndex(character.profile, prefix + "Forearm");
            const int hand = linkIndex(character.profile, prefix + "Hand");
            if (upper >= 0 && fore >= 0 && hand >= 0) {
                const auto anchor = [&](int link) {
                    const auto& definition = character.profile.links[static_cast<std::size_t>(link)];
                    return input.linkPositions[static_cast<std::size_t>(link)]
                        + linkRotation(link).rotate(definition.inboundJoint.anchorModelPosition - definition.modelPosition);
                };
                const Vec3 shoulder = anchor(upper), elbow = anchor(fore), wrist = anchor(hand);
                const Vec3 a = elbow - shoulder, b = wrist - elbow;
                const float lengths = a.length() * b.length();
                const float bend = lengths > 1e-6f ? std::acos(std::clamp(dot(a, b) / lengths, -1.0f, 1.0f)) : 0.0f;
                open = AutomaticOpen
                    * (1.0f - smooth01((bend - AutomaticStraightBend) / (AutomaticBentBend - AutomaticStraightBend)));
            }
            break;
        }
        }
        const std::size_t s = static_cast<std::size_t>(side);
        m_open[s] += (open - m_open[s]) * blend;
        m_fist[s] += (fist - m_fist[s]) * blend;
    }

    for (std::size_t k = 0; k < visuals.size(); ++k) {
        const RagdollVisualBone3D& visual = visuals[k];
        const std::size_t side = visual.side == 0 ? 0 : 1;
        if (visual.kind == RagdollVisualBone3D::Kind::Finger) {
            // A flexao (em volta de -y do osso) e, na base, o giro ate o dedo de
            // referencia (em volta do z): a relaxada e a exportada; aberta e
            // fechada pelas tabelas.
            const int finger = std::clamp(visual.finger, 0, 4);
            const int joint = std::clamp(visual.joint - 1, 0, 2);
            const float o = m_open[side], f = m_fist[side];
            const float flexion = visual.flexion + (OpenHand.flexion[finger][joint] * Degrees - visual.flexion) * o
                + (FistHand.flexion[finger][joint] * Degrees - visual.flexion) * f;
            Quaternion rotation = Quaternion::fromAxisAngle({ 0.0f, -1.0f, 0.0f }, flexion);
            if (visual.joint == 1) {
                const float close = visual.close + (OpenHand.close[finger] - visual.close) * o
                    + (FistHand.close[finger] - visual.close) * f;
                rotation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, visual.closeAngle * close) * rotation;
            } else {
                // Os ossos da esquerda sao o espelho dos da direita (o z deles
                // aponta para o outro lado da mao, como o closeAngle com o
                // sinal trocado): para dentro e o giro ao contrario.
                const float across = (OpenHand.across[finger][joint - 1] * o + FistHand.across[finger][joint - 1] * f) * Degrees
                    * (visual.side == 0 ? -1.0f : 1.0f);
                if (across != 0.0f) rotation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, across) * rotation;
            }
            m_rotations[k] = rotation.normalized();
        } else if (visual.kind == RagdollVisualBone3D::Kind::Toe && visual.parentBone < character.boneLinkIndices.size()) {
            // Os dedos do pe: a ponta deles (o eixo do osso, que na pose de
            // ligacao fica a tipClear do chao) nunca abaixo disso - o pe
            // inclinando com a ponta apoiada dobra a base dos dedos.
            const std::size_t link = character.boneLinkIndices[visual.parentBone];
            const auto& bind = character.profile.links[link];
            const Quaternion foot = input.linkOrientations[link];
            const Vec3 joint = input.linkPositions[link] + foot.rotate(visual.localBindPosition);
            const Vec3 along = (foot * visual.localBindOrientation).normalized().rotate({ 1.0f, 0.0f, 0.0f });
            const float length = std::max(0.01f, visual.length);
            const Vec3 jointBind = bind.modelPosition + bind.modelOrientation.rotate(visual.localBindPosition);
            const Vec3 tipBind = jointBind
                + (bind.modelOrientation * visual.localBindOrientation).normalized().rotate({ length, 0.0f, 0.0f });
            const float tipClear = tipBind.z + character.profile.standingRootHeightMeters;
            const float height = joint.z - input.groundHeight[side];
            const float pitch = std::asin(std::clamp(along.z, -1.0f, 1.0f));
            const float need = std::asin(std::clamp((tipClear - height) / length, -1.0f, 1.0f));
            const float target = input.toesFollowGround ? std::clamp(need - pitch, 0.0f, ToeBendMaximum) : 0.0f;
            m_toeBend[side] = m_started ? std::max(target, m_toeBend[side] - ToeReleaseRate * dt) : target;
            m_rotations[k] = Quaternion::fromAxisAngle({ 0.0f, -1.0f, 0.0f }, m_toeBend[side]);
        } else {
            m_rotations[k] = visual.restRotation;
        }
    }
    m_started = true;
    return m_rotations;
}

} // namespace MatterEngine
