#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <utility>

namespace MatterEngine::Modeling {

std::string_view sideName(Side side) {
    switch (side) {
    case Side::Left: return "esquerda";
    case Side::Right: return "direita";
    case Side::Center: break;
    }
    return "centro";
}

std::string_view jointKindName(JointKind kind) {
    switch (kind) {
    case JointKind::Finger: return "dedo";
    case JointKind::Toe: return "dedos do pe";
    case JointKind::Tip: return "ponta";
    case JointKind::Helper: return "auxiliar";
    case JointKind::Body: break;
    }
    return "corpo";
}

int Skeleton::find(std::string_view name) const {
    for (std::size_t i = 0; i < joints.size(); ++i)
        if (joints[i].name == name) return static_cast<int>(i);
    return -1;
}

std::vector<int> Skeleton::children(int joint) const {
    std::vector<int> result;
    for (std::size_t i = 0; i < joints.size(); ++i)
        if (joints[i].parent == joint) result.push_back(static_cast<int>(i));
    return result;
}

Box3 Skeleton::bounds(float padding) const {
    Box3 box;
    for (const Joint& joint : joints) box.include(joint.bind.position, padding);
    return box;
}

float Skeleton::height() const {
    float top = groundHeight;
    for (const Joint& joint : joints) top = std::max(top, joint.bind.position.z);
    return top - groundHeight;
}

float Skeleton::boneLength(int joint) const {
    if (joint < 0 || joint >= static_cast<int>(joints.size())) return 0.0f;
    const int parent = joints[static_cast<std::size_t>(joint)].parent;
    if (parent < 0) return 0.0f;
    return distance(joints[static_cast<std::size_t>(joint)].bind.position,
        joints[static_cast<std::size_t>(parent)].bind.position);
}

std::vector<std::string> Skeleton::validate() const {
    std::vector<std::string> problems;
    std::set<std::string> names;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const Joint& joint = joints[i];
        if (!names.insert(joint.name).second) problems.push_back("nome repetido: " + joint.name);
        if (joint.parent >= static_cast<int>(i)) problems.push_back("pai depois do filho: " + joint.name);
    }
    return problems;
}

SkeletonPose bindPose(const Skeleton& skeleton) {
    SkeletonPose pose;
    pose.joints.reserve(skeleton.joints.size());
    for (const Joint& joint : skeleton.joints) pose.joints.push_back(joint.bind);
    return pose;
}

int addHelperJoint(Skeleton& skeleton, std::string name, int from, int to, float fraction, Vec3 axis, float sign) {
    const int count = static_cast<int>(skeleton.joints.size());
    if (from < 0 || to < 0 || from >= count || to >= count) return -1;
    const Joint& lower = skeleton.joints[static_cast<std::size_t>(to)];
    Joint helper;
    helper.name = std::move(name);
    helper.parent = from;
    helper.bind = lower.bind;
    helper.kind = JointKind::Helper;
    helper.side = lower.side;
    helper.radius = lower.radius;
    helper.derivedFrom = from;
    helper.derivedTo = to;
    helper.derivedFraction = fraction;
    helper.derivedAxis = axis;
    helper.derivedSign = sign;
    skeleton.joints.push_back(std::move(helper));
    return count;
}

namespace {

// O ponto que o giro relativo deixa parado (o pivo de verdade): no plano
// perpendicular ao eixo, c = (t + cot(angulo/2) u x t) / 2; ao longo do eixo,
// o da ligacao. Quando o osso do modelo nao esta no pivo do movimento (a
// ancora fisica em outro lugar), a auxiliar gira junto com o movimento real.
// Giro quase nulo: o pivo da ligacao.
Vec3 rotationPivot(const Transform& relative, Vec3 fallback) {
    Quaternion q = relative.orientation.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const float half = std::acos(std::clamp(q.w, -1.0f, 1.0f));
    const float sine = std::sin(half);
    if (sine < 0.004f) return fallback;
    const Vec3 axis { q.x / sine, q.y / sine, q.z / sine };
    const Vec3 t = relative.position - axis * dot(relative.position, axis);
    const Vec3 across = (t + cross(axis, t) * (std::cos(half) / sine)) * 0.5f;
    return across + axis * dot(fallback, axis);
}

} // namespace

void updateDerivedJoints(const Skeleton& skeleton, SkeletonPose& pose, bool clavicleCarry) {
    if (pose.joints.size() < skeleton.joints.size()) return;
    // As claviculas pela pose de antes (o giro do braco em relacao ao peito);
    // o braco vai junto no fim.
    std::vector<std::pair<std::size_t, Transform>> carries;
    for (std::size_t h = 0; h < skeleton.joints.size(); ++h) {
        const Joint& helper = skeleton.joints[h];
        if (helper.kind != JointKind::Helper || !helper.derivedCarry || helper.derivedFrom < 0 || helper.derivedTo < 0) continue;
        const auto from = static_cast<std::size_t>(helper.derivedFrom), to = static_cast<std::size_t>(helper.derivedTo);
        const Transform upper = pose.joints[from] * skeleton.joints[from].bind.inverse();
        if (!clavicleCarry) {
            pose.joints[h] = upper * helper.bind;
            continue;
        }
        const Transform lower = pose.joints[to] * skeleton.joints[to].bind.inverse();
        const Quaternion part = clavicleTurn(elevationTurn((upper.inverse() * lower).orientation, helper.derivedAxis),
            helper.derivedFraction, helper.derivedCap);
        const Transform partial { helper.derivedPivot - part.rotate(helper.derivedPivot), part };
        pose.joints[h] = upper * partial * helper.bind;
        carries.push_back({ h, upper * partial * upper.inverse() });
    }
    for (std::size_t h = 0; h < skeleton.joints.size(); ++h) {
        const Joint& helper = skeleton.joints[h];
        if (helper.kind != JointKind::Helper || helper.derivedCarry || helper.derivedFrom < 0 || helper.derivedTo < 0) continue;
        const auto from = static_cast<std::size_t>(helper.derivedFrom), to = static_cast<std::size_t>(helper.derivedTo);
        // O skin das duas (pose vezes a inversa da ligacao) e o giro de "to"
        // em relacao a "from", em volta do pivo desse giro (a junta "to" nas
        // dobras de teste; a ancora fisica nas animacoes do jogo).
        const Transform upper = pose.joints[from] * skeleton.joints[from].bind.inverse();
        const Transform lower = pose.joints[to] * skeleton.joints[to].bind.inverse();
        const Transform relative = upper.inverse() * lower;
        float fraction = helper.derivedFraction, ramp = 0.0f;
        if (helper.derivedRampEnd > helper.derivedRampStart) {
            const Quaternion lift = elevationTurn(relative.orientation, helper.derivedRampAlong);
            const float rise = 2.0f * std::acos(std::clamp(std::abs(lift.w), 0.0f, 1.0f));
            const float t = std::clamp((rise - helper.derivedRampStart) / (helper.derivedRampEnd - helper.derivedRampStart), 0.0f, 1.0f);
            ramp = t * t * (3.0f - 2.0f * t);
            if (helper.derivedRampFraction > 0.0f) fraction += (helper.derivedRampFraction - fraction) * ramp;
        }
        const Quaternion part = partialRotation(twistAbout(relative.orientation, helper.derivedAxis, helper.derivedSign), fraction);
        const Vec3 pivot = rotationPivot(relative, skeleton.joints[to].bind.position);
        const Transform partial { pivot - part.rotate(pivot), part };
        pose.joints[h] = upper * partial * helper.bind;
        pose.joints[h].position = pose.joints[h].position + upper.orientation.rotate(helper.derivedRampOffset) * ramp;
    }
    // A clavicula leva o ombro, sem girar o braco (25/10): a direcao dele vem
    // pronta em relacao ao peito (a animacao, a dobra da tela, o corpo fisico).
    // Girando o braco junto, como filho, ela somava o giro dela ao pedido e o
    // braco erguido passava da vertical por cima da cabeca.
    for (const auto& [h, carry] : carries) {
        const int arm = skeleton.joints[h].derivedTo;
        const Vec3 root = pose.joints[static_cast<std::size_t>(arm)].position;
        const Vec3 shift = carry.apply(root) - root;
        for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
            const Joint& joint = skeleton.joints[j];
            if (j == h) continue;
            // (Uma auxiliar que nao gira com o braco fica com o peito: o volume da axila.)
            if (joint.kind == JointKind::Helper && joint.derivedFraction <= 0.0f && joint.derivedRampFraction <= 0.0f) continue;
            int up = joint.kind == JointKind::Helper ? joint.derivedTo : static_cast<int>(j);
            while (up >= 0 && up != arm) up = skeleton.joints[static_cast<std::size_t>(up)].parent;
            if (up == arm) pose.joints[j].position = pose.joints[j].position + shift;
        }
    }
}

Vec3 fingerFlexionAxis(const Quaternion& frame, bool thumb, bool left) {
    const Vec3 flexion = -axisY(frame);
    return thumb ? safeNormalized(flexion + axisZ(frame) * (left ? -1.0f : 1.0f), flexion) : flexion;
}

Vec3 jointHingeAxis(const Skeleton& skeleton, const SkeletonPose& pose, int joint) {
    if (joint < 0 || joint >= static_cast<int>(skeleton.joints.size()) || pose.joints.size() != skeleton.joints.size())
        return { 0.0f, 1.0f, 0.0f };
    const Joint& target = skeleton.joints[static_cast<std::size_t>(joint)];
    const Quaternion frame = pose.joints[static_cast<std::size_t>(joint)].orientation;
    for (const char* finger : { "Thumb", "Index", "Middle", "Ring", "Pinky" })
        if (target.name.find(finger) != std::string::npos)
            return fingerFlexionAxis(frame, std::string_view(finger) == "Thumb", target.side == Side::Left);
    int child = -1;
    for (std::size_t k = 0; k < skeleton.joints.size() && child < 0; ++k)
        if (skeleton.joints[k].parent == joint && skeleton.joints[k].kind == JointKind::Body) child = static_cast<int>(k);
    if (target.parent < 0 || child < 0) return safeNormalized(-axisY(frame), { 0.0f, 1.0f, 0.0f });
    const Vec3 at = pose.joints[static_cast<std::size_t>(joint)].position;
    const Vec3 upper = at - pose.joints[static_cast<std::size_t>(target.parent)].position;
    const Vec3 lower = pose.joints[static_cast<std::size_t>(child)].position - at;
    const Vec3 normal = cross(upper, lower);
    // Dobrado mais de ~12 graus: o plano do membro.
    if (normal.length() > 0.2f * upper.length() * lower.length()) return safeNormalized(normal, { 0.0f, 1.0f, 0.0f });
    const Vec3 forward { 1.0f, 0.0f, 0.0f };
    const bool backward = target.name.find("Shin") != std::string::npos;
    return safeNormalized(backward ? cross(forward, lower) : cross(lower, forward), { 0.0f, 1.0f, 0.0f });
}

SkeletonPose bendPose(const Skeleton& skeleton, std::span<const JointBend> bends) {
    return bendPose(skeleton, bindPose(skeleton), bends);
}

SkeletonPose bendPose(const Skeleton& skeleton, const SkeletonPose& start, std::span<const JointBend> bends) {
    SkeletonPose pose = start;
    if (pose.joints.size() != skeleton.joints.size()) pose = bindPose(skeleton);
    const SkeletonPose initial = pose;
    std::vector<JointBend> ordered(bends.begin(), bends.end());
    // De cima para baixo: o pai gira antes (a junta filha ja vem no lugar).
    std::stable_sort(ordered.begin(), ordered.end(), [](const JointBend& a, const JointBend& b) { return a.joint < b.joint; });
    for (const JointBend& bend : ordered) {
        if (bend.joint < 0 || bend.joint >= static_cast<int>(skeleton.joints.size())) continue;
        Vec3 axis = bend.axis;
        // O eixo preso ao pai: o giro dele desde a partida (as dobras de
        // cima ja aplicadas) leva o eixo junto.
        if (const int parent = skeleton.joints[static_cast<std::size_t>(bend.joint)].parent; bend.followParent && parent >= 0) {
            const auto p = static_cast<std::size_t>(parent);
            axis = (pose.joints[p].orientation * initial.joints[p].orientation.conjugate()).normalized().rotate(axis);
        }
        const Quaternion rotation = Quaternion::fromAxisAngle(axis, bend.radians);
        const Vec3 pivot = pose.joints[static_cast<std::size_t>(bend.joint)].position;
        std::vector<bool> inside(skeleton.joints.size(), false);
        for (std::size_t k = 0; k < skeleton.joints.size(); ++k) {
            const int parent = skeleton.joints[k].parent;
            inside[k] = static_cast<int>(k) == bend.joint || (parent >= 0 && inside[static_cast<std::size_t>(parent)]);
            if (!inside[k] || skeleton.joints[k].kind == JointKind::Helper) continue;
            Transform& frame = pose.joints[k];
            frame.position = pivot + rotation.rotate(frame.position - pivot);
            frame.orientation = (rotation * frame.orientation).normalized();
        }
    }
    updateDerivedJoints(skeleton, pose);
    return pose;
}

std::vector<int> fingerJoints(const Skeleton& skeleton, Side side, bool thumb) {
    std::vector<int> joints;
    const std::string prefix = side == Side::Left ? "Left" : "Right";
    std::vector<const char*> fingers { "Index", "Middle", "Ring", "Pinky" };
    if (thumb) fingers.push_back("Thumb");
    for (const char* finger : fingers)
        for (int k = 1; k <= 3; ++k)
            if (const int joint = skeleton.find(prefix + finger + std::to_string(k)); joint >= 0) joints.push_back(joint);
    return joints;
}

std::vector<JointBend> blendJointBends(const Skeleton& skeleton, const SkeletonPose& from, const SkeletonPose& to,
    std::span<const int> joints, float fraction) {
    std::vector<JointBend> bends;
    if (fraction == 0.0f || from.joints.size() != skeleton.joints.size() || to.joints.size() != skeleton.joints.size()) return bends;
    for (const int joint : joints) {
        if (joint < 0 || joint >= static_cast<int>(skeleton.joints.size())) continue;
        const int parent = skeleton.joints[static_cast<std::size_t>(joint)].parent;
        if (parent < 0) continue;
        const auto j = static_cast<std::size_t>(joint), p = static_cast<std::size_t>(parent);
        const Quaternion start = (from.joints[p].orientation.conjugate() * from.joints[j].orientation).normalized();
        const Quaternion target = (to.joints[p].orientation.conjugate() * to.joints[j].orientation).normalized();
        // O giro da partida ao alvo no referencial da propria junta (o menor).
        Quaternion delta = (start.conjugate() * target).normalized();
        if (delta.w < 0.0f) delta = Quaternion { -delta.x, -delta.y, -delta.z, -delta.w };
        const float angle = 2.0f * std::acos(std::clamp(delta.w, -1.0f, 1.0f));
        if (angle < 1e-5f) continue;
        const Vec3 local = safeNormalized({ delta.x, delta.y, delta.z }, { 0.0f, 1.0f, 0.0f });
        bends.push_back({ joint, from.joints[j].orientation.rotate(local), fraction * angle, true });
    }
    return bends;
}

std::vector<Transform> skinningTransforms(const Skeleton& skeleton, const SkeletonPose& pose) {
    std::vector<Transform> result(skeleton.joints.size());
    for (std::size_t i = 0; i < skeleton.joints.size(); ++i) {
        const Transform& posed = i < pose.joints.size() ? pose.joints[i] : skeleton.joints[i].bind;
        result[i] = posed * skeleton.joints[i].bind.inverse();
    }
    return result;
}

std::string mirroredName(std::string_view name) {
    constexpr std::array<std::pair<std::string_view, std::string_view>, 4> Swaps { {
        { "Left", "Right" }, { "left", "right" }, { "Esquerd", "Direit" }, { "esquerd", "direit" } } };
    std::string result(name);
    for (const auto& [a, b] : Swaps) {
        const std::size_t at = result.find(a);
        if (at != std::string::npos) return result.replace(at, a.size(), b);
        const std::size_t bt = result.find(b);
        if (bt != std::string::npos) return result.replace(bt, b.size(), a);
    }
    return result;
}

int mirroredJoint(const Skeleton& skeleton, int joint) {
    if (joint < 0 || joint >= static_cast<int>(skeleton.joints.size())) return joint;
    const int other = skeleton.find(mirroredName(skeleton.joints[static_cast<std::size_t>(joint)].name));
    return other >= 0 ? other : joint;
}

} // namespace MatterEngine::Modeling
