#include "Engine/RL/Body/RLBody3D.hpp"

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/RL/Body/RLFootSplit3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <set>
#include <stdexcept>

namespace MatterEngine {
namespace {

const char* axisName(std::size_t axis) {
    switch (axis) {
        case 0: return "twist";
        case 1: return "swing1";
        default: return "swing2";
    }
}

std::string readText(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("corpo de RL: arquivo ausente: " + path.string());
    return { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
}

std::uint64_t fnv1a(std::string_view text, std::uint64_t hash = 14695981039346656037ull) {
    for (const char c : text) {
        hash ^= static_cast<unsigned char>(c);
        hash *= 1099511628211ull;
    }
    return hash;
}

std::uint32_t findLink(const RagdollProfile3D& profile, std::string_view id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return static_cast<std::uint32_t>(i);
    throw std::runtime_error("corpo de RL: o perfil nao tem o link " + std::string(id));
}

// As juntas e a inclinacao da pelve num quadro de clipe (o hash do corpo
// passa a incluir o clipe).
void readClipPose(const RagdollProfile3D& profile, const std::filesystem::path& clipPath, float time,
    std::vector<Vec3>& coordinates, Quaternion& rootRotation, std::uint64_t& hash) {
    const AnimationClip3D clip = loadAnimationClip3D(clipPath.string());
    hash = fnv1a(readText(clipPath), hash);
    coordinates.assign(profile.links.size(), {});
    rootRotation = {};
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[i].id);
        if (!track) continue;
        const AnimationTransformSample3D sample = sampleAnimationTrack3D(*track, time, clip.durationSeconds, true);
        if (i == 0) rootRotation = sample.rotationDelta.normalized();
        else coordinates[i] = sample.jointPositionRadians;
    }
}

// Uma pose guardada como dado no arquivo do corpo (o corpo v2: o jogo nao
// carrega clipe nenhum para o Corpo): "coordinates" (o vetor de rotacao de cada
// junta, pelo id do link; os que faltam ficam em zero) e "rootRotation" (a
// inclinacao da pelve, [x, y, z, w]).
void readDataPose(const RagdollProfile3D& profile, const nlohmann::json& source, std::vector<Vec3>& coordinates,
    Quaternion& rootRotation) {
    coordinates.assign(profile.links.size(), {});
    rootRotation = {};
    if (source.contains("rootRotation")) {
        const auto& q = source.at("rootRotation");
        rootRotation = Quaternion { q.at(0).get<float>(), q.at(1).get<float>(), q.at(2).get<float>(), q.at(3).get<float>() }
            .normalized();
    }
    for (const auto& [id, value] : source.at("coordinates").items()) {
        const std::uint32_t link = findLink(profile, id);
        coordinates[link] = { value.at(0).get<float>(), value.at(1).get<float>(), value.at(2).get<float>() };
    }
}

// A pose de uma entrada do arquivo: de um quadro de clipe ("clip") ou dos dados.
void readPose(const RagdollProfile3D& profile, const std::filesystem::path& directory, const nlohmann::json& entry,
    std::vector<Vec3>& coordinates, Quaternion& rootRotation, std::uint64_t& hash) {
    if (entry.contains("coordinates")) readDataPose(profile, entry, coordinates, rootRotation);
    else readClipPose(profile, (directory / entry.at("clip").get<std::string>()).lexically_normal(),
        entry.value("timeSeconds", 0.0f), coordinates, rootRotation, hash);
}

// Uma postura das juntas de um quadro: os eixos atuados presos no curso e as
// medidas com os pes no chao.
RLPosture3D makePosture(const RLBody3D& body, std::string id, std::string displayName, std::vector<Vec3> coordinates,
    Quaternion rootRotation) {
    const RagdollProfile3D& profile = body.profile;
    RLPosture3D posture;
    posture.id = std::move(id);
    posture.displayName = std::move(displayName);
    for (const RLBodyDof3D& dof : body.dofs) {
        Vec3& c = coordinates[dof.linkIndex];
        float& value = dof.axis == RagdollAxis3D::Twist ? c.x : dof.axis == RagdollAxis3D::Swing1 ? c.y : c.z;
        value = std::clamp(value, dof.minimumRadians, dof.maximumRadians);
        posture.dofRadians.push_back(value);
    }
    // Os eixos travados ficam na pose base em qualquer postura (o motor os segura la).
    for (const RLBodyDof3D& dof : body.lockedDofs) {
        Vec3& c = coordinates[dof.linkIndex];
        (dof.axis == RagdollAxis3D::Twist ? c.x : dof.axis == RagdollAxis3D::Swing1 ? c.y : c.z) = dof.baseRadians;
    }
    posture.jointCoordinates = std::move(coordinates);
    posture.rootRotation = rootRotation;
    const Quaternion root = profile.links.front().modelOrientation * rootRotation;
    const RLBodyPose3D pose = rlBodyPose3D(profile, {}, root, posture.jointCoordinates);
    const float lowest = rlBodyLowestFootPoint3D(body, pose);
    posture.standingPelvisHeight = pose.positions.front().z - lowest;
    posture.standingHeadHeight = pose.positions[body.head].z - lowest;
    // O rumo pelo eixo da frente da pelve (como o ambiente le) e o "para cima"
    // de cada parte nele.
    const Quaternion pelvisFrame = (pose.orientations.front() * profile.links.front().modelOrientation.conjugate())
        .normalized();
    const Vec3 forward = pelvisFrame.rotate({ 1.0f, 0.0f, 0.0f });
    const Quaternion headingInverse = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -std::atan2(forward.y, forward.x));
    const auto up = [&](std::uint32_t link) {
        const Quaternion frame = pose.orientations[link] * profile.links[link].modelOrientation.conjugate();
        return headingInverse.rotate(frame.rotate({ 0.0f, 0.0f, 1.0f })).normalized();
    };
    posture.pelvisUp = up(0);
    posture.chestUp = up(body.upperChest);
    posture.headUp = up(body.head);
    if (!(posture.standingPelvisHeight > 0.3f) || !(posture.standingHeadHeight > posture.standingPelvisHeight))
        throw std::runtime_error("corpo de RL: a postura " + posture.id + " nao fica de pe");
    return posture;
}

} // namespace

float RLBodyDof3D::target(float action) const {
    action = std::clamp(action, -RLActionLimit, RLActionLimit);
    return std::clamp(baseRadians + actionScale * action, minimumRadians, maximumRadians);
}

Quaternion rlQuaternionExp3D(Vec3 rotationVector) {
    const float angle = rotationVector.length();
    if (angle < 1e-8f) return {};
    return Quaternion::fromAxisAngle(rotationVector * (1.0f / angle), angle);
}

Vec3 rlQuaternionLog3D(Quaternion rotation) {
    if (rotation.w < 0.0f) rotation = { -rotation.x, -rotation.y, -rotation.z, -rotation.w };
    const Vec3 axis { rotation.x, rotation.y, rotation.z };
    const float sine = axis.length();
    if (sine < 1e-8f) return axis * 2.0f;
    return axis * (2.0f * std::atan2(sine, rotation.w) / sine);
}

RLBodyPose3D rlBodyPose3D(const RagdollProfile3D& profile, Vec3 rootPosition, Quaternion rootOrientation,
    std::span<const Vec3> jointCoordinates) {
    RLBodyPose3D pose;
    pose.positions.resize(profile.links.size());
    pose.orientations.resize(profile.links.size());
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        const RagdollLinkDefinition3D& link = profile.links[index];
        if (link.parentIndex < 0) {
            pose.positions[index] = rootPosition;
            pose.orientations[index] = rootOrientation.normalized();
            continue;
        }
        const std::size_t parentIndex = static_cast<std::size_t>(link.parentIndex);
        const RagdollLinkDefinition3D& parent = profile.links[parentIndex];
        Vec3 coordinates = index < jointCoordinates.size() ? jointCoordinates[index] : Vec3 {};
        float* axes[] = { &coordinates.x, &coordinates.y, &coordinates.z };
        for (std::size_t a = 0; a < 3; ++a) {
            const RagdollAxisDefinition3D& axis = link.inboundJoint.axes[a];
            *axes[a] = axis.enabled ? std::clamp(*axes[a], axis.minimumRadians, axis.maximumRadians) : 0.0f;
        }
        const Quaternion parentFrame = (parent.modelOrientation.conjugate() * link.inboundJoint.frameModelOrientation)
            .normalized();
        const Quaternion childFrame = (link.modelOrientation.conjugate() * link.inboundJoint.frameModelOrientation)
            .normalized();
        pose.orientations[index] = (pose.orientations[parentIndex] * parentFrame * rlQuaternionExp3D(coordinates)
            * childFrame.conjugate()).normalized();
        const Vec3 parentAnchor =
            parent.modelOrientation.conjugate().rotate(link.inboundJoint.anchorModelPosition - parent.modelPosition);
        const Vec3 childAnchor =
            link.modelOrientation.conjugate().rotate(link.inboundJoint.anchorModelPosition - link.modelPosition);
        const Vec3 anchor = pose.positions[parentIndex] + pose.orientations[parentIndex].rotate(parentAnchor);
        pose.positions[index] = anchor - pose.orientations[index].rotate(childAnchor);
    }
    return pose;
}

void rlBodyLinkStates3D(const RagdollProfile3D& profile, Vec3 rootPosition, Quaternion rootOrientation,
    Vec3 rootLinearVelocity, Vec3 rootAngularVelocity, std::span<const Vec3> jointCoordinates,
    std::span<const Vec3> jointVelocities, std::vector<PhysicsBodyState3D>& out) {
    constexpr float H = 1e-3f;
    // As duas poses com a raiz na origem: a diferenca de posicoes longe dela
    // (os mundos de treino ficam a centenas de metros) perderia a precisao do
    // float (a 375 m, ~0,03 m/s de erro na velocidade).
    const RLBodyPose3D now = rlBodyPose3D(profile, {}, rootOrientation, jointCoordinates);
    std::vector<Vec3> ahead(jointCoordinates.begin(), jointCoordinates.end());
    for (std::size_t i = 0; i < ahead.size() && i < jointVelocities.size(); ++i) ahead[i] += jointVelocities[i] * H;
    const RLBodyPose3D next =
        rlBodyPose3D(profile, {}, rlQuaternionExp3D(rootAngularVelocity * H) * rootOrientation, ahead);
    out.resize(profile.links.size());
    for (std::size_t i = 0; i < out.size(); ++i) {
        PhysicsBodyState3D& state = out[i];
        state.position = rootPosition + now.positions[i];
        state.orientation = now.orientations[i];
        state.linearVelocity = rootLinearVelocity + (next.positions[i] - now.positions[i]) * (1.0f / H);
        state.angularVelocity =
            rlQuaternionLog3D((next.orientations[i] * now.orientations[i].conjugate()).normalized()) * (1.0f / H);
        state.sleeping = false;
        state.frozen = false;
    }
}

float rlBodyLowestFootPoint3D(const RLBody3D& body, const RLBodyPose3D& pose) {
    float lowest = std::numeric_limits<float>::max();
    std::vector<std::uint32_t> parts;
    for (const auto& side : body.footParts) parts.insert(parts.end(), side.begin(), side.end());
    if (parts.empty()) parts = { body.leftFoot, body.rightFoot };
    for (const std::uint32_t foot : parts) {
        const RagdollCapsuleDefinition3D& collider = body.profile.links[foot].collider;
        const Vec3 center = pose.positions[foot] + pose.orientations[foot].rotate(collider.localPosition);
        const Quaternion orientation = pose.orientations[foot] * collider.localOrientation;
        if (ragdollColliderHasBox3D(collider)) {
            // Os cantos da caixa, ou os vertices da casca da chuteira.
            for (const Vec3& corner : ragdollColliderCorners3D(collider))
                lowest = std::min(lowest, (center + orientation.rotate(corner)).z);
        } else {
            const float half = ragdollCapsuleHalfSegment3D(collider);
            for (const float side : { -1.0f, 1.0f })
                lowest = std::min(lowest, (center + orientation.rotate({ side * half, 0.0f, 0.0f })).z
                    - collider.radiusMeters);
        }
    }
    return lowest;
}

Vec3 rlJointCoordinates3D(const RagdollProfile3D& profile, std::size_t linkIndex, Quaternion parentOrientation,
    Quaternion childOrientation) {
    const RagdollLinkDefinition3D& link = profile.links[linkIndex];
    const RagdollLinkDefinition3D& parent = profile.links[static_cast<std::size_t>(link.parentIndex)];
    const Quaternion parentFrame =
        (parent.modelOrientation.conjugate() * link.inboundJoint.frameModelOrientation).normalized();
    const Quaternion childFrame =
        (link.modelOrientation.conjugate() * link.inboundJoint.frameModelOrientation).normalized();
    return rlQuaternionLog3D(
        (parentFrame.conjugate() * parentOrientation.conjugate() * childOrientation * childFrame).normalized());
}

RLBody3D loadRLBody3D(const std::filesystem::path& definitionPath) {
    const std::string text = readText(definitionPath);
    const nlohmann::json document = nlohmann::json::parse(text);
    if (document.value("schema", std::string()) != "matter-rl-body-1")
        throw std::runtime_error("corpo de RL: schema desconhecido em " + definitionPath.string());

    RLBody3D body;
    body.id = document.at("id").get<std::string>();
    body.definitionPath = definitionPath;
    const std::filesystem::path directory = definitionPath.parent_path();
    body.profilePath = (directory / document.at("profile").get<std::string>()).lexically_normal();
    body.profile = loadRagdollProfile3D(body.profilePath.string());
    // A junta dos dedos (o corpo v2): o pe dividido em calcanhar e ponta.
    if (document.contains("toes")) {
        const nlohmann::json& toes = document["toes"];
        RLToeSettings3D settings;
        constexpr float Radians = 3.14159265358979f / 180.0f;
        settings.jointFromHeel = toes.value("jointFromHeel", settings.jointFromHeel);
        settings.jointHeight = toes.value("jointHeightMeters", settings.jointHeight);
        settings.massShare = toes.value("massShare", settings.massShare);
        settings.minimumRadians = toes.value("minimumDegrees", settings.minimumRadians / Radians) * Radians;
        settings.maximumRadians = toes.value("maximumDegrees", settings.maximumRadians / Radians) * Radians;
        settings.stiffness = toes.value("stiffness", settings.stiffness);
        settings.damping = toes.value("damping", settings.damping);
        settings.maximumTorque = toes.value("maximumTorque", settings.maximumTorque);
        splitRagdollToes3D(body.profile, settings);
    }
    const RagdollProfile3D& profile = body.profile;
    body.hash = fnv1a(text, fnv1a(readText(body.profilePath)));

    body.leftFoot = findLink(profile, "LeftFoot");
    body.rightFoot = findLink(profile, "RightFoot");
    body.head = findLink(profile, "Head");
    body.leftHand = findLink(profile, "LeftHand");
    body.rightHand = findLink(profile, "RightHand");
    body.upperChest = findLink(profile, "UpperChest");
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftToe") body.leftToe = static_cast<int>(i);
        if (profile.links[i].id == "RightToe") body.rightToe = static_cast<int>(i);
    }
    body.footParts[0] = { body.leftFoot };
    body.footParts[1] = { body.rightFoot };
    if (body.leftToe >= 0) body.footParts[0].push_back(static_cast<std::uint32_t>(body.leftToe));
    if (body.rightToe >= 0) body.footParts[1].push_back(static_cast<std::uint32_t>(body.rightToe));
    body.supportLink.assign(profile.links.size(), 0);
    for (const auto& id : document.at("supportLinks")) body.supportLink[findLink(profile, id.get<std::string>())] = 1;

    if (document.contains("solver")) {
        body.solverPositionIterations = document["solver"].value("positionIterations", 0u);
        body.solverVelocityIterations = document["solver"].value("velocityIterations", 0u);
    }

    // A pose base: as coordenadas das juntas e a inclinacao da pelve num
    // quadro do clipe.
    body.baseJointCoordinates.assign(profile.links.size(), {});
    if (document.contains("basePose"))
        readPose(profile, directory, document["basePose"], body.baseJointCoordinates, body.baseRootRotation, body.hash);

    const float defaultActionScale = document.value("actionScale", 0.25f);
    const nlohmann::json actionScales = document.value("actionScales", nlohmann::json::object());
    float stiffnessGain = 1.0f, dampingGain = 1.0f;
    if (document.contains("gains")) {
        stiffnessGain = document["gains"].value("stiffnessScale", 1.0f);
        dampingGain = document["gains"].value("dampingScale", 1.0f);
    }
    const nlohmann::json torque = document.value("torque", nlohmann::json::object());
    std::set<std::string> locked;
    for (const auto& name : document.value("lockedAxes", nlohmann::json::array())) locked.insert(name.get<std::string>());
    for (std::size_t i = 1; i < profile.links.size(); ++i) {
        const RagdollLinkDefinition3D& link = profile.links[i];
        for (std::size_t a = 0; a < 3; ++a) {
            const RagdollAxisDefinition3D& axis = link.inboundJoint.axes[a];
            if (!axis.enabled) continue;
            RLBodyDof3D dof;
            dof.name = link.id + "." + axisName(a);
            dof.linkIndex = static_cast<std::uint32_t>(i);
            dof.axis = static_cast<RagdollAxis3D>(a);
            dof.minimumRadians = axis.minimumRadians;
            dof.maximumRadians = axis.maximumRadians;
            const float base = a == 0 ? body.baseJointCoordinates[i].x
                : a == 1 ? body.baseJointCoordinates[i].y : body.baseJointCoordinates[i].z;
            dof.baseRadians = std::clamp(base, axis.minimumRadians, axis.maximumRadians);
            dof.actionScale = actionScales.contains(link.id) ? actionScales[link.id].get<float>() : defaultActionScale;
            // Rigidez/amortecimento do perfil vezes o ganho; o modo ativo do
            // backend multiplica a escala por 5 (rigidez) e 2 (amortecimento).
            dof.stiffness = axis.stiffness * stiffnessGain;
            dof.damping = axis.damping * dampingGain;
            dof.stiffnessScale = std::clamp(stiffnessGain / 5.0f, 0.0f, 2.0f);
            dof.dampingScale = std::clamp(dampingGain / 2.0f, 0.0f, 2.5f);
            float cap = axis.maximumTorque;
            if (torque.contains(link.id)) {
                const auto& values = torque[link.id];
                if (a < values.size() && values[a].get<float>() > 0.0f) cap = values[a].get<float>();
            }
            dof.maximumTorque = std::min(cap, axis.maximumTorque * 3.0f);
            dof.maximumTorqueScale = axis.maximumTorque > 0.0f ? dof.maximumTorque / axis.maximumTorque : 0.0f;
            if (locked.erase(dof.name) > 0) body.lockedDofs.push_back(std::move(dof));
            else body.dofs.push_back(std::move(dof));
        }
    }
    if (!locked.empty()) throw std::runtime_error("corpo de RL: eixo travado que nao existe: " + *locked.begin());
    // Clipes guardam as juntas ja limitadas; a base volta aos limites tambem
    // no vetor por link (para a cinematica).
    std::vector<const RLBodyDof3D*> allDofs;
    for (const RLBodyDof3D& dof : body.dofs) allDofs.push_back(&dof);
    for (const RLBodyDof3D& dof : body.lockedDofs) allDofs.push_back(&dof);
    for (const RLBodyDof3D* pointer : allDofs) {
        const RLBodyDof3D& dof = *pointer;
        Vec3& coordinates = body.baseJointCoordinates[dof.linkIndex];
        if (dof.axis == RagdollAxis3D::Twist) coordinates.x = dof.baseRadians;
        else if (dof.axis == RagdollAxis3D::Swing1) coordinates.y = dof.baseRadians;
        else coordinates.z = dof.baseRadians;
    }

    // A base e a primeira postura; as medidas dela ficam tambem nos campos da
    // base (o contrato da observacao e da acao parte dela).
    body.postures.push_back(makePosture(body, "natural", "Natural", body.baseJointCoordinates, body.baseRootRotation));
    const RLPosture3D& base = body.postures.front();
    body.standingPelvisHeight = base.standingPelvisHeight;
    body.standingHeadHeight = base.standingHeadHeight;
    body.basePelvisUp = base.pelvisUp;
    body.baseChestUp = base.chestUp;
    body.baseHeadUp = base.headUp;
    for (const nlohmann::json& entry : document.value("postures", nlohmann::json::array())) {
        const std::string id = entry.at("id").get<std::string>();
        if (findRLPosture3D(body, id) >= 0) throw std::runtime_error("corpo de RL: postura repetida: " + id);
        std::vector<Vec3> coordinates;
        Quaternion rootRotation;
        readPose(profile, directory, entry, coordinates, rootRotation, body.hash);
        body.postures.push_back(makePosture(body, id, entry.value("displayName", id), std::move(coordinates), rootRotation));
    }
    return body;
}

int findRLPosture3D(const RLBody3D& body, std::string_view id) {
    for (std::size_t i = 0; i < body.postures.size(); ++i)
        if (body.postures[i].id == id) return static_cast<int>(i);
    return -1;
}

} // namespace MatterEngine
