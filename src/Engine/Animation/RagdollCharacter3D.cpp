#include "Engine/Animation/RagdollCharacter3D.hpp"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace MatterEngine {
namespace {
using Json = nlohmann::json;
Json readJson(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot open character asset: " + path.string());
    return Json::parse(stream);
}
Vec3 vector3(const Json& value) {
    if (!value.is_array() || value.size() != 3)
        throw std::runtime_error("Character vector must have three components");
    Vec3 result { value.at(0).get<float>(), value.at(1).get<float>(), value.at(2).get<float>() };
    if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.z))
        throw std::runtime_error("Non-finite character vector");
    return result;
}
Quaternion quaternion4(const Json& value, const char* what) {
    if (!value.is_array() || value.size() != 4)
        throw std::runtime_error(std::string("Invalid ") + what);
    const Quaternion q { value.at(0).get<float>(), value.at(1).get<float>(),
        value.at(2).get<float>(), value.at(3).get<float>() };
    const float norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(norm) || std::abs(norm-1.0f) > 0.001f)
        throw std::runtime_error(std::string(what) + " is not unit length");
    return q.normalized();
}

// Um movimento rigido (o osso na pose vezes a inversa da ligacao).
struct Rigid {
    Quaternion orientation;
    Vec3 position;
    [[nodiscard]] Vec3 apply(Vec3 point) const { return position + orientation.rotate(point); }
    [[nodiscard]] Rigid then(const Rigid& inner) const {
        return { (orientation * inner.orientation).normalized(), apply(inner.position) };
    }
    [[nodiscard]] Rigid inverse() const {
        const Quaternion back = orientation.conjugate();
        return { back, back.rotate(position * -1.0f) };
    }
};

// A fracao "fraction" do giro q (o mesmo eixo).
Quaternion partialRotation(Quaternion q, float fraction) {
    q = q.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const float half = std::acos(std::clamp(q.w, -1.0f, 1.0f));
    const float sine = std::sin(half);
    if (sine < 1e-6f) return {};
    return Quaternion::fromAxisAngle({ q.x/sine, q.y/sine, q.z/sine }, 2.0f*half*fraction);
}

// A parte do giro q em volta do eixo (balanco e torcao); sem eixo, o giro
// inteiro. (A mesma da modelagem: twistAbout.)
Quaternion twistAbout(Quaternion q, Vec3 axis) {
    const float length = axis.length();
    if (length < 1e-6f) return q;
    const Vec3 a = axis / length;
    const float along = q.x*a.x + q.y*a.y + q.z*a.z;
    const Quaternion twist { a.x*along, a.y*along, a.z*along, q.w };
    const float norm = std::sqrt(twist.x*twist.x + twist.y*twist.y + twist.z*twist.z + twist.w*twist.w);
    if (norm < 1e-6f) return {};
    return { twist.x/norm, twist.y/norm, twist.z/norm, twist.w/norm };
}

// O ponto que o movimento relativo deixa parado (o pivo de verdade): no plano
// perpendicular ao eixo, c = (t + cot(angulo/2) u x t) / 2; ao longo do eixo,
// o do pivo de reserva. Giro quase nulo: o de reserva. (O mesmo da modelagem:
// Skeleton updateDerivedJoints.)
Vec3 rotationPivot(const Rigid& relative, Vec3 fallback) {
    Quaternion q = relative.orientation.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const float half = std::acos(std::clamp(q.w, -1.0f, 1.0f));
    const float sine = std::sin(half);
    if (sine < 0.004f) return fallback;
    const Vec3 axis { q.x/sine, q.y/sine, q.z/sine };
    const Vec3 t = relative.position - axis * dot(relative.position, axis);
    const Vec3 across = (t + cross(axis, t) * (std::cos(half)/sine)) * 0.5f;
    return across + axis * dot(fallback, axis);
}
}

RagdollCharacter3D loadRagdollCharacter3D(const std::string& manifestPath) {
    const std::filesystem::path path(manifestPath);
    const Json manifest = readJson(path);
    if (manifest.at("schema") != "matter-ragdoll-character-1")
        throw std::runtime_error("Unsupported character schema");
    RagdollCharacter3D result;
    result.id = manifest.at("id").get<std::string>();
    result.displayName = manifest.at("displayName").get<std::string>();
    if(manifest.contains("locomotion")) {
        const auto& motion=manifest.at("locomotion");
        result.idleClipId=motion.value("idle", std::string {});
        result.walkClipId=motion.value("walk", std::string {});
        result.walkBackwardClipId=motion.value("walkBackward", std::string {});
        result.sprintClipId=motion.value("sprint", std::string {});
        result.idleToSprintClipId=motion.value("idleToSprint", std::string {});
        result.crouchToSprintClipId=motion.value("crouchToSprint", std::string {});
        result.runForwardArcLeftClipId=motion.value("runForwardArcLeft", std::string {});
        result.runForwardArcRightClipId=motion.value("runForwardArcRight", std::string {});
        result.runBackwardArcRightClipId=motion.value("runBackwardArcRight", std::string {});
        result.sprintBackwardClipId=motion.value("sprintBackward", std::string {});
        result.strafeLeftClipId=motion.value("strafeLeft", std::string {});
        result.strafeRightClipId=motion.value("strafeRight", std::string {});
        result.sprintStrafeLeftClipId=motion.value("sprintStrafeLeft", std::string {});
        result.sprintStrafeRightClipId=motion.value("sprintStrafeRight", std::string {});
        result.jumpStandingClipId=motion.value("jumpStanding", std::string {});
        result.jumpForwardClipId=motion.value("jumpForward", std::string {});
        result.jumpBackwardClipId=motion.value("jumpBackward", std::string {});
        result.jumpLeftClipId=motion.value("jumpLeft", std::string {});
        result.jumpRightClipId=motion.value("jumpRight", std::string {});
        result.jumpPrepClipId=motion.value("jumpPrep", std::string {});
        result.standUpFrontClipId=motion.value("standUpFront", std::string {});
        result.standUpBackClipId=motion.value("standUpBack", std::string {});
        result.turnLeftClipId=motion.value("turnLeft", std::string {});
        result.turnRightClipId=motion.value("turnRight", std::string {});
        if(motion.contains("tackles"))
            for(const auto& id:motion.at("tackles"))result.tackleClipIds.push_back(id.get<std::string>());
    }
    if(manifest.contains("defaultLocomotion"))
        for(const auto& [role, id]:manifest.at("defaultLocomotion").items())
            result.defaultLocomotionClipIds.emplace_back(role, id.get<std::string>());
    if (manifest.contains("thumbnail"))
        result.thumbnailPath = (path.parent_path()/manifest.at("thumbnail").get<std::string>()).string();
    result.flatShaded = manifest.value("flatShaded", true);
    result.facet = std::clamp(manifest.value("facet", result.flatShaded ? 1.0f : 0.0f), 0.0f, 1.0f);
    if (manifest.contains("albedo"))
        result.albedoPath = (path.parent_path()/manifest.at("albedo").get<std::string>()).string();
    result.profile = loadRagdollProfile3D((path.parent_path()
        / manifest.at("physicsProfile").get<std::string>()).string());
    validateRagdollProfileOrThrow3D(result.profile);
    const Json skin = readJson(path.parent_path() / manifest.at("skin").get<std::string>());
    const std::string skinSchema = skin.at("schema").get<std::string>();
    // Versao 2: cada osso com o repouso e a ligacao da malha.
    const bool restAndBind = skinSchema == "matter-ragdoll-skin-2";
    if ((skinSchema != "matter-ragdoll-skin-1" && !restAndBind)
        || skin.at("rigId") != result.profile.id)
        throw std::runtime_error("Skin and physics profile mismatch");
    std::unordered_set<std::string> seen;
    for (const auto& bone : skin.at("bones")) {
        const auto id = bone.at("link").get<std::string>();
        const auto link = std::find_if(result.profile.links.begin(), result.profile.links.end(),
            [&](const auto& candidate) { return candidate.id == id; });
        if (link == result.profile.links.end() || !seen.insert(id).second)
            throw std::runtime_error("Unknown or duplicate skin bone: " + id);
        const auto position = vector3(bone.at(restAndBind ? "restPosition" : "bindPosition"));
        const auto& q = bone.at(restAndBind ? "restOrientation" : "bindOrientation");
        if (!q.is_array() || q.size() != 4) throw std::runtime_error("Invalid bind orientation");
        Quaternion rotation { q.at(0).get<float>(), q.at(1).get<float>(),
            q.at(2).get<float>(), q.at(3).get<float>() };
        const float dot = rotation.x * link->modelOrientation.x
            + rotation.y * link->modelOrientation.y + rotation.z * link->modelOrientation.z
            + rotation.w * link->modelOrientation.w;
        const float norm = rotation.x*rotation.x + rotation.y*rotation.y
            + rotation.z*rotation.z + rotation.w*rotation.w;
        if (!std::isfinite(norm) || std::abs(norm-1.0f) > 0.0001f
            || (position-link->modelPosition).length() > 0.0001f
            || std::abs(std::abs(dot)-1.0f) > 0.0001f)
            throw std::runtime_error("Stale skin bind: rebuild character after changing profile");
        result.boneLinkIndices.push_back(static_cast<std::size_t>(link-result.profile.links.begin()));
        // A ligacao da malha: na versao 1, o proprio repouso.
        const Vec3 bindPosition = restAndBind ? vector3(bone.at("bindPosition")) : position;
        const Quaternion bindOrientation = restAndBind
            ? quaternion4(bone.at("bindOrientation"), "bind orientation") : rotation.normalized();
        result.bindPositions.push_back(bindPosition);
        result.bindOrientations.push_back(bindOrientation);
        result.inverseBindMatrices.push_back(Mat4::rotation(bindOrientation.conjugate())
            * Mat4::translation(bindPosition * -1.0f));
    }
    if (seen.size() != result.profile.links.size())
        throw std::runtime_error("Character skin must cover every physical link");
    // Ossos visuais: depois dos fisicos, cada um com o pai ja lido.
    const auto quaternion = [](const Json& value) {
        if (!value.is_array() || value.size() != 4)
            throw std::runtime_error("Invalid visual bone quaternion");
        const Quaternion q { value.at(0).get<float>(), value.at(1).get<float>(),
            value.at(2).get<float>(), value.at(3).get<float>() };
        const float norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(norm) || std::abs(norm-1.0f) > 0.001f)
            throw std::runtime_error("Visual bone quaternion is not unit length");
        return q.normalized();
    };
    // O repouso de cada osso ja lido: os visuais se prendem ao pai por ele.
    std::vector<std::string> boneNames;
    std::vector<Vec3> restPositions;
    std::vector<Quaternion> restOrientations;
    for (const std::size_t link : result.boneLinkIndices) {
        boneNames.push_back(result.profile.links[link].id);
        restPositions.push_back(result.profile.links[link].modelPosition);
        restOrientations.push_back(result.profile.links[link].modelOrientation);
    }
    if (skin.contains("visualBones")) {
        for (const auto& bone : skin.at("visualBones")) {
            RagdollVisualBone3D visual;
            visual.name = bone.at("name").get<std::string>();
            const auto parentName = bone.at("parent").get<std::string>();
            const auto parent = std::find(boneNames.begin(), boneNames.end(), parentName);
            if (parent == boneNames.end())
                throw std::runtime_error("Visual bone parent must come first: " + visual.name);
            if (std::find(boneNames.begin(), boneNames.end(), visual.name) != boneNames.end())
                throw std::runtime_error("Duplicate skin bone: " + visual.name);
            visual.parentBone = static_cast<std::size_t>(parent - boneNames.begin());
            const Vec3 position = vector3(bone.at(restAndBind ? "restPosition" : "bindPosition"));
            const Quaternion orientation = quaternion(bone.at(restAndBind ? "restOrientation" : "bindOrientation"));
            const Quaternion parentInverse = restOrientations[visual.parentBone].conjugate();
            visual.localBindPosition = parentInverse.rotate(
                position - restPositions[visual.parentBone]);
            visual.localBindOrientation = (parentInverse * orientation).normalized();
            const Vec3 bindPosition = restAndBind ? vector3(bone.at("bindPosition")) : position;
            const Quaternion bindOrientation = restAndBind ? quaternion(bone.at("bindOrientation")) : orientation;
            visual.restRotation = bone.contains("restRotation")
                ? quaternion(bone.at("restRotation")) : Quaternion {};
            // O que ele e (07/10: os dedos da mao com a pose relaxada; os do pe).
            visual.side = visual.name.rfind("Right", 0) == 0 ? 1 : 0;
            const std::string kind = bone.contains("kind") ? bone.at("kind").get<std::string>() : std::string {};
            if (kind == "finger") {
                visual.kind = RagdollVisualBone3D::Kind::Finger;
                static const char* const fingers[] = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
                const std::string finger = bone.at("finger").get<std::string>();
                for (int k = 0; k < 5; ++k)
                    if (finger == fingers[k]) visual.finger = k;
                visual.joint = bone.at("joint").get<int>();
                visual.flexion = bone.value("flexion", 0.0f);
                visual.close = bone.value("close", 0.0f);
                visual.closeAngle = bone.value("closeAngle", 0.0f);
            } else if (kind == "toe") {
                visual.kind = RagdollVisualBone3D::Kind::Toe;
                visual.length = bone.value("length", 0.06f);
            }
            boneNames.push_back(visual.name);
            restPositions.push_back(position);
            restOrientations.push_back(orientation);
            result.bindPositions.push_back(bindPosition);
            result.bindOrientations.push_back(bindOrientation);
            result.inverseBindMatrices.push_back(Mat4::rotation(bindOrientation.conjugate())
                * Mat4::translation(bindPosition * -1.0f));
            result.visualBones.push_back(std::move(visual));
        }
    }
    // Os auxiliares (versao 2): os dois ossos ja lidos, a fracao e o pivo.
    if (restAndBind && skin.contains("helperBones")) {
        // (So ossos fisicos e visuais: um auxiliar nao segue outro.)
        const std::size_t skinned = boneNames.size();
        const auto named = [&](const std::string& name) {
            const auto found = std::find(boneNames.begin(), boneNames.begin() + static_cast<std::ptrdiff_t>(skinned), name);
            if (found == boneNames.begin() + static_cast<std::ptrdiff_t>(skinned))
                throw std::runtime_error("Helper bone must follow a physical or visual bone: " + name);
            return static_cast<std::size_t>(found - boneNames.begin());
        };
        for (const auto& bone : skin.at("helperBones")) {
            RagdollHelperBone3D helper;
            helper.name = bone.at("name").get<std::string>();
            if (std::find(boneNames.begin(), boneNames.end(), helper.name) != boneNames.end())
                throw std::runtime_error("Duplicate skin bone: " + helper.name);
            helper.upperBone = named(bone.at("upper").get<std::string>());
            helper.lowerBone = named(bone.at("lower").get<std::string>());
            helper.fraction = bone.value("fraction", 0.5f);
            if (!std::isfinite(helper.fraction) || helper.fraction < 0.0f || helper.fraction > 1.0f)
                throw std::runtime_error("Helper bone fraction must be in [0, 1]: " + helper.name);
            helper.pivot = vector3(bone.at("pivot"));
            if (bone.contains("axis")) helper.axis = vector3(bone.at("axis"));
            helper.sign = bone.value("sign", 0.0f);
            helper.carry = bone.contains("carry");
            helper.cap = bone.value("capDegrees", 0.0f) * 3.14159265f / 180.0f;
            if (bone.contains("root")) helper.root = vector3(bone.at("root"));
            if (bone.contains("ramp")) {
                const auto& ramp = bone.at("ramp");
                helper.rampFraction = ramp.at("fraction").get<float>();
                helper.rampStart = ramp.at("startDegrees").get<float>() * 3.14159265f / 180.0f;
                helper.rampEnd = ramp.at("endDegrees").get<float>() * 3.14159265f / 180.0f;
                helper.rampAlong = vector3(ramp.at("along"));
                if (ramp.contains("offset")) helper.rampOffset = vector3(ramp.at("offset"));
            }
            boneNames.push_back(helper.name);
            result.helperBones.push_back(std::move(helper));
        }
        // (Os que a clavicula leva podem ser auxiliares de depois dela.)
        std::size_t k = 0;
        for (const auto& bone : skin.at("helperBones")) {
            RagdollHelperBone3D& helper = result.helperBones[k++];
            if (!helper.carry) continue;
            for (const auto& name : bone.at("carry")) {
                const auto found = std::find(boneNames.begin(), boneNames.end(), name.get<std::string>());
                if (found == boneNames.end()) throw std::runtime_error("Clavicle carries an unknown bone: " + name.get<std::string>());
                helper.carried.push_back(static_cast<std::size_t>(found - boneNames.begin()));
            }
        }
    }
    const std::size_t paletteSize = boneNames.size();
    for (const auto& item : skin.at("vertices")) {
        MeshVertex3D vertex;
        vertex.position = vector3(item.at("position"));
        vertex.normal = vector3(item.at("normal"));
        vertex.color = vector3(item.at("color"));
        if (item.contains("uv")) {
            vertex.uv={item.at("uv").at(0).get<float>(),item.at("uv").at(1).get<float>()};
            if (!std::isfinite(vertex.uv.x)||!std::isfinite(vertex.uv.y))
                throw std::runtime_error("Non-finite skin UV");
        }
        vertex.joints = item.at("joints").get<std::array<std::uint32_t,4>>();
        vertex.weights = item.at("weights").get<std::array<float,4>>();
        float total = 0.0f;
        for (std::size_t i=0; i<4; ++i) {
            if (vertex.joints[i] >= paletteSize
                || !std::isfinite(vertex.weights[i]) || vertex.weights[i] < 0.0f)
                throw std::runtime_error("Invalid skin influence");
            total += vertex.weights[i];
        }
        if (std::abs(total-1.0f) > 0.0001f || vertex.normal.length() < 0.9f)
            throw std::runtime_error("Invalid skin weight sum or normal");
        // O sistema de aparencia: todos os vertices com a origem, ou nenhum.
        const bool surfaceData = item.contains("source");
        if (!result.mesh.vertices.empty() && surfaceData == result.surfaceVertices.empty())
            throw std::runtime_error("Skin vertices mix surface data and none");
        if (surfaceData) {
            // (A faixa conferida antes de estreitar; o resto, contra o template, depois.)
            const auto integer = [&](const char* key, std::int64_t highest) {
                const auto& value = item.at(key);
                if (!value.is_number_integer() || value.get<std::int64_t>() < 0 || value.get<std::int64_t>() > highest)
                    throw std::runtime_error(std::string("Invalid skin surface ") + key);
                return value.get<std::int64_t>();
            };
            // (O UV0 dentro do atlas; o UV1 pode passar um pouco do bloco: o triangulo da zona
            // atravessa a borda do retangulo dela.)
            const auto uv = [&](const char* key, bool atlas) {
                const auto& value = item.at(key);
                if (!value.is_array() || value.size() != 2) throw std::runtime_error(std::string("Invalid skin surface ") + key);
                const Vec2 p { value[0].get<float>(), value[1].get<float>() };
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || (atlas && (p.x < 0.0f || p.x > 1.0f || p.y < 0.0f || p.y > 1.0f)))
                    throw std::runtime_error(std::string("Invalid skin surface ") + key);
                return p;
            };
            RagdollCharacter3D::SurfaceVertex extra;
            extra.source = static_cast<std::uint32_t>(integer("source", std::numeric_limits<std::uint32_t>::max() - 1));
            extra.region = static_cast<std::int16_t>(integer("region", std::numeric_limits<std::int16_t>::max()));
            if (item.contains("kitUv")) extra.kitUv = uv("kitUv", true);
            if (item.contains("identity") != item.contains("identityUv")) throw std::runtime_error("Skin surface identity without its UV");
            if (item.contains("identity")) {
                extra.identity = static_cast<std::int8_t>(integer("identity", std::numeric_limits<std::int8_t>::max()));
                extra.identityUv = uv("identityUv", false);
            }
            result.surfaceVertices.push_back(extra);
        }
        result.mesh.vertices.push_back(vertex);
    }
    result.mesh.indices = skin.at("indices").get<std::vector<std::uint32_t>>();
    if (result.mesh.vertices.empty() || result.mesh.indices.empty()
        || result.mesh.indices.size()%3 != 0)
        throw std::runtime_error("Empty or non-triangular character mesh");
    for (auto index : result.mesh.indices)
        if (index >= result.mesh.vertices.size()) throw std::runtime_error("Invalid skin triangle index");
    recomputeBounds(result.mesh);
    // O template de superficie: os triangulos da pele pelos vertices de origem (as
    // costuras do template sao arestas da modelagem).
    if (manifest.contains("surface")) {
        if (result.surfaceVertices.size() != result.mesh.vertices.size())
            throw std::runtime_error("Character surface without per-vertex surface data");
        std::vector<std::array<std::uint32_t, 3>> sources(result.mesh.indices.size() / 3);
        std::uint32_t count = 0;
        for (std::size_t i = 0; i < result.mesh.indices.size(); ++i) {
            const std::uint32_t source = result.surfaceVertices[result.mesh.indices[i]].source;
            sources[i / 3][i % 3] = source;
            count = std::max(count, source + 1);
        }
        std::string message;
        std::optional<Appearance::SurfaceTemplate> surface = Appearance::parseSurfaceTemplate(
            readJson(path.parent_path() / manifest.at("surface").get<std::string>()), sources, count, &message);
        if (!surface) throw std::runtime_error(message);
        // Cada canto da pele igual ao template: a regiao, a zona, o UV0 (presente so onde ha
        // painel) e o UV1 (so onde ha zona). O que a pele leva ao render e o que o template diz.
        const auto same = [](Vec2 a, Vec2 b) { return std::abs(a.x - b.x) <= 1e-6f && std::abs(a.y - b.y) <= 1e-6f; };
        for (std::size_t i = 0; i < result.mesh.indices.size(); ++i) {
            const std::size_t t = i / 3, k = i % 3;
            const RagdollCharacter3D::SurfaceVertex& vertex = result.surfaceVertices[result.mesh.indices[i]];
            const bool panel = surface->trianglePanel[t] >= 0;
            if (vertex.region != surface->triangleRegion[t] || vertex.identity != surface->triangleIdentity[t]
                || (vertex.kitUv.x >= 0.0f) != panel || (panel && !same(vertex.kitUv, surface->triangleKitUv[t][k]))
                || (vertex.identity >= 0 && !same(vertex.identityUv, surface->triangleIdentityUv[t][k])))
                throw std::runtime_error("Skin corner " + std::to_string(i) + " disagrees with the surface template");
        }
        result.surface = std::make_shared<const Appearance::SurfaceTemplate>(std::move(*surface));
    } else if (!result.surfaceVertices.empty()) {
        throw std::runtime_error("Skin surface data without a surface template");
    }
    return result;
}

std::vector<Mat4> buildRagdollSkinMatrices3D(const RagdollCharacter3D& character,
    std::span<const Vec3> positions, std::span<const Quaternion> orientations,
    std::span<const Quaternion> visualRotations) {
    if (positions.size() != character.profile.links.size() || orientations.size() != positions.size())
        throw std::runtime_error("Skin pose does not match physical profile");
    const std::size_t physical = character.boneLinkIndices.size();
    std::vector<Vec3> bonePositions(physical + character.visualBones.size());
    std::vector<Quaternion> boneOrientations(bonePositions.size());
    for (std::size_t bone=0; bone<physical; ++bone) {
        bonePositions[bone] = positions[character.boneLinkIndices[bone]];
        boneOrientations[bone] = orientations[character.boneLinkIndices[bone]];
    }
    // Visuais: o pai (ja calculado) vezes a ligacao local e a rotacao dele (a
    // pedida ou a de repouso).
    for (std::size_t k=0; k<character.visualBones.size(); ++k) {
        const auto& visual = character.visualBones[k];
        const Quaternion parent = boneOrientations[visual.parentBone];
        bonePositions[physical + k] = bonePositions[visual.parentBone]
            + parent.rotate(visual.localBindPosition);
        const Quaternion pose = k < visualRotations.size() ? visualRotations[k] : visual.restRotation;
        boneOrientations[physical + k] = (parent * visual.localBindOrientation * pose).normalized();
    }
    std::vector<Mat4> result;
    result.reserve(bonePositions.size() + character.helperBones.size());
    for (std::size_t bone=0; bone<bonePositions.size(); ++bone) {
        result.push_back(Mat4::translation(bonePositions[bone])
            * Mat4::rotation(boneOrientations[bone])
            * character.inverseBindMatrices[bone]);
    }
    // Os auxiliares: o skin do osso de cima vezes a fracao do giro do de baixo
    // em relacao a ele, em volta do pivo desse giro.
    const auto skin = [&](std::size_t bone) {
        const Rigid pose { boneOrientations[bone], bonePositions[bone] };
        return pose.then(Rigid { character.bindOrientations[bone], character.bindPositions[bone] }.inverse());
    };
    std::vector<std::pair<const RagdollHelperBone3D*, Rigid>> carries;
    for (const RagdollHelperBone3D& helper : character.helperBones) {
        const Rigid upper = skin(helper.upperBone), lower = skin(helper.lowerBone);
        const Rigid relative = upper.inverse().then(lower);
        if (helper.carry) {
            const Vec3 before = helper.axis.lengthSquared() > 0.0f ? helper.axis.normalized() : Vec3 {};
            const Vec3 after = relative.orientation.rotate(before);
            const float rise = std::asin(std::clamp(after.z, -1.0f, 1.0f)) - std::asin(std::clamp(before.z, -1.0f, 1.0f));
            const float turn = rise * helper.fraction;
            const float angle = helper.cap > 0.0f ? helper.cap * std::tanh(turn / helper.cap) : turn;
            const Quaternion part = rise > 0.0f ? Quaternion::fromAxisAngle({ before.y >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f }, angle) : Quaternion {};
            const Rigid moved = upper.then(Rigid { part, helper.pivot - part.rotate(helper.pivot) });
            carries.push_back({ &helper, moved.then(upper.inverse()) });
            result.push_back(Mat4::translation(moved.position) * Mat4::rotation(moved.orientation));
            continue;
        }
        Quaternion twist = twistAbout(relative.orientation, helper.axis);
        // (So um sentido: o outro nao gira - a mesma da modelagem.)
        if (helper.sign != 0.0f) {
            const float along = (twist.x * helper.axis.x + twist.y * helper.axis.y + twist.z * helper.axis.z)
                * (twist.w < 0.0f ? -1.0f : 1.0f);
            if (along * helper.sign < 0.0f) twist = {};
        }
        float fraction = helper.fraction, ramp = 0.0f;
        if (helper.rampEnd > helper.rampStart) {
            const Vec3 before = helper.rampAlong.lengthSquared() > 0.0f ? helper.rampAlong.normalized() : Vec3 {};
            const Vec3 after = relative.orientation.rotate(before);
            const float rise = std::max(0.0f, std::asin(std::clamp(after.z, -1.0f, 1.0f)) - std::asin(std::clamp(before.z, -1.0f, 1.0f)));
            const float t = std::clamp((rise - helper.rampStart) / (helper.rampEnd - helper.rampStart), 0.0f, 1.0f);
            ramp = t * t * (3.0f - 2.0f * t);
            if (helper.rampFraction > 0.0f) fraction += (helper.rampFraction - fraction) * ramp;
        }
        const Quaternion part = partialRotation(twist, fraction);
        const Vec3 pivot = rotationPivot(relative, helper.pivot);
        const Rigid moved = upper.then(Rigid { part, pivot - part.rotate(pivot) });
        result.push_back(Mat4::translation(moved.position + upper.orientation.rotate(helper.rampOffset) * ramp) * Mat4::rotation(moved.orientation));
    }
    // O ombro vai com a clavicula, sem girar o braco (a direcao dele e a do
    // corpo fisico) - a mesma da modelagem.
    for (const auto& [helper, carry] : carries) {
        const Rigid arm = skin(helper->lowerBone);
        const Vec3 root = helper->root ? arm.position + arm.orientation.rotate(*helper->root) : bonePositions[helper->lowerBone];
        const Mat4 by = Mat4::translation(carry.position + carry.orientation.rotate(root) - root);
        for (const std::size_t bone : helper->carried) result[bone] = by * result[bone];
    }
    return result;
}

Vec3 skinVertexPosition3D(const MeshVertex3D& vertex, std::span<const Mat4> palette) {
    Vec3 result;
    for (std::size_t i=0; i<4; ++i) {
        if (vertex.joints[i] >= palette.size()) throw std::runtime_error("Skin palette index out of bounds");
        const auto& m=palette[vertex.joints[i]];
        const auto p=vertex.position;
        result += Vec3 { m.at(0,0)*p.x+m.at(0,1)*p.y+m.at(0,2)*p.z+m.at(0,3),
            m.at(1,0)*p.x+m.at(1,1)*p.y+m.at(1,2)*p.z+m.at(1,3),
            m.at(2,0)*p.x+m.at(2,1)*p.y+m.at(2,2)*p.z+m.at(2,3) } * vertex.weights[i];
    }
    return result;
}
} // namespace MatterEngine
