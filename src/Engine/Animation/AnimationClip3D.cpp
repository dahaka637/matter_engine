#include "Engine/Animation/AnimationClip3D.hpp"

#include "Engine/Physics/RagdollProfile3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <unordered_set>

namespace MatterEngine {
namespace {

using Json = nlohmann::json;

Vec3 readVec3(const Json& value, const std::string& path) {
    if (!value.is_array() || value.size() != 3) {
        throw std::runtime_error(path + " precisa ser um vetor JSON [x,y,z]");
    }
    return { value.at(0).get<float>(), value.at(1).get<float>(),
        value.at(2).get<float>() };
}

Quaternion readQuaternion(const Json& value, const std::string& path) {
    if (!value.is_array() || value.size() != 4) {
        throw std::runtime_error(path
            + " precisa ser um quaternion JSON [x,y,z,w]");
    }
    return { value.at(0).get<float>(), value.at(1).get<float>(),
        value.at(2).get<float>(), value.at(3).get<float>() };
}

Quaternion normalizedLerp(Quaternion from, Quaternion to, float amount) {
    const float dot = from.x * to.x + from.y * to.y + from.z * to.z
        + from.w * to.w;
    if (dot < 0.0f) {
        to = { -to.x, -to.y, -to.z, -to.w };
    }
    return Quaternion {
        from.x + (to.x - from.x) * amount,
        from.y + (to.y - from.y) * amount,
        from.z + (to.z - from.z) * amount,
        from.w + (to.w - from.w) * amount
    }.normalized();
}

bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z);
}

bool finite(Quaternion value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z) && std::isfinite(value.w);
}

Quaternion quaternionExponential(Vec3 rotationVector) {
    const float angle = rotationVector.length();
    if (angle < 0.000001f) return {};
    return Quaternion::fromAxisAngle(rotationVector * (1.0f / angle), angle);
}

Quaternion jointLocalOrientation(const RagdollProfile3D& profile,
    std::size_t linkIndex, Vec3 jointCoordinates) {
    const RagdollLinkDefinition3D& link = profile.links[linkIndex];
    const RagdollLinkDefinition3D& parent =
        profile.links[static_cast<std::size_t>(link.parentIndex)];
    const Quaternion parentFrame =
        (parent.modelOrientation.conjugate()
            * link.inboundJoint.frameModelOrientation).normalized();
    const Quaternion childFrame =
        (link.modelOrientation.conjugate()
            * link.inboundJoint.frameModelOrientation).normalized();
    float* coordinates[] = {
        &jointCoordinates.x, &jointCoordinates.y, &jointCoordinates.z
    };
    for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
        const RagdollAxisDefinition3D& axis =
            link.inboundJoint.axes[axisIndex];
        *coordinates[axisIndex] = axis.enabled
            ? std::clamp(*coordinates[axisIndex],
                axis.minimumRadians, axis.maximumRadians)
            : 0.0f;
    }
    return (parentFrame * quaternionExponential(jointCoordinates)
        * childFrame.conjugate()).normalized();
}

} // namespace

AnimationClip3D loadAnimationClip3D(std::string_view filePath) {
    std::ifstream input { std::string(filePath) };
    if (!input) {
        throw std::runtime_error("Não foi possível abrir clipe de animação: "
            + std::string(filePath));
    }

    Json root;
    try {
        input >> root;
    } catch (const Json::exception& error) {
        throw std::runtime_error("JSON de animação inválido em "
            + std::string(filePath) + ": " + error.what());
    }

    AnimationClip3D clip;
    try {
        const std::string schema = root.at("schema").get<std::string>();
        if (schema != "matter-ragdoll-animation-1") {
            throw std::runtime_error("Schema de animação não suportado: "
                + schema);
        }
        clip.id = root.at("id").get<std::string>();
        clip.displayName = root.at("displayName").get<std::string>();
        clip.targetRigId = root.at("targetRigId").get<std::string>();
        clip.sourceAssetPath = root.value("sourceAssetPath", std::string {});
        clip.durationSeconds = root.at("durationSeconds").get<float>();
        clip.sourceSampleRateHz =
            root.at("sourceSampleRateHz").get<float>();
        clip.loops = root.value("loops", true);
        if(root.contains("sourceRootDisplacementMeters")) {
            clip.sourceRootDisplacementMeters=readVec3(root.at("sourceRootDisplacementMeters"),"sourceRootDisplacementMeters");
            if(!finite(clip.sourceRootDisplacementMeters))throw std::runtime_error("Invalid source root displacement");
        }
        clip.nominalSpeedMetersPerSecond =
            root.value("nominalSpeedMetersPerSecond", 0.0f);
        clip.travelDirectionRadians =
            root.value("travelDirectionDegrees", 0.0f)
                * (3.14159265358979323846f / 180.0f);
        if (!std::isfinite(clip.travelDirectionRadians)) {
            throw std::runtime_error("Invalid travel direction");
        }
        if (root.contains("contacts")) {
            const Json& contacts = root.at("contacts");
            const auto loadContact = [&](std::string_view name,
                                         AnimationScalarTrack3D& target) {
                if (!contacts.contains(name)) return;
                for (const Json& source : contacts.at(name)) {
                    target.keyframes.push_back({
                        source.at("timeSeconds").get<float>(),
                        source.at("value").get<float>()
                    });
                }
            };
            loadContact("leftFoot", clip.leftFootContact);
            loadContact("rightFoot", clip.rightFootContact);
        }
        if (root.contains("retargetReport")) {
            const Json& report = root.at("retargetReport");
            clip.retargetReport.available = true;
            clip.retargetReport.passed = report.at("passed").get<bool>();
            clip.retargetReport.directionRmsDegrees =
                report.at("directionRmsDegrees").get<float>();
            clip.retargetReport.directionMaxDegrees =
                report.at("directionMaxDegrees").get<float>();
            clip.retargetReport.limbDirectionMaxDegrees =
                report.at("limbDirectionMaxDegrees").get<float>();
            clip.retargetReport.maximumJointStepDegrees =
                report.at("maximumJointStepDegrees").get<float>();
            clip.retargetReport.limitHitCount =
                report.at("limitHitCount").get<std::uint32_t>();
        }

        const Json& tracks = root.at("tracks");
        if (!tracks.is_array()) {
            throw std::runtime_error("tracks precisa ser um array");
        }
        clip.tracks.reserve(tracks.size());
        for (std::size_t trackIndex = 0;
                trackIndex < tracks.size(); ++trackIndex) {
            const Json& sourceTrack = tracks.at(trackIndex);
            const std::string trackPath = "tracks["
                + std::to_string(trackIndex) + "]";
            AnimationTrack3D track;
            track.targetLinkId =
                sourceTrack.at("targetLinkId").get<std::string>();
            const std::string space =
                sourceTrack.at("space").get<std::string>();
            if (space == "root") {
                track.space = AnimationTrackSpace3D::Root;
            } else if (space == "joint") {
                track.space = AnimationTrackSpace3D::Joint;
            } else {
                throw std::runtime_error(trackPath
                    + ".space desconhecido: " + space);
            }
            const Json& keyframes = sourceTrack.at("keyframes");
            if (!keyframes.is_array()) {
                throw std::runtime_error(trackPath
                    + ".keyframes precisa ser um array");
            }
            track.keyframes.reserve(keyframes.size());
            for (std::size_t keyIndex = 0;
                    keyIndex < keyframes.size(); ++keyIndex) {
                const Json& sourceKey = keyframes.at(keyIndex);
                const std::string keyPath = trackPath + ".keyframes["
                    + std::to_string(keyIndex) + "]";
                AnimationKeyframe3D keyframe;
                keyframe.timeSeconds =
                    sourceKey.at("timeSeconds").get<float>();
                if (track.space == AnimationTrackSpace3D::Root) {
                    keyframe.translationOffsetMeters = readVec3(
                        sourceKey.at("rootTranslationOffsetMeters"),
                        keyPath + ".rootTranslationOffsetMeters");
                    keyframe.rotationDelta = readQuaternion(
                        sourceKey.at("rootRotationDelta"),
                        keyPath + ".rootRotationDelta");
                } else {
                    keyframe.jointPositionRadians = readVec3(
                        sourceKey.at("jointPositionRadians"),
                        keyPath + ".jointPositionRadians");
                }
                track.keyframes.push_back(keyframe);
            }
            clip.tracks.push_back(std::move(track));
        }
    } catch (const Json::exception& error) {
        throw std::runtime_error("Estrutura de animação inválida em "
            + std::string(filePath) + ": " + error.what());
    }

    const auto issues = validateAnimationClip3D(clip);
    if (!issues.empty()) {
        throw std::runtime_error("Clipe de animação inválido em "
            + std::string(filePath) + " (" + issues.front().path + ": "
            + issues.front().message + ")");
    }
    return clip;
}

AnimationTransformSample3D sampleAnimationTrack3D(
    const AnimationTrack3D& track, float timeSeconds,
    float clipDurationSeconds, bool loop) {
    if (track.keyframes.empty()) return {};
    if (track.keyframes.size() == 1) {
        return { track.keyframes.front().translationOffsetMeters,
            track.keyframes.front().rotationDelta.normalized(),
            track.keyframes.front().jointPositionRadians };
    }

    float sampleTime = std::max(0.0f, timeSeconds);
    if (loop && clipDurationSeconds > 0.0f) {
        sampleTime = std::fmod(sampleTime, clipDurationSeconds);
    } else {
        sampleTime = std::min(sampleTime, clipDurationSeconds);
    }

    const auto next = std::upper_bound(track.keyframes.begin(),
        track.keyframes.end(), sampleTime,
        [](float time, const AnimationKeyframe3D& keyframe) {
            return time < keyframe.timeSeconds;
        });
    if (next == track.keyframes.begin()) {
        return { next->translationOffsetMeters,
            next->rotationDelta.normalized(), next->jointPositionRadians };
    }
    if (next == track.keyframes.end()) {
        const AnimationKeyframe3D& last = track.keyframes.back();
        return { last.translationOffsetMeters,
            last.rotationDelta.normalized(), last.jointPositionRadians };
    }

    const AnimationKeyframe3D& to = *next;
    const AnimationKeyframe3D& from = *(next - 1);
    const float interval = to.timeSeconds - from.timeSeconds;
    const float amount = interval > 0.000001f
        ? std::clamp((sampleTime - from.timeSeconds) / interval, 0.0f, 1.0f)
        : 0.0f;
    return {
        from.translationOffsetMeters
            + (to.translationOffsetMeters - from.translationOffsetMeters)
                * amount,
        normalizedLerp(from.rotationDelta, to.rotationDelta, amount),
        from.jointPositionRadians
            + (to.jointPositionRadians - from.jointPositionRadians) * amount
    };
}

float sampleAnimationScalarTrack3D(const AnimationScalarTrack3D& track,
    float timeSeconds, float clipDurationSeconds, bool loop) {
    if (track.keyframes.empty()) return 0.0f;
    if (track.keyframes.size() == 1) return track.keyframes.front().value;
    float sampleTime = std::max(0.0f, timeSeconds);
    if (loop && clipDurationSeconds > 0.0f) {
        sampleTime = std::fmod(sampleTime, clipDurationSeconds);
    } else {
        sampleTime = std::min(sampleTime, clipDurationSeconds);
    }
    const auto next = std::upper_bound(track.keyframes.begin(),
        track.keyframes.end(), sampleTime,
        [](float time, const AnimationScalarKeyframe3D& keyframe) {
            return time < keyframe.timeSeconds;
        });
    if (next == track.keyframes.begin()) return next->value;
    if (next == track.keyframes.end()) return track.keyframes.back().value;
    const AnimationScalarKeyframe3D& to = *next;
    const AnimationScalarKeyframe3D& from = *(next - 1);
    const float interval = to.timeSeconds - from.timeSeconds;
    const float amount = interval > 0.000001f
        ? std::clamp((sampleTime - from.timeSeconds) / interval, 0.0f, 1.0f)
        : 0.0f;
    return from.value + (to.value - from.value) * amount;
}

std::vector<AnimationClipValidationIssue3D> validateAnimationClip3D(
    const AnimationClip3D& clip) {
    std::vector<AnimationClipValidationIssue3D> issues;
    if (clip.id.empty()) issues.push_back({ "id", "ID vazio" });
    if (clip.displayName.empty()) {
        issues.push_back({ "displayName", "Nome de exibição vazio" });
    }
    if (clip.targetRigId.empty()) {
        issues.push_back({ "targetRigId", "Rig alvo vazio" });
    }
    if (!std::isfinite(clip.durationSeconds)
        || clip.durationSeconds <= 0.0f) {
        issues.push_back(
            { "durationSeconds", "Duração deve ser positiva e finita" });
    }
    if (!std::isfinite(clip.sourceSampleRateHz)
        || clip.sourceSampleRateHz < 0.0f) {
        issues.push_back({ "sourceSampleRateHz",
            "Taxa de amostragem deve ser finita e não negativa" });
    }
    if (!std::isfinite(clip.nominalSpeedMetersPerSecond)
        || clip.nominalSpeedMetersPerSecond < 0.0f) {
        issues.push_back({ "nominalSpeedMetersPerSecond",
            "Velocidade nominal deve ser finita e não negativa" });
    }
    if (clip.tracks.empty()) {
        issues.push_back({ "tracks", "Clipe não possui canais" });
    }

    std::unordered_set<std::string> targets;
    for (std::size_t trackIndex = 0;
            trackIndex < clip.tracks.size(); ++trackIndex) {
        const AnimationTrack3D& track = clip.tracks[trackIndex];
        const std::string path = "tracks[" + std::to_string(trackIndex) + "]";
        if (track.targetLinkId.empty()) {
            issues.push_back({ path + ".targetLinkId", "Link alvo vazio" });
        } else if (!targets.insert(track.targetLinkId).second) {
            issues.push_back({ path + ".targetLinkId",
                "Mais de um canal aponta para o mesmo link" });
        }
        if (track.keyframes.empty()) {
            issues.push_back({ path + ".keyframes", "Canal sem quadros" });
            continue;
        }
        float previousTime = -1.0f;
        for (std::size_t keyIndex = 0;
                keyIndex < track.keyframes.size(); ++keyIndex) {
            const AnimationKeyframe3D& keyframe = track.keyframes[keyIndex];
            const std::string keyPath = path + ".keyframes["
                + std::to_string(keyIndex) + "]";
            if (!std::isfinite(keyframe.timeSeconds)
                || keyframe.timeSeconds < 0.0f
                || keyframe.timeSeconds > clip.durationSeconds) {
                issues.push_back({ keyPath + ".timeSeconds",
                    "Tempo fora da duração do clipe" });
            }
            if (keyframe.timeSeconds <= previousTime) {
                issues.push_back({ keyPath + ".timeSeconds",
                    "Quadros precisam estar em ordem estritamente crescente" });
            }
            if (!finite(keyframe.translationOffsetMeters)) {
                issues.push_back({ keyPath + ".translationOffsetMeters",
                    "Translação contém valor não finito" });
            }
            if (!finite(keyframe.rotationDelta)) {
                issues.push_back({ keyPath + ".rotationDelta",
                    "Rotação contém valor não finito" });
            }
            if (!finite(keyframe.jointPositionRadians)) {
                issues.push_back({ keyPath + ".jointPositionRadians",
                    "Coordenadas da junta contêm valor não finito" });
            }
            previousTime = keyframe.timeSeconds;
        }
    }
    const auto validateScalar = [&](const AnimationScalarTrack3D& track,
                                    std::string_view name) {
        float previousTime = -1.0f;
        for (std::size_t index = 0; index < track.keyframes.size(); ++index) {
            const AnimationScalarKeyframe3D& keyframe = track.keyframes[index];
            const std::string path = std::string(name) + "["
                + std::to_string(index) + "]";
            if (!std::isfinite(keyframe.timeSeconds)
                || keyframe.timeSeconds < 0.0f
                || keyframe.timeSeconds > clip.durationSeconds
                || keyframe.timeSeconds <= previousTime) {
                issues.push_back({ path + ".timeSeconds",
                    "Tempo de contato inválido" });
            }
            if (!std::isfinite(keyframe.value)
                || keyframe.value < 0.0f || keyframe.value > 1.0f) {
                issues.push_back({ path + ".value",
                    "Contato deve estar no intervalo 0..1" });
            }
            previousTime = keyframe.timeSeconds;
        }
    };
    validateScalar(clip.leftFootContact, "contacts.leftFoot");
    validateScalar(clip.rightFootContact, "contacts.rightFoot");
    return issues;
}

std::vector<AnimationClipValidationIssue3D>
validateAnimationClipForRagdoll3D(
    const AnimationClip3D& clip, const RagdollProfile3D& profile) {
    std::vector<AnimationClipValidationIssue3D> issues;
    if (clip.targetRigId != profile.id) {
        issues.push_back({ "targetRigId",
            "Clipe e perfil de ragdoll não correspondem" });
        return issues;
    }
    if (!clip.retargetReport.available || !clip.retargetReport.passed) {
        issues.push_back({ "retargetReport",
            "Clipe não passou pelos gates do importador" });
    }
    const AnimationTrack3D* referenceTrack =
        clip.tracks.empty() ? nullptr : &clip.tracks.front();
    std::unordered_set<std::string> coveredLinks;
    for (std::size_t trackIndex = 0;
            trackIndex < clip.tracks.size(); ++trackIndex) {
        const AnimationTrack3D& track = clip.tracks[trackIndex];
        const auto link = std::find_if(profile.links.begin(),
            profile.links.end(), [&](const RagdollLinkDefinition3D& candidate) {
                return candidate.id == track.targetLinkId;
            });
        const std::string path = "tracks[" + std::to_string(trackIndex) + "]";
        if (link == profile.links.end()) {
            issues.push_back({ path + ".targetLinkId",
                "Link não existe no perfil alvo" });
            continue;
        }
        coveredLinks.insert(link->id);
        const bool root = link->parentIndex < 0;
        if ((root && track.space != AnimationTrackSpace3D::Root)
            || (!root && track.space != AnimationTrackSpace3D::Joint)) {
            issues.push_back({ path + ".space",
                root ? "Link raiz exige canal root"
                     : "Link articulado exige canal joint" });
            continue;
        }
        if (root) continue;
        for (std::size_t keyIndex = 0;
                keyIndex < track.keyframes.size(); ++keyIndex) {
            const Vec3 coordinates =
                track.keyframes[keyIndex].jointPositionRadians;
            const float values[] = {
                coordinates.x, coordinates.y, coordinates.z
            };
            for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
                const RagdollAxisDefinition3D& axis =
                    link->inboundJoint.axes[axisIndex];
                if ((!axis.enabled && std::abs(values[axisIndex]) > 0.0001f)
                    || (axis.enabled
                        && (values[axisIndex] < axis.minimumRadians - 0.0001f
                            || values[axisIndex]
                                > axis.maximumRadians + 0.0001f))) {
                    issues.push_back({ path + ".keyframes["
                            + std::to_string(keyIndex)
                            + "].jointPositionRadians["
                            + std::to_string(axisIndex) + "]",
                        "Alvo fora dos limites físicos da junta" });
                }
            }
        }
        if (referenceTrack != nullptr
            && track.keyframes.size() != referenceTrack->keyframes.size()) {
            issues.push_back({ path + ".keyframes",
                "Todos os canais precisam ter a mesma amostragem" });
        } else if (referenceTrack != nullptr) {
            for (std::size_t keyIndex = 0;
                    keyIndex < track.keyframes.size(); ++keyIndex) {
                if (std::abs(track.keyframes[keyIndex].timeSeconds
                        - referenceTrack->keyframes[keyIndex].timeSeconds)
                    > 0.00001f) {
                    issues.push_back({ path + ".keyframes["
                            + std::to_string(keyIndex) + "].timeSeconds",
                        "Tempo difere dos outros canais" });
                    break;
                }
            }
        }
    }
    for (const RagdollLinkDefinition3D& link : profile.links) {
        if (!coveredLinks.contains(link.id)) {
            issues.push_back({ "tracks",
                "Clipe não cobre o link obrigatório " + link.id });
        }
    }
    return issues;
}

const AnimationTrack3D* findAnimationTrack3D(
    const AnimationClip3D& clip, std::string_view targetLinkId) {
    const auto found = std::find_if(clip.tracks.begin(), clip.tracks.end(),
        [&](const AnimationTrack3D& track) {
            return track.targetLinkId == targetLinkId;
        });
    return found != clip.tracks.end() ? &*found : nullptr;
}

RagdollAnimationPose3D sampleRagdollAnimationPose3D(
    const RagdollProfile3D& profile, const AnimationClip3D* clip,
    float timeSeconds, bool loop) {
    RagdollAnimationPose3D pose;
    pose.linkPositions.resize(profile.links.size());
    pose.linkOrientations.resize(profile.links.size());
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        const RagdollLinkDefinition3D& link = profile.links[index];
        AnimationTransformSample3D sample;
        if (clip != nullptr) {
            if (const AnimationTrack3D* track =
                    findAnimationTrack3D(*clip, link.id)) {
                // Quem chama decide se este playback deve repetir. O
                // metadado do clipe continua sendo o padrão seguro para
                // controladores, mas o visualizador pode deliberadamente
                // inspecionar uma ação one-shot em loop.
                sample = sampleAnimationTrack3D(*track, timeSeconds,
                    clip->durationSeconds, loop);
            }
        }

        if (link.parentIndex < 0) {
            pose.linkPositions[index] =
                link.modelPosition + sample.translationOffsetMeters;
            pose.linkOrientations[index] =
                (link.modelOrientation * sample.rotationDelta).normalized();
            continue;
        }

        const std::size_t parentIndex =
            static_cast<std::size_t>(link.parentIndex);
        const RagdollLinkDefinition3D& parent = profile.links[parentIndex];
        const Quaternion localOrientation = jointLocalOrientation(
            profile, index, sample.jointPositionRadians);
        pose.linkOrientations[index] =
            (pose.linkOrientations[parentIndex] * localOrientation)
                .normalized();

        const Vec3 parentAnchorLocal =
            parent.modelOrientation.conjugate().rotate(
                link.inboundJoint.anchorModelPosition
                    - parent.modelPosition);
        const Vec3 childAnchorLocal =
            link.modelOrientation.conjugate().rotate(
                link.inboundJoint.anchorModelPosition
                    - link.modelPosition);
        const Vec3 worldAnchor = pose.linkPositions[parentIndex]
            + pose.linkOrientations[parentIndex].rotate(parentAnchorLocal);
        pose.linkPositions[index] = worldAnchor
            - pose.linkOrientations[index].rotate(childAnchorLocal);
    }
    return pose;
}

} // namespace MatterEngine
