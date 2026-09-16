#include "Engine/Physics/RagdollProfile3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numbers>
#include <stdexcept>
#include <unordered_set>

namespace MatterEngine {
namespace {

using Json = nlohmann::json;

constexpr float DegreesToRadians = std::numbers::pi_v<float> / 180.0f;

Vec3 readVec3(const Json& value, std::string_view path) {
    if (!value.is_array() || value.size() != 3) {
        throw std::runtime_error(std::string(path)
            + " precisa ser um vetor JSON com 3 números");
    }
    return { value.at(0).get<float>(), value.at(1).get<float>(),
        value.at(2).get<float>() };
}

Quaternion readQuaternion(const Json& value, std::string_view path) {
    if (!value.is_array() || value.size() != 4) {
        throw std::runtime_error(std::string(path)
            + " precisa ser um quaternion JSON [x,y,z,w]");
    }
    return Quaternion { value.at(0).get<float>(), value.at(1).get<float>(),
        value.at(2).get<float>(), value.at(3).get<float>() }.normalized();
}

bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z);
}

bool finite(Quaternion value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z) && std::isfinite(value.w);
}

std::size_t axisIndex(std::string_view name) {
    if (name == "twist") return static_cast<std::size_t>(
        RagdollAxis3D::Twist);
    if (name == "swing1") return static_cast<std::size_t>(
        RagdollAxis3D::Swing1);
    if (name == "swing2") return static_cast<std::size_t>(
        RagdollAxis3D::Swing2);
    throw std::runtime_error("Eixo de ragdoll desconhecido: "
        + std::string(name));
}

void addIssue(std::vector<RagdollProfileValidationIssue3D>& issues,
    std::string path, std::string message) {
    issues.push_back({ std::move(path), std::move(message) });
}

} // namespace

RagdollProfile3D loadRagdollProfile3D(std::string_view filePath) {
    std::ifstream input { std::string(filePath) };
    if (!input) {
        throw std::runtime_error("Não foi possível abrir perfil de ragdoll: "
            + std::string(filePath));
    }

    Json root;
    try {
        input >> root;
    } catch (const Json::exception& error) {
        throw std::runtime_error("JSON inválido em "
            + std::string(filePath) + ": " + error.what());
    }

    RagdollProfile3D profile;
    try {
        profile.id = root.at("id").get<std::string>();
        profile.totalMassKg = root.at("totalMassKg").get<float>();
        profile.uniformRadiusMeters =
            root.at("uniformRadiusMeters").get<float>();
        profile.standingRootHeightMeters =
            root.at("standingRootHeightMeters").get<float>();

        const Json& links = root.at("links");
        if (!links.is_array()) {
            throw std::runtime_error("links precisa ser um array");
        }
        profile.links.reserve(links.size());
        for (std::size_t index = 0; index < links.size(); ++index) {
            const Json& source = links.at(index);
            const std::string path = "links[" + std::to_string(index) + "]";
            RagdollLinkDefinition3D link;
            link.id = source.at("id").get<std::string>();
            link.parentIndex = source.at("parent").get<int>();
            link.modelPosition = readVec3(source.at("position"),
                path + ".position");
            link.modelOrientation = source.contains("orientation")
                ? readQuaternion(source.at("orientation"),
                    path + ".orientation")
                : Quaternion {};
            link.massFraction = source.at("massFraction").get<float>();
            link.centerOfMassLocal = source.contains("centerOfMass")
                ? readVec3(source.at("centerOfMass"),
                    path + ".centerOfMass")
                : Vec3 {};

            const bool hasBox = source.contains("box");
            const Json& shape = hasBox
                ? source.at("box") : source.at("capsule");
            link.collider.shape = hasBox
                ? RagdollColliderShape3D::Box
                : RagdollColliderShape3D::Capsule;
            link.collider.localPosition = shape.contains("position")
                ? readVec3(shape.at("position"),
                    path + (hasBox ? ".box.position" : ".capsule.position"))
                : Vec3 {};
            link.collider.localOrientation =
                readQuaternion(shape.at("orientation"),
                    path + (hasBox
                        ? ".box.orientation" : ".capsule.orientation"));
            if (hasBox) {
                link.collider.boxHalfExtents = readVec3(
                    shape.at("halfExtents"), path + ".box.halfExtents");
            } else {
                link.collider.lengthMeters =
                    shape.at("length").get<float>();
                link.collider.radiusMeters = shape.contains("radius")
                    ? shape.at("radius").get<float>()
                    : profile.uniformRadiusMeters;
            }
            link.collider.materialId = shape.value("material", "default");
            link.collider.contactSensor =
                shape.value("contactSensor", false);

            if (link.parentIndex >= 0) {
                const Json& joint = source.at("joint");
                const std::string type = joint.at("type").get<std::string>();
                if (type == "revolute") {
                    link.inboundJoint.type = RagdollJointType3D::Revolute;
                } else if (type == "spherical") {
                    link.inboundJoint.type = RagdollJointType3D::Spherical;
                } else {
                    throw std::runtime_error(path
                        + ".joint.type desconhecido: " + type);
                }
                link.inboundJoint.anchorModelPosition = readVec3(
                    joint.at("anchor"), path + ".joint.anchor");
                link.inboundJoint.frameModelOrientation = readQuaternion(
                    joint.at("frameOrientation"),
                    path + ".joint.frameOrientation");
                const Json& axes = joint.at("axes");
                if (!axes.is_array()) {
                    throw std::runtime_error(path
                        + ".joint.axes precisa ser um array");
                }
                for (const Json& axis : axes) {
                    RagdollAxisDefinition3D& destination =
                        link.inboundJoint.axes[axisIndex(
                            axis.at("axis").get<std::string>())];
                    destination.enabled = true;
                    destination.minimumRadians =
                        axis.at("minimumDegrees").get<float>()
                        * DegreesToRadians;
                    destination.maximumRadians =
                        axis.at("maximumDegrees").get<float>()
                        * DegreesToRadians;
                    destination.stiffness =
                        axis.at("stiffness").get<float>();
                    destination.damping = axis.at("damping").get<float>();
                    destination.maximumTorque =
                        axis.at("maximumTorque").get<float>();
                }
            }
            profile.links.push_back(std::move(link));
        }
    } catch (const Json::exception& error) {
        throw std::runtime_error("Perfil de ragdoll incompleto em "
            + std::string(filePath) + ": " + error.what());
    }

    validateRagdollProfileOrThrow3D(profile);
    return profile;
}

std::vector<RagdollProfileValidationIssue3D> validateRagdollProfile3D(
    const RagdollProfile3D& profile) {
    std::vector<RagdollProfileValidationIssue3D> issues;
    if (profile.id.empty()) addIssue(issues, "id", "não pode ser vazio");
    if (!std::isfinite(profile.totalMassKg) || profile.totalMassKg <= 0.0f) {
        addIssue(issues, "totalMassKg", "precisa ser positivo e finito");
    }
    if (!std::isfinite(profile.uniformRadiusMeters)
        || profile.uniformRadiusMeters < 0.015f) {
        addIssue(issues, "uniformRadiusMeters",
            "precisa ter ao menos 1,5 cm");
    }
    if (!std::isfinite(profile.standingRootHeightMeters)
        || profile.standingRootHeightMeters <= 0.0f) {
        addIssue(issues, "standingRootHeightMeters",
            "precisa ser positivo e finito");
    }
    if (profile.links.empty()) {
        addIssue(issues, "links", "precisa conter ao menos a raiz");
        return issues;
    }

    std::unordered_set<std::string> ids;
    float massSum = 0.0f;
    std::size_t rootCount = 0;
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        const RagdollLinkDefinition3D& link = profile.links[index];
        const std::string path = "links[" + std::to_string(index) + "]";
        if (link.id.empty() || !ids.insert(link.id).second) {
            addIssue(issues, path + ".id",
                "precisa ser único e não vazio");
        }
        if (link.parentIndex < 0) {
            ++rootCount;
            if (index != 0) {
                addIssue(issues, path + ".parent",
                    "somente o primeiro link pode ser raiz");
            }
        } else if (static_cast<std::size_t>(link.parentIndex) >= index) {
            addIssue(issues, path + ".parent",
                "o pai precisa existir antes do filho");
        }
        if (!finite(link.modelPosition) || !finite(link.modelOrientation)) {
            addIssue(issues, path + ".transform",
                "posição/orientação inválida");
        }
        if (!std::isfinite(link.massFraction) || link.massFraction <= 0.0f) {
            addIssue(issues, path + ".massFraction",
                "precisa ser positiva e finita");
        } else {
            massSum += link.massFraction;
        }
        if (link.collider.shape == RagdollColliderShape3D::Capsule) {
            if (!std::isfinite(link.collider.lengthMeters)
                || link.collider.lengthMeters
                    <= link.collider.radiusMeters * 2.0f) {
                addIssue(issues, path + ".capsule.length",
                    "precisa exceder o diâmetro da cápsula");
            }
            if (!std::isfinite(link.collider.radiusMeters)
                || link.collider.radiusMeters < 0.005f) {
                addIssue(issues, path + ".capsule.radius",
                    "raio precisa ser finito e ter ao menos 5 mm");
            }
        } else if (!finite(link.collider.boxHalfExtents)
            || link.collider.boxHalfExtents.x <= 0.0f
            || link.collider.boxHalfExtents.y <= 0.0f
            || link.collider.boxHalfExtents.z <= 0.0f) {
            addIssue(issues, path + ".box.halfExtents",
                "precisa conter três semiextensões positivas e finitas");
        }
        if (!finite(link.collider.localPosition)
            || !finite(link.collider.localOrientation)
            || !finite(link.centerOfMassLocal)) {
            addIssue(issues, path + ".capsule",
                "transform ou centro de massa inválido");
        }

        if (link.parentIndex < 0) continue;
        const auto& joint = link.inboundJoint;
        if (!finite(joint.anchorModelPosition)
            || !finite(joint.frameModelOrientation)) {
            addIssue(issues, path + ".joint",
                "frame da articulação inválido");
        }
        std::size_t enabledAxes = 0;
        for (std::size_t axisIndexValue = 0;
                axisIndexValue < joint.axes.size(); ++axisIndexValue) {
            const RagdollAxisDefinition3D& axis =
                joint.axes[axisIndexValue];
            if (!axis.enabled) continue;
            ++enabledAxes;
            const std::string axisPath = path + ".joint.axes["
                + std::to_string(axisIndexValue) + "]";
            if (!std::isfinite(axis.minimumRadians)
                || !std::isfinite(axis.maximumRadians)
                || axis.minimumRadians >= axis.maximumRadians) {
                addIssue(issues, axisPath,
                    "limite inferior precisa ser menor que o superior");
            }
            const float maximumAngle =
                joint.type == RagdollJointType3D::Spherical
                ? std::numbers::pi_v<float>
                : 2.0f * std::numbers::pi_v<float>;
            if (axis.minimumRadians < -maximumAngle
                || axis.maximumRadians > maximumAngle) {
                addIssue(issues, axisPath,
                    "limite excede a faixa aceita pelo PhysX");
            }
            if (axis.stiffness < 0.0f || axis.damping < 0.0f
                || axis.maximumTorque < 0.0f
                || !std::isfinite(axis.stiffness)
                || !std::isfinite(axis.damping)
                || !std::isfinite(axis.maximumTorque)) {
                addIssue(issues, axisPath,
                    "parâmetros de drive precisam ser finitos e não negativos");
            }
        }
        if (enabledAxes == 0) {
            addIssue(issues, path + ".joint.axes",
                "ao menos um eixo precisa estar habilitado");
        }
        if (joint.type == RagdollJointType3D::Revolute
            && (enabledAxes != 1
                || !joint.axes[static_cast<std::size_t>(
                    RagdollAxis3D::Twist)].enabled)) {
            addIssue(issues, path + ".joint.axes",
                "junta revoluta deve habilitar somente TWIST");
        }
    }
    if (rootCount != 1) {
        addIssue(issues, "links", "precisa existir exatamente uma raiz");
    }
    if (std::abs(massSum - 1.0f) > 0.001f) {
        addIssue(issues, "links.massFraction",
            "as frações de massa precisam somar 1,0 (soma atual "
                + std::to_string(massSum) + ")");
    }
    return issues;
}

void validateRagdollProfileOrThrow3D(const RagdollProfile3D& profile) {
    const auto issues = validateRagdollProfile3D(profile);
    if (issues.empty()) return;
    std::string message = "Perfil de ragdoll inválido";
    for (const auto& issue : issues) {
        message += "\n - " + issue.path + ": " + issue.message;
    }
    throw std::runtime_error(message);
}

std::size_t ragdollDegreesOfFreedom3D(const RagdollProfile3D& profile) {
    std::size_t count = 0;
    for (const RagdollLinkDefinition3D& link : profile.links) {
        if (link.parentIndex < 0) continue;
        count += static_cast<std::size_t>(std::count_if(
            link.inboundJoint.axes.begin(), link.inboundJoint.axes.end(),
            [](const RagdollAxisDefinition3D& axis) {
                return axis.enabled;
            }));
    }
    return count;
}

} // namespace MatterEngine
