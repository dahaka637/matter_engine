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

// A casca pronta para uso: centrada na caixa justa dela (o colisor desloca
// junto, e a caixa vira o contorno para quem le so ela) e com os planos das
// faces (a normal para fora pelo centroide).
void prepareHull(RagdollCapsuleDefinition3D& collider) {
    if (collider.hullPoints.empty()) return;
    Vec3 low { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    Vec3 high = low * -1.0f;
    for (const Vec3& p : collider.hullPoints) {
        low = { std::min(low.x, p.x), std::min(low.y, p.y), std::min(low.z, p.z) };
        high = { std::max(high.x, p.x), std::max(high.y, p.y), std::max(high.z, p.z) };
    }
    const Vec3 center = (low + high) * 0.5f;
    for (Vec3& p : collider.hullPoints) p = p - center;
    collider.localPosition = collider.localPosition + collider.localOrientation.rotate(center);
    collider.boxHalfExtents = (high - low) * 0.5f;
    Vec3 centroid {};
    for (const Vec3& p : collider.hullPoints) centroid = centroid + p;
    centroid = centroid * (1.0f / static_cast<float>(collider.hullPoints.size()));
    collider.hullPlanes.clear();
    for (const auto& face : collider.hullFaces) {
        if (face[0] >= collider.hullPoints.size() || face[1] >= collider.hullPoints.size()
            || face[2] >= collider.hullPoints.size()) {
            collider.hullPlanes.push_back({ 0.0f, 0.0f, 0.0f, 0.0f });
            continue;
        }
        const Vec3 a = collider.hullPoints[face[0]], b = collider.hullPoints[face[1]], c = collider.hullPoints[face[2]];
        Vec3 normal = cross(b - a, c - a);
        const float length = normal.length();
        if (length < 1.0e-12f) {
            collider.hullPlanes.push_back({ 0.0f, 0.0f, 0.0f, 0.0f });
            continue;
        }
        normal = normal * (1.0f / length);
        if (dot(normal, a - centroid) < 0.0f) normal = normal * -1.0f;
        collider.hullPlanes.push_back({ normal.x, normal.y, normal.z, dot(normal, a) });
    }
    // O contorno da sola: em 16 rumos, o vertice do plano de baixo (ate 3 mm
    // dele) mais longe nesse rumo; sem repetir, na ordem dos rumos.
    collider.hullSole.clear();
    const float bottom = -collider.boxHalfExtents.z;
    constexpr int Directions = 16;
    for (int k = 0; k < Directions; ++k) {
        const float angle = 6.2831853f * static_cast<float>(k) / static_cast<float>(Directions);
        const float cx = std::cos(angle), cy = std::sin(angle);
        const Vec3* best = nullptr;
        float reach = -std::numeric_limits<float>::max();
        for (const Vec3& p : collider.hullPoints) {
            if (p.z > bottom + 0.003f) continue;
            const float r = cx * p.x + cy * p.y;
            if (r > reach) {
                reach = r;
                best = &p;
            }
        }
        if (best && (collider.hullSole.empty() || (collider.hullSole.back() - *best).lengthSquared() > 1e-10f)
            && (collider.hullSole.empty() || (collider.hullSole.front() - *best).lengthSquared() > 1e-10f))
            collider.hullSole.push_back({ best->x, best->y, bottom });
    }
}

// O ponto de um triangulo mais perto de p (Ericson, "Real-Time Collision
// Detection", 5.1.5).
Vec3 closestOnTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;
    const Vec3 bp = p - b;
    const float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) return a + ab * (d1 / (d1 - d3));
    const Vec3 cp = p - c;
    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) return a + ac * (d2 / (d2 - d6));
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const float denominator = 1.0f / (va + vb + vc);
    return a + ab * (vb * denominator) + ac * (vc * denominator);
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

            const bool hasHull = source.contains("hull");
            const bool hasBox = !hasHull && source.contains("box");
            const std::string kind = hasHull ? "hull" : hasBox ? "box" : "capsule";
            const Json& shape = source.at(kind);
            link.collider.shape = hasHull ? RagdollColliderShape3D::ConvexHull
                : hasBox                  ? RagdollColliderShape3D::Box
                                          : RagdollColliderShape3D::Capsule;
            link.collider.localPosition = shape.contains("position")
                ? readVec3(shape.at("position"), path + "." + kind + ".position")
                : Vec3 {};
            link.collider.localOrientation =
                readQuaternion(shape.at("orientation"), path + "." + kind + ".orientation");
            if (hasBox) {
                link.collider.boxHalfExtents = readVec3(
                    shape.at("halfExtents"), path + ".box.halfExtents");
            } else if (hasHull) {
                for (std::size_t p = 0; p < shape.at("points").size(); ++p)
                    link.collider.hullPoints.push_back(readVec3(shape.at("points").at(p),
                        path + ".hull.points[" + std::to_string(p) + "]"));
                for (const Json& face : shape.at("faces")) {
                    if (!face.is_array() || face.size() != 3)
                        throw std::runtime_error("Perfil de ragdoll: face invalida em " + path + ".hull.faces");
                    link.collider.hullFaces.push_back({ face.at(0).get<std::uint16_t>(),
                        face.at(1).get<std::uint16_t>(), face.at(2).get<std::uint16_t>() });
                }
                link.collider.convexRadiusMeters = shape.value("convexRadius", link.collider.convexRadiusMeters);
                prepareHull(link.collider);
            } else {
                link.collider.lengthMeters =
                    shape.at("length").get<float>();
                link.collider.radiusMeters = shape.contains("radius")
                    ? shape.at("radius").get<float>()
                    : profile.uniformRadiusMeters;
                link.collider.radiusAtPositiveXMeters =
                    shape.value("radiusAtPositiveX", 0.0f);
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
        if (root.contains("selfCollisionIgnoredPairs")) {
            for (const Json& pair : root.at("selfCollisionIgnoredPairs")) {
                const Json& ids = pair.at("links");
                if (!ids.is_array() || ids.size() != 2) {
                    throw std::runtime_error(
                        "selfCollisionIgnoredPairs: links precisa ter 2 ids");
                }
                profile.selfCollisionIgnoredPairs.push_back({
                    ids.at(0).get<std::string>(),
                    ids.at(1).get<std::string>(),
                    pair.value("reason", std::string {})
                });
            }
        }
    } catch (const Json::exception& error) {
        throw std::runtime_error("Perfil de ragdoll incompleto em "
            + std::string(filePath) + ": " + error.what());
    }

    validateRagdollProfileOrThrow3D(profile);
    return profile;
}

float ragdollCapsuleRadiusAtPositiveX3D(
    const RagdollCapsuleDefinition3D& capsule) {
    return capsule.radiusAtPositiveXMeters > 0.0f
        ? capsule.radiusAtPositiveXMeters : capsule.radiusMeters;
}

float ragdollCapsuleHalfSegment3D(const RagdollCapsuleDefinition3D& capsule) {
    return std::max(0.0f, 0.5f * (capsule.lengthMeters - capsule.radiusMeters
        - ragdollCapsuleRadiusAtPositiveX3D(capsule)));
}

bool ragdollColliderHasBox3D(const RagdollCapsuleDefinition3D& collider) {
    return collider.shape == RagdollColliderShape3D::Box || collider.shape == RagdollColliderShape3D::ConvexHull;
}

RagdollColliderClosest3D ragdollColliderClosestPoint3D(const RagdollCapsuleDefinition3D& collider, Vec3 p) {
    RagdollColliderClosest3D out;
    if (collider.shape == RagdollColliderShape3D::Box) {
        const Vec3 h = collider.boxHalfExtents;
        const Vec3 q { std::clamp(p.x, -h.x, h.x), std::clamp(p.y, -h.y, h.y), std::clamp(p.z, -h.z, h.z) };
        const Vec3 away = p - q;
        const float length = away.length();
        if (length > 1.0e-9f) {
            out.point = q;
            out.normal = away * (1.0f / length);
            out.distance = length;
            return out;
        }
        // Por dentro: a face mais perto.
        const std::array<float, 3> gaps { h.x - std::abs(p.x), h.y - std::abs(p.y), h.z - std::abs(p.z) };
        const std::size_t axis = static_cast<std::size_t>(std::min_element(gaps.begin(), gaps.end()) - gaps.begin());
        std::array<float, 3> point { p.x, p.y, p.z }, normal { 0.0f, 0.0f, 0.0f };
        const std::array<float, 3> half { h.x, h.y, h.z };
        const float sign = point[axis] < 0.0f ? -1.0f : 1.0f;
        point[axis] = sign * half[axis];
        normal[axis] = sign;
        out.point = { point[0], point[1], point[2] };
        out.normal = { normal[0], normal[1], normal[2] };
        out.distance = -gaps[axis];
        return out;
    }
    if (collider.shape == RagdollColliderShape3D::Capsule) {
        // O eixo e o x local; na conica, o raio vai do da -X ao da +X.
        const float half = ragdollCapsuleHalfSegment3D(collider);
        const float t = std::clamp(p.x, -half, half);
        const float blend = half > 1.0e-6f ? (t + half) / (2.0f * half) : 0.5f;
        const float radius = collider.radiusMeters
            + (ragdollCapsuleRadiusAtPositiveX3D(collider) - collider.radiusMeters) * blend;
        const Vec3 away = p - Vec3 { t, 0.0f, 0.0f };
        const float length = away.length();
        out.normal = length > 1.0e-9f ? away * (1.0f / length) : Vec3 { 0.0f, 0.0f, 1.0f };
        out.point = Vec3 { t, 0.0f, 0.0f } + out.normal * radius;
        out.distance = length - radius;
        return out;
    }
    // A casca convexa: por dentro, o plano de face mais perto; por fora, o
    // ponto mais perto dos triangulos.
    float deepest = -std::numeric_limits<float>::max();
    std::size_t deepestFace = 0;
    for (std::size_t f = 0; f < collider.hullPlanes.size(); ++f) {
        const auto& plane = collider.hullPlanes[f];
        const float signedDistance = plane[0] * p.x + plane[1] * p.y + plane[2] * p.z - plane[3];
        if (signedDistance > deepest) {
            deepest = signedDistance;
            deepestFace = f;
        }
    }
    if (collider.hullPlanes.empty()) return out;
    if (deepest <= 0.0f) {
        const auto& plane = collider.hullPlanes[deepestFace];
        out.normal = { plane[0], plane[1], plane[2] };
        out.point = p - out.normal * deepest;
        out.distance = deepest;
        return out;
    }
    float best = std::numeric_limits<float>::max();
    for (const auto& face : collider.hullFaces) {
        const Vec3 q = closestOnTriangle(p, collider.hullPoints[face[0]], collider.hullPoints[face[1]],
            collider.hullPoints[face[2]]);
        const float squared = (p - q).lengthSquared();
        if (squared < best) {
            best = squared;
            out.point = q;
        }
    }
    out.distance = std::sqrt(best);
    out.normal = out.distance > 1.0e-9f ? (p - out.point) * (1.0f / out.distance) : Vec3 { 0.0f, 0.0f, 1.0f };
    return out;
}

std::vector<Vec3> ragdollColliderSolePoints3D(const RagdollCapsuleDefinition3D& collider) {
    if (collider.shape == RagdollColliderShape3D::ConvexHull && collider.hullSole.size() >= 3) return collider.hullSole;
    if (ragdollColliderHasBox3D(collider)) {
        const Vec3 h = collider.boxHalfExtents;
        return { { -h.x, -h.y, -h.z }, { h.x, -h.y, -h.z }, { h.x, h.y, -h.z }, { -h.x, h.y, -h.z } };
    }
    const float half = ragdollCapsuleHalfSegment3D(collider);
    return { { -half, 0.0f, -collider.radiusMeters }, { half, 0.0f, -ragdollCapsuleRadiusAtPositiveX3D(collider) } };
}

std::vector<Vec3> ragdollColliderCorners3D(const RagdollCapsuleDefinition3D& collider) {
    if (collider.shape == RagdollColliderShape3D::ConvexHull) return collider.hullPoints;
    std::vector<Vec3> corners;
    if (collider.shape == RagdollColliderShape3D::Box) {
        const Vec3 h = collider.boxHalfExtents;
        for (const float x : { -h.x, h.x })
            for (const float y : { -h.y, h.y })
                for (const float z : { -h.z, h.z }) corners.push_back({ x, y, z });
        return corners;
    }
    const float half = ragdollCapsuleHalfSegment3D(collider);
    for (const auto& [x, radius] : { std::pair { -half, collider.radiusMeters },
             std::pair { half, ragdollCapsuleRadiusAtPositiveX3D(collider) } })
        for (const Vec3 axis : { Vec3 { 1.0f, 0.0f, 0.0f }, Vec3 { 0.0f, 1.0f, 0.0f }, Vec3 { 0.0f, 0.0f, 1.0f } }) {
            corners.push_back(Vec3 { x, 0.0f, 0.0f } + axis * radius);
            corners.push_back(Vec3 { x, 0.0f, 0.0f } - axis * radius);
        }
    return corners;
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
            const float endRadius =
                ragdollCapsuleRadiusAtPositiveX3D(link.collider);
            if (!std::isfinite(link.collider.lengthMeters)
                || link.collider.lengthMeters
                    <= link.collider.radiusMeters + endRadius) {
                addIssue(issues, path + ".capsule.length",
                    "precisa exceder a soma dos raios das duas pontas");
            }
            if (!std::isfinite(link.collider.radiusAtPositiveXMeters)
                || (link.collider.radiusAtPositiveXMeters != 0.0f
                    && link.collider.radiusAtPositiveXMeters < 0.005f)) {
                addIssue(issues, path + ".capsule.radiusAtPositiveX",
                    "raio da ponta +X precisa ter ao menos 5 mm");
            }
            if (!std::isfinite(link.collider.radiusMeters)
                || link.collider.radiusMeters < 0.005f) {
                addIssue(issues, path + ".capsule.radius",
                    "raio precisa ser finito e ter ao menos 5 mm");
            }
        } else if (link.collider.shape == RagdollColliderShape3D::ConvexHull) {
            const RagdollCapsuleDefinition3D& hull = link.collider;
            if (hull.hullPoints.size() < 4 || hull.hullPoints.size() > 256)
                addIssue(issues, path + ".hull.points", "precisa de 4 a 256 pontos");
            if (hull.hullFaces.size() < 4 || hull.hullPlanes.size() != hull.hullFaces.size())
                addIssue(issues, path + ".hull.faces", "precisa de ao menos 4 faces");
            for (const Vec3& point : hull.hullPoints)
                if (!finite(point)) {
                    addIssue(issues, path + ".hull.points", "precisa de pontos finitos");
                    break;
                }
            // Convexa: nenhum ponto passa de um plano de face (1 mm de folga).
            bool convex = true, degenerate = false;
            for (const auto& plane : hull.hullPlanes) {
                if (plane[0] == 0.0f && plane[1] == 0.0f && plane[2] == 0.0f) degenerate = true;
                for (const Vec3& point : hull.hullPoints)
                    if (plane[0] * point.x + plane[1] * point.y + plane[2] * point.z - plane[3] > 0.001f) convex = false;
            }
            if (degenerate) addIssue(issues, path + ".hull.faces", "tem face degenerada ou indice fora dos pontos");
            if (!convex) addIssue(issues, path + ".hull", "nao e convexa");
            const float smallest = std::min({ hull.boxHalfExtents.x, hull.boxHalfExtents.y, hull.boxHalfExtents.z });
            if (!std::isfinite(hull.convexRadiusMeters) || hull.convexRadiusMeters < 0.0f
                || hull.convexRadiusMeters > 0.5f * smallest)
                addIssue(issues, path + ".hull.convexRadius", "precisa ficar entre 0 e metade da menor medida");
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
    for (std::size_t index = 0;
            index < profile.selfCollisionIgnoredPairs.size(); ++index) {
        const auto& pair = profile.selfCollisionIgnoredPairs[index];
        const std::string path = "selfCollisionIgnoredPairs["
            + std::to_string(index) + "]";
        if (!ids.contains(pair.firstLinkId)
            || !ids.contains(pair.secondLinkId)) {
            addIssue(issues, path, "link desconhecido");
        } else if (pair.firstLinkId == pair.secondLinkId) {
            addIssue(issues, path, "par precisa de dois links diferentes");
        }
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

void prepareRagdollConvexHull3D(RagdollCapsuleDefinition3D& collider) {
    collider.shape = RagdollColliderShape3D::ConvexHull;
    prepareHull(collider);
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
