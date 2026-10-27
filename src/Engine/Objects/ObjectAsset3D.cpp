#include "Engine/Objects/ObjectAsset3D.hpp"

#include "Engine/Modeling/Mesh/ShadingNormals.hpp"
#include "Engine/Physics/PhysicalBodyBuilder3D.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <tuple>

namespace MatterEngine {

namespace {

constexpr float CreaseDegrees = 45.0f;

} // namespace

ObjectRenderData3D objectRenderData3D(const Modeling::ProceduralObject& object) {
    const Modeling::Mesh& mesh = object.mesh;
    ObjectRenderData3D result;
    const Modeling::ShadingNormals shading(mesh, CreaseDegrees);
    std::vector<Vec3> corners;
    shading.compute(mesh.positions, mesh.triangles, 0.0f, corners);
    const bool uv = mesh.hasUv();
    std::map<std::tuple<std::uint32_t, std::uint32_t, std::int32_t, std::int32_t>, std::uint32_t> unique;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (int k = 0; k < 3; ++k) {
            const std::uint32_t source = mesh.triangles[t][static_cast<std::size_t>(k)];
            const Modeling::Uv coordinate = uv ? mesh.triangleUv[t][static_cast<std::size_t>(k)] : Modeling::Uv {};
            const auto key = std::make_tuple(source, shading.group(t, k), static_cast<std::int32_t>(std::lround(coordinate.u * 1048576.0f)),
                static_cast<std::int32_t>(std::lround(coordinate.v * 1048576.0f)));
            auto found = unique.find(key);
            if (found == unique.end()) {
                MeshVertex3D vertex;
                vertex.position = mesh.positions[source];
                vertex.normal = corners[t * 3 + static_cast<std::size_t>(k)];
                vertex.uv = { coordinate.u, coordinate.v };
                // Sem textura, a cor do material vai no vertice.
                const std::size_t material = t < mesh.triangleMaterial.size() ? mesh.triangleMaterial[t] : 0;
                if (material < mesh.materials.size() && mesh.materials[material].texture < 0) {
                    const Modeling::Color color = mesh.materials[material].color;
                    vertex.color = { color.r, color.g, color.b };
                }
                result.mesh.vertices.push_back(vertex);
                found = unique.emplace(key, static_cast<std::uint32_t>(result.mesh.vertices.size() - 1)).first;
            }
            result.mesh.indices.push_back(found->second);
        }
    recomputeBounds(result.mesh);
    if (!mesh.textures.empty()) {
        const Modeling::Image& image = mesh.textures.front().image;
        result.width = static_cast<std::uint32_t>(image.width());
        result.height = static_cast<std::uint32_t>(image.height());
        const auto pixels = image.pixels();
        result.rgba.resize(pixels.size() * 4);
        for (std::size_t i = 0; i < pixels.size(); ++i) {
            result.rgba[i * 4 + 0] = static_cast<std::byte>(pixels[i].r);
            result.rgba[i * 4 + 1] = static_cast<std::byte>(pixels[i].g);
            result.rgba[i * 4 + 2] = static_cast<std::byte>(pixels[i].b);
            result.rgba[i * 4 + 3] = static_cast<std::byte>(pixels[i].a);
        }
    }
    return result;
}

ObjectBodyData3D objectBodyData3D(const Modeling::ProceduralObject& object, const MaterialLibrary& materials) {
    const Modeling::ObjectPhysics& physics = object.physics;
    const SurfaceMaterial* material = materials.find(physics.material);
    if (material == nullptr) material = materials.find("default");
    if (material == nullptr) throw std::invalid_argument("catalogo de materiais sem o default");
    if (physics.colliders.empty()) throw std::invalid_argument(object.id + ": objeto sem forma de colisao");
    ObjectBodyData3D result;
    result.dimensions = object.size;
    float volume = 0.0f;
    for (const Modeling::ObjectCollider& collider : physics.colliders) {
        PhysicsShape3D shape;
        shape.localPosition = collider.center;
        shape.localOrientation = collider.orientation;
        shape.materialId = material->id;
        constexpr float Pi = 3.14159265f;
        switch (collider.shape) {
        case Modeling::ObjectColliderShape::Box:
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = collider.halfExtents;
            volume += 8.0f * collider.halfExtents.x * collider.halfExtents.y * collider.halfExtents.z;
            break;
        case Modeling::ObjectColliderShape::Sphere:
            shape.type = PhysicsShapeType3D::Sphere;
            shape.radius = collider.radius;
            volume += 4.0f / 3.0f * Pi * collider.radius * collider.radius * collider.radius;
            break;
        case Modeling::ObjectColliderShape::Capsule:
        case Modeling::ObjectColliderShape::Cylinder: {
            // A capsula da fisica e ao longo do y local; a do objeto, do z.
            shape.type = PhysicsShapeType3D::Capsule;
            shape.radius = collider.radius;
            const bool cylinder = collider.shape == Modeling::ObjectColliderShape::Cylinder;
            shape.capsuleHalfHeight = std::max(0.001f, cylinder ? collider.halfHeight - collider.radius : collider.halfHeight);
            shape.localOrientation = (collider.orientation * Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, Pi * 0.5f)).normalized();
            volume += Pi * collider.radius * collider.radius * 2.0f * collider.halfHeight;
            break;
        }
        }
        result.shapes.push_back(shape);
    }
    BodyMaterialBinding3D binding;
    binding.massMode = BodyMassMode3D::OverrideKilograms;
    binding.massOverrideKg = std::max(0.01f, physics.massKg);
    result.body = buildDynamicBodyDefinition(*material, binding, std::max(volume, 1e-6f), object.size);
    if (physics.fixed) result.body.motionType = PhysicsMotionType3D::Static;
    return result;
}

ObjectNetRenderData3D objectNetRenderData3D(const Modeling::ProceduralObject& object, Vec3 position, Quaternion orientation) {
    ObjectNetRenderData3D result;
    if (!object.net) return result;
    const Modeling::ObjectNet& net = *object.net;
    for (const Vec3& knot : net.knots) result.restKnots.push_back(position + orientation.rotate(knot));
    // (O tubo de 3 lados e mais estreito que o redondo: com 1,25 x o raio do
    // fio ele cobre a largura do fio de qualquer lado.)
    const float radius = 0.625f * net.twineDiameter;
    const Vec3 color { net.color.r, net.color.g, net.color.b };
    constexpr float Third = 2.0943951f;
    result.mesh.vertices.reserve(net.drawnSegments.size() * 6);
    result.mesh.indices.reserve(net.drawnSegments.size() * 18);
    for (const auto& [a, b] : net.drawnSegments) {
        const Vec3 from = position + orientation.rotate(net.drawnKnots[a]);
        const Vec3 to = position + orientation.rotate(net.drawnKnots[b]);
        const Vec3 along = to - from;
        const float length = std::sqrt(along.lengthSquared());
        if (length < 1e-6f) continue;
        const Vec3 direction = along * (1.0f / length);
        const Vec3 reference = std::abs(direction.z) < 0.9f ? Vec3 { 0.0f, 0.0f, 1.0f } : Vec3 { 1.0f, 0.0f, 0.0f };
        Vec3 side = cross(direction, reference);
        side = side * (1.0f / std::sqrt(side.lengthSquared()));
        const Vec3 up = cross(direction, side);
        const auto first = static_cast<std::uint32_t>(result.mesh.vertices.size());
        for (const std::uint32_t end : { a, b }) {
            const Modeling::ObjectNetWeight& weight = net.drawnWeights[end];
            const Vec3 center = end == a ? from : to;
            for (int k = 0; k < 3; ++k) {
                const float angle = Third * static_cast<float>(k);
                const Vec3 out = side * std::cos(angle) + up * std::sin(angle);
                MeshVertex3D vertex;
                vertex.position = center + out * radius;
                vertex.normal = out;
                vertex.color = color;
                vertex.joints = weight.knots;
                vertex.weights = weight.weights;
                result.mesh.vertices.push_back(vertex);
            }
        }
        for (std::uint32_t k = 0; k < 3; ++k) {
            const std::uint32_t next = (k + 1) % 3;
            for (const std::uint32_t index : { first + k, first + 3 + k, first + 3 + next, first + k, first + 3 + next, first + next })
                result.mesh.indices.push_back(index);
        }
    }
    recomputeBounds(result.mesh);
    return result;
}

void objectNetSkin3D(std::span<const Vec3> knots, std::span<const Vec3> restKnots, std::vector<Mat4>& out) {
    out.resize(std::min(knots.size(), restKnots.size()));
    for (std::size_t i = 0; i < out.size(); ++i) out[i] = Mat4::translation(knots[i] - restKnots[i]);
}

NetDescription3D objectNetDescription3D(const Modeling::ProceduralObject& object, Vec3 position, Quaternion orientation,
    float groundHeight) {
    NetDescription3D result;
    if (!object.net) return result;
    const Modeling::ObjectNet& net = *object.net;
    result.positions.reserve(net.knots.size());
    for (const Vec3& knot : net.knots) result.positions.push_back(position + orientation.rotate(knot));
    result.masses = net.masses;
    result.pinned = net.pinned;
    result.segments.reserve(net.segments.size());
    for (const Modeling::ObjectNetSegment& segment : net.segments)
        result.segments.push_back({ segment.a, segment.b, segment.restLength, segment.stiffness, segment.cord });
    result.faces = net.faces;
    for (const Modeling::ObjectNetWeight& w : net.drawnWeights) result.samples.push_back({ w.knots, w.weights });
    for (const float length : net.twineLengths) result.dragAreas.push_back(net.twineDiameter * length);
    for (const Modeling::ObjectCollider& collider : object.physics.colliders) {
        if (collider.shape != Modeling::ObjectColliderShape::Cylinder && collider.shape != Modeling::ObjectColliderShape::Capsule) continue;
        const Vec3 center = position + orientation.rotate(collider.center);
        const Vec3 axis = (orientation * collider.orientation).rotate({ 0.0f, 0.0f, collider.halfHeight });
        result.colliders.push_back({ center - axis, center + axis, collider.radius });
    }
    result.settings.twineRadius = 0.5f * net.twineDiameter;
    result.settings.contactMassScale = net.contactMassScale;
    result.settings.groundHeight = groundHeight;
    return result;
}

std::vector<ObjectBodyData3D> objectNetBarrier3D(const Modeling::ProceduralObject& object) {
    std::vector<ObjectBodyData3D> result;
    if (!object.net) return result;
    const auto add = [&](const std::vector<std::vector<Vec3>>& hulls, PhysicsBarrier3D role) {
        if (hulls.empty()) return;
        ObjectBodyData3D& body = result.emplace_back();
        body.body.motionType = PhysicsMotionType3D::Static;
        body.body.barrier = role;
        body.dimensions = object.size;
        for (const std::vector<Vec3>& hull : hulls) {
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::ConvexHull;
            shape.hullPoints = hull;
            body.shapes.push_back(std::move(shape));
        }
    };
    add(object.net->barrier, PhysicsBarrier3D::Players);
    add(object.net->capsuleBarrier, PhysicsBarrier3D::Capsule);
    return result;
}

} // namespace MatterEngine
