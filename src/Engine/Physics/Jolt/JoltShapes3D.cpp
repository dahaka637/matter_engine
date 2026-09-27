#include "Engine/Physics/Jolt/JoltShapes3D.hpp"

#include "Engine/Physics/Jolt/JoltConversions.hpp"
#include "Engine/Physics/Jolt/JoltInternals3D.hpp"

#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace MatterEngine {
namespace {

constexpr float MinimumShapeSizeMeters = 0.001f;
const std::string DefaultMaterialId { "default" };

[[nodiscard]] bool hasLocalOffset(const PhysicsShape3D& shape) {
    constexpr float Epsilon = 1.0e-6f;
    const Quaternion& rotation = shape.localOrientation;
    return shape.localPosition.lengthSquared() > Epsilon * Epsilon
        || std::abs(rotation.x) > Epsilon || std::abs(rotation.y) > Epsilon
        || std::abs(rotation.z) > Epsilon
        || std::abs(std::abs(rotation.w) - 1.0f) > Epsilon;
}

// O resultado do Jolt carrega a mensagem de erro; perder isso transformaria uma
// shape degenerada em "falha ao criar shape" sem causa.
[[nodiscard]] JPH::RefConst<JPH::Shape> finalize(
    const JPH::ShapeSettings::ShapeResult& result, const char* what) {
    if (result.HasError()) {
        throw std::runtime_error(std::string("Jolt: ") + what + ": "
            + result.GetError().c_str());
    }
    return result.Get();
}

} // namespace

std::string_view joltShapeMaterialId(
    const JPH::PhysicsMaterial* material) {
    // Sem dynamic_cast de proposito: o Jolt e compilado com -fno-rtti, portanto
    // nao existe typeinfo para JPH::PhysicsMaterial e um dynamic_cast nem
    // linkaria. GetDebugName e a via que o proprio SDK oferece para nomear um
    // material, e JoltSurfaceMaterial devolve exatamente o id da engine.
    //
    // Um material que nao seja nosso (PhysicsMaterial::sDefault, por exemplo)
    // devolve um nome que nao existe no catalogo, e a busca cai em "default" -
    // o mesmo comportamento de antes.
    if (material == nullptr) return DefaultMaterialId;
    const char* name = material->GetDebugName();
    if (name == nullptr || *name == '\0') return DefaultMaterialId;
    return name;
}

JPH::RefConst<JPH::Shape> createJoltShape(const PhysicsShape3D& shape,
    bool allowConcaveMesh) {
    // O material acompanha a shape, nao o corpo: um compound de hulls pode ter
    // partes de materiais diferentes.
    JPH::Ref<JoltSurfaceMaterial> material =
        new JoltSurfaceMaterial(shape.materialId);

    JPH::RefConst<JPH::Shape> geometry;
    switch (shape.type) {
    case PhysicsShapeType3D::Box: {
        if (shape.halfExtents.x < MinimumShapeSizeMeters
            || shape.halfExtents.y < MinimumShapeSizeMeters
            || shape.halfExtents.z < MinimumShapeSizeMeters) {
            throw std::invalid_argument("Box collider degenerado");
        }
        // O raio convexo do Jolt precisa caber na menor meia-extensao, senao a
        // caixa fica arredondada demais (ou o SDK reclama). Caixas finas -
        // tampo de mesa, obstaculo baixo - sao comuns nos assets do projeto.
        const float smallestExtent = std::min({ shape.halfExtents.x,
            shape.halfExtents.y, shape.halfExtents.z });
        const float convexRadius = std::min(JPH::cDefaultConvexRadius,
            smallestExtent * 0.5f);
        JPH::BoxShapeSettings settings(toJolt(shape.halfExtents),
            convexRadius, material);
        geometry = finalize(settings.Create(), "caixa invalida");
        break;
    }
    case PhysicsShapeType3D::Sphere: {
        if (shape.radius < MinimumShapeSizeMeters) {
            throw std::invalid_argument("Sphere collider degenerado");
        }
        JPH::SphereShapeSettings settings(shape.radius, material);
        geometry = finalize(settings.Create(), "esfera invalida");
        break;
    }
    case PhysicsShapeType3D::Capsule: {
        if (shape.radius < MinimumShapeSizeMeters
            || shape.capsuleHalfHeight < 0.0f) {
            throw std::invalid_argument("Capsule collider degenerado");
        }
        // Meia-altura do cilindro (sem as tampas) e eixo longitudinal em Y: a
        // mesma convencao do contrato neutro, portanto sem correcao.
        if (shape.capsuleHalfHeight <= MinimumShapeSizeMeters) {
            // O Jolt rejeita capsula de cilindro nulo; uma esfera e o que ela
            // geometricamente e.
            JPH::SphereShapeSettings settings(shape.radius, material);
            geometry = finalize(settings.Create(),
                "capsula degenerada em esfera");
            break;
        }
        if (shape.capsuleTopRadius > 0.0f
            && std::abs(shape.capsuleTopRadius - shape.radius) > 0.0005f) {
            JPH::TaperedCapsuleShapeSettings settings(shape.capsuleHalfHeight,
                shape.capsuleTopRadius, shape.radius, material);
            geometry = finalize(settings.Create(), "capsula conica invalida");
            break;
        }
        JPH::CapsuleShapeSettings settings(shape.capsuleHalfHeight,
            shape.radius, material);
        geometry = finalize(settings.Create(), "capsula invalida");
        break;
    }
    case PhysicsShapeType3D::ConvexMesh: {
        const PhysicsMesh3D::Impl* mesh = shape.mesh
            ? PhysicsMesh3D::Impl::of(*shape.mesh) : nullptr;
        if (shape.mesh == nullptr
            || shape.mesh->type() != PhysicsMeshType3D::Convex
            || mesh == nullptr || !mesh->shape) {
            throw std::invalid_argument("Convex mesh invalida");
        }
        // A shape cozida e imutavel e compartilhada por milhares de
        // instancias; o material dela foi fixado no cooking.
        geometry = mesh->shape;
        break;
    }
    case PhysicsShapeType3D::TriangleMesh: {
        if (!allowConcaveMesh) {
            throw std::invalid_argument(
                "Triangle mesh nao pode ser usada em corpo dinamico");
        }
        const PhysicsMesh3D::Impl* mesh = shape.mesh
            ? PhysicsMesh3D::Impl::of(*shape.mesh) : nullptr;
        if (shape.mesh == nullptr
            || shape.mesh->type() != PhysicsMeshType3D::Triangle
            || mesh == nullptr || !mesh->shape) {
            throw std::invalid_argument("Triangle mesh invalida");
        }
        geometry = mesh->shape;
        break;
    }
    }

    if (!geometry) {
        throw std::runtime_error("Jolt: tipo de shape nao suportado");
    }
    if (!hasLocalOffset(shape)) return geometry;

    JPH::RotatedTranslatedShapeSettings offset(toJolt(shape.localPosition),
        toJolt(shape.localOrientation), geometry);
    return finalize(offset.Create(), "offset local invalido");
}

JPH::RefConst<JPH::Shape> createJoltBodyShape(
    std::span<const PhysicsShape3D> shapes, bool allowConcaveMesh) {
    if (shapes.empty()) {
        throw std::invalid_argument("Corpo fisico exige pelo menos uma shape");
    }
    if (shapes.size() == 1) {
        return createJoltShape(shapes.front(), allowConcaveMesh);
    }

    JPH::StaticCompoundShapeSettings compound;
    compound.mSubShapes.reserve(shapes.size());
    for (const PhysicsShape3D& shape : shapes) {
        // O offset ja esta dentro da shape traduzida (via
        // RotatedTranslatedShape), portanto o compound a adiciona na origem.
        // Empilhar os dois deslocaria duas vezes.
        compound.AddShape(JPH::Vec3::sZero(), JPH::Quat::sIdentity(),
            createJoltShape(shape, allowConcaveMesh).GetPtr());
    }
    return finalize(compound.Create(), "compound invalido");
}

} // namespace MatterEngine
