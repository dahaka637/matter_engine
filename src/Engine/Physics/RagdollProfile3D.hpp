#pragma once

#include "Engine/Math/Quaternion.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine {

struct RagdollHandle3D {
    std::uint32_t index = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t generation = 0;

    [[nodiscard]] constexpr bool valid() const {
        return index != std::numeric_limits<std::uint32_t>::max();
    }
    constexpr explicit operator bool() const { return valid(); }
    constexpr bool operator==(const RagdollHandle3D&) const = default;
};

enum class RagdollJointType3D : std::uint8_t {
    Revolute,
    Spherical
};

enum class RagdollAxis3D : std::uint8_t {
    Twist = 0,
    Swing1 = 1,
    Swing2 = 2
};

enum class RagdollColliderShape3D : std::uint8_t {
    Capsule,
    Box
};

struct RagdollAxisDefinition3D {
    bool enabled = false;
    float minimumRadians = 0.0f;
    float maximumRadians = 0.0f;
    float stiffness = 0.0f;
    float damping = 0.0f;
    float maximumTorque = 0.0f;
};

struct RagdollCapsuleDefinition3D {
    RagdollColliderShape3D shape = RagdollColliderShape3D::Capsule;
    Vec3 localPosition;
    Quaternion localOrientation;
    // Comprimento total, incluindo as duas tampas hemisféricas.
    float lengthMeters = 0.1f;
    float radiusMeters = 0.05f;
    // Usado quando shape == Box. Mantemos o mesmo container para não
    // fragmentar o contrato do perfil; cada shape valida apenas seus campos.
    Vec3 boxHalfExtents { 0.05f, 0.05f, 0.05f };
    std::string materialId = "default";
    // Contatos detalhados são caros e, portanto, opt-in por link. Pés usam
    // esta telemetria para suporte/CoP; mãos e joelhos poderão habilitá-la
    // quando o controlador de levantar-se for implementado.
    bool contactSensor = false;
};

struct RagdollJointDefinition3D {
    RagdollJointType3D type = RagdollJointType3D::Spherical;
    // Ponto da articulação no espaço de modelo do ragdoll.
    Vec3 anchorModelPosition;
    // Base anatômica compartilhada pelos frames pai e filho. O eixo X desta
    // base vira TWIST; Y/Z viram SWING1/SWING2 no adaptador PhysX.
    Quaternion frameModelOrientation;
    std::array<RagdollAxisDefinition3D, 3> axes;
};

struct RagdollLinkDefinition3D {
    std::string id;
    // -1 identifica a raiz flutuante. Pais precisam aparecer antes dos filhos.
    int parentIndex = -1;
    Vec3 modelPosition;
    Quaternion modelOrientation;
    RagdollCapsuleDefinition3D collider;
    float massFraction = 0.0f;
    Vec3 centerOfMassLocal;
    RagdollJointDefinition3D inboundJoint;
};

struct RagdollProfile3D {
    std::string id;
    float totalMassKg = 75.0f;
    // Cápsulas anatômicas mantêm o mesmo raio. Links funcionais como os pés
    // podem declarar caixas para criar uma área de suporte física real.
    // Legacy field name retained for assets; default radius for capsules
    // without an explicit per-link radius (not a uniformity restriction).
    float uniformRadiusMeters = 0.055f;
    // Distância vertical da origem da pelve ao ponto mais baixo na pose neutra.
    float standingRootHeightMeters = 0.98f;
    std::vector<RagdollLinkDefinition3D> links;
};

struct RagdollProfileValidationIssue3D {
    std::string path;
    std::string message;
};

[[nodiscard]] RagdollProfile3D loadRagdollProfile3D(
    std::string_view filePath);
[[nodiscard]] std::vector<RagdollProfileValidationIssue3D>
    validateRagdollProfile3D(const RagdollProfile3D& profile);
void validateRagdollProfileOrThrow3D(const RagdollProfile3D& profile);
[[nodiscard]] std::size_t ragdollDegreesOfFreedom3D(
    const RagdollProfile3D& profile);

} // namespace MatterEngine
