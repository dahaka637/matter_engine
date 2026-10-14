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
    Box,
    // Casca convexa (a chuteira): pontos e faces no referencial do colisor.
    // boxHalfExtents e a caixa justa dela, centrada (o contorno da sola, a
    // ponta e o calcanhar para quem so precisa deles).
    ConvexHull
};

struct RagdollCapsuleDefinition3D;
// Raio da cápsula na ponta +X local (igual ao da -X se for uniforme).
[[nodiscard]] float ragdollCapsuleRadiusAtPositiveX3D(
    const RagdollCapsuleDefinition3D& capsule);
// Meia distância entre os centros das duas esferas das pontas.
[[nodiscard]] float ragdollCapsuleHalfSegment3D(
    const RagdollCapsuleDefinition3D& capsule);
// O colisor tem caixa (a dele, ou a justa de uma casca convexa): os que leem o
// contorno de um pe (a sola, a ponta, o calcanhar) aceitam os dois.
[[nodiscard]] bool ragdollColliderHasBox3D(const RagdollCapsuleDefinition3D& collider);
// O ponto do colisor mais perto de um ponto, tudo no referencial do colisor
// (localPosition/localOrientation): a distancia com sinal (negativa por
// dentro) e a normal de fora no ponto. Caixa, capsula e casca convexa exatas.
struct RagdollColliderClosest3D {
    Vec3 point;
    Vec3 normal { 0.0f, 0.0f, 1.0f };
    float distance = 0.0f;
};
[[nodiscard]] RagdollColliderClosest3D ragdollColliderClosestPoint3D(
    const RagdollCapsuleDefinition3D& collider, Vec3 localPoint);
// Os pontos de canto do colisor no referencial dele (os 8 da caixa, os da
// casca; a capsula, os das duas tampas nos 6 rumos): o mais baixo no mundo, a
// sola de um pe, sai deles.
[[nodiscard]] std::vector<Vec3> ragdollColliderCorners3D(const RagdollCapsuleDefinition3D& collider);
// O contorno da sola (o apoio no chao) no referencial do colisor: os 4 cantos
// de baixo da caixa; na casca, os vertices do plano de baixo (ate 16, os de
// mais longe em cada rumo, em volta). A capsula: as duas tampas por baixo.
[[nodiscard]] std::vector<Vec3> ragdollColliderSolePoints3D(const RagdollCapsuleDefinition3D& collider);

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
    // Raio na ponta -X local. Com radiusAtPositiveXMeters > 0 a cápsula é
    // cônica (coxa grossa no quadril e fina no joelho, por exemplo): as
    // tampas são esferas desses dois raios. Zero = cápsula uniforme.
    float radiusMeters = 0.05f;
    float radiusAtPositiveXMeters = 0.0f;
    // Usado quando shape == Box. Mantemos o mesmo container para não
    // fragmentar o contrato do perfil; cada shape valida apenas seus campos.
    // Numa casca convexa, a caixa justa dela (o leitor a calcula e centra).
    Vec3 boxHalfExtents { 0.05f, 0.05f, 0.05f };
    // Usados quando shape == ConvexHull: os vertices e os triangulos (indices),
    // no referencial do colisor; os planos das faces (normal para fora e o
    // deslocamento, n.p <= d dentro), que o leitor calcula; e o arredondamento
    // das quinas na fisica (m; as faces ficam onde estao).
    std::vector<Vec3> hullPoints;
    std::vector<std::array<std::uint16_t, 3>> hullFaces;
    std::vector<std::array<float, 4>> hullPlanes;
    std::vector<Vec3> hullSole;
    float convexRadiusMeters = 0.005f;
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

// Par de links que nao colide entre si, alem das regras automaticas do
// backend (pai-filho, sobrepostos no repouso, mesma cadeia a menos de 1 cm).
// Existe para contatos falsos que a forma aproximada dos colisores cria
// dentro da amplitude normal do corpo; o motivo fica gravado no perfil.
struct RagdollIgnoredCollisionPair3D {
    std::string firstLinkId;
    std::string secondLinkId;
    std::string reason;
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
    std::vector<RagdollIgnoredCollisionPair3D> selfCollisionIgnoredPairs;
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
// Uma casca convexa montada fora do arquivo (os pontos e as faces no
// referencial do colisor): centra na caixa justa (o colisor desloca junto) e
// calcula os planos das faces e o contorno da sola, como o leitor faz.
void prepareRagdollConvexHull3D(RagdollCapsuleDefinition3D& collider);

} // namespace MatterEngine
