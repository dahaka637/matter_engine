#pragma once

#include "Engine/Materials/MaterialTypes.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <optional>

namespace MatterEngine {

enum class BodyMassMode3D {
    AutomaticFromDensity,
    OverrideKilograms
};

// Overrides que pertencem ao objeto. O material continua sendo a fonte de
// densidade, contato e assinatura acustica; massa explicita representa o asset
// montado (por exemplo cadeira oca, barril ou bola inflada).
struct BodyMaterialBinding3D {
    BodyMassMode3D massMode = BodyMassMode3D::AutomaticFromDensity;
    float massOverrideKg = 0.0f;
    bool enableAerodynamicDrag = true;
    std::optional<float> dragCoefficientOverride;
    std::optional<float> referenceAreaOverrideSquareMeters;
    std::optional<AcousticBodyStructure3D> acousticStructureOverride;
    float acousticGain = 1.0f;
    float acousticDampingScale = 1.0f;
};

[[nodiscard]] AcousticBodyStructure3D defaultAcousticStructure(
    const SurfaceMaterial& material);

// Consolida material + metadados + geometria em um descriptor pronto. Essa
// etapa roda uma vez por asset; o loop de simulacao nao consulta strings nem
// recalcula massa/dimensoes.
[[nodiscard]] PhysicsBodyDefinition3D buildDynamicBodyDefinition(
    const SurfaceMaterial& material,
    const BodyMaterialBinding3D& binding,
    float collisionVolumeCubicMeters,
    Vec3 dimensionsMeters);

} // namespace MatterEngine
