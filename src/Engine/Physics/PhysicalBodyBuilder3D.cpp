#include "Engine/Physics/PhysicalBodyBuilder3D.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace MatterEngine {
namespace {

constexpr float MinimumPhysicalValue = 1.0e-6f;

} // namespace

AcousticBodyStructure3D defaultAcousticStructure(
    const SurfaceMaterial& material) {
    if (material.deformation.inflated) {
        return AcousticBodyStructure3D::Inflated;
    }
    if (material.materialClass == MaterialClass::Granular
        || material.id == "grass" || material.id == "rubber") {
        return AcousticBodyStructure3D::Soft;
    }
    return AcousticBodyStructure3D::Solid;
}

PhysicsBodyDefinition3D buildDynamicBodyDefinition(
    const SurfaceMaterial& material,
    const BodyMaterialBinding3D& binding,
    float collisionVolumeCubicMeters,
    Vec3 dimensionsMeters) {
    if (!std::isfinite(collisionVolumeCubicMeters)
        || collisionVolumeCubicMeters < 0.0f
        || dimensionsMeters.x <= MinimumPhysicalValue
        || dimensionsMeters.y <= MinimumPhysicalValue
        || dimensionsMeters.z <= MinimumPhysicalValue) {
        throw std::invalid_argument(
            "Geometria invalida ao construir propriedades do corpo");
    }

    float mass = material.massForVolume(collisionVolumeCubicMeters);
    if (binding.massMode == BodyMassMode3D::OverrideKilograms) {
        mass = binding.massOverrideKg;
    }
    if (!std::isfinite(mass) || mass <= MinimumPhysicalValue) {
        throw std::invalid_argument(
            "Massa automatica exige volume fechado ou override positivo");
    }
    if (!std::isfinite(binding.acousticGain)
        || binding.acousticGain < 0.0f
        || !std::isfinite(binding.acousticDampingScale)
        || binding.acousticDampingScale < 0.0f) {
        throw std::invalid_argument("Override acustico invalido");
    }

    const float largestDimension = std::max({ dimensionsMeters.x,
        dimensionsMeters.y, dimensionsMeters.z });
    const float projectedArea = std::max({
        dimensionsMeters.x * dimensionsMeters.y,
        dimensionsMeters.x * dimensionsMeters.z,
        dimensionsMeters.y * dimensionsMeters.z });
    PhysicsBodyDefinition3D definition;
    definition.massKg = mass;
    definition.materialId = material.id;
    definition.characteristicSizeMeters = largestDimension;
    definition.acousticGain = binding.acousticGain;
    definition.acousticDamping = binding.acousticDampingScale;
    definition.acousticStructure =
        binding.acousticStructureOverride.value_or(
            defaultAcousticStructure(material));
    definition.aerodynamicDragEnabled = binding.enableAerodynamicDrag;
    definition.aerodynamicDragCoefficient =
        binding.dragCoefficientOverride.value_or(
            1.05f * material.aerodynamicDragScale);
    definition.aerodynamicReferenceAreaSquareMeters =
        binding.referenceAreaOverrideSquareMeters.value_or(projectedArea);
    // A bola combina raio pequeno e velocidades altas de chute; nela CCD e
    // parte da fidelidade do gameplay. Props convencionais permanecem no
    // caminho discreto mais barato, adequado ao passo fixo de 120 Hz.
    definition.collisionMode = material.id == "soccer_ball"
        ? PhysicsCollisionMode3D::Continuous
        : PhysicsCollisionMode3D::Discrete;
    if (!std::isfinite(definition.aerodynamicDragCoefficient)
        || definition.aerodynamicDragCoefficient < 0.0f
        || !std::isfinite(
            definition.aerodynamicReferenceAreaSquareMeters)
        || definition.aerodynamicReferenceAreaSquareMeters < 0.0f) {
        throw std::invalid_argument("Override aerodinamico invalido");
    }
    return definition;
}

} // namespace MatterEngine
