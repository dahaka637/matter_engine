#include "Engine/Materials/MaterialLibrary.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace MatterEngine {
namespace {

bool finiteNonNegative(float value) {
    return std::isfinite(value) && value >= 0.0f;
}

bool finiteNormalized(float value) {
    return finiteNonNegative(value) && value <= 1.0f;
}

void validate(const SurfaceMaterial& material) {
    const ContactMaterialProperties& contact = material.contact;
    const StructuralMaterialProperties& structural = material.structural;
    const AcousticMaterialProperties& acoustic = material.acoustic;
    const DeformationMaterialProperties& deformation = material.deformation;
    if (material.id.empty()) {
        throw std::invalid_argument("Material id cannot be empty");
    }
    if (!finiteNonNegative(material.densityKgPerCubicMeter)
        || material.densityKgPerCubicMeter <= 0.0f
        || !finiteNonNegative(contact.staticFriction)
        || !finiteNonNegative(contact.dynamicFriction)
        || !finiteNonNegative(contact.rollingFriction)
        || !finiteNormalized(contact.restitution)
        || !finiteNonNegative(structural.hardnessMegaPascals)
        || !finiteNonNegative(
            structural.penetrationResistanceJoulesPerMeter)
        || !finiteNonNegative(
            structural.fractureEnergyJoulesPerSquareMeter)
        || !finiteNonNegative(acoustic.minimumImpactEnergyJoules)
        || !finiteNonNegative(
            acoustic.minimumSpecificImpactEnergyJoulesPerKg)
        || !finiteNormalized(acoustic.acousticEfficiency)
        || !finiteNormalized(acoustic.staticBodyResponse)
        || !finiteNormalized(acoustic.absorption)
        || !finiteNormalized(acoustic.internalDamping)
        || !finiteNonNegative(acoustic.referencePitch)
        || !finiteNonNegative(acoustic.sizePitchExponent)
        || !std::isfinite(acoustic.energyPitchShiftOctaves)
        || !finiteNonNegative(acoustic.pitchVariation)
        || !finiteNormalized(acoustic.softImpactMuffle)
        || !finiteNonNegative(material.aerodynamicDragScale)
        || !finiteNonNegative(deformation.complianceMetersPerNewton)
        || !finiteNormalized(deformation.maximumCompressionRatio)
        || !finiteNormalized(deformation.recoveryDampingRatio)
        || !finiteNonNegative(deformation.internalPressureKiloPascals)) {
        throw std::invalid_argument(
            "Material properties must be finite and physically valid");
    }
    if (contact.dynamicFriction > contact.staticFriction) {
        throw std::invalid_argument(
            "Dynamic friction cannot exceed static friction");
    }
    if (acoustic.referencePitch <= 0.0f) {
        throw std::invalid_argument("Acoustic reference pitch must be positive");
    }
    if (acoustic.impactSoundSet.empty()) {
        throw std::invalid_argument("Impact sound set cannot be empty");
    }
    if (deformation.inflated && !deformation.enabled) {
        throw std::invalid_argument(
            "An inflated material must enable deformation");
    }
}

SurfaceMaterial baseMaterial(std::string id, float density,
    MaterialClass materialClass = MaterialClass::Homogeneous) {
    SurfaceMaterial result;
    result.id = std::move(id);
    result.displayName = result.id;
    result.materialClass = materialClass;
    result.densityKgPerCubicMeter = density;
    const std::string prefix = "material." + result.id;
    result.acoustic.impactSoundSet = prefix + ".impact";
    result.acoustic.footstepSoundSet = prefix + ".footstep";
    result.acoustic.rollingSoundSet = prefix + ".roll";
    result.acoustic.fractureSoundSet = prefix + ".fracture";
    return result;
}

SurfaceMaterial makeDefault() {
    return baseMaterial("default", 1000.0f);
}

SurfaceMaterial makeSand() {
    SurfaceMaterial value = baseMaterial(
        "sand", 1600.0f, MaterialClass::Granular);
    value.contact = { 0.95f, 0.78f, 0.16f, 0.01f };
    value.structural = { 0.15f, 180.0f, 45.0f };
    value.acoustic.minimumImpactEnergyJoules = 0.06f;
    value.acoustic.minimumSpecificImpactEnergyJoulesPerKg = 0.035f;
    value.acoustic.acousticEfficiency = 0.10f;
    value.acoustic.staticBodyResponse = 0.08f;
    value.acoustic.absorption = 0.88f;
    value.acoustic.internalDamping = 0.92f;
    value.acoustic.referencePitch = 0.76f;
    value.acoustic.softImpactMuffle = 0.92f;
    value.aerodynamicDragScale = 1.05f;
    return value;
}

SurfaceMaterial makeConcrete() {
    SurfaceMaterial value = baseMaterial("concrete", 2400.0f);
    value.contact = { 0.72f, 0.58f, 0.024f, 0.05f };
    value.structural = { 85.0f, 18000.0f, 120.0f };
    value.acoustic.minimumImpactEnergyJoules = 0.035f;
    value.acoustic.minimumSpecificImpactEnergyJoulesPerKg = 0.018f;
    value.acoustic.acousticEfficiency = 0.42f;
    value.acoustic.staticBodyResponse = 0.22f;
    value.acoustic.absorption = 0.08f;
    value.acoustic.internalDamping = 0.38f;
    value.acoustic.referencePitch = 0.82f;
    return value;
}

SurfaceMaterial makeGrass() {
    SurfaceMaterial value = baseMaterial(
        "grass", 1050.0f, MaterialClass::Organic);
    value.contact = { 0.88f, 0.70f, 0.075f, 0.015f };
    value.structural = { 0.4f, 350.0f, 180.0f };
    value.acoustic.minimumImpactEnergyJoules = 0.055f;
    value.acoustic.minimumSpecificImpactEnergyJoulesPerKg = 0.040f;
    value.acoustic.acousticEfficiency = 0.12f;
    value.acoustic.staticBodyResponse = 0.10f;
    value.acoustic.absorption = 0.82f;
    value.acoustic.internalDamping = 0.88f;
    value.acoustic.referencePitch = 0.84f;
    value.acoustic.softImpactMuffle = 0.88f;
    value.aerodynamicDragScale = 1.10f;
    return value;
}

SurfaceMaterial makePlastic() {
    SurfaceMaterial value = baseMaterial("plastic", 950.0f);
    value.contact = { 0.48f, 0.34f, 0.022f, 0.24f };
    value.structural = { 24.0f, 4800.0f, 820.0f };
    value.acoustic.minimumImpactEnergyJoules = 0.018f;
    value.acoustic.minimumSpecificImpactEnergyJoulesPerKg = 0.018f;
    value.acoustic.acousticEfficiency = 0.38f;
    value.acoustic.staticBodyResponse = 0.16f;
    value.acoustic.absorption = 0.28f;
    value.acoustic.internalDamping = 0.55f;
    value.acoustic.referencePitch = 1.18f;
    value.acoustic.sizePitchExponent = 0.22f;
    return value;
}

SurfaceMaterial makeBrick() {
    SurfaceMaterial value = baseMaterial("brick", 1900.0f);
    value.contact = { 0.76f, 0.61f, 0.030f, 0.045f };
    value.structural = { 32.0f, 10500.0f, 95.0f };
    value.acoustic.minimumImpactEnergyJoules = 0.035f;
    value.acoustic.minimumSpecificImpactEnergyJoulesPerKg = 0.020f;
    value.acoustic.acousticEfficiency = 0.36f;
    value.acoustic.staticBodyResponse = 0.18f;
    value.acoustic.absorption = 0.12f;
    value.acoustic.internalDamping = 0.46f;
    value.acoustic.referencePitch = 0.90f;
    return value;
}

SurfaceMaterial makeWood() {
    SurfaceMaterial value = baseMaterial(
        "wood", 700.0f, MaterialClass::Organic);
    value.contact = { 0.58f, 0.42f, 0.028f, 0.18f };
    value.structural = { 35.0f, 6200.0f, 520.0f };
    value.acoustic.minimumImpactEnergyJoules = 0.020f;
    value.acoustic.minimumSpecificImpactEnergyJoulesPerKg = 0.015f;
    value.acoustic.acousticEfficiency = 0.52f;
    value.acoustic.staticBodyResponse = 0.28f;
    value.acoustic.absorption = 0.24f;
    value.acoustic.internalDamping = 0.40f;
    value.acoustic.referencePitch = 0.94f;
    value.acoustic.sizePitchExponent = 0.24f;
    return value;
}

SurfaceMaterial makeSteel() {
    SurfaceMaterial value = baseMaterial("steel", 7850.0f);
    value.contact = { 0.62f, 0.46f, 0.012f, 0.12f };
    value.structural = { 210.0f, 42000.0f, 1400.0f };
    value.acoustic.minimumImpactEnergyJoules = 0.025f;
    value.acoustic.minimumSpecificImpactEnergyJoulesPerKg = 0.010f;
    value.acoustic.acousticEfficiency = 0.72f;
    value.acoustic.staticBodyResponse = 0.12f;
    value.acoustic.absorption = 0.03f;
    value.acoustic.internalDamping = 0.12f;
    value.acoustic.referencePitch = 1.12f;
    value.acoustic.sizePitchExponent = 0.28f;
    return value;
}

SurfaceMaterial makeRubber() {
    SurfaceMaterial value = baseMaterial("rubber", 1100.0f);
    value.contact = { 1.15f, 0.92f, 0.045f, 0.72f };
    value.structural = { 7.0f, 4200.0f, 1800.0f };
    value.acoustic.minimumImpactEnergyJoules = 0.030f;
    value.acoustic.minimumSpecificImpactEnergyJoulesPerKg = 0.030f;
    value.acoustic.acousticEfficiency = 0.18f;
    value.acoustic.staticBodyResponse = 0.10f;
    value.acoustic.absorption = 0.65f;
    value.acoustic.internalDamping = 0.76f;
    value.acoustic.referencePitch = 0.78f;
    value.acoustic.softImpactMuffle = 0.78f;
    return value;
}

SurfaceMaterial makeSoccerBall() {
    // 78 kg/m3 e uma densidade efetiva do conjunto oco. Em uma esfera de
    // raio 11 cm ela resulta em aproximadamente 435 g, dentro da faixa de
    // uma bola oficial, sem fingir que o interior e borracha macica.
    SurfaceMaterial value = baseMaterial(
        "soccer_ball", 78.0f, MaterialClass::Composite);
    value.contact = { 0.78f, 0.60f, 0.018f, 0.68f };
    value.structural = { 12.0f, 3200.0f, 1650.0f };
    value.acoustic.minimumImpactEnergyJoules = 0.012f;
    value.acoustic.minimumSpecificImpactEnergyJoulesPerKg = 0.022f;
    value.acoustic.acousticEfficiency = 0.34f;
    value.acoustic.staticBodyResponse = 0.08f;
    value.acoustic.absorption = 0.46f;
    value.acoustic.internalDamping = 0.44f;
    value.acoustic.referencePitch = 1.0f;
    value.acoustic.sizePitchExponent = 0.12f;
    value.acoustic.softImpactMuffle = 0.58f;
    value.deformation = {
        true, true, 0.000018f, 0.16f, 0.24f, 80.0f
    };
    value.aerodynamicDragScale = 0.95f;
    return value;
}

SurfaceMaterial makeGlass() {
    SurfaceMaterial value = baseMaterial("glass", 2500.0f);
    value.contact = { 0.48f, 0.34f, 0.012f, 0.08f };
    value.structural = { 550.0f, 8500.0f, 12.0f };
    value.acoustic.acousticEfficiency = 0.78f;
    value.acoustic.absorption = 0.04f;
    value.acoustic.internalDamping = 0.08f;
    value.acoustic.referencePitch = 1.32f;
    return value;
}

SurfaceMaterial makeSoil() {
    SurfaceMaterial value = baseMaterial(
        "soil", 1600.0f, MaterialClass::Granular);
    value.contact = { 0.86f, 0.72f, 0.11f, 0.01f };
    value.structural = { 2.0f, 900.0f, 260.0f };
    value.acoustic.acousticEfficiency = 0.10f;
    value.acoustic.absorption = 0.72f;
    value.acoustic.internalDamping = 0.84f;
    value.acoustic.referencePitch = 0.78f;
    return value;
}

} // namespace

MaterialLibrary::MaterialLibrary() {
    registerStandardMaterials();
}

SurfaceMaterial& MaterialLibrary::registerMaterial(
    SurfaceMaterial materialDefinition) {
    validate(materialDefinition);
    const std::string id = materialDefinition.id;
    auto [iterator, inserted] = m_materials.insert_or_assign(
        id, std::move(materialDefinition));
    static_cast<void>(inserted);
    return iterator->second;
}

const SurfaceMaterial* MaterialLibrary::find(const std::string& id) const {
    const auto iterator = m_materials.find(id);
    return iterator != m_materials.end() ? &iterator->second : nullptr;
}

void MaterialLibrary::registerStandardMaterials() {
    registerMaterial(makeDefault());
    registerMaterial(makeSand());
    registerMaterial(makeConcrete());
    registerMaterial(makeGrass());
    registerMaterial(makePlastic());
    registerMaterial(makeBrick());
    registerMaterial(makeWood());
    registerMaterial(makeSteel());
    registerMaterial(makeRubber());
    registerMaterial(makeSoccerBall());

    // Mantidos por compatibilidade com assets existentes. Eles seguem a
    // mesma estrutura nova e podem ser refinados junto do catalogo principal.
    registerMaterial(makeGlass());
    registerMaterial(makeSoil());
}

} // namespace MatterEngine
