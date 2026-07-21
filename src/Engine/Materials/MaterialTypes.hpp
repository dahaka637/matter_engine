#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace MatterEngine {

// Propriedades usadas pelo backend quando duas superficies entram em contato.
// Os coeficientes continuam intrinsecos ao material; o PhysX combina o par
// conforme os modos configurados pela cena.
struct ContactMaterialProperties {
    float staticFriction = 0.60f;
    float dynamicFriction = 0.48f;
    float rollingFriction = 0.015f;
    float restitution = 0.10f;
};

// Propriedades estruturais do material macico. A resistencia final de uma
// peca ainda depende de sua espessura, geometria, orientacao e defeitos.
struct StructuralMaterialProperties {
    float hardnessMegaPascals = 50.0f;
    float penetrationResistanceJoulesPerMeter = 2500.0f;
    float fractureEnergyJoulesPerSquareMeter = 500.0f;
};

// Perfil acustico independente dos arquivos de audio. Os IDs abaixo apontam
// para conjuntos logicos; futuramente o catalogo de audio podera associar
// varias gravacoes a cada conjunto sem contaminar a simulacao fisica.
struct AcousticMaterialProperties {
    std::string impactSoundSet = "material.generic.impact";
    std::string footstepSoundSet = "material.generic.footstep";
    std::string rollingSoundSet = "material.generic.roll";
    std::string fractureSoundSet = "material.generic.fracture";

    // Um impacto precisa superar os dois limiares. O limiar especifico
    // (energia por massa do corpo excitado) impede que uma garrafa leve faca
    // uma bigorna macica soar como se ela tivesse recebido uma marretada.
    float minimumImpactEnergyJoules = 0.025f;
    float minimumSpecificImpactEnergyJoulesPerKg = 0.020f;
    float acousticEfficiency = 0.30f;
    float staticBodyResponse = 0.20f;
    float absorption = 0.15f;
    float internalDamping = 0.30f;

    // O pitch final tambem considera tamanho e energia do objeto. Estes
    // valores descrevem apenas a assinatura relativa do material.
    float referencePitch = 1.0f;
    float sizePitchExponent = 0.18f;
    float energyPitchShiftOctaves = 0.035f;
    float pitchVariation = 0.035f;
    float softImpactMuffle = 0.40f;
};

// Extensao reservada a materiais que armazenam e devolvem energia por
// deformacao. O solver rigido ainda nao consome estes campos; mante-los
// tipados evita que a futura bola seja implementada com chaves soltas.
struct DeformationMaterialProperties {
    bool enabled = false;
    bool inflated = false;
    float complianceMetersPerNewton = 0.0f;
    float maximumCompressionRatio = 0.0f;
    float recoveryDampingRatio = 0.0f;
    float internalPressureKiloPascals = 0.0f;
};

enum class MaterialClass : std::uint8_t {
    Homogeneous,
    Granular,
    Organic,
    Composite
};

// Definicao canonica de um material da engine. Massa e peso nao sao gravados
// aqui: massa = densidade * volume do collider e peso = massa * gravidade.
struct SurfaceMaterial {
    std::string id;
    std::string displayName;
    MaterialClass materialClass = MaterialClass::Homogeneous;
    float densityKgPerCubicMeter = 1000.0f;
    ContactMaterialProperties contact;
    StructuralMaterialProperties structural;
    AcousticMaterialProperties acoustic;
    DeformationMaterialProperties deformation;

    // Ajuste pequeno da rugosidade superficial sobre o arrasto calculado
    // pela forma. O coeficiente aerodinamico principal pertence ao corpo.
    float aerodynamicDragScale = 1.0f;

    // Extras ainda nao modelados permanecem preservados para que uma versao
    // nova do importador nao destrua dados autorados em Blender.
    std::vector<std::pair<std::string, std::string>> customProperties;

    [[nodiscard]] const std::string* findCustomProperty(
        std::string_view key) const {
        for (const auto& entry : customProperties) {
            if (entry.first == key) return &entry.second;
        }
        return nullptr;
    }

    [[nodiscard]] float massForVolume(float cubicMeters) const {
        return densityKgPerCubicMeter * cubicMeters;
    }
};

} // namespace MatterEngine
