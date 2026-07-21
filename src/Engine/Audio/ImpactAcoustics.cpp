#include "Engine/Audio/ImpactAcoustics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace MatterEngine {
namespace {

constexpr float MinimumMass = 0.001f;
constexpr float ReferenceAcousticSizeMeters = 0.50f;

float structureGain(AcousticBodyStructure3D structure) {
    switch (structure) {
    case AcousticBodyStructure3D::Hollow: return 1.28f;
    case AcousticBodyStructure3D::ThinShell: return 1.45f;
    case AcousticBodyStructure3D::Soft: return 0.58f;
    case AcousticBodyStructure3D::Inflated: return 1.16f;
    case AcousticBodyStructure3D::Solid: return 1.0f;
    }
    return 1.0f;
}

std::uint64_t stableBodyKey(std::uint64_t bodyId, std::size_t bodyIndex) {
    if (bodyId != 0) return bodyId;
    if (bodyIndex == InvalidPhysicsBodyIndex) return 0;
    // O bit alto separa o fallback por indice dos IDs persistentes pequenos.
    return (std::uint64_t { 1 } << 63)
        | static_cast<std::uint64_t>(bodyIndex + 1);
}

std::uint64_t hashText(std::string_view text) {
    std::uint64_t hash = 1469598103934665603ull;
    for (const char character : text) {
        hash ^= static_cast<std::uint8_t>(character);
        hash *= 1099511628211ull;
    }
    return hash;
}

float deterministicSignedVariation(std::uint64_t value) {
    value ^= value >> 33;
    value *= 0xff51afd7ed558ccdull;
    value ^= value >> 33;
    const float normalized = static_cast<float>(value & 0xFFFFu)
        / 65535.0f;
    return normalized * 2.0f - 1.0f;
}

ImpactIntensity3D intensityForVolume(float volume) {
    if (volume >= 0.76f) return ImpactIntensity3D::Hard;
    if (volume >= 0.36f) return ImpactIntensity3D::Medium;
    return ImpactIntensity3D::Soft;
}

} // namespace

std::size_t ImpactAcousticResolver::EmissionKeyHash::operator()(
    const EmissionKey& key) const noexcept {
    std::uint64_t value = key.pairA;
    value ^= key.pairB + 0x9e3779b97f4a7c15ull
        + (value << 6) + (value >> 2);
    value ^= key.source + 0x9e3779b97f4a7c15ull
        + (value << 6) + (value >> 2);
    return static_cast<std::size_t>(value);
}

void ImpactAcousticResolver::resolve(
    std::span<const ContactImpactEvent3D> impacts,
    const MaterialLibrary& materials, float deltaTime) {
    m_commands.clear();
    m_timeSeconds += std::max(0.0f, deltaTime);

    const auto emitBody = [&](const ContactImpactEvent3D& impact,
            bool emitA) {
        // A energia isolada nao prova que houve uma pancada: corpos muito
        // pesados podem acumular energia em um contato lento. A velocidade
        // minima e, portanto, uma porta obrigatoria antes de qualquer
        // avaliacao por material. Isso elimina sons em acomodacoes, repouso
        // e pequenos deslizamentos sem mascarar impactos de verdade.
        constexpr float MinimumAudibleImpactSpeed = 0.90f;
        if (impact.approachSpeedMetersPerSecond
            < MinimumAudibleImpactSpeed) {
            return;
        }

        const std::string& materialId = emitA
            ? impact.materialA : impact.materialB;
        const SurfaceMaterial* material = materials.find(materialId);
        if (material == nullptr) return;
        const AcousticMaterialProperties& acoustic = material->acoustic;
        const float bodyMass = emitA ? impact.massA : impact.massB;
        const bool staticBody = emitA ? impact.staticA : impact.staticB;
        const float bodyGain = emitA
            ? impact.acousticGainA : impact.acousticGainB;
        const float dampingScale = emitA
            ? impact.acousticDampingA : impact.acousticDampingB;
        const float size = std::max(MinimumMass,
            emitA ? impact.characteristicSizeA
                : impact.characteristicSizeB);
        const AcousticBodyStructure3D structure = emitA
            ? impact.structureA : impact.structureB;

        // Corpos estaticos sem massa autorada usam uma massa local virtual:
        // somente a regiao proxima ao contato participa da primeira vibracao.
        // Um corpo estatico com massa conhecida (por exemplo, uma bigorna)
        // conserva sua massa real e recebe excitacao especifica muito menor.
        const float participatingMass = bodyMass > MinimumMass
            ? bodyMass
            : std::max(MinimumMass, impact.effectiveMassKg * 6.0f);
        const float specificEnergy = impact.transferredEnergyJoules
            / participatingMass;
        const float mobility = std::clamp(std::sqrt(
            std::max(MinimumMass, impact.effectiveMassKg)
                / participatingMass), 0.015f, 1.0f);
        const float staticResponse = staticBody
            ? acoustic.staticBodyResponse : 1.0f;
        const float emittedEnergy = impact.transferredEnergyJoules
            * acoustic.acousticEfficiency * mobility * staticResponse
            * structureGain(structure) * std::max(0.0f, bodyGain);
        if (emittedEnergy < acoustic.minimumImpactEnergyJoules
            || specificEnergy
                < acoustic.minimumSpecificImpactEnergyJoulesPerKg) {
            return;
        }

        const std::uint64_t sourceKey = stableBodyKey(
            emitA ? impact.bodyIdA : impact.bodyIdB,
            emitA ? impact.bodyA : impact.bodyB);
        const std::uint64_t otherKey = stableBodyKey(
            emitA ? impact.bodyIdB : impact.bodyIdA,
            emitA ? impact.bodyB : impact.bodyA);
        const EmissionKey emissionKey {
            std::min(sourceKey, otherKey),
            std::max(sourceKey, otherKey),
            sourceKey
        };
        const auto cooldown = m_nextAllowedTime.find(emissionKey);
        if (cooldown != m_nextAllowedTime.end()
            && cooldown->second > m_timeSeconds) {
            return;
        }

        // Duas grandezas independentes controlam a audibilidade: energia
        // emitida e velocidade de aproximacao. Isso impede que acomodacoes
        // lentas de corpos pesados soem como pancadas, mantendo impactos
        // realmente fortes progressivamente mais altos.
        const float normalizedSpeed = std::clamp(
            (impact.approachSpeedMetersPerSecond
                - MinimumAudibleImpactSpeed) / 5.2f,
            0.0f, 1.0f);
        const float speedGate = normalizedSpeed * normalizedSpeed
            * (3.0f - 2.0f * normalizedSpeed);
        const float energyVolume = 1.0f - std::exp(
            -0.38f * std::sqrt(emittedEnergy));
        const float volume = std::clamp(
            energyVolume * (0.05f + 0.95f * speedGate), 0.0f, 1.0f);
        if (volume < 0.012f) return;
        const float threshold = std::max(
            acoustic.minimumSpecificImpactEnergyJoulesPerKg, 0.0001f);
        const float energyOctaves = acoustic.energyPitchShiftOctaves
            * std::log2(1.0f + specificEnergy / threshold);
        const float sizeScale = std::pow(
            ReferenceAcousticSizeMeters / size,
            acoustic.sizePitchExponent);
        const float variation = deterministicSignedVariation(
            sourceKey ^ otherKey ^ hashText(materialId));
        const float pitch = std::clamp(acoustic.referencePitch
            * sizeScale * std::exp2(energyOctaves)
            * (1.0f + variation * acoustic.pitchVariation),
            0.45f, 2.25f);
        const float muffle = std::clamp(
            acoustic.absorption * 0.56f
            + acoustic.internalDamping * dampingScale * 0.28f
            + (1.0f - volume) * acoustic.softImpactMuffle * 0.40f,
            0.0f, 0.96f);

        ImpactSoundCommand3D command;
        command.sourceBodyId = sourceKey;
        command.materialId = materialId;
        command.soundSet = acoustic.impactSoundSet;
        command.position = impact.position;
        command.intensity = intensityForVolume(volume);
        command.volume = volume;
        command.pitch = pitch;
        command.muffle = muffle;
        command.priority = volume * (1.0f
            + 0.12f * std::log1p(impact.transferredEnergyJoules));
        command.emittedEnergyJoules = emittedEnergy;
        command.durationSeconds = std::clamp(
            0.055f + size * 0.16f
                + std::log1p(emittedEnergy) * 0.055f,
            0.07f, 0.85f)
            * std::clamp(1.18f - acoustic.internalDamping
                * dampingScale * 0.62f, 0.42f, 1.18f);
        m_commands.push_back(std::move(command));

        const float cooldownSeconds = volume >= 0.76f ? 0.075f
            : volume >= 0.36f ? 0.125f : 0.210f;
        m_nextAllowedTime[emissionKey] = m_timeSeconds + cooldownSeconds;
    };

    for (const ContactImpactEvent3D& impact : impacts) {
        emitBody(impact, true);
        emitBody(impact, false);
    }
    std::sort(m_commands.begin(), m_commands.end(),
        [](const ImpactSoundCommand3D& left,
            const ImpactSoundCommand3D& right) {
            return left.priority > right.priority;
        });

    // IDs removidos nao devem deixar o mapa crescer para sempre em sessoes
    // longas de editor. Dois segundos cobrem com folga todos os cooldowns.
    for (auto iterator = m_nextAllowedTime.begin();
        iterator != m_nextAllowedTime.end();) {
        if (iterator->second + 2.0f < m_timeSeconds) {
            iterator = m_nextAllowedTime.erase(iterator);
        } else {
            ++iterator;
        }
    }
}

void ImpactAcousticResolver::reset() {
    m_commands.clear();
    m_nextAllowedTime.clear();
    m_timeSeconds = 0.0f;
}

} // namespace MatterEngine
