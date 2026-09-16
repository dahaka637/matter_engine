#include "Workbench/WorkbenchApp.hpp"

#include "Engine/Core/Log.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_set>

namespace MatterEngine::Workbench {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float BenchmarkMinimumHeightMeters = 80.0f;
constexpr float BenchmarkMaximumHeightMeters = 100.0f;
constexpr float BenchmarkMinimumRadiusMeters = 5.0f;
constexpr float BenchmarkMaximumRadiusMeters = 42.0f;
constexpr std::uint32_t MaximumPlacementAttempts = 256;
constexpr std::size_t MaximumAnalyticsSamples = 120;
constexpr float AnalyticsWindowSeconds = 5.0f;
const Vec3 BenchmarkRagdollHalfExtents { 0.31f, 0.84f, 0.91f };
const Vec3 BenchmarkRagdollCenterOffset { 0.0f, 0.0f, -0.04f };

float nextRandomUnit(std::uint32_t& state) {
    // Xorshift32: sequência reproduzível, barata e suficiente para distribuir
    // entidades. O benchmark não depende de entropia do sistema operacional.
    state ^= state << 13u;
    state ^= state >> 17u;
    state ^= state << 5u;
    return static_cast<float>(state & 0x00FFFFFFu)
        / static_cast<float>(0x01000000u);
}

Vec3 randomBenchmarkPosition(std::uint32_t& randomState) {
    // sqrt(U) torna a densidade uniforme na área do disco (U puro concentraria
    // entidades demais no centro). A faixa vertical é exatamente 80–100 m.
    const float radius = BenchmarkMinimumRadiusMeters
        + std::sqrt(nextRandomUnit(randomState))
            * (BenchmarkMaximumRadiusMeters
                - BenchmarkMinimumRadiusMeters);
    const float angle = nextRandomUnit(randomState) * 2.0f * Pi;
    return {
        std::cos(angle) * radius,
        std::sin(angle) * radius,
        BenchmarkMinimumHeightMeters
            + nextRandomUnit(randomState)
                * (BenchmarkMaximumHeightMeters
                    - BenchmarkMinimumHeightMeters)
    };
}

} // namespace

void WorkbenchApp::beginPhysicsBenchmark() {
    if (!m_physicsScene || !m_propCatalog.loaded()
        || !m_humanRagdollProfile || !m_laboratoryMapLoaded) {
        m_notification = {
            "Benchmark indisponivel enquanto os assets carregam",
            2.4f, 2.4f
        };
        return;
    }
    if (m_physicsBenchmarkPaused
        && (m_physicsBenchmarkSpawnedProps
                < static_cast<std::uint32_t>(
                    m_physicsBenchmarkRequestedProps)
            || m_physicsBenchmarkSpawnedRagdolls
                < static_cast<std::uint32_t>(
                    m_physicsBenchmarkRequestedRagdolls))) {
        m_physicsBenchmarkPaused = false;
        m_physicsBenchmarkRunning = true;
        m_notification = { "Benchmark retomado", 1.6f, 1.6f };
        return;
    }

    if (!m_physicsBenchmarkPropEntityIds.empty()
        || !m_physicsBenchmarkRagdollEntityIds.empty()) {
        clearPhysicsBenchmark();
    }
    m_physicsBenchmarkRunning = true;
    m_physicsBenchmarkPaused = false;
    m_physicsBenchmarkSpawnAccumulator = 0.0f;
    m_physicsBenchmarkSpawnedProps = 0;
    m_physicsBenchmarkSpawnedRagdolls = 0;
    m_physicsBenchmarkRandomState = 0x4D415454u;
    m_physicsBenchmarkSamples.clear();
    m_physicsBenchmarkSampleSeconds = 0.0f;
    m_physicsBenchmarkFpsSum = 0.0f;
    m_physicsBenchmarkMinimumFps = 0.0f;
    m_physicsBenchmarkPhysicsMillisecondsSum = 0.0f;
    m_physicsBenchmarkMaximumPhysicsMilliseconds = 0.0f;
    m_physicsBenchmarkFrameSamples = 0;
    // ImGui mantém uma média móvel curta de FPS, então este valor representa
    // melhor o estado imediatamente anterior ao benchmark que 1/deltaTime.
    m_physicsBenchmarkBaselineFps = ImGui::GetIO().Framerate;
    m_physicsBenchmarkOverallFpsSum = 0.0f;
    m_physicsBenchmarkOverallMinimumFps = 0.0f;
    m_physicsBenchmarkOverallFrameSamples = 0;
    m_notification = { "Benchmark fisico iniciado", 1.8f, 1.8f };
}

void WorkbenchApp::pausePhysicsBenchmark() {
    if (!m_physicsBenchmarkRunning && !m_physicsBenchmarkPaused) return;
    m_physicsBenchmarkPaused = !m_physicsBenchmarkPaused;
    m_physicsBenchmarkRunning = !m_physicsBenchmarkPaused;
    m_notification = {
        m_physicsBenchmarkPaused ? "Benchmark pausado"
                                 : "Benchmark retomado",
        1.6f, 1.6f
    };
}

void WorkbenchApp::clearPhysicsBenchmark() {
    if (!m_physicsScene) return;
    const std::unordered_set<std::uint64_t> propIds(
        m_physicsBenchmarkPropEntityIds.begin(),
        m_physicsBenchmarkPropEntityIds.end());
    const std::unordered_set<std::uint64_t> ragdollIds(
        m_physicsBenchmarkRagdollEntityIds.begin(),
        m_physicsBenchmarkRagdollEntityIds.end());

    if (propIds.contains(m_physGunGrabbedEntityId)
        || ragdollIds.contains(m_physGunGrabbedEntityId)) {
        endPhysGunGrab();
    }
    for (const SpawnedPropInstance& instance : m_spawnedProps) {
        if (propIds.contains(instance.entityId)) {
            m_physicsScene->destroyBody(instance.physicsBody);
        }
    }
    std::erase_if(m_spawnedProps,
        [&](const SpawnedPropInstance& instance) {
            return propIds.contains(instance.entityId);
        });
    m_spawnedPropByBodyIndex.clear();
    for (std::size_t index = 0; index < m_spawnedProps.size(); ++index) {
        m_spawnedPropByBodyIndex.insert_or_assign(
            m_spawnedProps[index].physicsBody.index, index);
    }

    for (const SpawnedRagdollInstance& instance : m_spawnedRagdolls) {
        if (ragdollIds.contains(instance.entityId)) {
            m_physicsScene->destroyRagdoll(instance.physicsRagdoll);
        }
    }
    std::erase_if(m_spawnedRagdolls,
        [&](const SpawnedRagdollInstance& instance) {
            return ragdollIds.contains(instance.entityId);
        });

    m_physicsBenchmarkPropEntityIds.clear();
    m_physicsBenchmarkRagdollEntityIds.clear();
    m_physicsBenchmarkSpawnedProps = 0;
    m_physicsBenchmarkSpawnedRagdolls = 0;
    m_physicsBenchmarkSpawnAccumulator = 0.0f;
    m_physicsBenchmarkRunning = false;
    m_physicsBenchmarkPaused = false;
    m_notification = {
        "Entidades do benchmark removidas", 1.9f, 1.9f
    };
}

void WorkbenchApp::updatePhysicsBenchmark(float deltaTime) {
    if (!m_physicsBenchmarkRunning || m_physicsBenchmarkPaused
        || !m_physicsScene || !m_humanRagdollProfile) {
        return;
    }
    const std::uint32_t requestedProps =
        static_cast<std::uint32_t>(m_physicsBenchmarkRequestedProps);
    const std::uint32_t requestedRagdolls =
        static_cast<std::uint32_t>(m_physicsBenchmarkRequestedRagdolls);
    if (m_physicsBenchmarkSpawnedProps >= requestedProps
        && m_physicsBenchmarkSpawnedRagdolls >= requestedRagdolls) {
        m_physicsBenchmarkRunning = false;
        m_laboratoryStatus = "Benchmark: carga completa";
        return;
    }

    m_physicsBenchmarkSpawnAccumulator += std::max(1.0f,
        m_physicsBenchmarkSpawnRatePerSecond) * deltaTime;
    const std::uint32_t spawnBudget = std::min<std::uint32_t>(32u,
        static_cast<std::uint32_t>(
            std::floor(m_physicsBenchmarkSpawnAccumulator)));
    if (spawnBudget == 0) return;
    m_physicsBenchmarkSpawnAccumulator -=
        static_cast<float>(spawnBudget);

    const auto& definitions = m_propCatalog.definitions();
    for (std::uint32_t spawnIndex = 0; spawnIndex < spawnBudget;
            ++spawnIndex) {
        const std::uint32_t propsRemaining =
            requestedProps - m_physicsBenchmarkSpawnedProps;
        const std::uint32_t ragdollsRemaining =
            requestedRagdolls - m_physicsBenchmarkSpawnedRagdolls;
        if (propsRemaining == 0 && ragdollsRemaining == 0) break;

        const float ragdollShare = static_cast<float>(ragdollsRemaining)
            / static_cast<float>(propsRemaining + ragdollsRemaining);
        const bool spawnRagdoll = ragdollsRemaining > 0
            && (propsRemaining == 0
                || nextRandomUnit(m_physicsBenchmarkRandomState)
                    < ragdollShare);
        bool spawned = false;
        if (spawnRagdoll) {
            const Quaternion orientation = Quaternion::fromAxisAngle(
                { 0.0f, 0.0f, 1.0f },
                nextRandomUnit(m_physicsBenchmarkRandomState)
                    * 2.0f * Pi);
            for (std::uint32_t attempt = 0;
                    attempt < MaximumPlacementAttempts; ++attempt) {
                const Vec3 candidate = randomBenchmarkPosition(
                    m_physicsBenchmarkRandomState);
                if (m_physicsScene->overlapsBox(
                        candidate
                            + orientation.rotate(
                                BenchmarkRagdollCenterOffset),
                        BenchmarkRagdollHalfExtents
                            + Vec3 { 0.08f, 0.08f, 0.08f },
                        orientation)) {
                    continue;
                }
                spawned = spawnHumanRagdollAt(
                    candidate, orientation, true);
                if (spawned) break;
            }
            if (spawned) ++m_physicsBenchmarkSpawnedRagdolls;
        } else if (!definitions.empty()) {
            const std::size_t definitionIndex = std::min(
                definitions.size() - 1,
                static_cast<std::size_t>(
                    nextRandomUnit(m_physicsBenchmarkRandomState)
                        * static_cast<float>(definitions.size())));
            const PropDefinition3D& definition =
                definitions[definitionIndex];
            const Quaternion orientation = Quaternion::fromAxisAngle(
                { 0.0f, 0.0f, 1.0f },
                nextRandomUnit(m_physicsBenchmarkRandomState)
                    * 2.0f * Pi);
            for (std::uint32_t attempt = 0;
                    attempt < MaximumPlacementAttempts; ++attempt) {
                const Vec3 candidate = randomBenchmarkPosition(
                    m_physicsBenchmarkRandomState);
                if (m_physicsScene->overlapsBox(candidate,
                        definition.dimensionsMeters * 0.5f
                            + Vec3 { 0.06f, 0.06f, 0.06f },
                        orientation)) {
                    continue;
                }
                spawned = spawnPropAt(
                    definitionIndex, candidate, orientation, true);
                if (spawned) break;
            }
            if (spawned) ++m_physicsBenchmarkSpawnedProps;
        }
        if (!spawned) {
            // A taxa não acumula indefinidamente e não cria uma rajada de
            // centenas de atores quando uma vaga aparecer. Tenta novamente
            // no próximo intervalo, mantendo o frame responsivo.
            m_laboratoryStatus =
                "Benchmark procurando volume de spawn livre";
            break;
        }
    }
}

void WorkbenchApp::updatePhysicsBenchmarkAnalytics(float deltaTime) {
    if (!m_physicsBenchmarkShowAnalytics
        || (m_physicsBenchmarkPropEntityIds.empty()
            && m_physicsBenchmarkRagdollEntityIds.empty()
            && !m_physicsBenchmarkRunning)) {
        return;
    }
    const float fps = ImGui::GetIO().Framerate;
    const float physicsMilliseconds = m_physicsScene
        ? m_physicsScene->diagnostics().totalStepMilliseconds : 0.0f;
    m_physicsBenchmarkSampleSeconds += deltaTime;
    m_physicsBenchmarkFpsSum += fps;
    m_physicsBenchmarkPhysicsMillisecondsSum += physicsMilliseconds;
    m_physicsBenchmarkMaximumPhysicsMilliseconds = std::max(
        m_physicsBenchmarkMaximumPhysicsMilliseconds,
        physicsMilliseconds);
    if (m_physicsBenchmarkFrameSamples == 0) {
        m_physicsBenchmarkMinimumFps = fps;
    } else {
        m_physicsBenchmarkMinimumFps =
            std::min(m_physicsBenchmarkMinimumFps, fps);
    }
    ++m_physicsBenchmarkFrameSamples;
    m_physicsBenchmarkOverallFpsSum += fps;
    if (m_physicsBenchmarkOverallFrameSamples == 0) {
        m_physicsBenchmarkOverallMinimumFps = fps;
    } else {
        m_physicsBenchmarkOverallMinimumFps =
            std::min(m_physicsBenchmarkOverallMinimumFps, fps);
    }
    ++m_physicsBenchmarkOverallFrameSamples;

    if (m_physicsBenchmarkSampleSeconds < AnalyticsWindowSeconds) return;
    const float inverseSamples = 1.0f
        / static_cast<float>(std::max<std::uint32_t>(
            1u, m_physicsBenchmarkFrameSamples));
    PhysicsBenchmarkSample sample;
    sample.averageFps = m_physicsBenchmarkFpsSum * inverseSamples;
    sample.minimumFps = m_physicsBenchmarkMinimumFps;
    sample.averagePhysicsMilliseconds =
        m_physicsBenchmarkPhysicsMillisecondsSum * inverseSamples;
    sample.maximumPhysicsMilliseconds =
        m_physicsBenchmarkMaximumPhysicsMilliseconds;
    sample.propCount = static_cast<std::uint32_t>(
        m_physicsBenchmarkPropEntityIds.size());
    sample.ragdollCount = static_cast<std::uint32_t>(
        m_physicsBenchmarkRagdollEntityIds.size());
    if (m_physicsBenchmarkSamples.size() >= MaximumAnalyticsSamples) {
        m_physicsBenchmarkSamples.erase(
            m_physicsBenchmarkSamples.begin());
    }
    m_physicsBenchmarkSamples.push_back(sample);
    Log::info("Benchmark 5 s: " + std::to_string(sample.averageFps)
        + " FPS medio; " + std::to_string(sample.minimumFps)
        + " minimo; PhysX "
        + std::to_string(sample.averagePhysicsMilliseconds)
        + " ms; props " + std::to_string(sample.propCount)
        + "; ragdolls " + std::to_string(sample.ragdollCount) + ".");

    m_physicsBenchmarkSampleSeconds =
        std::fmod(m_physicsBenchmarkSampleSeconds, AnalyticsWindowSeconds);
    m_physicsBenchmarkFpsSum = 0.0f;
    m_physicsBenchmarkMinimumFps = 0.0f;
    m_physicsBenchmarkPhysicsMillisecondsSum = 0.0f;
    m_physicsBenchmarkMaximumPhysicsMilliseconds = 0.0f;
    m_physicsBenchmarkFrameSamples = 0;
}

} // namespace MatterEngine::Workbench
