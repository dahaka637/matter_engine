#include "Engine/Physics/PhysicsScene3D.hpp"

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysX/PhysXInternals3D.hpp"

#include <characterkinematic/PxControllerManager.h>
#include <characterkinematic/PxCapsuleController.h>
#include <extensions/PxD6Joint.h>
#include <extensions/PxRigidActorExt.h>
#include <extensions/PxRigidBodyExt.h>
#include <task/PxCpuDispatcher.h>
#include <task/PxTask.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace MatterEngine {
namespace {

constexpr float MinimumMassKg = 0.001f;
constexpr float MinimumShapeSizeMeters = 0.0005f;
constexpr float DegreesToRadians = 0.01745329251994329577f;
constexpr std::uint32_t PhysXScratchAlignmentBytes = 16u;
constexpr std::uint32_t PhysXScratchGranularityBytes = 16u * 1024u;
constexpr physx::PxU32 FilterFlagContinuousCollision = 1u << 0;

// Adaptador privado que entrega as tarefas nativas da PhysX ao pool central
// da MatterEngine. Dessa maneira fisica, gameplay e preparacao grafica nao
// criam pools concorrentes disputando os mesmos nucleos da CPU.
class MatterCpuDispatcher final : public physx::PxCpuDispatcher {
public:
    MatterCpuDispatcher(std::shared_ptr<TaskScheduler> scheduler,
        std::uint32_t advertisedWorkerCount)
        : m_scheduler(std::move(scheduler)),
          m_advertisedWorkerCount(advertisedWorkerCount) {
    }

    void submitTask(physx::PxBaseTask& task) override {
        m_scheduler->submit({ &executePhysXTask, &task });
    }

    [[nodiscard]] std::uint32_t getWorkerCount() const override {
        return m_currentWorkerCount.load(std::memory_order_acquire);
    }

    void setWorkerCount(std::uint32_t count) {
        m_currentWorkerCount.store(std::clamp(count, 1u,
            m_advertisedWorkerCount), std::memory_order_release);
    }

private:
    static void executePhysXTask(void* context) noexcept {
        auto* task = static_cast<physx::PxBaseTask*>(context);
        task->run();
        task->release();
    }

    std::shared_ptr<TaskScheduler> m_scheduler;
    std::uint32_t m_advertisedWorkerCount = 1;
    std::atomic<std::uint32_t> m_currentWorkerCount { 1 };
};

physx::PxFilterFlags simulationFilterShader(
    physx::PxFilterObjectAttributes attributes0,
    physx::PxFilterData filterData0,
    physx::PxFilterObjectAttributes attributes1,
    physx::PxFilterData filterData1,
    physx::PxPairFlags& pairFlags,
    const void*, physx::PxU32) {
    if (physx::PxFilterObjectIsTrigger(attributes0)
        || physx::PxFilterObjectIsTrigger(attributes1)) {
        pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
        return physx::PxFilterFlag::eDEFAULT;
    }
    if ((filterData0.word0 & filterData1.word1) == 0
        || (filterData1.word0 & filterData0.word1) == 0) {
        return physx::PxFilterFlag::eSUPPRESS;
    }

    pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT
        | physx::PxPairFlag::eNOTIFY_TOUCH_FOUND
        | physx::PxPairFlag::eNOTIFY_CONTACT_POINTS;
    if (((filterData0.word2 | filterData1.word2)
            & FilterFlagContinuousCollision) != 0) {
        pairFlags |= physx::PxPairFlag::eDETECT_CCD_CONTACT;
    }
    return physx::PxFilterFlag::eDEFAULT;
}

float moveToward(float current, float target, float maximumDelta) {
    if (current < target) return std::min(current + maximumDelta, target);
    return std::max(current - maximumDelta, target);
}

Vec3 moveToward(Vec3 current, Vec3 target, float maximumDelta) {
    const Vec3 difference = target - current;
    const float distance = difference.length();
    if (distance <= maximumDelta || distance <= 1.0e-6f) return target;
    return current + difference * (maximumDelta / distance);
}

physx::PxExtendedVec3 toExtended(Vec3 value) {
    return { static_cast<physx::PxExtended>(value.x),
        static_cast<physx::PxExtended>(value.y),
        static_cast<physx::PxExtended>(value.z) };
}

Vec3 fromExtended(const physx::PxExtendedVec3& value) {
    return { static_cast<float>(value.x), static_cast<float>(value.y),
        static_cast<float>(value.z) };
}

float capsuleCylinderHeight(float totalHeight, float radius) {
    return std::max(0.01f, totalHeight - radius * 2.0f);
}

// A filtragem de scene queries nao usa automaticamente o shader de pares da
// simulacao. O CCT e os overlaps de seguranca passam por este mesmo filtro,
// mantendo layers/masks coerentes e permitindo que o voo ignore o mundo sem
// remover ou teletransportar o controller.
class CharacterQueryFilter final : public physx::PxQueryFilterCallback {
public:
    explicit CharacterQueryFilter(const physx::PxRigidActor* ignoredActor)
        : m_ignoredActor(ignoredActor) {
    }

    physx::PxQueryHitType::Enum preFilter(
        const physx::PxFilterData& queryData,
        const physx::PxShape* shape,
        const physx::PxRigidActor* actor,
        physx::PxHitFlags&) override {
        if (shape == nullptr || actor == m_ignoredActor
            || !shape->getFlags().isSet(
                physx::PxShapeFlag::eSIMULATION_SHAPE)) {
            return physx::PxQueryHitType::eNONE;
        }
        const physx::PxFilterData shapeData = shape->getQueryFilterData();
        if ((queryData.word0 & shapeData.word1) == 0
            || (shapeData.word0 & queryData.word1) == 0) {
            return physx::PxQueryHitType::eNONE;
        }
        return physx::PxQueryHitType::eBLOCK;
    }

    physx::PxQueryHitType::Enum postFilter(
        const physx::PxFilterData&, const physx::PxQueryHit&,
        const physx::PxShape*, const physx::PxRigidActor*) override {
        return physx::PxQueryHitType::eBLOCK;
    }

private:
    const physx::PxRigidActor* m_ignoredActor = nullptr;
};

} // namespace

struct PhysicsScene3D::Impl final
    : physx::PxSimulationEventCallback,
      physx::PxUserControllerHitReport {
    struct ShapeRecord {
        std::string materialId;
    };

    struct BodyRecord {
        PhysicsBodyHandle3D handle;
        PhysicsBodyDefinition3D definition;
        physx::PxRigidActor* actor = nullptr;
        bool frozen = false;
        std::vector<std::unique_ptr<ShapeRecord>> shapes;
    };

    struct BodySlot {
        std::uint32_t generation = 1;
        std::unique_ptr<BodyRecord> record;
    };

    std::shared_ptr<PhysicsEngine3D::Impl> engine;
    PhysicsSceneSettings3D settings;
    std::unique_ptr<MatterCpuDispatcher> dispatcher;
    physx::PxScene* scene = nullptr;
    std::unordered_map<std::string, physx::PxMaterial*> materials;
    std::vector<BodySlot> bodySlots;
    std::vector<std::uint32_t> freeBodySlots;
    std::vector<ContactImpactEvent3D> contactImpacts;
    std::vector<PhysicsBodyStateUpdate3D> activeBodyStateUpdates;
    std::vector<BodyRecord*> activeAerodynamicBodies;
    PhysicsStepDiagnostics3D diagnostics;
    std::vector<std::byte> scratchStorage;
    void* scratchBlock = nullptr;
    std::uint32_t scratchBlockSize = 0;

    physx::PxD6Joint* grabJoint = nullptr;
    PhysicsBodyHandle3D grabBody;

    physx::PxControllerManager* controllerManager = nullptr;
    physx::PxCapsuleController* character = nullptr;
    PhysicsCharacterState3D characterState;
    float coyoteRemaining = 0.0f;
    float jumpBufferRemaining = 0.0f;
    Vec3 characterMoveVelocity;
    // Copiados de CharacterMotorSettings3D a cada moveCharacter(), para que o
    // callback onShapeHit (que so recebe o hit da PhysX, sem settings) saiba
    // limitar o empurrao em props dinamicos sem depender de constantes fixas.
    float characterPushSpeedLimit = 0.0f;
    float characterPushSaturationPenetration = 0.25f;

    [[nodiscard]] BodyRecord* body(PhysicsBodyHandle3D handle) {
        if (!handle.valid() || handle.index >= bodySlots.size()) return nullptr;
        BodySlot& slot = bodySlots[handle.index];
        return slot.generation == handle.generation
            ? slot.record.get() : nullptr;
    }

    [[nodiscard]] const BodyRecord* body(PhysicsBodyHandle3D handle) const {
        if (!handle.valid() || handle.index >= bodySlots.size()) return nullptr;
        const BodySlot& slot = bodySlots[handle.index];
        return slot.generation == handle.generation
            ? slot.record.get() : nullptr;
    }

    void markAerodynamicBodyActive(BodyRecord* record) {
        if (record == nullptr
            || !record->definition.aerodynamicDragEnabled
            || std::find(activeAerodynamicBodies.begin(),
                activeAerodynamicBodies.end(), record)
                != activeAerodynamicBodies.end()) {
            return;
        }
        activeAerodynamicBodies.push_back(record);
    }

    [[nodiscard]] physx::PxMaterial& material(std::string_view id) const {
        const auto found = materials.find(std::string(id));
        if (found == materials.end() || found->second == nullptr) {
            throw std::runtime_error(
                "Material fisico nao registrado na cena: " + std::string(id));
        }
        return *found->second;
    }

    void releaseGrab() {
        if (grabJoint != nullptr) {
            grabJoint->release();
            grabJoint = nullptr;
        }
        grabBody = {};
    }

    void releaseCharacter() {
        if (character != nullptr) {
            character->release();
            character = nullptr;
        }
        characterState = {};
        coyoteRemaining = 0.0f;
        jumpBufferRemaining = 0.0f;
    }

    [[nodiscard]] bool characterCapsuleIsClear(float totalHeight,
        float radius) const {
        if (character == nullptr || scene == nullptr) return false;

        // Retraimos 2 mm da geometria consultada. Isso evita que o contato
        // tangente legitimo com o piso seja interpretado como penetracao,
        // sem abrir espaco suficiente para atravessar paredes ou tetos.
        constexpr float QueryInsetMeters = 0.002f;
        const float queryRadius = std::max(MinimumShapeSizeMeters,
            radius - QueryInsetMeters);
        const float cylinderHalfHeight = 0.5f
            * capsuleCylinderHeight(totalHeight, radius);
        const Vec3 feet = fromExtended(character->getFootPosition());
        const Vec3 center = feet
            + Vec3 { 0.0f, 0.0f, totalHeight * 0.5f };
        const physx::PxCapsuleGeometry geometry(queryRadius,
            cylinderHalfHeight);
        // PxCapsuleGeometry e longitudinal em X; a MatterEngine usa Z como
        // eixo vertical para personagem e mundo.
        const physx::PxTransform pose(toPhysX(center), physx::PxQuat(
            -physx::PxHalfPi, physx::PxVec3(0.0f, 1.0f, 0.0f)));
        const physx::PxFilterData collisionFilter(1u, 0xFFFFFFFFu, 0, 0);
        CharacterQueryFilter callback(character->getActor());
        physx::PxQueryFilterData query(collisionFilter,
            physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::eDYNAMIC
                | physx::PxQueryFlag::ePREFILTER
                | physx::PxQueryFlag::eANY_HIT);
        physx::PxOverlapBuffer overlap;
        return !scene->overlap(geometry, pose, overlap, query, &callback);
    }

    void releaseAll() {
        releaseGrab();
        releaseCharacter();
        if (controllerManager != nullptr) {
            controllerManager->release();
            controllerManager = nullptr;
        }
        for (BodySlot& slot : bodySlots) {
            if (slot.record && slot.record->actor != nullptr) {
                slot.record->actor->release();
                slot.record->actor = nullptr;
            }
            slot.record.reset();
        }
        activeAerodynamicBodies.clear();
        activeBodyStateUpdates.clear();
        if (scene != nullptr) {
            scene->release();
            scene = nullptr;
        }
        for (auto& [id, material] : materials) {
            static_cast<void>(id);
            if (material != nullptr) material->release();
        }
        materials.clear();
        dispatcher.reset();
        scratchBlock = nullptr;
        scratchBlockSize = 0;
        scratchStorage.clear();
    }

    ~Impl() override { releaseAll(); }

    void onConstraintBreak(physx::PxConstraintInfo*, physx::PxU32) override {}
    void onWake(physx::PxActor**, physx::PxU32) override {}
    void onSleep(physx::PxActor**, physx::PxU32) override {}
    void onTrigger(physx::PxTriggerPair*, physx::PxU32) override {}
    void onAdvance(const physx::PxRigidBody* const*, const physx::PxTransform*,
        const physx::PxU32) override {}

    void onContact(const physx::PxContactPairHeader& header,
        const physx::PxContactPair* pairs, physx::PxU32 pairCount) override {
        const auto* recordA = header.actors[0] != nullptr
            ? static_cast<const BodyRecord*>(header.actors[0]->userData)
            : nullptr;
        const auto* recordB = header.actors[1] != nullptr
            ? static_cast<const BodyRecord*>(header.actors[1]->userData)
            : nullptr;
        if (recordA == nullptr && recordB == nullptr) return;

        for (physx::PxU32 pairIndex = 0; pairIndex < pairCount; ++pairIndex) {
            const physx::PxContactPair& pair = pairs[pairIndex];
            if (!(pair.events & physx::PxPairFlag::eNOTIFY_TOUCH_FOUND)
                || pair.contactCount == 0) {
                continue;
            }
            // Contatos de audio nao justificam uma alocacao de heap por par.
            // Sessenta e quatro pontos cobrem com folga manifolds de props
            // compostos; o solver continua processando todos os contatos.
            constexpr physx::PxU32 MaximumReportedContactPoints = 64;
            std::array<physx::PxContactPairPoint,
                MaximumReportedContactPoints> points;
            const physx::PxU32 extracted = pair.extractContacts(
                points.data(), std::min<physx::PxU32>(
                    static_cast<physx::PxU32>(pair.contactCount),
                    MaximumReportedContactPoints));
            if (extracted == 0) continue;

            physx::PxVec3 totalImpulse(0.0f);
            const physx::PxContactPairPoint* representative = &points[0];
            float representativeImpulse = -1.0f;
            for (physx::PxU32 pointIndex = 0; pointIndex < extracted;
                ++pointIndex) {
                const physx::PxContactPairPoint& point = points[pointIndex];
                totalImpulse += point.impulse;
                const float magnitude = point.impulse.magnitudeSquared();
                if (magnitude > representativeImpulse) {
                    representativeImpulse = magnitude;
                    representative = &point;
                }
            }

            const float massA = recordA != nullptr
                && recordA->definition.motionType == PhysicsMotionType3D::Dynamic
                ? std::max(MinimumMassKg, recordA->definition.massKg)
                : 0.0f;
            const float massB = recordB != nullptr
                && recordB->definition.motionType == PhysicsMotionType3D::Dynamic
                ? std::max(MinimumMassKg, recordB->definition.massKg)
                : 0.0f;
            float effectiveMass = 0.0f;
            if (massA > 0.0f && massB > 0.0f) {
                effectiveMass = (massA * massB) / (massA + massB);
            } else {
                effectiveMass = std::max(massA, massB);
            }
            if (effectiveMass <= 0.0f) continue;

            const physx::PxVec3 normal = representative->normal;
            const float normalImpulse = std::abs(totalImpulse.dot(normal));
            const float totalMagnitude = totalImpulse.magnitude();
            const float tangentialImpulse = std::sqrt(std::max(0.0f,
                totalMagnitude * totalMagnitude
                    - normalImpulse * normalImpulse));
            // Impulso normal / massa efetiva e a variacao de velocidade que
            // efetivamente atravessou o contato. Ela alimenta o audio sem
            // solicitar o stream extra de velocidades pre-solver para todos
            // os pares da cena.
            const float approachSpeed = normalImpulse / effectiveMass;

            const auto* shapeA = pair.shapes[0] != nullptr
                ? static_cast<const ShapeRecord*>(pair.shapes[0]->userData)
                : nullptr;
            const auto* shapeB = pair.shapes[1] != nullptr
                ? static_cast<const ShapeRecord*>(pair.shapes[1]->userData)
                : nullptr;
            ContactImpactEvent3D impact;
            impact.bodyA = recordA != nullptr
                ? recordA->handle.index : InvalidPhysicsBodyIndex;
            impact.bodyB = recordB != nullptr
                ? recordB->handle.index : InvalidPhysicsBodyIndex;
            impact.bodyIdA = recordA != nullptr
                ? recordA->definition.entityId : 0;
            impact.bodyIdB = recordB != nullptr
                ? recordB->definition.entityId : 0;
            impact.materialA = shapeA != nullptr ? shapeA->materialId
                : recordA != nullptr ? recordA->definition.materialId : "default";
            impact.materialB = shapeB != nullptr ? shapeB->materialId
                : recordB != nullptr ? recordB->definition.materialId : "default";
            impact.position = fromPhysX(representative->position);
            impact.normal = fromPhysX(normal);
            impact.normalImpulseNewtonSeconds = normalImpulse;
            impact.tangentialImpulseNewtonSeconds = tangentialImpulse;
            impact.approachSpeedMetersPerSecond = approachSpeed;
            impact.effectiveMassKg = effectiveMass;
            impact.transferredEnergyJoules =
                0.5f * effectiveMass * approachSpeed * approachSpeed;
            impact.massA = massA;
            impact.massB = massB;
            impact.characteristicSizeA = recordA != nullptr
                ? recordA->definition.characteristicSizeMeters : 1.0f;
            impact.characteristicSizeB = recordB != nullptr
                ? recordB->definition.characteristicSizeMeters : 1.0f;
            impact.acousticGainA = recordA != nullptr
                ? recordA->definition.acousticGain : 1.0f;
            impact.acousticGainB = recordB != nullptr
                ? recordB->definition.acousticGain : 1.0f;
            impact.acousticDampingA = recordA != nullptr
                ? recordA->definition.acousticDamping : 1.0f;
            impact.acousticDampingB = recordB != nullptr
                ? recordB->definition.acousticDamping : 1.0f;
            impact.structureA = recordA != nullptr
                ? recordA->definition.acousticStructure
                : AcousticBodyStructure3D::Solid;
            impact.structureB = recordB != nullptr
                ? recordB->definition.acousticStructure
                : AcousticBodyStructure3D::Solid;
            impact.staticA = massA <= 0.0f;
            impact.staticB = massB <= 0.0f;
            contactImpacts.push_back(std::move(impact));
        }
    }

    void onShapeHit(const physx::PxControllerShapeHit& hit) override {
        auto* dynamic = hit.actor != nullptr
            ? hit.actor->is<physx::PxRigidDynamic>() : nullptr;
        if (dynamic == nullptr
            || dynamic->getRigidBodyFlags()
                .isSet(physx::PxRigidBodyFlag::eKINEMATIC)
            || std::abs(hit.dir.z) > 0.65f) {
            return;
        }
        const Vec3 direction = fromPhysX(hit.dir).normalized();
        const float closingSpeed = std::max(0.0f,
            dot(characterMoveVelocity, direction));
        if (closingSpeed <= 0.01f) return;

        // O empurrao e limitado por uma velocidade maxima realista (nunca por
        // um impulso fixo em kg*m/s): dessa forma a massa do prop e que decide
        // o impulso necessario, nao o contrario, e um objeto leve nao sai
        // arremessado so por ter pouca massa. hit.length mede quanto do passo
        // deste frame foi barrado por este contato; usamos isso apenas para
        // suavizar toques de raspao. Nao dividimos por nenhum passo de tempo
        // aqui porque PxForceMode::eIMPULSE ja aplica a variacao de velocidade
        // de uma vez, sem depender do deltaTime — a versao anterior dividia
        // por dt antes de aplicar como impulso instantaneo, inflando o
        // resultado em ~120x a 120 Hz, que era a causa raiz do arremesso.
        const float pushFraction = std::clamp(hit.length
            / characterPushSaturationPenetration, 0.0f, 1.0f);
        const float pushSpeed = std::min(closingSpeed,
            characterPushSpeedLimit) * pushFraction;
        if (pushSpeed <= 0.01f) return;

        const float dynamicMass = std::max(MinimumMassKg, dynamic->getMass());
        const float impulseMagnitude = dynamicMass * pushSpeed;
        physx::PxRigidBodyExt::addForceAtPos(*dynamic,
            toPhysX(direction * impulseMagnitude),
            physx::PxVec3(static_cast<float>(hit.worldPos.x),
                static_cast<float>(hit.worldPos.y),
                static_cast<float>(hit.worldPos.z)),
            physx::PxForceMode::eIMPULSE, true);
    }

    void onControllerHit(const physx::PxControllersHit&) override {}
    void onObstacleHit(const physx::PxControllerObstacleHit&) override {}
};

PhysicsScene3D::PhysicsScene3D(PhysicsEngine3D& physicsEngine,
    const PhysicsSceneSettings3D& sceneSettings,
    const MaterialLibrary& materialLibrary)
    : m_impl(std::make_unique<Impl>()) {
    m_impl->engine = physicsEngine.m_impl;
    m_impl->settings = sceneSettings;
    m_impl->contactImpacts.reserve(1024);
    m_impl->activeBodyStateUpdates.reserve(1024);
    m_impl->activeAerodynamicBodies.reserve(1024);
    const std::uint32_t availableWorkers =
        m_impl->engine->scheduler->workerCount();
    const std::uint32_t workerCount = sceneSettings.workerThreadCount > 0
        ? std::min(sceneSettings.workerThreadCount, availableWorkers)
        : availableWorkers;
    m_impl->dispatcher = std::make_unique<MatterCpuDispatcher>(
        m_impl->engine->scheduler, std::max(1u, workerCount));

    physx::PxSceneDesc descriptor(m_impl->engine->scale);
    descriptor.gravity = toPhysX(sceneSettings.gravity);
    descriptor.cpuDispatcher = m_impl->dispatcher.get();
    descriptor.filterShader = simulationFilterShader;
    descriptor.simulationEventCallback = m_impl.get();
    descriptor.broadPhaseType = physx::PxBroadPhaseType::ePABP;
    descriptor.solverType = physx::PxSolverType::eTGS;
    descriptor.flags |= physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS;
    if (sceneSettings.enableContinuousCollision) {
        descriptor.flags |= physx::PxSceneFlag::eENABLE_CCD;
    }
    if (sceneSettings.enableStabilization) {
        descriptor.flags |= physx::PxSceneFlag::eENABLE_STABILIZATION;
    }
    m_impl->scene = m_impl->engine->physics->createScene(descriptor);
    if (m_impl->scene == nullptr) {
        m_impl->dispatcher.reset();
        throw std::runtime_error("PhysX: falha ao criar a cena");
    }

    for (const auto& [id, definition] : materialLibrary.all()) {
        physx::PxMaterial* material = m_impl->engine->physics->createMaterial(
            definition.contact.staticFriction,
            definition.contact.dynamicFriction,
            definition.contact.restitution);
        if (material == nullptr) {
            throw std::runtime_error("PhysX: falha ao criar material " + id);
        }
        material->setFrictionCombineMode(physx::PxCombineMode::eAVERAGE);
        material->setRestitutionCombineMode(physx::PxCombineMode::eMAX);
        m_impl->materials.emplace(id, material);
    }
    m_impl->controllerManager = PxCreateControllerManager(
        *m_impl->scene, false);
    if (m_impl->controllerManager == nullptr) {
        throw std::runtime_error("PhysX: falha ao criar o gerenciador CCT");
    }

    const std::uint32_t requestedScratch = sceneSettings.scratchBufferSizeBytes;
    m_impl->scratchBlockSize = requestedScratch
        - requestedScratch % PhysXScratchGranularityBytes;
    if (m_impl->scratchBlockSize > 0) {
        m_impl->scratchStorage.resize(static_cast<std::size_t>(
            m_impl->scratchBlockSize) + PhysXScratchAlignmentBytes - 1u);
        void* raw = m_impl->scratchStorage.data();
        std::size_t available = m_impl->scratchStorage.size();
        m_impl->scratchBlock = std::align(PhysXScratchAlignmentBytes,
            m_impl->scratchBlockSize, raw, available);
        if (m_impl->scratchBlock == nullptr) {
            throw std::runtime_error(
                "PhysX: falha ao alinhar scratch buffer da cena");
        }
    }
}

PhysicsScene3D::~PhysicsScene3D() = default;

PhysicsBodyHandle3D PhysicsScene3D::createBody(
    const PhysicsBodyDefinition3D& definition,
    std::span<const PhysicsShape3D> shapes) {
    if (shapes.empty()) {
        throw std::invalid_argument("Um corpo PhysX precisa de ao menos uma shape");
    }
    if (definition.motionType == PhysicsMotionType3D::Dynamic
        && (!std::isfinite(definition.massKg)
            || definition.massKg < MinimumMassKg)) {
        throw std::invalid_argument("Massa dinamica invalida");
    }

    std::uint32_t slotIndex;
    if (!m_impl->freeBodySlots.empty()) {
        slotIndex = m_impl->freeBodySlots.back();
        m_impl->freeBodySlots.pop_back();
    } else {
        slotIndex = static_cast<std::uint32_t>(m_impl->bodySlots.size());
        m_impl->bodySlots.emplace_back();
    }
    Impl::BodySlot& slot = m_impl->bodySlots[slotIndex];
    const PhysicsBodyHandle3D handle { slotIndex, slot.generation };
    auto record = std::make_unique<Impl::BodyRecord>();
    record->handle = handle;
    record->definition = definition;

    const physx::PxTransform pose = toPhysX(definition.position,
        definition.orientation);
    if (definition.motionType == PhysicsMotionType3D::Static) {
        record->actor = m_impl->engine->physics->createRigidStatic(pose);
    } else {
        record->actor = m_impl->engine->physics->createRigidDynamic(pose);
    }
    if (record->actor == nullptr) {
        m_impl->freeBodySlots.push_back(slotIndex);
        throw std::runtime_error("PhysX: falha ao criar ator rigido");
    }
    record->actor->userData = record.get();

    try {
        for (const PhysicsShape3D& shape : shapes) {
            const std::string& materialId = shape.materialId.empty()
                ? definition.materialId : shape.materialId;
            physx::PxMaterial& material = m_impl->material(materialId);
            physx::PxShape* nativeShape = nullptr;
            switch (shape.type) {
            case PhysicsShapeType3D::Box:
                if (shape.halfExtents.x < MinimumShapeSizeMeters
                    || shape.halfExtents.y < MinimumShapeSizeMeters
                    || shape.halfExtents.z < MinimumShapeSizeMeters) {
                    throw std::invalid_argument("Box collider degenerado");
                }
                nativeShape = physx::PxRigidActorExt::createExclusiveShape(
                    *record->actor,
                    physx::PxBoxGeometry(toPhysX(shape.halfExtents)), material);
                break;
            case PhysicsShapeType3D::Sphere:
                if (shape.radius < MinimumShapeSizeMeters) {
                    throw std::invalid_argument("Sphere collider degenerado");
                }
                nativeShape = physx::PxRigidActorExt::createExclusiveShape(
                    *record->actor, physx::PxSphereGeometry(shape.radius),
                    material);
                break;
            case PhysicsShapeType3D::Capsule:
                if (shape.radius < MinimumShapeSizeMeters
                    || shape.capsuleHalfHeight < 0.0f) {
                    throw std::invalid_argument("Capsule collider degenerado");
                }
                nativeShape = physx::PxRigidActorExt::createExclusiveShape(
                    *record->actor, physx::PxCapsuleGeometry(shape.radius,
                        shape.capsuleHalfHeight), material);
                break;
            case PhysicsShapeType3D::ConvexMesh:
                if (!shape.mesh || shape.mesh->m_type != PhysicsMeshType3D::Convex
                    || shape.mesh->m_impl->convex == nullptr) {
                    throw std::invalid_argument("Convex mesh invalida");
                }
                nativeShape = physx::PxRigidActorExt::createExclusiveShape(
                    *record->actor,
                    physx::PxConvexMeshGeometry(shape.mesh->m_impl->convex),
                    material);
                break;
            case PhysicsShapeType3D::TriangleMesh:
                if (definition.motionType == PhysicsMotionType3D::Dynamic) {
                    throw std::invalid_argument(
                        "Triangle mesh nao pode ser usada em corpo dinamico");
                }
                if (!shape.mesh || shape.mesh->m_type != PhysicsMeshType3D::Triangle
                    || shape.mesh->m_impl->triangle == nullptr) {
                    throw std::invalid_argument("Triangle mesh invalida");
                }
                nativeShape = physx::PxRigidActorExt::createExclusiveShape(
                    *record->actor,
                    physx::PxTriangleMeshGeometry(shape.mesh->m_impl->triangle),
                    material);
                break;
            }
            if (nativeShape == nullptr) {
                throw std::runtime_error("PhysX: falha ao criar shape");
            }
            Quaternion localOrientation = shape.localOrientation;
            if (shape.type == PhysicsShapeType3D::Capsule) {
                // PxCapsuleGeometry usa X como eixo longitudinal; o contrato
                // da MatterEngine usa Y para coincidir com assets DCC.
                localOrientation = (localOrientation
                    * Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                        1.57079632679f)).normalized();
            }
            nativeShape->setLocalPose(toPhysX(shape.localPosition,
                localOrientation));
            const physx::PxU32 filterFlags = definition.collisionMode
                    == PhysicsCollisionMode3D::Continuous
                ? FilterFlagContinuousCollision : 0u;
            const physx::PxFilterData filter(definition.collisionLayer,
                definition.collisionMask, filterFlags, 0);
            nativeShape->setSimulationFilterData(filter);
            nativeShape->setQueryFilterData(filter);
            auto shapeRecord = std::make_unique<Impl::ShapeRecord>();
            shapeRecord->materialId = materialId;
            nativeShape->userData = shapeRecord.get();
            record->shapes.push_back(std::move(shapeRecord));
        }

        if (auto* dynamic = record->actor->is<physx::PxRigidDynamic>()) {
            dynamic->setLinearDamping(std::max(0.0f,
                definition.linearDamping));
            dynamic->setAngularDamping(std::max(0.0f,
                definition.angularDamping));
            dynamic->setLinearVelocity(toPhysX(definition.linearVelocity));
            dynamic->setAngularVelocity(toPhysX(definition.angularVelocity));
            dynamic->setSolverIterationCounts(
                m_impl->settings.solverPositionIterations,
                m_impl->settings.solverVelocityIterations);
            dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eENABLE_CCD,
                definition.collisionMode == PhysicsCollisionMode3D::Continuous
                    && m_impl->settings.enableContinuousCollision);
            dynamic->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY,
                definition.motionType == PhysicsMotionType3D::Kinematic);
            if (definition.motionType == PhysicsMotionType3D::Kinematic) {
                dynamic->setRigidBodyFlag(
                    physx::PxRigidBodyFlag::eKINEMATIC, true);
            } else if (!physx::PxRigidBodyExt::setMassAndUpdateInertia(
                    *dynamic, definition.massKg)) {
                throw std::runtime_error(
                    "PhysX nao conseguiu calcular a inercia do corpo");
            }
            if (!definition.allowSleeping) {
                dynamic->setSleepThreshold(0.0f);
            }
            if (!definition.startAwake
                && definition.motionType == PhysicsMotionType3D::Dynamic) {
                dynamic->putToSleep();
            }
        }
        m_impl->scene->addActor(*record->actor);
        if (definition.motionType == PhysicsMotionType3D::Dynamic
            && definition.aerodynamicDragEnabled
            && definition.startAwake) {
            m_impl->markAerodynamicBodyActive(record.get());
        }
    } catch (...) {
        record->actor->release();
        record->actor = nullptr;
        m_impl->freeBodySlots.push_back(slotIndex);
        throw;
    }
    slot.record = std::move(record);
    return handle;
}

void PhysicsScene3D::destroyBody(PhysicsBodyHandle3D bodyHandle) {
    Impl::BodyRecord* record = m_impl->body(bodyHandle);
    if (record == nullptr) return;
    if (m_impl->grabBody == bodyHandle) m_impl->releaseGrab();
    std::erase(m_impl->activeAerodynamicBodies, record);
    if (record->actor != nullptr) {
        record->actor->release();
        record->actor = nullptr;
    }
    Impl::BodySlot& slot = m_impl->bodySlots[bodyHandle.index];
    slot.record.reset();
    ++slot.generation;
    if (slot.generation == 0) ++slot.generation;
    m_impl->freeBodySlots.push_back(bodyHandle.index);
}

void PhysicsScene3D::clear() {
    m_impl->releaseGrab();
    m_impl->releaseCharacter();
    for (std::uint32_t index = 0; index < m_impl->bodySlots.size(); ++index) {
        Impl::BodySlot& slot = m_impl->bodySlots[index];
        if (!slot.record) continue;
        const PhysicsBodyHandle3D handle { index, slot.generation };
        destroyBody(handle);
    }
    m_impl->contactImpacts.clear();
}

bool PhysicsScene3D::contains(PhysicsBodyHandle3D body) const {
    return m_impl->body(body) != nullptr;
}

PhysicsBodyState3D PhysicsScene3D::bodyState(
    PhysicsBodyHandle3D bodyHandle) const {
    const Impl::BodyRecord* record = m_impl->body(bodyHandle);
    if (record == nullptr || record->actor == nullptr) {
        throw std::out_of_range("Handle de corpo fisico expirado");
    }
    const physx::PxTransform pose = record->actor->getGlobalPose();
    PhysicsBodyState3D state;
    state.position = fromPhysX(pose.p);
    state.orientation = fromPhysX(pose.q);
    state.frozen = record->frozen;
    if (const auto* dynamic = record->actor->is<physx::PxRigidDynamic>()) {
        state.linearVelocity = fromPhysX(dynamic->getLinearVelocity());
        state.angularVelocity = fromPhysX(dynamic->getAngularVelocity());
        state.sleeping = dynamic->isSleeping();
    }
    return state;
}

float PhysicsScene3D::bodyMass(PhysicsBodyHandle3D bodyHandle) const {
    const Impl::BodyRecord* record = m_impl->body(bodyHandle);
    if (record == nullptr) throw std::out_of_range("Handle expirado");
    return record->definition.motionType == PhysicsMotionType3D::Dynamic
        ? record->definition.massKg : 0.0f;
}

void PhysicsScene3D::setBodyFrozen(PhysicsBodyHandle3D bodyHandle,
    bool frozen) {
    Impl::BodyRecord* record = m_impl->body(bodyHandle);
    if (record == nullptr) return;
    auto* dynamic = record->actor->is<physx::PxRigidDynamic>();
    if (dynamic == nullptr
        || record->definition.motionType != PhysicsMotionType3D::Dynamic
        || record->frozen == frozen) {
        return;
    }
    if (frozen) {
        dynamic->setLinearVelocity(physx::PxVec3(0.0f));
        dynamic->setAngularVelocity(physx::PxVec3(0.0f));
        dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eENABLE_CCD, false);
        dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, true);
        dynamic->setKinematicTarget(dynamic->getGlobalPose());
    } else {
        dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, false);
        dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eENABLE_CCD,
            record->definition.collisionMode
                == PhysicsCollisionMode3D::Continuous
                && m_impl->settings.enableContinuousCollision);
        dynamic->wakeUp();
        m_impl->markAerodynamicBodyActive(record);
    }
    record->frozen = frozen;
}

bool PhysicsScene3D::bodyFrozen(PhysicsBodyHandle3D bodyHandle) const {
    const Impl::BodyRecord* record = m_impl->body(bodyHandle);
    return record != nullptr && record->frozen;
}

void PhysicsScene3D::wakeBody(PhysicsBodyHandle3D bodyHandle) {
    Impl::BodyRecord* record = m_impl->body(bodyHandle);
    if (record == nullptr) return;
    if (auto* dynamic = record->actor->is<physx::PxRigidDynamic>();
        dynamic != nullptr && !record->frozen) {
        dynamic->wakeUp();
        m_impl->markAerodynamicBodyActive(record);
    }
}

void PhysicsScene3D::applyForceAtPoint(PhysicsBodyHandle3D bodyHandle,
    Vec3 force, Vec3 worldPoint) {
    Impl::BodyRecord* record = m_impl->body(bodyHandle);
    if (record == nullptr || record->frozen) return;
    if (auto* dynamic = record->actor->is<physx::PxRigidDynamic>()) {
        m_impl->markAerodynamicBodyActive(record);
        physx::PxRigidBodyExt::addForceAtPos(*dynamic, toPhysX(force),
            toPhysX(worldPoint), physx::PxForceMode::eFORCE, true);
    }
}

void PhysicsScene3D::applyTorque(PhysicsBodyHandle3D bodyHandle, Vec3 torque) {
    Impl::BodyRecord* record = m_impl->body(bodyHandle);
    if (record == nullptr || record->frozen) return;
    if (auto* dynamic = record->actor->is<physx::PxRigidDynamic>()) {
        m_impl->markAerodynamicBodyActive(record);
        dynamic->addTorque(toPhysX(torque), physx::PxForceMode::eFORCE, true);
    }
}

namespace {

// A API publica da MatterEngine so conhece corpos registrados em BodyRecord.
// O PhysX tambem mantem atores internos, como o PxController do jogador; sem
// este filtro uma scene query pode acertar um desses atores e retornar um hit
// sem PhysicsBodyHandle3D valido. A opcao dynamicOnly tambem exclui atores
// cinematicos, pois a Physgun deve manipular exclusivamente props dinamicos.
class ManagedBodyQueryFilter final : public physx::PxQueryFilterCallback {
public:
    explicit ManagedBodyQueryFilter(bool dynamicOnly)
        : m_dynamicOnly(dynamicOnly) {
    }

    physx::PxQueryHitType::Enum preFilter(
        const physx::PxFilterData&, const physx::PxShape* shape,
        const physx::PxRigidActor* actor, physx::PxHitFlags&) override {
        if (shape == nullptr || actor == nullptr || actor->userData == nullptr
            || shape->userData == nullptr) {
            return physx::PxQueryHitType::eNONE;
        }
        const auto* body = static_cast<const PhysicsScene3D::Impl::BodyRecord*>(
            actor->userData);
        if (m_dynamicOnly
            && body->definition.motionType != PhysicsMotionType3D::Dynamic) {
            return physx::PxQueryHitType::eNONE;
        }
        return physx::PxQueryHitType::eBLOCK;
    }

    physx::PxQueryHitType::Enum postFilter(
        const physx::PxFilterData&, const physx::PxQueryHit&,
        const physx::PxShape*, const physx::PxRigidActor*) override {
        return physx::PxQueryHitType::eBLOCK;
    }

private:
    bool m_dynamicOnly = false;
};

bool raycastImpl(const PhysicsScene3D::Impl& implementation,
    const Ray3D& ray, float maximumDistance, physx::PxQueryFlags queryFlags,
    bool dynamicOnly, PhysicsRayHit3D& hit) {
    if (maximumDistance <= 0.0f || ray.direction.lengthSquared() <= 1.0e-10f) {
        return false;
    }
    physx::PxRaycastBuffer buffer;
    physx::PxQueryFilterData filter;
    filter.flags = queryFlags | physx::PxQueryFlag::ePREFILTER;
    ManagedBodyQueryFilter callback(dynamicOnly);
    const bool found = implementation.scene->raycast(toPhysX(ray.origin),
        toPhysX(ray.direction.normalized()), maximumDistance, buffer,
        physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL,
        filter, &callback);
    if (!found || !buffer.hasBlock) return false;
    const auto* body = buffer.block.actor != nullptr
        ? static_cast<const PhysicsScene3D::Impl::BodyRecord*>(
            buffer.block.actor->userData)
        : nullptr;
    const auto* shape = buffer.block.shape != nullptr
        ? static_cast<const PhysicsScene3D::Impl::ShapeRecord*>(
            buffer.block.shape->userData)
        : nullptr;
    hit.body = body != nullptr ? body->handle : PhysicsBodyHandle3D {};
    hit.position = fromPhysX(buffer.block.position);
    hit.normal = fromPhysX(buffer.block.normal);
    hit.distance = buffer.block.distance;
    hit.materialId = shape != nullptr ? shape->materialId
        : body != nullptr ? body->definition.materialId : "default";
    return true;
}

} // namespace

bool PhysicsScene3D::raycast(const Ray3D& ray, float maximumDistance,
    PhysicsRayHit3D& hit) const {
    return raycastImpl(*m_impl, ray, maximumDistance,
        physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::eDYNAMIC,
        false, hit);
}

bool PhysicsScene3D::raycastDynamic(const Ray3D& ray,
    float maximumDistance, PhysicsRayHit3D& hit) const {
    return raycastImpl(*m_impl, ray, maximumDistance,
        physx::PxQueryFlag::eDYNAMIC, true, hit);
}

bool PhysicsScene3D::raycastStatic(const Ray3D& ray, float maximumDistance,
    PhysicsRayHit3D& hit) const {
    return raycastImpl(*m_impl, ray, maximumDistance,
        physx::PxQueryFlag::eSTATIC, false, hit);
}

namespace {

bool sweepSphereImpl(const PhysicsScene3D::Impl& implementation,
    const Ray3D& ray, float radius, float maximumDistance,
    physx::PxQueryFlags queryFlags, bool dynamicOnly,
    PhysicsRayHit3D& hit) {
    if (maximumDistance <= 0.0f || radius <= 0.0f
        || ray.direction.lengthSquared() <= 1.0e-10f) {
        return false;
    }
    physx::PxSweepBuffer buffer;
    physx::PxQueryFilterData filter;
    filter.flags = queryFlags | physx::PxQueryFlag::ePREFILTER;
    ManagedBodyQueryFilter callback(dynamicOnly);
    const physx::PxSphereGeometry geometry(radius);
    const physx::PxTransform pose(toPhysX(ray.origin));
    const bool found = implementation.scene->sweep(geometry, pose,
        toPhysX(ray.direction.normalized()), maximumDistance, buffer,
        physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL,
        filter, &callback);
    if (!found || !buffer.hasBlock) return false;
    const auto* body = buffer.block.actor != nullptr
        ? static_cast<const PhysicsScene3D::Impl::BodyRecord*>(
            buffer.block.actor->userData)
        : nullptr;
    const auto* shape = buffer.block.shape != nullptr
        ? static_cast<const PhysicsScene3D::Impl::ShapeRecord*>(
            buffer.block.shape->userData)
        : nullptr;
    hit.body = body != nullptr ? body->handle : PhysicsBodyHandle3D {};
    hit.position = fromPhysX(buffer.block.position);
    hit.normal = fromPhysX(buffer.block.normal);
    hit.distance = buffer.block.distance;
    hit.materialId = shape != nullptr ? shape->materialId
        : body != nullptr ? body->definition.materialId : "default";
    return true;
}

} // namespace

bool PhysicsScene3D::sweepSphere(const Ray3D& ray, float radius,
    float maximumDistance, PhysicsRayHit3D& hit) const {
    return sweepSphereImpl(*m_impl, ray, radius, maximumDistance,
        physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::eDYNAMIC,
        false, hit);
}

bool PhysicsScene3D::sweepSphereDynamic(const Ray3D& ray, float radius,
    float maximumDistance, PhysicsRayHit3D& hit) const {
    return sweepSphereImpl(*m_impl, ray, radius, maximumDistance,
        physx::PxQueryFlag::eDYNAMIC, true, hit);
}

bool PhysicsScene3D::beginGrab(PhysicsBodyHandle3D bodyHandle,
    Vec3 localGrabPoint, const PhysicsGrabTarget3D& target,
    const PhysicsHandleSettings3D& handleSettings) {
    m_impl->releaseGrab();
    Impl::BodyRecord* record = m_impl->body(bodyHandle);
    auto* dynamic = record != nullptr
        ? record->actor->is<physx::PxRigidDynamic>() : nullptr;
    if (dynamic == nullptr
        || record->definition.motionType != PhysicsMotionType3D::Dynamic) {
        return false;
    }
    if (record->frozen) setBodyFrozen(bodyHandle, false);

    m_impl->grabJoint = physx::PxD6JointCreate(*m_impl->engine->physics,
        nullptr, physx::PxTransform(physx::PxIdentity), dynamic,
        physx::PxTransform(toPhysX(localGrabPoint)));
    if (m_impl->grabJoint == nullptr) return false;
    for (physx::PxD6Axis::Enum axis : { physx::PxD6Axis::eX,
            physx::PxD6Axis::eY, physx::PxD6Axis::eZ,
            physx::PxD6Axis::eTWIST, physx::PxD6Axis::eSWING1,
            physx::PxD6Axis::eSWING2 }) {
        m_impl->grabJoint->setMotion(axis, physx::PxD6Motion::eFREE);
    }
    m_impl->grabJoint->setAngularDriveConfig(
        physx::PxD6AngularDriveConfig::eSLERP);
    m_impl->grabBody = bodyHandle;
    updateGrabTarget(target, handleSettings);
    dynamic->wakeUp();
    m_impl->markAerodynamicBodyActive(record);
    return true;
}

void PhysicsScene3D::updateGrabTarget(const PhysicsGrabTarget3D& target,
    const PhysicsHandleSettings3D& handleSettings) {
    if (m_impl->grabJoint == nullptr) return;
    Impl::BodyRecord* record = m_impl->body(m_impl->grabBody);
    auto* dynamic = record != nullptr
        ? record->actor->is<physx::PxRigidDynamic>() : nullptr;
    if (dynamic == nullptr) {
        m_impl->releaseGrab();
        return;
    }
    const float mass = std::max(MinimumMassKg, dynamic->getMass());
    const float linearDamping = 2.0f
        * std::max(0.0f, handleSettings.linearDampingRatio)
        * std::sqrt(std::max(0.0f,
            handleSettings.linearStiffness * mass));
    const physx::PxD6JointDrive linearDrive(
        std::max(0.0f, handleSettings.linearStiffness), linearDamping,
        std::max(0.0f, handleSettings.maximumForce), false);
    m_impl->grabJoint->setDrive(physx::PxD6Drive::eX, linearDrive);
    m_impl->grabJoint->setDrive(physx::PxD6Drive::eY, linearDrive);
    m_impl->grabJoint->setDrive(physx::PxD6Drive::eZ, linearDrive);

    const physx::PxVec3 inertia = dynamic->getMassSpaceInertiaTensor();
    const float averageInertia = std::max(1.0e-5f,
        (inertia.x + inertia.y + inertia.z) / 3.0f);
    const float angularDamping = 2.0f
        * std::max(0.0f, handleSettings.angularDampingRatio)
        * std::sqrt(std::max(0.0f,
            handleSettings.angularStiffness * averageInertia));
    const physx::PxD6JointDrive angularDrive(
        target.lockOrientation
            ? std::max(0.0f, handleSettings.angularStiffness) : 0.0f,
        target.lockOrientation ? angularDamping : 0.0f,
        target.lockOrientation
            ? std::max(0.0f, handleSettings.maximumTorque) : 0.0f,
        false);
    m_impl->grabJoint->setDrive(physx::PxD6Drive::eSLERP, angularDrive);
    m_impl->grabJoint->setDrivePosition(toPhysX(target.position,
        target.orientation), true);
}

void PhysicsScene3D::endGrab() { m_impl->releaseGrab(); }
bool PhysicsScene3D::grabbing() const { return m_impl->grabJoint != nullptr; }
PhysicsBodyHandle3D PhysicsScene3D::grabbedBody() const {
    return m_impl->grabBody;
}

void PhysicsScene3D::createCharacter(Vec3 feetPosition,
    const CharacterMotorSettings3D& settings) {
    m_impl->releaseCharacter();
    physx::PxCapsuleControllerDesc descriptor;
    descriptor.radius = settings.radius;
    descriptor.height = capsuleCylinderHeight(settings.standingHeight,
        settings.radius);
    descriptor.position = toExtended(feetPosition
        + Vec3 { 0.0f, 0.0f, settings.standingHeight * 0.5f });
    descriptor.upDirection = physx::PxVec3(0.0f, 0.0f, 1.0f);
    descriptor.slopeLimit = std::cos(
        settings.maximumSlopeDegrees * DegreesToRadians);
    descriptor.stepOffset = settings.maximumStepHeight;
    descriptor.contactOffset = settings.skinWidth;
    descriptor.climbingMode = physx::PxCapsuleClimbingMode::eCONSTRAINED;
    descriptor.nonWalkableMode =
        physx::PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING;
    descriptor.material = &m_impl->material("default");
    descriptor.reportCallback = m_impl.get();
    descriptor.scaleCoeff = 0.8f;
    descriptor.density = 80.0f;
    m_impl->character = static_cast<physx::PxCapsuleController*>(
        m_impl->controllerManager->createController(descriptor));
    if (m_impl->character == nullptr) {
        throw std::runtime_error("PhysX: falha ao criar controller do jogador");
    }
    m_impl->characterState.position = feetPosition
        + Vec3 { 0.0f, 0.0f, settings.standingHeight * 0.5f };
}

void PhysicsScene3D::destroyCharacter() { m_impl->releaseCharacter(); }

void PhysicsScene3D::placeCharacter(Vec3 feetPosition,
    const CharacterMotorSettings3D& settings) {
    if (m_impl->character == nullptr) {
        createCharacter(feetPosition, settings);
        return;
    }
    m_impl->character->resize(capsuleCylinderHeight(settings.standingHeight,
        settings.radius));
    m_impl->character->setFootPosition(toExtended(feetPosition));
    m_impl->characterState = {};
    m_impl->characterState.position = feetPosition
        + Vec3 { 0.0f, 0.0f, settings.standingHeight * 0.5f };
    m_impl->coyoteRemaining = 0.0f;
    m_impl->jumpBufferRemaining = 0.0f;
}

void PhysicsScene3D::moveCharacter(const CharacterMotorCommand3D& command,
    const CharacterMotorSettings3D& settings, float deltaTime) {
    if (m_impl->character == nullptr || deltaTime <= 0.0f) return;
    PhysicsCharacterState3D& state = m_impl->characterState;
    state.flightExitBlocked = false;

    if (command.toggleFlight) {
        if (state.flying) {
            const float activeHeight = state.crouched
                ? settings.crouchedHeight : settings.standingHeight;
            if (m_impl->characterCapsuleIsClear(activeHeight,
                    settings.radius)) {
                state.flying = false;
                // A velocidade de voo no instante da troca e preservada; a
                // gravidade e o controle aereo voltam a atuar normalmente.
            } else {
                state.flightExitBlocked = true;
            }
        } else {
            state.flying = true;
            state.velocity = {};
        }
    }

    const bool wantsCrouch = command.crouch;
    if (wantsCrouch != state.crouched) {
        const float targetHeight = wantsCrouch
            ? settings.crouchedHeight : settings.standingHeight;
        // Reduzir a capsula sempre e seguro. Para levantar, um overlap explicito
        // impede que resize() coloque o jogador dentro de um teto ou prop.
        if (wantsCrouch || m_impl->characterCapsuleIsClear(targetHeight,
                settings.radius)) {
            m_impl->character->resize(capsuleCylinderHeight(targetHeight,
                settings.radius));
            state.crouched = wantsCrouch;
        }
    }

    Vec3 inputDirection = command.moveDirection;
    if (inputDirection.lengthSquared() > 1.0f) {
        inputDirection = inputDirection.normalized();
    }
    if (state.flying) {
        const float speed = command.sprint
            ? settings.fastFlightSpeed : settings.flightSpeed;
        // Voo editorial deliberadamente sem inercia: soltar a tecla zera o
        // movimento, mantendo controle preciso. A saida do modo preserva a
        // velocidade corrente conforme tratado acima.
        state.velocity = inputDirection * speed;
    } else {
        const float speed = state.crouched ? settings.crouchedSpeed
            : command.sprint ? settings.sprintSpeed : settings.walkSpeed;
        Vec3 desired = inputDirection;
        desired.z = 0.0f;
        if (desired.lengthSquared() > 1.0f) desired = desired.normalized();
        desired *= speed;
        const Vec3 currentPlanar { state.velocity.x, state.velocity.y, 0.0f };
        const float acceleration = state.grounded
            ? (desired.lengthSquared() > 0.0f
                ? settings.groundAcceleration : settings.groundDeceleration)
            : settings.airAcceleration;
        const Vec3 planar = moveToward(currentPlanar, desired,
            acceleration * deltaTime);
        state.velocity.x = planar.x;
        state.velocity.y = planar.y;

        m_impl->coyoteRemaining = state.grounded
            ? settings.coyoteTime
            : std::max(0.0f, m_impl->coyoteRemaining - deltaTime);
        if (command.jumpPressed) {
            m_impl->jumpBufferRemaining = settings.jumpBufferTime;
        } else {
            m_impl->jumpBufferRemaining = std::max(0.0f,
                m_impl->jumpBufferRemaining - deltaTime);
        }
        if (m_impl->jumpBufferRemaining > 0.0f
            && m_impl->coyoteRemaining > 0.0f) {
            state.velocity.z = settings.jumpSpeed;
            state.grounded = false;
            m_impl->jumpBufferRemaining = 0.0f;
            m_impl->coyoteRemaining = 0.0f;
        } else {
            state.velocity.z = std::max(-settings.maximumFallSpeed,
                state.velocity.z - 9.81f * settings.gravityScale * deltaTime);
        }
    }

    m_impl->characterMoveVelocity = state.velocity;
    m_impl->characterPushSpeedLimit = settings.maximumPushSpeedMetersPerSecond;
    m_impl->characterPushSaturationPenetration = std::max(0.001f,
        settings.pushSaturationPenetrationMeters);
    const physx::PxFilterData collisionFilter = state.flying
        ? physx::PxFilterData(0u, 0u, 0u, 0u)
        : physx::PxFilterData(1u, 0xFFFFFFFFu, 0u, 0u);
    CharacterQueryFilter queryFilter(m_impl->character->getActor());
    const physx::PxControllerFilters filters(&collisionFilter, &queryFilter);
    const physx::PxControllerCollisionFlags flags =
        m_impl->character->move(toPhysX(state.velocity * deltaTime),
            0.0001f, deltaTime, filters);
    state.grounded = !state.flying
        && flags.isSet(physx::PxControllerCollisionFlag::eCOLLISION_DOWN);
    if (state.grounded && state.velocity.z < 0.0f) state.velocity.z = 0.0f;
    if (flags.isSet(physx::PxControllerCollisionFlag::eCOLLISION_UP)
        && state.velocity.z > 0.0f) {
        state.velocity.z = 0.0f;
    }
    const Vec3 feet = fromExtended(m_impl->character->getFootPosition());
    const float bodyHeight = state.crouched
        ? settings.crouchedHeight : settings.standingHeight;
    state.position = feet + Vec3 { 0.0f, 0.0f, bodyHeight * 0.5f };
}

bool PhysicsScene3D::hasCharacter() const {
    return m_impl->character != nullptr;
}

const PhysicsCharacterState3D& PhysicsScene3D::characterState() const {
    return m_impl->characterState;
}

void PhysicsScene3D::setAirVelocity(Vec3 velocity) {
    m_impl->settings.airVelocity = velocity;
}

void PhysicsScene3D::simulate(float deltaTime) {
    if (deltaTime <= 0.0f || !std::isfinite(deltaTime)) return;
    using Clock = std::chrono::steady_clock;
    const auto stepStart = Clock::now();
    m_impl->contactImpacts.clear();

    // Arrasto quadratico e uma forca externa, nao parte do solver de contato.
    // Ele e aplicado somente a corpos acordados; props em sleep continuam com
    // custo zero e nao sao despertados por um ar parado.
    for (Impl::BodyRecord* record : m_impl->activeAerodynamicBodies) {
        if (record == nullptr || record->frozen) {
            continue;
        }
        auto* dynamic = record->actor->is<physx::PxRigidDynamic>();
        if (dynamic == nullptr || dynamic->isSleeping()
            || dynamic->getRigidBodyFlags()
                .isSet(physx::PxRigidBodyFlag::eKINEMATIC)) {
            continue;
        }
        Vec3 effectiveAirVelocity = m_impl->settings.airVelocity;
        const float windSpeed = effectiveAirVelocity.length();
        if (windSpeed > 0.01f
            && m_impl->settings.windShelterDistanceMeters > 0.0f) {
            // O vento vem de fora - se houver parede/geometria estatica logo
            // a barlavento (entre o corpo e de onde o vento sopra), o ar ali
            // fica parado/turbulento, nao com a velocidade livre do vento
            // aberto. Sem isso, uma sala fechada empurraria props com a
            // mesma forca que um terreno aberto, o que e fisicamente errado.
            const Vec3 bodyPosition =
                fromPhysX(dynamic->getGlobalPose().p);
            const Vec3 upwindDirection =
                effectiveAirVelocity * (-1.0f / windSpeed);
            const Ray3D upwindRay { bodyPosition, upwindDirection };
            PhysicsRayHit3D shelterHit;
            if (raycastStatic(upwindRay,
                    m_impl->settings.windShelterDistanceMeters, shelterHit)) {
                // Falloff suave (smoothstep) em vez de um corte abrupto entre
                // "vento total" e "nada": parede colada bloqueia quase tudo,
                // parede no limite do alcance mal se nota.
                const float t = std::clamp(shelterHit.distance
                    / m_impl->settings.windShelterDistanceMeters, 0.0f, 1.0f);
                const float exposure = t * t * (3.0f - 2.0f * t);
                effectiveAirVelocity = effectiveAirVelocity * exposure;
            }
        }

        const Vec3 relativeVelocity = fromPhysX(dynamic->getLinearVelocity())
            - effectiveAirVelocity;
        const float speedSquared = relativeVelocity.lengthSquared();
        if (speedSquared <= 1.0e-8f) continue;
        const float speed = std::sqrt(speedSquared);
        float forceMagnitude = 0.5f
            * std::max(0.0f,
                m_impl->settings.airDensityKgPerCubicMeter)
            * std::max(0.0f,
                record->definition.aerodynamicDragCoefficient)
            * std::max(0.0f, record->definition
                .aerodynamicReferenceAreaSquareMeters)
            * speedSquared;
        // Um passo discreto nao pode remover mais momento que o existente;
        // esse limite fisico evita inverter instantaneamente um corpo leve.
        forceMagnitude = std::min(forceMagnitude,
            dynamic->getMass() * speed / deltaTime);
        dynamic->addForce(toPhysX(relativeVelocity
            * (-forceMagnitude / speed)), physx::PxForceMode::eFORCE, false);
    }
    const auto dispatchStart = Clock::now();
    // O custo de particionar uma cena pequena pode superar o trabalho do
    // solver. A estimativa combina atores ativos e contatos do passo anterior
    // e cresce gradualmente ate o limite do pool compartilhado.
    const std::size_t estimatedWork = std::max(
        m_impl->activeAerodynamicBodies.size(),
        m_impl->diagnostics.activeDynamicBodyCount)
        + m_impl->diagnostics.discreteContactPairs * 2;
    const std::uint32_t desiredWorkers = static_cast<std::uint32_t>(
        std::max<std::size_t>(1, (estimatedWork + 127) / 128));
    m_impl->dispatcher->setWorkerCount(desiredWorkers);
    m_impl->diagnostics.physicsWorkerCount =
        m_impl->dispatcher->getWorkerCount();
    m_impl->scene->simulate(deltaTime, nullptr, m_impl->scratchBlock,
        m_impl->scratchBlockSize);
    const auto dispatchEnd = Clock::now();
    if (!m_impl->scene->fetchResults(true)) {
        throw std::runtime_error("PhysX nao concluiu o passo de simulacao");
    }
    const auto fetchEnd = Clock::now();

    // eENABLE_ACTIVE_ACTORS fornece apenas atores que alteraram estado neste
    // passo. O mesmo conjunto alimenta arrasto do proximo passo e snapshots
    // graficos, eliminando varreduras e leituras nativas de corpos em sleep.
    m_impl->activeAerodynamicBodies.clear();
    m_impl->activeBodyStateUpdates.clear();
    physx::PxU32 activeActorCount = 0;
    physx::PxActor** activeActors =
        m_impl->scene->getActiveActors(activeActorCount);
    for (physx::PxU32 index = 0; index < activeActorCount; ++index) {
        auto* actor = activeActors[index] != nullptr
            ? activeActors[index]->is<physx::PxRigidActor>() : nullptr;
        auto* record = actor != nullptr
            ? static_cast<Impl::BodyRecord*>(actor->userData) : nullptr;
        if (record == nullptr) continue;
        auto* dynamic = actor->is<physx::PxRigidDynamic>();
        if (dynamic != nullptr && record->definition.aerodynamicDragEnabled
            && !record->frozen && !dynamic->isSleeping()
            && !dynamic->getRigidBodyFlags().isSet(
                physx::PxRigidBodyFlag::eKINEMATIC)) {
            m_impl->activeAerodynamicBodies.push_back(record);
        }

        PhysicsBodyStateUpdate3D update;
        update.body = record->handle;
        const physx::PxTransform pose = actor->getGlobalPose();
        update.state.position = fromPhysX(pose.p);
        update.state.orientation = fromPhysX(pose.q);
        update.state.frozen = record->frozen;
        if (dynamic != nullptr) {
            update.state.linearVelocity =
                fromPhysX(dynamic->getLinearVelocity());
            update.state.angularVelocity =
                fromPhysX(dynamic->getAngularVelocity());
            update.state.sleeping = dynamic->isSleeping();
        }
        m_impl->activeBodyStateUpdates.push_back(update);
    }

    physx::PxSimulationStatistics statistics;
    m_impl->scene->getSimulationStatistics(statistics);
    m_impl->diagnostics.staticBodyCount = statistics.nbStaticBodies;
    m_impl->diagnostics.dynamicBodyCount = statistics.nbDynamicBodies;
    m_impl->diagnostics.activeDynamicBodyCount =
        statistics.nbActiveDynamicBodies;
    m_impl->diagnostics.sleepingDynamicBodyCount =
        statistics.nbDynamicBodies >= statistics.nbActiveDynamicBodies
        ? statistics.nbDynamicBodies - statistics.nbActiveDynamicBodies : 0;
    m_impl->diagnostics.broadPhaseAdds = statistics.getNbBroadPhaseAdds();
    m_impl->diagnostics.broadPhaseRemoves = statistics.getNbBroadPhaseRemoves();
    m_impl->diagnostics.discreteContactPairs =
        statistics.nbDiscreteContactPairsTotal;
    std::size_t ccdPairs = 0;
    for (physx::PxU32 first = 0;
        first < physx::PxGeometryType::eGEOMETRY_COUNT; ++first) {
        for (physx::PxU32 second = 0;
            second < physx::PxGeometryType::eGEOMETRY_COUNT; ++second) {
            ccdPairs += statistics.getRbPairStats(
                physx::PxSimulationStatistics::eCCD_PAIRS,
                static_cast<physx::PxGeometryType::Enum>(first),
                static_cast<physx::PxGeometryType::Enum>(second));
        }
    }
    m_impl->diagnostics.ccdPairs = ccdPairs;
    m_impl->diagnostics.simulationDispatchMilliseconds =
        std::chrono::duration<float, std::milli>(
            dispatchEnd - dispatchStart).count();
    m_impl->diagnostics.simulationWaitMilliseconds =
        std::chrono::duration<float, std::milli>(
            fetchEnd - dispatchEnd).count();
    m_impl->diagnostics.totalStepMilliseconds =
        std::chrono::duration<float, std::milli>(
            Clock::now() - stepStart).count();
}

std::span<const ContactImpactEvent3D>
PhysicsScene3D::contactImpacts() const {
    return m_impl->contactImpacts;
}

const PhysicsStepDiagnostics3D& PhysicsScene3D::diagnostics() const {
    return m_impl->diagnostics;
}

std::span<const PhysicsBodyStateUpdate3D>
PhysicsScene3D::activeBodyStates() const {
    return m_impl->activeBodyStateUpdates;
}

} // namespace MatterEngine
