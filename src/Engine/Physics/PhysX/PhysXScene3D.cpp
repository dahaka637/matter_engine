#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollInteractions3D.hpp"

#include "Engine/Environment/OceanSurface.hpp"
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
#include <optional>
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
constexpr physx::PxU32 FilterFlagContactReports = 1u << 1;
constexpr physx::PxU32 RagdollCollisionLayer = 1u << 1;
constexpr physx::PxU32 RagdollLinkTokenBits = 8u;
constexpr physx::PxU32 RagdollLinkTokenMask =
    (1u << RagdollLinkTokenBits) - 1u;
constexpr physx::PxU32 RagdollContactSensorFlag = 1u << 31;
constexpr physx::PxU32 RagdollPackedIdentityMask =
    ~RagdollContactSensorFlag;
constexpr std::uint32_t InvalidRagdollDof =
    std::numeric_limits<std::uint32_t>::max();

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
        m_submittedTasks.fetch_add(1, std::memory_order_relaxed);
        // A implementação oficial PxDefaultCpuDispatcher faz exatamente
        // isto quando configurada com zero workers. Para cenas pequenas,
        // executar a cadeia diretamente evita mutex, wake-up e troca de
        // contexto para dezenas de tarefas menores que o custo de agendá-las.
        if (m_currentWorkerCount.load(std::memory_order_relaxed) == 0) {
            task.run();
            task.release();
            return;
        }
        m_scheduler->submit({ &executePhysXTask, &task });
    }

    [[nodiscard]] std::uint32_t getWorkerCount() const override {
        return m_currentWorkerCount.load(std::memory_order_acquire);
    }

    void setWorkerCount(std::uint32_t count) {
        m_currentWorkerCount.store(std::min(count,
            m_advertisedWorkerCount), std::memory_order_release);
    }

    void resetStepCounters() {
        m_submittedTasks.store(0, std::memory_order_relaxed);
    }

    [[nodiscard]] std::size_t submittedTaskCount() const {
        return m_submittedTasks.load(std::memory_order_relaxed);
    }

private:
    static void executePhysXTask(void* context) noexcept {
        auto* task = static_cast<physx::PxBaseTask*>(context);
        task->run();
        task->release();
    }

    std::shared_ptr<TaskScheduler> m_scheduler;
    std::uint32_t m_advertisedWorkerCount = 1;
    std::atomic<std::uint32_t> m_currentWorkerCount { 0 };
    std::atomic<std::size_t> m_submittedTasks { 0 };
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

    const bool isRagdoll0 =
        (filterData0.word0 & RagdollCollisionLayer) != 0;
    const bool isRagdoll1 =
        (filterData1.word0 & RagdollCollisionLayer) != 0;
    if (isRagdoll0 && isRagdoll1) {
        const physx::PxU32 ragdoll0 =
            (filterData0.word3 & RagdollPackedIdentityMask)
                >> RagdollLinkTokenBits;
        const physx::PxU32 ragdoll1 =
            (filterData1.word3 & RagdollPackedIdentityMask)
                >> RagdollLinkTokenBits;
        if (ragdoll0 != 0 && ragdoll0 == ragdoll1) {
            const physx::PxU32 token0 =
                filterData0.word3 & RagdollLinkTokenMask;
            const physx::PxU32 token1 =
                filterData1.word3 & RagdollLinkTokenMask;
            if (token0 == 0 || token1 == 0
                || token0 > 32u || token1 > 32u
                || (filterData0.word2 & (1u << (token1 - 1u))) == 0
                || (filterData1.word2 & (1u << (token0 - 1u))) == 0) {
                // A matriz anatômica é imutável durante a vida das shapes.
                // eKILL evita até o placeholder de interação que eSUPPRESS
                // manteria para um possível resetFiltering futuro.
                return physx::PxFilterFlag::eKILL;
            }
        }
    }

    pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;
    // Relatórios são uma saída auxiliar para áudio, não parte da resolução
    // física. Links de ragdoll não possuem BodyRecord/material acústico e
    // nunca devem gerar um stream detalhado de contato que será descartado.
    const bool containsRagdoll = isRagdoll0 || isRagdoll1;
    const bool wantsContactReports =
        ((filterData0.word2 | filterData1.word2)
            & FilterFlagContactReports) != 0;
    if ((wantsContactReports && !containsRagdoll)
        || containsRagdoll) {
        pairFlags |= physx::PxPairFlag::eNOTIFY_TOUCH_FOUND
            // Contato que continua tocando passo a passo (nao so o instante
            // em que comecou) alimenta o som de arrasto/atrito.
            | physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS
            | physx::PxPairFlag::eNOTIFY_CONTACT_POINTS;
    }
    const bool wantsContinuousCollision =
        (!isRagdoll0
            && (filterData0.word2 & FilterFlagContinuousCollision) != 0)
        || (!isRagdoll1
            && (filterData1.word2 & FilterFlagContinuousCollision) != 0);
    if (wantsContinuousCollision) {
        pairFlags |= physx::PxPairFlag::eDETECT_CCD_CONTACT;
    }
    return physx::PxFilterFlag::eDEFAULT;
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

std::uint32_t ragdollSelfCollisionMask(
    const RagdollProfile3D& profile, std::size_t linkIndex) {
    // O proprio PhysX nunca cria contato pai-filho numa articulation. Todos
    // os demais pares precisam permanecer ativos: em particular, o Chest
    // (avo) funciona como barreira real para o UpperArm cujo pai e
    // UpperChest. Desligar avo/neto foi o que permitiu o braço atravessar o
    // peito mesmo com a self-collision global ligada.
    const auto isDirectParent = [&](std::size_t parent,
            std::size_t child) {
        return profile.links[child].parentIndex
            == static_cast<int>(parent);
    };

    std::uint32_t mask = 0;
    for (std::size_t other = 0; other < profile.links.size(); ++other) {
        if (other == linkIndex) continue;
        if (!isDirectParent(linkIndex, other)
            && !isDirectParent(other, linkIndex)) {
            mask |= 1u << other;
        }
    }
    return mask;
}

physx::PxArticulationAxis::Enum toPhysX(RagdollAxis3D axis) {
    switch (axis) {
    case RagdollAxis3D::Twist:
        return physx::PxArticulationAxis::eTWIST;
    case RagdollAxis3D::Swing1:
        return physx::PxArticulationAxis::eSWING1;
    case RagdollAxis3D::Swing2:
        return physx::PxArticulationAxis::eSWING2;
    }
    return physx::PxArticulationAxis::eTWIST;
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

    struct RagdollJointRuntime {
        physx::PxArticulationJointReducedCoordinate* joint = nullptr;
        std::array<float, 3> targets {};
        std::array<std::uint32_t, 3> dofIndices {
            std::numeric_limits<std::uint32_t>::max(),
            std::numeric_limits<std::uint32_t>::max(),
            std::numeric_limits<std::uint32_t>::max()
        };
    };

    struct RagdollRecord {
        RagdollHandle3D handle;
        std::uint64_t entityId = 0;
        RagdollProfile3D profile;
        physx::PxArticulationReducedCoordinate* articulation = nullptr;
        physx::PxArticulationCache* cache = nullptr;
        physx::PxAggregate* aggregate = nullptr;
        std::vector<physx::PxArticulationLink*> links;
        // Alinhado com profile.links; o elemento zero não possui inbound joint.
        std::vector<RagdollJointRuntime> joints;
        RagdollState3D state;
        std::vector<RagdollContactPoint3D> stepContacts;
        std::vector<RagdollInteraction3D> stepInteractions;
        std::vector<RagdollDriveTarget3D> stagedActiveTargets;
        RagdollAnimationConstraint3D animationConstraint;
        physx::PxVec3 animationStepOrigin{0.0f};
        bool animationConstraintStaged = false;
        float externalForceMagnitude = 0.0f;
        float rigidityPercent = 0.0f;
        float passiveRigidityPercent = 0.0f;
        bool active = false;
        bool frozen = false;
        bool stagedActiveTargetsDirty = false;
        bool gravityCompensationEnabled = false;
        bool stagedGravityCompensationEnabled = false;
        bool stateInitialized = false;
    };

    struct RagdollSlot {
        std::uint32_t generation = 1;
        std::unique_ptr<RagdollRecord> record;
    };

    enum class RagdollCommandType : std::uint8_t {
        SetRigidity,
        CapturePose,
        NeutralPose,
        Release,
        SetActive,
        SetFrozen
    };

    struct RagdollCommand {
        RagdollHandle3D handle;
        RagdollCommandType type = RagdollCommandType::SetRigidity;
        float value = 0.0f;
    };

    std::shared_ptr<PhysicsEngine3D::Impl> engine;
    PhysicsSceneSettings3D settings;
    std::unique_ptr<MatterCpuDispatcher> dispatcher;
    physx::PxScene* scene = nullptr;
    std::unordered_map<std::string, physx::PxMaterial*> materials;
    std::vector<BodySlot> bodySlots;
    std::vector<std::uint32_t> freeBodySlots;
    std::vector<RagdollSlot> ragdollSlots;
    std::vector<std::uint32_t> freeRagdollSlots;
    std::vector<RagdollCommand> pendingRagdollCommands;
    std::vector<ContactImpactEvent3D> contactImpacts;
    std::vector<ContactSlideEvent3D> contactSlides;
    std::vector<PhysicsBodyStateUpdate3D> activeBodyStateUpdates;
    std::vector<BodyRecord*> activeAerodynamicBodies;
    std::optional<OceanVolume3D> ocean;
    float oceanTimeSeconds = 0.0f;
    // Apenas atores acordados entram nesta lista; o custo do oceano continua
    // proporcional ao que realmente está se movendo.
    std::vector<BodyRecord*> activeOceanBodies;
    PhysicsStepDiagnostics3D diagnostics;
    std::atomic<std::uint64_t> contactCallbackNanoseconds { 0 };
    std::atomic<std::size_t> reportedContactPairs { 0 };
    std::atomic<std::size_t> reportedContactPoints { 0 };
    std::vector<std::byte> scratchStorage;
    void* scratchBlock = nullptr;
    std::uint32_t scratchBlockSize = 0;

    physx::PxD6Joint* grabJoint = nullptr;
    PhysicsBodyHandle3D grabBody;
    RagdollHandle3D grabRagdoll;
    std::uint32_t grabRagdollLink =
        std::numeric_limits<std::uint32_t>::max();

    physx::PxControllerManager* controllerManager = nullptr;
    physx::PxCapsuleController* character = nullptr;
    PhysicsCharacterState3D characterState;
    float coyoteRemaining = 0.0f;
    float jumpBufferRemaining = 0.0f;
    float jumpChargeSeconds = 0.0f;
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

    [[nodiscard]] RagdollRecord* ragdoll(RagdollHandle3D handle) {
        if (!handle.valid() || handle.index >= ragdollSlots.size()) {
            return nullptr;
        }
        RagdollSlot& slot = ragdollSlots[handle.index];
        return slot.generation == handle.generation
            ? slot.record.get() : nullptr;
    }

    [[nodiscard]] const RagdollRecord* ragdoll(
        RagdollHandle3D handle) const {
        if (!handle.valid() || handle.index >= ragdollSlots.size()) {
            return nullptr;
        }
        const RagdollSlot& slot = ragdollSlots[handle.index];
        return slot.generation == handle.generation
            ? slot.record.get() : nullptr;
    }

    [[nodiscard]] RagdollRecord* ragdollFromFilterData(
        const physx::PxFilterData& data,
        std::uint32_t& linkIndex) {
        if ((data.word0 & RagdollCollisionLayer) == 0) return nullptr;
        const physx::PxU32 packed =
            data.word3 & RagdollPackedIdentityMask;
        const physx::PxU32 instance = packed >> RagdollLinkTokenBits;
        const physx::PxU32 token = packed & RagdollLinkTokenMask;
        if (instance == 0 || token == 0) return nullptr;
        const std::size_t slotIndex =
            static_cast<std::size_t>(instance - 1u);
        if (slotIndex >= ragdollSlots.size()
            || !ragdollSlots[slotIndex].record) {
            return nullptr;
        }
        RagdollRecord* record = ragdollSlots[slotIndex].record.get();
        linkIndex = token - 1u;
        return linkIndex < record->links.size() ? record : nullptr;
    }

    [[nodiscard]] bool findRagdollLink(const physx::PxRigidActor* actor,
        RagdollHandle3D& handle, std::uint32_t& linkIndex) const {
        if (actor == nullptr) return false;
        for (const RagdollSlot& slot : ragdollSlots) {
            if (!slot.record) continue;
            const RagdollRecord& record = *slot.record;
            for (std::size_t index = 0; index < record.links.size(); ++index) {
                if (record.links[index] != actor) continue;
                handle = record.handle;
                linkIndex = static_cast<std::uint32_t>(index);
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] physx::PxRigidBody* grabbedRigidBody() {
        if (BodyRecord* record = body(grabBody); record != nullptr) {
            return record->actor->is<physx::PxRigidDynamic>();
        }
        RagdollRecord* record = ragdoll(grabRagdoll);
        if (record == nullptr || grabRagdollLink >= record->links.size()) {
            return nullptr;
        }
        return record->links[grabRagdollLink];
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
        grabRagdoll = {};
        grabRagdollLink = std::numeric_limits<std::uint32_t>::max();
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
        float radius, bool ignoreRagdolls = false) const {
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
        const physx::PxU32 mask = ignoreRagdolls
            ? (0xFFFFFFFFu & ~RagdollCollisionLayer) : 0xFFFFFFFFu;
        const physx::PxFilterData collisionFilter(1u, mask, 0, 0);
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
        for (RagdollSlot& slot : ragdollSlots) {
            if (slot.record && slot.record->cache != nullptr) {
                slot.record->cache->release();
                slot.record->cache = nullptr;
            }
            if (slot.record && slot.record->articulation != nullptr) {
                slot.record->articulation->release();
                slot.record->articulation = nullptr;
            }
            if (slot.record && slot.record->aggregate != nullptr) {
                slot.record->aggregate->release();
                slot.record->aggregate = nullptr;
            }
            slot.record.reset();
        }
        pendingRagdollCommands.clear();
        freeRagdollSlots.clear();
        activeAerodynamicBodies.clear();
        activeOceanBodies.clear();
        activeBodyStateUpdates.clear();
        ocean.reset();
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
        using ContactClock = std::chrono::steady_clock;
        const auto callbackStart = ContactClock::now();
        struct CallbackTimer {
            std::atomic<std::uint64_t>& destination;
            ContactClock::time_point start;
            ~CallbackTimer() {
                const auto elapsed = std::chrono::duration_cast<
                    std::chrono::nanoseconds>(ContactClock::now() - start);
                destination.fetch_add(
                    static_cast<std::uint64_t>(elapsed.count()),
                    std::memory_order_relaxed);
            }
        } callbackTimer { contactCallbackNanoseconds, callbackStart };
        const auto* recordA = header.actors[0] != nullptr
            ? static_cast<const BodyRecord*>(header.actors[0]->userData)
            : nullptr;
        const auto* recordB = header.actors[1] != nullptr
            ? static_cast<const BodyRecord*>(header.actors[1]->userData)
            : nullptr;
        // Velocidade do PONTO material do corpo que coincide com worldPoint
        // neste instante (translacao + rotacao) - formula classica de corpo
        // rigido v_ponto = v_linear + w x (worldPoint - centroDeMassa).
        // Corpos estaticos/kinematicos (dynamic == nullptr) nao tem
        // velocidade propria aqui, retorna zero. Usada so pelo ramo de
        // arrasto (ver mais abaixo) para medir deslizamento tangencial sem
        // depender do impulso do solver, que so reporta o componente normal.
        const auto contactPointVelocity = [](const BodyRecord* record,
                const physx::PxVec3& worldPoint) -> physx::PxVec3 {
            if (record == nullptr) return physx::PxVec3(0.0f);
            auto* dynamic = record->actor->is<physx::PxRigidDynamic>();
            if (dynamic == nullptr) return physx::PxVec3(0.0f);
            const physx::PxVec3 comWorld = dynamic->getGlobalPose()
                .transform(dynamic->getCMassLocalPose()).p;
            return dynamic->getLinearVelocity()
                + dynamic->getAngularVelocity().cross(worldPoint - comWorld);
        };
        const auto actorPointVelocity = [](const physx::PxActor* actor,
                const physx::PxVec3& worldPoint) -> physx::PxVec3 {
            if (actor == nullptr) return physx::PxVec3(0.0f);
            const auto* rigidActor = actor->is<physx::PxRigidActor>();
            const auto* body = rigidActor != nullptr
                ? rigidActor->is<physx::PxRigidBody>() : nullptr;
            if (body == nullptr) return physx::PxVec3(0.0f);
            const physx::PxVec3 comWorld = body->getGlobalPose()
                .transform(body->getCMassLocalPose()).p;
            return body->getLinearVelocity()
                + body->getAngularVelocity().cross(worldPoint - comWorld);
        };

        for (physx::PxU32 pairIndex = 0; pairIndex < pairCount; ++pairIndex) {
            const physx::PxContactPair& pair = pairs[pairIndex];
            const physx::PxFilterData filterA = pair.shapes[0] != nullptr
                ? pair.shapes[0]->getSimulationFilterData()
                : physx::PxFilterData {};
            const physx::PxFilterData filterB = pair.shapes[1] != nullptr
                ? pair.shapes[1]->getSimulationFilterData()
                : physx::PxFilterData {};
            std::uint32_t ragdollLinkA = 0;
            std::uint32_t ragdollLinkB = 0;
            RagdollRecord* ragdollA =
                ragdollFromFilterData(filterA, ragdollLinkA);
            RagdollRecord* ragdollB =
                ragdollFromFilterData(filterB, ragdollLinkB);
            const bool sensorA = ragdollA != nullptr
                && (filterA.word3 & RagdollContactSensorFlag) != 0;
            const bool sensorB = ragdollB != nullptr
                && (filterB.word3 & RagdollContactSensorFlag) != 0;
            if (recordA == nullptr && recordB == nullptr
                && !ragdollA && !ragdollB) {
                continue;
            }
            // "Acabou de tocar" (impacto, ContactImpactEvent3D) e "continua
            // tocando" (arrasto/atrito, ContactSlideEvent3D) compartilham
            // toda a extracao abaixo - so o tipo de struct produzido no
            // final muda. Um pair pode reportar os dois eventos no mesmo
            // passo (ex.: primeiro contato que ja chega deslizando); tratar
            // como impacto nesse caso, o ramo de persistencia so importa a
            // partir do proximo passo.
            const bool isFound = pair.events.isSet(
                physx::PxPairFlag::eNOTIFY_TOUCH_FOUND);
            const bool isPersisting = !isFound && pair.events.isSet(
                physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS);
            if ((!isFound && !isPersisting) || pair.contactCount == 0) {
                continue;
            }
            if (!sensorA && !sensorB
                && (recordA != nullptr || recordB != nullptr)) {
                reportedContactPairs.fetch_add(1,
                    std::memory_order_relaxed);
                reportedContactPoints.fetch_add(pair.contactCount,
                    std::memory_order_relaxed);
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

            // Stream mínimo e separado do áudio. Somente links explicitamente
            // marcados como sensores chegam aqui, e cada manifold é limitado
            // para manter custo previsível mesmo com muitos personagens.
            constexpr std::size_t MaximumSensorPointsPerRagdoll = 64;
            const auto appendSensorPoints = [&](RagdollRecord& ragdoll,
                    std::uint32_t linkIndex, bool firstShape) {
                for (physx::PxU32 pointIndex = 0;
                        pointIndex < extracted
                        && ragdoll.stepContacts.size()
                            < MaximumSensorPointsPerRagdoll;
                        ++pointIndex) {
                    const physx::PxContactPairPoint& point =
                        points[pointIndex];
                    const physx::PxVec3 outwardNormal = firstShape
                        ? point.normal : -point.normal;
                    const physx::PxVec3 relativeVelocity =
                        actorPointVelocity(header.actors[0], point.position)
                        - actorPointVelocity(
                            header.actors[1], point.position);
                    const physx::PxVec3 tangent = relativeVelocity
                        - point.normal * relativeVelocity.dot(point.normal);
                    RagdollContactPoint3D contact;
                    contact.linkIndex = linkIndex;
                    const auto* otherActor=header.actors[firstShape?1:0];
                    contact.otherBodyDynamic=otherActor && !otherActor->is<physx::PxRigidStatic>();
                    contact.otherMotion = !contact.otherBodyDynamic
                        ? RagdollContactMotion3D::Static
                        : otherActor->is<physx::PxRigidDynamic>()
                            && otherActor->is<physx::PxRigidDynamic>()->getRigidBodyFlags()
                                .isSet(physx::PxRigidBodyFlag::eKINEMATIC)
                            ? RagdollContactMotion3D::Kinematic
                            : RagdollContactMotion3D::Dynamic;
                    // Impulso do relatorio de contato: depois do solver.
                    contact.impulseEstimated = false;
                    contact.position = fromPhysX(point.position);
                    contact.normal = fromPhysX(outwardNormal).normalized();
                    contact.normalImpulseNewtonSeconds =
                        std::abs(point.impulse.dot(point.normal));
                    contact.tangentialSpeedMetersPerSecond =
                        tangent.magnitude();
                    ragdoll.stepContacts.push_back(contact);
                }
            };
            // Self-collision remains physical, but cannot count as support
            // from the environment (a foot on its own shin is not ground).
            if (sensorA && ragdollA != ragdollB) {
                appendSensorPoints(*ragdollA, ragdollLinkA, true);
            }
            if (sensorB && ragdollA != ragdollB) {
                appendSensorPoints(*ragdollB, ragdollLinkB, false);
            }
            const auto appendInteractions = [&](RagdollRecord* ragdoll,
                std::uint32_t link, bool first) {
                if (!ragdoll || ragdollA == ragdollB) return;
                const auto* actor = header.actors[first ? 1 : 0];
                const auto* dynamic = actor ? actor->is<physx::PxRigidDynamic>() : nullptr;
                const auto motion = dynamic && dynamic->getRigidBodyFlags().isSet(physx::PxRigidBodyFlag::eKINEMATIC)
                    ? RagdollContactMotion3D::Kinematic
                    : actor && !actor->is<physx::PxRigidStatic>()
                        ? RagdollContactMotion3D::Dynamic : RagdollContactMotion3D::Static;
                for (physx::PxU32 i = 0; i < extracted; ++i) {
                    const auto& p = points[i];
                    const auto velocity = actorPointVelocity(header.actors[first ? 0 : 1], p.position)
                        - actorPointVelocity(actor, p.position);
                    accumulateRagdollInteraction3D(ragdoll->stepInteractions, link, motion,
                        fromPhysX(p.position), fromPhysX(first ? p.normal : -p.normal),
                        fromPhysX(velocity), std::abs(p.impulse.dot(p.normal)), false, false);
                }
            };
            appendInteractions(ragdollA, ragdollLinkA, true);
            appendInteractions(ragdollB, ragdollLinkB, false);
            // Sensores biomecânicos não atravessam a resolução acústica.
            // Isso preserva o custo e o significado do stream de áudio.
            if (ragdollA || ragdollB) continue;

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
            // NAO usar totalImpulse pra medir atrito: PxContactPairPoint::
            // impulse so reporta o componente NORMAL do impulso resolvido
            // pelo solver (confirmado empiricamente - um corpo deslizando
            // de verdade sob forca real reporta impulso tangencial
            // EXATAMENTE zero por este campo, mesmo com atrito configurado
            // e velocidade relativa clara). A velocidade de deslizamento
            // usada pelo som de arrasto (ver ContactSlideEvent3D mais
            // abaixo) precisa vir da CINEMATICA dos corpos, nao do impulso.
            //
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
            const std::size_t bodyIndexA = recordA != nullptr
                ? recordA->handle.index : InvalidPhysicsBodyIndex;
            const std::size_t bodyIndexB = recordB != nullptr
                ? recordB->handle.index : InvalidPhysicsBodyIndex;
            const std::uint64_t bodyEntityIdA = recordA != nullptr
                ? recordA->definition.entityId : 0;
            const std::uint64_t bodyEntityIdB = recordB != nullptr
                ? recordB->definition.entityId : 0;
            const std::string materialIdA = shapeA != nullptr
                ? shapeA->materialId
                : recordA != nullptr ? recordA->definition.materialId : "default";
            const std::string materialIdB = shapeB != nullptr
                ? shapeB->materialId
                : recordB != nullptr ? recordB->definition.materialId : "default";

            if (isFound) {
                ContactImpactEvent3D impact;
                impact.bodyA = bodyIndexA;
                impact.bodyB = bodyIndexB;
                impact.bodyIdA = bodyEntityIdA;
                impact.bodyIdB = bodyEntityIdB;
                impact.materialA = materialIdA;
                impact.materialB = materialIdB;
                impact.position = fromPhysX(representative->position);
                impact.normal = fromPhysX(normal);
                impact.normalImpulseNewtonSeconds = normalImpulse;
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
            } else {
                // Velocidade relativa no ponto de contato (cinematica, nao
                // impulso - ver comentario de contactPointVelocity acima) -
                // a componente TANGENCIAL (perpendicular a normal) e o que
                // realmente significa "deslizando": rolamento sem deslizar
                // tem essa componente quase nula por definicao fisica,
                // mesmo com velocidade linear alta.
                const physx::PxVec3 relativeVelocity =
                    contactPointVelocity(recordA, representative->position)
                    - contactPointVelocity(recordB, representative->position);
                const physx::PxVec3 tangentialVelocity = relativeVelocity
                    - normal * relativeVelocity.dot(normal);

                ContactSlideEvent3D slide;
                slide.bodyA = bodyIndexA;
                slide.bodyB = bodyIndexB;
                slide.bodyIdA = bodyEntityIdA;
                slide.bodyIdB = bodyEntityIdB;
                slide.materialA = materialIdA;
                slide.materialB = materialIdB;
                slide.position = fromPhysX(representative->position);
                slide.normalImpulseNewtonSeconds = normalImpulse;
                slide.tangentialSpeedMetersPerSecond =
                    tangentialVelocity.magnitude();
                slide.effectiveMassKg = effectiveMass;
                slide.massA = massA;
                slide.massB = massB;
                slide.staticA = massA <= 0.0f;
                slide.staticB = massB <= 0.0f;
                contactSlides.push_back(std::move(slide));
            }
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
    // A interface neutra usa 0 como "o backend escolhe": estes sao os valores
    // calibrados para o solver TGS do PhysX em passo fixo de 120 Hz. Uma ilha
    // usa a maior contagem pedida por qualquer ator; elevar isto globalmente
    // duplicava o trabalho de todos os ragdolls.
    if (m_impl->settings.solverPositionIterations == 0) {
        m_impl->settings.solverPositionIterations = 4;
    }
    if (m_impl->settings.solverVelocityIterations == 0) {
        m_impl->settings.solverVelocityIterations = 1;
    }
    if (m_impl->settings.scratchBufferSizeBytes == 0) {
        m_impl->settings.scratchBufferSizeBytes = 1024u * 1024u;
    }
    m_impl->contactImpacts.reserve(1024);
    m_impl->contactSlides.reserve(1024);
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
    // A documentação oficial recomenda ABP como escolha geral. PABP só
    // supera o ABP single-thread em cenas grandes e custa mais em cenas
    // pequenas/médias como os 22 jogadores deste projeto.
    descriptor.broadPhaseType = physx::PxBroadPhaseType::eABP;
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

    const std::uint32_t requestedScratch =
        m_impl->settings.scratchBufferSizeBytes;
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

namespace {

void captureRagdollTargets(
    PhysicsScene3D::Impl::RagdollRecord& record) {
    for (std::size_t linkIndex = 1;
            linkIndex < record.profile.links.size(); ++linkIndex) {
        auto& runtime = record.joints[linkIndex];
        const auto& definition =
            record.profile.links[linkIndex].inboundJoint;
        if (runtime.joint == nullptr) continue;
        for (std::size_t axisIndexValue = 0;
                axisIndexValue < definition.axes.size(); ++axisIndexValue) {
            if (!definition.axes[axisIndexValue].enabled) continue;
            const auto axis = static_cast<RagdollAxis3D>(axisIndexValue);
            runtime.targets[axisIndexValue] = std::clamp(
                runtime.joint->getJointPosition(toPhysX(axis)),
                definition.axes[axisIndexValue].minimumRadians,
                definition.axes[axisIndexValue].maximumRadians);
        }
    }
}

void applyRagdollRigidity(
    PhysicsScene3D::Impl::RagdollRecord& record,
    float rigidityPercent, bool captureWhenEnabling) {
    const float previous = record.rigidityPercent;
    const float normalized = std::clamp(rigidityPercent, 0.0f, 100.0f)
        * 0.01f;
    if (captureWhenEnabling && previous <= 0.001f
        && normalized > 0.00001f) {
        captureRagdollTargets(record);
    }
    record.rigidityPercent = normalized * 100.0f;
    record.state.rigidityPercent = record.rigidityPercent;

    const float stiffnessWeight = std::pow(normalized, 2.2f);
    const float dampingWeight = std::pow(normalized, 1.4f);
    const float torqueWeight = std::pow(normalized, 1.25f);
    for (std::size_t linkIndex = 1;
            linkIndex < record.profile.links.size(); ++linkIndex) {
        auto& runtime = record.joints[linkIndex];
        const auto& definition =
            record.profile.links[linkIndex].inboundJoint;
        if (runtime.joint == nullptr) continue;
        for (std::size_t axisIndexValue = 0;
                axisIndexValue < definition.axes.size(); ++axisIndexValue) {
            const RagdollAxisDefinition3D& axisDefinition =
                definition.axes[axisIndexValue];
            if (!axisDefinition.enabled) continue;
            const auto axis = toPhysX(
                static_cast<RagdollAxis3D>(axisIndexValue));
            const physx::PxArticulationDrive drive(
                axisDefinition.stiffness * stiffnessWeight,
                axisDefinition.damping * dampingWeight,
                axisDefinition.maximumTorque * torqueWeight,
                normalized > 0.00001f
                    ? physx::PxArticulationDriveType::eFORCE
                    : physx::PxArticulationDriveType::eNONE);
            runtime.joint->setDriveParams(axis, drive);
            runtime.joint->setDriveTarget(
                axis, runtime.targets[axisIndexValue], false);
            runtime.joint->setDriveVelocity(axis, 0.0f, false);
        }
    }
    if (normalized > 0.00001f && record.articulation->getScene() != nullptr) {
        record.articulation->wakeUp();
    }
}

void applyActiveRagdollTargets(
    PhysicsScene3D::Impl::RagdollRecord& record) {
    if (!record.active) return;
    for (const RagdollDriveTarget3D& target :
            record.stagedActiveTargets) {
        if (target.linkIndex == 0
            || target.linkIndex >= record.profile.links.size()) {
            continue;
        }
        const std::size_t axisIndexValue =
            static_cast<std::size_t>(target.axis);
        if (axisIndexValue >= 3) continue;
        auto& runtime = record.joints[target.linkIndex];
        const RagdollAxisDefinition3D& definition =
            record.profile.links[target.linkIndex]
                .inboundJoint.axes[axisIndexValue];
        if (runtime.joint == nullptr || !definition.enabled) continue;

        const float position = std::clamp(target.positionRadians,
            definition.minimumRadians, definition.maximumRadians);
        const float stiffnessScale =
            std::clamp(target.stiffnessScale, 0.0f, 2.0f);
        const float dampingScale =
            std::clamp(target.dampingScale, 0.0f, 2.5f);
        const float torqueScale =
            std::clamp(target.maximumTorqueScale, 0.0f, 3.0f);
        // Authored gains are torque/radian and torque/(radian/second).
        // Acceleration drives normalize by the free articulation response,
        // which can be dominated by a light foot even when contact makes that
        // ankle support the body. A large torque ceiling alone cannot correct
        // that underpowered servo. Keep muscle impedance in physical units.
        constexpr float ActiveStiffnessScale = 5.0f;
        constexpr float ActiveDampingScale = 2.0f;
        physx::PxArticulationDrive drive(
            definition.stiffness * stiffnessScale
                * ActiveStiffnessScale,
            definition.damping * dampingScale
                * ActiveDampingScale,
            definition.maximumTorque * torqueScale,
            torqueScale > 0.00001f
                ? physx::PxArticulationDriveType::eFORCE
                : physx::PxArticulationDriveType::eNONE);
        // PhysX 5.9 envelope caps the combined implicit motor + external
        // joint feedforward, unlike maxForce alone. Gravity compensation
        // must consume the same muscle budget as posture and balance.
        if (torqueScale > 0.00001f)
            drive.envelope = physx::PxPerformanceEnvelope(
                definition.maximumTorque * torqueScale, 20.0f, 0.0f, 0.0f);
        const auto axis = toPhysX(target.axis);
        runtime.joint->setDriveParams(axis, drive);
        runtime.joint->setDriveTarget(axis, position, false);
        runtime.joint->setDriveVelocity(axis,
            std::isfinite(target.velocityRadiansPerSecond)
                ? target.velocityRadiansPerSecond : 0.0f, false);
        runtime.targets[axisIndexValue] = position;
    }
    record.stagedActiveTargetsDirty = false;
}

void applyActiveRagdollFeedforward(
    PhysicsScene3D::Impl::RagdollRecord& record) {
    if (!record.active || record.cache == nullptr
        || record.articulation == nullptr) {
        return;
    }
    const physx::PxU32 dofCount = record.articulation->getDofs();
    std::fill_n(record.cache->jointForce, dofCount, 0.0f);

    if (record.gravityCompensationEnabled) {
    record.articulation->copyInternalStateToCache(*record.cache,
        physx::PxArticulationCacheFlag::ePOSITION
                | physx::PxArticulationCacheFlag::eROOT_TRANSFORM);
        record.articulation->applyCache(*record.cache,
            physx::PxArticulationCacheFlag::ePOSITION
                | physx::PxArticulationCacheFlag::eROOT_TRANSFORM,
            false);
        record.articulation->commonInit();
        record.articulation->computeGravityCompensation(*record.cache);
        for (physx::PxU32 dof = 0; dof < dofCount; ++dof) {
            // Floating base: os seis primeiros valores são o wrench da raiz.
            // Eles são deliberadamente descartados. Aplicar essa parcela
            // seria a força "mágica" na pelve que a arquitetura proíbe.
            record.cache->jointForce[dof] =
                record.cache->gravityCompensationForce[dof + 6u];
        }
    }
    for (const RagdollDriveTarget3D& target :
            record.stagedActiveTargets) {
        if (target.linkIndex == 0
            || target.linkIndex >= record.joints.size()) {
            continue;
        }
        const std::size_t axisIndexValue =
            static_cast<std::size_t>(target.axis);
        if (axisIndexValue >= 3
            || !std::isfinite(target.feedforwardTorqueNewtonMeters)) {
            continue;
        }
        const std::uint32_t dof =
            record.joints[target.linkIndex].dofIndices[axisIndexValue];
        if (dof == InvalidRagdollDof || dof >= dofCount) continue;
        const RagdollAxisDefinition3D& definition =
            record.profile.links[target.linkIndex]
                .inboundJoint.axes[axisIndexValue];
        const float torqueLimit = definition.maximumTorque
            * std::clamp(target.maximumTorqueScale, 0.0f, 3.0f);
        if (record.gravityCompensationEnabled
            && std::isfinite(target.gravityCompensationScale)) {
            record.cache->jointForce[dof] *= std::clamp(
                target.gravityCompensationScale, 0.0f, 1.0f);
        }
        record.cache->jointForce[dof] += std::clamp(
            target.feedforwardTorqueNewtonMeters,
            -torqueLimit, torqueLimit);
    }
    record.articulation->applyCache(*record.cache,
        physx::PxArticulationCacheFlag::eFORCE, true);
}

// Resolve the coherent animation guide after contacts have been solved and
// before any snapshot is exposed. Normal gameplay uses full authority and is
// therefore exact. Reactive modes explicitly lower authority; no independent
// vertical support or upright solver can fight the root target.
void resolveRagdollAnimationConstraint(PhysicsScene3D::Impl::RagdollRecord& record,
    float dt, bool grabbed) {
    float impulse=record.externalForceMagnitude*dt;
    record.externalForceMagnitude=0;
    for(const auto& c:record.stepContacts) {
        if(c.linkIndex>=record.profile.links.size())continue;
        const auto& id=record.profile.links[c.linkIndex].id;
        if((id=="LeftFoot"||id=="RightFoot")&&c.normal.z>0.62f&&!c.otherBodyDynamic)continue;
        impulse+=std::max(0.0f,c.normalImpulseNewtonSeconds);
    }
    const float weight=std::max(1.0f,record.profile.totalMassKg*9.81f);
    const float pressure=impulse/(weight*dt);
    record.state.externalInterference=pressure>0.04f?std::clamp(pressure/0.5f,0.0f,1.0f):0;
    record.state.animationAuthority=0;
    if(record.frozen||!record.active||!record.animationConstraintStaged
        ||!record.cache||grabbed) {
        record.animationConstraintStaged=false;return;
    }
    record.animationConstraintStaged=false;
    auto target=record.animationConstraint;
    if(target.releaseOnInteraction && record.state.externalInterference>0) {
        const float cap=1.0f-0.90f*record.state.externalInterference;
        target.poseAuthority=std::min(target.poseAuthority,cap);
        target.rootTranslationAuthority=std::min(
            target.rootTranslationAuthority,cap);
        target.rootRotationAuthority=std::min(
            target.rootRotationAuthority,cap);
    }
    const auto blendFor=[dt](float authority) {
        authority=std::clamp(authority,0.0f,1.0f);
        return authority>=1.0f?1.0f:
            1.0f-std::exp(-dt*12.0f*authority/std::max(0.001f,1-authority));
    };
    const float poseBlend=blendFor(target.poseAuthority);
    const float translationBlend=blendFor(target.rootTranslationAuthority);
    const float rotationBlend=blendFor(target.rootRotationAuthority);
    if(poseBlend<=0&&translationBlend<=0&&rotationBlend<=0)return;
    // Only write back what authority actually asked for. Applying joint
    // position/velocity when poseBlend is zero would hand the solver its own
    // values back every tick - numerically a no-op, but it still resets the
    // articulation's internal state, and that showed up as a permanent
    // shiver on a character that was supposed to be standing still.
    auto flags=physx::PxArticulationCacheFlag::eROOT_TRANSFORM|
        physx::PxArticulationCacheFlag::eROOT_VELOCITIES;
    if(poseBlend>0) {
        flags=flags|physx::PxArticulationCacheFlag::ePOSITION
            |physx::PxArticulationCacheFlag::eVELOCITY;
    }
    record.articulation->copyInternalStateToCache(*record.cache,flags);
    for(const auto& drive:record.stagedActiveTargets) {
        if(drive.linkIndex==0||drive.linkIndex>=record.joints.size())continue;
        const auto axis=static_cast<std::size_t>(drive.axis);
        if(axis>=3||!std::isfinite(drive.positionRadians)||!std::isfinite(drive.velocityRadiansPerSecond))continue;
        const auto dof=record.joints[drive.linkIndex].dofIndices[axis];
        if(dof==InvalidRagdollDof||dof>=record.articulation->getDofs())continue;
        const auto& limit=record.profile.links[drive.linkIndex].inboundJoint.axes[axis];
        const float q=std::clamp(drive.positionRadians,limit.minimumRadians,limit.maximumRadians);
        if(target.poseAuthority>=1.0f) {
            record.cache->jointPosition[dof]=q;
            record.cache->jointVelocity[dof]=drive.velocityRadiansPerSecond;
        } else {
            record.cache->jointPosition[dof]+=poseBlend*(q-record.cache->jointPosition[dof]);
            record.cache->jointVelocity[dof]+=poseBlend*(drive.velocityRadiansPerSecond-record.cache->jointVelocity[dof]);
        }
    }
    auto& root=*record.cache->rootLinkData;
    const physx::PxVec3 desiredPosition=toPhysX(target.rootPositionWorld);
    physx::PxQuat desiredOrientation=toPhysX(target.rootOrientationWorld);
    if(root.transform.q.dot(desiredOrientation)<0)desiredOrientation=-desiredOrientation;
    const physx::PxVec3 desiredLinear=toPhysX(target.rootLinearVelocityWorld);
    const physx::PxVec3 desiredAngular=toPhysX(target.rootAngularVelocityWorld);
    if(target.rootTranslationAuthority>=1.0f) {
        root.transform.p=desiredPosition;
        root.worldLinVel=desiredLinear;
    } else if(translationBlend>0) {
        root.transform.p+=translationBlend*(desiredPosition-root.transform.p);
        root.worldLinVel+=translationBlend*(desiredLinear-root.worldLinVel);
    }
    if(target.rootRotationAuthority>=1.0f) {
        root.transform.q=desiredOrientation;
        root.worldAngVel=desiredAngular;
    } else if(rotationBlend>0) {
        root.transform.q=(root.transform.q*(1-rotationBlend)+
            desiredOrientation*rotationBlend).getNormalized();
        root.worldAngVel+=rotationBlend*(desiredAngular-root.worldAngVel);
    }
    record.articulation->applyCache(*record.cache,flags,true);
    record.state.animationAuthority=std::min({target.poseAuthority,
        target.rootTranslationAuthority,target.rootRotationAuthority});
}

void updateRagdollState(
    PhysicsScene3D::Impl::RagdollRecord& record) {
    if (record.articulation == nullptr
        || record.articulation->getScene() == nullptr) {
        return;
    }
    record.state.sleeping = record.articulation->isSleeping();
    record.state.rigidityPercent = record.rigidityPercent;
    record.state.active = record.active;
    record.state.frozen = record.frozen;
    record.state.contacts = record.stepContacts;
    publishRagdollInteractions3D(record.stepInteractions, record.state.interactions);
    if (record.state.links.size() != record.links.size()) {
        record.state.links.resize(record.links.size());
    }
    if (record.state.joints.size() != record.links.size()) {
        record.state.joints.resize(record.links.size());
    }
    // Links adormecidos não mudaram desde o fetch anterior. Preservar os
    // snapshots evita vinte leituras nativas por ragdoll parado.
    if (record.state.sleeping && record.stateInitialized) return;
    if(record.cache)
        record.articulation->copyInternalStateToCache(*record.cache,
            physx::PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);
    for (std::size_t index = 0; index < record.links.size(); ++index) {
        physx::PxArticulationLink* link = record.links[index];
        if (link == nullptr) continue;
        PhysicsBodyState3D& state = record.state.links[index];
        const physx::PxTransform pose = link->getGlobalPose();
        state.position = fromPhysX(pose.p);
        state.orientation = fromPhysX(pose.q);
        state.linearVelocity = fromPhysX(link->getLinearVelocity());
        state.angularVelocity = fromPhysX(link->getAngularVelocity());
        state.sleeping = record.state.sleeping;
        if (index == 0 || record.joints[index].joint == nullptr) continue;
        RagdollJointState3D& jointState = record.state.joints[index];
        if(record.cache) {
            const auto torque=record.cache->linkIncomingJointForce[link->getLinkIndex()].torque;
            jointState.transmittedTorqueNewtonMeters={torque.x,torque.y,torque.z};
        }
        const auto& definition =
            record.profile.links[index].inboundJoint;
        for (std::size_t axisIndexValue = 0;
                axisIndexValue < definition.axes.size();
                ++axisIndexValue) {
            if (!definition.axes[axisIndexValue].enabled) continue;
            const auto axis = toPhysX(
                static_cast<RagdollAxis3D>(axisIndexValue));
            jointState.positionRadians[axisIndexValue] =
                record.joints[index].joint->getJointPosition(axis);
            jointState.velocityRadiansPerSecond[axisIndexValue] =
                record.joints[index].joint->getJointVelocity(axis);
        }
    }
    record.stateInitialized = true;
}

} // namespace

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
            physx::PxU32 filterFlags = FilterFlagContactReports;
            if (definition.collisionMode
                == PhysicsCollisionMode3D::Continuous) {
                filterFlags |= FilterFlagContinuousCollision;
            }
            // Valores que o contrato neutro sempre entregou por default,
            // agora explicitos: corpo comum em uma unica layer, colidindo com
            // tudo. Ver PhysicsBodyDefinition3D sobre a remocao do par
            // layer/mask configuravel.
            constexpr physx::PxU32 DefaultBodyLayer = 1u;
            constexpr physx::PxU32 DefaultBodyMask = 0xFFFFFFFFu;
            const physx::PxFilterData filter(DefaultBodyLayer,
                DefaultBodyMask, filterFlags, 0);
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
            // Se uma colisão extrema ou erro numérico produzir penetração,
            // deixa o solver separar o par sem converter a correção inteira
            // em um impulso explosivo num único passo. O teto de impulso é
            // escalado pela massa, portanto limita delta-v em vez de punir
            // objetos pesados de forma diferente.
            dynamic->setMaxDepenetrationVelocity(4.0f);
            dynamic->setMaxContactImpulse(
                std::max(definition.massKg, MinimumMassKg) * 25.0f);
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

bool PhysicsScene3D::overlapsBox(Vec3 center, Vec3 halfExtents,
    Quaternion orientation) const {
    if (!m_impl || m_impl->scene == nullptr
        || !std::isfinite(center.x) || !std::isfinite(center.y)
        || !std::isfinite(center.z)
        || !std::isfinite(halfExtents.x)
        || !std::isfinite(halfExtents.y)
        || !std::isfinite(halfExtents.z)) {
        return true;
    }
    halfExtents.x = std::max(halfExtents.x, MinimumShapeSizeMeters);
    halfExtents.y = std::max(halfExtents.y, MinimumShapeSizeMeters);
    halfExtents.z = std::max(halfExtents.z, MinimumShapeSizeMeters);
    physx::PxOverlapBuffer hit;
    const physx::PxQueryFilterData filter(
        physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::eDYNAMIC
            | physx::PxQueryFlag::eANY_HIT);
    return m_impl->scene->overlap(
        physx::PxBoxGeometry(toPhysX(halfExtents)),
        toPhysX(center, orientation.normalized()), hit, filter);
}

RagdollHandle3D PhysicsScene3D::createRagdoll(
    const RagdollProfile3D& profile,
    const RagdollSpawnDefinition3D& definition) {
    validateRagdollProfileOrThrow3D(profile);
    if (profile.links.empty()) {
        throw std::invalid_argument("Ragdoll precisa de ao menos um link");
    }
    if (profile.links.size() > 32) {
        throw std::invalid_argument(
            "Self-collision seletiva suporta no máximo 32 links");
    }
    if (!std::isfinite(definition.rigidityPercent)) {
        throw std::invalid_argument("Rigidez inicial de ragdoll inválida");
    }

    std::uint32_t slotIndex;
    if (!m_impl->freeRagdollSlots.empty()) {
        slotIndex = m_impl->freeRagdollSlots.back();
        m_impl->freeRagdollSlots.pop_back();
    } else {
        slotIndex = static_cast<std::uint32_t>(
            m_impl->ragdollSlots.size());
        m_impl->ragdollSlots.emplace_back();
    }
    Impl::RagdollSlot& slot = m_impl->ragdollSlots[slotIndex];
    const RagdollHandle3D handle { slotIndex, slot.generation };
    auto record = std::make_unique<Impl::RagdollRecord>();
    record->handle = handle;
    record->entityId = definition.entityId;
    record->profile = profile;
    record->links.resize(profile.links.size());
    record->joints.resize(profile.links.size());
    record->state.links.resize(profile.links.size());
    record->state.active = definition.active;
    record->active = definition.active;
    record->passiveRigidityPercent = std::clamp(
        definition.rigidityPercent, 0.0f, 100.0f);
    record->stepContacts.reserve(24);
    record->stepInteractions.resize(profile.links.size() * 6);
    record->state.interactions.reserve(profile.links.size() * 6);
    record->stagedActiveTargets.reserve(
        ragdollDegreesOfFreedom3D(profile));
    record->articulation =
        m_impl->engine->physics->createArticulationReducedCoordinate();
    if (record->articulation == nullptr) {
        m_impl->freeRagdollSlots.push_back(slotIndex);
        throw std::runtime_error("PhysX não conseguiu criar a articulation");
    }

    try {
        record->articulation->setName(record->profile.id.c_str());
        record->articulation->setSolverIterationCounts(
            definition.active
                ? std::max(8u, m_impl->settings.solverPositionIterations)
                : m_impl->settings.solverPositionIterations,
            definition.active
                ? std::max(2u, m_impl->settings.solverVelocityIterations)
                : m_impl->settings.solverVelocityIterations);
        record->articulation->setArticulationFlag(
            physx::PxArticulationFlag::eFIX_BASE, false);
        record->articulation->setArticulationFlag(
            physx::PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES, true);
        // Parent/filho já é filtrado internamente pela PhysX. Os demais pares
        // passam pela matriz topológica gravada no PxFilterData: membros
        // distantes colidem, vizinhos que se sobrepõem por projeto não.
        record->articulation->setArticulationFlag(
            physx::PxArticulationFlag::eDISABLE_SELF_COLLISION, false);

        const Quaternion spawnOrientation =
            definition.orientation.normalized();
        for (std::size_t index = 0; index < profile.links.size(); ++index) {
            const RagdollLinkDefinition3D& linkDefinition =
                record->profile.links[index];
            physx::PxArticulationLink* parent = nullptr;
            if (linkDefinition.parentIndex >= 0) {
                parent = record->links[static_cast<std::size_t>(
                    linkDefinition.parentIndex)];
            }
            const Vec3 worldPosition = definition.pelvisPosition
                + spawnOrientation.rotate(linkDefinition.modelPosition);
            const Quaternion worldOrientation = (spawnOrientation
                * linkDefinition.modelOrientation).normalized();
            physx::PxArticulationLink* link =
                record->articulation->createLink(parent,
                    toPhysX(worldPosition, worldOrientation));
            if (link == nullptr) {
                throw std::runtime_error(
                    "PhysX não conseguiu criar link " + linkDefinition.id);
            }
            record->links[index] = link;
            link->setName(linkDefinition.id.c_str());

            physx::PxMaterial& material =
                m_impl->material(linkDefinition.collider.materialId);
            physx::PxShape* shape = nullptr;
            if (linkDefinition.collider.shape
                == RagdollColliderShape3D::Box) {
                shape = physx::PxRigidActorExt::createExclusiveShape(*link,
                    physx::PxBoxGeometry(
                        toPhysX(linkDefinition.collider.boxHalfExtents)),
                    material);
            } else {
                // PhysX nao tem capsula conica: a do maior raio, que cobre
                // a conica (backend arquivado; o Jolt e o padrao).
                const float radius = std::max(
                    linkDefinition.collider.radiusMeters,
                    ragdollCapsuleRadiusAtPositiveX3D(
                        linkDefinition.collider));
                const float capsuleHalfHeight = std::max(0.0f,
                    linkDefinition.collider.lengthMeters * 0.5f - radius);
                shape = physx::PxRigidActorExt::createExclusiveShape(*link,
                    physx::PxCapsuleGeometry(radius, capsuleHalfHeight),
                    material);
            }
            if (shape == nullptr) {
                throw std::runtime_error(
                    "PhysX não conseguiu criar collider de "
                    + linkDefinition.id);
            }
            shape->setLocalPose(toPhysX(
                linkDefinition.collider.localPosition,
                linkDefinition.collider.localOrientation));
            physx::PxU32 ragdollInstance =
                (slotIndex + 1u) << RagdollLinkTokenBits;
            if (linkDefinition.collider.contactSensor) {
                ragdollInstance |= RagdollContactSensorFlag;
            }
            const physx::PxU32 linkToken =
                static_cast<physx::PxU32>(index + 1u);
            const physx::PxFilterData collisionFilter(
                RagdollCollisionLayer, 0xFFFFFFFFu,
                ragdollSelfCollisionMask(record->profile, index),
                ragdollInstance | linkToken);
            shape->setSimulationFilterData(collisionFilter);
            shape->setQueryFilterData(collisionFilter);
            shape->setContactOffset(0.012f);
            shape->setRestOffset(0.0f);

            const float mass = std::max(MinimumMassKg,
                profile.totalMassKg * linkDefinition.massFraction);
            const physx::PxVec3 centerOfMass =
                toPhysX(linkDefinition.centerOfMassLocal);
            if (!physx::PxRigidBodyExt::setMassAndUpdateInertia(
                    *link, mass, &centerOfMass)) {
                throw std::runtime_error(
                    "PhysX não calculou a inércia de "
                    + linkDefinition.id);
            }
            link->setLinearDamping(0.04f);
            link->setAngularDamping(0.16f);
            // Penetracoes profundas podem surgir quando a Physgun empurra
            // dois membros um contra o outro. Sem clamp, o bias do solver
            // injeta uma velocidade arbitrariamente alta para separa-los,
            // percebida como o ragdoll "explodindo".
            link->setMaxDepenetrationVelocity(3.0f);
            // Limite por contato, proporcional a massa: ainda permite quedas
            // e tackles fortes, mas um unico ponto nao consegue transferir
            // dezenas de m/s em um passo de 1/120 s.
            link->setMaxContactImpulse(mass * 12.0f);
            link->setMaxAngularVelocity(24.0f);
            // Contato discreto pode perder uma capsula fina quando um membro
            // gira muito entre dois ticks. Speculative CCD expande a geração
            // de contato pela velocidade, incluindo movimento angular, sem
            // pagar os sweeps do CCD completo em todos os 18 links.
            link->setRigidBodyFlag(
                physx::PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);

            if (parent == nullptr) continue;
            auto* joint = link->getInboundJoint();
            if (joint == nullptr) {
                throw std::runtime_error("Link sem inbound joint: "
                    + linkDefinition.id);
            }
            record->joints[index].joint = joint;
            const RagdollLinkDefinition3D& parentDefinition =
                record->profile.links[static_cast<std::size_t>(
                    linkDefinition.parentIndex)];
            const Quaternion parentFrameOrientation =
                (parentDefinition.modelOrientation.conjugate()
                    * linkDefinition.inboundJoint.frameModelOrientation)
                    .normalized();
            const Quaternion childFrameOrientation =
                (linkDefinition.modelOrientation.conjugate()
                    * linkDefinition.inboundJoint.frameModelOrientation)
                    .normalized();
            const Vec3 parentFramePosition =
                parentDefinition.modelOrientation.conjugate().rotate(
                    linkDefinition.inboundJoint.anchorModelPosition
                        - parentDefinition.modelPosition);
            const Vec3 childFramePosition =
                linkDefinition.modelOrientation.conjugate().rotate(
                    linkDefinition.inboundJoint.anchorModelPosition
                        - linkDefinition.modelPosition);
            joint->setParentPose(toPhysX(parentFramePosition,
                parentFrameOrientation));
            joint->setChildPose(toPhysX(childFramePosition,
                childFrameOrientation));
            joint->setJointType(
                linkDefinition.inboundJoint.type
                    == RagdollJointType3D::Revolute
                ? physx::PxArticulationJointType::eREVOLUTE
                : physx::PxArticulationJointType::eSPHERICAL);

            for (std::size_t axisIndexValue = 0;
                    axisIndexValue
                        < linkDefinition.inboundJoint.axes.size();
                    ++axisIndexValue) {
                const RagdollAxisDefinition3D& axisDefinition =
                    linkDefinition.inboundJoint.axes[axisIndexValue];
                const auto axis = toPhysX(
                    static_cast<RagdollAxis3D>(axisIndexValue));
                if (!axisDefinition.enabled) {
                    joint->setMotion(axis,
                        physx::PxArticulationMotion::eLOCKED);
                    continue;
                }
                joint->setMotion(axis,
                    physx::PxArticulationMotion::eLIMITED);
                joint->setLimitParams(axis, physx::PxArticulationLimit(
                    axisDefinition.minimumRadians,
                    axisDefinition.maximumRadians));
                // Uma armadura pequena regulariza a inércia aparente dos
                // graus de liberdade leves (punhos/tornozelos) e evita que
                // ganhos rápidos transformem ruído de contato em oscilações.
                // O valor é deliberadamente baixo: não altera a massa dos
                // links nem substitui a dinâmica da articulation.
                joint->setArmature(axis, 0.012f);
                joint->setMaxJointVelocity(axis, 14.0f);
                record->joints[index].targets[axisIndexValue] = 0.0f;
            }
        }

        // Um ragdoll é uma coleção espacialmente coerente. A recomendação
        // oficial da PhysX é publicá-lo como PxAggregate para ocupar uma
        // única entrada no broad phase; colisões com mundo/outros ragdolls
        // continuam normais, enquanto pares internos já nascem filtrados.
        record->aggregate = m_impl->engine->physics->createAggregate(
            static_cast<physx::PxU32>(record->links.size()),
            static_cast<physx::PxU32>(record->links.size()),
            physx::PxGetAggregateFilterHint(
                physx::PxAggregateType::eGENERIC, true));
        if (record->aggregate == nullptr
            || !record->aggregate->addArticulation(
                *record->articulation)) {
            throw std::runtime_error(
                "PhysX não conseguiu agregar os links do ragdoll");
        }
        m_impl->scene->addAggregate(*record->aggregate);
        if (record->articulation->getDofs()
            != ragdollDegreesOfFreedom3D(record->profile)) {
            throw std::runtime_error(
                "PhysX publicou uma contagem inesperada de DOFs");
        }
        record->cache = record->articulation->createCache();
        if (record->cache == nullptr) {
            throw std::runtime_error(
                "PhysX não criou o cache dinâmico do ragdoll");
        }
        // Cache indexing segue a ordem interna dos links e dos eixos PhysX.
        // Guardar o mapeamento uma vez evita buscas nos 41 DOFs a cada tick.
        std::vector<physx::PxArticulationLink*> orderedLinks = record->links;
        std::sort(orderedLinks.begin(), orderedLinks.end(),
            [](const physx::PxArticulationLink* first,
                    const physx::PxArticulationLink* second) {
                return first->getLinkIndex() < second->getLinkIndex();
            });
        std::uint32_t dofOffset = 0;
        for (physx::PxArticulationLink* link : orderedLinks) {
            if (link == nullptr || link->getLinkIndex() == 0) continue;
            const auto found = std::find(record->links.begin(),
                record->links.end(), link);
            if (found == record->links.end()) continue;
            const std::size_t profileIndex = static_cast<std::size_t>(
                std::distance(record->links.begin(), found));
            for (std::size_t axisIndexValue = 0;
                    axisIndexValue < 3; ++axisIndexValue) {
                if (!record->profile.links[profileIndex].inboundJoint
                        .axes[axisIndexValue].enabled) {
                    continue;
                }
                record->joints[profileIndex].dofIndices[axisIndexValue] =
                    dofOffset++;
            }
        }
        if (dofOffset != record->articulation->getDofs()) {
            throw std::runtime_error(
                "Mapeamento dos DOFs do ragdoll ficou inconsistente");
        }
        // A pose neutra recém-criada é o alvo inicial. Mudanças posteriores
        // de zero para rigidez positiva capturam a pose física atual.
        applyRagdollRigidity(*record, definition.active ? 0.0f
            : std::clamp(definition.rigidityPercent, 0.0f, 100.0f), false);
        updateRagdollState(*record);
    } catch (...) {
        if (record->cache != nullptr) {
            record->cache->release();
            record->cache = nullptr;
        }
        record->articulation->release();
        record->articulation = nullptr;
        if (record->aggregate != nullptr) {
            record->aggregate->release();
            record->aggregate = nullptr;
        }
        m_impl->freeRagdollSlots.push_back(slotIndex);
        throw;
    }

    slot.record = std::move(record);
    return handle;
}

void PhysicsScene3D::destroyRagdoll(RagdollHandle3D handle) {
    Impl::RagdollRecord* record = m_impl->ragdoll(handle);
    if (record == nullptr) return;
    if (m_impl->grabRagdoll == handle) m_impl->releaseGrab();
    std::erase_if(m_impl->pendingRagdollCommands,
        [handle](const Impl::RagdollCommand& command) {
            return command.handle == handle;
        });
    if (record->articulation != nullptr) {
        if (record->cache != nullptr) {
            record->cache->release();
            record->cache = nullptr;
        }
        record->articulation->release();
        record->articulation = nullptr;
    }
    if (record->aggregate != nullptr) {
        record->aggregate->release();
        record->aggregate = nullptr;
    }
    Impl::RagdollSlot& slot = m_impl->ragdollSlots[handle.index];
    slot.record.reset();
    ++slot.generation;
    if (slot.generation == 0) ++slot.generation;
    m_impl->freeRagdollSlots.push_back(handle.index);
}

bool PhysicsScene3D::contains(RagdollHandle3D handle) const {
    return m_impl->ragdoll(handle) != nullptr;
}

RagdollState3D PhysicsScene3D::ragdollState(
    RagdollHandle3D handle) const {
    const Impl::RagdollRecord* record = m_impl->ragdoll(handle);
    return record != nullptr ? record->state : RagdollState3D {};
}

RagdollDynamics3D PhysicsScene3D::ragdollDynamics(
    RagdollHandle3D handle) const {
    RagdollDynamics3D result;
    const Impl::RagdollRecord* record = m_impl->ragdoll(handle);
    if (record == nullptr || record->articulation == nullptr
        || record->articulation->getScene() == nullptr) {
        return result;
    }

    // Esta função já computava matriz de massa, Jacobiano denso, matriz de
    // momento centroidal e compensações de gravidade/Coriolis a cada chamada —
    // por ragdoll, por tick — porque tudo isso vem junto do
    // PxArticulationCache. Nada disso tinha consumidor: o contrato neutro foi
    // estreitado para os quatro campos realmente lidos, e o custo saiu com
    // eles. Ver RagdollDynamics3D em PhysicsScene3D.hpp.
    result.generalizedDofCount = record->articulation->getDofs() + 6u;
    result.centerOfMass =
        fromPhysX(record->articulation->computeArticulationCOM(false));

    result.jointGeneralizedDof.resize(record->joints.size());
    for (auto& indices : result.jointGeneralizedDof) {
        indices.fill(RagdollDynamics3D::InvalidIndex);
    }
    for (std::size_t linkIndex = 1;
            linkIndex < record->joints.size(); ++linkIndex) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const std::uint32_t dof =
                record->joints[linkIndex].dofIndices[axis];
            if (dof != InvalidRagdollDof
                && dof + 6u < result.generalizedDofCount) {
                result.jointGeneralizedDof[linkIndex][axis] = dof + 6u;
            }
        }
    }
    result.valid = true;
    return result;
}

void PhysicsScene3D::setRagdollRigidity(
    RagdollHandle3D handle, float rigidityPercent) {
    Impl::RagdollRecord* record = m_impl->ragdoll(handle);
    if (record == nullptr
        || !std::isfinite(rigidityPercent)) {
        return;
    }
    record->passiveRigidityPercent =
        std::clamp(rigidityPercent, 0.0f, 100.0f);
    m_impl->pendingRagdollCommands.push_back({ handle,
        Impl::RagdollCommandType::SetRigidity,
        record->passiveRigidityPercent });
}

void PhysicsScene3D::captureRagdollPose(RagdollHandle3D handle) {
    if (m_impl->ragdoll(handle) == nullptr) return;
    m_impl->pendingRagdollCommands.push_back({ handle,
        Impl::RagdollCommandType::CapturePose, 0.0f });
}

void PhysicsScene3D::setRagdollNeutralPose(RagdollHandle3D handle) {
    if (m_impl->ragdoll(handle) == nullptr) return;
    m_impl->pendingRagdollCommands.push_back({ handle,
        Impl::RagdollCommandType::NeutralPose, 0.0f });
}

void PhysicsScene3D::releaseRagdollDrives(RagdollHandle3D handle) {
    if (m_impl->ragdoll(handle) == nullptr) return;
    m_impl->pendingRagdollCommands.push_back({ handle,
        Impl::RagdollCommandType::Release, 0.0f });
}

void PhysicsScene3D::setRagdollActive(
    RagdollHandle3D handle, bool active) {
    if (m_impl->ragdoll(handle) == nullptr) return;
    m_impl->pendingRagdollCommands.push_back({ handle,
        Impl::RagdollCommandType::SetActive, active ? 1.0f : 0.0f });
}

void PhysicsScene3D::setRagdollFrozen(
    RagdollHandle3D handle, bool frozen) {
    if (m_impl->ragdoll(handle) == nullptr) return;
    m_impl->pendingRagdollCommands.push_back({ handle,
        Impl::RagdollCommandType::SetFrozen, frozen ? 1.0f : 0.0f });
}

void PhysicsScene3D::setRagdollActiveDriveTargets(
    RagdollHandle3D handle,
    std::span<const RagdollDriveTarget3D> targets,
    bool gravityCompensationEnabled) {
    Impl::RagdollRecord* record = m_impl->ragdoll(handle);
    if (record == nullptr) return;
    record->stagedActiveTargets.assign(targets.begin(), targets.end());
    record->stagedActiveTargetsDirty = true;
    record->stagedGravityCompensationEnabled =
        gravityCompensationEnabled;
}

void PhysicsScene3D::setRagdollAnimationConstraint(RagdollHandle3D handle,
    const RagdollAnimationConstraint3D& constraint) {
    auto* record=m_impl->ragdoll(handle);
    if(!record)return;
    record->animationConstraintStaged=false;
    const auto& q=constraint.rootOrientationWorld;
    const auto& position=constraint.rootPositionWorld;
    const auto& velocity=constraint.rootLinearVelocityWorld;
    const auto& angular=constraint.rootAngularVelocityWorld;
    if(!std::isfinite(constraint.poseAuthority)||
       !std::isfinite(constraint.rootTranslationAuthority)||
       !std::isfinite(constraint.rootRotationAuthority)||
       !std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)||!std::isfinite(q.w)||
       q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<0.001f||
       !std::isfinite(position.x)||!std::isfinite(position.y)||!std::isfinite(position.z)||
       !std::isfinite(velocity.x)||!std::isfinite(velocity.y)||!std::isfinite(velocity.z)||
       !std::isfinite(angular.x)||!std::isfinite(angular.y)||
       !std::isfinite(angular.z))return;
    record->animationConstraint=constraint;
    record->animationConstraint.poseAuthority=std::clamp(
        constraint.poseAuthority,0.0f,1.0f);
    record->animationConstraint.rootTranslationAuthority=std::clamp(
        constraint.rootTranslationAuthority,0.0f,1.0f);
    record->animationConstraint.rootRotationAuthority=std::clamp(
        constraint.rootRotationAuthority,0.0f,1.0f);
    record->animationConstraint.rootOrientationWorld=q.normalized();
    record->animationConstraintStaged=true;
}

void PhysicsScene3D::applyRagdollRootForce(RagdollHandle3D handle,
    Vec3 forceNewtons, Vec3 torqueNewtonMeters) {
    applyRagdollLinkForce(handle, 0, forceNewtons, torqueNewtonMeters);
}

void PhysicsScene3D::applyRagdollControlRootForce(RagdollHandle3D handle,
    Vec3 forceNewtons, Vec3 torqueNewtonMeters) {
    Impl::RagdollRecord* record = m_impl->ragdoll(handle);
    const auto finiteVector = [](Vec3 value) {
        return std::isfinite(value.x) && std::isfinite(value.y)
            && std::isfinite(value.z);
    };
    if (record == nullptr || record->frozen || record->links.empty()
        || record->links.front() == nullptr
        || !finiteVector(forceNewtons) || !finiteVector(torqueNewtonMeters)) {
        return;
    }
    if (record->articulation != nullptr) record->articulation->wakeUp();
    record->links.front()->addForce(toPhysX(forceNewtons),
        physx::PxForceMode::eFORCE, true);
    record->links.front()->addTorque(toPhysX(torqueNewtonMeters),
        physx::PxForceMode::eFORCE, true);
}

void PhysicsScene3D::applyRagdollControlLinkForce(RagdollHandle3D handle,
    std::uint32_t linkIndex, Vec3 forceNewtons,
    Vec3 torqueNewtonMeters) {
    // Como applyRagdollLinkForce, sem contar como interferencia externa.
    Impl::RagdollRecord* record = m_impl->ragdoll(handle);
    if (record == nullptr || record->frozen
        || linkIndex >= record->links.size()
        || record->links[linkIndex] == nullptr) {
        return;
    }
    if (record->articulation != nullptr) record->articulation->wakeUp();
    record->links[linkIndex]->addForce(toPhysX(forceNewtons),
        physx::PxForceMode::eFORCE, true);
    record->links[linkIndex]->addTorque(toPhysX(torqueNewtonMeters),
        physx::PxForceMode::eFORCE, true);
}

void PhysicsScene3D::applyRagdollLinkForce(RagdollHandle3D handle,
    std::uint32_t linkIndex, Vec3 forceNewtons,
    Vec3 torqueNewtonMeters) {
    Impl::RagdollRecord* record = m_impl->ragdoll(handle);
    if (record == nullptr || record->frozen
        || linkIndex >= record->links.size()
        || record->links[linkIndex] == nullptr) {
        return;
    }
    // PxArticulationLink herda PxRigidBody; addForce/addTorque funcionam
    // exatamente como num PxRigidDynamic solto, aplicados imediatamente
    // (não passam pela fila de comandos estruturais — força não é estado
    // persistente, é reaplicada a cada tick pelo chamador). wakeUp() é
    // necessário aqui: ao contrário de setDriveParams/setDriveTarget (que
    // já acordam a articulation internamente), addForce sozinho em um
    // corpo dormindo não tira a articulation do sono, e o snapshot público
    // (RagdollState3D) para de ser atualizado para corpos dormindo.
    if (record->articulation != nullptr) {
        record->articulation->wakeUp();
    }
    record->externalForceMagnitude+=forceNewtons.length()+torqueNewtonMeters.length()/
        std::max(0.2f,record->profile.standingRootHeightMeters);
    record->links[linkIndex]->addForce(toPhysX(forceNewtons),
        physx::PxForceMode::eFORCE, true);
    record->links[linkIndex]->addTorque(toPhysX(torqueNewtonMeters),
        physx::PxForceMode::eFORCE, true);
}

void PhysicsScene3D::destroyBody(PhysicsBodyHandle3D bodyHandle) {
    Impl::BodyRecord* record = m_impl->body(bodyHandle);
    if (record == nullptr) return;
    if (m_impl->grabBody == bodyHandle) m_impl->releaseGrab();
    std::erase(m_impl->activeAerodynamicBodies, record);
    // As duas listas carregam ponteiros crus para o registro entre passos.
    // Deixar o corpo na lista de empuxo depois de liberar o registro causava
    // use-after-free no simulate seguinte (crash frequente ao apagar/recriar
    // props desde que a agua foi adicionada).
    std::erase(m_impl->activeOceanBodies, record);
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
    for (std::uint32_t index = 0;
            index < m_impl->ragdollSlots.size(); ++index) {
        Impl::RagdollSlot& slot = m_impl->ragdollSlots[index];
        if (!slot.record) continue;
        destroyRagdoll({ index, slot.generation });
    }
    m_impl->contactImpacts.clear();
    m_impl->contactSlides.clear();
    for (Impl::RagdollSlot& slot : m_impl->ragdollSlots) {
        if (slot.record) {
            slot.record->stepContacts.clear();
            std::fill(slot.record->stepInteractions.begin(), slot.record->stepInteractions.end(), RagdollInteraction3D {});
        }
    }
    m_impl->activeAerodynamicBodies.clear();
    m_impl->activeOceanBodies.clear();
    m_impl->activeBodyStateUpdates.clear();
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

physx::PxTransform traversalCapsulePose(Vec3 center) {
    // PxCapsuleGeometry is longitudinal in X; traversal capsules are vertical.
    return { toPhysX(center), physx::PxQuat(physx::PxHalfPi,
        physx::PxVec3(0.0f, 1.0f, 0.0f)) };
}

bool sweepTraversalCapsule(const PhysicsScene3D::Impl& implementation,
    Vec3 center, float radius, float halfHeight, Vec3 displacement,
    physx::PxSweepHit& hit) {
    const float distance = displacement.length();
    if (distance <= 1.0e-6f) return false;
    const physx::PxCapsuleGeometry geometry(radius, halfHeight);
    physx::PxSweepBuffer buffer;
    physx::PxQueryFilterData filter;
    filter.flags = physx::PxQueryFlag::eSTATIC;
    const bool found = implementation.scene->sweep(geometry,
        traversalCapsulePose(center), toPhysX(displacement / distance),
        distance, buffer,
        physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL,
        filter);
    if (!found || !buffer.hasBlock) return false;
    hit = buffer.block;
    return true;
}

GroundProbeResult3D probeTraversalGround(
    const PhysicsScene3D::Impl& implementation, Vec3 center, float radius,
    float halfHeight, float distance, float maximumSlopeDegrees) {
    GroundProbeResult3D result;
    if (radius <= 0.0f || halfHeight < 0.0f || distance <= 0.0f) {
        return result;
    }
    physx::PxSweepHit hit;
    if (!sweepTraversalCapsule(implementation, center, radius, halfHeight,
            { 0.0f, 0.0f, -distance }, hit)) {
        return result;
    }
    result.hasSurface = true;
    result.pointWorld = fromPhysX(hit.position);
    result.normalWorld = fromPhysX(hit.normal).normalized();
    result.distanceMeters = hit.distance;
    result.slopeDegrees = std::acos(std::clamp(result.normalWorld.z,
        -1.0f, 1.0f)) * 180.0f / physx::PxPi;
    result.walkable = result.normalWorld.z >= std::cos(
        std::clamp(maximumSlopeDegrees, 0.0f, 89.0f)
            * physx::PxPi / 180.0f);
    return result;
}

} // namespace

GroundProbeResult3D PhysicsScene3D::probeGround(Vec3 capsuleCenterWorld,
    float radiusMeters, float cylinderHalfHeightMeters,
    float probeDistanceMeters, float maximumSlopeDegrees) const {
    return probeTraversalGround(*m_impl, capsuleCenterWorld, radiusMeters,
        cylinderHalfHeightMeters, probeDistanceMeters, maximumSlopeDegrees);
}

// (PhysX nao e o backend de jogo: o terreno ve so o estatico, como o chao.)
GroundProbeResult3D PhysicsScene3D::probeTerrain(Vec3 capsuleCenterWorld,
    float radiusMeters, float cylinderHalfHeightMeters,
    float probeDistanceMeters, float maximumSlopeDegrees) const {
    return probeGround(capsuleCenterWorld, radiusMeters, cylinderHalfHeightMeters,
        probeDistanceMeters, maximumSlopeDegrees);
}

CapsuleTraversalResult3D PhysicsScene3D::solveCapsuleTraversal(
    const CapsuleTraversalQuery3D& source) const {
    CapsuleTraversalResult3D result;
    result.requestedDisplacementWorld = source.desiredDisplacementWorld;
    if (!std::isfinite(source.centerWorld.x)
        || !std::isfinite(source.centerWorld.y)
        || !std::isfinite(source.centerWorld.z)
        || !std::isfinite(source.desiredDisplacementWorld.x)
        || !std::isfinite(source.desiredDisplacementWorld.y)
        || !std::isfinite(source.desiredDisplacementWorld.z)) {
        return result;
    }

    const float radius = std::clamp(source.radiusMeters, 0.05f, 2.0f);
    const float halfHeight = std::clamp(
        source.cylinderHalfHeightMeters, 0.0f, 3.0f);
    const float skin = std::clamp(source.skinWidthMeters, 0.001f, radius * 0.4f);
    const float step = std::clamp(source.stepOffsetMeters, 0.0f, 0.8f);
    Vec3 position = source.centerWorld;
    Vec3 remaining = source.desiredDisplacementWorld;
    remaining.z = 0.0f;
    const std::uint32_t iterations = std::clamp(
        source.maximumIterations, 1u, 8u);

    for (std::uint32_t iteration = 0;
            iteration < iterations && remaining.lengthSquared() > 1.0e-10f;
            ++iteration) {
        physx::PxSweepHit hit;
        if (!sweepTraversalCapsule(*m_impl, position, radius, halfHeight,
                remaining, hit)) {
            position += remaining;
            remaining = {};
            break;
        }

        const float requestedDistance = remaining.length();
        const Vec3 direction = remaining / requestedDistance;
        const float advance = std::max(0.0f, hit.distance - skin);
        position += direction * advance;
        Vec3 left = remaining - direction * advance;
        const Vec3 normal = fromPhysX(hit.normal).normalized();

        // A low obstacle may be traversable. Test the same remainder with the
        // query capsule lifted by stepOffset; ground probing later brings it
        // back to the actual support surface.
        if (step > 0.0f && normal.z < 0.35f) {
            physx::PxSweepHit raisedHit;
            if (!sweepTraversalCapsule(*m_impl,
                    position + Vec3 { 0.0f, 0.0f, step }, radius,
                    halfHeight, left, raisedHit)) {
                position.z += step;
                position += left;
                remaining = {};
                continue;
            }
        }

        result.blocked = true;
        const float intoSurface = dot(left, normal);
        if (intoSurface < 0.0f) left -= normal * intoSurface;
        left.z = std::max(0.0f, left.z);
        result.slideDisplacementWorld += left;
        remaining = left;
    }

    const float downDistance = step
        + std::max(source.groundProbeDistanceMeters,
            source.groundAdhesionDistanceMeters) + skin;
    const Vec3 raised = position + Vec3 { 0.0f, 0.0f, step };
    result.ground = probeTraversalGround(*m_impl, raised, radius, halfHeight,
        downDistance, source.maximumSlopeDegrees);
    if (result.ground.hasSurface && result.ground.walkable) {
        const float descend = result.ground.distanceMeters - step;
        const bool withinAdhesion = descend <=
            source.groundAdhesionDistanceMeters + skin;
        if (descend <= source.groundProbeDistanceMeters + skin
            || descend < 0.0f) {
            position.z -= descend;
            result.mode = RagdollTraversalMode3D::Grounded;
            result.groundAdhesionActive = descend > skin && withinAdhesion;
        } else {
            result.mode = RagdollTraversalMode3D::Airborne;
        }
    } else if (result.ground.hasSurface) {
        result.mode = RagdollTraversalMode3D::SlidingSteep;
    } else {
        result.mode = RagdollTraversalMode3D::Airborne;
    }
    result.allowedDisplacementWorld = position - source.centerWorld;
    return result;
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

namespace {

class RagdollLinkQueryFilter final : public physx::PxQueryFilterCallback {
public:
    explicit RagdollLinkQueryFilter(
        const PhysicsScene3D::Impl& implementation)
        : m_implementation(implementation) {
    }

    physx::PxQueryHitType::Enum preFilter(
        const physx::PxFilterData&, const physx::PxShape* shape,
        const physx::PxRigidActor* actor, physx::PxHitFlags&) override {
        RagdollHandle3D ragdoll;
        std::uint32_t linkIndex = 0;
        return shape != nullptr
                && m_implementation.findRagdollLink(
                    actor, ragdoll, linkIndex)
            ? physx::PxQueryHitType::eBLOCK
            : physx::PxQueryHitType::eNONE;
    }

    physx::PxQueryHitType::Enum postFilter(
        const physx::PxFilterData&, const physx::PxQueryHit&,
        const physx::PxShape*, const physx::PxRigidActor*) override {
        return physx::PxQueryHitType::eBLOCK;
    }

private:
    const PhysicsScene3D::Impl& m_implementation;
};

template <typename NativeHit>
bool fillRagdollHit(const PhysicsScene3D::Impl& implementation,
    const NativeHit& nativeHit,
    PhysicsRagdollRayHit3D& hit) {
    if (!implementation.findRagdollLink(
            nativeHit.actor, hit.ragdoll, hit.linkIndex)) {
        return false;
    }
    hit.position = fromPhysX(nativeHit.position);
    hit.normal = fromPhysX(nativeHit.normal);
    hit.distance = nativeHit.distance;
    return true;
}

} // namespace

bool PhysicsScene3D::raycastRagdoll(const Ray3D& ray,
    float maximumDistance, PhysicsRagdollRayHit3D& hit) const {
    if (maximumDistance <= 0.0f
        || ray.direction.lengthSquared() <= 1.0e-10f) {
        return false;
    }
    physx::PxRaycastBuffer buffer;
    physx::PxQueryFilterData filter;
    filter.flags = physx::PxQueryFlag::eDYNAMIC
        | physx::PxQueryFlag::ePREFILTER;
    RagdollLinkQueryFilter callback(*m_impl);
    const bool found = m_impl->scene->raycast(toPhysX(ray.origin),
        toPhysX(ray.direction.normalized()), maximumDistance, buffer,
        physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL,
        filter, &callback);
    return found && buffer.hasBlock
        && fillRagdollHit(*m_impl, buffer.block, hit);
}

bool PhysicsScene3D::sweepSphereRagdoll(const Ray3D& ray, float radius,
    float maximumDistance, PhysicsRagdollRayHit3D& hit) const {
    if (maximumDistance <= 0.0f || radius <= 0.0f
        || ray.direction.lengthSquared() <= 1.0e-10f) {
        return false;
    }
    physx::PxSweepBuffer buffer;
    physx::PxQueryFilterData filter;
    filter.flags = physx::PxQueryFlag::eDYNAMIC
        | physx::PxQueryFlag::ePREFILTER;
    RagdollLinkQueryFilter callback(*m_impl);
    const physx::PxSphereGeometry geometry(radius);
    const physx::PxTransform pose(toPhysX(ray.origin));
    const bool found = m_impl->scene->sweep(geometry, pose,
        toPhysX(ray.direction.normalized()), maximumDistance, buffer,
        physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL,
        filter, &callback);
    return found && buffer.hasBlock
        && fillRagdollHit(*m_impl, buffer.block, hit);
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

bool PhysicsScene3D::beginRagdollGrab(RagdollHandle3D ragdollHandle,
    std::uint32_t linkIndex, Vec3 localGrabPoint,
    const PhysicsGrabTarget3D& target,
    const PhysicsHandleSettings3D& handleSettings) {
    m_impl->releaseGrab();
    Impl::RagdollRecord* record = m_impl->ragdoll(ragdollHandle);
    if (record == nullptr || linkIndex >= record->links.size()
        || record->links[linkIndex] == nullptr) {
        return false;
    }
    physx::PxArticulationLink* link = record->links[linkIndex];
    m_impl->grabJoint = physx::PxD6JointCreate(*m_impl->engine->physics,
        nullptr, physx::PxTransform(physx::PxIdentity), link,
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
    m_impl->grabRagdoll = ragdollHandle;
    m_impl->grabRagdollLink = linkIndex;
    updateGrabTarget(target, handleSettings);
    record->articulation->wakeUp();
    return true;
}

void PhysicsScene3D::updateGrabTarget(const PhysicsGrabTarget3D& target,
    const PhysicsHandleSettings3D& handleSettings) {
    if (m_impl->grabJoint == nullptr) return;
    physx::PxRigidBody* rigidBody = m_impl->grabbedRigidBody();
    if (rigidBody == nullptr) {
        m_impl->releaseGrab();
        return;
    }
    const float mass = std::max(MinimumMassKg, rigidBody->getMass());
    const float linearDamping = 2.0f
        * std::max(0.0f, handleSettings.linearDampingRatio)
        * std::sqrt(std::max(0.0f,
            handleSettings.linearStiffness * mass));
    float maximumForce = std::max(0.0f, handleSettings.maximumForce);
    if (m_impl->grabRagdoll) {
        // O controle global pode chegar a centenas de kN para props pesados.
        // Aplicar esse mesmo teto a um antebraco de poucos quilos transforma
        // a Physgun num canhao. Para ragdolls, limitamos aceleracao em vez de
        // uma forca absoluta, preservando a mesma resposta entre membros.
        constexpr float MaximumGrabAcceleration = 180.0f;
        maximumForce = std::min(maximumForce,
            mass * MaximumGrabAcceleration);
    }
    const physx::PxD6JointDrive linearDrive(
        std::max(0.0f, handleSettings.linearStiffness), linearDamping,
        maximumForce, false);
    m_impl->grabJoint->setDrive(physx::PxD6Drive::eX, linearDrive);
    m_impl->grabJoint->setDrive(physx::PxD6Drive::eY, linearDrive);
    m_impl->grabJoint->setDrive(physx::PxD6Drive::eZ, linearDrive);

    const physx::PxVec3 inertia = rigidBody->getMassSpaceInertiaTensor();
    const float averageInertia = std::max(1.0e-5f,
        (inertia.x + inertia.y + inertia.z) / 3.0f);
    const float angularDamping = 2.0f
        * std::max(0.0f, handleSettings.angularDampingRatio)
        * std::sqrt(std::max(0.0f,
            handleSettings.angularStiffness * averageInertia));
    float maximumTorque = std::max(0.0f,
        handleSettings.maximumTorque);
    if (m_impl->grabRagdoll) {
        constexpr float MaximumGrabAngularAcceleration = 140.0f;
        maximumTorque = std::min(maximumTorque,
            averageInertia * MaximumGrabAngularAcceleration);
    }
    const physx::PxD6JointDrive angularDrive(
        target.lockOrientation
            ? std::max(0.0f, handleSettings.angularStiffness) : 0.0f,
        target.lockOrientation ? angularDamping : 0.0f,
        target.lockOrientation
            ? maximumTorque : 0.0f,
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
RagdollHandle3D PhysicsScene3D::grabbedRagdoll() const {
    return m_impl->grabRagdoll;
}
std::uint32_t PhysicsScene3D::grabbedRagdollLink() const {
    return m_impl->grabRagdollLink;
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
                    settings.radius, command.ignoreRagdolls)) {
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
                settings.radius, command.ignoreRagdolls)) {
            m_impl->character->resize(capsuleCylinderHeight(targetHeight,
                settings.radius));
            state.crouched = wantsCrouch;
        }
    }

    Vec3 inputDirection = command.moveDirection;
    if (inputDirection.lengthSquared() > 1.0f) {
        inputDirection = inputDirection.normalized();
    }

    const float activeBodyHeight = state.crouched
        ? settings.crouchedHeight : settings.standingHeight;
    bool oceanContainsCharacter = false;
    float immersionAtFeet = 0.0f;
    if (m_impl->ocean) {
        const OceanVolume3D& ocean = *m_impl->ocean;
        const Vec2 local {
            state.position.x - ocean.center.x,
            state.position.y - ocean.center.y
        };
        const float feetHeight =
            state.position.z - activeBodyHeight * 0.5f;
        const bool horizontal =
            std::abs(local.x) <= ocean.halfExtents.x
            && std::abs(local.y) <= ocean.halfExtents.y;
        const bool aboveOceanFloor =
            feetHeight >= ocean.meanSeaLevelMeters - ocean.depthMeters;
        if (horizontal && aboveOceanFloor) {
            const float surface = evaluateOceanSurface(
                { state.position.x, state.position.y },
                ocean.meanSeaLevelMeters, m_impl->oceanTimeSeconds)
                    .heightMeters;
            oceanContainsCharacter = true;
            immersionAtFeet = surface - feetHeight;
        }
    }
    if (state.flying || !oceanContainsCharacter) {
        state.swimming = false;
    } else {
        // Água rasa continua permitindo caminhar. A transição para natação
        // acontece quando o tronco está imerso e usa duas profundidades
        // diferentes para não oscilar na superfície.
        state.swimming = state.swimming
            ? immersionAtFeet > 0.38f
            : immersionAtFeet > 0.72f;
    }

    if (state.flying) {
        const float speed = command.sprint
            ? settings.fastFlightSpeed : settings.flightSpeed;
        // Voo editorial deliberadamente sem inercia: soltar a tecla zera o
        // movimento, mantendo controle preciso. A saida do modo preserva a
        // velocidade corrente conforme tratado acima.
        state.velocity = inputDirection * speed;
    } else if (state.swimming) {
        const float speed = command.sprint
            ? settings.fastSwimSpeed : settings.swimSpeed;
        const Vec3 desired = inputDirection * speed;
        // A aproximação vetorial funciona como arrasto hidrodinâmico sem
        // cancelar a inércia em um único quadro.
        state.velocity = moveToward(state.velocity, desired,
            settings.swimAcceleration * deltaTime);
        state.grounded = false;
        m_impl->coyoteRemaining = 0.0f;
        m_impl->jumpBufferRemaining = 0.0f;
    } else {
        const float speed = (state.crouched ? settings.crouchedSpeed
            : command.sprint ? settings.sprintSpeed : settings.walkSpeed)
            * std::clamp(command.speedScale, 0.0f, 2.0f);
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
            m_impl->jumpChargeSeconds = command.jumpChargeSeconds;
        } else {
            m_impl->jumpBufferRemaining = std::max(0.0f,
                m_impl->jumpBufferRemaining - deltaTime);
        }
        if (m_impl->jumpBufferRemaining > 0.0f
            && m_impl->coyoteRemaining > 0.0f) {
            state.velocity.z = characterJumpSpeed3D(settings,
                m_impl->jumpChargeSeconds);
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
    const physx::PxU32 characterMask = command.ignoreRagdolls
        ? (0xFFFFFFFFu & ~RagdollCollisionLayer) : 0xFFFFFFFFu;
    const physx::PxFilterData collisionFilter = state.flying
        ? physx::PxFilterData(0u, 0u, 0u, 0u)
        : physx::PxFilterData(1u, characterMask, 0u, 0u);
    if (physx::PxRigidDynamic* actor = m_impl->character->getActor()) {
        std::array<physx::PxShape*, 2> shapes {};
        const physx::PxU32 shapeCount = actor->getShapes(shapes.data(),
            static_cast<physx::PxU32>(shapes.size()));
        for (physx::PxU32 index = 0; index < shapeCount; ++index) {
            shapes[index]->setSimulationFilterData(collisionFilter);
            shapes[index]->setQueryFilterData(collisionFilter);
        }
    }
    CharacterQueryFilter queryFilter(m_impl->character->getActor());
    const physx::PxControllerFilters filters(&collisionFilter, &queryFilter);
    const physx::PxControllerCollisionFlags flags =
        m_impl->character->move(toPhysX((state.velocity + Vec3 {
                command.followVelocity.x, command.followVelocity.y, 0.0f })
                * deltaTime),
            0.0001f, deltaTime, filters);
    state.grounded = !state.flying && !state.swimming
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

void PhysicsScene3D::setOcean(const OceanVolume3D& ocean) {
    if (ocean.halfExtents.x <= 0.0f || ocean.halfExtents.y <= 0.0f
        || ocean.depthMeters <= 0.0f
        || ocean.densityKgPerCubicMeter <= 0.0f) {
        throw std::invalid_argument("Dominio fisico do oceano invalido");
    }
    m_impl->ocean = ocean;
}

void PhysicsScene3D::clearOcean() {
    m_impl->ocean.reset();
    m_impl->activeOceanBodies.clear();
}

void PhysicsScene3D::setOceanTimeSeconds(float timeSeconds) {
    if (std::isfinite(timeSeconds)) {
        m_impl->oceanTimeSeconds = timeSeconds;
    }
}

void PhysicsScene3D::simulate(float deltaTime) {
    if (deltaTime <= 0.0f || !std::isfinite(deltaTime)) return;
    using Clock = std::chrono::steady_clock;
    const auto stepStart = Clock::now();
    m_impl->contactCallbackNanoseconds.store(0, std::memory_order_relaxed);
    m_impl->reportedContactPairs.store(0, std::memory_order_relaxed);
    m_impl->reportedContactPoints.store(0, std::memory_order_relaxed);
    m_impl->dispatcher->resetStepCounters();
    m_impl->contactImpacts.clear();
    m_impl->contactSlides.clear();
    for (Impl::RagdollSlot& slot : m_impl->ragdollSlots) {
        if (slot.record) {
            slot.record->stepContacts.clear();
            std::fill(slot.record->stepInteractions.begin(), slot.record->stepInteractions.end(), RagdollInteraction3D {});
        }
    }

    // Safe point da articulation: o passo anterior já terminou em
    // fetchResults() e o próximo simulate() ainda não começou.
    for (const Impl::RagdollCommand& command :
            m_impl->pendingRagdollCommands) {
        Impl::RagdollRecord* record = m_impl->ragdoll(command.handle);
        if (record == nullptr) continue;
        switch (command.type) {
        case Impl::RagdollCommandType::SetRigidity:
            record->passiveRigidityPercent = command.value;
            if (!record->active) {
                applyRagdollRigidity(*record, command.value, true);
            }
            break;
        case Impl::RagdollCommandType::CapturePose:
            if (record->active) break;
            captureRagdollTargets(*record);
            applyRagdollRigidity(*record,
                record->rigidityPercent, false);
            break;
        case Impl::RagdollCommandType::NeutralPose:
            if (record->active) break;
            for (std::size_t linkIndex = 1;
                    linkIndex < record->joints.size(); ++linkIndex) {
                auto& runtime = record->joints[linkIndex];
                const auto& jointDefinition =
                    record->profile.links[linkIndex].inboundJoint;
                for (std::size_t axisIndexValue = 0;
                        axisIndexValue < runtime.targets.size();
                        ++axisIndexValue) {
                    if (!jointDefinition.axes[axisIndexValue].enabled) {
                        continue;
                    }
                    runtime.targets[axisIndexValue] = std::clamp(0.0f,
                        jointDefinition.axes[axisIndexValue].minimumRadians,
                        jointDefinition.axes[axisIndexValue].maximumRadians);
                }
            }
            applyRagdollRigidity(*record,
                record->rigidityPercent, false);
            break;
        case Impl::RagdollCommandType::Release:
            record->passiveRigidityPercent = 0.0f;
            if (!record->active) {
                applyRagdollRigidity(*record, 0.0f, false);
            }
            break;
        case Impl::RagdollCommandType::SetActive: {
            const bool active = command.value > 0.5f;
            if (record->active == active) break;
            record->active = active;
            record->state.active = active;
            record->articulation->setSolverIterationCounts(
                active
                    ? std::max(8u,
                        m_impl->settings.solverPositionIterations)
                    : m_impl->settings.solverPositionIterations,
                active
                    ? std::max(2u,
                        m_impl->settings.solverVelocityIterations)
                    : m_impl->settings.solverVelocityIterations);
            if (active) {
                const float passiveRigidity =
                    record->passiveRigidityPercent;
                applyRagdollRigidity(*record, 0.0f, false);
                record->passiveRigidityPercent = passiveRigidity;
            } else {
                record->gravityCompensationEnabled = false;
                applyRagdollRigidity(*record,
                    record->passiveRigidityPercent, true);
            }
            break;
        }
        case Impl::RagdollCommandType::SetFrozen: {
            const bool frozen = command.value > 0.5f;
            if (record->frozen == frozen || record->aggregate == nullptr
                || record->articulation == nullptr) {
                break;
            }
            if (frozen) {
                if (m_impl->grabRagdoll == record->handle) {
                    m_impl->releaseGrab();
                }
                record->animationConstraintStaged = false;
                record->gravityCompensationEnabled = false;
                record->stagedGravityCompensationEnabled = false;
                if (record->articulation->getScene() != nullptr) {
                    record->articulation->putToSleep();
                    m_impl->scene->removeAggregate(*record->aggregate, false);
                }
                record->frozen = true;
                record->state.frozen = true;
                record->state.sleeping = true;
                record->state.contacts.clear();
                for (PhysicsBodyState3D& linkState : record->state.links) {
                    linkState.linearVelocity = {};
                    linkState.angularVelocity = {};
                    linkState.sleeping = true;
                }
            } else {
                m_impl->scene->addAggregate(*record->aggregate);
                record->frozen = false;
                record->state.frozen = false;
                record->state.sleeping = false;
                record->articulation->wakeUp();
            }
            break;
        }
        }
    }
    m_impl->pendingRagdollCommands.clear();
    for (Impl::RagdollSlot& slot : m_impl->ragdollSlots) {
        if (!slot.record || !slot.record->active || slot.record->frozen) {
            continue;
        }
        Impl::RagdollRecord& record = *slot.record;
        if (record.stagedActiveTargetsDirty) {
            applyActiveRagdollTargets(record);
        }
        record.animationStepOrigin=record.links.front()->getGlobalPose().p;
        record.gravityCompensationEnabled =
            record.stagedGravityCompensationEnabled;
        applyActiveRagdollFeedforward(record);
    }

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

    // Novo solver oceânico: cinco amostras de volume por ator. Não existe
    // shape/collider na superfície; somente forças contínuas de empuxo e
    // arrasto aplicadas aos corpos que cruzam o domínio.
    if (m_impl->ocean) {
        const OceanVolume3D& ocean = *m_impl->ocean;
        const float oceanMinX = ocean.center.x - ocean.halfExtents.x;
        const float oceanMaxX = ocean.center.x + ocean.halfExtents.x;
        const float oceanMinY = ocean.center.y - ocean.halfExtents.y;
        const float oceanMaxY = ocean.center.y + ocean.halfExtents.y;
        const float oceanFloor =
            ocean.meanSeaLevelMeters - ocean.depthMeters;
        const std::array<Vec2, 5> SampleOffsets {{
            { 0.0f, 0.0f },
            { -1.0f, -1.0f }, { 1.0f, -1.0f },
            { -1.0f, 1.0f }, { 1.0f, 1.0f }
        }};
        constexpr std::array<float, 5> SampleWeights {
            0.28f, 0.18f, 0.18f, 0.18f, 0.18f
        };

        for (Impl::BodyRecord* record : m_impl->activeOceanBodies) {
            if (record == nullptr || record->frozen
                || record->definition.bodyVolumeCubicMeters <= 0.0f) {
                continue;
            }
            auto* dynamic = record->actor->is<physx::PxRigidDynamic>();
            if (dynamic == nullptr || dynamic->isSleeping()
                || dynamic->getRigidBodyFlags().isSet(
                    physx::PxRigidBodyFlag::eKINEMATIC)) {
                continue;
            }
            const physx::PxBounds3 bounds = record->actor->getWorldBounds();
            if (bounds.isEmpty()
                || bounds.maximum.x < oceanMinX
                || bounds.minimum.x > oceanMaxX
                || bounds.maximum.y < oceanMinY
                || bounds.minimum.y > oceanMaxY
                || bounds.maximum.z < oceanFloor
                || bounds.minimum.z > ocean.meanSeaLevelMeters
                    + OceanMaximumDisplacementMeters) {
                continue;
            }

            const Vec3 center {
                (bounds.minimum.x + bounds.maximum.x) * 0.5f,
                (bounds.minimum.y + bounds.maximum.y) * 0.5f,
                (bounds.minimum.z + bounds.maximum.z) * 0.5f
            };
            const float height =
                std::max(0.01f, bounds.maximum.z - bounds.minimum.z);
            const float offsetX =
                (bounds.maximum.x - bounds.minimum.x) * 0.34f;
            const float offsetY =
                (bounds.maximum.y - bounds.minimum.y) * 0.34f;
            const float maximumLift =
                ocean.densityKgPerCubicMeter * 9.81f
                * record->definition.bodyVolumeCubicMeters;
            float totalSubmersion = 0.0f;
            float surfaceVerticalSpeed = 0.0f;

            for (std::size_t sampleIndex = 0;
                    sampleIndex < SampleOffsets.size(); ++sampleIndex) {
                const Vec2 offset = SampleOffsets[sampleIndex];
                const Vec2 position {
                    center.x + offset.x * offsetX,
                    center.y + offset.y * offsetY
                };
                if (position.x < oceanMinX || position.x > oceanMaxX
                    || position.y < oceanMinY || position.y > oceanMaxY) {
                    continue;
                }
                const OceanSurfaceSample3D surface = evaluateOceanSurface(
                    position, ocean.meanSeaLevelMeters,
                    m_impl->oceanTimeSeconds);
                const float submerged = std::clamp(
                    (surface.heightMeters - bounds.minimum.z) / height,
                    0.0f, 1.0f);
                const float weighted =
                    submerged * SampleWeights[sampleIndex];
                totalSubmersion += weighted;
                surfaceVerticalSpeed +=
                    surface.verticalSpeedMetersPerSecond
                    * weighted;
                if (weighted <= 0.0f) continue;

                physx::PxRigidBodyExt::addForceAtPos(*dynamic,
                    physx::PxVec3(0.0f, 0.0f, maximumLift * weighted),
                    toPhysX({ position.x, position.y, center.z }),
                    physx::PxForceMode::eFORCE, false);
            }
            if (totalSubmersion <= 0.0f) continue;

            const Vec3 fluidVelocity { 0.0f, 0.0f,
                surfaceVerticalSpeed / std::max(totalSubmersion, 0.01f) };
            const Vec3 relative =
                fromPhysX(dynamic->getLinearVelocity()) - fluidVelocity;
            const float speed = relative.length();
            if (speed > 0.001f) {
                const float area = std::max(0.01f, record->definition
                    .aerodynamicReferenceAreaSquareMeters);
                float drag = 0.5f * ocean.densityKgPerCubicMeter
                    * 0.34f * area * speed * speed * totalSubmersion;
                drag = std::min(drag,
                    dynamic->getMass() * speed * 0.72f / deltaTime);
                dynamic->addForce(toPhysX(relative * (-drag / speed)),
                    physx::PxForceMode::eFORCE, false);
            }
            const Vec3 spin = fromPhysX(dynamic->getAngularVelocity());
            dynamic->addTorque(toPhysX(spin
                    * (-dynamic->getMass() * totalSubmersion * 1.4f)),
                physx::PxForceMode::eFORCE, false);
        }
    }
    const auto dispatchStart = Clock::now();
    // O custo de particionar uma cena pequena pode superar o trabalho do
    // solver. A estimativa combina atores ativos e contatos do passo anterior
    // e cresce gradualmente ate o limite do pool compartilhado.
    std::size_t activeRagdollDofs = 0;
    for (const Impl::RagdollSlot& slot : m_impl->ragdollSlots) {
        if (slot.record && !slot.record->state.sleeping) {
            activeRagdollDofs += ragdollDegreesOfFreedom3D(
                slot.record->profile);
        }
    }
    const std::size_t estimatedWork = std::max(
        m_impl->activeAerodynamicBodies.size(),
        m_impl->diagnostics.activeDynamicBodyCount)
        + activeRagdollDofs
        + m_impl->diagnostics.discreteContactPairs * 2;
    // Abaixo de ~512 unidades de trabalho o dispatcher oficial também tende
    // a ganhar em modo zero-thread: toda a cadeia roda na thread chamadora.
    // Acima disso liberamos paralelismo gradualmente, evitando lançar dez
    // workers para uma ilha pequena.
    const std::uint32_t desiredWorkers = estimatedWork < 512
        ? 0u
        : static_cast<std::uint32_t>(
            std::max<std::size_t>(2, (estimatedWork + 383) / 384));
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

    const auto stateSyncStart = Clock::now();
    for (Impl::RagdollSlot& slot : m_impl->ragdollSlots) {
        if (slot.record) {
            resolveRagdollAnimationConstraint(*slot.record,deltaTime,m_impl->grabRagdoll==slot.record->handle);
            updateRagdollState(*slot.record);
        }
    }

    // eENABLE_ACTIVE_ACTORS fornece apenas atores que alteraram estado neste
    // passo. O mesmo conjunto alimenta arrasto do proximo passo e snapshots
    // graficos, eliminando varreduras e leituras nativas de corpos em sleep.
    m_impl->activeAerodynamicBodies.clear();
    m_impl->activeOceanBodies.clear();
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
        const bool eligibleForExternalForce = dynamic != nullptr
            && !record->frozen && !dynamic->isSleeping()
            && !dynamic->getRigidBodyFlags().isSet(
                physx::PxRigidBodyFlag::eKINEMATIC);
        if (eligibleForExternalForce
            && record->definition.aerodynamicDragEnabled) {
            m_impl->activeAerodynamicBodies.push_back(record);
        }
        if (eligibleForExternalForce
            && record->definition.bodyVolumeCubicMeters > 0.0f) {
            m_impl->activeOceanBodies.push_back(record);
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
    const auto stateSyncEnd = Clock::now();

    const auto statisticsStart = Clock::now();
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
    m_impl->diagnostics.reportedContactPairs =
        m_impl->reportedContactPairs.load(std::memory_order_relaxed);
    m_impl->diagnostics.reportedContactPoints =
        m_impl->reportedContactPoints.load(std::memory_order_relaxed);
    m_impl->diagnostics.submittedPhysicsTasks =
        m_impl->dispatcher->submittedTaskCount();
    m_impl->diagnostics.preSimulationMilliseconds =
        std::chrono::duration<float, std::milli>(
            dispatchStart - stepStart).count();
    m_impl->diagnostics.simulationDispatchMilliseconds =
        std::chrono::duration<float, std::milli>(
            dispatchEnd - dispatchStart).count();
    m_impl->diagnostics.simulationWaitMilliseconds =
        std::chrono::duration<float, std::milli>(
            fetchEnd - dispatchEnd).count();
    m_impl->diagnostics.contactCallbackMilliseconds =
        static_cast<float>(m_impl->contactCallbackNanoseconds.load(
            std::memory_order_relaxed)) * 1.0e-6f;
    m_impl->diagnostics.stateSyncMilliseconds =
        std::chrono::duration<float, std::milli>(
            stateSyncEnd - stateSyncStart).count();
    m_impl->diagnostics.statisticsMilliseconds =
        std::chrono::duration<float, std::milli>(
            Clock::now() - statisticsStart).count();
    m_impl->diagnostics.totalStepMilliseconds =
        std::chrono::duration<float, std::milli>(
            Clock::now() - stepStart).count();
}

std::span<const ContactImpactEvent3D>
PhysicsScene3D::contactImpacts() const {
    return m_impl->contactImpacts;
}

std::span<const ContactSlideEvent3D>
PhysicsScene3D::contactSlides() const {
    return m_impl->contactSlides;
}

const PhysicsStepDiagnostics3D& PhysicsScene3D::diagnostics() const {
    return m_impl->diagnostics;
}

std::span<const PhysicsBodyStateUpdate3D>
PhysicsScene3D::activeBodyStates() const {
    return m_impl->activeBodyStateUpdates;
}

} // namespace MatterEngine
