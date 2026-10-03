#pragma once

#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Physics/Articulation/ArticulationGravity3D.hpp"
#include "Engine/Physics/Articulation/ArticulationIndexing3D.hpp"
#include "Engine/Physics/Jolt/JoltConversions.hpp"
#include "Engine/Physics/Jolt/JoltJobSystem.hpp"
#include "Engine/Physics/Jolt/JoltLayers.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <Jolt/Jolt.h>

#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/PhysicsStepListener.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace MatterEngine {

// Estado interno do backend Jolt. Fica num header proprio porque a
// implementacao esta dividida por responsabilidade em varios .cpp - o backend
// PhysX concentrava tudo num arquivo de 3.532 linhas, e essa era uma das razoes
// declaradas para a reescrita.
//
// Este e o UNICO lugar em que <Jolt/...> e o contrato neutro aparecem juntos.

namespace JoltDetail {

// O Jolt exige registro global de processo (alocador, Factory, tipos de shape)
// antes de qualquer uso, e desregistro simetrico depois do ultimo. Como a
// engine pode ter mais de uma instancia viva - os testes de ciclo de vida criam
// e destroem varias -, o registro e contado por referencia.
class GlobalRegistration final {
public:
    static void acquire();
    static void release();

private:
    static std::mutex s_mutex;
    static std::size_t s_useCount;
};

// Identidade de um corpo do Jolt de volta para o dominio da engine.
// Substitui o empacotamento em PxFilterData::word3 que o backend PhysX usava.
enum class BodyKind : std::uint8_t {
    None = 0,
    // Corpo comum: rigido estatico, dinamico ou cinematico.
    Body = 1,
    // Link de uma articulacao.
    RagdollLink = 2,
    // Corpo interno do character controller.
    Character = 3
};

struct BodyIdentity {
    BodyKind kind = BodyKind::None;
    // Indice de slot em bodySlots ou ragdollSlots, conforme o kind.
    std::uint32_t slotIndex = 0;
    // Somente para RagdollLink.
    std::uint32_t linkIndex = 0;
};

[[nodiscard]] std::uint64_t packBodyIdentity(const BodyIdentity& identity);
[[nodiscard]] BodyIdentity unpackBodyIdentity(std::uint64_t userData);

} // namespace JoltDetail

struct PhysicsEngine3D::Impl final
    : std::enable_shared_from_this<PhysicsEngine3D::Impl> {
    std::shared_ptr<TaskScheduler> scheduler;
    bool registrationHeld = false;

    ~Impl();
};

struct PhysicsMesh3D::Impl final {
    // Mantem o registro global vivo enquanto qualquer shape nativa existir.
    std::shared_ptr<PhysicsEngine3D::Impl> engineLifetime;
    // Convex hull ou malha de triangulos, conforme PhysicsMesh3D::type().
    JPH::RefConst<JPH::Shape> shape;

    // A shape nativa de uma mesh publica. PhysicsMesh3D declara `friend struct
    // Impl`, portanto este e o unico ponto autorizado a alcancar m_impl.
    [[nodiscard]] static const Impl* of(const PhysicsMesh3D& mesh) {
        return mesh.m_impl.get();
    }
};

struct PhysicsScene3D::Impl final : public JPH::ContactListener,
                                    public JPH::PhysicsStepListener,
                                    public JPH::CharacterContactListener {
    struct BodyRecord {
        PhysicsBodyHandle3D handle;
        PhysicsBodyDefinition3D definition;
        JPH::BodyID bodyId;
        bool frozen = false;
        // Tipo de movimento original, para restaurar depois de descongelar.
        // O material por ponto de contato vem da propria shape (ver
        // JoltSurfaceMaterial), nao de um registro paralelo aqui.
        JPH::EMotionType motionType = JPH::EMotionType::Static;
    };

    struct BodySlot {
        std::uint32_t generation = 1;
        std::unique_ptr<BodyRecord> record;
    };

    struct RagdollJointRuntime {
        // O Jolt nao tem articulacao em coordenadas reduzidas: cada junta e uma
        // constraint de seis eixos entre pai e filho, com motor por eixo.
        JPH::Ref<JPH::SixDOFConstraint> constraint;
        // Alvos passivos (rigidez do laboratorio): capturados da pose ou
        // zerados. O modo ativo usa stagedActiveTargets. A numeracao de DOF
        // vem de RagdollRecord::indexing, derivada do perfil.
        std::array<float, 3> targets {};
        // O que os motores receberam neste tick (telemetria): alvos efetivos,
        // feedforward aplicado e limite nominal de torque, por eixo.
        std::array<float, 3> appliedTargets {};
        std::array<float, 3> appliedTargetVelocities {};
        std::array<float, 3> appliedFeedforward {};
        std::array<float, 3> appliedTorqueLimit {};
    };

    struct RagdollRecord {
        RagdollHandle3D handle;
        std::uint64_t entityId = 0;
        RagdollProfile3D profile;
        JPH::Ref<JPH::RagdollSettings> settings;
        // A auto-colisao (GroupFilterTable) vive dentro das settings.
        JPH::Ref<JPH::Ragdoll> ragdoll;
        std::vector<JPH::BodyID> links;
        // Alinhado com profile.links; o elemento zero nao possui inbound joint.
        std::vector<RagdollJointRuntime> joints;
        // Numeracao de DOF generalizado derivada do perfil, nao do SDK.
        ArticulationIndexing3D indexing;
        // Substitui computeGravityCompensation, que o Jolt nao tem.
        ArticulationGravityCompensator3D gravityCompensator;
        std::vector<ArticulationLinkPose3D> gravityPoses;
        RagdollState3D state;
        std::vector<RagdollContactPoint3D> stepContacts;
        std::vector<RagdollInteraction3D> stepInteractions;
        std::vector<RagdollDriveTarget3D> stagedActiveTargets;
        // Guia de animacao do tick, consumido depois do solver por
        // resolveRagdollGuides.
        RagdollAnimationConstraint3D animationConstraint;
        bool animationConstraintStaged = false;
        float externalForceMagnitude = 0.0f;
        float rigidityPercent = 0.0f;
        float passiveRigidityPercent = 0.0f;
        bool active = false;
        bool frozen = false;
        bool stagedActiveTargetsDirty = false;
        bool gravityCompensationEnabled = false;
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

    JoltBroadPhaseLayerInterface broadPhaseLayers;
    JoltObjectVsBroadPhaseLayerFilter objectVsBroadPhaseFilter;
    JoltObjectLayerPairFilter objectLayerPairFilter;
    std::unique_ptr<JoltJobSystem> jobSystem;
    std::unique_ptr<JPH::TempAllocator> tempAllocator;
    std::unique_ptr<JPH::PhysicsSystem> system;

    // Materiais de contato por id, resolvidos na criacao da cena.
    struct SurfaceContact {
        float staticFriction = 0.5f;
        float dynamicFriction = 0.5f;
        float restitution = 0.0f;
    };
    std::unordered_map<std::string, SurfaceContact> materials;

    std::vector<BodySlot> bodySlots;
    std::vector<std::uint32_t> freeBodySlots;
    std::vector<RagdollSlot> ragdollSlots;
    std::vector<std::uint32_t> freeRagdollSlots;
    std::vector<RagdollCommand> pendingRagdollCommands;

    std::vector<ContactImpactEvent3D> contactImpacts;
    std::vector<ContactSlideEvent3D> contactSlides;
    std::vector<PhysicsBodyStateUpdate3D> activeBodyStateUpdates;
    std::optional<OceanVolume3D> ocean;
    float oceanTimeSeconds = 0.0f;
    PhysicsStepDiagnostics3D diagnostics;

    // Os callbacks de contato do Jolt rodam em varios jobs em paralelo, com
    // todos os corpos travados. So o armazenamento dos eventos e serializado
    // (contactMutex, abaixo); consolidateContacts ordena depois do passo para
    // que a saida nao dependa de qual worker chegou primeiro. Medido com 22
    // bonecos em contato: o listener nao e gargalo (desligar os sensores nao
    // mudou o tempo do passo), entao buffers por worker nao se pagariam.
    std::atomic<std::size_t> reportedContactPairs { 0 };
    std::atomic<std::size_t> reportedContactPoints { 0 };

    // Physgun: motores de seis eixos entre um corpo cinematico auxiliar e o
    // alvo. O PhysX usava um PxD6Joint com a mesma ideia.
    JPH::Ref<JPH::SixDOFConstraint> grabConstraint;
    JPH::BodyID grabAnchorBody;
    PhysicsBodyHandle3D grabBody;
    RagdollHandle3D grabRagdoll;
    std::uint32_t grabRagdollLink =
        std::numeric_limits<std::uint32_t>::max();

    // Character controller. CharacterVirtual nao e simulado pelo
    // PhysicsSystem: ele resolve a propria varredura e precisa ser atualizado
    // explicitamente.
    std::unique_ptr<JPH::CharacterVirtual> character;
    CharacterMotorSettings3D characterSettings;
    PhysicsCharacterState3D characterState;
    float coyoteRemaining = 0.0f;
    // A capsula tem chao pisavel embaixo, ate a altura de um degrau (numa
    // quina de escada o Jolt a da como "em chao ingreme").
    bool characterSupported = false;
    float jumpBufferRemaining = 0.0f;
    // Carga do pulo pedido (guardada com o buffer de pulo).
    float jumpChargeSeconds = 0.0f;
    Vec3 characterMoveVelocity;
    bool characterIgnoreRagdolls = false;
    bool characterNavigationProxy = false;

    void OnContactAdded(const JPH::CharacterVirtual*, const JPH::CharacterContact&,
        JPH::CharacterContactSettings&) override;
    void OnContactPersisted(const JPH::CharacterVirtual*, const JPH::CharacterContact&,
        JPH::CharacterContactSettings&) override;
    void pushCharacterContact(const JPH::CharacterContact&, JPH::CharacterContactSettings&);
    float characterDeltaTime = 1.0f / 120.0f;
    JPH::BodyIDVector previouslyActiveBodies;
    int collisionSteps = 1;
    float stepDeltaTime = 1.0f / 120.0f;
    std::mutex contactMutex;
    std::atomic<bool> callbackFailed { false };
    std::atomic<std::size_t> allContactPairs { 0 };
    JPH::BodyIDVector activeBodies;
    struct BodyWrench { JPH::BodyID body; Vec3 force; Vec3 torque; };
    std::vector<BodyWrench> pendingBodyWrenches;
    void recordContact(const JPH::Body&, const JPH::Body&,
        const JPH::ContactManifold&, JPH::ContactSettings&, bool);
    void publishRagdoll(RagdollRecord&);
    ~Impl() override;

    // --- acesso a slots -----------------------------------------------------
    [[nodiscard]] BodyRecord* body(PhysicsBodyHandle3D handle);
    [[nodiscard]] const BodyRecord* body(PhysicsBodyHandle3D handle) const;
    [[nodiscard]] RagdollRecord* ragdoll(RagdollHandle3D handle);
    [[nodiscard]] const RagdollRecord* ragdoll(
        RagdollHandle3D handle) const;
    [[nodiscard]] BodyRecord* bodyFromId(JPH::BodyID bodyId);
    [[nodiscard]] RagdollRecord* ragdollFromId(JPH::BodyID bodyId,
        std::uint32_t& outLinkIndex);

    [[nodiscard]] const SurfaceContact& contactMaterial(
        const std::string& materialId) const;

    // --- JPH::ContactListener ----------------------------------------------
    void OnContactAdded(const JPH::Body& first, const JPH::Body& second,
        const JPH::ContactManifold& manifold,
        JPH::ContactSettings& settings) override;
    void OnContactPersisted(const JPH::Body& first,
        const JPH::Body& second, const JPH::ContactManifold& manifold,
        JPH::ContactSettings& settings) override;

    // --- JPH::PhysicsStepListener ------------------------------------------
    // Forcas externas (arrasto aerodinamico, empuxo, wrenches de controle) e o
    // feedforward dos drives entram aqui: e o ponto em que o Jolt permite ler e
    // escrever corpos com tudo travado, antes do solver do passo.
    void OnStep(const JPH::PhysicsStepListenerContext& context) override;

    // --- etapas internas do passo ------------------------------------------
    void applyPendingRagdollCommands();
    void applyAerodynamicDrag(float deltaTime);
    void applyBuoyancy(float deltaTime);
    void applyRagdollDrives();
    // Guia de animacao da raiz, depois do solver (semantica do PhysX).
    void resolveRagdollGuides();
    void applyPendingWrenches();
    void publishStepResults();
    void consolidateContacts();
    void releaseGrab();
    void releaseCharacter();
};

} // namespace MatterEngine
