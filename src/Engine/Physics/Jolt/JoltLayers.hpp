#pragma once

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

namespace MatterEngine {

// Camadas de colisao do backend Jolt.
//
// O PhysX usava par layer/mask de 32 bits por shape, vindo de
// PhysicsBodyDefinition3D. Na pratica NENHUM chamador jamais definiu esses
// campos: todo corpo ficava no default (layer 1, mask 0xFFFFFFFF), e a unica
// distincao real era um bit que o proprio backend ligava para links de ragdoll.
// Um bitmask configuravel que ninguem configura e um contrato falso, entao os
// campos sairam da interface neutra junto com esta migracao.
//
// O que sobrou e o que era de fato usado: quatro papeis semanticos. Se um dia
// existir necessidade real (uma bola que atravessa a rede do gol, digamos), ela
// volta como papel nomeado aqui, nao como mascara crua no contrato.
namespace JoltObjectLayers {

// Geometria estatica do mundo: piso, plataforma, obstaculos.
inline constexpr JPH::ObjectLayer NonMoving = 0;
// Props dinamicos e corpos cinematicos.
inline constexpr JPH::ObjectLayer Moving = 1;
// Links de articulacao. Separado de Moving porque o personagem controlado
// precisa poder ignorar a propria representacao articulada ao varrer a capsula
// (CharacterMotorCommand3D::ignoreRagdolls).
inline constexpr JPH::ObjectLayer Ragdoll = 2;
// Corpo interno do character controller. Existe para que consultas e outros
// corpos VEJAM o personagem, como o ator cinematico do CCT do PhysX fazia.
inline constexpr JPH::ObjectLayer Character = 3;

inline constexpr JPH::uint Count = 4;

} // namespace JoltObjectLayers

namespace JoltBroadPhaseLayers {

inline constexpr JPH::BroadPhaseLayer NonMoving { 0 };
inline constexpr JPH::BroadPhaseLayer Moving { 1 };

inline constexpr JPH::uint Count = 2;

} // namespace JoltBroadPhaseLayers

// Agrupamento de broad phase: tudo que se move num lado, mundo estatico no
// outro. A separacao e o que permite a arvore estatica nunca ser reconstruida.
class JoltBroadPhaseLayerInterface final
    : public JPH::BroadPhaseLayerInterface {
public:
    [[nodiscard]] JPH::uint GetNumBroadPhaseLayers() const override {
        return JoltBroadPhaseLayers::Count;
    }

    [[nodiscard]] JPH::BroadPhaseLayer GetBroadPhaseLayer(
        JPH::ObjectLayer layer) const override;

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    [[nodiscard]] const char* GetBroadPhaseLayerName(
        JPH::BroadPhaseLayer layer) const override;
#endif
};

class JoltObjectVsBroadPhaseLayerFilter final
    : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    [[nodiscard]] bool ShouldCollide(JPH::ObjectLayer layer,
        JPH::BroadPhaseLayer broadPhaseLayer) const override;
};

// Matriz de colisao entre papeis. A auto-colisao DENTRO de um ragdoll nao e
// resolvida aqui: e por CollisionGroup/GroupFilterTable por instancia, senao
// dois bonecos distintos deixariam de colidir entre si.
class JoltObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter {
public:
    [[nodiscard]] bool ShouldCollide(JPH::ObjectLayer first,
        JPH::ObjectLayer second) const override;
};

} // namespace MatterEngine
