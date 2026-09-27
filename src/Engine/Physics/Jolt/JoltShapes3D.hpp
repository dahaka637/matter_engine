#pragma once

#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace MatterEngine {

// Material de superficie visto pelo Jolt. Existe para que o id do material da
// engine viaje DENTRO da shape: num contato, `Shape::GetMaterial(subShapeID)`
// resolve atravessando compounds sozinho, e o backend nao precisa refazer
// aritmetica de indice de sub-shape para descobrir em que material bateu - que
// era a parte fragil de reconstruir o material do lado do PhysX.
class JoltSurfaceMaterial final : public JPH::PhysicsMaterial {
public:
    explicit JoltSurfaceMaterial(std::string materialId)
        : m_materialId(std::move(materialId)) {
    }

    [[nodiscard]] const char* GetDebugName() const override {
        return m_materialId.c_str();
    }

    [[nodiscard]] const std::string& materialId() const {
        return m_materialId;
    }

private:
    std::string m_materialId;
};

// Resolve o id do material de uma shape do Jolt, caindo para "default" quando
// a shape nao carrega um JoltSurfaceMaterial. Devolve view para a string dentro
// do material, que vive junto da shape.
[[nodiscard]] std::string_view joltShapeMaterialId(
    const JPH::PhysicsMaterial* material);

// Traduz uma shape do contrato neutro. Aplica o offset local (posicao e
// orientacao) envolvendo em RotatedTranslatedShape somente quando ha offset -
// uma shape centrada nao paga decorador.
//
// Convencoes que casam sem correcao: caixa por meias-extensoes, e capsula
// longitudinal em Y (a do PhysX era em X e exigia rotacao de 90 graus).
[[nodiscard]] JPH::RefConst<JPH::Shape> createJoltShape(
    const PhysicsShape3D& shape, bool allowConcaveMesh);

// Uma shape isolada quando ha somente uma sem offset; StaticCompoundShape
// quando ha varias. Compound estatico e o certo aqui: a composicao de um corpo
// nao muda depois da criacao, e a variante mutavel custaria mais por consulta.
[[nodiscard]] JPH::RefConst<JPH::Shape> createJoltBodyShape(
    std::span<const PhysicsShape3D> shapes, bool allowConcaveMesh);

} // namespace MatterEngine
