#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <cstddef>
#include <vector>

namespace MatterEngine {

// Cinematica do ragdoll sobre coordenadas de junta (vetor de rotacao no
// referencial de cada junta, como os alvos dos motores): direta (pose no
// referencial da raiz) e inversa por minimos quadrados amortecidos. Usada
// pelo controlador de locomocao e pelo levantar pelo corpo.

// Rotacao de uma coordenada de junta (exponencial do vetor de rotacao).
[[nodiscard]] Quaternion ragdollJointRotation3D(Vec3 coordinates);
// Velocidade angular para uma variacao `velocity` das coordenadas.
[[nodiscard]] Vec3 ragdollJointJacobian3D(Vec3 coordinates, Vec3 velocity);

// Posicao e orientacao de cada link no referencial da raiz.
void buildRagdollLocalPose3D(const RagdollProfile3D& profile,
    const std::vector<Vec3>& coordinates, RagdollAnimationPose3D& pose);

// Leva um ponto preso ao link endIndex (pointInEnd, no referencial dele) a um
// alvo no referencial da raiz, mexendo as juntas dele e dos ancestrais ate
// `depth` links, dentro dos limites. Parte das coordenadas atuais (a solucao
// fica perto da pose dada); weight mistura o resultado. axisWeights (por link,
// um peso 0..1 por eixo) restringe quanto cada eixo participa: 0 trava.
void solveRagdollChainPoint3D(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t endIndex, Vec3 pointInEnd,
    int depth, Vec3 targetRootLocal, float weight, int iterations = 7,
    const std::vector<Vec3>* axisWeights = nullptr);

} // namespace MatterEngine
