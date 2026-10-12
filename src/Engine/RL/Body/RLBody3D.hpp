#pragma once

#include "Engine/Physics/PhysicsTypes3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// Um eixo atuado do corpo de treino: o que a politica comanda.
//
// O alvo da junta sai da acao (em [-1, 1]) por uma reta em cada lado da pose
// base: acao 0 segura a pose base, +1 vai ao limite maximo e -1 ao minimo.
// A politica alcanca o curso inteiro da junta sem que a pose de pe deixe de
// ser o zero dela.
struct RLBodyDof3D {
    std::string name;
    std::uint32_t linkIndex = 0;
    RagdollAxis3D axis = RagdollAxis3D::Twist;
    float minimumRadians = 0.0f;
    float maximumRadians = 0.0f;
    float baseRadians = 0.0f;
    // Rigidez (N.m/rad), amortecimento (N.m.s/rad) e teto de torque (N.m)
    // de verdade, e as escalas sobre o perfil que o backend recebe.
    float stiffness = 0.0f;
    float damping = 0.0f;
    float maximumTorque = 0.0f;
    float stiffnessScale = 0.0f;
    float dampingScale = 0.0f;
    float maximumTorqueScale = 0.0f;

    [[nodiscard]] float target(float action) const;
    // Radianos por unidade de acao, na media dos dois lados (para o ruido
    // inicial da politica sair do mesmo tamanho em todas as juntas).
    [[nodiscard]] float meanRadiansPerAction() const;
};

// O corpo de treino: o perfil do jogo mais os atuadores do RL por cima
// (assets/rl/bodies/<id>.json). O perfil do jogo nao muda; os tetos e os
// ganhos do RL entram pelas escalas de cada alvo.
struct RLBody3D {
    std::string id;
    std::filesystem::path definitionPath;
    std::filesystem::path profilePath;
    RagdollProfile3D profile;
    std::vector<RLBodyDof3D> dofs;
    // Coordenadas da pose base por link (o vetor de rotacao de cada junta) e
    // a inclinacao da pelve nela.
    std::vector<Vec3> baseJointCoordinates;
    Quaternion baseRootRotation;
    std::uint32_t solverPositionIterations = 0;
    std::uint32_t solverVelocityIterations = 0;
    // Links que podem tocar o chao sem ser queda (os pes).
    std::vector<std::uint8_t> supportLink;
    std::uint32_t leftFoot = 0;
    std::uint32_t rightFoot = 0;
    std::uint32_t head = 0;
    std::uint32_t leftHand = 0;
    std::uint32_t rightHand = 0;
    std::uint32_t upperChest = 0;
    // Medidas da pose base apoiada no chao (z = 0).
    float standingPelvisHeight = 0.0f;
    float standingHeadHeight = 0.0f;
    // Identidade do corpo: hash do perfil e da definicao (FNV-1a de 64 bits).
    std::uint64_t hash = 0;
};

[[nodiscard]] RLBody3D loadRLBody3D(const std::filesystem::path& definitionPath);

// Poses dos links (origem, como PhysicsBodyState3D) a partir das coordenadas
// das juntas, com a raiz dada. Mesma convencao do backend e dos clipes.
struct RLBodyPose3D {
    std::vector<Vec3> positions;
    std::vector<Quaternion> orientations;
};

[[nodiscard]] RLBodyPose3D rlBodyPose3D(const RagdollProfile3D& profile, Vec3 rootPosition,
    Quaternion rootOrientation, std::span<const Vec3> jointCoordinates);

// Os links com velocidades: a pose em q e em q + qdot*h, por diferenca.
void rlBodyLinkStates3D(const RagdollProfile3D& profile, Vec3 rootPosition, Quaternion rootOrientation,
    Vec3 rootLinearVelocity, Vec3 rootAngularVelocity, std::span<const Vec3> jointCoordinates,
    std::span<const Vec3> jointVelocities, std::vector<PhysicsBodyState3D>& out);

// O ponto mais baixo dos colisores dos pes numa pose (z do mundo).
[[nodiscard]] float rlBodyLowestFootPoint3D(const RLBody3D& body, const RLBodyPose3D& pose);

// Coordenadas de junta de uma orientacao relativa (inverso do usado acima).
[[nodiscard]] Vec3 rlJointCoordinates3D(const RagdollProfile3D& profile, std::size_t linkIndex,
    Quaternion parentOrientation, Quaternion childOrientation);

[[nodiscard]] Quaternion rlQuaternionExp3D(Vec3 rotationVector);
[[nodiscard]] Vec3 rlQuaternionLog3D(Quaternion rotation);

} // namespace MatterEngine
