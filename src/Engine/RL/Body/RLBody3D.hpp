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

// O maior modulo de acao que o corpo aceita (o resto o ambiente corta).
inline constexpr float RLActionLimit = 10.0f;

// Um eixo atuado do corpo de treino: o que a politica comanda.
//
// O alvo da junta e a pose base mais actionScale radianos por unidade de
// acao, limitado ao curso da junta: acao 0 segura a pose base. A acao nao e
// presa em [-1, 1] (so em +-ActionLimit): o ruido de exploracao fica pequeno
// em radianos (~0,15 rad com desvio 0,6) e a politica ainda alcanca o curso
// inteiro pedindo valores maiores (o mapa dos robos com pernas, rsl_rl).
struct RLBodyDof3D {
    std::string name;
    std::uint32_t linkIndex = 0;
    RagdollAxis3D axis = RagdollAxis3D::Twist;
    float minimumRadians = 0.0f;
    float maximumRadians = 0.0f;
    float baseRadians = 0.0f;
    float actionScale = 0.25f;
    // Rigidez (N.m/rad), amortecimento (N.m.s/rad) e teto de torque (N.m)
    // de verdade, e as escalas sobre o perfil que o backend recebe.
    float stiffness = 0.0f;
    float damping = 0.0f;
    float maximumTorque = 0.0f;
    float stiffnessScale = 0.0f;
    float dampingScale = 0.0f;
    float maximumTorqueScale = 0.0f;

    [[nodiscard]] float target(float action) const;
};

// Uma postura parada (um quadro de clipe): o que uma tarefa de ficar de pe
// pede. A primeira do corpo e a pose base ("natural", de onde a acao parte);
// as outras vem de "postures" no arquivo do corpo. A acao continua partindo
// da base: segurar outra postura e pedir os desvios dela.
struct RLPosture3D {
    std::string id;
    std::string displayName;
    // Por link (o vetor de rotacao de cada junta, dentro do curso) e a
    // inclinacao da pelve.
    std::vector<Vec3> jointCoordinates;
    Quaternion rootRotation;
    // O angulo de cada eixo atuado (na ordem de RLBody3D::dofs).
    std::vector<float> dofRadians;
    // Medidas com os pes no chao (z = 0) e o "para cima" da pelve, do peito
    // (UpperChest) e da cabeca no referencial do rumo.
    float standingPelvisHeight = 0.0f;
    float standingHeadHeight = 0.0f;
    Vec3 pelvisUp { 0.0f, 0.0f, 1.0f };
    Vec3 chestUp { 0.0f, 0.0f, 1.0f };
    Vec3 headUp { 0.0f, 0.0f, 1.0f };
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
    // O "para cima" da pelve, do peito (UpperChest) e da cabeca na pose base,
    // no referencial do rumo do corpo: a postura que a recompensa pede.
    Vec3 basePelvisUp { 0.0f, 0.0f, 1.0f };
    Vec3 baseChestUp { 0.0f, 0.0f, 1.0f };
    Vec3 baseHeadUp { 0.0f, 0.0f, 1.0f };
    // As posturas paradas: a base primeiro ("natural"), depois as do arquivo.
    std::vector<RLPosture3D> postures;
    // Os eixos travados ("lockedAxes" no arquivo; no corpo v2, os pulsos):
    // ficam fora da acao e da entrada, e o motor os segura na pose base.
    // Destravar e tirar da lista: o eixo volta como canal novo.
    std::vector<RLBodyDof3D> lockedDofs;
    // As partes de cada pe que pisam (0 esquerdo, 1 direito): o pe e, com a
    // junta dos dedos ("toes" no arquivo), a ponta (LeftToe, RightToe).
    std::array<std::vector<std::uint32_t>, 2> footParts;
    int leftToe = -1;
    int rightToe = -1;
    // Identidade do corpo: hash do perfil e da definicao (FNV-1a de 64 bits).
    std::uint64_t hash = 0;
};

[[nodiscard]] RLBody3D loadRLBody3D(const std::filesystem::path& definitionPath);

// O indice de uma postura do corpo pelo id (-1 se nao houver).
[[nodiscard]] int findRLPosture3D(const RLBody3D& body, std::string_view id);

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
