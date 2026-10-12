#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/RLConstants3D.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

class BalanceEnvironmentBatch3D;
class PhysicsEngine3D;

struct RLVectorChannel3D {
  std::string name;
  std::string unit;
  float minimum = -1.0f;
  float maximum = 1.0f;
};

struct RLVectorSchema3D {
  std::string id;
  std::uint32_t version = 0;
  std::vector<RLVectorChannel3D> channels;

  [[nodiscard]] std::size_t size() const { return channels.size(); }
};

struct BalanceEnvironmentSettings3D {
  // 4 subpassos a 120 Hz = uma decisao de politica a 30 Hz.
  std::uint32_t actionRepeat = 4;
  std::uint32_t maximumEpisodePolicySteps = 30u * 20u;
  float groundClearanceMeters = 0.015f;

  // Escalas de normalizacao da observacao; todos os canais publicados sao
  // limitados aos intervalos descritos pelo schema.
  float heightErrorScaleMeters = 0.50f;
  float linearVelocityScaleMetersPerSecond = 5.0f;
  float angularVelocityScaleRadiansPerSecond = 10.0f;
  float jointVelocityScaleRadiansPerSecond = 20.0f;
  float centerOfMassOffsetScaleMeters = 0.50f;
  float footOffsetScaleMeters = 1.0f;

  // Primeiros ganhos do contrato, ainda provisórios enquanto torque e ROM
  // não forem aprovados biomecanicamente. Todos continuam limitados pelo
  // perfil físico; nenhuma força é aplicada à raiz.
  float driveStiffnessScale = 0.20f;
  float driveDampingScale = 0.50f;
  float driveMaximumTorqueScale = 1.0f;
  bool gravityCompensationEnabled = true;

  float terminationMinimumPelvisHeightFraction = 0.55f;
  float terminationMinimumPelvisUp = 0.25f;
};

struct BalanceReward3D {
  float upright = 0.0f;
  float height = 0.0f;
  float stillness = 0.0f;
  float actionPenalty = 0.0f;
  float total = 0.0f;
};

struct BalanceResetResult3D {
  std::vector<float> observation;
  std::uint64_t seed = 0;
};

struct BalanceStepResult3D {
  std::vector<float> observation;
  BalanceReward3D reward;
  std::uint32_t episodePolicyStep = 0;
  std::uint64_t episodePhysicsStep = 0;
  bool terminated = false;
  bool truncated = false;
  bool actionClipped = false;
  std::string terminationReason;
};

// Primeiro ambiente executável do projeto. É deliberadamente uma cena C++
// independente e headless: reset reconstrói a cena; step escreve apenas
// alvos dos motores, avança passos fixos e então lê o snapshot. Não há
// renderer, guia cinemático, transform escrito nem força auxiliar na raiz.
class BalanceEnvironment3D final {
public:
  explicit BalanceEnvironment3D(RagdollProfile3D profile,
                                BalanceEnvironmentSettings3D settings = {});
  ~BalanceEnvironment3D();

  BalanceEnvironment3D(const BalanceEnvironment3D &) = delete;
  BalanceEnvironment3D &operator=(const BalanceEnvironment3D &) = delete;

  [[nodiscard]] const RLVectorSchema3D &actionSchema() const;
  [[nodiscard]] const RLVectorSchema3D &observationSchema() const;
  [[nodiscard]] std::uint32_t rewardSchemaVersion() const { return 1; }
  [[nodiscard]] float policyStepSeconds() const;
  [[nodiscard]] std::uint32_t actionRepeat() const;

  [[nodiscard]] BalanceResetResult3D reset(std::uint64_t seed);
  [[nodiscard]] BalanceStepResult3D step(std::span<const float> action);
  [[nodiscard]] RagdollState3D state() const;

private:
  BalanceEnvironment3D(RagdollProfile3D profile,
                       BalanceEnvironmentSettings3D settings,
                       std::shared_ptr<PhysicsEngine3D> sharedEngine);

  struct Impl;
  std::unique_ptr<Impl> m_impl;

  friend class BalanceEnvironmentBatch3D;
};

} // namespace MatterEngine
