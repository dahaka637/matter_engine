#pragma once
#include "Engine/RL/Motor/MotorTraining3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/RL/Supervisor/TrainingHistory3D.hpp"
#include "Workbench/ReinforcementLearning/ViewerWorld3D.hpp"
#include <random>

namespace MatterEngine {
class MotorViewer3D final : public ViewerWorld3D {
  public:
    static bool isMotorTask(const std::filesystem::path &path);
    static TrainingHistory3D readHistory(const std::filesystem::path &directory, std::size_t maximum, double since);
    MotorViewer3D(const std::filesystem::path &assets, const std::filesystem::path &task, const std::string &drill,
                  std::uint64_t seed);
    void setPolicy(const std::filesystem::path &path, std::string &label) override;
    void clearPolicy() override { m_policy.reset(); }
    bool hasPolicy() const override { return bool(m_policy); }
    void step() override;
    float policyStepSeconds() const override { return MotorTraining3D::DecisionSeconds; }
    void reset(std::uint64_t seed) override;
    void setManualSpeed(std::optional<float>) override {}
    void push(Vec3, float) override {}
    void setLook(float radians);
    BodyWorldView3D view() const override;
    const RagdollProfile3D &profile() const override { return m_batch->profile(); }

  private:
    void readTrotLimits();
    std::unique_ptr<MotorTraining3D> m_batch;
    std::filesystem::path m_task;
    std::unique_ptr<PolicyRuntime3D> m_policy;
    std::vector<float> m_obs, m_action;
    std::uint32_t m_steps = 0;
    std::mt19937_64 m_random;
    std::normal_distribution<float> m_normal {0.f, 1.f};
};
} // namespace MatterEngine
