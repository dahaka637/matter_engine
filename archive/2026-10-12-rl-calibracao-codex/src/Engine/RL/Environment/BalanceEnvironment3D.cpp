#include "Engine/RL/Environment/BalanceEnvironment3D.hpp"

#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace MatterEngine {
namespace {

constexpr std::uint32_t InvalidLink = std::numeric_limits<std::uint32_t>::max();

struct DegreeOfFreedom {
  std::uint32_t linkIndex = 0;
  RagdollAxis3D axis = RagdollAxis3D::Twist;
};

float clampUnit(float value) { return std::clamp(value, -1.0f, 1.0f); }

bool finite(float value) { return std::isfinite(value); }

bool finite(Vec3 value) {
  return finite(value.x) && finite(value.y) && finite(value.z);
}

bool finite(Quaternion value) {
  return finite(value.x) && finite(value.y) && finite(value.z) &&
         finite(value.w);
}

const char *axisName(RagdollAxis3D axis) {
  switch (axis) {
  case RagdollAxis3D::Twist:
    return "twist";
  case RagdollAxis3D::Swing1:
    return "swing1";
  case RagdollAxis3D::Swing2:
    return "swing2";
  }
  return "unknown";
}

void appendVectorChannels(RLVectorSchema3D &schema, std::string_view prefix,
                          std::string_view unit, float minimum = -1.0f,
                          float maximum = 1.0f) {
  schema.channels.push_back(
      {std::string(prefix) + ".x", std::string(unit), minimum, maximum});
  schema.channels.push_back(
      {std::string(prefix) + ".y", std::string(unit), minimum, maximum});
  schema.channels.push_back(
      {std::string(prefix) + ".z", std::string(unit), minimum, maximum});
}

PhysicsBodyHandle3D createGround(PhysicsScene3D &scene) {
  PhysicsShape3D shape;
  shape.type = PhysicsShapeType3D::Box;
  shape.halfExtents = {20.0f, 20.0f, 0.25f};
  shape.materialId = "concrete";

  PhysicsBodyDefinition3D body;
  body.motionType = PhysicsMotionType3D::Static;
  body.position = {0.0f, 0.0f, -0.25f};
  body.materialId = "concrete";
  return scene.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
}

float normalizedJointPosition(float position,
                              const RagdollAxisDefinition3D &axis) {
  const float range = axis.maximumRadians - axis.minimumRadians;
  if (range <= 0.0f)
    return 0.0f;
  return clampUnit(2.0f * (position - axis.minimumRadians) / range - 1.0f);
}

float actionTarget(float action, const RagdollAxisDefinition3D &axis) {
  action = clampUnit(action);
  // Zero é a pose neutra de coordenadas exponenciais. Cada metade do
  // intervalo normalizado alcança independentemente o limite do seu lado.
  return action >= 0.0f ? action * axis.maximumRadians
                        : (-action) * axis.minimumRadians;
}

} // namespace

struct BalanceEnvironment3D::Impl {
  explicit Impl(RagdollProfile3D sourceProfile,
                BalanceEnvironmentSettings3D sourceSettings,
                std::shared_ptr<PhysicsEngine3D> sharedEngine)
      : profile(std::move(sourceProfile)), settings(sourceSettings),
        engine(std::move(sharedEngine)) {
    if (!engine)
      throw std::invalid_argument(
          "O ambiente de equilibrio exige um PhysicsEngine3D");
    validateRagdollProfileOrThrow3D(profile);
    validateSettings();
    buildSchemas();
  }

  void validateSettings() {
    const auto positive = [](float value) {
      return finite(value) && value > 0.0f;
    };
    if (settings.actionRepeat == 0 || settings.maximumEpisodePolicySteps == 0 ||
        !finite(settings.groundClearanceMeters) ||
        settings.groundClearanceMeters < 0.0f ||
        !positive(settings.heightErrorScaleMeters) ||
        !positive(settings.linearVelocityScaleMetersPerSecond) ||
        !positive(settings.angularVelocityScaleRadiansPerSecond) ||
        !positive(settings.jointVelocityScaleRadiansPerSecond) ||
        !positive(settings.centerOfMassOffsetScaleMeters) ||
        !positive(settings.footOffsetScaleMeters) ||
        !finite(settings.driveStiffnessScale) ||
        settings.driveStiffnessScale < 0.0f ||
        !finite(settings.driveDampingScale) ||
        settings.driveDampingScale < 0.0f ||
        !finite(settings.driveMaximumTorqueScale) ||
        settings.driveMaximumTorqueScale < 0.0f ||
        !finite(settings.terminationMinimumPelvisHeightFraction) ||
        settings.terminationMinimumPelvisHeightFraction <= 0.0f ||
        settings.terminationMinimumPelvisHeightFraction > 1.0f ||
        !finite(settings.terminationMinimumPelvisUp) ||
        settings.terminationMinimumPelvisUp < -1.0f ||
        settings.terminationMinimumPelvisUp >= 1.0f) {
      throw std::invalid_argument(
          "Configuracao invalida do ambiente de equilibrio");
    }
  }

  void buildSchemas() {
    actionSchema.id = "matter.balance.action";
    actionSchema.version = 1;
    for (std::size_t linkIndex = 1; linkIndex < profile.links.size();
         ++linkIndex) {
      const RagdollLinkDefinition3D &link = profile.links[linkIndex];
      if (link.id == "LeftFoot")
        leftFoot = static_cast<std::uint32_t>(linkIndex);
      if (link.id == "RightFoot")
        rightFoot = static_cast<std::uint32_t>(linkIndex);
      for (std::size_t axisIndex = 0; axisIndex < link.inboundJoint.axes.size();
           ++axisIndex) {
        const RagdollAxisDefinition3D &axis = link.inboundJoint.axes[axisIndex];
        if (!axis.enabled)
          continue;
        if (axis.minimumRadians > 0.0f || axis.maximumRadians < 0.0f) {
          throw std::invalid_argument(
              "O ambiente balance.v1 exige pose neutra zero "
              "dentro de todos os limites ativos");
        }
        const auto axisId = static_cast<RagdollAxis3D>(axisIndex);
        dofs.push_back({static_cast<std::uint32_t>(linkIndex), axisId});
        actionSchema.channels.push_back(
            {"joints." + link.id + "." + axisName(axisId) + ".target",
             "normalized", -1.0f, 1.0f});
      }
    }
    if (leftFoot == InvalidLink || rightFoot == InvalidLink)
      throw std::invalid_argument(
          "O ambiente de equilibrio exige LeftFoot e RightFoot");

    observationSchema.id = "matter.balance.observation";
    observationSchema.version = 1;
    observationSchema.channels.push_back(
        {"pelvis.height_error", "normalized", -1.0f, 1.0f});
    appendVectorChannels(observationSchema, "pelvis.gravity_local",
                         "unit_vector");
    appendVectorChannels(observationSchema, "pelvis.linear_velocity_local",
                         "normalized");
    appendVectorChannels(observationSchema, "pelvis.angular_velocity_local",
                         "normalized");
    appendVectorChannels(observationSchema, "body.com_offset_local",
                         "normalized");
    appendVectorChannels(observationSchema, "feet.left.position_local",
                         "normalized");
    appendVectorChannels(observationSchema, "feet.right.position_local",
                         "normalized");
    observationSchema.channels.push_back(
        {"feet.left.support", "boolean", 0.0f, 1.0f});
    observationSchema.channels.push_back(
        {"feet.right.support", "boolean", 0.0f, 1.0f});
    for (const DegreeOfFreedom &dof : dofs) {
      const std::string base = "joints." + profile.links[dof.linkIndex].id +
                               "." + axisName(dof.axis);
      observationSchema.channels.push_back(
          {base + ".position", "normalized", -1.0f, 1.0f});
    }
    for (const DegreeOfFreedom &dof : dofs) {
      const std::string base = "joints." + profile.links[dof.linkIndex].id +
                               "." + axisName(dof.axis);
      observationSchema.channels.push_back(
          {base + ".velocity", "normalized", -1.0f, 1.0f});
    }
    for (const DegreeOfFreedom &dof : dofs) {
      const std::string base = "joints." + profile.links[dof.linkIndex].id +
                               "." + axisName(dof.axis);
      observationSchema.channels.push_back(
          {base + ".previous_action", "normalized", -1.0f, 1.0f});
    }
    previousAction.assign(dofs.size(), 0.0f);
  }

  void createScene() {
    PhysicsSceneSettings3D sceneSettings;
    sceneSettings.gravity = {0.0f, 0.0f, -9.81f};
    sceneSettings.workerThreadCount = 1;
    scene = engine->createScene(sceneSettings, materials);
    (void)createGround(*scene);

    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.pelvisPosition = {0.0f, 0.0f,
                            profile.standingRootHeightMeters +
                                settings.groundClearanceMeters};
    spawn.rigidityPercent = 0.0f;
    spawn.active = true;
    ragdoll = scene->createRagdoll(profile, spawn);
  }

  [[nodiscard]] std::vector<float> observe(const RagdollState3D &body) const {
    if (body.links.size() != profile.links.size() ||
        body.joints.size() != profile.links.size()) {
      throw std::runtime_error(
          "Snapshot incompleto do ragdoll no ambiente de equilibrio");
    }

    std::vector<float> values;
    values.reserve(observationSchema.size());
    const PhysicsBodyState3D &pelvis = body.links.front();
    const Quaternion pelvisFrame =
        (pelvis.orientation *
         profile.links.front().modelOrientation.conjugate())
            .normalized();
    const Quaternion worldToPelvis = pelvisFrame.conjugate();
    const float targetHeight =
        profile.standingRootHeightMeters + settings.groundClearanceMeters;
    values.push_back(clampUnit((pelvis.position.z - targetHeight) /
                               settings.heightErrorScaleMeters));

    const auto appendVec = [&values](Vec3 value) {
      values.push_back(clampUnit(value.x));
      values.push_back(clampUnit(value.y));
      values.push_back(clampUnit(value.z));
    };
    appendVec(worldToPelvis.rotate({0.0f, 0.0f, -1.0f}));
    appendVec(worldToPelvis.rotate(pelvis.linearVelocity) /
              settings.linearVelocityScaleMetersPerSecond);
    appendVec(worldToPelvis.rotate(pelvis.angularVelocity) /
              settings.angularVelocityScaleRadiansPerSecond);

    Vec3 centerOfMass;
    float massFractionSum = 0.0f;
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
      const RagdollLinkDefinition3D &definition = profile.links[index];
      const PhysicsBodyState3D &state = body.links[index];
      centerOfMass += (state.position +
                       state.orientation.rotate(definition.centerOfMassLocal)) *
                      definition.massFraction;
      massFractionSum += definition.massFraction;
    }
    centerOfMass = centerOfMass / massFractionSum;
    appendVec(worldToPelvis.rotate(centerOfMass - pelvis.position) /
              settings.centerOfMassOffsetScaleMeters);
    appendVec(
        worldToPelvis.rotate(body.links[leftFoot].position - pelvis.position) /
        settings.footOffsetScaleMeters);
    appendVec(
        worldToPelvis.rotate(body.links[rightFoot].position - pelvis.position) /
        settings.footOffsetScaleMeters);

    bool leftSupport = false;
    bool rightSupport = false;
    for (const RagdollContactPoint3D &contact : body.contacts) {
      if (contact.otherMotion != RagdollContactMotion3D::Static ||
          contact.normal.z < 0.6f)
        continue;
      leftSupport = leftSupport || contact.linkIndex == leftFoot;
      rightSupport = rightSupport || contact.linkIndex == rightFoot;
    }
    values.push_back(leftSupport ? 1.0f : 0.0f);
    values.push_back(rightSupport ? 1.0f : 0.0f);

    for (const DegreeOfFreedom &dof : dofs) {
      const std::size_t axis = static_cast<std::size_t>(dof.axis);
      values.push_back(normalizedJointPosition(
          body.joints[dof.linkIndex].positionRadians[axis],
          profile.links[dof.linkIndex].inboundJoint.axes[axis]));
    }
    for (const DegreeOfFreedom &dof : dofs) {
      const std::size_t axis = static_cast<std::size_t>(dof.axis);
      values.push_back(
          clampUnit(body.joints[dof.linkIndex].velocityRadiansPerSecond[axis] /
                    settings.jointVelocityScaleRadiansPerSecond));
    }
    values.insert(values.end(), previousAction.begin(), previousAction.end());
    if (values.size() != observationSchema.size())
      throw std::runtime_error(
          "Layout da observacao divergiu do schema balance.v1");
    return values;
  }

  [[nodiscard]] bool finiteState(const RagdollState3D &body) const {
    if (body.links.size() != profile.links.size() ||
        body.joints.size() != profile.links.size())
      return false;
    for (const PhysicsBodyState3D &link : body.links) {
      if (!finite(link.position) || !finite(link.orientation) ||
          !finite(link.linearVelocity) || !finite(link.angularVelocity))
        return false;
    }
    for (const RagdollJointState3D &joint : body.joints) {
      for (std::size_t axis = 0; axis < joint.positionRadians.size(); ++axis) {
        if (!finite(joint.positionRadians[axis]) ||
            !finite(joint.velocityRadiansPerSecond[axis]))
          return false;
      }
    }
    return true;
  }

  [[nodiscard]] BalanceReward3D reward(const RagdollState3D &body) const {
    BalanceReward3D result;
    const PhysicsBodyState3D &pelvis = body.links.front();
    const Quaternion pelvisFrame =
        (pelvis.orientation *
         profile.links.front().modelOrientation.conjugate())
            .normalized();
    const float pelvisUp = pelvisFrame.rotate({0.0f, 0.0f, 1.0f}).z;
    result.upright =
        std::clamp((pelvisUp - settings.terminationMinimumPelvisUp) /
                       (1.0f - settings.terminationMinimumPelvisUp),
                   0.0f, 1.0f);
    const float targetHeight =
        profile.standingRootHeightMeters + settings.groundClearanceMeters;
    const float heightError =
        (pelvis.position.z - targetHeight) / settings.heightErrorScaleMeters;
    result.height = std::exp(-4.0f * heightError * heightError);
    result.stillness = std::exp(-0.20f * pelvis.linearVelocity.lengthSquared() -
                                0.04f * pelvis.angularVelocity.lengthSquared());
    float squaredAction = 0.0f;
    for (const float action : previousAction)
      squaredAction += action * action;
    result.actionPenalty =
        previousAction.empty()
            ? 0.0f
            : squaredAction / static_cast<float>(previousAction.size());
    result.total = 0.45f * result.upright + 0.30f * result.height +
                   0.25f * result.stillness - 0.05f * result.actionPenalty;
    return result;
  }

  [[nodiscard]] std::string
  terminationReason(const RagdollState3D &body) const {
    if (!finiteState(body))
      return "non_finite_state";
    const PhysicsBodyState3D &pelvis = body.links.front();
    if (pelvis.position.z < profile.standingRootHeightMeters *
                                settings.terminationMinimumPelvisHeightFraction)
      return "pelvis_below_minimum";
    const Quaternion pelvisFrame =
        (pelvis.orientation *
         profile.links.front().modelOrientation.conjugate())
            .normalized();
    if (pelvisFrame.rotate({0.0f, 0.0f, 1.0f}).z <
        settings.terminationMinimumPelvisUp)
      return "pelvis_not_upright";
    return {};
  }

  RagdollProfile3D profile;
  BalanceEnvironmentSettings3D settings;
  std::shared_ptr<PhysicsEngine3D> engine;
  MaterialLibrary materials;
  std::unique_ptr<PhysicsScene3D> scene;
  RagdollHandle3D ragdoll;
  RLVectorSchema3D actionSchema;
  RLVectorSchema3D observationSchema;
  std::vector<DegreeOfFreedom> dofs;
  std::vector<float> previousAction;
  std::uint32_t leftFoot = InvalidLink;
  std::uint32_t rightFoot = InvalidLink;
  std::uint64_t seed = 0;
  std::uint32_t episodePolicyStep = 0;
  std::uint64_t episodePhysicsStep = 0;
  bool episodeDone = true;
};

BalanceEnvironment3D::BalanceEnvironment3D(
    RagdollProfile3D profile, BalanceEnvironmentSettings3D settings)
    : BalanceEnvironment3D(
          std::move(profile), settings,
          std::make_shared<PhysicsEngine3D>(
              std::make_shared<TaskScheduler>(TaskSchedulerSettings{1}))) {}

BalanceEnvironment3D::BalanceEnvironment3D(
    RagdollProfile3D profile, BalanceEnvironmentSettings3D settings,
    std::shared_ptr<PhysicsEngine3D> sharedEngine)
    : m_impl(std::make_unique<Impl>(std::move(profile), settings,
                                    std::move(sharedEngine))) {}

BalanceEnvironment3D::~BalanceEnvironment3D() = default;

const RLVectorSchema3D &BalanceEnvironment3D::actionSchema() const {
  return m_impl->actionSchema;
}

const RLVectorSchema3D &BalanceEnvironment3D::observationSchema() const {
  return m_impl->observationSchema;
}

float BalanceEnvironment3D::policyStepSeconds() const {
  return static_cast<float>(m_impl->settings.actionRepeat) *
         ReinforcementLearningFixedStepSeconds;
}

std::uint32_t BalanceEnvironment3D::actionRepeat() const {
  return m_impl->settings.actionRepeat;
}

BalanceResetResult3D BalanceEnvironment3D::reset(std::uint64_t seed) {
  m_impl->scene.reset();
  m_impl->seed = seed;
  m_impl->episodePolicyStep = 0;
  m_impl->episodePhysicsStep = 0;
  m_impl->episodeDone = false;
  std::fill(m_impl->previousAction.begin(), m_impl->previousAction.end(), 0.0f);
  m_impl->createScene();
  const RagdollState3D body = m_impl->scene->ragdollState(m_impl->ragdoll);
  return {m_impl->observe(body), seed};
}

BalanceStepResult3D BalanceEnvironment3D::step(std::span<const float> action) {
  if (!m_impl->scene || m_impl->episodeDone)
    throw std::logic_error(
        "reset() e obrigatorio antes de step(), inclusive apos o fim");
  if (action.size() != m_impl->dofs.size())
    throw std::invalid_argument(
        "Dimensao de action incompatível com matter.balance.action.v1");

  for (const float value : action) {
    if (!finite(value))
      throw std::invalid_argument("Action contem valor nao finito");
  }

  std::vector<RagdollDriveTarget3D> targets;
  targets.reserve(m_impl->dofs.size());
  bool clipped = false;
  for (std::size_t index = 0; index < action.size(); ++index) {
    const float normalized = clampUnit(action[index]);
    clipped = clipped || normalized != action[index];
    m_impl->previousAction[index] = normalized;
    const DegreeOfFreedom &dof = m_impl->dofs[index];
    const std::size_t axisIndex = static_cast<std::size_t>(dof.axis);
    RagdollDriveTarget3D target;
    target.linkIndex = dof.linkIndex;
    target.axis = dof.axis;
    target.positionRadians = actionTarget(
        normalized,
        m_impl->profile.links[dof.linkIndex].inboundJoint.axes[axisIndex]);
    target.stiffnessScale = m_impl->settings.driveStiffnessScale;
    target.dampingScale = m_impl->settings.driveDampingScale;
    target.maximumTorqueScale = m_impl->settings.driveMaximumTorqueScale;
    target.gravityCompensationScale = 1.0f;
    targets.push_back(target);
  }

  for (std::uint32_t substep = 0; substep < m_impl->settings.actionRepeat;
       ++substep) {
    m_impl->scene->setRagdollActiveDriveTargets(
        m_impl->ragdoll, targets, m_impl->settings.gravityCompensationEnabled);
    m_impl->scene->simulate(ReinforcementLearningFixedStepSeconds);
    ++m_impl->episodePhysicsStep;
  }
  ++m_impl->episodePolicyStep;

  const RagdollState3D body = m_impl->scene->ragdollState(m_impl->ragdoll);
  BalanceStepResult3D result;
  result.episodePolicyStep = m_impl->episodePolicyStep;
  result.episodePhysicsStep = m_impl->episodePhysicsStep;
  result.actionClipped = clipped;
  result.terminationReason = m_impl->terminationReason(body);
  result.terminated = !result.terminationReason.empty();
  if (result.terminationReason == "non_finite_state") {
    result.observation.assign(m_impl->observationSchema.size(), 0.0f);
    result.reward.total = -1.0f;
  } else {
    result.observation = m_impl->observe(body);
    result.reward = m_impl->reward(body);
  }
  result.truncated =
      !result.terminated &&
      m_impl->episodePolicyStep >= m_impl->settings.maximumEpisodePolicySteps;
  if (result.truncated)
    result.terminationReason = "time_limit";
  m_impl->episodeDone = result.terminated || result.truncated;
  return result;
}

RagdollState3D BalanceEnvironment3D::state() const {
  if (!m_impl->scene)
    return {};
  return m_impl->scene->ragdollState(m_impl->ragdoll);
}

} // namespace MatterEngine
