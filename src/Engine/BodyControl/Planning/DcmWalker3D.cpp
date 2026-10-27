#include "Engine/BodyControl/Planning/DcmWalker3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

double omegaOf(const DcmWalkerSettings3D& settings) {
    return std::sqrt(settings.gravity / std::max(0.3, settings.comHeight));
}

} // namespace

DcmWalkerPlan3D planDcmWalk3D(const DcmWalkerSettings3D& settings, const Eigen::Vector2d& stance, int stanceSide,
    const Eigen::Vector2d& left, const Eigen::Vector2d& velocity) {
    DcmWalkerPlan3D plan;
    const double half = 0.5 * settings.stepWidth;
    // O centro do caminho passa entre os pes; cada passo anda a parte de frente de v T.
    // De lado, o pe que lidera abre 2 T v e o outro so fecha (passos iguais cruzariam as pernas).
    const double lateral = velocity.dot(left);
    const Eigen::Vector2d forward = velocity - lateral * left;
    Eigen::Vector2d center = stance - stanceSide * half * left;
    plan.footsteps.push_back(stance);
    int side = stanceSide;
    for (int i = 1; i <= std::max(1, settings.horizon); ++i) {
        side = -side;
        center += settings.stepDuration * forward;
        if (side * lateral > 0.0) center += 2.0 * settings.stepDuration * lateral * left;
        plan.footsteps.push_back(center + side * half * left);
    }
    if (forward.norm() > 1e-9) plan.copShift = settings.copTravel * std::min(1.0, forward.norm()) * forward.normalized();
    // De tras para frente: o ultimo pe e a parada (o DCM termina sobre ele, o CoP parado);
    // o DCM no fim do passo i e o do comeco do passo i + 1.
    const double decay = std::exp(-omegaOf(settings) * settings.stepDuration);
    plan.endOfStep.assign(plan.footsteps.size(), Eigen::Vector2d::Zero());
    plan.endOfStep.back() = plan.footsteps.back();
    for (int i = static_cast<int>(plan.footsteps.size()) - 2; i >= 0; --i) {
        const std::size_t next = static_cast<std::size_t>(i + 1);
        const Eigen::Vector2d shift = next + 1 == plan.footsteps.size() ? Eigen::Vector2d::Zero() : plan.copShift;
        const Eigen::Vector2d lead = shift / (omegaOf(settings) * settings.stepDuration);
        const Eigen::Vector2d heel = plan.footsteps[next] - 0.5 * shift, toe = plan.footsteps[next] + 0.5 * shift;
        plan.endOfStep[static_cast<std::size_t>(i)] = heel + lead + decay * (plan.endOfStep[next] - toe - lead);
    }
    return plan;
}

DcmReference3D sampleDcmWalk3D(const DcmWalkerSettings3D& settings, const DcmWalkerPlan3D& plan, double time) {
    DcmReference3D reference;
    if (plan.footsteps.empty()) return reference;
    const double omega = omegaOf(settings);
    const double t = std::clamp(time, 0.0, settings.stepDuration);
    const Eigen::Vector2d shift = plan.footsteps.size() > 1 ? plan.copShift : Eigen::Vector2d::Zero();
    const Eigen::Vector2d lead = shift / (omega * settings.stepDuration);
    const Eigen::Vector2d toe = plan.footsteps.front() + 0.5 * shift;
    reference.cop = plan.footsteps.front() - 0.5 * shift + (t / settings.stepDuration) * shift;
    reference.position = reference.cop + lead + std::exp(omega * (t - settings.stepDuration)) * (plan.endOfStep.front() - toe - lead);
    reference.rate = omega * (reference.position - reference.cop);
    return reference;
}

Eigen::Vector2d adjustDcmLanding3D(const DcmWalkerSettings3D& settings, const DcmWalkerPlan3D& plan,
    const DcmLandingLimits3D& limits, const Eigen::Vector2d& dcm, int stanceSide, const Eigen::Vector2d& left, double timeLeft) {
    if (plan.footsteps.size() < 2) return dcm;
    const Eigen::Vector2d& stance = plan.footsteps[0];
    // O DCM previsto no toque com o CoP do plano andando no resto do apoio.
    const double omega = omegaOf(settings), remaining = std::clamp(timeLeft, 0.0, settings.stepDuration);
    const Eigen::Vector2d lead = plan.copShift / (omega * settings.stepDuration);
    const Eigen::Vector2d copNow = stance - 0.5 * plan.copShift + (1.0 - remaining / settings.stepDuration) * plan.copShift;
    const Eigen::Vector2d predicted = stance + 0.5 * plan.copShift + lead + std::exp(omega * remaining) * (dcm - copNow - lead);
    Eigen::Vector2d adjustment = predicted - plan.endOfStep[0];
    if (adjustment.norm() > limits.maximumAdjustment) adjustment *= limits.maximumAdjustment / adjustment.norm();
    Eigen::Vector2d landing = plan.footsteps[1] + adjustment;
    // O pe que pousa fica do lado dele: pelo menos a largura minima do de apoio.
    const double side = -stanceSide * (landing - stance).dot(left);
    if (side < limits.minimumWidth) landing += -stanceSide * (limits.minimumWidth - side) * left;
    return landing;
}

} // namespace MatterEngine
