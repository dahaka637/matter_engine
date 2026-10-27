// A marcha pelo DCM (G3): as pegadas alternam os lados com a largura certa, a
// referencia do DCM e continua entre os passos e segue a dinamica xi' = omega (xi - p).
#include "Engine/BodyControl/Planning/DcmWalker3D.hpp"

#include <cmath>
#include <cstdio>
#include <string>

using namespace MatterEngine;

namespace {
int failures = 0;
void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::printf("FALHOU: %s\n", message.c_str());
    }
}
} // namespace

int main() {
    DcmWalkerSettings3D settings;
    settings.stepDuration = 0.5;
    settings.stepWidth = 0.2;
    settings.horizon = 4;
    const Eigen::Vector2d left(0.0, 1.0), velocity(1.0, 0.0);
    const DcmWalkerPlan3D plan = planDcmWalk3D(settings, Eigen::Vector2d(0.0, 0.1), +1, left, velocity);
    check(plan.footsteps.size() == 5, "horizonte de pegadas");
    check(std::abs(plan.footsteps[1].y() + 0.1) < 1e-9 && std::abs(plan.footsteps[1].x() - 0.5) < 1e-9, "a proxima pegada do outro lado, v T a frente");
    check(std::abs(plan.footsteps[2].y() - 0.1) < 1e-9 && std::abs(plan.footsteps[2].x() - 1.0) < 1e-9, "alterna os lados");
    // A dinamica: xi' = omega (xi - p) e o fim do passo bate com o planejado.
    const double omega = std::sqrt(settings.gravity / settings.comHeight);
    const DcmReference3D start = sampleDcmWalk3D(settings, plan, 0.0);
    const DcmReference3D end = sampleDcmWalk3D(settings, plan, settings.stepDuration);
    check((end.position - plan.endOfStep.front()).norm() < 1e-9, "o fim do passo e o DCM planejado");
    check((start.rate - omega * (start.position - plan.footsteps.front())).norm() < 1e-9, "xi' = omega (xi - p)");
    // A continuidade: o fim de um passo e o comeco do proximo (com o pe seguinte).
    const DcmWalkerPlan3D next = planDcmWalk3D(settings, plan.footsteps[1], -1, left, velocity);
    const DcmReference3D nextStart = sampleDcmWalk3D(settings, next, 0.0);
    check((nextStart.position - end.position).norm() < 0.05, "continuidade entre passos (horizonte finito)");
    // O ajuste do pouso: sem erro, o pouso nominal; DCM adiantado, pouso adiantado e^(omega T) vezes.
    const DcmLandingLimits3D limits;
    const Eigen::Vector2d onPlan = adjustDcmLanding3D(settings, plan, limits, start.position, +1, left, settings.stepDuration);
    check((onPlan - plan.footsteps[1]).norm() < 1e-9, "sem erro, o pouso nominal");
    const Eigen::Vector2d ahead = adjustDcmLanding3D(settings, plan, limits, start.position + Eigen::Vector2d(0.01, 0.0), +1, left, settings.stepDuration);
    check(std::abs(ahead.x() - plan.footsteps[1].x() - 0.01 * std::exp(omega * settings.stepDuration)) < 1e-9, "DCM adiantado, pouso adiantado");
    const Eigen::Vector2d crossing = adjustDcmLanding3D(settings, plan, limits, start.position + Eigen::Vector2d(0.0, 0.05), +1, left, settings.stepDuration);
    check(crossing.y() <= 0.1 - limits.minimumWidth + 1e-9, "o pe nao cruza o de apoio");
    // De lado (para a esquerda, saindo o pe esquerdo): os pes nunca cruzam e a media e v.
    const DcmWalkerPlan3D sideways = planDcmWalk3D(settings, Eigen::Vector2d(0.0, -0.1), -1, left, Eigen::Vector2d(0.0, 0.4));
    bool apart = true;
    for (std::size_t i = 1; i < sideways.footsteps.size(); ++i) {
        const double gap = (sideways.footsteps[i].y() - sideways.footsteps[i - 1].y()) * (i % 2 ? 1.0 : -1.0);
        apart &= gap >= settings.stepWidth - 1e-9;
    }
    check(apart, "de lado, os pes nunca cruzam");
    check(std::abs(sideways.footsteps[4].y() - sideways.footsteps[0].y() - 4 * settings.stepDuration * 0.4) < 1e-9, "de lado, a media e v");
    std::printf(failures ? "marcha DCM: %d falha(s)\n" : "marcha DCM: ok\n", failures);
    return failures ? 1 : 0;
}
