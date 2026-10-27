// O apoio do salto (G4): a forca vai de zero a zero, leva a velocidade da de entrada a
// +v0, a forma fechada bate com a integracao e, no ciclo (-v0 a +v0), a altura volta.
#include "Engine/BodyControl/Planning/HopPlanner3D.hpp"

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
    HopSettings3D settings;
    settings.flightTime = 0.12;
    settings.stanceTime = 0.2;
    const double v0 = hopTakeoffSpeed3D(settings), T = settings.stanceTime;
    check(std::abs(v0 - 0.5 * 9.81 * 0.12) < 1e-12, "v0 = g t_v / 2");
    for (const double entry : { -v0, 0.0 }) {
        double velocity = entry, height = 0.0;
        const int steps = 20000;
        for (int i = 0; i < steps; ++i) {
            const double a = sampleHopStance3D(settings, (i + 0.5) * T / steps, entry).acceleration;
            height += (velocity + 0.5 * a * T / steps) * T / steps;
            velocity += a * T / steps;
        }
        const HopSample3D end = sampleHopStance3D(settings, T, entry);
        check(std::abs(velocity - v0) < 1e-6 && std::abs(end.velocity - v0) < 1e-9, "o apoio leva a velocidade a +v0");
        check(std::abs(height - end.displacement) < 1e-6, "a forma fechada bate com a integracao");
        check(std::abs(sampleHopStance3D(settings, 0.0, entry).force) < 1e-9 && std::abs(end.force) < 1e-6, "de zero a zero");
    }
    check(std::abs(sampleHopStance3D(settings, T, -v0).displacement) < 1e-9, "no ciclo a altura volta");
    check(std::abs(sampleHopStance3D(settings, 0.5 * T, -v0).force - hopPeakForce3D(settings, -v0)) < 1e-9, "pico no meio");
    // O empurrao de perna estendida: chega a velocidade de saida em T, a forma fechada bate.
    {
        double velocity = -0.1, height = 0.0;
        const int steps = 20000;
        for (int i = 0; i < steps; ++i) {
            const double a = sampleHopPush3D(settings, (i + 0.5) * T / steps, -0.1, 0.8).acceleration;
            height += (velocity + 0.5 * a * T / steps) * T / steps;
            velocity += a * T / steps;
        }
        const HopSample3D end = sampleHopPush3D(settings, T, -0.1, 0.8);
        check(std::abs(velocity - 0.8) < 1e-6 && std::abs(end.velocity - 0.8) < 1e-12, "empurrao: chega a saida");
        check(std::abs(height - end.displacement) < 1e-6, "empurrao: a forma fechada bate");
        check(std::abs(sampleHopPush3D(settings, 0.0, -0.1, 0.8).force - settings.mass * settings.gravity) < 1e-9, "empurrao: comeca no peso");
    }
    std::printf(failures ? "salto: %d falha(s)\n" : "salto: ok\n", failures);
    return failures ? 1 : 0;
}
