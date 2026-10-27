// O apoio do salto (G4): o impulso inverte -v0 em +v0, o pico e (pi/2) vezes a media
// acima do peso e, fora do apoio, so a gravidade.
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
    const double v0 = hopTakeoffSpeed3D(settings);
    check(std::abs(v0 - 0.5 * 9.81 * 0.12) < 1e-12, "v0 = g t_v / 2");
    // Integra a aceleracao do apoio: a velocidade vai de -v0 a +v0.
    double velocity = -v0;
    const int steps = 20000;
    for (int i = 0; i < steps; ++i) velocity += sampleHopStance3D(settings, (i + 0.5) * settings.stanceTime / steps).acceleration * settings.stanceTime / steps;
    check(std::abs(velocity - v0) < 1e-6, "o apoio inverte a velocidade");
    check(std::abs(sampleHopStance3D(settings, 0.0).force - settings.mass * settings.gravity) < 1e-9, "comeca no peso");
    check(std::abs(sampleHopStance3D(settings, 0.5 * settings.stanceTime).force - hopPeakForce3D(settings)) < 1e-9, "pico no meio");
    check(std::abs(sampleHopStance3D(settings, 0.3).acceleration + 9.81) < 1e-12, "no voo, so a gravidade");
    // Sem voo e apoio continuo: o pico e o peso (ficar parado).
    settings.flightTime = 0.0;
    check(std::abs(hopPeakForce3D(settings) - settings.mass * settings.gravity) < 1e-9, "sem voo, o peso");
    std::printf(failures ? "salto: %d falha(s)\n" : "salto: ok\n", failures);
    return failures ? 1 : 0;
}
