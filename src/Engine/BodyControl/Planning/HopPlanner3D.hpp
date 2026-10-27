#pragma once

namespace MatterEngine {

// O apoio do salto e da corrida (G4 da locomocao fisica nova; notas 31-32 do vault):
// um voo de duracao t_v pede a decolagem com v0 = g t_v / 2 (e o pouso com -v0); o
// apoio de duracao T inverte a velocidade com a forca vertical em meio seno,
//   F(t) = m g + A sin(pi t / T),   A = pi m (2 v0 + g T) / (2 T) - m g,
// de modo que o impulso total e m (2 v0 + g T) e o CoM sai do apoio com +v0.
struct HopSettings3D {
    double flightTime = 0.1;
    double stanceTime = 0.25;
    double mass = 80.0;
    double gravity = 9.81;
};

struct HopSample3D {
    // A forca vertical de chao e a aceleracao vertical do CoM nesse instante do apoio.
    double force = 0.0;
    double acceleration = 0.0;
};

[[nodiscard]] double hopTakeoffSpeed3D(const HopSettings3D& settings);
// O pico da forca vertical do apoio (N).
[[nodiscard]] double hopPeakForce3D(const HopSettings3D& settings);
// t de 0 a T (fora disso, so a gravidade: o voo).
[[nodiscard]] HopSample3D sampleHopStance3D(const HopSettings3D& settings, double time);

} // namespace MatterEngine
