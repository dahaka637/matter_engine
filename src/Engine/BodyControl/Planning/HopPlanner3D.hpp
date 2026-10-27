#pragma once

namespace MatterEngine {

// O apoio do salto e da corrida (G4 da locomocao fisica nova; notas 31-32 do vault):
// um voo de duracao t_v pede a decolagem com v0 = g t_v / 2. O apoio de duracao T leva
// a velocidade vertical do CoM da de entrada (o pouso medido; -v0 no ciclo) a +v0 com a
// forca de chao em meio seno, de zero a zero (o pe decola sem carga),
//   F(t) = Fp sin(pi t / T),   Fp = pi m (v0 - v_entrada + g T) / (2 T).
struct HopSettings3D {
    double flightTime = 0.1;
    double stanceTime = 0.25;
    double mass = 80.0;
    double gravity = 9.81;
};

struct HopSample3D {
    // A forca vertical de chao e a aceleracao, a velocidade e o deslocamento verticais
    // do CoM (desde o comeco do apoio) nesse instante.
    double force = 0.0;
    double acceleration = 0.0;
    double velocity = 0.0;
    double displacement = 0.0;
};

[[nodiscard]] double hopTakeoffSpeed3D(const HopSettings3D& settings);
// O pico da forca vertical do apoio (N), para a velocidade vertical de entrada dada.
[[nodiscard]] double hopPeakForce3D(const HopSettings3D& settings, double entrySpeed);
// t de 0 a T no apoio (fora disso, so a gravidade).
[[nodiscard]] HopSample3D sampleHopStance3D(const HopSettings3D& settings, double time, double entrySpeed);

} // namespace MatterEngine
