#include "Engine/Audio/WindAmbienceRamp.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

float windAmbienceVolumeRamp(float speed, float quietThreshold,
    float referenceSpeed, float curveExponent) {
    const float denominator = referenceSpeed - quietThreshold;
    const float linearRamp = denominator > 1.0e-5f
        ? std::clamp((speed - quietThreshold) / denominator, 0.0f, 1.0f)
        : (speed > quietThreshold ? 1.0f : 0.0f);

    // Expoentes 2 e 3 (os dois usos reais, ver WindAmbienceRamp.hpp) por
    // multiplicacao direta em vez de std::pow: sem chamada transcendental
    // por voz por quadro, e mantem o uivo ambiente (curveExponent=2)
    // bit-a-bit identico ao calculo antigo (ramp*ramp) - comparar float por
    // igualdade e seguro aqui porque curveExponent so chega como literal
    // constexpr nos dois pontos de chamada existentes.
    if (curveExponent == 2.0f) return linearRamp * linearRamp;
    if (curveExponent == 3.0f) return linearRamp * linearRamp * linearRamp;
    return std::pow(linearRamp, curveExponent);
}

} // namespace MatterEngine
