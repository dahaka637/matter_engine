#include "Engine/Audio/Waveform.hpp"

#include <cmath>

namespace MatterEngine {
namespace {

constexpr float TwoPi = 6.28318530718f;

} // namespace

float oscillatorSample(Waveform waveform, float phase01) {
    // Normaliza qualquer fase (inclusive negativa ou > 1) de volta pra
    // [0,1) - permite que quem chama acumule fase livremente (ex.:
    // tempo*frequencia por muitos segundos) sem se preocupar em envolver o
    // valor manualmente.
    const float phase = phase01 - std::floor(phase01);
    switch (waveform) {
    case Waveform::Sine:
        return std::sin(phase * TwoPi);
    case Waveform::Square:
        return phase < 0.5f ? 1.0f : -1.0f;
    case Waveform::Triangle:
        return phase < 0.5f
            ? (phase * 4.0f - 1.0f)
            : (3.0f - phase * 4.0f);
    case Waveform::Sawtooth:
        return phase * 2.0f - 1.0f;
    }
    return 0.0f;
}

} // namespace MatterEngine
