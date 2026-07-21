#include "Engine/Audio/BiquadFilter.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {
constexpr float Pi = 3.14159265358979323846f;
} // namespace

BiquadCoefficients biquadCoefficients(BiquadFilterType type,
    float sampleRateHz, float cutoffHz, float q) {
    const float safeSampleRate = std::max(1.0f, sampleRateHz);
    // Perto/acima de Nyquist (metade da taxa de amostragem) a formula do
    // cookbook fica numericamente instavel - 0.49 deixa uma margem segura
    // sem restringir de forma perceptivel nenhum uso pratico de audio.
    const float clampedCutoff = std::clamp(cutoffHz, 10.0f,
        safeSampleRate * 0.49f);
    const float safeQ = std::max(0.05f, q);

    const float omega = 2.0f * Pi * clampedCutoff / safeSampleRate;
    const float sinOmega = std::sin(omega);
    const float cosOmega = std::cos(omega);
    const float alpha = sinOmega / (2.0f * safeQ);

    float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a0 = 1.0f, a1 = 0.0f, a2 = 0.0f;
    switch (type) {
    case BiquadFilterType::LowPass:
        b0 = (1.0f - cosOmega) * 0.5f;
        b1 = 1.0f - cosOmega;
        b2 = (1.0f - cosOmega) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosOmega;
        a2 = 1.0f - alpha;
        break;
    case BiquadFilterType::HighPass:
        b0 = (1.0f + cosOmega) * 0.5f;
        b1 = -(1.0f + cosOmega);
        b2 = (1.0f + cosOmega) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosOmega;
        a2 = 1.0f - alpha;
        break;
    case BiquadFilterType::BandPass:
        b0 = alpha;
        b1 = 0.0f;
        b2 = -alpha;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosOmega;
        a2 = 1.0f - alpha;
        break;
    case BiquadFilterType::Notch:
        b0 = 1.0f;
        b1 = -2.0f * cosOmega;
        b2 = 1.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosOmega;
        a2 = 1.0f - alpha;
        break;
    }

    return BiquadCoefficients {
        b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0
    };
}

float BiquadState::process(float input, const BiquadCoefficients& c) {
    // Forma direta I (Direct Form I) - a mais simples e numericamente
    // robusta das formas equivalentes, adequada aqui porque os
    // coeficientes podem mudar a cada amostra (a Forma II Transposta
    // exigiria cuidado extra nesse caso).
    const float output = c.b0 * input + c.b1 * x1 + c.b2 * x2
        - c.a1 * y1 - c.a2 * y2;
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;
    return output;
}

} // namespace MatterEngine
