#pragma once

#include <algorithm>
#include <cmath>

namespace MatterEngine {

// As curvas dos gestos: tudo continuo na posicao, na velocidade e na
// aceleracao (o "duro e bugado" de antes vinha de quinas e saltos).

// O degrau suave de grau 5 (0 em 0, 1 em 1, velocidade e aceleracao zero nas
// pontas).
[[nodiscard]] inline float gestureSmoothStep5(float x) {
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * x * (10.0f + x * (-15.0f + 6.0f * x));
}

// O pulso suave: 0 ate start, sobe ate 1 em peak e volta a 0 em end (cada
// lado um degrau de grau 5).
[[nodiscard]] inline float gestureBump(float t, float start, float peak, float end) {
    if (t <= start || t >= end) return 0.0f;
    if (t <= peak) return gestureSmoothStep5((t - start) / std::max(1e-4f, peak - start));
    return 1.0f - gestureSmoothStep5((t - peak) / std::max(1e-4f, end - peak));
}

// O trecho quintico de Hermite entre dois estados (posicao, velocidade,
// aceleracao), em s de 0 a 1 num trecho de h segundos. velocity e
// acceleration saem com a derivada no tempo.
struct GestureQuintic {
    float p0 = 0.0f, v0 = 0.0f, a0 = 0.0f, p1 = 0.0f, v1 = 0.0f, a1 = 0.0f, h = 1.0f;
    [[nodiscard]] float position(float s) const {
        s = std::clamp(s, 0.0f, 1.0f);
        const float s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
        const float h0 = 1.0f - 10.0f * s3 + 15.0f * s4 - 6.0f * s5;
        const float h1 = s - 6.0f * s3 + 8.0f * s4 - 3.0f * s5;
        const float h2 = 0.5f * s2 - 1.5f * s3 + 1.5f * s4 - 0.5f * s5;
        const float h3 = 0.5f * s3 - s4 + 0.5f * s5;
        const float h4 = -4.0f * s3 + 7.0f * s4 - 3.0f * s5;
        const float h5 = 10.0f * s3 - 15.0f * s4 + 6.0f * s5;
        return h0 * p0 + h1 * h * v0 + h2 * h * h * a0 + h3 * h * h * a1 + h4 * h * v1 + h5 * p1;
    }
    [[nodiscard]] float velocity(float s) const {
        s = std::clamp(s, 0.0f, 1.0f);
        const float s2 = s * s, s3 = s2 * s, s4 = s3 * s;
        const float d0 = -30.0f * s2 + 60.0f * s3 - 30.0f * s4;
        const float d1 = 1.0f - 18.0f * s2 + 32.0f * s3 - 15.0f * s4;
        const float d2 = s - 4.5f * s2 + 6.0f * s3 - 2.5f * s4;
        const float d3 = 1.5f * s2 - 4.0f * s3 + 2.5f * s4;
        const float d4 = -12.0f * s2 + 28.0f * s3 - 15.0f * s4;
        const float d5 = 30.0f * s2 - 60.0f * s3 + 30.0f * s4;
        return (d0 * p0 + d1 * h * v0 + d2 * h * h * a0 + d3 * h * h * a1 + d4 * h * v1 + d5 * p1) / h;
    }
    [[nodiscard]] float acceleration(float s) const {
        s = std::clamp(s, 0.0f, 1.0f);
        const float s2 = s * s, s3 = s2 * s;
        const float e0 = -60.0f * s + 180.0f * s2 - 120.0f * s3;
        const float e1 = -36.0f * s + 96.0f * s2 - 60.0f * s3;
        const float e2 = 1.0f - 9.0f * s + 18.0f * s2 - 10.0f * s3;
        const float e3 = 3.0f * s - 12.0f * s2 + 10.0f * s3;
        const float e4 = -24.0f * s + 84.0f * s2 - 60.0f * s3;
        return (e0 * (p0 - p1) + e1 * h * v0 + e2 * h * h * a0 + e3 * h * h * a1 + e4 * h * v1) / (h * h);
    }
};

// A tangente no meio de tres pontos que nunca passa do ponto (Fritsch-Carlson):
// a media harmonica das inclinacoes de cada lado, zero se elas trocam de sinal.
[[nodiscard]] inline float gestureMonotoneTangent(float before, float after) {
    if (before * after <= 0.0f) return 0.0f;
    return 2.0f * before * after / (before + after);
}

} // namespace MatterEngine
