#include "Engine/Environment/WindSystem.hpp"

#include "Engine/Math/Hash.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace MatterEngine {
namespace {

// Ruido de valor 1D com suavizacao quintica - interpola entre dois "quadros-
// chave" hasheados nos inteiros vizinhos de t. Totalmente deterministico: o
// mesmo par (t, seed) sempre produz o mesmo resultado, e t so cresce (nunca
// e negativo) porque WindSystem::advance() so acumula tempo positivo.
float valueNoise1D(float t, std::uint32_t seed) {
    const float cellFloor = std::floor(t);
    float fraction = t - cellFloor;
    fraction = fraction * fraction * fraction
        * (fraction * (fraction * 6.0f - 15.0f) + 10.0f);
    const std::uint32_t cellIndex =
        static_cast<std::uint32_t>(cellFloor) * 2654435761u + seed;
    const float a = hashToUnitFloat(cellIndex);
    const float b = hashToUnitFloat(cellIndex + 2654435761u);
    return a + (b - a) * fraction;
}

} // namespace

WindSystem::WindSystem(WindSettings3D settings) : m_settings(settings) {}

void WindSystem::advance(float deltaTimeSeconds) {
    if (deltaTimeSeconds <= 0.0f || !std::isfinite(deltaTimeSeconds)) return;
    m_elapsedSeconds += deltaTimeSeconds;
}

Vec3 WindSystem::velocityAtHeight(float heightMeters) const {
    // Direcao: vagueia lentamente entre 0 e 2*PI via ruido de valor 1D
    // escalado para um angulo completo - troca de direcao suave e continua,
    // nunca um salto instantaneo.
    constexpr float TwoPi = 6.28318530718f;
    const float directionNoise = valueNoise1D(
        m_elapsedSeconds * m_settings.directionWanderFrequencyHz, 1001u);
    const float headingRadians = directionNoise * TwoPi;

    // Rajada: ruido de valor centrado em zero (-1..1) multiplicando a
    // amplitude configurada - some e subtrai da velocidade base.
    const float gustNoise = valueNoise1D(
        m_elapsedSeconds * m_settings.gustFrequencyHz, 2003u) * 2.0f - 1.0f;
    const float speedAtReferenceHeight = std::max(0.0f,
        m_settings.baseSpeedMetersPerSecond
            + gustNoise * m_settings.gustAmplitudeMetersPerSecond);

    // Rampa chao->ceu (ver comentario de WindSettings3D::groundHeightMeters)
    // - smoothstep entre os dois marcos, 0 na linha do chao, 1 a partir da
    // altura de referencia. std::clamp evita depender de groundHeightMeters
    // < referenceHeightMeters (uma configuracao invertida so satura em 0 ou
    // 1, nunca produz um resultado fora de [0,1] nem divide por zero).
    const float heightRange = m_settings.referenceHeightMeters
        - m_settings.groundHeightMeters;
    const float t = std::abs(heightRange) < 1.0e-5f
        ? (heightMeters >= m_settings.referenceHeightMeters ? 1.0f : 0.0f)
        : std::clamp((heightMeters - m_settings.groundHeightMeters)
            / heightRange, 0.0f, 1.0f);
    const float heightMultiplier = t * t * (3.0f - 2.0f * t);
    const float speed = speedAtReferenceHeight * heightMultiplier;

    return Vec3 {
        std::cos(headingRadians) * speed,
        std::sin(headingRadians) * speed,
        0.0f
    };
}

} // namespace MatterEngine
