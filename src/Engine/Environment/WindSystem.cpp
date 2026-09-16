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
    const float gustedSpeed = std::max(0.0f,
        m_settings.baseSpeedMetersPerSecond
            + gustNoise * m_settings.gustAmplitudeMetersPerSecond);

    // Envelope de calmaria/vento forte (ver WindSettings3D::
    // calmEnvelopeFrequencyHz): um canal de ruido MUITO mais lento que a
    // rajada acima, moldado por DOIS passes de smoothstep sobre o proprio
    // valor do ruido (nao so a interpolacao interna que valueNoise1D ja faz
    // entre celulas vizinhas). smoothstep tem inclinacao zero nas pontas e
    // maxima no meio - aplica-lo duas vezes empurra o resultado pra perto
    // dos extremos 0/1 com mais frequencia do que o ruido cru faria,
    // produzindo trechos longos de calmaria OU de vento forte, em vez da
    // maior parte do tempo num meio-termo mediano.
    const float calmRaw = valueNoise1D(
        m_elapsedSeconds * m_settings.calmEnvelopeFrequencyHz, 3301u);
    const float calmPass1 = calmRaw * calmRaw * (3.0f - 2.0f * calmRaw);
    const float calmPass2 =
        calmPass1 * calmPass1 * (3.0f - 2.0f * calmPass1);
    const float calmMultiplier = m_settings.calmEnvelopeFloor
        + (1.0f - m_settings.calmEnvelopeFloor) * calmPass2;
    const float speedAtReferenceHeight = gustedSpeed * calmMultiplier;

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
    const float smoothRamp = t * t * (3.0f - 2.0f * t);

    // Resquicio raro de rajada forte perto do chao (ver WindSettings3D::
    // groundGustEventFrequencyHz): canal de ruido independente (frequencia E
    // semente diferentes do envelope de calmaria acima - de proposito, para
    // que "calmaria geral" e "resquicio no chao" sejam percebidos como dois
    // fenomenos distintos, nao a mesma variacao reaproveitada), elevado a um
    // expoente alto para que a maior parte do tempo fique perto de zero com
    // picos breves e esparsos quando o ruido de base passa perto de 1.
    const float gustEventRaw = valueNoise1D(
        m_elapsedSeconds * m_settings.groundGustEventFrequencyHz, 5507u);
    const float gustEventSpike = std::pow(
        gustEventRaw, m_settings.groundGustEventSharpness);
    // Quao forte a rajada ATUAL esta (0..1) - o resquicio no chao so deveria
    // significar algo quando ha uma rajada forte de verdade rolando em
    // altura; sem isso, um pico do ruido de evento nao teria "forca" nenhuma
    // pra vazar (gustNoise ja e -1..1, ver acima).
    const float gustStrengthFraction =
        std::clamp(gustNoise * 0.5f + 0.5f, 0.0f, 1.0f);
    const float groundGustResidue = gustEventSpike * gustStrengthFraction
        * m_settings.groundGustResidueMaxFraction;

    // max() com uma expressao que nao depende de altura preserva a
    // monotonicidade em t da rampa suave: para t1<t2, smoothRamp(t1) <=
    // smoothRamp(t2) implica max(smoothRamp(t1), c) <= max(smoothRamp(t2), c)
    // para qualquer constante c fixa no instante. O resquicio tambem se
    // autoconfina a baixa altitude sem gate extra: com
    // groundGustResidueMaxFraction=0.65, smoothRamp so ultrapassa esse valor
    // por volta de t~0.57 (~5.7m), entao acima disso a rampa normal sempre
    // vence o max().
    const float heightMultiplier = std::max(smoothRamp, groundGustResidue);
    const float speed = speedAtReferenceHeight * heightMultiplier;

    return Vec3 {
        std::cos(headingRadians) * speed,
        std::sin(headingRadians) * speed,
        0.0f
    };
}

} // namespace MatterEngine
