#include "Engine/Audio/WindSound.hpp"

#include "Engine/Audio/BiquadFilter.hpp"
#include "Engine/Audio/ProceduralNoise.hpp"
#include "Engine/Audio/Waveform.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

std::vector<std::int16_t> generateWindSoundSamples(
    float durationSeconds, int sampleRateHz, std::uint32_t seed,
    const WindSoundSettings& settings) {
    // Ruido rosa bruto (textura de banda larga) - ver ProceduralNoise.hpp.
    // Sozinho ja soa "parecido com vento" de longe, mas sem o timbre de
    // assobio/uivo que separa vento de estatica generica; o filtro abaixo
    // e o que faz essa diferenca.
    const std::vector<std::int16_t> rawNoise =
        generateWindNoiseSamples(durationSeconds, sampleRateHz, seed);
    if (rawNoise.empty()) return rawNoise;

    const float safeSampleRate =
        static_cast<float>(std::max(1, sampleRateHz));
    BiquadState filterState;
    std::vector<std::int16_t> shaped(rawNoise.size());

    for (std::size_t index = 0; index < rawNoise.size(); ++index) {
        const float elapsedSeconds =
            static_cast<float>(index) / safeSampleRate;

        // LFO (oscilador de baixa frequencia) abrindo e fechando o corte do
        // filtro ao longo do tempo - e o que da a sensacao de rajadas
        // turbulentas variando, em vez de um timbre estatico e artificial.
        const float lfo = oscillatorSample(Waveform::Sine,
            elapsedSeconds * settings.lfoFrequencyHz);
        const float cutoffHz = std::max(20.0f, settings.baseCutoffHz
            + lfo * settings.cutoffModulationHz);

        // Recalcular os coeficientes a cada amostra e barato o bastante
        // aqui porque isto roda so UMA VEZ, na inicializacao (nao em um
        // callback de audio em tempo real) - simplicidade e clareza
        // ganham de otimizacoes prematuras (ex.: recalcular so a cada N
        // amostras) que a ninguem vai notar num custo de inicializacao.
        const BiquadCoefficients coefficients = biquadCoefficients(
            BiquadFilterType::LowPass, safeSampleRate, cutoffHz,
            settings.resonanceQ);

        const float input =
            static_cast<float>(rawNoise[index]) * (1.0f / 32768.0f);
        const float filtered = filterState.process(input, coefficients);
        shaped[index] = static_cast<std::int16_t>(
            std::clamp(filtered, -1.0f, 1.0f) * 32767.0f);
    }
    return shaped;
}

} // namespace MatterEngine
