#include "Engine/Audio/ProceduralNoise.hpp"

#include "Engine/Math/Hash.hpp"

#include <algorithm>
#include <cstdint>

namespace MatterEngine {

std::vector<std::int16_t> generateWindNoiseSamples(
    float durationSeconds, int sampleRateHz, std::uint32_t seed) {
    const std::size_t sampleCount = (durationSeconds > 0.0f && sampleRateHz > 0)
        ? static_cast<std::size_t>(durationSeconds
            * static_cast<float>(sampleRateHz))
        : 0;
    std::vector<std::int16_t> samples(sampleCount, 0);

    // Filtro de Paul Kellet (formulacao de dominio publico, musicdsp.org,
    // "Pink Noise") - soma seis filtros IIR de primeira ordem em paralelo,
    // cada um com um polo/ganho diferente, aproximando bem o espectro 1/f
    // do ruido rosa a partir de ruido branco puro. E a mesma tecnica usada
    // em geradores de ruido de vento/chuva de audio de jogos por ser barata
    // (poucas multiplicacoes por amostra) e nao exigir FFT nem tabelas.
    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f, b3 = 0.0f, b4 = 0.0f, b5 = 0.0f,
        b6 = 0.0f;

    // Constante multiplicativa impar grande (sequencia de Weyl) - garante
    // que indices consecutivos produzam entradas de hash bem distintas
    // entre si antes mesmo do hash em si misturar os bits, evitando
    // correlacao de curto alcance entre amostras vizinhas do ruido branco
    // de entrada (o filtro de Kellet acima e quem da a cor "rosa" ao
    // resultado; o branco de entrada precisa ser o mais descorrelacionado
    // possivel para nao introduzir um padrao proprio por cima).
    constexpr std::uint32_t WeylIncrement = 2654435761u;

    for (std::size_t index = 0; index < sampleCount; ++index) {
        const std::uint32_t hashInput =
            seed + static_cast<std::uint32_t>(index) * WeylIncrement;
        const float white = hashToUnitFloat(hashInput) * 2.0f - 1.0f;

        b0 = 0.99886f * b0 + white * 0.0555179f;
        b1 = 0.99332f * b1 + white * 0.0750759f;
        b2 = 0.96900f * b2 + white * 0.1538520f;
        b3 = 0.86650f * b3 + white * 0.3104856f;
        b4 = 0.55000f * b4 + white * 0.5329522f;
        b5 = -0.7616f * b5 - white * 0.0168980f;
        const float pink = b0 + b1 + b2 + b3 + b4 + b5 + b6 + white * 0.5362f;
        b6 = white * 0.115926f;

        // 0.11 e a constante de normalizacao padrao da formulacao de
        // Kellet, que traz a soma dos seis estagios de volta perto de
        // [-1, 1]; o clamp e so uma rede de seguranca contra picos raros
        // acima disso, nao o mecanismo principal de escala.
        const float normalized = std::clamp(pink * 0.11f, -1.0f, 1.0f);
        samples[index] = static_cast<std::int16_t>(normalized * 32767.0f);
    }
    return samples;
}

} // namespace MatterEngine
