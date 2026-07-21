#pragma once

#include <cstdint>
#include <vector>

namespace MatterEngine {

// Ruido de banda larga tipo "vento" (aproximacao de ruido rosa, espectro
// 1/f - mais grave e natural que ruido branco puro, que soa como estatica
// aguda demais pra passar por vento) gerado inteiramente por codigo, sem
// depender de nenhum arquivo de audio gravado.
//
// Deterministico: a mesma tripla (duracao, taxa de amostragem, semente)
// sempre produz exatamente as mesmas amostras - o que permite testar por
// ctest e elimina a necessidade de versionar/distribuir um asset de audio
// so pra este som.
//
// Retorna PCM linear mono de 16 bits - o mesmo formato que
// AudioDevice3D::loadBuffer() produz ao converter um .wav; ver
// AudioDevice3D::loadProceduralBuffer(), a contraparte que recebe este
// resultado e cria um buffer OpenAL a partir dele.
[[nodiscard]] std::vector<std::int16_t> generateWindNoiseSamples(
    float durationSeconds, int sampleRateHz, std::uint32_t seed);

} // namespace MatterEngine
