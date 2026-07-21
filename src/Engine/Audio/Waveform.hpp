#pragma once

namespace MatterEngine {

// Formas de onda basicas de um oscilador - o tijolo TONAL da sintese
// procedural (ruido, ver ProceduralNoise.hpp, e o tijolo TEXTURAL). Junto
// com BiquadFilter.hpp e Envelope.hpp, formam o conjunto minimo de blocos
// pra compor qualquer efeito sonoro por codigo: um motor e um oscilador +
// harmonicos; um zap eletrico e uma onda quadrada com pitch decaindo rapido;
// um vento e ruido moldado por um filtro cujo corte um oscilador lento (um
// LFO - Low-Frequency Oscillator, o mesmo osciladorSample() usado numa
// frequencia bem baixa) vai abrindo e fechando (ver WindSound.cpp).
enum class Waveform {
    Sine,
    Square,
    Triangle,
    Sawtooth
};

// Amostra de uma forma de onda numa fase [0,1) (0 = inicio do ciclo, 1 =
// fim/proximo ciclo, valores fora do intervalo sao normalizados de volta
// pra ele). Funcao pura e sem estado - quem chama controla a fase (via
// acumulacao de fase propria, ou simplesmente frequencia*tempo decorrido
// para osciladores de frequencia constante). Retorna sempre em [-1, 1].
[[nodiscard]] float oscillatorSample(Waveform waveform, float phase01);

} // namespace MatterEngine
