#pragma once

namespace MatterEngine {

// Tipos de filtro de segunda ordem cobertos pelas formulas do "Audio EQ
// Cookbook" (Robert Bristow-Johnson) - a referencia padrao da industria
// pra filtros digitais simples em audio, dominio publico e amplamente
// reusada. LowPass/HighPass moldam brilho (corta agudos/graves);
// BandPass/Notch isolam ou removem uma faixa estreita - Notch com Q alto
// e o que da o timbre de "assobio"/ressonancia caracteristico do vento
// quando aplicado sobre ruido (ver WindSound.cpp).
enum class BiquadFilterType {
    LowPass,
    HighPass,
    BandPass,
    Notch
};

struct BiquadCoefficients {
    float b0 = 1.0f;
    float b1 = 0.0f;
    float b2 = 0.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;
};

// cutoffHz e a frequencia central/de corte; q controla o quao estreita e a
// resposta (q alto = pico estreito e ressonante; q baixo/~0.707 = corte
// suave sem ressonancia perceptivel). cutoffHz e clampado internamente a
// uma faixa segura em relacao a sampleRateHz (o filtro fica instavel perto
// ou acima da frequencia de Nyquist).
[[nodiscard]] BiquadCoefficients biquadCoefficients(BiquadFilterType type,
    float sampleRateHz, float cutoffHz, float q);

// Estado (memoria de 2 amostras de entrada/saida) de uma secao biquad -
// separado dos coeficientes de proposito: os coeficientes podem mudar a
// cada amostra (corte modulado por um LFO, ver WindSound.cpp) sem perder o
// estado acumulado, o que seria impossivel se filtro fosse uma unica
// funcao sem memoria persistente entre chamadas.
struct BiquadState {
    float x1 = 0.0f;
    float x2 = 0.0f;
    float y1 = 0.0f;
    float y2 = 0.0f;

    [[nodiscard]] float process(float input, const BiquadCoefficients& c);
};

} // namespace MatterEngine
