#pragma once

// A cobertura no footprint do texel (o sistema de aparencia, etapa 2): a media
// de uma funcao da camada ao longo de uma direcao, s + a X + b Y, com X e Y
// uniformes em [-1/2, 1/2] (o texel e um paralelogramo no espaco da camada; a e
// b sao as derivadas da posicao por texel). Exata, e estavel de footprints
// minusculos a enormes. Somar |a| + |b| num intervalo so vale com um lado nulo:
// na rampa diagonal s = x + y daria 1/4 contra o exato 1/6.
namespace MatterEngine::Appearance {

// A quina: o degrau H(x) (order 0) ou a dobradica max(x, 0) (order 1).
[[nodiscard]] double kinkMean(double x, double a, double b, int order);
// A rampa presa clamp(s, 0, 1) (o degrade).
[[nodiscard]] double rampMean(double s, double a, double b);
// O degrau s >= 0 (a borda de uma faixa).
[[nodiscard]] double stepMean(double s, double a, double b);
// A onda quadrada: listra em [0, width) a cada period (as listras).
[[nodiscard]] double stripeMean(double s, double a, double b, double period, double width);

} // namespace MatterEngine::Appearance
