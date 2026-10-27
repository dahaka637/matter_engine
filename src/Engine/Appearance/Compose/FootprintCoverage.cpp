#include "Engine/Appearance/Compose/FootprintCoverage.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace MatterEngine::Appearance {

// A media de uma quina e exata longe dela (o footprint todo de um lado) e, perto,
// vem das primitivas max(x, 0)^n / n!, que ali sao pequenas: nada de subtrair
// primitivas grandes quase iguais (com a = b = 1e-8, a diferenca de quatro
// primitivas da rampa dava 0,62 onde o certo e 0,5). Com b <= 1e-6 a, so a
// primeira primitiva (o erro e da ordem de b / a).
double kinkMean(double x, double a, double b, int order) {
    a = std::abs(a);
    b = std::abs(b);
    if (a < b) std::swap(a, b);
    const double h = 0.5 * (a + b);
    if (x - h >= 0.0) return order == 0 ? 1.0 : x;
    if (x + h <= 0.0) return 0.0;
    const auto power = [](double u, int n) {
        u = std::max(u, 0.0);
        double value = 1.0;
        for (int i = 1; i <= n; ++i) value *= u / static_cast<double>(i);
        return value;
    };
    if (b <= 1e-6 * a) return (power(x + 0.5 * a, order + 1) - power(x - 0.5 * a, order + 1)) / a;
    return (power(x + h, order + 2) - power(x + 0.5 * (a - b), order + 2) - power(x - 0.5 * (a - b), order + 2) + power(x - h, order + 2))
        / (a * b);
}

// A rampa presa clamp(s, 0, 1) e o degrau s >= 0. (A rampa com o footprint
// todo de um lado e exata; com o footprint de mais de 1e6, o degrau no meio dela,
// com erro de 1e-6: a diferenca de duas dobradicas enormes perderia tudo.)
double rampMean(double s, double a, double b) {
    const double h = 0.5 * (std::abs(a) + std::abs(b));
    if (s - h >= 1.0) return 1.0;
    if (s + h <= 0.0) return 0.0;
    if (h > 1e6) return kinkMean(s - 0.5, a, b, 0);
    return kinkMean(s, a, b, 1) - kinkMean(s - 1.0, a, b, 1);
}
double stepMean(double s, double a, double b) { return kinkMean(s, a, b, 0); }

// A onda quadrada (listra em [0, width) a cada period): o valor no comeco do
// footprint e os degraus dentro dele. (Com o footprint de mais de 32 periodos,
// a media de cada periodo inteiro mais o resto, pela primitiva da onda.) A fase
// pelo resto com sinal (exato): a reducao para [0, period) arredondaria uma
// fase negativa minuscula para period e apagaria a distancia a borda.
double stripeMean(double s, double a, double b, double period, double width) {
    s = std::remainder(s, period);
    const double h = 0.5 * (std::abs(a) + std::abs(b));
    const auto f1 = [&](double u) {
        const double k = std::floor(u / period);
        return k * width + std::clamp(u - k * period, 0.0, width);
    };
    if (h > 16.0 * period) {
        const double wide = std::max(std::abs(a), std::abs(b)), narrow = std::min(std::abs(a), std::abs(b));
        if (narrow <= 1e-6 * wide) return (f1(s + 0.5 * wide) - f1(s - 0.5 * wide)) / wide;
        const auto f2 = [&](double u) {
            const double k = std::floor(u / period), r = u - k * period;
            const double whole = width * period * k * (k - 1.0) * 0.5 + k * (width * period - 0.5 * width * width);
            return whole + k * width * r + (r <= width ? 0.5 * r * r : 0.5 * width * width + width * (r - width));
        };
        return (f2(s + h) - f2(s + 0.5 * (wide - narrow)) - f2(s - 0.5 * (wide - narrow)) + f2(s - h)) / (wide * narrow);
    }
    const double lo = s - h, hi = s + h;
    double mean = lo - std::floor(lo / period) * period < width ? 1.0 : 0.0;
    for (double k = std::floor(lo / period); k * period <= hi; k += 1.0) {
        const double rise = k * period, fall = rise + width;
        if (rise > lo && rise <= hi) mean += stepMean(s - rise, a, b);
        if (fall > lo && fall <= hi) mean -= stepMean(s - fall, a, b);
    }
    return mean;
}

} // namespace MatterEngine::Appearance
