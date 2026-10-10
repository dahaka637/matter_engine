#pragma once

#include <cmath>

namespace MatterEngine {

// Uma mola criticamente amortecida (valor e velocidade): vai ao alvo sem
// passar e sem salto de velocidade - o alvo pode mudar de uma vez (um plano
// novo, o lado que troca, o olhar depois do contato) e o corpo nao. Em
// `seconds` faz ~90% do caminho (omega = 4 / seconds). A solucao exata:
// estavel em qualquer passo.
template <typename T>
struct CriticalSpring3D {
    T value {};
    T rate {};

    void reset(T to) {
        value = to;
        rate = T {};
    }

    void step(T target, float seconds, float deltaTime) {
        if (seconds <= 1e-4f) {
            reset(target);
            return;
        }
        const float omega = 4.0f / seconds;
        const T error = value - target;
        const T slope = rate + error * omega;
        const float decay = std::exp(-omega * deltaTime);
        value = target + (error + slope * deltaTime) * decay;
        rate = (rate - slope * (omega * deltaTime)) * decay;
    }
};

// Duas molas criticas em serie: alem da velocidade, a aceleracao tambem
// comeca em zero (a mola sozinha, com o alvo mudando de uma vez, comeca
// acelerando omega^2 - o corpo dava o tranco no primeiro tick). Faz ~90% do
// caminho em ~0,85 `seconds`; o pico de aceleracao e ~2,6 vezes menor que o
// da mola sozinha no mesmo tempo.
template <typename T>
struct SmoothSpring3D {
    CriticalSpring3D<T> lead;
    CriticalSpring3D<T> follow;

    [[nodiscard]] T value() const { return follow.value; }
    [[nodiscard]] T rate() const { return follow.rate; }

    void reset(T to) {
        lead.reset(to);
        follow.reset(to);
    }

    void step(T target, float seconds, float deltaTime) {
        lead.step(target, 0.5f * seconds, deltaTime);
        follow.step(lead.value, 0.5f * seconds, deltaTime);
    }
};

} // namespace MatterEngine
