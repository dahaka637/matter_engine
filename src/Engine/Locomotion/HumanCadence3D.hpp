#pragma once

#include <cstdlib>

namespace MatterEngine {

// A trava da cadencia (02/11, ele: "os movimentos freneticos... nao podem ocorrer NUNCA"): os pousos por segundo
// nunca passam de HumanCadenceCap x os de um humano na velocidade do centro de massa (os clipes: 1,9 ate 1,4 m/s,
// 2,73 a 3, 4,26 a 7,5 m/s e dali plano) - venha o passo do ciclo, da rede, da freada, do giro ou da recuperacao.
// Mais rapido, so com a passada mais longa. (T_HumanCadenceCap=0 a desliga, para medir.)
inline float humanCadenceCap3D() {
    static const float cap = [] {
        const char* value = std::getenv("T_HumanCadenceCap");
        return value != nullptr ? std::strtof(value, nullptr) : 1.15f;
    }();
    return cap;
}

inline float humanStepsPerSecond3D(float speed) {
    if (speed <= 1.4f) return 1.9f;
    if (speed <= 3.0f) return 1.9f + (speed - 1.4f) / 1.6f * 0.83f;
    if (speed <= 7.5f) return 2.73f + (speed - 3.0f) / 4.5f * 1.53f;
    return 4.26f;
}

// O menor intervalo entre dois pousos (s); 0 com a trava desligada.
inline float minimumStepInterval3D(float speed) {
    const float cap = humanCadenceCap3D();
    return cap > 0.0f ? 1.0f / (cap * humanStepsPerSecond3D(speed)) : 0.0f;
}

} // namespace MatterEngine
