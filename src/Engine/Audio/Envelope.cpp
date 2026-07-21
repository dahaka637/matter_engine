#include "Engine/Audio/Envelope.hpp"

#include <algorithm>

namespace MatterEngine {

float EnvelopeAdsr::sample(float elapsedSeconds,
    float totalDurationSeconds) const {
    if (elapsedSeconds < 0.0f) return 0.0f;

    // O release sempre ocupa os ultimos releaseSeconds da duracao total -
    // checar isso primeiro garante que sons curtos (mais curtos que
    // attack+decay) ainda terminem suavemente em vez de cortar abrupto.
    // A progressao usa a janela de release REALMENTE disponivel
    // (totalDurationSeconds - releaseStart), nao releaseSeconds diretamente:
    // se o som inteiro for mais curto que releaseSeconds, releaseStart fica
    // preso em 0 e o release precisa completar no tempo que sobrou, senao
    // o som terminaria abrupto no meio do release em vez de chegar a 0.
    const float releaseStart = std::max(0.0f,
        totalDurationSeconds - releaseSeconds);
    if (elapsedSeconds >= releaseStart) {
        const float releaseElapsed = elapsedSeconds - releaseStart;
        const float availableReleaseWindow =
            totalDurationSeconds - releaseStart;
        const float releaseProgress = availableReleaseWindow > 0.0f
            ? std::clamp(releaseElapsed / availableReleaseWindow, 0.0f, 1.0f)
            : 1.0f;
        return sustainLevel * (1.0f - releaseProgress);
    }
    if (elapsedSeconds < attackSeconds) {
        return attackSeconds > 0.0f
            ? std::clamp(elapsedSeconds / attackSeconds, 0.0f, 1.0f)
            : 1.0f;
    }
    const float decayElapsed = elapsedSeconds - attackSeconds;
    if (decayElapsed < decaySeconds) {
        const float decayProgress = decaySeconds > 0.0f
            ? std::clamp(decayElapsed / decaySeconds, 0.0f, 1.0f)
            : 1.0f;
        return 1.0f - decayProgress * (1.0f - sustainLevel);
    }
    return sustainLevel;
}

} // namespace MatterEngine
