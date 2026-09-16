#include "Engine/Audio/BodyAcousticEnvelope.hpp"

#include <algorithm>

namespace MatterEngine {

void applyEnvelopeSpike(BodyAcousticEnvelopeState& state, float volume,
    float pitch, float muffle) {
    // So troca a identidade do pico (peak/pitch/muffle) quando este spike
    // realmente eleva o envelope audivel - um spike mais fraco que o pico
    // ainda decaindo nao deveria "roubar" o timbre do pico maior em curso.
    if (volume > state.volume) {
        state.peakVolume = volume;
        state.spikePitch = pitch;
        state.spikeMuffle = muffle;
    }
    state.volume = std::max(state.volume, volume);
}

void advanceEnvelope(BodyAcousticEnvelopeState& state, float sustainTarget,
    float decayPerSecond, float deltaTime) {
    if (deltaTime <= 0.0f) return;
    const float decayed = state.volume - decayPerSecond * deltaTime;
    // Nunca abaixo do teto de sustain - e o proprio "teto gradativo": o
    // arrasto define um piso continuo, o impacto so decai ATE ele, nunca
    // alem. Se o sustain sobe (arrasto ficou mais intenso) o envelope sobe
    // junto na hora, sem rampa propria - o proprio arrasto ja e continuo
    // quadro a quadro, nao precisa de uma segunda suavizacao por cima.
    state.volume = std::max(sustainTarget, decayed);
}

float envelopeSpikeFactor(const BodyAcousticEnvelopeState& state,
    float sustainTarget) {
    const float range = state.peakVolume - sustainTarget;
    // Faixa degenerada (pico igual ou abaixo do teto de sustain atual, ex.:
    // pico antigo ja irrelevante porque o sustain subiu depois) - sem pico
    // de verdade acontecendo, usa o timbre do arrasto.
    if (range <= 1.0e-5f) return 0.0f;
    return std::clamp((state.volume - sustainTarget) / range, 0.0f, 1.0f);
}

} // namespace MatterEngine
