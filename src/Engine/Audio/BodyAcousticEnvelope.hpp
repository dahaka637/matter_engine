#pragma once

namespace MatterEngine {

// Estado do envelope acustico de UM corpo. Existe pra unificar impacto
// (pico transiente) e arrasto/atrito (teto continuo) numa unica voz por
// corpo, em vez de duas vozes independentes tocando o mesmo clip por cima
// uma da outra (ver WorldAudioController::m_bodyVoices). Funcoes puras, sem
// nenhum estado de OpenAL - testaveis sem AudioDevice3D real, mesmo
// espirito de WindAmbienceRamp.hpp.
struct BodyAcousticEnvelopeState {
    float volume = 0.0f;
    // Pico do ultimo spike aplicado - normaliza envelopeSpikeFactor, nao
    // reseta sozinho (so um novo spike mais alto o substitui).
    float peakVolume = 0.0f;
    float spikePitch = 1.0f;
    float spikeMuffle = 0.0f;
};

// Impacto: eleva o envelope na hora (nunca reduz - um pico mais fraco que o
// volume atual so seria mascarado mesmo). Um pico novo mais alto que o
// anterior vira a nova referencia de envelopeSpikeFactor.
void applyEnvelopeSpike(BodyAcousticEnvelopeState& state, float volume,
    float pitch, float muffle);

// Avanca um passo fixo: decai exponencialmente em direcao ao teto de
// sustain definido pelo arrasto (nunca abaixo dele - e literalmente o
// "teto gradativo" pedido: o arrasto e o piso/teto continuo, o impacto e
// so um pico temporario por cima).
void advanceEnvelope(BodyAcousticEnvelopeState& state, float sustainTarget,
    float decayPerSecond, float deltaTime);

// Fracao 0..1 de "quao perto do pico do ultimo impacto" o envelope esta
// agora - usada pra misturar o timbre (pitch/muffle) do impacto com o do
// arrasto sem precisar de uma maquina de estados separada por tempo. 1 =
// acabou de ter um pico, ainda no volume dele; 0 = ja decaiu de volta pro
// teto de sustain (ou zero, se nao ha arrasto).
[[nodiscard]] float envelopeSpikeFactor(
    const BodyAcousticEnvelopeState& state, float sustainTarget);

} // namespace MatterEngine
