#pragma once

namespace MatterEngine {

// Envelope ADSR (Attack/Decay/Sustain/Release) - controla como a amplitude
// de um som evolui no tempo, o terceiro tijolo fundamental de sintese
// (junto de Waveform.hpp e BiquadFilter.hpp). Praticamente todo efeito
// sonoro que nao seja um loop de intensidade constante precisa de um: um
// impacto e ruido/oscilador com ataque quase instantaneo e release curto;
// uma explosao tem ataque instantaneo e release longo; um motor sustentado
// usa um sustain longo com ataque/release suaves.
struct EnvelopeAdsr {
    float attackSeconds = 0.01f;
    float decaySeconds = 0.1f;
    float sustainLevel = 0.7f;
    float releaseSeconds = 0.2f;

    // Amplitude (0..1) no instante `elapsedSeconds` de um som com duracao
    // total `totalDurationSeconds`. Pensado para sons pre-renderizados de
    // duracao fixa (o release comeca automaticamente nos ultimos
    // releaseSeconds da duracao total) - nao ha um "note off" independente
    // como num sintetizador tocado ao vivo.
    [[nodiscard]] float sample(float elapsedSeconds,
        float totalDurationSeconds) const;
};

} // namespace MatterEngine
