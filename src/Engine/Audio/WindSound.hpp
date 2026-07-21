#pragma once

#include <cstdint>
#include <vector>

namespace MatterEngine {

// Parametros de TIMBRE do vento (como ele soa) - deliberadamente separados
// de WindSettings3D (Engine/Environment/WindSystem.hpp), que descreve o
// vento como fenomeno FISICO (direcao/velocidade/rajada). Aqui e sintese de
// audio: como o ruido bruto e esculpido pelo filtro pra soar como vento de
// verdade, nao estatica.
struct WindSoundSettings {
    // Frequencia central do filtro passa-baixa ressonante, em Hz - e o que
    // separa "vento" de "ruido generico": um passa-baixa comum so abafa
    // agudos; um passa-baixa RESSONANTE (ver resonanceQ abaixo) reforca uma
    // banda estreita ao redor do corte, dando o timbre de assobio/uivo
    // caracteristico.
    float baseCutoffHz = 450.0f;
    // Quanto o LFO (oscilador lento) balanca o corte pra cima/baixo, em Hz.
    // Zero por padrao de proposito: um LFO baseado so em TEMPO (nao na
    // velocidade real do vento) cria uma "respiracao" ciclica propria do
    // timbre, dissociada da condicao real - somada ao loop do buffer de
    // ruido (WindNoiseDurationSeconds, ver WorldAudioController.cpp), isso
    // virava um padrao periodico perceptivel ("vem, vai, vem, vai") mesmo
    // com o vento estavel. A sensacao de forca do vento agora vem do PITCH
    // da voz em tempo real (ver WorldAudioController::updateWindAmbience -
    // pitch sobe continuamente junto com a velocidade de verdade), nao de
    // uma modulacao canned aqui. O campo continua existindo (nao e
    // mecanismo morto) para quem quiser reintroduzir alguma variacao no
    // futuro.
    float cutoffModulationHz = 0.0f;
    // Fator de qualidade do filtro - quanto maior, mais estreita e
    // ressonante a banda reforcada (mais "assobio"); valores baixos (~0.7)
    // soam como um passa-baixa comum, sem carater proprio.
    float resonanceQ = 1.8f;
    // Velocidade do LFO que modula o corte - sem efeito audivel enquanto
    // cutoffModulationHz acima for 0 (ver comentario la).
    float lfoFrequencyHz = 0.15f;
};

// Gera o ruido de vento PRONTO (ja moldado pelo filtro ressonante modulado
// - ver WindSoundSettings) como PCM mono de 16 bits, no mesmo formato que
// AudioDevice3D::loadProceduralBuffer() espera. Determinístico: a mesma
// entrada sempre produz a mesma saida.
//
// Esta e a primeira "receita" de som construida sobre os tijolos genericos
// de sintese (Waveform.hpp, BiquadFilter.hpp, Envelope.hpp, ProceduralNoise.
// hpp) - o padrao pensado para se repetir: cada efeito sonoro futuro da
// engine (passos, explosao, motor, eletricidade...) ganha sua propria
// funcao pequena e focada que combina os mesmos tijolos de um jeito
// diferente, em vez de crescer um unico gerador monolitico que tenta
// cobrir todos os casos com parametros demais.
[[nodiscard]] std::vector<std::int16_t> generateWindSoundSamples(
    float durationSeconds, int sampleRateHz, std::uint32_t seed,
    const WindSoundSettings& settings = {});

} // namespace MatterEngine
