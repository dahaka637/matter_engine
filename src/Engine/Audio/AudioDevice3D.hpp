#pragma once

#include "Engine/Audio/AcousticEnvironmentSystem3D.hpp"
#include "Engine/Math/Vec3.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace MatterEngine {

// Postura do ouvinte no espaco 3D do jogo (mesma convencao Z-up usada pelo
// resto da engine). `up` nao precisa ser recomputado por quem chama: um
// listener parado no chao pode sempre passar {0,0,1}.
struct AudioListenerPose3D {
    Vec3 position;
    Vec3 forward { 1.0f, 0.0f, 0.0f };
    Vec3 up { 0.0f, 0.0f, 1.0f };
};

// Fachada PIMPL sobre o OpenAL Soft: nenhum header AL/ALC/EFX atravessa esta
// API publica. HRTF (audio binaural), o filtro EFX de abafamento e a
// oclusao continua dependem apenas do mixer interno do OpenAL Soft, entao
// funcionam mesmo sob o backend "null" usado em testes automatizados
// headless.
class AudioDevice3D final {
public:
    struct Settings {
        bool hrtfEnabled = true;
        std::uint32_t maxSources = 64;
    };

    AudioDevice3D();
    ~AudioDevice3D();

    AudioDevice3D(const AudioDevice3D&) = delete;
    AudioDevice3D& operator=(const AudioDevice3D&) = delete;

    bool initialize(const Settings& settings = {});
    void shutdown();

    [[nodiscard]] bool hrtfActive() const;
    // Se falso, `muffle` em play()/playTimed() e aceito mas nao tem efeito
    // audivel (dispositivo sem a extensao EFX) - o chamador pode usar isto
    // para decidir se vale a pena calcular oclusao continua.
    [[nodiscard]] bool efxAvailable() const;

    // Carrega um WAV e converte para mono (fontes 3D do OpenAL so
    // espacializam/aplicam HRTF a buffers de um unico canal). Retorna um id
    // de buffer para passar a play()/playTimed(), ou -1 em falha.
    int loadBuffer(const std::string& path);

    // Contraparte de loadBuffer() para audio gerado por codigo em vez de
    // carregado de arquivo (ver Engine/Audio/ProceduralNoise.hpp) - recebe
    // PCM mono de 16 bits ja pronto, na taxa de mixSampleRateHz(), e cria um
    // buffer OpenAL diretamente a partir dele (sem passar pelo SDL_LoadWAV/
    // SDL_ConvertAudioSamples que loadBuffer() usa). Compartilha o mesmo
    // espaco de ids de loadBuffer() - os dois podem ser usados de forma
    // intercambiavel em qualquer play*(). Retorna -1 em falha.
    int loadProceduralBuffer(std::span<const std::int16_t> monoSamples);

    [[nodiscard]] float bufferDurationSeconds(int bufferId) const;

    // Taxa de amostragem interna de mixagem: todo buffer (de arquivo ou
    // procedural) e reamostrado/gerado nesta taxa. Publica para quem gera
    // audio proceduralmente poder casar a taxa exatamente, sem duplicar o
    // numero em dois arquivos.
    [[nodiscard]] static int mixSampleRateHz();

    void setListenerPose(const AudioListenerPose3D& pose);

    // Aplica a mistura de zonas acusticas atual (ver
    // AcousticEnvironmentSystem3D::evaluate) aos 2 sends auxiliares de
    // reverb reservados desde initialize(). Sem efeito se efxAvailable()
    // for falso. Toda voz do pool ja envia para os 2 slots desde a
    // criacao; o que muda aqui e o preset/ganho de cada slot.
    void setEnvironment(const AcousticEnvironmentBlend3D& blend);

    // `muffle` (0..1) aplica um filtro passa-baixa EFX por fonte (0 = sem
    // filtro, 1 = agudos quase totalmente cortados); sem efeito audivel se
    // efxAvailable() for falso.
    void play(int bufferId, Vec3 position, float volume = 1.0f,
        float pitch = 1.0f, float muffle = 0.0f);
    void playTimed(int bufferId, Vec3 position, float durationSeconds,
        float fadeOutSeconds, float volume = 1.0f, float pitch = 1.0f,
        float muffle = 0.0f);

    // Voz continua e persistente (ex.: assobio de vento) - toca em loop
    // nativo do OpenAL (AL_LOOPING) ate stopLooping() ser chamado; ao
    // contrario de play()/playTimed(), nunca e reciclada automaticamente por
    // outro som (ver findFreeVoice() em AudioDevice3D.cpp) nem expira
    // sozinha. Retorna um handle de voz para setLoopingVolume()/
    // setLoopingPitch()/stopLooping(), ou -1 em falha.
    // listenerRelative=true faz a fonte acompanhar o ouvinte automaticamente
    // (AL_SOURCE_RELATIVE, posicao interpretada como deslocamento do
    // ouvinte) sem precisar reenviar posicao a cada quadro - apropriado
    // para ambiencia "colada" no jogador, como vento no ouvido; nesse caso
    // `position` costuma ser {0,0,0}.
    int playLooping(int bufferId, Vec3 position, float volume = 1.0f,
        float pitch = 1.0f, bool listenerRelative = false);
    void setLoopingVolume(int voiceHandle, float volume);
    void setLoopingPitch(int voiceHandle, float pitch);
    // Mesmo filtro passa-baixa EFX de play()/playTimed() (0 = sem filtro,
    // 1 = agudos quase totalmente cortados), so que endereçavel por handle
    // de voz em loop em vez de reaplicado a cada chamada de play. Reutiliza
    // o mecanismo de abafamento por obstrucao para um proposito diferente
    // (moldar o timbre de uma ambiencia continua conforme a intensidade) -
    // o parametro fisico (o filtro em si) e o mesmo, so o motivo de quem
    // chama muda.
    void setLoopingMuffle(int voiceHandle, float muffle);
    void stopLooping(int voiceHandle);

    // O mixer do OpenAL Soft roda sozinho a partir de alSourcePlay(); esta
    // chamada so rampeia o ganho de fade-out e recicla vozes finalizadas.
    // Precisa ser chamada uma vez por passo fixo.
    void update(float deltaTime);

    // Somente para testes automatizados: reflete se o `muffle` passado na
    // ultima chamada a play()/playTimed() (para a voz que ela usou) foi
    // maior que zero. AL_DIRECT_FILTER e escrita-somente no OpenAL (nao da
    // pra consultar o filtro de volta), entao isto existe para verificar
    // que reciclar uma voz nunca herda o abafamento de um som anterior sem
    // relacao; nao deve ser usado por logica de jogo.
    [[nodiscard]] bool debugLastVoiceHasActiveFilter() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
