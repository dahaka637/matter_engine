#include "Engine/Audio/AudioDevice3D.hpp"

#include "Engine/Core/Log.hpp"

// A MatterEngine liga estaticamente contra o proprio OpenAL Soft (nao um
// OpenAL32.dll generico do sistema): as funcoes de filtro EFX sao simbolos C
// normais nesse build, entao podemos chama-las direto em vez de resolve-las
// em runtime via alGetProcAddress.
#define AL_ALEXT_PROTOTYPES

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>
#include <AL/efx-presets.h>
#include <AL/efx.h>

#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace MatterEngine {
namespace {

// 48 kHz e o padrao de producao para audio de jogos; todo buffer carregado
// e reamostrado para esta taxa unica, independente da taxa original do WAV.
constexpr int MixSampleRateHz = 48000;

void logAlErrors(const char* context) {
    for (ALenum error = alGetError(); error != AL_NO_ERROR;
        error = alGetError()) {
        Log::warn(std::string("OpenAL erro em ") + context + ": codigo "
            + std::to_string(error));
    }
}

// O contexto e criado com ALC_MAX_AUXILIARY_SENDS=2 (ver initialize()); todo
// o sistema de ambientes acusticos (AcousticEnvironmentSystem3D) foi
// desenhado em torno desse mesmo limite de 2 zonas simultaneas.
constexpr ALsizei EnvironmentSlotCount = 2;

// Parametros curados da propria OpenAL Soft (AL/efx-presets.h), em vez de
// valores de reverb inventados a mao - cada macro expande para um
// inicializador de EFXEAXREVERBPROPERTIES.
EFXEAXREVERBPROPERTIES reverbPropertiesForPreset(
    AcousticReverbPreset3D preset) {
    switch (preset) {
    case AcousticReverbPreset3D::Generic:
        return EFXEAXREVERBPROPERTIES EFX_REVERB_PRESET_GENERIC;
    case AcousticReverbPreset3D::SmallRoom:
        return EFXEAXREVERBPROPERTIES EFX_REVERB_PRESET_ROOM;
    case AcousticReverbPreset3D::Hallway:
        return EFXEAXREVERBPROPERTIES EFX_REVERB_PRESET_HALLWAY;
    case AcousticReverbPreset3D::Cave:
        return EFXEAXREVERBPROPERTIES EFX_REVERB_PRESET_CAVE;
    case AcousticReverbPreset3D::Outdoor:
        return EFXEAXREVERBPROPERTIES EFX_REVERB_PRESET_PLAIN;
    case AcousticReverbPreset3D::Warehouse:
        return EFXEAXREVERBPROPERTIES EFX_REVERB_PRESET_FACTORY_LARGEROOM;
    }
    return EFXEAXREVERBPROPERTIES EFX_REVERB_PRESET_GENERIC;
}

// So os campos escalares relevantes para um reverb ambiente generico -
// os vetores de pan (sempre {0,0,0} em todos os presets, ja que nenhum
// deles direciona a reflexao) e echo/modulacao (relevantes so para
// efeitos especiais, nao para "estou numa caverna") ficam de fora.
void applyReverbProperties(ALuint effect,
    const EFXEAXREVERBPROPERTIES& props) {
    alEffecti(effect, AL_EFFECT_TYPE, AL_EFFECT_EAXREVERB);
    alEffectf(effect, AL_EAXREVERB_DENSITY, props.flDensity);
    alEffectf(effect, AL_EAXREVERB_DIFFUSION, props.flDiffusion);
    alEffectf(effect, AL_EAXREVERB_GAIN, props.flGain);
    alEffectf(effect, AL_EAXREVERB_GAINHF, props.flGainHF);
    alEffectf(effect, AL_EAXREVERB_GAINLF, props.flGainLF);
    alEffectf(effect, AL_EAXREVERB_DECAY_TIME, props.flDecayTime);
    alEffectf(effect, AL_EAXREVERB_DECAY_HFRATIO, props.flDecayHFRatio);
    alEffectf(effect, AL_EAXREVERB_DECAY_LFRATIO, props.flDecayLFRatio);
    alEffectf(effect, AL_EAXREVERB_REFLECTIONS_GAIN,
        props.flReflectionsGain);
    alEffectf(effect, AL_EAXREVERB_REFLECTIONS_DELAY,
        props.flReflectionsDelay);
    alEffectf(effect, AL_EAXREVERB_LATE_REVERB_GAIN,
        props.flLateReverbGain);
    alEffectf(effect, AL_EAXREVERB_LATE_REVERB_DELAY,
        props.flLateReverbDelay);
    alEffectf(effect, AL_EAXREVERB_AIR_ABSORPTION_GAINHF,
        props.flAirAbsorptionGainHF);
    alEffectf(effect, AL_EAXREVERB_HFREFERENCE, props.flHFReference);
    alEffectf(effect, AL_EAXREVERB_LFREFERENCE, props.flLFReference);
    alEffectf(effect, AL_EAXREVERB_ROOM_ROLLOFF_FACTOR,
        props.flRoomRolloffFactor);
    alEffecti(effect, AL_EAXREVERB_DECAY_HFLIMIT, props.iDecayHFLimit);
}

} // namespace

struct AudioDevice3D::Impl {
    struct BufferEntry {
        ALuint handle = 0;
        float durationSeconds = 0.0f;
    };

    // Uma fonte OpenAL e um recurso finito do backend (nao um vetor que
    // cresce livremente como no antigo mixer por software). O pool e criado
    // uma unica vez em initialize() e reciclado por play()/playTimed().
    struct Voice {
        ALuint source = 0;
        // Filtro EFX passa-baixa dedicado desta voz (0 se EFX indisponivel).
        // Cada voz tem o seu proprio porque atribuir AL_DIRECT_FILTER a uma
        // fonte copia os parametros atuais do filtro naquele instante; um
        // filtro compartilhado entre vozes vazaria o muffle de uma no som
        // da outra.
        ALuint filter = 0;
        bool active = false;
        bool timed = false;
        // Voz de loop persistente (ver AudioDevice3D::playLooping) - nunca
        // expira sozinha (o OpenAL a mantem tocando via AL_LOOPING) e nunca
        // e roubada por outro som (ver findFreeVoice()); so termina via
        // stopLooping() explicito.
        bool looping = false;
        float elapsedSeconds = 0.0f;
        float durationSeconds = 0.0f;
        float fadeOutSeconds = 0.0f;
        float baseVolume = 1.0f;
        // AL_DIRECT_FILTER e escrita-somente no OpenAL (alGetSourcei nao o
        // aceita, so gera AL_INVALID_ENUM); guardamos o ultimo muffle
        // aplicado aqui para debugLastVoiceHasActiveFilter() poder
        // verificar reciclagem de voz sem depender de uma consulta que a
        // biblioteca nao oferece.
        float appliedMuffle = 0.0f;
    };

    ALCdevice* device = nullptr;
    ALCcontext* context = nullptr;
    bool hrtfIsActive = false;
    bool efxIsAvailable = false;
    std::vector<BufferEntry> buffers;
    std::vector<Voice> voices;
    // Rastreado so para debugLastVoiceHasActiveFilter() (uso exclusivo de
    // testes automatizados).
    Voice* lastVoiceUsed = nullptr;

    // Os 2 sends de ambiente acustico (Fase 3). Cada voz e roteada para
    // ambos desde a criacao (o roteamento em si nao muda); setEnvironment()
    // so troca o preset/ganho de cada slot conforme a zona mais proxima.
    std::array<ALuint, EnvironmentSlotCount> environmentSlots {};
    std::array<ALuint, EnvironmentSlotCount> environmentEffects {};
    // Evita recarregar os ~20 parametros do EAXREVERB a cada passo fixo
    // quando o preset de um slot nao mudou de um frame para o outro; so o
    // ganho (barato, e o que faz o blend suave por distancia) e sempre
    // reaplicado.
    std::array<bool, EnvironmentSlotCount> environmentSlotLoaded {};
    std::array<AcousticReverbPreset3D, EnvironmentSlotCount>
        environmentSlotPreset {};

    // Prefere uma voz ociosa; se o pool inteiro estiver ocupado, rouba a voz
    // mais avancada (a mais perto de terminar de qualquer forma) em vez de
    // simplesmente descartar o som novo em silencio. Vozes em loop
    // persistente (ex.: assobio de vento) ficam de fora da lista de roubo:
    // um loop cortado silenciosamente por um som transitorio qualquer seria
    // um bug dificil de notar (o ambiente simplesmente some sem log nenhum).
    Voice* findFreeVoice() {
        for (Voice& voice : voices) {
            if (!voice.active) return &voice;
        }
        Voice* oldest = nullptr;
        for (Voice& voice : voices) {
            if (voice.looping) continue;
            if (oldest == nullptr
                || voice.elapsedSeconds > oldest->elapsedSeconds) {
                oldest = &voice;
            }
        }
        // So chega aqui com oldest nulo se o pool inteiro (todas as
        // dezenas de vozes) estiver em loop simultaneamente - nao acontece
        // na pratica (o unico loop hoje e o assobio de vento, uma voz so);
        // o fallback existe para nunca retornar um ponteiro nulo.
        return oldest != nullptr ? oldest : &voices.front();
    }

    // muffle==0 remove o filtro (AL_FILTER_NULL), tanto para o caso comum
    // de sons limpos quanto para garantir que uma voz reciclada nunca
    // herde o abafamento do som anterior que tocou nela.
    void applyMuffle(Voice& voice, float muffle) {
        const float clamped = std::clamp(muffle, 0.0f, 1.0f);
        voice.appliedMuffle = clamped;
        if (!efxIsAvailable || voice.filter == 0) return;
        if (clamped <= 0.0f) {
            alSourcei(voice.source, AL_DIRECT_FILTER, AL_FILTER_NULL);
            return;
        }
        // AL_LOWPASS_GAIN fica em 1 (o ganho geral e responsabilidade do
        // AL_GAIN da fonte); so AL_LOWPASS_GAINHF corta os agudos.
        alFilterf(voice.filter, AL_LOWPASS_GAIN, 1.0f);
        alFilterf(voice.filter, AL_LOWPASS_GAINHF,
            std::clamp(1.0f - clamped * 0.92f, 0.03f, 1.0f));
        alSourcei(voice.source, AL_DIRECT_FILTER,
            static_cast<ALint>(voice.filter));
    }
};

AudioDevice3D::AudioDevice3D() = default;

AudioDevice3D::~AudioDevice3D() {
    shutdown();
}

bool AudioDevice3D::initialize(const Settings& settings) {
    if (m_impl != nullptr && m_impl->context != nullptr) {
        return true;
    }
    m_impl = std::make_unique<Impl>();

    m_impl->device = alcOpenDevice(nullptr);
    if (m_impl->device == nullptr) {
        Log::warn("alcOpenDevice falhou ao abrir o dispositivo padrao");
        m_impl.reset();
        return false;
    }

    // ALC_MAX_AUXILIARY_SENDS so pode ser pedido na criacao do contexto;
    // reservamos 2 sends agora mesmo sem uso ainda, para a Fase 3 (ambientes
    // acusticos) nao precisar reabrir o dispositivo depois.
    const ALCint attributes[] {
        ALC_HRTF_SOFT, settings.hrtfEnabled ? ALC_TRUE : ALC_FALSE,
        ALC_MAX_AUXILIARY_SENDS, 2,
        0
    };
    m_impl->context = alcCreateContext(m_impl->device, attributes);
    if (m_impl->context == nullptr
        || alcMakeContextCurrent(m_impl->context) == ALC_FALSE) {
        Log::warn("Falha ao criar/ativar o contexto OpenAL");
        if (m_impl->context != nullptr) {
            alcDestroyContext(m_impl->context);
        }
        alcCloseDevice(m_impl->device);
        m_impl.reset();
        return false;
    }

    ALCint hrtfStatus = ALC_FALSE;
    alcGetIntegerv(m_impl->device, ALC_HRTF_SOFT, 1, &hrtfStatus);
    m_impl->hrtfIsActive = hrtfStatus == ALC_TRUE;
    m_impl->efxIsAvailable =
        alcIsExtensionPresent(m_impl->device, "ALC_EXT_EFX") == ALC_TRUE;

    // Atenuacao continua por distancia (sem corte abrupto), com a curva
    // final ajustada por fonte logo abaixo via AL_REFERENCE_DISTANCE.
    alDistanceModel(AL_INVERSE_DISTANCE_CLAMPED);

    if (m_impl->efxIsAvailable) {
        alGenAuxiliaryEffectSlots(EnvironmentSlotCount,
            m_impl->environmentSlots.data());
        alGenEffects(EnvironmentSlotCount,
            m_impl->environmentEffects.data());
        for (ALsizei index = 0; index < EnvironmentSlotCount; ++index) {
            // Comeca silencioso: nenhuma zona acustica ainda foi definida
            // (WorldAudioController::setAcousticZones ainda nem rodou).
            alAuxiliaryEffectSlotf(m_impl->environmentSlots[
                static_cast<std::size_t>(index)], AL_EFFECTSLOT_GAIN, 0.0f);
        }
    }

    const std::uint32_t sourceCount = std::max<std::uint32_t>(
        1, settings.maxSources);
    m_impl->voices.resize(sourceCount);
    for (Impl::Voice& voice : m_impl->voices) {
        alGenSources(1, &voice.source);
        alSourcef(voice.source, AL_REFERENCE_DISTANCE, 6.0f);
        alSourcef(voice.source, AL_ROLLOFF_FACTOR, 1.0f);
        if (m_impl->efxIsAvailable) {
            alGenFilters(1, &voice.filter);
            alFilteri(voice.filter, AL_FILTER_TYPE, AL_FILTER_LOWPASS);
            for (ALsizei index = 0; index < EnvironmentSlotCount; ++index) {
                alSource3i(voice.source, AL_AUXILIARY_SEND_FILTER,
                    static_cast<ALint>(m_impl->environmentSlots[
                        static_cast<std::size_t>(index)]),
                    index, AL_FILTER_NULL);
            }
        }
    }
    logAlErrors("initialize");
    return true;
}

void AudioDevice3D::shutdown() {
    if (!m_impl) return;
    for (Impl::Voice& voice : m_impl->voices) {
        if (voice.source != 0) {
            alSourceStop(voice.source);
            alDeleteSources(1, &voice.source);
        }
        if (voice.filter != 0) {
            alDeleteFilters(1, &voice.filter);
        }
    }
    for (Impl::BufferEntry& buffer : m_impl->buffers) {
        if (buffer.handle != 0) {
            alDeleteBuffers(1, &buffer.handle);
        }
    }
    if (m_impl->efxIsAvailable) {
        alDeleteAuxiliaryEffectSlots(EnvironmentSlotCount,
            m_impl->environmentSlots.data());
        alDeleteEffects(EnvironmentSlotCount,
            m_impl->environmentEffects.data());
    }
    if (m_impl->context != nullptr) {
        alcMakeContextCurrent(nullptr);
        alcDestroyContext(m_impl->context);
    }
    if (m_impl->device != nullptr) {
        alcCloseDevice(m_impl->device);
    }
    m_impl.reset();
}

bool AudioDevice3D::hrtfActive() const {
    return m_impl != nullptr && m_impl->hrtfIsActive;
}

bool AudioDevice3D::efxAvailable() const {
    return m_impl != nullptr && m_impl->efxIsAvailable;
}

int AudioDevice3D::loadBuffer(const std::string& path) {
    if (!m_impl) return -1;

    SDL_AudioSpec sourceSpec {};
    Uint8* sourceBuffer = nullptr;
    Uint32 sourceLength = 0;
    if (!SDL_LoadWAV(path.c_str(), &sourceSpec, &sourceBuffer,
            &sourceLength)) {
        Log::warn("Falha ao carregar som '" + path + "': " + SDL_GetError());
        return -1;
    }

    const SDL_AudioSpec targetSpec { SDL_AUDIO_S16, 1, MixSampleRateHz };
    Uint8* convertedBuffer = nullptr;
    int convertedBytes = 0;
    const bool converted = SDL_ConvertAudioSamples(&sourceSpec, sourceBuffer,
        static_cast<int>(sourceLength), &targetSpec, &convertedBuffer,
        &convertedBytes);
    SDL_free(sourceBuffer);
    if (!converted) {
        Log::warn("Falha ao converter som '" + path + "': " + SDL_GetError());
        return -1;
    }

    ALuint handle = 0;
    alGenBuffers(1, &handle);
    alBufferData(handle, AL_FORMAT_MONO16, convertedBuffer, convertedBytes,
        MixSampleRateHz);
    SDL_free(convertedBuffer);
    logAlErrors("loadBuffer");
    if (handle == 0) return -1;

    const std::size_t frameCount = static_cast<std::size_t>(convertedBytes)
        / sizeof(std::int16_t);
    Impl::BufferEntry entry;
    entry.handle = handle;
    entry.durationSeconds = static_cast<float>(frameCount)
        / static_cast<float>(MixSampleRateHz);
    m_impl->buffers.push_back(entry);
    return static_cast<int>(m_impl->buffers.size()) - 1;
}

int AudioDevice3D::loadProceduralBuffer(
    std::span<const std::int16_t> monoSamples) {
    if (!m_impl || monoSamples.empty()) return -1;

    ALuint handle = 0;
    alGenBuffers(1, &handle);
    alBufferData(handle, AL_FORMAT_MONO16, monoSamples.data(),
        static_cast<ALsizei>(monoSamples.size_bytes()), MixSampleRateHz);
    logAlErrors("loadProceduralBuffer");
    if (handle == 0) return -1;

    Impl::BufferEntry entry;
    entry.handle = handle;
    entry.durationSeconds = static_cast<float>(monoSamples.size())
        / static_cast<float>(MixSampleRateHz);
    m_impl->buffers.push_back(entry);
    return static_cast<int>(m_impl->buffers.size()) - 1;
}

int AudioDevice3D::mixSampleRateHz() {
    return MixSampleRateHz;
}

float AudioDevice3D::bufferDurationSeconds(int bufferId) const {
    if (!m_impl || bufferId < 0
        || bufferId >= static_cast<int>(m_impl->buffers.size())) {
        return 0.0f;
    }
    return m_impl->buffers[static_cast<std::size_t>(bufferId)]
        .durationSeconds;
}

void AudioDevice3D::setListenerPose(const AudioListenerPose3D& pose) {
    if (!m_impl) return;
    alListener3f(AL_POSITION, pose.position.x, pose.position.y,
        pose.position.z);
    const std::array<float, 6> orientation {
        pose.forward.x, pose.forward.y, pose.forward.z,
        pose.up.x, pose.up.y, pose.up.z
    };
    alListenerfv(AL_ORIENTATION, orientation.data());
}

void AudioDevice3D::setEnvironment(const AcousticEnvironmentBlend3D& blend) {
    if (!m_impl || !m_impl->efxIsAvailable) return;
    for (std::size_t index = 0; index < EnvironmentSlotCount; ++index) {
        const ALuint slot = m_impl->environmentSlots[index];
        const AcousticEnvironmentBlend3D::Slot& target = blend.slots[index];
        if (!target.active || target.wetSendGain <= 0.0f) {
            alAuxiliaryEffectSlotf(slot, AL_EFFECTSLOT_GAIN, 0.0f);
            continue;
        }
        // Recarregar os ~20 parametros do EAXREVERB so quando o preset
        // realmente mudou; o ganho (o que produz o blend suave por
        // distancia) e barato e sempre reaplicado.
        if (!m_impl->environmentSlotLoaded[index]
            || m_impl->environmentSlotPreset[index] != target.preset) {
            const EFXEAXREVERBPROPERTIES props =
                reverbPropertiesForPreset(target.preset);
            applyReverbProperties(m_impl->environmentEffects[index], props);
            alAuxiliaryEffectSloti(slot, AL_EFFECTSLOT_EFFECT,
                static_cast<ALint>(m_impl->environmentEffects[index]));
            m_impl->environmentSlotLoaded[index] = true;
            m_impl->environmentSlotPreset[index] = target.preset;
        }
        alAuxiliaryEffectSlotf(slot, AL_EFFECTSLOT_GAIN,
            std::clamp(target.wetSendGain, 0.0f, 1.0f));
    }
}

void AudioDevice3D::play(int bufferId, Vec3 position, float volume,
    float pitch, float muffle) {
    if (!m_impl || bufferId < 0
        || bufferId >= static_cast<int>(m_impl->buffers.size())) {
        return;
    }
    Impl::Voice* voice = m_impl->findFreeVoice();
    const ALuint bufferHandle =
        m_impl->buffers[static_cast<std::size_t>(bufferId)].handle;
    alSourceStop(voice->source);
    alSourcei(voice->source, AL_BUFFER, static_cast<ALint>(bufferHandle));
    alSourcei(voice->source, AL_LOOPING, AL_FALSE);
    // Uma voz reciclada pode ter sido usada por playLooping() antes (ver
    // AL_SOURCE_RELATIVE la embaixo) - sem isto ela continuaria presa a
    // posicao do ouvinte para este som novo, que espera posicao absoluta.
    alSourcei(voice->source, AL_SOURCE_RELATIVE, AL_FALSE);
    alSource3f(voice->source, AL_POSITION, position.x, position.y,
        position.z);
    alSourcef(voice->source, AL_GAIN, std::clamp(volume, 0.0f, 1.0f));
    alSourcef(voice->source, AL_PITCH, std::max(0.01f, pitch));
    m_impl->applyMuffle(*voice, muffle);
    alSourcePlay(voice->source);

    voice->active = true;
    voice->timed = false;
    voice->looping = false;
    voice->elapsedSeconds = 0.0f;
    voice->baseVolume = std::clamp(volume, 0.0f, 1.0f);
    m_impl->lastVoiceUsed = voice;
}

void AudioDevice3D::playTimed(int bufferId, Vec3 position,
    float durationSeconds, float fadeOutSeconds, float volume, float pitch,
    float muffle) {
    if (!m_impl || bufferId < 0
        || bufferId >= static_cast<int>(m_impl->buffers.size())
        || durationSeconds <= 0.0f) {
        return;
    }
    Impl::Voice* voice = m_impl->findFreeVoice();
    const ALuint bufferHandle =
        m_impl->buffers[static_cast<std::size_t>(bufferId)].handle;
    alSourceStop(voice->source);
    alSourcei(voice->source, AL_BUFFER, static_cast<ALint>(bufferHandle));
    alSourcei(voice->source, AL_LOOPING, AL_FALSE);
    alSourcei(voice->source, AL_SOURCE_RELATIVE, AL_FALSE);
    alSource3f(voice->source, AL_POSITION, position.x, position.y,
        position.z);
    alSourcef(voice->source, AL_GAIN, std::clamp(volume, 0.0f, 1.0f));
    alSourcef(voice->source, AL_PITCH, std::max(0.01f, pitch));
    m_impl->applyMuffle(*voice, muffle);
    alSourcePlay(voice->source);

    voice->active = true;
    voice->timed = true;
    voice->looping = false;
    voice->elapsedSeconds = 0.0f;
    voice->durationSeconds = durationSeconds;
    voice->fadeOutSeconds = std::max(0.0f, fadeOutSeconds);
    voice->baseVolume = std::clamp(volume, 0.0f, 1.0f);
    m_impl->lastVoiceUsed = voice;
}

int AudioDevice3D::playLooping(int bufferId, Vec3 position, float volume,
    float pitch, bool listenerRelative) {
    if (!m_impl || bufferId < 0
        || bufferId >= static_cast<int>(m_impl->buffers.size())) {
        return -1;
    }
    Impl::Voice* voice = m_impl->findFreeVoice();
    const ALuint bufferHandle =
        m_impl->buffers[static_cast<std::size_t>(bufferId)].handle;
    alSourceStop(voice->source);
    alSourcei(voice->source, AL_BUFFER, static_cast<ALint>(bufferHandle));
    alSourcei(voice->source, AL_LOOPING, AL_TRUE);
    alSourcei(voice->source, AL_SOURCE_RELATIVE,
        listenerRelative ? AL_TRUE : AL_FALSE);
    alSource3f(voice->source, AL_POSITION, position.x, position.y,
        position.z);
    alSourcef(voice->source, AL_GAIN, std::clamp(volume, 0.0f, 1.0f));
    alSourcef(voice->source, AL_PITCH, std::max(0.01f, pitch));
    // Ambiencia continua nunca precisou de abafamento por obstrucao ate
    // hoje (so impactos usam muffle) - garante que uma voz reciclada nao
    // herde o filtro de um som anterior.
    m_impl->applyMuffle(*voice, 0.0f);
    alSourcePlay(voice->source);

    voice->active = true;
    voice->timed = false;
    voice->looping = true;
    voice->elapsedSeconds = 0.0f;
    voice->baseVolume = std::clamp(volume, 0.0f, 1.0f);
    m_impl->lastVoiceUsed = voice;
    return static_cast<int>(voice - m_impl->voices.data());
}

void AudioDevice3D::setLoopingVolume(int voiceHandle, float volume) {
    if (!m_impl || voiceHandle < 0
        || voiceHandle >= static_cast<int>(m_impl->voices.size())) {
        return;
    }
    Impl::Voice& voice = m_impl->voices[static_cast<std::size_t>(voiceHandle)];
    if (!voice.active || !voice.looping) return;
    voice.baseVolume = std::clamp(volume, 0.0f, 1.0f);
    alSourcef(voice.source, AL_GAIN, voice.baseVolume);
}

void AudioDevice3D::setLoopingPitch(int voiceHandle, float pitch) {
    if (!m_impl || voiceHandle < 0
        || voiceHandle >= static_cast<int>(m_impl->voices.size())) {
        return;
    }
    Impl::Voice& voice = m_impl->voices[static_cast<std::size_t>(voiceHandle)];
    if (!voice.active || !voice.looping) return;
    alSourcef(voice.source, AL_PITCH, std::max(0.01f, pitch));
}

void AudioDevice3D::setLoopingMuffle(int voiceHandle, float muffle) {
    if (!m_impl || voiceHandle < 0
        || voiceHandle >= static_cast<int>(m_impl->voices.size())) {
        return;
    }
    Impl::Voice& voice = m_impl->voices[static_cast<std::size_t>(voiceHandle)];
    if (!voice.active || !voice.looping) return;
    m_impl->applyMuffle(voice, muffle);
}

void AudioDevice3D::stopLooping(int voiceHandle) {
    if (!m_impl || voiceHandle < 0
        || voiceHandle >= static_cast<int>(m_impl->voices.size())) {
        return;
    }
    Impl::Voice& voice = m_impl->voices[static_cast<std::size_t>(voiceHandle)];
    if (!voice.looping) return;
    alSourceStop(voice.source);
    alSourcei(voice.source, AL_LOOPING, AL_FALSE);
    voice.active = false;
    voice.looping = false;
}

bool AudioDevice3D::debugLastVoiceHasActiveFilter() const {
    if (!m_impl || m_impl->lastVoiceUsed == nullptr) return false;
    return m_impl->lastVoiceUsed->appliedMuffle > 0.0f;
}

void AudioDevice3D::update(float deltaTime) {
    if (!m_impl) return;
    const float dt = std::max(0.0f, deltaTime);

    for (Impl::Voice& voice : m_impl->voices) {
        if (!voice.active) continue;
        voice.elapsedSeconds += dt;

        if (voice.timed) {
            if (voice.elapsedSeconds >= voice.durationSeconds) {
                alSourceStop(voice.source);
                voice.active = false;
                continue;
            }
            if (voice.fadeOutSeconds > 0.0f) {
                const float remaining =
                    voice.durationSeconds - voice.elapsedSeconds;
                if (remaining < voice.fadeOutSeconds) {
                    const float gain = voice.baseVolume * std::clamp(
                        remaining / voice.fadeOutSeconds, 0.0f, 1.0f);
                    alSourcef(voice.source, AL_GAIN, gain);
                }
            }
        } else if (!voice.looping) {
            ALint state = AL_STOPPED;
            alGetSourcei(voice.source, AL_SOURCE_STATE, &state);
            if (state == AL_STOPPED || state == AL_INITIAL) {
                voice.active = false;
            }
        }
        // Vozes em loop (AL_LOOPING) nunca chegam a AL_STOPPED sozinhas -
        // permanecem ativas ate stopLooping() ser chamado explicitamente.
    }
}

} // namespace MatterEngine
