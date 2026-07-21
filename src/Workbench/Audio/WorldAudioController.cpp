#include "Workbench/Audio/WorldAudioController.hpp"

#include "Engine/Audio/ProceduralNoise.hpp"
#include "Engine/Audio/WindSound.hpp"
#include "Engine/Core/Log.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace MatterEngine::Workbench {
namespace {

constexpr float SpeedOfSoundMetersPerSecond = 343.0f;
constexpr std::size_t MaximumSimultaneousImpactCommands = 12;

// Duracao do ruido de vento gerado (ver ProceduralNoise.hpp). Longo o
// bastante para o ouvido humano nao prender facilmente o ponto onde o loop
// repete - ruido de banda larga sem estrutura periodica propria (diferente
// de uma melodia) precisa de bem menos duracao que musica pra esconder a
// costura, mas alguns segundos ainda evitam qualquer sensacao de "clique"
// ritmico se alguem prestar atencao de proposito.
constexpr float WindNoiseDurationSeconds = 6.0f;
// Semente fixa (arbitraria) - determinismo total do ruido gerado, sem
// precisar versionar nenhum arquivo de audio (ver generateWindNoiseSamples).
constexpr std::uint32_t WindNoiseSeed = 0x57494e44u; // ASCII "WIND"

// O vento produz DOIS fenomenos acusticos reais e independentes, que este
// controlador modela como duas vozes separadas em vez de uma formula so:
//
//   1) Assobio de MOVIMENTO - o som de deslocar o proprio corpo/cabeca
//      rapido pelo ar. Depende da velocidade do JOGADOR, nao do vento.
//   2) Uivo AMBIENTE - o som do vento em si soprando forte, audivel mesmo
//      com o jogador parado (folhas/estruturas vibrando, ar turbulento
//      perto do ouvido). Depende so da intensidade do vento.
//
// A primeira tentativa desta feature usava um unico calculo (vento MENOS
// velocidade do jogador, um vetor so) pros dois casos ao mesmo tempo. Isso
// e hipersensivel por geometria: como o vento parado ja sopra alguns m/s,
// qualquer movimento do jogador numa direcao nao-alinhada com o vento SOMA
// por Pitagoras em vez de subtrair, entao um passo lento ja produzia uma
// velocidade relativa desproporcional ao movimento real. Separar os dois
// gatilhos resolve isso na raiz (nao so ajustando limiares de novo): o
// assobio de movimento agora reage so ao proprio jogador, previsivel
// independente de pra onde o vento estiver soprando.

// --- Assobio de movimento (velocidade PROPRIA do jogador) ---
// Abaixo disto, silencio total - deliberadamente perto do sprint (8.2 m/s,
// ver CharacterMotorSettings3D::sprintSpeed) para que so um deslocamento
// rapido de verdade dispare o som, nao um passo qualquer.
constexpr float WindMovementQuietThresholdMetersPerSecond = 7.0f;
// Velocidade em que o assobio de movimento se aproxima do volume maximo -
// alem do sprint sozinho, cobre queda rapida/voo veloz (ver
// CharacterMotorSettings3D::fastFlightSpeed).
constexpr float WindMovementReferenceSpeedMetersPerSecond = 20.0f;
// Deslocamento maximo de pitch no movimento mais rapido - sutil de
// proposito (friccao do ar realmente soa "mais aguda" quanto mais rapido,
// mas isto e um toque de realismo, nao um efeito chamativo).
constexpr float WindMovementMaxPitchBoost = 0.15f;
// Quase sem filtro - um assobio "colado no ouvido" deveria soar brilhante/
// proximo, nao abafado.
constexpr float WindMovementMuffle = 0.05f;

// --- Uivo ambiente (intensidade do vento em si) ---
// Rampa bem mais larga e mais baixa que a do assobio de movimento de
// proposito: o uivo ambiente deve comecar audivel (bem baixinho) ja numa
// brisa leve, crescendo aos poucos ate uma tempestade de verdade, em vez de
// ficar em silencio total ate o vento ficar excepcional e so entao "ligar"
// de repente. QuietThreshold perto de zero deixa so a calmaria total em
// silencio; ReferenceSpeed bem acima da faixa tipica de rajada default
// (baseSpeed + gustAmplitude, ver WindSettings3D) da uma rampa longa o
// bastante pra cobrir do sussurro ao vendaval sem saturar cedo demais.
constexpr float WindAmbientQuietThresholdMetersPerSecond = 0.8f;
constexpr float WindAmbientReferenceSpeedMetersPerSecond = 18.0f;
// Bem mais abafado que o assobio de movimento - um uivo distante/ambiente,
// nao um som proximo do ouvido; a diferenca de timbre ajuda a diferenciar
// os dois fenomenos mesmo tocando o mesmo buffer de ruido por baixo.
constexpr float WindAmbientMuffle = 0.55f;
// Deslocamento de pitch na velocidade de referencia - a mesma rampa que
// controla o volume tambem controla a frequencia, continuamente e em tempo
// real (sem LFO nem outra fonte de variacao por baixo, ver
// WindSoundSettings::cutoffModulationHz): em X velocidade de vento, sempre
// o mesmo X de deslocamento de pitch, e nada muda se o vento nao mudar.
// Mais pronunciado que o do assobio de movimento (WindMovementMaxPitchBoost)
// de proposito - aqui o pitch e o sinal PRINCIPAL de forca do vento, nao um
// toque sutil por cima de outra coisa.
constexpr float WindAmbientMaxPitchShift = 0.40f;

} // namespace

bool WorldAudioController::initialize(std::string_view assetsDirectory,
    bool hrtfEnabled) {
    if (m_initialized) return true;
    AudioDevice3D::Settings settings;
    settings.hrtfEnabled = hrtfEnabled;
    if (!m_audio.initialize(settings)) return false;
    if (hrtfEnabled && !m_audio.hrtfActive()) {
        Log::warn("HRTF foi pedido mas o OpenAL Soft nao ativou "
            "(dispositivo sem suporte); audio binaural desligado.");
    }

    const std::filesystem::path root =
        std::filesystem::path(assetsDirectory) / "audio" / "materials";
    std::error_code error;
    if (std::filesystem::exists(root, error)) {
        for (const std::filesystem::directory_entry& materialDirectory :
            std::filesystem::directory_iterator(root, error)) {
            if (error || !materialDirectory.is_directory()) continue;
            const std::string materialId =
                materialDirectory.path().filename().string();
            const std::filesystem::path impactDirectory =
                materialDirectory.path() / "impact";
            if (!std::filesystem::exists(impactDirectory, error)) continue;
            for (const std::filesystem::directory_entry& clipFile :
                std::filesystem::directory_iterator(impactDirectory, error)) {
                if (error || !clipFile.is_regular_file()
                    || clipFile.path().extension() != ".wav") {
                    continue;
                }
                const int clip = m_audio.loadBuffer(
                    clipFile.path().string());
                if (clip >= 0) {
                    m_impactClips.insert_or_assign(materialId, clip);
                    break;
                }
            }
        }
    }
    // Vento (ver WindSystem) - ruido gerado por codigo (ProceduralNoise.hpp),
    // nao um arquivo gravado; ver o comentario junto das constantes
    // WindMovement*/WindAmbient* acima para a divisao em duas vozes. Ambas
    // tocam em loop com ganho zero desde o inicio (em vez de iniciar/parar o
    // loop sob demanda), para nunca reiniciar a posicao de reproducao e
    // produzir um "corte" audivel toda vez que o vento cruza o limiar de ser
    // ouvido.
    // Ruido rosa bruto moldado por um filtro passa-baixa ressonante com
    // corte modulado por um LFO lento (ver WindSound.hpp) - e a diferenca
    // entre "vento de verdade" (timbre de assobio/uivo, intensidade
    // variando organicamente) e ruido generico sem carater proprio.
    const std::vector<std::int16_t> windNoiseSamples =
        generateWindSoundSamples(WindNoiseDurationSeconds,
            AudioDevice3D::mixSampleRateHz(), WindNoiseSeed);
    m_windClipId = m_audio.loadProceduralBuffer(windNoiseSamples);
    if (m_windClipId >= 0) {
        m_windMovementVoiceHandle = m_audio.playLooping(m_windClipId,
            Vec3 {}, 0.0f, 1.0f, /*listenerRelative=*/true);
        m_windAmbientVoiceHandle = m_audio.playLooping(m_windClipId,
            Vec3 {}, 0.0f, 1.0f, /*listenerRelative=*/true);
    } else {
        Log::warn("Ruido de vento procedural nao pode ser carregado - "
            "assobio/uivo de vento desligados.");
    }

    m_initialized = true;
    Log::info("Banco acústico carregado com "
        + std::to_string(m_impactClips.size()) + " materiais.");
    return true;
}

void WorldAudioController::shutdown() {
    if (m_windMovementVoiceHandle >= 0) {
        m_audio.stopLooping(m_windMovementVoiceHandle);
    }
    if (m_windAmbientVoiceHandle >= 0) {
        m_audio.stopLooping(m_windAmbientVoiceHandle);
    }
    m_windMovementVoiceHandle = -1;
    m_windAmbientVoiceHandle = -1;
    m_windClipId = -1;
    m_pendingImpacts.clear();
    m_impactClips.clear();
    m_audio.shutdown();
    m_initialized = false;
}

int WorldAudioController::clipForMaterial(
    const std::string& materialId) const {
    const auto exact = m_impactClips.find(materialId);
    if (exact != m_impactClips.end()) return exact->second;
    const auto fallback = m_impactClips.find("default");
    return fallback != m_impactClips.end() ? fallback->second : -1;
}

void WorldAudioController::submitImpacts(
    std::span<const ImpactSoundCommand3D> commands) {
    if (!m_initialized) return;
    const std::size_t count = std::min(commands.size(),
        MaximumSimultaneousImpactCommands);
    for (std::size_t index = 0; index < count; ++index) {
        const ImpactSoundCommand3D& command = commands[index];
        const int clip = clipForMaterial(command.materialId);
        if (clip < 0) continue;
        // O atraso final é calculado a partir da distância ao ouvinte durante
        // update(); inicialmente zero permite que fontes próximas toquem no
        // mesmo passo e evita armazenar uma posição antiga do ouvinte.
        m_pendingImpacts.push_back({ command, clip, -1.0f });
    }
}

WorldAudioController::SpatialParameters WorldAudioController::spatialize(
    Vec3 source, Vec3 listenerPosition,
    const PhysicsScene3D* physicsScene) const {
    SpatialParameters result;
    const Vec3 offset = source - listenerPosition;
    const float distance = offset.length();

    // Distância/atenuação já são inteiramente responsabilidade do modelo 3D
    // do OpenAL (AL_REFERENCE_DISTANCE/AL_ROLLOFF_FACTOR); calcular de novo
    // aqui duplicaria a queda por distância. Esta função só decide se há
    // obstrução de linha de visada entre ouvinte e fonte.
    result.additionalMuffle = std::clamp(distance / 180.0f, 0.0f, 0.34f);

    if (physicsScene != nullptr && distance > 0.20f) {
        const float occlusion = computeOcclusionFactor3D(
            *physicsScene, listenerPosition, source);
        // Sem fingir uma simulação de difração completa: uma obstrução
        // reduz energia direta e fecha o filtro, preservando ainda a
        // parcela que chegaria por reflexão/difração. A amostragem em
        // cruz (computeOcclusionFactor3D) torna essa transição contínua em
        // vez de alternar abruptamente entre livre e bloqueado.
        result.occlusionGain = std::clamp(1.0f - occlusion * 0.48f,
            0.0f, 1.0f);
        result.additionalMuffle = std::max(
            result.additionalMuffle, occlusion * 0.58f);
    }
    return result;
}

void WorldAudioController::setAcousticZones(
    std::vector<AcousticZoneDefinition3D> zones) {
    m_environment.setZones(std::move(zones));
}

void WorldAudioController::update(const AudioListenerPose3D& listenerPose,
    const PhysicsScene3D* physicsScene, float deltaTime) {
    if (!m_initialized) return;
    m_audio.setListenerPose(listenerPose);
    m_audio.setEnvironment(m_environment.evaluate(listenerPose.position));

    for (auto iterator = m_pendingImpacts.begin();
        iterator != m_pendingImpacts.end();) {
        PendingImpact& pending = *iterator;
        const float distance =
            (pending.command.position - listenerPose.position).length();
        if (pending.delaySeconds < 0.0f) {
            pending.delaySeconds = distance
                / SpeedOfSoundMetersPerSecond;
        }
        pending.delaySeconds -= std::max(0.0f, deltaTime);
        if (pending.delaySeconds > 0.0f) {
            ++iterator;
            continue;
        }

        const SpatialParameters spatial = spatialize(
            pending.command.position, listenerPose.position, physicsScene);
        const float volume = std::clamp(
            pending.command.volume * spatial.occlusionGain, 0.0f, 1.0f);
        if (volume > 0.002f) {
            const float clipDuration =
                m_audio.bufferDurationSeconds(pending.clipId);
            const float duration = std::min(clipDuration,
                pending.command.durationSeconds
                    / std::max(0.05f, pending.command.pitch));
            m_audio.playTimed(pending.clipId, pending.command.position,
                std::max(0.035f, duration),
                std::min(0.055f, duration * 0.35f), volume,
                pending.command.pitch,
                std::clamp(pending.command.muffle
                    + spatial.additionalMuffle, 0.0f, 0.98f));
        }
        iterator = m_pendingImpacts.erase(iterator);
    }
}

void WorldAudioController::updateWindAmbience(
    Vec3 windVelocityMetersPerSecond, Vec3 characterVelocityMetersPerSecond,
    float masterVolume, float /*deltaTime*/) {
    if (!m_initialized) return;
    const float clampedMasterVolume = std::clamp(masterVolume, 0.0f, 1.0f);

    if (m_windMovementVoiceHandle >= 0) {
        // So a velocidade PROPRIA do jogador - ver o comentario junto das
        // constantes WindMovement*/WindAmbient* sobre por que isto nao usa
        // mais o vetor relativo ao vento.
        const float playerSpeed = characterVelocityMetersPerSecond.length();
        const float ramp = std::clamp(
            (playerSpeed - WindMovementQuietThresholdMetersPerSecond)
                / (WindMovementReferenceSpeedMetersPerSecond
                    - WindMovementQuietThresholdMetersPerSecond),
            0.0f, 1.0f);
        // Ruido aerodinamico real cresce mais rapido que linear com a
        // velocidade (~v^2) - o quadrado da rampa aproxima essa curva sem
        // simular acustica de verdade.
        const float volume = ramp * ramp * clampedMasterVolume;
        const float pitch = 1.0f + ramp * WindMovementMaxPitchBoost;
        m_audio.setLoopingVolume(m_windMovementVoiceHandle, volume);
        m_audio.setLoopingPitch(m_windMovementVoiceHandle, pitch);
        m_audio.setLoopingMuffle(m_windMovementVoiceHandle,
            WindMovementMuffle);
    }

    if (m_windAmbientVoiceHandle >= 0) {
        // So a intensidade do vento em si - independente do jogador estar
        // se movendo ou nao.
        const float windSpeed = windVelocityMetersPerSecond.length();
        const float ramp = std::clamp(
            (windSpeed - WindAmbientQuietThresholdMetersPerSecond)
                / (WindAmbientReferenceSpeedMetersPerSecond
                    - WindAmbientQuietThresholdMetersPerSecond),
            0.0f, 1.0f);
        const float volume = ramp * ramp * clampedMasterVolume;
        const float pitch = 1.0f + ramp * WindAmbientMaxPitchShift;
        m_audio.setLoopingVolume(m_windAmbientVoiceHandle, volume);
        m_audio.setLoopingPitch(m_windAmbientVoiceHandle, pitch);
        m_audio.setLoopingMuffle(m_windAmbientVoiceHandle, WindAmbientMuffle);
    }
}

} // namespace MatterEngine::Workbench
