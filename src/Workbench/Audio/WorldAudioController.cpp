#include "Workbench/Audio/WorldAudioController.hpp"

#include "Engine/Audio/ProceduralNoise.hpp"
#include "Engine/Audio/WindAmbienceRamp.hpp"
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

// --- Voz unica por corpo (impacto = pico, arrasto/atrito = teto de
// sustain - ver BodyAcousticEnvelope.hpp) ---
// Bem abaixo do pool de vozes do AudioDevice3D (64 fontes) - vozes em loop
// nunca sao roubadas por outro som (ver AudioDevice3D::findFreeVoice), entao
// um numero ilimitado de corpos simultaneos esgotaria o pool em silencio;
// sobra espaco de sobra pra vento/outros sons com esse teto.
constexpr std::size_t MaximumSimultaneousBodyVoices = 8;
// Teto de volume do arrasto sobre a intensidade 0..1 do resolver - sempre
// bem mais baixo que um impacto de verdade ("menor", como pedido), mesmo no
// caso mais extremo (bigorna arrastando com forca total).
constexpr float DragMaxVolumeFraction = 0.35f;
// Mais grave que o clip original - ajuda a diferenciar de uma pancada
// repetindo quando o mesmo clip sustenta por baixo do arrasto.
constexpr float DragPitch = 0.75f;
// Bem abafado - arredonda o ataque transiente do clip de impacto (pensado
// pra tocar uma vez, nao sustentar), lendo mais como textura continua de
// raspado do que como "clonk-clonk-clonk".
constexpr float DragMuffle = 0.65f;
// Tempo em silencio total (sem pico nem sustain) antes de parar a voz de
// vez - evita ficar ligando/desligando a voz a cada quadro numa transicao
// rapida (ex.: arrasto que pausa e retoma quase na hora).
constexpr float BodyVoiceSilenceGraceSeconds = 0.3f;

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
// Velocidade em que o assobio de movimento atinge o volume maximo - ancorada
// exatamente em CharacterMotorSettings3D::fastFlightSpeed (o teto real de
// velocidade do personagem), nao um numero solto. Antes disto era 20.0f
// (abaixo de fastFlightSpeed=28.0f): a rampa saturava em 1.0 assim que o
// jogador excedia 20 m/s, sobrando ~8 m/s de voo veloz sem nenhum aumento
// audivel de volume - alem disso, com expoente 2 (ver ramp*ramp), a faixa
// 7-20 ja soava perto do talo bem antes do voo padrao (12 m/s) terminar.
// Ampliar a referencia pra 28 e trocar o expoente pra 3 (ver
// WindMovementCurveExponent) resolvem os dois problemas juntos.
constexpr float WindMovementReferenceSpeedMetersPerSecond = 28.0f;
// Expoente da rampa de volume (ver windAmbienceVolumeRamp). 3 em vez do
// quadratico padrao: sobe mais devagar no meio da faixa de velocidades reais
// do personagem (andar/correr/voo padrao) e acelera mais perto do topo, pra
// que o voo padrao (12 m/s) va soar claramente "comecando", nao "quase no
// maximo".
constexpr float WindMovementCurveExponent = 3.0f;
// Deslocamento maximo de pitch no movimento mais rapido - sutil de
// proposito (friccao do ar realmente soa "mais aguda" quanto mais rapido,
// mas isto e um toque de realismo, nao um efeito chamativo). Continua usando
// a rampa LINEAR (nao a curva de volume acima) - so o volume precisa do
// crescimento mais gradual, o pitch nao deveria ficar tao contido assim.
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
// Expoente quadratico classico (ver windAmbienceVolumeRamp) - preservado sem
// mudanca de comportamento; so o assobio de movimento precisava de uma curva
// mais gradual.
constexpr float WindAmbientCurveExponent = 2.0f;
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
    for (const auto& [bodyId, state] : m_bodyVoices) {
        m_audio.stopLooping(state.voiceHandle);
    }
    m_bodyVoices.clear();
    m_pendingSpikes.clear();
    m_pendingSustain.clear();
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
        // O atraso final é calculado a partir da distância ao ouvinte
        // durante update(); inicialmente negativo sinaliza "ainda nao
        // calculado" (permite fontes próximas tocarem no mesmo passo, ver
        // update()). O pico so e de fato aplicado na voz do corpo quando o
        // atraso zerar - nunca cria uma voz propria (ver
        // findOrCreateBodyVoice).
        PendingSpike pending;
        pending.sourceBodyId = command.sourceBodyId;
        pending.position = command.position;
        pending.volume = command.volume;
        pending.pitch = command.pitch;
        pending.muffle = command.muffle;
        pending.durationSeconds = command.durationSeconds;
        pending.clipId = clip;
        pending.delaySeconds = -1.0f;
        m_pendingSpikes.push_back(pending);
    }
}

void WorldAudioController::submitDrags(
    std::span<const DragSoundCommand3D> commands) {
    if (!m_initialized) return;
    // Reconstruido do zero todo quadro - "quem esta arrastando agora" nao
    // acumula estado proprio, so alimenta update() no mesmo passo (ver
    // m_pendingSustain).
    m_pendingSustain.clear();
    for (const DragSoundCommand3D& command : commands) {
        // Reaproveita o clip de IMPACTO ja carregado (por materialId, nao
        // soundSet) como fonte do sustain de arrasto - nenhum asset novo
        // exige ser autorado agora (ver DragAcoustics.hpp/soundSet
        // reservado mas ainda sem WAV correspondente).
        const int clip = clipForMaterial(command.materialId);
        if (clip < 0) continue;
        m_pendingSustain[command.sourceBodyId] =
            { command.position, command.intensity, clip };
    }
}

WorldAudioController::BodyVoiceState*
WorldAudioController::findOrCreateBodyVoice(std::uint64_t sourceBodyId,
    int clipId, Vec3 position, float incomingLevel) {
    auto tracked = m_bodyVoices.find(sourceBodyId);
    if (tracked != m_bodyVoices.end()) return &tracked->second;

    if (m_bodyVoices.size() >= MaximumSimultaneousBodyVoices) {
        // Pool cheio: so abre espaco se este pedido for mais intenso que o
        // corpo mais fraco ja tocando agora (pico ou sustain, o que for
        // maior).
        auto weakest = m_bodyVoices.begin();
        auto weakestLevel = [](const BodyVoiceState& state) {
            return std::max(state.envelope.volume, state.sustainTarget);
        };
        for (auto it = m_bodyVoices.begin(); it != m_bodyVoices.end();
            ++it) {
            if (weakestLevel(it->second) < weakestLevel(weakest->second)) {
                weakest = it;
            }
        }
        if (weakestLevel(weakest->second) >= incomingLevel) return nullptr;
        m_audio.stopLooping(weakest->second.voiceHandle);
        m_bodyVoices.erase(weakest);
    }

    BodyVoiceState state;
    state.voiceHandle = m_audio.playLooping(clipId, position, 0.0f, 1.0f,
        /*listenerRelative=*/false);
    if (state.voiceHandle < 0) return nullptr;
    state.position = position;
    return &m_bodyVoices.emplace(sourceBodyId, state).first->second;
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

    // 1) Impactos pendentes (atraso de propagacao) que ja chegaram viram um
    // PICO no envelope da voz unica do corpo - nunca uma voz propria (ver
    // BodyAcousticEnvelope.hpp e o comentario de submitImpacts).
    for (auto iterator = m_pendingSpikes.begin();
        iterator != m_pendingSpikes.end();) {
        PendingSpike& pending = *iterator;
        const float distance =
            (pending.position - listenerPose.position).length();
        if (pending.delaySeconds < 0.0f) {
            pending.delaySeconds = distance / SpeedOfSoundMetersPerSecond;
        }
        pending.delaySeconds -= std::max(0.0f, deltaTime);
        if (pending.delaySeconds > 0.0f) {
            ++iterator;
            continue;
        }

        const SpatialParameters spatial = spatialize(
            pending.position, listenerPose.position, physicsScene);
        const float volume = std::clamp(
            pending.volume * spatial.occlusionGain, 0.0f, 1.0f);
        if (volume > 0.002f) {
            BodyVoiceState* voice = findOrCreateBodyVoice(
                pending.sourceBodyId, pending.clipId, pending.position,
                volume);
            if (voice != nullptr) {
                const float muffle = std::clamp(
                    pending.muffle + spatial.additionalMuffle, 0.0f, 0.98f);
                applyEnvelopeSpike(voice->envelope, volume, pending.pitch,
                    muffle);
                // O pico decai de volta pro teto de sustain (ou a zero, se
                // o corpo nao estiver arrastando) no mesmo tempo que o
                // impacto duraria sozinho hoje - reaproveita
                // durationSeconds, ja calibrado por material/tamanho pelo
                // resolver (ver ImpactAcoustics.cpp), em vez de inventar
                // uma taxa de decaimento nova.
                voice->decayPerSecond = voice->envelope.peakVolume
                    / std::max(0.05f, pending.durationSeconds);
                voice->position = pending.position;
            }
        }
        iterator = m_pendingSpikes.erase(iterator);
    }

    // 2) Teto de sustain do arrasto (ver DragAcoustics.hpp) - so aplica
    // oclusao/abafamento por distancia aqui: submitDrags() nao tem acesso a
    // listenerPose/physicsScene. Antes o arrasto nunca passava por
    // spatialize() (um corpo arrastando atras de uma parede soava igual a
    // um em campo aberto) - este redesenho fecha essa lacuna de graca, ja
    // que agora arrasto e impacto passam pelo mesmo caminho.
    for (const auto& [sourceBodyId, sustain] : m_pendingSustain) {
        const SpatialParameters spatial = spatialize(
            sustain.position, listenerPose.position, physicsScene);
        const float sustainVolume = std::clamp(sustain.intensity
            * DragMaxVolumeFraction * spatial.occlusionGain, 0.0f, 1.0f);
        if (sustainVolume < 0.001f) continue;
        BodyVoiceState* voice = findOrCreateBodyVoice(sourceBodyId,
            sustain.clipId, sustain.position, sustainVolume);
        if (voice == nullptr) continue;
        voice->position = sustain.position;
        voice->sustainTarget = sustainVolume;
        voice->sustainPitch = DragPitch;
        voice->sustainMuffle = std::clamp(
            DragMuffle + spatial.additionalMuffle, 0.0f, 0.98f);
        voice->sustainedThisFrame = true;
    }
    m_pendingSustain.clear();

    // 3) Avanca o envelope de cada corpo rastreado e escreve na voz unica.
    // O "teto gradativo" pedido: enquanto o corpo estiver arrastando,
    // sustainTarget e o piso continuo que o envelope nunca cruza pra baixo;
    // um impacto novo so cria um pico temporario por cima, que decai de
    // volta pra esse piso (nao pra zero, se ainda estiver arrastando).
    for (auto it = m_bodyVoices.begin(); it != m_bodyVoices.end();) {
        BodyVoiceState& voice = it->second;
        const float sustainTarget = voice.sustainedThisFrame
            ? voice.sustainTarget : 0.0f;
        advanceEnvelope(voice.envelope, sustainTarget, voice.decayPerSecond,
            deltaTime);

        if (voice.envelope.volume < 0.001f && sustainTarget < 0.001f) {
            voice.silentSeconds += std::max(0.0f, deltaTime);
        } else {
            voice.silentSeconds = 0.0f;
        }
        if (voice.silentSeconds > BodyVoiceSilenceGraceSeconds) {
            m_audio.stopLooping(voice.voiceHandle);
            it = m_bodyVoices.erase(it);
            continue;
        }

        // Mistura o timbre do pico de impacto com o do teto de arrasto
        // conforme o envelope decai de um pra outro - sem essa mistura, o
        // pitch/muffle trocaria de repente assim que o pico terminasse.
        const float spikeFactor = envelopeSpikeFactor(voice.envelope,
            sustainTarget);
        const float pitch = voice.sustainPitch
            + (voice.envelope.spikePitch - voice.sustainPitch) * spikeFactor;
        const float muffle = voice.sustainMuffle
            + (voice.envelope.spikeMuffle - voice.sustainMuffle)
                * spikeFactor;

        m_audio.setLoopingVolume(voice.voiceHandle, voice.envelope.volume);
        m_audio.setLoopingPitch(voice.voiceHandle, pitch);
        m_audio.setLoopingMuffle(voice.voiceHandle, muffle);
        m_audio.setLoopingPosition(voice.voiceHandle, voice.position);
        voice.sustainedThisFrame = false;
        ++it;
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
        // Pitch continua na rampa linear (ver comentario de
        // WindMovementCurveExponent) - so o volume usa a curva mais gradual.
        const float linearRamp = std::clamp(
            (playerSpeed - WindMovementQuietThresholdMetersPerSecond)
                / (WindMovementReferenceSpeedMetersPerSecond
                    - WindMovementQuietThresholdMetersPerSecond),
            0.0f, 1.0f);
        const float volume = windAmbienceVolumeRamp(playerSpeed,
            WindMovementQuietThresholdMetersPerSecond,
            WindMovementReferenceSpeedMetersPerSecond,
            WindMovementCurveExponent) * clampedMasterVolume;
        const float pitch = 1.0f + linearRamp * WindMovementMaxPitchBoost;
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
        const float volume = windAmbienceVolumeRamp(windSpeed,
            WindAmbientQuietThresholdMetersPerSecond,
            WindAmbientReferenceSpeedMetersPerSecond,
            WindAmbientCurveExponent) * clampedMasterVolume;
        const float pitch = 1.0f + ramp * WindAmbientMaxPitchShift;
        m_audio.setLoopingVolume(m_windAmbientVoiceHandle, volume);
        m_audio.setLoopingPitch(m_windAmbientVoiceHandle, pitch);
        m_audio.setLoopingMuffle(m_windAmbientVoiceHandle, WindAmbientMuffle);
    }
}

} // namespace MatterEngine::Workbench
