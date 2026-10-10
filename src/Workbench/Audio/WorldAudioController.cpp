#include "Workbench/Audio/WorldAudioController.hpp"

#include "Engine/Audio/ProceduralNoise.hpp"
#include "Engine/Audio/WindAmbienceRamp.hpp"
#include "Engine/Audio/WindSound.hpp"
#include "Engine/Core/Log.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace MatterEngine::Workbench {
namespace {

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

bool WorldAudioController::initialize(bool hrtfEnabled) {
    if (m_initialized) return true;
    AudioDevice3D::Settings settings;
    settings.hrtfEnabled = hrtfEnabled;
    if (!m_audio.initialize(settings)) return false;
    if (hrtfEnabled && !m_audio.hrtfActive()) {
        Log::warn("HRTF foi pedido mas o OpenAL Soft nao ativou "
            "(dispositivo sem suporte); audio binaural desligado.");
    }
    // Vento (ver WindSystem) - ruido gerado por codigo (ProceduralNoise.hpp),
    // nao um arquivo gravado; ver o comentario junto das constantes
    // WindMovement*/WindAmbient* acima para a divisao em duas vozes. Ambas
    // tocam em loop com ganho zero desde o inicio (em vez de iniciar/parar o
    // loop sob demanda), para nunca reiniciar a posicao de reproducao e
    // produzir um "corte" audivel toda vez que o vento cruza o limiar de ser
    // ouvido. Ruido rosa moldado por um filtro passa-baixa ressonante com
    // corte modulado por um LFO lento (ver WindSound.hpp).
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
    m_audio.shutdown();
    m_initialized = false;
}

void WorldAudioController::setAcousticZones(
    std::vector<AcousticZoneDefinition3D> zones) {
    m_environment.setZones(std::move(zones));
}

void WorldAudioController::update(const AudioListenerPose3D& listenerPose) {
    if (!m_initialized) return;
    m_audio.setListenerPose(listenerPose);
    m_audio.setEnvironment(m_environment.evaluate(listenerPose.position));
}

void WorldAudioController::updateWindAmbience(
    Vec3 windVelocityMetersPerSecond, Vec3 characterVelocityMetersPerSecond,
    float masterVolume) {
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
