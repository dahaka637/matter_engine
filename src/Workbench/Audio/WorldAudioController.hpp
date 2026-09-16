#pragma once

#include "Engine/Audio/AcousticEnvironmentSystem3D.hpp"
#include "Engine/Audio/AcousticOcclusion3D.hpp"
#include "Engine/Audio/AcousticZone3D.hpp"
#include "Engine/Audio/AudioDevice3D.hpp"
#include "Engine/Audio/BodyAcousticEnvelope.hpp"
#include "Engine/Audio/DragAcoustics.hpp"
#include "Engine/Audio/ImpactAcoustics.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace MatterEngine::Workbench {

// Faz a ponte entre comandos acústicos em unidades físicas e o mixer. A
// simulação não conhece clips, distância ao ouvinte, ganho por obstrução ou
// atraso de propagação; essas decisões pertencem exclusivamente à
// apresentação sonora. Distância/panorâmica em si já não são calculadas
// aqui: com fontes 3D reais do OpenAL, isso é papel do próprio AudioDevice3D
// a partir da posição/orientação do ouvinte e da posição da fonte.
class WorldAudioController final {
public:
    bool initialize(std::string_view assetsDirectory, bool hrtfEnabled);
    void shutdown();

    // Impacto e arrasto de um MESMO corpo tocam na mesma voz (ver
    // m_bodyVoices/BodyAcousticEnvelope.hpp) - nunca duas vozes
    // independentes disputando o mesmo clip. submitImpacts enfileira o
    // pico (respeitando o atraso de propagacao do som); submitDrags so
    // atualiza o teto de sustain. update() e o unico lugar que de fato
    // decide o volume/pitch final e fala com o AudioDevice3D.
    void submitImpacts(std::span<const ImpactSoundCommand3D> commands);
    void submitDrags(std::span<const DragSoundCommand3D> commands);
    void update(const AudioListenerPose3D& listenerPose,
        const PhysicsScene3D* physicsScene, float deltaTime);

    // Vento continuo no ouvido do jogador (ver WindSystem), como DOIS
    // fenomenos acusticos independentes tocando ao mesmo tempo (ver
    // implementacao para a justificativa de separa-los):
    //   - assobio de movimento: dirigido pela velocidade PROPRIA do
    //     jogador, nao pelo vento - so dispara com deslocamento rapido de
    //     verdade, de forma previsivel.
    //   - uivo ambiente: dirigido pela intensidade do vento em si,
    //     independente do jogador estar se movendo - so audivel quando o
    //     vento esta excepcionalmente forte.
    // Chamado uma vez por quadro, junto de update() - separado dela porque
    // o vento vem de fora da camada de audio (WorkbenchApp possui o
    // WindSystem), enquanto update() só depende do que a propria camada de
    // audio ja rastreia. Quem chama decide o que "vento"/"jogador" significam
    // - inclusive um modo de audicao manual (ver LaboratoryScreen), que
    // substitui as entradas reais por valores sinteticos controlados por
    // slider, sem esta funcao precisar saber que isso esta acontecendo.
    void updateWindAmbience(Vec3 windVelocityMetersPerSecond,
        Vec3 characterVelocityMetersPerSecond, float masterVolume,
        float deltaTime);

    // Zonas autoradas no Blender (entity_type=acoustic_zone) do mapa atual;
    // ver GltfAcousticZone3D.hpp. Chamado uma vez ao carregar o mapa, nao
    // por passo fixo.
    void setAcousticZones(std::vector<AcousticZoneDefinition3D> zones);

    // So pra teste (ver AudioFoundationTests.cpp) - confirma que impacto e
    // arrasto do MESMO corpo nunca abrem duas vozes.
    [[nodiscard]] std::size_t debugActiveBodyVoiceCount() const {
        return m_bodyVoices.size();
    }

private:
    struct PendingSpike {
        std::uint64_t sourceBodyId = 0;
        Vec3 position;
        float volume = 0.0f;
        float pitch = 1.0f;
        float muffle = 0.0f;
        float durationSeconds = 0.18f;
        int clipId = -1;
        float delaySeconds = 0.0f;
    };

    // Teto de sustain pendente pra este quadro (ver submitDrags) - so bruto,
    // ainda sem oclusao/abafamento por distancia (aplicados em update(), que
    // e quem tem acesso a listenerPose/physicsScene).
    struct PendingSustain {
        Vec3 position;
        float intensity = 0.0f;
        int clipId = -1;
    };

    struct BodyVoiceState {
        int voiceHandle = -1;
        Vec3 position;
        BodyAcousticEnvelopeState envelope;
        float decayPerSecond = 3.0f;
        float sustainTarget = 0.0f;
        float sustainPitch = 1.0f;
        float sustainMuffle = 0.0f;
        bool sustainedThisFrame = false;
        float silentSeconds = 0.0f;
    };

    struct SpatialParameters {
        // Multiplicador sobre o volume do impacto (1 = linha livre, <1 =
        // obstruído por geometria estática). Não é a atenuação por
        // distância: essa já é feita pelo próprio modelo 3D do OpenAL.
        float occlusionGain = 1.0f;
        // Aplicado como filtro passa-baixa EFX por voz (AudioDevice3D::play/
        // playTimed); sem efeito audível se efxAvailable() for falso.
        float additionalMuffle = 0.0f;
    };

    [[nodiscard]] int clipForMaterial(const std::string& materialId) const;
    [[nodiscard]] SpatialParameters spatialize(Vec3 source,
        Vec3 listenerPosition, const PhysicsScene3D* physicsScene) const;
    // Acha a voz do corpo em m_bodyVoices, ou cria uma nova (evictando a
    // mais fraca se o pool estiver cheio) - usado tanto por picos de
    // impacto quanto por teto de arrasto, ja que os dois compartilham a
    // mesma voz por corpo agora.
    [[nodiscard]] BodyVoiceState* findOrCreateBodyVoice(
        std::uint64_t sourceBodyId, int clipId, Vec3 position,
        float incomingLevel);

    AudioDevice3D m_audio;
    AcousticEnvironmentSystem3D m_environment;
    std::unordered_map<std::string, int> m_impactClips;
    std::vector<PendingSpike> m_pendingSpikes;
    // Reconstruido do zero a cada submitDrags() - representa "quem esta
    // arrastando AGORA", consumido e limpo por update() no mesmo quadro.
    std::unordered_map<std::uint64_t, PendingSustain> m_pendingSustain;
    // Chave = sourceBodyId (impacto ou arrasto, mesma identidade). Uma unica
    // voz por corpo - ver comentario de submitImpacts/submitDrags.
    std::unordered_map<std::uint64_t, BodyVoiceState> m_bodyVoices;
    bool m_initialized = false;

    int m_windClipId = -1;
    // Handles das duas vozes de loop persistente (ver
    // AudioDevice3D::playLooping) - criadas uma vez em initialize() junto
    // do buffer, moduladas todo quadro por updateWindAmbience().
    int m_windMovementVoiceHandle = -1;
    int m_windAmbientVoiceHandle = -1;
};

} // namespace MatterEngine::Workbench
