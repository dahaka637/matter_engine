#pragma once

#include "Engine/Audio/AcousticEnvironmentSystem3D.hpp"
#include "Engine/Audio/AcousticZone3D.hpp"
#include "Engine/Audio/AudioDevice3D.hpp"

#include <string_view>
#include <vector>

namespace MatterEngine::Workbench {

// O audio do mundo: o ouvinte, o ambiente acustico das zonas do mapa e o
// vento. Os sons de impacto e de arrasto (arquivos WAV por material) sairam
// em 09/10 a pedido do usuario: o sistema de som sera refeito, procedural
// por receita, sem arquivos de audio. Distancia e panoramica sao do modelo 3D
// do OpenAL (AudioDevice3D).
class WorldAudioController final {
public:
    bool initialize(bool hrtfEnabled);
    void shutdown();

    // Uma vez por quadro: o ouvinte e o ambiente da zona em que ele esta.
    void update(const AudioListenerPose3D& listenerPose);

    // Vento continuo no ouvido do jogador (ver WindSystem), como DOIS
    // fenomenos acusticos independentes tocando ao mesmo tempo:
    //   - assobio de movimento: dirigido pela velocidade PROPRIA do
    //     jogador, nao pelo vento - so dispara com deslocamento rapido de
    //     verdade, de forma previsivel;
    //   - uivo ambiente: dirigido pela intensidade do vento em si,
    //     independente do jogador estar se movendo - so audivel quando o
    //     vento esta forte.
    // Quem chama decide o que "vento"/"jogador" significam - inclusive um
    // modo de audicao manual (ver LaboratoryScreen), que substitui as
    // entradas reais por valores sinteticos controlados por slider.
    void updateWindAmbience(Vec3 windVelocityMetersPerSecond,
        Vec3 characterVelocityMetersPerSecond, float masterVolume);

    // Zonas autoradas no Blender (entity_type=acoustic_zone) do mapa atual;
    // ver GltfAcousticZone3D.hpp. Chamado uma vez ao carregar o mapa.
    void setAcousticZones(std::vector<AcousticZoneDefinition3D> zones);

private:
    AudioDevice3D m_audio;
    AcousticEnvironmentSystem3D m_environment;
    bool m_initialized = false;
    int m_windClipId = -1;
    // As duas vozes de loop persistente (ver AudioDevice3D::playLooping),
    // criadas uma vez em initialize() junto do buffer e moduladas todo
    // quadro por updateWindAmbience().
    int m_windMovementVoiceHandle = -1;
    int m_windAmbientVoiceHandle = -1;
};

} // namespace MatterEngine::Workbench
