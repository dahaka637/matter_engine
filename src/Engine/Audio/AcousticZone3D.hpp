#pragma once

#include "Engine/Math/Vec3.hpp"

#include <cstdint>
#include <string>

namespace MatterEngine {

// Cada valor mapeia para um preset de reverb EAX/EFX curado (ver
// AudioDevice3D.cpp) em vez de parametros inventados a mao.
enum class AcousticReverbPreset3D : std::uint8_t {
    Generic,
    SmallRoom,
    Hallway,
    Cave,
    Outdoor,
    Warehouse
};

// Autorado no Blender como um empty com entity_type=acoustic_zone (ver
// parseAcousticZones em Engine/Geometry/GltfAcousticZone3D.hpp - o parsing
// fica junto do GltfLoader, nao aqui, para MatterAudio nao precisar linkar
// cgltf/stb so por causa de GltfExtras::find()). O raio e a distancia de
// transicao sao campos explicitos (nao a escala do empty) porque
// LoadedGltfEntity nao carrega escala.
struct AcousticZoneDefinition3D {
    std::string name;
    Vec3 center;
    // Dentro deste raio, a zona contribui com peso maximo (1.0).
    float radiusMeters = 8.0f;
    // Alem do raio, o peso cai linearmente ate 0 nesta distancia extra -
    // e o que evita a zona ligar/desligar de golpe na borda.
    float blendDistanceMeters = 3.0f;
    AcousticReverbPreset3D preset = AcousticReverbPreset3D::Generic;
    // Intensidade autoral do reverb desta zona especifica (0..1); zonas
    // sobrepostas ainda assim nunca somam acima de 1 no resultado final
    // (ver AcousticEnvironmentSystem3D::evaluate).
    float wetSendGain = 1.0f;
};

} // namespace MatterEngine
