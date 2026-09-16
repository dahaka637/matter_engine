#pragma once

namespace MatterEngine {

// Fracao continua [0,1] de intensidade entre quietThreshold (0) e
// referenceSpeed (1), moldada por curveExponent: 1.0 = linear, 2.0 =
// quadratica (aproxima ruido aerodinamico real, que cresce ~v^2 - usada pelo
// uivo ambiente do vento), >2.0 = sobe mais devagar no meio da faixa e
// acelera perto do topo (usada pelo assobio de movimento, pra evitar que a
// rampa pareca "quase no talo" bem antes da velocidade de referencia).
// Extraida de WorldAudioController::updateWindAmbience como funcao pura
// para poder ser testada sem nenhum AudioDevice3D real (mesmo espirito de
// AcousticOcclusion3D::computeOcclusionFactor3D - cada efeito sonoro ganha
// sua propria funcao pequena e focada em vez de crescer um calculo
// monolitico dentro do controlador).
[[nodiscard]] float windAmbienceVolumeRamp(float speed, float quietThreshold,
    float referenceSpeed, float curveExponent);

} // namespace MatterEngine
