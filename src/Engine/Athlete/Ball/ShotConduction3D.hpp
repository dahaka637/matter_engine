#pragma once

#include "Engine/Athlete/Ball/BallOutcome3D.hpp"
#include "Engine/Math/Vec3.hpp"

namespace MatterEngine {

// A conducao do chute (28/10, pedido do usuario: "enquanto a bola ainda esta
// na area de alcance, pudesse ser redirecionada e aplicada efeito apos o
// chute, como se o pe ainda estivesse na bola"). A forca principal sai inteira
// no comando; depois, a cada tick com a bola chutada no alcance dos pes, o
// olhar de agora contra o do chute gira a velocidade dela (ate o limite do
// chute, menor quanto mais forte) e poe giro de lado no mesmo sentido. Sem o
// olhar mudar, nada muda; fora do alcance, quem chama a encerra. Como o chute
// sai do alcance em 2 a 5 ticks, quem conduz e o mouse ja em movimento ao
// soltar (o "flick").
struct ShotConduction3D {
    float lookYaw = 0.0f;  // o olhar no chute (rad)
    float limit = 0.0f;    // o giro maximo deste chute (rad)
    float turned = 0.0f;   // o ja girado (rad)
};

// No tick do chute de pe: o olhar e a barra (0..1).
[[nodiscard]] ShotConduction3D startShotConduction3D(float lookYaw, float bar, const BallOutcomeSettings3D& force);

// Um tick com a bola no alcance: a velocidade e o giro que ela passa a ter.
struct ShotConductionStep3D {
    Vec3 velocity;
    Vec3 spin;
};
[[nodiscard]] ShotConductionStep3D stepShotConduction3D(ShotConduction3D& conduction, Vec3 velocity, Vec3 spin,
    float lookYaw, const BallOutcomeSettings3D& force);

} // namespace MatterEngine
