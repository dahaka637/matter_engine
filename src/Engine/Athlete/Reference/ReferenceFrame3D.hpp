#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Athlete/Reference/BodyForecast3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <array>
#include <vector>

namespace MatterEngine {

// A referencia de um tick (a reescrita da locomocao, R4; vault
// 04_locomocao/reescrita_da_locomocao.md): o que o executor fisico e o RL leem
// do movimento - nunca a trajetoria exata do padrao (os pes dele deslizam, a
// inclinacao e so visual, a capsula acelera a 16-30 m/s2), e sim:
// - a pose de estilo (alvo suave);
// - a fase e, por pe, o apoio e a meta de contato (quando e onde pousa);
// - a intencao e o plano de navegacao a frente, por um perfil que o corpo
//   consegue cumprir (o do fisico tem limites menores que o da capsula).

// O movimento que a referencia le (so dados): a saida de quem gera o movimento
// - hoje o padrao, convertido pela casca de migracao (Athlete/Legacy/
// LegacyReference3D); a referencia nao conhece o controle antigo.
struct ReferenceMotion3D {
    static constexpr int ForecastCount = 16;
    bool valid = false;
    // A pose (sem copia: vale ate o proximo update de quem a gerou).
    const RagdollAnimationPose3D* pose = nullptr;
    const std::vector<Vec3>* coordinates = nullptr;
    Vec3 rootPosition;
    Quaternion rootOrientation;
    float facingYaw = 0.0f;
    float phase = 0.0f;
    float movingShare = 0.0f;
    float cycleSeconds = -1.0f;
    bool cycling = false;
    bool airborne = false;
    // Os pes: o apoio pela fase (andando) e pelo contato (parado), o tempo ate
    // sair e ate pousar (s; -1 fora).
    std::array<bool, 2> cyclePlanted { true, true };
    std::array<bool, 2> footPlanted { true, true };
    std::array<float, 2> secondsToLiftoff { -1.0f, -1.0f };
    std::array<float, 2> cycleSecondsToTouchdown { -1.0f, -1.0f };
    std::array<float, 2> secondsToTouchdown { -1.0f, -1.0f };
    // O pe do estilo no futuro (no referencial do corpo em cada instante), a
    // cada forecastStep s a partir de agora.
    float forecastStep = 0.04f;
    const std::array<std::array<Vec3, ForecastCount>, 2>* footForecast = nullptr;
};

// O perfil de navegacao (a resposta da capsula: m/s2, rad/s). O padrao e o
// animado (os aprovados, 100% da regua "Resposta").
struct NavigationProfile3D {
    float acceleration = 16.0f;
    float deceleration = 22.0f;
    float brake = 30.0f;
    float turnRate = 10.0f;

    [[nodiscard]] bool valid() const;
};

struct ReferenceFootGoal3D {
    // Apoiado pela fase da marcha (parado: os dois).
    bool planted = true;
    // Quanto falta para sair e para pousar (s; -1 fora do ciclo).
    float secondsToLiftoff = -1.0f;
    float secondsToTouchdown = -1.0f;
    // Onde o pe do estilo pousa (o link do pe, no mundo), pela previsao do
    // pe e do corpo; so no balanco, dentro do horizonte da previsao.
    bool touchdownValid = false;
    Vec3 touchdownWorld;
};

struct ReferenceFrame3D {
    static constexpr int PlanCount = 3;

    bool valid = false;
    // A pose de estilo (a do padrao, sem copia: vale ate o proximo update
    // dele) e o rumo do corpo.
    const RagdollAnimationPose3D* pose = nullptr;
    const std::vector<Vec3>* coordinates = nullptr;
    Vec3 rootPosition;
    Quaternion rootOrientation;
    float facingYaw = 0.0f;
    // A fase (0..1; 0 = pouso do pe esquerdo), o quanto anda e o ciclo (s).
    float phase = 0.0f;
    float movingShare = 0.0f;
    bool cycling = false;
    float cycleSeconds = -1.0f;
    bool airborne = false;
    std::array<ReferenceFootGoal3D, 2> feet {};
    // A intencao (as teclas, no plano), a velocidade de agora e o plano a
    // frente (a pelve no chao e a velocidade nos instantes planTimes).
    Vec3 requestedVelocity;
    Vec3 velocity;
    std::array<float, PlanCount> planTimes { 0.1f, 0.25f, 0.5f };
    std::array<Vec3, PlanCount> planPosition {};
    std::array<Vec3, PlanCount> planVelocity {};
};

struct ReferenceFrameInput3D {
    // A pelve no chao (x, y da pelve; z do chao), a velocidade no plano, as
    // teclas (no plano) e o rumo do corpo e quanto ele gira.
    Vec3 pelvisGround;
    Vec3 velocity;
    Vec3 requested;
    float facingYaw = 0.0f;
    float facingRate = 0.0f;
    // O perfil de navegacao do plano a frente.
    NavigationProfile3D profile;
    // Os quadris no referencial do corpo (como o BodyForecast3D os quer).
    std::array<Vec3, 2> hipInBody {};
};

// Monta a referencia a partir do movimento (hoje o padrao) e do estado da
// capsula. A previsao do corpo vai em `forecast` (reusada entre
// ticks, sem alocar de novo).
[[nodiscard]] ReferenceFrame3D buildReferenceFrame3D(const ReferenceMotion3D& motion,
    const ReferenceFrameInput3D& input, BodyForecast3D& forecast);

} // namespace MatterEngine
