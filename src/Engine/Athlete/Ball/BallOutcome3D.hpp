#pragma once

#include "Engine/Math/Vec3.hpp"

#include <cstdint>

namespace MatterEngine {

// O pedido de bola e o resultado aprovado (rework do manejo, 10/10/2026,
// Etapa 5). O resultado e o modelo do golpe com atrito do V3 (keepBallMotion e
// o toque pela barra), com os valores da aba "Forca na bola"; o que muda e
// quem o chama: o encosto de verdade do pe na bola (o sensor), uma vez, como
// impulso - sem impor a velocidade e sem segurar a bola.
enum class BallActionKind3D : std::uint8_t { None, Dribble, Shot };
// A parte do pe que toca (o calcanhar: a bola atras tocada para tras).
enum class BallTouchSurface3D : std::uint8_t { Laces, Inside, Outside, Sole, Heel };

// A forca na bola (07/10, pedido do usuario: uma aba no laboratorio para
// ajustar tudo, o toque e o chute): os padroes sao os aprovados (T_Ball* e
// T_Shot* no ambiente mudam os padroes).
struct BallOutcomeSettings3D {
    // Toque (manejo): a velocidade que a barra da (m/s) = minimo + forca x
    // barra^curva (curva > 1: fraca embaixo e forte em cima; 1: reta).
    float touchMinimum = 0.2f;
    float touchPower = 13.0f;
    float touchCurve = 2.65f;
    // A sola (puxar e rolar): a mesma conta.
    float soleMinimum = 0.4f;
    float solePower = 2.6f;
    float soleCurve = 2.2f;
    // A velocidade do jogador: quanto da dele no rumo do toque a bola leva, a
    // antecipacao dela (s; so no corpo animado), o empurrao a mais por m/s do
    // corpo e o quanto da velocidade contra o rumo do toque - a do corpo e a
    // que a bola trazia - volta nela.
    float carry = 1.0f;
    float carryAhead = 0.05f;
    float speedPush = 0.05f;
    float reverseBoost = 0.0f;
    // O atrito do toque: o que fica do movimento de lado da bola (em relacao
    // ao corpo) e do excesso dela ja mais rapida no rumo do toque.
    float touchSideKeep = 0.4f;
    float touchAheadKeep = 0.3f;
    // Chute: a velocidade (m/s) = minimo + forca x barra^curva.
    float shotMinimum = 6.0f;
    float shotPower = 24.0f;
    float shotCurve = 0.6f;
    // A altura (graus): com a barra vazia, a mais com ela cheia, e os extremos
    // do gesto do mouse (para cima a bola alta, para baixo a rasteira).
    float shotElevationMinimum = 4.0f;
    float shotElevationPower = 8.0f;
    float shotHighElevation = 38.0f;
    float shotLowElevation = 0.5f;
    // O efeito: giro de lado no gesto cheio (rad/s), quanto a bola sai para
    // fora da mira para a curva voltar nela (graus), giro para tras na alta e
    // por cima na rasteira (rad/s).
    float shotCurveSpin = 55.0f;
    float shotCurveLaunch = 5.0f;
    float shotBackSpin = 25.0f;
    float shotTopSpin = 20.0f;
    // O atrito do chute: o que fica do movimento de lado da bola, o quique do
    // que ela trazia contra o chute e o excesso dela ja mais rapida.
    float shotSideKeep = 0.4f;
    float shotBounce = 0.3f;
    float shotAheadKeep = 0.3f;
    // Chute correndo: quanto o corpo freia (0..1 da velocidade).
    float kickRunSlowdown = 0.3f;
    // Depois do toque ou do chute: quanto tempo a bola fica na velocidade dele
    // (o pe ainda encostando nao a muda; s).
    float holdSeconds = 0.12f;
    // A hitbox do pe: a caixa dele aumentada disto em cada lado (m; no corpo
    // fisico, cujo pe atrasa o alvo, vezes 1,6).
    float hitboxMargin = 0.07f;
};

// O que o pedido quer da bola.
struct BallOutcomeRequest3D {
    BallActionKind3D kind = BallActionKind3D::Dribble;
    // A barra (0..1) e o rumo no plano (drible: as teclas, ou o olhar sem
    // tecla).
    float power = 0.0f;
    Vec3 direction { 1.0f, 0.0f, 0.0f };
    // Chute: a mira (rad) e o gesto do mouse (altura e efeito, -1..1).
    float aimYawRadians = 0.0f;
    float lift = 0.0f;
    float curve = 0.0f;
    // A sola (puxar e rolar) usa a conta dela.
    BallTouchSurface3D surface = BallTouchSurface3D::Inside;
    // Drible com a velocidade exata (m/s; negativa: a da barra).
    float touchSpeed = -1.0f;
};

struct BallOutcome3D {
    Vec3 velocity;
    Vec3 spin;
};

// A velocidade e o giro da bola depois do toque ou do chute: a definida pela
// barra (e pela velocidade do corpo, no toque) mais o que fica da que a bola
// trazia. bodyVelocity: a do corpo agora; aheadVelocity: a que ele vai ter
// (o toque leva a maior das duas no rumo dele).
[[nodiscard]] BallOutcome3D computeBallOutcome3D(const BallOutcomeRequest3D& request, Vec3 ballVelocity,
    Vec3 bodyVelocity, Vec3 aheadVelocity, float ballRadius, const BallOutcomeSettings3D& settings);

// A velocidade do golpe que fica da que a bola trazia (o atrito: de lado, o
// quique contra e o excesso no rumo).
[[nodiscard]] Vec3 keepBallMotion3D(Vec3 defined, Vec3 ballVelocity, float sideKeep, float bounce, float aheadKeep);

} // namespace MatterEngine
