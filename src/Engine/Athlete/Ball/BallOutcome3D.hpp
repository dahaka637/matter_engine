#pragma once

#include "Engine/Math/Vec3.hpp"

#include <cstdint>

namespace MatterEngine {

// O pedido de bola e o resultado (a fisica do toque e do chute, nao o gesto):
// o golpe com atrito (keepBallMotion) e o toque pela barra, com os valores da
// aba "Forca na bola". Desde 24/10 a forca sai no comando, de uma vez, como
// impulso (os gestos sao so animacao: Engine/Gesture).
// (Raise: levantar a bola - o botao do meio, 24/10.)
enum class BallActionKind3D : std::uint8_t { None, Dribble, Shot, Raise };
// A parte do pe que toca (o calcanhar: a bola atras tocada para tras).
enum class BallTouchSurface3D : std::uint8_t { Laces, Inside, Outside, Sole, Heel };
// A parte do corpo (24/10, pedido do usuario): os pes, o torax (o dominio no
// peito) e a cabeca (o cabeceio).
enum class BallBodyPart3D : std::uint8_t { Foot, Chest, Head };

// A forca na bola (07/10, pedido do usuario: uma aba no laboratorio para
// ajustar tudo, o toque e o chute): os padroes sao os aprovados (T_Ball* e
// T_Shot* no ambiente mudam os padroes).
struct BallOutcomeSettings3D {
    // Toque (manejo): a velocidade que a barra da (m/s) = minimo + forca x
    // barra^curva (curva > 1: fraca embaixo e forte em cima; 1: reta).
    // (23/10: a calibragem do usuario na aba.)
    float touchMinimum = 0.08f;
    float touchPower = 15.30f;
    float touchCurve = 2.35f;
    // A velocidade do jogador: quanto da dele no rumo do toque a bola leva e o
    // empurrao a mais por m/s do corpo.
    float carry = 0.74f;
    float speedPush = 0.3f;
    // O poder de parada (24/10, pedido do usuario; era a "forca para inverter
    // o rumo"): a bola vindo contra o manejo, o toque mais fraco que ela corta
    // esta fracao do que sobraria dela seguindo (1: para no pe; 0: segue como
    // antes); o mais forte inverte o rumo com o que sobra. No chute fraco, o
    // quique vira amortecer pela barra.
    float stopPower = 0.8f;
    // O atrito do toque: o que fica do movimento de lado da bola (em relacao
    // ao corpo), do excesso dela ja mais rapida no rumo do toque e do que ela
    // trazia contra o rumo (20/10, pedido do usuario: tocar contra a bola
    // amortece - sai a pedida menos a que vinha; 1 = impulso puro).
    float touchSideKeep = 0.4f;
    float touchAheadKeep = 0.3f;
    float touchAgainstKeep = 1.0f;
    // Chute: a velocidade (m/s) = minimo + forca x barra^curva.
    float shotMinimum = 4.0f;
    float shotPower = 38.0f;
    float shotCurve = 0.77f;
    // A altura (graus): com a barra vazia, a mais com ela cheia, e os extremos
    // do gesto do mouse (para cima a bola alta, para baixo a rasteira).
    float shotElevationMinimum = 4.5f;
    float shotElevationPower = 4.5f;
    float shotHighElevation = 38.0f;
    float shotLowElevation = 0.5f;
    // O efeito: giro de lado no gesto cheio (rad/s), quanto a bola sai para
    // fora da mira para a curva voltar nela (graus), giro para tras na alta e
    // por cima na rasteira (rad/s).
    float shotCurveSpin = 120.0f;
    float shotCurveLaunch = 5.0f;
    float shotBackSpin = 25.0f;
    float shotTopSpin = 25.0f;
    // O atrito do chute: o que fica do movimento de lado da bola, o quique do
    // que ela trazia contra o chute e o excesso dela ja mais rapida.
    float shotSideKeep = 0.4f;
    float shotBounce = 0.3f;
    float shotAheadKeep = 0.3f;
    // A recepcao (24/10, pedido do usuario: o toque ou o chute fraco na bola
    // vindo para ele a param junto do corpo): o que fica no rumo das teclas
    // (m/s) = minimo + forca x barra, sobre a velocidade do corpo (sem tecla,
    // nada); e o quanto ela precisa vir no rumo dele (m/s) para ser recepcao.
    float receiveMinimum = 0.3f;
    float receivePower = 1.6f;
    float receiveClosing = 2.5f;
    // Quanto da velocidade do jogador a bola recebida herda (25/10, pedido do
    // usuario: com a tecla apertada ela saia com a velocidade inteira do corpo
    // e, soltando a tecla, voltava com forca; 0: ela para no chao e sai so o
    // resto; 1: o de antes).
    float receiveCarry = 0.0f;
    // A recepcao espera a bola chegar: apertado com ela ainda longe, o pedido
    // fica armado ate isto (s), e a forca sai com ela no alcance e a
    // receiveArmDistance da pelve (m, no plano: onde o gesto pronto a para -
    // parando onde estava no aperto, ela ficava ate meio metro a frente do
    // pe), ou quando ela para de vir (passando de lado).
    float receiveArmSeconds = 0.3f;
    float receiveArmDistance = 0.45f;
    // O peito e a cabeca (24/10, pedido do usuario), pelo maximo de agora do
    // toque e do chute (minimo + forca): o peito vai ate chestShare dele e
    // para a bola - tira chestStop do que ela trazia contra o corpo (1: o
    // dominio perfeito; o jogador bom, depois) -; a cabeca e o toque e o
    // chute de sempre ate headShare do maximo (o minimo igual).
    float chestShare = 0.25f;
    float chestStop = 1.0f;
    float headShare = 0.5f;
    // Levantar a bola (24/10, pedido do usuario: o botao do meio, a barra
    // amarela): sobe a velocidade da barra (m/s) = minimo + forca x
    // barra^curva; no rumo das teclas sai raiseAhead dela a frente (o chapeu),
    // sobre a velocidade do corpo; sem tecla, reto para cima.
    float raiseMinimum = 2.5f;
    float raisePower = 5.0f;
    float raiseCurve = 1.0f;
    float raiseAhead = 0.5f;
};

// Os padroes (os de cima; T_Ball* e T_Shot* no ambiente os mudam).
[[nodiscard]] BallOutcomeSettings3D defaultBallOutcomeSettings3D();

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
    // A parte do pe (o gesto que a mostra).
    BallTouchSurface3D surface = BallTouchSurface3D::Inside;
    // Drible com a velocidade exata (m/s; negativa: a da barra).
    float touchSpeed = -1.0f;
    // A recepcao: a bola para junto do corpo (so o resto no rumo; sem rumo,
    // nada).
    bool receive = false;
    BallBodyPart3D part = BallBodyPart3D::Foot;
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
