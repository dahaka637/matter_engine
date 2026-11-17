#pragma once

#include "Engine/Math/Vec3.hpp"

#include <array>

namespace MatterEngine {

// O planejador de contato (17/11, pedido dele: "o pe ir para onde a bola VAI
// ESTAR no momento do contato... escolher a perna com base nisso, nao apenas
// onde ela esta mas onde ela vai estar e o momento do contato... vale para
// tudo: manejo manual, chute, interceptacao").
//
// Para cada perna e cada instante dos proximos `latest` segundos ele preve
// onde a bola vai estar (voando, quicando, rolando e freando) e onde o corpo
// vai estar (indo para a velocidade que as teclas pedem, girando para o rumo
// pedido), e onde o pe daquela perna estaria na passada (a previsao do estilo
// da locomocao padrao). Escolhe o encontro mais cedo e mais natural: o pe que
// a propria passada leva perto da bola, no balanco, sem cruzar as pernas. O
// gesto entao leva o pe ate a bola naquele instante e a forca sai no encosto.

// A bola daqui a `seconds` (o mesmo integrador do planejador).
[[nodiscard]] Vec3 predictBallPosition3D(Vec3 position, Vec3 velocity, float radius, float groundZ,
    float rollingDeceleration, float seconds);

struct BallContactLeg3D {
    // Andando no ciclo: a previsao do pe (a origem do link, no referencial do
    // corpo daquele instante: x frente, y esquerda) a cada forecastStep s.
    bool cycling = false;
    std::array<Vec3, 16> forecast {};
    float forecastStep = 0.04f;
    // O apoio agora e os tempos do ciclo (s; -1 fora dele).
    bool plantedNow = true;
    float secondsToLiftoff = -1.0f;
    float secondsToTouchdown = -1.0f;
    float swingSeconds = -1.0f;
    float cycleSeconds = -1.0f;
    // O pe agora no referencial do corpo (parado: de onde ele sai).
    Vec3 footNow;
};

struct BallContactQuery3D {
    Vec3 ballPosition;
    Vec3 ballVelocity;
    float ballRadius = 0.11f;
    float rollingDeceleration = 0.5f;
    // O corpo: a pelve, a base dos pes, o rumo agora e o pedido, a velocidade
    // agora e a pedida (ela chega com a constante de tempo).
    Vec3 pelvis;
    float feetBase = 0.0f;
    float yaw = 0.0f;
    float targetYaw = 0.0f;
    float yawRate = 7.0f;
    Vec3 bodyVelocity;
    Vec3 wantedVelocity;
    // A resposta medida da locomocao padrao (17/11): a rapidez vai para a
    // pedida com esta constante de tempo (s) e o rumo do movimento gira para o
    // pedido nesta taxa (rad/s) - a 4,5 m/s ela vira 90 graus em ~0,2 s sem
    // perder rapidez e para do sprint em ~0,4 s.
    float velocityTimeConstant = 0.25f;
    float headingRate = 10.0f;
    std::array<BallContactLeg3D, 2> legs {};
    // A janela do contato (s): o gesto mais rapido ate o contato e o mais
    // longe que se planeja.
    float earliest = 0.06f;
    float latest = 0.6f;
    // O quanto o pe pode sair da passada para chegar na bola (m) e a bola mais
    // alta para o pe (sobre a base dos pes).
    float maxDeviation = 0.42f;
    float highestBall = 0.70f;
    // So uma perna (-1: as duas).
    int onlySide = -1;
    // O peso do tempo no custo (o pedido do jogador: o encontro mais cedo pesa
    // mais - a resposta na hora).
    float timeWeight = 1.0f;
    // O instante natural do contato do gesto (s): mais cedo que ele, o gesto se
    // comprime (mais rapido) - possivel, mas custa.
    float naturalContact = 0.0f;
    // A rapidez media do pe saindo da passada ate a bola (m/s; 0: sem): o gesto
    // precisa ainda de desvio/footSpeed alem do naturalContact (o toque parado
    // para tras ia 55 cm em 0,07 s e a canela girava a 50 rad/s).
    float footSpeed = 0.0f;
};

struct BallContactPlan3D {
    bool valid = false;
    int side = 0;
    float seconds = 0.0f;
    // A bola no contato: no mundo e no referencial do corpo daquele instante
    // (x frente, y esquerda, z sobre a base dos pes).
    Vec3 ballWorld;
    Vec3 ballInBody;
    // O quanto o pe sai da passada (m) e o custo da escolha.
    float deviation = 0.0f;
    float cost = 0.0f;
};

[[nodiscard]] BallContactPlan3D planBallContact3D(const BallContactQuery3D& query);

// O corpo daqui a `seconds` pelo mesmo modelo (a posicao em relacao a de agora e
// a velocidade): a rapidez para a pedida, o rumo girando para o pedido.
struct PredictedBody3D {
    Vec3 travel;
    Vec3 velocity;
};
[[nodiscard]] PredictedBody3D predictBodyMotion3D(Vec3 velocity, Vec3 wanted, float timeConstant, float headingRate,
    float seconds);

} // namespace MatterEngine
