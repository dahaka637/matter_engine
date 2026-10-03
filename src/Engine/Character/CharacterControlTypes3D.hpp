#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"

#include <array>
#include <cstdint>

namespace MatterEngine {

// Intencao do jogador (ou da IA) para um personagem neste tick. E o pedido
// puro: nunca e sobrescrita pelo que a capsula/proxy conseguiu andar, nem
// apagada porque o corpo perdeu o equilibrio, caiu ou esta levantando - o
// corpo pode executar so parte dela, mas ela continua existindo.
struct CharacterIntent3D {
    // Velocidade pedida no plano (m/s): direcao das teclas x velocidade do
    // passo naquela direcao (ja com o limite do terreno a frente).
    Vec3 requestedVelocityWorld;
    // Direcao pedida (unitaria) ou zero.
    Vec3 requestedDirectionWorld;
    float requestedFacingYawRadians = 0.0f;
    float lookPitchRadians = 0.0f;
    // Sprint/prontidao, 0..1.
    float effort = 0.0f;
    bool crouch = false;
    // Segurado pela PhysGun.
    bool manipulated = false;
};

// De onde vem a informacao de um apoio: so geometria (sola rente ao chao sem
// evento de contato), evento de contato com impulso estimado antes do solver
// (Jolt) ou impulso resolvido pelo solver (PhysX). Decisoes sensiveis a carga
// precisam saber qual das tres usaram.
enum class ContactEvidence3D : std::uint8_t {
    None,
    Geometric,
    EstimatedImpulse,
    SolvedImpulse
};

// Apoio medido de um pe. Nunca vem do plano de passos: um pe "travado" pelo
// planejador pode estar no ar, e so o que o corpo toca conta aqui.
struct FootSupportEstimate3D {
    std::uint32_t linkIndex = RagdollDynamics3D::InvalidIndex;
    // Ha contato com alguma superficie (qualquer normal).
    bool touching = false;
    // O contato consegue sustentar peso (normal para cima) - ou a sola esta
    // rente ao chao sondado, parada (evidencia geometrica).
    bool supporting = false;
    ContactEvidence3D evidence = ContactEvidence3D::None;
    // Sem evento de contato neste tick, mas com um ha menos de 50 ms e a sola
    // parada: os eventos de um pe com pouca carga piscam tick sim, tick nao.
    // Memoria so para medir/planejar - nunca autoriza forca de sustentacao.
    bool heldFromMemory = false;
    // Apoio observado estrito (E5, R1 do pacote v2): contato com normal para
    // cima neste tick ou ha no maximo 2 ticks (so o pisca do evento) - sem a
    // memoria de 50 ms nem a evidencia geometrica. O que a marcha usa para
    // dizer se o pe esta no chao.
    bool contactObserved = false;
    // Ha quanto tempo o pe esta apoiado sem interrupcao (s; 0 sem apoio).
    float supportingSeconds = 0.0f;
    // Apoiado em corpo movel (caixa, outro personagem).
    bool onDynamicBody = false;
    // Cantos da sola (mundo) e o ponto mais baixo dela.
    std::array<Vec3, 4> soleCornersWorld {};
    Vec3 soleCenterWorld;
    float soleLowestHeight = 0.0f;
    // Velocidade do centro da sola (mundo): escorregar/arrastar.
    Vec3 soleVelocityWorld;
    // Carga normal aproximada (N) - estimativa, ver `evidence`.
    float loadNewtons = 0.0f;
};

// Estado fisico medido de um personagem no tick concluido. Mesmo calculo para
// o personagem controlado e para os bonecos soltos: nao depende de
// RagdollDynamics3D.
struct CharacterPhysicalState3D {
    bool valid = false;
    float massKilograms = 0.0f;
    // Centro de massa pela massa de cada link; velocidade pelas velocidades
    // fisicas de cada link (publicadas no centro de massa dele).
    Vec3 centerOfMassWorld;
    Vec3 centerOfMassVelocityWorld;
    // Aceleracao OBSERVADA do centro de massa (diferenca da velocidade entre
    // ticks): crua (mostra impactos) e filtrada (~40 ms). Com gravidade.
    Vec3 centerOfMassAccelerationRawWorld;
    Vec3 centerOfMassAccelerationWorld;
    Vec3 rootPositionWorld;
    Quaternion rootOrientationWorld;
    Vec3 rootLinearVelocityWorld;
    Vec3 rootAngularVelocityWorld;
    // Pelve contra a vertical.
    float rootUpright = 1.0f;
    float rootTiltRadians = 0.0f;
    std::array<FootSupportEstimate3D, 2> feet {};
    // Apoio atual no plano: cantos das solas que sustentam.
    std::array<Vec3, 8> supportPointsWorld {};
    std::uint32_t supportPointCount = 0;
    bool supported = false;
    // Altura do apoio (a sola mais baixa entre as que sustentam).
    float supportHeight = 0.0f;
    // Ponto de captura de altura constante (heuristica de apoio, nao a
    // fronteira exata): centro de massa + v/omega, e quanto (m) e para onde
    // (unitario, do centro do apoio) ele sai do apoio atual.
    Vec3 capturePointWorld;
    float captureOmega = 3.2f;
    float captureOutsideMeters = 0.0f;
    Vec3 captureDirectionWorld;
    // Forca externa (N, no plano) dos contatos no tronco e nos bracos -
    // pernas nao: tropecar numa caixa nao e empurrao. Limitada por tick e
    // filtrada (~0,1 s). No Jolt e sempre estimativa (impulso antes do
    // solver).
    Vec3 externalForceWorld;
    ContactEvidence3D externalForceEvidence = ContactEvidence3D::None;
    // Outro segmento que nao os pes (joelho/canela, mao, tronco) apoiado em
    // algo por baixo (normal para cima), seja chao ou objeto.
    bool nonFootGroundContact = false;
};

} // namespace MatterEngine
