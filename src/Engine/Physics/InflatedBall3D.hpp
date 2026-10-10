#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <string>

namespace MatterEngine {

// Bola inflada (a bola de futebol): um corpo deformavel de parametros
// concentrados. O movimento e de corpo rigido (posicao, orientacao,
// velocidades) e os contatos sao complacentes, calculados pela fisica de uma
// casca inflada e integrados em subpassos dentro do tick:
//
//  - compressao de cada contato = o quanto a esfera de repouso entra no
//    outro corpo;
//  - ar interno adiabatico: o volume perdido pelas calotas comprimidas sobe a
//    pressao (P_abs = P_abs0 (V0/V)^gama), e a forca normal e a pressao
//    manometrica sobre a area da mancha achatada, mais a rigidez da casca;
//  - amortecimento com razao fixa sobre a rigidez instantanea (histerese da
//    casca), calibrado pela queda da FIFA;
//  - atrito de Coulomb na mancha (o giro sai de contato fora do centro) e
//    resistencia ao rolar;
//  - no ar: arrasto com a crise (Reynolds), Magnus pelo giro e decaimento do
//    giro.
//
// O soft body de particulas do Jolt foi medido e nao serve: com a pressao de
// uma bola real (0,8 bar) a membrana de 642 particulas explode em menos de
// meio segundo, com qualquer numero de iteracoes (pressao explicita por
// subpasso contra restricoes de aresta tangentes; ver o projeto em
// brain/30_Projects/matter_engine/Bola_Procedural.md).

struct InflatedBallHandle3D {
    std::uint32_t index = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t generation = 0;

    [[nodiscard]] constexpr bool valid() const {
        return index != std::numeric_limits<std::uint32_t>::max();
    }
    constexpr explicit operator bool() const { return valid(); }
    constexpr bool operator==(const InflatedBallHandle3D&) const = default;
};

// Propriedades da bola (SI). Os padroes sao a bola tamanho 5 do FIFA Quality
// Pro: circunferencia 68,5-69,5 cm, 420-445 g, testada a 0,8 bar; solta de 2
// m sobre base rigida quica 135-155 cm (a calibracao da 1,39 m, perto da bola
// de teste de superficies da FIFA, 1,35 m). A rigidez de um contato (~54 N/mm
// a 1 cm) e a de pressao x mancha; ensaios entre duas placas medem metade
// (22-28 N/mm). O que a superficie faz com a bola (gramado absorve, freia o
// rolar) esta no material (BallSurfaceProperties). Arrasto e Magnus de Asai
// et al. (2007).
struct InflatedBallProfile3D {
    float radiusMeters = 0.1098f;
    float massKg = 0.43f;
    // Pressao manometrica (acima da atmosfera) e a atmosfera.
    float gaugePressurePascals = 80000.0f;
    float atmosphericPressurePascals = 101325.0f;
    // Compressao rapida (impacto): adiabatica.
    float adiabaticExponent = 1.4f;
    // Razao de amortecimento da casca sobre a rigidez instantanea do contato,
    // e o quanto ela sobe nos impactos lentos: perto do repouso a histerese
    // da casca e do contato pesa mais e os quiques pequenos morrem (sem ela,
    // solta de 10 cm a bola quicava 21 vezes).
    // (Calibrado em 04/10 pelo estudo de calibracao: 2 m -> 1,39 m em base
    // rigida; solta de 10 cm ela quica 3 vezes e para em 0,5 s.)
    float dampingRatio = 0.045f;
    float lowSpeedDampingRatio = 0.3f;
    float lowSpeedDampingSpeed = 1.5f;
    // Pressao em que a razao de amortecimento vale (Pa, manometrica): o freio
    // da casca e dela, nao do ar - mais cheia, a bola fica mais rigida com a
    // mesma perda e quica mais; murcha, quica menos (pela razao fixa sobre a
    // rigidez, a pressao nao mudava o quique: 1,389-1,393 m de 8,5 a 15,6 psi).
    float dampingReferencePressurePascals = 80000.0f;
    // Rigidez da casca (flexao), N/m^1.5, por cima da do ar.
    float carcassStiffness = 0.0f;
    // Compressao maxima de um contato, fracao do raio (guarda numerica).
    float maximumCompressionShare = 0.75f;
    // Atrito da superficie da bola (combinado com o do outro corpo pela media
    // geometrica) e resistencia ao rolar (m: torque = coeficiente x forca).
    float friction = 0.6f;
    float rollingResistanceMeters = 0.004f;
    // Ar.
    float airDensity = 1.2f;
    float airKinematicViscosity = 1.5e-5f;
    float dragSubcritical = 0.47f;
    float dragSupercritical = 0.18f;
    float criticalReynolds = 2.2e5f;
    float criticalReynoldsSpan = 0.8e5f;
    // Com giro o arrasto sobe (~0,25-0,3 em Sp 0,2-0,4).
    float dragSpinGain = 0.25f;
    // Coeficiente de Magnus: inclinacao em Sp e teto.
    float liftSlope = 1.2f;
    float liftMaximum = 0.3f;
    // Decaimento do giro no ar (1/s).
    float spinDecay = 0.05f;
    // Subpassos por tick (a 120 Hz, 16 = ~2 kHz: um chute a 30 m/s anda 1,6
    // cm por subpasso; o contato de um chute dura ~9 ms).
    std::uint32_t substeps = 16;
};

struct InflatedBallDefinition3D {
    InflatedBallProfile3D profile;
    Vec3 position;
    Quaternion orientation;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    // Material da superficie (som e o que as outras camadas consultarem).
    std::string materialId = "soccer_ball";
    std::uint64_t entityId = 0;
    // Toca os corpos dos personagens (ragdolls). Desligado, a bola os
    // atravessa (o manejo por area, enquanto nao ha um toque decente).
    bool collidesWithRagdolls = true;
};

// Um contato da bola no ultimo subpasso do tick.
struct InflatedBallContact3D {
    Vec3 pointWorld;
    // Do outro corpo para a bola.
    Vec3 normalWorld { 0.0f, 0.0f, 1.0f };
    float compressionMeters = 0.0f;
    float normalForceNewtons = 0.0f;
    bool otherDynamic = false;
    // O outro corpo e um link de ragdoll: a entidade e o link (quem a bola
    // tocou - o manejo deixa o pe e a canela de quem tocou sem colisao).
    bool otherRagdoll = false;
    std::uint64_t otherEntityId = 0;
    std::uint32_t otherRagdollLink = 0;
};

struct InflatedBallState3D {
    Vec3 position;
    Quaternion orientation;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    float gaugePressurePascals = 0.0f;
    float volumeRatio = 1.0f;
    // Maior compressao no ultimo tick (m) e o pico desde a criacao.
    float compressionMeters = 0.0f;
    float peakCompressionMeters = 0.0f;
    std::uint32_t contactCount = 0;
    static constexpr std::uint32_t MaximumContacts = 8;
    std::array<InflatedBallContact3D, MaximumContacts> contacts {};
    bool frozen = false;
};

// Modelo (puro, sem o motor de fisica): usado pelo backend e testavel sozinho.
namespace InflatedBallModel3D {

[[nodiscard]] float restVolume(const InflatedBallProfile3D& profile);
// Volume de uma calota comprimida em d (m^3).
[[nodiscard]] float capVolume(const InflatedBallProfile3D& profile, float compression);
// Area da mancha achatada em d (m^2).
[[nodiscard]] float patchArea(const InflatedBallProfile3D& profile, float compression);
// Pressao manometrica com o volume que sobrou (Pa).
[[nodiscard]] float gaugePressure(const InflatedBallProfile3D& profile, float volume);
// Forca normal de um contato: elastica (ar + casca) e amortecimento pela
// velocidade de aproximacao (>0 comprimindo). Nunca puxa.
[[nodiscard]] float normalForce(const InflatedBallProfile3D& profile, float compression,
    float approachSpeed, float volume, float extraDampingRatio);
// Forca do ar (arrasto + Magnus) com a velocidade relativa ao ar.
[[nodiscard]] Vec3 aerodynamicForce(const InflatedBallProfile3D& profile, Vec3 relativeVelocity,
    Vec3 angularVelocity);
// Momento de inercia da casca (2/3 m R^2).
[[nodiscard]] float shellInertia(const InflatedBallProfile3D& profile);
// A bola de outro tamanho com o mesmo jeito (07/10, o tamanho no laboratorio):
// o peso pela area do couro (o arrasto no ar a desacelera igual e o mesmo
// chute a leva igual), a pressao caindo com o raio (no mesmo impacto ela
// achata o mesmo em relacao ao tamanho - inflada igual, a grande ficava dura),
// a referencia do amortecimento junto (quica igual) e o rolar pelo tamanho da
// marca no chao.
[[nodiscard]] InflatedBallProfile3D scaledProfile(const InflatedBallProfile3D& base, float radiusMeters);

} // namespace InflatedBallModel3D

} // namespace MatterEngine
