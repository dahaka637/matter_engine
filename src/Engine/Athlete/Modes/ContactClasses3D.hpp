#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

// As classes de contato da reescrita da locomocao (vault:
// 04_locomocao/reescrita_da_locomocao.md, R3). Um lugar so decide o que e
// apoio, chao, raspao, impacto ou contato da acao - antes, a "normal do chao"
// morava no backend, a regra do degrau no passo do atleta e cada modo tinha o
// seu limiar.
//
// Invariantes: a classificacao nao depende do modo do corpo; um contato e de
// uma classe so; os limiares sao normalizados pela massa (do corpo e do link)
// e pela velocidade de aproximacao.

enum class BodyRegion3D : std::uint8_t { Trunk, Leg, Foot, Arm };

enum class ContactClass3D : std::uint8_t {
    // O pe no chao ou no degrau (andar).
    Support,
    // O chao embaixo da base tocado por outra parte (o quadril no carrinho, o
    // joelho no levantar, o agachamento fundo do pouso): nunca impacto.
    Ground,
    // Encostou sem forca (aproximacao lenta ou impulso abaixo dos limiares).
    Graze,
    Impact,
    // O contato que a acao em curso declarou (o pe do gesto na bola, a perna
    // do carrinho): quem trata e a acao, nao a regra de modo.
    Intentional
};

struct ContactClassParams3D {
    // Superficie virada para cima (a mesma regra do chao do corpo guiado).
    float supportNormalZ = 0.6f;
    // O pe ate esta altura acima da base e degrau, nao parede (a regra de
    // hoje: subir escada nao vai ao fisico a cada degrau).
    float footStepBandMeters = 0.40f;
    // O pe com a normal acima disto tambem e apoio (o espelho baixo do
    // degrau, a rampa).
    float footSupportNormalZ = 0.2f;
    // O chao embaixo da base: ate esta altura acima dela.
    float groundBandMeters = 0.05f;
    // Abaixo desta aproximacao (m/s) e raspao.
    float grazeSpeed = 0.3f;
    // Impacto, os de hoje como valores iniciais: a soma dos impulsos do tick
    // com o que se move >= bodyDeltaV x massa do corpo (4 N.s em 80 kg); com o
    // mundo parado >= staticBodyDeltaV x massa (2,5 N.s); ou, num contato do
    // tronco ou das pernas, J >= linkDeltaV x massa do link (o link leve
    // sacudido de verdade).
    float bodyDeltaV = 0.05f;
    float staticBodyDeltaV = 0.031f;
    float linkDeltaV = 1.0f;
    // Tronco e pernas entrando no mundo parado (consulta de pose), com
    // qualquer normal.
    float penetrationMeters = 0.015f;

    // Faixas validas (falha segura na carga dos parametros).
    [[nodiscard]] bool valid() const;
};

// O corpo como a classificacao o ve (do perfil, uma vez por atleta).
struct ContactBody3D {
    std::vector<BodyRegion3D> regions;
    std::vector<float> linkMassKg;
    float massKg = 0.0f;
    [[nodiscard]] bool valid() const { return massKg > 0.0f && regions.size() == linkMassKg.size() && !regions.empty(); }
};

[[nodiscard]] BodyRegion3D bodyRegion3D(const std::string& linkId);
[[nodiscard]] ContactBody3D contactBody3D(const RagdollProfile3D& profile);

// O que a acao em curso declarou (um bit por link).
struct ContactDeclarations3D {
    std::uint64_t intentionalLinks = 0;
};

// O tick inteiro: o impacto mais forte e o que a ajuda precisa saber.
struct ContactAssessment3D {
    bool impact = false;
    // A parte do impacto mais forte e de onde ele veio.
    std::uint32_t link = 0;
    bool fromPenetration = false;
    // O maior Dv do corpo e do link entre os impactos (m/s).
    float bodyDeltaV = 0.0f;
    float linkDeltaV = 0.0f;
    // A soma dos impulsos de impacto (N.s): e o que derruba a ajuda.
    float impactImpulse = 0.0f;
    // A maior penetracao de tronco e pernas fora do chao (m).
    float penetrationMeters = 0.0f;
    // O empurrao de fora (forca aplicada, sem contato) contou neste tick, e
    // quanto (N.s; fora de impactImpulse, que e so dos contatos).
    bool fromPush = false;
    float pushImpulse = 0.0f;
    // O contato da acao (a acao decide o que fazer com ele).
    bool intentional = false;
    float intentionalImpulse = 0.0f;
    int grazes = 0;
};

// A velocidade de aproximacao de um contato (m/s, >= 0): a normal e a da
// superficie do outro apontando para o link; a velocidade relativa e a do link
// menos a do outro, antes do solver.
[[nodiscard]] float contactApproachSpeed3D(const RagdollInteraction3D& contact);

[[nodiscard]] ContactClass3D classifyInteraction3D(const RagdollInteraction3D& contact, const ContactBody3D& body,
    const ContactDeclarations3D& declared, float baseHeight, const ContactClassParams3D& params);

// Uma penetracao da consulta de pose (o corpo guiado nao colide com
// superficies viradas para cima, entao o topo de uma caixa so aparece aqui).
[[nodiscard]] ContactClass3D classifyPenetration3D(const RagdollWorldContact3D& contact, const ContactBody3D& body,
    float baseHeight, const ContactClassParams3D& params);

// pushImpulse: o empurrao de fora no tick (N.s; forca aplicada sem contato),
// somado como um contato com algo que se move.
[[nodiscard]] ContactAssessment3D assessContacts3D(std::span<const RagdollInteraction3D> interactions,
    std::span<const RagdollWorldContact3D> penetrations, const ContactBody3D& body,
    const ContactDeclarations3D& declared, float baseHeight, const ContactClassParams3D& params,
    float pushImpulse = 0.0f);

// A soma curta dos impulsos (N.s): um choque que se espalha por alguns ticks
// (a caixa pesada, o empurrao que dura) conta inteiro. E um decaimento
// exponencial com a constante de tempo dada (nao uma janela dura): 100 ms
// depois de um choque de 40 N.s sobra ~1,5 N.s, abaixo do minimo da ajuda; a
// cauda nunca cria uma queda nova da ajuda (ela so recomeca com um choque
// maior que o que ainda resta).
struct ImpulseWindow3D {
    float value = 0.0f;
    float add(float impulse, float deltaTime, float timeConstantSeconds);
};

[[nodiscard]] const char* contactClassName3D(ContactClass3D value);

} // namespace MatterEngine
