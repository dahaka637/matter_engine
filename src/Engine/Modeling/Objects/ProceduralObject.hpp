#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Objects/ObjectRecipe.hpp"
#include "Engine/Modeling/Raster/Image.hpp"

#include <array>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Os objetos procedurais (a caixa hoje; a trave, a rede e o que vier): cada
// tipo e um gerador com os parametros dele, e a receita (ObjectRecipe) da os
// valores. O objeto gerado traz a malha da modelagem (com material, UV e a
// textura em pixel art, para as mesmas analises e a mesma inspecao do
// jogador) e a descricao fisica (o material, a massa e as formas de colisao).
// O nucleo nao conhece a engine: quem leva ao jogo e a fisica e o modulo
// Engine/Objects.
namespace MatterEngine::Modeling {

enum class ObjectColliderShape : std::uint8_t { Box, Sphere, Capsule, Cylinder };

// Uma forma de colisao no espaco do objeto (a origem no centro da caixa que o
// contem, z para cima).
struct ObjectCollider {
    ObjectColliderShape shape = ObjectColliderShape::Box;
    Vec3 center;
    Quaternion orientation;
    // Caixa: as meias-dimensoes. Esfera: radius. Capsula e cilindro: radius e
    // halfHeight ao longo do z local.
    Vec3 halfExtents;
    float radius = 0.0f;
    float halfHeight = 0.0f;
};

struct ObjectPhysics {
    // O material fisico (o catalogo de materiais da engine: atrito, quique).
    std::string material = "default";
    float massKg = 1.0f;
    std::vector<ObjectCollider> colliders;
    // Preso no mundo (a trave fincada): so colide, nao cai.
    bool fixed = false;
};

// Uma rede de fios presa no objeto (a rede do gol), no espaco do objeto, em
// duas malhas da mesma rede:
//  - a da fisica (a rede de Engine/Physics/Net): os nos, quais estao presos
//    na estrutura, o peso de cada um, os fios (o comprimento solto, a rigidez
//    EA e o peso por metro), as celulas em triangulos (a bola toca a rede
//    por elas) e o fio de verdade que cada no carrega (o arrasto do ar). Ela
//    pode ser mais grossa que a de verdade: cada fio dela vale pelos fios
//    paralelos que representa (a rigidez somada), e o peso e o fio de cada no
//    sao os da malha de verdade em volta dele;
//  - a desenhada (a malha de verdade): cada no dela e uma media fixa de ate
//    quatro nos da fisica (os pesos do repouso, como a pele nos ossos).
struct ObjectNetSegment {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    float restLength = 0.0f;
    float stiffness = 10000.0f;
    float linearDensity = 0.007f;
    // O cabo tensor (nao um fio da malha).
    bool cord = false;
};

// Um no desenhado: os nos da fisica e os pesos (somam 1).
struct ObjectNetWeight {
    std::array<std::uint32_t, 4> knots {};
    std::array<float, 4> weights {};
};

struct ObjectNet {
    std::vector<Vec3> knots;
    std::vector<std::uint8_t> pinned;
    std::vector<float> masses;
    std::vector<ObjectNetSegment> segments;
    std::vector<std::array<std::uint32_t, 3>> faces;
    // m
    std::vector<float> twineLengths;
    // A malha desenhada: os nos no repouso, os fios e os pesos de cada no.
    std::vector<Vec3> drawnKnots;
    std::vector<std::array<std::uint32_t, 2>> drawnSegments;
    std::vector<ObjectNetWeight> drawnWeights;
    // A barreira invisivel dos jogadores (a trave: os lados inteiros, o teto e
    // uma parede reta na barra de tras; a boca aberta): os pontos da casca convexa de
    // cada placa. O corpo do boneco e a capsula param nela; a bola e os
    // objetos so na rede.
    std::vector<std::vector<Vec3>> barrier;
    // As placas so da capsula do personagem (a trave: o fundo inclinado - com
    // o corpo batendo nele, o boneco caido embaixo nao levantava).
    std::vector<std::vector<Vec3>> capsuleBarrier;
    // A escala de massa dos nos da fisica no contato com os corpos (a da
    // malha grossa e menor: os nos dela ja sao mais pesados).
    float contactMassScale = 6.0f;
    float twineDiameter = 0.006f;
    Color color = rgb(240, 240, 236);
};

// Os nos desenhados a partir dos nos da fisica de agora (no espaco deles: os
// pesos valem em qualquer um). Cada no desenhado fica num triangulo de
// colisao, entao fica fora da bola e dos membros que a fisica afasta (nos
// chutes da grade, o fio encosta na bola sem entrar).
void drawnNetKnots(const ObjectNet& net, std::span<const Vec3> knots, std::vector<Vec3>& out);

struct ProceduralObject {
    std::string id;
    std::string displayName;
    std::string type;
    // No espaco do objeto (a origem no centro da caixa que o contem), com o
    // material e a textura ja ampliada (cada texel um bloco).
    Mesh mesh;
    // A pixel art no tamanho real (um texel por pixel).
    Image pixelArt;
    ObjectPhysics physics;
    Vec3 size;
    // A rede presa nele (so os objetos com rede: a trave).
    std::optional<ObjectNet> net;
};

struct ObjectType {
    std::string id;
    std::string label;
    std::vector<ObjectParameterSpec> parameters;
    std::function<bool(const ObjectRecipe&, const ObjectType&, ProceduralObject&, std::string&)> generate;

    [[nodiscard]] const ObjectParameterSpec* find(std::string_view key) const;
};

[[nodiscard]] const std::vector<ObjectType>& objectTypes();
[[nodiscard]] const ObjectType* findObjectType(std::string_view id);
// O valor de um parametro (o da receita, ou o padrao do tipo), no intervalo.
[[nodiscard]] float objectNumber(const ObjectRecipe& recipe, const ObjectType& type, std::string_view key);
[[nodiscard]] int objectInteger(const ObjectRecipe& recipe, const ObjectType& type, std::string_view key);
[[nodiscard]] bool objectToggle(const ObjectRecipe& recipe, const ObjectType& type, std::string_view key);
[[nodiscard]] Color objectColor(const ObjectRecipe& recipe, const ObjectType& type, std::string_view key);
// Uma receita do tipo com todos os padroes escritos.
[[nodiscard]] ObjectRecipe defaultObjectRecipe(const ObjectType& type, std::string id, std::string displayName,
    std::string material);
// Gera o objeto da receita (o tipo dela); nada com o erro.
[[nodiscard]] std::optional<ProceduralObject> generateObject(const ObjectRecipe& recipe, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
