#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Objects/ObjectRecipe.hpp"
#include "Engine/Modeling/Raster/Image.hpp"

#include <functional>
#include <optional>
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
