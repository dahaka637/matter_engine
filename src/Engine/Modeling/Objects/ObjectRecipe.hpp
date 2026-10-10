#pragma once

#include "Engine/Modeling/Raster/Color.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// A receita de um objeto procedural (assets/objects/<id>.json): o tipo (o
// gerador), o nome, o material fisico e os valores dos parametros do tipo -
// o que a receita nao trouxer fica no padrao do tipo. O inspetor de objetos
// edita os valores e grava a receita de volta (cores em "#rrggbb").
namespace MatterEngine::Modeling {

enum class ObjectParameterKind : std::uint8_t { Number, Integer, Toggle, Color };

// Um parametro de um tipo de objeto: o que o inspetor mostra e o padrao.
struct ObjectParameterSpec {
    std::string key;
    std::string label;
    // O grupo na tela (Forma, Textura, Fisica...).
    std::string group;
    ObjectParameterKind kind = ObjectParameterKind::Number;
    float defaultValue = 0.0f;
    float minimum = 0.0f;
    float maximum = 1.0f;
    std::string unit;
    Color defaultColor;
};

struct ObjectRecipe {
    std::string id;
    std::string displayName;
    std::string type;
    std::string material = "default";
    // Os valores (numero, inteiro e liga/desliga sao numeros; cores a parte).
    std::map<std::string, float> numbers;
    std::map<std::string, Color> colors;
};

[[nodiscard]] std::optional<ObjectRecipe> parseObjectRecipe(std::string_view json, std::string* error = nullptr);
[[nodiscard]] std::optional<ObjectRecipe> readObjectRecipe(const std::filesystem::path& path, std::string* error = nullptr);
[[nodiscard]] std::string objectRecipeJson(const ObjectRecipe& recipe);
bool writeObjectRecipe(const ObjectRecipe& recipe, const std::filesystem::path& path, std::string* error = nullptr);
// As receitas da pasta (os .json), em ordem de nome.
[[nodiscard]] std::vector<std::filesystem::path> objectRecipeFiles(const std::filesystem::path& directory);

// "#rrggbb" <-> cor.
[[nodiscard]] std::optional<Color> parseHexColor(std::string_view text);
[[nodiscard]] std::string hexColor(Color color);

} // namespace MatterEngine::Modeling
