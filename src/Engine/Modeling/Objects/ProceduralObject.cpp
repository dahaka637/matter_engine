#include "Engine/Modeling/Objects/ProceduralObject.hpp"

#include "Engine/Modeling/Objects/WoodCrate.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

const ObjectParameterSpec* ObjectType::find(std::string_view key) const {
    for (const ObjectParameterSpec& spec : parameters)
        if (spec.key == key) return &spec;
    return nullptr;
}

const std::vector<ObjectType>& objectTypes() {
    // Um tipo novo (a trave, a rede...) entra aqui, com o gerador dele.
    static const std::vector<ObjectType> types { woodCrateObjectType() };
    return types;
}

const ObjectType* findObjectType(std::string_view id) {
    for (const ObjectType& type : objectTypes())
        if (type.id == id) return &type;
    return nullptr;
}

float objectNumber(const ObjectRecipe& recipe, const ObjectType& type, std::string_view key) {
    const ObjectParameterSpec* spec = type.find(key);
    if (spec == nullptr) return 0.0f;
    const auto found = recipe.numbers.find(std::string(key));
    const float value = found != recipe.numbers.end() ? found->second : spec->defaultValue;
    return std::clamp(value, spec->minimum, spec->maximum);
}

int objectInteger(const ObjectRecipe& recipe, const ObjectType& type, std::string_view key) {
    return static_cast<int>(std::lround(objectNumber(recipe, type, key)));
}

bool objectToggle(const ObjectRecipe& recipe, const ObjectType& type, std::string_view key) {
    return objectNumber(recipe, type, key) >= 0.5f;
}

Color objectColor(const ObjectRecipe& recipe, const ObjectType& type, std::string_view key) {
    const ObjectParameterSpec* spec = type.find(key);
    if (spec == nullptr) return {};
    const auto found = recipe.colors.find(std::string(key));
    return found != recipe.colors.end() ? found->second : spec->defaultColor;
}

ObjectRecipe defaultObjectRecipe(const ObjectType& type, std::string id, std::string displayName, std::string material) {
    ObjectRecipe recipe;
    recipe.id = std::move(id);
    recipe.displayName = std::move(displayName);
    recipe.type = type.id;
    recipe.material = std::move(material);
    for (const ObjectParameterSpec& spec : type.parameters) {
        if (spec.kind == ObjectParameterKind::Color) recipe.colors[spec.key] = spec.defaultColor;
        else recipe.numbers[spec.key] = spec.defaultValue;
    }
    return recipe;
}

std::optional<ProceduralObject> generateObject(const ObjectRecipe& recipe, std::string* error) {
    const ObjectType* type = findObjectType(recipe.type);
    if (type == nullptr || !type->generate) {
        if (error != nullptr) *error = recipe.id + ": tipo de objeto desconhecido: " + recipe.type;
        return std::nullopt;
    }
    ProceduralObject object;
    object.id = recipe.id;
    object.displayName = recipe.displayName;
    object.type = recipe.type;
    std::string message;
    if (!type->generate(recipe, *type, object, message)) {
        if (error != nullptr) *error = recipe.id + ": " + message;
        return std::nullopt;
    }
    return object;
}

} // namespace MatterEngine::Modeling
