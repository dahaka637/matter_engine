#include "Engine/Modeling/Objects/ObjectRecipe.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace MatterEngine::Modeling {

namespace {

using Json = nlohmann::ordered_json;

constexpr std::string_view Schema = "matter.object.v1";

int hexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

} // namespace

std::optional<Color> parseHexColor(std::string_view text) {
    if (text.size() != 7 || text.front() != '#') return std::nullopt;
    int channels[3] {};
    for (int i = 0; i < 3; ++i) {
        const int high = hexDigit(text[static_cast<std::size_t>(1 + i * 2)]);
        const int low = hexDigit(text[static_cast<std::size_t>(2 + i * 2)]);
        if (high < 0 || low < 0) return std::nullopt;
        channels[i] = high * 16 + low;
    }
    return rgb(channels[0], channels[1], channels[2]);
}

std::string hexColor(Color color) {
    const auto channel = [](float value) { return static_cast<int>(std::lround(std::clamp(value, 0.0f, 1.0f) * 255.0f)); };
    char text[8];
    std::snprintf(text, sizeof text, "#%02x%02x%02x", channel(color.r), channel(color.g), channel(color.b));
    return text;
}

std::optional<ObjectRecipe> parseObjectRecipe(std::string_view text, std::string* error) {
    const auto fail = [&](std::string message) -> std::optional<ObjectRecipe> {
        if (error != nullptr) *error = std::move(message);
        return std::nullopt;
    };
    Json json;
    try {
        json = Json::parse(text);
    } catch (const std::exception& exception) {
        return fail(std::string("receita de objeto: ") + exception.what());
    }
    if (!json.is_object() || json.value("schema", std::string()) != Schema)
        return fail("receita de objeto: schema diferente de " + std::string(Schema));
    ObjectRecipe recipe;
    recipe.id = json.value("id", std::string());
    recipe.displayName = json.value("displayName", recipe.id);
    recipe.type = json.value("type", std::string());
    recipe.material = json.value("material", std::string("default"));
    if (recipe.id.empty() || recipe.type.empty()) return fail("receita de objeto: faltam id e type");
    if (json.contains("parameters")) {
        const Json& parameters = json.at("parameters");
        if (!parameters.is_object()) return fail("receita de objeto: parameters deve ser um objeto");
        for (const auto& [key, value] : parameters.items()) {
            if (value.is_number()) {
                const float number = value.get<float>();
                if (!std::isfinite(number)) return fail("receita de objeto: " + key + " nao e finito");
                recipe.numbers[key] = number;
            } else if (value.is_boolean()) {
                recipe.numbers[key] = value.get<bool>() ? 1.0f : 0.0f;
            } else if (value.is_string()) {
                const std::optional<Color> color = parseHexColor(value.get<std::string>());
                if (!color) return fail("receita de objeto: " + key + " nao e uma cor #rrggbb");
                recipe.colors[key] = *color;
            } else {
                return fail("receita de objeto: " + key + " deve ser numero, booleano ou cor");
            }
        }
    }
    return recipe;
}

std::optional<ObjectRecipe> readObjectRecipe(const std::filesystem::path& path, std::string* error) {
    std::ifstream file(path);
    if (!file) {
        if (error != nullptr) *error = "nao abriu " + path.string();
        return std::nullopt;
    }
    std::stringstream text;
    text << file.rdbuf();
    std::string parseError;
    std::optional<ObjectRecipe> recipe = parseObjectRecipe(text.str(), &parseError);
    if (!recipe && error != nullptr) *error = path.filename().string() + ": " + parseError;
    return recipe;
}

std::string objectRecipeJson(const ObjectRecipe& recipe) {
    Json json;
    json["schema"] = Schema;
    json["id"] = recipe.id;
    json["displayName"] = recipe.displayName;
    json["type"] = recipe.type;
    json["material"] = recipe.material;
    Json parameters = Json::object();
    // Os numeros com 4 casas (a receita fica legivel e estavel).
    for (const auto& [key, value] : recipe.numbers) {
        const double rounded = std::round(static_cast<double>(value) * 10000.0) / 10000.0;
        if (rounded == std::floor(rounded) && std::abs(rounded) < 1e9) parameters[key] = static_cast<long long>(rounded);
        else parameters[key] = rounded;
    }
    for (const auto& [key, value] : recipe.colors) parameters[key] = hexColor(value);
    json["parameters"] = parameters;
    return json.dump(2) + "\n";
}

bool writeObjectRecipe(const ObjectRecipe& recipe, const std::filesystem::path& path, std::string* error) {
    std::ofstream file(path);
    if (file) file << objectRecipeJson(recipe);
    if (!file) {
        if (error != nullptr) *error = "nao gravou " + path.string();
        return false;
    }
    return true;
}

std::vector<std::filesystem::path> objectRecipeFiles(const std::filesystem::path& directory) {
    std::vector<std::filesystem::path> files;
    std::error_code code;
    for (const auto& entry : std::filesystem::directory_iterator(directory, code))
        if (entry.is_regular_file() && entry.path().extension() == ".json") files.push_back(entry.path());
    std::sort(files.begin(), files.end());
    return files;
}

} // namespace MatterEngine::Modeling
