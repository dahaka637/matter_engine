#include "Engine/Modeling/Parts/NeckRecipe.hpp"

#include "Engine/Modeling/Parts/RecipeJson.hpp"

namespace MatterEngine::Modeling {

NeckRecipe NeckRecipe::defaults() {
    NeckRecipe recipe;
    // As medidas das referencias (o pescoco de frente e de lado; embaixo, o
    // alto do peito que aparece no V).
    recipe.centerX = { { 1.54f, -0.042f }, { 1.57f, -0.04f }, { 1.6f, -0.037f }, { 1.63f, -0.035f }, { 1.66f, -0.033f },
        { 1.7f, -0.03f } };
    recipe.front = { { 1.54f, 0.1f }, { 1.57f, 0.083f }, { 1.6f, 0.059f }, { 1.63f, 0.06f }, { 1.66f, 0.066f }, { 1.7f, 0.068f } };
    recipe.back = { { 1.54f, 0.09f }, { 1.57f, 0.082f }, { 1.6f, 0.076f }, { 1.63f, 0.07f }, { 1.66f, 0.065f }, { 1.7f, 0.065f } };
    recipe.side = { { 1.54f, 0.14f }, { 1.57f, 0.105f }, { 1.6f, 0.078f }, { 1.63f, 0.069f }, { 1.66f, 0.0675f }, { 1.7f, 0.067f } };
    recipe.exponent = { { 1.54f, 2.4f }, { 1.57f, 2.3f }, { 1.6f, 2.2f }, { 1.7f, 2.2f } };
    return recipe;
}

namespace {

void validateRecipe(const NeckRecipe& r) {
    const auto check = [](bool condition, const char* message) {
        if (!condition) throw std::invalid_argument(message);
    };
    check(r.headFrontHeight > 1.0f && r.headBackHeight > 1.0f && r.headFrontHeight < 2.2f && r.headBackHeight < 2.2f,
        "alturas da abertura da cabeca invalidas");
    check(r.rings >= 0 && r.rings <= 12 && r.jitter >= 0.0f && r.jitter < 0.01f, "aneis ou jitter invalidos");
    for (float h = 1.5f; h <= std::max(r.headFrontHeight, r.headBackHeight) + 1e-4f; h += 0.01f)
        check(r.front(h) > 0.02f && r.back(h) > 0.02f && r.side(h) > 0.03f && r.exponent(h) >= 1.5f, "pescoco sem largura");
}

} // namespace

std::optional<NeckRecipe> parseNeckRecipe(std::string_view json, std::string* error) {
    try {
        const nlohmann::json document = nlohmann::json::parse(json);
        if (!document.is_object() || (document.contains("type") && document.at("type") != "neck"))
            throw std::invalid_argument("esperado objeto do tipo neck");
        NeckRecipe recipe = NeckRecipe::defaults();
        readCurve(document, "centerX", recipe.centerX, 2.5f);
        readCurve(document, "front", recipe.front, 2.5f);
        readCurve(document, "back", recipe.back, 2.5f);
        readCurve(document, "side", recipe.side, 2.5f);
        readCurve(document, "exponent", recipe.exponent, 2.5f);
        readValue(document, "headFrontHeight", recipe.headFrontHeight);
        readValue(document, "headBackHeight", recipe.headBackHeight);
        readValue(document, "rings", recipe.rings);
        readValue(document, "jitter", recipe.jitter);
        readValue(document, "seed", recipe.seed);
        if (document.contains("colors")) readColor(document.at("colors"), "skin", recipe.skinColor);
        validateRecipe(recipe);
        if (error != nullptr) error->clear();
        return recipe;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("receita do pescoco invalida: ") + exception.what();
        return std::nullopt;
    }
}

} // namespace MatterEngine::Modeling
