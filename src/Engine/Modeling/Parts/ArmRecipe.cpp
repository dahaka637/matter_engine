#include "Engine/Modeling/Parts/ArmRecipe.hpp"

#include "Engine/Modeling/Parts/RecipeJson.hpp"

namespace MatterEngine::Modeling {

ArmRecipe ArmRecipe::defaults() {
    ArmRecipe recipe;
    // As medidas das referencias (o braco saindo da manga ate o cotovelo).
    // Na pose T com a palma para baixo o antebraco gira: perto do cotovelo
    // o lado largo e o vertical (os epicondilos), no pulso o da frente para
    // tras (do polegar ao mindinho). A massa dos flexores fica embaixo.
    recipe.front = { { 0.25f, 0.048f }, { 0.303f, 0.044f }, { 0.345f, 0.045f }, { 0.383f, 0.046f }, { 0.42f, 0.045f },
        { 0.46f, 0.041f }, { 0.5f, 0.037f }, { 0.54f, 0.033f }, { 0.573f, 0.031f } };
    recipe.back = { { 0.25f, 0.05f }, { 0.303f, 0.044f }, { 0.345f, 0.046f }, { 0.383f, 0.046f }, { 0.42f, 0.045f },
        { 0.46f, 0.041f }, { 0.5f, 0.037f }, { 0.54f, 0.034f }, { 0.573f, 0.033f } };
    recipe.top = { { 0.25f, 0.052f }, { 0.303f, 0.047f }, { 0.345f, 0.047f }, { 0.383f, 0.05f }, { 0.42f, 0.047f },
        { 0.46f, 0.04f }, { 0.5f, 0.032f }, { 0.54f, 0.024f }, { 0.573f, 0.018f } };
    recipe.bottom = { { 0.25f, 0.05f }, { 0.303f, 0.047f }, { 0.345f, 0.053f }, { 0.383f, 0.062f }, { 0.42f, 0.058f },
        { 0.46f, 0.052f }, { 0.5f, 0.046f }, { 0.54f, 0.039f }, { 0.573f, 0.034f } };
    recipe.exponent = { { 0.25f, 2.2f }, { 0.42f, 2.2f }, { 0.573f, 2.6f } };
    recipe.forward = { { 0.25f, 0.0f }, { 0.573f, 0.0f } };
    recipe.rings = { 0.278f, 0.29f, 0.303f, 0.316f, 0.33f, 0.345f, 0.37f, 0.395f, 0.42f, 0.45f, 0.48f, 0.51f, 0.54f, 0.573f };
    return recipe;
}

namespace {

void validateRecipe(const ArmRecipe& r) {
    const auto check = [](bool condition, const char* message) {
        if (!condition) throw std::invalid_argument(message);
    };
    check(!r.rings.empty(), "rings: ao menos o anel do cotovelo");
    for (std::size_t i = 0; i < r.rings.size(); ++i)
        check(std::isfinite(r.rings[i]) && r.rings[i] > 0.0f && r.rings[i] < 1.0f && (i == 0 || r.rings[i] > r.rings[i - 1] + 0.003f),
            "rings: estacoes crescentes");
    for (float s = 0.2f; s <= r.rings.back() + 1e-4f; s += 0.01f)
        check(r.front(s) > 0.015f && r.back(s) > 0.015f && r.top(s) > 0.015f && r.bottom(s) > 0.015f && r.exponent(s) >= 1.5f
                && std::abs(r.forward(s)) < 0.04f,
            "braco sem largura");
    check(r.jitter >= 0.0f && r.jitter < 0.01f, "jitter invalido");
    check(r.forearmFrom > r.rings.front() && r.forearmFrom < r.rings.back(), "forearmFrom: entre o primeiro e o ultimo anel");
    check(r.olecranonProtrusion >= 0.0f && r.olecranonProtrusion < 0.02f && r.olecranonHalfLength > 0.005f
            && r.olecranonHalfLength < 0.08f && r.olecranonHalfAngle > 0.1f && r.olecranonHalfAngle < 2.0f,
        "olecrano invalido");
}

} // namespace

std::optional<ArmRecipe> parseArmRecipe(std::string_view json, std::string* error) {
    try {
        const nlohmann::json document = nlohmann::json::parse(json);
        if (!document.is_object() || (document.contains("type") && document.at("type") != "arm"))
            throw std::invalid_argument("esperado objeto do tipo arm");
        ArmRecipe recipe = ArmRecipe::defaults();
        readCurve(document, "front", recipe.front);
        readCurve(document, "back", recipe.back);
        readCurve(document, "top", recipe.top);
        readCurve(document, "bottom", recipe.bottom);
        readCurve(document, "exponent", recipe.exponent);
        readCurve(document, "forward", recipe.forward);
        if (document.contains("rings")) recipe.rings = document.at("rings").get<std::vector<float>>();
        readValue(document, "forearmFrom", recipe.forearmFrom);
        readValue(document, "olecranonProtrusion", recipe.olecranonProtrusion);
        readValue(document, "olecranonHalfLength", recipe.olecranonHalfLength);
        readValue(document, "olecranonHalfAngle", recipe.olecranonHalfAngle);
        readValue(document, "jitter", recipe.jitter);
        readValue(document, "seed", recipe.seed);
        if (document.contains("colors")) readColor(document.at("colors"), "skin", recipe.skinColor);
        validateRecipe(recipe);
        if (error != nullptr) error->clear();
        return recipe;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("receita do braco invalida: ") + exception.what();
        return std::nullopt;
    }
}

} // namespace MatterEngine::Modeling
