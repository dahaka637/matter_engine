#include "Engine/Modeling/Parts/LegRecipe.hpp"

#include "Engine/Modeling/Parts/RecipeJson.hpp"

namespace MatterEngine::Modeling {

LegRecipe LegRecipe::defaults() {
    LegRecipe recipe;
    // As medidas das referencias (frente e lado), no eixo do osso de pe.
    recipe.front = { { 0.15f, 0.042f }, { 0.185f, 0.04f }, { 0.23f, 0.039f }, { 0.28f, 0.042f }, { 0.33f, 0.05f },
        { 0.38f, 0.06f }, { 0.425f, 0.066f }, { 0.45f, 0.066f }, { 0.48f, 0.066f }, { 0.51f, 0.066f }, { 0.5375f, 0.068f },
        { 0.56f, 0.07f }, { 0.585f, 0.069f }, { 0.62f, 0.07f }, { 0.66f, 0.076f }, { 0.705f, 0.085f } };
    recipe.back = { { 0.15f, 0.05f }, { 0.185f, 0.046f }, { 0.23f, 0.05f }, { 0.28f, 0.055f }, { 0.33f, 0.064f },
        { 0.38f, 0.075f }, { 0.425f, 0.082f }, { 0.45f, 0.08f }, { 0.48f, 0.072f }, { 0.51f, 0.063f }, { 0.5375f, 0.057f },
        { 0.56f, 0.055f }, { 0.585f, 0.055f }, { 0.62f, 0.06f }, { 0.66f, 0.068f }, { 0.705f, 0.085f } };
    recipe.medial = { { 0.15f, 0.038f }, { 0.185f, 0.037f }, { 0.23f, 0.037f }, { 0.28f, 0.042f }, { 0.33f, 0.051f },
        { 0.38f, 0.065f }, { 0.425f, 0.077f }, { 0.45f, 0.071f }, { 0.48f, 0.063f }, { 0.51f, 0.058f },
        { 0.5375f, 0.056f }, { 0.56f, 0.057f }, { 0.585f, 0.061f }, { 0.62f, 0.067f }, { 0.66f, 0.072f },
        { 0.705f, 0.078f } };
    recipe.lateral = { { 0.15f, 0.038f }, { 0.185f, 0.037f }, { 0.23f, 0.037f }, { 0.28f, 0.041f },
        { 0.33f, 0.048f }, { 0.38f, 0.06f }, { 0.425f, 0.07f }, { 0.45f, 0.067f }, { 0.48f, 0.061f },
        { 0.51f, 0.057f }, { 0.5375f, 0.055f }, { 0.56f, 0.055f }, { 0.585f, 0.057f }, { 0.62f, 0.063f },
        { 0.66f, 0.073f }, { 0.705f, 0.082f } };
    recipe.exponent = { { 0.15f, 2.2f }, { 0.33f, 2.3f }, { 0.555f, 2.4f }, { 0.705f, 2.3f } };
    recipe.rings = { 0.165f, 0.19f, 0.22f, 0.25f, 0.28f, 0.31f, 0.34f, 0.37f, 0.4f, 0.43f, 0.455f, 0.48f, 0.5025f,
        0.525f, 0.545f, 0.565f, 0.585f, 0.605f, 0.63f, 0.66f, 0.69f, 0.705f };
    return recipe;
}

namespace {

void validateRecipe(const LegRecipe& r) {
    const auto check = [](bool condition, const char* message) {
        if (!condition) throw std::invalid_argument(message);
    };
    check(r.ringPoints >= 8 && r.ringPoints <= 64 && r.ringPoints % 4 == 0, "ringPoints: de 8 a 64, multiplo de 4");
    check(r.rings.size() >= 3 && r.rings.size() <= 128, "rings: de 3 a 128 alturas");
    for (std::size_t i = 0; i < r.rings.size(); ++i)
        check(std::isfinite(r.rings[i]) && r.rings[i] > 0.0f && r.rings[i] < 1.5f && (i == 0 || r.rings[i] > r.rings[i - 1]),
            "rings: alturas crescentes");
    check(r.collarRings >= 0 && r.collarRings <= 6 && r.collarLift >= 0.0f && r.collarLift < 0.05f, "saida da gola invalida");
    check(r.sockTop > r.rings.front() && r.bandTop > r.sockTop && r.bandTop < r.rings.back(), "meiao e faixa fora da perna");
    check(r.jitter >= 0.0f && r.jitter < 0.01f && r.verticalJitter >= 0.0f && r.verticalJitter < 0.02f, "jitter invalido");
    check(r.patellaHalfHeight > 0.005f && r.patellaHalfAngle > 0.05f && r.patellaHalfAngle < 1.5f
            && std::abs(r.patellaProtrusion) < 0.02f && r.tuberosityHalfHeight > 0.003f
            && std::abs(r.tuberosityProtrusion) < 0.01f && std::abs(r.poplitealDepth) < 0.02f,
        "joelho invalido (patela, tuberosidade ou oco)");
    check(!r.front.empty() && !r.back.empty() && !r.medial.empty() && !r.lateral.empty() && !r.exponent.empty(),
        "secao incompleta (front, back, medial, lateral, exponent)");
    for (const float h : r.rings)
        check(r.front(h) > 0.01f && r.back(h) > 0.01f && r.medial(h) > 0.01f && r.lateral(h) > 0.01f && r.exponent(h) >= 1.5f
                && r.exponent(h) <= 6.0f,
            "secao sem largura em alguma altura (ou expoente fora de [1,5; 6])");
}

} // namespace

std::optional<LegRecipe> parseLegRecipe(std::string_view json, std::string* error) {
    try {
        const nlohmann::json document = nlohmann::json::parse(json);
        if (!document.is_object() || (document.contains("type") && document.at("type") != "leg"))
            throw std::invalid_argument("esperado objeto do tipo leg");
        LegRecipe recipe = LegRecipe::defaults();
        readCurve(document, "front", recipe.front, 1.5f);
        readCurve(document, "back", recipe.back, 1.5f);
        readCurve(document, "medial", recipe.medial, 1.5f);
        readCurve(document, "lateral", recipe.lateral, 1.5f);
        readCurve(document, "exponent", recipe.exponent, 1.5f);
        if (document.contains("rings")) recipe.rings = document.at("rings").get<std::vector<float>>();
        readValue(document, "ringPoints", recipe.ringPoints);
        readValue(document, "collarRings", recipe.collarRings);
        readValue(document, "collarLift", recipe.collarLift);
        readValue(document, "sockTop", recipe.sockTop);
        readValue(document, "bandTop", recipe.bandTop);
        readValue(document, "patellaCenter", recipe.patellaCenter);
        readValue(document, "patellaHalfHeight", recipe.patellaHalfHeight);
        readValue(document, "patellaHalfAngle", recipe.patellaHalfAngle);
        readValue(document, "patellaProtrusion", recipe.patellaProtrusion);
        readValue(document, "tuberosityCenter", recipe.tuberosityCenter);
        readValue(document, "tuberosityHalfHeight", recipe.tuberosityHalfHeight);
        readValue(document, "tuberosityProtrusion", recipe.tuberosityProtrusion);
        readValue(document, "poplitealDepth", recipe.poplitealDepth);
        readValue(document, "jitter", recipe.jitter);
        readValue(document, "verticalJitter", recipe.verticalJitter);
        readValue(document, "seed", recipe.seed);
        if (document.contains("colors")) {
            const nlohmann::json& colors = document.at("colors");
            readColor(colors, "sock", recipe.sockColor);
            readColor(colors, "band", recipe.bandColor);
            readColor(colors, "skin", recipe.skinColor);
        }
        validateRecipe(recipe);
        if (error != nullptr) error->clear();
        return recipe;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("receita da perna invalida: ") + exception.what();
        return std::nullopt;
    }
}

} // namespace MatterEngine::Modeling
