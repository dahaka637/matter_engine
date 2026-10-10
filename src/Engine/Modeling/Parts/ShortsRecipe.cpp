#include "Engine/Modeling/Parts/ShortsRecipe.hpp"

#include "Engine/Modeling/Parts/RecipeJson.hpp"

namespace MatterEngine::Modeling {

ShortsRecipe ShortsRecipe::defaults() {
    ShortsRecipe recipe;
    // As medidas das referencias (calcao de frente, de lado e de costas).
    recipe.legFront = { { 0.705f, 0.105f }, { 0.75f, 0.108f }, { 0.8f, 0.112f }, { 0.85f, 0.115f }, { 0.9f, 0.12f } };
    recipe.legBack = { { 0.705f, 0.11f }, { 0.75f, 0.115f }, { 0.8f, 0.122f }, { 0.85f, 0.128f }, { 0.9f, 0.135f } };
    recipe.legMedial = { { 0.705f, 0.08f }, { 0.75f, 0.082f }, { 0.8f, 0.084f }, { 0.85f, 0.086f }, { 0.9f, 0.088f } };
    recipe.legLateral = { { 0.705f, 0.118f }, { 0.75f, 0.122f }, { 0.8f, 0.126f }, { 0.85f, 0.13f }, { 0.9f, 0.135f } };
    recipe.hipFront = { { 0.9f, 0.12f }, { 0.95f, 0.118f }, { 1.0f, 0.112f }, { 1.05f, 0.105f }, { 1.1f, 0.1f } };
    recipe.hipBack = { { 0.9f, 0.135f }, { 0.95f, 0.14f }, { 1.0f, 0.132f }, { 1.05f, 0.118f }, { 1.1f, 0.11f } };
    recipe.hipSide = { { 0.9f, 0.225f }, { 0.95f, 0.218f }, { 1.0f, 0.205f }, { 1.05f, 0.19f }, { 1.1f, 0.176f } };
    recipe.hipExponent = { { 0.9f, 2.4f }, { 1.1f, 2.2f } };
    recipe.legRings = { 0.74f, 0.775f, 0.81f, 0.84f };
    recipe.hipRings = { 0.94f, 0.98f, 1.02f, 1.045f };
    return recipe;
}

namespace {

void validateRecipe(const ShortsRecipe& r) {
    const auto check = [](bool condition, const char* message) {
        if (!condition) throw std::invalid_argument(message);
    };
    check(r.hemHeight > 0.2f && r.crotchHeight > r.hemHeight + 0.05f && r.seatHeight > r.crotchHeight
            && r.waistHeight > r.seatHeight + 0.08f && r.waistHeight < 1.6f,
        "alturas: barra < virilha < assento < cos");
    check(r.waistbandHeight > 0.005f && r.waistbandHeight < 0.1f && r.waistbandStep >= 0.0f && r.waistbandStep < 0.01f,
        "cos invalido");
    const auto increasing = [&](const std::vector<float>& values, float from, float to, const char* message) {
        for (std::size_t i = 0; i < values.size(); ++i)
            check(std::isfinite(values[i]) && values[i] > from && values[i] < to && (i == 0 || values[i] > values[i - 1]), message);
    };
    increasing(r.legRings, r.hemHeight, r.crotchHeight - 0.01f, "legRings: crescentes entre a barra e a virilha");
    increasing(r.hipRings, r.seatHeight, r.waistHeight - r.waistbandHeight, "hipRings: crescentes entre o assento e o cos");
    check(r.hipPoints >= 8 && r.hipPoints <= 64 && r.crotchPoints >= 2 && r.crotchPoints <= 24, "pontos invalidos");
    check(r.stripeWidth > 0.005f && r.stripeWidth < 0.15f && r.jitter >= 0.0f && r.jitter < 0.01f && r.verticalJitter >= 0.0f
            && r.verticalJitter < 0.02f,
        "faixa ou jitter invalidos");
    check(r.numberHeight > 0.01f && r.numberHeight < 0.2f && r.numberStroke > 0.002f && r.numberStroke < r.numberHeight * 0.4f
            && r.numberAboveHem >= 0.0f && r.numberAboveHem < 0.15f && r.number.size() <= 3,
        "numero invalido");
    for (const char c : r.number) check(c >= '0' && c <= '9', "numero: so digitos");
    for (float h = r.hemHeight; h <= r.seatHeight + 1e-4f; h += 0.01f)
        check(r.legFront(h) > 0.03f && r.legBack(h) > 0.03f && r.legMedial(h) > 0.03f && r.legLateral(h) > 0.03f,
            "perna do calcao sem largura");
    for (float h = r.seatHeight; h <= r.waistHeight + 1e-4f; h += 0.01f)
        check(r.hipFront(h) > 0.05f && r.hipBack(h) > 0.05f && r.hipSide(h) > 0.1f && r.hipExponent(h) >= 1.5f,
            "quadril sem largura");
}

} // namespace

std::optional<ShortsRecipe> parseShortsRecipe(std::string_view json, std::string* error) {
    try {
        const nlohmann::json document = nlohmann::json::parse(json);
        if (!document.is_object() || (document.contains("type") && document.at("type") != "shorts"))
            throw std::invalid_argument("esperado objeto do tipo shorts");
        ShortsRecipe recipe = ShortsRecipe::defaults();
        readCurve(document, "legFront", recipe.legFront, 2.0f);
        readCurve(document, "legBack", recipe.legBack, 2.0f);
        readCurve(document, "legMedial", recipe.legMedial, 2.0f);
        readCurve(document, "legLateral", recipe.legLateral, 2.0f);
        readCurve(document, "hipFront", recipe.hipFront, 2.0f);
        readCurve(document, "hipBack", recipe.hipBack, 2.0f);
        readCurve(document, "hipSide", recipe.hipSide, 2.0f);
        readCurve(document, "hipExponent", recipe.hipExponent, 2.0f);
        readValue(document, "hemHeight", recipe.hemHeight);
        readValue(document, "crotchHeight", recipe.crotchHeight);
        readValue(document, "seatHeight", recipe.seatHeight);
        readValue(document, "waistHeight", recipe.waistHeight);
        readValue(document, "waistbandHeight", recipe.waistbandHeight);
        readValue(document, "waistbandStep", recipe.waistbandStep);
        if (document.contains("legRings")) recipe.legRings = document.at("legRings").get<std::vector<float>>();
        if (document.contains("hipRings")) recipe.hipRings = document.at("hipRings").get<std::vector<float>>();
        readValue(document, "hipPoints", recipe.hipPoints);
        readValue(document, "crotchPoints", recipe.crotchPoints);
        readValue(document, "stripeWidth", recipe.stripeWidth);
        readValue(document, "jitter", recipe.jitter);
        readValue(document, "verticalJitter", recipe.verticalJitter);
        readValue(document, "seed", recipe.seed);
        readValue(document, "number", recipe.number);
        readValue(document, "numberHeight", recipe.numberHeight);
        readValue(document, "numberAboveHem", recipe.numberAboveHem);
        readValue(document, "numberAngle", recipe.numberAngle);
        readValue(document, "numberStroke", recipe.numberStroke);
        if (document.contains("colors")) {
            const nlohmann::json& colors = document.at("colors");
            readColor(colors, "shorts", recipe.shortsColor);
            readColor(colors, "stripe", recipe.stripeColor);
            readColor(colors, "inside", recipe.insideColor);
            readColor(colors, "number", recipe.numberColor);
        }
        validateRecipe(recipe);
        if (error != nullptr) error->clear();
        return recipe;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("receita do calcao invalida: ") + exception.what();
        return std::nullopt;
    }
}

} // namespace MatterEngine::Modeling
