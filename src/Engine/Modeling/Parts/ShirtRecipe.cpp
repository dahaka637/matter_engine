#include "Engine/Modeling/Parts/ShirtRecipe.hpp"

#include "Engine/Modeling/Parts/RecipeJson.hpp"

namespace MatterEngine::Modeling {

ShirtRecipe ShirtRecipe::defaults() {
    ShirtRecipe recipe;
    // As medidas das referencias (camisa de frente, de lado e de costas).
    recipe.torsoCenterX = { { 1.0f, -0.026f }, { 1.08f, -0.021f }, { 1.2f, -0.009f }, { 1.32f, -0.021f }, { 1.43f, -0.042f },
        { 1.52f, -0.05f }, { 1.6f, -0.047f }, { 1.645f, -0.035f }, { 1.66f, -0.033f } };
    recipe.torsoFront = { { 1.0f, 0.154f }, { 1.04f, 0.15f }, { 1.08f, 0.144f }, { 1.12f, 0.135f }, { 1.16f, 0.13f },
        { 1.2f, 0.123f }, { 1.24f, 0.127f }, { 1.28f, 0.135f }, { 1.32f, 0.14f }, { 1.36f, 0.147f }, { 1.4f, 0.153f },
        { 1.43f, 0.156f }, { 1.48f, 0.147f }, { 1.52f, 0.132f }, { 1.56f, 0.108f }, { 1.59f, 0.09f }, { 1.6f, 0.083f },
        { 1.612f, 0.078f }, { 1.62f, 0.074f }, { 1.632f, 0.071f }, { 1.645f, 0.069f }, { 1.66f, 0.068f } };
    recipe.torsoBack = { { 1.0f, 0.129f }, { 1.04f, 0.126f }, { 1.08f, 0.118f }, { 1.12f, 0.11f }, { 1.16f, 0.099f },
        { 1.2f, 0.094f }, { 1.24f, 0.099f }, { 1.28f, 0.102f }, { 1.32f, 0.108f }, { 1.36f, 0.11f }, { 1.4f, 0.112f },
        { 1.43f, 0.114f }, { 1.48f, 0.113f }, { 1.52f, 0.11f }, { 1.56f, 0.107f }, { 1.59f, 0.097f }, { 1.6f, 0.09f },
        { 1.612f, 0.085f }, { 1.62f, 0.08f }, { 1.632f, 0.074f }, { 1.645f, 0.07f }, { 1.66f, 0.068f } };
    recipe.torsoSide = { { 1.0f, 0.21f }, { 1.04f, 0.211f }, { 1.08f, 0.2015f }, { 1.12f, 0.193f }, { 1.16f, 0.185f },
        { 1.2f, 0.178f }, { 1.24f, 0.1815f }, { 1.28f, 0.1895f }, { 1.32f, 0.1965f }, { 1.36f, 0.2085f }, { 1.4f, 0.2205f },
        { 1.43f, 0.226f }, { 1.48f, 0.226f }, { 1.52f, 0.224f }, { 1.56f, 0.22f }, { 1.59f, 0.212f }, { 1.6f, 0.205f },
        { 1.612f, 0.18f }, { 1.62f, 0.14f }, { 1.632f, 0.1f }, { 1.645f, 0.082f }, { 1.66f, 0.075f } };
    recipe.torsoExponent = { { 1.0f, 2.0f }, { 1.43f, 2.1f }, { 1.52f, 2.8f }, { 1.6f, 3.0f }, { 1.63f, 2.4f }, { 1.66f, 2.2f } };
    recipe.torsoSideExponent = { { 1.0f, 1.5f }, { 1.2f, 1.45f }, { 1.32f, 1.45f }, { 1.43f, 1.6f }, { 1.5f, 2.4f },
        { 1.6f, 3.0f }, { 1.63f, 2.4f }, { 1.66f, 2.2f } };
    recipe.sleeveRadius = { { 0.0f, 0.09f }, { 0.023f, 0.0915f }, { 0.063f, 0.0895f }, { 0.123f, 0.075f }, { 0.183f, 0.0695f },
        { 0.223f, 0.0665f }, { 0.3f, 0.065f } };
    recipe.sleeveLift = { { 0.0f, 0.016f }, { 0.063f, 0.0135f }, { 0.123f, 0.009f }, { 0.183f, -0.0015f }, { 0.223f, -0.0055f },
        { 0.3f, -0.009f } };
    recipe.sleeveForward = { { 0.0f, 0.0f }, { 0.3f, -0.003f } };
    recipe.rings = { 1.045f, 1.085f, 1.125f, 1.165f, 1.205f, 1.245f, 1.285f, 1.325f, 1.365f, 1.4f };
    recipe.stripeHalfWidth = { { 1.01f, 0.042f }, { 1.2f, 0.038f }, { 1.43f, 0.048f } };
    recipe.sleeveRings = { 0.07f, 0.12f, 0.17f };
    recipe.vNeckHeight = { { 0.0f, 1.54f }, { 0.02f, 1.55f }, { 0.04f, 1.561f }, { 0.06f, 1.576f }, { 0.08f, 1.588f } };
    recipe.backNeckHeight = { { 0.0f, 1.621f }, { 0.04f, 1.625f }, { 0.06f, 1.628f }, { 0.08f, 1.632f } };
    return recipe;
}

namespace {

void validateRecipe(const ShirtRecipe& r) {
    const auto check = [](bool condition, const char* message) {
        if (!condition) throw std::invalid_argument(message);
    };
    check(r.hemHeight > 0.5f && r.armpitHeight > r.hemHeight + 0.15f && r.armpitHeight < 1.8f,
        "alturas: barra < axila");
    check(r.tuckHeight > r.hemHeight + 0.005f && r.tuckHeight < r.hemHeight + 0.1f, "tuckHeight: logo acima da barra");
    check(r.hemClearance >= 0.0f && r.hemClearance < 0.05f, "hemClearance invalido");
    for (std::size_t i = 0; i < r.rings.size(); ++i)
        check(std::isfinite(r.rings[i]) && r.rings[i] > r.hemHeight + 0.005f && r.rings[i] < r.armpitHeight - 0.005f
                && (i == 0 || r.rings[i] > r.rings[i - 1] + 0.005f),
            "rings: crescentes entre a barra e a axila");
    check(r.backColumns >= 2 && r.backColumns <= 24 && r.frontColumns >= 2 && r.frontColumns <= 24,
        "colunas: de 2 a 24");
    check(r.backColumns == r.frontColumns, "colunas: a frente e as costas dividem a costura do ombro (mesmo numero)");
    check(r.armholeDepth > 0.08f && r.armholeDepth < 0.4f && r.armholeRows >= 3 && r.armholeRows <= 24, "cava invalida");
    check(std::abs(r.shoulderPointAngle) < 60.0f, "shoulderPointAngle: ate 60 graus");
    check(!r.vNeckHeight.empty() && !r.backNeckHeight.empty() && r.vNeckHeight.points().front().first == 0.0f
            && r.backNeckHeight.points().front().first == 0.0f,
        "vNeckHeight/backNeckHeight: curvas a partir do meio (0)");
    check(r.vNeckHeight.points().back().first < r.neckPointSide && r.backNeckHeight.points().back().first < r.neckPointSide,
        "a borda da gola termina antes do ponto do pescoco (neckPointSide)");
    check(r.vNeckHeight(0.0f) > r.armpitHeight + 0.02f && r.backNeckHeight(0.0f) > r.armpitHeight + 0.02f,
        "a gola fica acima da axila");
    check(r.neckPointSide > 0.03f && r.neckPointSide < 0.2f && r.vSteps >= 2 && r.vSteps <= 16 && r.backNeckSteps >= 2
            && r.backNeckSteps <= 16 && r.armholeRows > r.vSteps && r.armholeRows > r.backNeckSteps,
        "gola invalida (os passos do V e de tras menores que armholeRows)");
    check(r.collarWidth > 0.005f && r.collarWidth < 0.06f && r.collarRaise >= 0.0f && r.collarRaise < 0.01f
            && r.collarDepth > 0.0f && r.collarDepth < 0.02f,
        "faixa da gola invalida");
    for (std::size_t i = 0; i < r.sleeveRings.size(); ++i)
        check(std::isfinite(r.sleeveRings[i]) && r.sleeveRings[i] > 0.0f
                && r.sleeveRings[i] < r.sleeveLength - r.cuffWidth - 0.01f
                && (i == 0 || r.sleeveRings[i] > r.sleeveRings[i - 1] + 0.005f),
            "sleeveRings: crescentes entre o ombro e o punho");
    check(r.sleeveLength > 0.05f && r.sleeveLength < 0.6f && std::abs(r.sleeveEndTilt) < 0.6f && r.cuffWidth > 0.005f
            && r.cuffWidth < r.sleeveLength * 0.5f && r.cuffOpening > 0.02f && std::abs(r.cuffOpeningForward) < 0.05f,
        "manga invalida");
    check(r.frontNumberHeight > 0.01f && r.frontNumberHeight < 0.3f && r.backNumberHeight > 0.01f && r.backNumberHeight < 0.4f
            && r.number.size() <= 3 && r.numberStroke >= 0.08f && r.numberStroke <= 0.28f,
        "numero invalido");
    for (const char c : r.number) check(c >= '0' && c <= '9', "numero: so digitos");
    check(r.jitter >= 0.0f && r.jitter < 0.01f && r.slideJitter >= 0.0f && r.slideJitter < 0.02f && r.ringSlide >= 0.0f
            && r.ringSlide <= 0.45f && r.smoothing >= 0
            && r.smoothing <= 64,
        "facetado invalido");
    check(r.sleeveBlend >= 0.0f && r.sleeveBlend < 0.15f, "sleeveBlend invalido");
    for (float h = r.hemHeight; h <= r.backNeckHeight(0.0f) + 1e-4f; h += 0.01f)
        check(r.torsoFront(h) > 0.02f && r.torsoBack(h) > 0.02f && r.torsoSide(h) > 0.03f && r.torsoExponent(h) >= 1.5f
                && (r.torsoSideExponent.empty() || r.torsoSideExponent(h) >= 1.1f),
            "tronco sem largura (ou expoentes abaixo de 1,5 e 1,1)");
    for (float s = 0.0f; s <= r.sleeveLength + 1e-4f; s += 0.01f)
        check(r.sleeveRadius(s) > r.cuffOpening, "manga mais fina que a abertura do punho");
    for (float h = r.hemHeight; h <= r.armpitHeight; h += 0.01f)
        check(r.stripeHalfWidth(h) > 0.005f && r.stripeHalfWidth(h) < 0.1f, "faixa do lado invalida");
}

} // namespace

std::optional<ShirtRecipe> parseShirtRecipe(std::string_view json, std::string* error) {
    try {
        const nlohmann::json document = nlohmann::json::parse(json);
        if (!document.is_object() || (document.contains("type") && document.at("type") != "shirt"))
            throw std::invalid_argument("esperado objeto do tipo shirt");
        ShirtRecipe recipe = ShirtRecipe::defaults();
        readCurve(document, "torsoCenterX", recipe.torsoCenterX, 2.5f);
        readCurve(document, "torsoFront", recipe.torsoFront, 2.5f);
        readCurve(document, "torsoBack", recipe.torsoBack, 2.5f);
        readCurve(document, "torsoSide", recipe.torsoSide, 2.5f);
        readCurve(document, "torsoExponent", recipe.torsoExponent, 2.5f);
        readCurve(document, "torsoSideExponent", recipe.torsoSideExponent, 2.5f);
        readCurve(document, "sleeveRadius", recipe.sleeveRadius, 1.0f);
        readCurve(document, "sleeveLift", recipe.sleeveLift, 1.0f);
        readCurve(document, "sleeveForward", recipe.sleeveForward, 1.0f);
        readValue(document, "sleeveBlend", recipe.sleeveBlend);
        readValue(document, "hemHeight", recipe.hemHeight);
        readValue(document, "armpitHeight", recipe.armpitHeight);
        if (document.contains("rings")) recipe.rings = document.at("rings").get<std::vector<float>>();
        readValue(document, "tuckHeight", recipe.tuckHeight);
        readValue(document, "hemClearance", recipe.hemClearance);
        readValue(document, "backColumns", recipe.backColumns);
        readValue(document, "frontColumns", recipe.frontColumns);
        readCurve(document, "stripeHalfWidth", recipe.stripeHalfWidth, 2.5f);
        readValue(document, "armholeDepth", recipe.armholeDepth);
        readValue(document, "armholeRows", recipe.armholeRows);
        readValue(document, "shoulderPointAngle", recipe.shoulderPointAngle);
        readCurve(document, "vNeckHeight", recipe.vNeckHeight, 0.3f);
        readCurve(document, "backNeckHeight", recipe.backNeckHeight, 0.3f);
        readValue(document, "neckPointX", recipe.neckPointX);
        readValue(document, "neckPointSide", recipe.neckPointSide);
        readValue(document, "vSteps", recipe.vSteps);
        readValue(document, "backNeckSteps", recipe.backNeckSteps);
        readValue(document, "collarWidth", recipe.collarWidth);
        readValue(document, "collarRaise", recipe.collarRaise);
        readValue(document, "collarDepth", recipe.collarDepth);
        if (document.contains("sleeveRings")) recipe.sleeveRings = document.at("sleeveRings").get<std::vector<float>>();
        readValue(document, "sleeveLength", recipe.sleeveLength);
        readValue(document, "sleeveEndTilt", recipe.sleeveEndTilt);
        readValue(document, "cuffWidth", recipe.cuffWidth);
        readValue(document, "cuffOpening", recipe.cuffOpening);
        readValue(document, "cuffOpeningForward", recipe.cuffOpeningForward);
        readValue(document, "number", recipe.number);
        readValue(document, "frontNumberHeight", recipe.frontNumberHeight);
        readValue(document, "frontNumberCenter", recipe.frontNumberCenter);
        readValue(document, "backNumberHeight", recipe.backNumberHeight);
        readValue(document, "backNumberCenter", recipe.backNumberCenter);
        readValue(document, "numberStroke", recipe.numberStroke);
        readValue(document, "jitter", recipe.jitter);
        readValue(document, "slideJitter", recipe.slideJitter);
        readValue(document, "ringSlide", recipe.ringSlide);
        readValue(document, "seed", recipe.seed);
        readValue(document, "smoothing", recipe.smoothing);
        if (document.contains("colors")) {
            const nlohmann::json& colors = document.at("colors");
            readColor(colors, "shirt", recipe.shirtColor);
            readColor(colors, "trim", recipe.trimColor);
            readColor(colors, "inside", recipe.insideColor);
            readColor(colors, "number", recipe.numberColor);
        }
        validateRecipe(recipe);
        if (error != nullptr) error->clear();
        return recipe;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("receita da camisa invalida: ") + exception.what();
        return std::nullopt;
    }
}

} // namespace MatterEngine::Modeling
