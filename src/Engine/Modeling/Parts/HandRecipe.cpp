#include "Engine/Modeling/Parts/HandRecipe.hpp"

#include "Engine/Modeling/Parts/RecipeJson.hpp"

namespace MatterEngine::Modeling {

HandRecipe HandRecipe::defaults() {
    HandRecipe recipe;
    // As medidas das referencias (a mao de frente, de costas e em 3/4). No
    // pulso, o anel do braco; a palma alarga e afina ate os nos.
    recipe.radial = { { 0.0f, 0.031f }, { 0.02f, 0.0355f }, { 0.045f, 0.039f }, { 0.07f, 0.0405f }, { 0.1f, 0.0415f },
        { 0.13f, 0.0415f } };
    recipe.ulnar = { { 0.0f, 0.033f }, { 0.02f, 0.038f }, { 0.045f, 0.0435f }, { 0.07f, 0.047f }, { 0.1f, 0.0505f },
        { 0.13f, 0.051f } };
    recipe.back = { { 0.0f, 0.018f }, { 0.02f, 0.0165f }, { 0.045f, 0.015f }, { 0.07f, 0.0135f }, { 0.1f, 0.0125f },
        { 0.13f, 0.012f } };
    recipe.palm = { { 0.0f, 0.034f }, { 0.02f, 0.029f }, { 0.045f, 0.024f }, { 0.07f, 0.019f }, { 0.1f, 0.016f },
        { 0.13f, 0.014f } };
    recipe.exponent = { { 0.0f, 2.6f }, { 0.03f, 2.8f }, { 0.1f, 3.0f } };
    recipe.palmRings = { 0.014f, 0.032f, 0.05f, 0.068f, 0.086f };
    recipe.fingers = { {
        { { 0.0105f, 0.0099f, 0.0089f, 0.008f }, { 0.0098f, 0.009f, 0.008f, 0.007f } },
        { { 0.0107f, 0.0101f, 0.0091f, 0.0082f }, { 0.01f, 0.0092f, 0.0082f, 0.0071f } },
        { { 0.01f, 0.0095f, 0.0086f, 0.0077f }, { 0.0094f, 0.0087f, 0.0077f, 0.0067f } },
        { { 0.009f, 0.0085f, 0.0077f, 0.0069f }, { 0.0085f, 0.0079f, 0.007f, 0.0061f } },
    } };
    recipe.thumb = { { 0.0125f, 0.0112f, 0.0101f, 0.009f }, { 0.0112f, 0.0098f, 0.0086f, 0.0074f } };
    return recipe;
}

namespace {

void readSize(const nlohmann::json& document, FingerSize& size) {
    if (document.contains("width")) size.width = document.at("width").get<std::array<float, 4>>();
    if (document.contains("thickness")) size.thickness = document.at("thickness").get<std::array<float, 4>>();
}

void validateRecipe(const HandRecipe& r) {
    const auto check = [](bool condition, const char* message) {
        if (!condition) throw std::invalid_argument(message);
    };
    check(r.palmRings.size() >= 3, "palmRings: ao menos tres aneis (a janela do polegar)");
    for (std::size_t i = 0; i < r.palmRings.size(); ++i)
        check(std::isfinite(r.palmRings[i]) && r.palmRings[i] > 0.003f && r.palmRings[i] < 0.15f
                && (i == 0 || r.palmRings[i] > r.palmRings[i - 1] + 0.005f),
            "palmRings: estacoes crescentes");
    for (float s = 0.0f; s <= 0.14f; s += 0.005f)
        check(r.radial(s) > 0.015f && r.ulnar(s) > 0.015f && r.back(s) > 0.006f && r.palm(s) > 0.006f && r.exponent(s) >= 1.5f
                && r.exponent(s) <= 6.0f,
            "palma sem largura");
    const auto validSize = [](const FingerSize& size) {
        for (std::size_t k = 0; k < 4; ++k)
            if (!(size.width[k] > 0.003f && size.width[k] < 0.03f && size.thickness[k] > 0.003f && size.thickness[k] < 0.03f))
                return false;
        return true;
    };
    for (const FingerSize& finger : r.fingers) check(validSize(finger), "fingers: medidas entre 3 mm e 3 cm");
    check(validSize(r.thumb), "thumb: medidas entre 3 mm e 3 cm");
    check(r.knuckleBump >= 0.0f && r.knuckleBump < 0.008f && r.knuckleWidth >= 0.8f && r.knuckleWidth <= 1.6f, "nos invalidos");
    check(r.palmEdge >= 0.0f && r.palmEdge < 0.015f, "palmEdge invalido");
    check(r.webDepth > 0.1f && r.webDepth < 0.7f, "webDepth: entre 0.1 e 0.7");
    check(r.jointBump >= 0.0f && r.jointBump < 0.005f, "jointBump invalido");
    check(r.tipStart > 0.5f && r.tipStart < 0.95f && r.tipScale > 0.4f && r.tipScale <= 1.0f, "ponta invalida");
    check(r.thumbFromRing >= 0 && static_cast<std::size_t>(r.thumbFromRing) + 2 < r.palmRings.size(),
        "thumbFromRing: tres aneis da palma a partir dele");
    check(r.thenarBulge >= 0.0f && r.thenarBulge < 0.015f && std::abs(r.thumbRoll) < 1.6f, "polegar invalido");
    check(r.jitter >= 0.0f && r.jitter < 0.004f, "jitter invalido");
}

} // namespace

std::optional<HandRecipe> parseHandRecipe(std::string_view json, std::string* error) {
    try {
        const nlohmann::json document = nlohmann::json::parse(json);
        if (!document.is_object() || (document.contains("type") && document.at("type") != "hand"))
            throw std::invalid_argument("esperado objeto do tipo hand");
        HandRecipe recipe = HandRecipe::defaults();
        readCurve(document, "radial", recipe.radial);
        readCurve(document, "ulnar", recipe.ulnar);
        readCurve(document, "back", recipe.back);
        readCurve(document, "palm", recipe.palm);
        readCurve(document, "exponent", recipe.exponent);
        if (document.contains("palmRings")) recipe.palmRings = document.at("palmRings").get<std::vector<float>>();
        readValue(document, "knuckleBump", recipe.knuckleBump);
        readValue(document, "knuckleWidth", recipe.knuckleWidth);
        readValue(document, "palmEdge", recipe.palmEdge);
        readValue(document, "webDepth", recipe.webDepth);
        if (document.contains("fingers")) {
            const nlohmann::json& fingers = document.at("fingers");
            if (!fingers.is_array() || fingers.size() != 4) throw std::invalid_argument("fingers: quatro dedos");
            for (std::size_t i = 0; i < 4; ++i) readSize(fingers.at(i), recipe.fingers[i]);
        }
        if (document.contains("thumb")) readSize(document.at("thumb"), recipe.thumb);
        readValue(document, "jointBump", recipe.jointBump);
        readValue(document, "tipStart", recipe.tipStart);
        readValue(document, "tipScale", recipe.tipScale);
        readValue(document, "thumbFromRing", recipe.thumbFromRing);
        readValue(document, "thenarBulge", recipe.thenarBulge);
        readValue(document, "thumbRoll", recipe.thumbRoll);
        readValue(document, "jitter", recipe.jitter);
        readValue(document, "seed", recipe.seed);
        if (document.contains("colors")) {
            readColor(document.at("colors"), "skin", recipe.skinColor);
            readColor(document.at("colors"), "nail", recipe.nailColor);
        }
        validateRecipe(recipe);
        if (error != nullptr) error->clear();
        return recipe;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("receita da mao invalida: ") + exception.what();
        return std::nullopt;
    }
}

} // namespace MatterEngine::Modeling
