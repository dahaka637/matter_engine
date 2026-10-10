#include "Engine/Modeling/Parts/BootRecipe.hpp"

#include "Engine/Modeling/Parts/RecipeJson.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace MatterEngine::Modeling {

BootRecipe BootRecipe::defaults() {
    BootRecipe recipe;
    // Gerado de partes/chuteira.json (ferramentas_chuteira/sincronizar_padroes.py):
    // o padrao e o desenho atual; o teste compara as duas malhas.
    recipe.medialHalfWidth = { { 0.0f, 0.03f }, { 0.04f, 0.032f }, { 0.08f, 0.034f }, { 0.12f, 0.035f }, { 0.16f, 0.0355f },
        { 0.22f, 0.0345f }, { 0.3f, 0.0335f }, { 0.38f, 0.035f }, { 0.46f, 0.0385f }, { 0.54f, 0.043f },
        { 0.62f, 0.048f }, { 0.68f, 0.0495f }, { 0.74f, 0.0475f }, { 0.8f, 0.0445f }, { 0.86f, 0.0405f },
        { 0.91f, 0.0365f }, { 0.95f, 0.0335f }, { 1.0f, 0.0305f } };
    recipe.lateralHalfWidth = { { 0.0f, 0.031f }, { 0.04f, 0.034f }, { 0.08f, 0.036f }, { 0.12f, 0.037f }, { 0.16f, 0.0385f },
        { 0.22f, 0.04f }, { 0.3f, 0.0425f }, { 0.38f, 0.045f }, { 0.46f, 0.048f }, { 0.54f, 0.052f },
        { 0.62f, 0.054f }, { 0.68f, 0.054f }, { 0.74f, 0.052f }, { 0.8f, 0.048f }, { 0.86f, 0.042f },
        { 0.91f, 0.036f }, { 0.95f, 0.031f }, { 1.0f, 0.026f } };
    recipe.topHeight = { { 0.0f, 0.121f }, { 0.045f, 0.12f }, { 0.1f, 0.122f }, { 0.2f, 0.126f }, { 0.33f, 0.123f },
        { 0.37f, 0.124f }, { 0.4f, 0.127f }, { 0.445f, 0.1205f }, { 0.5f, 0.1125f }, { 0.56f, 0.102f },
        { 0.62f, 0.091f }, { 0.68f, 0.0815f }, { 0.74f, 0.0745f }, { 0.8f, 0.069f }, { 0.86f, 0.065f },
        { 0.92f, 0.0625f }, { 0.96f, 0.0605f }, { 1.0f, 0.0585f } };
    recipe.collarHeight = { { 0.0f, 0.121f }, { 0.03f, 0.121f }, { 0.06f, 0.116f }, { 0.09f, 0.111f }, { 0.13f, 0.106f },
        { 0.18f, 0.101f }, { 0.24f, 0.098f }, { 0.29f, 0.099f }, { 0.32f, 0.101f }, { 0.345f, 0.106f },
        { 0.37f, 0.114f }, { 0.39f, 0.122f }, { 0.41f, 0.129f } };
    recipe.soleBottom = { { 0.0f, 0.023f }, { 0.04f, 0.0185f }, { 0.06f, 0.018f }, { 0.9f, 0.018f }, { 0.94f, 0.0195f },
        { 0.97f, 0.022f }, { 0.99f, 0.0265f }, { 1.0f, 0.031f } };
    recipe.soleRelief = { { 0.0f, 0.0f }, { 0.12f, 0.002f }, { 0.24f, 0.003f }, { 0.31f, 0.001f }, { 0.41f, 0.01f },
        { 0.51f, 0.0015f }, { 0.69f, 0.009f }, { 0.88f, 0.002f }, { 0.998f, 0.0f } };
    recipe.tongueHalfWidth = { { 0.401f, 0.02f }, { 0.412f, 0.022f }, { 0.424f, 0.022f }, { 0.434f, 0.02f } };
    recipe.collarLeanProfile = { { 0.0f, 0.016f }, { 0.1f, 0.018f }, { 0.18f, 0.04f }, { 0.23f, 0.045f }, { 0.4f, 0.045f } };
    recipe.solePanelStations = { 0.0f, 0.11f, 0.28f, 0.5f, 0.72f, 0.92f, 0.998f };
    recipe.stations = { 0.0f, 0.006f, 0.02f, 0.045f, 0.085f, 0.15f, 0.235f, 0.32f, 0.36f, 0.4f, 0.47f, 0.565f, 0.66f,
        0.755f, 0.835f, 0.9f, 0.95f, 0.975f, 0.99f, 0.9985f };
    recipe.studs = { { 0.1f, 0.021f, 0.0128f, 0.01f }, { 0.1f, -0.023f, 0.0128f, 0.01f },
        { 0.245f, 0.021f, 0.0128f, 0.01f }, { 0.245f, -0.025f, 0.0128f, 0.01f },
        { 0.575f, 0.028f, 0.0118f, 0.0092f }, { 0.58f, -0.035f, 0.0118f, 0.0092f },
        { 0.715f, 0.035f, 0.0118f, 0.0092f }, { 0.71f, -0.038f, 0.0118f, 0.0092f },
        { 0.862f, 0.025f, 0.011f, 0.0086f }, { 0.858f, -0.025f, 0.011f, 0.0086f } };
    return recipe;
}

namespace {

void validateRecipe(const BootRecipe& r) {
    const auto check = [](bool condition, const char* message) {
        if (!condition) throw std::invalid_argument(message);
    };
    const auto stations = [&](const std::vector<float>& values) {
        check(values.size() >= 2 && values.size() <= 128, "estacoes: esperado de 2 a 128 valores");
        for (std::size_t i = 0; i < values.size(); ++i)
            check(std::isfinite(values[i]) && values[i] >= 0 && values[i] <= 1 && (i == 0 || values[i] > values[i - 1]),
                "estacoes precisam crescer estritamente em [0,1]");
    };
    stations(r.stations);
    if (!r.solePanelStations.empty()) stations(r.solePanelStations);
    check(r.length > 0.05f && r.length <= 0.6f, "length fora do intervalo [0.05,0.6]");
    check(r.soleThickness > 0 && r.soleThickness < 0.05f && r.soleOverhang > 0.0016f && r.soleOverhang < 0.02f,
        "espessura ou borda da sola invalida");
    check(r.studSides >= 4 && r.studSides <= 12 && r.studs.size() <= 32 && r.studBevel >= 0,
        "travas: lados, quantidade ou chanfro invalido");
    for (const auto& stud : r.studs)
        check(std::isfinite(stud.t) && std::isfinite(stud.v) && std::isfinite(stud.baseRadius) && std::isfinite(stud.tipRadius)
            && stud.t >= 0 && stud.t <= 1 && stud.baseRadius > 0 && stud.baseRadius <= 0.05f
            && stud.tipRadius > 0 && stud.tipRadius <= stud.baseRadius && std::abs(stud.v) < 0.15f,
            "trava com posicao ou raios invalidos");
    check(r.laceRows >= 2 && r.laceRows <= 16 && r.laceStart > r.tongueTop && r.laceEnd > r.laceStart && r.laceEnd < 1,
        "fileiras do cadarco invalidas");
    check(r.laceWidth > 0 && r.laceThickness > 0 && r.crossWidth > 0 && r.crossThickness > 0
        && r.laceClearance > 0 && r.slotLength > 0 && r.reliefBevel >= 0 && r.reliefBevel < 0.005f,
        "cadarco com dimensoes invalidas");
    check(r.openingStart >= 0 && r.openingFull > r.openingStart && r.tongueStart > r.openingFull
        && r.tongueTop > r.tongueStart && r.tongueTop < 1 && r.rimThickness > 0 && r.collarLean > 0,
        "abertura invalida");
    check(r.heelRound > 0 && r.heelRound < 0.5f && r.toeRound > 0 && r.toeRound < 0.5f
        && r.heelEndScale > 0 && r.heelEndScale <= 1 && r.toeEndScale > 0 && r.toeEndScale <= 1
        && r.heelRoundExponent >= 1 && r.toeRoundExponent >= 1 && r.noseExponent >= 1 && r.sideBulge > 0
        && r.heelSideBulge >= 0 && r.heelSideBulge < 0.5f && r.toeJitter >= 0 && r.toeJitter <= 2
        && r.toeTipReach >= 0 && r.toeTipReach < 0.01f && r.jitterSpacing > 0.001f && r.jitterSpacing < 0.1f,
        "contorno das pontas invalido");
    for (int i = 0; i <= 100; ++i) {
        const float t = static_cast<float>(i) / 100.0f;
        check(r.medialHalfWidth(t) > 0.005f && r.lateralHalfWidth(t) > 0.005f && r.soleBottom(t) > r.studBevel
            && r.topHeight(t) > r.soleBottom(t) + r.soleThickness && r.soleRelief(t) >= 0
            && r.soleRelief(t) < r.soleBottom(t), "perfis sem altura ou largura suficiente");
    }
    if (!r.tongueHalfWidth.empty())
        check(r.tongueHalfWidth.points().size() >= 2 && r.tongueThickness > 0
                && r.tongueHalfWidth.points().front().first >= r.tongueTop,
            "lingua sem largura ou espessura, ou comecando antes do alto da lingua (sobre a gola)");
    check(r.laceDive > 0.001f && r.laceDive < 0.02f && r.crossDive > 0.001f && r.crossDive < 0.02f,
        "mergulho das pontas do cadarco invalido");
    check(r.soleToeOverhang >= 0 && r.soleToeOverhang < 0.02f && r.soleToeThinning >= 0 && r.soleToeThinning < 0.9f,
        "ponta da sola invalida");
}

} // namespace

std::optional<BootRecipe> parseBootRecipe(std::string_view json, std::string* error) {
    try {
        const nlohmann::json document = nlohmann::json::parse(json);
        if (!document.is_object() || (document.contains("type") && document.at("type") != "boot"))
            throw std::invalid_argument("esperado objeto do tipo boot");
        BootRecipe recipe = BootRecipe::defaults();
        readValue(document, "length", recipe.length);
        readValue(document, "heelBehindAnkle", recipe.heelBehindAnkle);
        readCurve(document, "medialHalfWidth", recipe.medialHalfWidth);
        readCurve(document, "lateralHalfWidth", recipe.lateralHalfWidth);
        readCurve(document, "topHeight", recipe.topHeight);
        readCurve(document, "collarHeight", recipe.collarHeight);
        readCurve(document, "soleBottom", recipe.soleBottom);
        readValue(document, "soleThickness", recipe.soleThickness);
        readValue(document, "soleOverhang", recipe.soleOverhang);
        readValue(document, "soleToeOverhang", recipe.soleToeOverhang);
        readValue(document, "soleToeThinning", recipe.soleToeThinning);
        readCurve(document, "soleRelief", recipe.soleRelief);
        readValue(document, "solePanelStations", recipe.solePanelStations);
        readValue(document, "heelRound", recipe.heelRound);
        readValue(document, "heelRoundExponent", recipe.heelRoundExponent);
        readValue(document, "heelEndScale", recipe.heelEndScale);
        readValue(document, "toeRound", recipe.toeRound);
        readValue(document, "toeRoundExponent", recipe.toeRoundExponent);
        readValue(document, "toeEndScale", recipe.toeEndScale);
        readValue(document, "toeTipReach", recipe.toeTipReach);
        readValue(document, "noseRound", recipe.noseRound);
        readValue(document, "noseExponent", recipe.noseExponent);
        readValue(document, "heelSlant", recipe.heelSlant);
        readValue(document, "heelBulge", recipe.heelBulge);
        readValue(document, "heelSlantReach", recipe.heelSlantReach);
        readValue(document, "sectionExponent", recipe.sectionExponent);
        readValue(document, "sideBulge", recipe.sideBulge);
        readValue(document, "heelSideBulge", recipe.heelSideBulge);
        readValue(document, "openingStart", recipe.openingStart);
        readValue(document, "openingFull", recipe.openingFull);
        readValue(document, "openingRound", recipe.openingRound);
        readValue(document, "tongueStart", recipe.tongueStart);
        readValue(document, "tongueTop", recipe.tongueTop);
        readValue(document, "tongueLean", recipe.tongueLean);
        readValue(document, "tongueWallBlend", recipe.tongueWallBlend);
        readCurve(document, "tongueHalfWidth", recipe.tongueHalfWidth);
        readValue(document, "tongueThickness", recipe.tongueThickness);
        readValue(document, "rimThickness", recipe.rimThickness);
        readValue(document, "collarLean", recipe.collarLean);
        readCurve(document, "collarLeanProfile", recipe.collarLeanProfile);
        readValue(document, "collarLateralDrop", recipe.collarLateralDrop);
        readValue(document, "insoleAboveSole", recipe.insoleAboveSole);
        if (document.contains("stations")) recipe.stations = document.at("stations").get<std::vector<float>>();
        if (document.contains("studs")) {
            recipe.studs.clear();
            for (const nlohmann::json& entry : document.at("studs"))
                recipe.studs.push_back({ entry.value("t", 0.5f), entry.value("v", 0.0f), entry.value("baseRadius", 0.012f),
                    entry.value("tipRadius", 0.0094f) });
        }
        readValue(document, "studSides", recipe.studSides);
        readValue(document, "studBevel", recipe.studBevel);
        readValue(document, "laceStart", recipe.laceStart);
        readValue(document, "laceEnd", recipe.laceEnd);
        readValue(document, "laceRows", recipe.laceRows);
        readValue(document, "laceHalfSpacingStart", recipe.laceHalfSpacingStart);
        readValue(document, "laceHalfSpacingEnd", recipe.laceHalfSpacingEnd);
        readValue(document, "laceWidth", recipe.laceWidth);
        readValue(document, "laceThickness", recipe.laceThickness);
        readValue(document, "laceOverhang", recipe.laceOverhang);
        readValue(document, "crossWidth", recipe.crossWidth);
        readValue(document, "crossThickness", recipe.crossThickness);
        readValue(document, "crossInset", recipe.crossInset);
        readValue(document, "crossEndShift", recipe.crossEndShift);
        readValue(document, "laceChannel", recipe.laceChannel);
        readValue(document, "laceClearance", recipe.laceClearance);
        readValue(document, "laceDive", recipe.laceDive);
        readValue(document, "crossDive", recipe.crossDive);
        readValue(document, "slotLength", recipe.slotLength);
        readValue(document, "reliefBevel", recipe.reliefBevel);
        readValue(document, "jitter", recipe.jitter);
        readValue(document, "topJitter", recipe.topJitter);
        readValue(document, "toeJitter", recipe.toeJitter);
        readValue(document, "jitterSpacing", recipe.jitterSpacing);
        readValue(document, "lengthJitter", recipe.lengthJitter);
        readValue(document, "seed", recipe.seed);
        if (document.contains("colors")) {
            const nlohmann::json& colors = document.at("colors");
            readColor(colors, "upper", recipe.upperColor);
            readColor(colors, "sole", recipe.soleColor);
            readColor(colors, "studs", recipe.studColor);
            readColor(colors, "laces", recipe.laceColor);
            readColor(colors, "eyelets", recipe.eyeletColor);
        }
        validateRecipe(recipe);
        if (error != nullptr) error->clear();
        return recipe;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("receita da chuteira invalida: ") + exception.what();
        return std::nullopt;
    }
}

} // namespace MatterEngine::Modeling
