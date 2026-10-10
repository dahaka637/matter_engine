#include "Engine/Modeling/Parts/HeadRecipe.hpp"

#include "Engine/Modeling/Parts/RecipeJson.hpp"

namespace MatterEngine::Modeling {

HeadRecipe HeadRecipe::defaults() {
    HeadRecipe recipe;
    // As medidas das referencias (a cabeca de frente, de costas e de lado na
    // escala do corpo; o rosto de rosto.png, com o alto a 1,905 m e o queixo
    // a 1,672): o cranio com 16,5 cm de largura e 23 de fundo.
    recipe.centerX = { { 1.66f, -0.03f }, { 1.69f, -0.016f }, { 1.72f, -0.014f }, { 1.905f, -0.014f } };
    recipe.front = { { 1.66f, 0.064f }, { 1.672f, 0.098f }, { 1.69f, 0.1f }, { 1.79f, 0.1f }, { 1.815f, 0.102f },
        { 1.835f, 0.101f }, { 1.85f, 0.097f }, { 1.865f, 0.093f }, { 1.88f, 0.0845f }, { 1.89f, 0.0715f },
        { 1.898f, 0.055f }, { 1.903f, 0.03f }, { 1.905f, 0.004f } };
    recipe.back = { { 1.66f, 0.066f }, { 1.672f, 0.08f }, { 1.69f, 0.08f }, { 1.705f, 0.079f }, { 1.725f, 0.084f },
        { 1.745f, 0.092f }, { 1.765f, 0.098f }, { 1.785f, 0.105f }, { 1.805f, 0.108f }, { 1.825f, 0.104f }, { 1.845f, 0.099f },
        { 1.865f, 0.088f }, { 1.88f, 0.074f }, { 1.89f, 0.059f }, { 1.898f, 0.04f }, { 1.903f, 0.021f },
        { 1.905f, 0.004f } };
    recipe.side = { { 1.66f, 0.066f }, { 1.69f, 0.068f }, { 1.705f, 0.0695f }, { 1.725f, 0.071f }, { 1.745f, 0.073f },
        { 1.765f, 0.0755f }, { 1.785f, 0.0785f }, { 1.805f, 0.0815f }, { 1.825f, 0.083f }, { 1.845f, 0.08f },
        { 1.86f, 0.0752f }, { 1.87f, 0.0705f }, { 1.88f, 0.06f }, { 1.89f, 0.0465f }, { 1.898f, 0.031f },
        { 1.903f, 0.016f }, { 1.905f, 0.004f } };
    recipe.exponent = { { 1.66f, 2.2f }, { 1.75f, 2.3f }, { 1.8f, 2.2f }, { 1.905f, 2.0f } };
    recipe.sideExponent = { { 1.66f, 2.2f }, { 1.905f, 2.0f } };
    recipe.frontExponent = { { 1.66f, 1.9f }, { 1.672f, 1.75f }, { 1.69f, 1.75f }, { 1.74f, 1.8f }, { 1.78f, 1.85f },
        { 1.81f, 1.95f }, { 1.85f, 2.0f }, { 1.905f, 2.0f } };
    recipe.frontSideExponent = { { 1.66f, 2.2f }, { 1.69f, 1.85f }, { 1.7f, 1.8f }, { 1.76f, 1.8f }, { 1.8f, 1.85f },
        { 1.85f, 1.95f }, { 1.905f, 2.0f } };
    // O perfil do meio sem o nariz (rosto.png de lado): o queixo (em baixo,
    // na frente), o sulco sob o labio, o labio de baixo, a linha da boca, o
    // labio de cima, a base do nariz, a raiz entre os olhos, a glabela e a
    // testa.
    recipe.profile = { { 1.672f, 0.08f }, { 1.676f, 0.0847f }, { 1.682f, 0.0874f }, { 1.688f, 0.0883f },
        { 1.694f, 0.0874f }, { 1.698f, 0.0858f }, { 1.702f, 0.0895f }, { 1.706f, 0.0933f }, { 1.711f, 0.0933f },
        { 1.715f, 0.0921f }, { 1.718f, 0.0952f }, { 1.721f, 0.0967f }, { 1.727f, 0.0958f }, { 1.735f, 0.0946f },
        { 1.76f, 0.0936f }, { 1.789f, 0.0928f }, { 1.801f, 0.096f }, { 1.812f, 0.0952f }, { 1.83f, 0.0895f },
        { 1.853f, 0.0848f } };
    recipe.profileWidth = { { 1.672f, 0.024f }, { 1.688f, 0.026f }, { 1.698f, 0.03f }, { 1.706f, 0.034f },
        { 1.715f, 0.038f }, { 1.721f, 0.038f }, { 1.735f, 0.034f }, { 1.76f, 0.028f }, { 1.789f, 0.02f },
        { 1.801f, 0.03f }, { 1.83f, 0.05f }, { 1.853f, 0.06f } };
    // O nariz: a base entre os labios, a ponta (o ponto mais a frente), a
    // ponte reta ate a raiz entre os olhos; estreito na base (a columela) e
    // na ponta, alargando na raiz; a ponte de frente reta e lados a pique.
    recipe.noseProfile = { { 1.735f, 0.0946f }, { 1.7395f, 0.1008f }, { 1.744f, 0.1072f }, { 1.748f, 0.1109f },
        { 1.7525f, 0.1098f }, { 1.758f, 0.1068f }, { 1.766f, 0.1031f }, { 1.778f, 0.0976f }, { 1.789f, 0.0928f } };
    recipe.noseWidth = { { 1.735f, 0.0045f }, { 1.7395f, 0.011f }, { 1.744f, 0.0138f }, { 1.748f, 0.0145f },
        { 1.756f, 0.0135f }, { 1.766f, 0.0135f }, { 1.778f, 0.0145f }, { 1.789f, 0.0155f } };
    recipe.noseShape = { { 1.735f, 2.0f }, { 1.744f, 3.0f }, { 1.752f, 3.0f }, { 1.762f, 2.8f }, { 1.775f, 3.0f },
        { 1.789f, 2.5f } };
    // As sobrancelhas (rosto.png de frente): grossas, quase retas, a de
    // baixo logo acima da palpebra; o fim de dentro cortado em diagonal e a
    // ponta de fora fina.
    recipe.browTop = { { 0.0124f, 1.8068f }, { 0.022f, 1.808f }, { 0.032f, 1.809f }, { 0.042f, 1.81f }, { 0.052f, 1.8112f } };
    recipe.browBottom = { { 0.0083f, 1.796f }, { 0.02f, 1.797f }, { 0.03f, 1.798f }, { 0.042f, 1.7995f },
        { 0.0538f, 1.8007f } };
    // A linha da mandibula (rosto.png de frente e de lado): reta embaixo do
    // queixo, subindo a uns 35 graus ate o canto (a uns 96 graus em volta) e
    // dai para cima ate o lobulo da orelha.
    recipe.jawLine = { { 0.0f, 1.672f }, { 10.0f, 1.672f }, { 18.5f, 1.6725f }, { 24.0f, 1.6754f }, { 39.0f, 1.6856f },
        { 52.0f, 1.691f }, { 65.0f, 1.6965f }, { 79.0f, 1.7025f }, { 90.0f, 1.7055f }, { 96.0f, 1.708f }, { 98.0f, 1.72f },
        { 99.5f, 1.735f } };
    // A orelha (rosto.png de lado e de frente): da frente (o tragus) subindo
    // pela raiz da helice, o alto, as costas (o mais afastado da cabeca), o
    // lobulo e de volta a frente.
    recipe.earOutline = { { 0.0158f, -0.003f, 0.004f }, { 0.0135f, 0.0106f, 0.005f }, { 0.0102f, 0.0173f, 0.008f },
        { 0.0079f, 0.0366f, 0.012f }, { -0.0091f, 0.0385f, 0.016f }, { -0.0204f, 0.0322f, 0.022f },
        { -0.0249f, 0.0218f, 0.026f }, { -0.0231f, 0.0049f, 0.025f }, { -0.0147f, -0.0121f, 0.02f },
        { -0.0057f, -0.0223f, 0.0145f }, { 0.0045f, -0.0245f, 0.01f }, { 0.0158f, -0.0223f, 0.0055f },
        { 0.0169f, -0.0098f, 0.0035f } };
    return recipe;
}

namespace {

void validateRecipe(const HeadRecipe& r) {
    const auto check = [](bool condition, const char* message) {
        if (!condition) throw std::invalid_argument(message);
    };
    check(r.crownHeight > 1.6f && r.crownHeight < 2.3f, "crownHeight invalido");
    check(r.faceSpacing > 0.005f && r.faceSpacing < 0.05f && r.skullSpacing >= r.faceSpacing && r.skullSpacing < 0.08f
            && r.detailSpacing > 0.2f && r.detailSpacing <= 1.0f,
        "espacamentos invalidos");
    check(r.capHeight > 1.7f && r.capHeight < r.crownHeight - 0.008f && r.capColumns >= 4 && r.capColumns <= 40,
        "fileira do alto invalida");
    for (float h = 1.66f; h < r.crownHeight; h += 0.005f)
        check(r.front(h) > 0.003f && r.back(h) > 0.003f && r.side(h) > 0.003f && r.exponent(h) >= 1.5f, "cabeca sem largura");
    check(r.profile.points().size() >= 4 && r.profileWidth.points().size() >= 2, "perfil do meio curto demais");
    check(r.noseProfile.points().size() >= 3 && !r.noseWidth.empty() && !r.noseShape.empty(), "nariz sem perfil");
    for (const auto& [h, w] : r.noseWidth.points()) check(w > 0.001f && w < 0.03f, "noseWidth fora de (1 mm, 3 cm)");
    for (const auto& [h, k] : r.noseShape.points()) check(k >= 1.0f && k <= 6.0f, "noseShape fora de [1, 6]");
    check(r.alaRadius > 0.002f && r.alaHalfHeight > 0.002f && r.alaDepth >= 0.0f && r.alaDepth < 0.02f,
        "asas do nariz invalidas");
    check(r.nostrilRadius > 0.001f && r.nostrilHalfHeight > 0.0008f && r.nostrilDepth >= 0.0f && r.nostrilDepth < 0.006f,
        "narinas invalidas");
    check(r.mouthWidth > 0.015f && r.mouthWidth < 0.045f && r.mouthDepth >= 0.0f && r.mouthDepth < 0.01f
            && r.mouthCornerDepth >= 0.0f && r.mouthCornerDepth < 0.012f && r.upperLip > 0.002f && r.lipPeak >= r.upperLip
            && r.lipPeak < 0.02f && r.lowerLip > 0.002f && r.lowerLip < 0.02f,
        "boca invalida");
    check(r.mouthHeight + r.lipPeak < r.noseProfile.points().front().first - 0.004f, "a boca encosta no nariz");
    check(r.lipTaper >= 0.0f && r.lipTaper <= 1.2f, "lipTaper fora de [0, 1,2]");
    check(r.upperPadWidth > 0.005f && r.upperPadWidth < 0.05f && r.lowerPadWidth > 0.005f && r.lowerPadWidth < 0.05f
            && std::abs(r.upperPadDepth) < 0.008f && std::abs(r.lowerPadDepth) < 0.008f,
        "almofadas dos labios invalidas");
    check(r.eyeHalfWidth > 0.008f && r.eyeHalfWidth < 0.03f && r.eyeOpening > 0.003f && r.eyeOpening < 0.02f
            && r.eyeDepth >= 0.0f && r.eyeDepth < 0.02f && std::abs(r.eyeTilt) < 0.006f && r.eyeLateral > r.eyeHalfWidth,
        "olho invalido");
    check(r.lidThickness > 0.0003f && r.lidThickness < 0.005f && r.eyeGap > 0.0f && r.eyeGap < 0.004f && r.eyeBulge >= 0.0f
            && r.eyeBulge < 0.006f && r.irisRadius > 0.002f && r.irisRadius < r.eyeHalfWidth,
        "globo do olho invalido");
    check(r.browTop.points().size() >= 2 && r.browBottom.points().size() >= 2 && r.browThickness > 0.0005f
            && r.browThickness < 0.01f && r.browEdge > 0.0f && r.browEdge < r.browThickness,
        "sobrancelhas invalidas");
    check(r.browTipLateral > r.browTop.points().back().first && r.browTipLateral > r.browBottom.points().back().first
            && r.browBottom.points().front().first > 0.002f,
        "a ponta da sobrancelha deve ficar depois das bordas");
    for (float l = r.browBottom.points().front().first; l <= r.browTop.points().back().first; l += 0.002f)
        check(r.browTop(l) > r.browBottom(l) + 0.003f, "a sobrancelha tem que ter altura (browTop acima de browBottom)");
    check(r.browBottom(r.eyeLateral) > r.eyeHeight + 0.5f * r.eyeOpening + 0.002f, "a sobrancelha encosta no olho");
    check(r.cheekRadius > 0.005f && r.cheekRadius < 0.06f && std::abs(r.cheekDepth) < 0.015f, "macas invalidas");
    check(r.hollowRadius > 0.005f && r.hollowRadius < 0.06f && std::abs(r.hollowDepth) < 0.015f, "bochecha cavada invalida");
    check(r.ridgeHalfHeight > 0.002f && r.ridgeHalfHeight < 0.03f && r.ridgeReach > 0.01f && r.ridgeReach < 0.09f
            && std::abs(r.ridgeDepth) < 0.015f && r.socketWidth > 0.005f && r.socketHeight > 0.005f
            && std::abs(r.socketDepth) < 0.02f,
        "arco da testa ou orbita invalidos");
    check(r.jawLine.points().size() >= 3 && std::abs(r.jawRidge) < 0.008f && r.jawRidgeHeight > 0.002f
            && r.jawRidgeHeight < 0.04f && r.jawCurve >= 1.0f && r.jawCurve <= 5.0f,
        "mandibula invalida");
    check(r.earOutline.size() >= 6 && r.earOutline.size() <= 40, "earOutline: de 6 a 40 pontos");
    check(r.earConchaWidth > 0.003f && r.earConchaHeight > 0.003f && r.earRoot > 1.05f && r.earRoot < 2.0f && r.earRim > 0.0005f
            && r.earRim < 0.01f,
        "concha, raiz ou borda da orelha invalidas");
    {
        double area = 0.0;
        for (std::size_t k = 0; k < r.earOutline.size(); ++k) {
            const auto& a = r.earOutline[k];
            const auto& b = r.earOutline[(k + 1) % r.earOutline.size()];
            area += static_cast<double>(a[0]) * b[1] - static_cast<double>(b[0]) * a[1];
            check(a[2] > r.earRim + 0.0003f && a[2] < 0.05f, "earOutline: o afastamento passa da borda e fica abaixo de 5 cm");
            const float angle = std::atan2(a[1], a[0]);
            const float conchaU = r.earRoot * r.earConchaWidth * std::cos(angle), conchaV = r.earRoot * r.earConchaHeight * std::sin(angle);
            check(std::hypot(a[0], a[1]) > std::hypot(conchaU, conchaV) + 0.002f, "earOutline: o contorno tem que ficar fora da raiz");
        }
        check(area > 0.0, "earOutline: anti-horario visto de fora (para a frente, subindo, para tras, descendo)");
    }
    check(r.jitter >= 0.0f && r.jitter < 0.005f, "jitter invalido");
}

} // namespace

std::optional<HeadRecipe> parseHeadRecipe(std::string_view json, std::string* error) {
    try {
        const nlohmann::json document = nlohmann::json::parse(json);
        if (!document.is_object() || (document.contains("type") && document.at("type") != "head"))
            throw std::invalid_argument("esperado objeto do tipo head");
        HeadRecipe recipe = HeadRecipe::defaults();
        readCurve(document, "centerX", recipe.centerX, 2.5f);
        readCurve(document, "front", recipe.front, 2.5f);
        readCurve(document, "back", recipe.back, 2.5f);
        readCurve(document, "side", recipe.side, 2.5f);
        readCurve(document, "exponent", recipe.exponent, 2.5f);
        readCurve(document, "sideExponent", recipe.sideExponent, 2.5f);
        readCurve(document, "frontExponent", recipe.frontExponent, 2.5f);
        readCurve(document, "frontSideExponent", recipe.frontSideExponent, 2.5f);
        readValue(document, "crownHeight", recipe.crownHeight);
        readValue(document, "faceSpacing", recipe.faceSpacing);
        readValue(document, "detailSpacing", recipe.detailSpacing);
        readValue(document, "skullSpacing", recipe.skullSpacing);
        readValue(document, "capHeight", recipe.capHeight);
        readValue(document, "capColumns", recipe.capColumns);
        readCurve(document, "profile", recipe.profile, 2.5f);
        readCurve(document, "profileWidth", recipe.profileWidth, 2.5f);
        readCurve(document, "noseProfile", recipe.noseProfile, 2.5f);
        readCurve(document, "noseWidth", recipe.noseWidth, 2.5f);
        readCurve(document, "noseShape", recipe.noseShape, 2.5f);
        readValue(document, "alaLateral", recipe.alaLateral);
        readValue(document, "alaHeight", recipe.alaHeight);
        readValue(document, "alaRadius", recipe.alaRadius);
        readValue(document, "alaHalfHeight", recipe.alaHalfHeight);
        readValue(document, "alaDepth", recipe.alaDepth);
        readValue(document, "nostrilLateral", recipe.nostrilLateral);
        readValue(document, "nostrilHeight", recipe.nostrilHeight);
        readValue(document, "nostrilRadius", recipe.nostrilRadius);
        readValue(document, "nostrilHalfHeight", recipe.nostrilHalfHeight);
        readValue(document, "nostrilDepth", recipe.nostrilDepth);
        readValue(document, "mouthHeight", recipe.mouthHeight);
        readValue(document, "mouthWidth", recipe.mouthWidth);
        readValue(document, "mouthDepth", recipe.mouthDepth);
        readValue(document, "mouthCornerDepth", recipe.mouthCornerDepth);
        readValue(document, "upperLip", recipe.upperLip);
        readValue(document, "lipPeak", recipe.lipPeak);
        readValue(document, "lowerLip", recipe.lowerLip);
        readValue(document, "lipTaper", recipe.lipTaper);
        readValue(document, "upperPadWidth", recipe.upperPadWidth);
        readValue(document, "upperPadDepth", recipe.upperPadDepth);
        readValue(document, "lowerPadWidth", recipe.lowerPadWidth);
        readValue(document, "lowerPadDepth", recipe.lowerPadDepth);
        readValue(document, "eyeLateral", recipe.eyeLateral);
        readValue(document, "eyeHeight", recipe.eyeHeight);
        readValue(document, "eyeHalfWidth", recipe.eyeHalfWidth);
        readValue(document, "eyeOpening", recipe.eyeOpening);
        readValue(document, "eyeTilt", recipe.eyeTilt);
        readValue(document, "eyeDepth", recipe.eyeDepth);
        readValue(document, "lidThickness", recipe.lidThickness);
        readValue(document, "eyeGap", recipe.eyeGap);
        readValue(document, "eyeBulge", recipe.eyeBulge);
        readValue(document, "irisRadius", recipe.irisRadius);
        readCurve(document, "browTop", recipe.browTop, 0.2f);
        readCurve(document, "browBottom", recipe.browBottom, 0.2f);
        readValue(document, "browTipLateral", recipe.browTipLateral);
        readValue(document, "browTipHeight", recipe.browTipHeight);
        readValue(document, "browThickness", recipe.browThickness);
        readValue(document, "browEdge", recipe.browEdge);
        readValue(document, "ridgeHeight", recipe.ridgeHeight);
        readValue(document, "ridgeHalfHeight", recipe.ridgeHalfHeight);
        readValue(document, "ridgeReach", recipe.ridgeReach);
        readValue(document, "ridgeDepth", recipe.ridgeDepth);
        readValue(document, "socketWidth", recipe.socketWidth);
        readValue(document, "socketHeight", recipe.socketHeight);
        readValue(document, "socketDepth", recipe.socketDepth);
        readValue(document, "cheekLateral", recipe.cheekLateral);
        readValue(document, "cheekHeight", recipe.cheekHeight);
        readValue(document, "cheekRadius", recipe.cheekRadius);
        readValue(document, "cheekDepth", recipe.cheekDepth);
        readValue(document, "hollowLateral", recipe.hollowLateral);
        readValue(document, "hollowHeight", recipe.hollowHeight);
        readValue(document, "hollowRadius", recipe.hollowRadius);
        readValue(document, "hollowDepth", recipe.hollowDepth);
        readCurve(document, "jawLine", recipe.jawLine, 180.0f);
        readValue(document, "jawRidge", recipe.jawRidge);
        readValue(document, "jawRidgeHeight", recipe.jawRidgeHeight);
        readValue(document, "jawCurve", recipe.jawCurve);
        readValue(document, "earX", recipe.earX);
        readValue(document, "earHeight", recipe.earHeight);
        if (document.contains("earOutline")) recipe.earOutline = document.at("earOutline").get<std::vector<std::array<float, 3>>>();
        readValue(document, "earConchaWidth", recipe.earConchaWidth);
        readValue(document, "earConchaHeight", recipe.earConchaHeight);
        readValue(document, "earRoot", recipe.earRoot);
        readValue(document, "earRim", recipe.earRim);
        readValue(document, "jitter", recipe.jitter);
        readValue(document, "seed", recipe.seed);
        if (document.contains("colors")) {
            const nlohmann::json& colors = document.at("colors");
            readColor(colors, "skin", recipe.skinColor);
            readColor(colors, "sclera", recipe.scleraColor);
            readColor(colors, "iris", recipe.irisColor);
            readColor(colors, "lash", recipe.lashColor);
            readColor(colors, "brow", recipe.browColor);
            readColor(colors, "nostril", recipe.nostrilColor);
            readColor(colors, "canal", recipe.canalColor);
        }
        validateRecipe(recipe);
        if (error != nullptr) error->clear();
        return recipe;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("receita da cabeca invalida: ") + exception.what();
        return std::nullopt;
    }
}

} // namespace MatterEngine::Modeling
