#include "Engine/Modeling/Assembly/AssemblyRecipe.hpp"

#include "Engine/Modeling/Parts/RecipeJson.hpp"

namespace MatterEngine::Modeling {

AssemblyRecipe AssemblyRecipe::defaults() {
    return {};
}

namespace {

Vec3 readDirection(const nlohmann::json& document, const char* key, Vec3 fallback) {
    if (!document.contains(key)) return fallback;
    const std::vector<float> value = document.at(key).get<std::vector<float>>();
    if (value.size() != 3) throw std::invalid_argument(std::string(key) + ": esperado [x, y, z]");
    const Vec3 direction { value[0], value[1], value[2] };
    if (direction.length() < 1e-6f) throw std::invalid_argument(std::string(key) + ": direcao nula");
    return direction;
}

void validateRecipe(const AssemblyRecipe& r) {
    const auto check = [](bool condition, const char* message) {
        if (!condition) throw std::invalid_argument(message);
    };
    check(!r.bodyFile.empty() && !r.headFile.empty(), "faltam os arquivos do corpo e da cabeca");
    check(r.bodyHeight > 0.5f && r.bodyHeight < 2.5f, "bodyHeight fora de (0,5 m, 2,5 m)");
    check(r.headScale > 0.01f && r.headScale < 100.0f, "headScale invalido");
    check(r.headTop > r.neckHeight + 0.1f && r.headTop < 2.6f, "headTop tem que ficar acima do pescoco");
    check(std::abs(r.neckTilt) < 60.0f, "neckTilt fora de (-60, 60) graus");
    check(r.neckSnap >= 0.0f && r.neckSnap < 0.03f, "neckSnap fora de [0, 3 cm)");
    check(r.neckBlend > 0.005f && r.neckBlend < 0.2f, "neckBlend fora de (5 mm, 20 cm)");
    check(r.neckWeld >= 0.0f && r.neckWeld < 0.03f, "neckWeld fora de [0, 3 cm)");
    check(r.shirtHem > 0.5f && r.shirtHem < r.collarDepth - 0.2f && r.sleeveAt > 0.1f && r.sleeveAt < 0.95f,
        "camisa: barra, manga ou gola invalidas");
    check(r.collarSide > r.collarDepth && r.collarBack > r.collarDepth && r.collarSide < r.collarDepth + 0.2f
            && r.collarBack < r.collarDepth + 0.2f && r.collarOpening > 20.0f && r.collarOpening < 160.0f,
        "gola: o lado e as costas acima do fundo do V (ate 20 cm) e a abertura em (20, 160) graus");
    check(r.shortsTop > r.shirtHem + 0.01f && r.shortsHem > 0.3f && r.shortsHem < r.shirtHem - 0.1f, "calcao: alto ou barra invalidos");
    for (const float loose : { r.shirtLoose, r.shirtLooseHem, r.sleeveLoose, r.collarLoose, r.armpitLoose, r.shortsLoose,
             r.shortsLooseTop, r.shortsLooseHem })
        check(loose >= 0.0f && loose < 0.06f, "folga do uniforme fora de [0, 6 cm)");
    check(r.shortsSubdivision >= 0 && r.shortsSubdivision <= 2 && r.shortsCoarseCrotch >= 0.0f && r.shortsCoarseCrotch < 0.15f
            && r.shortsCoarseTop >= 0.0f && r.shortsCoarseTop < 0.2f,
        "shortsSubdivision de 0 a 2, shortsCoarseCrotch de 0 a 15 cm, shortsCoarseTop de 0 a 20 cm");
    check(r.hipCreaseOuter[1] <= 0.0f
            || (r.hipCreaseOuter[0] < r.hipCreaseInner[0] && r.hipCreaseInner[0] <= 0.0f && r.hipCreaseOuter[1] > 0.5f
                && r.hipCreaseOuter[1] < 1.3f && r.hipCreaseInner[1] > 0.5f && r.hipCreaseInner[1] < r.hipCreaseOuter[1]),
        "hipCrease: o ponto de fora mais para o lado e mais alto que o do gancho (y <= 0, alturas de 0,5 a 1,3 m)");
    for (const float width : { r.hipCreaseFront, r.hipCreaseSide, r.hipCreaseBack, r.hipCreaseInside })
        check(width > 0.005f && width < 0.4f, "hipCreaseFront, hipCreaseSide, hipCreaseBack e hipCreaseInside de 5 mm a 40 cm");
    check(std::abs(r.hipCreaseBackCenter) < 0.2f, "hipCreaseBackCenter ate 20 cm");
    check((r.hipLiftRise[0] == 0.0f && r.hipLiftRise[1] == 0.0f)
            || (r.hipLiftRise[0] >= 0.0f && r.hipLiftRise[1] > r.hipLiftRise[0] && r.hipLiftRise[1] < 0.4f),
        "hipLiftRise: de 0 a 40 cm acima da barra, o comeco antes do fim (ou [0, 0])");
    check(r.hipLiftFront[0] >= -1.0f && r.hipLiftFront[1] > r.hipLiftFront[0] && r.hipLiftFront[1] <= 1.0f,
        "hipLiftFront: cossenos de -1 a 1, o primeiro menor");
    check(r.hipLiftMidline[0] >= 0.0f && r.hipLiftMidline[1] > r.hipLiftMidline[0] && r.hipLiftMidline[1] < 0.2f,
        "hipLiftMidline: de 0 a 20 cm do meio, o primeiro menor");
    check(r.shortsLooseTop < r.shirtLooseHem, "o alto do calcao tem que ficar dentro da camisa (folga menor que a da barra dela)");
    check(r.recess > 0.003f && r.recess < 0.06f && r.clothRelax >= 0 && r.clothRelax < 40 && r.shoulderRelax >= 0.0f
            && r.shoulderRelax <= 1.0f && r.clothSteps >= 1 && r.clothSteps < 40
            && r.clothSmoothing >= 0 && r.clothSmoothing < 20 && r.clothHang >= 0.0f && r.clothHang <= 2.0f && r.shortsHang >= 0.0f
            && r.shortsHang <= 2.0f && r.armpitWeightSmoothing >= 0
            && r.armpitWeightSmoothing <= 200 && r.fingerWeightSmoothing >= 0 && r.fingerWeightSmoothing <= 200 && r.midlineBlend >= 0.0f && r.midlineBlend < 0.2f && r.crotchPinRadius >= 0.0f
            && r.crotchPinRadius < 0.3f && r.crotchPin >= 0.0f && r.crotchPin <= 1.0f && r.clothShortEdge >= 0.0f
            && r.clothShortEdge < 0.9f && r.crotchBand >= 0.0f && r.crotchBand < 0.06f,
        "recuo, passos, alisamento ou queda do pano invalidos");
    check(!r.bootFile.empty() && r.bootScale > 0.5f && r.bootScale < 1.5f && r.bootCollarGap > 0.002f && r.bootCollarGap < 0.06f,
        "chuteira: arquivo, escala (0,5 a 1,5) ou folga da gola invalidos");
    check(r.bootShade > 0.05f && r.bootShade <= 2.0f, "chuteira: o escurecido de 0,05 a 2");
    check(r.atlasSize >= 256 && r.atlasSize <= 8192 && r.faceDensity >= 0.5f && r.faceDensity <= 8.0f && r.laceDensity >= 0.5f
            && r.laceDensity <= 8.0f && r.garmentDensity >= 0.5f && r.garmentDensity <= 8.0f && r.paintEdge >= 0.0f && r.paintEdge <= 4.0f,
        "textura: atlas de 256 a 8192 texels, densidades do rosto, do cadarco e do uniforme de 0,5 a 8 e borda da pintura de 0 a 4 texels");
    check(r.neck.amount >= 0.0f && r.neck.amount < 0.05f && r.neck.from > 1.0f && r.neck.topFront[0] > r.neck.topBack[0]
            && r.neck.topFront[1] > r.neck.from && r.neck.topBack[1] > r.neck.from,
        "neck: encurtar de 0 a 5 cm, a base acima de 1 m e a linha de cima (da frente para tras) acima dela");
    if (!r.referenceFit.file.empty())
        check(r.referenceFit.jawLine.size() >= 2 && r.referenceFit.alignAbove > 1.0f && r.referenceFit.alignMaxSide > 0.01f
                && r.referenceFit.topBlend > 0.0f && r.referenceFit.topBlend < 0.1f && r.referenceFit.maxDistance > 0.0f
                && r.referenceFit.maxDistance < 0.2f && r.referenceFit.collarClearance >= 0.0f && r.referenceFit.collarClearance < 0.05f && r.referenceFit.neckRamp > 0.0f
                && r.referenceFit.neckRamp <= 1.0f,
            "referenceFit: a linha da mandibula com dois pontos ou mais, alinhar acima de 1 m, transicao ate 10 cm, distancia ate 20 cm");
    check(r.shirtBackThighWeight >= 0.0f && r.shirtBackThighWeight <= 1.0f && r.shirtNeckWeight >= 0.0f && r.shirtNeckWeight <= 1.0f
            && r.weightRelaxIterations >= 0 && r.weightRelaxIterations <= 20,
        "shirtBackThighWeight e shirtNeckWeight de 0 a 1, weightRelax ate 20 voltas");
    for (const WeightRelaxPoint& point : r.weightRelax)
        check(point.at.y <= 1e-5f && point.radius > 0.002f && point.radius < 0.1f, "weightRelax: na metade direita (y <= 0), raio de 2 mm a 10 cm");
    for (const VertexMove& move : r.faceShape)
        check(move.at.y <= 1e-5f && move.to.y <= 1e-5f && (move.to - move.at).length() < 0.03f,
            "faceShape: na metade direita (y <= 0), cada vertice anda ate 3 cm");
    for (const SculptPull& pull : r.headSculpt)
        check(pull.radius > 0.001f && pull.radius < 0.1f && pull.move.length() < 0.03f, "headSculpt: raio de 1 mm a 10 cm, puxao ate 3 cm");
    check(r.headPinAbove >= 0.0f && r.headPinAbove < 0.05f && r.collarPinDepth >= 0.0f && r.collarPinDepth < 0.1f
            && r.collarPinWidth >= 0.0f && r.collarPinWidth < 0.2f,
        "pesos da cabeca e da gola invalidos");
    check(r.neckFrontBase > r.neck.from - 0.2f && r.neckFrontBase < r.neck.topFront[1], "neckFrontBase: abaixo da linha de cima do pescoco");
    const FacePaintShape& f = r.face;
    check(f.eyeY < -0.01f && f.eyeY > -0.08f && f.eyeHalfWidth > 0.005f && f.eyeHalfWidth < 0.03f && f.eyeUpper > 0.001f
            && f.eyeLower > 0.001f && f.irisRadius > f.pupilRadius && f.irisRadius < 0.012f && f.browInner > 0.0f && f.browOuter > 0.0f,
        "face: olho ou sobrancelha invalidos");
    // As linhas da boca: do meio ao canto (|y| crescendo), as duas no mesmo canto.
    const auto ordered = [](const std::vector<std::array<float, 2>>& points) {
        if (points.size() < 2 || points.front()[0] < 0.0f) return false;
        for (std::size_t i = 1; i < points.size(); ++i)
            if (points[i][0] <= points[i - 1][0]) return false;
        return true;
    };
    check(ordered(f.mouthSlit) && ordered(f.upperLipTop) && std::abs(f.upperLipTop.back()[0] - f.mouthSlit.back()[0]) < 1e-4f,
        "face: as linhas da boca vao do meio ao canto (|y| crescendo) e acabam no mesmo canto");
    const EyeShape& e = r.eye;
    check(e.centerY < -0.005f && e.centerY > -0.1f, "eye.centerY: o olho direito fica em y < 0");
    check(e.width > 0.01f && e.width < 0.06f && e.height > 0.003f && e.height < e.width, "eye: largura e altura invalidas");
    check(std::abs(e.yaw) < 60.0f && std::abs(e.pitch) < 45.0f && std::abs(e.tilt) < 0.5f, "eye: yaw, pitch ou tilt invalidos");
    check(e.marginUpper > e.lidBandUpper && e.marginLower > e.lidBandLower && e.lidBandUpper > 0.0002f
            && e.lidBandLower > 0.0002f && e.cornerMargin >= 0.0f
            && e.cornerMargin < 0.01f,
        "eye: a placa tem que passar da faixa da palpebra");
    check(e.lidUpper >= 0.0f && e.lidUpper < 0.01f && e.lidLower >= 0.0f && e.lidLower < 0.01f, "eye: palpebras invalidas");
    check(e.recess > 0.0f && e.recess < 0.015f && e.bulge >= 0.0f && e.bulge < 0.015f, "eye: globo invalido");
}

} // namespace

std::optional<AssemblyRecipe> parseAssemblyRecipe(std::string_view json, std::string* error) {
    try {
        const nlohmann::json document = nlohmann::json::parse(json);
        if (!document.is_object() || (document.contains("type") && document.at("type") != "assembly"))
            throw std::invalid_argument("esperado objeto do tipo assembly");
        AssemblyRecipe recipe = AssemblyRecipe::defaults();
        readValue(document, "bodyFile", recipe.bodyFile);
        recipe.bodyForward = readDirection(document, "bodyForward", recipe.bodyForward);
        readValue(document, "bodyHeight", recipe.bodyHeight);
        readValue(document, "neckHeight", recipe.neckHeight);
        readValue(document, "neckTilt", recipe.neckTilt);
        readValue(document, "neckSnap", recipe.neckSnap);
        readValue(document, "neckBlend", recipe.neckBlend);
        readValue(document, "neckWeld", recipe.neckWeld);
        readValue(document, "headFile", recipe.headFile);
        recipe.headForward = readDirection(document, "headForward", recipe.headForward);
        readValue(document, "headScale", recipe.headScale);
        readValue(document, "headTop", recipe.headTop);
        readValue(document, "headShift", recipe.headShift);
        readValue(document, "bootFile", recipe.bootFile);
        readValue(document, "bootScale", recipe.bootScale);
        readValue(document, "bootCollarGap", recipe.bootCollarGap);
        readValue(document, "uniform", recipe.uniform);
        readValue(document, "shirtHem", recipe.shirtHem);
        readValue(document, "sleeveAt", recipe.sleeveAt);
        readValue(document, "collarDepth", recipe.collarDepth);
        readValue(document, "collarSide", recipe.collarSide);
        readValue(document, "collarBack", recipe.collarBack);
        readValue(document, "collarOpening", recipe.collarOpening);
        readValue(document, "shirtLoose", recipe.shirtLoose);
        readValue(document, "shirtLooseHem", recipe.shirtLooseHem);
        readValue(document, "sleeveLoose", recipe.sleeveLoose);
        readValue(document, "collarLoose", recipe.collarLoose);
        readValue(document, "armpitLoose", recipe.armpitLoose);
        readValue(document, "shortsTop", recipe.shortsTop);
        readValue(document, "shortsHem", recipe.shortsHem);
        readValue(document, "shortsLoose", recipe.shortsLoose);
        readValue(document, "shortsLooseTop", recipe.shortsLooseTop);
        readValue(document, "shortsLooseHem", recipe.shortsLooseHem);
        readValue(document, "recess", recipe.recess);
        readValue(document, "clothRelax", recipe.clothRelax);
        readValue(document, "shoulderRelax", recipe.shoulderRelax);
        readValue(document, "clothSteps", recipe.clothSteps);
        readValue(document, "clothSmoothing", recipe.clothSmoothing);
        readValue(document, "clothHang", recipe.clothHang);
        // Sem a do calcao, a mesma da camisa.
        recipe.shortsHang = recipe.clothHang;
        readValue(document, "shortsHang", recipe.shortsHang);
        readValue(document, "armpitWeightSmoothing", recipe.armpitWeightSmoothing);
        readValue(document, "fingerWeightSmoothing", recipe.fingerWeightSmoothing);
        readValue(document, "shirtBackThighWeight", recipe.shirtBackThighWeight);
        readValue(document, "shirtNeckWeight", recipe.shirtNeckWeight);
        if (document.contains("hipCreaseOuter")) recipe.hipCreaseOuter = document.at("hipCreaseOuter").get<std::array<float, 2>>();
        if (document.contains("hipCreaseInner")) recipe.hipCreaseInner = document.at("hipCreaseInner").get<std::array<float, 2>>();
        readValue(document, "hipCreaseFront", recipe.hipCreaseFront);
        readValue(document, "hipCreaseSide", recipe.hipCreaseSide);
        readValue(document, "hipCreaseBack", recipe.hipCreaseBack);
        readValue(document, "hipCreaseBackCenter", recipe.hipCreaseBackCenter);
        readValue(document, "hipCreaseInside", recipe.hipCreaseInside);
        if (document.contains("hipLiftRise")) recipe.hipLiftRise = document.at("hipLiftRise").get<std::array<float, 2>>();
        if (document.contains("hipLiftFront")) recipe.hipLiftFront = document.at("hipLiftFront").get<std::array<float, 2>>();
        if (document.contains("hipLiftMidline")) recipe.hipLiftMidline = document.at("hipLiftMidline").get<std::array<float, 2>>();
        readValue(document, "shortsSubdivision", recipe.shortsSubdivision);
        readValue(document, "shortsCoarseCrotch", recipe.shortsCoarseCrotch);
        readValue(document, "shortsCoarseTop", recipe.shortsCoarseTop);
        if (document.contains("weightRelax")) {
            const nlohmann::json& relax = document.at("weightRelax");
            readValue(relax, "iterations", recipe.weightRelaxIterations);
            if (relax.contains("points"))
                for (const nlohmann::json& point : relax.at("points")) {
                    const auto at = point.at("at").get<std::array<float, 3>>();
                    recipe.weightRelax.push_back({ { at[0], at[1], at[2] }, point.value("radius", 0.02f), point.value("part", std::string {}) });
                }
        }
        readValue(document, "midlineBlend", recipe.midlineBlend);
        readValue(document, "crotchPinRadius", recipe.crotchPinRadius);
        readValue(document, "crotchPin", recipe.crotchPin);
        readValue(document, "clothShortEdge", recipe.clothShortEdge);
        readValue(document, "crotchBand", recipe.crotchBand);
        if (document.contains("colors")) {
            readColor(document.at("colors"), "shirt", recipe.shirtColor);
            readColor(document.at("colors"), "shorts", recipe.shortsColor);
        }
        readValue(document, "paintFile", recipe.paintFile);
        readValue(document, "bootShade", recipe.bootShade);
        readValue(document, "textured", recipe.textured);
        readValue(document, "garmentDensity", recipe.garmentDensity);
        readValue(document, "paintEdge", recipe.paintEdge);
        readValue(document, "atlasSize", recipe.atlasSize);
        readValue(document, "faceDensity", recipe.faceDensity);
        readValue(document, "laceDensity", recipe.laceDensity);
        readValue(document, "modeledEyes", recipe.modeledEyes);
        if (document.contains("face")) {
            const nlohmann::json& face = document.at("face");
            FacePaintShape& f = recipe.face;
            const std::pair<const char*, float*> fields[] { { "eyeY", &f.eyeY }, { "eyeHeight", &f.eyeHeight }, { "eyeYaw", &f.eyeYaw },
                { "eyeHalfWidth", &f.eyeHalfWidth }, { "eyeUpper", &f.eyeUpper }, { "eyeLower", &f.eyeLower }, { "eyeTilt", &f.eyeTilt },
                { "irisRadius", &f.irisRadius }, { "pupilRadius", &f.pupilRadius }, { "lashWidth", &f.lashWidth },
                { "creaseWidth", &f.creaseWidth }, { "creaseShade", &f.creaseShade }, { "upperLidCurve", &f.upperLidCurve },
                { "earOutside", &f.earOutside },
                { "browInner", &f.browInner }, { "browOuter", &f.browOuter },
                { "browLowInner", &f.browLowInner }, { "browLowOuter", &f.browLowOuter }, { "browThickInner", &f.browThickInner },
                { "browThickOuter", &f.browThickOuter } };
            for (const auto& [name, value] : fields) readValue(face, name, *value);
            // As linhas da boca: [[|y|, altura], ...] do meio ao canto.
            const std::pair<const char*, std::vector<std::array<float, 2>>*> lines[] { { "mouthSlit", &f.mouthSlit },
                { "upperLipTop", &f.upperLipTop } };
            for (const auto& [name, points] : lines)
                if (face.contains(name)) *points = face.at(name).get<std::vector<std::array<float, 2>>>();
            readColor(face, "sclera", f.sclera);
            readColor(face, "iris", f.iris);
            readColor(face, "pupil", f.pupil);
            readColor(face, "lash", f.lash);
            readColor(face, "brow", f.brow);
            readColor(face, "mouthLine", f.mouthLine);
        }
        if (document.contains("referenceFit")) {
            const nlohmann::json& fit = document.at("referenceFit");
            ReferenceFitShape& f = recipe.referenceFit;
            readValue(fit, "file", f.file);
            readValue(fit, "alignAbove", f.alignAbove);
            readValue(fit, "alignMaxSide", f.alignMaxSide);
            readValue(fit, "topBlend", f.topBlend);
            readValue(fit, "maxDistance", f.maxDistance);
            readValue(fit, "collarClearance", f.collarClearance);
            readValue(fit, "neckRamp", f.neckRamp);
            if (fit.contains("jawLine")) f.jawLine = fit.at("jawLine").get<std::vector<std::array<float, 2>>>();
        }
        if (document.contains("faceShape"))
            for (const nlohmann::json& move : document.at("faceShape")) {
                const auto at = move.at("at").get<std::array<float, 3>>();
                const auto to = move.at("to").get<std::array<float, 3>>();
                recipe.faceShape.push_back({ { at[0], at[1], at[2] }, { to[0], to[1], to[2] } });
            }
        if (document.contains("headSculpt"))
            for (const nlohmann::json& pull : document.at("headSculpt")) {
                SculptPull entry;
                const auto at = pull.at("at").get<std::array<float, 3>>();
                const auto move = pull.at("move").get<std::array<float, 3>>();
                entry.at = { at[0], at[1], at[2] };
                entry.move = { move[0], move[1], move[2] };
                entry.radius = pull.value("radius", 0.01f);
                recipe.headSculpt.push_back(entry);
            }
        if (document.contains("neck")) {
            const nlohmann::json& neck = document.at("neck");
            readValue(neck, "shorten", recipe.neck.amount);
            readValue(neck, "from", recipe.neck.from);
            if (neck.contains("topFront")) recipe.neck.topFront = neck.at("topFront").get<std::array<float, 2>>();
            if (neck.contains("topBack")) recipe.neck.topBack = neck.at("topBack").get<std::array<float, 2>>();
        }
        readValue(document, "headPinAbove", recipe.headPinAbove);
        readValue(document, "collarPinDepth", recipe.collarPinDepth);
        readValue(document, "collarPinWidth", recipe.collarPinWidth);
        readValue(document, "neckGradient", recipe.neckGradient);
        readValue(document, "neckFrontBase", recipe.neckFrontBase);
        if (document.contains("eye")) {
            const nlohmann::json& eye = document.at("eye");
            EyeShape& e = recipe.eye;
            readValue(eye, "centerY", e.centerY);
            readValue(eye, "centerZ", e.centerZ);
            readValue(eye, "yaw", e.yaw);
            readValue(eye, "pitch", e.pitch);
            readValue(eye, "width", e.width);
            readValue(eye, "height", e.height);
            readValue(eye, "tilt", e.tilt);
            readValue(eye, "marginUpper", e.marginUpper);
            readValue(eye, "marginLower", e.marginLower);
            readValue(eye, "cornerMargin", e.cornerMargin);
            readValue(eye, "lidUpper", e.lidUpper);
            readValue(eye, "lidLower", e.lidLower);
            readValue(eye, "lidBandUpper", e.lidBandUpper);
            readValue(eye, "lidBandLower", e.lidBandLower);
            readValue(eye, "recess", e.recess);
            readValue(eye, "bulge", e.bulge);
        }
        validateRecipe(recipe);
        if (error != nullptr) error->clear();
        return recipe;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("receita da montagem invalida: ") + exception.what();
        return std::nullopt;
    }
}

} // namespace MatterEngine::Modeling
