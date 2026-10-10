// Contratos geometricos das pecas procedurais, sem engine, janela ou GPU.
#include "Engine/Modeling/Analysis/ConnectivityCheck.hpp"
#include "Engine/Modeling/Analysis/IntersectionCheck.hpp"
#include "Engine/Modeling/Analysis/TopologyCheck.hpp"
#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshRepair.hpp"
#include "Engine/Modeling/Parts/ArmGenerator.hpp"
#include "Engine/Modeling/Parts/BootGenerator.hpp"
#include "Engine/Modeling/Parts/HandGenerator.hpp"
#include "Engine/Modeling/Parts/HeadGenerator.hpp"
#include "Engine/Modeling/Parts/HeadRecipe.hpp"
#include "Engine/Modeling/Parts/LegGenerator.hpp"
#include "Engine/Modeling/Parts/NeckGenerator.hpp"
#include "Engine/Modeling/Parts/NumberGlyphs.hpp"
#include "Engine/Modeling/Parts/ShapeField.hpp"
#include "Engine/Modeling/Parts/ShirtGenerator.hpp"
#include "Engine/Modeling/Parts/ShortsGenerator.hpp"
#include "Engine/Modeling/Parts/SuperSection.hpp"
#include "Engine/Modeling/Parts/SurfaceDrape.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace MatterEngine;
using namespace MatterEngine::Modeling;

namespace {
int checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

void cleanMesh(const Mesh& mesh, bool allowSlivers = false, bool connected = false) {
    require(mesh.validateLayout().empty(), "layout consistente");
    Report report;
    checkTopology(mesh, report);
    if (connected) checkConnectivity(mesh, report);
    checkIntersections(mesh, mesh.positions, report);
    for (const auto& finding : report.findings())
        require(finding.severity == Severity::Info || (allowSlivers && finding.code == "triangulo_agulha"), report.text());
    require(report.count(Severity::Error) == 0, report.text());
}

void testProfile() {
    const ProfileCurve curve { { 0.0f, 0.03f }, { 0.25f, 0.07f }, { 0.55f, 0.04f }, { 1.0f, 0.06f } };
    require(std::abs(curve(-1) - 0.03f) < 1e-7f && std::abs(curve(2) - 0.06f) < 1e-7f, "curva limita as pontas");
    for (const auto& [t, value] : curve.points()) require(std::abs(curve(t) - value) < 1e-7f, "curva passa pelos controles");
    for (std::size_t i = 1; i < curve.points().size(); ++i) {
        const auto [t0, a] = curve.points()[i - 1];
        const auto [t1, b] = curve.points()[i];
        for (int step = 0; step <= 100; ++step) {
            const float v = curve(t0 + (t1 - t0) * static_cast<float>(step) / 100.0f);
            require(v >= std::min(a, b) - 1e-7f && v <= std::max(a, b) + 1e-7f, "perfil sem overshoot");
        }
    }
    require(ProfileCurve {}(0.5f) == 0.0f, "curva vazia e neutra");
}

void testRecipeAndBoot() {
    std::ifstream file(MATTERMODELING_RECIPE_PATH);
    std::stringstream text;
    text << file.rdbuf();
    std::string error;
    const auto recipe = parseBootRecipe(text.str(), &error);
    require(recipe.has_value(), "receita versionada: " + error);
    const auto partial = parseBootRecipe(R"({"length":0.30})", &error);
    require(partial && std::abs(partial->length - 0.30f) < 1e-7f, "receita parcial herda os padroes");
    for (const char* invalid : { "[", "[]", "null", R"({"type":"unknown"})", R"({"length":-1})",
             R"({"stations":[]})", R"({"stations":[0,0.5,0.5,1]})", R"({"stations":[0,1,0.5]})",
             R"({"topHeight":[]})", R"({"topHeight":[[0,0.1],[0,0.2]]})", R"({"medialHalfWidth":[[0,-0.1]]})",
             R"({"laceRows":0})", R"({"studSides":10000})", R"({"studs":[{"baseRadius":-1}]})",
             R"({"colors":{"upper":[256,0,0]}})", R"({"solePanelStations":[0.5]})", R"({"length":1e100})" }) {
        require(!parseBootRecipe(invalid, &error) && !error.empty(), std::string("receita invalida aceita: ") + invalid);
    }
    Mesh built, defaults;
    const BootSeams seams = buildBoot(built, *recipe, {});
    const BootSeams defaultSeams = buildBoot(defaults, BootRecipe::defaults(), {});
    require(seams.ok && defaultSeams.ok, "chuteira montada: " + seams.error + defaultSeams.error);
    MeshBuilder(built).compact();
    MeshBuilder(defaults).compact();
    // Uma superficie so: as pecas costuradas vertice com vertice, so a gola
    // aberta (declarada), nada se cruzando.
    cleanMesh(built, false, true);
    cleanMesh(defaults, false, true);
    require(built.parts.size() == 6 && built.findPart("chuteira_lingua_direita") >= 0
            && built.findPart("chuteira_ilhos_direita") >= 0, "seis partes: cabedal, lingua, sola, travas, ilhoses e cadarco");
    const auto declared = [&](const char* part, const char* other) {
        const int index = built.findPart(part), host = built.findPart(other);
        return index >= 0 && host >= 0
            && built.parts[static_cast<std::size_t>(index)].connectedWith(built.parts[static_cast<std::size_t>(host)]);
    };
    require(declared("chuteira_cabedal_direita", "chuteira_sola_direita") && declared("chuteira_travas_direita", "chuteira_sola_direita")
            && declared("chuteira_lingua_direita", "chuteira_cabedal_direita")
            && declared("chuteira_ilhos_direita", "chuteira_cabedal_direita")
            && declared("chuteira_cadarco_direita", "chuteira_ilhos_direita"),
        "costuras declaradas entre as pecas (a analise cobra as arestas)");
    require(seams.collar.size() >= 12, "a gola deixa a volta para a meia");
    for (const std::uint32_t v : seams.collar) require(v < built.positions.size(), "volta da gola na malha");
    require(built.positions.size() == defaults.positions.size() && built.triangles == defaults.triangles,
        "defaults e JSON com a mesma topologia");
    for (std::size_t v = 0; v < built.positions.size(); ++v) {
        require(distance(built.positions[v], defaults.positions[v]) < 1e-7f, "defaults e JSON com a mesma forma");
        require(built.positions[v].z >= -1e-7f, "nenhum vertice abaixo do chao");
    }
    require(built.materials.size() == defaults.materials.size() && built.triangleMaterial == defaults.triangleMaterial,
        "defaults e JSON com os mesmos materiais");
    for (std::size_t i = 0; i < built.materials.size(); ++i) {
        const Color a = built.materials[i].color, b = defaults.materials[i].color;
        require(std::abs(a.r - b.r) + std::abs(a.g - b.g) + std::abs(a.b - b.b) < 1e-7f, "cores sincronizadas");
    }
    const auto studVertices = built.partVertices(built.findPart("chuteira_travas_direita"));
    int grounded = 0;
    for (auto v : studVertices) grounded += std::abs(built.positions[v].z) < 1e-7f ? 1 : 0;
    require(grounded == static_cast<int>(recipe->studs.size()) * recipe->studSides, "todas as dez travas tocam o chao");

    // A orientacao de pecas pequenas nao depende da distancia da origem.
    for (Vec3& p : built.positions) p += Vec3 { 0.1f, -0.09f, -0.9776f };
    cleanMesh(built);
}

void clearance(const LocalMesh& strap, const DrapeField& field, float gap) {
    for (const Triangle& t : strap.triangles) {
        const std::array<Vec3, 3> face { strap.positions[t[0]], strap.positions[t[1]], strap.positions[t[2]] };
        if (cross(face[1] - face[0], face[2] - face[0]).z < -1e-10f)
            require(surfaceClearanceDeficit(face, field, gap) < 2e-6f, "folga exata, inclusive no interior dos triangulos");
    }
}

void testDrape() {
    Mesh mesh;
    MeshBuilder builder(mesh);
    builder.part("box");
    builder.box({}, { 0.05f, 0.05f, 0.015f });
    DrapeField field;
    field.addLayer(localMeshFrom(mesh, 0, {}));
    const StrapSpec spec { .width = 0.007f, .thickness = 0.0015f, .clearance = 0.0004f, .hostClearance = 0.0004f };
    const LocalMesh first = drapeStrap(field, sampleLine({ -0.035f, 0 }, { 0.035f, 0 }, 0.003f), spec);
    clearance(first, field, spec.clearance);
    builder.part("first");
    emitLocalMesh(builder, {}, first);
    field.addLayer(first);
    const LocalMesh second = drapeStrap(field, sampleLine({ 0, -0.035f }, { 0, 0.035f }, 0.003f), spec);
    clearance(second, field, spec.clearance);
    builder.part("second");
    emitLocalMesh(builder, {}, second);
    cleanMesh(mesh);

    // Sela: lados com inclinacoes contrarias. Diagonais diferentes nas faces
    // de cima e de baixo atravessavam a propria tira mesmo com folga externa.
    LocalMesh saddle;
    saddle.positions = { { -0.02f, -0.02f, 0.00f }, { 0.02f, -0.02f, 0.02f },
        { 0.02f, 0.02f, 0.00f }, { -0.02f, 0.02f, 0.02f } };
    saddle.triangles = { { 0, 1, 2 }, { 0, 2, 3 } };
    DrapeField uneven;
    uneven.addLayer(saddle);
    const std::array<Point2, 2> path { Point2 { -0.012f, 0.0f }, Point2 { 0.012f, 0.0f } };
    for (bool taut : { false, true }) {
        const StrapSpec thin { .width = 0.009f, .thickness = 0.0006f, .clearance = 0.00035f, .hostClearance = 0.00035f,
            .taut = taut };
        const auto ribbon = drapeStrap(uneven, path, thin);
        clearance(ribbon, uneven, thin.clearance);
        Mesh isolated;
        MeshBuilder b(isolated);
        b.part("saddle_ribbon");
        emitLocalMesh(b, {}, ribbon);
        cleanMesh(isolated, true);
    }
}
} // namespace

std::string readPart(const char* name) {
    std::ifstream file(std::string(MATTERMODELING_PARTS_DIR) + "/" + name);
    std::stringstream text;
    text << file.rdbuf();
    return text.str();
}

// As receitas de cima (opcionais): com a camisa, o calcao para na dobra da
// barra dela.
struct UpperBody {
    const ShirtRecipe* shirt = nullptr;
    const ArmRecipe* arm = nullptr;
    const NeckRecipe* neck = nullptr;
    const HandRecipe* hand = nullptr;
    const HeadRecipe* head = nullptr;
};

// A mao direita nas juntas do esqueleto do jogo (relativas ao pulso, a
// palma para baixo na pose T): o dorso dos dedos para cima, o do polegar
// como o do osso dele.
HandPlacement gameHand(Vec3 wrist) {
    HandPlacement placement;
    placement.wrist = wrist;
    const std::array<std::array<Vec3, 4>, 4> fingers { {
        { { { 0.026f, -0.117f, -0.007f }, { 0.025f, -0.15f, -0.009f }, { 0.024f, -0.179f, -0.014f }, { 0.023f, -0.208f, -0.019f } } },
        { { { 0.0f, -0.115f, 0.0f }, { -0.002f, -0.15f, -0.002f }, { -0.005f, -0.183f, -0.009f }, { -0.008f, -0.213f, -0.016f } } },
        { { { -0.022f, -0.116f, 0.001f }, { -0.027f, -0.144f, -0.003f }, { -0.031f, -0.17f, -0.009f }, { -0.035f, -0.193f, -0.014f } } },
        { { { -0.041f, -0.1f, 0.001f }, { -0.046f, -0.126f, -0.002f }, { -0.051f, -0.148f, -0.008f }, { -0.056f, -0.172f, -0.014f } } },
    } };
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t k = 0; k < 4; ++k) {
            placement.fingers[i].points[k] = wrist + fingers[i][k];
            if (k < 3) placement.fingers[i].dorsal[k] = { 0.0f, 0.0f, 1.0f };
        }
    const std::array<Vec3, 4> thumb { { { 0.025f, -0.039f, -0.016f }, { 0.045f, -0.071f, -0.029f }, { 0.057f, -0.1f, -0.045f },
        { 0.068f, -0.127f, -0.059f } } };
    for (std::size_t k = 0; k < 4; ++k) placement.thumb.points[k] = wrist + thumb[k];
    placement.thumb.dorsal = { Vec3 { 0.25f, -0.22f, 0.94f }, Vec3 { 0.27f, -0.37f, 0.89f }, Vec3 { 0.25f, -0.36f, 0.9f } };
    return placement;
}

// A metade direita da cintura para baixo num referencial simples (o chao em
// z = 0, o eixo da perna sobre o tornozelo da chuteira), espelhada; com as
// receitas de cima, a camisa, o braco e o pescoco por cima (o ombro e o
// cotovelo nas alturas do esqueleto do jogo, sem skin).
Mesh lowerBody(const BootRecipe& bootRecipe, const LegRecipe& legRecipe, const ShortsRecipe& shortsRecipe,
    const UpperBody& upper = {}) {
    Mesh mesh;
    BootPlacement boot;
    boot.frame = { { -bootRecipe.heelBehindAnkle, -0.09f, 0.0f }, {} };
    boot.collarOpening = false;
    const BootSeams bootSeams = buildBoot(mesh, bootRecipe, boot);
    require(bootSeams.ok, "chuteira: " + bootSeams.error);
    LegPlacement leg;
    leg.axis = { 0.0f, -0.09f };
    leg.ankleHeight = 0.136f;
    leg.kneeHeight = 0.5376f;
    leg.collar = bootSeams.collar;
    leg.bootPart = bootSeams.upperPart;
    leg.topOpening = false;
    const LegSeams legSeams = buildLeg(mesh, legRecipe, leg);
    require(legSeams.ok, "perna: " + legSeams.error);
    ShortsPlacement shorts;
    shorts.legAxis = leg.axis;
    shorts.legTop = legSeams.top;
    shorts.legPart = legSeams.skinPart;
    if (upper.shirt != nullptr) shorts.coverHeight = upper.shirt->tuckHeight;
    const ShortsSeams shortsSeams = buildShortsHalf(mesh, shortsRecipe, shorts);
    require(shortsSeams.ok, "calcao: " + shortsSeams.error);
    std::string root = shortsSeams.part;
    if (upper.shirt != nullptr) {
        ShirtPlacement shirt;
        shirt.shoulder = { -0.087f, -0.177f, 1.506f };
        shirt.elbow = { -0.087f, -0.48f, 1.506f };
        shirt.pelvisHeight = 0.983f;
        shirt.abdomenHeight = 1.086f;
        shirt.chestHeight = 1.278f;
        shirt.upperChestHeight = 1.411f;
        shirt.tuck = shortsSeams.top;
        shirt.tuckPart = shortsSeams.part;
        shirt.cuffOpening = upper.arm == nullptr;
        shirt.collarOpening = upper.neck == nullptr;
        const ShirtSeams shirtSeams = buildShirtHalf(mesh, *upper.shirt, shirt);
        require(shirtSeams.ok, "camisa: " + shirtSeams.error);
        root = shirtSeams.part;
        if (upper.arm != nullptr) {
            ArmPlacement arm;
            arm.shoulder = shirt.shoulder;
            arm.elbow = shirt.elbow;
            arm.cuff = shirtSeams.cuff;
            arm.sleevePart = shirtSeams.sleevePart;
            arm.wristOpening = upper.hand == nullptr;
            const ArmSeams armSeams = buildArm(mesh, *upper.arm, arm);
            require(armSeams.ok, "braco: " + armSeams.error);
            if (upper.hand != nullptr) {
                HandPlacement hand = gameHand({ -0.087f, -0.75f, 1.506f });
                hand.wristRing = armSeams.wrist;
                hand.forearmPart = armSeams.forearmPart;
                const HandSeams handSeams = buildHand(mesh, *upper.hand, hand);
                require(handSeams.ok, "mao: " + handSeams.error);
            }
        }
        if (upper.neck != nullptr) {
            NeckPlacement neck;
            neck.collar = shirtSeams.collar;
            neck.collarPart = shirtSeams.collarPart;
            neck.headOpening = upper.head == nullptr;
            const NeckSeams neckSeams = buildNeckHalf(mesh, *upper.neck, neck);
            require(neckSeams.ok, "pescoco: " + neckSeams.error);
            if (upper.head != nullptr) {
                HeadPlacement head;
                head.neckRing = neckSeams.top;
                head.neckPart = neckSeams.part;
                const HeadSeams headSeams = buildHeadHalf(mesh, *upper.head, head);
                require(headSeams.ok, "cabeca: " + headSeams.error);
            }
        }
    }
    mesh.parts[static_cast<std::size_t>(mesh.findPart(root))].root = true;
    std::vector<std::string> names;
    for (const MeshPart& part : mesh.parts) names.push_back(part.name);
    MeshBuilder builder(mesh);
    builder.mirrorParts(names);
    if (upper.shirt != nullptr) {
        std::string error;
        require(imprintShirtNumbers(mesh, *upper.shirt, 0.0f, "camisa", &error), "numeros da camisa: " + error);
    }
    builder.compact();
    flipFlatNeedles(mesh);
    return mesh;
}

void testLegAndShorts() {
    std::string error;
    const auto legRecipe = parseLegRecipe(readPart("perna.json"), &error);
    require(legRecipe.has_value(), "receita da perna versionada: " + error);
    const auto shortsRecipe = parseShortsRecipe(readPart("calcao.json"), &error);
    require(shortsRecipe.has_value(), "receita do calcao versionada: " + error);
    const auto bootRecipe = parseBootRecipe(readPart("chuteira.json"), &error);
    require(bootRecipe.has_value(), "receita da chuteira versionada: " + error);
    for (const char* invalid : { R"({"ringPoints":7})", R"({"rings":[0.3,0.2,0.4]})", R"({"sockTop":0.6,"bandTop":0.5})",
             R"({"front":[[0.2,-0.01]]})", R"({"patellaHalfHeight":0})" })
        require(!parseLegRecipe(invalid, &error) && !error.empty(), std::string("perna invalida aceita: ") + invalid);
    for (const char* invalid : { R"({"crotchHeight":0.6})", R"({"number":"1a"})", R"({"legRings":[0.9]})",
             R"({"hipSide":[[0.9,0.01]]})" })
        require(!parseShortsRecipe(invalid, &error) && !error.empty(), std::string("calcao invalido aceito: ") + invalid);

    // O JSON e o desenho padrao do codigo.
    const Mesh built = lowerBody(*bootRecipe, *legRecipe, *shortsRecipe);
    const Mesh defaults = lowerBody(BootRecipe::defaults(), LegRecipe::defaults(), ShortsRecipe::defaults());
    require(built.positions.size() == defaults.positions.size() && built.triangles == defaults.triangles,
        "perna e calcao: defaults e JSON com a mesma topologia");
    for (std::size_t v = 0; v < built.positions.size(); ++v)
        require(distance(built.positions[v], defaults.positions[v]) < 1e-7f, "perna e calcao: defaults e JSON com a mesma forma");

    // Uma superficie so, so o cos aberto, nada se cruzando.
    cleanMesh(built, false, true);
    Report report;
    checkConnectivity(built, report);
    std::size_t openings = 0;
    for (const Finding& finding : report.findings()) openings += finding.code == "abertura_declarada" ? 1 : 0;
    require(openings == 1, "so o cos aberto:\n" + report.text());
    for (const auto& [name, value] : report.statistics())
        if (name == "superficies (pedacos ligados)") require(value == "1", "uma superficie so");
    // O meio do calcao (a frente, as costas, a virilha) e o mesmo dos dois lados.
    std::size_t middle = 0;
    for (const Vec3& p : built.positions) middle += std::abs(p.y) < 1e-6f ? 1 : 0;
    require(middle >= static_cast<std::size_t>(shortsRecipe->crotchPoints) + 8, "a costura do meio soldada");
}

// A forma implicita (o campo da camisa): o valor, a projecao e a mistura.
void testShapeField() {
    ShapeField field;
    field.add(ConeChain { { { 0.0f, 0.0f, 0.0f }, { 0.0f, -0.3f, 0.0f } }, { 0.1f, 0.06f } });
    require(field.value({ 0.0f, 0.0f, 0.0f }) < -0.09f, "o centro fica dentro");
    require(std::abs(field.value({ 0.0f, -0.15f, 0.08f }) - 0.0f) < 0.005f, "o casco do cone entre as esferas");
    const Vec3 projected = field.project({ 0.2f, -0.1f, 0.1f });
    require(std::abs(field.value(projected)) < 1e-5f, "a projecao cai na superficie");
    const std::optional<Vec3> hit = field.cross({ 0.0f, -0.1f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 0.5f);
    require(hit.has_value() && std::abs(field.value(*hit)) < 1e-4f && hit->z > 0.05f, "o raio de dentro acha a saida");
    const RoundCone side { { 0.15f, 0.0f, 0.0f }, { 0.25f, 0.0f, 0.0f }, 0.06f, 0.06f };
    ShapeField blended = field, alone;
    blended.add(side, 0.04f);
    alone.add(side);
    const Vec3 between { 0.1f, 0.0f, 0.0f };
    require(blended.value(between) < std::min(field.value(between), alone.value(between)) - 1e-4f, "a mistura preenche a dobra");
    require(smoothMinimum(0.1f, 0.5f, 0.04f) == 0.1f && smoothMinimum(0.1f, 0.1f, 0.04f) < 0.1f, "minimo suave");
    // A secao de dois expoentes: os pontos ficam na superelipse e os eixos
    // nos limites.
    SuperSection section { 0.01f, 0.15f, 0.12f, 0.2f, 2.2f, 1.0f, 1.45f };
    require(std::abs(section.radius(0.0f) - 0.15f) < 1e-5f && std::abs(section.radius(Pi) - 0.12f) < 1e-5f
            && std::abs(section.radius(0.5f * Pi) - 0.2f) < 1e-5f,
        "secao: frente, tras e lado");
    for (int k = 0; k < 64; ++k) {
        const float angle = 2.0f * Pi * static_cast<float>(k) / 64.0f;
        const Vec3 p = section.point(angle);
        const float a = std::cos(angle) >= 0.0f ? section.front : section.back;
        const float sum = std::pow(std::abs(p.x - section.centerX) / a, 1.45f) + std::pow(std::abs(p.y) / section.side, 2.2f);
        require(std::abs(sum - 1.0f) < 1e-3f, "o ponto da secao fica na superelipse");
    }
}

// Os algarismos: contornos simples e anti-horarios, furos dentro.
void testNumberGlyphs() {
    const auto area = [](const std::vector<Point2>& loop) {
        float sum = 0.0f;
        for (std::size_t i = 0; i < loop.size(); ++i) {
            const Point2 a = loop[i], b = loop[(i + 1) % loop.size()];
            sum += a.x * b.y - b.x * a.y;
        }
        return 0.5f * sum;
    };
    const auto inside = [](const std::vector<Point2>& loop, Point2 p) {
        bool in = false;
        for (std::size_t i = 0, j = loop.size() - 1; i < loop.size(); j = i++)
            if ((loop[i].y > p.y) != (loop[j].y > p.y)
                && p.x < (loop[j].x - loop[i].x) * (p.y - loop[i].y) / (loop[j].y - loop[i].y) + loop[i].x)
                in = !in;
        return in;
    };
    for (char digit = '0'; digit <= '9'; ++digit) {
        const DigitGlyph glyph = digitGlyph(digit, 0.19f);
        require(!glyph.fills.empty() && glyph.width > 0.3f, std::string("algarismo com contorno: ") + digit);
        for (const auto& fill : glyph.fills) {
            require(area(fill) > 0.1f, std::string("contorno anti-horario e com area: ") + digit);
            for (const Point2& p : fill)
                require(p.x >= -1e-6f && p.x <= glyph.width + 1e-6f && p.y >= -1e-6f && p.y <= 1.0f + 1e-6f,
                    std::string("algarismo dentro da caixa: ") + digit);
        }
        for (const auto& hole : glyph.holes) {
            Point2 c {};
            for (const Point2& p : hole) c = { c.x + p.x / static_cast<float>(hole.size()), c.y + p.y / static_cast<float>(hole.size()) };
            require(std::abs(area(hole)) > 0.01f && inside(glyph.fills.front(), c), std::string("furo dentro do algarismo: ") + digit);
        }
    }
    NumberLayout layout;
    layout.height = 0.1f;
    const std::vector<ImprintLoop> loops = numberLoops("10", layout, 1, 0);
    require(loops.size() == 3 && loops[0].material == 1 && loops[2].material == 0, "o 10: o 1, o 0 e o furo do 0");
}

void testUpperBody() {
    std::string error;
    const auto shirtRecipe = parseShirtRecipe(readPart("camisa.json"), &error);
    require(shirtRecipe.has_value(), "receita da camisa versionada: " + error);
    const auto armRecipe = parseArmRecipe(readPart("braco.json"), &error);
    require(armRecipe.has_value(), "receita do braco versionada: " + error);
    const auto neckRecipe = parseNeckRecipe(readPart("pescoco.json"), &error);
    require(neckRecipe.has_value(), "receita do pescoco versionada: " + error);
    const auto handRecipe = parseHandRecipe(readPart("mao.json"), &error);
    require(handRecipe.has_value(), "receita da mao versionada: " + error);
    const auto headRecipe = parseHeadRecipe(readPart("cabeca.json"), &error);
    require(headRecipe.has_value(), "receita da cabeca versionada: " + error);
    for (const char* invalid : { R"({"armpitHeight":1.0})", R"({"rings":[1.3,1.2]})", R"({"backColumns":5})",
             R"({"armholeRows":3})", R"({"sleeveRings":[0.3]})", R"({"number":"x"})", R"({"cuffOpening":0.09})" })
        require(!parseShirtRecipe(invalid, &error) && !error.empty(), std::string("camisa invalida aceita: ") + invalid);
    for (const char* invalid : { R"({"rings":[]})", R"({"rings":[0.3,0.29]})", R"({"front":[[0.25,0.001]]})" })
        require(!parseArmRecipe(invalid, &error) && !error.empty(), std::string("braco invalido aceito: ") + invalid);
    require(!parseNeckRecipe(R"({"rings":-1})", &error) && !error.empty(), "pescoco invalido aceito");
    for (const char* invalid : { R"({"palmRings":[0.05,0.04,0.06]})", R"({"fingers":[]})", R"({"thumbFromRing":3})",
             R"({"webDepth":0.9})", R"({"thumb":{"width":[0.01,0.01,0.01,0.05]}})" })
        require(!parseHandRecipe(invalid, &error) && !error.empty(), std::string("mao invalida aceita: ") + invalid);
    for (const char* invalid : { R"({"eyeOpening":0.05})", R"({"earOutline":[[0.01,0.0,0.01]]})", R"({"noseShape":[[1.74,9.0]]})",
             R"({"browTop":[[0.01,1.79],[0.05,1.79]]})", R"({"jawCurve":0.5})", R"({"capColumns":2})",
             R"({"earOutline":[[0.016,-0.003,0.004],[0.017,-0.01,0.004],[0.016,-0.022,0.005],[0.005,-0.025,0.007],[-0.015,-0.012,0.016],[-0.023,0.005,0.02]]})" })
        require(!parseHeadRecipe(invalid, &error) && !error.empty(), std::string("cabeca invalida aceita: ") + invalid);

    const BootRecipe boot = BootRecipe::defaults();
    const LegRecipe leg = LegRecipe::defaults();
    const ShortsRecipe shorts = ShortsRecipe::defaults();
    // O JSON e o desenho padrao do codigo.
    const ShirtRecipe shirtDefaults = ShirtRecipe::defaults();
    const ArmRecipe armDefaults = ArmRecipe::defaults();
    const NeckRecipe neckDefaults = NeckRecipe::defaults();
    const HandRecipe handDefaults = HandRecipe::defaults();
    const HeadRecipe headDefaults = HeadRecipe::defaults();
    const Mesh built = lowerBody(boot, leg, shorts, { &*shirtRecipe, &*armRecipe, &*neckRecipe, &*handRecipe, &*headRecipe });
    const Mesh defaults = lowerBody(boot, leg, shorts, { &shirtDefaults, &armDefaults, &neckDefaults, &handDefaults, &headDefaults });
    require(built.positions.size() == defaults.positions.size() && built.triangles == defaults.triangles,
        "camisa, braco, mao, pescoco e cabeca: defaults e JSON com a mesma topologia");
    for (std::size_t v = 0; v < built.positions.size(); ++v)
        require(distance(built.positions[v], defaults.positions[v]) < 1e-7f,
            "camisa, braco, mao, pescoco e cabeca: defaults e JSON com a mesma forma");
    require(built.findPart("mao_direita") >= 0 && built.findPart("mao_esquerda") >= 0 && built.findMaterial("unha") >= 0,
        "as duas maos, com as unhas");
    for (const char* material : { "olho_iris", "olho_branco", "sobrancelha", "narina", "orelha_canal" })
        require(built.findMaterial(material) >= 0, std::string("a cabeca com o material ") + material);

    // Uma superficie so e fechada (a cabeca fecha em cima); nada se cruza.
    cleanMesh(built, false, true);
    Report report;
    checkConnectivity(built, report);
    std::size_t openings = 0;
    for (const Finding& finding : report.findings()) openings += finding.code == "abertura_declarada" ? 1 : 0;
    require(openings == 0, "nada aberto com a cabeca:\n" + report.text());
    // Sem a cabeca, a volta de cima do pescoco fica aberta (declarada).
    const Mesh headless = lowerBody(boot, leg, shorts, { &shirtDefaults, &armDefaults, &neckDefaults, &handDefaults });
    Report headlessReport;
    checkConnectivity(headless, headlessReport);
    std::size_t headlessOpenings = 0;
    for (const Finding& finding : headlessReport.findings()) headlessOpenings += finding.code == "abertura_declarada" ? 1 : 0;
    require(headlessOpenings == 1, "sem a cabeca, so o alto do pescoco aberto");
    // Sem a mao, o pulso fica aberto (declarado).
    const Mesh withoutHands = lowerBody(boot, leg, shorts, { &shirtDefaults, &armDefaults, &neckDefaults });
    Report handlessReport;
    checkConnectivity(withoutHands, handlessReport);
    std::size_t wrists = 0;
    for (const Finding& finding : handlessReport.findings())
        wrists += finding.code == "abertura_declarada" && finding.message.find("pulso") != std::string::npos ? 1 : 0;
    require(wrists == 2, "sem as maos, os dois pulsos abertos");
    for (const auto& [name, value] : report.statistics())
        if (name == "superficies (pedacos ligados)") require(value == "1", "uma superficie so");
    // A barra por fora do calcao (a folga da receita, medida no plano dela).
    const int shortsPart = built.findPart("calcao");
    float hemLowest = 10.0f;
    for (const std::uint32_t t : built.partTriangles(built.findPart("camisa")))
        for (const std::uint32_t v : built.triangles[t]) hemLowest = std::min(hemLowest, built.positions[v].z);
    require(std::abs(hemLowest - shirtRecipe->hemHeight) < 0.002f, "a barra na altura da receita");
    float shortsTop = 0.0f;
    for (const std::uint32_t t : built.partTriangles(shortsPart))
        for (const std::uint32_t v : built.triangles[t]) shortsTop = std::max(shortsTop, built.positions[v].z);
    require(std::abs(shortsTop - shirtRecipe->tuckHeight) < 1e-4f, "o calcao para na dobra da barra");
    // Sem a camisa o cos continua aberto (o calcao sozinho).
    const Mesh lower = lowerBody(boot, leg, shorts);
    Report lowerReport;
    checkConnectivity(lower, lowerReport);
    std::size_t lowerOpenings = 0;
    for (const Finding& finding : lowerReport.findings()) lowerOpenings += finding.code == "abertura_declarada" ? 1 : 0;
    require(lowerOpenings == 1, "sem camisa so o cos aberto");
}

int main() {
    try {
        testProfile();
        testRecipeAndBoot();
        testLegAndShorts();
        testShapeField();
        testNumberGlyphs();
        testUpperBody();
        testDrape();
        std::cout << "MatterModelingParts tests passed (" << checks << " checagens)\n";
    } catch (const std::exception& exception) {
        std::cerr << "MatterModelingParts failure: " << exception.what() << '\n';
        return 1;
    }
}
