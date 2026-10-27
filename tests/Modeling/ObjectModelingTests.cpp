// Os objetos procedurais (o nucleo, Modeling/Objects): a receita (ler,
// gravar, padroes, erros), a caixa gerada (o bloco fechado e orientado para
// fora, o mapa de textura por face, a densidade de pixels igual nas faces, a
// pixel art deterministica pela semente) e a descricao fisica. Linkado so
// contra MatterModeling.

#include "Engine/Modeling/Analysis/TopologyCheck.hpp"
#include "Engine/Modeling/Objects/ObjectRecipe.hpp"
#include "Engine/Modeling/Objects/PixelCanvas.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace MatterEngine;
using namespace MatterEngine::Modeling;

namespace {

int checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

bool near(float a, float b, float tolerance) { return std::abs(a - b) <= tolerance; }

const std::filesystem::path Objects = std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "objects";

bool sameImage(const Image& a, const Image& b) {
    if (a.width() != b.width() || a.height() != b.height()) return false;
    for (int y = 0; y < a.height(); ++y)
        for (int x = 0; x < a.width(); ++x) {
            const Color p = a.pixel(x, y), q = b.pixel(x, y);
            if (p.r != q.r || p.g != q.g || p.b != q.b) return false;
        }
    return true;
}

ObjectRecipe crateRecipe() {
    std::string error;
    const std::optional<ObjectRecipe> recipe = readObjectRecipe(Objects / "wood_crate.json", &error);
    require(recipe.has_value(), "a receita da caixa: " + error);
    return *recipe;
}

void testRecipes() {
    const std::vector<std::filesystem::path> files = objectRecipeFiles(Objects);
    require(!files.empty(), "as receitas em assets/objects");
    for (const std::filesystem::path& file : files) {
        std::string error;
        const std::optional<ObjectRecipe> recipe = readObjectRecipe(file, &error);
        require(recipe.has_value(), "receita valida: " + error);
        require(findObjectType(recipe->type) != nullptr, "o tipo existe: " + recipe->type);
        require(generateObject(*recipe, &error).has_value(), "a receita gera: " + error);
    }
    const ObjectRecipe recipe = crateRecipe();
    require(recipe.id == "wood_crate" && recipe.type == "caixa" && recipe.material == "wood", "a caixa: id, tipo e material");
    // Ida e volta: o texto gravado le igual.
    const std::string text = objectRecipeJson(recipe);
    std::string error;
    const std::optional<ObjectRecipe> again = parseObjectRecipe(text, &error);
    require(again.has_value() && objectRecipeJson(*again) == text, "gravar e ler de novo da a mesma receita: " + error);
    // Erros: schema, tipo, cor.
    require(!parseObjectRecipe("{}", &error), "sem schema e erro");
    require(!parseObjectRecipe(R"({"schema":"matter.object.v1","id":"x","type":"caixa","parameters":{"cor_madeira":"azul"}})", &error),
        "cor fora de #rrggbb e erro");
    const std::optional<ObjectRecipe> unknown = parseObjectRecipe(R"({"schema":"matter.object.v1","id":"x","type":"nave"})", &error);
    require(unknown.has_value() && !generateObject(*unknown, &error) && error.find("nave") != std::string::npos,
        "tipo desconhecido: nao gera, com o nome no erro");
    require(parseHexColor("#d98e3a").has_value() && hexColor(*parseHexColor("#d98e3a")) == "#d98e3a", "a cor em hex vai e volta");
    // Fora do intervalo: preso nele.
    const ObjectType& type = *findObjectType("caixa");
    ObjectRecipe wild = recipe;
    wild.numbers["largura"] = 50.0f;
    require(near(objectNumber(wild, type, "largura"), type.find("largura")->maximum, 1e-6f), "o valor fica no intervalo do parametro");
    // Os padroes do tipo geram.
    const ObjectRecipe defaults = defaultObjectRecipe(type, "nova", "Nova caixa", "wood");
    require(generateObject(defaults, &error).has_value(), "os padroes do tipo geram: " + error);
}

void testCrate() {
    const ObjectRecipe recipe = crateRecipe();
    std::string error;
    const std::optional<ProceduralObject> crate = generateObject(recipe, &error);
    require(crate.has_value(), "a caixa gera: " + error);
    const Mesh& mesh = crate->mesh;
    require(mesh.validateLayout().empty(), "a malha consistente");
    require(mesh.positions.size() == 8 && mesh.triangles.size() == 12, "um bloco: 8 cantos e 12 triangulos");
    // Fechado e para fora: cada aresta em dois triangulos, volume positivo
    // igual ao do bloco, sem achados na topologia.
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
    for (const Triangle& t : mesh.triangles)
        for (int k = 0; k < 3; ++k) ++edges[{ std::min(t[k], t[(k + 1) % 3]), std::max(t[k], t[(k + 1) % 3]) }];
    for (const auto& [edge, uses] : edges) require(uses == 2, "cada aresta em dois triangulos (fechado)");
    require(near(mesh.signedVolume(), 0.6f * 0.6f * 0.6f, 1e-4f), "o volume do bloco, positivo (para fora)");
    Report report;
    checkTopology(mesh, report);
    require(report.count(Severity::Error) == 0 && report.count(Severity::Warning) == 0, "a topologia sem achados:\n" + report.text());
    const Box3 box = mesh.bounds();
    require(near(box.minimum.z, -0.3f, 1e-5f) && near(box.maximum.x, 0.3f, 1e-5f), "centrado na origem");
    // O mapa: cada canto dentro do atlas, todos os triangulos na textura.
    require(mesh.hasUv() && mesh.materials.size() == 1 && mesh.materials.front().texture == 0 && mesh.textures.size() == 1,
        "um material com a textura e o mapa");
    for (const auto& corners : mesh.triangleUv)
        for (const Uv& uv : corners) require(uv.u > 0.0f && uv.u < 1.0f && uv.v > 0.0f && uv.v <= 1.0f, "o mapa dentro do atlas");
    // A pixel art: 32 no lado maior; o atlas tem os 3 ladrilhos (lado X,
    // lado Y e a tampa) e a textura e ela ampliada sem filtro.
    const Image& art = crate->pixelArt;
    require(art.width() == 96 && art.height() == 32, "o atlas: 3 ladrilhos de 32 px");
    const Image& texture = mesh.textures.front().image;
    require(texture.width() % art.width() == 0 && texture.width() / art.width() == texture.height() / art.height()
            && texture.width() > art.width(),
        "a textura e a pixel art ampliada");
    const int scale = texture.width() / art.width();
    for (int y = 0; y < art.height(); y += 5)
        for (int x = 0; x < art.width(); x += 7) {
            const Color a = art.pixel(x, y), b = texture.pixel(x * scale + scale / 2, y * scale + scale / 2);
            require(near(a.r, b.r, 1e-3f) && near(a.g, b.g, 1e-3f), "cada texel vira um bloco da mesma cor");
        }
    // Determinismo: a mesma receita da a mesma imagem; outra semente, outra.
    require(sameImage(generateObject(recipe)->pixelArt, art), "a mesma receita, a mesma pixel art");
    ObjectRecipe other = recipe;
    other.numbers["semente"] = 8.0f;
    require(!sameImage(generateObject(other)->pixelArt, art), "outra semente, outra pixel art");
    // A madeira domina (a cor media perto da da receita) e ha contraste
    // (frestas escuras e realces).
    float red = 0.0f, darkest = 1.0f, brightest = 0.0f;
    for (int y = 0; y < art.height(); ++y)
        for (int x = 0; x < art.width(); ++x) {
            const Color c = art.pixel(x, y);
            red += c.r;
            const float luma = 0.3f * c.r + 0.59f * c.g + 0.11f * c.b;
            darkest = std::min(darkest, luma);
            brightest = std::max(brightest, luma);
        }
    red /= static_cast<float>(art.width() * art.height());
    require(red > 0.6f && red < 0.95f, "a caixa e de madeira (o vermelho medio " + std::to_string(red) + ")");
    require(brightest - darkest > 0.4f, "a pixel art tem frestas e realces");
    // A fisica: uma caixa do tamanho dela, a massa e o material da receita.
    const ObjectPhysics& physics = crate->physics;
    require(physics.material == "wood" && near(physics.massKg, 14.0f, 1e-4f) && !physics.fixed, "o material e a massa");
    require(physics.colliders.size() == 1 && physics.colliders.front().shape == ObjectColliderShape::Box
            && near(physics.colliders.front().halfExtents.x, 0.3f, 1e-6f) && near(physics.colliders.front().halfExtents.z, 0.3f, 1e-6f),
        "a colisao: a caixa do tamanho dela");
    require(near(crate->size.y, 0.6f, 1e-6f), "o tamanho");
}

void testLongCrate() {
    // Comprida no x: a densidade de pixels igual - o lado Y (x por z) tem o
    // dobro da largura do lado X (y por z).
    ObjectRecipe recipe = crateRecipe();
    recipe.numbers["largura"] = 1.2f;
    std::string error;
    const std::optional<ProceduralObject> crate = generateObject(recipe, &error);
    require(crate.has_value(), "a caixa comprida gera: " + error);
    require(crate->pixelArt.width() == 16 + 32 + 32 && crate->pixelArt.height() == 16, "os ladrilhos pela medida de cada lado");
    require(near(crate->mesh.signedVolume(), 1.2f * 0.6f * 0.6f, 1e-4f), "o volume da comprida");
    require(near(crate->physics.colliders.front().halfExtents.x, 0.6f, 1e-6f), "a colisao da comprida");
}

void testPixelCanvas() {
    PixelCanvas canvas(16, 8, rgb(0, 0, 0));
    canvas.fillRect(2, 2, 5, 4, rgb(255, 0, 0));
    require(canvas.at(2, 2).r > 0.99f && canvas.at(5, 4).r > 0.99f && canvas.at(6, 4).r < 0.01f, "o retangulo inclusivo");
    canvas.line(0.0f, 0.0f, 16.0f, 8.0f, 1.0f, rgb(0, 255, 0));
    require(canvas.at(0, 0).g > 0.99f && canvas.at(15, 7).g > 0.99f, "a linha de ponta a ponta");
    const Image big = canvas.upscaled(4);
    require(big.width() == 64 && big.height() == 32 && big.pixel(9, 9).r > 0.99f, "ampliada por blocos");
    require(pixelNoise(3, 4, 7) == pixelNoise(3, 4, 7) && pixelNoise(3, 4, 7) != pixelNoise(3, 4, 8), "o ruido pela semente");
}

} // namespace

// A trave com quadro de fundo (assets/objects/trave.json): o vao da regra
// medido nos cilindros dos postes e do travessao, os pes no chao, cada tubo
// fechado e para fora, e a rede (presa nos tubos, o cabo do alto, o peso, a
// folga de cada pano pela receita).
void testGoal() {
    std::string error;
    const auto recipe = readObjectRecipe(Objects / "trave.json", &error);
    require(recipe.has_value(), "a receita da trave: " + error);
    const auto goal = generateObject(*recipe, &error);
    require(goal.has_value(), "a trave gera: " + error);
    const Mesh& mesh = goal->mesh;
    require(mesh.validateLayout().empty(), "a malha da trave consistente");
    const float ground = -0.5f * goal->size.z;
    require(near(mesh.bounds().minimum.z, ground, 1e-4f), "os pes no chao");
    // Os postes (cilindros em pe, grossos) e o travessao (deitado ao longo de y).
    float postInner = 1e9f, crossbarBottom = 1e9f;
    int posts = 0, crossbars = 0;
    for (const ObjectCollider& collider : goal->physics.colliders) {
        const Vec3 axis = collider.orientation.rotate({ 0.0f, 0.0f, 1.0f });
        if (collider.radius < 0.055f) continue;
        if (std::abs(axis.z) > 0.99f) {
            ++posts;
            postInner = std::min(postInner, std::abs(collider.center.y) - collider.radius);
        } else if (std::abs(axis.y) > 0.99f) {
            ++crossbars;
            crossbarBottom = std::min(crossbarBottom, collider.center.z - collider.radius - ground);
        }
    }
    require(posts == 2 && crossbars == 1, "dois postes e um travessao de 12 cm");
    require(near(2.0f * postInner, 7.32f, 1e-3f), "o vao de 7,32 m entre as faces de dentro (" + std::to_string(2.0f * postInner) + ")");
    require(near(crossbarBottom, 2.44f, 1e-3f), "o travessao a 2,44 m do chao (" + std::to_string(crossbarBottom) + ")");
    require(goal->physics.fixed && goal->physics.material == "steel", "fixa no chao, de aco");
    require(goal->size.x > 2.0f && goal->size.x < 2.2f && near(goal->size.y, 7.56f, 1e-3f), "2 m de fundo no chao e os postes por fora do vao");
    // Cada tubo (uma parte) fechado e orientado para fora.
    for (int part = 0; part < static_cast<int>(mesh.parts.size()); ++part) {
        std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
        float volume = 0.0f;
        for (const std::uint32_t t : mesh.partTriangles(part)) {
            const Triangle& tri = mesh.triangles[t];
            for (int k = 0; k < 3; ++k) {
                const std::uint32_t a = tri[k], b = tri[(k + 1) % 3];
                ++edges[{ std::min(a, b), std::max(a, b) }];
            }
            volume += dot(mesh.positions[tri[0]], cross(mesh.positions[tri[1]], mesh.positions[tri[2]])) / 6.0f;
        }
        for (const auto& [edge, uses] : edges) require(uses == 2, "o tubo " + mesh.parts[part].name + " fechado");
        require(volume > 0.0f, "o tubo " + mesh.parts[part].name + " para fora");
    }
    // A rede: a da fisica (a celula de 2 malhas) e a de verdade desenhada nela.
    require(goal->net.has_value(), "a trave tem rede");
    const ObjectNet& net = *goal->net;
    require(net.drawnKnots.size() > 2000 && net.drawnWeights.size() == net.drawnKnots.size(), "os nos da rede de verdade (malha de 12 cm)");
    require(net.knots.size() * 3 < net.drawnKnots.size() && net.knots.size() == net.pinned.size() && net.knots.size() == net.masses.size()
            && net.knots.size() == net.twineLengths.size(),
        "a fisica com perto da quarta parte dos nos (" + std::to_string(net.knots.size()) + ")");
    float mass = 0.0f;
    std::size_t pinned = 0;
    const Box3 frame = mesh.bounds();
    const auto inside = [&](Vec3 p) {
        return p.x >= frame.minimum.x && p.x <= frame.maximum.x && p.y >= frame.minimum.y && p.y <= frame.maximum.y && p.z >= frame.minimum.z
            && p.z <= frame.maximum.z;
    };
    for (std::size_t i = 0; i < net.knots.size(); ++i) {
        mass += net.masses[i];
        pinned += net.pinned[i];
        require(inside(net.knots[i]), "a rede dentro da trave");
    }
    for (const Vec3& p : net.drawnKnots) require(inside(p), "a rede desenhada dentro da trave");
    require(mass > 3.0f && mass < 6.0f, "o peso da rede (fio de 7 g/m): " + std::to_string(mass) + " kg");
    require(pinned > 100, "a rede presa nos tubos (" + std::to_string(pinned) + " nos)");
    // Cada no desenhado e a media dos nos da fisica: no repouso, cai nele.
    float worst = 0.0f;
    for (std::size_t i = 0; i < net.drawnKnots.size(); ++i) {
        const ObjectNetWeight& w = net.drawnWeights[i];
        Vec3 p;
        float sum = 0.0f;
        for (int k = 0; k < 4; ++k) {
            require(w.knots[k] < net.knots.size() && w.weights[k] >= 0.0f, "o peso de um no desenhado");
            p = p + net.knots[w.knots[k]] * w.weights[k];
            sum += w.weights[k];
        }
        require(near(sum, 1.0f, 1e-5f), "os pesos de um no desenhado somam 1");
        worst = std::max(worst, std::sqrt((p - net.drawnKnots[i]).lengthSquared()));
    }
    require(worst < 1e-4f, "a malha desenhada sai da fisica parada: " + std::to_string(worst) + " m");
    for (const auto& [a, b] : net.drawnSegments) require(a < net.drawnKnots.size() && b < net.drawnKnots.size() && a != b, "o fio desenhado liga dois nos");
    std::size_t cords = 0;
    std::vector<int> degree(net.knots.size(), 0);
    // A folga de cada pano (o teto 2,5%, o fundo 6%, as laterais 8%): o fio na
    // divisa de dois panos pode ser de qualquer um deles.
    float roofZ = 0.0f, sideY = 0.0f;
    for (const Vec3& p : net.knots) {
        roofZ = std::max(roofZ, p.z);
        sideY = std::max(sideY, std::abs(p.y));
    }
    const std::array<float, 3> slacks { 0.025f, 0.06f, 0.08f };
    std::array<std::size_t, 3> own {};
    for (const ObjectNetSegment& segment : net.segments) {
        require(segment.a < net.knots.size() && segment.b < net.knots.size() && segment.a != segment.b, "o fio liga dois nos");
        ++degree[segment.a];
        ++degree[segment.b];
        const float span = std::sqrt((net.knots[segment.b] - net.knots[segment.a]).lengthSquared());
        if (segment.cord) {
            ++cords;
            require(segment.restLength < span, "o cabo do alto esticado (mais curto que o vao)");
        } else {
            const Vec3 pa = net.knots[segment.a], pb = net.knots[segment.b];
            const bool roof = std::abs(pa.z - roofZ) < 1e-4f && std::abs(pb.z - roofZ) < 1e-4f;
            const bool side = std::abs(std::abs(pa.y) - sideY) < 1e-4f && std::abs(std::abs(pb.y) - sideY) < 1e-4f;
            const bool diagonal = std::abs(pa.x - pb.x) > 1e-4f && std::abs(pa.z - pb.z) > 1e-4f;
            const std::array<bool, 3> can { roof, !roof && (!side || diagonal), side };
            bool matches = false;
            for (std::size_t k = 0; k < 3; ++k) matches = matches || (can[k] && near(segment.restLength, span * (1.0f + slacks[k]), 1e-4f));
            require(matches, "cada fio com a folga do seu pano (teto 2,5%, fundo 6%, laterais 8%)");
            if (can[0] + can[1] + can[2] == 1) ++own[can[0] ? 0 : can[1] ? 1 : 2];
            require(near(segment.stiffness, 20000.0f, 1e-2f), "o fio da fisica vale pelos 2 fios paralelos (a rigidez somada)");
            // (O debrum numa borda inclinada vai de cruzamento a cruzamento:
            // ate a diagonal da celula.)
            require(segment.restLength < 0.24f * std::sqrt(2.0f) * 1.06f, "nenhum fio maior que a diagonal da celula: "
                + std::to_string(segment.restLength));
        }
    }
    require(cords > 25, "o cabo do alto, de ponta a ponta");
    require(own[0] > 50 && own[1] > 50 && own[2] > 50, "fios so do teto, so do fundo e so das laterais");
    for (const int links : degree) require(links >= 1, "nenhum no solto");
    // As celulas cobrem a rede: a mesma area com a celula de 1 malha (a fisica
    // e a propria malha de verdade, cada no desenhado num no dela).
    const auto areaOf = [](const ObjectNet& n) {
        float area = 0.0f;
        for (const auto& face : n.faces) {
            require(face[0] < n.knots.size() && face[1] < n.knots.size() && face[2] < n.knots.size(), "a celula liga tres nos");
            area += 0.5f * cross(n.knots[face[1]] - n.knots[face[0]], n.knots[face[2]] - n.knots[face[0]]).length();
        }
        return area;
    };
    ObjectRecipe single = *recipe;
    single.numbers["malha_fisica"] = 1.0f;
    const auto fine = generateObject(single, &error);
    require(fine && fine->net->knots.size() == fine->net->drawnKnots.size(), "com 1 malha, a fisica e a malha de verdade");
    require(near(net.contactMassScale, 4.0f, 1e-6f) && near(fine->net->contactMassScale, 6.0f, 1e-6f),
        "a escala de contato menor na malha grossa (nos mais pesados)");
    for (std::size_t i = 0; i < fine->net->drawnKnots.size(); ++i) {
        const ObjectNetWeight& w = fine->net->drawnWeights[i];
        const int heaviest = static_cast<int>(std::max_element(w.weights.begin(), w.weights.end()) - w.weights.begin());
        require(w.weights[heaviest] > 0.9999f
                && (fine->net->knots[w.knots[heaviest]] - fine->net->drawnKnots[i]).lengthSquared() < 1e-10f,
            "com 1 malha, cada no desenhado e um no da fisica");
    }
    const float area = areaOf(net), fineArea = areaOf(*fine->net);
    require(area > 25.0f && near(area, fineArea, 1e-3f * fineArea),
        "as celulas cobrem a rede: " + std::to_string(area) + " m2 (com 1 malha, " + std::to_string(fineArea) + ")");
    const auto again = generateObject(*recipe);
    require(again && again->net->knots.size() == net.knots.size() && again->net->segments.size() == net.segments.size()
            && again->net->drawnKnots.size() == net.drawnKnots.size(),
        "a mesma rede");
    for (std::size_t i = 0; i < net.knots.size(); ++i) require(again->net->knots[i].x == net.knots[i].x, "a mesma rede, no a no");
    for (std::size_t i = 0; i < net.drawnKnots.size(); ++i) require(again->net->drawnKnots[i].x == net.drawnKnots[i].x, "a mesma rede desenhada");
    // Outro vao: a rede e a estrutura acompanham.
    ObjectRecipe small = *recipe;
    small.numbers["largura"] = 5.0f;
    small.numbers["altura"] = 2.0f;
    const auto smaller = generateObject(small, &error);
    require(smaller.has_value() && near(smaller->size.y, 5.24f, 1e-3f) && smaller->net->knots.size() < net.knots.size(),
        "a trave de 5 x 2 m gera junto com a rede");
}

int main() {
    try {
        testRecipes();
        testCrate();
        testLongCrate();
        testPixelCanvas();
        testGoal();
    } catch (const std::exception& exception) {
        std::cerr << "MatterObjectModeling test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "MatterObjectModeling tests passed (" << checks << " checagens)\n";
    return 0;
}
