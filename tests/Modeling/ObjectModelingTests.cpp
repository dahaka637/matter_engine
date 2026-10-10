// Os objetos procedurais (o nucleo, Modeling/Objects): a receita (ler,
// gravar, padroes, erros), a caixa gerada (o bloco fechado e orientado para
// fora, o mapa de textura por face, a densidade de pixels igual nas faces, a
// pixel art deterministica pela semente) e a descricao fisica. Linkado so
// contra MatterModeling.

#include "Engine/Modeling/Analysis/TopologyCheck.hpp"
#include "Engine/Modeling/Objects/ObjectRecipe.hpp"
#include "Engine/Modeling/Objects/PixelCanvas.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

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

int main() {
    try {
        testRecipes();
        testCrate();
        testLongCrate();
        testPixelCanvas();
    } catch (const std::exception& exception) {
        std::cerr << "MatterObjectModeling test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "MatterObjectModeling tests passed (" << checks << " checagens)\n";
    return 0;
}
