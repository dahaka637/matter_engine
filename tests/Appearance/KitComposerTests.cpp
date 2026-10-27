// O compositor do uniforme (etapa 2 do sistema de aparencia; o contrato no vault,
// 06_render/plano_sistema_de_aparencia.md): na troca instalada, o desenho
// branco e o de tortura compostos; o determinismo (1 e N threads, bit a bit), o
// tecido todo coberto, a continuidade nas costuras continuas (um desenho na
// referencia 3D, continuo em toda costura), a cobertura conjunta da forma e da
// mascara, os documentos invalidos recusados e o numero nunca espelhado.
#include "Engine/Appearance/Compose/KitCompositor.hpp"
#include "Engine/Appearance/Compose/FootprintCoverage.hpp"
#include "Engine/Appearance/Compose/KitMaterial.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace MatterEngine;
using namespace MatterEngine::Appearance;
using Json = nlohmann::json;

namespace {

int checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

const std::string Assets = MATTERENGINE_TEST_ASSETS_DIR;

struct Installed {
    SurfaceTemplate surface;
    std::vector<std::array<std::uint32_t, 3>> triangles;
};

Installed loadInstalled() {
    std::ifstream file(Assets + "/characters/player_model/blender/jogador.troca.json");
    require(static_cast<bool>(file), "a troca do jogador");
    Json document;
    file >> document;
    Installed out;
    out.triangles = document.at("triangles").get<std::vector<std::array<std::uint32_t, 3>>>();
    std::string error;
    std::optional<SurfaceTemplate> surface = parseSurfaceTemplate(document, out.triangles, document.at("positions").size(), &error);
    require(surface.has_value(), "o template instalado: " + error);
    out.surface = std::move(*surface);
    return out;
}

ComposedKit compose(const Installed& installed, const LoadedKit& loaded, int threads) {
    std::string error;
    ComposeOptions options;
    options.threads = threads;
    std::optional<ComposedKit> composed = composeKit(installed.surface, installed.triangles, *loaded.design, loaded.kit, options, &error);
    require(composed.has_value(), "compor: " + error);
    return std::move(*composed);
}

std::array<float, 4> sample(const KitTexture& texture, Vec2 uv) {
    // O bilinear (o centro do texel em +0,5; a borda repetida).
    const float x = uv.x * texture.width - 0.5f, y = uv.y * texture.height - 0.5f;
    const int x0 = static_cast<int>(std::floor(x)), y0 = static_cast<int>(std::floor(y));
    const float fx = x - x0, fy = y - y0;
    std::array<float, 4> out {};
    for (int j = 0; j < 2; ++j)
        for (int i = 0; i < 2; ++i) {
            const int cx = std::clamp(x0 + i, 0, texture.width - 1), cy = std::clamp(y0 + j, 0, texture.height - 1);
            const float w = (i ? fx : 1.0f - fx) * (j ? fy : 1.0f - fy);
            for (int k = 0; k < 4; ++k)
                out[static_cast<std::size_t>(k)] += w * texture.rgba[(static_cast<std::size_t>(cy) * texture.width + cx) * 4 + k] / 255.0f;
        }
    return out;
}

void testWhiteAndTorture(const Installed& installed) {
    for (const char* id : { "branco_10", "tortura_10" }) {
        std::string error;
        std::optional<LoadedKit> loaded = loadKit(Assets + "/appearance", id, installed.surface, &error);
        require(loaded.has_value(), std::string(id) + ": " + error);
        const ComposedKit one = compose(installed, *loaded, 1);
        const ComposedKit many = compose(installed, *loaded, 8);
        require(one.kit.size() == many.kit.size() && one.identity.size() == many.identity.size(), std::string(id) + ": os niveis");
        for (std::size_t l = 0; l < one.kit.size(); ++l)
            require(one.kit[l].rgba == many.kit[l].rgba, std::string(id) + ": 1 e 8 threads iguais no nivel " + std::to_string(l));
        for (std::size_t l = 0; l < one.identity.size(); ++l)
            require(one.identity[l].rgba == many.identity[l].rgba, std::string(id) + ": a identificacao igual no nivel " + std::to_string(l));
        // (A camisa e o calcao pintados inteiros; o alto do meiao pode mostrar a pele.)
        for (const char* region : { "camisa", "calcao" }) {
            const std::size_t holes = one.transparentTexels[static_cast<std::size_t>(installed.surface.regionIndex(region))];
            require(holes == 0, std::string(id) + ": " + region + " sem tinta em " + std::to_string(holes) + " texels");
        }
        require(one.kitSafeLevel >= 2, std::string(id) + ": nivel seguro do atlas " + std::to_string(one.kitSafeLevel));
        require(one.identitySafeLevel >= 2, std::string(id) + ": nivel seguro da identificacao " + std::to_string(one.identitySafeLevel));
        std::printf("%s: atlas %dx%d, %zu niveis, seguro %d; identificacao %dx%d, segura %d; triangulos sem texel %zu; "
                    "%.0f ms (preparo %.0f, tecido %.0f, margens %.0f, mips %.0f, identificacao %.0f) com 8 threads\n",
            id, many.kit.front().width, many.kit.front().height, many.kit.size(), many.kitSafeLevel, many.identity.front().width,
            many.identity.front().height, many.identitySafeLevel, many.trianglesWithoutTexel, many.milliseconds.total,
            many.milliseconds.prepare, many.milliseconds.compose, many.milliseconds.margins, many.milliseconds.mips,
            many.milliseconds.identity);
        // O numero tem tinta nas tres zonas.
        for (const SurfaceZone& zone : installed.surface.zones) {
            float ink = 0.0f;
            for (int y = zone.block[1]; y < zone.block[1] + zone.block[3]; ++y)
                for (int x = zone.block[0]; x < zone.block[0] + zone.block[2]; ++x)
                    ink += many.identity.front().rgba[(static_cast<std::size_t>(y) * many.identity.front().width + x) * 4 + 3];
            require(ink > 0.0f, std::string(id) + ": o numero na zona " + zone.id);
        }
    }
}

// O numero nunca espelhado: na zona, o UV1 anda com a coordenada de desenho (u
// cresce com x, v desce com y) - o "1" fica a esquerda do "0", visto de fora.
void testIdentityOrientation(const Installed& installed) {
    const SurfaceTemplate& s = installed.surface;
    for (std::size_t z = 0; z < s.zones.size(); ++z) {
        int checked = 0;
        for (std::size_t t = 0; t < s.trianglePanel.size(); ++t) {
            if (s.triangleIdentity[t] != static_cast<int>(z)) continue;
            const Vec2 d1 = s.triangleDesign[t][1] - s.triangleDesign[t][0], d2 = s.triangleDesign[t][2] - s.triangleDesign[t][0];
            const Vec2 u1 = s.triangleIdentityUv[t][1] - s.triangleIdentityUv[t][0], u2 = s.triangleIdentityUv[t][2] - s.triangleIdentityUv[t][0];
            const float design = d1.x * d2.y - d1.y * d2.x, uv = u1.x * u2.y - u1.y * u2.x;
            if (std::abs(design) < 1e-12f) continue;
            // (v para baixo: a orientacao do UV1 e a oposta a do desenho.)
            require(design * uv < 0.0f, "zona " + s.zones[z].id + ": o UV1 espelhado no triangulo " + std::to_string(t));
            ++checked;
        }
        require(checked > 0, "zona " + s.zones[z].id + " sem triangulos");
    }
}

// Um desenho de prova na referencia 3D (continuo e suave em toda costura): dois
// degrades cruzados (a altura e o lado), em cores intermediarias. (Listras finas
// dao diferencas de alguns /255 so pela grade de texels de cada lado, com a borda
// da listra em cima da costura: a prova da costura e com o desenho suave.)
Json probeDesign(const SurfaceTemplate& s) {
    Json regions = Json::array();
    for (const char* region : { "camisa", "camisa_recuo", "calcao", "calcao_recuo", "meiao" })
        if (s.regionIndex(region) >= 0) regions.push_back(region);
    return Json {
        { "schema", "matter.design.v1" }, { "id", "prova" },
        { "templates", Json::array({ { { "parametrization", s.parametrization }, { "storage", s.storage } } }) },
        { "slots", Json::array({ { { "id", "a" }, { "default", "#4a7fb3" } }, { { "id", "b" }, { "default", "#d9b45c" } },
                       { { "id", "c" }, { "default", "#8c3a52" } } }) },
        { "layers", Json::array({
                        { { "id", "degrade" }, { "target", { { "regions", regions } } }, { "space", "reference" },
                            { "content", { { "type", "gradient" }, { "from", { 0.0, -0.9 } }, { "to", { 0.0, 0.6 } }, { "colors", { "a", "b" } } } } },
                        { { "id", "lados" }, { "target", { { "regions", regions } } }, { "space", "reference" },
                            { "content", { { "type", "gradient" }, { "from", { -0.3, 0.0 } }, { "to", { 0.3, 0.0 } }, { "colors", { "a", "c" } } } },
                            { "opacity", 0.5 } } }) } };
}

void testSeamContinuity(const Installed& installed) {
    const SurfaceTemplate& s = installed.surface;
    std::string error;
    std::optional<KitDesign> design = parseKitDesign(probeDesign(s), s, nullptr, &error);
    require(design.has_value(), "o desenho de prova: " + error);
    Kit kit;
    kit.id = "prova";
    kit.design = "prova";
    kit.colors = design->slotDefaults;
    std::optional<ComposedKit> composed = composeKit(s, installed.triangles, *design, kit, {}, &error);
    require(composed.has_value(), "compor a prova: " + error);
    // Cada aresta de costura continua: a cor dos dois lados no meio dela.
    const auto key = [](std::uint32_t a, std::uint32_t b) { return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b); };
    std::map<std::uint64_t, std::vector<std::pair<std::size_t, int>>> sides;
    for (std::size_t t = 0; t < installed.triangles.size(); ++t)
        if (s.trianglePanel[t] >= 0)
            for (int k = 0; k < 3; ++k) sides[key(installed.triangles[t][k], installed.triangles[t][(k + 1) % 3])].push_back({ t, k });
    int edges = 0;
    std::array<float, 32> perLevel {};
    for (const SurfaceSeam& seam : s.seams) {
        if (seam.kind != SeamKind::Seam || !seam.continuous) continue;
        for (const auto& edge : seam.edges) {
            const auto found = sides.find(key(edge[0], edge[1]));
            if (found == sides.end() || found->second.size() != 2) continue;
            for (int level = 0; level <= composed->kitSafeLevel; ++level) {
                std::array<std::array<float, 4>, 2> color {};
                for (int side = 0; side < 2; ++side) {
                    const auto [t, k] = found->second[static_cast<std::size_t>(side)];
                    const Vec2 a = s.triangleKitUv[t][k], b = s.triangleKitUv[t][(k + 1) % 3];
                    color[static_cast<std::size_t>(side)] = sample(composed->kit[static_cast<std::size_t>(level)], (a + b) * 0.5f);
                }
                for (int c = 0; c < 3; ++c) {
                    perLevel[static_cast<std::size_t>(level)] = std::max(perLevel[static_cast<std::size_t>(level)], std::abs(color[0][c] - color[1][c]));
                    require(std::abs(color[0][c] - color[1][c]) <= 2.0f / 255.0f + 1e-4f,
                        "costura " + seam.id + ", nivel " + std::to_string(level) + ": " + std::to_string(color[0][c] * 255.0f)
                            + " x " + std::to_string(color[1][c] * 255.0f));
                }
            }
            ++edges;
        }
    }
    require(edges > 0, "nenhuma aresta de costura continua conferida");
    std::printf("costuras continuas: %d arestas, pior diferenca por nivel (/255):", edges);
    for (int level = 0; level <= composed->kitSafeLevel; ++level) std::printf(" %.2f", perLevel[static_cast<std::size_t>(level)] * 255.0f);
    std::printf("\n");
}

// A forma e a mascara iguais com meia cobertura: o produto por amostra da meia
// cobertura (dois produtos ja integrados dariam um quarto).
void testJointCoverage(const Installed& installed) {
    const SurfaceTemplate& s = installed.surface;
    // Meio plano do painel da frente (x < 1,5 mm), forma e mascara iguais. (A
    // borda a ~0,6 texel da linha do meio: o layout poe x = 0 na divisa dos
    // texels, e nenhum ficaria dividido.)
    constexpr float Edge = 0.0015f;
    const std::string half = "M -10 -10 L 0.0015 -10 L 0.0015 10 L -10 10 Z";
    Json document = Json {
        { "schema", "matter.design.v1" }, { "id", "conjunta" },
        { "templates", Json::array({ { { "parametrization", s.parametrization }, { "storage", s.storage } } }) },
        { "slots", Json::array({ { { "id", "fundo" }, { "default", "#000000" } }, { { "id", "tinta" }, { "default", "#ffffff" } } }) },
        { "layers", Json::array({
                        { { "id", "fundo" }, { "target", { { "regions", { "camisa" } } } }, { "content", { { "type", "fill" } } }, { "color", "fundo" } },
                        { { "id", "meio" }, { "target", { { "panels", { "camisa_frente" } } } },
                            { "content", { { "type", "path" }, { "d", half } } }, { "mask", { { "d", half } } }, { "color", "tinta" } } }) } };
    std::string error;
    std::optional<KitDesign> design = parseKitDesign(document, s, nullptr, &error);
    require(design.has_value(), "o desenho da cobertura conjunta: " + error);
    Kit kit { "conjunta", "conjunta", design->slotDefaults, {} };
    std::optional<ComposedKit> composed = composeKit(s, installed.triangles, *design, kit, {}, &error);
    require(composed.has_value(), "compor a cobertura conjunta: " + error);
    // Nos texels sobre a linha x = 0 da frente (a malha tem uma fileira de
    // vertices nela), o valor e a fracao do texel com x < 0 (medida aqui em 32x32
    // pela transformacao do triangulo); o produto de duas coberturas ja
    // integradas daria a fracao ao quadrado.
    const int frente = s.panelIndex("camisa_frente");
    const KitTexture& level0 = composed->kit.front();
    const float size = static_cast<float>(level0.width);
    int found = 0, distinguished = 0;
    for (std::size_t t = 0; t < s.trianglePanel.size(); ++t) {
        if (s.trianglePanel[t] != frente) continue;
        const std::array<Vec2, 3>& c = s.triangleKitUv[t];
        const std::array<Vec2, 3>& d = s.triangleDesign[t];
        const auto edge = [](Vec2 p, Vec2 q, Vec2 r) { return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x); };
        const float area = edge(c[0], c[1], c[2]);
        const auto designX = [&](Vec2 uv) {
            const float w0 = edge(c[1], c[2], uv) / area, w1 = edge(c[2], c[0], uv) / area;
            return w0 * d[0].x + w1 * d[1].x + (1.0f - w0 - w1) * d[2].x;
        };
        if (std::abs(d[0].x) > 1e-4f && std::abs(d[1].x) > 1e-4f && std::abs(d[2].x) > 1e-4f) continue;
        const int x0 = static_cast<int>(std::floor(std::min({ c[0].x, c[1].x, c[2].x }) * size));
        const int x1 = static_cast<int>(std::floor(std::max({ c[0].x, c[1].x, c[2].x }) * size));
        const int y0 = static_cast<int>(std::floor(std::min({ c[0].y, c[1].y, c[2].y }) * size));
        const int y1 = static_cast<int>(std::floor(std::max({ c[0].y, c[1].y, c[2].y }) * size));
        for (int y = std::max(0, y0); y <= std::min(level0.height - 1, y1); ++y)
            for (int x = std::max(0, x0); x <= std::min(level0.width - 1, x1); ++x) {
                const Vec2 center { (static_cast<float>(x) + 0.5f) / size, (static_cast<float>(y) + 0.5f) / size };
                const float w0 = edge(c[1], c[2], center) / area, w1 = edge(c[2], c[0], center) / area;
                if (w0 < 0.0f || w1 < 0.0f || w0 + w1 > 1.0f) continue;
                int inside = 0;
                for (int i = 0; i < 32; ++i)
                    for (int j = 0; j < 32; ++j)
                        if (designX({ (static_cast<float>(x) + (i + 0.5f) / 32.0f) / size, (static_cast<float>(y) + (j + 0.5f) / 32.0f) / size }) < Edge)
                            ++inside;
                const float fraction = static_cast<float>(inside) / 1024.0f;
                if (fraction <= 0.0f || fraction >= 1.0f) continue;
                const float value = level0.rgba[(static_cast<std::size_t>(y) * level0.width + x) * 4] / 255.0f;
                // (A grade de 4x4 amostras erra ate ~1/8 numa borda reta: meia fileira.)
                require(std::abs(value - fraction) < 0.13f, "a cobertura conjunta: " + std::to_string(value) + " contra a fracao " + std::to_string(fraction));
                if (fraction > 0.25f && fraction < 0.75f) {
                    require(std::abs(value - fraction) < std::abs(value - fraction * fraction), "a cobertura conjunta nao e o produto das integradas");
                    ++distinguished;
                }
                ++found;
            }
    }
    std::printf("cobertura conjunta: %d texels na linha, %d distinguem a fracao do quadrado\n", found, distinguished);
    require(distinguished > 0, "nenhum texel com meia cobertura na linha");
    require(found > 0, "nenhum texel na borda da forma");
}

void testRejections(const Installed& installed) {
    const SurfaceTemplate& s = installed.surface;
    std::string error;
    const auto rejects = [&](Json document, const std::string& what) {
        require(!parseKitDesign(document, s, nullptr, &error).has_value(), "recusa: " + what);
    };
    const Json good = probeDesign(s);
    Json bad = good;
    bad["schema"] = "matter.design.v0";
    rejects(bad, "schema errado");
    bad = good;
    bad["layers"][0]["target"] = Json::object();
    rejects(bad, "camada sem alvo");
    bad = good;
    bad["layers"][1]["content"]["colors"][1] = "nao_existe";
    rejects(bad, "vaga desconhecida no degrade");
    bad = good;
    bad["layers"][0]["content"] = { { "type", "fill" } };
    bad["layers"][0]["color"] = "nao_existe";
    rejects(bad, "vaga desconhecida");
    bad = good;
    bad["layers"][0]["target"] = { { "panels", { "painel_que_nao_existe" } } };
    rejects(bad, "painel desconhecido");
    bad = good;
    bad["templates"][0]["storage"] = s.storage + 1;
    rejects(bad, "template de outra versao");
    bad = good;
    bad["layers"][0]["space"] = "chart:carta_que_nao_existe";
    rejects(bad, "carta desconhecida");
    bad = good;
    bad["layers"][0]["target"] = { { "panels", { "camisa_frente" } } };
    bad["layers"][0]["space"] = "panel";
    bad["layers"][0]["content"] = { { "type", "border" }, { "seam", "gola" }, { "width", 0.018 } };
    bad["layers"][0]["color"] = "a";
    require(parseKitDesign(bad, s, nullptr, &error).has_value(), "a borda de prova: " + error);
    bad["layers"][0]["mask"] = { { "d", "M -1 -1 L 1 -1 L 1 1 Z" } };
    rejects(bad, "borda com mascara");
    const std::optional<KitDesign> design = parseKitDesign(good, s, nullptr, &error);
    require(design.has_value(), "o desenho de prova");
    const auto designOf = [&](const std::string& id) { return id == "prova" ? &*design : nullptr; };
    require(!parseKit(Json { { "schema", "matter.kit.v1" }, { "id", "x" }, { "design", "prova" }, { "colors", { { "nao_existe", "#ffffff" } } } },
                designOf, &error).has_value(), "recusa: cor para vaga desconhecida");
    require(!parseKit(Json { { "schema", "matter.kit.v1" }, { "id", "x" }, { "design", "outro" } }, designOf, &error).has_value(),
        "recusa: desenho desconhecido");
}

// Os valores compostos de um painel contra uma referencia: para cada texel com o
// centro num triangulo do painel, a funcao de referencia avaliada em n x n
// amostras pela transformacao do triangulo (UV para coordenada de desenho).
template <typename Reference>
void compareWithReference(const SurfaceTemplate& s, const ComposedKit& composed, const char* panelName, int n, const Reference& reference,
    float maximum, float meanLimit, const std::string& what) {
    const int panel = s.panelIndex(panelName);
    const KitTexture& level0 = composed.kit.front();
    const float size = static_cast<float>(level0.width);
    double sum = 0.0;
    float worst = 0.0f;
    int count = 0;
    const auto edge = [](Vec2 p, Vec2 q, Vec2 r) { return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x); };
    for (std::size_t t = 0; t < s.trianglePanel.size(); ++t) {
        if (s.trianglePanel[t] != panel) continue;
        const std::array<Vec2, 3>& c = s.triangleKitUv[t];
        const std::array<Vec2, 3>& d = s.triangleDesign[t];
        const float area = edge(c[0], c[1], c[2]);
        const auto design = [&](Vec2 uv) {
            const float w0 = edge(c[1], c[2], uv) / area, w1 = edge(c[2], c[0], uv) / area, w2 = 1.0f - w0 - w1;
            return Vec2 { w0 * d[0].x + w1 * d[1].x + w2 * d[2].x, w0 * d[0].y + w1 * d[1].y + w2 * d[2].y };
        };
        const int x0 = static_cast<int>(std::floor(std::min({ c[0].x, c[1].x, c[2].x }) * size));
        const int x1 = static_cast<int>(std::floor(std::max({ c[0].x, c[1].x, c[2].x }) * size));
        const int y0 = static_cast<int>(std::floor(std::min({ c[0].y, c[1].y, c[2].y }) * size));
        const int y1 = static_cast<int>(std::floor(std::max({ c[0].y, c[1].y, c[2].y }) * size));
        for (int y = std::max(0, y0); y <= std::min(level0.height - 1, y1); ++y)
            for (int x = std::max(0, x0); x <= std::min(level0.width - 1, x1); ++x) {
                const Vec2 center { (static_cast<float>(x) + 0.5f) / size, (static_cast<float>(y) + 0.5f) / size };
                const float w0 = edge(c[1], c[2], center) / area, w1 = edge(c[2], c[0], center) / area;
                if (w0 < 0.0f || w1 < 0.0f || w0 + w1 > 1.0f) continue;
                double expected = 0.0;
                for (int i = 0; i < n; ++i)
                    for (int j = 0; j < n; ++j)
                        expected += reference(design({ (static_cast<float>(x) + (i + 0.5f) / n) / size, (static_cast<float>(y) + (j + 0.5f) / n) / size }));
                expected /= static_cast<double>(n * n);
                const float value = level0.rgba[(static_cast<std::size_t>(y) * level0.width + x) * 4] / 255.0f;
                const float error = std::abs(value - static_cast<float>(expected));
                worst = std::max(worst, error);
                sum += error;
                ++count;
            }
    }
    require(count > 0, what + ": nenhum texel");
    std::printf("%s: %d texels, erro max %.3f, medio %.4f\n", what.c_str(), count, worst, sum / count);
    require(worst <= maximum && sum / count <= meanLimit, what + ": erro max " + std::to_string(worst) + ", medio " + std::to_string(sum / count));
}

Json oneLayerDesign(const SurfaceTemplate& s, const Json& layer) {
    return Json {
        { "schema", "matter.design.v1" }, { "id", "uma" },
        { "templates", Json::array({ { { "parametrization", s.parametrization }, { "storage", s.storage } } }) },
        { "slots", Json::array({ { { "id", "fundo" }, { "default", "#000000" } }, { { "id", "tinta" }, { "default", "#ffffff" } },
                       { { "id", "nada" }, { "default", "#00000000" } } }) },
        { "layers", Json::array({ { { "id", "fundo" }, { "target", { { "regions", { "camisa" } } } }, { "content", { { "type", "fill" } } }, { "color", "fundo" } },
                        layer }) } };
}

ComposedKit composeDesign(const Installed& installed, const Json& document, const FontResolver& fonts = nullptr) {
    std::string error;
    std::optional<KitDesign> design = parseKitDesign(document, installed.surface, fonts, &error);
    require(design.has_value(), "o desenho de prova: " + error);
    Kit kit { "prova", "prova", design->slotDefaults, {} };
    std::optional<ComposedKit> composed = composeKit(installed.surface, installed.triangles, *design, kit, {}, &error);
    require(composed.has_value(), "compor a prova: " + error);
    return std::move(*composed);
}

// Caminho e texto (as curvas e o furo do "0" da fonte em bloco, e um traco de 1
// mm, menor que o texel) contra a referencia de 16x16 amostras: a grade de 4x4
// erra ate ~1/8 numa borda reta e um pouco mais nas curvas; na media, pouco.
void testShapeAgainstReference(const Installed& installed) {
    const SurfaceTemplate& s = installed.surface;
    std::string error;
    const std::optional<GlyphFont> font = loadGlyphFont(Assets + "/appearance/fonts/numeros_bloco.json", &error);
    require(font.has_value(), "a fonte: " + error);
    const auto shared = std::make_shared<const GlyphFont>(*font);
    const FontResolver fonts = [&](const std::string& id) { return id == "numeros_bloco" ? shared : nullptr; };
    const Json text = { { "id", "texto" }, { "target", { { "panels", { "camisa_frente" } } } },
        { "content", { { "type", "text" }, { "font", "numeros_bloco" }, { "text", "80" }, { "height", 0.12 }, { "spacing", 0.07 } } },
        { "transform", { { "offset", { 0.0, 0.30 } } } }, { "color", "tinta" } };
    const ComposedKit withText = composeDesign(installed, oneLayerDesign(s, text), fonts);
    const std::optional<VectorShape> shape = layoutText(*font, U"80", 0.12f, 0.07f, 1.0f, 0, &error);
    require(shape.has_value(), "o texto de prova");
    const VectorShape placed = shape->transformed({ 1.0f, 0.0f }, { 0.0f, 1.0f }, { 0.0f, 0.30f });
    compareWithReference(s, withText, "camisa_frente", 16, [&](Vec2 p) { return placed.contains(p) ? 1.0 : 0.0; }, 0.19f, 0.01f,
        "texto contra 16x16");
    const std::string thin = "M -0.10 0.10 L 0.10 0.13 L 0.10 0.131 L -0.10 0.101 Z";
    const Json stroke = { { "id", "traco" }, { "target", { { "panels", { "camisa_frente" } } } },
        { "content", { { "type", "path" }, { "d", thin } } }, { "color", "tinta" } };
    const ComposedKit withStroke = composeDesign(installed, oneLayerDesign(s, stroke));
    const std::optional<VectorShape> line = parsePathData(thin, false, &error);
    compareWithReference(s, withStroke, "camisa_frente", 16, [&](Vec2 p) { return line->contains(p) ? 1.0 : 0.0; }, 0.19f, 0.01f,
        "traco de 1 mm contra 16x16");
}

// O degrade (o caso do Codex: do preto ao branco em pouco mais de um texel) contra
// a media exata da rampa presa no texel.
void testGradientFootprint(const Installed& installed) {
    const SurfaceTemplate& s = installed.surface;
    const Json gradient = { { "id", "degrade" }, { "target", { { "panels", { "camisa_frente" } } } },
        { "content", { { "type", "gradient" }, { "from", { 0.0, 0.0 } }, { "to", { 0.003, 0.0 } }, { "colors", { "fundo", "tinta" } } } } };
    const ComposedKit composed = composeDesign(installed, oneLayerDesign(s, gradient));
    compareWithReference(s, composed, "camisa_frente", 32, [&](Vec2 p) { return std::clamp(p.x / 0.003, 0.0, 1.0); }, 3.0f / 255.0f,
        1.0f / 255.0f, "degrade contra a media exata");
    // Na diagonal (o footprint e um quadrado inclinado na direcao do degrade: a
    // soma das derivadas como intervalo daria 1/4 onde o exato e 1/6).
    const auto diagonal = [](Vec2 p) { return std::clamp((static_cast<double>(p.x) + p.y) / 0.004, 0.0, 1.0); };
    const Json oblique = { { "id", "degrade" }, { "target", { { "panels", { "camisa_frente" } } } },
        { "content", { { "type", "gradient" }, { "from", { 0.0, 0.0 } }, { "to", { 0.002, 0.002 } }, { "colors", { "fundo", "tinta" } } } } };
    compareWithReference(s, composeDesign(installed, oneLayerDesign(s, oblique)), "camisa_frente", 32, diagonal, 3.0f / 255.0f,
        1.0f / 255.0f, "degrade diagonal contra a media exata");
    // Do transparente ao branco por cima do preto: pre-multiplicado, a cor e a
    // posicao (a media dela no texel); a media sem pre-multiplicar daria o quadrado.
    const Json fading = { { "id", "degrade" }, { "target", { { "panels", { "camisa_frente" } } } },
        { "content", { { "type", "gradient" }, { "from", { 0.0, 0.0 } }, { "to", { 0.002, 0.002 } }, { "colors", { "nada", "tinta" } } } } };
    compareWithReference(s, composeDesign(installed, oneLayerDesign(s, fading)), "camisa_frente", 32, diagonal, 3.0f / 255.0f,
        1.0f / 255.0f, "degrade com alfa contra a media exata");
    const Json wide = { { "id", "degrade" }, { "target", { { "panels", { "camisa_frente" } } } },
        { "content", { { "type", "gradient" }, { "from", { -0.1, 0.0 } }, { "to", { 0.1, 0.0 } }, { "colors", { "nada", "tinta" } } } } };
    compareWithReference(s, composeDesign(installed, oneLayerDesign(s, wide)), "camisa_frente", 8,
        [](Vec2 p) { return std::clamp((static_cast<double>(p.x) + 0.1) / 0.2, 0.0, 1.0); }, 2.0f / 255.0f, 1.0f / 255.0f,
        "degrade largo com alfa (linear, nao o quadrado)");
    // O footprint minusculo nas duas direcoes (um degrade de 200 km: 1e-8 por
    // texel): a media e o meio exato. (A diferenca de quatro primitivas grandes
    // quase iguais dava 0,62.)
    const Json huge = { { "id", "degrade" }, { "target", { { "panels", { "camisa_frente" } } } },
        { "content", { { "type", "gradient" }, { "from", { -1.0e5, -1.0e5 } }, { "to", { 1.0e5, 1.0e5 } }, { "colors", { "fundo", "tinta" } } } } };
    compareWithReference(s, composeDesign(installed, oneLayerDesign(s, huge)), "camisa_frente", 4, [](Vec2) { return 0.5; }, 1.0f / 255.0f,
        1.0f / 255.0f, "degrade de footprint minusculo");
    // As listras numa escala de 1e6 (o footprint de ~2,5e-9 da listra): a borda
    // em x = 0 continua exata.
    const Json scaled = { { "id", "listras" }, { "target", { { "panels", { "camisa_frente" } } } },
        { "content", { { "type", "stripes" }, { "period", 1.0 }, { "width", 0.5 }, { "angle", 0.0 } } },
        { "transform", { { "scale", { 1.0e6, 1.0e6 } } } }, { "color", "tinta" } };
    compareWithReference(s, composeDesign(installed, oneLayerDesign(s, scaled)), "camisa_frente", 32,
        [](Vec2 p) { return p.x >= 0.0f ? 1.0 : 0.0; }, 3.0f / 255.0f, 1.0f / 255.0f, "listras de footprint minusculo");
    // Com mascara (meio plano a 1,5 mm da linha do meio): a media das amostras
    // aceitas, pre-multiplicada. A grade de 4x4 erra na borda da mascara.
    Json masked = fading;
    masked["mask"] = { { "d", "M -10 -10 L 0.0015 -10 L 0.0015 10 L -10 10 Z" } };
    compareWithReference(s, composeDesign(installed, oneLayerDesign(s, masked)), "camisa_frente", 32,
        [&](Vec2 p) { return p.x < 0.0015f ? diagonal(p) : 0.0; }, 0.19f, 0.01f, "degrade com alfa e mascara contra 32x32");
}

// A cobertura no footprint contra a forca bruta (n x n amostras: o erro dela e
// ~1/n numa borda) em parametros sorteados de 1e-9 a 1, e os extremos que o Codex
// achou: o footprint minusculo nas duas direcoes, a rampa enorme, a fase
// negativa minuscula da listra e o lado quase nulo.
void testFootprintCoverage() {
    constexpr int N = 256;
    const auto brute = [](auto f, double s, double a, double b) {
        double sum = 0.0;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) sum += f(s + a * ((i + 0.5) / N - 0.5) + b * ((j + 0.5) / N - 0.5));
        return sum / (static_cast<double>(N) * N);
    };
    const auto ramp = [](double u) { return std::clamp(u, 0.0, 1.0); };
    const auto step = [](double u) { return u >= 0.0 ? 1.0 : 0.0; };
    const double period = 0.02, width = 0.007;
    const auto wave = [&](double u) { return u - std::floor(u / period) * period < width ? 1.0 : 0.0; };
    std::uint64_t seed = 12345;
    const auto uniform = [&](double lo, double hi) {
        seed = seed * 6364136223846793005ull + 1442695040888963407ull;
        return lo + (hi - lo) * static_cast<double>(seed >> 11) / 9007199254740992.0;
    };
    double worstRamp = 0.0, worstStep = 0.0, worstWave = 0.0;
    for (int n = 0; n < 300; ++n) {
        const double scale = std::pow(10.0, uniform(-9.0, 0.0));
        const double ratio = n % 3 == 0 ? 1.0 : n % 3 == 1 ? 1e-7 : 0.3;
        const double a = uniform(-1.0, 1.0) * scale, b = uniform(-1.0, 1.0) * scale * ratio;
        // (Metade perto das quinas, onde o footprint importa.)
        const double s = n % 2 == 0 ? uniform(-2.0, 2.0) : (n % 4 == 1 ? 0.0 : 1.0) + uniform(-1.0, 1.0) * scale;
        worstRamp = std::max(worstRamp, std::abs(rampMean(s, a, b) - brute(ramp, s, a, b)));
        worstStep = std::max(worstStep, std::abs(stepMean(s, a, b) - brute(step, s, a, b)));
        // (A forca bruta so ate 8 periodos: alem disso, 256 amostras dao aliasing.)
        const double ws = std::pow(10.0, uniform(-6.0, std::log10(8.0 * period))), wa = uniform(-1.0, 1.0) * ws, wb = uniform(-1.0, 1.0) * ws * ratio;
        const double wsPos = uniform(-3.0, 3.0);
        worstWave = std::max(worstWave, std::abs(stripeMean(wsPos, wa, wb, period, width) - brute(wave, wsPos, wa, wb)));
    }
    std::printf("cobertura no footprint contra %dx%d: rampa %.5f, degrau %.5f, listra %.5f\n", N, N, worstRamp, worstStep, worstWave);
    require(worstRamp < 1e-4 && worstStep < 2.0 / N && worstWave < 2.0 / N, "a cobertura no footprint contra a forca bruta");
    // Os extremos, com o valor exato.
    require(rampMean(0.5, 1e-8, 1e-8) == 0.5, "rampa minuscula nas duas direcoes");
    require(rampMean(1e17, 1e15, 1e15) == 1.0, "rampa enorme toda acima de 1");
    require(rampMean(-1e17, 1e15, 1e15) == 0.0, "rampa enorme toda abaixo de 0");
    require(std::abs(rampMean(0.0, 1e20, 1e20) - 0.5) < 1e-6, "rampa de footprint enorme em cima do degrade");
    require(std::abs(stepMean(0.0, 1e-8, 1e-8) - 0.5) < 1e-12, "degrau minusculo na quina");
    require(std::abs(kinkMean(0.0, 1e-8, 1e-8, 1) - 1e-8 / 6.0) < 1e-20, "dobradica minuscula na quina");
    require(std::abs(stepMean(-0.49998, 1.0, 1e-4) - 2.45e-5) < 1e-9, "degrau com um lado quase nulo");
    require(stripeMean(-1e-18, 1e-18, 1e-18, 1.0, 0.5) == 0.0, "listra: fase negativa minuscula, sem tinta");
    require(stripeMean(1e-18, 1e-18, 1e-18, 1.0, 0.5) == 1.0, "listra: fase positiva minuscula, tinta");
    // Muitos periodos (o caminho largo): a fracao width / period.
    require(std::abs(stripeMean(0.3, 100.0, 37.0, 1.0, 0.5) - 0.5) < 1e-3, "listra: footprint de muitos periodos");
    for (const double wide : { 0.7, 3.0, 50.0 })
        require(std::abs(stripeMean(0.013, wide, 0.4 * wide, period, width) - width / period) < 0.04 / wide, "listra: footprint largo, a fracao");
}

// Faixas sobrepostas sao a uniao: o mesmo desenho, bit a bit, que a faixa unida
// (a soma das coberturas pintaria o texel da borda duas vezes).
void testOverlappingBands(const Installed& installed) {
    const SurfaceTemplate& s = installed.surface;
    const auto bands = [&](const Json& list) {
        return Json { { "id", "faixas" }, { "target", { { "panels", { "camisa_frente" } } } }, { "space", "panel" },
            { "content", { { "type", "bands" }, { "axis", "x" }, { "bands", list } } }, { "color", "tinta" } };
    };
    std::string error;
    const Json overlapping = oneLayerDesign(s, bands(Json::array({ Json::array({ 0.0, 0.05 }), Json::array({ -0.02, 0.10 }), Json::array({ -0.02, 0.03 }),
                                                      Json::array({ 0.10, 0.12 }) })));
    const std::optional<KitDesign> parsed = parseKitDesign(overlapping, s, nullptr, &error);
    require(parsed.has_value() && parsed->layers.back().bands.size() == 1 && parsed->layers.back().bands[0][0] == -0.02f
            && parsed->layers.back().bands[0][1] == 0.12f,
        "as faixas sobrepostas e encostadas viram uma");
    const ComposedKit a = composeDesign(installed, overlapping);
    const ComposedKit b = composeDesign(installed, oneLayerDesign(s, bands(Json::array({ Json::array({ -0.02, 0.12 }) }))));
    require(a.kit.front().rgba == b.kit.front().rgba, "as faixas sobrepostas pintam como a uniao");
    // A borda da uniao contra 32x32 (duas faixas comecam em -0,02: a soma daria o
    // dobro da cobertura nos texels dessa borda).
    // (Numa borda reta, a referencia de 32 amostras erra ate 1/32; a soma
    // erraria ate 1/2.)
    compareWithReference(s, a, "camisa_frente", 32, [](Vec2 p) { return p.x >= -0.02f && p.x < 0.12f ? 1.0 : 0.0; }, 1.0f / 32.0f + 1.0f / 255.0f,
        1.0f / 255.0f, "faixas sobrepostas contra 32x32");
}

// O trilinear do material com um resultado esperado independente: cada nivel de
// uma cor so (o bilinear de uma cor e ela mesma), e o LOD fracionario mistura os
// dois niveis vizinhos; depois do nivel seguro, a mistura com a media da regiao.
// E o bilinear de 2x2 conferido a mao.
void testTrilinear(const Installed& installed) {
    const SurfaceTemplate& s = installed.surface;
    std::string error;
    const std::optional<LoadedKit> loaded = loadKit(Assets + "/appearance", "branco_10", s, &error);
    require(loaded.has_value(), "o branco: " + error);
    ComposedKit kit = compose(installed, *loaded, 0);
    const auto value = [](int level) { return static_cast<float>(20 + 23 * level); };
    for (std::size_t l = 0; l < kit.kit.size(); ++l)
        for (std::size_t i = 0; i < kit.kit[l].rgba.size(); ++i)
            kit.kit[l].rgba[i] = static_cast<std::uint8_t>(i % 4 == 3 ? 255 : value(static_cast<int>(l)));
    std::size_t t = 0;
    while (s.trianglePanel[t] < 0 || s.triangleIdentity[t] >= 0) ++t;
    const std::array<float, 3> center { 0.2f, 0.3f, 0.5f };
    const Rgba& average = kit.regionAverage[s.triangleRegion[t]];
    int checks = 0;
    for (float lod = 0.0f; lod <= static_cast<float>(kit.kitSafeLevel) + 1.5f; lod += 0.125f) {
        const int l0 = std::min(static_cast<int>(std::floor(lod)), kit.kitSafeLevel);
        const int l1 = std::min(l0 + 1, kit.kitSafeLevel);
        const float f = lod >= static_cast<float>(kit.kitSafeLevel) ? 0.0f : lod - std::floor(lod);
        const float cloth = (value(l0) + (value(l1) - value(l0)) * f) / 255.0f;
        const float toAverage = std::clamp(lod - static_cast<float>(kit.kitSafeLevel), 0.0f, 1.0f);
        const float expected = cloth + (average[0] - cloth) * toAverage;
        const Rgba got = kitMaterialColor(kit, s, t, center, lod, 0.0f);
        require(std::abs(got[0] - expected) < 1e-5f, "trilinear no lod " + std::to_string(lod) + ": " + std::to_string(got[0] * 255.0f)
                + " (esperado " + std::to_string(expected * 255.0f) + ")");
        ++checks;
    }
    KitTexture square { 2, 2, { 0, 0, 0, 0, 100, 0, 0, 0, 40, 0, 0, 0, 200, 0, 0, 0 } };
    require(std::abs(kitSample(square, 0.5f, 0.5f)[0] - 85.0f / 255.0f) < 1e-6f, "bilinear no meio");
    require(std::abs(kitSample(square, 0.25f, 0.25f)[0]) < 1e-6f, "bilinear no centro do texel");
    require(std::abs(kitSample(square, 0.5f, 0.25f)[0] - 50.0f / 255.0f) < 1e-6f, "bilinear entre dois");
    require(std::abs(kitSample(square, 1.0f, 1.0f)[0] - 200.0f / 255.0f) < 1e-6f, "bilinear na borda (clamp)");
    std::printf("trilinear: %d niveis fracionarios conferidos ate o seguro + 1,5\n", checks);
}

// A transicao do material (a conta do shader, kitMaterialColor): depois do nivel
// seguro, a media da regiao; a identificacao some ate alfa zero e so vale dentro
// da zona.
void testMaterialTransition(const Installed& installed) {
    const SurfaceTemplate& s = installed.surface;
    std::string error;
    const std::optional<LoadedKit> loaded = loadKit(Assets + "/appearance", "branco_10", s, &error);
    require(loaded.has_value(), "o branco: " + error);
    const ComposedKit kit = compose(installed, *loaded, 0);
    int checkedAverage = 0, checkedInk = 0;
    for (std::size_t t = 0; t < s.trianglePanel.size(); ++t) {
        if (s.trianglePanel[t] < 0) continue;
        const std::array<float, 3> center { 1.0f / 3.0f, 1.0f / 3.0f, 1.0f / 3.0f };
        if (s.triangleIdentity[t] < 0) {
            const Rgba far = kitMaterialColor(kit, s, t, center, static_cast<float>(kit.kitSafeLevel) + 1.0f, 0.0f);
            const Rgba& average = kit.regionAverage[s.triangleRegion[t]];
            for (std::size_t k = 0; k < 4; ++k) require(std::abs(far[k] - average[k]) < 1e-6f, "longe, a media da regiao");
            ++checkedAverage;
            continue;
        }
        // Na zona: o numero perto, nada dele longe (alfa zero), e o mesmo que sem zona.
        const Rgba near = kitMaterialColor(kit, s, t, center, 0.0f, 0.0f);
        const Rgba gone = kitMaterialColor(kit, s, t, center, 0.0f, static_cast<float>(kit.identitySafeLevel) + 1.0f);
        const std::array<Vec2, 3>& uv = s.triangleKitUv[t];
        const Rgba cloth = kitTrilinear(kit.kit, kit.kitSafeLevel, 0.0f, (uv[0].x + uv[1].x + uv[2].x) / 3.0f, (uv[0].y + uv[1].y + uv[2].y) / 3.0f);
        for (std::size_t k = 0; k < 4; ++k) require(std::abs(gone[k] - cloth[k]) < 1e-6f, "a identificacao some ate alfa zero");
        if (std::abs(near[0] - cloth[0]) > 0.1f) ++checkedInk;
    }
    require(checkedAverage > 0 && checkedInk > 0, "a transicao sem casos");
    std::printf("transicao: %d triangulos na media, %d com tinta do numero perto\n", checkedAverage, checkedInk);
}

} // namespace

int main() {
    try {
        const Installed installed = loadInstalled();
        testIdentityOrientation(installed);
        testRejections(installed);
        testJointCoverage(installed);
        testShapeAgainstReference(installed);
        testGradientFootprint(installed);
        testMaterialTransition(installed);
        testTrilinear(installed);
        testFootprintCoverage();
        testOverlappingBands(installed);
        testSeamContinuity(installed);
        testWhiteAndTorture(installed);
    } catch (const std::exception& exception) {
        std::cerr << "MatterKitComposer test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "MatterKitComposer tests passed (" << checks << " checagens)\n";
    return 0;
}
