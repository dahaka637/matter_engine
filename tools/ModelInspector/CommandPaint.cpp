#include "Commands.hpp"

#include "Engine/Modeling/Assembly/AssemblyRecipe.hpp"
#include "Engine/Modeling/Assembly/PlayerPaintSpaces.hpp"
#include "Engine/Modeling/Paint/PaintDesign.hpp"
#include "Engine/Modeling/Raster/TextRenderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

namespace {

// A cor final num ponto da superficie: a base da parte e todas as camadas do
// desenho que caem nele (cada uma no espaco dela).
Color paintedAt(const SurfaceSample& sample, const Mesh& mesh, const PaintDesign& design, const std::map<std::string, PaintSpace>& spaces) {
    Color color = sample.material >= 0 && static_cast<std::size_t>(sample.material) < mesh.materials.size()
        ? mesh.materials[static_cast<std::size_t>(sample.material)].color : Color { 0.8f, 0.8f, 0.8f, 1.0f };
    if (sample.part >= 0 && static_cast<std::size_t>(sample.part) < mesh.parts.size())
        if (const auto base = design.partBase.find(mesh.parts[static_cast<std::size_t>(sample.part)].name); base != design.partBase.end())
            color = base->second;
    for (const PaintLayer& layer : design.layers) {
        const auto space = spaces.find(layer.space);
        if (space == spaces.end()) continue;
        const std::optional<SpacePoint> at = space->second.locate(sample);
        if (at && layer.appliesTo(at->side)) color = paintOver(color, layer, at->point, at->footprint);
    }
    return color;
}

} // namespace

// Os moldes da pintura: cada espaco (PlayerPaintSpaces) desenrolado numa
// imagem - a superficie amostrada a cada "--passo" milimetros, com a cor
// final da pintura em cada ponto (o lado direito; "--lado esquerdo"), a grade
// de 5 cm e as linhas das ancoras com nome. Para desenhar um uniforme novo no
// pintura.json sem adivinhar coordenada.
//   --espaco tronco (padrao: todos)  --passo 3  --escala 2000 (px por metro)
int runPaint(const ModelContext& context, const CommandLine& line) {
    const PlayerModelManifest& manifest = context.loaded.manifest;
    if (manifest.assemblyPath.empty()) {
        std::cerr << "o modelo nao e montado (sem montagem.json)\n";
        return 1;
    }
    std::ifstream file(manifest.assemblyPath);
    std::stringstream text;
    text << file.rdbuf();
    std::string error;
    const std::optional<AssemblyRecipe> recipe = parseAssemblyRecipe(text.str(), &error);
    if (!recipe) {
        std::cerr << "receita: " << error << '\n';
        return 1;
    }
    const InspectionModel& model = context.model();
    const Mesh& mesh = model.mesh;
    const std::map<std::string, PaintSpace> spaces = playerPaintSpaces(mesh, *recipe, model.skeleton);
    PaintDesign design;
    if (!recipe->paintFile.empty()) {
        std::optional<PaintDesign> loaded = loadPaintDesign(manifest.assemblyPath.parent_path() / recipe->paintFile, spaces, &error);
        if (!loaded) {
            std::cerr << error << '\n';
            return 1;
        }
        design = std::move(*loaded);
    }
    const float step = std::max(0.5f, line.number("passo", 3.0f)) / 1000.0f;
    const float scale = std::max(200.0f, line.number("escala", 2000.0f));
    const int wantSide = line.text("lado", "direito") == "esquerdo" ? 1 : -1;
    const std::string only = line.text("espaco", "");
    const TextRenderer& text2 = TextRenderer::standard();
    int written = 0;
    for (const auto& [name, space] : spaces) {
        if (!only.empty() && name != only) continue;
        struct Dot {
            Point2 at;
            Color color;
        };
        std::vector<Dot> dots;
        Box2 bounds;
        // Unidades do espaco por metro da superficie (a orelha e em fracoes
        // da placa): o passo fica fino o bastante nele.
        float unitsPerMeter = 1.0f;
        const bool fractions = name == "orelha";
        const float sampleStep = fractions ? std::min(step, 0.0005f) : step;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
            const Vec3 a = mesh.positions[mesh.triangles[t][0]], b = mesh.positions[mesh.triangles[t][1]], c = mesh.positions[mesh.triangles[t][2]];
            const float edge = std::max({ (b - a).length(), (c - b).length(), (a - c).length() });
            const int n = std::max(1, static_cast<int>(std::ceil(edge / sampleStep)));
            SurfaceSample sample;
            sample.normal = mesh.normal(t);
            sample.part = mesh.partOf(t);
            sample.material = t < mesh.triangleMaterial.size() ? mesh.triangleMaterial[t] : -1;
            sample.triangle = t;
            sample.footprint = 0.0005f;
            for (int i = 0; i <= n; ++i)
                for (int j = 0; i + j <= n; ++j) {
                    const float u = static_cast<float>(i) / n, v = static_cast<float>(j) / n;
                    sample.position = a + (b - a) * u + (c - a) * v;
                    const std::optional<SpacePoint> at = space.locate(sample);
                    if (!at || at->side != wantSide) continue;
                    unitsPerMeter = at->footprint / sample.footprint;
                    dots.push_back({ at->point, paintedAt(sample, mesh, design, spaces) });
                    bounds.include(at->point);
                }
        }
        if (dots.empty()) continue;
        // A imagem: a coordenada x para a direita e y para cima, "scale" px
        // por unidade (o espaco da orelha e em fracoes: a placa com 600 px).
        const float pixels = fractions ? 600.0f : scale;
        const float margin = 60.0f;
        const int width = static_cast<int>((bounds.maximum.x - bounds.minimum.x) * pixels + 2.0f * margin) + 1;
        const int height = static_cast<int>((bounds.maximum.y - bounds.minimum.y) * pixels + 2.0f * margin) + 1;
        if (width > 6000 || height > 6000) {
            std::cerr << name << ": grande demais (" << width << "x" << height << "), use --escala menor\n";
            continue;
        }
        Image image(width, height, rgb(22, 26, 32));
        const auto toPixel = [&](Point2 p) {
            return Point2 { margin + (p.x - bounds.minimum.x) * pixels, static_cast<float>(height) - margin - (p.y - bounds.minimum.y) * pixels };
        };
        // A grade (5 cm; na orelha, decimos) e as ancoras.
        const float grid = fractions ? 0.1f : 0.05f;
        for (float x = std::ceil(bounds.minimum.x / grid) * grid; x <= bounds.maximum.x; x += grid) {
            const int px = static_cast<int>(toPixel({ x, 0.0f }).x);
            for (int y = 0; y < height; ++y) image.setPixel(px, y, std::abs(x) < 1e-4f ? rgb(120, 140, 170) : rgb(44, 50, 60));
        }
        for (float y = std::ceil(bounds.minimum.y / grid) * grid; y <= bounds.maximum.y; y += grid) {
            const int py = static_cast<int>(toPixel({ 0.0f, y }).y);
            for (int x = 0; x < width; ++x) image.setPixel(x, py, rgb(44, 50, 60));
        }
        const int radius = std::max(1, static_cast<int>(std::ceil(sampleStep * unitsPerMeter * pixels * 0.75f)));
        for (const Dot& dot : dots) {
            const Point2 p = toPixel(dot.at);
            for (int dy = -radius; dy <= radius; ++dy)
                for (int dx = -radius; dx <= radius; ++dx)
                    if (dx * dx + dy * dy <= radius * radius) image.setPixel(static_cast<int>(p.x) + dx, static_cast<int>(p.y) + dy, dot.color);
        }
        // (Ancoras quase na mesma altura: os nomes lado a lado.)
        std::vector<std::pair<int, float>> labels;
        for (const auto& [anchor, value] : space.anchors) {
            if (value < bounds.minimum.y - 0.2f || value > bounds.maximum.y + 0.2f) continue;
            const int py = static_cast<int>(toPixel({ 0.0f, value }).y);
            for (int x = 0; x < width; x += 2) image.setPixel(x, py, rgb(255, 196, 40));
            float left = 6.0f;
            for (const auto& [row, right] : labels)
                if (std::abs(row - py) < 20) left = std::max(left, right + 10.0f);
            char label[96];
            std::snprintf(label, sizeof label, "%s = %.3f", anchor.c_str(), value);
            text2.drawLabel(image, left, static_cast<float>(py) - 18.0f, label, 15.0f, rgb(255, 210, 90), rgb(8, 12, 18, 0.8f), 3.0f);
            labels.push_back({ py, left + text2.measure(label, 15.0f).width + 6.0f });
        }
        char title[200];
        std::snprintf(title, sizeof title, "espaco \"%s\" (lado %s): x %.3f a %.3f, y %.3f a %.3f%s", name.c_str(), wantSide < 0 ? "direito" : "esquerdo",
            bounds.minimum.x, bounds.maximum.x, bounds.minimum.y, bounds.maximum.y, fractions ? " (fracoes da placa)" : " (m; grade de 5 cm)");
        text2.drawLabel(image, 6.0f, 6.0f, title, 17.0f, Palette::Text, Palette::LabelBackground, 4.0f);
        if (!saveImage(image, context.output(line, "pintura_" + name + ".png"))) return 1;
        ++written;
    }
    if (written == 0) {
        std::cerr << "nenhum espaco desenhado" << (only.empty() ? "" : " (" + only + ")") << '\n';
        return 1;
    }
    return 0;
}

} // namespace MatterEngine::ModelInspector
