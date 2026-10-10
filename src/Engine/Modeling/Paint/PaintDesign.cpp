#include "Engine/Modeling/Paint/PaintDesign.hpp"

#include "Engine/Modeling/Paint/PaintFont.hpp"
#include "Engine/Modeling/Paint/SvgPath.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace MatterEngine::Modeling {

namespace {

using Json = nlohmann::json;

constexpr float DegreesToRad = 0.017453292f;

struct Context {
    const std::map<std::string, Color>* colors = nullptr;
    const std::map<std::string, float>* anchors = nullptr;
    std::filesystem::path directory;
    std::map<std::string, std::shared_ptr<PaintFont>>* fonts = nullptr;
    std::map<std::string, std::shared_ptr<const PaintImage>>* images = nullptr;
    std::vector<std::filesystem::path>* files = nullptr;
};

[[noreturn]] void fail(const std::string& message) { throw std::runtime_error(message); }

// Um numero ou "ancora", "ancora+0.02", "ancora-0.02".
float coordinate(const Json& value, const Context& context) {
    if (value.is_number()) return value.get<float>();
    if (!value.is_string()) fail("coordenada: esperado numero ou ancora");
    const std::string text = value.get<std::string>();
    std::size_t split = text.find_first_of("+-", 1);
    const std::string name = text.substr(0, split);
    if (context.anchors == nullptr || !context.anchors->count(name)) fail("ancora desconhecida: " + name);
    float offset = 0.0f;
    if (split != std::string::npos) {
        try {
            offset = std::stof(text.substr(split));
        } catch (...) {
            fail("deslocamento invalido: " + text);
        }
    }
    return context.anchors->at(name) + offset;
}

Point2 point(const Json& value, const Context& context) {
    if (!value.is_array() || value.size() != 2) fail("ponto: esperado [x, y]");
    return { coordinate(value[0], context), coordinate(value[1], context) };
}

Point2 radius(const Json& value, const Context& context) {
    if (value.is_number()) return { value.get<float>(), value.get<float>() };
    return point(value, context);
}

Color color(const Json& value, const Context& context) {
    if (value.is_string()) {
        const std::string text = value.get<std::string>();
        if (!text.empty() && text[0] == '#' && text.size() == 7) {
            const auto hex = [&](std::size_t at) { return std::stoi(text.substr(at, 2), nullptr, 16); };
            return rgb(hex(1), hex(3), hex(5));
        }
        if (context.colors != nullptr && context.colors->count(text)) return context.colors->at(text);
        fail("cor desconhecida: " + text);
    }
    if (value.is_array() && (value.size() == 3 || value.size() == 4))
        return rgb(value[0].get<int>(), value[1].get<int>(), value[2].get<int>(), value.size() == 4 ? value[3].get<float>() : 1.0f);
    fail("cor: esperado [r, g, b], \"#rrggbb\" ou um nome");
}

const PaintFont& font(const Json& shape, const Context& context) {
    const std::string name = shape.value("font", std::string("block"));
    if (name == "block") return PaintFont::block();
    auto& cache = *context.fonts;
    if (!cache.count(name)) {
        std::string error;
        std::optional<PaintFont> loaded = PaintFont::loadTrueType(context.directory / name, &error);
        if (!loaded) fail(error);
        context.files->push_back(context.directory / name);
        cache[name] = std::make_shared<PaintFont>(std::move(*loaded));
    }
    return *cache.at(name);
}

Shape2D shape(const Json& value, const Context& context) {
    const std::string type = value.value("type", std::string());
    Shape2D out;
    if (type == "polygon") {
        std::vector<Point2> points;
        for (const Json& p : value.at("points")) points.push_back(point(p, context));
        if (points.size() < 3) fail("polygon: tres pontos ou mais");
        out = polygonShape(std::move(points));
    } else if (type == "rectangle") {
        out = rectangleShape(point(value.at("from"), context), point(value.at("to"), context), value.value("chamfer", 0.0f),
            value.value("round", 0.0f));
    } else if (type == "ellipse") {
        out = ellipseShape(point(value.at("center"), context), radius(value.at("radius"), context), value.value("segments", 64));
    } else if (type == "line") {
        std::vector<Point2> points;
        for (const Json& p : value.at("points")) points.push_back(point(p, context));
        out = strokeShape(points, value.at("width").get<float>(), value.value("closed", false));
    } else if (type == "text") {
        const PaintFont& f = font(value, context);
        float width = 0.0f;
        const Shape2D line = f.layout(value.at("text").get<std::string>(), value.value("spacing", 0.0f), &width);
        const float height = value.at("height").get<float>();
        const float stretch = value.value("stretch", 1.0f);
        const float slant = std::tan(value.value("slant", 0.0f) * DegreesToRad);
        const std::string align = value.value("align", std::string("center"));
        const std::string vertical = value.value("vertical", std::string("middle"));
        const Point2 at = point(value.at("at"), context);
        const float startX = align == "left" ? 0.0f : align == "right" ? -width : -0.5f * width;
        const float startY = vertical == "baseline" ? 0.0f : -0.5f;
        // Na altura 1: o comeco, a inclinacao (em volta da linha do meio) e a escala.
        Affine2 place = Affine2::translation(startX, startY);
        Affine2 shear;
        shear.b = slant;
        place = place.then(shear).then(Affine2::scaling(height * stretch, height)).then(Affine2::translation(at.x, at.y));
        out = line.transformed(place);
    } else if (type == "path") {
        // O caminho do SVG ("d", ou o <path> de um arquivo pelo id; sem id,
        // todos): o quadro (o viewBox, ou a caixa do caminho) centrado em
        // "at" com a altura "height" - as camadas de um escudo de varias cores
        // com o mesmo quadro ficam alinhadas. O y do SVG desce.
        Shape2D drawing;
        std::optional<std::array<float, 4>> frame;
        std::string error;
        if (value.contains("file")) {
            const std::filesystem::path svg = context.directory / value.at("file").get<std::string>();
            const std::optional<SvgDocument> document = readSvgDocument(svg, &error);
            if (!document) fail(error);
            context.files->push_back(svg);
            frame = document->viewBox;
            const std::string id = value.value("id", std::string());
            for (const auto& [pathId, d] : document->paths) {
                if (!id.empty() && pathId != id) continue;
                const std::optional<Shape2D> parsed = parseSvgPath(d, &error);
                if (!parsed) fail(error + (pathId.empty() ? "" : " (" + pathId + ")"));
                drawing.add(*parsed);
            }
            if (drawing.empty()) fail("svg sem o caminho " + id);
        } else {
            const std::optional<Shape2D> parsed = parseSvgPath(value.at("d").get<std::string>(), &error);
            if (!parsed) fail(error);
            drawing = *parsed;
        }
        if (value.contains("viewBox")) {
            const Json& box = value.at("viewBox");
            if (!box.is_array() || box.size() != 4) fail("viewBox: esperado [x, y, largura, altura]");
            frame = std::array<float, 4> { box[0].get<float>(), box[1].get<float>(), box[2].get<float>(), box[3].get<float>() };
        }
        const Box2 bounds = drawing.bounds();
        const std::array<float, 4> f = frame.value_or(std::array<float, 4> { bounds.minimum.x, bounds.minimum.y, bounds.maximum.x - bounds.minimum.x,
            bounds.maximum.y - bounds.minimum.y });
        if (f[3] <= 0.0f) fail("path: quadro sem altura");
        const float scale = value.at("height").get<float>() / f[3];
        const Point2 at = point(value.at("at"), context);
        out = drawing.transformed(Affine2::translation(-(f[0] + 0.5f * f[2]), -(f[1] + 0.5f * f[3]))
                                      .then(Affine2::scaling(scale * value.value("stretch", 1.0f), -scale))
                                      .then(Affine2::translation(at.x, at.y)));
    } else {
        fail("forma desconhecida: " + type);
    }
    if (value.contains("rotate")) {
        const Box2 box = out.bounds();
        const Point2 pivot = value.contains("pivot") ? point(value.at("pivot"), context)
                                                     : Point2 { 0.5f * (box.minimum.x + box.maximum.x), 0.5f * (box.minimum.y + box.maximum.y) };
        out = out.transformed(Affine2::translation(-pivot.x, -pivot.y)
                                  .then(Affine2::rotation(value.at("rotate").get<float>() * DegreesToRad))
                                  .then(Affine2::translation(pivot.x, pivot.y)));
    }
    return out;
}

Fill fill(const Json& layer, const Context& context) {
    Fill out;
    if (layer.contains("color")) out.color = color(layer.at("color"), context);
    if (layer.contains("image")) {
        // A imagem: o arquivo (PNG com transparencia) na caixa "from" a "to",
        // ou centrada em "at" com a altura "height" (a largura pela proporcao
        // dela, vezes "stretch"); "rotate" em graus.
        if (layer.contains("gradient")) fail("image ou gradient, nao os dois");
        const Json& image = layer.at("image");
        const std::string file = image.at("file").get<std::string>();
        auto& cache = *context.images;
        if (!cache.count(file)) {
            std::string error;
            const std::optional<Image> loaded = readImage((context.directory / file).string(), &error);
            if (!loaded || loaded->empty()) fail("imagem: " + error);
            cache[file] = std::make_shared<const PaintImage>(*loaded);
            context.files->push_back(context.directory / file);
        }
        out.kind = Fill::Kind::Image;
        out.image = cache.at(file);
        if (image.contains("from")) {
            const Point2 a = point(image.at("from"), context), b = point(image.at("to"), context);
            out.from = { std::min(a.x, b.x), std::min(a.y, b.y) };
            out.to = { std::max(a.x, b.x), std::max(a.y, b.y) };
        } else {
            const Point2 at = point(image.at("at"), context);
            const float height = image.at("height").get<float>();
            const float width = height * static_cast<float>(out.image->width()) / static_cast<float>(std::max(1, out.image->height()))
                * image.value("stretch", 1.0f);
            out.from = { at.x - 0.5f * width, at.y - 0.5f * height };
            out.to = { at.x + 0.5f * width, at.y + 0.5f * height };
        }
        out.angle = image.value("rotate", 0.0f) * DegreesToRad;
        return out;
    }
    if (!layer.contains("gradient")) return out;
    const Json& g = layer.at("gradient");
    const std::string type = g.value("type", std::string("linear"));
    if (type == "linear") {
        out.kind = Fill::Kind::Linear;
        out.from = point(g.at("from"), context);
        out.to = point(g.at("to"), context);
    } else if (type == "radial") {
        out.kind = Fill::Kind::Radial;
        out.center = point(g.at("center"), context);
        out.radius = radius(g.at("radius"), context);
        out.angle = g.value("angle", 0.0f) * DegreesToRad;
    } else {
        fail("degrade desconhecido: " + type);
    }
    for (const Json& stop : g.at("stops")) {
        if (!stop.is_array() || stop.size() != 2) fail("stops: esperado [posicao, cor]");
        out.stops.push_back({ stop[0].get<float>(), color(stop[1], context) });
    }
    if (out.stops.empty()) fail("degrade sem cores");
    for (std::size_t i = 1; i < out.stops.size(); ++i)
        if (out.stops[i].at < out.stops[i - 1].at) fail("degrade: as posicoes em ordem");
    return out;
}

} // namespace

std::optional<PaintDesign> parsePaintDesign(const std::string& text, const std::map<std::string, PaintSpace>& spaces,
    const std::filesystem::path& directory, std::string* error) {
    try {
        const Json document = Json::parse(text);
        PaintDesign design;
        std::map<std::string, Color> colors;
        std::map<std::string, std::shared_ptr<PaintFont>> fonts;
        std::map<std::string, std::shared_ptr<const PaintImage>> images;
        Context context;
        context.colors = &colors;
        context.directory = directory;
        context.fonts = &fonts;
        context.images = &images;
        context.files = &design.files;
        if (document.contains("colors"))
            for (const auto& [name, value] : document.at("colors").items()) colors[name] = color(value, context);
        if (document.contains("base"))
            for (const auto& [part, value] : document.at("base").items()) design.partBase[part] = color(value, context);
        if (document.contains("layers"))
            for (const Json& entry : document.at("layers")) {
                PaintLayer layer;
                layer.name = entry.value("name", std::string());
                layer.space = entry.at("space").get<std::string>();
                const auto space = spaces.find(layer.space);
                if (space == spaces.end()) fail("camada \"" + layer.name + "\": espaco desconhecido " + layer.space);
                context.anchors = &space->second.anchors;
                try {
                    layer.fill = fill(entry, context);
                    // Sem forma, a imagem e a forma (a caixa dela, girada com ela).
                    if (entry.contains("shape")) {
                        layer.shape = PreparedShape(shape(entry.at("shape"), context));
                    } else if (layer.fill.kind == Fill::Kind::Image) {
                        const Point2 middle { 0.5f * (layer.fill.from.x + layer.fill.to.x), 0.5f * (layer.fill.from.y + layer.fill.to.y) };
                        layer.shape = PreparedShape(rectangleShape(layer.fill.from, layer.fill.to)
                                                        .transformed(Affine2::translation(-middle.x, -middle.y)
                                                                         .then(Affine2::rotation(layer.fill.angle))
                                                                         .then(Affine2::translation(middle.x, middle.y))));
                    } else {
                        fail("sem \"shape\"");
                    }
                    layer.opacity = std::clamp(entry.value("opacity", 1.0f), 0.0f, 1.0f);
                    const std::string blend = entry.value("blend", std::string("normal"));
                    if (blend != "normal" && blend != "multiply") fail("blend: normal ou multiply");
                    layer.blend = blend == "multiply" ? Blend::Multiply : Blend::Normal;
                    if (entry.contains("outline")) {
                        layer.outline = entry.at("outline").at("width").get<float>();
                        layer.outlineColor = color(entry.at("outline").at("color"), context);
                    }
                    const std::string sides = entry.value("sides", std::string("both"));
                    if (sides != "both" && sides != "right" && sides != "left") fail("sides: both, right ou left");
                    layer.sides = sides == "right" ? Sides::Right : sides == "left" ? Sides::Left : Sides::Both;
                } catch (const std::exception& inner) {
                    fail("camada \"" + layer.name + "\": " + inner.what());
                }
                if (layer.shape.empty()) fail("camada \"" + layer.name + "\": forma vazia");
                design.layers.push_back(std::move(layer));
            }
        return design;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("pintura: ") + exception.what();
        return std::nullopt;
    }
}

std::optional<PaintDesign> loadPaintDesign(const std::filesystem::path& file, const std::map<std::string, PaintSpace>& spaces, std::string* error) {
    std::ifstream stream(file);
    if (!stream) {
        if (error != nullptr) *error = "pintura: nao abriu " + file.string();
        return std::nullopt;
    }
    std::stringstream text;
    text << stream.rdbuf();
    std::optional<PaintDesign> design = parsePaintDesign(text.str(), spaces, file.parent_path(), error);
    if (design) design->files.insert(design->files.begin(), file);
    return design;
}

} // namespace MatterEngine::Modeling
