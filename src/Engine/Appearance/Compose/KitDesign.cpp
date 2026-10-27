#include "Engine/Appearance/Compose/KitDesign.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace MatterEngine::Appearance {

// O .ttf (TrueTypeFont.cpp: o stb_truetype estatico da aparencia).
std::optional<GlyphFont> loadTrueTypeFont(const std::filesystem::path& path, std::string* error);

namespace {

bool fail(std::string* error, const std::string& message) {
    if (error != nullptr) *error = message;
    return false;
}

std::optional<nlohmann::json> readJsonFile(const std::filesystem::path& path, std::string* error) {
    std::ifstream file(path);
    if (!file) {
        fail(error, "nao abre " + path.string());
        return std::nullopt;
    }
    try {
        return nlohmann::json::parse(file);
    } catch (const std::exception& exception) {
        fail(error, path.string() + ": " + exception.what());
        return std::nullopt;
    }
}

std::u32string utf32(const std::string& text) {
    std::u32string out;
    for (std::size_t i = 0; i < text.size();) {
        const unsigned char c = static_cast<unsigned char>(text[i]);
        const int extra = c < 0x80 ? 0 : c < 0xE0 ? 1 : c < 0xF0 ? 2 : 3;
        char32_t code = extra == 0 ? c : extra == 1 ? (c & 0x1F) : extra == 2 ? (c & 0x0F) : (c & 0x07);
        for (int k = 1; k <= extra && i + static_cast<std::size_t>(k) < text.size(); ++k)
            code = (code << 6) | (static_cast<unsigned char>(text[i + static_cast<std::size_t>(k)]) & 0x3F);
        out.push_back(code);
        i += static_cast<std::size_t>(extra) + 1;
    }
    return out;
}

Vec2 vec2(const nlohmann::json& value, Vec2 fallback) {
    if (!value.is_array() || value.size() != 2) return fallback;
    return { value[0].get<float>(), value[1].get<float>() };
}

} // namespace

std::optional<Rgba> parseColor(const std::string& text) {
    if ((text.size() != 7 && text.size() != 9) || text[0] != '#') return std::nullopt;
    Rgba out { 0.0f, 0.0f, 0.0f, 1.0f };
    for (std::size_t c = 0; c < (text.size() - 1) / 2; ++c) {
        int value = 0;
        for (std::size_t k = 0; k < 2; ++k) {
            const char h = text[1 + 2 * c + k];
            const int digit = h >= '0' && h <= '9' ? h - '0' : h >= 'a' && h <= 'f' ? h - 'a' + 10
                : h >= 'A' && h <= 'F' ? h - 'A' + 10 : -1;
            if (digit < 0) return std::nullopt;
            value = value * 16 + digit;
        }
        out[c] = static_cast<float>(value) / 255.0f;
    }
    return out;
}

std::optional<GlyphFont> parseGlyphFont(const nlohmann::json& document, std::string* error) {
    if (document.value("schema", "") != "matter.glyphs.v1") {
        fail(error, "fonte: schema nao e matter.glyphs.v1");
        return std::nullopt;
    }
    GlyphFont font;
    font.id = document.value("id", "");
    if (font.id.empty() || !document.contains("glyphs") || !document["glyphs"].is_object()) {
        fail(error, "fonte sem id ou sem glifos");
        return std::nullopt;
    }
    for (const auto& [key, value] : document["glyphs"].items()) {
        const std::u32string code = utf32(key);
        if (code.size() != 1) {
            fail(error, "fonte " + font.id + ": glifo de mais de um caractere: " + key);
            return std::nullopt;
        }
        GlyphFont::Glyph glyph;
        glyph.advance = value.value("advance", 0.0f);
        std::string message;
        std::optional<VectorShape> shape = parsePathData(value.value("d", ""), false, &message);
        if (!shape || glyph.advance <= 0.0f) {
            fail(error, "fonte " + font.id + ", glifo " + key + ": " + (shape ? "sem largura" : message));
            return std::nullopt;
        }
        glyph.shape = std::move(*shape);
        font.glyphs.emplace(code[0], std::move(glyph));
    }
    return font;
}

std::optional<GlyphFont> loadGlyphFont(const std::filesystem::path& path, std::string* error) {
    if (path.extension() == ".ttf" || path.extension() == ".otf") return loadTrueTypeFont(path, error);
    const std::optional<nlohmann::json> document = readJsonFile(path, error);
    if (!document) return std::nullopt;
    return parseGlyphFont(*document, error);
}

std::optional<VectorShape> layoutText(const GlyphFont& font, const std::u32string& text, float height, float spacing,
    float stretch, int align, std::string* error) {
    VectorShape out;
    float pen = 0.0f;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const auto found = font.glyphs.find(text[i]);
        if (found == font.glyphs.end()) {
            if (text[i] == U' ') {
                pen += 0.3f * stretch + spacing;
                continue;
            }
            fail(error, "fonte " + font.id + " sem o glifo " + std::to_string(static_cast<std::uint32_t>(text[i])));
            return std::nullopt;
        }
        out.append(found->second.shape.transformed({ stretch, 0.0f }, { 0.0f, 1.0f }, { pen, 0.0f }));
        pen += found->second.advance * stretch + (i + 1 < text.size() ? spacing : 0.0f);
    }
    // (A caixa: de 0 a pen na largura e de 0 a 1 na altura; o centro, a esquerda
    // ou a direita dela na origem, e a altura no meio.)
    const float shift = align < 0 ? 0.0f : align > 0 ? -pen : -0.5f * pen;
    out = out.transformed({ height, 0.0f }, { 0.0f, height }, { shift * height, -0.5f * height });
    return out;
}

int KitDesign::slotIndex(std::string_view slot) const {
    for (std::size_t i = 0; i < slotIds.size(); ++i)
        if (slotIds[i] == slot) return static_cast<int>(i);
    return -1;
}

std::optional<KitDesign> parseKitDesign(const nlohmann::json& document, const SurfaceTemplate& surface,
    const FontResolver& fonts, std::string* error) {
    KitDesign design;
    const auto bad = [&](const std::string& message) -> std::optional<KitDesign> {
        fail(error, "desenho " + design.id + ": " + message);
        return std::nullopt;
    };
    try {
        if (document.value("schema", "") != "matter.design.v1") return bad("schema nao e matter.design.v1");
        design.id = document.value("id", "");
        design.name = document.value("name", design.id);
        if (design.id.empty()) return bad("sem id");
        bool compatible = false;
        for (const nlohmann::json& entry : document.value("templates", nlohmann::json::array()))
            compatible |= entry.value("parametrization", -1) == surface.parametrization
                && entry.value("storage", -1) == surface.storage;
        if (!compatible)
            return bad("nao e compativel com o template (parametrizacao " + std::to_string(surface.parametrization)
                + ", armazenamento " + std::to_string(surface.storage) + ")");
        for (const nlohmann::json& slot : document.value("slots", nlohmann::json::array())) {
            const std::string id = slot.value("id", "");
            const std::optional<Rgba> color = parseColor(slot.value("default", ""));
            if (id.empty() || !color || design.slotIndex(id) >= 0) return bad("vaga sem id, repetida ou sem cor padrao: " + id);
            design.slotIds.push_back(id);
            design.slotNames.push_back(slot.value("name", id));
            design.slotDefaults.push_back(*color);
        }
        const auto slotOf = [&](const nlohmann::json& value, int& out) {
            out = value.is_string() ? design.slotIndex(value.get<std::string>()) : -1;
            return out >= 0;
        };
        const std::size_t panelCount = surface.panels.size(), regionCount = surface.regions.size();
        for (const nlohmann::json& entry : document.value("layers", nlohmann::json::array())) {
            DesignLayer layer;
            layer.id = entry.value("id", "");
            const std::string where = "camada " + layer.id + ": ";
            layer.regions.assign(regionCount, 0);
            layer.panels.assign(panelCount, 0);
            bool any = false;
            const nlohmann::json target = entry.value("target", nlohmann::json::object());
            for (const nlohmann::json& name : target.value("regions", nlohmann::json::array())) {
                const int index = surface.regionIndex(name.get<std::string>());
                if (index < 0) return bad(where + "regiao desconhecida " + name.get<std::string>());
                layer.regions[static_cast<std::size_t>(index)] = 1;
                any = true;
            }
            for (const nlohmann::json& name : target.value("panels", nlohmann::json::array())) {
                const int index = surface.panelIndex(name.get<std::string>());
                if (index < 0) return bad(where + "painel desconhecido " + name.get<std::string>());
                layer.panels[static_cast<std::size_t>(index)] = 1;
                any = true;
            }
            if (!any) return bad(where + "sem alvo");
            const std::string space = entry.value("space", "panel");
            if (space == "panel") layer.space = LayerSpace::Panel;
            else if (space == "reference") layer.space = LayerSpace::Reference;
            else if (space.rfind("chart:", 0) == 0) {
                layer.space = LayerSpace::Chart;
                layer.chart = surface.chartIndex(space.substr(6));
                if (layer.chart < 0) return bad(where + "carta desconhecida " + space.substr(6));
            } else return bad(where + "espaco desconhecido " + space);
            if (entry.contains("anchor")) {
                const nlohmann::json& anchor = entry["anchor"];
                if (anchor.contains("anchor")) {
                    const std::string id = anchor["anchor"].get<std::string>();
                    const auto found = std::find_if(surface.anchors.begin(), surface.anchors.end(),
                        [&](const SurfaceAnchor& a) { return a.id == id; });
                    if (found == surface.anchors.end()) return bad(where + "ancora desconhecida " + id);
                    layer.anchorPanel = found->panel;
                    layer.origin = found->position;
                } else if (anchor.contains("zone")) {
                    const std::string id = anchor["zone"].get<std::string>();
                    const auto found = std::find_if(surface.zones.begin(), surface.zones.end(),
                        [&](const SurfaceZone& z) { return z.id == id; });
                    if (found == surface.zones.end()) return bad(where + "zona desconhecida " + id);
                    layer.anchorPanel = found->panel;
                    layer.origin = (found->min + found->max) * 0.5f;
                } else return bad(where + "ancoragem sem ancora nem zona");
                if (layer.space != LayerSpace::Panel) return bad(where + "ancoragem so no espaco do painel");
            }
            if (entry.contains("transform")) {
                const nlohmann::json& transform = entry["transform"];
                layer.offset = vec2(transform.value("offset", nlohmann::json()), layer.offset);
                layer.scale = vec2(transform.value("scale", nlohmann::json()), layer.scale);
                layer.rotateDegrees = transform.value("rotate", 0.0f);
                if (layer.scale.x == 0.0f || layer.scale.y == 0.0f) return bad(where + "escala zero");
            }
            const nlohmann::json content = entry.value("content", nlohmann::json::object());
            const std::string type = content.value("type", "");
            if (type == "fill") layer.content = LayerContent::Fill;
            else if (type == "bands") {
                layer.content = LayerContent::Bands;
                layer.axis = content.value("axis", "y") == "x" ? 0 : 1;
                const std::string from = content.value("from", "base");
                layer.fromTop = from == "topo";
                if (from != "base" && from != "topo") return bad(where + "faixas com from desconhecido " + from);
                if (layer.fromTop && layer.space != LayerSpace::Panel) return bad(where + "from topo so no painel");
                for (const nlohmann::json& band : content.value("bands", nlohmann::json::array())) {
                    if (!band.is_array() || band.size() != 2 || band[0].get<float>() >= band[1].get<float>())
                        return bad(where + "faixa invalida");
                    layer.bands.push_back({ band[0].get<float>(), band[1].get<float>() });
                }
                if (layer.bands.empty()) return bad(where + "sem faixas");
                // A uniao das faixas (as que se tocam ou sobrepoem viram uma): a
                // cobertura analitica soma as faixas, e a soma de duas sobrepostas
                // pintaria o texel duas vezes.
                std::sort(layer.bands.begin(), layer.bands.end());
                std::size_t merged = 0;
                for (std::size_t i = 1; i < layer.bands.size(); ++i) {
                    if (layer.bands[i][0] <= layer.bands[merged][1]) layer.bands[merged][1] = std::max(layer.bands[merged][1], layer.bands[i][1]);
                    else layer.bands[++merged] = layer.bands[i];
                }
                layer.bands.resize(merged + 1);
            } else if (type == "stripes") {
                layer.content = LayerContent::Stripes;
                layer.period = content.value("period", 0.0f);
                layer.width = content.value("width", 0.0f);
                layer.angleDegrees = content.value("angle", 0.0f);
                layer.phase = content.value("phase", 0.0f);
                if (layer.period <= 0.0f || layer.width <= 0.0f || layer.width >= layer.period)
                    return bad(where + "listras com periodo ou largura invalidos");
            } else if (type == "gradient") {
                layer.content = LayerContent::Gradient;
                layer.from = vec2(content.value("from", nlohmann::json()), layer.from);
                layer.to = vec2(content.value("to", nlohmann::json()), layer.to);
                const nlohmann::json colors = content.value("colors", nlohmann::json::array());
                if (colors.size() != 2 || !slotOf(colors[0], layer.gradientSlots[0]) || !slotOf(colors[1], layer.gradientSlots[1]))
                    return bad(where + "degrade sem as duas vagas");
                if ((layer.to - layer.from).lengthSquared() <= 0.0f) return bad(where + "degrade sem comprimento");
            } else if (type == "border") {
                layer.content = LayerContent::Border;
                const std::string seam = content.value("seam", "");
                for (std::size_t s = 0; s < surface.seams.size(); ++s)
                    if (surface.seams[s].id == seam) layer.seam = static_cast<int>(s);
                layer.borderWidth = content.value("width", 0.0f);
                if (layer.seam < 0) return bad(where + "borda de costura desconhecida " + seam);
                if (layer.borderWidth <= 0.0f) return bad(where + "borda sem largura");
                if (layer.space != LayerSpace::Panel || entry.contains("mask")) return bad(where + "borda so no painel e sem mascara");
            } else if (type == "path") {
                layer.content = LayerContent::Path;
                std::string message;
                std::optional<VectorShape> shape = parsePathData(content.value("d", ""), content.value("rule", "nonzero") == "evenodd", &message);
                if (!shape) return bad(where + message);
                layer.shape = std::move(*shape);
            } else if (type == "text") {
                layer.content = LayerContent::Text;
                const std::shared_ptr<const GlyphFont> font = fonts ? fonts(content.value("font", "")) : nullptr;
                if (!font) return bad(where + "fonte desconhecida " + content.value("font", ""));
                const std::string align = content.value("align", "center");
                std::string message;
                std::optional<VectorShape> shape = layoutText(*font, utf32(content.value("text", "")), content.value("height", 0.0f),
                    content.value("spacing", 0.0f), content.value("stretch", 1.0f), align == "left" ? -1 : align == "right" ? 1 : 0, &message);
                if (!shape || shape->empty()) return bad(where + (shape ? "texto vazio" : message));
                layer.shape = std::move(*shape);
            } else return bad(where + "conteudo desconhecido " + type);
            if (entry.contains("mask")) {
                const nlohmann::json& mask = entry["mask"];
                std::string message;
                std::optional<VectorShape> shape = parsePathData(mask.value("d", ""), mask.value("rule", "nonzero") == "evenodd", &message);
                if (!shape) return bad(where + "mascara: " + message);
                layer.mask = std::move(*shape);
            }
            if (layer.content != LayerContent::Gradient && !slotOf(entry.value("color", nlohmann::json()), layer.slot))
                return bad(where + "vaga desconhecida " + entry.value("color", nlohmann::json()).dump());
            layer.opacity = entry.value("opacity", 1.0f);
            if (layer.opacity < 0.0f || layer.opacity > 1.0f) return bad(where + "opacidade fora de 0..1");
            design.layers.push_back(std::move(layer));
        }
        if (design.layers.empty()) return bad("sem camadas");
        if (document.contains("identity")) {
            const nlohmann::json& identity = document["identity"];
            design.identityFont = fonts ? fonts(identity.value("font", "")) : nullptr;
            if (!design.identityFont) return bad("identificacao com fonte desconhecida " + identity.value("font", ""));
            if (!slotOf(identity.value("color", nlohmann::json()), design.identitySlot)) return bad("identificacao com vaga desconhecida");
            const nlohmann::json zones = identity.value("zones", nlohmann::json::object());
            for (const auto& [zoneId, style] : zones.items()) {
                IdentityStyle out;
                for (std::size_t z = 0; z < surface.zones.size(); ++z)
                    if (surface.zones[z].id == zoneId) out.zone = static_cast<int>(z);
                if (out.zone < 0) return bad("identificacao em zona desconhecida " + zoneId);
                out.height = style.value("height", 0.0f);
                out.offset = vec2(style.value("offset", nlohmann::json()), out.offset);
                out.spacing = style.value("spacing", 0.0f);
                out.stretch = style.value("stretch", 1.0f);
                out.rotateDegrees = style.value("rotate", 0.0f);
                if (out.height <= 0.0f || out.stretch <= 0.0f) return bad("identificacao " + zoneId + " sem altura ou largura");
                design.identity.push_back(out);
            }
        }
    } catch (const std::exception& exception) {
        return bad(std::string("documento invalido: ") + exception.what());
    }
    return design;
}

std::optional<Kit> parseKit(const nlohmann::json& document, const std::function<const KitDesign*(const std::string&)>& designOf,
    std::string* error) {
    Kit kit;
    try {
        if (document.value("schema", "") != "matter.kit.v1") {
            fail(error, "uniforme: schema nao e matter.kit.v1");
            return std::nullopt;
        }
        kit.id = document.value("id", "");
        kit.design = document.value("design", "");
        const KitDesign* design = designOf ? designOf(kit.design) : nullptr;
        if (kit.id.empty() || design == nullptr) {
            fail(error, "uniforme " + kit.id + ": sem id ou desenho desconhecido " + kit.design);
            return std::nullopt;
        }
        kit.colors = design->slotDefaults;
        const nlohmann::json colors = document.value("colors", nlohmann::json::object());
        for (const auto& [slot, value] : colors.items()) {
            const int index = design->slotIndex(slot);
            const std::optional<Rgba> color = value.is_string() ? parseColor(value.get<std::string>()) : std::nullopt;
            if (index < 0 || !color) {
                fail(error, "uniforme " + kit.id + ": vaga desconhecida ou cor invalida " + slot);
                return std::nullopt;
            }
            kit.colors[static_cast<std::size_t>(index)] = *color;
        }
        kit.number = utf32(document.value("identity", nlohmann::json::object()).value("number", ""));
    } catch (const std::exception& exception) {
        fail(error, std::string("uniforme invalido: ") + exception.what());
        return std::nullopt;
    }
    return kit;
}

std::optional<LoadedKit> loadKit(const std::filesystem::path& directory, const std::string& kitId, const SurfaceTemplate& surface,
    std::string* error) {
    const std::optional<nlohmann::json> kitDocument = readJsonFile(directory / "kits" / (kitId + ".json"), error);
    if (!kitDocument) return std::nullopt;
    std::map<std::string, std::shared_ptr<const GlyphFont>> loaded;
    std::string fontError;
    const FontResolver fonts = [&](const std::string& id) -> std::shared_ptr<const GlyphFont> {
        if (const auto found = loaded.find(id); found != loaded.end()) return found->second;
        for (const char* extension : { ".json", ".ttf", ".otf" }) {
            const std::filesystem::path path = directory / "fonts" / (id + extension);
            if (!std::filesystem::exists(path)) continue;
            std::optional<GlyphFont> font = loadGlyphFont(path, &fontError);
            if (!font) return nullptr;
            return loaded[id] = std::make_shared<const GlyphFont>(std::move(*font));
        }
        fontError = "fonte " + id + " nao encontrada";
        return nullptr;
    };
    const std::string designId = kitDocument->value("design", "");
    const std::optional<nlohmann::json> designDocument = readJsonFile(directory / "designs" / (designId + ".json"), error);
    if (!designDocument) return std::nullopt;
    std::optional<KitDesign> design = parseKitDesign(*designDocument, surface, fonts, error);
    if (!design) {
        if (error != nullptr && !fontError.empty()) *error += " (" + fontError + ")";
        return std::nullopt;
    }
    LoadedKit out;
    out.design = std::make_shared<const KitDesign>(std::move(*design));
    std::optional<Kit> kit = parseKit(*kitDocument, [&](const std::string& id) { return id == out.design->id ? out.design.get() : nullptr; },
        error);
    if (!kit) return std::nullopt;
    out.kit = std::move(*kit);
    return out;
}

} // namespace MatterEngine::Appearance
