#include "Engine/Appearance/Surface/SurfaceTemplate.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

namespace MatterEngine::Appearance {

namespace {

using Json = nlohmann::json;

// Uma falha da leitura (a mensagem diz o campo e o indice).
struct Invalid : std::runtime_error {
    using std::runtime_error::runtime_error;
};

[[noreturn]] void invalid(const std::string& message) {
    throw Invalid(message);
}

const Json& field(const Json& object, const char* key, const std::string& where) {
    if (!object.is_object() || !object.contains(key)) invalid(where + ": falta \"" + key + "\"");
    return object.at(key);
}

Vec2 vec2(const Json& value, const std::string& where) {
    if (!value.is_array() || value.size() != 2 || !value[0].is_number() || !value[1].is_number())
        invalid(where + ": esperava [x, y]");
    const Vec2 v { value[0].get<float>(), value[1].get<float>() };
    if (!std::isfinite(v.x) || !std::isfinite(v.y)) invalid(where + ": coordenada invalida");
    return v;
}

Vec3 vec3(const Json& value, const std::string& where) {
    if (!value.is_array() || value.size() != 3 || !value[0].is_number() || !value[1].is_number() || !value[2].is_number())
        invalid(where + ": esperava [x, y, z]");
    const Vec3 v { value[0].get<float>(), value[1].get<float>(), value[2].get<float>() };
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) invalid(where + ": posicao invalida");
    return v;
}

Corners corners(const Json& value, const std::string& where) {
    if (!value.is_array() || value.size() != 3) invalid(where + ": esperava tres cantos");
    return { vec2(value[0], where), vec2(value[1], where), vec2(value[2], where) };
}

std::vector<std::string> names(const Json& value, const std::string& where) {
    if (!value.is_array()) invalid(where + ": esperava uma lista de nomes");
    std::vector<std::string> out;
    std::set<std::string> seen;
    for (const Json& item : value) {
        if (!item.is_string() || item.get<std::string>().empty()) invalid(where + ": nome vazio ou invalido");
        out.push_back(item.get<std::string>());
        if (!seen.insert(out.back()).second) invalid(where + ": nome repetido " + out.back());
    }
    return out;
}

std::vector<float> numbers(const Json& value, std::size_t count, const std::string& where) {
    if (!value.is_array() || value.size() != count) invalid(where + ": esperava " + std::to_string(count) + " numeros");
    std::vector<float> out;
    for (const Json& item : value) {
        if (!item.is_number()) invalid(where + ": esperava numeros");
        out.push_back(item.get<float>());
        if (!std::isfinite(out.back())) invalid(where + ": numero invalido");
    }
    return out;
}

const Json& perTriangle(const Json& document, const char* key, std::size_t count) {
    const Json& value = field(document, key, "documento");
    if (!value.is_array() || value.size() != count)
        invalid(std::string(key) + ": " + std::to_string(value.is_array() ? value.size() : 0) + " entradas para "
            + std::to_string(count) + " triangulos");
    return value;
}

int indexOf(const std::vector<std::string>& list, std::string_view name) {
    for (std::size_t i = 0; i < list.size(); ++i)
        if (list[i] == name) return static_cast<int>(i);
    return -1;
}

std::uint64_t edgeKey(std::uint32_t a, std::uint32_t b) {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(a) << 32) | b;
}

const char* kindName(SeamKind kind) {
    return kind == SeamKind::Seam ? "costura" : kind == SeamKind::Opening ? "abertura" : "juncao";
}

Json pair(Vec2 v) {
    return Json::array({ v.x, v.y });
}

Json cornerJson(const Corners& c) {
    return Json::array({ pair(c[0]), pair(c[1]), pair(c[2]) });
}

} // namespace

int SurfaceTemplate::panelIndex(std::string_view name) const {
    return indexOf(panels, name);
}

int SurfaceTemplate::regionIndex(std::string_view name) const {
    return indexOf(regions, name);
}

int SurfaceTemplate::chartIndex(std::string_view id) const {
    for (std::size_t i = 0; i < charts.size(); ++i)
        if (charts[i].id == id) return static_cast<int>(i);
    return -1;
}

const SurfaceSeam* SurfaceTemplate::seam(std::string_view id) const {
    for (const SurfaceSeam& item : seams)
        if (item.id == id) return &item;
    return nullptr;
}

std::optional<SurfaceTemplate> parseSurfaceTemplate(const Json& document, std::span<const std::array<std::uint32_t, 3>> triangles,
    std::size_t vertexCount, std::string* error) {
    try {
        const Json& block = field(document, "surface", "documento");
        SurfaceTemplate s;
        s.schema = field(block, "schema", "surface").get<std::string>();
        if (s.schema != "matter.surface.v1") invalid("schema " + s.schema + " (esperava matter.surface.v1)");
        s.parametrization = field(block, "parametrization", "surface").get<int>();
        s.storage = field(block, "storage", "surface").get<int>();
        s.panels = names(field(block, "panels", "surface"), "panels");
        s.regions = names(field(block, "regions", "surface"), "regions");
        if (s.panels.size() > 32000 || s.regions.size() > 255) invalid("paineis ou regioes demais");
        const auto label = [&](const std::string& name) { return indexOf(s.panels, name) >= 0 || indexOf(s.regions, name) >= 0; };

        // As costuras primeiro: as cartas dizem quais unem.
        for (const Json& item : field(block, "seams", "surface")) {
            SurfaceSeam seam;
            seam.id = field(item, "id", "seams").get<std::string>();
            const std::string where = "costura " + seam.id;
            const std::string kind = field(item, "kind", where).get<std::string>();
            seam.kind = kind == "costura" ? SeamKind::Seam : kind == "abertura" ? SeamKind::Opening : kind == "juncao" ? SeamKind::Junction
                                                                                                                     : (invalid(where + ": tipo " + kind), SeamKind::Junction);
            seam.panels = field(item, "panels", where).get<std::vector<std::string>>();
            if (seam.panels.size() < 2) invalid(where + ": menos de dois lados");
            for (const auto& name : seam.panels)
                if (!label(name)) invalid(where + ": lado desconhecido " + name);
            if (seam.kind != SeamKind::Junction)
                for (const auto& name : seam.panels)
                    if (indexOf(s.panels, name) < 0) invalid(where + ": " + name + " nao e um painel");
            seam.continuous = field(item, "continuous", where).get<bool>();
            for (const Json& edge : field(item, "edges", where)) {
                const auto pairOf = edge.get<std::array<std::uint32_t, 2>>();
                if (pairOf[0] >= vertexCount || pairOf[1] >= vertexCount || pairOf[0] == pairOf[1]) invalid(where + ": aresta invalida");
                seam.edges.push_back(pairOf);
            }
            if (seam.edges.empty()) invalid(where + ": sem arestas");
            if (s.seam(seam.id) != nullptr) invalid(where + ": repetida");
            s.seams.push_back(std::move(seam));
        }
        for (const Json& item : field(block, "charts", "surface")) {
            SurfaceChart chart;
            chart.id = field(item, "id", "charts").get<std::string>();
            const std::string where = "carta " + chart.id;
            chart.panels = field(item, "panels", where).get<std::vector<std::string>>();
            for (const auto& name : chart.panels)
                if (indexOf(s.panels, name) < 0) invalid(where + ": painel desconhecido " + name);
            chart.joined = field(item, "joined", where).get<std::vector<std::string>>();
            for (const auto& name : chart.joined) {
                const SurfaceSeam* seam = s.seam(name);
                if (seam == nullptr || seam->kind != SeamKind::Seam) invalid(where + ": une " + name + ", que nao e uma costura");
            }
            const Json& cut = field(item, "cut", where);
            chart.cut = cut.is_null() ? std::string() : cut.get<std::string>();
            // (O metodo e o campo sao a proveniencia da coordenada: ditos, nunca supostos.)
            chart.method = field(item, "method", where).get<std::string>();
            if (chart.method != "campo" && chart.method != "isometrica") invalid(where + ": metodo " + chart.method);
            chart.field = item.value("field", std::string());
            if (chart.method == "campo" && chart.field != "tronco" && chart.field != "lateral" && chart.field != "membro")
                invalid(where + ": campo \"" + chart.field + "\" (tronco, lateral ou membro)");
            if (chart.method == "isometrica" && !chart.field.empty()) invalid(where + ": a isometrica nao tem campo");
            if (s.chartIndex(chart.id) >= 0) invalid(where + ": repetida");
            s.charts.push_back(std::move(chart));
        }
        if (s.charts.size() > 32) invalid("mais de 32 cartas");
        for (const Json& item : field(block, "anchors", "surface")) {
            SurfaceAnchor anchor;
            anchor.id = field(item, "id", "anchors").get<std::string>();
            const std::string where = "ancora " + anchor.id;
            anchor.panel = indexOf(s.panels, field(item, "panel", where).get<std::string>());
            if (anchor.panel < 0) invalid(where + ": painel desconhecido");
            anchor.position = vec2(field(item, "position", where), where);
            anchor.up = vec2(field(item, "up", where), where);
            anchor.size = vec2(field(item, "size", where), where);
            s.anchors.push_back(std::move(anchor));
        }
        const Json& identity = field(block, "identityLayout", "surface");
        const std::vector<float> size = numbers(field(identity, "size", "identityLayout"), 2, "identityLayout.size");
        s.identityLayout = { static_cast<int>(size[0]), static_cast<int>(size[1]), field(identity, "inset", "identityLayout").get<int>() };
        if (s.identityLayout.width <= 0 || s.identityLayout.height <= 0 || s.identityLayout.inset < 0) invalid("identityLayout invalido");
        for (const Json& item : field(block, "zones", "surface")) {
            SurfaceZone zone;
            zone.id = field(item, "id", "zones").get<std::string>();
            const std::string where = "zona " + zone.id;
            zone.panel = indexOf(s.panels, field(item, "panel", where).get<std::string>());
            if (zone.panel < 0) invalid(where + ": painel desconhecido");
            const std::vector<float> rect = numbers(field(item, "rect", where), 4, where + ".rect");
            zone.min = { rect[0], rect[1] };
            zone.max = { rect[2], rect[3] };
            if (!(zone.min.x < zone.max.x && zone.min.y < zone.max.y)) invalid(where + ": retangulo vazio");
            zone.block = field(item, "block", where).get<std::array<int, 4>>();
            zone.inset = field(item, "inset", where).get<int>();
            const auto [bx, by, bw, bh] = zone.block;
            if (bx < 0 || by < 0 || bw <= 2 * zone.inset || bh <= 2 * zone.inset || bx + bw > s.identityLayout.width
                || by + bh > s.identityLayout.height)
                invalid(where + ": o bloco sai da textura de identificacao");
            s.zones.push_back(std::move(zone));
        }
        if (s.zones.size() > 127) invalid("zonas demais");
        const Json& storage = field(block, "storageLayout", "surface");
        s.storageLayout.version = field(storage, "version", "storageLayout").get<int>();
        s.storageLayout.baseSize = field(storage, "baseSize", "storageLayout").get<int>();
        s.storageLayout.texelsPerMeter = field(storage, "texelsPerMeter", "storageLayout").get<float>();
        if (s.storageLayout.version != s.storage) invalid("storageLayout: versao diferente da do template");
        if (s.storageLayout.baseSize <= 0 || !(s.storageLayout.texelsPerMeter > 0.0f)) invalid("storageLayout: tamanho ou densidade invalidos");
        std::set<int> placed;
        for (const Json& item : field(storage, "panels", "storageLayout")) {
            StorageRect rect;
            const std::string name = field(item, "panel", "storageLayout").get<std::string>();
            rect.panel = indexOf(s.panels, name);
            if (rect.panel < 0 || !placed.insert(rect.panel).second) invalid("storageLayout: painel " + name + " desconhecido ou repetido");
            const std::vector<float> r = numbers(field(item, "rect", name), 4, "storageLayout " + name);
            const std::vector<float> b = numbers(field(item, "bbox", name), 4, "storageLayout " + name);
            rect.rectMin = { r[0], r[1] };
            rect.rectMax = { r[2], r[3] };
            rect.boxMin = { b[0], b[1] };
            rect.boxMax = { b[2], b[3] };
            rect.density = field(item, "density", name).get<float>();
            if (r[0] < 0.0f || r[1] < 0.0f || r[2] > 1.0f || r[3] > 1.0f || r[0] >= r[2] || r[1] >= r[3] || b[0] >= b[2] || b[1] >= b[3]
                || !(rect.density > 0.0f))
                invalid("storageLayout " + name + ": retangulo, caixa ou densidade invalidos");
            s.storageLayout.panels.push_back(rect);
        }

        // ---- Por triangulo ----
        const std::size_t n = triangles.size();
        const Json& panel = perTriangle(document, "trianglePanel", n);
        const Json& region = perTriangle(document, "triangleRegion", n);
        const Json& zone = perTriangle(document, "triangleIdentity", n);
        const Json& mask = perTriangle(document, "triangleChartMask", n);
        const Json& design = perTriangle(document, "triangleDesign", n);
        const Json& kit = perTriangle(document, "triangleKitUv", n);
        const Json& identityUv = perTriangle(document, "triangleIdentityUv", n);
        const Json& reference = perTriangle(document, "triangleReference", n);
        s.trianglePanel.resize(n);
        s.triangleRegion.resize(n);
        s.triangleIdentity.resize(n);
        s.triangleChartMask.resize(n);
        s.triangleDesign.resize(n);
        s.triangleKitUv.resize(n);
        s.triangleIdentityUv.resize(n);
        s.triangleReference.resize(n);
        const std::uint32_t chartBits = s.charts.size() >= 32 ? 0xFFFFFFFFu : ((1u << s.charts.size()) - 1u);
        for (std::size_t t = 0; t < n; ++t) {
            const std::string where = "triangulo " + std::to_string(t);
            const int p = panel[t].get<int>();
            const int r = region[t].get<int>();
            const int z = zone[t].get<int>();
            const std::uint32_t bits = mask[t].get<std::uint32_t>();
            if (p < -1 || p >= static_cast<int>(s.panels.size())) invalid(where + ": painel fora da lista");
            if (r < 0 || r >= static_cast<int>(s.regions.size())) invalid(where + ": regiao fora da lista");
            if (z < -1 || z >= static_cast<int>(s.zones.size())) invalid(where + ": zona fora da lista");
            if ((bits & ~chartBits) != 0u) invalid(where + ": carta fora da lista");
            for (const std::uint32_t v : triangles[t])
                if (v >= vertexCount) invalid(where + ": vertice fora da malha");
            s.trianglePanel[t] = static_cast<std::int16_t>(p);
            s.triangleRegion[t] = static_cast<std::uint8_t>(r);
            s.triangleIdentity[t] = static_cast<std::int8_t>(z);
            s.triangleChartMask[t] = bits;
            if ((p >= 0) == design[t].is_null()) invalid(where + ": coordenada de desenho " + (p >= 0 ? "ausente" : "sem painel"));
            if ((p >= 0) == kit[t].is_null()) invalid(where + ": UV0 " + (p >= 0 ? "ausente" : "sem painel"));
            if ((z >= 0) == identityUv[t].is_null()) invalid(where + ": UV1 " + (z >= 0 ? "ausente" : "sem zona"));
            if ((p >= 0) == reference[t].is_null()) invalid(where + ": referencia 3D " + (p >= 0 ? "ausente" : "sem painel"));
            if (p >= 0) {
                if (!reference[t].is_array() || reference[t].size() != 3) invalid(where + " referencia 3D: esperava tres cantos");
                for (std::size_t k = 0; k < 3; ++k) s.triangleReference[t][k] = vec3(reference[t][k], where + " referencia 3D");
                s.triangleDesign[t] = corners(design[t], where + " desenho");
                s.triangleKitUv[t] = corners(kit[t], where + " UV0");
                for (const Vec2& uv : s.triangleKitUv[t])
                    if (uv.x < -1e-4f || uv.y < -1e-4f || uv.x > 1.0001f || uv.y > 1.0001f) invalid(where + ": UV0 fora do atlas");
            }
            if (z >= 0) {
                if (s.zones[static_cast<std::size_t>(z)].panel != p) invalid(where + ": zona de outro painel");
                s.triangleIdentityUv[t] = corners(identityUv[t], where + " UV1");
            }
        }
        const Json& charts = field(document, "triangleChart", "documento");
        s.triangleChart.assign(s.charts.size(), std::vector<Corners>(n));
        for (std::size_t c = 0; c < s.charts.size(); ++c) {
            const std::string where = "carta " + s.charts[c].id;
            const Json& entries = field(charts, s.charts[c].id.c_str(), "triangleChart");
            std::size_t expected = 0;
            for (std::size_t t = 0; t < n; ++t) expected += s.inChart(c, t) ? 1u : 0u;
            if (entries.size() != expected) invalid(where + ": " + std::to_string(entries.size()) + " triangulos para " + std::to_string(expected) + " marcados");
            for (const auto& [key, value] : entries.items()) {
                std::size_t t = 0;
                try {
                    t = static_cast<std::size_t>(std::stoul(key));
                } catch (const std::exception&) {
                    invalid(where + ": triangulo " + key);
                }
                if (t >= n || !s.inChart(c, t)) invalid(where + ": triangulo " + key + " fora da carta");
                s.triangleChart[c][t] = corners(value, where + " triangulo " + key);
            }
        }

        // ---- As costuras contra a malha: cada aresta existe e separa os lados dela ----
        std::map<std::uint64_t, std::vector<std::size_t>> edgeTriangles;
        for (std::size_t t = 0; t < n; ++t)
            for (int k = 0; k < 3; ++k)
                edgeTriangles[edgeKey(triangles[t][static_cast<std::size_t>(k)], triangles[t][static_cast<std::size_t>((k + 1) % 3)])].push_back(t);
        const auto labelOf = [&](std::size_t t) {
            return s.trianglePanel[t] >= 0 ? s.panels[static_cast<std::size_t>(s.trianglePanel[t])] : s.regions[s.triangleRegion[t]];
        };
        std::set<std::uint64_t> declared;
        for (const SurfaceSeam& seam : s.seams) {
            for (const auto& [a, b] : seam.edges) {
                if (!declared.insert(edgeKey(a, b)).second)
                    invalid(std::string(kindName(seam.kind)) + " " + seam.id + ": a aresta " + std::to_string(a) + "-" + std::to_string(b) + " ja e de outra fronteira");
                const auto found = edgeTriangles.find(edgeKey(a, b));
                if (found == edgeTriangles.end() || found->second.size() != 2)
                    invalid(std::string(kindName(seam.kind)) + " " + seam.id + ": a aresta " + std::to_string(a) + "-" + std::to_string(b) + " nao e da malha");
                const std::string la = labelOf(found->second[0]), lb = labelOf(found->second[1]);
                if (la == lb || indexOf(seam.panels, la) < 0 || indexOf(seam.panels, lb) < 0)
                    invalid(std::string(kindName(seam.kind)) + " " + seam.id + ": a aresta " + std::to_string(a) + "-" + std::to_string(b) + " separa " + la
                        + " de " + lb);
            }
        }
        // E nenhuma fronteira fica de fora (uma aresta de fechamento de um anel esquecida, por exemplo).
        for (const auto& [key, around] : edgeTriangles) {
            if (around.size() != 2 || labelOf(around[0]) == labelOf(around[1]) || declared.count(key) != 0u) continue;
            invalid("a fronteira " + std::to_string(key >> 32) + "-" + std::to_string(key & 0xFFFFFFFFu) + " entre " + labelOf(around[0]) + " e "
                + labelOf(around[1]) + " nao esta em nenhuma costura, abertura ou juncao");
        }
        return s;
    } catch (const Invalid& failure) {
        if (error != nullptr) *error = std::string("superficie: ") + failure.what();
    } catch (const Json::exception& failure) {
        if (error != nullptr) *error = std::string("superficie: ") + failure.what();
    }
    return std::nullopt;
}

Json surfaceTemplateDocument(const SurfaceTemplate& s) {
    Json block;
    block["schema"] = s.schema;
    block["parametrization"] = s.parametrization;
    block["storage"] = s.storage;
    block["panels"] = s.panels;
    block["regions"] = s.regions;
    block["charts"] = Json::array();
    for (const SurfaceChart& chart : s.charts) {
        Json item { { "id", chart.id }, { "panels", chart.panels }, { "joined", chart.joined }, { "method", chart.method } };
        item["cut"] = chart.cut.empty() ? Json() : Json(chart.cut);
        if (!chart.field.empty()) item["field"] = chart.field;
        block["charts"].push_back(std::move(item));
    }
    block["seams"] = Json::array();
    for (const SurfaceSeam& seam : s.seams)
        block["seams"].push_back(
            { { "id", seam.id }, { "kind", kindName(seam.kind) }, { "panels", seam.panels }, { "continuous", seam.continuous }, { "edges", seam.edges } });
    block["anchors"] = Json::array();
    for (const SurfaceAnchor& anchor : s.anchors)
        block["anchors"].push_back({ { "id", anchor.id }, { "panel", s.panels[static_cast<std::size_t>(anchor.panel)] }, { "position", pair(anchor.position) },
            { "up", pair(anchor.up) }, { "size", pair(anchor.size) } });
    block["zones"] = Json::array();
    for (const SurfaceZone& zone : s.zones)
        block["zones"].push_back({ { "id", zone.id }, { "panel", s.panels[static_cast<std::size_t>(zone.panel)] },
            { "rect", Json::array({ zone.min.x, zone.min.y, zone.max.x, zone.max.y }) }, { "block", zone.block }, { "inset", zone.inset } });
    block["identityLayout"] = { { "size", Json::array({ s.identityLayout.width, s.identityLayout.height }) }, { "inset", s.identityLayout.inset } };
    Json storage { { "version", s.storageLayout.version }, { "baseSize", s.storageLayout.baseSize }, { "texelsPerMeter", s.storageLayout.texelsPerMeter } };
    storage["panels"] = Json::array();
    for (const StorageRect& rect : s.storageLayout.panels)
        storage["panels"].push_back({ { "panel", s.panels[static_cast<std::size_t>(rect.panel)] },
            { "rect", Json::array({ rect.rectMin.x, rect.rectMin.y, rect.rectMax.x, rect.rectMax.y }) },
            { "bbox", Json::array({ rect.boxMin.x, rect.boxMin.y, rect.boxMax.x, rect.boxMax.y }) }, { "density", rect.density } });
    block["storageLayout"] = std::move(storage);
    Json document;
    document["surface"] = std::move(block);
    const std::size_t n = s.trianglePanel.size();
    Json design = Json::array(), kit = Json::array(), identityUv = Json::array(), reference = Json::array();
    for (std::size_t t = 0; t < n; ++t) {
        if (s.trianglePanel[t] >= 0) {
            Json corners3 = Json::array();
            for (const Vec3& p : s.triangleReference[t]) corners3.push_back(Json::array({ p.x, p.y, p.z }));
            reference.push_back(std::move(corners3));
        } else {
            reference.push_back(Json());
        }
        design.push_back(s.trianglePanel[t] >= 0 ? cornerJson(s.triangleDesign[t]) : Json());
        kit.push_back(s.trianglePanel[t] >= 0 ? cornerJson(s.triangleKitUv[t]) : Json());
        identityUv.push_back(s.triangleIdentity[t] >= 0 ? cornerJson(s.triangleIdentityUv[t]) : Json());
    }
    document["trianglePanel"] = s.trianglePanel;
    document["triangleRegion"] = s.triangleRegion;
    document["triangleIdentity"] = s.triangleIdentity;
    document["triangleChartMask"] = s.triangleChartMask;
    document["triangleDesign"] = std::move(design);
    document["triangleKitUv"] = std::move(kit);
    document["triangleIdentityUv"] = std::move(identityUv);
    document["triangleReference"] = std::move(reference);
    Json charts = Json::object();
    for (std::size_t c = 0; c < s.charts.size(); ++c) {
        Json entries = Json::object();
        for (std::size_t t = 0; t < n; ++t)
            if (s.inChart(c, t)) entries[std::to_string(t)] = cornerJson(s.triangleChart[c][t]);
        charts[s.charts[c].id] = std::move(entries);
    }
    document["triangleChart"] = std::move(charts);
    return document;
}

} // namespace MatterEngine::Appearance
