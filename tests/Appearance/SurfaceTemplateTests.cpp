// O template de superficie do sistema de aparencia (etapa 1,
// docs/SISTEMA_DE_APARENCIA.md), na troca instalada do jogador:
//  - lido e conferido: os paineis da construcao, as regioes, as cartas, as
//    costuras e as aberturas, as ancoras e as zonas;
//  - as cartas continuas nas costuras unidas, nenhum triangulo virado, e as de
//    volta com o "para cima" no eixo (a faixa sai reta);
//  - a distorcao medida (o estiramento dos paineis e das cartas);
//  - a injetividade triangulo a triangulo (o desenho de cada painel, cada carta e
//    o UV0 inteiro) e o detector pegando a dobra dentro de um painel;
//  - o UV0 dentro do retangulo de cada painel; o UV1 a zona levada ao bloco dela;
//    o espelho exato dos paineis de cada lado; a referencia 3D igual ao corpo
//    instalado (o de referencia);
//  - as recusas (campo ausente, nome desconhecido, coordenada invalida, UV0 fora
//    do atlas, bloco fora da textura, costura que nao separa os lados dela,
//    fronteira que nenhuma costura declara, referencia 3D ausente);
//  - a reindexacao (os triangulos permutados dao o mesmo template por nome) e a
//    ida e volta do documento.
// Linkado so contra MatterAppearance.

#include "Engine/Appearance/Surface/SurfaceTemplate.hpp"

#include <cstdio>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using namespace MatterEngine;
using namespace MatterEngine::Appearance;
using Json = nlohmann::json;

namespace {

int checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

struct Troca {
    Json document;
    std::vector<std::array<std::uint32_t, 3>> triangles;
    std::vector<std::array<double, 3>> positions;
};

Troca loadTroca() {
    std::ifstream file(std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/characters/player_model/blender/jogador.troca.json");
    require(static_cast<bool>(file), "a troca do jogador");
    Troca troca;
    file >> troca.document;
    troca.triangles = troca.document.at("triangles").get<std::vector<std::array<std::uint32_t, 3>>>();
    troca.positions = troca.document.at("positions").get<std::vector<std::array<double, 3>>>();
    return troca;
}

std::optional<SurfaceTemplate> parse(const Json& document, const std::vector<std::array<std::uint32_t, 3>>& triangles, std::size_t vertices,
    std::string* error) {
    return parseSurfaceTemplate(document, triangles, vertices, error);
}

struct Vec3d {
    double x, y, z;
};

Vec3d sub(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}

double dot3(Vec3d a, Vec3d b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3d cross3(Vec3d a, Vec3d b) {
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

double length3(Vec3d a) {
    return std::sqrt(dot3(a, a));
}

// O estiramento de um triangulo (max(s1, 1/s2) do jacobiano do 3D ao plano) e a
// area 3D; area2d < 0 diz que ele virou.
struct Stretch {
    double value, area, area2d;
};

Stretch stretchOf(const Troca& troca, std::size_t t, const Corners& c) {
    const auto& tri = troca.triangles[t];
    const Vec3d e1 = sub(troca.positions[tri[1]], troca.positions[tri[0]]);
    const Vec3d e2 = sub(troca.positions[tri[2]], troca.positions[tri[0]]);
    const double l1 = length3(e1);
    const Vec3d n = cross3(e1, e2);
    const double area = 0.5 * length3(n);
    const Vec3d u = { e1.x / l1, e1.y / l1, e1.z / l1 };
    const Vec3d nn = { n.x / (2.0 * area), n.y / (2.0 * area), n.z / (2.0 * area) };
    const Vec3d v = cross3(nn, u);
    // O triangulo no plano dele (isometrico) e no mapa.
    const double x1 = l1, x2 = dot3(e2, u), y2 = dot3(e2, v);
    const double a1 = c[1].x - c[0].x, b1 = c[1].y - c[0].y, a2 = c[2].x - c[0].x, b2 = c[2].y - c[0].y;
    // J * [x1 x2; 0 y2] = [a1 a2; b1 b2]
    const double j11 = a1 / x1, j21 = b1 / x1;
    const double j12 = (a2 - j11 * x2) / y2, j22 = (b2 - j21 * x2) / y2;
    const double t1 = j11 * j11 + j21 * j21, t2 = j12 * j12 + j22 * j22, t3 = j11 * j12 + j21 * j22;
    const double mean = 0.5 * (t1 + t2), spread = std::sqrt(std::max(0.0, 0.25 * (t1 - t2) * (t1 - t2) + t3 * t3));
    const double s1 = std::sqrt(mean + spread), s2 = std::sqrt(std::max(1e-18, mean - spread));
    return { std::max(s1, 1.0 / s2), area, a1 * b2 - a2 * b1 };
}

// A area da intersecao de dois triangulos anti-horarios: a recortado pelos tres lados de b
// (Sutherland-Hodgman). Dividir uma aresta ou um vertice da zero.
double clippedArea(const std::array<std::array<double, 2>, 3>& a, const std::array<std::array<double, 2>, 3>& b) {
    std::vector<std::array<double, 2>> poly(a.begin(), a.end()), out;
    for (std::size_t k = 0; k < 3 && poly.size() >= 3; ++k) {
        const auto& p0 = b[k];
        const auto& p1 = b[(k + 1) % 3];
        const auto side = [&](const std::array<double, 2>& q) { return (p1[0] - p0[0]) * (q[1] - p0[1]) - (p1[1] - p0[1]) * (q[0] - p0[0]); };
        out.clear();
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const auto& cur = poly[i];
            const auto& next = poly[(i + 1) % poly.size()];
            const double sc = side(cur), sn = side(next);
            if (sc >= 0.0) out.push_back(cur);
            if ((sc >= 0.0) != (sn >= 0.0)) {
                const double r = sc / (sc - sn);
                out.push_back({ cur[0] + (next[0] - cur[0]) * r, cur[1] + (next[1] - cur[1]) * r });
            }
        }
        poly.swap(out);
    }
    if (poly.size() < 3) return 0.0;
    double area = 0.0;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const auto& p = poly[i];
        const auto& q = poly[(i + 1) % poly.size()];
        area += p[0] * q[1] - q[0] * p[1];
    }
    return 0.5 * std::abs(area);
}

// A injetividade exata do mapa: os pares de triangulos que se sobrepoem (a intersecao com mais de
// 1e-6 da area do menor; a borda compartilhada da zero) mais os degenerados (area ate 1e-9 do
// quadrado da maior aresta). Os candidatos vem de uma grade pelas caixas.
std::size_t overlappingPairs(const std::vector<std::pair<std::size_t, Corners>>& triangles) {
    struct Item {
        std::array<std::array<double, 2>, 3> c;
        double area;
        std::array<double, 2> lo, hi;
    };
    std::vector<Item> items;
    std::size_t bad = 0;
    std::vector<double> extents;
    for (const auto& [t, corners] : triangles) {
        Item item;
        for (std::size_t k = 0; k < 3; ++k) item.c[k] = { corners[k].x, corners[k].y };
        const double cross = (item.c[1][0] - item.c[0][0]) * (item.c[2][1] - item.c[0][1]) - (item.c[2][0] - item.c[0][0]) * (item.c[1][1] - item.c[0][1]);
        double longest = 0.0;
        for (std::size_t k = 0; k < 3; ++k) {
            const double dx = item.c[k][0] - item.c[(k + 2) % 3][0], dy = item.c[k][1] - item.c[(k + 2) % 3][1];
            longest = std::max(longest, dx * dx + dy * dy);
        }
        if (std::abs(0.5 * cross) <= 1e-9 * longest) {
            ++bad;
            continue;
        }
        if (cross < 0.0) std::swap(item.c[1], item.c[2]);
        item.area = 0.5 * std::abs(cross);
        item.lo = { std::min({ item.c[0][0], item.c[1][0], item.c[2][0] }), std::min({ item.c[0][1], item.c[1][1], item.c[2][1] }) };
        item.hi = { std::max({ item.c[0][0], item.c[1][0], item.c[2][0] }), std::max({ item.c[0][1], item.c[1][1], item.c[2][1] }) };
        extents.push_back(std::max(item.hi[0] - item.lo[0], item.hi[1] - item.lo[1]));
        items.push_back(item);
    }
    if (items.empty()) return bad;
    std::nth_element(extents.begin(), extents.begin() + static_cast<std::ptrdiff_t>(extents.size() / 2), extents.end());
    const double size = std::max(2.0 * extents[extents.size() / 2], 1e-12);
    std::map<std::pair<long, long>, std::vector<std::size_t>> cells;
    for (std::size_t i = 0; i < items.size(); ++i)
        for (long x = std::lround(std::floor(items[i].lo[0] / size)); x <= std::lround(std::floor(items[i].hi[0] / size)); ++x)
            for (long y = std::lround(std::floor(items[i].lo[1] / size)); y <= std::lround(std::floor(items[i].hi[1] / size)); ++y)
                cells[{ x, y }].push_back(i);
    std::set<std::pair<std::size_t, std::size_t>> seen;
    for (const auto& [cell, members] : cells)
        for (std::size_t x = 0; x < members.size(); ++x)
            for (std::size_t y = x + 1; y < members.size(); ++y) {
                const Item& a = items[members[x]];
                const Item& b = items[members[y]];
                if (a.lo[0] >= b.hi[0] || b.lo[0] >= a.hi[0] || a.lo[1] >= b.hi[1] || b.lo[1] >= a.hi[1]) continue;
                if (!seen.insert({ members[x], members[y] }).second) continue;
                if (clippedArea(a.c, b.c) > 1e-6 * std::min(a.area, b.area)) ++bad;
            }
    return bad;
}

// O angulo (graus) entre o "para cima" da coordenada (o gradiente de y no plano do triangulo) e o
// eixo dado, projetado no plano; -1 onde o eixo quase sai do plano (o topo do ombro no tronco).
double alongAngle(const Troca& troca, std::size_t t, const Corners& c, Vec3d axis) {
    const auto& tri = troca.triangles[t];
    const Vec3d e1 = sub(troca.positions[tri[1]], troca.positions[tri[0]]);
    const Vec3d e2 = sub(troca.positions[tri[2]], troca.positions[tri[0]]);
    const double l1 = length3(e1);
    Vec3d n = cross3(e1, e2);
    const double ln = length3(n);
    n = { n.x / ln, n.y / ln, n.z / ln };
    const Vec3d u = { e1.x / l1, e1.y / l1, e1.z / l1 };
    const Vec3d v = cross3(n, u);
    const double x2 = dot3(e2, u), y2 = dot3(e2, v);
    const double gu = (c[1].y - c[0].y) / l1, gv = (c[2].y - c[0].y - gu * x2) / y2;
    const double au = dot3(axis, u), av = dot3(axis, v);
    const double along = std::sqrt(au * au + av * av) / length3(axis);
    if (along < 0.5) return -1.0;
    const double cosine = (gu * au + gv * av) / (std::sqrt(gu * gu + gv * gv) * std::sqrt(au * au + av * av));
    return std::acos(std::clamp(cosine, -1.0, 1.0)) * 180.0 / 3.14159265358979;
}

double percentile(std::vector<std::pair<double, double>> values, double q) {
    std::sort(values.begin(), values.end());
    double total = 0.0;
    for (const auto& [value, weight] : values) total += weight;
    double run = 0.0;
    for (const auto& [value, weight] : values) {
        run += weight;
        if (run >= q * total) return value;
    }
    return values.empty() ? 0.0 : values.back().first;
}

void testInstalledTemplate() {
    const Troca troca = loadTroca();
    std::string error;
    const std::optional<SurfaceTemplate> surface = parse(troca.document, troca.triangles, troca.positions.size(), &error);
    require(surface.has_value(), "o template da troca instalada: " + error);
    const SurfaceTemplate& s = *surface;
    require(s.schema == "matter.surface.v1" && s.parametrization == 1 && s.storage == s.storageLayout.version, "o schema e as versoes");
    for (const char* name : { "camisa_frente", "camisa_costas", "manga_direita", "manga_esquerda", "calcao_frente_direita", "calcao_costas_direita",
             "calcao_frente_esquerda", "calcao_costas_esquerda", "recuo_gola", "recuo_barra", "recuo_punho_direito", "recuo_punho_esquerdo",
             "recuo_cos", "recuo_barra_direita", "recuo_barra_esquerda", "meiao_direito", "meiao_esquerdo", "meiao_borda_direito",
             "meiao_borda_esquerdo" })
        require(s.panelIndex(name) >= 0, std::string("o painel ") + name);
    for (const char* name : { "camisa", "camisa_recuo", "calcao", "calcao_recuo", "meiao", "pele", "pe_oculto", "chuteira" })
        require(s.regionIndex(name) >= 0, std::string("a regiao ") + name);
    require(s.charts.size() == 10 && s.zones.size() == 3 && s.anchors.size() == 5, "as cartas, as zonas e as ancoras");

    // Toda face da roupa tem painel, e o painel diz a regiao.
    const auto parts = troca.document.at("parts");
    std::vector<std::string> partNames;
    for (const auto& part : parts) partNames.push_back(part.at("name").get<std::string>());
    const auto trianglePart = troca.document.at("trianglePart").get<std::vector<int>>();
    std::map<std::string, std::size_t> perPanel;
    for (std::size_t t = 0; t < troca.triangles.size(); ++t) {
        const std::string part = partNames[static_cast<std::size_t>(trianglePart[t])];
        if (part == "camisa" || part == "calcao") require(s.trianglePanel[t] >= 0, "triangulo da roupa sem painel");
        if (s.trianglePanel[t] >= 0) ++perPanel[s.panels[static_cast<std::size_t>(s.trianglePanel[t])]];
        const std::string region = s.regions[s.triangleRegion[t]];
        if (s.trianglePanel[t] >= 0) {
            const std::string panel = s.panels[static_cast<std::size_t>(s.trianglePanel[t])];
            const std::string expected = panel.rfind("meiao_", 0) == 0 ? "meiao"
                : panel.rfind("recuo_", 0) == 0                        ? (part == "camisa" ? "camisa_recuo" : "calcao_recuo")
                                                                       : part;
            require(region == expected, "a regiao do painel " + panel + ": " + region);
        }
    }
    require(perPanel["meiao_direito"] > 50 && perPanel["meiao_direito"] == perPanel["meiao_esquerdo"], "os meioes, iguais dos dois lados");

    // As costuras: as do pano continuas, as aberturas e juncoes nao.
    std::size_t seams = 0, openings = 0;
    for (const SurfaceSeam& seam : s.seams) {
        require(seam.continuous == (seam.kind == SeamKind::Seam), "a continuidade da costura " + seam.id);
        seams += seam.kind == SeamKind::Seam;
        openings += seam.kind == SeamKind::Opening;
    }
    require(seams == 16 && openings == 7, "16 costuras e 7 aberturas");

    // As cartas: continuas nas costuras unidas; nenhum triangulo virado.
    std::map<std::uint64_t, std::vector<std::size_t>> edgeTriangles;
    const auto key = [](std::uint32_t a, std::uint32_t b) { return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b); };
    for (std::size_t t = 0; t < troca.triangles.size(); ++t)
        for (int k = 0; k < 3; ++k) edgeTriangles[key(troca.triangles[t][static_cast<std::size_t>(k)], troca.triangles[t][static_cast<std::size_t>((k + 1) % 3)])].push_back(t);
    const auto cornerAt = [&](const Corners& c, std::size_t t, std::uint32_t vertex) {
        for (std::size_t k = 0; k < 3; ++k)
            if (troca.triangles[t][k] == vertex) return c[k];
        throw std::runtime_error("vertice fora do triangulo");
    };
    for (std::size_t c = 0; c < s.charts.size(); ++c) {
        const SurfaceChart& chart = s.charts[c];
        std::size_t checked = 0;
        for (const std::string& name : chart.joined) {
            for (const auto& [a, b] : s.seam(name)->edges) {
                const auto& pair = edgeTriangles.at(key(a, b));
                if (!s.inChart(c, pair[0]) || !s.inChart(c, pair[1])) continue;
                for (const std::uint32_t v : { a, b }) {
                    const Vec2 p = cornerAt(s.triangleChart[c][pair[0]], pair[0], v), q = cornerAt(s.triangleChart[c][pair[1]], pair[1], v);
                    require(std::abs(p.x - q.x) < 1e-5f && std::abs(p.y - q.y) < 1e-5f, "a carta " + chart.id + " abre na costura " + name);
                }
                ++checked;
            }
        }
        require(chart.joined.empty() || checked > 0, "a carta " + chart.id + " sem costura unida conferida");
        std::vector<std::pair<double, double>> values;
        for (std::size_t t = 0; t < troca.triangles.size(); ++t) {
            if (!s.inChart(c, t)) continue;
            const Stretch st = stretchOf(troca, t, s.triangleChart[c][t]);
            require(st.area2d > 0.0, "triangulo virado na carta " + chart.id);
            values.push_back({ st.value, st.area });
        }
        const double p95 = percentile(values, 0.95);
        require(p95 < 1.25, "a carta " + chart.id + " estica demais (p95 " + std::to_string(p95) + ")");
        // As cartas de volta existem para a faixa ficar reta: o "para cima" delas segue o eixo (o
        // vertical no tronco; o do osso nos membros) e a faixa de y constante da a volta perpendicular.
        if (chart.id.rfind("volta_", 0) != 0) continue;
        const std::string side = chart.id.find("direit") != std::string::npos ? "Right" : "Left";
        const auto joint = [&](const std::string& name) {
            for (const auto& entry : troca.document.at("joints"))
                if (entry.at("name") == name) return entry.at("position").get<std::array<double, 3>>();
            throw std::runtime_error("sem a junta " + name);
        };
        const auto bone = [&](const char* from, const char* to) { return sub(joint(side + from), joint(side + to)); };
        const Vec3d axis = chart.id == "volta_tronco"            ? Vec3d { 0.0, 0.0, 1.0 }
            : chart.id.rfind("volta_manga_", 0) == 0             ? bone("UpperArm", "Forearm")
            : chart.id.rfind("volta_perna_", 0) == 0             ? bone("Thigh", "Shin")
                                                                 : bone("Shin", "Foot");
        std::vector<std::pair<double, double>> angles;
        for (std::size_t t = 0; t < troca.triangles.size(); ++t) {
            if (!s.inChart(c, t)) continue;
            const double angle = alongAngle(troca, t, s.triangleChart[c][t], axis);
            if (angle >= 0.0) angles.push_back({ angle, stretchOf(troca, t, s.triangleChart[c][t]).area });
        }
        require(angles.size() > 50 && percentile(angles, 0.95) < 4.0,
            "a carta " + chart.id + " gira em relacao ao eixo (p95 " + std::to_string(percentile(angles, 0.95)) + " graus)");
    }

    // Os paineis: estiramento baixo, nenhum virado, o espelho exato dos lados.
    for (std::size_t p = 0; p < s.panels.size(); ++p) {
        std::vector<std::pair<double, double>> values;
        for (std::size_t t = 0; t < troca.triangles.size(); ++t) {
            if (s.trianglePanel[t] != static_cast<int>(p)) continue;
            const Stretch st = stretchOf(troca, t, s.triangleDesign[t]);
            require(st.area2d > 0.0, "triangulo virado no painel " + s.panels[p]);
            values.push_back({ st.value, st.area });
        }
        require(percentile(values, 0.95) < 1.15, "o painel " + s.panels[p] + " estica demais");
    }
    std::map<std::array<long, 3>, std::uint32_t> byPosition;
    const auto grid = [](const std::array<double, 3>& p) {
        return std::array<long, 3> { std::lround(p[0] * 1e5), std::lround(p[1] * 1e5), std::lround(p[2] * 1e5) };
    };
    for (std::uint32_t v = 0; v < troca.positions.size(); ++v) byPosition[grid(troca.positions[v])] = v;
    std::map<std::array<std::uint32_t, 3>, std::size_t> byCorners;
    for (std::size_t t = 0; t < troca.triangles.size(); ++t) {
        auto sorted = troca.triangles[t];
        std::sort(sorted.begin(), sorted.end());
        byCorners[sorted] = t;
    }
    std::size_t mirrored = 0;
    for (std::size_t t = 0; t < troca.triangles.size(); ++t) {
        if (s.trianglePanel[t] < 0) continue;
        const std::string panel = s.panels[static_cast<std::size_t>(s.trianglePanel[t])];
        if (panel.find("direit") == std::string::npos) continue;
        std::array<std::uint32_t, 3> partner {};
        for (std::size_t k = 0; k < 3; ++k) {
            auto p = troca.positions[troca.triangles[t][k]];
            p[1] = -p[1];
            partner[k] = byPosition.at(grid(p));
        }
        auto sorted = partner;
        std::sort(sorted.begin(), sorted.end());
        const std::size_t m = byCorners.at(sorted);
        for (std::size_t k = 0; k < 3; ++k) {
            const Vec2 a = s.triangleDesign[t][k], b = cornerAt(s.triangleDesign[m], m, partner[k]);
            require(std::abs(a.x + b.x) < 1e-5f && std::abs(a.y - b.y) < 1e-5f, "o espelho do painel " + panel);
        }
        ++mirrored;
    }
    require(mirrored > 300, "os paineis da direita conferidos com o espelho");

    // A injetividade exata, par a par de triangulos: o desenho de cada painel, cada carta e o UV0 inteiro.
    for (std::size_t p = 0; p < s.panels.size(); ++p) {
        std::vector<std::pair<std::size_t, Corners>> triangles;
        for (std::size_t t = 0; t < troca.triangles.size(); ++t)
            if (s.trianglePanel[t] == static_cast<int>(p)) triangles.push_back({ t, s.triangleDesign[t] });
        require(overlappingPairs(triangles) == 0, "o desenho do painel " + s.panels[p] + " se sobrepoe ou degenera");
    }
    for (std::size_t c = 0; c < s.charts.size(); ++c) {
        std::vector<std::pair<std::size_t, Corners>> triangles;
        for (std::size_t t = 0; t < troca.triangles.size(); ++t)
            if (s.inChart(c, t)) triangles.push_back({ t, s.triangleChart[c][t] });
        require(overlappingPairs(triangles) == 0, "a carta " + s.charts[c].id + " se sobrepoe ou degenera");
    }
    {
        std::vector<std::pair<std::size_t, Corners>> triangles;
        for (std::size_t t = 0; t < troca.triangles.size(); ++t)
            if (s.trianglePanel[t] >= 0) triangles.push_back({ t, s.triangleKitUv[t] });
        const std::size_t clashes = overlappingPairs(triangles);
        require(clashes == 0, "o UV0 se sobrepoe ou degenera em " + std::to_string(clashes) + " pares de triangulos");
        // O detector pega a dobra dentro de um painel so (o defeito antigo do meiao), ate um triangulo
        // minusculo (a 1% do vizinho, menor que um texel de 2048) posto por cima do vizinho do mesmo
        // painel, e o degenerado.
        std::size_t a = 0;
        while (s.trianglePanel[triangles[a].first] != s.trianglePanel[triangles[a + 1].first]) ++a;
        const Corners under = triangles[a].second;
        const Vec2 center = (under[0] + under[1] + under[2]) / 3.0f;
        for (std::size_t k = 0; k < 3; ++k) triangles[a + 1].second[k] = center + (under[k] - center) * 0.01f;
        require(overlappingPairs(triangles) == 1, "o detector nao viu um triangulo minusculo por cima do vizinho do mesmo painel");
        triangles[a + 1].second = { under[0], under[1], under[0] + (under[1] - under[0]) * 0.5f };
        require(overlappingPairs(triangles) == 1, "o detector nao viu um triangulo degenerado");
    }
    // A referencia 3D canonica: a do corpo de referencia (o instalado e ele), canto a canto.
    for (std::size_t t = 0; t < troca.triangles.size(); ++t) {
        if (s.trianglePanel[t] < 0) continue;
        for (std::size_t k = 0; k < 3; ++k) {
            const auto& p = troca.positions[troca.triangles[t][k]];
            const Vec3& r = s.triangleReference[t][k];
            require(std::abs(r.x - p[0]) < 1e-6 && std::abs(r.y - p[1]) < 1e-6 && std::abs(r.z - p[2]) < 1e-6, "a referencia 3D do canto");
        }
    }

    // O UV0: dentro do retangulo de cada painel e sem sobreposicao entre paineis.
    for (const StorageRect& rect : s.storageLayout.panels) {
        const float slack = 1.5f / static_cast<float>(s.storageLayout.baseSize);
        for (std::size_t t = 0; t < troca.triangles.size(); ++t) {
            if (s.trianglePanel[t] != rect.panel) continue;
            for (const Vec2& uv : s.triangleKitUv[t])
                require(uv.x >= rect.rectMin.x - slack && uv.y >= rect.rectMin.y - slack && uv.x <= rect.rectMax.x + slack && uv.y <= rect.rectMax.y + slack,
                    "o UV0 sai do retangulo de " + s.panels[static_cast<std::size_t>(rect.panel)]);
        }
    }
    constexpr int size = 1024;
    std::vector<int> owner(static_cast<std::size_t>(size * size), -1);
    std::size_t overlaps = 0, covered = 0;
    for (std::size_t t = 0; t < troca.triangles.size(); ++t) {
        if (s.trianglePanel[t] < 0) continue;
        const Corners& c = s.triangleKitUv[t];
        const float x0 = std::min({ c[0].x, c[1].x, c[2].x }) * size, x1 = std::max({ c[0].x, c[1].x, c[2].x }) * size;
        const float y0 = std::min({ c[0].y, c[1].y, c[2].y }) * size, y1 = std::max({ c[0].y, c[1].y, c[2].y }) * size;
        const float area = (c[1].x - c[0].x) * (c[2].y - c[0].y) - (c[2].x - c[0].x) * (c[1].y - c[0].y);
        for (int y = std::max(0, static_cast<int>(y0)); y <= std::min(size - 1, static_cast<int>(y1)); ++y)
            for (int x = std::max(0, static_cast<int>(x0)); x <= std::min(size - 1, static_cast<int>(x1)); ++x) {
                const float px = (static_cast<float>(x) + 0.5f) / size, py = (static_cast<float>(y) + 0.5f) / size;
                const float w0 = ((c[1].x - px) * (c[2].y - py) - (c[2].x - px) * (c[1].y - py)) / area;
                const float w1 = ((c[2].x - px) * (c[0].y - py) - (c[0].x - px) * (c[2].y - py)) / area;
                const float w2 = 1.0f - w0 - w1;
                if (w0 <= 1e-4f || w1 <= 1e-4f || w2 <= 1e-4f) continue;
                int& here = owner[static_cast<std::size_t>(y * size + x)];
                if (here >= 0 && here != s.trianglePanel[t]) ++overlaps;
                if (here < 0) ++covered;
                here = s.trianglePanel[t];
            }
    }
    require(overlaps == 0, "o UV0 sobrepoe paineis em " + std::to_string(overlaps) + " texels");
    // (Um sexto: o layout s2 tem 64 texels de folga entre os paineis - o nivel
    // seguro dos mips em 4 -, e o tecido cobre ~20% do atlas; o s1, com 16, ~40%.)
    std::printf("o UV0 cobre %.1f%% do atlas\n", 100.0 * static_cast<double>(covered) / static_cast<double>(size * size));
    require(covered > static_cast<std::size_t>(size * size) / 6, "o UV0 cobre menos de um sexto do atlas");

    // O UV1: a zona levada ao bloco dela (o mesmo mapa afim em todos os cantos).
    for (std::size_t t = 0; t < troca.triangles.size(); ++t) {
        if (s.triangleIdentity[t] < 0) continue;
        const SurfaceZone& zone = s.zones[static_cast<std::size_t>(s.triangleIdentity[t])];
        require(s.trianglePanel[t] == zone.panel, "a zona " + zone.id + " fora do painel dela");
        const float w = static_cast<float>(s.identityLayout.width), h = static_cast<float>(s.identityLayout.height);
        for (std::size_t k = 0; k < 3; ++k) {
            const Vec2 d = s.triangleDesign[t][k];
            const float u = (static_cast<float>(zone.block[0] + zone.inset)
                                + (d.x - zone.min.x) / (zone.max.x - zone.min.x) * static_cast<float>(zone.block[2] - 2 * zone.inset)) / w;
            const float v = (static_cast<float>(zone.block[1] + zone.inset)
                                + (zone.max.y - d.y) / (zone.max.y - zone.min.y) * static_cast<float>(zone.block[3] - 2 * zone.inset)) / h;
            require(std::abs(u - s.triangleIdentityUv[t][k].x) < 2e-5f && std::abs(v - s.triangleIdentityUv[t][k].y) < 2e-5f, "o UV1 da zona " + zone.id);
        }
    }
}

// Uma troca alterada e a falha esperada (a mensagem tem o trecho dado).
void requireRejected(Json document, const std::vector<std::array<std::uint32_t, 3>>& triangles, std::size_t vertices, const std::string& expected,
    const std::string& what) {
    std::string error;
    require(!parse(document, triangles, vertices, &error).has_value(), what + ": aceito");
    require(error.find(expected) != std::string::npos, what + ": a mensagem \"" + error + "\" nao diz \"" + expected + "\"");
}

void testRejections() {
    const Troca troca = loadTroca();
    const std::size_t n = troca.positions.size();
    {
        Json d = troca.document;
        d.erase("trianglePanel");
        requireRejected(d, troca.triangles, n, "trianglePanel", "sem a lista dos paineis por triangulo");
    }
    {
        Json d = troca.document;
        d["surface"]["seams"][0]["panels"][0] = "camisa_inventada";
        requireRejected(d, troca.triangles, n, "desconhecido", "uma costura com um lado desconhecido");
    }
    {
        Json d = troca.document;
        std::size_t t = 0;
        while (d["trianglePanel"][t].get<int>() < 0) ++t;
        d["triangleDesign"][t][1] = Json::array({ "x", 0.0 });
        requireRejected(d, troca.triangles, n, "esperava [x, y]", "uma coordenada de desenho invalida");
    }
    {
        Json d = troca.document;
        std::size_t t = 0;
        while (d["trianglePanel"][t].get<int>() < 0) ++t;
        d["triangleKitUv"][t][0] = Json::array({ 1.5, 0.5 });
        requireRejected(d, troca.triangles, n, "UV0 fora do atlas", "um UV0 fora do atlas");
    }
    {
        Json d = troca.document;
        d["surface"]["zones"][0]["block"][0] = 600;
        requireRejected(d, troca.triangles, n, "o bloco sai", "uma zona fora da textura de identificacao");
    }
    {
        Json d = troca.document;
        d["surface"]["charts"][0]["joined"][0] = "costura_inventada";
        requireRejected(d, troca.triangles, n, "nao e uma costura", "uma carta unindo uma costura que nao existe");
    }
    {
        // Uma aresta do lado direito na costura do ombro: ela nao separa os lados dele.
        Json d = troca.document;
        Json* side = nullptr;
        Json* shoulder = nullptr;
        for (Json& seam : d["surface"]["seams"]) {
            if (seam["id"] == "lado_direito") side = &seam;
            if (seam["id"] == "calcao_meio_frente") shoulder = &seam;
        }
        require(side != nullptr && shoulder != nullptr, "as costuras do teste");
        (*shoulder)["edges"].push_back((*side)["edges"][0]);
        (*side)["edges"].erase((*side)["edges"].begin());
        requireRejected(d, troca.triangles, n, "separa", "uma costura com uma aresta de outra");
    }
    {
        // Uma aresta de fronteira que nenhuma costura declara (o fechamento de um anel esquecido).
        Json d = troca.document;
        for (Json& seam : d["surface"]["seams"])
            if (seam["id"] == "gola") seam["edges"].erase(seam["edges"].begin());
        requireRejected(d, troca.triangles, n, "nao esta em nenhuma costura", "uma fronteira esquecida");
    }
    {
        // A proveniencia das cartas: o metodo dito (nunca suposto) e o campo so no metodo do campo.
        Json d = troca.document;
        d["surface"]["charts"][0].erase("method");
        requireRejected(d, troca.triangles, n, "falta \"method\"", "uma carta sem o metodo");
        d = troca.document;
        for (Json& chart : d["surface"]["charts"])
            if (chart["method"] == "campo") chart.erase("field");
        requireRejected(d, troca.triangles, n, "tronco, lateral ou membro", "uma carta do campo sem o campo");
        d = troca.document;
        for (Json& chart : d["surface"]["charts"])
            if (chart["method"] == "isometrica") chart["field"] = "membro";
        requireRejected(d, troca.triangles, n, "a isometrica nao tem campo", "uma carta isometrica com campo");
    }
    {
        Json d = troca.document;
        std::size_t t = 0;
        while (d["trianglePanel"][t].get<int>() < 0) ++t;
        d["triangleReference"][t] = nullptr;
        requireRejected(d, troca.triangles, n, "referencia 3D ausente", "um triangulo de painel sem a referencia 3D");
    }
    {
        Json d = troca.document;
        d["triangleDesign"].erase(d["triangleDesign"].end() - 1);
        requireRejected(d, troca.triangles, n, "triangleDesign", "a lista de desenho mais curta que os triangulos");
    }
}

// Os triangulos permutados (e tudo por triangulo junto): o mesmo template por nome.
void testReindexing() {
    const Troca troca = loadTroca();
    std::string error;
    const auto original = parse(troca.document, troca.triangles, troca.positions.size(), &error);
    require(original.has_value(), error);
    const std::size_t n = troca.triangles.size();
    std::vector<std::size_t> order(n);
    std::iota(order.begin(), order.end(), 0u);
    std::mt19937 random(7);
    std::shuffle(order.begin(), order.end(), random);
    Json d = troca.document;
    std::vector<std::array<std::uint32_t, 3>> triangles(n);
    for (const char* key : { "trianglePanel", "triangleRegion", "triangleIdentity", "triangleChartMask", "triangleDesign", "triangleKitUv",
             "triangleIdentityUv", "triangleReference" }) {
        Json moved = Json::array();
        for (std::size_t i = 0; i < n; ++i) moved.push_back(troca.document[key][order[i]]);
        d[key] = moved;
    }
    std::vector<std::size_t> where(n);
    for (std::size_t i = 0; i < n; ++i) {
        triangles[i] = troca.triangles[order[i]];
        where[order[i]] = i;
    }
    Json charts = Json::object();
    for (const auto& [id, entries] : troca.document["triangleChart"].items()) {
        Json moved = Json::object();
        for (const auto& [t, corners] : entries.items()) moved[std::to_string(where[std::stoul(t)])] = corners;
        charts[id] = moved;
    }
    d["triangleChart"] = charts;
    const auto shuffled = parse(d, triangles, troca.positions.size(), &error);
    require(shuffled.has_value(), "o template com os triangulos permutados: " + error);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t t = order[i];
        require(shuffled->trianglePanel[i] == original->trianglePanel[t] && shuffled->triangleRegion[i] == original->triangleRegion[t]
                && shuffled->triangleIdentity[i] == original->triangleIdentity[t],
            "a permutacao muda o painel, a regiao ou a zona");
        for (std::size_t k = 0; k < 3 && original->trianglePanel[t] >= 0; ++k)
            require(shuffled->triangleDesign[i][k].x == original->triangleDesign[t][k].x && shuffled->triangleKitUv[i][k].y == original->triangleKitUv[t][k].y,
                "a permutacao muda as coordenadas");
        for (std::size_t c = 0; c < original->charts.size(); ++c)
            if (original->inChart(c, t)) require(shuffled->triangleChart[c][i][0].x == original->triangleChart[c][t][0].x, "a permutacao muda as cartas");
    }
    require(shuffled->seams.size() == original->seams.size() && shuffled->zones.size() == original->zones.size(), "as definicoes iguais por nome");
}

// O documento gravado de volta le igual.
void testDocumentRoundTrip() {
    const Troca troca = loadTroca();
    std::string error;
    const auto first = parse(troca.document, troca.triangles, troca.positions.size(), &error);
    require(first.has_value(), error);
    const Json written = surfaceTemplateDocument(*first);
    const auto second = parse(written, troca.triangles, troca.positions.size(), &error);
    require(second.has_value(), "o documento gravado: " + error);
    require(second->panels == first->panels && second->regions == first->regions && second->seams.size() == first->seams.size()
            && second->storageLayout.panels.size() == first->storageLayout.panels.size(),
        "as definicoes na ida e volta");
    for (std::size_t t = 0; t < troca.triangles.size(); ++t) {
        require(second->trianglePanel[t] == first->trianglePanel[t] && second->triangleChartMask[t] == first->triangleChartMask[t], "os dados por triangulo");
        for (std::size_t k = 0; k < 3 && first->trianglePanel[t] >= 0; ++k)
            require(second->triangleDesign[t][k].x == first->triangleDesign[t][k].x && second->triangleKitUv[t][k].y == first->triangleKitUv[t][k].y
                    && second->triangleReference[t][k].z == first->triangleReference[t][k].z,
                "as coordenadas na ida e volta");
    }
    for (std::size_t c = 0; c < first->charts.size(); ++c) {
        require(second->charts[c].method == first->charts[c].method && second->charts[c].field == first->charts[c].field
                && second->charts[c].joined == first->charts[c].joined && second->charts[c].cut == first->charts[c].cut,
            "a carta " + first->charts[c].id + " na ida e volta");
    }
}

} // namespace

int main() {
    try {
        testInstalledTemplate();
        testRejections();
        testReindexing();
        testDocumentRoundTrip();
    } catch (const std::exception& exception) {
        std::cerr << "MatterAppearance test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "MatterAppearance tests passed (" << checks << " checagens)\n";
    return 0;
}
