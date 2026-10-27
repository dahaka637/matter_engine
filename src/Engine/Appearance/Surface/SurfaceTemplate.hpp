#pragma once

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"

#include <nlohmann/json_fwd.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// O template de superficie do sistema de aparencia (docs/SISTEMA_DE_APARENCIA.md):
// onde a pintura mora. Os paineis (os moldes da roupa), as regioes, as cartas
// (unioes de paineis pelas costuras escolhidas), as costuras e as aberturas, as
// ancoras e as zonas de identificacao e, por triangulo da malha (na ordem dela),
// o painel, a regiao, a zona e as coordenadas: a de desenho (metros no pano
// planificado), a de cada carta, o UV0 (o endereco no atlas do uniforme) e o UV1
// (o da identificacao). So os dados e a conferencia deles: nada de render, de
// interface nem da modelagem.
namespace MatterEngine::Appearance {

using Corners = std::array<Vec2, 3>;

enum class SeamKind : std::uint8_t { Seam, Opening, Junction };

struct SurfaceChart {
    std::string id;
    std::vector<std::string> panels;
    std::vector<std::string> joined;
    std::string cut;
    // "campo" (Poisson pelos campos de direcao) ou "isometrica" (ARAP).
    std::string method;
    std::string field;
};

// Uma costura (entre dois paineis: a pintura pode continuar), uma abertura
// (painel x faixa do recuo) ou uma juncao (com a pele, entre faixas); as
// arestas sao pares de vertices da malha.
struct SurfaceSeam {
    std::string id;
    SeamKind kind = SeamKind::Junction;
    // Os paineis dos dois lados (nas juncoes, tambem regioes).
    std::vector<std::string> panels;
    bool continuous = false;
    std::vector<std::array<std::uint32_t, 2>> edges;
};

struct SurfaceAnchor {
    std::string id;
    int panel = -1;
    Vec2 position;
    Vec2 up { 0.0f, 1.0f };
    Vec2 size;
};

// Uma zona de identificacao: o retangulo canonico em coordenada de desenho do
// painel e o bloco dela na textura de identificacao (x, y, largura, altura em
// texels), com o conteudo recuado inset texels.
struct SurfaceZone {
    std::string id;
    int panel = -1;
    Vec2 min;
    Vec2 max;
    std::array<int, 4> block {};
    int inset = 0;
};

struct StorageRect {
    int panel = -1;
    // O retangulo no atlas (0 a 1, v para baixo) e a caixa do painel em
    // coordenada de desenho quando o layout foi feito.
    Vec2 rectMin;
    Vec2 rectMax;
    Vec2 boxMin;
    Vec2 boxMax;
    float density = 1.0f;
};

struct StorageLayout {
    int version = 0;
    int baseSize = 0;
    float texelsPerMeter = 0.0f;
    std::vector<StorageRect> panels;
};

struct IdentityLayout {
    int width = 0;
    int height = 0;
    int inset = 0;
};

struct SurfaceTemplate {
    std::string schema;
    int parametrization = 0;
    int storage = 0;
    std::vector<std::string> panels;
    std::vector<std::string> regions;
    std::vector<SurfaceChart> charts;
    std::vector<SurfaceSeam> seams;
    std::vector<SurfaceAnchor> anchors;
    std::vector<SurfaceZone> zones;
    StorageLayout storageLayout;
    IdentityLayout identityLayout;
    // Por triangulo (-1: sem painel, sem zona). Painel, regiao, zona e as cartas
    // sao dados da face: nunca se interpolam.
    std::vector<std::int16_t> trianglePanel;
    std::vector<std::uint8_t> triangleRegion;
    std::vector<std::int8_t> triangleIdentity;
    std::vector<std::uint32_t> triangleChartMask;
    // A coordenada de desenho e o UV0 (validos onde ha painel), o UV1 (onde ha
    // zona) e as cartas ([carta][triangulo], validas pelo bit da carta).
    std::vector<Corners> triangleDesign;
    std::vector<Corners> triangleKitUv;
    std::vector<Corners> triangleIdentityUv;
    std::vector<std::vector<Corners>> triangleChart;
    // A referencia 3D canonica de cada canto (valida onde ha painel): a posicao
    // na pose de ligacao do corpo de referencia, que um corpo diferente nao muda
    // (os desenhos projetados em 3D sao avaliados nela).
    std::vector<std::array<Vec3, 3>> triangleReference;

    [[nodiscard]] int panelIndex(std::string_view name) const;
    [[nodiscard]] int regionIndex(std::string_view name) const;
    [[nodiscard]] int chartIndex(std::string_view id) const;
    [[nodiscard]] const SurfaceSeam* seam(std::string_view id) const;
    [[nodiscard]] bool inChart(std::size_t chart, std::size_t triangle) const {
        return ((triangleChartMask[triangle] >> chart) & 1u) != 0u;
    }
};

// O template de um documento (a troca do Blender ou a superficie do jogo): o
// bloco "surface" e as listas por triangulo, conferidos contra a malha (os
// triangulos, com os indices dos vertices do documento, e o numero de
// vertices). Toda aresta entre dois rotulos diferentes (painel, ou a regiao de
// um triangulo sem painel) tem que estar numa costura, abertura ou juncao: uma
// fronteira esquecida e erro. Erro: a primeira falha, dita.
[[nodiscard]] std::optional<SurfaceTemplate> parseSurfaceTemplate(const nlohmann::json& document,
    std::span<const std::array<std::uint32_t, 3>> triangles, std::size_t vertexCount, std::string* error);

// O documento do template, no formato que parseSurfaceTemplate le.
[[nodiscard]] nlohmann::json surfaceTemplateDocument(const SurfaceTemplate& surface);

} // namespace MatterEngine::Appearance
