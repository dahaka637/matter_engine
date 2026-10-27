#pragma once

#include "Engine/Appearance/Compose/VectorShape.hpp"
#include "Engine/Appearance/Surface/SurfaceTemplate.hpp"

#include <nlohmann/json_fwd.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Os documentos do desenho (docs/SISTEMA_DE_APARENCIA.md, etapa 2): as fontes
// (glifos de contorno ou TrueType), o modelo de desenho (vagas e camadas) e o
// uniforme escolhido (o desenho e as cores). Cada nome e conferido contra o
// template: vaga, painel, regiao, carta, ancora ou zona desconhecida e erro.
// A cor e a de compatibilidade: #RRGGBB/255, sem conversao sRGB.
namespace MatterEngine::Appearance {

using Rgba = std::array<float, 4>;

// Uma fonte: cada glifo com o contorno (x para a direita, y para cima, a base
// em 0 e a altura dos numeros em 1) e a largura.
struct GlyphFont {
    struct Glyph {
        float advance = 0.0f;
        VectorShape shape;
    };
    std::string id;
    std::map<char32_t, Glyph> glyphs;
};

// Os glifos de um documento matter.glyphs.v1 ou de um .ttf (a altura das
// maiusculas vira 1; o leitor e o da aparencia, sem a modelagem).
[[nodiscard]] std::optional<GlyphFont> parseGlyphFont(const nlohmann::json& document, std::string* error);
[[nodiscard]] std::optional<GlyphFont> loadGlyphFont(const std::filesystem::path& path, std::string* error);

// Um texto na fonte: o centro da caixa (a largura toda, a altura de 0 a 1) na
// origem; height a altura, spacing o espaco entre glifos em fracao da altura,
// stretch a largura; align -1 esquerda, 0 centro, 1 direita.
[[nodiscard]] std::optional<VectorShape> layoutText(const GlyphFont& font, const std::u32string& text, float height,
    float spacing, float stretch, int align, std::string* error);

enum class LayerSpace : std::uint8_t { Panel, Chart, Reference };
enum class LayerContent : std::uint8_t { Fill, Bands, Stripes, Gradient, Path, Text, Border };

struct DesignLayer {
    std::string id;
    // Os alvos: por regiao e por painel (a uniao).
    std::vector<std::uint8_t> regions;
    std::vector<std::uint8_t> panels;
    LayerSpace space = LayerSpace::Panel;
    int chart = -1;
    // A ancora ou a zona: so naquele painel, a origem no centro dela.
    int anchorPanel = -1;
    Vec2 origin { 0.0f, 0.0f };
    Vec2 offset { 0.0f, 0.0f };
    Vec2 scale { 1.0f, 1.0f };
    float rotateDegrees = 0.0f;
    LayerContent content = LayerContent::Fill;
    // As faixas (bands): o eixo (0 x, 1 y), contadas do topo do painel e os intervalos.
    int axis = 1;
    bool fromTop = false;
    std::vector<std::array<float, 2>> bands;
    // As listras.
    float period = 0.0f, width = 0.0f, angleDegrees = 0.0f, phase = 0.0f;
    // A borda: a faixa de largura width (m de pano) a partir das arestas de uma
    // abertura do template (a gola V, o punho), no painel de cada face.
    int seam = -1;
    float borderWidth = 0.0f;
    // O degrade (as duas vagas).
    Vec2 from { 0.0f, 0.0f }, to { 0.0f, 1.0f };
    std::array<int, 2> gradientSlots { -1, -1 };
    // O caminho ou o texto ja como forma (no espaco do conteudo).
    VectorShape shape;
    std::optional<VectorShape> mask;
    int slot = -1;
    float opacity = 1.0f;
};

struct IdentityStyle {
    int zone = -1;
    float height = 0.0f;
    Vec2 offset { 0.0f, 0.0f };
    float spacing = 0.0f, stretch = 1.0f, rotateDegrees = 0.0f;
};

struct KitDesign {
    std::string id;
    std::string name;
    std::vector<std::string> slotIds;
    std::vector<std::string> slotNames;
    std::vector<Rgba> slotDefaults;
    std::vector<DesignLayer> layers;
    std::shared_ptr<const GlyphFont> identityFont;
    int identitySlot = -1;
    std::vector<IdentityStyle> identity;

    [[nodiscard]] int slotIndex(std::string_view id) const;
};

// O desenho conferido contra o template. fonts acha uma fonte pelo id (as dos
// textos e a da identificacao).
using FontResolver = std::function<std::shared_ptr<const GlyphFont>(const std::string&)>;
[[nodiscard]] std::optional<KitDesign> parseKitDesign(const nlohmann::json& document, const SurfaceTemplate& surface,
    const FontResolver& fonts, std::string* error);

// O uniforme escolhido: as cores de todas as vagas (as omitidas, o padrao) e a
// identificacao estatica desta entrega.
struct Kit {
    std::string id;
    std::string design;
    std::vector<Rgba> colors;
    std::u32string number;
};
// So o documento (o desenho e outro arquivo): designOf traz o desenho pelo id.
[[nodiscard]] std::optional<Kit> parseKit(const nlohmann::json& document,
    const std::function<const KitDesign*(const std::string&)>& designOf, std::string* error);

// "#RRGGBB" ou "#RRGGBBAA" em valores de compatibilidade.
[[nodiscard]] std::optional<Rgba> parseColor(const std::string& text);

// Tudo de assets/appearance: o desenho e as fontes do kit, conferidos.
struct LoadedKit {
    std::shared_ptr<const KitDesign> design;
    Kit kit;
};
[[nodiscard]] std::optional<LoadedKit> loadKit(const std::filesystem::path& appearanceDirectory,
    const std::string& kitId, const SurfaceTemplate& surface, std::string* error);

} // namespace MatterEngine::Appearance
