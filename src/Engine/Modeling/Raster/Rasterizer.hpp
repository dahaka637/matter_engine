#pragma once

#include "Engine/Modeling/Raster/Camera.hpp"
#include "Engine/Modeling/Raster/Image.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace MatterEngine::Modeling {

// Como linhas e pontos tratam o que esta na frente deles.
enum class DepthMode : std::uint8_t {
    // Sempre por cima.
    Ignore,
    // Escondidos atras das superficies.
    Test,
    // Atras das superficies ficam fracos (os ossos dentro da malha).
    Ghost,
};

// A luz das superficies lisas, calculada em cada pixel (a mesma das luzes de
// estudio da inspecao): um ambiente, a luz principal, a de enchimento, a de
// tras e um recorte claro na silhueta. "view": da camera para a superficie.
struct PixelLight {
    Vec3 key { 0.0f, 0.0f, 1.0f };
    Vec3 fill { 0.0f, 0.0f, 1.0f };
    Vec3 back { 0.0f, 0.0f, 1.0f };
    Vec3 view { 0.0f, 0.0f, -1.0f };
    float ambient = 0.30f;
    float keyWeight = 0.62f;
    float fillWeight = 0.20f;
    float backWeight = 0.10f;
    float rimWeight = 0.18f;

    [[nodiscard]] float intensity(Vec3 normal) const {
        const float rim = 1.0f - std::abs(dot(normal, view));
        return ambient + keyWeight * std::max(0.0f, dot(normal, key)) + fillWeight * std::max(0.0f, dot(normal, fill))
            + backWeight * std::max(0.0f, dot(normal, back)) + rimWeight * rim * rim * rim;
    }
};

// A imagem de trabalho com profundidade (chave: maior = mais perto).
struct RasterTarget {
    Image color;
    std::vector<float> depth;

    RasterTarget(int width, int height);
    [[nodiscard]] int width() const { return color.width(); }
    [[nodiscard]] int height() const { return color.height(); }
    [[nodiscard]] float depthAt(int x, int y) const;
    void clearDepth();
};

// Desenha primitivas 3D (clipadas no plano proximo) numa RasterTarget.
// Triangulos sem antisserrilhado (o supersampling cuida); linhas e discos
// com cobertura pela distancia (bordas suaves ja na resolucao de trabalho).
class Rasterizer {
public:
    Rasterizer(RasterTarget& target, const Projector& projector);

    [[nodiscard]] const Projector& projector() const { return m_projector; }
    [[nodiscard]] RasterTarget& target() { return m_target; }
    // So as linhas [begin, end) da imagem: cada thread desenha a sua faixa
    // sem tocar nas outras.
    void restrictRows(int begin, int end);
    [[nodiscard]] int rowBegin() const { return m_rowBegin; }
    [[nodiscard]] int rowEnd() const { return m_rowEnd; }

    // Cor chapada (o visual facetado), com teste e escrita de profundidade.
    void triangle(Vec3 a, Vec3 b, Vec3 c, Color color, bool depthWrite = true);
    // Cores nos vertices (pesos de skin, calor), corrigidas pela perspectiva.
    void triangleGradient(Vec3 a, Vec3 b, Vec3 c, Color colorA, Color colorB, Color colorC);
    // Textura (as coordenadas nos vertices, corrigidas pela perspectiva;
    // bilinear) vezes o tom da luz, com teste e escrita de profundidade.
    void triangleTextured(Vec3 a, Vec3 b, Vec3 c, Uv uvA, Uv uvB, Uv uvC, const Image& texture, float shade);
    // Superficie lisa: as normais dos cantos interpoladas (com a
    // perspectiva) e a luz em cada pixel, sobre a cor ou a textura.
    void triangleLit(Vec3 a, Vec3 b, Vec3 c, const std::array<Vec3, 3>& normals, Color color, const PixelLight& light);
    void triangleTexturedLit(Vec3 a, Vec3 b, Vec3 c, Uv uvA, Uv uvB, Uv uvC, const std::array<Vec3, 3>& normals, const Image& texture,
        const PixelLight& light);
    // A cor de cada pixel por uma funcao das baricentricas do triangulo
    // original (corrigidas pela perspectiva e pelo recorte), sob a mesma luz:
    // o material da roupa do sistema de aparencia.
    // (x, y: o pixel, para as derivadas do material.)
    using PixelShader = std::function<Color(float b0, float b1, float b2, int x, int y)>;
    void triangleShadedLit(Vec3 a, Vec3 b, Vec3 c, const std::array<Vec3, 3>& normals, const PixelShader& shader,
        const PixelLight& light);
    // So a profundidade (o arame sem superficie esconde as arestas de tras).
    void depthOnly(Vec3 a, Vec3 b, Vec3 c);
    // Por cima de uma superficie ja desenhada (destaques): misturado, sem
    // escrever profundidade, visivel ate "biasMeters" atras do que ja esta.
    void overlayTriangle(Vec3 a, Vec3 b, Vec3 c, Color color, float biasMeters = 0.003f);
    // Soma "amount" nos pixels cobertos, sem profundidade (o raio-x).
    void accumulate(Vec3 a, Vec3 b, Vec3 c, float amount, std::vector<float>& buffer);
    // Linha com espessura em pixels; o bias (metros) a deixa por cima da
    // superficie em que ela esta.
    void line(Vec3 a, Vec3 b, float thickness, Color color, DepthMode mode, float ghostAlpha = 0.22f,
        float biasMeters = 0.002f);
    void disc(Vec3 center, float radius, Color color, DepthMode mode, float ghostAlpha = 0.22f,
        float biasMeters = 0.004f);

private:
    void rasterClipped(Vec3 a, Vec3 b, Vec3 c, int kind, Color color, Color colorB, Color colorC, float amount,
        std::vector<float>* buffer, bool depthWrite, float biasMeters = 0.0f, const Image* texture = nullptr,
        const std::array<Vec3, 3>* normals = nullptr, const PixelLight* light = nullptr);

    RasterTarget& m_target;
    const Projector& m_projector;
    int m_rowBegin = 0;
    int m_rowEnd = 0;
    const PixelShader* m_pixelShader = nullptr;
};

// 2D na imagem final (rotulos, gizmo, regua): bordas suaves.
void drawLine2D(Image& image, float x0, float y0, float x1, float y1, float thickness, Color color);
void fillDisc2D(Image& image, float x, float y, float radius, Color color);
void strokeRect2D(Image& image, float x0, float y0, float x1, float y1, float thickness, Color color);

} // namespace MatterEngine::Modeling
