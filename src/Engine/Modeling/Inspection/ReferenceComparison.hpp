#pragma once

#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"

#include <string>
#include <vector>

namespace MatterEngine::Modeling {

// A figura de uma imagem de referencia: o que nao e fundo (o tom dos cantos),
// o maior pedaco ligado.
struct Silhouette {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> mask;
    int top = 0, bottom = -1, left = 0, right = -1;

    [[nodiscard]] bool valid() const { return bottom > top && right > left; }
    [[nodiscard]] bool at(int x, int y) const { return mask[static_cast<std::size_t>(y) * width + x] != 0; }
    // A coluna media da figura entre duas alturas (fracoes de cima para baixo).
    [[nodiscard]] float meanColumn(float fromFraction, float toFraction) const;
    // Largura cheia (pixels) na linha: a soma dos trechos da figura (as duas
    // pernas sem o vao entre elas).
    [[nodiscard]] int rowWidth(int y) const;
    // Do primeiro ao ultimo pixel da figura na linha (com os vaos).
    [[nodiscard]] int rowSpan(int y) const;
};

[[nodiscard]] Silhouette extractSilhouette(const Image& image, float threshold = 0.12f);
// A silhueta da cena (malha) vista pela camera, no tamanho dado.
[[nodiscard]] Silhouette renderSilhouette(const InspectionScene& scene, const OrbitCamera& camera, int width, int height);

struct ReferenceComparison {
    OrbitCamera camera;
    // A referencia com o modelo por cima (contorno e ossos).
    Image overlay;
    // O modelo solido na mesma camera (mesma escala e lugar da referencia):
    // para comparar lado a lado o acabamento (faixas, gola, facetas).
    Image solid;
    // Vermelho: so o modelo; azul: so a referencia; cinza: os dois.
    Image difference;
    bool hasMesh = false;
    float intersectionOverUnion = 0.0f;
    float metersPerPixel = 0.0f;
    // Larguras comparadas por altura e as alturas das juntas na imagem.
    std::vector<std::string> lines;
};

// O modelo na mesma vista (ortografica, de frente/costas/lado) da
// referencia: escalado pela altura (sola ao alto da cabeca) e alinhado
// pelas pernas.
[[nodiscard]] ReferenceComparison compareWithReference(const InspectionScene& scene, const Image& reference,
    const ViewPreset& view, const std::string& title);

} // namespace MatterEngine::Modeling
