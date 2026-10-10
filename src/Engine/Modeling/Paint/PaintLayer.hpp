#pragma once

#include "Engine/Modeling/Paint/PaintImage.hpp"
#include "Engine/Modeling/Paint/Shape2D.hpp"
#include "Engine/Modeling/Raster/Color.hpp"

#include <memory>
#include <string>
#include <vector>

// As camadas da pintura: uma forma (PreparedShape) num espaco da superficie,
// a tinta dela (cor, degrade linear ou radial, imagem), o contorno, a opacidade e o
// jeito de misturar com o que esta embaixo (por cima ou multiplicando: a
// sombra de uma cavidade). A cobertura de cada pixel vem da distancia exata
// ate a borda: nitida e lisa.
namespace MatterEngine::Modeling {

struct ColorStop {
    float at = 0.0f;
    Color color;
};

struct Fill {
    enum class Kind { Solid, Linear, Radial, Image };
    Kind kind = Kind::Solid;
    Color color { 0.0f, 0.0f, 0.0f, 1.0f };
    // Os degrades: as cores ao longo de 0 a 1 (em ordem).
    std::vector<ColorStop> stops;
    // Linear: 0 em "from", 1 em "to" (a projecao na reta).
    Point2 from {};
    Point2 to { 1.0f, 0.0f };
    // Radial: 0 no centro, 1 na elipse dos raios (girada "angle" radianos).
    Point2 center {};
    Point2 radius { 1.0f, 1.0f };
    float angle = 0.0f;
    // Imagem: a caixa de "from" (o canto de baixo) a "to" (o de cima), girada
    // "angle" radianos em volta do meio; fora dela, transparente.
    std::shared_ptr<const PaintImage> image;

    // "footprint": o lado do pixel (a imagem le o nivel desse tamanho).
    [[nodiscard]] Color at(Point2 point, float footprint = 0.0f) const;
};

enum class Blend { Normal, Multiply };

// Os lados da superficie em que a camada vale (o direito e y < 0).
enum class Sides { Both, Right, Left };

struct PaintLayer {
    std::string name;
    std::string space;
    PreparedShape shape;
    Fill fill;
    float opacity = 1.0f;
    Blend blend = Blend::Normal;
    // O contorno por fora da forma (largura nas unidades do espaco).
    float outline = 0.0f;
    Color outlineColor { 0.0f, 0.0f, 0.0f, 1.0f };
    Sides sides = Sides::Both;

    [[nodiscard]] bool appliesTo(int side) const { return sides == Sides::Both || (sides == Sides::Right ? side < 0 : side > 0); }
};

// A cor com a camada por cima, no ponto do espaco; "footprint": o lado do
// pixel nas unidades do espaco.
[[nodiscard]] Color paintOver(Color below, const PaintLayer& layer, Point2 point, float footprint);

} // namespace MatterEngine::Modeling
