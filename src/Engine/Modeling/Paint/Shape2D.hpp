#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"

#include <cstdint>
#include <limits>
#include <vector>

// As formas da pintura em 2D (vetoriais): contornos fechados em linha reta
// (curvas viram segmentos curtos), cheios pela regra das voltas (por dentro
// de um contorno no sentido anti-horario conta +1, no horario -1; dentro e
// soma diferente de zero - os furos sao contornos no sentido contrario do de
// fora). A distancia com sinal ate a borda e exata, e a cobertura de um
// pixel sai dela e do tamanho do pixel: borda nitida e lisa em qualquer
// resolucao, sem depender de onde a malha tem vertices.
namespace MatterEngine::Modeling {

struct Box2 {
    Point2 minimum { std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    Point2 maximum { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };

    void include(Point2 point);
    void include(const Box2& other);
    [[nodiscard]] bool valid() const { return minimum.x <= maximum.x && minimum.y <= maximum.y; }
    // A distancia de fora ate a caixa (0 dentro).
    [[nodiscard]] float distanceOutside(Point2 point) const;
};

// x' = a x + b y + tx, y' = c x + d y + ty.
struct Affine2 {
    float a = 1.0f;
    float b = 0.0f;
    float c = 0.0f;
    float d = 1.0f;
    float tx = 0.0f;
    float ty = 0.0f;

    [[nodiscard]] Point2 apply(Point2 point) const { return { a * point.x + b * point.y + tx, c * point.x + d * point.y + ty }; }
    // Primeiro esta, depois "next".
    [[nodiscard]] Affine2 then(const Affine2& next) const;
    // O quanto a area muda (negativo: espelha).
    [[nodiscard]] float determinant() const { return a * d - b * c; }

    [[nodiscard]] static Affine2 translation(float x, float y);
    [[nodiscard]] static Affine2 scaling(float sx, float sy);
    [[nodiscard]] static Affine2 rotation(float radians);
};

struct Shape2D {
    std::vector<std::vector<Point2>> contours;

    [[nodiscard]] bool empty() const { return contours.empty(); }
    [[nodiscard]] Box2 bounds() const;
    // A soma das voltas em volta do ponto (dentro: diferente de zero).
    [[nodiscard]] int winding(Point2 point) const;
    [[nodiscard]] bool contains(Point2 point) const { return winding(point) != 0; }

    // Junta os contornos (a uniao, pela regra das voltas).
    void add(const Shape2D& other);
    // Os contornos levados pela transformacao; uma que espelha tem os
    // contornos virados de volta (os furos continuam furos).
    [[nodiscard]] Shape2D transformed(const Affine2& transform) const;
};

// A area com sinal de um contorno (positiva no sentido anti-horario).
[[nodiscard]] float contourArea(const std::vector<Point2>& contour);

// As formas basicas (contornos anti-horarios).
[[nodiscard]] Shape2D polygonShape(std::vector<Point2> points);
// O retangulo com os cantos chanfrados a 45 graus ("chamfer") ou
// arredondados ("round"), nas unidades da forma.
[[nodiscard]] Shape2D rectangleShape(Point2 minimum, Point2 maximum, float chamfer = 0.0f, float round = 0.0f);
[[nodiscard]] Shape2D ellipseShape(Point2 center, Point2 radii, int segments = 64);
// Uma linha grossa ao longo dos pontos (pontas retas, juntas em quina com
// limite); fechada, um anel (dois contornos).
[[nodiscard]] Shape2D strokeShape(const std::vector<Point2>& points, float width, bool closed = false);

// A forma pronta para consultas rapidas: so as arestas que sao borda de
// verdade da uniao (a parte de uma aresta por dentro de outro contorno nao
// conta: duas formas sobrepostas na mesma camada nao mostram costura) e as
// arestas por faixas de altura (texto com centenas de arestas fica rapido).
class PreparedShape {
public:
    PreparedShape() = default;
    explicit PreparedShape(const Shape2D& shape);

    [[nodiscard]] bool empty() const { return m_segments.empty(); }
    [[nodiscard]] const Box2& bounds() const { return m_bounds; }
    [[nodiscard]] bool contains(Point2 point) const;
    // A distancia ate a borda (sem sinal); "limit": nao procura alem disso
    // (devolve o limite).
    [[nodiscard]] float edgeDistance(Point2 point, float limit = std::numeric_limits<float>::max()) const;
    // Negativa dentro.
    [[nodiscard]] float signedDistance(Point2 point) const;
    // A fracao coberta de um pixel de lado "footprint" centrado no ponto (0 a
    // 1; footprint 0: dentro ou fora). "grow" engorda a forma (para fora,
    // positivo): o contorno de uma letra.
    [[nodiscard]] float coverage(Point2 point, float footprint, float grow = 0.0f) const;

private:
    struct Segment {
        Point2 a;
        Point2 b;
    };
    // Todas as arestas (para as voltas) e as da borda (para a distancia).
    std::vector<Segment> m_edges;
    std::vector<Segment> m_segments;
    Box2 m_bounds;
    float m_bandHeight = 1.0f;
    std::vector<std::vector<std::uint32_t>> m_edgeBands;
    std::vector<std::vector<std::uint32_t>> m_segmentBands;

    [[nodiscard]] int band(float y) const;
};

} // namespace MatterEngine::Modeling
