#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"

#include <array>
#include <string>
#include <vector>

// Triangulacao de Delaunay restrita no plano: a base dos remendos da malha
// (o furo onde a trava entra na sola, a pegada de uma tira sobre o cabedal,
// o contorno de um numero no calcao). Pontos em qualquer escala; as contas
// sao em double num espaco normalizado.
namespace MatterEngine::Modeling {

struct Triangulation2DInput {
    std::vector<Point2> points;
    // Voltas fechadas (indices em points): a primeira e a borda de fora do
    // dominio, as outras sao furos. Toda volta vira aresta obrigatoria.
    std::vector<std::vector<int>> loops;
    // Arestas obrigatorias de dentro do dominio (o contorno de uma regiao
    // que muda de material, sem furo).
    std::vector<std::array<int, 2>> segments;
};

struct Triangulation2DResult {
    bool ok = false;
    std::string error;
    // Anti-horarios no plano, so os de dentro do dominio.
    std::vector<std::array<int, 3>> triangles;
    // Quando duas arestas obrigatorias se cruzam: as pontas das duas.
    std::array<int, 4> crossing { -1, -1, -1, -1 };
};

// Delaunay incremental (Bowyer-Watson), as arestas obrigatorias recuperadas
// por trocas de diagonal e o dominio pelas voltas. Pontos repetidos (a menos
// de 1e-9 do tamanho do conjunto) viram um so; um ponto exatamente sobre uma
// aresta obrigatoria a divide. Nao cria pontos novos.
[[nodiscard]] Triangulation2DResult triangulateDomain(const Triangulation2DInput& input);

// Area com sinal (positiva = anti-horario) e ponto dentro de poligono.
[[nodiscard]] double polygonArea2D(const std::vector<Point2>& polygon);
[[nodiscard]] bool pointInPolygon2D(Point2 point, const std::vector<Point2>& polygon);

} // namespace MatterEngine::Modeling
