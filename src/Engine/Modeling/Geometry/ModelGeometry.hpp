#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <limits>

// Geometria basica da modelagem do personagem: caixas, transformacoes rigidas,
// planos e consultas de distancia. O espaco e o do modelo do jogo: z para
// cima, x para a frente e y para a esquerda do personagem (metros).
namespace MatterEngine::Modeling {

inline constexpr float Pi = 3.14159265358979323846f;
inline constexpr float DegreesToRadians = Pi / 180.0f;
inline constexpr float RadiansToDegrees = 180.0f / Pi;

struct Point2 {
    float x = 0.0f;
    float y = 0.0f;
};

[[nodiscard]] Vec3 lerp(Vec3 a, Vec3 b, float t);
[[nodiscard]] Vec3 componentMin(Vec3 a, Vec3 b);
[[nodiscard]] Vec3 componentMax(Vec3 a, Vec3 b);
// Normalizado, ou o reserva quando o vetor e quase nulo.
[[nodiscard]] Vec3 safeNormalized(Vec3 vector, Vec3 fallback);
// Um vetor unitario qualquer perpendicular a este (para montar bases).
[[nodiscard]] Vec3 anyPerpendicular(Vec3 vector);
[[nodiscard]] float distance(Vec3 a, Vec3 b);

struct Box3 {
    Vec3 minimum { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max() };
    Vec3 maximum { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max() };

    void include(Vec3 point);
    void include(const Box3& box);
    // Inclui uma esfera (o ponto e o raio em volta).
    void include(Vec3 center, float radius);
    [[nodiscard]] bool valid() const;
    [[nodiscard]] Vec3 center() const;
    [[nodiscard]] Vec3 size() const;
    [[nodiscard]] float radius() const;
    [[nodiscard]] Box3 expanded(float margin) const;
    [[nodiscard]] bool overlaps(const Box3& other) const;
    [[nodiscard]] bool contains(Vec3 point) const;
};

// Transformacao rigida (rotacao e translacao): o referencial de uma junta.
struct Transform {
    Vec3 position;
    Quaternion orientation;

    [[nodiscard]] Vec3 apply(Vec3 point) const;
    [[nodiscard]] Vec3 applyVector(Vec3 vector) const;
    [[nodiscard]] Transform inverse() const;
    // Esta seguida da filha: (this * child).apply(p) = apply(child.apply(p)).
    [[nodiscard]] Transform operator*(const Transform& child) const;
};

struct Plane {
    Vec3 normal { 0.0f, 0.0f, 1.0f };
    float offset = 0.0f;

    [[nodiscard]] static Plane fromPointNormal(Vec3 point, Vec3 normal);
    [[nodiscard]] float signedDistance(Vec3 point) const;
};

// O quaternion da base ortonormal (os eixos x, y e z do referencial).
[[nodiscard]] Quaternion quaternionFromBasis(Vec3 xAxis, Vec3 yAxis, Vec3 zAxis);
// O referencial com x no eixo dado e z o mais perto possivel do "cima".
[[nodiscard]] Quaternion frameAlong(Vec3 xAxis, Vec3 upHint);
[[nodiscard]] Vec3 axisX(Quaternion orientation);
[[nodiscard]] Vec3 axisY(Quaternion orientation);
[[nodiscard]] Vec3 axisZ(Quaternion orientation);
// O angulo (rad) entre duas orientacoes.
[[nodiscard]] float angleBetween(Quaternion a, Quaternion b);
// Uma fracao do giro q: o mesmo eixo, o angulo vezes "fraction" (pelo
// caminho mais curto).
[[nodiscard]] Quaternion partialRotation(Quaternion q, float fraction);
// A parte do giro q em volta de um eixo (a decomposicao balanco e torcao): o
// giro so nesse eixo, sem o resto. Sem eixo, o giro inteiro; giro todo fora
// do eixo (meia volta), nenhum.
[[nodiscard]] Quaternion twistAbout(Quaternion q, Vec3 axis);

// Distancia de um ponto ao segmento; t recebe o parametro do mais proximo.
[[nodiscard]] float pointSegmentDistance(Vec3 point, Vec3 a, Vec3 b, float* t = nullptr);
[[nodiscard]] Vec3 closestPointOnTriangle(Vec3 point, Vec3 a, Vec3 b, Vec3 c);
[[nodiscard]] Vec3 triangleNormal(Vec3 a, Vec3 b, Vec3 c);
[[nodiscard]] float triangleArea(Vec3 a, Vec3 b, Vec3 c);
// Se dois triangulos se cruzam (Moller); coplanares contam como nao.
[[nodiscard]] bool trianglesIntersect(Vec3 a0, Vec3 a1, Vec3 a2, Vec3 b0, Vec3 b1, Vec3 b2);

} // namespace MatterEngine::Modeling
