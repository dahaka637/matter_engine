#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"
#include "Engine/Modeling/Parts/ProfileCurve.hpp"

#include <cstdint>
#include <optional>
#include <vector>

// A forma de uma peca como superficie implicita: primitivas simples unidas
// com mistura suave (a axila, o ombro e a base do pescoco saem da mistura).
// O valor aproxima a distancia ate a superficie (metros, negativo dentro).
// A peca continua dona da topologia (aneis, paineis, costuras): ela so
// projeta os seus vertices aqui para pegar a forma.
namespace MatterEngine::Modeling {

// Pilha de secoes horizontais por altura (metros do chao): superelipses de
// quatro quadrantes em volta de (centerX, 0), ate front/back no x a partir do
// centro e ate side no y (ver SuperSection). Fora da faixa de alturas a
// secao da ponta segue reta (um cilindro).
struct SectionStack {
    ProfileCurve centerX;
    ProfileCurve front;
    ProfileCurve back;
    ProfileCurve side;
    ProfileCurve exponent;
    float ground = 0.0f;
    // O expoente do termo do x (lados mais fechados quanto menor); vazio =
    // o mesmo exponent.
    ProfileCurve sideExponent;
};

// Duas esferas e o casco entre elas (a manga, o braco).
struct RoundCone {
    Vec3 a;
    Vec3 b;
    float radiusA = 0.0f;
    float radiusB = 0.0f;
};

// Uma corrente de esferas (centro e raio) com o casco entre cada par
// seguido: um tubo de raio variavel ao longo de uma linha (a manga).
struct ConeChain {
    std::vector<Vec3> centers;
    std::vector<float> radii;
};

// Elipsoide com os raios nos eixos do referencial.
struct FieldEllipsoid {
    Transform frame;
    Vec3 radii;
};

class ShapeField {
public:
    // Cada primitiva entra na uniao com o seu raio de mistura (metros; 0 =
    // uniao seca). A primeira nao mistura com nada.
    void add(SectionStack stack, float blend = 0.0f);
    void add(RoundCone cone, float blend = 0.0f);
    void add(ConeChain chain, float blend = 0.0f);
    void add(FieldEllipsoid ellipsoid, float blend = 0.0f);
    [[nodiscard]] bool empty() const { return m_primitives.empty(); }

    [[nodiscard]] float value(Vec3 point) const;
    [[nodiscard]] Vec3 gradient(Vec3 point) const;
    // A normal para fora (o gradiente normalizado).
    [[nodiscard]] Vec3 normal(Vec3 point) const;
    // O ponto da superficie mais perto (Newton pelo gradiente).
    [[nodiscard]] Vec3 project(Vec3 point, int iterations = 24) const;
    // A primeira passagem pela superficie no raio origin + t * direction
    // (t de 0 a length), de fora para dentro ou de dentro para fora.
    [[nodiscard]] std::optional<Vec3> cross(Vec3 origin, Vec3 direction, float length, float step = 0.002f) const;

private:
    enum class Kind : std::uint8_t { Stack, Cone, Chain, Ellipsoid };
    struct Primitive {
        Kind kind = Kind::Stack;
        std::size_t index = 0;
        float blend = 0.0f;
    };
    [[nodiscard]] float primitiveValue(const Primitive& primitive, Vec3 point) const;

    std::vector<Primitive> m_primitives;
    std::vector<SectionStack> m_stacks;
    std::vector<RoundCone> m_cones;
    std::vector<ConeChain> m_chains;
    std::vector<FieldEllipsoid> m_ellipsoids;
};

// O minimo suave (polinomial): igual ao minimo longe da mistura.
[[nodiscard]] float smoothMinimum(float a, float b, float blend);

} // namespace MatterEngine::Modeling
