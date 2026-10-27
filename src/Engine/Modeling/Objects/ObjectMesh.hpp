#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <array>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// As formas dos objetos procedurais na malha da modelagem, ja com o mapa de
// textura (um retangulo do atlas por face). Fechadas e orientadas para fora:
// passam nas mesmas analises do jogador (topologia, avesso).
namespace MatterEngine::Modeling {

// Um retangulo do atlas: u para a direita e v para baixo, vistos de fora.
struct UvRect {
    float u0 = 0.0f;
    float v0 = 0.0f;
    float u1 = 1.0f;
    float v1 = 1.0f;
};

// As 6 faces de um bloco: +X, -X, +Y, -Y, +Z, -Z. Nos lados o "em cima" do
// retangulo e o +z; em cima e embaixo, o +y. inset: quanto o mapa entra no
// retangulo (o meio texel que impede a amostra de puxar o vizinho).
struct BoxFaceUvs {
    std::array<UvRect, 6> faces {};
    float inset = 0.0f;
};

// Um bloco centrado na origem (os 8 cantos divididos pelas faces; o mapa por
// canto de triangulo), na parte (raiz) e no material dados (branco, sem
// textura: quem chama liga a dela).
bool buildTexturedBox(Mesh& mesh, Vec3 size, const BoxFaceUvs& uvs, std::string_view part, std::string_view material,
    std::string& error);

// Um tubo redondo pelo caminho (os pontos do eixo), com "sides" lados, na
// parte (raiz) e no material dados. Cada canto do caminho tem o raio da dobra
// dele (bendRadii, um por ponto do meio; vazio = todos 0): 0 e o canto
// soldado em meia-esquadria (o corte exato na bissetriz), mais que 0 a dobra
// em arco. As pontas fecham com tampas (malha fechada, para fora). Devolve o
// eixo final (com os arcos) em "axis" se pedido.
bool buildTube(Mesh& mesh, std::span<const Vec3> path, float radius, int sides, std::span<const float> bendRadii,
    std::string_view part, std::string_view material, std::string& error, std::vector<Vec3>* axis = nullptr);

} // namespace MatterEngine::Modeling
