#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <array>
#include <string>
#include <string_view>

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

} // namespace MatterEngine::Modeling
