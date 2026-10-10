#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <string>
#include <utility>
#include <vector>

// O mapa de textura automatico de uma malha facetada: os triangulos viram
// ilhas (vizinhos da mesma parte com a normal perto da media da ilha), cada
// ilha e projetada no plano dela (um metro e o mesmo numero de texels em toda
// parte, fora as partes com densidade extra) e as ilhas sao arrumadas pelo
// formato delas (IslandPacking) num atlas quadrado, com folga entre elas, na
// maior escala que cabe. A pintura vem depois, pelo bake (TextureBake): o
// desenho e uma funcao do ponto 3D, o atlas so guarda.
namespace MatterEngine::Modeling {

struct UvAtlasSettings {
    int size = 2048;
    // Texels livres em volta de cada ilha (a sangria do bake cabe nela).
    int padding = 4;
    // Graus entre a normal de um triangulo e a media da ilha dele.
    float chartAngle = 60.0f;
    // Mais texels por metro em algumas partes (o rosto): o nome e o fator.
    std::vector<std::pair<std::string, float>> partDensity;
};

struct UvAtlasResult {
    bool ok = false;
    std::string error;
    std::size_t charts = 0;
    // Texels por metro (fator 1) e a fracao do atlas coberta pelas ilhas.
    float texelsPerMeter = 0.0f;
    float coverage = 0.0f;
};

// Preenche mesh.triangleUv (substitui o que houver).
[[nodiscard]] UvAtlasResult unwrapAtlas(Mesh& mesh, const UvAtlasSettings& settings = {});

} // namespace MatterEngine::Modeling
