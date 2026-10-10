#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <string>

// O olho aberto numa cabeca pronta (como na referencia de mudanca dos olhos):
// uma placa hexagonal no plano da orbita com a abertura em amendoa, a
// palpebra de cima em faixa saliente, a de baixo mais fina sobre o plano sob
// o olho e o globo um pouco recuado e facetado. Os triangulos da orbita por
// baixo da placa sao refeitos em volta dela (triangulacao restrita, sem
// lascas); o olho continua da volta, vertice com vertice. So o olho direito
// (y < 0): o espelho faz o outro.
namespace MatterEngine::Modeling {

struct EyeShape {
    // O centro da abertura visto de frente (o y e a altura; o x vem da
    // pele) e o olhar: o giro para fora e para baixo (graus).
    float centerY = -0.0375f;
    float centerZ = 1.6955f;
    float yaw = 14.0f;
    float pitch = 4.0f;
    // A abertura (largura e altura) e quanto o canto de fora sobe e o de
    // dentro desce (fracao da altura).
    float width = 0.04f;
    float height = 0.0135f;
    float tilt = 0.06f;
    // A placa do olho em volta da abertura (hexagono, como na referencia): o
    // alto acima dela, a base abaixo (o plano sob o olho) e os cantos alem
    // dos cantos dela.
    float marginUpper = 0.0042f;
    float marginLower = 0.0075f;
    float cornerMargin = 0.0012f;
    // As palpebras: o quanto a crista salta da pele e a distancia dela a
    // abertura (a de cima, uma faixa grossa ate perto do alto da placa; a de
    // baixo, fina).
    float lidUpper = 0.003f;
    float lidLower = 0.0012f;
    float lidBandUpper = 0.0032f;
    float lidBandLower = 0.0016f;
    // O globo: o quanto ele fica atras da borda da abertura e o quanto o
    // meio dele salta.
    float recess = 0.0016f;
    float bulge = 0.0022f;
};

struct EyeSocketResult {
    bool ok = false;
    std::string error;
};

// Abre o olho direito nos triangulos da parte dada (a pele da orbita);
// o olho fica na mesma parte, com os materiais "pele" e "olho".
[[nodiscard]] EyeSocketResult buildEyeSocket(Mesh& mesh, const EyeShape& eye, int part);

} // namespace MatterEngine::Modeling
