#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Mesh/SurfaceFit.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

// O queixo, a mandibula e o pescoco no molde de uma cabeca de referencia
// (densa, de escultor), medidos dela em vez de copiados a olho. A referencia e
// alinhada a nossa cabeca pelo cranio e pela parte de cima do rosto
// (SurfaceFit: Horn e ICP, simetrico) e os vertices abaixo da linha da
// mandibula vao para a superficie dela, ao longo da normal deles. Logo abaixo
// da linha a mudanca cresce aos poucos (o rosto de cima fica como esta); no
// pescoco ela cai em linha reta ate zero na gola, onde ele encaixa no nosso
// corpo (inteira na linha de cima do pescoco, NeckShortening::top: o pescoco
// sai reto da gola ate o molde).
namespace MatterEngine::Modeling {

struct UniformLines;

struct ReferenceFitShape {
    // O arquivo glTF da referencia (relativo a receita); vazio = sem molde.
    std::string file;
    // Os pontos do alinhamento: os vertices da cabeca acima desta altura (do
    // chao) e mais perto do meio que "alignMaxSide" (sem as orelhas).
    float alignAbove = 1.635f;
    float alignMaxSide = 0.07f;
    // A linha da mandibula, de lado: [x, altura do chao], da frente para tras
    // (abaixo dela, o molde; fora das pontas, a ponta).
    std::vector<std::array<float, 2>> jawLine;
    // Abaixo da linha, a mudanca cresce ate inteira nesta distancia.
    float topBlend = 0.015f;
    // A pele logo acima da gola (ate esta altura dela) fica como esta: a gola
    // da camisa encosta nela.
    float collarClearance = 0.01f;
    // A mudanca no pescoco fica inteira nesta fracao do caminho da gola ate a
    // linha de cima do pescoco (1 = so na linha).
    float neckRamp = 1.0f;
    // A maior distancia de um vertice ate a superficie da referencia.
    float maxDistance = 0.05f;
};

struct ReferenceFitReport {
    // O alinhamento: a raiz da media dos quadrados das distancias (m), a
    // escala da referencia e a inclinacao (graus, + para a frente).
    float alignmentRms = 0.0f;
    // Leva a referencia para o espaco do modelo.
    Similarity transform;
    float scale = 1.0f;
    float pitchDegrees = 0.0f;
    std::size_t alignmentPoints = 0;
    // Os vertices que andaram e o maior passo (m); cada um de onde para onde
    // e com que peso (na metade direita).
    std::size_t moved = 0;
    float largestMove = 0.0f;
    struct Move {
        Vec3 from;
        Vec3 to;
        float weight = 0.0f;
    };
    std::vector<Move> moves;
};

// Na metade direita, depois do pescoco costurado (as partes "cabeca" e o
// corpo; "headPart" e a da cabeca).
bool fitToReference(Mesh& rightHalf, const ReferenceFitShape& shape, const std::filesystem::path& directory, const UniformLines& lines,
    int headPart, ReferenceFitReport& report, std::string& error);

} // namespace MatterEngine::Modeling
