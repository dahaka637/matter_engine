#pragma once

#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Math/Mat4.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

// Bola de futebol procedural: o desenho classico de 32 gomos (icosaedro
// truncado, 12 pentagonos pretos e 20 hexagonos brancos), cada gomo uma
// almofada costurada - levemente estufado no meio, com a borda rolando para
// dentro de um sulco de costura estreito. Cor por vertice e gomos com
// vertices proprios: a divisa preto/branco fica nitida em qualquer distancia.
//
// A malha e ligada a uma "gaiola" de pontos sobre a esfera (icosfera de 2562
// pontos), um osso por ponto, com pesos baricentricos do triangulo da gaiola
// que contem cada vertice. A deformacao do contato e escrita nas matrizes
// desses ossos e desenhada pelo skinning comum da GPU; sem contato a bola e
// desenhada rigida, sem matriz nenhuma.
struct SoccerBallStyle3D {
    // Raio da esfera de colisao (as superficies dos gomos ficam nele).
    float radiusMeters = 0.1098f;
    Vec3 whiteColor { 0.92f, 0.92f, 0.90f };
    Vec3 blackColor { 0.02f, 0.02f, 0.022f };
    // Meia largura e profundidade do sulco da costura.
    float seamHalfWidthMeters = 0.0017f;
    float seamDepthMeters = 0.0011f;
    // Quanto o meio do gomo estufa e em que distancia da costura.
    float panelPuffMeters = 0.0005f;
    float panelPuffReachMeters = 0.008f;
    // Escurecimento do fundo da costura (oclusao) nos gomos brancos.
    float seamShadow = 0.45f;
    // Segmentos por aresta de gomo (multiplo de 4: os niveis de detalhe usam
    // metade e um quarto).
    std::uint32_t edgeSegments = 12;
    // Gaiola: icosfera de 2562 pontos (~8 mm entre ossos). Com 642 a borda
    // da mancha de um chute forte ondula entre os ossos.
    std::uint32_t cageSubdivisions = 4;
};

struct SoccerBallMesh3D {
    MeshData3D mesh;
    // Niveis de detalhe sobre os mesmos vertices (menos aneis e metade/um
    // quarto dos segmentos). O ultimo serve tambem para as sombras.
    std::vector<std::vector<std::uint32_t>> lods;
    // Direcoes de repouso dos ossos da gaiola (unitarias, no espaco da bola).
    std::vector<Vec3> cageDirections;
    float radiusMeters = 0.0f;
};

[[nodiscard]] SoccerBallMesh3D buildProceduralSoccerBall3D(
    const SoccerBallStyle3D& style = {});

// Plano contra o qual a esfera de repouso e achatada: um ponto dele e a
// normal que sai do outro corpo para a bola.
struct SoccerBallContactPlane3D {
    Vec3 pointWorld;
    Vec3 normalWorld { 0.0f, 0.0f, 1.0f };
};

// Achatamento visivel minimo (m): abaixo disso a bola e desenhada rigida.
inline constexpr float SoccerBallVisibleCompressionMeters = 0.00025f;

// Compressao da esfera de repouso centrada em center contra o plano (m).
[[nodiscard]] float soccerBallPlaneCompression3D(Vec3 center, float radiusMeters,
    const SoccerBallContactPlane3D& plane);

// Escreve as matrizes dos ossos da gaiola (espaco do mundo, como o skinning
// espera) para a bola em center/orientation achatada contra os planos.
// Cada ponto abaixo de um plano vai para ele; perto da borda da mancha uma
// curva suave (a casca nao dobra em quina) e uma leve saliencia. A normal de
// cada osso gira junto, para a mancha achatada sombrear como plana.
void buildSoccerBallSkin3D(std::span<const Vec3> cageDirections, float radiusMeters,
    Vec3 center, Quaternion orientation,
    std::span<const SoccerBallContactPlane3D> planes, std::vector<Mat4>& outMatrices);

} // namespace MatterEngine
