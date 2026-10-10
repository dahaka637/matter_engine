#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"

#include <optional>
#include <span>
#include <vector>

// Encaixar uma malha na forma de outra, medindo em vez de copiar a olho: uma
// referencia pronta e densa (a cabeca de um escultor) e a nossa, leve. O
// alinhamento acha a semelhanca (escala, giro e translacao) que poe a
// referencia sobre uma regiao nossa: minimos quadrados pelo metodo de Horn,
// repetido por ICP (cada ponto nosso com o ponto mais perto da referencia). A
// projecao leva cada vertice para a superficie da referencia ao longo da
// normal dele. Sem vertice novo: a forma da referencia nos vertices que ja
// existem.
namespace MatterEngine::Modeling {

struct Similarity {
    float scale = 1.0f;
    Quaternion rotation;
    Vec3 translation;

    [[nodiscard]] Vec3 apply(Vec3 point) const { return rotation.rotate(point) * scale + translation; }
    [[nodiscard]] Similarity inverse() const;
};

// A semelhanca que leva cada source[i] o mais perto possivel de target[i]
// (minimos quadrados); sem "allowScale", escala 1. Nada com menos de 3 pontos
// ou com todos no mesmo lugar.
[[nodiscard]] std::optional<Similarity> fitSimilarity(std::span<const Vec3> source, std::span<const Vec3> target,
    bool allowScale = true);

struct AlignSettings {
    int iterations = 80;
    // Os piores pares (fracao) ficam de fora de cada volta: as partes que a
    // referencia nao tem iguais (a orelha, o nariz de outro jeito).
    float trim = 0.15f;
    bool allowScale = true;
    // As duas malhas simetricas no plano do meio (y = 0): o giro fica so em
    // volta do y e a translacao sem y.
    bool symmetric = true;
};

struct AlignResult {
    bool ok = false;
    // Leva a referencia para o espaco dos pontos.
    Similarity transform;
    // A raiz da media dos quadrados das distancias dos pontos (os que ficaram)
    // a referencia alinhada.
    float rms = 0.0f;
    int iterations = 0;
};

// O alinhamento por ICP a partir de "initial" (que ja leva a referencia para
// perto dos pontos).
[[nodiscard]] AlignResult alignReference(std::span<const Vec3> referencePositions, std::span<const Triangle> referenceTriangles,
    std::span<const Vec3> points, const Similarity& initial, const AlignSettings& settings = {});

// As normais dos vertices (a media dos triangulos em volta, pesada pela area).
[[nodiscard]] std::vector<Vec3> vertexNormals(const Mesh& mesh);

// O ponto da superficie na reta pelo ponto na direcao dada (os dois
// sentidos, o mais perto, ate maxDistance); sem acerto, o ponto mais perto da
// superficie (ate maxDistance); nada se longe demais.
[[nodiscard]] std::optional<Vec3> projectOntoSurface(const TriangleBvh& surface, Vec3 point, Vec3 direction, float maxDistance);

} // namespace MatterEngine::Modeling
