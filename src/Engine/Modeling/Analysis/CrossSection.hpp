#pragma once

#include "Engine/Modeling/Analysis/Report.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/BodyRegions.hpp"

#include <span>
#include <vector>

namespace MatterEngine::Modeling {

struct SectionLoop {
    std::vector<Vec3> points;
    bool closed = false;
    float length = 0.0f;
    Vec3 centroid;
};

struct MeshSection {
    Plane plane;
    std::vector<SectionLoop> loops;
    [[nodiscard]] float totalLength() const;
};

// O corte da malha por um plano: os contornos (fechados quando a malha e).
// partFilter vazio = todas as partes.
[[nodiscard]] MeshSection sliceMesh(const Mesh& mesh, std::span<const Vec3> positions, const Plane& plane,
    std::span<const int> partFilter = {});
// Se o ponto (no plano) esta dentro do contorno.
[[nodiscard]] bool loopContains(const SectionLoop& loop, const Plane& plane, Vec3 point);

struct BoneSection {
    bool found = false;
    // O ponto no osso e o contorno mais interno que o envolve.
    Vec3 point;
    SectionLoop loop;
    float circumference = 0.0f;
    // Largura e profundidade do contorno nos eixos do referencial da junta.
    float width = 0.0f;
    float depth = 0.0f;
    // Distancia do osso ao centro do contorno.
    float offset = 0.0f;
    bool inside = false;
};

// O corte perpendicular ao osso que sai da junta, a "t" do caminho ate a
// filha (0 = na junta). O contorno escolhido e o mais interno que envolve o
// osso (a pele, nao a manga por cima).
[[nodiscard]] BoneSection sectionAcrossBone(const Mesh& mesh, std::span<const Vec3> positions,
    const Skeleton& skeleton, int joint, float t, std::span<const int> partFilter = {});
// A filha que segue o membro (o dedo medio na mao; a primeira que nao e ponta).
[[nodiscard]] int limbChild(const Skeleton& skeleton, int joint);

// Cada junta com malha na regiao dela deve ficar dentro do corpo e perto
// do centro do corte.
void checkJointCentering(const Mesh& mesh, const Skeleton& skeleton, const BodyRegions* regions,
    std::span<const int> jointRegion, Report& report);

} // namespace MatterEngine::Modeling
