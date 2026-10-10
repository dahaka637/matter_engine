#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <filesystem>
#include <string>

// Malhas prontas (glTF 2.0, .glb ou .gltf) trazidas para o espaco da
// modelagem: as malhas base do corpo e da cabeca. So a forma (posicoes e
// triangulos, com as transformacoes dos nos aplicadas); vertices no mesmo
// lugar (as costuras de UV e de normal do arquivo) viram um so.
namespace MatterEngine::Modeling {

struct GltfImportSettings {
    // Para onde o modelo olha no espaco do glTF (o glTF tem +Y para cima):
    // vira o +x do modelo; o +Y vira o +z e o lado esquerdo do modelo, o +y.
    Vec3 forward { 0.0f, 0.0f, 1.0f };
    // Vertices mais perto que isso (fracao do tamanho da malha) viram um so.
    float weldTolerance = 1e-6f;
    // O nome da parte e do material dos triangulos importados.
    std::string part = "importado";
    std::string material = "importado";
};

struct GltfImportResult {
    bool ok = false;
    std::string error;
    Mesh mesh;
};

[[nodiscard]] GltfImportResult importGltfMesh(const std::filesystem::path& path, const GltfImportSettings& settings = {});

} // namespace MatterEngine::Modeling
