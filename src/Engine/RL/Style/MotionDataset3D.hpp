#pragma once

#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Style/StyleFeatures3D.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace MatterEngine {

// O acervo do juiz de estilo (plano_claude_rl.md secao 6.2,
// assets/rl/style/<id>.json): os clipes tocados no corpo do RL a 120 Hz, em
// pares de instantes separados por uma decisao, com o peso por grupo (um
// parado longo nao afoga o resto) e os reservados fora do treino (a
// avaliacao: o juiz congelado precisa dar nota alta a eles).
//
// Com "mirror" no arquivo, cada clipe ganha a copia espelhada (o lado
// esquerdo vira o direito, pelo Mirror3D): os pares espelhados vem logo depois
// dos do clipe, no mesmo bloco, e dividem o peso do grupo.

struct MotionGroup3D {
    std::string id;
    std::string displayName;
    float weight = 1.0f;
};

struct MotionClip3D {
    std::string id;
    std::uint32_t group = 0;
    std::filesystem::path path;
    bool loops = false;
    bool reserved = false;
    std::uint32_t frames = 0;
    // Os pares dele em MotionDataset3D (o primeiro e quantos).
    std::uint32_t firstPair = 0;
    std::uint32_t pairCount = 0;
    float meanSpeed = 0.0f;
};

struct MotionDataset3D {
    std::string id;
    std::filesystem::path definitionPath;
    // FNV-1a de 64 bits da definicao e dos clipes.
    std::uint64_t hash = 0;
    // Os quadros (1/120 s) entre os dois instantes de um par: uma decisao.
    std::uint32_t pairFrames = 4;
    bool mirror = false;
    std::size_t featureSize = 0;
    std::vector<MotionGroup3D> groups;
    std::vector<MotionClip3D> clips;
    // Os pares: pairCount() x (2 x featureSize), o instante e o seguinte.
    std::vector<float> pairs;
    std::vector<std::uint32_t> pairClip;
    // O par e a copia espelhada (1) de um do clipe.
    std::vector<std::uint8_t> pairMirrored;
    // O peso de amostra de cada par: o do grupo dividido pelos pares de treino
    // dele (soma 1 nos de treino; os reservados ficam com 0).
    std::vector<float> pairWeight;

    [[nodiscard]] std::size_t pairCount() const { return pairClip.size(); }
};

// So o catalogo: os grupos e os clipes (com os reservados), sem tocar os
// clipes no corpo nem montar os pares (os comecos pelo acervo usam).
[[nodiscard]] MotionDataset3D loadMotionCatalog3D(const std::filesystem::path& definitionPath);

// heightScale: a altura do jogador sobre a nominal (o acervo na escala dele).
[[nodiscard]] MotionDataset3D loadMotionDataset3D(const RLBody3D& body, const StyleFeatures3D& features,
    const std::filesystem::path& definitionPath, float heightScale = 1.0f);

} // namespace MatterEngine
