#pragma once

#include "Engine/Modeling/Assembly/AssemblyRecipe.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <chrono>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

// O jogador montado das malhas base: o corpo com a cabeca costurada no
// pescoco e a chuteira nos pes. Monta a metade direita e espelha (simetria
// exata; a costura do meio e a mesma dos dois lados).
namespace MatterEngine::Modeling {

struct AssemblyResult {
    bool ok = false;
    std::string error;
    Mesh mesh;
    // O atlas da textura: texels por metro (fator 1) e a fracao coberta.
    float texelsPerMeter = 0.0f;
    float atlasCoverage = 0.0f;
    // O molde da cabeca de referencia (o alinhamento e o quanto andou).
    ReferenceFitReport referenceFit;
    // O tempo de cada etapa (milissegundos), na ordem.
    std::vector<std::pair<std::string, double>> timings;
    // Os arquivos lidos (as malhas base, a chuteira, o molde, o desenho da
    // pintura e o que ele usa): quem mostra o modelo recarrega quando mudam.
    std::vector<std::filesystem::path> files;
};

// Marca o fim de cada etapa da montagem em "timings".
class StageClock {
public:
    explicit StageClock(std::vector<std::pair<std::string, double>>& timings) : m_timings(&timings) {}
    void mark(std::string stage) {
        const auto now = std::chrono::steady_clock::now();
        m_timings->push_back({ std::move(stage), std::chrono::duration<double, std::milli>(now - m_last).count() });
        m_last = now;
    }

private:
    std::vector<std::pair<std::string, double>>* m_timings;
    std::chrono::steady_clock::time_point m_last = std::chrono::steady_clock::now();
};

// "directory": de onde os arquivos da receita sao lidos (o do manifesto). A
// malha sai no espaco do esqueleto (o chao no dele, a chuteira nas juntas do
// pe) e com os pesos do skin (o corpo pelo calor dos ossos, a chuteira no pe
// e nos dedos do pe); com o uniforme (Uniform) se a receita pede.
[[nodiscard]] AssemblyResult assemblePlayer(const AssemblyRecipe& recipe, const std::filesystem::path& directory,
    const Skeleton& skeleton);

// As etapas: a metade direita sem uniforme (aberta no meio) e o espelho
// com a limpeza.
[[nodiscard]] AssemblyResult assembleRightHalf(const AssemblyRecipe& recipe, const std::filesystem::path& directory,
    const Skeleton& skeleton);
// (Perto do meio, a menos de "midlineBlend", os pesos vao para a media com
// o espelho: ver blendMidlineWeights.)
void mirrorHalf(Mesh& mesh, const Skeleton& skeleton, float midlineBlend = 0.0f);

} // namespace MatterEngine::Modeling
