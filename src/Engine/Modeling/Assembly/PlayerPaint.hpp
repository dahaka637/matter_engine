#pragma once

#include "Engine/Modeling/Assembly/AssemblyRecipe.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Paint/PaintDesign.hpp"
#include "Engine/Modeling/Paint/PaintSpace.hpp"
#include "Engine/Modeling/Parts/BootRecipe.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"
#include "Engine/Modeling/Texture/TextureBake.hpp"

#include <map>
#include <memory>
#include <string>

// A pintura do jogador montado (o bake leva para o atlas), como funcao do
// ponto na pose de ligacao: a cor de base de cada parte, o rosto (os olhos,
// as sobrancelhas e a boca, FacePaint), a chuteira (o cadarco) e por cima as
// camadas do desenho (pintura.json: o uniforme, o meiao, a orelha), cada uma
// no espaco dela (PlayerPaintSpaces).
namespace MatterEngine::Modeling {

// A chuteira direita para o cadarco pintado: a receita (o desenho do
// cadarco), o referencial dela (BootPlacement::frame) e a escala sobre o
// calcanhar.
struct BootPaint {
    BootRecipe recipe;
    Transform frame;
    float scale = 1.0f;
};

// O desenho e os espacos dele (vazio: so as cores de base).
struct PlayerPaintDesign {
    std::shared_ptr<const PaintDesign> design;
    std::shared_ptr<const std::map<std::string, PaintSpace>> spaces;
};

[[nodiscard]] SurfacePaint playerPaint(const Mesh& mesh, const AssemblyRecipe& recipe, const Skeleton& skeleton, const PlayerPaintDesign& design,
    const BootPaint* boot = nullptr);

} // namespace MatterEngine::Modeling
