#pragma once

#include "Engine/Appearance/Surface/SurfaceTemplate.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

// A malha do jogador modelada no Blender (22/10/2026): o JSON de troca que
// tools/blender/exportar_troca.py grava do .blend (MatterModelInspector
// exportar-blender faz a ida). A malha, as partes (com as costuras e aberturas
// declaradas), os materiais, o mapa por canto e as texturas; os pesos por
// junta, casados pelo nome com o esqueleto do jogo (uma junta que nao existe e
// erro). No espaco do modelo: x frente, y esquerda, z cima. Com o template de
// superficie (tools/blender/superficie.py, 26/10), ele vem junto, conferido
// contra os triangulos da malha (a mesma ordem).
namespace MatterEngine::Modeling {

struct BlenderExchange {
    Mesh mesh;
    // Vazio quando a troca nao tem o template.
    std::shared_ptr<const Appearance::SurfaceTemplate> surface;
};

[[nodiscard]] std::optional<BlenderExchange> loadBlenderExchange(const std::filesystem::path& path, const Skeleton& skeleton,
    std::string* error = nullptr);

} // namespace MatterEngine::Modeling
