#pragma once

#include "Engine/Materials/MaterialTypes.hpp"

#include <string>
#include <unordered_map>

namespace MatterEngine {

// Catalogo canonico indexado pelo ID autorado em Blender. IDs desconhecidos
// nao sao inventados durante a importacao: um typo precisa falhar cedo para
// nao contaminar massa, atrito e audio com valores genericos.
class MaterialLibrary {
public:
    MaterialLibrary();

    // Registra uma definição completa. IDs vazios e valores fisicamente
    // inválidos são rejeitados em vez de contaminarem silenciosamente a cena.
    SurfaceMaterial& registerMaterial(SurfaceMaterial material);
    [[nodiscard]] const SurfaceMaterial* find(const std::string& id) const;
    [[nodiscard]] const std::unordered_map<std::string, SurfaceMaterial>& all() const {
        return m_materials;
    }

private:
    void registerStandardMaterials();
    std::unordered_map<std::string, SurfaceMaterial> m_materials;
};

} // namespace MatterEngine
