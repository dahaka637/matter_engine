#pragma once

#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"

#include <span>
#include <string>
#include <string_view>

namespace MatterEngine::Modeling {

struct SheetPanel {
    OrbitCamera camera;
    InspectionStyle style;
    std::string caption;
};

// Varias vistas da mesma cena numa imagem so (grade com legendas): ver o
// modelo de todos os lados numa leitura.
[[nodiscard]] Image renderSheet(const InspectionScene& scene, std::span<const SheetPanel> panels, int columns,
    int panelWidth, int panelHeight, std::string_view title);

// Junta imagens prontas numa grade com legendas.
[[nodiscard]] Image composeSheet(std::span<const Image> images, std::span<const std::string> captions, int columns,
    std::string_view title);

} // namespace MatterEngine::Modeling
