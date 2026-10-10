#pragma once

#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"

#include <vector>

namespace MatterEngine::Modeling {

// A caixa de uma regiao na cena: as partes da malha dela (na pose) e os
// ossos (com o raio do corpo). A visao geral e o corpo todo.
[[nodiscard]] Box3 regionBounds(const InspectionScene& scene, int region);

// Os pontos (com raio) de uma regiao: juntas, ossos e vertices da malha.
[[nodiscard]] std::vector<FramePoint> regionPoints(const InspectionScene& scene, int region);

// A camera num angulo (graus) enquadrando a caixa.
[[nodiscard]] OrbitCamera frameBounds(const Box3& box, float yawDegrees, float pitchDegrees, float aspect,
    float margin = 1.2f, bool orthographic = false);

// O close-up de uma regiao: o angulo dela (ou o da vista dada) e a distancia
// para ela caber na imagem.
[[nodiscard]] OrbitCamera frameRegion(const InspectionScene& scene, int region, float aspect,
    const ViewPreset* view = nullptr, bool orthographic = false);

} // namespace MatterEngine::Modeling
