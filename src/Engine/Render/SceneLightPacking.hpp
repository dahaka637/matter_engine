#pragma once

#include "Engine/Render/Scene3D.hpp"

#include <array>
#include <span>
#include <vector>

namespace MatterEngine {

// Layout GPU (std430) de uma luz - ver o SSBO "SceneLights" declarado em
// scene3d_mesh.frag/scene3d_sky.frag. Cada campo e um vec4 de proposito, para
// este struct em C++ bater byte a byte com o std430 do GLSL sem nenhum
// preenchimento surpresa (o mesmo risco de descompasso silencioso entre CPU
// e GPU que motivou extrair o SceneUniform compartilhado na Fase 1).
struct alignas(16) GpuLightData3D {
    std::array<float, 4> positionRange {};
    std::array<float, 4> directionOuterCosine {};
    std::array<float, 4> colorIntensity {};
    std::array<float, 4> parameters {};
};

static_assert(sizeof(GpuLightData3D) == 64);

// Converte a lista de luzes backend-neutra (Scene3DFrame::lights) para o
// layout que a GPU consome - resolve o cosseno do cone a partir do angulo em
// graus, clampa intensidade/alcance/angulo para valores validos e aplica o
// comportamento default por tipo (ex.: uma luz Directional nunca usa alcance
// nem cone, ainda que o chamador tenha deixado esses campos com o valor
// default da struct). Funcao pura, sem nenhuma dependencia de Vulkan, para
// poder ser testada diretamente (ver EngineFoundationTests.cpp).
std::vector<GpuLightData3D> packSceneLights(std::span<const LightRender3D> lights);

} // namespace MatterEngine
